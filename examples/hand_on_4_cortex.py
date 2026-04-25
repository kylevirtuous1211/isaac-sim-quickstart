# ============================================================
# Hand-on 4: Isaac Cortex Decider Network — Reactive Stacking
# Franka stacks 4 colored cubes into a tower using the built-in
# block_stacking_behavior decider network. The network has three
# top-level states (Dispatch -> {Pick | Place | Go home}) and
# reacts automatically when a cube is disturbed (dropped, moved,
# or misplaced) — it rebuilds the tower from scratch.
#
# Built on the Isaac Sim "Franka Cortex Examples" sample:
#   exts/isaacsim.examples.interactive/.../franka_cortex/franka_cortex.py
#
# Decider network (see Isaac_Cortex_Hand_on.pdf):
#   Dispatch (reads gripper state)
#     ├── Go home     — task complete
#     ├── Pick RLDS   — gripper empty
#     │     ├── open_gripper  (priority: always open if not)
#     │     ├── pick_block    (priority: state-machine grasp)
#     │     └── reach_to_block
#     │            └── ChooseNextBlock
#     │                 ├── ChooseNextBlockForTowerBuildUp  (pyramid good)
#     │                 └── ChooseNextBlockForTowerTeardown (pyramid wrong)
#     └── Place RLDS  — gripper has block
#           ├── place_block    (priority: state-machine place)
#           └── reach_to_placement
#                  ├── ReachToPlaceOnTower (on pyramid)
#                  └── ReachToPlaceOnTable (temp location)
#
# Run via: python3 run_in_isaac.py examples/hand_on_4_cortex.py
# Then in the Isaac Sim viewport: drop or drag a cube to see the
# robot react and recover.
# ============================================================
import traceback
from collections import OrderedDict

import numpy as np
import omni.kit.app

import isaacsim.cortex.framework.math_util as math_util
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.cortex.behaviors.franka import block_stacking_behavior
from isaacsim.cortex.behaviors.franka.block_stacking_behavior import (
    BlockPickAndPlaceDispatch,
    BuildTowerContext,
    make_block_grasp_Ts,
)
from isaacsim.cortex.framework.cortex_object import CortexObject
from isaacsim.cortex.framework.cortex_world import CortexWorld
from isaacsim.cortex.framework.df import DfNetwork
from isaacsim.cortex.framework.dfb import DfRobotApiContext
from isaacsim.cortex.framework.robot import add_franka_to_stage
from isaacsim.examples.interactive.cortex.cortex_base import CortexBase

# CortexBase.load_world_async only runs setup_scene when CortexWorld.instance()
# is None — otherwise it reuses the existing CortexWorld and the Franka/cubes
# never get added, leaving self.robot = None. Always wipe the singleton so
# every run is a fresh load.
if World.instance() is not None:
    print("[hand_on_4] Clearing stale World/CortexWorld singleton from previous run")
    World.clear_instance()


# ── Scene config ─────────────────────────────────────────────
# Pyramid layout (viewed from the robot, +y is away):
#     [Green]         <- slot 2 (top, centered)
#   [Blue][Yellow]    <- slot 0 (left),  slot 1 (right)
# PyramidTower.desired_stack indexes slots 0→1→2 in this pick order.
CUBE_SIZE = 0.0515
CUBE_SPECS = [
    ("BlueCube",   np.array([0.0, 0.0, 0.7])),
    ("YellowCube", np.array([0.7, 0.7, 0.0])),
    ("GreenCube",  np.array([0.0, 0.7, 0.0])),
]
# Spawn row on the NEAR side of the robot (y=-0.4).
# Pyramid target center is at (0.4, 0.3, 0.0) — see make_pyramid_decider_network.
SPAWN_Y = -0.4
SPAWN_X_MIN, SPAWN_X_MAX = 0.3, 0.6
PYRAMID_CENTER = np.array([0.4, 0.3, 0.0])


# ── Pyramid decider network ─────────────────────────────────
# The built-in block_stacking_behavior builds a vertical TOWER. We swap in a
# PyramidTower that defines 3 slots (bottom-left, bottom-right, top-center) and
# overrides placement + monitoring so the same Dispatch / Pick / Place RLDS
# machinery from block_stacking_behavior drives a pyramid build instead.
# Everything else (ChooseNextBlock tear-down, gripper suppression, recovery
# on disturbance) continues to work unchanged.
class PyramidTower:
    """3-slot pyramid replacement for BuildTowerContext.BlockTower."""

    def __init__(self, center, block_height, context):
        self.context = context
        self.tower_position = center     # BuildTowerContext uses this name
        self.block_height = block_height
        self.desired_stack = ["BlueCube", "YellowCube", "GreenCube"]
        self.slots = [None, None, None]
        self.prev_slots = [None, None, None]

    # Slot centers in world coords. The bottom pair is laid along the WORLD-Y
    # axis (not X) so the gripper fingers — which open in ±x under the
    # ReachToPlaceOnTower grasp (desired_ax = [0,-1,0]) — don't collide with
    # the already-placed Blue block when placing Yellow beside it.
    # dy chosen so edge-to-edge gap is ~1.5 cm (safe for fingers) while the
    # top block still overlaps each bottom block by ~1.8 cm (stable).
    def slot_position(self, i):
        dy = self.block_height * 0.65
        dz_bot = 0.5 * self.block_height
        dz_top = 1.5 * self.block_height
        offsets = [
            np.array([0.0, -dy, dz_bot]),  # slot 0 — Blue   (bottom, -y)
            np.array([0.0, +dy, dz_bot]),  # slot 1 — Yellow (bottom, +y)
            np.array([0.0,  0.0, dz_top]), # slot 2 — Green  (top, centered)
        ]
        return self.tower_position + offsets[i]

    @property
    def stack(self):
        return [b for b in self.slots if b is not None]

    @property
    def height(self):
        return len(self.stack)

    @property
    def top_block(self):
        for b in reversed(self.slots):
            if b is not None:
                return b
        return None

    @property
    def next_slot(self):
        for i, b in enumerate(self.slots):
            if b is None:
                return i
        return None

    @property
    def next_block(self):
        i = self.next_slot
        if i is None:
            return None
        return self.context.blocks[self.desired_stack[i]]

    @property
    def next_block_placement_T(self):
        # Target EE transform for the NEXT slot to fill. When the pyramid is
        # already full, fall back to the top slot so exit paths don't crash.
        i = self.next_slot if self.next_slot is not None else len(self.slots) - 1
        return math_util.pack_Rp(np.eye(3), self.slot_position(i))

    @property
    def current_stack_in_correct_order(self):
        for i, b in enumerate(self.slots):
            if b is not None and b.name != self.desired_stack[i]:
                return False
        return True

    @property
    def is_complete(self):
        return (
            all(b is not None for b in self.slots)
            and self.current_stack_in_correct_order
        )

    def stash_stack(self):
        self.prev_slots = list(self.slots)
        self.slots = [None, None, None]

    def find_new_and_removed(self):
        new = [b for i, b in enumerate(self.slots)
               if b is not None and b != self.prev_slots[i]]
        removed = [b for i, b in enumerate(self.prev_slots)
                   if b is not None and b != self.slots[i]]
        return new, removed

    def set_top_block_to_aligned(self):
        top = self.top_block
        if top is not None:
            top.is_aligned = True


class PyramidContext(BuildTowerContext):
    """BuildTowerContext that stacks into a 3-slot pyramid."""

    def __init__(self, robot, pyramid_center):
        DfRobotApiContext.__init__(self, robot)
        self.robot = robot
        self.block_height = 0.0515
        self.block_pick_height = 0.02
        self.block_grasp_Ts = make_block_grasp_Ts(self.block_pick_height)
        self.tower_position = pyramid_center
        self.diagnostics_message = ""
        self.reset()
        self.add_monitors([
            BuildTowerContext.monitor_perception,
            PyramidContext._monitor_pyramid,
            BuildTowerContext.monitor_gripper_has_block,
            BuildTowerContext.monitor_suppression_requirements,
            BuildTowerContext.monitor_diagnostics,
        ])

    def reset(self):
        self.blocks = OrderedDict()
        for i, (name, cortex_obj) in enumerate(self.robot.registered_obstacles.items()):
            if not isinstance(cortex_obj, CortexObject):
                cortex_obj = CortexObject(cortex_obj)
            cortex_obj.sync_throttle_dt = 0.25
            self.blocks[name] = BuildTowerContext.Block(
                i, cortex_obj, self.block_grasp_Ts
            )
        self.block_tower = PyramidTower(self.tower_position, self.block_height, self)
        self.active_block = None
        self.in_gripper = None
        self.placement_target_eff_T = None
        self.print_dt = 0.25
        self.next_print_time = None
        self.start_time = None

    # Which blocks aren't in a pyramid slot yet.
    def find_not_in_tower(self):
        in_pyramid = {b.name for b in self.block_tower.stack}
        return [b for name, b in self.blocks.items() if name not in in_pyramid]

    @property
    def next_block_name(self):
        remaining = {b.name for b in self.find_not_in_tower()}
        if not remaining:
            return None
        for name in self.block_tower.desired_stack:
            if name in remaining:
                return name
        return None

    def _monitor_pyramid(self):
        """Per-tick: assign each free-standing block to its nearest slot (if any)."""
        slot_radius = self.block_height / 2
        new_slots = [None, None, None]
        for name, block in self.blocks.items():
            if self.gripper_has_block and self.in_gripper.name == name:
                continue
            p, _ = block.obj.get_world_pose()
            for i in range(3):
                sp = self.block_tower.slot_position(i)
                if (np.linalg.norm(p[:2] - sp[:2]) <= slot_radius
                        and abs(p[2] - sp[2]) <= slot_radius * 2):
                    new_slots[i] = block
                    break

        self.block_tower.stash_stack()
        self.block_tower.slots = new_slots
        new_blocks, removed = self.block_tower.find_new_and_removed()
        for b in new_blocks:
            b.is_aligned = False
        for b in removed:
            b.is_aligned = None


def make_pyramid_decider_network(robot, pyramid_center):
    return DfNetwork(
        BlockPickAndPlaceDispatch(),
        context=PyramidContext(robot, pyramid_center=pyramid_center),
    )


class FrankaCortexStacker(CortexBase):
    """CortexBase subclass that loads block_stacking_behavior on a fresh scene."""

    def __init__(self):
        super().__init__()
        self.robot = None
        self.decider_network = None

    def setup_scene(self):
        """Add Franka + 4 colored dynamic cubes, register cubes as obstacles."""
        world = self.get_world()
        world.scene.add_default_ground_plane()

        # Franka via cortex helper — returns a CortexFranka with motion_commander.
        # world.add_robot() registers it so decider network can find it.
        self.robot = world.add_robot(
            add_franka_to_stage(name="franka", prim_path="/World/Franka")
        )

        # Lay the cubes in a row on the -y side of the workspace.
        xs = np.linspace(SPAWN_X_MIN, SPAWN_X_MAX, len(CUBE_SPECS))
        for x, (name, color) in zip(xs, CUBE_SPECS):
            cube = world.scene.add(
                DynamicCuboid(
                    prim_path=f"/World/Obs/{name}",
                    name=name,
                    size=CUBE_SIZE,
                    color=color,
                    position=np.array([x, SPAWN_Y, CUBE_SIZE / 2.0]),
                )
            )
            # Registering as an obstacle (a) exposes the cube to the behavior's
            # BuildTowerContext via robot.registered_obstacles, and (b) makes
            # RMPflow avoid it during motion. The behavior dynamically
            # suppresses avoidance for the active target.
            self.robot.register_obstacle(cube)

    async def setup_post_load(self):
        """Build the decider network and attach it to the CortexWorld."""
        world = self.get_world()

        # If the CortexWorld singleton was reused (setup_scene skipped), fall
        # back to the robot already registered on the world. Matches the
        # reference franka_cortex.py behavior.
        if self.robot is None:
            self.robot = world._robots.get("franka")
        if self.robot is None:
            raise RuntimeError(
                "Franka robot not found on CortexWorld — setup_scene was skipped "
                "(stale singleton). Restart Isaac Sim or rerun this script."
            )

        # Build the pyramid decider network (reuses block_stacking_behavior's
        # Dispatch / Pick RLDS / Place RLDS with our PyramidContext swapped in).
        self.decider_network = make_pyramid_decider_network(self.robot, PYRAMID_CENTER)
        world.add_decider_network(self.decider_network)

        # Physics callback ticks the decider network every sim step.
        # CortexWorld.step(render=False, step_sim=False) advances the cortex
        # logical-state monitors and decider stack without re-stepping physics
        # (physics is already stepped by the World loop).
        world.add_physics_callback("sim_step", self._on_physics_step)
        await world.play_async()

    def _on_physics_step(self, step_size):
        self.get_world().step(False, False)

    async def setup_pre_reset(self):
        world = self.get_world()
        if world.physics_callback_exists("sim_step"):
            world.remove_physics_callback("sim_step")

    async def setup_post_reset(self):
        """Rebuild the decider network after a RESET."""
        world = self.get_world()
        world.reset_cortex()
        self.decider_network = block_stacking_behavior.make_decider_network(self.robot)
        world.add_decider_network(self.decider_network)
        world.add_physics_callback("sim_step", self._on_physics_step)
        await world.play_async()


# ── Run ───────────────────────────────────────────────────────
# Wrapped so any exception during world load or in the run loop is captured
# and re-raised — that way the VSCode executor socket returns the traceback
# to the terminal instead of timing out silently as "(no response)".
try:
    sample = FrankaCortexStacker()
    print("[hand_on_4] loading world...", flush=True)
    await sample.load_world_async()
    print(f"[hand_on_4] world loaded. cubes={len(CUBE_SPECS)} robot={sample.robot}", flush=True)
    print("Isaac Cortex pyramid behavior running.", flush=True)
    print("Target pyramid: Blue (bottom-left), Yellow (bottom-right), Green (top).", flush=True)
    print("Drag/drop a cube in the viewport to see the decider network react.", flush=True)

    # Keep max_ticks BELOW run_in_isaac.py's 10-min socket timeout (~36000 ticks
    # at 60 Hz). If the tower finishes earlier we exit; otherwise the executor
    # returns cleanly and the behavior stops. Re-run the script to continue.
    max_ticks = 30000  # ~8 min at 60 Hz — fits inside the 600 s socket timeout
    for i in range(max_ticks):
        await omni.kit.app.get_app().next_update_async()
        ctx = sample.decider_network.context if sample.decider_network else None
        if ctx is not None and ctx.block_tower.is_complete:
            print(f"[hand_on_4] tower complete at tick {i}", flush=True)
            break

    print("Cortex demo done.", flush=True)
except Exception as e:
    print(f"[hand_on_4] ERROR: {type(e).__name__}: {e}", flush=True)
    traceback.print_exc()
    raise
