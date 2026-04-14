# ============================================================
# Hand-on 2: Joint & Motion Control
# Franka Panda picks up 3 colored cubes at random positions
# and stacks them into a pyramid.
# Run via: python3 run_in_isaac.py hand_on_2_franka.py
# ============================================================
import numpy as np
import omni.kit.app

from isaacsim.examples.interactive.base_sample import BaseSample
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.robot.manipulators.examples.franka import Franka
from isaacsim.robot.manipulators.examples.franka.controllers import PickPlaceController

# ── Config ────────────────────────────────────────────────────
CUBE_SIZE = 0.0515  # 5.15 cm cube (matches Isaac Sim default)
CUBE_HALF = CUBE_SIZE / 2.0

# Cube colors
CUBE_COLORS = [
    np.array([1.0, 0.0, 0.0]),  # red
    np.array([0.0, 1.0, 0.0]),  # green
    np.array([0.0, 0.0, 1.0]),  # blue
]

# Random spawn zone: cubes are placed randomly within this annular region
# so they're reachable (~0.8m max) but not too close to the arm base (~0.25m min).
# Cubes only spawn in front of the robot (positive x) to avoid arm self-collision.
SPAWN_RADIUS_MIN = 0.25
SPAWN_RADIUS_MAX = 0.5
MIN_CUBE_SEPARATION = 0.15  # minimum distance between cubes to avoid overlap
MIN_PYRAMID_CLEARANCE = 0.12  # minimum XY distance from any pyramid target slot

# Drop height: gripper releases the cube this far ABOVE the target position,
# letting it fall into place. Avoids gripper fingers bumping placed cubes.
DROP_HEIGHT = 0.03  # 10 cm above target

# Pyramid layout — only the FIRST cube has a fixed target.
# Cubes 2 and 3 are computed dynamically from where previous cubes actually land.
#   [cube2]          <- top-center, computed from cube0 & cube1
#  [cube0] [cube1]   <- bottom row
PYRAMID_X = 0.3                                          # how far in front of robot
FIRST_PLACE_POS = np.array([PYRAMID_X, 0.0, CUBE_HALF])  # bottom-left seed position

# ── Super-robust pick config ─────────────────────────────────
# The PickPlaceController is TIME-based: phases advance by timer, not by
# checking if the end-effector actually reached the target. If approach or
# descend completes before the arm arrives, the gripper closes on air.
#
# Robust mode: slower phase timing, pick verification, auto-retry.
ROBUST = True
MAX_PICK_RETRIES = 3             # retries per cube before skipping
PICK_Z_OFFSET = 0           # 1 cm above cube center — ground clearance for fingers
PICK_SUCCESS_XY_TOL = 0.10       # cube within 10 cm XY of target = placed OK
SETTLE_TICKS = 120               # physics ticks to idle between pick cycles (~2 s)

# Robust events_dt: halve approach/descend/lower speeds so arm has time to reach.
#   Phase:    0        1       2     3     4      5      6       7     8      9
#   Action: approach descend pause close  lift  transit lower  open  retract pause
EVENTS_DT_ROBUST  = [0.004, 0.0025, 1, 0.1, 0.1,  0.1,  0.0025, 1, 0.016, 0.15]
EVENTS_DT_DEFAULT = [0.008, 0.005, 1, 0.1, 0.1,  0.1,  0.0025, 1, 0.016, 0.15]


def _random_cube_positions(n=3):
    """Generate n random positions within the Franka's reach, well-separated.

    Spawns cubes in front of the robot (x > 0) within an annular zone so the arm
    can reach them without self-collision. Enforces minimum separation between cubes.
    """
    positions = []
    for _ in range(n):
        for _attempt in range(100):
            angle = np.random.uniform(-np.pi / 3, np.pi / 3)  # front 120 degrees
            radius = np.random.uniform(SPAWN_RADIUS_MIN, SPAWN_RADIUS_MAX)
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            pos = np.array([x, y, CUBE_HALF])

            # Check separation from all previously placed cubes
            too_close_to_cube = any(
                np.linalg.norm(pos[:2] - p[:2]) < MIN_CUBE_SEPARATION for p in positions)
            # Check spawn doesn't land on the pyramid zone (approximate:
            # the whole area around PYRAMID_X is reserved for stacking)
            too_close_to_pyramid = (
                np.linalg.norm(pos[:2] - FIRST_PLACE_POS[:2]) < MIN_PYRAMID_CLEARANCE)
            if not too_close_to_cube and not too_close_to_pyramid:
                positions.append(pos)
                break
        else:
            # Fallback: place at a safe default if random sampling fails
            positions.append(np.array([0.3 + 0.15 * len(positions), 0.0, CUBE_HALF]))
    return positions


# ── BaseSample lifecycle ──────────────────────────────────────
class FrankaPyramid(BaseSample):
    def __init__(self):
        super().__init__()
        self._retries = 0
        self._settling = False
        self._settle_counter = 0
        return

    def setup_scene(self):
        """Step 1: Create three cubes with different colors at random positions.

        Uses DynamicCuboid (rigid-body with physics) so they can be picked up.
        Positions are randomized within ~0.8m of the robot base (per spec p.31).
        """
        world = self.get_world()
        world.scene.add_default_ground_plane()

        # Franka Panda 7-DOF arm with parallel gripper
        world.scene.add(Franka(prim_path="/World/Fancy_Franka", name="fancy_franka"))

        # Spawn cubes at random positions within reach
        self._cube_names = []
        random_positions = _random_cube_positions(len(CUBE_COLORS))
        for i, (color, pos) in enumerate(zip(CUBE_COLORS, random_positions)):
            name = f"cube_{i}"
            self._cube_names.append(name)
            world.scene.add(
                DynamicCuboid(
                    prim_path=f"/World/{name}",
                    name=name,
                    position=pos,
                    scale=np.array([CUBE_SIZE, CUBE_SIZE, CUBE_SIZE]),
                    color=color,
                )
            )
            print(f"  Spawned {name} at ({pos[0]:.2f}, {pos[1]:.2f})")
        return

    async def setup_post_load(self):
        """Steps 2-3: Get references to each cube and initialize the controller.

        Defines the pick & place sequence: pick cubes in order, place at
        DROP_POSITIONS (elevated above PYRAMID_POSITIONS so cubes fall into place).
        """
        self._world = self.get_world()
        self._franka = self._world.scene.get_object("fancy_franka")
        self._cubes = [self._world.scene.get_object(name) for name in self._cube_names]

        # PickPlaceController: internal state machine handling
        # approach -> pick -> lift -> move -> place -> release
        #
        # events_dt: per-tick increment (larger = faster, phase ends at 1.0)
        # Event:      0        1       2      3      4      5       6      7      8      9
        # Action:  approach  descend  pause  close  lift  transit  lower  open  retract  pause
        # Default: [0.008,   0.005,   1,     0.1,   0.05,  0.05, 0.0025, 1,    0.008,  0.08]
        #
        # MUST keep 0 (approach) and 1 (descend) slow — arm needs time to reach the cube.
        # MUST keep 6 (lower to place) slow — arm needs time to position above target.
        # Safe to speed up: 4 (lift), 5 (transit), 8 (retract), 9 (pause).
        edt = EVENTS_DT_ROBUST if ROBUST else EVENTS_DT_DEFAULT
        self._controller = PickPlaceController(
            name="pick_place_controller",
            gripper=self._franka.gripper,
            robot_articulation=self._franka,
            events_dt=edt,
        )
        self._current_cube_index = 0
        self._done = False
        self._placed_positions = []  # actual landed positions of placed cubes

        # Start with gripper open, ready to pick
        self._franka.gripper.set_joint_positions(self._franka.gripper.joint_opened_positions)

        # Register physics_step callback — called automatically every tick
        self._world.add_physics_callback("sim_step", callback_fn=self.physics_step)
        await self._world.play_async()
        return

    def _compute_place_target(self):
        """Compute the next pyramid target from where previous cubes actually landed.

        Cube 0: fixed seed position (FIRST_PLACE_POS).
        Cube 1: same Z, beside cube 0 — offset by +CUBE_SIZE in Y.
        Cube 2: centered on top of cubes 0 & 1, one layer up.
        """
        idx = self._current_cube_index
        if idx == 0:
            return FIRST_PLACE_POS.copy()
        elif idx == 1:
            p0 = self._placed_positions[0]
            return np.array([p0[0], p0[1] + CUBE_SIZE + 0.02, CUBE_SIZE])
        else:  # idx == 2
            p0 = self._placed_positions[0]
            p1 = self._placed_positions[1]
            mid_x = (p0[0] + p1[0]) / 2.0
            mid_y = (p0[1] + p1[1]) / 2.0
            return np.array([mid_x, mid_y, CUBE_SIZE + CUBE_HALF])

    def _advance_to_next_cube(self):
        """Reset controller and gripper, enter settling phase before next pick."""
        self._controller.reset()
        self._franka.gripper.set_joint_positions(
            self._franka.gripper.joint_opened_positions)
        if ROBUST:
            self._settling = True
            self._settle_counter = SETTLE_TICKS

    def physics_step(self, step_size):
        """Step 4: Control loop — pick each cube and stack into pyramid.

        Strategy: the gripper releases each cube at DROP_HEIGHT above the actual
        pyramid target, letting it fall into place via gravity. This prevents
        the gripper fingers from contacting already-placed cubes.

        Robust mode adds:
        - Pick Z offset so gripper fingers clear the ground
        - Post-cycle verification: checks cube XY vs target
        - Auto-retry up to MAX_PICK_RETRIES per cube
        - Settling pause between cycles for arm to stabilize
        """
        if self._done:
            return

        # Settling phase: idle between pick-place cycles so arm + cubes stabilize
        if self._settling:
            self._settle_counter -= 1
            if self._settle_counter <= 0:
                self._settling = False
            return

        cube = self._cubes[self._current_cube_index]
        target_pos = self._compute_place_target()
        drop_pos = target_pos + np.array([0, 0, DROP_HEIGHT])

        cube_position, _ = cube.get_world_pose()
        current_joint_positions = self._franka.get_joint_positions()

        # Offset picking height so gripper fingers don't collide with ground
        pick_pos = cube_position.copy()
        if ROBUST:
            pick_pos[2] += PICK_Z_OFFSET

        # controller.forward() returns joint actions for the current phase
        # of the pick-and-place sequence
        actions = self._controller.forward(
            picking_position=pick_pos,
            placing_position=drop_pos,
            current_joint_positions=current_joint_positions,
        )
        self._franka.apply_action(actions)

        # is_done() = state machine reached final state (cube released)
        if self._controller.is_done():
            final_pos, _ = cube.get_world_pose()
            dist_xy = np.linalg.norm(final_pos[:2] - target_pos[:2])

            if ROBUST:
                if dist_xy < PICK_SUCCESS_XY_TOL:
                    print(f"  OK cube_{self._current_cube_index} placed "
                          f"(err={dist_xy:.3f} m)")
                    self._placed_positions.append(final_pos.copy())
                    self._retries = 0
                    self._current_cube_index += 1
                elif self._retries < MAX_PICK_RETRIES:
                    self._retries += 1
                    print(f"  MISS cube_{self._current_cube_index} "
                          f"(err={dist_xy:.3f} m, attempt "
                          f"{self._retries}/{MAX_PICK_RETRIES})")
                    self._advance_to_next_cube()
                    return
                else:
                    print(f"  FAIL cube_{self._current_cube_index} after "
                          f"{MAX_PICK_RETRIES} retries — skipping")
                    self._placed_positions.append(final_pos.copy())
                    self._retries = 0
                    self._current_cube_index += 1
            else:
                print(f"Dropped cube_{self._current_cube_index}!")
                self._placed_positions.append(final_pos.copy())
                self._current_cube_index += 1

            if self._current_cube_index >= len(self._cubes):
                self._done = True
                self._world.pause()
            else:
                self._advance_to_next_cube()
        return

    async def setup_post_reset(self):
        """Step 5: Reset all custom state variables for RESET button."""
        self._controller.reset()
        self._current_cube_index = 0
        self._done = False
        self._retries = 0
        self._settling = False
        self._settle_counter = 0
        self._placed_positions = []
        self._franka.gripper.set_joint_positions(self._franka.gripper.joint_opened_positions)
        await self._world.play_async()
        return


# ── Run ───────────────────────────────────────────────────────
sample = FrankaPyramid()
await sample.load_world_async()
print(f"Stacking {len(CUBE_COLORS)} randomly-placed cubes into a pyramid")

# physics_step is called automatically each tick via the registered callback.
# We just wait here until all cubes are placed.
# Robust mode needs more ticks: slower phases + retries + settling pauses.
max_ticks = 80000 if ROBUST else 20000
for _ in range(max_ticks):
    await omni.kit.app.get_app().next_update_async()
    if sample._done:
        break

print("Pyramid stacking complete!")
