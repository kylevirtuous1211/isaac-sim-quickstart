# ============================================================
# Hand-on 3: RMPflow Obstacle Avoidance
# Franka Panda picks up 3 colored cubes on the RIGHT side of
# a vertical wall and places them on the LEFT side as a
# pyramid, using RMPflow to navigate over/around the wall.
# Run via: python3 run_in_isaac.py hand_on_3_rmpflow.py
# ============================================================
import os
import numpy as np
import omni.kit.app
import isaacsim.robot_motion.motion_generation as mg

from isaacsim.examples.interactive.base_sample import BaseSample
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.robot.manipulators.examples.franka import Franka
from isaacsim.robot.manipulators.examples.franka.controllers import RMPFlowController

# ── Config ────────────────────────────────────────────────────
CUBE_SIZE = 0.0515  # 5.15 cm cube (matches Isaac Sim default)
CUBE_HALF = CUBE_SIZE / 2.0

CUBE_COLORS = [
    np.array([1.0, 0.0, 0.0]),  # red
    np.array([0.0, 1.0, 0.0]),  # green
    np.array([0.0, 0.0, 1.0]),  # blue
]

# ── Wall (vertical board) ────────────────────────────────────
# Placed in front of robot at y=0, extends along x-axis.
# The wall divides the workspace into LEFT (y > 0) and RIGHT (y < 0).
WALL_X = 0.4                     # how far in front of robot
WALL_HEIGHT = 0.15               # 15 cm tall — arm must go OVER this
WALL_THICKNESS = 0.015           # 1.5 cm thin
WALL_WIDTH = 0.30                # 30 cm wide along x-axis
WALL_COLOR = np.array([0.85, 0.65, 0.0])  # gold/amber

# ── Cube pick positions (RIGHT side, y < 0) ──────────────────
# First cube (red) is at a known straight-ahead position for reliable first grab.
PICK_POSITIONS = [
    np.array([0.60, -0.20, CUBE_HALF]),   # red — straight ahead
    np.array([0.45, -0.25, CUBE_HALF]),   # green
    np.array([0.30, -0.25, CUBE_HALF]),   # blue
]

# ── Pyramid place targets (LEFT side, y > 0) ─────────────────
# Only the FIRST cube has a fixed target. Cubes 2 and 3 are
# computed dynamically from where previous cubes actually land.
#   [cube2]          <- top-center, computed from cube0 & cube1
#  [cube0] [cube1]   <- bottom row
FIRST_PLACE_POS = np.array([0.35, 0.20, CUBE_HALF])

# Drop height: gripper releases the cube this far ABOVE the target,
# letting it fall into place (same pattern as hand_on_2_franka.py).
# Avoids gripper fingers bumping already-placed cubes.
DROP_HEIGHT = 0.05  # 5 cm above target

# ── RMPflow config ────────────────────────────────────────────
# Local copies of Franka config files (mounted at /workspace/configs/).
# Edit franka_rmpflow_common.yaml to tune obstacle avoidance params:
#   collision_rmp.repulsion_gain   (5000) — repulsive force strength
#   collision_rmp.metric_scalar    (50000) — overall avoidance weight
CONFIG_DIR = "/workspace/configs/rmpflow"

# ── Motion parameters ────────────────────────────────────────
CLEARANCE_HEIGHT = WALL_HEIGHT + 0.15  # 30 cm — clearly above the wall
CONVERGE_TOL = 0.06              # 6 cm convergence threshold
MAX_PHASE_TICKS = 400            # ~7 s safety timeout
GRIPPER_TICKS = 15               # brief pause for gripper (~0.25s)

# ── Gripper orientation for top-down grasp ────────────────────
# 180° pitch = gripper Z-axis pointing down in world frame.
# Only enforced during pick phases; left free during wall crossing
# so RMPflow can orient the arm however needed to clear the wall.
GRIPPER_DOWN_QUAT = euler_angles_to_quat(np.array([0, np.pi, 0]))

# ── Approach velocity filter ─────────────────────────────────
# When EE enters SLOW_RADIUS of the target, the effective RMPflow target
# is damped toward the current EE position, producing smooth deceleration.
# speed ramps linearly: 1.0 at SLOW_RADIUS → MIN_APPROACH_SPEED at target.
SLOW_RADIUS = 0.18              # start slowing at 18 cm from target
MIN_APPROACH_SPEED = 0.25       # at target, move at 25% of full speed


# ── State machine phases ─────────────────────────────────────
# Symmetric pick & place pattern:
#   PICK:  MOVE_ABOVE_PICK → LOWER_TO_PICK → CLOSE_GRIPPER → LIFT_UP
#   PLACE: MOVE_OVER_WALL  → LOWER_TO_DROP → OPEN_GRIPPER  → RETREAT_UP
class Phase:
    MOVE_ABOVE_PICK = 0  # Go above cube at CLEARANCE_HEIGHT
    LOWER_TO_PICK   = 1  # Descend straight down to cube
    CLOSE_GRIPPER   = 2  # Brief pause to close gripper
    LIFT_UP         = 3  # Lift straight up (prevents dragging)
    MOVE_OVER_WALL  = 4  # Move laterally to above place pos (clears wall)
    LOWER_TO_DROP   = 5  # Descend to DROP_HEIGHT above place target
    OPEN_GRIPPER    = 6  # Release cube (falls into place)
    RETREAT_UP      = 7  # Lift straight up (won't hit placed cubes)

PHASE_NAMES = [
    "MOVE_ABOVE_PICK", "LOWER_TO_PICK", "CLOSE_GRIPPER", "LIFT_UP",
    "MOVE_OVER_WALL", "LOWER_TO_DROP", "OPEN_GRIPPER", "RETREAT_UP",
]


# ── BaseSample lifecycle ──────────────────────────────────────
class FrankaRMPflow(BaseSample):
    def __init__(self):
        super().__init__()
        self._phase = Phase.MOVE_ABOVE_PICK
        self._current_cube_index = 0
        self._phase_ticks = 0
        self._done = False
        self._placed_positions = []
        self._rmp_flow = None
        self._lift_target = None
        self._retreat_target = None
        return

    def setup_scene(self):
        """Add Franka, wall, and cubes to the scene."""
        world = self.get_world()
        world.scene.add_default_ground_plane()

        # Franka Panda 7-DOF arm with parallel gripper
        world.scene.add(Franka(prim_path="/World/Fancy_Franka", name="fancy_franka"))

        # Vertical wall dividing left and right sides
        world.scene.add(
            FixedCuboid(
                prim_path="/World/wall",
                name="wall",
                position=np.array([WALL_X, 0.0, WALL_HEIGHT / 2.0]),
                scale=np.array([WALL_WIDTH, WALL_THICKNESS, WALL_HEIGHT]),
                color=WALL_COLOR,
            )
        )
        print(f"  Wall at x={WALL_X}, height={WALL_HEIGHT}m")

        # Spawn cubes on the RIGHT side (y < 0)
        self._cube_names = []
        for i, (color, pos) in enumerate(zip(CUBE_COLORS, PICK_POSITIONS)):
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
            print(f"  Spawned {name} at ({pos[0]:.2f}, {pos[1]:.2f}) [RIGHT]")
        return

    async def setup_post_load(self):
        """Initialize RMPflow controller from local config files and register the wall."""
        self._world = self.get_world()
        self._franka = self._world.scene.get_object("fancy_franka")
        self._wall = self._world.scene.get_object("wall")
        self._cubes = [self._world.scene.get_object(name) for name in self._cube_names]

        self._init_rmpflow_controller()

        # Start with gripper open
        self._franka.gripper.set_joint_positions(self._franka.gripper.joint_opened_positions)

        # Register physics_step callback — called automatically every tick
        self._world.add_physics_callback("sim_step", callback_fn=self.physics_step)
        await self._world.play_async()
        return

    def _init_rmpflow_controller(self):
        """Create RMPflow motion policy controller.

        Tries local config files first (/workspace/configs/rmpflow/) so the
        user can edit franka_rmpflow_common.yaml to tune obstacle avoidance.
        Falls back to the built-in RMPFlowController if local configs are
        not found (e.g., Docker container not restarted after adding mount).
        """
        config_yaml = f"{CONFIG_DIR}/franka_rmpflow_common.yaml"
        if os.path.exists(config_yaml):
            print(f"  Loading RMPflow from local configs: {CONFIG_DIR}")
            rmpflow_config = {
                "end_effector_frame_name": "right_gripper",
                "maximum_substep_size": 0.00334,
                "ignore_robot_state_updates": False,
                "robot_description_path": f"{CONFIG_DIR}/robot_descriptor.yaml",
                "urdf_path": f"{CONFIG_DIR}/lula_franka_gen.urdf",
                "rmpflow_config_path": config_yaml,
            }
            self._rmp_flow = mg.lula.motion_policies.RmpFlow(**rmpflow_config)
            articulation_rmp = mg.ArticulationMotionPolicy(
                self._franka, self._rmp_flow, 1.0 / 60.0
            )
            self._controller = mg.MotionPolicyController(
                name="rmpflow_controller",
                articulation_motion_policy=articulation_rmp,
            )
            pos, ori = self._franka.get_world_pose()
            self._rmp_flow.set_robot_base_pose(
                robot_position=pos, robot_orientation=ori
            )
        else:
            print(f"  WARNING: Local configs not found at {CONFIG_DIR}")
            print("  Falling back to built-in RMPFlowController")
            self._controller = RMPFlowController(
                name="rmpflow_controller",
                robot_articulation=self._franka,
            )
            self._rmp_flow = self._controller.rmp_flow

        # Register the wall as a static obstacle
        self._controller.add_obstacle(self._wall, static=True)
        print("  RMPflow controller initialized with wall obstacle")

    def _compute_place_target(self):
        """Compute the next pyramid target from where previous cubes actually landed."""
        idx = self._current_cube_index
        if idx == 0:
            return FIRST_PLACE_POS.copy()
        elif idx == 1:
            p0 = self._placed_positions[0]
            return np.array([p0[0], p0[1] + (CUBE_SIZE + 0.02), CUBE_HALF])
        else:  # idx == 2
            p0 = self._placed_positions[0]
            p1 = self._placed_positions[1]
            mid_x = (p0[0] + p1[0]) / 2.0
            mid_y = (p0[1] + p1[1]) / 2.0
            return np.array([mid_x, mid_y, CUBE_SIZE + CUBE_HALF])

    def _ee_pos(self):
        """Get current end-effector world position."""
        pos, _ = self._franka.end_effector.get_world_pose()
        return pos

    def _move_to(self, target_pos, slow=False, orient=None):
        """Command RMPflow to move end-effector toward target position.

        Args:
            target_pos: 3D target position.
            slow: If True, decelerate smoothly near the target.
            orient: Quaternion (w,x,y,z) for end-effector orientation.
                    None = RMPflow chooses freely (good for wall crossing).
                    GRIPPER_DOWN_QUAT = straight-down grasp (good for picking).
        """
        if slow:
            ee = self._ee_pos()
            dist = np.linalg.norm(ee - target_pos)
            if dist < SLOW_RADIUS and dist > 1e-4:
                speed = MIN_APPROACH_SPEED + (1.0 - MIN_APPROACH_SPEED) * (dist / SLOW_RADIUS)
                target_pos = ee + speed * (target_pos - ee)

        actions = self._controller.forward(
            target_end_effector_position=target_pos,
            target_end_effector_orientation=orient,
        )
        self._franka.apply_action(actions)

    def _dist_to(self, target):
        """Distance from end-effector to target."""
        return np.linalg.norm(self._ee_pos() - target)

    def _switch_phase(self, new_phase):
        """Transition to a new phase, reset tick counter."""
        print(f"  cube_{self._current_cube_index}: "
              f"{PHASE_NAMES[self._phase]} -> {PHASE_NAMES[new_phase]}")
        self._phase = new_phase
        self._phase_ticks = 0

    def physics_step(self, step_size):
        """Per-tick control loop — 6-phase motion.

        MOVE_TO_PICK → CLOSE_GRIPPER → MOVE_OVER_WALL → LOWER_TO_DROP
                                                          → OPEN_GRIPPER → RETREAT_UP
        Drop pattern from hand_on_2: release cube above target so it falls
        into place, then retreat vertically to avoid bumping placed cubes.
        """
        if self._done:
            return

        self._phase_ticks += 1
        timed_out = self._phase_ticks > MAX_PHASE_TICKS
        if timed_out:
            print(f"  WARNING: {PHASE_NAMES[self._phase]} timed out "
                  f"for cube_{self._current_cube_index}")

        cube = self._cubes[self._current_cube_index]

        if self._phase == Phase.MOVE_ABOVE_PICK:
            # Go above cube at clearance height (fast, no slow filter)
            cube_pos, _ = cube.get_world_pose()
            above_target = np.array([cube_pos[0], cube_pos[1], CLEARANCE_HEIGHT])
            self._move_to(above_target)
            if self._dist_to(above_target) < CONVERGE_TOL or timed_out:
                self._switch_phase(Phase.LOWER_TO_PICK)

        elif self._phase == Phase.LOWER_TO_PICK:
            # Descend straight down to cube — gripper facing down
            cube_pos, _ = cube.get_world_pose()
            self._move_to(cube_pos, orient=GRIPPER_DOWN_QUAT)
            if self._dist_to(cube_pos) < CONVERGE_TOL or timed_out:
                self._switch_phase(Phase.CLOSE_GRIPPER)

        elif self._phase == Phase.CLOSE_GRIPPER:
            # Hold position while grasping
            cube_pos, _ = cube.get_world_pose()
            self._move_to(cube_pos, orient=GRIPPER_DOWN_QUAT)
            self._franka.gripper.close()
            if self._phase_ticks >= GRIPPER_TICKS:
                ee = self._ee_pos()
                self._lift_target = np.array([ee[0], ee[1], CLEARANCE_HEIGHT])
                self._switch_phase(Phase.LIFT_UP)

        elif self._phase == Phase.LIFT_UP:
            # Lift straight up — prevents dragging cube on floor
            self._move_to(self._lift_target)
            self._franka.gripper.close()
            if self._dist_to(self._lift_target) < CONVERGE_TOL or timed_out:
                self._switch_phase(Phase.MOVE_OVER_WALL)

        elif self._phase == Phase.MOVE_OVER_WALL:
            # Move laterally (already at height) to above the place position
            place_pos = self._compute_place_target()
            over_target = np.array([place_pos[0], place_pos[1], CLEARANCE_HEIGHT])
            self._move_to(over_target)
            self._franka.gripper.close()
            if self._dist_to(over_target) < CONVERGE_TOL or timed_out:
                self._switch_phase(Phase.LOWER_TO_DROP)

        elif self._phase == Phase.LOWER_TO_DROP:
            # Descend with gripper down for precise drop
            place_pos = self._compute_place_target()
            drop_pos = place_pos + np.array([0, 0, DROP_HEIGHT])
            self._move_to(drop_pos, orient=GRIPPER_DOWN_QUAT)
            self._franka.gripper.close()
            if self._dist_to(drop_pos) < CONVERGE_TOL or timed_out:
                self._switch_phase(Phase.OPEN_GRIPPER)

        elif self._phase == Phase.OPEN_GRIPPER:
            # Release cube — it falls DROP_HEIGHT onto the target
            place_pos = self._compute_place_target()
            drop_pos = place_pos + np.array([0, 0, DROP_HEIGHT])
            self._move_to(drop_pos, orient=GRIPPER_DOWN_QUAT)
            self._franka.gripper.open()
            if self._phase_ticks >= GRIPPER_TICKS:
                actual_pos, _ = cube.get_world_pose()
                self._placed_positions.append(actual_pos.copy())
                print(f"  Placed cube_{self._current_cube_index} at "
                      f"({actual_pos[0]:.2f}, {actual_pos[1]:.2f}, "
                      f"{actual_pos[2]:.2f}) [LEFT]")
                # Capture retreat target: straight up from current EE position
                ee = self._ee_pos()
                self._retreat_target = np.array([ee[0], ee[1], CLEARANCE_HEIGHT])
                self._switch_phase(Phase.RETREAT_UP)

        elif self._phase == Phase.RETREAT_UP:
            # Go straight up so gripper won't hit placed cubes
            self._move_to(self._retreat_target)
            if self._dist_to(self._retreat_target) < CONVERGE_TOL or timed_out:
                self._current_cube_index += 1
                if self._current_cube_index >= len(self._cubes):
                    self._done = True
                    self._world.pause()
                    print("All cubes placed! Pyramid complete on LEFT side.")
                else:
                    self._franka.gripper.open()
                    self._switch_phase(Phase.MOVE_ABOVE_PICK)

        return

    async def setup_post_reset(self):
        """Reset all state for RESET button."""
        self._init_rmpflow_controller()
        self._phase = Phase.MOVE_ABOVE_PICK
        self._current_cube_index = 0
        self._phase_ticks = 0
        self._done = False
        self._placed_positions = []
        self._lift_target = None
        self._retreat_target = None
        self._franka.gripper.set_joint_positions(self._franka.gripper.joint_opened_positions)
        await self._world.play_async()
        return


# ── Run ───────────────────────────────────────────────────────
sample = FrankaRMPflow()
await sample.load_world_async()
print(f"Moving {len(CUBE_COLORS)} cubes over wall using RMPflow obstacle avoidance")

max_ticks = 60000
for _ in range(max_ticks):
    await omni.kit.app.get_app().next_update_async()
    if sample._done:
        break

print("RMPflow obstacle avoidance demo complete!")
