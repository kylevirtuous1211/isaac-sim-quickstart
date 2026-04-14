# ============================================================
# Hand-on 1: AMR Movement Control
# JetBot navigates along colored waypoints in a square path.
# Run via: python3 run_in_isaac.py hand_on_1_amr.py
# ============================================================
import numpy as np
import omni.kit.app

from isaacsim.examples.interactive.base_sample import BaseSample
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.robot.wheeled_robots.controllers.wheel_base_pose_controller import WheelBasePoseController
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
from isaacsim.storage.native import get_assets_root_path

# ── Config ────────────────────────────────────────────────────
# Square path defined by 4 waypoints (x, y) in meters
WAYPOINTS = [
    np.array([0.0, 0.0]),  # origin (start)
    np.array([0.0, 1.0]),  # 1m along Y
    np.array([1.0, 1.0]),  # diagonal corner
    np.array([1.0, 0.0]),  # 1m along X
]
# Distinct color for each waypoint marker cube
WAYPOINT_COLORS = [
    np.array([1.0, 0.0, 0.0]),  # red
    np.array([0.0, 1.0, 0.0]),  # green
    np.array([0.0, 0.0, 1.0]),  # blue
    np.array([1.0, 1.0, 0.0]),  # yellow
]
# How close (meters) the robot must be to a waypoint to count as "reached"
WAYPOINT_REACH_THRESHOLD = 0.04
# Number of full loops around the square before stopping
NUM_LAPS = 2


# ── BaseSample lifecycle ──────────────────────────────────────
class AMRNavigation(BaseSample):
    def __init__(self):
        super().__init__()
        return

    def setup_scene(self):
        """Add all objects to the scene (called once on LOAD)."""
        world = self.get_world()
        world.scene.add_default_ground_plane()

        # Load JetBot from NVIDIA's asset library.
        # WheeledRobot wraps the USD and exposes wheel joint control.
        assets_root = get_assets_root_path()
        world.scene.add(
            WheeledRobot(
                prim_path="/World/Fancy_Robot",
                name="kyle_jetbot",
                wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
                create_robot=True,
                usd_path=assets_root + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd",
            )
        )

        # Place a small colored cube at each waypoint for visualization
        for i, (wp, color) in enumerate(zip(WAYPOINTS, WAYPOINT_COLORS)):
            world.scene.add(
                VisualCuboid(
                    prim_path=f"/World/Waypoint_{i}",
                    name=f"waypoint_{i}",
                    position=np.array([wp[0], wp[1], 0.05]),
                    scale=np.array([0.08, 0.08, 0.08]),
                    color=color,
                )
            )
        return

    async def setup_post_load(self):
        """Get object references and initialize controllers (called after physics ready)."""
        self._world = self.get_world()
        self._jetbot = self._world.scene.get_object("kyle_jetbot")

        # WheelBasePoseController (high-level): takes start pose + goal position,
        # internally uses DifferentialController (low-level) to compute wheel velocities.
        # wheel_radius and wheel_base are JetBot-specific physical dimensions.
        # is_holonomic=False because JetBot is a differential-drive robot (cannot strafe).
        self._my_controller = WheelBasePoseController(
            name="cool_controller",
            open_loop_wheel_controller=DifferentialController(
                name="simple_control",
                wheel_radius=0.03,
                wheel_base=0.1125,
                max_linear_speed=0.5,   # default is 0.3 m/s — increase for faster driving
                max_angular_speed=1.0,  # default is ~0.52 rad/s — increase for faster turning
            ),
            is_holonomic=False,
        )

        # Navigation state
        self._current_waypoint_index = 0
        self._laps_completed = 0

        # Register physics_step to be called automatically every simulation tick
        self._world.add_physics_callback("sim_step", callback_fn=self.physics_step)
        await self._world.play_async()
        return

    def physics_step(self, step_size):
        """Called every physics tick — implements the navigation FSM.

        1. Read the robot's current world pose
        2. Compute distance to current target waypoint
        3. If close enough, advance to next waypoint (wrap around for laps)
        4. Send navigation command to the controller
        """
        position, orientation = self._jetbot.get_world_pose()
        goal = WAYPOINTS[self._current_waypoint_index]
        dist = np.linalg.norm(position[:2] - goal)

        # Check arrival — advance to next waypoint if within threshold
        if dist < WAYPOINT_REACH_THRESHOLD:
            prev = self._current_waypoint_index
            self._current_waypoint_index = (self._current_waypoint_index + 1) % len(WAYPOINTS)
            if self._current_waypoint_index == 0:
                self._laps_completed += 1
            print(f"Reached waypoint {prev}! Next: {self._current_waypoint_index} -> {WAYPOINTS[self._current_waypoint_index]}")
            self._my_controller.reset()

        # Command the robot to drive toward the current waypoint
        self._jetbot.apply_action(
            self._my_controller.forward(
                start_position=position,
                start_orientation=orientation,
                goal_position=np.array([goal[0], goal[1], 0.0]),
            )
        )
        return

    async def setup_post_reset(self):
        """Reset navigation state when RESET is pressed."""
        self._current_waypoint_index = 0
        self._laps_completed = 0
        self._my_controller.reset()
        await self._world.play_async()
        return


# ── Run ───────────────────────────────────────────────────────
# Instantiate and run the full BaseSample lifecycle:
#   create_new_stage -> World() -> initialize_simulation_context
#   -> setup_scene -> reset_async -> setup_post_load
sample = AMRNavigation()
await sample.load_world_async()
print(f"Starting AMR navigation: {NUM_LAPS} laps through {len(WAYPOINTS)} waypoints")

# physics_step is called automatically each tick via the registered callback.
# We just wait here until the robot completes the required laps.
for _ in range(15000):
    await omni.kit.app.get_app().next_update_async()
    if sample._laps_completed >= NUM_LAPS:
        print(f"Completed {NUM_LAPS} laps!")
        break

await sample.get_world().pause_async()
print("AMR navigation demo complete!")
