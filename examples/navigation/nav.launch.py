"""Waypoint following, episode running and video recording — as an EXAMPLE.

    ./scripts/bringup.sh --config configs/testbed.yaml   # the simulator
    ./examples/navigation/run.sh                         # this

These three nodes were started by `bringup.sh` until 2026-08-04, which made a navigation
stack the price of admission for a drone simulator. They are out of scope for the repository
(see `.ai/AGENTS.md` → Scope) and now start separately, against the public ROS 2 interface,
exactly as anyone else's navigation code would.

What each one is, and why none of it belongs to the simulator:

* **offboard_control** — turns a `GroundedWaypoint` into a stream of `TrajectorySetpoint`.
  It decides *where the aircraft goes*: standoff from the surface, a per-step distance cap,
  an altitude floor, and slew limits on velocity and heading. Every one of those is a policy
  choice, and a user with their own navigation stack wants none of them.
* **episode_runner** — starts a scenario, scores distance-to-goal, and writes a result.
  Benchmarking a policy.
* **recorder** — writes the onboard view, with a HUD, per episode. Closest to simulator
  tooling of the three, but it draws episode status, so it follows the episode runner.

**You do not need this to fly the aircraft.** `examples/ros2_full_control.py` takes off,
flies waypoints, holds a velocity, commands an attitude and lands, through
`/fmu/in/trajectory_setpoint` alone, importing nothing from this project. That is the proof
the simulator's interface stands on its own without any of the above.

**Known wart, deliberately not fixed here.** The parameters these nodes read still live in
`configs/testbed.yaml` under `graph.*` and are rendered into the bringup package's config.
Splitting navigation parameters out into `examples/navigation/config/` is follow-up work; it
touches `apply_config.py` and every scenario override, and doing it in the same change as
the move would make both harder to review.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("bringup")
    default_params = os.path.join(share, "config", "testbed.yaml")
    scenarios = os.path.join(
        get_package_share_directory("evaluation"), "scenarios", "default.yaml")

    args = [
        DeclareLaunchArgument("params", default_value=default_params,
                              description="parameter file applied to every node"),
        DeclareLaunchArgument("evaluation", default_value="true",
                              description="run the episode runner and scoring"),
        DeclareLaunchArgument("record", default_value="true",
                              description="record the onboard view to out/videos/<episode_id>.mp4"),
        DeclareLaunchArgument("scenarios", default_value=scenarios,
                              description="scenario definitions the episode runner reads"),
    ]

    params = LaunchConfiguration("params")

    nodes = [
        Node(
            package="control", executable="offboard_node", name="offboard_control",
            output="screen", parameters=[params],
        ),
        Node(
            package="evaluation", executable="recorder", name="recorder", output="screen",
            parameters=[params],
            # A condition, not a bool parameter: LaunchConfiguration hands over the string
            # "true", and feeding a string to a bool parameter fails at launch.
            condition=IfCondition(PythonExpression(
                ["'", LaunchConfiguration("record"), "' == 'true'"])),
        ),
        Node(
            package="evaluation", executable="episode_runner", name="episode_runner",
            output="screen",
            parameters=[params, {"scenarios_file": LaunchConfiguration("scenarios")}],
            condition=IfCondition(PythonExpression(
                ["'", LaunchConfiguration("evaluation"), "' == 'true'"])),
        ),
    ]

    return LaunchDescription(args + nodes)
