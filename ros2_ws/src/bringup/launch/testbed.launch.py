"""Compose the simulator: the CARLA-Air bridge, and nothing else.

Assumes two things are already up, because both are slow and neither belongs in a launch
file that gets restarted every few minutes:

    ./scripts/run_sim.sh                       # CARLA-Air, headless (~40 s to serve)
    ./.venv/bin/python sim_bridge/server.py    # the Python 3.10 sidecar

`scripts/bringup.sh` starts all of it in order and is the normal entry point.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """The simulator's ROS 2 surface, and nothing that decides where to fly.

    This has shed two layers, both for the same reason.

    `vlm_client` and `grounding` left in R-02: a VLM was the price of admission for a drone
    simulator. They are an example now - `examples/vlm_navigation/`.

    `control` and `evaluation` left on 2026-08-04, under the scope agreed that day
    (`.ai/AGENTS.md` -> Scope). `offboard_control` decides WHERE THE AIRCRAFT GOES - standoff,
    step capping, an altitude floor, slew limits - and `episode_runner` scores a navigation
    policy against a scenario. Both are things you build ON a simulator. They live in
    `examples/navigation/` and start separately.

    What is left is what a simulator owes every user regardless of what is flying it: sensors,
    the command surface, world control. `examples/ros2_full_control.py` flies the aircraft
    through this alone, importing nothing from the project, which is the test that it is
    sufficient.
    """
    share = get_package_share_directory("bringup")
    default_params = os.path.join(share, "config", "testbed.yaml")

    args = [
        DeclareLaunchArgument("params", default_value=default_params,
                              description="parameter file applied to every node"),
        DeclareLaunchArgument("socket_path", default_value="/tmp/carla_air_testbed.sock"),
    ]

    nodes = [
        Node(
            package="carla_air_bridge", executable="bridge_node", name="carla_air_bridge",
            output="screen",
            parameters=[LaunchConfiguration("params"),
                        {"socket_path": LaunchConfiguration("socket_path")}],
        ),
    ]

    return LaunchDescription(args + nodes)
