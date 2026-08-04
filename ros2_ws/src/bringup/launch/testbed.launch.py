"""Compose the testbed: bridge → VLM → grounding → control (+ optional evaluation).

Assumes two things are already up, because both are slow and neither belongs in a launch
file that gets restarted every few minutes:

    ./scripts/run_sim.sh                       # CARLA-Air, headless (~40 s to serve)
    ./.venv/bin/python sim_bridge/server.py    # the Python 3.10 sidecar

`scripts/bringup.sh` starts all four in order and is the normal entry point.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """The simulator, and nothing that interprets a camera.

    This used to start `vlm_client` and `grounding` too, which made a VLM the price of
    admission for a drone simulator. They are an EXAMPLE now, started separately by
    `examples/vlm_navigation/vlm.launch.py` against the ROS 2 interface like anyone else's
    code would be. See todo.md R-02.

    What stays here is what a simulator owes any user regardless of what is flying it: the
    bridge, the offboard controller, episode running and scoring, and video recording.
    `episode_runner` is NOT VLM-specific - it starts episodes and scores distance-to-goal
    from odometry.
    """
    share = get_package_share_directory("bringup")
    default_params = os.path.join(share, "config", "testbed.yaml")
    scenarios = os.path.join(
        get_package_share_directory("evaluation"), "scenarios", "default.yaml")

    args = [
        DeclareLaunchArgument("params", default_value=default_params,
                              description="parameter file applied to every node"),
        DeclareLaunchArgument("evaluation", default_value="true",
                              description="run the episode runner"),
        DeclareLaunchArgument("socket_path", default_value="/tmp/carla_air_testbed.sock"),
        # On by default: a failed episode used to leave a number and nothing to look at.
        DeclareLaunchArgument("record", default_value="true",
                              description="record every episode to out/videos/<episode_id>.mp4"),
    ]

    params = LaunchConfiguration("params")

    nodes = [
        Node(
            package="carla_air_bridge", executable="bridge_node", name="carla_air_bridge",
            output="screen",
            parameters=[params, {"socket_path": LaunchConfiguration("socket_path")}],
        ),
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
            output="screen", parameters=[params, {"scenarios_file": scenarios}],
            condition=IfCondition(PythonExpression(
                ["'", LaunchConfiguration("evaluation"), "' == 'true'"])),
        ),
    ]

    return LaunchDescription(args + nodes)
