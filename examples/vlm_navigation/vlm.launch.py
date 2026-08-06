"""See-Point-Fly on top of the simulator — an example, not part of it.

    ./scripts/bringup.sh --config configs/testbed.yaml                       # terminal 1: the simulator, no VLM
    ./examples/vlm_navigation/run.sh --backend oracle    # terminal 2: this

Two nodes, and everything they touch is the public ROS 2 interface:

    /camera/rgb/image_raw   ->  vlm_client  ->  /vlm/annotation      (a pixel)
    /vlm/annotation         ->  grounding   ->  /control/waypoint    (an NED point)

`grounding` needs `/camera/depth/image_raw`, `/camera/rgb/camera_info` and
`/fmu/out/vehicle_odometry` as well. All of them come from the bridge, none of them from a
private channel — which is the whole point of this being an example rather than a component:
if it can be built this way, so can anything else.

Config lives HERE, in config/vlm.yaml, not in the simulator's configs/testbed.yaml. Someone
installing a drone simulator should not find an Anthropic model name in it.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    default_params = os.path.join(HERE, "config", "vlm.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("params", default_value=default_params),
        DeclareLaunchArgument(
            "instruction", default_value="fly forward and stay clear of buildings"),
        # `backend:=none` starts the GROUNDING LAYER ONLY, for anyone bringing their own
        # model. Grounding is the pixel-to-NED transform; it consumes /vlm/annotation and
        # does not care what produced it. Without this the only way to reuse it was to run
        # `ros2 run grounding grounding_node` by hand, or to run a shipped backend that then
        # competes with your own for the same topic — see examples/byo_agent.py.
        DeclareLaunchArgument(
            "backend", default_value="geometric",
            description="mock | scripted | geometric | oracle | claude | none"),
        Node(
            package="vlm_client", executable="vlm_node", name="vlm_client", output="screen",
            parameters=[LaunchConfiguration("params"),
                        {"backend": LaunchConfiguration("backend"),
                         "instruction": LaunchConfiguration("instruction")}],
            condition=UnlessCondition(PythonExpression(
                ["'", LaunchConfiguration("backend"), "' == 'none'"])),
        ),
        Node(
            package="grounding", executable="grounding_node", name="grounding",
            output="screen", parameters=[LaunchConfiguration("params")],
        ),
    ])
