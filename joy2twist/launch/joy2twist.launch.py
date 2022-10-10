from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    joy2twist_cfg_path = PathJoinSubstitution(
        [FindPackageShare("joy2twist"), "config", "joy2twist.yaml"]
    )

    joy2twist_params_file_argument = DeclareLaunchArgument(
        "joy2twist_params_file",
        default_value=joy2twist_cfg_path,
        description="ROS2 parameters file to use with joy1twist node",
    )

    joy2twist_node = Node(
        package="joy2twist",
        executable="joy2twist",
        parameters=[LaunchConfiguration("joy2twist_params_file")],
        # output={"stdout": "screen", "stderr": "screen"},
        emulate_tty="true",
    )

    actions = [joy2twist_params_file_argument, joy2twist_node]

    return LaunchDescription(actions)
