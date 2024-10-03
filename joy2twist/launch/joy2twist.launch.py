from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes.",
    )

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
        emulate_tty="true",
        namespace=namespace,
    )

    actions = [declare_namespace_arg, joy2twist_params_file_argument, joy2twist_node]

    return LaunchDescription(actions)
