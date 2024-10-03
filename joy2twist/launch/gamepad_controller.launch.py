from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
        description="ROS2 parameters file to use with joy2twist node",
    )

    joy2twist_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("joy2twist"), "launch", "joy2twist.launch.py"]
                )
            ]
        ),
        launch_arguments={
            "joy2twist_params_file": LaunchConfiguration("joy2twist_params_file"),
            "namespace": namespace,
        }.items(),
    )

    joy_linux_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
        emulate_tty="true",
        namespace=namespace,
        remappings=[("/diagnostics", "diagnostics")],
    )

    actions = [
        declare_namespace_arg,
        joy2twist_params_file_argument,
        joy2twist_launch,
        joy_linux_node,
    ]

    return LaunchDescription(actions)
