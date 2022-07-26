from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    joy2twist_cfg_path = PathJoinSubstitution(
        [FindPackageShare("joy2twist"), "config", "joy2twist.yaml"]
    )

    joy2twist_params_file_argument = DeclareLaunchArgument(
        "joy2twist_params_file",
        default_value=joy2twist_cfg_path,
        description="ROS2 parameters file to use with joy2twist node",
    )

    joy2twist_launch = create_include_launch(
        package="joy2twist",
        rel_launch_path="/launch/joy2twist.launch.py",
        arguments={
            "joy2twist_params_file": LaunchConfiguration("joy2twist_params_file")
        },
    )

    joy_linux_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
        output={"stdout": "screen", "stderr": "screen"},
        emulate_tty="true",
    )

    actions = [joy2twist_params_file_argument, joy2twist_launch, joy_linux_node]

    return LaunchDescription(actions)


def create_include_launch(package: str, rel_launch_path: str, arguments: dict):
    share_dir_path = get_package_share_directory(package)
    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(share_dir_path + rel_launch_path),
        launch_arguments=arguments.items(),
    )
    return included_launch