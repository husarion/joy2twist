import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    joy2twist_cfg_path = os.path.join(get_package_share_directory(
        'joy2twist'), 'config', 'joy2twist.yaml')

    declare_joy2twist_params_file_argument = DeclareLaunchArgument(
        'joy2twist_params_file',
        default_value=joy2twist_cfg_path,
        description='ROS2 parameters file to use with joy1twist node')
    ld.add_action(declare_joy2twist_params_file_argument)

    joy2twist_node = Node(
        package='joy2twist',
        executable='joy2twist',
        parameters=[LaunchConfiguration('joy2twist_params_file')],
        output={
            'stdout': 'screen',
            'stderr': 'screen'
        },
        emulate_tty='true')
    ld.add_action(joy2twist_node)

    return ld
