import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()

    joy2twist_cfg_path = os.path.join(get_package_share_directory(
        'joy2twist'), 'config', 'joy2twist.yaml')

    declare_joy2twist_params_file_argument = DeclareLaunchArgument(
        'joy2twist_params_file',
        default_value=joy2twist_cfg_path,
        description='ROS2 parameters file to use with joy2twist node')
    ld.add_action(declare_joy2twist_params_file_argument)

    run_joy_linux = ExecuteProcess(
        cmd=[['ros2 run joy_linux joy_linux_node']],
        shell=True)
    ld.add_action(run_joy_linux)

    launch_joy2twist = create_include_launch(package='joy2twist',
                                             rel_launch_path='/launch/joy2twist.launch.py',
                                             arguments={'joy2twist_params_file': LaunchConfiguration('joy2twist_params_file')})
    ld.add_action(launch_joy2twist)

    return ld


def create_include_launch(package: str, rel_launch_path: str, arguments: dict):
    share_dir_path = get_package_share_directory(package)
    included_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        share_dir_path + rel_launch_path),
        launch_arguments=arguments.items())
    return included_launch
