from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(ExecuteProcess(
        cmd=[["ros2 run joy_linux joy_linux_node"]],
        shell=True
    ))

    launch_joy2twist = create_include_launch(package='joy2twist',
                                             rel_launch_path='/launch/joy2twist.launch.py',
                                             arguments={})
    ld.add_action(launch_joy2twist)

    return ld


def create_include_launch(package: str, rel_launch_path: str, arguments: dict):
    share_dir_path = get_package_share_directory(package)
    included_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        share_dir_path + rel_launch_path),
        launch_arguments=arguments.items())
    return included_launch
