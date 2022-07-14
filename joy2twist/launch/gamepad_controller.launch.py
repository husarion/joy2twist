from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    lanuch_arbiter = create_include_launch(package='joy',
                                           rel_launch_path='/launch/arbiter.launch.py')
    ld.add_action(lanuch_arbiter)

    launch_rm = create_include_launch(package='joy2twist',
                                      rel_launch_path='/launch/joy2twist.launch.py')
    ld.add_action(launch_rm)

    return ld


def create_include_launch(package: str, rel_launch_path: str, arguments: dict):
    share_dir_path = get_package_share_directory(package)
    included_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        share_dir_path + rel_launch_path),
        launch_arguments=arguments.items())
    return included_launch
