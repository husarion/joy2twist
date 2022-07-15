import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    cfg_path = os.path.join(get_package_share_directory(
        'joy2twist'), 'config', 'joy2twist.yaml')

    with open(cfg_path, 'r') as config_o:
        config = yaml.safe_load(config_o)
    ros_param = config['/**']['ros__parameters']

    joy2twist_node = Node(
        package='joy2twist',
        executable='joy2twist',
        parameters=[ros_param],
        output={
            'stdout': 'screen',
            'stderr': 'screen'
        },
        emulate_tty='true'
    )

    ld.add_action(joy2twist_node)

    return ld
