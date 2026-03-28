import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nev_teleop_bot = get_package_share_directory('nev_teleop_bot')
    params_file = os.path.join(pkg_nev_teleop_bot, 'config', 'net_bridge_params.yaml')

    net_bridge_node = Node(
        package='nev_teleop_bot',
        executable='net_bridge.py',
        name='net_bridge',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        net_bridge_node,
    ])
