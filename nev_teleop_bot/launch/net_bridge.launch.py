import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('nev_teleop_bot')
    network_file = os.path.join(pkg, 'config', 'network.yaml')
    params_file = os.path.join(pkg, 'config', 'net_bridge_params.yaml')
    teleop_topics_config = os.path.join(pkg, 'config', 'teleop_topics.yaml')

    net_bridge_node = Node(
        package='nev_teleop_bot',
        executable='net_bridge.py',
        name='net_bridge',
        output='screen',
        parameters=[
            network_file,
            params_file,
            {'teleop_topics_config': teleop_topics_config},
        ],
    )

    return LaunchDescription([
        net_bridge_node,
    ])
