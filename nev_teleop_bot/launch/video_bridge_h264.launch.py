import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('nev_teleop_bot')
    network_file = os.path.join(pkg, 'config', 'network.yaml')
    params_file = os.path.join(pkg, 'config', 'video_params_h264.yaml')

    video_bridge_node = Node(
        package='nev_teleop_bot',
        executable='video_bridge_h264',
        name='video_bridge_h264',
        output='screen',
        arguments=['--ros-args', '--log-level', 'video_bridge_h264:=debug'],
        parameters=[network_file, params_file],
    )

    return LaunchDescription([
        video_bridge_node,
    ])
