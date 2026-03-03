import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nev_remote = get_package_share_directory('nev_remote')
    params_file = os.path.join(pkg_nev_remote, 'config', 'video_encoder_params.yaml')

    zenoh_video_encoder_node = Node(
        package='nev_remote',
        executable='zenoh_video_encoder.py',
        name='zenoh_video_encoder',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        zenoh_video_encoder_node,
    ])
