import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nev_remote = get_package_share_directory('nev_remote')
    params_file = os.path.join(pkg_nev_remote, 'config', 'quad_camera_params.yaml')

    node = Node(
        package='nev_remote',
        executable='quad_camera_encoder.py',
        name='quad_camera_encoder',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([node])
