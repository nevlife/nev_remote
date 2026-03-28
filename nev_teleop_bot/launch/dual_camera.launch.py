import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('nev_teleop_bot')
    params_file = os.path.join(pkg, 'config', 'dual_camera_params.yaml')

    dual_camera_node = Node(
        package='nev_teleop_bot',
        executable='dual_camera_encoder.py',
        name='dual_camera_encoder',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([dual_camera_node])
