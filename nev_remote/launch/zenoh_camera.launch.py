from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('nev_remote'), 'config', 'zenoh_camera_params.yaml'
    ])

    return LaunchDescription([
        Node(
            package='nev_remote',
            executable='zenoh_camera.py',
            name='zenoh_camera',
            output='screen',
            parameters=[params],
        ),
    ])
