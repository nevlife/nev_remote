from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('nev_remote'), 'config', 'video_encoder_params.yaml'
    ])

    return LaunchDescription([
        Node(
            package='nev_remote',
            executable='zenoh_video_encoder.py',
            name='zenoh_video_encoder',
            output='screen',
            parameters=[params],
        ),
    ])
