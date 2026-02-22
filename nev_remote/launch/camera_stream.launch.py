from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare('nev_remote'), 'config', 'camera_stream_params.yaml'
    ])

    return LaunchDescription([
        Node(
            package='nev_remote',
            executable='webrtc_camera.py',
            name='camera_streamer',
            output='screen',
            parameters=[params_file],
        ),
    ])
