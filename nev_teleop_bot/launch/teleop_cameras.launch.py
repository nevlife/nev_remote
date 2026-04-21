from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        name='teleop_camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='nev_teleop_bot',
                plugin='MultiCamPub',
                name='multi_cam_pub',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='nev_teleop_bot',
                plugin='WideCamPub',
                name='wide_cam_pub',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )
    return LaunchDescription([container])
