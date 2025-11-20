from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='apriltag_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        emulate_tty=True,
        composable_node_descriptions=[
            ComposableNode(
                package='apriltag_ros',
                plugin='apriltag_ros::AprilTagNode',
                name='apriltag',
                parameters=['$(find-pkg-share apriltag_ros)/cfg/tags_36h11.yaml'],
                remappings=[
                    ('image_rect', '/camera/image_rect'),
                    ('camera_info', '/camera/camera/color/camera_info'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ]
    )

    return LaunchDescription([container])

