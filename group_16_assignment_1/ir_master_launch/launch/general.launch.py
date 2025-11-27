from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Avoid Gazebo, Nav2, Rviz spam
    set_log_level = SetEnvironmentVariable('ROS_LOG_LEVEL', 'WARN')

    # Shorter format for messages
    log_format = SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] [{name}]: {message}')

    # NODES
    # Start the simulation (now silent)
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('ir_master_launch'),
                'launch',
                'simulation.launch.py'
            ])
        )
    )

    # Add initial pose node
    initial_pose_publisher = Node(
            package='initial_pose_publisher',
            executable='initial_pose_publisher',
            output='screen',
            # INFO is to let the node talk skipping the global WARN limit
            arguments=['--ros-args', '--log-level', 'INFO']
    )
    
    # Add navigation node
    navigation = Node(
        package='navigation_node',
        executable='navigation_node',
        output='screen',
    )

    # Add table detection node
    table_node = Node(
        package='table_node',
        executable='table_detection_node',
        output='screen',
        # Set to info for same reason
        arguments=['--ros-args', '--log-level', 'INFO']
    )
    
    # Add apriltag detector
    apriltag = Node(
        package='apriltag_detect',
        executable='apriltag_node',
        remappings=[
            ('image_rect', '/rgb_camera/image'),
            ('camera_info', '/rgb_camera/camera_info'),
        ],
        parameters=[
            PathJoinSubstitution([
                get_package_share_directory('apriltag_detect'),
                'cfg',
                'tags_36h11.yaml'
            ])
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO']
    )

    # Add compute goal node
    compute_goal = Node(
        package='compute_goal',
        executable='compute_goal_node',
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO']
    )

    # Return launch descriptions with delays
    return LaunchDescription([
        # Clean messages
        set_log_level,
        log_format,

        # Simulation runs immediately
        simulation_launch,

        # Wait 5 seconds before the other 3 components
        TimerAction(period=5.0, actions=[initial_pose_publisher]),

        TimerAction(period=10.0, actions=[navigation]),
        TimerAction(period=10.0, actions=[table_node]),
        TimerAction(period=14.0, actions=[apriltag]),
        TimerAction(period=15.0, actions=[compute_goal]),
    ])
