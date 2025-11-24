from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction        # Need this to call the tutors launch file without copying the code
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Start the simulation
    # Include original tutors launch file
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                # Path to the actual file
                get_package_share_directory('ir_master_launch'),
                'launch',
                'assignment_1.launch.py'
            ])
        )
    )


    # Start the navigation Node
    navigation = Node(
        package='navigation_node',
        executable='navigation_node',
        # Send messages and logs to the terminal
        output='screen'
    )

    # Start the apriltag detector
    apriltag = Node(
        package='apriltag_detect',
        executable='apriltag_node',
        remappings=[
            # Remap expected topics names (between camera and node)
            ('image_rect', '/rgb_camera/image'),
            ('camera_info', '/rgb_camera/camera_info'),
        ],
        parameters=[
            PathJoinSubstitution([
                # LOad file with tag detection parameters
                get_package_share_directory('apriltag_detect'),
                'cfg',
                'tags_36h11.yaml'
            ])
        ],
        output='screen'
    )

    # Start the compute goal node
    compute_goal = Node(
        package='compute_goal',
        executable='compute_goal_node',
        output='screen'
    )

    # Return launch descriptions with delays
    return LaunchDescription([
        # Simulation runs immediately
        simulation_launch,

        # Wait 5 seconds before the other 3 components
        
        TimerAction(period=5.0, actions=[navigation]),

        TimerAction(period=5.0, actions=[apriltag]),

        TimerAction(period=5.0, actions=[compute_goal]),
    ])
