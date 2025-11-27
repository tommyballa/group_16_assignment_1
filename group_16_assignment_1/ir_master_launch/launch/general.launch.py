from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction        # Need this to call the tutors launch file without copying the code
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Start the simulation
    # Include simulation launch file
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                # Path to the actual file
                get_package_share_directory('ir_master_launch'),
                'launch',
                'simulation.launch.py'
            ])
        )
    )

    # EACH NODE GOES IN ITS OWN TERMINAL FOR BETTER OUTPUT READING
    # Add initial pose node
    initial_pose_publisher = ExecuteProcess(
    cmd=[
        'gnome-terminal', '--',
        'ros2', 'run', 'initial_pose_publisher', 'initial_pose_publisher'
    ],
    output='screen'
    )

    # Add navigation node
    navigation = ExecuteProcess(
    cmd=[
        'gnome-terminal', '--',
        'ros2', 'run', 'navigation_node', 'navigation_node'
    ],
    output='screen'
    )

    # Add table detection node
    table_node = ExecuteProcess(
    cmd=[
        'gnome-terminal', '--',
        'ros2', 'run', 'table_node', 'table_detection_node'
    ],
    output='screen'
    )
    
    # APRILTAG PARAMS
    apriltag_params = os.path.join(
    get_package_share_directory('apriltag_detect'),
        'cfg',
        'tags_36h11.yaml'
    )
    # Add apriltag detector
    apriltag = ExecuteProcess(
    cmd=[
        'gnome-terminal', '--',
        'ros2', 'run', 'apriltag_detect', 'apriltag_node',
        '--ros-args',
        '-r', 'image_rect:=/rgb_camera/image',
        '-r', 'camera_info:=/rgb_camera/camera_info',
        '--params-file', apriltag_params
    ],
    output='screen'
    )

    # Add compute goal node
    compute_goal = ExecuteProcess(
    cmd=[
        'gnome-terminal', '--',
        'ros2', 'run', 'compute_goal', 'compute_goal_node'
    ],
    output='screen'
    )

    # Return launch descriptions with delays
    return LaunchDescription([
        # Simulation runs immediately
        simulation_launch,

        # Wait 5 seconds before the other 3 components
        TimerAction(period=5.0, actions=[initial_pose_publisher]),

        TimerAction(period=10.0, actions=[navigation]),
        TimerAction(period=10.0, actions=[table_node]),
        TimerAction(period=14.0, actions=[apriltag]),
        TimerAction(period=15.0, actions=[compute_goal]),
    ])
