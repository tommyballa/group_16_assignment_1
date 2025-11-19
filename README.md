# group_16_assignment_1
Assignment 1 of Intelligent Robotics


# how to run
```bash
source /opt/ros/jazzy/setup.bash \
colcon build \
source install/setup.bash \
ros2 launch ir_launch assignment_1.launch.py \
ros2 launch slam_toolbox online_async_launch.py \

ros2 run apriltag_detect apriltag_node --ros-args \\
     -r image_rect:=/rgb_camera/image \\
     -r camera_info:=/rgb_camera/image/camera_info \\
     --params-file `ros2 pkg prefix apriltag_detect`/share/apriltag_detect/cfg/tags_36h11.yaml \ 
    


ros2 run compute_goal compute_goal_node

