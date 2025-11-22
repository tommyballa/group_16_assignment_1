# group_16_assignment_1
Assignment 1 of Intelligent Robotics


# how to run
```bash
source /opt/ros/jazzy/setup.bash 
colcon build 
source install/setup.bash

terminal 1:
ros2 launch ir_launch assignment_1.launch.py

terminal 2:
ros2 run navigation_node navigation_node

---ros2 launch slam_toolbox online_async_launch.py--- secondo me non serve questo

terminal 3:
ros2 run apriltag_detect apriltag_node --ros-args \
     -r image_rect:=/rgb_camera/image \
     -r camera_info:=/rgb_camera/camera_info \
     --params-file `ros2 pkg prefix apriltag_detect`/share/apriltag_detect/cfg/tags_36h11.yaml

terminal 4:
ros2 run compute_goal compute_goal_node



