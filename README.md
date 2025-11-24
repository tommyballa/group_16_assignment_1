# RUNNING THE GENERAL LAUNCH

Needed in workspace:
ir2526/
ir_general_launch/
turtlebot3_navigator/

Terminal:

source /opt/ros/jazzy/setup.bash 
colcon build
source install/setup.bash
ros2 launch ir_master_launch general.launch.py
