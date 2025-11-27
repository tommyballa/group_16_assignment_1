# group_16_assignment_1
    Assignment 1 of Intelligent Robotics

# Suggested workspace structure:

    ws_16_assignments

    └── src

        ├── group_16_assignment_1
        
        │       ├── ir_master_launch
        
        │       ├── README.md
        
        │       └── turtlebot3_navigation
        
        └── ir_2526
        
                ├── apriltag_ros
                
                ├── ir_base
                
                ├── ir_desription
                
                ├── ir_launch
                
                └── ir_movit_config

# How to run
```bash
    source /opt/ros/jazzy/setup.bash 
    colcon build 
    
    source install/setup.bash
    
    ros2 launch ir_master_launch general.launch.py
