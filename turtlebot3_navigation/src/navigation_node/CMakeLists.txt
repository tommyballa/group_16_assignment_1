cmake_minimum_required(VERSION 3.8)
project(navigation_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

add_executable(navigation_node src/navigation_node.cpp)

ament_target_dependencies(navigation_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  sensor_msgs
)

install(TARGETS navigation_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


