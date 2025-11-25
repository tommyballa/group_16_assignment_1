cmake_minimum_required(VERSION 3.16)
project(initial_pose_publisher)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(initial_pose_publisher src/initial_pose_publisher.cpp)
ament_target_dependencies(initial_pose_publisher rclcpp geometry_msgs)

install(TARGETS
  initial_pose_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

