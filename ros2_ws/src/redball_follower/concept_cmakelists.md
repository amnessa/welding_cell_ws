cmake_minimum_required(VERSION 3.8)
project(ur10_ball_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_ros REQUIRED)

# Ball Tracker Node
add_executable(ball_tracker_node src/ball_tracker_node.cpp)
ament_target_dependencies(ball_tracker_node rclcpp sensor_msgs cv_bridge OpenCV geometry_msgs)
install(TARGETS ball_tracker_node DESTINATION lib/${PROJECT_NAME})

# Jacobian Calculator Node
add_executable(jacobian_calculator_node src/jacobian_calculator_node.cpp)
ament_target_dependencies(jacobian_calculator_node rclcpp sensor_msgs)
install(TARGETS jacobian_calculator_node DESTINATION lib/${PROJECT_NAME})

# UR10 Arm Controller Node
add_executable(ur10_arm_controller_node src/ur10_arm_controller_node.cpp)
ament_target_dependencies(ur10_arm_controller_node rclcpp std_msgs geometry_msgs sensor_msgs)
install(TARGETS ur10_arm_controller_node DESTINATION lib/${PROJECT_NAME})

# Mobile Base Controller Node
add_executable(mobile_base_controller src/mobile_base_controller.cpp)
ament_target_dependencies(mobile_base_controller rclcpp geometry_msgs)
install(TARGETS mobile_base_controller DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
