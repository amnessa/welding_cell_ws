cmake_minimum_required(VERSION 3.8)
project(redball_follower)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_geometry REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_core REQUIRED)

# Include directories for OpenCV
include_directories(${OpenCV_INCLUDE_DIRS})

# ============================================
# C++ Executables
# ============================================

# Ball Tracker Node - D435 camera red ball detection
add_executable(ball_tracker_node src/ball_tracker_node.cpp)
ament_target_dependencies(ball_tracker_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  message_filters
  image_geometry
)
target_link_libraries(ball_tracker_node ${OpenCV_LIBRARIES})

# UR5e Velocity Controller Node - IBVS control with manipulability
add_executable(ur5e_velocity_controller_node src/ur5e_velocity_controller_node.cpp)
ament_target_dependencies(ur5e_velocity_controller_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
)

# Jacobian Calculator Node - Manipulability computation
add_executable(jacobian_calculator_node src/jacobian_calculator_node.cpp)
ament_target_dependencies(jacobian_calculator_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  moveit_ros_planning
  moveit_core
)
target_link_libraries(jacobian_calculator_node Eigen3::Eigen)

# Install C++ executables
install(TARGETS
  ball_tracker_node
  ur5e_velocity_controller_node
  jacobian_calculator_node
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================
# Python scripts
# ============================================
install(PROGRAMS
  scripts/red_ball_spawner.py
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================
# Install directories
# ============================================
install(DIRECTORY
  launch
  config
  model
  urdf
  meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
