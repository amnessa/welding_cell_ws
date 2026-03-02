cmake_minimum_required(VERSION 3.8)
project(sand_drawer)

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
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_core REQUIRED)

# ============================================
# C++ Executables
# ============================================

# UR5e Velocity Controller Node - IBVS control with manipulability
add_executable(sand_drawer_velocity_controller_node src/ur5e_velocity_controller_node.cpp)
ament_target_dependencies(sand_drawer_velocity_controller_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
)

# Jacobian Calculator Node - Manipulability computation
add_executable(sand_drawer_jacobian_node src/jacobian_calculator_node.cpp)
ament_target_dependencies(sand_drawer_jacobian_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  moveit_ros_planning
  moveit_core
)
target_link_libraries(sand_drawer_jacobian_node Eigen3::Eigen)

# Lula Position Control Node - ROS2 C++ trigger wrapper for Isaac Lula script
add_executable(lula_position_control_node src/lula_position_control.cpp)
ament_target_dependencies(lula_position_control_node
  rclcpp
  std_srvs
)

# Install C++ executables
install(TARGETS
  sand_drawer_velocity_controller_node
  sand_drawer_jacobian_node
  lula_position_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================
# Python scripts
# ============================================
install(PROGRAMS
  scripts/plane_solver_node.py
  scripts/lula_complicated_trajectory.py
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
  world
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
