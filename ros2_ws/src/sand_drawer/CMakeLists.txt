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
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_core REQUIRED)

# ============================================
# C++ Executables
# ============================================

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

# Install C++ executables
install(TARGETS
  sand_drawer_jacobian_node
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================
# Python scripts
# ============================================
install(PROGRAMS
  scripts/plane_solver_node.py
  scripts/plane_frame_broadcaster.py
  scripts/planar_servo_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================
# Install directories
# ============================================
install(DIRECTORY
  launch
  config
  urdf
  meshes
  world
  generated_planes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
