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
find_package(rosidl_default_generators REQUIRED)

# ============================================
# Action / message generation
# ============================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteDrawing.action"
  "srv/ComputeTOTG.srv"
  DEPENDENCIES geometry_msgs
)

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

# TOTG Service Node — Time-Optimal Trajectory Generation
add_executable(totg_service_node src/totg_service_node.cpp)
ament_target_dependencies(totg_service_node
  rclcpp
  moveit_core
)
target_link_libraries(totg_service_node Eigen3::Eigen)
# Link generated service interfaces to the TOTG node
rosidl_get_typesupport_target(totg_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(totg_service_node ${totg_typesupport})

# Install C++ executables
install(TARGETS
  sand_drawer_jacobian_node
  totg_service_node
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================
# Python scripts
# ============================================
install(PROGRAMS
  scripts/plane_solver_node.py
  scripts/plane_frame_broadcaster.py
  scripts/planar_servo_controller.py
  scripts/plane_teleop_keyboard.py
  scripts/ur5e_rrt_planner.py
  scripts/cartesian_square_controller.py
  scripts/pd_auto_tuner.py
  scripts/freedrive_plane_capture.py
  scripts/drawing_action_server.py
  scripts/drawing_dispatcher.py
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

ament_export_dependencies(rosidl_default_runtime)
ament_package()
