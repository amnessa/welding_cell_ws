cmake_minimum_required(VERSION 3.8)
project(admittance_control)

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
add_executable(admittance_control_jacobian_node src/jacobian_calculator_node.cpp)
ament_target_dependencies(admittance_control_jacobian_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  moveit_ros_planning
  moveit_core
)
target_link_libraries(admittance_control_jacobian_node Eigen3::Eigen)

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
  admittance_control_jacobian_node
  totg_service_node
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================
# Python library module (importable as `admittance_control.*`)
# Shared code (kinematics, geometry, sam6d I/O) lives here and is imported
# by the node scripts.
#
# NOTE: we deliberately do NOT call ament_python_install_package() here.
# rosidl_generate_interfaces() already registers a Python package named
# `admittance_control` (to ship the generated action/srv bindings), so a
# second registration would clash on CMake target names. Instead we install
# our module files straight into that same package directory. rosidl provides
# the (empty) top-level __init__.py; our modules sit alongside its
# action/ srv/ msg/ submodules.
# ============================================
install(
  FILES
    admittance_control/__init__.py
    admittance_control/kinematics.py
    admittance_control/geometry.py
    admittance_control/sam6d_io.py
    admittance_control/icp.py
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)

# ============================================
# Python scripts — main pipeline nodes only
# (calibration / legacy / GUI live under helper/ and are run directly, not
#  installed as package executables)
# ============================================
install(PROGRAMS
  scripts/realsense_camera_node.py
  scripts/realsense_sim_camera_node.py
  scripts/foundationpose_bridge_node.py
  scripts/sam6d_bridge_node.py
  scripts/detection_marker_node.py
  scripts/icp_pose_refiner_node.py
  scripts/camera_extrinsic_tf_publisher.py
  scripts/move_to_object_node.py
  scripts/cartesian_admittance_controller.py
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
  models
  world
  generated_planes
  helper
  scripts/rgb_depth_to_send
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
