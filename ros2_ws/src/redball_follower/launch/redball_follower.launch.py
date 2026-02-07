"""
Red Ball Follower - Main Launch File

This is the unified launcher for the UR5e red ball tracking system.
It launches all necessary nodes for:
- Robot state publisher (UR5e URDF)
- Ball tracker node (D455 camera vision)
- Velocity controller node (IBVS control)
- Jacobian calculator node (manipulability)
- Red ball spawner (dummy ball for testing)

Isaac Sim Topics (expected from simulation):
  - /d455/color/image_raw  (sensor_msgs/Image) - RGB camera
  - /d455/depth/image_raw  (sensor_msgs/Image) - Depth camera
  - /isaac_joint_states    (sensor_msgs/JointState) - Robot state
  - /isaac_joint_commands  (sensor_msgs/JointState) - Robot commands
  - /clock                 (rosgraph_msgs/Clock) - Simulation time

Camera Transform (D455 mounted on wrist_3_link):
  - Translation: (0, 0, 0.025) - 2.5cm along Z
  - Orientation: quat(0.5, 0.5, 0.5, -0.5) - 90° rotation

Usage:
  ros2 launch redball_follower redball_follower.launch.py

For Isaac Sim:
  1. Start Isaac Sim in a fresh terminal (no ROS sourced)
  2. Open welding_world.usda
  3. Press Play in Isaac Sim
  4. In a separate terminal: ros2-env && ros2 launch redball_follower redball_follower.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package paths
    pkg_share = get_package_share_directory('redball_follower')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    spawn_ball = LaunchConfiguration('spawn_ball', default='true')

    # URDF/Xacro file
    urdf_file = os.path.join(pkg_share, 'urdf', 'ur5e.urdf.xacro')

    # Robot description from xacro (no extra args — ur5e.urdf.xacro is self-contained)
    robot_description = Command(['xacro ', urdf_file])

    return LaunchDescription([
        # ================== Launch Arguments ==================
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock from Isaac Sim'
        ),
        DeclareLaunchArgument(
            'spawn_ball',
            default_value='true',
            description='Spawn the dummy red ball for testing'
        ),

        # ================== Robot State Publisher ==================
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(robot_description, value_type=str),
                'use_sim_time': use_sim_time
            }]
        ),

        # ================== Ball Tracker Node ==================
        # Processes D455 camera images to detect and track red ball
        # Topics subscribed directly: /d455/color/image_raw, /d455/depth/image_raw
        Node(
            package='redball_follower',
            executable='ball_tracker_node',
            name='ball_tracker_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'min_area': 150,
                'ema_alpha': 0.35,
                'depth_default': 0.8,
                'depth_scale': 0.001  # Isaac Sim depth is in meters (32FC1)
            }]
            # No remappings needed - code subscribes to /d455/* directly
        ),

        # ================== UR5e Velocity Controller ==================
        # IBVS controller with manipulability-aware cost function
        # Publishes Twist to /end_effector_velocity → Jacobian node does IK
        Node(
            package='redball_follower',
            executable='ur5e_velocity_controller_node',
            name='ur5e_velocity_controller_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'image_width': 1280,
                'image_height': 720,
                'fx': 600.0,
                'fy': 600.0,
                'depth_target': 0.5,       # desired standoff distance (m)
                'k_pixel_gain': 0.6,       # Kp: bearing → Cartesian velocity
                'k_depth_gain': 0.4,       # Kp: range error → Z velocity
                'kd_pixel_gain': 0.30,     # Kd: bearing derivative damping
                'kd_depth_gain': 0.10,     # Kd: range derivative damping
                'lost_timeout': 2.0,       # seconds before HOLDING mode
                'min_manipulability': 0.02,
                'w1_pixel': 1.0,           # pixel/bearing cost weight
                'w3_depth': 1.0,           # depth/range cost weight
                'max_linear_vel': 0.15,    # m/s  — Twist saturation
                'max_angular_vel': 0.30,   # rad/s — Twist saturation
            }]
        ),

        # ================== Jacobian Calculator ==================
        # Proper Jacobian-based IK: Twist → joint commands
        # Subscribes: /end_effector_velocity (Twist from velocity controller)
        # Publishes:  /isaac_joint_commands (JointState to Isaac Sim)
        # Cost function: J = w1*||bearing_error||² + w2*(1/μ) + w3*||range_error||²
        Node(
            package='redball_follower',
            executable='jacobian_calculator_node',
            name='jacobian_calculator_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': ParameterValue(robot_description, value_type=str),
                'robot_description_semantic': ParameterValue(
                    open(os.path.join(pkg_share, 'config', 'ur5e.srdf')).read(),
                    value_type=str),
                'planning_group': 'ur_manipulator',
                'end_effector_link': 'tool0',
                'control_mode': 'position',
                'joint_state_topic': '/isaac_joint_states',
                # Manipulability cost parameters
                'min_manipulability': 0.02,
                'w2_manipulability': 1.0,
                'manipulability_gain': 0.4,
                'damping_mu_reference': 0.05,
                'slowdown_mu_threshold': 0.04,
                'max_joint_velocity': 0.5,    # rad/s per joint clamp
                # Nullspace optimization
                'use_nullspace_posture': True,
                'posture_gain': 0.4,
            }]
        ),

        # ================== Red Ball Spawner (Dummy) ==================
        # Spawns a moving red ball for testing without Isaac Sim ball
        Node(
            package='redball_follower',
            executable='red_ball_spawner.py',
            name='red_ball_spawner',
            output='screen',
            condition=IfCondition(spawn_ball),
            parameters=[{
                'use_sim_time': use_sim_time,
                'radius': 0.05,
                'orbit_radius': 0.4,
                'orbit_speed': 0.3,
                'orbit_center_x': 0.5,
                'orbit_center_y': 0.0,
                'orbit_center_z': 0.5
            }]
        ),
    ])
