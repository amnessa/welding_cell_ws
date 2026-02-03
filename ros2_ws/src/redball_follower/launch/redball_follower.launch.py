"""
Red Ball Follower - Main Launch File

This is the unified launcher for the UR5e red ball tracking system.
It launches all necessary nodes for:
- Robot state publisher (UR5e URDF)
- Ball tracker node (D435 camera vision)
- Velocity controller node (IBVS control)
- Jacobian calculator node (manipulability)
- Red ball spawner (dummy ball for testing)

Usage:
  ros2 launch redball_follower redball_follower.launch.py

For Isaac Sim:
  1. Start Isaac Sim manually
  2. Open/create your world with UR5e + D435 camera
  3. Run this launch file
  4. Press Play in Isaac Sim
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package paths
    pkg_share = get_package_share_directory('redball_follower')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    spawn_ball = LaunchConfiguration('spawn_ball', default='true')

    # URDF/Xacro file
    urdf_file = os.path.join(pkg_share, 'urdf', 'ur.urdf.xacro')

    # Robot description from xacro
    robot_description = Command([
        'xacro ', urdf_file,
        ' name:=ur5e',
        ' ur_type:=ur5e',
        ' prefix:=""',
        ' sim_isaac:=true'
    ])

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
                'robot_description': robot_description,
                'use_sim_time': use_sim_time
            }]
        ),

        # ================== Ball Tracker Node ==================
        # Processes D435 camera images to detect and track red ball
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
                'depth_scale': 0.001
            }],
            remappings=[
                ('/rsd455_img', '/d435/color/image_raw'),
                ('/rsd455_depth', '/d435/depth/image_raw'),
            ]
        ),

        # ================== UR5e Velocity Controller ==================
        # IBVS controller with manipulability-aware cost function
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
                'depth_default': 0.8,
                'depth_target': 0.5,
                'k_pixel_gain': 1.8,
                'k_depth_gain': 1.2,
                'lost_timeout': 0.5,
                'min_manipulability': 0.02,
                'w1_pixel': 2.5,
                'w3_depth': 1.5
            }]
        ),

        # ================== Jacobian Calculator ==================
        # Computes manipulability measure for cost function
        Node(
            package='redball_follower',
            executable='jacobian_calculator_node',
            name='jacobian_calculator_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time
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
