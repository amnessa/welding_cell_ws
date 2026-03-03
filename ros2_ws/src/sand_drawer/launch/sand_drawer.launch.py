"""
Sand Drawer — Planar Servo Launch File

Launches the full pipeline for constrained plane servoing on a UR5e in Isaac Sim:

  1. robot_state_publisher   – publishes URDF TF tree from /isaac_joint_states
  2. plane_frame_broadcaster – publishes static TF  base_link → drawing_plane
  3. planar_servo_controller – generates constrained Twist commands on the plane
  4. jacobian_calculator_node – converts Twist → joint commands via Jacobian IK

Isaac Sim topics expected:
  /isaac_joint_states   (sensor_msgs/JointState)  – robot state feedback
  /isaac_joint_commands (sensor_msgs/JointState)  – robot command input
  /clock               (rosgraph_msgs/Clock)      – sim time

Usage:
  ros2 launch sand_drawer sand_drawer.launch.py

  # With custom plane file:
  ros2 launch sand_drawer sand_drawer.launch.py plane_json:=/path/to/plane.json

  # Loop the trajectory:
  ros2 launch sand_drawer sand_drawer.launch.py loop:=true
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("sand_drawer")
    urdf_file = os.path.join(pkg_share, "urdf", "ur5e.urdf.xacro")
    srdf_file = os.path.join(pkg_share, "config", "ur5e.srdf")

    # ---- resolve default plane JSON path (prefer source tree) ----
    workspace_root = (
        pkg_share.split("/install/sand_drawer/share/sand_drawer")[0]
        if "/install/sand_drawer/share/sand_drawer" in pkg_share
        else ""
    )
    default_plane_json = (
        os.path.join(workspace_root, "src", "sand_drawer",
                     "generated_planes", "sand_drawer_plane.json")
        if workspace_root
        else os.path.join(pkg_share, "generated_planes",
                          "sand_drawer_plane.json")
    )

    # ---- launch-time arguments ----
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    plane_json   = LaunchConfiguration("plane_json",   default=default_plane_json)
    loop         = LaunchConfiguration("loop",         default="false")
    traj_key     = LaunchConfiguration("trajectory_key",
                                       default="projected_vector_trajectory")

    robot_description = Command(["xacro ", urdf_file])

    return LaunchDescription([

        # ================ Launch Arguments ================
        DeclareLaunchArgument("use_sim_time",    default_value="true"),
        DeclareLaunchArgument("plane_json",      default_value=default_plane_json),
        DeclareLaunchArgument("loop",            default_value="false"),
        DeclareLaunchArgument("trajectory_key",  default_value="projected_vector_trajectory"),

        # ================ Robot State Publisher ================
        # Reads /isaac_joint_states and publishes full TF tree for the UR5e
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": ParameterValue(robot_description,
                                                    value_type=str),
                "use_sim_time": use_sim_time,
            }],
            remappings=[
                ("joint_states", "/isaac_joint_states"),
            ],
        ),

        # ================ Plane Frame Broadcaster ================
        # Reads plane JSON and publishes static TF: base_link → drawing_plane
        Node(
            package="sand_drawer",
            executable="plane_frame_broadcaster.py",
            name="plane_frame_broadcaster",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "plane_json_file": plane_json,
                "parent_frame": "base_link",
                "child_frame": "drawing_plane",
            }],
        ),

        # ================ Planar Servo Controller ================
        # Generates constrained Twist on the drawing plane
        # Publishes: /end_effector_velocity  (geometry_msgs/Twist)
        Node(
            package="sand_drawer",
            executable="planar_servo_controller.py",
            name="planar_servo_controller",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "plane_json_file": plane_json,
                "ee_link": "tool0",
                "base_frame": "base_link",
                "approach_height": 0.08,
                "waypoint_threshold": 0.015,
                "approach_threshold": 0.025,
                "orientation_threshold": 0.15,
                "kp_linear": 0.5,
                "kp_angular": 1.0,
                "max_linear_vel": 0.10,
                "max_angular_vel": 0.30,
                "plane_z_correction_gain": 2.0,
                "loop_trajectory": loop,
                "trajectory_key": traj_key,
                "boundary_margin": 0.01,
            }],
        ),

        # ================ Jacobian Calculator ================
        # Subscribes: /end_effector_velocity (Twist)
        # Publishes:  /isaac_joint_commands  (JointState)
        # Uses damped pseudoinverse + nullspace manipulability optimisation
        Node(
            package="sand_drawer",
            executable="sand_drawer_jacobian_node",
            name="jacobian_calculator_node",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "robot_description": ParameterValue(robot_description,
                                                    value_type=str),
                "robot_description_semantic": ParameterValue(
                    open(srdf_file).read(), value_type=str),
                "planning_group": "ur_manipulator",
                "end_effector_link": "tool0",
                "control_mode": "position",
                "joint_state_topic": "/isaac_joint_states",
                # Manipulability / damping
                "min_manipulability": 0.02,
                "w2_manipulability": 1.0,
                "manipulability_gain": 0.4,
                "damping_mu_reference": 0.05,
                "slowdown_mu_threshold": 0.04,
                "max_joint_velocity": 0.5,
                # Nullspace
                "use_nullspace_posture": True,
                "posture_gain": 0.4,
            }],
        ),
    ])
