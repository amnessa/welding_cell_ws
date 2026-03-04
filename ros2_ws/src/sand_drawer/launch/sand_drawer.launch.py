"""
Sand Drawer — Planar Servo Launch File

Launches the full pipeline for constrained plane servoing on a UR5e in Isaac Sim.

Modes (via mode:= launch argument):
  trajectory (default) — follow waypoints from the plane JSON (velocity servo)
  teleop               — move EE to plane center, then accept keyboard commands
  cartesian            — position-controlled square drawing via Cartesian IK
  capture              — run plane_solver_node to capture 4 points

Usage:
  ros2 launch sand_drawer sand_drawer.launch.py
  ros2 launch sand_drawer sand_drawer.launch.py mode:=teleop
  ros2 launch sand_drawer sand_drawer.launch.py mode:=cartesian
  ros2 launch sand_drawer sand_drawer.launch.py mode:=cartesian loop:=true
  ros2 launch sand_drawer sand_drawer.launch.py loop:=true
  ros2 launch sand_drawer sand_drawer.launch.py mode:=capture
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    """Resolve all substitutions and build the node list based on mode."""

    pkg_share = get_package_share_directory("sand_drawer")
    urdf_file = os.path.join(pkg_share, "urdf", "ur5e.urdf.xacro")
    srdf_file = os.path.join(pkg_share, "config", "ur5e.srdf")

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

    # ---- resolve launch-time arguments ----
    use_sim_time_str = LaunchConfiguration("use_sim_time").perform(context)
    plane_json_str   = LaunchConfiguration("plane_json").perform(context)
    loop_str         = LaunchConfiguration("loop").perform(context)
    traj_key_str     = LaunchConfiguration("trajectory_key").perform(context)
    mode_str         = LaunchConfiguration("mode").perform(context)
    line_u_start_str = LaunchConfiguration("line_u_start").perform(context)
    line_v_start_str = LaunchConfiguration("line_v_start").perform(context)
    line_u_end_str   = LaunchConfiguration("line_u_end").perform(context)
    line_v_end_str   = LaunchConfiguration("line_v_end").perform(context)
    kp_linear_str    = LaunchConfiguration("kp_linear").perform(context)
    kd_linear_str    = LaunchConfiguration("kd_linear").perform(context)
    kp_angular_str   = LaunchConfiguration("kp_angular").perform(context)
    kd_angular_str   = LaunchConfiguration("kd_angular").perform(context)

    from subprocess import check_output
    robot_description_str = check_output(
        ["xacro", urdf_file]).decode("utf-8")

    with open(srdf_file, "r") as f:
        srdf_str = f.read()

    nodes = []

    # ---- Robot State Publisher (always) ----
    nodes.append(Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description_str,
            "use_sim_time": use_sim_time_str == "true",
        }],
        remappings=[("joint_states", "/isaac_joint_states")],
    ))

    if mode_str == "capture":
        # ---- Plane capture mode ----
        nodes.append(Node(
            package="sand_drawer",
            executable="plane_solver_node.py",
            name="plane_solver_node",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time_str == "true",
                "input_point_topic": "/red_ball/ground_truth",
                "source_frame": "world",
                "target_frame": "base_link",
                "output_file": plane_json_str or default_plane_json,
            }],
        ))
        return nodes

    if mode_str == "cartesian":
        # ---- Position-controlled Cartesian drawing ----
        # Default to 'line' unless user explicitly set a different trajectory_key
        cart_traj_key = traj_key_str if traj_key_str != "projected_vector_trajectory" else "line"
        nodes.append(Node(
            package="sand_drawer",
            executable="cartesian_square_controller.py",
            name="cartesian_draw_controller",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time_str == "true",
                "plane_json_file": plane_json_str or default_plane_json,
                "approach_height": 0.08,
                "max_linear_vel": 0.05,
                "max_linear_accel": 0.05,
                "ik_damping": 0.05,
                "execution_hz": 60.0,
                "waypoints_per_tick": 1,
                "loop_trajectory": loop_str == "true",
                "trajectory_key": cart_traj_key,
                "max_joint_step": 0.15,
                # Elbow-up constraints to prevent table collisions
                "shoulder_lift_max": 0.0,
                "shoulder_lift_min": -2.5,
                "elbow_max": -0.3,
                "elbow_min": -3.14,
                "ik_num_seeds": 30,
                # Line UV coordinates (used when trajectory_key='line')
                "line_u_start": float(line_u_start_str),
                "line_v_start": float(line_v_start_str),
                "line_u_end":   float(line_u_end_str),
                "line_v_end":   float(line_v_end_str),
            }],
        ))
        return nodes

    # ---- Plane Frame Broadcaster ----
    nodes.append(Node(
        package="sand_drawer",
        executable="plane_frame_broadcaster.py",
        name="plane_frame_broadcaster",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time_str == "true",
            "plane_json_file": plane_json_str or default_plane_json,
            "parent_frame": "base_link",
            "child_frame": "drawing_plane",
        }],
    ))

    is_teleop = mode_str == "teleop"

    # ---- Planar Servo Controller ----
    nodes.append(Node(
        package="sand_drawer",
        executable="planar_servo_controller.py",
        name="planar_servo_controller",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time_str == "true",
            "plane_json_file": plane_json_str or default_plane_json,
            "ee_link": "tool0",
            "base_frame": "base_link",
            "approach_height": 0.08,
            "waypoint_threshold": 0.03,
            "approach_threshold": 0.06,
            "orientation_threshold": 0.15,
            "kp_linear": float(kp_linear_str),
            "kd_linear": float(kd_linear_str),
            "kp_angular": float(kp_angular_str),
            "kd_angular": float(kd_angular_str),
            "max_linear_vel": 0.25,
            "max_angular_vel": 0.60,
            "plane_z_correction_gain": 2.0,
            "loop_trajectory": loop_str == "true",
            "trajectory_key": traj_key_str,
            "boundary_margin": 0.01,
            "teleop_mode": is_teleop,
            "teleop_speed": 0.10,
            "descent_step": 0.002,
            # Elbow-up configuration constraints
            "shoulder_lift_max": 0.0,
            "shoulder_lift_min": -2.5,
            "elbow_max": -0.3,
            "elbow_min": -3.14,
            "ik_num_seeds": 30,
            "ik_damping": 0.05,
            "max_joint_step": 0.15,
            # Line UV coordinates (used when trajectory_key='line')
            "line_u_start": float(line_u_start_str),
            "line_v_start": float(line_v_start_str),
            "line_u_end":   float(line_u_end_str),
            "line_v_end":   float(line_v_end_str),
        }],
    ))

    # ---- Jacobian Calculator ----
    nodes.append(Node(
        package="sand_drawer",
        executable="sand_drawer_jacobian_node",
        name="jacobian_calculator_node",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time_str == "true",
            "robot_description": robot_description_str,
            "robot_description_semantic": srdf_str,
            "planning_group": "ur_manipulator",
            "end_effector_link": "tool0",
            "control_mode": "position",
            "joint_state_topic": "/isaac_joint_states",
            "min_manipulability": 0.005,
            "w2_manipulability": 1.0,
            "manipulability_gain": 0.4,
            "damping_mu_reference": 0.02,
            "slowdown_mu_threshold": 0.01,
            "max_joint_velocity": 1.5,
            "use_nullspace_posture": True,
            "posture_gain": 0.4,
        }],
    ))

    return nodes


def generate_launch_description():
    pkg_share = get_package_share_directory("sand_drawer")
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

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time",   default_value="true"),
        DeclareLaunchArgument("plane_json",     default_value=default_plane_json),
        DeclareLaunchArgument("loop",           default_value="false"),
        DeclareLaunchArgument("trajectory_key", default_value="projected_vector_trajectory"),
        DeclareLaunchArgument("mode",           default_value="trajectory",
                              description="trajectory | teleop | cartesian | capture"),
        # Line UV coordinates (shared by cartesian & velocity modes)
        DeclareLaunchArgument("line_u_start",   default_value="0.5"),
        DeclareLaunchArgument("line_v_start",   default_value="0.3"),
        DeclareLaunchArgument("line_u_end",     default_value="0.5"),
        DeclareLaunchArgument("line_v_end",     default_value="0.7"),
        # PD gains (velocity controller)
        DeclareLaunchArgument("kp_linear",      default_value="1.5"),
        DeclareLaunchArgument("kd_linear",      default_value="0.0"),
        DeclareLaunchArgument("kp_angular",     default_value="1.5"),
        DeclareLaunchArgument("kd_angular",     default_value="0.0"),
        OpaqueFunction(function=launch_setup),
    ])
