"""
Sand Drawer — Planar Servo Launch File

Launches the full pipeline for constrained plane servoing on a UR5e in Isaac Sim.

Modes (via mode:= launch argument):
  trajectory (default)  — follow waypoints from the plane JSON (velocity servo)
  teleop                — move EE to plane center, then accept keyboard commands
  cartesian             — position-controlled square drawing via Cartesian IK
  action                — action-based sequential drawing (server + dispatcher)
  capture               — run plane_solver_node to capture 4 points (sim, red ball)
  freedrive_capture     — capture 4 TCP poses on real robot via freedrive mode

Usage:
  ros2 launch sand_drawer sand_drawer.launch.py
  ros2 launch sand_drawer sand_drawer.launch.py mode:=teleop
  ros2 launch sand_drawer sand_drawer.launch.py mode:=cartesian real_robot:=true
  ros2 launch sand_drawer sand_drawer.launch.py mode:=cartesian loop:=true
  ros2 launch sand_drawer sand_drawer.launch.py mode:=action real_robot:=true
  ros2 launch sand_drawer sand_drawer.launch.py mode:=action continuous:=true
  ros2 launch sand_drawer sand_drawer.launch.py loop:=true
  ros2 launch sand_drawer sand_drawer.launch.py mode:=capture
  ros2 launch sand_drawer sand_drawer.launch.py mode:=freedrive_capture
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
    real_robot_str   = LaunchConfiguration("real_robot").perform(context)
    real_robot        = real_robot_str == "true"
    continuous_str    = LaunchConfiguration("continuous").perform(context)
    text_string_str   = LaunchConfiguration("text_string").perform(context)
    max_joint_speed_deg = float(
        LaunchConfiguration("max_joint_speed_deg").perform(context))
    max_joint_accel_deg = float(
        LaunchConfiguration("max_joint_accel_deg").perform(context))

    # When real_robot is active, force use_sim_time=false (wall clock)
    use_sim_time = False if real_robot else (use_sim_time_str == "true")

    from subprocess import check_output
    robot_description_str = check_output(
        ["xacro", urdf_file]).decode("utf-8")

    with open(srdf_file, "r") as f:
        srdf_str = f.read()

    nodes = []

    # ---- Robot State Publisher (always — needed so sim mirrors real robot) ----
    # When real_robot=true, listen to the real robot's /joint_states
    # so TF frames stay in sync with wall-clock timestamps.
    rsp_joint_remap = "/joint_states" if real_robot else "/isaac_joint_states"
    nodes.append(Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description_str,
            "use_sim_time": use_sim_time,
        }],
        remappings=[("joint_states", rsp_joint_remap)],
    ))

    if mode_str == "capture":
        # ---- Plane capture mode (simulation — uses red ball TF) ----
        nodes.append(Node(
            package="sand_drawer",
            executable="plane_solver_node.py",
            name="plane_solver_node",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "input_point_topic": "/red_ball/ground_truth",
                "source_frame": "world",
                "target_frame": "base_link",
                "output_file": plane_json_str or default_plane_json,
            }],
        ))
        return nodes

    if mode_str == "freedrive_capture":
        # ---- Plane capture on real robot (freedrive + TCP pose) ----
        nodes.append(Node(
            package="sand_drawer",
            executable="freedrive_plane_capture.py",
            name="freedrive_plane_capture",
            output="screen",
            prefix="xterm -e" if False else "",  # set True for separate terminal
            parameters=[{
                "output_file": plane_json_str or default_plane_json,
                "square_scale": 0.8,
                "tcp_pose_topic": "/tcp_pose_broadcaster/pose",
                "freedrive_controller": "freedrive_mode_controller",
                "trajectory_controller": "scaled_joint_trajectory_controller",
            }],
        ))
        return nodes

    if mode_str == "action":
        # ---- Action-based sequential drawing (server + dispatcher) ----
        # trajectory_key for action mode:
        #   'random'   — random geometric shape each time (default)
        #   'line'     — random line on the plane
        #   'triangle' — random equilateral triangle
        #   'square'   — random square
        #   'circle'   — random circle
        #   'text'     — render text_string as multi-stroke trajectory
        # If user didn't explicitly set it, default to 'random'.
        action_traj_key = traj_key_str if traj_key_str != "projected_vector_trajectory" else "random"
        nodes.append(Node(
            package="sand_drawer",
            executable="drawing_action_server.py",
            name="drawing_action_server",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "plane_json_file": plane_json_str or default_plane_json,
                "approach_height": 0.10,
                "max_linear_vel": 0.07,
                "max_linear_accel": 0.05,
                "approach_linear_vel": 0.04,
                "approach_linear_accel": 0.03,
                "ik_damping": 0.05,
                "execution_hz": 100.0,
                "max_joint_step": 0.15,
                "shoulder_lift_max": 0.0,
                "shoulder_lift_min": -2.5,
                "elbow_max": -0.3,
                "elbow_min": -3.14,
                "ik_num_seeds": 30,
                "real_robot": real_robot,
                "max_joint_speed_deg": max_joint_speed_deg,
                "max_joint_accel_deg": max_joint_accel_deg,
                "totg_path_tolerance": 0.1,
                "totg_resample_dt": 0.01,
            }],
        ))
        # TOTG service node (MoveIt 2 Time-Optimal Trajectory Generation)
        nodes.append(Node(
            package="sand_drawer",
            executable="totg_service_node",
            name="totg_service_node",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ))
        nodes.append(Node(
            package="sand_drawer",
            executable="drawing_dispatcher.py",
            name="drawing_dispatcher",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "plane_json_file": plane_json_str or default_plane_json,
                "trajectory_key": action_traj_key,
                "continuous": continuous_str == "true",
                "text_string": text_string_str,
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
                "use_sim_time": use_sim_time,
                "plane_json_file": plane_json_str or default_plane_json,
                "approach_height": 0.10,
                "max_linear_vel": 10.0,
                "max_linear_accel": 10.0,
                "approach_linear_vel": 10.0,
                "approach_linear_accel": 10.0,
                "ik_damping": 0.05,
                "execution_hz": 100.0,
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
                # Real robot
                "real_robot": real_robot,
                "max_joint_speed_deg": max_joint_speed_deg,
                "max_joint_accel_deg": max_joint_accel_deg,
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
            "use_sim_time": use_sim_time,
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
            "use_sim_time": use_sim_time,
            "plane_json_file": plane_json_str or default_plane_json,
            "ee_link": "tool0",
            "base_frame": "base_link",
            "approach_height": 0.10,
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
            "execution_hz": 100.0,
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
            # Real robot
            "real_robot": real_robot,
            "max_joint_speed_deg": max_joint_speed_deg,
            "max_joint_accel_deg": max_joint_accel_deg,
        }],
    ))

    # ---- Jacobian Calculator ----
    nodes.append(Node(
        package="sand_drawer",
        executable="sand_drawer_jacobian_node",
        name="jacobian_calculator_node",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
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
                              description="trajectory | teleop | cartesian | action | capture | freedrive_capture"),
        DeclareLaunchArgument("continuous",      default_value="false",
                              description="Continuous drawing dispatch (action mode)"),
        DeclareLaunchArgument("text_string",     default_value="ROMER",
                              description="Text string to draw (action mode, trajectory_key=text)"),
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
        DeclareLaunchArgument("real_robot",      default_value="false",
                              description="Enable bridging to real UR robot via JointTrajectory"),
        DeclareLaunchArgument("max_joint_speed_deg", default_value="90.0",
                              description="Maximum joint speed in deg/s (all joints)"),
        DeclareLaunchArgument("max_joint_accel_deg", default_value="40.0",
                              description="Maximum joint acceleration in deg/s² (all joints)"),
        OpaqueFunction(function=launch_setup),
    ])
