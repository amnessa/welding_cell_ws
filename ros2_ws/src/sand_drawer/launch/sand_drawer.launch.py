"""
Sand Drawer — Planar Servo Launch File

Launches the full pipeline for constrained plane servoing on a UR5e in Isaac Sim.

Modes (via mode:= launch argument):
  trajectory (default) — follow waypoints from the plane JSON
  teleop               — move EE to plane center, then accept keyboard commands
  capture              — run plane_solver_node to capture 4 points

Usage:
  ros2 launch sand_drawer sand_drawer.launch.py
  ros2 launch sand_drawer sand_drawer.launch.py mode:=teleop
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
            "waypoint_threshold": 0.015,
            "approach_threshold": 0.025,
            "orientation_threshold": 0.15,
            "kp_linear": 0.5,
            "kp_angular": 1.0,
            "max_linear_vel": 0.10,
            "max_angular_vel": 0.30,
            "plane_z_correction_gain": 2.0,
            "loop_trajectory": loop_str == "true",
            "trajectory_key": traj_key_str,
            "boundary_margin": 0.01,
            "teleop_mode": is_teleop,
            "teleop_speed": 0.05,
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
            "min_manipulability": 0.02,
            "w2_manipulability": 1.0,
            "manipulability_gain": 0.4,
            "damping_mu_reference": 0.05,
            "slowdown_mu_threshold": 0.1,
            "max_joint_velocity": 0.5,
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
                              description="trajectory | teleop | capture"),
        OpaqueFunction(function=launch_setup),
    ])
