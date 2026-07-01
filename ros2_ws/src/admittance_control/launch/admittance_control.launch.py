"""Sand drawer action-mode launch file.

Launches the surface-drawing action stack for a UR5e in simulation and/or
real-hardware bridge mode.

Current behavior:
        - Uses drawing_action_server + drawing_dispatcher + TOTG service.
        - Assumes a single orthogonal surface-drawing tool.
        - The action server derives wrist orientation internally so the tool stays
            normal to the plane.
        - orthogonal_tool_length_m sets the plane-normal stand-off from wrist to tip.

Supported trajectory_key values:
    random | line | triangle | square | circle

Usage:
    ros2 launch admittance_control admittance_control.launch.py
    ros2 launch admittance_control admittance_control.launch.py real_robot:=true
    ros2 launch admittance_control admittance_control.launch.py continuous:=true
    ros2 launch admittance_control admittance_control.launch.py trajectory_key:=circle
    ros2 launch admittance_control admittance_control.launch.py orthogonal_tool_length_m:=0.13
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    """Resolve substitutions and build the action-mode node list."""

    pkg_share = get_package_share_directory("admittance_control")
    urdf_file = os.path.join(pkg_share, "urdf", "ur5e.urdf.xacro")

    workspace_root = (
        pkg_share.split("/install/admittance_control/share/admittance_control")[0]
        if "/install/admittance_control/share/admittance_control" in pkg_share
        else ""
    )
    default_plane_json = (
        os.path.join(workspace_root, "src", "admittance_control",
                     "generated_planes", "sand_drawer_plane.json")
        if workspace_root
        else os.path.join(pkg_share, "generated_planes",
                          "sand_drawer_plane.json")
    )

    # ---- resolve launch-time arguments ----
    use_sim_time_str = LaunchConfiguration("use_sim_time").perform(context)
    plane_json_str   = LaunchConfiguration("plane_json").perform(context)
    traj_key_str     = LaunchConfiguration("trajectory_key").perform(context)
    real_robot_str   = LaunchConfiguration("real_robot").perform(context)
    real_robot        = real_robot_str == "true"
    continuous_str    = LaunchConfiguration("continuous").perform(context)
    approach_height_str = LaunchConfiguration("approach_height").perform(context)
    surface_z_offset_str = LaunchConfiguration("surface_z_offset").perform(context)
    orthogonal_tool_length_str = LaunchConfiguration(
        "orthogonal_tool_length_m").perform(context)
    max_joint_speed_deg = float(
        LaunchConfiguration("max_joint_speed_deg").perform(context))
    max_joint_accel_deg = float(
        LaunchConfiguration("max_joint_accel_deg").perform(context))
    approach_height = float(approach_height_str)
    surface_z_offset = float(surface_z_offset_str)
    orthogonal_tool_length = float(orthogonal_tool_length_str)

    # When real_robot is active, force use_sim_time=false (wall clock)
    use_sim_time = False if real_robot else (use_sim_time_str == "true")

    from subprocess import check_output
    robot_description_str = check_output(
        ["xacro", urdf_file]).decode("utf-8")

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

    # ---- Action-based sequential drawing ----
    # trajectory_key values:
    #   'random'   — random geometric shape each time (default)
    #   'line'     — random line on the plane
    #   'triangle' — random equilateral triangle
    #   'square'   — random square
    #   'circle'   — random circle
    # Any legacy or unsupported key falls back to 'random'.
    action_supported_traj_keys = {
        "random", "line", "triangle", "square", "circle"
    }
    action_traj_key = traj_key_str
    if action_traj_key not in action_supported_traj_keys:
        action_traj_key = "random"

    nodes.append(Node(
        package="admittance_control",
        executable="drawing_action_server.py",
        name="drawing_action_server",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "plane_json_file": plane_json_str or default_plane_json,
            "approach_height": approach_height,
            "surface_z_offset": surface_z_offset,
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
            "totg_path_tolerance": 0.005,
            "totg_resample_dt": 0.01,
            "orthogonal_tool_length_m": orthogonal_tool_length,
        }],
    ))

    nodes.append(Node(
        package="admittance_control",
        executable="totg_service_node",
        name="totg_service_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    ))

    nodes.append(Node(
        package="admittance_control",
        executable="drawing_dispatcher.py",
        name="drawing_dispatcher",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "plane_json_file": plane_json_str or default_plane_json,
            "trajectory_key": action_traj_key,
            "continuous": continuous_str == "true",
            "orthogonal_tool_length_m": orthogonal_tool_length,
        }],
    ))

    return nodes


def generate_launch_description():
    pkg_share = get_package_share_directory("admittance_control")
    workspace_root = (
        pkg_share.split("/install/admittance_control/share/admittance_control")[0]
        if "/install/admittance_control/share/admittance_control" in pkg_share
        else ""
    )
    default_plane_json = (
        os.path.join(workspace_root, "src", "admittance_control",
                     "generated_planes", "sand_drawer_plane.json")
        if workspace_root
        else os.path.join(pkg_share, "generated_planes",
                          "sand_drawer_plane.json")
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time",   default_value="true"),
        DeclareLaunchArgument("plane_json",     default_value=default_plane_json),
        DeclareLaunchArgument("trajectory_key", default_value="random",
                              description="Action-mode drawing primitive: random | line | triangle | square | circle"),
        DeclareLaunchArgument("continuous",      default_value="false",
                              description="Continuous action-mode drawing dispatch"),
        DeclareLaunchArgument("approach_height", default_value="0.10",
                      description="Approach/ascent offset from drawing surface (meters)"),
        DeclareLaunchArgument("surface_z_offset", default_value="-0.01",
                  description="Drawing surface offset along -plane normal (meters)"),
        DeclareLaunchArgument("orthogonal_tool_length_m", default_value="0.13",
                      description="Orthogonal drawing tool length from wrist to tip (meters)"),
        DeclareLaunchArgument("real_robot",      default_value="false",
                              description="Enable bridging to real UR robot via JointTrajectory"),
        DeclareLaunchArgument("max_joint_speed_deg", default_value="90.0",
                              description="Maximum joint speed in deg/s (all joints)"),
        DeclareLaunchArgument("max_joint_accel_deg", default_value="40.0",
                              description="Maximum joint acceleration in deg/s² (all joints)"),
        OpaqueFunction(function=launch_setup),
    ])
