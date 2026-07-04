"""Digital-twin colored pointcloud (Isaac Sim UR5e + eye-in-hand RealSense).

Isaac Sim publishes clean RGB-D on the ``*_sim`` topics but — unlike the ZED
example — provides no pointcloud, no CameraInfo, and stamps the images with an
empty frame_id that is not in the robot TF tree. RViz therefore shows nothing
when you subscribe to the (nonexistent) cloud, and even the raw images can't be
placed in a ``base_link``/``world`` fixed frame.

This launch assembles everything needed to render the cloud in a robot frame:

    robot_state_publisher  (/isaac_joint_states)
        base_link -> ... -> wrist_3_link -> flange -> tool0          [from URDF]

    camera_extrinsic_tf_publisher (T_tcp_to_cam.npy)
        tool0 -> camera_color_optical_frame                          [static]

    realsense_sim_camera_node  (extended)
        - relays *_sim -> /camera/color|depth/... with depth noise
        - rewrites frame_id -> camera_color_optical_frame
        - synthesizes /camera/color/camera_info (twin shares the real intrinsics)

    depth_image_proc::PointCloudXyzrgbNode
        /camera/depth/color/points  (PointCloud2, XYZRGB)  in camera_color_optical_frame

With the TF chain complete, set the RViz Fixed Frame to ``base_link`` (or
``world``) and add a PointCloud2 display on ``/camera/depth/color/points``.

Usage
-----
    ros2 launch admittance_control digital_twin_pointcloud.launch.py
    ros2 launch admittance_control digital_twin_pointcloud.launch.py launch_rviz:=true
    # if your calibrated camera frame hangs off flange instead of tool0:
    ros2 launch admittance_control digital_twin_pointcloud.launch.py extrinsic_parent:=flange

Requires: sudo apt install ros-jazzy-depth-image-proc
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


# Optical frame the cloud is expressed in; the static extrinsic connects it to
# the arm, so it must match the sim node's output_frame_id AND the extrinsic
# publisher's child_frame.
CAMERA_FRAME = "camera_color_optical_frame"

RGB_TOPIC = "/camera/color/image_raw"
DEPTH_TOPIC = "/camera/depth/image_rect_raw"
CAMERA_INFO_TOPIC = "/camera/color/camera_info"
POINTS_TOPIC = "/camera/depth/color/points"


def _default_extrinsic_path(pkg_share: str) -> str:
    """Resolve T_tcp_to_cam.npy from the source tree (it lives under notebooks/).

    Mirrors the workspace_root trick used in admittance_control.launch.py so the
    file is found whether we run from a symlink-install or a plain install.
    """
    marker = "/install/admittance_control/share/admittance_control"
    workspace_root = pkg_share.split(marker)[0] if marker in pkg_share else ""
    if workspace_root:
        candidate = os.path.join(
            workspace_root, "src", "admittance_control",
            "notebooks", "T_tcp_to_cam.npy")
        if os.path.exists(candidate):
            return candidate
    return os.path.join(pkg_share, "notebooks", "T_tcp_to_cam.npy")


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("admittance_control")
    urdf_file = os.path.join(pkg_share, "urdf", "ur5e.urdf.xacro")

    launch_robot = LaunchConfiguration("launch_robot_state").perform(context) == "true"
    launch_rviz = LaunchConfiguration("launch_rviz")
    extrinsic_parent = LaunchConfiguration("extrinsic_parent").perform(context)
    extrinsic_path = LaunchConfiguration("extrinsic_path").perform(context) \
        or _default_extrinsic_path(pkg_share)

    nodes = []

    # ---- Robot TF: base_link -> ... -> tool0 from Isaac joint states ----
    if launch_robot:
        from subprocess import check_output
        robot_description_str = check_output(["xacro", urdf_file]).decode("utf-8")
        nodes.append(Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description_str,
                "use_sim_time": True,
            }],
            remappings=[("joint_states", "/isaac_joint_states")],
        ))

    # ---- Static eye-in-hand extrinsic: tool0 -> camera_color_optical_frame ----
    nodes.append(Node(
        package="admittance_control",
        executable="camera_extrinsic_tf_publisher.py",
        name="camera_extrinsic_tf_publisher",
        output="screen",
        parameters=[{
            "extrinsic_path": extrinsic_path,
            "parent_frame": extrinsic_parent,
            "child_frame": CAMERA_FRAME,
            "use_sim_time": True,
        }],
    ))

    # ---- Sim camera relay: noise + frame_id rewrite + CameraInfo ----
    nodes.append(Node(
        package="admittance_control",
        executable="realsense_sim_camera_node.py",
        name="realsense_sim_camera",
        output="screen",
        parameters=[{
            "output_frame_id": CAMERA_FRAME,
            "publish_camera_info": True,
            "camera_info_topic": CAMERA_INFO_TOPIC,
            "use_sim_time": True,
        }],
    ))

    # ---- RGB + depth -> colored PointCloud2 ----
    nodes.append(ComposableNodeContainer(
        name="pointcloud_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="depth_image_proc",
                plugin="depth_image_proc::PointCloudXyzrgbNode",
                name="point_cloud_xyzrgb",
                remappings=[
                    ("rgb/image_rect_color", RGB_TOPIC),
                    ("rgb/camera_info", CAMERA_INFO_TOPIC),
                    ("depth_registered/image_rect", DEPTH_TOPIC),
                    ("points", POINTS_TOPIC),
                ],
                parameters=[{"queue_size": 10, "use_sim_time": True}],
            ),
        ],
    ))

    nodes.append(Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(launch_rviz),
    ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "launch_robot_state", default_value="true",
            description="Start robot_state_publisher on /isaac_joint_states. "
                        "Set false if the main admittance_control launch already runs it."),
        DeclareLaunchArgument(
            "launch_rviz", default_value="false",
            description="Open RViz alongside the pointcloud pipeline."),
        DeclareLaunchArgument(
            "extrinsic_parent", default_value="tool0",
            description="Parent frame of the camera extrinsic (tool0 or flange)."),
        DeclareLaunchArgument(
            "extrinsic_path", default_value="",
            description="Path to the 4x4 hand-eye transform (.npy). "
                        "Empty = notebooks/T_tcp_to_cam.npy."),
        OpaqueFunction(function=launch_setup),
    ])
