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


To send rgbd readings to other computer


ros2 launch admittance_control digital_twin_pointcloud.launch.py \
  launch_foundationpose:=true \
  foundationpose_server_url:=http://ip_to_that_pc:5000/predict_pose \
  launch_rviz:=true \
  detection_min_score:=0.1

  -------------------

  to tune ground removal height in base_link (z) for ICP, use:

  ros2 param set /icp_pose_refiner ground_z_m -0.10

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


def _default_rviz_config() -> str:
    """The saved RViz layout, as installed into share/.

    Shared with ``pointcloud.launch.py``: both launches publish the same topics
    in the same frames, so one layout serves the sim and the real cell.

    Note this resolves under ``install/``, not ``src/``: ``install(DIRECTORY
    launch ...)`` copies the file at build time, so a config edited in ``src/``
    does not take effect until the next ``colcon build`` -- unless the workspace
    was built with ``--symlink-install``, in which case it does immediately.
    """
    return os.path.join(get_package_share_directory("admittance_control"),
                        "launch", "point_cloud_config.rviz")


def _rviz_arguments(context) -> list:
    """``-d <config>``, or nothing if the config was cleared or is missing."""
    config = LaunchConfiguration("rviz_config").perform(context)
    if not config:
        return []
    if not os.path.exists(config):
        print(f"[digital_twin_pointcloud.launch.py] rviz_config {config} does not "
              f"exist; starting RViz with its default layout. If you just edited "
              f"the config under src/, rebuild the package.")
        return []
    return ["-d", config]


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("admittance_control")
    urdf_file = os.path.join(pkg_share, "urdf", "ur5e.urdf.xacro")

    launch_robot = LaunchConfiguration("launch_robot_state").perform(context) == "true"
    launch_rviz = LaunchConfiguration("launch_rviz")
    extrinsic_parent = LaunchConfiguration("extrinsic_parent").perform(context)
    extrinsic_path = LaunchConfiguration("extrinsic_path").perform(context) \
        or _default_extrinsic_path(pkg_share)

    # One directory for the whole pose handoff: the bridge writes detection_pem.json
    # / detection_ism.npz here, the ICP node seeds off them. Empty = let both nodes
    # use their (matching) defaults.
    pose_results_dir = LaunchConfiguration("pose_results_dir").perform(context)

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

    # ---- FoundationPose bridge (optional; needs the Flask pose server reachable) ----
    # Triggering it blocks until the operator clicks the object in the window the
    # server opens on the GPU host -- that click is what SAM2 turns into the mask.
    launch_foundationpose = \
        LaunchConfiguration("launch_foundationpose").perform(context) == "true"
    if launch_foundationpose:
        nodes.append(Node(
            package="admittance_control",
            executable="foundationpose_bridge_node.py",
            name="foundationpose_bridge",
            output="screen",
            parameters=[{
                "server_url":
                    LaunchConfiguration("foundationpose_server_url").perform(context),
                "camera_frame": CAMERA_FRAME,
                "request_timeout_sec":
                    float(LaunchConfiguration("foundationpose_timeout_sec").perform(context)),
                # Same value the ICP node gets below -- the bridge writes the
                # artifacts there and the ICP node reads them back, so the two
                # must never be set independently.
                "results_dir": pose_results_dir,
                "use_sim_time": True,
            }],
        ))

    # ---- Detection3DArray -> RViz MarkerArray (pose triads + labels + CAD box) ----
    # Must be the SAME CAD the pose server registers against (fp_server.py
    # MESH_PATH -- check it with `curl <server>/health`). The returned pose is a
    # transform into THAT mesh's local frame, so drawing a different mesh at it
    # renders a box in the wrong orientation even when the pose is perfect.
    bbox_model = LaunchConfiguration("bbox_model_path").perform(context) \
        or os.path.join(pkg_share, "models", "test_objv2_base.ply")
    nodes.append(Node(
        package="admittance_control",
        executable="detection_marker_node.py",
        name="detection_marker",
        output="screen",
        parameters=[{
            "min_score": float(LaunchConfiguration("detection_min_score").perform(context)),
            "bbox_model_path": bbox_model,
            "bbox_model_units": LaunchConfiguration("bbox_model_units").perform(context),
            "use_sim_time": True,
        }],
    ))

    # ---- ICP pose refiner (segmented scene cloud vs CAD model at the detected pose) ----
    # The ICP object is chosen with its own arg so it can differ from the bbox
    # marker; it MUST match the CAD the pose server runs (fp_server.py MESH_PATH).
    icp_model = LaunchConfiguration("icp_model_path").perform(context) or bbox_model
    if LaunchConfiguration("launch_icp").perform(context) == "true":
        nodes.append(Node(
            package="admittance_control",
            executable="icp_pose_refiner_node.py",
            name="icp_pose_refiner",
            output="screen",
            parameters=[{
                "model_path": icp_model,
                "model_units": LaunchConfiguration("bbox_model_units").perform(context),
                "results_dir": pose_results_dir,
                "scene_from": LaunchConfiguration("icp_scene_from").perform(context),
                "camera_frame": CAMERA_FRAME,
                "anderson_depth": int(LaunchConfiguration("icp_anderson_depth").perform(context)),
                "use_open3d": LaunchConfiguration("icp_use_open3d").perform(context) == "true",
                "crop_margin_m": float(LaunchConfiguration("icp_crop_margin_m").perform(context)),
                "tracking_rate_hz": float(LaunchConfiguration("icp_tracking_rate_hz").perform(context)),
                "auto_track": LaunchConfiguration("icp_auto_track").perform(context) == "true",
                "ground_removal": LaunchConfiguration("icp_ground_removal").perform(context) == "true",
                "ground_z_m": float(LaunchConfiguration("icp_ground_z_m").perform(context)),
                "use_sim_time": True,
            }],
        ))

    nodes.append(Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=_rviz_arguments(context),
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
            "rviz_config", default_value=_default_rviz_config(),
            description="RViz layout to open with (-d). Defaults to the saved "
                        "point_cloud_config.rviz shipped in the package. Pass a "
                        "path to use your own, or rviz_config:='' for a bare RViz."),
        DeclareLaunchArgument(
            "extrinsic_parent", default_value="tool0",
            description="Parent frame of the camera extrinsic (tool0 or flange)."),
        DeclareLaunchArgument(
            "extrinsic_path", default_value="",
            description="Path to the 4x4 hand-eye transform (.npy). "
                        "Empty = notebooks/T_tcp_to_cam.npy."),
        DeclareLaunchArgument(
            "launch_foundationpose", default_value="false",
            description="Also start the FoundationPose bridge (needs the Flask pose "
                        "server reachable at foundationpose_server_url)."),
        DeclareLaunchArgument(
            "foundationpose_server_url",
            default_value="http://127.0.0.1:5000/predict_pose",
            description="FoundationPose Flask /predict_pose endpoint (fp_server.py)."),
        DeclareLaunchArgument(
            "foundationpose_timeout_sec", default_value="300.0",
            description="How long the bridge waits for the server's reply. The "
                        "server blocks on an operator clicking the object, so this "
                        "is a human-patience timeout, not a compute one."),
        DeclareLaunchArgument(
            "pose_results_dir", default_value="",
            description="Where the bridge writes the server's artifacts and the ICP "
                        "node reads them from -- one arg feeds both, so they cannot "
                        "drift apart. Empty = scripts/foundationpose_results. Point "
                        "it at scripts/sam6d_results to replay an old SAM-6D capture."),
        DeclareLaunchArgument(
            "detection_min_score", default_value="0.0",
            description="Drop detections below this score in the RViz markers."),
        DeclareLaunchArgument(
            "bbox_model_path", default_value="",
            description="CAD .ply for the oriented box on the best detection. "
                        "MUST be the mesh the pose server registers against "
                        "(fp_server.py MESH_PATH), or the box is drawn in the "
                        "wrong orientation. Empty = models/test_objv2_base.ply."),
        DeclareLaunchArgument(
            "bbox_model_units", default_value="mm",
            description="Units of the CAD model vertices (mm or m)."),
        DeclareLaunchArgument(
            "launch_icp", default_value="true",
            description="Start the ICP pose refiner (call its ~/run_icp service)."),
        DeclareLaunchArgument(
            "icp_model_path", default_value="",
            description="CAD .ply the ICP node tracks/renders (the green model "
                        "cloud). MUST be the same object the pose server "
                        "detects (fp_server.py MESH_PATH). Empty = fall back to "
                        "bbox_model_path (models/test_objv2_base.ply). For the "
                        "second part of the assembly, switch BOTH sides: set the "
                        "server's MESH_PATH to the ear, then: ros2 param set "
                        "/icp_pose_refiner model_path <...>/test_objv2_ear.ply "
                        "and re-run ~/run_icp."),
        DeclareLaunchArgument(
            "icp_scene_from", default_value="pointcloud",
            description="ICP scene source for the Phase-1 init: 'pointcloud' "
                        "(live cloud) or 'depth_png' (the frame the pose server saw). "
                        "The Phase-2 tracking loop always uses the live cloud."),
        DeclareLaunchArgument(
            "icp_anderson_depth", default_value="5",
            description="Fast-ICP Anderson acceleration history depth "
                        "(0 = plain point-to-plane Gauss-Newton)."),
        DeclareLaunchArgument(
            "icp_use_open3d", default_value="true",
            description="Use Open3D for per-frame voxel downsample + normals on "
                        "the cropped cloud (faster). Falls back to NumPy if "
                        "Open3D is not installed."),
        DeclareLaunchArgument(
            "icp_crop_margin_m", default_value="0.03",
            description="Margin added to the model AABB for the dynamic "
                        "CropBox in the tracking loop (metres)."),
        DeclareLaunchArgument(
            "icp_tracking_rate_hz", default_value="15.0",
            description="Rate of the Phase-2 tracking loop (Hz)."),
        DeclareLaunchArgument(
            "icp_auto_track", default_value="true",
            description="Start tracking automatically after the ~/run_icp seed."),
        DeclareLaunchArgument(
            "icp_ground_removal", default_value="true",
            description="Delete the floor/table before ICP by hard Z-truncation "
                        "in base_link (needs the base_link<-camera TF)."),
        DeclareLaunchArgument(
            "icp_ground_z_m", default_value="-0.10",
            description="Ground plane height in base_link: points with z <= this "
                        "are dropped. Sim floor and the real bench differ, so "
                        "tune this per setup. Tune live with: ros2 param set "
                        "/icp_pose_refiner ground_z_m <value>."),
        OpaqueFunction(function=launch_setup),
    ])
