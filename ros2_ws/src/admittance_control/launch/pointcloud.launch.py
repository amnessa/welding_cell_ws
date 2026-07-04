"""Colored pointcloud from the RealSense RGB + aligned depth streams.

Turns the image topics published by ``realsense_camera_node.py`` into a single
``sensor_msgs/PointCloud2`` that RViz can render (like the ZED
``point_cloud/cloud_registered`` topic).

Pipeline
--------
    realsense_camera_node  ->  /camera/color/image_raw        (rgb8)
                               /camera/depth/image_rect_raw    (16UC1, mm, aligned)
                               /camera/color/camera_info
                                        |
                    depth_image_proc::PointCloudXyzrgbNode
                                        |
                               /camera/depth/color/points      (PointCloud2, XYZRGB)

The depth is already aligned to the color frame in the camera node, so the two
images share one CameraInfo — exactly what PointCloudXyzrgb needs. Depth is
16UC1 in millimetres, which the node handles natively.

Usage
-----
    # camera + pointcloud (default)
    ros2 launch admittance_control pointcloud.launch.py

    # pointcloud only (camera already running elsewhere)
    ros2 launch admittance_control pointcloud.launch.py launch_camera:=false

    # also open RViz with the cloud displayed
    ros2 launch admittance_control pointcloud.launch.py launch_rviz:=true

Then in RViz add a PointCloud2 display on /camera/depth/color/points and set the
Fixed Frame to camera_color_optical_frame.

Requires: sudo apt install ros-jazzy-depth-image-proc
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


# Topics published by realsense_camera_node.py (its parameter defaults).
RGB_TOPIC = "/camera/color/image_raw"
DEPTH_TOPIC = "/camera/depth/image_rect_raw"
CAMERA_INFO_TOPIC = "/camera/color/camera_info"
POINTS_TOPIC = "/camera/depth/color/points"


def generate_launch_description() -> LaunchDescription:
    launch_camera = LaunchConfiguration("launch_camera")
    launch_rviz = LaunchConfiguration("launch_rviz")

    camera_node = Node(
        package="admittance_control",
        executable="realsense_camera_node.py",
        name="realsense_camera",
        output="screen",
        condition=IfCondition(launch_camera),
    )

    # Composable container so the pointcloud node shares memory with any other
    # image_pipeline components (zero-copy intra-process transport).
    pointcloud_container = ComposableNodeContainer(
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
                # RViz is happier with a dense (organized) cloud; drop NaNs only
                # if you feed the cloud to a filter that dislikes them.
                parameters=[{"queue_size": 10}],
            ),
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "launch_camera", default_value="true",
            description="Also start realsense_camera_node.py."),
        DeclareLaunchArgument(
            "launch_rviz", default_value="false",
            description="Open RViz alongside the pointcloud node."),
        camera_node,
        pointcloud_container,
        rviz_node,
    ])
