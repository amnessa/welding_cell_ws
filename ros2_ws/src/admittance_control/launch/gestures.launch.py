"""The hands-free operator console: gesture control + the finger-pointing camera aim.

    gesture_control_node   MediaPipe on the RealSense colour stream: gestures -> the
                           bridge / ICP services; the fingertip -> SAM2 clicks, and
                           before the capture -> /gesture/aim
    camera_aim_node        /gesture/aim -> the wrist camera turns to centre the
                           fingertip (forward_velocity_controller, switched by itself)

Start it next to pointcloud.launch.py (the camera, the bridge, the ICP node) and the
UR driver. Closing the gesture window ('q') shuts both down.

Usage
-----
    ros2 launch admittance_control gestures.launch.py                    # the arm aims (LIVE)
    ros2 launch admittance_control gestures.launch.py aim_dry_run:=true  # aim logged only
    ros2 launch admittance_control gestures.launch.py aim:=false         # gestures only
    ros2 launch admittance_control gestures.launch.py tracking_source:=icp

The aim is LIVE by default here, unlike `ros2 run ... camera_aim_node.py` (dry run by
default): this file is the everyday entry point, and the arm moving on Pointing_Up is
what it is for.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _default_model_path(pkg_share: str) -> str:
    """notebooks/gesture_recognizer.task from the source tree (it is not installed)."""
    marker = "/install/admittance_control/share/admittance_control"
    workspace_root = pkg_share.split(marker)[0] if marker in pkg_share else ""
    for root in ([os.path.join(workspace_root, "src", "admittance_control")] if workspace_root else []) + [pkg_share]:
        candidate = os.path.join(root, "notebooks", "gesture_recognizer.task")
        if os.path.exists(candidate):
            return candidate
    return "gesture_recognizer.task"


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("admittance_control")

    gesture = Node(
        package="admittance_control",
        executable="gesture_control_node.py",
        name="gesture_control",
        output="screen",
        parameters=[{
            "model_path": LaunchConfiguration("model_path"),
            "tracking_source": LaunchConfiguration("tracking_source"),
            "aim_enabled": LaunchConfiguration("aim"),
            "display_scale": LaunchConfiguration("display_scale"),
        }],
    )
    aim = Node(
        package="admittance_control",
        executable="camera_aim_node.py",
        name="camera_aim",
        output="screen",
        condition=IfCondition(LaunchConfiguration("aim")),
        parameters=[{
            "dry_run": PythonExpression(["'", LaunchConfiguration("aim_dry_run"), "'.lower() == 'true'"]),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("model_path", default_value=_default_model_path(pkg_share),
                              description="MediaPipe gesture_recognizer.task"),
        DeclareLaunchArgument("tracking_source", default_value="fp",
                              description="fp | icp - must match the ICP node's"),
        DeclareLaunchArgument("aim", default_value="true",
                              description="Pointing_Up before the capture aims the camera"),
        DeclareLaunchArgument("aim_dry_run", default_value="false",
                              description="true: the aim node only logs where it would turn"),
        DeclareLaunchArgument("display_scale", default_value="1.0",
                              description="gesture window scale (picking is in full-res pixels)"),
        gesture,
        aim,
        # 'q' in the gesture window ends the console: no aim node left behind
        RegisterEventHandler(OnProcessExit(target_action=gesture,
                                           on_exit=[EmitEvent(event=Shutdown(reason="gesture window closed"))])),
    ])
