"""Launch safe-probe target generation together with the 3D visualizer.

Usage:
    1. Run `ros2 run admittance_control segment_area.py` and inspect the segmentation.
  2. Run this launch file to compute safe probe points from the latest saved
     segmentation artifact bundle and show them in the GUI.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('admittance_control')
    default_plane_json = os.path.join(
        pkg_share, 'generated_planes', 'sand_drawer_plane.json')

    return LaunchDescription([
        DeclareLaunchArgument(
            'plane_json',
            default_value=default_plane_json,
            description='Plane JSON used by the drawing GUI for table visualization',
        ),
        DeclareLaunchArgument(
            'segment_run_dir',
            default_value='',
            description='Optional explicit segment artifact run directory; empty uses the latest run',
        ),
        DeclareLaunchArgument(
            'keep_alive',
            default_value='true',
            description='Keep depth_to_safe_probe running so its latched outputs stay available',
        ),
        Node(
            package='admittance_control',
            executable='depth_to_safe_probe.py',
            name='depth_to_safe_probe',
            output='screen',
            parameters=[{
                'segment_run_dir': LaunchConfiguration('segment_run_dir'),
                'keep_alive': LaunchConfiguration('keep_alive'),
            }],
        ),
        Node(
            package='admittance_control',
            executable='drawing_gui.py',
            name='drawing_gui',
            output='screen',
            arguments=[LaunchConfiguration('plane_json')],
        ),
    ])