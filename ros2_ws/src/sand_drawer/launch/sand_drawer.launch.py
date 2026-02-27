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

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    point_topic = LaunchConfiguration("point_topic", default="/red_ball/ground_truth")
    source_frame = LaunchConfiguration("source_frame", default="world")
    target_frame = LaunchConfiguration("target_frame", default="base_link")
    output_file = LaunchConfiguration("output_file", default="/tmp/sand_drawer_plane.json")

    robot_description = Command(["xacro ", urdf_file])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("point_topic", default_value="/red_ball/ground_truth"),
        DeclareLaunchArgument("source_frame", default_value="world"),
        DeclareLaunchArgument("target_frame", default_value="base_link"),
        DeclareLaunchArgument("output_file", default_value="/tmp/sand_drawer_plane.json"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": ParameterValue(robot_description, value_type=str),
                "use_sim_time": use_sim_time,
            }],
        ),

        Node(
            package="sand_drawer",
            executable="plane_solver_node.py",
            name="plane_solver_node",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "input_point_topic": point_topic,
                "source_frame": source_frame,
                "target_frame": target_frame,
                "output_file": output_file,
            }],
        ),
    ])
