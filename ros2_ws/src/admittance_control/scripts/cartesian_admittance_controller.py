#!/usr/bin/env python3
"""ROS-native Cartesian admittance controller for surface following.

This node consumes a nominal Cartesian path, listens to the force/torque wrench
topic, and publishes TwistStamped commands to MoveIt Servo. X/Y track the
nominal path aggressively, while Z is corrected by a 1-DoF admittance model so
the tool can maintain target contact force over a non-flat surface.
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import TwistStamped, WrenchStamped
from nav_msgs.msg import Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from tf2_ros import TransformException


def axis_angle_to_quat(axis_angle: Sequence[float]) -> np.ndarray:
    vector = np.asarray(axis_angle, dtype=float)
    angle = float(np.linalg.norm(vector))
    if angle <= 1e-9:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    axis = vector / angle
    sin_half = math.sin(0.5 * angle)
    return np.array([
        axis[0] * sin_half,
        axis[1] * sin_half,
        axis[2] * sin_half,
        math.cos(0.5 * angle),
    ], dtype=float)


def quat_to_rotmat(q_xyzw: Sequence[float]) -> np.ndarray:
    x_val, y_val, z_val, w_val = q_xyzw
    return np.array([
        [1 - 2 * (y_val * y_val + z_val * z_val), 2 * (x_val * y_val - z_val * w_val), 2 * (x_val * z_val + y_val * w_val)],
        [2 * (x_val * y_val + z_val * w_val), 1 - 2 * (x_val * x_val + z_val * z_val), 2 * (y_val * z_val - x_val * w_val)],
        [2 * (x_val * z_val - y_val * w_val), 2 * (y_val * z_val + x_val * w_val), 1 - 2 * (x_val * x_val + y_val * y_val)],
    ], dtype=float)


def orientation_error_aa(q_current_xyzw: Sequence[float], q_target_xyzw: Sequence[float]) -> np.ndarray:
    rotation_current = quat_to_rotmat(q_current_xyzw)
    rotation_target = quat_to_rotmat(q_target_xyzw)
    rotation_error = rotation_target @ rotation_current.T
    trace = float(rotation_error[0, 0] + rotation_error[1, 1] + rotation_error[2, 2])
    cos_angle = float(np.clip((trace - 1.0) / 2.0, -1.0, 1.0))
    angle = math.acos(cos_angle)
    if abs(angle) < 1e-6:
        return np.zeros(3, dtype=float)

    axis = np.array([
        rotation_error[2, 1] - rotation_error[1, 2],
        rotation_error[0, 2] - rotation_error[2, 0],
        rotation_error[1, 0] - rotation_error[0, 1],
    ], dtype=float)
    axis_norm = float(np.linalg.norm(axis))
    if axis_norm < 1e-8:
        return np.zeros(3, dtype=float)
    axis = axis / axis_norm
    return angle * axis


def clamp_vector(vector: np.ndarray, max_magnitude: float) -> np.ndarray:
    magnitude = float(np.linalg.norm(vector))
    if magnitude > max_magnitude and magnitude > 1e-9:
        return vector * (max_magnitude / magnitude)
    return vector


def clamp_scalar(value: float, lower: float, upper: float) -> float:
    return float(min(max(value, lower), upper))


def parse_float_sequence(
    value: Sequence[float] | str,
    expected_length: int,
    parameter_name: str,
) -> List[float]:
    if isinstance(value, str):
        stripped = value.strip()
        if stripped.startswith('[') and stripped.endswith(']'):
            stripped = stripped[1:-1]
        parsed = [float(item.strip()) for item in stripped.split(',') if item.strip()]
    else:
        parsed = [float(item) for item in value]

    if len(parsed) != expected_length:
        raise ValueError(
            f'Parameter {parameter_name!r} must contain {expected_length} values, got {len(parsed)}: {parsed}')
    return parsed


class CartesianAdmittanceController(Node):
    def __init__(self):
        super().__init__('cartesian_admittance_controller')

        self.declare_parameter('path_topic', '/visualizer/drawing_path')
        self.declare_parameter('wrench_topic', '/force_torque_sensor_broadcaster/wrench')
        self.declare_parameter('servo_twist_topic', '/servo_node/delta_twist_cmds')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('ee_link', 'tool0')
        self.declare_parameter('command_frame', 'base_link')
        self.declare_parameter('control_hz', 100.0)
        self.declare_parameter('target_force_z', -3.0)
        self.declare_parameter('admittance_mass_z', 1.0)
        self.declare_parameter('admittance_damping_z', 30.0)
        self.declare_parameter('admittance_stiffness_z', 0.0)
        self.declare_parameter('xy_position_gain', 6.0)
        self.declare_parameter('z_position_gain', 4.0)
        self.declare_parameter('angular_gain', 3.0)
        self.declare_parameter('max_xy_speed', 0.03)
        self.declare_parameter('max_z_speed', 0.02)
        self.declare_parameter('max_angular_speed', 0.30)
        self.declare_parameter('max_z_offset', 0.02)
        self.declare_parameter('waypoint_xy_tolerance', 0.003)
        self.declare_parameter('waypoint_z_tolerance', 0.005)
        self.declare_parameter('target_orientation_axis_angle', [3.14159, 0.0, 0.0])
        self.declare_parameter('loop_path', False)
        self.declare_parameter('publish_zero_when_idle', True)
        self.declare_parameter('force_low_pass_alpha', 0.2)

        self.path_topic = str(self.get_parameter('path_topic').value)
        self.wrench_topic = str(self.get_parameter('wrench_topic').value)
        self.servo_twist_topic = str(self.get_parameter('servo_twist_topic').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.ee_link = str(self.get_parameter('ee_link').value)
        self.command_frame = str(self.get_parameter('command_frame').value)
        self.control_hz = float(self.get_parameter('control_hz').value)
        self.target_force_z = float(self.get_parameter('target_force_z').value)
        self.mass_z = max(float(self.get_parameter('admittance_mass_z').value), 1e-4)
        self.damping_z = float(self.get_parameter('admittance_damping_z').value)
        self.stiffness_z = float(self.get_parameter('admittance_stiffness_z').value)
        self.xy_gain = float(self.get_parameter('xy_position_gain').value)
        self.z_gain = float(self.get_parameter('z_position_gain').value)
        self.angular_gain = float(self.get_parameter('angular_gain').value)
        self.max_xy_speed = float(self.get_parameter('max_xy_speed').value)
        self.max_z_speed = float(self.get_parameter('max_z_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.max_z_offset = float(self.get_parameter('max_z_offset').value)
        self.waypoint_xy_tolerance = float(self.get_parameter('waypoint_xy_tolerance').value)
        self.waypoint_z_tolerance = float(self.get_parameter('waypoint_z_tolerance').value)
        self.target_orientation = axis_angle_to_quat(
            parse_float_sequence(
                self.get_parameter('target_orientation_axis_angle').value,
                3,
                'target_orientation_axis_angle',
            ))
        self.loop_path = bool(self.get_parameter('loop_path').value)
        self.publish_zero_when_idle = bool(self.get_parameter('publish_zero_when_idle').value)
        self.force_low_pass_alpha = clamp_scalar(
            float(self.get_parameter('force_low_pass_alpha').value), 0.0, 1.0)

        path_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._path_sub = self.create_subscription(
            NavPath, self.path_topic, self._path_cb, path_qos)
        self._wrench_sub = self.create_subscription(
            WrenchStamped, self.wrench_topic, self._wrench_cb, 10)
        self._twist_pub = self.create_publisher(
            TwistStamped, self.servo_twist_topic, 10)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._path_points: List[np.ndarray] = []
        self._waypoint_idx = 0
        self._filtered_force_z: Optional[float] = None
        self._z_offset = 0.0
        self._z_velocity = 0.0
        self._last_loop_time: Optional[float] = None
        self._last_status_time: Optional[float] = None

        self._timer = self.create_timer(1.0 / self.control_hz, self._control_loop)

        self.get_logger().info(
            'Cartesian admittance controller started: '
            f'path_topic={self.path_topic}, wrench_topic={self.wrench_topic}, '
            f'servo_twist_topic={self.servo_twist_topic}, target_force_z={self.target_force_z:.3f} N')

    def _path_cb(self, msg: NavPath):
        self._path_points = [
            np.array([
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            ], dtype=float)
            for pose in msg.poses
        ]
        self._waypoint_idx = 0
        self._z_offset = 0.0
        self._z_velocity = 0.0

        if self._path_points:
            self.get_logger().info(
                f'Received path with {len(self._path_points)} waypoints from {self.path_topic}')
        else:
            self.get_logger().warn(f'Received empty path on {self.path_topic}')

    def _wrench_cb(self, msg: WrenchStamped):
        measured_force_z = float(msg.wrench.force.z)
        if self._filtered_force_z is None:
            self._filtered_force_z = measured_force_z
            return

        alpha = self.force_low_pass_alpha
        self._filtered_force_z = alpha * measured_force_z + (1.0 - alpha) * self._filtered_force_z

    def _lookup_current_pose(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        try:
            transform = self._tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_link,
                Time(),
            )
        except TransformException as exc:
            self.get_logger().debug(
                f'Failed to look up transform {self.base_frame} -> {self.ee_link}: {exc}')
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        position = np.array([translation.x, translation.y, translation.z], dtype=float)
        quaternion = np.array([rotation.x, rotation.y, rotation.z, rotation.w], dtype=float)
        return position, quaternion

    def _publish_zero_twist(self):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = self.command_frame
        self._twist_pub.publish(twist_msg)

    def _advance_waypoint_if_reached(self, current_position: np.ndarray):
        while self._path_points and self._waypoint_idx < len(self._path_points):
            target = self._path_points[self._waypoint_idx]
            xy_error = float(np.linalg.norm(target[:2] - current_position[:2]))
            z_error = abs(float(target[2] - current_position[2]))
            if xy_error > self.waypoint_xy_tolerance or z_error > self.waypoint_z_tolerance:
                return
            self._waypoint_idx += 1

        if self._path_points and self._waypoint_idx >= len(self._path_points):
            if self.loop_path:
                self._waypoint_idx = 0
            else:
                self._path_points = []

    def _control_loop(self):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if self._last_loop_time is None:
            self._last_loop_time = now_sec
            return

        dt = max(now_sec - self._last_loop_time, 1.0 / max(self.control_hz, 1.0))
        self._last_loop_time = now_sec

        if not self._path_points:
            if self.publish_zero_when_idle:
                self._publish_zero_twist()
            return

        pose = self._lookup_current_pose()
        if pose is None:
            if self.publish_zero_when_idle:
                self._publish_zero_twist()
            return

        current_position, current_quaternion = pose
        self._advance_waypoint_if_reached(current_position)

        if not self._path_points:
            if self.publish_zero_when_idle:
                self._publish_zero_twist()
            return

        target_position = self._path_points[self._waypoint_idx]

        measured_force_z = self._filtered_force_z if self._filtered_force_z is not None else 0.0
        force_error = self.target_force_z - measured_force_z
        z_acceleration = (
            force_error
            - (self.damping_z * self._z_velocity)
            - (self.stiffness_z * self._z_offset)
        ) / self.mass_z
        self._z_velocity += z_acceleration * dt
        self._z_velocity = clamp_scalar(self._z_velocity, -self.max_z_speed, self.max_z_speed)
        self._z_offset += self._z_velocity * dt
        self._z_offset = clamp_scalar(self._z_offset, -self.max_z_offset, self.max_z_offset)

        desired_position = target_position.copy()
        desired_position[2] += self._z_offset

        position_error = desired_position - current_position
        xy_velocity = self.xy_gain * position_error[:2]
        xy_velocity = clamp_vector(xy_velocity, self.max_xy_speed)

        z_velocity_cmd = (self.z_gain * position_error[2]) + self._z_velocity
        z_velocity_cmd = clamp_scalar(z_velocity_cmd, -self.max_z_speed, self.max_z_speed)

        angular_error = orientation_error_aa(current_quaternion, self.target_orientation)
        angular_velocity = self.angular_gain * angular_error
        angular_velocity = clamp_vector(angular_velocity, self.max_angular_speed)

        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = self.command_frame
        twist_msg.twist.linear.x = float(xy_velocity[0])
        twist_msg.twist.linear.y = float(xy_velocity[1])
        twist_msg.twist.linear.z = float(z_velocity_cmd)
        twist_msg.twist.angular.x = float(angular_velocity[0])
        twist_msg.twist.angular.y = float(angular_velocity[1])
        twist_msg.twist.angular.z = float(angular_velocity[2])
        self._twist_pub.publish(twist_msg)

        if self._last_status_time is None or (now_sec - self._last_status_time) >= 1.0:
            self._last_status_time = now_sec
            self.get_logger().info(
                f'wp={self._waypoint_idx + 1}/{len(self._path_points)}, '
                f'force_z={measured_force_z:.3f} N, force_error={force_error:.3f} N, '
                f'z_offset={self._z_offset:.4f} m')


def main(args=None):
    rclpy.init(args=args)
    node = CartesianAdmittanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.publish_zero_when_idle:
            try:
                node._publish_zero_twist()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()