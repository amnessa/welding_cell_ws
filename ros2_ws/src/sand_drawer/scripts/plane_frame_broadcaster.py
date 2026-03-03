#!/usr/bin/env python3
"""
Plane Frame Broadcaster — Static TF for the drawing plane.

Reads the plane definition from the JSON file produced by plane_solver_node
and broadcasts a static TF frame 'drawing_plane' relative to 'base_link'.

Frame convention (in drawing_plane):
  X axis → plane x_axis  (movement axis 1)
  Y axis → plane y_axis  (movement axis 2)
  Z axis → plane normal   (perpendicular to surface)
"""

import json
import math
import os

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros


def rotation_matrix_to_quaternion(R: np.ndarray):
    """Convert a 3×3 rotation matrix to quaternion [x, y, z, w]."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]

    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s

    return [x, y, z, w]


class PlaneFrameBroadcaster(Node):
    def __init__(self):
        super().__init__('plane_frame_broadcaster')

        self.declare_parameter('plane_json_file', '')
        self.declare_parameter('parent_frame', 'base_link')
        self.declare_parameter('child_frame', 'drawing_plane')

        json_file = self.get_parameter('plane_json_file').value
        parent_frame = self.get_parameter('parent_frame').value
        child_frame = self.get_parameter('child_frame').value

        if not json_file or not os.path.exists(json_file):
            self.get_logger().fatal(f'Plane JSON file not found: {json_file}')
            raise RuntimeError(f'Plane JSON file not found: {json_file}')

        with open(json_file, 'r') as f:
            data = json.load(f)

        plane = data['plane']
        origin = plane['origin']
        x_axis = np.array(plane['x_axis'], dtype=float)
        y_axis = np.array(plane['y_axis'], dtype=float)
        normal = np.array(plane['normal'], dtype=float)

        # Ensure orthonormality
        x_axis /= np.linalg.norm(x_axis)
        y_axis /= np.linalg.norm(y_axis)
        normal /= np.linalg.norm(normal)

        # Build rotation matrix: columns are the plane axes expressed in parent frame
        R = np.column_stack([x_axis, y_axis, normal])
        quat = rotation_matrix_to_quaternion(R)

        # Broadcast static transform
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = float(origin[0])
        t.transform.translation.y = float(origin[1])
        t.transform.translation.z = float(origin[2])
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.static_broadcaster.sendTransform(t)
        self.get_logger().info(
            f'Static TF published: {parent_frame} → {child_frame} '
            f'origin=[{origin[0]:.4f}, {origin[1]:.4f}, {origin[2]:.4f}] '
            f'quat=[{quat[0]:.4f}, {quat[1]:.4f}, {quat[2]:.4f}, {quat[3]:.4f}]')


def main(args=None):
    rclpy.init(args=args)
    node = PlaneFrameBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
