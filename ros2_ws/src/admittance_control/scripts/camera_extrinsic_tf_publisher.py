#!/usr/bin/env python3
"""Publish the eye-in-hand camera extrinsic as a static TF.

Loads the hand-eye calibration result ``T_tool0_camera.npy`` (a 4x4 homogeneous
transform giving the camera optical frame expressed in the tool0/flange frame)
and broadcasts it once as a static transform ``tool0 -> camera_color_optical_frame``.

With this transform in the tree, and robot_state_publisher providing
``base_link -> tool0`` from ``/joint_states``, any node can look up
``base_link -> camera_color_optical_frame`` at any time via tf2 — which is how
the perception pipeline converts SAM-6D poses (camera frame) into base_link.

Parameters
----------
  extrinsic_path : str   path to T_tool0_camera.npy (4x4)
  parent_frame   : str   default 'tool0'
  child_frame    : str   default 'camera_color_optical_frame'
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster

from admittance_control.geometry import rotmat_to_quat


class CameraExtrinsicTfPublisher(Node):
    def __init__(self) -> None:
        super().__init__('camera_extrinsic_tf_publisher')

        self.declare_parameter('extrinsic_path', '')
        self.declare_parameter('parent_frame', 'tool0')
        self.declare_parameter('child_frame', 'camera_color_optical_frame')

        extrinsic_path = str(self.get_parameter('extrinsic_path').value)
        parent = str(self.get_parameter('parent_frame').value)
        child = str(self.get_parameter('child_frame').value)

        if not extrinsic_path:
            raise RuntimeError(
                'extrinsic_path parameter is required (path to T_tool0_camera.npy).')
        path = Path(extrinsic_path).expanduser()
        if not path.exists():
            raise FileNotFoundError(f'Extrinsic file not found: {path}')

        T = np.load(path)
        if T.shape != (4, 4):
            raise ValueError(f'Expected a 4x4 transform in {path}, got shape {T.shape}.')

        translation = T[:3, 3]
        qx, qy, qz, qw = rotmat_to_quat(T[:3, :3])

        self._broadcaster = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = parent
        tf.child_frame_id = child
        tf.transform.translation.x = float(translation[0])
        tf.transform.translation.y = float(translation[1])
        tf.transform.translation.z = float(translation[2])
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self._broadcaster.sendTransform(tf)

        self.get_logger().info(
            f'Broadcasting static TF {parent} -> {child} from {path}\n'
            f'  translation (m): [{translation[0]:.4f}, {translation[1]:.4f}, {translation[2]:.4f}]\n'
            f'  quaternion xyzw: [{qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f}]')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraExtrinsicTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
