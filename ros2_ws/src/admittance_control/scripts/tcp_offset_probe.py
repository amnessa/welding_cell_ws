#!/usr/bin/env python3
"""Measure the pendant's active TCP relative to tool0 from the driver's own topics.

    ros2 run admittance_control tcp_offset_probe.py
    # prints D = T_tool0_tcp once per second; Ctrl-C when the numbers are steady

It prints the offset twice, with the planner's NOMINAL FK and with the robot's
CALIBRATED FK (`config/ur5e_calibration.yaml`): the driver computes its TCP pose with the
calibrated chain, so the calibrated row equals the pendant TCP at every pose while the
nominal row wanders with the pose - the kinematic mismatch, directly.

The UR driver's `tcp_pose_broadcaster` publishes the robot's TCP pose (the pendant's
active TCP, in the UR `base` frame) and `robot_state_publisher` gives base_link -> tool0
from /joint_states (here recomputed with the package FK, the same chain the planner
uses). Their difference is the TCP offset the pendant applies:

    D = inv(T_baselink_tool0) @ T_baselink_base @ T_base_tcp

The hand-eye capture (extract_extrinsics.py, getActualTCPPose) recorded the robot in
THAT TCP frame, so the solved extrinsic is T_tcp_cam and must be composed as
T_tool0_cam = D @ T_tcp_cam before it is attached to tool0 - which is what
`resolve_handeye.py --tcp-offset x y z rx ry rz` does with the values printed here.
Zero means the calibration file is already in tool0.

`base_frame_rotation_deg` (default 180): the UR `base` frame is base_link rotated
about z by 180 degrees in ur_description. If the broadcaster's frame_id is already
base_link, set it to 0.
"""

from __future__ import annotations

import sys
import threading
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

from admittance_control.geometry import quat_to_rotmat  # noqa: E402
from admittance_control.kinematics import load_kinematics, ur5e_fk, ur5e_fk_params  # noqa: E402
from admittance_control.tack_reach import UR_ORDER, joint_state_to_ur_order  # noqa: E402


def _axis_angle(R: np.ndarray) -> np.ndarray:
    c = np.clip((np.trace(R) - 1) / 2, -1, 1); th = np.arccos(c)
    if th < 1e-9:
        return np.zeros(3)
    return th / (2 * np.sin(th)) * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


class TcpOffsetProbe(Node):
    def __init__(self) -> None:
        super().__init__('tcp_offset_probe')
        self.declare_parameter('tcp_pose_topic', '/tcp_pose_broadcaster/pose')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('base_frame_rotation_deg', 180.0)
        # the robot's factory calibration: printed next to the nominal FK. The driver's
        # TCP pose uses the CALIBRATED chain, so with it the offset is the pendant TCP at
        # every pose; with the nominal chain it wanders with the pose (the mismatch).
        self.declare_parameter('calibration_file', str(PKG / 'config' / 'ur5e_calibration.yaml'))
        self._q = None
        self._tcp = None
        self._lock = threading.Lock()
        self.create_subscription(JointState, str(self.get_parameter('joint_states_topic').value),
                                 self._on_joints, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, str(self.get_parameter('tcp_pose_topic').value),
                                 self._on_tcp, qos_profile_sensor_data)
        self.create_timer(1.0, self._report)
        self._samples = []
        self._cal = load_kinematics(str(self.get_parameter('calibration_file').value))

    def _on_joints(self, msg: JointState) -> None:
        if all(n in msg.name for n in UR_ORDER):
            with self._lock:
                self._q = joint_state_to_ur_order(msg.name, msg.position)

    def _on_tcp(self, msg: PoseStamped) -> None:
        p, o = msg.pose.position, msg.pose.orientation
        T = np.eye(4)
        T[:3, :3] = quat_to_rotmat([o.x, o.y, o.z, o.w])
        T[:3, 3] = [p.x, p.y, p.z]
        with self._lock:
            self._tcp = (T, msg.header.frame_id)

    def _report(self) -> None:
        with self._lock:
            q, tcp = self._q, self._tcp
        if q is None or tcp is None:
            self.get_logger().info('waiting for /joint_states and the tcp pose topic ...')
            return
        T_tcp, frame = tcp
        a = np.deg2rad(float(self.get_parameter('base_frame_rotation_deg').value))
        Rz = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
        T_bl_base = np.eye(4); T_bl_base[:3, :3] = Rz
        if frame == 'base_link':
            T_bl_base = np.eye(4)
        D = np.linalg.inv(ur5e_fk(q)) @ T_bl_base @ T_tcp
        Dc = np.linalg.inv(ur5e_fk_params(q, self._cal)) @ T_bl_base @ T_tcp
        t = D[:3, 3]; rv = _axis_angle(D[:3, :3])
        tc = Dc[:3, 3]; rvc = _axis_angle(Dc[:3, :3])
        self._samples.append(np.concatenate([t, rv]))
        self.get_logger().info(
            f"pendant TCP seen from tool0 (frame '{frame}'), joints {np.round(q, 3).tolist()}:\n"
            f"  NOMINAL FK    xyz {np.round(t * 1000, 2)} mm, rot {np.degrees(np.linalg.norm(rv)):.3f} deg\n"
            f"  CALIBRATED FK xyz {np.round(tc * 1000, 2)} mm, rot {np.degrees(np.linalg.norm(rvc)):.3f} deg\n"
            f"  -> the calibrated row should equal the pendant TCP at EVERY pose; the nominal row "
            f"moves with the pose by the kinematic mismatch (gap {np.linalg.norm(t - tc) * 1000:.2f} mm here)")


def main() -> None:
    rclpy.init()
    node = TcpOffsetProbe()
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
