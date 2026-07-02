#!/usr/bin/env python3
"""Move the pen tip to a standoff point in front of a detected object.

This is a raw-pose test node: it takes the best SAM-6D detection (object centre,
camera frame), transforms it into ``base_link`` via TF, and commands the UR5e so
the pen tip stops a fixed standoff short of the object, approaching **along the
camera ray**.

Geometry (all in base_link)::

    P_obj    = TF(base_link <- camera) * p_cam            # object centre
    a_hat    = R(base_link <- camera) * (p_cam / |p_cam|) # camera ray, unit
    P_tip    = P_obj - standoff * a_hat                    # pen tip target
    P_flange = P_tip  - tool_length * a_hat                # tool0 target
    tool0 +Z = a_hat                                       # pen points along ray

The tool0 target pose is solved with the shared damped-least-squares IK and the
result is sent as a slow JointTrajectory to the real robot controller.

Safety
------
- Nothing moves until the ``~/go`` service (std_srvs/Trigger) is called.
- The move is refused if any joint would jump more than ``max_joint_jump_rad``
  from the current pose (guards against wild IK reconfigurations).
- Set ``dry_run:=true`` to compute + publish the target poses without moving.

Publishes (always, for RViz verification)
  /welding/target_tip     geometry_msgs/PoseStamped  (base_link)
  /welding/target_flange  geometry_msgs/PoseStamped  (base_link)
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from vision_msgs.msg import Detection3DArray

from admittance_control.geometry import quat_to_rotmat, rotmat_to_quat
from admittance_control.kinematics import ik_solve, ur5e_fk

JOINT_NAMES = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]
# Extra IK seeds (UR5e order) to complement the current pose.
FALLBACK_SEEDS = [
    np.array([-0.7854, -0.4363, -2.5307, -0.1745, 0.0, 0.0]),
    np.array([-0.78, -1.00, -1.50, -0.17, 0.0, 0.0]),
    np.array([0.0, -1.20, -1.60, -1.30, 1.57, 0.0]),
]


def _pose_msg(frame_id: str, stamp, p: np.ndarray, q_xyzw) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, p)
    msg.pose.orientation.x = float(q_xyzw[0])
    msg.pose.orientation.y = float(q_xyzw[1])
    msg.pose.orientation.z = float(q_xyzw[2])
    msg.pose.orientation.w = float(q_xyzw[3])
    return msg


def _frame_from_z(z_axis: np.ndarray) -> np.ndarray:
    """Build a right-handed rotation matrix whose +Z is ``z_axis``.

    The pen is axially symmetric so roll about Z is arbitrary; we pick X by
    projecting a reference axis, falling back if it is near-parallel to Z.
    """
    z = z_axis / max(float(np.linalg.norm(z_axis)), 1e-9)
    ref = np.array([1.0, 0.0, 0.0])
    if abs(float(np.dot(ref, z))) > 0.9:
        ref = np.array([0.0, 1.0, 0.0])
    x = ref - float(np.dot(ref, z)) * z
    x /= max(float(np.linalg.norm(x)), 1e-9)
    y = np.cross(z, x)
    y /= max(float(np.linalg.norm(y)), 1e-9)
    x = np.cross(y, z)
    R = np.eye(3)
    R[:, 0], R[:, 1], R[:, 2] = x, y, z
    return R


class MoveToObjectNode(Node):
    def __init__(self) -> None:
        super().__init__('move_to_object')

        self.declare_parameter('detections_topic', '/perception/detections')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('trajectory_topic',
                               '/scaled_joint_trajectory_controller/joint_trajectory')
        self.declare_parameter('tool_length_m', 0.19)
        self.declare_parameter('standoff_m', 0.05)
        self.declare_parameter('move_time_sec', 8.0)
        self.declare_parameter('min_score', 0.0)
        self.declare_parameter('max_joint_jump_rad', 2.5)
        self.declare_parameter('dry_run', False)

        self._base_frame = str(self.get_parameter('base_frame').value)
        self._tool_length = float(self.get_parameter('tool_length_m').value)
        self._standoff = float(self.get_parameter('standoff_m').value)
        self._move_time = float(self.get_parameter('move_time_sec').value)
        self._min_score = float(self.get_parameter('min_score').value)
        self._max_jump = float(self.get_parameter('max_joint_jump_rad').value)
        self._dry_run = bool(self.get_parameter('dry_run').value)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._latest_det: Optional[Detection3DArray] = None
        self._latest_js: Optional[JointState] = None

        self.create_subscription(
            Detection3DArray, str(self.get_parameter('detections_topic').value),
            self._on_detections, 10)
        self.create_subscription(
            JointState, str(self.get_parameter('joint_state_topic').value),
            self._on_joint_state, 10)

        self._traj_pub = self.create_publisher(
            JointTrajectory, str(self.get_parameter('trajectory_topic').value), 10)
        self._tip_pub = self.create_publisher(PoseStamped, '/welding/target_tip', 10)
        self._flange_pub = self.create_publisher(PoseStamped, '/welding/target_flange', 10)

        self._srv = self.create_service(Trigger, '~/go', self._on_go)

        self.get_logger().info(
            f'move_to_object ready (dry_run={self._dry_run}). '
            f'tool_length={self._tool_length:.3f}m standoff={self._standoff:.3f}m. '
            f'Call ~/go to compute'
            + ('.' if self._dry_run else ' and move.'))

    # ── Subscriptions ────────────────────────────────────────────────────
    def _on_detections(self, msg: Detection3DArray) -> None:
        self._latest_det = msg

    def _on_joint_state(self, msg: JointState) -> None:
        self._latest_js = msg

    def _current_q(self) -> Optional[np.ndarray]:
        if self._latest_js is None:
            return None
        name_map = dict(zip(self._latest_js.name, self._latest_js.position))
        try:
            return np.array([name_map[n] for n in JOINT_NAMES], dtype=float)
        except KeyError:
            return None

    # ── Service ──────────────────────────────────────────────────────────
    def _on_go(self, request, response):
        ok, message = self._compute_and_move()
        response.success = ok
        response.message = message
        return response

    def _best_detection(self):
        if self._latest_det is None or not self._latest_det.detections:
            return None, None
        best, best_score = None, -1.0
        for det in self._latest_det.detections:
            if not det.results:
                continue
            score = float(det.results[0].hypothesis.score)
            if score >= self._min_score and score > best_score:
                best, best_score = det, score
        return best, self._latest_det.header.frame_id

    def _compute_and_move(self):
        best, camera_frame = self._best_detection()
        if best is None:
            return False, 'no detection with score >= min_score available'
        if not camera_frame:
            return False, 'detections have empty frame_id; cannot transform'

        pos = best.results[0].pose.pose.position
        p_cam = np.array([pos.x, pos.y, pos.z], dtype=float)  # metres, camera frame
        if np.linalg.norm(p_cam) < 1e-6:
            return False, 'object point is at the camera origin (bad depth)'

        # base_link <- camera transform (rotation + translation)
        try:
            tf = self._tf_buffer.lookup_transform(
                self._base_frame, camera_frame, rclpy.time.Time())
        except TransformException as exc:
            return False, (f'TF {self._base_frame}<-{camera_frame} unavailable: {exc}. '
                           'Is robot_state_publisher + the camera static TF running?')

        t = tf.transform.translation
        r = tf.transform.rotation
        R_bc = quat_to_rotmat([r.x, r.y, r.z, r.w])
        t_bc = np.array([t.x, t.y, t.z], dtype=float)

        p_obj = R_bc @ p_cam + t_bc
        a_hat = R_bc @ (p_cam / np.linalg.norm(p_cam))     # camera ray in base
        a_hat /= max(float(np.linalg.norm(a_hat)), 1e-9)

        p_tip = p_obj - self._standoff * a_hat
        p_flange = p_tip - self._tool_length * a_hat
        R_tool = _frame_from_z(a_hat)
        q_tool = rotmat_to_quat(R_tool)

        stamp = self.get_clock().now().to_msg()
        self._tip_pub.publish(_pose_msg(self._base_frame, stamp, p_tip, q_tool))
        self._flange_pub.publish(_pose_msg(self._base_frame, stamp, p_flange, q_tool))

        self.get_logger().info(
            f'object@base=[{p_obj[0]:.3f},{p_obj[1]:.3f},{p_obj[2]:.3f}] '
            f'tip=[{p_tip[0]:.3f},{p_tip[1]:.3f},{p_tip[2]:.3f}] '
            f'flange=[{p_flange[0]:.3f},{p_flange[1]:.3f},{p_flange[2]:.3f}] '
            f'score={best.results[0].hypothesis.score:.3f}')

        # Solve IK for the tool0 target pose.
        T_target = np.eye(4)
        T_target[:3, :3] = R_tool
        T_target[:3, 3] = p_flange

        cur_q = self._current_q()
        seeds = ([cur_q] if cur_q is not None else []) + FALLBACK_SEEDS
        target_q = None
        for seed in seeds:
            cand = ik_solve(T_target, seed, max_iter=200)
            if cand is not None:
                target_q = cand
                break
        if target_q is None:
            return False, 'IK found no reachable solution for the flange target'

        # Report the achieved tip position (FK sanity).
        T_reached = ur5e_fk(target_q)
        tip_reached = T_reached[:3, 3] + self._tool_length * T_reached[:3, 2]
        tip_err_mm = float(np.linalg.norm(tip_reached - p_tip) * 1000.0)
        self.get_logger().info(f'IK tip error: {tip_err_mm:.1f} mm')

        if self._dry_run:
            return True, (f'dry run: target published, IK ok, tip error {tip_err_mm:.1f} mm '
                          '(no motion)')

        if cur_q is None:
            return False, 'no /joint_states yet; cannot start a safe trajectory'
        jump = float(np.max(np.abs(target_q - cur_q)))
        if jump > self._max_jump:
            return False, (f'refused: max joint jump {jump:.2f} rad exceeds '
                           f'max_joint_jump_rad={self._max_jump:.2f}. Move to a closer '
                           'start pose or raise the limit.')

        self._send_trajectory(cur_q, target_q)
        return True, (f'moving: max joint jump {jump:.2f} rad over {self._move_time:.1f}s, '
                      f'IK tip error {tip_err_mm:.1f} mm')

    def _send_trajectory(self, start_q: np.ndarray, target_q: np.ndarray) -> None:
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = list(JOINT_NAMES)

        start_pt = JointTrajectoryPoint()
        start_pt.positions = start_q.tolist()
        start_pt.time_from_start = Duration(sec=0, nanosec=10_000_000)
        traj.points.append(start_pt)

        end_pt = JointTrajectoryPoint()
        end_pt.positions = target_q.tolist()
        sec = int(self._move_time)
        end_pt.time_from_start = Duration(sec=sec, nanosec=int((self._move_time - sec) * 1e9))
        traj.points.append(end_pt)

        self._traj_pub.publish(traj)
        self.get_logger().info(
            f'Sent trajectory to {self._traj_pub.topic_name} ({self._move_time:.1f}s)')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MoveToObjectNode()
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
