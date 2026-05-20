#!/usr/bin/env python3
"""Action-mode drawing dispatcher for the admittance_control migration.

This trimmed dispatcher keeps only the behavior needed to draw on a detected
surface:
- load the plane JSON,
- generate metric-space geometric primitives on that plane,
- validate reachability with the orthogonal drawing tool, and
- send the path to the ExecuteDrawing action server.

Text trajectories, sweep patterns, and non-orthogonal tool variants are
intentionally excluded here so the migration surface stays small.
"""

import json
import math
import os
import random
import sys
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

try:
    from admittance_control.action import ExecuteDrawing
except ImportError:
    from sand_drawer.action import ExecuteDrawing

# ── Import IK solver from the kinematics library ─────────────────────────
_scripts_dir = os.path.dirname(os.path.realpath(__file__))
if _scripts_dir not in sys.path:
    sys.path.insert(0, _scripts_dir)
from ur5e_rrt_planner import ik_solve  # noqa: E402


# ── Quaternion → rotation matrix (self-contained helper) ─────────────────

def _quat_to_rotmat(q_xyzw) -> np.ndarray:
    """Quaternion [x, y, z, w] → 3×3 rotation matrix."""
    x, y, z, w = q_xyzw
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),      1 - 2*(x*x + y*y)],
    ])


def _rotmat_to_quat(R: np.ndarray) -> list:
    """3×3 rotation matrix → [x, y, z, w]."""
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
    q = np.array([x, y, z, w], dtype=float)
    q /= np.linalg.norm(q)
    return q.tolist()


# ── Supported shape names ────────────────────────────────────────────────
_SHAPE_POOL = ('line', 'triangle', 'square', 'circle')
_SUPPORTED_TRAJECTORY_KEYS = _SHAPE_POOL + ('random',)


class DrawingDispatcher(Node):
    """Generates IK-validated drawing paths for a fixed orthogonal tool."""

    # ------------------------------------------------------------------
    # Initialisation
    # ------------------------------------------------------------------
    def __init__(self):
        super().__init__('drawing_dispatcher')

        # ---- parameters ----
        self.declare_parameter('plane_json_file', '')
        self.declare_parameter('trajectory_key', 'random')
        self.declare_parameter('continuous', False)
        self.declare_parameter('shape_size_min_pct', 0.10)
        self.declare_parameter('shape_size_max_pct', 0.50)
        self.declare_parameter('ik_max_attempts', 100)
        self.declare_parameter('orthogonal_tool_length_m', 0.13)

        self._traj_key = self.get_parameter('trajectory_key').value
        self._continuous = self.get_parameter('continuous').value
        self._size_min_pct = self.get_parameter('shape_size_min_pct').value
        self._size_max_pct = self.get_parameter('shape_size_max_pct').value
        self._ik_max_attempts = int(
            self.get_parameter('ik_max_attempts').value)
        self._active_tool = 'orthogonal'
        self._orthogonal_tool_length = float(
            self.get_parameter('orthogonal_tool_length_m').value)

        # Home configuration — absolute baseline IK seed
        self._ik_seed = np.array(
            [-0.7854, -0.4363, -2.5307, -0.1745, 0.0, 0.0], dtype=float)
        self._ik_seeds = [
            self._ik_seed.copy(),
            np.array([-0.78, -1.00, -1.50, -0.17, 0.0, 0.0], dtype=float),
            np.array([-0.78, -0.20, -2.80, -0.17, 0.0, 0.0], dtype=float),
        ]

        # ---- load plane data (with frame correction) ----
        self._load_plane_json()

        # ---- action client ----
        self._client = ActionClient(
            self, ExecuteDrawing, 'execute_drawing')

        # ---- Path publisher for the PyQt visualizer GUI ----
        # TRANSIENT_LOCAL so a late-joining GUI still receives the last path.
        _path_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._path_pub = self.create_publisher(
            Path, '/visualizer/drawing_path', _path_qos)

        # ---- state ----
        self._goal_handle = None
        self._done = False
        self._drawing_count = 0
        self._send_pending = False
        self._retry_timer = None
        self._retry_delay = 2.0
        self._max_retry_delay = 16.0

        self.get_logger().info(
            f'Drawing dispatcher ready — '
            f'trajectory_key={self._traj_key}, '
            f'tool={self._active_tool}, '
            f'continuous={self._continuous}, '
            f'size={self._size_min_pct*100:.0f}–{self._size_max_pct*100:.0f}% '
            f'of table')

    def _get_tool_transform(self) -> Tuple[float, list]:
        """Return (tool_length_m, wrist_orientation_xyzw) for the orthogonal tool."""
        T = self._orthogonal_wrist_pose_from_tip(self.table_center_3d)
        return max(float(self._orthogonal_tool_length), 0.0), \
            _rotmat_to_quat(T[:3, :3])

    def _orthogonal_wrist_pose_from_tip(self, tip_pos: np.ndarray) -> np.ndarray:
        """Build tool0 pose for the orthogonal tool with wrist_3 normal to the plane."""
        tool_length = max(float(self._orthogonal_tool_length), 0.0)

        # The physical orthogonal tip follows +tool0_z, so tool0_z must point
        # toward the plane normal for the tip to point down at the surface.
        z_wrist = self.plane_n / max(float(np.linalg.norm(self.plane_n)), 1e-9)
        x_seed = self.base_forward - float(np.dot(self.base_forward, z_wrist)) * z_wrist
        x_norm = float(np.linalg.norm(x_seed))
        if x_norm < 1e-6:
            x_seed = self.plane_x - float(np.dot(self.plane_x, z_wrist)) * z_wrist
            x_norm = float(np.linalg.norm(x_seed))
        if x_norm < 1e-6:
            x_seed = self.plane_y.copy()
            x_norm = float(np.linalg.norm(x_seed))
        x_seed = x_seed / max(x_norm, 1e-9)

        y_seed = np.cross(z_wrist, x_seed)
        y_seed = y_seed / max(float(np.linalg.norm(y_seed)), 1e-9)
        x_wrist = np.cross(y_seed, z_wrist)
        x_wrist = x_wrist / max(float(np.linalg.norm(x_wrist)), 1e-9)
        y_wrist = y_seed

        # orthogonal_tool_length_m is the stand-off along the plane normal.
        wrist_pos = tip_pos - tool_length * z_wrist

        T = np.eye(4)
        T[:3, 0] = x_wrist
        T[:3, 1] = y_wrist
        T[:3, 2] = z_wrist
        T[:3, 3] = wrist_pos
        return T

    def _wrist_pose_from_tip(self, tip_pos: np.ndarray) -> np.ndarray:
        """Build the wrist pose for the fixed orthogonal tool."""
        return self._orthogonal_wrist_pose_from_tip(tip_pos)

    # ------------------------------------------------------------------
    # Plane JSON loading (with Rz(π) frame correction)
    # ------------------------------------------------------------------
    def _load_plane_json(self):
        json_path = self.get_parameter('plane_json_file').value
        if not json_path or not os.path.exists(json_path):
            self.get_logger().fatal(f'Plane JSON not found: {json_path}')
            raise RuntimeError(f'Plane JSON not found: {json_path}')

        with open(json_path, 'r') as f:
            data = json.load(f)

        # Frame correction: UR "base" → URDF "base_link" via Rz(π)
        target_frame = data.get('target_frame', 'base_link')
        if target_frame == 'base':
            self.get_logger().info(
                'Plane data in UR "base" frame → applying Rz(π) correction')

            def _rz(v):
                return [-v[0], -v[1], v[2]]

            def _qrz(q):
                x, y, z, w = q
                return [-y, x, w, -z]

            p = data['plane']
            p['origin'] = _rz(p['origin'])
            p['x_axis'] = _rz(p['x_axis'])
            p['y_axis'] = _rz(p['y_axis'])
            p['normal'] = _rz(p['normal'])
            data['rectangle_corners'] = [
                _rz(c) for c in data['rectangle_corners']]
            # Correct orientations in trajectory data (used for default quat)
            for key in ('square_trajectory', 'projected_vector_trajectory'):
                for wp in data.get(key, []):
                    wp['position'] = _rz(wp['position'])
                    wp['orientation_xyzw'] = _qrz(wp['orientation_xyzw'])

        # Store plane data
        self._plane_data = data
        plane = data['plane']
        self.plane_origin = np.array(plane['origin'], dtype=float)

        # Orthogonal unit vectors — using these prevents baklava skew
        self.plane_x = np.array(plane['x_axis'], dtype=float)
        self.plane_y = np.array(plane['y_axis'], dtype=float)
        self.plane_n = np.array(plane['normal'], dtype=float)

        corners = [np.array(c, dtype=float)
                   for c in data['rectangle_corners']]
        self.rect_origin = corners[0]
        self.rect_width_vec = corners[1] - corners[0]
        self.rect_height_vec = corners[3] - corners[0]

        # Absolute table dimensions in metres
        self.table_width_m = float(np.linalg.norm(self.rect_width_vec))
        self.table_height_m = float(np.linalg.norm(self.rect_height_vec))
        self.table_center_3d = (
            self.rect_origin
            + 0.5 * self.rect_width_vec
            + 0.5 * self.rect_height_vec)

        vec_to_center = self.table_center_3d.copy()
        vec_to_center = vec_to_center - np.dot(vec_to_center, self.plane_n) * self.plane_n
        if abs(float(np.dot(self.plane_x, vec_to_center))) > abs(float(np.dot(self.plane_y, vec_to_center))):
            self.base_forward = self.plane_x if float(np.dot(self.plane_x, vec_to_center)) > 0.0 else -self.plane_x
        else:
            self.base_forward = self.plane_y if float(np.dot(self.plane_y, vec_to_center)) > 0.0 else -self.plane_y
        self.base_forward = self.base_forward / max(float(np.linalg.norm(self.base_forward)), 1e-9)

        # Tool-aware orientation and wrist-tip offset
        self.tool_length, self._default_orientation = self._get_tool_transform()

        self.get_logger().info(
            f'Loaded plane — '
            f'width={self.table_width_m:.3f}m, '
            f'height={self.table_height_m:.3f}m, '
            f'tool={self._active_tool}, '
            f'tool_length={self.tool_length:.3f}m')

    # ------------------------------------------------------------------
    # Fast-Fail IK Reachability Check
    # ------------------------------------------------------------------

    # Elbow-up configuration constraints (must match action server)
    _SHOULDER_LIFT_MAX = 0.0
    _SHOULDER_LIFT_MIN = -2.5
    _ELBOW_MAX = -0.3
    _ELBOW_MIN = -3.14

    @staticmethod
    def _config_ok(q: np.ndarray) -> bool:
        """Elbow-up constraint check — mirrors the action server."""
        if q[1] > DrawingDispatcher._SHOULDER_LIFT_MAX or \
           q[1] < DrawingDispatcher._SHOULDER_LIFT_MIN:
            return False
        if q[2] > DrawingDispatcher._ELBOW_MAX or \
           q[2] < DrawingDispatcher._ELBOW_MIN:
            return False
        return True

    def _is_reachable(self, center_3d: np.ndarray,
                      positions: List[np.ndarray]) -> bool:
        """Two-stage reachability check with edge-midpoint coverage.

        Stage 1 — Fast fail: IK for the shape centre using the home seed.
                  Must also pass elbow-up config constraints.
        Stage 2 — Full boundary + edge-midpoint validation: IK for ALL
                  waypoints AND midpoints between consecutive waypoints,
                  using the centre's joint solution as seed.
                  This catches unreachable regions between corners that
                  are individually reachable.
        """
        # Stage 1: centre fast-fail (with multi-seed config check)
        T = self._wrist_pose_from_tip(center_3d)
        center_sol = None
        for seed in self._ik_seeds:
            sol = ik_solve(T, seed, max_iter=80)
            if sol is not None and self._config_ok(sol):
                center_sol = sol
                break
        if center_sol is None:
            return False

        # Stage 2: ALL waypoints + edge midpoints
        # For a 5-point square this is ~9 checks, still very fast.
        check_points: List[np.ndarray] = []
        for i, p in enumerate(positions):
            check_points.append(p)
            # Add midpoint to next waypoint (except after last)
            if i < len(positions) - 1:
                mid = 0.5 * (p + positions[i + 1])
                check_points.append(mid)

        current_seed = center_sol
        for pt in check_points:
            T = self._wrist_pose_from_tip(pt)
            # Try continuity seed first; fall back to global elbow-up seeds.
            seed_list = [current_seed] + self._ik_seeds
            sol = None
            for seed in seed_list:
                cand = ik_solve(T, seed, max_iter=80)
                if cand is not None and self._config_ok(cand):
                    sol = cand
                    break
            if sol is None:
                return False
            current_seed = sol  # chain for same-config continuity
        return True

    def _is_path_reachable(self, positions: List[np.ndarray], max_checks: int = 0) -> bool:
        """Reachability check for long paths without center fast-fail gating."""
        if not positions:
            return False

        check_points: List[np.ndarray] = []
        for i, p in enumerate(positions):
            check_points.append(p)
            if i < len(positions) - 1:
                mid = 0.5 * (p + positions[i + 1])
                check_points.append(mid)

        if max_checks > 1 and len(check_points) > max_checks:
            idxs = np.linspace(0, len(check_points) - 1, max_checks, dtype=int)
            check_points = [check_points[int(i)] for i in idxs]

        start_sol = None
        T0 = self._wrist_pose_from_tip(check_points[0])
        for seed in self._ik_seeds:
            cand = ik_solve(T0, seed, max_iter=80)
            if cand is not None and self._config_ok(cand):
                start_sol = cand
                break
        if start_sol is None:
            return False

        current_seed = start_sol
        for pt in check_points[1:]:
            T = self._wrist_pose_from_tip(pt)
            seed_list = [current_seed] + self._ik_seeds
            sol = None
            for seed in seed_list:
                cand = ik_solve(T, seed, max_iter=80)
                if cand is not None and self._config_ok(cand):
                    sol = cand
                    break
            if sol is None:
                return False
            current_seed = sol
        return True

    def _first_reachable_pose(self, positions: List[np.ndarray]) -> bool:
        """Lightweight guard: ensure at least the first sweep point has an IK solution."""
        if not positions:
            return False
        T0 = self._wrist_pose_from_tip(positions[0])
        for seed in self._ik_seeds:
            cand = ik_solve(T0, seed, max_iter=100)
            if cand is not None and self._config_ok(cand):
                return True
        return False

    # ------------------------------------------------------------------
    # Metric-space shape generation (no UV — no baklava)
    # ------------------------------------------------------------------
    def _generate_random_shape_3d(self, shape_type: str
                                  ) -> Tuple[np.ndarray, List[np.ndarray]]:
        """Generate a shape in physical 3D space on the drawing plane.

        Uses the plane's orthonormal basis vectors (plane_x, plane_y)
        so circles are circular and squares have 90° corners regardless
        of how skewed the captured rectangle is.

        Returns (center_3d, list_of_3d_positions).
        """
        # Random size: 10–50% of the smallest table dimension
        min_dim = min(self.table_width_m, self.table_height_m)
        size_min_pct = float(self._size_min_pct)
        size_max_pct = float(self._size_max_pct)

        radius = random.uniform(
            size_min_pct * min_dim / 2.0,
            size_max_pct * min_dim / 2.0)

        # Safe centre location in local metric coords
        margin = radius + 0.02  # 2 cm padding from rectangle edge
        if self.table_width_m < 2 * margin:
            margin = self.table_width_m / 3.0
        if self.table_height_m < 2 * margin:
            margin = self.table_height_m / 3.0

        cx = random.uniform(margin, self.table_width_m - margin)
        cy = random.uniform(margin, self.table_height_m - margin)

        center_3d = (self.rect_origin
                     + cx * self.plane_x
                     + cy * self.plane_y)

        pts: List[np.ndarray] = []

        if shape_type == 'line':
            angle = random.uniform(0, math.pi)
            dx = radius * math.cos(angle) * self.plane_x
            dy = radius * math.sin(angle) * self.plane_y
            pts = [center_3d - dx - dy, center_3d + dx + dy]

        elif shape_type == 'triangle':
            angle = random.uniform(0, 2 * math.pi)
            for i in range(3):
                theta = angle + i * 2 * math.pi / 3
                pts.append(center_3d
                           + radius * math.cos(theta) * self.plane_x
                           + radius * math.sin(theta) * self.plane_y)
            pts.append(pts[0])  # close the loop

        elif shape_type == 'square':
            angle = random.uniform(0, math.pi / 2)
            # Inscribed square: vertex at radius, not edge
            sq_r = radius  # half-diagonal
            for i in range(4):
                theta = angle + i * math.pi / 2 + math.pi / 4
                pts.append(center_3d
                           + sq_r * math.cos(theta) * self.plane_x
                           + sq_r * math.sin(theta) * self.plane_y)
            pts.append(pts[0])  # close the loop

        elif shape_type == 'circle':
            # Dynamic resolution: more points for larger circles
            num_pts = max(int(radius * 400), 20)
            for i in range(num_pts + 1):
                theta = i * 2 * math.pi / num_pts
                pts.append(center_3d
                           + radius * math.cos(theta) * self.plane_x
                           + radius * math.sin(theta) * self.plane_y)

        return center_3d, pts

    # ------------------------------------------------------------------
    # Drawing generation with rejection sampling
    # ------------------------------------------------------------------
    def _generate_drawing(self) -> Optional[Tuple[List[Point], Quaternion]]:
        """Generate a reachable, geometrically correct drawing goal."""
        if self._traj_key == 'random':
            pick_random = True
        elif self._traj_key in _SHAPE_POOL:
            pick_random = False
        else:
            self.get_logger().error(
                f'Unsupported trajectory_key: "{self._traj_key}". '
                f'Use one of {list(_SUPPORTED_TRAJECTORY_KEYS)}.')
            return None

        orientation = self._default_orientation

        for attempt in range(1, self._ik_max_attempts + 1):
            shape = (random.choice(_SHAPE_POOL)
                     if pick_random else self._traj_key)

            center_3d, positions = self._generate_random_shape_3d(shape)
            if not positions:
                continue
            reachable = self._is_reachable(center_3d, positions)

            if reachable:
                self.get_logger().info(
                    f'Generated reachable {shape} '
                    f'({len(positions)} pts, '
                    f'r={np.linalg.norm(positions[0] - center_3d)*1000:.0f}mm) '
                    f'on attempt {attempt}')
                return self._to_ros_msgs(positions, orientation)

            if attempt % 20 == 0:
                self.get_logger().warn(
                    f'IK rejection sampling: {attempt} attempts so far…')

        self.get_logger().error(
            f'Failed to find a reachable shape after '
            f'{self._ik_max_attempts} attempts')
        return None

    @staticmethod
    def _to_ros_msgs(positions: List[np.ndarray],
                     orientation: list) -> Tuple[List[Point], Quaternion]:
        """Convert numpy positions + orientation to ROS messages."""
        waypoints = [
            Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
            for p in positions
        ]
        quat = Quaternion(
            x=orientation[0], y=orientation[1],
            z=orientation[2], w=orientation[3])
        return waypoints, quat

    # ------------------------------------------------------------------
    # Send a single drawing goal
    # ------------------------------------------------------------------
    def send_next_drawing(self):
        """Generate and send the next drawing goal (guarded)."""
        if self._send_pending:
            self.get_logger().debug('Send already pending — skipping')
            return
        self._send_pending = True
        self._cancel_retry_timer()

        result = self._generate_drawing()
        if result is None:
            self.get_logger().error('Failed to generate drawing — stopping')
            self._send_pending = False
            self._done = True
            return

        waypoints, orientation = result

        self.get_logger().info('Waiting for action server…')
        self._client.wait_for_server()

        goal = ExecuteDrawing.Goal()
        goal.waypoints = waypoints
        goal.orientation = orientation

        # Broadcast the path to the PyQt GUI visualizer
        path_msg = Path()
        path_msg.header.frame_id = 'base_link'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for wp in waypoints:
            pose = PoseStamped()
            pose.pose.position = wp
            path_msg.poses.append(pose)
        self._path_pub.publish(path_msg)

        self._drawing_count += 1
        self.get_logger().info(
            f'Sending drawing #{self._drawing_count} '
            f'({len(waypoints)} waypoints)')

        send_future = self._client.send_goal_async(
            goal, feedback_callback=self._feedback_cb)
        send_future.add_done_callback(self._goal_response_cb)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _goal_response_cb(self, future):
        """Called when the server accepts/rejects the goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by server')
            self._send_pending = False
            if self._continuous:
                self._schedule_retry()
            else:
                self._done = True
            return

        self.get_logger().info('Goal accepted — executing')
        self._send_pending = False
        self._retry_delay = 2.0
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    # ------------------------------------------------------------------
    # Retry helpers
    # ------------------------------------------------------------------
    def _schedule_retry(self):
        """Schedule a single retry with exponential back-off."""
        self._cancel_retry_timer()
        delay = self._retry_delay
        self.get_logger().info(f'Retrying in {delay:.1f}s …')
        self._retry_timer = self.create_timer(
            delay, self._retry_timer_cb)
        self._retry_delay = min(self._retry_delay * 2.0,
                                self._max_retry_delay)

    def _retry_timer_cb(self):
        """Fire once then destroy the timer."""
        self._cancel_retry_timer()
        self.send_next_drawing()

    def _cancel_retry_timer(self):
        if self._retry_timer is not None:
            self._retry_timer.cancel()
            self.destroy_timer(self._retry_timer)
            self._retry_timer = None

    def _feedback_cb(self, feedback_msg):
        """Log feedback from the server."""
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'  [{fb.current_phase}]  '
            f'drawing={fb.drawing_progress*100:.0f}%',
            throttle_duration_sec=2.0)

    def _result_cb(self, future):
        """Called when the drawing is complete (or failed/canceled)."""
        result = future.result().result
        if result.success:
            self.get_logger().info(
                f'Drawing #{self._drawing_count} complete: {result.message}')
        else:
            self.get_logger().warn(
                f'Drawing #{self._drawing_count} failed: {result.message}')

        self._goal_handle = None
        self._send_pending = False
        self._cancel_retry_timer()

        if self._continuous:
            self.get_logger().info('Dispatching next drawing (after 0.5s)…')
            self._retry_timer = self.create_timer(
                0.5, self._retry_timer_cb)
        else:
            self.get_logger().info('Single drawing done — exiting')
            self._done = True

    # ------------------------------------------------------------------
    # Main run loop
    # ------------------------------------------------------------------
    def run(self):
        """Blocking run: send drawings and spin until done."""
        self.send_next_drawing()

        while rclpy.ok() and not self._done:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(
            f'Dispatcher finished — {self._drawing_count} drawings sent')


# ═══════════════════════════════════════════════════════════════════════════
# Entry point
# ═══════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    dispatcher = DrawingDispatcher()
    try:
        dispatcher.run()
    except KeyboardInterrupt:
        dispatcher.get_logger().info('Dispatcher interrupted')
    finally:
        dispatcher.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
