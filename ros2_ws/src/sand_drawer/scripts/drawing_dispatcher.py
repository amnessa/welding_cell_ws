#!/usr/bin/env python3
"""
Drawing Dispatcher — IK-aware action client that feeds drawings to the server.

Loads the plane JSON, generates drawing waypoints (random shapes and locations
on the rectangular plane), verifies kinematic reachability via IK rejection
sampling, and dispatches them to the DrawingActionServer.

Usage (via launch):
  ros2 launch sand_drawer sand_drawer.launch.py mode:=action
  ros2 launch sand_drawer sand_drawer.launch.py mode:=action continuous:=true
  ros2 launch sand_drawer sand_drawer.launch.py mode:=action trajectory_key:=circle

Trajectory keys
---------------
  line      — random line on the plane
  triangle  — random equilateral triangle
  square    — random square
  circle    — random circle (30-point polyline)
  random    — pick a random shape each time (default in continuous mode)
  fixed_line — legacy: explicit UV endpoints from line_u/v params

Rejection sampling
------------------
  The rectangular drawing plane often extends into regions the UR5e cannot
  reach with the required pen-down orientation.  Before any goal is sent,
  the dispatcher runs fast IK on a sparse subset of the generated waypoints
  (first, middle, last).  If any check fails the shape is discarded and a
  new random location is tried — up to 100 attempts per drawing.

Subscribes to:  (none)
Publishes to:   (none — communicates via action client)
"""

import json
import math
import os
import random
import sys
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Quaternion
from rclpy.action import ActionClient
from rclpy.node import Node

from sand_drawer.action import ExecuteDrawing

# ── Import IK solver from the kinematics library ─────────────────────────
_scripts_dir = os.path.dirname(os.path.realpath(__file__))
if _scripts_dir not in sys.path:
    sys.path.insert(0, _scripts_dir)
from ur5e_rrt_planner import ik_solve, ur5e_fk  # noqa: E402


# ── Quaternion → rotation matrix (self-contained helper) ─────────────────

def _quat_to_rotmat(q_xyzw) -> np.ndarray:
    """Quaternion [x, y, z, w] → 3×3 rotation matrix."""
    x, y, z, w = q_xyzw
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),      1 - 2*(x*x + y*y)],
    ])


# ── Valid shape names ────────────────────────────────────────────────────
_SHAPE_POOL = ('line', 'triangle', 'square', 'circle')


class DrawingDispatcher(Node):
    """Generates IK-validated drawing paths and dispatches them."""

    # ------------------------------------------------------------------
    # Initialisation
    # ------------------------------------------------------------------
    def __init__(self):
        super().__init__('drawing_dispatcher')

        # ---- parameters ----
        self.declare_parameter('plane_json_file', '')
        self.declare_parameter('trajectory_key', 'random')
        self.declare_parameter('line_u_start', 0.5)
        self.declare_parameter('line_v_start', 0.3)
        self.declare_parameter('line_u_end', 0.5)
        self.declare_parameter('line_v_end', 0.7)
        self.declare_parameter('continuous', False)
        self.declare_parameter('shape_size_min', 0.15)
        self.declare_parameter('shape_size_max', 0.35)
        self.declare_parameter('ik_max_attempts', 100)

        self._traj_key = self.get_parameter('trajectory_key').value
        self._continuous = self.get_parameter('continuous').value
        self._line_u_start = self.get_parameter('line_u_start').value
        self._line_v_start = self.get_parameter('line_v_start').value
        self._line_u_end = self.get_parameter('line_u_end').value
        self._line_v_end = self.get_parameter('line_v_end').value
        self._shape_size_min = self.get_parameter('shape_size_min').value
        self._shape_size_max = self.get_parameter('shape_size_max').value
        self._ik_max_attempts = int(self.get_parameter('ik_max_attempts').value)

        # Home configuration — used as IK seed for reachability checks
        self._ik_seed = np.array(
            [-0.7854, -0.4363, -2.5307, -0.1745, 0.0, 0.0], dtype=float)

        # ---- load plane data (with frame correction) ----
        self._load_plane_json()

        # ---- action client ----
        self._client = ActionClient(
            self, ExecuteDrawing, 'execute_drawing')

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
            f'continuous={self._continuous}')

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
            for key in ('square_trajectory', 'projected_vector_trajectory'):
                for wp in data.get(key, []):
                    wp['position'] = _rz(wp['position'])
                    wp['orientation_xyzw'] = _qrz(wp['orientation_xyzw'])

        # Store plane data
        self._plane_data = data
        plane = data['plane']
        self.plane_origin = np.array(plane['origin'], dtype=float)
        self.plane_n = np.array(plane['normal'], dtype=float)

        corners = [np.array(c, dtype=float)
                   for c in data['rectangle_corners']]
        self.rect_origin = corners[0]
        self.rect_width_vec = corners[1] - corners[0]
        self.rect_height_vec = corners[3] - corners[0]

        # Default orientation from trajectory data
        traj = data.get('square_trajectory', [])
        if traj:
            self._default_orientation = list(traj[0]['orientation_xyzw'])
        else:
            self._default_orientation = [0.0, 0.0, 0.0, 1.0]
            self.get_logger().warn(
                'No square_trajectory in JSON — using identity orientation')

        self.get_logger().info(
            f'Loaded plane — rect width={np.linalg.norm(self.rect_width_vec):.3f}m, '
            f'height={np.linalg.norm(self.rect_height_vec):.3f}m')

    # ------------------------------------------------------------------
    # IK Reachability Check (rejection sampling filter)
    # ------------------------------------------------------------------
    def _is_reachable(self, positions: List[np.ndarray],
                      orientation_xyzw: list) -> bool:
        """Fast IK check on sparse subset of waypoints.

        Checks first, middle, and last points.  Returns False if any
        point is kinematically unreachable with the required orientation.
        """
        R = _quat_to_rotmat(orientation_xyzw)
        T = np.eye(4)
        T[:3, :3] = R

        # Sparse check: first, middle, last
        n = len(positions)
        check_indices = list({0, n // 2, n - 1})

        current_seed = self._ik_seed.copy()
        for idx in check_indices:
            T[:3, 3] = positions[idx]
            sol = ik_solve(T, current_seed, max_iter=100)
            if sol is None:
                return False
            current_seed = sol  # chain seeds for realistic continuity
        return True

    # ------------------------------------------------------------------
    # Shape generators (UV coordinates, 0–1 range)
    # ------------------------------------------------------------------
    def _random_uv_shape(self, shape: str) -> List[Tuple[float, float]]:
        """Generate UV coordinates for a shape at a random location.

        The shape is centred at a random (cu, cv) with enough margin so
        the bounding box stays within [0, 1]².
        """
        size = random.uniform(self._shape_size_min, self._shape_size_max)
        margin = size + 0.05  # keep away from rectangle edges
        cu = random.uniform(margin, 1.0 - margin)
        cv = random.uniform(margin, 1.0 - margin)

        if shape == 'line':
            angle = random.uniform(0, 2 * math.pi)
            half = size / 2.0
            dx, dy = half * math.cos(angle), half * math.sin(angle)
            return [(cu - dx, cv - dy), (cu + dx, cv + dy)]

        elif shape == 'triangle':
            angle = random.uniform(0, 2 * math.pi)
            pts = []
            for i in range(3):
                theta = angle + i * 2 * math.pi / 3
                pts.append((cu + size * math.cos(theta),
                            cv + size * math.sin(theta)))
            pts.append(pts[0])  # close the loop
            return pts

        elif shape == 'square':
            angle = random.uniform(0, math.pi / 2)
            pts = []
            for i in range(4):
                theta = angle + i * 2 * math.pi / 4 + math.pi / 4
                pts.append((cu + size * math.cos(theta),
                            cv + size * math.sin(theta)))
            pts.append(pts[0])  # close the loop
            return pts

        elif shape == 'circle':
            num_pts = 30
            pts = []
            for i in range(num_pts + 1):
                theta = i * 2 * math.pi / num_pts
                pts.append((cu + size * math.cos(theta),
                            cv + size * math.sin(theta)))
            return pts

        else:
            self.get_logger().error(f'Unknown shape: {shape}')
            return [(cu, cv)]

    def _uv_to_3d(self, uv_coords: List[Tuple[float, float]]
                   ) -> List[np.ndarray]:
        """Map UV coordinates to 3D Cartesian positions on the plane."""
        return [
            self.rect_origin
            + u * self.rect_width_vec
            + v * self.rect_height_vec
            for u, v in uv_coords
        ]

    # ------------------------------------------------------------------
    # Drawing generation with rejection sampling
    # ------------------------------------------------------------------
    def _generate_drawing(self) -> Optional[Tuple[List[Point], Quaternion]]:
        """Generate a reachable drawing goal.

        Returns (waypoints, orientation) or None if exhausted.
        """
        # Legacy: explicit UV endpoints
        if self._traj_key == 'fixed_line':
            return self._generate_fixed_line()

        # Pick shape(s)
        if self._traj_key == 'random':
            pick_random = True
        elif self._traj_key in _SHAPE_POOL:
            pick_random = False
        else:
            self.get_logger().error(
                f'Unknown trajectory_key: "{self._traj_key}"')
            return None

        orientation = self._default_orientation

        for attempt in range(1, self._ik_max_attempts + 1):
            shape = random.choice(_SHAPE_POOL) if pick_random else self._traj_key
            uv = self._random_uv_shape(shape)
            positions = self._uv_to_3d(uv)

            if self._is_reachable(positions, orientation):
                self.get_logger().info(
                    f'Generated reachable {shape} '
                    f'({len(positions)} pts) on attempt {attempt}')
                return self._to_ros_msgs(positions, orientation)

            if attempt % 20 == 0:
                self.get_logger().warn(
                    f'IK rejection sampling: {attempt} attempts so far…')

        self.get_logger().error(
            f'Failed to find a reachable shape after '
            f'{self._ik_max_attempts} attempts')
        return None

    def _generate_fixed_line(self) -> Optional[
            Tuple[List[Point], Quaternion]]:
        """Legacy: generate a line from explicit UV parameters."""
        p_start = (self.rect_origin
                   + self._line_u_start * self.rect_width_vec
                   + self._line_v_start * self.rect_height_vec)
        p_end = (self.rect_origin
                 + self._line_u_end * self.rect_width_vec
                 + self._line_v_end * self.rect_height_vec)
        positions = [p_start, p_end]
        orientation = self._default_orientation

        length = float(np.linalg.norm(p_end - p_start))
        self.get_logger().info(
            f'Fixed line: UV ({self._line_u_start},{self._line_v_start})'
            f'→({self._line_u_end},{self._line_v_end})  '
            f'length={length:.3f}m')

        if not self._is_reachable(positions, orientation):
            self.get_logger().error(
                'Fixed line endpoints are NOT reachable — check UV params')
            return None

        return self._to_ros_msgs(positions, orientation)

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
