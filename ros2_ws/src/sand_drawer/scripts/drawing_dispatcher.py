#!/usr/bin/env python3
"""
Drawing Dispatcher — IK-aware action client that feeds drawings to the server.

Loads the plane JSON, generates drawing waypoints in STRICT METRIC SPACE using
orthogonal basis vectors from the plane definition.  This eliminates the
"baklava" skew that occurs when UV coordinates are mapped onto physically
non-square rectangles.

Shapes are randomly placed and sized between 10% and 50% of the shortest
table dimension.  Kinematic reachability is verified via a Fast-Fail Center
Check: IK is tested on the shape centre first; only if it succeeds is the
centre's joint solution used as seed for boundary points.  This is both
faster (centre is a single IK call vs. 3) and prevents configuration flips
(all points share the same elbow-up solution branch).

Usage (via launch):
  ros2 launch sand_drawer sand_drawer.launch.py mode:=action
  ros2 launch sand_drawer sand_drawer.launch.py mode:=action continuous:=true
  ros2 launch sand_drawer sand_drawer.launch.py mode:=action trajectory_key:=circle
  ros2 launch sand_drawer sand_drawer.launch.py mode:=action trajectory_key:=text text_string:=HELLO

Trajectory keys
---------------
  line      — random line on the plane
  triangle  — random equilateral triangle
  square    — random square
  circle    — random circle (resolution scales with size)
  text      — render text_string as multi-stroke trajectory (pen up/down)
  random    — pick a random geometric shape each time (default)

Subscribes to:  (none)
Publishes to:   /visualizer/drawing_path (nav_msgs/Path)
"""

import json
import math
import os
import random
import sys
from typing import List, Optional, Tuple

from matplotlib.textpath import TextPath
import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

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


# ── Valid shape names ────────────────────────────────────────────────────
# Geometric primitives for the random pool.  'text' is handled separately
# (only via trajectory_key=text) so random mode doesn't pick it.
_SHAPE_POOL = ('line', 'triangle', 'square', 'circle')


class DrawingDispatcher(Node):
    """Generates IK-validated, geometrically perfect drawing paths."""

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
        self.declare_parameter('text_string', 'ROMER')

        self._traj_key = self.get_parameter('trajectory_key').value
        self._text_string = self.get_parameter('text_string').value
        self._continuous = self.get_parameter('continuous').value
        self._size_min_pct = self.get_parameter('shape_size_min_pct').value
        self._size_max_pct = self.get_parameter('shape_size_max_pct').value
        self._ik_max_attempts = int(
            self.get_parameter('ik_max_attempts').value)

        # Home configuration — absolute baseline IK seed
        self._ik_seed = np.array(
            [-0.7854, -0.4363, -2.5307, -0.1745, 0.0, 0.0], dtype=float)

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
            f'continuous={self._continuous}, '
            f'size={self._size_min_pct*100:.0f}–{self._size_max_pct*100:.0f}% '
            f'of table')

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

        # Default orientation from trajectory data
        traj = data.get('square_trajectory', [])
        if traj:
            self._default_orientation = list(traj[0]['orientation_xyzw'])
        else:
            self._default_orientation = [0.0, 0.0, 0.0, 1.0]
            self.get_logger().warn(
                'No square_trajectory in JSON — using identity orientation')

        self.get_logger().info(
            f'Loaded plane — '
            f'width={self.table_width_m:.3f}m, '
            f'height={self.table_height_m:.3f}m')

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
                      positions: List[np.ndarray],
                      orientation_xyzw: list) -> bool:
        """Two-stage reachability check with edge-midpoint coverage.

        Stage 1 — Fast fail: IK for the shape centre using the home seed.
                  Must also pass elbow-up config constraints.
        Stage 2 — Full boundary + edge-midpoint validation: IK for ALL
                  waypoints AND midpoints between consecutive waypoints,
                  using the centre's joint solution as seed.
                  This catches unreachable regions between corners that
                  are individually reachable.
        """
        R = _quat_to_rotmat(orientation_xyzw)
        T = np.eye(4)
        T[:3, :3] = R

        # Stage 1: centre fast-fail (with config check)
        T[:3, 3] = center_3d
        center_sol = ik_solve(T, self._ik_seed, max_iter=80)
        if center_sol is None or not self._config_ok(center_sol):
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
            T[:3, 3] = pt
            sol = ik_solve(T, current_seed, max_iter=80)
            if sol is None or not self._config_ok(sol):
                return False
            current_seed = sol  # chain for same-config continuity
        return True

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
        radius = random.uniform(
            self._size_min_pct * min_dim / 2.0,
            self._size_max_pct * min_dim / 2.0)

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
    # Text-to-trajectory generation (multi-stroke with pen lifts)
    # ------------------------------------------------------------------
    def _generate_text_3d(
        self,
    ) -> Tuple[np.ndarray, List[np.ndarray], List[np.ndarray]]:
        """Convert a text string into a 3D multi-stroke trajectory.

        Uses matplotlib.textpath to discretise TrueType font outlines
        into polygon arrays.  Pen-up/pen-down transitions are encoded
        as Z-axis offsets along the plane normal, so the existing
        action server handles them as ordinary 3D waypoints — slowing
        down at the end of a stroke, lifting, gliding, lowering, and
        resuming the next stroke.

        Returns
        -------
        center_3d     : 3D centre of the text bounding box
        pts_3d        : full trajectory (all strokes with pen lifts)
        bbox_corners  : bounding rectangle corners for IK reachability
                        check (much cheaper than testing all waypoints)
        """
        text = self._text_string

        # 1. Discretise the font glyphs into 2D polygon arrays
        tp = TextPath((0, 0), text, size=1.0)
        polygons = tp.to_polygons()
        if not polygons:
            self.get_logger().warn(
                f'TextPath returned no polygons for "{text}"')
            return self.plane_origin, [], []

        # 2. Bounding box → scale to fit the drawing table
        all_points = np.vstack(polygons)
        min_xy = np.min(all_points, axis=0)
        max_xy = np.max(all_points, axis=0)
        text_width = max_xy[0] - min_xy[0]
        text_height = max_xy[1] - min_xy[1]
        if text_width < 1e-6 or text_height < 1e-6:
            return self.plane_origin, [], []

        # Auto-orient: lay the text along the LONGER table axis so
        # wide strings like "ROMER" exploit the full table length.
        if self.table_height_m > self.table_width_m:
            text_u = self.plane_y          # text width  → long axis
            text_v = -self.plane_x         # text height → short axis (flip for readability)
            span_along = self.table_height_m
            span_across = self.table_width_m
        else:
            text_u = self.plane_x          # text width  → long axis
            text_v = self.plane_y          # text height → short axis
            span_along = self.table_width_m
            span_across = self.table_height_m

        target_width = random.uniform(
            min(self._size_min_pct * 2.1, 0.95) * span_along,
            min(self._size_max_pct * 2.1, 0.95) * span_along)
        scale = target_width / text_width
        scaled_height = text_height * scale

        # Clamp if scaled height exceeds the cross-axis
        if scaled_height > 0.90 * span_across:
            scale = (0.90 * span_across) / text_height
            target_width = text_width * scale
            scaled_height = text_height * scale

        # 3. Random safe centre (with margin so it doesn't fall off)
        margin_u = (target_width / 2.0) + 0.02
        margin_v = (scaled_height / 2.0) + 0.02

        if span_along < 2 * margin_u:
            margin_u = span_along / 3.0
        if span_across < 2 * margin_v:
            margin_v = span_across / 3.0

        cu = random.uniform(margin_u, span_along - margin_u)
        cv = random.uniform(margin_v, span_across - margin_v)
        center_3d = (self.rect_origin
                     + cu * (text_u / np.linalg.norm(text_u))
                     + cv * (text_v / np.linalg.norm(text_v)))

        # 4. Bounding box corners on the surface for IK reachability
        half_w = target_width / 2.0
        half_h = scaled_height / 2.0
        bbox_corners = [
            center_3d - half_w * text_u - half_h * text_v,
            center_3d + half_w * text_u - half_h * text_v,
            center_3d + half_w * text_u + half_h * text_v,
            center_3d - half_w * text_u + half_h * text_v,
            center_3d - half_w * text_u - half_h * text_v,
        ]  # closed rectangle — _is_reachable also tests midpoints

        # 5. Build 3D trajectory with Z-axis pen lifts between strokes
        lift_height = 0.10  # 10 cm lift → tip 6 cm above sand
        center_2d = np.array([
            min_xy[0] + text_width / 2.0,
            min_xy[1] + text_height / 2.0])

        pts_3d: List[np.ndarray] = []
        for poly in polygons:
            # Scale around text centre
            poly_scaled = (poly - center_2d) * scale

            # Map 2D polygon → 3D points on the drawing plane
            # px runs along text_u (long axis), py along text_v
            stroke_3d = [
                center_3d + px * text_u + py * text_v
                for (px, py) in poly_scaled
            ]
            if not stroke_3d:
                continue

            # Pen-up: hover above the start of this stroke
            pts_3d.append(stroke_3d[0] - self.plane_n * lift_height)
            # Pen-down: draw the stroke on the surface
            pts_3d.extend(stroke_3d)
            # Pen-up: hover above the end of this stroke
            pts_3d.append(stroke_3d[-1] - self.plane_n * lift_height)

        self.get_logger().info(
            f'Text "{text}": {len(polygons)} strokes, '
            f'{len(pts_3d)} waypoints, '
            f'width={target_width*100:.1f}cm '
            f'(along {"height" if self.table_height_m > self.table_width_m else "width"})')

        return center_3d, pts_3d, bbox_corners

    # ------------------------------------------------------------------
    # Drawing generation with rejection sampling
    # ------------------------------------------------------------------
    def _generate_drawing(self) -> Optional[Tuple[List[Point], Quaternion]]:
        """Generate a reachable, geometrically correct drawing goal."""
        if self._traj_key == 'random':
            pick_random = True
        elif self._traj_key in _SHAPE_POOL or self._traj_key == 'text':
            pick_random = False
        else:
            self.get_logger().error(
                f'Unknown trajectory_key: "{self._traj_key}"')
            return None

        orientation = self._default_orientation

        for attempt in range(1, self._ik_max_attempts + 1):
            shape = (random.choice(_SHAPE_POOL)
                     if pick_random else self._traj_key)

            if shape == 'text':
                center_3d, positions, bbox_check = self._generate_text_3d()
                if not positions:
                    continue
                # Reachability via bounding-box corners only (9 IK checks)
                # instead of all 151+ waypoints.  If the rectangle fits,
                # every letter stroke inside it is reachable.
                reachable = self._is_reachable(
                    center_3d, bbox_check, orientation)
            else:
                center_3d, positions = self._generate_random_shape_3d(shape)
                if not positions:
                    continue
                reachable = self._is_reachable(
                    center_3d, positions, orientation)

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
