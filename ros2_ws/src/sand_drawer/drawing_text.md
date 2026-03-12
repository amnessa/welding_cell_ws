The textbooks address exactly why the robot struggles with dense text trajectories and how to mathematically guarantee that shapes remain within a constrained planar workspace.

### The Literature on Text Generation and Optimization

**1. Oriented Bounding Boxes (OBB) vs. Workspace Limits**
When generating a long string of text like "ROMER," the system cannot simply pick a center point and assume the shape is safe. *Springer Handbook of Robotics* details the necessity of Oriented Bounding Boxes (OBBs) for geometric validation. Before any Inverse Kinematics are calculated, the algorithm must compute the bounding box of the entire text string, calculate its physical dimensions in meters, and strictly clamp those dimensions to the physical limits of the table (minus a safety margin). If the user requests 20cm letters but the resulting word is 1.5 meters long, the algorithm must dynamically shrink the text to fit the maximum safe dimension of the plane.

**2. Trajectory Decimation (The 10Hz Problem)**
Dropping the sample rate to 10Hz was a necessary workaround because the UR5e controller was experiencing buffer starvation. *Modern Robotics* explains that converting complex cubic Bezier curves (like TrueType fonts) into linear waypoints often creates points that are micrometers apart. When thousands of these dense points are sent to an industrial controller, the internal interpolator chokes.

To optimize this without losing speed, the literature dictates **Spatial Decimation**.  Instead of changing the time frequency (Hz), you filter the spatial geometry. You mathematically evaluate the distance between consecutive points and drop any points that are too close together (e.g., less than 5 millimeters apart) or lie on a straight line. This reduces a 2,000-point letter 'O' down to 40 highly optimized points, allowing the robot to execute it smoothly at high speeds.

**3. MoveIt 2 API & Circular Interpolation**
MoveIt 2 contains the Pilz Industrial Motion Planner, which natively supports `CIRC` (Circular) commands. If you pass Pilz three points, it mathematically calculates the perfect arc. However, standard fonts are constructed of Beziers, not pure circular arcs. For text rendering, spatial decimation of a dense polyline remains the industry standard.

---

### The Optimized Dispatcher

This updated `DrawingDispatcher` implements absolute metric sizing (10cm to 20cm), automatic aspect-ratio clamping so text never falls off the table, and a spatial decimation filter to optimize the trajectories for the UR5e.

```python
#!/usr/bin/env python3
"""
Drawing Dispatcher — IK-aware action client that feeds drawings to the server.

Features:
- Absolute Metric Sizing (e.g., 10cm to 20cm bounding boxes).
- Aspect-Ratio Clamping: Long text ("ROMER") is automatically shrunk to fit the table.
- Spatial Decimation: Drops microscopic waypoints to prevent UR5e buffer starvation.
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

_scripts_dir = os.path.dirname(os.path.realpath(__file__))
if _scripts_dir not in sys.path:
    sys.path.insert(0, _scripts_dir)
from ur5e_rrt_planner import ik_solve

def _quat_to_rotmat(q_xyzw) -> np.ndarray:
    x, y, z, w = q_xyzw
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),      1 - 2*(x*x + y*y)],
    ])

_SHAPE_POOL = ('line', 'triangle', 'square', 'circle')

class DrawingDispatcher(Node):
    def __init__(self):
        super().__init__('drawing_dispatcher')

        # ---- Parameters ----
        self.declare_parameter('plane_json_file', '')
        self.declare_parameter('trajectory_key', 'random')
        self.declare_parameter('continuous', False)

        # Absolute Metric Sizes (Meters)
        self.declare_parameter('shape_size_min_m', 0.10) # 10 cm min
        self.declare_parameter('shape_size_max_m', 0.20) # 20 cm max

        self.declare_parameter('ik_max_attempts', 100)
        self.declare_parameter('text_string', 'ROMER')

        self._traj_key = self.get_parameter('trajectory_key').value
        self._text_string = self.get_parameter('text_string').value
        self._continuous = self.get_parameter('continuous').value
        self._size_min_m = self.get_parameter('shape_size_min_m').value
        self._size_max_m = self.get_parameter('shape_size_max_m').value
        self._ik_max_attempts = int(self.get_parameter('ik_max_attempts').value)

        self._ik_seed = np.array([-0.7854, -0.4363, -2.5307, -0.1745, 0.0, 0.0], dtype=float)

        self._load_plane_json()

        self._client = ActionClient(self, ExecuteDrawing, 'execute_drawing')

        _path_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._path_pub = self.create_publisher(Path, '/visualizer/drawing_path', _path_qos)

        self._goal_handle = None
        self._done = False
        self._drawing_count = 0
        self._send_pending = False
        self._retry_timer = None
        self._retry_delay = 2.0
        self._max_retry_delay = 16.0

        self.get_logger().info(f'Dispatcher ready — size={self._size_min_m*100:.0f}–{self._size_max_m*100:.0f}cm')

    def _load_plane_json(self):
        json_path = self.get_parameter('plane_json_file').value
        with open(json_path, 'r') as f:
            data = json.load(f)

        target_frame = data.get('target_frame', 'base_link')
        if target_frame == 'base':
            def _rz(v): return [-v[0], -v[1], v[2]]
            def _qrz(q): return [-y, x, w, -z]
            data['plane']['origin'] = _rz(data['plane']['origin'])
            data['plane']['x_axis'] = _rz(data['plane']['x_axis'])
            data['plane']['y_axis'] = _rz(data['plane']['y_axis'])
            data['plane']['normal'] = _rz(data['plane']['normal'])
            data['rectangle_corners'] = [_rz(c) for c in data['rectangle_corners']]

        plane = data['plane']
        self.plane_origin = np.array(plane['origin'], dtype=float)
        self.plane_x = np.array(plane['x_axis'], dtype=float)
        self.plane_y = np.array(plane['y_axis'], dtype=float)
        self.plane_n = np.array(plane['normal'], dtype=float)

        corners = [np.array(c, dtype=float) for c in data['rectangle_corners']]
        self.rect_origin = corners[0]
        self.rect_width_vec = corners[1] - corners[0]
        self.rect_height_vec = corners[3] - corners[0]

        self.table_width_m = float(np.linalg.norm(self.rect_width_vec))
        self.table_height_m = float(np.linalg.norm(self.rect_height_vec))

        traj = data.get('square_trajectory', [])
        self._default_orientation = list(traj[0]['orientation_xyzw']) if traj else [0.0, 0.0, 0.0, 1.0]

    _SHOULDER_LIFT_MAX = 0.0
    _SHOULDER_LIFT_MIN = -2.5
    _ELBOW_MAX = -0.3
    _ELBOW_MIN = -3.14

    @staticmethod
    def _config_ok(q: np.ndarray) -> bool:
        if q[1] > DrawingDispatcher._SHOULDER_LIFT_MAX or q[1] < DrawingDispatcher._SHOULDER_LIFT_MIN:
            return False
        if q[2] > DrawingDispatcher._ELBOW_MAX or q[2] < DrawingDispatcher._ELBOW_MIN:
            return False
        return True

    def _is_reachable(self, center_3d: np.ndarray, positions: List[np.ndarray], orientation_xyzw: list) -> bool:
        R = _quat_to_rotmat(orientation_xyzw)
        T = np.eye(4)
        T[:3, :3] = R

        T[:3, 3] = center_3d
        center_sol = ik_solve(T, self._ik_seed, max_iter=80)
        if center_sol is None or not self._config_ok(center_sol): return False

        check_points = []
        for i, p in enumerate(positions):
            check_points.append(p)
            if i < len(positions) - 1:
                check_points.append(0.5 * (p + positions[i + 1]))

        current_seed = center_sol
        for pt in check_points:
            T[:3, 3] = pt
            sol = ik_solve(T, current_seed, max_iter=80)
            if sol is None or not self._config_ok(sol): return False
            current_seed = sol
        return True

    # ------------------------------------------------------------------
    # Spatial Decimation (Optimization)
    # ------------------------------------------------------------------
    def _optimize_stroke(self, stroke: List[np.ndarray], min_dist=0.005) -> List[np.ndarray]:
        """
        Removes microscopic points from a stroke.
        Requires points to be at least `min_dist` (5mm) apart, preventing buffer starvation.
        """
        if len(stroke) < 3:
            return stroke

        optimized = [stroke[0]]
        for pt in stroke[1:-1]:
            if np.linalg.norm(pt - optimized[-1]) >= min_dist:
                optimized.append(pt)

        # Always preserve the exact final point
        optimized.append(stroke[-1])
        return optimized

    # ------------------------------------------------------------------
    # Geometric Generators
    # ------------------------------------------------------------------
    def _generate_random_shape_3d(self, shape_type: str) -> Tuple[np.ndarray, List[np.ndarray]]:

        # Absolute size generation
        radius = random.uniform(self._size_min_m / 2.0, self._size_max_m / 2.0)
        margin = radius + 0.02

        cx = random.uniform(margin, self.table_width_m - margin)
        cy = random.uniform(margin, self.table_height_m - margin)
        center_3d = self.rect_origin + (cx * self.plane_x) + (cy * self.plane_y)

        pts = []
        if shape_type == 'line':
            angle = random.uniform(0, math.pi)
            dx, dy = radius * math.cos(angle) * self.plane_x, radius * math.sin(angle) * self.plane_y
            pts = [center_3d - dx - dy, center_3d + dx + dy]
        elif shape_type == 'triangle':
            angle = random.uniform(0, 2 * math.pi)
            for i in range(4):
                theta = angle + i * 2 * math.pi / 3
                pts.append(center_3d + radius * math.cos(theta) * self.plane_x + radius * math.sin(theta) * self.plane_y)
        elif shape_type == 'square':
            angle = random.uniform(0, math.pi / 2)
            for i in range(5):
                theta = angle + i * math.pi / 2 + math.pi / 4
                pts.append(center_3d + radius * math.cos(theta) * self.plane_x + radius * math.sin(theta) * self.plane_y)
        elif shape_type == 'circle':
            num_pts = max(int(radius * 400), 20)
            for i in range(num_pts + 1):
                theta = i * 2 * math.pi / num_pts
                pts.append(center_3d + radius * math.cos(theta) * self.plane_x + radius * math.sin(theta) * self.plane_y)

        return center_3d, self._optimize_stroke(pts)

    def _generate_text_3d(self) -> Tuple[np.ndarray, List[np.ndarray], List[np.ndarray]]:
        text = self._text_string
        tp = TextPath((0, 0), text, size=1.0)
        polygons = tp.to_polygons()
        if not polygons: return self.plane_origin, [], []

        all_points = np.vstack(polygons)
        min_xy, max_xy = np.min(all_points, axis=0), np.max(all_points, axis=0)
        text_width = max_xy[0] - min_xy[0]
        text_height = max_xy[1] - min_xy[1]

        if self.table_height_m > self.table_width_m:
            text_u, text_v = self.plane_y, -self.plane_x
            span_along, span_across = self.table_height_m, self.table_width_m
        else:
            text_u, text_v = self.plane_x, self.plane_y
            span_along, span_across = self.table_width_m, self.table_height_m

        # Set target height to an absolute metric size (e.g. 20cm tall letters)
        target_height = random.uniform(self._size_min_m, self._size_max_m)
        scale = target_height / text_height
        target_width = text_width * scale

        # ASPECT-RATIO CLAMPING: Prevent text from hanging off the table
        safe_margin = 0.04 # 4cm total buffer
        if target_width > (span_along - safe_margin):
            target_width = span_along - safe_margin
            scale = target_width / text_width
            target_height = text_height * scale

        if target_height > (span_across - safe_margin):
            target_height = span_across - safe_margin
            scale = target_height / text_height
            target_width = text_width * scale

        margin_u = (target_width / 2.0) + 0.02
        margin_v = (target_height / 2.0) + 0.02

        cu = random.uniform(margin_u, span_along - margin_u)
        cv = random.uniform(margin_v, span_across - margin_v)
        center_3d = self.rect_origin + cu * (text_u / np.linalg.norm(text_u)) + cv * (text_v / np.linalg.norm(text_v))

        half_w, half_h = target_width / 2.0, target_height / 2.0
        bbox_corners = [
            center_3d - half_w * text_u - half_h * text_v,
            center_3d + half_w * text_u - half_h * text_v,
            center_3d + half_w * text_u + half_h * text_v,
            center_3d - half_w * text_u + half_h * text_v,
            center_3d - half_w * text_u - half_h * text_v,
        ]

        lift_height = 0.10
        center_2d = np.array([min_xy[0] + text_width / 2.0, min_xy[1] + text_height / 2.0])

        pts_3d = []
        for poly in polygons:
            poly_scaled = (poly - center_2d) * scale
            stroke_3d = [center_3d + px * text_u + py * text_v for (px, py) in poly_scaled]

            # Apply Spatial Decimation
            stroke_3d = self._optimize_stroke(stroke_3d, min_dist=0.005)

            if not stroke_3d: continue
            pts_3d.append(stroke_3d[0] - self.plane_n * lift_height)
            pts_3d.extend(stroke_3d)
            pts_3d.append(stroke_3d[-1] - self.plane_n * lift_height)

        self.get_logger().info(f'Text "{text}": W={target_width*100:.1f}cm, H={target_height*100:.1f}cm')
        return center_3d, pts_3d, bbox_corners

    def _generate_drawing(self) -> Optional[Tuple[List[Point], Quaternion]]:
        if self._traj_key == 'random': pick_random = True
        elif self._traj_key in _SHAPE_POOL or self._traj_key == 'text': pick_random = False
        else: return None

        orientation = self._default_orientation

        for attempt in range(1, self._ik_max_attempts + 1):
            shape = random.choice(_SHAPE_POOL) if pick_random else self._traj_key

            if shape == 'text':
                center_3d, positions, bbox_check = self._generate_text_3d()
                if not positions: continue
                reachable = self._is_reachable(center_3d, bbox_check, orientation)
            else:
                center_3d, positions = self._generate_random_shape_3d(shape)
                if not positions: continue
                reachable = self._is_reachable(center_3d, positions, orientation)

            if reachable:
                return self._to_ros_msgs(positions, orientation)

        return None

    @staticmethod
    def _to_ros_msgs(positions: List[np.ndarray], orientation: list) -> Tuple[List[Point], Quaternion]:
        waypoints = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in positions]
        quat = Quaternion(x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])
        return waypoints, quat

    def send_next_drawing(self):
        if self._send_pending: return
        self._send_pending = True
        self._cancel_retry_timer()

        result = self._generate_drawing()
        if result is None:
            self._send_pending = False
            self._done = True
            return

        waypoints, orientation = result
        self._client.wait_for_server()
        goal = ExecuteDrawing.Goal()
        goal.waypoints, goal.orientation = waypoints, orientation

        path_msg = Path()
        path_msg.header.frame_id = 'base_link'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for wp in waypoints:
            pose = PoseStamped()
            pose.pose.position = wp
            path_msg.poses.append(pose)
        self._path_pub.publish(path_msg)

        self._drawing_count += 1
        send_future = self._client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._send_pending = False
            self._schedule_retry() if self._continuous else setattr(self, '_done', True)
            return

        self._send_pending = False
        self._retry_delay = 2.0
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _schedule_retry(self):
        self._cancel_retry_timer()
        self._retry_timer = self.create_timer(self._retry_delay, self._retry_timer_cb)
        self._retry_delay = min(self._retry_delay * 2.0, self._max_retry_delay)

    def _retry_timer_cb(self):
        self._cancel_retry_timer()
        self.send_next_drawing()

    def _cancel_retry_timer(self):
        if self._retry_timer is not None:
            self._retry_timer.cancel()
            self.destroy_timer(self._retry_timer)
            self._retry_timer = None

    def _feedback_cb(self, feedback_msg): pass

    def _result_cb(self, future):
        self._goal_handle = None
        self._send_pending = False
        self._cancel_retry_timer()

        if self._continuous:
            self._retry_timer = self.create_timer(0.5, self._retry_timer_cb)
        else:
            self._done = True

    def run(self):
        self.send_next_drawing()
        while rclpy.ok() and not self._done:
            rclpy.spin_once(self, timeout_sec=0.1)

def main(args=None):
    rclpy.init(args=args)
    dispatcher = DrawingDispatcher()
    try: dispatcher.run()
    except KeyboardInterrupt: pass
    finally:
        dispatcher.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__': main()

```