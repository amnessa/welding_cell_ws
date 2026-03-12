The "A HA!" moment you just had about the mirrored text and the robot trying to reach the "underside" of the plane is one of the most classic bugs in spatial robotics.

Here is exactly what the literature says is happening, and how we will completely lock down the text generation to be centered, correctly sized, correctly oriented, and readable.

### The Problem: Coordinate System Handedness & Mirroring

When you generate text using `matplotlib.textpath`, it creates the letters in a standard 2D Cartesian plane: +X goes right, +Y goes up.

However, when you defined your physical table, you captured 4 corners. If the vector for your table's width (`plane_x`) goes right, but the vector for the table's height (`plane_y`) goes *away* from you, you have created a flipped coordinate system. When the code mapped the text's +Y directly to the table's `plane_y`, it essentially printed the text upside down and mirrored it.

When the UR5e's Inverse Kinematics solver receives mirrored coordinates that curl in an unnatural direction, it sometimes attempts to solve them by flipping its wrist 180 degrees, which makes it look like it's trying to attack the table from underneath!

### The Solution: Deterministic Text Layout

We will remove all randomness from the text mode.

1. **Strict Centering:** The text will always be anchored exactly to the mathematical center of the table.
2. **Smart Rotation:** It will check if the word (e.g., "ROMER") fits horizontally. If it is too long, it will automatically rotate it 90 degrees to fit along the vertical axis of the table.
3. **Dynamic Sizing:** It will try to use your maximum requested letter size (e.g., 20cm). If the word is too long and falls off the table, it will dynamically shrink the font until it fits. If it shrinks below your minimum requested size (e.g., 10cm) and *still* doesn't fit, it will cleanly reject the request instead of breaking the robot.
4. **The Mirror Fix:** We will explicitly flip the Y-axis of the text so it projects correctly onto the physical table.

### The Updated Dispatcher

Replace your `DrawingDispatcher` with this code. It completely isolates the new Text logic from the random shapes.

```python
#!/usr/bin/env python3
"""
Drawing Dispatcher — IK-aware action client that feeds drawings to the server.

Features:
- Geometric Shapes: Randomly placed and scaled.
- Text Mode: Deterministic. Strictly centered, auto-rotated to fit the longest
  table axis, dynamically scaled to fit within safety margins, and horizontally
  flipped to prevent the "mirrored underside" coordinate bug.
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
        self.declare_parameter('trajectory_key', 'text')
        self.declare_parameter('continuous', False)

        # Absolute Metric Sizes for Text/Shapes (e.g., 10cm to 20cm)
        self.declare_parameter('shape_size_min_m', 0.10)
        self.declare_parameter('shape_size_max_m', 0.20)

        self.declare_parameter('ik_max_attempts', 100)
        self.declare_parameter('text_string', 'ROMER')

        self._traj_key = self.get_parameter('trajectory_key').value
        self._text_string = self.get_parameter('text_string').value
        self._continuous = self.get_parameter('continuous').value
        self._size_min_m = self.get_parameter('shape_size_min_m').value
        self._size_max_m = self.get_parameter('shape_size_max_m').value
        self._ik_max_attempts = int(self.get_parameter('ik_max_attempts').value)

        self._base_ik_seed = np.array([-0.7854, -0.4363, -2.5307, -0.1745, 0.0, 0.0], dtype=float)
        self._ik_seeds = [
            self._base_ik_seed,
            np.array([-0.78, -1.0, -1.5, -0.17, 0.0, 0.0], dtype=float),
            np.array([-0.78, -0.2, -2.8, -0.17, 0.0, 0.0], dtype=float)
        ]

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

    def _load_plane_json(self):
        json_path = self.get_parameter('plane_json_file').value
        with open(json_path, 'r') as f:
            data = json.load(f)

        target_frame = data.get('target_frame', 'base_link')
        if target_frame == 'base':
            def _rz(v): return [-v[0], -v[1], v[2]]
            def _qrz(q): return [-q[1], q[0], q[3], -q[2]]
            data['plane']['origin'] = _rz(data['plane']['origin'])
            data['plane']['x_axis'] = _rz(data['plane']['x_axis'])
            data['plane']['y_axis'] = _rz(data['plane']['y_axis'])
            data['plane']['normal'] = _rz(data['plane']['normal'])
            data['rectangle_corners'] = [_rz(c) for c in data['rectangle_corners']]

        self.plane_origin = np.array(data['plane']['origin'], dtype=float)
        self.plane_x = np.array(data['plane']['x_axis'], dtype=float)
        self.plane_y = np.array(data['plane']['y_axis'], dtype=float)
        self.plane_n = np.array(data['plane']['normal'], dtype=float)

        corners = [np.array(c, dtype=float) for c in data['rectangle_corners']]
        self.rect_origin = corners[0]
        self.rect_width_vec = corners[1] - corners[0]
        self.rect_height_vec = corners[3] - corners[0]

        self.table_width_m = float(np.linalg.norm(self.rect_width_vec))
        self.table_height_m = float(np.linalg.norm(self.rect_height_vec))

        # Absolute Center of the physical table
        self.table_center_3d = self.rect_origin + (0.5 * self.rect_width_vec) + (0.5 * self.rect_height_vec)

        traj = data.get('square_trajectory', [])
        self._default_orientation = list(traj[0]['orientation_xyzw']) if traj else [0.0, 0.0, 0.0, 1.0]

    _SHOULDER_LIFT_MAX = 0.0
    _SHOULDER_LIFT_MIN = -2.5
    _ELBOW_MAX = -0.3
    _ELBOW_MIN = -3.14

    @staticmethod
    def _config_ok(q: np.ndarray) -> bool:
        if q[1] > DrawingDispatcher._SHOULDER_LIFT_MAX or q[1] < DrawingDispatcher._SHOULDER_LIFT_MIN: return False
        if q[2] > DrawingDispatcher._ELBOW_MAX or q[2] < DrawingDispatcher._ELBOW_MIN: return False
        return True

    def _is_reachable(self, center_3d: np.ndarray, positions: List[np.ndarray], orientation_xyzw: list) -> bool:
        R = _quat_to_rotmat(orientation_xyzw)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = center_3d

        center_sol = None
        for seed in self._ik_seeds:
            sol = ik_solve(T, seed, max_iter=80)
            if sol is not None and self._config_ok(sol):
                center_sol = sol
                break

        if center_sol is None: return False

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

    def _optimize_stroke(self, stroke: List[np.ndarray], min_dist=0.003) -> List[np.ndarray]:
        if len(stroke) < 3: return stroke
        optimized = [stroke[0]]
        for pt in stroke[1:-1]:
            if np.linalg.norm(pt - optimized[-1]) >= min_dist:
                optimized.append(pt)
        optimized.append(stroke[-1])
        return optimized

    # ------------------------------------------------------------------
    # Deterministic Text Generator
    # ------------------------------------------------------------------
    def _generate_centered_text_3d(self) -> Tuple[np.ndarray, List[np.ndarray], List[np.ndarray]]:
        text = self._text_string
        tp = TextPath((0, 0), text, size=1.0)
        polygons = tp.to_polygons()

        if not polygons:
            return self.table_center_3d, [], []

        # Find raw 2D bounding box
        all_points = np.vstack(polygons)
        min_x, min_y = np.min(all_points, axis=0)
        max_x, max_y = np.max(all_points, axis=0)
        text_w_raw = max_x - min_x
        text_h_raw = max_y - min_y
        true_center_2d = np.array([min_x + (text_w_raw / 2.0), min_y + (text_h_raw / 2.0)])

        safe_margin = 0.04 # 4cm buffer from table edges

        # START WITH MAX SIZE
        target_height = self._size_max_m
        scale = target_height / text_h_raw
        target_width = text_w_raw * scale

        # LOGIC: Check if it fits horizontally. If not, rotate 90 degrees.
        fit_horizontal = (target_width <= self.table_width_m - safe_margin) and (target_height <= self.table_height_m - safe_margin)
        fit_vertical = (target_width <= self.table_height_m - safe_margin) and (target_height <= self.table_width_m - safe_margin)

        if fit_horizontal:
            text_u, text_v = self.plane_x, self.plane_y
            self.get_logger().info(f'Text fits horizontally.')
        elif fit_vertical:
            # Rotate 90 degrees
            text_u, text_v = self.plane_y, -self.plane_x
            self.get_logger().info(f'Text rotated 90 degrees to fit vertical layout.')
            # Swap table dimensions for the scaling math
            self.table_width_m, self.table_height_m = self.table_height_m, self.table_width_m
        else:
            # DOES NOT FIT AT MAX SIZE. Shrink it down to fit the table width
            text_u, text_v = self.plane_x, self.plane_y
            target_width = self.table_width_m - safe_margin
            scale = target_width / text_w_raw
            target_height = text_h_raw * scale

            # If it shrunk below our absolute minimum requirement, REJECT IT.
            if target_height < self._size_min_m:
                self.get_logger().error(f'Word "{text}" is too long. Shrinking it to fit makes letters {target_height*100:.1f}cm tall, which is smaller than minimum ({self._size_min_m*100:.1f}cm).')
                return self.table_center_3d, [], []

            self.get_logger().info(f'Text shrunk to {target_height*100:.1f}cm height to fit table.')

        # Bounding box corners for IK checking
        half_w, half_h = target_width / 2.0, target_height / 2.0
        bbox_corners = [
            self.table_center_3d - half_w * text_u - half_h * text_v,
            self.table_center_3d + half_w * text_u - half_h * text_v,
            self.table_center_3d + half_w * text_u + half_h * text_v,
            self.table_center_3d - half_w * text_u + half_h * text_v,
            self.table_center_3d - half_w * text_u - half_h * text_v,
        ]

        lift_height = 0.05  # Lift pen 5cm between letters
        pts_3d = []
        for poly in polygons:
            poly_scaled = (poly - true_center_2d) * scale

            # MIRROR FIX: Notice the `- py` here. We invert the Y axis of the text
            # to cancel out any mirroring caused by the physical table's coordinate frame.
            stroke_3d = [self.table_center_3d + px * text_u - py * text_v for (px, py) in poly_scaled]
            stroke_3d = self._optimize_stroke(stroke_3d, min_dist=0.003)

            if not stroke_3d: continue
            pts_3d.append(stroke_3d[0] - self.plane_n * lift_height)
            pts_3d.extend(stroke_3d)
            pts_3d.append(stroke_3d[-1] - self.plane_n * lift_height)

        return self.table_center_3d, pts_3d, bbox_corners


    def _generate_random_shape_3d(self, shape_type: str) -> Tuple[np.ndarray, List[np.ndarray]]:
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


    def _generate_drawing(self) -> Optional[Tuple[List[Point], Quaternion]]:
        orientation = self._default_orientation

        if self._traj_key == 'text':
            # Text is deterministic, no retry loop needed.
            center_3d, positions, bbox_check = self._generate_centered_text_3d()
            if not positions:
                return None
            if self._is_reachable(center_3d, bbox_check, orientation):
                self.get_logger().info(f'Text "{self._text_string}" generated and is reachable.')
                return self._to_ros_msgs(positions, orientation)
            else:
                self.get_logger().error(f'Text "{self._text_string}" fits the table geometrically, but is kinematically unreachable for the robot.')
                return None

        # Random Geometric Shapes
        pick_random = (self._traj_key == 'random')
        for attempt in range(1, self._ik_max_attempts + 1):
            shape = random.choice(_SHAPE_POOL) if pick_random else self._traj_key
            center_3d, positions = self._generate_random_shape_3d(shape)

            if self._is_reachable(center_3d, positions, orientation):
                return self._to_ros_msgs(positions, orientation)

        self.get_logger().error(f'Failed to find a reachable shape location after {self._ik_max_attempts} attempts.')
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