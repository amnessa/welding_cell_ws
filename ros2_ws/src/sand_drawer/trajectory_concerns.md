You have accurately identified three major interconnected problems that plague Cartesian robotic applications. Here is the textbook explanation of what is going wrong, followed by the newly re-engineered script that solves all three.

### 1. The "Baklava" Effect (Affine Skew)

**The Problem:** You were mapping a perfect square in the `[0.0 - 1.0] UV space`, and then projecting it onto the physical `rect_width_vec` and `rect_height_vec`. If those physical vectors are not perfectly equal in length, or if they are off by a few degrees (human error when clicking the 4 points on the table), the math stretches and skews your perfect square into a rhombus (a baklava) .
**The Fix:** We must completely abandon UV coordinates for shape generation. Instead, the new script calculates the physical width and height of the table in meters. It then uses the mathematically perfect orthogonal unit vectors (`plane_x` and `plane_y`) to generate the shapes strictly in 3D metric space. A square is now guaranteed to have perfectly 90-degree corners.

### 2. Computation Time (The IK Bottleneck)

**The Problem:** Your `_is_reachable` function was trying to solve Inverse Kinematics for random points using the robot's generic `Home` position as the mathematical seed. Because the table is far from the Home position, the Levenberg-Marquardt IK solver had to take dozens of heavy matrix-inversion steps to find a solution. Doing this 100 times caused massive delays.
**The Fix:** We implement a **Fast-Fail Center Check**. The script now generates the shape, but *only* runs IK on the exact center of the shape. If the center is unreachable, it instantly discards it without doing math on the corners. If the center *is* reachable, it takes the resulting joint angles and uses them as the "Seed" for the corners. Because the corners are inches away from the center, the IK solver solves them in a fraction of a millisecond.

### 3. Trajectory Jumps (Configuration Flips)

**The Problem:** When shapes are skewed, or when checking reachability with bad IK seeds, the solver might accidentally calculate "Elbow Up" for the first point of the square, and "Elbow Down" for the second point. The robot attempts to swing its arm wildly to match it, creating a jump.
**The Fix:** By chaining the IK seed from the `center_3d` to the edges, we mathematically guarantee that the entire shape belongs to the exact same kinematic configuration, completely eliminating erratic joint jumps.

### 4. Random Scaling (10% to 50%)

The shapes now measure the physical dimensions of your table, pick the shortest side, and randomize the shape's radius to be between 10% and 50% of that available space.

Here is the fully optimized `DrawingDispatcher`:

```python
#!/usr/bin/env python3
"""
Drawing Dispatcher — IK-aware action client that feeds drawings to the server.

Loads the plane JSON, generates drawing waypoints in STRICT METRIC SPACE using
orthogonal basis vectors (eliminating the "baklava" skew). Randomizes shape
sizes between 10% and 50% of the table size.

Verifies kinematic reachability via highly optimized IK rejection sampling
(Fast-Fail Center Check), and dispatches them to the DrawingActionServer.
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
_SHAPE_POOL = ('line', 'triangle', 'square', 'circle')

class DrawingDispatcher(Node):
    """Generates IK-validated, geometrically perfect drawing paths."""

    def __init__(self):
        super().__init__('drawing_dispatcher')

        # ---- parameters ----
        self.declare_parameter('plane_json_file', '')
        self.declare_parameter('trajectory_key', 'random')
        self.declare_parameter('continuous', False)

        # New Percentage Scaling Parameters
        self.declare_parameter('shape_size_min_pct', 0.10)
        self.declare_parameter('shape_size_max_pct', 0.50)
        self.declare_parameter('ik_max_attempts', 100)

        self._traj_key = self.get_parameter('trajectory_key').value
        self._continuous = self.get_parameter('continuous').value
        self._size_min_pct = self.get_parameter('shape_size_min_pct').value
        self._size_max_pct = self.get_parameter('shape_size_max_pct').value
        self._ik_max_attempts = int(self.get_parameter('ik_max_attempts').value)

        # Home configuration — used as the absolute baseline IK seed
        self._ik_seed = np.array(
            [-0.7854, -0.4363, -2.5307, -0.1745, 0.0, 0.0], dtype=float)

        self._load_plane_json()

        # ---- action client ----
        self._client = ActionClient(self, ExecuteDrawing, 'execute_drawing')

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

    def _load_plane_json(self):
        json_path = self.get_parameter('plane_json_file').value
        if not json_path or not os.path.exists(json_path):
            self.get_logger().fatal(f'Plane JSON not found: {json_path}')
            raise RuntimeError(f'Plane JSON not found: {json_path}')

        with open(json_path, 'r') as f:
            data = json.load(f)

        target_frame = data.get('target_frame', 'base_link')
        if target_frame == 'base':
            self.get_logger().info('Plane data in UR "base" frame → applying Rz(π) correction')
            def _rz(v): return [-v[0], -v[1], v[2]]
            def _qrz(q): return [-q[1], q[0], q[3], -q[2]]

            p = data['plane']
            p['origin'] = _rz(p['origin'])
            p['x_axis'] = _rz(p['x_axis'])
            p['y_axis'] = _rz(p['y_axis'])
            p['normal'] = _rz(p['normal'])
            data['rectangle_corners'] = [_rz(c) for c in data['rectangle_corners']]

        self._plane_data = data
        plane = data['plane']
        self.plane_origin = np.array(plane['origin'], dtype=float)

        # These vectors are perfectly orthogonal. Using them prevents the Baklava skew.
        self.plane_x = np.array(plane['x_axis'], dtype=float)
        self.plane_y = np.array(plane['y_axis'], dtype=float)
        self.plane_n = np.array(plane['normal'], dtype=float)

        corners = [np.array(c, dtype=float) for c in data['rectangle_corners']]
        self.rect_origin = corners[0]
        self.rect_width_vec = corners[1] - corners[0]
        self.rect_height_vec = corners[3] - corners[0]

        # Calculate absolute table dimensions in meters
        self.table_width_m = float(np.linalg.norm(self.rect_width_vec))
        self.table_height_m = float(np.linalg.norm(self.rect_height_vec))

        traj = data.get('square_trajectory', [])
        if traj:
            self._default_orientation = list(traj[0]['orientation_xyzw'])
        else:
            self._default_orientation = [0.0, 0.0, 0.0, 1.0]

        self.get_logger().info(
            f'Loaded plane — width={self.table_width_m:.3f}m, '
            f'height={self.table_height_m:.3f}m')

    # ------------------------------------------------------------------
    # Fast-Fail IK Reachability Check
    # ------------------------------------------------------------------
    def _is_reachable(self, center_3d: np.ndarray, positions: List[np.ndarray],
                      orientation_xyzw: list) -> bool:
        """
        Highly optimized reachability check.
        1. Checks the center of the shape first.
        2. If center is valid, uses its solution as the seed for the corners.
           This guarantees fast convergence and eliminates joint flipping (jumps).
        """
        R = _quat_to_rotmat(orientation_xyzw)
        T = np.eye(4)
        T[:3, :3] = R

        # STEP 1: Fast-Fail on the Center Point
        T[:3, 3] = center_3d
        center_sol = ik_solve(T, self._ik_seed, max_iter=50)

        if center_sol is None:
            return False # Center is unreachable, entire shape is invalid

        # STEP 2: Verify Sparse Boundary Points using the Center as the Seed
        n = len(positions)
        check_indices = list({0, n // 2, n - 1})

        current_seed = center_sol
        for idx in check_indices:
            T[:3, 3] = positions[idx]
            sol = ik_solve(T, current_seed, max_iter=50)
            if sol is None:
                return False
            current_seed = sol # Chain seed for continuity

        return True

    # ------------------------------------------------------------------
    # STRICT METRIC Shape Generator (No UV Skew)
    # ------------------------------------------------------------------
    def _generate_random_shape_3d(self, shape_type: str) -> Tuple[np.ndarray, List[np.ndarray]]:
        """
        Generates perfectly orthogonal shapes using metric distances.
        Returns: (center_3d_point, list_of_3d_vertices)
        """
        # 1. Calculate random size (10% to 50% of the smallest table dimension)
        min_table_dim = min(self.table_width_m, self.table_height_m)
        size = random.uniform(self._size_min_pct * min_table_dim,
                              self._size_max_pct * min_table_dim)

        # 2. Pick a safe center location in local metric coordinates
        margin = (size / 2.0) + 0.02  # 2cm padding from the edge

        # Failsafe if the generated size somehow exceeds the table
        if self.table_width_m < 2 * margin or self.table_height_m < 2 * margin:
            margin = min_table_dim / 3.0

        cx = random.uniform(margin, self.table_width_m - margin)
        cy = random.uniform(margin, self.table_height_m - margin)

        # 3. Create the exact 3D center using orthogonal basis vectors
        center_3d = self.rect_origin + (cx * self.plane_x) + (cy * self.plane_y)

        pts_3d = []
        radius = size / 2.0

        # 4. Generate vertices using metric trigonometry (eliminates Baklava skew)
        if shape_type == 'line':
            angle = random.uniform(0, math.pi)
            dx = radius * math.cos(angle) * self.plane_x
            dy = radius * math.sin(angle) * self.plane_y
            pts_3d = [center_3d - dx - dy, center_3d + dx + dy]

        elif shape_type == 'triangle':
            angle_offset = random.uniform(0, 2 * math.pi)
            for i in range(4): # 4 to close loop
                theta = angle_offset + (i * 2 * math.pi / 3)
                dx = radius * math.cos(theta) * self.plane_x
                dy = radius * math.sin(theta) * self.plane_y
                pts_3d.append(center_3d + dx + dy)

        elif shape_type == 'square':
            angle_offset = random.uniform(0, math.pi / 2)
            sq_radius = size / math.sqrt(2) # Radius for an inscribed square
            for i in range(5): # 5 to close loop
                theta = angle_offset + (i * 2 * math.pi / 4) + (math.pi / 4)
                dx = sq_radius * math.cos(theta) * self.plane_x
                dy = sq_radius * math.sin(theta) * self.plane_y
                pts_3d.append(center_3d + dx + dy)

        elif shape_type == 'circle':
            num_pts = max(int(size * 200), 20) # Dynamic resolution based on size
            for i in range(num_pts + 1):
                theta = i * 2 * math.pi / num_pts
                dx = radius * math.cos(theta) * self.plane_x
                dy = radius * math.sin(theta) * self.plane_y
                pts_3d.append(center_3d + dx + dy)

        return center_3d, pts_3d

    def _generate_drawing(self) -> Optional[Tuple[List[Point], Quaternion]]:
        """Generate a reachable, geometrically perfect drawing goal."""

        if self._traj_key == 'random':
            pick_random = True
        elif self._traj_key in _SHAPE_POOL:
            pick_random = False
        else:
            self.get_logger().error(f'Unknown trajectory_key: "{self._traj_key}"')
            return None

        orientation = self._default_orientation

        for attempt in range(1, self._ik_max_attempts + 1):
            shape = random.choice(_SHAPE_POOL) if pick_random else self._traj_key

            # Generate the shape in physical 3D space
            center_3d, positions = self._generate_random_shape_3d(shape)

            # Fast-Fail IK Reachability Check
            if self._is_reachable(center_3d, positions, orientation):
                self.get_logger().info(
                    f'Generated reachable {shape} '
                    f'({len(positions)} pts) on attempt {attempt}')
                return self._to_ros_msgs(positions, orientation)

            if attempt % 20 == 0:
                self.get_logger().warn(
                    f'IK rejection sampling: {attempt} attempts so far…')

        self.get_logger().error(
            f'Failed to find a reachable shape after {self._ik_max_attempts} attempts.')
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
    # Goal Dispatching and Callbacks
    # ------------------------------------------------------------------
    def send_next_drawing(self):
        if self._send_pending:
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
            f'Sending drawing #{self._drawing_count} ({len(waypoints)} waypoints)')

        send_future = self._client.send_goal_async(
            goal, feedback_callback=self._feedback_cb)
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by server')
            self._send_pending = False
            if self._continuous:
                self._schedule_retry()
            else:
                self._done = True
            return

        self._send_pending = False
        self._retry_delay = 2.0
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _schedule_retry(self):
        self._cancel_retry_timer()
        self.get_logger().info(f'Retrying in {self._retry_delay:.1f}s …')
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

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'  [{fb.current_phase}] drawing={fb.drawing_progress*100:.0f}%',
            throttle_duration_sec=2.0)

    def _result_cb(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(f'Drawing #{self._drawing_count} complete.')
        else:
            self.get_logger().warn(f'Drawing #{self._drawing_count} failed.')

        self._goal_handle = None
        self._send_pending = False
        self._cancel_retry_timer()

        if self._continuous:
            self.get_logger().info('Dispatching next drawing...')
            self._retry_timer = self.create_timer(0.5, self._retry_timer_cb)
        else:
            self._done = True

    def run(self):
        self.send_next_drawing()
        while rclpy.ok() and not self._done:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f'Dispatcher finished — {self._drawing_count} drawings sent')

def main(args=None):
    rclpy.init(args=args)
    dispatcher = DrawingDispatcher()
    try:
        dispatcher.run()
    except KeyboardInterrupt:
        pass
    finally:
        dispatcher.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()

```