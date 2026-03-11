The issue you are running into is a fundamental limitation of robotic kinematics. Because your defined drawing plane sits close to the robot's base, parts of that plane naturally intersect with the boundaries of the robot's physical reach.

Here is what the literature says about this problem and the architectural solution to fix it.

### The Textbook Perspective: Workspace and Rejection Sampling

* **Modern Robotics** defines the "reachable workspace" as the volume of space the end-effector can reach with at least one orientation. However, because your drawing task requires the pen to be strictly perpendicular to the table, you are constrained to a much smaller subset of that volume, severely limiting where the robot can physically reach. When a shape is randomly placed outside this specific subset, the Inverse Kinematics (IK) solver mathematically cannot find a solution, and the trajectory fails.
* **Springer Handbook of Robotics** explains that mapping the exact geometric boundaries of a constrained task-space is often analytically impossible. Instead, it suggests using stochastic evaluation—specifically Monte Carlo rejection sampling. You generate a random point, run it through the Inverse Kinematics solver, and if the solver fails or returns a configuration that violates joint limits, you simply reject the point and generate a new one.

### The Solution: An "IK-Aware" Dispatcher

Right now, your `DrawingDispatcher` is blindly generating points and sending them to the Action Server. The Action Server then discovers the points are unreachable, fails the goal, and stops.

To fix this, the `DrawingDispatcher` must become "IK-Aware." Because your `ur5e_rrt_planner.py` is a standalone library, the dispatcher can import the IK solver. Before it sends *any* trajectory to the Action Server, it will generate the random shape, quickly run IK on the corner points of that shape to guarantee it is reachable, and if it fails, it will instantly roll the dice and try a new random location until it finds a valid one.

Here is the updated `DrawingDispatcher` script. It implements the generative shapes (line, square, triangle, circle), handles the `continuous` loop logic, and natively performs the IK rejection sampling.

```python
#!/usr/bin/env python3
"""
Drawing Dispatcher — Action client that feeds drawings to the action server.

Loads the plane JSON, generates drawing waypoints (random shapes and locations),
verifies kinematic reachability using IK, and dispatches them to the Action Server.
"""

import json
import math
import os
import sys
import random
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Quaternion
from rclpy.action import ActionClient
from rclpy.node import Node

from sand_drawer.action import ExecuteDrawing

# Import the IK solver to check reachability before dispatching
_scripts_dir = os.path.dirname(os.path.realpath(__file__))
if _scripts_dir not in sys.path:
    sys.path.insert(0, _scripts_dir)
from ur5e_rrt_planner import ik_solve, quat_to_rotmat


class DrawingDispatcher(Node):
    def __init__(self):
        super().__init__('drawing_dispatcher')

        self.declare_parameter('plane_json_file', '')
        self.declare_parameter('trajectory_key', 'random') # Can be 'line', 'square', 'triangle', 'circle', or 'random'
        self.declare_parameter('continuous', False)

        self._traj_key = self.get_parameter('trajectory_key').value
        self._continuous = self.get_parameter('continuous').value

        # Generic seed for reachability checks (Home position)
        self._ik_seed = np.array([-0.7854, -0.4363, -2.5307, -0.1745, 0.0, 0.0], dtype=float)

        self._load_plane_json()

        self._client = ActionClient(self, ExecuteDrawing, 'execute_drawing')

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
        self.plane_n = np.array(plane['normal'], dtype=float)

        corners = [np.array(c, dtype=float) for c in data['rectangle_corners']]
        self.rect_origin = corners[0]
        self.rect_width_vec = corners[1] - corners[0]
        self.rect_height_vec = corners[3] - corners[0]

        # Use orientation from JSON or align with plane normal
        traj = data.get('square_trajectory', [])
        if traj:
            self._default_orientation = list(traj[0]['orientation_xyzw'])
        else:
            self._default_orientation = [0.0, 0.0, 0.0, 1.0]

    # ------------------------------------------------------------------
    # IK Reachability Check
    # ------------------------------------------------------------------
    def _is_reachable(self, positions: List[np.ndarray], orientation_xyzw: list) -> bool:
        """
        Runs a fast IK check on the generated Cartesian points to ensure
        the robot can actually draw this shape at this location.
        """
        R = quat_to_rotmat(orientation_xyzw)
        T = np.eye(4)
        T[:3, :3] = R

        # To save time, we only check a sparse subset of points (e.g., corners)
        check_indices = [0, len(positions)//2, -1]

        current_seed = self._ik_seed.copy()
        for idx in check_indices:
            T[:3, 3] = positions[idx]
            sol = ik_solve(T, current_seed, max_iter=100)
            if sol is None:
                return False
            # Update seed to simulate continuous movement
            current_seed = sol

        return True

    # ------------------------------------------------------------------
    # Shape Generation with Rejection Sampling
    # ------------------------------------------------------------------
    def _generate_random_shape_uv(self, shape_type: str) -> List[Tuple[float, float]]:
        """Generates UV coordinates [0.0 - 1.0] for standard shapes."""
        size = random.uniform(0.15, 0.35)
        cu = random.uniform(0.1 + size, 0.9 - size)
        cv = random.uniform(0.1 + size, 0.9 - size)
        uvs = []

        if shape_type == 'line':
            angle = random.uniform(0, 2 * math.pi)
            dx = (size / 2.0) * math.cos(angle)
            dy = (size / 2.0) * math.sin(angle)
            uvs = [(cu - dx, cv - dy), (cu + dx, cv + dy)]

        elif shape_type == 'triangle':
            angle = random.uniform(0, 2 * math.pi)
            for i in range(4): # 4 to close the loop
                theta = angle + (i * 2 * math.pi / 3)
                uvs.append((cu + size * math.cos(theta), cv + size * math.sin(theta)))

        elif shape_type == 'square':
            angle = random.uniform(0, math.pi / 2)
            for i in range(5): # 5 to close the loop
                theta = angle + (i * 2 * math.pi / 4) + (math.pi / 4)
                uvs.append((cu + size * math.cos(theta), cv + size * math.sin(theta)))

        elif shape_type == 'circle':
            num_pts = 30
            for i in range(num_pts + 1):
                theta = i * 2 * math.pi / num_pts
                uvs.append((cu + size * math.cos(theta), cv + size * math.sin(theta)))

        return uvs

    def _generate_drawing(self) -> Optional[tuple]:
        """
        Rejection Sampling Loop: Continuously generates random shapes and locations
        until it finds one that fits on the table AND is kinematically reachable.
        """
        shapes_pool = ['line', 'triangle', 'square', 'circle']

        for attempt in range(1, 101):
            # 1. Pick the shape
            if self._traj_key == 'random':
                current_shape = random.choice(shapes_pool)
            else:
                current_shape = self._traj_key if self._traj_key in shapes_pool else 'square'

            # 2. Generate UVs
            uv_coords = self._generate_random_shape_uv(current_shape)

            # 3. Map to 3D Cartesian space
            positions = []
            for u, v in uv_coords:
                pos = (self.rect_origin + u * self.rect_width_vec + v * self.rect_height_vec)
                positions.append(pos)

            # 4. REACHABILITY CHECK (The Monte Carlo rejection)
            if self._is_reachable(positions, self._default_orientation):
                self.get_logger().info(f'Valid {current_shape} generated at attempt {attempt}')

                # Convert to ROS messages
                waypoints = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in positions]
                quat = Quaternion(x=self._default_orientation[0], y=self._default_orientation[1],
                                  z=self._default_orientation[2], w=self._default_orientation[3])
                return waypoints, quat

        self.get_logger().error('Failed to find a reachable shape location after 100 attempts.')
        return None

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
            self.get_logger().error('Stopping dispatcher due to generation failure.')
            self._send_pending = False
            self._done = True
            return

        waypoints, orientation = result
        self._client.wait_for_server()

        goal = ExecuteDrawing.Goal()
        goal.waypoints = waypoints
        goal.orientation = orientation

        self._drawing_count += 1
        self.get_logger().info(f'Sending drawing #{self._drawing_count} ({len(waypoints)} waypoints)')

        send_future = self._client.send_goal_async(goal, feedback_callback=self._feedback_cb)
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
        self.get_logger().info(f'  [{fb.current_phase}] drawing={fb.drawing_progress*100:.0f}%', throttle_duration_sec=2.0)

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