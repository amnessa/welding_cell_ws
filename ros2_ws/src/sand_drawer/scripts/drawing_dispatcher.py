#!/usr/bin/env python3
"""
Drawing Dispatcher — Action client that feeds drawings to the action server.

Loads the plane JSON, generates drawing waypoints based on the selected
trajectory key, and dispatches them one at a time to the DrawingActionServer
via the ExecuteDrawing action interface.

Usage (via launch):
  ros2 launch sand_drawer sand_drawer.launch.py mode:=action

Usage (standalone):
  ros2 run sand_drawer drawing_dispatcher.py \
      --ros-args -p plane_json_file:=<path> -p trajectory_key:=line

Modes
-----
  Single (default):
      Sends one drawing and exits.
  Continuous (continuous:=true):
      After each drawing completes, sends the next from a queue.
      Currently re-sends the same drawing; future versions can generate
      random Zen shapes, RRT-based patterns, or GUI-drawn paths.

Subscribes to:  (none)
Publishes to:   (none — communicates via action client)
"""

import json
import math
import os
import sys
from typing import List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Quaternion
from rclpy.action import ActionClient
from rclpy.node import Node

from sand_drawer.action import ExecuteDrawing


class DrawingDispatcher(Node):
    """Generates drawing paths and dispatches them to the action server."""

    def __init__(self):
        super().__init__('drawing_dispatcher')

        # ---- parameters ----
        self.declare_parameter('plane_json_file', '')
        self.declare_parameter('trajectory_key', 'line')
        self.declare_parameter('line_u_start', 0.5)
        self.declare_parameter('line_v_start', 0.3)
        self.declare_parameter('line_u_end', 0.5)
        self.declare_parameter('line_v_end', 0.7)
        self.declare_parameter('continuous', False)

        self._traj_key = self.get_parameter('trajectory_key').value
        self._continuous = self.get_parameter('continuous').value
        self._line_u_start = self.get_parameter('line_u_start').value
        self._line_v_start = self.get_parameter('line_v_start').value
        self._line_u_end = self.get_parameter('line_u_end').value
        self._line_v_end = self.get_parameter('line_v_end').value

        # ---- load plane data (with frame correction) ----
        self._load_plane_json()

        # ---- action client ----
        self._client = ActionClient(
            self, ExecuteDrawing, 'execute_drawing')

        # ---- state ----
        self._goal_handle = None
        self._done = False
        self._drawing_count = 0
        self._send_pending = False      # guard against concurrent sends
        self._retry_timer = None        # one-shot retry timer
        self._retry_delay = 2.0         # seconds between retries
        self._max_retry_delay = 16.0    # exponential back-off cap

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

        # Store what we need
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
            # Fallback: compute from plane normal
            self._default_orientation = [0.0, 0.0, 0.0, 1.0]
            self.get_logger().warn(
                'No square_trajectory in JSON — using identity orientation')

        self.get_logger().info(
            f'Loaded plane with {len(data.get("square_trajectory", []))} '
            f'square wps, '
            f'{len(data.get("projected_vector_trajectory", []))} '
            f'projected wps')

    # ------------------------------------------------------------------
    # Generate drawing waypoints from the selected trajectory key
    # ------------------------------------------------------------------
    def _generate_drawing(self) -> Optional[
            tuple]:  # (List[Point], Quaternion)
        """Generate (waypoints, orientation) for the next drawing.

        Returns None if generation fails.
        """
        data = self._plane_data

        if self._traj_key == 'line':
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
                f'Generated line: UV ({self._line_u_start},{self._line_v_start})'
                f'→({self._line_u_end},{self._line_v_end})  '
                f'length={length:.3f}m')

        elif self._traj_key in ('square_trajectory',
                                'projected_vector_trajectory'):
            traj = data.get(self._traj_key, [])
            if not traj:
                self.get_logger().error(
                    f'No data under key "{self._traj_key}"')
                return None
            positions = [np.array(wp['position'], dtype=float)
                         for wp in traj]
            orientation = list(traj[0]['orientation_xyzw'])
            self.get_logger().info(
                f'Loaded {len(positions)} waypoints from "{self._traj_key}"')
        else:
            self.get_logger().error(
                f'Unknown trajectory_key: "{self._traj_key}"')
            return None

        # Convert to ROS messages
        waypoints = []
        for pos in positions:
            pt = Point()
            pt.x, pt.y, pt.z = float(pos[0]), float(pos[1]), float(pos[2])
            waypoints.append(pt)

        quat = Quaternion()
        quat.x = orientation[0]
        quat.y = orientation[1]
        quat.z = orientation[2]
        quat.w = orientation[3]

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

        # Cancel any outstanding retry timer
        self._cancel_retry_timer()

        result = self._generate_drawing()
        if result is None:
            self.get_logger().error('Failed to generate drawing — stopping')
            self._send_pending = False
            self._done = True
            return

        waypoints, orientation = result

        self.get_logger().info('Waiting for action server...')
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
            self._send_pending = False          # allow next attempt
            if self._continuous:
                self._schedule_retry()
            else:
                self._done = True
            return

        self.get_logger().info('Goal accepted — executing')
        self._send_pending = False              # clear guard (goal is in-flight)
        self._retry_delay = 2.0                 # reset back-off on success
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    # ------------------------------------------------------------------
    # One-shot retry helpers
    # ------------------------------------------------------------------
    def _schedule_retry(self):
        """Schedule a single retry with exponential back-off."""
        self._cancel_retry_timer()
        delay = self._retry_delay
        self.get_logger().info(f'Retrying in {delay:.1f}s …')
        self._retry_timer = self.create_timer(
            delay, self._retry_timer_cb)
        # Increase delay for next rejection (capped)
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
        self._send_pending = False          # ensure guard is cleared
        self._cancel_retry_timer()          # cancel any stale retry

        if self._continuous:
            self.get_logger().info('Dispatching next drawing (after 0.5s)...')
            # Brief pause to let the server's executor fully complete
            self._cancel_retry_timer()
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
