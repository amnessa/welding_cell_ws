#!/usr/bin/env python3
"""
Freedrive Plane Capture — Capture 4 end-effector poses on the real UR5e
to define a drawing plane, then solve and save the plane JSON.

This node uses ROS 2 **services** for interaction (no stdin needed),
so it works correctly when launched via ``ros2 launch``.

Usage (with UR driver running):
  # Terminal 1 — start the capture node (enables freedrive automatically):
  ros2 launch sand_drawer sand_drawer.launch.py mode:=freedrive_capture

  # Terminal 2 — capture each point by calling the service:
  ros2 service call /freedrive_plane_capture/capture_point std_srvs/srv/Trigger
  ros2 service call /freedrive_plane_capture/capture_point std_srvs/srv/Trigger
  ros2 service call /freedrive_plane_capture/capture_point std_srvs/srv/Trigger
  ros2 service call /freedrive_plane_capture/capture_point std_srvs/srv/Trigger
  # (auto-solves and saves after the 4th capture)

  # Other useful services:
  ros2 service call /freedrive_plane_capture/undo_last   std_srvs/srv/Trigger
  ros2 service call /freedrive_plane_capture/reset        std_srvs/srv/Trigger
  ros2 service call /freedrive_plane_capture/solve_and_save std_srvs/srv/Trigger
  ros2 service call /freedrive_plane_capture/finish       std_srvs/srv/Trigger

The 4 capture points define the rectangle corners in order:
  Point 1 — Right-Upper
  Point 2 — Right-Down
  Point 3 — Left-Upper
  Point 4 — Left-Down
"""

import json
import math
import os
import sys
import time
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from controller_manager_msgs.srv import SwitchController


# ---------------------------------------------------------------------------
# Default output path (same as plane_solver_node)
# ---------------------------------------------------------------------------
def _default_output_file() -> str:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    package_root = os.path.dirname(script_dir)
    if "/install/sand_drawer/lib/sand_drawer" in script_dir:
        workspace_root = script_dir.split("/install/sand_drawer/lib/sand_drawer")[0]
        source_package_root = os.path.join(workspace_root, "src", "sand_drawer")
        output_root = source_package_root
    else:
        output_root = package_root
    return os.path.join(output_root, "generated_planes", "sand_drawer_plane.json")


# ---------------------------------------------------------------------------
# Rotation matrix → quaternion
# ---------------------------------------------------------------------------
def rotmat_to_quat(R: np.ndarray) -> List[float]:
    m00, m01, m02 = R[0, 0], R[0, 1], R[0, 2]
    m10, m11, m12 = R[1, 0], R[1, 1], R[1, 2]
    m20, m21, m22 = R[2, 0], R[2, 1], R[2, 2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s; x = (m21 - m12) / s; y = (m02 - m20) / s; z = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (m21 - m12) / s; x = 0.25 * s; y = (m01 + m10) / s; z = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (m02 - m20) / s; x = (m01 + m10) / s; y = 0.25 * s; z = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (m10 - m01) / s; x = (m02 + m20) / s; y = (m12 + m21) / s; z = 0.25 * s
    q = np.array([x, y, z, w], dtype=float)
    q /= np.linalg.norm(q)
    return q.tolist()


# ---------------------------------------------------------------------------
# Point labels
# ---------------------------------------------------------------------------
POINT_LABELS = ["Right-Upper", "Right-Down", "Left-Upper", "Left-Down"]


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
class FreedrivePlaneCapture(Node):
    def __init__(self):
        super().__init__('freedrive_plane_capture')

        # Parameters
        self.declare_parameter('output_file', _default_output_file())
        self.declare_parameter('square_scale', 0.8)
        self.declare_parameter('vector_path_uv',
                               [0.2, 0.2, 0.8, 0.2, 0.8, 0.8, 0.2, 0.8, 0.2, 0.2])
        self.declare_parameter('tcp_pose_topic', '/tcp_pose_broadcaster/pose')
        self.declare_parameter('freedrive_controller', 'freedrive_mode_controller')
        self.declare_parameter('trajectory_controller', 'scaled_joint_trajectory_controller')
        self.declare_parameter('auto_enable_freedrive', True)

        self.output_file = self.get_parameter('output_file').value
        self.square_scale = float(self.get_parameter('square_scale').value)
        self.vector_path_uv = list(self.get_parameter('vector_path_uv').value)
        self.tcp_pose_topic = self.get_parameter('tcp_pose_topic').value
        self.freedrive_controller = self.get_parameter('freedrive_controller').value
        self.trajectory_controller = self.get_parameter('trajectory_controller').value
        self.auto_enable = self.get_parameter('auto_enable_freedrive').value

        # Callback group for anything that calls service clients
        self._cb_group = ReentrantCallbackGroup()

        # TCP pose subscriber
        self._latest_pose: Optional[PoseStamped] = None
        self._pose_sub = self.create_subscription(
            PoseStamped, self.tcp_pose_topic,
            self._pose_cb, 10)

        # Freedrive enable publisher (keepalive)
        self._freedrive_pub = self.create_publisher(
            Bool, f'/{self.freedrive_controller}/enable_freedrive_mode', 10)

        # Controller manager client (reentrant so it can be called from callbacks)
        self._switch_client = self.create_client(
            SwitchController, '/controller_manager/switch_controller',
            callback_group=self._cb_group)

        # Freedrive keepalive timer (10 Hz)
        self._freedrive_active = False
        self._keepalive_timer = self.create_timer(0.1, self._keepalive_cb)

        # Captured data
        self.captured_points: List[np.ndarray] = []
        self.captured_orientations: List[List[float]] = []
        self._solved = False

        # ---- Services (user calls these from another terminal) ----
        self.create_service(Trigger, '~/capture_point', self._srv_capture)
        self.create_service(Trigger, '~/solve_and_save', self._srv_solve)
        self.create_service(Trigger, '~/undo_last', self._srv_undo)
        self.create_service(Trigger, '~/reset', self._srv_reset)
        self.create_service(Trigger, '~/finish', self._srv_finish,
                           callback_group=self._cb_group)
        self.create_service(Trigger, '~/enable_freedrive',
                           self._srv_enable_freedrive,
                           callback_group=self._cb_group)
        self.create_service(Trigger, '~/disable_freedrive',
                           self._srv_disable_freedrive,
                           callback_group=self._cb_group)

        # Status logger (prints TCP position every 2 seconds)
        self._status_timer = self.create_timer(2.0, self._status_log)

        self.get_logger().info("=" * 60)
        self.get_logger().info("  FREEDRIVE PLANE CAPTURE")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"TCP pose topic: {self.tcp_pose_topic}")
        self.get_logger().info(f"Output file:    {self.output_file}")
        self.get_logger().info("")
        self.get_logger().info("SERVICES (call from another terminal):")
        self.get_logger().info("  Capture:  ros2 service call /freedrive_plane_capture/capture_point std_srvs/srv/Trigger")
        self.get_logger().info("  Undo:     ros2 service call /freedrive_plane_capture/undo_last std_srvs/srv/Trigger")
        self.get_logger().info("  Solve:    ros2 service call /freedrive_plane_capture/solve_and_save std_srvs/srv/Trigger")
        self.get_logger().info("  Reset:    ros2 service call /freedrive_plane_capture/reset std_srvs/srv/Trigger")
        self.get_logger().info("  Finish:   ros2 service call /freedrive_plane_capture/finish std_srvs/srv/Trigger")
        self.get_logger().info("=" * 60)

        # Auto-enable freedrive on startup (one-shot timer)
        if self.auto_enable:
            self._startup_timer = self.create_timer(
                1.0, self._startup_enable_freedrive,
                callback_group=self._cb_group)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _pose_cb(self, msg: PoseStamped):
        self._latest_pose = msg

    def _keepalive_cb(self):
        if self._freedrive_active:
            msg = Bool()
            msg.data = True
            self._freedrive_pub.publish(msg)

    def _status_log(self):
        tcp = self._get_current_tcp()
        if tcp is None:
            self.get_logger().info(
                "Waiting for TCP pose...", throttle_duration_sec=5.0)
            return
        pos, _ = tcp
        fd_str = "FREEDRIVE ON " if self._freedrive_active else "FREEDRIVE OFF"
        n = len(self.captured_points)
        next_label = POINT_LABELS[n] if n < 4 else "DONE"
        self.get_logger().info(
            f"[{fd_str}]  TCP: [{pos[0]:+.4f}, {pos[1]:+.4f}, {pos[2]:+.4f}]  "
            f"Captured: {n}/4  Next: {next_label}")

    def _startup_enable_freedrive(self):
        """One-shot timer to enable freedrive after node is fully started."""
        # Cancel this timer so it only fires once
        self._startup_timer.cancel()
        if not self._freedrive_active:
            ok = self._enable_freedrive()
            if ok:
                self.get_logger().info(
                    "Freedrive ACTIVE — move the robot by hand, "
                    "then call capture_point service.")
            else:
                self.get_logger().error(
                    "Failed to enable freedrive. "
                    "Is the External Control program running on the pendant?")

    # ------------------------------------------------------------------
    # Get current TCP pose
    # ------------------------------------------------------------------
    def _get_current_tcp(self) -> Optional[Tuple[np.ndarray, List[float]]]:
        if self._latest_pose is None:
            return None
        p = self._latest_pose.pose.position
        o = self._latest_pose.pose.orientation
        pos = np.array([p.x, p.y, p.z], dtype=float)
        quat = [o.x, o.y, o.z, o.w]
        return pos, quat

    # ------------------------------------------------------------------
    # Controller switching
    # ------------------------------------------------------------------
    def _switch_controllers(self, activate: List[str],
                            deactivate: List[str]) -> bool:
        if not self._switch_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                "controller_manager/switch_controller not available!")
            return False
        req = SwitchController.Request()
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True

        future = self._switch_client.call_async(req)
        # Poll until done (no nested spin — MultiThreadedExecutor
        # processes the response on another thread)
        deadline = time.time() + 10.0
        while not future.done():
            if time.time() > deadline:
                self.get_logger().error(
                    "Switch controller service call timed out.")
                return False
            time.sleep(0.05)
        if future.result() is None:
            self.get_logger().error("Switch controller service call failed.")
            return False
        if not future.result().ok:
            self.get_logger().error(
                f"Controller switch rejected: "
                f"activate={activate}, deactivate={deactivate}")
            return False
        self.get_logger().info(
            f"Controllers switched: +{activate}, -{deactivate}")
        return True

    def _enable_freedrive(self) -> bool:
        self.get_logger().info("Activating freedrive mode...")
        ok = self._switch_controllers(
            activate=[self.freedrive_controller],
            deactivate=[self.trajectory_controller])
        if ok:
            self._freedrive_active = True
            self.get_logger().info("Freedrive mode ACTIVE.")
        return ok

    def _disable_freedrive(self) -> bool:
        self.get_logger().info("Deactivating freedrive mode...")
        self._freedrive_active = False
        msg = Bool()
        msg.data = False
        self._freedrive_pub.publish(msg)
        time.sleep(0.2)
        ok = self._switch_controllers(
            activate=[self.trajectory_controller],
            deactivate=[self.freedrive_controller])
        if ok:
            self.get_logger().info(
                "Freedrive DISABLED — trajectory controller restored.")
        return ok

    # ------------------------------------------------------------------
    # Service handlers
    # ------------------------------------------------------------------
    def _srv_capture(self, request, response):
        """Capture the current TCP position as the next corner point."""
        tcp = self._get_current_tcp()
        if tcp is None:
            response.success = False
            response.message = \
                "No TCP pose available. Is the UR driver running?"
            return response

        if len(self.captured_points) >= 4:
            response.success = False
            response.message = (
                "Already have 4 points. "
                "Call /solve_and_save, /reset, or /undo_last.")
            return response

        pos, quat = tcp
        idx = len(self.captured_points)
        label = POINT_LABELS[idx]

        # Reject if too close to the previous point
        if self.captured_points:
            dist = float(np.linalg.norm(pos - self.captured_points[-1]))
            if dist < 0.01:
                response.success = False
                response.message = (
                    f"Too close to previous point ({dist:.4f} m). "
                    f"Move the robot further before capturing.")
                return response

        self.captured_points.append(pos.copy())
        self.captured_orientations.append(quat)
        self._solved = False

        n = len(self.captured_points)
        msg = (f"Captured {n}/4 ({label}): "
               f"[{pos[0]:+.4f}, {pos[1]:+.4f}, {pos[2]:+.4f}]")
        self.get_logger().info(msg)

        # Auto-solve after the 4th point
        if n == 4:
            ok, solve_msg = self._do_solve_and_save()
            msg += f"  |  Auto-solve: {solve_msg}"
            response.success = ok
        else:
            next_label = POINT_LABELS[n] if n < 4 else "DONE"
            msg += f"  |  Next: {next_label}"
            response.success = True

        response.message = msg
        return response

    def _srv_solve(self, request, response):
        """Solve the plane from captured points and save the JSON."""
        if len(self.captured_points) < 4:
            response.success = False
            response.message = \
                f"Need 4 points, have {len(self.captured_points)}."
            return response
        ok, msg = self._do_solve_and_save()
        response.success = ok
        response.message = msg
        return response

    def _srv_undo(self, request, response):
        """Remove the last captured point."""
        if not self.captured_points:
            response.success = False
            response.message = "No points to undo."
            return response
        removed = self.captured_points.pop()
        self.captured_orientations.pop()
        self._solved = False
        n = len(self.captured_points)
        next_label = POINT_LABELS[n] if n < 4 else "DONE"
        response.success = True
        response.message = (
            f"Removed [{removed[0]:+.4f}, {removed[1]:+.4f}, "
            f"{removed[2]:+.4f}]. Now {n}/4. Next: {next_label}")
        self.get_logger().info(response.message)
        return response

    def _srv_reset(self, request, response):
        """Clear all captured points."""
        self.captured_points.clear()
        self.captured_orientations.clear()
        self._solved = False
        response.success = True
        response.message = \
            "All points cleared. Ready for Point 1 (Right-Upper)."
        self.get_logger().info(response.message)
        return response

    def _srv_finish(self, request, response):
        """Disable freedrive and signal node to shut down."""
        self._disable_freedrive()
        response.success = True
        response.message = \
            "Freedrive disabled. You can now Ctrl+C the capture node."
        self.get_logger().info(response.message)
        return response

    def _srv_enable_freedrive(self, request, response):
        ok = self._enable_freedrive()
        response.success = ok
        response.message = \
            "Freedrive enabled." if ok else "Failed to enable freedrive."
        return response

    def _srv_disable_freedrive(self, request, response):
        ok = self._disable_freedrive()
        response.success = ok
        response.message = \
            "Freedrive disabled." if ok else "Failed to disable freedrive."
        return response

    # ------------------------------------------------------------------
    # Plane solving (same algorithm as plane_solver_node)
    # ------------------------------------------------------------------
    def _do_solve_and_save(self) -> Tuple[bool, str]:
        try:
            result = self._solve_plane(
                self.captured_points[0], self.captured_points[1],
                self.captured_points[2], self.captured_points[3])
        except ValueError as e:
            return False, f"Solve failed: {e}"
        self._solved = True
        normal = result['normal']
        rect = result['rectangle']
        width = float(np.linalg.norm(rect[1] - rect[0]))
        height = float(np.linalg.norm(rect[3] - rect[0]))
        self.get_logger().info(
            f"Plane solved! Normal=[{normal[0]:.4f}, {normal[1]:.4f}, "
            f"{normal[2]:.4f}]  Rectangle: {width:.4f} x {height:.4f} m")
        return self._save_result(result)

    def _solve_plane(self, p1, p2, p3, p4):
        eps = 1e-8
        x_axis = p2 - p1
        x_norm = np.linalg.norm(x_axis)
        if x_norm < eps:
            raise ValueError("Points 1 and 2 are too close.")
        x_axis /= x_norm

        y_seed = p3 - p1
        y_axis = y_seed - np.dot(y_seed, x_axis) * x_axis
        y_norm = np.linalg.norm(y_axis)
        if y_norm < eps:
            raise ValueError("Point 3 is collinear with 1-2.")
        y_axis /= y_norm

        normal = np.cross(x_axis, y_axis)
        n_norm = np.linalg.norm(normal)
        if n_norm < eps:
            raise ValueError("Cannot compute plane normal.")
        normal /= n_norm

        u4 = float(np.dot(p4 - p1, x_axis))
        v4 = float(np.dot(p4 - p1, y_axis))
        if abs(u4) < eps: u4 = x_norm
        if abs(v4) < eps: v4 = y_norm

        c1 = p1
        c2 = p1 + u4 * x_axis
        c4 = p1 + v4 * y_axis
        c3 = p1 + u4 * x_axis + v4 * y_axis
        rectangle = [c1, c2, c3, c4]

        width = abs(u4)
        height = abs(v4)
        side = min(width, height) * float(
            np.clip(self.square_scale, 0.05, 1.0))
        sx = 1.0 if u4 >= 0.0 else -1.0
        sy = 1.0 if v4 >= 0.0 else -1.0
        center = p1 + 0.5 * u4 * x_axis + 0.5 * v4 * y_axis
        half = side * 0.5

        s1 = center + (-sx * half) * x_axis + (-sy * half) * y_axis
        s2 = center + (sx * half) * x_axis + (-sy * half) * y_axis
        s3 = center + (sx * half) * x_axis + (sy * half) * y_axis
        s4 = center + (-sx * half) * x_axis + (sy * half) * y_axis
        square = [s1, s2, s3, s4, s1]

        quat = rotmat_to_quat(np.column_stack([x_axis, y_axis, normal]))
        return {
            'normal': normal, 'x_axis': x_axis, 'y_axis': y_axis,
            'rectangle': rectangle, 'square': square,
            'orientation_xyzw': quat,
        }

    def _project_vector_path(self, rectangle):
        p1 = rectangle[0]
        width_vec = rectangle[1] - rectangle[0]
        height_vec = rectangle[3] - rectangle[0]
        projected = []
        for idx in range(0, len(self.vector_path_uv), 2):
            u = float(np.clip(self.vector_path_uv[idx], 0.0, 1.0))
            v = float(np.clip(self.vector_path_uv[idx + 1], 0.0, 1.0))
            projected.append(p1 + u * width_vec + v * height_vec)
        return projected

    # ------------------------------------------------------------------
    # Save JSON (same format as plane_solver_node)
    # ------------------------------------------------------------------
    def _save_result(self, result) -> Tuple[bool, str]:
        projected_vector = self._project_vector_path(result['rectangle'])
        quat = result['orientation_xyzw']

        payload = {
            "target_frame": (self._latest_pose.header.frame_id
                             if self._latest_pose else "base"),
            "source_topic": self.tcp_pose_topic,
            "points_source": "freedrive_capture",
            "captured_points_base": [
                p.tolist() for p in self.captured_points],
            "plane": {
                "origin": self.captured_points[0].tolist(),
                "x_axis": result['x_axis'].tolist(),
                "y_axis": result['y_axis'].tolist(),
                "normal": result['normal'].tolist(),
            },
            "rectangle_corners": [
                p.tolist() for p in result['rectangle']],
            "square_trajectory": [
                {"position": p.tolist(), "orientation_xyzw": quat}
                for p in result['square']
            ],
            "vector_path_uv": self.vector_path_uv,
            "projected_vector_trajectory": [
                {"position": p.tolist(), "orientation_xyzw": quat}
                for p in projected_vector
            ],
        }

        try:
            out_dir = os.path.dirname(self.output_file)
            if out_dir:
                os.makedirs(out_dir, exist_ok=True)
            with open(self.output_file, "w", encoding="utf-8") as f:
                json.dump(payload, f, indent=2)
            msg = f"Plane saved to {self.output_file}"
            self.get_logger().info(msg)
            return True, msg
        except Exception as exc:
            return False, f"Failed to save: {exc}"


def main(args=None):
    rclpy.init(args=args)
    node = FreedrivePlaneCapture()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Best-effort freedrive disable (context may already be torn down)
        try:
            node._freedrive_active = False
            node._disable_freedrive()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
