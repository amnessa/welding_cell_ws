#!/usr/bin/env python3
"""
Freedrive Plane Capture — Capture 4 end-effector poses on the real UR5e
to define a drawing plane, then solve and save the plane JSON.

Usage (with UR driver running):
  ros2 run sand_drawer freedrive_plane_capture.py

Workflow:
  1. Activates freedrive_mode_controller (deactivates scaled_joint_trajectory_controller)
  2. Enables freedrive so you can move the robot by hand
  3. Prompts you to move the TCP to 4 corner positions and press ENTER to capture
  4. Solves the plane + rectangle + trajectory and saves the same JSON as plane_solver_node
  5. Deactivates freedrive and restores the trajectory controller

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
import threading
import time
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
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
# Rotation matrix ↔ quaternion helpers
# ---------------------------------------------------------------------------
def quat_to_rotmat(q_xyzw) -> np.ndarray:
    x, y, z, w = q_xyzw
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),      1 - 2*(x*x + y*y)],
    ])


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
# Node
# ---------------------------------------------------------------------------
POINT_LABELS = ["Right-Upper", "Right-Down", "Left-Upper", "Left-Down"]


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

        self.output_file = self.get_parameter('output_file').value
        self.square_scale = float(self.get_parameter('square_scale').value)
        self.vector_path_uv = list(self.get_parameter('vector_path_uv').value)
        self.tcp_pose_topic = self.get_parameter('tcp_pose_topic').value
        self.freedrive_controller = self.get_parameter('freedrive_controller').value
        self.trajectory_controller = self.get_parameter('trajectory_controller').value

        # TCP pose subscriber
        self._latest_pose: Optional[PoseStamped] = None
        self._pose_sub = self.create_subscription(
            PoseStamped, self.tcp_pose_topic,
            self._pose_cb, 10)

        # Freedrive enable publisher
        self._freedrive_pub = self.create_publisher(
            Bool, f'/{self.freedrive_controller}/enable_freedrive_mode', 10)

        # Controller manager switch service client
        self._switch_client = self.create_client(
            SwitchController, '/controller_manager/switch_controller')

        # Freedrive keepalive timer (publishes True at 10 Hz while active)
        self._freedrive_active = False
        self._keepalive_timer = self.create_timer(0.1, self._keepalive_cb)

        # Captured data
        self.captured_points: List[np.ndarray] = []
        self.captured_orientations: List[List[float]] = []

        self.get_logger().info("Freedrive Plane Capture node ready.")
        self.get_logger().info(f"Listening for TCP pose on: {self.tcp_pose_topic}")
        self.get_logger().info(f"Output will be saved to: {self.output_file}")

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

    # ------------------------------------------------------------------
    # Controller switching
    # ------------------------------------------------------------------
    def _switch_controllers(self, activate: List[str], deactivate: List[str]) -> bool:
        if not self._switch_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("controller_manager/switch_controller service not available!")
            return False

        req = SwitchController.Request()
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True

        future = self._switch_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is None:
            self.get_logger().error("Switch controller service call failed.")
            return False

        if not future.result().ok:
            self.get_logger().error(
                f"Controller switch rejected: activate={activate}, deactivate={deactivate}")
            return False

        self.get_logger().info(
            f"Controllers switched: activated={activate}, deactivated={deactivate}")
        return True

    def enable_freedrive(self) -> bool:
        """Activate freedrive controller (deactivate trajectory controller first)."""
        self.get_logger().info("Activating freedrive mode...")
        ok = self._switch_controllers(
            activate=[self.freedrive_controller],
            deactivate=[self.trajectory_controller])
        if ok:
            self._freedrive_active = True
            time.sleep(0.3)  # let the controller settle
            self.get_logger().info("Freedrive mode ACTIVE — you can move the robot by hand.")
        return ok

    def disable_freedrive(self) -> bool:
        """Deactivate freedrive and restore trajectory controller."""
        self.get_logger().info("Deactivating freedrive mode...")
        # Stop sending enable keepalive
        self._freedrive_active = False
        # Explicitly send disable
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
    # Get current TCP position
    # ------------------------------------------------------------------
    def get_current_tcp(self) -> Optional[Tuple[np.ndarray, List[float]]]:
        """Return (position_xyz, orientation_xyzw) from the latest TCP pose."""
        if self._latest_pose is None:
            return None
        p = self._latest_pose.pose.position
        o = self._latest_pose.pose.orientation
        pos = np.array([p.x, p.y, p.z], dtype=float)
        quat = [o.x, o.y, o.z, o.w]
        return pos, quat

    # ------------------------------------------------------------------
    # Plane solving (same algorithm as plane_solver_node)
    # ------------------------------------------------------------------
    def solve_plane(self, p1, p2, p3, p4):
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
        side = min(width, height) * float(np.clip(self.square_scale, 0.05, 1.0))
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

    def project_vector_path(self, result, rectangle):
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
    def save_result(self, result) -> Tuple[bool, str]:
        projected_vector = self.project_vector_path(result, result['rectangle'])
        quat = result['orientation_xyzw']

        payload = {
            "target_frame": self._latest_pose.header.frame_id if self._latest_pose else "base",
            "source_topic": self.tcp_pose_topic,
            "points_source": "freedrive_capture",
            "captured_points_base": [p.tolist() for p in self.captured_points],
            "plane": {
                "origin": self.captured_points[0].tolist(),
                "x_axis": result['x_axis'].tolist(),
                "y_axis": result['y_axis'].tolist(),
                "normal": result['normal'].tolist(),
            },
            "rectangle_corners": [p.tolist() for p in result['rectangle']],
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

    # ------------------------------------------------------------------
    # Interactive capture loop (runs in a separate thread)
    # ------------------------------------------------------------------
    def run_capture_session(self):
        """Interactive terminal session: enable freedrive, capture 4 points,
        solve plane, save, disable freedrive."""

        print("\n" + "=" * 60)
        print("  FREEDRIVE PLANE CAPTURE")
        print("=" * 60)

        # Wait for first TCP pose
        print("\nWaiting for TCP pose data...")
        for _ in range(50):  # 5 seconds
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._latest_pose is not None:
                break

        if self._latest_pose is None:
            print("\n[ERROR] No TCP pose received. Is the UR driver running?")
            print("  Check: ros2 topic echo /tcp_pose_broadcaster/pose --once")
            return False

        tcp = self.get_current_tcp()
        print(f"\nCurrent TCP position: [{tcp[0][0]:.4f}, {tcp[0][1]:.4f}, {tcp[0][2]:.4f}]")
        print(f"Frame: {self._latest_pose.header.frame_id}")

        # Enable freedrive
        print("\n--- Enabling freedrive mode ---")
        # Spin so the keepalive timer and service calls work
        if not self.enable_freedrive():
            print("[ERROR] Failed to enable freedrive. Check controller_manager.")
            print("  You may need the External Control program running on the pendant.")
            return False

        print("\n" + "-" * 60)
        print("  FREEDRIVE IS ACTIVE — Move the robot by hand")
        print("-" * 60)
        print("\nYou will capture 4 corner points of the working plane.")
        print("Move the TCP to each corner and press ENTER to capture.")
        print("Press 'q' + ENTER to abort at any time.\n")

        self.captured_points.clear()
        self.captured_orientations.clear()

        for i in range(4):
            label = POINT_LABELS[i]
            while True:
                # Spin to get fresh poses
                rclpy.spin_once(self, timeout_sec=0.05)

                tcp = self.get_current_tcp()
                if tcp is None:
                    print(f"  [WARN] No TCP data — move robot or check driver")
                    time.sleep(0.5)
                    continue

                pos, quat = tcp
                print(f"\rPoint {i+1}/4 ({label})  TCP: "
                      f"[{pos[0]:+.4f}, {pos[1]:+.4f}, {pos[2]:+.4f}]  "
                      f"Press ENTER to capture, 'q' to abort", end="", flush=True)

                # Non-blocking input check
                import select
                readable, _, _ = select.select([sys.stdin], [], [], 0.1)
                if readable:
                    line = sys.stdin.readline().strip()
                    if line.lower() == 'q':
                        print("\n\nAborted by user.")
                        self.disable_freedrive()
                        return False

                    # Capture the point
                    # Get a fresh pose right at capture time
                    rclpy.spin_once(self, timeout_sec=0.05)
                    tcp = self.get_current_tcp()
                    if tcp is None:
                        print("\n  [WARN] Lost TCP data, try again.")
                        continue

                    pos, quat = tcp
                    self.captured_points.append(pos.copy())
                    self.captured_orientations.append(quat)

                    print(f"\n  ✓ Captured {label}: "
                          f"[{pos[0]:+.4f}, {pos[1]:+.4f}, {pos[2]:+.4f}]")

                    # Validate min distance from previous points
                    if len(self.captured_points) > 1:
                        prev = self.captured_points[-2]
                        dist = float(np.linalg.norm(pos - prev))
                        print(f"    Distance from previous: {dist:.4f} m")
                        if dist < 0.01:
                            print("    [WARN] Very close to previous point — "
                                  "consider recapturing.")
                    print()
                    break

        # Disable freedrive
        print("--- Disabling freedrive mode ---")
        self.disable_freedrive()

        # Solve
        print("\n" + "=" * 60)
        print("  SOLVING PLANE")
        print("=" * 60)

        print("\nCaptured points:")
        for i, (pt, label) in enumerate(zip(self.captured_points, POINT_LABELS)):
            print(f"  {i+1}. {label}: [{pt[0]:+.4f}, {pt[1]:+.4f}, {pt[2]:+.4f}]")

        try:
            result = self.solve_plane(
                self.captured_points[0], self.captured_points[1],
                self.captured_points[2], self.captured_points[3])
        except ValueError as e:
            print(f"\n[ERROR] Plane solving failed: {e}")
            return False

        print(f"\nPlane normal: [{result['normal'][0]:.4f}, "
              f"{result['normal'][1]:.4f}, {result['normal'][2]:.4f}]")
        print(f"X axis: [{result['x_axis'][0]:.4f}, "
              f"{result['x_axis'][1]:.4f}, {result['x_axis'][2]:.4f}]")
        print(f"Y axis: [{result['y_axis'][0]:.4f}, "
              f"{result['y_axis'][1]:.4f}, {result['y_axis'][2]:.4f}]")

        rect = result['rectangle']
        width = float(np.linalg.norm(rect[1] - rect[0]))
        height = float(np.linalg.norm(rect[3] - rect[0]))
        print(f"Rectangle: {width:.4f} x {height:.4f} m")

        # Save
        ok, msg = self.save_result(result)
        if ok:
            print(f"\n  ✓ {msg}")
        else:
            print(f"\n  ✗ {msg}")

        print("\n" + "=" * 60)
        print("  DONE")
        print("=" * 60)
        return ok


def main(args=None):
    rclpy.init(args=args)
    node = FreedrivePlaneCapture()

    # Run the interactive capture in the main thread
    # (we spin manually inside run_capture_session)
    try:
        success = node.run_capture_session()
    except KeyboardInterrupt:
        print("\n\nInterrupted — disabling freedrive...")
        node.disable_freedrive()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
