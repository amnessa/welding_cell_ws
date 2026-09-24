#!/usr/bin/env python3
"""How much does the ICP pose move when nothing moves? The registration's noise floor.

    ros2 run admittance_control pose_jitter_probe.py --ros-args -p seconds:=30.0
    # with tracking running on a STATIONARY part and a stationary arm

Listens to the refined pose (`/perception/icp/refined_pose`, camera frame) for
`seconds` and prints the standard deviation and range of the position (mm) and of the
orientation (deg), plus a line for the base-plate reading of it: at the seam this
jitter enters the tack points directly. If it is millimetres, `save_object` should
average N tracked poses instead of taking the last one (an easy fix in the ICP node),
and the D435i's depth noise at the working distance is the cause to look at.
"""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def _quat_to_R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class PoseJitterProbe(Node):
    def __init__(self) -> None:
        super().__init__('pose_jitter_probe')
        self.declare_parameter('topic', '/perception/icp/refined_pose')
        self.declare_parameter('seconds', 30.0)
        self._t = []
        self._R = []
        self._t0 = None
        self.create_subscription(PoseStamped, str(self.get_parameter('topic').value),
                                 self._on_pose, qos_profile_sensor_data)
        self.create_timer(1.0, self._tick)
        self.get_logger().info(f"collecting {self.get_parameter('seconds').value} s of "
                               f"{self.get_parameter('topic').value} ...")

    def _on_pose(self, msg: PoseStamped) -> None:
        p, o = msg.pose.position, msg.pose.orientation
        self._t.append([p.x, p.y, p.z])
        self._R.append(_quat_to_R([o.x, o.y, o.z, o.w]))
        if self._t0 is None:
            self._t0 = self.get_clock().now()

    def _tick(self) -> None:
        if self._t0 is None:
            return
        elapsed = (self.get_clock().now() - self._t0).nanoseconds * 1e-9
        if elapsed < float(self.get_parameter('seconds').value):
            self.get_logger().info(f"{len(self._t)} poses, {elapsed:.0f} s")
            return
        t = np.array(self._t)
        Rs = np.array(self._R)
        R_mean = Rs.mean(0); U, _, Vt = np.linalg.svd(R_mean); R_mean = U @ Vt
        angs = [np.degrees(np.arccos(np.clip((np.trace(R_mean.T @ R) - 1) / 2, -1, 1))) for R in Rs]
        self.get_logger().info(
            f"\n{len(t)} poses over {elapsed:.0f} s (part and arm stationary):\n"
            f"  position std  {np.round(t.std(0) * 1000, 2)} mm, range {np.round((t.max(0) - t.min(0)) * 1000, 2)} mm\n"
            f"  orientation   std {np.std(angs):.3f} deg, max {np.max(angs):.3f} deg from the mean\n"
            f"  -> a {np.max(angs):.2f} deg swing moves a point 250 mm from the part's centre by "
            f"{250 * np.sin(np.deg2rad(np.max(angs))):.1f} mm")
        raise SystemExit(0)


def main() -> None:
    rclpy.init()
    node = PoseJitterProbe()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
