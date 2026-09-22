#!/usr/bin/env python3
"""Show the pen tool's collision envelope on the robot in RViz.

Publishes the primitives of `config/pen_tool.json` (holder, pen, camera arm, camera
body) as a latched MarkerArray in `tool0`, plus a sphere at the pen tip and, in green,
the CALIBRATED camera optical origin and axis - that green dot must sit on the lens face
of the blue camera box; if it does not, the hand-eye calibration is off. They ride along with the robot model, so a wrong number in the config
shows up as a box that does not cover the real bracket. Milestone 1 of
notes/pen_marking_plan.md: fix the envelope here before the collision model trusts it.

    ros2 run admittance_control tool_model_marker_node.py
    ros2 run admittance_control tool_model_marker_node.py --ros-args -p config:=<pen_tool.json>
    # RViz: add MarkerArray on /tool_model/markers

Parameters
----------
  config          : path to pen_tool.json ('' -> the package's config/)
  extrinsic_path  : hand-eye calibration .npy ('' -> the one the config names)
  topic           : /tool_model/markers
  alpha           : 0.45
"""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray

from admittance_control.geometry import rotmat_to_quat
from admittance_control.tool_model import load_tool_model


class ToolModelMarkerNode(Node):
    def __init__(self) -> None:
        super().__init__('tool_model_marker')
        self.declare_parameter('config', '')
        self.declare_parameter('extrinsic_path', '')
        self.declare_parameter('topic', '/tool_model/markers')
        self.declare_parameter('alpha', 0.45)
        cfg = str(self.get_parameter('config').value) or None
        ext = str(self.get_parameter('extrinsic_path').value) or None
        self._tool = load_tool_model(cfg, ext)
        self.get_logger().info(self._tool.describe())
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = self.create_publisher(MarkerArray, str(self.get_parameter('topic').value), latched)
        self._alpha = float(self.get_parameter('alpha').value)
        self._publish()
        self.create_timer(2.0, self._publish)           # re-stamp so RViz never drops it

    def _marker(self, mid: int, ns: str, rgb) -> Marker:
        mk = Marker()
        mk.header.frame_id = self._tool.parent_frame
        # stamp 0 = "latest transform": a stamp of now is newer than the last joint
        # state and RViz refuses to extrapolate ("No transform to fixed frame")
        mk.ns, mk.id, mk.action = ns, mid, Marker.ADD
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = (*rgb, self._alpha)
        mk.pose.orientation.w = 1.0
        return mk

    def _publish(self) -> None:
        arr = MarkerArray()
        mid = 0
        for p in self._tool.primitives_tool0():
            colour = (0.2, 0.6, 1.0) if p['name'] == 'camera_body' else (1.0, 0.6, 0.1)
            if p['type'] == 'capsule':
                a, b = np.asarray(p['p0']), np.asarray(p['p1'])
                mk = self._marker(mid, p['name'], colour); mid += 1
                mk.type = Marker.CYLINDER
                d = b - a; L = float(np.linalg.norm(d))
                z = d / L
                ref = np.array([1.0, 0, 0]) if abs(z[0]) < 0.9 else np.array([0, 1.0, 0])
                x = np.cross(ref, z); x /= np.linalg.norm(x); y = np.cross(z, x)
                q = rotmat_to_quat(np.column_stack([x, y, z]))
                c = 0.5 * (a + b)
                mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = map(float, c)
                mk.pose.orientation.x, mk.pose.orientation.y, mk.pose.orientation.z, mk.pose.orientation.w = q
                mk.scale.x = mk.scale.y = 2.0 * p['radius']; mk.scale.z = L
                arr.markers.append(mk)
                for end in (a, b):                       # the capsule's round ends
                    s = self._marker(mid, p['name'], colour); mid += 1
                    s.type = Marker.SPHERE
                    s.pose.position.x, s.pose.position.y, s.pose.position.z = map(float, end)
                    s.scale.x = s.scale.y = s.scale.z = 2.0 * p['radius']
                    arr.markers.append(s)
            else:
                mk = self._marker(mid, p['name'], colour); mid += 1
                mk.type = Marker.CUBE
                c = np.asarray(p['centre']); q = rotmat_to_quat(np.asarray(p['R']))
                mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = map(float, c)
                mk.pose.orientation.x, mk.pose.orientation.y, mk.pose.orientation.z, mk.pose.orientation.w = q
                mk.scale.x, mk.scale.y, mk.scale.z = (float(2 * h) for h in p['half'])
                arr.markers.append(mk)
        tip = self._marker(mid, 'pen_tip', (1.0, 0.1, 0.1)); mid += 1
        tip.type = Marker.SPHERE
        tip.color.a = 1.0
        tip.pose.position.x, tip.pose.position.y, tip.pose.position.z = map(float, self._tool.tip_tool0)
        tip.scale.x = tip.scale.y = tip.scale.z = 0.008
        arr.markers.append(tip)
        axis = self._marker(mid, 'pen_axis', (1.0, 0.1, 0.1)); mid += 1
        axis.type = Marker.ARROW
        axis.color.a = 1.0
        axis.scale.x, axis.scale.y, axis.scale.z = 0.003, 0.008, 0.012
        axis.points = [Point(x=0.0, y=0.0, z=0.0),
                       Point(x=float(self._tool.tip_tool0[0]), y=float(self._tool.tip_tool0[1]),
                             z=float(self._tool.tip_tool0[2]))]
        arr.markers.append(axis)
        if self._tool.T_tool0_cam is not None:
            # the CALIBRATED optical origin: must sit on the lens face of the blue box
            o = self._tool.T_tool0_cam[:3, 3]
            dot = self._marker(mid, 'camera_optical_origin', (0.1, 1.0, 0.3)); mid += 1
            dot.type = Marker.SPHERE
            dot.color.a = 1.0
            dot.pose.position.x, dot.pose.position.y, dot.pose.position.z = map(float, o)
            dot.scale.x = dot.scale.y = dot.scale.z = 0.01
            arr.markers.append(dot)
            ray = self._marker(mid, 'camera_optical_axis', (0.1, 1.0, 0.3)); mid += 1
            ray.type = Marker.ARROW
            ray.color.a = 1.0
            ray.scale.x, ray.scale.y, ray.scale.z = 0.003, 0.008, 0.012
            e = o + 0.05 * self._tool.T_tool0_cam[:3, 2]
            ray.points = [Point(x=float(o[0]), y=float(o[1]), z=float(o[2])),
                          Point(x=float(e[0]), y=float(e[1]), z=float(e[2]))]
            arr.markers.append(ray)
        self._pub.publish(arr)


def main() -> None:
    rclpy.init()
    node = ToolModelMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
