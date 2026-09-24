#!/usr/bin/env python3
"""Show the reachability plan in RViz: the pen axis at every tack, green or red.

    ros2 run admittance_control tack_reach_marker_node.py --ros-args \
  -p report:=src/admittance_control/scripts/foundationpose_results/tack_reach.json \
  -p extrinsic_path:=src/admittance_control/notebooks/T_tcp_to_cam.npy

    # RViz: MarkerArray on /tack_reach/markers (base_link)

Per tack: an arrow from the approach point down to the tack point along the pen axis
(green = reachable, red = not), a text label "seam.tack_no t<tilt> r<roll>" and, for
reachable tacks, the arm's collision capsules (grey) and the tool envelope (blue) at
the approach pose, so a tight clearance is visible where it happens. Pass
`-p extrinsic_path:=<T_tcp_to_cam.npy>` when running from the install space.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray

from admittance_control.collision import ur5e_capsules
from admittance_control.geometry import rotmat_to_quat
from admittance_control.kinematics import ur5e_link_frames
from admittance_control.tool_model import load_tool_model


class TackReachMarkerNode(Node):
    def __init__(self) -> None:
        super().__init__('tack_reach_marker')
        self.declare_parameter('report', '')
        self.declare_parameter('frame', 'base_link')
        self.declare_parameter('topic', '/tack_reach/markers')
        self.declare_parameter('show_arm', True)
        self.declare_parameter('tool_config', '')
        self.declare_parameter('extrinsic_path', '')
        path = str(self.get_parameter('report').value)
        if not path:
            self.get_logger().error('set -p report:=<tack_reach.json>')
            sys.exit(2)
        self._report = json.loads(Path(path).read_text())
        self._frame = str(self.get_parameter('frame').value)
        self._tool = load_tool_model(str(self.get_parameter('tool_config').value) or None,
                                     str(self.get_parameter('extrinsic_path').value) or None)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = self.create_publisher(MarkerArray, str(self.get_parameter('topic').value), latched)
        self._publish()
        self.create_timer(2.0, self._publish)
        n_ok = sum(1 for t in self._report['tacks'] if t['ok'])
        self.get_logger().info(f"{n_ok}/{len(self._report['tacks'])} tacks reachable; markers on "
                               f"{self.get_parameter('topic').value}")

    def _mk(self, mid, ns, rgb, a=1.0) -> Marker:
        mk = Marker()
        mk.header.frame_id = self._frame           # stamp 0: latest transform
        mk.ns, mk.id, mk.action = ns, mid, Marker.ADD
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = (*rgb, a)
        mk.pose.orientation.w = 1.0
        return mk

    def _publish(self) -> None:
        arr = MarkerArray()
        mid = 0
        standoff = float(self._report['config']['standoff_m'])
        for t in self._report['tacks']:
            p = np.asarray(t['point_m'], float)
            axis = np.asarray(t['axis_m'], float)
            app = p - standoff * axis
            colour = (0.1, 0.9, 0.2) if t['ok'] else (0.95, 0.15, 0.15)
            arrow = self._mk(mid, 'pen_axis', colour); mid += 1
            arrow.type = Marker.ARROW
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.003, 0.008, 0.012
            arrow.points = [Point(x=float(app[0]), y=float(app[1]), z=float(app[2])),
                            Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))]
            arr.markers.append(arrow)
            txt = self._mk(mid, 'label', (1.0, 1.0, 1.0)); mid += 1
            txt.type = Marker.TEXT_VIEW_FACING
            lab = app - 0.015 * axis
            txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = map(float, lab)
            txt.scale.z = 0.012
            roll = f" t{t['tilt_deg']:+.0f} r{t['roll_deg']:.0f}" if t['ok'] else ""
            txt.text = f"{t['seam_id']}.{t['tack_no']}{roll}" + ("" if t['ok'] else " X")
            arr.markers.append(txt)
            if t['ok'] and bool(self.get_parameter('show_arm').value) and t['q_app'] is not None:
                q_app = np.asarray(t['q_app'], float)
                for c in ur5e_capsules(q_app):
                    ln = self._mk(mid, f'arm_{t["tack_id"]}', (0.7, 0.7, 0.7), 0.25); mid += 1
                    ln.type = Marker.LINE_STRIP
                    ln.scale.x = 2.0 * c['radius']
                    ln.points = [Point(x=float(c['p0'][0]), y=float(c['p0'][1]), z=float(c['p0'][2])),
                                 Point(x=float(c['p1'][0]), y=float(c['p1'][1]), z=float(c['p1'][2]))]
                    arr.markers.append(ln)
                # the tool envelope at the approach pose (blue), where the clearance is decided
                T = ur5e_link_frames(q_app)['tool0']
                for prim in self._tool.primitives_in(T):
                    mk = self._mk(mid, f'tool_{t["tack_id"]}', (0.3, 0.6, 1.0), 0.35); mid += 1
                    if prim['type'] == 'capsule':
                        mk.type = Marker.LINE_STRIP
                        mk.scale.x = 2.0 * prim['radius']
                        mk.points = [Point(x=float(prim['p0'][0]), y=float(prim['p0'][1]), z=float(prim['p0'][2])),
                                     Point(x=float(prim['p1'][0]), y=float(prim['p1'][1]), z=float(prim['p1'][2]))]
                    else:
                        mk.type = Marker.CUBE
                        c = prim['centre']; q = rotmat_to_quat(np.asarray(prim['R']))
                        mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = map(float, c)
                        mk.pose.orientation.x, mk.pose.orientation.y, mk.pose.orientation.z, mk.pose.orientation.w = q
                        mk.scale.x, mk.scale.y, mk.scale.z = (float(2 * h) for h in prim['half'])
                    arr.markers.append(mk)
        self._pub.publish(arr)


def main() -> None:
    rclpy.init()
    node = TackReachMarkerNode()
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
