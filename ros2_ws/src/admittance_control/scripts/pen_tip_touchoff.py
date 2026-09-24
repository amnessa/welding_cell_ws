#!/usr/bin/env python3
"""Measure the pen tip in tool0 by touch-off - the classic 4-point TCP, no camera.

    ros2 run admittance_control pen_tip_touchoff.py
    # bring the pen tip onto ONE fixed point (a pencil dot, a punch mark, a corner) in
    # freedrive, from a clearly different wrist orientation each time, then:
    ros2 service call /pen_tip_touchoff/record std_srvs/srv/Trigger    # x 4 or more
    ros2 service call /pen_tip_touchoff/solve  std_srvs/srv/Trigger    # prints d, p, RMS
    ros2 service call /pen_tip_touchoff/clear  std_srvs/srv/Trigger

Each record stores the tool0 pose from /joint_states (FK, the same chain the planner
uses, so the number lands in the right frame). `solve` fits R_i d + t_i = p and prints
the tip `d` in tool0 as the `pen_tip_m` line for config/pen_tool.json, the touched point
`p` in base_link, and the RMS of the residuals (repeatability of the touches; aim for
under 1 mm). It writes <notebooks>/pen_tip_touchoff.json with everything. Tilt the wrist
by 30-60 degrees between touches: with nearly equal orientations the fit is singular
along the tip direction and the RMS says nothing.

The point `p` is also useful afterwards: it is a bench point whose base_link position
is known from the robot alone, so registering a part with a mark at `p` tests the
camera path with no unknowns on the pen side.
"""

from __future__ import annotations

import json
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

from admittance_control.kinematics import ur5e_fk  # noqa: E402
from admittance_control.tack_reach import UR_ORDER, joint_state_to_ur_order  # noqa: E402
from admittance_control.tool_model import solve_tip_offset  # noqa: E402


class PenTipTouchoff(Node):
    def __init__(self) -> None:
        super().__init__('pen_tip_touchoff')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('out', str(PKG / 'notebooks' / 'pen_tip_touchoff.json'))
        self._q = None
        self._lock = threading.Lock()
        self._poses: list[dict] = []
        self.create_subscription(JointState, str(self.get_parameter('joint_states_topic').value),
                                 self._on_joints, qos_profile_sensor_data)
        for name, cb in (('~/record', self._record), ('~/solve', self._solve), ('~/clear', self._clear)):
            self.create_service(Trigger, name, cb)
        self.get_logger().info('touch the same point from different orientations; ~/record each time')

    def _on_joints(self, msg: JointState) -> None:
        if all(n in msg.name for n in UR_ORDER):
            with self._lock:
                self._q = joint_state_to_ur_order(msg.name, msg.position)

    def _record(self, request, response):
        with self._lock:
            q = None if self._q is None else self._q.copy()
        if q is None:
            response.success, response.message = False, 'no /joint_states yet'
            return response
        T = ur5e_fk(q)
        self._poses.append({'q': q.tolist(), 'T_base_tool0': T.tolist(), 'time': time.time()})
        n = len(self._poses)
        note = ''
        if n >= 2:
            R0 = np.asarray(self._poses[0]['T_base_tool0'])[:3, :3]
            ang = np.degrees(np.arccos(np.clip((np.trace(R0.T @ T[:3, :3]) - 1) / 2, -1, 1)))
            note = f'; {ang:.0f} deg from the first orientation'
        response.success = True
        response.message = f'recorded pose {n} at tool0 {np.round(T[:3, 3], 4).tolist()}{note}'
        self.get_logger().info(response.message)
        return response

    def _solve(self, request, response):
        if len(self._poses) < 3:
            response.success, response.message = False, f'need >= 3 poses, have {len(self._poses)}'
            return response
        Ts = [np.asarray(p['T_base_tool0']) for p in self._poses]
        d, p, rms = solve_tip_offset(Ts)
        out = Path(str(self.get_parameter('out').value))
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps({'pen_tip_m': d.tolist(), 'point_base_m': p.tolist(), 'rms_mm': rms,
                                   'poses': self._poses}, indent=1))
        msg = (f'pen tip in tool0: {np.round(d, 4).tolist()} m (lateral {np.hypot(d[0], d[1]) * 1000:.1f} mm '
               f'off the flange axis, {d[2] * 1000:.1f} mm out); point in base_link {np.round(p, 4).tolist()}; '
               f'RMS {rms:.2f} mm over {len(Ts)} touches -> put "pen_tip_m": {np.round(d, 4).tolist()} '
               f'in config/pen_tool.json (and p1 of the pen capsule one radius short). Wrote {out}')
        response.success, response.message = rms < 2.0, msg
        self.get_logger().info(msg)
        return response

    def _clear(self, request, response):
        self._poses = []
        response.success, response.message = True, 'cleared'
        return response


def main() -> None:
    rclpy.init()
    node = PenTipTouchoff()
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
