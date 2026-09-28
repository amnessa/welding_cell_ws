#!/usr/bin/env python3
"""Measure the table plane by touching it with the pen - no camera involved.

    # 1. empty table; jog the arm (pendant or freedrive) so the pen points roughly
    #    straight DOWN, 3-8 cm above the middle of the area to measure
    # 2. External Control running on the pendant (Play), then:
    ros2 run admittance_control table_touchoff.py --ros-args -p dry_run:=false
    ros2 service call /table_touchoff/run   std_srvs/srv/Trigger
    ros2 service call /table_touchoff/abort std_srvs/srv/Trigger     # any time

The pattern is a regular hexagon (`sides` 6, `edge_m` 10 cm, so its radius is also
10 cm) centred under the pen's current position, plus its centre - seven touches, so
the plane fit has four spare points and a real residual (three points fit any plane
exactly). `sides:=3 edge_m:=0.15` gives the triangle. At each spot:

    move at the current height to above the spot, pen vertical (tool yaw kept)
    measure the wrench bias in the air
    FAST touch: descend at `v_fast_m_s` (4 mm/s) until |F - bias| > touch force (1.5 N)
    release AT ONCE: straight up `backoff_m` from wherever the tip is, re-measure the bias
    SLOW touch: descend at `v_slow_m_s` (1 mm/s) until contact -> the recorded point
    release, then retract to the hover height

Why slow, and why the release is never blocked: the pen is not spring-loaded and the
table is rigid, so everything past the moment the force crosses 1.5 N (the arm needs
a few tens of ms to stop) is pressed straight into the force: at 10 mm/s the first run
read 2.66 N at detection and ~10 N once stopped, and the 8 N abort then cancelled the
back-off itself, leaving the pen pressed. Detection is still at 1.5 N; the PEAK is set
by approach speed x reaction time x stiffness. Moves away from the surface are guarded
by `release_abort_force_n` only.

The tip at contact is read from the UR driver's `tcp_pose_broadcaster` at the moment
the force crosses the threshold: the pendant's TCP is the pen tip, and the driver uses
the robot's CALIBRATED kinematics, so this avoids the ~3 mm of the planner's nominal
FK. The FK tip is recorded next to it (their difference is that FK gap). The UR `base`
frame is base_link rotated 180 deg about z: z is the same, x and y flip.

Result: the least-squares plane (height at the touches' centroid, tilt, RMSE over the
touches, and with `repeats` > 1 the per-spot repeatability), printed and written
to `<notebooks>/table_plane.json`, with the lines to paste: the ICP node's `ground_z_m`
and `marking.json`'s `table_z_m`.

Safety: `dry_run:=true` (default) plans and prints, sends nothing. A descent stops at
`max_travel_m` without contact (reported, retracted); |F| > `abort_force_n` cancels
anything; every trajectory's first point must be within `max_joint_jump_rad` of the
current joints (so the pen must start roughly vertical); IK is seeded from the current
joints and kept on their branch.
"""

from __future__ import annotations

import json
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

from admittance_control.kinematics import ur5e_fk  # noqa: E402
from admittance_control.marking import time_descent  # noqa: E402
from admittance_control.table_probe import fit_plane, repeatability, touch_pattern  # noqa: E402
from admittance_control.tack_reach import (UR_ORDER, MarkingConfig, _unwrap_to,  # noqa: E402
                                           joint_state_to_ur_order, solve_on_branch)
from admittance_control.tool_model import load_tool_model  # noqa: E402

Rz180 = np.diag([-1.0, -1.0, 1.0])                  # UR 'base' -> base_link


class TableTouchoff(Node):
    def __init__(self) -> None:
        super().__init__('table_touchoff')
        p = self.declare_parameter
        p('dry_run', True)
        p('sides', 6)                       # regular polygon: 6 = hexagon
        p('edge_m', 0.10)                   # edge length (hexagon: = its radius)
        p('with_centre', True)
        p('repeats', 1)                     # touches per spot (>1 gives repeatability)
        p('v_fast_m_s', 0.004)              # stiff pen on a rigid table: slow approaches
        p('v_slow_m_s', 0.001)              # keep the overshoot past 1.5 N small
        p('v_move_m_s', 0.03)
        p('backoff_m', 0.003)
        p('max_travel_m', 0.12)             # give up a descent after this much
        p('abort_force_n', 8.0)             # moves TOWARD the table
        p('release_abort_force_n', 40.0)    # moves AWAY from it (back off / retract)
        p('bias_window_s', 0.3)
        p('max_joint_jump_rad', 0.35)
        p('tool_config', '')
        p('action', '/scaled_joint_trajectory_controller/follow_joint_trajectory')
        p('wrench_topic', '/force_torque_sensor_broadcaster/wrench')
        p('tcp_pose_topic', '/tcp_pose_broadcaster/pose')
        p('out', str(PKG / 'notebooks' / 'table_plane.json'))

        self._tool = load_tool_model(str(self.get_parameter('tool_config').value) or None)
        self._touch = float(self._tool.touch_force_n)
        self._lock = threading.Lock()
        self._q = None
        self._tcp = None                    # tip position in base_link from the driver
        self._force = np.zeros(3)
        self._bias = np.zeros(3)
        self._goal = None
        self._abort = False
        cb = ReentrantCallbackGroup()
        self.create_subscription(JointState, '/joint_states', self._on_joints, qos_profile_sensor_data, callback_group=cb)
        self.create_subscription(WrenchStamped, str(self.get_parameter('wrench_topic').value),
                                 self._on_wrench, qos_profile_sensor_data, callback_group=cb)
        self.create_subscription(PoseStamped, str(self.get_parameter('tcp_pose_topic').value),
                                 self._on_tcp, qos_profile_sensor_data, callback_group=cb)
        self._client = ActionClient(self, FollowJointTrajectory, str(self.get_parameter('action').value),
                                    callback_group=cb)
        srv = MutuallyExclusiveCallbackGroup()
        self.create_service(Trigger, '~/run', self._srv_run, callback_group=srv)
        self.create_service(Trigger, '~/abort', self._srv_abort, callback_group=cb)
        self.get_logger().info(f'dry_run={self.dry_run}; pen down 3-8 cm above the table, then ~/run')

    # ------------------------------------------------------------------ state ----------
    @property
    def dry_run(self) -> bool:
        return bool(self.get_parameter('dry_run').value)

    def _on_joints(self, msg):
        if all(n in msg.name for n in UR_ORDER):
            with self._lock:
                self._q = joint_state_to_ur_order(msg.name, msg.position)

    def _on_wrench(self, msg):
        f = msg.wrench.force
        with self._lock:
            self._force = np.array([f.x, f.y, f.z])

    def _on_tcp(self, msg):
        p = msg.pose.position
        v = np.array([p.x, p.y, p.z])
        with self._lock:
            self._tcp = Rz180 @ v if msg.header.frame_id != 'base_link' else v

    def _get(self):
        with self._lock:
            return (None if self._q is None else self._q.copy(),
                    None if self._tcp is None else self._tcp.copy(),
                    float(np.linalg.norm(self._force - self._bias)))

    def _measure_bias(self):
        samples, t_end = [], time.time() + float(self.get_parameter('bias_window_s').value)
        while time.time() < t_end:
            with self._lock:
                samples.append(self._force.copy())
            time.sleep(0.004)
        with self._lock:
            self._bias = np.mean(samples, axis=0)

    # ------------------------------------------------------------------ motion ---------
    def _pose_down(self, tip: np.ndarray, x_hint: np.ndarray) -> np.ndarray:
        """tool0 pose with the pen straight down, its tip at `tip`, tool x along the
        horizontal projection of `x_hint` (the current tool yaw: no wrist spin)."""
        z = np.array([0.0, 0.0, -1.0])
        x = x_hint - (x_hint @ z) * z
        if np.linalg.norm(x) < 1e-6:
            x = np.array([1.0, 0.0, 0.0])
        x /= np.linalg.norm(x)
        R = np.column_stack([x, np.cross(z, x), z])
        T = np.eye(4); T[:3, :3] = R; T[:3, 3] = tip - R @ self._tool.tip_tool0
        return T

    def _chain(self, tips: np.ndarray, x_hint, q_seed, cfg):
        chain = [q_seed]
        for tip in tips:
            q = solve_on_branch(self._pose_down(tip, x_hint), [chain[-1]], cfg, n_random=0)
            if q is None:
                return None
            q = _unwrap_to(q, chain[-1])
            if np.abs(q - chain[-1]).max() > 0.3:
                return None
            chain.append(q)
        return chain

    def _line(self, a, b, step=0.002):
        n = max(2, int(np.ceil(np.linalg.norm(b - a) / step)) + 1)
        return a[None, :] + (b - a)[None, :] * np.linspace(0, 1, n)[1:, None]

    def _execute(self, chain, v, label, watch=False, away=False):
        """Send the chain as one goal at tip speed `v`; with `watch`, cancel at the touch
        force and return the driver's tip at that instant. -> (ok, msg, tip|None).

        `away=True` marks a move that LEAVES the surface (back off, retract): the normal
        abort force must not apply to it - after a touch on a rigid surface the pen can
        still be pressed at ~10 N, and blocking the move that releases it is what held
        the arm on the table on 2026-09-28. Only `release_abort_force_n` guards it."""
        q_now, _, _ = self._get()
        timed = time_descent(chain, self._tool, v)
        jump = float(np.abs(timed[0][0] - q_now).max())
        if jump > float(self.get_parameter('max_joint_jump_rad').value):
            return False, f'{label}: first point {np.degrees(jump):.0f} deg from the current joints (pen not vertical?) - refused', None
        if self.dry_run:
            return True, f'{label}: dry run, {len(timed)} pts, {timed[-1][1]:.1f} s', None
        traj = JointTrajectory(); traj.joint_names = list(UR_ORDER)
        for qk, tk in timed:
            pt = JointTrajectoryPoint(); pt.positions = [float(x) for x in qk]
            s = int(tk); pt.time_from_start = Duration(sec=s, nanosec=int((tk - s) * 1e9))
            traj.points.append(pt)
        goal = FollowJointTrajectory.Goal(); goal.trajectory = traj
        if not self._client.wait_for_server(timeout_sec=2.0):
            return False, f'{label}: action server not available', None
        fut = self._client.send_goal_async(goal)
        while not fut.done():
            time.sleep(0.005)
        self._goal = fut.result()
        if not self._goal.accepted:
            return False, f'{label}: goal rejected (External Control running on the pendant?)', None
        res = self._goal.get_result_async()
        abort_f = float(self.get_parameter('release_abort_force_n' if away else 'abort_force_n').value)
        peak = 0.0
        while not res.done():
            q, tcp, f = self._get()
            peak = max(peak, f)
            if self._abort or f > abort_f:
                self._goal.cancel_goal_async()
                return False, f'{label}: cancelled ({"abort" if self._abort else f"force {f:.1f} N"})', None
            if watch and f > self._touch:
                # no settling pause: the caller releases the pen at once
                self._goal.cancel_goal_async()
                tip_fk = self._tool.tip_in(ur5e_fk(q))
                return True, f'{label}: contact at {f:.2f} N', {'tcp': tcp, 'fk': tip_fk, 'force': f, 'q': q}
            time.sleep(0.002)
        tail = f' (peak {peak:.1f} N while leaving)' if away and peak > self._touch else ''
        return True, f'{label}: {"no contact" if watch else "done"}{tail}', None

    def _release(self, x_hint, cfg, rise_m: float, v: float, label: str):
        """Straight UP by `rise_m` from wherever the tip is now (FK-relative, so the
        nominal-vs-calibrated FK gap cannot turn it into a push), as an `away` move."""
        q_now = self._get()[0]
        tip = self._tool.tip_in(ur5e_fk(q_now))
        c = self._chain(self._line(tip, tip + np.array([0.0, 0.0, rise_m]), step=0.0005), x_hint, q_now, cfg)
        if c is None:
            return False, f'{label}: no IK for the release', None
        ok, msg, _ = self._execute(c, max(v, 0.004), label, away=True)
        return ok, msg, c[-1]

    # ------------------------------------------------------------------ the run --------
    def _srv_abort(self, request, response):
        self._abort = True
        if self._goal is not None:
            self._goal.cancel_goal_async()
        response.success, response.message = True, 'abort requested'
        return response

    def _srv_run(self, request, response):
        self._abort = False
        q0, _, _ = self._get()
        if q0 is None:
            response.success, response.message = False, 'no /joint_states'
            return response
        T0 = ur5e_fk(q0)
        tip0 = self._tool.tip_in(T0)
        if T0[:3, 2] @ np.array([0, 0, -1.0]) < np.cos(np.deg2rad(25)):
            response.success = False
            response.message = 'the pen is not pointing roughly down (> 25 deg off vertical): jog it first'
            return response
        cfg = MarkingConfig(home_q=q0.copy())                 # lock to the start pose's branch
        x_hint = T0[:3, 0]
        hover_z = tip0[2]
        spots = touch_pattern(tip0[:2], float(self.get_parameter('edge_m').value),
                              bool(self.get_parameter('with_centre').value),
                              sides=int(self.get_parameter('sides').value))
        reps = max(1, int(self.get_parameter('repeats').value))
        travel = float(self.get_parameter('max_travel_m').value)
        backoff = float(self.get_parameter('backoff_m').value)
        v_fast, v_slow, v_move = (float(self.get_parameter(k).value) for k in ('v_fast_m_s', 'v_slow_m_s', 'v_move_m_s'))
        log, points, groups = [], [], []
        q_prev, tip_prev = q0, tip0
        if self.dry_run:
            bad = []
            for i, xy in enumerate(spots):
                hover = np.array([xy[0], xy[1], hover_z])
                c1 = self._chain(self._line(tip_prev, hover), x_hint, q_prev, cfg)
                c2 = None if c1 is None else self._chain(self._line(hover, hover - [0, 0, travel]), x_hint, c1[-1], cfg)
                if c1 is None or c2 is None:
                    bad.append(i)
                    continue
                jump = float(np.abs(c1[1] - q0).max()) if i == 0 and len(c1) > 1 else 0.0
                log.append(f'spot {i} at ({xy[0]:.3f}, {xy[1]:.3f}): reachable'
                           + (f', first step {np.degrees(jump):.0f} deg' if i == 0 else ''))
                q_prev, tip_prev = c1[-1], hover
            response.success = not bad
            response.message = (f'dry run: {len(spots)} spots ({int(self.get_parameter("sides").value)}-gon, '
                                f'edge {float(self.get_parameter("edge_m").value) * 1000:.0f} mm'
                                f'{" + centre" if bool(self.get_parameter("with_centre").value) else ""}) around '
                                f'{np.round(tip0[:2], 3).tolist()}, hover z {hover_z:.3f} m, max travel '
                                f'{travel * 1000:.0f} mm; '
                                + ('ALL REACHABLE' if not bad else f'NO IK for spots {bad} - move the start') + ' | '
                                + ' | '.join(log))
            return response
        for i, xy in enumerate(spots):
            group = []
            hover = np.array([xy[0], xy[1], hover_z])
            chain = self._chain(self._line(tip_prev, hover), x_hint, q_prev, cfg)
            if chain is None:
                log.append(f'spot {i}: no IK to the hover point'); break
            ok, msg, _ = self._execute(chain, v_move, f'spot {i} move')
            log.append(msg)
            if not ok:
                break
            q_prev, tip_prev = chain[-1], hover
            for r in range(reps):
                self._measure_bias()
                down = self._chain(self._line(hover, hover - [0, 0, travel]), x_hint, q_prev, cfg)
                ok, msg, fast = self._execute(down, v_fast, f'spot {i}.{r} fast', watch=True)
                log.append(msg)
                if not ok or fast is None:
                    if ok:
                        log.append(f'spot {i}: no table within {travel * 1000:.0f} mm - check max_travel_m')
                    self._return(q_prev, x_hint, cfg, hover, v_move); break
                # release at once, straight up from where the tip is (not to an absolute z)
                ok, msg, q_up = self._release(x_hint, cfg, backoff, v_fast, f'spot {i}.{r} back off')
                log.append(msg)
                if not ok:
                    break
                self._measure_bias()
                up = self._tool.tip_in(ur5e_fk(q_up))
                slow = self._chain(self._line(up, up - [0, 0, 3 * backoff], step=0.0005), x_hint, q_up, cfg)
                ok, msg, hit = self._execute(slow, v_slow, f'spot {i}.{r} slow', watch=True)
                log.append(msg)
                if hit is not None:
                    ok_r, msg_r, _ = self._release(x_hint, cfg, backoff, v_fast, f'spot {i}.{r} release')
                    log.append(msg_r)
                if hit is not None:
                    tip = hit['tcp'] if hit['tcp'] is not None else hit['fk']
                    points.append(tip); group.append(tip)
                    log.append(f'   spot {i}.{r}: z = {tip[2] * 1000:.2f} mm (driver), FK says '
                               f'{hit["fk"][2] * 1000:.2f} mm, force {hit["force"]:.2f} N')
                self._return(self._get()[0], x_hint, cfg, hover, v_move)
                q_prev = self._chain(np.array([hover]), x_hint, q_prev, cfg)[-1]
            groups.append(np.array(group) if group else np.zeros((0, 3)))
            if self._abort:
                log.append('aborted'); break
        # back over the start
        if not self.dry_run and not self._abort:
            back = self._chain(self._line(tip_prev, tip0), x_hint, self._get()[0], cfg)
            if back is not None:
                log.append(self._execute(back, v_move, 'return to start')[1])
        if len(points) < 3:
            response.success, response.message = False, 'fewer than 3 touches: ' + ' | '.join(log)
            return response
        fit = fit_plane(np.array(points))
        rep = repeatability([g for g in groups if len(g)])
        out = Path(str(self.get_parameter('out').value))
        out.write_text(json.dumps({'written': time.strftime('%Y-%m-%d %H:%M:%S'), 'frame': 'base_link',
                                   'source': 'tcp_pose_broadcaster (pendant TCP = pen tip)',
                                   'points_m': [p.tolist() for p in points], 'plane': fit,
                                   'repeatability': rep, 'log': log}, indent=1))
        z = fit['z_at_centroid_m']
        response.success = True
        response.message = (
            f"table at z = {z * 1000:.2f} mm (base_link, at the touches' centroid), tilt "
            f"{fit['tilt_deg']:.2f} deg, RMSE {fit['rmse_mm']:.2f} mm over {fit['n_points']} touches"
            + (f", repeatability {rep['pooled_z_std_mm']:.2f} mm" if rep['pooled_z_std_mm'] is not None else '')
            + f". Wrote {out}. To use it: marking.json \"table_z_m\": {z:.4f}; the ICP ground cut belongs "
            f"just above the HOLDER tops (table + holder height - 2 mm), e.g. "
            f"ros2 param set /icp_pose_refiner ground_z_m {z + 0.005:.4f} to remove only the table.")
        self.get_logger().info(response.message)
        return response

    def _return(self, q_now, x_hint, cfg, hover, v):
        if self.dry_run or q_now is None:
            return
        tip = self._tool.tip_in(ur5e_fk(q_now))
        c = self._chain(self._line(tip, hover), x_hint, q_now, cfg)
        if c is not None:
            self._execute(c, v, 'retract', away=True)


def main() -> None:
    rclpy.init()
    node = TableTouchoff()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
