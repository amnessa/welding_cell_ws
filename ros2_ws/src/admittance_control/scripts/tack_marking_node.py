#!/usr/bin/env python3
"""Mark the tacks with the pen: transit, force-gated descent, dwell, retract.

Milestone 4 of notes/pen_marking_plan.md. Inputs are what the earlier steps wrote in the
save dir: `tack_reach.json` (poses on the elbow-up branch, one tilt/roll per seam),
`welding_tacks.json` + `assembly.json` (the parts, for the collision model). Motion goes
through the controller that is already active on the UR driver,
`scaled_joint_trajectory_controller`, as FollowJointTrajectory goals; the descent goal
is CANCELLED the moment the wrench exceeds `touch_force_n` (1.5 N, from
config/pen_tool.json) and the joints at that moment are the contact record. No Servo,
no controller switching.

    ros2 run admittance_control tack_marking_node.py --ros-args -p dry_run:=false -p stroke_mode:=tack
    ros2 service call /tack_marking/plan  std_srvs/srv/Trigger   # from the current joints
    ros2 service call /tack_marking/next  std_srvs/srv/Trigger   # one tack (transit, descend, dwell, retract)
    ros2 service call /tack_marking/all   std_srvs/srv/Trigger   # every remaining tack, then home
    ros2 service call /tack_marking/home  std_srvs/srv/Trigger
    ros2 service call /tack_marking/abort std_srvs/srv/Trigger   # cancel the running goal
    # -p stroke_mode:=tack   draw each tack's own segment (p0 -> p1, welding_tacks.json)
    # -p stroke_mode:=seam   draw the whole weldable seam (welding_seams.json), once per seam

`dry_run:=true` (default) computes and publishes everything (the planned tip path on
/tack_marking/tip_path, the log) and sends NO goal. Set it false only after the plan
has been looked at in RViz.

Safety gates, all of them before any goal: the current joints must be clear in the
collision model and on the locked branch; every trajectory's first point must be within
`max_joint_jump_rad` of the current joints; |F| above `abort_force_n` at ANY time
cancels whatever runs; the wrench bias is re-measured at each approach point over
`bias_window_s` before the descent.

The stroke (stroke_mode tack | seam) is contact-referenced: the descent finds the
surface at the stroke's start, the stroke runs at that depth plus `press_m` along the
pen, in `stroke_chunk_m` chunks; between chunks the depth moves by
`depth_gain_m_per_n` x (hold force - measured force along the pen), clipped, and two
chunks under `min_contact_force_n` stop it. Per-chunk forces and depths are recorded.

Writes `<save_dir>/tack_marks.json`: per tack the commanded point, the tip at contact
(FK of the joints when the goal was cancelled), the contact depth along the pen axis
(+ = surface met before the registered point), the force, and `no_contact` when the
descent ran to its overshoot without meeting anything.
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
from geometry_msgs.msg import Point, WrenchStamped
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))
sys.path.insert(0, str(PKG.parents[2] / "weld_generator"))

from admittance_control import seam_from_registration as sfr  # noqa: E402
from admittance_control.collision import CollisionModel, boxes_from_parts  # noqa: E402
from admittance_control.kinematics import ur5e_fk  # noqa: E402
from admittance_control.marking import (build_marking_plan, contact_depth_m, line_chain,  # noqa: E402
                                        plan_to_dict, stroke_chain, stroke_targets,
                                        time_descent, time_joint_path)
from admittance_control.tack_reach import (UR_ORDER, joint_state_to_ur_order,  # noqa: E402
                                           load_marking_config, same_branch)
from admittance_control.tool_model import load_tool_model  # noqa: E402
from admittance_control.weldgen_registry import load_registry  # noqa: E402


class TackMarkingNode(Node):
    def __init__(self) -> None:
        super().__init__('tack_marking')
        p = self.declare_parameter
        p('save_dir', str(PKG / 'scripts' / 'foundationpose_results'))
        p('registry', str(PKG / 'models' / 'weldgen_objects.json'))
        p('tool_config', '')
        p('marking_config', '')
        p('extrinsic_path', '')
        p('dry_run', True)
        p('action', '/scaled_joint_trajectory_controller/follow_joint_trajectory')
        p('wrench_topic', '/force_torque_sensor_broadcaster/wrench')
        p('joint_states_topic', '/joint_states')
        p('v_joint_rad_s', 0.3)            # transit speed cap
        p('v_tip_m_s', 0.02)               # descent tip speed
        p('overshoot_m', 0.003)
        p('dwell_s', 0.5)
        # the stroke (C2): what to draw after contact and how to hold the pen on it
        p('stroke_mode', 'dot')            # dot | tack (own segment) | seam (whole seam)
        p('v_stroke_m_s', 0.02)
        p('press_m', 0.001)                # into the surface, past the measured contact
        p('stroke_chunk_m', 0.01)          # depth is corrected between chunks
        p('hold_force_n', 0.0)             # 0 -> the touch force
        p('depth_gain_m_per_n', 0.0004)    # per chunk, clipped to +/- max_depth_step_m
        p('max_depth_step_m', 0.0005)
        p('min_contact_force_n', 0.4)      # below this for 2 chunks = pen lifted off -> stop
        p('abort_force_n', 8.0)
        p('bias_window_s', 0.5)
        p('max_joint_jump_rad', 0.35)

        self._save_dir = Path(str(self.get_parameter('save_dir').value))
        self._tool = load_tool_model(str(self.get_parameter('tool_config').value) or None,
                                     str(self.get_parameter('extrinsic_path').value) or None)
        self._cfg = load_marking_config(str(self.get_parameter('marking_config').value) or None)
        self._touch_force = float(self._tool.touch_force_n)
        self._model = None
        self._report = None
        self._plan = None
        self._next_step = 0
        self._marks: list[dict] = []

        self._q = None
        self._q_sim = None                 # where the DRY RUN pretends the arm is
        self._q_lock = threading.Lock()
        self._force = np.zeros(3)
        self._force_bias = np.zeros(3)
        self._force_lock = threading.Lock()
        self._force_trace = None           # list while a stroke chunk runs (bias removed)
        self._axis_sign = -1.0             # sign of the pen-axis force at contact (set then)
        self._goal_handle = None
        self._abort = False

        sub_cbg = ReentrantCallbackGroup()
        self._srv_cbg = MutuallyExclusiveCallbackGroup()
        self.create_subscription(JointState, str(self.get_parameter('joint_states_topic').value),
                                 self._on_joints, qos_profile_sensor_data, callback_group=sub_cbg)
        self.create_subscription(WrenchStamped, str(self.get_parameter('wrench_topic').value),
                                 self._on_wrench, qos_profile_sensor_data, callback_group=sub_cbg)
        self._client = ActionClient(self, FollowJointTrajectory, str(self.get_parameter('action').value),
                                    callback_group=sub_cbg)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._path_pub = self.create_publisher(Marker, '~/tip_path', latched)
        self._status_pub = self.create_publisher(String, '~/status', latched)
        for name, cb in (('~/plan', self._srv_plan), ('~/next', self._srv_next),
                         ('~/all', self._srv_all), ('~/home', self._srv_home),
                         ('~/abort', self._srv_abort)):
            self.create_service(Trigger, name, cb, callback_group=self._srv_cbg)
        self.get_logger().info(f"dry_run={self.dry_run}; touch {self._touch_force} N; "
                               f"action {self.get_parameter('action').value}")

    # ---------------------------------------------------------------- state ---------
    @property
    def dry_run(self) -> bool:
        return bool(self.get_parameter('dry_run').value)

    def _on_joints(self, msg: JointState) -> None:
        if all(n in msg.name for n in UR_ORDER):
            with self._q_lock:
                self._q = joint_state_to_ur_order(msg.name, msg.position)

    def _on_wrench(self, msg: WrenchStamped) -> None:
        f = msg.wrench.force
        with self._force_lock:
            self._force = np.array([f.x, f.y, f.z])
            if self._force_trace is not None:
                self._force_trace.append(self._force - self._force_bias)

    def _force_mag(self) -> float:
        with self._force_lock:
            return float(np.linalg.norm(self._force - self._force_bias))

    def _measure_bias(self) -> None:
        samples = []
        t_end = time.time() + float(self.get_parameter('bias_window_s').value)
        while time.time() < t_end:
            with self._force_lock:
                samples.append(self._force.copy())
            time.sleep(0.005)
        with self._force_lock:
            self._force_bias = np.mean(samples, axis=0) if samples else np.zeros(3)

    def _current_q(self) -> np.ndarray | None:
        with self._q_lock:
            return None if self._q is None else self._q.copy()

    def _where(self) -> np.ndarray | None:
        """The arm's joints for planning and gating: the real ones, or in a dry run the
        end of the last pretended motion (nothing moves, so /joint_states never follows)."""
        if self.dry_run and self._q_sim is not None:
            return self._q_sim.copy()
        return self._current_q()

    def _replan_transit(self, target: np.ndarray, label: str):
        """The plan froze the transit at the joints of `~/plan`; if the arm has been jogged
        since (a TCP calibration, freedrive), re-plan from where it is now instead of
        refusing. Returns (path or None, message)."""
        from admittance_control.marking import transit_path
        q_now = self._where()
        if q_now is None:
            return None, f'{label}: no joint states'
        if self._model.in_collision(q_now):
            return None, f'{label}: current joints are in collision - ' + self._model.report(q_now)
        path = transit_path(q_now, target, self._model)
        if path is None:
            return None, f'{label}: no collision-free transit from the current joints'
        return path, ''


    def _say(self, text: str) -> None:
        self.get_logger().info(text)
        self._status_pub.publish(String(data=text))

    # ---------------------------------------------------------------- scene ---------
    def _load_scene(self) -> str | None:
        try:
            report = json.loads((self._save_dir / 'tack_reach.json').read_text())
            assembly = json.loads((self._save_dir / 'assembly.json').read_text())
        except FileNotFoundError as exc:
            return f"missing input: {exc}"
        objects = [(o['model'], np.asarray(o['pose_static'], float).reshape(4, 4))
                   for o in assembly['objects']]
        parts = sfr.posed_parts(objects, load_registry(str(self.get_parameter('registry').value)))
        # the clearance the reachability report was made with (it may have been overridden
        # on its command line); the plan must judge with the same number or the two disagree
        rep_cfg = report.get('config', {})
        self._cfg.clearance_m = float(rep_cfg.get('clearance_m', self._cfg.clearance_m))
        if rep_cfg.get('table_z_m') is not None:
            self._cfg.table_z_m = float(rep_cfg['table_z_m'])
        self._model = CollisionModel(tool=self._tool, scene_boxes=boxes_from_parts(parts),
                                     table_z=self._cfg.table_z_m, clearance=self._cfg.clearance_m)
        self._report = report
        self._tacks_json = self._seams_json = None
        try:
            self._tacks_json = json.loads((self._save_dir / 'welding_tacks.json').read_text())['tacks']
            self._seams_json = json.loads((self._save_dir / 'welding_seams.json').read_text())['seams']
        except (FileNotFoundError, KeyError):
            pass
        return None

    # ---------------------------------------------------------------- services ------
    def _srv_plan(self, request, response):
        err = self._load_scene()
        q = self._current_q()
        if err or q is None:
            response.success = False
            response.message = err or 'no /joint_states yet'
            return response
        if not same_branch(q, self._cfg.branch_signature, self._cfg.branch_joints):
            response.success = False
            response.message = f'current joints are not on the locked branch {self._cfg.branch_signature}'
            return response
        if self._model.in_collision(q):
            response.success = False
            response.message = 'current joints are in collision: ' + self._model.report(q)
            return response
        mode = str(self.get_parameter('stroke_mode').value)
        if mode != 'dot' and (self._tacks_json is None or self._seams_json is None):
            response.success = False
            response.message = f'stroke_mode={mode} needs welding_tacks.json + welding_seams.json in the save dir'
            return response
        strokes = stroke_targets(mode, self._report['tacks'], self._tacks_json, self._seams_json)
        self._plan = build_marking_plan(self._report, self._tool, self._model, self._cfg, q,
                                        float(self.get_parameter('overshoot_m').value),
                                        strokes=strokes, stroke_mode=mode)
        self._drawn_seams = set()
        self._q_sim = q.copy()
        self._next_step = 0
        self._marks = []
        (self._save_dir / 'tack_marking_plan.json').write_text(json.dumps(plan_to_dict(self._plan), indent=1))
        self._publish_tip_path()
        response.success = self._plan.ok
        response.message = self._plan.summary()
        self._say(response.message)
        return response

    def _srv_next(self, request, response):
        ok, msg = self._run_next()
        response.success, response.message = ok, msg
        return response

    def _srv_all(self, request, response):
        msgs = []
        while self._plan is not None and self._next_step < len(self._plan.steps):
            ok, msg = self._run_next()
            msgs.append(msg)
            if not ok:
                response.success, response.message = False, ' | '.join(msgs)
                return response
        ok, msg = self._go_home()
        msgs.append(msg)
        response.success, response.message = ok, ' | '.join(msgs)
        return response

    def _srv_home(self, request, response):
        ok, msg = self._go_home()
        response.success, response.message = ok, msg
        return response

    def _srv_abort(self, request, response):
        self._abort = True
        self._cancel_goal()
        response.success, response.message = True, 'abort requested'
        return response

    # ---------------------------------------------------------------- execution -----
    def _run_next(self) -> tuple[bool, str]:
        if self._plan is None:
            return False, 'no plan; call ~/plan first'
        if self._next_step >= len(self._plan.steps):
            return False, 'all tacks done; call ~/home'
        step = self._plan.steps[self._next_step]
        if not (step.transit_ok and step.descent_ok):
            self._next_step += 1
            return False, f'tack {step.tack_id} skipped: {step.reason}'
        self._abort = False
        tag = f'tack {step.tack_id} (seam {step.seam_id} #{step.tack_no})'
        # 1. transit - from where the arm IS, which may not be where the plan started
        q_now = self._where()
        transit = step.transit
        if q_now is None or np.abs(q_now - transit[0]).max() > 1e-3:
            transit, err = self._replan_transit(step.q_app, f'{tag} transit')
            if transit is None:
                return False, err
        ok, msg = self._execute(time_joint_path(transit, float(self.get_parameter('v_joint_rad_s').value)),
                                f'{tag} transit', watch_touch=False)
        if not ok:
            return False, msg
        # 2. descent, force-gated
        self._measure_bias()
        timed = time_descent(step.descent, self._tool, float(self.get_parameter('v_tip_m_s').value))
        ok, msg, contact = self._execute(timed, f'{tag} descent', watch_touch=True, want_contact=True)
        if not ok and contact is None:
            return False, msg
        record = {'tack_id': step.tack_id, 'seam_id': step.seam_id, 'tack_no': step.tack_no,
                  'point_m': step.point_m.tolist(), 'axis_m': step.axis_m.tolist(),
                  'tack_point_m': None if step.tack_point_m is None else step.tack_point_m.tolist(),
                  'roll_deg': step.roll_deg, 'tilt_deg': step.tilt_deg, 'stroke_mode': step.stroke_mode,
                  'dry_run': self.dry_run, 'time': time.time()}
        if contact is not None:
            q_c, f_c = contact
            tip = self._tool.tip_in(ur5e_fk(q_c))
            record.update({'contact': True, 'q_contact': q_c.tolist(), 'tip_contact_m': tip.tolist(),
                           'force_n': f_c, 'contact_depth_m': contact_depth_m(step.point_m, tip, step.axis_m)})
            self._say(f"{tag}: contact at {f_c:.2f} N, depth {record['contact_depth_m'] * 1000:+.1f} mm "
                      f"along the pen axis (+ = surface before the registered point)")
        else:
            record.update({'contact': False, 'contact_depth_m': None})
            self._say(f'{tag}: NO CONTACT within the overshoot')
        # 3. the mark: a dot (dwell) or the stroke, contact-referenced
        if step.stroke_points_m is not None and record['contact']:
            if step.stroke_mode == 'seam' and step.seam_id in self._drawn_seams:
                self._say(f'{tag}: seam {step.seam_id} already drawn - skipping the stroke')
            else:
                record['stroke'] = self._stroke(step, contact, record['contact_depth_m'], tag)
                if step.stroke_mode == 'seam':
                    self._drawn_seams.add(step.seam_id)
        elif not self.dry_run:
            time.sleep(float(self.get_parameter('dwell_s').value))
        # 4. retract: straight back along the pen axis from wherever the pen is
        q_now = self._where()
        tip_now = self._tool.tip_in(ur5e_fk(q_now))
        lift = np.linspace(0.0, 1.0, 8)[1:, None] * (-self._cfg.standoff_m * step.axis_m)[None, :] + tip_now
        back = line_chain(self._tool, lift, step.axis_m, np.deg2rad(step.roll_deg), q_now, self._cfg)
        if back is None:                                     # fall back to the descent chain reversed
            k_stop = int(np.argmin([np.abs(q - q_now).max() for q in step.descent]))
            back = step.descent[:k_stop][::-1]
        ok, msg = self._execute(time_descent([q_now] + list(back), self._tool,
                                             float(self.get_parameter('v_tip_m_s').value)),
                                f'{tag} retract', watch_touch=False)
        self._marks.append(record)
        self._write_marks()
        self._next_step += 1
        return ok, f'{tag}: ' + ('contact ' if record['contact'] else 'no contact ') + msg

    def _stroke(self, step, contact, contact_depth: float, tag: str) -> dict:
        """Draw `step.stroke_points_m` from the contact: the surface is where the pen met
        it, so the stroke runs at (press - contact_depth) along the pen axis relative to
        the registered polyline, in chunks; between chunks the depth is corrected from
        the mean force along the pen (a first-order admittance at chunk rate), and two
        chunks without contact stop the stroke (the pen lifted off: a tilt larger than
        the press can follow)."""
        q_c, f_c = contact
        press = float(self.get_parameter('press_m').value)
        chunk_m = float(self.get_parameter('stroke_chunk_m').value)
        gain = float(self.get_parameter('depth_gain_m_per_n').value)
        max_step = float(self.get_parameter('max_depth_step_m').value)
        f_hold = float(self.get_parameter('hold_force_n').value) or self._touch_force
        f_min = float(self.get_parameter('min_contact_force_n').value)
        v = float(self.get_parameter('v_stroke_m_s').value)
        pts = step.stroke_points_m
        depth = press - float(contact_depth)
        # which way does the pen-axis force go when pressed? read it at the contact
        with self._force_lock:
            fz = float((self._force - self._force_bias)[2])
        self._axis_sign = -1.0 if fz < 0 else 1.0
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1); cum = np.concatenate([[0.0], np.cumsum(seg)])
        length = float(cum[-1])
        n_chunks = max(1, int(np.ceil(length / chunk_m)))
        out = {'mode': step.stroke_mode, 'length_mm': length * 1000.0, 'depth_start_mm': depth * 1000.0,
               'hold_force_n': f_hold, 'chunks': [], 'completed': False, 'reason': ''}
        q_prev = q_c
        lost = 0
        from admittance_control.marking import resample_polyline
        dense = resample_polyline(pts, 0.002)
        s_dense = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(dense, axis=0), axis=1))])
        for k in range(n_chunks):
            s0, s1 = k * length / n_chunks, (k + 1) * length / n_chunks
            sel = dense[(s_dense >= s0 - 1e-9) & (s_dense <= s1 + 1e-9)]
            if len(sel) < 2:
                continue
            chain = line_chain(self._tool, sel + depth * step.axis_m, step.axis_m,
                               np.deg2rad(step.roll_deg), q_prev, self._cfg)
            if chain is None:
                out['reason'] = f'chunk {k}: IK chain broke'
                break
            with self._force_lock:
                self._force_trace = []
            ok, msg = self._execute(time_descent([q_prev] + chain, self._tool, v),
                                    f'{tag} stroke chunk {k + 1}/{n_chunks}', watch_touch=False)
            with self._force_lock:
                trace = np.array(self._force_trace) if self._force_trace else np.zeros((0, 3))
                self._force_trace = None
            if not ok:
                out['reason'] = msg
                break
            f_axis = float(np.mean(self._axis_sign * trace[:, 2])) if len(trace) else f_hold
            f_mag = float(np.mean(np.linalg.norm(trace, axis=1))) if len(trace) else f_hold
            out['chunks'].append({'depth_mm': depth * 1000.0, 'force_axis_n': f_axis, 'force_mag_n': f_mag,
                                  'n_samples': int(len(trace))})
            if f_axis < f_min:
                lost += 1
                if lost >= 2:
                    out['reason'] = f'pen lifted off (force {f_axis:.2f} N for 2 chunks)'
                    break
            else:
                lost = 0
            depth += float(np.clip(gain * (f_hold - f_axis), -max_step, max_step))
            q_prev = chain[-1]
        else:
            out['completed'] = True
        self._say(f"{tag}: stroke {'done' if out['completed'] else 'STOPPED: ' + out['reason']}, "
                  f"{length * 1000:.0f} mm in {len(out['chunks'])}/{n_chunks} chunks, depth "
                  f"{out['depth_start_mm']:+.1f} -> {depth * 1000:+.1f} mm, force "
                  f"{np.mean([c['force_axis_n'] for c in out['chunks']]) if out['chunks'] else 0:.2f} N mean")
        return out

    def _go_home(self) -> tuple[bool, str]:
        if self._plan is None or self._plan.home_path is None:
            return False, 'no home path'
        path, err = self._replan_transit(self._cfg.home_q, 'home')
        if path is None:
            return False, err
        return self._execute(time_joint_path(path, float(self.get_parameter('v_joint_rad_s').value)),
                             'home', watch_touch=False)

    def _write_marks(self) -> None:
        (self._save_dir / 'tack_marks.json').write_text(json.dumps({
            'touch_force_n': self._touch_force, 'overshoot_m': float(self.get_parameter('overshoot_m').value),
            'sequence': 'seam by seam, tack_no ascending', 'marks': self._marks}, indent=1))

    def _execute(self, timed, label: str, watch_touch: bool, want_contact: bool = False):
        """Send one trajectory goal and wait; with `watch_touch` cancel at the touch force.
        Returns (ok, msg) or, with `want_contact`, (ok, msg, contact|None)."""
        q = self._where()
        if q is None:
            return (False, 'no joint states', None) if want_contact else (False, 'no joint states')
        jump = float(np.abs(timed[0][0] - q).max())
        if jump > float(self.get_parameter('max_joint_jump_rad').value):
            msg = f'{label}: first point is {np.degrees(jump):.0f} deg from the current joints - refused'
            return (False, msg, None) if want_contact else (False, msg)
        if self.dry_run:
            msg = f'{label}: dry run, {len(timed)} points, {timed[-1][1]:.1f} s'
            self._say(msg)
            if want_contact:
                # pretend the surface is where the model says: contact at the tack point,
                # which the chain reaches standoff/(standoff+overshoot) of the way down
                cfg, over = self._cfg, float(self.get_parameter('overshoot_m').value)
                k = int(round((len(timed) - 1) * cfg.standoff_m / (cfg.standoff_m + over)))
                self._q_sim = np.asarray(timed[k][0], float).copy()
                return True, msg, (timed[k][0], self._touch_force)
            self._q_sim = np.asarray(timed[-1][0], float).copy()
            return True, msg
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = list(UR_ORDER)
        for qk, tk in timed:
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in qk]
            sec = int(tk); pt.time_from_start = Duration(sec=sec, nanosec=int((tk - sec) * 1e9))
            traj.points.append(pt)
        goal.trajectory = traj
        if not self._client.wait_for_server(timeout_sec=2.0):
            msg = f'{label}: action server not available'
            return (False, msg, None) if want_contact else (False, msg)
        send = self._client.send_goal_async(goal)
        while not send.done():
            time.sleep(0.005)
        self._goal_handle = send.result()
        if not self._goal_handle.accepted:
            msg = (f'{label}: goal REJECTED by the controller before any motion. It prints the '
                   f'reason in the ur_control terminal; the usual one after using the pendant is '
                   f'that the External Control program is not running (press Play on the pendant), '
                   f'else the controller is inactive or the trajectory is malformed.')
            return (False, msg, None) if want_contact else (False, msg)
        result_fut = self._goal_handle.get_result_async()
        contact = None
        abort_force = float(self.get_parameter('abort_force_n').value)
        while not result_fut.done():
            f = self._force_mag()
            if self._abort or f > abort_force or (watch_touch and f > self._touch_force):
                q_c = self._current_q()
                self._cancel_goal()
                if watch_touch and f > self._touch_force and not self._abort and f <= abort_force:
                    contact = (q_c, f)
                    break
                msg = f'{label}: cancelled ({"abort" if self._abort else f"force {f:.1f} N"})'
                return (False, msg, None) if want_contact else (False, msg)
            time.sleep(0.002)
        if contact is None:
            res = result_fut.result()
            code = res.result.error_code if res is not None else -99
            ok = code == FollowJointTrajectory.Result.SUCCESSFUL
            msg = f'{label}: {"done" if ok else f"controller error {code}"}'
            return (ok, msg, None) if want_contact else (ok, msg)
        # wait for the cancel to settle so the joints are truly at rest
        time.sleep(0.1)
        return True, f'{label}: contact at {contact[1]:.2f} N', contact

    def _cancel_goal(self) -> None:
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass

    # ---------------------------------------------------------------- RViz ----------
    def _publish_tip_path(self) -> None:
        mk = Marker()
        mk.header.frame_id = 'base_link'
        mk.ns, mk.id, mk.type, mk.action = 'tip_path', 0, Marker.LINE_STRIP, Marker.ADD
        mk.scale.x = 0.002
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.2, 0.8, 1.0, 0.9
        mk.pose.orientation.w = 1.0
        for s in self._plan.steps:
            for q in list(s.transit) + list(s.descent) + list(s.descent[::-1]):
                t = self._tool.tip_in(ur5e_fk(q))
                mk.points.append(Point(x=float(t[0]), y=float(t[1]), z=float(t[2])))
        if self._plan.home_path:
            for q in self._plan.home_path:
                t = self._tool.tip_in(ur5e_fk(q))
                mk.points.append(Point(x=float(t[0]), y=float(t[1]), z=float(t[2])))
        self._path_pub.publish(mk)


def main() -> None:
    rclpy.init()
    node = TackMarkingNode()
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
