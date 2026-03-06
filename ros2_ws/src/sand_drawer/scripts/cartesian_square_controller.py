#!/usr/bin/env python3
"""
Cartesian Drawing Controller — Monolithic pre-compute architecture.

All motion phases (retract up, homing, RRT approach, descent, drawing, ascent)
are pre-computed into one dense joint-space trajectory BEFORE the robot moves.
This eliminates real-time IK from the control loop, guarantees seamlessly
stitched boundaries between phases, and applies a single global trapezoidal
velocity profile across the entire operation.

Architecture
------------
  PRE_COMPUTE  — Build entire joint path offline (retract up → home hold →
                 RRT → descent → drawing → ascent).  All IK solved here.
  EXECUTING    — Lightweight playback:
                   Sim:  time-based interpolation at execution_hz
                   Real: one JointTrajectory message → convergence monitoring
  DONE         — Hold position

References:
  Modern Robotics (Lynch & Park), Chapter 9 — Time Scaling of Trajectories
  Springer Handbook of Robotics (Siciliano & Khatib), Chapter 7

Publishes
---------
  /isaac_joint_commands  (sensor_msgs/JointState)

Subscribes
----------
  /isaac_joint_states    (sensor_msgs/JointState)
"""

import json
import math
import os
import sys
from enum import Enum, auto
from typing import List, Optional, Tuple

# Ensure sibling scripts are importable
_scripts_dir = os.path.dirname(os.path.realpath(__file__))
if _scripts_dir not in sys.path:
    sys.path.insert(0, _scripts_dir)

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# ---------------------------------------------------------------------------
# Quaternion / rotation helpers
# ---------------------------------------------------------------------------

def quat_to_rotmat(q_xyzw) -> np.ndarray:
    """Quaternion [x, y, z, w] → 3×3 rotation matrix."""
    x, y, z, w = q_xyzw
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),      1 - 2*(x*x + y*y)],
    ])


def rotmat_to_quat(R: np.ndarray) -> list:
    """3×3 rotation matrix → [x, y, z, w]."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w], dtype=float)
    q /= np.linalg.norm(q)
    return q.tolist()


# ---------------------------------------------------------------------------
# Cartesian interpolation with trapezoidal velocity profile
# ---------------------------------------------------------------------------

def _interpolate_cartesian_smooth(
    positions: List[np.ndarray],
    orientation_xyzw: list,
    v_max: float = 0.05,
    a_max: float = 0.05,
    dt: float = 1.0 / 100.0,
) -> List[Tuple[np.ndarray, list]]:
    """Dense Cartesian path with trapezoidal velocity profile per segment."""
    if not positions or len(positions) < 2:
        return [(positions[0].copy(), orientation_xyzw)] if positions else []

    waypoints: List[Tuple[np.ndarray, list]] = []

    for i in range(len(positions) - 1):
        p_start = positions[i]
        p_end = positions[i + 1]
        dist = float(np.linalg.norm(p_end - p_start))
        if dist < 1e-5:
            continue
        line_dir = (p_end - p_start) / dist

        t_accel = v_max / a_max
        d_accel = 0.5 * a_max * (t_accel ** 2)

        if 2 * d_accel > dist:
            d_accel = dist / 2.0
            t_accel = math.sqrt(2 * d_accel / a_max)
            actual_v_max = a_max * t_accel
            t_cruise = 0.0
            d_cruise = 0.0
        else:
            actual_v_max = v_max
            d_cruise = dist - 2 * d_accel
            t_cruise = d_cruise / actual_v_max

        total_time = 2 * t_accel + t_cruise
        n_steps = max(int(total_time / dt), 1)

        for step in range(n_steps):
            t = step * dt
            if t < t_accel:
                s = 0.5 * a_max * (t ** 2)
            elif t < t_accel + t_cruise:
                s = d_accel + actual_v_max * (t - t_accel)
            else:
                t_dec = t - t_accel - t_cruise
                s = d_accel + d_cruise + actual_v_max * t_dec - 0.5 * a_max * (t_dec ** 2)
            s = min(s, dist)
            waypoints.append((p_start + line_dir * s, orientation_xyzw))

    waypoints.append((positions[-1].copy(), orientation_xyzw))
    return waypoints


# ---------------------------------------------------------------------------
# Phase enum
# ---------------------------------------------------------------------------

class Phase(Enum):
    PRE_COMPUTE = auto()
    EXECUTING   = auto()
    DONE        = auto()


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class CartesianDrawController(Node):
    def __init__(self):
        super().__init__('cartesian_draw_controller')

        # ---- parameters ----
        self.declare_parameter('plane_json_file', '')
        self.declare_parameter('approach_height', 0.08)
        self.declare_parameter('max_linear_vel', 0.05)
        self.declare_parameter('max_linear_accel', 0.05)
        self.declare_parameter('approach_linear_vel', 0.20)
        self.declare_parameter('approach_linear_accel', 0.20)
        self.declare_parameter('ik_damping', 0.05)
        self.declare_parameter('execution_hz', 100.0)
        self.declare_parameter('waypoints_per_tick', 1)
        self.declare_parameter('loop_trajectory', False)
        self.declare_parameter('trajectory_key', 'line')
        self.declare_parameter('max_joint_step', 0.15)
        self.declare_parameter('shoulder_lift_max', 0.0)
        self.declare_parameter('shoulder_lift_min', -2.5)
        self.declare_parameter('elbow_max', -0.3)
        self.declare_parameter('elbow_min', -3.14)
        self.declare_parameter('ik_num_seeds', 30)
        self.declare_parameter('line_u_start', 0.5)
        self.declare_parameter('line_v_start', 0.3)
        self.declare_parameter('line_u_end', 0.5)
        self.declare_parameter('line_v_end', 0.7)
        self.declare_parameter('real_robot', False)
        self.declare_parameter('real_robot_joint_state_topic', '/joint_states')
        self.declare_parameter('real_robot_trajectory_topic',
                               '/scaled_joint_trajectory_controller/joint_trajectory')
        self.declare_parameter('real_robot_trajectory_duration', 0.2)
        self.declare_parameter('max_joint_speed_deg', 45.0)
        self.declare_parameter('max_joint_accel_deg', 40.0)

        self._load_params()
        self._load_plane_json()

        # ---- joint names & home configuration ----
        self._joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
        ]
        self._home_positions = np.array([
            -0.7854, -0.4363, -2.5307, -0.1745, 0.0, 0.0,
        ], dtype=float)

        # ---- subscribers ----
        self._last_joint_state: Optional[JointState] = None
        self._last_sim_joint_state: Optional[JointState] = None
        self._last_real_joint_state: Optional[JointState] = None
        self.joint_state_sub = self.create_subscription(
            JointState, '/isaac_joint_states',
            self._sim_joint_state_cb, 10)

        # ---- publisher ----
        self.joint_cmd_pub = self.create_publisher(
            JointState, '/isaac_joint_commands', 10)

        # ---- real-robot bridging ----
        self._real_robot = self.get_parameter('real_robot').value
        self._real_traj_pub = None
        self._real_converge_tol = math.radians(2.0)
        if self._real_robot:
            real_js_topic = self.get_parameter(
                'real_robot_joint_state_topic').value
            real_traj_topic = self.get_parameter(
                'real_robot_trajectory_topic').value
            self._real_traj_pub = self.create_publisher(
                JointTrajectory, real_traj_topic, 10)
            self._real_joint_sub = self.create_subscription(
                JointState, real_js_topic,
                self._real_joint_state_cb, 10)
            self.get_logger().info(
                f'REAL ROBOT mode: subscribing {real_js_topic}, '
                f'publishing {real_traj_topic}')

        # ---- speed / accel limits ----
        self._max_joint_speed_deg = float(
            self.get_parameter('max_joint_speed_deg').value)
        self._max_joint_speed_rad = math.radians(self._max_joint_speed_deg)
        self._max_joint_accel_deg = float(
            self.get_parameter('max_joint_accel_deg').value)
        self._max_joint_accel_rad = math.radians(self._max_joint_accel_deg)
        self._dt = 1.0 / self.execution_hz
        self._prev_cmd_q: Optional[np.ndarray] = None
        self.get_logger().info(
            f'Max joint speed: {self._max_joint_speed_deg:.1f} deg/s, '
            f'accel: {self._max_joint_accel_deg:.1f} deg/s²')

        # ---- safe retract / home hold settings ----
        self._retract_height = 0.15
        self._home_hold_sec = 0.5

        # ---- master trajectory (filled by _pre_compute) ----
        self._master_path: List[np.ndarray] = []
        self._master_times: List[float] = []
        self._draw_start_idx = 0
        self._draw_end_idx = 0
        self._traj_sent = False
        self._exec_start_time = None
        self._last_q: Optional[np.ndarray] = None

        # ---- IK diagnostics ----
        self._ik_fail_count = 0
        self._ik_jump_count = 0
        self._ik_cfg_count = 0
        self._ik_total_count = 0

        # ---- phase ----
        self.phase = Phase.PRE_COMPUTE

        # ---- control timer ----
        self.timer = self.create_timer(self._dt, self._control_loop)

        self.get_logger().info(
            f'Cartesian draw controller started — '
            f'{len(self.draw_positions)} waypoints ({self.traj_key}), '
            f'approach_height={self.approach_height:.3f} m, '
            f'execution_hz={self.execution_hz:.0f}')

    # ------------------------------------------------------------------
    # Parameter loading
    # ------------------------------------------------------------------
    def _load_params(self):
        g = self.get_parameter
        self.approach_height     = g('approach_height').value
        self.v_max               = g('max_linear_vel').value
        self.a_max               = g('max_linear_accel').value
        self.approach_v_max      = g('approach_linear_vel').value
        self.approach_a_max      = g('approach_linear_accel').value
        self.ik_damping          = g('ik_damping').value
        self.execution_hz        = g('execution_hz').value
        self.wp_per_tick         = g('waypoints_per_tick').value
        self.do_loop             = g('loop_trajectory').value
        self.traj_key            = g('trajectory_key').value
        self.max_joint_step      = g('max_joint_step').value
        self.shoulder_lift_max   = g('shoulder_lift_max').value
        self.shoulder_lift_min   = g('shoulder_lift_min').value
        self.elbow_max           = g('elbow_max').value
        self.elbow_min           = g('elbow_min').value
        self.ik_num_seeds        = g('ik_num_seeds').value
        self.line_u_start        = g('line_u_start').value
        self.line_v_start        = g('line_v_start').value
        self.line_u_end          = g('line_u_end').value
        self.line_v_end          = g('line_v_end').value

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

        # ---- plane geometry ----
        plane = data['plane']
        self.plane_origin = np.array(plane['origin'], dtype=float)
        self.plane_x = np.array(plane['x_axis'], dtype=float)
        self.plane_y = np.array(plane['y_axis'], dtype=float)
        self.plane_n = np.array(plane['normal'], dtype=float)

        # ---- rectangle bounds (for UV → world) ----
        corners = [np.array(c, dtype=float) for c in data['rectangle_corners']]
        self.rect_origin = corners[0]
        self.rect_width_vec = corners[1] - corners[0]
        self.rect_height_vec = corners[3] - corners[0]

        # ---- build trajectory ----
        if self.traj_key == 'line':
            p_start = (self.rect_origin
                       + self.line_u_start * self.rect_width_vec
                       + self.line_v_start * self.rect_height_vec)
            p_end = (self.rect_origin
                     + self.line_u_end * self.rect_width_vec
                     + self.line_v_end * self.rect_height_vec)
            self.draw_positions = [p_start, p_end]
            traj = data.get('square_trajectory', [])
            if traj:
                self.target_quat = list(traj[0]['orientation_xyzw'])
            else:
                raise RuntimeError('Need square_trajectory for orientation')
            self.get_logger().info(
                f'Line trajectory: UV ({self.line_u_start},{self.line_v_start})'
                f'→({self.line_u_end},{self.line_v_end})  '
                f'length={float(np.linalg.norm(p_end - p_start)):.3f}m')
        else:
            traj = data.get(self.traj_key) or data.get('square_trajectory', [])
            if not traj:
                raise RuntimeError(
                    f'No trajectory found under key "{self.traj_key}"')
            self.draw_positions: List[np.ndarray] = []
            for wp in traj:
                self.draw_positions.append(
                    np.array(wp['position'], dtype=float))
            self.target_quat = list(traj[0]['orientation_xyzw'])

        self.get_logger().info(
            f'Loaded {len(self.draw_positions)} draw waypoints '
            f'from "{self.traj_key}"')

    # ------------------------------------------------------------------
    # Joint state callbacks
    # ------------------------------------------------------------------
    def _sim_joint_state_cb(self, msg: JointState):
        self._last_sim_joint_state = msg
        if not self._real_robot:
            self._last_joint_state = msg

    def _real_joint_state_cb(self, msg: JointState):
        self._last_real_joint_state = msg
        if self._real_robot:
            self._last_joint_state = msg

    # ------------------------------------------------------------------
    # Joint helpers
    # ------------------------------------------------------------------
    def _get_ordered_joints(self) -> Optional[np.ndarray]:
        if self._last_joint_state is None:
            return None
        name_map = {n: p for n, p in
                    zip(self._last_joint_state.name,
                        self._last_joint_state.position)}
        try:
            return np.array([name_map[n] for n in self._joint_names])
        except KeyError:
            return None

    def _get_real_ordered_joints(self) -> Optional[np.ndarray]:
        if self._last_real_joint_state is None:
            return None
        name_map = {n: p for n, p in
                    zip(self._last_real_joint_state.name,
                        self._last_real_joint_state.position)}
        try:
            return np.array([name_map[n] for n in self._joint_names])
        except KeyError:
            return None

    def _real_robot_converged(self, target_q: np.ndarray) -> bool:
        if not self._real_robot:
            return True
        real_q = self._get_real_ordered_joints()
        if real_q is None:
            return False
        return float(np.max(np.abs(target_q - real_q))) < self._real_converge_tol

    def _mirror_real_to_sim(self):
        if not self._real_robot:
            return
        real_q = self._get_real_ordered_joints()
        if real_q is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._joint_names)
        msg.position = real_q.tolist()
        self.joint_cmd_pub.publish(msg)

    # ------------------------------------------------------------------
    # Trapezoidal timing for joint-space paths
    # ------------------------------------------------------------------
    def _compute_phase_timing(self, path: List[np.ndarray]) -> List[float]:
        """Trapezoidal velocity profile over one phase (segment group).

        Builds cumulative joint-space arc length, then maps a single
        accel → cruise → decel onto it.  Call once per logical phase
        (retract, homing, RRT, descent, drawing, ascent) so the robot
        smoothly ramps within each phase and naturally stops between them.
        """
        if len(path) <= 1:
            return [0.0] * len(path)
        v_max = self._max_joint_speed_rad
        a_max = self._max_joint_accel_rad

        # Cumulative arc length (bottleneck joint per step)
        cum = [0.0]
        for i in range(1, len(path)):
            d = float(np.max(np.abs(path[i] - path[i - 1])))
            cum.append(cum[-1] + d)
        total_dist = cum[-1]

        if total_dist < 1e-9:
            # Stationary (e.g. home hold) — uniform small steps
            return [i * 0.01 for i in range(len(path))]

        # Trapezoidal profile parameters
        d_accel = v_max * v_max / (2.0 * a_max)
        if 2.0 * d_accel >= total_dist:
            # Triangle profile — never reaches v_max
            d_accel = total_dist / 2.0
            v_peak = math.sqrt(2.0 * a_max * d_accel)
            t_accel = v_peak / a_max
            t_cruise = 0.0
        else:
            v_peak = v_max
            t_accel = v_max / a_max
            t_cruise = (total_dist - 2.0 * d_accel) / v_max
        d_decel_start = total_dist - d_accel

        # Map cumulative distance → time
        times = [0.0]
        for i in range(1, len(path)):
            s = cum[i]
            if s <= d_accel:
                t = math.sqrt(2.0 * s / a_max) if s > 0 else 0.0
            elif s <= d_decel_start:
                t = t_accel + (s - d_accel) / v_peak
            else:
                s_d = s - d_decel_start
                disc = max(v_peak * v_peak - 2.0 * a_max * s_d, 0.0)
                t = t_accel + t_cruise + (v_peak - math.sqrt(disc)) / a_max
            times.append(max(t, times[-1] + 1e-4))
        return times

    def _concat_phase_times(self, phases: list) -> Tuple[List[np.ndarray], List[float]]:
        """Concatenate multiple phases, each timed independently.

        Args:
            phases: list of (label, path_list) tuples
        Returns:
            (master_path, master_times)
        """
        master_path: List[np.ndarray] = []
        master_times: List[float] = []
        t_offset = 0.0
        for label, plist in phases:
            if not plist:
                continue
            phase_times = self._compute_phase_timing(plist)
            for j, (q, t) in enumerate(zip(plist, phase_times)):
                if j == 0 and master_path:
                    continue  # skip duplicate boundary point
                master_path.append(np.asarray(q, dtype=float))
                master_times.append(t_offset + t)
            if phase_times:
                t_offset = master_times[-1]
            dur = phase_times[-1] if phase_times else 0.0
            self.get_logger().info(
                f'  Phase timing: {label:15s}  {len(plist):4d} pts  '
                f'{dur:.2f}s')
        return master_path, master_times

    def _send_full_trajectory(self, path: List[np.ndarray],
                              times: List[float]) -> None:
        """Send an entire joint-space path as one multi-point JointTrajectory
        to the real robot."""
        if self._real_traj_pub is None:
            return
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = list(self._joint_names)
        for q, t in zip(path, times):
            pt = JointTrajectoryPoint()
            pt.positions = q.tolist()
            dur_sec = int(t)
            dur_nsec = int((t - dur_sec) * 1e9)
            pt.time_from_start = Duration(sec=dur_sec, nanosec=dur_nsec)
            traj.points.append(pt)
        self._real_traj_pub.publish(traj)
        self.get_logger().info(
            f'Sent full trajectory ({len(path)} pts, '
            f'{times[-1]:.1f}s) to real robot')

    # ------------------------------------------------------------------
    # Batch IK solver for a Cartesian waypoint sequence
    # ------------------------------------------------------------------
    def _ik_solve_cartesian_path(
        self,
        cart_wps: List[Tuple[np.ndarray, list]],
        q_seed: np.ndarray,
        label: str = 'path',
    ) -> List[np.ndarray]:
        """Solve IK for every (pos, quat) waypoint. Skips failed waypoints."""
        path: List[np.ndarray] = []
        seed = q_seed.copy()
        fails = 0
        for pos, quat in cart_wps:
            T = self._pose44(pos, quat)
            q_sol = self._ik(T, seed)
            if q_sol is not None:
                path.append(q_sol)
                seed = q_sol.copy()
            else:
                fails += 1
        self.get_logger().info(
            f'  {label}: IK solved {len(path)}/{len(cart_wps)} '
            f'({fails} fails)')
        return path

    # ------------------------------------------------------------------
    # Publish joint command (with velocity clamping for sim safety)
    # ------------------------------------------------------------------
    def _pub_joints(self, q: np.ndarray):
        if not self._real_robot:
            if self._prev_cmd_q is None:
                cur = self._get_ordered_joints()
                if cur is not None:
                    self._prev_cmd_q = cur.copy()
            if self._prev_cmd_q is not None:
                delta = q - self._prev_cmd_q
                max_delta = float(np.max(np.abs(delta)))
                max_allowed = self._max_joint_speed_rad * self._dt
                if max_delta > max_allowed and max_delta > 1e-9:
                    scale = max_allowed / max_delta
                    q = self._prev_cmd_q + delta * scale
            self._prev_cmd_q = q.copy()

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self._joint_names
            msg.position = q.tolist()
            self.joint_cmd_pub.publish(msg)
            return

        # real-robot mode: just mirror (trajectory already sent)
        self._mirror_real_to_sim()

    # ------------------------------------------------------------------
    # Static helper: 4×4 pose from position + quaternion
    # ------------------------------------------------------------------
    @staticmethod
    def _pose44(pos: np.ndarray, quat_xyzw: list) -> np.ndarray:
        T = np.eye(4)
        T[:3, :3] = quat_to_rotmat(quat_xyzw)
        T[:3, 3] = pos
        return T

    # ------------------------------------------------------------------
    # IK with joint-jump rejection + elbow-up constraint
    # ------------------------------------------------------------------
    def _ik(self, T_target: np.ndarray,
            q_seed: np.ndarray) -> Optional[np.ndarray]:
        from ur5e_rrt_planner import ik_solve

        self._ik_total_count += 1
        q_sol = ik_solve(T_target, q_seed, max_iter=300,
                         pos_tol=5e-4, orient_tol=1e-3,
                         damping=self.ik_damping)
        if q_sol is None:
            self._ik_fail_count += 1
            return None
        if not self._config_ok(q_sol):
            self._ik_cfg_count += 1
            return None
        joint_delta = np.abs(q_sol - q_seed)
        if float(np.max(joint_delta)) > self.max_joint_step:
            self._ik_jump_count += 1
            return None
        return q_sol

    def _config_ok(self, q: np.ndarray) -> bool:
        """Elbow-up configuration check."""
        if q[1] > self.shoulder_lift_max or q[1] < self.shoulder_lift_min:
            return False
        if q[2] > self.elbow_max or q[2] < self.elbow_min:
            return False
        return True

    # ------------------------------------------------------------------
    # Constrained multi-seed IK (for RRT goal)
    # ------------------------------------------------------------------
    def _constrained_ik_for_pose(
            self, T_target: np.ndarray) -> Optional[np.ndarray]:
        from ur5e_rrt_planner import ik_solve
        import random

        home = self._home_positions.copy()
        candidates = []
        seeds = [home.copy()]
        for _ in range(self.ik_num_seeds):
            s = home.copy()
            s[0] += random.uniform(-1.5, 1.5)
            s[1] += random.uniform(-1.0, 0.3)
            s[2] += random.uniform(-0.5, 0.5)
            s[3] += random.uniform(-1.0, 1.0)
            s[4] += random.uniform(-1.0, 1.0)
            s[5] += random.uniform(-1.0, 1.0)
            seeds.append(s)

        for seed in seeds:
            q_sol = ik_solve(T_target, seed, max_iter=300,
                             pos_tol=5e-4, orient_tol=1e-3,
                             damping=self.ik_damping)
            if q_sol is not None and self._config_ok(q_sol):
                dist_to_home = float(np.linalg.norm(q_sol - home))
                candidates.append((dist_to_home, q_sol))

        if not candidates:
            self.get_logger().error(
                f'Constrained IK failed: {len(seeds)} seeds, none satisfy '
                f'elbow-up constraints')
            return None

        candidates.sort(key=lambda x: x[0])
        best_q = candidates[0][1]
        self.get_logger().info(
            f'  Constrained IK: {len(candidates)}/{len(seeds)} valid  '
            f'best shoulder_lift={best_q[1]:.3f} elbow={best_q[2]:.3f}')
        return best_q

    # ------------------------------------------------------------------
    # FK helper
    # ------------------------------------------------------------------
    def _fk_position(self, q: np.ndarray) -> np.ndarray:
        from ur5e_rrt_planner import ur5e_fk
        return ur5e_fk(q)[:3, 3]

    # ------------------------------------------------------------------
    # PRE_COMPUTE — build the entire trajectory offline
    # ------------------------------------------------------------------
    def _pre_compute(self):
        """Build a single dense joint-space trajectory covering all phases:
        retract up → retract home → home hold → RRT approach → descent →
        drawing → ascent.  All IK is solved here, before any motion."""
        from ur5e_rrt_planner import (ur5e_fk, rrt_connect,
                                      smooth_path, bezier_smooth_path)

        q_current = self._get_ordered_joints()
        if q_current is None:
            self.get_logger().warn(
                'PRE_COMPUTE — waiting for joint states…',
                throttle_duration_sec=2.0)
            return

        self.get_logger().info(
            '═══════════════════════════════════════════════\n'
            '  PRE-COMPUTING ENTIRE TRAJECTORY OFFLINE\n'
            '═══════════════════════════════════════════════')

        full_path: List[np.ndarray] = []
        cart_dt = 1.0 / self.execution_hz

        # ── SEGMENT 1: RETRACT UP (Cartesian vertical lift → IK) ─────
        T_now = ur5e_fk(q_current)
        current_pos = T_now[:3, 3].copy()
        current_quat = rotmat_to_quat(T_now[:3, :3])

        lift_pos = current_pos.copy()
        lift_pos[2] += self._retract_height

        self.get_logger().info(
            f'  [1/6] Retract up: z={current_pos[2]:.3f} → '
            f'z={lift_pos[2]:.3f} (+{self._retract_height:.3f}m)')

        retract_wps = _interpolate_cartesian_smooth(
            [current_pos, lift_pos], current_quat,
            self.approach_v_max, self.approach_a_max, cart_dt)
        retract_path = self._ik_solve_cartesian_path(
            retract_wps, q_current, 'Retract up')
        if not retract_path:
            self.get_logger().error(
                'PRE_COMPUTE FAILED at retract up — retrying…')
            return
        full_path.extend(retract_path)

        # ── SEGMENT 2: RETRACT HOME (cosine joint-space interpolation) ──
        q_retracted = full_path[-1]
        q_home = self._home_positions.copy()
        n_home = max(int(1.0 * self.execution_hz), 20)

        retract_home_path = []
        for i in range(1, n_home + 1):
            alpha = 0.5 * (1.0 - math.cos(math.pi * i / n_home))
            q_interp = (1.0 - alpha) * q_retracted + alpha * q_home
            retract_home_path.append(q_interp)
            full_path.append(q_interp)
        # Include the starting point for phase timing
        retract_home_path.insert(0, q_retracted.copy())

        self.get_logger().info(
            f'  [2/6] Retract home: {n_home} joint-interp steps')

        # ── SEGMENT 3: HOME HOLD (hold at home for stability) ────────
        n_hold = max(int(self._home_hold_sec * self.execution_hz), 1)
        home_hold_path = [q_home.copy() for _ in range(n_hold + 1)]
        for _ in range(n_hold):
            full_path.append(q_home.copy())

        self.get_logger().info(
            f'  [3/6] Home hold: {self._home_hold_sec:.1f}s '
            f'({n_hold} waypoints)')

        # ── SEGMENT 4: RRT (home → approach pose above first point) ──
        approach_pos = (self.draw_positions[0]
                        - self.approach_height * self.plane_n)
        T_approach = self._pose44(approach_pos, self.target_quat)

        self.get_logger().info(
            f'  [4/6] RRT: home → approach '
            f'[{approach_pos[0]:.3f}, {approach_pos[1]:.3f}, '
            f'{approach_pos[2]:.3f}]')

        q_goal = self._constrained_ik_for_pose(T_approach)
        if q_goal is None:
            self.get_logger().error(
                'PRE_COMPUTE FAILED — no elbow-up IK for approach. '
                'Try widening constraints.')
            return

        raw_rrt = rrt_connect(full_path[-1], q_goal,
                              step_size=0.2, max_iter=10000)
        if raw_rrt is None:
            self.get_logger().error(
                'PRE_COMPUTE FAILED — RRT planning failed, retrying…')
            return

        smoothed = smooth_path(raw_rrt, max_attempts=200)
        rrt_dense = bezier_smooth_path(smoothed, max_step=0.02)
        full_path.extend(rrt_dense[1:])  # skip first (duplicate)

        self.get_logger().info(
            f'        RRT: {len(raw_rrt)} raw → {len(smoothed)} smoothed '
            f'→ {len(rrt_dense)} dense')

        # Verify approach FK
        T_goal_fk = ur5e_fk(full_path[-1])
        goal_pos = T_goal_fk[:3, 3]
        self.get_logger().info(
            f'        Approach FK = [{goal_pos[0]:.4f}, '
            f'{goal_pos[1]:.4f}, {goal_pos[2]:.4f}]')

        # ── SEGMENT 5: DESCENT (approach height → surface) ───────────
        descent_wps = _interpolate_cartesian_smooth(
            [approach_pos, self.draw_positions[0].copy()],
            self.target_quat,
            self.approach_v_max, self.approach_a_max, cart_dt)
        descent_path = self._ik_solve_cartesian_path(
            descent_wps, full_path[-1], 'Descent')
        if descent_path:
            full_path.extend(descent_path[1:])
        else:
            self.get_logger().warn(
                '  Descent IK failed entirely — '
                'drawing will start from approach height')

        # ── SEGMENT 6: DRAWING (dense Cartesian waypoints on surface) ─
        self._draw_start_idx = len(full_path)

        draw_wps = _interpolate_cartesian_smooth(
            self.draw_positions, self.target_quat,
            self.v_max, self.a_max, cart_dt)
        draw_path = self._ik_solve_cartesian_path(
            draw_wps, full_path[-1], 'Drawing')
        if not draw_path or len(draw_path) < 2:
            self.get_logger().error(
                'PRE_COMPUTE FAILED — drawing IK returned too few solutions')
            return
        full_path.extend(draw_path[1:])
        self._draw_end_idx = len(full_path) - 1

        # ── SEGMENT 7: ASCENT (surface → approach height) — skip if looping
        ascent_path = []
        if not self.do_loop:
            last_draw_pos = self.draw_positions[-1].copy()
            ascent_pos = last_draw_pos - self.approach_height * self.plane_n
            ascent_wps = _interpolate_cartesian_smooth(
                [last_draw_pos, ascent_pos], self.target_quat,
                self.approach_v_max, self.approach_a_max, cart_dt)
            ascent_path = self._ik_solve_cartesian_path(
                ascent_wps, full_path[-1], 'Ascent')
            if ascent_path:
                full_path.extend(ascent_path[1:])

        # ── PER-PHASE TRAPEZOIDAL TIMING ───────────────────────────────
        # Each phase gets its own accel→cruise→decel so the robot
        # naturally stops between phases but moves smoothly within each.
        phases = [
            ('retract_up',   retract_path),
            ('retract_home', retract_home_path),
            ('home_hold',    home_hold_path),
            ('rrt',          list(rrt_dense)),
            ('descent',      descent_path if descent_path else []),
            ('drawing',      draw_path),
        ]
        if ascent_path:
            phases.append(('ascent', ascent_path))

        self._master_path, self._master_times = \
            self._concat_phase_times(phases)

        # Recompute draw indices in the new master path
        # (descent end = draw start, draw end = draw start + len(draw_path) - 1)
        pre_draw_count = 0
        for label, plist in phases:
            if label == 'drawing':
                break
            pre_draw_count += max(len(plist) - 1, 0)  # -1 for boundary dedup
        self._draw_start_idx = pre_draw_count
        self._draw_end_idx = pre_draw_count + len(draw_path) - 1
        self._draw_end_idx = min(self._draw_end_idx,
                                 len(self._master_path) - 1)

        total_dur = self._master_times[-1] if self._master_times else 0.0
        draw_dur = (self._master_times[self._draw_end_idx]
                    - self._master_times[self._draw_start_idx]
                    if self._draw_end_idx > self._draw_start_idx else 0.0)

        self.get_logger().info(
            '═══════════════════════════════════════════════\n'
            f'  PRE-COMPUTATION COMPLETE\n'
            f'  Total waypoints : {len(full_path)}\n'
            f'  Total duration  : {total_dur:.1f}s\n'
            f'  Drawing segment : idx {self._draw_start_idx}'
            f'–{self._draw_end_idx} ({draw_dur:.1f}s)\n'
            f'  IK stats: {self._ik_fail_count} fails, '
            f'{self._ik_jump_count} jumps, '
            f'{self._ik_cfg_count} cfg-rejects '
            f'/ {self._ik_total_count} total\n'
            '═══════════════════════════════════════════════')

        self._traj_sent = False
        self._exec_start_time = None
        self.phase = Phase.EXECUTING

    # ------------------------------------------------------------------
    # EXECUTING — lightweight playback of the pre-computed trajectory
    # ------------------------------------------------------------------
    def _execute(self):
        if self._real_robot:
            self._execute_real()
        else:
            self._execute_sim()

    def _execute_real(self):
        """Real robot: send trajectory once, then monitor convergence."""
        if not self._traj_sent:
            if self.do_loop:
                # Send up to end of drawing (no ascent)
                path = self._master_path[:self._draw_end_idx + 1]
                times = self._master_times[:self._draw_end_idx + 1]
            else:
                path = self._master_path
                times = self._master_times
            self._send_full_trajectory(path, times)
            self._traj_sent = True

        self._mirror_real_to_sim()

        # Progress logging
        real_q = self._get_real_ordered_joints()
        if real_q is not None:
            dists = [float(np.max(np.abs(real_q - q)))
                     for q in self._master_path]
            closest = int(np.argmin(dists))
            total = len(self._master_path) - 1
            pct = 100.0 * closest / max(total, 1)
            self.get_logger().info(
                f'EXECUTING  ~step {closest}/{total}  ({pct:.0f}%)',
                throttle_duration_sec=1.0)

        # Convergence check
        if self.do_loop:
            target_q = self._master_path[self._draw_end_idx]
            if self._real_robot_converged(target_q):
                self.get_logger().info('Looping drawing segment…')
                draw_seg = self._master_path[
                    self._draw_start_idx:self._draw_end_idx + 1]
                draw_times = self._compute_phase_timing(draw_seg)
                self._send_full_trajectory(draw_seg, draw_times)
        else:
            target_q = self._master_path[-1]
            if self._real_robot_converged(target_q):
                self._last_q = target_q.copy()
                self.get_logger().info('Trajectory complete → DONE')
                self.phase = Phase.DONE

    def _execute_sim(self):
        """Sim: time-based interpolation along the master timeline."""
        if self._exec_start_time is None:
            self._exec_start_time = self.get_clock().now()

        elapsed = (self.get_clock().now()
                   - self._exec_start_time).nanoseconds * 1e-9

        total_time = self._master_times[-1]
        draw_start_t = self._master_times[self._draw_start_idx]
        draw_end_t = self._master_times[self._draw_end_idx]

        # Handle looping
        if self.do_loop and elapsed > draw_end_t:
            loop_dur = draw_end_t - draw_start_t
            if loop_dur > 1e-6:
                elapsed = draw_start_t + (
                    (elapsed - draw_start_t) % loop_dur)
        elif not self.do_loop and elapsed >= total_time:
            q_final = self._master_path[-1]
            self._pub_joints(q_final)
            self._last_q = q_final.copy()
            self.get_logger().info('Trajectory complete → DONE')
            self.phase = Phase.DONE
            return

        # Binary search for current segment
        times_arr = self._master_times
        idx = int(np.searchsorted(times_arr, elapsed, side='right')) - 1
        idx = max(0, min(idx, len(self._master_path) - 2))

        # Linear interpolation between adjacent waypoints
        t0 = times_arr[idx]
        t1 = times_arr[idx + 1]
        alpha = (elapsed - t0) / max(t1 - t0, 1e-9)
        alpha = max(0.0, min(1.0, alpha))

        q_interp = ((1.0 - alpha) * self._master_path[idx]
                     + alpha * self._master_path[idx + 1])
        self._pub_joints(q_interp)
        self._last_q = q_interp.copy()

        # Progress logging
        pct = 100.0 * elapsed / max(total_time, 0.01)
        # Determine current segment name
        if idx < self._draw_start_idx:
            segment = 'pre-draw'
        elif idx <= self._draw_end_idx:
            segment = 'DRAWING'
        else:
            segment = 'post-draw'

        self.get_logger().info(
            f'EXECUTING  t={elapsed:.1f}/{total_time:.1f}s  ({pct:.0f}%)  '
            f'idx={idx}/{len(self._master_path)}  [{segment}]',
            throttle_duration_sec=1.0)

    # ------------------------------------------------------------------
    # Main control loop
    # ------------------------------------------------------------------
    def _control_loop(self):
        if self.phase == Phase.PRE_COMPUTE:
            self._pre_compute()
        elif self.phase == Phase.EXECUTING:
            self._execute()
        elif self.phase == Phase.DONE:
            if self._real_robot:
                self._mirror_real_to_sim()
            elif self._last_q is not None:
                self._pub_joints(self._last_q)


def main(args=None):
    rclpy.init(args=args)
    node = CartesianDrawController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
