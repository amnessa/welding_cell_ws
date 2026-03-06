#!/usr/bin/env python3
"""
Planar Servo Controller — Pre-compute + velocity-control hybrid architecture.

Pre-surface phases (retract up, homing, RRT approach, descent) are pre-computed
into one dense joint-space trajectory BEFORE the robot moves — identical to
the cartesian controller.  Once the end-effector reaches the drawing surface,
the controller switches to PD velocity control with a Jacobian-based resolver.

Architecture
------------
  PRE_COMPUTE  — Build joint path offline (retract up → home hold → RRT →
                 descent).  All IK solved here.
  EXECUTING    — Lightweight playback of pre-computed trajectory:
                   Sim:  time-based interpolation at execution_hz
                   Real: one JointTrajectory message → convergence monitoring
  SERVO        — PD velocity control on-surface (follows waypoints)
  TELEOP       — PD velocity control on-surface (keyboard input)
  DONE         — Hold (zero velocity)

Publishes
---------
  /end_effector_velocity  (geometry_msgs/Twist)  — SERVO & TELEOP phases
  /isaac_joint_commands   (sensor_msgs/JointState) — pre-surface phases

Subscribes
----------
  /isaac_joint_states     (sensor_msgs/JointState)
"""

import json
import math
import os
import sys
from enum import Enum, auto
from typing import List, Optional, Tuple

_scripts_dir = os.path.dirname(os.path.realpath(__file__))
if _scripts_dir not in sys.path:
    sys.path.insert(0, _scripts_dir)

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf2_ros
from tf2_ros import TransformException


# ---------------------------------------------------------------------------
# Quaternion / rotation helpers
# ---------------------------------------------------------------------------

def quat_to_rotmat(q_xyzw):
    """Quaternion [x, y, z, w] → 3×3 rotation matrix."""
    x, y, z, w = q_xyzw
    return np.array([
        [1 - 2*(y*y + z*z),  2*(x*y - z*w),      2*(x*z + y*w)],
        [2*(x*y + z*w),      1 - 2*(x*x + z*z),   2*(y*z - x*w)],
        [2*(x*z - y*w),      2*(y*z + x*w),        1 - 2*(x*x + y*y)]
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


def orientation_error_aa(q_current_xyzw, q_target_xyzw):
    """Orientation error as angle-axis vector (current → target)."""
    R_c = quat_to_rotmat(q_current_xyzw)
    R_t = quat_to_rotmat(q_target_xyzw)
    R_err = R_t @ R_c.T
    trace = R_err[0, 0] + R_err[1, 1] + R_err[2, 2]
    cos_a = np.clip((trace - 1.0) / 2.0, -1.0, 1.0)
    angle = math.acos(cos_a)
    if abs(angle) < 1e-6:
        return np.zeros(3)
    axis = np.array([
        R_err[2, 1] - R_err[1, 2],
        R_err[0, 2] - R_err[2, 0],
        R_err[1, 0] - R_err[0, 1]])
    n = np.linalg.norm(axis)
    if n < 1e-8:
        return np.zeros(3)
    axis /= n
    return angle * axis


def clamp_vec(v, max_mag):
    """Clamp a numpy vector's magnitude."""
    mag = np.linalg.norm(v)
    if mag > max_mag and mag > 1e-6:
        return v * (max_mag / mag)
    return v


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
                s = (d_accel + d_cruise
                     + actual_v_max * t_dec
                     - 0.5 * a_max * (t_dec ** 2))
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
    SERVO       = auto()
    TELEOP      = auto()
    DONE        = auto()


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class PlanarServoController(Node):
    def __init__(self):
        super().__init__('planar_servo_controller')

        # ---- parameters ----
        self.declare_parameter('plane_json_file', '')
        self.declare_parameter('ee_link', 'tool0')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('approach_height', 0.08)
        self.declare_parameter('waypoint_threshold', 0.03)
        self.declare_parameter('approach_threshold', 0.06)
        self.declare_parameter('orientation_threshold', 0.15)
        self.declare_parameter('kp_linear', 1.5)
        self.declare_parameter('kd_linear', 0.0)
        self.declare_parameter('kp_angular', 1.5)
        self.declare_parameter('kd_angular', 0.0)
        self.declare_parameter('max_linear_vel', 0.25)
        self.declare_parameter('max_angular_vel', 0.60)
        self.declare_parameter('plane_z_correction_gain', 2.0)
        self.declare_parameter('loop_trajectory', False)
        self.declare_parameter('trajectory_key', 'projected_vector_trajectory')
        self.declare_parameter('boundary_margin', 0.01)
        self.declare_parameter('teleop_mode', False)
        self.declare_parameter('teleop_speed', 0.10)
        self.declare_parameter('execution_hz', 100.0)
        self.declare_parameter('descent_step', 0.002)
        self.declare_parameter('shoulder_lift_max', 0.0)
        self.declare_parameter('shoulder_lift_min', -2.5)
        self.declare_parameter('elbow_max', -0.3)
        self.declare_parameter('elbow_min', -3.14)
        self.declare_parameter('ik_num_seeds', 30)
        self.declare_parameter('ik_damping', 0.05)
        self.declare_parameter('max_joint_step', 0.15)
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
        # Cartesian interpolation speed for pre-surface phases
        self.declare_parameter('cart_v_max', 0.05)
        self.declare_parameter('cart_a_max', 0.05)

        self._load_params()
        self._load_plane_json()

        # ---- joint names & home ----
        self._joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
        ]
        self._home_positions = np.array([
            -0.7854, -0.4363, -2.5307, -0.1745, 0.0, 0.0,
        ], dtype=float)

        # ---- TF2 ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- publishers ----
        self.twist_pub = self.create_publisher(
            Twist, '/end_effector_velocity', 10)
        self.joint_cmd_pub = self.create_publisher(
            JointState, '/isaac_joint_commands', 10)

        # ---- subscribers ----
        self._last_joint_state: Optional[JointState] = None
        self._last_sim_joint_state: Optional[JointState] = None
        self._last_real_joint_state: Optional[JointState] = None
        self.joint_state_sub = self.create_subscription(
            JointState, '/isaac_joint_states',
            self._sim_joint_state_cb, 10)

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

        # ---- safe retract / home hold ----
        self._retract_height = 0.15
        self._home_hold_sec = 0.5

        # ---- master trajectory (filled by _pre_compute) ----
        self._master_path: List[np.ndarray] = []
        self._master_times: List[float] = []
        self._traj_sent = False
        self._exec_start_time = None
        self._last_q: Optional[np.ndarray] = None

        # ---- PD derivative state ----
        self._prev_err_px = 0.0
        self._prev_err_py = 0.0
        self._prev_err_pz = 0.0
        self._prev_orient_err = np.zeros(3)
        self._prev_time = None

        # ---- teleop subscription ----
        self._teleop_vel = Twist()
        self._teleop_stamp = self.get_clock().now()
        if self.teleop_mode:
            self.teleop_sub = self.create_subscription(
                Twist, '/teleop_plane_vel',
                self._teleop_callback, 10)

        # ---- servo state ----
        self.waypoint_idx = 0

        # ---- phase ----
        self.phase = Phase.PRE_COMPUTE

        # ---- control timer ----
        self.timer = self.create_timer(self._dt, self._control_loop)

        self.get_logger().info(
            f'Planar servo controller started — '
            f'{len(self.waypoints)} waypoints ({self.traj_key}), '
            f'teleop_mode={self.teleop_mode}, '
            f'execution_hz={self.execution_hz:.0f}')

    # ------------------------------------------------------------------
    # Parameter loading
    # ------------------------------------------------------------------
    def _load_params(self):
        g = self.get_parameter
        self.ee_link          = g('ee_link').value
        self.base_frame       = g('base_frame').value
        self.approach_height  = g('approach_height').value
        self.wp_thresh        = g('waypoint_threshold').value
        self.approach_thresh  = g('approach_threshold').value
        self.orient_thresh    = g('orientation_threshold').value
        self.kp_lin           = g('kp_linear').value
        self.kd_lin           = g('kd_linear').value
        self.kp_ang           = g('kp_angular').value
        self.kd_ang           = g('kd_angular').value
        self.max_lin          = g('max_linear_vel').value
        self.max_ang          = g('max_angular_vel').value
        self.z_corr_gain      = g('plane_z_correction_gain').value
        self.do_loop          = g('loop_trajectory').value
        self.traj_key         = g('trajectory_key').value
        self.boundary_margin  = g('boundary_margin').value
        self.teleop_mode      = g('teleop_mode').value
        self.teleop_speed     = g('teleop_speed').value
        self.execution_hz     = g('execution_hz').value
        self.descent_step     = g('descent_step').value
        self.shoulder_lift_max = g('shoulder_lift_max').value
        self.shoulder_lift_min = g('shoulder_lift_min').value
        self.elbow_max        = g('elbow_max').value
        self.elbow_min        = g('elbow_min').value
        self.ik_num_seeds     = g('ik_num_seeds').value
        self.ik_damping       = g('ik_damping').value
        self.max_joint_step   = g('max_joint_step').value
        self.line_u_start     = g('line_u_start').value
        self.line_v_start     = g('line_v_start').value
        self.line_u_end       = g('line_u_end').value
        self.line_v_end       = g('line_v_end').value
        self.cart_v_max       = g('cart_v_max').value
        self.cart_a_max       = g('cart_a_max').value

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
        self.R_plane2base = np.column_stack(
            [self.plane_x, self.plane_y, self.plane_n])

        # ---- rectangle bounds ----
        corners = [np.array(c, dtype=float) for c in data['rectangle_corners']]
        pxs, pys = [], []
        for c in corners:
            rel = c - self.plane_origin
            pxs.append(float(np.dot(rel, self.plane_x)))
            pys.append(float(np.dot(rel, self.plane_y)))
        self.bounds = dict(
            x_min=min(pxs), x_max=max(pxs),
            y_min=min(pys), y_max=max(pys))

        self.rect_origin = corners[0]
        self.rect_width_vec = corners[1] - corners[0]
        self.rect_height_vec = corners[3] - corners[0]

        # ---- trajectory waypoints ----
        if self.traj_key == 'line':
            p_start = (self.rect_origin
                       + self.line_u_start * self.rect_width_vec
                       + self.line_v_start * self.rect_height_vec)
            p_end = (self.rect_origin
                     + self.line_u_end * self.rect_width_vec
                     + self.line_v_end * self.rect_height_vec)
            sq = data.get('square_trajectory', [])
            if not sq:
                raise RuntimeError('Need square_trajectory for orientation')
            quat = list(sq[0]['orientation_xyzw'])
            line_vec = p_end - p_start
            line_len = float(np.linalg.norm(line_vec))
            n_pts = max(int(line_len / 0.005), 2)
            self.waypoints = []
            for i in range(n_pts + 1):
                t = i / n_pts
                self.waypoints.append((p_start + t * line_vec, quat))
            self.get_logger().info(
                f'Line trajectory: {len(self.waypoints)} waypoints, '
                f'length={line_len:.3f}m')
        else:
            traj = data.get(self.traj_key) or \
                   data.get('square_trajectory', [])
            self.waypoints = []
            for wp in traj:
                pos = np.array(wp['position'], dtype=float)
                quat = list(wp['orientation_xyzw'])
                self.waypoints.append((pos, quat))
        if not self.waypoints:
            raise RuntimeError('No waypoints in plane JSON')

        self.target_quat = self.waypoints[0][1]

        # Plane center (for teleop approach target)
        cx = (self.bounds['x_min'] + self.bounds['x_max']) / 2.0
        cy = (self.bounds['y_min'] + self.bounds['y_max']) / 2.0
        self.plane_center = (self.plane_origin
                             + cx * self.plane_x
                             + cy * self.plane_y)

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
    # Per-phase trapezoidal timing
    # ------------------------------------------------------------------
    def _compute_phase_timing(self, path: List[np.ndarray]) -> List[float]:
        """Trapezoidal velocity profile over one phase (segment group).

        Builds cumulative joint-space arc length, then maps a single
        accel → cruise → decel onto it.  Call once per logical phase
        (retract, homing, RRT, descent) so the robot smoothly ramps
        within each phase and naturally stops between them.
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

    def _send_full_trajectory(self, path, times) -> None:
        if self._real_traj_pub is None:
            return
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = list(self._joint_names)
        for q, t in zip(path, times):
            pt = JointTrajectoryPoint()
            pt.positions = (q if isinstance(q, list) else q.tolist())
            dur_sec = int(t)
            dur_nsec = int((t - dur_sec) * 1e9)
            pt.time_from_start = Duration(sec=dur_sec, nanosec=dur_nsec)
            traj.points.append(pt)
        self._real_traj_pub.publish(traj)
        self.get_logger().info(
            f'Sent full trajectory ({len(path)} pts, '
            f'{times[-1]:.1f}s) to real robot')

    # ------------------------------------------------------------------
    # Batch IK solver
    # ------------------------------------------------------------------
    def _ik_solve_cartesian_path(self, cart_wps, q_seed,
                                 label='path'):
        path = []
        seed = q_seed.copy()
        fails = 0
        for pos, quat in cart_wps:
            T = self._pose44(pos, quat)
            q_sol = self._safe_ik(T, seed)
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
    # Publish joint command (sim: velocity clamped, real: mirror only)
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

        self._mirror_real_to_sim()

    # ------------------------------------------------------------------
    # IK helpers
    # ------------------------------------------------------------------
    def _safe_ik(self, T_target, q_seed):
        """IK with elbow-up + joint-jump rejection."""
        from ur5e_rrt_planner import ik_solve
        q_sol = ik_solve(T_target, q_seed, max_iter=300,
                         pos_tol=5e-4, orient_tol=1e-3,
                         damping=self.ik_damping)
        if q_sol is None:
            return None
        if not self._config_ok(q_sol):
            return None
        if float(np.max(np.abs(q_sol - q_seed))) > self.max_joint_step:
            return None
        return q_sol

    def _config_ok(self, q):
        if q[1] > self.shoulder_lift_max or q[1] < self.shoulder_lift_min:
            return False
        if q[2] > self.elbow_max or q[2] < self.elbow_min:
            return False
        return True

    def _constrained_ik_for_pose(self, T_target):
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
                dist = float(np.linalg.norm(q_sol - home))
                candidates.append((dist, q_sol))

        if not candidates:
            self.get_logger().error(
                f'Constrained IK failed: {len(seeds)} seeds, '
                f'none satisfy elbow-up constraints')
            return None

        candidates.sort(key=lambda x: x[0])
        best_q = candidates[0][1]
        self.get_logger().info(
            f'  Constrained IK: {len(candidates)}/{len(seeds)} valid  '
            f'best shoulder_lift={best_q[1]:.3f} elbow={best_q[2]:.3f}')
        return best_q

    @staticmethod
    def _pose44(pos, quat_xyzw):
        T = np.eye(4)
        T[:3, :3] = quat_to_rotmat(quat_xyzw)
        T[:3, 3] = pos
        return T

    # ------------------------------------------------------------------
    # TF / plane helpers (for SERVO / TELEOP phases)
    # ------------------------------------------------------------------
    def _get_ee_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_link, rclpy.time.Time())
            pos = np.array([
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z])
            quat = [
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w]
            return pos, quat
        except TransformException as ex:
            self.get_logger().warn(f'TF lookup failed: {ex}',
                                   throttle_duration_sec=2.0)
            return None, None

    def _to_plane(self, pos_base):
        rel = pos_base - self.plane_origin
        return (float(np.dot(rel, self.plane_x)),
                float(np.dot(rel, self.plane_y)),
                float(np.dot(rel, self.plane_n)))

    def _clamp_to_bounds(self, vx, vy, ex, ey):
        m = self.boundary_margin
        if ex <= self.bounds['x_min'] + m and vx < 0:
            vx = 0.0
        if ex >= self.bounds['x_max'] - m and vx > 0:
            vx = 0.0
        if ey <= self.bounds['y_min'] + m and vy < 0:
            vy = 0.0
        if ey >= self.bounds['y_max'] - m and vy > 0:
            vy = 0.0
        return vx, vy

    def _teleop_callback(self, msg: Twist):
        self._teleop_vel = msg
        self._teleop_stamp = self.get_clock().now()

    # ------------------------------------------------------------------
    # Twist builder helpers (SERVO / TELEOP)
    # ------------------------------------------------------------------
    def _make_twist(self, lin_base, ang_base):
        tw = Twist()
        tw.linear.x = float(lin_base[0])
        tw.linear.y = float(lin_base[1])
        tw.linear.z = float(lin_base[2])
        tw.angular.x = float(ang_base[0])
        tw.angular.y = float(ang_base[1])
        tw.angular.z = float(ang_base[2])
        return tw

    def _orient_vel(self, quat_now):
        err = orientation_error_aa(quat_now, self.target_quat)
        now = self.get_clock().now()
        if self._prev_time is not None:
            dt = max((now - self._prev_time).nanoseconds * 1e-9, 1e-6)
        else:
            dt = 0.1
        d_err = (err - self._prev_orient_err) / dt
        self._prev_orient_err = err.copy()
        cmd = self.kp_ang * err + self.kd_ang * d_err
        return clamp_vec(cmd, self.max_ang)

    # ------------------------------------------------------------------
    # PRE_COMPUTE — build pre-surface trajectory offline
    # ------------------------------------------------------------------
    def _pre_compute(self):
        """Build joint-space trajectory: retract up → home hold → RRT →
        descent.  After playback, transitions to SERVO or TELEOP."""
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
            '  PRE-COMPUTING PRE-SURFACE TRAJECTORY\n'
            '═══════════════════════════════════════════════')

        full_path: List[np.ndarray] = []
        cart_dt = 1.0 / self.execution_hz

        # ── SEGMENT 1: RETRACT UP ────────────────────────────────────
        T_now = ur5e_fk(q_current)
        current_pos = T_now[:3, 3].copy()
        current_quat = rotmat_to_quat(T_now[:3, :3])

        lift_pos = current_pos.copy()
        lift_pos[2] += self._retract_height

        self.get_logger().info(
            f'  [1/5] Retract up: z={current_pos[2]:.3f} → '
            f'z={lift_pos[2]:.3f}')

        retract_wps = _interpolate_cartesian_smooth(
            [current_pos, lift_pos], current_quat,
            self.cart_v_max, self.cart_a_max, cart_dt)
        retract_path = self._ik_solve_cartesian_path(
            retract_wps, q_current, 'Retract up')
        if not retract_path:
            self.get_logger().error(
                'PRE_COMPUTE FAILED at retract up — retrying…')
            return
        full_path.extend(retract_path)

        # ── SEGMENT 2: RETRACT HOME ──────────────────────────────────
        q_retracted = full_path[-1]
        q_home = self._home_positions.copy()
        n_home = max(int(1.0 * self.execution_hz), 20)

        retract_home_path = [q_retracted.copy()]
        for i in range(1, n_home + 1):
            alpha = 0.5 * (1.0 - math.cos(math.pi * i / n_home))
            retract_home_path.append(
                (1.0 - alpha) * q_retracted + alpha * q_home)
        full_path.extend(retract_home_path[1:])

        self.get_logger().info(
            f'  [2/5] Retract home: {n_home} joint-interp steps')

        # ── SEGMENT 3: HOME HOLD ─────────────────────────────────────
        n_hold = max(int(self._home_hold_sec * self.execution_hz), 1)
        home_hold_path = [q_home.copy() for _ in range(n_hold + 1)]
        full_path.extend(home_hold_path[1:])

        self.get_logger().info(
            f'  [3/5] Home hold: {self._home_hold_sec:.1f}s')

        # ── SEGMENT 4: RRT ───────────────────────────────────────────
        if self.teleop_mode:
            on_plane_pos = self.plane_center.copy()
        else:
            on_plane_pos = self.waypoints[0][0].copy()

        approach_pos = on_plane_pos - self.approach_height * self.plane_n
        T_approach = self._pose44(approach_pos, self.target_quat)

        self.get_logger().info(
            f'  [4/5] RRT: home → approach '
            f'[{approach_pos[0]:.3f}, {approach_pos[1]:.3f}, '
            f'{approach_pos[2]:.3f}]')

        q_goal = self._constrained_ik_for_pose(T_approach)
        if q_goal is None:
            self.get_logger().error(
                'PRE_COMPUTE FAILED — no elbow-up IK for approach')
            return

        raw_rrt = rrt_connect(full_path[-1], q_goal,
                              step_size=0.2, max_iter=10000)
        if raw_rrt is None:
            self.get_logger().error(
                'PRE_COMPUTE FAILED — RRT planning failed, retrying…')
            return

        smoothed = smooth_path(raw_rrt, max_attempts=200)
        rrt_dense = bezier_smooth_path(smoothed, max_step=0.02)
        full_path.extend(rrt_dense[1:])

        self.get_logger().info(
            f'        RRT: {len(raw_rrt)} raw → {len(smoothed)} '
            f'smoothed → {len(rrt_dense)} dense')

        # ── SEGMENT 5: DESCENT ────────────────────────────────────────
        descent_wps = _interpolate_cartesian_smooth(
            [approach_pos, on_plane_pos], self.target_quat,
            self.cart_v_max, self.cart_a_max, cart_dt)
        descent_path = self._ik_solve_cartesian_path(
            descent_wps, full_path[-1], 'Descent')
        if descent_path:
            full_path.extend(descent_path[1:])
        else:
            self.get_logger().warn(
                '  Descent IK failed — SERVO will start from approach')
            descent_path = []

        # ── PER-PHASE TRAPEZOIDAL TIMING ──────────────────────────────
        phases = [
            ('retract_up',   retract_path),
            ('retract_home', retract_home_path),
            ('home_hold',    home_hold_path),
            ('rrt',          list(rrt_dense)),
            ('descent',      descent_path if descent_path else []),
        ]

        self._master_path, self._master_times = \
            self._concat_phase_times(phases)

        total_dur = self._master_times[-1] if self._master_times else 0.0
        self.get_logger().info(
            '═══════════════════════════════════════════════\n'
            f'  PRE-COMPUTATION COMPLETE\n'
            f'  Total waypoints : {len(self._master_path)}\n'
            f'  Total duration  : {total_dur:.1f}s\n'
            '═══════════════════════════════════════════════')

        self._traj_sent = False
        self._exec_start_time = None
        self.phase = Phase.EXECUTING

    # ------------------------------------------------------------------
    # EXECUTING — playback pre-computed trajectory, then → SERVO/TELEOP
    # ------------------------------------------------------------------
    def _execute(self):
        if self._real_robot:
            self._execute_real()
        else:
            self._execute_sim()

    def _execute_real(self):
        if not self._traj_sent:
            self._send_full_trajectory(
                self._master_path, self._master_times)
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

        final_q = self._master_path[-1]
        if self._real_robot_converged(final_q):
            self._last_q = final_q.copy()
            self._transition_to_surface()

    def _execute_sim(self):
        if self._exec_start_time is None:
            self._exec_start_time = self.get_clock().now()

        elapsed = (self.get_clock().now()
                   - self._exec_start_time).nanoseconds * 1e-9
        total_time = self._master_times[-1]

        if elapsed >= total_time:
            q_final = self._master_path[-1]
            self._pub_joints(q_final)
            self._last_q = q_final.copy()
            self._transition_to_surface()
            return

        idx = int(np.searchsorted(
            self._master_times, elapsed, side='right')) - 1
        idx = max(0, min(idx, len(self._master_path) - 2))

        t0 = self._master_times[idx]
        t1 = self._master_times[idx + 1]
        alpha = (elapsed - t0) / max(t1 - t0, 1e-9)
        alpha = max(0.0, min(1.0, alpha))

        q_interp = ((1.0 - alpha) * self._master_path[idx]
                     + alpha * self._master_path[idx + 1])
        self._pub_joints(q_interp)
        self._last_q = q_interp.copy()

        pct = 100.0 * elapsed / max(total_time, 0.01)
        self.get_logger().info(
            f'EXECUTING  t={elapsed:.1f}/{total_time:.1f}s  ({pct:.0f}%)  '
            f'idx={idx}/{len(self._master_path)}',
            throttle_duration_sec=1.0)

    def _transition_to_surface(self):
        """Transition from EXECUTING → SERVO or TELEOP."""
        if self.teleop_mode:
            self.get_logger().info(
                'Pre-surface trajectory complete → TELEOP')
            self.phase = Phase.TELEOP
        else:
            self.get_logger().info(
                'Pre-surface trajectory complete → SERVO')
            self.waypoint_idx = 0
            self.phase = Phase.SERVO

    # ------------------------------------------------------------------
    # SERVO — PD velocity control on the drawing surface
    # ------------------------------------------------------------------
    def _servo(self, pos, quat):
        if self.waypoint_idx >= len(self.waypoints):
            if self.do_loop:
                self.waypoint_idx = 0
                self.get_logger().info('Looping trajectory…')
            else:
                self.get_logger().info('Trajectory complete → DONE')
                self.phase = Phase.DONE
                return self._make_twist(np.zeros(3), np.zeros(3))

        target = self.waypoints[self.waypoint_idx][0]
        err_base = target - pos
        err_px = float(np.dot(err_base, self.plane_x))
        err_py = float(np.dot(err_base, self.plane_y))
        err_pz = float(np.dot(err_base, self.plane_n))

        now = self.get_clock().now()
        if self._prev_time is not None:
            dt = (now - self._prev_time).nanoseconds * 1e-9
        else:
            dt = 1.0 / self.execution_hz
        dt = max(dt, 1e-6)

        d_err_px = (err_px - self._prev_err_px) / dt
        d_err_py = (err_py - self._prev_err_py) / dt

        self._prev_err_px = err_px
        self._prev_err_py = err_py
        self._prev_err_pz = err_pz
        self._prev_time = now

        vx = self.kp_lin * err_px + self.kd_lin * d_err_px
        vy = self.kp_lin * err_py + self.kd_lin * d_err_py
        vz = self.z_corr_gain * err_pz

        ex, ey, _ = self._to_plane(pos)
        vx, vy = self._clamp_to_bounds(vx, vy, ex, ey)

        vel_base = (vx * self.plane_x
                    + vy * self.plane_y
                    + vz * self.plane_n)
        vel_base = clamp_vec(vel_base, self.max_lin)
        ang = self._orient_vel(quat)

        dist = np.linalg.norm(err_base)
        self.get_logger().info(
            f'SERVO  wp={self.waypoint_idx}/{len(self.waypoints)}  '
            f'dist={dist:.4f}m  z_off={err_pz:.4f}m',
            throttle_duration_sec=1.0)

        if dist < self.wp_thresh:
            self.get_logger().info(
                f'Reached waypoint {self.waypoint_idx}')
            self.waypoint_idx += 1

        return self._make_twist(vel_base, ang)

    # ------------------------------------------------------------------
    # TELEOP — keyboard velocity control on the surface
    # ------------------------------------------------------------------
    def _teleop(self, pos, quat):
        dt = (self.get_clock().now() - self._teleop_stamp).nanoseconds * 1e-9
        if dt > 0.5:
            vx_cmd, vy_cmd = 0.0, 0.0
        else:
            vx_cmd = self._teleop_vel.linear.x
            vy_cmd = self._teleop_vel.linear.y

        _, _, ez = self._to_plane(pos)
        vz = -self.z_corr_gain * ez

        ex, ey, _ = self._to_plane(pos)
        vx_cmd, vy_cmd = self._clamp_to_bounds(vx_cmd, vy_cmd, ex, ey)

        vel_base = (vx_cmd * self.plane_x
                    + vy_cmd * self.plane_y
                    + vz * self.plane_n)
        vel_base = clamp_vec(vel_base, self.max_lin)
        ang = self._orient_vel(quat)

        self.get_logger().info(
            f'TELEOP  plane_xy=({ex:.3f},{ey:.3f})  '
            f'z_off={ez:.4f}m  cmd=({vx_cmd:.3f},{vy_cmd:.3f})',
            throttle_duration_sec=2.0)

        return self._make_twist(vel_base, ang)

    # ------------------------------------------------------------------
    # Main control loop
    # ------------------------------------------------------------------
    def _control_loop(self):
        if self.phase == Phase.PRE_COMPUTE:
            self._pre_compute()
            return
        if self.phase == Phase.EXECUTING:
            self._execute()
            return

        # SERVO / TELEOP / DONE — velocity commands via Jacobian node
        pos, quat = self._get_ee_pose()
        if pos is None:
            self.twist_pub.publish(Twist())
            return

        if self.phase == Phase.SERVO:
            tw = self._servo(pos, quat)
        elif self.phase == Phase.TELEOP:
            tw = self._teleop(pos, quat)
        else:  # DONE
            tw = Twist()

        self.twist_pub.publish(tw)


def main(args=None):
    rclpy.init(args=args)
    node = PlanarServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
