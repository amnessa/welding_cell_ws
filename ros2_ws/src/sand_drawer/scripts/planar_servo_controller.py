#!/usr/bin/env python3
"""
Planar Servo Controller — Constrained trajectory following on the drawing plane.

Reads the plane definition (orientation, bounds, waypoints) from the JSON file
produced by plane_solver_node.  Uses TF2 to track the current end-effector
pose and generates velocity commands constrained to the plane surface.

Publishes geometry_msgs/Twist on /end_effector_velocity which the existing
jacobian_calculator_node converts to joint commands via damped-pseudoinverse IK.

Phases
------
1. RETRACT_UP   – Lift EE straight up to safe clearance (Cartesian IK)
2. RETRACT_HOME – Interpolate in joint-space from retracted pose to home
3. HOME_HOLD    – Hold at home for 2 s (verify stability)
4. PLANNING     – Compute RRT-Connect path in C-space to plane target
5. EXECUTING    – Follow the planned joint-space path
6. SERVO        – Follow the waypoint trajectory on the plane
7. TELEOP       – Accept manual velocity commands from /teleop_plane_vel
8. DONE         – Hold (zero velocity)
"""


import json
import math
import os
import sys
from enum import Enum, auto

# Ensure the scripts directory is on sys.path so ur5e_rrt_planner can be imported
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
# Helpers
# ---------------------------------------------------------------------------

def quat_to_rotmat(q_xyzw):
    """Quaternion [x, y, z, w] → 3×3 rotation matrix."""
    x, y, z, w = q_xyzw
    return np.array([
        [1 - 2*(y*y + z*z),  2*(x*y - z*w),      2*(x*z + y*w)],
        [2*(x*y + z*w),      1 - 2*(x*x + z*z),   2*(y*z - x*w)],
        [2*(x*z - y*w),      2*(y*z + x*w),        1 - 2*(x*x + y*y)]
    ])


def orientation_error_aa(q_current_xyzw, q_target_xyzw):
    """
    Orientation error as an angle-axis vector in the base frame.

    Returns a 3-vector whose direction is the rotation axis and magnitude
    is the rotation angle (radians) needed to go from current → target.
    """
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
        R_err[1, 0] - R_err[0, 1]
    ])
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
# Node
# ---------------------------------------------------------------------------

class Phase(Enum):
    RETRACT_UP   = auto()   # Cartesian lift to safe height
    RETRACT_HOME = auto()   # Joint-space interpolation to home
    HOME_HOLD    = auto()   # Hold at home position
    PLANNING     = auto()
    EXECUTING    = auto()
    DESCENDING   = auto()   # velocity-controlled lowering onto the surface
    SERVO        = auto()
    TELEOP       = auto()
    DONE         = auto()


class PlanarServoController(Node):
    def __init__(self):
        super().__init__('planar_servo_controller')

        # ---- parameters ----
        self.declare_parameter('plane_json_file', '')
        self.declare_parameter('ee_link', 'tool0')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('approach_height', 0.08)        # m above plane
        self.declare_parameter('waypoint_threshold', 0.03)     # m — "close enough"
        self.declare_parameter('approach_threshold', 0.06)     # m — generous for damped IK
        self.declare_parameter('orientation_threshold', 0.15)  # rad
        self.declare_parameter('kp_linear', 1.5)
        self.declare_parameter('kd_linear', 0.0)               # D-gain for linear vel
        self.declare_parameter('kp_angular', 1.5)
        self.declare_parameter('kd_angular', 0.0)              # D-gain for angular vel
        self.declare_parameter('max_linear_vel', 0.25)         # m/s
        self.declare_parameter('max_angular_vel', 0.60)        # rad/s
        self.declare_parameter('plane_z_correction_gain', 2.0)
        self.declare_parameter('loop_trajectory', False)
        self.declare_parameter('trajectory_key', 'projected_vector_trajectory')
        self.declare_parameter('boundary_margin', 0.01)        # m
        self.declare_parameter('teleop_mode', False)
        self.declare_parameter('teleop_speed', 0.10)            # m/s default
        self.declare_parameter('execution_hz', 60.0)            # control loop rate
        self.declare_parameter('descent_step', 0.002)           # m per tick during descent
        # Elbow-up configuration constraints (same as cartesian controller)
        self.declare_parameter('shoulder_lift_max', 0.0)
        self.declare_parameter('shoulder_lift_min', -2.5)
        self.declare_parameter('elbow_max', -0.3)
        self.declare_parameter('elbow_min', -3.14)
        self.declare_parameter('ik_num_seeds', 30)
        self.declare_parameter('ik_damping', 0.05)
        self.declare_parameter('max_joint_step', 0.15)
        # Line trajectory UV parameters (used when trajectory_key='line')
        self.declare_parameter('line_u_start', 0.5)
        self.declare_parameter('line_v_start', 0.3)
        self.declare_parameter('line_u_end', 0.5)
        self.declare_parameter('line_v_end', 0.7)
        # Real-robot bridging
        self.declare_parameter('real_robot', False)
        self.declare_parameter('real_robot_joint_state_topic', '/joint_states')
        self.declare_parameter('real_robot_trajectory_topic',
                               '/scaled_joint_trajectory_controller/joint_trajectory')
        self.declare_parameter('real_robot_trajectory_duration', 0.2)
        self.declare_parameter('max_joint_speed_deg', 45.0)
        self.declare_parameter('max_joint_accel_deg', 40.0)

        self._load_params()
        self._load_plane_json()

        # ---- TF2 ----
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- publisher ----
        self.twist_pub = self.create_publisher(Twist, '/end_effector_velocity', 10)
        self.joint_cmd_pub = self.create_publisher(JointState, '/isaac_joint_commands', 10)

        # ---- real-robot bridging ----
        self._real_robot = self.get_parameter('real_robot').value
        self._real_traj_pub = None
        self._real_converge_tol = math.radians(2.0)  # 2° tolerance
        self._last_sim_joint_state = None
        self._last_real_joint_state = None
        if self._real_robot:
            real_js_topic = self.get_parameter(
                'real_robot_joint_state_topic').value
            real_traj_topic = self.get_parameter(
                'real_robot_trajectory_topic').value
            self._real_traj_duration = float(self.get_parameter(
                'real_robot_trajectory_duration').value)
            self._real_traj_pub = self.create_publisher(
                JointTrajectory, real_traj_topic, 10)
            self._real_joint_sub = self.create_subscription(
                JointState, real_js_topic,
                self._real_joint_state_cb, 10)
            self.get_logger().info(
                f'REAL ROBOT mode: subscribing {real_js_topic}, '
                f'publishing {real_traj_topic} '
                f'(sim syncs to real robot)')

        # ---- joint speed / acceleration limits ----
        self._max_joint_speed_deg = float(self.get_parameter('max_joint_speed_deg').value)
        self._max_joint_speed_rad = math.radians(self._max_joint_speed_deg)
        self._max_joint_accel_deg = float(self.get_parameter('max_joint_accel_deg').value)
        self._max_joint_accel_rad = math.radians(self._max_joint_accel_deg)
        self._dt = 1.0 / self.execution_hz
        self._prev_cmd_q: Optional[np.ndarray] = None
        self.get_logger().info(
            f'Max joint speed: {self._max_joint_speed_deg:.1f} deg/s '
            f'({self._max_joint_speed_rad:.4f} rad/s), '
            f'accel: {self._max_joint_accel_deg:.1f} deg/s\u00b2 '
            f'({self._max_joint_accel_rad:.4f} rad/s\u00b2)')

        # ---- joint state subscriber ----
        self._last_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState, '/isaac_joint_states',
            self._sim_joint_state_cb, 10)

        # ---- home position ----
        self._home_joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        self._home_joint_positions = [
            -0.7854, -0.4363, -2.5307, -0.1745, 0.0, 0.0
        ]
        self._home_reached_time = None  # timestamp when home was reached
        self._home_hold_duration = 2.0  # seconds to hold at home before transitioning

        # ---- safe retract state ----
        self._retract_height = 0.15   # metres above current EE to lift before homing
        self._retract_waypoints = []  # list of (pos, quat) for Cartesian lift
        self._retract_idx = 0
        self._retract_home_path = []  # joint-space interpolation to home
        self._retract_home_idx = 0
        self._retract_initialized = False

        # ---- RRT path execution state ----
        self._rrt_path = []       # dense joint-space path
        self._rrt_path_idx = 0    # current index in path
        self._rrt_exec_rate = int(self.execution_hz)  # waypoints per second (matches timer)
        self._rrt_traj_sent = False  # real-robot: full trajectory already published?

        # ---- RETRACT_HOME full-trajectory flag ----
        self._retract_home_traj_sent = False

        # ---- descent state (position-controlled, like cartesian controller) ----
        self._descent_waypoints = []   # list of (pos, quat) tuples
        self._descent_idx = 0
        self._last_q = None            # IK seed for descent

        # ---- teleop subscription ----
        self._teleop_vel = Twist()  # latest teleop command (in plane frame)
        self._teleop_stamp = self.get_clock().now()
        if self.teleop_mode:
            self.teleop_sub = self.create_subscription(
                Twist, '/teleop_plane_vel',
                self._teleop_callback, 10)

        # ---- state ----
        self.phase        = Phase.RETRACT_UP
        self.waypoint_idx = 0

        # ---- control timer ----
        self.timer = self.create_timer(1.0 / self.execution_hz, self._control_loop)

        self.get_logger().info(
            f'Planar servo controller started — {len(self.waypoints)} waypoints, '
            f'trajectory_key={self.traj_key}, teleop_mode={self.teleop_mode}')

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
    # Joint helpers (sim + optional real robot)
    # ------------------------------------------------------------------
    def _get_ordered_joints(self) -> Optional[np.ndarray]:
        """Return ordered joint positions from the primary feedback source.
        When real_robot=true, this returns the REAL robot's joint state."""
        if self._last_joint_state is None:
            return None
        name_map = {n: p for n, p in
                    zip(self._last_joint_state.name,
                        self._last_joint_state.position)}
        try:
            return np.array([name_map[n] for n in self._home_joint_names])
        except KeyError:
            return None

    def _get_real_ordered_joints(self) -> Optional[np.ndarray]:
        """Return ordered joint positions from the REAL robot only."""
        if self._last_real_joint_state is None:
            return None
        name_map = {n: p for n, p in
                    zip(self._last_real_joint_state.name,
                        self._last_real_joint_state.position)}
        try:
            return np.array([name_map[n] for n in self._home_joint_names])
        except KeyError:
            return None

    def _real_robot_converged(self, target_q: np.ndarray) -> bool:
        """Check if real robot has reached target_q within tolerance.
        Always returns True when real_robot is disabled."""
        if not self._real_robot:
            return True
        real_q = self._get_real_ordered_joints()
        if real_q is None:
            return False
        return float(np.max(np.abs(target_q - real_q))) < self._real_converge_tol

    def _mirror_real_to_sim(self):
        """Publish the real robot's actual joint state to sim."""
        if not self._real_robot:
            return
        real_q = self._get_real_ordered_joints()
        if real_q is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._home_joint_names)
        msg.position = real_q.tolist()
        self.joint_cmd_pub.publish(msg)

    # ------------------------------------------------------------------
    # Trapezoidal timing for joint-space paths
    # ------------------------------------------------------------------
    def _compute_joint_path_timing(self, path) -> list:
        """Compute timestamps using trapezoidal velocity profile."""
        if len(path) <= 1:
            return [0.0] * len(path)
        v_max = self._max_joint_speed_rad
        a_max = self._max_joint_accel_rad
        times = [0.0]
        for i in range(1, len(path)):
            delta = float(np.max(np.abs(path[i] - path[i - 1])))
            if delta < 1e-9:
                times.append(times[-1] + 0.01)
                continue
            if delta <= v_max * v_max / a_max:
                dt_seg = 2.0 * math.sqrt(delta / a_max)
            else:
                dt_seg = delta / v_max + v_max / a_max
            times.append(times[-1] + max(dt_seg, 0.01))
        return times

    def _send_full_trajectory(self, path, times) -> None:
        """Send an entire joint-space path as one multi-point JointTrajectory
        to the real robot so the UR controller can interpolate smoothly."""
        if self._real_traj_pub is None:
            return
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = list(self._home_joint_names)
        for q, t in zip(path, times):
            pt = JointTrajectoryPoint()
            pt.positions = (q if isinstance(q, list) else q.tolist())
            dur_sec = int(t)
            dur_nsec = int((t - dur_sec) * 1e9)
            pt.time_from_start = Duration(sec=dur_sec, nanosec=dur_nsec)
            traj.points.append(pt)
        self._real_traj_pub.publish(traj)
        self.get_logger().info(
            f'Sent full trajectory ({len(path)} points, '
            f'{times[-1]:.1f} s) to real robot')

    def _accel_aware_duration(self, delta_rad: float) -> float:
        """Min duration for a joint move respecting speed + accel limits."""
        if delta_rad < 1e-9:
            return self._dt
        v = self._max_joint_speed_rad
        a = self._max_joint_accel_rad
        if delta_rad <= v * v / a:
            return max(2.0 * math.sqrt(delta_rad / a), self._dt)
        return max(delta_rad / v + v / a, self._dt)

    def _pub_joint_cmd(self, cmd: JointState):
        """Publish joint command.  In real-robot mode, mirrors real→sim
        and sends JointTrajectory to the real arm.  In sim-only mode,
        applies velocity clamping and publishes to sim."""
        q = np.array(cmd.position, dtype=float)

        if not self._real_robot:
            # ---- sim-only: velocity clamping ----
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
                    cmd.position = q.tolist()
            self._prev_cmd_q = q.copy()
            self.joint_cmd_pub.publish(cmd)
            return

        # ---- real-robot mode: mirror real→sim, send trajectory ----
        self._mirror_real_to_sim()

        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = list(self._home_joint_names)
        pt = JointTrajectoryPoint()
        pt.positions = q.tolist()
        dur = self._real_traj_duration  # fallback
        real_q = self._get_real_ordered_joints()
        if real_q is not None:
            real_delta = float(np.max(np.abs(q - real_q)))
            dur = self._accel_aware_duration(real_delta)
        dur_sec = int(dur)
        dur_nsec = int((dur - dur_sec) * 1e9)
        pt.time_from_start = Duration(sec=dur_sec, nanosec=dur_nsec)
        traj.points = [pt]
        self._real_traj_pub.publish(traj)

    # ------------------------------------------------------------------
    # Parameter helpers
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

        # ---- Derivative state (previous errors for D-term) ----
        self._prev_err_px = 0.0
        self._prev_err_py = 0.0
        self._prev_err_pz = 0.0
        self._prev_orient_err = np.zeros(3)
        self._prev_time = None

    def _load_plane_json(self):
        json_path = self.get_parameter('plane_json_file').value
        if not json_path or not os.path.exists(json_path):
            self.get_logger().fatal(f'Plane JSON not found: {json_path}')
            raise RuntimeError(f'Plane JSON not found: {json_path}')

        with open(json_path, 'r') as f:
            data = json.load(f)

        # ---- plane geometry ----
        plane = data['plane']
        self.plane_origin = np.array(plane['origin'], dtype=float)
        self.plane_x      = np.array(plane['x_axis'], dtype=float)
        self.plane_y      = np.array(plane['y_axis'], dtype=float)
        self.plane_n      = np.array(plane['normal'], dtype=float)

        # rotation: drawing_plane → base_link
        self.R_plane2base = np.column_stack([self.plane_x, self.plane_y, self.plane_n])

        # ---- rectangle bounds in plane-frame coords ----
        corners = [np.array(c, dtype=float) for c in data['rectangle_corners']]
        pxs, pys = [], []
        for c in corners:
            rel = c - self.plane_origin
            pxs.append(float(np.dot(rel, self.plane_x)))
            pys.append(float(np.dot(rel, self.plane_y)))
        self.bounds = dict(
            x_min=min(pxs), x_max=max(pxs),
            y_min=min(pys), y_max=max(pys))
        self.get_logger().info(
            f'Plane bounds (in plane frame): '
            f'X=[{self.bounds["x_min"]:.3f}, {self.bounds["x_max"]:.3f}] '
            f'Y=[{self.bounds["y_min"]:.3f}, {self.bounds["y_max"]:.3f}]')

        # ---- plane rectangle bounds (for UV → world conversion) ----
        corners = [np.array(c, dtype=float) for c in data['rectangle_corners']]
        self.rect_origin = corners[0]
        self.rect_width_vec = corners[1] - corners[0]
        self.rect_height_vec = corners[3] - corners[0]

        # ---- trajectory waypoints ----
        if self.traj_key == 'line':
            # Generate line from UV parameters on the rectangle
            p_start = (self.rect_origin
                       + self.line_u_start * self.rect_width_vec
                       + self.line_v_start * self.rect_height_vec)
            p_end = (self.rect_origin
                     + self.line_u_end * self.rect_width_vec
                     + self.line_v_end * self.rect_height_vec)
            # Grab orientation from square_trajectory
            sq = data.get('square_trajectory', [])
            if not sq:
                raise RuntimeError('Need square_trajectory for orientation')
            quat = list(sq[0]['orientation_xyzw'])
            # Discretize the line at the same resolution as cartesian controller
            line_vec = p_end - p_start
            line_len = float(np.linalg.norm(line_vec))
            n_pts = max(int(line_len / 0.005), 2)  # ~5 mm spacing
            self.waypoints = []
            for i in range(n_pts + 1):
                t = i / n_pts
                self.waypoints.append((p_start + t * line_vec, quat))
            self.get_logger().info(
                f'Line trajectory: UV ({self.line_u_start},{self.line_v_start})→'
                f'({self.line_u_end},{self.line_v_end})  '
                f'length={line_len:.3f}m  {len(self.waypoints)} waypoints')
        else:
            traj = data.get(self.traj_key) or data.get('square_trajectory', [])
            self.waypoints = []
            for wp in traj:
                pos  = np.array(wp['position'], dtype=float)
                quat = list(wp['orientation_xyzw'])          # [x, y, z, w]
                self.waypoints.append((pos, quat))
        if not self.waypoints:
            raise RuntimeError('No waypoints in plane JSON')

        # Use the first waypoint's orientation as the fixed target orientation
        self.target_quat = self.waypoints[0][1]

        # Compute rectangle center (in base frame) for teleop approach target
        cx = (self.bounds['x_min'] + self.bounds['x_max']) / 2.0
        cy = (self.bounds['y_min'] + self.bounds['y_max']) / 2.0
        self.plane_center = (self.plane_origin
                             + cx * self.plane_x
                             + cy * self.plane_y)

    # ------------------------------------------------------------------
    # TF helper
    # ------------------------------------------------------------------
    def _get_ee_pose(self):
        """Return (position_3, quat_xyzw) of the EE in base_link, or (None, None)."""
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

    # ------------------------------------------------------------------
    # Plane-frame helpers
    # ------------------------------------------------------------------
    def _to_plane(self, pos_base):
        """Project a base-frame position onto the plane frame → (px, py, pz)."""
        rel = pos_base - self.plane_origin
        return (float(np.dot(rel, self.plane_x)),
                float(np.dot(rel, self.plane_y)),
                float(np.dot(rel, self.plane_n)))

    def _clamp_to_bounds(self, vx, vy, ex, ey):
        """Zero velocity components that would push the EE outside the rectangle."""
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

    # ------------------------------------------------------------------
    # Teleop callback
    # ------------------------------------------------------------------
    def _teleop_callback(self, msg: Twist):
        self._teleop_vel = msg
        self._teleop_stamp = self.get_clock().now()

    # ------------------------------------------------------------------
    # RETRACT_UP — lift EE straight up before any homing
    # ------------------------------------------------------------------
    def _retract_up(self):
        """Lift the end-effector vertically (world Z+) to a safe clearance
        height using Cartesian IK interpolation."""
        if not self._retract_initialized:
            q_current = self._get_ordered_joints()
            if q_current is None:
                self.get_logger().warn('RETRACT_UP — waiting for joint states…',
                                       throttle_duration_sec=2.0)
                return

            from ur5e_rrt_planner import ur5e_fk
            T_now = ur5e_fk(q_current)
            current_pos = T_now[:3, 3]
            # Extract current orientation as quaternion
            R = T_now[:3, :3]
            trace = R[0,0] + R[1,1] + R[2,2]
            if trace > 0:
                s = 0.5 / math.sqrt(trace + 1.0)
                w = 0.25 / s; x = (R[2,1]-R[1,2])*s; y = (R[0,2]-R[2,0])*s; z = (R[1,0]-R[0,1])*s
            elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
                s = 2.0*math.sqrt(1.0+R[0,0]-R[1,1]-R[2,2])
                w=(R[2,1]-R[1,2])/s; x=0.25*s; y=(R[0,1]+R[1,0])/s; z=(R[0,2]+R[2,0])/s
            elif R[1,1] > R[2,2]:
                s = 2.0*math.sqrt(1.0+R[1,1]-R[0,0]-R[2,2])
                w=(R[0,2]-R[2,0])/s; x=(R[0,1]+R[1,0])/s; y=0.25*s; z=(R[1,2]+R[2,1])/s
            else:
                s = 2.0*math.sqrt(1.0+R[2,2]-R[0,0]-R[1,1])
                w=(R[1,0]-R[0,1])/s; x=(R[0,2]+R[2,0])/s; y=(R[1,2]+R[2,1])/s; z=0.25*s
            current_quat = [x, y, z, w]
            nrm = math.sqrt(sum(v*v for v in current_quat))
            current_quat = [v/nrm for v in current_quat]

            # Lift target: straight up in world Z
            lift_pos = current_pos.copy()
            lift_pos[2] += self._retract_height

            self.get_logger().info(
                f'RETRACT_UP — current EE z={current_pos[2]:.3f}, '
                f'lifting to z={lift_pos[2]:.3f} '
                f'(+{self._retract_height:.3f} m)')

            # Build dense waypoints for the lift
            dist = self._retract_height
            n_steps = max(int(2.0 * self.execution_hz), 20)  # ~2 sec
            self._retract_waypoints = []
            for i in range(n_steps + 1):
                alpha = i / n_steps
                alpha_smooth = 0.5 * (1.0 - math.cos(math.pi * alpha))
                p = current_pos + alpha_smooth * (lift_pos - current_pos)
                self._retract_waypoints.append((p.copy(), current_quat))

            self._retract_idx = 0
            self._last_q = q_current.copy()
            self._retract_initialized = True

        if self._retract_idx >= len(self._retract_waypoints):
            self.get_logger().info('RETRACT_UP complete → RETRACT_HOME')
            self.phase = Phase.RETRACT_HOME
            return

        pos, quat = self._retract_waypoints[self._retract_idx]
        T = self._pose44(pos, quat)
        q_sol = self._safe_ik(T, self._last_q)

        if q_sol is None:
            self.get_logger().warn(
                f'RETRACT_UP IK failed at step {self._retract_idx}'
                f'/{len(self._retract_waypoints)} — skipping')
            self._retract_idx += 1
            return

        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = self._home_joint_names
        cmd.position = q_sol.tolist()
        self._pub_joint_cmd(cmd)
        self._last_q = q_sol.copy()
        if self._real_robot_converged(q_sol):
            self._retract_idx += 1

        total = len(self._retract_waypoints)
        if self._retract_idx % 10 == 0 or self._retract_idx >= total:
            from ur5e_rrt_planner import ur5e_fk
            fk_pos = ur5e_fk(q_sol)[:3, 3]
            self.get_logger().info(
                f'RETRACT_UP  step {self._retract_idx}/{total}  '
                f'z={fk_pos[2]:.3f}',
                throttle_duration_sec=0.5)

    # ------------------------------------------------------------------
    # RETRACT_HOME — joint-space interpolation to home
    # ------------------------------------------------------------------
    def _retract_home(self):
        """Smoothly interpolate in joint-space from the current (retracted)
        position to the home configuration.  In real-robot mode the entire
        path is sent as one multi-point trajectory."""
        if not self._retract_home_path:
            q_current = self._last_q if self._last_q is not None else self._get_ordered_joints()
            if q_current is None:
                self.get_logger().warn('RETRACT_HOME — waiting for joint states…',
                                       throttle_duration_sec=2.0)
                return

            q_home = np.array(self._home_joint_positions, dtype=float)
            n_steps = max(int(2.0 * self.execution_hz), 20)
            path = []
            for i in range(n_steps + 1):
                alpha = i / n_steps
                alpha_smooth = 0.5 * (1.0 - math.cos(math.pi * alpha))
                q_interp = (1.0 - alpha_smooth) * q_current + alpha_smooth * q_home
                path.append(q_interp)

            self._retract_home_path = path
            self._retract_home_idx = 0
            self._retract_home_traj_sent = False
            self._retract_home_times = self._compute_joint_path_timing(path)
            self.get_logger().info(
                f'RETRACT_HOME — interpolating to home in {n_steps} steps '
                f'(~{self._retract_home_times[-1]:.1f} s)')

        # -- Real-robot mode: send full trajectory once, monitor --
        if self._real_robot:
            if not self._retract_home_traj_sent:
                self._send_full_trajectory(
                    self._retract_home_path, self._retract_home_times)
                self._retract_home_traj_sent = True

            self._mirror_real_to_sim()

            q_home = np.array(self._home_joint_positions, dtype=float)
            if self._real_robot_converged(q_home):
                self.get_logger().info('RETRACT_HOME complete → HOME_HOLD')
                self.phase = Phase.HOME_HOLD
            else:
                self.get_logger().info(
                    'RETRACT_HOME  waiting for real robot…',
                    throttle_duration_sec=1.0)
            return

        # -- Sim-only: step through one-by-one --
        if self._retract_home_idx >= len(self._retract_home_path):
            self.get_logger().info('RETRACT_HOME complete → HOME_HOLD')
            self.phase = Phase.HOME_HOLD
            return

        q_cmd = self._retract_home_path[self._retract_home_idx]
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = self._home_joint_names
        cmd.position = q_cmd.tolist()
        self._pub_joint_cmd(cmd)
        self._last_q = q_cmd.copy()
        self._retract_home_idx += 1

        total = len(self._retract_home_path)
        if self._retract_home_idx % 20 == 0 or self._retract_home_idx >= total:
            pct = 100.0 * self._retract_home_idx / max(total - 1, 1)
            self.get_logger().info(
                f'RETRACT_HOME  {self._retract_home_idx}/{total} ({pct:.0f}%)',
                throttle_duration_sec=0.5)

    # ------------------------------------------------------------------
    # HOME_HOLD — hold at home, then proceed to PLANNING
    # ------------------------------------------------------------------
    def _home_hold(self):
        """Hold at the home position for _home_hold_duration before planning."""
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = self._home_joint_names
        cmd.position = self._home_joint_positions
        self._pub_joint_cmd(cmd)

        if self._home_reached_time is None:
            self._home_reached_time = self.get_clock().now()
            self.get_logger().info('HOME_HOLD — holding at home position…')

        elapsed = (self.get_clock().now() - self._home_reached_time).nanoseconds * 1e-9
        self.get_logger().info(
            f'HOME_HOLD  ({elapsed:.1f}/{self._home_hold_duration:.1f}s)',
            throttle_duration_sec=1.0)

        if elapsed >= self._home_hold_duration and self._real_robot_converged(
                np.array(self._home_joint_positions)):
            self.get_logger().info('HOME_HOLD complete → PLANNING')
            self.phase = Phase.PLANNING

    # ------------------------------------------------------------------
    # Elbow-up configuration check (matches cartesian controller)
    # ------------------------------------------------------------------
    def _config_ok(self, q):
        """Return True if joint config satisfies elbow-up constraints."""
        shoulder_lift = q[1]
        elbow = q[2]
        if shoulder_lift > self.shoulder_lift_max or shoulder_lift < self.shoulder_lift_min:
            return False
        if elbow > self.elbow_max or elbow < self.elbow_min:
            return False
        return True

    # ------------------------------------------------------------------
    # Constrained multi-seed IK (same algorithm as cartesian controller)
    # ------------------------------------------------------------------
    def _constrained_ik_for_pose(self, T_target):
        """Solve IK with many seeds, return best elbow-up solution."""
        from ur5e_rrt_planner import ik_solve
        import random

        home = np.array(self._home_joint_positions, dtype=float)
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
                f'elbow-up [shoulder_lift≤{self.shoulder_lift_max:.2f}, '
                f'elbow≤{self.elbow_max:.2f}]')
            return None

        candidates.sort(key=lambda x: x[0])
        best_q = candidates[0][1]
        self.get_logger().info(
            f'Constrained IK: {len(candidates)}/{len(seeds)} valid  '
            f'best shoulder_lift={best_q[1]:.3f} elbow={best_q[2]:.3f}  '
            f'joints={np.round(best_q, 3).tolist()}')
        return best_q

    # ------------------------------------------------------------------
    # IK with jump + config rejection (for descent waypoints)
    # ------------------------------------------------------------------
    def _safe_ik(self, T_target, q_seed):
        """IK solve with elbow-up check + joint-jump rejection."""
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

    @staticmethod
    def _pose44(pos, quat_xyzw):
        T = np.eye(4)
        T[:3, :3] = quat_to_rotmat(quat_xyzw)
        T[:3, 3] = pos
        return T

    # ------------------------------------------------------------------
    # PLANNING phase (constrained elbow-up IK + RRT, like cartesian ctrl)
    # ------------------------------------------------------------------
    def _plan(self):
        """Plan RRT path to approach pose using constrained IK (elbow-up)."""
        if self._last_joint_state is None:
            self.get_logger().warn('Waiting for joint states…',
                                   throttle_duration_sec=2.0)
            cmd = JointState()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.name = self._home_joint_names
            cmd.position = self._home_joint_positions
            self._pub_joint_cmd(cmd)
            return

        q_current = self._get_ordered_joints()
        if q_current is None:
            self.get_logger().warn('Cannot extract joint positions',
                                   throttle_duration_sec=2.0)
            return

        from ur5e_rrt_planner import (ur5e_fk, rrt_connect,
                                      smooth_path, bezier_smooth_path)

        # Approach position: above first waypoint (or plane center for teleop)
        if self.teleop_mode:
            on_plane_pos = self.plane_center.copy()
        else:
            on_plane_pos = self.waypoints[0][0].copy()

        approach_pos = on_plane_pos - self.approach_height * self.plane_n
        T_approach = self._pose44(approach_pos, self.target_quat)

        self.get_logger().info(
            f'PLANNING  approach_pos=[{approach_pos[0]:.3f}, '
            f'{approach_pos[1]:.3f}, {approach_pos[2]:.3f}]  '
            f'(above surface by {self.approach_height:.3f}m)')
        self.get_logger().info(
            f'PLANNING  q_current={np.round(q_current, 3).tolist()}')

        # Step 1: Constrained IK — find elbow-up goal
        q_goal = self._constrained_ik_for_pose(T_approach)
        if q_goal is None:
            self.get_logger().error(
                'PLANNING FAILED — no elbow-up IK solution. '
                'Try widening shoulder_lift_max / elbow_max.')
            return

        # Step 2: RRT-Connect in joint space
        raw_path = rrt_connect(q_current, q_goal,
                               step_size=0.2, max_iter=10000)
        if raw_path is None:
            self.get_logger().error('RRT planning FAILED — retrying…')
            return

        # Step 3: Smooth + Bezier spline interpolation
        smoothed = smooth_path(raw_path, max_attempts=200)
        path = bezier_smooth_path(smoothed, max_step=0.02)

        self._rrt_path = path
        self._rrt_path_idx = 0
        self._rrt_traj_sent = False
        # Pre-compute trapezoidal timing
        self._rrt_times = self._compute_joint_path_timing(path)

        T_goal = ur5e_fk(path[-1])
        goal_pos = T_goal[:3, 3]
        self.get_logger().info(
            f'PLANNING complete — {len(path)} joint waypoints → EXECUTING')
        self.get_logger().info(
            f'  Goal FK pos=[{goal_pos[0]:.4f}, {goal_pos[1]:.4f}, '
            f'{goal_pos[2]:.4f}]  shoulder_lift={path[-1][1]:.3f}  '
            f'elbow={path[-1][2]:.3f}')

        self.phase = Phase.EXECUTING

    # ------------------------------------------------------------------
    # EXECUTING phase (follow planned joint-space path)
    # ------------------------------------------------------------------
    def _execute(self):
        """Step through the RRT path, sending joint position commands."""
        # -- Real-robot mode: send the FULL trajectory once, then monitor --
        if self._real_robot:
            if not self._rrt_traj_sent:
                self._send_full_trajectory(self._rrt_path, self._rrt_times)
                self._rrt_traj_sent = True

            self._mirror_real_to_sim()

            # Progress logging
            real_q = self._get_real_ordered_joints()
            if real_q is not None:
                dists = [float(np.max(np.abs(real_q - q)))
                         for q in self._rrt_path]
                closest = int(np.argmin(dists))
                total = len(self._rrt_path) - 1
                pct = 100.0 * closest / max(total, 1)
                self.get_logger().info(
                    f'EXECUTING  ~step {closest}/{total}  ({pct:.0f}%)',
                    throttle_duration_sec=1.0)

            # Check convergence to final waypoint
            final_q = self._rrt_path[-1]
            if self._real_robot_converged(final_q):
                self.get_logger().info(
                    'RRT path complete → DESCENDING (position-controlled)')
                self._last_q = final_q.copy()

                if self.teleop_mode:
                    on_plane_pos = self.plane_center.copy()
                else:
                    on_plane_pos = self.waypoints[0][0].copy()
                approach_pos = (on_plane_pos
                                - self.approach_height * self.plane_n)
                dist = float(np.linalg.norm(on_plane_pos - approach_pos))
                n_steps = max(int(dist / self.descent_step), 2)
                self._descent_waypoints = []
                for i in range(n_steps + 1):
                    t = i / n_steps
                    pos = approach_pos + t * (on_plane_pos - approach_pos)
                    self._descent_waypoints.append((pos, self.target_quat))
                self._descent_idx = 0
                self.get_logger().info(
                    f'  Descent: {len(self._descent_waypoints)} waypoints, '
                    f'{self.descent_step:.3f}m steps')
                self.phase = Phase.DESCENDING
            return

        # -- Sim-only mode: step through one-by-one --
        if self._rrt_path_idx >= len(self._rrt_path):
            self.get_logger().info('RRT path complete → DESCENDING (position-controlled)')
            self._last_q = self._rrt_path[-1].copy()

            if self.teleop_mode:
                on_plane_pos = self.plane_center.copy()
            else:
                on_plane_pos = self.waypoints[0][0].copy()
            approach_pos = on_plane_pos - self.approach_height * self.plane_n

            dist = float(np.linalg.norm(on_plane_pos - approach_pos))
            n_steps = max(int(dist / self.descent_step), 2)
            self._descent_waypoints = []
            for i in range(n_steps + 1):
                t = i / n_steps
                pos = approach_pos + t * (on_plane_pos - approach_pos)
                self._descent_waypoints.append((pos, self.target_quat))
            self._descent_idx = 0
            self.get_logger().info(
                f'  Descent: {len(self._descent_waypoints)} waypoints, '
                f'{self.descent_step:.3f}m steps')
            self.phase = Phase.DESCENDING
            return

        q_cmd = self._rrt_path[self._rrt_path_idx]

        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = self._home_joint_names
        cmd.position = q_cmd.tolist()
        self._pub_joint_cmd(cmd)

        if self._rrt_path_idx % 10 == 0 or \
           self._rrt_path_idx == len(self._rrt_path) - 1:
            pct = 100.0 * self._rrt_path_idx / (len(self._rrt_path) - 1)
            self.get_logger().info(
                f'EXECUTING  step {self._rrt_path_idx}/{len(self._rrt_path)-1}  '
                f'({pct:.0f}%)')

        self._rrt_path_idx += 1

    # ------------------------------------------------------------------
    # DESCENDING — position-controlled IK descent (like cartesian ctrl)
    # ------------------------------------------------------------------
    def _descend(self):
        """Lower the EE via position-controlled IK joint commands."""
        if self._descent_idx >= len(self._descent_waypoints):
            if self.teleop_mode:
                self.get_logger().info('Descent complete → TELEOP')
                self.phase = Phase.TELEOP
            else:
                self.get_logger().info('Descent complete → SERVO')
                self.phase = Phase.SERVO
                self.waypoint_idx = 0
            return

        pos, quat = self._descent_waypoints[self._descent_idx]
        T = self._pose44(pos, quat)
        q_sol = self._safe_ik(T, self._last_q)

        if q_sol is None:
            self.get_logger().warn(
                f'DESCEND IK failed at step {self._descent_idx}/'
                f'{len(self._descent_waypoints)} — skipping')
            self._descent_idx += 1
            return

        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = self._home_joint_names
        cmd.position = q_sol.tolist()
        self._pub_joint_cmd(cmd)
        self._last_q = q_sol.copy()

        total = len(self._descent_waypoints)
        if self._descent_idx % 5 == 0 or self._descent_idx >= total - 1:
            _, _, z_off = self._to_plane(pos)
            ex, ey, _ = self._to_plane(pos)
            self.get_logger().info(
                f'DESCEND  wp={self._descent_idx}/{total}  '
                f'z_off={z_off:.4f}m  plane_xy=({ex:.3f},{ey:.3f})')

        if self._real_robot_converged(q_sol):
            self._descent_idx += 1

    # ------------------------------------------------------------------
    # Twist builder helpers
    # ------------------------------------------------------------------
    def _make_twist(self, lin_base, ang_base):
        tw = Twist()
        tw.linear.x  = float(lin_base[0])
        tw.linear.y  = float(lin_base[1])
        tw.linear.z  = float(lin_base[2])
        tw.angular.x = float(ang_base[0])
        tw.angular.y = float(ang_base[1])
        tw.angular.z = float(ang_base[2])
        return tw

    def _orient_vel(self, quat_now):
        """Angular velocity to correct orientation towards target_quat (PD)."""
        err = orientation_error_aa(quat_now, self.target_quat)

        # Derivative
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
    # Control phases (on-plane)
    # ------------------------------------------------------------------
    def _servo(self, pos, quat):
        """Follow trajectory waypoints using PD control, constrained to the plane."""
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

        # Decompose error into plane-frame components
        err_px = float(np.dot(err_base, self.plane_x))
        err_py = float(np.dot(err_base, self.plane_y))
        err_pz = float(np.dot(err_base, self.plane_n))  # off-plane drift

        # ---- Compute dt for derivative term ----
        now = self.get_clock().now()
        if self._prev_time is not None:
            dt = (now - self._prev_time).nanoseconds * 1e-9
        else:
            dt = 1.0 / self.execution_hz  # first tick
        dt = max(dt, 1e-6)  # safety

        # ---- Derivative of position error (in plane frame) ----
        d_err_px = (err_px - self._prev_err_px) / dt
        d_err_py = (err_py - self._prev_err_py) / dt
        d_err_pz = (err_pz - self._prev_err_pz) / dt

        # Store for next iteration
        self._prev_err_px = err_px
        self._prev_err_py = err_py
        self._prev_err_pz = err_pz
        self._prev_time = now

        # ---- PD: Planar velocity + Z drift correction ----
        vx = self.kp_lin * err_px + self.kd_lin * d_err_px
        vy = self.kp_lin * err_py + self.kd_lin * d_err_py
        vz = self.z_corr_gain * err_pz  # Z uses its own gain (no D needed)

        # Get current plane coords for boundary check
        ex, ey, _ = self._to_plane(pos)
        vx, vy = self._clamp_to_bounds(vx, vy, ex, ey)

        # Convert back to base frame
        vel_base = (vx * self.plane_x +
                    vy * self.plane_y +
                    vz * self.plane_n)
        vel_base = clamp_vec(vel_base, self.max_lin)

        ang = self._orient_vel(quat)

        dist = np.linalg.norm(err_base)
        self.get_logger().info(
            f'SERVO  wp={self.waypoint_idx}/{len(self.waypoints)}  '
            f'dist={dist:.4f}m  z_off={err_pz:.4f}m  '
            f'plane_xy=({ex:.3f},{ey:.3f})  '
            f'Kp={self.kp_lin:.2f} Kd={self.kd_lin:.3f}',
            throttle_duration_sec=1.0)

        if dist < self.wp_thresh:
            self.get_logger().info(
                f'Reached waypoint {self.waypoint_idx}')
            self.waypoint_idx += 1

        return self._make_twist(vel_base, ang)

    def _teleop(self, pos, quat):
        """Accept velocity commands from /teleop_plane_vel, constrained to the plane."""
        # Check for stale teleop commands (> 0.5s old → stop)
        dt = (self.get_clock().now() - self._teleop_stamp).nanoseconds * 1e-9
        if dt > 0.5:
            vx_cmd, vy_cmd = 0.0, 0.0
        else:
            vx_cmd = self._teleop_vel.linear.x   # plane X velocity
            vy_cmd = self._teleop_vel.linear.y   # plane Y velocity

        # Off-plane drift correction
        # ez is the EE position projected onto plane normal; drive it to zero.
        # Sign: _to_plane returns signed distance along normal, and
        # vel_base += vz * plane_n, so we need to negate to push BACK to the surface.
        _, _, ez = self._to_plane(pos)
        vz = -self.z_corr_gain * ez

        # Boundary clamping
        ex, ey, _ = self._to_plane(pos)
        vx_cmd, vy_cmd = self._clamp_to_bounds(vx_cmd, vy_cmd, ex, ey)

        # Convert plane-frame velocity to base frame
        vel_base = (vx_cmd * self.plane_x +
                    vy_cmd * self.plane_y +
                    vz * self.plane_n)
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
        # HOME, PLANNING, EXECUTING, DESCENDING send joint commands directly
        if self.phase == Phase.RETRACT_UP:
            self._retract_up()
            return
        if self.phase == Phase.RETRACT_HOME:
            self._retract_home()
            return
        if self.phase == Phase.HOME_HOLD:
            self._home_hold()
            return
        if self.phase == Phase.PLANNING:
            self._plan()
            return
        if self.phase == Phase.EXECUTING:
            self._execute()
            return
        if self.phase == Phase.DESCENDING:
            self._descend()        # position-controlled, no twist needed
            return

        # SERVO / TELEOP / DONE use velocity commands via Jacobian node
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
