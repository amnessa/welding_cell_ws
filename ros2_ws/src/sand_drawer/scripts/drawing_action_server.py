#!/usr/bin/env python3
"""
Drawing Action Server — Sequential action-based drawing controller.

Receives drawing goals (Cartesian waypoints + orientation) via a ROS 2 action
interface.  For each goal the robot executes a pre-computed trajectory through:

    RETRACT_UP → [HOMING] → RRT → DESCENT → DRAWING → ASCENT

After completing a drawing the server returns to IDLE and waits for the next
goal.  The first goal includes a HOMING phase; subsequent goals skip it
(the robot is already at a safe hover from the previous ascent).

On shutdown (Ctrl-C) the server commands RETRACT_UP → HOMING to park safely.

Architecture
------------
  For each incoming goal the entire motion is pre-computed in joint space
  (IK, RRT, Bezier-smoothed C-space paths, per-phase trapezoidal timing)
  before any motion begins — identical to the proven monolithic approach in
  cartesian_square_controller.py.  The only structural change is that the
  pre-computation is now triggered *per goal* instead of once at startup.

Reuses
------
  ur5e_rrt_planner.py — FK, IK, RRT-Connect, Catmull-Rom path smoothing

Action interface
----------------
  sand_drawer/action/ExecuteDrawing
    Goal:     geometry_msgs/Point[] waypoints
              geometry_msgs/Quaternion orientation
    Result:   bool success, string message
    Feedback: string current_phase, float32 drawing_progress

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
import time as _time
from typing import List, Optional, Tuple

# Ensure sibling scripts are importable (ur5e_rrt_planner.py etc.)
_scripts_dir = os.path.dirname(os.path.realpath(__file__))
if _scripts_dir not in sys.path:
    sys.path.insert(0, _scripts_dir)

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Quaternion
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from sand_drawer.action import ExecuteDrawing


# ═══════════════════════════════════════════════════════════════════════════
# Quaternion / rotation helpers (self-contained math)
# ═══════════════════════════════════════════════════════════════════════════

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


# ═══════════════════════════════════════════════════════════════════════════
# Cartesian interpolation with trapezoidal velocity profile
# ═══════════════════════════════════════════════════════════════════════════

def _interpolate_cartesian_smooth(
    positions: List[np.ndarray],
    orientation_xyzw: list,
    v_max: float = 0.05,
    a_max: float = 0.05,
    dt: float = 0.01,
) -> List[Tuple[np.ndarray, list]]:
    """Dense Cartesian path with per-segment trapezoidal velocity profile."""
    if not positions or len(positions) < 2:
        return [(positions[0].copy(), orientation_xyzw)] if positions else []

    waypoints: List[Tuple[np.ndarray, list]] = []
    for i in range(len(positions) - 1):
        p_start, p_end = positions[i], positions[i + 1]
        dist = float(np.linalg.norm(p_end - p_start))
        if dist < 1e-5:
            continue
        line_dir = (p_end - p_start) / dist

        t_accel = v_max / a_max
        d_accel = 0.5 * a_max * t_accel**2

        if 2 * d_accel > dist:
            d_accel = dist / 2.0
            t_accel = math.sqrt(2 * d_accel / a_max)
            actual_v_max = a_max * t_accel
            t_cruise, d_cruise = 0.0, 0.0
        else:
            actual_v_max = v_max
            d_cruise = dist - 2 * d_accel
            t_cruise = d_cruise / actual_v_max

        total_time = 2 * t_accel + t_cruise
        n_steps = max(int(total_time / dt), 1)

        for step in range(n_steps):
            t = step * dt
            if t < t_accel:
                s = 0.5 * a_max * t**2
            elif t < t_accel + t_cruise:
                s = d_accel + actual_v_max * (t - t_accel)
            else:
                t_dec = t - t_accel - t_cruise
                s = (d_accel + d_cruise
                     + actual_v_max * t_dec - 0.5 * a_max * t_dec**2)
            waypoints.append((p_start + line_dir * min(s, dist),
                              orientation_xyzw))

    waypoints.append((positions[-1].copy(), orientation_xyzw))
    return waypoints


# ═══════════════════════════════════════════════════════════════════════════
# Drawing Action Server Node
# ═══════════════════════════════════════════════════════════════════════════

class DrawingActionServer(Node):
    """ROS 2 action server that executes sequential sand drawings."""

    # ------------------------------------------------------------------
    # Initialisation
    # ------------------------------------------------------------------
    def __init__(self):
        super().__init__('drawing_action_server')

        # ---- parameters ----
        self.declare_parameter('plane_json_file', '')
        self.declare_parameter('approach_height', 0.08)
        self.declare_parameter('max_linear_vel', 0.05)
        self.declare_parameter('max_linear_accel', 0.05)
        self.declare_parameter('approach_linear_vel', 0.20)
        self.declare_parameter('approach_linear_accel', 0.20)
        self.declare_parameter('ik_damping', 0.05)
        self.declare_parameter('execution_hz', 100.0)
        self.declare_parameter('max_joint_step', 0.15)
        self.declare_parameter('shoulder_lift_max', 0.0)
        self.declare_parameter('shoulder_lift_min', -2.5)
        self.declare_parameter('elbow_max', -0.3)
        self.declare_parameter('elbow_min', -3.14)
        self.declare_parameter('ik_num_seeds', 30)
        self.declare_parameter('real_robot', False)
        self.declare_parameter('real_robot_joint_state_topic', '/joint_states')
        self.declare_parameter('real_robot_trajectory_topic',
                               '/scaled_joint_trajectory_controller/joint_trajectory')
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
        self._max_joint_speed_rad = math.radians(self._max_joint_speed_deg)
        self._max_joint_accel_rad = math.radians(self._max_joint_accel_deg)
        self._dt = 1.0 / self.execution_hz
        self._prev_cmd_q: Optional[np.ndarray] = None

        # ---- safe retract / home hold settings ----
        self._retract_height = 0.15
        self._home_hold_sec = 0.5

        # ---- IK diagnostics (reset per goal) ----
        self._ik_fail_count = 0
        self._ik_jump_count = 0
        self._ik_cfg_count = 0
        self._ik_total_count = 0

        # ---- state tracking ----
        self._first_goal = True
        self._executing = False
        self._last_q: Optional[np.ndarray] = None

        # ---- action server ----
        self._action_server = ActionServer(
            self,
            ExecuteDrawing,
            'execute_drawing',
            execute_callback=self._execute_goal,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info(
            f'Drawing action server ready — '
            f'approach_height={self.approach_height:.3f}m, '
            f'execution_hz={self.execution_hz:.0f}, '
            f'real_robot={self._real_robot}')

    # ------------------------------------------------------------------
    # Parameter helpers
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
        self.max_joint_step      = g('max_joint_step').value
        self.shoulder_lift_max   = g('shoulder_lift_max').value
        self.shoulder_lift_min   = g('shoulder_lift_min').value
        self.elbow_max           = g('elbow_max').value
        self.elbow_min           = g('elbow_min').value
        self.ik_num_seeds        = g('ik_num_seeds').value
        self._max_joint_speed_deg = float(g('max_joint_speed_deg').value)
        self._max_joint_accel_deg = float(g('max_joint_accel_deg').value)

    # ------------------------------------------------------------------
    # Plane JSON loading — only plane geometry (normal, etc.)
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

            p = data['plane']
            p['origin'] = _rz(p['origin'])
            p['x_axis'] = _rz(p['x_axis'])
            p['y_axis'] = _rz(p['y_axis'])
            p['normal'] = _rz(p['normal'])

        # Store plane geometry
        plane = data['plane']
        self.plane_origin = np.array(plane['origin'], dtype=float)
        self.plane_x = np.array(plane['x_axis'], dtype=float)
        self.plane_y = np.array(plane['y_axis'], dtype=float)
        self.plane_n = np.array(plane['normal'], dtype=float)

        self.get_logger().info(
            f'Loaded plane: origin={self.plane_origin.tolist()}, '
            f'normal={self.plane_n.tolist()}')

    # ------------------------------------------------------------------
    # Joint-state callbacks
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
        """Mirror real robot joint state → Isaac Sim joint commands."""
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

        # Real-robot mode: trajectory already sent, just mirror
        self._mirror_real_to_sim()

    # ------------------------------------------------------------------
    # 4×4 pose from position + quaternion
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
        import random as _random

        home = self._home_positions.copy()
        candidates = []
        seeds = [home.copy()]
        for _ in range(self.ik_num_seeds):
            s = home.copy()
            s[0] += _random.uniform(-1.5, 1.5)
            s[1] += _random.uniform(-1.0, 0.3)
            s[2] += _random.uniform(-0.5, 0.5)
            s[3] += _random.uniform(-1.0, 1.0)
            s[4] += _random.uniform(-1.0, 1.0)
            s[5] += _random.uniform(-1.0, 1.0)
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
    # Batch IK for a Cartesian waypoint sequence
    # ------------------------------------------------------------------
    def _ik_solve_cartesian_path(
        self,
        cart_wps: List[Tuple[np.ndarray, list]],
        q_seed: np.ndarray,
        label: str = 'path',
    ) -> List[np.ndarray]:
        """Solve IK for every (pos, quat) waypoint. Skips failed points."""
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
    # Trapezoidal timing for joint-space paths (per phase)
    # ------------------------------------------------------------------
    def _compute_phase_timing(self, path: List[np.ndarray]) -> List[float]:
        """Trapezoidal velocity profile over one phase (segment group).

        Builds cumulative joint-space arc length, then maps a single
        accel → cruise → decel onto it.
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
            return [i * 0.01 for i in range(len(path))]

        # Trapezoidal profile parameters
        d_accel = v_max * v_max / (2.0 * a_max)
        if 2.0 * d_accel >= total_dist:
            d_accel = total_dist / 2.0
            v_peak = math.sqrt(2.0 * a_max * d_accel)
            t_accel = v_peak / a_max
            t_cruise = 0.0
        else:
            v_peak = v_max
            t_accel = v_max / a_max
            t_cruise = (total_dist - 2.0 * d_accel) / v_max
        d_decel_start = total_dist - d_accel

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

    # ------------------------------------------------------------------
    # Concatenate phases → master timeline with phase boundaries
    # ------------------------------------------------------------------
    def _concat_phase_times(
        self, phases: List[Tuple[str, List[np.ndarray]]]
    ) -> Tuple[List[np.ndarray], List[float],
               List[Tuple[str, int, int]]]:
        """Concatenate independently-timed phases.

        Returns
        -------
        master_path  : joint-space waypoints
        master_times : cumulative timestamps
        phase_info   : [(label, start_idx, end_idx), ...]
        """
        master_path: List[np.ndarray] = []
        master_times: List[float] = []
        phase_info: List[Tuple[str, int, int]] = []
        t_offset = 0.0

        for label, plist in phases:
            if not plist:
                continue
            phase_times = self._compute_phase_timing(plist)
            start_idx = len(master_path)
            for j, (q, t) in enumerate(zip(plist, phase_times)):
                if j == 0 and master_path:
                    continue  # skip duplicate boundary point
                master_path.append(np.asarray(q, dtype=float))
                master_times.append(t_offset + t)
            end_idx = len(master_path) - 1
            if phase_times:
                t_offset = master_times[-1]
            phase_info.append((label, start_idx, end_idx))
            dur = phase_times[-1] if phase_times else 0.0
            self.get_logger().info(
                f'  Phase: {label:15s}  {len(plist):4d} pts  {dur:.2f}s')

        return master_path, master_times, phase_info

    # ------------------------------------------------------------------
    # Send full trajectory to real robot
    # ------------------------------------------------------------------
    def _send_full_trajectory(self, path: List[np.ndarray],
                              times: List[float]) -> None:
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

    # ══════════════════════════════════════════════════════════════════
    # Action callbacks
    # ══════════════════════════════════════════════════════════════════

    def _goal_callback(self, goal_request):
        """Accept or reject an incoming goal."""
        if self._executing:
            self.get_logger().warn('Busy executing — rejecting new goal')
            return GoalResponse.REJECT
        n_wps = len(goal_request.waypoints)
        self.get_logger().info(f'Goal received: {n_wps} waypoints')
        if n_wps < 2:
            self.get_logger().warn(
                f'Goal rejected: need ≥2 waypoints, got {n_wps}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        """Always accept cancel requests — controller will finish
        current phase boundary then stop."""
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    # ══════════════════════════════════════════════════════════════════
    # Goal execution — main entry point for each drawing
    # ══════════════════════════════════════════════════════════════════

    def _execute_goal(self, goal_handle):
        """Process one drawing goal through the full phase sequence."""
        self._executing = True
        self._ik_fail_count = 0
        self._ik_jump_count = 0
        self._ik_cfg_count = 0
        self._ik_total_count = 0

        try:
            return self._execute_goal_inner(goal_handle)
        finally:
            self._executing = False

    def _execute_goal_inner(self, goal_handle):
        feedback = ExecuteDrawing.Feedback()

        # ---- extract goal data ----
        draw_positions = [
            np.array([p.x, p.y, p.z], dtype=float)
            for p in goal_handle.request.waypoints
        ]
        q = goal_handle.request.orientation
        orientation = [q.x, q.y, q.z, q.w]

        self.get_logger().info(
            f'Executing drawing: {len(draw_positions)} waypoints')

        # ---- wait for joint state ----
        feedback.current_phase = 'WAITING_JOINTS'
        feedback.drawing_progress = 0.0
        goal_handle.publish_feedback(feedback)

        for _ in range(100):  # 10 s timeout
            if self._get_ordered_joints() is not None:
                break
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return ExecuteDrawing.Result(
                    success=False, message='Canceled while waiting for joints')
            _time.sleep(0.1)
        else:
            goal_handle.abort()
            return ExecuteDrawing.Result(
                success=False, message='Timed out waiting for joint states')

        # ---- pre-compute trajectory (with retries) ----
        feedback.current_phase = 'PRE_COMPUTING'
        goal_handle.publish_feedback(feedback)

        max_retries = 3
        phases = None
        for attempt in range(1, max_retries + 1):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return ExecuteDrawing.Result(
                    success=False, message='Canceled during pre-computation')
            phases = self._pre_compute_goal(draw_positions, orientation)
            if phases is not None:
                break
            self.get_logger().warn(
                f'Pre-computation attempt {attempt}/{max_retries} failed')
            _time.sleep(0.5)

        if phases is None:
            goal_handle.abort()
            return ExecuteDrawing.Result(
                success=False,
                message='Pre-computation failed after all retries')

        # ---- concat & time the phases ----
        master_path, master_times, phase_info = \
            self._concat_phase_times(phases)

        total_dur = master_times[-1] if master_times else 0.0
        self.get_logger().info(
            f'Pre-computation complete: {len(master_path)} pts, '
            f'{total_dur:.1f}s, '
            f'IK stats: {self._ik_fail_count} fails, '
            f'{self._ik_jump_count} jumps, '
            f'{self._ik_cfg_count} config-rejects '
            f'/ {self._ik_total_count} total')

        # ---- execute ----
        if self._real_robot:
            success = self._execute_real_trajectory(
                master_path, master_times, phase_info, goal_handle, feedback)
        else:
            success = self._execute_sim_trajectory(
                master_path, master_times, phase_info, goal_handle, feedback)

        # ---- first-goal flag (homing done) ----
        if success:
            self._first_goal = False

        # ---- result ----
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return ExecuteDrawing.Result(
                success=False, message='Drawing canceled')
        if success:
            self._last_q = master_path[-1].copy()
            goal_handle.succeed()
            self.get_logger().info('Drawing complete ✓')
            return ExecuteDrawing.Result(
                success=True, message='Drawing complete')
        else:
            goal_handle.abort()
            return ExecuteDrawing.Result(
                success=False, message='Execution failed')

    # ══════════════════════════════════════════════════════════════════
    # Pre-compute trajectory for one drawing goal
    # ══════════════════════════════════════════════════════════════════

    def _pre_compute_goal(
        self,
        draw_positions: List[np.ndarray],
        orientation_xyzw: list,
    ) -> Optional[List[Tuple[str, List[np.ndarray]]]]:
        """Build all phases for one drawing. Returns list of
        (label, joint_path) tuples or None on failure."""
        from ur5e_rrt_planner import (ur5e_fk, rrt_connect,
                                      smooth_path, bezier_smooth_path)

        q_current = self._get_ordered_joints()
        if q_current is None:
            return None
        cart_dt = 1.0 / self.execution_hz
        phases: List[Tuple[str, List[np.ndarray]]] = []

        self.get_logger().info(
            '═══════════════════════════════════════════════\n'
            '  PRE-COMPUTING TRAJECTORY FOR GOAL\n'
            '═══════════════════════════════════════════════')

        # ── 1. RETRACT UP (Cartesian vertical lift) ───────────────────
        T_now = ur5e_fk(q_current)
        current_pos = T_now[:3, 3].copy()
        current_quat = rotmat_to_quat(T_now[:3, :3])

        lift_pos = current_pos.copy()
        lift_pos[2] += self._retract_height

        self.get_logger().info(
            f'  [RETRACT_UP] z={current_pos[2]:.3f} → '
            f'z={lift_pos[2]:.3f} (+{self._retract_height:.3f}m)')

        retract_wps = _interpolate_cartesian_smooth(
            [current_pos, lift_pos], current_quat,
            self.approach_v_max, self.approach_a_max, cart_dt)
        retract_path = self._ik_solve_cartesian_path(
            retract_wps, q_current, 'Retract up')
        if not retract_path:
            self.get_logger().error('RETRACT_UP IK failed')
            return None
        phases.append(('retract_up', retract_path))

        # ── 2. HOMING (first goal only) ──────────────────────────────
        if self._first_goal:
            q_retracted = retract_path[-1]
            q_home = self._home_positions.copy()
            n_home = max(int(1.0 * self.execution_hz), 20)

            retract_home_path = [q_retracted.copy()]
            for i in range(1, n_home + 1):
                alpha = 0.5 * (1.0 - math.cos(math.pi * i / n_home))
                q_interp = (1.0 - alpha) * q_retracted + alpha * q_home
                retract_home_path.append(q_interp)
            phases.append(('retract_home', retract_home_path))

            n_hold = max(int(self._home_hold_sec * self.execution_hz), 1)
            home_hold_path = [q_home.copy() for _ in range(n_hold + 1)]
            phases.append(('home_hold', home_hold_path))

            self.get_logger().info(
                f'  [HOMING] {n_home} interp steps + '
                f'{self._home_hold_sec:.1f}s hold')

        # ── 3. RRT (last phase end → approach above first draw point) ─
        last_q = phases[-1][1][-1]
        approach_pos = (draw_positions[0]
                        - self.approach_height * self.plane_n)
        T_approach = self._pose44(approach_pos, orientation_xyzw)

        self.get_logger().info(
            f'  [RRT] planning → approach '
            f'[{approach_pos[0]:.3f}, {approach_pos[1]:.3f}, '
            f'{approach_pos[2]:.3f}]')

        q_goal = self._constrained_ik_for_pose(T_approach)
        if q_goal is None:
            self.get_logger().error('RRT goal IK failed')
            return None

        raw_rrt = rrt_connect(last_q, q_goal,
                              step_size=0.2, max_iter=10000)
        if raw_rrt is None:
            self.get_logger().error('RRT planning failed')
            return None

        smoothed = smooth_path(raw_rrt, max_attempts=200)
        rrt_dense = bezier_smooth_path(smoothed, max_step=0.02)
        phases.append(('rrt', list(rrt_dense)))

        self.get_logger().info(
            f'        {len(raw_rrt)} raw → {len(smoothed)} smoothed '
            f'→ {len(rrt_dense)} dense')

        # Verify approach FK
        T_goal_fk = ur5e_fk(rrt_dense[-1])
        goal_pos = T_goal_fk[:3, 3]
        self.get_logger().info(
            f'        Approach FK = [{goal_pos[0]:.4f}, '
            f'{goal_pos[1]:.4f}, {goal_pos[2]:.4f}]')

        # ── 4. DESCENT (approach → surface) ───────────────────────────
        descent_wps = _interpolate_cartesian_smooth(
            [approach_pos, draw_positions[0].copy()],
            orientation_xyzw,
            self.approach_v_max, self.approach_a_max, cart_dt)
        descent_path = self._ik_solve_cartesian_path(
            descent_wps, rrt_dense[-1], 'Descent')
        if descent_path:
            phases.append(('descent', descent_path))
        else:
            self.get_logger().warn(
                '  Descent IK failed — drawing starts from approach height')

        # ── 5. DRAWING (surface waypoints) ────────────────────────────
        draw_wps = _interpolate_cartesian_smooth(
            draw_positions, orientation_xyzw,
            self.v_max, self.a_max, cart_dt)
        last_q_for_draw = phases[-1][1][-1]
        draw_path = self._ik_solve_cartesian_path(
            draw_wps, last_q_for_draw, 'Drawing')
        if not draw_path or len(draw_path) < 2:
            self.get_logger().error('Drawing IK returned too few solutions')
            return None
        phases.append(('drawing', draw_path))

        # ── 6. ASCENT (surface → approach height) ─────────────────────
        last_draw_pos = draw_positions[-1].copy()
        ascent_pos = last_draw_pos - self.approach_height * self.plane_n
        ascent_wps = _interpolate_cartesian_smooth(
            [last_draw_pos, ascent_pos], orientation_xyzw,
            self.approach_v_max, self.approach_a_max, cart_dt)
        ascent_path = self._ik_solve_cartesian_path(
            ascent_wps, draw_path[-1], 'Ascent')
        if ascent_path:
            phases.append(('ascent', ascent_path))
        else:
            self.get_logger().warn('Ascent IK failed — holding last draw pos')

        self.get_logger().info(
            '═══════════════════════════════════════════════')

        return phases

    # ══════════════════════════════════════════════════════════════════
    # Execution: Simulation (time-based interpolation)
    # ══════════════════════════════════════════════════════════════════

    def _execute_sim_trajectory(
        self,
        master_path: List[np.ndarray],
        master_times: List[float],
        phase_info: List[Tuple[str, int, int]],
        goal_handle,
        feedback: ExecuteDrawing.Feedback,
    ) -> bool:
        """Play back the pre-computed trajectory in simulation."""

        total_time = master_times[-1]
        start_time = self.get_clock().now()
        last_feedback_time = 0.0

        while True:
            # -- cancel check --
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Execution canceled')
                return False

            elapsed = (self.get_clock().now()
                       - start_time).nanoseconds * 1e-9

            if elapsed >= total_time:
                # Publish final position
                self._pub_joints(master_path[-1])
                self._last_q = master_path[-1].copy()
                break

            # -- interpolate --
            idx = int(np.searchsorted(master_times, elapsed,
                                      side='right')) - 1
            idx = max(0, min(idx, len(master_path) - 2))
            t0 = master_times[idx]
            t1 = master_times[idx + 1]
            alpha = (elapsed - t0) / max(t1 - t0, 1e-9)
            alpha = max(0.0, min(1.0, alpha))

            q_interp = ((1.0 - alpha) * master_path[idx]
                        + alpha * master_path[idx + 1])
            self._pub_joints(q_interp)
            self._last_q = q_interp.copy()

            # -- feedback (throttled to ~2 Hz) --
            if elapsed - last_feedback_time >= 0.5:
                last_feedback_time = elapsed
                phase_label = self._phase_for_idx(idx, phase_info)
                draw_pct = self._drawing_progress(idx, phase_info)
                feedback.current_phase = phase_label.upper()
                feedback.drawing_progress = draw_pct
                goal_handle.publish_feedback(feedback)
                self.get_logger().info(
                    f'  t={elapsed:.1f}/{total_time:.1f}s  '
                    f'[{phase_label}]  draw={draw_pct*100:.0f}%')

            _time.sleep(self._dt)

        return True

    # ══════════════════════════════════════════════════════════════════
    # Execution: Real robot (send trajectory + convergence monitoring)
    # ══════════════════════════════════════════════════════════════════

    def _execute_real_trajectory(
        self,
        master_path: List[np.ndarray],
        master_times: List[float],
        phase_info: List[Tuple[str, int, int]],
        goal_handle,
        feedback: ExecuteDrawing.Feedback,
    ) -> bool:
        """Send trajectory to real robot and monitor convergence."""

        self._send_full_trajectory(master_path, master_times)
        target_q = master_path[-1]
        total_time = master_times[-1]
        t0 = self.get_clock().now()
        last_feedback_time = 0.0

        # Monitor at 10 Hz — the full trajectory is already on the robot
        # controller; we only need to watch convergence, not drive the motion.
        monitor_dt = 0.1

        # Allow extra time for real robot to finish (2× trajectory duration)
        timeout = total_time * 2.0 + 10.0

        while True:
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Execution canceled (real)')
                return False

            try:
                self._mirror_real_to_sim()
            except Exception:
                pass  # best-effort sim mirror

            elapsed = (self.get_clock().now() - t0).nanoseconds * 1e-9

            # Convergence check
            if self._real_robot_converged(target_q):
                self._last_q = target_q.copy()
                self.get_logger().info('Real robot converged to target')
                return True

            if elapsed > timeout:
                self.get_logger().error(
                    f'Real robot timed out ({timeout:.0f}s)')
                return False

            # Estimate progress from nearest master_path point
            real_q = self._get_real_ordered_joints()
            if real_q is not None and elapsed - last_feedback_time >= 1.0:
                last_feedback_time = elapsed
                dists = [float(np.max(np.abs(real_q - q)))
                         for q in master_path]
                closest = int(np.argmin(dists))
                phase_label = self._phase_for_idx(closest, phase_info)
                draw_pct = self._drawing_progress(closest, phase_info)
                feedback.current_phase = phase_label.upper()
                feedback.drawing_progress = draw_pct
                goal_handle.publish_feedback(feedback)
                pct = 100.0 * closest / max(len(master_path) - 1, 1)
                self.get_logger().info(
                    f'  ~step {closest}/{len(master_path)-1}  '
                    f'({pct:.0f}%)  [{phase_label}]')

            _time.sleep(monitor_dt)

    # ------------------------------------------------------------------
    # Phase lookup helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _phase_for_idx(
        idx: int, phase_info: List[Tuple[str, int, int]]
    ) -> str:
        for label, start, end in phase_info:
            if start <= idx <= end:
                return label
        return 'unknown'

    @staticmethod
    def _drawing_progress(
        idx: int, phase_info: List[Tuple[str, int, int]]
    ) -> float:
        for label, start, end in phase_info:
            if label == 'drawing':
                if idx < start:
                    return 0.0
                elif idx > end:
                    return 1.0
                else:
                    return (idx - start) / max(end - start, 1)
        return 0.0

    # ══════════════════════════════════════════════════════════════════
    # Shutdown: retract + home on Ctrl-C
    # ══════════════════════════════════════════════════════════════════

    def shutdown_sequence(self):
        """Best-effort retract → home on shutdown."""
        self.get_logger().info('Shutdown sequence: retract + home …')

        try:
            q_current = self._get_ordered_joints()
        except Exception:
            q_current = None
        if q_current is None:
            self.get_logger().warn(
                'No joint state available — skipping shutdown sequence')
            return

        try:
            from ur5e_rrt_planner import ur5e_fk

            # Build retract + homing path
            T_now = ur5e_fk(q_current)
            current_pos = T_now[:3, 3].copy()
            current_quat = rotmat_to_quat(T_now[:3, :3])
            cart_dt = 1.0 / self.execution_hz

            lift_pos = current_pos.copy()
            lift_pos[2] += self._retract_height

            retract_wps = _interpolate_cartesian_smooth(
                [current_pos, lift_pos], current_quat,
                self.approach_v_max, self.approach_a_max, cart_dt)
            retract_path = self._ik_solve_cartesian_path(
                retract_wps, q_current, 'Shutdown retract')

            # Homing (cosine joint interpolation)
            q_retracted = retract_path[-1] if retract_path else q_current
            q_home = self._home_positions.copy()
            n_home = max(int(1.0 * self.execution_hz), 20)
            home_path = [q_retracted.copy()]
            for i in range(1, n_home + 1):
                alpha = 0.5 * (1.0 - math.cos(math.pi * i / n_home))
                home_path.append(
                    (1.0 - alpha) * q_retracted + alpha * q_home)

            full_path = (retract_path if retract_path else [q_current]) + \
                home_path[1:]

            if self._real_robot:
                # Send trajectory to real robot (fire-and-forget)
                times = self._compute_phase_timing(full_path)
                self._send_full_trajectory(full_path, times)
                self.get_logger().info(
                    'Shutdown trajectory sent to real robot')
            else:
                # Sim: play back quickly (best effort)
                for q in full_path:
                    self._pub_joints(q)
                    _time.sleep(self._dt)
                self.get_logger().info(
                    'Shutdown sequence published to sim')
        except Exception as exc:
            # Context may already be invalid after Ctrl-C — swallow errors
            self.get_logger().warn(
                f'Shutdown sequence could not complete: {exc}')


# ═══════════════════════════════════════════════════════════════════════════
# Entry point
# ═══════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    server = DrawingActionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown_sequence()
        server.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
