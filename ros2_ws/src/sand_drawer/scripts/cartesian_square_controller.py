#!/usr/bin/env python3
"""
Cartesian Drawing Controller — Position-controlled drawing on the plane.

Pipeline
--------
1. RETRACT_UP   – Lift end-effector straight up to safe clearance (Cartesian IK)
2. RETRACT_HOME – Interpolate in joint-space from retracted pose to home
3. HOME_HOLD    – Hold at home for 2 s (verify stability)
4. PLANNING     – RRT-Connect from home to approach pose (above first point)
5. EXECUTING    – Follow the planned RRT joint-space path
6. DESCENDING   – Linearly descend from approach height to the plane surface
5. DRAWING    – Follow dense Cartesian waypoints along the path
6. ASCENDING  – Lift off the plane
7. DONE       – Hold position (zero velocity)

Trajectory sources (via trajectory_key parameter):
  'line'                         — a straight line on the plane (UV coordinates)
  'square_trajectory'            — from plane JSON square_trajectory key
  'projected_vector_trajectory'  — from plane JSON projected_vector_trajectory key (may not be used)

During DRAWING the end-effector orientation is kept perpendicular to the
drawing surface (aligned with the plane normal).  The Cartesian path is
densely interpolated in task space and converted to joint space via damped-
least-squares IK step by step—analogous to MoveIt 2 computeCartesianPath()
but implemented locally so we stay independent of a running MoveGroup.

Publishes
---------
  /isaac_joint_commands  (sensor_msgs/JointState)
    Position-controlled joint targets at 100Hz.

Subscribes
----------
  /isaac_joint_states    (sensor_msgs/JointState)
    Current joint feedback for IK seeding.

Reads
-----
  The plane JSON produced by plane_solver_node (square_trajectory key).
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
from rclpy.node import Node
from sensor_msgs.msg import JointState


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
# Cartesian interpolation helpers
# ---------------------------------------------------------------------------

def _interpolate_cartesian_smooth(
    positions: List[np.ndarray],
    orientation_xyzw: list,
    v_max: float = 0.05,   # Maximum cruising speed (m/s)
    a_max: float = 0.05,   # Maximum acceleration (m/s^2)
    dt: float = 1.0/60.0   # Timer tick duration (1.0 / execution_hz)
) -> List[Tuple[np.ndarray, list]]:
    """
    Generates a dense Cartesian path using a Trapezoidal Velocity Profile.
    This guarantees smooth acceleration and deceleration between all points.
    """
    if not positions or len(positions) < 2:
        return [(positions[0].copy(), orientation_xyzw)] if positions else []

    waypoints = []

    # Loop through each line segment
    for i in range(len(positions) - 1):
        p_start = positions[i]
        p_end = positions[i + 1]

        dist = float(np.linalg.norm(p_end - p_start))
        if dist < 1e-5:
            continue

        line_dir = (p_end - p_start) / dist

        # 1. Calculate the time needed to accelerate and decelerate
        t_accel = v_max / a_max
        d_accel = 0.5 * a_max * (t_accel ** 2)

        # 2. Check if the line is too short to reach max velocity
        if 2 * d_accel > dist:
            # Triangle profile: We must start braking before reaching v_max
            d_accel = dist / 2.0
            t_accel = math.sqrt(2 * d_accel / a_max)
            actual_v_max = a_max * t_accel
            t_cruise = 0.0
            d_cruise = 0.0
        else:
            # Trapezoid profile: Accelerate, Cruise, Decelerate
            actual_v_max = v_max
            d_cruise = dist - (2 * d_accel)
            t_cruise = d_cruise / actual_v_max

        total_time = (2 * t_accel) + t_cruise
        n_steps = max(int(total_time / dt), 1)

        # 3. Generate the precise point for every time step
        for step in range(n_steps):
            t = step * dt

            if t < t_accel:
                # Acceleration phase
                s = 0.5 * a_max * (t ** 2)
            elif t < t_accel + t_cruise:
                # Cruising phase
                s = d_accel + actual_v_max * (t - t_accel)
            else:
                # Deceleration phase
                t_dec = t - t_accel - t_cruise
                s = d_accel + d_cruise + (actual_v_max * t_dec) - (0.5 * a_max * (t_dec ** 2))

            # Clamp to ensure no floating point overshoot
            s = min(s, dist)

            p = p_start + (line_dir * s)
            waypoints.append((p.copy(), orientation_xyzw))

    # Always append the exact final point of the entire sequence
    waypoints.append((positions[-1].copy(), orientation_xyzw))
    return waypoints


# ---------------------------------------------------------------------------
# Phase enum
# ---------------------------------------------------------------------------

class Phase(Enum):
    RETRACT_UP   = auto()   # Cartesian lift to safe height
    RETRACT_HOME = auto()   # Joint-space interpolation to home
    HOME_HOLD    = auto()   # Hold at home position
    PLANNING     = auto()
    EXECUTING    = auto()
    DESCENDING   = auto()
    DRAWING      = auto()
    ASCENDING    = auto()
    DONE         = auto()


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class CartesianDrawController(Node):
    def __init__(self):
        super().__init__('cartesian_draw_controller')

        # ---- parameters ----
        self.declare_parameter('plane_json_file', '')
        self.declare_parameter('approach_height', 0.08)
        self.declare_parameter('max_linear_vel', 0.05)        # m/s cruising speed
        self.declare_parameter('max_linear_accel', 0.05)      # m/s^2 max acceleration
        self.declare_parameter('ik_damping', 0.05)
        self.declare_parameter('execution_hz', 60.0)          # joint command rate
        self.declare_parameter('waypoints_per_tick', 1)       # Cartesian wp to advance per tick
        self.declare_parameter('loop_trajectory', False)
        self.declare_parameter('trajectory_key', 'line')      # 'line' | 'square_trajectory' | ...
        self.declare_parameter('max_joint_step', 0.15)        # rad — reject IK jumps larger than this
        # Elbow-up configuration constraints — prevents table collisions
        self.declare_parameter('shoulder_lift_max', 0.0)
        self.declare_parameter('shoulder_lift_min', -2.5)
        self.declare_parameter('elbow_max', -0.3)
        self.declare_parameter('elbow_min', -3.14)
        self.declare_parameter('ik_num_seeds', 30)
        # Line definition in UV coordinates (0-1) on the plane rectangle
        # Perpendicular to previous horizontal line: keep center (0.5, 0.5)
        # and swap variation from U to V.
        self.declare_parameter('line_u_start', 0.5)
        self.declare_parameter('line_v_start', 0.3)
        self.declare_parameter('line_u_end', 0.5)
        self.declare_parameter('line_v_end', 0.7)

        self._load_params()
        self._load_plane_json()

        # ---- joint state subscriber ----
        self._last_joint_state: Optional[JointState] = None
        self.joint_state_sub = self.create_subscription(
            JointState, '/isaac_joint_states',
            self._joint_state_cb, 10)

        # ---- joint command publisher ----
        self.joint_cmd_pub = self.create_publisher(
            JointState, '/isaac_joint_commands', 10)

        # ---- home configuration (same as planar_servo_controller) ----
        self._joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
        ]
        self._home_positions = [
            -0.7854, -0.4363, -2.5307, -0.1745, 0.0, 0.0,
        ]
        self._home_reached_time = None
        self._home_hold_sec = 2.0

        # ---- safe retract parameters ----
        self._retract_height = 0.15   # metres above current EE to lift before homing
        self._retract_waypoints: List[Tuple[np.ndarray, list]] = []
        self._retract_idx = 0
        self._retract_home_path: List[np.ndarray] = []  # joint-space interp to home
        self._retract_home_idx = 0
        self._retract_initialized = False

        # ---- RRT execution state ----
        self._rrt_path: List[np.ndarray] = []
        self._rrt_idx = 0

        # ---- Cartesian drawing state ----
        self._cart_waypoints: List[Tuple[np.ndarray, list]] = []
        self._cart_idx = 0
        self._last_q: Optional[np.ndarray] = None   # seed for IK

        # ---- descent / ascent state ----
        self._descent_target: Optional[np.ndarray] = None
        self._ascent_target: Optional[np.ndarray] = None

        # ---- phase ----
        self.phase = Phase.RETRACT_UP

        # ---- control timer ----
        dt = 1.0 / self.execution_hz
        self.timer = self.create_timer(dt, self._control_loop)

        self.get_logger().info(
            f'Cartesian draw controller started — '
            f'{len(self.draw_positions)} waypoints ({self.traj_key}), '
            f'approach_height={self.approach_height:.3f} m')

    # ------------------------------------------------------------------
    # Parameter loading
    # ------------------------------------------------------------------
    def _load_params(self):
        g = self.get_parameter
        self.approach_height  = g('approach_height').value
        self.v_max            = g('max_linear_vel').value
        self.a_max            = g('max_linear_accel').value
        self.ik_damping       = g('ik_damping').value
        self.execution_hz     = g('execution_hz').value
        self.wp_per_tick      = g('waypoints_per_tick').value
        self.do_loop          = g('loop_trajectory').value
        self.traj_key         = g('trajectory_key').value
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

        # Diagnostic counters
        self._ik_fail_count  = 0
        self._ik_jump_count  = 0
        self._ik_cfg_count   = 0   # rejected for configuration constraint
        self._ik_total_count = 0

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
        self.plane_x = np.array(plane['x_axis'], dtype=float)
        self.plane_y = np.array(plane['y_axis'], dtype=float)
        self.plane_n = np.array(plane['normal'], dtype=float)

        # ---- plane rectangle bounds (for UV → world conversion) ----
        corners = [np.array(c, dtype=float) for c in data['rectangle_corners']]
        # width_vec = corner[1] - corner[0], height_vec = corner[3] - corner[0]
        self.rect_origin = corners[0]
        self.rect_width_vec = corners[1] - corners[0]
        self.rect_height_vec = corners[3] - corners[0]

        # ---- build trajectory ----
        if self.traj_key == 'line':
            # Generate line from UV parameters
            p_start = (self.rect_origin
                       + self.line_u_start * self.rect_width_vec
                       + self.line_v_start * self.rect_height_vec)
            p_end = (self.rect_origin
                     + self.line_u_end * self.rect_width_vec
                     + self.line_v_end * self.rect_height_vec)
            self.draw_positions = [p_start, p_end]
            # Use orientation from square_trajectory (first entry)
            traj = data.get('square_trajectory', [])
            if traj:
                self.target_quat = list(traj[0]['orientation_xyzw'])
            else:
                raise RuntimeError('Need square_trajectory for orientation')
            line_len = float(np.linalg.norm(p_end - p_start))
            self.get_logger().info(
                f'Line trajectory: UV ({self.line_u_start},{self.line_v_start})→'
                f'({self.line_u_end},{self.line_v_end})  '
                f'length={line_len:.3f}m  '
                f'start=[{p_start[0]:.3f},{p_start[1]:.3f},{p_start[2]:.3f}]  '
                f'end=[{p_end[0]:.3f},{p_end[1]:.3f},{p_end[2]:.3f}]')
        else:
            # Load from JSON key
            traj = data.get(self.traj_key) or data.get('square_trajectory', [])
            if not traj:
                raise RuntimeError(f'No trajectory found under key "{self.traj_key}"')
            self.draw_positions: List[np.ndarray] = []
            for wp in traj:
                self.draw_positions.append(np.array(wp['position'], dtype=float))
            self.target_quat = list(traj[0]['orientation_xyzw'])

        self.get_logger().info(
            f'Loaded {len(self.draw_positions)} waypoints from "{self.traj_key}"')

    # ------------------------------------------------------------------
    # Joint state callback
    # ------------------------------------------------------------------
    def _joint_state_cb(self, msg: JointState):
        self._last_joint_state = msg

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

    # ------------------------------------------------------------------
    # Publish joint command helper
    # ------------------------------------------------------------------
    def _pub_joints(self, q: np.ndarray):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._joint_names
        msg.position = q.tolist()
        self.joint_cmd_pub.publish(msg)

    # ------------------------------------------------------------------
    # Build a 4×4 target pose from position + quaternion
    # ------------------------------------------------------------------
    @staticmethod
    def _pose44(pos: np.ndarray, quat_xyzw: list) -> np.ndarray:
        T = np.eye(4)
        T[:3, :3] = quat_to_rotmat(quat_xyzw)
        T[:3, 3] = pos
        return T

    # ------------------------------------------------------------------
    # RETRACT_UP — lift EE straight up before any homing
    # ------------------------------------------------------------------
    def _retract_up(self):
        """Lift the end-effector vertically (world Z+) to a safe clearance
        height using Cartesian IK interpolation.  This prevents the EE from
        dipping into the workpiece when transitioning to home."""
        # One-time initialisation: compute the lift trajectory
        if not self._retract_initialized:
            q_current = self._get_ordered_joints()
            if q_current is None:
                self.get_logger().warn('RETRACT_UP — waiting for joint states…',
                                       throttle_duration_sec=2.0)
                return

            from ur5e_rrt_planner import ur5e_fk
            T_now = ur5e_fk(q_current)
            current_pos = T_now[:3, 3]
            current_quat = rotmat_to_quat(T_now[:3, :3])

            # Lift target: straight up in world Z
            lift_pos = current_pos.copy()
            lift_pos[2] += self._retract_height

            self.get_logger().info(
                f'RETRACT_UP — current EE z={current_pos[2]:.3f}, '
                f'lifting to z={lift_pos[2]:.3f} '
                f'(+{self._retract_height:.3f} m)')

            self._retract_waypoints = _interpolate_cartesian_smooth(
                positions=[current_pos, lift_pos],
                orientation_xyzw=current_quat,
                v_max=self.v_max,
                a_max=self.a_max,
                dt=(1.0 / self.execution_hz),
            )
            self._retract_idx = 0
            self._last_q = q_current.copy()
            self._retract_initialized = True

        # Play through the lift waypoints
        if self._retract_idx >= len(self._retract_waypoints):
            self.get_logger().info('RETRACT_UP complete → RETRACT_HOME')
            self.phase = Phase.RETRACT_HOME
            return

        pos, quat = self._retract_waypoints[self._retract_idx]
        T = self._pose44(pos, quat)
        q_sol = self._ik(T, self._last_q)

        if q_sol is None:
            self.get_logger().warn(
                f'RETRACT_UP IK failed at step {self._retract_idx}'
                f'/{len(self._retract_waypoints)} — skipping')
            self._retract_idx += 1
            return

        self._pub_joints(q_sol)
        self._last_q = q_sol.copy()
        self._retract_idx += 1

        total = len(self._retract_waypoints)
        if self._retract_idx % 10 == 0 or self._retract_idx >= total:
            fk_pos = self._fk_position(q_sol)
            self.get_logger().info(
                f'RETRACT_UP  step {self._retract_idx}/{total}  '
                f'z={fk_pos[2]:.3f}',
                throttle_duration_sec=0.5)

    # ------------------------------------------------------------------
    # RETRACT_HOME — joint-space interpolation to home
    # ------------------------------------------------------------------
    def _retract_home(self):
        """Smoothly interpolate in joint-space from the current (retracted)
        position to the home configuration.  Uses a simple linear interpolation
        with many small steps so the robot moves slowly and predictably."""
        if not self._retract_home_path:
            q_current = self._last_q if self._last_q is not None else self._get_ordered_joints()
            if q_current is None:
                self.get_logger().warn('RETRACT_HOME — waiting for joint states…',
                                       throttle_duration_sec=2.0)
                return

            q_home = np.array(self._home_positions, dtype=float)
            # Number of interpolation steps — ~2 seconds at execution_hz
            n_steps = max(int(2.0 * self.execution_hz), 20)
            path = []
            for i in range(n_steps + 1):
                alpha = i / n_steps
                # Smooth ease-in / ease-out (cosine interpolation)
                alpha_smooth = 0.5 * (1.0 - math.cos(math.pi * alpha))
                q_interp = (1.0 - alpha_smooth) * q_current + alpha_smooth * q_home
                path.append(q_interp)

            self._retract_home_path = path
            self._retract_home_idx = 0
            self.get_logger().info(
                f'RETRACT_HOME — interpolating to home in {n_steps} steps '
                f'(~{n_steps / self.execution_hz:.1f} s)')

        if self._retract_home_idx >= len(self._retract_home_path):
            self.get_logger().info('RETRACT_HOME complete → HOME_HOLD')
            self.phase = Phase.HOME_HOLD
            return

        q_cmd = self._retract_home_path[self._retract_home_idx]
        self._pub_joints(q_cmd)
        self._last_q = q_cmd.copy()
        self._retract_home_idx += 1

        total = len(self._retract_home_path)
        if self._retract_home_idx % 20 == 0 or self._retract_home_idx >= total:
            pct = 100.0 * self._retract_home_idx / max(total - 1, 1)
            self.get_logger().info(
                f'RETRACT_HOME  {self._retract_home_idx}/{total} ({pct:.0f}%)',
                throttle_duration_sec=0.5)

    # ------------------------------------------------------------------
    # HOME_HOLD — hold at home, then proceed
    # ------------------------------------------------------------------
    def _home_hold(self):
        """Hold at the home position for _home_hold_sec before planning."""
        self._pub_joints(np.array(self._home_positions))

        if self._home_reached_time is None:
            self._home_reached_time = self.get_clock().now()
            self.get_logger().info('HOME_HOLD — holding at home position…')

        elapsed = (self.get_clock().now() - self._home_reached_time).nanoseconds * 1e-9
        self.get_logger().info(
            f'HOME_HOLD — ({elapsed:.1f}/{self._home_hold_sec:.1f} s)',
            throttle_duration_sec=1.0)

        if elapsed >= self._home_hold_sec:
            self.get_logger().info('HOME_HOLD complete → PLANNING')
            self.phase = Phase.PLANNING

    # ------------------------------------------------------------------
    # PLANNING  (RRT to approach pose above first corner)
    # ------------------------------------------------------------------
    def _plan(self):
        if self._last_joint_state is None:
            self._pub_joints(np.array(self._home_positions))
            self.get_logger().warn('Waiting for joint states…',
                                   throttle_duration_sec=2.0)
            return

        q_current = self._get_ordered_joints()
        if q_current is None:
            return

        from ur5e_rrt_planner import (ur5e_fk, rrt_connect,
                                        smooth_path, bezier_smooth_path)

        # Approach pose: above the first vertex, offset along plane normal
        approach_pos = self.draw_positions[0] - self.approach_height * self.plane_n
        T_approach = self._pose44(approach_pos, self.target_quat)

        self.get_logger().info(
            f'PLANNING → approach pos = '
            f'[{approach_pos[0]:.3f}, {approach_pos[1]:.3f}, {approach_pos[2]:.3f}]')

        # Step 1: Constrained IK — find elbow-up goal configuration
        q_goal = self._constrained_ik_for_pose(T_approach)
        if q_goal is None:
            self.get_logger().error(
                'PLANNING FAILED — no elbow-up IK solution.  '
                'Try widening shoulder_lift_max / elbow_max or changing '
                'the plane/approach_height.')
            return

        # Step 2: RRT-Connect in joint space
        raw_path = rrt_connect(q_current, q_goal,
                               step_size=0.2, max_iter=10000)
        if raw_path is None:
            self.get_logger().error('RRT planning FAILED — retrying next tick…')
            return

        # Step 3: Smooth + Bezier spline interpolation
        smoothed = smooth_path(raw_path, max_attempts=200)
        path = bezier_smooth_path(smoothed, max_step=0.02)

        self._rrt_path = path
        self._rrt_idx = 0

        # Verify goal FK and report
        T_goal = ur5e_fk(path[-1])
        goal_pos = T_goal[:3, 3]
        goal_z = T_goal[:3, 2]  # z-axis of tool
        self.get_logger().info(
            f'PLANNING complete — {len(path)} joint waypoints → EXECUTING')
        self.get_logger().info(
            f'  Goal FK pos=[{goal_pos[0]:.4f}, {goal_pos[1]:.4f}, '
            f'{goal_pos[2]:.4f}]  tool_z=[{goal_z[0]:.3f}, '
            f'{goal_z[1]:.3f}, {goal_z[2]:.3f}]')
        self.get_logger().info(
            f'  Goal joints={np.round(path[-1], 3).tolist()}  '
            f'shoulder_lift={path[-1][1]:.3f}  elbow={path[-1][2]:.3f}')
        self.phase = Phase.EXECUTING

    # ------------------------------------------------------------------
    # EXECUTING  (play back RRT joint-space path)
    # ------------------------------------------------------------------
    def _execute(self):
        if self._rrt_idx >= len(self._rrt_path):
            self.get_logger().info('RRT path complete → DESCENDING')
            self._last_q = self._rrt_path[-1].copy()

            # Prepare descent: from approach height to on-plane
            approach_pos = self.draw_positions[0] - self.approach_height * self.plane_n
            on_plane_pos = self.draw_positions[0].copy()
            self._descent_waypoints = _interpolate_cartesian_smooth(
                positions=[approach_pos, on_plane_pos],
                orientation_xyzw=self.target_quat,
                v_max=self.v_max,
                a_max=self.a_max,
                dt=(1.0 / self.execution_hz),
            )
            self._descent_idx = 0
            self.phase = Phase.DESCENDING
            return

        q_cmd = self._rrt_path[self._rrt_idx]
        self._pub_joints(q_cmd)

        if self._rrt_idx % 10 == 0 or self._rrt_idx == len(self._rrt_path) - 1:
            pct = 100.0 * self._rrt_idx / max(len(self._rrt_path) - 1, 1)
            fk_pos = self._fk_position(q_cmd)
            self.get_logger().info(
                f'EXECUTING  step {self._rrt_idx}/{len(self._rrt_path)-1}  '
                f'({pct:.0f}%)  fk_pos=[{fk_pos[0]:.3f}, '
                f'{fk_pos[1]:.3f}, {fk_pos[2]:.3f}]')
        self._rrt_idx += 1

    # ------------------------------------------------------------------
    # Plane-frame helpers (mirrors planar_servo_controller)
    # ------------------------------------------------------------------
    def _to_plane(self, pos_base: np.ndarray) -> Tuple[float, float, float]:
        """Project a base-frame position onto the plane → (px, py, pz_off)."""
        rel = pos_base - self.plane_origin
        return (float(np.dot(rel, self.plane_x)),
                float(np.dot(rel, self.plane_y)),
                float(np.dot(rel, self.plane_n)))

    # ------------------------------------------------------------------
    # FK + diagnostics helper
    # ------------------------------------------------------------------
    def _fk_diagnostics(self, q: np.ndarray, target_pos: np.ndarray,
                        phase_tag: str, wp_idx: int = 0,
                        total_wp: int = 0) -> None:
        """Log rich diagnostics: FK pos, pos error, plane coords, orientation
        error, manipulability, joint values — similar to servo controller."""
        from ur5e_rrt_planner import ur5e_fk, ur5e_jacobian

        T_fk = ur5e_fk(q)
        fk_pos = T_fk[:3, 3]
        fk_R = T_fk[:3, :3]

        # Position error
        pos_err = target_pos - fk_pos
        pos_err_mag = float(np.linalg.norm(pos_err))

        # Plane-frame coordinates of FK position
        px, py, pz_off = self._to_plane(fk_pos)

        # Orientation error (angle between FK z-axis and desired z-axis)
        R_target = quat_to_rotmat(self.target_quat)
        z_fk = fk_R[:, 2]
        z_target = R_target[:, 2]
        cos_ang = float(np.clip(np.dot(z_fk, z_target), -1.0, 1.0))
        orient_err_deg = math.degrees(math.acos(cos_ang))

        # Manipulability: sqrt(det(J·Jᵀ))
        J = ur5e_jacobian(q)
        manip = float(math.sqrt(max(np.linalg.det(J @ J.T), 0.0)))

        # Condition number (high = near singularity)
        sv = np.linalg.svd(J, compute_uv=False)
        cond = float(sv[0] / max(sv[-1], 1e-10))

        self.get_logger().info(
            f'{phase_tag}  wp={wp_idx}/{total_wp}  '
            f'pos_err={pos_err_mag:.4f}m  z_off={pz_off:.4f}m  '
            f'plane_xy=({px:.3f},{py:.3f})  '
            f'orient_err={orient_err_deg:.1f}°  '
            f'manip={manip:.4f}  cond={cond:.1f}',
            throttle_duration_sec=0.5)

    def _fk_position(self, q: np.ndarray) -> np.ndarray:
        """Return FK tool0 position for given joint angles."""
        from ur5e_rrt_planner import ur5e_fk
        return ur5e_fk(q)[:3, 3]

    # ------------------------------------------------------------------
    # Configuration constraint check
    # ------------------------------------------------------------------
    def _config_ok(self, q: np.ndarray) -> bool:
        """Check if the joint configuration satisfies elbow-up constraints.
        Returns True if OK, False if the configuration would collide."""
        shoulder_lift = q[1]
        elbow = q[2]
        if shoulder_lift > self.shoulder_lift_max or shoulder_lift < self.shoulder_lift_min:
            return False
        if elbow > self.elbow_max or elbow < self.elbow_min:
            return False
        return True

    # ------------------------------------------------------------------
    # IK helper with joint-jump rejection + configuration constraints
    # ------------------------------------------------------------------
    def _ik(self, T_target: np.ndarray, q_seed: np.ndarray) -> Optional[np.ndarray]:
        from ur5e_rrt_planner import ik_solve

        self._ik_total_count += 1
        q_sol = ik_solve(T_target, q_seed, max_iter=300,
                         pos_tol=5e-4, orient_tol=1e-3,
                         damping=self.ik_damping)

        if q_sol is None:
            self._ik_fail_count += 1
            return None

        # Configuration constraint: enforce elbow-up
        if not self._config_ok(q_sol):
            self._ik_cfg_count += 1
            self.get_logger().warn(
                f'IK CONFIG REJECTED  shoulder_lift={q_sol[1]:.3f} '
                f'(limit=[{self.shoulder_lift_min:.2f},{self.shoulder_lift_max:.2f}])  '
                f'elbow={q_sol[2]:.3f} '
                f'(limit=[{self.elbow_min:.2f},{self.elbow_max:.2f}])  '
                f'[cfg_reject={self._ik_cfg_count}/{self._ik_total_count}]',
                throttle_duration_sec=1.0)
            return None

        # Joint-jump rejection: if any joint moves more than max_joint_step
        # from the seed, the IK likely flipped configuration → collision risk
        joint_delta = np.abs(q_sol - q_seed)
        max_delta = float(np.max(joint_delta))
        worst_joint = int(np.argmax(joint_delta))

        if max_delta > self.max_joint_step:
            self._ik_jump_count += 1
            self.get_logger().warn(
                f'IK JUMP REJECTED  Δ={max_delta:.3f} rad on joint '
                f'{self._joint_names[worst_joint]} '
                f'(limit={self.max_joint_step:.3f})  '
                f'[fails={self._ik_fail_count}, jumps={self._ik_jump_count} '
                f'/ {self._ik_total_count} total]')
            return None

        return q_sol

    # ------------------------------------------------------------------
    # Constrained multi-seed IK for planning (finds elbow-up solutions)
    # ------------------------------------------------------------------
    def _constrained_ik_for_pose(self, T_target: np.ndarray) -> Optional[np.ndarray]:
        """Solve IK with many seeds, returning best elbow-up solution.
        Unlike _ik(), this does NOT apply the max_joint_step check
        (since there is no incremental seed to compare against)."""
        from ur5e_rrt_planner import ik_solve
        import random

        home = np.array(self._home_positions, dtype=float)
        candidates = []

        # Build a pool of seeds biased toward elbow-up configurations
        seeds = [home.copy()]
        # Perturbations of home (keep shoulder_lift negative, elbow negative)
        for _ in range(self.ik_num_seeds):
            s = home.copy()
            s[0] += random.uniform(-1.5, 1.5)    # shoulder_pan: wide range
            s[1] += random.uniform(-1.0, 0.3)    # shoulder_lift: bias negative
            s[2] += random.uniform(-0.5, 0.5)    # elbow: perturbation
            s[3] += random.uniform(-1.0, 1.0)    # wrist_1
            s[4] += random.uniform(-1.0, 1.0)    # wrist_2
            s[5] += random.uniform(-1.0, 1.0)    # wrist_3
            seeds.append(s)

        for seed in seeds:
            q_sol = ik_solve(T_target, seed, max_iter=300,
                             pos_tol=5e-4, orient_tol=1e-3,
                             damping=self.ik_damping)
            if q_sol is not None and self._config_ok(q_sol):
                # Score: prefer solutions closest to home
                dist_to_home = float(np.linalg.norm(q_sol - home))
                candidates.append((dist_to_home, q_sol))

        if not candidates:
            self.get_logger().error(
                f'Constrained IK failed: {len(seeds)} seeds tried, none '
                f'satisfy elbow-up constraints '
                f'[shoulder_lift≤{self.shoulder_lift_max:.2f}, '
                f'elbow≤{self.elbow_max:.2f}]')
            return None

        # Pick the solution closest to home (safest configuration)
        candidates.sort(key=lambda x: x[0])
        best_q = candidates[0][1]
        self.get_logger().info(
            f'Constrained IK: {len(candidates)}/{len(seeds)} valid  '
            f'best shoulder_lift={best_q[1]:.3f} elbow={best_q[2]:.3f}  '
            f'joints={np.round(best_q, 3).tolist()}')
        return best_q

    # ------------------------------------------------------------------
    # DESCENDING — Cartesian straight-line from approach to on-plane
    # ------------------------------------------------------------------
    def _descend(self):
        if self._descent_idx >= len(self._descent_waypoints):
            self.get_logger().info('Descent complete → DRAWING')
            # Build the full Cartesian drawing path
            self._cart_waypoints = _interpolate_cartesian_smooth(
                positions=self.draw_positions,
                orientation_xyzw=self.target_quat,
                v_max=self.v_max,
                a_max=self.a_max,
                dt=(1.0 / self.execution_hz),
            )
            self._cart_idx = 0
            self.phase = Phase.DRAWING
            return

        pos, quat = self._descent_waypoints[self._descent_idx]
        T = self._pose44(pos, quat)
        q_sol = self._ik(T, self._last_q)

        if q_sol is None:
            self.get_logger().warn(
                f'DESCENDING IK failed at step {self._descent_idx}'
                f'/{len(self._descent_waypoints)} — skipping  '
                f'target=[{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]')
            self._descent_idx += 1
            return

        self._pub_joints(q_sol)
        self._last_q = q_sol.copy()
        self._descent_idx += 1

        total = len(self._descent_waypoints)
        if self._descent_idx % 5 == 0 or self._descent_idx >= total:
            self._fk_diagnostics(q_sol, pos, 'DESCEND',
                                 self._descent_idx, total)

    # ------------------------------------------------------------------
    # DRAWING — follow dense Cartesian waypoints on the plane
    # ------------------------------------------------------------------
    def _draw(self):
        if self._cart_idx >= len(self._cart_waypoints):
            if self.do_loop:
                self._cart_idx = 0
                self.get_logger().info('Looping trajectory…')
                return
            else:
                self.get_logger().info(
                    f'Drawing complete → ASCENDING  '
                    f'[IK stats: {self._ik_fail_count} fails, '
                    f'{self._ik_jump_count} jumps / '
                    f'{self._ik_total_count} total]')
                # Prepare ascent: from last on-plane pos to approach height
                last_pos = self._cart_waypoints[-1][0]
                ascent_pos = last_pos - self.approach_height * self.plane_n
                self._ascent_waypoints = _interpolate_cartesian_smooth(
                    positions=[last_pos, ascent_pos],
                    orientation_xyzw=self.target_quat,
                    v_max=self.v_max,
                    a_max=self.a_max,
                    dt=(1.0 / self.execution_hz),
                )
                self._ascent_idx = 0
                self.phase = Phase.ASCENDING
                return

        # Advance by wp_per_tick waypoints per timer tick
        for _ in range(self.wp_per_tick):
            if self._cart_idx >= len(self._cart_waypoints):
                break

            pos, quat = self._cart_waypoints[self._cart_idx]
            T = self._pose44(pos, quat)
            q_sol = self._ik(T, self._last_q)

            if q_sol is None:
                # Report FK of last known config vs the failed target
                if self._last_q is not None:
                    fk_pos = self._fk_position(self._last_q)
                    px, py, pz = self._to_plane(fk_pos)
                    self.get_logger().warn(
                        f'DRAWING IK FAIL  wp={self._cart_idx}/{len(self._cart_waypoints)}  '
                        f'target=[{pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f}]  '
                        f'last_fk=[{fk_pos[0]:.4f},{fk_pos[1]:.4f},{fk_pos[2]:.4f}]  '
                        f'plane_xy=({px:.3f},{py:.3f})  z_off={pz:.4f}m  '
                        f'[fails={self._ik_fail_count}/{self._ik_total_count}]')
                else:
                    self.get_logger().warn(
                        f'DRAWING IK FAIL  wp={self._cart_idx}/{len(self._cart_waypoints)}')
                self._cart_idx += 1
                continue

            self._pub_joints(q_sol)
            self._last_q = q_sol.copy()
            self._cart_idx += 1

        # Rich diagnostics every 10 waypoints
        total = len(self._cart_waypoints)
        if self._cart_idx > 0 and (self._cart_idx % 10 == 0 or self._cart_idx >= total):
            pos_now, _ = self._cart_waypoints[min(self._cart_idx - 1, total - 1)]
            if self._last_q is not None:
                self._fk_diagnostics(self._last_q, pos_now, 'DRAWING',
                                     self._cart_idx, total)

                # Also log actual vs commanded joint comparison
                q_actual = self._get_ordered_joints()
                if q_actual is not None:
                    joint_err = np.abs(q_actual - self._last_q)
                    max_jerr = float(np.max(joint_err))
                    worst_j = int(np.argmax(joint_err))
                    self.get_logger().info(
                        f'DRAWING  joints_cmd={np.round(self._last_q, 3).tolist()}  '
                        f'max_joint_err={max_jerr:.4f} rad '
                        f'({self._joint_names[worst_j]})',
                        throttle_duration_sec=0.5)

    # ------------------------------------------------------------------
    # ASCENDING — lift off the plane
    # ------------------------------------------------------------------
    def _ascend(self):
        if self._ascent_idx >= len(self._ascent_waypoints):
            self.get_logger().info(
                f'Ascent complete → DONE  '
                f'[IK stats: {self._ik_fail_count} fails, '
                f'{self._ik_jump_count} jumps / '
                f'{self._ik_total_count} total]')
            self.phase = Phase.DONE
            return

        pos, quat = self._ascent_waypoints[self._ascent_idx]
        T = self._pose44(pos, quat)
        q_sol = self._ik(T, self._last_q)

        if q_sol is None:
            self.get_logger().warn(
                f'ASCENDING IK failed at step {self._ascent_idx}'
                f'/{len(self._ascent_waypoints)}  '
                f'target=[{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]')
            self._ascent_idx += 1
            return

        self._pub_joints(q_sol)
        self._last_q = q_sol.copy()
        self._ascent_idx += 1

        total = len(self._ascent_waypoints)
        if self._ascent_idx % 5 == 0 or self._ascent_idx >= total:
            self._fk_diagnostics(q_sol, pos, 'ASCEND',
                                 self._ascent_idx, total)

    # ------------------------------------------------------------------
    # Main control loop
    # ------------------------------------------------------------------
    def _control_loop(self):
        if self.phase == Phase.RETRACT_UP:
            self._retract_up()
        elif self.phase == Phase.RETRACT_HOME:
            self._retract_home()
        elif self.phase == Phase.HOME_HOLD:
            self._home_hold()
        elif self.phase == Phase.PLANNING:
            self._plan()
        elif self.phase == Phase.EXECUTING:
            self._execute()
        elif self.phase == Phase.DESCENDING:
            self._descend()
        elif self.phase == Phase.DRAWING:
            self._draw()
        elif self.phase == Phase.ASCENDING:
            self._ascend()
        elif self.phase == Phase.DONE:
            # Keep publishing last joint command to hold position
            if self._last_q is not None:
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
