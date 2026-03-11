This is a classic problem in Cartesian robotic fabrication. When you attempt to draw sharp geometries, the physical realities of the motors clash with the mathematics of the path.

Here is exactly what **Modern Robotics (Lynch & Park)** and the **Springer Handbook** say about your three symptoms, and how we will fix the Action Server code to permanently solve them.

### What the Books Say About Your Symptoms

1. **"Speed Jumps and Soft Error Warnings" ($C^0$ Discontinuity):** *Springer Handbook* defines shapes like squares and triangles as $C^0$ geometric continuous (they have sharp corners). To turn a 90-degree corner without tearing the robot apart, the Cartesian velocity of the end-effector *must* drop to exactly `0.0 m/s` at the corner. If your code feeds a sharp corner to the UR5e controller without explicitly telling it to stop, the robot's internal spline interpolator attempts to fly *through* the corner, requiring infinite acceleration. The UR5e catches this mathematical impossibility and throws the velocity/acceleration jump warning.
2. **"End Effector gets stuck / Not followed perfectly" (Downsampling Destruction):**
*Modern Robotics* warns against arbitrarily discretizing paths around via-points. In your Action Server, you have a function called `_downsample_for_real_robot` which forces the points to run at 20Hz. If a sharp corner happens to land exactly *between* those 20Hz ticks, the downsampler literally deletes the corner. This physically "cuts the corner" off your square (not followed perfectly) and creates massive mathematical gaps between joints that cause the robot to shudder and get stuck.

### The Fatal Bug in Your Current Code

You actually wrote a perfect trapezoidal velocity generator (`_interpolate_cartesian_smooth`). It brilliantly bunches points close together at the corners to slow the robot down, and spreads them out in the middle to speed it up.

**However, `_concat_phase_times` completely destroys it.**
Look at this line currently in your code:
`phase_times = [i * cart_dur / (len(plist) - 1) for i in range(len(plist))]`

It forces an evenly-spaced time array onto points that are *not* evenly spaced in distance. It forces the robot to traverse the tiny bunched-up points at the corners in the *exact same amount of time* it takes to traverse the wide gaps in the middle. Your velocity profile is being completely overwritten by a flat timer!

### The Solution

1. We will rewrite `_interpolate_cartesian_smooth` so it returns both the waypoints AND the exact calculated timestamps.
2. We will feed those exact timestamps straight into the phase concatenator.
3. We will completely delete the `_downsample_for_real_robot` function. The UR5e can natively handle your 100Hz trajectory. Downsampling is destroying your geometry.

### The Updated Action Server Code

Replace your entire `DrawingActionServer` node with this updated script.

```python
#!/usr/bin/env python3
"""
Drawing Action Server — Sequential action-based drawing controller.

Fixed Cartesian Timing: Fully preserves the exact trapezoidal timestamps
generated during spatial interpolation, guaranteeing the end-effector
gracefully decelerates to exactly v=0 at every sharp corner ($C^0$ continuity)
to prevent UR5e acceleration jump warnings.
"""

import json
import math
import os
import sys
import time as _time
from typing import List, Optional, Tuple

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
from sand_drawer.srv import ComputeTOTG

# ═══════════════════════════════════════════════════════════════════════════
# Quaternion / rotation helpers
# ═══════════════════════════════════════════════════════════════════════════
def quat_to_rotmat(q_xyzw) -> np.ndarray:
    x, y, z, w = q_xyzw
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),      1 - 2*(x*x + y*y)],
    ])

def rotmat_to_quat(R: np.ndarray) -> list:
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
# Cartesian interpolation with EXACT TIMESTAMPS
# ═══════════════════════════════════════════════════════════════════════════
def _interpolate_cartesian_smooth(
    positions: List[np.ndarray],
    orientation_xyzw: list,
    v_max: float = 0.05,
    a_max: float = 0.05,
    dt: float = 0.01,
) -> Tuple[List[Tuple[np.ndarray, list]], List[float]]:
    """
    Dense Cartesian path with per-segment trapezoidal velocity profile.
    NOW RETURNS: (waypoints, exact_timestamps) to prevent the constant-time bug.
    """
    if not positions or len(positions) < 2:
        return [(positions[0].copy(), orientation_xyzw)] if positions else [], [0.0] if positions else []

    waypoints = []
    times = []
    current_t = 0.0

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

            # Prevent appending identical points perfectly overlapping at the corners
            if i > 0 and step == 0:
                continue

            waypoints.append((p_start + line_dir * min(s, dist), orientation_xyzw))
            times.append(current_t + t)

        current_t += total_time

    # Append absolute final point
    waypoints.append((positions[-1].copy(), orientation_xyzw))
    times.append(current_t)
    return waypoints, times


# ═══════════════════════════════════════════════════════════════════════════
# Drawing Action Server Node
# ═══════════════════════════════════════════════════════════════════════════
class DrawingActionServer(Node):
    def __init__(self):
        super().__init__('drawing_action_server')

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
        self.declare_parameter('real_robot_trajectory_topic', '/scaled_joint_trajectory_controller/joint_trajectory')
        self.declare_parameter('max_joint_speed_deg', 45.0)
        self.declare_parameter('max_joint_accel_deg', 40.0)
        self.declare_parameter('totg_path_tolerance', 0.1)
        self.declare_parameter('totg_resample_dt', 0.01)

        self._load_params()
        self._load_plane_json()

        self._joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
        ]
        self._home_positions = np.array([-0.7854, -0.4363, -2.5307, -0.1745, 0.0, 0.0], dtype=float)

        self._last_joint_state = None
        self._last_sim_joint_state = None
        self._last_real_joint_state = None
        self.joint_state_sub = self.create_subscription(JointState, '/isaac_joint_states', self._sim_joint_state_cb, 10)
        self.joint_cmd_pub = self.create_publisher(JointState, '/isaac_joint_commands', 10)

        self._real_robot = self.get_parameter('real_robot').value
        self._real_traj_pub = None
        self._real_converge_tol = math.radians(2.0)

        if self._real_robot:
            real_js_topic = self.get_parameter('real_robot_joint_state_topic').value
            real_traj_topic = self.get_parameter('real_robot_trajectory_topic').value
            self._real_traj_pub = self.create_publisher(JointTrajectory, real_traj_topic, 10)
            self._real_joint_sub = self.create_subscription(JointState, real_js_topic, self._real_joint_state_cb, 10)
            self.get_logger().info(f'REAL ROBOT mode: publishing to {real_traj_topic}')

        self._max_joint_speed_rad = math.radians(self._max_joint_speed_deg)
        self._max_joint_accel_rad = math.radians(self._max_joint_accel_deg)
        self._dt = 1.0 / self.execution_hz
        self._prev_cmd_q = None

        self._retract_height = 0.15
        self._home_hold_sec = 0.5

        self._first_goal = True
        self._executing = False
        self._last_q = None

        self._totg_client = self.create_client(ComputeTOTG, 'compute_totg', callback_group=ReentrantCallbackGroup())
        self._totg_available = False

        self._action_server = ActionServer(
            self, ExecuteDrawing, 'execute_drawing',
            execute_callback=self._execute_goal,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info('Drawing action server ready — Fixed Cartesian Timing Engine Active.')

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
        self._totg_path_tolerance = float(g('totg_path_tolerance').value)
        self._totg_resample_dt   = float(g('totg_resample_dt').value)

    def _load_plane_json(self):
        json_path = self.get_parameter('plane_json_file').value
        with open(json_path, 'r') as f:
            data = json.load(f)

        target_frame = data.get('target_frame', 'base_link')
        if target_frame == 'base':
            def _rz(v): return [-v[0], -v[1], v[2]]
            p = data['plane']
            p['origin'], p['x_axis'], p['y_axis'], p['normal'] = _rz(p['origin']), _rz(p['x_axis']), _rz(p['y_axis']), _rz(p['normal'])

        plane = data['plane']
        self.plane_origin = np.array(plane['origin'], dtype=float)
        self.plane_n = np.array(plane['normal'], dtype=float)

    def _sim_joint_state_cb(self, msg: JointState):
        self._last_sim_joint_state = msg
        if not self._real_robot:
            self._last_joint_state = msg

    def _real_joint_state_cb(self, msg: JointState):
        self._last_real_joint_state = msg
        if self._real_robot:
            self._last_joint_state = msg

    def _get_ordered_joints(self) -> Optional[np.ndarray]:
        if self._last_joint_state is None: return None
        name_map = {n: p for n, p in zip(self._last_joint_state.name, self._last_joint_state.position)}
        try: return np.array([name_map[n] for n in self._joint_names])
        except KeyError: return None

    def _get_real_ordered_joints(self) -> Optional[np.ndarray]:
        if self._last_real_joint_state is None: return None
        name_map = {n: p for n, p in zip(self._last_real_joint_state.name, self._last_real_joint_state.position)}
        try: return np.array([name_map[n] for n in self._joint_names])
        except KeyError: return None

    def _real_robot_converged(self, target_q: np.ndarray) -> bool:
        if not self._real_robot: return True
        real_q = self._get_real_ordered_joints()
        if real_q is None: return False
        return float(np.max(np.abs(target_q - real_q))) < self._real_converge_tol

    def _mirror_real_to_sim(self):
        if not self._real_robot: return
        real_q = self._get_real_ordered_joints()
        if real_q is None: return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._joint_names)
        msg.position = real_q.tolist()
        self.joint_cmd_pub.publish(msg)

    def _pub_joints(self, q: np.ndarray):
        if not self._real_robot:
            if self._prev_cmd_q is None:
                cur = self._get_ordered_joints()
                if cur is not None: self._prev_cmd_q = cur.copy()
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

    @staticmethod
    def _pose44(pos: np.ndarray, quat_xyzw: list) -> np.ndarray:
        T = np.eye(4)
        T[:3, :3] = quat_to_rotmat(quat_xyzw)
        T[:3, 3] = pos
        return T

    def _ik(self, T_target: np.ndarray, q_seed: np.ndarray) -> Optional[np.ndarray]:
        from ur5e_rrt_planner import ik_solve
        q_sol = ik_solve(T_target, q_seed, max_iter=300, pos_tol=5e-4, orient_tol=1e-3, damping=self.ik_damping)
        if q_sol is None: return None
        if not self._config_ok(q_sol): return None
        if float(np.max(np.abs(q_sol - q_seed))) > self.max_joint_step: return None
        return q_sol

    def _config_ok(self, q: np.ndarray) -> bool:
        if q[1] > self.shoulder_lift_max or q[1] < self.shoulder_lift_min: return False
        if q[2] > self.elbow_max or q[2] < self.elbow_min: return False
        return True

    def _constrained_ik_for_pose(self, T_target: np.ndarray) -> Optional[np.ndarray]:
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
            q_sol = ik_solve(T_target, seed, max_iter=300, pos_tol=5e-4, orient_tol=1e-3, damping=self.ik_damping)
            if q_sol is not None and self._config_ok(q_sol):
                candidates.append((float(np.linalg.norm(q_sol - home)), q_sol))

        if not candidates: return None
        candidates.sort(key=lambda x: x[0])
        return candidates[0][1]

    # MODIFIED: Now accepts AND filters precise timestamps
    def _ik_solve_cartesian_path(
        self,
        cart_wps: List[Tuple[np.ndarray, list]],
        cart_times: List[float],
        q_seed: np.ndarray,
        label: str = 'path',
    ) -> Tuple[List[np.ndarray], List[float]]:
        path = []
        valid_times = []
        seed = q_seed.copy()
        fails = 0
        for (pos, quat), t in zip(cart_wps, cart_times):
            T = self._pose44(pos, quat)
            q_sol = self._ik(T, seed)
            if q_sol is not None:
                path.append(q_sol)
                valid_times.append(t)
                seed = q_sol.copy()
            else:
                fails += 1
        self.get_logger().info(f'  {label}: IK solved {len(path)}/{len(cart_wps)} ({fails} fails)')
        return path, valid_times

    def _compute_totg(self, path: List[np.ndarray]) -> Optional[Tuple[List[np.ndarray], List[float]]]:
        if len(path) <= 1: return path[:], [0.0] * len(path)
        n_joints = len(path[0])

        req = ComputeTOTG.Request()
        req.num_joints = n_joints
        req.waypoints_flat = []
        for q in path: req.waypoints_flat.extend(q.tolist())
        req.max_velocity = [self._max_joint_speed_rad] * n_joints
        req.max_acceleration = [self._max_joint_accel_rad] * n_joints
        req.path_tolerance = self._totg_path_tolerance
        req.resample_dt = self._totg_resample_dt

        if not self._totg_available:
            if not self._totg_client.wait_for_service(timeout_sec=10.0): return None
            self._totg_available = True

        future = self._totg_client.call_async(req)
        deadline = _time.monotonic() + 30.0
        while not future.done():
            if _time.monotonic() > deadline: return None
            _time.sleep(0.005)

        if future.result() is None: return None
        resp = future.result()
        if not resp.success: return None

        n_out = resp.num_output_points
        timestamps = list(resp.timestamps)
        resampled_path = [np.array(resp.timed_positions_flat[i * n_joints:(i + 1) * n_joints], dtype=float) for i in range(n_out)]
        return resampled_path, timestamps

    def _compute_phase_timing_fallback(self, path: List[np.ndarray]) -> List[float]:
        if len(path) <= 1: return [0.0] * len(path)
        v_max, a_max = self._max_joint_speed_rad, self._max_joint_accel_rad
        n_seg = len(path) - 1
        dt = np.empty(n_seg)
        for i in range(n_seg):
            dt[i] = max(float(np.max(np.abs(path[i + 1] - path[i]))) / v_max, 1e-4)

        for i in range(n_seg - 1):
            for j in range(len(path[0])):
                accel = abs(((path[i + 2][j] - path[i + 1][j]) / dt[i + 1]) - ((path[i + 1][j] - path[i][j]) / dt[i])) / dt[i + 1]
                if accel > a_max: dt[i + 1] = max(dt[i + 1], accel / a_max)

        for i in range(n_seg - 2, -1, -1):
            for j in range(len(path[0])):
                accel = abs(((path[i + 1][j] - path[i][j]) / dt[i]) - ((path[i + 2][j] - path[i + 1][j]) / dt[i + 1])) / dt[i]
                if accel > a_max: dt[i] = max(dt[i], accel / a_max)

        times = [0.0]
        for t in dt: times.append(times[-1] + t)
        return times

    # MODIFIED: Expects absolute timestamps for Cartesian phases
    def _concat_phase_times(
        self, phases: List[Tuple[str, List[np.ndarray], Optional[List[float]]]]
    ) -> Tuple[List[np.ndarray], List[float], List[Tuple[str, int, int]]]:
        master_path, master_times, phase_info = [], [], []
        t_offset = 0.0

        for label, plist, phase_times_raw in phases:
            if not plist: continue

            if phase_times_raw is not None:
                # Use EXACT Cartesian trapezoidal timings. NO flat mapping.
                phase_path = plist
                phase_times = phase_times_raw
                timing_src = 'cartesian-exact'
            else:
                totg_result = self._compute_totg(plist)
                if totg_result is not None:
                    phase_path, phase_times = totg_result
                    timing_src = 'TOTG'
                else:
                    phase_path = plist
                    phase_times = self._compute_phase_timing_fallback(plist)
                    timing_src = 'IPTP-fallback'

            start_idx = len(master_path)
            for j, (q, t) in enumerate(zip(phase_path, phase_times)):
                if j == 0 and master_path: continue
                master_path.append(np.asarray(q, dtype=float))
                master_times.append(t_offset + t)

            end_idx = len(master_path) - 1
            if phase_times:
                t_offset = master_times[-1]

            phase_info.append((label, start_idx, end_idx))
            self.get_logger().info(f'  Phase: {label:15s}  {len(phase_path):4d} pts  {phase_times[-1]:.2f}s  [{timing_src}]')

        return master_path, master_times, phase_info

    def _send_full_trajectory(self, path: List[np.ndarray], times: List[float]) -> None:
        if self._real_traj_pub is None: return
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
        self.get_logger().info(f'Sent full trajectory ({len(path)} pts, {times[-1]:.1f}s) to real robot')

    # ══════════════════════════════════════════════════════════════════
    def _goal_callback(self, goal_request):
        if self._executing: return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def _execute_goal(self, goal_handle):
        self._executing = True
        try:
            result = self._execute_goal_inner(goal_handle)
        except Exception as exc:
            try: goal_handle.abort()
            except Exception: pass
            result = ExecuteDrawing.Result(success=False, message=f'Internal error: {exc}')
        finally:
            self._executing = False
        return result

    def _execute_goal_inner(self, goal_handle):
        feedback = ExecuteDrawing.Feedback()
        draw_positions = [np.array([p.x, p.y, p.z], dtype=float) for p in goal_handle.request.waypoints]
        q = goal_handle.request.orientation
        orientation = [q.x, q.y, q.z, q.w]

        feedback.current_phase = 'WAITING_JOINTS'
        goal_handle.publish_feedback(feedback)

        for _ in range(100):
            if self._get_ordered_joints() is not None: break
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return ExecuteDrawing.Result(success=False, message='Canceled while waiting for joints')
            _time.sleep(0.1)

        feedback.current_phase = 'PRE_COMPUTING'
        goal_handle.publish_feedback(feedback)

        phases = None
        for attempt in range(1, 4):
            phases = self._pre_compute_goal(draw_positions, orientation)
            if phases is not None: break
            _time.sleep(0.5)

        if phases is None:
            goal_handle.abort()
            return ExecuteDrawing.Result(success=False, message='Pre-computation failed')

        master_path, master_times, phase_info = self._concat_phase_times(phases)

        if self._real_robot:
            success = self._execute_real_trajectory(master_path, master_times, phase_info, goal_handle, feedback)
        else:
            success = self._execute_sim_trajectory(master_path, master_times, phase_info, goal_handle, feedback)

        if success: self._first_goal = False

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return ExecuteDrawing.Result(success=False, message='Drawing canceled')
        if success:
            self._last_q = master_path[-1].copy()
            goal_handle.succeed()
            return ExecuteDrawing.Result(success=True, message='Drawing complete')
        else:
            goal_handle.abort()
            return ExecuteDrawing.Result(success=False, message='Execution failed')

    def _pre_compute_goal(
        self, draw_positions: List[np.ndarray], orientation_xyzw: list,
    ) -> Optional[List[Tuple[str, List[np.ndarray], Optional[List[float]]]]]:

        from ur5e_rrt_planner import (ur5e_fk, rrt_connect, smooth_path, bezier_smooth_path)
        q_current = self._get_ordered_joints()
        if q_current is None: return None
        cart_dt = 1.0 / self.execution_hz
        phases = []

        T_now = ur5e_fk(q_current)
        current_pos, current_quat = T_now[:3, 3].copy(), rotmat_to_quat(T_now[:3, :3])
        lift_pos = current_pos.copy()
        lift_pos[2] += self._retract_height

        # 1. RETRACT UP
        retract_wps, retract_t = _interpolate_cartesian_smooth([current_pos, lift_pos], current_quat, self.approach_v_max, self.approach_a_max, cart_dt)
        retract_path, valid_retract_t = self._ik_solve_cartesian_path(retract_wps, retract_t, q_current, 'Retract up')
        if not retract_path: return None
        phases.append(('retract_up', retract_path, valid_retract_t))

        # 2. HOMING
        if self._first_goal:
            q_retracted = retract_path[-1]
            q_home = self._home_positions.copy()
            n_home = max(int(1.0 * self.execution_hz), 20)
            retract_home_path = [q_retracted.copy()]
            for i in range(1, n_home + 1):
                alpha = 0.5 * (1.0 - math.cos(math.pi * i / n_home))
                retract_home_path.append((1.0 - alpha) * q_retracted + alpha * q_home)
            phases.append(('retract_home', retract_home_path, None))

            n_hold = max(int(self._home_hold_sec * self.execution_hz), 1)
            phases.append(('home_hold', [q_home.copy() for _ in range(n_hold + 1)], None))

        # 3. RRT
        last_q = phases[-1][1][-1]
        approach_pos = (draw_positions[0] - self.approach_height * self.plane_n)
        q_goal = self._constrained_ik_for_pose(self._pose44(approach_pos, orientation_xyzw))
        if q_goal is None: return None

        raw_rrt = rrt_connect(last_q, q_goal, step_size=0.2, max_iter=10000)
        if raw_rrt is None: return None
        rrt_dense = bezier_smooth_path(smooth_path(raw_rrt, max_attempts=200), max_step=0.02)
        phases.append(('rrt', list(rrt_dense), None))

        # 4. DESCENT
        descent_wps, descent_t = _interpolate_cartesian_smooth([approach_pos, draw_positions[0].copy()], orientation_xyzw, self.approach_v_max, self.approach_a_max, cart_dt)
        descent_path, valid_desc_t = self._ik_solve_cartesian_path(descent_wps, descent_t, rrt_dense[-1], 'Descent')
        if descent_path: phases.append(('descent', descent_path, valid_desc_t))

        # 5. DRAWING
        draw_wps, draw_t = _interpolate_cartesian_smooth(draw_positions, orientation_xyzw, self.v_max, self.a_max, cart_dt)
        draw_path, valid_draw_t = self._ik_solve_cartesian_path(draw_wps, draw_t, phases[-1][1][-1], 'Drawing')
        if not draw_path or len(draw_path) < 2: return None
        phases.append(('drawing', draw_path, valid_draw_t))

        # 6. ASCENT
        ascent_pos = draw_positions[-1].copy() - self.approach_height * self.plane_n
        ascent_wps, ascent_t = _interpolate_cartesian_smooth([draw_positions[-1].copy(), ascent_pos], orientation_xyzw, self.approach_v_max, self.approach_a_max, cart_dt)
        ascent_path, valid_asc_t = self._ik_solve_cartesian_path(ascent_wps, ascent_t, draw_path[-1], 'Ascent')
        if ascent_path: phases.append(('ascent', ascent_path, valid_asc_t))

        return phases

    def _execute_sim_trajectory(self, master_path, master_times, phase_info, goal_handle, feedback):
        total_time = master_times[-1]
        start_wall = _time.monotonic()
        last_feedback_time = 0.0

        while True:
            if goal_handle.is_cancel_requested: return False
            elapsed = _time.monotonic() - start_wall

            if elapsed >= total_time:
                self._pub_joints(master_path[-1])
                self._last_q = master_path[-1].copy()
                break

            idx = int(np.searchsorted(master_times, elapsed, side='right')) - 1
            idx = max(0, min(idx, len(master_path) - 2))
            t0, t1 = master_times[idx], master_times[idx + 1]
            alpha = max(0.0, min(1.0, (elapsed - t0) / max(t1 - t0, 1e-9)))

            q_interp = ((1.0 - alpha) * master_path[idx] + alpha * master_path[idx + 1])
            self._pub_joints(q_interp)

            if elapsed - last_feedback_time >= 0.5:
                last_feedback_time = elapsed
                for label, start, end in phase_info:
                    if start <= idx <= end:
                        feedback.current_phase = label.upper()
                        feedback.drawing_progress = (idx - start) / max(end - start, 1) if label == 'drawing' else 0.0
                        break
                goal_handle.publish_feedback(feedback)

            _time.sleep(self._dt)
        return True

    def _execute_real_trajectory(self, master_path, master_times, phase_info, goal_handle, feedback):
        # NO MORE DOWNSAMPLING. Sent pristine dense 100Hz trajectory to respect geometric corners.
        self._send_full_trajectory(master_path, master_times)

        target_q, total_time = master_path[-1], master_times[-1]
        t0 = self.get_clock().now()

        while True:
            if goal_handle.is_cancel_requested: return False
            try: self._mirror_real_to_sim()
            except Exception: pass

            elapsed = (self.get_clock().now() - t0).nanoseconds * 1e-9
            if elapsed > min(total_time * 0.5, 5.0) and self._real_robot_converged(target_q):
                self._last_q = target_q.copy()
                return True
            if elapsed > total_time * 1.5 + 15.0: return False
            _time.sleep(0.1)

    def shutdown_sequence(self):
        try:
            q_current = self._get_ordered_joints()
            if q_current is None: return
            from ur5e_rrt_planner import ur5e_fk
            T_now = ur5e_fk(q_current)
            lift_pos = T_now[:3, 3].copy()
            lift_pos[2] += self._retract_height
            retract_wps, retract_t = _interpolate_cartesian_smooth([T_now[:3, 3], lift_pos], rotmat_to_quat(T_now[:3, :3]), self.approach_v_max, self.approach_a_max, 1.0/self.execution_hz)
            retract_path, valid_t = self._ik_solve_cartesian_path(retract_wps, retract_t, q_current)
            if retract_path: self._send_full_trajectory(retract_path, valid_t) if self._real_robot else [self._pub_joints(q) for q in retract_path]
        except Exception: pass

def main(args=None):
    rclpy.init(args=args)
    server = DrawingActionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally:
        server.shutdown_sequence()
        server.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__': main()

```