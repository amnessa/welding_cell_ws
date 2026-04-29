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

Current TCP/tool behavior
-------------------------
The action server uses active tool selection and dynamic TCP handling.

- Tool faces are selected around wrist-3 as:
    fork=0°, pointy=+90°, empty=-90°, spatula=180°, orthogonal=0°.
- Tool lengths from wrist axis:
    fork=0.13 m, pointy=0.15 m, spatula=0.13 m, empty=0.13 m,
    orthogonal=orthogonal_tool_length_m.
- For draw/approach/descent/ascent path solving, the server computes a
    dynamic wrist pose from each tip waypoint (per-waypoint yaw adaptation)
    before IK. This reduces over-constraint and improves reachability.

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

    Note: in action mode, the position waypoints are authoritative. The server
    derives tool-consistent wrist orientation internally for IK robustness.

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
from sand_drawer.srv import ComputeTOTG


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
) -> Tuple[List[Tuple[np.ndarray, list]], List[float]]:
    """Dense Cartesian path with per-segment trapezoidal velocity profile.

    Returns (waypoints, exact_timestamps) so that the velocity profile
    is fully preserved through the IK and timing pipeline.  Each segment
    decelerates to v=0 at its endpoints, guaranteeing C0-safe cornering.
    """
    if not positions or len(positions) < 2:
        wp = [(positions[0].copy(), orientation_xyzw)] if positions else []
        return wp, [0.0] if wp else []

    waypoints: List[Tuple[np.ndarray, list]] = []
    timestamps: List[float] = []
    current_t = 0.0  # cumulative time offset across segments

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

            # Skip duplicate overlap at segment boundaries
            if i > 0 and step == 0:
                continue

            waypoints.append((p_start + line_dir * min(s, dist),
                              orientation_xyzw))
            timestamps.append(current_t + t)

        current_t += total_time

    # Always append exact final point
    waypoints.append((positions[-1].copy(), orientation_xyzw))
    timestamps.append(current_t)
    return waypoints, timestamps


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
        self.declare_parameter('approach_height', 0.07)
        self.declare_parameter('surface_z_offset', 0.00) # in cm
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
        self.declare_parameter('totg_path_tolerance', 0.002)
        self.declare_parameter('totg_resample_dt', 0.01)
        self.declare_parameter('active_tool', 'pointy')
        self.declare_parameter('orthogonal_tool_length_m', 0.13)
        self.declare_parameter('trajectory_batch_size', 4000)

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
        self._last_precompute_fail_reason = ''

        # ---- TOTG service client ----
        self._totg_client = self.create_client(
            ComputeTOTG, 'compute_totg',
            callback_group=ReentrantCallbackGroup())
        self._totg_available = False

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
            f'surface_z_offset={float(self.surface_z_offset):.3f}m, '
            f'execution_hz={self.execution_hz:.0f}, '
            f'active_tool={self._active_tool}, '
            f'real_robot={self._real_robot}')

    # ------------------------------------------------------------------
    # Parameter helpers
    # ------------------------------------------------------------------
    def _load_params(self):
        g = self.get_parameter
        self.approach_height     = float(g('approach_height').value)
        self.surface_z_offset    = float(g('surface_z_offset').value)
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
        self._active_tool        = str(g('active_tool').value)
        self._orthogonal_tool_length = float(
            g('orthogonal_tool_length_m').value)
        self._traj_batch_size    = int(g('trajectory_batch_size').value)

    def _get_tool_transform(self, tool_name: str) -> Tuple[float, list]:
        """Return (tool_length_m, wrist_orientation_xyzw) for active tool."""
        if tool_name == 'orthogonal':
            T = self._orthogonal_wrist_pose_from_tip(self.table_center_3d)
            return max(float(self._orthogonal_tool_length), 0.0), \
                rotmat_to_quat(T[:3, :3])

        length, yaw_offset = self._tool_length_and_yaw_offset(tool_name)

        # Build a strict orthonormal basis, then apply only yaw about local Z.
        z_axis = self.plane_y / max(float(np.linalg.norm(self.plane_y)), 1e-9)
        x_seed = self.plane_n - float(np.dot(self.plane_n, z_axis)) * z_axis
        if float(np.linalg.norm(x_seed)) < 1e-9:
            x_seed = self.plane_x - float(np.dot(self.plane_x, z_axis)) * z_axis
        x_seed = x_seed / max(float(np.linalg.norm(x_seed)), 1e-9)
        y_seed = np.cross(z_axis, x_seed)
        y_seed = y_seed / max(float(np.linalg.norm(y_seed)), 1e-9)
        x_seed = np.cross(y_seed, z_axis)
        x_seed = x_seed / max(float(np.linalg.norm(x_seed)), 1e-9)

        c = math.cos(yaw_offset)
        s = math.sin(yaw_offset)
        x_axis = c * x_seed + s * y_seed
        y_axis = -s * x_seed + c * y_seed
        R = np.column_stack([x_axis, y_axis, z_axis])

        return length, rotmat_to_quat(R)

    def _tool_length_and_yaw_offset(self, tool_name: str) -> Tuple[float, float]:
        """Return (tool_length_m, yaw_offset_rad) about local wrist_3 axis.

        Calibrated flange quadrant mapping (latest observed):
          fork = 0 deg
          pointy = +90 deg
          spatula = 180 deg
          empty = -90 deg
          orthogonal = 0 deg

        Tool length from the wrist axis:
          pointy = 0.15 m
          fork/spatula/empty = 0.13 m
          orthogonal = orthogonal_tool_length_m (default 0.13 m)
        """
        pointy_len = 0.15
        other_len = 0.13
        orthogonal_len = max(float(self._orthogonal_tool_length), 0.0)
        if tool_name == 'fork':
            return other_len, 0.0
        if tool_name == 'pointy':
            return pointy_len, math.pi / 2.0
        if tool_name == 'spatula':
            return other_len, math.pi
        if tool_name == 'empty':
            return other_len, -math.pi / 2.0
        if tool_name == 'orthogonal':
            return orthogonal_len, 0.0
        self.get_logger().error(
            f"Unknown tool '{tool_name}'. Defaulting to pointy.")
        return pointy_len, math.pi / 2.0

    def _dynamic_wrist_pose_from_tip(self, tip_pos: np.ndarray) -> np.ndarray:
        """Build wrist pose for a tip point using dynamic yaw and tool offset."""
        tool_length, yaw_offset = self._tool_length_and_yaw_offset(
            str(self._active_tool))

        N = self.plane_n
        z_wrist = tip_pos - np.dot(tip_pos, N) * N
        z_norm = float(np.linalg.norm(z_wrist))
        if z_norm < 1e-6:
            z_wrist = self.plane_x.copy()
        else:
            z_wrist = z_wrist / z_norm

        x_base = N / max(float(np.linalg.norm(N)), 1e-9)
        y_base = np.cross(z_wrist, x_base)
        y_norm = float(np.linalg.norm(y_base))
        if y_norm < 1e-6:
            y_base = self.plane_y.copy()
        else:
            y_base = y_base / y_norm

        x_base = np.cross(y_base, z_wrist)
        x_base = x_base / max(float(np.linalg.norm(x_base)), 1e-9)

        c = math.cos(yaw_offset)
        s = math.sin(yaw_offset)
        x_wrist = c * x_base + s * y_base
        y_wrist = -s * x_base + c * y_base

        wrist_pos = tip_pos - tool_length * x_wrist

        T = np.eye(4)
        T[:3, 0] = x_wrist
        T[:3, 1] = y_wrist
        T[:3, 2] = z_wrist
        T[:3, 3] = wrist_pos
        return T

    def _orthogonal_wrist_pose_from_tip(self, tip_pos: np.ndarray) -> np.ndarray:
        """Build tool0 pose for an orthogonal tool with wrist_3 normal to the plane."""
        tool_length, yaw_offset = self._tool_length_and_yaw_offset(
            str(self._active_tool))

        # The physical orthogonal tip follows +tool0_z, so tool0_z must point
        # toward the plane normal for the tip to point down at the surface.
        z_wrist = self.plane_n / max(float(np.linalg.norm(self.plane_n)), 1e-9)
        x_seed = self.base_forward - float(np.dot(self.base_forward, z_wrist)) * z_wrist
        x_norm = float(np.linalg.norm(x_seed))
        if x_norm < 1e-6:
            x_seed = self.plane_x - float(np.dot(self.plane_x, z_wrist)) * z_wrist
            x_norm = float(np.linalg.norm(x_seed))
        if x_norm < 1e-6:
            x_seed = self.plane_y.copy()
            x_norm = float(np.linalg.norm(x_seed))
        x_seed = x_seed / max(x_norm, 1e-9)

        y_seed = np.cross(z_wrist, x_seed)
        y_seed = y_seed / max(float(np.linalg.norm(y_seed)), 1e-9)
        x_seed = np.cross(y_seed, z_wrist)
        x_seed = x_seed / max(float(np.linalg.norm(x_seed)), 1e-9)

        c = math.cos(yaw_offset)
        s = math.sin(yaw_offset)
        x_wrist = c * x_seed + s * y_seed
        x_wrist = x_wrist / max(float(np.linalg.norm(x_wrist)), 1e-9)
        y_wrist = -s * x_seed + c * y_seed
        y_wrist = y_wrist / max(float(np.linalg.norm(y_wrist)), 1e-9)

        # orthogonal_tool_length_m is the stand-off along the plane normal.
        wrist_pos = tip_pos - tool_length * z_wrist

        T = np.eye(4)
        T[:3, 0] = x_wrist
        T[:3, 1] = y_wrist
        T[:3, 2] = z_wrist
        T[:3, 3] = wrist_pos
        return T

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

        corners = [np.array(c, dtype=float) for c in data.get('rectangle_corners', [])]
        if len(corners) >= 4:
            self.rect_origin = corners[0]
            self.rect_width_vec = corners[1] - corners[0]
            self.rect_height_vec = corners[3] - corners[0]
            self.table_width_m = float(np.linalg.norm(self.rect_width_vec))
            self.table_height_m = float(np.linalg.norm(self.rect_height_vec))
            self.table_center_3d = (
                self.rect_origin
                + 0.5 * self.rect_width_vec
                + 0.5 * self.rect_height_vec)
        else:
            self.rect_origin = self.plane_origin.copy()
            self.rect_width_vec = self.plane_x.copy()
            self.rect_height_vec = self.plane_y.copy()
            self.table_width_m = 1.0
            self.table_height_m = 1.0
            self.table_center_3d = self.plane_origin + 0.5 * self.plane_x + 0.5 * self.plane_y

        vec_to_center = self.table_center_3d.copy()
        vec_to_center = vec_to_center - np.dot(vec_to_center, self.plane_n) * self.plane_n
        if abs(float(np.dot(self.plane_x, vec_to_center))) > abs(float(np.dot(self.plane_y, vec_to_center))):
            self.base_forward = self.plane_x if float(np.dot(self.plane_x, vec_to_center)) > 0.0 else -self.plane_x
        else:
            self.base_forward = self.plane_y if float(np.dot(self.plane_y, vec_to_center)) > 0.0 else -self.plane_y
        self.base_forward = self.base_forward / max(float(np.linalg.norm(self.base_forward)), 1e-9)

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

    @staticmethod
    def _unwrap_to_seed(q_sol: np.ndarray, q_seed: np.ndarray) -> np.ndarray:
        """Map IK solution to the nearest 2*pi-equivalent around the seed."""
        return q_seed + ((q_sol - q_seed + math.pi) % (2.0 * math.pi) - math.pi)

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
        q_sol = self._unwrap_to_seed(q_sol, q_seed)
        if not self._config_ok(q_sol):
            self._ik_cfg_count += 1
            return None
        # Use wrapped angular distance to avoid false "jumps" at +/-pi.
        joint_delta = np.abs((q_sol - q_seed + math.pi) % (2.0 * math.pi) - math.pi)
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
            self,
            T_target: np.ndarray,
            seed_center: Optional[np.ndarray] = None,
            fast_mode: bool = False) -> Optional[np.ndarray]:
        from ur5e_rrt_planner import ik_solve
        import random as _random

        home = self._home_positions.copy()
        candidates = []
        base_seed = seed_center.copy() if seed_center is not None else home.copy()
        seeds = [base_seed, home.copy()]

        num_seeds = max(6, int(self.ik_num_seeds // 2)) if fast_mode else int(self.ik_num_seeds)
        pan_span = 1.2 if fast_mode else 1.5
        shoulder_lo = -0.8 if fast_mode else -1.0
        shoulder_hi = 0.2 if fast_mode else 0.3
        elbow_span = 0.35 if fast_mode else 0.5
        wrist_span = 0.6 if fast_mode else 1.0
        max_iter = 120 if fast_mode else 300

        for _ in range(num_seeds):
            s = base_seed.copy()
            s[0] += _random.uniform(-pan_span, pan_span)
            s[1] += _random.uniform(shoulder_lo, shoulder_hi)
            s[2] += _random.uniform(-elbow_span, elbow_span)
            s[3] += _random.uniform(-wrist_span, wrist_span)
            s[4] += _random.uniform(-wrist_span, wrist_span)
            s[5] += _random.uniform(-wrist_span, wrist_span)
            seeds.append(s)

        for seed in seeds:
            q_sol = ik_solve(T_target, seed, max_iter=max_iter,
                             pos_tol=5e-4, orient_tol=1e-3,
                             damping=self.ik_damping)
            if q_sol is not None:
                q_sol = self._unwrap_to_seed(q_sol, seed)
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

    def _quick_pose_ik(self, T_target: np.ndarray, q_seed: np.ndarray) -> Optional[np.ndarray]:
        """Fast IK for sweep approach without costly global constrained search."""
        from ur5e_rrt_planner import ik_solve

        base = q_seed.copy()
        seeds = [
            base,
            self._home_positions.copy(),
            base + np.array([0.5, 0.0, 0.0, 0.0, 0.0, 0.0]),
            base + np.array([-0.5, 0.0, 0.0, 0.0, 0.0, 0.0]),
        ]

        relaxed_candidate = None
        for seed in seeds:
            q_sol = ik_solve(T_target, seed, max_iter=35,
                             pos_tol=7e-4, orient_tol=2e-3,
                             damping=self.ik_damping)
            if q_sol is None:
                continue
            q_sol = self._unwrap_to_seed(q_sol, seed)
            if self._config_ok(q_sol):
                return q_sol
            if relaxed_candidate is None:
                relaxed_candidate = q_sol

        if relaxed_candidate is not None:
            self.get_logger().warn(
                'Using relaxed sweep approach IK outside elbow-up preference')
            return relaxed_candidate
        return None

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
        cart_times: List[float],
        q_seed: np.ndarray,
        label: str = 'path',
    ) -> Tuple[List[np.ndarray], List[float]]:
        """Solve IK for every (pos, quat) waypoint.

        Skips failed points AND their corresponding timestamp so that
        the returned (path, times) arrays stay perfectly aligned.
        """
        path: List[np.ndarray] = []
        valid_times: List[float] = []
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
        self.get_logger().info(
            f'  {label}: IK solved {len(path)}/{len(cart_wps)} '
            f'({fails} fails)')
        return path, valid_times

    # ------------------------------------------------------------------
    # Time-Optimal Trajectory Generation via MoveIt 2 TOTG service
    # ------------------------------------------------------------------
    def _compute_totg(
        self, path: List[np.ndarray]
    ) -> Optional[Tuple[List[np.ndarray], List[float]]]:
        """Call the C++ TOTG service to time-parameterize a joint path.

        Returns (resampled_path, timestamps) or None on failure.
        The TOTG algorithm (Kunz & Stilman 2012) produces the
        time-optimal trajectory that respects per-joint velocity
        and acceleration limits with smooth corner blending.
        """
        if len(path) <= 1:
            return path[:], [0.0] * len(path)

        n_joints = len(path[0])

        # Build service request
        req = ComputeTOTG.Request()
        req.num_joints = n_joints
        req.waypoints_flat = []
        for q in path:
            req.waypoints_flat.extend(q.tolist())
        req.max_velocity = [self._max_joint_speed_rad] * n_joints
        req.max_acceleration = [self._max_joint_accel_rad] * n_joints
        req.path_tolerance = self._totg_path_tolerance
        req.resample_dt = self._totg_resample_dt

        # Wait for service (with timeout)
        if not self._totg_available:
            if not self._totg_client.wait_for_service(timeout_sec=10.0):
                self.get_logger().error(
                    'TOTG service not available — cannot time-parameterize')
                return None
            self._totg_available = True

        # Call service — spin-free wait (compatible with MultiThreadedExecutor)
        future = self._totg_client.call_async(req)
        deadline = _time.monotonic() + 30.0
        while not future.done():
            if _time.monotonic() > deadline:
                self.get_logger().error('TOTG service call timed out')
                return None
            _time.sleep(0.005)

        if future.result() is None:
            self.get_logger().error('TOTG service call returned None')
            return None

        resp = future.result()
        if not resp.success:
            self.get_logger().error(f'TOTG failed: {resp.message}')
            return None

        # Unpack flat response
        n_out = resp.num_output_points
        timestamps = list(resp.timestamps)
        resampled_path = []
        for i in range(n_out):
            q = np.array(
                resp.timed_positions_flat[i * n_joints:(i + 1) * n_joints],
                dtype=float)
            resampled_path.append(q)

        self.get_logger().info(
            f'    TOTG: {len(path)} in → {n_out} out, '
            f'{timestamps[-1]:.2f}s')
        return resampled_path, timestamps

    def _compute_phase_timing_fallback(
        self, path: List[np.ndarray]
    ) -> List[float]:
        """Fallback IPTP timing if TOTG service is unavailable."""
        if len(path) <= 1:
            return [0.0] * len(path)

        v_max = self._max_joint_speed_rad
        a_max = self._max_joint_accel_rad
        n_seg = len(path) - 1

        dt = np.empty(n_seg)
        for i in range(n_seg):
            max_dist = float(np.max(np.abs(path[i + 1] - path[i])))
            dt[i] = max(max_dist / v_max, 1e-4)

        for i in range(n_seg - 1):
            for j in range(len(path[0])):
                v_cur = (path[i + 1][j] - path[i][j]) / dt[i]
                v_nxt = (path[i + 2][j] - path[i + 1][j]) / dt[i + 1]
                if abs(v_nxt - v_cur) / dt[i + 1] > a_max:
                    dt[i + 1] = max(dt[i + 1],
                                    abs(v_nxt - v_cur) / a_max)

        for i in range(n_seg - 2, -1, -1):
            for j in range(len(path[0])):
                v_cur = (path[i + 1][j] - path[i][j]) / dt[i]
                v_nxt = (path[i + 2][j] - path[i + 1][j]) / dt[i + 1]
                if abs(v_cur - v_nxt) / dt[i] > a_max:
                    dt[i] = max(dt[i], abs(v_cur - v_nxt) / a_max)

        times = [0.0]
        for t in dt:
            times.append(times[-1] + t)
        return times

    # ------------------------------------------------------------------
    # Concatenate phases → master timeline with phase boundaries
    # ------------------------------------------------------------------
    def _concat_phase_times(
        self, phases: List[Tuple[str, List[np.ndarray],
                                 Optional[List[float]]]]
    ) -> Tuple[List[np.ndarray], List[float],
               List[Tuple[str, int, int]]]:
        """Concatenate independently-timed phases.

        Each phase is ``(label, joint_path, exact_timestamps_or_none)``.

        **Cartesian phases** now carry their EXACT trapezoidal
        timestamps produced by ``_interpolate_cartesian_smooth``.
        This preserves the v→0 deceleration at every sharp corner
        ($C^0$ continuity) and prevents the UR5e velocity-jump
        soft-error that occurred with the old flat-time mapping.

        **Joint-space phases** (RRT, homing) have ``times=None``
        and are time-parameterised via TOTG (with IPTP fallback).

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

        for label, plist, exact_times in phases:
            if not plist:
                continue
            if exact_times is not None and len(exact_times) == len(plist):
                # Cartesian phase: USE EXACT trapezoidal timestamps.
                # This is the critical fix — we no longer flatten
                # the carefully-bunched corner points into uniform dt.
                phase_path = plist
                phase_times = exact_times
                timing_src = 'cartesian-exact'
            else:
                # Joint-space phase: use TOTG (with IPTP fallback)
                totg_result = self._compute_totg(plist)
                if totg_result is not None:
                    phase_path, phase_times = totg_result
                    timing_src = 'TOTG'
                else:
                    self.get_logger().warn(
                        f'  TOTG failed for {label} — using IPTP fallback')
                    phase_path = plist
                    phase_times = self._compute_phase_timing_fallback(plist)
                    timing_src = 'IPTP-fallback'
            start_idx = len(master_path)
            for j, (q, t) in enumerate(zip(phase_path, phase_times)):
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
                f'  Phase: {label:15s}  {len(phase_path):4d} pts  '
                f'{dur:.2f}s  [{timing_src}]')

        return master_path, master_times, phase_info

    # ------------------------------------------------------------------
    # Send full trajectory to real robot
    # ------------------------------------------------------------------
    def _send_full_trajectory(self, path: List[np.ndarray],
                              times: List[float],
                              start_stamp=None) -> None:
        if self._real_traj_pub is None:
            return
        traj = JointTrajectory()
        traj.header.stamp = start_stamp if start_stamp is not None \
            else self.get_clock().now().to_msg()
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
            result = self._execute_goal_inner(goal_handle)
        except Exception as exc:
            self.get_logger().error(f'Goal execution error: {exc}')
            try:
                goal_handle.abort()
            except Exception:
                pass
            result = ExecuteDrawing.Result(
                success=False, message=f'Internal error: {exc}')
        finally:
            # Clear _executing AFTER setting the goal terminal state
            # but the result object is already constructed.
            self._executing = False
        return result

    def _execute_goal_inner(self, goal_handle):
        feedback = ExecuteDrawing.Feedback()

        # ---- extract goal data ----
        draw_positions = [
            np.array([p.x, p.y, p.z], dtype=float)
            for p in goal_handle.request.waypoints
        ]
        raw_first = draw_positions[0].copy() if draw_positions else None
        if abs(self.surface_z_offset) > 1e-9:
            draw_positions = [
                p - self.surface_z_offset * self.plane_n
                for p in draw_positions
            ]
        else:
            self.get_logger().warn(
                'surface_z_offset=0.0m -> drawing follows raw plane waypoints (no surface offset)')
        q = goal_handle.request.orientation
        orientation = [q.x, q.y, q.z, q.w]

        if raw_first is not None:
            shifted_first = draw_positions[0]
            n_hat = self.plane_n / max(float(np.linalg.norm(self.plane_n)), 1e-9)
            delta_n = float(np.dot(shifted_first - raw_first, n_hat))
            self.get_logger().info(
                f'Waypoint[0] offset along normal = {delta_n*1000.0:.1f} mm '
                f'(configured surface_z_offset={self.surface_z_offset*1000.0:.1f} mm)')

        self.get_logger().info(
            f'Executing drawing: {len(draw_positions)} waypoints '
            f'(surface_z_offset={self.surface_z_offset*1000.0:.1f}mm)')

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
            try:
                goal_handle.succeed()
            except Exception as exc:
                self.get_logger().error(
                    f'goal_handle.succeed() failed: {exc}')
            self.get_logger().info('Drawing complete ✓')
            return ExecuteDrawing.Result(
                success=True, message='Drawing complete')
        else:
            try:
                goal_handle.abort()
            except Exception:
                pass
            return ExecuteDrawing.Result(
                success=False, message='Execution failed')

    # ══════════════════════════════════════════════════════════════════
    # Pre-compute trajectory for one drawing goal
    # ══════════════════════════════════════════════════════════════════

    def _pre_compute_goal(
        self,
        draw_positions: List[np.ndarray],
        orientation_xyzw: list,
    ) -> Optional[List[Tuple[str, List[np.ndarray], Optional[List[float]]]]]:
        """Build all phases for one drawing. Returns list of
        (label, joint_path, exact_timestamps_or_none) tuples."""
        from ur5e_rrt_planner import (ur5e_fk, rrt_connect,
                                      smooth_path, bezier_smooth_path)

        self._last_precompute_fail_reason = ''

        # In real-robot mode, prefer the actual robot joint state over
        # the sim joint state, which may lag or be stale after a prior
        # execution.  This prevents trajectory-start mismatches that
        # cause the UR joint trajectory controller to stall.
        if self._real_robot:
            q_current = self._get_real_ordered_joints()
            if q_current is not None:
                # Sync sim so everything is consistent
                self._last_q = q_current.copy()
                self._mirror_real_to_sim()
            else:
                q_current = self._get_ordered_joints()
        else:
            q_current = self._get_ordered_joints()
        if q_current is None:
            self._last_precompute_fail_reason = 'no_joint_state'
            return None

        heavy_sweep = (
            self._active_tool in ('spatula', 'pointy', 'orthogonal')
            and len(draw_positions) >= 40
        )

        # Use execution_hz (100 Hz) unconditionally so TOTG receives
        # the same density it outputs — no wasteful down/up-sampling.
        cart_dt = 1.0 / max(float(self.execution_hz), 1.0)

        tool_length, yaw_offset = self._tool_length_and_yaw_offset(
            str(self._active_tool))

        use_dynamic_wrist = heavy_sweep

        def tip_to_wrist_pose(tip_pos: np.ndarray) -> np.ndarray:
            if str(self._active_tool) == 'orthogonal':
                return self._orthogonal_wrist_pose_from_tip(tip_pos)

            # Keep one motion model across tools; active_tool only changes TCP offset.
            return self._dynamic_wrist_pose_from_tip(tip_pos)

        # Convert tip waypoints into selected wrist pose waypoints.
        def tip_to_dynamic_wrist(
            wps: List[Tuple[np.ndarray, list]]
        ) -> List[Tuple[np.ndarray, list]]:
            out: List[Tuple[np.ndarray, list]] = []
            for tip_pos, _ in wps:
                T = tip_to_wrist_pose(tip_pos)
                out.append((T[:3, 3].copy(), rotmat_to_quat(T[:3, :3])))
            return out

        phases: List[Tuple[str, List[np.ndarray], Optional[List[float]]]] = []

        self.get_logger().info(
            '═══════════════════════════════════════════════\n'
            '  PRE-COMPUTING TRAJECTORY FOR GOAL\n'
            '═══════════════════════════════════════════════')
        self.get_logger().info(
            f'  Tool TCP: {self._active_tool} '
            f'(len={tool_length:.3f}m, yaw={math.degrees(yaw_offset):.0f}deg)')

        # ── 1. RETRACT UP (Cartesian vertical lift) ───────────────────
        T_now = ur5e_fk(q_current)
        current_pos = T_now[:3, 3].copy()
        current_quat = rotmat_to_quat(T_now[:3, :3])

        lift_pos = current_pos.copy()
        lift_pos[2] += self._retract_height

        self.get_logger().info(
            f'  [RETRACT_UP] z={current_pos[2]:.3f} → '
            f'z={lift_pos[2]:.3f} (+{self._retract_height:.3f}m)')

        retract_wps, retract_times = _interpolate_cartesian_smooth(
            [current_pos, lift_pos], current_quat,
            self.approach_v_max, self.approach_a_max, cart_dt)
        retract_path, retract_valid_times = self._ik_solve_cartesian_path(
            retract_wps, retract_times, q_current, 'Retract up')
        if not retract_path:
            self._last_precompute_fail_reason = 'retract_ik'
            self.get_logger().error('RETRACT_UP IK failed')
            return None
        phases.append(('retract_up', retract_path, retract_valid_times))

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
            phases.append(('retract_home', retract_home_path, None))

            n_hold = max(int(self._home_hold_sec * self.execution_hz), 1)
            home_hold_path = [q_home.copy() for _ in range(n_hold + 1)]
            phases.append(('home_hold', home_hold_path, None))

            self.get_logger().info(
                f'  [HOMING] {n_home} interp steps + '
                f'{self._home_hold_sec:.1f}s hold')

        # ── 3. Approach transfer (RRT to approach pose) ─
        last_q = phases[-1][1][-1]
        selected_idx = 0

        n_draw = len(draw_positions)
        if n_draw >= 40:
            candidate_idxs = sorted(set([
                0,
                n_draw // 4,
                n_draw // 2,
                (3 * n_draw) // 4,
                n_draw - 1,
            ]))
        else:
            candidate_idxs = [0]

        q_goal = None
        T_approach = None
        approach_pos_tip = None
        for idx in candidate_idxs:
            tip = draw_positions[idx]
            candidate_approach_tip = tip - self.approach_height * self.plane_n
            candidate_T = tip_to_wrist_pose(candidate_approach_tip)
            candidate_q = self._constrained_ik_for_pose(candidate_T)
            if candidate_q is not None:
                selected_idx = idx
                q_goal = candidate_q
                T_approach = candidate_T
                approach_pos_tip = candidate_approach_tip
                break

        if q_goal is None or T_approach is None or approach_pos_tip is None:
            self._last_precompute_fail_reason = 'rrt_goal_ik'
            self.get_logger().error(
                f'RRT goal IK failed for all entry candidates: {candidate_idxs}')
            return None

        if selected_idx != 0:
            draw_positions = draw_positions[selected_idx:] + draw_positions[:selected_idx]
            self.get_logger().info(
                f'  [ENTRY_SELECT] using waypoint index {selected_idx}/{n_draw - 1}')

        approach_pos_wrist = T_approach[:3, 3]
        self.get_logger().info(
            f'  [APPROACH] target '
            f'[{approach_pos_wrist[0]:.3f}, {approach_pos_wrist[1]:.3f}, '
            f'{approach_pos_wrist[2]:.3f}] (wrist), entry_idx={selected_idx}')

        raw_rrt = rrt_connect(last_q, q_goal,
                              step_size=0.2, max_iter=10000)
        if raw_rrt is None:
            self._last_precompute_fail_reason = 'rrt_plan'
            self.get_logger().error('RRT planning failed')
            return None

        smoothed = smooth_path(raw_rrt, max_attempts=200)
        rrt_dense = bezier_smooth_path(smoothed, max_step=0.02)
        phases.append(('rrt', list(rrt_dense), None))

        self.get_logger().info(
            f'        {len(raw_rrt)} raw → {len(smoothed)} smoothed '
            f'→ {len(rrt_dense)} dense')
        approach_seed_q = rrt_dense[-1]

        # Verify approach FK
        T_goal_fk = ur5e_fk(approach_seed_q)
        goal_pos = T_goal_fk[:3, 3]
        self.get_logger().info(
            f'        Approach FK = [{goal_pos[0]:.4f}, '
            f'{goal_pos[1]:.4f}, {goal_pos[2]:.4f}]')

        # ── 4. DESCENT (approach → surface) ───────────────────────────
        descent_dt = max(cart_dt, 0.02) if heavy_sweep else cart_dt
        descent_wps, descent_times = _interpolate_cartesian_smooth(
            [approach_pos_tip, draw_positions[0].copy()],
            orientation_xyzw,
            self.approach_v_max, self.approach_a_max, descent_dt)
        descent_wps = tip_to_dynamic_wrist(descent_wps)
        descent_path, descent_valid_times = self._ik_solve_cartesian_path(
            descent_wps, descent_times, approach_seed_q, 'Descent')
        if descent_path:
            phases.append(('descent', descent_path, descent_valid_times))
        else:
            self.get_logger().warn(
                '  Descent IK failed — drawing starts from approach height')

        # ── 5. DRAWING (surface waypoints -> TOTG) ───────────────────
        # Generate evenly spaced Cartesian points and let phase concat route
        # this phase through TOTG by passing times=None.
        spatial_wps: List[np.ndarray] = []
        spatial_res = 0.01  # 1 cm
        for i in range(len(draw_positions) - 1):
            p1 = draw_positions[i]
            p2 = draw_positions[i + 1]
            dist = float(np.linalg.norm(p2 - p1))
            steps = max(int(dist / spatial_res), 1)
            for j in range(steps):
                spatial_wps.append(p1 + (p2 - p1) * (j / steps))
        spatial_wps.append(draw_positions[-1].copy())

        self.get_logger().info(
            f'  [DRAW_TOTG] spatial res={spatial_res*1000.0:.0f}mm, '
            f'{len(draw_positions)} -> {len(spatial_wps)} tip points')

        draw_wps = tip_to_dynamic_wrist(
            [(p, orientation_xyzw) for p in spatial_wps])
        last_q_for_draw = phases[-1][1][-1]
        dummy_times = [0.0] * len(draw_wps)
        draw_path, _ = self._ik_solve_cartesian_path(
            draw_wps, dummy_times, last_q_for_draw, 'Drawing')
        if not draw_path or len(draw_path) < 2:
            self.get_logger().error('Drawing IK returned too few solutions')
            return None
        phases.append(('drawing', draw_path, None))

        # ── 6. ASCENT (surface → approach height) ─────────────────────
        last_draw_pos = draw_positions[-1].copy()
        ascent_pos = last_draw_pos - self.approach_height * self.plane_n
        ascent_wps, ascent_times = _interpolate_cartesian_smooth(
            [last_draw_pos, ascent_pos], orientation_xyzw,
            self.approach_v_max, self.approach_a_max, cart_dt)
        ascent_wps = tip_to_dynamic_wrist(ascent_wps)
        ascent_path, ascent_valid_times = self._ik_solve_cartesian_path(
            ascent_wps, ascent_times, draw_path[-1], 'Ascent')
        if ascent_path:
            phases.append(('ascent', ascent_path, ascent_valid_times))
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
        start_wall = _time.monotonic()          # wall-clock — sim-time-safe
        last_feedback_time = 0.0

        self.get_logger().info(
            f'Starting sim execution: {len(master_path)} pts, '
            f'{total_time:.1f}s')

        while True:
            # -- cancel check --
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Execution canceled')
                return False

            elapsed = _time.monotonic() - start_wall

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
        """Stream trajectory to real robot in batches and monitor convergence.

        Global Planning, Chunked Execution with Absolute Time Locking:
        TOTG has already produced a single globally-optimal C2 speed
        profile.  We slice that output into batches of
        ≤ trajectory_batch_size points and stream them to the UR
        controller.  header.stamp is locked to T0 (captured once
        before the first batch) so that all batches share the same
        time origin — the controller uses header.stamp +
        time_from_start to schedule each point.  The next batch is
        dispatched when the robot reaches ~70% of the current batch
        so the controller's interpolation buffer never empties.
        """
        batch_sz = max(self._traj_batch_size, 500)
        n_master = len(master_path)
        target_q = master_path[-1]
        total_time = master_times[-1]

        # ---- Build batch boundaries ----
        # Each batch is [start_idx, end_idx) except the last which
        # includes the final point.  All timestamps stay absolute.
        batches: List[Tuple[int, int]] = []
        idx = 0
        while idx < n_master:
            end = min(idx + batch_sz, n_master)
            batches.append((idx, end))
            idx = end
        n_batches = len(batches)

        self.get_logger().info(
            f'Trajectory streaming: {n_master} pts → '
            f'{n_batches} batches (max {batch_sz} pts each), '
            f'{total_time:.1f}s total')

        # ---- Absolute Time Lock: capture T0 once for ALL batches ----
        t0 = self.get_clock().now()
        t0_msg = t0.to_msg()

        # ---- Send first batch ----
        cur_batch = 0
        b_start, b_end = batches[cur_batch]
        self._send_full_trajectory(
            master_path[b_start:b_end],
            master_times[b_start:b_end],
            start_stamp=t0_msg)
        self.get_logger().info(
            f'  Batch {cur_batch+1}/{n_batches} sent: '
            f'pts [{b_start}..{b_end-1}], '
            f't=[{master_times[b_start]:.1f}..{master_times[b_end-1]:.1f}]s')
        last_feedback_time = 0.0
        last_closest_idx = 0

        monitor_dt = 0.1
        min_monitor_time = total_time * 0.80
        min_progress_fraction = 0.85
        timeout = total_time * 1.5 + 15.0

        while True:
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Execution canceled (real)')
                return False

            try:
                self._mirror_real_to_sim()
            except Exception:
                pass

            elapsed = (self.get_clock().now() - t0).nanoseconds * 1e-9

            # ---- Stream next batch at 80% of current batch ----
            if cur_batch < n_batches - 1:
                b_start_cur, b_end_cur = batches[cur_batch]
                # 70% point of current batch by index
                trigger_idx = b_start_cur + int(
                    0.70 * (b_end_cur - b_start_cur))
                if last_closest_idx >= trigger_idx:
                    cur_batch += 1
                    b_start, b_end = batches[cur_batch]
                    self._send_full_trajectory(
                        master_path[b_start:b_end],
                        master_times[b_start:b_end],
                        start_stamp=t0_msg)
                    self.get_logger().info(
                        f'  Batch {cur_batch+1}/{n_batches} sent: '
                        f'pts [{b_start}..{b_end-1}], '
                        f't=[{master_times[b_start]:.1f}..'
                        f'{master_times[b_end-1]:.1f}]s')

            # ---- Convergence check ----
            progress_frac = last_closest_idx / max(n_master - 1, 1)
            if (elapsed > min_monitor_time
                    and progress_frac >= min_progress_fraction
                    and self._real_robot_converged(target_q)):
                real_q = self._get_real_ordered_joints()
                self._last_q = real_q if real_q is not None \
                    else target_q.copy()
                self._mirror_real_to_sim()
                self.get_logger().info(
                    f'Real robot converged to target '
                    f'(after {elapsed:.1f}s, '
                    f'progress={progress_frac*100:.0f}%)')
                return True

            if elapsed > timeout:
                real_q = self._get_real_ordered_joints()
                if real_q is not None:
                    self._last_q = real_q
                    self._mirror_real_to_sim()
                self.get_logger().error(
                    f'Real robot timed out ({timeout:.0f}s), '
                    f'progress={progress_frac*100:.0f}%')
                return False

            # ---- Progress estimation ----
            real_q = self._get_real_ordered_joints()
            if real_q is not None and elapsed - last_feedback_time >= 1.0:
                last_feedback_time = elapsed
                diff = np.array(master_path) - real_q
                dists = np.max(np.abs(diff), axis=1)
                closest = int(np.argmin(dists))
                last_closest_idx = max(last_closest_idx, closest)
                phase_label = self._phase_for_idx(
                    last_closest_idx, phase_info)
                draw_pct = self._drawing_progress(
                    last_closest_idx, phase_info)
                feedback.current_phase = phase_label.upper()
                feedback.drawing_progress = draw_pct
                goal_handle.publish_feedback(feedback)
                pct = 100.0 * last_closest_idx / max(n_master - 1, 1)
                self.get_logger().info(
                    f'  ~step {last_closest_idx}/{n_master-1}  '
                    f'({pct:.0f}%)  [{phase_label}]  '
                    f'batch {cur_batch+1}/{n_batches}')

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
            if self._real_robot:
                cart_dt = 0.10
            else:
                cart_dt = 1.0 / self.execution_hz

            lift_pos = current_pos.copy()
            lift_pos[2] += self._retract_height

            retract_wps, retract_times = _interpolate_cartesian_smooth(
                [current_pos, lift_pos], current_quat,
                self.approach_v_max, self.approach_a_max, cart_dt)
            retract_path, _ = self._ik_solve_cartesian_path(
                retract_wps, retract_times, q_current, 'Shutdown retract')

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
                times = self._compute_phase_timing_fallback(full_path)
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
