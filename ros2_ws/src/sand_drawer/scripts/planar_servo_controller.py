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
1. APPROACH  – Move to a point *above* the first waypoint (or plane center in teleop), fix orientation
2. LOWER     – Descend onto the plane surface
3. SERVO     – Follow the waypoint trajectory on the plane
4. TELEOP    – Accept manual velocity commands from /teleop_plane_vel
5. DONE      – Hold (zero velocity)
"""


import json
import math
import os
from enum import Enum, auto

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
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
    HOME     = auto()
    APPROACH = auto()
    LOWER    = auto()
    SERVO    = auto()
    TELEOP   = auto()
    DONE     = auto()


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
        self.declare_parameter('kp_angular', 1.5)
        self.declare_parameter('max_linear_vel', 0.25)         # m/s
        self.declare_parameter('max_angular_vel', 0.60)        # rad/s
        self.declare_parameter('plane_z_correction_gain', 2.0)
        self.declare_parameter('loop_trajectory', False)
        self.declare_parameter('trajectory_key', 'projected_vector_trajectory')
        self.declare_parameter('boundary_margin', 0.01)        # m
        self.declare_parameter('teleop_mode', False)
        self.declare_parameter('teleop_speed', 0.10)            # m/s default

        self._load_params()
        self._load_plane_json()

        # ---- TF2 ----
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- publisher ----
        self.twist_pub = self.create_publisher(Twist, '/end_effector_velocity', 10)
        self.joint_cmd_pub = self.create_publisher(JointState, '/isaac_joint_commands', 10)

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

        # ---- teleop subscription ----
        self._teleop_vel = Twist()  # latest teleop command (in plane frame)
        self._teleop_stamp = self.get_clock().now()
        if self.teleop_mode:
            self.teleop_sub = self.create_subscription(
                Twist, '/teleop_plane_vel',
                self._teleop_callback, 10)

        # ---- state ----
        self.phase        = Phase.HOME
        self.waypoint_idx = 0

        # ---- stuck detection ----
        self._stuck_dist      = float('inf')  # last recorded distance
        self._stuck_time      = self.get_clock().now()  # when dist last improved
        self._stuck_patience  = 8.0   # seconds without progress before forcing transition
        self._stuck_improve   = 0.005 # m — minimum improvement to reset timer

        # ---- control timer 10 Hz ----
        self.timer = self.create_timer(0.1, self._control_loop)

        self.get_logger().info(
            f'Planar servo controller started — {len(self.waypoints)} waypoints, '
            f'trajectory_key={self.traj_key}, teleop_mode={self.teleop_mode}')

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
        self.kp_ang           = g('kp_angular').value
        self.max_lin          = g('max_linear_vel').value
        self.max_ang          = g('max_angular_vel').value
        self.z_corr_gain      = g('plane_z_correction_gain').value
        self.do_loop          = g('loop_trajectory').value
        self.traj_key         = g('trajectory_key').value
        self.boundary_margin  = g('boundary_margin').value
        self.teleop_mode      = g('teleop_mode').value
        self.teleop_speed     = g('teleop_speed').value

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

        # ---- trajectory waypoints ----
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
    # Stuck detection helpers
    # ------------------------------------------------------------------
    def _update_stuck(self, dist):
        """Track distance progress; return seconds since last significant improvement."""
        if dist < self._stuck_dist - self._stuck_improve:
            # Made meaningful progress — reset timer
            self._stuck_dist = dist
            self._stuck_time = self.get_clock().now()
        return (self.get_clock().now() - self._stuck_time).nanoseconds * 1e-9

    def _reset_stuck(self):
        """Reset stuck detector for next phase."""
        self._stuck_dist = float('inf')
        self._stuck_time = self.get_clock().now()

    # ------------------------------------------------------------------
    # HOME phase
    # ------------------------------------------------------------------
    def _home(self):
        """Publish home joint positions directly, wait 2s, then transition."""
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = self._home_joint_names
        cmd.position = self._home_joint_positions
        self.joint_cmd_pub.publish(cmd)

        # Check if joints have arrived (via last joint state from TF/subscriber)
        # For simplicity, just start the hold timer on first call
        if self._home_reached_time is None:
            self._home_reached_time = self.get_clock().now()
            self.get_logger().info('HOME  Sending robot to home position…')

        elapsed = (self.get_clock().now() - self._home_reached_time).nanoseconds * 1e-9
        self.get_logger().info(
            f'HOME  holding ({elapsed:.1f}/{self._home_hold_duration:.1f}s)',
            throttle_duration_sec=1.0)

        if elapsed >= self._home_hold_duration:
            self.get_logger().info('Home hold complete → APPROACH')
            self.phase = Phase.APPROACH
            self._reset_stuck()

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
        """Angular velocity to correct orientation towards target_quat."""
        err = orientation_error_aa(quat_now, self.target_quat)
        return clamp_vec(self.kp_ang * err, self.max_ang)

    # ------------------------------------------------------------------
    # Control phases
    # ------------------------------------------------------------------
    def _approach(self, pos, quat):
        """Move to a hover point above the first waypoint (or plane center in teleop)."""
        if self.teleop_mode:
            target = self.plane_center.copy()
        else:
            target = self.waypoints[0][0].copy()
        # Go against the normal to hover above the plane
        target -= self.plane_n * self.approach_height

        err = target - pos
        dist = np.linalg.norm(err)
        orient_err_mag = np.linalg.norm(
            orientation_error_aa(quat, self.target_quat))

        lin = clamp_vec(self.kp_lin * err, self.max_lin)
        ang = self._orient_vel(quat)

        # Stuck detection: force transition if not making progress
        stuck_elapsed = self._update_stuck(dist)

        self.get_logger().info(
            f'APPROACH  dist={dist:.4f}m  orient_err={orient_err_mag:.3f}rad  '
            f'stuck={stuck_elapsed:.1f}s',
            throttle_duration_sec=1.0)

        threshold_ok = dist < self.approach_thresh and orient_err_mag < self.orient_thresh
        patience_ok  = (stuck_elapsed > self._stuck_patience
                        and orient_err_mag < self.orient_thresh)
        if threshold_ok or patience_ok:
            if patience_ok and not threshold_ok:
                self.get_logger().warn(
                    f'Approach forced by patience (dist={dist:.4f}m > thresh={self.approach_thresh}m)')
            self.get_logger().info('Approach complete → LOWER')
            self.phase = Phase.LOWER
            self._reset_stuck()

        return self._make_twist(lin, ang)

    def _lower(self, pos, quat):
        """Descend onto the plane (first waypoint or center position)."""
        if self.teleop_mode:
            target = self.plane_center.copy()
        else:
            target = self.waypoints[0][0]
        err = target - pos
        dist = np.linalg.norm(err)

        # Slower descent
        lin = clamp_vec(0.5 * self.kp_lin * err, 0.5 * self.max_lin)
        ang = self._orient_vel(quat)

        # Stuck detection
        stuck_elapsed = self._update_stuck(dist)

        self.get_logger().info(
            f'LOWER  dist={dist:.4f}m  stuck={stuck_elapsed:.1f}s',
            throttle_duration_sec=1.0)

        threshold_ok = dist < self.wp_thresh
        patience_ok  = stuck_elapsed > self._stuck_patience
        if threshold_ok or patience_ok:
            if patience_ok and not threshold_ok:
                self.get_logger().warn(
                    f'Lower forced by patience (dist={dist:.4f}m > thresh={self.wp_thresh}m)')
            if self.teleop_mode:
                self.get_logger().info('On plane → TELEOP (waiting for keyboard commands)')
                self.phase = Phase.TELEOP
            else:
                self.get_logger().info('On plane → SERVO')
                self.phase = Phase.SERVO
                self.waypoint_idx = 1        # already at wp 0, head to wp 1
            self._reset_stuck()

        return self._make_twist(lin, ang)

    def _servo(self, pos, quat):
        """Follow trajectory waypoints, constrained to the plane."""
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

        # Planar velocity + Z drift correction
        vx = self.kp_lin * err_px
        vy = self.kp_lin * err_py
        vz = self.z_corr_gain * err_pz  # push back to plane

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
            f'plane_xy=({ex:.3f},{ey:.3f})',
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
    # Main control loop (10 Hz)
    # ------------------------------------------------------------------
    def _control_loop(self):
        # HOME phase doesn't need EE pose — just sends joint commands
        if self.phase == Phase.HOME:
            self._home()
            return

        pos, quat = self._get_ee_pose()
        if pos is None:
            # No TF yet — publish zero
            self.twist_pub.publish(Twist())
            return

        if self.phase == Phase.APPROACH:
            tw = self._approach(pos, quat)
        elif self.phase == Phase.LOWER:
            tw = self._lower(pos, quat)
        elif self.phase == Phase.SERVO:
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
