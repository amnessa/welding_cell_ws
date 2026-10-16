#!/usr/bin/env python3
"""Finger-pointing camera aim: the wrist camera follows the operator's fingertip.

While the hand signs Pointing_Up and no frame is frozen (before the capture), the
gesture node publishes the index fingertip's pixel on /gesture/aim. This node turns the
camera - about its own optical centre, roll held at home - until that pixel is in the
middle of the image; the next ILoveYou captures from the aimed pose. The control law is
admittance_control/camera_aim.py (a filtered P loop with exact latency compensation).

Execution: the UR driver's forward_velocity_controller (loaded inactive by
ur_control.launch.py). This node computes joint rates at `rate_hz` (125 Hz: one Python
step costs 2.5 ms) and publishes them on `command_topic`, /camera_aim/joint_rates. The
C++ aim_rate_relay ramps between them at the UR's 500 Hz and forwards them to the
controller: sent directly, every new rate was a velocity step the arm applied as a
torque kick, and the wrist buzzed (notes/thesis_notes.md, 2026-10-16). Without the relay
the node refuses to start; `command_topic:=/forward_velocity_controller/commands`
streams directly (buzzes). On the first aim message - the arm
still, the camera within `home_tol_m` of where it is at home_q (config/marking.json) -
it switches scaled_joint_trajectory_controller (whatever holds the joints) to the
velocity controller. When the fingertip has been gone `deadman_s` the rate ramps to
zero, and once still it switches back, so refine_pose and marking find their controller.

Safety, every cycle:
  * deadman: no fingertip for `deadman_s` -> decelerate to a stop. Any other gesture does
    the same: the gesture node only publishes on Pointing_Up;
  * excursion: +-pan_limit_deg / +-tilt_limit_deg of the optical axis from the home view;
  * the pose `check_ahead_s` ahead, against the collision model (table plane and fixtures
    of marking.json, the arm and the pen tool), the branch lock and the wrist singularity;
  * |F| beyond `abort_force_n` (biased at the start), stale /joint_states;
  * the pendant's speed slider scales all of it.
A refusal stops the arm at once, hands the joints back to the trajectory controller and
stays latched until the hand has been away a second. Ctrl-C, SIGTERM and an exception in
the loop stop it too. A hard kill (SIGKILL) mid-turn is caught by the relay: no input for
0.1 s and it ramps the arm to zero itself.

The dry run (default) switches nothing and publishes nothing: it integrates the rates on
a simulated arm, from the real one, and logs where the camera would turn. The fingertip
ray is held in the base frame, so the simulated camera centres it as the real one would.

Run (the UR driver up): `ros2 launch admittance_control gestures.launch.py` starts the
gesture node, this node and the relay. On their own:
    ros2 run admittance_control aim_rate_relay
    ros2 run admittance_control camera_aim_node.py                         # dry run
    ros2 run admittance_control camera_aim_node.py --ros-args -p dry_run:=false
"""

from __future__ import annotations

import collections
import math
import signal
import sys
import time
from pathlib import Path
from typing import List, Optional

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import PointStamped, WrenchStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CameraInfo, JointState
from std_msgs.msg import Float64MultiArray, String

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

from admittance_control.camera_aim import (AimConfig, CameraAim, JointHistory,  # noqa: E402
                                           check_ahead, pixel_ray)
from admittance_control.collision import CollisionModel  # noqa: E402
from admittance_control.kinematics import use_kinematics  # noqa: E402
from admittance_control.marking import fast_is_valid  # noqa: E402
from admittance_control.tack_reach import (UR_ORDER, joint_state_to_ur_order,  # noqa: E402
                                           load_marking_config)
from admittance_control.tool_model import load_tool_model  # noqa: E402

# AimConfig fields exposed as parameters, read at the start of every session
AIM_PARAMS = ('gain_per_s', 'deadband_frac', 'max_rate_deg_s', 'max_acc_deg_s2',
              'pan_limit_deg', 'tilt_limit_deg', 'max_joint_speed_rad_s', 'check_ahead_s')


def _stamp_s(stamp) -> float:
    return float(stamp.sec) + 1e-9 * float(stamp.nanosec)


class CameraAimNode(Node):
    IDLE, STARTING, ACTIVE, STOPPING, LATCHED = 'idle', 'starting', 'active', 'stopping', 'latched'

    def __init__(self) -> None:
        super().__init__('camera_aim')
        self.declare_parameter('dry_run', True)
        self.declare_parameter('aim_topic', '/gesture/aim')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('wrench_topic', '/force_torque_sensor_broadcaster/wrench')
        self.declare_parameter('command_topic', '/camera_aim/joint_rates')   # -> aim_rate_relay
        self.declare_parameter('controller_manager', '/controller_manager')
        self.declare_parameter('velocity_controller', 'forward_velocity_controller')
        self.declare_parameter('marking_config', '')            # '' = config/marking.json
        self.declare_parameter('extrinsic_path', '')            # '' = the tool config's
        self.declare_parameter('kinematics_file', str(PKG / 'config' / 'ur5e_calibration.yaml'))
        self.declare_parameter('rate_hz', 125.0)
        self.declare_parameter('deadman_s', 0.3)
        self.declare_parameter('idle_s', 0.3)                   # still this long -> hand back
        self.declare_parameter('home_tol_m', 0.05)
        self.declare_parameter('still_rad_s', 0.02)             # "the arm is still"
        self.declare_parameter('abort_force_n', 15.0)
        defaults = AimConfig()
        for name in AIM_PARAMS:
            self.declare_parameter(name, float(getattr(defaults, name)))

        self.get_logger().info('kinematics: ' + use_kinematics(
            str(self.get_parameter('kinematics_file').value) or None))
        self._mcfg = load_marking_config(str(self.get_parameter('marking_config').value) or None)
        tool = load_tool_model(extrinsic_path=str(self.get_parameter('extrinsic_path').value) or None)
        if tool.T_tool0_cam is None:
            raise RuntimeError('the tool model has no camera extrinsic (T_tcp_to_cam.npy)')
        self._T_tool0_cam = tool.T_tool0_cam
        # The parts are not in it: before the capture none is registered. The table plane,
        # the fixtures and the arm + pen are what an orientation-only move can hit.
        model = CollisionModel(tool=tool, scene_boxes=self._mcfg.fixture_boxes(),
                               clearance=self._mcfg.clearance_m, **self._mcfg.collision_kwargs())
        self._is_valid = fast_is_valid(model)
        self._aim = self._new_aim()
        home_ok = self._is_valid(self._mcfg.home_q)
        self.get_logger().info(
            f'collision model: {self._mcfg.describe_table()}; home pose '
            f'{"clear" if home_ok else "IN COLLISION - check marking.json"}')

        self._dry = bool(self.get_parameter('dry_run').value)
        self._hist = JointHistory()
        self._q: Optional[np.ndarray] = None
        self._qd_meas = np.zeros(6)
        self._q_rx = 0.0
        self._q_sim: Optional[np.ndarray] = None
        self._K: Optional[np.ndarray] = None
        self._width = 0
        self._aim_rx = 0.0                     # monotonic time of the last fingertip
        self._wrench = collections.deque(maxlen=25)
        self._bias = np.zeros(3)
        self._state = self.IDLE
        self._handed_from: List[str] = []      # controllers we deactivated, to give back
        self._last_tick = time.monotonic()
        self._last_status = 0.0
        self._warned_clock = False

        self.create_subscription(JointState, str(self.get_parameter('joint_states_topic').value),
                                 self._on_joints, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, str(self.get_parameter('camera_info_topic').value),
                                 self._on_info, qos_profile_sensor_data)
        self.create_subscription(PointStamped, str(self.get_parameter('aim_topic').value),
                                 self._on_aim, 10)
        self.create_subscription(WrenchStamped, str(self.get_parameter('wrench_topic').value),
                                 self._on_wrench, qos_profile_sensor_data)
        self._cmd_pub = self.create_publisher(
            Float64MultiArray, str(self.get_parameter('command_topic').value), 10)
        self._status_pub = self.create_publisher(String, '~/status', 10)
        cm = str(self.get_parameter('controller_manager').value).rstrip('/')
        self._vel = str(self.get_parameter('velocity_controller').value)
        self._switch = self.create_client(SwitchController, f'{cm}/switch_controller')
        self._list = self.create_client(ListControllers, f'{cm}/list_controllers')

        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self._tick)
        self.create_timer(2.0, self._check_controllers_once)
        self._checked = False
        self._say(f'camera aim ready ({"DRY RUN: nothing moves" if self._dry else "LIVE: the arm moves"}); '
                  f'Pointing_Up before the capture turns the camera')

    def _new_aim(self) -> CameraAim:
        cfg = AimConfig(**{n: float(self.get_parameter(n).value) for n in AIM_PARAMS})
        return CameraAim(cfg, self._T_tool0_cam, self._mcfg.home_q)

    # ── messages ─────────────────────────────────────────────────────────
    def _say(self, text: str, warn: bool = False) -> None:
        # two call sites: rclpy refuses a logging call site whose severity changes
        if warn:
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)
        self._status_pub.publish(String(data=text))

    def _on_joints(self, msg: JointState) -> None:
        try:
            q = joint_state_to_ur_order(msg.name, msg.position)
            qd = (joint_state_to_ur_order(msg.name, msg.velocity)
                  if len(msg.velocity) == len(msg.name) else np.zeros(6))
        except KeyError:
            return                             # not the arm's joint states
        self._q, self._qd_meas, self._q_rx = q, qd, time.monotonic()
        self._hist.add(_stamp_s(msg.header.stamp), q)

    def _on_info(self, msg: CameraInfo) -> None:
        self._K = np.asarray(msg.k, float).reshape(3, 3)
        self._width = int(msg.width)

    def _on_wrench(self, msg: WrenchStamped) -> None:
        f = msg.wrench.force
        self._wrench.append(np.array([f.x, f.y, f.z]))

    def _on_aim(self, msg: PointStamped) -> None:
        if self._K is None or self._q is None:
            return
        self._aim_rx = time.monotonic()
        if self._state != self.ACTIVE:
            return
        t_img = _stamp_s(msg.header.stamp)
        q_img = self._hist.at(t_img)
        if q_img is None:
            # the image clock and the joint clock do not overlap: aim without the
            # latency compensation (stable, a little overshoot) and say so once
            if not self._warned_clock:
                self._warned_clock = True
                self._say('aim: no joint state at the image stamp (clocks?) - '
                          'latency not compensated', warn=True)
            q_img, t_img = self._q, self._hist_now()
        # the history holds the REAL arm, so in the dry run too this is the pose of the
        # camera that took the image; the step then turns the simulated one towards it
        self._aim.observe(t_img, pixel_ray(msg.point.x, msg.point.y, self._K), q_img)

    def _hist_now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ── the controllers ──────────────────────────────────────────────────
    def _check_controllers_once(self) -> None:
        if self._checked or self._dry:
            return
        if not self._list.service_is_ready():
            self.get_logger().warn('controller_manager not up yet', throttle_duration_sec=10.0)
            return
        self._checked = True

        def done(fut):
            names = {c.name: c.state for c in fut.result().controller}
            if self._vel not in names:
                self._say(f'{self._vel} is NOT loaded - ur_control.launch.py loads it inactive; '
                          f'otherwise: ros2 control load_controller --set-state inactive {self._vel}',
                          warn=True)
            else:
                self.get_logger().info(f'{self._vel}: {names[self._vel]}')
        self._list.call_async(ListControllers.Request()).add_done_callback(done)

    def _take_joints(self) -> None:
        """Deactivate whatever commands the arm joints, activate the velocity controller."""
        joints = set(UR_ORDER)

        def switched(fut):
            try:
                ok = fut.result().ok
            except Exception as exc:  # noqa: BLE001
                ok, exc_text = False, str(exc)
            else:
                exc_text = ''
            if self._state != self.STARTING:
                return
            if not ok:
                self._handed_from = []         # nothing was switched: nothing to give back
                self._latch(f'controller switch refused {exc_text}'.strip())
                return
            self._aim.start(self._q)
            self._state = self.ACTIVE
            self._say(f'AIM: {self._vel} active (from {", ".join(self._handed_from) or "nothing"})')

        def listed(fut):
            if self._state != self.STARTING:
                return
            try:
                ctrls = fut.result().controller
            except Exception as exc:  # noqa: BLE001
                self._latch(f'list_controllers failed: {exc}')
                return
            vel = [c for c in ctrls if c.name == self._vel]
            if not vel:
                self._latch(f'{self._vel} is not loaded')
                return
            if vel[0].state == 'active':
                # someone else streams to it (a second aim node, MoveIt Servo): two
                # writers on one velocity controller is not something to join
                self._latch(f'{self._vel} is already active - another aim node or Servo? '
                            f'After a crash of this node, hand the joints back by hand: '
                            f'ros2 control switch_controllers --activate '
                            f'scaled_joint_trajectory_controller --deactivate {self._vel}')
                return
            self._handed_from = [
                c.name for c in ctrls if c.state == 'active' and c.name != self._vel
                and any(i.split('/')[0] in joints for i in c.claimed_interfaces)]
            req = SwitchController.Request()
            req.activate_controllers = [self._vel]
            req.deactivate_controllers = list(self._handed_from)
            req.strictness = SwitchController.Request.STRICT
            req.activate_asap = True
            req.timeout = Duration(sec=2)
            self._switch.call_async(req).add_done_callback(switched)

        if not (self._list.service_is_ready() and self._switch.service_is_ready()):
            self._latch('controller_manager services are not available')
            return
        self._list.call_async(ListControllers.Request()).add_done_callback(listed)

    def _give_back(self, then: str) -> None:
        """Zero rate, then the trajectory controller(s) back; state `then` when done."""
        self._publish(np.zeros(6))
        back = list(self._handed_from)
        self._handed_from = []
        if self._dry:
            self._state = then
            return
        req = SwitchController.Request()
        req.activate_controllers = back
        req.deactivate_controllers = [self._vel]
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True
        req.timeout = Duration(sec=2)

        def done(fut):
            try:
                ok = fut.result().ok
            except Exception:  # noqa: BLE001
                ok = False
            if not ok:
                self._say(f'could not hand the joints back to {back}: activate it by hand '
                          f'(ros2 control switch_controllers --activate {" ".join(back)} '
                          f'--deactivate {self._vel})', warn=True)
            else:
                self.get_logger().info(f'joints handed back to {", ".join(back) or "nothing"}')
            if self._state == self.STOPPING:
                self._state = then
        self._state = self.STOPPING if then == self.IDLE else then
        self._switch.call_async(req).add_done_callback(done)

    def _publish(self, qd: np.ndarray) -> None:
        if not self._dry:
            self._cmd_pub.publish(Float64MultiArray(data=[float(x) for x in qd]))

    def _latch(self, why: str) -> None:
        """Stop now, give the joints back, wait for the hand to go away."""
        self._say(f'AIM STOPPED: {why}', warn=True)
        if self._handed_from or self._state in (self.ACTIVE, self.STOPPING):
            self._give_back(self.LATCHED)
        self._state = self.LATCHED
        self._aim.lose()

    # ── the loop ─────────────────────────────────────────────────────────
    def _tick(self) -> None:
        now = time.monotonic()
        dt = float(np.clip(now - self._last_tick, 0.005, 0.05))
        self._last_tick = now
        try:
            self._step(now, dt)
        except Exception as exc:  # noqa: BLE001 - whatever it was, the arm stops
            self._publish(np.zeros(6))
            try:
                self._latch(f'error in the control loop: {exc!r}')
            except Exception:  # noqa: BLE001
                self._state = self.LATCHED

    def _step(self, now: float, dt: float) -> None:
        fresh = (now - self._aim_rx) < float(self.get_parameter('deadman_s').value)
        if self._state == self.LATCHED:
            if now - self._aim_rx > 1.0:
                self._state = self.IDLE
                self._say('aim re-armed')
            return
        if self._state in (self.STARTING, self.STOPPING):
            return
        if self._q is None:
            return
        if now - self._q_rx > 0.2:
            if self._state == self.ACTIVE:
                self._latch('/joint_states stale')
            return

        if self._state == self.IDLE:
            if fresh:
                self._try_start()
            return

        # ACTIVE
        q = self._q_sim if self._dry else self._q
        if not fresh and self._aim.ray_base is not None:
            self._aim.lose()
        deadband = self._aim.cfg.deadband_frac * self._width / self._K[0, 0]
        qd, info = self._aim.step(q, dt, deadband)
        why = check_ahead(q, qd, self._aim.cfg, self._is_valid,
                          self._mcfg.branch_signature, self._mcfg.branch_joints)
        if why is None and self._wrench and not self._dry:
            f = float(np.linalg.norm(self._wrench[-1] - self._bias))
            if f > float(self.get_parameter('abort_force_n').value):
                why = f'|F| {f:.1f} N'
        if why is not None:
            self._latch(why)
            return
        if self._dry:
            self._q_sim = q + qd * dt
        self._publish(qd)

        if now - self._last_status > 0.5:
            self._last_status = now
            text = self._status_text(info)
            self._status_pub.publish(String(data=text))
            if self._dry:
                self.get_logger().info(text)

        idle = float(self.get_parameter('idle_s').value)
        if not fresh and self._aim.stopped() and now - self._aim_rx > self._dead() + idle:
            self._say(f'aim done: pan {info.pan_deg:+.1f} tilt {info.tilt_deg:+.1f} deg from home')
            self._give_back(self.IDLE)

    def _dead(self) -> float:
        return float(self.get_parameter('deadman_s').value)

    def _status_text(self, info) -> str:
        err = ('no fingertip' if info.error is None else
               f'error ({info.error[0] * self._K[0, 0]:+.0f}, {info.error[1] * self._K[1, 1]:+.0f}) px')
        return (f'{"DRY " if self._dry else ""}AIM {err}, {info.rate_deg_s:.1f} deg/s, '
                f'pan {info.pan_deg:+.1f} tilt {info.tilt_deg:+.1f} roll {info.roll_deg:+.1f} deg, '
                f'centre drift {info.drift_mm:.1f} mm'
                + (f', at the {"/".join(info.held)} limit' if info.held else ''))

    def _try_start(self) -> None:
        if self._K is None or self._width <= 0:
            self._latch('no camera_info yet')
            return
        moving = float(np.max(np.abs(self._qd_meas)))
        if moving > float(self.get_parameter('still_rad_s').value):
            self._latch(f'the arm is moving ({moving:.2f} rad/s) - aim only takes a still arm')
            return
        self._aim = self._new_aim()            # the parameters as they are now
        away = self._aim.distance_from_home_m(self._q)
        tol = float(self.get_parameter('home_tol_m').value)
        if away > tol:
            self._latch(f'the camera is {away * 1000:.0f} mm from its home position '
                        f'(> {tol * 1000:.0f}): move the arm home first')
            return
        if not self._is_valid(self._q):
            self._latch('the current pose is already in collision in the model')
            return
        self._bias = (np.mean(np.asarray(self._wrench), axis=0)
                      if self._wrench else np.zeros(3))
        if self._dry:
            self._q_sim = self._q.copy()
            self._aim.start(self._q)
            self._state = self.ACTIVE
            self._say('DRY AIM: simulating from the current pose (nothing moves)')
            return
        topic = str(self.get_parameter('command_topic').value)
        if self._cmd_pub.get_subscription_count() == 0:
            self._latch(f'nothing listens on {topic}: start aim_rate_relay (gestures.launch.py '
                        f'does), or command_topic:=/forward_velocity_controller/commands to '
                        f'stream directly (the wrist buzzes)')
            return
        self._state = self.STARTING
        self._take_joints()

    def shutdown_stop(self) -> None:
        """Ctrl-C / SIGTERM: zero rate and the joints back, waiting for the switch."""
        if self._dry:
            return
        self._publish(np.zeros(6))
        if not self._handed_from:
            return
        req = SwitchController.Request()
        req.activate_controllers = list(self._handed_from)
        req.deactivate_controllers = [self._vel]
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True
        req.timeout = Duration(sec=2)
        fut = self._switch.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        ok = fut.done() and fut.result() is not None and fut.result().ok
        self.get_logger().info(f'shutdown: joints handed back to {self._handed_from}: {ok}')


def main(args: Optional[List[str]] = None) -> None:
    # rclpy's own handler shuts the context down on SIGINT, and then the zero command
    # and the controller switch could no longer be sent: take the signals ourselves
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    def _term(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _term)
    # explicitly: a process started in the background inherits SIGINT ignored, and
    # Python then installs no handler at all - Ctrl-C would never stop the arm
    signal.signal(signal.SIGINT, signal.default_int_handler)
    node = CameraAimNode()
    # `-p dry_run:=false` WITHOUT --ros-args is not a parameter: rcl takes it for an old
    # remap rule and the node stays in the dry run (2026-10-15, on the bench)
    from rclpy.utilities import remove_ros_args
    stray = remove_ros_args(sys.argv if args is None else args)[1:]
    if stray:
        node.get_logger().error(
            f'ignored arguments {stray}: parameters go after --ros-args, e.g.\n'
            f'  ros2 run admittance_control camera_aim_node.py --ros-args -p dry_run:=false\n'
            f'still {"in the DRY RUN" if node._dry else "LIVE"}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ctrl-C reaches every node twice under ros2 launch (the terminal signals the
        # whole process group, then launch forwards its own): a second interrupt here
        # would abort the zero command and the hand-back half-way. Ignore it.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        try:
            node.shutdown_stop()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
