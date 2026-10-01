"""Joint-trajectory execution with a wrench watchdog, shared by the nodes that move the arm.

`TrajectoryExecutor` is attached to a node (composition, not a base class) and owns
everything one FollowJointTrajectory move needs:
  * /joint_states (the arm's joints, UR order) and the F/T wrench, with a bias that is
    re-measured on demand and an optional trace of the bias-free force while a move runs;
  * the action client on `scaled_joint_trajectory_controller` (no Servo, no controller
    switching);
  * the gates every goal passes: the first point within `max_joint_jump_rad` of the
    current joints, |F| above `abort_force_n` cancels whatever runs (above
    `release_abort_force_n` for moves AWAY from a contact, which a stopped pen pressing
    on a rigid part would otherwise block), an abort request cancels too;
  * the dry run: no goal is sent, and `where()` then follows the end of the last
    pretended move, since /joint_states never will.

It was the execution half of tack_marking_node.py (2026-09-24 .. 09-29) and moved here
on 2026-10-01 so the ICP node's multi-view refinement (notes/multiview_refine_plan.md,
step 1) drives the arm with the same safety gates.

Parameters are declared on the owning node under `prefix` and read at every use, so
`ros2 param set` takes effect on the next move. The marking node uses prefix '' (the
names it always had: dry_run, abort_force_n, ...), the ICP node 'mv_'.
"""

from __future__ import annotations

import threading
import time
from typing import Any, Optional, Sequence

import numpy as np
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import WrenchStamped
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .tack_reach import UR_ORDER, joint_state_to_ur_order

# name -> default; declared as prefix + name on the owning node
PARAMS: dict[str, Any] = {
    'dry_run': True,
    'action': '/scaled_joint_trajectory_controller/follow_joint_trajectory',
    'wrench_topic': '/force_torque_sensor_broadcaster/wrench',
    'joint_states_topic': '/joint_states',
    'abort_force_n': 8.0,
    'release_abort_force_n': 40.0,     # retract (moving away) is never blocked by abort_force_n
    'bias_window_s': 0.5,
    'max_joint_jump_rad': 0.35,
}

Timed = Sequence[tuple[np.ndarray, float]]


class TrajectoryExecutor:
    """Send joint trajectories and watch the wrench while they run (see the module doc).

    `touch_force_n`: the contact threshold for `execute(..., watch_touch=True)`, the pen's
    touch force for the marking node; settable at any time. `say`: where the executor's
    own messages go (the marking node also publishes them on ~/status); default the
    node's logger.
    """

    def __init__(self, node: Node, prefix: str = '', touch_force_n: float = 1.5,
                 callback_group=None, say=None) -> None:
        self._node = node
        self._prefix = prefix
        self._say = say or (lambda text: node.get_logger().info(text))
        for name, default in PARAMS.items():
            if not node.has_parameter(prefix + name):
                node.declare_parameter(prefix + name, default)
        self.touch_force_n = float(touch_force_n)

        self._q: Optional[np.ndarray] = None
        self._q_sim: Optional[np.ndarray] = None       # where the DRY RUN pretends the arm is
        self._q_lock = threading.Lock()
        self._force = np.zeros(3)
        self._force_bias = np.zeros(3)
        self._force_trace: Optional[list] = None       # bias-free samples while tracing
        self._force_lock = threading.Lock()
        self._goal_handle = None
        self._abort = False

        cbg = callback_group or ReentrantCallbackGroup()
        node.create_subscription(JointState, str(self.param('joint_states_topic')),
                                 self._on_joints, qos_profile_sensor_data, callback_group=cbg)
        node.create_subscription(WrenchStamped, str(self.param('wrench_topic')),
                                 self._on_wrench, qos_profile_sensor_data, callback_group=cbg)
        self._client = ActionClient(node, FollowJointTrajectory, str(self.param('action')),
                                    callback_group=cbg)

    # ------------------------------------------------------------- parameters -------
    def param(self, name: str) -> Any:
        return self._node.get_parameter(self._prefix + name).value

    @property
    def dry_run(self) -> bool:
        return bool(self.param('dry_run'))

    # ------------------------------------------------------------- joints -----------
    def _on_joints(self, msg: JointState) -> None:
        if all(n in msg.name for n in UR_ORDER):
            with self._q_lock:
                self._q = joint_state_to_ur_order(msg.name, msg.position)

    def current_q(self) -> Optional[np.ndarray]:
        """The real joints from /joint_states (None before the first message)."""
        with self._q_lock:
            return None if self._q is None else self._q.copy()

    def where(self) -> Optional[np.ndarray]:
        """The arm's joints for planning and gating: the real ones, or in a dry run the
        end of the last pretended motion (nothing moves, so /joint_states never follows)."""
        if self.dry_run and self._q_sim is not None:
            return self._q_sim.copy()
        return self.current_q()

    def set_sim(self, q) -> None:
        """Where a dry run starts pretending from (e.g. the joints a plan was made at)."""
        self._q_sim = None if q is None else np.asarray(q, float).copy()

    # ------------------------------------------------------------- force ------------
    def _on_wrench(self, msg: WrenchStamped) -> None:
        f = msg.wrench.force
        with self._force_lock:
            self._force = np.array([f.x, f.y, f.z])
            if self._force_trace is not None:
                self._force_trace.append(self._force - self._force_bias)

    def force_mag(self) -> float:
        """|F| with the bias removed."""
        with self._force_lock:
            return float(np.linalg.norm(self._force - self._force_bias))

    def force_vector(self) -> np.ndarray:
        """F with the bias removed (sensor frame)."""
        with self._force_lock:
            return (self._force - self._force_bias).copy()

    def measure_bias(self, window_s: Optional[float] = None) -> None:
        """Average the raw force over `window_s` (default the bias_window_s parameter)
        and subtract it from now on: call it with the arm still and nothing touching."""
        samples = []
        t_end = time.time() + float(self.param('bias_window_s') if window_s is None else window_s)
        while time.time() < t_end:
            with self._force_lock:
                samples.append(self._force.copy())
            time.sleep(0.005)
        with self._force_lock:
            self._force_bias = np.mean(samples, axis=0) if samples else np.zeros(3)

    def start_trace(self) -> None:
        with self._force_lock:
            self._force_trace = []

    def stop_trace(self) -> np.ndarray:
        """The bias-free force samples since `start_trace`, (n, 3)."""
        with self._force_lock:
            trace = np.array(self._force_trace) if self._force_trace else np.zeros((0, 3))
            self._force_trace = None
        return trace

    # ------------------------------------------------------------- abort ------------
    def abort(self) -> None:
        """Cancel the running goal and refuse to continue it (until `clear_abort`)."""
        self._abort = True
        self.cancel()

    def clear_abort(self) -> None:
        self._abort = False

    @property
    def aborted(self) -> bool:
        return self._abort

    def cancel(self) -> None:
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass

    # ------------------------------------------------------------- execute ----------
    def execute(self, timed: Timed, label: str, watch_touch: bool = False,
                want_contact: bool = False, away: bool = False,
                sim_contact_fraction: float = 1.0, rebias: bool = False):
        """Send one trajectory goal and wait; with `watch_touch` cancel at the touch force.

        `timed`: [(q, t_from_start)], UR joint order. `away`: a move away from a contact
        (only `release_abort_force_n` stops it). `sim_contact_fraction`: in a dry run with
        `want_contact`, the pretended contact is that fraction of the way along `timed`
        (the marking node passes standoff / (standoff + overshoot): the registered surface).
        `rebias`: re-measure the force bias right before sending, for a move that starts at
        rest and touching nothing (a transit, home). The sensor's zero drifts, e.g. by 7 N
        after a power cycle (2026-10-01); with a bias from minutes ago, or none yet, that
        offset alone reaches `abort_force_n`. Never for a move that starts in contact
        (retract, stroke): it would zero the contact force.

        Returns (ok, msg) or, with `want_contact`, (ok, msg, (q_contact, force) | None).
        """
        def out(ok, msg, contact=None):
            return (ok, msg, contact) if want_contact else (ok, msg)

        q = self.where()
        if q is None:
            return out(False, 'no joint states')
        jump = float(np.abs(np.asarray(timed[0][0], float) - q).max())
        if jump > float(self.param('max_joint_jump_rad')):
            return out(False, f'{label}: first point is {np.degrees(jump):.0f} deg from the current '
                              f'joints - refused')
        if self.dry_run:
            msg = f'{label}: dry run, {len(timed)} points, {timed[-1][1]:.1f} s'
            self._say(msg)
            if want_contact:
                k = int(round((len(timed) - 1) * float(sim_contact_fraction)))
                self._q_sim = np.asarray(timed[k][0], float).copy()
                return out(True, msg, (np.asarray(timed[k][0], float), self.touch_force_n))
            self._q_sim = np.asarray(timed[-1][0], float).copy()
            return out(True, msg)

        if rebias:
            self.measure_bias()
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = list(UR_ORDER)
        for qk, tk in timed:
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in qk]
            sec = int(tk)
            pt.time_from_start = Duration(sec=sec, nanosec=int((tk - sec) * 1e9))
            traj.points.append(pt)
        goal.trajectory = traj
        if not self._client.wait_for_server(timeout_sec=2.0):
            return out(False, f'{label}: action server not available')
        send = self._client.send_goal_async(goal)
        while not send.done():
            time.sleep(0.005)
        self._goal_handle = send.result()
        if not self._goal_handle.accepted:
            return out(False, f'{label}: goal REJECTED by the controller before any motion. It prints '
                              f'the reason in the ur_control terminal; the usual one after using the '
                              f'pendant is that the External Control program is not running (press '
                              f'Play on the pendant), else the controller is inactive or the '
                              f'trajectory is malformed.')
        result_fut = self._goal_handle.get_result_async()
        contact = None
        # a move AWAY from the surface (retract) must not be cancelled by the force of the
        # contact it is releasing: on a rigid part a stopped pen presses at ~10 N
        abort_force = float(self.param('release_abort_force_n' if away else 'abort_force_n'))
        while not result_fut.done():
            f = self.force_mag()
            if self._abort or f > abort_force or (watch_touch and f > self.touch_force_n):
                q_c = self.current_q()
                self.cancel()
                if watch_touch and f > self.touch_force_n and not self._abort and f <= abort_force:
                    contact = (q_c, f)
                    break
                if self._abort:
                    return out(False, f'{label}: cancelled (abort)')
                return out(False, f'{label}: cancelled (force {f:.1f} N over the {abort_force:g} N limit, '
                                  f'bias removed). Touching nothing? Then the reading drifts during the '
                                  f'move: check the payload (mass, CoG) on the pendant, zero the sensor '
                                  f'(ros2 service call /io_and_status_controller/zero_ftsensor '
                                  f'std_srvs/srv/Trigger) with the arm still')
            time.sleep(0.002)
        if contact is None:
            res = result_fut.result()
            code = res.result.error_code if res is not None else -99
            ok = code == FollowJointTrajectory.Result.SUCCESSFUL
            return out(ok, f'{label}: {"done" if ok else f"controller error {code}"}')
        # no settling pause: the caller releases the pen (a stopped pen on a rigid part
        # keeps pressing; the joints at the cancel are already the contact record)
        return out(True, f'{label}: contact at {contact[1]:.2f} N', contact)
