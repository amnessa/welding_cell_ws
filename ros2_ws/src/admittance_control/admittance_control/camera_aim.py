"""Finger-pointing camera aim: the control law, without ROS (notes/todo.md, HRI).

Before the first capture the operator points (MediaPipe `Pointing_Up`) and the wrist
camera turns until the index fingertip sits in the middle of the image. The camera turns
about its own optical centre - its position is held - so the operator steers WHERE IT
LOOKS and nothing else. scripts/camera_aim_node.py runs one `CameraAim.step` per control
cycle (50 Hz) and streams the joint rates to the UR driver's forward_velocity_controller.

One cycle, in order:
  1. the fingertip's ray, measured on an image ~0.1 s old, was turned into the BASE frame
     with the camera pose at that image's stamp (`CameraAim.observe`) and One-Euro
     filtered there. The camera only rotates about its optical centre, so a still
     fingertip has a still base-frame ray whatever its depth: the filter sees the hand's
     motion only, and turning the ray back into the CURRENT camera frame compensates the
     image latency exactly - nothing is predicted;
  2. the angular error to camera rates (`aim_rates`): w_x = -K e_y, w_y = K e_x, behind a
     deadband with hysteresis (move when the fingertip leaves ~5 % of the image, stop
     when it is back within 40 % of that), capped at ~15 deg/s. A P loop: an integral
     winds up over the ~0.1 s latency, a derivative of a hand landmark is noise;
  3. w_z holds the ROLL at home. Two rotations aim the optical axis; the third, about the
     axis itself, is the over-actuated one. Its twist relative to the home view
     (swing-twist about the optical axis, `twist_about_axis`) is driven to zero, so the
     image keeps its home orientation;
  4. the excursion limit (`limit_excursion`): pan and tilt of the optical axis from the
     home view, looked ahead by the stopping distance so the smoother stops AT the limit;
  5. two cascaded first-order lags with an acceleration cap (`RateSmoother`): continuous
     acceleration, smooth starts and stops, no overshoot of the commanded rate;
  6. the tool0 twist that keeps the optical centre where it was when the session started
     (`tool0_twist`, with a small P pull against integration drift);
  7. damped-least-squares joint rates, capped (`joint_rates`), and the safety look-ahead
     the node runs on them (`check_ahead`: collision model, branch lock, wrist).
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass
from typing import Callable, Optional, Sequence

import numpy as np
from scipy.spatial.transform import Rotation

from .kinematics import ur5e_fk, ur5e_jacobian
from .tack_reach import branch_signature


@dataclass
class AimConfig:
    #: camera rate (rad/s) per rad of fingertip error. The latency is compensated (see
    #: the module doc), so the loop's lag is the smoother and the arm, ~0.15 s; 2.5 /s
    #: keeps a ~50 deg phase margin even if the compensation is off (0.25 s)
    gain_per_s: float = 2.5
    deadband_frac: float = 0.05          # of the image width: start moving beyond this
    release_frac: float = 0.4            # ... and stop within this fraction of it
    max_rate_deg_s: float = 15.0         # the aiming speed cap
    max_acc_deg_s2: float = 60.0
    smooth_tau_s: float = 0.06           # each of the two lags
    roll_gain_per_s: float = 1.5
    pan_limit_deg: float = 35.0          # from the home view
    tilt_limit_deg: float = 25.0
    limit_lookahead_s: float = 0.35      # ~ the stopping time from max_rate
    pos_gain_per_s: float = 2.0          # optical-centre hold
    max_pos_speed_m_s: float = 0.02
    dls_lambda: float = 0.02
    max_joint_speed_rad_s: float = 0.5
    check_ahead_s: float = 0.3           # where the safety checks look
    wrist_margin_rad: float = 0.35       # |sin(wrist_2)| >= sin(margin)
    min_ray_z: float = 0.3               # a ray this far off the optical axis is ignored
    one_euro_min_cutoff_hz: float = 1.0
    one_euro_beta: float = 5.0           # Hz per rad/s of hand motion
    one_euro_d_cutoff_hz: float = 1.0

    @property
    def max_rate(self) -> float:
        return math.radians(self.max_rate_deg_s)

    @property
    def max_acc(self) -> float:
        return math.radians(self.max_acc_deg_s2)


# ---------------------------------------------------------------- filters -------------
class OneEuroFilter:
    """Casiez et al.'s One-Euro filter on a vector: a low-pass whose cutoff rises with
    the signal's speed (steady when still, little lag when it moves)."""

    def __init__(self, min_cutoff_hz: float = 1.0, beta: float = 5.0,
                 d_cutoff_hz: float = 1.0) -> None:
        self.min_cutoff = float(min_cutoff_hz)
        self.beta = float(beta)
        self.d_cutoff = float(d_cutoff_hz)
        self.reset()

    def reset(self) -> None:
        self._x: Optional[np.ndarray] = None
        self._dx: Optional[np.ndarray] = None
        self._t: Optional[float] = None

    @staticmethod
    def _alpha(cutoff_hz: float, dt: float) -> float:
        tau = 1.0 / (2.0 * math.pi * cutoff_hz)
        return 1.0 / (1.0 + tau / dt)

    def __call__(self, t: float, x: Sequence[float]) -> np.ndarray:
        x = np.asarray(x, float)
        if self._x is None or self._t is None or t <= self._t:
            if self._x is None or self._t is None:
                self._x, self._dx, self._t = x.copy(), np.zeros_like(x), float(t)
            return self._x.copy()
        dt = float(t) - self._t
        dx = (x - self._x) / dt
        self._dx = self._dx + self._alpha(self.d_cutoff, dt) * (dx - self._dx)
        cutoff = self.min_cutoff + self.beta * float(np.linalg.norm(self._dx))
        self._x = self._x + self._alpha(cutoff, dt) * (x - self._x)
        self._t = float(t)
        return self._x.copy()


class RateSmoother:
    """Two cascaded first-order lags with an acceleration cap: the commanded rate follows
    the target with continuous acceleration and without overshoot."""

    def __init__(self, tau_s: float, max_acc: float, n: int = 3) -> None:
        self.tau = float(tau_s)
        self.max_acc = float(max_acc)
        self.n = n
        self.reset()

    def reset(self) -> None:
        self.w1 = np.zeros(self.n)
        self.w = np.zeros(self.n)

    def step(self, target: np.ndarray, dt: float) -> np.ndarray:
        a = dt / (self.tau + dt)
        self.w1 = self.w1 + a * (np.asarray(target, float) - self.w1)
        dw = a * (self.w1 - self.w)
        n = float(np.linalg.norm(dw))
        cap = self.max_acc * dt
        if n > cap:
            dw *= cap / n
        self.w = self.w + dw
        return self.w.copy()


# ---------------------------------------------------------------- geometry ------------
def pixel_ray(u: float, v: float, K: np.ndarray) -> np.ndarray:
    """The unit ray of pixel (u, v) in the optical frame (x right, y down, z forward)."""
    d = np.array([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], 1.0])
    return d / np.linalg.norm(d)


def pan_tilt(R_home: np.ndarray, R: np.ndarray) -> tuple[float, float]:
    """(pan, tilt) in rad of the optical axis of `R` seen from the home camera frame:
    pan about the home y axis (+ = towards image right), tilt about its x axis
    (+ = towards image up)."""
    d = np.asarray(R_home, float).T @ np.asarray(R, float)[:, 2]
    return math.atan2(d[0], d[2]), math.atan2(-d[1], math.hypot(d[0], d[2]))


def twist_about_axis(R_home: np.ndarray, R: np.ndarray) -> float:
    """The roll (rad) of `R` about its optical axis relative to the home view: the twist
    of R_home^T R about z in a swing-twist split (R_rel = swing * twist)."""
    x, y, z, w = Rotation.from_matrix(np.asarray(R_home, float).T @ np.asarray(R, float)).as_quat()
    return 2.0 * math.atan2(z, w) if (z * z + w * w) > 1e-12 else 0.0


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def aim_rates(ex: float, ey: float, roll: float, deadband: float, engaged: bool,
              cfg: AimConfig) -> tuple[np.ndarray, bool]:
    """Camera-frame angular rate (rad/s) for the fingertip error (ex, ey) - tangents in the
    current image, x right, y down - and the roll from home; returns (w_cam, engaged).
    Outside `deadband` it engages; it then keeps going until the error is within
    `cfg.release_frac` of it. The rate is measured from half that inner radius - from the
    radius itself it would creep towards it for ever and never let go."""
    n = math.hypot(ex, ey)
    inner = cfg.release_frac * deadband
    engaged = (n > deadband) or (engaged and n > inner)
    w = np.zeros(3)
    if engaged:
        s = (n - 0.5 * inner) / n
        w[0] = -cfg.gain_per_s * ey * s           # +w_x turns the optical axis up (-y)
        w[1] = cfg.gain_per_s * ex * s            # +w_y turns it right (+x)
        m = float(np.linalg.norm(w[:2]))
        if m > cfg.max_rate:
            w[:2] *= cfg.max_rate / m
    w[2] = float(np.clip(-cfg.roll_gain_per_s * _wrap(roll), -cfg.max_rate, cfg.max_rate))
    return w, engaged


def limit_excursion(w_cam: np.ndarray, R_home: np.ndarray, R_now: np.ndarray,
                    cfg: AimConfig) -> tuple[np.ndarray, list[str]]:
    """`w_cam` with the pan / tilt components that would carry the optical axis past the
    limits within `limit_lookahead_s` removed (a component that brings it back is kept).
    Returns (w, which limits held)."""
    out = np.asarray(w_cam, float).copy()
    pl, tl = math.radians(cfg.pan_limit_deg), math.radians(cfg.tilt_limit_deg)
    p0, t0 = pan_tilt(R_home, R_now)
    held = []
    for i in (0, 1):
        if out[i] == 0.0:
            continue
        step = np.zeros(3)
        step[i] = out[i] * cfg.limit_lookahead_s
        p1, t1 = pan_tilt(R_home, R_now @ Rotation.from_rotvec(step).as_matrix())
        if abs(p1) > pl and abs(p1) > abs(p0):
            out[i] = 0.0
            held.append("pan")
        elif abs(t1) > tl and abs(t1) > abs(t0):
            out[i] = 0.0
            held.append("tilt")
    return out, held


def tool0_twist(w_world: np.ndarray, p_tool0: np.ndarray, p_cam: np.ndarray,
                p_anchor: np.ndarray, cfg: AimConfig) -> np.ndarray:
    """[v; w] of tool0 (base frame) that turns the camera at `w_world` about its optical
    centre, pulled back to `p_anchor` against drift."""
    v_cam = cfg.pos_gain_per_s * (np.asarray(p_anchor, float) - np.asarray(p_cam, float))
    n = float(np.linalg.norm(v_cam))
    if n > cfg.max_pos_speed_m_s:
        v_cam *= cfg.max_pos_speed_m_s / n
    w = np.asarray(w_world, float)
    v_tool0 = v_cam + np.cross(w, np.asarray(p_tool0, float) - np.asarray(p_cam, float))
    return np.concatenate([v_tool0, w])


def joint_rates(q: np.ndarray, twist: np.ndarray, cfg: AimConfig) -> np.ndarray:
    """Damped least squares q_dot = J^T (J J^T + l^2 I)^-1 x_dot, scaled down as a whole
    when a joint would exceed `max_joint_speed_rad_s` (the direction is kept)."""
    J = ur5e_jacobian(np.asarray(q, float))
    A = J @ J.T + (cfg.dls_lambda ** 2) * np.eye(6)
    qd = J.T @ np.linalg.solve(A, np.asarray(twist, float))
    m = float(np.max(np.abs(qd)))
    if m > cfg.max_joint_speed_rad_s:
        qd *= cfg.max_joint_speed_rad_s / m
    return qd


def check_ahead(q: np.ndarray, qd: np.ndarray, cfg: AimConfig,
                is_valid: Optional[Callable[[np.ndarray], bool]] = None,
                signature: Optional[tuple[int, ...]] = None,
                branch_joints: Sequence[int] = (1, 2, 4)) -> Optional[str]:
    """Why the pose `check_ahead_s` along `qd` is refused, None when it is fine: the
    collision model (`is_valid`), the branch lock, the wrist singularity."""
    if not np.any(qd):
        return None
    q1 = np.asarray(q, float) + np.asarray(qd, float) * cfg.check_ahead_s
    if abs(math.sin(q1[4])) < math.sin(cfg.wrist_margin_rad):
        return "wrist singularity ahead"
    if signature is not None and branch_signature(q1, branch_joints) != tuple(signature):
        return "would leave the locked branch"
    if is_valid is not None and not is_valid(q1):
        return "collision ahead (table, fixtures or the arm itself)"
    return None


# ---------------------------------------------------------------- joint history -------
class JointHistory:
    """Timestamped joint positions, for the camera pose at an image's stamp."""

    def __init__(self, keep_s: float = 2.0) -> None:
        self.keep_s = keep_s
        self._t: list[float] = []
        self._q: list[np.ndarray] = []

    def add(self, t: float, q: np.ndarray) -> None:
        if self._t and t <= self._t[-1]:
            return
        self._t.append(float(t))
        self._q.append(np.asarray(q, float).copy())
        while self._t and self._t[0] < t - self.keep_s:
            self._t.pop(0)
            self._q.pop(0)

    def at(self, t: float, max_gap_s: float = 0.25) -> Optional[np.ndarray]:
        """q at time `t` (linear between samples, the end sample up to `max_gap_s`
        beyond it); None outside the history - mismatched clocks, a stale stream."""
        if not self._t:
            return None
        if t <= self._t[0]:
            return self._q[0].copy() if self._t[0] - t <= max_gap_s else None
        if t >= self._t[-1]:
            return self._q[-1].copy() if t - self._t[-1] <= max_gap_s else None
        i = bisect.bisect_right(self._t, t)
        t0, t1 = self._t[i - 1], self._t[i]
        a = (t - t0) / (t1 - t0)
        return (1.0 - a) * self._q[i - 1] + a * self._q[i]


# ---------------------------------------------------------------- the session ---------
@dataclass
class StepInfo:
    error: Optional[tuple[float, float]] = None   # (ex, ey) in the current image
    pan_deg: float = 0.0
    tilt_deg: float = 0.0
    roll_deg: float = 0.0
    rate_deg_s: float = 0.0                        # |commanded camera rate|
    drift_mm: float = 0.0                          # optical centre from its anchor
    held: tuple[str, ...] = ()
    engaged: bool = False


class CameraAim:
    """One aiming session's state: the filtered fingertip ray, the smoother, the anchor.

        aim = CameraAim(cfg, T_tool0_cam, q_home)
        aim.start(q)                                   # anchor the optical centre here
        aim.observe(t_img, ray_cam, q_at_image)        # per image with a fingertip
        aim.lose()                                     # the hand went away
        qd, info = aim.step(q, dt, deadband)           # per control cycle
    """

    def __init__(self, cfg: AimConfig, T_tool0_cam: np.ndarray, q_home: np.ndarray) -> None:
        self.cfg = cfg
        self.T_tool0_cam = np.asarray(T_tool0_cam, float)
        self.q_home = np.asarray(q_home, float)
        T_home = self.camera_pose(self.q_home)
        self.R_home = T_home[:3, :3]
        self.p_home = T_home[:3, 3]
        self.filter = OneEuroFilter(cfg.one_euro_min_cutoff_hz, cfg.one_euro_beta,
                                    cfg.one_euro_d_cutoff_hz)
        self.smoother = RateSmoother(cfg.smooth_tau_s, cfg.max_acc)
        self.ray_base: Optional[np.ndarray] = None
        self.anchor: Optional[np.ndarray] = None
        self.engaged = False

    def camera_pose(self, q: np.ndarray) -> np.ndarray:
        return ur5e_fk(np.asarray(q, float)) @ self.T_tool0_cam

    def distance_from_home_m(self, q: np.ndarray) -> float:
        return float(np.linalg.norm(self.camera_pose(q)[:3, 3] - self.p_home))

    def start(self, q: np.ndarray) -> None:
        self.anchor = self.camera_pose(q)[:3, 3].copy()
        self.smoother.reset()
        self.filter.reset()
        self.ray_base = None
        self.engaged = False

    def observe(self, t: float, ray_cam: np.ndarray, q_at_image: np.ndarray) -> None:
        R_img = self.camera_pose(q_at_image)[:3, :3]
        r = self.filter(t, R_img @ (np.asarray(ray_cam, float) / np.linalg.norm(ray_cam)))
        self.ray_base = r / np.linalg.norm(r)

    def lose(self) -> None:
        self.ray_base = None
        self.filter.reset()
        self.engaged = False

    def stopped(self, tol: float = 1e-3) -> bool:
        return float(np.linalg.norm(self.smoother.w)) < tol and \
            float(np.linalg.norm(self.smoother.w1)) < tol

    def step(self, q: np.ndarray, dt: float, deadband: float) -> tuple[np.ndarray, StepInfo]:
        T_tool0 = ur5e_fk(np.asarray(q, float))
        T_cam = T_tool0 @ self.T_tool0_cam
        R = T_cam[:3, :3]
        info = StepInfo()
        pan, tilt = pan_tilt(self.R_home, R)
        roll = twist_about_axis(self.R_home, R)
        info.pan_deg, info.tilt_deg, info.roll_deg = map(math.degrees, (pan, tilt, roll))

        ex = ey = 0.0
        have = False
        if self.ray_base is not None:
            d = R.T @ self.ray_base
            if d[2] >= self.cfg.min_ray_z:
                ex, ey, have = d[0] / d[2], d[1] / d[2], True
                info.error = (ex, ey)
        if have:
            w, self.engaged = aim_rates(ex, ey, roll, deadband, self.engaged, self.cfg)
        else:
            self.engaged = False
            w, _ = aim_rates(0.0, 0.0, roll, deadband, False, self.cfg)
        w, held = limit_excursion(w, self.R_home, R, self.cfg)
        info.held, info.engaged = tuple(held), self.engaged

        w_cmd = self.smoother.step(w, dt)
        info.rate_deg_s = math.degrees(float(np.linalg.norm(w_cmd)))
        anchor = self.anchor if self.anchor is not None else T_cam[:3, 3]
        info.drift_mm = 1000.0 * float(np.linalg.norm(T_cam[:3, 3] - anchor))
        twist = tool0_twist(R @ w_cmd, T_tool0[:3, 3], T_cam[:3, 3], anchor, self.cfg)
        return joint_rates(q, twist, self.cfg), info
