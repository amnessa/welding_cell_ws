"""Finger-pointing camera aim: the control law (admittance_control/camera_aim.py). Plain
pytest, no ROS: the closed loop runs on the UR kinematics with a 30 Hz camera whose
images arrive 0.1 s late and an arm that lags its rate command."""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

PKG = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

from admittance_control.camera_aim import (AimConfig, CameraAim, JointHistory,  # noqa: E402
                                           OneEuroFilter, RateSmoother, aim_rates,
                                           check_ahead, joint_rates, limit_excursion,
                                           pan_tilt, pixel_ray, tool0_twist,
                                           twist_about_axis)
from admittance_control.kinematics import ur5e_fk, ur5e_jacobian  # noqa: E402
from admittance_control.tack_reach import load_marking_config  # noqa: E402

CFG = AimConfig()
K = np.array([[615.0, 0.0, 320.0], [0.0, 615.0, 240.0], [0.0, 0.0, 1.0]])
DEADBAND = CFG.deadband_frac * 640 / 615.0
HOME_Q = load_marking_config().home_q
T_TOOL0_CAM = np.load(PKG / "notebooks" / "T_tcp_to_cam.npy")


def _R(axis: str, deg: float) -> np.ndarray:
    return Rotation.from_euler(axis, deg, degrees=True).as_matrix()


# ---------------------------------------------------------------- the pieces ----------
def test_rates_turn_the_optical_axis_towards_the_fingertip():
    # fingertip right of centre: the axis turns right (+x); below: it turns down (+y)
    for ex, ey in [(0.2, 0.0), (0.0, 0.2), (-0.15, -0.1)]:
        w, engaged = aim_rates(ex, ey, 0.0, DEADBAND, False, CFG)
        assert engaged and w[2] == 0.0
        z_after = Rotation.from_rotvec(w * 0.01).as_matrix()[:, 2]
        assert np.sign(z_after[0]) == np.sign(ex) or ex == 0.0
        assert np.sign(z_after[1]) == np.sign(ey) or ey == 0.0


def test_deadband_has_hysteresis_and_the_rate_is_capped():
    inside = 0.8 * DEADBAND
    w, engaged = aim_rates(inside, 0.0, 0.0, DEADBAND, False, CFG)
    assert not engaged and not np.any(w)                     # idle and inside: stays put
    w, engaged = aim_rates(inside, 0.0, 0.0, DEADBAND, True, CFG)
    assert engaged and w[1] > 0                              # moving: carries on inside
    w, engaged = aim_rates(0.3 * DEADBAND, 0.0, 0.0, DEADBAND, True, CFG)
    assert not engaged and not np.any(w)                     # ... until the inner radius
    w, _ = aim_rates(1.0, 1.0, 0.0, DEADBAND, False, CFG)
    assert np.linalg.norm(w[:2]) == pytest.approx(CFG.max_rate)


def test_roll_is_measured_about_the_optical_axis_only():
    R_home = _R("xyz", [20.0, -30.0, 70.0])
    assert math.degrees(twist_about_axis(R_home, R_home @ _R("z", 10.0))) == pytest.approx(10.0)
    assert abs(twist_about_axis(R_home, R_home @ _R("x", 20.0))) < 1e-9     # a pure tilt
    assert abs(twist_about_axis(R_home, R_home @ _R("y", 25.0))) < 1e-9     # a pure pan
    w, _ = aim_rates(0.0, 0.0, math.radians(5.0), DEADBAND, False, CFG)
    assert w[2] < 0                                           # rolled +: turned back


def test_excursion_limit_blocks_outward_and_keeps_inward():
    R_home = np.eye(3)
    R_edge = _R("y", CFG.pan_limit_deg - 1.0)                # 1 deg inside the pan limit
    p, t = pan_tilt(R_home, R_edge)
    assert math.degrees(p) == pytest.approx(CFG.pan_limit_deg - 1.0) and abs(t) < 1e-9
    out = np.array([0.0, CFG.max_rate, 0.0])                  # further right
    w, held = limit_excursion(out, R_home, R_edge, CFG)
    assert w[1] == 0.0 and held == ["pan"]
    w, held = limit_excursion(-out, R_home, R_edge, CFG)      # back towards home
    assert w[1] == -CFG.max_rate and held == []


def test_smoother_is_smooth_and_does_not_overshoot():
    sm = RateSmoother(CFG.smooth_tau_s, CFG.max_acc)
    target = np.array([0.0, CFG.max_rate, 0.0])
    dt, prev, prev_acc, ws = 0.02, 0.0, 0.0, []
    for _ in range(100):
        w = sm.step(target, dt)[1]
        acc = (w - prev) / dt
        assert acc <= CFG.max_acc + 1e-9 and w <= CFG.max_rate + 1e-12
        prev, prev_acc = w, acc
        ws.append(w)
    assert ws[-1] == pytest.approx(CFG.max_rate, rel=1e-3)
    assert all(b >= a - 1e-12 for a, b in zip(ws, ws[1:]))  # monotonic: no overshoot
    assert ws[0] < 0.2 * CFG.max_rate                         # a soft start, not a step


def test_one_euro_is_steady_when_still_and_follows_a_move():
    rng = np.random.default_rng(0)
    f = OneEuroFilter(CFG.one_euro_min_cutoff_hz, CFG.one_euro_beta, CFG.one_euro_d_cutoff_hz)
    out = [f(i / 30.0, [0.003 * rng.normal()]) for i in range(60)]
    assert np.std(out[20:]) < 0.4 * 0.003                     # 2 px of jitter mostly gone
    f.reset()
    out = [f(i / 30.0, [0.0 if i < 10 else 0.2]) for i in range(40)]
    assert out[-1][0] == pytest.approx(0.2, abs=0.01)         # a moved hand is followed


def test_tool0_twist_keeps_the_optical_centre_still():
    T = ur5e_fk(HOME_Q)
    C = T @ T_TOOL0_CAM
    w = np.array([0.1, -0.2, 0.05])
    tw = tool0_twist(w, T[:3, 3], C[:3, 3], C[:3, 3], CFG)
    v_cam = tw[:3] + np.cross(tw[3:], C[:3, 3] - T[:3, 3])
    assert np.linalg.norm(v_cam) < 1e-12
    qd = joint_rates(HOME_Q, tw, CFG)
    assert np.allclose(ur5e_jacobian(HOME_Q) @ qd, tw, atol=2e-3)
    # ... and the arm does it: a small step along qd turns the camera at w about its
    # optical centre (the Jacobian's angular rows were doubled until 2026-10-22, and the
    # camera turned at w / 2 while its centre wandered)
    h = 1e-5
    C1 = ur5e_fk(HOME_Q + h * qd) @ T_TOOL0_CAM
    assert np.allclose(Rotation.from_matrix(C1[:3, :3] @ C[:3, :3].T).as_rotvec() / h, w,
                       atol=2e-3)
    assert np.linalg.norm(C1[:3, 3] - C[:3, 3]) / h < 2e-3


def test_check_ahead_names_the_refusal():
    qd = np.zeros(6)
    qd[4] = -1.943 / CFG.check_ahead_s                       # wrist_2 to zero
    assert "wrist" in check_ahead(HOME_Q, qd, CFG)
    qd = np.zeros(6)
    qd[2] = 2.0 / CFG.check_ahead_s                          # the elbow flips its sign
    assert "branch" in check_ahead(HOME_Q, qd, CFG, signature=(-1, -1, 1))
    qd = np.full(6, 0.01)
    assert "collision" in check_ahead(HOME_Q, qd, CFG, is_valid=lambda q: False)
    assert check_ahead(HOME_Q, qd, CFG, is_valid=lambda q: True, signature=(-1, -1, 1)) is None


def test_joint_history_interpolates_and_refuses_gaps():
    h = JointHistory()
    h.add(10.0, np.zeros(6))
    h.add(10.1, np.ones(6))
    assert np.allclose(h.at(10.05), 0.5)
    assert np.allclose(h.at(10.2), 1.0)                       # the newest, within the gap
    assert h.at(11.0) is None and h.at(5.0) is None           # clocks that do not overlap


# ---------------------------------------------------------------- the closed loop -----
def _run(target_px, latency=0.10, T=6.0, compensate=True, seed=0):
    """A still fingertip at `target_px` of the home view, 0.55 m out; returns per-cycle
    (t, pixel error, commanded rate, drift, roll, pan, achieved rate) and the aim."""
    rng = np.random.default_rng(seed)
    aim = CameraAim(CFG, T_TOOL0_CAM, HOME_Q)
    q = HOME_Q.copy()
    qd_arm = np.zeros(6)
    C0 = aim.camera_pose(q)
    P = C0[:3, :3] @ pixel_ray(*target_px, K) * 0.55 + C0[:3, 3]
    hist, pending, rows = JointHistory(), [], []
    aim.start(q)
    dt, t, next_img = 0.02, 0.0, 0.0
    while t < T:
        hist.add(t, q)
        if t >= next_img:                                     # a frame, delivered late
            C = aim.camera_pose(q)
            d = C[:3, :3].T @ (P - C[:3, 3])
            u = K[0, 0] * d[0] / d[2] + K[0, 2] + rng.normal(0, 1.5)
            v = K[1, 1] * d[1] / d[2] + K[1, 2] + rng.normal(0, 1.5)
            pending.append((t + latency, t, u, v))
            next_img += 1 / 30.0
        while pending and pending[0][0] <= t:
            _, ts, u, v = pending.pop(0)
            aim.observe(ts if compensate else t, pixel_ray(u, v, K),
                        hist.at(ts) if compensate else q)
        qd, info = aim.step(q, dt, DEADBAND)
        qd_arm += (dt / (0.03 + dt)) * (qd - qd_arm)          # the arm lags 30 ms
        R_before = aim.camera_pose(q)[:3, :3]
        q = q + qd_arm * dt
        t += dt
        C = aim.camera_pose(q)
        d = C[:3, :3].T @ (P - C[:3, 3])
        err = math.hypot(K[0, 0] * d[0] / d[2], K[1, 1] * d[1] / d[2])
        achieved = math.degrees(np.linalg.norm(
            Rotation.from_matrix(C[:3, :3] @ R_before.T).as_rotvec())) / dt
        rows.append((t, err, info.rate_deg_s, info.drift_mm, info.roll_deg, info.pan_deg,
                     achieved))
    return rows, aim


@pytest.mark.parametrize("target", [(560, 240), (320, 60), (100, 420)])
def test_the_camera_centres_a_still_fingertip(target):
    rows, aim = _run(target)
    t_in = next(r[0] for r in rows if r[1] < CFG.deadband_frac * 640)
    assert t_in < 4.0                                         # ~20 deg at 7.5 deg/s
    assert rows[-1][1] < CFG.deadband_frac * 640              # roughly centred ...
    assert rows[-1][2] < 0.05 and aim.stopped()               # ... and stopped there
    assert max(r[2] for r in rows) <= CFG.max_rate_deg_s + 1e-6
    # the arm turns the camera at the commanded rate, not half of it (until 2026-10-22)
    assert 0.9 * max(r[2] for r in rows) <= max(r[6] for r in rows) <= 1.05 * CFG.max_rate_deg_s
    assert max(abs(r[4]) for r in rows) < 0.5                 # roll held at home
    assert max(r[3] for r in rows) < 10.0                     # optical centre held (mm)


def test_latency_compensation_removes_the_overshoot():
    # 0.2 s of image latency: compensated, the pan never passes where it stops;
    # uncompensated it runs on past the target
    target = (560, 240)
    with_c, _ = _run(target, latency=0.2, compensate=True)
    without, _ = _run(target, latency=0.2, compensate=False)
    final = with_c[-1][5]
    assert max(r[5] for r in with_c) <= final + 0.05
    assert max(r[5] for r in without) > max(r[5] for r in with_c) + 0.3
