"""Smooth transits for the marking node (2026-10-05): the drawing server's spline through
the shortcut path, re-checked against the collision model, and its timing fallback.

Pure numpy; the T-joint scene comes from test_marking.
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

PKG = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))
sys.path.insert(0, str(PKG / "test"))

from admittance_control import marking as mk  # noqa: E402
from admittance_control.kinematics import _edge_valid  # noqa: E402

from test_marking import scene  # noqa: E402,F401  (fixture)


def _max_turn_deg(path):
    """The largest direction change between consecutive segments of a joint path."""
    d = [np.asarray(b) - np.asarray(a) for a, b in zip(path[:-1], path[1:])]
    d = [v / np.linalg.norm(v) for v in d if np.linalg.norm(v) > 1e-12]
    return max((np.degrees(np.arccos(np.clip(a @ b, -1, 1))) for a, b in zip(d[:-1], d[1:])),
               default=0.0)


CORNER = [np.zeros(6), np.array([0.8, 0, 0, 0, 0, 0]), np.array([0.8, 0.8, 0, 0, 0, 0])]


def test_spline_rounds_the_corner_and_keeps_the_ends():
    dense = mk.smooth_transit(CORNER, lambda q: True)
    assert dense is not None
    assert np.array_equal(dense[0], CORNER[0]) and np.array_equal(dense[-1], CORNER[-1])
    steps = [np.abs(b - a).max() for a, b in zip(dense[:-1], dense[1:])]
    assert max(steps) < 0.03                                     # sampled every ~0.02 rad
    assert _max_turn_deg(CORNER) == pytest.approx(90.0)
    assert _max_turn_deg(dense) < 15.0                           # no sharp corner left


def test_spline_is_refused_when_it_leaves_free_space():
    # free space = exactly the original polyline: the rounded corner cuts across it
    def on_polyline(q):
        return (abs(q[1]) < 1e-9 and -1e-9 <= q[0] <= 0.8 + 1e-9) or \
               (abs(q[0] - 0.8) < 1e-9 and -1e-9 <= q[1] <= 0.8 + 1e-9)
    assert mk.smooth_transit(CORNER, on_polyline) is None


def test_two_point_path_is_left_alone():
    p = [np.zeros(6), np.ones(6) * 0.1]
    out = mk.smooth_transit(p, lambda q: True)
    assert len(out) == 2 and np.array_equal(out[1], p[1])


def test_trapezoid_respects_speed_and_acceleration():
    v, a = 0.3, 0.5
    timed = mk.time_trapezoid(CORNER, v_max=v, a_max=a)
    qs = np.array([q for q, _ in timed]); ts = np.array([t for _, t in timed])
    assert ts[0] == 0.0 and np.all(np.diff(ts) > 0)
    assert np.array_equal(qs[0], CORNER[0]) and np.array_equal(qs[-1], CORNER[-1])
    speed = np.abs(np.diff(qs, axis=0)).max(axis=1) / np.diff(ts)
    assert speed.max() <= v * 1.02
    S = 1.6                                                      # 0.8 + 0.8 in the inf-norm
    assert ts[-1] == pytest.approx(S / v + v / a, rel=1e-6)      # 2 ramps of v/a + cruise
    # starts and ends slowly (a ramp, not a jump to v_max)
    assert speed[0] < 0.5 * v and speed[-1] < 0.5 * v


def test_trapezoid_short_move_is_triangular():
    p = [np.zeros(6), np.array([0.05, 0, 0, 0, 0, 0])]
    timed = mk.time_trapezoid(p, v_max=0.3, a_max=0.5)
    assert timed[-1][1] == pytest.approx(2 * np.sqrt(0.05 / 0.5), rel=1e-6)


def test_smoothed_transit_on_the_t_joint_stays_clear(scene):  # noqa: F811
    tool, cfg, model, report = scene
    tacks = report["tacks"]
    q_a = np.asarray(next(t for t in tacks if t["seam_id"] == 0)["q_app"])
    q_b = np.asarray(next(t for t in tacks if t["seam_id"] == 1)["q_app"])   # other side
    m = mk.transit_model(model, cfg, q_a, q_b)
    rough = mk.transit_path(q_a, q_b, m)
    smooth = mk.transit_path(q_a, q_b, m, smooth=True)
    assert rough is not None and smooth is not None
    assert np.allclose(smooth[0], q_a) and np.allclose(smooth[-1], rough[-1])
    for x, y in zip(smooth[:-1], smooth[1:]):
        assert _edge_valid(x, y, resolution=0.01, is_valid=m.is_valid)
    if len(rough) > 2:                                           # an RRT path, not the straight edge
        assert len(smooth) > len(rough)
        assert _max_turn_deg(smooth) < _max_turn_deg(rough)


def test_full_plan_with_smooth_transits(scene):  # noqa: F811
    tool, cfg, model, report = scene
    plan = mk.build_marking_plan(report, tool, model, cfg, cfg.home_q, smooth_transits=True)
    assert plan.ok, plan.summary()
    for s in plan.steps:
        assert np.allclose(s.transit[-1], s.q_app) and np.allclose(s.descent[0], s.q_app)
    assert np.allclose(plan.home_path[-1], cfg.home_q, atol=1e-9)
