"""Phase 6b step 1 — the exact seam curves (D33).

The claim under test is D1 itself, extended to curved seams: every point a curve
returns lies ON its defining surfaces to machine precision, tangents are the true
derivatives, arclength sampling covers the curve at the requested density, and the
seam-block JSON round-trips. No solver tolerance appears anywhere — that is what D33
withdrew, and these tests are the receipt.
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from weldgen.curves import (  # noqa: E402
    Arc3D, BSplineCurve, Ellipse3D, SaddleCurve,
    ellipse_from_plane_cylinder, from_parametric, saddle_from_cylinders,
)

ATOL = 1e-9


def _on_cylinder(pts, p0, axis, r):
    d = pts - np.asarray(p0, float)[None, :]
    am = d @ np.asarray(axis, float)
    return np.abs(np.einsum("ij,ij->i", d, d) - am ** 2 - r ** 2)


def _on_plane(pts, q, n):
    return np.abs((pts - np.asarray(q, float)[None, :]) @ np.asarray(n, float))


def _fd_tangent(curve, t, h=1e-7):
    d = curve.point(np.atleast_1d(t) + h) - curve.point(np.atleast_1d(t) - h)
    return d / np.linalg.norm(d, axis=1, keepdims=True)


# ------------------------------------------------------------- plane ∩ cylinder


def test_perpendicular_plane_cuts_a_circle():
    c = ellipse_from_plane_cylinder([0, 0, 5.0], [0, 0, 1], [2.0, -1.0, 0], [0, 0, 1],
                                    radius_mm=30.0)
    assert c.is_circle and abs(c.a_mm - 30.0) < ATOL
    ts = np.linspace(0, 2 * np.pi, 97)
    pts = c.point(ts)
    assert _on_cylinder(pts, [2, -1, 0], [0, 0, 1], 30.0).max() < ATOL
    assert _on_plane(pts, [0, 0, 5.0], [0, 0, 1]).max() < ATOL
    assert abs(c.length_mm - 2 * np.pi * 30.0) < 1e-3        # numeric table vs exact


def test_tilted_plane_cuts_an_ellipse_on_both_surfaces():
    rng = np.random.default_rng(3)
    for _ in range(20):
        n = rng.normal(size=3)
        n /= np.linalg.norm(n)
        a = rng.normal(size=3)
        a /= np.linalg.norm(a)
        if abs(n @ a) < 0.15:                                # skip near-parallel draws
            continue
        q = rng.uniform(-50, 50, 3)
        p0 = rng.uniform(-50, 50, 3)
        r = float(rng.uniform(5, 60))
        e = ellipse_from_plane_cylinder(q, n, p0, a, r)
        pts = e.point(np.linspace(0, 2 * np.pi, 181))
        assert _on_cylinder(pts, p0, a, r).max() < 1e-6
        assert _on_plane(pts, q, n).max() < 1e-6
        cos_t = abs(float(n @ a))
        assert abs(e.b_mm - r) < ATOL
        assert abs(e.a_mm - r / cos_t) < 1e-9
        assert abs(float(e.u_dir @ e.v_dir)) < ATOL


def test_axis_in_plane_is_rejected():
    with pytest.raises(ValueError):
        ellipse_from_plane_cylinder([0, 0, 0], [0, 0, 1], [0, 0, 0], [1, 0, 0], 10.0)


# ------------------------------------------------------------ cylinder ∩ cylinder


def test_saddle_lies_on_both_cylinders_for_the_hard_cases():
    """Unequal radii, offset axes, non-perpendicular branch — the configuration that
    motivated the (withdrawn) tolerance clause. Exactness to machine precision."""
    cases = [
        # (branch origin, branch axis, r, main point, main axis, R)
        ([0, 0, 120.0], [0, 0, -1.0], 20.0, [0, 0, 0], [1, 0, 0], 60.0),
        ([15.0, 8.0, 120.0], [0.2, -0.1, -1.0], 25.0, [0, 5.0, 0], [1, 0.1, 0], 70.0),
        ([-10.0, 0, 100.0], [0.5, 0.0, -1.0], 12.0, [0, 0, 0], [1, 0, 0], 55.0),
    ]
    for bo, ba, r, mp, ma, R in cases:
        sc = saddle_from_cylinders(bo, ba, r, mp, ma, R)
        phis = np.linspace(0, 2 * np.pi, 361, endpoint=False)
        pts = sc.point(phis)
        assert _on_cylinder(pts, bo, np.asarray(ba, float) / np.linalg.norm(ba),
                            r).max() < 1e-6, "not on the branch"
        assert _on_cylinder(pts, mp, np.asarray(ma, float) / np.linalg.norm(ma),
                            R).max() < 1e-6, "not on the main pipe"
        assert sc.closed


def test_saddle_rejects_grazing_and_parallel_configurations():
    with pytest.raises(ValueError):    # branch offset ACROSS the main axis: grazing
        saddle_from_cylinders([0.0, 45.0, 120.0], [0, 0, -1], 30.0,
                              [0, 0, 0], [1, 0, 0], 50.0)
    with pytest.raises(ValueError):                          # parallel axes
        saddle_from_cylinders([0, 0, 120.0], [1, 0, 0], 20.0,
                              [0, 0, 0], [1, 0, 0], 60.0)


def test_saddle_entering_root_sits_on_the_near_side():
    """root_sign = -1 with the branch axis pointing toward the main pipe must pick the
    intersection the stub reaches FIRST - larger s along a from a distant origin means
    ... the smaller |P - origin|; both roots are on the main pipe, only one is the
    seam a set-on stub can carry."""
    sc = saddle_from_cylinders([0, 0, 120.0], [0, 0, -1.0], 20.0,
                               [0, 0, 0], [1, 0, 0], 60.0)
    pts = sc.point(np.linspace(0, 2 * np.pi, 90, endpoint=False))
    assert (pts[:, 2] > 0).all(), "entering intersection is the upper sheet"


# ------------------------------------------------------------- constructed forms


def test_arc_is_exact_and_length_is_closed_form():
    arc = Arc3D(np.array([5.0, -2.0, 1.0]), np.array([1.0, 0, 0]),
                np.array([0, 1.0, 0]), radius_mm=40.0, t0=0.3, t1=2.1)
    assert abs(arc.length_mm - 40.0 * 1.8) < ATOL
    pts = arc.sample(10.0)
    d = np.linalg.norm(pts - np.array([5.0, -2.0, 1.0]), axis=1)
    assert np.abs(d - 40.0).max() < ATOL
    assert len(pts) == int(round(arc.length_mm * 10.0)) + 1


def test_bspline_clamps_endpoints_and_stays_smooth():
    ctrl = np.array([[0, 0, 0], [30, 5, 0], [60, -5, 0], [90, 10, 0], [120, 0, 0]],
                    dtype=float)
    sp = BSplineCurve(ctrl)
    assert np.allclose(sp.point([0.0])[0], ctrl[0], atol=1e-9)
    assert np.allclose(sp.point([1.0 - 1e-12])[0], ctrl[-1], atol=1e-6)
    # convex-hull property of B-splines: the curve stays inside the control bounds
    pts = sp.point(np.linspace(0, 1, 400))
    assert pts[:, 0].min() >= -1e-6 and pts[:, 0].max() <= 120.0 + 1e-6
    assert pts[:, 1].min() >= -5.0 - 1e-6 and pts[:, 1].max() <= 10.0 + 1e-6


# --------------------------------------------------------- shared curve contract


def _all_curves():
    yield ellipse_from_plane_cylinder([0, 0, 4.0], [0.2, 0.1, 1.0], [0, 0, 0],
                                      [0.1, -0.2, 1.0], 30.0)
    yield saddle_from_cylinders([10.0, 0, 120.0], [0.2, 0, -1.0], 18.0,
                                [0, 0, 0], [1, 0, 0], 60.0)
    yield Arc3D(np.zeros(3), np.array([1.0, 0, 0]), np.array([0, 1.0, 0]),
                50.0, 0.0, 2.5)
    yield BSplineCurve(np.array([[0, 0, 0], [40, 20, 0], [80, -20, 5], [120, 0, 0]],
                                dtype=float))


def test_tangents_match_finite_differences_and_are_unit():
    for c in _all_curves():
        ts = np.linspace(0.05, c.t_period - 0.05, 50)
        tan = c.tangent(ts)
        assert np.abs(np.linalg.norm(tan, axis=1) - 1.0).max() < 1e-9
        fd = _fd_tangent(c, ts)
        assert np.abs(np.einsum("ij,ij->i", tan, fd) - 1.0).max() < 1e-5, type(c)


def test_arclength_sampling_is_uniform_at_the_requested_density():
    for c in _all_curves():
        pts = c.sample(5.0)                                  # 5 points per mm
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        assert abs(np.median(seg) - 0.2) < 0.02, type(c)
        assert seg.std() / seg.mean() < 0.02, f"{type(c)}: non-uniform arclength"
        n_expected = c.length_mm * 5.0
        assert abs(len(pts) - n_expected) <= 2


def test_closed_curves_do_not_repeat_the_wrap_point():
    for c in _all_curves():
        pts = c.sample(2.0)
        if c.closed:
            assert np.linalg.norm(pts[0] - pts[-1]) > 1e-3
        else:
            assert np.allclose(pts[0], c.point([0.0])[0], atol=1e-9)


def test_parametric_json_roundtrip_reproduces_the_curve():
    for c in _all_curves():
        d = c.to_parametric()
        c2 = from_parametric(d)
        ts = np.linspace(0.0, min(c.t_period, c2.t_period) * 0.999, 40)
        assert np.allclose(c.point(ts), c2.point(ts), atol=1e-9), d["kind"]
    # and the circle special case serialises as "circle", not a degenerate ellipse
    circ = ellipse_from_plane_cylinder([0, 0, 5.0], [0, 0, 1], [0, 0, 0], [0, 0, 1],
                                       25.0)
    assert circ.to_parametric()["kind"] == "circle"


# --------------------------------------------------------------- the D29 sampler


def test_d29_sampler_covers_all_configurations_deterministically():
    from weldgen.d29 import D29_TABLE, sample_d29_seam
    rng = np.random.default_rng(0)
    seen = set()
    for _ in range(200):
        d = sample_d29_seam(rng)
        seen.add(d["config"])
    assert seen == set(D29_TABLE)
    # determinism: the same stream state reproduces the same parametric json
    a = sample_d29_seam(np.random.default_rng(7))
    b = sample_d29_seam(np.random.default_rng(7))
    assert a["curve"].to_parametric() == b["curve"].to_parametric()


def test_d29_realization_surfaces_contain_the_drawn_curve():
    """Curve-first direction proof: the pipe DERIVED from the drawn circle/ellipse
    (and the saddle family's two pipes) contains the curve exactly."""
    from weldgen.d29 import sample_d29_seam
    rng = np.random.default_rng(1)
    for cfg in (2, 3):
        d = sample_d29_seam(rng, config=cfg)
        g = d["realization"]
        pts = d["curve"].point(np.linspace(0, 2 * np.pi, 181))
        assert np.abs(pts[:, 2]).max() < 1e-9, "seam must lie in the plate plane"
        assert _on_cylinder(pts, g["pipe_point_mm"], g["pipe_axis"],
                            g["pipe_radius_mm"]).max() < 1e-6
    d = sample_d29_seam(rng, config=4)
    g = d["realization"]
    pts = d["curve"].point(np.linspace(0, 2 * np.pi, 181, endpoint=False))
    assert _on_cylinder(pts, g["branch_origin_mm"], g["branch_axis"],
                        g["branch_radius_mm"]).max() < 1e-6
    assert _on_cylinder(pts, g["main_point_mm"], g["main_axis"],
                        g["main_radius_mm"]).max() < 1e-6


def test_d29_curvature_floor_binds_the_drawn_families_only():
    """The 30 mm floor is a bound on curves WE draw (#6-#7). The artifact-derived
    configurations carry the curvature their sampled radii produce - a 10 mm tube
    corner IS the seam the artifact makes, which is D29's point."""
    from weldgen.d29 import MIN_RADIUS_MM, max_curvature, sample_d29_seam
    rng = np.random.default_rng(2)
    seen = set()
    for _ in range(80):
        d = sample_d29_seam(rng)
        if d["config"] not in (6, 7):
            continue
        seen.add(d["config"])
        k = max_curvature(d["curve"])
        assert k <= 1.0 / MIN_RADIUS_MM + 1e-6, (d["config"], 1.0 / k)
    assert seen == {6, 7}


def test_d29_closed_flags_match_the_topology():
    from weldgen.d29 import sample_d29_seam
    rng = np.random.default_rng(3)
    closed_expected = {1: False, 2: True, 3: True, 4: True, 5: True,
                      6: False, 7: False}
    for cfg, want in closed_expected.items():
        d = sample_d29_seam(rng, config=cfg)
        assert d["curve"].closed == want, cfg
