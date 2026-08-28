"""D29 seam sampler — Phase 6b step 1 (curve-first, per the 2026-08-28 amendment).

D29's rule: curved seams are DRAWN from curve families that admit a two-part
realization — the D3 inversion carried into Phase 6. This module draws a family and
its curve parameters FIRST, then derives the artifact that realises it (the
`realization` dict the step-2 constructors turn into solids): a circle's radius IS the
pipe radius, an ellipse's semi-minor and aspect ARE the pipe radius and tilt, a
rounded rectangle IS the tube's outer wall, an arc or planar spline IS the sweep path.
Surface∩surface therefore never appears in the generation path; the intersection
machinery lives in the D4 verification arms, which independently rediscover what
construction placed.

The one family whose parametrization LOOKS like an intersection is the saddle: its
family parameters (R, r, θ, offset) are exactly the two pipes', and its closed-form
parametrization is the D33 quadratic — evaluating a family member, not solving between
independently sampled parts.

Conventions: the base plane is z = 0 in the joint frame (the plate layouts'
convention); realized stubs stand in +z. The `seam_curve` substream drives every draw,
extending the Phase 6a discipline — corpora with curved seams off reproduce
bit-identically.

The curvature floor (PARAMETERS.md: seam curvature radius 30 mm – ∞) binds the
FREE-FORM families (#6–#7), where curvature is not pinned by a drawn radius: arcs by
construction, splines by a deterministic redraw against measured curvature. The
radius-parametrized families (#2–#5) carry the curvature their drawn radii imply — a
10 mm corner radius or an ellipse vertex at 1/(b·cos θ) is a property of the draw,
recorded in its parameters.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from .curves import (
    Arc3D, BSplineCurve, Ellipse3D, Segment3D, rounded_rect_curve,
    saddle_from_cylinders,
)

#: Minimum seam curvature radius for the DRAWN families (#6-#7), mm.
MIN_RADIUS_MM = 30.0

#: The D29 table: family id -> (curve family, joint type its realization produces).
#: #4 realises T in v1 (set-on stub, patch_phase6b scope call); #6 realises T (the
#: lap variant is a constructor option, not a different curve).
D29_TABLE = {
    1: ("line", "T"),
    2: ("circle", "T"),
    3: ("ellipse", "T"),
    4: ("saddle", "T"),
    5: ("rounded_rect", "T"),
    6: ("swept_path", "T"),
    7: ("arc_butt", "butt"),
}


def max_curvature(curve, n: int = 512) -> float:
    """Measured maximum curvature (1/mm) via circumscribed circles of point triples."""
    pts = curve.point(np.linspace(0.0, curve.t_period, n))
    a, b, c = pts[:-2], pts[1:-1], pts[2:]
    ab = np.linalg.norm(b - a, axis=1)
    bc = np.linalg.norm(c - b, axis=1)
    ca = np.linalg.norm(a - c, axis=1)
    area2 = np.linalg.norm(np.cross(b - a, c - a), axis=1)
    with np.errstate(divide="ignore", invalid="ignore"):
        k = 2.0 * area2 / (ab * bc * ca)
    return float(np.nanmax(k))


def _config_2(rng) -> tuple[Any, dict]:
    """Circle family: draw the CURVE (centre, radius); the pipe is derived from it."""
    r = float(rng.uniform(MIN_RADIUS_MM, 80.0))
    cx, cy = rng.uniform(-40.0, 40.0, 2)
    curve = Ellipse3D(np.array([cx, cy, 0.0]),
                      np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0]), r, r)
    realization = {"pipe_radius_mm": r, "pipe_axis": [0.0, 0.0, 1.0],
                   "pipe_point_mm": [float(cx), float(cy), 0.0]}
    return curve, realization


def _config_3(rng) -> tuple[Any, dict]:
    """Ellipse family: draw the CURVE (centre, semi-minor b, aspect via tilt,
    in-plane orientation); the tilted pipe is derived — radius = b, tilt =
    arccos(b/a), axis in the plane of the base normal and the major direction."""
    b = float(rng.uniform(MIN_RADIUS_MM, 70.0))
    tilt = float(rng.uniform(np.radians(10.0), np.radians(40.0)))
    psi = float(rng.uniform(0.0, 2.0 * np.pi))
    cx, cy = rng.uniform(-30.0, 30.0, 2)
    u = np.array([np.cos(psi), np.sin(psi), 0.0])         # major direction
    v = np.array([-np.sin(psi), np.cos(psi), 0.0])        # minor direction
    a = b / np.cos(tilt)
    curve = Ellipse3D(np.array([cx, cy, 0.0]), u, v, a, b)
    axis = np.cos(tilt) * np.array([0.0, 0.0, 1.0]) + np.sin(tilt) * u
    realization = {"pipe_radius_mm": b, "pipe_axis": [float(x) for x in axis],
                   "pipe_point_mm": [float(cx), float(cy), 0.0],
                   "tilt_deg": float(np.degrees(tilt))}
    return curve, realization


def _config_4(rng) -> tuple[Any, dict]:
    """Saddle family: (R, r, θ, offset) ARE the family's parameters — and the two
    pipes' — so the draw and the realization coincide; the curve is the family's
    closed-form parametrization (D33), evaluated, not solved."""
    R = float(rng.uniform(40.0, 80.0))
    r = float(rng.uniform(max(MIN_RADIUS_MM * 0.5, 0.3 * R), 0.7 * R))
    # branch tilted off the main pipe's surface normal, offset across the main axis
    # bounded so the stub lands fully on the pipe (the saddle guard re-checks it)
    tilt = float(rng.uniform(0.0, np.radians(30.0)))
    off = float(rng.uniform(-0.4, 0.4)) * (R - r)
    axis = np.array([np.sin(tilt), 0.0, -np.cos(tilt)])       # pointing at the main
    origin = np.array([0.0, off, R + r + 60.0])
    curve = saddle_from_cylinders(origin, axis, r, [0, 0, 0.0], [1.0, 0, 0], R)
    realization = {"main_radius_mm": R, "main_axis": [1.0, 0.0, 0.0],
           "main_point_mm": [0.0, 0.0, 0.0],
                   "branch_radius_mm": r,
                   "branch_axis": [float(v) for v in axis],
                   "branch_origin_mm": [float(v) for v in origin],
                   "tilt_deg": float(np.degrees(tilt)), "offset_mm": off}
    return curve, realization


def _config_5(rng) -> tuple[Any, dict]:
    w = float(rng.uniform(60.0, 160.0))
    h = float(rng.uniform(60.0, 160.0))
    rc = float(rng.uniform(8.0, 0.25 * min(w, h)))
    cx, cy = rng.uniform(-30.0, 30.0, 2)
    curve = rounded_rect_curve([cx, cy, 0.0], [1.0, 0, 0], [0, 1.0, 0], w, h, rc)
    realization = {"tube_w_mm": w, "tube_h_mm": h, "corner_r_mm": rc,
                   "center_mm": [float(cx), float(cy), 0.0]}
    return curve, realization


def _config_6(rng) -> tuple[Any, dict]:
    # A swept plate standing on the base: its foot is the seam, drawn FIRST (D3).
    # Half the draws are arcs, half gentle splines under the curvature floor.
    span = float(rng.uniform(120.0, 300.0))
    if rng.random() < 0.5:
        radius = float(rng.uniform(max(MIN_RADIUS_MM, span / np.pi), 400.0))
        half = span / (2.0 * radius)
        curve = Arc3D(np.array([0.0, radius, 0.0]), np.array([0.0, -1.0, 0.0]),
                      np.array([1.0, 0.0, 0.0]), radius,
                      -half, half)
        return curve, {"form": "arc", "radius_mm": radius, "span_mm": span}
    for _ in range(16):                       # deterministic redraws, same stream
        n_ctrl = int(rng.integers(4, 6))
        xs = np.linspace(-span / 2.0, span / 2.0, n_ctrl)
        ys = rng.uniform(-span / 6.0, span / 6.0, n_ctrl)
        ctrl = np.column_stack([xs, ys, np.zeros(n_ctrl)])
        curve = BSplineCurve(ctrl)
        if max_curvature(curve) <= 1.0 / MIN_RADIUS_MM:
            return curve, {"form": "bspline",
                           "control_mm": [[float(v) for v in row] for row in ctrl]}
    # pragma: no cover - flat fallback after 16 tries is measure-~zero
    ctrl = np.column_stack([xs, np.zeros(n_ctrl), np.zeros(n_ctrl)])
    return BSplineCurve(ctrl), {"form": "bspline",
                                "control_mm": [[float(v) for v in row]
                                               for row in ctrl]}


def _config_7(rng) -> tuple[Any, dict]:
    radius = float(rng.uniform(100.0, 400.0))
    span = float(rng.uniform(100.0, min(300.0, 0.9 * np.pi * radius)))
    half = span / (2.0 * radius)
    curve = Arc3D(np.array([0.0, radius, 0.0]), np.array([0.0, -1.0, 0.0]),
                  np.array([1.0, 0.0, 0.0]), radius, -half, half)
    return curve, {"form": "arc", "radius_mm": radius, "span_mm": span}


def _config_1(rng) -> tuple[Any, dict]:
    L = float(rng.uniform(80.0, 400.0))
    curve = Segment3D([-L / 2.0, 0.0, 0.0], [L / 2.0, 0.0, 0.0])
    return curve, {"length_mm": L}


_SAMPLERS = {1: _config_1, 2: _config_2, 3: _config_3, 4: _config_4,
             5: _config_5, 6: _config_6, 7: _config_7}


def sample_d29_seam(rng: np.random.Generator,
                    config: int | None = None,
                    weights: dict[int, float] | None = None) -> dict[str, Any]:
    """One D29 draw: pick a curve FAMILY (or take the given id), draw its curve
    parameters, derive the realization. Returns
    `{config, name, joint_type, curve, realization}` — `curve` is the weldgen.curves
    object (with `.to_parametric()` for the seam block) and `realization` is the
    artifact derived from it, which the step-2 constructors turn into solids."""
    if config is None:
        ids = sorted(_SAMPLERS)
        w = np.array([float((weights or {}).get(i, 1.0)) for i in ids])
        config = int(rng.choice(ids, p=w / w.sum()))
    name, joint_type = D29_TABLE[config]
    curve, realization = _SAMPLERS[config](rng)
    return {"config": int(config), "name": name, "joint_type": joint_type,
            "curve": curve, "realization": realization}
