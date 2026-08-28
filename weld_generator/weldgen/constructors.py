"""D29 part constructors — Phase 6b step 2.

Each constructor realises a `d29.sample_d29_seam` draw as posed solids whose surfaces
CONTAIN the drawn curve — the CURVE came first (D29 curve-first amendment, 2026-08-28)
and the parts are derived from it, so containment is exact by construction and the
tests verify it to machine precision; the D4 verification arms later rediscover the
same curve by intersection, which is where surface∩surface machinery lives. Set-on
throughout (patch_phase6b scope call):
the stub's end is a miter CUT by the surface it lands on — the plate plane for
configs 2–3, the main pipe's own cylinder for config 4 — evaluated from the same
closed forms as the seam, so the miter ring IS the seam (offset by the root gap
along the stub axis).

The line family (#1) stays with `layouts.build`, the plate machinery it always had.

Conventions: joint frame as everywhere - the base part's top surface contains z = 0
for the plate configs; the main pipe's axis is the x-axis for pipe-to-pipe. The
fixture is off for every configuration here (patch_phase6b), and D31/D32 do not
apply (plate-scoped).
"""

from __future__ import annotations

from typing import Any

import numpy as np

from .curves import SaddleCurve
from .geom import Slab, SweptSlab, Tube, translate


def _frame_T(axis_z: np.ndarray, origin: np.ndarray) -> np.ndarray:
    """A part pose whose local +z is `axis_z`, with a deterministic in-plane pair."""
    z = np.asarray(axis_z, dtype=float)
    z = z / np.linalg.norm(z)
    tmp = np.array([0.0, 0.0, 1.0])
    if abs(float(z @ tmp)) > 0.9:
        tmp = np.array([1.0, 0.0, 0.0])
    x = np.cross(tmp, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2] = x, y, z
    T[:3, 3] = np.asarray(origin, dtype=float)
    return T


def _plate_for(curve_xy_extent: tuple[float, float, float, float],
               rng: np.random.Generator, margin_mm: float = 25.0) -> Slab:
    """The base plate: top face on z = 0, footprint covering the seam plus margin."""
    x0, x1, y0, y1 = curve_xy_extent
    L = float(max(x1 - x0, 1.0) + 2.0 * margin_mm + rng.uniform(0.0, 120.0))
    W = float(max(y1 - y0, 1.0) + 2.0 * margin_mm + rng.uniform(0.0, 120.0))
    t = float(rng.uniform(4.0, 12.0))
    cx, cy = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
    return Slab("A", "workpiece", 0, (L, W, t),
                translate(cx, cy, -t / 2.0))


def _wall_for(r: float, rng: np.random.Generator) -> float:
    return float(rng.uniform(3.0, min(8.0, 0.4 * r)))


def build_config_2(draw: dict[str, Any], rng: np.random.Generator) -> dict[str, Any]:
    """Pipe perpendicular on plate: flat base cap at gap above the plate (#2)."""
    g = draw["realization"]
    r = g["pipe_radius_mm"]
    cx, cy, _ = g["pipe_point_mm"]
    gap = float(rng.uniform(0.1, 1.2))
    plate = _plate_for((cx - r, cx + r, cy - r, cy + r), rng)
    tube = Tube("B", "workpiece", 1, r, _wall_for(r, rng),
                float(rng.uniform(80.0, 220.0)),
                translate(cx, cy, 0.0), base_cut=None, gap_mm=gap)
    return {"parts": [plate, tube], "curve": draw["curve"], "gap_mm": gap,
            "config": 2, "name": draw["name"], "joint_type": draw["joint_type"],
            "realization": {**g, "wall_mm": tube.wall_mm,
                         "pipe_length_mm": tube.length_mm}}


def build_config_3(draw: dict[str, Any], rng: np.random.Generator) -> dict[str, Any]:
    """Pipe at an angle on plate: base MITERED by the plate plane (#3).

    The tube's local origin sits at the seam's centre (axis ∩ plate plane), so the
    plate plane z_world = 0 becomes the local plane `n_local · p = 0` with
    `n_local = Rᵀ ẑ` — the miter ring lies exactly in the plate plane, offset by the
    root gap along the AXIS (the gap for a mitered stub is measured along its own
    axis; document once, here).
    """
    g = draw["realization"]
    r = g["pipe_radius_mm"]
    axis = np.asarray(g["pipe_axis"], dtype=float)
    origin = np.asarray(g["pipe_point_mm"], dtype=float)     # on the plate plane
    T = _frame_T(axis, origin)
    n_local = T[:3, :3].T @ np.array([0.0, 0.0, 1.0])
    gap = float(rng.uniform(0.1, 1.2))
    # the miter's highest lip is r*tan(tilt); the stub must extend well past it
    e = draw["curve"]
    ext = e.point(np.linspace(0, 2 * np.pi, 181))
    lip = r * np.tan(np.radians(g["tilt_deg"]))
    tube = Tube("B", "workpiece", 1, r, _wall_for(r, rng),
                float(lip + rng.uniform(80.0, 220.0)), T,
                base_cut={"kind": "plane",
                          "n_local": [float(v) for v in n_local], "d": 0.0},
                gap_mm=gap)
    plate = _plate_for((ext[:, 0].min(), ext[:, 0].max(),
                        ext[:, 1].min(), ext[:, 1].max()), rng)
    return {"parts": [plate, tube], "curve": draw["curve"], "gap_mm": gap,
            "config": 3, "name": draw["name"], "joint_type": draw["joint_type"],
            "realization": {**g, "wall_mm": tube.wall_mm,
                         "pipe_length_mm": tube.length_mm}}


def build_config_4(draw: dict[str, Any], rng: np.random.Generator) -> dict[str, Any]:
    """Pipe-to-pipe: the branch's base is saddle-CUT by the main cylinder (#4).

    The branch tube's local +z points AWAY from the main pipe; its origin is placed
    on the branch axis just past the deepest saddle point, so every base height is a
    small positive number and the cut ring — the exact D33 quadratic per (φ, ρ) —
    reproduces the step-1 seam on the outer wall.
    """
    g = draw["realization"]
    R = g["main_radius_mm"]
    r = g["branch_radius_mm"]
    a_toward = np.asarray(g["branch_axis"], dtype=float)
    a_toward /= np.linalg.norm(a_toward)
    b_origin = np.asarray(g["branch_origin_mm"], dtype=float)
    gap = float(rng.uniform(0.1, 1.2))

    # main pipe along x, spanning the saddle with margin on both sides
    curve: SaddleCurve = draw["curve"]
    span = curve.point(np.linspace(0, 2 * np.pi, 361, endpoint=False))
    Lm = float((span[:, 0].max() - span[:, 0].min())
               + 2.0 * (40.0 + rng.uniform(0.0, 120.0)))
    x0 = float(0.5 * (span[:, 0].min() + span[:, 0].max()) - Lm / 2.0)
    main = Tube("A", "workpiece", 0, R, _wall_for(R, rng), Lm,
                _frame_T([1.0, 0.0, 0.0], [x0, 0.0, 0.0]))

    # branch: origin just past the deepest entering intersection along the axis
    _, s_vals = curve.s_of(np.linspace(0, 2 * np.pi, 720, endpoint=False))
    o_loc = b_origin + float(s_vals.max() + 1.0) * a_toward
    T = _frame_T(-a_toward, o_loc)
    Rl = T[:3, :3]
    main_pt_local = Rl.T @ (np.array([0.0, 0.0, 0.0]) - o_loc)
    main_ax_local = Rl.T @ np.array([1.0, 0.0, 0.0])
    branch = Tube("B", "workpiece", 1, r, _wall_for(r, rng),
                  float(s_vals.max() - s_vals.min() + 1.0
                        + rng.uniform(80.0, 200.0)), T,
                  base_cut={"kind": "cylinder",
                            "point_local": [float(v) for v in main_pt_local],
                            "axis_local": [float(v) for v in main_ax_local],
                            "radius_mm": float(R)},
                  gap_mm=gap)
    return {"parts": [main, branch], "curve": curve, "gap_mm": gap,
            "config": 4, "name": draw["name"], "joint_type": draw["joint_type"],
            "realization": {**g, "main_wall_mm": main.wall_mm,
                         "branch_wall_mm": branch.wall_mm,
                         "main_length_mm": Lm,
                         "branch_length_mm": branch.length_mm}}


def build_config_5(draw: dict[str, Any], rng: np.random.Generator) -> dict[str, Any]:
    """Rectangular tube on plate (#5): the step-1 rounded rectangle IS the tube's
    outer surface. The spine is CCW, so `n̂ = ẑ × T̂` points inward and the wall is
    the band [0, +wall]: `-w` (offset 0) is the outer wall, `+w` the bore."""
    g = draw["realization"]
    rc = g["corner_r_mm"]
    wall = float(min(rng.uniform(3.0, 8.0), 0.8 * rc))
    gap = float(rng.uniform(0.1, 1.2))
    H = float(rng.uniform(80.0, 220.0))
    tube = SweptSlab("B", "workpiece", 1, draw["curve"], 0.0, wall, gap, gap + H,
                     np.eye(4))
    ext = draw["curve"].point(np.linspace(0, draw["curve"].t_period, 361,
                                          endpoint=False))
    plate = _plate_for((ext[:, 0].min(), ext[:, 0].max(),
                        ext[:, 1].min(), ext[:, 1].max()), rng)
    return {"parts": [plate, tube], "curve": draw["curve"], "gap_mm": gap,
            "config": 5, "name": draw["name"], "joint_type": draw["joint_type"],
            "realization": {**g, "wall_mm": wall, "height_mm": H}}


def build_config_6(draw: dict[str, Any], rng: np.random.Generator) -> dict[str, Any]:
    """Swept stiffener on plate (#6): the drawn curve is the stiffener's centreline
    foot; the plate takes the D29 role of the base, the stiffener is the ±t/2 band
    standing at the root gap. The seam curve was an INPUT (D3/D33) — nothing here
    was fitted."""
    g = draw["realization"]
    t = float(rng.uniform(4.0, 10.0))
    gap = float(rng.uniform(0.1, 1.2))
    H = float(rng.uniform(50.0, 160.0))
    st = SweptSlab("B", "workpiece", 1, draw["curve"], -t / 2.0, t / 2.0,
                   gap, gap + H, np.eye(4))
    ext = draw["curve"].point(np.linspace(0, draw["curve"].t_period, 361))
    plate = _plate_for((ext[:, 0].min(), ext[:, 0].max(),
                        ext[:, 1].min(), ext[:, 1].max()), rng,
                       margin_mm=25.0 + t)
    return {"parts": [plate, st], "curve": draw["curve"], "gap_mm": gap,
            "config": 6, "name": draw["name"], "joint_type": draw["joint_type"],
            "realization": {**g, "thickness_mm": t, "height_mm": H}}


def build_config_7(draw: dict[str, Any], rng: np.random.Generator) -> dict[str, Any]:
    """Curved butt (#7): two flat plates whose prepared edges follow the drawn arc,
    the gap centred on it — the curve is the gap CENTRELINE, exactly the role the
    straight butt's centreline plays. Tops flush at z = 0 (the exposed face carrying
    the centreline is the sampled plane; the underside steps with dissimilar
    thickness, same convention as the straight butt's flush side)."""
    g = draw["realization"]
    R = g["radius_mm"]
    gap = float(rng.uniform(0.3, 2.0))
    t_a = float(rng.uniform(3.0, 10.0))
    t_b = t_a if rng.random() < 0.7 else float(rng.uniform(3.0, 10.0))
    W_a = float(min(rng.uniform(50.0, 150.0), 0.6 * R))
    W_b = float(min(rng.uniform(50.0, 150.0), 0.6 * R))
    A = SweptSlab("A", "workpiece", 0, draw["curve"],
                  gap / 2.0, gap / 2.0 + W_a, -t_a, 0.0, np.eye(4))
    B = SweptSlab("B", "workpiece", 1, draw["curve"],
                  -gap / 2.0 - W_b, -gap / 2.0, -t_b, 0.0, np.eye(4))
    return {"parts": [A, B], "curve": draw["curve"], "gap_mm": gap,
            "config": 7, "name": draw["name"], "joint_type": draw["joint_type"],
            "realization": {**g, "t_A_mm": t_a, "t_B_mm": t_b,
                         "W_A_mm": W_a, "W_B_mm": W_b}}


_BUILDERS = {2: build_config_2, 3: build_config_3, 4: build_config_4,
             5: build_config_5, 6: build_config_6, 7: build_config_7}


def build_d29(draw: dict[str, Any], rng: np.random.Generator) -> dict[str, Any]:
    """Realise a D29 draw as posed solids (configs 2–7; #1 stays with
    `layouts.build`, the plate machinery it always had)."""
    cfg = draw["config"]
    if cfg not in _BUILDERS:
        raise NotImplementedError(f"D29 config {cfg} constructor not yet built")
    return _BUILDERS[cfg](draw, rng)
