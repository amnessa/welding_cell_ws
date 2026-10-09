"""Seams of the curved parts at their registered poses - `notes/curved_seams_plan.md` step 2.

WeldSet draws a curved seam first and builds the parts around it; its verification arm
(`weldgen.verify_curved`) goes the other way and intersects the placed parts' surfaces.
The cell has exactly that input - every saved part is a primitive at a registered
pose - so the seam is computed, never detected:

  * **tube on a plate** (C1, E2): the plate face's plane ∩ the tube's outer cylinder,
    a circle or an ellipse (`ellipse_from_plane_cylinder`);
  * **tube on a tube** (S3 on R2): branch cylinder ∩ run cylinder, the saddle
    (`saddle_from_cylinders`);
  * **band on a plate** (RR1, a swept stiffener): each wall of the band traced along
    the band's own extrusion axis onto the plate plane - exact for a band standing at
    any tilt, since the wall is a ruled surface along that axis.

Registration is a few mm off, so the parts never touch exactly. The curve is taken on
the surfaces that do not move with that error (the plate plane and the tube's wall; a
tube sliding along its axis does not change it), and the distance from the member's
END (its cut or cap, which the error does move) to that curve, along the member's axis,
is the fit-up: positive = gap, negative = penetration, reported per seam as the plate
seams report theirs. Which end stands on the other part is not assumed: both ends are
tried, and an end counts only when its fit-up stays within `pose_tol_mm` all round
(within `NEAR_MISS` times it the seam is still reported, rejected, with its fit-up).

Every seam carries per-point frames, because on a curved seam they rotate with it:
`n_a_per_point` (the base surface), `n_b_per_point` (the member's wall), and
`approach_per_point`, their bisector. The verdict is weldgen's own (`seam_verdict`):
the D4 torch cone at every point, weldable when at least 95 % of the points clear; a
bore (the member's inner wall meeting the base) is a negative unless a tool fits in it.
"""

from __future__ import annotations

from typing import Any, Sequence

import numpy as np

#: The pen holder and the wrist camera cannot enter a pipe bore narrower than this.
#: weldgen's 80 mm is a welding torch's; the cell's tool is wider.
CELL_BORE_MIN_DIAMETER_MM = 200.0

#: A member's axis must meet the face it stands on at least this steeply. Below it the
#: plane ∩ cylinder ellipse runs off to infinity (a pipe lying on a plate touches it
#: along a line, which is not a seam this module computes).
MIN_AXIS_TO_FACE_DEG = 10.0

#: A pair whose best end misses by more than the pose tolerance but within this many
#: tolerances is reported as a rejected seam with its fit-up ("fitup_beyond_pose_tol")
#: instead of dropped: on the 2026-10-09 bench S3 sat on R2 with its axis registered
#: 7 mm aside, -5.3..10.5 mm against 10, and nothing at all was shown.
NEAR_MISS = 2.0


def _near_miss(gap: np.ndarray, tol: float) -> str | None:
    if float(np.abs(gap).max()) <= tol:
        return None
    return (f"fitup_beyond_pose_tol ({gap.min():.1f}..{gap.max():.1f} mm, "
            f"pose_tol {tol:g} mm)")


def _kind(part) -> str:
    return type(part).__name__


def _unit_rows(v: np.ndarray) -> np.ndarray:
    return v / np.clip(np.linalg.norm(v, axis=1, keepdims=True), 1e-12, None)


def _local(part, pts: np.ndarray) -> np.ndarray:
    T = np.asarray(part.T_world_part, float)
    return (pts - T[:3, 3]) @ T[:3, :3]


def _radial(tube, pts: np.ndarray, outward: bool = True) -> np.ndarray:
    """The tube wall's exact normal at points on it (world)."""
    loc = _local(tube, pts)
    loc[:, 2] = 0.0
    n = _unit_rows(loc) * (1.0 if outward else -1.0)
    return n @ np.asarray(tube.T_world_part, float)[:3, :3].T


def _sample(curve, density_per_mm: float) -> np.ndarray:
    """Curve parameters, uniform in arclength (closed: one period, no wrap point)."""
    L = curve.length_mm
    n = (max(16, int(round(L * density_per_mm))) if curve.closed
         else max(2, int(round(L * density_per_mm)) + 1))
    return curve.t_at_arclength(curve.arclengths(n))


def _poly_length(pts: np.ndarray, closed: bool) -> float:
    p = np.vstack([pts, pts[:1]]) if closed else pts
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())


def _tube_ends(tube):
    """`(face, s, z_end)` for both ends: `z_end(phi, r)` is the end's local z on the
    radius-`r` wall, `s` = +1 when the body lies at z >= it (the base, cut or flat),
    -1 when at z <= it (the flat top cap)."""
    return [("-w", +1.0, lambda phi, r: tube.base_height(phi, r)),
            ("+w", -1.0, lambda phi, r: np.full(len(phi), float(tube.length_mm)))]


def _end_gap(tube, pts: np.ndarray, s: float, z_end, r: float) -> np.ndarray:
    """Signed distance along the tube axis from the curve to the tube's end, at each
    curve point: > 0 the end stops short of the base surface (gap), < 0 it runs in."""
    loc = _local(tube, pts)
    phi = np.arctan2(loc[:, 1], loc[:, 0])
    return s * (z_end(phi, r) - loc[:, 2])


def _on_slab_face(slab, pts: np.ndarray, tol: float) -> float:
    """Fraction of the points inside the slab's broad-face rectangle (+tol)."""
    loc = _local(slab, pts)
    half = np.asarray(slab.dims_mm, float) / 2.0
    ok = (np.abs(loc[:, 0]) <= half[0] + tol) & (np.abs(loc[:, 1]) <= half[1] + tol)
    return float(ok.mean())


def _verdict(role: str, pts, approach, cavity, parts, access, acc) -> tuple[bool, str | None, float]:
    from weldgen.verify_curved import seam_verdict
    tc = {**acc.DEFAULT_ACCESS["torch_clearance"],
          "bore_min_diameter_mm": float(access.get("bore_min_diameter_mm",
                                                   CELL_BORE_MIN_DIAMETER_MM))}
    seam = {"role": role, "points": pts, "approach": approach}
    if cavity is not None:
        seam["cavity_width_mm"] = float(cavity)
    ok, why, frac = seam_verdict(seam, parts, {"torch_clearance": tc})
    return bool(ok), why, float(frac)


def _record(face_pair, role, closed, pts, nA, nB, gap, member_id, parts, access, acc,
            cavity=None, member_end=None, reject=None) -> dict[str, Any]:
    """One seam in the `compute_seams` format, plus the per-point frames."""
    approach = nA + nB
    deg = np.linalg.norm(approach, axis=1) < 1e-9
    approach[deg] = nA[deg]                                  # butt-like: the face normal
    approach = _unit_rows(approach)
    dih = 180.0 - np.degrees(np.arccos(np.clip(np.einsum("ij,ij->i", nA, nB), -1.0, 1.0)))
    if reject is None:
        ok, why, frac = _verdict(role, pts, approach, cavity, parts, access, acc)
    else:
        ok, why, frac = False, reject, float("nan")
    L = _poly_length(pts, closed)
    first, last = pts[0], (pts[0] if closed else pts[-1])
    return {
        "id": -1, "face_pair": list(face_pair), "seam_class": "fillet", "role": role,
        "closed": bool(closed), "weldable": ok, "reject_reason": why,
        "clear_fraction": None if np.isnan(frac) else frac,
        "dihedral_deg": float(np.median(dih)),
        "dihedral_deg_range": [float(dih.min()), float(dih.max())],
        "separation_mm": float(np.median(gap)), "length_mm": L,
        "fitup_mm": {member_id: [float(gap.min()), float(gap.max())]},
        "member_end": member_end,
        "cavity_width_mm": None if cavity is None else float(cavity),
        "pose_tol_mm": float(access["pose_tol_mm"]),
        "p0_mm": [float(x) for x in first], "p1_mm": [float(x) for x in last],
        "n_a": [float(x) for x in nA[0]], "n_b": [float(x) for x in nB[0]],
        "approach": [float(x) for x in approach[0]],
        "polyline_mm": pts.astype(float).tolist(),
        "n_a_per_point": nA.astype(float).tolist(),
        "n_b_per_point": nB.astype(float).tolist(),
        "approach_per_point": approach.astype(float).tolist(),
    }


# --- tube on a plate ------------------------------------------------------------------

def _tube_on_slab(tube, slab, parts, access, density, acc) -> list[dict[str, Any]]:
    from weldgen.curves import ellipse_from_plane_cylinder
    tol = float(access["pose_tol_mm"])
    T = np.asarray(tube.T_world_part, float)
    z = T[:3, 2]
    best = None
    for face in ("+w", "-w"):
        pl = slab.face_plane(face)
        for end, s, z_end in _tube_ends(tube):
            # the body must stand on the face's outer side, steeply enough
            if s * float(pl.n @ z) < np.sin(np.radians(MIN_AXIS_TO_FACE_DEG)):
                continue
            curve = ellipse_from_plane_cylinder(-pl.d * pl.n, pl.n, T[:3, 3], z,
                                                tube.r_outer_mm)
            pts = curve.point(_sample(curve, density))
            gap = _end_gap(tube, pts, s, z_end, tube.r_outer_mm)
            worst = float(np.abs(gap).max())
            if worst <= NEAR_MISS * tol and (best is None or worst < best[0]):
                best = (worst, face, end, s, z_end, pl, pts, gap)
    if best is None:
        return []
    _, face, end, s, z_end, pl, pts, gap = best
    off_edge = _on_slab_face(slab, pts, tol) < 1.0
    miss = _near_miss(gap, tol)
    nA = np.tile(pl.n, (len(pts), 1))
    out = [_record((f"{slab.id}:{face}", f"{tube.id}:lateral+"), "weld", True, pts, nA,
                   _radial(tube, pts, True), gap, tube.id, parts, access, acc,
                   member_end=end, reject=miss or ("off_plate_edge" if off_edge else None))]
    # the bore meets the plate too: kept as the negative weldgen keeps
    bore = ellipse_from_plane_cylinder(-pl.d * pl.n, pl.n, T[:3, 3], z, tube.r_inner_mm)
    bp = bore.point(_sample(bore, density))
    out.append(_record((f"{slab.id}:{face}", f"{tube.id}:lateral-"), "bore", True, bp,
                       np.tile(pl.n, (len(bp), 1)), _radial(tube, bp, False),
                       _end_gap(tube, bp, s, z_end, tube.r_inner_mm), tube.id, parts,
                       access, acc, cavity=2.0 * tube.r_inner_mm, member_end=end,
                       reject=miss or ("off_plate_edge" if off_edge else None)))
    return out


# --- tube on a tube (saddle) -------------------------------------------------------------

def _tube_on_tube(branch, main, parts, access, density, acc) -> list[dict[str, Any]]:
    from weldgen.curves import saddle_from_cylinders
    if branch.r_outer_mm >= main.r_outer_mm:
        return []
    tol = float(access["pose_tol_mm"])
    Tb = np.asarray(branch.T_world_part, float)
    Tm = np.asarray(main.T_world_part, float)
    best = None
    for end, s, z_end in _tube_ends(branch):
        toward = -s * Tb[:3, 2]                     # from the body through this end
        try:
            curve = saddle_from_cylinders(Tb[:3, 3], toward, branch.r_outer_mm,
                                          Tm[:3, 3], Tm[:3, 2], main.r_outer_mm)
            pts = curve.point(_sample(curve, density))
        except ValueError:                          # misses or grazes the main pipe
            continue
        # the wrong end (main pipe behind the body) fails here: its gap is the length
        gap = _end_gap(branch, pts, s, z_end, branch.r_outer_mm)
        zm = _local(main, pts)[:, 2]
        on_main = bool(((zm >= -tol) & (zm <= main.length_mm + tol)).all())
        worst = float(np.abs(gap).max())
        if worst <= NEAR_MISS * tol and on_main and (best is None or worst < best[0]):
            best = (worst, end, s, z_end, toward, pts, gap)
    if best is None:
        return []
    _, end, s, z_end, toward, pts, gap = best
    miss = _near_miss(gap, tol)
    out = [_record((f"{main.id}:lateral+", f"{branch.id}:lateral+"), "weld", True, pts,
                   _radial(main, pts, True), _radial(branch, pts, True), gap, branch.id,
                   parts, access, acc, member_end=end, reject=miss)]
    try:
        bore = saddle_from_cylinders(Tb[:3, 3], toward, branch.r_inner_mm,
                                     Tm[:3, 3], Tm[:3, 2], main.r_outer_mm)
        bp = bore.point(_sample(bore, density))
        out.append(_record((f"{main.id}:lateral+", f"{branch.id}:lateral-"), "bore", True,
                           bp, _radial(main, bp, True), _radial(branch, bp, False),
                           _end_gap(branch, bp, s, z_end, branch.r_inner_mm), branch.id,
                           parts, access, acc, cavity=2.0 * branch.r_inner_mm,
                           member_end=end, reject=miss))
    except ValueError:
        pass
    return out


# --- a swept band on a plate -----------------------------------------------------------

def _spine_ccw(spine) -> bool:
    xy = spine.point(np.linspace(0.0, spine.t_period, 512, endpoint=False))[:, :2]
    return float(np.sum(xy[:, 0] * np.roll(xy[:, 1], -1)
                        - np.roll(xy[:, 0], -1) * xy[:, 1])) > 0.0


def _min_width(xy: np.ndarray) -> float:
    """The narrowest width of a closed outline over all directions (1 deg steps)."""
    th = np.radians(np.arange(0.0, 180.0, 1.0))
    proj = xy @ np.vstack([np.cos(th), np.sin(th)])
    return float((proj.max(axis=0) - proj.min(axis=0)).min())


def _band_on_slab(band, slab, parts, access, density, acc) -> list[dict[str, Any]]:
    from weldgen.curves import OffsetCurve
    tol = float(access["pose_tol_mm"])
    T = np.asarray(band.T_world_part, float)
    R, zw = T[:3, :3], T[:3, 2]
    walls = [(band.offset_lo_mm, -1.0, "-w"), (band.offset_hi_mm, +1.0, "+w")]
    best = None
    for face in ("+w", "-w"):
        pl = slab.face_plane(face)
        for cap, z_cap, s in (("-v", band.z0_mm, +1.0), ("+v", band.z1_mm, -1.0)):
            nz = float(pl.n @ zw)
            if s * nz < np.sin(np.radians(MIN_AXIS_TO_FACE_DEG)):
                continue
            traced = []
            for off, side, wface in walls:
                oc = OffsetCurve(band.spine, off)
                ts = _sample(oc, density)
                q = oc.point(ts)                                 # local, z = 0
                p0 = q @ R.T + T[:3, 3]
                lam = -(p0 @ pl.n + pl.d) / nz                   # along the band's axis
                pts = p0 + lam[:, None] * zw
                tan = band.spine.tangent(ts)
                n_out = side * np.column_stack([-tan[:, 1], tan[:, 0], np.zeros(len(ts))])
                traced.append((wface, side, pts, _unit_rows(n_out @ R.T),
                               s * (z_cap - lam), q))
            worst = max(float(np.abs(g).max()) for *_, g, _q in traced)
            if worst <= NEAR_MISS * tol and (best is None or worst < best[0]):
                best = (worst, face, cap, pl, traced)
    if best is None:
        return []
    _, face, cap, pl, traced = best
    closed = bool(band.spine.closed)
    exterior = ("-w" if _spine_ccw(band.spine) else "+w") if closed else None
    out = []
    for wface, side, pts, nB, gap, q in traced:
        off_edge = _on_slab_face(slab, pts, tol) < 1.0
        miss = _near_miss(gap, tol)
        role, cavity = "weld", None
        if closed and wface != exterior:
            role, cavity = "bore", _min_width(q[:, :2])
        out.append(_record((f"{slab.id}:{face}", f"{band.id}:{wface}"), role, closed, pts,
                           np.tile(pl.n, (len(pts), 1)), nB, gap, band.id, parts, access,
                           acc, cavity=cavity, member_end=cap,
                           reject=miss or ("off_plate_edge" if off_edge else None)))
    return out


# --- dispatch ------------------------------------------------------------------------

def _unsupported(A, B, access) -> dict[str, Any]:
    return {"id": -1, "face_pair": [A.id, B.id], "seam_class": "curved", "role": None,
            "closed": False, "weldable": False,
            "reject_reason": f"no seam rule for a {_kind(A)}-{_kind(B)} pair yet",
            "dihedral_deg": None, "separation_mm": None, "length_mm": 0.0,
            "fitup_mm": None, "pose_tol_mm": float(access["pose_tol_mm"]),
            "p0_mm": None, "p1_mm": None, "n_a": [0.0, 0.0, 0.0], "n_b": [0.0, 0.0, 0.0],
            "approach": None, "polyline_mm": []}


def pair_seams(A, B, parts: Sequence[Any], access: dict[str, Any], acc,
               density_per_mm: float = 1.0) -> list[dict[str, Any]]:
    """Every seam between two registered parts when at least one is curved. Empty when
    they do not meet within the pose tolerance; one non-weldable record naming the pair
    when no rule exists for it (a band on a pipe, two bands, ...)."""
    ka, kb = _kind(A), _kind(B)
    args = (parts, access, density_per_mm, acc)
    if {ka, kb} == {"Tube", "Slab"}:
        tube, slab = (A, B) if ka == "Tube" else (B, A)
        return _tube_on_slab(tube, slab, *args)
    if ka == kb == "Tube":
        return _tube_on_tube(A, B, *args) or _tube_on_tube(B, A, *args)
    if {ka, kb} == {"SweptSlab", "Slab"}:
        band, slab = (A, B) if ka == "SweptSlab" else (B, A)
        return _band_on_slab(band, slab, *args)
    return [_unsupported(A, B, access)]
