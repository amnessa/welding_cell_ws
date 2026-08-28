"""D4 verification arms and weld frames for the curved families — Phase 6b step 3.

Two jobs, kept deliberately separate:

**Rediscovery (the Phase 2 gate, curved).** `rediscover_seam` reads only the PLACED
PARTS — tube poses, radii, the plate's face plane — and intersects their surfaces with
the step-1 factories. Per the D29 curve-first amendment this is the only place
surface∩surface machinery runs: generation drew the curve and derived the parts, and
this arm independently computes what those parts meet along. `curve_distance` measures
the drawn-vs-rediscovered gap; the gate is machine precision for the intersection
families (#2–#4) and containment residual for the constructed ones (#5–#7), whose
curve is an input by D3 — the same asymmetry the plate pipeline always had, stated in
the patch.

**Weld frames (the seam emission core).** `curved_seam_set` returns every seam a
configuration carries — the weld line(s) AND the interior negatives (a tube's bore
meets the plate plane too; nothing can reach it, and keeping it is the D22 discipline
of free hard negatives) — each with per-point exact surface normals `nA`/`nB`, the
approach bisector, and the dihedral, all varying along the curve, which is what the
per-sample dihedral array in the 6b schema is for. `cone_clear_fractions` runs the D4
torch-cone test per sample point and is what turns a candidate into
`weldable`/`bisector_blocked`, exactly as the plate enumeration does — the verdict is
derived, never declared.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from .accessibility import DEFAULT_ACCESS, _cone_directions
from .curves import OffsetCurve, ellipse_from_plane_cylinder, saddle_from_cylinders
from .geom import SweptSlab, Tube
from .visibility import ray_hits_part

#: Ray origins lift this far off the seam along the cone axis before casting, mirroring
#: visibility.SURFACE_EPS_MM's role for surfaces.
_EPS_MM = 1e-3


def curve_distance(c1, c2, n: int = 720) -> float:
    """Symmetric Hausdorff-style distance between two curves (dense, exact points)."""
    p1 = c1.point(np.linspace(0.0, c1.t_period, n, endpoint=not c1.closed))
    p2 = c2.point(np.linspace(0.0, c2.t_period, n, endpoint=not c2.closed))
    d12 = np.sqrt(((p1[:, None, :] - p2[None, :, :]) ** 2).sum(-1))
    return float(max(d12.min(axis=1).max(), d12.min(axis=0).max()))


def rediscover_seam(built: dict[str, Any]):
    """Recompute the seam from the placed parts alone (families #2-#4).

    Reads part poses and radii - never the drawn curve - and returns the intersection
    curve of the placed surfaces. The generation gate asserts it matches the record.
    """
    cfg = built["config"]
    parts = built["parts"]
    if cfg in (2, 3):
        tube = next(p for p in parts if isinstance(p, Tube))
        plate = next(p for p in parts if p.id == "A")
        pl = plate.face_plane("+w")
        T = tube.T_world_part
        return ellipse_from_plane_cylinder(-pl.d * pl.n, pl.n,
                                           T[:3, 3], T[:3, 2], tube.r_outer_mm)
    if cfg == 4:
        main = next(p for p in parts if p.id == "A")
        branch = next(p for p in parts if p.id == "B")
        Tm, Tb = main.T_world_part, branch.T_world_part
        # the branch's local +z points AWAY from the main; the factory wants the axis
        # pointing toward it
        return saddle_from_cylinders(Tb[:3, 3], -Tb[:3, 2], branch.r_outer_mm,
                                     Tm[:3, 3], Tm[:3, 2], main.r_outer_mm)
    raise ValueError(f"family {cfg} is constructed, not intersected - "
                     f"verify by containment instead")


def containment_residual(built: dict[str, Any], n: int = 361) -> float:
    """Constructed families (#5-#7): worst inconsistency between the recorded curve
    and the parts derived from it - the consistency half of the gate. Their curve is
    an input by D3, so the check is that construction kept its promises: the curve
    lies in the base plane, and the parts' bands sit where the curve dictates."""
    cfg = built["config"]
    curve = built["curve"]
    pts = curve.point(np.linspace(0.0, curve.t_period, n, endpoint=not curve.closed))
    worst = float(np.abs(pts[:, 2]).max())                    # curve in the z=0 plane
    if cfg == 5:
        band = next(p for p in built["parts"] if isinstance(p, SweptSlab))
        # the curve must BE the outer wall: zero offset, and zero distance to the
        # band's spine surface at offset 0
        worst = max(worst, abs(band.offset_lo_mm))
        ts = np.linspace(0.0, band.spine.t_period, 4 * n, endpoint=False)
        ref = band.spine.point(ts)
        d = np.sqrt(((pts[:, None, :2] - ref[None, :, :2]) ** 2).sum(-1)).min(1)
        worst = max(worst, float(d.max()))
    elif cfg == 6:
        band = next(p for p in built["parts"] if isinstance(p, SweptSlab))
        worst = max(worst, abs(band.offset_hi_mm + band.offset_lo_mm))  # centred
    elif cfg == 7:
        g = built["gap_mm"]
        A = next(p for p in built["parts"] if p.id == "A")
        B = next(p for p in built["parts"] if p.id == "B")
        worst = max(worst, abs(A.offset_lo_mm - g / 2.0),
                    abs(B.offset_hi_mm + g / 2.0))            # gap centred on the curve
    return worst


# ------------------------------------------------------------------- weld frames


def _radial_normal(tube: Tube, pts: np.ndarray, outward: bool = True) -> np.ndarray:
    T = tube.T_world_part
    loc = (pts - T[:3, 3]) @ T[:3, :3]
    n2 = loc.copy()
    n2[:, 2] = 0.0
    n2 /= np.linalg.norm(n2, axis=1, keepdims=True)
    if not outward:
        n2 = -n2
    return n2 @ T[:3, :3].T


def _band_normal(band: SweptSlab, ts: np.ndarray, side: float) -> np.ndarray:
    """Outward normal of a SweptSlab offset face at spine parameters `ts`; `side`
    is +1 for the offset_hi face, -1 for offset_lo."""
    tan = band.spine.tangent(ts)
    n2 = np.column_stack([-tan[:, 1], tan[:, 0], np.zeros(len(tan))]) * side
    T = band.T_world_part
    return n2 @ T[:3, :3].T


def curved_seam_set(built: dict[str, Any], n: int = 96) -> list[dict[str, Any]]:
    """Every seam a realized configuration carries, with per-point exact frames.

    Each entry: `{curve, role, ts, points, tangent, nA, nB, approach, dihedral_deg}`
    where `role` is `"weld"` (candidate weld line), `"bore"` (the inner-wall
    interface - a REAL inner fillet on large pipe, a confined negative on small; the
    verdict is per scene, see `seam_verdict`), or `"toe"` (the curved butt's gap
    edges, the same negatives the plate butt keeps). Bore entries carry
    `cavity_width_mm`, the gate `seam_verdict` applies.
    Frames are analytic: `nA`/`nB` are the two surfaces' exact normals at each sample,
    `approach` their bisector, `dihedral_deg` their angle - all arrays, because on a
    curved seam every one of them varies along the curve.
    """
    cfg = built["config"]
    parts = built["parts"]
    z_up = np.array([0.0, 0.0, 1.0])
    out: list[dict[str, Any]] = []

    def entry(curve, role, nA_fn, nB_fn):
        ts = np.linspace(0.0, curve.t_period, n, endpoint=not curve.closed)
        pts = curve.point(ts)
        nA = nA_fn(ts, pts)
        nB = nB_fn(ts, pts)
        bis = nA + nB
        norm = np.linalg.norm(bis, axis=1, keepdims=True)
        approach = np.where(norm > 1e-9, bis / np.clip(norm, 1e-12, None),
                            nA)                                # degenerate: butt-like
        dih = np.degrees(np.arccos(np.clip(np.einsum("ij,ij->i", nA, nB),
                                           -1.0, 1.0)))
        out.append({"curve": curve, "role": role, "ts": ts, "points": pts,
                    "tangent": curve.tangent(ts), "nA": nA, "nB": nB,
                    "approach": approach, "dihedral_deg": 180.0 - dih})

    plate_n = lambda ts, pts: np.tile(z_up, (len(pts), 1))     # noqa: E731

    if cfg in (2, 3):
        tube = next(p for p in parts if isinstance(p, Tube))
        curve = built["curve"]
        entry(curve, "weld", plate_n,
              lambda ts, pts: _radial_normal(tube, pts, outward=True))
        # the bore also meets the plate plane: unreachable, kept as the negative
        pl = next(p for p in parts if p.id == "A").face_plane("+w")
        T = tube.T_world_part
        bore = ellipse_from_plane_cylinder(-pl.d * pl.n, pl.n, T[:3, 3], T[:3, 2],
                                           tube.r_inner_mm)
        entry(bore, "bore", plate_n,
              lambda ts, pts: _radial_normal(tube, pts, outward=False))
        out[-1]["cavity_width_mm"] = 2.0 * tube.r_inner_mm
    elif cfg == 4:
        main = next(p for p in parts if p.id == "A")
        branch = next(p for p in parts if p.id == "B")
        entry(built["curve"], "weld",
              lambda ts, pts: _radial_normal(main, pts, outward=True),
              lambda ts, pts: _radial_normal(branch, pts, outward=True))
        Tb = branch.T_world_part
        bore = saddle_from_cylinders(Tb[:3, 3], -Tb[:3, 2], branch.r_inner_mm,
                                     main.T_world_part[:3, 3],
                                     main.T_world_part[:3, 2], main.r_outer_mm)
        entry(bore, "bore",
              lambda ts, pts: _radial_normal(main, pts, outward=True),
              lambda ts, pts: _radial_normal(branch, pts, outward=False))
        out[-1]["cavity_width_mm"] = 2.0 * branch.r_inner_mm
    elif cfg == 5:
        band = next(p for p in parts if isinstance(p, SweptSlab))
        entry(built["curve"], "weld", plate_n,
              lambda ts, pts: _band_normal(band, ts, side=-1.0))   # outer wall
        inner = OffsetCurve(band.spine, band.offset_hi_mm)
        entry(inner, "bore", plate_n,
              lambda ts, pts: _band_normal(band, ts, side=+1.0))
        r = built["realization"]
        out[-1]["cavity_width_mm"] = (min(r["tube_w_mm"], r["tube_h_mm"])
                                      - 2.0 * band.offset_hi_mm)
    elif cfg == 6:
        band = next(p for p in parts if isinstance(p, SweptSlab))
        for off, side in ((band.offset_hi_mm, +1.0), (band.offset_lo_mm, -1.0)):
            entry(OffsetCurve(band.spine, off), "weld", plate_n,
                  lambda ts, pts, s=side: _band_normal(band, ts, side=s))
    elif cfg == 7:
        A = next(p for p in parts if p.id == "A")
        B = next(p for p in parts if p.id == "B")
        entry(built["curve"], "weld", plate_n, plate_n)        # coplanar centreline
        entry(OffsetCurve(A.spine, A.offset_lo_mm), "toe", plate_n,
              lambda ts, pts: _band_normal(A, ts, side=-1.0))
        entry(OffsetCurve(B.spine, B.offset_hi_mm), "toe", plate_n,
              lambda ts, pts: _band_normal(B, ts, side=+1.0))
    else:
        raise ValueError(f"no curved seam set for family {cfg}")
    return out


def seam_verdict(seam: dict[str, Any], parts,
                 access: dict | None = None) -> tuple[bool, str | None, float]:
    """`(weldable, reject_reason, clear_fraction)` — derived, never declared.

    Toes are the centreline's own negatives (`toe_of_centreline`, the plate butt
    discipline). Bore seams pass the LOCAL cone but must also admit the torch body
    into the cavity (`bore_min_diameter_mm`, the D13-style semantic gate — measured:
    an open 50 mm bore clears a 15 mm standoff cone, and what actually forbids the
    weld is that no torch fits). Everything else is the cone verdict, with the clear
    fraction recorded because a curved seam can be partially reachable.
    """
    acc = {**DEFAULT_ACCESS, **(access or {})}
    if seam["role"] == "toe":
        return False, "toe_of_centreline", float("nan")
    frac = cone_clear_fractions(seam, parts, access)
    if seam["role"] == "bore":
        min_d = float(acc["torch_clearance"].get("bore_min_diameter_mm", 80.0))
        if seam.get("cavity_width_mm", 0.0) < min_d:
            return False, "confined_bore", frac
    if frac < 0.95:
        return False, "bisector_blocked", frac
    return True, None, frac


def cone_clear_fractions(seam: dict[str, Any], parts,
                         access: dict | None = None) -> float:
    """Fraction of a seam's sample points whose torch cone clears every part - the D4
    verdict, per point because on a curved seam the approach rotates with it."""
    acc = {**DEFAULT_ACCESS, **(access or {})}
    half = float(acc["torch_clearance"]["half_angle_deg"])
    standoff = float(acc["torch_clearance"]["standoff_mm"])
    pts = seam["points"]
    axes = seam["approach"]
    ok = np.zeros(len(pts), dtype=bool)
    for i in range(len(pts)):
        dirs = _cone_directions(axes[i], half)
        o = np.tile(pts[i] + axes[i] * _EPS_MM, (len(dirs), 1))
        tmax = np.full(len(dirs), standoff)
        hit = np.zeros(len(dirs), dtype=bool)
        for p in parts:
            hit |= ray_hits_part(o, dirs, tmax, p)
        ok[i] = not hit.any()
    return float(ok.mean())
