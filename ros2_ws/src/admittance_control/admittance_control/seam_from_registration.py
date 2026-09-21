"""Mode A: the weld seam COMPUTED from the registered part poses (never detected).

The SEPC is the parts' CAD at their ICP poses. Once `pose_static` of every part is
known, the seams are the intersections of the placed parts' surfaces - closed form,
and decided for weldability by the same D4 accessibility rule `weld_generator` uses
to construct its ground truth. This module wraps that rule for the cell:

    objects  = [(model_name, pose_static_4x4_in_metres), ...]   # from assembly.json
    parts    = posed_parts(objects, registry)                   # weldgen primitives, mm
    seams    = compute_seams(parts, access)                     # every candidate, judged

Each seam carries its polyline (mm, static frame), class (fillet / butt / lap_toe /
edge), the cleared torch approach axis, `weldable` and, when not, the reject reason -
the negatives are kept because the planner must avoid them. The only error in a mode-A
seam is the registration's; carry the ICP metrics next to it.

The generator's D4 gates assume parts that touch exactly; registered parts do not. On
the bench the standing plate of a T-joint came back 8-11 mm INSIDE the base in one run
and 1-8 mm above it (tilted) in the next - ICP cannot observe a thin plate sliding in
its own plane. The seam line itself (the intersection of the two face PLANES) is
invariant to exactly that slide, so mode A judges each face pair with a pose tolerance
instead of a contact tolerance (`judge_registered`): the faces must reach the line
within `pose_tol_mm`, each face must reach past the other's plane by more than that
tolerance (which is what keeps a plate's own underside from pairing with the standing
plate when the tolerance exceeds the sheet thickness), and the gap or penetration of
the abutting edge is reported as `fitup_mm` - the fit-up diagnostic of the plan, and
the number the fiducial-board bound on the poses has to explain.

`weld_generator` is imported at call time: pass its path (or install it) - the ROS
node exposes `weldgen_path`. Nothing here imports ROS.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any, Sequence

import numpy as np

DEFAULT_WELDGEN_PATH = "/workspaces/welding_cell_ws/weld_generator"


class RegistryError(RuntimeError):
    """A part in the assembly has no usable registry entry (-> mode B / radius-PCA)."""


def import_weldgen(path: str | None = None):
    try:
        import weldgen  # noqa: F401
    except ImportError:
        p = str(path or DEFAULT_WELDGEN_PATH)
        if p not in sys.path:
            sys.path.insert(0, p)
        import weldgen  # noqa: F401
    import weldgen.accessibility as acc
    import weldgen.geom as geom
    import weldgen.sampling as sampling
    return acc, geom, sampling


def _stem(model_name: str) -> str:
    return Path(model_name).stem


def posed_parts(objects: Sequence[tuple[str, np.ndarray]], registry: dict[str, Any],
                pose_units: str = "m", weldgen_path: str | None = None) -> list:
    """weldgen primitives for the saved objects, in the static frame, in millimetres.

    `pose_static` maps the CAD frame -> static frame; the registry's `T_cad_prim` maps
    the primitive frame -> CAD frame; the primitive is posed by their product. Poses
    arrive in metres from the node and are converted; the registry is in mm.
    """
    _, geom, _ = import_weldgen(weldgen_path)
    scale = 1000.0 if pose_units == "m" else 1.0
    parts_reg = registry.get("parts", registry)
    out = []
    missing = []
    for i, (name, pose) in enumerate(objects):
        entry = parts_reg.get(_stem(name))
        if not entry or not entry.get("primitive"):
            missing.append(f"{name}: {(entry or {}).get('reason', 'no registry entry')}")
            continue
        T = np.asarray(pose, dtype=float).copy()
        T[:3, 3] *= scale
        T_world_part = T @ np.asarray(entry["T_cad_prim"], dtype=float)
        obj = {"id": chr(ord("A") + i), "role": "workpiece", "object_id": i,
               "primitive": entry["primitive"], "T_world_part": T_world_part.tolist()}
        for k in ("dims_mm", "thickness_mm", "outline_uv", "outline_shape", "params"):
            if entry.get(k) is not None:
                obj[k] = entry[k]
        out.append(geom.from_object(obj))
    if missing:
        raise RegistryError("; ".join(missing))
    return out


def runtime_access(parts, pose_tol_mm: float = 10.0,
                   min_seam_length_mm: float = 10.0, **legacy) -> dict[str, Any]:
    """The D4 access settings for parts at REGISTERED poses.

    `pose_tol_mm` is how far a registered face may miss the other part and still be
    the same joint: the relative pose error of the two registrations, not the root gap
    (the generator's `contact_tol_mm`, which it sets from the gap it drew). Until the
    fiducial-board bound measures it, 10 mm covers what the bench has shown (8-11 mm).
    It is NOT capped below the sheet thickness the way the generator caps it: the
    wrap-around phantoms that cap prevents are rejected by `judge_registered`'s
    face-extent rule instead, so the tolerance can stay honest on 2 mm sheet - at the
    stated price that a lapping sheet thinner than the tolerance is undecidable.
    """
    if "contact_tol_mm" in legacy and legacy["contact_tol_mm"] is not None:
        pose_tol_mm = float(legacy["contact_tol_mm"])
    t_min = min(float(p.thickness_mm) for p in parts) if parts else pose_tol_mm
    tol = float(np.clip(pose_tol_mm, 0.5, 30.0))
    return {"contact_tol_mm": tol, "pose_tol_mm": tol,
            # "same plane?" for coplanar (butt / edge) pairs: the pose error, but never
            # more than half a sheet or two faces of one plate become one plane
            "coplanar_tol_mm": float(min(tol, 0.5 * t_min)),
            "min_seam_length_mm": float(min_seam_length_mm)}


def _face_corners(part, face: str) -> np.ndarray:
    """The four world corners of a slab face."""
    half = np.asarray(part.dims_mm, dtype=float) / 2.0
    axis = "uvw".index(face[1]); sign = 1.0 if face[0] == "+" else -1.0
    others = [k for k in range(3) if k != axis]
    corners = []
    for sa in (-1.0, 1.0):
        for sb in (-1.0, 1.0):
            c = np.zeros(3); c[axis] = sign * half[axis]
            c[others[0]] = sa * half[others[0]]; c[others[1]] = sb * half[others[1]]
            corners.append(c)
    T = part.T_world_part
    return np.asarray(corners) @ T[:3, :3].T + T[:3, 3]


def _edge_fitup(part, face: str, plane) -> tuple[float, float]:
    """Signed distance (mm) of the face's near edge to the other part's plane: the two
    corners closest to that plane, sorted. Positive = gap, negative = penetration."""
    d = np.sort(plane.n @ _face_corners(part, face).T + plane.d)
    return float(d[0]), float(d[1])


def _extends_beyond(part, face: str, plane, pose_tol: float) -> str:
    """Does this member's face reach past the other's plane by more than the tolerance?

    `"yes"` when the face's far corner lies more than `pose_tol` in front of the plane;
    `"behind"` when the face lies on the far side (nothing in front, or more of it
    behind than in front); `"short"` in between. A genuine joint
    has both members' faces crossing each other's plane: the standing plate of a T
    reaches 90 mm above the base, the base reaches 120 mm past the plate. A plate's
    own underside paired with the standing plate is `"behind"` - the plate is on the
    other side of the sheet - which is the wrap-around phantom the generator excludes
    by capping its tolerance below the sheet thickness; here the tolerance is the pose
    error and may exceed the sheet, so the geometry has to say it. `"short"` is the
    honest limit: a lapping sheet thinner than the pose error cannot be told from a
    penetrating one, so its toes are undecidable until the pose bound is tighter than
    the sheet (`member_within_pose_tol`).
    """
    d = plane.n @ _face_corners(part, face).T + plane.d
    front, back = float(d.max()), float(-d.min())
    if front > pose_tol:
        return "yes"
    # The base of a T reaches far past the standing plate on BOTH sides, so `back` may
    # only speak once `front` has failed: a face that pokes a few mm through the other
    # part's far side (a penetrating standing plate seen from the underside) has
    # `back` of the whole plate height against a `front` of the penetration.
    return "behind" if (front <= 1e-6 or back > front) else "short"


def clip_registered(part, face: str, point: np.ndarray, direction: np.ndarray,
                    slack_mm: float) -> tuple[float, float] | None:
    """`Slab.face_clip_line` for a face at a REGISTERED pose.

    The generator's clip grants the slack only to an in-face coordinate the line holds
    exactly constant, and waives a near-constant one only when the strict clip is
    empty - right for exact geometry, where a seam that overhangs its plate by the
    slack would then be measured as a separation. At registered poses the standing
    plate is tilted end to end (2 mm into the base at one end, 11 mm above it at the
    other, measured), so the seam line crosses its side face at a small angle: the
    strict clip keeps only the run where the plate reaches the base plane (44 of
    250 mm), and the waiver never fires because that run is not empty. Here the
    in-face axis the line holds nearly constant is granted the slack as an interval:
    the seam runs as far as that member stays within the tolerance, and stops where
    it floats further. The other axis stays strict, so the run is never extended
    along the seam. Non-slab parts fall back to their own clip.
    """
    if type(part).__name__ != "Slab":
        return part.face_clip_line(face, point, direction, slack_mm=slack_mm)
    half = np.asarray(part.dims_mm, dtype=float) / 2.0
    R = part.T_world_part[:3, :3]
    c = part.T_world_part[:3, 3]
    p_loc = (np.asarray(point, dtype=float) - c) @ R
    d_loc = np.asarray(direction, dtype=float) @ R
    axis = "uvw".index(face[1])
    in_face = [k for k in range(3) if k != axis]
    slack = float(slack_mm)

    # The in-face axis the line holds NEARLY constant (the smaller direction component)
    # gets the slack: its interval is the run over which its breach stays within the
    # tolerance. The other axis is clipped strictly, so the run is never extended
    # along the seam. Exactly parallel lines reduce to the generator's rule.
    k_const = min(in_face, key=lambda k: abs(d_loc[k]))

    def axis_interval(k):
        grow = slack if k == k_const else 0.0
        if abs(d_loc[k]) < 1e-12:
            if abs(p_loc[k]) > half[k] + grow + 1e-9:
                return None
            return (-np.inf, np.inf)
        a = (-half[k] - grow - p_loc[k]) / d_loc[k]
        b = (half[k] + grow - p_loc[k]) / d_loc[k]
        return (min(a, b), max(a, b))

    ivs = [axis_interval(k) for k in in_face]
    if any(v is None for v in ivs):
        return None
    lo = max(v[0] for v in ivs)
    hi = min(v[1] for v in ivs)
    if hi - lo > 1e-9 and np.isfinite(lo) and np.isfinite(hi):
        return float(lo), float(hi)
    return None


def judge_registered(A, fa: str, B, fb: str, ref: tuple[str, str], solids, access,
                     acc, samples_along: int = 5):
    """D4 for two faces at registered poses; returns (Candidate, fitup) or None.

    Same arms as `weldgen.accessibility._judge`; the differences are the ones the
    registration forces: the clip is `clip_registered` (slack on a tilted member's
    near-constant coordinate), the separation gate is the pose tolerance,
    the mutual-visibility probe (which needs the solids not to interpenetrate) is
    replaced by the material probe both ways, and the abutting edge's fit-up is measured
    and returned. A penetration is registration error, not geometry, so it does not
    reject - it is reported.
    """
    pa, pb = A.face_plane(fa), B.face_plane(fb)
    if pa is None or pb is None:
        return None
    na, nb = pa.n, pb.n
    tol = float(access["pose_tol_mm"])
    fixture_involved = A.role == "fixture" or B.role == "fixture"
    if float(np.linalg.norm(np.cross(na, nb))) < acc.PARALLEL_TOL:
        if float(na @ nb) > 0.5:
            c = acc._coplanar_candidate(A, fa, B, fb, ref, solids, access,
                                        fixture_involved, samples_along)
            return None if c is None else (c, None)
        sep = abs(float(pa.signed_distance(B.face_center(fb))))
        if sep > tol or not acc._faces_overlap(A, fa, B, fb):
            return None
        return (acc.Candidate(face_pair=ref, weldable=False,
                              reject_reason="fixture_contact" if fixture_involved
                              else "bisector_blocked",
                              p0=None, p1=None, n_a=na, n_b=nb,
                              dihedral_deg=180.0, separation_mm=sep), None)

    hit = acc.intersect_planes(pa, pb)
    if hit is None:
        return None
    point, direction = hit
    ia = clip_registered(A, fa, point, direction, tol)
    ib = clip_registered(B, fb, point, direction, tol)
    if ia is None or ib is None:
        return None
    lo, hi = max(ia[0], ib[0]), min(ia[1], ib[1])
    if hi - lo <= 1e-9:
        return None
    p0 = point + lo * direction
    p1 = point + hi * direction
    sep = acc._line_separation(A, fa, B, fb, p0, p1, samples_along)
    if sep > tol:
        return None
    reach = (_extends_beyond(A, fa, pb, tol), _extends_beyond(B, fb, pa, tol))
    if "behind" in reach:
        return None                    # one face is on the far side of the other part

    fitup = {A.id: _edge_fitup(A, fa, pb), B.id: _edge_fitup(B, fb, pa)}
    theta = acc.dihedral_deg(na, nb)
    bis = na + nb
    bis_norm = float(np.linalg.norm(bis))
    reason = None
    approach = None
    if "short" in reach:
        reason = "member_within_pose_tol"
    elif fixture_involved:
        reason = "fixture_contact"
    elif not (access["dihedral_min_deg"] <= theta <= access["dihedral_max_deg"]):
        reason = "degenerate_dihedral"
    elif hi - lo < access["_min_len"]:
        reason = "too_short"
    elif bis_norm < 1e-9:
        reason = "bisector_blocked"
    else:
        mids = np.linspace(0.0, 1.0, samples_along)[:, None]
        pts = p0[None, :] * (1 - mids) + p1[None, :] * mids
        approach = acc._clear_axis(solids, pts, bis / bis_norm, (na, nb), access)
        if approach is None:
            reason = "bisector_blocked"
    c = acc.Candidate(face_pair=ref, weldable=reason is None, reject_reason=reason,
                      p0=p0, p1=p1, n_a=na, n_b=nb, dihedral_deg=theta,
                      separation_mm=sep, approach=approach)
    return c, fitup


def enumerate_registered(parts, access: dict[str, Any], acc, samples_along: int = 5
                         ) -> list[tuple[Any, Any]]:
    """`enumerate_candidates` with `judge_registered`, same post-passes and ordering."""
    access = dict({**acc.DEFAULT_ACCESS, **access})
    access["_min_len"] = float(access["min_seam_length_mm"])
    solids = list(parts)
    out: list[tuple[Any, Any]] = []
    for i, A in enumerate(parts):
        for j, B in enumerate(parts):
            if j <= i:
                continue
            for fa in A.face_names():
                for fb in B.face_names():
                    ref = tuple(sorted((f"{A.id}:{fa}", f"{B.id}:{fb}")))
                    r = judge_registered(A, fa, B, fb, ref, solids, access, acc, samples_along)
                    if r is not None:
                        out.append(r)
    cands = [c for c, _ in out]
    acc._suppress_toes(cands, access)
    acc._drop_cross_runs(cands, frozenset(p.id for p in parts if p.role == "workpiece"),
                         access)
    out.sort(key=lambda cf: (
        not cf[0].primary, not cf[0].weldable, cf[0].face_pair[0], cf[0].face_pair[1],
        tuple(np.round(cf[0].p0, 6)) if cf[0].p0 is not None else (0.0, 0.0, 0.0)))
    return out


def compute_seams(parts, access: dict[str, Any] | None = None,
                  density_per_mm: float = 1.0, weldgen_path: str | None = None
                  ) -> list[dict[str, Any]]:
    """Every inter-part face pair, judged by the registration-tolerant D4 rule (mm)."""
    acc, _, sampling = import_weldgen(weldgen_path)
    access = access or runtime_access(parts)
    seams = []
    for k, (c, fitup) in enumerate(enumerate_registered(parts, access, acc)):
        poly = (sampling.sample_polyline(c.p0, c.p1, density_per_mm)
                if c.p0 is not None and c.length_mm > 1e-6 else np.zeros((0, 3)))
        seams.append({
            "id": k, "face_pair": list(c.face_pair), "seam_class": c.seam_class,
            "weldable": bool(c.weldable), "reject_reason": c.reject_reason,
            "dihedral_deg": float(c.dihedral_deg), "separation_mm": float(c.separation_mm),
            "length_mm": float(c.length_mm),
            # signed mm of each member's near edge to the other's face plane, per part
            # id: > 0 gap, < 0 penetration; the abutting member is the one within tol
            "fitup_mm": None if fitup is None else {k_: [float(a), float(b)]
                                                   for k_, (a, b) in fitup.items()},
            "pose_tol_mm": float(access["pose_tol_mm"]),
            "p0_mm": None if c.p0 is None else [float(x) for x in c.p0],
            "p1_mm": None if c.p1 is None else [float(x) for x in c.p1],
            "n_a": [float(x) for x in c.n_a], "n_b": [float(x) for x in c.n_b],
            "approach": None if c.approach is None else [float(x) for x in c.approach],
            "polyline_mm": poly.astype(float).tolist(),
        })
    seams.sort(key=lambda s: (not s["weldable"], -s["length_mm"]))
    for k, s in enumerate(seams):
        s["id"] = k
    return seams


def seams_points_m(seams: Sequence[dict[str, Any]], weldable_only: bool = True
                   ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """`(xyz_m, rgb_uint8, seam_index)` for a PointCloud2: weldable seams in reds,
    rejected ones grey - one colour per seam so RViz tells them apart."""
    pts, cols, idx = [], [], []
    reds = [(255, 20, 20), (255, 120, 20), (230, 20, 160), (255, 200, 20)]
    for s in seams:
        if weldable_only and not s["weldable"]:
            continue
        p = np.asarray(s["polyline_mm"], dtype=float)
        if len(p) == 0:
            continue
        c = reds[s["id"] % len(reds)] if s["weldable"] else (140, 140, 140)
        pts.append(p / 1000.0)
        cols.append(np.tile(np.array(c, np.uint8), (len(p), 1)))
        idx.append(np.full(len(p), s["id"], dtype=np.int32))
    if not pts:
        return np.zeros((0, 3)), np.zeros((0, 3), np.uint8), np.zeros(0, np.int32)
    return np.vstack(pts), np.vstack(cols), np.concatenate(idx)


def abutting_fitup(seam: dict[str, Any]) -> tuple[str, float, float] | None:
    """`(part_id, lo, hi)` of the member whose edge abuts the other's face, or None."""
    f = seam.get("fitup_mm")
    if not f:
        return None
    tol = float(seam.get("pose_tol_mm", 10.0))
    best = min(f.items(), key=lambda kv: min(abs(kv[1][0]), abs(kv[1][1])))
    lo, hi = best[1]
    if min(abs(lo), abs(hi)) > tol:
        return None
    return best[0], lo, hi


def _fitup_note(seam: dict[str, Any]) -> str:
    f = abutting_fitup(seam)
    if f is None:
        return ""
    _, lo, hi = f
    if hi < 0:
        return f" penetration {-hi:.1f}..{-lo:.1f}mm"
    if lo > 0:
        return f" gap {lo:.1f}..{hi:.1f}mm"
    return f" fit-up {lo:.1f}..{hi:.1f}mm"


def summarize(seams: Sequence[dict[str, Any]]) -> str:
    w = [s for s in seams if s["weldable"]]
    parts = [f"{s['seam_class']} {s['length_mm']:.0f}mm ({s['face_pair'][0]}x{s['face_pair'][1]})"
             + _fitup_note(s) for s in w]
    rej = {}
    for s in seams:
        if not s["weldable"]:
            rej[s["reject_reason"]] = rej.get(s["reject_reason"], 0) + 1
    return (f"{len(w)} weldable seam(s): " + ", ".join(parts) +
            (f"; rejected {rej}" if rej else ""))
