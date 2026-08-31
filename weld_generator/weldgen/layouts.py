"""Placement for the five joint types — PARAMETERS.md §2.6-§2.7.

Each layout returns the two workpiece slabs posed in the canonical joint frame:

    +X  along the seam
    +Z  up, out of part A's top face
    +Y  toward part B

The constructors do placement and nothing else. Every seam is then derived by the D4
enumeration in `accessibility.py`, which is what makes the Phase 2 gate meaningful: the
rule rediscovers the joint rather than being told what it is.

Cross-sections, `g` = root gap, `h` = linear misalignment, `s` = stack offset:

    T                corner            butt              lap              edge
      ██               ██                                  ▓▓▓▓▓▓          ▓▓▓▓
      ██               ██                                ▓▓▓▓▓▓            ▓▓▓▓
    ██████         ████                ████  ████      ██████            ██████
                                                                         (flush)
"""

from __future__ import annotations

import numpy as np

from .geom import PreparedSlab, Prism, Slab, rot_x, rot_z, translate
from .joints import JointSpec

JOINT_TYPES = ("T", "corner", "butt", "lap", "edge")

#: D31 - two boundary features are "the same edge" only when near-parallel. An edge
#: crossing a clearance band at an angle is overhang, not a class-ambiguous meeting;
#: same logic (near AND parallel) the D28 gate uses for seam-bearing edges.
AMBIGUITY_PARALLEL_DEG = 10.0


def class_clearance_mm(spec: JointSpec) -> float:
    """D31 clearance: the flat surface an intended fillet weld needs beside a contact
    line. ISO 17659 separates the classes by where the parts meet - on a face (T 3.10,
    lap 3.9) or at their edges (corner 3.13, edge 3.14) - and the clearance
    operationalises "meets the face": leg length (ISO 17659 3.21) is z ~= t of the
    thinner member, so 2t keeps room for a fillet on the outer side."""
    return 2.0 * min(spec.t_A, spec.t_B)


def _seg_seg_dist_2d(a0, a1, b0, b1) -> float:
    """Min distance between two 2-D segments (sampled - plate-scale tolerances)."""
    ts = np.linspace(0.0, 1.0, 17)[:, None]
    pa = a0[None, :] * (1 - ts) + a1[None, :] * ts
    d = b1 - b0
    L2 = float(d @ d)
    u = np.clip((pa - b0[None, :]) @ d / max(L2, 1e-12), 0.0, 1.0)
    return float(np.linalg.norm(pa - (b0[None, :] + u[:, None] * d[None, :]),
                                axis=1).min())


def class_ambiguity(spec: JointSpec, joint_type: str) -> list[str]:
    """D31: which OTHER joint types this sampled configuration resembles.

    Empty for a cleanly in-class configuration. Non-empty routes the seed either to
    rejection (`class_disjoint` corpora) or to the D32 class-boundary stratum
    (`class_boundary_stratum` corpora), where the scene records `ambiguous_with`.

    The tests are near-AND-parallel throughout: a boundary meeting counts only when the
    two features run within `AMBIGUITY_PARALLEL_DEG` of each other AND within the D31
    clearance `c = 2*min(t_A, t_B)`. Corner needs no test (ruled 2026-08-27): B past A's
    edge IS the class definition (ISO 17659 3.13), the seam stays at the corner. Butt has
    no bordering class.
    """
    c = class_clearance_mm(spec)
    out: list[str] = []

    if joint_type == "T":
        # B's contact centreline (yawed segment) vs A's four boundary edges: running
        # near-parallel within c of one is the corner-like configuration. End-overhang
        # is allowed - a crossing at an angle never triggers the parallel gate.
        yaw = np.radians(spec.in_plane_yaw_deg)
        d = np.array([np.cos(yaw), np.sin(yaw)])
        pc = np.array([spec.length_offset_mm,
                       spec.linear_misalignment_mm + spec.t_B / 2.0])
        p0, p1 = pc - d * spec.L_B / 2.0, pc + d * spec.L_B / 2.0
        hx, hy = spec.L_A / 2.0, spec.W_A / 2.0
        edges = [(np.array([-hx, -hy]), np.array([hx, -hy]), 0.0),
                 (np.array([-hx, hy]), np.array([hx, hy]), 0.0),
                 (np.array([-hx, -hy]), np.array([-hx, hy]), 90.0),
                 (np.array([hx, -hy]), np.array([hx, hy]), 90.0)]
        for e0, e1, edge_ang in edges:
            rel = abs(((np.degrees(yaw) - edge_ang) + 90.0) % 180.0 - 90.0)
            if rel < AMBIGUITY_PARALLEL_DEG                     and _seg_seg_dist_2d(p0, p1, e0, e1) < c:
                out.append("corner")
                break

    elif joint_type == "lap":
        # Generic under full-circle yaw: any boundary edge of B's YAWED footprint
        # running near-parallel within c of a boundary edge of A is a flush-coincidence
        # (edge-like) configuration. This subsumes the three yaw-0 special cases of the
        # first draft (overlap < c, flush at A's far edge, end coincidence) and catches
        # what the special cases could not: at yaw ~ 180 the rotation about the strip
        # centroid lands B's leading edge exactly on A's welded edge - always flush, so
        # laps genuinely cannot take yaw within ~10 deg of 180.
        overlap = float(spec.stack_offset_mm or 0.0)
        if spec.outline_B is not None:
            o = np.asarray(spec.outline_B, dtype=float)
            fp = np.column_stack([o[:, 0] + spec.length_offset_mm,
                                  o[:, 1] - overlap])
        else:
            x0, x1 = (spec.length_offset_mm - spec.L_B / 2.0,
                      spec.length_offset_mm + spec.L_B / 2.0)
            fp = np.array([[x0, -overlap], [x1, -overlap],
                           [x1, spec.H_B - overlap], [x0, spec.H_B - overlap]])
        yaw = np.radians(spec.in_plane_yaw_deg)
        R = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        pivot = np.array([spec.length_offset_mm, -overlap / 2.0])
        fp = (fp - pivot) @ R.T + pivot
        hx = spec.L_A / 2.0
        a_edges = [(np.array([-hx, 0.0]), np.array([hx, 0.0])),
                   (np.array([-hx, -spec.W_A]), np.array([hx, -spec.W_A])),
                   (np.array([-hx, 0.0]), np.array([-hx, -spec.W_A])),
                   (np.array([hx, 0.0]), np.array([hx, -spec.W_A]))]
        k = len(fp)
        done = False
        for i in range(k):
            b0, b1 = fp[i], fp[(i + 1) % k]
            e = b1 - b0
            ang_b = np.degrees(np.arctan2(e[1], e[0]))
            for a0, a1 in a_edges:
                ea = a1 - a0
                ang_a = np.degrees(np.arctan2(ea[1], ea[0]))
                rel = abs(((ang_b - ang_a) + 90.0) % 180.0 - 90.0)
                if rel < AMBIGUITY_PARALLEL_DEG \
                        and _seg_seg_dist_2d(b0, b1, a0, a1) < c:
                    out.append("edge")
                    done = True
                    break
            if done:
                break

    elif joint_type == "edge":
        # The welded edge is exactly flush by construction; every NON-welded pair must
        # clear by c so each boundary is unambiguously flush-seam or lap toe.
        if abs(spec.W_A - spec.H_B) > 1e-9 and abs(spec.W_A - spec.H_B) < c:
            out.append("lap")                        # far edges nearly (not exactly) flush
        else:
            for be in (spec.length_offset_mm - spec.L_B / 2.0,
                       spec.length_offset_mm + spec.L_B / 2.0):
                d_end = min(abs(be - spec.L_A / 2.0), abs(be + spec.L_A / 2.0))
                if 1e-9 < d_end < c:
                    out.append("lap")
                    break

    return out


def build(spec: JointSpec, joint_type: str, T_world_joint: np.ndarray) -> list[Slab]:
    """Place parts A and B for `joint_type`."""
    try:
        fn = _LAYOUTS[joint_type]
    except KeyError:
        raise ValueError(f"unknown joint type {joint_type!r}") from None
    return fn(spec, T_world_joint)


#: D28 outline vocabulary (patch_phase6). `rectangle` is what you get with outlines OFF,
#: so the sampler draws only the shapes that break the axis-alignment prior. Trapezoid
#: and parallelogram keep a far edge exactly parallel to the seam - they are in D28's
#: own vocabulary, so they stay - but they are deliberately a minority (2 of 5): the
#: corpus gate showed the parallel far edge is often the nearest free boundary edge, so
#: a vocabulary dominated by those two rebuilds the 0-degree spike it was meant to break.
OUTLINE_SHAPES = ("trapezoid", "parallelogram", "triangle", "quad", "convex")


def _is_convex_ccw(verts: np.ndarray) -> bool:
    v = np.asarray(verts, dtype=float)
    k = len(v)
    for i in range(k):
        a, b, c = v[i], v[(i + 1) % k], v[(i + 2) % k]
        e1, e2 = b - a, c - b
        if e1[0] * e2[1] - e1[1] * e2[0] <= 1e-9:   # 2-D cross, NumPy-2 safe
            return False
    return True


def sample_outline(rng: np.random.Generator, L: float, depth: float
                   ) -> tuple[str, tuple[tuple[float, float], ...]]:
    """One D28 outline in the canonical frame: seam edge (-L/2,0)->(L/2,0), body upward.

    The seam-bearing edge is pinned at full length - it is the one boundary the joint
    type constrains - and every shape reaches `depth`, so the part's sampled width keeps
    meaning what it meant for a slab (the outline redistributes area, it does not shrink
    the part). All draws come from the caller's stream (`seam_curve` via `sample_joint`),
    so outline-off corpora reproduce bit-identically and an outline-enabled regeneration
    is a free twin, exactly like yaw.
    """
    # Deliberately non-uniform: trapezoid and parallelogram keep a long far edge
    # exactly parallel to the seam (their identity), and the corpus gate showed a
    # uniform vocabulary puts ~35% of free-edge length in the 0-10 degree bin. They
    # stay in the vocabulary as D28 lists them, as a minority.
    shape = str(rng.choice(OUTLINE_SHAPES, p=[0.10, 0.10, 0.25, 0.30, 0.25]))
    base = [(-L / 2.0, 0.0), (L / 2.0, 0.0)]
    if shape == "trapezoid":
        a = float(rng.uniform(0.15, 0.55)) * float(rng.choice([-1.0, 1.0])) * L
        b = float(rng.uniform(0.15, 0.55)) * float(rng.choice([-1.0, 1.0])) * L
        u_l, u_r = -L / 2.0 + a, L / 2.0 - b
        if u_r - u_l < 0.2 * L:                       # keep a real far edge
            mid = 0.5 * (u_l + u_r)
            u_l, u_r = mid - 0.1 * L, mid + 0.1 * L
        verts = base + [(u_r, depth), (u_l, depth)]
    elif shape == "parallelogram":
        sh = float(rng.uniform(0.25, 0.65)) * float(rng.choice([-1.0, 1.0])) * L
        verts = base + [(L / 2.0 + sh, depth), (-L / 2.0 + sh, depth)]
    elif shape == "triangle":
        verts = base + [(float(rng.uniform(-0.4, 0.4)) * L, depth)]
    elif shape == "quad":
        # General convex quadrilateral: the far edge is TILTED (one end at full depth,
        # the other well short of it), so unlike the trapezoid nothing up there is
        # parallel to the seam.
        verts = None
        for _ in range(8):
            d_lo = float(rng.uniform(0.55, 0.85)) * depth
            u_r = L / 2.0 - float(rng.uniform(-0.2, 0.35)) * L
            u_l = -L / 2.0 + float(rng.uniform(-0.2, 0.35)) * L
            if bool(rng.random() < 0.5):
                cand = base + [(u_r, depth), (u_l, d_lo)]
            else:
                cand = base + [(u_r, d_lo), (u_l, depth)]
            if _is_convex_ccw(np.asarray(cand)):
                verts = cand
                break
        if verts is None:                             # pragma: no cover - ~never after 8
            verts = base + [(0.3 * L, depth), (-0.4 * L, 0.7 * depth)]
            shape = "quad"
    else:                                             # general convex pentagon
        verts = None
        for _ in range(8):                            # deterministic redraws, same stream
            u_r = L / 2.0 + float(rng.uniform(-0.05, 0.3)) * L
            d_r = float(rng.uniform(0.35, 0.75)) * depth
            u_m = float(rng.uniform(-0.3, 0.3)) * L
            u_l = -L / 2.0 - float(rng.uniform(-0.05, 0.3)) * L
            d_l = float(rng.uniform(0.35, 0.75)) * depth
            cand = base + [(u_r, d_r), (u_m, depth), (u_l, d_l)]
            if _is_convex_ccw(np.asarray(cand)):
                verts = cand
                break
        if verts is None:                             # pragma: no cover - ~never after 8
            verts = base + [(L / 2.0, depth), (-L / 2.0, depth)]
            shape = "trapezoid"
    return shape, tuple((float(u), float(v)) for u, v in verts)


def _outline_local(outline, depth: float, seam_at_plus_v: bool) -> np.ndarray:
    """Canonical outline -> the part's centred local u-v frame.

    The layouts place CENTRED slabs, so the prism reuses the exact same transform chains
    by living in the same centred coordinates: the canonical body [0, depth] maps onto
    [-depth/2, +depth/2] with the seam edge landing on whichever side the layout welds.
    A mirrored mapping flips the winding; `Prism.__post_init__` restores CCW.
    """
    o = np.asarray(outline, dtype=float)
    if seam_at_plus_v:
        return np.column_stack([o[:, 0], depth / 2.0 - o[:, 1]])
    return np.column_stack([o[:, 0], o[:, 1] - depth / 2.0])


def _part_B(spec: JointSpec, T_B: np.ndarray, seam_at_plus_v: bool):
    if spec.outline_B is None:
        return Slab("B", "workpiece", 1, (spec.L_B, spec.H_B, spec.t_B), T_B)
    return Prism("B", "workpiece", 1,
                 _outline_local(spec.outline_B, spec.H_B, seam_at_plus_v),
                 spec.t_B, T_B, shape=spec.outline_shape_B)


def _standing_B(spec: JointSpec, T: np.ndarray, y0: float, z0: float,
                joint_type: str) -> Slab:
    """A plate standing on edge, its near face at y0 and its bottom at z0.

    rot_x(90) carries B's local +v (height) onto joint +Z, which necessarily carries its
    local +w (thickness) onto joint -Y — mapping Y->Z and Z->Y at once would be a
    reflection. So `B:+w` is the near face and `B:-w` the far one.
    """
    T_B = (T
           @ translate(spec.length_offset_mm, y0, z0)
           @ rot_x(spec.tilt_deg(joint_type))
           @ translate(0.0, spec.t_B / 2.0, spec.H_B / 2.0)
           @ rot_x(90.0))
    # local +v is height, so the welded (bottom) edge is the -v side
    return _part_B(spec, T_B, seam_at_plus_v=False)


def _flat_B(spec: JointSpec, T: np.ndarray, y_centre: float, z_centre: float,
            joint_type: str, pivot: tuple[float, float] | None = None) -> Slab:
    """A plate lying flat (thickness along Z, like part A).

    `pivot` is the (y, z) the angular misalignment hinges about, and it must be the
    contact - two clamped plates hinge about where they touch, not about their own
    centres. Pivoting at the centre lifts the welded edge itself, by half the plate width
    times sin(beta), which is enormous next to the quantities that decide contact: 0.4 deg
    on a 179 mm plate is 0.62 mm against a 0.1 mm root gap, and 2 deg on a 133 mm plate
    is 2.4 mm against a 1.2 mm tolerance. Both cases lost the seam outright rather than
    recording a misaligned one.
    """
    tilt = spec.tilt_deg(joint_type)
    if pivot is None:
        T_B = (T @ translate(spec.length_offset_mm, y_centre, z_centre)
                 @ rot_x(tilt))
    else:
        pivot_y, pivot_z = pivot
        T_B = (T
               @ translate(spec.length_offset_mm, pivot_y, pivot_z)
               @ rot_x(tilt)
               @ translate(0.0, y_centre - pivot_y, z_centre - pivot_z))
    # edge joint welds B's +v edge (flush at y=0); butt welds the -v edge across the gap
    return _part_B(spec, T_B, seam_at_plus_v=(joint_type == "edge"))


def _base_A(spec: JointSpec, T: np.ndarray, y_centre: float) -> Slab:
    """Part A, lying flat with its `+w` face on the plane z = 0."""
    T_A = T @ translate(0.0, y_centre, -spec.t_A / 2.0)
    if spec.outline_A is None:
        return Slab("A", "workpiece", 0, (spec.L_A, spec.W_A, spec.t_A), T_A)
    # every outlined joint (corner, butt, edge) welds A's +v edge (the one at y = 0)
    return Prism("A", "workpiece", 0,
                 _outline_local(spec.outline_A, spec.W_A, seam_at_plus_v=True),
                 spec.t_A, T_A, shape=spec.outline_shape_A)


def _yawed(T: np.ndarray, yaw_deg: float, px: float, py: float) -> np.ndarray:
    """`T` with an in-plane rotation about the vertical line through `(px, py)` (D28).

    The pivot is the CONTACT footprint's centroid, so the support constraint stays a
    rotation of a fixed rectangle rather than a rotation plus a drift.
    """
    if abs(yaw_deg) < 1e-12:
        return T
    return (T @ translate(px, py, 0.0) @ rot_z(yaw_deg) @ translate(-px, -py, 0.0))


def max_supported_yaw_deg(spec: JointSpec, joint_type: str,
                          margin_mm: float = 1.0, step_deg: float = 0.5,
                          min_run_frac: float = 0.5) -> float:
    """Largest |yaw| keeping a supported seam run of at least `min_run_frac` its length.

    Two wrong bounds preceded this one, both caught by the D28 gate they were built to
    serve. Requiring B's full footprint on A returned 0 for every scene with L_B > L_A -
    a configuration the layouts explicitly allow at yaw 0, clipping the seam to the shared
    run. Requiring the *clipped* footprint's corners on A returned 0 whenever B overhangs
    both ends, because a rectangle already touching both walls cannot rotate - yet
    physically a small yaw just changes where the overhang falls. The criterion that
    matches the existing yaw-0 semantics (the `length_offset` clamp, `min_overlap_frac`)
    is about the SEAM, not the footprint: the chord of A's face along the yawed seam
    direction, through the contact centroid, must stay at least `min_run_frac` of
    `min(L_A, L_B)` - for the centreline and for both edges of the contact strip, so a
    wide lap overlap cannot hang half off the side of A.

    Scanned at `step_deg`; the largest angle before the first failure is returned. D27
    holds: the seam length remains pinned by the supported run.
    """
    geo = _yaw_support_geometry(spec, joint_type, margin_mm, min_run_frac)
    if geo is None:
        return 0.0
    ax, ay, cx, cy, offs, need = geo

    def chord(px, py, th_rad):
        d = np.array([np.cos(th_rad), np.sin(th_rad)])
        ts = []
        for k, (lo_, hi_) in enumerate((ax, ay)):
            if abs(d[k]) < 1e-12:
                if not (lo_ <= (px, py)[k] <= hi_):
                    return 0.0
                ts.append((-np.inf, np.inf))
            else:
                a, b = (lo_ - (px, py)[k]) / d[k], (hi_ - (px, py)[k]) / d[k]
                ts.append((min(a, b), max(a, b)))
        t0 = max(ts[0][0], ts[1][0])
        t1 = min(ts[0][1], ts[1][1])
        return max(0.0, t1 - t0)

    best = 0.0
    for th in np.arange(0.0, 90.0 + step_deg, step_deg):
        ok = True
        for sgn in (+1.0, -1.0):
            a = np.radians(sgn * th)
            n = np.array([-np.sin(a), np.cos(a)])      # across-seam direction
            for off in offs:
                if chord(cx + off * n[0], cy + off * n[1], a) < need:
                    ok = False
                    break
            if not ok:
                break
        if not ok:
            break
        best = th
    return float(min(best, 90.0))


def _yaw_support_geometry(spec: JointSpec, joint_type: str, margin_mm: float,
                          min_run_frac: float):
    """`(ax, ay, cx, cy, offs, need)` for the seam-support chord test, or None.

    `offs` are STRIP-FIXED offsets: `pivot + off * n(theta)` with a continuous
    `n(theta) = (-sin, cos)` follows the rotating contact strip, so the same off means
    the same physical line (the leading toe, the centreline) at every yaw - including
    past 90 deg, where a folded angle would silently test the wrong side.
    """
    if joint_type == "T":
        width = spec.t_B
        cy = spec.linear_misalignment_mm + spec.t_B / 2.0
        ax = (-spec.L_A / 2.0 + margin_mm, spec.L_A / 2.0 - margin_mm)
        ay = (-spec.W_A / 2.0 + margin_mm, spec.W_A / 2.0 - margin_mm)
        off_hi = max(0.0, width / 2.0 - margin_mm)
        offs = (-off_hi, 0.0, off_hi)                  # both fillet lines are interior
    elif joint_type == "lap":
        # The contact strip is A intersect B: y in [-min(overlap, W_A), 0]. Its y=0 edge
        # IS A's boundary edge - the A-side toe candidate - and a chord through a
        # boundary point in any tilted direction is half-length, so testing that edge
        # pinned every lap to yaw 0 (measured: 21 of 40 bench seeds). What the bound
        # must protect is the PRIMARY lap fillet, B's leading toe at the strip's other
        # edge, which is interior whenever B does not overspan A; the A-edge toe just
        # shortens under yaw and D4's exact clip records whatever run survives. When B
        # overspans A (overlap >= W_A) both strip edges are A's own boundary, so only
        # the centreline is testable.
        width = min(float(spec.stack_offset_mm or 0.0), spec.W_A)
        cy = -width / 2.0
        ax = (-spec.L_A / 2.0 + margin_mm, spec.L_A / 2.0 - margin_mm)
        ay = (-spec.W_A + margin_mm, -margin_mm)       # A occupies y in [-W_A, 0]
        off_lo = max(0.0, width / 2.0 - margin_mm)
        if float(spec.stack_offset_mm or 0.0) < spec.W_A:
            offs = (-off_lo, 0.0)                      # leading toe + centreline
        else:
            offs = (0.0,)
    else:
        return None
    cx = spec.length_offset_mm
    need = float(min_run_frac) * min(spec.L_A, spec.L_B)
    return ax, ay, cx, cy, offs, need


#: Grid pitch for the full-circle feasible-yaw scan. The support criterion is smooth at
#: plate scale, and every accepted configuration is re-verified downstream by the D4
#: clip and separation gates, so a borderline jitter can only cost a rejected seed,
#: never a wrong scene.
YAW_GRID_DEG = 1.0


def feasible_yaw_deg(spec: JointSpec, joint_type: str,
                     margin_mm: float = 1.0, min_run_frac: float = 0.5) -> np.ndarray:
    """All grid angles in [-180, 180) keeping the supported seam run (full circle, D28).

    The feasible set need not be a contiguous +-interval: a long narrow A supports the
    seam near 0 and near 180 but not sideways, and yaw 180 is a genuinely different
    configuration from 0 (B's body points the other way), which is why the range is the
    circle and not a folded half."""
    geo = _yaw_support_geometry(spec, joint_type, margin_mm, min_run_frac)
    if geo is None:
        return np.zeros(0)
    ax, ay, cx, cy, offs, need = geo

    def chord(px, py, th_rad):
        d = np.array([np.cos(th_rad), np.sin(th_rad)])
        ts = []
        for k, (lo_, hi_) in enumerate((ax, ay)):
            if abs(d[k]) < 1e-12:
                if not (lo_ <= (px, py)[k] <= hi_):
                    return 0.0
                ts.append((-np.inf, np.inf))
            else:
                a, b = (lo_ - (px, py)[k]) / d[k], (hi_ - (px, py)[k]) / d[k]
                ts.append((min(a, b), max(a, b)))
        t0 = max(ts[0][0], ts[1][0])
        t1 = min(ts[0][1], ts[1][1])
        return max(0.0, t1 - t0)

    out = []
    for th in np.arange(-180.0, 180.0, YAW_GRID_DEG):
        a = np.radians(th)
        n = np.array([-np.sin(a), np.cos(a)])
        if all(chord(cx + off * n[0], cy + off * n[1], a) >= need for off in offs):
            out.append(th)
    return np.asarray(out, dtype=float)


def sample_yaw_deg(spec: JointSpec, joint_type: str,
                   rng: np.random.Generator) -> float:
    """Uniform over the feasible set of the FULL circle (two draws: cell + jitter).

    Consumes no draws when the set is empty, matching the pre-360 behaviour where a
    zero bound consumed none - so the per-seed draw sequence stays deterministic and
    the outline draws that follow are untouched on exactly the same seeds."""
    angles = feasible_yaw_deg(spec, joint_type)
    if len(angles) == 0:
        return 0.0
    idx = int(rng.integers(len(angles)))
    jitter = float(rng.uniform(-YAW_GRID_DEG / 2.0, YAW_GRID_DEG / 2.0))
    yaw = float(angles[idx]) + jitter
    return float((yaw + 180.0) % 360.0 - 180.0)


def _layout_T(spec: JointSpec, T: np.ndarray) -> list[Slab]:
    """B stands in the MIDDLE of A. Two fillets, one per side of B."""
    A = _base_A(spec, T, y_centre=0.0)
    Ty = _yawed(T, spec.in_plane_yaw_deg,
                spec.length_offset_mm,
                spec.linear_misalignment_mm + spec.t_B / 2.0)
    B = _standing_B(spec, Ty, y0=spec.linear_misalignment_mm,
                    z0=spec.root_gap_mm, joint_type="T")
    return [A, B]


def _layout_corner(spec: JointSpec, T: np.ndarray) -> list[Slab]:
    """B stands BESIDE A, past its edge — the plates meet at their edges.

    A occupies y <= 0; B stands at y >= g with its bottom level with A's top face. The
    inside fillet and the outside corner are both real welds, so a corner joint yields
    two seams with opposite approach directions.

    `h` displaces B along Z, not along Y. ISO 5817 ref 5071 is the step between the two
    members' surfaces, a quantity independent of the root gap; adding it to `y0` instead
    made it a second, unlabelled gap widener, and the joint faces then sat `g + h` apart.
    Since `contact_tol` is capped by plate thickness that put them out of contact
    entirely, and the corner joint - alone among the five - lost about 10% of its scenes
    to a misalignment that should not have moved the gap at all.
    """
    A = _base_A(spec, T, y_centre=-spec.W_A / 2.0)
    B = _standing_B(spec, T,
                    y0=spec.root_gap_mm,
                    z0=spec.linear_misalignment_mm, joint_type="corner")
    return [A, B]


def _grooved_butt(spec: JointSpec, T: np.ndarray) -> list[Slab]:
    """D35 butt with an edge preparation: PreparedSlab pair (D30: straight only).

    A's prepared edge sits at y = 0 with material at y <= 0, exactly the local frame
    the primitive defines; B is the same primitive rotated 180 about z so its
    preparation faces A across the root gap. Equal thickness (forced by the sampler).
    `h` still offsets B vertically (ISO 5817 no. 5071). Single-bevel prepares A only;
    B stays a square Slab - ref 1.9.1's asymmetric preparation.
    """
    g = dict(spec.groove)
    prep_dict = {"kind": spec.prep, "bevel_deg": g["bevel_deg_per_side"],
                 "root_face_mm": g["root_face_mm"]}
    if spec.prep == "single_U":
        prep_dict["radius_mm"] = g["radius_mm"]
    A = PreparedSlab("A", "workpiece", 0, spec.L_A, spec.W_A, spec.t_A,
                     prep_dict, T)
    if spec.prep == "single_bevel":
        B = Slab("B", "workpiece", 1, (spec.L_B, spec.H_B, spec.t_B),
                 T @ translate(spec.length_offset_mm,
                               spec.root_gap_mm + spec.H_B / 2.0,
                               spec.linear_misalignment_mm - spec.t_B / 2.0))
    else:
        B = PreparedSlab("B", "workpiece", 1, spec.L_B, spec.H_B, spec.t_B,
                         dict(prep_dict),
                         T @ translate(spec.length_offset_mm, spec.root_gap_mm,
                                       spec.linear_misalignment_mm)
                         @ rot_z(180.0))
    return [A, B]


def _layout_butt(spec: JointSpec, T: np.ndarray) -> list[Slab]:
    """Coplanar plates, edge to edge across the gap. `h` offsets B vertically.

    Plates of unequal thickness are set FLUSH ON ONE FACE, here the underside, and `h` is
    measured from there. Centring both on the mid-thickness plane instead - which is what
    equal-thickness geometry silently implied - steps *both* faces by half the thickness
    difference, so a 6.3 mm plate butted to a 3.1 mm one had no coplanar face pair at all
    and no centreline was ever enumerated. On dissimilar thickness this joint therefore
    has one centreline, on the flush side, which is the physical answer: the other side is
    a step, and ISO 9692-1 treats it as a transition rather than a second weld.
    """
    if spec.prep != "square":
        return _grooved_butt(spec, T)
    A = _base_A(spec, T, y_centre=-spec.W_A / 2.0)
    z_centre = -spec.t_A + spec.t_B / 2.0 + spec.linear_misalignment_mm
    B = _flat_B(spec, T,
                y_centre=spec.root_gap_mm + spec.H_B / 2.0,
                z_centre=z_centre,
                joint_type="butt",
                # Hinge at the joint line, through B's own mid-thickness: the near edge
                # stays put across the gap and the far edge rises, which is what angular
                # misalignment means on a butt joint.
                pivot=(spec.root_gap_mm, z_centre))
    return [A, B]


def _layout_lap(spec: JointSpec, T: np.ndarray) -> list[Slab]:
    """Parallel plates overlapping face to face by `stack_offset_mm`.

    Included angle 0 (PARAMETERS.md §2.7). The mid-lap interface is a *facing* pair with
    no intersection line and a degenerate bisector — the hard negative a nearest-point
    rule would wrongly return.
    """
    A = _base_A(spec, T, y_centre=-spec.W_A / 2.0)
    overlap = spec.stack_offset_mm
    Ty = _yawed(T, spec.in_plane_yaw_deg, spec.length_offset_mm, -overlap / 2.0)
    B = _flat_B(spec, Ty,
                y_centre=spec.H_B / 2.0 - overlap,
                z_centre=spec.root_gap_mm + spec.t_B / 2.0,
                joint_type="lap",
                pivot=(-overlap / 2.0, spec.root_gap_mm))   # hinge in the overlap
    return [A, B]


def _layout_edge(spec: JointSpec, T: np.ndarray) -> list[Slab]:
    """Lap at zero offset: the free edges are flush (PARAMETERS.md §2.7).

    ISO 9692-1 Table 1 ref 1.1 ("raised edges") applies at t <= 2 mm, so edge scenes are
    a thin-sheet preparation by the standard's own scope, not by our choice.
    """
    A = _base_A(spec, T, y_centre=-spec.W_A / 2.0)
    # B keeps its OWN width and is aligned flush at the welded edge (y = 0). Forcing
    # B's width to A's made every edge joint a pair of twins with both long edges flush,
    # so the seam count was pinned at 2. Real edge joints join parts of different widths,
    # and then only the aligned edge is a weld: the far side becomes a lap toe, which D22
    # classifies and rejects as `wrong_class_for_joint`. So the count is 1 when the widths
    # differ and 2 when they match - and the classification, not the layout, is what keeps
    # it honest.
    B = _flat_B(spec, T,
                y_centre=-spec.H_B / 2.0,
                z_centre=spec.root_gap_mm + spec.t_B / 2.0,
                joint_type="edge",
                pivot=(0.0, spec.root_gap_mm))              # hinge at the WELDED edge
    return [A, B]


_LAYOUTS = {
    "T": _layout_T,
    "corner": _layout_corner,
    "butt": _layout_butt,
    "lap": _layout_lap,
    "edge": _layout_edge,
}
