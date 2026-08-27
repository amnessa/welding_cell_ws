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

from .geom import Slab, rot_x, rot_z, translate
from .joints import JointSpec

JOINT_TYPES = ("T", "corner", "butt", "lap", "edge")


def build(spec: JointSpec, joint_type: str, T_world_joint: np.ndarray) -> list[Slab]:
    """Place parts A and B for `joint_type`."""
    try:
        fn = _LAYOUTS[joint_type]
    except KeyError:
        raise ValueError(f"unknown joint type {joint_type!r}") from None
    return fn(spec, T_world_joint)


def _standing_B(spec: JointSpec, T: np.ndarray, y0: float, z0: float,
                joint_type: str) -> Slab:
    """A plate standing on edge, its near face at y0 and its bottom at z0.

    rot_x(90) carries B's local +v (height) onto joint +Z, which necessarily carries its
    local +w (thickness) onto joint -Y — mapping Y->Z and Z->Y at once would be a
    reflection. So `B:+w` is the near face and `B:-w` the far one.
    """
    return Slab("B", "workpiece", 1, (spec.L_B, spec.H_B, spec.t_B),
                T
                @ translate(spec.length_offset_mm, y0, z0)
                @ rot_x(spec.tilt_deg(joint_type))
                @ translate(0.0, spec.t_B / 2.0, spec.H_B / 2.0)
                @ rot_x(90.0))


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
        return Slab("B", "workpiece", 1, (spec.L_B, spec.H_B, spec.t_B),
                    T @ translate(spec.length_offset_mm, y_centre, z_centre)
                      @ rot_x(tilt))
    pivot_y, pivot_z = pivot
    return Slab("B", "workpiece", 1, (spec.L_B, spec.H_B, spec.t_B),
                T
                @ translate(spec.length_offset_mm, pivot_y, pivot_z)
                @ rot_x(tilt)
                @ translate(0.0, y_centre - pivot_y, z_centre - pivot_z))


def _base_A(spec: JointSpec, T: np.ndarray, y_centre: float) -> Slab:
    """Part A, lying flat with its `+w` face on the plane z = 0."""
    return Slab("A", "workpiece", 0, (spec.L_A, spec.W_A, spec.t_A),
                T @ translate(0.0, y_centre, -spec.t_A / 2.0))


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
    if joint_type == "T":
        width = spec.t_B
        cy = spec.linear_misalignment_mm + spec.t_B / 2.0
        ax = (-spec.L_A / 2.0 + margin_mm, spec.L_A / 2.0 - margin_mm)
        ay = (-spec.W_A / 2.0 + margin_mm, spec.W_A / 2.0 - margin_mm)
    elif joint_type == "lap":
        width = float(spec.stack_offset_mm or 0.0)
        cy = -width / 2.0
        ax = (-spec.L_A / 2.0 + margin_mm, spec.L_A / 2.0 - margin_mm)
        ay = (-spec.W_A + margin_mm, -margin_mm)       # A occupies y in [-W_A, 0]
    else:
        return 0.0
    cx = spec.length_offset_mm
    need = float(min_run_frac) * min(spec.L_A, spec.L_B)

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
            for off in (-width / 2.0, 0.0, width / 2.0):
                if chord(cx + off * n[0], cy + off * n[1], a) < need:
                    ok = False
                    break
            if not ok:
                break
        if not ok:
            break
        best = th
    return float(min(best, 90.0))


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
