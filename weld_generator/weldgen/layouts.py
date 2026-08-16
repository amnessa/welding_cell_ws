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

from .geom import Slab, rot_x, translate
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
                @ translate(0.0, y0, z0)
                @ rot_x(spec.tilt_deg(joint_type))
                @ translate(0.0, spec.t_B / 2.0, spec.H_B / 2.0)
                @ rot_x(90.0))


def _flat_B(spec: JointSpec, T: np.ndarray, y_centre: float, z_centre: float,
            joint_type: str) -> Slab:
    """A plate lying flat (thickness along Z, like part A)."""
    return Slab("B", "workpiece", 1, (spec.L_B, spec.H_B, spec.t_B),
                T @ translate(0.0, y_centre, z_centre) @ rot_x(spec.tilt_deg(joint_type)))


def _base_A(spec: JointSpec, T: np.ndarray, y_centre: float) -> Slab:
    """Part A, lying flat with its `+w` face on the plane z = 0."""
    return Slab("A", "workpiece", 0, (spec.L_A, spec.W_A, spec.t_A),
                T @ translate(0.0, y_centre, -spec.t_A / 2.0))


def _layout_T(spec: JointSpec, T: np.ndarray) -> list[Slab]:
    """B stands in the MIDDLE of A. Two fillets, one per side of B."""
    A = _base_A(spec, T, y_centre=0.0)
    B = _standing_B(spec, T, y0=spec.linear_misalignment_mm,
                    z0=spec.root_gap_mm, joint_type="T")
    return [A, B]


def _layout_corner(spec: JointSpec, T: np.ndarray) -> list[Slab]:
    """B stands BESIDE A, past its edge — the plates meet at their edges.

    A occupies y <= 0; B stands at y >= g with its bottom level with A's top face. The
    inside fillet and the outside corner are both real welds, so a corner joint yields
    two seams with opposite approach directions.
    """
    A = _base_A(spec, T, y_centre=-spec.W_A / 2.0)
    B = _standing_B(spec, T,
                    y0=spec.root_gap_mm + spec.linear_misalignment_mm,
                    z0=0.0, joint_type="corner")
    return [A, B]


def _layout_butt(spec: JointSpec, T: np.ndarray) -> list[Slab]:
    """Coplanar plates, edge to edge across the gap. `h` offsets B vertically."""
    A = _base_A(spec, T, y_centre=-spec.W_A / 2.0)
    B = _flat_B(spec, T,
                y_centre=spec.root_gap_mm + spec.H_B / 2.0,
                z_centre=-spec.t_B / 2.0 + spec.linear_misalignment_mm,
                joint_type="butt")
    return [A, B]


def _layout_lap(spec: JointSpec, T: np.ndarray) -> list[Slab]:
    """Parallel plates overlapping face to face by `stack_offset_mm`.

    Included angle 0 (PARAMETERS.md §2.7). The mid-lap interface is a *facing* pair with
    no intersection line and a degenerate bisector — the hard negative a nearest-point
    rule would wrongly return.
    """
    A = _base_A(spec, T, y_centre=-spec.W_A / 2.0)
    overlap = spec.stack_offset_mm
    B = _flat_B(spec, T,
                y_centre=spec.H_B / 2.0 - overlap,
                z_centre=spec.root_gap_mm + spec.t_B / 2.0,
                joint_type="lap")
    return [A, B]


def _layout_edge(spec: JointSpec, T: np.ndarray) -> list[Slab]:
    """Lap at zero offset: the free edges are flush (PARAMETERS.md §2.7).

    ISO 9692-1 Table 1 ref 1.1 ("raised edges") applies at t <= 2 mm, so edge scenes are
    a thin-sheet preparation by the standard's own scope, not by our choice.
    """
    A = _base_A(spec, T, y_centre=-spec.W_A / 2.0)
    B = _flat_B(spec, T,
                y_centre=-spec.H_B / 2.0,
                z_centre=spec.root_gap_mm + spec.t_B / 2.0,
                joint_type="edge")
    return [A, B]


_LAYOUTS = {
    "T": _layout_T,
    "corner": _layout_corner,
    "butt": _layout_butt,
    "lap": _layout_lap,
    "edge": _layout_edge,
}
