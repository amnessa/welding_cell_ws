"""Joint construction — Phase 1: the T-joint, straight seam, square preparation.

Construction order follows D1/D3: sample the joint specification, derive the placement
transforms from it, then compute every seam **in closed form from those transforms**.
Nothing here inspects a point cloud.

Canonical joint frame (before the assembly is posed in the world):

    +X  along the seam
    +Z  up, out of the base plate's top face
    +Y  completes the right-handed set

    Part A - base plate, horizontal. Its `+w` face is the plane z = 0, so A occupies
             z in [-t_A, 0].
    Part B - standing plate. Its thickness runs along Y, its height along Z. Before
             defects it occupies y in [0, t_B], z in [g, g + H_B].

A T-joint has **two** fillets (D5), one on each side of B:

    seam 0 = A:+w  x  B:-w
    seam 1 = A:+w  x  B:+w

Both are weldable: each bisector leaves the seam without re-entering material.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .geom import (
    Slab,
    approach_dir,
    dihedral_deg,
    intersect_planes,
    rot_x,
    translate,
)


@dataclass
class JointSpec:
    """The sampled specification of one T-joint, in millimetres and degrees."""

    L_A: float          # base plate length, along the seam
    W_A: float          # base plate width
    t_A: float          # base plate thickness
    L_B: float          # standing plate length, along the seam
    H_B: float          # standing plate height
    t_B: float          # standing plate thickness
    root_gap_mm: float          # g   - ISO 9692-1 preparation dimension
    linear_misalignment_mm: float   # h - ISO 5817 no. 5071
    angular_misalignment_deg: float  # beta - ISO 5817 no. 508 (Annex B)
    included_angle_deg: float        # alpha - ISO 9692-1 design nominal (D18)
    #: lap: 0 < s < L overlap; edge: 0 (flush). None for T / corner / butt.
    #: PARAMETERS.md §2.7 - lap and edge are one topology at different offsets.
    stack_offset_mm: float | None = None
    #: Longitudinal (along-seam) offset of B relative to A. With L_A != L_B this makes
    #: the seam clip to the SHARED run rather than always spanning the full plate.
    length_offset_mm: float = 0.0
    #: D28 (Phase 6a): in-plane rotation of B about the contact-face normal, for the
    #: joints that meet on a FACE (T, lap). NOT dihedral - dihedral is the fold about the
    #: seam axis (D18); yaw rotates the seam itself within the lower part's face, so the
    #: seam stops being systematically parallel to the base plate's boundary edges. 0 for
    #: corner / butt / edge, whose parts share an edge and get their D28 diversity from
    #: outlines instead.
    in_plane_yaw_deg: float = 0.0

    #: The included angle each joint type takes as its nominal, i.e. the angle at which
    #: part B is undeflected. NOT all 90: a butt joint is coplanar (180) and a lap or
    #: edge joint is parallel (0) - PARAMETERS.md §2.7.
    NOMINAL_INCLUDED_DEG = {"T": 90.0, "corner": 90.0, "butt": 180.0,
                            "lap": 0.0, "edge": 0.0}

    def tilt_deg(self, joint_type: str = "T") -> float:
        """Realised rotation of B about the seam axis: design departure plus defect.

        `included_angle_deg` is the design nominal and `angular_misalignment_deg` is the
        deviation from it (D18) — a 70 degree T-joint is not a 90 degree T-joint with a
        20 degree defect.

        The departure is measured from the joint type's OWN nominal. Using 90 for every
        type silently stands a butt or lap plate on edge, because their nominals are 180
        and 0.
        """
        try:
            nominal = self.NOMINAL_INCLUDED_DEG[joint_type]
        except KeyError:
            raise ValueError(f"unknown joint type {joint_type!r}") from None
        return (self.included_angle_deg - nominal) + self.angular_misalignment_deg

    @property
    def throat_thickness_mm(self) -> float:
        """aA = 0.7 * min(t) - the mitre-fillet convention, PARAMETERS.md §2.4."""
        return 0.7 * min(self.t_A, self.t_B)


@dataclass
class Seam:
    """One constructed seam, exact."""

    face_pair: tuple[str, str]
    weldable: bool
    reject_reason: str | None
    p0: np.ndarray          # nominal curve start, world
    p1: np.ndarray          # nominal curve end, world
    root_p0: np.ndarray     # D19 `root` curve
    root_p1: np.ndarray
    gapmid_p0: np.ndarray   # D19 `gap_mid` curve
    gapmid_p1: np.ndarray
    n_a: np.ndarray
    n_b: np.ndarray

    @property
    def length_mm(self) -> float:
        return float(np.linalg.norm(self.p1 - self.p0))

    @property
    def dihedral_deg(self) -> float:
        return dihedral_deg(self.n_a, self.n_b)

    @property
    def approach(self) -> np.ndarray:
        return approach_dir(self.n_a, self.n_b)


def build_t_joint(spec: JointSpec, T_world_joint: np.ndarray) -> tuple[list[Slab], list[Seam]]:
    """Place the two plates and derive both fillets analytically.

    `T_world_joint` poses the whole assembly in the world. Nothing is pinned to the world
    origin (SCHEMA.md §1.1) — a seam that always starts at the origin would be trivially
    predictable.
    """
    # --- placement -----------------------------------------------------------------
    # A: centroid at (0, 0, -t_A/2) so its +w face is exactly the plane z = 0.
    T_A = T_world_joint @ translate(0.0, 0.0, -spec.t_A / 2.0)

    # B: its local axes are (u = length, v = height, w = thickness) - the `w`-is-thickness
    # invariant. rot_x(90) carries local +v (height) onto joint +Z, which necessarily
    # carries local +w onto joint -Y: mapping Y->Z and Z->Y at once would be a reflection,
    # not a rotation. So `B:+w` is the face at y = 0 and `B:-w` the face at y = t_B.
    #
    # Then: shift so B occupies y in [0, t_B], z in [0, H_B]; tilt about the seam axis;
    # finally offset by the misalignment h and lift by the root gap g.
    T_B = (
        T_world_joint
        @ translate(0.0, spec.linear_misalignment_mm, spec.root_gap_mm)
        @ rot_x(spec.tilt_deg("T"))
        @ translate(0.0, spec.t_B / 2.0, spec.H_B / 2.0)
        @ rot_x(90.0)
    )

    A = Slab("A", "workpiece", 0, (spec.L_A, spec.W_A, spec.t_A), T_A)
    B = Slab("B", "workpiece", 1, (spec.L_B, spec.H_B, spec.t_B), T_B)

    # --- seams, in closed form from the transforms (D1) -----------------------------
    seams = [
        _fillet(A, B, "+w", "-w"),
        _fillet(A, B, "+w", "+w"),
    ]
    return [A, B], seams


def _fillet(A: Slab, B: Slab, face_a: str, face_b: str) -> Seam:
    """One fillet: the nominal, root and gap_mid curves for a face pair (D19)."""
    pa, pb = A.face_plane(face_a), B.face_plane(face_b)
    hit = intersect_planes(pa, pb)
    if hit is None:
        raise ValueError(f"faces {face_a}/{face_b} are parallel; not a fillet")
    point, direction = hit

    # Clip along the seam parameter to where BOTH faces have support. The clip is a
    # projection, not a containment test: at g > 0 the faces do not touch the nominal
    # line at all, which is exactly why `nominal` lives on the extended planes.
    a_lo, a_hi = A.face_extent_along(face_a, direction)
    b_lo, b_hi = B.face_extent_along(face_b, direction)
    lo, hi = max(a_lo, b_lo), min(a_hi, b_hi)

    # Re-anchor: `point` is the line's closest approach to the origin, so its own
    # projection is the parameter zero.
    s0 = float(point @ direction)
    p0 = point + (lo - s0) * direction
    p1 = point + (hi - s0) * direction

    # `root` - the edge of the terminating face, i.e. B's bottom edge on this side.
    # It is the intersection of B's joint face with B's own bottom face (-v in B-local,
    # since B's height runs along its local v axis).
    root_hit = intersect_planes(B.face_plane(face_b), B.face_plane("-v"))
    if root_hit is None:  # pragma: no cover - orthogonal by construction
        raise ValueError("degenerate root edge")
    rpoint, rdir = root_hit
    if rdir @ direction < 0:
        rdir = -rdir
    rs0 = float(rpoint @ rdir)
    root_p0 = rpoint + (lo - rs0) * rdir
    root_p1 = rpoint + (hi - rs0) * rdir

    # `gap_mid` - midway between the root edge and its projection onto plane A.
    gapmid_p0 = _midpoint_to_plane(root_p0, pa)
    gapmid_p1 = _midpoint_to_plane(root_p1, pa)

    return Seam(
        face_pair=(f"A:{face_a}", f"B:{face_b}"),
        weldable=True,
        reject_reason=None,
        p0=p0, p1=p1,
        root_p0=root_p0, root_p1=root_p1,
        gapmid_p0=gapmid_p0, gapmid_p1=gapmid_p1,
        n_a=pa.n, n_b=pb.n,
    )


def _midpoint_to_plane(p: np.ndarray, plane) -> np.ndarray:
    """Midpoint between `p` and its orthogonal projection onto `plane`."""
    return p - 0.5 * float(plane.signed_distance(p)) * plane.n
