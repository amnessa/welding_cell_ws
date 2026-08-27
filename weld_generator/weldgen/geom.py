"""Primitives, face registry and exact plane algebra — SCHEMA.md §2.2.

Phase 1 needs exactly one primitive: the `slab`. Its local frame obeys the invariant the
whole schema rests on:

    w is ALWAYS the thickness axis (the ISO 5817 `t`).

So for every slab in the dataset the broad faces are ±w and the edge faces are ±u, ±v,
and the Phase 7 edge-margin rule needs no per-joint special case.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import trimesh

#: Slab face names, in registry order. Index is the local face id.
SLAB_FACES = ("+u", "-u", "+v", "-v", "+w", "-w")

#: Outward unit normal of each slab face, in the part's LOCAL frame.
SLAB_LOCAL_NORMALS = {
    "+u": np.array([1.0, 0.0, 0.0]),
    "-u": np.array([-1.0, 0.0, 0.0]),
    "+v": np.array([0.0, 1.0, 0.0]),
    "-v": np.array([0.0, -1.0, 0.0]),
    "+w": np.array([0.0, 0.0, 1.0]),
    "-w": np.array([0.0, 0.0, -1.0]),
}


def rot_x(theta_deg: float) -> np.ndarray:
    """4x4 rotation about +X by `theta_deg` degrees."""
    t = np.deg2rad(float(theta_deg))
    c, s = np.cos(t), np.sin(t)
    T = np.eye(4)
    T[1, 1], T[1, 2] = c, -s
    T[2, 1], T[2, 2] = s, c
    return T


def rot_y(theta_deg: float) -> np.ndarray:
    """4x4 rotation about +Y by `theta_deg` degrees."""
    t = np.deg2rad(float(theta_deg))
    c, s_ = np.cos(t), np.sin(t)
    T = np.eye(4)
    T[0, 0], T[0, 2] = c, s_
    T[2, 0], T[2, 2] = -s_, c
    return T


def rot_z(theta_deg: float) -> np.ndarray:
    """4x4 rotation about +Z by `theta_deg` degrees."""
    t = np.deg2rad(float(theta_deg))
    c, s = np.cos(t), np.sin(t)
    T = np.eye(4)
    T[0, 0], T[0, 1] = c, -s
    T[1, 0], T[1, 1] = s, c
    return T


def translate(x: float, y: float, z: float) -> np.ndarray:
    T = np.eye(4)
    T[:3, 3] = (x, y, z)
    return T


@dataclass(frozen=True)
class Plane:
    """Oriented plane ``n·p + d = 0`` with `n` the OUTWARD unit normal."""

    n: np.ndarray
    d: float

    def signed_distance(self, p: np.ndarray) -> np.ndarray:
        return np.asarray(p) @ self.n + self.d


@dataclass
class Slab:
    """Axis-aligned box in its local frame, placed by `T_world_part`.

    Local extents are centred on the centroid: u in [-L/2, L/2], v in [-W/2, W/2],
    w in [-t/2, t/2]. All placement lives in `T_world_part`, so the local frame is
    always the canonical one and the face registry never varies.
    """

    id: str
    role: str
    object_id: int
    dims_mm: tuple[float, float, float]  # (L along u, W along v, t along w)
    T_world_part: np.ndarray

    @property
    def thickness_mm(self) -> float:
        return float(self.dims_mm[2])

    @property
    def part_geometry_id(self) -> str:
        L, W, t = self.dims_mm
        return f"slab_{L}x{W}x{t}"

    def mesh(self) -> trimesh.Trimesh:
        """Watertight, winding-consistent box at its world pose (D21)."""
        m = trimesh.creation.box(extents=np.asarray(self.dims_mm, dtype=float))
        m.apply_transform(self.T_world_part)
        return m

    def contains(self, points: np.ndarray, tol: float = 0.0) -> np.ndarray:
        """Exact point-in-box test, done analytically in the slab's own frame.

        Deliberately not `trimesh.Trimesh.contains`, which needs `rtree` for its bounds
        tree. D9 requires the tier-1 core to install on a clean machine with no GPU and
        no optional native deps, and a box does not need a ray-cast to answer this.
        """
        half = np.asarray(self.dims_mm, dtype=float) / 2.0 + float(tol)
        T = self.T_world_part
        local = (np.atleast_2d(np.asarray(points, dtype=float)) - T[:3, 3]) @ T[:3, :3]
        return np.all(np.abs(local) <= half, axis=1)

    def face_normal(self, name: str) -> np.ndarray:
        """Outward unit normal of a face, in WORLD coordinates."""
        R = self.T_world_part[:3, :3]
        return R @ SLAB_LOCAL_NORMALS[name]

    def face_center(self, name: str) -> np.ndarray:
        """Centre of a face, in WORLD coordinates."""
        half = np.asarray(self.dims_mm, dtype=float) / 2.0
        local = SLAB_LOCAL_NORMALS[name] * half
        return (self.T_world_part @ np.append(local, 1.0))[:3]

    def face_plane(self, name: str) -> Plane:
        """Exact supporting plane of a face, in WORLD coordinates."""
        n = self.face_normal(name)
        return Plane(n=n, d=float(-n @ self.face_center(name)))

    def face_area(self, name: str) -> float:
        L, W, t = self.dims_mm
        return {"+u": W * t, "-u": W * t,
                "+v": L * t, "-v": L * t,
                "+w": L * W, "-w": L * W}[name]

    @property
    def surface_area_mm2(self) -> float:
        """Total area of all six faces — the denominator for a realised point density."""
        L, W, t = self.dims_mm
        return 2.0 * (L * W + W * t + L * t)

    def face_clip_line(self, name: str, point: np.ndarray, direction: np.ndarray,
                       slack_mm: float = 0.0) -> tuple[float, float] | None:
        """Parameter interval of the line `point + s*direction` over THIS face's rectangle.

        The exact 2-D clip: the line is dropped into the slab's local frame, the face's
        own normal coordinate is ignored (at `g > 0` the seam line is offset from the face
        along its normal by construction - D19 defines `nominal` on the *extended*
        planes), and the interval is where the two IN-FACE coordinates lie within the
        face's half-extents. `None` when the line never crosses that footprint.

        `slack_mm` loosens ONLY the containment test of a coordinate the line holds
        constant - never the interval arithmetic. Both halves of that sentence were
        earned the hard way: with zero slack, a lap toe dies the moment the gap opens
        (its line lies exactly `g` outside the edge face that generates it, always in a
        constant coordinate - the offset is along the OTHER plane's normal, which this
        face cannot vary along); with slack applied to the interval axes as well, every
        seam overhangs its plate by the slack, the separation gate samples the overhang,
        and measures a "separation" exactly equal to the slack it was granted. The caller
        passes the same `contact_tol_mm` that decides adjacency; the two questions are
        the same question.

        This replaces `face_extent_along` at the seam-clipping site because that method
        is a 1-D shadow: the projection of a rectangle's corners onto the seam direction.
        At in-plane yaw 0 the shadow equals the true overlap and the difference is
        invisible - which is exactly how an axis-aligned shortcut survives in a codebase
        (D28). At yaw != 0 the shadow overhangs the rectangle's actual chord, the clipped
        segment extends past real support, and the separation gate then rejects the whole
        pair: a 13-degree yawed T-joint lost both fillets to a 4 mm "separation" measured
        at sample points that were off both plates.
        """
        half = np.asarray(self.dims_mm, dtype=float) / 2.0
        R = self.T_world_part[:3, :3]
        c = self.T_world_part[:3, 3]
        p_loc = (np.asarray(point, dtype=float) - c) @ R
        d_loc = np.asarray(direction, dtype=float) @ R
        axis = "uvw".index(name[1])
        lo, hi = -np.inf, np.inf
        for k in range(3):
            if k == axis:
                continue                                # the face's own normal axis
            if abs(d_loc[k]) < 1e-12:
                if abs(p_loc[k]) > half[k] + float(slack_mm) + 1e-9:
                    return None
                continue
            a = (-half[k] - p_loc[k]) / d_loc[k]
            b = (half[k] - p_loc[k]) / d_loc[k]
            lo = max(lo, min(a, b))
            hi = min(hi, max(a, b))
        if hi - lo <= 1e-9 or not np.isfinite(lo) or not np.isfinite(hi):
            return None
        return float(lo), float(hi)

    def face_extent_along(self, name: str, direction: np.ndarray) -> tuple[float, float]:
        """Projected span of a face onto a world `direction`, as (min, max).

        Used to clip a seam to where both faces have support (SCHEMA.md §1.3): the clip
        runs along the seam parameter only. It deliberately does NOT require the faces to
        literally touch the line, because at `g > 0` they do not — that is the whole
        reason `nominal` is defined on the *extended* planes.
        """
        half = np.asarray(self.dims_mm, dtype=float) / 2.0
        # The four corners of this face in local coordinates.
        axis = "uvw".index(name[1])
        sign = 1.0 if name[0] == "+" else -1.0
        corners = []
        others = [i for i in range(3) if i != axis]
        for s0 in (-1.0, 1.0):
            for s1 in (-1.0, 1.0):
                p = np.zeros(3)
                p[axis] = sign * half[axis]
                p[others[0]] = s0 * half[others[0]]
                p[others[1]] = s1 * half[others[1]]
                corners.append(p)
        local = np.asarray(corners).T                      # (3, 4)
        homo = np.vstack([local, np.ones(local.shape[1])])  # (4, 4)
        world = (self.T_world_part @ homo)[:3].T            # (4, 3)
        proj = world @ np.asarray(direction, dtype=float)
        return float(proj.min()), float(proj.max())


def intersect_planes(pa: Plane, pb: Plane, tol: float = 1e-9):
    """Exact line of intersection of two planes.

    Returns ``(point, direction)`` with `direction` a unit vector, or ``None`` if the
    planes are parallel within `tol`.
    """
    d = np.cross(pa.n, pb.n)
    norm = np.linalg.norm(d)
    if norm < tol:
        return None
    d = d / norm
    # Solve for the point on the line closest to the origin.
    A = np.vstack([pa.n, pb.n, d])
    b = np.array([-pa.d, -pb.d, 0.0])
    p = np.linalg.solve(A, b)
    return p, d


def dihedral_deg(na: np.ndarray, nb: np.ndarray) -> float:
    """Angle between two faces, measured through the material-free side.

    For two outward normals the seam's dihedral is ``180 - angle(na, nb)``: a 90 degree
    fillet has normals 90 degrees apart, and two coplanar faces (a flush edge joint) have
    normals 0 degrees apart and a 180 degree dihedral.
    """
    c = float(np.clip(np.asarray(na) @ np.asarray(nb), -1.0, 1.0))
    return float(180.0 - np.degrees(np.arccos(c)))


def approach_dir(na: np.ndarray, nb: np.ndarray) -> np.ndarray:
    """Unit dihedral bisector, pointing FROM the seam TOWARD the torch (SCHEMA.md §1.2)."""
    v = np.asarray(na, dtype=float) + np.asarray(nb, dtype=float)
    n = np.linalg.norm(v)
    if n < 1e-12:
        raise ValueError("degenerate face pair: normals are antiparallel")
    return v / n
