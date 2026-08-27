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

    def face_names(self) -> tuple[str, ...]:
        return SLAB_FACES

    def closest_on_face(self, name: str, pts: np.ndarray) -> np.ndarray:
        """Closest point on the finite rectangular face patch (not its infinite plane)."""
        half = np.asarray(self.dims_mm, dtype=float) / 2.0
        axis = "uvw".index(name[1])
        sign = 1.0 if name[0] == "+" else -1.0
        T = self.T_world_part
        R, t = T[:3, :3], T[:3, 3]
        local = (np.atleast_2d(np.asarray(pts, dtype=float)) - t) @ R
        target = local.copy()
        target[:, axis] = sign * half[axis]
        for k in range(3):
            if k != axis:
                target[:, k] = np.clip(target[:, k], -half[k], half[k])
        return target @ R.T + t

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
        in_face = [k for k in range(3) if k != axis]

        def axis_interval(k):
            if abs(d_loc[k]) < 1e-12:
                if abs(p_loc[k]) > half[k] + float(slack_mm) + 1e-9:
                    return None                         # parallel and outside the slack
                return (-np.inf, np.inf)
            a = (-half[k] - p_loc[k]) / d_loc[k]
            b = (half[k] - p_loc[k]) / d_loc[k]
            return (min(a, b), max(a, b))

        ivs = {k: axis_interval(k) for k in in_face}
        if any(v is None for v in ivs.values()):
            return None
        lo = max(v[0] for v in ivs.values())
        hi = min(v[1] for v in ivs.values())
        if hi - lo > 1e-9 and np.isfinite(lo) and np.isfinite(hi):
            return float(lo), float(hi)

        # NEAR-parallel waiver. The parallel branch above covers a line that holds a
        # coordinate exactly constant; with D28 yaw AND angular misalignment both
        # nonzero, a toe line drifts out of the face plane at ~sin(beta) per mm - not
        # constant, so the interval branch ran instead and quietly deleted the A-side
        # toe of every yawed misaligned lap. The waiver: an axis whose violation stays
        # within the slack OVER THE RUN THE OTHER AXES ALLOW imposes no clip. The run is
        # never EXTENDED (that was the measured slack-in-interval-axes pathology: the
        # overhang grows as slack/|d| and the separation gate measures it); a constraint
        # is only dropped where its breach is bounded by the same tolerance that decides
        # adjacency.
        if float(slack_mm) <= 0.0:
            return None
        for k in in_face:
            others = [ivs[j] for j in in_face if j != k]
            olo = max([v[0] for v in others], default=-np.inf)
            ohi = min([v[1] for v in others], default=np.inf)
            if ohi - olo <= 1e-9 or not np.isfinite(olo) or not np.isfinite(ohi):
                continue
            c0 = p_loc[k] + olo * d_loc[k]
            c1 = p_loc[k] + ohi * d_loc[k]
            if max(abs(c0), abs(c1)) <= half[k] + float(slack_mm) + 1e-9:
                return float(olo), float(ohi)
        return None

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


@dataclass
class Prism:
    """Convex polygon extruded along `w` — the Phase 6a outline primitive (D28).

    Local frame: the outline lives in the u-v plane as CCW vertices, `w` spans
    [-t/2, t/2] — so **w is still the thickness axis** and the schema invariant survives.
    The outline is NOT centred: layouts build it with the seam-bearing edge on v = 0
    spanning u in [-L/2, L/2], because for the edge-sharing joints (corner, butt, edge)
    the seam edge is the one boundary D28 allows to be constrained, and placing it at a
    known local line keeps the placement transforms as simple as the slab ones.

    Face registry: `+w` / `-w` are the polygon caps (the BROAD faces — same names as the
    slab's, so `BROAD_FACES`-based classification is untouched), and `s0..s{k-1}` are the
    side rectangles, `sk` spanning outline vertex k to k+1. Everything downstream asks the
    part for its faces (`face_names`) instead of assuming six.
    """

    id: str
    role: str
    object_id: int
    outline_uv: np.ndarray                # (k, 2) CCW, seam edge = s0 by construction
    thickness: float
    T_world_part: np.ndarray
    shape: str = "polygon"

    def __post_init__(self):
        o = np.asarray(self.outline_uv, dtype=float)
        if len(o) < 3:
            raise ValueError("a prism outline needs at least 3 vertices")
        # enforce CCW so every outward normal below is outward by construction
        area2 = float(np.sum(o[:, 0] * np.roll(o[:, 1], -1)
                             - np.roll(o[:, 0], -1) * o[:, 1]))
        if area2 < 0:
            o = o[::-1].copy()
        self.outline_uv = o

    # --- registry ---------------------------------------------------------------------
    def face_names(self) -> tuple[str, ...]:
        return ("+w", "-w") + tuple(f"s{k}" for k in range(len(self.outline_uv)))

    @property
    def thickness_mm(self) -> float:
        return float(self.thickness)

    @property
    def dims_mm(self) -> tuple[float, float, float]:
        """Bounding dims (L, W, t) — kept so pose- and size-level consumers of the slab
        interface keep working; the exact footprint is `outline_uv`."""
        o = self.outline_uv
        return (float(np.ptp(o[:, 0])), float(np.ptp(o[:, 1])), float(self.thickness))

    @property
    def part_geometry_id(self) -> str:
        # <primitive>_<dims> like the slab's, so the SCHEMA.md 5.4 pattern holds; the
        # vertex count rides in the dims tail because the prefix must stay [a-z_]+.
        L, W, t = self.dims_mm
        return f"prism_{self.shape}_{len(self.outline_uv)}x{L}x{W}x{t}"

    # --- per-face geometry ------------------------------------------------------------
    def _side(self, k: int):
        o = self.outline_uv
        a, b = o[k], o[(k + 1) % len(o)]
        e = b - a
        L = float(np.linalg.norm(e))
        e = e / max(L, 1e-12)
        n2 = np.array([e[1], -e[0]])                   # outward for a CCW outline
        return a, b, e, n2, L

    def face_vertices(self, name: str) -> np.ndarray:
        """World-frame vertices of a face (cap polygon or side rectangle)."""
        t2 = self.thickness / 2.0
        o = self.outline_uv
        if name == "+w":
            local = np.column_stack([o, np.full(len(o), t2)])
        elif name == "-w":
            local = np.column_stack([o[::-1], np.full(len(o), -t2)])
        else:
            k = int(name[1:])
            a, b, _, _, _ = self._side(k)
            local = np.array([[a[0], a[1], -t2], [b[0], b[1], -t2],
                              [b[0], b[1], t2], [a[0], a[1], t2]])
        T = self.T_world_part
        return local @ T[:3, :3].T + T[:3, 3]

    def face_normal(self, name: str) -> np.ndarray:
        R = self.T_world_part[:3, :3]
        if name == "+w":
            return R @ np.array([0.0, 0.0, 1.0])
        if name == "-w":
            return R @ np.array([0.0, 0.0, -1.0])
        _, _, _, n2, _ = self._side(int(name[1:]))
        return R @ np.array([n2[0], n2[1], 0.0])

    def face_center(self, name: str) -> np.ndarray:
        return self.face_vertices(name).mean(axis=0)

    def face_plane(self, name: str) -> Plane:
        n = self.face_normal(name)
        return Plane(n=n, d=float(-n @ self.face_center(name)))

    def _cap_area(self) -> float:
        o = self.outline_uv
        return 0.5 * abs(float(np.sum(o[:, 0] * np.roll(o[:, 1], -1)
                                      - np.roll(o[:, 0], -1) * o[:, 1])))

    def face_area(self, name: str) -> float:
        if name in ("+w", "-w"):
            return self._cap_area()
        return self._side(int(name[1:]))[4] * self.thickness

    @property
    def surface_area_mm2(self) -> float:
        return 2.0 * self._cap_area() + sum(
            self.face_area(f"s{k}") for k in range(len(self.outline_uv)))

    def face_extent_along(self, name: str, direction: np.ndarray) -> tuple[float, float]:
        proj = self.face_vertices(name) @ np.asarray(direction, dtype=float)
        return float(proj.min()), float(proj.max())

    # --- the 2-D frame every clip/closest query shares ---------------------------------
    def _face_frame(self, name: str):
        """`(origin, e1, e2, poly2d)`: a world orthonormal in-face frame and the face's
        vertices in it. The clip and closest-point logic below is one implementation over
        this frame for caps and sides alike."""
        V = self.face_vertices(name)
        n = self.face_normal(name)
        e1 = V[1] - V[0]
        e1 = e1 / max(np.linalg.norm(e1), 1e-12)
        e2 = np.cross(n, e1)
        poly = (V - V[0]) @ np.column_stack([e1, e2])
        return V[0], e1, e2, poly

    def face_clip_line(self, name: str, point: np.ndarray, direction: np.ndarray,
                       slack_mm: float = 0.0) -> tuple[float, float] | None:
        """Same contract as `Slab.face_clip_line`, over an arbitrary convex face.

        The face polygon's edge half-planes clip the line's parameter interval; a
        half-plane the line runs parallel to becomes a containment test with `slack_mm`,
        exactly the slab rule (the gap offsets a seam line only along coordinates the
        line holds constant).
        """
        origin, e1, e2, poly = self._face_frame(name)
        p2 = np.array([float((point - origin) @ e1), float((point - origin) @ e2)])
        d2 = np.array([float(direction @ e1), float(direction @ e2)])
        k = len(poly)
        cons = []                       # (num, den) with inward-positive unit normals
        for i in range(k):
            a, b = poly[i], poly[(i + 1) % k]
            e = b - a
            n2 = np.array([e[1], -e[0]])
            n2 = n2 / max(float(np.linalg.norm(n2)), 1e-12)   # unit, so slack is in mm
            # orient toward the polygon interior using its centroid
            if float((poly.mean(axis=0) - a) @ n2) < 0:
                n2 = -n2
            cons.append((float((a - p2) @ n2), float(d2 @ n2)))

        def clip(skip: int | None):
            lo, hi = -np.inf, np.inf
            for j, (num, den) in enumerate(cons):
                if j == skip:
                    continue
                if abs(den) < 1e-12:
                    if num > float(slack_mm) + 1e-9:
                        return None     # parallel and outside the slack
                    continue
                s_ = num / den
                if den > 0:
                    lo = max(lo, s_)
                else:
                    hi = min(hi, s_)
            if hi - lo <= 1e-9 or not np.isfinite(lo) or not np.isfinite(hi):
                return None
            return lo, hi

        got = clip(None)
        if got is not None:
            return float(got[0]), float(got[1])
        # near-parallel waiver: same rule and same reasoning as the slab clip above -
        # drop (never extend past) one constraint whose breach stays within the slack
        # over the run the remaining constraints allow.
        if float(slack_mm) <= 0.0:
            return None
        for j, (num, den) in enumerate(cons):
            got = clip(j)
            if got is None:
                continue
            # inward signed distance along the run; breach is where it goes negative
            worst = min(-num + got[0] * den, -num + got[1] * den)
            if worst >= -(float(slack_mm) + 1e-9):
                return float(got[0]), float(got[1])
        return None

    def closest_on_face(self, name: str, pts: np.ndarray) -> np.ndarray:
        """Closest point on the finite face patch, for each query point."""
        origin, e1, e2, poly = self._face_frame(name)
        rel = np.atleast_2d(np.asarray(pts, dtype=float)) - origin
        q = np.column_stack([rel @ e1, rel @ e2])
        k = len(poly)
        cen = poly.mean(axis=0)
        inside = np.ones(len(q), dtype=bool)
        for i in range(k):
            a, b = poly[i], poly[(i + 1) % k]
            e = b - a
            n2 = np.array([e[1], -e[0]])
            n2 = n2 / max(float(np.linalg.norm(n2)), 1e-12)
            if float((cen - a) @ n2) < 0:
                n2 = -n2
            inside &= (q - a) @ n2 >= -1e-9
        out = q.copy()
        todo = ~inside
        if todo.any():
            best_d = np.full(int(todo.sum()), np.inf)
            best_p = np.zeros((int(todo.sum()), 2))
            qq = q[todo]
            for i in range(k):
                a, b = poly[i], poly[(i + 1) % k]
                e = b - a
                L2 = float(e @ e)
                t = np.clip((qq - a) @ e / max(L2, 1e-12), 0.0, 1.0)
                proj = a + t[:, None] * e
                d = np.linalg.norm(qq - proj, axis=1)
                m = d < best_d
                best_d[m] = d[m]
                best_p[m] = proj[m]
            out[todo] = best_p
        return origin + out[:, :1] * e1 + out[:, 1:] * e2

    # --- solid queries ----------------------------------------------------------------
    def halfspaces(self) -> tuple[np.ndarray, np.ndarray]:
        """World `(normals, offsets)` with inside = `n·p + d <= 0`, for the ray clip."""
        ns, ds = [], []
        for name in self.face_names():
            pl = self.face_plane(name)
            ns.append(pl.n)
            ds.append(pl.d)
        return np.asarray(ns), np.asarray(ds)

    def contains(self, points: np.ndarray, tol: float = 0.0) -> np.ndarray:
        ns, ds = self.halfspaces()
        p = np.atleast_2d(np.asarray(points, dtype=float))
        return np.all(p @ ns.T + ds[None, :] <= float(tol) + 1e-9, axis=1)

    def mesh(self) -> trimesh.Trimesh:
        """Watertight, outward-wound mesh: fan-triangulated caps plus side quads.

        Built by hand rather than through `trimesh.creation.extrude_polygon`, which pulls
        in shapely - D9 keeps the tier-1 core free of optional native deps.
        """
        o = self.outline_uv
        k = len(o)
        t2 = self.thickness / 2.0
        verts = np.vstack([np.column_stack([o, np.full(k, -t2)]),
                           np.column_stack([o, np.full(k, t2)])])
        faces = []
        for i in range(1, k - 1):                      # -w cap, wound to face -w
            faces.append([0, i + 1, i])
        for i in range(1, k - 1):                      # +w cap, wound to face +w
            faces.append([k, k + i, k + i + 1])
        for i in range(k):                             # sides, outward for CCW outline
            j = (i + 1) % k
            faces.append([i, j, k + j])
            faces.append([i, k + j, k + i])
        m = trimesh.Trimesh(vertices=verts, faces=np.asarray(faces), process=False)
        m.apply_transform(self.T_world_part)
        return m


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
