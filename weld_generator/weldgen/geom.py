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


@dataclass
class Tube:
    """Circular pipe with wall thickness — the Phase 6b revolved primitive (D29 #2-#4).

    Local frame per SCHEMA §2.2: **w is the axis** (`+z` local), the base end at
    z = 0 and the top cap at z = `length_mm`. Face registry: `lateral+` (outer wall),
    `lateral-` (inner wall), `+w` (top annulus), `-w` (the base end). `thickness_mm`
    is the WALL — the `t` every ISO limit keys on for pipe.

    The base end is optionally CUT by the surface the stub lands on (`base_cut`):

      * `{"kind": "plane", "n_local": [...], "d": ...}` — a miter against a plane
        (pipe-on-plate, tilted or not); heights solve `n·p = d` per (φ, ρ).
      * `{"kind": "cylinder", "point_local": [...], "axis_local": [...],
         "radius_mm": ...}` — a saddle cut against another cylinder (pipe-to-pipe);
        heights are the exact D33 quadratic root per (φ, ρ), so the miter ring IS the
        saddle curve, to machine precision, at every radius through the wall.

    `gap_mm` retracts the cut end along +z (the root gap). Cloud points are sampled
    ANALYTICALLY on the true surfaces — the tessellated `mesh()` exists for D21
    checks, exports and occlusion, and carries the only chord error (D34), which
    `max_chord_error_mm` reports.
    """

    id: str
    role: str
    object_id: int
    r_outer_mm: float
    wall_mm: float
    length_mm: float
    T_world_part: np.ndarray
    base_cut: dict | None = None
    gap_mm: float = 0.0

    #: Tessellation: chord error r*(dphi)^2/8 <= _MESH_TOL_MM decides the ring count.
    _MESH_TOL_MM = 0.05

    def __post_init__(self):
        if not (0.0 < self.wall_mm < self.r_outer_mm):
            raise ValueError("wall must be positive and under the outer radius")
        if self.length_mm <= 0.0:
            raise ValueError("length must be positive")

    # --- registry ---------------------------------------------------------------------
    def face_names(self) -> tuple[str, ...]:
        return ("lateral+", "lateral-", "+w", "-w")

    @property
    def r_inner_mm(self) -> float:
        return self.r_outer_mm - self.wall_mm

    @property
    def thickness_mm(self) -> float:
        return float(self.wall_mm)

    @property
    def dims_mm(self) -> tuple[float, float, float]:
        d = 2.0 * self.r_outer_mm
        return (d, d, float(self.length_mm))

    @property
    def part_geometry_id(self) -> str:
        return f"tube_{2 * self.r_outer_mm}x{self.wall_mm}x{self.length_mm}"

    # --- the exact base-cut height ----------------------------------------------------
    def base_height(self, phi, radius) -> np.ndarray:
        """Local z of the base end at `phi` and `radius` (exact; both broadcast)."""
        phi = np.atleast_1d(np.asarray(phi, dtype=float))
        radius = np.broadcast_to(np.asarray(radius, dtype=float), phi.shape)
        if self.base_cut is None:
            return np.full(len(phi), self.gap_mm)
        x = radius * np.cos(phi)
        y = radius * np.sin(phi)
        cut = self.base_cut
        if cut["kind"] == "plane":
            n = np.asarray(cut["n_local"], dtype=float)
            if abs(n[2]) < 1e-9:
                raise ValueError("base-cut plane parallel to the axis")
            z = (float(cut["d"]) - n[0] * x - n[1] * y) / n[2]
            return z + self.gap_mm
        if cut["kind"] == "cylinder":
            p0 = np.asarray(cut["point_local"], dtype=float)
            m = np.asarray(cut["axis_local"], dtype=float)
            m = m / np.linalg.norm(m)
            R = float(cut["radius_mm"])
            # line (x, y, z) along +z into the main cylinder's implicit form:
            # A z^2 + B z + C = 0 with A = 1 - m_z^2  (D33's quadratic, per radius)
            dx = x - p0[0]
            dy = y - p0[1]
            dm_xy = dx * m[0] + dy * m[1]
            # d(z) = (dx, dy, z - p0z); |d|^2 - (d.m)^2 = R^2 expands to
            # A z^2 + B z + C with the coefficients below (C is the form evaluated
            # at z = 0, B its z-derivative there).
            A = 1.0 - m[2] ** 2
            B = 2.0 * ((-p0[2]) - (dm_xy + (-p0[2]) * m[2]) * m[2])
            C = (dx * dx + dy * dy + p0[2] * p0[2]
                 - (dm_xy - p0[2] * m[2]) ** 2 - R * R)
            disc = B * B - 4.0 * A * C
            if np.any(disc <= 0.0):
                raise ValueError("saddle cut: stub does not fully meet the cylinder")
            # +z points AWAY from the main pipe; the stub must stop at FIRST contact,
            # the root with the larger z.
            return (-B + np.sqrt(disc)) / (2.0 * A) + self.gap_mm
        raise ValueError(f"unknown base_cut kind {cut['kind']!r}")

    def _base_span(self, radius: float, n: int = 720) -> tuple[float, float]:
        h = self.base_height(np.linspace(0.0, 2.0 * np.pi, n, endpoint=False), radius)
        return float(h.min()), float(h.max())

    # --- solid queries ----------------------------------------------------------------
    def _local(self, points: np.ndarray) -> np.ndarray:
        T = self.T_world_part
        return (np.atleast_2d(np.asarray(points, dtype=float)) - T[:3, 3]) @ T[:3, :3]

    def contains(self, points: np.ndarray, tol: float = 0.0) -> np.ndarray:
        p = self._local(points)
        rho = np.hypot(p[:, 0], p[:, 1])
        phi = np.arctan2(p[:, 1], p[:, 0])
        ok = (rho <= self.r_outer_mm + tol) & (rho >= self.r_inner_mm - tol) \
            & (p[:, 2] <= self.length_mm + tol)
        if ok.any():
            base = self.base_height(phi[ok], np.clip(rho[ok], self.r_inner_mm,
                                                     self.r_outer_mm))
            sub = ok[ok].copy()
            sub &= p[ok, 2] >= base - tol
            ok[ok] = sub
        return ok

    # --- registry geometry ------------------------------------------------------------
    def face_area(self, name: str) -> float:
        if name == "+w":
            return float(np.pi * (self.r_outer_mm ** 2 - self.r_inner_mm ** 2))
        if name == "-w":
            # slanted annulus; area to density purposes only - flat-annulus estimate
            return float(np.pi * (self.r_outer_mm ** 2 - self.r_inner_mm ** 2))
        r = self.r_outer_mm if name == "lateral+" else self.r_inner_mm
        phis = np.linspace(0.0, 2.0 * np.pi, 720, endpoint=False)
        h = np.clip(self.length_mm - self.base_height(phis, r), 0.0, None)
        return float(r * np.mean(h) * 2.0 * np.pi)

    @property
    def surface_area_mm2(self) -> float:
        return sum(self.face_area(n) for n in self.face_names())

    def face_normal(self, name: str):
        R = self.T_world_part[:3, :3]
        if name == "+w":
            return R @ np.array([0.0, 0.0, 1.0])
        if name == "-w":
            return R @ np.array([0.0, 0.0, -1.0])
        raise ValueError(f"{name}: a curved face has no single normal")

    def face_plane(self, name: str):
        if name == "+w":
            n = self.face_normal(name)
            c = self.T_world_part @ np.array([0.0, 0.0, self.length_mm, 1.0])
            return Plane(n=n, d=float(-n @ c[:3]))
        return None                                       # curved / cut faces: surface

    def surface_desc(self, name: str) -> dict | None:
        """The `faces[].surface` block for the curved faces (world frame)."""
        if name not in ("lateral+", "lateral-"):
            return None
        T = self.T_world_part
        return {"kind": "cylinder",
                "point_mm": [float(v) for v in T[:3, 3]],
                "axis": [float(v) for v in T[:3, 2]],
                "radius_mm": float(self.r_outer_mm if name == "lateral+"
                                   else self.r_inner_mm),
                "outward": name == "lateral+"}

    # --- analytic surface sampling (no mesh involved: positions are exact) -----------
    def sample_face(self, name: str, n_pts: int, rng) -> tuple[np.ndarray, np.ndarray]:
        """(points, normals) in WORLD frame, area-uniform, exactly on the surface.

        The lateral draw uses a discretised inverse CDF over φ weighted by the local
        height span (a cut base makes the wall taller on one side): the φ DISTRIBUTION
        carries grid error, every position is still exactly on the cylinder.
        """
        two_pi = 2.0 * np.pi
        if name in ("lateral+", "lateral-"):
            r = self.r_outer_mm if name == "lateral+" else self.r_inner_mm
            grid = np.linspace(0.0, two_pi, 2048, endpoint=False)
            span = np.clip(self.length_mm - self.base_height(grid, r), 1e-9, None)
            cdf = np.concatenate([[0.0], np.cumsum(span)])
            cdf /= cdf[-1]
            u = rng.random(n_pts)
            phi = np.interp(u, cdf, np.concatenate([grid, [two_pi]]))
            z0 = self.base_height(phi, r)
            z = z0 + rng.random(n_pts) * (self.length_mm - z0)
            local = np.column_stack([r * np.cos(phi), r * np.sin(phi), z])
            sign = 1.0 if name == "lateral+" else -1.0
            n_loc = np.column_stack([sign * np.cos(phi), sign * np.sin(phi),
                                     np.zeros(n_pts)])
        elif name == "+w":
            phi = rng.random(n_pts) * two_pi
            r = np.sqrt(rng.uniform(self.r_inner_mm ** 2, self.r_outer_mm ** 2,
                                    n_pts))
            local = np.column_stack([r * np.cos(phi), r * np.sin(phi),
                                     np.full(n_pts, self.length_mm)])
            n_loc = np.tile([0.0, 0.0, 1.0], (n_pts, 1))
        elif name == "-w":
            phi = rng.random(n_pts) * two_pi
            r = np.sqrt(rng.uniform(self.r_inner_mm ** 2, self.r_outer_mm ** 2,
                                    n_pts))
            z = self.base_height(phi, r)
            local = np.column_stack([r * np.cos(phi), r * np.sin(phi), z])
            n_loc = np.tile([0.0, 0.0, -1.0], (n_pts, 1))    # nominal; cut face ~ -w
        else:
            raise ValueError(name)
        T = self.T_world_part
        return local @ T[:3, :3].T + T[:3, 3], n_loc @ T[:3, :3].T

    # --- mesh (D21 / D34) -------------------------------------------------------------
    def _n_phi(self) -> int:
        dphi = np.sqrt(8.0 * self._MESH_TOL_MM / self.r_outer_mm)
        return max(48, int(np.ceil(2.0 * np.pi / dphi)))

    @property
    def max_chord_error_mm(self) -> float:
        return float(self.r_outer_mm * (2.0 * np.pi / self._n_phi()) ** 2 / 8.0)

    def mesh(self) -> trimesh.Trimesh:
        n = self._n_phi()
        phis = np.linspace(0.0, 2.0 * np.pi, n, endpoint=False)
        ro, ri, L = self.r_outer_mm, self.r_inner_mm, self.length_mm
        rings = [np.column_stack([ro * np.cos(phis), ro * np.sin(phis),
                                  self.base_height(phis, ro)]),         # outer base
                 np.column_stack([ro * np.cos(phis), ro * np.sin(phis),
                                  np.full(n, L)]),                      # outer top
                 np.column_stack([ri * np.cos(phis), ri * np.sin(phis),
                                  np.full(n, L)]),                      # inner top
                 np.column_stack([ri * np.cos(phis), ri * np.sin(phis),
                                  self.base_height(phis, ri)])]         # inner base
        verts = np.vstack(rings)
        faces = []
        for a in range(4):                                # strip a -> a+1, closed loop
            b = (a + 1) % 4
            for i in range(n):
                j = (i + 1) % n
                faces.append([a * n + i, a * n + j, b * n + j])
                faces.append([a * n + i, b * n + j, b * n + i])
        m = trimesh.Trimesh(vertices=verts, faces=np.asarray(faces), process=False)
        if m.volume < 0:                                  # orient outward
            m.invert()
        m.apply_transform(self.T_world_part)
        return m


@dataclass
class SweptSlab:
    """A band offset from a spine curve, extruded in local z — Phase 6b (D29 #5-#7).

    `spine` is a `weldgen.curves` object living in the local z = 0 plane. The solid is
    the offset band `[offset_lo, offset_hi]` about it (offsets measured along
    `n̂(s) = ẑ × T̂(s)`, the left of travel), extruded over `z ∈ [z0, z1]`. One
    primitive, three configurations:

      * #5 rect tube on plate — CLOSED spine (the rounded rectangle, which is the
        step-1 seam), band `[-wall, 0]`: the spine surface IS the outer wall.
      * #6 swept stiffener — open spine, band `[-t/2, +t/2]`, standing (z1 > z0 > 0).
      * #7 curved butt — open arc spine, two flat one-sided bands either side of it.

    Face registry (SCHEMA §2.2 `swept_slab`): `+w` / `-w` are the offset surfaces at
    `offset_hi` / `offset_lo` (the BROAD faces — thickness = the band width, so the
    w-is-thickness invariant survives), `+v` / `-v` the top/bottom caps, and `+u` /
    `-u` the end caps of an open spine (absent for a closed one).

    Positions and normals on every face are ANALYTIC (spine point + exact tangent
    normal), so cloud samples carry no chord error; the tessellated mesh (D21/D34,
    occlusion) is the only discretised object, at a pitch chosen from the offset
    curvature, and `contains` resolves against a fine spine polyline — a boolean-only
    approximation, documented where the tube's interval refinement is.
    """

    id: str
    role: str
    object_id: int
    spine: object
    offset_lo_mm: float
    offset_hi_mm: float
    z0_mm: float
    z1_mm: float
    T_world_part: np.ndarray

    _MESH_TOL_MM = 0.05
    _N_FINE = 2048

    def __post_init__(self):
        if self.offset_hi_mm - self.offset_lo_mm <= 0.0:
            raise ValueError("band width must be positive")
        if self.z1_mm - self.z0_mm <= 0.0:
            raise ValueError("height must be positive")
        # self-intersection guard: 1 - kappa*offset must stay positive at both edges
        sp = self._speed_kappa()[1]
        for o in (self.offset_lo_mm, self.offset_hi_mm):
            if np.min(1.0 - sp * o) <= 0.05:
                raise ValueError("offset exceeds the spine's curvature radius")

    # --- spine machinery --------------------------------------------------------------
    def _grid(self):
        return np.linspace(0.0, self.spine.t_period, self._N_FINE,
                           endpoint=not self.spine.closed)

    def _speed_kappa(self):
        """(speed, signed curvature) on the fine grid — numeric, boolean/weight use
        only; face positions never touch it."""
        if not hasattr(self, "_sk"):
            ts = self._grid()
            h = self.spine.t_period * 1e-6
            p_plus = self.spine.point(ts + h)
            p_minus = self.spine.point(ts - h)
            d1 = (p_plus - p_minus) / (2.0 * h)
            d2 = (p_plus + p_minus - 2.0 * self.spine.point(ts)) / (h * h)
            speed = np.linalg.norm(d1[:, :2], axis=1)
            cross = d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]
            with np.errstate(divide="ignore", invalid="ignore"):
                kappa = cross / np.clip(speed, 1e-12, None) ** 3
            self._sk = (speed, np.nan_to_num(kappa))
        return self._sk

    def _normal2(self, ts):
        tan = self.spine.tangent(ts)
        return np.column_stack([-tan[:, 1], tan[:, 0], np.zeros(len(tan))])

    def offset_point(self, ts, offset: float) -> np.ndarray:
        """Exact point on the offset surface at spine parameter(s) `ts` (local, z=0)."""
        ts = np.atleast_1d(np.asarray(ts, dtype=float))
        return self.spine.point(ts) + offset * self._normal2(ts)

    # --- registry ---------------------------------------------------------------------
    def face_names(self) -> tuple[str, ...]:
        base = ("+w", "-w", "+v", "-v")
        return base if self.spine.closed else base + ("+u", "-u")

    @property
    def thickness_mm(self) -> float:
        return float(self.offset_hi_mm - self.offset_lo_mm)

    @property
    def dims_mm(self) -> tuple[float, float, float]:
        pts = self.spine.point(self._grid())
        return (float(np.ptp(pts[:, 0])), float(np.ptp(pts[:, 1])),
                float(self.z1_mm - self.z0_mm))

    @property
    def part_geometry_id(self) -> str:
        kind = self.spine.to_parametric()["kind"]
        return (f"swept_{kind}_{self.spine.length_mm:.6g}x"
                f"{self.thickness_mm:.6g}x{self.z1_mm - self.z0_mm:.6g}")

    def face_area(self, name: str) -> float:
        speed, kappa = self._speed_kappa()
        ds = self.spine.t_period / self._N_FINE
        H = self.z1_mm - self.z0_mm
        if name in ("+w", "-w"):
            o = self.offset_hi_mm if name == "+w" else self.offset_lo_mm
            return float(np.sum(speed * np.abs(1.0 - kappa * o)) * ds * H)
        if name in ("+v", "-v"):
            w = np.zeros_like(speed)
            for o0, o1 in [(self.offset_lo_mm, self.offset_hi_mm)]:
                w = speed * ((o1 - o0) - kappa * (o1 ** 2 - o0 ** 2) / 2.0)
            return float(abs(np.sum(w) * ds))
        return float(self.thickness_mm * H)                    # end caps

    @property
    def surface_area_mm2(self) -> float:
        return sum(self.face_area(n) for n in self.face_names())

    def face_plane(self, name: str):
        return None if name in ("+w", "-w") else self._cap_plane(name)

    def _cap_plane(self, name: str):
        R = self.T_world_part[:3, :3]
        t = self.T_world_part[:3, 3]
        if name in ("+v", "-v"):
            n = R @ np.array([0.0, 0.0, 1.0 if name == "+v" else -1.0])
            z = self.z1_mm if name == "+v" else self.z0_mm
            c = R @ np.array([0.0, 0.0, z]) + t
            return Plane(n=n, d=float(-n @ c))
        ts = self.spine.t_period if name == "+u" else 0.0
        tan = self.spine.tangent([ts])[0]
        sign = 1.0 if name == "+u" else -1.0
        n = R @ (sign * tan)
        c = R @ self.spine.point([ts])[0] + t
        return Plane(n=n, d=float(-n @ c))

    def surface_desc(self, name: str) -> dict | None:
        if name not in ("+w", "-w"):
            return None
        o = self.offset_hi_mm if name == "+w" else self.offset_lo_mm
        return {"kind": "offset_extrusion",
                "spine": self.spine.to_parametric(),
                "offset_mm": float(o),
                "z0_mm": float(self.z0_mm), "z1_mm": float(self.z1_mm),
                "T_world_part": [[float(v) for v in row]
                                 for row in self.T_world_part]}

    # --- analytic sampling ------------------------------------------------------------
    def sample_face(self, name: str, n_pts: int, rng) -> tuple[np.ndarray, np.ndarray]:
        speed, kappa = self._speed_kappa()
        ts_grid = self._grid()

        def draw_ts(weights, n):
            cdf = np.concatenate([[0.0], np.cumsum(np.clip(weights, 1e-12, None))])
            cdf /= cdf[-1]
            return np.interp(rng.random(n), cdf,
                             np.linspace(0.0, self.spine.t_period, len(cdf)))

        if name in ("+w", "-w"):
            o = self.offset_hi_mm if name == "+w" else self.offset_lo_mm
            ts = draw_ts(speed * np.abs(1.0 - kappa * o), n_pts)
            z = rng.uniform(self.z0_mm, self.z1_mm, n_pts)
            local = self.offset_point(ts, o)
            local[:, 2] = z
            n2 = self._normal2(ts) * (1.0 if name == "+w" else -1.0)
            n_loc = n2
        elif name in ("+v", "-v"):
            ts = draw_ts(speed, n_pts)                        # band-width ~ uniform
            o = rng.uniform(self.offset_lo_mm, self.offset_hi_mm, n_pts)
            local = self.spine.point(ts) + o[:, None] * self._normal2(ts)
            local[:, 2] = self.z1_mm if name == "+v" else self.z0_mm
            n_loc = np.tile([0.0, 0.0, 1.0 if name == "+v" else -1.0], (n_pts, 1))
        elif name in ("+u", "-u"):
            ts = np.full(n_pts, self.spine.t_period if name == "+u" else 0.0)
            o = rng.uniform(self.offset_lo_mm, self.offset_hi_mm, n_pts)
            z = rng.uniform(self.z0_mm, self.z1_mm, n_pts)
            local = self.spine.point(ts) + o[:, None] * self._normal2(ts)
            local[:, 2] = z
            tan = self.spine.tangent(ts)
            n_loc = tan * (1.0 if name == "+u" else -1.0)
        else:
            raise ValueError(name)
        T = self.T_world_part
        return local @ T[:3, :3].T + T[:3, 3], n_loc @ T[:3, :3].T

    # --- solid queries (fine-polyline resolution: boolean use only) -------------------
    def _poly(self):
        if not hasattr(self, "_pl"):
            ts = self._grid()
            self._pl = (ts, self.spine.point(ts)[:, :2])
        return self._pl

    def contains(self, points: np.ndarray, tol: float = 0.0) -> np.ndarray:
        T = self.T_world_part
        p = (np.atleast_2d(np.asarray(points, dtype=float)) - T[:3, 3]) @ T[:3, :3]
        ok_z = (p[:, 2] >= self.z0_mm - tol) & (p[:, 2] <= self.z1_mm + tol)
        out = np.zeros(len(p), dtype=bool)
        if not ok_z.any():
            return out
        ts, poly = self._poly()
        q = p[ok_z, :2]
        # nearest spine vertex (fine grid), then signed offset via the local normal
        best = np.empty(len(q), dtype=int)
        step = 200000 // max(len(poly), 1) + 1
        for i0 in range(0, len(q), step):
            d2 = ((q[i0:i0 + step, None, :] - poly[None, :, :]) ** 2).sum(-1)
            best[i0:i0 + step] = np.argmin(d2, axis=1)
        near_ts = ts[best]
        n2 = self._normal2(near_ts)[:, :2]
        off = np.einsum("ij,ij->i", q - poly[best], n2)
        ok = (off >= self.offset_lo_mm - tol) & (off <= self.offset_hi_mm + tol)
        if not self.spine.closed:
            interior = (best > 0) & (best < len(poly) - 1)
            ok &= interior
        out[ok_z] = ok
        return out

    # --- mesh (D21 / D34) -------------------------------------------------------------
    def _n_stations(self) -> int:
        speed, kappa = self._speed_kappa()
        k_off = max(float(np.max(np.abs(kappa / (1.0 - kappa * self.offset_hi_mm)))),
                    float(np.max(np.abs(kappa / (1.0 - kappa * self.offset_lo_mm)))),
                    1e-6)
        d_arc = np.sqrt(8.0 * self._MESH_TOL_MM / k_off)
        n = int(np.ceil(self.spine.length_mm / d_arc))
        return max(64, n)

    @property
    def max_chord_error_mm(self) -> float:
        speed, kappa = self._speed_kappa()
        n = self._n_stations()
        d_arc = self.spine.length_mm / n
        k_off = max(float(np.max(np.abs(kappa))), 1e-9)
        return float(k_off * d_arc ** 2 / 8.0)

    def mesh(self) -> trimesh.Trimesh:
        n = self._n_stations()
        s_vals = self.spine.arclengths(n)
        ts = self.spine.t_at_arclength(s_vals)
        lo = self.offset_point(ts, self.offset_lo_mm)
        hi = self.offset_point(ts, self.offset_hi_mm)
        rings = []
        for base, z in ((lo, self.z0_mm), (hi, self.z0_mm),
                        (hi, self.z1_mm), (lo, self.z1_mm)):
            ring = base.copy()
            ring[:, 2] = z
            rings.append(ring)
        verts = np.vstack(rings)
        faces = []
        m_seg = n if self.spine.closed else n - 1
        for a in range(4):
            b = (a + 1) % 4
            for i in range(m_seg):
                j = (i + 1) % n
                faces.append([a * n + i, a * n + j, b * n + j])
                faces.append([a * n + i, b * n + j, b * n + i])
        if not self.spine.closed:                            # end caps
            for i, flip in ((0, False), (n - 1, True)):
                quad = [0 * n + i, 1 * n + i, 2 * n + i, 3 * n + i]
                tri1 = [quad[0], quad[1], quad[2]]
                tri2 = [quad[0], quad[2], quad[3]]
                if flip:
                    tri1 = tri1[::-1]
                    tri2 = tri2[::-1]
                faces += [tri1, tri2]
        m = trimesh.Trimesh(vertices=verts, faces=np.asarray(faces), process=False)
        if m.volume < 0:
            m.invert()
        m.apply_transform(self.T_world_part)
        return m


@dataclass
class PreparedSlab:
    """A plate with an ISO 9692-1 edge preparation on its seam edge — Phase 6b step 4.

    Local frame like the butt plate it replaces: `u` = length (the seam direction),
    `v` = width with the PREPARED edge at v = 0 and material at v <= 0, `w` = thickness
    with the top face at w = 0 and material down to w = -t (grooves are welded from
    the top). The prepared edge is a MONOTONE profile `v_edge(w)`:

      * `single_V`     — root face of height `c`, then a straight fusion face receding
                         at `bevel_deg` from vertical (α/2 of the V's included angle;
                         each plate carries half the V).
      * `single_bevel` — same profile with the FULL angle on this plate (its partner
                         stays a square Slab; 9692-1 ref 1.9.1).
      * `single_U`     — root face, then a radius `R` sweeping from horizontal to
                         `bevel_deg` from vertical, then the straight fusion face
                         (refs 1.6 / 2.6). Requires t - c > R(1 - sin β).

    Face registry: `+u -u` ends, `-v` back, `+w` top, `-w` bottom, and the prepared
    faces named by ISO 17659 Table 1/2: `root` (root face, ref 4/12), `fusion` (fusion
    face, ref E), `radius` (the U's curved run; plane None, cylinder surface desc).
    Monotone `v_edge(w)` is what keeps `contains` EXACT in closed form, so the ray
    dispatch can refine against it, and every planar face is a rectangle strip - the
    slab face interface (clip, closest, extent) reduces to corner arithmetic.
    """

    id: str
    role: str
    object_id: int
    length_mm: float
    width_mm: float
    t_mm: float
    prep: dict                                 # kind, bevel_deg, root_face_mm[, radius_mm]
    T_world_part: np.ndarray

    _MESH_TOL_MM = 0.05

    def __post_init__(self):
        k = self.prep["kind"]
        if k not in ("single_V", "single_bevel", "single_U"):
            raise ValueError(f"unknown preparation {k!r}")
        self._beta = np.radians(float(self.prep["bevel_deg"]))
        self._c = float(self.prep["root_face_mm"])
        if not (0.0 < self._c < self.t_mm):
            raise ValueError("root face must be inside the thickness")
        if k == "single_U":
            self._R = float(self.prep["radius_mm"])
            run = self.t_mm - self._c - self._R * (1.0 - np.sin(self._beta))
            if run <= 0.5:
                raise ValueError("U radius leaves no straight fusion face")
            self._z_arc_top = -(self.t_mm - self._c) + self._R * (1.0 - np.sin(self._beta))
            self._v_arc_top = -self._R * np.cos(self._beta)
        else:
            self._R = 0.0

    # --- the profile ------------------------------------------------------------------
    def v_edge(self, w) -> np.ndarray:
        """The prepared edge's v at height(s) w (exact; w in [-t, 0])."""
        w = np.atleast_1d(np.asarray(w, dtype=float))
        z_root_top = -(self.t_mm - self._c)
        out = np.zeros(len(w))
        above = w > z_root_top
        if self.prep["kind"] in ("single_V", "single_bevel"):
            out[above] = -(w[above] - z_root_top) * np.tan(self._beta)
        else:
            arc = above & (w <= self._z_arc_top)
            hi = w > self._z_arc_top
            # arc: centre at (v=0, w=z_root_top + R) in the profile plane, radius R:
            # v = -sqrt(R^2 - (w - (z_root_top + R))^2) for the lower quarter
            zc = z_root_top + self._R
            out[arc] = -np.sqrt(np.clip(self._R ** 2 - (w[arc] - zc) ** 2, 0.0, None))
            out[hi] = self._v_arc_top - (w[hi] - self._z_arc_top) * np.tan(self._beta)
        return out

    @property
    def mouth_v_mm(self) -> float:
        """How far the prepared edge recedes at the top surface."""
        return float(self.v_edge(np.array([0.0]))[0])

    # --- registry ---------------------------------------------------------------------
    def face_names(self) -> tuple[str, ...]:
        base = ("+u", "-u", "-v", "+w", "-w", "root", "fusion")
        return base + ("radius",) if self.prep["kind"] == "single_U" else base

    @property
    def thickness_mm(self) -> float:
        return float(self.t_mm)

    @property
    def dims_mm(self) -> tuple[float, float, float]:
        return (float(self.length_mm), float(self.width_mm), float(self.t_mm))

    @property
    def part_geometry_id(self) -> str:
        return (f"prepared_{self.prep['kind'].lower()}_{self.length_mm}x"
                f"{self.width_mm}x{self.t_mm}")

    def _face_corners(self, name: str) -> np.ndarray:
        """The 4 local corners of a PLANAR face, CCW seen from outside."""
        L2, W, t = self.length_mm / 2.0, self.width_mm, self.t_mm
        zr = -(t - self._c)
        vm = self.mouth_v_mm
        if self.prep["kind"] == "single_U":
            z_lo, v_lo = self._z_arc_top, self._v_arc_top   # fusion starts atop the arc
        else:
            z_lo, v_lo = zr, 0.0
        C = {
            "-v": [(-L2, -W, -t), (L2, -W, -t), (L2, -W, 0.0), (-L2, -W, 0.0)],
            "+w": [(-L2, -W, 0.0), (L2, -W, 0.0), (L2, vm, 0.0), (-L2, vm, 0.0)],
            "-w": [(-L2, 0.0, -t), (L2, 0.0, -t), (L2, -W, -t), (-L2, -W, -t)],
            "root": [(-L2, 0.0, -t), (L2, 0.0, -t), (L2, 0.0, zr), (-L2, 0.0, zr)],
            "fusion": [(-L2, v_lo, z_lo), (L2, v_lo, z_lo),
                       (L2, vm, 0.0), (-L2, vm, 0.0)],
        }
        if name in C:
            return np.asarray(C[name], dtype=float)
        raise ValueError(name)

    def face_center(self, name: str) -> np.ndarray:
        if name == "radius":
            mid = 0.5 * (np.pi / 2.0 - self._beta)
            zc = -(self.t_mm - self._c) + self._R
            local = np.array([0.0, -self._R * np.sin(mid),
                              zc - self._R * np.cos(mid)])
            T = self.T_world_part
            return T[:3, :3] @ local + T[:3, 3]
        return self.face_vertices(name).mean(axis=0)   # face_vertices is world

    def face_vertices(self, name: str) -> np.ndarray:
        """World corners of a planar face (the coplanar arm's in-plane basis source).

        End caps return their bounding rectangle - only edge DIRECTIONS are consumed
        from this, and the bevel notch does not change them.
        """
        if name in ("+u", "-u"):
            x = np.copysign(self.length_mm / 2.0, 1.0 if name == "+u" else -1.0)
            local = np.array([[x, -self.width_mm, -self.t_mm],
                              [x, 0.0, -self.t_mm],
                              [x, 0.0, 0.0],
                              [x, -self.width_mm, 0.0]])
        elif name == "radius":
            raise ValueError("the radius face is not planar")
        else:
            local = self._face_corners(name)
        T = self.T_world_part
        return local @ T[:3, :3].T + T[:3, 3]

    def face_plane(self, name: str):
        R = self.T_world_part[:3, :3]
        tr = self.T_world_part[:3, 3]
        if name == "radius":
            return None
        if name in ("+u", "-u"):
            n_loc = np.array([1.0 if name == "+u" else -1.0, 0.0, 0.0])
            p_loc = np.array([np.copysign(self.length_mm / 2.0, n_loc[0]), 0.0, 0.0])
        else:
            corners = self._face_corners(name)
            e1 = corners[1] - corners[0]
            e2 = corners[3] - corners[0]
            n_loc = np.cross(e1, e2)
            n_loc /= np.linalg.norm(n_loc)
            # orient outward: away from the material centroid
            cen = np.array([0.0, -self.width_mm / 2.0, -self.t_mm / 2.0])
            if float((corners[0] - cen) @ n_loc) < 0:
                n_loc = -n_loc
            p_loc = corners[0]
        n = R @ n_loc
        c = R @ p_loc + tr
        return Plane(n=n, d=float(-n @ c))

    def face_normal(self, name: str):
        pl = self.face_plane(name)
        if pl is None:
            raise ValueError("the radius face has no single normal")
        return pl.n

    def face_area(self, name: str) -> float:
        if name in ("+u", "-u"):
            # end cap: width*t minus the groove's profile cut - numeric strip integral
            ws = np.linspace(-self.t_mm, 0.0, 512)
            return float(np.trapezoid(self.width_mm + self.v_edge(ws), ws))
        if name == "radius":
            ang = np.pi / 2.0 - self._beta
            return float(self._R * ang * self.length_mm)
        c = self._face_corners(name)
        return float(np.linalg.norm(c[1] - c[0]) * np.linalg.norm(c[3] - c[0]))

    @property
    def surface_area_mm2(self) -> float:
        return sum(self.face_area(n) for n in self.face_names())

    def surface_desc(self, name: str) -> dict | None:
        if name != "radius":
            return None
        T = self.T_world_part
        zc = -(self.t_mm - self._c) + self._R
        axis_pt = T[:3, :3] @ np.array([0.0, 0.0, zc]) + T[:3, 3]
        return {"kind": "cylinder", "point_mm": [float(v) for v in axis_pt],
                "axis": [float(v) for v in T[:3, 0]],
                "radius_mm": float(self._R), "outward": False}

    # --- slab face interface (rect strips: corner arithmetic) -------------------------
    def face_extent_along(self, name: str, direction) -> tuple[float, float]:
        d = np.asarray(direction, dtype=float)
        T = self.T_world_part
        if name in ("+u", "-u"):
            ws = np.linspace(-self.t_mm, 0.0, 64)
            pts = np.column_stack([np.full(64, np.copysign(self.length_mm / 2.0,
                                                           1.0 if name == "+u" else -1.0)),
                                   np.zeros(64), ws])
            pts[:, 1] = 0.0
            lo = np.column_stack([pts[:, 0], -np.full(64, self.width_mm), ws])
            world = np.vstack([pts, lo]) @ T[:3, :3].T + T[:3, 3]
        elif name == "radius":
            phis = np.linspace(0.0, np.pi / 2.0 - self._beta, 64)
            zc = -(self.t_mm - self._c) + self._R
            ring = np.column_stack([np.zeros(64), -self._R * np.sin(phis),
                                    zc - self._R * np.cos(phis)])
            world = np.vstack([ring + [self.length_mm / 2.0, 0, 0],
                               ring + [-self.length_mm / 2.0, 0, 0]]) \
                @ T[:3, :3].T + T[:3, 3]
        else:
            world = self._face_corners(name) @ T[:3, :3].T + T[:3, 3]
        proj = world @ d
        return float(proj.min()), float(proj.max())

    def closest_on_face(self, name: str, pts: np.ndarray) -> np.ndarray:
        """Closest point on a PLANAR face patch (rect strip clamp); the radius face is
        never a seam-bearing patch and is not supported here."""
        corners = self._face_corners(name) if name not in ("+u", "-u") else None
        T = self.T_world_part
        q = (np.atleast_2d(np.asarray(pts, dtype=float)) - T[:3, 3]) @ T[:3, :3]
        if corners is None:
            x = np.copysign(self.length_mm / 2.0, 1.0 if name == "+u" else -1.0)
            out = q.copy()
            out[:, 0] = x
            out[:, 1] = np.clip(out[:, 1], -self.width_mm, self.v_edge(
                np.clip(out[:, 2], -self.t_mm, 0.0)))
            out[:, 2] = np.clip(out[:, 2], -self.t_mm, 0.0)
        else:
            o = corners[0]
            e1 = corners[1] - corners[0]
            e2 = corners[3] - corners[0]
            L1, L2n = np.linalg.norm(e1), np.linalg.norm(e2)
            u1, u2 = e1 / L1, e2 / L2n
            rel = q - o
            a = np.clip(rel @ u1, 0.0, L1)
            b = np.clip(rel @ u2, 0.0, L2n)
            out = o + a[:, None] * u1 + b[:, None] * u2
        return out @ T[:3, :3].T + T[:3, 3]

    def face_clip_line(self, name: str, point, direction,
                       slack_mm: float = 0.0) -> tuple[float, float] | None:
        """The Phase 6a exact 2-D clip, on a rectangle strip given by its corners
        (same slack semantics, including the near-parallel waiver)."""
        if name == "radius":
            return None
        corners = (self._face_corners(name) if name not in ("+u", "-u") else None)
        T = self.T_world_part
        p = (np.asarray(point, dtype=float) - T[:3, 3]) @ T[:3, :3]
        d = np.asarray(direction, dtype=float) @ T[:3, :3]
        if corners is None:
            return None                        # end caps never carry a seam line
        o = corners[0]
        e1 = corners[1] - corners[0]
        e2 = corners[3] - corners[0]
        L1, L2n = np.linalg.norm(e1), np.linalg.norm(e2)
        u1, u2 = e1 / L1, e2 / L2n
        rel = p - o
        p2 = np.array([rel @ u1, rel @ u2])
        d2 = np.array([d @ u1, d @ u2])
        ivs = []
        for k, Lk in ((0, L1), (1, L2n)):
            if abs(d2[k]) < 1e-12:
                if p2[k] < -slack_mm - 1e-9 or p2[k] > Lk + slack_mm + 1e-9:
                    return None
                ivs.append((-np.inf, np.inf))
            else:
                a = (0.0 - p2[k]) / d2[k]
                b = (Lk - p2[k]) / d2[k]
                ivs.append((min(a, b), max(a, b)))
        lo = max(v[0] for v in ivs)
        hi = min(v[1] for v in ivs)
        if hi - lo > 1e-9 and np.isfinite(lo) and np.isfinite(hi):
            return float(lo), float(hi)
        if slack_mm <= 0.0:
            return None
        for k, Lk in ((0, L1), (1, L2n)):
            other = ivs[1 - k]
            if not (np.isfinite(other[0]) and np.isfinite(other[1])
                    and other[1] - other[0] > 1e-9):
                continue
            c0 = p2[k] + other[0] * d2[k]
            c1 = p2[k] + other[1] * d2[k]
            if min(c0, c1) >= -slack_mm - 1e-9 and max(c0, c1) <= Lk + slack_mm + 1e-9:
                return float(other[0]), float(other[1])
        return None

    # --- solid queries ----------------------------------------------------------------
    def contains(self, points: np.ndarray, tol: float = 0.0) -> np.ndarray:
        T = self.T_world_part
        q = (np.atleast_2d(np.asarray(points, dtype=float)) - T[:3, 3]) @ T[:3, :3]
        ok = (np.abs(q[:, 0]) <= self.length_mm / 2.0 + tol) \
            & (q[:, 1] >= -self.width_mm - tol) \
            & (q[:, 2] >= -self.t_mm - tol) & (q[:, 2] <= tol)
        if ok.any():
            edge = self.v_edge(np.clip(q[ok, 2], -self.t_mm, 0.0))
            sub = ok[ok].copy()
            sub &= q[ok, 1] <= edge + tol
            ok[ok] = sub
        return ok

    # --- analytic sampling ------------------------------------------------------------
    def sample_face(self, name: str, n_pts: int, rng) -> tuple[np.ndarray, np.ndarray]:
        L2 = self.length_mm / 2.0
        if name in ("+u", "-u"):
            # rejection-free: draw (v, w) in the bounding rect, keep under the profile
            # via inverse transform on w-strips - simple grid CDF like the tube
            ws_grid = np.linspace(-self.t_mm, 0.0, 1024)
            widths = self.width_mm + self.v_edge(ws_grid)
            cdf = np.concatenate([[0.0], np.cumsum(np.clip(widths, 1e-9, None))])
            cdf /= cdf[-1]
            w = np.interp(rng.random(n_pts), cdf, np.linspace(-self.t_mm, 0.0, 1025))
            vmax = self.v_edge(w)
            v = -self.width_mm + rng.random(n_pts) * (self.width_mm + vmax)
            x = np.full(n_pts, np.copysign(L2, 1.0 if name == "+u" else -1.0))
            local = np.column_stack([x, v, w])
            n_loc = np.tile([np.copysign(1.0, x[0]), 0.0, 0.0], (n_pts, 1))
        elif name == "radius":
            phis = rng.uniform(0.0, np.pi / 2.0 - self._beta, n_pts)
            zc = -(self.t_mm - self._c) + self._R
            local = np.column_stack([rng.uniform(-L2, L2, n_pts),
                                     -self._R * np.sin(phis),
                                     zc - self._R * np.cos(phis)])
            # the groove is CONCAVE: the arc centre sits in the void, so the outward
            # (material -> void) normal points from the surface toward the centre
            n_loc = np.column_stack([np.zeros(n_pts), np.sin(phis), np.cos(phis)])
        else:
            corners = self._face_corners(name)
            o = corners[0]
            e1 = corners[1] - corners[0]
            e2 = corners[3] - corners[0]
            a = rng.random(n_pts)
            b = rng.random(n_pts)
            local = o + a[:, None] * e1 + b[:, None] * e2
            pl_n = self.face_plane(name).n     # world normal; convert to local
            n_loc = np.tile(self.T_world_part[:3, :3].T @ pl_n, (n_pts, 1))
        T = self.T_world_part
        return (local @ T[:3, :3].T + T[:3, 3],
                (n_loc @ T[:3, :3].T))

    # --- mesh (D21 / D34) -------------------------------------------------------------
    @property
    def max_chord_error_mm(self) -> float:
        if self.prep["kind"] != "single_U":
            return 0.0
        n = self._arc_n()
        d_ang = (np.pi / 2.0 - self._beta) / n
        return float(self._R * d_ang ** 2 / 8.0)

    def _arc_n(self) -> int:
        return max(12, int(np.ceil((np.pi / 2.0 - self._beta)
                                   / np.sqrt(8.0 * self._MESH_TOL_MM / self._R))))

    def _profile(self) -> np.ndarray:
        """The (v, w) profile polygon, CCW, material inside."""
        t, W, c = self.t_mm, self.width_mm, self._c
        zr = -(t - c)
        pts = [(0.0, -t), (0.0, zr)]                    # up the root face
        if self.prep["kind"] == "single_U":
            zc = zr + self._R
            for ph in np.linspace(0.0, np.pi / 2.0 - self._beta,
                                  self._arc_n() + 1)[1:]:
                pts.append((-self._R * np.sin(ph), zc - self._R * np.cos(ph)))
        pts.append((self.mouth_v_mm, 0.0))              # top of the fusion face
        pts.append((-W, 0.0))
        pts.append((-W, -t))
        return np.asarray(pts, dtype=float)

    def mesh(self) -> trimesh.Trimesh:
        prof = self._profile()
        k = len(prof)
        L2 = self.length_mm / 2.0
        v0 = np.column_stack([np.full(k, -L2), prof[:, 0], prof[:, 1]])
        v1 = np.column_stack([np.full(k, L2), prof[:, 0], prof[:, 1]])
        verts = np.vstack([v0, v1])
        faces = []
        for i in range(k):                              # side strips, closed profile
            j = (i + 1) % k
            faces.append([i, j, k + j])
            faces.append([i, k + j, k + i])
        # end caps: fan triangulation from the back-bottom corner works because the
        # profile is a monotone staircase from that corner (star-shaped about it)
        anchor = k - 1                                  # (-W, -t)
        for i in range(k - 2):
            a, b = i, i + 1
            if a == anchor or b == anchor:
                continue
            faces.append([anchor, b, a])
            faces.append([k + anchor, k + a, k + b])
        m = trimesh.Trimesh(vertices=verts, faces=np.asarray(faces), process=True)
        if m.volume < 0:
            m.invert()
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
