"""D4 / D13 — the accessibility rule, as an independent verification function.

    A candidate seam is the clipped intersection of an **exterior** face pair whose
    dihedral bisector escapes to free space without re-entering material, and whose
    faces both belong to objects with `role == "workpiece"`.

This module knows nothing about how a joint was constructed. It enumerates every face
pair in the scene and decides each one on geometry alone — which is what makes it a
*verification* of the constructors rather than a restatement of them. Phase 4's baselines
reuse it.

**Three** mechanisms produce candidates. D4 originally named only the first; the third was
added 2026-08-16 after enumeration showed the rule structurally could not produce an
edge-joint seam.

  * **Intersecting pair** — non-parallel faces. The seam is the plane intersection,
    clipped to where both faces have support. Fillets and toes come from here.
  * **Facing pair** — antiparallel faces separated by a gap (the mid-lap interface).
    These have *no* intersection line, and their bisector `n_A + n_B` is the zero vector,
    so no torch direction exists at all. Emitted and rejected `bisector_blocked` — free
    hard negatives documenting exactly what a nearest-point rule would wrongly return.
  * **Coplanar exposed pair** — faces sharing a plane and a normal, separated by a gap
    *within* that plane. The seam is the gap's centreline. This is the edge joint and the
    butt centreline, and it is what makes PARAMETERS.md §2.7 literally true in code:
    a lap at `stack_offset > 0` buries its interface (facing pair, rejected), while at
    `stack_offset = 0` the same interface surfaces as a coplanar pair and becomes weldable.

Where a coplanar centreline exists, the intersecting candidates that bound its gap are the
weld's **toes**, not separate seams; they are rejected `toe_of_centreline` and kept as
negatives.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable, Sequence

import numpy as np

from .geom import SLAB_FACES, Plane, Slab, dihedral_deg, intersect_planes

#: Default accessibility parameters. Mirrors configs/*.yaml and scene.json's
#: `accessibility` block, which is stored so every rejection is reproducible.
DEFAULT_ACCESS: dict[str, Any] = {
    "torch_clearance": {"half_angle_deg": 30.0, "standoff_mm": 15.0},
    "dihedral_min_deg": 30.0,
    "dihedral_max_deg": 170.0,
    # MUST exceed the largest root gap the sampler can draw, including the below_D
    # over-range tail. Its job is to reject face pairs that are nowhere near each other
    # (opposite ends of a plate, tens of mm apart), NOT to police fit-up. Set below the
    # max gap it silently strips the seams from exactly the out-of-tolerance scenes the
    # dataset generates on purpose - `configs.assert_accessibility_covers_gap` checks it.
    "contact_tol_mm": 4.0,
    #: "Are these the SAME plane?" - a different question from "are these faces adjacent?"
    #: and it needs a far tighter tolerance. Reusing contact_tol here made two faces 8 mm
    #: apart register as coplanar, inventing a centreline that suppressed the real welds.
    "coplanar_tol_mm": 0.5,
    "min_seam_length_mm": 10.0,
    #: A seam must also be a meaningful FRACTION of the joint, not merely above an
    #: absolute floor. Without this a 200 mm lap emits its two 40 mm end runs alongside
    #: the two 200 mm toes, and the short cross-runs - real geometry, but not what anyone
    #: means by "the seam" - outnumber the long ones. Scale-free, so it behaves the same
    #: on an 80 mm coupon and a 400 mm plate.
    "min_seam_length_frac": 0.25,
}

#: Faces count as parallel within this angle. NOT exact parallelism: the sampler tilts
#: B by the angular misalignment beta (up to 4 deg, PARAMETERS.md §2.3), so an exact test
#: means butt and edge joints only ever find their centreline when beta happens to be 0.
#: That is how a real bug hid behind a test suite that only ever used beta = 0.
#: Which seam classes each joint type legitimately produces. The class follows from the
#: faces involved (see `seam_class`), and it is what makes the joint-type label honest:
#:
#:   edge joint - BOTH parts contribute an edge. A weld running along the surface of one
#:                of them is not an edge weld, it is a lap toe.
#:   lap joint  - exactly the opposite: the edge of one part against the FACE of the other.
#:   butt joint - the weld lies on the coplanar faces; the short runs across the plate
#:                thickness at the ends are not seams.
#:
#: A corner joint carries two: the inside fillet and the outside edge-to-edge weld.
ALLOWED_CLASSES: dict[str, frozenset[str]] = {
    "T":      frozenset({"fillet"}),
    "corner": frozenset({"fillet", "edge"}),
    "butt":   frozenset({"butt"}),
    "lap":    frozenset({"lap_toe"}),
    "edge":   frozenset({"edge"}),
}

#: Slab faces that are BROAD (the two large faces); everything else is an edge face.
BROAD_FACES = frozenset({"+w", "-w"})

PARALLEL_TOL_DEG = 8.0
PARALLEL_TOL = float(np.sin(np.deg2rad(PARALLEL_TOL_DEG)))


@dataclass
class Candidate:
    """One enumerated face pair and the verdict on it."""

    face_pair: tuple[str, str]
    weldable: bool
    reject_reason: str | None
    p0: np.ndarray | None
    p1: np.ndarray | None
    n_a: np.ndarray
    n_b: np.ndarray
    dihedral_deg: float
    separation_mm: float

    @property
    def seam_class(self) -> str:
        """What KIND of weld this is, from the faces that form it.

        Derived, never declared - the same discipline as `quality_level`. It is what
        stops a scene labelled `edge` from reporting a lap toe as an edge seam.
        """
        a, b = (f.split(":")[1] for f in self.face_pair)
        broad = (a in BROAD_FACES) + (b in BROAD_FACES)
        if broad == 2:
            return "butt" if self.dihedral_deg > 180.0 - PARALLEL_TOL_DEG else "fillet"
        if broad == 0:
            return "edge"
        return "lap_toe"

    @property
    def length_mm(self) -> float:
        if self.p0 is None:
            return 0.0
        return float(np.linalg.norm(self.p1 - self.p0))


def _cone_directions(axis: np.ndarray, half_angle_deg: float, n: int = 8) -> np.ndarray:
    """Rays sampling a cone about `axis` — a real nozzle has width, not zero width.

    At 90 degrees the bisector test is nearly free; at 60 degrees it is not, which is the
    point of modelling the torch as a cone rather than a single ray.
    """
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    if half_angle_deg <= 0.0 or n <= 0:
        return axis[None, :]
    tmp = np.array([0.0, 0.0, 1.0])
    if abs(axis @ tmp) > 0.9:
        tmp = np.array([1.0, 0.0, 0.0])
    u = np.cross(axis, tmp); u /= np.linalg.norm(u)
    v = np.cross(axis, u)
    a = np.deg2rad(half_angle_deg)
    phis = np.linspace(0.0, 2 * np.pi, n, endpoint=False)
    rim = (np.cos(a) * axis[None, :]
           + np.sin(a) * (np.cos(phis)[:, None] * u[None, :]
                          + np.sin(phis)[:, None] * v[None, :]))
    return np.vstack([axis[None, :], rim])


def _escapes(solids: Sequence[Slab], origin: np.ndarray, axis: np.ndarray,
             access: dict[str, Any]) -> bool:
    """True if a torch cone at `origin` along `axis` clears every solid."""
    clear = access["torch_clearance"]
    dirs = _cone_directions(axis, clear["half_angle_deg"])
    standoff = float(clear["standoff_mm"])
    # Step just off the surface so the origin itself is never counted as inside.
    starts = origin[None, :] + dirs * 1e-3
    probes = np.vstack([starts + dirs * (standoff * f) for f in (0.25, 0.5, 1.0)])
    for s in solids:
        if s.contains(probes).any():
            return False
    return True


def enumerate_candidates(
    parts: Sequence[Slab],
    access: dict[str, Any] | None = None,
    samples_along: int = 5,
    joint_type: str | None = None,
) -> list[Candidate]:
    """Every inter-object face pair, decided on geometry alone (D4 + D13).

    Returns candidates sorted by the documented seam ordering (SCHEMA.md §2.7): weldable
    first, then by canonical face-pair string, then by start point.
    """
    access = {**DEFAULT_ACCESS, **(access or {})}
    solids = list(parts)
    # Characteristic size of the assembly: the longest edge of any workpiece. The fixture
    # is excluded - it is far larger than the joint and would swamp the fraction.
    characteristic = max((max(p.dims_mm) for p in parts if p.role == "workpiece"),
                         default=0.0)
    access = dict(access)
    access["_min_len"] = max(float(access["min_seam_length_mm"]),
                             float(access.get("min_seam_length_frac", 0.0)) * characteristic)
    out: list[Candidate] = []

    for i, A in enumerate(parts):
        for j, B in enumerate(parts):
            if j <= i:
                continue
            for fa in SLAB_FACES:
                for fb in SLAB_FACES:
                    ref = tuple(sorted((f"{A.id}:{fa}", f"{B.id}:{fb}")))
                    c = _judge(A, fa, B, fb, ref, parts, solids, access, samples_along)
                    if c is not None:
                        out.append(c)

    _suppress_toes(out, access)

    # A seam whose class does not belong to this joint type is real geometry but is not
    # this joint's weld. Kept as a negative rather than dropped: it is exactly what a
    # geometric method returns when it does not know what kind of joint it is looking at.
    if joint_type is not None:
        allowed = ALLOWED_CLASSES.get(joint_type)
        if allowed is not None:
            for c in out:
                if c.weldable and c.seam_class not in allowed:
                    c.weldable = False
                    c.reject_reason = "wrong_class_for_joint"

    out.sort(key=lambda c: (
        not c.weldable, c.face_pair[0], c.face_pair[1],
        tuple(np.round(c.p0, 6)) if c.p0 is not None else (0.0, 0.0, 0.0),
    ))
    return out


def _judge(A: Slab, fa: str, B: Slab, fb: str, ref: tuple[str, str],
           parts: Sequence[Slab], solids: Sequence[Slab],
           access: dict[str, Any], samples_along: int) -> Candidate | None:
    pa, pb = A.face_plane(fa), B.face_plane(fb)
    na, nb = pa.n, pb.n
    cross = float(np.linalg.norm(np.cross(na, nb)))

    # --- D13: the semantic precondition, checked before any geometry ----------------
    # A plate standing on the fixture has two exterior faces, a clean 90 degree dihedral
    # and an escaping bisector. It passes D4 in full and is NOT weldable. The rejection
    # is not derivable from geometry, which is exactly why it is interesting.
    fixture_involved = A.role == "fixture" or B.role == "fixture"

    if cross < PARALLEL_TOL:
        if float(na @ nb) > 0.5:
            # --- coplanar exposed pair: the gap centreline is the seam ---------------
            return _coplanar_candidate(A, fa, B, fb, ref, solids, access,
                                       fixture_involved, samples_along)
        # --- facing pair: no intersection line exists at all ------------------------
        sep = abs(float(pa.signed_distance(B.face_center(fb))))
        if sep > access["contact_tol_mm"]:
            return None                      # too far apart to be a joint at all
        if not _faces_overlap(A, fa, B, fb):
            return None
        return Candidate(
            face_pair=ref, weldable=False,
            reject_reason="fixture_contact" if fixture_involved else "bisector_blocked",
            p0=None, p1=None, n_a=na, n_b=nb,
            dihedral_deg=180.0, separation_mm=sep,
        )

    # --- intersecting pair -----------------------------------------------------------
    hit = intersect_planes(pa, pb)
    if hit is None:                                            # pragma: no cover
        return None
    point, direction = hit
    a_lo, a_hi = A.face_extent_along(fa, direction)
    b_lo, b_hi = B.face_extent_along(fb, direction)
    lo, hi = max(a_lo, b_lo), min(a_hi, b_hi)
    if hi - lo <= 1e-9:
        return None                          # the faces do not overlap along the line
    s0 = float(point @ direction)
    p0 = point + (lo - s0) * direction
    p1 = point + (hi - s0) * direction

    # Are the two faces actually near each other along this line? Two faces on opposite
    # ends of a big plate intersect in a mathematically valid line that is nowhere near
    # either of them.
    sep = _line_separation(A, fa, B, fb, p0, p1, samples_along)

    # ...and is the gap between them OPEN, or is there material in the way?
    #
    # On thin sheet the plate thickness can fall below `contact_tol_mm`, so the far side
    # of a plate sits "close enough" to the other part: on 1.8 mm plate with a 2.0 mm
    # tolerance, A's underside is 1.9 mm from B, and six phantom seams appear wrapped
    # around the wrong surface. Tightening the tolerance is not the fix - it has to stay
    # above the root gap. The fix is to ask whether the two faces can SEE each other: the
    # straight path from the seam line to the nearest point on each face must not pass
    # through a solid. For a real fillet that path is the root gap and crosses nothing;
    # for the wrap-around it drives straight through the plate.
    #
    # This is the same bound radius-PCA lives under - a neighbourhood must not bridge a
    # plate's own two faces - arrived at from the opposite direction.
    if not _mutually_visible(A, fa, B, fb, p0, p1, solids, samples_along):
        return None

    theta = dihedral_deg(na, nb)
    bis = na + nb
    bis_norm = float(np.linalg.norm(bis))

    # Pairs that are nowhere near each other are not candidates at all - they are
    # dropped, not recorded. Two faces at opposite ends of a plate intersect in a
    # mathematically valid line that is 200 mm from either of them; emitting those as
    # `no_contact` rejections buried the meaningful negatives under ~25 rows per scene.
    # This is also what keeps D13 honest: `fixture_contact` must mean the workpiece is
    # actually resting on the fixture, not merely in the same scene as it.
    if sep > access["contact_tol_mm"]:
        return None

    reason: str | None = None
    if fixture_involved:
        reason = "fixture_contact"
    elif not (access["dihedral_min_deg"] <= theta <= access["dihedral_max_deg"]):
        reason = "degenerate_dihedral"
    elif hi - lo < access["_min_len"]:
        reason = "too_short"
    elif bis_norm < 1e-9:
        reason = "bisector_blocked"
    else:
        axis = bis / bis_norm
        mids = np.linspace(0.0, 1.0, samples_along)[:, None]
        pts = p0[None, :] * (1 - mids) + p1[None, :] * mids
        if not all(_escapes(solids, q, axis, access) for q in pts):
            reason = "bisector_blocked"

    return Candidate(
        face_pair=ref, weldable=reason is None, reject_reason=reason,
        p0=p0, p1=p1, n_a=na, n_b=nb,
        dihedral_deg=theta, separation_mm=sep,
    )


def d19_curves(A: Slab, fa: str, B: Slab, fb: str,
               p0: np.ndarray, p1: np.ndarray, n_samples: int = 2
               ) -> tuple[np.ndarray, np.ndarray]:
    """The `root` and `gap_mid` curves for any candidate — SCHEMA.md §1.3.

    Defined from closest points rather than from a joint-specific construction, so one
    definition covers fillets, toes and centrelines:

        a(q) = closest point on face patch A to q      } q on the nominal curve
        b(q) = closest point on face patch B to q      }

        gap_mid(q) = midpoint of a and b               - symmetric in A<->B
        root(q)    = whichever of a, b lies FARTHER from the nominal curve, i.e. the
                     terminating edge, the deeper side of the gap

    On a 90 degree T-fillet this reduces to the Phase 1 result exactly: a is the nominal
    point itself (distance 0), b is B's bottom edge (distance g), so root sits at g and
    gap_mid at g/2.
    """
    ts = np.linspace(0.0, 1.0, max(2, n_samples))[:, None]
    pts = np.asarray(p0)[None, :] * (1 - ts) + np.asarray(p1)[None, :] * ts
    a = _closest_on_face(A, fa, pts)
    b = _closest_on_face(B, fb, pts)
    gap_mid = 0.5 * (a + b)
    da = np.linalg.norm(a - pts, axis=1)
    db = np.linalg.norm(b - pts, axis=1)
    root = np.where((db >= da)[:, None], b, a)
    return root, gap_mid


def _closest_on_face(part: Slab, face: str, pts: np.ndarray) -> np.ndarray:
    """Closest point on the finite rectangular face patch (not its infinite plane)."""
    half = np.asarray(part.dims_mm, dtype=float) / 2.0
    axis = "uvw".index(face[1])
    sign = 1.0 if face[0] == "+" else -1.0
    T = part.T_world_part
    R, t = T[:3, :3], T[:3, 3]
    local = (np.asarray(pts) - t) @ R
    target = local.copy()
    target[:, axis] = sign * half[axis]
    for k in range(3):
        if k != axis:
            target[:, k] = np.clip(target[:, k], -half[k], half[k])
    return target @ R.T + t


def _plane_basis(n: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    tmp = np.array([0.0, 0.0, 1.0])
    if abs(float(n @ tmp)) > 0.9:
        tmp = np.array([1.0, 0.0, 0.0])
    u = np.cross(n, tmp); u /= np.linalg.norm(u)
    return u, np.cross(n, u)


def _coplanar_candidate(A: Slab, fa: str, B: Slab, fb: str, ref: tuple[str, str],
                        solids: Sequence[Slab], access: dict[str, Any],
                        fixture_involved: bool, samples_along: int) -> Candidate | None:
    """Two faces sharing a plane and a normal, separated by a gap within that plane.

    The seam is the centreline of that in-plane gap: an edge joint's weld, and a butt
    joint's weld. Both are welds that D4's intersection arm cannot express, because
    coplanar planes do not intersect in a line.
    """
    pa, pb = A.face_plane(fa), B.face_plane(fb)

    # Use the MEAN normal, and measure the perpendicular offset between the two face
    # centres. Comparing plane `d` values only works when the faces are exactly parallel;
    # under a 2 deg tilt a 180 mm face deviates by 6 mm along its length and the equality
    # test fails even though the joint is perfectly real.
    n = pa.n + pb.n
    n = n / np.linalg.norm(n)
    if abs(float((B.face_center(fb) - A.face_center(fa)) @ n)) > \
            access.get("coplanar_tol_mm", 0.5) + access["contact_tol_mm"]:
        return None                                    # parallel, but not the same plane

    # The in-plane basis comes from part A's OWN axes. Two earlier attempts were wrong:
    #   * a world-aligned basis found the gap only when the assembly happened to be
    #     axis-aligned - rotate the scene and the seam vanished;
    #   * the A->B centre offset gives a diagonal when the faces are offset along BOTH
    #     in-plane axes (a lap's plate ends differ in y AND z), and no extent is then
    #     cleanly disjoint, so the centreline was silently missed and both of its toes
    #     survived as separate seams.
    # A's own axes rotate with A, so they are pose-invariant and stay aligned with the
    # rectangular faces whose extents we are comparing.
    R = A.T_world_part[:3, :3]
    face_axis = "uvw".index(fa[1])
    in_plane = [R[:, k] for k in range(3) if k != face_axis]

    best = None
    for axis in in_plane:
        (a0, a1) = A.face_extent_along(fa, axis)
        (b0, b1) = B.face_extent_along(fb, axis)
        if b0 > a1:
            gap, mid = b0 - a1, 0.5 * (a1 + b0)
        elif a0 > b1:
            gap, mid = a0 - b1, 0.5 * (b1 + a0)
        else:
            continue                                   # the faces overlap on this axis
        if gap > access["contact_tol_mm"]:
            continue                                   # too far apart to be a joint
        other = np.cross(n, axis)
        (c0, c1) = A.face_extent_along(fa, other)
        (d0, d1) = B.face_extent_along(fb, other)
        lo, hi = max(c0, d0), min(c1, d1)
        if hi - lo <= 1e-9:
            continue                                   # no shared run along the seam
        if best is None or (hi - lo) > best[4] - best[3]:
            best = (axis, other, gap, mid, lo, hi)

    if best is None:
        return None
    axis, other, gap, mid, lo, hi = best
    origin = -pa.d * n                                 # a point on the shared plane
    base = origin + (mid - float(origin @ axis)) * axis
    p0 = base + (lo - float(base @ other)) * other
    p1 = base + (hi - float(base @ other)) * other


    reason: str | None = None
    if fixture_involved:
        reason = "fixture_contact"
    elif hi - lo < access["_min_len"]:
        reason = "too_short"
    else:
        # Bisector of two identical normals is the normal itself: straight out of
        # the shared plane. It escapes iff the gap is genuinely exposed rather than
        # buried under the other part.
        ts = np.linspace(0.0, 1.0, samples_along)[:, None]
        pts = p0[None, :] * (1 - ts) + p1[None, :] * ts
        if not all(_escapes(solids, q, n, access) for q in pts):
            reason = "bisector_blocked"

    return Candidate(
        face_pair=ref, weldable=reason is None, reject_reason=reason,
        p0=p0, p1=p1, n_a=n, n_b=n,
        dihedral_deg=180.0, separation_mm=gap,
    )


def _suppress_toes(cands: list[Candidate], access: dict[str, Any]) -> None:
    """Where a coplanar centreline exists, the lines bounding its gap are its toes.

    A butt weld is one weld with two toes, not three seams. The toes stay in the file as
    `weldable: false` negatives — they are exactly what a plane-pair baseline returns, so
    keeping them makes the weldable-vs-interior metric measurable.
    """
    centrelines = [c for c in cands
                   if c.weldable and c.p0 is not None and c.dihedral_deg == 180.0]
    for centre in centrelines:
        axis = centre.p1 - centre.p0
        axis = axis / np.linalg.norm(axis)
        # A toe bounds the gap, so it lies within about one gap width of the centreline.
        # Using the adjacency tolerance here swept up the far face's centreline too, and
        # since every centreline suppressed every other, all of them ended up rejected.
        reach = 1.5 * centre.separation_mm + 1e-6
        for c in cands:
            if c is centre or not c.weldable or c.p0 is None:
                continue
            if c.dihedral_deg == 180.0:
                continue                               # another centreline, not a toe
            d = c.p1 - c.p0
            if np.linalg.norm(np.cross(d / np.linalg.norm(d), axis)) > 1e-6:
                continue                               # not parallel to the centreline
            if float(np.linalg.norm(np.cross(c.p0 - centre.p0, axis))) <= reach:
                c.weldable = False
                c.reject_reason = "toe_of_centreline"


def _faces_overlap(A: Slab, fa: str, B: Slab, fb: str) -> bool:
    """Do two parallel faces actually shadow one another?"""
    n = A.face_normal(fa)
    tmp = np.array([0.0, 0.0, 1.0])
    if abs(n @ tmp) > 0.9:
        tmp = np.array([1.0, 0.0, 0.0])
    u = np.cross(n, tmp); u /= np.linalg.norm(u)
    v = np.cross(n, u)
    for axis in (u, v):
        a_lo, a_hi = A.face_extent_along(fa, axis)
        b_lo, b_hi = B.face_extent_along(fb, axis)
        if min(a_hi, b_hi) - max(a_lo, b_lo) <= 0.0:
            return False
    return True


def _mutually_visible(A: Slab, fa: str, B: Slab, fb: str,
                      p0: np.ndarray, p1: np.ndarray,
                      solids: Sequence[Slab], n: int) -> bool:
    """Can the two faces reach each other without passing through material?"""
    ts = np.linspace(0.15, 0.85, max(2, n))[:, None]
    pts = p0[None, :] * (1 - ts) + p1[None, :] * ts
    for part, face in ((A, fa), (B, fb)):
        target = _closest_on_face(part, face, pts)
        # Sample strictly between the line and the face, excluding both endpoints so a
        # point lying exactly on a surface is never counted as inside it.
        for f in (0.25, 0.5, 0.75):
            probe = pts + (target - pts) * f
            for s in solids:
                if s.contains(probe, tol=-1e-6).any():
                    return False
    return True


def _line_separation(A: Slab, fa: str, B: Slab, fb: str,
                     p0: np.ndarray, p1: np.ndarray, n: int) -> float:
    """Largest distance from the candidate line to the nearer of the two real faces."""
    ts = np.linspace(0.0, 1.0, n)[:, None]
    pts = p0[None, :] * (1 - ts) + p1[None, :] * ts
    worst = 0.0
    for part, face in ((A, fa), (B, fb)):
        d = _distance_to_face_patch(part, face, pts)
        worst = max(worst, float(d.max()))
    return worst


def _distance_to_face_patch(part: Slab, face: str, pts: np.ndarray) -> np.ndarray:
    """Distance from points to the finite rectangular face patch (not its plane)."""
    half = np.asarray(part.dims_mm, dtype=float) / 2.0
    axis = "uvw".index(face[1])
    sign = 1.0 if face[0] == "+" else -1.0
    T = part.T_world_part
    R, t = T[:3, :3], T[:3, 3]
    local = (np.asarray(pts) - t) @ R          # world -> part local
    target = local.copy()
    target[:, axis] = sign * half[axis]
    for k in range(3):
        if k != axis:
            target[:, k] = np.clip(target[:, k], -half[k], half[k])
    return np.linalg.norm(local - target, axis=1)
