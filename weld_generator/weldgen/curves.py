"""Exact seam curves — Phase 6b, D33.

Every D29 seam is closed-form or constructed, so D1 stands unqualified:

> **Substituting a line into a quadric yields a quadratic.** Intersecting any *ruled*
> surface (plane, cylinder, cone) with any *quadric* is closed-form along the ruling.

Plane ∩ cylinder is a circle or ellipse (`ellipse_from_plane_cylinder`). Cylinder ∩
cylinder is the saddle curve, parametrized around the BRANCH cylinder: every branch
surface point is `P(φ, s) = c(φ) + s·â`, a line in `s`, so the main cylinder's implicit
form gives a quadratic in `s` per `φ` — one square root, exact, with unequal radii,
arbitrary branch angle and offset axes all covered (`saddle_from_cylinders`). The swept
configurations (D29 #6–#7) never intersect anything: their seam is CONSTRUCTED first
(`Arc3D`, `BSplineCurve`) and the parts are swept along it, the same D3 move that made
every straight seam exact.

Positions and tangents are analytic everywhere. The arclength PARAMETRIZATION of the
non-circular forms is numeric (a fine cumulative-chord table over the exact curve): the
resampled points still lie exactly on the curve — only their spacing is approximate —
so no label position ever carries solver error. That distinction is what lets D1 keep
its claim while `sample(density)` stays available at any density.

Conventions shared by every form:

  * `point(t)` / `tangent(t)` — vectorised over a parameter array; tangents are unit.
  * `closed` — True for full intersection curves; the parameter period is `t_period`.
  * `length_mm` — total arclength (closed curves: one full period), per the 6b schema
    ruling that `length_mm` is arclength and `closed` disambiguates.
  * `sample(density_per_mm)` — uniform-in-arclength polyline; closed curves do NOT
    repeat the wrap point (the consumer knows to close via the flag).
  * `to_parametric()` / `from_parametric(d)` — the seam block's exact-form JSON.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

#: Grid used for the numeric arclength table. Positions on the table are exact curve
#: points; the table only maps arclength -> parameter, so its error affects spacing,
#: never membership of the curve.
_ARCLEN_N = 8192


def _unit(v: np.ndarray) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        raise ValueError("zero-length direction")
    return v / n


def _frame(axis: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """A deterministic orthonormal pair spanning the plane normal to `axis`."""
    a = _unit(axis)
    tmp = np.array([0.0, 0.0, 1.0])
    if abs(float(a @ tmp)) > 0.9:
        tmp = np.array([1.0, 0.0, 0.0])
    u = _unit(np.cross(a, tmp))
    return u, np.cross(a, u)


class _CurveBase:
    """Shared sampling / arclength machinery over exact `point` / `tangent`."""

    closed: bool = False
    t_period: float = 1.0

    # --- exact, provided by each form ---------------------------------------------
    def point(self, t):                                   # pragma: no cover - abstract
        raise NotImplementedError

    def tangent(self, t):                                 # pragma: no cover - abstract
        raise NotImplementedError

    # --- numeric arclength table over the exact curve -----------------------------
    def _table(self) -> tuple[np.ndarray, np.ndarray]:
        if not hasattr(self, "_tab"):
            ts = np.linspace(0.0, self.t_period, _ARCLEN_N + 1)
            pts = self.point(ts)
            seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
            s = np.concatenate([[0.0], np.cumsum(seg)])
            self._tab = (ts, s)
        return self._tab

    @property
    def length_mm(self) -> float:
        return float(self._table()[1][-1])

    def t_at_arclength(self, s) -> np.ndarray:
        ts, tab = self._table()
        return np.interp(np.asarray(s, dtype=float), tab, ts)

    def sample(self, density_per_mm: float) -> np.ndarray:
        """Uniform-in-arclength polyline of exact curve points.

        Open curves include both endpoints (>= 2 points, matching `sample_polyline`);
        closed curves cover one period WITHOUT repeating the wrap point.
        """
        L = self.length_mm
        if self.closed:
            n = max(3, int(round(L * float(density_per_mm))))
            s = np.linspace(0.0, L, n, endpoint=False)
        else:
            n = max(2, int(round(L * float(density_per_mm))) + 1)
            s = np.linspace(0.0, L, n)
        return self.point(self.t_at_arclength(s))

    def arclengths(self, n: int) -> np.ndarray:
        L = self.length_mm
        return (np.linspace(0.0, L, n, endpoint=False) if self.closed
                else np.linspace(0.0, L, n))


@dataclass
class Ellipse3D(_CurveBase):
    """`P(t) = c + a·cos t·û + b·sin t·v̂` — plane ∩ cylinder, and the circle at a = b.

    `û`, `v̂` are orthonormal; `a >= b` by construction in the factory (û is the major
    direction). Closed, period 2π.
    """

    center: np.ndarray
    u_dir: np.ndarray
    v_dir: np.ndarray
    a_mm: float
    b_mm: float

    closed = True
    t_period = 2.0 * np.pi

    def point(self, t):
        t = np.atleast_1d(np.asarray(t, dtype=float))
        return (self.center[None, :]
                + self.a_mm * np.cos(t)[:, None] * self.u_dir[None, :]
                + self.b_mm * np.sin(t)[:, None] * self.v_dir[None, :])

    def tangent(self, t):
        t = np.atleast_1d(np.asarray(t, dtype=float))
        d = (-self.a_mm * np.sin(t)[:, None] * self.u_dir[None, :]
             + self.b_mm * np.cos(t)[:, None] * self.v_dir[None, :])
        return d / np.linalg.norm(d, axis=1, keepdims=True)

    @property
    def is_circle(self) -> bool:
        return abs(self.a_mm - self.b_mm) < 1e-9

    def to_parametric(self) -> dict:
        kind = "circle" if self.is_circle else "ellipse"
        out = {"kind": kind,
               "center_mm": [float(v) for v in self.center],
               "u_dir": [float(v) for v in self.u_dir],
               "v_dir": [float(v) for v in self.v_dir]}
        if kind == "circle":
            out["radius_mm"] = float(self.a_mm)
        else:
            out["a_mm"] = float(self.a_mm)
            out["b_mm"] = float(self.b_mm)
        return out


@dataclass
class Arc3D(_CurveBase):
    """Open circular arc `P(t) = c + r(cos t·û + sin t·v̂)`, `t ∈ [t0, t1]`.

    A constructed form (D29 #6–#7): the seam is drawn first and the parts swept along
    it, so the arc's parameters are inputs, never fit outputs.
    """

    center: np.ndarray
    u_dir: np.ndarray
    v_dir: np.ndarray
    radius_mm: float
    t0: float
    t1: float

    closed = False

    def __post_init__(self):
        if self.t1 <= self.t0:
            raise ValueError("arc needs t1 > t0")
        self.t_period = self.t1 - self.t0

    def point(self, t):
        t = np.atleast_1d(np.asarray(t, dtype=float)) + self.t0
        return (self.center[None, :]
                + self.radius_mm * (np.cos(t)[:, None] * self.u_dir[None, :]
                                    + np.sin(t)[:, None] * self.v_dir[None, :]))

    def tangent(self, t):
        t = np.atleast_1d(np.asarray(t, dtype=float)) + self.t0
        d = (-np.sin(t)[:, None] * self.u_dir[None, :]
             + np.cos(t)[:, None] * self.v_dir[None, :])
        return d / np.linalg.norm(d, axis=1, keepdims=True)

    @property
    def length_mm(self) -> float:                          # closed form beats the table
        return float(self.radius_mm * (self.t1 - self.t0))

    def t_at_arclength(self, s):
        return np.asarray(s, dtype=float) / self.radius_mm

    def to_parametric(self) -> dict:
        return {"kind": "arc",
                "center_mm": [float(v) for v in self.center],
                "u_dir": [float(v) for v in self.u_dir],
                "v_dir": [float(v) for v in self.v_dir],
                "radius_mm": float(self.radius_mm),
                "t0": float(self.t0), "t1": float(self.t1)}


@dataclass
class SaddleCurve(_CurveBase):
    """Pipe-to-pipe intersection, parametrized around the branch (D33).

    `P(φ) = c(φ) + s(φ)·â` with `c(φ) = o + r(cos φ·û + sin φ·v̂)` on the branch and
    `s(φ)` the root of the quadratic obtained by substituting that line into the main
    cylinder's implicit form. `root_sign = -1` takes the entering intersection for a
    set-on stub whose axis `â` points toward the main pipe.
    """

    branch_origin: np.ndarray
    branch_axis: np.ndarray                 # unit, pointing from stub body toward main
    u_dir: np.ndarray
    v_dir: np.ndarray
    branch_radius_mm: float
    main_point: np.ndarray
    main_axis: np.ndarray                   # unit
    main_radius_mm: float
    root_sign: float = -1.0

    closed = True
    t_period = 2.0 * np.pi

    def _quad(self, phi):
        phi = np.atleast_1d(np.asarray(phi, dtype=float))
        c = (self.branch_origin[None, :]
             + self.branch_radius_mm * (np.cos(phi)[:, None] * self.u_dir[None, :]
                                        + np.sin(phi)[:, None] * self.v_dir[None, :]))
        d = c - self.main_point[None, :]
        am = float(self.branch_axis @ self.main_axis)
        dm = d @ self.main_axis
        A = 1.0 - am * am
        B = 2.0 * (d @ self.branch_axis - dm * am)
        C = np.einsum("ij,ij->i", d, d) - dm * dm - self.main_radius_mm ** 2
        return c, A, B, C

    def s_of(self, phi):
        c, A, B, C = self._quad(phi)
        disc = B * B - 4.0 * A * C
        if np.any(disc <= 0.0):
            raise ValueError("branch does not fully intersect the main pipe")
        return c, (-B + self.root_sign * np.sqrt(disc)) / (2.0 * A)

    def point(self, t):
        c, s = self.s_of(t)
        return c + s[:, None] * self.branch_axis[None, :]

    def tangent(self, t):
        # Implicit differentiation: Q(s, phi) = A s^2 + B(phi) s + C(phi) = 0, so
        # s' = -(B' s + C') / (2 A s + B); P' = c'(phi) + s' a. Every term closed-form.
        phi = np.atleast_1d(np.asarray(t, dtype=float))
        c, A, B, C = self._quad(phi)
        disc = np.sqrt(B * B - 4.0 * A * C)
        s = (-B + self.root_sign * disc) / (2.0 * A)
        cp = self.branch_radius_mm * (-np.sin(phi)[:, None] * self.u_dir[None, :]
                                      + np.cos(phi)[:, None] * self.v_dir[None, :])
        am = float(self.branch_axis @ self.main_axis)
        dpm = cp @ self.main_axis
        Bp = 2.0 * (cp @ self.branch_axis - dpm * am)
        d = c - self.main_point[None, :]
        Cp = 2.0 * (np.einsum("ij,ij->i", cp, d) - dpm * (d @ self.main_axis))
        sp = -(Bp * s + Cp) / (2.0 * A * s + B)
        out = cp + sp[:, None] * self.branch_axis[None, :]
        return out / np.linalg.norm(out, axis=1, keepdims=True)

    def to_parametric(self) -> dict:
        return {"kind": "saddle",
                "branch_origin_mm": [float(v) for v in self.branch_origin],
                "branch_axis": [float(v) for v in self.branch_axis],
                "u_dir": [float(v) for v in self.u_dir],
                "v_dir": [float(v) for v in self.v_dir],
                "branch_radius_mm": float(self.branch_radius_mm),
                "main_point_mm": [float(v) for v in self.main_point],
                "main_axis": [float(v) for v in self.main_axis],
                "main_radius_mm": float(self.main_radius_mm),
                "root_sign": float(self.root_sign)}


@dataclass
class BSplineCurve(_CurveBase):
    """Clamped uniform B-spline over control points — CONSTRUCTED input only (D33).

    Appears exclusively as a drawn seam that parts are swept along (D29 #6–#7); it is
    never a fitted approximation of an intersection, which is what keeps spline error
    out of every label. Degree 3 unless there are too few control points.
    """

    control_mm: np.ndarray
    degree: int = 3

    closed = False
    t_period = 1.0

    def __post_init__(self):
        self.control_mm = np.asarray(self.control_mm, dtype=float)
        n = len(self.control_mm)
        if n < 2:
            raise ValueError("a spline needs at least 2 control points")
        self.degree = int(min(self.degree, n - 1))
        p, m = self.degree, n
        inner = np.linspace(0.0, 1.0, m - p + 1)
        self._knots = np.concatenate([np.zeros(p), inner, np.ones(p)])

    def _basis_eval(self, t):
        # de Boor, vectorised over query points; clamped so endpoints interpolate.
        t = np.clip(np.atleast_1d(np.asarray(t, dtype=float)), 0.0, 1.0 - 1e-12)
        p, kn, P = self.degree, self._knots, self.control_mm
        out = np.empty((len(t), P.shape[1]))
        for i, tv in enumerate(t):
            k = int(np.searchsorted(kn, tv, side="right") - 1)
            k = min(max(k, p), len(P) - 1)
            d = [P[j + k - p].astype(float) for j in range(p + 1)]
            for r in range(1, p + 1):
                for j in range(p, r - 1, -1):
                    denom = kn[j + 1 + k - r] - kn[j + k - p]
                    alpha = 0.0 if denom == 0.0 else (tv - kn[j + k - p]) / denom
                    d[j] = (1.0 - alpha) * d[j - 1] + alpha * d[j]
            out[i] = d[p]
        return out

    def point(self, t):
        return self._basis_eval(t)

    def tangent(self, t):
        t = np.atleast_1d(np.asarray(t, dtype=float))
        h = 1e-6
        d = (self._basis_eval(np.clip(t + h, 0, 1))
             - self._basis_eval(np.clip(t - h, 0, 1)))
        return d / np.linalg.norm(d, axis=1, keepdims=True)

    def to_parametric(self) -> dict:
        return {"kind": "bspline",
                "degree": int(self.degree),
                "control_mm": [[float(v) for v in row] for row in self.control_mm]}


@dataclass
class Segment3D(_CurveBase):
    """Straight segment parametrized BY ARCLENGTH (t in [0, L]) — composite member."""

    p0: np.ndarray
    p1: np.ndarray

    closed = False

    def __post_init__(self):
        self.p0 = np.asarray(self.p0, dtype=float)
        self.p1 = np.asarray(self.p1, dtype=float)
        self.t_period = float(np.linalg.norm(self.p1 - self.p0))
        if self.t_period < 1e-9:
            raise ValueError("degenerate segment")
        self._dir = (self.p1 - self.p0) / self.t_period

    def point(self, t):
        t = np.atleast_1d(np.asarray(t, dtype=float))
        return self.p0[None, :] + t[:, None] * self._dir[None, :]

    def tangent(self, t):
        t = np.atleast_1d(np.asarray(t, dtype=float))
        return np.tile(self._dir, (len(t), 1))

    @property
    def length_mm(self) -> float:
        return self.t_period

    def t_at_arclength(self, s):
        return np.asarray(s, dtype=float)

    def to_parametric(self) -> dict:
        return {"kind": "line",
                "p0_mm": [float(v) for v in self.p0],
                "p1_mm": [float(v) for v in self.p1]}


class CompositeCurve(_CurveBase):
    """Open curves laid end to end, parametrized by GLOBAL arclength (D29 #5).

    Each member must start where the previous one ends; `closed=True` additionally
    requires the last end to meet the first start. Every member is exact, so the
    composite is - a rounded rectangle is four segments and four quarter arcs, all
    closed-form, which is how the tube-on-plate seam keeps D1's claim."""

    def __init__(self, segments, closed: bool = False):
        if not segments:
            raise ValueError("empty composite")
        ends = [seg.point([seg.t_period])[0] for seg in segments]
        starts = [seg.point([0.0])[0] for seg in segments]
        for a, b in zip(ends[:-1], starts[1:]):
            if np.linalg.norm(a - b) > 1e-6:
                raise ValueError("composite members do not chain")
        if closed and np.linalg.norm(ends[-1] - starts[0]) > 1e-6:
            raise ValueError("closed composite does not wrap")
        self.segments = list(segments)
        self.closed = bool(closed)
        self._lens = np.array([seg.length_mm for seg in self.segments])
        self._cum = np.concatenate([[0.0], np.cumsum(self._lens)])
        self.t_period = float(self._cum[-1])

    @property
    def length_mm(self) -> float:
        return self.t_period

    def _delegate(self, t, method):
        t = np.atleast_1d(np.asarray(t, dtype=float))
        t = np.clip(t, 0.0, self.t_period)
        idx = np.clip(np.searchsorted(self._cum, t, side="right") - 1,
                      0, len(self.segments) - 1)
        out = np.empty((len(t), 3))
        for k in np.unique(idx):
            seg = self.segments[k]
            local = t[idx == k] - self._cum[k]
            frac = local / self._lens[k]
            out[idx == k] = getattr(seg, method)(
                seg.t_at_arclength(frac * seg.length_mm)
                if not isinstance(seg, (Segment3D,)) else local)
        return out

    def point(self, t):
        return self._delegate(t, "point")

    def tangent(self, t):
        return self._delegate(t, "tangent")

    def t_at_arclength(self, s):
        return np.asarray(s, dtype=float)                  # t IS arclength here

    def to_parametric(self) -> dict:
        return {"kind": "composite", "closed": bool(self.closed),
                "segments": [seg.to_parametric() for seg in self.segments]}


def rounded_rect_curve(center, x_dir, y_dir, w_mm: float, h_mm: float,
                       corner_r_mm: float) -> CompositeCurve:
    """The closed rounded-rectangle seam of a rectangular tube on a plate (D29 #5):
    four straight runs and four quarter arcs, CCW in the (x_dir, y_dir) plane."""
    c = np.asarray(center, dtype=float)
    x = _unit(x_dir)
    y = _unit(y_dir)
    hw, hh, r = w_mm / 2.0, h_mm / 2.0, float(corner_r_mm)
    if not (0.0 < r < min(hw, hh)):
        raise ValueError("corner radius must be positive and under half the sides")
    segs = []
    # CCW starting mid-bottom edge; quarter arcs sweep 90 deg at each corner.
    corners = [(+1, -1, -0.5 * np.pi), (+1, +1, 0.0),
               (-1, +1, 0.5 * np.pi), (-1, -1, np.pi)]
    pts_in = [c - (hw - r) * x - hh * y, c + (hw - r) * x - hh * y]
    for k, (sx, sy, a0) in enumerate(corners):
        segs.append(Segment3D(pts_in[0], pts_in[1]))
        cc = c + sx * (hw - r) * x + sy * (hh - r) * y
        segs.append(Arc3D(cc, x, y, r, a0, a0 + 0.5 * np.pi))
        nxt = corners[(k + 1) % 4]
        # start/end of the next straight run, rotating the frame by the corner order
        if k == 0:
            pts_in = [c + hw * x - (hh - r) * y, c + hw * x + (hh - r) * y]
        elif k == 1:
            pts_in = [c + (hw - r) * x + hh * y, c - (hw - r) * x + hh * y]
        elif k == 2:
            pts_in = [c - hw * x + (hh - r) * y, c - hw * x - (hh - r) * y]
        del nxt
    return CompositeCurve(segs, closed=True)


@dataclass
class OffsetCurve(_CurveBase):
    """The in-plane parallel of a spine at signed `offset_mm` along `n̂ = ẑ × T̂`.

    This is what makes a swept STIFFENER's welds exact: the drawn spine is the sweep
    path (mid-material — not a weld), and the two fillet seams are its offsets at
    ±t/2. A parallel curve's tangent is parallel to the spine's (for offsets inside
    the curvature radius, which the SweptSlab guard enforces), so positions AND
    tangents stay analytic.
    """

    spine: object
    offset_mm: float

    def __post_init__(self):
        self.closed = bool(self.spine.closed)
        self.t_period = float(self.spine.t_period)

    def _n2(self, t):
        tan = self.spine.tangent(t)
        return np.column_stack([-tan[:, 1], tan[:, 0], np.zeros(len(tan))])

    def point(self, t):
        t = np.atleast_1d(np.asarray(t, dtype=float))
        return self.spine.point(t) + self.offset_mm * self._n2(t)

    def tangent(self, t):
        return self.spine.tangent(t)

    def to_parametric(self) -> dict:
        return {"kind": "offset", "spine": self.spine.to_parametric(),
                "offset_mm": float(self.offset_mm)}


# ------------------------------------------------------------------ the factories


def ellipse_from_plane_cylinder(plane_point, plane_normal, cyl_point, cyl_axis,
                                radius_mm: float) -> Ellipse3D:
    """Exact plane ∩ cylinder: a circle when the axis is normal to the plane, an
    ellipse with semi-minor `r` and semi-major `r / cos θ` otherwise (θ between axis
    and plane normal). Raises when the axis lies in the plane (no bounded curve).

    Per the D29 curve-first amendment this factory belongs to the VERIFICATION side:
    generation constructs the curve directly and derives the cylinder from it; the D4
    arms use this to rediscover the seam from the placed surfaces."""
    n = _unit(plane_normal)
    a = _unit(cyl_axis)
    cos_t = float(a @ n)
    if abs(cos_t) < 1e-6:
        raise ValueError("cylinder axis parallel to the plane: unbounded intersection")
    q = np.asarray(plane_point, dtype=float)
    p0 = np.asarray(cyl_point, dtype=float)
    center = p0 + (float((q - p0) @ n) / cos_t) * a
    r = float(radius_mm)
    if abs(abs(cos_t) - 1.0) < 1e-9:
        u, v = _frame(n)
        return Ellipse3D(center, u, v, r, r)
    v = _unit(np.cross(n, a))                # in-plane, perpendicular to the axis
    u = _unit(np.cross(n, v))                # in-plane major direction
    return Ellipse3D(center, u, v, r / abs(cos_t), r)


def saddle_from_cylinders(branch_origin, branch_axis, branch_radius_mm: float,
                          main_point, main_axis, main_radius_mm: float,
                          root_sign: float = -1.0) -> SaddleCurve:
    """Exact cylinder ∩ cylinder around the branch, with a full-penetration guard.

    The guard walks the whole φ circle and requires a strictly positive discriminant:
    a grazing branch would split the curve, which D29 #4 excludes by construction
    (the sampler draws configurations where the stub lands fully on the main pipe)."""
    a = _unit(branch_axis)
    m = _unit(main_axis)
    if abs(float(a @ m)) > 1.0 - 1e-6:
        raise ValueError("branch parallel to main: not a pipe joint")
    u, v = _frame(a)
    curve = SaddleCurve(np.asarray(branch_origin, dtype=float), a, u, v,
                        float(branch_radius_mm),
                        np.asarray(main_point, dtype=float), m,
                        float(main_radius_mm), float(root_sign))
    curve.s_of(np.linspace(0.0, 2.0 * np.pi, 720, endpoint=False))   # raises if grazing
    return curve


def from_parametric(d: dict):
    """Rebuild a curve from its seam-block JSON (the consumer-side inverse)."""
    k = d["kind"]
    if k == "circle":
        return Ellipse3D(np.asarray(d["center_mm"], float),
                         np.asarray(d["u_dir"], float), np.asarray(d["v_dir"], float),
                         float(d["radius_mm"]), float(d["radius_mm"]))
    if k == "ellipse":
        return Ellipse3D(np.asarray(d["center_mm"], float),
                         np.asarray(d["u_dir"], float), np.asarray(d["v_dir"], float),
                         float(d["a_mm"]), float(d["b_mm"]))
    if k == "arc":
        return Arc3D(np.asarray(d["center_mm"], float),
                     np.asarray(d["u_dir"], float), np.asarray(d["v_dir"], float),
                     float(d["radius_mm"]), float(d["t0"]), float(d["t1"]))
    if k == "saddle":
        return SaddleCurve(np.asarray(d["branch_origin_mm"], float),
                           np.asarray(d["branch_axis"], float),
                           np.asarray(d["u_dir"], float),
                           np.asarray(d["v_dir"], float),
                           float(d["branch_radius_mm"]),
                           np.asarray(d["main_point_mm"], float),
                           np.asarray(d["main_axis"], float),
                           float(d["main_radius_mm"]), float(d["root_sign"]))
    if k == "bspline":
        return BSplineCurve(np.asarray(d["control_mm"], float), int(d["degree"]))
    if k == "line":
        return Segment3D(np.asarray(d["p0_mm"], float), np.asarray(d["p1_mm"], float))
    if k == "composite":
        return CompositeCurve([from_parametric(seg) for seg in d["segments"]],
                              closed=bool(d.get("closed", False)))
    if k == "offset":
        return OffsetCurve(from_parametric(d["spine"]), float(d["offset_mm"]))
    raise ValueError(f"unknown parametric kind {k!r}")
