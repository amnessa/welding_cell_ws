"""Table plane from pen touches: the touch pattern and the plane fit. numpy only.

`scripts/table_touchoff.py` drives the robot; this module is the geometry it needs, so
it can be tested without the robot.

    pts  = touch_pattern(centre_xy, edge_m=0.10, sides=6)          # (7, 2) in base_link
    fit  = fit_plane(touch_points_base)                              # dict, see below

Why more than three points: three points define a plane exactly - the residual is zero
by construction and says nothing about how good the touches were. The default is a
hexagon with 10 cm edges plus its centre, seven touches: four spare points for the fit,
so its RMSE is a real number (flatness of the table plus the repeatability of the
touches), and with `repeats` > 1 the scatter of repeated touches at one spot separates
the two.
"""

from __future__ import annotations

from typing import Any

import numpy as np


def touch_pattern(centre_xy, edge_m: float = 0.10, with_centre: bool = True,
                  yaw_rad: float = 0.0, sides: int = 6) -> np.ndarray:
    """Vertices of a regular polygon with `sides` sides of length `edge_m`, centred on
    `centre_xy`, then (optionally) the centre. (N, 2) in the plane of the table.

    Circumradius = edge / (2 sin(pi/sides)): for the default hexagon it equals the edge
    (10 cm), for a triangle edge/sqrt(3).
    """
    if sides < 3:
        raise ValueError("a polygon needs at least 3 sides")
    c = np.asarray(centre_xy, float).reshape(2)
    r = edge_m / (2.0 * np.sin(np.pi / sides))
    angs = yaw_rad + np.pi / 2 + 2.0 * np.pi * np.arange(sides) / sides
    pts = [c + r * np.array([np.cos(a), np.sin(a)]) for a in angs]
    if with_centre:
        pts.append(c.copy())
    return np.array(pts)


def fit_plane(points: np.ndarray) -> dict[str, Any]:
    """Least-squares plane z = a x + b y + c through the touch points (base_link, m).

    Returns the coefficients, the unit normal (pointing up), the tilt from horizontal,
    the height at the points' centroid (THE table height to use), the height at the
    base_link origin, the per-point residuals and their RMSE (mm). With exactly three
    points the RMSE is 0 and `rmse_meaningful` is False.
    """
    P = np.asarray(points, float).reshape(-1, 3)
    if len(P) < 3:
        raise ValueError("need at least 3 touch points")
    A = np.column_stack([P[:, 0], P[:, 1], np.ones(len(P))])
    (a, b, c), *_ = np.linalg.lstsq(A, P[:, 2], rcond=None)
    res = P[:, 2] - A @ np.array([a, b, c])
    n = np.array([-a, -b, 1.0]); n /= np.linalg.norm(n)
    centroid = P.mean(axis=0)
    return {
        "a": float(a), "b": float(b), "c": float(c),
        "normal": n.tolist(),
        "tilt_deg": float(np.degrees(np.arccos(np.clip(n[2], -1.0, 1.0)))),
        "z_at_centroid_m": float(a * centroid[0] + b * centroid[1] + c),
        "z_at_origin_m": float(c),
        "centroid_m": centroid.tolist(),
        "residuals_mm": (res * 1000.0).tolist(),
        "rmse_mm": float(np.sqrt(np.mean(res ** 2)) * 1000.0),
        "z_span_mm": float((P[:, 2].max() - P[:, 2].min()) * 1000.0),
        "n_points": int(len(P)),
        "rmse_meaningful": bool(len(P) > 3),
    }


def repeatability(groups: list[np.ndarray]) -> dict[str, Any]:
    """Scatter of repeated touches at the same spot: per-spot z std (mm) and the pooled
    value. Separates the probe's repeatability from the table's flatness."""
    stds = [float(np.std(np.asarray(g, float)[:, 2]) * 1000.0) for g in groups if len(g) > 1]
    return {"per_spot_z_std_mm": stds,
            "pooled_z_std_mm": float(np.sqrt(np.mean(np.square(stds)))) if stds else None}


# --------------------------------------------------------------------------- TCP check
def rot_log(R: np.ndarray) -> np.ndarray:
    c = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    th = np.arccos(c)
    if th < 1e-9:
        return np.zeros(3)
    if np.pi - th < 1e-6:                               # 180 deg: axis from the diagonal
        k = int(np.argmax(np.diag(R)))
        v = R[:, k] + np.eye(3)[k]
        return th * v / np.linalg.norm(v)
    return th / (2 * np.sin(th)) * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


def rot_exp(w: np.ndarray) -> np.ndarray:
    th = float(np.linalg.norm(w))
    if th < 1e-12:
        return np.eye(3)
    k = np.asarray(w, float) / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


def slerp_R(R0: np.ndarray, R1: np.ndarray, s: float) -> np.ndarray:
    """Rotation a fraction `s` of the way from R0 to R1 (geodesic)."""
    return R0 @ rot_exp(s * rot_log(R0.T @ R1))


def check_orientations(tilts_deg=(0.0, 30.0), azimuths_deg=(0.0, 90.0, 180.0, 270.0),
                       x0=np.array([1.0, 0.0, 0.0])) -> list[tuple[float, float, np.ndarray, np.ndarray]]:
    """(tilt, azimuth, pen axis INTO the surface, tool x hint) for the TCP check.

    Tilt 0 at every azimuth is the pure ROLL about a vertical pen (the classic roll
    test: on paper, the dots of a correct TCP coincide). Tilt > 0 swings the pen off
    vertical toward each azimuth with the tool's yaw held FIXED: that is what makes the
    plane's normal look different in the pen's own frame at each azimuth, and so what
    makes the lateral TCP components observable from contact heights. (Rolling the
    tool together with the tilt direction keeps the normal constant in the pen's frame
    - one lateral direction, an ill-conditioned fit; the first draft did that.)
    """
    out = []
    x0 = np.asarray(x0, float)
    for t in tilts_deg:
        for a in azimuths_deg:
            tr, ar = np.deg2rad(t), np.deg2rad(a)
            axis = np.array([np.sin(tr) * np.cos(ar), np.sin(tr) * np.sin(ar), -np.cos(tr)])
            if t == 0.0:
                Rz = np.array([[np.cos(ar), -np.sin(ar), 0], [np.sin(ar), np.cos(ar), 0], [0, 0, 1.0]])
                out.append((float(t), float(a), axis, Rz @ x0))
            else:
                out.append((float(t), float(a), axis, x0.copy()))
    return out


def solve_tcp_error(tips_reported: np.ndarray, R_tcp: np.ndarray, plane_normal: np.ndarray,
                    plane_point: np.ndarray) -> dict[str, Any]:
    """The pen-tip (TCP) error `e`, in the TCP frame, from touches on a KNOWN plane.

    Each touch reports a tip `p_i` computed with the configured TCP, at orientation
    `R_i`. The real tip is `p_i + R_i e`, and it lies on the plane:
        n . (p_i + R_i e - p0) = 0   ->   (R_i^T n) . e + delta = -h_i,
    with h_i the reported tip's height above the plane and `delta` a free offset of
    the plane (the plane was itself measured with the same TCP, so it carries the
    vertical component of `e` - `delta` absorbs it). Vertical touches alone see only
    e_z (collinear with delta); tilted ones make e_x, e_y observable, with a
    sensitivity of sin(tilt).
    """
    P = np.asarray(tips_reported, float).reshape(-1, 3)
    Rs = np.asarray(R_tcp, float).reshape(-1, 3, 3)
    n = np.asarray(plane_normal, float); n = n / np.linalg.norm(n)
    p0 = np.asarray(plane_point, float)
    h = (P - p0) @ n
    A = np.column_stack([np.array([R.T @ n for R in Rs]), np.ones(len(P))])
    x, *_ = np.linalg.lstsq(A, -h, rcond=None)
    res = A @ x + h
    # Conditioning, per question. LATERAL (e_x, e_y - what the roll test asks): the
    # smallest singular value of their columns after the along-pen and plane-offset
    # columns are regressed out. Vertical touches on a nearly level table give
    # ~sin(table tilt), 0.02 here: formally full rank, useless in practice; 30 deg
    # tilts give ~0.3. ALONG THE PEN (e_z): its column after the rest is regressed out;
    # it separates from the plane offset only by (1 - cos tilt), so it stays weak
    # (~0.05 at 30 deg) - use 45 deg tilts if the pen length itself is in doubt.
    def _cond(cols, others):
        Aa, Ao = A[:, cols], A[:, others]
        Aa = Aa - Ao @ np.linalg.lstsq(Ao, Aa, rcond=None)[0]
        return float(np.linalg.svd(Aa, compute_uv=False).min() / np.sqrt(len(P)))
    sv_lat = _cond([0, 1], [2, 3])
    sv_z = _cond([2], [0, 1, 3])
    e = x[:3]
    return {"e_tcp_mm": (e * 1000.0).tolist(), "lateral_mm": float(np.hypot(e[0], e[1]) * 1000.0),
            "along_pen_mm": float(e[2] * 1000.0), "plane_offset_mm": float(x[3] * 1000.0),
            "heights_mm": (h * 1000.0).tolist(), "residual_rmse_mm": float(np.sqrt(np.mean(res ** 2)) * 1000.0),
            "lateral_conditioning": sv_lat, "well_conditioned": bool(sv_lat > 0.1),
            "along_pen_conditioning": sv_z, "along_pen_conditioned": bool(sv_z > 0.1),
            "n_touches": int(len(P))}


# --------------------------------------------------------------------------- extrinsic check
def surface_vs_plane(pts: np.ndarray, normal: np.ndarray, p0: np.ndarray,
                     search_m: float = 0.25, band_m: float = 0.008, radius_m: float = 0.0,
                     min_points: int = 500) -> dict[str, Any]:
    """The dominant flat surface among `pts` (base_link, m) compared with a known plane.

    The table is found as the densest 2 mm height bin within +-`search_m` of the known
    plane - whatever its height, so a large error is MEASURED, not filtered out - then
    the points within `band_m` of that bin get a robust plane fit (3 trimming rounds).
    Returns the height offset of the fitted surface against the known plane extended to
    the surface's own centre (mm, + = the camera sees the table higher), the tilt between
    the normals (deg) and its azimuth, the distance from `p0` (how far the known plane
    was extrapolated), or {"error": ...}.
    """
    P = np.asarray(pts, float).reshape(-1, 3)
    n = np.asarray(normal, float); n = n / np.linalg.norm(n)
    p0 = np.asarray(p0, float)
    h = (P - p0) @ n
    region = np.abs(h) < search_m
    if radius_m > 0.0:
        region &= np.linalg.norm((P - p0)[:, :2], axis=1) < radius_m
    if region.sum() < min_points:
        return {"error": f"no surface within +-{search_m * 1000:.0f} mm of the known plane "
                         f"(median height of all points {np.median(h) * 1000:+.1f} mm)"}
    edges = np.arange(-search_m, search_m + 0.002, 0.002)
    hist, _ = np.histogram(h[region], bins=edges)
    k = int(np.argmax(hist))
    peak = 0.5 * (edges[k] + edges[k + 1])
    sel = P[region & (np.abs(h - peak) < band_m)]
    if len(sel) < min_points:
        return {"error": f"only {len(sel)} points on the dominant surface (peak {peak * 1000:+.1f} mm)"}
    for _ in range(3):
        f = fit_plane(sel)
        res = np.asarray(f["residuals_mm"])
        mad = np.median(np.abs(res - np.median(res))) * 1.4826 + 0.05
        sel = sel[np.abs(res - np.median(res)) < 3.0 * mad]
    f = fit_plane(sel)
    c = sel.mean(axis=0)
    z_cam = f["a"] * c[0] + f["b"] * c[1] + f["c"]
    z_known = p0[2] - (n[0] * (c[0] - p0[0]) + n[1] * (c[1] - p0[1])) / n[2]
    nc = np.asarray(f["normal"], float)
    d = nc - (nc @ n) * n
    return {"height_offset_mm": float((z_cam - z_known) * 1000.0),
            "tilt_deg": float(np.degrees(np.arccos(np.clip(nc @ n, -1.0, 1.0)))),
            "tilt_azimuth_deg": float(np.degrees(np.arctan2(d[1], d[0]))) if np.linalg.norm(d) > 1e-12 else 0.0,
            "plane_rmse_mm": f["rmse_mm"], "n_points": int(len(sel)),
            "surface_centre_m": c.tolist(), "extrapolation_m": float(np.linalg.norm((c - p0)[:2]))}


def separate_tilt(tilt_deg, tilt_azimuth_deg, camera_yaw_deg) -> dict[str, Any]:
    """Split measured table tilts into a WORLD-fixed part and a CAMERA-fixed part.

    Each view measured a small tilt vector t_i (magnitude along its azimuth, in the table
    plane). A tilt caused by the camera - extrinsic rotation, a tilted depth sensor -
    turns with the camera's yaw psi_i; one caused by the world - the table's own shape
    there, an error of the pen plane - does not:
        t_i = a + Rot(psi_i) b          (small-angle, 2-D)
    `a` is the world part, `b` the camera part expressed in the camera's yaw frame.
    Needs camera yaws spread by >= 90 deg, or a and b are not separable (reported).
    """
    t = np.asarray(tilt_deg, float); az = np.deg2rad(np.asarray(tilt_azimuth_deg, float))
    psi = np.deg2rad(np.asarray(camera_yaw_deg, float))
    T = np.column_stack([t * np.cos(az), t * np.sin(az)]).reshape(-1)
    rows = []
    for p in psi:
        c, s = np.cos(p), np.sin(p)
        rows.append([1.0, 0.0, c, -s])
        rows.append([0.0, 1.0, s, c])
    A = np.array(rows)
    spread = float(np.degrees(np.ptp(np.unwrap(psi)))) if len(psi) > 1 else 0.0
    sv = np.linalg.svd(A, compute_uv=False)
    separable = bool(spread >= 90.0 and sv.min() > 0.3)
    x, *_ = np.linalg.lstsq(A, T, rcond=None)
    res = A @ x - T
    return {"world_tilt_deg": float(np.hypot(x[0], x[1])),
            "world_tilt_azimuth_deg": float(np.degrees(np.arctan2(x[1], x[0]))),
            "camera_tilt_deg": float(np.hypot(x[2], x[3])),
            "camera_tilt_direction_in_camera_deg": float(np.degrees(np.arctan2(x[3], x[2]))),
            "residual_deg": float(np.sqrt(np.mean(res ** 2))), "yaw_spread_deg": spread,
            "separable": separable}


def tilt_to_normal(n0: np.ndarray, tilt_deg: float, azimuth_deg: float) -> np.ndarray:
    """The unit normal tilted by `tilt_deg` from `n0` toward the table direction
    `azimuth_deg` (the inverse of how `surface_vs_plane` reports a tilt)."""
    n0 = np.asarray(n0, float) / np.linalg.norm(n0)
    a = np.deg2rad(azimuth_deg)
    u = np.array([np.cos(a), np.sin(a), 0.0])
    u = u - (u @ n0) * n0
    u /= np.linalg.norm(u)
    t = np.deg2rad(tilt_deg)
    return np.cos(t) * n0 + np.sin(t) * u


def refine_camera_rotation(R_base_cam: np.ndarray, n_observed: np.ndarray, n0: np.ndarray
                           ) -> dict[str, Any]:
    """The small camera rotation that makes every view's table level with the known one.

    Model: the extrinsic in use is off by a rotation Rc = exp([w]) in the CAMERA frame
    (true camera = used @ Rc). A plane whose true normal is n_true then appears, through
    the used extrinsic, as n_obs = R_i Rc^T R_i^T n_true, i.e. to first order
        n_obs_i - n0 = a + n0 x (R_i w),
    with `a` (in-plane, 2 dof) the table's own tilt there against the known plane - the
    world-fixed part - and w the camera-fixed part. w's component along the optical axis
    is unobservable from a plane when every view looks down, so it is fixed to 0: this
    refines the camera's two TILT angles, not its yaw about the optical axis.

    Returns w (deg, camera frame), a (deg), the residual (deg) and Rc; apply as
    T_tool0_cam_refined = T_tool0_cam @ [[Rc, 0], [0, 1]].
    """
    Rs = np.asarray(R_base_cam, float).reshape(-1, 3, 3)
    N = np.asarray(n_observed, float).reshape(-1, 3)
    n0 = np.asarray(n0, float) / np.linalg.norm(n0)
    e1 = np.cross(n0, [0.0, 1.0, 0.0]); e1 /= np.linalg.norm(e1)
    e2 = np.cross(n0, e1)
    rows, rhs = [], []
    for R, n in zip(Rs, N):
        d = n / np.linalg.norm(n) - n0
        A = np.column_stack([e1, e2, np.cross(n0, R[:, 0]), np.cross(n0, R[:, 1])])
        rows.append(A); rhs.append(d)
    A = np.vstack(rows); b = np.concatenate(rhs)
    x, *_ = np.linalg.lstsq(A, b, rcond=None)
    res = A @ x - b
    w = np.array([x[2], x[3], 0.0])
    sv = np.linalg.svd(A[:, 2:], compute_uv=False)
    return {"w_camera_deg": np.degrees(w).tolist(), "w_deg": float(np.degrees(np.linalg.norm(w))),
            "world_tilt_deg": float(np.degrees(np.hypot(x[0], x[1]))),
            "residual_deg": float(np.degrees(np.sqrt(np.mean(res ** 2)) * np.sqrt(3.0))),
            "conditioning": float(sv.min() / np.sqrt(len(Rs))), "Rc": rot_exp(w).tolist(),
            "n_views": int(len(Rs))}
