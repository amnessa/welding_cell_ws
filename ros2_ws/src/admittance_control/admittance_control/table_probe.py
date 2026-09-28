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
