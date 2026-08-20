"""Seam-detection metrics — Phase 4, `dataset_plan.md`.

All lengths **millimetres**.

Ground truth here is a **polyline**, not a point set, which is the whole reason this
dataset exists: the seam is stored parametrically, so distance-to-truth is an exact
point-to-segment distance rather than a nearest-sampled-neighbour approximation whose floor
is the sampling density. Comparing a prediction against sampled truth would put the
sampling interval into every error bar.

Chamfer is primary — cheap, standard, report it everywhere. Precision/recall at a tolerance
answers a different and equally necessary question, because Chamfer alone cannot distinguish
"the seam, found accurately" from "the seam plus a phantom along the fixture contact": one
extra component barely moves a mean, and destroys precision.
"""

from __future__ import annotations

from typing import Any

import numpy as np

try:
    from scipy.spatial import cKDTree
except ImportError:                                    # pragma: no cover
    cKDTree = None


def _nearest(query: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Distance from each `query` point to the nearest `target` point.

    A KD-tree rather than the obvious `query[:, None] - target[None, :]` broadcast: that
    allocates `len(query) x len(target) x 3` floats, and an edge joint's band is 26 000
    points, which is enough to take the process out with an OOM kill.
    """
    if len(query) == 0 or len(target) == 0:
        return np.full(len(query), np.inf)
    if cKDTree is None:                                # pragma: no cover
        return np.min(np.linalg.norm(query[:, None, :] - target[None, :, :], axis=-1),
                      axis=1)
    return cKDTree(target).query(query, k=1, workers=-1)[0]


# --------------------------------------------------------------------------------
# geometry
# --------------------------------------------------------------------------------

def point_segment_distance(pts: np.ndarray, p0: np.ndarray, p1: np.ndarray) -> np.ndarray:
    """Distance from each point to the SEGMENT p0-p1 (not the infinite line)."""
    pts = np.asarray(pts, dtype=float)
    p0 = np.asarray(p0, dtype=float)
    d = np.asarray(p1, dtype=float) - p0
    L2 = float(d @ d)
    if L2 < 1e-18:
        return np.linalg.norm(pts - p0, axis=1)
    t = np.clip((pts - p0) @ d / L2, 0.0, 1.0)         # clamped: the seam has ends
    return np.linalg.norm(pts - (p0 + t[:, None] * d), axis=1)


def distance_to_polylines(pts: np.ndarray, polylines) -> np.ndarray:
    """Distance from each point to the NEAREST of several polylines.

    A scene has several seams, and a prediction is not wrong for sitting on the second one.
    """
    pts = np.asarray(pts, dtype=float)
    if len(pts) == 0:
        return np.zeros(0)
    best = np.full(len(pts), np.inf)
    for poly in polylines:
        poly = np.asarray(poly, dtype=float)
        for a, b in zip(poly[:-1], poly[1:]):
            best = np.minimum(best, point_segment_distance(pts, a, b))
    return best


def _sample_polyline(poly: np.ndarray, step_mm: float) -> np.ndarray:
    """Resample a polyline at a fixed spacing, for the truth→prediction direction."""
    poly = np.asarray(poly, dtype=float)
    out = []
    for a, b in zip(poly[:-1], poly[1:]):
        L = float(np.linalg.norm(b - a))
        n = max(2, int(round(L / max(step_mm, 1e-6))) + 1)
        t = np.linspace(0.0, 1.0, n)[:, None]
        out.append(a[None, :] * (1 - t) + b[None, :] * t)
    return np.vstack(out) if out else np.asarray(poly, dtype=float)


# --------------------------------------------------------------------------------
# metrics
# --------------------------------------------------------------------------------

def chamfer_mm(pred: np.ndarray, gt_polylines, step_mm: float = 1.0) -> dict[str, float]:
    """Symmetric Chamfer between a predicted point set and the truth polylines.

    Both directions are returned separately as well as summed, because they fail
    differently and the sum hides which: `pred_to_gt` rises when the prediction wanders off
    the seam, `gt_to_pred` rises when part of the seam was never found. A method that
    returns three correct points out of a 200 mm seam scores beautifully on the first and
    terribly on the second.
    """
    pred = np.asarray(pred, dtype=float)
    polys = [np.asarray(p, dtype=float) for p in gt_polylines]
    if len(pred) == 0 or not polys:
        return {"pred_to_gt": float("nan"), "gt_to_pred": float("nan"),
                "chamfer": float("nan")}

    a = float(distance_to_polylines(pred, polys).mean())
    gt_pts = np.vstack([_sample_polyline(p, step_mm) for p in polys])
    b = float(_nearest(gt_pts, pred).mean())
    return {"pred_to_gt": a, "gt_to_pred": b, "chamfer": a + b}


def precision_recall_f1(pred: np.ndarray, gt_polylines, tol_mm: float,
                        step_mm: float = 1.0) -> dict[str, float]:
    """Fraction of the prediction on the seam, and of the seam covered, within `tol_mm`.

    Recall is computed over resampled truth, so it means "fraction of seam LENGTH covered"
    rather than "fraction of truth points hit" - the latter would depend on how densely the
    ground truth happened to be stored.
    """
    pred = np.asarray(pred, dtype=float)
    polys = [np.asarray(p, dtype=float) for p in gt_polylines]
    if len(pred) == 0 or not polys:
        return {"precision": 0.0, "recall": 0.0, "f1": 0.0}

    precision = float((distance_to_polylines(pred, polys) <= tol_mm).mean())
    gt_pts = np.vstack([_sample_polyline(p, step_mm) for p in polys])
    recall = float((_nearest(gt_pts, pred) <= tol_mm).mean())
    f1 = 0.0 if precision + recall == 0 else 2 * precision * recall / (precision + recall)
    return {"precision": precision, "recall": recall, "f1": f1}


def lateral_error_mm(pred: np.ndarray, gt_polylines) -> dict[str, float]:
    """Distance to truth as a distribution, not a mean.

    `p95` is the one that matters for a toolpath: a torch is not forgiving of the tail, and
    a method with a 1 mm mean and a 20 mm p95 is not usable even though it reports well.
    """
    d = distance_to_polylines(pred, [np.asarray(p, dtype=float) for p in gt_polylines])
    if len(d) == 0:
        return {"mean": float("nan"), "median": float("nan"), "p95": float("nan"),
                "max": float("nan")}
    return {"mean": float(d.mean()), "median": float(np.median(d)),
            "p95": float(np.percentile(d, 95)), "max": float(d.max())}


def band_width_mm(pred: np.ndarray, gt_polylines) -> float:
    """Twice the median distance to truth — how wide the returned "seam" actually is.

    README §8 reports the surviving band as ~16 mm wide rather than two thin lines, and
    calls it an honest caveat. This is that caveat as a number, and it is the quantity the
    line-fit upgrade is supposed to move.
    """
    d = distance_to_polylines(pred, [np.asarray(p, dtype=float) for p in gt_polylines])
    return float(2.0 * np.median(d)) if len(d) else float("nan")


def seam_count_error(n_pred: int, n_gt: int) -> int:
    """Signed miscount. Negative = seams missed, positive = phantoms."""
    return int(n_pred) - int(n_gt)


def densify(polylines_or_points, step_mm: float = 1.0) -> np.ndarray:
    """A comparable point set from either a list of polylines or a raw (N,3) cloud.

    Both sides of Chamfer must be sampled at the same rate or the metric measures the
    sampling. A predicted *polyline* is two endpoints; scored as a point set against 200 mm
    of truth it looks like a method that found almost nothing, when in fact it drew the
    whole seam. Densifying both sides removes that artefact.
    """
    if isinstance(polylines_or_points, np.ndarray) and polylines_or_points.ndim == 2 \
            and not isinstance(polylines_or_points, list):
        return np.asarray(polylines_or_points, dtype=float)
    parts = [_sample_polyline(np.asarray(p, dtype=float), step_mm)
             for p in polylines_or_points if len(np.asarray(p)) >= 2]
    return np.vstack(parts) if parts else np.zeros((0, 3))


def evaluate_band(band: np.ndarray, gt_polylines, n_clusters: int | None = None,
                  tol_mm: float = 3.0, step_mm: float = 1.0) -> dict[str, Any]:
    """Score the band **as a point set**, with no line fitted through it.

    The line fit is gone on purpose. On these joints the band is a rectangle and a
    total-least-squares line through it sits in the middle of that rectangle - which is the
    mid-surface between two plates, not the seam. Scoring the band directly asks the
    question the method can actually answer: *is the seam inside what it returned, and how
    much of what it returned is not seam?*

    `precision` is now the honest cost of the band's width: a 16 mm band around a seam
    scores badly on it however well centred it is, which is exactly the `README §8` caveat
    turned into a number rather than a paragraph.
    """
    band = np.asarray(band, dtype=float)
    out: dict[str, Any] = {"n_band_points": int(len(band)),
                           "n_clusters": int(n_clusters) if n_clusters is not None else -1,
                           "n_gt_seams": len(gt_polylines)}
    if n_clusters is not None:
        out["cluster_count_error"] = int(n_clusters) - len(gt_polylines)
    out.update(chamfer_mm(band, gt_polylines, step_mm))
    out.update(precision_recall_f1(band, gt_polylines, tol_mm, step_mm))
    out.update({f"lat_{k}": v for k, v in lateral_error_mm(band, gt_polylines).items()})
    out["band_width_mm"] = band_width_mm(band, gt_polylines)
    return out


def evaluate(pred_seams, gt_polylines, band: np.ndarray | None = None,
             tol_mm: float = 3.0, step_mm: float = 1.0) -> dict[str, Any]:
    """Every metric at once, for one scene, as a flat dict ready for a DataFrame.

    `pred_seams` is the list of fitted seam polylines - the method's actual answer.
    `band`, if given, is the raw thresholded point set before the line fit; it is scored
    alongside under a `band_` prefix, because the difference between the two is exactly
    what the line-fit upgrade is worth and it should be visible rather than asserted.
    """
    pred_pts = densify(pred_seams, step_mm)
    out: dict[str, Any] = {
        "n_pred_seams": len(pred_seams), "n_gt_seams": len(gt_polylines),
        "seam_count_error": seam_count_error(len(pred_seams), len(gt_polylines)),
        "n_pred_points": int(len(pred_pts)),
    }
    out.update(chamfer_mm(pred_pts, gt_polylines, step_mm))
    out.update(precision_recall_f1(pred_pts, gt_polylines, tol_mm, step_mm))
    out.update({f"lat_{k}": v
                for k, v in lateral_error_mm(pred_pts, gt_polylines).items()})
    out["band_width_mm"] = band_width_mm(pred_pts, gt_polylines)

    if band is not None:
        b = np.asarray(band, dtype=float)
        out["band_chamfer"] = chamfer_mm(b, gt_polylines, step_mm)["chamfer"]
        out["band_lat_p95"] = lateral_error_mm(b, gt_polylines)["p95"]
        out["band_width_raw_mm"] = band_width_mm(b, gt_polylines)
        out["n_band_points"] = int(len(b))
    return out
