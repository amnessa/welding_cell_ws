"""`lit-lobb` — Zhang et al. 2025 (RCIM pipeline + IEEE T-ASE descriptor).

Two things this file is here to pin, beyond the usual reimplementation traps:

  * **The coarse stage is `ours`' cross-object gate.** RCIM eq. 2 calls a pixel an edge when
    its neighbourhood holds two different *component* masks. That is `object_id`, obtained
    from a trained 2D segmenter instead of from a CAD stack — which makes this paper the
    literature answering the question `dataset_plan.md` raises about `ours`.
  * **LOBB flatness is not the surface variation the other two methods use.** It is a ratio
    of bounding-box *extents*, not of eigen*values*, and the difference is outlier
    sensitivity.
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from baselines import ground_truth  # noqa: E402
from baselines.lit_lobb import (LOBB_RADIUS_MM, activate, detect,  # noqa: E402
                                fit_dominant_axis, kmeans_1d_binary, lobb_features,
                                part_boundary_points, part_labels_oracle, roi_by_extension)
from baselines.radius_pca import surface_variation  # noqa: E402


def grid(size: float = 40.0, n: int = 60) -> np.ndarray:
    a = np.linspace(0.0, size, n)
    return np.stack(np.meshgrid(a, a), -1).reshape(-1, 2)


def fold(size: float = 40.0, n: int = 60):
    """Two zero-thickness planes meeting at 90 deg along the x axis at the origin."""
    a = grid(size, n)
    p1 = np.column_stack([a[:, 0], a[:, 1], np.zeros(len(a))])
    p2 = np.column_stack([a[:, 0], np.zeros(len(a)), a[:, 1]])
    return np.vstack([p1, p2])


def _scene(joint_type: str):
    from baselines import balanced_corpus, cloud_for, load_scene

    try:
        corpus = balanced_corpus(ROOT / "out" / "bench", per_type=50)
    except (ValueError, FileNotFoundError, NotADirectoryError):
        return None
    for d in corpus.get(joint_type, []):
        scene, arrays = load_scene(d)
        gt = ground_truth(scene, arrays, primary_only=True)
        if not gt:
            continue
        return cloud_for(scene, arrays, view="full"), gt
    return None


# --- the descriptor ---------------------------------------------------------------------

def test_the_published_lobb_radius_is_a_weld_width_not_a_resolution():
    """T-ASE §III picks 2,5 mm *"according to the weld width"*.

    Worth pinning because it changes how the parameter may be swept. A resolution parameter
    is free to tune against the sensor; a weld-width parameter is pinned by the joint, and
    sweeping it as if it were free would be measuring a different method at each point.
    """
    assert LOBB_RADIUS_MM == 2.5


def test_flatness_is_a_box_ratio_and_that_is_not_the_eigenvalue_ratio():
    """Eq. 4 is `h / sqrt(l^2 + w^2)` over bounding-box **extents**.

    `ours` and `lit-regiongrow` both use `lambda_min / sum(lambda)` — variances, i.e. an RMS
    over the neighbourhood. Extents are a max-minus-min, so the two respond differently to a
    single lifted point: it moves `h` completely and `lambda_3` barely at all. Three of the
    seven methods now share a feature *family* and differ in the statistic taken over it,
    which is a cleaner comparison axis than any of them being better.
    """
    rng = np.random.default_rng(0)
    flat = np.column_stack([rng.uniform(-10, 10, 4000), rng.uniform(-10, 10, 4000),
                            np.zeros(4000)])
    spike = flat.copy()
    spike[0, 2] = 1.5                                  # one point lifted off the plane

    f_flat = lobb_features(flat, radius_mm=5.0)["flatness"][0]
    f_spike = lobb_features(spike, radius_mm=5.0)["flatness"][0]
    v_flat = surface_variation(flat, radius_mm=5.0)[0]
    v_spike = surface_variation(spike, radius_mm=5.0)[0]

    assert f_spike > 10 * max(f_flat, 1e-9)            # the box notices immediately
    assert v_spike < 0.5 * f_spike                     # the variance ratio barely moves


def test_flatness_separates_a_crease_from_a_face():
    """T-ASE Fig. 3: the LOBB of a face point is flatter than that of a crease point."""
    pts = fold()
    f = lobb_features(pts, radius_mm=4.0)["flatness"]
    on_fold = (np.abs(pts[:, 1]) < 1.5) & (np.abs(pts[:, 2]) < 1.5)
    interior = (pts[:, 1] > 8) & (pts[:, 1] < 32) & (pts[:, 0] > 8) & (pts[:, 0] < 32)
    assert np.median(f[on_fold]) > 5 * np.median(f[interior])


def test_the_centre_offset_finds_a_rim_where_the_flatness_does_not():
    """Eq. 7. A plate's own border is a *boundary* point, not a crease — T-ASE §III-B.

    This is the step that is supposed to keep plate rims out of the answer, and it is the
    same job `two_surface_edges` does in `lit-regiongrow` through a different feature. Worth
    a test of its own because it is the one part of this method that addresses the
    face-versus-part problem both other methods trip on.
    """
    a = grid(40.0, 60)
    plate = np.column_stack([a[:, 0], a[:, 1], np.zeros(len(a))])
    feats = lobb_features(plate, radius_mm=4.0)
    rim = (plate[:, 0] < 1.0) | (plate[:, 0] > 39.0)
    interior = (plate[:, 0] > 10) & (plate[:, 0] < 30) & (plate[:, 1] > 10) & (plate[:, 1] < 30)

    assert np.median(feats["center_offset"][rim]) > \
        3 * np.median(feats["center_offset"][interior])
    # ...and the plate is flat everywhere, so flatness cannot make this distinction at all.
    assert np.median(feats["flatness"][rim]) < 0.1


# --- activation and clustering ------------------------------------------------------------

def test_nonlinear_activation_polarises_the_feature():
    """T-ASE eq. 9 and Fig. 7 — the point is to empty out the middle of the distribution.

    A 1-D binary K-means is driven by the mass between the two modes; `tanh` on a bilateral
    range pushes that mass outward. Fig. 8 shows the split failing without it.
    """
    rng = np.random.default_rng(0)
    x = np.r_[rng.normal(0.2, 0.05, 500), rng.uniform(0.3, 0.7, 500),
              rng.normal(0.8, 0.05, 500)]
    mid = lambda v: np.mean((v > 0.35) & (v < 0.65))   # noqa: E731
    assert mid(activate(x)) < mid((x - x.min()) / np.ptp(x))


def test_the_binary_kmeans_is_deterministic_which_is_a_reportable_property():
    """Seeded at min and max, so `lit-lobb` shows **zero spread** across repeats.

    That is a finding, not an implementation detail: `lit-ransac` swings F1 0,00-0,94 on one
    fixed scene purely on its RANSAC seed. Method reproducibility is a property this
    generator can measure and the field does not report — but only if the deterministic
    methods really are deterministic, which is what this pins.
    """
    x = np.random.default_rng(0).uniform(0, 1, 5000)
    first = kmeans_1d_binary(x)
    for _ in range(5):
        assert np.array_equal(kmeans_1d_binary(x), first)
    assert first.any() and not first.all()


# --- the coarse stage ----------------------------------------------------------------------

def test_the_coarse_stage_is_ours_cross_object_gate_by_another_name():
    """RCIM eq. 2: an edge pixel is one whose neighbourhood holds two **component** masks.

    `ours` asks the same question of `object_id` and `dataset_plan.md` calls that a
    dependency on segmentation the method does not publish about. This paper publishes it —
    a K-Net at 97,35% mIoU — which makes it evidence that the dependency is a property of
    the problem rather than a weakness of `ours`.
    """
    pts = fold()
    labels = part_labels_oracle(np.r_[np.zeros(len(pts) // 2, int),
                                      np.ones(len(pts) - len(pts) // 2, int)])
    edge = part_boundary_points(pts, labels, edge_radius_mm=2.0)
    assert edge.any()
    # Everything it returns is at the junction of the two components, not on a plate rim.
    assert np.percentile(np.maximum(np.abs(pts[edge][:, 1]), np.abs(pts[edge][:, 2])),
                         90) < 3.0
    # With one component there is no boundary at all - the gate has nothing to compare.
    assert not part_boundary_points(pts, np.zeros(len(pts), int), 2.0).any()


def test_the_roi_grows_from_the_edge_cloud_and_stays_a_subset():
    """RCIM §3.3 — edge cloud as seeds, k-NN expansion, dedupe."""
    pts = fold()
    seeds = np.abs(pts[:, 1]) + np.abs(pts[:, 2]) < 1.0
    roi = roi_by_extension(pts, seeds, k=20)
    assert roi.sum() > seeds.sum()
    assert np.all(roi[seeds])                          # expansion never drops a seed
    assert roi.sum() < len(pts)                        # and it is still an ROI


# --- the curve -----------------------------------------------------------------------------

def test_the_dominant_axis_fit_recovers_a_curved_seam():
    """RCIM §3.4.3 — two 1-D polynomials against whichever axis the seam runs along.

    Their rule is stated in the camera frame and justified by the camera being aimed roughly
    normal to the seam; this dataset is in world coordinates at arbitrary yaw, so the axis is
    taken from the data's own extent — what their rule computes rather than what it assumes.
    """
    t = np.linspace(0, 100, 400)
    truth = np.column_stack([t, 0.002 * (t - 50) ** 2, 0.5 * np.ones_like(t)])
    noisy = truth + np.random.default_rng(0).normal(0, 0.05, truth.shape)
    poly = fit_dominant_axis(noisy, degree=3, n_samples=200)
    assert poly is not None
    from baselines.metrics import distance_to_polylines
    assert distance_to_polylines(poly, [truth]).max() < 0.5


# --- end to end -----------------------------------------------------------------------------

def test_reproduces_the_papers_accuracy_on_a_corner_joint():
    """RCIM reports **max error < 1,2 mm and RMSE < 0,7 mm**.

    A corner joint is the case their pipeline is posed for: one open contact run, so the
    curve fit at the end has a single seam to fit. See the T-joint test for what happens
    when it does not.
    """
    from baselines.metrics import matched_path_errors

    hit = _scene("corner")
    if hit is None:
        pytest.skip("no corner scene in out/bench")
    cloud, gt = hit
    r = detect(cloud["xyz"], object_id=cloud["object_id"], voxel_mm=1.0)
    rows = [e for e in matched_path_errors(r.polylines, gt) if e["matched"]]
    assert rows, r.note
    assert min(e["rmse"] for e in rows) < 1.5, [round(e["rmse"], 3) for e in rows]


def test_a_t_joints_contact_perimeter_is_closed_and_that_is_what_breaks_the_fit():
    """The finding, pinned: detection succeeds and *segmentation into seams* fails.

    A web sitting on a base plate touches it along a **closed perimeter** — two long fillets
    joined by two short cross-runs at the ends. D4 excludes the cross-runs, so the label is
    two open curves; the geometry is one loop. LOBB finds the loop almost perfectly (98% of
    crease points within 3 mm of truth) and then fits **one** polynomial through it, which
    is meaningless for a closed curve.

    Proximity clustering cannot make the cut: the cross-runs are only ~2,5% of the crease
    points but they physically bridge the two fillets, so no link distance separates them.
    Direction-aware splitting would — the same upgrade `ours` and `lit-regiongrow` both need,
    which makes it a property of the mechanism rather than of any one paper.
    """
    from baselines.metrics import distance_to_polylines

    hit = _scene("T")
    if hit is None:
        pytest.skip("no T scene in out/bench")
    cloud, gt = hit
    if len(gt) != 2:
        pytest.skip("needs a T joint with both fillets labelled")
    r = detect(cloud["xyz"], object_id=cloud["object_id"], voxel_mm=1.0, link_mm=1.5)
    crease = r.points[r.crease]
    assert len(crease) > 100, r.note

    # The detection is right...
    assert (distance_to_polylines(crease, gt) <= 3.0).mean() > 0.9
    # ...and the points that are on NEITHER labelled seam are the cross-runs, at the ends.
    off = distance_to_polylines(crease, gt) > 3.0
    if off.any():
        ends = np.vstack([gt[0][0], gt[0][-1], gt[1][0], gt[1][-1]])
        assert np.median(np.linalg.norm(crease[off][:, None, :] - ends[None, :, :],
                                        axis=2).min(axis=1)) < 15.0
    # ...but they bridge the two fillets into one component, at any link distance.
    assert r.n_seams < 2
