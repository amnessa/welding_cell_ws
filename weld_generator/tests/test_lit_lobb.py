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
from baselines.lit_lobb import (EDGE_RADIUS_MM_LEGACY, EDGE_RADIUS_SCALE,  # noqa: E402
                                LOBB_RADIUS_MM, N_SAMPLES, activate, detect,
                                edge_radius_for, fit_dominant_axis, kmeans_1d_binary,
                                lobb_features, mean_point_spacing, part_boundary_points,
                                part_labels_oracle, roi_by_extension)
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


def tee(size: float = 40.0, n: int = 41, web_h: float = 20.0, nh: int = 21,
        sigma: float = 0.1, seed: int = 0):
    """A synthetic two-plate T: a base plate in z=0 and a web standing on it at y=size/2.

    Zero thickness, ~1 mm spacing, one fillet line along `y = size/2, z = 0` — the smallest
    scene that has a crease, a face and a plate rim at once, with `object_id` per plate so
    the RCIM §3.1-3.3 coarse stage runs for real. `sigma` is a light sensor-like jitter:
    without it the flatness histogram is two delta functions and every clustering variant
    agrees, which is exactly the regime that hid the flattened hierarchy.
    """
    a = np.linspace(0.0, size, n)
    X, Y = np.meshgrid(a, a)
    base = np.column_stack([X.ravel(), Y.ravel(), np.zeros(X.size)])
    z = np.linspace(0.0, web_h, nh)
    X2, Z = np.meshgrid(a, z)
    web = np.column_stack([X2.ravel(), np.full(X2.size, size / 2.0), Z.ravel()])
    pts = np.vstack([base, web])
    oid = np.r_[np.zeros(len(base), int), np.ones(len(web), int)]
    if sigma:
        pts = pts + np.random.default_rng(seed).normal(0.0, sigma, pts.shape)
    return pts, oid


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


def test_the_box_extents_are_sorted_l_ge_w_ge_h():
    """T-ASE §III-A: *"an OBB with length l, width w, and height h ... where l >= w >= h"*.

    The trap this pins: PCA orders **variances**, not extents, and the two orders differ
    whenever a direction is long-but-sparse (a couple of outliers: big extent, small
    variance) while another is short-but-bimodal (small extent, big variance). This patch is
    built to be exactly that — uniform along x, two tight lobes along y, four outliers along
    z — so its eigen-ordered extents come out `(40, 11,5, 16)`, un-sorted. Taking `h` as the
    third eigen-direction then puts a 16 mm extent in the numerator of eq. 4 and reports a
    crease (f = 0,385) where the sorted box reports a band (f = 0,268).

    Checked against the sorted box computed here in the test rather than against a magic
    number, so it pins the *rule*, and against the un-sorted value to show the rule bites.
    """
    rng = np.random.default_rng(0)
    n = 400
    x = rng.uniform(-20.0, 20.0, n)                             # extent 40, variance 133
    y = np.where(rng.random(n) < 0.5, -5.0, 5.0) + rng.normal(0, 0.2, n)  # extent 10, var 25
    z = np.zeros(n)
    z[:4] = [-8.0, 8.0, -8.0, 8.0]                              # extent 16, variance 0,6
    patch = np.column_stack([x, y, z])

    c = patch - patch.mean(axis=0)
    axes = np.linalg.eigh(c.T @ c / len(patch))[1][:, ::-1]     # e1, e2, e3, lambda desc
    loc = c @ axes
    ext = loc.max(axis=0) - loc.min(axis=0)
    assert not np.all(np.diff(ext) <= 0), ext                   # the premise: NOT sorted

    l_, w_, h_ = np.sort(ext)[::-1]                             # l >= w >= h, as published
    assert l_ >= w_ >= h_
    got = lobb_features(patch, radius_mm=1e6)["flatness"][0]
    assert got == pytest.approx(h_ / np.hypot(l_, w_), rel=1e-9)
    # ...and that is not what the eigen-order would have said.
    assert abs(got - ext[2] / np.hypot(ext[0], ext[1])) > 0.1
    # The entropy is a std over the same three numbers, so the sort leaves it alone.
    assert lobb_features(patch, radius_mm=1e6)["corner_entropy"][0] == \
        pytest.approx(float(np.std(ext)), rel=1e-9)


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


def test_the_edge_radius_is_resolution_relative_not_three_millimetres():
    """RCIM eq. 1-2 is an `n x n` **pixel** window with `n = 3` (§4.1), not a metric ball.

    Their camera is 1280 x 1024 over a 220 mm near field of view, ~0,17 mm/px, so the
    published window is about half a millimetre across — three SAMPLES wide. This module
    used a fixed 3,0 mm ball, which at this generator's ~1 pt/mm2 is six times too wide in
    sample units and drags a fat slab of face into the ROI seed. The default is now
    `1,5 x` the measured spacing, which is what a `3 x 3` window is worth at any density.

    On a regular grid the mean nearest-neighbour distance IS the grid step, so both halves
    of the claim are checkable exactly.
    """
    for step in (0.5, 1.0, 2.0):
        a = np.arange(0.0, 20.0 + 1e-9, step)
        g = np.stack(np.meshgrid(a, a), -1).reshape(-1, 2)
        plate = np.column_stack([g[:, 0], g[:, 1], np.zeros(len(g))])
        assert mean_point_spacing(plate) == pytest.approx(step, rel=1e-9)
        assert edge_radius_for(plate) == pytest.approx(EDGE_RADIUS_SCALE * step, rel=1e-9)
        assert edge_radius_for(plate, 3.0) == 3.0      # an explicit value still wins

    # ...and the narrower ball really does return a narrower seam band. Two components
    # meeting along y = 0 at 1 mm spacing: 1,5 mm reaches two rows each side, 3,0 mm four.
    pts = fold(40.0, 41)
    labels = part_labels_oracle(np.r_[np.zeros(len(pts) // 2, int),
                                      np.ones(len(pts) - len(pts) // 2, int)])
    auto = part_boundary_points(pts, labels)                    # None -> 1,5 x spacing
    legacy = part_boundary_points(pts, labels, EDGE_RADIUS_MM_LEGACY)
    assert auto.any() and auto.sum() < legacy.sum()
    assert np.all(legacy[auto])                                 # the narrow band is inside


def test_a_root_gap_wider_than_the_window_switches_the_coarse_stage_off():
    """The price of reading RCIM eq. 1-2 faithfully, pinned so nobody rediscovers it.

    Their window never has to cross a root gap: in the image the two masks abut in
    projection however far apart the parts physically are. In 3D they do not. At ~1 pt/mm2
    the resolution-relative ball is ~1,5 mm, so a 2 mm gap puts every point of B out of
    reach of every point of A and the edge test returns nothing — on `bench_phase4`
    `butt/line_square` (gap 1,49 mm) that is exactly what happens, and `detect` says so in
    `note` rather than inventing a seam. A gapped joint needs `edge_radius_mm` set
    explicitly, and the number is then a JOINT parameter that has to be reported as one.
    """
    a = np.linspace(0.0, 30.0, 31)
    X, Y = np.meshgrid(a, a)
    left = np.column_stack([X.ravel() - 31.0, Y.ravel(), np.zeros(X.size)])
    right = np.column_stack([X.ravel() + 1.0, Y.ravel(), np.zeros(X.size)])   # 2 mm gap
    pts = np.vstack([left, right])
    labels = part_labels_oracle(np.r_[np.zeros(len(left), int), np.ones(len(right), int)])

    assert not part_boundary_points(pts, labels).any()          # the window cannot cross it
    assert part_boundary_points(pts, labels, 3.0).any()         # an explicit ball can
    r = detect(pts, object_id=labels, voxel_mm=None)
    assert r.n_seams == 0 and "ROI too small" in r.note


def test_the_roi_grows_from_the_edge_cloud_and_stays_a_subset():
    """RCIM §3.3 — edge cloud as seeds, k-NN expansion, dedupe."""
    pts = fold()
    seeds = np.abs(pts[:, 1]) + np.abs(pts[:, 2]) < 1.0
    roi = roi_by_extension(pts, seeds, k=20)
    assert roi.sum() > seeds.sum()
    assert np.all(roi[seeds])                          # expansion never drops a seed
    assert roi.sum() < len(pts)                        # and it is still an ROI


def test_the_hierarchy_is_a_hierarchy_not_two_flat_splits():
    """T-ASE Fig. 9 (p. 79): boundary layer **first**, crease layer on what it left.

    *"In this algorithm, the boundary layer is performed first, and then the crease layer is
    performed."* The crease K-means therefore sees the **non-boundary subset**, and that is
    not the same algorithm as clustering the whole ROI and subtracting the boundary mask
    afterwards: a 1-D K-means threshold is fitted to whatever population it is handed, so
    leaving the rim points in the pot moves the split. `kmeans="flat"` keeps the old path so
    the difference is measured rather than asserted.

    Measured on this synthetic T at sigma = 0,1 mm: both find ~230 crease points, and the
    hierarchical split puts 95,6% of them within 2 mm of the true fillet line against 94,0%
    flat. A small gap, and one-directional — the hierarchy never dirties the crease set, and
    on a bench corner scene the same change moves RMSE 0,428 -> 0,112 mm.
    """
    pts, oid = tee()
    out = {}
    for mode in ("hierarchical", "flat"):
        r = detect(pts, object_id=oid, voxel_mm=None, kmeans=mode)
        c = r.points[r.crease]
        near = np.hypot(np.abs(c[:, 1] - 20.0), np.abs(c[:, 2])) < 2.0
        # Face points: on the base plate, well clear of both the fillet and the plate rim.
        P = r.points
        face = ((np.abs(P[:, 1] - 20.0) > 4.0) & (np.abs(P[:, 2]) < 1.0)
                & (P[:, 0] > 6) & (P[:, 0] < 34) & (P[:, 1] > 6) & (P[:, 1] < 34))
        out[mode] = (len(c), float(near.mean()), int(r.crease[face].sum()), int(face.sum()))

    for mode, (n_c, _, _, _) in out.items():
        assert n_c > 100, (mode, out)                  # (a) both find the crease
    # (b) the hierarchical split classifies NO face point as a crease point...
    assert out["hierarchical"][2] == 0, out
    # ...and is at least as pure as the flat one, which is the direction the fix claims.
    assert out["hierarchical"][1] >= out["flat"][1], out


def test_the_corner_layer_is_reachable_and_honest_about_why_it_is_off():
    """T-ASE Fig. 9's third level, and the precondition this dataset does not meet.

    §III-B2 builds the entropy LOBB *"on the extracted edge LINE point cloud"* — a curve.
    Our crease layer returns a band several millimetres wide, so at a 2,5 mm LOBB radius
    nearly every edge neighbourhood is isotropic and the low-entropy (corner) cluster
    swallows the seam. Hence `corner_layer=False` by default; the flag stays reachable and
    the docstring carries the TODO (thin the edge cloud to a curve first). This pins the
    wiring — the mask exists, is confined to the edge cloud, and cuts the fit — so the
    experiment is one kwarg away.
    """
    pts, oid = tee()
    r = detect(pts, object_id=oid, voxel_mm=None, corner_layer=True)
    assert r.corner.shape == r.crease.shape
    assert np.all(r.corner <= (r.crease | r.boundary))  # corners live in the edge cloud
    off = detect(pts, object_id=oid, voxel_mm=None)
    assert not off.corner.any() and off.params["corner_layer"] is False


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
    # RCIM §4.1/§4.3 interpolate the fitted curve to 80 points before error analysis, so
    # that is the default sampling — it was 40 until the fidelity audit.
    assert N_SAMPLES == 80
    assert len(fit_dominant_axis(noisy, degree=3)) == 80


# --- end to end -----------------------------------------------------------------------------

def test_reproduces_the_papers_accuracy_on_a_corner_joint():
    """RCIM reports **max error < 1,2 mm and RMSE < 0,7 mm**, and this now meets both.

    A corner joint is the case their pipeline is posed for: one open contact run, so the
    curve fit at the end has a single seam to fit. See the T-joint test for what happens
    when it does not.

    The bound used to be 1,5 mm — a third of the way to the published claim — and the
    fidelity audit is why it can be tightened onto the paper's own number: with the T-ASE
    Fig. 9 hierarchy restored and the RCIM eq. 1-2 window read as a resolution-relative
    ball, this scene goes from RMSE 0,428 / max 4,321 mm to **RMSE 0,112 / max 0,416 mm**.
    The old behaviour is still one call away (`kmeans="flat", edge_radius_mm=3.0`), and it
    does NOT meet the published claim: its max error is 3,6x the paper's 1,2 mm.
    """
    from baselines.metrics import matched_path_errors

    hit = _scene("corner")
    if hit is None:
        pytest.skip("no corner scene in out/bench")
    cloud, gt = hit
    r = detect(cloud["xyz"], object_id=cloud["object_id"], voxel_mm=1.0)
    rows = [e for e in matched_path_errors(r.polylines, gt) if e["matched"]]
    assert rows, r.note
    best = min(rows, key=lambda e: e["rmse"])
    assert best["rmse"] < 0.7, [round(e["rmse"], 3) for e in rows]      # the published RMSE
    assert best["me"] < 1.2, [round(e["me"], 3) for e in rows]          # the published max


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
