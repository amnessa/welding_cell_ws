"""Baseline A (ours) — the migrated radius-PCA, checked against known geometry.

These are not accuracy claims. They pin the things the migration could silently get wrong:
the unit change from metres to millimetres, the validity window, and the fact that the
cross-object gate is an oracle rather than something a sensor provides.
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from baselines import (cloud_for, detect, evaluate, ground_truth,  # noqa: E402
                       load_scene, scene_facts)
from baselines.metrics import (chamfer_mm, densify,  # noqa: E402
                               point_segment_distance, precision_recall_f1)
from baselines.radius_pca import (mean_spacing_mm, surface_variation,  # noqa: E402
                                  validity_window_mm)


def fold(n: int = 45, size: float = 50.0):
    """Two 50 mm planes meeting at 90 deg along the x axis, with per-plane object ids.

    Zero-thickness planes on purpose, for the tests that are about the fold itself. Where a
    test is about a plate's own *border* firing, use `slabs()` instead - a plane's boundary
    is still planar, so it produces no fold and the synthetic case cannot show the effect.
    """
    a = np.stack(np.meshgrid(np.linspace(0, size, n), np.linspace(0, size, n)), -1)
    a = a.reshape(-1, 2)
    p1 = np.column_stack([a[:, 0], a[:, 1], np.zeros(len(a))])
    p2 = np.column_stack([a[:, 0], np.zeros(len(a)), a[:, 1]])
    pts = np.vstack([p1, p2])
    oid = np.r_[np.zeros(len(p1), int), np.ones(len(p2), int)]
    return pts, oid


def slabs(joint_type: str = "T", density: float = 1.0, **kw):
    """A real sampled assembly from the generator - plates with thickness, gap and ids."""
    sys.path.insert(0, str(ROOT))
    from weldgen.joints import JointSpec
    from weldgen.layouts import build
    from weldgen.sampling import sample_scene_surface

    p = dict(L_A=200.0, W_A=100.0, t_A=8.0, L_B=200.0, H_B=80.0, t_B=8.0,
             root_gap_mm=1.0, linear_misalignment_mm=0.0, angular_misalignment_deg=0.0,
             included_angle_deg=JointSpec.NOMINAL_INCLUDED_DEG[joint_type],
             stack_offset_mm={"lap": 40.0, "edge": 0.0}.get(joint_type))
    p.update(kw)
    parts = build(JointSpec(**p), joint_type, np.eye(4))
    cloud = sample_scene_surface(parts, density, np.random.default_rng(0))
    return cloud["xyz"].astype(float), cloud["object_id"].astype(int), parts


# --- the migrated core ----------------------------------------------------------------

def test_surface_variation_reproduces_the_published_signature():
    """README §8: flat faces give V ~ 0, a 90 deg fold gives 0,05 - 0,15."""
    pts, _ = fold()
    v = surface_variation(pts, 4.0, 5)
    on_fold = (np.abs(pts[:, 1]) < 1.5) & (np.abs(pts[:, 2]) < 1.5)
    assert 0.05 <= float(v[on_fold].max()) <= 0.20
    assert float(np.median(v[~on_fold])) < 0.01


def test_radius_search_is_what_makes_it_work():
    """The parts do not touch. A ball narrower than the gap sees one plane and reads flat.

    This is the substitution README §8 calls "the one that makes the whole method work",
    so it is the one worth pinning: widen the gap past R and the signal disappears.
    """
    pts, oid, _ = slabs("T", density=1.0, root_gap_mm=4.0, t_A=12.0, t_B=12.0)
    on_joint = np.abs(pts[:, 2]) < 6.0                 # near the fillet, either plate

    narrow = surface_variation(pts[on_joint], 2.0, 5)  # R below the 4 mm gap
    wide = surface_variation(pts[on_joint], 7.0, 5)    # R above it
    assert float(np.mean(narrow > 0.03)) < float(np.mean(wide > 0.03)), (
        "a ball narrower than the gap must see one plate and read flat")


def test_the_validity_window_closes_on_thin_sheet():
    """`gap + spacing < R < thickness`. On 1 mm sheet with a 1 mm gap there is no R.

    That is a result about the method class, not a configuration error - which is why the
    window is returned rather than asserted.
    """
    lo, hi = validity_window_mm(1.1, 2.7, 8.4)         # the repo's measured T-joint
    assert lo < hi and lo == pytest.approx(3.8) and hi == pytest.approx(8.4)
    lo, hi = validity_window_mm(1.0, 1.0, 1.5)         # thin sheet
    assert lo >= hi, "the window must be empty, not merely narrow"


def test_lengths_are_millimetres():
    """The node this came from works in metres. A metre-scale radius must find nothing.

    Cheap, and it catches the single most plausible migration bug: a constant that kept
    its value and changed its meaning by 1000x.
    """
    pts, oid = fold()
    assert detect(pts, 0.006, object_id=oid).n_seams == 0, "6 mm written as 0.006"
    assert detect(pts, 6.0, object_id=oid).n_seams >= 1


# --- the upgrades ---------------------------------------------------------------------

def test_cross_object_is_reported_as_an_oracle():
    """Per-point object membership comes from registered CAD. No sensor provides it.

    Using it is fine - it is what the published method does - but a number produced with
    it must never be quoted as point-cloud-only, so the result says which arm it came from.
    """
    pts, oid = fold()
    assert detect(pts, 4.0, object_id=oid, cross_object=True).used_object_oracle
    assert not detect(pts, 4.0, object_id=oid, cross_object=False).used_object_oracle
    assert not detect(pts, 4.0, object_id=None).used_object_oracle


def test_a_parts_own_border_fires_without_the_cross_object_gate():
    """README §8: at R = 6 mm the outer borders were half the high-V points.

    The gate is not a refinement; without it the answer is a blob rather than a seam.
    """
    pts, oid, _ = slabs("T", density=0.6)
    with_gate = detect(pts, 5.0, object_id=oid, cross_object=True)
    without = detect(pts, 5.0, object_id=oid, cross_object=False)
    assert len(without.band) > len(with_gate.band), (
        "each plate's own border is a 90 deg fold too, and fires without the gate")


def test_multi_seam_returns_two_components_not_one_blob():
    """Two seams 200 mm apart must not be fitted with one line through the middle."""
    a, oid_a = fold()
    b, oid_b = fold()
    b[:, 1] += 200.0
    pts = np.vstack([a, b])
    oid = np.r_[oid_a, oid_b + 2]
    r = detect(pts, 4.0, object_id=oid, link_mm=4.0)
    assert r.n_seams == 2, f"expected 2 seams, got {r.n_seams}"


def test_the_line_fit_narrows_the_band():
    """README §8's caveat: the published seam is a ~16 mm band, not a centreline."""
    pts, oid = fold()
    r = detect(pts, 4.0, object_id=oid)
    gt = [np.array([[0.0, 0.0, 0.0], [50.0, 0.0, 0.0]])]
    m = evaluate(r.seams, gt, band=r.band)
    assert m["band_width_mm"] <= m["band_width_raw_mm"] + 1e-9


# --- metrics --------------------------------------------------------------------------

def test_distance_is_to_the_segment_not_the_infinite_line():
    """A seam has ends. A prediction 100 mm past the end is 100 mm wrong, not 0."""
    p0, p1 = np.zeros(3), np.array([10.0, 0.0, 0.0])
    d = point_segment_distance(np.array([[110.0, 0.0, 0.0]]), p0, p1)
    assert float(d[0]) == pytest.approx(100.0)


def test_both_chamfer_directions_are_reported_because_they_fail_differently():
    """Three correct points out of a 200 mm seam score well one way and badly the other."""
    gt = [np.array([[0.0, 0.0, 0.0], [200.0, 0.0, 0.0]])]
    stub = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
    m = chamfer_mm(stub, gt)
    assert m["pred_to_gt"] < 0.01
    assert m["gt_to_pred"] > 40.0


def test_recall_is_measured_over_seam_length_not_stored_points():
    """Otherwise it depends on how densely the truth happened to be sampled."""
    gt = [np.array([[0.0, 0.0, 0.0], [100.0, 0.0, 0.0]])]
    half = densify([np.array([[0.0, 0.0, 0.0], [50.0, 0.0, 0.0]])], 1.0)
    r = precision_recall_f1(half, gt, tol_mm=1.0)
    assert r["recall"] == pytest.approx(0.5, abs=0.02)
    assert r["precision"] == pytest.approx(1.0)


def test_a_polyline_prediction_is_densified_before_scoring():
    """Two endpoints ARE the whole seam; scored as a point set they look like almost none."""
    gt = [np.array([[0.0, 0.0, 0.0], [200.0, 0.0, 0.0]])]
    m = evaluate([np.array([[0.0, 0.0, 0.0], [200.0, 0.0, 0.0]])], gt)
    assert m["recall"] == pytest.approx(1.0)
    assert m["chamfer"] < 1e-6


# --- against real generated scenes ----------------------------------------------------

@pytest.mark.skipif(not (ROOT / "out" / "phase3" / "index.jsonl").exists(),
                    reason="needs out/phase3; run `python -m weldgen generate`")
def test_runs_end_to_end_on_a_generated_scene():
    import json
    row = next(json.loads(ln) for ln in
               (ROOT / "out" / "phase3" / "index.jsonl").read_text().splitlines()
               if json.loads(ln)["emitted"])
    scene, arrays = load_scene(ROOT / "out" / "phase3" / row["scene_id"])
    facts = scene_facts(scene)
    cloud = cloud_for(scene, arrays, view="full")
    gt = ground_truth(scene, arrays)
    assert gt, "an emitted scene always has at least one primary seam"

    spacing = mean_spacing_mm(cloud["xyz"])
    lo, hi = validity_window_mm(facts["root_gap_mm"], spacing, facts["t_min_mm"])
    r = detect(cloud["xyz"], 0.5 * (lo + hi) if lo < hi else lo,
               object_id=cloud["object_id"])
    m = evaluate(r.seams, gt, band=r.band)
    assert set(m) >= {"chamfer", "precision", "recall", "f1", "seam_count_error"}


@pytest.mark.skipif(not (ROOT / "out" / "phase3" / "index.jsonl").exists(),
                    reason="needs out/phase3")
def test_the_single_view_arm_sees_strictly_less_than_the_full_one():
    """The two arms must be reported separately; this pins that they differ at all."""
    import json
    row = next(json.loads(ln) for ln in
               (ROOT / "out" / "phase3" / "index.jsonl").read_text().splitlines()
               if json.loads(ln)["emitted"])
    scene, arrays = load_scene(ROOT / "out" / "phase3" / row["scene_id"])
    assert len(cloud_for(scene, arrays, "single")["xyz"]) \
        < len(cloud_for(scene, arrays, "full")["xyz"])


def test_the_batched_variation_matches_the_original_point_by_point_form():
    """The migration replaced a Python loop with batched linear algebra.

    Same arithmetic, ~100x faster - which is the difference between a corpus sweep that
    finishes and one that does not. Worth pinning exactly, because an optimisation that
    quietly changes the number would invalidate every comparison against the repo's
    published behaviour.
    """
    from baselines.radius_pca import _surface_variation_slow

    pts = np.random.default_rng(0).random((900, 3)) * 30.0
    fast = surface_variation(pts, 4.0, 5)
    slow = _surface_variation_slow(pts, 4.0, 5)
    assert np.allclose(fast, slow, atol=1e-12)


def test_chunking_does_not_change_the_answer():
    pts = np.random.default_rng(1).random((1500, 3)) * 40.0
    assert np.allclose(surface_variation(pts, 5.0, 5, chunk=100),
                       surface_variation(pts, 5.0, 5, chunk=100_000), atol=1e-12)
