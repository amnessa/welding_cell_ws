"""`lit-quadric` — Li, Wang & Wang 2026 (JMST 40(3):2039), checked against known geometry.

The method is in the comparison because it is the one whose surface model is a quadric,
so the curved families are inside its mechanism. The claims to pin: the three
intersection types it must express (plane∩plane, plane∩cylinder, cylinder∩cylinder)
come back EXACT on exact geometry; the flat/curved verdict follows §4.2's angle reading
and the paper's literal σ statistic would misclassify a plane (reading 1); the plane fit
is total least squares (reading 2); the published distance ordering folds a closed ring
and the chain arm does not (reading 5); a finite patch test keeps an extended plane from
inventing seams (reading 4); the far-side mirror is a full-view cost that single view
does not pay; the method is deterministic per seed and runs through the harness.
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT))

from baselines import REGISTRY, prepare, run_matrix  # noqa: E402
from baselines.lit_quadric import (detect, fit_plane, fit_quadric,  # noqa: E402
                                   order_points, surface_kind)
from baselines.lit_regiongrow import local_pca  # noqa: E402
from weldgen.config import load_config  # noqa: E402
from weldgen.scene import SceneRejected, generate_scene  # noqa: E402
from weldgen.scene_curved import generate_curved_scene  # noqa: E402
from weldgen.writer import write_scene  # noqa: E402


def grid(L=100.0, n=60):
    a = np.linspace(0.0, L, n)
    return np.stack(np.meshgrid(a, a), -1).reshape(-1, 2)


def t_fold():
    """Base plate top face (y in [-50, 50]) and a standing plate (z in [0, 100]); the
    seam is the x axis. Labels are the two surfaces = the two parts."""
    g = grid()
    A = np.column_stack([g[:, 0], g[:, 1] - 50, np.zeros(len(g))])
    B = np.column_stack([g[:, 0], np.zeros(len(g)), g[:, 1]])
    lab = np.r_[np.zeros(len(A), int), np.ones(len(B), int)]
    return np.vstack([A, B]), lab


def pipe_on_plate(r=30.0):
    th = np.linspace(0, 2 * np.pi, 240, endpoint=False)
    P = np.array([[r * np.cos(t), r * np.sin(t), z] for z in np.linspace(0, 60, 40) for t in th])
    g = grid()
    base = np.column_stack([g[:, 0] - 50, g[:, 1] - 50, np.zeros(len(g))])
    base = base[np.hypot(base[:, 0], base[:, 1]) > r + 1]
    lab = np.r_[np.zeros(len(base), int), np.ones(len(P), int)]
    return np.vstack([base, P]), lab


# --- §3.1 the verdict ---------------------------------------------------------------------

def test_flat_and_curved_verdicts_follow_the_angle_reading():
    pts, lab = pipe_on_plate()
    n_plate, _ = local_pca(pts[lab == 0], k=20)
    n_pipe, _ = local_pca(pts[lab == 1], k=20)
    assert surface_kind(n_plate)["kind"] == "flat"
    assert surface_kind(n_pipe)["kind"] == "curved"


def test_the_literal_sigma_statistic_would_call_a_plane_curved():
    """Reading 1: 'over 90 % within the standard deviation' is scale-free. On a noisy plane
    the share of deviation angles within one σ is far below 90 %, so the literal rule
    fails on the paper's own flat blade; the §4.2 angle rule is the workable reading."""
    rng = np.random.default_rng(0)
    g = grid()
    A = np.column_stack([g[:, 0], g[:, 1], np.zeros(len(g))]) + rng.normal(0, 0.05, (len(g), 3))
    n, _ = local_pca(A, k=20)
    v = surface_kind(n)
    assert v["kind"] == "flat" and v["within_angle"] > 0.95
    assert v["within_sigma"] < 0.90


# --- §3.2 the fits ------------------------------------------------------------------------

def test_plane_fit_is_total_least_squares():
    rng = np.random.default_rng(1)
    g = grid()
    n_true = np.array([1.0, 2.0, 3.0]); n_true /= np.linalg.norm(n_true)
    u, v = np.linalg.svd(np.eye(3) - np.outer(n_true, n_true))[0][:, :2].T
    pts = 5 * n_true + g[:, :1] * u + g[:, 1:] * v + rng.normal(0, 0.02, (len(g), 3))
    s = fit_plane(pts)
    assert abs(abs(float(s.normal @ n_true)) - 1.0) < 1e-4
    assert s.rms_residual_mm < 0.05


def test_quadric_fit_recovers_a_cylinder_exactly():
    pts, lab = pipe_on_plate()
    s = fit_quadric(pts[lab == 1])
    assert s.rms_residual_mm < 1e-6
    assert np.abs(s.distance(pts[lab == 1])).max() < 1e-4


# --- §3.3 the three intersections, exact -------------------------------------------------

def test_plane_plane_intersection_is_the_seam_line():
    pts, lab = t_fold()
    r = detect(pts, region_labels=lab, part_labels=lab, voxel_mm=None)
    assert r.n_seams == 1
    s = r.seams[0]
    assert np.abs(s[:, 1]).max() < 1e-6 and np.abs(s[:, 2]).max() < 1e-6
    assert s[:, 0].min() < 2.0 and s[:, 0].max() > 98.0


def test_plane_cylinder_intersection_is_the_circle():
    pts, lab = pipe_on_plate(r=30.0)
    r = detect(pts, region_labels=lab, part_labels=lab, voxel_mm=None, ordering="chain")
    assert r.n_seams == 1 and [x["kind"] for x in r.surfaces] == ["flat", "curved"]
    s = r.seams[0]
    assert np.abs(np.hypot(s[:, 0], s[:, 1]) - 30.0).max() < 1e-4
    assert np.abs(s[:, 2]).max() < 1e-4


def test_cylinder_cylinder_intersection_is_the_saddle():
    th = np.linspace(0, 2 * np.pi, 240, endpoint=False)
    big = np.array([[x, 40 * np.cos(t), 40 * np.sin(t)] for x in np.linspace(-80, 80, 80) for t in th])
    small = np.array([[20 * np.cos(t), 20 * np.sin(t), z] for z in np.linspace(35, 110, 50) for t in th])
    small = small[np.hypot(small[:, 1], small[:, 2]) > 40.5]
    lab = np.r_[np.zeros(len(big), int), np.ones(len(small), int)]
    r = detect(np.vstack([big, small]), region_labels=lab, part_labels=lab, voxel_mm=None,
               ordering="chain")
    assert r.n_seams == 1 and [x["kind"] for x in r.surfaces] == ["curved", "curved"]
    s = r.seams[0]
    assert np.abs(np.hypot(s[:, 0], s[:, 1]) - 20.0).max() < 1e-3
    assert np.abs(np.hypot(s[:, 1], s[:, 2]) - 40.0).max() < 1e-3


# --- the ordering (reading 5) ------------------------------------------------------------

def test_the_published_distance_ordering_folds_a_ring_and_the_chain_does_not():
    th = np.linspace(0, 2 * np.pi, 200, endpoint=False)
    ring = np.column_stack([30 * np.cos(th), 30 * np.sin(th), np.zeros(200)])
    L = lambda p: float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())
    assert L(order_points(ring, "chain")) < 1.02 * 2 * np.pi * 30
    assert L(order_points(ring, "distance")) > 10 * 2 * np.pi * 30


def test_the_distance_ordering_is_correct_on_a_straight_seam():
    line = np.column_stack([np.linspace(0, 100, 80), np.zeros(80), np.zeros(80)])
    rng = np.random.default_rng(0)
    shuffled = line[rng.permutation(80)]
    o = order_points(shuffled, "distance")
    assert np.all(np.diff(o[:, 0]) > 0) or np.all(np.diff(o[:, 0]) < 0)


# --- the finite patch (reading 4) --------------------------------------------------------

def test_an_extended_plane_does_not_invent_a_seam_on_a_far_face():
    """A standing plate's side plane, extended, crosses the base plate's far side face.
    Without the patch test that crossing is a 'seam'; with it the pair has no band."""
    pts, lab = t_fold()
    g = grid(100.0, 60)
    far = np.column_stack([g[:, 0], np.full(len(g), -50.0), -g[:, 1] * 0.2])   # A's far side face
    pts = np.vstack([pts, far]); lab = np.r_[lab, np.full(len(far), 2)]
    parts = np.where(lab == 1, 1, 0)
    r = detect(pts, region_labels=lab, part_labels=parts, voxel_mm=None)
    assert r.n_seams == 1                                    # the fold only
    status = {(p["i"], p["j"]): p["status"] for p in r.pairs}
    assert status[(1, 2)] in ("no_band", "too_few", "too_short")


# --- through the harness -----------------------------------------------------------------

def _scene(tmp_path, gen, cfg, tag):
    for seed in range(30):
        try:
            scene, arrays = gen(cfg, seed)
            break
        except SceneRejected:
            continue
    else:
        pytest.fail(f"no emitting seed for {tag}")
    write_scene(tmp_path, scene, arrays)
    return prepare([tmp_path / scene["scene_id"]])


def test_full_view_pays_the_far_side_mirror_and_single_view_does_not(tmp_path):
    cfg = load_config(str(ROOT / "configs" / "bench6a_T.yaml"))
    prep = _scene(tmp_path, generate_scene, cfg, "T")
    full = run_matrix(prep, methods=["lit-quadric"], seeds=[0], verify_seeds=1,
                      view="full_exterior").iloc[0]
    single = run_matrix(prep, methods=["lit-quadric"], seeds=[0], verify_seeds=1,
                        view="single").iloc[0]
    assert full.recall > 0.8                                # both fillets found
    assert single.precision >= full.precision - 1e-9         # no mirror to pay for


def test_finds_the_ring_on_a_pipe_on_plate_scene(tmp_path):
    cfg = load_config(str(ROOT / "configs" / "curved_smoke.yaml"))
    cfg["seam_families"] = [2]
    prep = _scene(tmp_path, generate_curved_scene, cfg, "circle")
    row = run_matrix(prep, methods=["lit-quadric"], seeds=[0], verify_seeds=1,
                     view="single", method_kw={"lit-quadric": {"ordering": "chain"}}).iloc[0]
    # recall is bounded by the ring's visible fraction from one camera (~0,25-0,5 on a
    # pipe): the claims are that the tube reads as CURVED, that what is found is ON the
    # ring, and that it is a substantial arc - not that an invisible arc was recovered
    assert row.n_curved >= 1
    assert row.precision > 0.8 and row.rmse_med < 0.5
    vis = max(s["visible_fraction"] for s in prep[0].scene["seams"] if s["weldable"])
    assert row.recall > 0.4 * vis


def test_deterministic_and_seed_invariant_through_the_registry():
    """Reading 6: the published random start only flips the traversal direction, and the
    harness demands an exact zero spread from a deterministic method - so the default
    start is canonical and two seeds give identical output; the random start is an arm."""
    assert REGISTRY["lit-quadric"].randomised is False
    pts, lab = t_fold()
    a = detect(pts, region_labels=lab, part_labels=lab, voxel_mm=None, seed=3).seams[0]
    b = detect(pts, region_labels=lab, part_labels=lab, voxel_mm=None, seed=11).seams[0]
    assert np.array_equal(a, b)
    ring = np.column_stack([30 * np.cos(np.linspace(0, 2 * np.pi, 200, endpoint=False)),
                            30 * np.sin(np.linspace(0, 2 * np.pi, 200, endpoint=False)), np.zeros(200)])
    r1 = order_points(ring, "chain", seed=1, start="random"); r2 = order_points(ring, "chain", seed=2, start="random")
    assert not np.array_equal(r1, r2)                        # the arm is seed-dependent
