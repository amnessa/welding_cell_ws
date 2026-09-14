"""`lit-pcaslice` — Wang et al. 2026 (Welding in the World), checked against known geometry."""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from baselines import REGISTRY, balanced_corpus, prepare, run_matrix  # noqa: E402
from baselines.lit_pcaslice import (band_is_closed, bspline_path, detect,  # noqa: E402
                                    msac_wtlsd_plane, pca_centerline,
                                    statistical_filter, torch_poses)


def strip(n: int = 3000, length: float = 100.0, width: float = 6.0, seed: int = 0,
          curve: float = 0.0) -> np.ndarray:
    """A strip-like weld cloud: the §4.2 input. `curve` bends it into an arc."""
    rng = np.random.default_rng(seed)
    t = rng.uniform(0, length, n)
    return np.column_stack([t, curve * (t - length / 2) ** 2 / length
                            + rng.uniform(-width / 2, width / 2, n),
                            rng.normal(0, 0.2, n)])


def test_the_slicing_direction_comes_from_the_data_not_from_an_axis():
    """The paper's claim against projection methods: PCA finds the direction itself.

    The same strip, arbitrarily rotated, must give the same centreline (rotated) — no
    axis was ever chosen by hand. Centres are compared in the strip's own frame.
    """
    s0 = strip()
    c0 = pca_centerline(s0, slice_mm=5.0)
    a = np.radians(37.0)
    R = np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])
    c1 = pca_centerline(s0 @ R.T, slice_mm=5.0) @ R
    assert len(c0) == len(c1)
    assert np.abs(np.sort(c0[:, 0]) - np.sort(c1[:, 0])).max() < 1.0
    assert np.abs(c0[:, 1]).max() < 1.0                # centres on the spine


def test_per_slice_centres_follow_a_curved_strip():
    """The 'adaptive' half of the claim, and the reason the plan wants this method at
    Phase 6: on a curved strip the slice centres track the bend."""
    s = strip(curve=0.3, width=4.0)
    path = bspline_path(pca_centerline(s, slice_mm=5.0), n_samples=50)
    t = path[:, 0]
    expect = 0.3 * (t - 50.0) ** 2 / 100.0
    assert np.abs(path[:, 1] - expect).max() < 1.5


def test_two_seams_in_one_instance_give_the_midpoint_of_neither():
    """The mechanism pinned: a slice's GEOMETRIC CENTRE cannot represent two seams.

    Two parallel strips 8 mm apart in one mask: every slice centre lands mid-plate,
    ~4 mm from both — the same mid-surface failure that made `ours` delete its line fit.
    This is why the coarse stage being per-INSTANCE (their YOLO boxes each weld
    separately) is structural rather than a convenience, and why the L1 arm must be read
    as a broken-assumption arm.
    """
    a = strip(width=2.0)
    b = strip(width=2.0, seed=1)
    b[:, 1] += 8.0
    both = np.vstack([a, b])
    centers = pca_centerline(both, slice_mm=5.0)
    assert np.abs(centers[:, 1] - 4.0).max() < 1.5     # mid-plate, on neither seam

    r2 = detect(both, instance_masks=[np.r_[np.ones(len(a), bool), np.zeros(len(b), bool)],
                                      np.r_[np.zeros(len(a), bool), np.ones(len(b), bool)]])
    assert r2.n_seams == 2
    ys = sorted(float(np.median(p[:, 1])) for p in r2.seams)
    assert abs(ys[0] - 0.0) < 1.0 and abs(ys[1] - 8.0) < 1.0


def test_statistical_filter_removes_the_sparse_outliers_the_paper_filters():
    s = strip()
    spiked = np.vstack([s, [[50.0, 40.0, 30.0], [10.0, -35.0, 25.0]]])
    out = statistical_filter(spiked, k=12, std_mul=2.0)
    assert len(out) < len(spiked)
    assert np.abs(out[:, 2]).max() < 5.0


def test_msac_feeds_the_posture_only_and_the_path_is_deterministic():
    """Where the randomness lives, pinned to the registry flag.

    MSAC is sampling-based, but it plans the torch POSTURE (§4.4), which Phase 4 does not
    score. The path (§4.1-4.3) must be bit-identical run to run, and the registry must say
    deterministic so the harness's repeat policy is right.
    """
    s = strip()
    a = detect(s)
    b = detect(s)
    assert [p.tolist() for p in a.seams] == [p.tolist() for p in b.seams]
    assert not REGISTRY["lit-pcaslice"].randomised

    plane = msac_wtlsd_plane(strip(width=20.0)[:, :3], seed=3)
    assert plane is not None
    n, _ = plane
    assert abs(n[2]) > 0.99                            # the strip's plane is z ~ 0


def test_runs_through_the_harness_with_per_instance_oracles():
    try:
        corpus = balanced_corpus(ROOT / "out" / "bench", per_type=50)
    except (ValueError, FileNotFoundError, NotADirectoryError):
        pytest.skip("no balanced corpus at out/bench")
    prep = prepare(corpus["T"][:1])
    df = run_matrix(prep, methods=["lit-pcaslice"], seeds=range(4), verify_seeds=2)
    assert len(df) == 2
    assert df.f1.nunique() == 1
    assert (df.n_instances == df.n_gt).all()           # one YOLO box per weld instance


# --------------------------------------------------------------------------------
# closed seams — the input their four camera poses never produce in one piece
# --------------------------------------------------------------------------------

def ring(radius: float = 60.0, width: float = 6.0, density: float = 1.0,
         noise: float = 0.1, seed: int = 0) -> np.ndarray:
    """A CLOSED circular weld band: radius 60 mm, ~1 pt/mm^2, 0,1 mm normal noise."""
    rng = np.random.default_rng(seed)
    n = int(2 * np.pi * radius * width * density)
    th = rng.uniform(0.0, 2 * np.pi, n)
    r = radius + rng.uniform(-width / 2, width / 2, n)
    return np.column_stack([r * np.cos(th), r * np.sin(th), rng.normal(0, noise, n)])


def _radial_error(path: np.ndarray, radius: float = 60.0) -> np.ndarray:
    return np.abs(np.hypot(path[:, 0], path[:, 1]) - radius)


def test_a_closed_ring_is_sliced_per_ARC_and_comes_back_closed_and_on_the_seam():
    """§4.2 never sees a ring: each camera pose hands it one open crescent (Figs. 7/8c/11).

    Recreating that — four overlapping sectors, slice each, stitch the centres, fit one
    PERIODIC cubic (Fig. 13b) — puts the path back on the circle to well inside the
    paper's 1 mm claim and closes it. The whole point is that the mechanism is fine; it
    is the *input* that has to be an arc.
    """
    from baselines.metrics import distance_to_polylines

    P = ring()
    assert detect(P).closed_flags == [True]            # auto-detected, nothing passed
    r = detect(P)
    assert r.n_seams == 1
    path = r.seams[0]

    assert np.linalg.norm(path[0] - path[-1]) < 1e-9   # a closed polyline, no seam gap
    th = np.degrees(np.arctan2(path[:, 1], path[:, 0]))
    assert len(set((th // 10).astype(int))) == 36      # every 10-degree bin, full turn

    e = _radial_error(path)
    assert float(np.sqrt((e ** 2).mean())) < 0.5       # RMSE, against "within 1 mm"
    assert float(np.abs(path[:, 2]).max()) < 0.5

    truth = np.column_stack([60 * np.cos(np.radians(np.arange(360))),
                             60 * np.sin(np.radians(np.arange(360))),
                             np.zeros(360)])
    recall = float((distance_to_polylines(truth, [path]) <= 1.0).mean())
    precision = float((e <= 1.0).mean())
    assert 2 * recall * precision / (recall + precision) > 0.9


def test_the_closed_kwarg_wins_over_the_auto_detection():
    """A caller holding the seam's own `closed` flag must not have it re-inferred.

    Passing the flag reproduces the auto-detected result exactly; passing the WRONG flag
    changes the answer, which is what "the kwarg wins" has to mean to be worth anything.
    """
    P = ring()
    auto, told = detect(P), detect(P, closed=True)
    assert told.params["closed_given"] and not auto.params["closed_given"]
    assert np.allclose(auto.seams[0], told.seams[0])

    forced_open = detect(P, closed=False)
    assert forced_open.closed_flags == [False]
    assert np.median(_radial_error(forced_open.seams[0])) > 10.0

    both = detect(np.vstack([P, ring(seed=1) + [300.0, 0, 0]]),
                  instance_masks=[np.r_[np.ones(len(P), bool), np.zeros(len(P), bool)],
                                  np.r_[np.zeros(len(P), bool), np.ones(len(P), bool)]],
                  closed=[True, True])
    assert both.closed_flags == [True, True]


def test_b_ring_mode_whole_reproduces_the_mid_air_centres_it_is_kept_to_price():
    """The ablation rung, pinned to its failure so the arm stays interpretable.

    Slice a ring in one piece and the PCA axis lies IN the ring's plane: every slice cuts
    the ring twice and the geometric centre of the pair is a point in the hole, ~30 mm off
    a 60 mm circle at the middle of the sweep. Same mid-surface failure as two seams in
    one mask — one seam that closes is enough.
    """
    P = ring()
    r = detect(P, ring_mode="whole")
    assert r.closed_flags == [False]                   # not treated as a ring by §4.2
    centers = r.centers[0]
    assert float(np.median(_radial_error(centers))) > 10.0
    assert float(np.median(np.hypot(centers[:, 0], centers[:, 1]))) < 40.0  # inside the hole
    assert float(np.median(_radial_error(r.seams[0]))) > 10.0
    assert np.median(_radial_error(detect(P).seams[0])) < 1.0   # what "arcs" fixes


def test_c_an_open_straight_band_is_untouched_by_any_of_this():
    """Open bands must take exactly the path they took before: no ring, no arcs, no
    periodic fit — the default `ring_mode="arcs"` may only fire on a closed band."""
    s = strip()
    assert not band_is_closed(s)
    r = detect(s)
    assert r.closed_flags == [False]
    path = r.seams[0]
    assert float(np.abs(path[:, 1]).max()) < 1.0       # the line, on the spine
    assert float(np.abs(path[:, 2]).max()) < 0.5
    assert np.linalg.norm(path[0] - path[-1]) > 90.0   # open: ends far apart
    assert np.allclose(path, detect(s, ring_mode="whole").seams[0])
    assert np.allclose(path, detect(s, closed=False).seams[0])


def test_d_bspline_path_closed_fits_the_periodic_curve_of_fig_13b():
    th = np.linspace(0, 2 * np.pi, 24, endpoint=False)
    centers = np.column_stack([60 * np.cos(th), 60 * np.sin(th), np.zeros(24)])

    closed = bspline_path(centers, n_samples=80, closed=True)
    assert len(closed) == 81                           # endpoint excluded, then wrapped
    assert np.linalg.norm(closed[0] - closed[-1]) < 1e-9
    # 1,5 mm, not 1 mm: `s = 0,25` per centre lets the SMOOTHING spline breathe ~0,5 mm
    # RMS even on exact input, which is the module's published-free choice of s and not
    # something the periodic fit changes. What this test is for is the closure.
    assert float(np.abs(np.hypot(closed[:, 0], closed[:, 1]) - 60.0).max()) < 1.5
    assert len(set((np.degrees(np.arctan2(closed[:, 1], closed[:, 0])) // 10)
                   .astype(int))) == 36

    opened = bspline_path(centers, n_samples=80)
    assert np.linalg.norm(opened[0] - opened[-1]) > 10.0   # the gap the periodic fit closes


def test_the_torch_frame_follows_eq_12_including_its_signs():
    """Eq. (12): Y_W = tangent, Z_W = -(n_b + n_m), X_W = Z_W x Y_W.

    Built on a 90-degree fillet — two half-planes meeting along x — the torch axis must
    point INTO the dihedral (up and out of the corner is the OUTWARD bisector; the paper's
    minus sign puts Z on the other side), and X must be the cross product in the paper's
    order, not the reverse.
    """
    rng = np.random.default_rng(0)
    n = 20000
    a = np.column_stack([rng.uniform(0, 60, n), rng.uniform(0, 20, n), np.zeros(n)])
    b = np.column_stack([rng.uniform(0, 60, n), np.zeros(n), rng.uniform(0, 20, n)])
    cloud = np.vstack([a, b])
    path = np.column_stack([np.linspace(10, 50, 12), np.zeros(12), np.zeros(12)])

    frames = torch_poses(path, cloud, radius_mm=6.0, seed=0)
    outward = np.array([0.0, 1.0, 1.0]) / np.sqrt(2)   # the bisector away from the corner
    for F in frames:
        x, y, z = F[:, 0], F[:, 1], F[:, 2]
        zy = np.cross(z, y)
        assert np.allclose(x, zy / np.linalg.norm(zy), atol=1e-6)   # X_W = Z_W x Y_W
        assert abs(abs(y @ np.array([1.0, 0, 0])) - 1.0) < 1e-6
        assert z @ outward < 0.0                              # Z_W = -(n_b + n_m)
