"""`lit-pcaslice` — Wang et al. 2026 (Welding in the World), checked against known geometry."""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from baselines import REGISTRY, balanced_corpus, prepare, run_matrix  # noqa: E402
from baselines.lit_pcaslice import (bspline_path, detect, msac_wtlsd_plane,  # noqa: E402
                                    pca_centerline, statistical_filter)


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
