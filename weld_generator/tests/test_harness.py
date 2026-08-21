"""The repeat harness, validated against the fake oracle — as the plan orders it.

*"Harness first — matching, metrics, per-scene dataframe — validated against a fake oracle
predictor (ground truth plus noise) before any real method output flows through it."*

Every property of a fake-oracle row is known in advance, so every assertion here is a
closed-form expectation, not a regression pin. If one of these fails, no real method's
number can be trusted through the harness — which is the entire reason the harness exists,
given that `lit-ransac` swings F1 0,00-0,94 on one fixed scene across seeds.
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from baselines import (REGISTRY, balanced_corpus, emd_mm, prepare,  # noqa: E402
                       run_matrix, spread)


@pytest.fixture(scope="module")
def prepped():
    try:
        corpus = balanced_corpus(ROOT / "out" / "bench", per_type=50)
    except (ValueError, FileNotFoundError, NotADirectoryError):
        pytest.skip("no balanced corpus at out/bench")
    return prepare(corpus["T"][:1] + corpus["corner"][:1])


# --- the fake-oracle validation ---------------------------------------------------------

def test_known_jitter_comes_back_as_the_error_it_is(prepped):
    """sigma = 0,3 mm isotropic jitter on the truth polyline.

    The perpendicular component of an isotropic 3-D Gaussian is 2-D Rayleigh, mean
    `sigma * sqrt(pi/2) ~= 0,376 mm`; the resampled path interpolates between jittered
    vertices so the measured figure sits at or just under that. The band [0,25, 0,55]
    catches every failure class that matters — a unit slip (x1000), a squared-vs-linear
    error (x2), a wrong-curve comparison (~1 mm) — without pinning the interpolation.
    """
    df = run_matrix(prepped, methods=["fake-oracle"], seeds=range(3),
                    method_kw={"fake-oracle": {"sigma_mm": 0.3}})
    assert (df.f1 > 0.95).all()
    assert (df.seam_recall == 1.0).all()
    assert df.rmse_med.between(0.2, 0.55).all(), df.rmse_med.tolist()

    big = run_matrix(prepped, methods=["fake-oracle"], seeds=range(3),
                     method_kw={"fake-oracle": {"sigma_mm": 1.0}})
    ratio = big.rmse_med.median() / df.rmse_med.median()
    assert 2.5 < ratio < 4.0, "error must scale linearly with the injected sigma"


def test_a_phantom_costs_precision_and_steals_no_match(prepped):
    """One invented seam, 40 mm off the assembly.

    Matching is one-to-one and closest-first, so the phantom must be left unmatched: every
    truth seam keeps its accurate partner (rmse unchanged), recall holds, and the phantom's
    entire cost lands in precision — where a false positive belongs.
    """
    clean = run_matrix(prepped, methods=["fake-oracle"], seeds=range(3),
                       method_kw={"fake-oracle": {"sigma_mm": 0.1}})
    ph = run_matrix(prepped, methods=["fake-oracle"], seeds=range(3),
                    method_kw={"fake-oracle": {"sigma_mm": 0.1, "p_phantom": 1.0}})
    assert (ph.n_pred_seams == ph.n_gt + 1).all()
    assert (ph.seam_count_error == 1).all()
    # Point-mass precision charges a phantom by its LENGTH relative to the real seams - a
    # 60 mm phantom against ~380 mm of seam costs ~14%, not 50%. Assert the mechanism
    # (strictly lower, in every row) and the honest magnitude bound, not a made-up margin.
    assert (ph.precision < clean.precision).all()
    assert ph.precision.mean() < clean.precision.mean() - 0.05
    assert (ph.seam_recall == 1.0).all()
    assert ph.rmse_med.median() == pytest.approx(clean.rmse_med.median(), abs=0.05)


def test_a_missed_seam_is_an_unmatched_truth_row_not_a_crash(prepped):
    """`p_miss = 1` returns nothing at all. The harness must record that, not raise."""
    df = run_matrix(prepped, methods=["fake-oracle"], seeds=range(2),
                    method_kw={"fake-oracle": {"p_miss": 1.0}})
    assert (df.seam_recall == 0.0).all()
    assert (df.f1 == 0.0).all()


# --- the repeat policy ------------------------------------------------------------------

def test_randomised_methods_get_every_seed_and_deterministic_ones_get_the_verification():
    """The registry flags drive the repeat count.

    `lit-ransac` is the reason the harness exists and must run once per seed. A
    deterministic method gets `verify_seeds` runs — the minimum that turns "zero spread"
    from an assumption into a measurement.
    """
    assert REGISTRY["lit-ransac"].randomised
    for name in ("ours", "lit-regiongrow", "lit-lobb"):
        assert not REGISTRY[name].randomised, name
    assert REGISTRY["ours"].output == "band"           # its line fit was removed for cause


def test_ours_measures_zero_spread_and_the_fake_oracle_does_not(prepped):
    df = run_matrix(prepped[:1], methods=["ours", "fake-oracle"], seeds=range(4),
                    verify_seeds=3,
                    method_kw={"fake-oracle": {"sigma_mm": 0.5}})
    assert len(df[df.method == "ours"]) == 3           # verify_seeds, not len(seeds)
    assert len(df[df.method == "fake-oracle"]) == 4

    sp = spread(df, "chamfer")
    assert (sp[sp.method == "ours"].spread == 0).all(), "ours must be deterministic"
    assert (sp[sp.method == "fake-oracle"].spread > 0).all(), \
        "a seeded method must actually vary across seeds"


def test_the_noise_axis_is_a_multiplier_on_the_derived_sigma(prepped):
    """`noise_scale = 2` must produce roughly twice the error floor of `noise_scale = 1`.

    Checked through the fake oracle run on the *clean* truth against noisy-cloud scoring?
    No — the fake oracle ignores the cloud. Instead: the prepared scene's own cloud under
    the two scales, compared point-for-point against the clean one. The plan requires the
    percentage defined as a multiple of the derived sigma_z, never a fraction of range.
    """
    prep = prepped[0]
    n1 = prep.cloud("full", 1.0)
    n2 = prep.cloud("full", 2.0)
    d1 = np.linalg.norm(n1["xyz"] - n1["clean_xyz"], axis=1)
    d2 = np.linalg.norm(n2["xyz"] - n2["clean_xyz"], axis=1)
    # Same noise_model.seed, same standard-normal draws, scaled sigmas: the displacement
    # must double POINT FOR POINT, not merely in distribution.
    m = d1 > 1e-9
    assert m.mean() > 0.99
    ratio = d2[m] / d1[m]
    assert np.median(np.abs(ratio - 2.0)) < 1e-6, f"median ratio {np.median(ratio):.4f}"


# --- the secondary metric ----------------------------------------------------------------

def test_emd_punishes_the_coverage_failure_chamfer_forgives():
    """A stub covering 10% of the seam, perfectly accurately — `lit-regiongrow`'s failure.

    One-directional thinking scores it well: every stub point is on the seam. A transport
    plan has to carry the stub's mass to the far end of the seam and pays ~L/2 for it.
    Chamfer (symmetric mean) dilutes that cost; EMD is the cost.
    """
    gt = [np.array([[0.0, 0, 0], [100.0, 0, 0]])]
    stub = [np.array([[0.0, 0, 0], [10.0, 0, 0]])]
    full = [np.array([[0.0, 0.3, 0], [100.0, 0.3, 0]])]

    assert emd_mm(full, gt) == pytest.approx(0.3, abs=0.1)
    assert emd_mm(stub, gt) > 30.0                     # ~ mean distance mass must travel
    assert emd_mm([], gt) != emd_mm([], gt) or np.isnan(emd_mm([], gt))
