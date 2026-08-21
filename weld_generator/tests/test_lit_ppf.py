"""`lit-ppf` — Wang et al. 2024 (Sci. Rep. 14:21137), checked against known geometry.

Beyond the usual reimplementation traps, this file pins the two claims about the paper
that change how its numbers must be read: **as published the pipeline is deterministic**
(the plan's "randomised" grouping came from its RANSAC-alternative framing, not from the
method), and **it is the first method here that consumes normals**, which makes the
estimated-vs-exact-normals delta measurable for the first time.
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from baselines import REGISTRY, balanced_corpus, prepare, run_matrix  # noqa: E402
from baselines.lit_ppf import (SAMPLE_INTERVAL_MM, detect, opp_vote, ppf,  # noqa: E402
                               ppf_planes, rotation_to_z)
from baselines.lit_regiongrow import local_pca  # noqa: E402


def grid(size: float = 100.0, n: int = 70) -> np.ndarray:
    a = np.linspace(0.0, size, n)
    return np.stack(np.meshgrid(a, a), -1).reshape(-1, 2)


def fold(size: float = 100.0, n: int = 70):
    """Two zero-thickness planes meeting at 90 deg along the x axis at the origin."""
    a = grid(size, n)
    p1 = np.column_stack([a[:, 0], a[:, 1], np.zeros(len(a))])
    p2 = np.column_stack([a[:, 0], np.zeros(len(a)), a[:, 1]])
    return np.vstack([p1, p2])


# --- the descriptor ---------------------------------------------------------------------

def test_the_descriptor_reads_coplanarity_and_orthogonality():
    """The two readings the paper takes from F(p1, p2).

    Coplanar oriented points: both normals perpendicular to the offset, normals parallel.
    An orthogonal pair: the angle component at 90 deg. These are the only two facts the
    whole pipeline extracts from the descriptor, so they are the facts to pin.
    """
    z = np.array([0.0, 0, 1])
    y = np.array([0.0, 1, 0])
    d, a2, a3, a4 = ppf([0, 0, 0], z, [10, 0, 0], z)   # same plane
    assert d == pytest.approx(10.0)
    assert a2 == pytest.approx(90.0) and a3 == pytest.approx(90.0)
    assert a4 == pytest.approx(0.0)

    _, _, _, a4 = ppf([0, 0, 0], z, [10, 0, 10], y)    # orthogonal pair
    assert a4 == pytest.approx(90.0)


def test_rotation_to_z_puts_the_reference_normal_on_the_z_axis():
    """Eq. 21, for arbitrary normals — the step that makes the vote two-dimensional."""
    rng = np.random.default_rng(0)
    for _ in range(20):
        n = rng.normal(size=3)
        n /= np.linalg.norm(n)
        assert np.allclose(rotation_to_z(n) @ n, [0, 0, 1], atol=1e-9)


# --- stage 1 ----------------------------------------------------------------------------

def test_ppf_planes_recovers_the_two_faces_and_merges_duplicate_samples():
    """25 mm sampling puts ~16 samples on a 100 mm face, all growing the SAME plane.

    The paper never mentions merging; without it every downstream pair test runs a plane
    against copies of itself. Merged, a fold is two planes, axis-aligned.
    """
    pts = fold()
    normals, _ = local_pca(pts, k=20)
    planes = ppf_planes(pts, normals, SAMPLE_INTERVAL_MM)
    assert len(planes) == 2
    ns = np.abs(np.stack([p.normal for p in planes]))
    assert sorted(np.argmax(ns, axis=1).tolist()) == [1, 2]   # y-normal and z-normal


# --- stage 2 ----------------------------------------------------------------------------

def test_the_vote_accepts_the_fold_and_refuses_a_parallel_pair():
    """The OPP gate is the mechanism the coverage prediction is about.

    Two parallel sheets (an edge joint with no side walls) present no orthogonal pair at
    all: `opp_vote` must return `None` before a single vote is cast. That refusal — not a
    tuning failure downstream — is why the plan predicts `lit-ppf` cannot express the
    coplanar arm of D4.
    """
    pts = fold()
    normals, _ = local_pca(pts, k=20)
    a, b = ppf_planes(pts, normals, SAMPLE_INTERVAL_MM)
    v = opp_vote(pts, normals, a, b)
    assert v is not None and v[0] > 50

    two = np.vstack([fold()[: 70 * 70],                     # the z = 0 sheet, twice,
                     fold()[: 70 * 70] + [0, 0, 8.0]])      # 8 mm apart
    n2, _ = local_pca(two, k=20)
    planes = ppf_planes(two, n2, SAMPLE_INTERVAL_MM)
    assert len(planes) == 2
    assert opp_vote(two, n2, planes[0], planes[1]) is None

    r = detect(two)
    assert r.n_seams == 0
    assert all(p["status"] == "not_orthogonal" for p in r.pairs)


def test_end_to_end_on_a_fold_with_the_papers_own_endpoint_rule():
    """One 90-deg fold in, one seam out: the farthest-pair corners span the fold."""
    r = detect(fold(), voxel_mm=None)
    assert r.n_seams >= 1
    best = max(r.seams, key=lambda s: np.linalg.norm(s[-1] - s[0]))
    L = float(np.linalg.norm(best[-1] - best[0]))
    assert L == pytest.approx(100.0, abs=5.0)
    # and the line is the fold: both corners near y = z = 0
    assert np.abs(best[:, 1:]).max() < 4.0


def test_the_published_feature_threshold_is_for_a_50_micron_scanner():
    """`feature_tol_mm = 0,1` literally, on a ~1,4 mm-spacing cloud: nothing survives.

    Not a bug in the paper — their Photoneo scans at 50 um accuracy, ours sample at
    ~1 mm — but it is the constant most likely to be copied blindly, so the failure is
    pinned and the default scales with spacing instead.

    The fold is built OFFSET from the crease: an axis-aligned grid puts a row of points
    mathematically on the seam line, and 0,1 mm "survives" on those — an artifact no
    sampled or noisy cloud reproduces, which the first version of this test tripped over.
    """
    a = np.linspace(0.7, 100.7, 70)
    g = np.stack(np.meshgrid(a, a), -1).reshape(-1, 2)
    off_fold = np.vstack([np.column_stack([g[:, 0], g[:, 1], np.zeros(len(g))]),
                          np.column_stack([g[:, 0], np.zeros(len(g)), g[:, 1]])])
    r_lit = detect(off_fold, voxel_mm=None, feature_tol_mm=0.1)
    r_def = detect(off_fold, voxel_mm=None)
    assert r_def.n_seams > 0
    assert r_lit.n_seams == 0


# --- what the plan assumed, corrected ---------------------------------------------------

def test_as_published_the_method_is_deterministic():
    """Grid sampling, Hough voting, DBSCAN, farthest-pair corners: no RNG anywhere.

    `dataset_plan.md` grouped `lit-ppf` with `lit-ransac` as randomised, on the strength
    of its RANSAC-alternative framing. Reading the paper corrects that, and the registry
    flag is what the harness's repeat policy runs on — so it is pinned here.
    """
    a = detect(fold())
    b = detect(fold())
    assert [s.tolist() for s in a.seams] == [s.tolist() for s in b.seams]
    assert not REGISTRY["lit-ppf"].randomised
    assert REGISTRY["lit-ppf"].oracle_name == "band"


def test_exact_normals_are_an_explicit_arm_and_estimation_is_the_default():
    """The first method that consumes normals, so the L2 rung starts here.

    `"estimate"` is the faithful condition (their PCL pipeline computes normals from the
    scan); `"exact"` takes the generator's analytic normals and is an oracle arm. Both
    must run; `"exact"` without the array must refuse loudly.
    """
    pts = fold()
    n_true = np.zeros((len(pts), 3))
    n_true[: len(pts) // 2] = [0, 0, 1]
    n_true[len(pts) // 2:] = [0, 1, 0]

    r_est = detect(pts, normals="estimate")
    r_ex = detect(pts, normals="exact", normals_xyz=n_true)
    assert r_est.n_seams >= 1 and r_ex.n_seams >= 1
    assert r_ex.used_exact_normals and not r_est.used_exact_normals
    with pytest.raises(ValueError):
        detect(pts, normals="exact")


def test_runs_through_the_harness_from_its_first_execution():
    """The plan's commitment: `lit-ppf` never runs outside `run_matrix`.

    One bench scene through the harness: the row exists, carries the L0 band arm, and the
    repeat policy treats it as deterministic (verify_seeds rows, not the full seed list).
    """
    try:
        corpus = balanced_corpus(ROOT / "out" / "bench", per_type=50)
    except (ValueError, FileNotFoundError, NotADirectoryError):
        pytest.skip("no balanced corpus at out/bench")
    prep = prepare(corpus["T"][:1])
    df = run_matrix(prep, methods=["lit-ppf"], seeds=range(6), verify_seeds=2)
    assert len(df) == 2                                # deterministic: verify_seeds rows
    assert (df.method == "lit-ppf").all() and (df.arm == "L0").all()
    assert df.f1.nunique() == 1, "zero spread must hold on a real scene too"
