"""`self-seamfind` — the `Self_ideas/test2_cagdas/seam_finder.md` method, per-stage tests
from the plan, on small analytic grids (mm), plus one harness run on the tier-1 corpus.

Stage 1: MLS projection error < 0.1 h on a plane, normals within 1 deg; per-part normals at
a 90 deg corner within 3 deg at 1 h from it. Stage 3: T-joint seeds lie near the root, on
both corners. Stage 4: d_k never increases; gap error < 0.25 mm at a 1 mm gap. Stage 5:
pipe on plate gives one closed seam.
"""
from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "Self_ideas" / "test2_cagdas"))
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT))

from seamfind import Params, extract  # noqa: E402
from seamfind import walk  # noqa: E402
from seamfind.geom import PartSurface  # noqa: E402

BENCH = ROOT / "out" / "bench_phase4"


def grid(u0, u1, v0, v1, h):
    u, v = np.meshgrid(np.arange(u0, u1 + 1e-9, h), np.arange(v0, v1 + 1e-9, h))
    return u.ravel(), v.ravel()


def box(x0, x1, y0, y1, z0, z1, h):
    """Surface samples + outward normals of an axis-aligned box."""
    P, N = [], []
    for ax, (lo, hi) in enumerate(((x0, x1), (y0, y1), (z0, z1))):
        o = [i for i in range(3) if i != ax]
        rng = ((x0, x1), (y0, y1), (z0, z1))
        u, v = grid(*rng[o[0]], *rng[o[1]], h)
        for val, s in ((lo, -1), (hi, 1)):
            Q = np.zeros((len(u), 3)); Q[:, ax] = val; Q[:, o[0]] = u; Q[:, o[1]] = v
            n = np.zeros((len(u), 3)); n[:, ax] = s
            P.append(Q); N.append(n)
    return np.vstack(P), np.vstack(N)


def tjoint(h=1.0, gap=0.0, t=6.0):
    A, nA = box(-50, 50, -40, 40, -10, 0, h)
    B, nB = box(-t / 2, t / 2, -40, 40, gap, 60, h)
    keepA = ~((np.abs(A[:, 0]) < t / 2 - 1e-9) & (nA[:, 2] > 0.5))      # top face under the stem: buried
    keepB = ~((np.abs(B[:, 0]) < t / 2 - 1e-9) & (nB[:, 2] < -0.5))     # stem foot face: buried
    return A[keepA], nA[keepA], B[keepB], nB[keepB]


def test_stage1_mls_on_a_plane():
    u, v = grid(-20, 20, -20, 20, 1.0)
    P = np.column_stack([u, v, np.zeros_like(u)])
    S = PartSurface(P, 1.0, orient=np.tile([0, 0, 1.0], (len(P), 1)))
    X = np.column_stack([np.random.default_rng(0).uniform(-10, 10, (50, 2)), np.full(50, 2.0)])
    foot, n, on = S.project(X)
    assert np.abs(foot[:, 2]).max() < 0.1
    assert np.degrees(np.arccos(np.clip(n[:, 2], -1, 1))).max() < 1.0
    assert on.all()


def test_stage1_per_part_normals_at_a_corner():
    A, nA, B, nB = tjoint()
    S = PartSurface(A, 1.0, orient=nA)
    m = (np.abs(np.abs(A[:, 0]) - 4.0) < 0.5) & (A[:, 2] == 0) & (np.abs(A[:, 1]) < 30)   # 1 h from the root
    err = np.degrees(np.arccos(np.clip((S.N[m] * nA[m]).sum(1), -1, 1)))
    assert err.max() < 3.0


def test_stage4_walker_never_overshoots_and_measures_the_gap():
    A, nA, B, nB = tjoint(gap=1.0)
    SA, SB = PartSurface(A, 1.0, orient=nA), PartSurface(B, 1.0, orient=nB)
    starts = np.array([[x, y, 0.0] for x in (6.0, 8.0, -7.0) for y in (-20.0, 0.0, 20.0)])
    toe, _, d, steps, conv, H = walk.walk(SA, SB, starts, 0.9, 0.05, 0.01, 50)
    Hf = np.where(np.isfinite(H), H, np.inf)
    dif = np.diff(np.where(np.isfinite(H), H, np.nan), axis=1)
    assert np.nanmax(dif) <= 1e-9                      # d_k non-increasing
    assert conv.all()
    assert np.abs(d - 1.0).max() < 0.25                # gap error < 0.25 mm at 1 mm


def test_stage3_to_5_t_joint_two_seams_on_the_root():
    A, nA, B, nB = tjoint(gap=0.5)
    r = extract(A, B, Params(), normals_A=nA, normals_B=nB)
    assert len(r.seams) == 2
    for s in r.seams:
        x = np.abs(s["points"][:, 0]); z = s["points"][:, 2]
        assert np.median(np.abs(x - 3.0)) < 0.5 and np.median(np.abs(z)) < 0.5   # nominal: x = ±t/2, z = 0
        assert 60 < s["length"] < 85
        assert abs(np.median(s["dihedral"]) - 90) < 5


def test_stage5_pipe_on_plate_one_closed_seam():
    h = 1.0
    u, v = grid(-60, 60, -60, 60, h)
    A = np.column_stack([u, v, np.zeros_like(u)]); A = A[np.hypot(A[:, 0], A[:, 1]) > 30]
    th = np.arange(0, 2 * np.pi, h / 30.0); z = np.arange(0.5, 60, h)
    T, Z = np.meshgrid(th, z)
    B = np.column_stack([30 * np.cos(T.ravel()), 30 * np.sin(T.ravel()), Z.ravel()])
    nB = np.column_stack([np.cos(T.ravel()), np.sin(T.ravel()), np.zeros(T.size)])
    r = extract(A, B, Params(), normals_A=np.tile([0, 0, 1.0], (len(A), 1)), normals_B=nB)
    assert len(r.seams) == 1
    s = r.seams[0]
    rad = np.hypot(s["points"][:, 0], s["points"][:, 1])
    assert s["closed"] and abs(np.median(rad) - 30) < 0.3


@pytest.mark.skipif(not BENCH.exists(), reason="tier-1 benchmark not on disk")
def test_runs_through_the_harness_deterministically():
    import pandas as pd
    from baselines import prepare, run_matrix
    from baselines import self_seamfind as sk
    f = pd.read_csv(BENCH / "facts.csv")
    r = f[(f.joint_type == "T") & f.seam_family.isna()].iloc[0]
    df = run_matrix(prepare([BENCH / "T" / r.scene_id]), methods=[sk.spec()], seeds=[0, 1],
                    verify_seeds=2, view="single")
    assert len(df) == 2 and df.f1.nunique() == 1 and df.f1.iloc[0] > 0.8
