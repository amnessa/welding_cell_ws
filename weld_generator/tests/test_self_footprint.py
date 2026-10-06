"""`self-footprint` — the `Self_ideas/test1_ismail_hoca/weld_seam.py` extractor through the
harness adapter `baselines.self_footprint`.

Claims to pin: `extract` (the in-memory body split out of `run`) reproduces the script's
own synthetic T joint; the synthesised base scan is part A only, an INDEPENDENT draw from
the assembly cloud (no shared points), complete under `full_exterior` and camera-limited
under `single`; and the adapter runs through `run_matrix` deterministically.
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT))

from baselines import self_footprint as sf  # noqa: E402

BENCH = ROOT / "out" / "bench_phase4"


def test_extract_reproduces_the_scripts_synthetic_t_joint():
    from make_synthetic import scenario
    A, B, gt = scenario("tjoint")
    rng = np.random.default_rng(0)
    pa, pb = A.sample(1.0), B.sample(1.0)
    combined = np.vstack([pa[~B.inside(pa, 0.3)], pb[~A.inside(pb, 0.3)]])
    base = pa + rng.normal(0, 0.05, pa.shape)
    combined = combined + rng.normal(0, 0.05, combined.shape)
    polys, r = sf.detect(base, combined)
    assert len(polys) == 2
    e = sf.weld_seam.evaluate(r["paths"], gt, r["v"])     # the script's own metric
    assert e["path_to_gt_mean"] < 0.3 and e["gt_coverage"] > 0.95


@pytest.mark.skipif(not BENCH.exists(), reason="tier-1 benchmark not on disk")
def test_base_scan_is_part_a_alone_and_an_independent_draw():
    import pandas as pd
    from scipy.spatial import cKDTree
    from baselines import prepare
    f = pd.read_csv(BENCH / "facts.csv")
    r = f[f.joint_type == "lap"].iloc[0]
    prep = prepare([BENCH / "lap" / r.scene_id])[0]
    full = sf.base_scan(prep, "full_exterior")
    single = sf.base_scan(prep, "single")
    c = prep.cloud("full_exterior", 0.0)
    a_pts = c["xyz"][c["object_id"] == 0]
    b_pts = c["xyz"][c["object_id"] == 1]
    assert cKDTree(c["xyz"]).query(full)[0].min() > 1e-6     # no shared point
    assert np.median(cKDTree(a_pts).query(full)[0]) < 1.0    # lies on A
    assert np.quantile(cKDTree(full).query(b_pts)[0], 0.5) > 2.0   # not on B
    assert 0 < len(single) < len(full)
    assert np.array_equal(sf.base_scan(prep, "single"), single)      # cached, deterministic


@pytest.mark.skipif(not BENCH.exists(), reason="tier-1 benchmark not on disk")
def test_runs_through_the_harness_with_zero_seed_spread():
    import pandas as pd
    from baselines import prepare, run_matrix
    f = pd.read_csv(BENCH / "facts.csv")
    r = f[(f.joint_type == "T") & (f.seam_family == "saddle")].iloc[0]
    df = run_matrix(prepare([BENCH / "T" / r.scene_id]), methods=[sf.spec()], seeds=[0, 1],
                    verify_seeds=2, view="single")
    assert len(df) == 2 and df.f1.nunique() == 1
    assert {"gap_mm", "n_base", "voxel_mm"} <= set(df.columns)
