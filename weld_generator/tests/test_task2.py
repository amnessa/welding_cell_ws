"""The Task-2 chunk (6c(c)) — MPS selection + localization through `run_task2`.

Validated the way the harness itself was: against the fake oracle, where every property
of a row is known in advance. Claims under test: a jittered-truth predictor must come
back `mps_matched` with RMSE ~ sigma; a predictor that returns nothing must come back
as an unmatched row, not an error; a closed MPS ring must carry `end_error_mm = NaN`
(D39 — a ring has no endpoints) while its distance metrics stay finite and
ROTATION-INVARIANT; and a band-output method must be refused, because a band cannot
select a seam.
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from baselines.harness import prepare, run_task2  # noqa: E402
from baselines.metrics import path_error_mm  # noqa: E402
from weldgen.config import load_config  # noqa: E402
from weldgen.scene import SceneRejected, generate_scene  # noqa: E402
from weldgen.scene_curved import generate_curved_scene  # noqa: E402
from weldgen.writer import write_scene  # noqa: E402


def _prep(tmp_path, gen, cfg, tag):
    for seed in range(30):
        try:
            scene, arrays = gen(cfg, seed)
            break
        except SceneRejected:
            continue
    else:
        pytest.fail(f"no emitting seed for {tag}")
    write_scene(tmp_path, scene, arrays)
    return prepare([tmp_path / scene["scene_id"]], primary_only=False)


@pytest.fixture(scope="module")
def plate(tmp_path_factory):
    cfg = load_config(str(ROOT / "configs" / "bench6a_T.yaml"))
    return _prep(tmp_path_factory.mktemp("plate"), generate_scene, cfg, "plate")


@pytest.fixture(scope="module")
def ring(tmp_path_factory):
    cfg = load_config(str(ROOT / "configs" / "curved_smoke.yaml"))
    cfg["seam_families"] = [2]
    return _prep(tmp_path_factory.mktemp("ring"), generate_curved_scene, cfg, "ring")


def test_jittered_truth_matches_the_mps_seam(plate):
    df = run_task2(plate, methods=["fake-oracle"], seeds=[0], verify_seeds=1,
                   method_kw={"fake-oracle": {"sigma_mm": 0.3}})
    r = df.iloc[0]
    assert not r["mps_null"] and r["mps_matched"]
    assert r["rmse"] < 1.0                       # jitter of 0.3 mm, not a mismatch
    assert np.isfinite(r["end_error_mm"])        # open seam: endpoints are real


def test_empty_prediction_is_an_unmatched_row_not_an_error(plate):
    df = run_task2(plate, methods=["fake-oracle"], seeds=[0], verify_seeds=1,
                   method_kw={"fake-oracle": {"p_miss": 1.0}})
    r = df.iloc[0]
    assert not r["mps_matched"] and np.isnan(r["rmse"])
    assert r["mps_seam_id"] is not None          # the truth label does not vanish


def test_closed_ring_has_no_endpoint_error_and_is_rotation_invariant(ring):
    df = run_task2(ring, methods=["fake-oracle"], seeds=[0], verify_seeds=1,
                   method_kw={"fake-oracle": {"sigma_mm": 0.1}})
    r = df.iloc[0]
    assert r["mps_closed"] and r["mps_matched"]
    assert np.isnan(r["end_error_mm"])           # D39: a ring has no endpoints
    assert np.isfinite(r["rmse"])
    # the D39 requirement itself: every reported distance metric must not care where
    # along the ring the prediction starts
    gi = next(i for i, m in enumerate(ring[0].gt_meta)
              if m["id"] == r["mps_seam_id"])
    gt = ring[0].gt[gi]
    a = path_error_mm(gt, gt)
    b = path_error_mm(np.roll(gt, len(gt) // 3, axis=0), gt)
    for k in ("rmse", "me", "lateral_rmse"):
        assert abs(a[k] - b[k]) < 0.05, f"{k} moved under a rotation of the ring"


def test_band_output_method_is_refused(plate):
    with pytest.raises(ValueError, match="band"):
        run_task2(plate, methods=["ours"], seeds=[0], verify_seeds=1)
