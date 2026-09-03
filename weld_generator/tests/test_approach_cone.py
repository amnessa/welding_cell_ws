"""Phase 6c step (b) — the D26 `approach_cone` camera regime.

Claims under test: the regime is a free twin of `uniform_sphere` (same seed, same
geometry and seam truth, different camera — the flag is not a geometry key); its
draws are appended to substream 5 and its fields appear only in configs that set it;
the target seam actually clears the visibility threshold when `cleared` says so; and
the 6c(b) gate holds where grading is geometrically possible — on edge and lap the
regime yields MORE scenes AND more graded MPS margins than the uniform sampler
(T pins to 0 by the complementary-lobe structure and corner to ~1 by the same-corner
tie; both are recorded task properties, not regime failures).
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from weldgen.config import load_config  # noqa: E402
from weldgen.mps import mps_margin  # noqa: E402
from weldgen.scene import SceneRejected, generate_scene  # noqa: E402
from weldgen.scene_curved import generate_curved_scene  # noqa: E402


def _cone_cfg(**over):
    cfg = load_config(str(ROOT / "configs" / "task2_cone.yaml"))
    cfg.update(over)
    return cfg


def _uniform_twin(cfg):
    twin = dict(cfg)
    del twin["camera_regime"]
    return twin


def test_regime_is_a_free_twin_with_recorded_target():
    cone = _cone_cfg()
    uni = _uniform_twin(cone)
    for seed in range(20):
        try:
            sc, ac = generate_scene(cone, seed)
            su, au = generate_scene(uni, seed)
        except SceneRejected:
            continue
        assert sc["twin_key"] == su["twin_key"]
        assert np.array_equal(ac["seams.npz:seam_0"], au["seams.npz:seam_0"]), \
            "geometry and seam truth must be bit-identical across regimes"
        assert sc["camera"]["T_world_cam"] != su["camera"]["T_world_cam"]
        cam = sc["camera"]
        assert cam["regime"] == "approach_cone"
        assert "regime" not in su["camera"], "fields only in configs that set the flag"
        if cam["approach_cleared"]:
            assert cam["target_visible_fraction"] >= 0.5
        assert cam["target_seam_id"] is not None
        tgt = next(s for s in sc["seams"] if s["id"] == cam["target_seam_id"])
        assert list(tgt["face_pair"]) == list(cam["target_face_pair"])
        return
    pytest.fail("no seed emitted under both regimes")


def test_gate_yield_and_graded_margins_on_edge_and_lap():
    """The 6c(b) gate, on the classes where grading is geometrically possible."""
    kept = {"cone": 0, "uni": 0}
    graded = {"cone": 0, "uni": 0}
    for jt in ("edge", "lap"):
        cone = _cone_cfg(joint_type=[jt])
        uni = _uniform_twin(cone)
        for seed in range(20):
            for name, cfg in (("cone", cone), ("uni", uni)):
                try:
                    s, _ = generate_scene(cfg, seed)
                except SceneRejected:
                    continue
                kept[name] += 1
                m = mps_margin(s)
                if m is not None and 0.05 < m < 0.95:
                    graded[name] += 1
    assert kept["cone"] > kept["uni"], \
        "coarse positioning must recover the NoVisibleSeams losses (D26)"
    assert graded["cone"] > graded["uni"] and graded["cone"] >= 8, \
        f"the MPS margin must come out graded under the cone: {graded}"


def test_curved_pipeline_carries_the_regime():
    cfg = load_config(str(ROOT / "configs" / "curved_smoke.yaml"))
    cfg["seam_families"] = [3]                       # tilted pipe: fast, two welds
    cfg["camera_regime"] = "approach_cone"
    for seed in range(15):
        try:
            s, _ = generate_curved_scene(cfg, seed)
        except SceneRejected:
            continue
        cam = s["camera"]
        assert cam["regime"] == "approach_cone"
        assert cam["target_seam_id"] is not None
        if cam["approach_cleared"]:
            assert cam["target_visible_fraction"] >= 0.5
        return
    pytest.fail("no curved scene emitted")
