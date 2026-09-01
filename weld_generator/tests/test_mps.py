"""Phase 6c step (a) — the MPS rule (D25).

Claims under test: the rule is a PURE function of stored per-seam fields implementing
the D25 order (visible arclength, then larger fold, then lower id) with a null answer
below `min_len_mm`; curved seams' `{min,max,mean}` dihedral is consumed via the mean;
and emission is flag-gated so a pre-6c scene reproduces bit-identically — the flag adds
exactly the `mps` block and nothing else.
"""

from __future__ import annotations

import json
import pathlib
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from weldgen.config import load_config  # noqa: E402
from weldgen.hashing import content_hash  # noqa: E402
from weldgen.mps import RULE_VERSION, mps_rule  # noqa: E402
from weldgen.scene import SceneRejected, generate_scene  # noqa: E402

try:
    import jsonschema
except ImportError:                                        # pragma: no cover
    jsonschema = None


def _seam(i, cls="fillet", vis=1.0, length=100.0, dih=90.0, weldable=True):
    return {"id": i, "seam_class": cls, "weldable": weldable,
            "visible_fraction": vis, "length_mm": length, "dihedral_deg": dih}


def test_rule_prefers_visible_arclength():
    scene = {"seams": [_seam(0, vis=1.0, length=80.0),
                       _seam(1, vis=0.5, length=200.0),      # 100 mm visible: wins
                       _seam(2, vis=0.9, length=90.0, weldable=False)]}
    block = mps_rule(scene)
    assert block["rule_version"] == RULE_VERSION
    assert block["seam_id"] == 1 and block["class"] == "fillet"


def test_ties_break_by_fold_then_lower_id():
    # equal visible arclength: the sharper crease (fillet, fold 90) beats the flat
    # butt centreline (fold 0)
    scene = {"seams": [_seam(0, cls="butt", dih=180.0), _seam(1, cls="fillet")]}
    assert mps_rule(scene)["seam_id"] == 1
    # equal arclength AND fold: the LOWER id wins
    scene = {"seams": [_seam(3), _seam(2)]}
    assert mps_rule(scene)["seam_id"] == 2


def test_null_below_min_len_is_an_answer():
    scene = {"seams": [_seam(0, vis=0.05, length=100.0)]}   # 5 mm visible
    block = mps_rule(scene)
    assert block["seam_id"] is None and block["class"] is None
    assert block["params"]["min_len_mm"] == 10.0
    assert mps_rule(scene, min_len_mm=4.0)["seam_id"] == 0  # the param is real


def test_curved_dihedral_summary_uses_the_mean():
    varying = {"min": 74.8, "max": 105.2, "mean": 90.0}
    scene = {"seams": [_seam(0, dih=180.0), _seam(1, dih=varying)]}
    assert mps_rule(scene)["seam_id"] == 1


def test_emission_is_flag_gated_and_adds_only_the_block():
    cfg = load_config(str(ROOT / "configs" / "bench_butt.yaml"))
    for seed in range(30):
        try:
            scene_off, arrays = generate_scene(cfg, seed)
            break
        except SceneRejected:
            continue
    else:
        pytest.fail("no emitting seed")
    assert "mps" not in scene_off, "default off: pre-6c scenes reproduce"

    cfg_on = dict(cfg, emit_mps=True)
    scene_on, arrays_on = generate_scene(cfg_on, seed)
    assert scene_on["twin_key"] == scene_off["twin_key"], \
        "emit_mps is not a geometry key - the twin pairing must survive it"
    assert scene_on["mps"] == mps_rule(scene_on)
    if jsonschema is not None:
        schema = json.loads((ROOT / "docs" / "scene.schema.json").read_text())
        jsonschema.validate(scene_on, schema)
    # a config that sets the flag IS a different config, so the identity fields
    # (config_id, and scene_id which embeds it) legitimately differ; everything
    # else must be bit-identical once the block is stripped
    stripped = dict(scene_on)
    del stripped["mps"]
    for k in ("scene_id", "config_id"):
        assert stripped[k] != scene_off[k]
        stripped[k] = scene_off[k]
    assert content_hash(stripped, arrays_on) == content_hash(scene_off, arrays), \
        "the flag must add exactly the mps block - no draw, no other field"
