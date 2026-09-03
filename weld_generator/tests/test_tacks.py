"""Phase 7 — tackrule-0.1 (D38) and the closed-loop phase convention (D39).

Claims under test: spacing obeys the thickness-scaled bounds with the 400 mm ceiling;
a too-short seam collapses to ONE recorded tack instead of nonsense; closed loops get
an even n >= 4, a deterministic seed-free phase, and an opposite-pair weld order; the
sequence staggers same-class seams round-robin; and emission is flag-gated exactly
like `mps` — the flag adds the block and nothing else.
"""

from __future__ import annotations

import json
import pathlib
import sys

import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from weldgen.config import load_config  # noqa: E402
from weldgen.hashing import content_hash  # noqa: E402
from weldgen.scene import SceneRejected, generate_scene  # noqa: E402
from weldgen.scene_curved import generate_curved_scene  # noqa: E402
from weldgen.tacks import RULE_VERSION, tack_rule  # noqa: E402

try:
    import jsonschema
except ImportError:                                        # pragma: no cover
    jsonschema = None


def _scene(seams_spec, t=6.0):
    scene = {"scene_id": "deadbeef-0000000001",
             "objects": [{"role": "workpiece", "thickness_mm": t},
                         {"role": "workpiece", "thickness_mm": t}],
             "seams": []}
    arrays = {}
    for i, (L, closed, cls) in enumerate(seams_spec):
        if closed:
            th = np.linspace(0.0, 2 * np.pi, 400, endpoint=False)
            r = L / (2 * np.pi)
            poly = np.stack([r * np.cos(th), r * np.sin(th), np.zeros_like(th)], 1)
        else:
            u = np.linspace(0.0, L, 200)
            poly = np.stack([u, np.zeros_like(u), np.zeros_like(u)], 1)
        scene["seams"].append({"id": i, "weldable": True, "matches_joint_type": True,
                               "seam_class": cls, "closed": closed, "length_mm": L,
                               "sampled": {"array": f"seam_{i}"}})
        arrays[f"seams.npz:seam_{i}"] = poly
    return scene, arrays


def test_open_seam_obeys_the_thickness_scaled_bounds():
    scene, arrays = _scene([(600.0, False, "fillet")], t=6.0)
    b = tack_rule(scene, arrays)
    s = np.asarray(b["arclength_mm"])
    assert b["rule_version"] == RULE_VERSION
    # margin = max(2t, tack_len) = tack_len = 24 at both effective endpoints
    assert abs(s[0] - 24.0) < 1e-9 and abs(s[-1] - (600.0 - 24.0)) < 1e-9
    gaps = np.diff(s)
    assert (gaps >= 10.0 * 6.0 - 1e-9).all() and (gaps <= 33.0 * 6.0 + 1e-9).all()
    assert b["tack_length_mm"][0] == 24.0
    # weld order: ends first, then bisection
    ranked = np.argsort(b["order"])
    assert list(ranked[:2]) == [0, len(s) - 1]


def test_the_400mm_ceiling_binds_thick_plate():
    scene, arrays = _scene([(3000.0, False, "butt")], t=12.0)
    s = np.asarray(tack_rule(scene, arrays)["arclength_mm"])
    assert np.diff(s).max() <= 400.0 + 1e-9      # k_max*t = 396 < 400 already; pin it


def test_short_seam_collapses_to_one_tack():
    scene, arrays = _scene([(80.0, False, "fillet")], t=8.0)
    b = tack_rule(scene, arrays)                 # margin 32 each end, L_eff 16 <= 32
    assert len(b["arclength_mm"]) == 1
    assert abs(b["arclength_mm"][0] - 40.0) < 1e-9


def test_closed_loop_is_even_phased_and_opposite_paired():
    L = 314.0
    scene, arrays = _scene([(L, True, "fillet")], t=6.0)
    b = tack_rule(scene, arrays)
    n = len(b["arclength_mm"])
    assert n >= 4 and n % 2 == 0
    ph = b["params"]["phase_by_seam"]["0"]
    assert 0.0 <= ph < 1.0
    assert tack_rule(scene, arrays)["params"]["phase_by_seam"]["0"] == ph
    # the first two tacks in weld order are antipodal (shrinkage balance)
    ranked = np.argsort(b["order"])
    s = np.asarray(b["arclength_mm"])
    gap = abs(s[ranked[0]] - s[ranked[1]])
    assert abs(min(gap, L - gap) - L / 2.0) < 2.0


def test_sequence_staggers_same_class_seams():
    scene, arrays = _scene([(600.0, False, "fillet"), (600.0, False, "fillet")])
    b = tack_rule(scene, arrays)
    ranked = np.argsort(b["order"])
    ids = [b["seam_id"][i] for i in ranked]
    assert ids[:4] == [0, 1, 0, 1], "round-robin across the pair - the heat balance"


def test_emission_is_flag_gated_like_mps():
    cfg = load_config(str(ROOT / "configs" / "bench6a_T.yaml"))
    for seed in range(30):
        try:
            scene_off, arrays = generate_scene(cfg, seed)
            break
        except SceneRejected:
            continue
    else:
        pytest.fail("no emitting seed")
    assert scene_off["tacks"] is None
    scene_on, arrays_on = generate_scene(dict(cfg, emit_tacks=True), seed)
    assert scene_on["tacks"] == tack_rule(scene_on, arrays_on)
    primary = {s["id"] for s in scene_on["seams"]
               if s["weldable"] and s["matches_joint_type"]}
    assert set(scene_on["tacks"]["seam_id"]) <= primary
    if jsonschema is not None:
        schema = json.loads((ROOT / "docs" / "scene.schema.json").read_text())
        jsonschema.validate(scene_on, schema)
    stripped = dict(scene_on, tacks=None)
    for k in ("scene_id", "config_id"):
        stripped[k] = scene_off[k]
    assert content_hash(stripped, arrays_on) == content_hash(scene_off, arrays)


def test_curved_closed_ring_gets_on_curve_tacks():
    cfg = load_config(str(ROOT / "configs" / "curved_smoke.yaml"))
    cfg["seam_families"] = [2]
    cfg["emit_tacks"] = True
    for seed in range(20):
        try:
            scene, arrays = generate_curved_scene(cfg, seed)
            break
        except SceneRejected:
            continue
    else:
        pytest.fail("no family-2 scene")
    b = scene["tacks"]
    assert len(b["seam_id"]) >= 4 and len(b["points_mm"]) == len(b["order"])
    weld = next(s for s in scene["seams"] if s["weldable"] and s["closed"])
    poly = arrays[f'seams.npz:{weld["sampled"]["array"]}']
    for sid, pt in zip(b["seam_id"], b["points_mm"]):
        if sid != weld["id"]:
            continue
        d = np.linalg.norm(poly - np.asarray(pt), axis=1).min()
        assert d < 2.0, "tack centres must sit ON the stored ring"
