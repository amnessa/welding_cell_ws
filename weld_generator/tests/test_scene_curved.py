"""Phase 6b step 3b — curved scenes are SCENES: schema-valid, hashed, reproducible.

Claims under test: every family emits a schema-valid scene through the same writer
contract as the plate pipeline; generation is bit-reproducible; the stored parametric
form regenerates the stored arrays (the D1 receipt at file level); the per-sample
dihedral array backs the block summary; and the D34 chord figure rides in the cloud
block under its gate.
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
from weldgen.curves import from_parametric  # noqa: E402
from weldgen.hashing import content_hash  # noqa: E402
from weldgen.scene import SceneRejected  # noqa: E402
from weldgen.scene_curved import generate_curved_scene  # noqa: E402

try:
    import jsonschema
except ImportError:                                        # pragma: no cover
    jsonschema = None

SCHEMA = json.loads((ROOT / "docs" / "scene.schema.json").read_text())


def _cfg(families):
    cfg = load_config(str(ROOT / "configs" / "curved_smoke.yaml"))
    cfg["seam_families"] = list(families)
    return cfg


def _first_scene(families, budget=20):
    cfg = _cfg(families)
    for seed in range(budget):
        try:
            return generate_curved_scene(cfg, seed), seed, cfg
        except SceneRejected:
            continue
    pytest.fail(f"no scene emitted for families {families} in {budget} seeds")


@pytest.mark.parametrize("family", [2, 3, 4, 5, 6, 7])
def test_every_family_emits_a_schema_valid_scene(family):
    (scene, arrays), _, _ = _first_scene([family])
    if jsonschema is not None:
        jsonschema.validate(scene, SCHEMA)
    assert scene["joint"]["seam_family"] is not None
    assert scene["cloud"]["max_chord_error_mm"] <= 0.25          # D34, recorded
    prims = {o["primitive"] for o in scene["objects"]}
    assert prims <= {"slab", "tube", "swept_slab"}
    # every curved face carries either a plane or a surface description
    for f in scene["faces"]:
        assert (f["plane"] is not None) or (f["surface"] is not None) \
            or f["name"] == "-w", f["ref"]                        # cut faces: neither
    # closed topology where the family demands it
    closed_expected = {2: True, 3: True, 4: True, 5: True, 6: False, 7: False}
    weld0 = next(s for s in scene["seams"] if s["weldable"])
    assert weld0["closed"] == closed_expected[family]


def test_generation_is_bit_reproducible():
    cfg = _cfg([4])
    for seed in range(10):
        try:
            a_s, a_a = generate_curved_scene(cfg, seed)
            b_s, b_a = generate_curved_scene(cfg, seed)
        except SceneRejected:
            continue
        assert content_hash(a_s, a_a) == content_hash(b_s, b_a)
        return
    pytest.fail("no emitting seed found")


@pytest.mark.parametrize("family", [2, 4, 6])
def test_stored_parametric_regenerates_the_stored_arrays(family):
    """The D1 receipt at file level: rebuild the curve from scene.json's parametric
    block and resample it at the stored density - it must reproduce seams.npz."""
    (scene, arrays), _, _ = _first_scene([family])
    for s in scene["seams"]:
        key = s["sampled"]["array"]
        stored = arrays[f"seams.npz:{key}"]
        curve = from_parametric(s["parametric"])
        regen = curve.sample(s["sampled"]["density_per_mm"]).astype(np.float32)
        assert regen.shape == stored.shape, (family, s["id"])
        assert np.abs(regen - stored).max() < 1e-4, (family, s["id"])
        assert abs(curve.length_mm - s["length_mm"]) < 1e-6


def test_dihedral_array_backs_the_block_summary():
    (scene, arrays), _, _ = _first_scene([3])              # tilted pipe: it varies
    weld = next(s for s in scene["seams"] if s["weldable"])
    dih = arrays[f"seams.npz:{weld['sampled']['array']}_dihedral"]
    assert isinstance(weld["dihedral_deg"], dict), \
        "a varying dihedral must be summarised, not averaged away"
    assert abs(float(dih.min()) - weld["dihedral_deg"]["min"]) < 0.5
    assert abs(float(dih.max()) - weld["dihedral_deg"]["max"]) < 0.5
    assert weld["dihedral_deg"]["max"] - weld["dihedral_deg"]["min"] > 10.0


def test_frames_arrays_are_worldposed_and_unit():
    (scene, arrays), _, _ = _first_scene([2])
    weld = next(s for s in scene["seams"] if s["weldable"])
    key = weld["sampled"]["array"]
    for suffix in ("_tangent", "_approach", "_nA", "_nB"):
        v = arrays[f"seams.npz:{key}{suffix}"]
        assert np.abs(np.linalg.norm(v, axis=1) - 1.0).max() < 1e-6
    # nA is the base plane's normal: after a z-preserving pose it is still +z
    assert np.abs(arrays[f"seams.npz:{key}_nA"]
                  - np.array([0, 0, 1.0], dtype=np.float32)).max() < 1e-6


def test_writer_roundtrip(tmp_path):
    from weldgen.writer import write_scene
    (scene, arrays), _, _ = _first_scene([5])
    root = write_scene(tmp_path, scene, arrays)
    assert (root / "scene.json").exists()
    assert (root / "cloud.npz").exists() and (root / "seams.npz").exists()
    reloaded = json.loads((root / "scene.json").read_text())
    assert reloaded["joint"]["seam_family"] == "rounded_rect"
    npz = np.load(root / "seams.npz")
    weld = next(s for s in reloaded["seams"] if s["weldable"])
    assert f"{weld['sampled']['array']}_dihedral" in npz.files
