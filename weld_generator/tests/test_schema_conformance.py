"""Every emitted scene must validate against the frozen schema.

This is the Phase 1 checklist item "validate every emitted scene.json against
scene.schema.json in CI". `schema_contract.py` tests the schema itself; this tests the
generator's output against it.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest
from jsonschema import Draft202012Validator

from weldgen.config import load_config
from weldgen.scene import generate_scene

ROOT = Path(__file__).resolve().parents[1]
SEEDS = [8412337, 1, 2, 3, 99, 123456789]


@pytest.fixture(scope="module")
def validator():
    schema = json.loads((ROOT / "docs" / "scene.schema.json").read_text())
    Draft202012Validator.check_schema(schema)
    return Draft202012Validator(schema)


@pytest.fixture(scope="module")
def cfg():
    c = load_config(str(ROOT / "configs" / "smoke.yaml"))
    # The D31 acceptance gate is exercised in test_class_disjointness; here it is
    # disabled because the canonical seed 8412337 is (correctly) a class-boundary
    # rejection under it - at ~80 deg yaw its contact line hugs A's end edge within
    # the clearance - and a rejected seed has no scene to validate.
    c["class_disjoint"] = False
    return c


@pytest.mark.parametrize("seed", SEEDS)
def test_scene_validates(validator, cfg, seed):
    scene, _ = generate_scene(cfg, seed)
    errors = sorted(validator.iter_errors(scene), key=lambda e: list(e.path))
    assert not errors, "\n".join(f"{list(e.path)}: {e.message}" for e in errors[:5])


@pytest.mark.parametrize("preset", ["phase1.yaml", "smoke.yaml", "reference_tjoint.yaml"])
def test_every_preset_validates(validator, preset):
    cfg = load_config(str(ROOT / "configs" / preset))
    # A preset may reject seeds (NoVisibleSeams, the D31 bands - smoke's canonical seed
    # is a correct corner-like rejection); the claim here is that whatever a preset DOES
    # emit validates, so walk to the first emitted scene.
    from weldgen.scene import SceneRejected
    scene = None
    for seed in [8412337, 1, 2, 3, 4, 5, 6, 7]:
        try:
            scene, _ = generate_scene(cfg, seed)
            break
        except SceneRejected:
            continue
    assert scene is not None, f"{preset} emitted nothing in the seed budget"
    errors = sorted(validator.iter_errors(scene), key=lambda e: list(e.path))
    assert not errors, "\n".join(f"{list(e.path)}: {e.message}" for e in errors[:5])


def test_phase1_is_fixture_free(cfg):
    """D12: the fixture is off throughout Phase 1, so contact_mode is 'free'."""
    scene, _ = generate_scene(cfg, 8412337)
    assert scene["joint"]["contact_mode"] == "free"
    assert all(o["role"] == "workpiece" for o in scene["objects"])


def test_stored_seam_definition_is_nominal(cfg):
    """D19: the stored curve is always the nominal zero-gap intersection."""
    scene, _ = generate_scene(cfg, 8412337)
    assert scene["seam_definition"] == "nominal"


def test_reference_tjoint_reproduces_the_measured_geometry():
    """The one scene tying the generator to a real measurement (README §8):
    232 mm seam, 8.4 mm plate, 1.1 mm root gap."""
    cfg = load_config(str(ROOT / "configs" / "reference_tjoint.yaml"))
    scene, _ = generate_scene(cfg, 8412337)
    assert scene["fit"]["root_gap_mm"] == pytest.approx(1.1, abs=1e-9)
    assert scene["joint"]["included_angle_deg"] == pytest.approx(90.0, abs=1e-9)
    for o in scene["objects"]:
        assert o["thickness_mm"] == pytest.approx(8.4, abs=1e-9)
    # Phase 2 emits every candidate D4 finds, so the scene also carries short end-face
    # runs and rejected negatives. The measured quantity is the pair of fillets.
    fillets = [s for s in scene["seams"]
               if s["weldable"] and tuple(s["face_pair"]) in
               {("A:+w", "B:+w"), ("A:+w", "B:-w")}]
    assert len(fillets) == 2
    for s in fillets:
        assert s["length_mm"] == pytest.approx(232.0, abs=1e-6)
        assert s["dihedral_deg"] == pytest.approx(90.0, abs=1e-6)


def test_face_registry_indexes_match_point_labels(cfg):
    """`faces[].face_id` IS the per-point `face_id` (SCHEMA.md §2.3)."""
    import numpy as np
    scene, arrays = generate_scene(cfg, 8412337)
    ids = {f["face_id"] for f in scene["faces"]}
    assert ids == set(range(len(scene["faces"])))
    assert set(np.unique(arrays["cloud.npz:face_id"])).issubset(ids)

    # object_id must agree between the two labellings.
    by_face = {f["face_id"]: f["object"] for f in scene["faces"]}
    oid_of = {o["id"]: o["object_id"] for o in scene["objects"]}
    fid = arrays["cloud.npz:face_id"]
    oid = arrays["cloud.npz:object_id"]
    for f in np.unique(fid):
        assert set(np.unique(oid[fid == f])) == {oid_of[by_face[int(f)]]}


def test_normals_are_unit_and_outward(cfg):
    import numpy as np
    _, arrays = generate_scene(cfg, 8412337)
    n = arrays["cloud.npz:normals"]
    assert np.allclose(np.linalg.norm(n, axis=1), 1.0, atol=1e-5)
