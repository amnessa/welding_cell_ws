"""Phase 8 M1 — `geom.from_object` is the exact inverse of the two object serialisers.

The claim: for every primitive, serialise -> rebuild -> serialise is the identity at the
canonical-JSON level, the rebuilt mesh is watertight and winding-consistent (D21), and
`part_geometry_id` survives. Hand-built parts cover all six primitives quickly; generated
scenes cover the three pipelines (plate, grooved, curved) end to end.
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from weldgen.config import load_config  # noqa: E402
from weldgen.curves import Arc3D, Ellipse3D  # noqa: E402
from weldgen.geom import (PRIMITIVES, PreparedPrism, PreparedSlab, Prism, Slab,  # noqa: E402
                          SweptSlab, Tube, from_object)
from weldgen.hashing import canonical_json  # noqa: E402
from weldgen.scene import SceneRejected, _object_entry, generate_scene  # noqa: E402
from weldgen.scene_curved import _objects_block, generate_curved_scene  # noqa: E402


def _serialise(part) -> dict:
    if isinstance(part, (Tube, SweptSlab)):
        return _objects_block([part])[0]
    return _object_entry(part)


def _roundtrip(part):
    entry = _serialise(part)
    rebuilt = from_object(entry)
    assert type(rebuilt) is type(part)
    assert canonical_json(_serialise(rebuilt)) == canonical_json(entry)
    assert rebuilt.part_geometry_id == entry["part_geometry_id"]
    m = rebuilt.mesh()
    assert m.is_watertight and m.is_winding_consistent
    return rebuilt


def _pose(yaw_deg=17.0, t=(12.0, -30.0, 5.0)):
    a = np.radians(yaw_deg)
    T = np.eye(4)
    T[:3, :3] = [[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]]
    T[:3, 3] = t
    return T


# ------------------------------------------------------------------ hand-built parts


def test_slab_roundtrip():
    _roundtrip(Slab("A", "workpiece", 0, (241.3, 216.1, 5.7), _pose()))


def test_fixture_slab_keeps_role_and_id():
    r = _roundtrip(Slab("F", "fixture", 255, (400.0, 300.0, 20.0), _pose(0.0)))
    assert r.role == "fixture" and r.object_id == 255


def test_prism_roundtrip():
    outline = np.array([[0.0, 0.0], [120.0, 0.0], [130.0, 70.0], [-5.0, 80.0]])
    _roundtrip(Prism("B", "workpiece", 1, outline, 6.0, _pose(), shape="trapezoid"))


def test_prepared_slab_roundtrip():
    for prep in ({"kind": "single_V", "bevel_deg": 30.0, "root_face_mm": 2.0},
                 {"kind": "single_bevel", "bevel_deg": 35.0, "root_face_mm": 1.5},
                 {"kind": "single_U", "bevel_deg": 10.0, "root_face_mm": 2.0,
                  "radius_mm": 4.0}):
        _roundtrip(PreparedSlab("A", "workpiece", 0, 150.0, 80.0, 10.0, prep, _pose()))


def test_tube_roundtrip_with_and_without_cut():
    th = np.radians(25.0)
    cut = {"kind": "plane", "n_local": [float(np.sin(th)), 0.0, float(np.cos(th))],
           "d": 0.0}
    _roundtrip(Tube("B", "workpiece", 1, 40.0, 6.0, 150.0, _pose()))
    r = _roundtrip(Tube("B", "workpiece", 1, 40.0, 6.0, 150.0, _pose(),
                        base_cut=cut, gap_mm=1.2))
    assert r.base_cut == cut and r.gap_mm == 1.2


def test_swept_slab_roundtrip_open_and_closed_spine():
    arc = Arc3D(np.zeros(3), np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), 60.0,
                0.2, 2.4)
    _roundtrip(SweptSlab("B", "workpiece", 1, arc, -3.0, 3.0, 0.5, 80.0, _pose()))
    ring = Ellipse3D(np.zeros(3), np.array([1.0, 0, 0]), np.array([0, 1.0, 0]),
                     50.0, 50.0)
    _roundtrip(SweptSlab("B", "workpiece", 1, ring, -6.0, 0.0, 0.0, 90.0, _pose()))


def test_unknown_primitive_is_an_error():
    with pytest.raises(ValueError, match="unknown primitive"):
        from_object({"primitive": "torus", "id": "X", "role": "workpiece",
                     "object_id": 3, "T_world_part": np.eye(4).tolist()})


def test_vocabulary_is_complete():
    assert set(PRIMITIVES) == {"slab", "prism", "prepared_slab", "prepared_prism",
                               "tube", "swept_slab"}


# ------------------------------------------------------------------ generated scenes


def _first_scene(gen, cfg, seeds=range(1, 40)):
    for seed in seeds:
        try:
            return gen(cfg, seed)[0]
        except SceneRejected:
            continue
    pytest.fail("no accepted seed")


def _roundtrip_scene(scene, expect: set[str]):
    kinds = {o["primitive"] for o in scene["objects"]}
    assert expect <= kinds, kinds
    for entry in scene["objects"]:
        rebuilt = from_object(entry)
        assert canonical_json(_serialise(rebuilt)) == canonical_json(entry)
        assert rebuilt.mesh().is_watertight


def test_plate_scene_prisms():
    cfg = load_config(str(ROOT / "configs" / "bench6a_T.yaml"))
    _roundtrip_scene(_first_scene(generate_scene, cfg), {"prism"})


def test_grooved_scene_prepared_prisms():
    cfg = load_config(str(ROOT / "configs" / "grooved_butt.yaml"))
    _roundtrip_scene(_first_scene(generate_scene, cfg), {"prepared_prism"})


def test_curved_scene_tube():
    cfg = load_config(str(ROOT / "configs" / "curved_smoke.yaml"))
    cfg["seam_families"] = [2]
    _roundtrip_scene(_first_scene(generate_curved_scene, cfg), {"tube", "slab"})
