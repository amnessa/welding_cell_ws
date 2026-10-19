"""D12 fixture twins for the curved families (2026-10-19, SCHEMA 2.4.0).

Claims under test: turning the fixture on keeps `twin_key` and the workpiece samples (in the
joint frame) exactly, adds one `role: "fixture"` table whose top touches the lowest part
without cutting it, and records how the assembly rests - `flat` on a base plate or strip,
`line` under the saddle's main pipe; turning it OFF reproduces a stored benchmark scene
exactly (the change only appends draws to the placement substream); both arms are
deterministic and schema-valid.
"""

from __future__ import annotations

import json
import pathlib
import sys

import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from weldgen.config import load_config  # noqa: E402
from weldgen.geom import from_object  # noqa: E402
from weldgen.hashing import content_hash  # noqa: E402
from weldgen.scene import SceneRejected  # noqa: E402
from weldgen.scene_curved import generate_curved_scene  # noqa: E402

try:
    import jsonschema
except ImportError:                                        # pragma: no cover
    jsonschema = None

SCHEMA = json.loads((ROOT / "docs" / "scene.schema.json").read_text())
BENCH = ROOT / "out" / "bench_phase4"


def _cfg(family: int, fixture: bool):
    cfg = load_config(str(ROOT / "configs" / "curved_smoke.yaml"))
    cfg["seam_families"] = [family]
    cfg["fixture_present"] = fixture
    return cfg


def _first_emitting_seed(family: int, fixture: bool, start: int = 2000000):
    for seed in range(start, start + 40):
        try:
            return seed, generate_curved_scene(_cfg(family, fixture), seed)
        except SceneRejected:
            continue
    pytest.skip(f"no emitting seed for family {family}")


def _joint_frame(scene, xyz):
    T = np.asarray(scene["T_world_joint"], float)
    return (np.asarray(xyz, float) - T[:3, 3]) @ T[:3, :3]


@pytest.mark.parametrize("family,mode", [(7, "flat"), (4, "line")])
def test_fixture_twin_keeps_geometry_and_rests_the_assembly(family, mode):
    seed, (s_on, a_on) = _first_emitting_seed(family, True)
    s_off, a_off = generate_curved_scene(_cfg(family, False), seed)
    assert s_on["twin_key"] == s_off["twin_key"]
    assert s_on["joint"]["contact_mode"] == mode and s_off["joint"]["contact_mode"] == "free"
    fx = [o for o in s_on["objects"] if o["role"] == "fixture"]
    assert len(fx) == 1 and fx[0]["object_id"] == 255
    assert not any(o["role"] == "fixture" for o in s_off["objects"])
    # workpiece samples: identical in the joint frame (same draws, different world pose)
    m_on, m_off = a_on["cloud.npz:object_id"] != 255, a_off["cloud.npz:object_id"] != 255
    np.testing.assert_allclose(_joint_frame(s_on, a_on["cloud.npz:xyz"][m_on]),
                               _joint_frame(s_off, a_off["cloud.npz:xyz"][m_off]), atol=1e-3)
    # the table top touches the lowest workpiece point and never cuts a part
    table = from_object(fx[0]).mesh()
    top = _joint_frame(s_on, table.vertices)[:, 2].max()
    low = min(_joint_frame(s_on, from_object(o).mesh().vertices)[:, 2].min()
              for o in s_on["objects"] if o["role"] == "workpiece")
    lowest_true = _joint_frame(s_on, a_on["cloud.npz:xyz"][m_on])[:, 2].min()
    assert top <= lowest_true + 1e-3                  # no part sample under the table top
    assert low - top < 0.3                            # contact within the D34 chord budget
    # same seam truth, verdicts may only lose weldability to the table
    assert [m["face_pair"] for m in s_on["seams"]] == [m["face_pair"] for m in s_off["seams"]]
    for a, b in zip(s_off["seams"], s_on["seams"]):
        assert a["length_mm"] == pytest.approx(b["length_mm"])
        assert b["weldable"] <= a["weldable"]


def test_fixture_arm_is_deterministic_and_schema_valid():
    seed, (s1, a1) = _first_emitting_seed(7, True)
    s2, a2 = generate_curved_scene(_cfg(7, True), seed)
    assert content_hash(s1, a1) == content_hash(s2, a2)
    assert s1["schema_version"] == "2.4.0"
    if jsonschema is not None:
        jsonschema.validate(s1, SCHEMA)
        s_line = dict(s1); s_line["joint"] = {**s1["joint"], "contact_mode": "line"}
        jsonschema.validate(s_line, SCHEMA)           # the 2.4.0 member is accepted
        s_old = dict(s1); s_old["schema_version"] = "2.3.0"
        jsonschema.validate(s_old, SCHEMA)            # 2.3.0 scenes stay valid


@pytest.mark.skipif(not BENCH.exists(), reason="tier-1 benchmark not on disk")
def test_fixture_off_reproduces_a_stored_benchmark_scene():
    from baselines.dataset import load_scene
    rows = [json.loads(ln) for ln in (BENCH / "butt" / "index.jsonl").read_text().splitlines()]
    r = next(r for r in rows if r["source"] == "arc" and r.get("emitted"))
    s, a = generate_curved_scene(_cfg(7, False), r["seed"])
    st, sa = load_scene(BENCH / "butt" / r["scene_id"])
    drop = ("schema_version", "provenance", "mps", "tacks")   # version bump + rule blocks applied later
    assert json.dumps({k: v for k, v in s.items() if k not in drop}, sort_keys=True) == \
        json.dumps({k: v for k, v in st.items() if k not in drop}, sort_keys=True)
    assert sorted(a) == sorted(sa) and all(np.array_equal(a[k], sa[k]) for k in a)
