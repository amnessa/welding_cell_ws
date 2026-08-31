"""Phase 6b step 5 — the D28 gate's curved arm and the D34 sweep.

Claims under test: the rigid-signature provenance test is pose-invariant and
discriminative (that is what lets it match a world-posed seam spine to a part-local
one); a family-7 scene reports NOTHING because both parts are D29-derived from the
seam's own curve, while a family-6 scene reports only against the independent plate's
edges; closed seams are exempt; and the chord sweep flags a violation (binding D34).
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

from qa_d28_gate import (  # noqa: E402
    _rigid_signature, chord_sweep, collect_curved,
)
from weldgen.config import load_config  # noqa: E402
from weldgen.curves import transform_parametric  # noqa: E402
from weldgen.scene import SceneRejected  # noqa: E402
from weldgen.scene_curved import generate_curved_scene  # noqa: E402
from weldgen.writer import write_scene  # noqa: E402


def _emit(family, out_dir, budget=30):
    cfg = load_config(str(ROOT / "configs" / "curved_smoke.yaml"))
    cfg["seam_families"] = [family]
    for seed in range(budget):
        try:
            scene, arrays = generate_curved_scene(cfg, seed)
        except SceneRejected:
            continue
        write_scene(out_dir, scene, arrays)
        return scene
    pytest.fail(f"no family-{family} scene in {budget} seeds")


def test_rigid_signature_is_pose_invariant_and_discriminative():
    arc = {"kind": "arc", "center_mm": [0.0, 0.0, 0.0], "radius_mm": 120.0,
           "t0": -0.5, "t1": 0.7, "u_dir": [1.0, 0.0, 0.0], "v_dir": [0.0, 1.0, 0.0]}
    th = 0.83
    T = np.eye(4)
    T[:3, :3] = [[np.cos(th), -np.sin(th), 0.0],
                 [np.sin(th), np.cos(th), 0.0], [0.0, 0.0, 1.0]]
    T[:3, 3] = [31.0, -12.0, 44.0]
    posed = transform_parametric(arc, T)
    assert _rigid_signature(arc) == _rigid_signature(posed)
    other = dict(arc, radius_mm=121.0)
    assert _rigid_signature(arc) != _rigid_signature(other)


def test_family7_reports_nothing_family6_reports_plate_only(tmp_path):
    _emit(7, tmp_path / "f7")
    rows7, st7 = collect_curved(tmp_path / "f7")
    assert rows7 == [], "both #7 parts derive from the seam curve - no free direction"
    assert st7["provenance_excluded"] > 0 and st7["foreign_spine"] == 0

    scene6 = _emit(6, tmp_path / "f6")
    rows6, st6 = collect_curved(tmp_path / "f6")
    assert st6["provenance_excluded"] > 0 and st6["foreign_spine"] == 0
    assert rows6, "the plate is independent of the drawn spine - it must contribute"
    # every surviving pair can only have come from the slab: total per-seam weight
    # against one edge is that edge's length, and slab edges bound the plate dims
    max_len = max(max(o["dims_mm"][:2]) for o in scene6["objects"]
                  if o["primitive"] == "slab")
    n_pts = {s["sampled"]["n"] for s in scene6["seams"]}
    assert max(w for _, _, w in rows6) <= max_len / min(n_pts) + 1e-6


def test_closed_seams_are_exempt(tmp_path):
    _emit(2, tmp_path)                     # pipe-on-plate: every primary seam closed
    rows, st = collect_curved(tmp_path)
    assert rows == [] and st["closed_exempt"] > 0 and st["open_seams"] == 0


def test_chord_sweep_flags_violation(tmp_path):
    for i, v in enumerate((0.1, 0.4)):
        d = tmp_path / f"s{i}"
        d.mkdir()
        (d / "scene.json").write_text(json.dumps(
            {"scene_id": f"s{i}", "joint": {}, "cloud": {"max_chord_error_mm": v}}))
    n, worst, bad = chord_sweep(tmp_path)
    assert n == 2 and abs(worst - 0.4) < 1e-12
    assert [b[0] for b in bad] == ["s1"]
