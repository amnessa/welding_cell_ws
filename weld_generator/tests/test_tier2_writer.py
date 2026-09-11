"""Phase 8 M3 - draws, masks, writer, render hash: pure, no renderer.

Claims: the render id is a function of the resolved config and the background set only;
draws are a function of (scene_id, render_id); the seam mask lands on the projected seam
at the physical width and disappears behind an occluder; the writer round-trips every
array within the depth quantum; `render.sha256` is reproducible from the files on disk;
and writing a render never touches the tier-1 files.
"""

from __future__ import annotations

import hashlib
import json
import pathlib
import sys

import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from weldgen.camera import look_at, project  # noqa: E402
from weldgen.geom import Slab  # noqa: E402
from weldgen.render.config import hashable_config, load_render_config, render_id  # noqa: E402
from weldgen.render.conventions import LABEL_ENV, object_label  # noqa: E402
from weldgen.render.draws import draw_appearance, draw_views, render_rng  # noqa: E402
from weldgen.render.masks import seam_and_tack_masks  # noqa: E402
from weldgen.render.writer import read_view, render_hash, verify_render, write_render, write_view  # noqa: E402

W, H, F = 320, 180, 240.0
K = [[F, 0, W / 2], [0, F, H / 2], [0, 0, 1.0]]


def _fake_backgrounds():
    return {"set_hash": "abc123", "_dir": "/nowhere", "_panoramas": ["/nowhere/p1.jpeg", "/nowhere/p2.jpeg"],
            "photos": [{"file": "bg1.jpeg", "sha256": "0" * 16, "span_m": 0.5, "width_px": 2000, "height_px": 1500},
                       {"file": "bg2.jpeg", "sha256": "1" * 16, "span_m": 1.5, "width_px": 1500, "height_px": 2000}]}


def _cfg():
    return load_render_config(ROOT / "configs" / "render" / "lab_v1.yaml")


def _scene_with_seam():
    """Slab A on the ground, a straight weldable seam along its +u edge, camera looking down."""
    slab = Slab("A", "workpiece", 0, (200.0, 120.0, 8.0), np.eye(4))
    T = look_at(np.array([0.0, -250.0, 300.0]), np.array([0.0, 0.0, 4.0]))
    n = 1001
    pts = np.column_stack([np.linspace(-100, 100, n), np.full(n, 60.0), np.full(n, 4.0)])
    seams = {"seam_0": pts.astype(np.float32), "seam_0_s": np.linspace(0, 200, n).astype(np.float32),
             "seam_0_approach": np.tile([0.0, 1.0, 1.0] / np.sqrt(2), (n, 1)).astype(np.float32)}
    scene = {"scene_id": "deadbeef-0000000001", "camera": {"K": K, "T_world_cam": T.tolist(), "width": W, "height": H,
             "elevation_deg": 50.0, "standoff_mm": 390.0},
             "objects": [{"id": "A", "role": "workpiece", "object_id": 0, "dims_mm": [200.0, 120.0, 8.0],
                          "T_world_part": np.eye(4).tolist()}],
             "seams": [{"id": 0, "weldable": True, "matches_joint_type": True, "length_mm": 200.0, "closed": False}],
             "tacks": {"seam_id": [0, 0], "arclength_mm": [30.0, 170.0], "tack_length_mm": [20.0, 20.0]},
             "noise_model": {"min_z_mm": 50.0}}
    return scene, seams, [slab], T


def test_render_id_depends_on_config_and_background_set_only():
    cfg = _cfg(); bg = _fake_backgrounds()
    a = render_id(cfg, bg)
    cfg2 = dict(cfg); cfg2["environment"] = {**cfg["environment"], "backgrounds_manifest": "/elsewhere/manifest.json"}
    assert render_id(cfg2, bg) == a, "paths must not enter the id"
    bg2 = {**bg, "set_hash": "different"}
    assert render_id(cfg, bg2) != a
    cfg3 = json.loads(json.dumps(cfg)); cfg3["masks"]["seam_width_mm"] = 3.0
    assert render_id(cfg3, bg) != a
    assert "backgrounds_manifest" not in hashable_config(cfg, bg)["environment"]


def test_draws_are_a_function_of_scene_id_and_render_id():
    cfg = _cfg(); bg = _fake_backgrounds(); scene, seams, parts, _ = _scene_with_seam()
    d1 = draw_appearance(render_rng("s1", "r1"), cfg, scene, bg)
    d2 = draw_appearance(render_rng("s1", "r1"), cfg, scene, bg)
    d3 = draw_appearance(render_rng("s2", "r1"), cfg, scene, bg)
    assert d1 == d2
    assert d1 != d3
    assert d1["alloy"] in cfg["materials"]["alloys"] and set(d1["surface_condition"]) == {"A"}
    assert 0.5 * 0.7 <= d1["substrate"]["span_m"] <= 1.5 * 1.3


def test_drawn_views_obey_the_rules_and_are_reproducible():
    cfg = _cfg(); scene, seams, parts, T = _scene_with_seam()
    prim = {0: (seams["seam_0"].astype(float), seams["seam_0_approach"].astype(float))}
    v1 = draw_views(render_rng("s", "r"), cfg, scene, prim, parts, n_views=4)
    v2 = draw_views(render_rng("s", "r"), cfg, scene, prim, parts, n_views=4)
    assert json.dumps(v1) == json.dumps(v2)
    assert v1[0]["view_kind"] == "tier1" and v1[0]["T_world_cam"] == T.tolist()
    dv = cfg["drawn_views"]
    for v in v1[1:]:
        assert v["view_kind"] == "drawn"
        assert dv["elevation_deg"][0] <= v["elevation_deg"] <= dv["elevation_deg"][1]
        assert dv["standoff_mm"][0] <= v["standoff_mm"] <= dv["standoff_mm"][1]
        assert v["best_primary_visible"] >= dv["min_visible_fraction"] and v["best_primary_seam_px"] >= dv["min_seam_px"]
        assert v["attempts"] >= 1


def test_seam_mask_lands_on_the_seam_and_tacks_on_their_intervals():
    scene, seams, parts, T = _scene_with_seam()
    ms, mt, st = seam_and_tack_masks(scene, seams, parts, K, T, W, H, 2.0, 3.0)
    assert st["seams_drawn"] == 1 and st["tacks_drawn"] == 2
    uv, z = project(seams["seam_0"].astype(float), T, np.asarray(K))
    j = np.round(uv[:, 0] - 0.5).astype(int); i = np.round(uv[:, 1] - 0.5).astype(int)
    assert (ms[i, j] == 1).mean() > 0.99, "every projected seam point is inside the seam mask"
    ys, xs = np.nonzero(ms)
    # width: the mask is thin - a few px at this depth (2 mm at ~400 mm with f=240 -> ~1.2 px radius)
    assert 0 < (ms > 0).sum() < 8 * len(np.unique(j))
    # tacks: two disjoint runs of ~20 mm, labelled 1 and 2, inside the seam mask
    assert set(np.unique(mt)) == {0, 1, 2}
    assert np.all(ms[mt > 0] > 0)


def test_occluded_seam_is_not_drawn():
    scene, seams, parts, T = _scene_with_seam()
    blocker = Slab("B", "workpiece", 1, (400.0, 400.0, 4.0), np.eye(4)); blocker.T_world_part[2, 3] = 60.0
    ms, mt, st = seam_and_tack_masks(scene, seams, parts + [blocker], K, T, W, H, 2.0, 3.0)
    assert st["seams_drawn"] == 0 and (ms > 0).sum() == 0 and (mt > 0).sum() == 0


def test_writer_round_trip_and_render_hash(tmp_path):
    scene, seams, parts, T = _scene_with_seam()
    sd = tmp_path / scene["scene_id"]; sd.mkdir()
    (sd / "scene.json").write_text(json.dumps(scene)); (sd / "scene.sha256").write_text("tier1hash\n")
    before = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in sd.iterdir()}
    g = np.random.default_rng(0)
    depth = g.uniform(300, 900, (H, W)); valid = g.random((H, W)) > 0.1
    rgb = g.integers(0, 255, (H, W, 3), dtype=np.uint8)
    ms, mt, _ = seam_and_tack_masks(scene, seams, parts, K, T, W, H)
    mo = np.where(valid, object_label(0), LABEL_ENV).astype(np.uint8)
    entries = [write_view(sd / "views" / "0", rgb, depth, valid, ms, mt, mo, {"view": 0, "view_kind": "tier1", "K": K,
                          "T_world_cam": T.tolist(), "width": W, "height": H})]
    hcfg = hashable_config(_cfg(), _fake_backgrounds())
    meta = {"render_id": "01234567", "scene_id": scene["scene_id"], "backend": {"name": "test", "version": "0"},
            "draws": {"alloy": "brass", "surface_condition": {"A": "ground"}, "substrate": {}, "dome": {}, "key_light": {}},
            "twin_gate": {"pass": True, "residual_p99_mm": 0.0, "coverage": {}, "objects": {}}}
    digest = write_render(sd, meta, entries, hcfg)
    back = read_view(sd / "views" / "0")
    assert np.array_equal(back["valid"], valid) and np.all(np.abs(back["depth_mm"][valid] - depth[valid]) <= 0.025 + 1e-9)
    assert np.array_equal(back["mask_seam"], ms) and np.array_equal(back["mask_tack"], mt) and np.array_equal(back["mask_object"], mo)
    assert np.array_equal(back["rgb"], rgb) and back["meta"]["depth"]["scale_mm"] == 0.05
    ok, got = verify_render(sd)
    assert ok and got == digest == (sd / "render.sha256").read_text().strip()
    after = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in sd.iterdir() if p.is_file() and p.name in before}
    assert after == before, "tier-1 files must be byte-identical after a render"
    import jsonschema
    jsonschema.validate(json.loads((sd / "render.json").read_text()), json.loads((ROOT / "docs" / "render.schema.json").read_text()))
    # RGB is informational: changing it must not change render.sha256
    entries[0]["rgb_sha256"] = "x"
    assert render_hash(entries, hcfg) == digest
