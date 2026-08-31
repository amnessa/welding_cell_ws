"""Phase 6b step 4 — groove preparations (D35/D36/D37, D30).

Claims under test: the D35 pool is the verified 9692-1 rows and thickness-correlated
by design; the PreparedSlab is watertight with exact volumes and a monotone exact
profile; grooved butt scenes keep their MOUTH centreline weldable (D37's blanket-
rejection risk, disproved by construction: the nominal is mouth-anchored, so the cone
never starts inside the constriction); `groove_root` sits exactly at t − c below the
top and REPLACES the square-prep-only D19 triple (D36); and the flag off reproduces
pre-6b draws.
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from weldgen.config import (  # noqa: E402
    load_config, sample_groove, sample_joint, valid_preps,
)
from weldgen.geom import PreparedSlab  # noqa: E402
from weldgen.rng import Streams  # noqa: E402
from weldgen.scene import SceneRejected, generate_scene  # noqa: E402


def _prep(kind, **kw):
    base = {"single_V": {"kind": "single_V", "bevel_deg": 25.0, "root_face_mm": 2.0},
            "single_bevel": {"kind": "single_bevel", "bevel_deg": 45.0,
                             "root_face_mm": 1.5},
            "single_U": {"kind": "single_U", "bevel_deg": 10.0,
                         "root_face_mm": 4.5, "radius_mm": 7.0}}[kind]
    base.update(kw)
    return base


def _slab(kind, t=16.0, **kw):
    return PreparedSlab("A", "workpiece", 0, 200.0, 120.0, t,
                        _prep(kind, **kw), np.eye(4))


# ------------------------------------------------------------------ the D35 pool


def test_valid_preps_matches_the_verified_rows():
    assert valid_preps(3.0)[0] == ["square"]
    assert valid_preps(6.0)[0] == ["square", "single_bevel", "single_V"]
    assert valid_preps(9.0)[0] == ["single_bevel", "single_V"]
    assert valid_preps(11.0)[0] == ["single_V"]
    assert valid_preps(16.0)[0] == ["single_V", "single_U"]
    _, refs = valid_preps(8.0)
    assert refs["single_V"] == "1.3" and valid_preps(16.0)[1]["single_V"] == "1.5"
    assert refs["single_bevel"] == "1.9.1"


def test_sample_groove_respects_row_ranges_and_the_u_radius_cap():
    rng = np.random.default_rng(0)
    seen = set()
    for _ in range(200):
        prep, groove, gap = sample_groove(rng, 14.0)
        seen.add(prep)
        if prep == "single_U":
            g = groove
            assert 8.0 <= g["bevel_deg_per_side"] <= 12.0
            assert 1.0 <= gap <= 3.0
            beta = np.radians(g["bevel_deg_per_side"])
            run = 14.0 - g["root_face_mm"] - g["radius_mm"] * (1.0 - np.sin(beta))
            assert run >= 0.5, "U radius must leave a straight fusion face"
        elif prep == "single_V":
            assert 58.0 <= groove["groove_included_angle_deg"] <= 62.0   # row 1.5
    assert seen == {"single_V", "single_U"}


# ------------------------------------------------------------------ the primitive


@pytest.mark.parametrize("kind", ["single_V", "single_bevel", "single_U"])
def test_prepared_slab_is_watertight_with_exact_volume(kind):
    ps = _slab(kind)
    m = ps.mesh()
    assert m.is_watertight and m.is_winding_consistent
    v_true = 200.0 * ps.face_area("+u")          # length x exact cross-section
    assert abs(m.volume - v_true) / v_true < 0.002
    assert ps.max_chord_error_mm <= 0.25         # D34; 0 for straight preps


@pytest.mark.parametrize("kind", ["single_V", "single_bevel", "single_U"])
def test_prepared_slab_faces_bound_the_material_exactly(kind):
    ps = _slab(kind)
    rng = np.random.default_rng(1)
    for name in ps.face_names():
        pts, nrm = ps.sample_face(name, 600, rng)
        assert np.abs(np.linalg.norm(nrm, axis=1) - 1.0).max() < 1e-9, name
        assert ps.contains(pts - 0.05 * nrm).all(), (name, "material inside")
        assert not ps.contains(pts + 0.05 * nrm).any(), (name, "void outside")


def test_u_radius_that_eats_the_fusion_face_is_rejected():
    with pytest.raises(ValueError):
        _slab("single_U", t=10.0, root_face_mm=4.5, radius_mm=9.0)


def test_profile_is_monotone_and_reaches_the_mouth():
    ps = _slab("single_U")
    ws = np.linspace(-16.0, 0.0, 400)
    v = ps.v_edge(ws)
    assert (np.diff(v) <= 1e-12).all(), "the prepared edge must recede monotonically"
    assert abs(v[0]) < 1e-12 and abs(v[-1] - ps.mouth_v_mm) < 1e-9


# --------------------------------------------------------------- scenes and gates


def _grooved_scene(want_kind=None, budget=40):
    cfg = load_config(str(ROOT / "configs" / "grooved_butt.yaml"))
    for seed in range(budget):
        try:
            scene, arrays = generate_scene(cfg, seed)
        except SceneRejected:
            continue
        if scene["joint"]["prep"] == "square":
            continue
        if want_kind is None or scene["joint"]["prep"] == want_kind:
            return scene, arrays
    pytest.fail(f"no {want_kind or 'grooved'} scene in {budget} seeds")


@pytest.mark.parametrize("kind", ["single_V", "single_bevel", "single_U"])
def test_grooved_centreline_is_weldable_from_the_mouth(kind):
    """The D37 receipt: measured beforehand, a 30 deg cone from the ROOT of any V at
    or below 60 deg included is blocked by the bevel walls. The nominal is
    MOUTH-anchored by construction, so the verdict must come back weldable."""
    scene, _ = _grooved_scene(kind)
    cl = [s for s in scene["seams"]
          if s["seam_class"] == "butt" and s["weldable"]]
    assert cl, f"{kind}: the mouth centreline must be weldable"


def test_groove_root_replaces_the_d19_triple_exactly(tmp_path):
    scene, arrays = _grooved_scene()
    g = scene["joint"]["groove"]
    t = scene["objects"][0]["thickness_mm"]
    cl = next(s for s in scene["seams"]
              if s["seam_class"] == "butt" and s["weldable"])
    key = cl["sampled"]["array"]
    assert f"seams.npz:{key}_grooveroot" in arrays
    assert f"seams.npz:{key}_root" not in arrays           # D36: triple is square-only
    assert f"seams.npz:{key}_gapmid" not in arrays
    nom = arrays[f"seams.npz:{key}"]
    gr = arrays[f"seams.npz:{key}_grooveroot"]
    depth = float(np.mean(nom[:, 2] - gr[:, 2]))
    assert abs(depth - (t - g["root_face_mm"])) < 1e-6
    assert scene["joint"]["iso_9692_ref"] == g["iso_ref"]


def test_square_scenes_keep_the_d19_triple():
    cfg = load_config(str(ROOT / "configs" / "grooved_butt.yaml"))
    for seed in range(40):
        try:
            scene, arrays = generate_scene(cfg, seed)
        except SceneRejected:
            continue
        if scene["joint"]["prep"] != "square":
            continue
        cl = next(s for s in scene["seams"]
                  if s["seam_class"] == "butt" and s["weldable"])
        key = cl["sampled"]["array"]
        assert f"seams.npz:{key}_root" in arrays
        assert f"seams.npz:{key}_grooveroot" not in arrays
        return
    pytest.fail("no square scene found")


def test_single_bevel_mouth_centreline_is_asymmetric():
    """Ref 1.9.1 prepares ONE plate: the exposed centreline shifts toward the beveled
    side while groove_root stays at the true root centre - the two curves genuinely
    differ in y, which is exactly why D36 separates them."""
    scene, arrays = _grooved_scene("single_bevel")
    cl = next(s for s in scene["seams"]
              if s["seam_class"] == "butt" and s["weldable"])
    key = cl["sampled"]["array"]
    nom = arrays[f"seams.npz:{key}"]
    gr = arrays[f"seams.npz:{key}_grooveroot"]
    lateral = np.linalg.norm((nom - gr)[:, :2], axis=1)
    assert lateral.mean() > 0.5, "mouth centre must shift off the root centre"


def test_flag_off_reproduces_pre_6b_draws():
    cfg_off = load_config(str(ROOT / "configs" / "bench_butt.yaml"))
    assert not cfg_off.get("groove_preps", False)
    for seed in (0, 5, 11):
        spec, _, _ = sample_joint(cfg_off, Streams(seed))
        assert spec.prep == "square" and spec.groove is None
