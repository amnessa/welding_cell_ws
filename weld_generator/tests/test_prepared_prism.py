"""`PreparedPrism` — the Phase 6a outline composed with the Phase 6b preparation.

Found 2026-09-07 by the D28 gate on `out/bench_phase4`: the grooved stratum was the one
plate the anti-shortcut outlines never reached, because `_grooved_butt` built
rectangular PreparedSlabs and discarded the outline draws it was handed. Claims under
test: the prism is watertight and winding-consistent for every preparation x outline
shape; its volume is the exact slice integral (the loft is exact, only the U's arc
carries the D34 chord budget); a RECTANGULAR outline reproduces the PreparedSlab to
machine precision (same edge oracle, so same solid); every face's analytic sample sits
on the material boundary with an outward unit normal; the root face still clips a
seam line to the full pinned seam edge; grooved scenes now emit `prepared_prism`
objects that validate, keep `groove_root`, and read `outline` in the D28 gate; and
`polygon_outlines: false` still builds the rectangle.
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
from weldgen.geom import PreparedPrism, PreparedSlab  # noqa: E402
from weldgen.scene import SceneRejected, generate_scene  # noqa: E402
from weldgen.writer import write_scene  # noqa: E402

try:
    import jsonschema
except ImportError:                                        # pragma: no cover
    jsonschema = None

PREPS = {"single_V": {"kind": "single_V", "bevel_deg": 25.0, "root_face_mm": 2.0},
         "single_bevel": {"kind": "single_bevel", "bevel_deg": 45.0,
                          "root_face_mm": 1.5},
         "single_U": {"kind": "single_U", "bevel_deg": 10.0, "root_face_mm": 4.5,
                      "radius_mm": 7.0}}
L, W, T = 200.0, 120.0, 16.0
OUTLINES = {
    "rect": [(L / 2, 0), (-L / 2, 0), (-L / 2, -W), (L / 2, -W)],
    "trapezoid": [(L / 2, 0), (-L / 2, 0), (-L / 2 + 30, -W), (L / 2 - 50, -W)],
    "quad": [(L / 2, 0), (-L / 2, 0), (-L / 2 - 20, -W), (L / 2 + 10, -0.6 * W)],
    "pentagon": [(L / 2, 0), (-L / 2, 0), (-L / 2 - 25, -0.5 * W), (10, -W),
                 (L / 2 + 30, -0.4 * W)],
    "triangle": [(L / 2, 0), (-L / 2, 0), (20, -W)],
}


def _pp(kind, oname, t=T):
    return PreparedPrism("A", "workpiece", 0, np.array(OUTLINES[oname]), t,
                         PREPS[kind], np.eye(4))


def _slice_area(poly):
    return 0.5 * abs(float(np.sum(poly[:, 0] * np.roll(poly[:, 1], -1)
                                  - np.roll(poly[:, 0], -1) * poly[:, 1])))


@pytest.mark.parametrize("kind", list(PREPS))
@pytest.mark.parametrize("oname", list(OUTLINES))
def test_watertight_with_exact_loft_volume(kind, oname):
    pp = _pp(kind, oname)
    m = pp.mesh()
    assert m.is_watertight and m.is_winding_consistent
    ws = np.linspace(-T, 0.0, 2001)
    v_true = float(np.trapezoid([_slice_area(pp._slice(float(v)))
                                 for v in pp.v_edge(ws)], ws))
    tol = 2e-4 if kind == "single_U" else 1e-8          # the arc's D34 chord budget
    assert abs(m.volume - v_true) / v_true < tol
    assert pp.max_chord_error_mm <= 0.25
    assert abs(m.area - pp.surface_area_mm2) / m.area < tol


@pytest.mark.parametrize("kind", list(PREPS))
def test_rectangle_reproduces_the_prepared_slab(kind):
    pp = _pp(kind, "rect")
    ps = PreparedSlab("A", "workpiece", 0, L, W, T, PREPS[kind], np.eye(4))
    assert abs(pp.mesh().volume - ps.mesh().volume) < 1e-6
    for face in ("+w", "-w", "root", "fusion") + (("radius",) if kind == "single_U"
                                                  else ()):
        assert abs(pp.face_area(face) - ps.face_area(face)) < 1e-6, face
    rng = np.random.default_rng(3)
    q = rng.uniform([-L / 2 - 5, -W - 5, -T - 2], [L / 2 + 5, 5, 2], size=(20000, 3))
    assert (pp.contains(q) == ps.contains(q)).all()


@pytest.mark.parametrize("kind", list(PREPS))
@pytest.mark.parametrize("oname", ["trapezoid", "pentagon", "triangle"])
def test_every_face_samples_on_the_boundary_with_outward_normals(kind, oname):
    pp = _pp(kind, oname)
    rng = np.random.default_rng(1)
    for name in pp.face_names():
        pts, nrm = pp.sample_face(name, 600, rng)
        assert np.abs(np.linalg.norm(nrm, axis=1) - 1.0).max() < 1e-9, name
        # 0,02 mm probes; a few samples within that of an oblique neighbour face are
        # allowed to exit through it, which is the probe's limitation not the solid's
        assert pp.contains(pts - 0.02 * nrm).mean() > 0.98, (name, "material inside")
        assert pp.contains(pts + 0.02 * nrm).mean() < 0.02, (name, "void outside")


def test_root_face_clips_a_seam_line_to_the_pinned_seam_edge():
    pp = _pp("single_V", "trapezoid")
    zr = -(T - PREPS["single_V"]["root_face_mm"])
    got = pp.face_clip_line("root", np.array([0.0, 0.0, 0.5 * (-T + zr)]),
                            np.array([1.0, 0.0, 0.0]))
    assert got is not None and abs(got[0] + L / 2) < 1e-9 and abs(got[1] - L / 2) < 1e-9
    # the notched end faces never carry a seam line (the PreparedSlab rule)
    assert pp.face_clip_line("s1", np.zeros(3), np.array([0.0, 0.0, 1.0])) is None


def test_mouth_guard_refuses_a_vertex_inside_the_groove_depth():
    shallow = [(L / 2, 0), (-L / 2, 0), (-L / 2 - 10, -3.0), (0, -W), (L / 2 + 10, -3.0)]
    with pytest.raises(ValueError, match="mouth"):
        PreparedPrism("A", "workpiece", 0, np.array(shallow), T, PREPS["single_V"],
                      np.eye(4))


# ---------------------------------------------------------------- scenes and the gate


def _grooved_scenes(n, budget=80, **cfg_over):
    cfg = load_config(str(ROOT / "configs" / "grooved_butt.yaml"))
    cfg.update(cfg_over)
    out = []
    for seed in range(budget):
        try:
            scene, arrays = generate_scene(cfg, seed)
        except SceneRejected:
            continue
        if scene["joint"]["prep"] != "square":
            out.append((scene, arrays))
        if len(out) >= n:
            return out
    pytest.fail("not enough grooved scenes")


def test_grooved_scenes_emit_prepared_prisms_that_validate_and_keep_groove_root():
    schema = json.loads((ROOT / "docs" / "scene.schema.json").read_text())
    seen = set()
    for scene, arrays in _grooved_scenes(8):
        prims = {o["primitive"] for o in scene["objects"] if o["role"] == "workpiece"}
        seen |= prims
        if jsonschema is not None:
            jsonschema.validate(scene, schema)
        pp = [o for o in scene["objects"] if o["primitive"] == "prepared_prism"]
        for o in pp:
            assert len(o["outline_uv"]) >= 3 and o["outline_shape"]
            assert o["params"]["kind"] == scene["joint"]["prep"] or \
                scene["joint"]["prep"] == "single_bevel"
        cl = next(s for s in scene["seams"]
                  if s["seam_class"] == "butt" and s["weldable"])
        assert f'seams.npz:{cl["sampled"]["array"]}_grooveroot' in arrays
    assert "prepared_prism" in seen


def test_grooved_stratum_reads_outline_in_the_d28_gate(tmp_path):
    from qa_d28_gate import collect
    for scene, arrays in _grooved_scenes(12):
        write_scene(tmp_path, scene, arrays)
    rows = collect(tmp_path)
    mechs = {r[1] for r in rows}
    assert mechs and mechs <= {"outline", "yaw+out"}, mechs
    ang = np.array([r[2] for r in rows])
    ln = np.array([r[3] for r in rows])
    terminal = ln[(ang < 10.0) | (ang > 80.0)].sum() / ln.sum()
    assert terminal < 0.9, "the rectangle read 1,00 - the outline must move it"


def test_outlines_off_still_builds_the_rectangle():
    for scene, _ in _grooved_scenes(2, polygon_outlines=False):
        prims = {o["primitive"] for o in scene["objects"] if o["role"] == "workpiece"}
        assert "prepared_prism" not in prims and "prepared_slab" in prims
