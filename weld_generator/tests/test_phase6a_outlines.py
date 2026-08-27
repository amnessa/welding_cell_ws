"""Phase 6a, second half — polygon outlines (D28) via the Prism primitive.

The claim under test: outlined parts break the seam-parallel-to-boundary prior for the
edge-sharing joints WITHOUT touching anything else — the Prism satisfies the Slab face
interface exactly (clip, closest-point, ray occlusion), the D4 enumeration rediscovers
every joint's seams on prisms and clips the coplanar run to the seam-edge overlap that
hull extents overstate, meshes stay watertight (D21), and corpora generated with
outlines off reproduce bit-identically because outline draws share the `seam_curve`
substream discipline that yaw established.
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from weldgen.config import load_config, sample_joint  # noqa: E402
from weldgen.geom import Prism, Slab, rot_x, translate  # noqa: E402
from weldgen.joints import JointSpec  # noqa: E402
from weldgen.layouts import (  # noqa: E402
    OUTLINE_SHAPES, build, sample_outline,
)
from weldgen.rng import Streams  # noqa: E402


def _prism(outline=None, T=None, t=8.0):
    o = np.array([[-50, 0], [50, 0], [35, -60], [-45, -60]], float) \
        if outline is None else np.asarray(outline, float)
    return Prism("A", "workpiece", 0, o, t, np.eye(4) if T is None else T)


def _spec(joint_type, **kw):
    p = dict(L_A=220.0, W_A=160.0, t_A=8.0, L_B=180.0, H_B=90.0, t_B=6.0,
             root_gap_mm=0.5, linear_misalignment_mm=0.0,
             angular_misalignment_deg=0.0,
             included_angle_deg=JointSpec.NOMINAL_INCLUDED_DEG[joint_type],
             stack_offset_mm=40.0 if joint_type in ("lap",) else
             (0.0 if joint_type == "edge" else None))
    p.update(kw)
    return JointSpec(**p)


def _enumerate(spec, joint_type):
    from weldgen.accessibility import enumerate_candidates
    parts = build(spec, joint_type, np.eye(4))
    access = {k: (v.copy() if isinstance(v, dict) else v)
              for k, v in load_config()["accessibility"].items()}
    access["contact_tol_mm"] = 2.0 * spec.root_gap_mm + 0.5
    return parts, enumerate_candidates(parts, access, joint_type=joint_type)


def _primaries(cands):
    return [c for c in cands if c.weldable and c.matches_joint_type
            and c.p0 is not None and c.length_mm > 1e-6]


def _outlines_for(joint_type, spec, rng):
    kw = {}
    if joint_type in ("corner", "butt", "edge"):
        kw["outline_shape_A"], kw["outline_A"] = sample_outline(
            rng, spec.L_A, spec.W_A)
    kw["outline_shape_B"], kw["outline_B"] = sample_outline(rng, spec.L_B, spec.H_B)
    return kw


# ------------------------------------------------------------------ the primitive


def test_prism_matches_the_slab_interface_where_a_rectangle_is_a_slab():
    """A rectangular prism must agree with the identical slab on every face query the
    pipeline makes — same planes, areas, extents, clips and closest points, with the
    caps carrying the slab's own broad-face names."""
    T = translate(12.0, -7.0, 3.0) @ rot_x(31.0)
    slab = Slab("A", "workpiece", 0, (120.0, 80.0, 6.0), T)
    rect = np.array([[-60, -40], [60, -40], [60, 40], [-60, 40]], float)
    prism = Prism("A", "workpiece", 0, rect, 6.0, T)

    for name in ("+w", "-w"):
        ps, pp = slab.face_plane(name), prism.face_plane(name)
        assert np.allclose(ps.n, pp.n) and abs(ps.d - pp.d) < 1e-9
        assert abs(slab.face_area(name) - prism.face_area(name)) < 1e-9
    assert abs(slab.surface_area_mm2 - prism.surface_area_mm2) < 1e-9

    d = np.array([0.3, 0.9, 0.1])
    d /= np.linalg.norm(d)
    for name in ("+w", "-w"):
        assert np.allclose(slab.face_extent_along(name, d),
                           prism.face_extent_along(name, d), atol=1e-9)
    # side faces exist under different names but jointly cover the same boundary
    side_area_s = sum(slab.face_area(n) for n in ("+u", "-u", "+v", "-v"))
    side_area_p = sum(prism.face_area(f"s{k}") for k in range(4))
    assert abs(side_area_s - side_area_p) < 1e-9

    pts = np.random.default_rng(0).uniform(-150, 150, (200, 3))
    got = prism.closest_on_face("+w", pts)
    want = slab.closest_on_face("+w", pts)
    assert np.allclose(got, want, atol=1e-8)

    line_p = T[:3, 3] + T[:3, :3] @ np.array([0.0, 10.0, 3.0])
    line_d = T[:3, :3] @ np.array([1.0, 0.0, 0.0])
    assert np.allclose(prism.face_clip_line("+w", line_p, line_d),
                       slab.face_clip_line("+w", line_p, line_d), atol=1e-9)


def test_prism_clip_slack_is_millimetres_not_edge_lengths():
    """The parallel-branch slack must compare a UNIT-normal distance: the first
    implementation compared against an edge-length-scaled one, so a 3 mm offset read
    as 300 and every seam line just off a long face was rejected."""
    P = _prism()
    inside = P.face_clip_line("+w", np.array([0.0, -30.0, 4.0]),
                              np.array([1.0, 0.0, 0.0]))
    assert inside is not None and inside[1] - inside[0] > 80.0
    # 3 mm outside the 100 mm seam edge: in with 4 mm slack, out with none
    p = np.array([0.0, 3.0, 4.0])
    d = np.array([1.0, 0.0, 0.0])
    assert P.face_clip_line("+w", p, d) is None
    got = P.face_clip_line("+w", p, d, slack_mm=4.0)
    assert got is not None and got[1] - got[0] > 90.0


def test_prism_mesh_is_watertight_with_outward_winding():
    for k, outline in ((3, [[-50, 0], [50, 0], [10, -70]]),
                       (5, [[-50, 0], [50, 0], [70, -30], [0, -80], [-60, -35]])):
        m = _prism(outline).mesh()
        assert m.is_watertight and m.is_winding_consistent
        assert m.volume > 0.0                     # positive == wound outward
        assert len(m.vertices) == 2 * k


def test_prism_ray_occlusion_agrees_with_the_slab_path():
    T = translate(5.0, 2.0, -1.0) @ rot_x(25.0)
    slab = Slab("A", "workpiece", 0, (120.0, 80.0, 6.0), T)
    rect = np.array([[-60, -40], [60, -40], [60, 40], [-60, 40]], float)
    prism = Prism("A", "workpiece", 0, rect, 6.0, T)
    from weldgen.visibility import ray_hits_convex, ray_hits_slab
    rng = np.random.default_rng(1)
    o = rng.uniform(-200, 200, (4000, 3))
    d = rng.normal(size=(4000, 3))
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    tm = rng.uniform(10, 500, 4000)
    assert (ray_hits_slab(o, d, tm, slab)
            == ray_hits_convex(o, d, tm, prism)).all()


# ------------------------------------------------------------- D4 on outlined joints


@pytest.mark.parametrize("joint_type", ["T", "corner", "butt", "lap", "edge"])
def test_d4_rediscovers_the_seams_of_an_outlined_joint(joint_type):
    """Every joint type keeps its primary seams when the D28 outlines replace the
    rectangles, and the seam direction stays the joint's own axis (the outline varies
    the free boundaries, never the seam-bearing edge)."""
    spec = _spec(joint_type)
    rng = np.random.default_rng(7)
    spec = _spec(joint_type, **_outlines_for(joint_type, spec, rng))
    parts, cands = _enumerate(spec, joint_type)
    assert any(not isinstance(p, Slab) for p in parts)
    prim = _primaries(cands)
    assert prim, f"outlined {joint_type} lost its primary seams"
    for c in prim:
        t = (c.p1 - c.p0) / c.length_mm
        assert abs(t @ np.array([1.0, 0.0, 0.0])) > 0.999, \
            "the seam-bearing edge is pinned by the joint, outline or not"


def test_coplanar_run_is_clipped_to_the_seam_edge_overlap():
    """Two parallelograms sheared the SAME way overhang the seam edge on both ends;
    the hull-extent rule alone would run the butt centreline out to the overhang. The
    run must stop where the seam edges stop."""
    L, sh = 200.0, 60.0
    out_A = ((-L / 2, 0.0), (L / 2, 0.0), (L / 2 + sh, 120.0), (-L / 2 + sh, 120.0))
    out_B = ((-L / 2, 0.0), (L / 2, 0.0), (L / 2 + sh, 100.0), (-L / 2 + sh, 100.0))
    spec = _spec("butt", L_A=L, W_A=120.0, L_B=L, H_B=100.0, t_B=8.0, t_A=8.0,
                 root_gap_mm=1.0, outline_A=out_A, outline_B=out_B,
                 outline_shape_A="parallelogram", outline_shape_B="parallelogram")
    _, cands = _enumerate(spec, "butt")
    prim = _primaries(cands)
    assert prim
    for c in prim:
        u = sorted([float(c.p0[0]), float(c.p1[0])])
        assert u[0] > -L / 2 - 2.0 and u[1] < L / 2 + 2.0, \
            f"run {u} overruns the seam-edge overlap [+-{L / 2}]"


def test_outline_vocabulary_is_convex_reaches_depth_and_pins_the_seam_edge():
    rng = np.random.default_rng(3)
    seen = set()
    for _ in range(300):
        shape, verts = sample_outline(rng, 180.0, 90.0)
        seen.add(shape)
        v = np.asarray(verts)
        assert v[0].tolist() == [-90.0, 0.0] and v[1].tolist() == [90.0, 0.0]
        assert abs(v[:, 1].max() - 90.0) < 1e-9, "every shape must reach full depth"
        k = len(v)
        for i in range(k):
            a, b, c = v[i], v[(i + 1) % k], v[(i + 2) % k]
            e1, e2 = b - a, c - b
            assert e1[0] * e2[1] - e1[1] * e2[0] > 0.0, f"{shape} not convex CCW"
    assert seen == set(OUTLINE_SHAPES)


# --------------------------------------------------------- stream discipline / twins


def test_outlines_off_reproduces_the_pre_outline_stream_draws():
    """With `polygon_outlines` off nothing consumes the outline draws, so specs match
    the pre-6a-outline generator field for field (the yaw guarantee, extended)."""
    cfg = load_config(str(ROOT / "configs" / "bench_corner.yaml"))
    assert not cfg.get("polygon_outlines", False)
    for seed in (0, 3, 11):
        spec, _, _ = sample_joint(cfg, Streams(seed))
        assert spec.outline_A is None and spec.outline_B is None


def test_outlines_on_is_a_twin_in_everything_but_the_outlines():
    """Same seed, outlines toggled: every scalar of the joint draw must agree, because
    outline draws come from `seam_curve` after the yaw draw and nothing else moved."""
    for preset, joint in (("bench_corner", "corner"), ("bench_T", "T")):
        cfg_off = load_config(str(ROOT / "configs" / f"{preset}.yaml"))
        cfg_on = dict(cfg_off)
        cfg_on["polygon_outlines"] = True
        cfg_on["in_plane_yaw"] = True
        for seed in (0, 5):
            off, _, _ = sample_joint(cfg_off, Streams(seed))
            on, _, _ = sample_joint(cfg_on, Streams(seed))
            for f in ("L_A", "W_A", "t_A", "L_B", "H_B", "t_B", "root_gap_mm",
                      "linear_misalignment_mm", "angular_misalignment_deg",
                      "included_angle_deg", "length_offset_mm"):
                assert getattr(off, f) == getattr(on, f), (preset, seed, f)
            assert on.outline_B is not None
            if joint == "corner":
                assert on.outline_A is not None and on.in_plane_yaw_deg == 0.0
            else:
                assert on.outline_A is None       # T keeps A rectangular (yaw covers A)


def test_b_is_outlined_for_every_joint_type_a_only_for_edge_sharing_ones():
    """The gate measured why: B co-rotates with the seam on T/lap, so yaw can never
    decorrelate B's OWN boundary from the seam - only B's outline can."""
    for jt, preset in (("T", "bench_T"), ("lap", "bench_lap"),
                       ("butt", "bench_butt")):
        cfg = load_config(str(ROOT / "configs" / f"{preset}.yaml"))
        cfg["polygon_outlines"] = True
        cfg["in_plane_yaw"] = True
        spec, _, _ = sample_joint(cfg, Streams(2))
        assert spec.outline_B is not None, jt
        assert (spec.outline_A is not None) == (jt in ("corner", "butt", "edge")), jt


# ------------------------------------------------------------------ full pipeline


def test_an_outlined_scene_generates_validates_and_carries_prism_objects():
    from weldgen.scene import generate_scene
    cfg = load_config(str(ROOT / "configs" / "bench6a_butt.yaml"))
    scene, arrays = generate_scene(cfg, 0)
    prisms = [o for o in scene["objects"] if o["primitive"] == "prism"]
    assert len(prisms) == 2
    for o in prisms:
        assert len(o["outline_uv"]) >= 3
        assert o["outline_shape"] in OUTLINE_SHAPES
        assert o["part_geometry_id"].startswith("prism_")
    n_faces = sum(2 + len(o["outline_uv"]) if o["primitive"] == "prism" else 6
                  for o in scene["objects"])
    assert len(scene["faces"]) == n_faces
    assert [f["face_id"] for f in scene["faces"]] == list(range(n_faces))
    fid = arrays["cloud.npz:face_id"]
    assert int(fid.max()) < n_faces

    import json
    import jsonschema
    schema = json.loads((ROOT / "docs" / "scene.schema.json").read_text())
    jsonschema.validate(scene, schema)


def test_modelreg_rebuilds_a_prism_model_from_scene_json():
    sys.path.insert(0, str(ROOT / "scripts"))
    from baselines.lit_modelreg import build_model, prism_edge_points
    from weldgen.scene import generate_scene
    cfg = load_config(str(ROOT / "configs" / "bench6a_corner.yaml"))
    scene, arrays = generate_scene(cfg, 0)
    import re
    gt = [arrays[k] for k in arrays
          if re.fullmatch(r"seams\.npz:seam_\d+", k)]     # D19 nominal curves
    W, P, Twj = build_model(scene, gt)
    assert len(W) > 200 and len(P) >= 1
    # the edge model must trace the outline, not a bounding box: every model point,
    # pushed back to world, lies on the boundary of one of the true parts (analytic
    # face-patch distance - trimesh's mesh distance needs rtree, which D9 forbids)
    parts = build_parts_from(scene)
    world = W @ Twj[:3, :3].T + Twj[:3, 3]
    dmin = np.full(len(world), np.inf)
    for part in parts:
        for face in part.face_names():
            d = np.linalg.norm(world - part.closest_on_face(face, world), axis=1)
            dmin = np.minimum(dmin, d)
    assert float(dmin.max()) < 1.0


def build_parts_from(scene):
    out = []
    for o in scene["objects"]:
        if o["role"] != "workpiece":
            continue
        T = np.asarray(o["T_world_part"])
        if o["primitive"] == "prism":
            out.append(Prism(o["id"], o["role"], o["object_id"],
                             np.asarray(o["outline_uv"]), o["thickness_mm"], T))
        else:
            out.append(Slab(o["id"], o["role"], o["object_id"],
                            tuple(o["dims_mm"]), T))
    return out
