"""Multi-view refinement of the curved parts (notes/curved_seams_plan.md step 5). Plain
pytest, no ROS.

Synthetic views of the printed parts (the real CADs) - each camera sees only the faces
turned towards it and nothing behind another part's TRUE shape - with 0.5 mm depth noise,
as test_multiview_touching_parts.py renders the plate T. Claims:
  * part_shapes: every CAD vertex lies on its shape (distance 0), the normals point out,
    and the sign agrees with the primitive's own inside test;
  * C1 registered 8 mm off sideways on the plate (the bench's error, 2026-10-14) is
    brought back to within 1 mm by four views round it, both parts accepted, its spin
    about its own axis named as the symmetry it is;
  * S3 on R2, as designed: with the true shapes nothing "penetrates" and both refine;
    with bounding boxes (the old neighbours) S3's saddle flanks, wrapped round R2, sit
    ~5 mm inside R2's box and the joint does not refine;
  * RR1 3 mm off is brought back too (a band: its walls, its interior free);
  * the ISO 5817 fit-up limit takes each part's registered gauge: C1 on the 8 mm plate is
    judged by its 3 mm wall (0.51 mm at level C), not its bounding box (62 mm, 1.62 mm).
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

PKG = pathlib.Path(__file__).resolve().parents[1]
WS = PKG.parents[2]
sys.path.insert(0, str(PKG))
sys.path.insert(0, str(WS / "weld_generator"))

pytest.importorskip("weldgen")

from admittance_control import multiview as mv  # noqa: E402
from admittance_control import multiview_refine as mr  # noqa: E402
from admittance_control.icp import load_ply_mesh, sample_mesh_surface  # noqa: E402
from admittance_control.part_shapes import load_shapes  # noqa: E402

MODELS = PKG / "models"                         # the live library (the CAD check)
# the scenes: the frozen bench CADs (the plate pose below is for the old 256 x 249 x 8 plate)
BENCH = PKG / "test" / "data" / "bench_models"
VIEWS4 = [(0, 45, 0), (90, 60, 0), (180, 45, 0), (270, 60, 0)]


def _need(*names, where=None):
    for n in names:
        if not ((where or BENCH) / n).exists():
            pytest.skip(f"{n} not in {(where or BENCH).name}/")


def _part(name, T, n=4000, seed=0, shapes=None):
    v, f = load_ply_mesh(BENCH / name)
    p, nn = sample_mesh_surface(v, f, n, np.random.default_rng(seed), return_normals=True)
    shape = (shapes or load_shapes(BENCH, [name]))[name]
    return mr.Part(name, p * 1e-3, nn, v * 1e-3, np.asarray(T, float), None, shape)


def _pose(R=None, t_mm=(0, 0, 0)):
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    T[:3, 3] = np.asarray(t_mm, float) / 1000.0
    return T


def _rx(deg):
    a = np.radians(deg); c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _shifted(T, mm):
    T = T.copy()
    T[:3, 3] += np.asarray(mm, float) / 1000.0
    return T


def _views(parts, poses, views=VIEWS4, noise_mm=0.5, n_per_view=3000, seed=1, distance=0.4):
    """Each camera's cloud of the TRUE scene: faces turned to it, occluded by the parts'
    true shapes (their near-exact boxes)."""
    rng = np.random.default_rng(seed)
    P = np.vstack([p.pts @ T[:3, :3].T + T[:3, 3] for p, T in zip(parts, poses)])
    N = np.vstack([p.nrm @ T[:3, :3].T for p, T in zip(parts, poses)])
    boxes = []
    for i, (p, T) in enumerate(zip(parts, poses)):
        boxes += (p.shape.boxes(T, chr(65 + i)) if p.shape is not None else [mr.box_of(p, T)])
    target = (P.min(0) + P.max(0)) / 2
    vc = mv.ViewConfig()
    out = []
    for az, el, roll in views:
        e, a = np.radians(el), np.radians(az)
        cam = target + distance * np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
        Tc = mv.look_at(cam, target, np.radians(roll))
        vis = mv.visible(Tc, P, N, boxes, vc)
        Pv, Nv = P[vis], N[vis]
        if len(Pv) > n_per_view:
            sel = rng.choice(len(Pv), n_per_view, replace=False)
            Pv, Nv = Pv[sel], Nv[sel]
        ray = Pv - cam
        ray /= np.linalg.norm(ray, axis=1)[:, None]
        Pv = Pv + ray * rng.normal(0, noise_mm / 1000, len(Pv))[:, None]
        out.append(mr.View(Pv, Nv, Tc[:3, :3]))
    return out


def _centre_err_mm(part, T, T_truth):
    c = part.pts.mean(0)
    return np.linalg.norm((T[:3, :3] @ c + T[:3, 3]) - (T_truth[:3, :3] @ c + T_truth[:3, 3])) * 1000


# the plate CAD: thickness along CAD y, top face (+w) at CAD y = 0 -> turn y up
PLATE = _pose(_rx(90.0), (-500.0, 80.0, -90.0))


def _on_plate(t_xy_mm, z_up_mm=0.0, R=None):
    """A part's CAD frame standing on the plate's top (world z = -90 mm) at (x, y)."""
    return _pose(R, (t_xy_mm[0], t_xy_mm[1], -90.0 + z_up_mm))


def test_shapes_match_their_cad():
    _need("C1.ply", "E2.ply", "S3.ply", "RR1.ply", "test_objv2_base.ply", where=MODELS)
    import trimesh
    names = ["C1.ply", "E2.ply", "S3.ply", "RR1.ply", "test_objv2_base.ply"]
    shapes = load_shapes(MODELS, names)
    for n in names:
        s = shapes[n]
        assert s is not None, n
        m = trimesh.load(str(MODELS / n), force="mesh")
        sd, nn = s.sdf(np.asarray(m.vertices) / 1000.0)
        assert np.abs(sd).max() < 1e-5, n                                  # on the surface
        assert np.mean(np.einsum("ij,ij->i", nn, np.asarray(m.vertex_normals)) > 0.3) > 0.99, n
        rng = np.random.default_rng(0)
        lo, hi = m.bounds / 1000.0
        q = rng.uniform(lo - 0.01, hi + 0.01, (2000, 3))
        sq, _ = s.sdf(q)
        inside = s.prim.contains(q * 1000.0)
        ok = (sq < 0) == inside
        if s.kind == "Tube":                                 # the bore counts as solid
            T = s.prim.T_world_part
            loc = (q * 1000.0 - T[:3, 3]) @ T[:3, :3]
            ok |= np.hypot(loc[:, 0], loc[:, 1]) <= s.prim.r_inner_mm + 0.5
        ok |= np.abs(sq) < 5e-4
        assert ok.mean() > 0.995, n


def test_the_fitup_limit_takes_the_registered_gauge_not_the_box():
    # until 2026-10-22 a part's t was its box's smallest extent: C1 counted as its 62 mm
    # diameter, the plate's 8 mm set the limit, and a 0.8 mm gap under a 3 mm wall passed
    _need("C1.ply", "test_objv2_base.ply")
    shapes = load_shapes(BENCH, ["C1.ply", "test_objv2_base.ply"])
    plate = _part("test_objv2_base.ply", PLATE, shapes=shapes)
    c1 = _part("C1.ply", _on_plate((-500.0, 80.0)), seed=1, shapes=shapes)
    assert mr._gauge_mm(c1) == pytest.approx(3.0, abs=0.01)
    assert mr._gauge_mm(plate) == pytest.approx(8.0, abs=0.01)
    boxed = mr.Part(c1.name, c1.pts, c1.nrm, c1.verts, c1.T_saved, None, None)
    assert mr._gauge_mm(boxed) == pytest.approx(62.0, abs=0.1)        # the old reading
    res = [mr.PartResult(plate.name, plate.T_saved, plate.T_saved),
           mr.PartResult(c1.name, c1.T_saved, c1.T_saved, gaps_mm={plate.name: (0.6, 0.8)})]
    mr.fitup_warnings(res, [plate, c1], mr.RefineConfig())
    assert res[1].gap_limit_mm[plate.name] == pytest.approx(0.51, abs=0.005)
    assert any("exceeds the ISO 5817" in w and "t 3.0 mm" in w for w in res[1].warnings), res[1].warnings


def test_c1_registered_8_mm_off_sideways_is_brought_back():
    _need("C1.ply", "test_objv2_base.ply")
    shapes = load_shapes(BENCH, ["C1.ply", "test_objv2_base.ply"])
    truth = [PLATE, _on_plate((-500.0, 80.0))]
    plate = _part("test_objv2_base.ply", _shifted(truth[0], (0.0, 0.0, 0.5)), shapes=shapes)
    c1 = _part("C1.ply", _shifted(truth[1], (6.0, -5.3, 0.0)), seed=1, shapes=shapes)   # 8.0 mm
    views = _views([plate, c1], truth)
    res, diag = mr.refine_assembly([plate, c1], views)
    rp, rc = res
    assert rp.accepted and rc.accepted, (rp.reason, rc.reason)
    assert _centre_err_mm(c1, c1.T_saved, truth[1]) == pytest.approx(8.0, abs=0.05)
    assert _centre_err_mm(c1, rc.T, truth[1]) < 1.0
    assert _centre_err_mm(plate, rp.T, truth[0]) < 1.0
    assert any("spin about its own axis" in w for w in rc.weak), rc.weak
    assert rc.max_penetration_mm < 1.0


def _s3_on_r2():
    # S3's CAD frame is the reference: R2's axis on the saddle cut's axis (CAD z through
    # (0, 1.193) mm), then the pair turned so R2 lies along world x and S3 rises from it:
    # S3's body lies along its CAD -y from the cut, so CAD -y is world up
    R = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    W = np.array([-500.0, 80.0, -40.0])
    def world(t_mm):
        return _pose(R, R @ np.asarray(t_mm, float) + W)
    return [world((0.0, 1.193, -55.0)), world((0.0, 0.0, 0.0))]


def test_s3_on_r2_does_not_penetrate_with_the_true_shapes():
    _need("S3.ply", "R2.ply")
    shapes = load_shapes(BENCH, ["R2.ply", "S3.ply"])
    truth = _s3_on_r2()
    r2 = _part("R2.ply", truth[0], shapes=shapes)
    s3 = _part("S3.ply", _shifted(truth[1], (0.0, 1.5, -1.0)), seed=1, shapes=shapes)
    views = _views([r2, s3], truth)
    res, _ = mr.refine_assembly([r2, s3], views)
    assert all(r.accepted for r in res), [r.reason for r in res]
    assert max(r.max_penetration_mm for r in res) < 1.0
    assert _centre_err_mm(s3, res[1].T, truth[1]) < 1.0
    # the old neighbours: each part as its bounding box. S3's saddle flanks, wrapped
    # round R2, sit ~5 mm inside R2's box - five times the 1 mm the overlap rule allows -
    # and the joint is refused (or pushed off R2)
    old = [mr.Part(p.name, p.pts, p.nrm, p.verts, p.T_saved, None, None) for p in (r2, s3)]
    nb = mr.box_of(old[0], truth[0])
    s3_end = s3.pts[s3.pts[:, 1] > -0.06] @ truth[1][:3, :3].T + truth[1][:3, 3]   # the saddle end
    assert -mr.signed_distance_box(s3_end, nb)[0].min() > 0.003
    assert -mr.signed_distance_box(s3_end, mr.box_of(r2, truth[0]))[0].min() < 0.0015   # rests on R2
    res_old, _ = mr.refine_assembly(old, views)
    assert not all(r.accepted for r in res_old) or max(r.max_penetration_mm for r in res_old) > 1.0 \
        or _centre_err_mm(s3, res_old[1].T, truth[1]) > 1.5, [(r.accepted, r.reason) for r in res_old]


def test_rr1_registered_3_mm_off_is_brought_back():
    _need("RR1.ply", "test_objv2_base.ply")
    shapes = load_shapes(BENCH, ["RR1.ply", "test_objv2_base.ply"])
    # RR1 stands along CAD y (0..100, base at y = -100): turn +y up, base on the plate
    truth = [PLATE, _on_plate((-532.0, 48.0), 100.0, _rx(90.0))]
    plate = _part("test_objv2_base.ply", truth[0], shapes=shapes)
    rr = _part("RR1.ply", _shifted(truth[1], (2.0, 2.2, 0.0)), seed=1, shapes=shapes)
    views = _views([plate, rr], truth)
    res, _ = mr.refine_assembly([plate, rr], views)
    assert all(r.accepted for r in res), [r.reason for r in res]
    assert _centre_err_mm(rr, res[1].T, truth[1]) < 1.0


def test_e2_standing_on_its_mitre_refines_and_its_box_did_not():
    """E2 stands tilted on its 25 deg mitre, so the plate's top cuts through E2's tilted
    bounding box: with the old neighbours the plate was pushed 49.7 mm and both parts
    refused, E2 left 5 mm off. With the true shapes E2 comes back."""
    _need("E2.ply", "test_objv2_base.ply")
    import json
    d = json.loads((BENCH / "weldgen_objects.json").read_text())["parts"]["E2"]["params"]["base_cut"]["d"]
    a = np.radians(25.0)
    Ry = np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])
    shapes = load_shapes(BENCH, ["E2.ply", "test_objv2_base.ply"])
    truth = [PLATE, _pose(Ry, (-500.0, 80.0, -90.0 + d))]          # the cut on the plate top
    plate = _part("test_objv2_base.ply", truth[0], shapes=shapes)
    e2 = _part("E2.ply", _shifted(truth[1], (4.0, -3.0, 0.0)), seed=1, shapes=shapes)  # 5 mm off
    assert (e2.pts @ truth[1][:3, :3].T + truth[1][:3, 3])[:, 2].min() == pytest.approx(-0.090, abs=1e-4)
    views = _views([plate, e2], truth)
    res, _ = mr.refine_assembly([plate, e2], views)
    assert all(r.accepted for r in res), [r.reason for r in res]
    assert _centre_err_mm(e2, res[1].T, truth[1]) < 1.0
    old = [mr.Part(p.name, p.pts, p.nrm, p.verts, p.T_saved, None, None) for p in (plate, e2)]
    res_old, _ = mr.refine_assembly(old, views)
    assert not all(r.accepted for r in res_old)


def test_views_are_planned_round_the_bench_c1():
    """The real C1 assembly of 2026-10-14 (archived): the planner, with C1 as its 6 boxes,
    finds views from several sides of the pipe - what pins a round part's position."""
    arch = PKG / "scripts" / "foundationpose_results" / "previous_assemblies" / "20261014-123653"
    if not (arch / "assembly.json").exists():
        pytest.skip("the archived C1 assembly is not on this machine")
    import json
    from admittance_control.collision import CollisionModel
    from admittance_control.tack_reach import load_marking_config
    from admittance_control.tool_model import load_tool_model
    a = json.loads((arch / "assembly.json").read_text())
    objects = [(o["model"], np.asarray(o["pose_static"], float)) for o in a["objects"]]
    surfaces, boxes = mv.surfaces_from_models(objects, BENCH)
    assert sum(1 for b in boxes if b["part"] == 1) == 6                    # C1: 6 boxes
    cfg = load_marking_config(None)
    tool = load_tool_model(None, None)
    if tool.T_tool0_cam is None:
        pytest.skip("no camera extrinsic on this machine")
    model = CollisionModel(tool=tool, scene_boxes=boxes, clearance=cfg.clearance_m,
                           **cfg.collision_kwargs())
    plan = mv.plan_views(surfaces, boxes, tool, model, cfg, cfg.home_q, plan_paths=False)
    assert plan.ok, plan.summary()
    az = sorted(v.azimuth_deg for v in plan.views)
    gaps = np.diff(az + [az[0] + 360.0])
    assert len(plan.views) >= 3 and gaps.max() <= 180.0, plan.summary()  # no side unseen
    print("\n" + plan.summary())


def test_the_judge_applies_corrections_within_the_limits():
    """2026-10-15, the user's rule: a correction within the limits is applied.
      * an overlap over the 1 mm limit refuses a refined pose only when it is WORSE than
        the saved assembly's (E2's saved pose sat in its plate; the better refinement was
        refused and the worse saved pose kept);
      * a pipe's spin about its own axis is not judged by the rotation limit (it does not
        move the seam; E2's 5.13 deg was 5.08 spin + 0.71 tilt)."""
    _need("E2.ply", "test_objv2_base.ply")
    import json
    d = json.loads((BENCH / "weldgen_objects.json").read_text())["parts"]["E2"]["params"]["base_cut"]["d"]
    a = np.radians(25.0)
    Ry = np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])
    shapes = load_shapes(BENCH, ["E2.ply", "test_objv2_base.ply"])
    T_plate = PLATE
    T_e2 = _pose(Ry, (-500.0, 80.0, -90.0 + d))
    plate = _part("test_objv2_base.ply", T_plate, shapes=shapes)
    sunk = lambda mm: _shifted(T_e2, (0.0, 0.0, -mm))                       # noqa: E731
    e2 = _part("E2.ply", sunk(3.0), seed=1, shapes=shapes)                  # saved: 3 mm in
    cfg = mr.RefineConfig()

    def judged(T_e2_refined):
        res = [mr.PartResult("test_objv2_base.ply", T_plate, T_plate.copy(), fitness=0.5),
               mr.PartResult("E2.ply", e2.T_saved, T_e2_refined, fitness=0.5)]
        poses = [T_plate, T_e2_refined]
        for i, r in enumerate(res):
            r.max_penetration_mm = mr.penetration_mm([plate, e2], poses, i, cfg)
            r.correction_mm, r.correction_deg = mr.pose_delta(r.T_saved, r.T, [plate, e2][i].pts.mean(0))
        mr.judge(res, [plate, e2], cfg)
        return res[1]

    better = judged(sunk(2.0))
    assert better.accepted and any("no worse than the saved" in w for w in better.warnings)
    worse = judged(sunk(4.5))
    assert not worse.accepted and "more than the saved" in worse.reason
    # a 6 deg spin about E2's own axis (through its base centre) plus a tiny tilt: applied
    axis_w = T_e2[:3, :3] @ shapes["E2.ply"].prim.T_world_part[:3, 2]
    c = T_e2[:3, :3] @ e2.pts.mean(0) + T_e2[:3, 3]
    R = mr._rodrigues(np.radians(6.0) * axis_w)
    spun = mr._T(R, c - R @ c) @ sunk(3.0)
    r = judged(spun)
    assert r.correction_deg > 5.0 and "over" not in r.reason.split("overlaps")[0], r.reason
    assert any("spin about its own axis" in w for w in r.warnings)
