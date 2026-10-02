"""The multi-view JOINT REFINEMENT of touching parts (admittance_control/multiview_refine.py;
step 3 of notes/multiview_refine_plan.md). Plain pytest, no ROS.

Synthetic views of the bench T (the real CAD at its 2026-10-01 pose): every camera sees
only the faces turned towards it, nothing behind another part, with 0.5 mm depth noise,
and optionally an extrinsic translation error d (camera frame) that shifts each view's
cloud by R_cam d - the error that turns with the wrist. Claims:
  * no camera error: every MEASURED direction lands on the truth (base height and tilt,
    the ear's position across its face - the weld root), unmeasured ones (the base's
    in-plane slide, the ear's slide along its length) stay at the saved pose and are
    reported as such; both parts accepted;
  * with d: the extrinsic estimate recovers d; the joint fit would change the fit-up in
    barely measured directions, so both parts are kept as saved (never a half-refined
    assembly); correcting the views by the estimate (self-calibration, step 7) makes both
    land on the truth;
  * the prior holds a slide no view measures; low views that see the edges measure it;
  * the overlap rule: a part started sunk 2 mm into its neighbour ends < 0.5 mm deep, and
    a real 1 mm gap is kept, not closed;
  * ownership and its dead band: labels as designed; for the T (parts meeting at a right
    angle) they change nothing, the normal gate already separates; for a lap joint
    (parallel faces) ownership plus max_corr <= half the gap is what keeps one plate off
    the other's top face;
  * the rounds converge and do not depend on the order the parts are listed in;
  * the acceptance rules (D7).
"""

from __future__ import annotations


import pathlib
import sys

import numpy as np
import pytest

PKG = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

from admittance_control import multiview as mv  # noqa: E402
from admittance_control import multiview_refine as mr  # noqa: E402
from admittance_control.icp import load_ply_mesh, sample_mesh_surface  # noqa: E402

BASE = np.array([[-0.99432, 0.00976, 0.10594, -0.42685], [0.10569, -0.02313, 0.99413, -0.06161],
                 [0.01216, 0.99968, 0.02196, -0.05357], [0.0, 0.0, 0.0, 1.0]])
EAR = np.array([[0.66784, -0.74381, -0.02711, -0.60938], [-0.74351, -0.66837, 0.02196, 0.16042],
                [-0.03445, 0.00549, -0.99939, 0.056], [0.0, 0.0, 0.0, 1.0]])
T_VIEWS = [(60, 45, 180), (300, 60, 90), (0, 45, 180), (240, 45, 270)]   # what plan_views picked
D_CAM_MM = (3.0, -2.0, 1.0)


# ---------------------------------------------------------------- helpers ----------------
def _ortho(T):
    U, _, Vt = np.linalg.svd(T[:3, :3])
    T = T.copy()
    T[:3, :3] = U @ Vt
    return T


def _part_from_ply(name, T, n=4000, seed=0):
    v, f = load_ply_mesh(PKG / "models" / name)
    p, nn = sample_mesh_surface(v, f, n, np.random.default_rng(seed), return_normals=True)
    return mr.Part(name, p * 1e-3, nn, v * 1e-3, _ortho(T))


def _box_part(name, dims, T, n=4000, seed=0):
    x, y, z = dims
    v = np.array([[0, 0, 0], [x, 0, 0], [x, y, 0], [0, y, 0], [0, 0, z], [x, 0, z], [x, y, z], [0, y, z]], float)
    f = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (2, 3, 7, 6), (1, 2, 6, 5), (0, 4, 7, 3)]
    p, nn = sample_mesh_surface(v, f, n, np.random.default_rng(seed), return_normals=True)
    return mr.Part(name, p, nn, v, T)


def _with_pose(p, T):
    return mr.Part(p.name, p.pts, p.nrm, p.verts, T)


def _move(T, mm=(0, 0, 0), deg=0.0, axis=(0, 0, 1)):
    a = np.asarray(axis, float)
    a /= np.linalg.norm(a)
    R = mr._rodrigues(np.radians(deg) * a)
    c = T[:3, 3]
    return mr._T(R, c - R @ c + np.asarray(mm, float) / 1000) @ T


def _views(parts, poses, views=T_VIEWS, d_cam_mm=(0, 0, 0), noise_mm=0.5, n_per_view=3000, seed=1,
           keep=None, distance=0.4):
    rng = np.random.default_rng(seed)
    P, N = [], []
    for i, (p, T) in enumerate(zip(parts, poses)):
        pm, nm = p.pts, p.nrm
        if keep is not None:
            k = keep(i, pm, nm, T)
            pm, nm = pm[k], nm[k]
        P.append(pm @ T[:3, :3].T + T[:3, 3])
        N.append(nm @ T[:3, :3].T)
    P, N = np.vstack(P), np.vstack(N)
    boxes = [mr.box_of(p, T) for p, T in zip(parts, poses)]
    allp = np.vstack([mr.posed(p, T)[0] for p, T in zip(parts, poses)])
    target = (allp.min(0) + allp.max(0)) / 2
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
        Pv = Pv + Tc[:3, :3] @ (np.asarray(d_cam_mm, float) / 1000)
        out.append(mr.View(Pv, Nv, Tc[:3, :3]))
    return out


def _centre_err_mm(part, T, T_truth):
    c = part.pts.mean(0)
    return ((T[:3, :3] @ c + T[:3, 3]) - (T_truth[:3, :3] @ c + T_truth[:3, 3])) * 1000


@pytest.fixture(scope="module")
def tee():
    base = _part_from_ply("test_objv2_base.ply", BASE, seed=0)
    ear = _part_from_ply("test_objv2_ear.ply", EAR, seed=1)
    truth = [base.T_saved, ear.T_saved]
    saved = [_move(truth[0], mm=(1.5, -1.0, 1.0), deg=0.4, axis=(1, 1, 0)),
             _move(truth[1], mm=(-1.0, 1.5, -0.5), deg=0.5, axis=(0, 1, 1))]
    parts = [_with_pose(base, saved[0]), _with_pose(ear, saved[1])]
    return parts, truth


# ---------------------------------------------------------------- the T ------------------
def test_no_camera_error_measured_directions_land_on_truth(tee):
    parts, truth = tee
    views = _views(parts, truth)
    res, diag = mr.refine_assembly(parts, views)
    base, ear = res
    assert base.accepted and ear.accepted, (base.reason, ear.reason)
    up, ear_n, ear_len = BASE[:3, 1], EAR[:3, 1], EAR[:3, 0]
    # base: tilt and height measured -> truth; in-plane slide unmeasured -> held at the saved pose
    assert mr.pose_delta(base.T, truth[0], parts[0].pts.mean(0))[1] < 0.1
    e = _centre_err_mm(parts[0], base.T, truth[0])
    assert abs(e @ up) < 0.5
    held = _centre_err_mm(parts[0], base.T, parts[0].T_saved)
    assert np.linalg.norm(held - (held @ up) * up) < 0.3
    assert any(w.startswith("slide") for w in base.weak)
    # ear: tilt and the position ACROSS its face (where the weld root is) -> truth
    assert mr.pose_delta(ear.T, truth[1], parts[1].pts.mean(0))[1] < 0.1
    assert abs(_centre_err_mm(parts[1], ear.T, truth[1]) @ ear_n) < 0.5
    # ... its slide along its own length is not measured and stays where it was saved
    assert abs(_centre_err_mm(parts[1], ear.T, parts[1].T_saved) @ ear_len) < 0.3
    assert np.linalg.norm(diag.extrinsic_d_mm) < 0.3
    assert diag.rounds_run <= 2
    assert ear.view_spread_mm < 0.3                         # the views agree


def test_camera_error_estimated_rejected_and_self_calibrated(tee):
    parts, truth = tee
    # the planned T views (45/60 deg) determine every direction of d to well under 0.5 mm
    views = _views(parts, truth, d_cam_mm=D_CAM_MM)
    no_online = mr.RefineConfig(online_selfcal=False)
    res, diag = mr.refine_assembly(parts, views, no_online)
    assert np.allclose(diag.extrinsic_d_mm, D_CAM_MM, atol=0.5), diag.extrinsic_d_mm
    assert diag.extrinsic_weak == [] and diag.extrinsic_sigma_mm.max() < 0.3
    # without the online correction the disagreeing views reject the refinement: both kept
    # as saved, never a half-refined assembly
    for r, p in zip(res, parts):
        assert not r.accepted and "disagree" in r.reason and np.allclose(r.T, p.T_saved)
    # the default (online self-calibration) takes d out of the views and refines again
    res_on, diag_on = mr.refine_assembly(parts, views)
    assert np.allclose(diag_on.applied_d_mm, D_CAM_MM, atol=0.5) and np.linalg.norm(diag_on.residual_d_mm) < 0.3
    # self-calibration: take the estimate back out of every view and refine again. (Saved
    # poses 1 mm off here: the fixture's independent ~3 mm errors change the ear-base
    # fit-up by up to 5 mm, which the 2 mm fit-up rule rightly refuses; poses from one
    # real scan share its camera error and their fit-up is far more consistent.)
    d = diag.extrinsic_d_mm / 1000
    fixed = [mr.View(v.pts - v.R_cam @ d, v.nrm, v.R_cam) for v in views]
    near = [_with_pose(parts[0], _move(truth[0], mm=(0.6, -0.4, 0.8), deg=0.2, axis=(1, 1, 0))),
            _with_pose(parts[1], _move(truth[1], mm=(-0.4, 0.6, -0.5), deg=0.2, axis=(0, 1, 1)))]
    res2, diag2 = mr.refine_assembly(near, fixed)
    assert diag2.extrinsic_weak == []
    assert all(r.accepted for r in res2), [r.reason for r in res2]
    assert abs(_centre_err_mm(parts[0], res2[0].T, truth[0]) @ BASE[:3, 1]) < 0.5
    assert abs(_centre_err_mm(parts[1], res2[1].T, truth[1]) @ EAR[:3, 1]) < 0.5
    assert np.linalg.norm(diag2.extrinsic_d_mm) < 0.5


def test_rounds_converge_and_order_does_not_matter(tee):
    parts, truth = tee
    views = _views(parts, truth)
    res_a, diag = mr.refine_assembly(parts, views, per_view=False)
    res_b, _ = mr.refine_assembly(parts[::-1], views, per_view=False)
    assert diag.rounds_run <= 2
    for ra in res_a:
        rb = next(r for r in res_b if r.name == ra.name)
        p = next(p for p in parts if p.name == ra.name)
        dmm, ddeg = mr.pose_delta(ra.T, rb.T, p.pts.mean(0))
        assert dmm < 0.3 and ddeg < 0.05


def test_ownership_dead_band_labels_and_no_effect_on_the_tee(tee):
    parts, truth = tee
    views = _views(parts, truth)
    pts = np.vstack([v.pts for v in views])
    cfg = mr.RefineConfig()
    owner = mr.ownership(pts, parts, truth, cfg)
    d = np.stack([mr.NNIndex(mr.posed(p, T)[0]).query(pts)[1] for p, T in zip(parts, truth)], axis=1)
    both_near = np.sort(d, axis=1)[:, 1] < cfg.dead_band_m
    assert both_near.any() and np.all(owner[both_near] == -1)            # the strip where they meet
    clear = (~both_near) & (d.min(axis=1) < 0.002)
    assert np.all(owner[clear] == np.argmin(d[clear], axis=1))
    # for parts meeting at a right angle the normal gate already separates them: with or
    # without ownership and dead band the result is the same
    a, _ = mr.refine_assembly(parts, views, per_view=False)
    b, _ = mr.refine_assembly(parts, views, mr.RefineConfig(dead_band_m=0.0), use_ownership=False, per_view=False)
    for ra, rb, p in zip(a, b, parts):
        assert mr.pose_delta(ra.T, rb.T, p.pts.mean(0))[0] < 0.2


# ---------------------------------------------------------------- prior and edges --------
def test_prior_holds_an_unmeasured_slide_and_low_views_measure_it():
    base = _part_from_ply("test_objv2_base.ply", BASE, n=8000, seed=0)
    truth = base.T_saved
    edge = BASE[:3, 0]                                       # the base's long edge, in the table plane
    saved = _move(truth, mm=3.0 * edge)
    part = _with_pose(base, saved)
    # 45 deg views from 0.4 m see the 8 mm edges at > 60 deg incidence: the slide is unmeasured
    res, _ = mr.refine_assembly([part], _views([base], [truth]), per_view=False)
    assert abs(_centre_err_mm(base, res[0].T, saved) @ edge) < 0.3                   # held
    assert any("slide" in w and abs(np.array(eval(w.split("along ")[1])) @ edge) > 0.9 for w in res[0].weak)
    # add low views (20 deg) that look at the edges: now it is measured and corrected.
    # (Low views ALONE see no top face - fitness 0.04, rightly rejected.)
    low = T_VIEWS + [(az, 20, 0) for az in (0, 90, 180, 270)]
    res2, _ = mr.refine_assembly([part], _views([base], [truth], views=low), per_view=False)
    assert res2[0].accepted, res2[0].reason
    assert abs(_centre_err_mm(base, res2[0].T, truth) @ edge) < 0.5
    assert not any("slide" in w and abs(np.array(eval(w.split("along ")[1])) @ edge) > 0.9 for w in res2[0].weak)


# ---------------------------------------------------------------- the overlap rule -------
def _no_ear_top(i, pm, nm, T):
    """Drop the ear's top edge from the scene: its height is then unmeasured, so only the
    prior and the overlap rule decide it."""
    if i != 1:
        return np.ones(len(pm), bool)
    return np.abs((nm @ T[:3, :3].T)[:, 2]) < 0.9


def test_overlap_rule_no_sinking_and_a_real_gap_is_kept(tee):
    parts, truth = tee
    base = _with_pose(parts[0], truth[0])
    # ear started 2 mm down INTO the base (truth: standing on it)
    ear = _with_pose(parts[1], _move(truth[1], mm=(0, 0, -2.0)))
    views = _views([base, ear], truth, keep=_no_ear_top)
    res, _ = mr.refine_assembly([base, ear], views, per_view=False)
    nb = [mr.box_of(base, res[0].T)]
    contacts = mr.contact_samples(ear, res[1].T, nb, mr.RefineConfig())
    sd = mr.signed_distance_box(contacts[:, :3] @ res[1].T[:3, :3].T + res[1].T[:3, 3], nb[0])[0]
    assert -sd.min() < 0.0005 + 1e-4                       # no deeper than the tolerance
    # a REAL 1 mm gap (the ear's truth floats 1 mm above the base) is kept, not closed
    truth_gap = [truth[0], _move(truth[1], mm=(0, 0, 1.0))]
    ear_g = _with_pose(parts[1], truth_gap[1])
    res_g, _ = mr.refine_assembly([base, ear_g], _views([base, ear_g], truth_gap, keep=_no_ear_top), per_view=False)
    assert abs(_centre_err_mm(ear_g, res_g[1].T, truth_gap[1])[2]) < 0.3


# ---------------------------------------------------------------- a lap joint -------------
def test_lap_joint_parallel_faces():
    """Two 8 mm plates, the upper one lying half on the lower one: their top faces are
    parallel and 8 mm apart, which the normal gate cannot separate. Findings (2026-10-01):
    the feared swap (the upper plate pulled onto the lower plate's exposed top) does not
    happen - not even without ownership at a 10 mm matching distance - because that top
    lies BESIDE the upper plate, never under its visible top; and capping the matching
    distance at half the gap (`parallel_gap_rule`) costs correction range. So the rule is
    off by default and the gap is only reported."""
    lo = _box_part("lower", (0.15, 0.10, 0.008), mr._T(np.eye(3), np.array([-0.60, 0.0, -0.05])), seed=3)
    up = _box_part("upper", (0.15, 0.10, 0.008), mr._T(np.eye(3), np.array([-0.525, 0.02, -0.042])), seed=4)
    truth = [lo.T_saved, up.T_saved]
    views = _views([lo, up], truth, views=[(az, 50, 0) for az in (0, 90, 180, 270)])
    dirs = np.array([v.R_cam[:, 2] for v in views])
    cfg = mr.RefineConfig()
    assert mr.parallel_face_gap([lo, up], truth, cfg, dirs) == pytest.approx(0.008, abs=1e-3)
    assert mr.corr_distance([lo, up], truth, cfg, dirs) == cfg.max_corr_m              # rule off
    rule = mr.RefineConfig(parallel_gap_rule=True)
    assert mr.corr_distance([lo, up], truth, rule, dirs) == pytest.approx(0.004, abs=5e-4)
    for dz in (-3.0, -6.0):                                  # the upper plate saved too low
        parts = [_with_pose(lo, truth[0]), _with_pose(up, _move(truth[1], mm=(0, 0, dz)))]
        res, diag = mr.refine_assembly(parts, views, per_view=False)
        assert res[1].accepted and abs(_centre_err_mm(up, res[1].T, truth[1])[2]) < 0.5
        assert diag.parallel_gap_m == pytest.approx(0.008 + dz / 1000, abs=1e-3)   # at the saved poses
        loose = mr.RefineConfig(dead_band_m=0.0)
        res_n, _ = mr.refine_assembly(parts, views, loose, use_ownership=False, per_view=False)
        if dz == -3.0:                                       # no swap even without ownership
            assert abs(_centre_err_mm(up, res_n[1].T, truth[1])[2]) < 0.5
    # with the rule the 6 mm error is out of reach (4 mm matching): not corrected
    parts = [_with_pose(lo, truth[0]), _with_pose(up, _move(truth[1], mm=(0, 0, -6.0)))]
    res_r, _ = mr.refine_assembly(parts, views, rule, per_view=False)
    assert _centre_err_mm(up, res_r[1].T, truth[1])[2] < -5.0


# ---------------------------------------------------------------- acceptance (D7) --------
def test_acceptance_rules(tee):
    parts, truth = tee
    cfg = mr.RefineConfig()

    def result(i, T, weak=None, **kw):
        r = mr.PartResult(name=parts[i].name, T_saved=truth[i], T=T, fitness=0.8, **kw)
        r.correction_mm, r.correction_deg = mr.pose_delta(truth[i], T, parts[i].pts.mean(0))
        r.weak_vectors = np.zeros((0, 6)) if weak is None else np.asarray(weak, float)
        return r

    base_parts = [_with_pose(parts[0], truth[0]), _with_pose(parts[1], truth[1])]
    # too large a correction / an overlap -> rejected
    rs = [result(0, _move(truth[0], mm=(0, 0, 12.0))), result(1, truth[1], max_penetration_mm=1.5)]
    mr.judge(rs, base_parts, cfg)
    assert not rs[0].accepted and "correction" in rs[0].reason
    assert not rs[1].accepted and "overlaps" in rs[1].reason
    # the ear moved 3 mm relative to the base: along a MEASURED direction -> accepted
    ear_n = EAR[:3, 1]
    rs = [result(0, truth[0]), result(1, _move(truth[1], mm=3.0 * ear_n))]
    mr.judge(rs, base_parts, cfg)
    assert rs[0].accepted and rs[1].accepted
    # ... the same move along a direction the views did NOT measure -> rejected
    weak = [np.concatenate([np.zeros(3), ear_n])]
    rs = [result(0, truth[0]), result(1, _move(truth[1], mm=3.0 * ear_n), weak=weak)]
    mr.judge(rs, base_parts, cfg)
    assert not rs[1].accepted and "did not measure" in rs[1].reason
