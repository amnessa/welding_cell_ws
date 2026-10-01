"""The multi-view VIEW PLANNER (admittance_control/multiview.py; step 2 of
notes/multiview_refine_plan.md). Plain pytest, no ROS.

Geometry: the bench T as registered on 2026-10-01 (the 250 x 256 x 8 base on its
fixture, the 250 x 99 x 8 ear standing on it diagonally), the real tool model and the
real marking config. Claims: the camera frame looks where it should; the visibility test
honours depth range, incidence and occlusion; the seam region is the strip either side
of the joint, not the touching faces; and the plan for the T has 4 feasible views at
0.40 m (never under the D435i's 0.28 m), sees BOTH faces of the ear, keeps every joint
inside its limits and away from the wrist and elbow singularities, and connects the
views and the way home with collision-free transits that end exactly at home.
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

PKG = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

from admittance_control import multiview as mv  # noqa: E402
from admittance_control.collision import CollisionModel  # noqa: E402
from admittance_control.kinematics import JOINT_LIMITS  # noqa: E402
from admittance_control.tack_reach import load_marking_config  # noqa: E402
from admittance_control.tool_model import load_tool_model  # noqa: E402

BASE = np.array([[-0.99432, 0.00976, 0.10594, -0.42685], [0.10569, -0.02313, 0.99413, -0.06161],
                 [0.01216, 0.99968, 0.02196, -0.05357], [0.0, 0.0, 0.0, 1.0]])
EAR = np.array([[0.66784, -0.74381, -0.02711, -0.60938], [-0.74351, -0.66837, 0.02196, 0.16042],
                [-0.03445, 0.00549, -0.99939, 0.056], [0.0, 0.0, 0.0, 1.0]])
OBJECTS = [("test_objv2_base.ply", BASE), ("test_objv2_ear.ply", EAR)]


@pytest.fixture(scope="module")
def scene():
    surfaces, boxes = mv.surfaces_from_models(OBJECTS, PKG / "models")
    cfg = load_marking_config(None)
    tool = load_tool_model(None, None)
    if tool.T_tool0_cam is None:
        pytest.skip("no camera extrinsic (notebooks/T_tcp_to_cam.npy) on this machine")
    model = CollisionModel(tool=tool, scene_boxes=boxes, table_z=cfg.table_z_m, clearance=cfg.clearance_m)
    return surfaces, boxes, cfg, tool, model


@pytest.fixture(scope="module")
def plan(scene):
    surfaces, boxes, cfg, tool, model = scene
    return mv.plan_views(surfaces, boxes, tool, model, cfg, cfg.home_q)


def test_look_at_frame():
    cam, tgt = np.array([0.3, 0.1, 0.4]), np.array([0.0, 0.0, 0.0])
    for roll in (0.0, 0.7, np.pi):
        T = mv.look_at(cam, tgt, roll)
        R = T[:3, :3]
        assert np.allclose(R.T @ R, np.eye(3), atol=1e-12) and np.linalg.det(R) == pytest.approx(1.0)
        assert np.allclose(R[:, 2], (tgt - cam) / np.linalg.norm(tgt - cam))       # optical axis
    T0 = mv.look_at(cam, tgt, 0.0)
    assert T0[:3, 1] @ np.array([0, 0, -1.0]) > 0                                    # image down ~ world down
    assert np.allclose(mv.look_at(cam, tgt, np.pi / 2)[:3, 0], T0[:3, 1])            # roll turns x onto y


def test_visibility_depth_incidence_occlusion():
    cfg = mv.ViewConfig()
    T = mv.look_at(np.array([0.0, 0.0, 0.40]), np.zeros(3))
    up = np.array([[0.0, 0.0, 1.0]])
    assert mv.visible(T, np.zeros((1, 3)), up, [], cfg)[0]
    assert not mv.visible(T, np.zeros((1, 3)), -up, [], cfg)[0]                      # facing away
    side = np.array([[np.cos(np.radians(70)), 0.0, np.sin(np.radians(70))]])
    assert not mv.visible(T, np.zeros((1, 3)), np.array([[1.0, 0.0, 0.0]]), [], cfg)[0]   # 90 deg incidence
    assert mv.visible(T, np.zeros((1, 3)), side, [], cfg)[0]                         # 20 deg incidence
    near = mv.look_at(np.array([0.0, 0.0, 0.20]), np.zeros(3))
    assert not mv.visible(near, np.zeros((1, 3)), up, [], cfg)[0]                     # under 0.28 m
    blocker = {"name": "b", "type": "box", "centre": np.array([0.0, 0.0, 0.2]), "R": np.eye(3),
               "half": np.array([0.02, 0.02, 0.01]), "group": "scene"}
    assert not mv.visible(T, np.zeros((1, 3)), up, [blocker], cfg)[0]                # occluded
    own = {"name": "own", "type": "box", "centre": np.array([0.0, 0.0, -0.004]), "R": np.eye(3),
           "half": np.array([0.05, 0.05, 0.004]), "group": "scene"}
    assert mv.visible(T, np.zeros((1, 3)), up, [own], cfg)[0]                         # its own face is not in the way


def test_seam_region_is_the_strip_beside_the_joint(scene):
    surfaces, boxes, *_ = scene
    cfg = mv.ViewConfig()
    pts, nrm, w, own = mv.seam_targets(surfaces, boxes, cfg)
    seam = w >= 1.0
    assert seam.sum() > 500 and (~seam).sum() > 1000
    assert set(own.tolist()) == {0, 1}
    d_other = np.where(own == 0, mv._points_box_distance(pts, boxes[1]), mv._points_box_distance(pts, boxes[0]))
    assert np.all(d_other[seam] < cfg.seam_band_m) and np.all(d_other > cfg.contact_gap_m)
    ear_n = EAR[:3, 1]                                         # the ear's thickness axis
    faces = seam & (np.abs(nrm @ ear_n) > 0.9)
    assert (nrm[faces] @ ear_n > 0).any() and (nrm[faces] @ ear_n < 0).any()     # both ear faces


def test_nearest_in_limits():
    ref = np.zeros(6)
    q = np.array([0.1, -0.2, 0.3, -0.4, 0.5, -7.97])
    out = mv.nearest_in_limits(q, np.array([0, 0, 0, 0, 0, -6.0]))
    assert out[5] == pytest.approx(-7.97 + 2 * np.pi)                            # back inside +-2 pi
    assert np.allclose(mv.nearest_in_limits(q[:5].tolist() + [1.0], ref), q[:5].tolist() + [1.0])


def test_plan_for_the_bench_t(plan, scene):
    surfaces, boxes, cfg, tool, model = scene
    vcfg = mv.ViewConfig()
    assert plan.ok, plan.summary()
    assert len(plan.views) == vcfg.n_views
    pts, nrm, w, _ = mv.seam_targets(surfaces, boxes, vcfg)
    seam = w >= 1.0
    ear_n = EAR[:3, 1]
    seen_any = np.any([v.seen for v in plan.views], axis=0)
    for sign in (1.0, -1.0):                                   # BOTH faces of the ear are seen
        face = seam & (sign * nrm @ ear_n > 0.9)
        assert seen_any[face].mean() > 0.5
    assert plan.coverage["seam_1"] > 0.9 and plan.coverage["seam_2"] > 0.3
    sing = np.sin(cfg.wrist_singularity_margin_rad)
    for k, v in enumerate(plan.views):
        assert np.linalg.norm(v.T_cam[:3, 3] - plan.target_m) == pytest.approx(vcfg.distance_m)
        depth = (pts[v.seen] - v.T_cam[:3, 3]) @ v.T_cam[:3, 2]
        assert depth.min() > vcfg.min_depth_m                 # never under the D435i minimum
        assert all(lo <= qk <= hi for qk, (lo, hi) in zip(v.q, JOINT_LIMITS))
        assert abs(np.sin(v.q[4])) >= sing and abs(np.sin(v.q[2])) >= sing
        assert v.clearance_m >= cfg.transit_clearance_m
        assert np.allclose(plan.paths[k][-1], v.q)            # the transit ends at the view
        if k:
            assert np.allclose(plan.paths[k][0], plan.views[k - 1].q)   # ... and starts at the last one
    dirs = np.array([v.direction for v in plan.views])
    cos = dirs @ dirs.T - 2 * np.eye(len(dirs))
    assert cos.max() < np.cos(np.radians(vcfg.min_separation_deg))   # directions kept apart
    assert np.allclose(plan.paths[0][0], cfg.home_q)
    assert plan.home_path is not None and np.allclose(plan.home_path[-1], cfg.home_q)   # exactly home


def test_too_few_views_fails_cleanly(scene):
    surfaces, boxes, cfg, tool, model = scene
    vcfg = mv.ViewConfig(min_depth_m=0.9)                     # every target closer than the minimum
    p = mv.plan_views(surfaces, boxes, tool, model, cfg, cfg.home_q, vcfg, plan_paths=False)
    assert not p.ok and "usable view" in p.reason and not p.views
