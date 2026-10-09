"""Collision boxes, reachability and strokes for curved seams (curved_seams_plan.md
step 3). Plain pytest, no ROS.

Claims: a pipe's boxes cover it and stand at most ~1 mm (3.3 % of r) outside its wall,
where one box stood 41 % proud; a mitred pipe's sector boxes hang under the cut by no
more than the cut's rise in one sector; a band's chain of boxes covers its wall within
the sag budget and leaves a rounded-rect tube's interior free; a pipe on a plate in front of the robot gets every tack of its closed seam
reached with a roll per tack, where one roll for the whole seam fails at the second
tack (the wrist would have to turn > 69 deg); a stroke on a curved seam follows the
curve, and `seam` strokes on a curved seam become per-tack sections.
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

trimesh = pytest.importorskip("trimesh")
weldgen = pytest.importorskip("weldgen")

from weldgen.curves import rounded_rect_curve  # noqa: E402

from admittance_control import seam_from_registration as sfr  # noqa: E402
from admittance_control import tack_reach as tr  # noqa: E402
from admittance_control.collision import (CUT_RISE_MM, CollisionModel,  # noqa: E402
                                          boxes_from_parts)
from admittance_control.marking import seam_section, stroke_modes, stroke_targets  # noqa: E402
from admittance_control.tool_model import load_tool_model  # noqa: E402

I4 = np.eye(4).tolist()
SN, CS = np.sin(np.radians(25.0)), np.cos(np.radians(25.0))
REG = {"parts": {
    "plate": {"primitive": "slab", "dims_mm": [256.0, 249.0, 8.0], "T_cad_prim": I4},
    "pipe": {"primitive": "tube", "T_cad_prim": I4,
             "params": {"r_outer_mm": 31.0, "wall_mm": 3.0, "length_mm": 100.0}},
    "mitre": {"primitive": "tube", "T_cad_prim": I4,
              "params": {"r_outer_mm": 45.0, "wall_mm": 5.0, "length_mm": 110.0,
                         "base_cut": {"kind": "plane", "n_local": [SN, 0.0, -CS], "d": 0.0}}},
    "rr": {"primitive": "swept_slab", "T_cad_prim": I4,
           "params": {"spine": rounded_rect_curve([0, 0, 0], [1, 0, 0], [0, 1, 0],
                                                  64.0, 64.0, 12.0).to_parametric(),
                      "offset_lo_mm": 0.0, "offset_hi_mm": 3.0, "z0_mm": 0.0, "z1_mm": 100.0}},
}}


def _pose(t_mm):
    T = np.eye(4); T[:3, 3] = np.asarray(t_mm, float) / 1000.0
    return T


def _grid_points(boxes):
    g = np.stack(np.meshgrid(*[np.linspace(-1, 1, 7)] * 3), -1).reshape(-1, 3)
    return np.vstack([b["centre"] + (b["R"] @ (b["half"] * g).T).T for b in boxes])


def _inside_any(pts, boxes):
    ok = np.zeros(len(pts), bool)
    for b in boxes:
        loc = (pts - b["centre"]) @ b["R"]
        ok |= np.all(np.abs(loc) <= b["half"] + 1e-9, axis=1)
    return ok


def test_pipe_boxes_cover_it_and_stand_about_a_millimetre_proud():
    (pipe,) = sfr.posed_parts([("pipe.ply", np.eye(4))], REG)
    boxes = boxes_from_parts([pipe], scale=1.0)
    surf = pipe.mesh().vertices
    assert _inside_any(np.asarray(surf) * (1 - 1e-6), boxes).all()          # covered
    rho = np.hypot(*_grid_points(boxes)[:, :2].T)
    assert rho.max() - 31.0 == pytest.approx(31.0 * (np.hypot(1, np.sin(np.pi / 12)) - 1), abs=0.01)
    assert rho.max() - 31.0 < 1.1


def test_mitred_pipe_sector_boxes_hug_the_cut():
    (m,) = sfr.posed_parts([("mitre.ply", np.eye(4))], REG)
    boxes = boxes_from_parts([m], scale=1.0)
    assert len(boxes) > 6
    loc = _grid_points(boxes)
    rho = np.hypot(loc[:, 0], loc[:, 1]); phi = np.arctan2(loc[:, 1], loc[:, 0])
    wall = rho >= m.r_inner_mm
    below = m.base_height(phi[wall], np.clip(rho[wall], m.r_inner_mm, m.r_outer_mm)) - loc[wall, 2]
    assert below.max() <= CUT_RISE_MM + 0.5
    v = np.asarray(m.mesh().vertices)
    assert _inside_any(v + 1e-6 * (np.mean(v, axis=0) - v), boxes).mean() > 0.999


def test_band_box_chain_covers_the_tube_within_the_sag():
    (rr,) = sfr.posed_parts([("rr.ply", np.eye(4))], REG)
    boxes = boxes_from_parts([rr], scale=1.0)
    assert 4 < len(boxes) < 40
    v = np.asarray(rr.mesh().vertices)
    assert _inside_any(v, boxes).all()           # the wall only: the interior stays free
    assert not _inside_any(np.array([[0.0, 0.0, 50.0]]), boxes).any()
    # nothing more than the sag budget outside the 64 x 64 footprint
    p = _grid_points(boxes)
    assert np.abs(p[:, :2]).max() <= 32.0 + 1.0 + 1e-6


@pytest.fixture(scope="module")
def pipe_in_front():
    tool, cfg = load_tool_model(), tr.load_marking_config()
    cx, cy, ztop = 550.0, 100.0, 20.0
    objs = [("plate.ply", _pose([cx, cy, ztop - 4.0])), ("pipe.ply", _pose([cx, cy, ztop + 1.0]))]
    parts = sfr.posed_parts(objs, REG)
    seams = sfr.compute_seams(parts)
    tacks = sfr.compute_tacks(parts, seams)["tacks"]
    model = CollisionModel(tool=tool, scene_boxes=boxes_from_parts(parts), table_z=None,
                           clearance=cfg.clearance_m)
    return tool, cfg, model, seams, tacks


def test_closed_seam_round_a_pipe_is_reached_with_a_roll_per_tack(pipe_in_front):
    tool, cfg, model, _, tacks = pipe_in_front
    assert len(tacks) == 4 and all(t["seam_closed"] and t["seam_curved"] for t in tacks)
    report = tr.plan_tacks(tacks, tool, model, cfg)
    (s,) = report["seams"]
    assert s["per_tack"] and s["n_reachable"] == 4 and report["all_ok"], tr.format_report(report)
    assert s["min_clearance_m"] >= cfg.clearance_m
    assert len({round(t["roll_deg"]) for t in report["tacks"]}) > 1          # it varies
    assert "roll per tack" in tr.format_report(report)


def test_one_roll_for_the_whole_loop_fails(pipe_in_front):
    tool, cfg, model, _, tacks = pipe_in_front
    straight = [{**t, "seam_curved": False} for t in tacks]
    report = tr.plan_tacks(straight, tool, model, cfg)
    assert not report["all_ok"]
    assert "joint step" in report["seams"][0]["reason"]


def test_a_curved_stroke_follows_the_seam(pipe_in_front):
    *_, seams, tacks = pipe_in_front
    weld = next(s for s in seams if s["weldable"])
    t = tacks[0]
    sec = seam_section(weld, t["arclength_mm"], t["tack_length_mm"])
    assert np.hypot(sec[:, 0] - 550.0, sec[:, 1] - 100.0) == pytest.approx(31.0, abs=0.05)
    rep = [{"tack_id": tk["id"], "seam_id": tk["seam_id"]} for tk in tacks]
    strokes = stroke_targets("seam", rep, tacks, seams)
    assert all(len(strokes[tk["id"]]) >= 10 for tk in tacks)                  # sections
    assert set(stroke_modes("seam", rep, seams).values()) == {"tack"}
    chord = np.linalg.norm(np.subtract(t["p1_mm"], t["p0_mm"]))
    assert chord < t["tack_length_mm"]                                        # it is an arc
