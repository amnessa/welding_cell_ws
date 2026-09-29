"""FK from a UR kinematics file. Plain pytest, no ROS.

Claims: the nominal file reproduces the planner's FK and link frames to machine
precision; the robot's calibration file parses and moves the flange by millimetres,
not centimetres (a sanity bound on the parser); and at the scan home the calibrated
chain turns the camera by ~0.5 deg against the nominal one (the mismatch found
2026-09-29).
"""

from __future__ import annotations

import json
import pathlib
import sys

import numpy as np
import pytest

PKG = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

from admittance_control.kinematics import (active_kinematics, ik_solve, load_kinematics,  # noqa: E402
                                           ur5e_fk, ur5e_fk_params, ur5e_link_frames,
                                           ur5e_link_frames_params, use_kinematics)


def test_nominal_file_reproduces_the_planner_fk():
    nom = load_kinematics(PKG / "config" / "default_kinematics.yaml")
    rng = np.random.default_rng(0)
    for _ in range(50):
        q = rng.uniform(-np.pi, np.pi, 6)
        assert np.allclose(ur5e_fk_params(q, nom), ur5e_fk(q), atol=1e-8)
        F1, F2 = ur5e_link_frames(q), ur5e_link_frames_params(q, nom)
        assert all(np.allclose(F1[k], F2[k], atol=1e-8) for k in F1)


def test_calibration_file_is_a_small_perturbation():
    cal = load_kinematics(PKG / "config" / "ur5e_calibration.yaml")
    rng = np.random.default_rng(1)
    gaps = [np.linalg.norm(ur5e_fk_params(q, cal)[:3, 3] - ur5e_fk(q)[:3, 3])
            for q in rng.uniform(-np.pi, np.pi, (50, 6))]
    assert 0.0002 < max(gaps) < 0.01                       # millimetres, not centimetres
    home = np.asarray(json.loads((PKG / "config" / "marking.json").read_text())["home_q"])
    Rn, Rc = ur5e_fk(home)[:3, :3], ur5e_fk_params(home, cal)[:3, :3]
    ang = np.degrees(np.arccos(np.clip((np.trace(Rn.T @ Rc) - 1) / 2, -1, 1)))
    assert 0.3 < ang < 0.8


def test_incomplete_file_is_rejected(tmp_path):
    f = tmp_path / "bad.yaml"
    f.write_text("kinematics:\n  shoulder:\n    x: 0\n")
    with pytest.raises(ValueError):
        load_kinematics(f)


def test_use_kinematics_switches_fk_links_and_ik_then_resets():
    cal_path = PKG / "config" / "ur5e_calibration.yaml"
    cal = load_kinematics(cal_path)
    q = np.array([-0.346, -1.444, -1.553, -1.553, 1.946, -0.86])    # the scan home
    T_nom = ur5e_fk(q)
    try:
        assert use_kinematics(cal_path) == str(cal_path) == active_kinematics()
        assert np.allclose(ur5e_fk(q), ur5e_fk_params(q, cal), atol=1e-12)
        assert np.allclose(ur5e_link_frames(q)["tool0"], ur5e_fk_params(q, cal), atol=1e-12)
        # IK now solves in the calibrated model: its answer reaches the target there
        target = ur5e_fk_params(q, cal)
        sol = ik_solve(target, q + 0.05)
        assert sol is not None and np.allclose(ur5e_fk_params(sol, cal)[:3, 3], target[:3, 3], atol=1e-4)
    finally:
        assert use_kinematics(None) == "nominal"
    assert np.allclose(ur5e_fk(q), T_nom)
