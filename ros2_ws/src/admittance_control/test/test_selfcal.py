"""Extrinsic translation self-calibration (admittance_control/selfcal.py; step 7 of
notes/multiview_refine_plan.md). Plain pytest, no ROS.

Claims: the correction has the right sign through the whole chain (a translation error
Delta in tool0 shows up as R_cam d with d = R_tc^T Delta, and t - R_tc d undoes it); end
to end, three synthetic refine runs of the bench T with such an error, recorded into a
history, yield an extrinsic within 0.5 mm of the true one; and the agreement rules hold
(too few runs, a run that disagrees, other extrinsic files, poor conditioning, no write
without agreement).
"""

from __future__ import annotations

import json
import pathlib
import sys

import numpy as np
import pytest

PKG = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))
sys.path.insert(0, str(PKG / "test"))

from admittance_control import multiview_refine as mr  # noqa: E402
from admittance_control import selfcal as sc  # noqa: E402
import test_multiview_touching_parts as tt  # noqa: E402

DELTA_MM = np.array([2.0, -3.0, 1.5])            # the extrinsic's translation error, tool0 frame


def _extrinsic():
    p = PKG / "notebooks" / "T_tcp_to_cam.npy"
    if p.exists():
        return np.load(p)
    T = np.eye(4)
    T[:3, :3] = mr._rodrigues(np.radians(179.0) * np.array([0.0, 0.0, 1.0]))
    T[:3, 3] = [0.035, 0.089, 0.055]
    return T


def test_correction_sign_through_the_chain():
    rng = np.random.default_rng(0)
    T_true = _extrinsic()
    T_used = T_true.copy()
    T_used[:3, 3] += DELTA_MM / 1000
    for _ in range(5):
        T_bt = mr._T(mr._rodrigues(rng.normal(size=3)), rng.normal(size=3))
        p_c = rng.normal(size=3)
        measured = (T_bt @ T_used @ np.r_[p_c, 1])[:3]
        true = (T_bt @ T_true @ np.r_[p_c, 1])[:3]
        R_cam = T_bt[:3, :3] @ T_used[:3, :3]
        d = T_used[:3, :3].T @ DELTA_MM                 # what the multi-view estimate sees
        assert np.allclose(measured - true, R_cam @ d / 1000)
    assert np.allclose(sc.corrected_extrinsic(T_used, d), T_true)


def _three_runs(tmp_path, views):
    T_true = _extrinsic()
    T_used = T_true.copy()
    T_used[:3, 3] += DELTA_MM / 1000
    ext = tmp_path / "T_tcp_to_cam.npy"
    np.save(ext, T_used)
    hist = tmp_path / "selfcal_history.json"
    d_cam = T_used[:3, :3].T @ DELTA_MM
    base = tt._part_from_ply("test_objv2_base.ply", tt.BASE, seed=0)
    ear = tt._part_from_ply("test_objv2_ear.ply", tt.EAR, seed=1)
    truth = [base.T_saved, ear.T_saved]
    for run, (mm_b, mm_e) in enumerate([((1.5, -1.0, 1.0), (-1.0, 1.5, -0.5)),
                                        ((-1.0, 0.5, -1.5), (0.8, -1.2, 1.0)),
                                        ((0.5, 1.5, 0.8), (1.5, 0.5, -1.0))]):
        parts = [tt._with_pose(base, tt._move(truth[0], mm=mm_b, deg=0.3, axis=(1, 0, 1))),
                 tt._with_pose(ear, tt._move(truth[1], mm=mm_e, deg=0.4, axis=(0, 1, 1)))]
        vs = tt._views([base, ear], truth, views=views, d_cam_mm=d_cam, seed=10 + run)
        _, diag = mr.refine_assembly(parts, vs)
        sc.record_run(hist, diag.extrinsic_d_mm, diag.extrinsic_info, ext, {"run": run})
    return T_true, T_used, ext, hist


def test_three_synthetic_runs_recover_the_extrinsic(tmp_path):
    T_true, T_used, ext, hist = _three_runs(tmp_path, tt.T_VIEWS)
    runs = sc.load_history(hist)
    assert len(runs) == 3 and all(r["extrinsic_sha1"] == sc.file_sha1(ext) for r in runs)
    v = sc.evaluate(runs, sc.file_sha1(ext))
    assert v.ready, v.message
    assert v.undetermined == []
    out = sc.write_selfcal(ext, v, runs, kinematics="test")
    T_new = np.load(out)
    assert np.linalg.norm((T_new[:3, 3] - T_true[:3, 3]) * 1000) < 0.5
    assert np.allclose(T_new[:3, :3], T_used[:3, :3])                    # rotation untouched
    side = json.loads(out.with_suffix(".json").read_text())
    assert side["base_extrinsic_sha1"] == sc.file_sha1(ext) and len(side["runs"]) == 3


def test_an_undetermined_direction_is_left_as_it_was(tmp_path):
    """Runs that cannot see z (its std above max_sigma_mm in every run and combined): the
    correction is applied in x and y only, z is named, the file says so."""
    ext = tmp_path / "T_tcp_to_cam.npy"
    T = _extrinsic()
    np.save(ext, T)
    sha = sc.file_sha1(ext)
    I = np.diag([10.0, 10.0, 0.2])                                 # z: std 2.2 mm per run
    runs = [_run((3.0, -2.0, 1.0), I, sha), _run((3.1, -2.1, 0.0), I, sha), _run((2.9, -1.9, 2.0), I, sha)]
    v = sc.evaluate(runs, sha)
    assert v.ready, v.message
    assert len(v.undetermined) == 1 and abs(v.undetermined[0][2]) > 0.99
    assert np.allclose(v.d_cam_mm, [3.0, -2.0, 0.0], atol=0.05)
    out = sc.write_selfcal(ext, v, runs)
    shift_cam = T[:3, :3].T @ (np.load(out)[:3, 3] - T[:3, 3]) * 1000
    assert np.allclose(shift_cam, [-3.0, 2.0, 0.0], atol=0.05)
    assert json.loads(out.with_suffix(".json").read_text())["undetermined_cam_directions"]


def _run(d, info=None, sha="abc"):
    return {"d_cam_mm": list(d), "info_per_mm2": (np.eye(3) * 10 if info is None else info).tolist(),
            "extrinsic_sha1": sha}


def test_agreement_rules():
    good = [_run((3.0, -2.0, 1.0)), _run((3.2, -1.9, 1.1)), _run((2.9, -2.1, 0.9))]
    assert sc.evaluate(good, "abc").ready
    assert not sc.evaluate(good[:2], "abc").ready                          # too few
    v = sc.evaluate(good + [_run((9.0, 3.0, -4.0))], "abc")
    assert not v.ready and "disagree" in v.message                         # one run far off
    v = sc.evaluate(good[:2] + [_run((3.0, -2.0, 1.0), sha="other")], "abc")
    assert not v.ready and v.skipped == {"other extrinsic": 1}            # other calibration file
    v = sc.evaluate(good[:2] + [_run((3.0, -2.0, 1.0), info=np.zeros((3, 3)))], "abc")
    assert not v.ready and v.skipped == {"determines nothing": 1}
    v = sc.evaluate(good, "abc")
    assert np.allclose(v.d_cam_mm, [3.0333, -2.0, 1.0], atol=0.05) and v.spread_mm < 1.0 and v.loo_mm < 1.0


def test_runs_fill_in_each_others_blind_directions():
    """A run blind along z and one blind along x together determine everything; each is
    judged only in the directions it sees."""
    d = np.array([3.0, -2.0, 1.0])
    I_xy, I_yz = np.diag([10.0, 10.0, 0.0]), np.diag([0.0, 10.0, 10.0])
    runs = [_run(np.diag([1, 1, 0]) @ d, I_xy), _run(np.diag([0, 1, 1]) @ d, I_yz),
            _run(np.diag([1, 1, 0]) @ d + [0.1, 0.0, 0.0], I_xy)]
    v = sc.evaluate(runs, "abc")
    assert v.ready and v.undetermined == []
    assert np.allclose(v.d_cam_mm, d, atol=0.1)


def test_nothing_is_written_without_agreement(tmp_path):
    ext = tmp_path / "T_tcp_to_cam.npy"
    np.save(ext, np.eye(4))
    v = sc.evaluate([_run((1.0, 0.0, 0.0), sha=sc.file_sha1(ext))], sc.file_sha1(ext))
    with pytest.raises(ValueError):
        sc.write_selfcal(ext, v, [])
    assert not (tmp_path / "T_tcp_to_cam_selfcal.npy").exists()
