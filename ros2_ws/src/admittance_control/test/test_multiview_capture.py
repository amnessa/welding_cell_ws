"""Multi-view captures and their offline replay (admittance_control/multiview_capture.py,
scripts/multiview_refine_offline.py; step 4 of notes/multiview_refine_plan.md). No ROS.

Claims: a capture survives the disk round trip; the synthetic renderer is exact (ray cast
against the part boxes: no depth bias, which a z-buffer of surface samples had - it
looked like an extrinsic error); preprocessing crops to the parts, cuts the table, keeps
normals facing each camera; and the whole replay of a synthetic capture of the bench T
lands within 0.5 mm / 0.2 deg of the truth - with no camera error, and with an
extrinsic error that the run estimates and takes out itself (online self-calibration);
with that switched off, the disagreeing views reject the refinement.
"""

from __future__ import annotations

import json
import pathlib
import subprocess
import sys

import numpy as np
import pytest

PKG = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))
sys.path.insert(0, str(PKG / "scripts"))

from admittance_control import multiview as mv  # noqa: E402
from admittance_control import multiview_capture as mc  # noqa: E402
from admittance_control import multiview_refine as mr  # noqa: E402
import multiview_refine_offline as offline  # noqa: E402

D_CAM_MM = (3.0, -2.0, 1.0)


@pytest.fixture(scope="module")
def caps(tmp_path_factory):
    root = tmp_path_factory.mktemp("captures")
    return offline.make_synthetic(root / "d0", (0, 0, 0)), offline.make_synthetic(root / "d", D_CAM_MM)


def _truth_err(rp):
    out = []
    for r, p, Tt in zip(rp.results, rp.parts, rp.capture.meta["pose_true"]):
        out.append(mr.pose_delta(r.T, np.asarray(Tt), p.pts.mean(0)))
    return out


def test_round_trip(tmp_path):
    xyz = np.random.default_rng(0).normal(size=(6, 8, 3)).astype(np.float32)
    xyz[0, 0] = np.nan
    T = np.eye(4)
    T[:3, 3] = [0.1, -0.2, 0.3]
    p = mc.save_capture(tmp_path / "c", [mc.RawView(xyz, T, {"azimuth_deg": 60.0})],
                        {"objects": []}, {"extrinsic_sha1": "abc"})
    cap = mc.load_capture(p)
    assert np.array_equal(np.isnan(cap.views[0].xyz), np.isnan(xyz))
    assert np.allclose(np.nan_to_num(cap.views[0].xyz), np.nan_to_num(xyz))
    assert np.allclose(cap.views[0].T_base_cam, T) and cap.views[0].meta["azimuth_deg"] == 60.0
    assert cap.meta["extrinsic_sha1"] == "abc" and cap.assembly == {"objects": []}
    assert mc.latest_capture(tmp_path) is None                       # only under <results>/multiview


def test_renderer_is_exact():
    """A 200 x 200 x 8 mm plate, its top 0.4 m straight below the camera: every hit on
    the top reads 0.4 m exactly (no noise), and the plate's outline lands where the
    pinhole puts it."""
    v = np.array([[0, 0, 0], [0.2, 0, 0], [0.2, 0.2, 0], [0, 0.2, 0],
                  [0, 0, 0.008], [0.2, 0, 0.008], [0.2, 0.2, 0.008], [0, 0.2, 0.008]], float)
    plate = mr.Part("plate", v, np.tile([0, 0, 1.0], (8, 1)), v, mr._T(np.eye(3), np.array([-0.1, -0.1, -0.008])))
    T_cam = mv.look_at(np.array([0.0, 0.0, 0.4]), np.zeros(3))
    K = mc.pinhole(200, 120)
    xyz = mc.render_view([plate], [plate.T_saved], T_cam, K, 200, 120, noise_m=0.0)
    z = xyz[..., 2]
    hit = np.isfinite(z)
    assert hit.any() and np.allclose(z[hit], 0.4, atol=1e-9)
    half_px = 0.1 / 0.4 * K[0, 0]                                     # the plate's half-width in pixels
    cols = np.flatnonzero(hit[60])
    assert abs((cols.max() - cols.min() + 1) / 2 - half_px) <= 1.0


def test_preprocess_crops_cuts_and_faces_the_camera(caps):
    cap = mc.load_capture(caps[0])
    parts = mc.parts_from_assembly(cap.assembly, PKG / "models")
    raw = cap.views[0]
    v = mc.preprocess_view(raw, parts)
    cam = raw.T_base_cam[:3, 3]
    assert len(v.pts) > 1000
    assert np.all(np.einsum("ij,ij->i", v.nrm, cam - v.pts) > 0)      # normals face that camera
    boxes = [mr.box_of(p, p.T_saved) for p in parts]
    d = np.min([mv._points_box_distance(v.pts, b) for b in boxes], axis=0)
    assert d.max() < mc.PreprocessConfig().crop_margin_m
    # a table plane just under the base's top: the base top points are cut
    top_z = float(np.percentile(v.pts[:, 2], 30))
    v2 = mc.preprocess_view(raw, parts, plane=(0.0, 0.0, top_z + 0.001))
    assert len(v2.pts) < len(v.pts)


def test_replay_without_camera_error_lands_on_truth(caps):
    rp = mc.refine_capture(mc.load_capture(caps[0]), PKG / "models")
    assert all(r.accepted for r in rp.results), [r.reason for r in rp.results]
    for mm, deg in _truth_err(rp):
        assert mm < 0.5 and deg < 0.2
    assert np.linalg.norm(rp.diag.extrinsic_d_mm) < 0.3 and rp.diag.applied_d_mm is None
    text = mc.format_replay(rp)
    assert "ACCEPTED" in text and "vs the synthetic truth" in text


def test_replay_with_camera_error_self_calibrates_online(caps):
    rp = mc.refine_capture(mc.load_capture(caps[1]), PKG / "models")
    assert np.allclose(rp.diag.extrinsic_d_mm, D_CAM_MM, atol=0.3)
    assert np.allclose(rp.diag.applied_d_mm, D_CAM_MM, atol=0.3)
    assert np.linalg.norm(rp.diag.residual_d_mm) < 0.3
    assert all(r.accepted for r in rp.results), [r.reason for r in rp.results]
    for mm, deg in _truth_err(rp):
        assert mm < 0.5 and deg < 0.2
    # without the online correction the disagreeing views reject the refinement
    off = mc.refine_capture(mc.load_capture(caps[1]), PKG / "models", mr.RefineConfig(online_selfcal=False))
    assert not any(r.accepted for r in off.results)
    assert all("disagree" in r.reason for r in off.results)
    assert all(np.allclose(r.T, r.T_saved) for r in off.results)


def test_offline_script(caps, tmp_path):
    hist = tmp_path / "hist.json"
    out = subprocess.run([sys.executable, str(PKG / "scripts" / "multiview_refine_offline.py"), str(caps[1]),
                          "--write", "--record", "--history", str(hist)], capture_output=True, text=True, timeout=300)
    assert out.returncode == 0, out.stdout + out.stderr
    assert "online self-calibration" in out.stdout
    refined = json.loads((caps[1] / "assembly_refined.json").read_text())
    assert all(o["refine"]["accepted"] for o in refined["objects"])
    runs = json.loads(hist.read_text())["runs"]
    assert len(runs) == 1 and np.allclose(runs[0]["d_cam_mm"], D_CAM_MM, atol=0.3)
    # --set overrides a config field; an unknown one is refused
    bad = subprocess.run([sys.executable, str(PKG / "scripts" / "multiview_refine_offline.py"), str(caps[0]),
                          "--set", "no_such_knob=1"], capture_output=True, text=True, timeout=300)
    assert bad.returncode != 0 and "no such" in (bad.stdout + bad.stderr)


def test_saved_poses_carrying_the_scan_views_error(tmp_path):
    """The bench case of 2026-10-02: the saved poses were registered through the same
    camera from the scan pose, so they carry R_scan d; the views carry R_cam d. Taking d
    out of the views but NOT out of the prior left the base's unmeasured slide at the
    uncorrected saved pose - a fake fit-up change, both parts rejected. With the prior
    corrected too, both are accepted and land on the truth, the slide included."""
    import multiview_refine_offline as off
    d = np.array(D_CAM_MM)
    models = PKG / "models"
    assembly = {"objects": [{"model": "test_objv2_base.ply", "pose_static": off._ortho(off.BASE).tolist()},
                            {"model": "test_objv2_ear.ply", "pose_static": off._ortho(off.EAR).tolist()}]}
    parts = mc.parts_from_assembly(assembly, models)
    truth = [p.T_saved for p in parts]
    allp = np.vstack([mr.posed(p, T)[0] for p, T in zip(parts, truth)])
    target = (allp.min(0) + allp.max(0)) / 2
    scan = mv.look_at(target + np.array([0.15, 0.25, 0.52]), target)          # the scan home, ~0.6 m
    saved = []
    for T in truth:                                                              # registered through the error
        Ts = T.copy()
        Ts[:3, 3] += scan[:3, :3] @ (d / 1000)
        saved.append(Ts)
    cams, metas = [], []
    for az, el, roll in off.T_VIEWS:
        e, a = np.radians(el), np.radians(az)
        cam = target + 0.4 * np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
        cams.append(mv.look_at(cam, target, np.radians(roll)))
        metas.append({"azimuth_deg": az, "elevation_deg": el, "roll_deg": roll})
    path = mc.render_synthetic(tmp_path / "cap", parts, truth, saved, cams, d, view_meta=metas,
                               models_dir=models, scan_cam=scan)
    rp = mc.refine_capture(mc.load_capture(path), models)
    assert all(r.accepted for r in rp.results), [r.reason for r in rp.results]
    for r, p, Tt in zip(rp.results, rp.parts, truth):
        mm, deg = mr.pose_delta(r.T, Tt, p.pts.mean(0))
        assert mm < 0.5 and deg < 0.2, (r.name, mm, deg)
        assert r.prior_shift_mm == pytest.approx(np.linalg.norm(d), abs=0.3)
        assert r.beyond_mm < 0.5
    assert "camera error at its scan pose" in mc.format_replay(rp)

