"""Phase 8 M2 — the twin gate and the render conventions, under plain pytest.

No renderer here: a slab and a camera give an ANALYTIC depth image (ray-plane intersection
through pixel centres), and the gate must call that a perfect twin. Then the same depth
with a 1 mm bias must fail the residual statement, and a wrong id buffer must fail the
object statement. The conventions are checked against the pilot's pinned numbers.
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from weldgen.camera import look_at  # noqa: E402
from weldgen.geom import Slab  # noqa: E402
from weldgen.render.conventions import (DEPTH_SCALE_MM, LABEL_ENV, backproject,  # noqa: E402
                                        decode_depth, encode_depth, object_label,
                                        pixel_rays, usd_camera_matrix, usd_intrinsics)
from weldgen.render.gate import THRESHOLDS, twin_gate  # noqa: E402

W, H, F = 160, 90, 120.0
K = np.array([[F, 0, W / 2], [0, F, H / 2], [0, 0, 1.0]])


def _slab():
    return Slab("A", "workpiece", 0, (200.0, 150.0, 8.0), np.eye(4))


def _camera():
    return look_at(np.array([120.0, -160.0, 260.0]), np.array([0.0, 0.0, 4.0]))


def _analytic_view(slab, T):
    """Depth of the slab's TOP face (z = t/2 plane) through pixel centres, else invalid."""
    rays = pixel_rays(K, W, H).reshape(-1, 3) @ T[:3, :3].T
    o = T[:3, 3]; z_top = slab.dims_mm[2] / 2
    s = (z_top - o[2]) / rays[:, 2]
    hit = rays * s[:, None] + o
    L, Wd, _ = slab.dims_mm
    on = (s > 0) & (np.abs(hit[:, 0]) <= L / 2) & (np.abs(hit[:, 1]) <= Wd / 2)
    z_cam = ((hit - o) @ T[:3, :3])[:, 2]          # depth along the optical axis
    depth = np.where(on, z_cam, 0.0).reshape(H, W)
    valid = on.reshape(H, W)
    mask = np.where(valid, object_label(0), LABEL_ENV).astype(np.uint8)
    # a tier-1-like cloud on the top face, all visible
    g = np.random.default_rng(1)
    pts = np.column_stack([g.uniform(-L / 2, L / 2, 4000), g.uniform(-Wd / 2, Wd / 2, 4000), np.full(4000, z_top)])
    cloud = {"xyz": pts.astype(np.float32), "object_id": np.zeros(4000, np.uint8),
             "visible_from_cam": np.ones(4000, bool)}
    scene = {"camera": {"K": K.tolist(), "T_world_cam": T.tolist(), "width": W, "height": H},
             "noise_model": {"min_z_mm": 50.0}}
    return scene, cloud, depth, valid, mask


def test_analytic_view_is_a_perfect_twin():
    slab, T = _slab(), _camera()
    scene, cloud, depth, valid, mask = _analytic_view(slab, T)
    assert valid.sum() > 500
    r = twin_gate(scene, cloud, [slab.mesh()], depth, valid, mask)
    assert r["pass"], r
    assert r["residual_p99_mm"] < 1e-6
    assert r["coverage"]["agreement"] > 0.99 and r["objects"]["agreement"] == 1.0


def test_biased_depth_fails_the_residual_statement():
    slab, T = _slab(), _camera()
    scene, cloud, depth, valid, mask = _analytic_view(slab, T)
    r = twin_gate(scene, cloud, [slab.mesh()], depth + 1.0 * valid, valid, mask)
    assert not r["pass"] and r["residual_p99_mm"] > 0.5


def test_wrong_id_buffer_fails_the_object_statement():
    slab, T = _slab(), _camera()
    scene, cloud, depth, valid, mask = _analytic_view(slab, T)
    wrong = np.where(valid, object_label(1), LABEL_ENV).astype(np.uint8)
    r = twin_gate(scene, cloud, [slab.mesh()], depth, valid, wrong)
    assert not r["pass"] and r["objects"]["agreement"] == 0.0


def test_hidden_points_are_counted_as_disagreement():
    slab, T = _slab(), _camera()
    scene, cloud, depth, valid, mask = _analytic_view(slab, T)
    cloud["visible_from_cam"][:] = False           # tier 1 claims nothing is visible
    r = twin_gate(scene, cloud, [slab.mesh()], depth, valid, mask)
    assert r["coverage"]["agreement"] < 0.05 and not r["pass"]


def test_backproject_uses_pixel_centres():
    T = _camera()
    d = np.full((H, W), 300.0); d[0, 0] = 0.0
    xyz, rows, cols = backproject(d, K, T)
    assert len(xyz) == H * W - 1 and rows[0] == 0 and cols[0] == 1
    cam = (xyz - T[:3, 3]) @ T[:3, :3]
    u = cam[:, 0] / cam[:, 2] * F + W / 2
    assert np.allclose(u, cols + 0.5)


def test_depth_codec_round_trip_within_half_a_quantum():
    d = np.array([[0.0, 12.345, 400.02], [2999.99, np.inf, 5.0]])
    png = encode_depth(d)
    assert png.dtype == np.uint16 and png[0, 0] == 0 and png[1, 1] == 0
    back, valid = decode_depth(png)
    assert valid.sum() == 4 and np.all(np.abs(back[valid] - d[valid]) <= DEPTH_SCALE_MM / 2 + 1e-9)


def test_usd_conventions_match_the_pilot():
    T = np.eye(4); T[:3, 3] = [100.0, -50.0, 400.0]
    U = usd_camera_matrix(T)
    assert np.allclose(U[:3, 3], [0.1, -0.05, 0.4]) and np.allclose(U[:3, :3], np.diag([1, -1, -1]))
    f, ha, va = usd_intrinsics([[450.0, 0, 640], [0, 450.0, 360], [0, 0, 1]], 1280, 720)
    assert np.isclose(f, 450.0 * 20.955 / 1280) and np.isclose(va, 20.955 * 720 / 1280)
    assert object_label(0) == 1 and object_label(255) == 255 and LABEL_ENV == 254


def test_full_gate_on_a_rendered_scene():
    """Runs only under Isaac Python with a corpus scene present."""
    pytest.importorskip("isaacsim")
    sd = ROOT / "out" / "bench_phase4" / "T" / "1ce3c6d2-0002000008"
    if not sd.exists():
        pytest.skip("corpus scene not present")
    import json, subprocess
    r = subprocess.run([sys.executable, str(ROOT / "scripts" / "tier2_gate.py"), str(sd)], capture_output=True, text=True, cwd="/tmp")
    assert "1/1 scenes pass" in r.stdout, r.stdout[-2000:]


def test_point_triangle_distance_regions():
    from weldgen.render.gate import distance_to_mesh, point_triangle_distance
    a, b, c = np.array([[0.0, 0, 0]]), np.array([[10.0, 0, 0]]), np.array([[0.0, 10, 0]])
    cases = {(2.0, 2.0, 3.0): 3.0, (-3.0, 0.0, 4.0): 5.0, (5.0, -2.0, 0.0): 2.0,
             (12.0, 12.0, 0.0): np.hypot(7, 7), (11.0, -1.0, 0.0): np.sqrt(2)}
    for p, want in cases.items():
        assert np.isclose(point_triangle_distance(np.array([p]), a, b, c)[0], want), p
    m = _slab().mesh()
    pts = np.array([[0.0, 0.0, 4.0], [0.0, 0.0, 6.0], [150.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    assert np.allclose(distance_to_mesh(pts, m), [0.0, 2.0, 50.0, 4.0])
