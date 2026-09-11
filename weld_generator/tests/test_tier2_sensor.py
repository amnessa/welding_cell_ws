"""Phase 8 M4 - the D16 sensor model on rendered depth, under plain pytest.

Claims: validity is deterministic and drops grazing pixels and the blind zone exactly as
`noise.apply` does on a cloud; a realisation is reproducible from its seed and its axial
scatter matches `sigma_z_mm(z)`; the writer stores the sensor validity, hashes it, and
round-trips it.
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from weldgen import noise  # noqa: E402
from weldgen.camera import look_at  # noqa: E402
from weldgen.render.conventions import pixel_rays  # noqa: E402
from weldgen.render.sensor import realise, sensor_validity  # noqa: E402

W, H, F = 200, 120, 160.0
K = np.array([[F, 0, W / 2], [0, F, H / 2], [0, 0, 1.0]])
NM = {"kind": "stereo_z2", "profile": "stereo_poor", "baseline_mm": 35.0, "focal_px": 450.0,
      "subpixel_px": 0.15, "lateral_sigma_px": 0.8, "grazing_dropout_deg": 75.0, "min_z_mm": 400.0, "seed": 7}


def _two_plane_view():
    """A floor (normal +z) and a SIDE wall (x = 150, normal -x) that the rays skim along."""
    T = look_at(np.array([0.0, -600.0, 450.0]), np.array([0.0, 0.0, 100.0]))
    rays = pixel_rays(K, W, H).reshape(-1, 3) @ T[:3, :3].T; o = T[:3, 3]
    depth = np.zeros(H * W); normals = np.zeros((H * W, 3))
    sf = -o[2] / rays[:, 2]; pf = o + rays * sf[:, None]
    onf = (sf > 0) & (np.abs(pf[:, 0]) < 300) & (np.abs(pf[:, 1]) < 300)
    with np.errstate(divide="ignore", invalid="ignore"):
        sw = (150 - o[0]) / rays[:, 0]
    pw = o + rays * sw[:, None]
    onw = (rays[:, 0] > 1e-9) & (sw > 0) & (pw[:, 2] > 0) & (pw[:, 2] < 200) & (np.abs(pw[:, 1]) < 300)
    take_w = onw & (~onf | (sw < sf))
    depth[onf] = ((pf[onf] - o) @ T[:3, :3])[:, 2]; normals[onf] = [0, 0, 1]
    depth[take_w] = ((pw[take_w] - o) @ T[:3, :3])[:, 2]; normals[take_w] = [-1, 0, 0]
    valid = depth > 0
    return T, depth.reshape(H, W), valid.reshape(H, W), normals.reshape(H, W, 3)


def test_validity_matches_noise_apply_on_the_same_points_and_is_deterministic():
    T, depth, valid, normals = _two_plane_view()
    v1 = sensor_validity(depth, valid, normals, K, T, NM); v2 = sensor_validity(depth, valid, normals, K, T, NM)
    assert np.array_equal(v1, v2) and v1.sum() > 0 and v1.sum() < valid.sum()
    # the rule, pixel by pixel: valid <=> incidence < grazing_dropout_deg AND depth > min_z
    rays = pixel_rays(K, W, H) @ T[:3, :3].T
    rays /= np.linalg.norm(rays, axis=-1, keepdims=True)
    cos_inc = np.abs(np.einsum("ijk,ijk->ij", rays, normals))
    want = valid & (cos_inc >= np.cos(np.deg2rad(NM["grazing_dropout_deg"]))) & (depth > NM["min_z_mm"])
    assert np.array_equal(v1, want)
    floor = valid & (normals[..., 2] == 1); wall = valid & (normals[..., 0] == -1)
    assert wall.sum() > 100 and v1[floor].mean() > 0.95 and v1[wall].mean() < v1[floor].mean()
    v3 = sensor_validity(depth, valid, normals, K, T, {**NM, "min_z_mm": 650.0})
    assert not v3[valid & (depth <= 650)].any() and v3[floor & (depth > 650)].mean() > 0.95, "blind zone"


def test_realisation_is_seeded_and_its_axial_scatter_matches_sigma_z():
    T, depth, valid, normals = _two_plane_view()
    a, va = realise(depth, valid, normals, K, T, NM); b, vb = realise(depth, valid, normals, K, T, NM)
    c, _ = realise(depth, valid, normals, K, T, NM, seed=8)
    assert np.array_equal(a, b) and np.array_equal(va, vb) and not np.array_equal(a, c)
    floor = va & (normals[..., 2] == 1)
    dz = a[floor] - depth[floor]; z = depth[floor]
    # axial noise is along the ray; on a floor seen at ~37 deg its z-component is ~cos(view angle)
    # times sigma_z, so compare the ratio to sigma within a loose band rather than exactly
    ratio = dz.std() / noise.sigma_z_mm(np.median(z), NM)
    assert 0.5 < ratio < 1.5, ratio
    assert np.all(a[~va] == 0)


def test_writer_stores_and_hashes_the_sensor_validity(tmp_path):
    from weldgen.render.writer import read_view, render_hash, write_view
    T, depth, valid, normals = _two_plane_view()
    sv = sensor_validity(depth, valid, normals, K, T, NM)
    z = np.zeros((H, W), np.uint8); rgb = np.zeros((H, W, 3), np.uint8)
    e1 = write_view(tmp_path / "0", rgb, depth, valid, z, z, z, {"view": 0, "view_kind": "tier1"}, sensor_valid=sv)
    back = read_view(tmp_path / "0")
    assert np.array_equal(back["sensor_valid"], sv & valid) and np.array_equal(back["valid"], valid)
    assert e1["depth"]["sensor_valid_fraction"] < e1["depth"]["valid_fraction"]
    e2 = write_view(tmp_path / "1", rgb, depth, valid, z, z, z, {"view": 1, "view_kind": "tier1"}, sensor_valid=None)
    assert render_hash([e1], {}) != render_hash([e2], {}), "depth_valid is part of render.sha256"
