"""Table plane from touches - the geometry of table_touchoff.py. Plain pytest."""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

PKG = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

from admittance_control.table_probe import fit_plane, repeatability, touch_pattern  # noqa: E402


def test_default_pattern_is_a_hexagon_with_10cm_edges_plus_its_centre():
    pts = touch_pattern([0.4, -0.2])
    assert pts.shape == (7, 2)
    ring = pts[:6]
    edges = [np.linalg.norm(ring[i] - ring[(i + 1) % 6]) for i in range(6)]
    assert np.allclose(edges, 0.10)
    assert np.allclose(np.linalg.norm(ring - [0.4, -0.2], axis=1), 0.10)   # circumradius = edge
    assert np.allclose(pts[6], [0.4, -0.2])
    tri = touch_pattern([0.0, 0.0], edge_m=0.15, sides=3, with_centre=False)
    assert tri.shape == (3, 2) and np.allclose(np.linalg.norm(tri[0] - tri[1]), 0.15)
    with pytest.raises(ValueError):
        touch_pattern([0, 0], sides=2)


def test_plane_fit_recovers_height_tilt_and_noise():
    rng = np.random.default_rng(0)
    xy = touch_pattern([0.45, 0.1])
    tilt = np.deg2rad(0.5)
    z = -0.1123 + np.tan(tilt) * (xy[:, 0] - 0.45) + rng.normal(scale=2e-4, size=len(xy))
    fit = fit_plane(np.column_stack([xy, z]))
    assert fit["z_at_centroid_m"] == pytest.approx(-0.1123, abs=5e-4)
    assert fit["tilt_deg"] == pytest.approx(0.5, abs=0.3)
    assert fit["rmse_meaningful"] and 0.0 < fit["rmse_mm"] < 0.5
    exact = fit_plane(np.column_stack([xy[:3], z[:3]]))
    assert not exact["rmse_meaningful"] and exact["rmse_mm"] < 1e-9     # 3 points: no residual
    with pytest.raises(ValueError):
        fit_plane(np.zeros((2, 3)))


def test_repeatability_pools_per_spot_scatter():
    g1 = np.array([[0, 0, 0.0], [0, 0, 0.0002], [0, 0, -0.0002]])
    g2 = np.array([[1, 0, 0.0], [1, 0, 0.0]])
    rep = repeatability([g1, g2])
    assert rep["per_spot_z_std_mm"][0] == pytest.approx(0.1633, abs=1e-3)
    assert rep["pooled_z_std_mm"] == pytest.approx(np.sqrt((0.1633 ** 2 + 0) / 2), abs=1e-3)
    assert repeatability([g2[:1]])["pooled_z_std_mm"] is None


def test_tcp_error_is_recovered_from_contact_heights_on_a_known_plane():
    from admittance_control.table_probe import check_orientations, solve_tcp_error, slerp_R, rot_exp
    rng = np.random.default_rng(4)
    n = np.array([0.0175, -0.021, 1.0]); n /= np.linalg.norm(n)
    p0 = np.array([-0.516, 0.096, -0.0977])
    e_true = np.array([0.0021, -0.0013, 0.0008])            # a 2.5 mm lateral TCP error + 0.8 along
    P, Rs = [], []
    for tilt, az, axis, xh in check_orientations((0.0, 30.0)):
        z = axis / np.linalg.norm(axis)
        x = xh - (xh @ z) * z; x /= np.linalg.norm(x)
        R = np.column_stack([x, np.cross(z, x), z])
        true_tip = p0 + np.array([0.004, -0.003, 0.0])
        true_tip[2] = p0[2] - (n[0] * (true_tip[0] - p0[0]) + n[1] * (true_tip[1] - p0[1])) / n[2]
        reported = true_tip - R @ e_true + rng.normal(scale=1e-4, size=3) * np.array([1, 1, 1])
        P.append(reported); Rs.append(R)
    out = solve_tcp_error(np.array(P), np.array(Rs), n, p0)
    assert out["well_conditioned"]
    assert np.allclose(out["e_tcp_mm"][:2], [2.1, -1.3], atol=0.5)     # the lateral part: the question
    assert abs(out["lateral_mm"] - np.hypot(2.1, 1.3)) < 0.5
    # vertical touches alone cannot see the lateral error
    vert = solve_tcp_error(np.array(P[:4]), np.array(Rs[:4]), n, p0)
    assert not vert["well_conditioned"]
    R0 = rot_exp(np.array([0, 0, 0.3])); R1 = rot_exp(np.array([0.2, 0, 1.2]))
    assert np.allclose(slerp_R(R0, R1, 0.0), R0) and np.allclose(slerp_R(R0, R1, 1.0), R1)


def test_surface_vs_plane_measures_a_large_error_and_ignores_the_floor():
    from admittance_control.table_probe import surface_vs_plane, rot_exp
    rng = np.random.default_rng(5)
    n = np.array([0.0175, -0.021, 1.0]); n /= np.linalg.norm(n)
    p0 = np.array([-0.516, 0.096, -0.0977])
    # the camera's view of the table 0.4 m away from the touched patch, seen through an
    # extrinsic that is 20 mm too high and rotated 0.6 deg about x: offset and tilt
    xy = np.column_stack([rng.uniform(-0.10, 0.10, 6000) - 0.1, rng.uniform(-0.1, 0.1, 6000) + 0.45])
    z = p0[2] - (n[0] * (xy[:, 0] - p0[0]) + n[1] * (xy[:, 1] - p0[1])) / n[2]
    table = np.column_stack([xy, z])
    R = rot_exp(np.array([np.deg2rad(0.6), 0, 0]))
    c = table.mean(axis=0)
    seen = (table - c) @ R.T + c + np.array([0, 0, 0.020]) + rng.normal(scale=0.0008, size=table.shape)
    floor = np.column_stack([rng.uniform(-0.6, -0.2, 3000), rng.uniform(0.3, 0.6, 3000), np.full(3000, -0.75)])
    out = surface_vs_plane(np.vstack([seen, floor]), n, p0)
    assert "error" not in out, out
    assert out["height_offset_mm"] == pytest.approx(20.0, abs=0.5)
    assert out["tilt_deg"] == pytest.approx(0.6, abs=0.1)
    assert out["extrapolation_m"] > 0.3
    assert "error" in surface_vs_plane(floor, n, p0)                        # no table in view


def test_separate_tilt_splits_world_and_camera_parts():
    from admittance_control.table_probe import separate_tilt
    a = np.array([0.2, 0.1])                                   # table tilts 0.22 deg (world)
    b = np.array([0.6, -0.3])                                  # camera rotation error 0.67 deg
    yaws = np.array([0.0, 90.0, 180.0, 270.0, 45.0])
    tv = []
    for p in np.deg2rad(yaws):
        R = np.array([[np.cos(p), -np.sin(p)], [np.sin(p), np.cos(p)]])
        tv.append(a + R @ b)
    tv = np.array(tv)
    out = separate_tilt(np.linalg.norm(tv, axis=1), np.degrees(np.arctan2(tv[:, 1], tv[:, 0])), yaws)
    assert out["separable"]
    assert out["world_tilt_deg"] == pytest.approx(np.hypot(*a), abs=1e-6)
    assert out["camera_tilt_deg"] == pytest.approx(np.hypot(*b), abs=1e-6)
    same = separate_tilt([0.7, 0.7, 0.7], [110, 110, 110], [0, 5, 10])   # today's views: one yaw
    assert not same["separable"]


def test_refine_camera_rotation_recovers_a_known_tilt_error():
    from admittance_control.table_probe import refine_camera_rotation, rot_exp, tilt_to_normal
    n0 = np.array([0.0175, -0.021, 1.0]); n0 /= np.linalg.norm(n0)
    w_true = np.deg2rad([0.25, -0.30, 0.0])                   # the extrinsic's tilt error
    Rc = rot_exp(w_true)
    a = tilt_to_normal(n0, 0.35, 70.0)                        # the table there tilts too
    Rs, Ns = [], []
    for yaw in (-135.0, -51.0, 39.0, 127.0):                  # today's four wrist yaws
        y = np.deg2rad(yaw)
        # camera looking down (optical z = -base z), x along the yaw
        R = np.column_stack([[np.cos(y), np.sin(y), 0.0], [np.sin(y), -np.cos(y), 0.0], [0.0, 0.0, -1.0]])
        R = rot_exp(np.deg2rad([0.4, -0.2, 0.0])) @ R          # a slightly oblique view
        Rs.append(R); Ns.append(R @ Rc.T @ R.T @ a)           # what the used extrinsic shows
    out = refine_camera_rotation(np.array(Rs), np.array(Ns), n0)
    assert np.allclose(out["w_camera_deg"][:2], np.degrees(w_true[:2]), atol=0.02)
    assert out["world_tilt_deg"] == pytest.approx(0.35, abs=0.02)
    assert out["residual_deg"] < 0.01 and out["conditioning"] > 0.3
    # tilt_to_normal inverts the reported (tilt, azimuth)
    n = tilt_to_normal(n0, 0.7, 110.0)
    assert np.degrees(np.arccos(n @ n0)) == pytest.approx(0.7, abs=1e-9)
