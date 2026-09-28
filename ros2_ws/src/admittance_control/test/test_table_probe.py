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
