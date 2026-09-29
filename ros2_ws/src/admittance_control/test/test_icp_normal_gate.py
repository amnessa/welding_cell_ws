"""ICP normal gate on a thin plate seen from one side. Plain pytest, no ROS.

The bench case (2026-09-29): an 8 mm plate standing 99 mm tall, the camera sees one
face and the top edge. Without the gate the model's hidden back face pairs with the
visible face too, and ICP rests in a pose that straddles the two faces - the visible
face on the data at the top, the back face at the bottom (a lean of atan(8/99)) - so
the weld root at the bottom edge is off by a plate thickness. Claims: the sampled
normals point outward (also for an inside-out mesh); from that straddled start the
gated ICP returns to the true pose, root within 1.5 mm and < 1 deg.
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np

PKG = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

from admittance_control.icp import icp_point_to_plane, sample_mesh_surface  # noqa: E402

LX, TH, H = 0.250, 0.008, 0.099          # plate: x in [0, LX], y in [-TH, 0], z in [0, H]


def _box(lo, hi):
    """Closed box mesh, outward winding (positive signed volume)."""
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    v = np.array([[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
                  [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]], float)
    f = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (2, 3, 7, 6), (1, 2, 6, 5), (0, 4, 7, 3)]
    return v, f


def _rotx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _scene(rng):
    """What the camera (on the -y side, above) sees: the y=-TH face and the top edge."""
    n = 3000
    front = np.c_[rng.uniform(0, LX, n), np.full(n, -TH), rng.uniform(0, H, n)]
    m = 300
    top = np.c_[rng.uniform(0, LX, m), rng.uniform(-TH, 0, m), np.full(m, H)]
    pts = np.vstack([front, top]) + rng.normal(0, 0.0005, (n + m, 3))
    nrm = np.vstack([np.tile([0, -1.0, 0], (n, 1)), np.tile([0, 0, 1.0], (m, 1))])
    return pts, nrm


def _straddled_pose():
    """Visible face on the data at the top edge, the hidden face on it at the bottom."""
    pivot = np.array([0, -TH, H])
    for a in (np.arctan2(TH, H), -np.arctan2(TH, H)):
        T = np.eye(4)
        T[:3, :3] = _rotx(a)
        T[:3, 3] = pivot - T[:3, :3] @ pivot
        back_bottom = T[:3, :3] @ np.array([LX / 2, 0.0, 0.0]) + T[:3, 3]
        if abs(back_bottom[1] + TH) < 1e-3:
            return T
    raise AssertionError('no straddle found')


def _root_err_mm(T):
    """Where the pose puts the visible-side root (bottom front edge) vs the truth."""
    p = np.array([LX / 2, -TH, 0.0])
    return np.linalg.norm(T[:3, :3] @ p + T[:3, 3] - p) * 1000


def _rot_err_deg(T):
    return np.degrees(np.arccos(np.clip((np.trace(T[:3, :3]) - 1) / 2, -1, 1)))


def test_sampled_normals_point_outward_even_when_inside_out():
    rng = np.random.default_rng(0)
    v, f = _box([0, -TH, 0], [LX, 0, H])
    c = v.mean(0)
    for faces in (f, [tuple(reversed(x)) for x in f]):
        p, n = sample_mesh_surface(v, faces, 2000, rng, return_normals=True)
        assert np.all(np.einsum('ij,ij->i', p - c, n) > 0)
    assert sample_mesh_surface(v, f, 10, rng).shape == (10, 3)     # old signature unchanged


def test_gate_recovers_the_straddled_plate():
    rng = np.random.default_rng(1)
    v, f = _box([0, -TH, 0], [LX, 0, H])
    model, model_n = sample_mesh_surface(v, f, 2500, rng, return_normals=True)
    scene, scene_n = _scene(rng)
    T0 = _straddled_pose()
    assert _root_err_mm(T0) > 7.0                                   # the bench failure

    T, info = icp_point_to_plane(model, scene, scene_n, init=T0, max_corr_dist=0.02,
                                 max_iter=60, anderson_depth=5, robust=True,
                                 source_normals=model_n)
    assert _root_err_mm(T) < 1.5, _root_err_mm(T)
    assert _rot_err_deg(T) < 1.0, _rot_err_deg(T)
    assert info['fitness'] > 0.2

    # the plain point-to-plane path takes the gate too
    T, _ = icp_point_to_plane(model, scene, scene_n, init=T0, max_corr_dist=0.02,
                              max_iter=60, source_normals=model_n)
    assert _root_err_mm(T) < 1.5, _root_err_mm(T)


def test_gate_off_is_the_old_behaviour():
    rng = np.random.default_rng(2)
    v, f = _box([0, -TH, 0], [LX, 0, H])
    model, model_n = sample_mesh_surface(v, f, 2500, rng, return_normals=True)
    scene, scene_n = _scene(rng)
    T0 = _straddled_pose()
    kw = dict(init=T0, max_corr_dist=0.02, max_iter=60, anderson_depth=5, robust=True)
    Ta, _ = icp_point_to_plane(model, scene, scene_n, **kw)
    Tb, _ = icp_point_to_plane(model, scene, scene_n, source_normals=model_n,
                               normal_gate_deg=None, **kw)
    assert np.allclose(Ta, Tb)
