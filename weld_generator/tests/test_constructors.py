"""Phase 6b step 2 — the Tube primitive and the revolved-configuration constructors.

Claims under test: meshes are watertight in every cut mode (D21), the saddle/miter
cut is EXACT (the base ring lies on the cutting surface to machine precision — that
ring IS the seam, offset by the gap), cloud samples sit exactly on the true surfaces
(D34's chord error lives only in the mesh), the analytic ray test agrees with exact
containment, and each constructor's posed parts contain the step-1 curve on their
surfaces — tying step 2 back to step 1 without a single tolerance clause.
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from weldgen.constructors import build_d29  # noqa: E402
from weldgen.d29 import sample_d29_seam  # noqa: E402
from weldgen.geom import Slab, Tube  # noqa: E402
from weldgen.visibility import ray_hits_tube  # noqa: E402


def _tube(cut=None, gap=0.0, r=40.0, wall=6.0, L=150.0, T=None):
    return Tube("B", "workpiece", 1, r, wall, L,
                np.eye(4) if T is None else T, base_cut=cut, gap_mm=gap)


# ------------------------------------------------------------------ the primitive


def test_tube_mesh_is_watertight_in_every_cut_mode():
    th = np.radians(25.0)
    cuts = [
        (None, 0.0),
        ({"kind": "plane", "n_local": [np.sin(th), 0.0, np.cos(th)],
          "d": 30.0 * np.cos(th)}, 0.5),
        ({"kind": "cylinder", "point_local": [0, 0, -60.0],
          "axis_local": [1.0, 0, 0], "radius_mm": 60.0}, 0.3),
    ]
    for cut, gap in cuts:
        t = _tube(cut, gap, r=20.0 if cut and cut["kind"] == "cylinder" else 40.0,
                  wall=4.0, L=120.0)
        m = t.mesh()
        assert m.is_watertight and m.is_winding_consistent, cut
        assert m.volume > 0.0
        assert t.max_chord_error_mm <= 0.25          # the D34 gate, at part level


def test_uncut_tube_volume_matches_the_analytic_shell():
    t = _tube()
    v_true = np.pi * (40.0 ** 2 - 34.0 ** 2) * 150.0
    assert abs(t.mesh().volume - v_true) / v_true < 0.005    # chords only


def test_saddle_cut_ring_lies_exactly_on_the_cutting_cylinder():
    t = _tube({"kind": "cylinder", "point_local": [0, 0, -60.0],
               "axis_local": [1.0, 0, 0], "radius_mm": 60.0},
              gap=0.0, r=20.0, wall=4.0, L=120.0)
    for radius in (20.0, 16.0, 18.3):                # through the whole wall
        phis = np.linspace(0, 2 * np.pi, 181, endpoint=False)
        h = t.base_height(phis, radius)
        ring = np.column_stack([radius * np.cos(phis), radius * np.sin(phis), h])
        d = ring - np.array([0.0, 0.0, -60.0])
        res = np.einsum("ij,ij->i", d, d) - (d @ np.array([1.0, 0, 0])) ** 2 - 60.0 ** 2
        assert np.abs(res).max() < 1e-9, radius
        # the stub sits OUTSIDE the main: just above the ring is outside its solid
        up = ring + np.array([0.0, 0.0, 1.5])
        du = up - np.array([0.0, 0.0, -60.0])
        rho2 = np.einsum("ij,ij->i", du, du) - (du @ np.array([1.0, 0, 0])) ** 2
        assert (rho2 > 60.0 ** 2).all()


def test_face_samples_lie_exactly_on_the_true_surfaces():
    t = _tube({"kind": "plane", "n_local": [0.3, 0.1, 0.95], "d": 20.0}, gap=0.4)
    rng = np.random.default_rng(0)
    for name in t.face_names():
        pts, nrm = t.sample_face(name, 2000, rng)
        assert np.abs(np.linalg.norm(nrm, axis=1) - 1.0).max() < 1e-9
        rho = np.hypot(pts[:, 0], pts[:, 1])
        if name == "lateral+":
            assert np.abs(rho - t.r_outer_mm).max() < 1e-9
        elif name == "lateral-":
            assert np.abs(rho - t.r_inner_mm).max() < 1e-9
        elif name == "+w":
            assert np.abs(pts[:, 2] - t.length_mm).max() < 1e-9
            assert (rho >= t.r_inner_mm - 1e-9).all()
            assert (rho <= t.r_outer_mm + 1e-9).all()
        else:                                        # the miter face
            phi = np.arctan2(pts[:, 1], pts[:, 0])
            want = t.base_height(phi, rho)
            assert np.abs(pts[:, 2] - want).max() < 1e-9


def test_ray_hits_tube_agrees_with_exact_containment_marching():
    t = _tube({"kind": "plane", "n_local": [0.35, 0.0, 0.94], "d": 15.0}, gap=0.5,
              r=35.0, wall=6.0, L=120.0)
    rng = np.random.default_rng(1)
    o = rng.uniform(-150, 150, (600, 3)) + np.array([0, 0, 40.0])
    d = rng.normal(size=(600, 3))
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    tm = rng.uniform(50, 400, 600)
    got = ray_hits_tube(o, d, tm, t)
    # reference: dense containment marching along each ray
    steps = np.linspace(1e-3, 1.0, 400)
    ref = np.zeros(600, dtype=bool)
    for i in range(600):
        pts = o[i][None, :] + (steps * tm[i])[:, None] * d[i][None, :]
        ref[i] = bool(t.contains(pts).any())
    agree = (got == ref).mean()
    assert agree > 0.995, f"agreement {agree}"        # sub-sliver misses only


def test_tube_registry_and_metadata():
    t = _tube()
    assert t.face_names() == ("lateral+", "lateral-", "+w", "-w")
    assert t.thickness_mm == 6.0                      # the WALL - what ISO keys on
    assert t.face_plane("+w") is not None and t.face_plane("lateral+") is None
    sd = t.surface_desc("lateral+")
    assert sd["kind"] == "cylinder" and sd["radius_mm"] == 40.0 and sd["outward"]
    assert t.part_geometry_id.startswith("tube_")


# ---------------------------------------------------------------- the constructors


def _dist_to_cylinder(pts, point, axis, r):
    d = pts - np.asarray(point, float)[None, :]
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    am = d @ a
    rho = np.sqrt(np.clip(np.einsum("ij,ij->i", d, d) - am ** 2, 0.0, None))
    return np.abs(rho - r)


@pytest.mark.parametrize("config", [2, 3, 4])
def test_constructed_parts_carry_the_step1_curve_on_their_surfaces(config):
    rng = np.random.default_rng(config)
    for _ in range(3):
        draw = sample_d29_seam(rng, config=config)
        built = build_d29(draw, rng)
        parts = built["parts"]
        for p in parts:
            m = p.mesh()
            assert m.is_watertight and m.is_winding_consistent, (config, p.id)
        pts = built["curve"].point(
            np.linspace(0, built["curve"].t_period, 181, endpoint=False))
        tube = next(p for p in parts if isinstance(p, Tube) and p.id == "B")
        Tw = tube.T_world_part
        # seam on the stub's OUTER cylinder, exactly
        loc = (pts - Tw[:3, 3]) @ Tw[:3, :3]
        assert np.abs(np.hypot(loc[:, 0], loc[:, 1])
                      - tube.r_outer_mm).max() < 1e-6, config
        if config in (2, 3):
            plate = next(p for p in parts if isinstance(p, Slab))
            assert np.abs(pts[:, 2]).max() < 1e-9    # on the plate top plane
            assert plate.T_world_part[2, 3] < 0.0    # plate body below it
        else:
            main = next(p for p in parts if p.id == "A")
            assert _dist_to_cylinder(pts, main.T_world_part[:3, 3],
                                     main.T_world_part[:3, 2],
                                     main.r_outer_mm).max() < 1e-6


@pytest.mark.parametrize("config", [2, 3, 4])
def test_gap_separates_the_parts_without_penetration(config):
    rng = np.random.default_rng(10 + config)
    draw = sample_d29_seam(rng, config=config)
    built = build_d29(draw, rng)
    a, b = built["parts"]
    sr = np.random.default_rng(0)
    pts_b, _ = b.sample_face("lateral+", 3000, sr)
    assert not a.contains(pts_b, tol=-1e-6).any(), "stub penetrates the base part"
    if config in (2, 3):
        pts_low, _ = b.sample_face("-w", 2000, sr)
        assert (pts_low[:, 2] > -1e-6).all(), "miter dips below the plate plane"


def test_config4_miter_ring_reproduces_the_seam_at_the_gap():
    rng = np.random.default_rng(5)
    draw = sample_d29_seam(rng, config=4)
    built = build_d29(draw, rng)
    branch = next(p for p in built["parts"] if p.id == "B")
    g = built["gap_mm"]
    phis = np.linspace(0, 2 * np.pi, 240, endpoint=False)
    h = branch.base_height(phis, branch.r_outer_mm)
    ring_local = np.column_stack([branch.r_outer_mm * np.cos(phis),
                                  branch.r_outer_mm * np.sin(phis), h])
    Tw = branch.T_world_part
    ring = ring_local @ Tw[:3, :3].T + Tw[:3, 3]
    # retract the gap along the stub axis: the ring must land on the main cylinder
    ring_on = ring - g * Tw[:3, 2]
    geo = built["realization"]
    assert _dist_to_cylinder(ring_on, geo["main_point_mm"], geo["main_axis"],
                             geo["main_radius_mm"]).max() < 1e-6


# ------------------------------------------------------------- the swept family


@pytest.mark.parametrize("config", [5, 6, 7])
def test_swept_family_builds_watertight_parts_under_the_chord_gate(config):
    from weldgen.geom import SweptSlab
    rng = np.random.default_rng(20 + config)
    for _ in range(3):
        draw = sample_d29_seam(rng, config=config)
        built = build_d29(draw, rng)
        for p in built["parts"]:
            m = p.mesh()
            assert m.is_watertight and m.is_winding_consistent, (config, p.id)
            if isinstance(p, SweptSlab):
                assert p.max_chord_error_mm <= 0.25, (config, p.id)   # D34


def test_config5_outer_wall_is_the_step1_curve():
    rng = np.random.default_rng(31)
    draw = sample_d29_seam(rng, config=5)
    built = build_d29(draw, rng)
    tube = built["parts"][1]
    # the tube's outer surface (offset 0) IS the sampled rounded rectangle
    assert tube.offset_lo_mm == 0.0
    pts, _ = tube.sample_face("-w", 2000, np.random.default_rng(0))
    dense = built["curve"].point(
        np.linspace(0, built["curve"].t_period, 6000, endpoint=False))
    d = np.sqrt(((pts[:, None, :2] - dense[None, :, :2]) ** 2).sum(-1)).min(1)
    assert float(d.max()) < 0.1          # dense-reference resolution, not model error
    assert (pts[:, 2] >= built["gap_mm"] - 1e-9).all()


def test_config6_stiffener_stands_centred_on_the_drawn_curve():
    rng = np.random.default_rng(32)
    draw = sample_d29_seam(rng, config=6)
    built = build_d29(draw, rng)
    plate, st = built["parts"]
    assert st.offset_lo_mm == -st.offset_hi_mm            # centred band
    assert st.z0_mm == built["gap_mm"] and st.z0_mm > 0.0  # standing at the gap
    # both broad faces exactly t/2 from the spine
    for name, o in (("+w", st.offset_hi_mm), ("-w", st.offset_lo_mm)):
        pts, _ = st.sample_face(name, 1500, np.random.default_rng(1))
        dense = built["curve"].point(np.linspace(0, built["curve"].t_period, 4000))
        d = np.sqrt(((pts[:, None, :2] - dense[None, :, :2]) ** 2).sum(-1)).min(1)
        assert abs(float(np.median(d)) - abs(o)) < 0.15, name
    # stiffener never dips into the plate
    pts, _ = st.sample_face("-v", 1500, np.random.default_rng(2))
    assert (pts[:, 2] > 0.0).all()


def test_config7_curve_is_the_gap_centreline_with_flush_tops():
    rng = np.random.default_rng(33)
    draw = sample_d29_seam(rng, config=7)
    built = build_d29(draw, rng)
    A, B = built["parts"]
    g = built["gap_mm"]
    assert A.offset_lo_mm == g / 2.0 and B.offset_hi_mm == -g / 2.0
    assert A.z1_mm == 0.0 and B.z1_mm == 0.0              # flush exposed faces
    # the two prepared edge faces sit g apart, symmetric about the drawn arc
    pa, _ = A.sample_face("-w", 1200, np.random.default_rng(3))
    pb, _ = B.sample_face("+w", 1200, np.random.default_rng(4))
    dense = built["curve"].point(np.linspace(0, built["curve"].t_period, 4000))
    for pts in (pa, pb):
        d = np.sqrt(((pts[:, None, :2] - dense[None, :, :2]) ** 2).sum(-1)).min(1)
        inner = d[(pts[:, 0] > dense[:, 0].min()) & (pts[:, 0] < dense[:, 0].max())]
        assert abs(float(np.median(inner)) - g / 2.0) < 0.05
    # and the parts do not touch: every A point clears B's band entirely
    assert not B.contains(pa).any() and not A.contains(pb).any()
