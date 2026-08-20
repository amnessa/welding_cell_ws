"""`lit-ransac` — Yi et al. 2026, checked against known geometry.

These are not accuracy claims either. They pin the four things a reimplementation of a
paper can silently get wrong: the published constants and their units, the two equations
that are unusable as printed (18 and 19), the guard that is frame-dependent (§6.2), and
the coverage limit the whole comparison turns on — that a mechanism built on plane
*intersections* has nothing to return when the two faces are parallel.
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from baselines import evaluate, ground_truth  # noqa: E402
from baselines.lit_ransac import (ETA_0, T_D2_MM, T_MPP,  # noqa: E402
                                  Plane, adaptive_iterations, detect,
                                  intersection_line, multi_plane_fit, optimize_plane,
                                  plane_from_three_points, seam_endpoints,
                                  seam_region_oracle, torch_pose, triple_intersection)


def grid(size: float = 100.0, n: int = 60) -> np.ndarray:
    a = np.linspace(0.0, size, n)
    return np.stack(np.meshgrid(a, a), -1).reshape(-1, 2)


def fold(size: float = 100.0, n: int = 60):
    """Two zero-thickness planes meeting at 90 deg along the x axis at the origin."""
    a = grid(size, n)
    p1 = np.column_stack([a[:, 0], a[:, 1], np.zeros(len(a))])
    p2 = np.column_stack([a[:, 0], np.zeros(len(a)), a[:, 1]])
    return np.vstack([p1, p2])


def slabs(joint_type: str = "T", density: float = 1.0, **kw):
    """A real sampled assembly from the generator — plates with thickness and gap."""
    sys.path.insert(0, str(ROOT))
    from weldgen.joints import JointSpec
    from weldgen.layouts import build
    from weldgen.sampling import sample_scene_surface

    p = dict(L_A=200.0, W_A=100.0, t_A=8.0, L_B=200.0, H_B=80.0, t_B=8.0,
             root_gap_mm=1.0, linear_misalignment_mm=0.0, angular_misalignment_deg=0.0,
             included_angle_deg=JointSpec.NOMINAL_INCLUDED_DEG[joint_type],
             stack_offset_mm={"lap": 40.0, "edge": 0.0}.get(joint_type))
    p.update(kw)
    parts = build(JointSpec(**p), joint_type, np.eye(4))
    cloud = sample_scene_surface(parts, density, np.random.default_rng(0))
    return cloud["xyz"].astype(float), parts


# --- the published constants ------------------------------------------------------------

def test_the_published_constants_are_the_defaults_and_are_millimetres():
    """§5.2 T_d2 = 2 mm, §5.3 T_mpp = 0,025, eta_0 = 0,7.

    The unit is the trap. `T_d2` is 2 **millimetres**, chosen in §5.2 as roughly three
    times a 0,64 mm point spacing on 6 mm plate; the same number read as metres would make
    every point in every scene an inlier of the first hypothesis and return one plane.
    """
    assert T_D2_MM == 2.0 and T_MPP == 0.025 and ETA_0 == 0.7
    pts = fold()
    assert len(multi_plane_fit(pts, dist_thresh_mm=T_D2_MM, seed=0)) == 2
    assert len(multi_plane_fit(pts, dist_thresh_mm=T_D2_MM * 1000, seed=0)) == 1


def test_equation_18_as_printed_is_unusable_and_the_adaptive_reading_is_not():
    """Deviation 1. The literal `3/N_fp` gives a budget no runtime could match.

    The paper reports 0,187 s for a seven-plane fit (Table 6). At the printed formula a
    single 10 000-point subset needs ~1e10 samples, so the printed formula is not what was
    run; the standard adaptive form with `w = n_in/N_fp` is, and it lands near 40.
    """
    literal = adaptive_iterations(3000, 10_000, ETA_0, rule="literal", cap=10**12)
    adaptive = adaptive_iterations(3000, 10_000, ETA_0, rule="adaptive")
    assert literal > 1e9
    assert 20 < adaptive < 100


def test_the_inlier_occupancy_ratio_is_against_the_original_cloud_not_the_remainder():
    """Eq. 17 divides by `N_seg`. Against the remainder it would never terminate.

    A face too small to be 2,5% of the workpiece is dropped — and on a lap joint the
    plate *edge* face is exactly that, which is why §6 cannot reach a lap toe. Measured,
    not argued: the same cloud with the ratio lowered recovers the missing planes.
    """
    a = grid(100.0, 60)
    big = np.column_stack([a[:, 0], a[:, 1], np.zeros(len(a))])
    b = grid(100.0, 8)                                  # ~1,7% of the points
    small = np.column_stack([b[:, 0], np.zeros(len(b)), b[:, 1] * 0.1 + 5.0])
    pts = np.vstack([big, small])

    assert len(multi_plane_fit(pts, min_inlier_ratio=T_MPP, seed=0)) == 1
    assert len(multi_plane_fit(pts, min_inlier_ratio=0.005, seed=0)) >= 2


# --- §5, the fit ------------------------------------------------------------------------

def test_a_plane_from_three_points_is_unit_normal_and_signed_offset():
    """Eqs. 13-14, and `None` rather than a NaN plane when the three points are collinear."""
    n, d = plane_from_three_points([0, 0, 3.0], [1, 0, 3.0], [0, 1, 3.0])
    assert np.isclose(np.linalg.norm(n), 1.0)
    assert np.allclose(np.abs(n), [0, 0, 1]) and np.isclose(abs(d), 3.0)
    assert plane_from_three_points([0, 0, 0], [1, 1, 1], [2, 2, 2]) is None


def test_the_centroid_refit_removes_the_tilt_that_over_segmentation_puts_in():
    """§5.4. The contaminated points sit at the intersection, i.e. far from the centroid.

    That is the whole mechanism, and it is why restricting the refit to a neighbourhood of
    `p_o` works at all. Built here as a plane whose inlier set has been deliberately
    poisoned along one edge, so the recovered tilt is a number rather than a picture.
    """
    a = grid(100.0, 60)
    flat = np.column_stack([a[:, 0], a[:, 1], np.zeros(len(a))])
    edge = a[a[:, 1] < 3.0]                             # the strip nearest y = 0
    bad = np.column_stack([edge[:, 0], edge[:, 1], np.full(len(edge), 1.8)])
    pts = np.vstack([flat, bad])
    idx = np.arange(len(pts))

    c = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - c, full_matrices=False)
    tilted = Plane(normal=vt[-1], d=float(-vt[-1] @ c), inliers=idx, centroid=c,
                   n_inliers=len(idx), inlier_ratio=1.0)
    fixed = optimize_plane(pts, tilted, mode="centroid_lstsq", k_neighbors=400)

    def tilt(pl):
        return np.degrees(np.arccos(abs(float(np.clip(pl.normal @ [0, 0, 1], -1, 1)))))

    assert tilt(fixed) < 0.1 < tilt(tilted)


# --- §6, the geometry -------------------------------------------------------------------

def test_the_intersection_line_of_a_ninety_degree_fold_is_exact():
    """Eqs. 20-22 on the case the method is built for. No tolerance to hide behind."""
    planes = multi_plane_fit(fold(), seed=0)
    p, d = intersection_line(planes[0], planes[1])
    assert np.allclose(np.abs(d), [1, 0, 0], atol=1e-6)
    assert np.allclose(p[1:], [0, 0], atol=1e-6)


def test_parallel_faces_have_no_intersection_line_and_that_is_the_coverage_result():
    """`n1 x n2 = 0`. This return, not a tuning failure, is why butt and edge are out.

    `dataset_plan.md` §4 predicts `lit-ransac` structurally cannot express the coplanar
    exposed arm of D4. Here is the line of code the prediction is about.
    """
    n = np.array([0.0, 0.0, 1.0])
    a = Plane(n, 0.0, np.arange(0), np.zeros(3), 0, 0.0)
    b = Plane(-n, 8.0, np.arange(0), np.zeros(3), 0, 0.0)
    assert intersection_line(a, b) is None


def test_an_edge_joint_returns_nothing_and_says_why():
    """The prediction, end to end on a generated edge joint rather than on two normals.

    `pairs` carries the verdict on every plane pair §6 looked at, so the claim "the
    mechanism cannot express this joint" is a value in a table rather than a sentence in a
    paper. Every pair here is either parallel or off-orthogonal — none is rejected for
    want of support or length, which is what a tuning failure would look like.
    """
    pts, _ = slabs("edge")
    r = detect(pts, seed=0, prefilter_density_per_mm2=1.0)
    assert r.n_seams == 0
    assert "parallel" in r.note
    assert r.pairs and all(p["status"] in ("parallel", "not_orthogonal") for p in r.pairs)
    assert not any(p["status"] == "seam" for p in r.pairs)


def test_the_endpoints_come_from_the_plate_that_terminates_not_the_base():
    """§6.2's "vertical point cloud", operationally: the shorter extent along the seam.

    A base plate overruns the plate welded to it. Projecting the base's inliers would
    return its own length as the weld length, which on this fold is 2x too long.
    """
    a = grid(200.0, 80)
    base = np.column_stack([a[:, 0] - 100.0, a[:, 1] - 100.0, np.zeros(len(a))])
    b = grid(100.0, 60)
    web = np.column_stack([b[:, 0] - 50.0, np.zeros(len(b)), b[:, 1]])
    pts = np.vstack([base, web])
    planes = multi_plane_fit(pts, seed=0)
    i, j = (0, 1) if abs(planes[0].normal[2]) > abs(planes[1].normal[2]) else (1, 0)
    p_line, d = intersection_line(planes[i], planes[j])

    p0, p1 = seam_endpoints(pts, planes[i], planes[j], p_line, d, None, "smaller_plane")
    assert np.isclose(np.linalg.norm(p1 - p0), 100.0, atol=2.0)
    q0, q1 = seam_endpoints(pts, planes[i], planes[j], p_line, d, None, "both")
    assert np.isclose(np.linalg.norm(q1 - q0), 200.0, atol=2.0)


def test_equation_24_is_a_reflection_as_printed():
    """Deviation 6. `X_t = X_s, Z_t = -Z_s, Y_t = X_t x Z_t` has det = -1.

    Harmless for the seam geometry, fatal if the matrix is handed to a robot controller as
    a pose, so it is fixed by default and the printed form is kept reachable.
    """
    pts = fold()
    planes = multi_plane_fit(pts, seed=0)
    p_line, d = intersection_line(planes[0], planes[1])
    _, R_lit = torch_pose(d, p_line, planes[0], planes[1], right_handed=False)
    _, R_fix = torch_pose(d, p_line, planes[0], planes[1], right_handed=True)
    assert np.isclose(np.linalg.det(R_lit), -1.0)
    assert np.isclose(np.linalg.det(R_fix), 1.0)


def test_the_approach_vector_bisects_outward_from_the_corner():
    """Eq. 23. `v_0 = v_1 + v_2` with both pointing out along their own plate."""
    pts = fold()
    planes = multi_plane_fit(pts, seed=0)
    p_line, d = intersection_line(planes[0], planes[1])
    v0, _ = torch_pose(d, p_line, planes[0], planes[1])
    assert np.allclose(np.abs(v0), [0.0, 1 / np.sqrt(2), 1 / np.sqrt(2)], atol=1e-3)


def test_the_printed_coordinate_bound_rejects_a_corner_on_this_datasets_origin():
    """Deviation 4. §6.2's `|coord| in [0.001, 1000]` is frame-dependent.

    The generator places joints at the world origin, so the literal test throws away
    exactly the corners it is meant to keep. The default replaces it with a bound relative
    to the cloud's own extent, which does the job §6.2 says it wants done.
    """
    ex, ey, ez = np.eye(3)
    a = Plane(ex, 0.0, np.arange(0), np.zeros(3), 0, 0.0)
    b = Plane(ey, 0.0, np.arange(0), np.zeros(3), 0, 0.0)
    c = Plane(ez, 0.0, np.arange(0), np.zeros(3), 0, 0.0)
    bbox = (np.full(3, -100.0), np.full(3, 100.0))

    assert np.allclose(triple_intersection(a, b, c, bbox), [0, 0, 0])
    assert triple_intersection(a, b, c, bbox, coord_bounds_mm=(1e-3, 1e3)) is None


# --- what the method depends on ---------------------------------------------------------

def test_the_segmentation_oracle_is_what_removes_the_plate_border_phantoms():
    """The L0 -> L1 delta, on one T-joint. **This is the method's real dependency.**

    Without §3's PointNet++ the fitter sees a whole workpiece, and a plate's own top face
    and edge face are an orthogonal intersecting pair exactly like a fillet is. Every plate
    border becomes a weld: 12 seams against 2, precision 0,26. Restricting the input to
    §3's ~40 mm band takes it to 8 and 0,77 — better, and still not clean, because the
    plate's *own end faces* fall inside a 40 mm band around the seam it terminates. That
    residue is a property of the mechanism, not of the segmentation quality.

    Thick plate on purpose. At 8 mm the band is wider than the workpiece is deep, so it
    admits every border anyway and the two arms coincide — which is itself worth knowing:
    the oracle only buys anything when the geometry is larger than the annotation width.
    """
    pts, _ = slabs("T", t_A=20.0, t_B=20.0, W_A=160.0, H_B=120.0, root_gap_mm=0.0)
    truth = [np.array([[-100.0, 0.0, 0.0], [100.0, 0.0, 0.0]]),
             np.array([[-100.0, 20.0, 0.0], [100.0, 20.0, 0.0]])]
    free = detect(pts, seed=0, prefilter_density_per_mm2=1.0)
    gated = detect(pts, seed=0, prefilter_density_per_mm2=1.0,
                   segmentation_mask=seam_region_oracle(pts, truth))
    assert gated.used_segmentation_oracle and not free.used_segmentation_oracle
    assert gated.n_seams < free.n_seams
    m_free = evaluate(free.polylines, truth, tol_mm=3.0)
    m_gated = evaluate(gated.polylines, truth, tol_mm=3.0)
    assert m_free["recall"] == m_gated["recall"] == 1.0
    assert m_gated["precision"] > m_free["precision"] + 0.2


def test_the_same_seed_gives_the_same_answer_because_the_method_is_randomised():
    """Reproducibility is asserted; spread across seeds is *measured*, not asserted.

    `lit-ransac` is one of the two randomised methods, and the plan requires 100 repeats
    and a box plot for it rather than a single number. All this test guarantees is that a
    repeat harness controls the randomness through `seed`.
    """
    pts, _ = slabs("T")
    a = detect(pts, seed=7, prefilter_density_per_mm2=1.0)
    b = detect(pts, seed=7, prefilter_density_per_mm2=1.0)
    assert [s.polyline.tolist() for s in a.seams] == [s.polyline.tolist() for s in b.seams]


def test_runs_end_to_end_on_a_generated_scene():
    """Against constructed truth, through the same `evaluate` every other method uses."""
    from baselines import cloud_for, load_scene, scene_dirs

    dirs = [d for d in scene_dirs(ROOT / "out" / "phase3")]
    if not dirs:
        pytest.skip("no generated corpus at out/phase3")
    for d in dirs:
        scene, arrays = load_scene(d)
        if scene["joint"]["type"] not in ("T", "corner"):
            continue
        gt = ground_truth(scene, arrays, primary_only=True)
        if not gt:
            continue
        pts = cloud_for(scene, arrays, view="full")["xyz"]
        r = detect(pts, seed=0, prefilter_density_per_mm2=1.0,
                   segmentation_mask=seam_region_oracle(pts, gt))
        m = evaluate(r.polylines, gt, tol_mm=3.0)
        assert set(m) >= {"chamfer", "precision", "recall", "f1"}
        assert m["recall"] > 0.5, (d.name, m)
        return
    pytest.skip("no T or corner scene with primary seams")
