"""`lit-regiongrow` — Wei et al. 2024, checked against known geometry.

The interesting property of this method, and the reason it is worth a test file of its own:
**its curvature is the same quantity `ours` uses.** Eq. 3's `δ = λ₀/(λ₀+λ₁+λ₂)` is
`radius_pca.surface_variation`. So the two methods differ only in what they do with it, and
each of those differences is pinned here rather than described:

  * a k-NN neighbourhood instead of a radius ball (`README §8` says this is the whole game)
  * region growing off the smoothest point instead of a global threshold
  * a two-surface test built from the method's OWN regions instead of an `object_id` oracle
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from baselines import ground_truth  # noqa: E402
from baselines.lit_regiongrow import (CURVATURE_THRESH, K_NEIGHBORS,  # noqa: E402
                                      KEYPOINT_CROP_FRACTION,
                                      KEYPOINT_RADIUS_BOUNDS_MM, SMOOTHNESS_DEG,
                                      VOXEL_MM, _edge_from_labels, detect, fit_seam,
                                      keypoint_crop, local_pca, preprocess, region_grow,
                                      two_surface_edges)
from baselines.metrics import distance_to_polylines  # noqa: E402
from baselines.lit_ransac import seam_region_oracle  # noqa: E402
from baselines.radius_pca import surface_variation  # noqa: E402


def grid(size: float = 100.0, n: int = 80) -> np.ndarray:
    a = np.linspace(0.0, size, n)
    return np.stack(np.meshgrid(a, a), -1).reshape(-1, 2)


def fold(size: float = 100.0, n: int = 80):
    """Two zero-thickness planes meeting at 90 deg along the x axis at the origin."""
    a = grid(size, n)
    p1 = np.column_stack([a[:, 0], a[:, 1], np.zeros(len(a))])
    p2 = np.column_stack([a[:, 0], np.zeros(len(a)), a[:, 1]])
    return np.vstack([p1, p2])


def slabs(joint_type: str = "T", density: float = 1.0, **kw):
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
    return cloud["xyz"].astype(float), cloud["normals"].astype(float)


def _t_scene(min_thickness: float = 6.0):
    from baselines import cloud_for, load_scene, scene_dirs, scene_facts

    for d in scene_dirs(ROOT / "out" / "phase3"):
        scene, arrays = load_scene(d)
        f = scene_facts(scene)
        if f["joint_type"] != "T" or f["t_min_mm"] < min_thickness:
            continue
        gt = ground_truth(scene, arrays, primary_only=True)
        if not gt:
            continue
        return cloud_for(scene, arrays, view="full")["xyz"], gt
    return None


# --- the shared feature -----------------------------------------------------------------

def test_equation_3_is_the_same_quantity_ours_calls_surface_variation():
    """Wei et al. eq. 3 and `radius_pca.surface_variation` are one formula, two names.

    Both are `λ_min / Σλ` over a neighbourhood covariance. Pinned because it is the premise
    of the whole comparison: if these two methods disagree on a joint, the feature is not
    what separated them, and the test says so with numbers instead of with an argument.
    """
    pts = fold(50.0, 40)
    _, delta = local_pca(pts, radius_mm=4.0)
    V = surface_variation(pts, radius_mm=4.0, min_neighbors=3)
    ok = V > 0                                         # `ours` zeroes sparse points
    assert np.allclose(delta[ok], V[ok], atol=1e-9)


def test_a_knn_neighbourhood_cannot_cross_the_gap_and_a_ball_can():
    """`README §8`'s central claim, as an A/B on one code path.

    The parts do not touch. A k-NN neighbourhood on a point at part A's edge contains only
    part-A points, so it looks flat; a radius ball wider than the gap reaches across. This
    is why `ours` uses radius search, and it is the first thing separating the two methods.

    Measured **at the seam**, not over the whole cloud: a plate's own outer rim fires under
    both settings, so a global maximum cannot tell them apart. The k is chosen so its reach
    (~2 mm at this density) is under the 6 mm gap, and the ball is chosen over it — which is
    the entire content of the claim, and also the reason `k` is not a free parameter of this
    method so much as a decision about which joints it can see.
    """
    pts, _ = slabs("butt", root_gap_mm=6.0)
    _, d_knn = local_pca(pts, k=10)
    _, d_ball = local_pca(pts, radius_mm=8.0)

    # Points straddling the joint: the butt seam runs along x at y = 0, on the top face.
    at_seam = (np.abs(pts[:, 1]) < 4.0) & (pts[:, 2] > pts[:, 2].max() - 0.5)
    assert at_seam.sum() > 50
    assert np.median(d_ball[at_seam]) > 5.0 * np.median(d_knn[at_seam])


# --- Alg. 1 ------------------------------------------------------------------------------

def test_growth_starts_at_the_smoothest_point():
    """§III-D: *"growing from the smoothest region could improve efficiency."*

    The seed order is the paper's one stated design choice in Alg. 1, so it is pinned: the
    first region must claim the global curvature minimum.
    """
    pts = fold()
    normals, curv = local_pca(pts, K_NEIGHBORS)
    labels, _ = region_grow(pts, normals, curv, K_NEIGHBORS, smoothness_deg=10.0)
    assert labels[int(np.argmin(curv))] == 0


def test_the_angle_threshold_is_what_makes_it_a_boundary_detector():
    """Alg. 1's Threshold1 splits *surface* from *edge*, and that is the second difference.

    `ours` asks "is this point curved?". This asks "did the normal jump between these two
    points?" — so the seam comes back as the boundary between two surfaces rather than as a
    set of high-curvature points. Widen the threshold past the fold's own 90 deg and the
    distinction collapses: one surface, no edges.
    """
    pts = fold()
    normals, curv = local_pca(pts, K_NEIGHBORS)
    _, tight = region_grow(pts, normals, curv, K_NEIGHBORS, smoothness_deg=10.0)
    _, wide = region_grow(pts, normals, curv, K_NEIGHBORS, smoothness_deg=89.0)
    assert tight.sum() > 0
    assert wide.sum() < tight.sum()


def test_the_two_surface_test_is_the_cross_object_gate_without_the_oracle():
    """§III-D: an edge must *"represent the intersection between two surfaces"*.

    This is the most transferable idea in the paper for this project. `ours` needs
    `object_id` to know two points sit on different parts, and loses F1 0,75 -> 0,03 when it
    is withheld. Here the regions are the method's own output, so the same test runs with no
    oracle at all — and it must still reject a plate's own outer rim, which is a boundary
    with only ONE surface behind it.
    """
    pts = fold(60.0, 50)
    normals, curv = local_pca(pts, K_NEIGHBORS)
    labels, is_edge = region_grow(pts, normals, curv, K_NEIGHBORS, smoothness_deg=10.0)
    keep = two_surface_edges(pts, labels, is_edge, edge_radius_mm=4.0)
    assert keep.sum() > 0

    # Everything kept sits on the fold at y = z = 0, not on the outer rim at x or y = 60.
    on_fold = np.maximum(np.abs(pts[keep][:, 1]), np.abs(pts[keep][:, 2]))
    assert np.percentile(on_fold, 90) < 6.0


# --- the path ----------------------------------------------------------------------------

def test_the_plane_projection_is_what_saves_the_line_fit():
    """§III-D fits a plane, projects, then fits `(x, y)` — not a line through a 3D ribbon.

    `ours` removed its line fit because a total-least-squares line through a band lands in
    the band's middle, which is the mid-surface between two plates rather than the seam.
    The projection step is why this method can still fit a line: the ribbon is flattened
    onto its own plane first, so the spread that remains is along the seam, not across it.
    """
    rng = np.random.default_rng(0)
    t = rng.uniform(0, 100, 4000)
    ribbon = np.column_stack([t, rng.normal(0, 0.4, len(t)), rng.normal(0, 0.4, len(t))])
    poly = fit_seam(ribbon, curve="line")
    assert poly is not None
    d = poly[-1] - poly[0]
    assert abs(d[0]) > 95.0                            # spans the ribbon
    assert np.linalg.norm(np.cross(d / np.linalg.norm(d), [1, 0, 0])) < 0.05


def test_the_published_voxel_grid_is_the_only_constant_the_paper_gives():
    """§IV-A publishes 3 mm and nothing else. Every other default here is **tuned**.

    Worth pinning because it is the fact most likely to be lost when the numbers are written
    up. `k`, Threshold1, Threshold2 and the two-surface radius are given no value anywhere in
    the paper, so some choice has to be made; these were swept against constructed truth
    rather than inherited from PCL, because PCL's defaults put a 9 mm neighbourhood on an
    8 mm plate and mark every point in the scene as an edge.

    The asymmetry against `lit-ransac` matters for the Phase 4 protocol: that paper publishes
    all three of its constants, so it gets none of this freedom. Equal treatment of the seven
    means tuning what a paper leaves unspecified, and saying which ones those were.
    """
    assert VOXEL_MM == 3.0                             # §IV-A, the only published value
    assert (K_NEIGHBORS, SMOOTHNESS_DEG, CURVATURE_THRESH) == (10, 20.0, 0.03)
    pts = fold()
    assert len(preprocess(pts, 3.0)) < len(preprocess(pts, 1.0)) < len(pts)


def test_runs_end_to_end_on_a_generated_scene():
    """Against constructed truth, through the same harness every other method uses."""
    from baselines.metrics import matched_path_errors

    hit = _t_scene()
    if hit is None:
        pytest.skip("no T scene at 6 mm plate in out/phase3")
    xyz, gt = hit
    r = detect(xyz, segmentation_mask=seam_region_oracle(xyz, gt, end_margin_mm=0.0))
    assert r.used_segmentation_oracle
    assert r.n_regions >= 2, r.note
    rows = [e for e in matched_path_errors(r.polylines, gt) if e["matched"]]
    assert rows, r.note


# --- §III-C: the coarse stage is a KEYPOINT crop, and it is severe ----------------------

def t_plates(spacing: float = 2.0, length: float = 200.0, width: float = 120.0):
    """Two 200 x 120 plates in a T, and the seam along the edge where they meet.

    The base lies on `z = 0` spanning `y in [0, width]`; the web stands on `y = 0` rising to
    `z = width`. The seam is the line they share, `y = z = 0`, which is an *edge* of both
    plates — the geometry a weld-keypoint detector is trained to click.
    """
    x = np.arange(0.0, length + 1e-9, spacing)
    w = np.arange(0.0, width + 1e-9, spacing)
    base = np.stack(np.meshgrid(x, w), -1).reshape(-1, 2)
    web = np.stack(np.meshgrid(x, w[1:]), -1).reshape(-1, 2)   # z = 0 row is the base's
    pts = np.vstack([np.column_stack([base[:, 0], base[:, 1], np.zeros(len(base))]),
                     np.column_stack([web[:, 0], np.zeros(len(web)), web[:, 1]])])
    seam = [np.array([[0.0, 0.0, 0.0], [length, 0.0, 0.0]])]
    return pts, seam


def test_the_keypoint_crop_keeps_the_fraction_table_ii_publishes():
    """§III-C keeps 12,7 % of the cloud, and that is the number the stand-in has to hit.

    Table II (p. 6) is the only quantitative description of the coarse stage: 81 730 points
    in, 10 356 out, *"over 80% of redundant point clouds"* removed. The crop the harness
    used instead — `surface_intersection_crop` over all twelve faces of a two-plate joint —
    keeps 54-97 % of these scenes, i.e. most of the workpiece, and the audit measured what
    that does downstream: 9-31 % of all points returned as edges, 10-15 % of them within
    3 mm of a seam. So the fraction is not a detail of the crop, it *is* the coarse stage.

    `keypoint_crop` is an **ORACLE** — it reads the truth polylines, exactly as
    `seam_region_oracle` does for `lit-ransac`'s PointNet++ — and the radius that realises
    the published fraction is searched per scene, because it depends on the extent and the
    sampling density, neither of which the paper shares with this corpus.
    """
    pts, seam = t_plates()
    mask, info = keypoint_crop(pts, seam)

    assert info["searched"] is True
    assert abs(info["fraction"] - KEYPOINT_CROP_FRACTION) < 0.03, info
    assert abs(info["fraction"] - mask.mean()) < 1e-12
    assert info["n_kept"] == int(mask.sum()) and info["n_total"] == len(pts)

    lo, hi = KEYPOINT_RADIUS_BOUNDS_MM
    assert lo <= info["radius_mm"] <= hi                # the clamp held
    assert info["note"] == ""                           # ... without being hit

    # Everything kept is inside the realised radius of the seam, and everything dropped is
    # outside it: the mask IS the ball, not an approximation of one.
    d = distance_to_polylines(pts, seam)
    assert d[mask].max() <= info["radius_mm"] + 1e-9
    assert d[~mask].min() > info["radius_mm"]


def test_the_keypoint_crop_radius_can_be_given_instead_of_searched():
    """A fixed radius is the sweep handle; the search is only the default."""
    pts, seam = t_plates()
    mask, info = keypoint_crop(pts, seam, radius_mm=10.0)
    assert info["searched"] is False and info["radius_mm"] == 10.0
    assert distance_to_polylines(pts, seam)[mask].max() <= 10.0


def test_the_keypoint_crop_says_so_when_it_is_not_the_papers_crop():
    """With no seams to click there is no §III-C, and silence would be a lie.

    Returning an empty cloud would read downstream as "the method found no seam"; returning
    everything without a note would read as "the crop ran". It keeps everything and says it
    is not the paper's stage.
    """
    pts, _ = t_plates()
    mask, info = keypoint_crop(pts, [])
    assert mask.all() and info["fraction"] == 1.0
    assert "NOT" in info["note"]


# --- §III-D clause 1: high curvature ----------------------------------------------------

def flat_plate_with_a_label_seam(size: float = 100.0, n: int = 50, split: float = 50.0):
    """One flat plate, cut in two by a *label* boundary that no geometry corresponds to."""
    a = grid(size, n)
    pts = np.column_stack([a[:, 0], a[:, 1], np.zeros(len(a))])
    labels = (pts[:, 0] >= split).astype(int)
    is_edge = np.abs(pts[:, 0] - split) < 2.5          # as a boundary test would mark them
    return pts, labels, is_edge


def test_the_curvature_gate_rejects_a_label_boundary_on_a_flat_face():
    """§III-D p. 4: edges *"not only need to have high curvature but also"* span two surfaces.

    The conjunction is the whole sentence, and `two_surface_edges` used to test only its
    second half. A flat plate labelled into two halves satisfies that half perfectly — the
    neighbourhood spans two surfaces, both share is well over `min_region_share` — while δ
    is zero everywhere, because the plate is flat. Without the gate 100 points come back as
    weld seam on a plate with no weld on it; with it, none do.

    This is the mechanism behind the audit's headline number. Over-detection was never the
    two-surface test being too loose, it was the curvature clause never running.
    """
    pts, labels, is_edge = flat_plate_with_a_label_seam()
    _, curv = local_pca(pts, K_NEIGHBORS)
    assert curv.max() < CURVATURE_THRESH               # nothing here is an edge, by geometry

    without = two_surface_edges(pts, labels, is_edge, edge_radius_mm=6.0)
    gated = two_surface_edges(pts, labels, is_edge, edge_radius_mm=6.0, curvature=curv)
    assert without.sum() > 0                           # the pre-audit behaviour
    assert gated.sum() == 0                            # the paper's


def test_the_curvature_gate_keeps_a_real_fold():
    """The gate must reject flat-face boundaries without rejecting the seam it is there for."""
    pts = fold(60.0, 50)
    normals, curv = local_pca(pts, K_NEIGHBORS)
    labels, is_edge = region_grow(pts, normals, curv, K_NEIGHBORS, smoothness_deg=10.0)
    gated = two_surface_edges(pts, labels, is_edge, edge_radius_mm=4.0, curvature=curv)
    assert gated.sum() > 0
    on_fold = np.maximum(np.abs(pts[gated][:, 1]), np.abs(pts[gated][:, 2]))
    assert np.percentile(on_fold, 90) < 6.0


# --- §III-D with supplied surfaces: the label argument must be read ----------------------

def test_edge_from_labels_fires_on_a_label_boundary_and_only_there():
    """`_edge_from_labels` took `labels` and never looked at it. It does now.

    With the surfaces supplied (§III-C's FastSAM returns one mask per surface, so the
    supplied-labels arm is the rung the paper actually occupies), "edge" means *where two of
    those surfaces meet*. The old implementation tested only the normal angle, so every
    normal wobble in the cloud qualified and the `labels` argument was decoration.

    Pinned three ways: it fires at the fold, it fires **nowhere** when every point carries
    the same label — which is the property the old code could not have had — and it is a
    strict narrowing of the normal-only test rather than a different test.
    """
    pts = fold(100.0, 60)
    normals, _ = local_pca(pts, K_NEIGHBORS)
    labels = (pts[:, 2] > 0).astype(int)               # the standing plane is surface 1
    args = (pts, labels, normals, K_NEIGHBORS, 10.0, None, 3.0)

    fires = _edge_from_labels(*args, True)
    angle_only = _edge_from_labels(*args, False)
    assert fires.sum() > 0
    assert fires.sum() < angle_only.sum()              # a narrowing, not a replacement
    assert (fires & ~angle_only).sum() == 0

    # One surface everywhere: there is no intersection of two surfaces, so no edge.
    one_surface = _edge_from_labels(pts, np.zeros(len(pts), int), normals,
                                    K_NEIGHBORS, 10.0, None, 3.0, True)
    assert one_surface.sum() == 0

    # What does fire sits on the fold at y = z = 0, not on the plates' outer rims.
    on_fold = np.maximum(np.abs(pts[fires][:, 1]), np.abs(pts[fires][:, 2]))
    assert on_fold.max() < 4.0


# --- Alg. 1 lines 12-14 applied to every neighbour ---------------------------------------

def test_the_seed_test_applies_to_edge_neighbours_too_and_does_not_shatter_the_regions():
    """Alg. 1 lines 12-14 test `Curvature < Threshold2` on **every** neighbour.

    Lines 7-10 sort the neighbour into `S_edges` or `S_surfaces`; lines 12-14 then run
    unconditionally on it. The first implementation here `continue`d out of the loop after
    marking an edge, so that branch never ran on an edge point. `seed_edges=True` is the
    pseudocode read literally and is now the default.

    The risk it carries is growth leaking through a fold — an edge point's normal is a
    blend of the two faces, so seeding from one could merge the surfaces or fragment them.
    It does neither here, and that is a measurement rather than an assumption: an edge point
    is by construction a high-curvature point, so it rarely passes Threshold2 at all.
    """
    pts = fold(100.0, 60)
    normals, curv = local_pca(pts, K_NEIGHBORS)
    kw = dict(k=K_NEIGHBORS, smoothness_deg=10.0)
    lab_new, edge_new = region_grow(pts, normals, curv, seed_edges=True, **kw)
    lab_old, edge_old = region_grow(pts, normals, curv, seed_edges=False, **kw)

    n_new, n_old = int(lab_new.max()) + 1, int(lab_old.max()) + 1
    assert n_old == 2                                  # the fold is two planes
    assert n_new == n_old                              # and stays two
    assert edge_new.sum() == edge_old.sum()            # same edge set on this geometry
    # Edge points are never labelled either way: §III-D's two-surface ball needs them
    # unassigned so it can see labelled surface on both sides of the band.
    assert (lab_new[edge_new] == -1).all()


# --- the harness contract ----------------------------------------------------------------

def test_every_invented_constant_is_named_in_the_params():
    """Plan item 3: write down every input a method consumes, including the made-up ones.

    Five of this method's constants have no counterpart anywhere in the paper. They travel
    with the row so that a result turning on one of them is legible as a result about this
    reimplementation. The harness's 1,5 mm voxel rides along next to §IV-A's published 3 mm
    for the same reason — Deviation 7.
    """
    pts = fold(60.0, 50)
    r = detect(pts, voxel_mm=1.5)
    for name in ("min_region_pts", "min_cluster_pts", "link_mm", "edge_radius_mm",
                 "min_region_share"):
        assert name in r.params["invented"] and name in r.params
    assert r.params["voxel_mm"] == 1.5 and r.params["voxel_mm_published"] == VOXEL_MM
    assert r.params["edge_curvature_gate"] is True     # §III-D clause 1, default ON
    assert r.params["edge_curv_thresh"] == CURVATURE_THRESH
    assert r.params["seed_edges"] is True              # Alg. 1 lines 12-14, default ON
    assert r.params["label_span_radius_mm"] == 1.5     # one voxel
