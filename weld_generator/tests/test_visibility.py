"""Phase 3 gate — the visibility layer.

`visible_from_cam` is a statement about geometry: framed, in range, front-facing,
unoccluded. Sensor dropout is deliberately elsewhere (`noise.apply`), so occlusion stays
comparable across sensor profiles. These tests pin that split as much as the arithmetic.
"""

from __future__ import annotations

import numpy as np
import pytest

from weldgen.camera import in_frustum, intrinsics, look_at, project, sample_pose
from weldgen.config import SENSOR_PROFILES, load_config
from weldgen.geom import Slab, translate
from weldgen.layouts import build
from weldgen.noise import apply as apply_noise
from weldgen.noise import sigma_z_mm
from weldgen.sampling import raster_density_per_mm2
from weldgen.scene import NoSeamsFound, SceneRejected, generate_scene
from weldgen.visibility import hpr_exterior, occluded, ray_hits_slab, visible_mask

K = intrinsics(674.0, 1280, 720)


def box(centre, dims) -> Slab:
    return Slab("X", "workpiece", 0, tuple(float(v) for v in dims),
                translate(*(float(v) for v in centre)))


# --- ray-box intersection ------------------------------------------------------------

@pytest.mark.parametrize("origin,direction,expect", [
    ([0, 0, 20], [0, 0, 1], False),      # above, pointing away
    ([0, 0, 20], [0, 0, -1], True),      # above, pointing at it
    ([0, 0, 5.001], [0, 0, 1], False),   # just off the surface, leaving
    ([20, 0, 0], [0, 0, 1], False),      # beside it, parallel to the face
    ([0, 0, 0], [0, 0, 1], True),        # starting inside
])
def test_ray_box_cases(origin, direction, expect):
    b = box((0, 0, 0), (10, 10, 10))
    got = ray_hits_slab(np.array([origin], float), np.array([direction], float),
                        np.array([1e9]), b)
    assert bool(got[0]) is expect


def test_a_hit_beyond_t_max_is_not_a_hit():
    """Occlusion asks "is anything BETWEEN me and the camera", not "anywhere along"."""
    b = box((0, 0, 0), (10, 10, 10))
    ray = (np.array([[0.0, 0.0, 20.0]]), np.array([[0.0, 0.0, -1.0]]))
    assert ray_hits_slab(*ray, np.array([5.0]), b)[0] == False      # noqa: E712
    assert ray_hits_slab(*ray, np.array([50.0]), b)[0] == True      # noqa: E712


def test_a_surface_point_does_not_occlude_itself():
    """Without the epsilon lift every point sits on its own face and reads as blocked."""
    b = box((0, 0, 0), (100, 100, 10))
    top = np.array([[0.0, 0.0, 5.0], [20.0, -30.0, 5.0]])
    n = np.tile([0.0, 0.0, 1.0], (2, 1))
    assert not occluded(top, np.array([0.0, 0.0, 500.0]), [b], n).any()


# --- camera --------------------------------------------------------------------------

def test_look_at_is_right_handed_and_points_at_the_target():
    T = look_at(np.array([300.0, 100.0, 400.0]), np.zeros(3))
    r, d, f = T[:3, 0], T[:3, 1], T[:3, 2]
    assert np.allclose(np.cross(r, d), f, atol=1e-9), "OpenCV needs x cross y = z"
    for a in (r, d, f):
        assert float(np.linalg.norm(a)) == pytest.approx(1.0, abs=1e-9)
    # +Z looks from the eye toward the target.
    assert float(f @ (np.zeros(3) - T[:3, 3])) > 0.0
    # +Y is down: it must have a negative world-z component for any non-degenerate view.
    assert float(d[2]) < 0.0


def test_elevation_is_measured_from_the_world_xy_plane():
    """SCHEMA.md §1.1. Elevation is not a polar angle from +Z."""
    for el in (15.0, 45.0, 85.0):
        T = sample_pose(np.zeros(3), 600.0, el, 37.0, 0.0)
        got = np.degrees(np.arcsin(T[2, 3] / np.linalg.norm(T[:3, 3])))
        assert got == pytest.approx(el, abs=1e-9)


def test_the_blind_zone_belongs_to_the_profile():
    """D16: `min_z_mm` is a sensor property, not a schema constant."""
    uv = np.array([[640.0, 360.0]])
    for name, prof in SENSOR_PROFILES.items():
        z = np.array([prof["min_z_mm"] * 0.9])
        assert not in_frustum(uv, z, 1280, 720, prof["min_z_mm"])[0], name
        assert in_frustum(uv, z * 2.0, 1280, 720, prof["min_z_mm"])[0], name


def test_points_outside_the_image_are_not_visible():
    b = box((0, 0, 0), (10, 10, 10))
    T = look_at(np.array([0.0, 0.0, 500.0]), np.zeros(3))
    far = np.array([[0.0, 0.0, 5.0], [900.0, 0.0, 5.0]])
    n = np.tile([0.0, 0.0, 1.0], (2, 1))
    got = visible_mask(far, n, [b], T, K, 1280, 720, 280.0)
    assert got[0] and not got[1]


# --- occlusion -----------------------------------------------------------------------

def test_an_occluder_hides_the_point_and_removing_it_reveals_it():
    target = box((0, 0, 0), (100, 100, 10))
    wall = box((0, 0, 200), (400, 400, 10))
    T = look_at(np.array([0.0, 0.0, 900.0]), np.zeros(3))
    p = np.array([[0.0, 0.0, 5.0]])
    n = np.array([[0.0, 0.0, 1.0]])
    assert not visible_mask(p, n, [target, wall], T, K, 1280, 720, 280.0)[0]
    assert visible_mask(p, n, [target], T, K, 1280, 720, 280.0)[0]


def test_back_facing_surface_points_are_invisible():
    b = box((0, 0, 0), (100, 100, 10))
    T = look_at(np.array([0.0, 0.0, 900.0]), np.zeros(3))
    under = np.array([[0.0, 0.0, -5.0]])
    assert not visible_mask(under, np.array([[0.0, 0.0, -1.0]]), [b],
                            T, K, 1280, 720, 280.0)[0]


def test_a_seam_is_not_subject_to_the_facing_test():
    """A seam lies on the crease, on the boundary of both solids.

    Pressing `approach_dir` into service as a surface normal rejects a fillet the moment
    the camera passes 90 deg from the torch direction — which on a T-joint is most of the
    hemisphere it is plainly visible from. Every seam in the first Phase 3 scenes came
    back `occluded_fraction: 1.0` because of it.
    """
    b = box((0, 0, 0), (200, 100, 10))
    T = look_at(np.array([0.0, 0.0, 900.0]), np.zeros(3))
    edge = np.array([[0.0, 50.0, 5.0]])
    approach = np.array([[0.0, 0.9, -0.44]])       # tilted well away from the camera
    assert not visible_mask(edge, approach, [b], T, K, 1280, 720, 280.0, face_test=True)[0]
    assert visible_mask(edge, approach, [b], T, K, 1280, 720, 280.0, face_test=False)[0]


# --- noise model ---------------------------------------------------------------------

@pytest.mark.parametrize("z,expect", [(300, 0.21), (500, 0.59), (1000, 2.37), (2000, 9.50)])
def test_sigma_z_matches_the_published_table(z, expect):
    """PARAMETERS.md §4.2, the `d435i` column. Derived, not tuned."""
    prof = SENSOR_PROFILES["d435i"]
    nm = {"subpixel_px": prof["subpixel_px"], "focal_px": prof["focal_px"],
          "baseline_mm": prof["baseline_mm"]}
    assert float(sigma_z_mm(z, nm)) == pytest.approx(expect, abs=0.01)


def test_noise_is_deterministic_in_its_seed_and_not_stored():
    cfg = load_config()
    cfg["joint_type"] = ["T"]
    scene, arrays = generate_scene(cfg, 4242)
    xyz = arrays["cloud.npz:xyz"].astype(float)
    nrm = arrays["cloud.npz:normals"].astype(float)
    T = np.array(scene["camera"]["T_world_cam"])
    a, _ = apply_noise(xyz, nrm, T, scene["noise_model"])
    b, _ = apply_noise(xyz, nrm, T, scene["noise_model"])
    assert np.array_equal(a, b), "same params must give the same realisation"
    # SCHEMA.md §5.1 - the realisation is a convention, not an artefact. Only the clean
    # cloud is stored, so `apply` must be the only place it exists.
    assert not any(k.endswith("xyz_noisy") for k in arrays)


def test_grazing_incidence_drops_out_and_head_on_does_not():
    prof = SENSOR_PROFILES["d435i"]
    nm = {**prof, "lateral_sigma_px": 0.8, "grazing_dropout_deg": 75.0, "seed": 1}
    T = look_at(np.array([0.0, 0.0, 600.0]), np.zeros(3))
    p = np.zeros((2, 3))
    n = np.array([[0.0, 0.0, 1.0],                       # head on
                  [np.sin(np.deg2rad(80.0)), 0.0, np.cos(np.deg2rad(80.0))]])
    _, valid = apply_noise(p, n, T, nm)
    assert valid[0] and not valid[1]


def test_noise_grows_with_range_as_z_squared():
    prof = SENSOR_PROFILES["d435i"]
    nm = {**prof, "lateral_sigma_px": 0.8, "grazing_dropout_deg": 75.0, "seed": 7}
    n = np.tile([0.0, 0.0, 1.0], (4000, 1))
    spread = []
    for z in (400.0, 800.0):
        T = look_at(np.array([0.0, 0.0, z]), np.zeros(3))
        noisy, _ = apply_noise(np.zeros((4000, 3)), n, T, nm)
        spread.append(float(noisy[:, 2].std()))
    assert spread[1] / spread[0] == pytest.approx(4.0, rel=0.15)


# --- D20 raster mode -----------------------------------------------------------------

def test_raster_density_falls_off_with_the_square_of_range():
    assert raster_density_per_mm2(500.0, 674.0) / raster_density_per_mm2(1000.0, 674.0) \
        == pytest.approx(4.0, rel=1e-9)


def test_camera_raster_keeps_the_hidden_surface():
    """D20: a raster alone leaves the mask all-True and collapses `occluded_fraction`.

    So the hidden surface is sampled too, rate-matched, and flagged false. Without it the
    metric reads zero exactly when the scene is hardest.
    """
    cfg = load_config()
    cfg["joint_type"] = ["T"]
    cfg["sampling_mode"] = "camera_raster"
    scene, arrays = generate_scene(cfg, 99)
    vis = arrays["cloud.npz:visible_from_cam"]
    assert scene["cloud"]["sampling_mode"] == "camera_raster"
    assert vis.any() and not vis.all(), "both mask classes must be present"


# --- the gate ------------------------------------------------------------------------

def test_occluded_fraction_spans_the_range_and_is_not_always_zero():
    """Phase 3 gate. A sampler that never hides a seam has no difficulty axis.

    The distribution is close to binary and that is honest: a straight seam under a convex
    occluder is shadowed all-or-nothing. Graded values need a third occluder or a curved
    seam, which is Phase 6. What must hold now is that both extremes occur often.
    """
    vals = []
    for jt in ("T", "corner", "butt", "lap", "edge"):
        cfg = load_config()
        cfg["joint_type"] = [jt]
        # The gate is about the RAW sampler, so the tier-1 omission policy is off: with it
        # on, the scenes it drops are exactly the ones at the top of the distribution.
        cfg["require_visible_seam"] = False
        for seed in range(7_000_000, 7_000_020):
            try:
                scene, _ = generate_scene(cfg, seed)
            except NoSeamsFound:
                continue
            vals += [s["occluded_fraction"] for s in scene["seams"] if s["weldable"]]

    v = np.array(vals)
    assert len(v) > 100
    assert v.min() < 0.05, "some seam must be plainly visible"
    assert v.max() > 0.80, "some seam must be mostly hidden - that is the difficulty axis"
    assert 0.15 < (v > 0.5).mean() < 0.85, "not all-or-nothing across the dataset"


def test_seam_occluded_fraction_agrees_with_its_stored_mask():
    cfg = load_config()
    cfg["joint_type"] = ["corner"]
    cfg["require_visible_seam"] = False        # this seed's camera sees neither seam
    scene, arrays = generate_scene(cfg, 31337)
    for s in scene["seams"]:
        mask = arrays[f"seams.npz:{s['sampled']['array']}_visible"]
        assert s["occluded_fraction"] == pytest.approx(1.0 - mask.mean(), abs=1e-9)


# --- HPR exteriority -----------------------------------------------------------------

def test_hpr_marks_the_buried_lap_interface_interior():
    """The CAD-free exteriority test the Phase 4 baselines need.

    Ground truth does not use this - it knows the transforms. A point-cloud-only method
    does not, and this is how it rejects the buried interface radius-PCA fires on.
    """
    from weldgen.geom import SLAB_FACES
    from weldgen.joints import JointSpec
    from weldgen.sampling import sample_scene_surface

    spec = JointSpec(L_A=200.0, W_A=100.0, t_A=8.0, L_B=200.0, H_B=80.0, t_B=8.0,
                     root_gap_mm=0.5, linear_misalignment_mm=0.0,
                     angular_misalignment_deg=0.0, included_angle_deg=0.0,
                     stack_offset_mm=60.0)
    parts = build(spec, "lap", np.eye(4))          # A spans y in [-100, 0], B in [-60, 20]
    cloud = sample_scene_surface(parts, 0.2, np.random.default_rng(0))
    ext = hpr_exterior(cloud["xyz"].astype(float), n_views=20)

    top_of_A = (cloud["object_id"] == 0) & (cloud["face_id"] == SLAB_FACES.index("+w"))
    y = cloud["xyz"][:, 1]
    buried = top_of_A & (y > -60.0) & (y < 0.0)    # roofed by B, 0,5 mm away
    exposed = top_of_A & (y < -62.0)               # the same face, past B's edge

    assert buried.sum() > 100 and exposed.sum() > 100
    assert ext[buried].mean() < 0.05, "the mid-lap interface must read as interior"
    assert ext[exposed].mean() > 0.95, "...and the same face outside the overlap must not"


# --- the tier-1 omission policy -------------------------------------------------------

def test_scenes_with_no_visible_seam_are_omitted_not_relabelled():
    """A "no seam" label would encode joint type and camera placement, not the task.

    Whether a seam is visible depends on where the camera landed and, structurally, on the
    joint type - the lower toe of a lap joint is never visible from above the table. A
    model would learn to predict the label from the wrong evidence, which is a false
    positive manufactured by a design choice. So the scene is dropped instead.
    """
    from weldgen.scene import NoVisibleSeams

    cfg = load_config()
    cfg["joint_type"] = ["corner"]
    with pytest.raises(NoVisibleSeams):
        generate_scene(cfg, 31337)

    cfg["require_visible_seam"] = False
    scene, _ = generate_scene(cfg, 31337)      # same seed, policy off -> a scene
    assert scene["seams"], "the geometry itself was never the problem"


def test_every_emitted_scene_carries_a_seam_worth_supervising():
    cfg = load_config()
    cfg["joint_type"] = ["T", "corner", "butt", "lap", "edge"]
    kept = 0
    for seed in range(8_600_000, 8_600_040):
        try:
            scene, _ = generate_scene(cfg, seed)
        except SceneRejected:
            continue
        kept += 1
        best = min(s["occluded_fraction"] for s in scene["seams"] if s["weldable"])
        assert best <= cfg["max_occluded_fraction"]
    assert kept > 10, "the policy must not reject nearly everything"


def test_occlusion_is_still_recorded_on_the_scenes_that_survive():
    """Tier 1 does not use occlusion; tier 2 does, so it is kept either way."""
    cfg = load_config()
    cfg["joint_type"] = ["T"]
    for seed in range(8_700_000, 8_700_010):
        try:
            scene, arrays = generate_scene(cfg, seed)
        except SceneRejected:
            continue
        assert all("occluded_fraction" in s for s in scene["seams"])
        assert any(f"{s['sampled']['array']}_visible" in k
                   for s in scene["seams"] for k in arrays)
        return
    pytest.fail("no scene survived the policy in ten seeds")
