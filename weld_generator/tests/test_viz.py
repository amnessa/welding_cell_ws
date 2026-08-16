"""The assembly view frame must undo the sampled world pose.

Written because the first cross-section plot sliced on world X and plotted world (y, z),
which is not a cross-section of anything once the scene carries a random yaw — the seam
markers wandered onto the face of the standing plate and moved with every seed.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from weldgen.config import load_config
from weldgen.scene import generate_scene
from weldgen.viz import assembly_frame, cross_section, to_assembly

ROOT = Path(__file__).resolve().parents[1]
SEEDS = [8412337, 7, 99, 123456789]


@pytest.fixture(scope="module")
def cfg():
    return load_config(str(ROOT / "configs" / "reference_tjoint.yaml"))


@pytest.mark.parametrize("seed", SEEDS)
def test_frame_is_rigid(cfg, seed):
    scene, _ = generate_scene(cfg, seed)
    T = assembly_frame(scene)
    R = T[:3, :3]
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-9)
    assert np.linalg.det(R) == pytest.approx(1.0, abs=1e-9)


@pytest.mark.parametrize("seed", SEEDS)
def test_base_plate_top_face_is_z_zero(cfg, seed):
    """Whatever the world pose, A must occupy z in [-t_A, 0]."""
    scene, arrays = generate_scene(cfg, seed)
    T = assembly_frame(scene)
    xyz = to_assembly(arrays["cloud.npz:xyz"], T)
    oid = arrays["cloud.npz:object_id"]
    t_A = next(o for o in scene["objects"] if o["id"] == "A")["thickness_mm"]

    a = xyz[oid == 0]
    assert a[:, 2].max() == pytest.approx(0.0, abs=1e-3)
    assert a[:, 2].min() == pytest.approx(-t_A, abs=1e-3)


@pytest.mark.parametrize("seed", SEEDS)
def test_standing_plate_sits_above_the_gap(cfg, seed):
    """B's lowest point is exactly the root gap above A's top face."""
    scene, arrays = generate_scene(cfg, seed)
    T = assembly_frame(scene)
    xyz = to_assembly(arrays["cloud.npz:xyz"], T)
    oid = arrays["cloud.npz:object_id"]
    g = scene["fit"]["root_gap_mm"]

    b = xyz[oid == 1]
    assert b[:, 2].min() == pytest.approx(g, abs=1e-3)
    assert b[:, 1].min() == pytest.approx(0.0, abs=1e-3)   # B stands at y >= 0


@pytest.mark.parametrize("seed", SEEDS)
def test_seam_runs_along_local_x(cfg, seed):
    """In the assembly frame the seam is a straight run along +X at constant (y, z)."""
    scene, arrays = generate_scene(cfg, seed)
    T = assembly_frame(scene)
    for s in scene["seams"]:
        p = to_assembly(arrays[f'seams.npz:seam_{s["id"]}'], T)
        assert np.ptp(p[:, 1]) < 1e-3
        assert np.ptp(p[:, 2]) < 1e-3
        assert np.ptp(p[:, 0]) == pytest.approx(s["length_mm"], abs=1e-3)


@pytest.mark.parametrize("seed", SEEDS)
def test_the_three_d19_curves_land_where_they_should(cfg, seed):
    """The whole point of the fix.

    nominal on A's top face (z = 0); root at B's bottom edge (z = g); gap_mid exactly
    midway between the two plates (z = g/2). All three at the same y as their seam.
    """
    scene, arrays = generate_scene(cfg, seed)
    T = assembly_frame(scene)
    g = scene["fit"]["root_gap_mm"]

    for s in scene["seams"]:
        i = s["id"]
        nom = to_assembly(arrays[f"seams.npz:seam_{i}"], T)[0]
        root = to_assembly(arrays[f"seams.npz:seam_{i}_root"], T)[0]
        mid = to_assembly(arrays[f"seams.npz:seam_{i}_gapmid"], T)[0]

        # TOL: these come from the stored float32 arrays (SCHEMA.md §1), so 1e-6 mm is
        # below the storage precision. 1e-3 mm is a micrometre - far tighter than anything
        # the dataset claims, and it still pins the g / g/2 / 0 structure exactly.
        assert nom[2] == pytest.approx(0.0, abs=1e-3)
        assert root[2] == pytest.approx(g, abs=1e-3)
        assert mid[2] == pytest.approx(g / 2.0, abs=1e-3)
        # ...and all three sit on the same seam line in y.
        assert root[1] == pytest.approx(nom[1], abs=1e-3)
        assert mid[1] == pytest.approx(nom[1], abs=1e-3)
        # The ordering is exact regardless of tolerance: nominal < gap_mid < root.
        assert nom[2] < mid[2] < root[2] or g == 0.0


@pytest.mark.parametrize("seed", SEEDS)
def test_seams_sit_on_the_standing_plate_faces(cfg, seed):
    """The two fillets are at y = 0 and y = t_B — the faces of B, not arbitrary points."""
    scene, arrays = generate_scene(cfg, seed)
    T = assembly_frame(scene)
    t_B = next(o for o in scene["objects"] if o["id"] == "B")["thickness_mm"]
    ys = sorted(to_assembly(arrays[f'seams.npz:seam_{s["id"]}'], T)[0, 1]
                for s in scene["seams"])
    assert ys[0] == pytest.approx(0.0, abs=1e-3)
    assert ys[1] == pytest.approx(t_B, abs=1e-3)


@pytest.mark.parametrize("seed", SEEDS)
def test_gap_mid_is_equidistant_from_both_plates(cfg, seed):
    """The literal reading of "gap_mid": equally far from each plate face.

    Stated as a distance to the two bounding faces rather than as a z coordinate, because
    that is the property someone actually checks by eye — and it holds in ANY frame, so it
    cannot be faked by a convenient choice of axes.
    """
    from weldgen.geom import Slab

    scene, arrays = generate_scene(cfg, seed)
    g = scene["fit"]["root_gap_mm"]
    objs = {o["id"]: o for o in scene["objects"]}
    A = Slab("A", "workpiece", 0, tuple(objs["A"]["dims_mm"]),
             np.array(objs["A"]["T_world_part"]))
    B = Slab("B", "workpiece", 1, tuple(objs["B"]["dims_mm"]),
             np.array(objs["B"]["T_world_part"]))
    top_of_A = A.face_plane("+w")      # one side of the gap
    bottom_of_B = B.face_plane("-v")   # the other side

    for s in scene["seams"]:
        mid = arrays[f'seams.npz:seam_{s["id"]}_gapmid'][0].astype(float)
        d_a = abs(float(top_of_A.signed_distance(mid)))
        d_b = abs(float(bottom_of_B.signed_distance(mid)))
        assert d_a == pytest.approx(d_b, abs=1e-4), "gap_mid is not centred in the gap"
        assert d_a == pytest.approx(g / 2.0, abs=1e-4)


@pytest.mark.parametrize("seed", SEEDS)
def test_root_lies_on_the_standing_plate_bottom_edge(cfg, seed):
    """And `root` sits ON B's bottom face, at zero distance — it is B's own edge."""
    from weldgen.geom import Slab

    scene, arrays = generate_scene(cfg, seed)
    objs = {o["id"]: o for o in scene["objects"]}
    B = Slab("B", "workpiece", 1, tuple(objs["B"]["dims_mm"]),
             np.array(objs["B"]["T_world_part"]))
    bottom_of_B = B.face_plane("-v")

    for s in scene["seams"]:
        root = arrays[f'seams.npz:seam_{s["id"]}_root'][0].astype(float)
        assert abs(float(bottom_of_B.signed_distance(root))) == pytest.approx(0.0, abs=1e-4)


def test_cross_section_selects_a_band(cfg):
    scene, arrays = generate_scene(cfg, SEEDS[0])
    T = assembly_frame(scene)
    xyz = to_assembly(arrays["cloud.npz:xyz"], T)
    m = cross_section(xyz, half_width_mm=3.0)
    assert m.any()
    assert np.ptp(xyz[m][:, 0]) <= 6.0 + 1e-6
