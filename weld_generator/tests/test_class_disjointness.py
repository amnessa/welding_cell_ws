"""D31/D32 — joint classes disjoint by construction (patch_class_disjointness).

The claims under test: the clearance bands keep the T/corner and lap/edge borders
empty (and the D32 stratum inverts them); edge scenes declare the union (lap toes
in-class, cross-run demotion off for the stacked joints); underside lap toes carry a
distinct label the single-view arm scores around; the near-parallel clip waiver
restores the A-side lap toe that yaw x misalignment coupling silently deleted; and
none of it consumes a stream draw, so pre-D31 corpora reproduce bit-identically.
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from weldgen.config import load_config, sample_joint  # noqa: E402
from weldgen.joints import JointSpec  # noqa: E402
from weldgen.layouts import class_ambiguity, class_clearance_mm  # noqa: E402
from weldgen.rng import Streams  # noqa: E402
from weldgen.scene import (  # noqa: E402
    AmbiguousClassConfiguration, ClassBoundaryMiss, generate_scene,
)


def _spec(joint_type, **kw):
    p = dict(L_A=220.0, W_A=160.0, t_A=8.0, L_B=180.0, H_B=90.0, t_B=6.0,
             root_gap_mm=0.5, linear_misalignment_mm=0.0,
             angular_misalignment_deg=0.0,
             included_angle_deg=JointSpec.NOMINAL_INCLUDED_DEG[joint_type],
             stack_offset_mm=40.0 if joint_type == "lap" else
             (0.0 if joint_type == "edge" else None))
    p.update(kw)
    return JointSpec(**p)


# ------------------------------------------------------------------ the clearance


def test_clearance_is_twice_the_thinner_member():
    assert class_clearance_mm(_spec("T", t_A=8.0, t_B=6.0)) == 12.0
    assert class_clearance_mm(_spec("T", t_A=3.0, t_B=10.0)) == 6.0


def test_t_flags_corner_like_contact_only_when_near_parallel_and_near():
    # contact line at h + t_B/2 from A's centre; A's edge at W_A/2 = 80
    near = _spec("T", linear_misalignment_mm=80.0 - 6.0 - 3.0)   # ~6 mm from the edge
    assert class_ambiguity(near, "T") == ["corner"]
    # same position, yawed 30 deg: crossing the band at an angle is overhang, not corner
    yawed = _spec("T", linear_misalignment_mm=80.0 - 6.0 - 3.0, in_plane_yaw_deg=30.0)
    assert class_ambiguity(yawed, "T") == []
    # centred contact is clean, and end-overhang alone never triggers (L_B > L_A)
    assert class_ambiguity(_spec("T"), "T") == []
    assert class_ambiguity(_spec("T", L_B=400.0), "T") == []


def test_lap_bands_cover_both_flush_configurations_and_the_ends():
    c = class_clearance_mm(_spec("lap"))                          # 12 mm
    assert class_ambiguity(_spec("lap", stack_offset_mm=c * 0.5), "lap") == ["edge"]
    assert class_ambiguity(_spec("lap", stack_offset_mm=40.0), "lap") == []
    # flush at A's FAR edge: overlap ~ W_A
    far = _spec("lap", stack_offset_mm=160.0 - c * 0.5)
    assert class_ambiguity(far, "lap") == ["edge"]
    # cleanly overspanning is fine
    assert class_ambiguity(_spec("lap", stack_offset_mm=200.0), "lap") == []
    # end edges nearly coincident
    ends = _spec("lap", length_offset_mm=(220.0 - 180.0) / 2.0 - c * 0.4)
    assert class_ambiguity(ends, "lap") == ["edge"]
    # yaw breaks the parallelism, and with it the band
    for bad in (_spec("lap", stack_offset_mm=c * 0.5),):
        bad = _spec("lap", stack_offset_mm=c * 0.5, in_plane_yaw_deg=25.0)
        assert class_ambiguity(bad, "lap") == []


def test_edge_bands_apply_to_the_non_welded_pairs_only():
    c = class_clearance_mm(_spec("edge"))
    # exactly flush far edges (H_B == W_A) are a constructed edge seam, not ambiguity
    assert class_ambiguity(_spec("edge", H_B=160.0), "edge") == []
    # nearly flush far edges are the band
    assert class_ambiguity(_spec("edge", H_B=160.0 - c * 0.5), "edge") == ["lap"]
    assert class_ambiguity(_spec("edge", H_B=90.0), "edge") == []
    # nearly coincident end edges
    ends = _spec("edge", length_offset_mm=(220.0 - 180.0) / 2.0 - c * 0.4)
    assert class_ambiguity(ends, "edge") == ["lap"]


def test_lap_at_yaw_180_is_always_flush_and_rejected():
    """Rotating B by 180 about the strip centroid maps its leading edge exactly onto
    A's welded edge - the flush configuration - so laps cannot take yaw near 180."""
    assert class_ambiguity(_spec("lap", in_plane_yaw_deg=180.0), "lap") == ["edge"]
    assert class_ambiguity(_spec("lap", in_plane_yaw_deg=175.0), "lap") == ["edge"]
    assert class_ambiguity(_spec("lap", in_plane_yaw_deg=160.0), "lap") == []


def test_yaw_is_sampled_over_the_full_circle():
    """D28 amendment (ruled 2026-08-27): uniform over the feasible set of the full
    circle, not a +-bound - theta and theta+180 are different configurations."""
    from weldgen.layouts import feasible_yaw_deg, sample_yaw_deg
    spec = _spec("T", L_A=400.0, W_A=400.0, L_B=120.0)
    angles = feasible_yaw_deg(spec, "T")
    assert angles.min() < -170.0 and angles.max() > 170.0
    rng = np.random.default_rng(0)
    draws = [sample_yaw_deg(spec, "T", rng) for _ in range(300)]
    assert min(draws) < -120.0 and max(draws) > 120.0     # far beyond the old +-90
    assert sum(1 for d in draws if abs(d) > 90.0) > 50    # the new half is populated


def test_iso_derived_fields_map_by_sub_clause():
    from weldgen.scene import _iso_17659_term, _iso_9692_ref
    assert _iso_17659_term("T", 90.0) == "T-joint (3.10)"
    assert _iso_17659_term("T", 72.0) == "angle joint (3.12)"
    assert _iso_17659_term("edge", 0.0) == "edge joint (3.14)"
    assert _iso_9692_ref("T", 90.0, 8.0, "square") == "3.1.1/4.1.1"
    assert _iso_9692_ref("T", 65.0, 8.0, "square") == "3.1.3/4.1.2"
    assert _iso_9692_ref("T", 90.0, 1.8, "square") is None      # Table 3 wants t > 2
    assert _iso_9692_ref("butt", 180.0, 3.0, "square") == "1.2.1"
    assert _iso_9692_ref("butt", 180.0, 6.0, "square") == "2.1"
    assert _iso_9692_ref("butt", 180.0, 10.0, "square") is None
    assert _iso_9692_ref("edge", 0.0, 1.5, "square") == "1.1"
    assert _iso_9692_ref("lap", 0.0, 6.0, "square") is None


def test_corner_and_butt_have_no_bands():
    assert class_ambiguity(_spec("corner"), "corner") == []
    assert class_ambiguity(_spec("butt"), "butt") == []


# ------------------------------------------------------- generation-level behaviour


def _first(cfg, n=60, want=1):
    got = []
    for seed in range(n):
        try:
            got.append(generate_scene(cfg, seed))
        except Exception:
            continue
        if len(got) >= want:
            break
    assert got, "no scene emitted in the seed budget"
    return got


def test_disjoint_corpora_reject_in_band_seeds_and_stratum_inverts():
    cfg = load_config(str(ROOT / "configs" / "bench6a_lap.yaml"))
    seen_amb = seen_ok = False
    for seed in range(40):
        try:
            scene, _ = generate_scene(cfg, seed)
            seen_ok = True
            assert scene["joint"]["ambiguous_with"] is None
        except AmbiguousClassConfiguration:
            seen_amb = True
        except Exception:
            pass
    assert seen_ok and seen_amb, "expected both clean seeds and band rejections"

    amb_cfg = load_config(str(ROOT / "configs" / "amb_lap.yaml"))
    seen_ok = seen_miss = False
    for seed in range(40):
        try:
            scene, _ = generate_scene(amb_cfg, seed)
            seen_ok = True
            assert scene["joint"]["ambiguous_with"] == ["edge"]
        except ClassBoundaryMiss:
            seen_miss = True
        except Exception:
            pass
    assert seen_ok and seen_miss


def test_edge_scene_declares_the_union():
    """D31 combined configuration: lap toes are in-class for edge scenes, and the
    cross-run demotion no longer suppresses the off-direction boundary runs."""
    cfg = load_config(str(ROOT / "configs" / "bench6a_edge.yaml"))
    scene, _ = _first(cfg)[0]
    classes = {s["seam_class"] for s in scene["seams"]
               if s["weldable"] and s["matches_joint_type"]}
    assert "lap_toe" in classes, "the stack's lap toes must be in-class under D31"
    assert "edge" in classes, "the flush edge seam must survive alongside them"


def test_pre_d31_corpora_reproduce_bit_identically():
    """The acceptance checks consume no stream draws and default off: a pre-D31
    config yields field-identical specs, and enabling the flag changes only WHICH
    seeds emit, never what any seed draws."""
    cfg_off = load_config(str(ROOT / "configs" / "bench_lap.yaml"))
    cfg_on = dict(cfg_off)
    cfg_on["class_disjoint"] = True
    for seed in (0, 4, 9):
        a, qa, ja = sample_joint(cfg_off, Streams(seed))
        b, qb, jb = sample_joint(cfg_on, Streams(seed))
        assert (a, qa, ja) == (b, qb, jb)


# --------------------------------------------------------- underside toes (single view)


def test_underside_toe_is_flagged_and_single_view_scoring_skips_it():
    cfg = load_config(str(ROOT / "configs" / "bench6a_lap.yaml"))
    found = None
    for scene, arrays in _first(cfg, n=40, want=8):
        prim = [s for s in scene["seams"] if s["weldable"] and s["matches_joint_type"]]
        if any(s["underside"] for s in prim) and any(not s["underside"] for s in prim):
            found = (scene, arrays, prim)
            break
    assert found, "expected a lap scene carrying both an upper and an underside toe"
    scene, arrays, prim = found
    for s in scene["seams"]:
        if s["seam_class"] != "lap_toe":
            assert s["underside"] is None

    sys.path.insert(0, str(ROOT / "scripts"))
    from baselines.dataset import ground_truth
    gt, und = ground_truth(scene, arrays, with_underside=True)
    assert len(gt) == len(und) and any(und) and not all(und)

    from baselines.harness import PreparedScene
    from baselines.dataset import scene_facts
    prep = PreparedScene(pathlib.Path("."), scene, arrays, scene_facts(scene), gt,
                         gt_underside=und)
    assert len(prep.gt_for_scoring("single")) == sum(1 for u in und if not u)
    assert len(prep.gt_for_scoring("full_exterior")) == len(gt)


# ------------------------------------------------- the near-parallel clip regression


def test_yawed_misaligned_lap_keeps_its_a_side_toe():
    """Yaw x angular-misalignment coupling drifts the toe line out of A's edge-face
    plane at ~sin(beta) per mm, so it is NEAR-parallel instead of exactly parallel and
    the pre-waiver exact clip deleted the A-side (underside) toe of every such lap.
    The waiver drops a constraint only where its breach stays within the slack over
    the run the other constraints allow - it never extends a run."""
    from weldgen.accessibility import enumerate_candidates
    from weldgen.layouts import build
    spec = _spec("lap", in_plane_yaw_deg=20.0, angular_misalignment_deg=0.4,
                 root_gap_mm=0.65)
    parts = build(spec, "lap", np.eye(4))
    access = {k: (v.copy() if isinstance(v, dict) else v)
              for k, v in load_config()["accessibility"].items()}
    access["contact_tol_mm"] = 2.0 * spec.root_gap_mm + 0.5
    cands = enumerate_candidates(parts, access, joint_type="lap",
                                 class_disjoint=True)
    pairs = {c.face_pair for c in cands if c.weldable}
    assert ("A:+v", "B:-w") in pairs, \
        "the A-side toe must survive yaw x misalignment coupling"
