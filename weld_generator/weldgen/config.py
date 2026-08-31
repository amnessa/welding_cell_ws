"""Configuration and parameter sampling — PARAMETERS.md.

Every range here traces to a line in PARAMETERS.md. Where a value is [ISO] the clause is
named in a comment, so a reader can check it against the standard rather than trusting
this file.
"""

from __future__ import annotations

from typing import Any

import numpy as np
import yaml

from .joints import JointSpec
from .rng import Streams

#: Sensor profiles — PARAMETERS.md §4.0 (D16). All three ship in the release.
SENSOR_PROFILES: dict[str, dict[str, float]] = {
    "d435i":       {"baseline_mm": 50.0,  "focal_px": 674.0,  "subpixel_px": 0.08, "min_z_mm": 280.0},
    "stereo_good": {"baseline_mm": 120.0, "focal_px": 1100.0, "subpixel_px": 0.05, "min_z_mm": 200.0},
    "stereo_poor": {"baseline_mm": 35.0,  "focal_px": 450.0,  "subpixel_px": 0.15, "min_z_mm": 400.0},
}

DEFAULT_CONFIG: dict[str, Any] = {
    "name": "phase1",
    # --- geometry, substream 0 (PARAMETERS.md §3) --------------------------------
    # Joint type is sampled (Phase 2). A single string still works as a fixed choice.
    "joint_type": ["T", "corner", "butt", "lap", "edge"],
    "seam_shape": "line",
    "prep": "square",
    "plate_length_mm": [80.0, 400.0],
    #: L_A and L_B are drawn independently and B is shifted along the seam, so seams clip
    #: to the SHARED run instead of always spanning the full plate. This floor keeps the
    #: overlap from collapsing to nothing.
    "min_overlap_frac": 0.6,
    # D28 (Phase 6a): in-plane yaw of B for T and lap. Off by default so every config and
    # corpus generated before Phase 6a reproduces bit-identically; new configs opt in.
    "in_plane_yaw": False,
    # D28, Phase 6a - polygon outlines for the edge-sharing joints (corner, butt, edge).
    # Same contract as in_plane_yaw: draws come from the `seam_curve` substream, so
    # corpora generated with this off reproduce bit-identically and an enabled
    # regeneration differs only in the outlines - a free twin.
    "polygon_outlines": False,
    # D31 - joint classes disjoint by construction: the clearance bands reject seeds
    # whose configuration resembles another class, and classification switches to the
    # combined-edge semantics (lap toes in-class for edge scenes, no cross-run demotion
    # for the stacked joints). Off by default so pre-D31 corpora reproduce exactly.
    "class_disjoint": False,
    # D32 - the class-boundary stratum: INVERT the D31 acceptance so only in-band seeds
    # emit, each recording `joint.ambiguous_with`. Used by the amb_* presets only.
    "class_boundary_stratum": False,
    # Phase 6b - D29 curve families for curved scenes. None keeps the plate pipeline;
    # a list of family ids (2..7) routes generation through scene_curved.
    "seam_families": None,
    "plate_width_mm": [50.0, 250.0],
    "thickness_mm": [1.0, 12.0],
    "dissimilar_thickness_p": 0.30,
    # ISO 9692-1 Tables 3-4: 60 <= alpha <= 120 for square-preparation fillets.
    "included_angle_deg": [60.0, 120.0],
    # lap overlap, as a fraction of part B's width. [ours] - no ISO citation exists
    # (PARAMETERS.md §2.6); AWS D1.1 may give a minimum as a multiple of t.
    "stack_offset_frac": [0.15, 0.60],
    # ISO 9692-1 Table 1 ref 1.1 "raised edges" applies at t <= 2 mm, so edge joints are
    # a thin-sheet preparation by the standard's own scope, not by our choice.
    "edge_max_thickness_mm": 2.0,
    #: Angular misalignment cap for face-to-face stacked joints (lap, edge). See the note
    #: in sample_joint: the clamp physically prevents relative tilt.
    "stacked_max_beta_deg": 0.4,
    #: Probability that an edge joint's two parts share a width exactly. Equal widths make
    #: BOTH long edges flush (2 edge seams); unequal widths leave one flush edge and one
    #: lap toe (1 edge seam). Continuous sampling would otherwise never produce a match.
    "edge_equal_width_p": 0.35,
    # --- defects, substream 1 (PARAMETERS.md §2) ---------------------------------
    "quality_mix": {"B": 0.25, "C": 0.25, "D": 0.25, "below_D": 0.25},
    # ISO 9692-1 Tables 3-4 cap the fillet gap at b <= 2 mm; the over-range tail to
    # 3 mm is [ours] and is what generates `below_D` and breaks radius-PCA on purpose.
    "root_gap_mm": [0.0, 2.0],
    "root_gap_over_range_mm": 3.0,
    # --- placement, substream 3 --------------------------------------------------
    # D12: off in Phase 1, sampled ~50/50 in exact pairs from Phase 2. Pose-varied so
    # the fixture is not identifiable by pose alone - a working surface pinned to z = 0
    # teaches "contacts with z = 0 are never weldable", which transfers to nothing.
    "fixture_present": False,
    "fixture_tilt_deg": 10.0,
    "fixture_height_mm": [-50.0, 50.0],
    "fixture_dims_mm": {"length": [480.0, 720.0],
                        "width": [320.0, 480.0],
                        "thickness": [8.0, 12.0]},
    "assembly_translation_mm": 150.0,
    # --- sampling, substream 4 ---------------------------------------------------
    "density_per_mm2": [0.25, 4.0],
    # --- camera / sensor, substreams 5-6 -----------------------------------------
    "sensor_profiles": ["d435i", "stereo_good", "stereo_poor"],
    "standoff_mm": [300.0, 1200.0],
    # 15 deg is not a nuisance bound - below about 20 deg the standing plate of a T-joint
    # blocks its own seam, which IS the difficulty axis (PARAMETERS.md §4.1).
    "elevation_deg": [15.0, 85.0],
    "camera_roll_deg": [-15.0, 15.0],   # eye-in-hand mounting is not gravity-aligned
    # Standoff is chosen so the assembly's longest extent projects to this fraction of the
    # SHORT image side. Above 1,0 the assembly overflows the frame and a seam gets clipped
    # partway - the only graded source of lost visibility that straight seams on convex
    # slabs can produce, and what Phase 4's error-vs-visibility plot needs to be a curve
    # rather than two points. Sampling standoff uniformly instead framed everything.
    "frame_by_extent": True,
    "framing_frac": [0.35, 1.45],
    "image_size": [1280, 720],
    # Tier 1 omits scenes whose every weldable seam is hidden: they carry no supervision,
    # and labelling them "no seam" would encode joint type and camera placement rather
    # than anything about the task (see scene.NoVisibleSeams). The occlusion figures are
    # still recorded on every scene that IS emitted - tier 2 uses them.
    "require_visible_seam": True,
    # A scene is omitted unless some primary seam has at least this fraction of its length
    # returned by the sensor. Deliberately LOW: "no visible seam" means essentially
    # nothing, and a seam a quarter in frame is still supervisable. A high bar here would
    # eat the partially-framed band - the graded middle of the visibility axis, which only
    # exists because `framing_frac` puts the assembly out of frame on purpose - and
    # filtering out the very examples that make Phase 4's error-vs-visibility plot a curve
    # would be self-defeating.
    "min_visible_fraction": 0.1,
    # How far the aim may miss the joint, as a fraction of the longest workpiece edge.
    # Non-zero on purpose: aiming exactly at the seam would pin it to the image centre and
    # let a model read the answer off the camera pose (see `scene._aim_point`).
    "aim_jitter_frac": 0.15,
    # area_uniform | camera_raster (D20). area_uniform is the default because it is the
    # full-surface variant the annotation-error and R-window experiments are computed on;
    # camera_raster is the genuinely camera-like benchmark condition.
    "sampling_mode": "area_uniform",
    # --- accessibility, recorded so every reject_reason is reproducible ----------
    "accessibility": {
        "torch_clearance": {"half_angle_deg": 30.0, "standoff_mm": 15.0},
        "max_work_angle_deg": 45.0,   # how far the torch may tilt off the bisector
        "dihedral_min_deg": 30.0,
        "dihedral_max_deg": 170.0,
        "contact_tol_mm": 4.0,   # scene.py widens this per scene to track the root gap
        "coplanar_tol_mm": 0.5,
        "cross_run_tol_deg": 45.0,
        "min_seam_length_mm": 10.0,
    },
}

#: Config keys consumed by substreams 0-2. `twin_key` is computed over exactly these,
#: so two scenes differing only in sensor or density share a twin_key (SCHEMA.md §6.4).
GEOMETRY_KEYS = (
    "joint_type", "seam_shape", "prep", "plate_length_mm", "plate_width_mm",
    "thickness_mm", "dissimilar_thickness_p", "included_angle_deg", "min_overlap_frac",
    "in_plane_yaw",
    "polygon_outlines",
    "class_disjoint",
    "class_boundary_stratum",
    "seam_families",
    "stack_offset_frac", "edge_max_thickness_mm", "stacked_max_beta_deg",
    "edge_equal_width_p",
    "quality_mix", "root_gap_mm", "root_gap_over_range_mm",
)


def load_config(path: str | None = None) -> dict[str, Any]:
    """Load a YAML preset over the defaults."""
    cfg = {k: (v.copy() if isinstance(v, (dict, list)) else v)
           for k, v in DEFAULT_CONFIG.items()}
    if path:
        with open(path) as fh:
            cfg.update(yaml.safe_load(fh) or {})
    return cfg


def geometry_config(cfg: dict[str, Any]) -> dict[str, Any]:
    return {k: cfg[k] for k in GEOMETRY_KEYS if k in cfg}


# --------------------------------------------------------------------------------
# ISO 5817 defect limits — PARAMETERS.md §2.1 and §2.3
# --------------------------------------------------------------------------------

def linear_misalignment_limit(t: float, level: str) -> float:
    """ISO 5817:2023 Table 1, no. 5071. `t` is the SMALLER thickness."""
    coef = {"D": 0.25, "C": 0.15, "B": 0.10}[level]
    if t <= 3.0:
        return coef * t + 0.2
    cap = {"D": 5.0, "C": 4.0, "B": 3.0}[level]
    return min(coef * t, cap)


def root_gap_limit(t: float, throat_mm: float, level: str) -> float:
    """ISO 5817:2023 Table 1, no. 617 — incorrect root gap for FILLET welds.

    Governs T, corner and lap. Butt and edge joints take their gap from ISO 9692-1
    preparation instead (PARAMETERS.md §2.4 scope caveat).
    """
    if t <= 3.0:
        add = {"D": 0.5, "C": 0.3, "B": 0.2}[level]
        return 0.1 * throat_mm + add
    coef, add, cap = {"D": (0.3, 1.0, 4.0),
                      "C": (0.2, 0.5, 3.0),
                      "B": (0.1, 0.5, 2.0)}[level]
    return min(coef * throat_mm + add, cap)


def angular_misalignment_limit(level: str) -> float:
    """Annex B Table B.1 values, mapped to D/C/B by OUR convention (PARAMETERS.md §2.3).

    Table 1 sets no limit on beta at all — its clause numbering skips 3.3 entirely. The
    2 deg / 1 deg values are fatigue classes C 63 / B 90, not quality levels, and D is
    unconstrained so we double C.
    """
    return {"D": 4.0, "C": 2.0, "B": 1.0}[level]


def classify_quality(t_min: float, h: float, beta: float, gap: float,
                     throat_mm: float) -> str:
    """Derive the quality level a realised joint actually satisfies.

    PARAMETERS.md §2.5: per imperfection take the strictest level it satisfies; the
    scene's level is the **weakest** of those — a joint is only as good as its worst
    defect. Returns `"below_D"` when any imperfection exceeds level D.

    This is *derived*, never assumed. Sampling picks a target level and draws defects
    inside it, but what gets stored is the recomputed answer, so a rounding slip or a
    pinned override can never leave a scene mislabelled.
    """
    worst = "B"
    order = {"B": 0, "C": 1, "D": 2, "below_D": 3}
    for value, limit in (
        (h, linear_misalignment_limit),
        (beta, lambda _t, lv: angular_misalignment_limit(lv)),
        (gap, lambda _t, lv: root_gap_limit(t_min, throat_mm, lv)),
    ):
        for level in ("B", "C", "D"):
            if value <= limit(t_min, level) + 1e-12:
                break
        else:
            level = "below_D"
        if order[level] > order[worst]:
            worst = level
    return worst


def _draw(g: np.random.Generator, spec_or_range: Any, fallback: float) -> float:
    """A config entry that is either an explicit [lo, hi] range or None."""
    if spec_or_range is None:
        return fallback
    lo, hi = spec_or_range
    return float(g.uniform(float(lo), float(hi)))


def assert_accessibility_covers_gap(cfg: dict[str, Any]) -> None:
    """`contact_tol_mm` must exceed every root gap the sampler can draw.

    Otherwise the D4 rule rejects the fillets of an out-of-tolerance joint as
    `no_contact`, and the below_D scenes - the ones generated specifically to break the
    baselines - ship with no ground truth at all. Silent, and invisible until someone
    plots error against root gap and finds the top of the range empty.
    """
    max_gap = max(float(cfg["root_gap_mm"][1]), float(cfg["root_gap_over_range_mm"]))
    # below_D draws up to 1.5x the level-D limit, which for thick plate reaches the cap.
    max_gap = max(max_gap, 1.5 * 4.0)
    tol = float(cfg["accessibility"]["contact_tol_mm"])
    if tol < max_gap:
        raise ValueError(
            f"accessibility.contact_tol_mm={tol} is below the maximum root gap "
            f"{max_gap}; below_D scenes would lose their seams")


def sample_joint(cfg: dict[str, Any], streams: Streams
                 ) -> tuple[JointSpec, str, str]:
    """Draw one joint specification. Returns `(spec, quality_level, joint_type)`.

    Draw order within each substream is fixed: appending is safe, reordering is not.
    """
    g0 = streams["joint_config"]
    g1 = streams["defects"]

    choices = cfg["joint_type"]
    joint_type = str(choices) if isinstance(choices, str) else str(g0.choice(choices))

    lo, hi = cfg["plate_length_mm"]
    L_A = float(g0.uniform(lo, hi))
    L_B = float(g0.uniform(lo, hi))
    lo, hi = cfg["plate_width_mm"]
    W_A = float(g0.uniform(lo, hi))
    H_B = float(g0.uniform(lo, hi))
    if joint_type == "edge" and g0.random() < float(cfg["edge_equal_width_p"]):
        H_B = W_A          # both long edges flush -> two edge seams
    lo, hi = cfg["thickness_mm"]
    if joint_type == "edge":
        # ISO 9692-1 Table 1 ref 1.1: raised edges apply at t <= 2 mm. Restricting edge
        # scenes makes the joint type a consequence of the standard, not an arbitrary
        # inclusion - and it lands exactly where §5 predicts radius-PCA has no valid R.
        hi = min(hi, float(cfg["edge_max_thickness_mm"]))
        # Clamp, never invert: if the preset's minimum thickness already exceeds the
        # edge-joint limit, uniform(lo, hi) with lo > hi silently samples the wrong
        # interval instead of erroring.
        lo = min(lo, hi)
    t_A = float(g0.uniform(lo, hi))
    t_B = float(g0.uniform(lo, hi)) if g0.random() < cfg["dissimilar_thickness_p"] else t_A

    # D18: the included angle is sampled about the joint type's OWN nominal. Only fillet
    # joints have a free angle; butt is coplanar (180) and lap/edge parallel (0).
    nominal = JointSpec.NOMINAL_INCLUDED_DEG[joint_type]
    if joint_type in ("T", "corner"):
        alpha = float(g0.uniform(*cfg["included_angle_deg"]))
    else:
        alpha = nominal

    # PARAMETERS.md §2.7 - lap and edge are one topology at different offsets.
    if joint_type == "lap":
        stack_offset = float(g0.uniform(*cfg["stack_offset_frac"]))
    elif joint_type == "edge":
        stack_offset = 0.0
    else:
        stack_offset = None

    levels = list(cfg["quality_mix"])
    probs = np.array([cfg["quality_mix"][k] for k in levels], dtype=float)
    target = str(g1.choice(levels, p=probs / probs.sum()))

    t_min = min(t_A, t_B)
    throat = 0.7 * t_min
    # The root gap answers to TWO standards at once, and they disagree on thin sheet:
    #   * ISO 9692-1 Tables 3-4  - preparation:  b <= 2 mm, independent of thickness
    #   * ISO 5817 clause 617    - imperfection: scales with the throat aA
    # On 2 mm sheet (aA = 1.4) level B allows only 0.34 mm, so a perfectly legal 2 mm
    # preparation gap is far below level D. Drawing from the preparation range alone
    # therefore drives ~37% of scenes to `below_D` and wrecks the stratification
    # balance PARAMETERS.md §2.5 asks for. So the gap is drawn against the 617 limit
    # for the target level, with the 9692-1 cap as an additional ceiling.
    iso9692_cap = float(cfg["root_gap_mm"][1])
    if target == "below_D":
        # Deliberately out of tolerance: real shop-floor fit-up, and it pushes the
        # sweep past where the baselines are expected to fail.
        h = float(g1.uniform(1.0, 2.0) * linear_misalignment_limit(t_min, "D"))
        beta = float(g1.uniform(1.0, 2.0) * angular_misalignment_limit("D"))
        lo = root_gap_limit(t_min, throat, "D")
        gap = float(g1.uniform(lo, max(lo * 1.5, cfg["root_gap_over_range_mm"])))
    else:
        h = float(g1.uniform(0.0, linear_misalignment_limit(t_min, target)))
        beta = float(g1.uniform(0.0, angular_misalignment_limit(target)))
        gap_lo = float(cfg["root_gap_mm"][0])
        # Clamp, never invert: a preset that pins the gap (lo == hi, e.g. the measured
        # reference joint) must reproduce it exactly even when the 617 limit for the
        # target level falls below it. An inverted uniform() silently samples the wrong
        # interval instead of erroring.
        gap_hi = max(gap_lo, min(root_gap_limit(t_min, throat, target), iso9692_cap))
        gap = float(g1.uniform(gap_lo, gap_hi)) if gap_hi > gap_lo else gap_lo

    # Stacked joints are clamped face to face, so relative tilt about the seam axis is
    # physically suppressed - the plates would have to lift off each other. Left at the
    # plate-joint limit, any beta opens the flush edge (a 4 deg tilt lifts a 100 mm plate's
    # far edge by 3.5 mm) and an edge joint stops being an edge joint: 19 of 30 seeds lost
    # their seam entirely. The defect that DOES occur on a stacked joint is poor contact,
    # which the root gap already carries.
    if joint_type in ("lap", "edge"):
        beta = min(beta, float(cfg["stacked_max_beta_deg"]))

    # Explicit overrides, for regression fixtures that must pin a measured geometry.
    h = _draw(g1, cfg.get("linear_misalignment_mm"), h)
    beta = _draw(g1, cfg.get("angular_misalignment_deg"), beta)

    # Longitudinal offset, bounded so the shared run never collapses. Both plates are
    # centred on x = 0, so the concentric overlap is min(L_A, L_B); the offset shifts B
    # along the seam and the clip keeps at least `min_overlap_frac` of that.
    slack = (1.0 - float(cfg["min_overlap_frac"])) * min(L_A, L_B)
    length_offset = float(g0.uniform(-slack, slack)) if slack > 0.0 else 0.0

    # D28 - in-plane yaw, drawn from the (previously unused) `seam_curve` substream so
    # every other stream's draws are untouched: corpora generated with yaw off reproduce
    # bit-identically, and a yaw-enabled regeneration differs ONLY in yaw - a free twin.
    # The range is the FULL CIRCLE (ruled 2026-08-27), uniform over the per-scene
    # feasible set: yaw theta and theta+180 are different configurations (B's body points
    # the other way), and the feasible set need not be one contiguous interval, so the
    # draw is a grid-cell index plus an in-cell jitter rather than a +-bound uniform.
    # Support stays honest per scene, and D31 separately rejects the flush-coincidence
    # angles (a lap at yaw ~ 180 lands B's leading edge exactly on A's welded edge).
    yaw = 0.0
    if bool(cfg.get("in_plane_yaw", False)) and joint_type in ("T", "lap"):
        from .layouts import sample_yaw_deg
        _probe = JointSpec(L_A=L_A, W_A=W_A, t_A=t_A, L_B=L_B, H_B=H_B, t_B=t_B,
                           length_offset_mm=length_offset, root_gap_mm=gap,
                           linear_misalignment_mm=h, angular_misalignment_deg=beta,
                           included_angle_deg=alpha,
                           stack_offset_mm=None if stack_offset is None
                           else stack_offset * H_B)
        yaw = sample_yaw_deg(_probe, joint_type, streams["seam_curve"])

    # D28 - the other half: polygon outlines. Part B gets one for EVERY joint type: its
    # seam-bearing edge (bottom for T/corner, leading edge for lap/butt, flush edge for
    # edge) is pinned straight and full-length by the layout mapping, and every other
    # boundary splays. Yaw alone cannot do this for T/lap - B co-rotates with the seam,
    # so a rectangular B keeps its top edge systematically parallel and its end edges
    # systematically perpendicular, which the corpus gate measured as a persistent 0/90
    # spike. Part A is outlined only for the edge-sharing joints (corner, butt, edge):
    # for T/lap the yaw draw already decorrelates A's edges from the seam, and the yaw
    # support bound is defined on A's rectangle. Draw order within `seam_curve` is fixed
    # (yaw, then A's outline, then B's), so each mechanism stays a free twin.
    outline_A = outline_B = None
    shape_A = shape_B = None
    if bool(cfg.get("polygon_outlines", False)):
        from .layouts import sample_outline
        g_sc = streams["seam_curve"]
        if joint_type in ("corner", "butt", "edge"):
            shape_A, outline_A = sample_outline(g_sc, L_A, W_A)
        shape_B, outline_B = sample_outline(g_sc, L_B, H_B)

    spec = JointSpec(
        L_A=L_A, W_A=W_A, t_A=t_A,
        L_B=L_B, H_B=H_B, t_B=t_B,
        length_offset_mm=length_offset,
        root_gap_mm=gap,
        linear_misalignment_mm=h,
        angular_misalignment_deg=beta,
        included_angle_deg=alpha,
        stack_offset_mm=None if stack_offset is None else stack_offset * H_B,
        in_plane_yaw_deg=yaw,
        outline_A=outline_A, outline_B=outline_B,
        outline_shape_A=shape_A, outline_shape_B=shape_B,
    )
    # Store what the joint ACTUALLY satisfies, not what was aimed for (PARAMETERS §2.5).
    return spec, classify_quality(t_min, h, beta, gap, throat), joint_type
