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
    "joint_type": "T",
    "seam_shape": "line",
    "prep": "square",
    "plate_length_mm": [80.0, 400.0],
    "plate_width_mm": [50.0, 250.0],
    "thickness_mm": [1.0, 12.0],
    "dissimilar_thickness_p": 0.30,
    # ISO 9692-1 Tables 3-4: 60 <= alpha <= 120 for square-preparation fillets.
    "included_angle_deg": [60.0, 120.0],
    # --- defects, substream 1 (PARAMETERS.md §2) ---------------------------------
    "quality_mix": {"B": 0.25, "C": 0.25, "D": 0.25, "below_D": 0.25},
    # ISO 9692-1 Tables 3-4 cap the fillet gap at b <= 2 mm; the over-range tail to
    # 3 mm is [ours] and is what generates `below_D` and breaks radius-PCA on purpose.
    "root_gap_mm": [0.0, 2.0],
    "root_gap_over_range_mm": 3.0,
    # --- placement, substream 3 --------------------------------------------------
    "fixture_present": False,        # Phase 1 is fixture-free (D12)
    "assembly_translation_mm": 150.0,
    # --- sampling, substream 4 ---------------------------------------------------
    "density_per_mm2": [0.25, 4.0],
    # --- camera / sensor, substreams 5-6 -----------------------------------------
    "sensor_profiles": ["d435i", "stereo_good", "stereo_poor"],
    "standoff_mm": [300.0, 1200.0],
    "elevation_deg": [15.0, 85.0],
    # --- accessibility, recorded so every reject_reason is reproducible ----------
    "accessibility": {
        "torch_clearance": {"half_angle_deg": 30.0, "standoff_mm": 15.0},
        "dihedral_min_deg": 30.0,
        "dihedral_max_deg": 170.0,
        "contact_tol_mm": 3.0,
        "min_seam_length_mm": 10.0,
    },
}

#: Config keys consumed by substreams 0-2. `twin_key` is computed over exactly these,
#: so two scenes differing only in sensor or density share a twin_key (SCHEMA.md §6.4).
GEOMETRY_KEYS = (
    "joint_type", "seam_shape", "prep", "plate_length_mm", "plate_width_mm",
    "thickness_mm", "dissimilar_thickness_p", "included_angle_deg",
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


def sample_joint(cfg: dict[str, Any], streams: Streams) -> tuple[JointSpec, str]:
    """Draw one joint specification. Returns `(spec, quality_level)`.

    Draw order within each substream is fixed: appending is safe, reordering is not.
    """
    g0 = streams["joint_config"]
    g1 = streams["defects"]

    lo, hi = cfg["plate_length_mm"]
    L = float(g0.uniform(lo, hi))
    lo, hi = cfg["plate_width_mm"]
    W_A = float(g0.uniform(lo, hi))
    H_B = float(g0.uniform(lo, hi))
    lo, hi = cfg["thickness_mm"]
    t_A = float(g0.uniform(lo, hi))
    t_B = float(g0.uniform(lo, hi)) if g0.random() < cfg["dissimilar_thickness_p"] else t_A
    lo, hi = cfg["included_angle_deg"]
    alpha = float(g0.uniform(lo, hi))

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
        gap = float(g1.uniform(
            cfg["root_gap_mm"][0],
            min(root_gap_limit(t_min, throat, target), iso9692_cap)))

    # Explicit overrides, for regression fixtures that must pin a measured geometry.
    h = _draw(g1, cfg.get("linear_misalignment_mm"), h)
    beta = _draw(g1, cfg.get("angular_misalignment_deg"), beta)

    spec = JointSpec(
        L_A=L, W_A=W_A, t_A=t_A,
        L_B=L, H_B=H_B, t_B=t_B,
        root_gap_mm=gap,
        linear_misalignment_mm=h,
        angular_misalignment_deg=beta,
        included_angle_deg=alpha,
    )
    # Store what the joint ACTUALLY satisfies, not what was aimed for (PARAMETERS §2.5).
    return spec, classify_quality(t_min, h, beta, gap, throat)
