"""Seam-detection baselines, evaluated against constructed ground truth (Phase 4).

These are **not** contributions. They are the comparison table, and the reason the
generator exists is to put a number under them instead of an RViz screenshot.

    radius_pca   Baseline A (OURS) - the admittance-control repo's method, migrated
                 and upgraded. A is ours; B, C and D are taken from the literature
                 for comparison, so the table compares methodologies
    metrics      Chamfer, precision/recall, lateral error, band width

Everything here is **millimetres**, like the rest of `weld_generator` (SCHEMA.md §1). The
ROS node this was migrated from works in metres, so every length constant changed by 1000x
in the move; that is the single most likely way to get a plausible-looking wrong answer out
of this code, and it is why the parameter names all carry `_mm`.
"""

from .dataset import (cloud_for, ground_truth, iter_scenes, load_scene,
                      scene_dirs, scene_facts)
from .metrics import (band_width_mm, chamfer_mm, densify, evaluate,
                      evaluate_band,
                      lateral_error_mm, precision_recall_f1)
from .radius_pca import RadiusPCAResult, detect, surface_variation, validity_window_mm

__all__ = [
    "surface_variation", "detect", "RadiusPCAResult", "validity_window_mm",
    "chamfer_mm", "precision_recall_f1", "lateral_error_mm", "band_width_mm",
    "densify", "evaluate", "evaluate_band",
    "load_scene", "iter_scenes", "scene_dirs", "cloud_for", "ground_truth",
    "scene_facts",
]
