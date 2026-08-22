"""Seam-detection baselines, evaluated against constructed ground truth (Phase 4).

These are **not** contributions. They are the comparison table, and the reason the
generator exists is to put a number under them instead of an RViz screenshot.

    radius_pca   `ours` - the admittance-control repo's method, migrated. One entry in a
                 seven-method comparison (`dataset_plan.md` §4); the other six are
                 reimplementations from the literature. `ours` is NOT claimed as a novel
                 extractor - the contribution is the dataset and the comparison
    lit_ransac   `lit-ransac` - Yi et al., Automation in Construction 183 (2026) 106792.
                 Improved multi-plane RANSAC, then plane intersections for the path.
                 Randomised: every number it gives is a distribution over `seed`
    lit_regiongrow  `lit-regiongrow` - Wei et al., arXiv:2408.10710, 2024. Curvature-seeded
                 region growing, then the edge points that span two grown surfaces. Uses
                 the SAME feature as `ours` (lambda_0 / sum lambda), so the k-NN vs radius
                 substitution and the oracle-free two-surface test are both measurable
    lit_lobb     `lit-lobb` - Zhang et al., RCIM 95 (2025) 102987, with the LOBB descriptor
                 of IEEE T-ASE 22 (2025) 75. Bounding-box flatness, tanh activation,
                 binary K-means, polynomial fit. Its coarse stage is K-Net **component**
                 masks - i.e. `object_id`, the same input `ours` calls an oracle
    lit_ppf      `lit-ppf` - Wang et al., Sci. Rep. 14 (2024) 21137. PPF-coplanarity
                 planes, orthogonal-pair Hough voting, farthest-pair corners. As
                 published it is DETERMINISTIC (the plan's "randomised" assumption came
                 from its RANSAC-alternative framing), and it is the first implemented
                 method that consumes normals - the L2 rung becomes real here
    lit_pcaslice `lit-pcaslice` - Wang et al., Welding in the World (2026). PCA-adaptive
                 slicing, per-slice geometric centres, cubic B-spline path. Per-INSTANCE
                 coarse stage (YOLO11 boxes each weld separately) - the single-strip
                 assumption is structural, and the harness supplies it as such
    lit_modelreg `lit-modelreg` - Fang & Tian, RCIM 89 (2024) 102772. A basic model
                 (CAD-by-construction from scene.json) registered onto the scan by CPD;
                 the seam is TRANSFERRED, never detected. Every number is L0-with-CAD -
                 the top rung of the oracle ladder, held by a real published method
    metrics      Chamfer, precision/recall, lateral error, band width, and RMSE / ME on a
                 matched path - the literature's own metric, the only one a paper's
                 reported accuracy can be checked against
    harness      the repeat harness: every method x scene x seed as one dataframe, each
                 method given ITS OWN paper's coarse stage at L0, the noise axis as a
                 multiplier on the derived sigma_z, and a fake-oracle predictor the whole
                 thing is validated against before any real method's number flows through

Everything here is **millimetres**, like the rest of `weld_generator` (SCHEMA.md §1). The
ROS node this was migrated from works in metres, so every length constant changed by 1000x
in the move; that is the single most likely way to get a plausible-looking wrong answer out
of this code, and it is why the parameter names all carry `_mm`.
"""

from .dataset import (balanced_corpus, cloud_for, ground_truth, iter_scenes,
                      load_scene, scene_dirs, scene_facts)
from .metrics import (band_width_mm, chamfer_mm, densify, emd_mm, evaluate,
                      evaluate_band, match_seams, matched_path_errors,
                      lateral_error_mm, path_error_mm, polyline_length_mm,
                      precision_recall_f1)
from .harness import (REGISTRY, MethodSpec, PreparedScene, fake_oracle, prepare,
                      run_matrix, spread)
from .lit_lobb import (LobbResult, activate, detect as lit_lobb_detect,
                       kmeans_1d_binary, lobb_features, part_labels_oracle)
from .lit_modelreg import ModelRegResult, build_model, cpd_similarity
from .lit_modelreg import detect as lit_modelreg_detect
from .lit_pcaslice import PCASliceResult, bspline_path
from .lit_pcaslice import detect as lit_pcaslice_detect
from .lit_pcaslice import pca_centerline
from .lit_ppf import (PPFPlane, PPFResult, detect as lit_ppf_detect, opp_vote, ppf,
                      ppf_planes)
from .lit_ransac import (LitRansacResult, LitRansacSeam, Plane,
                         detect as lit_ransac_detect, multi_plane_fit,
                         seam_region_oracle, surface_intersection_crop,
                         surface_labels_oracle)
from .lit_regiongrow import (RegionGrowResult, detect as lit_regiongrow_detect,
                             local_pca, region_grow, two_surface_edges)
from .radius_pca import (RadiusPCAResult, detect, directional_components, local_tangent,
                         surface_variation, validity_window_mm)

__all__ = [
    "surface_variation", "detect", "RadiusPCAResult", "validity_window_mm",
    "directional_components", "local_tangent",
    "chamfer_mm", "precision_recall_f1", "lateral_error_mm", "band_width_mm",
    "densify", "emd_mm", "evaluate", "evaluate_band",
    "path_error_mm", "match_seams", "matched_path_errors", "polyline_length_mm",
    "load_scene", "iter_scenes", "scene_dirs", "cloud_for", "ground_truth",
    "scene_facts", "balanced_corpus",
    "lit_ransac_detect", "LitRansacResult", "LitRansacSeam", "Plane", "multi_plane_fit",
    "seam_region_oracle", "surface_labels_oracle", "surface_intersection_crop",
    "lit_regiongrow_detect", "RegionGrowResult", "local_pca", "region_grow",
    "two_surface_edges",
    "lit_ppf_detect", "PPFResult", "PPFPlane", "ppf", "ppf_planes", "opp_vote",
    "lit_pcaslice_detect", "PCASliceResult", "pca_centerline", "bspline_path",
    "lit_modelreg_detect", "ModelRegResult", "build_model", "cpd_similarity",
    "lit_lobb_detect", "LobbResult", "lobb_features", "activate", "kmeans_1d_binary",
    "part_labels_oracle",
    "run_matrix", "prepare", "spread", "REGISTRY", "MethodSpec", "PreparedScene",
    "fake_oracle",
]
