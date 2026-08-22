"""The Phase 4 repeat harness — every method, every seed, one dataframe.

Built as the plan orders it: *"Harness first — matching, metrics, per-scene dataframe —
validated against a fake oracle predictor (ground truth plus noise) before any real method
output flows through it."* And built now because it stopped being a protocol nicety:
`lit-ransac` swings F1 0,00-0,94 on one fixed scene, varying nothing but its RANSAC seed, so
any single-draw number for a randomised method is noise. `tests/test_harness.py` is the
fake-oracle validation; the box plots the advisor asked for are `notebooks/07_repeats.ipynb`.

What a row is
-------------
One `(method, scene, seed, arm, view, noise_scale)` cell. Deterministic methods still get
`verify_seeds` runs (default 3), because *"the deterministic methods show zero spread"* is a
reportable finding only if the spread was measured, not assumed.

The oracle ladder (plan item 3: write down every input each method consumes)
----------------------------------------------------------------------------
`L0` gives each method **its own paper's** coarse stage — not a shared mask. Conflating the
three stages already produced one wrong conclusion (`dataset_plan.md` §4, the regiongrow
ladder), so the harness makes the choice explicit and per-method:

    method          L0 oracle (its published stage)                  source array
    ours            object_id for the cross-object gate              cloud.npz:object_id
    lit-ransac      ~40 mm weld-seam band (their PointNet++)         truth seams (band)
    lit-regiongrow  per-surface masks + derived crop (their FastSAM) cloud.npz:face_id
    lit-lobb        per-component masks (their K-Net)                cloud.npz:object_id
    lit-ppf         weld-box crop (their Faster R-CNN)               truth seams (band)
    lit-pcaslice    PER-INSTANCE weld masks (their YOLO11+DeepLab)   truth seams, one band
                                                                     per seam
    lit-modelreg    the basic model itself (CAD-by-construction)     scene.json + truth

`lit-ppf` additionally consumes NORMALS - the first method here that does. Its faithful
condition estimates them from the cloud (their PCL pipeline); `normals="exact"` through
`method_kw` supplies the generator's analytic normals instead, which is the normal-oracle
rung, and the delta between the two arms prices normal estimation.

`L1` withholds the stage and changes nothing else. The full L0-L3 ladder adds estimated
normals and noise; here `noise_scale` is the noise axis and normals are not yet estimated
(no implemented method consumes them — revisit at `lit-ppf`, which does).

The noise axis
--------------
`noise_scale = k` multiplies the derived sensor sigmas (`subpixel_px` and
`lateral_sigma_px`) by `k` before `weldgen.noise.apply` — so "200% noise" means **2x the
sigma_z the sensor model derives**, never a fraction of range; the plan is explicit that
the percentage must be defined this way. `0` is the clean cloud; `1` is the stored profile.
The realisation stays deterministic in `noise_model.seed`, so a sweep is reproducible.
"""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from .dataset import cloud_for, ground_truth, load_scene, scene_facts
from .metrics import evaluate, evaluate_band, matched_path_errors


# --------------------------------------------------------------------------------
# scenes, prepared once — oracles are per-method but the cloud is shared
# --------------------------------------------------------------------------------

@dataclass
class PreparedScene:
    """One scene, loaded once, with every per-method oracle computed once.

    Seeds vary hundreds of times per scene; nothing seed-independent may be recomputed
    inside the seed loop or a corpus sweep stops finishing. Oracles are built lazily on
    first request and cached.
    """

    path: Path
    scene: dict
    arrays: dict
    facts: dict
    gt: list
    _cache: dict = field(default_factory=dict)

    def cloud(self, view: str = "full", noise_scale: float = 0.0) -> dict:
        key = ("cloud", view, float(noise_scale))
        if key in self._cache:
            return self._cache[key]
        if noise_scale == 0.0:
            c = cloud_for(self.scene, self.arrays, view=view, noisy=False)
        else:
            from weldgen.noise import apply as apply_noise
            nm = dict(self.scene["noise_model"])
            nm["subpixel_px"] = float(nm["subpixel_px"]) * float(noise_scale)
            nm["lateral_sigma_px"] = float(nm["lateral_sigma_px"]) * float(noise_scale)
            base = cloud_for(self.scene, self.arrays, view=view, noisy=False)
            xyz, valid = apply_noise(
                base["xyz"], base["normals"],
                np.array(self.scene["camera"]["T_world_cam"]), nm)
            c = {**base, "xyz": xyz[valid], "normals": base["normals"][valid],
                 "object_id": base["object_id"][valid], "valid": valid[valid],
                 "clean_xyz": base["xyz"][valid]}
        self._cache[key] = c
        return c

    def oracle(self, name: str, view: str, noise_scale: float):
        key = ("oracle", name, view, float(noise_scale))
        if key in self._cache:
            return self._cache[key]
        c = self.cloud(view, noise_scale)
        if name == "band":                              # Yi et al.: weld-seam band
            from .lit_ransac import seam_region_oracle
            val = seam_region_oracle(c["xyz"], self.gt, end_margin_mm=0.0)
        elif name == "surfaces":                        # Wei et al.: per-surface masks + crop
            from .lit_ransac import surface_intersection_crop, surface_labels_oracle
            lab = surface_labels_oracle(self.arrays["cloud.npz:face_id"])
            if len(lab) != len(c["xyz"]):               # single view / noisy subset
                from scipy.spatial import cKDTree
                full = cloud_for(self.scene, self.arrays, view="full", noisy=False)["xyz"]
                _, idx = cKDTree(full).query(c["xyz"], k=1, workers=-1)
                lab = lab[idx]
            val = (surface_intersection_crop(c["xyz"], lab), lab)
        elif name == "objects":                         # Zhang et al. K-Net / `ours` gate
            val = c["object_id"]
        else:
            raise ValueError(f"unknown oracle {name!r}")
        self._cache[key] = val
        return val


def prepare(scene_dirs, primary_only: bool = True) -> list[PreparedScene]:
    out = []
    for d in scene_dirs:
        scene, arrays = load_scene(d)
        gt = ground_truth(scene, arrays, primary_only=primary_only)
        if not gt:
            continue
        out.append(PreparedScene(Path(d), scene, arrays, scene_facts(scene), gt))
    return out


# --------------------------------------------------------------------------------
# the registry
# --------------------------------------------------------------------------------

@dataclass
class MethodSpec:
    """One entry in the comparison, and what the harness must know to score it honestly.

    `randomised` decides the repeat count: a randomised method gets the full seed list, a
    deterministic one gets `verify_seeds` runs so its zero spread is measured rather than
    assumed. `output` decides the scoring path — `ours` returns a **band** (its line fit was
    removed for cause) and pretending it returns polylines would score its band's width as
    if it were a curve.
    """

    name: str
    run: Callable[..., Any]               # (prep, seed, oracle: bool, view, noise_scale)
    randomised: bool
    output: str = "polylines"             # "polylines" | "band"
    oracle_name: str | None = None        # which coarse stage L0 supplies


def _run_ours(prep: PreparedScene, seed: int, oracle: bool, view: str, ns: float):
    from .radius_pca import detect, validity_window_mm
    rho = 0.5
    c = prep.cloud(view, ns)
    lo, hi = validity_window_mm(prep.facts["root_gap_mm"], 1.0 / np.sqrt(rho),
                                prep.facts["t_min_mm"])
    R = 0.5 * (lo + hi) if lo < hi else lo
    r = detect(c["xyz"], R, object_id=prep.oracle("objects", view, ns) if oracle else None,
               cross_object=oracle, prefilter_density_per_mm2=rho)
    return r.band, {"n_clusters": r.n_clusters, "window_open": lo < hi}


def _run_lit_ransac(prep: PreparedScene, seed: int, oracle: bool, view: str, ns: float):
    from .lit_ransac import detect
    c = prep.cloud(view, ns)
    mask = prep.oracle("band", view, ns) if oracle else None
    r = detect(c["xyz"], seed=seed, prefilter_density_per_mm2=1.0, segmentation_mask=mask)
    return r.polylines, {"n_planes": len(r.planes)}


def _run_lit_regiongrow(prep: PreparedScene, seed: int, oracle: bool, view: str, ns: float):
    from .lit_regiongrow import detect
    c = prep.cloud(view, ns)
    if oracle:
        crop, lab = prep.oracle("surfaces", view, ns)
        r = detect(c["xyz"], voxel_mm=1.5, edge_radius_mm=6.0, link_mm=3.0,
                   segmentation_mask=crop, region_labels=lab[crop])
    else:
        r = detect(c["xyz"], voxel_mm=1.5, edge_radius_mm=6.0, link_mm=3.0)
    return r.polylines, {"n_regions": r.n_regions}


def _run_lit_ppf(prep: PreparedScene, seed: int, oracle: bool, view: str, ns: float,
                 normals: str = "estimate"):
    from .lit_ppf import detect
    c = prep.cloud(view, ns)
    r = detect(c["xyz"], normals=normals,
               normals_xyz=c["normals"] if normals == "exact" else None,
               segmentation_mask=prep.oracle("band", view, ns) if oracle else None)
    return r.polylines, {"n_planes": len(r.planes), "normals": normals}


def _run_lit_lobb(prep: PreparedScene, seed: int, oracle: bool, view: str, ns: float):
    from .lit_lobb import detect
    c = prep.cloud(view, ns)
    r = detect(c["xyz"], object_id=prep.oracle("objects", view, ns) if oracle else None,
               voxel_mm=1.0)
    return r.polylines, {"n_crease": int(r.crease.sum())}


def _length(poly) -> float:
    poly = np.asarray(poly, dtype=float)
    return float(np.linalg.norm(np.diff(poly, axis=0), axis=1).sum())


def _coarsen(poly: np.ndarray, step_mm: float) -> np.ndarray:
    """Resample a polyline at `step_mm` by arclength interpolation, endpoints kept."""
    poly = np.asarray(poly, dtype=float)
    seg = np.linalg.norm(np.diff(poly, axis=0), axis=1)
    cum = np.r_[0.0, np.cumsum(seg)]
    if cum[-1] < 1e-9:
        return poly[[0, -1]]
    t = np.linspace(0.0, cum[-1], max(2, int(cum[-1] // step_mm) + 1))
    return np.column_stack([np.interp(t, cum, poly[:, k]) for k in range(3)])


def _run_lit_pcaslice(prep: PreparedScene, seed: int, oracle: bool, view: str, ns: float):
    from .lit_pcaslice import detect
    from .lit_ransac import seam_region_oracle
    c = prep.cloud(view, ns)
    masks = None
    if oracle:
        # One band PER TRUTH SEAM - their YOLO boxes each weld instance separately, and
        # the per-slice geometric centre cannot survive two seams in one strip.
        masks = [seam_region_oracle(c["xyz"], [g], end_margin_mm=0.0) for g in prep.gt]
    r = detect(c["xyz"], instance_masks=masks, seed=seed)
    return r.polylines, {"n_instances": r.params["n_instances"]}


def _run_lit_modelreg(prep: PreparedScene, seed: int, oracle: bool, view: str, ns: float,
                      mode: str = "nonrigid", target_features: str = "oracle",
                      init: str = "near"):
    # `oracle` (the harness flag) is ignored: the model IS the oracle, constitutively -
    # there is no L1 arm for a method whose seam arrives from CAD. Its own two arms are
    # `target_features` ("oracle" = the paper's model-derived target features; "dense" =
    # raw surfaces, where the slide failures live) and `init` ("near" = the paper's
    # roughly-positioned envelope; "global" = outside it, where near-symmetric assemblies
    # register onto their symmetric counterpart).
    from .lit_modelreg import detect
    c = prep.cloud(view, ns)
    r = detect(c["xyz"], prep.scene, prep.gt, mode=mode, target_features=target_features,
               init=init)
    return r.polylines, {"reg_scale": r.s, "mode": mode, "target_features": target_features}


def fake_oracle(prep: PreparedScene, seed: int, oracle: bool, view: str, ns: float,
                sigma_mm: float = 0.3, p_phantom: float = 0.0, p_miss: float = 0.0):
    """Ground truth plus known noise — the predictor the harness is validated against.

    The whole point: every property of a harness row is *known in advance* here. Jitter of
    `sigma_mm` must come back as RMSE ~= sigma (the expected |N(0, s)| distance of a point
    to the line it was jittered off); a phantom must cost precision and never steal a match
    from a real seam; a missed seam must appear as an unmatched truth row. If any of that
    fails, the harness is broken and no real method's number can be trusted through it.
    """
    rng = np.random.default_rng(seed)
    out = []
    for g in prep.gt:
        if p_miss and rng.random() < p_miss:
            continue
        # Resample to a ~4 mm vertex spacing before jittering. The stored truth carries
        # ~0,1 mm vertex spacing, and jittering THAT produces a prediction whose densified
        # point count is set by the storage density rather than by anything a method would
        # return - the first phantom test failed for exactly that reason: thousands of real
        # points against a 61-point phantom moved precision by 1%, not 14%. NB
        # `metrics._sample_polyline` cannot do this: it floors at one sample per SEGMENT
        # (correct for a metric that must never under-sample, useless for coarsening), so
        # the resample is by arclength interpolation here.
        cg = _coarsen(np.asarray(g, dtype=float), 4.0)
        out.append(cg + rng.normal(0.0, sigma_mm, cg.shape))
    if p_phantom and rng.random() < p_phantom:
        lo, hi = np.min([g.min(0) for g in prep.gt], axis=0), \
                 np.max([g.max(0) for g in prep.gt], axis=0)
        a = lo + rng.random(3) * (hi - lo) + np.array([0, 0, 40.0])
        out.append(np.stack([a, a + np.array([60.0, 0, 0])]))
    return out, {}


REGISTRY: dict[str, MethodSpec] = {
    "ours": MethodSpec("ours", _run_ours, randomised=False, output="band",
                       oracle_name="objects"),
    "lit-ransac": MethodSpec("lit-ransac", _run_lit_ransac, randomised=True,
                             oracle_name="band"),
    "lit-regiongrow": MethodSpec("lit-regiongrow", _run_lit_regiongrow, randomised=False,
                                 oracle_name="surfaces"),
    "lit-lobb": MethodSpec("lit-lobb", _run_lit_lobb, randomised=False,
                           oracle_name="objects"),
    # DETERMINISTIC as published - grid sampling, Hough voting, DBSCAN, farthest-pair
    # corners; no stage draws a random number. The plan grouped it with lit-ransac as
    # randomised on its RANSAC-alternative framing; reading the paper corrects that.
    "lit-ppf": MethodSpec("lit-ppf", _run_lit_ppf, randomised=False, oracle_name="band"),
    # Path pipeline deterministic; MSAC (seeded) feeds only the torch posture, which the
    # Phase 4 metrics do not score. Flip the flag if pose metrics land.
    "lit-pcaslice": MethodSpec("lit-pcaslice", _run_lit_pcaslice, randomised=False,
                               oracle_name="band"),
    # Deterministic EM from a PCA init. `oracle_name="model"` is a statement, not a mask:
    # the method is constitutively L0-with-CAD and has no L1 arm.
    "lit-modelreg": MethodSpec("lit-modelreg", _run_lit_modelreg, randomised=False,
                               oracle_name="model"),
    "fake-oracle": MethodSpec("fake-oracle", fake_oracle, randomised=True,
                              oracle_name=None),
}


# --------------------------------------------------------------------------------
# the runner
# --------------------------------------------------------------------------------

_NAN_PATH = {k: float("nan") for k in ("rmse_med", "rmse_max", "me_med", "me_max")}


def _score(spec: MethodSpec, pred, gt, tol_mm: float) -> dict[str, Any]:
    if spec.output == "band":
        row = evaluate_band(np.asarray(pred), gt, tol_mm=tol_mm)
        row.update(_NAN_PATH, n_pred_seams=float("nan"), seam_recall=float("nan"))
        return row
    row = evaluate(pred, gt, tol_mm=tol_mm)
    errs = [e for e in matched_path_errors(pred, gt) if e["matched"]]
    row.update({
        "rmse_med": float(np.median([e["rmse"] for e in errs])) if errs else float("nan"),
        "rmse_max": max((e["rmse"] for e in errs), default=float("nan")),
        "me_med": float(np.median([e["me"] for e in errs])) if errs else float("nan"),
        "me_max": max((e["me"] for e in errs), default=float("nan")),
        "seam_recall": len(errs) / max(len(gt), 1),
    })
    return row


def run_matrix(prepared: list[PreparedScene], methods=None, seeds=range(30),
               verify_seeds: int = 3, oracle: bool = True, view: str = "full",
               noise_scale: float = 0.0, tol_mm: float = 3.0, method_kw=None,
               progress: bool = False):
    """Every method x scene x seed, as a long dataframe ready for box plots.

    Randomised methods get every seed in `seeds`; deterministic ones get the first
    `verify_seeds`, which is the minimum that turns "zero spread" from an assumption into a
    measurement. `method_kw` maps method name -> extra kwargs for its runner (used by the
    fake oracle's `sigma_mm` / `p_phantom` / `p_miss`).
    """
    import pandas as pd

    methods = [REGISTRY[m] if isinstance(m, str) else m
               for m in (methods or ["ours", "lit-ransac", "lit-regiongrow", "lit-lobb"])]
    method_kw = method_kw or {}
    seeds = list(seeds)
    rows = []
    for si, prep in enumerate(prepared):
        for spec in methods:
            use = seeds if spec.randomised else seeds[:max(1, verify_seeds)]
            for seed in use:
                t0 = time.time()
                pred, aux = spec.run(prep, seed, oracle, view, noise_scale,
                                     **method_kw.get(spec.name, {}))
                row = {"method": spec.name, "scene_id": prep.facts["scene_id"],
                       "joint_type": prep.facts["joint_type"], "seed": seed,
                       "arm": "L0" if oracle else "L1", "view": view,
                       "noise_scale": float(noise_scale),
                       "randomised": spec.randomised, "n_gt": len(prep.gt),
                       "t_min_mm": prep.facts["t_min_mm"],
                       "root_gap_mm": prep.facts["root_gap_mm"],
                       "sec": time.time() - t0, **aux}
                row.update(_score(spec, pred, gt=prep.gt, tol_mm=tol_mm))
                rows.append(row)
        if progress and (si + 1) % 10 == 0:
            print(f"  {si + 1}/{len(prepared)} scenes", flush=True)
    return pd.DataFrame(rows)


def spread(df, metric: str = "f1"):
    """Per method x scene spread over seeds — the box-plot table, and the zero-spread check.

    A deterministic method must show `spread == 0` on every scene; that row in the output
    is the finding the plan asks to state (*"method reproducibility is a property the
    generator can measure and the field does not report"*), not a wasted computation.
    """
    g = df.groupby(["method", "scene_id"])[metric]
    out = g.agg(["min", "median", "max", "std", "count"])
    out["spread"] = out["max"] - out["min"]
    return out.reset_index()
