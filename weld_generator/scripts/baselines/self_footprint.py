"""`self-footprint` — the footprint / rolling-ball extractor of
`Self_ideas/test1_ismail_hoca/weld_seam.py`, run on the tier-1 corpus.

NOT one of the seven literature reimplementations and not in `REGISTRY` (the paper's
comparison is closed). It is a candidate for the project's own extractor, scored by the
same harness, metrics and strata so its numbers sit next to the seven without a caveat.

What the method consumes
------------------------
Two scans, which is its whole idea:

    --base      part A ALONE, scanned before B is placed
    --combined  the assembly A + B in weld position

The assembly scan is the harness cloud for the condition (`full_exterior` / `single`),
exactly what every other method gets. The base scan does not exist in a scene on disk, so
it is SYNTHESISED here from the generator's own primitive, the honest way a second scan
would arise:

* an independent area-uniform sample of part A (`object_id == 0`, the base part in every
  joint type) at the scene's own density, through the tier-1 sampler (`sample_slab_surface`) - a different draw
  from the assembly's, so the two clouds never share a point (sharing points would make
  step 3, the A/B segmentation, trivially exact);
* visibility recomputed with A as the ONLY occluder: `full_exterior` -> the exterior
  scan of A alone (the faces B later covers are visible in it, as in a real pre-scan),
  `single` -> the same camera, A alone (`base_view="complete"` keeps the whole surface,
  which is what the method's own `make_synthetic.py` writes);
* the same stereo noise model and multiplier as the assembly cloud when `noise_scale > 0`.

No oracle is consumed: the base scan is a physical input of the method, not a coarse
stage standing in for a learned network, so the harness `oracle` flag is ignored.

The one process parameter is the expected fit-up gap (`--gap`). `gap="wps"` passes the
scene's `root_gap_mm` - the gap a welding procedure specification states, i.e. the
method's declared input; `gap="zero"` is the script's CLI default and is kept as a rung.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[2]
_SELF = _ROOT / "Self_ideas" / "test1_ismail_hoca"
for p in (str(_ROOT), str(_SELF)):
    if p not in sys.path:
        sys.path.insert(0, p)

import weld_seam  # noqa: E402  (the method itself, unmodified apart from extract())

BASE_SEED_OFFSET = 7_919        # base-scan draw != assembly draw (any fixed offset)


def method_args(**overrides):
    """A `weld_seam.build_parser()` namespace with the script's defaults, plus overrides
    (underscored names: `ball_radius=`, `grad_min=`, ...)."""
    a = weld_seam.build_parser().parse_args(["--base", "-", "--combined", "-"])
    a.no_plots, a.verbose = True, False
    for k, v in overrides.items():
        if not hasattr(a, k):
            raise TypeError(f"weld_seam has no parameter {k!r}")
        setattr(a, k, v)
    return a


def base_scan(prep, view: str, noise_scale: float = 0.0, base_view: str = "match",
              seed: int = 0) -> np.ndarray:
    """The pre-scan of part A alone, under the assembly cloud's condition."""
    from weldgen.geom import from_object
    from weldgen.sampling import sample_slab_surface
    from weldgen.visibility import exterior_scan_subsampled, visible_mask

    key = ("self-footprint-base", view, float(noise_scale), base_view, int(seed))
    if key in prep._cache:
        return prep._cache[key]
    sc = prep.scene
    entry = next(o for o in sc["objects"] if int(o["object_id"]) == 0)
    part = from_object(entry)
    rng = np.random.default_rng(int(sc["seed"]) + BASE_SEED_OFFSET + int(seed))
    cl = sample_slab_surface(part, float(sc["cloud"]["density_per_mm2"]), rng, face_id_base=0)
    xyz, nrm = cl["xyz"].astype(float), cl["normals"].astype(float)

    cam = sc["camera"]
    T_cam = np.asarray(cam["T_world_cam"], dtype=float)
    if base_view == "match" and view == "single":
        m = visible_mask(xyz, nrm, [part], T_cam, np.asarray(cam["K"], dtype=float),
                         int(cam["width"]), int(cam["height"]),
                         float(sc["noise_model"]["min_z_mm"]))
    elif base_view == "match" and view == "full_exterior":
        m = exterior_scan_subsampled(xyz, nrm, [part])
    elif base_view == "complete" or view == "full":
        m = np.ones(len(xyz), dtype=bool)
    else:
        raise ValueError(f"base_view={base_view!r} view={view!r}")
    xyz, nrm = xyz[m], nrm[m]

    if noise_scale:
        from weldgen.noise import apply as apply_noise
        nm = dict(sc["noise_model"])
        nm["subpixel_px"] = float(nm["subpixel_px"]) * float(noise_scale)
        nm["lateral_sigma_px"] = float(nm["lateral_sigma_px"]) * float(noise_scale)
        nm["seed"] = int(nm["seed"]) + BASE_SEED_OFFSET
        xyz, valid = apply_noise(xyz, nrm, T_cam, nm)
        xyz = xyz[valid]
    prep._cache[key] = xyz
    return xyz


def detect(base_xyz: np.ndarray, combined_xyz: np.ndarray, **params):
    """Run `weld_seam.extract`; returns (polylines, extract-result dict)."""
    r = weld_seam.extract(np.asarray(base_xyz, float), np.asarray(combined_xyz, float),
                          method_args(**params))
    return [np.asarray(p["points"], float) for p in r["paths"]], r


def run(prep, seed: int, oracle: bool, view: str, ns: float, gap: str | float = "wps",
        base_view: str = "match", **params):
    """Harness runner, signature of `harness.MethodSpec.run`."""
    c = prep.cloud(view, ns)
    A0 = base_scan(prep, view, ns, base_view=base_view)
    g = float(prep.facts["root_gap_mm"]) if gap == "wps" else (0.0 if gap == "zero" else float(gap))
    try:
        polys, r = detect(A0, c["xyz"], gap=g, **params)
    except (SystemExit, IndexError, ValueError) as e:   # "Part B is empty", or a near-empty cloud
        return [], {"gap_mm": g, "base_view": base_view, "n_base": len(A0),
                    "failure": str(e)[:80]}
    s = [p["summary"] for p in r["paths"]]
    return polys, {"gap_mm": g, "base_view": base_view, "n_base": len(A0),
                   "voxel_mm": r["v"], "n_B": len(r["CB"]), "n_candidates": int(len(r["X"])),
                   "joint_types": ",".join(q["joint_type"] for q in s), "failure": ""}


def spec():
    from .harness import MethodSpec
    return MethodSpec("self-footprint", run, randomised=False, oracle_name=None)
