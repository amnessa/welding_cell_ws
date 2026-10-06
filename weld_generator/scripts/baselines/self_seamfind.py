"""`self-seamfind` — the matching + sphere-tracing seam finder of
`Self_ideas/test2_cagdas/seam_finder.md` (package `Self_ideas/test2_cagdas/seamfind`),
run on the tier-1 corpus through the Phase 4 harness.

Not in `REGISTRY` (the paper's seven-method comparison is closed). Inputs, as the plan
declares them, and where the harness takes them from:

    labelled clouds A, B   the condition's cloud split by `object_id` - the "objects"
                           oracle (lit-lobb's K-Net stage at L0); the plan assumes labels
    normal orientation     `full_exterior`: the sign of the stored surface normals (the
                           plan's synthetic rule, "from the mesh"); `single`: towards the
                           camera (the plan's real-scan rule)
    gap bound              ISO 9692-1 / 5817 range of the corpus (5 mm) by default;
                           `gap="wps"` passes the scene's root gap (+0.5 mm) instead

L1 (`oracle=False`) is not defined - the method has no label-free mode - and raises.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[2]
_PKG = _ROOT / "Self_ideas" / "test2_cagdas"
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from seamfind import Params, extract  # noqa: E402


def run(prep, seed: int, oracle: bool, view: str, ns: float, gap: str = "iso", **params):
    if not oracle:
        raise ValueError("self-seamfind consumes part labels; there is no L1 arm")
    c = prep.cloud(view, ns)
    lab = c["object_id"]
    A, B = c["xyz"][lab == 0], c["xyz"][lab == 1]
    prm = Params(**params)
    kw = {}
    if view == "single":
        kw["cam_pos"] = np.asarray(prep.scene["camera"]["T_world_cam"], float)[:3, 3]
    else:
        kw["normals_A"], kw["normals_B"] = c["normals"][lab == 0], c["normals"][lab == 1]
    if gap == "wps":
        kw["gap_hint_mm"] = float(prep.facts["root_gap_mm"]) + 0.5
    r = extract(A, B, prm, **kw)
    polys = [s["points"] for s in r.seams]
    gaps = [float(np.nanmedian(s["gap"])) for s in r.seams]
    return polys, {"gap": gap, "h_mm": r.h, "n_seams_cop": sum(s["joint_class"] == "coplanar" for s in r.seams),
                   "gap_est_med": float(np.median(gaps)) if gaps else float("nan"),
                   "dihedral_med": float(np.nanmedian(np.concatenate([s["dihedral"] for s in r.seams])))
                   if r.seams else float("nan"),
                   "weldable_frac": float(np.mean(np.concatenate([s["weldable"] for s in r.seams])))
                   if r.seams and "weldable" in r.seams[0] else float("nan"),
                   **{f"c_{k}": v for k, v in r.counts.items()},
                   **{f"t_{k}": v for k, v in r.timings.items()}}


def spec():
    from .harness import MethodSpec
    return MethodSpec("self-seamfind", run, randomised=False, oracle_name="objects")
