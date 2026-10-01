"""Self-calibration of the camera extrinsic's TRANSLATION from multi-view refinements
(step 7 of notes/multiview_refine_plan.md). Pure numpy; no ROS.

Every `refine_pose` run estimates `d` (multiview_refine.AssemblyDiag.extrinsic_d_mm): the
extrinsic translation error in the CAMERA frame, recovered from how the part poses that
each view alone gives move with the camera's orientation. The synthetic tests of step 3
showed the joint refinement does not average such an error out (the part shared by all
views stays in the pose) - removing it from the extrinsic does.

The sign: with the extrinsic in use T_tool0_cam = (R_tc, t_used) and the true translation
t_true = t_used - Delta (tool0 frame), a camera point lands at base <- tool0 <- (R_tc,
t_used), i.e. displaced by R_base_tool0 Delta = R_base_cam (R_tc^T Delta). That is the
R_cam d of the estimate, so d = R_tc^T Delta and the correction is
    t_true = t_used - R_tc d.

Not every direction of d is determined by one run's views (multiview_refine.estimate_
extrinsic: views at one elevation cannot see a shift that moves every cloud straight up
or down), so each run carries its 3x3 INFORMATION (1/mm^2) and d only in the directions it
determines. One run is not enough to touch a calibration: each is kept in a history
(`notebooks/selfcal_history.json`) with the extrinsic FILE it was made with (sha1 of the
content: estimates under different extrinsics are never mixed). The runs are combined by
their information, d = (sum I_r)^+ sum I_r d_r, so a direction one run cannot see is
filled in by another with different views; directions no run determines stay at 0 and
are named. A correction is offered when at least `min_runs` usable runs agree: each
run's deviation from the combined d, within the directions IT determines, has an RMS
under `max_spread_mm`, and leaving any one run out moves the result less than
`max_loo_mm`. It is written NEXT TO the extrinsic as
`T_tcp_to_cam_selfcal.npy` with a sidecar `.json` (like the table refinement); promoting
it (copying it over T_tcp_to_cam.npy, or launching with extrinsic_path:=...) is by hand.
Only the translation; the rotation stays the table refinement's job.
"""

from __future__ import annotations

import hashlib
import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional, Sequence

import numpy as np


@dataclass
class SelfcalConfig:
    min_runs: int = 3
    max_spread_mm: float = 1.0
    max_loo_mm: float = 1.0
    max_sigma_mm: float = 0.5             # a direction is determined when its std is at most this


def file_sha1(path: str | Path) -> str:
    return hashlib.sha1(Path(path).read_bytes()).hexdigest()


def corrected_extrinsic(T_tool0_cam: np.ndarray, d_cam_mm: Sequence[float]) -> np.ndarray:
    """The extrinsic with the estimated translation error taken out: t - R_tc d."""
    T = np.asarray(T_tool0_cam, float).copy()
    T[:3, 3] = T[:3, 3] - T[:3, :3] @ (np.asarray(d_cam_mm, float) / 1000.0)
    return T


# ---------------------------------------------------------------- history --------------
def load_history(path: str | Path) -> list[dict[str, Any]]:
    p = Path(path)
    if not p.exists():
        return []
    return json.loads(p.read_text()).get("runs", [])


def record_run(path: str | Path, d_cam_mm: Sequence[float], info: Optional[np.ndarray],
               extrinsic_file: str | Path, meta: Optional[dict[str, Any]] = None) -> dict[str, Any]:
    """Append one refine_pose run's estimate (d and its 3x3 information, 1/mm^2, both in
    the camera frame) to the history file and return the entry."""
    p = Path(path)
    runs = load_history(p)
    entry = {"time": time.strftime("%Y-%m-%d %H:%M:%S"), "d_cam_mm": [float(x) for x in d_cam_mm],
             "info_per_mm2": None if info is None else np.asarray(info, float).tolist(),
             "extrinsic_file": str(extrinsic_file),
             "extrinsic_sha1": file_sha1(extrinsic_file), **(meta or {})}
    runs.append(entry)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps({"_comment": "refine_pose extrinsic-translation estimates (selfcal.py); "
                                         "d in the camera frame, mm", "runs": runs}, indent=1))
    return entry


@dataclass
class Verdict:
    ready: bool
    message: str
    d_cam_mm: Optional[np.ndarray] = None       # the combined estimate (determined directions only)
    spread_mm: float = float("nan")             # RMS of each run's deviation in its determined directions
    loo_mm: float = float("nan")                # largest shift of the result leaving one run out
    used: list[int] = field(default_factory=list)   # indices into the history
    skipped: dict[str, int] = field(default_factory=dict)
    undetermined: list[list[float]] = field(default_factory=list)   # camera-frame directions no run sees


def _determined(I: np.ndarray, max_sigma_mm: float) -> tuple[np.ndarray, np.ndarray]:
    """(mask, eigenvectors): directions whose std 1/sqrt(info) is at most max_sigma_mm."""
    lam, U = np.linalg.eigh((I + I.T) / 2)
    return lam >= 1.0 / max_sigma_mm ** 2, U


def _projector(I: np.ndarray, max_sigma_mm: float) -> np.ndarray:
    ok, U = _determined(I, max_sigma_mm)
    return U[:, ok] @ U[:, ok].T


def combine(runs: Sequence[dict[str, Any]], max_sigma_mm: float = 0.5):
    """(d, I, undetermined directions) from runs weighted by their information."""
    Is = [np.asarray(r["info_per_mm2"], float) for r in runs]
    I = np.sum(Is, axis=0)
    d = np.linalg.pinv(I, rcond=1e-9) @ np.sum([Ii @ np.asarray(r["d_cam_mm"], float) for Ii, r in zip(Is, runs)], axis=0)
    ok, U = _determined(I, max_sigma_mm)
    und = [U[:, k].tolist() for k in range(3) if not ok[k]]
    return U[:, ok] @ U[:, ok].T @ d, I, und


def evaluate(runs: Sequence[dict[str, Any]], extrinsic_sha1: str, cfg: Optional[SelfcalConfig] = None
             ) -> Verdict:
    """Is there a correction the runs made with THIS extrinsic agree on?"""
    cfg = cfg or SelfcalConfig()
    skipped: dict[str, int] = {}
    used = []
    for k, r in enumerate(runs):
        info = r.get("info_per_mm2")
        if r.get("extrinsic_sha1") != extrinsic_sha1:
            skipped["other extrinsic"] = skipped.get("other extrinsic", 0) + 1
        elif info is None or not _determined(np.asarray(info, float), cfg.max_sigma_mm)[0].any():
            skipped["determines nothing"] = skipped.get("determines nothing", 0) + 1
        else:
            used.append(k)
    v = Verdict(False, "", used=used, skipped=skipped)
    if not used:
        v.message = f"no usable run with this extrinsic, need {cfg.min_runs}"
        return v
    sel = [runs[k] for k in used]
    d, I, und = combine(sel, cfg.max_sigma_mm)
    v.d_cam_mm, v.undetermined = d, und
    if len(used) < cfg.min_runs:
        v.message = f"{len(used)} usable run(s) with this extrinsic, need {cfg.min_runs}"
        return v
    dev = [_projector(np.asarray(r["info_per_mm2"], float), cfg.max_sigma_mm)
           @ (np.asarray(r["d_cam_mm"], float) - d) for r in sel]
    v.spread_mm = float(np.sqrt(np.mean([np.sum(e ** 2) for e in dev])))
    v.loo_mm = float(max(np.linalg.norm(combine(sel[:i] + sel[i + 1:], cfg.max_sigma_mm)[0] - d)
                         for i in range(len(sel))))
    note = (f"; NOT determined by any run (left as is): {np.round(und, 2).tolist()}" if und else "")
    if v.spread_mm > cfg.max_spread_mm:
        v.message = f"the runs disagree: spread {v.spread_mm:.2f} mm > {cfg.max_spread_mm:g}"
    elif v.loo_mm > cfg.max_loo_mm:
        v.message = f"one run moves the result by {v.loo_mm:.2f} mm > {cfg.max_loo_mm:g}"
    else:
        v.ready = True
        v.message = (f"{len(used)} runs agree on d = {np.round(d, 2).tolist()} mm (camera frame), "
                     f"spread {v.spread_mm:.2f} mm, leave-one-out {v.loo_mm:.2f} mm{note}")
    return v


def write_selfcal(extrinsic_path: str | Path, verdict: Verdict, runs: Sequence[dict[str, Any]],
                  kinematics: str = "", out_path: Optional[str | Path] = None) -> Path:
    """Write the corrected extrinsic NEXT TO the one in use, with a sidecar .json."""
    if not verdict.ready:
        raise ValueError(f"no agreed correction: {verdict.message}")
    src = Path(extrinsic_path)
    T = np.load(src)
    T_new = corrected_extrinsic(T, verdict.d_cam_mm)
    out = Path(out_path) if out_path else src.with_name("T_tcp_to_cam_selfcal.npy")
    np.save(out, T_new)
    shift = T_new[:3, 3] - T[:3, 3]
    out.with_suffix(".json").write_text(json.dumps({
        "written": time.strftime("%Y-%m-%d %H:%M:%S"),
        "base_extrinsic": str(src), "base_extrinsic_sha1": file_sha1(src),
        "frame": "tool0",
        "d_cam_mm": np.asarray(verdict.d_cam_mm).tolist(),
        "translation_change_tool0_mm": (shift * 1000).tolist(),
        "spread_mm": verdict.spread_mm, "leave_one_out_mm": verdict.loo_mm,
        "undetermined_cam_directions": verdict.undetermined,
        "runs": [runs[k] for k in verdict.used],
        "kinematics": kinematics,
        "note": ("translation self-calibrated from multi-view refine_pose runs (selfcal.py): "
                 "t_new = t - R_tool0_cam d, d only in the directions the runs determine "
                 "(undetermined_cam_directions are left as they were); rotation unchanged. "
                 "Promote by hand."),
    }, indent=1))
    return out
