"""Multi-view captures on disk, their preprocessing, and synthetic captures (steps 4/5 of
notes/multiview_refine_plan.md, designs D3/D4). Pure numpy; no ROS.

A CAPTURE is one refine_pose run's raw data, written by the ICP node (step 5) and
replayed offline by scripts/multiview_refine_offline.py (step 4):

    <results>/multiview/<YYYYmmdd-HHMMSS>/
        capture.json     time, extrinsic file + sha1, kinematics, table-plane file, and per
                         view: azimuth / elevation / roll, joints q, T_base_cam
        view_<k>.npz     xyz: (H, W, 3) float32 organized cloud in the CAMERA optical frame
                         (per-pixel median over the frames, NaN where invalid), T_base_cam
        assembly.json    the saved parts at capture time (the poses being refined)

The camera frame is kept so a capture can be replayed with a different extrinsic: the
cloud is only moved to base_link in `preprocess_view`, with the T_base_cam that was in TF
at capture time (calibrated kinematics + the extrinsic of that moment).

`preprocess_view` is D4: normals from the organized cloud (oriented to the camera), to
base_link, crop to within `crop_margin_m` of the saved parts, the table-plane cut, voxel
downsampling. `render_synthetic` writes captures of box-shaped parts from given camera poses
(exact ray casting against the part boxes, noise along the ray, optionally an extrinsic
translation error) - the same format, so the whole offline path is testable without the robot.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional, Sequence

import numpy as np

from .icp import estimate_normals_organized, load_ply_mesh, sample_mesh_surface, voxel_downsample
from .multiview import _points_box_distance
from .multiview_refine import Part, View, box_of


@dataclass
class PreprocessConfig:
    crop_margin_m: float = 0.03
    ground_offset_m: float = 0.002        # drop points less than this above the table plane
    voxel_m: float = 0.003
    min_depth_m: float = 0.15             # D435i: nothing valid closer anyway
    max_depth_m: float = 1.0


@dataclass
class RawView:
    xyz: np.ndarray                       # (H, W, 3) camera optical frame, NaN invalid
    T_base_cam: np.ndarray
    meta: dict[str, Any] = field(default_factory=dict)


@dataclass
class Capture:
    path: Path
    meta: dict[str, Any]
    views: list[RawView]
    assembly: dict[str, Any]


# ---------------------------------------------------------------- disk ----------------
def save_capture(out_dir: str | Path, views: Sequence[RawView], assembly: dict[str, Any],
                 meta: dict[str, Any]) -> Path:
    """Write one capture (see the module doc). `meta` holds extrinsic_file,
    extrinsic_sha1, kinematics, ground_plane_file, ... ; per-view metadata goes in each
    RawView.meta."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    vmeta = []
    for k, v in enumerate(views):
        np.savez_compressed(out / f"view_{k}.npz", xyz=np.asarray(v.xyz, np.float32),
                            T_base_cam=np.asarray(v.T_base_cam, float))
        vmeta.append({"k": k, "T_base_cam": np.asarray(v.T_base_cam, float).tolist(), **v.meta})
    (out / "assembly.json").write_text(json.dumps(assembly, indent=1))
    (out / "capture.json").write_text(json.dumps({"time": time.strftime("%Y-%m-%d %H:%M:%S"),
                                                  **meta, "views": vmeta}, indent=1))
    return out


def load_capture(path: str | Path) -> Capture:
    p = Path(path)
    meta = json.loads((p / "capture.json").read_text())
    views = []
    for vm in meta["views"]:
        z = np.load(p / f"view_{vm['k']}.npz")
        views.append(RawView(z["xyz"].astype(float), z["T_base_cam"], vm))
    return Capture(p, meta, views, json.loads((p / "assembly.json").read_text()))


def latest_capture(results_dir: str | Path) -> Optional[Path]:
    root = Path(results_dir) / "multiview"
    caps = sorted(d for d in root.glob("*") if (d / "capture.json").exists()) if root.exists() else []
    return caps[-1] if caps else None


# ---------------------------------------------------------------- parts ----------------
def parts_from_assembly(assembly: dict[str, Any], models_dir: str | Path, n_points: int = 4000,
                        rng: Optional[np.random.Generator] = None) -> list[Part]:
    """The saved objects as refinement Parts: CAD surface samples with outward normals,
    vertices for the box, and pose_static as the saved pose."""
    rng = rng or np.random.default_rng(0)
    parts = []
    for o in assembly["objects"]:
        v, f = load_ply_mesh(Path(models_dir) / o["model"])
        scale = 1e-3 if np.abs(v).max() > 5 else 1.0
        p, n = sample_mesh_surface(v, f, n_points, rng, return_normals=True)
        T_scan = np.asarray(o["T_static_camera"], float) if o.get("T_static_camera") is not None else None
        parts.append(Part(o["model"], p * scale, n, v * scale, np.asarray(o["pose_static"], float), T_scan))
    return parts


# ---------------------------------------------------------------- D4 ------------------
def table_plane(path: str | Path | None):
    """(a, b, c) of z = a x + b y + c from table_plane.json, or None."""
    if not path or not Path(path).exists():
        return None
    pl = json.loads(Path(path).read_text())["plane"]
    return float(pl["a"]), float(pl["b"]), float(pl["c"])


def preprocess_view(raw: RawView, parts: Sequence[Part], cfg: Optional[PreprocessConfig] = None,
                    plane=None) -> View:
    """D4 for one view: organized normals (towards the camera), base_link, crop to the
    saved parts (+ crop_margin_m), table-plane cut, voxel downsampling."""
    cfg = cfg or PreprocessConfig()
    xyz = np.asarray(raw.xyz, float)
    nrm = estimate_normals_organized(xyz)
    ok = np.isfinite(xyz).all(2) & np.isfinite(nrm).all(2)
    ok &= (xyz[..., 2] > cfg.min_depth_m) & (xyz[..., 2] < cfg.max_depth_m)
    pc, nc = xyz[ok], nrm[ok]
    T = np.asarray(raw.T_base_cam, float)
    pb, nb = pc @ T[:3, :3].T + T[:3, 3], nc @ T[:3, :3].T
    boxes = [box_of(p, p.T_saved) for p in parts]
    near = np.min([_points_box_distance(pb, b) for b in boxes], axis=0) < cfg.crop_margin_m
    pb, nb = pb[near], nb[near]
    if plane is not None:
        a, b, c = plane
        keep = pb[:, 2] - (a * pb[:, 0] + b * pb[:, 1] + c) > cfg.ground_offset_m
        pb, nb = pb[keep], nb[keep]
    pb, nb = voxel_downsample(pb, cfg.voxel_m, nb)
    return View(pb, nb, T[:3, :3].copy())


# ---------------------------------------------------------------- synthetic ------------
def pinhole(width: int, height: int, hfov_deg: float = 69.0) -> np.ndarray:
    f = width / 2 / np.tan(np.radians(hfov_deg) / 2)
    return np.array([[f, 0, (width - 1) / 2], [0, f, (height - 1) / 2], [0, 0, 1.0]])


def render_view(parts: Sequence[Part], poses: Sequence[np.ndarray], T_base_cam_true: np.ndarray,
                K: np.ndarray, width: int, height: int, noise_m: float = 0.0005,
                rng: Optional[np.random.Generator] = None) -> np.ndarray:
    """An organized cloud (camera frame, NaN invalid) of the parts as the camera at
    T_base_cam_true sees them: every pixel's ray cast exactly against each part's box (the
    plate parts ARE boxes), the nearest hit, noise along the ray. (A first version
    z-buffered surface samples and re-cast them along the pixel-centre ray; at 1.3 mm pixels
    and 45-60 deg incidence that biased every view ~1 mm towards its camera - which looks
    exactly like an extrinsic error along the optical axis.)"""
    rng = rng or np.random.default_rng(0)
    uu, vv = np.meshgrid(np.arange(width), np.arange(height))
    dirs = np.stack([(uu - K[0, 2]) / K[0, 0], (vv - K[1, 2]) / K[1, 1], np.ones_like(uu, float)], axis=2)
    dirs = dirs.reshape(-1, 3)                              # z = 1: the ray parameter IS the depth
    Tinv = np.linalg.inv(T_base_cam_true)
    depth = np.full(len(dirs), np.inf)
    for p, T in zip(parts, poses):
        bx = box_of(p, T)
        c = Tinv[:3, :3] @ bx["centre"] + Tinv[:3, 3]
        R = Tinv[:3, :3] @ bx["R"]
        o = R.T @ (-c)                                      # camera origin in the box frame
        dl = dirs @ R                                       # ray directions in the box frame
        with np.errstate(divide="ignore", invalid="ignore"):
            t1 = (-bx["half"] - o) / dl
            t2 = (bx["half"] - o) / dl
        tmin = np.nanmax(np.minimum(t1, t2), axis=1)
        tmax = np.nanmin(np.maximum(t1, t2), axis=1)
        hit = (tmax >= tmin) & (tmin > 0.05)
        depth = np.where(hit & (tmin < depth), tmin, depth)
    z = np.where(np.isfinite(depth), depth + rng.normal(0, noise_m, depth.shape), np.nan)
    return (dirs * z[:, None]).reshape(height, width, 3).astype(np.float32)


def render_synthetic(out_dir: str | Path, parts: Sequence[Part], poses_true: Sequence[np.ndarray],
                     saved_poses: Sequence[np.ndarray], cam_poses_true: Sequence[np.ndarray],
                     d_cam_mm: Sequence[float] = (0.0, 0.0, 0.0), width: int = 424, height: int = 240,
                     noise_m: float = 0.0005, view_meta: Optional[Sequence[dict]] = None,
                     models_dir: str | Path | None = None, extra_meta: Optional[dict] = None,
                     seed: int = 0, scan_cam: Optional[np.ndarray] = None) -> Path:
    """Write a synthetic capture: each view rendered from its TRUE camera pose, but stored
    with the T_base_cam the robot would believe - shifted by R_cam d, the error of an
    extrinsic whose translation is off by R_tc d in tool0."""
    rng = np.random.default_rng(seed)
    K = pinhole(width, height)
    d = np.asarray(d_cam_mm, float) / 1000
    raws = []
    for k, Tc in enumerate(cam_poses_true):
        xyz = render_view(parts, poses_true, Tc, K, width, height, noise_m, rng=rng)
        T_believed = np.asarray(Tc, float).copy()
        T_believed[:3, 3] += Tc[:3, :3] @ d
        raws.append(RawView(xyz, T_believed, dict((view_meta or [{}] * len(cam_poses_true))[k])))
    assembly = {"static_frame": "base_link",
                "objects": [{"model": p.name, "pose_static": np.asarray(T, float).tolist(),
                             **({"T_static_camera": np.asarray(scan_cam, float).tolist()} if scan_cam is not None else {})}
                            for p, T in zip(parts, saved_poses)]}
    meta = {"synthetic": True, "d_cam_mm_injected": list(map(float, d_cam_mm)),
            "pose_true": [np.asarray(T, float).tolist() for T in poses_true], **(extra_meta or {})}
    return save_capture(out_dir, raws, assembly, meta)


# ---------------------------------------------------------------- the whole path --------
@dataclass
class Replay:
    capture: Capture
    parts: list[Part]
    views: list[View]
    view_points: list[tuple[int, int]]    # (valid pixels, points after preprocessing) per view
    results: list
    diag: Any


def refine_capture(cap: Capture, models_dir: str | Path, rcfg=None, pcfg: Optional[PreprocessConfig] = None,
                   ground_plane_file: str | Path | None = None) -> Replay:
    """D4 + D5-D7 on a capture: what refine_pose does after the robot has moved (step 5
    calls this right after capturing; the offline script replays saved captures)."""
    from .multiview_refine import RefineConfig, refine_assembly
    rcfg = rcfg or RefineConfig()
    parts = parts_from_assembly(cap.assembly, models_dir)
    plane = table_plane(ground_plane_file if ground_plane_file is not None else cap.meta.get("ground_plane_file"))
    views, counts = [], []
    for raw in cap.views:
        v = preprocess_view(raw, parts, pcfg, plane)
        views.append(v)
        counts.append((int(np.isfinite(raw.xyz).all(2).sum()), len(v.pts)))
    results, diag = refine_assembly(parts, views, rcfg)
    return Replay(cap, parts, views, counts, results, diag)


def format_replay(rp: Replay) -> str:
    """The report refine_pose replies with (and the offline script prints)."""
    lines = []
    m = rp.capture.meta
    lines.append(f"capture {rp.capture.path.name}: {len(rp.views)} views, taken {m.get('time', '?')}"
                 + (f", SYNTHETIC (injected d = {m.get('d_cam_mm_injected')} mm)" if m.get("synthetic") else ""))
    for k, ((n_raw, n_pts), raw) in enumerate(zip(rp.view_points, rp.capture.views)):
        vm = raw.meta
        ang = (f"az {vm['azimuth_deg']:.0f} el {vm['elevation_deg']:.0f} roll {vm['roll_deg']:.0f}, "
               if "azimuth_deg" in vm else "")
        lines.append(f"  view {k}: {ang}{n_raw} valid pixels -> {n_pts} points near the parts")
    d = rp.diag
    lines.append(f"  rounds {d.rounds_run}, matching distance {d.max_corr_m * 1000:.0f} mm, "
                 f"smallest parallel-face gap {d.parallel_gap_m * 1000:.1f} mm")
    for r in rp.results:
        lines.append(f"  {r.name}: {'ACCEPTED' if r.accepted else 'KEPT AS SAVED'}"
                     f"{'' if r.accepted else ' - ' + r.reason}")
        if getattr(r, "prior_shift_mm", 0.0):
            lines.append(f"     of the correction below, {r.prior_shift_mm:.2f} mm is the camera error at its scan pose "
                         f"(R_scan d, taken out of the saved pose first); beyond it {r.beyond_mm:.2f} mm / "
                         f"{r.beyond_deg:.2f} deg (this is what the limits judge)")
        lines.append(f"     {'correction' if r.accepted else 'attempted correction (not applied)'} "
                     f"{r.correction_mm:.2f} mm / {r.correction_deg:.2f} deg; fitness {r.fitness:.2f}, "
                     f"rmse {r.rmse_mm:.2f} mm; {r.n_owned} points owned; overlap {r.max_penetration_mm:.2f} mm; "
                     f"single views spread {r.view_spread_mm:.2f} mm")
        if r.weak:
            lines.append(f"     not measured (held at the saved pose): {', '.join(r.weak)}")
        for nb, (gmin, gmax) in getattr(r, "gaps_mm", {}).items():
            lim = r.gap_limit_mm.get(nb)
            lines.append(f"     resting on {nb}: gap {gmin:+.1f} .. {gmax:+.1f} mm along the contact"
                         + (f" (ISO 5817 no. 617 limit {lim:.1f} mm)" if lim is not None else ""))
        for wn in getattr(r, "warnings", []):
            lines.append(f"     WARNING: {wn}")
        truth = m.get("pose_true")
        if truth:
            from .multiview_refine import pose_delta
            k = rp.results.index(r)
            Tt = np.asarray(truth[k], float)
            c = rp.parts[k].pts.mean(0)
            s_mm, s_deg = pose_delta(r.T_saved, Tt, c)
            f_mm, f_deg = pose_delta(r.T, Tt, c)
            lines.append(f"     vs the synthetic truth: saved {s_mm:.2f} mm / {s_deg:.2f} deg -> "
                         f"result {f_mm:.2f} mm / {f_deg:.2f} deg (unmeasured directions included)")
    if d.extrinsic_d_mm is not None:
        sig = ", ".join(f"{s:.2f}" for s in d.extrinsic_sigma_mm) if d.extrinsic_sigma_mm is not None else "?"
        lines.append(f"  extrinsic translation error d = {np.round(d.extrinsic_d_mm, 2).tolist()} mm "
                     f"(camera frame; std per direction {sig} mm; fit rms {d.extrinsic_rms_mm:.2f} mm)")
        for wk in d.extrinsic_weak:
            lines.append(f"     not determined: {wk}")
        if d.applied_d_mm is not None:
            lines.append(f"  online self-calibration: d taken out of every view and refined again; the "
                         f"corrected views still say {np.round(d.residual_d_mm, 2).tolist() if d.residual_d_mm is not None else '?'} mm")
    return "\n".join(lines)
