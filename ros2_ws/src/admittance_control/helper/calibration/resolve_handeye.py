#!/usr/bin/env python3
"""Re-solve the eye-in-hand calibration from saved samples and JUDGE it. numpy only.

    python helper/calibration/resolve_handeye.py                       # report only
    python helper/calibration/resolve_handeye.py --write notebooks/T_tcp_to_cam.npy
    python helper/calibration/resolve_handeye.py --compare notebooks/T_tcp_to_cam.npy
    python helper/calibration/resolve_handeye.py --tcp-offset 0 0 0.028 0 0 0   # pendant TCP

`handeye_samples.npz` (written by extract_extrinsics.py) holds, per sample, the robot
TCP pose in base (R_gripper2base, t_gripper2base) and the ChArUco board pose in the
camera (R_board2cam, t_board2cam). Eye-in-hand: X = T_tcp_cam solves A X = X B over
relative motions; Park & Martin (1994): rotation from the log-map least squares,
translation from a linear least squares.

The judge is the consistency residual: mapped through X, the board must sit at ONE
place in the base frame across all samples - `T_base_tcp_i @ X @ T_cam_board_i` -
so the spread of that position (mm) and orientation (deg) is the calibration's error.
A good calibration: a few mm and under a degree. The file this repo carried on
2026-09-24 had a 120 mm range against its own samples (the OpenCV solve had failed
silently); the re-solve of the same samples has a 3 mm std. Leave-one-out shows which
sample hurts.

The robot pose is the pendant's ACTIVE TCP (getActualTCPPose), and the ROS side attaches
the result to `tool0` (the flange). A non-zero pendant TCP does not show in the residual
(a rigid change of the gripper frame is invisible to it) but shifts the camera by
exactly that offset. Pass it with --tcp-offset (UR pose: x y z in m, rx ry rz axis-angle)
to express the result in tool0: T_tool0_cam = T_tool0_tcp @ T_tcp_cam.
"""

from __future__ import annotations

import argparse
import itertools
import json
import time
from pathlib import Path

import numpy as np

PKG = Path(__file__).resolve().parents[2]
DEFAULT_SAMPLES = PKG / "notebooks" / "handeye_samples.npz"


def T(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    M = np.eye(4); M[:3, :3] = R; M[:3, 3] = np.asarray(t, float).reshape(3); return M


def logm(R: np.ndarray) -> np.ndarray:
    c = np.clip((np.trace(R) - 1) / 2, -1, 1); th = np.arccos(c)
    if th < 1e-9:
        return np.zeros(3)
    return th / (2 * np.sin(th)) * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


def rodrigues(rvec) -> np.ndarray:
    r = np.asarray(rvec, float); th = np.linalg.norm(r)
    if th < 1e-12:
        return np.eye(3)
    k = r / th; K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


def load_samples(path: Path):
    d = np.load(path)
    return (d["R_gripper2base"], d["t_gripper2base"].reshape(-1, 3),
            d["R_board2cam"], d["t_board2cam"].reshape(-1, 3))


def park_martin(Rg, tg, Rb, tb, idx=None) -> np.ndarray:
    idx = list(range(len(Rg))) if idx is None else list(idx)
    As, Bs = [], []
    for i, j in itertools.combinations(idx, 2):
        As.append(np.linalg.inv(T(Rg[i], tg[i])) @ T(Rg[j], tg[j]))     # tcp_i -> tcp_j
        Bs.append(T(Rb[i], tb[i]) @ np.linalg.inv(T(Rb[j], tb[j])))     # cam_i -> cam_j
    M = sum(np.outer(logm(B[:3, :3]), logm(A[:3, :3])) for A, B in zip(As, Bs))
    w, V = np.linalg.eigh(M.T @ M)
    Rx = V @ np.diag(w ** -0.5) @ V.T @ M.T
    if np.linalg.det(Rx) < 0:
        Rx = -Rx
    U, _, Vt = np.linalg.svd(Rx); Rx = U @ Vt                          # exact rotation
    C = np.vstack([np.eye(3) - A[:3, :3] for A in As])
    dvec = np.concatenate([A[:3, 3] - Rx @ B[:3, 3] for A, B in zip(As, Bs)])
    tx = np.linalg.lstsq(C, dvec, rcond=None)[0]
    return T(Rx, tx)


def residual(X, Rg, tg, Rb, tb, idx=None):
    idx = list(range(len(Rg))) if idx is None else list(idx)
    boards = [T(Rg[i], tg[i]) @ X @ T(Rb[i], tb[i]) for i in idx]
    P = np.array([B[:3, 3] for B in boards])
    R0 = boards[0][:3, :3]
    ang = max(np.degrees(np.arccos(np.clip((np.trace(R0.T @ B[:3, :3]) - 1) / 2, -1, 1))) for B in boards)
    return P.std(0) * 1000, (P.max(0) - P.min(0)) * 1000, ang, P


def describe(X, label, Rg, tg, Rb, tb) -> str:
    s, r, a, _ = residual(X, Rg, tg, Rb, tb)
    return (f"{label}: camera origin in tcp {np.round(X[:3, 3] * 1000, 1)} mm, optical axis "
            f"{np.round(X[:3, 2], 3)}, optical x {np.round(X[:3, 0], 3)}\n"
            f"    board-in-base spread: std {np.round(s, 1)} mm, range {np.round(r, 1)} mm, "
            f"rotation {a:.1f} deg  -> {verdict(s, a)}")


def verdict(std_mm, rot_deg) -> str:
    """Position spread is the number that matters for the seam; the rotation spread is
    the worst PAIR of samples and carries the board's own pose noise."""
    n = float(np.linalg.norm(std_mm))
    if n < 6 and rot_deg < 2:
        return "GOOD"
    if n < 10 and rot_deg < 5:
        return "FAIR (usable; more samples with the board nearer would tighten it)"
    return "BAD"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--samples", default=str(DEFAULT_SAMPLES))
    ap.add_argument("--compare", default=None, help="an existing T_tcp_to_cam.npy to judge")
    ap.add_argument("--write", default=None, help="write the re-solved 4x4 here")
    ap.add_argument("--tcp-offset", nargs=6, type=float, default=None, metavar=("X", "Y", "Z", "RX", "RY", "RZ"),
                    help="the pendant's active TCP during capture (m, axis-angle rad); result then in tool0")
    args = ap.parse_args()
    Rg, tg, Rb, tb = load_samples(Path(args.samples))
    n = len(Rg)
    print(f"{n} samples from {args.samples}")
    angs = [np.degrees(np.arccos(np.clip((np.trace(Rg[i].T @ Rg[j]) - 1) / 2, -1, 1)))
            for i, j in itertools.combinations(range(n), 2)]
    print(f"robot rotation diversity: max {max(angs):.0f} deg, median {np.median(angs):.0f} deg "
          f"({'ok' if max(angs) > 40 else 'LOW - rotate more between samples'})")
    if args.compare:
        print(describe(np.load(args.compare), f"file {args.compare}", Rg, tg, Rb, tb))
    X = park_martin(Rg, tg, Rb, tb)
    print(describe(X, "Park-Martin re-solve", Rg, tg, Rb, tb))
    print("leave-one-out (spread without the sample; a sample whose removal helps a lot is suspect):")
    for k in range(n):
        idx = [i for i in range(n) if i != k]
        s, _, a, _ = residual(park_martin(Rg, tg, Rb, tb, idx), Rg, tg, Rb, tb, idx)
        print(f"  drop {k:2d}: std {np.round(s, 1)} mm |{np.linalg.norm(s):5.1f}| rot {a:.1f} deg")
    if args.tcp_offset is not None:
        off = np.asarray(args.tcp_offset, float)
        D = T(rodrigues(off[3:]), off[:3])                            # tool0 -> tcp
        X = D @ X
        print(f"composed with the pendant TCP {off}: camera origin in tool0 {np.round(X[:3, 3] * 1000, 1)} mm")
    if args.write:
        out = Path(args.write); out.parent.mkdir(parents=True, exist_ok=True)
        np.save(out, X)
        std, rng_, rot, _ = residual(park_martin(Rg, tg, Rb, tb), Rg, tg, Rb, tb)
        side = out.with_suffix(".json")
        side.write_text(json.dumps({
            "written": time.strftime("%Y-%m-%d %H:%M:%S"), "samples": str(Path(args.samples).resolve()),
            "n_samples": int(n), "solver": "park_martin",
            "tcp_offset_composed": None if args.tcp_offset is None else list(map(float, args.tcp_offset)),
            "frame": "tool0" if args.tcp_offset is not None else "pendant TCP at capture (NOT tool0)",
            "board_in_base_std_mm": [float(v) for v in std], "rotation_spread_deg": float(rot),
            "camera_origin_mm": [float(v) for v in X[:3, 3] * 1000]}, indent=1))
        print(f"wrote {out} (+ {side.name}: what was composed, from which samples, how good)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
