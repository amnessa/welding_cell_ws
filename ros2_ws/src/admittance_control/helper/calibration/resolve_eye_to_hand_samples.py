#!/usr/bin/env python3

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_ROOT = next(
    (parent for parent in SCRIPT_PATH.parents if (parent / "notebooks").is_dir()),
    SCRIPT_PATH.parent,
)
NOTEBOOKS_DIR = PACKAGE_ROOT / "notebooks"
DEFAULT_SAMPLES_PATH = NOTEBOOKS_DIR / "handeye_samples.npz"
DEFAULT_OUTPUT_PATH = NOTEBOOKS_DIR / "T_base_to_cam_true.npy"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Re-solve eye-to-hand extrinsics from saved handeye_samples.npz without recapturing data."
    )
    parser.add_argument(
        "--samples-path",
        type=Path,
        default=DEFAULT_SAMPLES_PATH,
        help="Path to the saved handeye_samples.npz bundle.",
    )
    parser.add_argument(
        "--output-path",
        type=Path,
        default=DEFAULT_OUTPUT_PATH,
        help="Path to save the corrected 4x4 T_base_to_cam matrix.",
    )
    return parser.parse_args()


def load_samples(samples_path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    samples_path = samples_path.expanduser().resolve()
    if not samples_path.exists():
        raise FileNotFoundError(f"Samples file does not exist: {samples_path}")

    data = np.load(samples_path)
    required_keys = ["R_gripper2base", "t_gripper2base", "R_board2cam", "t_board2cam"]
    missing_keys = [key for key in required_keys if key not in data]
    if missing_keys:
        raise KeyError(f"Samples file is missing keys: {missing_keys}")

    rotations_gripper_to_base = np.asarray(data["R_gripper2base"], dtype=float)
    translations_gripper_to_base = np.asarray(data["t_gripper2base"], dtype=float)
    rotations_board_to_cam = np.asarray(data["R_board2cam"], dtype=float)
    translations_board_to_cam = np.asarray(data["t_board2cam"], dtype=float)

    sample_count = len(rotations_gripper_to_base)
    if sample_count < 3:
        raise ValueError(
            f"Need at least 3 samples to solve hand-eye calibration, found {sample_count}."
        )

    if not (
        len(translations_gripper_to_base) == sample_count
        == len(rotations_board_to_cam)
        == len(translations_board_to_cam)
    ):
        raise ValueError("Sample arrays do not all have the same length.")

    return (
        rotations_gripper_to_base,
        translations_gripper_to_base,
        rotations_board_to_cam,
        translations_board_to_cam,
    )


def invert_pose_sequence(
    rotations_gripper_to_base: np.ndarray,
    translations_gripper_to_base: np.ndarray,
) -> tuple[list[np.ndarray], list[np.ndarray]]:
    rotations_base_to_gripper: list[np.ndarray] = []
    translations_base_to_gripper: list[np.ndarray] = []

    for rotation, translation in zip(rotations_gripper_to_base, translations_gripper_to_base):
        rotation_inverse = rotation.T
        translation_inverse = -rotation_inverse @ translation
        rotations_base_to_gripper.append(rotation_inverse)
        translations_base_to_gripper.append(translation_inverse)

    return rotations_base_to_gripper, translations_base_to_gripper


def solve_eye_to_hand(
    rotations_gripper_to_base: np.ndarray,
    translations_gripper_to_base: np.ndarray,
    rotations_board_to_cam: np.ndarray,
    translations_board_to_cam: np.ndarray,
) -> np.ndarray:
    rotations_base_to_gripper, translations_base_to_gripper = invert_pose_sequence(
        rotations_gripper_to_base,
        translations_gripper_to_base,
    )

    rotation_cam_to_base, translation_cam_to_base = cv2.calibrateHandEye(
        rotations_base_to_gripper,
        translations_base_to_gripper,
        [np.asarray(rotation, dtype=float) for rotation in rotations_board_to_cam],
        [np.asarray(translation, dtype=float) for translation in translations_board_to_cam],
        method=cv2.CALIB_HAND_EYE_TSAI,
    )

    transform_base_to_cam = np.eye(4, dtype=float)
    transform_base_to_cam[:3, :3] = rotation_cam_to_base
    transform_base_to_cam[:3, 3] = translation_cam_to_base.reshape(3)
    return transform_base_to_cam


def save_result(output_path: Path, transform_base_to_cam: np.ndarray) -> Path:
    output_path = output_path.expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.save(output_path, transform_base_to_cam)
    return output_path


def main() -> int:
    args = parse_args()
    (
        rotations_gripper_to_base,
        translations_gripper_to_base,
        rotations_board_to_cam,
        translations_board_to_cam,
    ) = load_samples(args.samples_path)

    print(f"Loaded {len(rotations_gripper_to_base)} samples from {args.samples_path.expanduser().resolve()}")
    print("Inverting robot poses from gripper->base to base->gripper for eye-to-hand calibration...")
    transform_base_to_cam = solve_eye_to_hand(
        rotations_gripper_to_base,
        translations_gripper_to_base,
        rotations_board_to_cam,
        translations_board_to_cam,
    )

    print("=== TRUE Eye-to-Hand Extrinsic Matrix ===")
    print(np.round(transform_base_to_cam, 4))

    saved_path = save_result(args.output_path, transform_base_to_cam)
    print(f"Saved corrected extrinsic matrix to {saved_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())