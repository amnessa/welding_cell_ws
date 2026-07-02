#!/usr/bin/env python3

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
from rtde_receive import RTDEReceiveInterface


SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_ROOT = next(
    (parent for parent in SCRIPT_PATH.parents if (parent / "notebooks").is_dir()),
    SCRIPT_PATH.parent,
)
NOTEBOOKS_DIR = PACKAGE_ROOT / "notebooks"
DEFAULT_ROBOT_IP = "192.168.8.4"
DEFAULT_INTRINSICS_PATH = NOTEBOOKS_DIR / "realsense_intrinsics.npy"
DEFAULT_SAMPLES_PATH = NOTEBOOKS_DIR / "handeye_samples.npz"
DEFAULT_REFERENCE_BOARD_PATH = NOTEBOOKS_DIR / "charuco_reference.png"
DEFAULT_DEBUG_FRAME_PATH = NOTEBOOKS_DIR / "handeye_debug_latest.png"
DEFAULT_ARUCO_DICTIONARY = "DICT_4X4_250"
DEFAULT_CALIBRATION_SETUP = "eye-in-hand"

# The solved extrinsic lives in a different frame depending on the setup:
# eye-in-hand yields the camera pose in the TCP frame (constant while the
# camera stays bolted to the wrist), eye-to-hand yields it in the base frame.
DEFAULT_OUTPUT_PATHS = {
    "eye-in-hand": NOTEBOOKS_DIR / "T_tcp_to_cam.npy",
    "eye-to-hand": NOTEBOOKS_DIR / "T_base_to_cam.npy",
}
RESULT_LABELS = {
    "eye-in-hand": "T_tcp_to_cam",
    "eye-to-hand": "T_base_to_cam",
}


def available_dictionary_names() -> list[str]:
    return sorted(name for name in dir(cv2.aruco) if name.startswith("DICT_"))


def resolve_aruco_dictionary(dictionary_name: str):
    if not hasattr(cv2.aruco, dictionary_name):
        raise ValueError(
            f"Unknown ArUco dictionary '{dictionary_name}'. Available values include: {', '.join(available_dictionary_names()[:12])}, ..."
        )
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture ChArUco samples from a RealSense RGB stream and solve for camera extrinsics."
    )
    parser.add_argument("--robot-ip", default=DEFAULT_ROBOT_IP, help="UR robot IP address.")
    parser.add_argument(
        "--dictionary",
        default=DEFAULT_ARUCO_DICTIONARY,
        choices=available_dictionary_names(),
        help="ArUco dictionary used by the printed ChArUco board.",
    )
    parser.add_argument(
        "--intrinsics-path",
        type=Path,
        default=DEFAULT_INTRINSICS_PATH,
        help="Path to a saved 3x3 camera intrinsics matrix (.npy).",
    )
    parser.add_argument(
        "--output-path",
        type=Path,
        default=None,
        help=(
            "Path for the solved 4x4 extrinsic matrix (.npy). Defaults to "
            "T_tcp_to_cam.npy for eye-in-hand and T_base_to_cam.npy for eye-to-hand."
        ),
    )
    parser.add_argument(
        "--samples-output",
        type=Path,
        default=DEFAULT_SAMPLES_PATH,
        help="Path for captured hand-eye samples (.npz).",
    )
    parser.add_argument(
        "--calibration-setup",
        choices=["eye-to-hand", "eye-in-hand"],
        default=DEFAULT_CALIBRATION_SETUP,
        help=(
            "Hand-eye setup. Use 'eye-in-hand' for a camera mounted on the robot looking at a "
            "fixed board, 'eye-to-hand' for a fixed camera looking at the robot workspace."
        ),
    )
    parser.add_argument(
        "--reference-board-path",
        type=Path,
        default=DEFAULT_REFERENCE_BOARD_PATH,
        help="Path where a generated reference ChArUco board image will be saved.",
    )
    parser.add_argument(
        "--debug-frame-path",
        type=Path,
        default=DEFAULT_DEBUG_FRAME_PATH,
        help="Path where the latest failed detection frame will be saved.",
    )
    parser.add_argument("--width", type=int, default=1280, help="RGB stream width.")
    parser.add_argument("--height", type=int, default=720, help="RGB stream height.")
    parser.add_argument("--fps", type=int, default=30, help="RGB stream frames per second.")
    parser.add_argument(
        "--target-samples",
        type=int,
        default=15,
        help="Recommended number of captures before solving.",
    )
    parser.add_argument(
        "--min-samples",
        type=int,
        default=5,
        help="Minimum number of captures required before solving.",
    )
    parser.add_argument(
        "--squares-x",
        type=int,
        default=4,
        help="Number of chessboard squares along x for the ChArUco board.",
    )
    parser.add_argument(
        "--squares-y",
        type=int,
        default=4,
        help="Number of chessboard squares along y for the ChArUco board.",
    )
    parser.add_argument(
        "--square-length",
        type=float,
        default=0.030,
        help="ChArUco square length in meters.",
    )
    parser.add_argument(
        "--marker-length",
        type=float,
        default=0.022,
        help="ChArUco marker length in meters.",
    )
    parser.add_argument(
        "--legacy-pattern",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Use the legacy OpenCV ChArUco square parity. Enable this when the generated reference starts with the opposite square pattern from the printed board.",
    )
    args = parser.parse_args()
    if args.output_path is None:
        args.output_path = DEFAULT_OUTPUT_PATHS[args.calibration_setup]
    return args


def load_intrinsics(intrinsics_path: Path) -> tuple[np.ndarray, np.ndarray]:
    intrinsics_path = intrinsics_path.expanduser().resolve()
    if not intrinsics_path.exists():
        raise FileNotFoundError(f"Intrinsics file does not exist: {intrinsics_path}")

    matrix = np.load(intrinsics_path)
    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != (3, 3):
        raise ValueError(
            f"Expected a 3x3 intrinsics matrix at {intrinsics_path}, found shape {matrix.shape}."
        )

    distortion = np.zeros((5, 1), dtype=float)
    print(f"Loaded intrinsics from {intrinsics_path}")
    print("Using zero distortion coefficients because the saved file contains only K.")
    return matrix, distortion


def connect_robot(robot_ip: str) -> RTDEReceiveInterface:
    print(f"Connecting to UR robot at {robot_ip}...")
    try:
        rtde_r = RTDEReceiveInterface(robot_ip)
        tcp_pose = rtde_r.getActualTCPPose()
    except Exception as exc:
        raise RuntimeError(
            f"Failed to connect to the robot at {robot_ip}. Check the IP, network, and External Control state."
        ) from exc

    print("Robot connection OK.")
    print(f"Current TCP pose: {np.round(np.asarray(tcp_pose, dtype=float), 4).tolist()}")
    return rtde_r


def start_camera(serial_number: str | None, width: int, height: int, fps: int):
    ctx = rs.context()
    devices = list(ctx.query_devices())
    if not devices:
        raise RuntimeError(
            "No RealSense device is visible through pyrealsense2. Reopen the devcontainer or reconnect the camera first."
        )

    device = devices[0]
    device_name = device.get_info(rs.camera_info.name) if device.supports(rs.camera_info.name) else "RealSense camera"
    device_serial = (
        device.get_info(rs.camera_info.serial_number)
        if device.supports(rs.camera_info.serial_number)
        else serial_number
    )
    sensor_names = [
        sensor.get_info(rs.camera_info.name)
        for sensor in device.query_sensors()
        if sensor.supports(rs.camera_info.name)
    ]
    if "RGB Camera" not in sensor_names:
        raise RuntimeError(
            f"{device_name} does not expose an RGB Camera sensor right now. Visible sensors: {sensor_names}."
        )

    pipeline = rs.pipeline()
    config = rs.config()
    if device_serial:
        config.enable_device(device_serial)
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

    print(f"Starting RealSense RGB stream from {device_name} at {width}x{height}@{fps}...")
    profile = pipeline.start(config)
    for _ in range(10):
        pipeline.wait_for_frames()

    print(f"Camera connection OK. Sensors: {sensor_names}")
    return pipeline, profile, device_name, sensor_names


def create_board(args: argparse.Namespace):
    aruco_dict = resolve_aruco_dictionary(args.dictionary)
    board = cv2.aruco.CharucoBoard(
        (args.squares_x, args.squares_y),
        args.square_length,
        args.marker_length,
        aruco_dict,
    )
    board.setLegacyPattern(args.legacy_pattern)
    detector_params = cv2.aruco.DetectorParameters()
    aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)
    return board, aruco_detector


def save_reference_board(board, output_path: Path) -> Path:
    output_path = output_path.expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    board_image = board.generateImage((1400, 1400), marginSize=80, borderBits=1)
    if not cv2.imwrite(str(output_path), board_image):
        raise RuntimeError(f"Failed to write reference board image to {output_path}")
    return output_path


def save_debug_frame(output_path: Path, frame: np.ndarray) -> Path:
    output_path = output_path.expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output_path), frame):
        raise RuntimeError(f"Failed to write debug frame to {output_path}")
    return output_path


def draw_status_lines(image: np.ndarray, lines: list[str]) -> None:
    y = 28
    for line in lines:
        cv2.putText(
            image,
            line,
            (10, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
        y += 28


def save_outputs(
    output_path: Path,
    samples_output: Path,
    transform: np.ndarray,
    robot_rotations: list[np.ndarray],
    robot_translations: list[np.ndarray],
    board_rotations: list[np.ndarray],
    board_translations: list[np.ndarray],
) -> None:
    output_path = output_path.expanduser().resolve()
    samples_output = samples_output.expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    samples_output.parent.mkdir(parents=True, exist_ok=True)

    np.save(output_path, transform)
    np.savez(
        samples_output,
        R_gripper2base=np.stack(robot_rotations),
        t_gripper2base=np.stack(robot_translations),
        R_board2cam=np.stack(board_rotations),
        t_board2cam=np.stack(board_translations),
    )

    print(f"Saved extrinsic matrix to {output_path}")
    print(f"Saved capture bundle to {samples_output}")


def invert_pose_sequence(
    rotations: list[np.ndarray],
    translations: list[np.ndarray],
) -> tuple[list[np.ndarray], list[np.ndarray]]:
    inverted_rotations: list[np.ndarray] = []
    inverted_translations: list[np.ndarray] = []

    for rotation, translation in zip(rotations, translations):
        rotation_inverse = rotation.T
        translation_inverse = -rotation_inverse @ translation
        inverted_rotations.append(rotation_inverse)
        inverted_translations.append(translation_inverse)

    return inverted_rotations, inverted_translations


def solve_extrinsics(
    robot_rotations: list[np.ndarray],
    robot_translations: list[np.ndarray],
    board_rotations: list[np.ndarray],
    board_translations: list[np.ndarray],
    calibration_setup: str,
) -> np.ndarray:
    if calibration_setup == "eye-to-hand":
        print(
            "Solving eye-to-hand calibration: inverting robot poses from gripper->base to base->gripper before cv2.calibrateHandEye()."
        )
        calibration_rotations, calibration_translations = invert_pose_sequence(
            robot_rotations,
            robot_translations,
        )
    else:
        print(
            "Solving eye-in-hand calibration directly from gripper->base robot poses. "
            "The result is the camera pose in the TCP frame (T_tcp_to_cam)."
        )
        calibration_rotations = robot_rotations
        calibration_translations = robot_translations

    # eye-in-hand: cv2 returns cam->gripper, i.e. the camera pose in the TCP frame.
    # eye-to-hand (with inverted robot poses): cv2 returns cam->base instead.
    rotation, translation = cv2.calibrateHandEye(
        calibration_rotations,
        calibration_translations,
        board_rotations,
        board_translations,
        method=cv2.CALIB_HAND_EYE_TSAI,
    )

    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation.reshape(3)
    return transform


def main() -> int:
    args = parse_args()

    if args.min_samples < 3:
        raise ValueError("--min-samples must be at least 3.")
    if args.target_samples < args.min_samples:
        raise ValueError("--target-samples must be greater than or equal to --min-samples.")

    intrinsics_matrix, distortion_coeffs = load_intrinsics(args.intrinsics_path)
    board, aruco_detector = create_board(args)
    reference_board_path = save_reference_board(board, args.reference_board_path)
    debug_frame_path = args.debug_frame_path.expanduser().resolve()

    rtde_r = None
    pipeline = None
    pipeline_started = False
    window_name = "Extrinsic Extraction (c: capture, q: solve, Esc: abort)"

    robot_rotations: list[np.ndarray] = []
    robot_translations: list[np.ndarray] = []
    board_rotations: list[np.ndarray] = []
    board_translations: list[np.ndarray] = []

    try:
        rtde_r = connect_robot(args.robot_ip)
        pipeline, _profile, device_name, _sensor_names = start_camera(
            serial_number=None,
            width=args.width,
            height=args.height,
            fps=args.fps,
        )
        pipeline_started = True

        print("Preview ready.")
        if args.calibration_setup == "eye-in-hand":
            print(
                "Eye-in-hand: keep the ChArUco board fixed in the workspace and move the robot "
                "so the wrist camera sees it from varied positions and orientations."
            )
        else:
            print("Move the robot so the ChArUco board is clearly visible to the camera.")
        print("Press 'c' to capture a sample, 'q' to solve, or Esc to abort.")
        print(
            "Expected board: "
            + f"{args.dictionary}, {args.squares_x}x{args.squares_y} squares, "
            + f"square={args.square_length:.3f} m, marker={args.marker_length:.3f} m, "
            + f"legacy_pattern={args.legacy_pattern}"
        )
        print(f"Reference board image saved to {reference_board_path}")

        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        last_status = "Waiting for board detection"
        last_capture_time = None

        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            image = np.asanyarray(color_frame.get_data())
            image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            display_image = image.copy()

            corners, ids, _rejected = aruco_detector.detectMarkers(image_gray)
            marker_count = 0 if ids is None else len(ids)
            rejected_count = len(_rejected)
            detected_ids = [] if ids is None else ids.reshape(-1).tolist()
            board_found = False
            rvec = None
            tvec = None
            charuco_count = 0

            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(display_image, corners, ids)
                _retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                    corners, ids, image_gray, board
                )

                if charuco_ids is not None:
                    charuco_count = len(charuco_ids)

                if charuco_corners is not None and charuco_ids is not None and len(charuco_corners) > 3:
                    cv2.aruco.drawDetectedCornersCharuco(
                        display_image, charuco_corners, charuco_ids, (0, 255, 0)
                    )
                    success, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                        charuco_corners,
                        charuco_ids,
                        board,
                        intrinsics_matrix,
                        distortion_coeffs,
                        None,
                        None,
                    )
                    if success:
                        board_found = True
                        cv2.drawFrameAxes(
                            display_image,
                            intrinsics_matrix,
                            distortion_coeffs,
                            rvec,
                            tvec,
                            0.1,
                        )

            if board_found:
                last_status = f"Pose solved ({charuco_count} corners)"
            elif marker_count == 0:
                last_status = "No ArUco markers detected"
            elif charuco_count == 0:
                last_status = f"{marker_count} markers detected, no ChArUco corners"
            elif charuco_count < 4:
                last_status = f"{marker_count} markers, {charuco_count} corners; need >= 4"
            else:
                last_status = f"{marker_count} markers, {charuco_count} corners; pose solve failed"

            overlay_lines = [
                f"Robot: connected to {args.robot_ip}",
                f"Camera: {device_name}",
                f"Samples: {len(robot_rotations)}/{args.target_samples} (min {args.min_samples})",
                f"Status: {last_status}",
                f"Markers: {marker_count} | Rejected: {rejected_count} | ChArUco: {charuco_count}",
                "Keys: c=capture, q=solve, Esc=abort",
            ]
            if detected_ids:
                id_text = ",".join(str(marker_id) for marker_id in detected_ids[:8])
                if len(detected_ids) > 8:
                    id_text += ",..."
                overlay_lines.append(f"IDs: {id_text}")
            if last_capture_time is not None:
                overlay_lines.append(f"Last capture: {last_capture_time}")
            draw_status_lines(display_image, overlay_lines)

            cv2.imshow(window_name, display_image)
            key = cv2.waitKey(1) & 0xFF

            if key == 27:
                print("Aborted without saving calibration outputs.")
                return 1

            if key == ord("c"):
                if not board_found or rvec is None or tvec is None:
                    saved_debug_frame = save_debug_frame(debug_frame_path, display_image)
                    print("Capture skipped because the ChArUco board pose is not currently solved.")
                    print(
                        f"  markers={marker_count}, rejected={rejected_count}, charuco_corners={charuco_count}, ids={detected_ids}"
                    )
                    print(f"  Saved debug frame to {saved_debug_frame}")
                    print(f"  Reference board image: {reference_board_path}")
                    continue

                try:
                    tcp_pose = rtde_r.getActualTCPPose()
                except Exception as exc:
                    raise RuntimeError("Lost robot connection while capturing a sample.") from exc

                robot_tvec = np.asarray(tcp_pose[:3], dtype=float).reshape(3, 1)
                robot_rvec = np.asarray(tcp_pose[3:], dtype=float).reshape(3, 1)
                robot_rotation, _ = cv2.Rodrigues(robot_rvec)
                board_rotation, _ = cv2.Rodrigues(rvec)

                robot_rotations.append(robot_rotation)
                robot_translations.append(robot_tvec)
                board_rotations.append(board_rotation)
                board_translations.append(np.asarray(tvec, dtype=float).reshape(3, 1))

                last_capture_time = time.strftime("%H:%M:%S")
                print(f"[{len(robot_rotations)}] Sample captured at {last_capture_time}")
                print(f"  TCP pose: {np.round(np.asarray(tcp_pose, dtype=float), 4).tolist()}")
                print(
                    "  Board tvec: "
                    + str(np.round(np.asarray(tvec, dtype=float).reshape(-1), 4).tolist())
                )

            if key == ord("q"):
                if len(robot_rotations) < args.min_samples:
                    print(
                        f"Need at least {args.min_samples} captures before solving; currently have {len(robot_rotations)}."
                    )
                    continue
                break

        print("Solving hand-eye calibration...")
        transform = solve_extrinsics(
            robot_rotations,
            robot_translations,
            board_rotations,
            board_translations,
            args.calibration_setup,
        )

        print("\n=== Extrinsic Result ===")
        print(f"{RESULT_LABELS[args.calibration_setup]}:")
        print(np.round(transform, 4))

        save_outputs(
            args.output_path,
            args.samples_output,
            transform,
            robot_rotations,
            robot_translations,
            board_rotations,
            board_translations,
        )
        return 0

    finally:
        if pipeline_started and pipeline is not None:
            pipeline.stop()
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass
        if rtde_r is not None and hasattr(rtde_r, "disconnect"):
            try:
                rtde_r.disconnect()
            except Exception:
                pass


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nInterrupted by user.", file=sys.stderr)
        raise SystemExit(1)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise