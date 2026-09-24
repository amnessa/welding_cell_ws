"""The capture script's board detection on this machine's OpenCV. Plain pytest.

Claim: a ChArUco board rendered at a KNOWN pose in front of a synthetic pinhole camera
is detected and its pose recovered to millimetres and a fraction of a degree, through
`detect_board_pose` - the version-independent path (OpenCV 5 dropped the legacy ArUco
functions the script used to call).
"""

from __future__ import annotations

import pathlib
import sys
import types

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
PKG = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG / "helper" / "calibration"))
pytest.importorskip("pyrealsense2", reason="extract_extrinsics imports pyrealsense2 at module level")
pytest.importorskip("rtde_receive", reason="extract_extrinsics imports ur_rtde at module level")
import extract_extrinsics as ex  # noqa: E402


def _render_board_at(board, K, rvec, tvec, size=(1280, 720)):
    """Warp the board's own image into the camera view of a board at (rvec, tvec)."""
    px = 1000
    n_x, n_y = board.getChessboardSize()
    sq = board.getSquareLength()
    img = board.generateImage((int(n_x * sq * px), int(n_y * sq * px)), marginSize=0, borderBits=1)
    # board plane: image pixel (u, v) <-> board point (u/px, v/px, 0) ... the generated
    # image has y DOWN while the board's y axis is UP in OpenCV's ChArUco convention
    h_img, w_img = img.shape[:2]
    src = np.float32([[0, 0], [w_img, 0], [w_img, h_img], [0, h_img]])
    obj = np.float32([[0, n_y * sq, 0], [n_x * sq, n_y * sq, 0], [n_x * sq, 0, 0], [0, 0, 0]])
    dst, _ = cv2.projectPoints(obj, rvec, tvec, K, None)
    H = cv2.getPerspectiveTransform(src, dst.reshape(-1, 2).astype(np.float32))
    return cv2.warpPerspective(img, H, size, borderValue=200)


def test_board_pose_is_recovered_on_this_opencv():
    args = types.SimpleNamespace(dictionary="DICT_4X4_250", squares_x=4, squares_y=4,
                                 square_length=0.030, marker_length=0.022, legacy_pattern=True)
    board, detector = ex.create_board(args)
    K = np.array([[900.0, 0, 640.0], [0, 900.0, 360.0], [0, 0, 1.0]])
    rvec = np.array([[0.15], [-0.25], [0.1]])
    tvec = np.array([[-0.05], [-0.04], [0.45]])
    gray = _render_board_at(board, K, rvec, tvec)
    corners, ids, ch_c, ch_ids, r, t, ok = ex.detect_board_pose(gray, board, detector, K, np.zeros(5))
    assert ok and ids is not None and len(ids) >= 4 and len(ch_ids) >= 4
    assert np.linalg.norm(t.reshape(3) - tvec.reshape(3)) < 3e-3            # < 3 mm at 0.45 m
    R_est, _ = cv2.Rodrigues(r); R_true, _ = cv2.Rodrigues(rvec)
    ang = np.degrees(np.arccos(np.clip((np.trace(R_true.T @ R_est) - 1) / 2, -1, 1)))
    assert ang < 0.5
