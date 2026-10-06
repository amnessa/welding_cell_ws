"""GestureLatch with the FoundationPose mapping (2026-10-06): Closed_Fist moves the
robot (refine_pose) and must be HELD, Open_Palm only recomputes seams.

Imports gesture_control_node (rclpy, cv2; MediaPipe is imported only when the node
builds its recognizer, so it is not needed here)."""

import pathlib
import sys
import time

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("cv2")
PKG = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))
sys.path.insert(0, str(PKG / "scripts"))

import gesture_control_node as g  # noqa: E402


def _feed(latch, gesture, frames, dt=0.0):
    fired = []
    for _ in range(frames):
        a = latch.update(gesture)
        if a:
            fired.append(a)
        if dt:
            time.sleep(dt)
    return fired


def test_fist_must_be_held_before_refine_pose_fires():
    latch = g.GestureLatch(stable_frames=3, cooldown_sec=0.0,
                           actions=dict(g.FP_GESTURE_TO_ACTION), hold_sec={'Closed_Fist': 0.3})
    assert _feed(latch, 'Closed_Fist', 6) == []                 # stable, but held ~0 s
    assert latch.progress() < 1.0
    time.sleep(0.32)
    assert _feed(latch, 'Closed_Fist', 1) == ['REFINE_POSE']    # held long enough
    assert _feed(latch, 'Closed_Fist', 10, dt=0.01) == []       # fires once per hold


def test_a_fist_that_only_passes_by_does_not_move_the_robot():
    latch = g.GestureLatch(stable_frames=2, cooldown_sec=0.0,
                           actions=dict(g.FP_GESTURE_TO_ACTION), hold_sec={'Closed_Fist': 0.3})
    fired = _feed(latch, 'Closed_Fist', 5, dt=0.02)             # ~0.1 s of fist
    fired += _feed(latch, None, 2)
    fired += _feed(latch, 'Closed_Fist', 5, dt=0.02)            # again, the hold restarted
    assert fired == []


def test_palm_is_quick_and_maps_to_welding_points_or_undo():
    latch = g.GestureLatch(stable_frames=3, cooldown_sec=0.0,
                           actions=dict(g.FP_GESTURE_TO_ACTION), hold_sec={'Closed_Fist': 2.0})
    assert _feed(latch, 'Open_Palm', 3) == ['UNDO_OR_WELD']


def test_icp_mapping_is_unchanged():
    latch = g.GestureLatch(stable_frames=3, cooldown_sec=0.0)
    assert _feed(latch, 'Closed_Fist', 3) == ['RUN_ICP']
    latch = g.GestureLatch(stable_frames=3, cooldown_sec=0.0)
    assert _feed(latch, 'Open_Palm', 3) == ['STOP_ICP']
