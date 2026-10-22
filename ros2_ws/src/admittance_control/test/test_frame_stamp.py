"""The camera image's stamp is the frame's capture time (admittance_control/frame_stamp.py).
Plain pytest, no camera, no ROS.

Claims: a global-time frame is stamped at the middle of its exposure (its start of
readout minus the device's readout - mid-exposure gap), or at the start of readout when
that metadata is missing or implausible; a system-time frame at its arrival; a frame on
the device clock alone, one in the future or one older than the limit falls back to now
and says why; asking for 'now' gives the old publish-time stamp.
"""

from __future__ import annotations

import pathlib
import sys

import pytest

PKG = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

from admittance_control.frame_stamp import (FrameTimes, age_summary,  # noqa: E402
                                            capture_stamp_ns, readout_gap_ms)

NOW_NS = 1_760_000_000_000_000_000                       # 2025-10-09, ns since the epoch
NOW_MS = NOW_NS / 1e6


def _global(age_ms, gap_us=None, frame_us=5_000_000):
    return FrameTimes(NOW_MS - age_ms, "global_time", frame_us,
                      None if gap_us is None else frame_us - gap_us)


def test_global_time_is_moved_back_to_mid_exposure():
    stamp, how = capture_stamp_ns(_global(40.0, gap_us=12_500), NOW_NS)
    assert how == "mid-exposure"
    assert (NOW_NS - stamp) / 1e6 == pytest.approx(52.5, abs=1e-3)   # 40 ms + the 12.5 ms gap


def test_without_exposure_metadata_the_start_of_readout_is_used():
    stamp, how = capture_stamp_ns(_global(40.0), NOW_NS)
    assert how == "readout" and (NOW_NS - stamp) / 1e6 == pytest.approx(40.0, abs=1e-3)
    stamp, how = capture_stamp_ns(_global(40.0, gap_us=12_500), NOW_NS, source="frame")
    assert how == "readout" and (NOW_NS - stamp) / 1e6 == pytest.approx(40.0, abs=1e-3)


@pytest.mark.parametrize("gap_us", [-300, 250_000])
def test_an_implausible_readout_gap_is_not_applied(gap_us):
    # a wrapped 32-bit us counter or junk metadata
    assert readout_gap_ms(_global(40.0, gap_us=gap_us)) is None
    assert capture_stamp_ns(_global(40.0, gap_us=gap_us), NOW_NS)[1] == "readout"


def test_system_time_is_the_arrival():
    stamp, how = capture_stamp_ns(FrameTimes(NOW_MS - 15.0, "system_time", 1, 0), NOW_NS)
    assert how == "arrival" and (NOW_NS - stamp) / 1e6 == pytest.approx(15.0, abs=1e-3)


@pytest.mark.parametrize("ft,why", [
    (FrameTimes(123456.7, "hardware_clock"), "hardware clock only"),
    (FrameTimes(NOW_MS + 50.0, "global_time"), "+-50 ms from now"),    # in the future
    (FrameTimes(NOW_MS - 800.0, "global_time"), "+800 ms from now"),   # too old
])
def test_an_unusable_frame_time_falls_back_to_now_and_says_why(ft, why):
    stamp, how = capture_stamp_ns(ft, NOW_NS, max_offset_s=0.5)
    assert stamp == NOW_NS and how.startswith("now: ")
    assert why.replace("+-", "-") in how


def test_now_is_the_old_publish_time_stamp():
    assert capture_stamp_ns(_global(40.0, gap_us=12_500), NOW_NS, source="now") == (NOW_NS, "now")
    with pytest.raises(ValueError):
        capture_stamp_ns(_global(40.0), NOW_NS, source="exposure_start")


def test_age_summary():
    assert age_summary([]) == "no frames"
    assert age_summary([40.0] * 19 + [100.0]) == "median 40 ms, p95 43 ms (20 frames)"
