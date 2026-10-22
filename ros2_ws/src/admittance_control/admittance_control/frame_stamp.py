"""The capture time of a RealSense frame on the host clock, for the images' header.stamp.

Until 2026-10-22 scripts/realsense_camera_node.py stamped every image with `now()` once
`wait_for_frames` had returned and the depth had been aligned: the PUBLISH time, later
than the exposure by the readout, the USB transfer, the frame queue and the alignment.
Everything that looks the camera pose up at an image's stamp - the aim's latency
compensation (camera_aim.py), the tracker's ego-motion (fp_tracker_node.py), the TF of
a captured frame - got the arm where it was that much later. On a still arm that changes
nothing; on a moving one the pose was late.

librealsense times each frame itself, `frame.get_timestamp()` in ms, in one of three
domains:
  * global_time:    the device clock at the start of the frame's readout (the
                    FRAME_TIMESTAMP metadata), converted to the host clock by
                    librealsense's running device-to-host fit (`global_time_enabled`,
                    which the node switches on);
  * system_time:    the host clock when the frame ARRIVED (no metadata): after the
                    transfer, still earlier than the publish;
  * hardware_clock: the device clock alone, not on the host clock: not used.
`mid_exposure` moves a global-time stamp further back, from the start of readout to the
middle of the exposure, by the gap between the two device-clock metadata
(FRAME_TIMESTAMP minus SENSOR_TIMESTAMP, the mid-exposure time, both in us): the instant
the image shows. A stamp that is not plausible - in the future, or older than
`max_offset_s` (a clock fit not converged yet, a stalled stream) - falls back to `now`
and says why.

Nothing here imports pyrealsense2 or ROS: the node reads a frame into `FrameTimes`.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np

SOURCES = ("mid_exposure", "frame", "now")
#: a readout - mid-exposure gap beyond this is a wrapped counter or junk metadata
MAX_READOUT_GAP_US = 200_000
#: a stamp this far in the future is a bad clock fit, not jitter
MAX_FUTURE_S = 0.01


@dataclass(frozen=True)
class FrameTimes:
    """What a frame says about its own time."""
    timestamp_ms: float                  # frame.get_timestamp()
    domain: str                          # 'global_time', 'system_time' or 'hardware_clock'
    frame_ts_us: Optional[int] = None    # FRAME_TIMESTAMP: start of readout, device clock
    sensor_ts_us: Optional[int] = None   # SENSOR_TIMESTAMP: mid-exposure, device clock


def readout_gap_ms(ft: FrameTimes) -> Optional[float]:
    """Start of readout minus mid-exposure (ms), from the device-clock metadata; None when
    either is missing or the gap is not plausible."""
    if ft.frame_ts_us is None or ft.sensor_ts_us is None:
        return None
    d = int(ft.frame_ts_us) - int(ft.sensor_ts_us)
    return d / 1000.0 if 0 <= d <= MAX_READOUT_GAP_US else None


def capture_stamp_ns(ft: FrameTimes, now_ns: int, source: str = "mid_exposure",
                     max_offset_s: float = 0.5) -> tuple[int, str]:
    """(header stamp in ns on the host clock, how it was obtained) for one frame.

    `how` is 'mid-exposure', 'readout' (global time, no usable exposure metadata),
    'arrival' (system time), 'now' (asked for), or 'now: <why>' when the frame's own
    time was not usable."""
    if source not in SOURCES:
        raise ValueError(f"stamp source {source!r}: expected one of {SOURCES}")
    if source == "now":
        return int(now_ns), "now"
    if ft.domain == "global_time":
        t_ms, how = float(ft.timestamp_ms), "readout"
        gap = readout_gap_ms(ft) if source == "mid_exposure" else None
        if gap is not None:
            t_ms, how = t_ms - gap, "mid-exposure"
    elif ft.domain == "system_time":
        t_ms, how = float(ft.timestamp_ms), "arrival"
    else:
        return int(now_ns), f"now: the frame is timed by the {ft.domain.replace('_', ' ')} only"
    stamp = int(round(t_ms * 1e6))
    age_s = (int(now_ns) - stamp) * 1e-9
    if age_s < -MAX_FUTURE_S or age_s > max_offset_s:
        return int(now_ns), f"now: the frame's {how} time is {age_s * 1000.0:+.0f} ms from now"
    return stamp, how


def age_summary(ages_ms: Sequence[float]) -> str:
    """'median 41 ms, p95 55 ms (300 frames)' of the stamps' age at publish."""
    if not len(ages_ms):
        return "no frames"
    a = np.asarray(ages_ms, float)
    return (f"median {np.median(a):.0f} ms, p95 {np.percentile(a, 95):.0f} ms "
            f"({len(a)} frames)")
