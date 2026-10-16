#!/usr/bin/env python3
"""How much the camera aim buzzes the wrist, from a rosbag: one line of numbers per bag.

The aim's velocity commands reach the UR as steps; each step is a torque kick, and a
stream of them at 50 Hz is the buzz heard on 2026-10-15 (notes/thesis_notes.md,
2026-10-16). This reads `/joint_states` (500 Hz, with effort) and
`/forward_velocity_controller/commands` and reports, for the aiming part of the bag:

  * the command rate and the size of the jump between consecutive commands (wrist_1);
  * the buzz: the 20-250 Hz part of the wrist efforts while aiming, against the same
    measure while the trajectory controller holds the arm (idle) - the ratio is the
    buzz index, 1 = as quiet as standing still;
  * a vibration LINE: the wrist_1 velocity frequency above 30 Hz that stands out most
    from its NEIGHBOURS (x the median of the bins 8-24 Hz either side); below 3x it is
    reported as no line. Against the whole spectrum the slope at the high-pass edge
    (~20-30 Hz) looked like a peak.

    source /opt/ros/jazzy/setup.bash
    /usr/bin/python3 scripts/aim_bag_report.py <bag_dir> [<bag_dir> ...]
    /usr/bin/python3 scripts/aim_bag_report.py <bag_dir> --npz out.npz    # arrays, for plots

Record a bag to compare (with the aim running, point and move around near the centre):
    ros2 bag record /joint_states /forward_velocity_controller/commands \\
        /camera_aim/joint_rates /camera_aim/status

The first bag, the 50 Hz direct stream: ros2_ws/rosbag2_2026_10_16-13_49_03.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

UR = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
      'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
FS = 500.0
WINDOW_S = 0.5


def read_bag(path: str) -> dict:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    meta = Path(path) / 'metadata.yaml'
    storage = 'mcap' if 'mcap' in meta.read_text() else 'sqlite3'
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=storage),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    js_t, js_v, js_e, cmd_t, cmd, in_t = [], [], [], [], [], []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == '/joint_states':
            m = deserialize_message(data, get_message(types[topic]))
            idx = [m.name.index(n) for n in UR]
            js_t.append(t * 1e-9)
            js_v.append([m.velocity[i] for i in idx])
            js_e.append([m.effort[i] for i in idx] if len(m.effort) == len(m.name) else [np.nan] * 6)
        elif topic == '/forward_velocity_controller/commands':
            m = deserialize_message(data, get_message(types[topic]))
            cmd_t.append(t * 1e-9)
            cmd.append(list(m.data))
        elif topic == '/camera_aim/joint_rates':
            in_t.append(t * 1e-9)
    t0 = js_t[0] if js_t else 0.0
    return {'js_t': np.asarray(js_t) - t0, 'js_v': np.asarray(js_v), 'js_e': np.asarray(js_e),
            'cmd_t': np.asarray(cmd_t) - t0, 'cmd': np.asarray(cmd).reshape(-1, 6),
            'in_t': np.asarray(in_t) - t0}


def _highpass(x: np.ndarray, n: int = 25) -> np.ndarray:
    """x minus its 50 ms moving average: what is left is above ~20 Hz."""
    return (x - np.convolve(x, np.ones(n) / n, mode='same'))[n:-n]


def report(d: dict) -> dict:
    jt, ct, c = d['js_t'], d['cmd_t'], d['cmd']
    if len(ct) < 10:
        raise SystemExit('no velocity commands in this bag: was the aim live?')
    # aiming = windows with a non-zero command; idle = windows with no command at all
    edges = np.arange(0.0, jt[-1], WINDOW_S)
    aim_w, idle_w = [], []
    for a in edges:
        mc = (ct >= a) & (ct < a + WINDOW_S)
        mj = (jt >= a) & (jt < a + WINDOW_S)
        if mj.sum() < 0.8 * FS * WINDOW_S:
            continue
        if mc.sum() == 0:
            idle_w.append(mj)
        elif np.abs(c[mc]).max() > 0:
            aim_w.append(mj)

    def effort_rms(masks):
        vals = [np.std(_highpass(d['js_e'][m, j])) for m in masks for j in (3, 4, 5)]
        return float(np.nanmean(vals)) if vals else float('nan')

    buzz_aim, buzz_idle = effort_rms(aim_w), effort_rms(idle_w)
    steps = np.abs(np.diff(c[:, 3]))
    moving = np.abs(c[1:, 3]) > 0
    # the strongest vibration line of the wrist_1 velocity while aiming
    segs = []
    for m in aim_w:
        x = _highpass(d['js_v'][m, 3])
        if len(x) >= 128:
            segs.append(np.abs(np.fft.rfft(x[:128] * np.hanning(128))) ** 2)
    f = np.fft.rfftfreq(128, 1 / FS)
    peak_hz, peak_x = float('nan'), float('nan')
    if segs:
        P = np.mean(segs, 0)
        best = (float('nan'), 0.0)
        for k in np.nonzero(f > 30)[0]:
            near = [i for i in range(k - 6, k + 7) if 0 <= i < len(P) and abs(i - k) > 1]
            x = float(P[k] / np.median(P[near]))
            if x > best[1]:
                best = (float(f[k]), x)
        peak_hz, peak_x = best
    return {
        'aiming_s': len(aim_w) * WINDOW_S, 'idle_s': len(idle_w) * WINDOW_S,
        'cmd_hz': 1.0 / float(np.median(np.diff(ct))),
        'in_hz': (1.0 / float(np.median(np.diff(d['in_t'])))) if len(d['in_t']) > 10 else float('nan'),
        'step_median': float(np.median(steps[moving])) if moving.any() else 0.0,
        'step_p95': float(np.percentile(steps[moving], 95)) if moving.any() else 0.0,
        'buzz_aim': buzz_aim, 'buzz_idle': buzz_idle, 'buzz_index': buzz_aim / buzz_idle,
        'peak_hz': peak_hz, 'peak_x': peak_x,
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--npz', help='save the arrays of the (first) bag for plotting')
    args = ap.parse_args()
    print(f"{'bag':<34} {'aim s':>6} {'cmd Hz':>7} {'in Hz':>6} {'step med':>9} {'step p95':>9} "
          f"{'effort HF aim':>13} {'idle':>8} {'BUZZ x':>7} {'line':>14}")
    for i, bag in enumerate(args.bags):
        d = read_bag(bag)
        r = report(d)
        print(f"{Path(bag).name:<34} {r['aiming_s']:6.1f} {r['cmd_hz']:7.0f} {r['in_hz']:6.0f} "
              f"{r['step_median']:9.5f} {r['step_p95']:9.5f} {r['buzz_aim']:13.4f} "
              f"{r['buzz_idle']:8.4f} {r['buzz_index']:7.1f} "
              + (f"{r['peak_hz']:6.1f} Hz x{r['peak_x']:<4.0f}" if r['peak_x'] >= 3 else f"{'none':>14}"))
        if args.npz and i == 0:
            np.savez(args.npz, **d)
            print(f'  arrays -> {args.npz}')
    print("step: |jump| between consecutive wrist_1 commands (rad/s); effort HF: the 20-250 Hz "
          "wrist effort RMS; BUZZ x: aiming / idle (1 = as quiet as standing still); line: the "
          "wrist_1 velocity line above 30 Hz, x its neighbours (none below 3x)")


if __name__ == '__main__':
    main()
