#!/usr/bin/env python3
"""WeldSet tier-1 benchmark viewer: browse `bench_phase4/` scene by scene.

`bench_phase4` holds 720 analytic weld scenes in 12 strata (60 each). Every scene is two
workpieces in weld position, sampled as a point cloud, with the weld seams CONSTRUCTED from
the geometry (never annotated). This viewer shows, per scene:

  * the point cloud in 3D, under one of three input conditions:
      exterior     what a perfect multi-view scan returns (benchmark Task 1 input)
      camera view  what the scene's single depth camera sees (Task 2 input)
      all samples  every surface sample, including faces buried at the joint (truth only)
  * the ground-truth seams: primary seams (weldable, of the joint's own class) in green,
    other weldable seams in amber, rejected candidates dotted grey; the seam named by the
    "most promising seam" rule (Task 2's target) is marked MPS; tack welds are black crosses
  * the single camera's view as a depth image, with the seams projected onto it
  * the scene's parameters (ISO 17659 joint term, ISO 9692-1 preparation, ISO 5817 quality
    level, gap, angle, plate thickness, sensor) and a table of every seam candidate

Requirements: Python 3.9+, numpy, matplotlib   (pip install numpy matplotlib)
Optional:     open3d, to open the current scene in a smooth 3D viewer (key `o`)

Run it from the folder that contains `bench_phase4/` (or pass --root):

    python view_bench_phase4.py
    python view_bench_phase4.py --stratum T/circle --start 10
    python view_bench_phase4.py --list                 # print the catalogue and exit
    python view_bench_phase4.py --save-all pngs/       # one PNG per scene, no window

Controls: the slider and the Prev / Next buttons move through the scenes; the panels on the
left pick the stratum, the input condition and the colouring. Keys:
    left / right   previous / next scene         up / down   previous / next stratum
    1 2 3          exterior / camera view / all samples
    c              cycle colouring (part, face, seen by camera)
    k              show / hide the camera position     r   look from the camera again
    + -            more / fewer points drawn            s   save the figure as PNG
    o              open the scene in Open3D (if installed)
    h              help                                 q   quit
Drag in the 3D panel to rotate, right-drag to zoom.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent

STRATA = ["T/line", "T/circle", "T/ellipse", "T/saddle", "T/rounded_rect", "T/swept_path",
          "corner/line", "butt/line", "butt/line_grooved", "butt/arc_butt", "lap/line", "edge/line"]
DESCRIBE = {
    "T/line": "T-joint of two plates, straight seams (incl. angled T)",
    "T/circle": "pipe standing on a plate, circular seam",
    "T/ellipse": "inclined pipe on a plate, elliptical seam",
    "T/saddle": "pipe branching from a pipe, saddle seam",
    "T/rounded_rect": "rectangular tube on a plate, rounded-rectangle seam",
    "T/swept_path": "curved strip standing on a plate, arc / B-spline seam",
    "corner/line": "corner joint of two plates",
    "butt/line": "square butt joint of two plates",
    "butt/line_grooved": "grooved butt joint (ISO 9692-1 single V / U / bevel)",
    "butt/arc_butt": "butt joint of two curved strips, arc seam",
    "lap/line": "lap joint (overlapping plates)",
    "edge/line": "edge joint (stacked plates, edges flush)",
}
CONDITIONS = ["exterior", "camera view", "all samples"]
COLOURINGS = ["part", "face", "seen by camera"]

PART = {0: "#2a78d6", 1: "#eb6834", 255: "#a8a7a2"}           # part A, part B, fixture
PART_NAME = {0: "part A", 1: "part B", 255: "fixture"}
SEAM_PRIMARY, SEAM_OTHER, SEAM_REJECTED = "#008300", "#eda100", "#8a8984"
INK, INK2 = "#0b0b0b", "#52514e"
FIXTURE_ID = 255


# --------------------------------------------------------------------------- catalogue
def stratum_of(scene: dict) -> str:
    j = scene["joint"]
    fam = j.get("seam_family")
    if not fam:
        fam = "line_grooved" if j.get("prep", "square") != "square" else "line"
    return f"{j['type']}/{fam}"


def build_catalogue(root: Path) -> list[dict]:
    """One entry per scene folder under root/<joint type>/<scene id>/, in stratum order."""
    entries = []
    for f in sorted(root.glob("*/*/scene.json")):
        s = json.loads(f.read_text())
        st = stratum_of(s)
        entries.append({"dir": f.parent, "scene": s, "stratum": st, "seed": int(s["seed"]),
                        "scene_id": s["scene_id"], "twin_key": s.get("twin_key")})
    order = {s: i for i, s in enumerate(STRATA)}
    entries.sort(key=lambda e: (order.get(e["stratum"], 99), e["seed"]))
    return entries


def load_scene(entry: dict) -> dict:
    s = entry["scene"]
    d = entry["dir"]
    with np.load(d / "cloud.npz") as z:
        cloud = {k: z[k] for k in z.files}
    seams = []
    with np.load(d / "seams.npz") as z:
        for m in s["seams"]:
            key = m["sampled"]["array"]
            if key not in z.files:
                continue
            vis = z[key + "_visible"] if key + "_visible" in z.files else None
            seams.append({**m, "xyz": z[key].astype(float), "visible": vis})
    tacks = np.asarray((s.get("tacks") or {}).get("points_mm") or [], float).reshape(-1, 3)
    mps = (s.get("mps") or {}).get("seam_id")
    return {"scene": s, "cloud": cloud, "seams": seams, "tacks": tacks, "mps": mps}


def is_primary(m: dict) -> bool:
    return bool(m["weldable"]) and bool(m.get("matches_joint_type", True))


# --------------------------------------------------------------------------- geometry
def condition_mask(cloud: dict, condition: str) -> np.ndarray:
    if condition == "exterior":
        return cloud["exterior"].astype(bool)
    if condition == "camera view":
        return cloud["visible_from_cam"].astype(bool)
    return np.ones(len(cloud["xyz"]), bool)


def workpiece_box(sc: dict, margin: float = 0.25):
    """Axis-aligned box around the workpieces (the fixture table is far larger)."""
    xyz, oid = sc["cloud"]["xyz"], sc["cloud"]["object_id"]
    P = xyz[oid != FIXTURE_ID]
    lo, hi = P.min(0), P.max(0)
    pad = margin * (hi - lo).max()
    return lo - pad, hi + pad


def pick_points(sc: dict, condition: str, budget: int, seed: int = 0) -> np.ndarray:
    cloud = sc["cloud"]
    m = condition_mask(cloud, condition)
    if (cloud["object_id"] == FIXTURE_ID).any():
        lo, hi = workpiece_box(sc)
        m &= np.all((cloud["xyz"] >= lo) & (cloud["xyz"] <= hi), axis=1)
    idx = np.flatnonzero(m)
    if len(idx) > budget:
        idx = np.sort(np.random.default_rng(seed).choice(idx, budget, replace=False))
    return idx


def camera_of(scene: dict):
    cam = scene["camera"]
    T = np.asarray(cam["T_world_cam"], float)
    K = np.asarray(cam["K"], float)
    return T, K, int(cam["width"]), int(cam["height"])


def project(xyz: np.ndarray, T: np.ndarray, K: np.ndarray):
    cam = (np.asarray(xyz, float) - T[:3, 3]) @ T[:3, :3]        # world -> camera (OpenCV axes)
    z = cam[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        uv = cam[:, :2] / z[:, None] * np.array([K[0, 0], K[1, 1]]) + np.array([K[0, 2], K[1, 2]])
    return uv, z


def depth_image(sc: dict, scale: int = 4) -> np.ndarray:
    """Z-buffer of the camera-visible samples at 1/scale resolution (mm, NaN = no return)."""
    T, K, W, H = camera_of(sc["scene"])
    cloud = sc["cloud"]
    P = cloud["xyz"][cloud["visible_from_cam"].astype(bool)]
    uv, z = project(P, T, K)
    w, h = W // scale, H // scale
    u = np.floor(uv[:, 0] / scale).astype(int)
    v = np.floor(uv[:, 1] / scale).astype(int)
    ok = (z > 0) & (u >= 0) & (u < w) & (v >= 0) & (v < h)
    img = np.full(w * h, np.inf)
    np.minimum.at(img, v[ok] * w + u[ok], z[ok])
    img[~np.isfinite(img)] = np.nan
    return fill_holes(img.reshape(h, w))


def fill_holes(img: np.ndarray) -> np.ndarray:
    """Fill empty pixels that lie between returns (sparse samples at 1/4 resolution leave
    speckle); a pixel is filled only when >= 4 of its 8 neighbours have a return, so the
    background stays empty."""
    pad = np.pad(img, 1, constant_values=np.nan)
    H, W = img.shape
    stack = np.stack([pad[1 + dy:H + 1 + dy, 1 + dx:W + 1 + dx] for dy in (-1, 0, 1) for dx in (-1, 0, 1)])
    ok = np.isfinite(stack)
    near = np.where(ok, stack, np.inf).min(0)
    holes = np.isnan(img) & (ok.sum(0) >= 4)
    out = img.copy()
    out[holes] = near[holes]
    return out


def default_view(sc: dict, show_camera: bool):
    """Look from the sensor; with the camera drawn, swing 75 deg sideways so it is visible."""
    el, az = look_from_camera(sc)
    return (20.0, az + 75.0) if show_camera else (el, az)


def look_from_camera(sc: dict):
    """(elev, azim) for matplotlib's view_init so the 3D panel looks from the sensor."""
    T, _, _, _ = camera_of(sc["scene"])
    lo, hi = workpiece_box(sc, 0.0)
    d = T[:3, 3] - 0.5 * (lo + hi)
    d /= np.linalg.norm(d)
    return float(np.degrees(np.arcsin(np.clip(d[2], -1, 1)))), float(np.degrees(np.arctan2(d[1], d[0])))


# --------------------------------------------------------------------------- drawing
def point_colours(sc: dict, idx: np.ndarray, colouring: str) -> np.ndarray:
    """(n, 4) RGBA array - numeric, so matplotlib does not parse one colour string per point."""
    import matplotlib
    from matplotlib.colors import to_rgba
    cloud = sc["cloud"]
    if colouring == "part":
        lut = np.tile(to_rgba("#a8a7a2"), (256, 1))
        for k, c in PART.items():
            lut[k] = to_rgba(c)
        return lut[cloud["object_id"][idx].astype(int)]
    if colouring == "face":
        cmap = matplotlib.colormaps["tab20"] if hasattr(matplotlib, "colormaps") else matplotlib.cm.get_cmap("tab20")
        return cmap((cloud["face_id"][idx].astype(int) % 20) / 19.0)
    seen = cloud["visible_from_cam"][idx].astype(bool)
    return np.where(seen[:, None], to_rgba("#1c5cab"), to_rgba("#d5d4cf"))


def draw_3d(ax, sc: dict, condition: str, colouring: str, budget: int, show_camera: bool,
            view=None, title: str | None = None):
    from matplotlib.lines import Line2D
    ax.cla()
    idx = pick_points(sc, condition, budget)
    P = sc["cloud"]["xyz"][idx]
    ax.scatter(P[:, 0], P[:, 1], P[:, 2], s=0.7, c=point_colours(sc, idx, colouring),
               alpha=0.55, linewidths=0, depthshade=False, rasterized=True)
    for m in sc["seams"]:
        X = m["xyz"]
        if is_primary(m):
            kw = dict(color=SEAM_PRIMARY, linewidth=2.6 if m["id"] == sc["mps"] else 2.0)
        elif m["weldable"]:
            kw = dict(color=SEAM_OTHER, linewidth=1.4, linestyle="--")
        else:
            kw = dict(color=SEAM_REJECTED, linewidth=0.9, linestyle=":")
        ax.plot(X[:, 0], X[:, 1], X[:, 2], **kw)
        if m["id"] == sc["mps"]:
            c = X[len(X) // 2]
            ax.text(c[0], c[1], c[2], "  MPS", color=INK, fontsize=8, fontweight="bold")
    if len(sc["tacks"]):
        t = sc["tacks"]
        ax.scatter(t[:, 0], t[:, 1], t[:, 2], marker="x", s=28, color=INK, depthshade=False)
    lo, hi = workpiece_box(sc, 0.08)
    ctr, half = 0.5 * (lo + hi), 0.5 * (hi - lo).max()
    if show_camera:
        T, K, W, H = camera_of(sc["scene"])
        C = T[:3, 3]
        dist = np.linalg.norm(C - ctr)
        rays = [np.linalg.inv(K) @ np.array([u, v, 1.0]) for u, v in ((0, 0), (W, 0), (W, H), (0, H))]
        corners = [C + T[:3, :3] @ (r / r[2]) * 0.35 * dist for r in rays]
        for a, b in zip(corners, corners[1:] + corners[:1]):
            ax.plot(*np.c_[a, b], color=INK2, linewidth=0.8)
        for a in corners:
            ax.plot(*np.c_[C, a], color=INK2, linewidth=0.8)
        ax.scatter(*C, color=INK, s=30, marker="s", depthshade=False)
        ax.plot(*np.c_[C, ctr], color=INK2, linewidth=0.6, linestyle=":")
        allp = np.vstack([lo, hi, C])
        ctr, half = 0.5 * (allp.min(0) + allp.max(0)), 0.5 * np.ptp(allp, axis=0).max()
    ax.set_xlim(ctr[0] - half, ctr[0] + half); ax.set_ylim(ctr[1] - half, ctr[1] + half)
    ax.set_zlim(ctr[2] - half, ctr[2] + half)
    try:
        ax.set_box_aspect((1, 1, 1))
    except AttributeError:
        pass
    ax.set_xlabel("x (mm)", fontsize=7, color=INK2); ax.set_ylabel("y (mm)", fontsize=7, color=INK2)
    ax.set_zlabel("z (mm)", fontsize=7, color=INK2); ax.tick_params(labelsize=6, colors=INK2)
    if view is not None:
        ax.view_init(*view)
    oids = sorted(set(int(o) for o in np.unique(sc["cloud"]["object_id"])))
    handles = [Line2D([], [], marker="o", linestyle="", color=PART[o], label=PART_NAME[o]) for o in oids
               if colouring == "part" and o in PART]
    if colouring == "seen by camera":
        handles += [Line2D([], [], marker="o", linestyle="", color="#1c5cab", label="seen by camera"),
                    Line2D([], [], marker="o", linestyle="", color="#d5d4cf", label="not seen")]
    handles += [Line2D([], [], color=SEAM_PRIMARY, linewidth=2, label="primary seam (truth)"),
                Line2D([], [], color=SEAM_OTHER, linewidth=1.4, linestyle="--", label="other weldable seam"),
                Line2D([], [], color=SEAM_REJECTED, linewidth=0.9, linestyle=":", label="rejected candidate"),
                Line2D([], [], marker="x", linestyle="", color=INK, label="tack weld")]
    ax.legend(handles=handles, loc="upper left", fontsize=7, frameon=False)
    ax.set_title(title or f"point cloud · {condition} · {len(idx):,} of {int(condition_mask(sc['cloud'], condition).sum()):,} points drawn",
                 loc="left", fontsize=9, color=INK)


def draw_camera_image(ax, sc: dict, title: str | None = None):
    from matplotlib.colors import LinearSegmentedColormap
    ax.cla()
    img = depth_image(sc)
    cmap = LinearSegmentedColormap.from_list("depth", ["#0d366b", "#1c5cab", "#3987e5", "#86b6ef", "#cde2fb"])
    if hasattr(cmap, "with_extremes"):                  # matplotlib >= 3.4; set_bad is being deprecated
        cmap = cmap.with_extremes(bad="white")
    else:
        cmap.set_bad("white")
    T, K, W, H = camera_of(sc["scene"])
    im = ax.imshow(img, cmap=cmap, extent=(0, W, H, 0), interpolation="nearest")
    for m in sc["seams"]:
        if not is_primary(m):
            continue
        uv, z = project(m["xyz"], T, K)
        vis = m["visible"].astype(bool) if m["visible"] is not None else np.ones(len(uv), bool)
        ax.plot(*np.where(vis[:, None], uv, np.nan).T, color=SEAM_PRIMARY, linewidth=2)
        ax.plot(*np.where(~vis[:, None] & (z[:, None] > 0), uv, np.nan).T, color=SEAM_PRIMARY,
                linewidth=1, linestyle=":")
    vv, uu = np.nonzero(np.isfinite(img))
    if len(uu):                                   # zoom to the returns (far cameras leave the scene small)
        sx, sy = W / img.shape[1], H / img.shape[0]
        u0, u1, v0, v1 = uu.min() * sx, (uu.max() + 1) * sx, vv.min() * sy, (vv.max() + 1) * sy
        cu, cv = 0.5 * (u0 + u1), 0.5 * (v0 + v1)
        half_u = 0.5 * max(u1 - u0, (v1 - v0) * W / H) * 1.15
        half_v = half_u * H / W
        ax.set_xlim(cu - half_u, cu + half_u); ax.set_ylim(cv + half_v, cv - half_v)
    else:
        ax.set_xlim(0, W); ax.set_ylim(H, 0)
    ax.set_xticks([]); ax.set_yticks([])
    for sp in ax.spines.values():
        sp.set_color("#c9c8c3")
    prof = sc["scene"]["noise_model"].get("profile", "?")
    ax.set_title(title or f"camera view ({prof}): depth image, dark = near; primary seams green, dotted = hidden",
                 loc="left", fontsize=8, color=INK)
    return im


def seam_table(sc: dict, max_rows: int = 12) -> str:
    rows = [" id  class     length  closed primary weldable seen*   reject reason"]
    for m in sorted(sc["seams"], key=lambda m: (not is_primary(m), not m["weldable"], m["id"])):
        if len(rows) > max_rows:
            rows.append(f" ... {len(sc['seams']) - max_rows} more candidates")
            break
        vis = m.get("visible_fraction")
        rows.append(f" {m['id']:>2}  {m['seam_class']:<8} {m['length_mm']:6.0f} mm  {'yes' if m.get('closed') else 'no ':<6} "
                    f"{'yes' if is_primary(m) else '-':<7} {'yes' if m['weldable'] else 'no':<8} "
                    f"{(f'{vis:.2f}' if vis is not None else '-'):<7}  {m.get('reject_reason') or '-'}"
                    + ("   <- MPS" if m["id"] == sc["mps"] else ""))
    rows.append(" * seen = fraction of the seam the scene's camera sees.  MPS = the seam the")
    rows.append("   most-promising-seam rule picks (Task 2's target).")
    return "\n".join(rows)


def scene_text(entry: dict, sc: dict, pos: str = "") -> str:
    s = sc["scene"]
    j, fit, cam, nm = s["joint"], s.get("fit", {}), s["camera"], s["noise_model"]
    wp = [o for o in s["objects"] if o["role"] == "workpiece"]
    t = [o.get("thickness_mm") for o in wp if o.get("thickness_mm")]
    cloud = sc["cloud"]
    n_prim = sum(is_primary(m) for m in sc["seams"])
    n_other = sum(m["weldable"] and not is_primary(m) for m in sc["seams"])
    fixture = any(o["role"] == "fixture" for o in s["objects"])
    groove = f" · groove {j['prep']}" if j.get("prep", "square") != "square" else ""
    lines = [
        pos,
        f"stratum    {entry['stratum']}: {DESCRIBE.get(entry['stratum'], '')}",
        f"scene      {s['scene_id']}   (seed {s['seed']})",
        f"joint      {j.get('iso_17659_term', j['type'])} [ISO 17659]{groove}",
        f"quality    level {j.get('quality_level', '?')} [ISO 5817] · root gap {fit.get('root_gap_mm', float('nan')):.2f} mm"
        + (f" · misalignment {fit['linear_misalignment_mm']:.2f} mm" if fit.get("linear_misalignment_mm") else ""),
        f"geometry   {' + '.join(o['primitive'] for o in wp)} · thickness "
        + (f"{min(t):.1f}–{max(t):.1f} mm" if t else "-")
        + f" · included angle {j.get('included_angle_deg', float('nan')):.1f}°",
        f"fixture    {'yes (clamping table, grey)' if fixture else 'no'} · contact mode {j.get('contact_mode', '-')}",
        f"sensor     {nm.get('profile', '?')} · standoff {cam.get('standoff_mm', float('nan')):.0f} mm"
        f" · elevation {cam.get('elevation_deg', float('nan')):.0f}° · {s['cloud']['density_per_mm2']:.2f} samples/mm²",
        f"cloud      {len(cloud['xyz']):,} samples · {int(cloud['exterior'].sum()):,} exterior"
        f" · {int(cloud['visible_from_cam'].sum()):,} seen by the camera",
        f"seams      {n_prim} primary · {n_other} other weldable · {len(sc['seams']) - n_prim - n_other} rejected"
        f" · {len(sc['tacks'])} tack welds",
        "",
        seam_table(sc),
    ]
    return "\n".join(lines)


HELP = """keys:  left / right  previous / next scene      up / down  previous / next stratum
       1 2 3  exterior / camera view / all samples   c  colouring   k  camera on/off
       r  look from the camera   + -  points drawn   s  save PNG   o  Open3D   q  quit
mouse: drag in 3D to rotate, right-drag to zoom; slider and buttons below to move"""


def open_in_open3d(sc: dict, condition: str):
    try:
        import open3d as o3d
    except ImportError:
        print("open3d is not installed (pip install open3d)")
        return
    from matplotlib.colors import to_rgb
    idx = np.flatnonzero(condition_mask(sc["cloud"], condition))
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(sc["cloud"]["xyz"][idx].astype(float)))
    pc.colors = o3d.utility.Vector3dVector(
        np.array([to_rgb(PART.get(int(o), "#a8a7a2")) for o in sc["cloud"]["object_id"][idx]]))
    geoms = [pc]
    for m in sc["seams"]:
        X = m["xyz"]
        ls = o3d.geometry.LineSet(o3d.utility.Vector3dVector(X),
                                  o3d.utility.Vector2iVector(np.c_[np.arange(len(X) - 1), np.arange(1, len(X))]))
        ls.paint_uniform_color(to_rgb(SEAM_PRIMARY if is_primary(m) else SEAM_OTHER if m["weldable"] else SEAM_REJECTED))
        geoms.append(ls)
    o3d.visualization.draw_geometries(geoms, window_name=sc["scene"]["scene_id"])


def radio(ax, labels, active: int = 0):
    """RadioButtons WITHOUT blitting. With blitting (matplotlib's default since 3.7) the dots
    are 'animated' artists drawn only after a completed redraw; on some backends / scaled
    displays, or when redraws pile up, they vanish and the panel looks dead."""
    from matplotlib.widgets import RadioButtons
    try:
        r = RadioButtons(ax, labels, active=active, useblit=False)
    except TypeError:                                   # matplotlib < 3.7 has no useblit
        r = RadioButtons(ax, labels, active=active)
    for t in r.labels:
        t.set_fontsize(8)
    return r


def environment() -> str:
    import platform
    import matplotlib
    return (f"python {platform.python_version()} · numpy {np.__version__} · matplotlib {matplotlib.__version__}"
            f" · backend {matplotlib.get_backend()} · {platform.system()} {platform.release()}")


class Navigator:
    """Coalesces navigation. Keys, buttons and slider moves only record the scene wanted;
    one load-and-draw runs `delay_ms` after the LAST request. Without this, every arrow press
    or slider position loads and draws a scene inside the GUI event handler, and input that
    arrives faster than a draw completes piles up until the window stops responding."""

    def __init__(self, fig, apply, delay_ms: int = 120):
        self.apply, self.want = apply, None
        self.timer = fig.canvas.new_timer(interval=delay_ms)
        try:
            self.timer.single_shot = True
        except AttributeError:
            pass
        self.timer.add_callback(self._fire)

    def request(self, i: int):
        self.want = i
        self.timer.stop()
        self.timer.start()

    def _fire(self):
        self.timer.stop()
        if self.want is not None:
            i, self.want = self.want, None
            self.apply(i)

    def flush(self):                                    # run now (tests, start-up)
        self._fire()


def safe(fn):
    """Run a drawing / event function; on an error print it, and keep the viewer alive."""
    def wrapped(self, *a, **k):
        try:
            return fn(self, *a, **k)
        except Exception:
            import traceback
            tb = traceback.format_exc()
            print(tb, file=sys.stderr)
            try:
                self.axt.cla(); self.axt.axis("off")
                self.axt.text(0, 1, "could not draw this scene - please send this message:\n\n" + tb[-1800:],
                              va="top", family="monospace", fontsize=6.5, color="#d03b3b",
                              transform=self.axt.transAxes)
                self.fig.canvas.draw_idle()
            except Exception:
                pass
    return wrapped


def free_keys():
    import matplotlib.pyplot as plt
    used = {"left", "right", "up", "down", "1", "2", "3", "c", "k", "r", "s", "o", "h", "+", "-", "="}
    for name in list(plt.rcParams):
        if name.startswith("keymap."):
            plt.rcParams[name] = [k for k in plt.rcParams[name] if k not in used]


# --------------------------------------------------------------------------- the viewer
class Viewer:
    def __init__(self, entries: list[dict], start: int = 0, stratum: str = "all", budget: int = 30000,
                 title: str = "WeldSet tier-1 benchmark (bench_phase4)"):
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Button, Slider
        free_keys()
        self.all = entries
        self.budget = budget
        self.condition, self.colouring, self.show_camera = "exterior", "part", False
        self.fig = plt.figure(figsize=(16, 9))
        self.fig.canvas.manager.set_window_title(title) if self.fig.canvas.manager else None
        self.fig.text(0.006, 0.975, title, fontsize=12, fontweight="bold", color=INK)
        self.ax3 = self.fig.add_axes([0.15, 0.09, 0.47, 0.84], projection="3d")
        self.axi = self.fig.add_axes([0.645, 0.56, 0.345, 0.37])
        self.axt = self.fig.add_axes([0.645, 0.07, 0.345, 0.46]); self.axt.axis("off")
        counts = {s: sum(e["stratum"] == s for e in entries) for s in STRATA}
        self.strata = ["all"] + [s for s in STRATA if counts[s]]
        labels = [f"all ({len(entries)})"] + [f"{s} ({counts[s]})" for s in self.strata[1:]]
        self.fig.text(0.006, 0.93, "stratum (scenes)", fontsize=8.5, color=INK2)
        self.r_stratum = radio(self.fig.add_axes([0.004, 0.42, 0.135, 0.5], frameon=False), labels,
                               active=self.strata.index(stratum) if stratum in self.strata else 0)
        self.fig.text(0.006, 0.385, "input condition", fontsize=8.5, color=INK2)
        self.r_cond = radio(self.fig.add_axes([0.004, 0.25, 0.135, 0.13], frameon=False), CONDITIONS)
        self.fig.text(0.006, 0.215, "colour points by", fontsize=8.5, color=INK2)
        self.r_col = radio(self.fig.add_axes([0.004, 0.08, 0.135, 0.13], frameon=False), COLOURINGS)
        self.slider = Slider(self.fig.add_axes([0.20, 0.025, 0.33, 0.025]), "scene", 1, 2, valinit=1, valstep=1,
                             valfmt="%d")
        self.nav = Navigator(self.fig, self.show)
        self.b_prev = Button(self.fig.add_axes([0.555, 0.02, 0.035, 0.035]), "Prev")
        self.b_next = Button(self.fig.add_axes([0.595, 0.02, 0.035, 0.035]), "Next")
        self.fig.text(0.645, 0.012, HELP, fontsize=6.8, color=INK2, family="monospace")
        self.r_stratum.on_clicked(self._on_stratum)
        self.r_cond.on_clicked(lambda lab: self._set(condition=lab))
        self.r_col.on_clicked(lambda lab: self._set(colouring=lab))
        self.slider.on_changed(lambda v: self.nav.request(int(v) - 1))
        self.b_prev.on_clicked(lambda _: self.step(-1))
        self.b_next.on_clicked(lambda _: self.step(+1))
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self.sc = None
        self.fig.text(0.995, 0.975, environment(), fontsize=6, color=INK2, ha="right")
        self._filter(stratum if stratum in self.strata else "all", start)

    # -- navigation
    def _filter(self, stratum: str, start: int = 0):
        self.stratum = stratum
        self.items = self.all if stratum == "all" else [e for e in self.all if e["stratum"] == stratum]
        n = len(self.items)
        self.slider.valmin, self.slider.valmax = 1, max(n, 2)
        self.slider.ax.set_xlim(1, max(n, 2))
        self.i = None
        start = int(np.clip(start, 0, n - 1))
        self.slider.set_val(start + 1)
        self.nav.request(start)
        self.nav.flush()                                # a stratum switch draws at once

    def _on_stratum(self, label: str):
        self._filter(label.split(" (")[0])

    def step(self, d: int):
        cur = self.nav.want if self.nav.want is not None else (self.i or 0)   # held keys accumulate
        self.slider.set_val(int(np.clip(cur + d, 0, len(self.items) - 1)) + 1)

    def _set(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)
        self.render(keep_view=True)

    @safe
    def _on_key(self, ev):
        k = ev.key
        if k in ("right", "n"):
            self.step(+1)
        elif k in ("left", "p"):
            self.step(-1)
        elif k in ("up", "down"):
            j = self.strata.index(self.stratum) + (1 if k == "down" else -1)
            self.r_stratum.set_active(int(np.clip(j, 0, len(self.strata) - 1)))
        elif k in ("1", "2", "3"):
            self.r_cond.set_active(int(k) - 1)
        elif k == "c":
            self.r_col.set_active((COLOURINGS.index(self.colouring) + 1) % len(COLOURINGS))
        elif k == "k":
            self.show_camera = not self.show_camera
            self.render(keep_view=False)
        elif k == "r":
            self.render(keep_view=False)
        elif k in ("+", "="):
            self._set(budget=int(self.budget * 2))
        elif k == "-":
            self._set(budget=max(2000, int(self.budget / 2)))
        elif k == "s":
            name = f"{self.items[self.i]['stratum'].replace('/', '_')}_{self.items[self.i]['scene_id']}.png"
            self.fig.savefig(HERE / name, dpi=130)
            print("saved", HERE / name)
        elif k == "o":
            open_in_open3d(self.sc, self.condition)
        elif k == "h":
            print(__doc__)

    @safe
    def show(self, i: int):
        i = int(np.clip(i, 0, len(self.items) - 1)) if self.items else 0
        if i == self.i or not self.items:
            return
        self.i = i
        self.entry = self.items[i]
        self.sc = load_scene(self.entry)
        self.render(keep_view=False)

    # -- drawing
    def position(self) -> str:
        return (f"scene {self.i + 1} / {len(self.items)}" + (f" in {self.stratum}" if self.stratum != "all" else "")
                + f"   ·   {len(self.all)} scenes in total")

    @safe
    def render(self, keep_view: bool):
        if self.sc is None:
            return
        view = (self.ax3.elev, self.ax3.azim) if keep_view else default_view(self.sc, self.show_camera)
        draw_3d(self.ax3, self.sc, self.condition, self.colouring, self.budget, self.show_camera, view)
        draw_camera_image(self.axi, self.sc)
        self.axt.cla(); self.axt.axis("off")
        self.axt.text(0, 1, scene_text(self.entry, self.sc, self.position()), va="top", ha="left",
                      family="monospace", fontsize=7.4, color=INK, transform=self.axt.transAxes)
        self.fig.canvas.draw_idle()


def print_catalogue(entries: list[dict]):
    print(f"{len(entries)} scenes")
    for st in STRATA:
        es = [e for e in entries if e["stratum"] == st]
        if es:
            print(f"  {st:<18} {len(es):>3}  {DESCRIBE[st]}")


def save_all(entries: list[dict], out: Path, budget: int):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    out.mkdir(parents=True, exist_ok=True)
    for k, e in enumerate(entries):
        sc = load_scene(e)
        fig = plt.figure(figsize=(16, 9))
        ax3 = fig.add_axes([0.02, 0.05, 0.6, 0.88], projection="3d")
        axi = fig.add_axes([0.645, 0.56, 0.345, 0.37]); axt = fig.add_axes([0.645, 0.04, 0.345, 0.49]); axt.axis("off")
        draw_3d(ax3, sc, "exterior", "part", budget, False, look_from_camera(sc))
        draw_camera_image(axi, sc)
        axt.text(0, 1, scene_text(e, sc, f"scene {k + 1} / {len(entries)}"), va="top", family="monospace",
                 fontsize=7.4, transform=axt.transAxes)
        fig.savefig(out / f"{k + 1:03d}_{e['stratum'].replace('/', '_')}_{e['scene_id']}.png", dpi=110)
        plt.close(fig)
        print(f"  {k + 1}/{len(entries)}", end="\r", flush=True)
    print(f"\n{len(entries)} PNGs in {out}")


def main(default_root: str = "bench_phase4"):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", default=str(HERE / default_root), help="dataset folder (default: next to this script)")
    ap.add_argument("--stratum", default="all", help="start in one stratum, e.g. T/circle")
    ap.add_argument("--start", type=int, default=0, help="scene index to open first (within the stratum)")
    ap.add_argument("--points", type=int, default=30000, help="points drawn in 3D (keys + / - change it)")
    ap.add_argument("--list", action="store_true", help="print the catalogue and exit")
    ap.add_argument("--save-all", metavar="DIR", help="write one PNG per scene to DIR instead of opening a window")
    a = ap.parse_args()
    root = Path(a.root)
    if not root.is_dir():
        sys.exit(f"dataset folder not found: {root}  (put this script next to it, or pass --root)")
    entries = build_catalogue(root)
    if not entries:
        sys.exit(f"no scenes under {root}")
    print_catalogue(entries)
    if a.list:
        return
    if a.save_all:
        es = entries if a.stratum == "all" else [e for e in entries if e["stratum"] == a.stratum]
        save_all(es, Path(a.save_all), a.points)
        return
    import matplotlib.pyplot as plt
    print(environment())
    Viewer(entries, start=a.start, stratum=a.stratum, budget=a.points)
    plt.show()


if __name__ == "__main__":
    main()
