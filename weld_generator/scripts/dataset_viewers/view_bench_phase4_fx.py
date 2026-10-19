#!/usr/bin/env python3
"""WeldSet fixture-twin viewer: browse `bench_phase4_fx/` next to its originals in `bench_phase4/`.

`bench_phase4_fx` holds 274 FIXTURE TWINS: plate scenes of `bench_phase4` regenerated with the
workpieces clamped on a fixture table. A twin and its original share `twin_key`: the same two
parts, same dimensions, same relative pose, same seam candidates. What differs is the table
itself, where the assembly sits on it, the camera pose drawn for that placement, and which
seams stay weldable and visible (the table can block the torch or hide a seam from the camera).
The benchmark scores every method on these pairs to measure what a fixture in view costs it.

Why 274 and not 720: only the six straight-seam PLATE strata (360 scenes) get a fixture (the
fixture model describes how a plate rests on a table; a tube resting on a line does not fit
it). Seeds the fixture makes unusable are recorded and not replaced; the viewer lists how
many each stratum kept and why the rest were lost.

This viewer shows each pair side by side: left the original (no fixture), right the twin
(fixture grey), each with its camera's depth image, plus a seam-by-seam comparison.

Requirements: Python 3.9+, numpy, matplotlib   (pip install numpy matplotlib)
It imports the drawing code from `view_bench_phase4.py`, which must sit in the same folder.
Run it from the folder that contains `bench_phase4_fx/` and `bench_phase4/`:

    python view_bench_phase4_fx.py
    python view_bench_phase4_fx.py --stratum corner/line
    python view_bench_phase4_fx.py --list

Controls: slider / Prev / Next move through the pairs; panels on the left pick the stratum,
input condition and colouring. Keys: left / right (pair), up / down (stratum), 1 2 3
(exterior / camera view / all samples), c (colouring), r (look from the camera again),
+ - (points drawn), s (save PNG), o (open the twin in Open3D), q (quit). Rotating one 3D
panel rotates the other when you release the mouse.
"""
from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import view_bench_phase4 as base  # noqa: E402

INK, INK2 = base.INK, base.INK2
REASON = {"NoVisibleSeams": "no visible seam",       # the fixture hides every seam from the camera
          "NoSeamsFound": "no weldable seam"}         # a fixture contact closes the joint
SOURCE_TO_STRATUM = {("T", "line"): "T/line", ("corner", "line"): "corner/line",
                     ("butt", "line_square"): "butt/line", ("butt", "line_grooved"): "butt/line_grooved",
                     ("lap", "line"): "lap/line", ("edge", "line"): "edge/line"}


def survival(fx_root: Path) -> dict:
    """Per stratum: seeds replayed, twins kept, and the reasons for the rest (index.jsonl)."""
    out = {}
    for idx in sorted(fx_root.glob("*/index.jsonl")):
        jt = idx.parent.name
        for ln in idx.read_text().splitlines():
            r = json.loads(ln)
            st = SOURCE_TO_STRATUM.get((jt, r["source"]), f"{jt}/{r['source']}")
            d = out.setdefault(st, {"seeds": 0, "kept": 0, "lost": Counter()})
            d["seeds"] += 1
            if r.get("emitted"):
                d["kept"] += 1
            else:
                d["lost"][r.get("reason", "?")] += 1
    return out


def survival_text(surv: dict) -> str:
    rows = ["twins per stratum: kept / plate seeds replayed with a fixture",
            "lost seeds are recorded, not replaced. no visible seam = the fixture",
            "hides every seam from the camera; no weldable seam = a fixture contact",
            "closes the joint"]
    for st in base.STRATA:
        if st in surv:
            d = surv[st]
            why = ", ".join(f"{n} {REASON.get(k, k)}" for k, n in d["lost"].most_common())
            rows.append(f" {st:<18} {d['kept']:>3} / {d['seeds']:<3} {why}")
    tot_k = sum(d["kept"] for d in surv.values()); tot_s = sum(d["seeds"] for d in surv.values())
    rows.append(f" {'total':<18} {tot_k:>3} / {tot_s}  (curved strata: no fixture, by design)")
    return "\n".join(rows)


def seam_comparison(orig: dict, twin: dict) -> str:
    """Workpiece seams of the original and the twin, matched on their face pair."""
    def key(m):
        return tuple(m["face_pair"])
    A = {key(m): m for m in orig["seams"]}
    B = {key(m): m for m in twin["seams"] if not any(f.startswith("F:") for f in m["face_pair"])}
    n_fix = sum(any(f.startswith("F:") for f in m["face_pair"]) for m in twin["seams"])
    def state(m):
        if m is None:
            return "  -  "
        return "prim." if base.is_primary(m) else ("weld." if m["weldable"] else "rej. ")
    def seen(m):
        v = None if m is None else m.get("visible_fraction")
        return " -  " if v is None else f"{v:.2f}"
    rows = ["seams of the workpieces: without fixture -> with fixture",
            " faces      class   length state       seen       reject reason (twin)"]
    order = sorted(set(A) | set(B), key=lambda k: (not (k in A and base.is_primary(A[k])), k))
    for k in order[:14]:
        a, b = A.get(k), B.get(k)
        m = a or b
        changed = "  *" if state(a) != state(b) else ""
        rows.append(f" {'x'.join(k):<10} {m['seam_class']:<7} {m['length_mm']:4.0f}mm {state(a)}->{state(b)} "
                    f"{seen(a)}->{seen(b)} {(b or {}).get('reject_reason') or '-'}{changed}")
    if len(order) > 14:
        rows.append(f" ... {len(order) - 14} more")
    rows.append(" prim. = primary seam (truth), weld. = other weldable, rej. = rejected")
    rows.append(" seen = fraction seen by the camera; * = changed by the fixture")
    rows.append(f" + {n_fix} part-fixture contact lines in the twin (rejected: fixture_contact)")
    return "\n".join(rows)


def pair_text(k: int, n: int, stratum: str, e_o: dict, e_t: dict, so: dict, st: dict) -> str:
    s = st["scene"]
    j, fit = s["joint"], s.get("fit", {})
    fx = next(o for o in s["objects"] if o["role"] == "fixture")
    wp = [o for o in s["objects"] if o["role"] == "workpiece"]
    t = [o.get("thickness_mm") for o in wp if o.get("thickness_mm")]
    co, ct = so["scene"]["camera"], s["camera"]
    lines = [
        f"pair {k + 1} / {n}" + (f" in {stratum}" if stratum != "all" else ""),
        f"stratum    {e_t['stratum']}: {base.DESCRIBE.get(e_t['stratum'], '')}",
        f"twin_key   {e_t['twin_key']}   seed {s['seed']}",
        f"original   {e_o['scene_id']} (bench_phase4)",
        f"twin       {e_t['scene_id']} (bench_phase4_fx)",
        f"joint      {j.get('iso_17659_term', j['type'])} · quality {j.get('quality_level', '?')} [ISO 5817]"
        f" · gap {fit.get('root_gap_mm', float('nan')):.2f} mm",
        f"parts      {' + '.join(o['primitive'] for o in wp)} · thickness "
        + (f"{min(t):.1f}–{max(t):.1f} mm" if t else "-"),
        f"fixture    table {fx['dims_mm'][0]:.0f} x {fx['dims_mm'][1]:.0f} x {fx['dims_mm'][2]:.1f} mm"
        f" · parts rest '{j.get('contact_mode', '-')}'",
        f"camera     {s['noise_model'].get('profile', '?')} · standoff {co.get('standoff_mm', 0):.0f}->"
        f"{ct.get('standoff_mm', 0):.0f} mm · elev. {co.get('elevation_deg', 0):.0f}°->{ct.get('elevation_deg', 0):.0f}°",
        "           (each scene draws its own camera for its placement)",
        "",
        seam_comparison(so, st),
    ]
    return "\n".join(lines)


class TwinViewer:
    def __init__(self, pairs: list, surv: dict, start: int = 0, stratum: str = "all", budget: int = 20000):
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Button, Slider
        base.free_keys()
        self.all, self.surv, self.budget = pairs, surv, budget
        self.condition, self.colouring = "exterior", "part"
        self.fig = plt.figure(figsize=(18, 10))
        # The figure must own the viewer. matplotlib holds widget and key callbacks through WEAK
        # references, so a viewer nothing else refers to is freed by the next garbage
        # collection: the window stays open (the 3D panel still rotates) but every button,
        # the slider and the keys go dead - after a random 10-20 scenes, when gc happens to run.
        self.fig._weldset_viewer = self
        title = "WeldSet fixture twins (bench_phase4_fx): left the original without fixture, right the same parts on the fixture table"
        if self.fig.canvas.manager:
            self.fig.canvas.manager.set_window_title("WeldSet fixture twins")
        self.fig.text(0.006, 0.975, title, fontsize=12, fontweight="bold", color=INK)
        self.axo = self.fig.add_axes([0.125, 0.40, 0.30, 0.54], projection="3d")
        self.axf = self.fig.add_axes([0.43, 0.40, 0.30, 0.54], projection="3d")
        self.imo = self.fig.add_axes([0.135, 0.085, 0.285, 0.28])
        self.imf = self.fig.add_axes([0.44, 0.085, 0.285, 0.28])
        self.axt = self.fig.add_axes([0.738, 0.06, 0.26, 0.88]); self.axt.axis("off")
        counts = {s: sum(p[1]["stratum"] == s for p in pairs) for s in base.STRATA}
        self.strata = ["all"] + [s for s in base.STRATA if counts[s]]
        labels = [f"all ({len(pairs)})"] + [f"{s} ({counts[s]})" for s in self.strata[1:]]
        self.fig.text(0.006, 0.93, "stratum (pairs)", fontsize=8.5, color=INK2)
        self.r_stratum = base.radio(self.fig.add_axes([0.004, 0.62, 0.115, 0.3], frameon=False), labels,
                                    active=self.strata.index(stratum) if stratum in self.strata else 0)
        self.fig.text(0.006, 0.58, "input condition", fontsize=8.5, color=INK2)
        self.r_cond = base.radio(self.fig.add_axes([0.004, 0.45, 0.115, 0.12], frameon=False), base.CONDITIONS)
        self.fig.text(0.006, 0.41, "colour points by", fontsize=8.5, color=INK2)
        self.r_col = base.radio(self.fig.add_axes([0.004, 0.28, 0.115, 0.12], frameon=False), base.COLOURINGS)
        self.slider = Slider(self.fig.add_axes([0.18, 0.025, 0.42, 0.022]), "pair", 1, 2, valinit=1, valstep=1,
                             valfmt="%d")
        self.b_prev = Button(self.fig.add_axes([0.625, 0.018, 0.04, 0.034]), "Prev")
        self.b_next = Button(self.fig.add_axes([0.67, 0.018, 0.04, 0.034]), "Next")
        self.nav = base.Navigator(self.fig, self.show)
        self.fig.text(0.995, 0.975, base.environment(), fontsize=6, color=INK2, ha="right")
        self.r_stratum.on_clicked(lambda lab: self._filter(lab.split(" (")[0]))
        self.r_cond.on_clicked(lambda lab: self._set(condition=lab))
        self.r_col.on_clicked(lambda lab: self._set(colouring=lab))
        self.slider.on_changed(lambda v: self.nav.request(int(v) - 1))
        self.b_prev.on_clicked(lambda _: self.step(-1))
        self.b_next.on_clicked(lambda _: self.step(+1))
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self.fig.canvas.mpl_connect("button_release_event", self._sync_views)
        self.i = None
        self._filter(stratum if stratum in self.strata else "all", start)

    def _filter(self, stratum: str, start: int = 0):
        self.stratum = stratum
        self.items = self.all if stratum == "all" else [p for p in self.all if p[1]["stratum"] == stratum]
        n = len(self.items)
        self.slider.valmin, self.slider.valmax = 1, max(n, 2)
        self.slider.ax.set_xlim(1, max(n, 2))
        self.i = None
        start = int(np.clip(start, 0, n - 1))
        self.slider.set_val(start + 1)
        self.nav.request(start)
        self.nav.flush()

    def step(self, d: int):
        cur = self.nav.want if self.nav.want is not None else (self.i or 0)
        self.slider.set_val(int(np.clip(cur + d, 0, len(self.items) - 1)) + 1)

    def _set(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)
        self.render(keep_view=True)

    def _sync_views(self, ev):
        src = ev.inaxes
        if src not in (self.axo, self.axf):
            return
        dst = self.axf if src is self.axo else self.axo
        dst.view_init(src.elev, src.azim)
        self.fig.canvas.draw_idle()

    @base.safe
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
            self.r_col.set_active((base.COLOURINGS.index(self.colouring) + 1) % len(base.COLOURINGS))
        elif k == "r":
            self.render(keep_view=False)
        elif k in ("+", "="):
            self._set(budget=int(self.budget * 2))
        elif k == "-":
            self._set(budget=max(2000, int(self.budget / 2)))
        elif k == "s":
            name = f"twin_{self.items[self.i][1]['stratum'].replace('/', '_')}_{self.items[self.i][1]['twin_key']}.png"
            self.fig.savefig(HERE / name, dpi=120)
            print("saved", HERE / name)
        elif k == "o":
            base.open_in_open3d(self.st, self.condition)

    @base.safe
    def show(self, i: int):
        i = int(np.clip(i, 0, len(self.items) - 1)) if self.items else 0
        if i == self.i or not self.items:
            return
        self.i = i
        self.e_o, self.e_t = self.items[i]
        self.so = base.load_scene(self.e_o) if self.e_o else None
        self.st = base.load_scene(self.e_t)
        self.render(keep_view=False)

    @base.safe
    def render(self, keep_view: bool):
        if getattr(self, "st", None) is None:
            return
        view = (self.axf.elev, self.axf.azim) if keep_view else base.look_from_camera(self.st)
        base.draw_3d(self.axf, self.st, self.condition, self.colouring, self.budget, False, view,
                     title=f"with fixture (twin) · {self.condition}")
        base.draw_camera_image(self.imf, self.st, title="twin's camera: depth image, primary seams green")
        if self.so is not None:
            base.draw_3d(self.axo, self.so, self.condition, self.colouring, self.budget, False, view,
                         title=f"without fixture (original) · {self.condition}")
            base.draw_camera_image(self.imo, self.so, title="original's camera: depth image, primary seams green")
            txt = pair_text(self.i, len(self.items), self.stratum, self.e_o, self.e_t, self.so, self.st)
        else:
            for ax in (self.axo, self.imo):
                ax.cla()
            self.axo.set_title("original not found (put bench_phase4/ next to bench_phase4_fx/)", loc="left", fontsize=9)
            txt = base.scene_text(self.e_t, self.st, f"twin {self.i + 1} / {len(self.items)}")
        self.axt.cla(); self.axt.axis("off")
        self.axt.text(0, 1, txt + "\n\n" + survival_text(self.surv), va="top", ha="left", family="monospace",
                      fontsize=6.6, color=INK, transform=self.axt.transAxes)
        self.fig.canvas.draw_idle()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", default=str(HERE / "bench_phase4_fx"), help="fixture-twin folder (default: next to this script)")
    ap.add_argument("--originals", default=str(HERE / "bench_phase4"), help="the fixture-free benchmark folder")
    ap.add_argument("--stratum", default="all", help="start in one stratum, e.g. corner/line")
    ap.add_argument("--start", type=int, default=0, help="pair index to open first (within the stratum)")
    ap.add_argument("--points", type=int, default=20000, help="points drawn per 3D panel")
    ap.add_argument("--list", action="store_true", help="print the twin counts and exit")
    a = ap.parse_args()
    root, orig_root = Path(a.root), Path(a.originals)
    if not root.is_dir():
        sys.exit(f"fixture-twin folder not found: {root}  (put this script next to it, or pass --root)")
    twins = base.build_catalogue(root)
    surv = survival(root)
    print(survival_text(surv))
    if a.list:
        return
    originals = {}
    if orig_root.is_dir():
        originals = {e["twin_key"]: e for e in base.build_catalogue(orig_root)}
    else:
        print(f"originals not found at {orig_root}: showing the twins alone")
    pairs = [(originals.get(e["twin_key"]), e) for e in twins]
    print(f"{len(pairs)} twins, {sum(p[0] is not None for p in pairs)} matched to their original")
    import matplotlib.pyplot as plt
    print(base.environment())
    viewer = TwinViewer(pairs, surv, start=a.start, stratum=a.stratum, budget=a.points)  # noqa: F841
    plt.show()


if __name__ == "__main__":
    main()
