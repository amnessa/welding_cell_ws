"""Fig. 1: the twelve strata as tier-1 clouds with the constructed seams, plus a strip of
tier-2 renders (view 0, with the seam mask) where they exist. One scene per stratum."""
from __future__ import annotations
import json, pathlib, sys
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image
ROOT = pathlib.Path(__file__).resolve().parents[2]; FIG = pathlib.Path(__file__).resolve().parent / "figures"
sys.path.insert(0, str(ROOT))
from weldgen.camera import project
ORDER = [("T", "line"), ("T", "circle"), ("T", "ellipse"), ("T", "saddle"), ("T", "rounded_rect"), ("T", "swept_path"),
         ("corner", "line"), ("butt", "line_square"), ("butt", "line_grooved"), ("butt", "arc"), ("lap", "line"), ("edge", "line")]
LABEL = {("T", "line"): "T line", ("T", "circle"): "T pipe", ("T", "ellipse"): "T pipe, cut", ("T", "saddle"): "T saddle",
         ("T", "rounded_rect"): "T rect. tube", ("T", "swept_path"): "T swept", ("corner", "line"): "corner",
         ("butt", "line_square"): "butt square", ("butt", "line_grooved"): "butt grooved", ("butt", "arc"): "butt arc",
         ("lap", "line"): "lap", ("edge", "line"): "edge"}
def first_scene(corpus, cls, src, k=0):
    ids = [json.loads(l)["scene_id"] for l in open(ROOT / f"out/{corpus}/{cls}/index.jsonl") if json.loads(l).get("emitted") and json.loads(l)["source"] == src]
    return ROOT / f"out/{corpus}/{cls}/{ids[k]}"
plt.rcParams.update({"font.size": 7, "pdf.fonttype": 42})
fig, axes = plt.subplots(2, 6, figsize=(7.1, 2.75), subplot_kw={"projection": "3d"}, gridspec_kw={"wspace": 0.02, "hspace": 0.02})
for ax, (cls, src) in zip(axes.ravel(), ORDER):
    sd = first_scene("bench_phase4", cls, src, 1 if src == "line" else 0)
    scene = json.loads((sd / "scene.json").read_text()); c = np.load(sd / "cloud.npz"); s = np.load(sd / "seams.npz")
    xyz = c["xyz"]; sub = np.random.default_rng(0).choice(len(xyz), min(4000, len(xyz)), replace=False)
    ax.scatter(xyz[sub, 0], xyz[sub, 1], xyz[sub, 2], s=0.18, c=np.where(c["object_id"][sub] == 0, "#aeb6c0", "#d6c7a6"), depthshade=False, linewidths=0, zorder=1)
    for sm in scene["seams"]:
        if sm["weldable"]:
            p = s[f"seam_{sm['id']}"]; ax.plot(p[:, 0], p[:, 1], p[:, 2], color="#d03b3b" if sm["matches_joint_type"] else "#eda100", linewidth=1.6, zorder=10)
    lo, hi = xyz.min(0), xyz.max(0); ctr, span = (lo + hi) / 2, (hi - lo).max() / 2
    ax.set_xlim(ctr[0] - span, ctr[0] + span); ax.set_ylim(ctr[1] - span, ctr[1] + span); ax.set_zlim(ctr[2] - span, ctr[2] + span)
    ax.view_init(elev=32, azim=-55); ax.set_axis_off(); ax.set_title(LABEL[(cls, src)], fontsize=7, pad=-2)
plt.savefig(FIG / "fig_gallery.pdf", bbox_inches="tight", pad_inches=0.02); plt.close()
# tier-2 strip: view 0 rgb with the seam mask, for strata that have renders in train_v1
tiles = []
for cls, src in ORDER:
    try:
        sd = first_scene("train_v1", cls, src)
    except Exception:
        continue
    v = sd / "views/0"
    if not (v / "rgb.png").exists():
        continue
    rgb = np.array(Image.open(v / "rgb.png")); ms = np.array(Image.open(v / "mask_seam.png")); mt = np.array(Image.open(v / "mask_tack.png"))
    ov = rgb.astype(float); ov[ms > 0] = 0.35 * ov[ms > 0] + 0.65 * np.array([0, 230, 0]); ov[mt > 0] = [235, 0, 0]
    # crop to the workpiece bounding box with a margin
    mo = np.array(Image.open(v / "mask_object.png")); ys, xs = np.nonzero((mo > 0) & (mo != 40))
    if len(ys) == 0: continue
    y0, y1 = max(ys.min() - 60, 0), min(ys.max() + 60, rgb.shape[0]); x0, x1 = max(xs.min() - 60, 0), min(xs.max() + 60, rgb.shape[1])
    side = max(y1 - y0, x1 - x0); cy, cx = (y0 + y1) // 2, (x0 + x1) // 2
    y0, y1 = max(cy - side // 2, 0), min(cy + side // 2, rgb.shape[0]); x0, x1 = max(cx - side // 2, 0), min(cx + side // 2, rgb.shape[1])
    tiles.append((LABEL[(cls, src)], np.array(Image.fromarray(ov[y0:y1, x0:x1].astype(np.uint8)).resize((220, 220)))))
if tiles:
    fig, axes = plt.subplots(1, len(tiles), figsize=(7.1, 7.1 / len(tiles) + 0.25), gridspec_kw={"wspace": 0.03})
    for ax, (lab, im) in zip(np.atleast_1d(axes), tiles):
        ax.imshow(im); ax.set_axis_off(); ax.set_title(lab, fontsize=6.5, pad=2)
    plt.savefig(FIG / "fig_renders.pdf", bbox_inches="tight", pad_inches=0.02, dpi=200); plt.close()
print("gallery written;", len(tiles), "render tiles")
