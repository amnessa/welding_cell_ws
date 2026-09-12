"""ICRA 2027 paper - every figure, table and headline number from the data, in one command.

    /workspaces/welding_cell_ws/.venv/bin/python paper/icra2027/make_figures.py

Reads out/phase4_batch/phase4_batch.csv.gz, out/bench_phase4/facts.csv, the corpus, and
out/annotation/annotation_scores.csv; writes paper/icra2027/figures/*.pdf, tables/*.tex and
tables/numbers.tex (\\newcommand macros used in the text). Computations are the ones of
notebooks/build_15_phase4_results.py: per-scene medians over seeds first, then medians.
"""
from __future__ import annotations
import json, pathlib, sys
import numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

ROOT = pathlib.Path(__file__).resolve().parents[2]
PAPER = pathlib.Path(__file__).resolve().parent
FIG, TAB = PAPER / "figures", PAPER / "tables"; FIG.mkdir(exist_ok=True); TAB.mkdir(exist_ok=True)
sys.path.insert(0, str(ROOT))

DF = pd.read_csv(ROOT / "out/phase4_batch/phase4_batch.csv.gz", low_memory=False)
FACTS = pd.read_csv(ROOT / "out/bench_phase4/facts.csv")
def _fam(r):
    if isinstance(r.seam_family, str): return r.seam_family
    return "line_grooved" if (isinstance(r.prep, str) and r.prep != "square") else "line"
FACTS["family"] = FACTS.apply(_fam, axis=1); FACTS["stratum"] = FACTS.joint_type + "/" + FACTS.family
JOIN = ["scene_id", "family", "stratum", "sensor_profile", "twin_key"]
DF = DF.drop(columns=[c for c in JOIN[1:] if c in DF.columns], errors="ignore").merge(FACTS[JOIN], on="scene_id", how="left")
METHODS = ["lit-ransac", "lit-regiongrow", "lit-lobb", "lit-ppf", "lit-pcaslice", "lit-modelreg", "lit-quadric"]
SHORT = {"lit-ransac": "ransac", "lit-regiongrow": "regiongrow", "lit-lobb": "lobb", "lit-ppf": "ppf",
         "lit-pcaslice": "pcaslice", "lit-modelreg": "modelreg", "lit-quadric": "quadric"}
PAPERTAG = {"lit-ransac": "M1", "lit-regiongrow": "M2", "lit-lobb": "M3", "lit-ppf": "M4", "lit-pcaslice": "M5", "lit-modelreg": "M6", "lit-quadric": "M7"}
ORDER_JT = ["T", "corner", "butt", "lap", "edge"]
ORDER_STRATA = ["T/line", "T/circle", "T/ellipse", "T/saddle", "T/rounded_rect", "T/swept_path",
                "corner/line", "butt/line", "butt/line_grooved", "butt/arc_butt", "lap/line", "edge/line"]
STRATA_LABEL = {"T/line": "T line", "T/circle": "T pipe (circle)", "T/ellipse": "T pipe (cut)", "T/saddle": "T saddle",
                "T/rounded_rect": "T rect. tube", "T/swept_path": "T swept stiffener", "corner/line": "corner",
                "butt/line": "butt square", "butt/line_grooved": "butt grooved", "butt/arc_butt": "butt arc",
                "lap/line": "lap", "edge/line": "edge"}
COLOR = dict(zip(METHODS, ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#4a3aa7"]))
GRAY, INK, INK2 = "#b8b7b2", "#0b0b0b", "#52514e"
SEQ = LinearSegmentedColormap.from_list("seq", ["#f4f8fd", "#cde2fb", "#86b6ef", "#3987e5", "#1c5cab", "#0d366b"])
DIV = LinearSegmentedColormap.from_list("div", ["#d03b3b", "#f0efec", "#2a78d6"])
plt.rcParams.update({"font.size": 7.5, "axes.titlesize": 8, "axes.titleweight": "bold", "axes.labelsize": 7.5,
                     "xtick.labelsize": 7, "ytick.labelsize": 7, "legend.fontsize": 7, "axes.spines.top": False,
                     "axes.spines.right": False, "axes.edgecolor": "#c9c8c3", "pdf.fonttype": 42, "ps.fonttype": 42,
                     "axes.grid": False, "legend.frameon": False})
COLW, FULLW = 3.45, 7.1   # inches: IEEE column and full width
NUMS = {}
def num(name, value, fmt="{:.2f}"):
    NUMS[name] = fmt.format(value) if isinstance(value, (int, float, np.floating, np.integer)) else str(value)

def scene_level(d, col):
    return d.groupby(["method", "stratum", "joint_type", "scene_id"], observed=True)[col].median().reset_index()

def heat(tbl, ax, cmap=SEQ, vmin=0, vmax=1, fmt="{:.2f}", xlabels=None, ylabels=None, cbar=True, cbar_label=""):
    im = ax.imshow(tbl.values.astype(float), cmap=cmap, vmin=vmin, vmax=vmax, aspect="auto")
    ax.set_xticks(range(tbl.shape[1])); ax.set_xticklabels(xlabels or [PAPERTAG.get(c, c) for c in tbl.columns])
    ax.set_yticks(range(tbl.shape[0])); ax.set_yticklabels(ylabels or [STRATA_LABEL.get(i, i) for i in tbl.index])
    for i in range(tbl.shape[0]):
        for j in range(tbl.shape[1]):
            v = tbl.values[i, j]
            if np.isfinite(v):
                rel = (v - vmin) / (vmax - vmin) if vmax > vmin else 0
                ax.text(j, i, fmt.format(v), ha="center", va="center", fontsize=6.2,
                        color="white" if (cmap is SEQ and rel > 0.55) or (cmap is DIV and abs(rel - 0.5) > 0.32) else INK)
            else:
                ax.text(j, i, "--", ha="center", va="center", fontsize=6.2, color=GRAY)
    if cbar:
        plt.colorbar(im, ax=ax, shrink=0.8, pad=0.02, label=cbar_label)
    return im

# ------------------------------------------------------------------ 1. coverage (Task 1 + single view)
cov = DF[DF.chunk.str.startswith("coverage")]
def cov_table(view, col="f1"):
    s = scene_level(cov[cov.condition == view], col)
    return s.pivot_table(index="stratum", columns="method", values=col, aggfunc="median").reindex(ORDER_STRATA)[METHODS]
T_full, T_single = cov_table("full_exterior"), cov_table("single")
fig, axes = plt.subplots(1, 2, figsize=(FULLW, 2.9), gridspec_kw={"wspace": 0.55})
heat(T_full, axes[0], cbar=False); axes[0].set_title("(a) Task 1: full exterior scan", loc="left")
heat(T_single, axes[1], ylabels=[""] * 12, cbar_label="median F1 @ 3 mm"); axes[1].set_title("(b) single view", loc="left")
plt.savefig(FIG / "fig_coverage.pdf", bbox_inches="tight"); plt.close()
num("ransacTline", T_full.loc["T/line", "lit-ransac"]); num("quadricTline", T_full.loc["T/line", "lit-quadric"])
curved = [s for s in ORDER_STRATA if s.startswith("T/") and s != "T/line"]
num("ransacCurvedMax", T_full.loc[curved, "lit-ransac"].max()); num("ppfCurvedMin", T_full.loc[curved, "lit-ppf"].min()); num("ppfCurvedMax", T_full.loc[curved, "lit-ppf"].max())
num("quadricCurvedMean", T_full.loc[curved, "lit-quadric"].mean()); num("lobbCurvedMean", T_full.loc[curved, "lit-lobb"].mean())
num("groovedMax", T_full.loc["butt/line_grooved"].drop("lit-modelreg").max())
num("nStrataNonzeroLobb", int((T_full["lit-lobb"] > 0.05).sum()), "{:d}")
(TAB / "coverage_full.tex").write_text(T_full.rename(columns=PAPERTAG).rename(index=STRATA_LABEL).to_latex(float_format="%.2f", na_rep="--"))

# ------------------------------------------------------------------ 2. straight vs curved, per method (table)
straight = T_full.loc["T/line"]; curv = T_full.loc[curved].mean()
sc = pd.DataFrame({"T line": straight, "curved T (mean of 5)": curv}).T
sc.index.name = None; sc.columns.name = None
(TAB / "straight_curved.tex").write_text(sc.rename(columns=PAPERTAG).to_latex(float_format="%.2f", na_rep="--"))

# ------------------------------------------------------------------ 3. oracle ladder (L1 - L0)
l0 = scene_level(cov[cov.condition == "full_exterior"], "f1").rename(columns={"f1": "L0"})
l1 = scene_level(DF[DF.chunk.str.startswith("l1_")], "f1").rename(columns={"f1": "L1"})
lad = l0.merge(l1, on=["method", "stratum", "joint_type", "scene_id"])
tbl = lad.groupby(["method", "joint_type"])[["L0", "L1"]].median().unstack("joint_type")
LM = [m for m in METHODS if m != "lit-modelreg"]
L0t, L1t = tbl["L0"][ORDER_JT].reindex(LM), tbl["L1"][ORDER_JT].reindex(LM)
fig, axes = plt.subplots(1, 2, figsize=(FULLW, 2.2), gridspec_kw={"wspace": 0.35, "width_ratios": [1, 1]})
heat(L0t.T, axes[0], xlabels=[PAPERTAG[m] for m in LM], ylabels=ORDER_JT, cbar=False); axes[0].set_title("(a) L0: with the paper's own coarse stage (oracle)", loc="left")
heat(L1t.T, axes[1], xlabels=[PAPERTAG[m] for m in LM], ylabels=ORDER_JT, cbar_label="median F1"); axes[1].set_title("(b) L1: coarse stage withheld", loc="left")
plt.savefig(FIG / "fig_ladder.pdf", bbox_inches="tight"); plt.close()
pooled0 = lad.groupby("method").L0.median(); pooled1 = lad.groupby("method").L1.median()
num("quadricLzero", pooled0["lit-quadric"]); num("quadricLone", pooled1["lit-quadric"]); num("lobbLzero", pooled0["lit-lobb"]); num("lobbLone", pooled1["lit-lobb"])
(TAB / "ladder.tex").write_text(pd.DataFrame({"L0": pooled0, "L1": pooled1}).reindex(LM).rename(index=PAPERTAG).to_latex(float_format="%.2f"))

# ------------------------------------------------------------------ 4. sensor noise small multiples
n0 = cov[cov.condition == "single"].assign(ns=0.0)
nn = DF[DF.chunk.str.startswith("noise")].assign(ns=lambda d: d.noise_scale)
N = pd.concat([n0, nn]); N = N[N.method != "lit-modelreg"]
fig, axes = plt.subplots(1, 3, figsize=(FULLW, 1.9), sharey=True, gridspec_kw={"wspace": 0.12})
for ax, prof in zip(axes, ["d435i", "stereo_good", "stereo_poor"]):
    sub = N[N.sensor_profile == prof]
    g = sub.groupby(["method", "ns", "scene_id"]).f1.median().groupby(["method", "ns"]).median().unstack("ns")
    for m in METHODS:
        if m not in g.index: continue
        emph = m in ("lit-quadric", "lit-lobb")
        ax.plot(g.columns, g.loc[m], marker="o", markersize=3, linewidth=1.6 if emph else 1.0, color=COLOR[m] if emph else GRAY, zorder=3 if emph else 2)
        ax.text(g.columns[-1] + 0.06, g.loc[m].iloc[-1], PAPERTAG[m], fontsize=6.5, va="center", color=COLOR[m] if emph else INK2)
    ax.set_title(prof.replace("_", " "), loc="left"); ax.set_xticks([0, 1, 2]); ax.set_xticklabels(["clean", "1$\\sigma$", "2$\\sigma$"]); ax.set_xlim(-0.15, 2.7); ax.set_ylim(-0.02, 1.02)
axes[0].set_ylabel("median F1, single view")
plt.savefig(FIG / "fig_noise.pdf", bbox_inches="tight"); plt.close()
gN = N.groupby(["method", "ns", "scene_id"]).f1.median().groupby(["method", "ns"]).median().unstack("ns")
for m in ("lit-quadric", "lit-lobb", "lit-ransac"):
    if m in gN.index: num(f"noise{SHORT[m].capitalize()}Clean", gN.loc[m, 0.0]); num(f"noise{SHORT[m].capitalize()}Two", gN.loc[m, 2.0])

# ------------------------------------------------------------------ 5. Task 2 table
t2 = DF[DF.chunk.str.startswith("task2")].copy()
t2["mps_matched"] = t2.mps_matched.astype(str).str.lower() == "true"; t2["mps_null"] = t2.mps_null.astype(str).str.lower() == "true"; t2 = t2[~t2.mps_null]
sel = t2.groupby(["method", "joint_type", "scene_id"]).mps_matched.mean().groupby(["method", "joint_type"]).mean().unstack("joint_type")[ORDER_JT].reindex(METHODS)
loc = t2[t2.mps_matched].groupby(["method", "joint_type", "scene_id"]).rmse.median().groupby(["method", "joint_type"]).median().unstack("joint_type")[ORDER_JT].reindex(METHODS)
t2tab = pd.concat({"selection rate": sel, "RMSE (mm)": loc}, axis=1).rename(index=PAPERTAG)
t2tab.columns.names = [None, None]; t2tab.index.name = None
(TAB / "task2.tex").write_text(t2tab.to_latex(float_format="%.2f", na_rep="--", multicolumn_format="c"))
num("taskTwoQuadricSel", sel.loc["lit-quadric"].mean()); num("taskTwoQuadricRmseMax", loc.loc["lit-quadric"].max(), "{:.1f}"); num("taskTwoLobbSel", sel.loc["lit-lobb"].mean())

# ------------------------------------------------------------------ 6. fixture pairs (table)
fx = DF[DF.chunk.str.startswith("fixture")].copy()
if len(fx):
    tk = {}
    for jt in ORDER_JT:
        for ln in (ROOT / f"out/bench_phase4_fx/{jt}/index.jsonl").read_text().splitlines():
            r = json.loads(ln)
            if r.get("emitted"): tk[r["scene_id"]] = r["twin_key"]
    fx["twin_key"] = fx.twin_key.fillna(fx.scene_id.map(tk))
    g = fx.groupby(["method", "fixture", "twin_key"]).agg(f1=("f1", "median"), prec=("precision", "median"), n=("n_pred_seams", "median")).reset_index()
    p = g.pivot_table(index=["method", "twin_key"], columns="fixture", values=["f1", "prec", "n"])
    d = pd.DataFrame({"dF1": p[("f1", True)] - p[("f1", False)], "dPrec": p[("prec", True)] - p[("prec", False)], "dN": p[("n", True)] - p[("n", False)]}).dropna().reset_index()
    fxt = d.groupby("method").agg(pairs=("dF1", "size"), dF1=("dF1", "median"), dPrec=("dPrec", "median"), extra=("dN", "median")).reindex(METHODS).rename(index=PAPERTAG)
    fxt.columns = ["pairs", "$\\Delta$F1", "$\\Delta$precision", "extra seams"]; fxt.index.name = None
    (TAB / "fixture.tex").write_text(fxt.to_latex(float_format="%.2f", na_rep="--", escape=False))
    num("fixtureQuadricDFone", fxt.loc["M7", "$\\Delta$F1"]); num("fixtureLobbExtra", fxt.loc["M3", "extra seams"], "{:.0f}")

# ------------------------------------------------------------------ 7. cost table
cost = pd.concat([cov.assign(arm="L0"), DF[DF.chunk.str.startswith("l1_")].assign(arm="L1", condition="full_exterior")])
cost = cost.assign(condition=cost.condition.str.replace("_", " "))
ct = cost.groupby(["method", "arm", "condition"]).sec.median().unstack(["arm", "condition"]).reindex(METHODS).rename(index=PAPERTAG)
ct.columns.names = [None, None]; ct.index.name = None
(TAB / "cost.tex").write_text(ct.to_latex(float_format="%.2f", na_rep="--", multicolumn_format="c"))

# ------------------------------------------------------------------ 8. Phase 5 annotation floor
ann = pd.read_csv(ROOT / "out/annotation/annotation_scores.csv"); br = ann[ann.role == "briefed"]
fig, ax = plt.subplots(figsize=(COLW, 1.9))
order = ["edge", "corner", "T", "butt", "lap"]
data = [br[br.joint_type == jt].lat_rmse.dropna().values for jt in order]
bp = ax.boxplot(data, tick_labels=order, widths=0.5, patch_artist=True, showfliers=True, flierprops=dict(marker="o", markersize=2.5, markerfacecolor=GRAY, markeredgecolor="none"))
for b in bp["boxes"]: b.set(facecolor="white", edgecolor="#2a78d6", linewidth=1.2)
for l in bp["medians"]: l.set(color="#2a78d6", linewidth=1.6)
ax.axhline(0.6, color="#d03b3b", linewidth=1, linestyle="--"); ax.text(5.45, 0.6, "reported\n0.6 mm", color="#d03b3b", fontsize=6.5, va="center")
ax.axhline(br.lat_rmse.median(), color=INK2, linewidth=0.8, linestyle=":"); ax.text(5.45, br.lat_rmse.median(), f"median\n{br.lat_rmse.median():.1f} mm", color=INK2, fontsize=6.5, va="center")
ax.set_ylabel("lateral RMSE of hand labels (mm)"); ax.set_yscale("log"); ax.set_xlim(0.4, 6.3)
plt.savefig(FIG / "fig_annotation.pdf", bbox_inches="tight"); plt.close()
num("annMedian", br.lat_rmse.median(), "{:.1f}"); num("annPninetyfive", br.lat_rmse.quantile(0.95), "{:.1f}"); num("annLap", br[br.joint_type == "lap"].lat_rmse.median(), "{:.1f}")
num("annEnd", br.end_err.median(), "{:.1f}"); num("annMiss", br.missed.sum() / br.n_gt.sum(), "{:.2f}"); num("annRatio", br.lat_rmse.median() / 0.6, "{:.1f}")
num("annOwner", ann[ann.role != "briefed"].lat_rmse.median(), "{:.1f}")

# ------------------------------------------------------------------ 9. corpus numbers
num("nScenes", int(FACTS.scene_id.nunique()), "{:d}"); num("nRows", int(len(DF)), "{:,d}"); num("nStrata", len(ORDER_STRATA), "{:d}")
num("nRansacSeeds", int(DF[DF.method == "lit-ransac"].seed.nunique()), "{:d}")
(TAB / "numbers.tex").write_text("".join(f"\\newcommand{{\\{k}}}{{{v}}}\n" for k, v in NUMS.items()))
print("figures:", sorted(p.name for p in FIG.glob("*.pdf"))); print("tables:", sorted(p.name for p in TAB.glob("*.tex")))
print("numbers:", {k: v for k, v in list(NUMS.items())[:14]})
