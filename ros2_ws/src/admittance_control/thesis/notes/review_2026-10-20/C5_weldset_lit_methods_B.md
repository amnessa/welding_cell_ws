# Material map: lit-pcaslice, lit-modelreg, lit-quadric, and `ours` (radius-PCA)

Everything was read-only. I created, modified and deleted nothing, and changed no git state. I ran one `python3 -c` command beyond JSON/.ipynb parsing: `print(abs(hash('T_000123')) % 2**32)` three times, to confirm Python's per-process hash salting (see §4.8). It reads and writes nothing. I also read `facts.csv` with sed/awk, and `scene.json` files with `python3 -c`.

Decimal points are used below. Repo quotes keep their decimal commas.

## 0. Caveats on where the numbers come from (read this first)

**The local `out/` is the stale laptop copy.**
- `out/phase4_batch/` only has the 2026-09-09 chunk files. These are from before the visibility fix and before the audit.
- There are no `pcaslice_whole_ring_*`, `quadric_projected_*` or `modelreg_edge_features` files locally.
- The post-audit numbers below come from notebook 15's stored outputs and from the ICRA `tables/numbers.tex`. Notebook 15 was executed 2026-09-15 11:05 UTC, after the audit landed at 2026-09-15 00:00 UTC.

**When each notebook's stored outputs were produced** (from cell execution metadata cross-checked with git):

| Notebook | Executed (UTC) | Corpus | Condition | Status |
|---|---|---|---|---|
| 03 | 2026-08-21 15:41–16:16 | `out/bench`, 250 scenes | view `"full"` (the whole CAD surface, including buried faces) and `"single"` | Cells 1 and 5 were re-executed later from an IDE on `out/bench6a` (git commits 0c8cb39 on 08-27 and f951bfe on 09-04), so the notebook mixes two corpora. |
| 09, 10 | 2026-08-22 | `out/bench`, 12 scenes/type | view `"full"` (the `run_matrix` default at commit 85f0fc7), 1 seed | Outputs unchanged since then. Later commits only changed the kernel name. Pre-audit. |
| 11 | 2026-09-09 12:54–13:04 | `bench_phase4`, pre-visfix version | — | Pre-audit: output was effectively "projected" and the flat test was the 3-D angle. The 2026-09-21 commit changed only the kernel display name. `build_11_lit_quadric.py` (dated Sep 9) matches the notebook (26 cells). |

**Number vintages used below:**
- (A) `out/phase3`, 2026-08-18.
- (B) `out/bench`, 2026-08-21/22.
- (C) `bench_phase4` batch, 2026-09-09: pre-visfix, pre-audit, 184 680 rows.
- (D) Visfix re-run, 2026-09-12 → 14.
- (E) Post-audit batch, 2026-09-15. Row count is quoted as 268 320 in README, handoff and nb15 cell 0, but nb15's own chunk counts sum to 271 200, and ICRA `\nRows` = 271,200.

**The paper's "ME" means different things in different papers.**
- WeldSet's `me` is the **maximum** error (`metrics.path_error_mm`: `"me": d.max()`). The mean is reported separately as `mean`.
- Wang et al. (pcaslice): "mean error (ME)".
- Fang & Tian (modelreg): "maximum error (ME)".
- Li et al. (quadric): per-axis position errors in Fig. 6.

---

## 1. Bibliographic data, verified on the PDFs

**lit-pcaslice** — `papers/implemented_papers/lit-pcaslice.pdf` (22 pages)
- Authors: Tianqi Wang¹·² (corresponding, wtq0622@163.com), Yifan Liang¹·², Xinqi Liu¹·², Zhigang Wang³.
- Affiliations: ¹ Tianjin Key Laboratory of Advanced Mechatronics Equipment Technology, Tianjin 300387; ² School of Mechanical Engineering, TianGong University, Tianjin; ³ Shandong Feizhou Intelligent Equipment Co., Ltd., Jinan.
- Title: "3D vision‑based intersecting pipe welding path planning". Research paper, *Welding in the World*, DOI 10.1007/s40194-026-02509-9.
- Received 10 March 2026; accepted 26 May 2026; © International Institute of Welding 2026; recommended by IIW Commission XVIII.
- **No volume, issue or pages are printed** (online-first).
- Published accuracy: average deviation < 1 mm and RMSE < 0.9 mm against 25 manually taught points.
  - Table 4, case (a): ME x/y/z 0.834/0.921/0.986; RMSE 0.752/0.683/0.701 mm.
  - Table 4, case (b): ME 0.906/0.899/0.989; RMSE 0.817/0.765/0.824 mm.

**lit-modelreg** — `lit-modelreg.pdf` (16 pages)
- Authors: **Weihua Fang, Xincheng Tian** (corresponding, txch@sdu.edu.cn). **Exactly two authors.**
- Affiliations: School of Control Science and Engineering, Shandong University, Jinan; Engineering Research Center of Intelligent Unmanned System, Ministry of Education.
- Title: "A novel model-based welding trajectory planning method for identical structural workpieces".
- *Robotics and Computer-Integrated Manufacturing* **89 (2024) 102772**, DOI 10.1016/j.rcim.2024.102772.
- Received 24 Apr 2023; revised 24 Jan 2024; accepted 29 Mar 2024; online 29 Apr 2024.
- Published accuracy (Table 1):
  - Offline: "ME not exceeding 0.6 mm and RMSE less than 0.3 mm". This is inconsistent with the paper's own Table 1, which lists lap A X ME = 0.63.
  - Online: ME ≤ 1.5 mm, RMSE < 0.6 mm; torch orientation ME < 0.2 rad.
- Registration parameters given in the paper: BCPD β = 1.2 (fillet), 0.9 (impeller), 1.8 (butt and lap); λ = 200; 500 iterations; tolerance 1e‑6. Model and voxel sampling interval: 1 mm.

**lit-quadric** — `lit-quadric.pdf` (8 pages)
- Authors: Dongmin Li¹·² (corresponding, ldm753@163.com), Yu Wang¹, Zhengyong Wang¹.
- Affiliations: ¹ College of Intelligent Equipment, Shandong University of Science and Technology, Tai'an; ² R&D Section, Shandong Judoshi Energy Technology Co.
- Title: "Automatic recognition on impeller shape and weld seam based on normal of point clouds and PCA".
- *Journal of Mechanical Science and Technology* **40 (3) (2026) 2039–2046** (printed as "2039~2046"), DOI 10.1007/s12206-026-0243-8.
- Received 23 Jan 2025; revised 12 Sep 2025; accepted 28 Oct 2025; editor Hae-Jin Choi.
- **The PDF's metadata title is wrong** ("Molybdenum and Cobalt Silicide Field Emitter Arrays", author "Isis"; a template artefact). Never take bibliography data from its metadata.

## 2. Every file in `papers/` and `papers/implemented_papers/`

`papers/` (13 PDFs plus the subdirectory):
1. *3D curve weld seam path and posture planning based on line laser sensors* — RCIM 94 (2025) 102985.
2. *A novel method for dense point cloud reconstruction and weld seam detection for tubesheet welding robot* — Opt. Laser Technol. 163 (2023) 109346.
3. *A novel seam extraction and path planning method for robotic welding of medium-thickness plate structural parts based on 3D vision* — RCIM 79 (2023) 102433.
4. *A Coarse-to-Fine Multimodal Detection Framework Based on Deep Learning for Robotic Coating Tasks* — IEEE/ASME T-Mech 31(1) 2026, 10.1109/TMECH.2025.3595263.
5. *A Feature Extraction Approach Over Workpiece Point Clouds for Robotic Welding* — IEEE T-ASE 22 (2025), 10.1109/TASE.2023.3345868. This is the LOBB descriptor paper.
6. *LWSNet: A lightweight network for automated welding point cloud segmentation* — Measurement 243 (2025) 116290.
7. *Path Planning with Automatic Seam Extraction over Point Cloud Models for Robotic Arc Welding* — Zhou, Peng, Xu, Wu, Navarro-Alarcon, arXiv:2011.11951v1 (2020).
8. *PAEAR: Point Clouds Area Exploration and Active Recognition method driven by reinforcement learning for robotic welding* — RCIM 102 (2026) 103340.
9. *RRT\*-Connect: Faster, asymptotically optimal motion planning* — ROBIO 2015, 10.1109/ROBIO.2015.7419012.
10. *SANet: A Pure Vision Strip-Aware Network with PSSCA and Multistage Fusion for Weld Seam Detection* — Zhu, Gu, Yang, Zhao, Song, Wang.
11. *Weld seam detection based on 2D image and 3D point cloud segmentation: a coarse-to-fine weld path planning method* — Ma et al., Int J Adv Manuf Technol (2025) 141:5665–5679, 10.1007/s00170-025-16967-w.
12. `WeldNet.pdf` — *WeldNet: A voxel-based deep learning network for point cloud annular weld seam detection* — Sci China Tech Sci 67(4):1215–1225 (2024), 10.1007/s11431-023-2569-1.
13. *YOLO–LaserGalvo: A Vision–Laser-Ranging System for High-Precision Welding Torch Localization* — Li, Wang, Wei.

`papers/implemented_papers/` (7 PDFs):
- `lit-lobb.pdf` — *A novel weld seam extraction method with semantic segmentation and point cloud feature for irregular structure workpieces* — RCIM 95 (2025) 102987.
- `lit-modelreg.pdf` — see §1.
- `lit-pcaslice.pdf` — see §1.
- `lit-ppf.pdf` — *Weld seam object detection system based on the fusion of 2D images and 3D point clouds using interpretable neural networks* — Sci. Rep., 10.1038/s41598-024-71989-w.
- `lit-quadric.pdf` — see §1.
- `lit-ransac.pdf` — *Weld seam extraction and path generation for robotic welding of steel structures based on 3D vision* — Autom. Constr. 183 (2026) 106792.
- `lit-regiongrow.pdf` — *Coarse-to-Fine Detection of Multiple Seams for Robotic Welding* — Wei, Cheng, Li, Song, Zhang, Zhang, arXiv:2408.10710v1 (20 Aug 2024).

---

## 3. lit-pcaslice — `scripts/baselines/lit_pcaslice.py`

### 3.1 The paper, the coarse stage, L0 and L1

**Mechanism in one sentence:** the weld cloud's own PCA principal axis sets the slicing direction; each uniform slice contributes its geometric centre; a cubic NURBS (B-spline) is fitted through the centres.

**Paper's coarse stage:** YOLO11 boxes each weld instance; an improved DeepLab V3+ (MobileNetV2 + ECA/CBAM) segments the weld pixels in the ROI; pixel mapping lifts them to a weld point cloud. The paper works from **four viewing angles**, stitched into the robot base frame (eq. 11), with voxel resampling of the overlaps.

**L0 in WeldSet** (`harness.py:275–289`):
- One mask per truth seam: `seam_region_oracle(xyz, [g], end_margin_mm=0.0)`, i.e. a 20 mm half-width band (`SEAM_REGION_HALF_WIDTH_MM = 20.0`, so 40 mm wide).
- Plus each seam's `closed` flag from `gt_meta`.

**L1:** `instance_masks=None`, so the whole cloud is one instance and "closed" is inferred by `band_is_closed`. This is the "broken-assumption arm".

### 3.2 How it works, as implemented

1. **Instance groups** (`:478–487`). Instances with fewer than `2·min_centers` = 6 points are skipped (`:495`).
2. **§4.1 statistical filter** (`:104–116`). Mean k-NN distance $\bar d_i=\frac1k\sum_{j=1}^{k}\lVert x_i-x_{(j)}\rVert$ with k = 12; keep $\bar d_i\le\mu_{\bar d}+2\sigma_{\bar d}$. **The Gaussian filter of §4.1 is not implemented anywhere.**
3. **Closed or open** (`:498`). Use the kwarg if given, otherwise `band_is_closed` (`:140–191`):
   - Band frame by PCA (`band_plane`, `:119–129`): centroid $c$, $e_1,e_2$ (two largest axes), normal = least-variance axis.
   - Polar coordinates: $\theta_i=\operatorname{atan2}((x_i-c)\cdot e_2,(x_i-c)\cdot e_1)$, $r_i$; 36 angular bins.
   - Closed iff (a) the widest empty run of bins is ≤ 40°, **and** (b) the median, over bins with at least $\max(5, 0.2\,\bar n_{bin})$ points, of $r_{10}/r_{90}$ is ≥ 0.45.
   - Requires ≥ 3·36 = 108 points.
4. **Open band, or `ring_mode="whole"`: `pca_centerline`** (`:253–281`, eqs. 5–10):
   - $\bar x=\frac1m\sum x_i$, $C=\frac{1}{m-1}\sum(x_i-\bar x)(x_i-\bar x)^\top$, $a=v_{\max}(C)$, $t_i=a^\top(x_i-\bar x)$.
   - Slice width $w=\max(3\tilde s,1\,\text{mm})$, where $\tilde s$ is the median nearest-neighbour distance (`radius_pca.mean_spacing_mm`: 2000-point sample, seed 0).
   - Slices $S_j=\{i:t_{\min}+jw\le t_i<t_{\min}+(j+1)w\}$; centre $c_j=\frac{1}{|S_j|}\sum_{i\in S_j}x_i$ if $|S_j|\ge3$.
5. **Closed band with `ring_mode="arcs"` (the default): `arc_centerline`** (`:194–250`):
   - 4 sectors $[-\pi+k\Delta-\delta/2,\;-\pi+(k+1)\Delta+\delta/2]$ with $\Delta=90^\circ$ and $\delta=15^\circ$, so each sector spans 105°.
   - `pca_centerline` runs per sector, using that sector's own PCA axis (its chord).
   - Only centres inside each sector's own 90° core are kept; all centres are then sorted by angle in the band frame.
6. **At least 3 centres are required** (`:502`).
7. **§4.3 `bspline_path`** (`:284–317`):
   - `splprep` with k = 3 and smoothing $s=0.25\,n_c$.
   - Open: 60 samples, $u\in[0,1]$.
   - Closed: periodic fit (`per=1`) on the wrapped centre sequence, 60 distinct $u\in[0,1)$, with the first sample appended, giving a closed 61-point polyline.
   - With ≤ 4 centres it returns the polyline itself (wrapped if closed).
   - The NURBS → B-spline substitution is labelled an "equivalence" (uniform weights).
8. **§4.4 torch poses** (`plan_poses=False` by default; unscored) (`:320–418`):
   - Ball of radius $r$ = `TUBE_RADIUS_MM` = 5 mm (invented). Needs ≥ 6 points.
   - MSAC: 60 iterations, $T$ = 1 mm, loss $\sum\min(d_i,T)^2$.
   - WTLSD as a **single** weighted-SVD pass with $w_i=1/(|d_i|+10^{-3})$.
   - The second plane is fitted to points more than 1 mm from the first.
   - Outward normal signs are fixed by concavity: each surface's points must lie on the other surface's positive side.
   - Eq. 12: $Z_W=-(n_b+n_m)/\lVert\cdot\rVert$ (`:407`); $X_W=Z_W\times Y_W$ (`:415`). The code takes $Y_W$ as the **central** difference $p_{i+1}-p_{i-1}$ (`:384`).
   - One-plane fallback: $Z_W=-n_{up}$ (invented).

### 3.3 Parameters and defaults

| Parameter | Default | Paper-given or invented |
|---|---|---|
| `slice_mm` | None → max(3 × spacing, 1) mm | invented; the paper gives no value |
| `filter_k` | 12 | invented |
| `filter_std` | 2.0 | invented |
| `min_slice_pts` | 3 | invented |
| `min_centers` | 3 | invented |
| `n_samples` | 60 | invented |
| `degree` | 3 | **paper** ("cubic NURBS") |
| spline smoothing `s` | 0.25 · n_centres | invented |
| `closed` | None (auto) | — |
| `band_is_closed(gap_deg, hole_frac, n_bins)` | 40°, 0.45, 36 | invented |
| `ring_mode` | "arcs" | substitution for the paper's 4-view capture |
| `n_arcs` | 4 | paper (4 viewing angles) |
| `arc_overlap_deg` | 15° | invented |
| `plan_poses` | False | — |
| `seed` | 0 | used only for poses |
| ball radius r (`TUBE_RADIUS_MM`) | 5.0 mm | **invented** (the paper keeps r symbolic) |
| MSAC `tol_mm` | 1.0 mm | invented (the paper's T is unpublished) |
| MSAC `iters` | 60 | invented |
| plane split distance / minimum ball points | 1.0 mm / 6 | invented |
| WTLSD | 1 pass, weights 1/(d+1e‑3) | deviation (paper's eq. 25 iterates to δ₀, unpublished) |

### 3.4 Fidelity audit of 2026-09-14 (`phase8_plan.md` §1.2.4; commit a98c19d)

**(a) Closed rings.** Before the audit the slicer received the whole closed band: the PCA axis lies in the ring's plane, so each slice cuts the ring twice and its centre lands in mid-air. Now closed bands are four overlapping arcs, stitched, with a periodic spline.
- Smoke test (first scene per stratum, full scan): circle 0.09 → 0.77; ellipse 0.05 → 0.80; rounded rect 0.04 → 0.57. Open seams are bit-identical.
- Batch, full-scan medians: **circle 0.06 → 0.85, ellipse 0.03 → 0.66, rounded rect 0.07 → 0.65**.
- The old reading is kept as the rung **`pcaslice_whole_ring_full_exterior` / `_single`** (`ring_mode="whole"`), which reproduces the pre-audit path exactly. Its closed-strata maximum is 0.07 (ICRA `\pcasliceWholeClosedMax`; module docstring: "F1 <= 0,07 with 25-30 mm median error").

**(b) Eq. 12 signs.** Before: $z=+(n_1+n_2)$, with $n_2$ aligned to $n_1$ whose sign was MSAC-random, and $x=t\times z$. After: $z=-(n_b+n_m)$ with outward normals, and $x=z\times y$. Poses are not scored, so no number moved.

**(c) The 5 mm ball radius.** The old comment "§4.4, 'empirically set to 5 mm'" was struck. The sentence actually comes from **Fang & Tian §3.1** ("The surface radius of the circular tube is empirically set to 5 mm"). Wang et al. keep r symbolic.

**(d) Single-pass WTLSD** is now documented.

### 3.5 Rungs and arms exposed to the harness

- Registry: `"lit-pcaslice"`, `randomised=False`, `oracle_name="band"`, polyline output.
- Aux columns: `n_instances`, `ring_mode`, `n_closed`.
- Batch chunks: `coverage_lit-pcaslice_full_exterior`, `coverage_lit-pcaslice_single`, `l1_lit-pcaslice_full_exterior`, `noise1_lit-pcaslice_single`, `noise2_lit-pcaslice_single`, `pcaslice_whole_ring_full_exterior`, `pcaslice_whole_ring_single`, `task2_lit-pcaslice_single`, `fixture_lit-pcaslice`.
- Seeds: 2 (`verify_seeds`).
- **Deterministic**: the path pipeline has no sampling; MSAC is seeded and feeds only the unscored poses.

### 3.6 Every recorded number

**Notebook 09** (2026-08-22, `out/bench`, 12/type = 60 scenes, view "full", 1 seed). Precision/recall/F1 are means; rmse and me are medians (me = max); seam_rec is a mean.

| Arm | Joint | Prec | Rec | F1 | RMSE | ME | seam_rec |
|---|---|---|---|---|---|---|---|
| L0 | T | 0.924 | 0.937 | 0.931 | 2.000 | 4.624 | 1.000 |
| L0 | butt | 0.400 | 0.364 | 0.378 | 4.417 | 5.688 | 1.000 |
| L0 | corner | 0.651 | 0.565 | 0.596 | 2.739 | 3.950 | 1.000 |
| L0 | edge | 0.000 | 0.000 | 0.000 | 9.489 | 10.039 | 1.000 |
| L0 | lap | 0.497 | 0.547 | 0.520 | 3.057 | 4.852 | 1.000 |
| L1 | T | 0.037 | 0.065 | 0.046 | 77.208 | 177.864 | 0.542 |
| L1 | butt | 0.004 | 0.008 | 0.005 | 106.689 | 213.208 | 0.542 |
| L1 | corner | 0 | 0 | 0 | 103.039 | 183.672 | 0.528 |
| L1 | edge | 0 | 0 | 0 | 93.091 | 161.319 | 0.958 |
| L1 | lap | 0.029 | 0.047 | 0.035 | 76.611 | 162.595 | 0.500 |

**dataset_plan "first numbers"** (dated 2026-08-21): "near-ceiling on straight seams (F1 0,95–1,0)". **This does not match notebook 09's stored table** (see §8).

**Module docstring:**
- `band_is_closed` ratio: 0.53–0.61 on closed circle/ellipse/rounded_rect L0 bands; 0.21–0.34 on open line/swept bands; 0.93 on a 6 mm synthetic ring.
- Whole ring: F1 ≤ 0.07, 25–30 mm median error.

**Fixture twins:**
- dataset_plan, 2026-09-07 (40 pairs, pre-audit): ΔF1 −0.24 (butt −0.36, edge −0.30).
- nb15, post-audit (40 pairs): dF1_med −0.20, dPrec −0.18, extra seams −1. By joint type: T −0.01, corner −0.08, butt −0.01, lap −0.23, edge −0.30.
- dataset_plan step 3 (grooved stratum rebuild): single-view 0.49 → 0.47.

**handoff_phase8 §3** (pre-visfix, pre-audit table): T/line 0.96, circle 0.06, ellipse 0.03, saddle 0.04, rounded 0.06, swept 0.92, corner 0.65, butt 0.54, grooved 0.00, arc 0.01, lap 0.84, edge 0.37.

**nb15, post-audit (vintage E), median F1 per stratum:**

| Condition | T/line | circle | ellipse | saddle | rounded | swept | corner | butt | grooved | arc | lap | edge |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| full_exterior | 0.96 | 0.85 | 0.66 | 0.50 | 0.65 | 0.92 | 0.65 | 0.54 | 0.00 | 0.01 | 0.84 | 0.37 |
| single | 0.02 | 0.04 | 0.01 | 0.00 | 0.04 | 0.00 | 0.01 | 0.63 | 0.47 | 0.73 | 0.09 | 0.24 |

Other nb15 / ICRA numbers:
- Straight → curved T: 0.96 → 0.72.
- T/line median RMSE: 2.21 mm.
- L0/L1 by joint type: T 0.78/0.00, butt 0.01/0.00, corner 0.65/0.00, edge 0.37/0.04, lap 0.84/0.03. ICRA pooled L0 0.64 → L1 0.00.
- Noise (single view, median F1 at 0/1×/2×σ): 0.05 / 0.07 / 0.06.
- Task 2: selection 1.00 in every class (an oracle artefact, as nb15 itself states); localization RMSE T 9.7, corner 6.8, butt 4.1, lap 5.3, edge 8.7 mm.
- ISO 17659 split: T/line 0.97 (T-joint) vs 0.95 (angle joint); curved 0.82 vs 0.59.
- Runtime (median s): L0 full 0.47, single 0.22, L1 0.91.
- Max seed spread: 0.0.
- ICRA macros: closed max 0.85 / min 0.65; swept 0.92; arc full 0.01 / single 0.73; grooved single 0.47.

### 3.7 Known limitations and failure modes

- **Two seams in one mask:** each slice centre is the midpoint of neither seam. Pinned by a test with strips 8 mm apart; centres land at about 4 mm. This is why L1 collapses.
- **A whole closed ring** gives mid-air centres. Handled by arcs; priced by the whole-ring rung.
- `band_is_closed` is an inference: a band nearly as wide as its own radius defeats it. L0 passes the true flag.
- Sectors are cut geometrically, so the paper's hand-eye stitching error is not modelled.
- **Fixture points inside the per-instance band shift the slice centres** (ΔF1 above).
- On straight seams the method is "nearly a ruler".
- **Single-view collapse on T strata** (0.00–0.09) is recorded but not explained in the sources I read.

### 3.8 Code ↔ docs discrepancies

- `statistical_filter`'s docstring says "The noisy arm gets the same treatment their scan does", but **no Gaussian filter exists for any arm**.
- The docstring says "Eq. (12) exactly: Y_W is the tangent O_Fi → O_Fi+1", but **the code uses a central difference**.
- **Eq. 12 as printed ($X=Z\times Y$) produces a left-handed frame**: $\det[X,Y,Z]=-\lVert Y\times Z\rVert<0$. The code reproduces it faithfully and the test pins `x = z × y`. Before the audit the frame was right-handed but had a random-signed Z.
- MSAC loss: the code squares the truncated distance. pdftotext of the paper's eq. 14 reads linear, ρ(e,T) = e for e < T, else T. Please check visually.
- WTLSD: the paper uses an EIV fit of $z=ax+by+c$ with normalised distance weights (eqs. 22–23). The code uses a weighted PCA plane. This only affects the unscored poses.
- Notebook 09's header table still says "**5 mm tube radius (§4.4)**", which is stale and wrong (see §3.4c).
- `params["slice_mm"]` logs `None` when the default is used, so the actual slice width is not recorded.
- dataset_plan says "6 tests" and "IMPLEMENTED 2026-08-21". There are now 12 tests, and the implementation commit and notebook run are dated 2026-08-22.

### 3.9 What the tests pin (`tests/test_lit_pcaslice.py`, 12 functions)

1. The slicing direction is rotation-equivariant (37° rotation).
2. Centres follow a curved strip to within 1.5 mm.
3. Two strips in one mask give centres at y ≈ 4 (±1.5 mm); two instance masks give two seams at y ≈ 0 and 8 (±1 mm).
4. The statistical filter removes spikes.
5. The path is bit-identical across runs, the registry says not randomised, and an MSAC plane recovers |n_z| > 0.99.
6. A harness run on an `out/bench` T scene gives 2 rows with one F1 value and `n_instances == n_gt` (skipped without the corpus).
7. A 60 mm ring (6 mm wide, 1 pt/mm², 0.1 mm noise) is auto-detected as closed; the path is closed, covers all 36 10° bins, has radial RMSE < 0.5 mm, |z| < 0.5 mm, and F1 at 1 mm > 0.9.
8. The `closed` kwarg wins over inference; `closed=False` gives a median radial error > 10 mm.
9. `ring_mode="whole"` gives mid-air centres (median radial error > 10 mm, median radius < 40 mm) while "arcs" stays < 1 mm.
10. An open strip is untouched by any of this and identical across modes.
11. The periodic spline returns 81 points and is closed with deviation < 1.5 mm; the open fit leaves a > 10 mm gap.
12. Eq. 12 signs on a 90° fillet: x = normalise(z × y), y along the seam, and z·(outward bisector) < 0.

---

## 4. lit-modelreg — `scripts/baselines/lit_modelreg.py`

### 4.1 The paper, the coarse stage, L0 and L1

**Mechanism in one sentence:** a basic model (seam P, auxiliary trajectory P′, edge set W) is registered onto the target's feature point set by Bayesian CPD (similarity plus coherent deformation, eq. 3), and the seam is **transferred, never detected**. It is then B-spline fitted (§3.3), and the torch pose comes from P and P′ (§3.4).

**Paper's front end: there is no learned stage.**
- The model comes from CAD.
- Target features come from 3D-model conversion (offline experiment), or from the scan after height-threshold segmentation plus "the point cloud edge detection algorithm [23]" (Demarsin et al., CAD 39(4) 2007) in the online experiment.

**L0 in WeldSet:** the model is rebuilt from `scene.json` plus the truth polylines ("CAD-by-construction", constitutively L0). Default target features: `"oracle"`.

**There is no L1.** `_run_lit_modelreg` ignores the oracle flag (`harness.py:292–305`).

### 4.2 How it works, as implemented

1. **`build_model`** (`:262–302`), in the joint frame $T_{wj}^{-1}$:
   - Slab and prism parts are sampled analytically. Edges at step min(step, 4) mm (`:115–142`, `:192–213`); surfaces as a grid at `step`.
   - Since 2026-09-14, tubes, swept slabs and prepared plates come from their exact D34 meshes: edges where the dihedral angle exceeds 25° (`:242–259`), surfaces by area-uniform sampling with seed 0 (`:227–239`).
   - $P = T_{wj}^{-1}\,\gamma_{truth}$.
   - **P′ is never built** (`AUX_OFFSET_MM = 15` is defined at `:87` and unused).
   - **The registered set is $Y=W$ only.** The paper uses $Y=P\cup W$.
2. **Target X** (`:607`, `:618–641`). Base: voxel 6 mm, then a stride cap of 900 points (`_cap`). Then, by `target_features`:
   - `"oracle"`: scan points (strided to about 60k) within 2 × 6 = 12 mm of the **true-posed model edges**, voxel 3 mm, cap 900. Used only if ≥ 30 points.
   - `"edges"`: `sharp_edge_points` (`:532–552`). PCA normals over k = 16 neighbours; a point is an edge iff $\min_j|n_i\cdot n_j|<\cos30^\circ$. Then voxel 3 mm, cap 900.
   - `"dense"`: the base cloud.
3. **Model side Y** (`:605–609`): `edges_only=(target_features=="oracle")`. So **Y is model edges only for "oracle"**; for both "dense" **and "edges"** it is dense model surfaces (voxel 6 mm, cap 900).
4. **Initialisation:**
   - `"near"` (default, `:642–646`, `_near_init_pose` `:509–529`): $T_0=T_{wj}\,T^{-1}$ with $R$ = 10° about a random axis and $t\sim20\,\text{mm}\cdot\mathcal N(0,I)/\sqrt3$, drawn from `default_rng(abs(hash(scene_id)) % 2**32)`. Y and P are pre-posed by $T_0$. Then `cpd_similarity(global_init=False)`: 5 EM steps from the identity, then 60.
   - `"global"` (`:647–648`): `_pca_init` (`:309–345`). Principal frames sorted descending, third-moment sign fix, det > 0. Four proper-rotation candidates $R=V_XFV_Y^\top$ with $F\in\{\mathrm{diag}(1,1,1),(1,-1,-1),(-1,1,-1),(-1,-1,1)\}$, $s=\sigma_X/\sigma_Y$, $t=c_X-sRc_Y$. Each is refined for max(15, iters//3) = 20 EM steps; the best by mean nearest-neighbour squared distance wins; then 60 more steps.
5. **Similarity CPD** (`_em_step`, `:395–423`; Myronenko):
   - E-step: $P_{nm}=\dfrac{e^{-\lVert x_n-T(y_m)\rVert^2/2\sigma^2}}{\sum_{m'}e^{-\lVert x_n-T(y_{m'})\rVert^2/2\sigma^2}+c}$ with $c=(2\pi\sigma^2)^{3/2}\frac{w}{1-w}\frac{M}{N}$.
   - M-step: $\mu_x=\frac{X^\top P1}{N_P}$, $\mu_y=\frac{Y^\top P^\top1}{N_P}$; $A=\hat X^\top P\hat Y=USV^\top$; $R=U\,\mathrm{diag}(1,1,\det UV^\top)V^\top$; $s=\mathrm{tr}(SC)/\mathrm{tr}(\hat Y^\top d(P^\top1)\hat Y)$; $t=\mu_x-sR\mu_y$; $\sigma^2=[\mathrm{tr}(\hat X^\top d(P1)\hat X)-s\,\mathrm{tr}(SC)]/(3N_P)$.
   - $\sigma^2$ is initialised from the **nearest-neighbour** mean squared residual, with a floor of 1 mm².
6. **Coherent deformation** (`cpd_nonrigid`, `:426–462`; `mode="nonrigid"` default): $G_{ij}=e^{-\lVert y_i-y_j\rVert^2/2\beta^2}$; solve $(d(P^\top1)G+\lambda\sigma^2I)W=P^\top X-d(P^\top1)Y$; $T(Y)=Y+GW$. Runs max(8, iters//3) = 20 iterations.
7. **Seam transfer** (`:651–657`): $p\mapsto sRp+t$, then $p\mapsto p+G(p,\tilde Y)W$ (`_apply_field`).
8. **§3.3 spline** (`spline_seam`, `:555–575`; `spline=True`):
   - Cubic `splprep` with $s=0.05^2 n_{sub}$ ("~0.05 mm RMS"); $n_{ctrl}$ = clip(L/10, 8, 200); 4·n_ctrl subsamples; ~1 mm sampling.
   - Periodic for closed seams. The closing segment is not repeated.
   - The closed flag is read from `scene["seams"][i]`, i.e. indexed over **all** seams, while P holds the **filtered** gt list (see §4.8).
9. Returns the seams, the registered model TY, s, R, t and params; `used_model_oracle=True`. §3.4 torch pose is **not implemented**.

### 4.3 Parameters and defaults

| Parameter | Default | Paper-given or invented |
|---|---|---|
| `mode` | "nonrigid" | classical CPD substituted for the paper's Bayesian CPD |
| `init` | "near" | — |
| `init_rot_deg` / `init_trans_mm` | 10° / 20 mm | invented envelope |
| `target_features` | "oracle" | truth-guided stand-in |
| `model_step_mm` | 5.0 | edges at 4 mm; **the paper uses 1 mm** |
| `target_voxel_mm` | 6.0 | 3 mm for oracle/edges; **the paper uses a 1 mm voxel** |
| `n_max` | 900 | invented (computational) |
| `beta_mm` | 40.0 | invented; **the paper's β (1.2 / 0.9 / 1.8) is not used or mentioned** |
| `lam` | 3.0 | invented; **the paper's λ (200) is not used or mentioned** |
| `iters` | 60 | 20 for the nonrigid stage; **the paper uses 500 iterations with a 1e‑6 tolerance** |
| `w` (outlier weight) | 0.1 | invented |
| `spline` | True | the spline itself is the paper's; its constants are invented |
| `edge_angle_deg` | 30° | invented |
| `sharp_edge_points` k / `max_points` | 16 / 120 000 | invented |
| mesh-edge dihedral | 25° | invented |
| `AUX_OFFSET_MM` (l_p) | 15 | **paper**, but unused |

Not implemented from the paper: the 5 mm tube radius for Z_S, l_w = 10 mm, R_z = 10, the ω angle.

### 4.4 Fidelity audit of 2026-09-14 (§1.2.2b; reviewed by the user)

**Every stratum.** Before, the model was plate-only (slab/prism) and the chunks ran on the plate subset. Now D34 meshes are used for the other primitives.
- Smoke test, one scene per stratum, full / single F1: circle 0.84/0.00, saddle 1.00/0.00, ellipse 0.82/0.86, swept 1.00/0.50, rounded rect 0.76/0.10, grooved 0.23/0.00, arc 0.94/0.58.

**§3.3 B-spline added**, and **classical edge features added** as the rung `modelreg_edge_features`.
- Smoke test, full-scan F1 for oracle / edges / dense: T line 0.99/0.98/0.98; T circle 1.00/1.00/1.00; butt square 0.99/1.00/0.23; corner 0.99/**0.06**/0.27.

**All nine M6 chunks were deleted** (archived to `pre_meshmodel_modelreg_chunks/`) and recomputed over 720 scenes.

**Rung names:** the old headline reading `"oracle"` is still the headline. `modelreg_dense_features` and `modelreg_global_init` are kept. The no-spline, plate-only reading is **not** kept as a rung (`spline=False` exists as a kwarg but is not reachable through the harness).

### 4.5 Rungs and arms exposed to the harness

- Registry: `"lit-modelreg"`, `randomised=False`, `oracle_name="model"`.
- The runner accepts **only** `mode`, `target_features` and `init`. It has no `**kw` and ignores the seed.
- Aux columns: `reg_scale`, `mode`, `target_features`.
- Batch chunks: `coverage_lit-modelreg_full_exterior`, `coverage_lit-modelreg_single`, `noise1_lit-modelreg_single`, `noise2_lit-modelreg_single`, `modelreg_dense_features`, `modelreg_global_init`, `modelreg_edge_features`, `task2_lit-modelreg_single`, `fixture_lit-modelreg`. There is no `l1_` chunk.
- Deterministic within one process; **not bit-reproducible across processes** (§4.8).

### 4.6 Every recorded number

**Notebook 10** (2026-08-22, `out/bench`, 12/type, view "full", pre-spline, plates only). Precision/recall/F1 are means; RMSE and ME are medians.

| Arm | Joint | Prec | Rec | F1 | RMSE | ME |
|---|---|---|---|---|---|---|
| L0-oracle | T | 0.998 | 0.967 | 0.982 | 0.652 | 1.061 |
| L0-oracle | butt | 0.773 | 0.750 | 0.761 | 1.340 | 2.014 |
| L0-oracle | corner | 0.969 | 0.955 | 0.962 | 0.691 | 1.161 |
| L0-oracle | edge | 0.264 | 0.256 | 0.260 | 5.091 | 5.779 |
| L0-oracle | lap | 0.619 | 0.592 | 0.605 | 2.594 | 3.714 |
| L0-dense | T | 0.745 | 0.694 | 0.718 | 1.916 | 3.804 |
| L0-dense | butt | 0.101 | 0.094 | 0.097 | 8.393 | 13.436 |
| L0-dense | corner | 0.783 | 0.734 | 0.758 | 2.094 | 3.822 |
| L0-dense | edge | 0.234 | 0.230 | 0.231 | 7.586 | 12.551 |
| L0-dense | lap | 0.263 | 0.237 | 0.249 | 8.292 | 14.362 |

**dataset_plan "first numbers"** (15/type, L0-oracle, near): "T 1,00 / corner 1,00 / butt 0,98 / lap 0,99 at RMSE 0,5–1,6 mm … edge and one lap at ~5 mm". This does not match notebook 10's table.

**Docstrings and notebook 10 prose:**
- Scale collapses to 0.14 when σ² is initialised from the all-pairs mean (from an initialisation already within 2.7°).
- A lap stack slides ~5 mm with dense surfaces.
- Third-moment sign ambiguity: 35–108 mm residual on a model that overlays at 0.24 mm.
- Symmetric counterpart under global init: **119 mm** on an edge joint.
- About 700 points a side registers a slab assembly to sub-millimetre in under a second.

**handoff_phase8 §3** (pre-audit): T/line 0.99, corner 0.99, butt 0.97, lap 0.69, edge 0.32; curved strata "—".

**nb15, post-audit (vintage E), median F1 per stratum:**

| Condition | T/line | circle | ellipse | saddle | rounded | swept | corner | butt | grooved | arc | lap | edge |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| full_exterior | 0.99 | 0.83 | 1.00 | 1.00 | 0.74 | 0.99 | 0.99 | 0.98 | 0.97 | 0.95 | 0.67 | 0.36 |
| single | 0.31 | 0.31 | 0.05 | 0.16 | 0.11 | 0.49 | 0.16 | 0.96 | 0.52 | 0.63 | 0.23 | 0.14 |

Other nb15 / ICRA numbers:
- Straight → curved T: 0.99 → 0.91 (ICRA `\modelregCurvedMean` 0.91).
- T/line RMSE: 0.70 mm.
- By joint type, L0 oracle/near: T 0.99, corner 0.99, butt 0.97, lap 0.67, edge 0.36.
- `modelreg_dense_features`: T 0.77, corner 0.87, butt 0.15, lap 0.18, edge 0.20.
- `modelreg_global_init`: T 0.98, corner 0.98, butt 0.78, lap 0.57, edge 0.31.
- Task 2: selection 1.00 everywhere (oracle artefact); localization T 4.5, corner 4.7, butt 1.9, lap 4.3, edge 6.4 mm.
- Fixture: dF1 +0.02 (lap +0.12, edge +0.30). dataset_plan: "unmoved".
- ISO 17659 split: T/line 0.98 vs 0.99; curved 0.94 vs 1.00.
- Runtime: 5.01 s (full), 4.59 s (single).
- ICRA macros: grooved 0.97, arc 0.95, `\modelregDenseButt` 0.15, `\modelregButt` 0.97.
- Modelreg is excluded from the noise table.

### 4.7 Known limitations and failure modes

- It is a **registration ceiling, never a detection**: every number is "L0-with-CAD".
- Sliding along a soft direction:
  - with dense features, a lap slides ~5 mm;
  - thin flush stacks (edge, lap) slide even with the oracle features;
  - with classical edge features, a corner slides (0.06).
- **Symmetric-counterpart failure** outside the roughly-positioned envelope (global init). The seam lands a plate-length away (119 mm).
- Near-symmetric partial clouds slide in single view (the zeros on curved strata).
- The fit-up arm (a nominal model against a defective scan) is unbuilt.

### 4.8 Code ↔ docs discrepancies (several are substantive)

1. **Reproducibility.** `_near_init_pose` seeds from Python's `hash(str)`, which is salted per process. `PYTHONHASHSEED` is not set anywhere in the repo or the environment. I confirmed the salting: three runs of `print(abs(hash('T_000123')) % 2**32)` printed 1361824948, 685616517 and 1211031569. This contradicts the docstring's "(seeded from `scene_id`, reproducible)" and "Deterministic". The harness's zero spread (both seeds run in one process) cannot detect it.
2. **The `"edges"` rung registers scan edges against a dense-surface model**, because `edges_only` is True only for "oracle" (`:605–609`). The paper registers edges against edges. This is a likely oversight, and it bears on `modelreg_edge_features` (for example corner 0.06).
3. **Y = W only, not P ∪ W** as in the paper. This is undocumented.
4. **The docstring claims P′ is built and §3.4 is "carried".** P′ is never built and `AUX_OFFSET_MM` is unused.
5. **Docstring self-contradiction:** lines 49–52 say the edge-feature "extraction from a raw scan it never specifies". Lines 63–69, and the paper's online experiment ("point cloud edge detection algorithm [23]"), contradict this.
6. **The paper's parameters** (β, λ, 500 iterations, 1e‑6 tolerance, 1 mm sampling) are not used and not mentioned.
7. **"EM from a fixed PCA-based initialisation"** describes only `init="global"`. The default `"near"` arm starts from the identity on a perturbed model.
8. **Latent spline closed-flag index bug.** It is harmless on every corpus present: 0 mismatches across bench_phase4 (720), bench_phase4_fx (274, laptop copy), bench (448) and bench_fx (511), checked for both the primary and the weldable gt lists.
9. MANUAL §8.1 calls "dense features, global init" "its two published arms". They are not published arms, and the `edges` rung (the paper's own stage) is missing.
10. The batch `--list` estimate still counts only the plate subset.
11. dataset_plan says "IMPLEMENTED 2026-08-21, 5 tests". The commit is dated 08-22, and there are now 8 tests.

### 4.9 What the tests pin (`tests/test_lit_modelreg.py`, 8 functions)

1. The model posed back onto a T scan has median nearest-neighbour distance < 1 mm, and the seam round-trips to < 1e‑9.
2. The slab samplers cover faces and edges.
3. CPD recovers s = 1.1, a 25° rotation and t = (30, −12, 7) on a notched slab: |Δs| < 0.01, angle < 0.5°, |Δt| < 0.5 mm. This is the regression pin for the σ² bug.
4. On a lap scene in view "full": RMSE(oracle) < 2 mm and RMSE(dense) > RMSE(oracle).
5. Deterministic: 2 rows with one F1 value; `randomised=False`; `oracle_name="model"`.
6. Tube, swept and prepared-prism models built from meshes lie on the parts: 99th percentile < 0.3 mm, and |W_surf| > |W_edges| > 20.
7. `spline_seam` stays within 0.2 mm of the polyline, open and closed.
8. `sharp_edge_points` finds box edges: between 0 and 35 % of the points, with > 90 % within 4 mm of an edge.

---

## 5. lit-quadric — `scripts/baselines/lit_quadric.py`

### 5.1 The paper, the coarse stage, L0 and L1

**Mechanism in one sentence:** PCA-normal statistics classify each welding surface as flat or curved; it is fitted by a TLS plane or a general quadric; the seam is the cloud points within a threshold of the fitted intersection, ordered by a graph walk and a distance sort.

**Paper's coarse stage:** region growing [19] splits the cloud into hub and blade. That gives two parts with one welding surface each, from a single camera.

**L0 in WeldSet:** per-face surface labels (`cloud.npz:face_id`) **plus** part membership (`object_id`). The pair rule is "surfaces from different parts". This is the richest oracle in the ladder.

**L1:** surfaces are grown by Wei et al.'s Alg. 1 (20°, curvature 0.10, ≥ 40 points). There is no part membership, so every pair is a candidate.

### 5.2 How it works, as implemented

1. **Pre-treatment:** optional Gaussian smoothing (off; `:178–196`, σ = r/2). Voxel-centroid downsampling at 1.5 mm. $\tau$ = `seam_tol_mm` = 1.5 × voxel = **2.25 mm** (`:594`).
2. **Labels.** At L0, face_id and object_id are carried to the voxel points by nearest raw point (`_labels_to`). At L1, `local_pca` (k = 20) is computed over the whole cloud, then `region_grow`.
3. **Per surface with ≥ 40 points** (`:629–646`): PCA normals on the **segmented surface** (k = min(20, n)) are folded into one hemisphere, $n_i\leftarrow n_i\,\mathrm{sgn}(n_i\cdot n_0)$, with $\mu=\bar n/\lVert\bar n\rVert$.
   - `"per_axis"` (default): $\delta_{i,a}=|\arccos n_{i,a}-\arccos\mu_a|$; flat iff $\min_a\frac1N\sum_i\mathbb 1[\delta_{i,a}\le10^\circ]\ge0.9$.
   - `"angle"`: $\delta_i=\arccos|n_i\cdot\mu|$; flat iff $\frac1N\sum\mathbb1[\delta_i\le10^\circ]\ge0.9$.
   - The literal σ statistic is stored alongside (`within_sigma`, `within_sigma_axis`).
4. **Fits:**
   - **TLS plane** (eqs. 7–11 as intended by their ref. 21): $c=\bar p$, $n$ = right singular vector of $P-c$ with the smallest singular value, $f(p)=n\cdot(p-c)$.
   - **Homogeneous quadric** (eq. 12): $q=(p-c)/s$ with $s=\sqrt{\frac1N\sum\lVert p_i-c\rVert^2}$; $D=[x^2,y^2,z^2,xy,xz,yz,x,y,z,1]$; $\theta=\arg\min_{\lVert\theta\rVert=1}\lVert D\theta\rVert$ (smallest right singular vector); $\nabla f=\frac1s[2Ax+Dy+Ez+G,\;2By+Dx+Fz+H,\;2Cz+Ex+Fy+I]$.
   - Sampson distance $d=|f|/\lVert\nabla f\rVert$.
   - Newton projection $p\leftarrow p-\frac{f}{\lVert\nabla f\rVert^2}\nabla f$: one step for a plane, three for a quadric.
5. **Seam points** (`seam_points`, `:364–411`). A point $a$ of surface A qualifies if:
   - (i) $d_B(a)\le\tau$;
   - (ii) its foot $\mathrm{proj}_B(a)$ lies within $\tau$ of B's actual points (the **finite patch** test);
   - (iii) after 12 alternating projections $p\leftarrow \mathrm{proj}_B(\mathrm{proj}_A(p))$, the result is finite and $\lVert p-a\rVert\le\tau$.
   - The same applies symmetrically to B. Points that fail are **dropped**, never emitted unprojected.
   - It returns paired (measured, projected) arrays.
6. **Pair gates, in order** (`:653–684`):
   - `same_part` (L0);
   - `no_band`;
   - angle $\varphi=\arccos(\mathrm{median}|\hat g_A\cdot\hat g_B|)$ over the projected band;
   - `coplanar` (both flat and φ < 10°);
   - `tangent` (φ < 10°);
   - `too_few` (< 8 points);
   - `too_short` (2 × max distance from the centroid < 10 mm);
   - otherwise `seam`.
   - The census is stored in `pairs`.
7. **Ordering** (`order_points`, `:421–482`):
   - A greedy nearest-neighbour walk from a start point visits every point; the **last point reached** is the initial point $x_0$. The paper's prose describes this walk. Its eq. 13, $d(v)=\min(d(v),d(u)+w(u,v))$, is a Dijkstra-type relaxation that is not literally implemented.
   - The start is `"farthest"` from the centroid by default; the paper says random.
   - `"distance"` (published): stable sort by $\lVert x-x_0\rVert$.
   - `"chain"` (corrected): nearest-neighbour chain from $x_0$.
8. **Output:** `"points"` (default, the paper's) returns the ordered measured points; `"projected"` returns their images on the analytic intersection.

**The three intersection types** (plane∩plane, plane∩cylinder, cylinder∩cylinder) come back exact on exact geometry, on the projected rung.

### 5.3 Parameters and defaults

| Parameter | Default | Paper-given or invented |
|---|---|---|
| `VOXEL_MM` | 1.5 | invented |
| `gaussian_radius_mm` | None | the paper uses a Gaussian filter (radius unpublished) |
| `K_NEIGHBORS` (k) | 20 | invented; the docstring says "PCL's default", which is unverifiable |
| `FLAT_FRACTION` | 0.90 | **paper** |
| `FLAT_ANGLE_DEG` | 10° | **paper** (§4.2) |
| `FLAT_TEST` | "per_axis" | the paper's Fig. 4 reading |
| `seam_tol_mm` | None → 2.25 mm | invented; also used as patch and path tolerance |
| alternating projections / Newton steps | 12 / 3 | invented |
| `min_surface_pts` | 40 | invented |
| `min_seam_pts` | 8 | invented |
| `min_seam_length_mm` | 10 | invented |
| `coplanar_deg` | 10° | invented |
| `ordering` | "distance" | **the default in both `detect` and `_run_lit_quadric`**; the batch passes "chain" |
| `output` | "points" | **paper** |
| `seed` | 0 | no effect |
| `smoothness_deg` | 20° | Wei et al.'s published value |
| `curvature_thresh` | 0.10 | invented, forced by thin plates |

### 5.4 Fidelity audit of 2026-09-14

**Points instead of projections.** Before the audit, the emitted points were projected onto the intersection: RMSE about 0.01 mm. Pre-audit code accepted a projection up to 3·τ and fell back to the raw point otherwise. Now the measured points are emitted.
- Smoke test: RMSE 0.02–0.2 → 1.3–1.8 mm, F1 within 0.03.
- Batch: **ring/saddle RMSE 0.01 → 1.7–2.5 mm** (ICRA: 1.67–2.53, projected rung 0.011–0.01); pooled L0 0.88 → 0.87.
- Module docstring, per scene (projected vs points): T/line 0.18/1.46, T/circle 0.19/1.84, T/saddle 0.02/1.26, butt/line_square 1.04/1.31, lap/line 0.41/1.25; |Δf1| ≤ 0.03. On a pipe-on-plate scene the error is 2.4 mm at τ = 2.25 and 0.6 mm at τ = 1.

**Per-axis flat test (Fig. 4) added.** It changed no verdict: 52 surfaces on 5 scenes were checked.

**The docstring now states** that the oracle is surfaces plus parts and that the headline uses the chain ordering.

**Rung names kept for old readings:**
- **`quadric_projected_full_exterior` / `_single`** (chain + projected). This is **not an exact replay** of the pre-audit default: the tolerance is now τ rather than 3τ, there is no raw fallback, and the flat test is per-axis.
- **`quadric_distance_full_exterior` / `_single`**: the published ordering, with points output.
- Before the 2026-09-09 swap (commit 355e550), the rung was named **`quadric_chain_*`** and "distance" was the headline.
- Not in the batch: `flat_test="angle"`, and `start="random"` (which `detect` cannot reach at all).

### 5.5 Rungs and arms exposed to the harness

- Registry: `"lit-quadric"`, `randomised=False`, `oracle_name="surfaces"`.
- The runner accepts `ordering` plus `**kw`.
- Aux columns: `n_surfaces`, `ordering`, `output`, `flat_test`, `n_curved`, `pairs_seam`.
- Batch chunks:
  - `coverage_lit-quadric_full_exterior` and `coverage_lit-quadric_single`, run with `{"ordering": "chain", "output": "points"}`;
  - `l1_lit-quadric_full_exterior`;
  - `noise1_lit-quadric_single`, `noise2_lit-quadric_single`;
  - `quadric_distance_*`, `quadric_projected_*`;
  - `task2_lit-quadric_single`;
  - `fixture_lit-quadric`.
- Fully deterministic; the seed is inert.

### 5.6 Every recorded number

**Notebook 11** (2026-09-09; `bench_phase4`, pre-visfix; pre-audit, i.e. projected output and the angle test; 2 scenes per family). Cell 14: 120 rows in 550 s. Median F1:

| View | Arm | line | circle | ellipse | saddle | rounded_rect | swept | arc_butt | butt | corner | lap |
|---|---|---|---|---|---|---|---|---|---|---|---|
| full | L0 chain | 0.50 | 0.87 | 1.00 | 0.83 | 0.74 | 0.98 | 0.76 | 0.50 | 1.0 | 0.80 |
| full | L0 distance | 0.50 | 0.40 | 0.48 | 0.40 | 0.28 | 0.98 | 0.77 | 0.50 | 1.0 | 0.80 |
| full | L1 chain | 0.74 | 0.41 | 0.50 | 0.00 | 0.23 | 0.36 | 0.00 | 0.00 | 0.5 | 0.07 |
| single | L0 chain | 0.50 | 0.56 | 0.32 | 0.44 | 0.38 | 0.64 | 0.78 | 0.33 | 1.0 | 0.60 |
| single | L0 distance | 0.50 | 0.56 | 0.32 | 0.44 | 0.38 | 0.64 | 0.78 | 0.33 | 1.0 | 0.60 |
| single | L1 chain | 0.00 | 0.07 | 0.07 | 0.00 | 0.17 | 0.35 | 0.41 | 0.00 | 0.0 | 0.00 |

Other notebook 11 outputs:
- Cell 6: plate TLS RMS residual 0.0000, normal [0, 0, 1]. Tube quadric RMS 0.0000; normalised coefficients x² −1.00, y² −1.00, z² −0.00, constant +0.37.
- Cell 8 (exact geometry): plane∩plane 180 points, max error 0.0; plane∩cylinder 120 points, 1.78e‑14; cylinder∩cylinder 306 points, 9.40e‑08.
- Cell 12 (T, census): full_exterior {same_part 25, no_band 18, too_few 2, too_short 4, seam 4, tangent 2}: two true fillets plus two far-side mirrors; bands 180/194/192/180 points; extents 114/112/114/115 mm. Single view {same_part 1, no_band 1, seam 1}: band 222, extent 113 mm. Truth: 2 fillets of 116 mm.
- Cell 22 ("against the paper's own metric", L0 chain, full, **projected era**): circle 3 seams, RMSE median 0.008 / max 0.014, ME max 0.096; ellipse 4 seams, 0.011/0.023/0.148; line 2 seams, 0.180/0.200/1.972; saddle 2 seams, 0.011/0.013/0.048 mm.
- Cell 24: zero spread on 4 scenes.
- Markdown: plate ≈ 1.0 within 10°, tube ≈ 0.1; the literal within-σ share is 0.19 on a plane with 0.05 mm jitter. Without the patch test there are 15 "seams" from 55 face pairs. A T with a 3.1 mm gap is missed at τ = 2.25.

**dataset_plan step 6** (2026-09-08): ring 0.01 mm RMSE; full-view F1 0.86 (chain) vs 0.38 (published).

**Batch 2026-09-09 (C)**, quoted in dataset_plan, MANUAL §9 and the handoff:
- L0 pooled 0.93; circle 1.00, ellipse 0.96, saddle 0.85 "at 0,01 mm"; butt 0.99, edge 0.90, lap 0.88.
- L1 0.00 (only rounded_rect 0.53 and swept 0.72 survive).
- Noise 0.56 → 0.49 → 0.57.
- Task 2: 0.0–0.7 mm, selection 0.49–0.91.
- Fixture ΔF1 −0.39.
- After the visfix the pooled L0 went 0.90 → 0.88.

**nb15, post-audit (vintage E), median F1 per stratum:**

| Condition | T/line | circle | ellipse | saddle | rounded | swept | corner | butt | grooved | arc | lap | edge |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| full_exterior | 0.99 | 0.97 | 0.88 | 0.91 | 0.55 | 0.98 | 1.00 | 0.99 | 0.00 | 0.68 | 0.87 | 0.89 |
| single | 0.66 | 0.50 | 0.59 | 0.66 | 0.44 | 0.63 | 1.00 | 0.60 | 0.00 | 0.93 | 0.58 | 0.43 |

**Published distance ordering vs chain (full_exterior):** T/line 0.99/0.99; circle 0.42/0.97; ellipse 0.39/0.88; saddle 0.50/0.91; rounded 0.31/0.55; swept 0.94/0.98; corner 1.00/1.00; butt 0.99/0.99; grooved 0.00/0.00; arc 0.57/0.68; lap 0.89/0.87; edge 0.91/0.89.

**Single view, distance vs chain:** T/line 0.66/0.66; circle 0.45/0.50; ellipse 0.53/0.59; saddle 0.60/0.66; rounded 0.41/0.44; swept 0.59/0.63; corner 1.00/1.00; butt 0.60/0.60; grooved 0.00/0.00; arc 0.93/0.93; lap 0.58/0.58; edge 0.43/0.43.

Other nb15 / ICRA numbers:
- L0/L1 by joint type: T 0.93/0.00, butt 0.61/0.00, corner 1.00/0.00, edge 0.89/0.00, lap 0.87/0.00. ICRA pooled 0.87 → 0.00.
- Straight → curved T: 0.99 → 0.86.
- T/line RMSE 1.46 mm; ICRA pooled RMSE 1.60 mm.
- Noise (single view): 0.58 / 0.44 / 0.50.
- Task 2: selection T 0.92, corner 0.25, butt 0.46, lap 0.80, edge 0.77 (ICRA mean 0.64); localization 1.5, 1.2, 1.3, 1.4, 1.3 mm.
- Fixture: dF1 −0.29, dPrec −0.47, +4 seams. By joint type: T −0.33, corner −0.42, butt 0.00, lap −0.48, edge −0.26.
- ISO 17659 split: T/line 0.99 vs 0.98; curved 0.94 vs 0.90.
- Runtime: L0 1.00 s (full), 0.33 s (single); L1 2.70 s.
- ICRA macros: best-or-tied detector on 10 strata; curved recall minimum 0.58; ring F1 minimum 0.88; precision 0.51 on rounded_rect and arc.

### 5.7 Known limitations and failure modes

- **The published distance sort folds closed or strongly curved seams.** The test pins a ring path more than 10× the circumference. Scoring the folded rings made one chunk take 8.9 h.
- **Far-side mirror in full view:** a side plane meets the bottom face of the base plate.
- **Extended-plane phantoms** without the finite patch test.
- **A root gap larger than τ makes the joint invisible** (gap-conditioned coverage).
- Coplanar butt and grooved strata are outside the mechanism (grooved 0.00). Rounded rectangles and B-spline curves are not quadrics.
- **L1 region growing on thin plates merges faces through the thickness**, so L1 is 0.00.
- **The fixture counts as a part** (the largest fixture price; D13).
- On the "points" rung the accuracy is set by the threshold, not by the fit.

### 5.8 Code ↔ docs discrepancies

- The **seed is inert**. Stale statements that say otherwise: the harness registry comment ("random start is seeded"), MANUAL §8.1 ("no (seeded walk)"), notebook 11 cells 0 and 23, and dataset_plan ("seeded walk").
- The docstring's "random start kept as an arm" is reachable only through `order_points`.
- The docstring says the two output rungs emit "the same number of points **in the same order**". The order is computed separately on the raw and the projected points, so only the count is guaranteed (the test checks only lengths and statuses).
- **The default `ordering="distance"` in `detect` and the harness** means a bare `REGISTRY.run` call gets the published fold, not the headline chain.
- dataset_plan step 6: "13 tests" (now 21 functions, 23 cases with parametrisation); "`quadric_chain_*` ladder arms" (renamed); "projected onto the intersection" (now a rung).
- The "0,01 mm" and "0,93 pooled" figures in dataset_plan, MANUAL §9, the handoff, the email and notebook 11 cell 22 are **pre-audit projected numbers**.
- The ICRA table gives M7's oracle as "welding-surface labels"; it also includes part membership.

### 5.9 What the tests pin (`tests/test_lit_quadric.py`, 21 functions, 23 cases)

- The flat/curved verdict on pipe-on-plate.
- The literal σ share is < 0.90 on a noisy plane while the angle verdict says flat.
- The TLS normal is exact to 1e‑4.
- The cylinder quadric RMS is < 1e‑6.
- The three exact intersections on the projected rung (1e‑6, 1e‑4 and 1e‑3).
- The default "points" output is a subset of the cloud, within τ, and not snapped (two noise levels).
- The projected output is not cloud points and has RMSE < 0.05 mm.
- Both outputs have the same census and the same lengths.
- No returned point is farther than τ, for τ ∈ {1, 1.5, 2}.
- The per-axis and angle tests agree on plane, cylinder and saddle, and the per-axis deviation is ≤ the 3-D deviation.
- `flat_test` is recorded.
- All gate parameters are logged.
- Distance ordering folds a ring (> 10× its length) while chain stays < 1.02×; distance ordering is correct on a line.
- The finite patch kills the extended-plane seam.
- A real T: full-view recall > 0.8 and single-view precision ≥ full-view precision.
- Circle family in single view: n_curved ≥ 1; precision > 0.8; recall > 0.4 × visible fraction; 0.5 < RMSE < 3; projected < 0.1; measured > 5× projected; a tighter τ lowers the error.
- Seed invariance, and `start="random"` is seed-dependent.

### 5.10 Author correspondence (`notes/correspondence/lit-quadric_authors_email.md`)

The file is a **draft**: to Dongmin Li <ldm753@163.com>, signed Çağdaş Güven (METU MSc, Robotics). Its seven questions:
1. Is the flat rule ≥ 90 % within 10° of the mean normal, and is the mean the arithmetic mean of the sign-aligned normals?
2. Is the plane estimator TLS, since the printed equations have a trivial zero solution?
3. How was the quadric conditioned, and how was the trivial solution excluded?
4. What is the threshold value (the draft states ours as "≈ 2 mm", actually 2.25)? Was the intersection computed analytically or from nearby points? Was the blade always in contact with the hub, or is the threshold chosen relative to the gap?
5. Is the method open-seam-only, given that the distance ordering folds rings?
6. What were the region-growing criteria and thresholds, and was the cloud from a single camera pose?
7. What is the taught path's error budget, and are Fig. 6's errors means or maxima?

Status:
- handoff_phase8: "**sent** (2026-09-09…) — reply pending".
- README Phase 9 open item: "the reply from the lit-quadric authors (asked 2026-09-09)".
- phase8_plan: "all were contacted, one replied". The one reply is Zhang (lit-lobb).
- **No reply from Li et al. is recorded anywhere.**

The sent text describes **pre-audit readings**: the 3-D angle test, projection onto the intersection, and "~0.01 mm RMSE".

---

## 6. `ours` — radius-PCA (`scripts/baselines/radius_pca.py`)

### 6.1 Origin and status

- Migrated from the ROS node `admittance_control/scripts/icp_pose_refiner_node.py` (`surface_variation`, `_welding_points`, `_cross_object_mask`) and the voxel merge in `icp.py`. ROS defaults: `weld_radius_m` 0.006, `weld_curvature_thresh` 0.03, `weld_min_neighbors` 5, `weld_voxel_m` 0.004, `weld_require_cross_object` True.
- **The ROS node now defaults to `weld_method='registration'`** ("Mode A", since 2026-09-21); radius-PCA is only the fallback (`weld_fallback_pca`). So "the method the robot pipeline actually uses" (dataset_plan §4) is outdated.
- **Excluded from Phase 4 by ruling on 2026-09-03.** `run_phase4_batch.py:70–76` says it "brings no mechanism the lit methods lack and its coplanar-seam limit is already measured and reported … the project's own contribution is the improvement built on whichever method wins it."
- There is no paper; L0 is the `object_id` gate.

### 6.2 How it works, as implemented

1. **Radius choice** (`_run_ours`, `harness.py:195–204`, ρ = 0.5 pts/mm²): $lo=g+1/\sqrt\rho$ (1.414 mm), $hi=t_{min}/2$; $R=(lo+hi)/2$ if $lo<hi$, otherwise $R=lo$.
2. **Prefilter** by voxel at $1/\sqrt\rho$. Object ids follow by nearest neighbour. An `exterior` mask is wired in but has no role, since exteriority is now the input condition.
3. **Surface variation over a radius ball** (`:120–174`, batched; equal to the original loop to 1e‑12): $C=\frac1N\sum_{j\in B_R(i)}(p_j-\bar p)(p_j-\bar p)^\top$, $V=\lambda_3/(\lambda_1+\lambda_2+\lambda_3)$, with $\lambda_3$ the smallest. $V$ = 0 if there are fewer than 5 neighbours.
4. **Threshold** $V\ge0.03$.
5. **Cross-object gate** (oracle, `:196–220`): keep $i$ iff some $j\in B_R(i)$ has $id_j\ne id_i$.
6. `band_raw`, and its `density_anomaly` (normalised to the median, radius 1.5 × spacing). An optional `density_keep` filter.
7. **Voxel merge** at $2R/3$ (centroid; `"raw"` keeps every point).
8. **Clustering:**
   - `"components"` (default): link at $R$.
   - `"dbscan"`: eps = 2 × spacing, min_samples = 6, raw band.
   - `"directional"` (`:394–478`): a pair $(i,j)$ is kept iff it is within the link distance **and** $|e_{ij}\cdot t_i|\ge\cos30^\circ$ and $|e_{ij}\cdot t_j|\ge\cos30^\circ$, where $t$ is the top local PCA axis over a ball of $2R$. Then a lateral split: project perpendicular to each component's axis and run connected components.
9. Drop clusters with fewer than 8 points.
10. Return the **band**, scored as a point set (`evaluate_band`). **No line fit.**

### 6.3 Validity window

- Original claim: $g+s<R<t$.
- **Corrected 2026-08-20 to $R<t/2$**: what bridges a plate's two faces is the ball's diameter. `upper="thickness"` keeps the old bound reachable.
- Reference T-joint (g 1.1, s 2.7, t 8.4): the old claim gave "≈ 4–8 mm, default 6 mm". **Corrected, it is 3.8–4.2 mm** (pinned by a test), so the ROS default of 6 mm lies outside it.

### 6.4 Parameters (`detect`)

| Parameter | Default |
|---|---|
| `curvature_thresh` | 0.03 |
| `min_neighbors` | 5 |
| `voxel_mm` | None → 2R/3 |
| `object_id` | None |
| `cross_object` | True |
| `exterior` | None |
| `min_component_pts` | 8 |
| `link_mm` | None → R |
| `prefilter_density_per_mm2` | None (harness passes 0.5) |
| `density_radius_mm` | None → 1.5 × spacing_hint |
| `band_sampling` | "voxel" |
| `cluster_method` | "components" |
| `tangent_radius_mm` | None → 2R |
| `max_turn_deg` | 30 |
| `lateral_link_mm` | None (**ignored**, see §6.7) |
| `dbscan_eps_mm` | None → 2 × spacing_hint |
| `dbscan_min_samples` | 6 |
| `density_keep` | None |

spacing_hint is $1/\sqrt\rho$ when a prefilter density is set, otherwise R/4.

### 6.5 Recorded numbers

**Docstring, test, README and dataset_plan** (44 paired scenes; corpus not named):
- F1 0.709 → 0.897 (38/44 improved); precision 0.583 → 0.839 (39/44); band width 5.70 → 3.31 mm (44/44); Chamfer 5.05 → 3.68 mm (42/44).
- Window: "44 of 70 runs against 70 of 70".
- A "valid" ball under the old bound spans 1.2–1.9× the plate.
- Density anomaly near the seam (0–0.25R): corner 1.26, lap 1.16, butt 1.14, T 1.05, edge 0.62.
- The original 266×211×134 mm blob became a 232×16 mm line.

**Notebook 03 stored outputs** (2026-08-21, `out/bench`, ρ = 0.5):
- **Window open under both bounds in 107 of 181 scenes.** F1 0.705 → 0.871 (96/107); precision 0.581 → 0.813 (97/107); band width 6.095 → 4.214 mm (102/107); Chamfer 5.429 → 4.210 mm (98/107).
- Window closed in 286 of 500 runs.
- Medians, full / single: Chamfer 3.777/4.237; lat_p95 4.333/2.230; precision 0.788/1.000; recall 1.000/0.872; F1 0.843/0.824; band 3.860/2.201 mm.
- Per joint type (full / single): T F1 0.870/0.867; butt 0.966/0.666; corner 1.000/0.954; **edge 0.071/0.507 (full-view band 116.7 mm)**; lap 0.802/0.877.
- Directional vs proximity clustering (50 scenes): exact 0.10 vs 0.08; |count error| 4.38 vs 2.30; F1 0.644 for both.
- Declared vs all-interface precision: T 0.938/0.951, corner 0.994/0.994, butt 0.722/0.948, lap 0.535/0.649, edge 0.017/0.132; recall 1.0 throughout.
- Density profile at 0–0.25R: T 1.13, corner 1.20, butt 1.09, lap 1.04, edge 0.56.
- **Oracle ablation (30 scenes, single view): CAD oracle F1 0.789, Chamfer 3.20 mm; cloud-only F1 0.372, Chamfer 53.76 mm.**
- One-scene detail (re-run on bench6a): F1 0.965 at R = 1.86 mm with the window closed.
- Sweep (an out/bench scene): R within 90 % of the best F1 is 2.87 mm; corrected window 3.37–5.99 mm (old upper bound 11.98).

**dataset_plan:**
- Step 1, 2026-08-18 (44 scenes in out/phase3, old bound): gate on/off in single view, **Chamfer 4.2 → 56.7 mm, F1 0.75 → 0.03**; window empty in 16/88 runs; usable R 2.5–11.9 mm vs predicted 3.4–6.6 mm.
- Old-bound directional clustering: exact 2.3 % → 34.1 %. Corrected bound: 6.8 % → 15.9 %.
- Edge precision 0.01–0.05 at a 170 mm band.
- Notebook 12 (out of my scope): fixture precision 0.74–0.90 → 0.13–0.24; window closes in 143/250 scenes.

### 6.6 What the tests pin (`tests/test_baselines.py`, 16 of 20 functions)

- V on a fold is 0.05–0.20; V on flat faces is < 0.01.
- A radius larger than the gap is needed.
- The window values (3.8, 4.2) and closure on thin sheet.
- t/2 vs t, and the ball must not exceed the plate.
- Lengths are in millimetres: R = 0.006 finds nothing.
- The oracle flag is reported.
- Plate borders fire when the gate is off.
- Two components, not one blob.
- The band contains the seam.
- End-to-end runs and the single- vs full-view arms (need out/phase3).
- The batched variation equals the slow loop; chunking does not change the result.
- Directional clustering cuts the cross-run and splits parallel runs; it is off by default, and the `lit-*` modules never import it.

### 6.7 Discrepancies

- **PARAMETERS.md §5 still uses the uncorrected bound throughout** ("gap + point_spacing < R < part_thickness", "g + 1/√ρ < t", "usable window ≈ 4–8 mm", all three tables, $z_{max}=f_{px}(t-g)\cos\theta$). The file was last committed 2026-08-31. So do README line 503 ("`R < t`"), the admittance README §8, the ROS code comment, and the docstring of `test_the_validity_window_closes_on_thin_sheet` (which asserts t/2).
  - My own derivation, **not recorded in the repo**: under $R<t/2$ the window exists iff $g+s<t/2$. The ISO two-sided butt ($b\approx t/2$) is then never satisfiable, fillets need $t>4+2s$, and $z_{max}=f_{px}(t/2-g)\cos\theta$.
- **"F1 0,75 → 0,03"** (README, dataset_plan ×5, the lit_lobb docstring, notebook 03 markdown) is the 2026-08-18 old-bound measurement. Notebook 03's stored re-measurement shows **0.789 → 0.372**.
- **The 44-scene pairing numbers** are quoted in notebook 03's markdown above an output showing the 107/181-scene run.
- Notebook 03's header says `out/bench`; its code says `out/bench6a`; its outputs mix both.
- The module docstring's items 3–4 ("one line fit per component", "That fit is here") contradict the code (band only). dataset_plan's "radius-PCA + nearest-point midpoint" is likewise stale.
- **`detect(lateral_link_mm=…)` is never forwarded** to `directional_components` (`:675`), and neither is `min_neighbors`. Notebook 03's directional arm asked for 0.6·t_min and actually ran with link = R.
- **`--include-ours` does not exist** in argparse, although the batch comment, dataset_plan §10 and the handoff refer to it.
- **The ICRA paper never mentions radius-PCA**, although dataset_plan §10 says its coplanar limit "stands in the paper".

---

## 7. BibTeX entries

**The keys `wang2026pipes`, `fang2024modelreg` and `li2026quadric` do not exist in `paper/icra2027/refs.bib`.** The ICRA file uses different keys:

```
@article{wang2026pipe,
  author={Wang, Tianqi and Liang, Yifan and Liu, Xinqi and Wang, Zhigang},
  title={{3D} vision-based intersecting pipe welding path planning},
  journal={Welding in the World}, year={2026}, note={doi:10.1007/s40194-026-02509-9}}
@article{fang2024model,
  author={Fang, Weihua and Tian, Xincheng},
  title={A novel model-based welding trajectory planning method for identical structural workpieces},
  journal={Robotics and Computer-Integrated Manufacturing}, volume={89}, pages={102772}, year={2024}}
@article{li2026impeller,
  author={Li, Dongmin and Wang, Yu and Wang, Zhengyong},
  title={Automatic recognition on impeller shape and weld seam based on normal of point clouds and {PCA}},
  journal={Journal of Mechanical Science and Technology}, volume={40}, number={3}, pages={2039--2046}, year={2026}}
```

All three ICRA entries match the implemented PDFs. They lack DOIs, except that the pcaslice entry carries its DOI in `note`.

The thesis file is `ros2_ws/src/admittance_control/thesis/manuscript/thesis.bib`, lines 58–104. The keys are cited at `chapters/ch4_methodology_weldset.tex:146`.

```
@article{wang2026pipes,
  author  = {Wang, Tianqi and Liang, Yifan and Liu, Xinqi and Wang, Zhigang},
  title   = {{3D} vision-based intersecting pipe welding path planning},
  journal = {Welding in the World},
  year    = {2026},
  doi     = {10.1007/s40194-026-02509-9}
}
```
**Correct.** It is the PCA-slicing paper by Wang et al., *Welding in the World* 2026: all four authors in order, exact title, correct DOI. No volume or pages exist on the PDF.

```
% VERIFY: the full author list.
@article{fang2024modelreg,
  author  = {Fang, Weihua and Tian, Xincheng and others},
  title   = {A novel model-based welding trajectory planning method for identical structural workpieces},
  journal = {Robotics and Computer-Integrated Manufacturing},
  volume  = {89},
  pages   = {102772},
  year    = {2024}
}
```
**Wrong author list: there are exactly two authors, so remove "and others".** Add `doi = {10.1016/j.rcim.2024.102772}`. Everything else matches.

```
% VERIFY: the full title (cut on the PDF's first page) and the end page.
@article{li2026quadric,
  author  = {Li, Dongmin and Wang, Y. and Wang, Z.},
  title   = {Automatic recognition on impeller shape and weld seam based on normals of point clouds},
  journal = {Journal of Mechanical Science and Technology},
  volume  = {40},
  number  = {3},
  pages   = {2039},
  year    = {2026},
  doi     = {10.1007/s12206-026-0243-8}
}
```
**Wrong title.** It must read "…based on **normal** of point clouds **and PCA**". The co-author names should be "Wang, Yu" and "Wang, Zhengyong", and the pages "2039--2046". The full title and the page range are printed on page 1 in the citation box. The DOI, volume and number are correct.

Outside my three entries, in the same citation line: `zhang_lobb` still has `year = {VERIFY}` (the PDF gives RCIM 95 (2025) 102987), and `wei_regiongrow` has "VERIFY venue" (arXiv:2408.10710, 2024).

---

## 8. Other discrepancies that could leak into the thesis

- **"Every one of these pipelines relies on a learned segmentation or detection stage"** (ICRA main.tex:51) is false for Fang & Tian (a CAD model plus classical edge detection) and for Li et al. (region growing).
- MANUAL §8.1 says "notebooks 04…10" (quadric is 11, `ours` is 03). MANUAL §9 ("as of 2026-09-08") is stale: pooled 0.93 at 0.01 mm, "deterministic five", "five of six".
- dataset_plan §4's "The seven" table lists `ours` and omits lit-quadric. The "first numbers" for pcaslice (F1 0.95–1.0) and modelreg (T 1.00, corner 1.00, butt 0.98, lap 0.99) do not match notebooks 09 and 10. The "Measured" paragraph of the Step 6 batch mixes the post-audit row count with pre-audit numbers.
- handoff_phase8 §3's table is labelled "notebook 15" but holds pre-visfix, pre-audit values.
- nb15 cell 0 still says modelreg is "absent (NaN) on curved families" and "the other five are deterministic" (there are six).
- The harness module docstring's oracle table, the `__init__` docstring and the `run_matrix` default method list omit lit-quadric; the default list also still includes `ours`.
- **Notebooks 09 and 10 ran on view "full"** (the whole CAD cloud, including buried faces), which is now "truth only, never a method input".
- **Task-2 selection of 1.00** for pcaslice and modelreg is an oracle artefact. nb15 says so.
