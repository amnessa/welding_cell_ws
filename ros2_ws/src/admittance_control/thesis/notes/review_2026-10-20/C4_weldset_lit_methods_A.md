# WeldSet material map: four literature baselines (lit-ransac, lit-regiongrow, lit-lobb, lit-ppf)

## 0. Provenance, reading rules, and the biggest thesis risks

**Process note.** I worked read-only throughout, with one slip: I sent notebook-15 output into a scratch file at `/tmp/claude-0/-workspaces-welding-cell-ws/ab70b953-a34b-4bce-b04c-436c69dd0e01/scratchpad/nb15_out.txt`. It is in the session scratchpad, outside the repo. I did not delete it, because deleting was also prohibited. No repository file was created or changed. Git was used read-only only: `log`, `show`, commit-to-commit `diff`, and `diff HEAD` with `--no-optional-locks`.

**Provenance**
- **Code state.** The working tree equals HEAD for every in-scope file.
  - All four modules were last changed in commit `a98c19d` (2026-09-14, the fidelity audit).
  - Pre-audit state: `c670770`.
- **All four method notebooks predate the audit.** Every stored number in them is pre-audit:

  | notebook | last commit |
  |---|---|
  | 04 | 2026-09-03 |
  | 05 | 2026-08-27 |
  | 06 | 2026-09-08 |
  | 08 | 2026-09-08 |

- **Where the current (post-audit) numbers live:**
  - Notebook 15 stored outputs ("nb15"): 271,200 rows, 760 scenes (720 + 40 fixture twins), 67 chunks, `out/bench_phase4`.
  - `paper/icra2027/tables/numbers.tex` ("macro"; `\nRows`=271,200).
  - The README says the same: "the current numbers are notebook 15's" (README.MD:544-545).
- **Where the audit is recorded:**
  - `notes/phase8_plan.md` §1.2.4 (lines 470-541).
  - nb15 addendum (`notebooks/build_15_phase4_results.py`:127-143).
  - `dataset_plan.md`:2149-2165.

**Metrics behind every "paper's own metric" claim** (`scripts/baselines/metrics.py`)
- `path_error_mm` (218-245) is **one-directional**. The prediction is resampled every 0.25 mm and its distance to the truth polyline is measured:
  - RMSE $=\sqrt{\overline{d^2}}$
  - ME $=\max d$
  - It does **not** penalise missing coverage, so a short accurate stub scores well.
- `match_seams` (248-271) is a greedy one-to-one match.
- F1 (`precision_recall_f1`, 113-130):
  - precision = fraction of the prediction, densified at 1 mm, lying within 3 mm of any truth seam;
  - recall = fraction of truth samples, at 1 mm, within 3 mm of any predicted point.
- **None of the papers' metrics is identical to this:**
  - Yi: max ME/RMSE against a taught path.
  - Wei: RMSE at 3 points per linear seam and 10 on the curved one.
  - Zhang (RCIM): per-axis (x, y, z) ME/RMSE over 80 interpolated points **after aligning endpoints**, on 2 seams (Table 2).
  - Wang: corner-distance (length) error.

**Top risks for the thesis** (details in each section)
1. Notebook and README/dataset_plan prose numbers are mostly pre-audit. Several notebook markdown numbers are not reproduced by the stored outputs of the same notebook.
2. "lit-ransac reproduces its paper" compares a **median** (0.60/0.81 mm) with the paper's **maximum** (0.64/1.16 mm). The maximum over the same rows is 3.023/4.150 mm. "Exactly zero" error without refit is the median; the mean is 0.394 and the max 3.179 mm.
3. "lit-lobb reproduces its paper on corners" has three problems:
   - Pre-audit it was RMSE-only; the ME was not met (median ME 3.362 mm; the showcase scene's ME was 4.315 mm).
   - Post-audit the Phase-4 corner median F1 is **0.00**.
   - Only one `out/bench` test scene meets both criteria (0.112/0.416 mm).
4. lit-regiongrow's L0 justification ("a trained weld-keypoint detector clicks each seam") is not in Wei et al. The paper clicks the **centre of each workpiece surface**; keypoint detection only automates those clicks. Only the 12.7 % retained fraction is the paper's. The ICRA text repeats the claim.
5. The author-requested `mean_shift=True` arm and the B-spline extension were **never run or implemented**, although the README says the arm "gets reported".
6. Thesis bibliography problems:
   - `zhang_lobb` and `wei_regiongrow` are incomplete.
   - `fang2024modelreg` lists the wrong authors.
   - `li2026quadric` has the wrong title and end page.
   - The T-ASE LOBB descriptor paper is missing (§5).

---

## 1. lit-ransac (Yi et al. 2026)

### 1.1 Paper
- **Citation (verified on PDF p.1):** Jinxin Yi, Xuan Kong (corresponding), Hao Tang, Jie Zhang, Zhenming Chen, Lu Deng.
  - Title: "Weld seam extraction and path generation for robotic welding of steel structures based on 3D vision."
  - *Automation in Construction*, vol. 183 (2026), article 106792. doi:10.1016/j.autcon.2026.106792.
  - Received 27 Aug 2025; revised 12 Jan 2026; accepted 16 Jan 2026; online 27 Jan 2026. 19 pp.
- **Mechanism (one sentence):** an improved multi-plane RANSAC (eq. 17 occupancy termination, eq. 18 confidence-driven iterations, eq. 19 centroid-neighbourhood refit) feeds pairwise intersection lines of orthogonal plates as fillet seams. Endpoints come from projecting the "vertical point cloud" onto the line, anchored at triple-plane corners (eqs. 25-28). The torch pose comes from the in-plane bisector (eqs. 23-24).
- **Their coarse stage:**
  - §3 PointNet++ weld-region segmentation: band "approximately 40 mm" wide, accuracy 98.16 %, mIoU 95.07 %.
  - §4 multi-scale ICP fusion of several coded-structured-light views; eq. 9 statistical filtering sits in §4.2.
- **L0 in WeldSet:** `seam_region_oracle`, all points within 20 mm of the truth seams with seam ends clipped (`end_margin_mm=0`) (lit_ransac.py:152-218; harness.py:139-141).
  - The view `full_exterior` stands in for §4 (a perfect multi-view scan).
  - The harness voxelises at ρ = 1 pt/mm², i.e. a 1 mm voxel (harness.py:211).
- **L1:** the whole cloud, no mask (harness.py:210).

### 1.2 Pipeline as implemented (`scripts/baselines/lit_ransac.py`)
1. **Mask** (870-872), then voxel centroid downsample with $v=1/\sqrt{\rho}$ (874-877; `radius_pca.voxel_downsample`, a centroid per voxel).
2. **Optional eq. 9 filter** (883-885, 352-373), off by default.
   - $\bar\delta_i=\frac1k\sum_{j\in\mathcal N_k(i)}\lVert\mathbf p_i-\mathbf p_j\rVert$; keep iff $\bar\delta_i\le\mu(\bar\delta)+\alpha\,\sigma(\bar\delta)$, with k = 20 and α = 1.
   - The printed form is $\lVert\mathbf p_k-\mathbf p_j\rVert/\sigma_j>\tau$, with τ unprinted.
3. **Point spacing** = mean nearest-neighbour distance, using 20 000 queries and seed 0 (316-349, 891). If `dist_thresh_rule="3x_spacing"`: $T_{d2}=3\bar s$ (892-896), which is §5.2's rule; the 2 mm of the paper is its *answer*.
4. **Support radius** $r_s=3T_{d2}$ = 6 mm (898-899).
5. **Multi-plane fit** (529-560), repeated on the shrinking remainder.
   - **RANSAC** (418-454): $\mathbf n=\frac{(\mathbf p_2-\mathbf p_1)\times(\mathbf p_3-\mathbf p_1)}{\lVert\cdot\rVert}$, $d=-\mathbf n^\top\mathbf p_1$ (eqs. 13-14). An inlier satisfies $|\mathbf n^\top\mathbf p+d|<T_{d2}$ (eq. 16, line 448).
   - **Iteration budget, eq. 18.**
     - Printed form: $(1-(3/N_{fp})^3)^K<1-\eta_0$.
     - Default reading `iteration_rule="adaptive"`: $K=\left\lceil\frac{\ln(1-\eta_0)}{\ln(1-w^3)}\right\rceil$ with $w=n_{in}/N_{fp}$, recomputed whenever the best hypothesis improves (396-415, 451-453).
     - K is clipped to [1, `max_iterations`=1000] and the loop runs at least 10 times.
     - `"literal"` uses $w=3/N_{fp}$, which gives K ≈ 4e10 at N = 10 000. The adaptive reading gives K = 44 at w = 0.3.
   - **Eq. 17 stop:** break if $n_{in}/N_{seg}\le T_{mpp}$, where $N_{seg}$ is the **original** cloud size (line 552).
   - **Eq. 19 refit** (478-526), using the k = 200 inliers nearest the centroid $\mathbf p_o$.
     - Default `centroid_lstsq`: PCA normal of that neighbourhood (512-518).
     - `centroid_3pt` (the literal eq. 19): $\mathbf n=(\mathbf p_{j_1}-\mathbf p_o)\times(\mathbf p_{j_2}-\mathbf p_o)$ with two random neighbours (502-511).
     - `none`: no refit.
     - The original RANSAC inlier set, not the refit set, is removed from the remainder (559).
6. **Triple points, eq. 25** (624-648, 930-937): solve $[\mathbf n_i;\mathbf n_j;\mathbf n_k]\mathbf p=-[d_i,d_j,d_k]^\top$ with |det| > 1e-9.
   - Default validity: inside the cloud bounding box ± 50 mm.
   - Literal §6.2 test: $|p_c|\in[10^{-3},10^3]$ via `coord_bounds_mm`.
7. **Every plane pair** (945-998):
   - Fold $\phi=\arccos|\mathbf n_1^\top\mathbf n_2|\in[0°,90°]$ (948-949).
   - **Eq. 21:** $\mathbf d=\mathbf n_1\times\mathbf n_2/\lVert\cdot\rVert$; "parallel" if $\lVert\mathbf n_1\times\mathbf n_2\rVert<10^{-6}$ (574-577).
   - **Eq. 22:** solve $\begin{bmatrix}A_1&B_1\\A_2&B_2\end{bmatrix}\begin{bmatrix}x\\y\end{bmatrix}=\begin{bmatrix}-D_1\\-D_2\end{bmatrix}$ at z = 0. The code tries dropping z, then y, then x, keeping the first with |det| > 1e-9 (582-592).
   - **Gate:**
     - `"orthogonal"` (default): $|\phi-90°|\le30°$.
     - `"intersecting"`: $\phi$ or $180°-\phi$ lies in [20°, 160°], which reduces to $\phi\ge20°$ (957-963, 722-731).
   - **Support** (an invented check): at least 10 inliers of *each* plane within $r_s$ of the line (966-975).
   - **Corner:** the first triple point within $r_s$ of the line (977-981).
   - **Endpoints, eqs. 26-28** (661-715): $t=(\mathbf p-\mathbf p_{line})\cdot\mathbf d$, computed over inliers within $r_s$.
     - `smaller_plane` (default) uses the plane with the shorter projected span, the reading of the "vertical point cloud".
     - With a corner inside that span: endpoints are the corner and the farthest projection. Otherwise: the extreme projections.
   - Seams shorter than 5 mm are dropped (989-991).
   - **Pose, eq. 23:** $\mathbf v_0=\widehat{\mathbf v_1+\mathbf v_2}$, where $\mathbf v_i$ is the centroid direction perpendicular to $\mathbf d$ (605-613).
   - **Pose, eq. 24:** $X_t=\mathbf d$, $Z_t=-\mathbf v_0$, $Y_t=X_t\times Z_t$. As printed det = −1; `right_handed=True` negates $Y_t$ (615-621).
   - The pose is computed but **not scored**.
8. **Pair census** in `pairs`: statuses seam / parallel / not_orthogonal / not_intersecting / no_support / no_endpoints / too_short. The no-seam `note` (1000-1012) is built from it.

**Readings, each behind a switch (docstring deviations 1-9, lines 40-116):**
1. Eq. 18 as adaptive RANSAC.
2. Eq. 19 estimator replaced by PCA.
3. Eq. 22 elimination by conditioning.
4. Frame-independent §6.2 bound.
5. Pair support (invented).
6. Eq. 24 handedness fix.
7. 30° orthogonality tolerance (invented), with the generous `intersecting` rung.
8. Fixed 2 mm threshold against the 3× spacing rule.
9. Eq. 9 filter off by default.

### 1.3 Parameters (`detect` 794-821 plus helpers)

| name = default | source |
|---|---|
| `dist_thresh_mm` = 2.0 mm (`T_D2_MM`) | paper §5.2 (Tables 3-4) |
| `dist_thresh_rule` = "fixed" | [ours]; "3x_spacing" = paper's rule |
| `min_inlier_ratio` = 0.025 (`T_MPP`) | paper §5.3, eq. 17 |
| `confidence` = 0.7 (`ETA_0`) | paper §5.3, eq. 18 |
| `SEAM_REGION_HALF_WIDTH_MM` = 20 mm | paper's ~40 mm width |
| `end_margin_mm` = 0 | [ours] |
| `seed` = 0 | harness supplies 0-29 |
| `voxel_mm` = None, `prefilter_density_per_mm2` = None | harness 1.0 pts/mm², i.e. 1 mm voxel [ours] |
| `outlier_filter` = False, `outlier_k` = 20, `outlier_std_ratio` = 1.0 | [ours, PCL-style]; paper prints neither k nor τ |
| `max_planes` = 20 | [ours] |
| `max_iterations` = 1000; `min_iterations` = 10 (in `ransac_plane`) | [ours] |
| `iteration_rule` = "adaptive" | [ours reading]; "literal" = eq. 18 |
| `plane_refit` = "centroid_lstsq" | [ours]; "centroid_3pt" = eq. 19; "none" |
| `refit_k` = 200 | [ours] |
| `pair_rule` = "orthogonal" | paper premise |
| `orthogonal_tol_deg` = 30° | **[ours]** |
| `intersect_range_deg` = (20°, 160°) | **[ours]** |
| `parallel_tol` = 1e-6 | [ours] |
| `support_radius_mm` = None (3·T_d2 = 6 mm); `min_pair_support` = 10 | [ours] |
| `endpoint_source` = "smaller_plane" (also "both", "overlap") | reading |
| `min_seam_length_mm` = 5.0 | [ours] |
| `max_extrapolation_mm` = 50.0; `coord_bounds_mm` = None | [ours]; (1e-3, 1e3) = paper |
| `right_handed` = True | [ours]; False = eq. 24 as printed |
| spacing estimator: 20 000 samples, seed 0 | [ours] |

### 1.4 Fidelity audit (2026-09-14)
- **Verdict:** "honest, two invented parameters on the corpus edge. Defaults unchanged (results stand)" (phase8_plan.md:520-524). The ransac headline chunks were not recomputed (phase8_plan.md:529).
- **Rungs added** (`run_phase4_batch.py`:155-159), each with 30 seeds:

  | rung | kwargs |
  |---|---|
  | `ransac_intersecting_full_exterior` | `pair_rule="intersecting"`, now windowed (20°, 160°) |
  | `ransac_thresh3x_full_exterior` | `dist_thresh_rule="3x_spacing"` |
  | `ransac_outlierfilter_full_exterior` | `outlier_filter=True` |

- **Before the audit**, "intersecting" accepted *any* non-parallel pair (git diff of `c670770`→`a98c19d`). The audit added the window, the 3× spacing rule, eq. 9, and spacing logging.
- **Numbers.**
  - Smoke test (first scene per stratum): "Corner 0,00 → 0,27 under the generous gate; grooved butt 0 under all."
  - Batch macros, median F1 on full_exterior:

    | condition | T/line | corner | grooved |
    |---|---|---|---|
    | headline | 0.98 | 0.50 | 0.00 |
    | intersecting | 0.98 | 0.50 | 0.06 |
    | thresh3x | 0.82 | 0.50 | 0.00 |
    | outlierfilter | 1.00 | 0.50 | 0.00 |

  - So at corpus level the generous gate does **not** move corners (0.50 → 0.50).

### 1.5 Arms and randomness
- **Registry:** `randomised=True`, oracle "band" (harness.py:366-367).
- **Seeds:** 30 in every batch chunk where it appears (`--seeds` default 30). Notebook 04 used 30, 8 and 5 seeds in different sections.
- **Chunks:**
  - `coverage_lit-ransac_{full_exterior,single}`, `l1_lit-ransac_full_exterior`
  - `noise1_lit-ransac_single`, `noise2_lit-ransac_single`
  - `task2_lit-ransac_single`, `fixture_lit-ransac`
  - the three rungs above
- **kwargs available but never batched:** `iteration_rule="literal"`, `plane_refit="centroid_3pt"/"none"`, `coord_bounds_mm=(1e-3,1e3)`, `right_handed=False`, `endpoint_source="both"/"overlap"`.

### 1.6 Recorded numbers

**(a) Notebook 04, stored outputs (pre-audit).** Setup: `out/phase3`, 44 scenes, view "full" (CAD cloud), ρ = 1, band oracle.

- **Scene 8** (corner, quality D, gap 0.95 mm, t_min 7.67 mm):
  - **L0:** 4 planes from 12 454 points; 3 predicted vs 2 true.
    - Chamfer 6.256; pred→gt 5.432; gt→pred 0.824; P 0.335; R 1.000; F1 0.502; lat_p95 7.957.
    - Plane ratios 0.315 / 0.220 / 0.290 / 0.174; iterations 38 / 36 / 10 / 10.
    - Pairs: (0,1) 75.47° seam; (0,2) 76.52° seam; (0,3) 3.02° not_orthogonal; (1,2) 1.06° not_orthogonal; (1,3) 72.46° no_support; (2,3) 73.51° seam.
  - **L1:** 4 planes, 2 predicted; P 0.344; R 1.0; F1 0.512; Chamfer 13.942.
- **Validation:** 12 T scenes, 1650 matched rows; ρ = 1/0.6427²; 5 seeds; window t ≥ 4 mm open in 8 of 12 scenes. Nominal curve, window open:

  | condition | refit | RMSE med / mean / max (mm) | ME med / mean / max (mm) |
  |---|---|---|---|
  | noiseless | 3pt | 2.671 / 2.972 / 13.658 | 5.037 / 5.375 / 22.184 |
  | noiseless | lstsq | 0.599 / 0.562 / 3.023 | 0.808 / 0.997 / 4.150 |
  | noiseless | none | 0.000 / 0.394 / 3.179 | 0.000 / 0.568 / 5.218 |
  | noisy | 3pt | 3.919 / 4.963 / 22.171 | 6.862 / 8.087 / 34.148 |
  | noisy | lstsq | 0.769 / 1.274 / 11.567 | 1.189 / 1.976 / 14.469 |
  | noisy | none | 1.222 / 1.505 / 7.341 | 1.788 / 2.173 / 9.458 |

  - D19 curve choice (median RMSE / ME): gapmid 1.078 / 1.368; nominal 0.599 / 0.808; root 0.735 / 1.341. The spread is 0.479 mm against a 0.64 mm target.
- **Corpus, seed 0** (88 runs, 439 pairs), mean F1 L0 → L1:

  | joint | F1 L0 → L1 | L0 precision / recall | n |
  |---|---|---|---|
  | T | 0.84 → 0.58 | 0.86 / 0.87 | 12 |
  | butt | 0.67 → 0.50 | | 15 |
  | corner | 0.55 → 0.45 | | 11 |
  | edge | 0 → 0 | | 4 |
  | lap | 0.30 → 0.00 | | 2 |

  - Mean Chamfer L0 / L1: T 5.38 / 11.95; butt 1.32 / 16.71; corner 5.30 / 26.47; lap 8.90 / NaN.
  - Fold angles (count / min / median / max): T 163 / 0.0 / 73.8 / 89.7; butt 99 / 0.0 / 10.4 / 89.7; corner 132 / 0.0 / 68.8 / 90.0; edge 11 / 0.2 / 5.3 / **25.7**; lap 34 / 0.2 / 6.6 / 90.0.
- **Lap sweep** (2 scenes × ρ ∈ {0.25, 0.5, 1, 2} × 8 seeds):
  - Spread of F1 across seeds 0.299; across ρ 0.270.
  - L0: mean F1 0.314; nothing returned in **50 %** of runs; fold > 60° in 32/64 runs.
  - L1: mean F1 0.069; nothing in 89 %; fold > 60° in 8/64.
- **Reproducibility** (3 scenes per type × 30 seeds, 420 runs), F1:

  | joint | min / median / max | std | zero runs | plane counts seen |
  |---|---|---|---|---|
  | T | 0.25 / 1.00 / 1.00 | 0.15 | 0/90 | {4, 5, 6} |
  | butt | 0 / 0 / 1.00 | 0.47 | 60/90 | {2, 3} |
  | corner | 0.31 / 0.57 / 1.00 | 0.23 | 0/90 | {2..7} |
  | edge | 0 / 0 / 0 | 0 | 90/90 | {1, 2, 3} |
  | lap | 0 / 0 / 0.67 | 0.31 | 39/60 | {4, 5, 6} |

- **Eq. 19 as printed** (300 draws on a poisoned plane): tilt median 0.556°, p95 4.936°, max 69.729°.

**(b) Docstrings and tests**
- §5.2 window: median RMSE 0.57 mm above 4 mm plate against 1.51 mm below, Spearman −0.78 (12 T scenes). Stated in a test docstring, not asserted.
- Capsule oracle artefact: seam 38 mm too long, line accurate to 0.6 mm (docstring) / 0.97 mm (dataset_plan). The first run gave RMSE 3.3 mm and ME 13.7 mm.
- Thick-plate L0 vs L1: 12 seams vs 2, precision 0.26 → 8 seams, 0.77.

**(c) README and dataset_plan**
- Reproduction: 0.60 / 0.81 vs 0.64 / 1.16; noisy 0.77 / 1.19; no refit 0.00 / 0.00; eq. 19 as printed 2.67 / 5.04 (8 scenes × 5 seeds).
- First per-type table (2026-08-20), L0 → L1 F1: T 0.75 → 0.58; lap 0.55 → 0.00; butt 0.54 → 0.50; corner 0.51 → 0.45; edge 0 → 0.
- Butt seed sweep on three scenes:

  | scene | gap | F1 min / median / max | zero runs |
  |---|---|---|---|
  | …412392 | 1.82 mm | 0 / 0.94 / 0.96 | 2/30 |
  | …412402 | 0.24 mm | 0 / 0 / 0.96 | 20/30 |
  | …412339 | 1.95 mm | 0 / 0 / 0 | 30/30 |

- Phase-6b run (pre-audit): 0.97 median F1 on straight T; 0.00 on every curved family.

**(d) Current numbers (nb15 and macros, bench_phase4)**
- Full_exterior median F1: T/line 0.98; circle, ellipse, saddle and rounded_rect 0.00; swept 0.77; corner 0.50; butt 0; grooved 0; arc 0; lap 0; edge 0.
- Single view: T/line 0.67; ellipse 0.10; saddle 0.16; rounded_rect 0.38; swept 0.60; **corner 0.93**; others 0.
- T-joint 0.99 vs angle joint 0.93; T/line RMSE 0.75 mm.
- L0 → L1 by joint type: T 0.00 → 0.25; butt 0 → 0; corner 0.50 → 0.42; edge 0 → 0; lap 0 → 0. Pooled 0.00 → 0.10 (ladder.tex).
- Noise (σ × 0 / 1 / 2): 0.20 / 0.00 / 0.00.
- Seed spread: on 5 % of full-scan scenes F1 ranges from 0 to above 0.9 (`\ransacBimodalShare`). Share of scenes with spread > 0.25: T 0.3, butt 0.43, corner 0.82, edge 0, lap 0.45.
- Fixture: ΔF1 0.00 overall (T −0.09, corner −0.17), extra seams 0.
- Task 2:
  - Selection: T 0.81, corner 0.61, butt 0.43, lap 0.36, edge 0.08.
  - Localisation RMSE (mm): 0.8, 0.0, 2.8, 0.9, 2.0.
- Cost: 0.02 s (L0 full), 0.01 s (single), 0.09 s (L1).

### 1.7 Measured limitations
- No seam is possible for coplanar or parallel faces.
  - Edge joints are rejected at fold ≈ 0-26° by the orthogonality gate, not by the parallel test.
  - Butt joints succeed only through root-gap walls. This depends on the corpus: 0.67 at seed 0 on phase3, but a 0.00 median on bench_phase4.
- T_mpp deletes the lap-toe ribbon (< 2 % of the area) in 89 % of L1 runs.
- Plate-border phantoms appear at L1; even at L0 the 40 mm band admits plate end faces.
- Strongly seed-dependent.
- The §5.2 window ($T_{d2}$ < plate thickness) closes on thin plate.
- The 30° gate sits exactly on the corpus's 60°/120° extremes (|fold − 90| = 30 within 1e-6).
- Curved families collapse to 0.
- Goes to zero at the first noise step.

### 1.8 Discrepancies (lit-ransac)
1. Module docstring line 16 and nb04 cell 0 name a §6 function `seams_from_planes`, which **does not exist**. §6 lives in `detect` (928-998) and its helpers.
2. nb04 cell 0 says "all six deviations"; the docstring now lists nine.
3. "Widest edge fold 15,8°" (nb04 markdown, dataset_plan:254) vs the stored output's **25.7°**.
4. "Over 90 seeded edge runs, 89 return nothing and one scores F1 0,23" (nb04, dataset_plan:256-257) vs stored output **90/90 zero**.
5. Lap table, "runs returning nothing L0 = 45 %" (nb04 markdown, README:640-641, dataset_plan:284) vs stored output **50 %**.
   - "Seeds 0,27 / ρ 0,31" vs stored output 0.299 / 0.270 (different, and the ordering is reversed).
   - "Fitted the ribbon: most" vs 32/64.
6. The dataset_plan 2026-08-20 per-type table (T 0.75, lap 0.55, butt 0.54, corner 0.51) does not match nb04's re-executed table (0.84 / 0.30 / 0.67 / 0.55) for the same corpus, seed and ρ.
7. The dataset_plan butt seed table totals 52/90 zero runs with max 0.96. nb04's stored output for three butt scenes shows **60/90** and max **1.00**.
8. "F1 0,00-0,94 on one fixed scene" (harness.py:6-7, nb04, nb06, test_lit_lobb docstring) quotes the **median** as the top of the range. dataset_plan/README give min 0.00, median 0.94, max 0.96, and dataset_plan:1467 says "0,00-0,96".
9. The reproduction claim compares a median with the paper's max (§0, risk 2). "Exactly zero" holds for the median only (mean 0.394 mm, max 3.179 mm). The test asserts only RMSE < 1.5 and ME < 3.0 mm on one scene.
10. Deviation 9 cites "(§1)" for "the segmented point cloud undergoes outlier removal". That sentence is in **§3**, and eq. 9 is in **§4.2** (fine registration).
11. MANUAL §8.3: "RANSAC over a full cloud … is 20× its oracle-assisted cost". nb15 gives 0.02 s → 0.09 s (≈4.5×); the ≈20× ratio is lit-lobb's (0.50 → 10.42 s).
12. MANUAL §9 ("as of 2026-09-08") and dataset_plan:1901 say 0.97 on straight T; the current figure is 0.98.
13. dataset_plan says 15 tests; there are 25 now.
14. dataset_plan:1438 still groups lit-ppf as randomised; this was corrected elsewhere.

### 1.9 Tests (`tests/test_lit_ransac.py`, 25) and what each pins
- **Constants:** T_d2 = 2.0, T_mpp = 0.025, η0 = 0.7, in mm. A 90° fold fits 2 planes at 2 mm and 1 plane at 2000 mm.
- **Eq. 18:** literal > 1e9 iterations; 20 < adaptive < 100 at w = 0.3, N = 10 000.
- **Eq. 17:** the ratio is taken against the original cloud. A 1.7 % face yields 1 plane at T_mpp and ≥ 2 planes at 0.005.
- **Eqs. 13-14:** unit normal; collinear points give None.
- **§5.4 refit:** removes a poisoned tilt (refit < 0.1° < raw).
- **Eqs. 20-22:** the 90° fold line is exact to 1e-6.
- **Parallel faces:** None.
- **Synthetic edge joint:** 0 seams; every pair parallel or not_orthogonal; max fold < 30°; note says not_orthogonal.
- **`smaller_plane` endpoints:** 100 ± 2 mm; `both` gives 200 ± 2 mm.
- **Eq. 24:** det −1 as printed, +1 fixed.
- **Eq. 23:** bisector ≈ (0, 1/√2, 1/√2).
- **§6.2 literal bound:** rejects a corner at the origin.
- **Capsule:** with a 20 mm end margin the length error is > 20 mm; clipped it is < 5 mm; lateral change < 0.5 mm.
- **Reproduction:** RMSE < 1.5 mm and ME < 3.0 mm on the first phase3 T scene with t ≥ 4 mm, at ρ_paper.
- **§5.2:** T_d2 = 2 mm beats 12 mm on a scene with t ≥ 6 mm.
- **Band oracle:** fewer seams, precision +0.2 at equal recall 1.0 (20 mm plates).
- **Seeds:** the same seed gives identical seams.
- **End to end:** recall > 0.5 on a phase3 T or corner scene.
- **30° gate:** 60° vee rejected at a knife edge; 59° rejected; `intersecting` accepts one seam along x.
- **`intersecting`:** refuses a 5° vee; a wide (1, 179) window lets a pair through; a bad `pair_rule` raises.
- **3× spacing:** 1 mm grid gives T_d2 = 3.0 and support radius 9.0.
- **Eq. 9 filter:** 40 flyers removed, > 90 % of the plate kept, params logged.
- **Literal switches reachable**, with their defaults asserted.
- **Added gates logged** in params.
- **New switches leave the defaults bit-identical.**

---

## 2. lit-regiongrow (Wei et al. 2024)

### 2.1 Paper
- **Citation (PDF p.1):** Pengkun Wei, Shuo Cheng, Dayou Li, Ran Song, Yipeng Zhang, Wei Zhang (corresponding).
  - Title: "Coarse-to-Fine Detection of Multiple Seams for Robotic Welding."
  - **arXiv:2408.10710v1 [cs.CV], 20 Aug 2024**, 7 pp. No DOI or journal printed. Shandong University; Y. Zhang at UCLA.
- **Mechanism:** region growing seeded at the smoothest point by $\delta=\lambda_0/\sum\lambda$. Neighbours split into surface or edge by the normal-angle jump. Seam points are edge points that have high curvature *and* lie between two surfaces. They are fitted by a plane fit, projection, and a 2D (x, y) fit; the pose is the mean normal.
- **Their coarse stage (§III-C):** FastSAM masks one surface per click.
  - "The prompts are obtained by clicking the center of each workpiece surface simply."
  - A keypoint detector is trained to generate those click prompts automatically: "only 4 key points need to be labeled for an open square weldment".
  - "The intersection area between the surfaces indicates the approximate location of the weld seams."
  - Table II: 81 730 → 10 356 points (12.67 % kept); "remove over 80% of redundant point clouds", saving about 68.5 % of the time.
- **L0 in WeldSet (since the audit)** (harness.py:224-234):
  - `keypoint_crop`: truth-seam band with its radius bisected so that 12.67 % of points are kept, clamped to 5-60 mm.
  - **plus** per-surface labels: `face_id` through `surface_labels_oracle`, nearest-neighbour mapped for single or noisy views, restricted to the crop.
- **Ladder rung:** `crop="surfaces"`, i.e. `surface_intersection_crop` plus labels (the pre-audit L0).
- **L1:** whole cloud; Alg. 1 grows its own regions (harness.py:235).

### 2.2 Pipeline as implemented (`lit_regiongrow.py`)
1. **Mask, then preprocess** (716-721, 269-281): optional z pass-through (off), then voxel centroid.
   - Eq. 2 (printed): $h_k=\lfloor(k-k_{min})/r\rfloor$. The code anchors at $\lfloor p/r\rfloor$, a world grid; the difference is negligible.
   - r = 3 mm by default; the harness uses 1.5 mm.
2. **Eq. 3 by k-NN PCA** (284-326): $M=\frac1k\sum(\mathbf P_i-\mathbf P_0)(\mathbf P_i-\mathbf P_0)^\top$, $\delta=\frac{\lambda_0}{\lambda_0+\lambda_1+\lambda_2}$, normal = $\mathbf v_0$. With k = 10 this equals `radius_pca.surface_variation` (pinned at 1e-9).
3. **Alg. 1, as a standard visited-set reading** (333-414). Seeds are taken in ascending δ (378) and popped depth-first.
   - For each neighbour j:
     - **Threshold1 → edge:** if $|\mathbf n_s^\top\mathbf n_j|<\cos20°$, mark it edge (399-400). With `seed_edges` it may still seed if $\delta_j<0.03$ (401-405, Alg. 1 lines 12-14 literal).
     - **Otherwise → surface:** label it, and it becomes a seed if $\delta_j<0.03$ (406-409).
   - Regions under 30 points are dissolved (410-413).
4. **Supplied labels instead (L0):** `_labels_for` maps labels by nearest neighbour (417-429). `_edge_from_labels` (432-472) requires *both* that the normal turns more than 20° within the k-NN, and that a ball of one voxel contains two distinct labels (`_spans_two_labels`, k floor 12).
5. **Both §III-D clauses** (`two_surface_edges`, 507-552). A candidate needs is_edge and $\delta\ge0.03$ (the curvature gate, 534-535). In a ball of radius $r_e$, the second-largest label count must satisfy $c_{(2)}\ge0.15\sum c$ (541-551).
6. **Euclidean clustering:** link distance 2 × voxel (784). Clusters need at least 8 points.
7. **`fit_seam`** (559-591): SVD gives in-plane axes $\mathbf e_0,\mathbf e_1$; $u=(\mathbf p-\mathbf c)\cdot\mathbf e_0$, $v=(\mathbf p-\mathbf c)\cdot\mathbf e_1$.
   - `line`: $v=au+b$, giving a 2-point polyline between $u_{min}$ and $u_{max}$.
   - `poly`: a degree-2 polynomial.
8. **Pose:** sign-aligned mean normal, $\bar{\mathbf n}=\widehat{\sum\operatorname{sgn}(\mathbf n_i^\top\mathbf n_0)\mathbf n_i}$ (594-607). Not scored.

**Readings (docstring deviations 1-8, lines 60-118):**
1. Alg. 1 completed as Rabbani/PCL region growing.
2. k, Threshold1 and Threshold2 are unpublished and were tuned.
3. Two-surface test with `min_region_share`.
4. Line fit, with an optional polynomial.
5. Euclidean clustering added.
6. Keypoint crop as an ORACLE.
7. Harness voxel 1.5 vs the published 3 mm.
8. Edge test as a conjunction; the unused `labels` bug fixed.

### 2.3 Parameters

| name = default | source |
|---|---|
| `voxel_mm` = 3.0 mm (`VOXEL_MM`) | paper §IV-A; **harness 1.5 mm** [ours, dev. 7]; both logged |
| `k` = 10 | **tuned**; PCL default 30 fails |
| `smoothness_deg` = 20° (Threshold1) | **tuned**; PCL 3° |
| `curvature_thresh` = 0.03 (Threshold2) | [ours] = `ours`' value |
| `neighbourhood` = "knn" | paper |
| `radius_mm` = 2 × voxel (radius A/B only) | [ours] |
| `min_region_pts` = 30 | **invented** |
| `edge_radius_mm` = 4 × voxel (harness 6.0) | **invented** |
| `min_region_share` = 0.15 | **invented** |
| `link_mm` = 2 × voxel (harness 3.0) | **invented** |
| `min_cluster_pts` = 8 | **invented** |
| `edge_curvature_gate` = True; `edge_curv_thresh` = curvature_thresh | paper clause 1 |
| `seed_edges` = True | Alg. 1 literal |
| `label_span` = True; `label_span_radius_mm` = 1 voxel | [ours reading] |
| `curve` = "line"; `poly_degree` = 2; `fit_seam` n_samples = 2 | [ours] |
| `z_range_mm` = None | paper's pass-through, off |
| `KEYPOINT_CROP_FRACTION` = 10356/81730 = 0.1267 | paper Table II |
| `KEYPOINT_RADIUS_BOUNDS_MM` = (5, 60) | [ours] |
| keypoint crop internals: densify 0.5 mm; 64 bisection steps; tie step-back; note if \|frac − target\| > 0.03 | [ours] |

The params record lists `invented=("min_region_pts","min_cluster_pts","link_mm","edge_radius_mm","min_region_share")`.

### 2.4 Fidelity audit (2026-09-14)
- **Verdict:** "our reading failed, not the method" (phase8_plan.md:487-495). Corrected:
  1. **L0 crop:** `keypoint_crop` at 12.7 %. The old crop, `surface_intersection_crop` over all faces, kept 54-97 %; it survives as rung **`regiongrow_surface_crop_full_exterior`** (`{"crop":"surfaces"}`, 2 seeds; batch line 147).
  2. §III-D curvature clause restored (`edge_curvature_gate`).
  3. Supplied-label edge test now reads its labels (`label_span`).
  4. Alg. 1 seeding applied to edge neighbours (`seed_edges`).
  - Items 2-4 are kwarg-only; there is no batch rung for them.
- **Measured failure before the fix:** gross over-detection, with 9-31 % of points marked as edges and 10-15 % of those within 3 mm of a seam.
- **Before → after:**
  - Smoke test: T line 0.00 → 0.78; corner 0.23 → 0.93 (43 → 3 seams); square butt still 0.
  - Batch: T line 0.00 → **0.58**; corner 0.00 → **0.51**; grooved 0.00 → **0.47**; curved mean 0.67.
  - The surface-crop rung stays ≤ 0.28 on every stratum (`\regiongrowSurfaceCropMax`).

### 2.5 Arms and randomness
- **Registry:** `randomised=False`, oracle "surfaces". Batch verify_seeds = 2.
- **Chunks:** as for ransac, with suffix `lit-regiongrow`, plus the rung above.
- **kwargs available but never batched:** `neighbourhood="radius"`, `seed_edges=False`, `label_span=False`, `edge_curvature_gate=False`, `curve="poly"`.

### 2.6 Recorded numbers

**(a) Notebook 05, stored outputs (2026-08-27, pre-audit).** Corpus `out/bench`, 50 per type. Its L0 is **Yi's band** (`seam_region_oracle`), not Wei's crop. Its arms use edge radius 2.25 and link 3.0, whereas the batch uses 6.0 and 3.0.

- **k-NN (k = 10) vs 8 mm ball, median δ at the seam, by butt gap:**

  | gap (mm) | k-NN | ball |
  |---|---|---|
  | 0.5 | 0.0000 | 0.1463 |
  | 1 | 0.0004 | 0.1437 |
  | 2 | 0.0088 | 0.1362 |
  | 4 | 0 | 0.1041 |
  | 6 | 0 | 0.0971 |
  | 8 | 0 | 0.0812 |

- **One T scene** (t = 8.2 mm, voxel 1.5): 6 regions; 221 edge points; 221 span two regions; **11 seams**. Seam points: median **0.66 mm** from truth, **100 %** within 3 mm.
- **Sweep, 12 T scenes** (thickness 1.3-12.0 mm). For each edge radius the seam-point count is constant across link distances:

  | edge radius (× voxel) | seam points | link 2× (n / F1 / RMSE / Δlen / recall) | link 5× | link 10× |
  |---|---|---|---|---|
  | 1.5 | 167.67 | 3.75 / 0.30 / 3.69 / −144.16 / 0.83 | 1.83 / 0.41 / 2.04 / −1.01 / 0.57 | 0.83 / 0.24 / 3.64 / −0.90 / 0.43 |
  | 3 | 324.00 | 6.75 / 0.47 / 1.88 / −118.04 / 0.78 | 1.25 / 0.31 / 2.75 / −0.43 / 0.48 | 0.83 / 0.23 / 3.55 / −0.43 / 0.43 |
  | 4 | 356.58 | 6.75 / 0.44 / 1.88 / −118.04 / 0.74 | 1.25 / 0.31 / 2.86 / −0.41 / 0.48 | 0.83 / 0.23 / 3.66 / −0.41 / 0.43 |
  | 5 | 359.17 | 6.75 / 0.44 / 1.88 / −118.04 / 0.74 | 1.25 / 0.31 / 2.88 / −0.41 / 0.48 | 0.83 / 0.23 / 3.67 / −0.41 / 0.43 |

- **Voxel sweep** (seam_recall here is a count out of 23):

  | voxel | RMSE med | ME med | F1 | n seams | matched |
  |---|---|---|---|---|---|
  | 1.5 mm | 1.882 | 2.499 | 0.440 | 6.75 | 17 |
  | 2 mm | 2.587 | 3.251 | 0.405 | 4.0 | 18 |
  | 3 mm | 3.998 | 4.828 | 0.279 | 2.667 | 18 |

- **Ladder** (15 T + 15 corner): median n_in, median seam points, mean F1, median RMSE.

  | arm | T: n_in / pts / F1 / RMSE | corner: n_in / pts / F1 / RMSE |
  |---|---|---|
  | L0-band | 9170 / 161 / 0.374 / 2.303 | 6456 / 386 / 0.472 / 0.654 |
  | L0-crop | 49466 / 2588 / 0.114 / 3.151 | 48626 / 2538 / 0.057 / 13.180 |
  | L0-paper | 49466 / 5635 / 0.069 / 41.850 | 48626 / 5860 / 0.000 / 85.174 |
  | L1 | 60164 / 2598 / 0.116 / 3.148 | 69495 / 2534 / 0.052 / 15.010 |

- **50 T + 50 corner:**
  - L0: F1 0.413; P 0.496; 230 seam points; median 2.6 mm from truth; 55 % within 3 mm.
  - L1: F1 0.113; P 0.095; 2217 points; 84.7 mm; 11 % within 3 mm.
  - Per-scene medians: L0 T F1 0.421, corner 0.501; L1 T 0.063, corner 0.045.
- **Fragments:**

  | arm / joint | n | RMSE | ME | coverage | P | R |
  |---|---|---|---|---|---|---|
  | L0 T | 78 | 1.169 | 1.492 | 0.104 | 0.757 | 0.370 |
  | L0 corner | 84 | 0.939 | 1.366 | 0.350 | 0.401 | 0.784 |
  | L1 T | 90 | 3.187 | 3.453 | 0.104 | 0.114 | 0.282 |
  | L1 corner | 86 | 3.923 | 5.113 | 0.236 | 0.077 | 0.351 |

  - 35 % of L0 fragments have RMSE ≤ 0.56 mm; 41 % have ME ≤ 1 mm.
- **Reproduction:** none reached the paper's 0.37/0.54/0.56 mm.

**(b) README and dataset_plan (pre-audit)**
- Four-rung "50 T + 50 corner" table: band 0.38 / 0.45; crop 0.13 / 0.10; paper 0.05 / 0.02; L1 0.13 / 0.10; RMSE 2.2 / 3.8 / 40.9 / 3.8; points 166 / 2101 / 5569 / 2102.
- Crop-vs-L1 agreement 0.125/0.126 and 0.102/0.101.
- 331 k of 522 k points kept by the surface crop.
- Seam band: 94 k points, 100 % within 20 mm. Surface crop: 331 k, 31 %.

**(c) Current numbers (nb15 and macros)**
- Full_exterior median F1:

  | T/line | circle | ellipse | saddle | rounded_rect | swept | corner | butt | grooved | arc | lap | edge |
  |---|---|---|---|---|---|---|---|---|---|---|---|
  | 0.58 | 0.75 | 0.74 | 0.29 | 0.75 | 0.80 | 0.51 | 0.51 | **0.47** | 0.31 | 0.39 | 0.72 |

- Single view: 0.03 / 0.08 / 0.13 / 0.19 / 0.13 / 0.20 / 0.01 / 0.08 / 0.00 / 0.44 / 0.04 / 0.31.
- T-joint 0.77 vs angle joint 0.55. T/line RMSE 2.16 mm.
- L0 → L1: T 0.72 → 0.17; butt 0.46 → 0; corner 0.51 → 0.55; edge 0.72 → 0.08; lap 0.39 → 0.08. Pooled 0.58 → 0.11.
- Noise: 0.09 / 0.19 / 0.22.
- Fixture: ΔF1 −0.14; ΔP −0.23; +8.5 seams (butt −0.52).
- Task 2: selection 0.72 / 0.57 / 0.76 / 0.78 / 0.83; localisation 0.5 / 6.2 / 3.0 / 2.4 / 1.3 mm.
- Cost: 0.45 / 0.22 / 3.61 s.

### 2.7 Measured limitations
- PCL defaults (k = 30, 3°) at the 3 mm grid give a ~9 mm reach on an 8 mm plate, so everything becomes an edge.
- A wall between fragmenting and merging: fillets are one plate thickness (8 mm) apart.
  - Coverage per fragment: 10 % on T, 35 % on corner (pre-audit).
  - The published 3 mm voxel is the worst setting.
- Face-vs-part category error: L1 accepts about 10× the points, 89 % of them away from the seam.
- Strong coarse-stage dependence (0.58 → 0.11).
- Collapses in single view.
- Fixture phantoms.
- The harness fits **straight lines per cluster even on curved families** (`curve="line"`; `poly` is never batched).

### 2.8 Discrepancies (lit-regiongrow)
1. **The §III-C justification is not the paper's text** (module lines 11-14; keypoint_crop docstring 181-187, "A keypoint detector … finds the seams").
   - The paper's keypoints are surface-centre click prompts; FastSAM segments surfaces and the seam region is *derived* from their intersections.
   - `lit_ransac.surface_labels_oracle`'s docstring describes it correctly.
   - The ICRA text ("prompted by a weld-keypoint detector") inherits the claim.
2. nb05 cell 0 still lists §III-C as `seam_region_oracle`, "the same one lit-ransac gets". This is stale.
3. nb05 cell 8 says "96 % within 3 mm, median ~1,3 mm, 17 seams"; the stored output shows **100 %, 0.66 mm, 11 seams**.
4. nb05 cell 9 says "more than a factor of six". The `edge_radius_mm` docstring says "1,5×voxel keeps 164 … 4-5×voxel keeps over a thousand". Stored output: 167.67 → 356.58 / 359.17 points (×2.1, never > 1000).
5. nb05 cell 11 says "length error −127 mm on a 188 mm seam"; stored output −144.16 / −118.04.
6. The four-rung table (nb05 cell 17, README:602-607, dataset_plan:400-405) is labelled "full 50 + 50". The stored output of that very cell is **15 + 15**, with different values (see 2.6a).
7. nb05 cell 21 and README:611-612: "Given the paper's own crop it works — F1 0,41". That arm used **Yi's band**, which nb05 cell 15 itself says is not this paper's stage.
8. nb05 cell 24 says "The L1 arm is untested here", although cells 18-22 test it.
9. "T and corner F1 0,47 / 0,49" (nb05, dataset_plan:461) cannot be found in the stored outputs; the medians are 0.421 / 0.501.
10. Counts of the paper's experiments disagree:
    - nb05 says "Wei et al.'s four workpieces are fillet welds on steel structures".
    - Module deviation 4 says "three linear workpieces and one curved one".
    - The paper's §IV says "two linear-weld-seam workpieces and one curved-weld-seam workpiece". Table I has rows linear-2, linear-10 and curved-1; Fig. 5 uses a 5-seam weldment; §IV-C is an industrial steel-structure scenario.
11. Code nuance in `region_grow`.
    - The neighbour skip test is `labels[j] != -1` (line 397), so an edge point marked during one region can later be **labelled** by another region.
    - An edge point can also start a region (385-389).
    - The docstring claim "Edge points are still never *labelled*" (362) therefore holds only conditionally.
12. dataset_plan:2088 says the fixture leaves lit-regiongrow "unmoved" (pre-audit); post-audit it is −0.14.
13. MANUAL §9 "Grooved butts defeat five of six (≈0)" is pre-audit; regiongrow now reads grooves at 0.47.
14. dataset_plan says 8 tests; there are 16 now.
15. Notebook arms used edge radius 2.25 mm; the batch uses 6.0 mm, so the notebook numbers are not comparable to the batch.

### 2.9 Tests (`tests/test_lit_regiongrow.py`, 16)
- Eq. 3 equals `surface_variation` (1e-9).
- At the seam of a 6 mm-gap butt, the median δ of an 8 mm ball exceeds 5× the k = 10 value.
- Region 0 claims the minimum-curvature point.
- Threshold1 makes it a boundary detector (10° vs 89°).
- The two-surface test keeps fold points, not rims (90th percentile < 6 mm).
- The planar projection saves the line fit (ribbon).
- `VOXEL_MM` = 3.0 and (k, smoothness, curvature) = (10, 20.0, 0.03).
- End to end on a phase3 T scene ≥ 6 mm, with the band: at least 2 regions and a match.
- Keypoint crop keeps 12.7 ± 3 %, within its bounds, empty note, and the mask is exactly a ball.
- A fixed radius is honoured.
- No seams: keep everything, note contains "NOT".
- The curvature gate rejects a flat label boundary (without the gate > 0 points; with it 0).
- The gate keeps a real fold.
- `_edge_from_labels` narrows correctly: zero with a single label; on the fold < 4 mm.
- `seed_edges` keeps 2 regions and the same edge set; edges stay unlabelled on the fold.
- Invented constants, the 1.5 vs 3.0 voxel, gate, threshold, seed_edges and span radius 1.5 are all present in params.

---

## 3. lit-lobb (Zhang et al. 2025, pipeline plus descriptor)

### 3.1 Papers
- **Pipeline (PDF p.1):** Yuankai Zhang, Yusen Geng, Xincheng Tian (corresponding), Yujie Sun, Xiaolong Xu.
  - Title: "A novel weld seam extraction method with semantic segmentation and point cloud feature for irregular structure workpieces."
  - *Robotics and Computer-Integrated Manufacturing* 95 (2025) 102987 (Full length article). doi:10.1016/j.rcim.2025.102987.
  - Received 4 Jun 2024; revised 26 Jan 2025; accepted 8 Feb 2025; online 21 Feb 2025.
- **Descriptor (its ref. [24]; PDF at `papers/A_Feature_Extraction_Approach_Over_Workpiece_Point_Clouds_for_Robotic_Welding.pdf`):** Yuankai Zhang, Yusen Geng, Xincheng Tian, Fuquan Zheng, Yong Jiang, Min Lai (corresponding: Tian, Zheng).
  - Title: "A Feature Extraction Approach Over Workpiece Point Clouds for Robotic Welding."
  - *IEEE Trans. Automation Science and Engineering* 22 (2025) 75-84. doi:10.1109/TASE.2023.3345868.
  - Received 18 Oct 2023; accepted 19 Dec 2023; published 1 Jan 2024; current version 13 Jan 2025.
  - RCIM cites it as "(2024)".
- **Mechanism:** Local Oriented Bounding Box (LOBB) features are computed on an ROI grown from component-mask boundaries. Features are tanh-activated and split by hierarchical 1-D binary K-means into boundary and crease points; crease points are fitted by a dominant-axis polynomial.
- **Their coarse stage:**
  - K-Net semantic segmentation into component masks "A"/"B" (aACC 99.09 %, mIoU 97.35 %, mACC 98.75 %).
  - Edge pixels are those whose n × n neighbourhood holds two mask values (eq. 2, n = 3 per §4.1), lifted to 3D.
  - ROI by k-NN shape extension (§3.3).
- **L0 in WeldSet:** `object_id` per point (`part_labels_oracle`), then the 3D eq. 2 test, then the ROI (harness.py:253).
- **L1:** no labels; the whole voxelised cloud is the ROI (lit_lobb.py:626).

### 3.2 Pipeline as implemented (`lit_lobb.py`, `detect` 591-698)
1. **Voxel downsample** 1.0 mm (594).
2. **Spacing** $\bar s$ = mean nearest-neighbour distance (20 000 samples, seed 0) (180-203).
   - Edge radius $r_e=1.5\,\bar s$ (`EDGE_RADIUS_SCALE`), the reading of "3 × 3 pixels" at about 0.17 mm/px (206-220).
   - A fixed 3.0 mm ball is the legacy value.
3. **Eq. 2 in 3D** (`part_boundary_points`, 223-254): point i is a seed if $\min_{j:\ell_j\ne\ell_i}\lVert\mathbf p_i-\mathbf p_j\rVert\le r_e$. Labels are nearest-neighbour mapped to voxel centroids (622-623).
4. **ROI** = seeds ∪ 30-NN of the seeds (257-275). If the ROI has fewer than 8 points the result is "ROI too small after the coarse stage" (628-631).
5. **LOBB on the ROI points only** (282-347; tree built on `P[sub]`).
   - Neighbourhood: ball R = 2.5 mm, at least 4 neighbours. PCA axes, then extents $=\max-\min$ in that frame, **sorted** $l\ge w\ge h$ with the axes permuted (339-340).
   - Flatness (eq. 4): $f=\tan\theta=\dfrac{h}{\sqrt{l^2+w^2}}$.
   - Centre offset (eq. 7, box-corner frame): $d=\sqrt{(x_Q-\tfrac l2)^2+(y_Q-\tfrac w2)^2}$.
   - Corner entropy (eq. 8): $s=\sqrt{\tfrac{(l-m)^2+(w-m)^2+(h-m)^2}{3}}$, $m=\tfrac{l+w+h}3$.
6. **Activation (eq. 9)**, over the whole ROI (350-368): $t_1=4\frac{t-t_{min}}{t_{max}-t_{min}}-2$, $t_2=\tanh t_1$, $t'=\frac{t_2-\min t_2}{\max t_2-\min t_2}$.
7. **1-D binary K-means**, seeded at min and max, iterated until labels are stable (≤ 100 iterations); "True" is the high cluster (371-395).
8. **Hierarchy (Fig. 9 reading)** (641-651):
   - boundary = KM($t'_d$) over the ROI;
   - crease = KM($t'_f$) over the **non-boundary** subset;
   - `kmeans="flat"` is the pre-audit alternative: both splits over the ROI, then crease ∧ ¬boundary.
9. **Optional corner layer** (off) (398-420, 654-659): entropy recomputed on boundary ∪ crease; the low-s cluster marks corners, which are cut from the crease set before fitting.
10. **Optional Mean-Shift** (off) (427-458). Code weights: $w_j=\exp(-\lVert\mathbf x_j-\mathbf x_i\rVert^2/h^2)$ within a 3h ball, h = 2 × 2.5 = 5 mm, up to 20 iterations, tolerance 1e-3 mm.
    - Paper (eqs. 4-7): $K\propto\phi(\lVert\mathbf x\rVert^2/h^2)$ with $\phi(u)=e^{-u/2}$.
11. **Euclidean clustering** with link 3 × voxel = 3 mm, at least 8 points (684-694).
12. **`fit_dominant_axis`** (RCIM eq. 8, $f(u)=\sum_{i=0}^{3}a_iu^i$) (461-495):
    - u = argmax span over x and y, or z if z dominates;
    - two polynomials of degree 3 against u;
    - 80 samples (§4.1/§4.3).

**Readings (docstring deviations 1-6, lines 56-118):**
1. Pixel window → resolution-relative ball (the root-gap cost is documented).
2. Deterministic 1-D K-means.
3. Mean-Shift unparameterised and off.
4. Multi-seam clustering added.
5. Hierarchy restored.
6. Corner layer off, with a measured reason and a TODO.

### 3.3 Parameters

| name = default | source |
|---|---|
| `LOBB_RADIUS_MM` = 2.5 mm | T-ASE §III-C ("according to the weld width") and §IV-B ("through many experiments", "based on … workpiece's dimensions") |
| `N_SAMPLES` = 80 | RCIM §4.1/§4.3 |
| `nonlinear_activation` = True | T-ASE eq. 9 |
| `kmeans` = "hierarchical" | Fig. 9 reading; "flat" is the rung |
| `drop_boundary` = True | paper |
| `EDGE_RADIUS_SCALE` = 1.5 × spacing | reading of n = 3 pixels |
| `EDGE_RADIUS_MM_LEGACY` = 3.0 mm | pre-audit value |
| `ROI_K` = 30 | unpublished |
| `POLY_DEGREE` = 3 | unpublished ("times") |
| `voxel_mm` = 1.0 | [ours]; paper "appropriately downsampled" |
| `lobb_k` = None (ball); `min_neighbors` = 4 | [ours] |
| `corner_layer` = False | [ours default]; paper has it |
| `mean_shift` = False; bandwidth = 2 × LOBB radius = 5 mm; window 3h; 20 iterations; tolerance 1e-3 | [ours] |
| `link_mm` = 3 × voxel; `min_cluster_pts` = 8 | [ours] |
| K-means: 100 iterations, initialised at min/max | [ours] |
| spacing: 20 000 samples, seed 0 | [ours] |

### 3.4 Fidelity audit (2026-09-14)
- **Corrected** (phase8_plan.md:503-512):
  1. T-ASE Fig. 9 hierarchy: crease split on the non-boundary subset (was flat).
  2. Edge window: 1.5 × spacing (was a fixed 3 mm ball, about 4× spacing).
  3. Extents sorted $l\ge w\ge h$ (were eigen-ordered).
  4. 80 output points (were 40).
  5. Corner layer implemented but off.
- **Rung:** **`lobb_flat_3mm_full_exterior`** (`{"kmeans":"flat","edge_radius_mm":3.0}`, 2 seeds; batch line 144). It keeps the new sort and the 80 samples, so it is **not bit-identical** to the pre-audit code.
- **Before → after:**
  - Smoke test: edge 0.41 → 0.83; corner max error 4.3 → 0.4 mm. Gapped butts now return no boundary (F1 0 either way; before, a wrong seam).
  - Batch: edge 0.29 → **0.63**; **corner 0.99 → 0.00**; butt 0.59 → 0.27.
  - The rung reproduces the old values: corner 1.00, butt 0.60, edge 0.22 (`\lobbFlatCorner`, `\lobbFlatButt`, `\lobbFlatEdge`). Corners return nothing because the window cannot cross root gaps up to 4.9 mm (`\cornerGapMax`).
  - Test scene (out/bench corner): RMSE 0.428 / max 4.321 mm → 0.112 / 0.416 mm.
  - Synthetic T (σ = 0.1): crease points within 2 mm of truth, 94.0 % (flat) → 95.6 % (hierarchical).
- **Corner-layer precondition failure:** 1228 of 1241 crease points were classed as corners on the first bench corner scene.

### 3.5 Arms and randomness
- **Registry:** `randomised=False`, oracle "objects". The K-means is deterministic and the spacing subsample is seeded. Batch verify_seeds = 2.
- **Chunks:** standard set plus `lobb_flat_3mm_full_exterior`.
- **kwargs available but never batched:** `nonlinear_activation=False`, `drop_boundary=False`, `corner_layer=True`, `mean_shift=True`, `roi_mask`.

### 3.6 Author correspondence (Yuankai Zhang, first author, 2026-08-27; quoted from lit_lobb.py:120-139)
The docstring is the **only stored record**: there is no email in `notes/correspondence/` (which holds only a lit-quadric draft).
1. "The oracle masks are endorsed": supplying per-component masks from generator ground truth "is equivalent to using the corresponding component masks obtained from a 2D image segmentation network for testing the subsequent modules. Therefore, this treatment is acceptable."
2. Mean-Shift off is accepted for density-uniform data, with the caveat that the filter "is intended to further ensure the stability of weld reconstruction". The docstring commits: "the Phase 4 write-up should report the `mean_shift=True` arm alongside the default".
3. LOBB radius: "The selection of the LOBB radius is not a free tuning parameter" (set by the weld width).
4. Closed contact perimeter: "you may consider using other curve-fitting methods, such as B-spline curves". A B-spline variant is an author-suggested **extension**, to be reported as a separate arm.

**Status:** no batch chunk, notebook or test ever runs `mean_shift=True`, and no B-spline variant exists. The README (576) nevertheless says the arm "gets reported anyway".

### 3.7 Recorded numbers

**(a) Notebook 06, stored outputs (2026-09-08, pre-audit; out/bench, view "full", voxel 1.0)**
- **First corner scene:** ROI 4535 of 197 865 points; crease 1488; boundary 943; **1 seam vs 2 true**. P 0.993, R 1.000, F1 0.996; **RMSE 0.377 mm, ME 4.315 mm**; covers 109 %.
- **12 scenes per type, medians:**

  | arm / joint | ROI | crease | det_med | in3 | F1 | RMSE | ME |
  |---|---|---|---|---|---|---|---|
  | L0 T | 4234 | 1285 | 1.500 | 0.963 | 0.664 (P 0.972 / R 0.500) | 1.734 | 4.963 |
  | L0 butt | 3352 | 783 | 1.993 | 0.773 | 0 | 4.259 | 4.907 |
  | L0 corner | 2561 | 767 | 1.102 | 0.995 | 0.994 | 0.564 | 3.362 |
  | L0 edge | 49 318.5 | 20 836 | 56.342 | 0.018 | 0.003 | 67.756 | 109.637 |
  | L0 lap | 7201.5 | 1051 | 2.062 | 0.641 | 0 | 13.955 | 19.229 |

  - L1 F1 = 0 for every type except lap (0.011). L1 RMSE 38.7-85.6 mm; ROI 107-135 k points.
- **T closed perimeter** (link 1.5): crease 1132 points = fillet A 513 + fillet B 591 + neither 28 (2.5 %). All 28 lie within 15 mm of a labelled endpoint (median 3.8 mm).

**(b) README and dataset_plan (2026-08-20, 25 per type, pre-audit)**

| arm | joint | F1 | median RMSE | crease within 3 mm |
|---|---|---|---|---|
| L0 | corner | 0.83 | 0.51 mm | 82 % |
| L0 | T | 0.47 | 2.82 | 76 % |
| L0 | butt | 0.33 | 3.14 | 64 % |
| L0 | lap | 0.04 | 28.5 | 52 % |
| L0 | edge | 0.02 | 64.7 | 3 % |
| L1 | corner | 0.02 | 80.1 | 10 % |
| L1 | T | 0.09 | 48.1 | 20 % |

- 18 of 125 L0 scenes reach < 0.7 mm. Corner 0.83 → 0.02 vs `ours` 0.75 → 0.03.
- MANUAL §9 and dataset_plan Phase 6b (pre-audit): lobb "won"; only method nonzero on every stratum but grooved; Task 2 0.79 selection / 0.7-1.6 mm; noise 0.44 → 0.31; ladder 0.43 → 0.03; fixture +23 seams, precision −0.22, ΔF1 −0.30 (T) / −0.49 (corner).

**(c) Current numbers (nb15 and macros)**
- Full_exterior median F1:

  | T/line | circle | ellipse | saddle | rounded_rect | swept | corner | butt | grooved | arc | lap | edge |
  |---|---|---|---|---|---|---|---|---|---|---|---|
  | 0.80 | 0.45 | 0.39 | 0.32 | 0.20 | 0.80 | **0.00** | 0.27 | 0.00 | 0.49 | 0.50 | 0.63 |

- Nonzero on 10 of 12 strata. Single view: 0.16 / 0.31 / 0.40 / 0.39 / 0.29 / 0.48 / 0 / 0 / 0 / 0.07 / 0.28 / 0.25.
- T-joint 0.96 vs angle joint 0.74. Curved mean 0.43. T/line RMSE 1.08 mm.
- L0 → L1: T 0.50 → 0.04; butt 0 → 0.02; corner 0 → 0; edge 0.63 → 0.09; lap 0.50 → 0.15. Pooled **0.40 → 0.03**.
- Noise: 0.22 / 0.26 / 0.30 (rises).
- Fixture: ΔF1 0.00 overall (T −0.15, edge −0.36); **+22 seams**.
- Task 2: selection 0.79 / 0.30 / 0.29 / 0.63 / 0.80 (pooled 0.56); localisation **0.6 / 0.6 / 0.7 / 0.7 / 1.2 mm** (best).
- Cost: 0.50 / 0.13 / **10.42 s** (L1).

### 3.8 Measured limitations
- The faithful window cannot cross a root gap, so corners and butts return nothing.
- A T joint's closed contact perimeter is fitted by one polynomial; no link distance separates the fillets.
- Precision failures from real creases that are not seams: far-side/bore creases on rings, corner inside/outside ties, fixture-contact lines.
- The Mean-Shift mechanism (density at creases) is absent from uniform sampling.
- The corner layer's precondition (a curve-thin edge cloud) is not met.
- Steep ladder.

### 3.9 Discrepancies (lit-lobb)
1. **The corner "reproduction" claim is too strong.**
   - nb06 cell 13 says "reproduces the paper on the first run", but the stored ME is 4.315 mm (> 1.2). The test docstring quotes 0.428/4.321 for the same scene, so the code state differed from nb06's run.
   - README:570-571 and dataset_plan report RMSE only, pre-audit.
   - Post-audit corpus corner F1 is 0.00.
   - RCIM's < 1.2 / < 0.7 mm are **per-axis errors after endpoint alignment on 2 seams**, not 3D errors.
2. dataset_plan:542 says "On a T joint it returns F1 0,00", against its own table (0.47) and nb06 (0.664).
3. "Measured at 1,5/2/3/5 mm" (dataset_plan:551) vs nb06's sweep of [1, 1.5, 2, 3, 5, 8] mm (a plot only).
4. Deviation 1 says the resolution-relative ball is "~1,1 mm"; a test docstring says "~1,5 mm".
5. The `mean_shift=True` arm and the B-spline arm were never run (3.6).
6. MANUAL §9 is pre-audit throughout, and its line "lit-lobb needs no segmentation" contradicts the ladder (0.40 → 0.03).
7. `lobb_flat_3mm` is not the exact pre-audit reading (0.22 vs 0.29 on edge).
8. Deviation 1 names the stratum `butt/line_square`; corpus strata are named `butt/line`.
9. "local-bounding-box flatness" (MANUAL §8.1, thesis ch4:143-145) omits the word "oriented" (LOBB = Local **Oriented** Bounding Box).
10. Mean-Shift kernel: the code uses $e^{-d^2/h^2}$, the paper $e^{-d^2/(2h^2)}$. This is a factor √2 in effective bandwidth; the arm is off and the bandwidth is unpublished.
11. RCIM eq. 1 is garbled as printed (index range $[-2n-1,2n-1]$). The module's 3 × 3 reading rests on §4.1, "neighborhood size … as 3", and this is not noted.
12. Test counts: dataset_plan says 11, there are 16 now.

### 3.10 Tests (`tests/test_lit_lobb.py`, 16)
- LOBB radius = 2.5.
- One lifted point gives $f_{spike}>10f_{flat}$ while the variance ratio < $0.5f$.
- The l ≥ w ≥ h sort bites: the docstring's eigen-ordered extents (40, 11.5, 16) give f = 0.385 unsorted vs 0.268 sorted. The test asserts the sorted ratio, a difference > 0.1, and an unchanged entropy.
- Flatness: crease > 5× face.
- Centre offset: rim > 3× interior, while rim flatness < 0.1.
- Activation empties the middle of the distribution.
- K-means is deterministic.
- Eq. 2 marks only the junction and nothing with a single label.
- Window = 1.5 × spacing at grid steps 0.5 / 1 / 2; an explicit 3.0 wins and gives a wider band.
- A 2 mm gap: no boundary at the default window, crossed by a 3 mm ball; `detect` returns "ROI too small".
- The ROI is a superset of the seeds but still a subset of the cloud.
- Hierarchy: no face points classed as crease, purity ≥ flat.
- Corner layer is wired (corner ⊆ crease ∪ boundary; off by default).
- Dominant-axis fit < 0.5 mm; N_SAMPLES = 80.
- Corner reproduction: best row RMSE < 0.7 and ME < 1.2 on the first out/bench corner scene.
- T perimeter: > 90 % of crease points within 3 mm; off-seam points near the ends; n_seams < 2.

---

## 4. lit-ppf (Wang et al. 2024)

### 4.1 Paper
- **Citation (PDF p.1):** Shengbo Wang, Zengxu Li, Guodong Chen, Yaobin Yue (corresponding).
  - Title: "Weld seam object detection system based on the fusion of 2D images and 3D point clouds using interpretable neural networks."
  - *Scientific Reports* 14 (2024) 21137. doi:10.1038/s41598-024-71989-w.
  - Received 28 Jun 2024; accepted 2 Sep 2024. Qingdao Univ. of Sci. & Tech.
- **Mechanism:**
  - Planes are extracted by PPF coplanarity from samples taken every 25 mm, with leftover points assigned by point-to-plane distance in ascending plane-size order.
  - Orthogonal plane pairs are found by local Hough voting in (θ, ρ) (eqs. 21-23).
  - Feature points are selected by a distance threshold (0.1 mm) and refined with DBSCAN plus outlier removal.
  - The two farthest feature points are the corner pair.
- **Their coarse stage:** Faster R-CNN (ResNet-50) 2D weld boxes; recognition rate 92 %. In the experiment the box was simulated by "the rectangular feature area … containing a 100 mm long side".
- **L0 in WeldSet:** the **same `band` oracle as Yi's** (20 mm half-width) (harness.py:245). There is no box oracle.
- **L1:** whole cloud.
- **Extra normal arm:** `normals="estimate"` (local PCA, k = 20, the faithful setting) vs `"exact"` (generator normals).

### 4.2 Pipeline as implemented (`lit_ppf.py`, `detect` 527-687)
1. **Mask, then voxel 1.5 mm** (533-542).
2. **Derived tolerances** (543-550). The spacing $s$ is the **median** nearest-neighbour distance on at most 2000 samples (`radius_pca.mean_spacing_mm`, despite its name).
   - $\tau_f=\max(1.5s,1)$ mm
   - DBSCAN $\varepsilon=3\tau_f$
   - dedup offset $2\tau_f$
   - $\epsilon_c=\max(2s,0.5)$ mm
3. **Normals:** PCA with k = 20, or exact normals mapped by nearest neighbour (570-580).
4. **`ppf_planes`** (162-291).
   - Seeds: nearest points to 25 mm voxel centroids.
   - `grow(b)`: point c is coplanar with seed b if $|(\mathbf c-\mathbf b)^\top\mathbf n_b|\le\epsilon_c$, $|\mathbf n_c^\top\mathbf n_b|\ge\cos15°$, and $|(\mathbf c-\mathbf b)^\top\mathbf n_c|\le\max(\epsilon_c,0.05\lVert\mathbf c-\mathbf b\rVert)$. A plane needs at least 30 points; its normal is refit by PCA (202-226).
   - Re-seeding, 8 rounds, from the lowest unclaimed index.
   - Duplicates merged: parallel within 15° and offset ≤ $2\epsilon_c$.
   - Leftovers go to the first plane within $\epsilon_c$ (distance only), visiting planes in **ascending size** (the printed order).
5. **Gate per pair** (`opp_vote` 362-372): $\psi=\arccos|\mathbf n_a^\top\mathbf n_b|$.
   - `orthogonal`: $|\psi-90°|\le15°$.
   - `intersecting`: $\psi\ge20°$.
6. **Vote** (373-394):
   - Eq. 21: $R_z=R_x(\omega)R_y(\phi)$ with $\phi=\operatorname{atan2}(n_x,n_z)$, $\omega=\operatorname{atan2}(r_y,r_z)$.
   - Pairs are strided: about 20 × 20 = 400.
   - Eq. 22: $\theta=\operatorname{atan2}((R_z\mathbf n_2)_y,(R_z\mathbf n_2)_x)$, **folded mod π**, 36 bins over [0, π).
   - Eq. 23: $\rho=\mathbf n_2^\top(\mathbf x_1-\mathbf x_2)$. It is computed, but only its median is returned and it **never decides acceptance**.
   - Accept if the peak θ-bin count ≥ 30 (absolute when ≥ 1, a fraction of votes when < 1) (605-609).
7. **Line in closed form** (613-618): $\mathbf d=\widehat{\mathbf n_a\times\mathbf n_b}$; $[\mathbf n_a;\mathbf n_b;\mathbf d]\mathbf p_0=[\mathbf n_a^\top\mathbf c_a,\ \mathbf n_b^\top\mathbf c_b,\ \mathbf d^\top(\mathbf c_a+\mathbf c_b)/2]$.
8. **Feature points** (625-628): within $2\tau_f$ of both planes **and** $\lVert(\mathbf p-\mathbf p_0)\times\mathbf d\rVert\le\tau_f$. At least 10 points are required.
9. **DBSCAN**(ε, minPts = 6), keeping the **largest** cluster (632-644). Length must be ≥ 5 mm.
10. **`merge_collinear`** (431-460, 656-674): lines merge if $|\mathbf d_1^\top\mathbf d_2|\ge\cos5°$ and the anchor offset ≤ $2\tau_f$. Feature sets are unioned and the corners are $\mathbf p_0+t_{min/max}\mathbf d$ ("two furthest").

**Recorded paper contradiction (docstring lines 32-37):** the prose proposes PPF *instead of* RANSAC, but the implementation paragraph says planes were "extracted … using RANSAC" (verified in the PDF). The module implements the prose. Algorithm 1 is an image in the PDF, so the pooled-F and "is_orthogonal" claims cannot be verified from extracted text.

### 4.3 Parameters

| name = default | source |
|---|---|
| `SAMPLE_INTERVAL_MM` = 25 | paper |
| `FEATURE_TOL_PAPER_MM` = 0.1 (not the default) | paper, for a 50 µm Photoneo PhoXi M (1.3 M points, 520 × 390 × 380 mm range) |
| `assign_order` = "asc" | paper ("least to most") |
| `normals` = "estimate" | paper (PCL) |
| `voxel_mm` = 1.5 | [ours] |
| `normal_k` = 20 | [ours] |
| `coplanar_tol_mm` = None → max(2s, 0.5) | [ours] |
| `angle_tol_deg` = 15 | [ours] |
| `min_plane_pts` = 30 | [ours] |
| `pair_rule` = "orthogonal" | paper |
| `ortho_tol_deg` = 15 | **[ours]** |
| `fold_min_deg` = 20 | [ours] |
| `theta_bins` = 36 | [ours] |
| `min_votes` = 30.0 (about 7.5 % of 400) | [ours] |
| `feature_tol_mm` = None → max(1.5s, 1.0) | [ours] |
| `dbscan_eps_mm` = 3·τ_f; `dbscan_min_samples` = 6 | [ours] |
| `keep_clusters` = "largest" | reading |
| `min_feature_pts` = 10; `min_seam_length_mm` = 5 | [ours] |
| `dedup` = True | reading of Alg. 1 |
| `dedup_angle_deg` = 5; `dedup_offset_mm` = 2·τ_f | [ours] |
| re-seed 8 rounds; relative perpendicular tolerance 0.05\|d\|; refit if \|v₀·n_b\| > 0.5; plane membership 2·τ_f; n_pairs 400 | [ours] |

### 4.4 Fidelity audit (2026-09-14)
- **Verdict:** "the gate is theirs, the tolerance is ours" (phase8_plan.md:513-519). Code diff `c670770`→`a98c19d`:
  1. One seam per scene, by dedup and collinear merge (was per pair).
  2. DBSCAN keeps the dominant cluster (was all non-noise).
  3. θ folded mod π into 36 bins over [0, π) (was 36 bins over [−π, π)).
  4. `min_votes` fraction semantics.
  5. `pair_rule` and tolerance made switchable. The orthogonal gate itself is unchanged: the old test `cosd > cos(90−tol)` was equivalent.
- **Rungs** (batch 150-154, 2 seeds each):
  - **`ppf_perpair_full_exterior`** (`{"dedup":False,"keep_clusters":"all"}`). It keeps the new θ binning, so it is not exactly the old code.
  - **`ppf_intersecting_full_exterior`**, **`ppf_tol30_full_exterior`**, **`ppf_tol45_full_exterior`**.
  - The pre-audit rung **`ppf_exact_normals_full_exterior`** also exists.
- **Numbers:**
  - "unchanged within 0,02". Smoke test: corner 0.00 → 1.00 at 30°.
  - Batch macros:

    | rung | T-joint | angle joint | corner | curved precision |
    |---|---|---|---|---|
    | headline (15°) | 0.62 | **0.03** | 0.50 | ≤ 0.28 (curved recall ≥ 0.47) |
    | tol30 | 0.59 | 0.60 | 0.50 | 0.28 |
    | tol45 | 0.58 | 0.60 | 0.50 | 0.28 |
    | intersecting | 0.57 | 0.59 | 0.46 | 0.28 |
    | perpair | 0.63 | 0.03 | 0.50 | 0.31 |

  - Exact-normals rung ΔF1: T +0.02, corner −0.01, butt **+0.49**, lap +0.06, edge **+0.42**.

### 4.5 Arms and randomness
- **Registry:** `randomised=False` (deterministic as published; this corrects the plan's grouping), oracle "band". Batch verify_seeds = 2.
- Standard chunks plus the five rungs above.

### 4.6 Recorded numbers

**(a) Notebook 08, stored outputs (2026-09-08, pre-audit; out/bench)**
- **Synthetic checks:** a fold yields 2 planes; `opp_vote` gives 420 votes at θ = 90.0°. Two parallel sheets: 0 seams, one pair `not_orthogonal` at 0°. Fold: one seam spanning 100.0 mm of a 100 mm crease.
- **Four arms, 15 scenes per type** (run_matrix, full_exterior, seed 0). Mean F1 per type, with the L0 median RMSE in brackets:

  | arm | T | butt | corner | edge | lap |
  |---|---|---|---|---|---|
  | L0 estimate | 0.411 (1.056) | 0.590 (2.134) | 0.545 (1.854) | 0.435 (2.064) | 0.435 (0.423) |
  | L0 exact | 0.656 (1.337) | 0.906 (0.315) | 0.667 (1.134) | 1.000 (0.215) | 0.487 (0.295) |
  | L1 estimate | 0.152 | 0.198 | 0.153 | 0.340 | 0.148 |
  | L1 exact | 0.134 | 0.221 | 0.144 | 0.256 | 0.159 |

  - L1 mean precision 0.074-0.294.
- **Paper metric** (length error / truth length; L0, estimated normals, view "full"):
  - T: n = 16, median 0.50 %, mean 0.50 %, RMSE 0.12 mm.
  - Corner: n = 22, median 0.39 %, mean 3.38 %, RMSE 0.79 mm.
  - Paper: T 2.17 % average, max 3.84 mm, min 0.21 mm; V 2.4 % average, max 6.82 mm, min 0.28 mm (on a 50 × 50 × 100 workpiece).
- **Determinism:** 6 scenes × 3 seeds, spread 0 (F1 values 0.665, 0, 1, 0, 1, 0).

**(b) dataset_plan and nb08 markdown (pre-audit)**
- Price of estimated normals: butt 0.91 → 0.59, edge 1.00 → 0.44, T 0.66 → 0.41. Normal estimation dominates runtime by 4-7×.
- L1: precision ~0.1 with 23-36 predicted seams.
- Coplanar seams appear only as gap-conditioned proxies:
  - edge survives single view at F1 0.91;
  - RMSE ≈ g/2-g;
  - butt RMSE correlates with gap at 0.55 and with thickness at 0.05;
  - first hypothesis against thickness: Spearman −0.56;
  - gaps ≤ 2.8 mm.
  - None of these is in the stored outputs.
- Phase 6b (pre-audit): curved fillets 0.25-0.36; "dies on butts".

**(c) Current numbers (nb15 and macros)**
- Full_exterior median F1:

  | T/line | circle | ellipse | saddle | rounded_rect | swept | corner | butt | grooved | arc | lap | edge |
  |---|---|---|---|---|---|---|---|---|---|---|---|
  | 0.46 | 0.27 | 0.27 | 0.31 | 0.37 | 0.42 | 0.50 | **0.01** | 0.03 | 0.00 | 0.37 | 0.34 |

- Single view: 0.45 / 0.33 / 0.28 / 0.28 / 0.37 / 0.50 / 0.04 / 0.02 / 0 / 0 / 0.34 / 0.04.
- Curved range 0.27-0.42. T/line RMSE 1.53 mm.
- L0 → L1: T 0.32 → 0.10; butt 0 → 0.03; corner 0.50 → 0.12; edge 0.34 → 0.30; lap 0.37 → 0.14. Pooled 0.31 → 0.10.
- Noise: 0.29 / 0.00 / 0.00.
- Fixture: ΔF1 −0.01 (corner −0.14, butt −0.30).
- Task 2: selection 0.90 / 0.53 / 0.49 / 0.73 / 0.52; localisation 5.8 / 0.2 / 6.8 / 3.6 / 2.3 mm.
- Cost: 0.23 / 0.10 / 1.39 s.

### 4.7 Measured limitations
- The invented 15° gate decides the angle-joint result (0.03 vs 0.60 at 30°).
- The OPP test fires on tangent-plane pairs around cylinders: low precision on curved members.
- Coplanar seams appear only as O(root-gap) proxies. On bench_phase4 butts they need exact normals (0.01 with estimated normals).
- The paper's 0.1 mm threshold leaves nothing on ~1 mm-spaced clouds.
- 25 mm sampling cannot seed a nearby sheet (re-seeding was added).
- Zero at the first noise step.
- Plate-border phantoms at L1.

### 4.8 Discrepancies (lit-ppf)
1. phase8_plan.md:536-537 says "tol-30° rung: … corner 0,00 → 0,50". The headline corner is **already 0.50** (nb15) and `\ppfTolThirtyCorner` = 0.50. Only "angle joints 0.03 → 0.60" is supported.
2. "0,25-0,36 on curved fillets" (MANUAL §9, dataset_plan:1903) is pre-audit; it is now 0.27-0.42.
3. The butt/edge success seen in nb08 (out/bench) does not hold on bench_phase4 with estimated normals (0.01 / 0.34). The ICRA paper's "rise by about 0.5" is +0.49 for butt and +0.42 for edge.
4. Docstring (lines 129-131) says "eq. 23 replaces component 1 by the signed rho". The paper writes $\rho=\mathbf n_2\cdot\mathbf d=F(p_1,p_2)_4$ and says the OPP relation comes from "the second component"; its numbering does not follow Drost's. The module uses ∠(n₁, n₂) for the gate.
5. Feature points by distance to the closed-form intersection line plus plane membership is presented in code comments as "the experiment section's rule". The paper speaks of the PPF "distance component" between point pairs on the two planes, so this is an **undeclared reading**.
6. MANUAL §8.1 says "a seam box crop"; the actual stand-in is Yi's 40 mm band.
7. Test counts: dataset_plan says 9, there are 16 now. dataset_plan:1438 calls the method randomised; this is superseded.
8. Spacing estimators differ by method: PPF uses the median NN distance on 2000 samples, ransac and lobb the mean on 20 000.

### 4.9 Tests (`tests/test_lit_ppf.py`, 16)
- Descriptor coplanarity/orthogonality readings.
- $R_z\mathbf n=\hat z$.
- Two merged planes for a fold.
- Vote > 50 on the fold; a parallel pair is refused before voting; two sheets: 0 seams, all `not_orthogonal`.
- Fold end to end: 100 ± 5 mm, corners < 4 mm off the line.
- The literal 0.1 mm threshold gives 0 seams (offset grid); the default > 0.
- A duplicated plane gives 2 seams without dedup and 1 with it (union, 100 ± 6 mm, `merged_seam` logged).
- `merge_collinear` is collinearity, not proximity.
- `intersecting` accepts a 60° fold that `orthogonal` refuses; parallel pairs refused by both; bad rule raises.
- `ortho_tol` is honoured end to end (15 → 0 seams, 35 → ≥ 1).
- The largest cluster defeats a blob 58 mm away (all > 150 mm vs 100 ± 6).
- `min_votes` fraction semantics and logged params.
- θ mod π: flipping half the normals leaves votes and pair count unchanged.
- Deterministic; registry flags.
- Exact-normals arm works; missing array raises.
- Through the harness: 2 rows, zero spread.

---

## 5. BibTeX

**ICRA `paper/icra2027/refs.bib` (lines 1-32).** All correct against the PDFs (no DOIs except pcaslice).
- `yi2026seam`: AiC 183, 106792, 2026.
- `wei2024coarse`: `journal={arXiv preprint arXiv:2408.10710}`, 2024.
- `zhang2025lobb`: RCIM 95, 102987, 2025.
- `zhang2025lobbtase`: T-ASE 22, 75--84, 2025.
- `wang2024ppf`: Sci. Rep. 14, 21137, 2024.
- `wang2026pipe`: Welding in the World 2026, note doi:10.1007/s40194-026-02509-9.
- `fang2024model`: Fang & Tian, RCIM 89, 102772, 2024.
- `li2026impeller`: Li, Dongmin; Wang, Yu; Wang, Zhengyong; "…based on normal of point clouds and {PCA}", JMST 40(3) 2039--2046, 2026.

**Thesis `manuscript/thesis.bib`, the four in-scope keys (verbatim):**
```
@article{yi2026seam,
  author  = {Yi, Jinxin and Kong, Xuan and Tang, Hao and Zhang, Jie and Chen, Zhenming and Deng, Lu},
  title   = {Weld seam extraction and path generation for robotic welding of steel structures based on {3D} vision},
  journal = {Automation in Construction},
  volume  = {183},
  pages   = {106792},
  year    = {2026}
}
% VERIFY: venue and year (not on the PDF's first page).
@misc{wei_regiongrow,
  author  = {Wei, Pengkun and Cheng, Shuo and Li, Dayou and Song, Ran and Zhang, Yipeng and Zhang, Wei},
  title   = {Coarse-to-Fine Detection of Multiple Seams for Robotic Welding},
  note    = {VERIFY venue}
}
% VERIFY: volume, pages, year (not on the PDF's first page).
@article{zhang_lobb,
  author  = {Zhang, Yuankai and Geng, Yusen and Tian, Xincheng and Sun, Yujie and Xu, Xiaolong},
  title   = {A novel weld seam extraction method with semantic segmentation and point cloud feature for irregular structure workpieces},
  journal = {Robotics and Computer-Integrated Manufacturing},
  year    = {VERIFY}
}
@article{wang2024ppfseam,
  author  = {Wang, Shengbo and Li, Zengxu and Chen, Guodong and Yue, Yaobin},
  title   = {Weld seam object detection system based on the fusion of {2D} images and {3D} point clouds using interpretable neural networks},
  journal = {Scientific Reports},
  volume  = {14},
  pages   = {21137},
  year    = {2024},
  doi     = {10.1038/s41598-024-71989-w}
}
```

**ch4 citation check** (`chapters/ch4_methodology_weldset.tex`:143-146). The order is RANSAC, region growing, LOBB, PPF, PCA slicing, CAD registration, quadric. **Every key points at the right paper**; the problems are in the fields:

| key | status | fix |
|---|---|---|
| `yi2026seam` | OK | DOI 10.1016/j.autcon.2026.106792 could be added |
| `wei_regiongrow` | **incomplete** | arXiv:2408.10710 [cs.CV], 2024 (v1, 20 Aug 2024). The VERIFY comment is wrong: the arXiv stamp is on p.1 |
| `zhang_lobb` | **incomplete** | vol 95, 102987, 2025, DOI 10.1016/j.rcim.2025.102987. The comment is wrong: the header on p.1 prints "95 (2025) 102987". **Also add** the T-ASE descriptor (ICRA key `zhang2025lobbtase`); lit-lobb implements both papers |
| `wang2024ppfseam` | OK | — |
| `wang2026pipes` | OK | online-first PDF shows only the DOI; received 10 Mar 2026, accepted 26 May 2026 |
| `fang2024modelreg` | **wrong** | "and others" should go: the PDF lists exactly Weihua Fang and Xincheng Tian. DOI 10.1016/j.rcim.2024.102772 |
| `li2026quadric` | **wrong** | the title is "…based on **normal** of point clouds **and PCA**"; pages 2039--2046. Full names Yu Wang, Zhengyong Wang are on the PDF's citation box; the DOI in the entry is right |

Side check: thesis ch2:33 says Yi et al. used "800 samples / 4 workpieces … on request". This matches the paper (200 samples per workpiece; "Data will be made available on request").

---

## 6. Cross-document items to fix before quoting
- **MANUAL §8.1:**
  - It says "notebooks/04…10"; lit-quadric's notebook is 11.
  - The audit rungs for ransac, regiongrow, lobb and ppf are not listed (only the ppf exact-normals and modelreg arms).
- **MANUAL §9** is dated "as of 2026-09-08" and is entirely pre-audit.
- **README audit paragraph** (531-545) is consistent with phase8_plan and nb15. The paragraphs after it are explicitly "first measurements … before the audit".
- **Outside scope but noticed:**
  - The ICRA text says "every method runs in well under 4 s per scene", but nb15's median L0 cost for lit-modelreg is 5.01 s, and the L1 medians reach 10.42 s (lit-lobb).
  - nb15's markdown says 268 320 rows while its stored output says 271,200.
