# WeldSet (`weld_generator`) material map: evaluation code, Phase 4 batch, results notebooks, `out/`, tests, ICRA numbers

Audit date: 2026-10-20. Everything was strictly read-only. I created, modified and deleted no files and executed no notebook. All Python ran with `-B`, read data and printed it. Paths below are relative to `/workspaces/welding_cell_ws/weld_generator/`.

## 0. Provenance legend and the five facts that matter most

**Tags used for every number in this report:**

- **[D-final: nb15] / [D-final: nb13]**: a desktop run, stored as committed notebook output. Both notebooks were executed 2026-09-15 between 11:05:08 and 11:05:21 UTC on the final batch: 271 200 rows, 67 chunks, 760 scenes.
- **[D-final: paper]**: a desktop run, committed in `paper/icra2027/tables/*.tex`, `numbers.tex` or the vector figure PDFs. These were built by `make_figures.py` from the same 271 200-row batch: figure PDFs at 2026-09-15 11:05:23–25 UTC, last table/macro commit 38e8c21 at 11:14 UTC.
- **[L-era: nbXX]**: notebook output executed on this laptop before the 2026-09-09 hand-off, on older corpora (`out/bench`, `out/bench_fx`). It is not Phase 4 batch data.
- **[L-stale]**: computed by me now from this laptop's `out/`, which is pre-visfix and pre-audit. Use it only for provenance, never in the thesis.
  - Exception: plate strata are byte-identical to the desktop (`notes/phase8_plan.md` §1.2.3: "every plate stratum byte-identical (60/60 hashes)"; `notes/dataset_plan.md` 2026-10-19: the 274 plate twins are byte-identical). So plate-only structural counts carry over.
- **[A]**: the `out/annotation/` files. They were created on this laptop 2026-09-01 and copied to the desktop per `notes/handoff_phase8.md`. Every annotation macro in the paper equals what these files give.
- **[doc: file, date]**: a statement in a document, not a computation.

**Fact 1. The final batch is 271 200 rows, not 268 320.**
- Evidence: nb13 cell 1 and nb15 cell 1 print 271,200 rows; nb15 adds 760 scenes and 67 chunks; the paper sets `\nRows{271,200}` [D-final].
- Reconstructing the chunk list from code with all 720 scenes gives exactly 271 200.
- "268 320" appears in `README.MD` l.16 and l.26, `notes/handoff_phase8.md` §0, `notes/phase8_plan.md` l.530, `notes/dataset_plan.md` l.2153 and nb15 cell 0 / `build_15_phase4_results.py` l.11. It is the count when the audit re-run "landed 2026-09-15 00:00 UTC" [doc: phase8_plan].
- History of `\nRows` in git:
  - 96,120 (625011d, 09-12, curved strata retired)
  - 141,480 (d6cb08d, 09-13)
  - 184,680 (295cdae, 09-14: post-visfix, pre-audit, M6 plate-only)
  - 269,760 (30cf17b, 09-15 09:09)
  - 271,200 (38e8c21, 09-15 11:14)
- Earlier laptop history [doc: memory phase4-run / dataset_plan]: 169 800 rows (2026-09-05, 38 chunks) → 173 000 → 184 680 (2026-09-09).

**Fact 2. This laptop's `out/` is stale.**
- `out/bench_phase4` is the pre-visfix corpus.
- `out/phase4_batch/phase4_batch.csv.gz` has 184 680 rows in 53 chunks and is dated 2026-09-09 06:50. It is pre-audit, has no audit rungs, and modelreg ran on plates only.
- There is no `train_v1` here.
- Caution: 184 680 rows also existed on the desktop on 2026-09-14 (commit 295cdae, different content). A row count alone does not identify the laptop file.

**Fact 3. Which notebooks show final data.** Only nb13 and nb15 hold desktop-final outputs. nb07, nb12 and nb14 are laptop-era and unrelated to the batch.

**Fact 4. The paper misstates the densification step.** It says predictions and truth are "densified to 0.5 mm spacing". The code uses **1.0 mm** for F1, Chamfer and the lateral statistics, and **0.25 mm** for matching and RMSE.

**Fact 5. The "NaN coverage figure" is only the stale PNG copies.** At HEAD (5cba849), `figures/*.pdf` and `main.pdf` are complete. Section 6.4 has the details.

---

## 1. Evaluation code

### 1.1 `scripts/baselines/__init__.py` (97 lines)

- Re-exports everything (l.48–97).
- Docstring (l.1–46): all lengths are mm. It lists the methods and their L0 stages, and states that `ours` is not a contribution.

### 1.2 `scripts/baselines/dataset.py` (254 lines)

**Curves and loading**
- `CURVES = ("nominal","root","gapmid")` (l.25).
- `load_scene` (l.28–37) loads arrays keyed `"<npz>:<key>"`.
- `scene_dirs` (l.40–52) prefers `index.jsonl` and keeps rows with `emitted`.
- `balanced_corpus(root="out/bench", per_type=50)` (l.55–96) takes the lowest-N seeds per class and raises if any class is short.

**`cloud_for(scene, arrays, view="full", noisy=False)` (l.109–157): what a method sees**
- `view="single"` → `visible_from_cam` (combined with sensor `valid` when noisy).
- `view="full_exterior"` → the `exterior` flag (KeyError if absent).
- `view="full"` → everything; "truth only".
- Returns `xyz`, `normals`, `object_id`, `valid`.

**`ground_truth(scene, arrays, curve="nominal", primary_only=True, min_visible_fraction=None, include_rejected=False, with_underside=False, with_meta=False)` (l.160–209)**
- Drops non-weldable seams (l.189–193).
- With `primary_only`, keeps only `matches_joint_type` (l.194–195).
- Returns the stored nominal array, or the derived `_root` / `_gapmid` arrays.
- `meta = {id, closed, underside, seam_class}` (l.202–204).

**`scene_facts` (l.212–254)**
- `t_min_mm` and `t_max_mm` are member gauges via `weldgen.tacks._member_gauge_mm`.
- Also provides `seam_family`, `prep`, `primitives`, `iso_17659_term`, `contact_mode`, sensor, camera, `twin_key`, `n_primary`, `mean_occluded_fraction`, `best_visible_fraction`.

### 1.3 `scripts/baselines/metrics.py` (408 lines): exact definitions

Notation:
- $\mathcal G=\{G_k\}$ are the truth polylines: nominal curve, primary seams. In single view the D31 underside seams are dropped (§1.4).
- $\mathcal P=\{P_m\}$ are the predicted polylines.

**Densification** (`_sample_polyline`, l.76–85; `densify`, l.338–351, default `step_mm=1.0`). For a polyline $Q=(q_0,\dots,q_n)$:
$$S_h(Q)=\bigcup_{i=0}^{n-1}\Big\{\,q_i+\tfrac{j}{m_i-1}(q_{i+1}-q_i)\Big\}_{j=0}^{m_i-1},\qquad m_i=\max\!\big(2,\ \mathrm{round}(\|q_{i+1}-q_i\|/h)+1\big).$$
- The result is a multiset: interior vertices appear twice.
- A segment shorter than $h$ still yields its two endpoints. So the stored truth (0.1 mm vertex spacing) is effectively sampled at its native ~0.1 mm, not at 1 mm. Predictions with dense vertices are likewise not resampled uniformly by arclength (the fake-oracle docstring at harness.py l.345–352 says this explicitly).
- A raw `(N,3)` ndarray passes through `densify` unchanged (l.346–348).
- Definitions: $\tilde P=\bigcup_m S_1(P_m)$, $\tilde G=\bigcup_k S_1(G_k)$.

**Distance to truth** (l.48–73). Exact point-to-segment distance with a clamped projection, minimised over every segment of every truth polyline:
$$d(x,\mathcal G)=\min_{k,i}\big\|x-\big(a+t^\*(b-a)\big)\big\|,\qquad t^\*=\mathrm{clip}\!\Big(\tfrac{(x-a)\cdot(b-a)}{\|b-a\|^2},0,1\Big).$$

**Precision, recall, F1** (`precision_recall_f1(pred, gt, tol_mm, step_mm=1.0)`, l.113–130). Called from `evaluate(..., tol_mm=3.0, step_mm=1.0)` (l.381–382), and `run_matrix` passes `tol_mm=3.0` (harness.py l.419):
$$\mathrm{prec}=\frac{1}{|\tilde P|}\sum_{x\in\tilde P}\mathbf 1\!\left[d(x,\mathcal G)\le\tau\right],\qquad \mathrm{rec}=\frac{1}{|\tilde G|}\sum_{y\in\tilde G}\mathbf 1\!\left[\min_{x\in\tilde P}\|y-x\|\le\tau\right],\qquad F_1=\frac{2\,\mathrm{prec}\cdot\mathrm{rec}}{\mathrm{prec}+\mathrm{rec}}$$
- $\tau=3$ mm.
- F1 = 0 if prec + rec = 0. All three are 0 if $\tilde P$ is empty.
- **A predicted point is a TP** if its exact 3-D distance to any primary truth segment is ≤ 3 mm. There is no per-seam assignment. The tolerance is on full 3-D distance, so a point up to 3 mm past a seam end still counts; "lateral tolerance" in nb15 is loose wording.
- **Recall** is the fraction of densified truth samples whose nearest densified predicted point (Euclidean, cKDTree, `_nearest` l.29–41) lies within 3 mm.

**Symmetric Chamfer** (`chamfer_mm(pred, gt, step_mm=1.0)`, l.92–110). Both directions are kept as separate columns (`pred_to_gt`, `gt_to_pred`) and summed (not averaged) into `chamfer`; all three are NaN if empty:
$$\mathrm{CD}_{P\to G}=\tfrac{1}{|\tilde P|}\textstyle\sum_{x\in\tilde P}d(x,\mathcal G),\qquad \mathrm{CD}_{G\to P}=\tfrac{1}{|\tilde G|}\textstyle\sum_{y\in\tilde G}\min_{x}\|y-x\|,\qquad \mathrm{chamfer}=\mathrm{CD}_{P\to G}+\mathrm{CD}_{G\to P}.$$

**Lateral error distribution** (`lateral_error_mm`, l.133–144).
- Computed over $D=\{d(x,\mathcal G)\}_{x\in\tilde P}$: `lat_mean`, `lat_median`, `lat_p95` (`np.percentile(…,95)`, linear interpolation) and `lat_max`.
- It is not decomposed: "lateral" here is the full point-to-polyline distance.
- `band_width_mm = 2·median(D)` (l.147–155).

**The literature's matched-path RMSE/ME, with lateral/endpoint decomposition** (`path_error_mm(pred, gt, step_mm=0.25)`, l.218–245). With $Q=S_{0.25}(p)$ and $e_j=d(q_j,g)$:
- $\mathrm{rmse}=\sqrt{\tfrac1{|Q|}\sum e_j^2}$; `me` $=\max_j e_j$ (maximum error, as in Yi et al.); `mean` $=\overline{e}$.
- Lateral: $\ell_j=\|(q_j-a)\times\hat u\|$, the distance to the **infinite supporting line** of the truth segment nearest to $q_j$ by clamped distance, with $\hat u=(b-a)/\|b-a\|$ (`_supporting_line_distance`, l.183–209). This gives `lateral_rmse` and `lateral_me`. Pointwise $\ell_j\le e_j$; they differ only where samples project past the truth ends.
- Along-seam components: `length_error_mm` $=L(p)-L(g)$ (`polyline_length_mm`, l.212–215), and
  $$\mathrm{end\_error}=\tfrac12\min\big(\|p_0-g_0\|+\|p_e-g_e\|,\ \|p_0-g_e\|+\|p_e-g_0\|\big).$$
  Also returned: `n_samples`. Everything is NaN if either side has fewer than 2 points.

**Matching** (`match_seams(pred, gt, step_mm=0.25)`, l.248–271).
- Cost: $c(k,m)=\frac{1}{|S_{0.25}(P_m)|}\sum_{q}d(q,G_k)$.
- Pairs are sorted ascending and accepted greedily, one-to-one. Unmatched truth gets `(k, None)`, which is a miss.
- **There is no distance cap**: any surviving pairing counts.
- `matched_path_errors` (l.274–290) produces one row per truth seam.

**Per-row aggregates in the batch** (`harness._score`, l.400–414). Over matched seams only:
- `rmse_med` = median, `rmse_max` = max, `me_med`, `me_max`.
- `seam_recall` = #matched / #scored truth.
- `lateral_*`, `length_error_mm` and `end_error_mm` are not stored in `run_matrix` rows. They appear only in Task 2 rows and in the annotation scorer.

**EMD** (`emd_mm(pred, gt, n=256, step_mm=1.0, seed=0)`, l.293–330).
- $P'=S_1(P)$, $G'=S_1(G)$.
- Each side is subsampled without replacement to ≤256 points (`default_rng(0)`). The larger side is then trimmed at random to $m=\min(|P'|,|G'|)$.
- Hungarian assignment (`linear_sum_assignment`) on the Euclidean `cdist`.
- $\mathrm{EMD}=\frac1m\sum\|P'_r-G'_{c(r)}\|$: equal mass $1/m$ per point, i.e. W1 between uniform empirical measures, in mm, with no length normalisation. NaN if either side is empty.
- **EMD is never computed in the batch, nb15 or the paper.** It is used only in nb07 and `tests/test_harness.py`.

**Other columns.** `seam_count_error = n_pred − n_gt` (l.333–335). `evaluate()` (l.381–408) also returns `n_pred_seams`, `n_gt_seams` (the scored truth count) and `n_pred_points`. `evaluate_band` (l.354–378) is used for `ours` only.

### 1.4 `scripts/baselines/harness.py` (546 lines)

**`PreparedScene` (l.83–156)**
- Clouds and oracles are built lazily and cached.
- `gt_for_scoring` (l.106–111): in **single view, D31 underside seams (lap lower toes) are dropped from scoring**, unless all seams are underside. Detection inputs and oracles always see the full truth.
- Noise (l.113–132): `noise_scale = k` multiplies the profile's `subpixel_px` and `lateral_sigma_px` by k before `weldgen.noise.apply`. The realisation is deterministic in `noise_model.seed`. Sensor dropout (`valid`) is applied.
- Oracles (l.134–156):
  - `"band"`: `seam_region_oracle(xyz, gt, end_margin_mm=0.0)`, half-width 20 mm, i.e. a 40 mm band (`lit_ransac.py` l.145, l.152–216, KD-tree on the truth resampled at max(hw/40, 0.5) mm).
  - `"surfaces"`: face-id surface labels plus a surface-intersection crop; labels are transferred by nearest neighbour for single-view or noisy subsets.
  - `"objects"`: `object_id`.

**`prepare(dirs, primary_only=True)` (l.159–170)** skips scenes without truth.

**Runners and their L0 inputs**
- `ours` (l.195–204): ρ = 0.5, R from the validity window. Excluded from the batch.
- `lit-ransac` (l.207–212): prefilter 1.0 pt/mm², band mask, seeds vary.
- `lit-regiongrow` (l.215–236): voxel 1.5 mm, edge radius 6 mm, link 3 mm. L0 is `keypoint_crop`: target fraction 10 356/81 730 = 0.1267, radius bisected within [5, 60] mm (`lit_regiongrow.py` l.157, l.161). Surface labels are also supplied. `crop="surfaces"` is the old rung.
- `lit-ppf` (l.239–247): band mask; `normals="estimate"` by default, `"exact"` is a rung.
- `lit-lobb` (l.250–256): `object_id`, voxel 1.0 mm.
- `lit-pcaslice` (l.275–289): one band per truth seam, plus the per-seam `closed` flag.
- `lit-modelreg` (l.292–305): model from `scene.json` and truth. Mode `nonrigid`, `target_features="oracle"`, `init="near"`. It ignores the oracle flag.
- `lit-quadric` (l.308–327): L0 is surface labels plus `part_labels=object_id`.

**`fake_oracle(sigma_mm=0.3, p_phantom=0.0, p_miss=0.0)` (l.330–360)**
- Truth is coarsened to 4 mm vertex spacing by arclength (`_coarsen`, l.264–272), then isotropic N(0, σ) jitter is added.
- A phantom is a 60 mm segment along +x, placed 40 mm above a uniform point in the truth bounding box.
- A miss drops a seam.

**REGISTRY (l.363–390)**
- `lit-ransac` and `fake-oracle` are randomised. All others are deterministic: `lit-ppf`, `lit-pcaslice` (seeded MSAC affects poses only), `lit-modelreg` and `lit-quadric` (seeded walk start).
- `ours` has `output="band"`.

**`run_matrix(prepared, methods=None, seeds=range(30), verify_seeds=3, oracle=True, view="full_exterior", noise_scale=0.0, tol_mm=3.0, method_kw=None)` (l.417–455)**
- Default methods are ours, ransac, regiongrow, lobb.
- Randomised methods get every seed. Deterministic methods get `seeds[:verify_seeds]`.
- Row keys: `method, scene_id, joint_type, seed, arm (L0/L1), view, noise_scale, randomised, n_gt` (all primary), `t_min_mm, root_gap_mm, sec`, plus aux columns and the score.
- **`sec` caveat.** `t0` is taken before `spec.run`. So `sec` includes the lazily built cloud and cached oracle on the first seed. It also includes any per-call oracle work inside a runner on every seed: the keypoint crop, pcaslice's per-instance masks, modelreg's model build. Scoring is excluded. nb15's "method call only — not oracle preparation" is therefore not exact. For 2-seed methods the median of two runs is their mean.

**`run_task2(prepared, methods=None, seeds=range(30), verify_seeds=2, noise_scale=0.0, tol_mm=3.0, method_kw=None)` (l.458–533).** Section 1.5 covers it. `tol_mm` is accepted but never used.

**`spread` (l.536–546)** gives min, median, max, std and count, with `spread = max − min`.

### 1.5 Task 2 scoring

**The label** (`weldgen/mps.py`)
- `mps_rule-0.1` (l.52–77): the argmax over **weldable** seams with visible arclength `visible_fraction·length_mm > 10 mm` (`MIN_LEN_MM`, l.43) of the key (visible arclength, fold = |180° − dihedral| using the curved mean, −id). Null below 10 mm.
- `mps_margin` (l.80–96): runner-up over winner visible length; 0 if only one seam clears, None if none.

**The run**
- `prepare([d], primary_only=False)` is called for task2 chunks (`run_phase4_batch.py` l.189). The truth is therefore every weldable seam, including off-class and underside seams.
- Consequence: **the L0 oracles of Task 2 are also built around all weldable seams.** That includes pcaslice's per-instance masks and modelreg's transferred seams, unlike the coverage chunks.
- The method always runs at L0 on `"single"` (l.514).
- Predictions are matched one-to-one (uncapped greedy) against all weldable truth. `mps_matched` records whether the MPS index received a partner.
- If matched, `path_error_mm(pred[π], G_mps)` is stored (rmse, me, mean, lateral_rmse, lateral_me, length_error_mm, end_error_mm). **`end_error_mm` is NaN when the MPS seam is closed (D39).**
- `mps_null` rows carry NaN. Band methods are refused (l.485–487).

**Rotation invariance (D39)**
- Every reported quantity is a sample-to-truth distance, length or end distance, so the start point on a ring is irrelevant, and the end error is NaN on rings.
- `test_task2` checks that rmse, me and lateral_rmse change by less than 0.05 mm when a truth ring is rolled by n/3 and scored against itself.
- Minor caveat: rings are stored without a duplicated closing vertex, so the omitted closing chord moves with the start index. This is negligible at 0.1 mm spacing.

**Aggregation** (nb15 cell 27 = `make_figures.py` l.211–218)
- Drop `mps_null` rows.
- Selection rate(method, class) = mean over scenes of [mean over seeds of 1(mps_matched)].
- Localization = median over scenes of [median over matched seeds of rmse].
- The paper's `\taskTwoQuadricSel` and `\taskTwoLobbSel` are the unweighted mean of the five class rates.

**Caveat for the thesis.** "Selection" is not a choice the method makes, and it has no distance gate. Any method that returns at least |W| polylines matches every weldable seam. pcaslice and modelreg score 1.00 by construction, which nb15 and the paper acknowledge. The annotation scorer, by contrast, gates matches at 15 mm.

### 1.6 Fake-oracle validation (`tests/test_harness.py`, on 2 `out/bench` scenes: T[:1] + corner[:1]; skipped if the corpus is absent)

- **Jitter (l.38–56).** At σ = 0.3: `f1 > 0.95`, `seam_recall == 1`, and `rmse_med ∈ [0.2, 0.55]`. The scaling ratio between σ = 1.0 and σ = 0.3 must lie in (2.5, 4.0); the ideal is 3.33.
  - The docstring cites the Rayleigh mean σ√(π/2) = 0.376 mm.
  - My derivation (not in the repo): with 4 mm vertices linearly interpolated, the expected RMSE is about √(4/3)·σ ≈ 1.15σ (0.35 mm), and the mean distance is about 1.02σ. "RMSE ≈ σ" is only approximately right, and the test pins a band rather than equality.
- **Phantom (l.59–78).** `n_pred = n_gt + 1`, `seam_count_error == 1`, precision is strictly lower in every row and lower than clean by more than 0.05 on the mean. `seam_recall == 1` and the median rmse is unchanged within 0.05 mm, so the phantom never steals a match.
- **Miss (l.81–86).** `p_miss = 1` gives `seam_recall == 0` and `f1 == 0`, with no crash. At the `matched_path_errors` level a miss is a `matched=False` row with NaN values.
- **Also pinned.**
  - The registry flags.
  - `ours` shows zero spread over 3 verify seeds while the fake oracle varies.
  - Noise scale 2 doubles every point displacement exactly (median |ratio − 2| < 1e-6, with more than 99% of points displaced).
  - EMD: a full seam offset 0.3 mm gives 0.3 ± 0.1; a 10% stub gives more than 30 mm; empty input gives NaN.

### 1.7 `scripts/apply_rule_blocks.py` (102 lines)

- Writes `scene["mps"] = mps_rule(scene)` and `scene["tacks"] = tack_rule(scene, arrays)` into every emitted `scene.json` (l.52–55), in place.
- Rewrites `scene.sha256` and the index `content_hash` (l.56–58). Identity (`scene_id`, `twin_key`) is unchanged.
- Skips scenes that already carry both blocks unless `--force` (l.49–51).
- Records stats: scenes, updated, skipped, tacks_total, single_tack_seams, closed_seams_tacked, mps_null.
- Writes `manifest["rule_blocks"]` (l.79–86).

### 1.8 Annotation (Phase 5)

**`export_for_annotation.py`**
- Defaults (l.109–118): `--out out/annotation/export`, `--manifest out/annotation/manifest.json`, `--per-type 4`, `--seed 0`, `--corpus out/bench`. `--with-joint-types` is `store_true` with `default=True`, so it cannot be switched off; `--unbriefed` controls the omission.
- Picks per class with `rng.choice` from `balanced_corpus(...,50)`, then shuffles the order (l.123–130).
- Writes ASCII PLYs of the `full_exterior` cloud as `scene_NN` (`%.3f` mm, l.142–144, l.160–168), plus `BRIEFING.md`. The manifest is kept outside the export.
- The actual export used `out/bench6a`: the manifest paths are `out/bench6a/...`.

**`score_annotations.py`**
- Parses CloudCompare lists using the last three numbers per line, and refuses lists with more than 2000 points (whole-cloud exports).
- `score_annotator(match_tol_mm=15.0)` (l.106–156). For each D19 curve, matched seams are kept only if `rmse ≤ 15 mm` (l.121–123, applied as `(matched and rmse ≤ tol) or unmatched`).
- `missed = n_gt − #matched` and `extra = n_ann − #matched` (l.136–137).
- Per scene: `lat_rmse`, `lat_me` and `end_err` are medians over matched seams of `lateral_rmse`, `lateral_me` and `end_error_mm` (nominal curve). `rmse_{nominal,root,gapmid}` are medians of the full RMSE.
- Headline (l.183–199): the `demo` role is excluded; reports per-annotator median and p95 of `lat_rmse`.
- `annotator_model.json` (l.202–216).
- Note: Phase 5 "lateral RMSE" is the supporting-line RMSE, a different quantity from the methods' point-to-segment `rmse`.

### 1.9 `scripts/dataset_viewers/`

- **README**:
  - `bench_phase4/`: 720 scenes, 12 strata × 60, no fixture.
  - `bench_phase4_fx/`: 634 twins, all 12 strata — 274 plate twins (schema 2.3.0) and 360 curved twins (2.4.0, `contact_mode: line`).
  - Also documents file contents, controls and requirements (numpy + matplotlib; open3d optional).
- **`view_bench_phase4.py`**: a matplotlib browser for the 720 scenes. Shows the cloud under exterior, camera-view or all-samples conditions; truth seams (primary green, other weldable amber, rejected dotted grey); MPS; tacks; the depth image with projected seams; and a parameter/seam table. Options `--stratum`, `--start`, `--points`, `--list`, `--save-all`.
- **`view_bench_phase4_fx.py`**: shows each twin pair side by side (original vs fixture twin) with a seam-by-seam comparison. It lists kept/lost twins per stratum and the loss reason: `NoVisibleSeams` = fixture hides every seam; `NoSeamsFound` = contact closes the joint.

---

## 2. The batch (`scripts/run_phase4_batch.py`, 373 lines)

### 2.1 Configuration and defaults

- `METHODS_ALL` = 7 literature methods (l.75–76). `ours` is excluded by the 2026-09-03 ruling.
- The comment at l.72–74 mentions `--include-ours`, but **no such flag exists**.
- `_kw` (l.90–93): lit-quadric runs `ordering="chain", output="points"` in every standard chunk.
- CLI defaults (l.227–248): `--out out/phase4_batch`, `--per-type 0` (all scenes), `--seeds 30` (ransac), `--only`, `--list`, `--append-missing`, `--retire`, `--corpus out/bench_phase4`.
- Deterministic methods get 2 seeds (0 and 1): `stream()` passes `seeds=range(spec["seeds"])` and `verify_seeds=spec["seeds"]` (l.192–201).

### 2.2 Chunk list (`chunks()`, l.101–181): 67 chunks, each with one method, every chunk over all 720 scenes except fixture

| group | chunks | view / arm / noise | seeds | kwargs | rows (final, 720 scenes) |
|---|---|---|---|---|---|
| coverage (14) | `coverage_{m}_{full_exterior,single}`, 7 methods | both views, L0, 0 | ransac 30, others 2 | quadric chain + points | 21 600 (ransac) / 1 440 each → 60 480 |
| l1 (6) | `l1_{m}_full_exterior`, m ≠ modelreg | full_exterior, L1, 0 | same | quadric `_kw` | 28 800 |
| noise (14) | `noise{1,2}_{m}_single` | single, L0, σ×1 / σ×2 | same | quadric `_kw` | 60 480 |
| ladder (19) | see below | full_exterior unless named `_single`, L0, 0 | 2; ransac rungs 30 | per rung | 87 840 |
| task2 (7) | `task2_{m}_single` | single, L0, 0, `primary_only=False` | ransac 30 / 2 | quadric `_kw` | 30 240 |
| fixture (7) | `fixture_{m}` | full_exterior, L0, 0 | ransac 30 / 2 | quadric `_kw` | 40 pairs × 2 arms → 2 400 / 160 → 3 360 |

The 19 ladder chunks (l.123–166):
- Five that predate the audit:
  - `ppf_exact_normals_full_exterior` (`normals=exact`)
  - `modelreg_dense_features` (`target_features=dense`)
  - `modelreg_global_init` (`init=global`)
  - `quadric_distance_{fe,single}` (`ordering=distance`)
- Fourteen 2026-09-14 audit rungs:
  - `quadric_projected_{fe,single}` (`output=projected`)
  - `pcaslice_whole_ring_{fe,single}` (`ring_mode=whole`)
  - `lobb_flat_3mm_full_exterior` (`kmeans=flat, edge_radius_mm=3`)
  - `regiongrow_surface_crop_full_exterior` (`crop=surfaces`)
  - `ppf_{perpair (dedup=False, keep_clusters=all), intersecting (pair_rule), tol30, tol45 (ortho_tol_deg)}_full_exterior`
  - `ransac_{intersecting (pair_rule), thresh3x (dist_thresh_rule=3x_spacing), outlierfilter (outlier_filter=True)}_full_exterior`
  - `modelreg_edge_features` (`target_features=edges`)

Totals: 60 480 + 28 800 + 60 480 + 87 840 + 30 240 + 3 360 = **271 200**. This matches nb15's printed per-prefix breakdown exactly [D-final: nb15].

**Fixture pairing** (`fixture_pairs`, l.210–222). It takes the first 8 twin-bearing scenes per joint type, in corpus index order (`per_type=min(8, args.per_type or 8)`, l.327), giving 40 pairs from the 274 plate twins.
- T pairs are therefore T/line only.
- The batch docstring's "~50 extra scenes per arm" (l.42–45) is wrong: the real figure is 40.
- The 360 curved twins added 2026-10-19 were **never run**. The fixture table "is the plate study as submitted" [doc: dataset_plan 2026-10-19].

### 2.3 Output columns

**Every row**
- `run_matrix` keys: `method, scene_id, joint_type, seed, arm, view, noise_scale, randomised, n_gt, t_min_mm, root_gap_mm, sec`.
- Score columns: `n_pred_seams, n_gt_seams, seam_count_error, n_pred_points, pred_to_gt, gt_to_pred, chamfer, precision, recall, f1, lat_mean, lat_median, lat_p95, lat_max, band_width_mm, rmse_med, rmse_max, me_med, me_max, seam_recall`.
- Added by the batch: `fixture, chunk, condition` (= view).

**Method aux columns**
- ransac: `n_planes`; in rung chunks also `pair_rule`, `dist_thresh_rule`, `outlier_filter`.
- regiongrow: `n_regions`; at L0 also `crop`, `crop_radius_mm`, `crop_fraction`.
- lobb: `n_crease`, `kmeans`, `edge_radius_mm` (the effective radius).
- ppf: `n_planes`, `normals`, `pair_rule`, `ortho_tol_deg`, `dedup`, `keep_clusters`.
- pcaslice: `n_instances`, `ring_mode`, `n_closed`.
- modelreg: `reg_scale`, `mode`, `target_features`.
- quadric: `n_surfaces`, `ordering`, `output`, `flat_test`, `n_curved`, `pairs_seam`.

**Task 2 rows** (no aux): `scene_id, joint_type, seam_family, view, noise_scale, mps_null, mps_seam_id, mps_class, mps_margin, mps_closed, method, seed, n_pred_seams, sec, mps_matched, rmse, me, mean, lateral_rmse, lateral_me, length_error_mm, end_error_mm`.

The laptop's stale file has 61 of these columns; the audit-rung aux columns are missing [L-stale]. nb13's DtypeWarning confirms the final file contains `kmeans, ring_mode, pair_rule, dedup, keep_clusters, output, flat_test, crop, outlier_filter, dist_thresh_rule` [D-final: nb13].

### 2.4 Resumability and repair modes

- **Resumable.** There is one `<name>.csv.gz` per chunk; an existing file is skipped (l.312–314). The concat `phase4_batch.csv.gz` is rewritten at the end of every run (`_concat`, l.361–369, which excludes `retired_rows_*`). Scenes stream one at a time and are freed with `gc` (l.184–207).
- **`--append-missing`** (l.315–352). For every existing non-fixture chunk, it runs only the corpus scenes with no rows and appends them. Existing fixture chunks are left alone (delete them to redo).
  - Side effect: a scene that yields no truth never gets rows, so it is retried on every pass.
- **`--retire JSON`** (l.290–308). Drops the listed `scene_ids` from every chunk file in place, archives them to `retired_rows_<stem>.csv.gz`, rewrites the concat and exits.
- **`--list`** (l.268–284). Estimate per chunk = n × reps × `EST_SEC[method]` (l.97–98: ransac 0.3, regiongrow 1.0, lobb 2.5, ppf 0.6, pcaslice 0.5, modelreg 2.5, quadric 1.0 s per run), × 2 for fixture.
  - It uses n = 300 plate scenes for any modelreg chunk. That is stale: modelreg has run on all 720 since 2026-09-14.
  - It uses n = 40 for fixture.
  - Replicated by me from code and `facts.csv` primitives (plate count 300 on both machines): total **≈38.1 h**. By group: coverage 8.9, l1 4.0, noise 8.9, ladder 11.3, task2 4.5, fixture 0.6 h. With modelreg on 720 scenes it would be about 42.8 h. It ignores scoring time, which was what dominated in practice.

### 2.5 Runtime: what the docs say

- [doc: MANUAL §8.3] "~64 h wall-clock on a 16-core laptop across three parallel runners; the L1 arm dominated (RANSAC over a full cloud with no band is 20× its oracle-assisted cost)."
- [doc: dataset_plan l.1898–1900, 2026-09-05; memory phase4-run] The 64 h is the **first** full batch: 169 800 rows, 720 scenes, 38 chunks, 3 runners, on this laptop. This laptop is an AMD Ryzen 7 7840HS, **8 cores / 16 threads**, with an RTX 4060 Laptop GPU (from `lscpu`/`nproc` now).
- [doc: handoff] "~70 h of compute: 38+9+6 chunk files".
- [doc: phase8_plan l.528–530] The final audit re-run was on the desktop: "51 chunks (incl. 14 new rungs) queued eight-wide (p4_queue.sh); ransac and modelreg headline chunks untouched. Landed 2026-09-15 00:00 UTC (10,5 h under a shared CPU): 268 320 rows".
- [doc: nb15 cell 37] "L1 lit-ransac chunk 15,2 h against 1,6 h for its oracle sibling in the 2026-09-12 re-run". The cost was in scoring hundreds of phantom polylines. Method time barely moved: 0.09 s vs 0.02 s [D-final: nb15/paper cost].
- So **"20×" is not supported** for RANSAC: wall-clock was about 9.5× and method time about 4.5×. The 20× figure is lit-lobb's L1/L0 method time (10.42 / 0.50).
- [doc: batch comment l.84–89, nb15] Scoring quadric's folded rings took 8.9 h for one chunk.
- Implementation note: `evaluate()` recomputes the Python-loop segment distance four times per row, and `match_seams` builds a full n_gt × n_pred cost table over 0.1 mm truth segments. That is why many-candidate outputs are expensive to score.

---

## 3. Results notebooks

### 3.1 Provenance per notebook

| nb | executed (UTC, from cell metadata) | data | verdict |
|---|---|---|---|
| 07 | 2026-08-21 18:17–18:29 | `out/bench` balanced 250 plate scenes | [L-era]; not batch data |
| 12 | 2026-08-25 11:52–12:09 | `out/bench` + `out/bench_fx`, `ours` only | [L-era] |
| 13 | 2026-09-15 11:05:18–11:05:21 | `phase4_batch.csv.gz`: **271 200 rows**, 67 chunks incl. all audit rungs | **[D-final]**, but omits lit-quadric and uses pooled means |
| 14 | no timestamps; execution counts [1,6,7,4]; file mtime 2026-09-03 | scenes generated inside the notebook from `configs/bench6a_*.yaml` and `curved_smoke.yaml` with `emit_tacks` | [L-era]; independent of `out/` |
| 15 | 2026-09-15 11:05:08–11:05:15 | **271 200 rows, 760 scenes (720 + 40 fixture twins), 67 chunks**, plus the desktop `facts.csv` | **[D-final]** |

Why nb15 is desktop data:
- Its composition counts (sensor 289/271/160, quality B 399 / C 127 / below_D 114 / D 80, T-joint 225 / angle 135) differ from this laptop's `facts.csv` (289/264/167; B 395 / C 130 / below_D 114 / D 81; 224/136) [L-stale].
- Its tables equal the paper tables.
- `build_15_phase4_results.py` reproduces all 40 nb15 cell sources byte-for-byte (verified via `ast`). Its prose is hard-coded; several prose numbers predate the audit (§7).

### 3.2 Notebook 15 results [D-final: nb15 unless stated]

**Corpus composition (cell 4).** `t_min` and `t_max` are the min and max of the thinner-member gauge `t_min_mm`; `angled` is the share of ISO 17659 angle joints (3.12).

| stratum | scenes | t_min | t_max | gap med | occl. med | angled |
|---|---|---|---|---|---|---|
| T/line | 60 | 1.09 | 11.97 | 0.81 | 0.50 | 0.60 |
| T/circle | 60 | 3.00 | 7.98 | 0.63 | 0.64 | 0.00 |
| T/ellipse | 60 | 3.06 | 7.97 | 0.50 | 0.55 | 1.00 |
| T/saddle | 60 | 3.11 | 7.21 | 0.64 | 0.52 | 0.65 |
| T/rounded_rect | 60 | 3.15 | 7.86 | 0.56 | 0.54 | 0.00 |
| T/swept_path | 60 | 4.14 | 9.77 | 0.58 | 0.50 | 0.00 |
| corner/line | 60 | 1.04 | 11.68 | 1.06 | 0.00 | 0.00 |
| butt/line | 60 | 1.16 | 11.98 | 1.10 | 0.50 | 0.00 |
| butt/line_grooved | 60 | 4.26 | 19.80 | 2.80 | 0.00 | 0.00 |
| butt/arc_butt | 60 | 3.06 | 9.92 | 1.02 | 0.00 | 0.00 |
| lap/line | 60 | 1.00 | 11.22 | 0.56 | 0.67 | 0.00 |
| edge/line | 60 | 1.00 | 2.00 | 0.28 | 0.67 | 0.00 |

Sensor profiles: d435i 289, stereo_good 271, stereo_poor 160. Quality: B 399, C 127, below_D 114, D 80. T class: T-joint (3.10) 225, angle joint (3.12) 135.

**Coverage: median F1 at 3 mm (cell 6).** Per-scene median over seeds, then median over scenes. This is identical to `coverage_full.tex` and `fig_coverage.pdf` [D-final: paper].

| stratum | ransac M1 | regiongrow M2 | lobb M3 | ppf M4 | pcaslice M5 | modelreg M6 | quadric M7 |
|---|---|---|---|---|---|---|---|
| **full_exterior** | | | | | | | |
| T/line | 0.98 | 0.58 | 0.80 | 0.46 | 0.96 | 0.99 | 0.99 |
| T/circle | 0.00 | 0.75 | 0.45 | 0.27 | 0.85 | 0.83 | 0.97 |
| T/ellipse | 0.00 | 0.74 | 0.39 | 0.27 | 0.66 | 1.00 | 0.88 |
| T/saddle | 0.00 | 0.29 | 0.32 | 0.31 | 0.50 | 1.00 | 0.91 |
| T/rounded_rect | 0.00 | 0.75 | 0.20 | 0.37 | 0.65 | 0.74 | 0.55 |
| T/swept_path | 0.77 | 0.80 | 0.80 | 0.42 | 0.92 | 0.99 | 0.98 |
| corner/line | 0.50 | 0.51 | 0.00 | 0.50 | 0.65 | 0.99 | 1.00 |
| butt/line | 0.00 | 0.51 | 0.27 | 0.01 | 0.54 | 0.98 | 0.99 |
| butt/line_grooved | 0.00 | 0.47 | 0.00 | 0.03 | 0.00 | 0.97 | 0.00 |
| butt/arc_butt | 0.00 | 0.31 | 0.49 | 0.00 | 0.01 | 0.95 | 0.68 |
| lap/line | 0.00 | 0.39 | 0.50 | 0.37 | 0.84 | 0.67 | 0.87 |
| edge/line | 0.00 | 0.72 | 0.63 | 0.34 | 0.37 | 0.36 | 0.89 |
| **single view** | | | | | | | |
| T/line | 0.67 | 0.03 | 0.16 | 0.45 | 0.02 | 0.31 | 0.66 |
| T/circle | 0.00 | 0.08 | 0.31 | 0.33 | 0.04 | 0.31 | 0.50 |
| T/ellipse | 0.10 | 0.13 | 0.40 | 0.28 | 0.01 | 0.05 | 0.59 |
| T/saddle | 0.16 | 0.19 | 0.39 | 0.28 | 0.00 | 0.16 | 0.66 |
| T/rounded_rect | 0.38 | 0.13 | 0.29 | 0.37 | 0.04 | 0.11 | 0.44 |
| T/swept_path | 0.60 | 0.20 | 0.48 | 0.50 | 0.00 | 0.49 | 0.63 |
| corner/line | 0.93 | 0.01 | 0.00 | 0.04 | 0.01 | 0.16 | 1.00 |
| butt/line | 0.00 | 0.08 | 0.00 | 0.02 | 0.63 | 0.96 | 0.60 |
| butt/line_grooved | 0.00 | 0.00 | 0.00 | 0.00 | 0.47 | 0.52 | 0.00 |
| butt/arc_butt | 0.00 | 0.44 | 0.07 | 0.00 | 0.73 | 0.63 | 0.93 |
| lap/line | 0.00 | 0.04 | 0.28 | 0.34 | 0.09 | 0.23 | 0.58 |
| edge/line | 0.00 | 0.31 | 0.25 | 0.04 | 0.24 | 0.14 | 0.43 |

**Straight-to-curved dumbbell (cell 9 = `straight_curved.tex`).** T/line median F1 → mean of the medians over the 5 curved T strata:

| ransac | regiongrow | lobb | ppf | pcaslice | modelreg | quadric |
|---|---|---|---|---|---|---|
| 0.98 → 0.15 | 0.58 → 0.67 | 0.80 → 0.43 | 0.46 → 0.33 | 0.96 → 0.72 | 0.99 → 0.91 | 0.99 → 0.86 |

**Localization (cell 11).**
- The matched-path `rmse_med` heatmap is stored only as a PNG. The printed row, T/line median RMSE in mm, is: ransac 0.75, modelreg 0.70, quadric 1.46, ppf 1.53, lobb 1.08, regiongrow 2.16, pcaslice 2.21.
- **`lat_p95` and Chamfer are never tabulated** in nb15 or the paper. The per-stratum values need a groupby on the desktop batch. The PNG heatmap cannot be read out under read-only rules.

**D19 offsets (cell 15).** Scope: primary seams of the square-prep plate strata; offset = mean over index-aligned samples of |curve − nominal|.

| class | seams | gap med | root med | root max | gap-mid med | root > 1 mm | root > 3 mm |
|---|---|---|---|---|---|---|---|
| butt | 117 | 1.10 | 1.24 | 7.21 | 0.63 | 0.58 | **0.23** |
| edge | 120 | 0.55 | 0.54 | 6.73 | 0.02 | 0.37 | 0.06 |
| fillet | 181 | 0.89 | 0.85 | 5.83 | 0.43 | 0.48 | **0.14** |
| lap_toe | 438 | 0.43 | 0.43 | 4.83 | 0.22 | 0.32 | 0.06 |

- Fillet root offset / gap by included angle: (59, 80°] **0.96**; (80, 100°] 1.00; (100, 121°] **1.25**.
- Class composition [L-stale, plates identical]: fillet = 118 T + 63 corner; edge = 60 corner + 60 edge-joint; lap_toe = 228 lap + 210 edge-joint.

**Repeatability (cell 18).**
- lit-ransac, share of full-exterior scenes with F1 spread > 0.25 over 30 seeds: T 0.30, butt 0.43, corner 0.82, edge 0.00, lap 0.45.
- The six deterministic methods have max spread 0.0.
- The paper's `\ransacBimodalShare = 5`%: share of scenes with min F1 ≤ 0 and max ≥ 0.9 [D-final: paper].

**Ladder (cell 21).** Median F1, L0 / L1, by joint type:

| method | T | butt | corner | edge | lap |
|---|---|---|---|---|---|
| lobb | 0.50 / 0.04 | 0.00 / 0.02 | 0.00 / 0.00 | 0.63 / 0.09 | 0.50 / 0.15 |
| pcaslice | 0.78 / 0.00 | 0.01 / 0.00 | 0.65 / 0.00 | 0.37 / 0.04 | 0.84 / 0.03 |
| ppf | 0.32 / 0.10 | 0.00 / 0.03 | 0.50 / 0.12 | 0.34 / 0.30 | 0.37 / 0.14 |
| quadric | 0.93 / 0.00 | 0.61 / 0.00 | 1.00 / 0.00 | 0.89 / 0.00 | 0.87 / 0.00 |
| ransac | 0.00 / 0.25 | 0.00 / 0.00 | 0.50 / 0.42 | 0.00 / 0.00 | 0.00 / 0.00 |
| regiongrow | 0.72 / 0.17 | 0.46 / 0.00 | 0.51 / 0.55 | 0.72 / 0.08 | 0.39 / 0.08 |

- Pooled median, L0 → L1 (`ladder.tex`) [D-final: paper]: M1 0.00 → 0.10; M2 0.58 → 0.11; M3 0.40 → 0.03; M4 0.31 → 0.10; M5 0.64 → 0.00; M7 0.87 → 0.00.
- Split rows from `fig_ladder.pdf` [D-final: paper], order M1 M2 M3 M4 M5 M7:

  | row | L0 | L1 |
  |---|---|---|
  | T straight | 0.98 0.58 0.80 0.46 0.96 0.99 | 0.63 0.06 0.02 0.13 0.00 0.00 |
  | T curved | 0.00 0.73 0.46 0.32 0.70 0.91 | 0.21 0.21 0.04 0.10 0.00 0.00 |
  | corner | 0.50 0.51 0.00 0.50 0.65 1.00 | 0.42 0.55 0.00 0.12 0.00 0.00 |
  | butt | 0.00 0.46 0.00 0.00 0.01 0.61 | 0.00 0.00 0.02 0.03 0.00 0.00 |
  | lap | 0.00 0.39 0.50 0.37 0.84 0.87 | 0.00 0.08 0.15 0.14 0.03 0.00 |
  | edge | 0.00 0.72 0.63 0.34 0.37 0.89 | 0.00 0.08 0.09 0.30 0.04 0.00 |

- ppf exact-normals rung, ΔF1 vs estimated normals: T +0.02, corner −0.01, butt +0.49, lap +0.06, edge +0.42.
- lit-quadric as published (distance ordering) vs chain, full_exterior:

  | | T/line | circle | ellipse | saddle | rounded_rect | swept | corner | butt | grooved | arc | lap | edge |
  |---|---|---|---|---|---|---|---|---|---|---|---|---|
  | distance | 0.99 | 0.42 | 0.39 | 0.50 | 0.31 | 0.94 | 1.00 | 0.99 | 0.00 | 0.57 | 0.89 | 0.91 |
  | chain | 0.99 | 0.97 | 0.88 | 0.91 | 0.55 | 0.98 | 1.00 | 0.99 | 0.00 | 0.68 | 0.87 | 0.89 |

- The same comparison in single view (distance / chain): 0.66/0.66, 0.45/0.50, 0.53/0.59, 0.60/0.66, 0.41/0.44, 0.59/0.63, 1.00/1.00, 0.60/0.60, 0.00/0.00, 0.93/0.93, 0.58/0.58, 0.43/0.43.
- modelreg by class (T, corner, butt, lap, edge):
  - L0: 0.99, 0.99, 0.97, 0.67, 0.36
  - dense features: 0.77, 0.87, 0.15, 0.18, 0.20
  - global init: 0.98, 0.98, 0.78, 0.57, 0.31
- **Unreported anywhere**: `modelreg_edge_features`, `quadric_projected_single`, `pcaslice_whole_ring_single` (the paper uses only the full-exterior versions of the latter two).

**Noise (cell 24).** Pooled median F1 on single view; modelreg excluded.

| | clean | 1σ | 2σ |
|---|---|---|---|
| lobb | 0.22 | 0.26 | 0.30 |
| pcaslice | 0.05 | 0.07 | 0.06 |
| ppf | 0.29 | 0.00 | 0.00 |
| quadric | 0.58 | 0.44 | 0.50 |
| ransac | 0.20 | 0.00 | 0.00 |
| regiongrow | 0.09 | 0.19 | 0.22 |

The per-profile values are only in the figures. I recovered them from the vector paths of `paper/icra2027/figures/fig_noise.pdf` (y axis 0.00 = 23.2676 pt, 1.00 = 92.5676 pt; ±0.001; the zorder-based method assignment was checked against the end labels) [D-final: paper]. Values are clean / 1σ / 2σ:

| profile | M1 ransac | M2 regiongrow | M3 lobb | M4 ppf | M5 pcaslice | M7 quadric |
|---|---|---|---|---|---|---|
| d435i | 0.174 / 0.000 / 0.000 | 0.080 / 0.198 / 0.242 | 0.203 / 0.241 / 0.317 | 0.279 / 0.000 / 0.000 | 0.068 / 0.089 / 0.080 | 0.547 / 0.403 / 0.527 |
| stereo_good | 0.234 / 0.079 / 0.012 | 0.120 / 0.192 / 0.255 | 0.322 / 0.293 / 0.354 | 0.319 / 0.041 / 0.061 | 0.028 / 0.045 / 0.043 | 0.628 / 0.495 / 0.583 |
| stereo_poor | 0.197 / 0.000 / 0.000 | 0.054 / 0.182 / 0.142 | 0.093 / 0.225 / 0.157 | 0.221 / 0.000 / 0.000 | 0.065 / 0.073 / 0.074 | 0.414 / 0.421 / 0.281 |

nb15's prose "stereo_good 0,63 → 0,50 → 0,58" matches this.

**Task 2 (cell 27 = `task2.tex`).** Selection rate per joint type, then median RMSE (mm) on the matched MPS seam. In the selection rows, "0.92 (paper 0.93)" shows how the notebook and paper print the same value (see note below the table).

| | sel T | sel corner | sel butt | sel lap | sel edge | RMSE T | RMSE corner | RMSE butt | RMSE lap | RMSE edge |
|---|---|---|---|---|---|---|---|---|---|---|
| M1 ransac | 0.81 | 0.61 | 0.43 | 0.36 | 0.08 | 0.80 | 0.00 | 2.79 | 0.88 | 1.96 |
| M2 regiongrow | 0.72 | 0.57 | 0.76 | 0.78 | 0.83 | 0.49 | 6.22 | 3.05 | 2.44 | 1.27 |
| M3 lobb | 0.79 | 0.30 | 0.29 | 0.63 | 0.80 | 0.64 | 0.60 | 0.67 | 0.71 | 1.16 |
| M4 ppf | 0.90 | 0.53 | 0.49 | 0.73 | 0.52 | 5.81 | 0.24 | 6.77 | 3.59 | 2.26 |
| M5 pcaslice | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 9.73 | 6.80 | 4.13 | 5.31 | 8.72 |
| M6 modelreg | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 4.46 | 4.65 | 1.86 | 4.29 | 6.39 |
| M7 quadric | 0.92 (paper 0.93) | 0.25 | 0.46 | 0.80 | 0.77 | 1.55 | 1.24 | 1.30 | 1.42 | 1.28 |

- The RMSE values are from `task2.tex`; nb15 prints the same numbers at 1 decimal.
- The 0.92 / 0.93 difference is rounding of the same value 0.925: numpy round-half-even gives 0.92, `%.2f` gives 0.93. Verified.
- Closed-ring rows with a finite end error: 0.

**MPS margin and class (cell 29).**
- Share of scenes with a graded margin (0 < m < 1): T/line 0.05, circle 0.20, ellipse 0.15, saddle 0.00, rounded_rect 0.10, swept 0.17, corner 0.80, butt/line 0.40, grooved 0.52, arc 0.00, lap 0.25, edge 0.33.
- MPS class by joint: T fillet 360; corner edge 21 / fillet 39; butt butt 160 / lap_toe 20; lap lap_toe 60; edge edge 33 / lap_toe 27. There are no null-MPS scenes (the counts sum to 720).

**Fixture (cell 32 = `fixture.tex`).** 40 pairs per method; paired medians:

| | ΔF1 | Δprecision | extra seams |
|---|---|---|---|
| M1 ransac | 0.00 | 0.00 | 0.0 |
| M2 regiongrow | −0.14 | −0.23 | 8.5 |
| M3 lobb | 0.00 | 0.00 | 22.0 |
| M4 ppf | −0.01 | −0.02 | 0.0 |
| M5 pcaslice | −0.20 | −0.18 | −1.0 |
| M6 modelreg | 0.02 | 0.00 | −1.0 |
| M7 quadric | −0.29 | −0.47 | 4.0 |

Median ΔF1 by joint type (ransac, regiongrow, lobb, ppf, pcaslice, modelreg, quadric):
- T: −0.09, −0.03, −0.15, 0.02, −0.01, 0.00, −0.33
- corner: −0.17, −0.05, 0.00, −0.14, −0.08, 0.01, −0.42
- butt: 0.00, −0.52, 0.00, −0.30, −0.01, 0.01, 0.00
- lap: 0.00, −0.15, 0.00, 0.00, −0.23, 0.12, −0.48
- edge: 0.00, −0.04, −0.36, −0.01, −0.30, 0.30, −0.26

Twin yield on the plates [L-stale, plates identical]:
- T 54/60, corner 36/60, butt square 40/60 (10 NoVisibleSeams + 10 NoSeamsFound), grooved 54/60 (3 + 3), lap 40/60, edge 50/60.
- So "corner worst 36/60" and "butt loses 13 to a contact" are correct.

**Covariates (cell 35).**
- Only the ISO 17659 split is printed. T/line, median F1, T-joint (3.10) vs angle joint (3.12):

  | | ransac | regiongrow | lobb | ppf | pcaslice | modelreg | quadric |
  |---|---|---|---|---|---|---|---|
  | T/line | 0.99 / 0.93 | 0.77 / 0.55 | 0.96 / 0.74 | 0.62 / 0.03 | 0.97 / 0.95 | 0.98 / 0.99 | 0.99 / 0.98 |
  | curved T families | 0.00 / 0.00 | 0.75 / 0.68 | 0.49 / 0.36 | 0.36 / 0.28 | 0.82 / 0.59 | 0.94 / 1.00 | 0.94 / 0.90 |

- T/line has 24 T-joints and 36 angle joints (angled share 0.60).
- The thickness, gap, density, elevation and occlusion bins are figure-only.
- The prose numbers ("0,38 → 0,67" density, "0,49 at 0–20°") look pre-audit: my stale recomputation gives 0.37 → 0.66 and 0.47 [L-stale].

**Cost (cell 38 = `cost.tex`).** Median `sec` per run.

| method | L0 full | L0 single | L1 full |
|---|---|---|---|
| ransac | 0.02 | 0.01 | 0.09 |
| regiongrow | 0.45 | 0.22 | 3.61 |
| lobb | 0.50 | 0.13 | 10.42 |
| ppf | 0.23 | 0.10 | 1.39 |
| pcaslice | 0.47 | 0.22 | 0.91 |
| modelreg | 5.01 | 4.59 | — |
| quadric | 1.00 | 0.33 | 2.70 |

### 3.3 Notebook 13 [D-final: nb13]

Six literature methods plus `ours` (NaN). **lit-quadric is absent.** Values are pooled row means.

- Coverage, mean F1, full_exterior, by class (T, corner, butt, lap, edge):
  - ransac 0.26, 0.54, 0.11, 0.08, 0.00
  - regiongrow 0.62, 0.49, 0.41, 0.37, 0.63
  - lobb 0.48, 0.37, 0.28, 0.43, 0.52
  - ppf 0.33, 0.44, 0.17, 0.29, 0.30
  - pcaslice 0.73, 0.55, 0.26, 0.76, 0.37
  - modelreg 0.86, 0.92, 0.81, 0.64, 0.39
- Max seed spread: ransac 1.0, the others 0.0. The cell title says "six at exactly zero" but only five are shown.
- L0 / L1 mean F1: ransac 0.207 / 0.191; regiongrow 0.540 / 0.239; lobb 0.420 / 0.112; ppf 0.292 / 0.122; pcaslice 0.569 / 0.018; modelreg 0.797 / —.
- Noise, mean F1 at σ×0 / 1 / 2: ransac 0.294 / 0.173 / 0.141; regiongrow 0.207 / 0.254 / 0.274; lobb 0.285 / 0.302 / 0.341; ppf 0.294 / 0.146 / 0.119; pcaslice 0.202 / 0.216 / 0.217; modelreg 0.373 / 0.302 / 0.301.
- Fixture, mean precision off / on (ratio): ransac 0.282 / 0.210 (1.345); regiongrow 0.481 / 0.197 (2.447); lobb 0.355 / 0.153 (2.323); ppf 0.402 / 0.230 (1.746); pcaslice 0.590 / 0.422 (1.397); modelreg 0.745 / 0.826 (0.902).
- Ladder extras, mean F1 (median rmse_med in mm): ppf estimated 0.292 (5.894); ppf exact 0.447 (5.407); modelreg faithful 0.797 (1.430); dense 0.514 (4.186); global 0.605 (2.202).
- Do not cite nb13 for thesis tables: it leaves out quadric and uses means, which breaks the nb15 reading rules.

### 3.4 Notebook 12 [L-era: nb12; `ours` only; out/bench 250 scenes]

| condition | class | n | prec | rec | F1 | Chamfer med | band width med |
|---|---|---|---|---|---|---|---|
| full_exterior | T | 50 | 0.806 | 0.919 | 0.831 | 3.140 | 3.231 |
| full_exterior | butt | 50 | 0.857 | 0.957 | 0.895 | 2.788 | 2.896 |
| full_exterior | corner | 50 | 0.910 | 0.862 | 0.874 | 2.415 | 2.649 |
| full_exterior | edge | 50 | 0.199 | 0.892 | 0.294 | 46.808 | 105.695 |
| full_exterior | lap | 50 | 0.676 | 0.957 | 0.771 | 5.620 | 3.487 |
| single | T | 50 | 0.887 | 0.443 | 0.543 | 6.427 | 2.089 |
| single | butt | 50 | 0.799 | 0.378 | 0.459 | 6.558 | 2.644 |
| single | corner | 50 | 0.874 | 0.690 | 0.735 | 2.467 | 1.846 |
| single | edge | 50 | 0.810 | 0.461 | 0.497 | 29.870 | 2.069 |
| single | lap | 50 | 0.892 | 0.403 | 0.539 | 31.826 | 2.124 |

prec, rec and F1 are means; Chamfer and band width are medians in mm.

- The validity window is closed in 143 of 250 scenes.
- Fixture (40 twin pairs, `out/bench_fx`), off → on: mean precision T 0.901 → 0.237, butt 0.739 → 0.143, corner 0.870 → 0.133, edge 0.184 → 0.139, lap 0.662 → 0.162.
- Sensor profile, noisy single view, median F1 / Chamfer / band width: d435i 0.546 / 8.414 / 3.433; stereo_good 0.538 / 8.092 / 3.386; stereo_poor 0.266 / 34.249 / 5.203.
- D19 conversion, median Chamfer by gap bin (gapmid / nominal / root):

  | gap bin | gapmid | nominal | root | spread |
  |---|---|---|---|---|
  | (0, 0.5] | 2.636 | 2.664 | 2.638 | 0.028 |
  | (0.5, 1] | 2.164 | 2.311 | 2.175 | 0.147 |
  | (1, 2] | 3.442 | 3.357 | 3.022 | 0.420 |
  | (2, 5] | 4.766 | 5.434 | 4.451 | 0.983 |

### 3.5 Notebook 07 [L-era: nb07; out/bench]

- 30-seed RANSAC boxes: 480 rows = 16 scenes × 30. Spread > 0.5 in 2 of 16 scenes; F1 = 0 at every seed in 6 of 16.
- Zero-spread table: ours, regiongrow and lobb 0.000 (6 scenes, 18 runs each); ransac max 0.574 (48 runs).
- RANSAC noise, median F1 at 0 / 1 / 2: 0.711 / 0.334 / 0.236. rmse_med 1.712 / 1.908 / 2.138; seam_recall 1.0.
- EMD demo, Chamfer vs EMD: full seam 0.3 mm off 0.91 vs 0.30; 10% stub 40.55 vs 32.27; two half-seams 0.91 vs 3.17.
- The fake-oracle validation figure has no printed numbers.

### 3.6 Notebook 14 [L-era: nb14; tackrule-0.1 on generated scenes]

- Butt bench6a: seams 0 and 1 each L = 181 mm, margin 22.6 mm (rule 22.6), gaps 136.0 and 135.9 mm, both inside [57, 187].
- Curved family 2: D39 phases {0: 0.0621, 1: 0.0804}; global weld sequence [0,1,0,1,0,1,0,1]. Ring 0: n = 4, first pair 225.9 mm apart = L/2. Ring 1: n = 4, 209.9 mm = L/2.
- T weld sequence [0,1,0,1].
- Short butt L = 37 mm gets one tack at s = 18.4 mm = L/2.

---

## 4. `out/` on this laptop (all stale relative to the desktop)

| dir | size | scenes | contents and dates |
|---|---|---|---|
| `annotation/` | 109 MB | 20 exported | 2026-09-01. Manifest: 20 scenes from `out/bench6a`, 4 per class, 30 primary seams. `export/`: 20 PLYs + BRIEFING. `annotations/annotator1` (43 lists, briefed) and `project_owner` (40, demo). `annotation_scores.csv` (40 rows). `annotator_model.json` |
| `annotation_archived_20260901/` | 154 MB | 20 | 2026-08-25/26 attempt on `out/bench`. `anil/` is empty; `cagdas` demo scored median lat_rmse 2.512 mm, p95 5.738, missed 9 / extra 7 of 38 |
| `bench/` | 1.4 GB | 448 | 2026-08-21. T 61, butt 53, corner 60, edge 92, lap 182. Used by nb07/nb12 and tests |
| `bench6a/` | 722 MB | 268 | 2026-08-27. T 60, butt 53, corner 52, edge 53, lap 50. Phase 5 source |
| `bench6b/` | 810 MB | 300 | 2026-08-31. 60 per class; manifest base_seed 2 000 000; T 6 families × 10; butt 15 + 15 + 30 |
| `bench_fx/` | 7.6 GB | 511 | 2026-08-25. Old plate twins |
| `bench_phase4/` | 2.9 GB | 720 | **Pre-visfix** (dirs 09-03..07). Manifest below. `facts.csv` (720 rows, 09-08). Gate report and retired-grooved json |
| `bench_phase4_fx/` | 3.8 GB | **274** | 2026-09-07. Plate twins only: T 54, butt 94 (40 + 54), corner 36, edge 50, lap 40. Rule blocks applied 2026-09-07. No curved twins |
| `phase4_batch/` | 26 MB | — | 53 chunk files + concat: **184 680 rows**, 760 scenes, 2026-09-09 06:50, pre-audit, modelreg plate-only (600 rows/chunk). Plus `retired_rows_retired_butt_line_grooved_scene_ids.csv.gz`: 13 800 rows, 60 scenes, 31 chunks, 2026-09-07 |
| `phase4_batch_dry/` | 0.3 MB | — | 2026-08-25. 44 chunks + concat (1 539 rows, 21 scenes, incl. `ours`, ransac 5 seeds) |
| `curved_smoke`, `gate_trial`, `smoke`, `phase1`, `phase2`, `phase2_fixture`, `phase3`, `phase3_raster` | — | 10 / 100 / 12 / 6 / 40 / 40 / 44 / 8 | 2026-08-15..31 |

- `bench_phase4/manifest.json`: per_family 60; T stride 6 (line, circle, ellipse, saddle, rounded_rect, swept_path); butt stride 4 (line_square, line_grooved rebuilt 2026-09-07, arc); rule blocks `mps_rule-0.1` / `tackrule-0.1` applied 2026-09-07.
- `bench_phase4/gate_report_2026-09-07.txt`: D28 terminal-bin mass 0.34 vs uniform 0.22 × 2.0 tolerance (= 0.44), PASS; 360 curved scenes (335 closed primary exempt, 180 open); D34 worst chord 0.0500 vs 0.25 mm, PASS.
- Missing here: `train_v1`, `backgrounds`, `render_assets`, `pilot_*`, the `_visfix*` corpora.

**Annotation numbers from the files [A].** Briefed annotator1: 20 scenes, of which 18 have at least one match within 15 mm.

- **Median lateral RMSE 1.4154 mm**; p95 11.8146; mean 2.71; range 0.354–12.92.
- Median end error 4.192; median lateral ME 2.32.
- Missed 6 of 30 (0.20): corner 4, lap 2. Extra 19 of 30 (0.633): edge 11 of 15 annotations, lap 6, butt 2.
- Ratio to 0.6 mm: 2.36, printed as 2.4.
- Per class, median lat_rmse / end_err: T 1.930 / 3.507; butt 3.378 / 5.571; corner 1.003 / 3.653; edge 0.423 / 2.215; **lap 8.500 / 24.555**. The lap value is the median of only 2 scenes (4.08 and 12.92); the other two lap scenes had no match.
- Curve preference, median RMSE nominal / root / gap-mid:
  - all scenes: 1.415 / 1.282 / 1.161
  - T: 1.979 / 1.282 / 1.414
  - butt: 3.378 / 1.980 / 1.306
  - corner: 1.003 / 1.031 / 0.958
  - edge: 0.564 / 0.748 / 0.564
  - lap: 9.96 / 9.96 / 9.96
- Demo (project_owner, excluded): median 1.921, p95 10.47, missed 6, extra 16.
- `annotator_model.json`: lateral_sigma 1.4154, endpoint_sigma 4.1920, miss 0.2, extra 0.6333.

---

## 5. Tests, one line each

- **`test_harness.py`**: the fake-oracle validation (jitter band and linear scaling; phantom costs precision, steals no match and leaves RMSE unchanged; miss gives recall 0 and F1 0), the registry and verify-seeds policy, measured zero spread, exact 2× noise displacement, and the EMD stub test. Needs `out/bench`.
- **`test_baselines.py`**: pins `ours` (radius-PCA) migration properties: fold signature, radius vs gap, validity window incl. the t/2 bound, mm units, object-id oracle flag, border firing, multi-seam clusters, band scoring. Pins the metric primitives: segment rather than line distance, both Chamfer directions, recall over seam length, densified polyline. Also the end-to-end run on `out/phase3`, single < full, batched == slow, and the fidelity guard that `lit-*` modules don't import directional clustering.
- **`test_annotation.py`**: the export ships no truth (only PLYs + BRIEFING; manifest outside; anonymous ids; 5 scenes at per-type 1). A fake annotator (25 mm clicks + N(0, 0.8 mm), one planted miss, one planted phantom 60 mm above) recovers missed = 1, extra = 1 and lat_rmse ∈ (0.5, 1.6). A demo-only run produces no headline.
- **`test_task2.py`**: run_task2 with the fake oracle. Jittered truth is matched with rmse < 1 and a finite open-seam end error. An empty prediction is an unmatched row (rmse NaN, MPS id kept). A closed ring gives end error NaN and finite rmse, and is rotation-invariant within 0.05 mm. A band method (`ours`) is refused.
- **`test_mps.py`**: the D25 order (visible arclength, then fold |180° − dihedral|, then lower id), null below 10 mm with the parameter effective, curved dihedral via the mean, and flag-gated emission (`twin_key` unchanged, schema-valid, content hash identical once the block is stripped).
- **`test_tacks.py`**: tackrule-0.1.
  - Margins: 600 mm at t = 6 → 24 mm.
  - Spacing within [10t, 33t] and the 400 mm ceiling.
  - Short seam: 80 mm at t = 8 → one tack at 40 mm.
  - Closed loop: even n ≥ 4, deterministic phase, first two tacks antipodal within 2 mm.
  - Round-robin stagger [0,1,0,1]; flag-gated emission.
  - Member-gauge regression (swept band 85 mm → gauge 6 mm); ring tacks on the ring (< 2 mm).

---

## 6. ICRA 2027 paper numbers (`paper/icra2027/`)

### 6.1 Inputs and wiring

`make_figures.py` reads:
- `out/phase4_batch/phase4_batch.csv.gz`
- `out/bench_phase4/facts.csv`
- `out/bench_phase4_fx/<jt>/index.jsonl` (twin keys)
- `out/annotation/annotation_scores.csv`

It writes `figures/*.pdf`, `tables/*.tex` and `numbers.tex`, using the same per-scene-median-then-median aggregation as nb15.

Wiring:
- `main.tex` inputs only `numbers.tex`, `task2.tex` and `fixture.tex`.
- `coverage_full.tex`, `straight_curved.tex`, `ladder.tex` and `cost.tex` are generated but not included in the paper.
- 50 of the 120 macros are unused (e.g. `\nRows`, `\taskTwoQuadricSel`, all ppf and ransac rung macros except `\ppfTolThirtyAngle`).
- `submission/tables` = `tables`, and `root.tex` = `main.tex`.

### 6.2 `numbers.tex`: every macro [D-final: paper]

Coverage-based macros are full-exterior median F1 unless noted.

**Coverage, strata and stratum groups**

| macro | value | meaning |
|---|---|---|
| `ransacTline` | 0.98 | M1 on T/line |
| `quadricTline` | 0.99 | M7 on T/line |
| `ransacCurvedMax` | 0.77 | M1 max over the 5 curved T strata |
| `ppfCurvedMin` / `ppfCurvedMax` | 0.27 / 0.42 | M4 min / max over curved T |
| `quadricCurvedMean` | 0.86 | M7 mean over curved T |
| `lobbCurvedMean` | 0.43 | M3 mean over curved T |
| `pcasliceCurvedMean` | 0.72 | M5 mean over curved T |
| `regiongrowCurvedMean` | 0.67 | M2 mean over curved T |
| `groovedMax` | 0.47 | best non-M6 F1 on grooved |
| `nStrataNonzeroLobb` | 10 | strata where M3 > 0.05 |
| `quadricCurvedRecallMin` | 0.58 | M7 min recall over curved T + arc |
| `quadricRingFoneMin` | 0.88 | M7 min F1 over circle, ellipse, saddle |
| `quadricRectPrec` / `quadricArcPrec` | 0.51 / 0.51 | M7 precision on rounded_rect / arc |
| `ppfCurvedRecallMin` / `ppfCurvedPrecMax` | 0.47 / 0.28 | M4 over curved T |
| `pcasliceClosedMax` / `pcasliceClosedMin` | 0.85 / 0.65 | M5 over circle, ellipse, rounded_rect |
| `pcasliceSwept` | 0.92 | M5 on swept |
| `regiongrowMax` | 0.80 | M2 max over strata |
| `ransacCornerFull` / `ransacCornerSingle` | 0.50 / 0.93 | M1 corner, both views |
| `pcasliceArcFull` / `pcasliceArcSingle` | 0.01 / 0.73 | M5 arc, both views |
| `pcasliceGroovedSingle` | 0.47 | M5 grooved, single view |
| `ppfTjoint` / `ppfAngle` | 0.62 / 0.03 | M4, T/line, T-joint vs angle joint |
| `lobbTjoint` / `lobbAngle` | 0.96 / 0.74 | M3, same split |
| `ransacTjoint` / `ransacAngle` | 0.99 / 0.93 | M1, same split |
| `ransacBimodalShare` | 5 | % of full-scan scenes with min F1 ≤ 0 and max ≥ 0.9 |
| `quadricBestDetectors` | 10 | strata where M7 ≥ best of M1–M5 − 0.005 |
| `ransacCurvedZeros` | 4 | curved T strata where M1 < 0.005 |
| `cornerGapMax` | 4.9 | max root gap (mm) on corner scenes (desktop `facts.csv`) |
| `lobbCorner` | 0.00 | M3 corner |
| `regiongrowGrooved` | 0.47 | M2 grooved |
| `quadricGroovedF` | 0.00 | M7 grooved |
| `modelregCurvedMean` | 0.91 | M6 mean over curved T |
| `modelregGrooved` / `modelregArc` | 0.97 / 0.95 | M6 grooved / arc |

**Audit rungs**

| macro | value | meaning |
|---|---|---|
| `pcasliceWholeClosedMax` | 0.07 | whole-ring rung, max over closed strata |
| `quadricProjRingRmseMax` / `quadricProjRingRmseMin` | 0.01 / 0.011 mm | projected rung, rings |
| `quadricRingRmseMax` / `quadricRingRmseMin` | 2.53 / 1.67 mm | headline (measured points), rings |
| `quadricProjCurvedMean` | 0.86 | projected rung, F1 mean over curved T |
| `quadricPooledRmse` | 1.60 | median of M7's 12 per-stratum median RMSEs (not a per-scene pooled value) |
| `regiongrowSurfaceCropMax` | 0.28 | surface-crop rung, max |
| `regiongrowTline` / `regiongrowCorner` / `regiongrowButtMax` | 0.58 / 0.51 / 0.51 | headline M2 |
| `lobbFlatEdge` / `lobbEdge` | 0.22 / 0.63 | edge: 3 mm rung vs headline |
| `lobbFlatButt` / `lobbButt` | 0.60 / 0.27 | butt/line: rung vs headline |
| `lobbFlatCorner` | 1.00 | corner, 3 mm rung |
| `ppfTolThirty{Angle, Tjoint, Corner, CurvedPrec}` | 0.60 / 0.59 / 0.50 / 0.28 | 30° tolerance |
| `ppfTolFortyfive…` | 0.60 / 0.58 / 0.50 / 0.28 | 45° tolerance |
| `ppfIntersecting…` | 0.59 / 0.57 / 0.46 / 0.28 | intersecting pair rule |
| `ppfPerpair…` | 0.03 / 0.63 / 0.50 / 0.31 | per-pair emission |
| `ransacIntersecting{Corner, Tline, Grooved}` | 0.50 / 0.98 / 0.06 | |
| `ransacThreshThreex…` | 0.50 / 0.82 / 0.00 | |
| `ransacOutlierfilter…` | 0.50 / 1.00 / 0.00 | |

**Ladder, noise, Task 2, fixture**

| macro | value | meaning |
|---|---|---|
| `quadricLzero` / `quadricLone` | 0.87 / 0.00 | pooled median over all merged scenes |
| `lobbLzero` / `lobbLone` | 0.40 / 0.03 | same |
| `regiongrowLzero` / `regiongrowLone` | 0.58 / 0.11 | same |
| `pcasliceLzero` / `pcasliceLone` | 0.64 / 0.00 | same |
| `modelregDenseButt` / `modelregButt` | 0.15 / 0.97 | M6 butt: dense-feature rung vs L0 |
| `noiseQuadricClean` / `noiseQuadricTwo` | 0.58 / 0.50 | pooled single view |
| `noiseLobbClean` / `noiseLobbTwo` | 0.22 / 0.30 | |
| `noiseRansacClean` / `noiseRansacTwo` | 0.20 / 0.00 | |
| `taskTwoQuadricSel` | 0.64 | unweighted mean of the 5 class rates |
| `taskTwoQuadricRmseMax` | 1.5 | max class RMSE, about 1.549 |
| `taskTwoLobbSel` | 0.56 | |
| `fixtureQuadricDFone` | −0.29 | |
| `fixtureLobbExtra` | 22 | |

**Annotation macros [A]**

| macro | value | meaning |
|---|---|---|
| `annMedian` | 1.4 | briefed median lat_rmse |
| `annPninetyfive` | 11.8 | p95 |
| `annLap` | 8.5 | lap median |
| `annEnd` | 4.2 | median end error |
| `annMiss` | 0.20 | missed / n_gt |
| `annRatio` | 2.4 | median / 0.6 |
| `annOwner` | 1.9 | non-briefed median |
| `annExtra` | 0.63 | extra / n_gt |
| `annButtNominal` / `annButtGapmid` | 3.4 / 1.3 | butt curve preference |
| `annTNominal` / `annTRoot` | 2.0 / 1.3 | T curve preference |

**Corpus**: `nScenes` 720, `nRows` 271,200, `nStrata` 12, `nRansacSeeds` 30.

### 6.3 Tables

All are [D-final: paper] and were last changed in 30cf17b (09:09, 269 760-row state). The 11:05 rebuild on 271 200 rows reproduced them unchanged; only `\nRows` changed in 38e8c21. Values are in §3.2:
- `coverage_full.tex`
- `straight_curved.tex`
- `ladder.tex`: pooled L0/L1, M1–M5 and M7
- `task2.tex`
- `fixture.tex`
- `cost.tex`: M6 L1 shown as "--"

### 6.4 The "NaN coverage figure"

- **HEAD is clean.** `figures/fig_coverage.pdf` (2026-09-15 11:05:23 UTC) and `main.pdf` (18:57:47 UTC, committed in 5cba849 at 19:03) show every cell filled, with no "--". `fig_ladder.pdf` already has the split T straight / T curved rows.
- **The stale files are PNGs.** `fig_coverage.png`, `fig_ladder.png`, `fig_noise.png` and `fig_annotation.png` were committed once (625011d, 2026-09-12) and never regenerated. They sit in `figures/`, `submission/figures/` and `icra2027_source.zip`.
  - `fig_coverage.png` shows "--" on all six curved strata and on M6 grooved, with pre-audit values (e.g. M2 T line 0.00).
  - `fig_ladder.png` has the unsplit T row.
- **The PNGs are not used.** `main.tex` includes the `.pdf` files explicitly (l.90, 99, 141, 157, 166, 197).
- **Why NaN.** On 2026-09-12 the 379 curved-strata ids had been retired from the batch after the visibility fix and not yet re-appended (`\nRows` 96,120), and `heat()` renders NaN as "--". The intermediate PDFs (d6cb08d 09-13, 295cdae 09-14) show "--" only for M6 on curved, grooved and arc, because modelreg was plate-only (slab/prism) until 2026-09-14; grooved scenes are `prepared_prism`.
- **The memory note is wrong for HEAD.** It says the git PDFs and `main.pdf` are stale ("11:05 build: curved strata NaN, ladder unsplit"); that holds only for the PNGs.
- Recommendation: delete or regenerate the PNGs and rebuild the source zip.

---

## 7. Discrepancy register

### High: would put a wrong fact into the thesis

1. **Row count.** 268 320 in the README, handoff, phase8_plan, dataset_plan and nb15 cell 0, versus a final 271 200 used by nb13, nb15 and the paper (§0).
2. **Densification step.** Paper §III-D says "densified to 0.5 mm spacing and matched at a tolerance τ = 3 mm". The code densifies at 1.0 mm (metrics.py l.92, 113, 338, 381; `_score` passes no step) and matches at 0.25 mm. Matching also has no tolerance; only F1 does.
3. **"Mean error".** The paper says "M1 gives 0.60 mm RMSE and 0.81 mm mean error". 0.81 is ME = **maximum** error (README: "ME 0,81 mm"; code `me = d.max()`).
4. **Runtime claim.** The paper says "every method runs in well under 4 s per scene". `cost.tex` from the same build shows M6 at 5.01 s (L0 full) and 4.59 s (single), M3 L1 at 10.42 s and M2 L1 at 3.61 s.
5. **Task 1 truth set.** The paper (§III-D, Fig. 3 caption) and MANUAL §1.4 define Task 1 as "every weldable seam". The code scores **primary seams only** (`prepare(primary_only=True)`, `run_phase4_batch.py` l.189), so off-class weldable seams count as false positives. That is 64 seams in 29 of 60 butt/line scenes and 15 in 5 of 60 grooved scenes [L-stale; plates identical to desktop]. The `dataset.py` docstring says finding them "is not wrong", but precision penalises them.
6. **Ladder claim about M2.** The paper says M2's crop "carries more than any other in the ladder (0.58 → 0.11)". In the same paper M7 drops 0.87 → 0.00 and M5 0.64 → 0.00, both larger.
7. **"Corners and butts return nothing" for M3.** Square butts give 0.27 (`\lobbButt`) and arc 0.49; only corner (0.00) and grooved (0.00) return nothing.
8. **Our own extractor on edge joints.** The paper says it scores "below 0.05 precision on edge joints". The source is a 2026-08-20 full-geometry measurement [doc: dataset_plan l.208–213]. nb12 (full_exterior) shows a mean of 0.199 [L-era: nb12], and `ours` has no final-batch rows.
9. **"Both curved butt strata".** Both the paper and nb15 cell 7 say this; there is one curved butt stratum (arc). M1 is 0 on all three butt strata.
10. **"Fillets from the root".** The paper says "on fillets 1.3 mm from the root against 2.0 mm". That is T-only (4 scenes). Corner fillets give nominal 1.00, root 1.03, gap-mid 0.96 [A].
11. **M3 corner reproduction.** The paper's "median 0.51 mm RMSE on corner joints" is the pre-audit README value (25 scenes, old reading). After the audit the test pins one corner scene at RMSE 0.112 / ME 0.416 mm (`tests/test_lit_lobb.py` l.385–410), while benchmark corner F1 is 0.00.
12. **MANUAL §8.3 runtime.** "~64 h … 16-core laptop … 20×" describes the 2026-09-05 first run (38 chunks, 169 800 rows). "16-core" means 8 cores / 16 threads. The final run was about 10.5 h on the desktop, and the 20× is unsupported for RANSAC (§2.5).
13. **MANUAL §9 ("as of 2026-09-08") is pre-audit and pre-visfix.** My stale recomputation reproduces most of its numbers [L-stale]. Old value → final:
    - ransac T 0.97 → 0.98
    - "0,00 on every curved family" → 0.77 on swept
    - ppf 0.25–0.36 → 0.27–0.42
    - quadric "pooled 0,93" (actually the median of stratum medians) → 0.87
    - quadric "rings at 0,01 mm" → 1.67–2.53 mm
    - lobb "nonzero on every stratum but grooved" → corner is now 0.00
    - lobb Task 2 0.79 selection / 0.7–1.6 mm → 0.56 class-mean / 0.6–1.2 mm
    - lobb noise 0.44 → 0.31 → now 0.22 → 0.30
    - lobb ladder 0.43 → 0.03 → 0.40 → 0.03
    - +23 seams → +22
    - "deterministic five" → six
14. **nb15 prose contradicts its own outputs.** The stale values come from the 2026-09-09 laptop batch or the 2026-09-13 intermediate batch. Cell by cell:
    - Cell 0: "modelreg absent (NaN) on curved/grooved" (it runs on all 12 strata; cell 39(c) says so); "the other five are deterministic" (six); "268 320".
    - Cell 7: lobb "only method nonzero … except grooved" (false); lobb full vs single "0,12/0,13/0,10 vs 0,43/0,57/0,34" (final 0.39/0.32/0.20 vs 0.40/0.39/0.29); "0,41 vs 0,38" (final 0.45 vs 0.31); modelreg "0,95–0,99 wherever it runs" (final 0.36–1.00); "edge 0,29–0,37" for the first six (final regiongrow 0.72, lobb 0.63); quadric "leads or ties every stratum but grooved" (false on rounded_rect: 0.55 vs M2 0.75); quadric "0,90 edge, 0,88 lap" (final 0.89, 0.87).
    - Cell 12: lobb 1.7, ppf 1.9, regiongrow 56, modelreg 0.66 mm (final 1.08, 1.53, 2.16, 0.70; the old values match the stale batch: 1.74 / 1.88 / 55.63 / 0.66).
    - Cell 22: ppf exact normals "+0,50 / +0,41" (final +0.49 / +0.42); dense "0,97 → 0,11" (final 0.15).
    - Cell 24: figure title "the winner degrades".
    - Cell 30: regiongrow "5–20 mm" (final 0.5–6.2 mm, and it contradicts the same paragraph); "corner lobb 0,35" (final 0.30); quadric "1,2–1,6 mm" (table max 1.55).
    - Cell 33: "modelreg and regiongrow are unmoved" (regiongrow −0.14).
    - Cell 36: lobb "0,81 → 0,79" (final 0.96 → 0.74; stale gives 0.81 / 0.79); ppf "0,63" (0.62).
    - Cell 37: "single view 3–5× cheaper for every method" (final 1.1–3.8×); "method call only — not oracle preparation" (see §1.4).
    - Cell 39: "every plane-intersection method scores zero on every curved family" (ppf 0.27–0.37); "the one mechanism that survives curvature — the crease detector"; "0,06 vs 0,41"; "(0,35)"; "+23 phantom seams, corner ΔF1 −0,49" (final +22, lobb corner 0.00).

### Medium

15. **README l.12–13** says "the plane-intersection mechanism scores zero on every curved family" (same issue as items 13–14).
16. **EMD** is presented as the secondary metric (MANUAL §8.2) but is never computed in the batch, nb15 or the paper.
17. **lat_p95 and Chamfer are never reported**, and per-stratum RMSE exists only as a PNG.
18. **Task 2 "selection" is uncapped and depends on the oracles.** Task 2 oracles are built from all weldable seams (§1.5).
19. **Pooled medians are reported** (ladder, noise, Task 2 class-mean) despite the "never pooled" rule (nb15 cell 0, batch docstring). The pooled values are T-dominated (360 of 720 scenes).
20. **Rounding.** The paper says M7 localises "within 1.5 mm on every class" while Table II shows 1.55 (T).
21. **Unreported rungs**: `modelreg_edge_features`, `quadric_projected_single`, `pcaslice_whole_ring_single`.
22. **The fixture study covers plates only**: 40 pairs, T/line only for T, from 274 plate twins. MANUAL §7 lists 634 twins (curved added 2026-10-19), never run; the laptop has 274.
23. **Viewer README** l.29 says "274 fixture twins" while l.8 says 634.
24. **Batch script**: the fixture docstring says "~50" (actual 40); `--include-ours` does not exist; `--list` assumes modelreg is plate-only.
25. **Harness docstring** says the default is "verify_seeds runs (default 3)"; the batch uses 2.
26. **Phase 5 sample sizes.** The lap median is from 2 of 4 scenes; the headline is over 18 of 20 scenes. Phase 5 "lateral RMSE" (supporting line) is not the methods' RMSE (segment).
27. **Paper "Only M6 exceeds it [M7]"** is false on rounded_rect (M2 0.75, M5 0.65) and grooved (M2 0.47). "Fail by the same factor" is also loose (M7/M5 → 0, M2 about 5×, M3 about 13×).
28. **nb12 / README** say `ours` lap "band width 5,8 → 3,1 mm"; nb12's output median is 3.487.

### Low

29. nb15 0.92 vs paper 0.93: rounding of 0.925.
30. nb15 cell 28 says "T/line exactly 0", but the graded share is 0.05.
31. "Plane methods go to zero at the first noise step" holds pooled; on stereo_good at 1σ M1 is 0.08 and M4 0.04 (§3.2 recovered values).
32. The paper's "M4 … stays at 0.60 at 30°" is odd wording (it rises from 0.03).
33. `export_for_annotation.py` defaults to `out/bench`, but the study used `bench6a`; `--with-joint-types` cannot be turned off.
34. `run_task2` ignores `tol_mm`.
35. MANUAL says "annotators" (plural) but there was one briefed annotator.
36. The jitter test expectation is about 1.15σ, not exactly σ (my derivation, §1.6).

---

## 8. Only on the desktop and needed for the thesis

1. **The final batch**: `out/phase4_batch/` with 271 200 rows and 67 chunk files, plus `pre_audit_chunks/` and the retired-rows archives for the 379 curved ids. It is the only source for:
   - per-stratum `rmse_med`, `lat_p95`, Chamfer, pred_to_gt and gt_to_pred
   - the covariate bins
   - the unreported rungs
   - any EMD column
   - the selection/localization by stratum, rather than by class
2. **The final corpus**: `out/bench_phase4` (visfix2 / entry-only rebuild, 2026-09-12) with its `facts.csv` and `gate_report_2026-09-12.txt`, plus the `_visfix1` and `_pre_visfix` archives.
3. **The twin-gate result**: `out/pilot_2026-09-11/gate_final_12strata.json` (twin gate 12/12 PASS).
4. **The full fixture-twin corpus**: `out/bench_phase4_fx` with 634 twins (360 curved, 2026-10-19).
5. **Phase 8 data**: `out/train_v1` (3600 scenes × 10 views = 36 000 frames), `out/backgrounds`, `out/render_assets`.
6. **Run evidence**: the batch queue and logs (`p4_queue.sh`, the 10.5 h and 15.2 h timings).
7. **The submitted PDF**, if it differs from the repo `main.pdf` (18:57:47 build). The repo copy already looks complete.

The `out/annotation` files are on both machines and are authoritative.
