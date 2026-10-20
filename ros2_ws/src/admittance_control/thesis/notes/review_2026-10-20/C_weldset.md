# C — WeldSet material map (weld_generator) for the thesis

Reviewer scope: everything under `/workspaces/welding_cell_ws/weld_generator` (README, docs, notes,
weldgen/, scripts/, notebooks/, tests/, configs/, Self_ideas/, papers/, paper/icra2027/), checked against
the draft chapter `ros2_ws/src/admittance_control/thesis/manuscript/chapters/ch4_methodology_weldset.tex`
(2026-10-16 first shape)
and the ch6 section "WeldSet Benchmark". Written 2026-10-20. Read-only review: nothing in the project
was changed.

Conventions used below:
* `file:line` cites the source; "nb15 cell 7" = stored output of a notebook cell.
* **[CODE]** = what the implementation does (the code and docs on this laptop are current, so the
  methodology sections are taken from them).
* Provenance labels on every result number (coordinator's rule):
  * **"laptop, stale: re-take on the desktop"** — measured from this laptop's `weld_generator/out/`
    (it predates the twin-gate "visfix" rebuilds and the 2026-09-14 fidelity audit) or from a notebook
    output computed here;
  * **"per <doc>, <date>"** — a number stated in a document or note;
  * **"desktop run, committed in <notebook/table>"** — produced on the RTX 5070 Ti desktop (out/ is a
    symlink to `/mnt/renders` there) and visible here only as a committed notebook output or ICRA table:
    usable as a pointer, but to be confirmed against the desktop files before it enters the thesis.
  * §4.9 is the list "DESKTOP: to collect there".
* "⚠" marks a discrepancy or a statement that must not enter the thesis as written.

# 0. Scope, sources, provenance, status

## 0.1 What WeldSet is
`weld_generator` builds synthetic two-part weld assemblies from the seam outward, so the seam (and, by a
versioned rule, the tacks) are known analytically (`README.MD:3-6`, `docs/MANUAL.md:11-25`). Tier 1 =
analytic clouds + exact ray-cast visibility + an analytic stereo-noise model, `trimesh` + NumPy only (D9);
tier 2 = the same scenes rendered in Isaac Sim / Replicator with seam/tack/object masks, paired by seed and
twin-gated. On the 720-scene, 12-stratum benchmark seven published extractors were reimplemented and run
(Phase 4), the hand-annotation floor was measured (Phase 5), an ICRA 2027 paper was submitted on the tier-1
results (2026-09-15), and two CAD-free extractors of the author's group were added on 2026-10-06 (§3).

## 0.2 Machines and provenance (decisive for every number)
* **Desktop (RTX 5070 Ti) holds the authoritative data** (`README.MD:25-28`; coordinator: `out/` there
  is a symlink to `/mnt/renders`): the final `bench_phase4`, the post-audit batch, `train_v1`,
  backgrounds and render assets. README: "Read notes/handoff_phase8.md and notes/phase8_plan.md before
  computing any number."
* **This laptop's `weld_generator/out/` is stale** (`out` is git-ignored, `.gitignore:1-2`):
  `out/bench_phase4` (scene provenance `created_utc 2026-09-03`, `schema_version 2.3.0`, rule blocks
  applied 2026-09-07; grooved stratum rebuilt 2026-09-07), `out/bench_phase4_fx` (dir dated 2026-09-07, i.e.
  only the 274 plate twins; the 360 curved twins of 2026-10-19, schema 2.4.0, are not here),
  `out/phase4_batch/phase4_batch.csv.gz` (2026-09-09, **184 680 rows**, pre-audit; lit-modelreg only on
  the five plate strata), no `train_v1`, no `out/self_footprint`, no `out/self_seamfind`.
* Desktop batch size: **268 320 rows** per README (2026-09-21) vs **271,200** per ICRA
  `tables/numbers.tex` macro `\nRows` (2026-09-18) — ⚠ unresolved; take the row count from the desktop
  file.

## 0.3 Timeline (git, `weld_generator`)
2026-08-13 papers/plan · 08-13 D12–D16 · 08-15 D17 withdrawn, D18–D21 · 08-16 D22–D24 · 08-20 advisor
meeting → D25–D27, tasks 1/2 · 08-21 repeat harness + EMD · 08-25 batch script, Phase 5 toolchain · 08-27
D28 (yaw + outlines), D31/D32, lit-lobb author reply · 08-28 D29 amendment, D35 grooves · 08-31 Phase 6b,
bench6b · 09-01 Phase 6c (MPS) · 09-03 Phase 7 tacks; `ours` excluded by ruling · 09-04 Task-2 chunk ·
09-05 "phase 4 ran" · 09-07 PreparedPrism, grooved stratum rebuilt, fixture twins, rule blocks applied ·
09-08 lit-quadric (seventh method), MANUAL written · 09-09 quadric authors asked · 09-10…12 Phase 8
rendering, "entry-only ray casting and visibility fixes" (the twin-gate bug fixes) · 09-14 fidelity
audit; "phase 4 run ended" · 09-15 ICRA upload ("complete before uploading to icra"), Phase 8 complete ·
09-21 README status, phase9_plan · 10-05/06 self-footprint, self-seamfind · 10-19 curved fixture twins
(schema 2.4.0) · 10-20 docs touched (MANUAL/SCHEMA/dataset_plan mtimes).

## 0.4 Coverage of this review (honest)
Update at hand-back: the full `dataset_plan.md` reading and the tier-2 reading (`weldgen/render`,
tier-2 scripts, phase8_plan, handoff_phase8) arrived and are merged (D-series sources and dates, the
Phase 4/5/6/7 history, the twin-gate thresholds and tier-2 caveats). The other seven readings below had
not reported.
Read line by line by me: the ch4/ch6 drafts; `docs/MANUAL.md`, `README.MD`, `docs/PARAMETERS.md`;
`notes/thesis_direction_handoff.md`, `notes/phase9_plan.md`, `notes/dataset_plan.md` §1–§3 (D1–D27);
`weldgen/tacks.py`, `mps.py`, `d29.py`, the random draws of `constructors.py`, `verify_curved.seam_verdict`;
`scripts/baselines/metrics.py` (F1, path errors, matching), `harness._score`; both Self_ideas methods,
their adapters, runners, tests and notebooks 16/17; notebooks 01/02 outputs; the ICRA source, tables and
bib; the cell modules' weldgen imports; `out/annotation`; the local `bench_phase4` manifest and all 720
`scene.json` (realised ranges). Nine background reading agents (the seven `lit_*` modules and notebooks
03–11, the plate core `geom/layouts/joints/config/scene/accessibility`, curves/scene_curved, camera/
sampling/visibility/noise/hashing/rng, `weldgen/render` + phase8 notes, harness/batch + notebooks 07/12–15,
`dataset_plan.md` in full, SCHEMA + patches + configs + tests) had **not reported when this report was
handed back**; items that depend on them are marked "not line-verified here" and should be completed
from those modules before the thesis text is frozen.

# 1. Per module / script / notebook / document

## 1.1 Documents
| doc | purpose, key content | feeds |
|---|---|---|
| `README.MD` (821 l., status 2026-09-21) | status page: phases 1–8 done, ICRA submitted, Phase 9 next; two machines; quick start; notebook guide; layout; Phase-3 products, seam counts per type (120 seeds), yields; D4's three arms; visibility; "things easy to get wrong"; baselines incl. the **fidelity audit 2026-09-14**; Phase 6 findings; Phase 8; Phase 9 | all of ch4 |
| `docs/MANUAL.md` (519 l., written 2026-09-08 at the close of Phase 7) | teaching doc: problem, design habits (truth vs convention, exactness, determinism, twins, anti-shortcut), pipeline 1–9 with RNG substreams, vocabulary, schema in words, reproducibility, corpora table, baselines + oracle ladder, findings (as of 09-08), rules layer, phases table, run commands, 5-minute version, **D-series glossary §14** | ws_goals…ws_protocol | 
| `docs/PARAMETERS.md` (743 l., params_version 2.2.0) | every range with a provenance tag [ISO]/[repo]/[ds]/[ours]; ISO 5817:2023 nos. 5071, 5072, 508 (not in Table 1 — Annex B fatigue only; β mapping D ≤ 4°, C ≤ 2°, B ≤ 1° is [ours]), 617 (throat aA = 0.7·min t [ours]); ISO 9692-1 fillet angles 60–120° [ISO], butt gaps b ≈ t / ≈ t/2, edge t ≤ 2 mm; groove pool by t; lap/edge one topology; quality = weakest of strictest; geometry table; fixture (D12); D28; ISO 17659 taxonomy; sensor profiles; pose sampling; σ_z table; density; radius-PCA window prediction; presets; open items | ws_parts, ws_sensor, ws_corpus |
| `docs/SCHEMA.md` (976 l.) | scene format, registries (faces, face_pair order, role, seam_class D22, frozen reject_reason vocabulary, bisector_blocked/work angle §2.6.1, too_short cross-run rule §2.6.2, seam ids), §1.3 D19 seam definitions, layout, `scene.json`, `cloud.npz`/`seams.npz`/`index.jsonl`/`splits.json`, reproducibility (substreams, content hash, D21 invariants, twin_key). Not line-verified here. Local scenes are schema 2.3.0; the 2026-10-19 curved twins 2.4.0 (new `contact_mode` member "line", git `64d4325`) | ws_d4, ws_corpus |
| `docs/scene.schema.json`, `docs/render.schema.json` | machine-checkable contracts | — |
| `notes/dataset_plan.md` (2671 l.) | the decision log (D1–D39) and the phase plans 0–9, metrics summary, named risks, open decisions; §1 "ground truth is constructed, never detected"; §2 D1–D27 table (verified, see §1.10); §3 Task 1 / Task 2 and the declined proposals (loading conditions, partial seams; D27 constraint); D28–D39 in the Phase 6/7 sections (l. 1616, 1657, 1697, 1810, 1845, 2181) | all |
| `notes/thesis_direction_handoff.md` (182 l.) | the direction arc (tack ordering → FEM → planner → dataset), settled: no welding, no FEM, no DL-on-FEM, extraction not a contribution, never an accuracy comparison; the advisor's dataset paper; "killer experiment" = annotation error; store/benchmark/regenerate table (RGB stored not benchmarked); tack-label triviality risk; method contribution (quality field, DP tack selection with d_min/d_max, ordering); literature map | ws_goals, ws_annotation, ws_tacks |
| `notes/phase9_plan.md` (138 l., 2026-09-21) | real subset: test-only, 11 strata × 10 configs × 5 views = 550 frames (grooved excluded), truth from measured dims + registered poses (D435i → SAM2 → FoundationPose → ICP), pose uncertainty stored, metal parts; **generator ranges measured on bench_phase4** (§1 table, used for §7 below); inventory and orders; fit-up vocabulary; pipeline to build; exclusions. ⚠ internal: header says "120-configuration matrix", §3 says 110; README says "12 strata × 10 × 5" | ws_corpus, ws_link |
| `notes/phase8_plan.md` (933 l.), `notes/handoff_phase8.md` (238 l.) | the render machine's log and the Phase 8 handoff; must be read before computing numbers (README). Not line-verified here | ws_sensor (tier 2) |
| `notes/plan_patch_v2.md` | source of D25–D27 (cited by `mps.py:3`). Not line-verified | ws_mps |
| `notes/patch_phase6.md`, `patch_class_disjointness.md`, `patch_phase6b.md` | Phase 6a/D31-D32/6b patch notes. Not line-verified | ws_parts, ws_curves |
| `notes/last_talk.md` (2026-08-13 file date) | meeting/talk notes. Not line-verified | — |
| `notes/correspondence/lit-quadric_authors_email.md` | the questions sent to the lit-quadric authors (2026-09-09); README 2026-09-21 still lists "the reply" as open | ws_protocol |
| standards PDFs in `notes/` | ISO 5817:2023 (Ed. 4), ISO 9692-1:2013, ISO 2553:2013, TS EN ISO 6520-1, BS EN ISO 17659:2004 | ws_parts |

## 1.2 `weldgen/` (generator; trimesh + NumPy, no simulator import, D9)
Purpose per `README.MD:153-193`; details I verified are marked ✓.
| module | purpose | notes |
|---|---|---|
| `rng.py` | 8 named `SeedSequence` substreams (geometry 0–2, sampling/noise 3–4/6, camera 5, render 7); draws appended, never inserted (MANUAL §3, §6) | twins rest on it |
| `geom.py` (2078 l.) | primitives + face registries + exact algebra: `Slab`, `Prism` (6a outlines), `PreparedSlab` (ISO 9692-1 groove), `PreparedPrism` (outline + groove, 2026-09-07), `Tube` (wall; base cut by a plane = miter or by a cylinder = saddle), `SweptSlab` (band along an exact spine); `from_object` rebuilds a primitive from `scene.json` (used by the cell) | watertight by construction (D21) |
| `curves.py` | exact curve library: ellipse/arc/saddle/B-spline/composite/offset, analytic positions **and** tangents, arclength sampling exact-on-curve; `ellipse_from_plane_cylinder`, `saddle_from_cylinders`, `rounded_rect_curve`, `OffsetCurve` | the cell reuses these |
| `d29.py` ✓ | the curve-first sampler: family table 1 line, 2 circle, 3 ellipse, 4 saddle, 5 rounded_rect, 6 swept_path (all T) and 7 arc_butt (butt); draws in §7 | `MIN_RADIUS_MM = 30` binds families 6–7 |
| `constructors.py` ✓(draws) | parts derived from the drawn curve (base plate t ∈ U(4, 12); tube wall U(3, min(8, 0.4r)); gaps U(0.1, 1.2), arc butt U(0.3, 2.0); heights) | §7 |
| `joints.py`, `layouts.py`, `config.py`, `scene.py` | JointSpec sampling (ISO limits, misalignment hinged at the contact, flush-on-one-face thickness), five plate layouts (placement only), DEFAULT_CONFIG + `valid_preps(t)` (D35), plate scene assembly, skip reasons `NoSeamsFound` / `NoVisibleSeams` | not line-verified here |
| `accessibility.py` | the D4/D13 rule: face-pair enumeration, three arms (intersecting / facing → `bisector_blocked` / coplanar exposed), torch cone (scene block: half-angle 30°, standoff 15 mm, work angle ≤ 45°, dihedral 30–170°, min seam 10 mm, coplanar tol 0.5 mm, cross-run tol 45°, `contact_tol_mm` capped by thickness — values read from a local `scene.json` `accessibility` block), class D22, `matches_joint_type`, D19 curves | not line-verified here |
| `verify_curved.py` ✓(verdict) | curved D4 arms: rediscovery from placed parts (10⁻¹³), per-point frames, `seam_verdict`: toes → `toe_of_centreline`; bores need cavity ≥ `bore_min_diameter_mm` = 80 else `confined_bore`; otherwise weldable iff ≥ 95 % of sample points have an unobstructed 30°/15 mm cone about their approach axis, else `bisector_blocked` (`:229-271`) | explains the ellipse tilt cap (§7) |
| `scene_curved.py` | curved pipeline; included angle = 90° − tilt (`:68-70`); camera/fixture/density draws (`:240-340`) | |
| `camera.py`, `sampling.py`, `visibility.py`, `noise.py` | pose sampler (profiles, standoff from framing, elevation 15–85°, roll ±15°, aim miss ±0.15·longest edge; regimes uniform_sphere / approach_cone D26); area_uniform / camera_raster (D20); exact occlusion (entry-only crossings after the 2026-09-12 fix), exterior flag, `hpr_exterior`; σ_z = s z²/(f b), lateral 0.8 px, dropout > 75°, parameters stored, realisation by `apply` (D14) | not line-verified here |
| `hashing.py`, `writer.py`, `cli.py`, `viz.py` | canonical JSON + content hash, config_id, twin_key; scene.json/cloud.npz/seams.npz/index.jsonl; `python -m weldgen generate | verify | verify-render`; assembly frame + cross-sections | |
| `mps.py` ✓ | `mps_rule-0.1` and `mps_margin` (§2.6) | |
| `tacks.py` ✓ | `tackrule-0.1` (§2.5) | |
| `render/` (12 modules) | Phase 8 tier 2 (`pxr`/`omni` imported inside functions only): conventions (USD camera, depth codec), render config → render_id, draws (substream 7), materials-1.1 (F0 from RTR4 Table 9.2, CC0 ambientCG), HDR dome rule, USD stage, Replicator annotators, `maskrule-0.1`, D16 on rendered depth, twin gate, writer | not line-verified here |

## 1.3 `scripts/`
| script | purpose | verified details |
|---|---|---|
| `baselines/metrics.py` ✓ | precision = share of predicted points within τ of the truth polylines (exact point-to-segment distance); recall = share of the truth resampled at `step_mm` within τ of a predicted point; F1; symmetric Chamfer (both directions kept); lateral distribution (p95 "the one that matters for a toolpath"); `path_error_mm` = Yi et al. §7.2.1 RMSE/ME over the predicted path resampled at 0.25 mm + lateral/length/endpoint decomposition; `match_seams` greedy one-to-one by mean distance (no distance gate); EMD (`emd_mm`, n = 256, 1 mm) | `evaluate(..., tol_mm=3.0, step_mm=1.0)` default; the harness passes only `tol_mm` → **1 mm densification in code** (⚠ the ICRA text says 0.5 mm) |
| `baselines/harness.py` | `prepare` (cache per-method oracles), `run_matrix`, `run_task2`, `_score` (adds `rmse_med`, `rmse_max`, `me_med`, `me_max`, `seam_recall`), `MethodSpec`, `spread`; noise multiplier on subpixel and lateral σ | |
| `baselines/dataset.py`, `__init__.py` | corpus loading, REGISTRY of the seven | not line-verified |
| `baselines/lit_*.py` (7) + `radius_pca.py` | the seven reimplementations + `ours` (excluded by ruling 2026-09-03) | summary §2.8, not line-verified |
| `baselines/self_footprint.py`, `self_seamfind.py` ✓ | §3 | |
| `run_phase4_batch.py` | the batch: explicit chunk list (coverage both views, L1, noise ×2, ladder rungs, task2, fixture), one csv.gz per chunk, resumable, `--append-missing`, `--retire`, `--list` | chunk names confirmed from the local batch: `coverage_<m>_<view>`, `l1_<m>_full_exterior`, `noise1/2_<m>_single`, `task2_<m>_single`, `fixture_<m>`, `ppf_exact_normals_full_exterior`, `modelreg_dense_features`, `modelreg_global_init`, `quadric_distance_*`, `retired_rows_*` (pre-audit set; the audit added rungs) |
| `run_self_footprint.py`, `run_self_seamfind.py` ✓ | §3 | |
| `make_bench6b.py` | corpus builder (bench6b; `--per-family 60` = bench_phase4; `--per-family 300 --base-seed 3000000` = train_v1); seeds interleaved per source | manifest ✓ (§7) |
| `qa_d28_gate.py` | D28 anti-shortcut histogram gate + binding D34 chord sweep | gate 0.34 vs 0.44 (paper); local `gate_report_2026-09-07.txt` exists |
| `make_fixture_twins.py`, `rebuild_stratum.py`, `apply_rule_blocks.py`, `emit_meshes.py` | D12 twins (`<corpus>_fx`), regenerate one stratum retiring old ids, retro-apply mps + tacks (hashes rewritten, ids kept), meshes + round-trip check | |
| `tier2_*.py`, `render_tier2.py`, `fetch_render_assets.py` | twin gate on N scenes, the tier-2 pass, regate/remask, CC0 asset fetch with licence/hash manifest | not line-verified |
| `annotation/export_for_annotation.py`, `score_annotations.py` | Phase 5: export clouds for CloudCompare (no truth ships), score clicks against analytic truth, annotator perturbation model | outputs ✓ (§4.6) |
| `dataset_viewers/` | viewers for bench_phase4 and its fixture twins (2026-10-19) | |

## 1.4 Notebooks
| nb | shows | provenance of stored outputs |
|---|---|---|
| 01 inspect_scene ✓ | constructed truth of one playground scene (here an ellipse, seed 132: chord error 0.0493 mm vs D34 0.25, included 65.1°, gap 0.134 mm; two closed fillets, dihedral 65.1–114.9°, clear 1.00); D19/D36/curved-frame regimes; content-hash determinism ("repeatable: True, provenance-independent: True") | laptop, 2026-08-31 (illustrative) |
| 02 inspect_visibility ✓ | single-view mask; per-seam verdict (saddle seed 12: outer fillet occl 0.53, vis 0.47 PRIMARY; bore rejected `confined_bore`, clear 0.73); azimuth × elevation visibility map; σ_z; area_uniform vs camera_raster (241 844 pts 1.396/mm² vs 604 972 pts 3.493/mm², both 36.0 % visible); yield table (playground config, 60 seeds/type: T 63 %, corner 38 %, butt 60 %, lap 23 %, edge 47 %) | laptop, stale, 2026-08-31 |
| 03 baselines | `ours` radius-PCA, validity window R < t/2 vs original, clustering, segmentation dependency | not checked here |
| 04 lit_ransac, 05 lit_regiongrow, 06 lit_lobb, 08 lit_ppf, 09 lit_pcaslice, 10 lit_modelreg, 11 lit_quadric (+ `build_11`) | one per method: every reading of the paper, reproductions, ambiguities (README `:91-120`) | not checked here |
| 07 repeats | fake-oracle validation, ransac seed box plots, zero spread of deterministic methods, noise multiplier, EMD | not checked here |
| 12 first_results, 13 seven_methods | the nine first-results plots; the older seven-method batch figures | not checked here |
| 14 inspect_tacks | tackrule-0.1 drawn on 5 plate joints and 6 curved families, order, D39 phase, stagger, one-tack collapse | not checked here |
| 15 phase4_results (+ `build_15`) | THE Phase 4 results: composition, coverage heatmaps, dumbbell, localisation, repeatability, ladder, noise × profile, Task 2 + MPS margin, fixture price, covariates, ISO 17659 split, cost, D19 offsets | must be re-read from the desktop execution |
| 16 self_footprint ✓, 17 self_seamfind ✓ | §3 | desktop run, committed 2026-10-06 |

## 1.5 Tests and configs
* Tests: 460 tests as of 2026-09-08 (per MANUAL §6) and **564** as of 2026-09-21 (per README `:229`), plus
  the later self-method tests (3 + 6, §3). Per-file inventory not re-counted here.
* Configs ✓: `bench6a_{T,corner,butt,lap,edge}.yaml` = one joint type each, `prep: square`, fixture off,
  `area_uniform`, profiles [d435i, stereo_good, stereo_poor], `in_plane_yaw: true`, `polygon_outlines:
  true`, `class_disjoint: true` (these build the five plate strata of bench_phase4); `grooved_butt.yaml` =
  butt, `thickness_mm: [3.0, 20.0]`, `groove_preps: true`, outlines + yaw on since 2026-09-07;
  `curved_smoke.yaml` = families 2–7; also `phase1/2/2_fixture/3/3_raster`, `bench_*` and `bench_*_fx`
  (pre-6a), `amb_lap`/`amb_T` (D32 boundary stratum), `task2_cone`, `smoke`, `reference_tjoint` (t 8.4 mm,
  g 1.1 mm, seam 232 mm, spacing 2.7 mm, d435i per PARAMETERS §6), `playground`, `render/lab_v1.yaml`
  (render_id).

## 1.6 `papers/` (list)
WeldNet; A Feature Extraction Approach Over Workpiece Point Clouds for Robotic Welding (the LOBB T-ASE
paper); LWSNet; 3D curve weld seam path and posture planning based on line laser sensors; RRT-Connect
(cell); A novel method for dense point cloud reconstruction and weld seam (tubesheet); YOLO–LaserGalvo;
PAEAR (point clouds area exploration … reinforcement learning); Path Planning with Automatic Seam
Extraction over Point Cloud Models (Zhou 2020); SANet; Weld seam detection based on 2D image and 3D point
cloud segmentation: a coarse-to-fine weld path planning method; A Coarse-to-Fine Multimodal Detection
Framework … Robotic Coating (Lin 2026); A novel seam extraction and path planning method for robotic
welding of medium-thickness plate structural parts based on 3D vision; and `papers/implemented_papers/`
(the seven reimplemented papers).

## 1.7 The ICRA 2027 paper (`paper/icra2027/`)
`main.tex` (= `submission/root.tex`, 218 l., identical tables), title "WeldSet: An Analytically
Constructed Benchmark and Rendered Dataset for Weld-Seam Extraction", anonymous; `tables/numbers.tex`
(120 macros), `cost/coverage_full/fixture/ladder/straight_curved/task2.tex`, `make_figures.py`,
`make_gallery.py`, `refs.bib`, figures (gallery, renders, coverage, ladder, noise, annotation). Method
keys: M1 `yi2026seam`, M2 `wei2024coarse`, M3 `zhang2025lobb` (+ `zhang2025lobbtase`), M4 `wang2024ppf`,
M5 `wang2026pipe`, M6 `fang2024model`, M7 `li2026impeller`. Generative-AI use is acknowledged in the
paper. Project memory notes "submitted with a NaN coverage figure to fix" (not traced here).

## 1.8 The seven methods (summary; modules not line-verified here)
Per MANUAL §8.1 and README `:514-523`: M1 lit-ransac (Yi et al., *Autom. Constr.* 183 (2026) 106792:
improved multi-plane RANSAC, pairwise plane intersection; randomised, 30 seeds; L0 = ~40 mm seam band for
their PointNet++); M2 lit-regiongrow (Wei et al., arXiv:2408.10710 (2024): curvature-seeded region growing,
edge points spanning two grown surfaces; L0 = the paper's keypoint crop, 12.7 % of the cloud, after the
audit); M3 lit-lobb (Zhang et al., *RCIM* 95 (2025) 102987 with the LOBB descriptor of *IEEE T-ASE* 22
(2025) 75–84: bounding-box flatness, tanh, binary K-means, polynomial fit; L0 = per-component masks for
their K-Net; first author's endorsement 2026-08-27); M4 lit-ppf (Wang et al., *Sci. Rep.* 14 (2024) 21137:
PPF coplanarity, orthogonal-pair Hough voting, farthest-pair corners; L0 = weld-box crop for their Faster
R-CNN; extra exact-normals rung); M5 lit-pcaslice (Wang et al., *Weld. World* (2026), doi
10.1007/s40194-026-02509-9: PCA-adaptive slicing, per-slice centres, B-spline; L0 = per-instance masks for
their YOLO11 + DeepLab; closed rings as four open arcs after the audit); M6 lit-modelreg (Fang & Tian,
*RCIM* 89 (2024) 102772: CPD registration of a basic model, seam transferred; constitutively L0, rungs
dense features / global init); M7 lit-quadric (Li, Wang & Wang, *J. Mech. Sci. Technol.* 40(3) (2026)
2039–2046: PCA-normal statistics → plane or general quadric per welding surface, seam = measured points
within a threshold of the fitted intersection; L0 = welding-surface labels + part membership). `ours`
(radius-PCA band, lab repo) excluded "by ruling" 2026-09-03.

## 1.9 Fidelity audit (2026-09-14, per README `:531-545`)
Every module re-read against its PDF; six readings corrected, old readings kept as labelled ladder rungs:
pcaslice closed rings as four open arcs + periodic spline (circle 0.06 → 0.85); regiongrow gets its
keypoint crop (12.7 %, Table II) and curvature condition (T line 0.00 → 0.58; reads a groove 0.47); quadric
reports measured points, not their projection (RMSE 0.01 → 1.7–2.5 mm, F1 unchanged); lobb edge window
3×3 px ≈ 1.5 spacings (edge 0.29 → 0.63, corner 0.99 → 0.00; 3 mm window kept as a rung); ppf and ransac
invented tolerances made explicit rungs; modelreg runs on every stratum; two bugs fixed (unused `labels`
in regiongrow's edge test; both signs of pcaslice's eq. 12 torch frame). README's earlier paragraphs are
the pre-audit record; "the current numbers are notebook 15's".

## 1.10 The D-series (D1–D39; no D40+ exists — grep over notes/docs/code)
D1–D27 verified in `notes/dataset_plan.md:32-64`; D28–D39 one-liners from `docs/MANUAL.md:503-519` and
the README/PARAMETERS passages cited.
| D | meaning (one line) | date |
|---|---|---|
| D1 | seam truth is analytic, from the placement transform | Phase 0 |
| D2 | radius-PCA is a baseline evaluated against truth, not infrastructure | Phase 0 |
| D3 | inverted pipeline: sample joint → seam → place parts; no intersection solving at generation | Phase 0 |
| D4 | weldable seam = exterior face pair whose dihedral bisector escapes to free space (torch reachability) | Phase 0 (three arms later) |
| D5 | several seams per scene are the norm (T 2 fillets, lap 2 toes, corner 2, butt 1–2, edge 1–2) | counts revised 2026-08-16 |
| D6 | one full cloud + per-point `visible_from_cam` mask, not two clouds | Phase 0 |
| D7 | tier boundary = sensor realism, not visibility; tier 1 ray-casts occlusion | Phase 0 |
| D8 | tacks are a derived layer from a versioned rule, separate from geometric truth | Phase 0 |
| D9 | no simulator import in tier 1; renderers are pluggable backends | Phase 0 |
| D10 | RGB rendered and stored, not benchmarked on | Phase 0 |
| D11 | splits hold out part geometries and joint configurations, not random frames | Phase 0 |
| D12 | fixture sampled ~50/50, paired seeds, pose-varied (tilt ±10°, height not pinned) | 2026-08-13 (revised) |
| D13 | both faces must belong to workpieces; part–fixture contact = `fixture_contact` | 2026-08-13 |
| D14 | cloud stores the noiseless sample + noise params; realisation by released `apply_noise` | 2026-08-13 |
| D15 | determinism gate = content hash, not byte identity | 2026-08-13 |
| D16 | stereo depth model with named profiles (d435i, stereo_good, stereo_poor) | 2026-08-13 |
| D17 | procedural features (chamfers, holes…) — **withdrawn** | 2026-08-15 |
| D18 | included angle is a sampled design parameter, independent of angular misalignment | 2026-08-15 |
| D19 | with a root gap the stored seam is the nominal zero-gap intersection; root and gap-mid derived | 2026-08-15 |
| D20 | under `camera_raster` the hidden surface is sampled separately at matched density | 2026-08-15 |
| D21 | every object watertight and winding-consistent; the union is not (disjoint at g > 0) | 2026-08-15 |
| D22 | seam class from the faces (edge×edge edge, edge×face lap_toe, coplanar face×face butt, angled face×face fillet); off-class kept as `wrong_class_for_joint` | 2026-08-16 |
| D23 | stacked joints (lap, edge) cap angular misalignment at ~0.4° | 2026-08-16 |
| D24 | groove preparations deferred to Phase 6 (Phases 3–5 square prep only) | 2026-08-16 |
| D25 | MPS label is a versioned rule (`mps_rule-0.1`) | 2026-08-20 |
| D26 | two camera regimes, approach_cone and uniform_sphere, twin-paired | 2026-08-20 |
| D27 | part dimensions vary across the seam, never along it | 2026-08-20 |
| D28 | only the seam-bearing edge may be axis-aligned (yaw + polygon outlines + corpus gate) | 2026-08-27 |
| D29 | curved seams drawn first from curve families that admit a two-part realisation | amended 2026-08-28 |
| D30 | grooves on straight butt seams only (a profile swept along a turning frame self-intersects; ISO 9692-1 has no pipe preparation) | 2026-08-27 (`notes/patch_phase6.md`) |
| D31/D32 | seam classes disjoint by construction (clearance c = 2·min(t_A, t_B)) + the class-boundary stratum (`amb_lap`, `amb_T`, `ambiguous_with`) | 2026-08-27 |
| D33 | every D29 seam is closed-form or constructed ("D1 stands unqualified"); B-splines constructed inputs, never fits; the saddle an exact quadratic verified ≤ 10⁻⁶ mm; the tolerance clause withdrawn | 2026-08-28 (`notes/patch_phase6b.md` L47–80) |
| D34 | render-mesh chord error ≤ 0.25 mm, bias ≈ RΔθ²/8, pitch Δθ ≤ √(8·tol/R), recorded per scene (worst measured 0.0500 mm) | 2026-08-28 (patch_phase6b L84–93) |
| D35 | groove rows: `valid_preps(t)` over PDF-verified ISO 9692-1 rows | 2026-08-28 |
| D36 | grooved centrelines store `groove_root` (t − c below the top) instead of the D19 triple | 2026-08-28 (patch_phase6b L121–137) |
| D37 | torch clearance for grooves tested from the mouth, not the root (realised at zero code cost: the nominal was already mouth-anchored) | 2026-08-28 (patch_phase6b L141–155) |
| D38 | `tackrule-0.1` | Phase 7 (plan 2026-09-01; committed 2026-09-03) |
| D39 | closed-loop tack phase from sha256(scene_id, seam_id); closed-seam tack scoring must be rotation-invariant (extended to Task 2: `end_error_mm` NaN on rings) | Phase 7 (as D38) |
Sources (from the full reading of the plan): D1–D27 in §2 (L32–60; D1–D11 frozen with schema 1.0.0 on
2026-08-13 per SCHEMA.md L3); D28–D30 = `##` headings in Phase 6 (L1616, 1657, 1697) from
`notes/patch_phase6.md` (2026-08-27); D31/D32 summary L1810, full text `notes/patch_class_disjointness.md`
(clearance near-AND-parallel within ~10°; class-boundary stratum ≈ 5 % of corpus size; no class-boundary
result recorded); **D33–D37 are only cited in the plan, defined in `notes/patch_phase6b.md` L47–155**;
D38/D39 inline in Phase 7 (L2188, L2210). D22's `wrong_class_for_joint` was retired by SCHEMA 2.3.0
(2026-08-17) in favour of `matches_joint_type` (the plan's §2 row is stale). MANUAL §14 nuances: D11 is a
held-out *dimensions* split through Phase 5; D24 says "Phase 6" (and J/double preps were later dropped);
D33 in full is "closed-form **or** constructed". D28 gate history: trial PASS 0.33 vs 0.22 uniform within a
2× tolerance (allowed 0.44) on 2026-08-27; bench_phase4 FAIL 0.45 vs 0.44 on 2026-09-03; PASS 0.34 on
2026-09-07. D26 is implemented with `approach_visibility_min` 0.5 (2026-09-01). ⚠ patch_phase6b's D34
rationale cites a superseded "2.5 mm Phase 5 annotation floor" (the author's early demo self-annotation).

# 2. Per thesis section of ch4 — facts, formulas, numbers, rationale

## 2.1 Design goals (`sec:ws_goals`)
* One rule: "ground truth is constructed, never detected" (`dataset_plan.md:10`); the label is the curve
  drawn; consequences: exact, re-samplable ("10 dots per mm, or 50 if someone asks"), reproducible (the
  release can be a program) (`:15-19`).
* Three faults of the field (MANUAL §1.2): inexact truth (Yi et al. state ~0.5 mm truth error and report
  0.64 mm RMSE), conditions not varied, datasets "available on reasonable request".
* Five habits (MANUAL §2): truth vs convention (D8, D14, D19, D25, D38: conventions ship as versioned rule
  functions; mps + tacks were retro-applied to the finished corpus "in 63 seconds"); exactness contract
  (analytic curves with tangents, watertight meshes D21, chord budget D34 ≤ 0.25 mm measured 0.05,
  closed-form containment, point-to-segment seam distances); determinism by content hash (D15;
  `scene_id = config_id + seed`); twins (only non-geometry keys flip: fixture presence, sensor profile,
  density, camera regime; `twin_key` hashes geometry keys + seed); anti-shortcut (D28 gate, D31).
* Accessibility: D9 "a generator that needs Isaac Sim and an RTX card is as inaccessible as the datasets
  it criticises"; tier 1 = trimesh + NumPy.
* Reproducibility caveat (ICRA §III-C): under a different numpy release arrays agree to 10⁻⁵ relative and
  hashes do not → the release pins its environment (numpy 2.3.5 / Python 3.12.3 in local provenance).

## 2.2 Parts, joints and assemblies (`sec:ws_parts`)
* Primitives (MANUAL §3.2): Slab (Phase 1), Prism (convex polygon outline, 6a, D28), PreparedSlab (ISO
  9692-1 V / bevel / U, 6b), PreparedPrism (outline + groove, added 2026-09-07 post-Phase-4), Tube (wall;
  base cut by a plane = miter or a cylinder = saddle, 6b), SweptSlab (band along an exact spine: ellipse,
  arc, rounded rect, B-spline; 6b). Face registry (`+w`, `-w`, `s0…`, `root`, `fusion`, `lateral±` …); `w`
  is always the thickness axis.
* Joint types: T, corner, butt, lap, edge (ISO 17659: T-joint 3.10 at ~90°, angle joint 3.12 otherwise;
  corner 3.13; butt 3.7; lap 3.9 partial overlap; edge 3.14 at 0–30°, the flush stack at 0°; parallel joint
  3.8 excluded as an area bond). Each scene stores `joint.iso_17659_term` and `iso_9692_ref`.
* Sampled parameters (PARAMETERS §2–3): t 1–12 mm [ours] (30 % dissimilar), along-seam length 80–400 mm,
  width 50–250 mm [ours]; root gap butt ≈ t (one side) / ≈ t/2 (two sides) [ISO], fillet ≤ 2 mm [ISO]
  bounded by no. 617 for the target level, plus an over-range tail [ours] (realised up to 4.9 mm on
  corners); included angle T/corner 60–120° [ISO 9692-1 Tables 3–4], butt 180°, lap/edge 0°; linear
  misalignment by no. 5071 per level; angular misalignment by the [ours] mapping over Annex B values;
  edge t ≤ 2 mm [ISO ref 1.1]; lap overlap [ours] (no ISO source); D23 β cap 0.4° on lap/edge; quality level
  derived after the draw (target mix 25 % B/C/D/below_D).
* Placement subtleties (README "easy to get wrong"): misalignment hinged at the contact; linear
  misalignment is a step, not a second gap; unequal thickness flush on one face; parallel tolerance 8°;
  aim point misses the joint by ±0.15 of the longest edge.
* Fixture (D12/D13, PARAMETERS §3.1): slab, `role: fixture`, object_id 255, 600×400×10 mm ±20 %, tilt ±10°,
  height ±50 mm, yaw 0–360°; contact modes flat 0.6 / on_edge 0.3 / propped 0.1; emitted in exact pairs;
  part–fixture contacts are hard negatives (`fixture_contact`).

## 2.3 Curved seams by construction (`sec:ws_curves`)
* D29: "a seam family is a curve family with a realizability condition, not a workpiece catalogue"; the
  curve is drawn first, parts derived (`d29.py:1-28`); surface ∩ surface lives only in verification
  (`rediscover_seam` recomputes each seam from the placed parts at 10⁻¹³ and detects a 0.5 mm
  perturbation, README `:703-705`). D33: every curve exact (closed-form quadric intersections or
  constructed sweeps).
* Closed forms as constructed (`d29.py`): circle $\mathbf c(\theta)=\mathbf c_0+r(\cos\theta\,\mathbf e_x+\sin\theta\,\mathbf e_y)$,
  r = pipe outer radius; ellipse with semi-minor b = pipe radius, semi-major $a=b/\cos\tau$ (τ = tilt),
  axis $\cos\tau\,\mathbf e_z+\sin\tau\,\mathbf u$ in the plane of the base normal and the major direction;
  saddle = the D33 closed-form parametrisation of `saddle_from_cylinders(origin, axis, r, …, R)` with
  branch tilt and offset; rounded rectangle = straight runs + quarter arcs of corner radius r_c (the tube's
  outer wall); swept path = arc (radius ≥ max(30, span/π)) or a 4–5-point cubic B-spline with curvature
  radius ≥ 30 mm; arc butt = arc of radius R. Swept-stiffener welds are the **offset curves at ±t/2** of the
  spine ("mid-material is not a weld").
* The cell's Eq. `eq:ellipse_seam` is the plane∩cylinder form (`curves.ellipse_from_plane_cylinder`), i.e.
  WeldSet's *verification* direction, not its constructor (see §5).
* Bore rule: an open 50 mm bore clears the local cone; what forbids the inner fillet is torch-body
  confinement → `bore_min_diameter_mm = 80` [ours] (README `:708-710`); the cell uses 200 mm for its pen +
  camera (`curved_seams.py:39-40`).

## 2.4 The accessibility rule D4 (`sec:ws_d4`)
* D4 (`dataset_plan.md:37`): weldable seam = intersection of an exterior face pair whose dihedral bisector
  escapes to free space — one rule for all five types, encoding torch reachability.
* Three arms (README `:356-370`): intersecting pair (plane intersection clipped to the common support:
  fillets, toes); facing pair (antiparallel, bisector $\mathbf n_A+\mathbf n_B=\mathbf 0$ → emitted and rejected
  `bisector_blocked`, a free hard negative); coplanar exposed pair (same plane and normal, gap within the
  plane: edge joint and butt centreline). Then D13 (`fixture_contact`) and D22 (`matches_joint_type`);
  `weldable` = reachability, `matches_joint_type` = taxonomy, `primary` = both.
* The torch may tilt off the bisector up to a work angle; the accepted axis is stored as `approach_dir`
  (SCHEMA §2.6.1). Scene `accessibility` block (local scene): cone half-angle 30°, standoff 15 mm,
  max work angle 45°, dihedral 30–170°, min seam length 10 mm, coplanar tol 0.5 mm, cross-run tol 45°,
  `contact_tol_mm` per scene (capped by plate thickness).
* Cross-runs are told by direction (length-weighted principal axis), not length (SCHEMA §2.6.2).
* D19: nominal (stored, scored: continuous as g → 0 and independent of which part is A), root, gap-mid
  (derived arrays); grooved butts store `groove_root` (D36). Measured on the Phase 4 corpus (MANUAL §3.3,
  nb15 §3.2): root offset ≈ one gap (×0.96 acute, ×1.25 obtuse), gap-mid ≈ half; root > 3 mm from nominal on
  23 % of butt centrelines and 14 % of fillets. Phase 5 annotators clicked gap-mid on butts and root on Ts.
* Seam counts (README `:266-274`, 120 seeds/type, no fixture): T 2 in 95 / 1 in 21; corner 2 in 114 / 1 in 4;
  butt 2 in 95 / 1 in 25; lap 2 in 120/120; edge 1 in 110 / 2 in 10 (pre-6a corpus).

## 2.5 Tack rule (`sec:ws_tacks`) — `weldgen/tacks.py` ✓
* Parameters (`DEFAULT_PARAMS`, `:58-61`): tack_len_per_t 4, min 10, max 50 mm; k_max 33, d_abs 400 mm; k_min
  10; margin_per_t 2. t = the thinnest workpiece **member gauge** (min over members of thickness; for a
  swept band min(band, extrusion height), `:128-142`).
* Open seam (`:100-112`): $\ell=\mathrm{clip}(4t,10,50)$, $m=\max(2t,\ell)$, $d_{\max}=\min(33t,400)$,
  $d_{\min}=10t$, $L_{\mathrm{eff}}=L-2m$; if $L_{\mathrm{eff}}\le\ell$ → **one tack at L/2** (degenerate,
  recorded); else $n=\max(2,\lceil L_{\mathrm{eff}}/d_{\max}\rceil+1)$, and if $n>2$ and $L_{\mathrm{eff}}/(n-1)<d_{\min}$
  then $n=\max(2,\lfloor L_{\mathrm{eff}}/d_{\min}\rfloor+1)$; tacks at $s_k=m+kL_{\mathrm{eff}}/(n-1)$.
* Closed seam (`:115-125`): $n=\max(4,\lceil L/d_{\max}\rceil)$ rounded up to even; if $L/n<d_{\min}$ then
  $n=\max(4,2\lfloor L/(2d_{\min})\rfloor)$; $s_k=(\phi L+kL/n)\bmod L$, φ = first 8 bytes of
  sha256("scene_id:seam_id")/2⁶⁴ (D39); no end margin on loops.
* Weld order: open = ends first then recursive bisection (n = 5 → [0, 4, 2, 1, 3]); closed = opposite pairs
  (i, i + n/2) in bisection order over the half loop; same-class seams interleaved round-robin (classes in
  sorted order); a positional half-pitch stagger was rejected (it breaks d_max with mandatory ends).
* Which seams: primary only (weldable ∧ matches_joint_type) — "you tack what you weld". Tacks are short
  welds with `tack_length_mm`; the block stores points, seam ids, arclengths, lengths, order, params incl.
  `t_mm` and `phase_by_seam`.
* Provenance (`:38-43`): shop-practice conventions, not code limits — thickness scaling and 400 mm ceiling
  attributed to **JASS 6 via the Kobelco handbook**; 4t/50 mm tack length attributed to **EN 1011-2, not
  verified against the standard's text**; **Tomków, Sobota & Krajewski 2020** the literature anchor (five
  tacks gave the highest angular distortion, handoff `:123`). MANUAL §11: texts still to be obtained before
  citing. ISO 9692-1 Table 2 footnote b: "dimensions apply to the tacked condition" (PARAMETERS `:175-177`).
* Retro-applied to bench_phase4 and _fx on 2026-09-07 (`apply_rule_blocks.py`); the cell calls the same
  `tack_rule` unchanged (`seam_from_registration.compute_tacks`).

## 2.6 Most probable seam (`sec:ws_mps`) — `weldgen/mps.py` ✓
* Task 2 (D25/D26, `dataset_plan.md:127-152`): from one view, the one seam a coarsely positioned torch is
  at; the coarse-positioning workflow ("önce bir kaynak bölgesine git") is the stated assumption that
  makes a single answer well posed; loading conditions declined (structural, not visible).
* Rule: $\mathrm{MPS}=\arg\max_{s\ \mathrm{weldable}} (\mathrm{visible\_fraction}_s\cdot\mathrm{length}_s)$, ties by
  larger fold $|180^\circ-\mathrm{dihedral}|$ (mean for curved), then lower id; null if the best visible
  arclength ≤ 10 mm. Weldable, not primary (an off-class seam is a real weld). A pure function of
  `scene.json` (retro-applicable); emitted only under `emit_mps: true`.
* Margin $=\ell_{(2)}/\ell_{(1)}\in[0,1]$ (runner-up over winner; 0 if only one seam clears, None if none).
  Structurally pinned classes: T → 0, corner → ~1, saddle → 0 (MANUAL §10). The advisor confirmed it is a
  geometric proxy.
* approach_cone regime: rejection-sampled until a target seam is at least half visible (MANUAL §3.4).

## 2.7 Sensor model, visibility and rendering (`sec:ws_sensor`)
* Profiles (PARAMETERS §4.0): d435i b = 50 mm, f = 674 px ($=640/\tan(43.5^\circ)$ from 87° HFOV at 1280×720),
  s = 0.08 px, min-Z 280 mm [ds]; stereo_good 120 mm, 1100 px, 0.05 px, 200 mm [ours]; stereo_poor 35 mm,
  450 px, 0.15 px, 400 mm [ours]. Image 1280×720, pinhole, principal point at the centre.
* Noise: $\sigma_z(z)=s\,z^2/(f_{px}\,b)$ (d435i: 0.21 mm at 300, 0.59 at 500, 2.37 at 1000, 3.42 at 1200, 9.50
  at 2000 mm); lateral blur 0.8 px; dropout beyond 75° incidence [ours] (ICRA says "70–75°": 70° is the analytic exterior-scan grazing cone, 75° the noise dropout — state them separately ⚠); the
  digital twin's sim noise is ~10× larger (σ(1 m) ≈ 25 mm), unresolved. Noise axis = ×0/×1/×2 on subpixel and
  lateral σ (harness).
* Pose (PARAMETERS §4.1): standoff derived from a framing fraction 0.35–1.45 of the short image side,
  clamped to [max(min_z, 300), 1200] mm; elevation 15–85°; azimuth 0–360°; roll ±15°; aim jitter ±0.15 ×
  longest edge. Phase-3 occlusion statistics: occluded_fraction 0 in 40 %, partial 1.7 %, 1 in 58 %;
  1 − in_frame 0 in 69 %, partial 23 %, 1 in 8 %; azimuth dominates (each T fillet visible from ~46 % of
  views, at least one from 92 %); lap lower toe visible from 0 %; butt top centreline from 100 %, underside
  21 %; ≈ 40 % of scenes have no half-visible seam.
* Density 0.25–4 pts/mm² (spacing 2.0/1.0/0.5 mm); camera_raster density $(f_{px}/z)^2$ (D20).
* Exterior flag = reachability by a perfect multi-view scan (ICRA: "a 30-direction ray scan"); interior faces
  of the CAD cloud are truth only.
* Tier 2 (README `:756-806`): view 0 = the tier-1 camera verbatim; depth uint16 at 0.05 mm/unit; masks
  constructed (seam 2 mm wide, tack 3 mm, object ids, 255 fixture, 254 environment); materials-1.1 (F0 from
  RTR4 Table 9.2, seven alloys; CC0 ambientCG; Poly Haven HDRIs + 2 lab panoramas; 41 substrate photos);
  views 1–9 drawn (elevation 25–70°, framing 0.5–1.0, aim jitter 0.35, redrawn until a primary seam ≥ 10 %
  visible and ≥ 100 px); **twin gate**: view-0 clean depth back-projected onto the exact primitives, p99
  residual < 0.25 mm (measured ≤ 0.033 mm), coverage agreement with `visible_from_cam` ≥ **0.90** and id
  agreement ≥ **0.98** as the thresholds (`gate.py:25`; README's "≥ 0.99 / 1.00" are the measured minima
  0.995 / 1.000 of the final 12-scene gate), measured id buffer
  1.00; 12/12 strata (= one gate scene per stratum; the 720-scene benchmark itself was never rendered — the
  rendered corpus is the separate train_v1, base seed 3 000 000) and 3600/3600 train_v1 scenes; view 0
  only; it found three tier-1 ray-test bugs (sampled
  containment stepping through thin tube walls; mesh cache keyed before re-posing; exit crossings counted
  as occluders), fixed with an exact Möller–Trumbore cast and the benchmark regenerated before the paper's
  numbers.

## 2.8 Corpus and strata (`sec:ws_corpus`)
* Corpora (MANUAL §7): bench (+_fx) 250 (50/type, Phase 3/4 plates); bench6a (6a plates, used by Phase 5);
  bench6b 300 (60/class); **bench_phase4 720 = 60 per family × 12** (D28 PASS 0.34, D34 PASS, mps + tacks
  applied); bench_phase4_fx **634** = 274 plate twins (2026-09-07) + 360 curved (2026-10-19, schema 2.4.0,
  saddle `contact_mode: "line"`); annotation 20; train_v1 3600 = 12 × 300 at base seed 3 000 000 (no seed
  shared), 36 000 frames.
* bench_phase4 build (local manifest): base_seed 2 000 000, per_family 60; per source attempts → 60
  emitted: T/line 83, circle 77, ellipse 107, saddle 73, rounded_rect 73, swept 67, butt square 118,
  grooved 89, arc 64, corner 128, lap 108, edge 110 (laptop, stale: re-take on the desktop).
* Stratum names in code/notebooks: `T/line, T/circle, T/ellipse, T/saddle, T/rounded_rect, T/swept_path,
  corner/line, butt/line, butt/line_grooved, butt/arc_butt, lap/line, edge/line` (nb16/17 `ORDER_STRATA`).
* Strata are reported separately (MANUAL §4), but the ladder and noise numbers of the paper are pooled
  medians.
* D28 gate: length-weighted seam-to-free-edge angle histogram vs terminal-bin concentration; 6a pass
  2026-08-27 at 0.33 vs 0.22 uniform; bench_phase4 failed 0.45 vs 0.44 (grooved stratum rectangular) →
  PreparedPrism, rebuilt, passes 0.34. Closed curved seams exempt; open curved reported not gated;
  exclusion by provenance; arc butt reports nothing.

## 2.9 Evaluation protocol (`sec:ws_protocol`)
* Oracle ladder (MANUAL §8.1): L0 = each paper's own unpublished coarse stage from truth; L1 withholds it;
  the oracles differ (seam band is 94 k points, 100 % within 20 mm of a seam; a surface crop 331 k points,
  31 %). M6 has no L1.
* Metrics (code ✓): F1/precision/recall at τ = 3 mm, predictions densified at 1 mm, precision against the
  exact polylines, recall over truth resampled at 1 mm; symmetric Chamfer; lateral p95; matched-path RMSE/ME
  (one-to-one greedy matching; predicted path resampled at 0.25 mm; per-scene `rmse_med`); EMD.
* Harness validated on a fake oracle before any method ran: jitter σ → RMSE ≈ σ; a phantom costs precision
  and never steals a match; a miss is an unmatched row (MANUAL §8.2).
* Conditions: full_exterior and single; L0/L1 (L1 on full_exterior); noise ×1, ×2 on single; fixture twins
  (40 pairs per method in the paper table); 30 seeds for lit-ransac; Task 2 on single; extra rungs.
* Batch: ~64 h wall clock on a 16-core laptop over three runners; L1 dominated (RANSAC on a full cloud is 20×
  its oracle-assisted cost) (MANUAL §8.3); final row count on the desktop (§0.2).

## 2.10 Proposed extractors (`sec:ws_own`) — see §3.

## 2.11 WeldSet ↔ cell (`sec:ws_link`)
* Reused unchanged: the weldgen primitives via `weldgen_registry` + `geom.from_object` (each library CAD
  verified against its primitive at 0.25 mm, `weldgen_registry.py:105-120`); `weldgen.tacks.tack_rule`
  (`seam_from_registration.compute_tacks`).
* Reused with registration-tolerant changes: the plate D4 rule (`judge_registered`: "same arms as
  `weldgen.accessibility._judge`" but `clip_registered`, a pose tolerance in place of the contact tolerance,
  the mutual-visibility probe replaced by a material probe both ways, fit-up measured and penetration
  reported not rejected, `seam_from_registration.py:233-242`).
* Curved seams in the cell are computed by intersection of the registered surfaces with weldgen's curve
  library (`ellipse_from_plane_cylinder`, `saddle_from_cylinders`, `OffsetCurve`) and judged by
  `verify_curved.seam_verdict` — WeldSet's verification arm, not its curve-first constructors; the cell's bore
  minimum is 200 mm (`curved_seams.py:1-40`).
* Phase 9 (planned) closes the loop: real scans through the same registration pipeline become tier-3 twins.

## 2.12 Annotation-error experiment (`sec:ws_annotation`) — it **was run** (Phase 5)
* Design (ICRA §V; `out/annotation`): 20 scenes from `out/bench6a`, four per joint type, exported as clouds
  (no truth ships), hand-labelled in CloudCompare by an independent annotator briefed with the literature's
  protocol and blind to the truth; 30 primary truth seams; 15 mm acceptance gate; toolchain validated on a
  synthetic annotator (0.8 mm noise, one planted miss, one planted phantom); the project owner's demo pass
  excluded.
* Numbers (laptop file `out/annotation/annotation_scores.csv` 2026-09-01, identical to the ICRA macros): median
  lateral RMSE **1.415 mm**, p95 **11.81 mm**, endpoint median 4.19 mm, misses 6/30 = 0.20, phantoms 19/30 =
  0.63 per true seam; per type T 1.93, butt 3.38, corner 1.00, edge 0.42, **lap 8.50 mm**; butts 1.31 mm from
  gap-mid vs 3.38 from nominal; Ts 1.28 from root vs 1.98 from nominal; owner 1.92 mm; ratio 1.415/0.6 ≈ 2.4×.
* Open: the annotator's intra-rater repeat (accuracy vs the consistency the literature reports).
* History (per dataset_plan Phase 5, L1513–1598): toolchain built 2026-08-22 (fake annotator 0.8 mm σ,
  one miss, one phantom; the 15 mm gate exists because an uncapped matcher absorbed a planted 60 mm
  phantom); anonymous ASCII PLYs of `full_exterior`, a fifteen-minute `BRIEFING.md`, CloudCompare point-list
  picking (4–10 clicks per seam); roles `demo` / `briefed` / `unbriefed` in code, `demo` excluded from
  headlines; the briefed annotator is **Anıl**, "a briefed engineering student, a reasonable proxy for
  whoever labelled WeldJoint-PCD"; done 2026-09-01 on `out/bench6a`; aggregate curve medians gap-mid 1.16 <
  root 1.28 < nominal 1.41 mm; corner misses = the opposed seam; edge 11 extras; endpoints ~3× worse than
  lateral; the earlier pre-6a export (the author's ~2.5 mm self-annotation) archived in
  `out/annotation_archived_20260901`.

# 3. The two proposed seam extractors in full

Both are CAD-free candidates for "the project's own extractor". Neither is in the harness `REGISTRY`
(the ICRA comparison of seven is closed), but both are scored with the same harness, metrics, tolerance
(3 mm) and the same 720-scene, 12-stratum tier-1 corpus `out/bench_phase4`, as extra rows next to the
seven (`scripts/baselines/self_footprint.py:4-6`, `scripts/baselines/self_seamfind.py:5`). Both were
added on **2026-10-06** (git `d5e5875` "add self-footprint extractor", 13:48; git `7eb3be7` "Implement
self-seamfind", 16:17). Their result notebooks (16, 17) were executed on the machine that holds the final
post-audit batch: the run outputs `out/self_footprint/` and `out/self_seamfind/` do **not** exist on this
laptop, and the "best of seven" columns they print equal the ICRA `tables/coverage_full.tex` values
(e.g. T/line 0.99, corner 1.00, grooved 0.97, edge 0.89), i.e. the post-2026-09-14 desktop batch.

Neither method has been run on real scans anywhere in `weld_generator` (seam_finder.md milestone M8
"Real scans" is unticked; no real-data script exists). Only "coverage"-type runs exist: L0 flag, clean
clouds (`noise_scale = 0`), views `full_exterior` and `single`, one seed — no noise, fixture, L1 or
Task-2 runs (`scripts/run_self_footprint.py:189-193`, `scripts/run_self_seamfind.py:110-114`).

## 3.A `self-footprint` — footprint extraction from sequential scans (test1_ismail_hoca)

**Attribution.** The method lives in `Self_ideas/test1_ismail_hoca/` ("İsmail Hoca"); the draft's TODO
(ch4 l.171) asks to credit him. The code files carry no author line; the folder name and the TODO are the
only attribution evidence in the repo. `notes/thesis_direction_handoff.md:173` mentions "İsmail's PhD
thesis reportedly has related material".

**Files.**
| file | lines | role |
|---|---|---|
| `Self_ideas/test1_ismail_hoca/weld_seam.py` | 874 | the method: CLI `run(args)` + in-memory `extract(A0, C0, args)` (steps 1-10) |
| `Self_ideas/test1_ismail_hoca/make_synthetic.py` | 175 | the method's own six synthetic scenarios with GT |
| `Self_ideas/test1_ismail_hoca/test_synthetic.py` | 38 | runs the six scenarios, prints a table |
| `Self_ideas/test1_ismail_hoca/example_*.png` | — | script outputs: rolling-ball cross-sections on the 30° angled T; overview of plate-on-pipe |
| `scripts/baselines/self_footprint.py` | 137 | harness adapter (base-scan synthesis, `run`, `spec`) |
| `scripts/run_self_footprint.py` | 103 | full run over 720 scenes, resumable |
| `tests/test_self_footprint.py` | 70 | 3 tests |
| `notebooks/build_16_self_footprint.py` → `16_self_footprint.ipynb` | 340 | results notebook |

### 3.A.1 Idea
Scan the base part A alone, then the assembly A+B in weld position. Whatever the base scan does not
explain is B, so the A/B segmentation comes from the order of placement rather than from a learned
model. Where B rests on A, each part leaves a *footprint* on the other; the footprint points are slid
onto the root, clustered into seams, ordered, smoothed, framed, and a ball rolled in each cross-section
gives toes, legs and bead area (`weld_seam.py:1-27`).

### 3.A.2 Parameters (CLI defaults, `weld_seam.py:843-868`; derived values `:600-606`)
All thresholds are multiples of the voxel size `v` "so they scale with data" (`:27`).

| parameter | default | meaning |
|---|---|---|
| `--voxel` v | estimated spacing = median distance to the 4th nearest neighbour over ≤ 5000 samples (`:142-147`) | voxel size |
| `--footprint` | 3.0 (voxels) | footprint threshold `g = footprint·v + gap` (`:601`) |
| `--gap` | 0.0 (data units) | expected fit-up gap = the **target distance** of the root projection |
| `--ball-radius` | 4v | rolling-ball radius |
| `--step` | 2v | resampling step `ds` |
| `--grad-min` | 0.25 | minimum ‖∇D‖ for projection |
| `--smooth` | 0 | distance-field smoothing iterations |
| `--butt-angle` | 155° | leg angle treated as butt |
| `--corner-angle` | 50° | split paths at sharper turns |
| `--min-points` | 20 | minimum cluster size |
| `--min-length` | 10 (voxels) | minimum seam length |
| `--boundary-only` | off | project only footprint-boundary points |
| `--register` | off | ICP of the base scan into the assembly |
| `--reg-max-dist` | 6v | initial ICP correspondence radius |
| `--outliers` | off | statistical outlier removal |
| derived | `r_s = 0.75v`, `R_dir = 4v`, `r_vis = 3v` | support radius of the bounded distance; leg-direction radius; exposure radius |

### 3.A.3 The pipeline step by step, as implemented (`extract`, `weld_seam.py:587-759`)
1. **Downsample.** Voxel grid of size v, centroid per occupied voxel, for both clouds (`:150-155`);
   optional outlier removal: drop points whose mean 12-NN distance exceeds mean + 3σ (`:158-161`).
2. **Normals.** Unoriented PCA normals of A, k = 16 (eigenvector of the smallest eigenvalue of the
   neighbourhood covariance, `:164-171`).
3. **Registration (optional; OFF in every WeldSet run, the poses are exact).** Point-to-plane ICP from the
   identity, ≤ 60 iterations (`:230-263`): correspondence radius shrinking linearly from 6v to 1.5v over
   the first 60 % of iterations; matches gated by normal compatibility `|n_s·n_d| > 0.8`; Huber weights
   $w_i=\sqrt{\min(1,\delta/|r_i|)}$, $\delta = 0.75v$ (= ½·min_dist); linearised residual
   $r_i + (\mathbf p_i\times\mathbf n_i)^\top\boldsymbol\omega + \mathbf n_i^\top\mathbf t$ solved by damped
   normal equations $(\mathbf H + 10^{-4}\,\tfrac{\operatorname{tr}\mathbf H}{6}\mathbf I)\mathbf x=\mathbf A^\top(-\mathbf r\odot\mathbf w)$
   ("directions the geometry cannot observe stay put"), Rodrigues update. The docstring warns that a
   global method (FPFH + RANSAC) is needed first on real data with large offsets (`:234-235`).
4. **Segmentation of the assembly** (`:267-278`). For each assembly point c (PCA normal, k = 12) and its
   nearest base point a: $c\in A \iff (\lVert c-a\rVert<1.5v \wedge |\mathbf n_c\!\cdot\!\mathbf n_a|\ge\cos45^\circ)\ \vee\ \lVert c-a\rVert<0.75v$;
   the rest is B; B is cleaned to radius-2.5v connected components of ≥ 30 points. Abort ("Part B is
   empty") if < 20 B points (`:619-620`).
5. **Bounded distance fields** (`:183-193`, `:626-631`). For each point of A the distance to B and vice
   versa, point-to-plane inside the support radius $r_s$ of each surface sample and growing like
   point-to-point outside it, minimum over the 4 nearest samples:
   $$D_Q(\mathbf x)=\min_{i\in\mathcal N_4(\mathbf x)}\sqrt{\big((\mathbf x-\mathbf q_i)^\top\mathbf n_i\big)^2+\max\!\big(\lVert(\mathbf x-\mathbf q_i)_\perp\rVert-r_s,\,0\big)^2},\quad r_s=0.75v,$$
   so that a coplanar neighbour (butt joint) does not read zero everywhere. Optional smoothing = mean
   over the 16-NN neighbourhood.
6. **Footprints** (`:632-635`):
   $$F_A=\{\mathbf p\in A: D_B(\mathbf p)<g_{\mathrm{fp}}\},\quad F_B=\{\mathbf q\in B: D_A(\mathbf q)<g_{\mathrm{fp}}\},\qquad g_{\mathrm{fp}}=3v+g_{\mathrm{fit}}$$
   (the code calls the threshold `g`; `g_fit` is `--gap`). `--boundary-only` keeps footprint points
   with at least one non-footprint neighbour.
7. **Direction vectors and root projection** (`:202-212`, `:281-300`, `:639-656`). The surface
   gradient of the distance field is a least-squares fit $D\approx c_0+c_1u+c_2v$ over the 16-NN
   neighbourhood in the local tangent basis (ridge 10⁻⁶); points with ‖∇D‖ ≤ 0.25 are dropped. Each
   footprint point slides along $-\nabla D/\lVert\nabla D\rVert$ (tangent to its own surface) until its
   distance to the other part equals the **fit-up gap** $g_{\mathrm{fit}}$: five secant iterations on
   $f(t)=D(\mathrm{snap}(\mathbf p-t\,\hat{\mathbf d}))-g_{\mathrm{fit}}$, start
   $t_0=\operatorname{clip}((D_0-g_{\mathrm{fit}})/\lVert\nabla D\rVert,\pm3g_{\mathrm{fp}})$, every iterate
   re-snapped onto its own surface (projection onto the nearest sample's tangent plane); accepted if
   $|t|<3g_{\mathrm{fp}}$ and $|f|<0.75v$. The accepted points of both parts are the root candidates X.
8. **Exposure filter** (`:660-662`): keep candidates with an A-in-assembly point and a B point within
   $r_{vis}=3v$ ("a real seam has both parts visible nearby").
9. **Clustering** (`:303-319`, `:664-670`). Leg directions $\mathbf d_A,\mathbf d_B$ = unit vector from the
   candidate to the centroid of the part's points within $R_{dir}=4v$ (≤ 64 neighbours; valid if ≥ 5 and
   the centroid is farther than 0.1 R_dir); opening direction
   $\mathbf o=\tfrac12(\mathbf d_A[\mathrm{ok}_A]+\mathbf d_B[\mathrm{ok}_B])$, left un-normalised (it vanishes
   for a butt, differs by ≈ 1 between the two sides of a T). Clusters = connected components of
   $(\mathbf x, 2v\,\mathbf o)\in\mathbb R^6$ at radius 2.5v.
10. **Paths** (`:322-381`, `:673-687`). Per cluster with ≥ 20 points: *thin* (3 passes moving each point to
    its neighbourhood centroid, r = 2.5v, only perpendicular to the local principal direction, so ends do
    not shrink); voxel-downsample at 0.5v; *order* by the longest path of the minimum spanning tree of the
    2v-neighbourhood graph (double Dijkstra on the MST of the largest component); ≥ 4 points and length
    ≥ 10v; *closed* if the ends are < 3v apart and the length > 12v; *smoothing spline* (scipy `splprep`,
    $s=m\sigma^2$, σ = 0.3v, cubic if m > 5, periodic if closed), dense evaluation and arclength resampling
    at ds = 2v; tangents from the spline derivative.
11. **Joint frame** (`:689-709`). Leg directions at each path point, projected perpendicular to the
    tangent: $\theta=\arccos(\mathbf l_A^\top\mathbf l_B)$, bisector $\mathbf b=\widehat{\mathbf l_A+\mathbf l_B}$,
    torch axis $-\mathbf b$. If $\theta\ge155^\circ$ ("butt"), the bisector is replaced by A's local normal
    oriented away from the material side.
12. **Rolling ball** (`:415-437`, `:711-719`; fillets only). Along the bisector from the root, h sampled in
    $[0,h_{\max}]$ (160 samples), $h_{\max}=2.5r/\sin(\theta/2)$ with θ/2 clipped to [5°, 85°]; the first h
    at which $\min(d_A,d_B)\ge r$ (linear interpolation) gives the ball centre $\mathbf c=\mathbf x+h\mathbf b$;
    the toes are the nearest A and B samples to c; valid only if both distances are within 0.25 r of r
    (two-sided contact). Legs $\ell_{A,B}=\lVert\mathrm{toe}_{A,B}-\mathbf x\rVert$. Bead area
    (`:440-447`): $A_{\mathrm{bead}}=|\triangle(\mathbf x,\mathrm{toe}_A,\mathbf c)|+|\triangle(\mathbf x,\mathbf c,\mathrm{toe}_B)|-\tfrac12r^2\varphi$,
    φ = ∠(toe_A − c, toe_B − c); volume = trapezoidal integral of the area along the path (`:735-736`).
    For an ideal fillet of opening θ this reduces to $h=r/\sin(\theta/2)$, $\ell=r\cot(\theta/2)$,
    $A_{\mathrm{bead}}=r^2\cot(\theta/2)-\tfrac12r^2(\pi-\theta)$ (my derivation, not in the code — label it so
    if used).
13. **Joint class** per point (`:450-457`): θ ≥ 155° butt; ≥ 150° open_corner; ≥ 30° fillet; else narrow;
    the path's class is the majority over frame-valid points; on butt paths the ball and volume are
    disabled (`:740-742`).
14. **Corner split** (`:384-412`, `:725-726`): split where the tangent turns more than 50° between samples
    ±w apart, w = max(1, round(3v/ds)) (= 2 at ds = 2v), corners ≥ 1.5w apart; pieces shorter than 3v are
    dropped; paths sorted by length.
15. **Outputs** (CLI only): `seam_paths.csv` (per point xyz, tangent, torch axis, θ, ball centre, toes,
    legs, bead area, flags, class), `debug_points.ply`, `summary.json`, `overview.png`,
    `cross_sections.png`; with `--gt` the script's own metric `evaluate` (`:471-483`): GT densified at
    v/4, path-to-GT mean/p95/max, coverage = share of GT within 2v of the path.

### 3.A.4 How it is run on the corpus (`scripts/baselines/self_footprint.py`, `run_self_footprint.py`)
* **Assembly cloud** = the harness cloud of the condition (`full_exterior` = analytic perfect multi-view
  exterior scan; `single` = the stored camera's view), exactly what the seven get (`:15-16`).
* **Base scan is synthesised** (`base_scan`, `:68-108`), because a scene has no scan of A alone: part A =
  `object_id 0` (the base part in every joint type) rebuilt with `weldgen.geom.from_object`; an
  **independent** area-uniform sample at the scene's `density_per_mm2` (`sample_slab_surface`, RNG seed =
  scene seed + `BASE_SEED_OFFSET = 7919` + harness seed) so the two clouds never share a point (otherwise
  step 4 would be trivially exact); visibility recomputed **with A as the only occluder**: `full_exterior`
  → `exterior_scan_subsampled(xyz, nrm, [A])` (the faces B later covers are visible, as in a real
  pre-scan), `single` → `visible_mask` with the same camera (K, T_world_cam, 1280×720, min_z) and A alone;
  `base_view="complete"` keeps the whole surface (the script's own `make_synthetic` convention);
  the same stereo noise model when `noise_scale > 0` (never used in the runs).
* **No oracle** is consumed: the base scan is a physical input, so the harness `oracle` flag is ignored;
  rows carry "L0" only because they reuse the chunk definition (`:30-31`; nb16 markdown).
* **The one process parameter** is the fit-up gap: `gap="wps"` passes the scene's constructed
  `root_gap_mm` ("the gap a welding procedure specification states", the headline arm); `gap="zero"` is the
  CLI default, kept as a rung (`:33-35`, `:123`). ⚠ `wps` is therefore a truth-derived input (the exact
  constructed gap).
* Failures ("Part B is empty", near-empty cloud) return an empty prediction with a failure string and
  score 0 (`:126-128`). `MethodSpec("self-footprint", randomised=False, oracle_name=None)` (`:135-137`).
* The only change to the script: the body of `run(args)` moved into `extract()`; the six synthetic
  scenarios give byte-identical `seam_paths.csv` before and after (nb16 markdown).
* Run matrix: 720 scenes × {full_exterior, single} × {wps, zero}, seed 0, tol 3 mm, 8 workers, one CSV
  per scene, concatenated to `out/self_footprint/self_footprint.csv.gz` = **2 880 rows**.
* Tests (`tests/test_self_footprint.py`): (1) `extract` reproduces the script's synthetic T joint (2 paths,
  mean path-to-GT < 0.3, coverage > 0.95 with the script's own metric); (2) the synthesised base scan is
  part A only, shares no point with the assembly cloud, lies on A (median < 1 mm), not on B, and the
  single-view base scan is a strict subset size; (3) harness run on a saddle scene, seeds 0/1, zero
  spread, extra columns `gap_mm, n_base, voxel_mm`.
* The method's own synthetic scenes (`make_synthetic.py`): tjoint (A 200×150×10 plate, B 8×120×80 stem),
  tjoint_angled (B leaning 30°), lap (6 mm plates), butt (two 100×150×6), rod_on_plate (Ø60 rod: circular
  seam r = 30), plate_on_pipe (plate on a Ø120 pipe: curved seam on a curved base); grid sampling at 1 mm,
  noise σ = 0.05 mm, contact faces removed (points within 0.3·spacing inside the other part); GT seams are
  **closed rectangle loops including the end runs across the stem**, unlike WeldSet, whose truth rejects
  cross-runs (SCHEMA §2.6.2) — one reason for the extra paths it returns on WeldSet.

### 3.A.5 Results recorded (desktop run, committed in nb16, executed 2026-10-06 — confirm against desktop `out/self_footprint/self_footprint.csv.gz` before use)
Provenance check: the "best of seven" column printed by nb16/nb17 cannot come from this laptop's
`out/phase4_batch/phase4_batch.csv.gz` (184 680 rows, 2026-09-09, pre-audit): recomputed here it gives
e.g. grooved 0.03, T/rounded_rect 0.62, T/saddle 0.85, butt/arc 0.68 (full view) instead of nb16's 0.97,
0.75, 1.00, 0.95, and the local batch has lit-modelreg only on the five plate strata. nb16's values equal
the ICRA `tables/coverage_full.tex` (post-audit desktop batch).
*Walk-through, one T/circle scene, full view* (nb16 cell 5): v = 1.03 mm, g_fp = 3.62 mm, ball r =
4.13 mm; footprints A 1429, B 590; root candidates 1603 → 1602 exposed; **1 path, fillet, L = 162.3 mm,
open**, θ ≈ 91° (the truth is a closed ring: a partial recovery).

*Round 1* (first scene per stratum, gap = wps, seeds 0 and 1; cell 7): 48 rows in 157 s; **zero seed
spread on all 24 scene × condition cells** (deterministic). F1 full / single: T/line 0.69/0.77, T/circle
0.46/0.41, T/ellipse 0.71/0.34, T/saddle 1.00/0.31, T/rounded_rect 0.74/0.28, T/swept_path 0.52/0.66,
corner 0.68/1.00, butt/line 1.00/0.47, butt/line_grooved 0.67/0.00, butt/arc 0.97/0.77, lap 0.21/0.00,
edge 0.67/0.00.

*Round 2* (all 720 scenes; cell 9): 2 880 rows, **16 failures** (B empty / near-empty cloud), scored 0.
Headline arm gap = wps, scene-level medians per stratum (n = 60 each):

| stratum | F1 full | F1 single | prec full | prec single | rec full | rec single | RMSE full (mm) | RMSE single (mm) |
|---|---|---|---|---|---|---|---|---|
| T/line | 0.80 | 0.60 | 0.98 | 1.00 | 0.68 | 0.43 | 0.66 | 0.60 |
| T/circle | 0.80 | 0.42 | 1.00 | 1.00 | 0.71 | 0.27 | 0.41 | 0.31 |
| T/ellipse | 0.78 | 0.53 | 1.00 | 1.00 | 0.75 | 0.38 | 0.40 | 0.30 |
| T/saddle | 1.00 | 0.66 | 1.00 | 1.00 | 1.00 | 0.49 | 0.56 | 0.39 |
| T/rounded_rect | 0.78 | 0.42 | 0.90 | 1.00 | 0.83 | 0.27 | 0.38 | 0.34 |
| T/swept_path | 0.93 | 0.61 | 1.00 | 1.00 | 0.87 | 0.44 | 0.46 | 0.77 |
| corner/line | 0.75 | 0.79 | 1.00 | 1.00 | 0.60 | 0.65 | 0.64 | 0.51 |
| butt/line | 0.88 | 0.51 | 0.98 | 0.99 | 0.80 | 0.35 | 0.91 | 1.61 |
| butt/line_grooved | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 4.68 | 3.11 |
| butt/arc_butt | 0.67 | 0.94 | 0.50 | 1.00 | 1.00 | 0.90 | 0.53 | 0.40 |
| lap/line | 0.72 | 0.48 | 0.86 | 1.00 | 0.74 | 0.31 | 0.90 | 0.73 |
| edge/line | 0.44 | 0.00 | 0.85 | 0.00 | 0.31 | 0.00 | 1.15 | 6.99 |

*Against the best of the seven* (L0, clean, same scenes, cell 11): best-of-seven F1 full/single —
T/line 0.99/0.67, T/circle 0.97/0.50, T/ellipse 1.00/0.59, T/saddle 1.00/0.66, T/rounded_rect 0.75/0.44,
T/swept_path 0.99/0.63, corner 1.00/1.00, butt/line 0.99/0.96, grooved 0.97/0.52, arc 0.95/0.93, lap
0.87/0.58, edge 0.89/0.43. self-footprint ties or exceeds it on T/saddle (both views), T/rounded_rect
(full: 0.78 vs 0.75) and butt/arc (single: 0.94 vs 0.93). Note that "best of seven" includes M6
`lit-modelreg`, which is handed the model.

*Matched-path RMSE, full view, median per stratum (mm), all eight entries* (cell 13):

| stratum | ransac | regiongrow | lobb | ppf | pcaslice | modelreg | quadric | self-footprint |
|---|---|---|---|---|---|---|---|---|
| T/line | 0.75 | 2.16 | 1.08 | 1.53 | 2.21 | 0.70 | 1.46 | **0.66** |
| T/circle | 7.13 | 0.62 | 2.43 | 9.78 | 3.48 | 2.17 | 2.53 | **0.41** |
| T/ellipse | 10.36 | 0.55 | 1.00 | 9.22 | 4.81 | 1.43 | 2.27 | **0.40** |
| T/saddle | 12.76 | 1.53 | 1.48 | 1.88 | 6.43 | 0.77 | 1.67 | **0.56** |
| T/rounded_rect | 6.71 | 0.58 | 2.15 | 6.48 | 4.71 | 2.79 | 3.70 | **0.38** |
| T/swept_path | 1.41 | 0.62 | 1.44 | 4.58 | 4.39 | 0.71 | 1.64 | **0.46** |
| corner/line | 3.62 | 3.43 | **0.45** | 4.33 | 3.61 | 0.73 | 1.07 | 0.64 |
| butt/line | 2.04 | 3.03 | 1.31 | 3.90 | 3.79 | 1.15 | 1.26 | **0.91** |
| butt/line_grooved | 4.63 | 4.33 | 4.71 | 5.69 | 5.47 | **0.96** | 4.58 | 4.68 |
| butt/arc_butt | 2.45 | 2.38 | 0.84 | 7.27 | 5.98 | 1.65 | 1.19 | **0.53** |
| lap/line | 32.73 | 2.91 | **0.88** | 5.68 | 2.76 | 2.72 | 1.46 | 0.90 |
| edge/line | 22.47 | 1.69 | 1.17 | 2.27 | 6.02 | 5.05 | 1.56 | **1.15** |

Lowest of eight on 9 of 12 strata (confirmed from the table). RMSE is over matched seams only, so it
must be read with recall.

*The gap rung* (cell 15), median F1 (full wps / full zero / single wps / single zero): T/line
0.80/0.58/0.60/0.40; T/circle 0.80/0.74/0.42/0.28; T/ellipse 0.78/0.77/0.53/0.38; T/saddle
1.00/0.84/0.66/0.50; T/rounded_rect 0.78/0.71/0.42/0.32; T/swept_path 0.93/0.79/0.61/0.43; corner
0.75/**0.00**/0.79/**0.00**; butt/line 0.88/0.45/0.51/0.00; grooved 0/0/0/0; arc 0.67/0.61/0.94/0.65; lap
0.72/0.48/0.48/0.23; edge 0.44/0.24/0.00/0.00. Mechanism: with gap = 0 the projection seeks a distance
of zero that a gapped joint never reaches, so candidates fail `|f| < 0.75v`.

*Thin sheets* (cell 17), median F1 by `t_min < 3v` (thin) vs not, headline arm: full — T/line 0.75
(48) vs thin 0.84 (12); butt 0.90 (52) vs 0.71 (8); corner 0.64 (48) vs 0.89 (12); edge 0.34 (5) vs 0.45
(55); **lap 0.85 (42) vs 0.42 (18)**; single — T/line 0.43 vs 0.79; butt 0.49 vs 0.61; corner 0.71 vs 0.83;
edge 0.25 (6) vs 0.00 (54); **lap 0.57 vs 0.11**.

*The notebook's reading* (nb16 last markdown), checked against its own tables:
* ⚠ "a median F1 of 0.75–1.00 on 10 of the 12 strata" — the table gives ≥ 0.75 on **8** strata (the six
  T strata, corner, butt/line); lap 0.72 and arc 0.67 are below. Correct form: "0.67–1.00 on 10 of 12" or
  "≥ 0.75 on 8 of 12".
* ✓ precision ≥ 0.85 everywhere except arc butt (0.50) and the grooved butt (0.00).
* ✓ lowest matched RMSE on 9 of 12 strata (0.38–1.15 mm); lit-lobb lower on corner (0.45 vs 0.64) and lap
  (0.88 vs 0.90), lit-modelreg on the grooved butt.
* ≈ "below the best of the seven by 0.06–0.45 (0.97 on grooved)" holds for the full view; in the single
  view the gaps are 0.02–0.52 (T/rounded_rect 0.42 vs 0.44, T/swept 0.61 vs 0.63).
* ⚠ "the curved T strata lose 0.01–0.16 with gap = 0" holds for the full view; single view T/swept_path
  loses 0.18 (0.61 → 0.43).
* ✓ corner 0.75 → 0.00, square butt 0.88 → 0.45 (single 0.51 → 0.00); ✓ lap thin split; ✓ 16 of 2 880 rows
  are failures.

### 3.A.6 Limitations (measured or structural)
1. **Grooved butt = 0 in both views.** The stored seam is the centreline between the two top faces, which
   an ISO 9692-1 groove holds up to ~25 mm apart; no top-face point is within g_fp of the other part, so
   there is no footprint there; the candidates that exist come from the groove's root faces, a groove depth
   below the truth (matched RMSE 4.7 mm) (nb16).
2. **Arc butt, full view: precision 0.50.** It also returns the line on the strips' opposite face
   (median 4.5 paths for 1 truth seam); in the single view that face is hidden and F1 = 0.94 (nb16).
3. **Edge joints, single view: 0.** 55 of 60 edge scenes have 1–2 mm plates (< 3v); their edge faces are a
   few samples tall (nb16).
4. **Thin laps** (t_min < 3v): F1 0.85 → 0.42 full, 0.57 → 0.11 single (the upper plate's edge face, only
   t tall, is the only B surface next to the root).
5. **Needs the fit-up gap.** gap = 0 deletes corners and gapped butts. In WeldSet the "wps" gap is the
   constructed truth value; on a real cell it would be a WPS nominal, not the realised gap.
6. Closed rings are often recovered as open partial paths (T/circle full recall 0.71; the walk-through
   returned a 162 mm open arc of a closed ring).
7. The rolling-ball outputs (toes, legs, bead area, volume, torch axis) are **not scored** — WeldSet has
   bare joints and no bead truth; only the seam polyline enters F1/RMSE.
8. Registration was off (exact poses); on real data the base scan must be registered into the assembly
   (local ICP only; a global initialisation is needed for large offsets).
9. Base scan noise-free and exact in all runs (noise_scale 0); no noise / fixture / Task-2 runs exist.

## 3.B `self-seamfind` — matching seeds + sphere-tracing walkers on labelled clouds (test2_cagdas)

**Files.** `Self_ideas/test2_cagdas/seam_finder.md` (320 lines, the plan, header "Oct 6, 2026 · @Çağdaş
Güven"); package `Self_ideas/test2_cagdas/seamfind/` (README 98, `config.py` 93, `geom.py` 133,
`access.py` 142, `seeds.py` 114, `walk.py` 142, `assemble.py` 139, `pipeline.py` 245); adapter
`scripts/baselines/self_seamfind.py` (62); runner `scripts/run_self_seamfind.py` (102); tests
`tests/test_self_seamfind.py` (122, 6 tests); `notebooks/build_17_self_seamfind.py` → `17_self_seamfind.ipynb`.

### 3.B.1 The plan (`seam_finder.md`) — what it proposed
* Inputs: point clouds A and B with known labels in one frame (mm); optional camera poses; a torch model
  (nozzle radius, work/travel-angle ranges, standoff). Outputs per seam: ordered root polyline, toe lines on
  A and B, per point gap, joint angle, joint type, torch direction and feasible-orientation interval, a
  weldable/not-weldable flag per span. Assumptions: labels correct away from the joint, rigid stationary
  parts, **gap ≤ ~3 mm** (larger = no seam), synthetic first then UR5e RGB-D (`:9-27`).
* Stage 0 synthetic data: six families (T 6 mm stem 90°; angled 30/45/60°; lap; butt with optional V; pipe
  Ø60 on plate; corner box with 3 plates) and perturbation sweeps — gap 0/0.5/1/2/3 mm, spacing
  0.5/1/2/4 mm, normal noise σ 0/0.25/0.5/1/2 mm ("RGB-D at 0.5 m ≈ 1–2 mm"), outliers 0/1/5 %, partial
  view from 1/2/4 cameras, label flips within 2 mm of the joint at p = 0/0.1/0.3 (`:35-65`). **In the
  implementation Stage 0 *is* the WeldSet generator** (README `:6-8`).
* Stage 1: per-part voxel downsample (h = 1 mm), PCA normals k = 20–30 per part ("labels keep normals
  clean at the corner"), orientation (synthetic: mesh; real: toward the observing camera), KD-trees, MLS
  projection Π (Gaussian-weighted plane within r_mls = 3h), $d_B(\mathbf x)=\lVert\mathbf x-\Pi_B(\mathbf x)\rVert$,
  $\mathbf u_B=(\Pi_B(\mathbf x)-\mathbf x)/d_B$ (`:67-81`).
* Stage 2: 2a cone prefilter (M = 16–32 rays, 60° half-angle, ray-march on the merged cloud's unsigned
  distance, keep if free-ray fraction ≥ τ_vis = 0.3); 2b HPR (Katz 2007) optional; 2c torch-cone
  capsule test on seam points (`:83-108`).
* Stage 3: FPS N = M = 1000–2000, N×M distance matrix, **mutual NN** with $D_{ij}<\tau_d=\mathrm{gap}_{\max}+2h$,
  θ_n per seed; joint type by θ_n (≈ 90° fillet, ≈ 0° butt, ≈ 180° facing = filter failure); ablations
  one-way NN, partial OT, seed density (`:110-129`).
* Stage 4 walker pseudocode (`:131-150`): $x\leftarrow\Pi_A(x+\alpha\,d\,\mathbf t/\lVert\mathbf t\rVert)$ with
  α = 0.9, ε = 0.05h, tol = 0.01h, K_max = 50; outputs toes, gap, root point (fillet: intersection of the two
  MLS tangent planes at the toes with the plane through the seed normal to the seam; gaps: toe midpoint),
  θ_n and $\theta_r=\arcsin\big((1-\bar r)/\alpha\big)$ from the mean step ratio (valid on planar faces);
  tests: iterations ≈ $\log(0.05h/d_0)/\log(1-\alpha\sin\theta)$, θ_r within 3°, gap error < 0.25 mm at
  1 mm, $d_k$ non-increasing (`:164-169`).
* Stage 5 assembly: drop non-converged seeds and |θ_n − θ_r| > 10°; DBSCAN (eps 3–5h, min 5) then split by
  normals; MST longest path; spline; interpolate toes/gap/angle; torch axis $-\widehat{\mathbf n_A+\mathbf n_B}$
  tilted by the travel angle; 2c → weldable spans (`:171-186`).
* Stage 6 (optional) rolling-ball spine: centres with $d_A(\mathbf c)=r,\ d_B(\mathbf c)=r$, predictor–corrector
  (tangent $\widehat{\nabla d_A\times\nabla d_B}$, Gauss–Newton correction in the normal plane, adaptive Δs,
  start at the root offset by $r/\sin(\varphi/2)$, $\varphi=180^\circ-\theta_n$) (`:188-208`).
* Evaluation plan: root error, completeness (truth within **1 mm**), precision (within 1 mm), toe/gap/angle
  errors, joint-type and accessibility accuracy, runtime at 50k/200k points; 10 seeds; sweeps; ablations
  (mutual vs one-way vs partial OT; seeds vs walkers; raw vs MLS; with/without 2a; per-part vs merged
  normals); baselines (RANSAC planes, contact band, curvature region growing); real data (5–10 UR5e
  joints, GT by CAD ICP or touch sensing 10–20 points); novelty-check queries; milestones M1–M9 (none
  ticked); risks and open questions (`:210-321`).

### 3.B.2 Parameters as implemented (`seamfind/config.py:132-194`, printed in nb17 cell 1)
Lengths relative to the point spacing h unless physical.

| group | parameter = default | plan value / note |
|---|---|---|
| Stage 1 | `h_mm=None` (estimated per part: median 4th-NN distance) | plan h = 1 mm; corpus densities 0.25–4 pts/mm² → h = 0.5–2 mm |
| | `k_normals=25`, `r_mls_h=3.0`, `mls_normal_gate=0.5` (cos 60°), `support_h=0.75`, `remove_outliers=False` (k 12, 3σ) | gate new: 1 mm plates put both faces in one 3h ball |
| Stage 2a | `prefilter=True`, `n_rays=24`, `cone_half_deg=60`, **`tau_vis=0.04`** (≥ 1 free ray of 24), `ray_start_h=0.05`, `ray_hit_h=0.5`, `ray_free_mm=40`, `ray_max_steps=60` | plan τ_vis = 0.3 |
| Stage 3 | **`gap_max_mm=5.0`**, `tau_d_extra_h=2.0` (τ = gap_max + 2h), **`seeding="oneway"`**, `n_fps=1500`, `coplanar_pairs=True`, `coplanar_tau_mm=25`, `coplanar_deg=20`, `facing_deg=150`, `coplanar_tol_mm=0.5` | plan gap ≤ 3, mutual NN on FPS |
| Stage 4 | `alpha=0.9`, `eps_h=0.05`, `tol_h=0.01`, `k_max=50`, **`crease_free="interior"`**, `crease_fit_h=6` | as plan + refit |
| Stage 5 | **`angle_check=False`**, `angle_disagree_deg=10`, `dbscan_eps_h=4`, `dbscan_min=5`, `dbscan_eps_spacing=3`, `normal_eps=0.35` (~20°), `suppress_toes=True`, `min_seam_mm=10`, `cross_runs=True`, `cross_run_tol_deg=45`, `close_h=3`, `ds_mm=2` | plan has the angle test on |
| Stage 2c | `torch_check=True`, `torch_half_deg=30`, `torch_standoff_mm=15`, `max_work_deg=45`, `travel_deg=(-15,0,15)`, `work_step_deg=15`, `bore_check=True`, `bore_min_diameter_mm=80`, `output="weldable"`, `dihedral_min_deg=30`, `dihedral_max_deg=170` | the generator's own accessibility block, not an invented nozzle |

### 3.B.3 The pipeline step by step, as implemented (`seamfind/pipeline.py:176-360`)
0. If either part has fewer than max(k_normals, 10) points → no seam (a part out of view; 3 single-view
   scenes see no point of A at all, nb17).
1. **Stage 1, per part** (`geom.py:38-133`). h_A, h_B estimated; h = max(h_A, h_B) for thresholds; each part
   voxel-downsampled at its own h (normal hints averaged per voxel). `PartSurface`: PCA normals (k = 25);
   **edge-aware normal** = the normal of the flattest neighbourhood (minimum $\lambda_0/\sum\lambda$) among
   the 10 nearest (`:55-62`); orientation sign from the stored surface normals (`full_exterior`, the plan's
   synthetic rule) or toward the camera position (`single`, the real-scan rule) — only the sign is taken
   from outside, the normals are always PCA estimates; one re-fit with the **oriented-normal gate**
   (neighbours with $\mathbf n_j^\top\mathbf n_i>0.5$ only). **MLS projection** $\Pi(\mathbf x)$ (`:91-124`):
   neighbours within $r_{mls}=3h$ of x's nearest sample, gated by normal agreement > 0.5, Gaussian weights
   (σ = r_mls/2) about that sample, weighted-PCA plane, foot = orthogonal projection of x; **support
   test**: the foot is "on the patch" if within 0.75h of a sample. **Closest point of the finite patch**
   $c(\mathbf x)$ = the MLS foot if on support, else the nearest sample (`:126-133`).
2. **Bands** (`pipeline.py:196-211`). $\tau=\mathrm{gap}_{\max}+2h$ (gap_max = 5 mm). Candidates = points of
   each part within τ of the other part; boundary masks (neighbourhood centroid displaced in the tangent
   plane by > 0.35 of the 16-NN radius, same-face neighbours only, `seeds.py:73-81`); coplanar candidates
   = boundary points within 25 mm of the other part.
3. **Stage 2a prefilter** (`access.py:178-208`; applied to the band and coplanar candidates only — "every
   point that can be in a kept pair is within τ of the other part", `pipeline.py:198-199`). 24 rays per
   point: the normal + a ring at 30° (7 rays) + a ring at 60° (16 rays), fixed azimuth phase
   (deterministic); origin $\mathbf p+0.05h\,\mathbf n$; sphere tracing on the merged cloud with step
   $\max(\text{distance to the nearest point},0.5h)$; a **hit** only if that distance < 0.5h **and** the
   nearest point lies ahead of the ray ($(\mathbf q-\mathbf x)^\top\mathbf d>0$); a ray is free after 40 mm
   (or unresolved after 60 steps). Points with free fraction < τ_vis = 0.04 are removed (i.e. kept if any of
   the 24 rays escapes — "the generator exterior rule: some ray in the grazing cone escapes").
4. **Stage 3 seeds** (`seeds.py`, `pipeline.py:226-252`). Default **one-way NN**: every A candidate within τ
   of B paired with its nearest B candidate and every B candidate with its nearest A candidate, pairs
   deduplicated (`oneway_nn`, `:50-60`). Alternatives: `"dense"` = mutual NN on all points via KD-trees
   (identical to an N×M matrix over all points; the `plan` arm), `"fps"` = the plan's literal FPS (1500) +
   N×M matrix (unused in the runs). **Stage 3b coplanar pairs** (new; `:84-94`): mutual NN among the
   coplanar boundary candidates within 25 mm, kept if $\mathbf n_A^\top\mathbf n_B>\cos20^\circ$ and
   $|(\mathbf b-\mathbf a)^\top\widehat{\mathbf n_A+\mathbf n_B}|<\max(0.5\,\mathrm{mm},0.5h)$; added when their A
   point is not already seeded. Per seed $\theta_n=\arccos(\mathbf n_A^\top\mathbf n_B)$ (outward normals);
   class (`:101-114`): θ_n < 20° → coplanar (butt/edge) unless the plane offset exceeds the tolerance →
   "parallel" (dropped: e.g. the two bottom faces of a lap); θ_n > 150° → "facing" (dropped); otherwise
   "fillet" (T, corner or lap toe, which θ_n alone cannot tell apart).
5. **Stage 4 walkers** (`walk.py:137-186`), vectorised over all seeds; one walker on A from $\mathbf a_i$
   toward B and a mirror walker on B. With $\mathbf x_0=\Pi_A(\mathbf a_i)$, at each step
   $$\mathbf q_k=c_B(\mathbf x_k),\quad d_k=\lVert\mathbf x_k-\mathbf q_k\rVert,\quad \mathbf u_k=\frac{\mathbf q_k-\mathbf x_k}{d_k},\quad \mathbf t_k=\mathbf u_k-(\mathbf u_k^\top\mathbf n_k)\mathbf n_k,$$
   $$\boxed{\mathbf x_{k+1}=\Pi_A\big(\mathbf x_k+\alpha\,d_k\,\mathbf t_k\big)=\Pi_A\big(\mathbf x_k+\alpha\,(\mathbf I-\mathbf n_k\mathbf n_k^\top)(\mathbf q_k-\mathbf x_k)\big)},$$
   i.e. the step is α times the **tangential component** of the vector to B (length $\alpha d_k\lVert\mathbf t_k\rVert$),
   **not** $\alpha d_k$ along $\mathbf t_k/\lVert\mathbf t_k\rVert$ as in the plan: "with B hovering a gap above A
   the full distance exceeds the in-plane distance to B's foot and the plan's step overshoots the toe — its
   own never-overshoot test fails without this" (README row 11b; `walk.py:118-121`, `:171-174`). Stop
   (converged) when $d_k<\varepsilon=0.05h$ (touching), $d_{k-1}-d_k<\mathrm{tol}=0.01h$ (stalled at a gap),
   $\lVert\mathbf t_k\rVert<10^{-3}$ (B straight along the normal: at the toe), or the next foot leaves A's
   support (the walker reached A's patch boundary, "which IS the toe"); K_max = 50. Outputs: toes
   $\mathbf T_A,\mathbf T_B$, their MLS normals, $d$ history.
6. **Step-ratio angle** (`walk.py:189-196`): $e_k=d_k-d_{\mathrm{final}}$ (gap-corrected, since a gapped walker
   converges to d = g), $r_k=e_{k+1}/e_k$ over valid steps, $\bar r$ = mean if ≥ 2 valid ratios (else NaN),
   $$\theta_r=\arcsin\!\Big(\operatorname{clip}\big(\tfrac{1-\bar r}{\alpha},0,1\big)\Big),$$
   averaged over the A and B walkers. Derivation (planar faces at angle θ, steps along A toward the
   line): $d_{k+1}=d_k(1-\alpha\sin\theta)$, hence $\bar r=1-\alpha\sin\theta$; iterations to reach ε
   $\approx\log(\varepsilon/d_0)/\log(1-\alpha\sin\theta)$ (plan test); a 90° joint converges in one or two
   steps at α = 0.9, so θ_r is often undefined there (`walk.py:125-128`). θ_r is planar-only (plan).
7. **Gap** $=\lVert\mathbf T_A-\mathbf T_B\rVert$ (toe to toe, `pipeline.py:265`).
8. **Crease-free supporting planes** (`walk.py:221-256`, `pipeline.py:266-281`; `crease_free="interior"`):
   for each toe that stopped *inside* its patch (not at a part edge), refit the joint face's plane by PCA on
   own points within 6h of the toe that are farther than gap + 2h from the other part and on the same face
   (normals agreeing > 0.5 with the far point nearest the toe), ≥ 8 points; the toe is projected onto the
   fitted plane, the normal replaced, the class re-evaluated. Reason: "at sub-mm gaps the exterior scan
   keeps a 1–2-sample strip of the buried face with unrecoverable normals; D19 truth is the extended-plane
   intersection" (README row 12).
9. **Root point** (`walk.py:199-218`): fillet → the point of the intersection line of plane(A at T_A) and
   plane(B at T_B) closest to the seed midpoint $\mathbf s=\tfrac12(\mathbf a_i+\mathbf b_j)$: with
   $\mathbf d=\mathbf n_A\times\mathbf n_B$, $h_a=\mathbf n_A^\top\mathbf T_A$, $h_b=\mathbf n_B^\top\mathbf T_B$,
   $\mathbf p_0=\big(h_a(\mathbf n_B\times\mathbf d)+h_b(\mathbf d\times\mathbf n_A)\big)/\lVert\mathbf d\rVert^2$,
   $\mathbf r=\mathbf p_0+((\mathbf s-\mathbf p_0)^\top\hat{\mathbf d})\hat{\mathbf d}$ — this is SCHEMA D19's
   `nominal`; coplanar → $\tfrac12(\mathbf T_A+\mathbf T_B)$, the gap centreline. Dihedral = 180° − θ_n
   (from the refitted normals).
10. **Seed filter** (`pipeline.py:287-293`): keep converged (both walkers) seeds whose root lies within
    τ + 2h of the seed midpoint and whose class is not facing/parallel; the θ_n vs θ_r test (|fold(θ_n) − θ_r|
    > 10°) only when `angle_check` (off by default).
11. **Stage 5 assembly** (`assemble.py`, `pipeline.py:301-334`): DBSCAN on root positions with
    $\varepsilon_c=\max(4h,\,3\times\text{median seed spacing})$, min 5; each cluster split by a second DBSCAN on
    the 6-D normal pair $(\mathbf n_A,\mathbf n_B)$ (eps 0.35 ≈ 20°, min 3) and re-split by position; order by the
    MST longest path (edges within $3\max(h,\varepsilon_c/3)$), every point assigned to its nearest path vertex
    and the vertex replaced by the mean of its points (removes the zig-zag of a one-way band); ≥ 4 points,
    length ≥ 10 mm; closed if the ends are < 3h apart and length > 12h; smoothing σ = max(RMS residual to an
    11-point moving average, 0.05h); `splprep` with $s=m\sigma^2$ (periodic if closed), resampled at Δs = 2 mm.
    Per resampled point (nearest root's seed): toes, gap, θ_n, θ_r, dihedral, $\mathbf n_A,\mathbf n_B$,
    bisector $\widehat{\mathbf n_A+\mathbf n_B}$, torch axis $= -$bisector; class coplanar if > 50 % of the
    points' seeds are coplanar.
12. **Generator rules** applied to the seam list: **cross-run rule** (SCHEMA §2.6.2; `assemble.py:119-139`):
    joint direction = principal eigenvector of $\sum_i L_i\hat{\mathbf c}_i\hat{\mathbf c}_i^\top$ over open
    seams with chord/length > 0.9; open seams with chord/length > 0.6 whose chord is > 45° off it are
    dropped; **toe suppression** (ISO 17659 / generator `toe_of_centreline`; `:106-116`): fillet seams with
    > 60 % of their points within (median coplanar gap/2 + 2h) of a coplanar seam are dropped.
13. **Stage 2c torch check** (`access.py:255-275`, `:234-252`; `pipeline.py:344-355`): at each seam point,
    7 work angles (−45…45° step 15) about the bisector in the plane ⊥ tangent × 3 travel angles (−15, 0,
    15°); a direction **u** is feasible if no cloud point beyond 2h and within the 15 mm standoff lies inside
    its 30° cone ($\mathbf Q^\top\mathbf u>\cos30^\circ\lVert\mathbf Q\rVert$); `frac` = feasible share of the
    21 directions. **Bore confinement** (the generator's `bore_min_diameter_mm = 80`): at heights 15/35/55 mm
    above the seam along $\mathbf n_A$ and $\mathbf n_B$, a ring of 12 rays parallel to that face is marched to
    40 mm; confined if, for either face, some ring closes (every ray hits). Weldable ⇔ frac > 0 ∧ (coplanar ∨
    30° ≤ dihedral ≤ 170°) ∧ not confined. Output (default) = **weldable spans** ≥ 10 mm only.
14. Stage 6 (rolling-ball spine) is **not implemented** (README `:8`, pipeline docstring).

**The 17 changes from the plan** (README table, `seamfind/README.md:39-59`; each is a parameter, the plan
stays runnable as arm `plan`): (1) gap_max 3 → 5 mm (ISO 9692-1 gap ranges plus the ISO 5817 below-D tail:
drawn gaps reach 4.9 mm on corners); (2) h estimated per part; (3) oriented-normal gate + flattest-neighbour
normal (1 mm plates); (4) FPS + N×M → NN on all points via KD-trees (1500 samples on a 200×266 mm plate are
~10 mm apart, more than τ_d); (5) mutual → one-way NN default (mutual pairs one per 2–3 mm per side; smoke
T-line F1 0.42 → 0.98); (6) θ_r agreement off (planar-only; drops most tube/swept seeds); (7) ray hit only if
the near point is ahead; (8) τ_vis 0.3 → ≥ 1 free ray (a 217 mm deep bore's floor seam has ~1/24 free rays and
is weldable in the truth); (9) same-plane test for "coplanar" (a lap's two bottom faces are parallel but one
plate thickness apart); (10) Stage 3b coplanar pairs over 25 mm (a groove opens the top faces up to
$b+2t\tan\beta\approx25$ mm); (11) support test on the walker (an MLS plane is infinite: it carried the walker
across a butt gap and "extrapolated a stem 1 mm down to the plate, gap read 0.03 for 1 mm"); (11b) tangential
step; (12) crease-free refit; (13) DBSCAN eps ≥ 3× seed spacing, normal split by a second DBSCAN; (14)
path vertex = mean of the seeds projected onto it; (15) toe suppression; (15b) cross-run rule; (16) generator
cone + bore ring instead of a capsule; (17) output = weldable spans.

⚠ Minor code-doc inconsistency: `seeds.py:6` says `seeding="dense"` is the default; `config.py:155`
(and README row 5, nb17 cell 1) make **`"oneway"`** the default.

### 3.B.4 How it is run on the corpus (`scripts/baselines/self_seamfind.py`, `run_self_seamfind.py`)
* Labels: the condition's cloud split by `object_id` (A = 0, B = 1) — the **"objects" oracle**, the same
  input lit-lobb gets at L0; `MethodSpec(..., oracle_name="objects")`; `oracle=False` raises ("there is no
  L1 arm") (`self_seamfind.py:8-16`, `:33-35`, `:60-62`).
* Normal sign: `full_exterior` → stored surface normals; `single` → camera position $T_{wc}[:3,3]$.
* Gap bound: 5 mm (`gap="iso"`); `gap="wps"` (root_gap + 0.5 mm) exists but is not used in the runs.
* Extra output columns: `h_mm`, `n_seams_cop`, `gap_est_med`, `dihedral_med`, `weldable_frac`, all
  stage counts `c_*` and timings `t_*`.
* Arms (`run_self_seamfind.py:96-98`): `default` = config defaults; `nocrease` = `crease_free=False`;
  `plan` = `seeding="dense"` (mutual NN on all points), `angle_check=True`, `tau_vis=0.3`, `crease_free=False`.
  720 scenes × 2 views × 3 arms, seed 0, tol 3 mm, noise 0 → `out/self_seamfind/self_seamfind.csv.gz` =
  **4 320 rows, 0 crashes**.
* Tests (`tests/test_self_seamfind.py`): Stage 1 MLS on a plane (foot error < 0.1h, normals < 1°, all on
  support); per-part normals at a 90° corner within 3° at 1h; Stage 4 walker $d_k$ non-increasing, all
  converged, gap error < 0.25 mm at a 1 mm gap; Stages 3–5 T joint (gap 0.5) → exactly 2 seams at the
  nominal line (x = ±t/2, z = 0 within 0.5 mm), length 60–85 mm, dihedral 90 ± 5°; pipe on plate → one
  closed seam at radius 30 ± 0.3 mm; one deterministic harness run on a T/line scene (single, F1 > 0.8).

### 3.B.5 Results recorded (desktop run, committed in nb17, executed 2026-10-06 — confirm against desktop `out/self_seamfind/self_seamfind.csv.gz` before use)
*Walk-through, T/line, full view* (cell 3): n_A 33 360, n_B 70 884, bands 3 686 / 2 071, 5 577 seeds (0 from
3b), all converged and kept, seed spacing 0.022 mm, h = 0.79 mm; timings (s) stage1 1.14, bands 0.68,
stage2a 2.65, stage3 0.006, stage4 2.54, stage5 0.68, stage2c 0.48; six fillet seams returned (L 132,
129, 128, 46, 38, 19 mm), gap estimates 0.63–1.46 mm against a true 0.17 mm, dihedral 98°/82° against a
design included angle of 96° (fillets 96° and 84°).

*Round 1* (first scene per stratum, default, seeds 0/1; cell 5): 48 rows in 239 s; zero spread on all 24
cells. F1 full/single: T/line 0.98/0.99, T/circle 0.76/0.42, T/ellipse 0.70/0.42, T/saddle 0.93/0.33,
T/rounded_rect 0.44/0.36, T/swept 1.00/0.66, corner 0.65/1.00, butt/line 0.95/0.61, grooved 0.65/0.00,
arc 0.76/0.77, lap 0.20/0.27, edge 0.56/0.11 (smoke means 0.72 / 0.49, README).

*Round 2, three arms* (cell 7), median F1 per stratum:

| stratum | default full | nocrease full | plan full | default single | nocrease single | plan single |
|---|---|---|---|---|---|---|
| T/line | 0.97 | 0.82 | 0.00 | 0.67 | 0.67 | 0.30 |
| T/circle | 0.85 | 0.53 | 0.04 | 0.43 | 0.43 | 0.20 |
| T/ellipse | 0.78 | 0.55 | 0.06 | 0.57 | 0.56 | 0.19 |
| T/saddle | 0.85 | 0.76 | 0.00 | 0.69 | 0.69 | 0.23 |
| T/rounded_rect | 0.80 | 0.38 | 0.05 | 0.43 | 0.42 | 0.23 |
| T/swept_path | 0.98 | 0.51 | 0.00 | 0.66 | 0.66 | 0.39 |
| corner/line | 0.91 | 0.65 | 0.00 | 0.99 | 0.99 | 0.49 |
| butt/line | 0.88 | 0.71 | 0.00 | 0.66 | 0.66 | 0.18 |
| butt/line_grooved | 0.64 | 0.57 | 0.00 | 0.00 | 0.00 | 0.00 |
| butt/arc_butt | 0.66 | 0.60 | 0.00 | 0.99 | 0.89 | 0.34 |
| lap/line | 0.72 | 0.66 | 0.18 | 0.63 | 0.63 | 0.30 |
| edge/line | 0.31 | 0.38 | 0.02 | 0.46 | 0.43 | 0.15 |
| **mean of stratum medians** | **0.780** | 0.593 | 0.029 | **0.597** | 0.586 | 0.252 |

*Default arm details* (cell 7) — precision full/single, recall full/single, matched RMSE full/single (mm):
T/line 0.98/1.00, 0.98/0.50, 0.38/0.33; T/circle 0.97/1.00, 0.91/0.31, 0.29/0.07; T/ellipse 0.89/1.00,
0.82/0.41, 0.25/0.06; T/saddle 0.76/1.00, 0.95/0.53, 0.42/0.08; T/rounded_rect 0.74/1.00, 0.92/0.27,
0.25/0.09; T/swept 1.00/1.00, 1.00/0.49, 0.32/0.26; corner 1.00/1.00, 0.91/1.00, 0.26/0.31; butt/line
0.97/0.99, 0.89/0.50, 1.20/1.98; grooved 0.97/0.00, 0.49/0.00, 5.10/5.17; arc 0.50/1.00, 1.00/0.99,
0.43/0.47; lap 0.76/1.00, 0.68/0.46, 1.20/0.58; edge 0.72/0.98, 0.20/0.31, 2.35/1.69.

*Side by side* (cell 9), median F1 full/single — self-footprint (wps) | self-seamfind (default) | best of
seven: T/line 0.80/0.60 | 0.97/0.67 | 0.99/0.67; T/circle 0.80/0.42 | 0.85/0.43 | 0.97/0.50; T/ellipse
0.78/0.53 | 0.78/0.57 | 1.00/0.59; T/saddle 1.00/0.66 | 0.85/0.69 | 1.00/0.66; T/rounded_rect 0.78/0.42 |
0.80/0.43 | 0.75/0.44; T/swept 0.93/0.61 | 0.98/0.66 | 0.99/0.63; corner 0.75/0.79 | 0.91/0.99 |
1.00/1.00; butt/line 0.88/0.51 | 0.88/0.66 | 0.99/0.96; grooved 0.00/0.00 | 0.64/0.00 | 0.97/0.52; arc
0.67/0.94 | 0.66/0.99 | 0.95/0.93; lap 0.72/0.48 | 0.72/0.63 | 0.87/0.58; edge 0.44/0.00 | 0.31/0.46 |
0.89/0.43.

*Matched-path RMSE, full view, self-seamfind column* (cell 9; the other eight columns as in §3.A.5):
T/line 0.38, T/circle 0.29, T/ellipse 0.25, T/saddle 0.42, T/rounded_rect 0.25, T/swept 0.32, corner 0.26,
butt/line 1.20, grooved 5.10, arc 0.43, lap 1.20, edge 2.35 → lowest of all nine entries on 8 of 12 strata
(six T strata 0.25–0.42 mm, corner, arc).

*Gap and joint angle* (cell 11, full view; gap error = estimate − constructed root gap; angle error =
estimated dihedral − median of the scene's truth-seam mean dihedrals, fillet strata):

| stratum | n | gap err mean | median | p95 (mm) |
|---|---|---|---|---|
| T/line | 59 | 1.30 | 0.95 | 4.03 |
| T/circle | 60 | 0.88 | 0.34 | 4.35 |
| T/ellipse | 60 | 1.06 | 0.42 | 5.00 |
| T/saddle | 60 | 1.95 | 1.12 | 5.32 |
| T/rounded_rect | 60 | 1.09 | 0.37 | 5.00 |
| T/swept_path | 60 | 1.15 | 0.28 | 5.72 |
| corner/line | 57 | 1.77 | 1.79 | 3.07 |
| butt/line | 60 | 1.11 | 0.92 | 2.94 |
| butt/line_grooved | 59 | 1.24 | 0.96 | 2.88 |
| butt/arc_butt | 60 | 0.78 | 0.40 | 3.01 |
| lap/line | 60 | 1.93 | 1.45 | 4.98 |
| edge/line | 58 | 1.23 | 1.12 | 2.38 |

Dihedral error (deg) mean / median / p95: T/circle −2.9/−1.3/0.3; T/ellipse −0.1/0.1/9.9; T/line
0.1/0.0/23.2; T/rounded_rect −1.8/−0.0/0.0; T/saddle −8.1/−7.6/3.0; T/swept −0.6/−0.1/0.5; corner
5.1/3.0/26.2.

*The notebook's reading*, checked: ✓ default 0.78/0.60 vs plan 0.03/0.25 vs nocrease 0.59/0.59 (crease-free
refit worth 0.19 in full view, ≈ 0.01 in single); ✓ full 0.78–0.98 on the six T strata, corner 0.91, square
butt 0.88; single corner 0.99, arc 0.99, T strata 0.43–0.69; ≈ "within 0.02–0.15 of the best of seven on 7
of 12 strata in full view" (T/swept is 0.01 behind; T/line single is a tie, unmentioned); ✓ further behind on
ellipse (0.22), arc (0.29), grooved (0.33), edge (0.58); ✓ at/above on rounded rect (full) and, single, saddle,
swept, arc, lap, edge; ✓ beats self-footprint on 11 of 12 single-view strata (tie at 0 on grooved); ✓ only one
of the two to find grooved butts (0.64 full, via Stage 3b); ✓ RMSE claims; ✓ dihedral within 1.3° on 5 of 7
fillet strata, −7.6° saddle, +3.0° corner; ✓ gap biased upward, median +0.3…+1.8 mm, p95 ≤ 5.7 mm.

### 3.B.6 Limitations (README "Known limits" + nb17)
1. **Thin sheets** (t ≈ 1–1.5h): the end faces of 1 mm plates are one or two samples wide, normals cannot be
   estimated; lap and edge keep false positives in full view (lap precision 0.76; edge F1 0.31 full —
   55/60 edge scenes have 1–2 mm plates).
2. **Grooved butts, single view: 0** (the single camera rarely sees both top faces); the groove-root line is
   not the stored seam and is not reported; full view 0.64 with RMSE 5.1 mm (root-face pairs still pull
   part of the path a groove depth down).
3. **Single view, curved seams:** recall ≈ 0.3–0.4 because half a ring is out of view (the ceiling every
   method has).
4. **Gap estimate biased upward** (median +0.3 to +1.8 mm): toe-to-toe at the walkers' stop points; a walker
   stalls (`d_prev − d < tol`) or meets its patch boundary before the true toe; "a gap-specific estimate (the
   closest-pair distance between the refitted faces at the root) is the next step".
5. **θ_r is planar-only** and undefined on 90° joints at α = 0.9; the agreement test is therefore off.
6. **Tuned on the smoke round.** Defaults 5, 6, 8, 12 were chosen on the 12-scene smoke round; the full run
   confirms them over `nocrease` and `plan`, "but a held-out split is the right protocol for further
   tuning" (README `:96-98`) — ⚠ state this in the thesis: the defaults were selected on 12 scenes of the
   same corpus.
7. Stage 6 not implemented; no noise/fixture/L1/Task-2 runs; the input labels are exact (object_id).

## 3.C How the two methods' inputs map to what the cell produces (to be verified on the cell side)

| input | in WeldSet (as run) | in the robotic cell (claim of ch4 §ws_link) | caveat |
|---|---|---|---|
| footprint: scan of A alone | synthesised exact, independent area-uniform sample of the base primitive, visibility with A as sole occluder | the cell registers and saves parts one at a time, so a scan of the base before B is placed exists | the cell's base and assembly scans must be in one frame (eye-in-hand extrinsic + kinematics; the base may need ICP into the assembly — WeldSet ran with registration OFF); depth noise ~mm vs v ≈ 1 mm |
| footprint: fit-up gap | the constructed `root_gap_mm` ("wps") | a WPS nominal, or the cell's measured `fitup_mm` (seam_from_registration) | gap = 0 deletes corners/gapped butts |
| seamfind: per-part labels | `object_id` of the exact cloud (the "objects" oracle) | per-part segmentation (SAM2 masks) + registration | a label boundary error near the joint is exactly the plan's "label noise" risk (untested: no label-noise run exists) |
| seamfind: normal sign | stored normals (full) / toward the camera (single) | toward the observing camera (the plan's real-scan rule) | — |
| seamfind: gap bound | 5 mm | same | — |

The thesis's TODO "DECIDE: do the two extractors run on the robot's real scans" — nothing in
`weld_generator` runs them on real scans; as of 2026-10-20 the evidence supports only WeldSet scores.

# 4. Results inventory

## 4.1 Phase 4 benchmark — desktop run, committed in ICRA `paper/icra2027/tables/*.tex` (2026-09-15/18)
Batch history (per dataset_plan Phase 6b/6c and phase8_plan): first run done 2026-09-05 (169 800 rows, 38
chunks, ~64 h, 3 runners) → fixture chunks 2026-09-07 (173 000; 6 chunks, 40 pairs = 8 per class) →
lit-quadric 2026-09-09 (**184 680** = this laptop's file) → re-run 2026-09-12…14 on the six curved strata
after the visibility fix (379 ids retired; e.g. lobb T/circle 0.06 → 0.41) → lit-modelreg on every stratum
and the fidelity audit 2026-09-14 → **268 320 rows** landed 2026-09-15 00:00 UTC (ICRA `\nRows` 271,200 ⚠).
The fixture twins used by the paper are the **plate** study only (274 of 360 plate seeds kept: corner 36,
lap 40, T 54, grooved 54 of 60, butt −13 to `NoSeamsFound`); the 360 curved twins (2026-10-19, 60/60 kept per
stratum) have not been run through the batch. Rule blocks (2026-09-07): 4 258 tacks, 335 closed rings,
single-tack collapse lap 84 / edge 44 / grooved 31 / curved 0; MPS: T always a fillet, butt centreline
160/180 (off-class lap_toe 20), corner fillet 39 / edge 21, edge edge 33 / lap_toe 27, one null.
Median F1 at 3 mm, Task 1 full exterior, L0 (`coverage_full.tex`):

| stratum | M1 ransac | M2 regiongrow | M3 lobb | M4 ppf | M5 pcaslice | M6 modelreg | M7 quadric |
|---|---|---|---|---|---|---|---|
| T line | 0.98 | 0.58 | 0.80 | 0.46 | 0.96 | 0.99 | 0.99 |
| T circle | 0.00 | 0.75 | 0.45 | 0.27 | 0.85 | 0.83 | 0.97 |
| T ellipse | 0.00 | 0.74 | 0.39 | 0.27 | 0.66 | 1.00 | 0.88 |
| T saddle | 0.00 | 0.29 | 0.32 | 0.31 | 0.50 | 1.00 | 0.91 |
| T rounded rect | 0.00 | 0.75 | 0.20 | 0.37 | 0.65 | 0.74 | 0.55 |
| T swept | 0.77 | 0.80 | 0.80 | 0.42 | 0.92 | 0.99 | 0.98 |
| corner | 0.50 | 0.51 | 0.00 | 0.50 | 0.65 | 0.99 | 1.00 |
| butt square | 0.00 | 0.51 | 0.27 | 0.01 | 0.54 | 0.98 | 0.99 |
| butt grooved | 0.00 | 0.47 | 0.00 | 0.03 | 0.00 | 0.97 | 0.00 |
| butt arc | 0.00 | 0.31 | 0.49 | 0.00 | 0.01 | 0.95 | 0.68 |
| lap | 0.00 | 0.39 | 0.50 | 0.37 | 0.84 | 0.67 | 0.87 |
| edge | 0.00 | 0.72 | 0.63 | 0.34 | 0.37 | 0.36 | 0.89 |

`straight_curved.tex` (curved T mean of 5): M1 0.15, M2 0.67, M3 0.43, M4 0.33, M5 0.72, M6 0.91, M7 0.86.
`ladder.tex` pooled L0 → L1: M1 0.00 → 0.10, M2 0.58 → 0.11, M3 0.40 → 0.03, M4 0.31 → 0.10, M5 0.64 → 0.00,
M7 0.87 → 0.00 (M6 none). `fixture.tex` (40 pairs each; ΔF1, Δprecision, extra seams): M1 0.00/0.00/0;
M2 −0.14/−0.23/8.5; M3 0.00/0.00/22; M4 −0.01/−0.02/0; M5 −0.20/−0.18/−1; M6 +0.02/0.00/−1; M7
−0.29/−0.47/4. `task2.tex` selection T/corner/butt/lap/edge and RMSE (mm): M1 0.81/0.61/0.43/0.36/0.08,
0.80/0.00/2.79/0.88/1.96; M2 0.72/0.57/0.76/0.78/0.83, 0.49/6.22/3.05/2.44/1.27; M3
0.79/0.30/0.29/0.63/0.80, 0.64/0.60/0.67/0.71/1.16; M4 0.90/0.53/0.49/0.73/0.52, 5.81/0.24/6.77/3.59/2.26;
M5 1.00 all, 9.73/6.80/4.13/5.31/8.72; M6 1.00 all, 4.46/4.65/1.86/4.29/6.39; M7 0.93/0.25/0.46/0.80/0.77,
1.55/1.24/1.30/1.42/1.28. `cost.tex` (s per scene; L0 full / L0 single / L1 full): M1 0.02/0.01/0.09; M2
0.45/0.22/3.61; M3 0.50/0.13/10.42; M4 0.23/0.10/1.39; M5 0.47/0.22/0.91; M6 5.01/4.59/–; M7 1.00/0.33/2.70.
`numbers.tex` macros (selection): ransacTline 0.98; ransacCurvedZeros 4 of 5 curved T + both curved butts;
ransacCurvedMax 0.77 (swept); quadricBestDetectors 10 of 12; quadricRingRmse 1.67–2.53 mm (projection
rung 0.011 mm); quadricRectPrec 0.51, quadricArcPrec 0.51; ppfCurvedRecallMin 0.47, ppfCurvedPrecMax 0.28;
pcasliceWholeClosedMax 0.07, pcasliceClosed 0.65–0.85, pcasliceSwept 0.92; lobbCorner 0.00 vs
lobbFlatCorner 1.00 (3 mm rung), lobbEdge 0.63 vs 0.22, lobbButt 0.27 vs 0.60; regiongrowSurfaceCropMax
0.28; ransacCorner 0.50 → 0.93 single; pcasliceArc 0.01 → 0.73 single; T-joint vs angle joint: ppf 0.62 →
0.03 (15° tolerance; 0.60 at 30°), lobb 0.96 → 0.74, ransac 0.99 → 0.93; ransacBimodalShare 5 %;
cornerGapMax 4.9 mm; modelregDenseButt 0.97 → 0.15; noise (single, pooled, clean → 2σ): quadric 0.58 →
0.50, lobb 0.22 → 0.30, ransac 0.20 → 0.00; taskTwoQuadricSel 0.64, taskTwoLobbSel 0.56, quadric Task-2 RMSE
≤ 1.5 mm; fixtureLobbExtra 22; nScenes 720, nStrata 12, nRansacSeeds 30, **nRows 271,200**.
Reproductions (README, pre-audit record, per README 2026-09-21): ransac RMSE 0.60 / ME 0.81 mm vs reported
≤ 0.64 / 1.16 on a T at 0.6427 mm spacing with the 40 mm region (exactly 0 noiseless with refit off); lobb
median RMSE 0.51 mm over 25 corner scenes vs < 0.7 reported.
⚠ MANUAL §9 (2026-09-08) and the first README paragraphs are **pre-audit** (e.g. "lobb won", "Task 2 0.79
selection", "+23 phantom seams", "quadric rings at 0.01 mm", "lobb 0.43 → 0.03"): do not cite them as
current.

## 4.2 Proposed extractors — desktop run, committed in nb16/nb17 (2026-10-06): see §3.A.5 and §3.B.5
Means of the 12 stratum medians (F1 full / single): self-seamfind default 0.780 / 0.597, nocrease 0.593 /
0.586, plan 0.029 / 0.252; self-footprint (wps) ≈ 0.73 / 0.50 (my mean of nb16's medians:
(0.80+0.80+0.78+1.00+0.78+0.93+0.75+0.88+0+0.67+0.72+0.44)/12 = 0.729; single 0.497).

## 4.3 Phase 5 annotation floor — laptop file `out/annotation` (2026-09-01), equals the ICRA macros
§2.12. These numbers depend only on the clicks and the analytic truth, not on the visfix; still, confirm the
file is identical on the desktop.

## 4.4 Corpus composition and realised ranges — laptop, stale: re-take on the desktop
720 scenes, 60 per stratum (local `facts.csv`, 23 columns). Realised ranges per stratum: §7. Quality mix
examples: T/line B 19, C 9, D 13, below_D 19; corner B 17, C 14, D 12, below_D 17; grooved B 2, C 12, D 14,
below_D 32; curved strata mostly B. ISO term T/line: T-joint (3.10) 24, angle joint (3.12) 36; T/saddle
20/40; T/ellipse all angle joints.

## 4.5 Yields
Per README (2026-09-21, Phase 3, 120 seeds per type): overall ~52 %; T 79 %, butt 57 %, corner 44 %, edge 42 %,
lap 36 % (README also says lap 34 % vs T 81 % in another paragraph ⚠ minor). nb02 (laptop, stale,
playground config): T 63 %, corner 38 %, butt 60 %, lap 23 %, edge 47 %. bench_phase4 attempts per 60
emitted: §2.8 (laptop, stale).

## 4.6 Gates and tier 2 — per README (2026-09-21), corrected per `notes/phase8_plan.md` and `weldgen/render/gate.py`
Corrections from the tier-2 code reading (this laptop holds no tier-2 data; every value is per the
docs): gate thresholds residual p99 < 0.25 mm, coverage agreement ≥ 0.90, object agreement ≥ 0.98
(`gate.py:25`); visfix 12-scene gate p99 ≤ **0.033 mm**, recall ≥ 0.994, objects ≥ 0.999
(`gate_visfix_12strata.json`); final entry-only 12-scene gate coverage ≥ 0.995, recall ≥ 0.986,
precision ≥ 0.987, objects 1.000 (`gate_final_12strata.json`); prototype 2026-09-10 p99 0.0006 mm, pilot
≤ 0.0008 mm; train_v1 re-gate 3600/3600 (no per-scene distribution documented: compute it from
`out/train_v1/twin_gate_sweep.json`). The three ray-test bugs: (1) sampled containment (K = 9 samples)
stepping through tube walls, 2026-09-11 (mean/worst wrongly visible: rounded_rect 35 %/72 %, saddle
17.5 %/35 %, circle 6.3 %/15 %, ellipse 4.2 %/9.5 %, swept 0.8 %, arc 0.1 %, plates 0) → `ray_hits_mesh`
(Möller–Trumbore on the D34 mesh); (2) mesh cache keyed before re-posing (same day); (3) exit crossings
counted as occluders → entry-only crossings, commit `f50aa05`, 2026-09-12. Regenerations: visfix
2026-09-11, visfix2 (entry-only) 2026-09-12 15:30 (plate strata byte-identical). train_v1: built
2026-09-12, materials-1.0 render done 2026-09-13, materials-1.1 render done 2026-09-15 (23.5 h, ≈ 2.35
s/frame, 0 gate failures, verify-render 3600/3600). Further caveats for the thesis: render substream 7 is
named but unused (fresh RNG from sha256(scene_id|render_id)); the key light's "elevation 30–70°" is not a
physical elevation (≈ half the draws put it below the horizon); the RGB is most likely RTX real-time, not
path-traced (default render mode; confirm on the desktop); substrate images 31–41 are generated, not
photographs; mask widths "2 mm / 3 mm" are nominal (realised ≈ 2.2–10 mm).
README's sentence, for reference:
D28 0.34 (vs 0.44 allowed; failed 0.45 before the grooved rebuild); D34 measured 0.05 mm (≤ 0.25); twin gate
p99 ≤ 0.03 mm (< 0.25), coverage ≥ 0.99, ids 1.00 on 12/12 strata and 3600/3600 train_v1; train_v1 36 000
frames rendered twice (materials-1.0 then 1.1) at ~0.1–1 s per frame; `verify-render` 3600/3600.

## 4.7 `ours` (radius-PCA, excluded) — per README (pre-audit record)
Corrected window R < t/2: F1 0.709 → 0.897 and band 5.70 → 3.31 mm on 44 paired scenes; lap with the exterior
condition precision 0.53 → 0.68, band 5.8 → 3.1 mm; without object ids F1 0.75 → 0.03; ICRA: "below 0.05
precision on edge joints".

## 4.8 Timing
Batch ~64 h on a 16-core laptop, three runners (per MANUAL §8.3, 2026-09-08); nb16 round 1 157 s for 48 rows,
nb17 239 s for 48 rows; seamfind stage timings on one T/line scene (§3.B.5).

## 4.9 DESKTOP: to collect there
1. The final batch file(s): row count (268 320 vs 271 200), chunk list, the post-audit rung names.
2. Corpus sizes per stratum and the build manifests of the final (visfix-regenerated) `bench_phase4`,
   `bench_phase4_fx` (634: 274 + 360) and `train_v1` (3600); yields (attempts per stratum) of the final build.
3. Realised generator ranges per stratum from the final `scene.json` files (re-run the §7 measurement there).
4. Every benchmark table per method × stratum × condition: Task 1 full and single, L1, noise ×1/×2 per
   profile, fixture pairs, Task 2 selection/localisation + MPS margin per stratum, repeatability, cost,
   ISO 17659 T/angle split, D19 offset statistics — re-executed nb15 (and nb13 if used).
5. self-footprint and self-seamfind result files (`out/self_footprint/self_footprint.csv.gz`, 2 880 rows;
   `out/self_seamfind/self_seamfind.csv.gz`, 4 320 rows) and their figures (nb16/17 `figs/`).
6. Renders for the seam-family figure (`fig:ws_families`, one per family) and tier-2 examples (views, seam
   and tack masks) from `train_v1` or the benchmark's view 0; the ICRA `fig_renders`/`fig_gallery` sources.
7. Twin-gate statistics per stratum (p99 residual, coverage, id agreement) from the `render.json` reports.
8. D28 and D34 gate reports of the final corpus.
9. The annotation files (to confirm identity with the laptop copy).
10. Tier-2 gate files: `out/train_v1/twin_gate_sweep.json` (per-scene residual/coverage/objects
    distribution), `gate_final_12strata.json`, `gate_visfix_12strata.json`, each `render.json` `twin_gate`;
    the render mode actually used (RTX real-time vs path-traced) and a key-light check; do not mix in
    `out/train_v1_renders_materials10/`.

# 5. Fact check of `ch4_methodology_weldset.tex`

| # | draft (line) | verdict | correct value / source |
|---|---|---|---|
| 1 | "one seed gives identical twin scenes across any one dial" incl. thickness, gap, angle (l.31-32) | ⚠ wrong for geometry dials | twins exist only across **non-geometry** keys (fixture presence, sensor profile/noise, density, camera regime/view, sampling mode, rendering); thickness/gap/angle change the geometry (MANUAL §2 "Twins") |
| 2 | Seven curve families "cover the cases of the literature" (l.50-53) | soften | families are chosen by the D29 realisability rule ("not a workpiece catalogue", README `:698-700`; ICRA: "that rule, not novelty, is the stopping point") |
| 3 | curve constructors "reused by the cell, Eq. ellipse_seam" (l.54-55, 238-241) | ⚠ | the cell reuses weldgen's **curve library** intersection forms (plane∩cylinder, cylinder∩cylinder, offsets) and `seam_verdict` — WeldSet's verification arm — not `constructors.py`/`d29.py` (`curved_seams.py:1-30`) |
| 4 | D4 "computes their nominal seam as the intersection of the two face planes … keeps it when the bisector escapes" (l.64-67) | incomplete | three arms: intersecting, facing (rejected `bisector_blocked`), **coplanar exposed** (butt centreline, edge joint — no plane intersection); the torch cone (30°, 15 mm) may tilt off the bisector up to 45° (SCHEMA §2.6.1); plus D13 and D22 gates and the cross-run rule (README `:356-383`) |
| 5 | D19 root = "the terminating face's edge" (l.69-70) | verify wording in SCHEMA §1.3 | ICRA: "the root line on the far face"; scored curve = nominal because continuous as g → 0 and independent of which part is A (`dataset_plan.md:52`) |
| 6 | tack equations (l.76-84) | ✓ but incomplete | add: d_min lowering ($n=\max(2,\lfloor L_{\mathrm{eff}}/d_{\min}\rfloor+1)$ if $L_{\mathrm{eff}}/(n-1)<d_{\min}$); one tack at L/2 if $L-2m\le\ell$; closed $n=\max(4,\lceil L/d_{\max}\rceil)$ rounded up to even, d_min lowering not below 4, phase from sha256(scene_id:seam_id), no margin on loops; t = thinnest member gauge; primary seams only (`tacks.py`) |
| 7 | "weld order: the ends, then the centre, then by bisection (opposite pairs on a loop), same-class interleaved" (l.86-88) | ✓ | `tacks.py:64-77,166,174-184`; interleaving is round-robin by sequence within a class |
| 8 | tack provenance TODO (l.89) | fill carefully | JASS 6 via Kobelco handbook (spacing, 400 mm); EN 1011-2 for tack length **attributed, not verified**; Tomków et al. 2020 anchor (`tacks.py:38-43`) |
| 9 | Tier-2 twin gate "rendered depth on the exact surfaces within 0.25 mm" (l.106-107) | ✓ add statistic | p99 residual of view-0 clean depth < 0.25 mm (measured ≤ 0.033 mm), plus coverage agreement ≥ 0.90 and id agreement ≥ 0.98 as thresholds (`gate.py:25`; measured minima 0.995 / 1.000); view 0 only; "12/12" = one scene per stratum |
| 9b | tier 2 = "the same scenes rendered", "paired by seed" (l.106) | ⚠ | the 720-scene benchmark was never rendered (12 gate scenes + pilots only); the rendered corpus is train_v1 (3600 scenes, base seed 3 000 000, no seed shared with the benchmark); the ICRA render figure tiles come from train_v1 (`make_gallery.py:37-55`) |
| 9c | "published material reflectances" (l.106) | qualify | F0 from RTR4 Table 9.2 for 6 of 7 alloys, **bronze interpolated** (`materials.py:46`); per-condition metallic/roughness/albedo constants hand-set; RTR4, Poly Haven and ambientCG are not cited in the ICRA bib — add them; render time ≈ 2.35 s/frame (materials-1.1, 23.5 h for 3600 scenes), not the planned 0.1–1 s; do not call the sensor "structured light" (D435-class active stereo) |
| 10 | depth noise referenced to the cell's Eq. depth_error (l.105) | ⚠ | that equation is a disparity-*offset* bias; WeldSet's model is the random error $\sigma_z=s\,z^2/(f b)$ with subpixel RMS s (PARAMETERS §4.2) — state it separately |
| 11 | strata table: T/line t 1–12, α 60–120° | ✓ | PARAMETERS §3; realised 1.09–11.97 mm, 62–118° |
| 12 | butt grooved t 4–20 mm, V/bevel/U | ✓ (realised) | config draws t 3–20 (`grooved_butt.yaml`); realised 4.26–19.80; preps single_V 36 / single_bevel 16 / single_U 8 (laptop, stale; same as phase9_plan) |
| 13 | edge t 1–2 mm | ✓ | ISO 9692-1 ref 1.1, t ≤ 2 |
| 14 | T/circle tube r 30–80, wall 3–8 | ✓ | `d29.py:74` r ∈ U(30, 80); wall U(3, min(8, 0.4r)) (`constructors.py:61`) |
| 15 | T/ellipse "tilted tube, 10–40°" | ⚠ | drawn U(10°, 40°) (`d29.py:88`) but the D4 cone (30° half-angle, ≥ 95 % clear) rejects dihedrals < 60°, so the corpus holds tilts **10.2–30.6°** (included angle 59–80°; phase9_plan "cut at 60–80°"); semi-minor b ∈ U(30, 70) |
| 16 | T/saddle branch r 0.3–0.7R | ✓ add | R ∈ U(40, 80), r ∈ U(max(15, 0.3R), 0.7R), tilt U(0, 30°), offset ±0.4(R − r) (`d29.py:106-111`) |
| 17 | T/rounded_rect w,h 60–160 | ✓ add | corner r_c ∈ U(8, 0.25 min(w,h)), wall min(U(3,8), 0.8 r_c), height U(80, 220) |
| 18 | T/swept_path band 4–10, height 50–156 | ✓ (156 is realised; draw 50–160) | t U(4,10), H U(50,160), span U(120,300), arc R U(max(30, span/π), 400) or B-spline ≥ 30 mm curvature radius |
| 19 | butt arc R 100–400 | ✓ add | span U(100, min(300, 0.9πR)), t U(3,10) (70 % equal), widths min(U(50,150), 0.6R), gap U(0.3, 2.0) |
| 20 | "strata are reported separately, never pooled" (l.136) | qualify | the ladder and noise results are pooled medians in the paper |
| 21 | "each with its own notebook" (l.142) | ✓ | notebooks 04, 05, 06, 08, 09, 10, 11 (07 = repeats, 03 = `ours`) |
| 22 | method descriptions (l.143-145) | adjust | M2 is "curvature-seeded region growing" (ICRA/README), not "normal-based"; M6 is non-rigid CPD registration of a model; M7 selects plane **or quadric** per surface and keeps measured points near the fitted intersection |
| 23 | citation keys (l.146) | ⚠ bib incomplete | thesis.bib: `wei_regiongrow` lacks venue (arXiv:2408.10710, 2024); `zhang_lobb` lacks year/volume/pages (RCIM 95 (2025) 102987); the LOBB descriptor paper (IEEE T-ASE 22 (2025) 75–84, ICRA key `zhang2025lobbtase`) is absent; `fang2024modelreg` author list "and others" (ICRA bib: Fang, Weihua and Tian, Xincheng); `li2026quadric` title truncated (ICRA: "…based on normal of point clouds and PCA", pp. 2039–2046) — verify all against the PDFs in `papers/implemented_papers/` |
| 24 | oracle ladder (l.149-152) | ✓ | M6 has no L1; M7's coarse stage in its paper is region growing (not learned) — "most" is right |
| 25 | metrics (l.154-157) | ✓ add | τ = 3 mm, densification 1 mm in code (⚠ ICRA says 0.5 mm), precision against exact polylines, one-to-one matched RMSE |
| 26 | conditions "noise levels" (l.159-160) | specify | ×0/×1/×2 σ on the single view only; L1 on full_exterior; fixture 40 pairs; 30 RANSAC seeds |
| 27 | "Two extractors … scored by the same harness, metrics and strata" (l.168-169) | ✓ qualify | only coverage runs (clean, two views, one seed); footprint gets the constructed gap ("wps"); seamfind gets exact `object_id` labels |
| 28 | "The base scan is registered into the assembly scan" (l.181) | ⚠ | ICP is optional and **off** on WeldSet (exact poses) |
| 29 | footprints "closer than the fit-up gap g" and points slide "until the distance reaches g" (l.183-190, Eq. footprint) | ⚠ wrong | footprint threshold $g_{fp}=3v+g_{fit}$; points slide until the distance equals the **fit-up gap** $g_{fit}$ (`weld_seam.py:601,632,651`) |
| 30 | "ball rolled … gives the toes, the leg lengths and the bead area" (l.193-194) | ✓ add | fillets only; not scored on WeldSet |
| 31 | "Seeds are mutual nearest neighbours between farthest-point samples" (l.209-210) | ⚠ wrong for the implementation | default = **one-way NN on all downsampled points** within τ = 5 mm + 2h, plus Stage 3b coplanar boundary pairs; mutual NN on FPS is the plan's form (unused); the `plan` arm uses mutual NN on all points |
| 32 | Eq. sphere_trace with $\mathbf t_k/\lVert\mathbf t_k\rVert$ (l.212-216) | ⚠ wrong for the implementation | $\mathbf x_{k+1}=\Pi_A(\mathbf x_k+\alpha d_k\mathbf t_k)$, $\mathbf t_k$ **not normalised** (α × tangential part of $\mathbf q_k-\mathbf x_k$); $d_k$ to B's closest finite-patch point; extra stop at A's patch boundary (`walk.py:171-180`) |
| 33 | "the disagreement is a quality flag" (l.222-223) | ⚠ | computed but the θ_n/θ_r test is **off by default** (planar-only; it drops curved seeds); on only in the `plan` arm |
| 34 | rolling-ball spine (TODO l.226) | not implemented | Stage 6 / M9 open |
| 35 | link: "seam rule … applied unchanged" (l.239-241) | ⚠ | only the tack rule (and primitives) unchanged; the D4 rule is the registration-tolerant `judge_registered`; curved seams via the curve library + `seam_verdict` |
| 36 | annotation section "If run" (l.263) | ⚠ outdated | it was run (Phase 5): 1.415 mm median, 11.8 mm p95, lap 8.5 mm (§2.12) |
| 37 | SRC MANUAL §1–2 / §8 | note | MANUAL §0 and §13 still say "six published methods"; §9 is pre-audit |

# 6. What the draft does not mention but should
1. The **fidelity audit** (2026-09-14) and the rung mechanism (every reading priced); the reproductions of
   the papers' own numbers (ransac 0.60/0.81 mm; lobb 0.51 mm); the lit-lobb first author's written
   endorsement (2026-08-27); the open request to the lit-quadric authors (2026-09-09).
2. The **twin gate found three tier-1 ray-test bugs** and the corpus was regenerated before the paper's
   numbers (the strongest argument for two independent exact implementations).
3. ISO grounding: ISO 5817 (5071, 617; 508 absent from Table 1, the β mapping is [ours]); ISO 9692-1
   (angles, gaps scaling with t, groove rows, tacked condition); ISO 17659 taxonomy (T vs angle joint —
   it exposes the ppf 0.62 → 0.03 drop); quality levels derived after the draw.
4. The fixture (D12/D13) as a paired hard-negative generator and its measured price (lobb +22 phantoms,
   quadric −0.29 F1).
5. D28 anti-shortcut gate (0.34 vs 0.44; the grooved rebuild with PreparedPrism) and D34 (0.05 mm).
6. Determinism (content hash, twin_key, substream append-only discipline, numpy pinning).
7. Yield / omission policy (~52 %; per type; `--n` is attempts; skipped seeds recorded in `index.jsonl`).
8. The `ours` baseline and its exclusion by ruling (2026-09-03); the radius-PCA window prediction
   (R < t/2; empty for one-sided square butts at any density).
9. Task 2 results and the MPS margin; the approach_cone regime.
10. Repeatability (ransac 5 % bimodal scenes; deterministic methods zero spread).
11. The three stated generator flatteries (ICRA §IV-B): exact analytic primitives; no burrs/holes/edge radii;
    the seam-bearing edge is the only axis-aligned one.
12. Tier 2 deliverables (train_v1 36 000 frames, masks, materials) and what is not done (tier-2 analysis
    notebook, a learned detector).
13. The Phase 9 real-subset plan and its tie to the cell's registration pipeline (truth from registered
    poses with measured uncertainty).
14. For the two own methods: the selection of seamfind's defaults on the 12-scene smoke round of the same
    corpus; footprint's use of the constructed gap; no noise/fixture/L1/Task-2 runs; no real scans.
15. The 1 mm densification step and one-to-one matching in the metric definitions.
16. Tier-2 honesty points: the benchmark was never rendered (train_v1 is a separate corpus); bronze F0
    interpolated; mask widths nominal; RGB render mode to confirm; gate thresholds 0.25 mm / 0.90 / 0.98.

# 7. The twelve strata and their generator ranges
Columns: **drawn** = sampler code (current); **realised** = min–max over the 60 scenes of this laptop's
`out/bench_phase4` (laptop, stale: re-take on the desktop); **phase9** = per `notes/phase9_plan.md` §1,
2026-09-21 (measured on bench_phase4).

| stratum | geometry drawn (code) | realised (laptop, stale) | phase9 | notes |
|---|---|---|---|---|
| T/line | plates t 1–12 mm, along-seam L 80–400, W 50–250 (PARAMETERS §3), α 60–120°, ISO gaps + tail; yaw full circle, polygon outline on B (`bench6a_T.yaml`) | t 1.09–11.97; α 62.4–118.5°; gap 0.01–4.17; h ≤ 4.44 mm; β ≤ 7.5°; part extents 89–544 × 51–245 mm; 24 T-joint / 36 angle joint | t 1–12, L 80–600, W 50–250, α 60–120, gap 0–5 | yield 60/83 |
| corner/line | as T; outlines on both parts | t 1.04–11.68; α 60.2–118.5°; gap 0.02–4.88; h ≤ 5.37; β ≤ 7.9° | same | 60/128 |
| butt/line (square) | plates t 1–12; gap ≈ t / ≈ t/2 by ISO row; α 180°; outlines both | t 1.16–11.98; gap 0.08–4.44; h ≤ 4.44 | same | 60/118 |
| butt/line_grooved | t ∈ [3, 20] (`grooved_butt.yaml`), `valid_preps(t)` (code, per the decision-log reading of `config.valid_preps`): t ≤ 4 {square}; 4–8 {square, bevel, V}; 8–10 {bevel, V}; 10–12 {V}; > 12 {V, U}; single-V row 1.3 (40–60°, b ≤ 4, c ≤ 2) switching to 1.5 (~60°, b 1–4, c 2–4) at 10 mm; single-bevel 1.9.1 (β 35–60°, b 2–4, c 1–2); single-U 1.6/2.6 (β 8–12°, b 1–3, c ≈ 4–5, R 6–9) — PARAMETERS' "bevel 3–10" is the ISO row; equal thickness; gap from the row's b | t 4.26–19.80; gap 1.06–3.93; bevel 8.2–58.4°; root face 0.64–4.75; U radius 6.2–8.8; V 36 / bevel 16 / U 8 | t 4–20; V 36/bevel 16/U 8; bevel 8–58° | 60/89; excluded from Phase 9 |
| butt/arc_butt | R ∈ U(100, 400); span U(100, min(300, 0.9πR)); t_A U(3,10), t_B = t_A (p 0.7); widths min(U(50,150), 0.6R); gap U(0.3, 2.0) | R 100.4–394.5; span 101–298; widths 50–145; t 3.06–9.95; gap 0.31–1.98 | R ≈ 250, band 50–145, t 3–10 | 60/64 |
| lap/line | plates t 1–12; overlap [ours]; β ≤ 0.4° (D23); yaw full circle | t 1.00–11.22; gap 0–4.10; stack offset 11–128 mm | t 1–12 … | 60/108 |
| edge/line | t ≤ 2 mm (ISO ref 1.1); stack offset 0; β ≤ 0.4° | t 1.00–2.00; gap 0–2.97 | t 1–2 | 60/110 |
| T/circle | r ∈ U(30, 80) (= pipe outer radius); wall U(3, min(8, 0.4r)); height U(80, 220); gap U(0.1, 1.2); base plate t U(4, 12) | r 31.5–79.9; wall 3.00–7.98; length 80–219; gap 0.11–1.19 | r_out 31–80, wall 3–8, length 80–220, plate 3–12 | 60/77 |
| T/ellipse | b ∈ U(30, 70); tilt U(10°, 40°); a = b/cos(tilt); gap U(0.1, 1.2) | b 30.4–69.5; **tilt 10.2–30.6°** (α 59.4–79.8°); wall 3.06–7.97 | "cut at 60–80°" | tilts > ~30° fail the D4 cone; 60/107 |
| T/saddle | R ∈ U(40, 80); r ∈ U(max(15, 0.3R), 0.7R); tilt U(0, 30°); offset ±0.4(R − r); branch length 80–200 + lip | R 40.1–79.8; r 15.5–50.3; tilt 0.6–30.0° (α 60.0–89.4°); walls 3.1–8.0 | "branch r 15–80 on run r ≤ 80, α 60–89°" ⚠ (branch ≤ 0.7R ≤ 56) | one primary seam (bore rejected); 60/73 |
| T/rounded_rect | w, h ∈ U(60, 160); r_c U(8, 0.25 min(w,h)); wall min(U(3, 8), 0.8 r_c); height U(80, 220) | w 60.1–160.0, h 60.8–159.7; r_c 8.1–35.7; wall 3.15–7.89; height 80–218 | "footprint 64–194, corner R ≈ 15, wall 3–8, height 80–220" (⚠ 194 > 160) | 1–2 primary seams; 60/73 |
| T/swept_path | span U(120, 300); half arcs (R U(max(30, span/π), 400)), half B-splines (4–5 ctrl pts, curvature radius ≥ 30); band t U(4, 10); height U(50, 160); gap U(0.1, 1.2) | span 121–288; arc R 84–395; 26 arcs / 34 B-splines; band 4.14–9.91; height 50.1–156.4; plate 224–457 long | band 4–10, height 50–156, plate 220–460 | two offset-curve fillets at ±t/2; 60/67 |

All strata: density 0.25–4 pts/mm², standoff 300–1200 mm, elevation 15–85°, three sensor profiles
(realised, laptop). Curved strata carry no linear/angular misalignment (h = β = 0) and gaps ≤ 1.2 mm
(arc butt ≤ 2.0 mm).
