# V3: fact check of `ch4_methodology_weldset.tex` against `weld_generator` (2026-10-20)

Draft: `ros2_ws/src/admittance_control/thesis/manuscript/chapters/ch4_methodology_weldset.tex` (733 lines, read in full).
Code paths below are relative to `/workspaces/welding_cell_ws/weld_generator/` unless prefixed `cell:` (= `ros2_ws/src/admittance_control/`).
Items the draft already marks as TODO (P1 #1–#5, P2 #6–#13, the P3 list, the ICRA list) are not re-reported.

Two numerical checks were run (scripts in this folder, read-only use of the repo code):
`walker_ratio.py` (seamfind walker on two analytic half-planes) and `saddle_check.py` (saddle closed form);
`nook_check.py` (D4 verdict of the acute fillet versus the dihedral).

## 1. Findings

Severity: **E** wrong, **I** imprecise, **N** nuance.

| # | draft line | draft says | correct value / what the code does | evidence | sev |
|---|---|---|---|---|---|
| 1 | 645–648 | On two planar faces at angle θ each step shrinks the distance by $d_{k+1}=d_k(1-\alpha\sin\theta)$, so $\theta_r=\arcsin((1-\bar r)/\alpha)$ | Wrong for the step of Eq. `sphere_trace` (α × the tangential part of $\mathbf q_k-\mathbf x_k$). With $d_k=s_k\sin\theta$ and tangential part $d_k\sin\theta$, the step is $\alpha s_k\sin^2\theta$, so $d_{k+1}=d_k(1-\alpha\sin^2\theta)$ for θ ≤ 90° (for θ ≥ 90° B's nearest point is its edge and the first ratio is $1-\alpha$, independent of θ). The correct estimate is $\theta_r=\arcsin\sqrt{(1-\bar r)/\alpha}$. The draft's relation is the plan's, for the plan's step $\alpha d_k\hat{\mathbf t}$ (seam_finder.md:148, 161, 166); the implementation changed the step (README row 11b) but kept the plan's formula in `walk.theta_r`, so the code's θ_r is biased low. Measured with the package's own `walk`: mean ratio 0.775 / 0.550 / 0.325 at 30° / 45° / 60° (= $1-0.9\sin^2\theta$ exactly; $1-0.9\sin\theta$ would be 0.55 / 0.36 / 0.22); `theta_r` returns 19.0° / 35.9° / 55.1°. No benchmark number depends on it because `angle_check=False` by default, but the sentence and any reported θ_r must be corrected (or `theta_r` fixed). | `Self_ideas/test2_cagdas/seamfind/walk.py:3-6, 57-60, 75-82`; `seamfind/config.py:66`; `seamfind/README.md:46, 52`; `Self_ideas/test2_cagdas/seam_finder.md:148, 161, 166`; scratchpad `walker_ratio.py` | **E** |
| 2 | 169–173 | Threat: "if every plate edge is parallel to a world axis…"; measure 1: "each assembly is turned by a random yaw over the full circle" | The D28 shortcut is the seam being parallel or perpendicular to the parts' **own free edges**, not to world axes. The global assembly yaw U(0°, 360°) (present since Phase 1, plates and curved) leaves every edge-to-seam angle unchanged, so it cannot affect the gate. The D28 mechanisms are (a) for **T and lap only**, an in-plane yaw of B about A's face normal, uniform over the per-scene feasible subset of the full circle (1° grid + jitter; seam run ≥ 50 % of min L), and (b) convex outlines for B in every joint type and for A in corner, butt and edge, seam-bearing edge pinned. The strata table (l.480 "yaw over the full circle, outline on the stem") already describes the right mechanism, so the paragraph contradicts it. | `scene.py:250-256` (global yaw); `config.py:446-483`; `layouts.py:444-494, 497-505, 613-628`; `notes/dataset_plan.md:1616-1645` (D28 text) | **E** |
| 3 | 273 (Alg. `ws_access`) | After the three arms: "reject if a part is the fixture, if the dihedral is outside [30°, 170°], if the line is shorter than 10 mm" | The dihedral range applies to the **intersecting arm only**. A coplanar centreline has dihedral 180° and is checked only for the fixture and the 10 mm length (then the cone search with axis = the shared normal); facing pairs return directly (`fixture_contact` or `bisector_blocked`). As written, the box rejects every butt and edge seam. Move the dihedral test into the first branch. | `accessibility.py:353-369, 425-440, 612-633` | **E** |
| 4 | 52–53 | Tier 1 "uses NumPy and trimesh only" | Also SciPy (`cKDTree` in `exterior_scan_subsampled`, called for every scene; `ConvexHull` in HPR) and PyYAML (config loading); all four are in requirements.txt. | `visibility.py:395, 498`; `scene.py:403-404`; `config.py:13`; `requirements.txt` | I |
| 5 | 75, 79–81, 87 | "keep the weldable ones as ground truth"; reject "if no weldable seam is visible enough" (the only rejection branch) | All candidates with a line are stored with their verdicts (weldable or not); scoring truth is the **primary** set (Task 1) or the weldable set (Task 2). The plate omission tests **primary** seams: `NoSeamsFound` if none, `NoVisibleSeams` if the best visible fraction < 0.1 (curved: best *weldable* < 0.1). Other recorded rejections: D31 `AmbiguousClassConfiguration` (before placement), `CurvedGateFailed` (rediscovery > 1e-6 mm or chord > 0.25 mm), `filtered_by_source_predicate` (grooved arm drew a square prep). | `scene.py:226-240, 413-414, 498-516, 522-532`; `scene_curved.py:208-218, 405-408, 487-490`; `make_bench6b.py:131-143`; `scripts/baselines/dataset.py:188-201` | I |
| 6 | 133–134 | "the part dimensions across the seam (they vary across it, never along it)" | The sampler draws the along-seam lengths too: L_A, L_B ~ U(80, 400) independently plus an offset of ±0.4 min L (the draft's own Table 2 says so). D27's "never along" is the rule for a *same-seam* variant family, where the along-seam dimension is pinned by the seam length. Reword: the along-seam lengths set the seam (the shared run); the across-seam widths are free. | `config.py:357-362, 443-444`; `notes/dataset_plan.md:60` (D27) | I |
| 7 | 134–135 | "three defects with ISO 5817 limits, namely g, h and β" | Only g (no. 617, fillets) and h (no. 5071) have ISO 5817 Table 1 limits. β uses Annex B Table B.1 fatigue values mapped to B/C/D by the project's convention (1° / 2° / 4°, D = doubled C); Table 1 has no β limit. Say "[ours, after Annex B]". | `config.py:212-219`; `joints.py:54` | I |
| 8 | 244 | "wider bores are weldable inner fillets" | A bore ≥ 80 mm passes only the confinement gate; it is weldable only if ≥ 95 % of its points also clear the 30° cone, otherwise `bisector_blocked` (the review counts 95 of 240 bores weldable, none on the saddle: DESKTOP). | `verify_curved.py:229-250` | I |
| 9 | 276 | "demote a fillet that runs along a weldable butt centreline (toe of centreline)" | Demoted is **any** weldable intersecting run parallel to, and within 1.5 × gap of, **any** weldable coplanar centreline (butt or edge class). The toes of a butt or edge gap are broad-face × edge-face pairs, i.e. `lap_toe` class, not fillets. | `accessibility.py:693-719, 149-155` | I |
| 10 | 390–392 | aim $\mathbf a=\mathbf m+0.15L\boldsymbol\xi$, m = mean of the weldable seam midpoints, L = longest part edge | True for plates (L = largest bounding dimension of any workpiece, `dims_mm`, outline bbox for prisms). Curved scenes: m = mean of the 64 coarse samples of every **weld-role** seam (regardless of verdict) and L = E (longest side of the workpieces' world AABB). | `scene.py:76-96`; `scene_curved.py:287-292`; `geom.py:325-329` | I |
| 11 | 656 (Alg. `ws_seamfind`) | candidates: "drop those no torch could see (cone prefilter)" | Stage 2a is an exterior-visibility test, not a torch test: 24 rays (normal + rings at 30° and 60°) in a 60° cone, sphere-traced on the merged cloud to 40 mm; a point is dropped only if **no** ray escapes (τ_vis = 0.04, ≥ 1 of 24). The 30° torch cone is Stage 2c, at the end. | `seamfind/config.py:39-47`; `seamfind/access.py:31-75`; `seamfind/pipeline.py:74-85` | I |
| 12 | 706 (Tab. `ws_link`, "Fit-up gap") | In WeldSet: "the constructed root gap" | Holds for the footprint method (`gap="wps"` = `root_gap_mm`; a zero-gap rung). The seam finder's default is the 5 mm bound (`gap="iso"`); `gap="wps"` (root gap + 0.5 mm) exists but was not run (review C §3.B.4). Split the row per method. | `scripts/baselines/self_footprint.py:118-123`; `scripts/baselines/self_seamfind.py:13-14, 33, 45-46` | I |
| 13 | 72 | substreams "(geometry, defects, seam curve, placement, sampling, camera, …)" | Names are `joint_config, defects, seam_curve, placement, surface_sample, camera, noise, render`. | `rng.py:23-32` | N |
| 14 | 82 | "apply the rule blocks: tacks and the most probable seam" | Order is MPS, then tacks; both are opt-in (`emit_mps`, `emit_tacks`) or applied after the fact (`scripts/apply_rule_blocks.py`). | `scene.py:637-642`; `scene_curved.py:491-496` | N |
| 15 | 96–97 | appended draws mean "adding a parameter does not change existing scenes" | The draws stay bit-identical, but a new **default** config key changes `config_id` and so every scene id (measured bench6a 5b2a54f4 → 825815e7, which is why `emit_mps` is not a default). | `config.py:117-123`; `hashing.py:72-74` | N |
| 16 | 98 | hash "covers the canonical description and every array" | `provenance` (timestamp, versions) is excluded; arrays are hashed in sorted key order with dtype (little-endian) and shape. | `hashing.py:20-21, 41-69` | N |
| 17 | 99–100 | arrays agreed "to $10^{-5}$ relative" | Notes: max relative 7e-6 on values near zero, 1e-15 elsewhere (numpy 2.3.5 → 2.5.3); "no array or label change". Consistent; more precise. | `notes/phase8_plan.md:381-388` | N |
| 18 | 130–131 | T = T-joint near 90°, angle joint otherwise | "Near" is the project's ±10° tolerance on ISO 17659 3.10 ("approximately right angles"). | `scene.py:175-185` | N |
| 19 | 146–148 (Tab. 2) | t U(1, 12); width U(50, 250) | Edge joints: H_B := W_A with p = 0.35 (two flush edges). Grooved stratum overrides t to U(3, 20). Outlines keep the seam edge at L but the bounding box can reach about 1.65 L (parallelogram shear ≤ 0.65 L). | `config.py:77, 363-364`; `configs/grooved_butt.yaml`; `layouts.py:208-210` | N |
| 20 | 178–180 | fixture "about 600×400×10 mm tilted by up to 10°" | Dims U(480, 720) × U(320, 480) × U(8, 12). Plate twins: assembly and fixture tilted by `rot_x(U(−10, 10)) · rot_y(U(−10, 10))`, combined up to ≈ 14.1°. Curved twins (2026-10-19): **no tilt** (z-preserving pose); the table sits under the lowest part, along a line under the saddle's main pipe (`contact_mode: "line"`). | `config.py:88-93`; `scene.py:248-256, 261-270`; `scene_curved.py:142-166, 243-254` | N |
| 21 | 191–192 | "no surface intersection is solved at generation time" | None is in the **label** path; the generation gate does intersect surfaces (`rediscover_seam`) at generation time, as l.237–238 says. | `scene_curved.py:207-218` | N |
| 22 | 229–230 | untilted, centred branch reduces to $z=\sqrt{R^2-r^2\sin^2\theta}$ | Correct when θ is measured from the main-pipe axis; with the code's φ (û = −ŷ) it reads $z=\sqrt{R^2-r^2\cos^2\varphi}$ (checked: 2e-14 mm). State which angle θ is. The general A, B, C, σ = −1 match the code (6e-14 mm on a tilted, offset case). | `curves.py:236-254`; scratchpad `saddle_check.py` | N |
| 23 | 235–236 | "Every curve has analytic positions" | The offset welds of a B-spline swept path take their normal from the finite-difference tangent, so their positions carry an O(h²) (h = 1e-6) numerical term. Negligible. | `curves.py:336-341, 493-499` | N |
| 24 | 236–237 | "Each part's ray-casting mesh records its chord error, which must stay under 0.25 mm" | Recorded per **scene** (`cloud.max_chord_error_mm` = max over parts) and gated, curved pipeline only (plate scenes record none). Mesh design tolerance 0.05 mm; the review measured up to 0.079 mm on cut rings (P3, not re-measured here). | `scene_curved.py:415-416, 476, 487-490`; `geom.py:582, 832, 1105` | N |
| 25 | 237–240 | rediscovery for circle, ellipse, saddle at 1e-6 mm | Families 5–7 (constructed) are checked instead by a containment residual under the same 1e-6 mm gate; the distance is a discrete symmetric Hausdorff over 720 samples. | `scene_curved.py:208-218`; `verify_curved.py:42-47, 77-104` | N |
| 26 | 276–277 | order: demote toes, demote cross-runs, then class seams | Code order: toes → class pass (`matches_joint_type`) → cross-run demotion (only primary seams vote; demoted with reason `too_short`; skipped for class-disjoint lap and edge). | `accessibility.py:299-328, 636-690` | N |
| 27 | 306–307 | root and gap-mid "stored alongside for plate seams" | Square-prep plate seams only; a grooved butt centreline stores `_grooveroot` instead; curved scenes store the nominal curve only. | `scene.py:424-443` | N |
| 28 | 335, 345, 362–363 | "both ends are always tacked"; degenerate tack "recorded as degenerate"; hashed phase "not tied to the curve's parametrisation" | The tacked ends are the **effective** ends at the margin m; the degenerate case is not flagged (the seam just has one tack at L/2); the closed-loop arclengths are measured from the stored polyline's first point, shifted by the hashed fraction (seed-free, but still referenced to the stored start). | `tacks.py:9-12, 107-112, 115-125, 86-97` | N |
| 29 | 380 | no seam if the largest visible length is "under 10 mm" | Null when no weldable seam has visible length **> 10 mm** (10 mm itself gives null). | `mps.py:64-66` | N |
| 30 | 422–423 | entry-only Möller–Trumbore against "0.05 mm chord meshes" | Plus a 0.25 mm self-hit floor (hits closer than 0.25 mm to the origin are ignored); 0.05 mm is the design chord tolerance. | `visibility.py:104-124, 168` | N |
| 31 | 432–433, 444 | the noise realisation is a "versioned rule"; k ∈ {0, 1, 2} | Noise carries no version string (it is `noise_model.kind = "stereo_z2"` plus stored parameters; only tacks and MPS are versioned, `tackrule-0.1`, `mps_rule-0.1`). At k = 0 the noise function is not called, so there is no grazing dropout. | `scene.py:607-617`; `harness.py:117-131`; `tacks.py:56`; `mps.py:39` | N |
| 32 | 453–454 | depth must "land on the exact primitives" | The residual is measured to the parts' D34 meshes (exact point-to-triangle distance), not to the analytic surfaces. | `weldgen/render/gate.py:84-94, 199-201` | N |
| 33 | 481 | corner/line "as T, outlines on both parts" | Corner has no in-plane yaw (only T and lap do); outlines on A and B. | `config.py:456, 481-483` | N |
| 34 | 506–518 | P and G "densified at 1 mm" | `_sample_polyline` subdivides **per segment**; the truth arrays are stored at 10 samples/mm, so G̃ is effectively at 0.1 mm (each vertex kept). Fractions are unaffected. Also: single-view scoring drops underside lap toes from G; Task 2 matches against the weldable set (`primary_only=False`). | `metrics.py:76-85, 113-130`; `scene.py:407, 419`; `dataset.py:199-201`; `harness.py:106-111, 458-497` | N |
| 35 | 540–545 (Tab. `ws_methods`) | L0 oracles | M1 and M4: a 20 mm half-width band with **no end caps** (`end_margin_mm=0`). M2: 12.7 % is the **target**; the radius is searched by bisection within 5–60 mm. M5: L0 also passes each truth seam's `closed` flag. | `lit_ransac.py:145, 152-177`; `lit_regiongrow.py:157-161, 168-206`; `harness.py:281-286` | N |
| 36 | 555 | the M3 first author "endorsed" the implementation choices | The endorsement (2026-08-27) covered the oracle masks, Mean-Shift off, the LOBB radius and the B-spline extension. It predates the 2026-09-14 audit, which changed M3's edge window. Date it. | `README.MD:573-578`; `scripts/baselines/lit_lobb.py:120-138`; `README.MD:538-540` | N |
| 37 | 561–562 | "a jitter of σ returns an error of about σ" | Expected RMSE ≈ 1.15σ (Rayleigh perpendicular part, reduced by interpolation between jittered vertices 4 mm apart). The test checks 0.2–0.55 mm at σ = 0.3 and a ratio of 2.5–4.0 for σ 1.0 / 0.3. | `harness.py:330-356`; `tests/test_harness.py:38-56` | N |
| 38 | 587 | $\lVert(\mathbf x-\mathbf q_i)_\perp\rVert$ | In the code it is the component **orthogonal to $\mathbf n_i$** (the in-plane offset, `lat`). "⊥" can be read as "along the normal": define it. Normals are unoriented PCA normals (the square removes the sign). | `weld_seam.py:183-193` | N |
| 39 | 603 | "voxel-downsample both at v (the median 4th-neighbour distance)" | v is estimated on the base scan A0 only (k = 5 query including the point itself), then used for both. | `weld_seam.py:142-147, 593-595` | N |
| 40 | 632–633 | a point of A "whose nearest point on B lies within the gap bound" | Within τ = g_max + 2h (h = max of the two part spacings). | `seamfind/pipeline.py:50-63` | N |
| 41 | 654, 658 | normal signs from "the camera position"; class "by the normal angle" | Exterior scan: signs from the stored surface normals; single view: towards the camera. "parallel" also needs the plane offset > max(0.5 mm, 0.5h). | `scripts/baselines/self_seamfind.py:41-44`; `seamfind/seeds.py:101-114`; `seamfind/pipeline.py:107-112` | N |
| 42 | 665 | torch check: 30° cone, 7 × 3 directions, 80 mm bore | Also the dihedral range 30–170° for non-coplanar seams; weldable ⇔ ≥ 1 feasible direction ∧ dihedral in range ∧ not confined. | `seamfind/pipeline.py:205-216` | N |
| 43 | 669–671 (TODO) | defaults differ from the plan "in 17 documented ways", chosen on a 12-scene smoke round | The README table has 17 numbered rows plus 11b and 15b (19 changes); only choices 5, 6, 8 and 12 were made on the smoke round, the others are argued from corpus facts. | `seamfind/README.md:39-59, 97-98` | N |
| 44 | 683–690 | the cell "applies the tack rule unchanged" and judges curved seams by WeldSet's verdict | Outside weld_generator: the cell calls `seam_verdict` with a 200 mm bore minimum (its tool), not 80 mm; it tacks **every weldable** seam (`matches_joint_type` forced True); it passes `primitive = "sweptslab"` and no `params`, so `_member_gauge_mm`'s swept-band correction (`"swept_slab"`, z0/z1) never fires (harmless for a stiffener, wrong for an arc-butt-like band). | cell: `admittance_control/curved_seams.py:37-40, 120-128`; `admittance_control/seam_from_registration.py:429-445`; `tacks.py:128-142` | N |
| 45 | 720–723 | twenty scenes, four per type, briefed, blind to the truth | Laptop manifest: scenes from `out/bench6a` (not the final benchmark), 20 scenes, 30 primary seams; the briefed arm is told each scene's joint type (no truth). Name the source corpus. | `scripts/annotation/export_for_annotation.py:19-21, 114-117, 139-146`; laptop `out/annotation/manifest.json` | N |

## 2. Claims confirmed

- Eight named substreams spawned from one seed (SeedSequence.spawn, PCG64), appended never reordered: `rng.py:13-14, 23-32, 43-49`.
- Scene id = config id (first 8 hex of sha256 of the canonical resolved config) + seed: `hashing.py:72-74`, `scene.py:543-547`.
- Skipped seeds recorded with their reason, never backfilled: `writer.py:116-129`, `make_bench6b.py:23-25, 131-143`.
- Primitive registry slab / prism / prepared_slab / prepared_prism / tube / swept_slab, with the parameters of Table `ws_primitives`: `geom.py:548-580, 797-830, 2032`; `scene.py:124-149`.
- w is the thickness axis; broad faces ±w: `joints.py:154-157`, `geom.py:810-813`.
- t U(1, 12), second part equal with p = 0.7; edge t U(1, 2) (ISO 9692-1 ref 1.1): `config.py:61-62, 365-376`.
- Along-seam length U(80, 400) per part, offset ±0.4 min L: `config.py:32-36, 357-359, 443-444`.
- Width U(50, 250): `config.py:60, 360-362`.
- Included angle U(60, 120) for T and corner; 180 butt; 0 lap and edge: `config.py:64, 378-384`, `joints.py:88-89`.
- Lap overlap U(0.15, 0.60) × width of the upper plate B: `config.py:67, 387-388, 508`, `layouts.py:613-628`.
- Gap B–D U(0, min(no. 617, 2 mm)); below D U(D limit, max(1.5 D limit, 3)) → ≤ 5.28 mm at t = 12: `config.py:197-209, 408-425`.
- h by no. 5071; β by level; below D ×U(1, 2); β ≤ 0.4° on lap and edge: `config.py:188-194, 212-219, 412-418, 433-434`.
- Quality target 25 % each, drawn first in the defects stream; realised level = worst of the three: `config.py:79, 290-316, 394-396, 515`.
- Groove rows 1.9.1, 1.3/1.5, 1.6/2.6 with bevel, root face, gap, radius drawn in the row: `config.py:222-287`.
- D28 gate: length-weighted 10° histogram of free-edge angles (edges ≥ 12 mm from the seam), terminal bins [0, 10) and [80, 90], threshold 2 × 20/90 = 0.444: `scripts/qa_d28_gate.py:86-88, 312-331, 387-410`.
- Gate history 0.45 FAIL → 0.34 PASS (2026-09-07): `notes/dataset_plan.md:2049-2070`.
- Convex outline vocabulary with the seam edge pinned full length: `layouts.py:169-247`.
- Fixture face pairs rejected as `fixture_contact` (D13): `accessibility.py:351, 366, 427-428, 615-616`.
- Fixture twins replay the emitted seeds with the fixture on: `scripts/make_fixture_twins.py:1-20, 41-60`.
- Seven D29 families; curve first, parts derived: `d29.py:48-56`, `constructors.py:1-20`.
- Family draws of Table `ws_families` (circle r U(30, 80); ellipse b U(30, 70), a = b/cos τ; saddle R U(40, 80), r U(max(15, 0.3R), 0.7R), tilt U(0, 30°), offset ±0.4(R − r); rounded rect w, h U(60, 160), r_c U(8, 0.25 min); swept t U(4, 10), H U(50, 160), arc or clamped cubic B-spline; arc butt R U(100, 400), edges at R ∓ g/2): `d29.py:72-168`, `constructors.py:60-221`, `curves.py:292-331`.
- Saddle A, B, C, σ = −1 as in Eq. `ws_saddle`: `curves.py:236-254` (re-derived and checked numerically).
- Swept welds = offsets at ±t/2 along ẑ × T̂; the spine is mid-material: `curves.py:476-502`, `constructors.py:181-198`, `verify_curved.py:211-215`.
- B-spline tangent by central difference: `curves.py:336-341`.
- Chord gate 0.25 mm: `scene_curved.py:487-490`, `qa_d28_gate.py:83`.
- Rediscovery gate 1e-6 mm, observed 1e-13 mm: `scene_curved.py:61-63, 208-213`, `verify_curved.py:50-74`.
- Bore rule 80 mm; an open 50 mm bore clears the local cone: `accessibility.py:45-51`, `verify_curved.py:244-247`.
- Parallel test at 8°: `accessibility.py:110-111, 345, 353`.
- Intersecting arm: line clipped to both faces with slack τ_c, dropped if not mutually visible or farther than τ_c: `accessibility.py:372-423`.
- Coplanar arm: centreline of the in-plane gap, gap within tolerance: `accessibility.py:492-610`.
- Facing pairs: no line, `bisector_blocked`: `accessibility.py:358-369`.
- Dihedral [30°, 170°], minimum length 10 mm (intersecting arm): `config.py:144-149`, `accessibility.py:429-432`.
- Torch axes ϑ ∈ {0, 15, 30, 45}°, φ = 2πk/8, positive component along both normals, 30° cone with axis + 8 rim rays, probes at 3.75 / 7.5 / 15 mm, five points along the seam, first clearing axis stored as approach: `accessibility.py:164-259, 436-438`, `scene.py:449`.
- Effective nook floor 60°: acute fillet blocked at 56–60°, weldable at 61° (scratchpad `nook_check.py`, real `layouts.build` + `enumerate_candidates`).
- Seam classes: two broad faces → fillet, or butt above 172°; two edge faces → edge; one each → lap toe: `accessibility.py:107-108, 142-155`.
- Primary = weldable ∧ in-class: `accessibility.py:137-140, 315-320`.
- Nominal stored and scored, continuous as g → 0 and symmetric in A ↔ B: `docs/SCHEMA.md:139-154`, `scene.py:584`.
- δ_root = H_e / sin φ from the D19 closest-point construction: `accessibility.py:449-476` (geometry), review C7 §8 numeric check.
- Tack rule ℓ = clip(4t, 10, 50), m = max(2t, ℓ), d_min = 10t, d_max = min(33t, 400); n = max(2, ⌈L_e/d_max⌉ + 1) with the d_min lowering; s_k = m + kL_e/(n − 1); one tack at L/2 if L − 2m ≤ ℓ; closed n = max(4, ⌈L/d_max⌉) rounded up to even, d_min lowering not below 4, phase from the first 8 bytes of sha256(scene_id:seam_id) / 2^64, opposite pairs in bisection order, same-class seams round-robin: `tacks.py:3-36, 58-125, 145-184`.
- t = thinnest member gauge, swept band min(band width, height): `tacks.py:128-151`.
- Only primary seams are tacked: `tacks.py:45, 155-157`.
- Tack constants' provenance (JASS 6 via Kobelco, EN 1011-2 attributed, Tomków et al.): `tacks.py:38-43`.
- MPS rule (largest visible arc length over weldable seams, fold, lower id) and margin ℓ(2)/ℓ(1): `mps.py:8-96`.
- 1280 × 720 pinhole, principal point at the centre, profile drawn from three: `config.py:111`, `camera.py:77-85`, `scene.py:314-315`.
- Pose equation, e U(15, 85), α U(0, 360), roll ±15, φ U(0.35, 1.45), E = world-AABB longest side, clip to [max(z_min, 300), 1200]: `camera.py:49-74`, `scene.py:316-338`, `config.py:99-110`.
- Plate aim point, jitter 0.15 in a cube: `scene.py:76-96, 321`, `config.py:135`.
- Sensor profile values: `config.py:19-23`.
- Area-uniform sampling at U(0.25, 4) pts/mm², exact normals, object and face ids: `config.py:96`, `sampling.py:22-95`.
- Visibility = in frame, beyond z_min, front-facing, unoccluded: `visibility.py:301-328`, `camera.py:104-113`.
- Exact slab and convex-prism clipping; entry-only Möller–Trumbore for tubes and swept parts: `visibility.py:31-82, 104-232, 265-282`.
- Exterior flag: normal + 8 rays at 38.5° + 8 at 66.5° (0.55 and 0.95 × 70°): `visibility.py:419-479`.
- Exterior → full-scan input, visible → single view: `scripts/baselines/dataset.py:139-153`.
- Noise model and dropout; d435i 0.594 mm at 0.5 m, 2.374 mm at 1 m: `noise.py:25-71`.
- Noise multiplier k on subpixel and lateral sigma: `harness.py:113-131`, `run_phase4_batch.py:115-121`.
- Twin gate p99 < 0.25 mm, coverage ≥ 0.90, object ≥ 0.98: `weldgen/render/gate.py:25, 193-217`.
- The three ray-test defects found by the twin gate and fixed: `notes/phase8_plan.md:270-345, 543-575`, `visibility.py:85-101, 104-170`.
- Reflectances from RTR4 Table 9.2: `weldgen/render/materials.py:1-50`.
- Twelve strata × 60 (`--per-family 60`): `make_bench6b.py:8-14, 29-37, 75-92`.
- Strata table ranges (as coded), incl. grooved t 4–20 because t ≤ 4 draws are square and filtered: `configs/grooved_butt.yaml`, `config.py:235-236`, `make_bench6b.py:70-71, 86`.
- Precision against exact segment distance, recall over truth samples to the nearest predicted point, τ = 3 mm, predictions densified at 1 mm: `metrics.py:113-130, 338-351, 381-397`, `harness.py:417-419`.
- Path metric: greedy one-to-one match by mean distance, 0.25 mm resampling, RMSE and maximum: `metrics.py:218-290`.
- Oracle ladder L0/L1: `harness.py:16-33`, `run_phase4_batch.py:30-33`.
- Method mechanisms and L0 oracles M1–M7: `harness.py:207-327`, docstrings of `lit_ransac.py`, `lit_regiongrow.py`, `lit_lobb.py`, `lit_ppf.py`, `lit_pcaslice.py`, `lit_modelreg.py`, `lit_quadric.py`.
- Fidelity audit: six readings corrected, old readings kept as rungs: `README.MD:531-545`.
- Conditions: L1 on the exterior scan only, noise ×1 and ×2 on the single view, 8 fixture pairs per type (40), 30 seeds for lit-ransac: `run_phase4_batch.py:100-180, 210-222, 327`.
- Fake-oracle phantom and miss behaviour: `tests/test_harness.py:59-86`.
- Eq. `fp_distance` (k = 4, r_s = 0.75v) and Eq. `footprint` (g_fp = 3v + g, strict <): `weld_seam.py:183-193, 600-602, 632, 849-850`.
- Footprint algorithm steps (ICP optional, A/B split, secant slide to D = g, exposure 3v, clustering on position + opening, thinning, MST longest path, smoothing spline, legs, θ, bisector, rolling ball, corner split, classes): `weld_seam.py:267-300, 587-759`.
- Rolling ball: first h with min(d_A, d_B) ≥ r; h = r/sin(θ/2), legs r cot(θ/2) (geometry verified): `weld_seam.py:415-437, 714-715`.
- Seam-finder walker equation, α = 0.9, stops at 0.05h, 0.01h and the patch edge: `seamfind/walk.py:23-72`, `seamfind/config.py:59-62`.
- Seam-finder pipeline steps of Alg. `ws_seamfind` (per-part spacing, flattest-neighbourhood normals, MLS, one-way NN seeds, coplanar pairs, crease-free refit, root rules, DBSCAN + normal split, MST order, 2 mm resampling, cross-run and toe rules, 7 × 3 torch check, 80 mm bore, weldable spans): `seamfind/pipeline.py:26-221`, `seamfind/geom.py:38-133`, `seamfind/seeds.py:50-114`, `seamfind/access.py:101-142`, `seamfind/walk.py:85-142`.
- Annotation: 20 scenes, 4 per type, anonymous truth-free PLY, CloudCompare, 4–10 clicks, 15 mm gate, synthetic annotator (σ = 0.8 mm, one miss, one phantom); laptop manifest: 30 primary seams: `export_for_annotation.py:12-40`, `score_annotations.py:107-129`, `tests/test_annotation.py:52-108`.
- Cell side: pose tolerance replaces contact tolerance; curved seams by the verification factories + `seam_verdict`; tack rule reused: cell `seam_from_registration.py:98-117, 412-445`, cell `curved_seams.py:1-30`.
- Phase 3 yields (T 79 %, butt 57 %, corner 44 %, edge 42 %, lap 36 %): `README.MD:290-291`.

## 3. Needs the desktop data

- Corpus sizes, attempts and yields per stratum of the final `out/bench_phase4` (2026-09-12 entry-only rebuild); the laptop copy is stale.
- Emitted ranges per stratum (laptop values in review C §7): ellipse tilt 10.2–30.6°, lap overlaps and edge width differences after the D31 rejections, gap maxima, T-joint vs angle-joint counts.
- Fixture twins: 634 = 274 plate + 360 curved; which 40 pairs the final batch's fixture chunks used (expected: T/line, butt square/grooved, corner, lap, edge, since the curved twins postdate the batch).
- Weldable bores (review: 95 of 240; circle 43, ellipse 31, saddle 0, rounded rect 21).
- D28 value on the final corpus (0.34 plate; swept path 0.4501, ungated) and the D34 worst chord.
- train_v1 (3600 scenes × 10 views), twin-gate statistics per stratum (final 12/12 PASS).
- Final batch rows (271 200), M1 reproduction (RMSE 0.60 / max 0.81 mm).
- Self-footprint and self-seamfind results (nb16/nb17 csv); check whether any table reports θ_r (finding 1).
- Annotation scores (laptop copy `out/annotation`, source corpus bench6a): confirm against the desktop copy.
- Hash reproducibility under the pinned numpy/trimesh versions.
