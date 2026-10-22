# WeldSet code review, 2026-10-20: what the thesis review found in `weld_generator`

A read-only review of `weld_generator` (eight reviewers: the generator core, the curved families,
the sensor model, the schema and tests, the seven literature methods, the harness and the
batch) found defects in the code, not only in the documents. Nothing was changed. Each item says
what is wrong, where, how big it is, and what it touches. Line numbers are as of 2026-10-20,
branch `tack_welding`. The raw reports are in `review_2026-10-20/` (C_*.md).

**Decide before the thesis numbers are frozen:**
- which of P1 to fix;
- whether to regenerate the affected strata and re-run their batch chunks on the desktop;
- whether the ICRA submission needs a correction (it uses Task-1 edge/lap numbers, the fixture
  table and states that the parts are disjoint).

## P1: changes data or ground truth

| # | Defect | Where | Size (measured) | Touches |
|---|---|---|---|---|
| 1 | **Exterior flag wrong on thin stacked plates.** The exact grazing-cone test runs on the first point of each 2 mm voxel (part A first) and every other point copies its nearest representative, so plate B's open top face inherits A's buried interface | `weldgen/visibility.py:489-510` (`exterior_scan_subsampled`) | Misflagged points: edge 12.4 % (worst scene 25.9 %), lap 2.4 % (worst 16.4 %), T/line 0.36 %, corner 0. Upper top face, which must be 100 % exterior: edge 0.60, lap 0.94 | The Task-1 `full_exterior` input of edge and lap (and, slightly, T). Task 2 (`single`) is unaffected. The exact scan costs about 0.25 s per plate scene |
| 2 | **The two parts interpenetrate.** The rotation hinge is on the wrong side: an acute T (α < 90°) sinks B's far bottom edge by $t_B\lvert\sin\theta\rvert$; on edge and lap joints β > 0 rotates B's far side into A | `layouts.py:280-284`, `joints.py:161-167`, `layouts.py:627, 649` | T: 28.3 % of specs, median 1.0 mm, max 5.4 mm (corpus 18/60 T scenes, max 3.1 mm). Edge 36/60 scenes, lap 10/60 (max 0.31 mm). Rare phantom primary seam on A's underside (≈ 0.75 % of T specs) | Contradicts D21 and SCHEMA §6.3 ("disjoint at g > 0"); buried points stay in the cloud; the acute-side fillet has root = gap-mid = nominal |
| 3 | **Coplanar-arm comparison mixes axes**: `if best is None or (hi - lo) > best[4] - best[3]:` (intended `best[5] - best[4]`) | `accessibility.py:572` | Only with non-Slab A (outlines on): 7 of 234 stored butt centrelines skewed 0.05-1.19°; 3/60 grooved scenes have no mouth centreline | Square and grooved butt strata |
| 4 | **Plate fixture twins are not viewed from the same relative pose.** The fixture-height draw precedes yaw and tilt in substream 3, so on/off twins get different assembly yaw while the camera is identical | `scene.py:250-256` | e.g. seeds 11-13: yaw 28.8° vs 98.9°, 21.6° vs −129.5°, −170.3° vs 160.5° | The fixture-price table (the effect mixes fixture and viewpoint). Curved twins (2026-10-19) are correct |
| 5 | **Grooved butts**: (a) the ISO 9692-1 row is chosen at min(t_A, t_B), then t_B := t_A, so a plate can be thicker than its row; (b) every grooved scene ships two primary butt seams (top/mouth and underside) with the same `_grooveroot` | `config.py:491-498`; `scene.py:424-439` | (a) 714 of 6000 draws (145 bevel plates > 10 mm, 128 V plates on row 1.3 at t > 10); 2 of 60 corpus scenes. (b) 57 of 60 scenes | Grooved stratum; contradicts "one seam at the groove root" (D36, PARAMETERS §2.6.1) |

## P2: wrong metadata or a range the text must state differently

| # | Item | Where | Note |
|---|---|---|---|
| 6 | Arc butt stores the band **width** W (50-150 mm) as thickness, so quality level is "B" for all 60 scenes and the throat is 0.7 W | `scene_curved.py:413`, `SweptSlab.thickness_mm` | facts.csv and the tack rule use the true gauge; only metadata is wrong |
| 7 | Quality level applies the fillet no. 617 gap limit to **every** joint type (butt, edge, grooved too) | `config.py:290-316` | PARAMETERS §2.4 says 617 does not apply to butt/edge |
| 8 | Effective dihedral floor is **60°**, not 30° (the 30° cone blocks any nook ≤ 60°); obtuse T joints lose the obtuse-side fillet silently (α > 95°: 48 % have one primary fillet, 5 % none) | accessibility cone | State the 60° floor; explains the ellipse coverage below |
| 9 | T/ellipse tilt is drawn U(10°, 40°) but the corpus holds **10.2-30.6°** (all 36 seeds above 30.6° rejected: acute-side dihedral < 59.4° fails the cone; the curved verdict has no ±45° work-angle fallback) | `d29.py:88`; `verify_curved.py:253-271` | Report the emitted range |
| 10 | D19 `gap_mid` has three definitions (SCHEMA closest-pair; pipeline per-face closest point; Phase 1 root projection); they agree only at 90° | `accessibility.py:449-476`, `joints.py:214-216` | Pick one for the text |
| 11 | Stored defects not built into the geometry: `h` unused by lap and edge layouts, β unused by grooved butt, `h` shifts B along y for T | `layouts.py:503, 552-584, 613-650` | Stored in `fit`, used in quality and contact tolerance |
| 12 | D28 axis prior on open curved seams: swept_path terminal mass **0.4501** (reported, not gated) is above the plate threshold 0.4444 | `qa_d28_gate.py` | "cannot arise by construction" does not hold for family 6 (spines along x on an axis-aligned plate) |
| 13 | Not every rejected candidate is stored (facing pairs, adjacency failures; `no_contact`/`interior_face` never emitted) | `scene.py:411-414` | Soften "a rejected candidate is kept with the reason" |

## P3: documentation and claims (fix the docs, or word the thesis carefully)

- **Determinism** holds only with pinned numpy and trimesh (all hashes changed numpy 2.3.5 → 2.5.3
  on 2026-09-11; `requirements.txt` unpinned). SCHEMA §6.2's hash pseudo-code differs from the code.
- **RNG substreams 6 (noise) and 7 (render) are never used**: the noise is seeded by the scene seed
  (`noise.py:48`), so strata that share a seed get correlated noise; renders use
  sha256(scene_id | render_id).
- `camera_raster` is uniform density at ρ = (f/s)² (s = standoff), not range- or incidence-dependent.
- Grooved plates are occlusion-tested by K = 9 sampled containment, not exactly (3 misses in 19 937 hits).
- Grazing angles: the exterior cone is 70° but its rays sit at 0°, 38.5° and 66.5°; the noise
  dropout is 75°. They are not "the same quantity".
- No sensor-profile twins exist in any corpus; six presets named in PARAMETERS §6 do not exist.
- D34 "measured 0.05 mm" is the design tolerance / analytic estimate; true deviations reach
  0.079 mm (tube cut rings) — still far under the 0.25 mm gate.
- Version strings disagree (SCHEMA header 2.3.0, code 2.4.0, scene.schema.json 2.2.0, `__init__`
  docstring 2.0.0, PARAMETERS 2.2.0); the semver policy was not applied 2026-08-17 .. 10-19.
- `tests/schema_contract.py` is stale (standalone; all 5 positive cases fail).
- The metric densification is 1 mm in code; the ICRA text says 0.5 mm.
- The final batch is **271 200 rows** (nb15, ICRA `\nRows`); README's 268 320 is stale.
- MANUAL still says "six methods" and "460 tests" (README: seven, 564).
- The swept-band face registry, the #5 band side, the primitive list (no ellipse spine; a
  `cylinder` primitive that is never emitted) and several ranges in PARAMETERS / phase9_plan §1
  disagree with the code (the corrected ranges are in Chapter 4's strata table).
- `ellipse`/`saddle` closed forms: the cell uses WeldSet's **verification** constructors
  (`ellipse_from_plane_cylinder`, `saddle_from_cylinders`), not the curve-first ones.

## Smallest fix set that protects the thesis numbers

1. Exact per-point exterior scan (item 1); regenerate the `exterior` arrays of edge, lap and T on
   the desktop; re-run the Task-1 (`full_exterior`) chunks of those strata.
2. Hinge on the correct side, or a non-penetration check that rejects the draw (item 2);
   regenerate the affected plate strata; re-run.
3. The one-line fix at `accessibility.py:572` (item 3).
4. Append the fixture-height draw after yaw/tilt for plates (item 4), as for curved twins;
   regenerate the plate fixture twins; re-run the fixture chunks.
5. Emit one grooved seam and choose the row at the final thickness (item 5).

Then compare the old and new tables stratum by stratum and say in the thesis what changed.

## Statements in the ICRA submission that do not match the code or the final batch

From the harness/batch review (C6 §7, "High"). Fix in the camera-ready if accepted, and do not
carry them into the thesis.

*Checked against the paper source on 2026-10-22* (`weld_generator/paper/icra2027/submission/root.tex`,
identical to `main.tex`, 2026-09-18): items 1-10 are sentences of the paper (lines 79, 108, 174, 79
and the Fig. 3 caption at 142, 149, 149, 206, 134, 194, 108). Items 11 and 12 are not in the paper:
11 is MANUAL §8.3, 12 is MANUAL §9 and notebook 15. One more sentence of the paper does not match its
own table (C6 #20): "\M7 localises the named seam within 1.5 mm on every class" (line 172), while
Table II gives 1.55 mm for T.

1. Densification: the paper says 0.5 mm; the code densifies at 1.0 mm (matching at 0.25 mm), and
   matching has no tolerance (only F1 does).
2. "0.81 mm mean error" for M1: 0.81 is the **maximum** error (ME = max d).
3. "Every method runs in well under 4 s per scene": `cost.tex` shows M6 at 5.01 s, M3 L1 at 10.42 s.
4. Task 1 truth set: the paper says "every weldable seam"; the code scores **primary seams only**,
   so off-class weldable seams count as false positives (64 seams in 29/60 butt scenes).
5. "M2's crop carries more than any other in the ladder (0.58 → 0.11)": M7 drops 0.87 → 0.00 and
   M5 0.64 → 0.00.
6. "Corners and butts return nothing" for M3: square butts give 0.27, arc 0.49.
7. Our radius-PCA "below 0.05 precision on edge joints": from an older full-geometry measurement;
   nb12 (full exterior) shows 0.199, and it has no final-batch rows.
8. "Both curved butt strata": there is one (arc).
9. "Fillets 1.3 mm from the root against 2.0 mm": T only (4 scenes); corners give 1.00/1.03/0.96.
10. M3 corner reproduction "median 0.51 mm RMSE": the pre-audit value; after the audit the test
    pins RMSE 0.112 mm on one corner scene while benchmark corner F1 is 0.00.
11. Batch runtime "~64 h on a 16-core laptop": the first run (2026-09-05); the final run was about
    10.5 h on the desktop.
12. MANUAL §9 and several notebook-15 prose cells are pre-audit (e.g. "plane intersection scores
    zero on every curved family" — ppf 0.27-0.37; lobb "only method non-zero" — false).
