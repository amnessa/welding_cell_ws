# seamfind — the `seam_finder.md` method on the weld_generator benchmark

Matching-based seeding plus sphere-tracing refinement (`../seam_finder.md`), implemented as
a package with one module per stage and run on the **tier-1 analytic corpus**
(`out/bench_phase4`, 720 scenes, 12 strata) through the Phase 4 harness
(`scripts/baselines/self_seamfind.py`, `scripts/run_self_seamfind.py`). No rendered data is
used. Stage 0 of the plan (the synthetic scene generator) **is** the benchmark generator, so
it is not re-implemented here; Stage 6 (rolling-ball spine) is not implemented yet.

```
seamfind/
  config.py    Params — every default, with the reason where it departs from the plan
  geom.py      Stage 1 — spacing, voxel downsample, per-part oriented PCA normals, MLS Π(x)
  access.py    Stage 2 — 2a cone prefilter (sphere tracing on the cloud), 2c torch cone,
               bore confinement
  seeds.py     Stage 3 — mutual NN (KD / N×M matrix on FPS), one-way NN, coplanar pairs (3b),
               θ_n, joint class
  walk.py      Stage 4 — walkers, θ_r, root point, crease-free supporting planes
  assemble.py  Stage 5 — clustering, MST ordering, spline, toe suppression
  pipeline.py  extract(A, B, Params, ...) -> Result(seams, seeds, h, timings, counts)
```

## How the benchmark supplies the inputs

| plan input | from the benchmark |
|---|---|
| clouds A, B with labels | the condition's cloud (`full_exterior` = perfect multi-view scan, `single` = one camera) split by `object_id` — the "objects" oracle, the same input lit-lobb gets at L0 |
| normal orientation | `full_exterior`: sign of the stored surface normals (plan: "synthetic: from the mesh"); `single`: towards the camera (plan: real-scan rule). Only the sign is used — the normals themselves are always PCA estimates |
| torch model | the generator's accessibility block: clearance cone half-angle 30°, standoff 15 mm, work angle ≤ 45°, dihedral 30–170°, bore ≥ 80 mm |
| ground truth | SCHEMA D19 `nominal`: the intersection of the two extended supporting planes (fillet / lap toe), the gap centreline (butt / edge) — which is what the plan's root-point rule computes |

## Where the plan was changed to fit the dataset

The plan's numbers were written without the five ISO standards the generator follows
(ISO 17659 joint terms, ISO 9692-1 preparations, ISO 5817 quality levels, ISO 2553, ISO 13920).
Each change below is a parameter or a named option, so the plan's version stays runnable
(`run_self_seamfind.py` arm `plan`).

| # | plan | here | why (corpus fact) |
|---|---|---|---|
| 1 | gap ≤ 3 mm, larger = no seam | `gap_max_mm = 5` | ISO 9692-1 gap ranges with the ISO 5817 below-D tail: drawn gaps reach 4.9 mm and are still seams |
| 2 | h = 1 mm | estimated per part | densities 0.25–4 pts/mm² → h = 0.5–2 mm |
| 3 | PCA k = 25, MLS r = 3h | + oriented-normal gate, + flattest-neighbour normal | plates go down to 1.0 mm (ISO 9692-1 square prep; edge joints 1–2 mm): a 3h ball holds both faces of one part |
| 4 | FPS N = M = 1500 + N×M matrix | NN on all points via KD-trees (identical rule) | 1500 samples on a 200×266 mm plate are ~10 mm apart, more than τ_d |
| 5 | mutual NN | **one-way NN** (the plan's own ablation) is the default | mutual pairs come out one per 2–3 mm per side; the walkers pull the one-way band onto the toes (smoke: T-line F1 0.42 → 0.98) |
| 6 | θ_r vs θ_n agreement drop (10°) | off by default | θ_r is planar-only (plan says so); on tube / swept seams it drops most seeds |
| 7 | 2a: start at p + εn, hit if d < h | hit only if the near point is **ahead** of the ray | at the root the other part is within h of every start point |
| 8 | 2a: τ_vis = 0.3 | ≥ 1 free ray (0.04) | generator exterior rule = "some ray in the grazing cone escapes"; a 217 mm deep bore's floor seam has ~1/24 free rays and is weldable in the truth |
| 9 | θ_n ≈ 0 → butt | + same-plane test (`coplanar_tol_mm = 0.5`) | a lap's two bottom faces are parallel but one plate thickness apart |
| 10 | — | Stage 3b coplanar boundary pairs over 25 mm | an ISO 9692-1 groove opens the top faces up to b + 2t·tanβ ≈ 25 mm; the stored butt seam is their centreline |
| 11 | walker step on MLS plane | + support test (foot within 0.75 h of a sample): off the samples the walker stops, and the closest point on the other part falls back to its nearest sample | an MLS plane is infinite: it would carry the walker across a butt gap, and at 1.5 h it extrapolated a stem 1 mm down to the plate (gap read 0.03 for 1 mm) |
| 11b | step = α d along the tangent | step = α × the **tangential** part of (q − x) | with B hovering a gap above A the full distance exceeds the in-plane distance to B's foot, and the plan's step overshoots the toe — its own never-overshoot test fails without this |
| 12 | root from the toes' MLS planes | supporting planes refitted away from the crease (`crease_free="interior"`) | at sub-mm gaps the exterior scan keeps a 1–2-sample strip of the buried face (tube foot annulus) with unrecoverable normals; D19 truth is the extended-plane intersection |
| 13 | DBSCAN eps 4h on positions, then split by normals | the same, eps ≥ 3 × seed spacing, normal split by a second DBSCAN | (a first version mixed normals into the features and fragmented seams) |
| 14 | MST longest path, project off-path points | each path vertex = mean of the seeds projected onto it | a one-way seed band sorted by arclength zig-zags |
| 15 | — | toe suppression (ISO 17659 `toe_of_centreline`) | the lines bounding a butt / edge gap are toes, not seams |
| 15b | — | cross-run rule (SCHEMA 2.6.2): straight seams > 45° off the length-weighted joint direction are dropped | the short runs across the plate at the ends (flush end faces) are real geometry but not seams |
| 16 | capsule torch check | generator cone + bore-confinement ring test | the generator rules a bore < 80 mm unweldable (`confined_bore`) |
| 17 | output all seams + flags | output = weldable spans (`output="all"` keeps everything) | the plan's output is weldable / not-weldable spans; truth holds weldable seams |

## Tests

`tests/test_self_seamfind.py` — the plan's per-stage tests on analytic grids: MLS error < 0.1 h
and normals < 1° on a plane; per-part normals < 3° at 1 h from a corner; walker d_k
non-increasing and gap error < 0.25 mm at a 1 mm gap; a T-joint gives exactly its two fillets
at the nominal line with a 90° dihedral; pipe on plate gives one closed seam at the right
radius; one deterministic harness run on the corpus.

## Smoke round (one scene per stratum, median F1 @ 3 mm)

| arm | full_exterior | single |
|---|---|---|
| default (this package) | 0.72 | 0.49 |
| no crease-free refit | 0.64 | 0.50 |
| the plan as written (mutual NN, θ_r test, τ_vis 0.3) | 0.09 | 0.33 |

## Full run (720 scenes, mean of the 12 stratum-median F1 @ 3 mm)

| arm | full_exterior | single |
|---|---|---|
| default | **0.78** | **0.60** |
| nocrease | 0.59 | 0.59 |
| plan | 0.03 | 0.25 |

Per-stratum tables, the comparison with the seven literature methods and `self-footprint`, path
RMSE, gap and angle errors: `notebooks/17_self_seamfind.ipynb`.

## Known limits

* **Thin sheets** (t ≈ 1–1.5 h): the end faces of 1 mm plates are one or two samples wide; their
  normals cannot be estimated, so lap and edge joints keep false positives in full view.
* **Grooved butts, single view**: the top-face centreline is found only when both top faces are
  in view; the groove-root line is not the stored seam and is not reported.
* **Single view, curved seams**: recall ≈ 0.3–0.4 because half of a ring is out of view — the
  same ceiling every method has in that condition.
* Gap estimate biased upwards (median +0.3 to +1.8 mm): toe-to-toe at the walkers' stop points.
* Default choices 5, 6, 8, 12 were made on the 12-scene smoke round; the full run confirms the
  default over `nocrease` and `plan`, but a held-out split is the right protocol for further tuning.
