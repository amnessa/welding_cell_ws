# weld_generator — the manual

*How the project works, why it is built the way it is, and how to explain it to someone
else. Written 2026-09-08 at the close of Phase 7. The README is the status page; this
is the teaching document. The frozen contracts are `SCHEMA.md` and `PARAMETERS.md`; the
decision log is `notes/dataset_plan.md` (the D-series). Nothing here overrides those —
where they disagree, they win and this file is wrong.*

---

## 0. The one-paragraph version

We built a generator of synthetic 3D scenes of parts waiting to be welded — two plates
forming a T, a pipe standing on a plate, a curved stiffener — in which the **weld seam
is known exactly**, because the scene was *constructed* from the seam outward rather
than the seam being *found* in a scan. That gives a benchmark where the answer is exact
to the micron, where every difficulty axis (joint type, curvature, groove, thickness,
fixture, camera view, sensor noise) is a dial we set, and where the same seed with one
dial changed gives a bit-identical twin. On that benchmark we reimplemented six
published seam-extraction methods and measured, for the first time under equal
conditions, which geometric mechanisms can express which joints. The headline: the
dominant mechanism in the literature (intersect two fitted planes) scores zero on
every curved joint, the one mechanism that survives curvature (a crease detector) pays
for it in precision, and the label noise of the hand-annotation protocol the field
scores against is 1,4 mm — more than twice the accuracy those papers report.

---

## 1. The problem, and the point

### 1.1 What "seam extraction" is

A welding robot needs the 3D curve along which two parts meet — the seam — to plan the
torch path. Vision systems recover it from a point cloud (a depth camera on the wrist,
a laser line scanner, a multi-view scan). The literature reports sub-millimetre
accuracy for this.

### 1.2 What is wrong with how that accuracy is measured

Three things, and the project answers each:

1. **The ground truth is not exact.** Published methods are scored against a
   hand-taught path or hand-clicked points in a scan. Yi et al. (the `lit-ransac`
   paper) measure their own truth's error budget at ~0,5 mm and then report 0,64 mm
   RMSE against it. Phase 5 of this project measured the same protocol with an
   independent annotator on our scenes: **1,4 mm median lateral error, 11,8 mm at the
   95th percentile.** A reported 0,6 mm against such labels is a statement about the
   labels, not the method.
2. **The conditions are not varied.** Each paper evaluates on its own handful of
   workpieces, one sensor, one fixture setup, straight seams. Nobody reports what
   happens on a curved seam, with the fixture in view, at twice the sensor noise, or
   with a different RANSAC seed — because they cannot: their truth is fixed to their
   scans.
3. **The datasets are not available.** "Available on reasonable request" is the norm.
   A benchmark others cannot download is not a benchmark.

### 1.3 The answer: construct, never detect

The project's one rule (`dataset_plan.md` §1): **ground truth is constructed, never
detected.** The generator samples a joint (type, thickness, gap, misalignment, angle),
places the parts by closed-form transforms, and the seam is *the intersection line of
two face planes* — computed algebraically from the same transforms, exact to floating
point. There is no annotator, no estimator, no "consistency" anywhere in the label
path. Everything else follows from this rule: what is stored, what is a convention,
what is a gate.

### 1.4 The two tasks

- **Task 1 — complete seam recovery.** Input: a perfect multi-view exterior scan of the
  assembly (`full_exterior`). Output: every weldable seam as a polyline. Scored by
  F1 at a 3 mm tolerance over densified polylines, Chamfer, and the literature's own
  matched-path RMSE.
- **Task 2 — the most probable seam (MPS).** Input: what *one* camera returns
  (`single`). Output: the one seam a coarse-positioned torch is at. The label is a
  versioned rule (§10), a geometric proxy by decision.

### 1.5 The tiers

- **Tier 1** (this repo, complete): analytic geometry, ray-cast visibility, an
  analytic stereo-noise model. No renderer, no simulator, no GPU — `trimesh` + NumPy
  only (decision D9: a generator that needs Isaac Sim and an RTX card is as
  inaccessible as the datasets it criticises).
- **Tier 2** (Phase 8, next): the same scenes rendered (RGB + depth) with materials,
  lighting and structured-light failure modes — a pluggable backend behind the same
  schema, paired to tier 1 by seed.
- **Real subset** (Phase 9): scanned MDF workpieces at known poses, the reality check.

---

## 2. The design philosophy — five habits you will see everywhere

**Truth vs convention (D8, D14, D19, D25, D38).** Only exact geometry is stored as
truth. Everything that is a *choice* — where the seam is when there is a root gap
(three candidate curves), which seam is "most probable", where the tacks go, what
sensor noise looks like — ships as a **versioned rule function** over the stored
geometry, with its parameters in the file. Nobody argues with a convention baked into
labels; they re-run the rule with different parameters. This is why the tack and MPS
blocks could be added to a finished corpus in 63 seconds without regenerating it (§10).

**Exactness contract.** Curves are analytic (positions *and* tangents), meshes are
watertight by construction (D21) with a stated chord budget (D34: ≤ 0,25 mm, measured
0,05), containment queries are closed-form, seam distances are point-to-segment
against the true polyline, never nearest-sampled-neighbour. When something is
approximate it says so in a field.

**Determinism by content hash (D15).** A scene is a pure function of
`(config, seed)`. `scene_id = config_id + seed`; the content hash covers canonical JSON
plus every array's bytes. `weldgen verify` re-hashes a corpus on disk.

**Twins (D12, §6.4 of SCHEMA).** Any config key that is *not* geometry (fixture
presence, sensor profile, density, camera regime) can be flipped and the workpiece
geometry stays bit-identical; `twin_key` pairs the arms. Every ablation in the project
is a `groupby` on `twin_key`.

**Anti-shortcut (D28, D31).** A dataset can leak its answer through a correlate. Here
the risk was axis alignment: a seam parallel to the plate edges lets a learned model
"find the rectangle, take its centreline". So every plate gets a polygon outline
(butt/corner/edge) or an in-plane yaw (T/lap), curved spines are drawn at random, and
a **gate** (`scripts/qa_d28_gate.py`) measures the seam-to-free-edge angle histogram
of every corpus and fails it if 0°/90° dominate. D31 makes the four seam classes
disjoint by construction so a class label is never a taxonomy accident.

---

## 3. How a scene is made — the pipeline

`weldgen/scene.py` (plates) and `weldgen/scene_curved.py` (curved families) run the
same sequence. Each step reads its own RNG substream (`weldgen/rng.py`, 8 named
`SeedSequence` children), which is what makes twins possible: changing the camera draw
cannot disturb the geometry draw.

```
config (YAML over DEFAULT_CONFIG)  ──►  config_id (hash of the resolved config)
        │
        ▼  substreams 0–2: geometry
1. sample_joint  ──► JointSpec  (joint type, ISO 9692-1 preparation & gap, ISO 5817
                                 misalignment/angle, thickness, outlines, yaw, groove)
2. layout        ──► parts placed by closed-form transforms  (Slab / Prism /
                                 PreparedSlab / PreparedPrism; curved: Tube / SweptSlab)
3. seams         ──► enumerate face pairs, apply the D4 accessibility rule,
                                 derive class (D22), reject with a stated reason,
                                 store the seam as an exact polyline + the D19 curves
        │
        ▼  substream 5: camera        ▼ substreams 3–4, 6: sampling, noise
4. camera pose   ──► a stereo profile, standoff, elevation, azimuth, roll, aim jitter
                                 (uniform_sphere or approach_cone regime, D26)
5. cloud         ──► area-uniform surface sample of every face, with face_id and
                                 object_id per point (or camera_raster, D20)
6. visibility    ──► ray-cast occlusion + frustum → visible_from_cam; HPR-style
                                 exterior flag → exterior (the Task 1 input)
7. noise model   ──► stored as PARAMETERS (baseline, focal, subpixel, seed); the
                                 realisation is recomputed by apply_noise (D14)
8. rules         ──► mps_rule-0.1 (Task 2 label), tackrule-0.1 (tacks) — optional,
                                 flag-gated or retro-applied
9. writer        ──► scene.json, cloud.npz, seams.npz, scene.sha256, index.jsonl row
```

### 3.1 Sampling the joint (`weldgen/config.py`, `joints.py`)

Every range is tagged by provenance in `PARAMETERS.md`: `[ISO]` (ISO 5817:2023
imperfection limits — linear misalignment 5071, angular misalignment 508, root gap
617; ISO 9692-1:2013 preparations and gaps; ISO 17659 joint names), `[repo]` (the
lab's admittance-control repo), `[ds]` (dataset design), `[ours]` (a judgment call,
stated). Quality level (B/C/D/below-D) is *classified after the draw* from what the
joint actually satisfies, never aimed for. The included angle of a T is a sampled
design parameter (D18) — a 70° T is not a 90° T with a 20° defect — and ISO 17659
names the result: **T-joint (3.10)** at 90°, **angle joint (3.12)** otherwise.

### 3.2 Placing the parts (`weldgen/layouts.py`, `geom.py`)

Five layouts, one per joint type, placement only — no seam solving at generation time
(D3: the pipeline is inverted; the seam is where the layout put the faces). Primitives
carry a **face registry** (`+w`, `-w`, `s0…`, `root`, `fusion`, `lateral+` …) so every
downstream query asks the part for its faces instead of assuming six. The primitives:

| primitive | what | since |
|---|---|---|
| `Slab` | rectangular plate | Phase 1 |
| `Prism` | convex polygon outline extruded — the D28 anti-shortcut plate | 6a |
| `PreparedSlab` | rectangular plate with an ISO 9692-1 groove profile (V, bevel, U) | 6b |
| `PreparedPrism` | the outline **and** the groove; the composition 6b missed, added 2026-09-07 | post-Phase-4 |
| `Tube` | pipe with wall, base cut by a plane (miter) or a cylinder (saddle) | 6b |
| `SweptSlab` | a band swept along an exact spine (ellipse, arc, rounded rect, B-spline) | 6b |

Curved families (D29) are **curve-first**: the seam curve is drawn from a family and
the parts are *derived* from it (`weldgen/d29.py`, `constructors.py`, `curves.py`), so
the seam is exact on the curve by construction. Seven families: line (1), circle (2),
ellipse (3), saddle (4), rounded rectangle (5), swept path (6), arc butt (7).

### 3.3 Finding the seams (`weldgen/accessibility.py`)

D4: a weldable seam is the intersection of an **exterior** face pair whose dihedral
bisector escapes to free space — one rule for all five joint types, encoding torch
reachability rather than mere geometry. D13: both faces must belong to workpieces; a
part–fixture contact is rejected as `fixture_contact`. Every rejected candidate is
stored with a frozen `reject_reason` (`bisector_blocked`, `no_contact`,
`degenerate_dihedral`, …) so "not a seam" is a recorded verdict, not silence. Seam
**class** (D22) is derived from the faces: `fillet` (face×face angled), `butt`
(face×face coplanar), `lap_toe` (edge×face), `edge` (edge×edge); the joint type says
which classes are *primary* (`matches_joint_type`).

D19: with a root gap the faces do not meet, so "the seam" is a choice. The stored
curve is the **nominal** zero-gap intersection of the extended faces; `_root` and
`_gapmid` are derived and stored alongside. Grooved butts store `_grooveroot` instead
(D36). The Phase 5 annotators, unprompted, clicked gap-mid on butts and root on Ts —
the choice is real. Measured on the Phase 4 corpus (notebook 15 §3.2): the root
offset is ≈ one gap (angle-dependent: ×0,96 at acute fillets, ×1,25 at obtuse), gap-mid
≈ half, and root lands more than 3 mm from nominal on 23% of butt centrelines and 14% of
fillets — so every table states its curve, and the two alternatives ship as arrays.

### 3.4 Camera, cloud, visibility (`camera.py`, `sampling.py`, `visibility.py`)

The camera is a stereo model with **named profiles** (D16: `d435i`, `stereo_good`,
`stereo_poor` — baseline, focal, subpixel) so sensor quality is a benchmark axis. The
pose is drawn around an aim point that deliberately *misses* the seam by a random
fraction (nothing is recoverable from the pose alone). Two regimes (D26):
`uniform_sphere`, and `approach_cone` (rejection-sampled until a target seam is at
least half visible — the regime that makes Task 2 a genuine choice).

The cloud is one full-surface sample with two per-point flags: `visible_from_cam` (ray
cast against every watertight part + frustum, D6/D7) and `exterior` (what a perfect
multi-view scan could return; the interior faces of a CAD-built cloud are truth only,
never a method input). `occluded_fraction` and `visible_fraction` per seam are
difficulty axes for free.

### 3.5 What is *not* stored

The noise realisation (D14) — parameters only, `apply_noise` is released. Meshes —
`--emit-meshes` regenerates them deterministically (Phase 8 will). Any convention
(§10) — recomputable from the stored files.

---

## 4. The vocabulary, so you can name things

- **Joint types (5):** T, butt, corner, lap, edge — the ISO 17659 taxonomy, with T
  split into T-joint and angle joint by included angle.
- **Seam classes (4, disjoint by D31):** fillet, butt, lap_toe, edge.
- **Seam families (7, D29):** line, circle, ellipse, saddle, rounded_rect, swept_path,
  arc_butt. Straight seams are family 1, not a separate thing.
- **Strata:** joint type / family (+ `line_grooved` for prepared butts). The Phase 4
  corpus has 12; numbers are reported per stratum, never pooled.
- **Preparation (D35):** square, single_V, single_bevel, single_U — ISO 9692-1 rows,
  reachable preps correlate with thickness by design.
- **Conditions:** `full_exterior` (Task 1), `single` (Task 2), noise scale 0/1/2×σ,
  fixture on/off, L0/L1 oracle arm.
- **Twin:** same seed, one non-geometry key changed; joined on `twin_key`.
- **Gates:** determinism (content hash), watertightness (D21), D28 anti-shortcut
  histogram, D34 chord error.

---

## 5. What is on disk — the schema in plain words

One directory per scene, named by `scene_id`:

```
<scene_id>/
  scene.json      everything that is not an array: objects (primitive, params,
                  outline, T_world_part), the face registry, joint (type, ISO term,
                  prep, angle, quality), fit (gap, misalignments), seams[] (class,
                  face_pair, parametric form, weldable, reject_reason, visible /
                  occluded fractions, closed, underside), camera (K, pose, regime,
                  profile), cloud (density, sampling mode, chord error), noise_model,
                  mps, tacks, twin_key, config_id, provenance
  cloud.npz       xyz, normals, object_id, face_id, visible_from_cam, exterior
  seams.npz       seam_<i> polylines + _root/_gapmid (or _grooveroot) + arclength
  scene.sha256    the content hash (weldgen verify checks it)
```

Per corpus: `<joint_type>/index.jsonl` (one flat row per seed — emitted or rejected
with reason — carrying the covariates every plot groups by), `manifest.json` (how it
was built, which rule blocks were applied), and for `bench_phase4` also `facts.csv`
(the analysis join: family, prep, primitives, ISO term, profile, …).

`docs/scene.schema.json` is the machine-checkable form; every test that generates a
scene validates against it.

---

## 6. Reproducibility — what "the same" means here

- `config_id` hashes the **resolved** config, so adding a default key re-ids every
  corpus (learned the hard way — new emission flags are opt-in per config file).
- `scene_id = config_id-seed`. Two runs anywhere, any architecture, give the same
  content hash (arrays are byte-swapped to little-endian before hashing).
- `twin_key` hashes only the geometry keys + seed.
- Substream discipline: draws are **appended** to a substream, never inserted, so
  an older corpus's draws survive a newer feature (the approach-cone draws live
  after the Phase 3 camera draws in substream 5).
- `python -m weldgen verify --out <corpus>` re-hashes everything on disk.
- `tests/` — 460 tests as of this writing, run in ~10 min; every phase pinned its
  claims by test (watertightness, exact volumes, seam-on-curve, twin identity, gate
  numbers, schema conformance, determinism across processes).

---

## 7. The corpora

| corpus | scenes | what it is for |
|---|---|---|
| `out/bench` (+ `_fx`) | 250 | Phase 3/4 plate corpus, 50 per joint type, the original fixture twins |
| `out/bench6a` | | Phase 6a plate corpus with outlines + yaw (the D28 fix), used by Phase 5 annotation |
| `out/bench6b` | 300 | Phase 6b family-balanced corpus (60 per class, families balanced within class) |
| **`out/bench_phase4`** | **720** | **the Phase 4 run corpus: 60 per family**, D28 PASS 0,34, D34 PASS, mps + tacks applied |
| `out/bench_phase4_fx` | 274 | its fixture twins (plate strata; the curved families keep the fixture off) |
| `out/annotation` | 20 | the Phase 5 export + scores |

Builders: `scripts/make_bench6b.py [--per-family N]`, `scripts/make_fixture_twins.py`,
`scripts/rebuild_stratum.py` (regenerate one stratum in place, retiring the old ids),
`scripts/apply_rule_blocks.py` (mps + tacks in place, hashes rewritten).

---

## 8. The baselines and the harness (`scripts/baselines/`)

### 8.1 The methods

Each is a faithful reimplementation of a published pipeline, with its notebook
(`notebooks/04…10`) documenting every reading of the paper, every ambiguity, and the
reproduction of the paper's own number where one exists.

| name | paper | mechanism | randomised | published coarse stage (L0 oracle) |
|---|---|---|---|---|
| `lit-ransac` | Yi et al. 2026 | plane RANSAC → pairwise plane intersection | yes (30 seeds) | a ~40 mm seam band (their PointNet++) |
| `lit-regiongrow` | Wei et al. 2024 | normal-based region growing → region boundaries | no | per-surface masks + crop (their FastSAM) |
| `lit-lobb` | Zhang et al. 2025 | local-bounding-box flatness → crease points → curve fit | no | per-part masks (their K-Net); reviewed with the first author |
| `lit-ppf` | Wang et al. 2024 | point-pair-feature voting for plane pairs | no | a seam box crop (their Faster R-CNN) |
| `lit-pcaslice` | Wang et al. 2026 | slice along the PCA axis, centre per slice | no | one mask per seam instance (their YOLO + DeepLab) |
| `lit-modelreg` | Fang & Tian 2024 | non-rigid registration of the CAD model; seam *transferred* | no | the model itself — constitutively L0-with-CAD |
| `lit-quadric` | Li, Wang & Wang 2026 | normal statistics → plane or **quadric** fit per surface → seam = intersection | no (seeded walk) | per-surface labels + part membership (their two-part region growing); added 2026-09-08, the only non-planar surface model |
| `ours` | the lab's admittance repo | radius-PCA band | no | object ids — **excluded from the Phase 4 run by ruling** |

**The oracle ladder** is the fairness device: at L0 each method gets *its own paper's*
coarse stage from truth (so no method is handicapped by a stage its authors did not
release); L1 withholds it and changes nothing else. The L0→L1 delta is the price of the
unpublished stage. `lit-ppf` has an extra exact-normals rung; `lit-modelreg` has its
two published arms (dense features, global init).

### 8.2 The harness (`harness.py`, `metrics.py`, `dataset.py`)

`prepare` loads a scene once and caches every per-method oracle; `run_matrix` runs
(method × seed) on one prepared scene and returns one row per run with every metric;
`run_task2` does the MPS scoring (one-to-one matching over the weldable truth set;
rotation-invariant on closed rings, D39). Validated against a **fake oracle** (truth
plus known jitter) before any real method ran: jitter of σ must come back as RMSE ≈ σ,
a phantom must cost precision and never steal a match, a miss must be an unmatched
row.

Metrics: F1/precision/recall at 3 mm over densified polylines; symmetric Chamfer with
both directions kept; lateral error distribution (p95 is the one a toolpath cares
about); the literature's matched-path RMSE/ME with a lateral/endpoint decomposition;
EMD as the secondary metric that punishes coverage failure.

### 8.3 The batch (`scripts/run_phase4_batch.py`)

An explicit chunk list — coverage (both views), L1, noise ×2, ladder, task2, fixture —
one csv.gz per chunk, resumable per file, streaming one scene at a time (~0,5 GB).
`--append-missing` repairs a dataframe after a stratum rebuild; `--retire` archives a
retired stratum's rows. The Phase 4 run took ~64 h wall-clock on a 16-core laptop
across three parallel runners; the L1 arm dominated (RANSAC over a full cloud with no
band is 20× its oracle-assisted cost).

---

## 9. What we found (as of 2026-09-08)

Full detail with figures: `notebooks/15_phase4_results.ipynb`. The headlines:

1. **The plane-intersection collapse.** `lit-ransac` 0,97 median F1 on straight T seams,
   **0,00 on every curved family**; `lit-ppf` degrades to 0,25–0,36 on curved fillets
   and dies on butts.
2. **`lit-lobb` wins**, being the only method nonzero on every stratum but grooved, the
   best on Task 2 (0,79 selection, 0,7–1,6 mm localization), and the most noise-robust
   (0,44 → 0,31 at 2×σ while the plane methods go to zero). Its three measured
   weaknesses are all *precision* failures of one kind — real creases that are not
   seams: far-side/bore creases on closed rings, the corner inside/outside tie, and
   fixture-contact lines (+23 phantom seams). All three share the physical prior the
   detector lacks: *a seam lies between two workpieces*.
3. **Grooved butts defeat five of six methods** (≈0,00), and the finding survived a
   rebuild of that stratum with decorrelated outlines — it is the groove, not the
   rectangle.
4. **The oracle ladder is steep for the winner**: lobb 0,43 → 0,03 without its masks.
5. **The fixture price** lands on the crease detector and the slicer, not on the plane
   methods (their band oracle shields them) — the opposite of the plan's prediction.
6. **Repeatability**: a single-draw `lit-ransac` number is not a measurement; the
   deterministic five show exactly zero spread.
7. **Phase 5**: the hand-annotation floor is 1,415 mm median lateral RMSE, 2,4× the
   accuracy the literature reports against such labels; lap is catastrophic for humans
   too (8,5 mm); the author's own demo pass was worse than the briefed annotator's.

---

## 10. The rules layer — Task 2 and tacks

**`mps_rule-0.1`** (D25, `weldgen/mps.py`): the most probable seam is the argmax of
*visible* arclength over weldable seams, ties broken by larger dihedral fold then lower
id, null below 10 mm. A pure function of `scene.json`. The advisor confirmed it is a
**geometric proxy**: it does not claim to identify the load-bearing seam, which no
single view can. The **MPS margin** (runner-up over winner) says how much of a choice a
view leaves; three classes are structurally pinned (T → 0, corner → ~1, saddle → 0).

**`tackrule-0.1`** (D38, `weldgen/tacks.py`): spacing bounds are functions of the
thinner member's gauge — `d_max = min(33t, 400 mm)`, `d_min = 10t`; tacks are short
welds `clip(4t, 10, 50) mm` with an end margin `max(2t, tack_len)`; open seams collapse
to one recorded centre tack when too short; closed loops take an even n ≥ 4 with a
deterministic phase from `sha256(scene_id, seam_id)` (D39 — any tack scoring on a
closed seam must be rotation-invariant); a stored weld order (ends → centre →
bisection; opposite pairs on loops) staggers same-class seams round-robin. Provenance
stated plainly: shop-practice conventions (JASS 6 via the Kobelco handbook for
spacing; EN 1011-2 for tack length, attributed not verified; Tomków et al. 2020 as the
literature anchor). Every constant is a parameter.

Both blocks were **retro-applied** to the finished corpus (`apply_rule_blocks.py`) —
the design's whole point — so `bench_phase4` is the tack-complete corpus with its
identity intact.

---

## 11. The phases — what each delivered

| phase | delivered |
|---|---|
| 0 | schema and parameter ranges frozen (`SCHEMA.md`, `PARAMETERS.md`, `scene.schema.json`) |
| 1 | tier-1 core, T-joint, straight seam, determinism gate |
| 2 | all five joint types, the D4 accessibility rule, the fixture (D12/D13), reject reasons |
| 3 | camera, visibility (ray cast + exterior), stereo noise model with profiles, `camera_raster` |
| 4 | seven methods reimplemented + notebooks; repeat harness validated on a fake oracle; **the batch (2026-09-05)** |
| 5 | annotation study: the 1,4 mm floor, per-type curve preference, lap catastrophe |
| 6a | polygon outlines + yaw (D28 gate), class disjointness (D31) |
| 6b | curve library, seven families, tube/swept primitives, grooves (D35–37), chord budget (D34), `bench6b` |
| 6c | MPS rule, approach-cone regime, Task-2 chunk |
| 7 | tack rule (D38/D39), notebook 13 |
| — | post-run: `PreparedPrism` (grooved outlines), fixture twins, rule blocks applied, self-healing batch |
| 8 | **next** — tier-2 rendering (Isaac Sim / Replicator chosen; RGB + depth from one render pass; the D16 noise model applied to clean rendered depth so the sensor axis is shared across tiers; camera pinned to the stored pose, only materials/lighting randomised) |
| 9 | real MDF subset via the ICP pipeline; held-out geometry splits (D11); GitHub + Zenodo release |

Open on the runway: `lit-nurbs` (an eighth method), the `camera_raster` and
`approach_cone` twin arms of `bench_phase4`, the lobb B-spline extension arm, the
annotator's intra-rater repeat, Tomków 2020 / EN 1011-2 texts in hand before citing.

---

## 12. Running things

```bash
pip install -r requirements.txt
python -m pytest tests/ -q                                   # ~10 min, 460 tests

python -m weldgen generate --config configs/phase2.yaml --n 40 --out out/phase2
python -m weldgen verify   --out out/phase2                  # re-hash every scene

python scripts/make_bench6b.py --per-family 60 --out out/bench_phase4
python scripts/make_fixture_twins.py out/bench_phase4        # -> out/bench_phase4_fx
python scripts/qa_d28_gate.py out/bench_phase4               # D28 + D34 gates
python scripts/apply_rule_blocks.py out/bench_phase4 out/bench_phase4_fx

python scripts/run_phase4_batch.py --list                    # chunks + estimate
nohup python scripts/run_phase4_batch.py > batch.log 2>&1 &  # resumable per chunk
python scripts/run_phase4_batch.py --only task2              # one group
python scripts/run_phase4_batch.py --append-missing          # after a stratum rebuild
```

Notebooks (restart the kernel after editing `weldgen/`): 01 the constructed truth of
one scene, 02 what a single view returns, 03–10 one per method, 07 the repeat harness,
12/13 the older results notebooks, **14 tacks**, **15 the Phase 4 results**.
`configs/playground.yaml` drives 01/02 — flip its keys to see any configuration.

---

## 13. How to explain it in five minutes

1. *The problem:* welding robots need the seam; vision finds it; the papers claim
   sub-millimetre accuracy against hand-made labels that are themselves worse than a
   millimetre, on data nobody can download.
2. *The move:* build the scene from the seam outward, so the truth is exact, every
   condition is a dial, and one seed gives bit-identical twins across any dial.
3. *The discipline:* only exact geometry is truth; every choice is a versioned rule;
   every corpus passes gates that would catch a shortcut or an approximation.
4. *The experiment:* six published methods, each with its own unpublished stage
   supplied as an oracle, on 720 scenes across 12 strata, under two views, three noise
   levels, fixture on/off, 30 seeds where it matters.
5. *The findings:* planes die on curves; the crease detector wins and fails only by
   over-reporting creases; grooves defeat almost everyone; the annotation floor is
   1,4 mm. *The contribution to come:* a single modification to the winner that adds
   the two-body prior, measured on the same rig that found the weakness.

Show: the coverage heatmap (§2 of notebook 15), the straight-to-curved dumbbell, the
noise small multiples, the Task-2 selection heatmap, and the tack notebook's grid.

---

## 14. Glossary of decisions (the D-series, one line each)

D1 analytic truth · D2 radius-PCA is a baseline · D3 inverted pipeline · D4
accessibility rule · D5 several seams per scene · D6 one cloud + visibility mask · D7
tier boundary = sensor realism · D8 tacks are a rule · D9 no simulator in tier 1 · D10
RGB stored, not benchmarked · D11 held-out geometry splits · D12 fixture sampled,
twin-paired, pose-varied · D13 both faces on workpieces · D14 noise realisation is a
rule · D15 content-hash determinism · D16 stereo model with profiles · D17 withdrawn ·
D18 included angle is a design parameter · D19 the nominal / root / gap-mid triple ·
D20 raster mode samples hidden surface separately · D21 watertight parts, disjoint
union · D22 seam class from faces · D23 stacked joints cap angular misalignment · D24
grooves deferred to 6b · D25 MPS is a rule · D26 two camera regimes · D27 dimensions
vary across the seam, not along it · D28 only the seam edge may be axis-aligned · D29
curve-first families · D30 grooves on straight butts only · D31/D32 class
disjointness + boundary stratum · D33 curves are constructed inputs, never fits · D34
chord ≤ 0,25 mm · D35–D37 groove rows, `groove_root`, mouth-anchored nominal · D38
tackrule-0.1 · D39 closed-loop phase and rotation-invariant scoring.
