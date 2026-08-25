# weld_generator

Analytic weld-seam scene generator. **Ground truth is constructed, never detected** —
seam geometry is computed in closed form from the placement transforms, so there is no
annotator, no estimator, and no intra-rater-consistency hand-waving anywhere in the label
path.

Status: **Phase 3 complete** (five joint types, the D4 accessibility rule, fixture sampled,
camera + visibility + analytic sensor noise).
Plan: [`notes/dataset_plan.md`](notes/dataset_plan.md).

---

## Quick start

```bash
pip install -r requirements.txt

python -m weldgen generate --config configs/phase2.yaml --n 40 --out out/phase2
python -m weldgen verify   --out out/phase2          # re-hash every scene on disk
python -m pytest tests/ -q
```

The D12 fixture ablation is two runs over the **same seed range**, one per arm:

```bash
python -m weldgen generate --config configs/phase2.yaml         --n 40 --out out/phase2
python -m weldgen generate --config configs/phase2_fixture.yaml --n 40 --out out/phase2_fixture
```

Fixture presence is a config axis rather than a per-scene draw, so the workpiece geometry
is bit-identical across a twin. `twin_key` pairs them:
`df.groupby(["twin_key", "fixture_present"])` is the whole ablation.

Two notebooks, both of which need a **kernel restart** after you change `weldgen/` —
`sys.path.insert` will not re-import a module that is already loaded:

- `notebooks/01_inspect_scene.ipynb` — the constructed ground truth: cloud, seams, the three
  D19 curves, weldable separated from rejected.
- `notebooks/02_inspect_visibility.ipynb` — what a single view of it returns: the mask in
  world and image space, an azimuth × elevation map of where each seam can be seen from, the
  sensor model, `area_uniform` vs `camera_raster`, and the per-joint-type yield of the
  omission policy.
- `notebooks/03_baselines.ipynb` — `ours` (radius-PCA) scored against constructed truth on
  the balanced corpus: both validity-window bounds on trial (the corrected `R < t/2` paired
  against the original claim), per-joint-type error, the direction-aware clustering
  measurement, and the segmentation-dependency ablation.
- `notebooks/04_lit_ransac.ipynb` — `lit-ransac` (Yi et al. 2026) reimplemented and put on
  trial: the reproduction of its published RMSE/ME, the two gates that decide coverage, the
  L0/L1 segmentation delta, the seed box plots, and the two equations that do not work as
  printed.
- `notebooks/05_lit_regiongrow.ipynb` — `lit-regiongrow` (Wei et al. 2024): the feature it
  shares with `ours`, why PCL's defaults put a 9 mm neighbourhood on an 8 mm plate, the
  four-rung coarse-detection ladder, and the clustering wall it hits where `ours` does.
- `notebooks/06_lit_lobb.ipynb` — `lit-lobb` (Zhang et al. 2025): bounding-box flatness against
  the eigenvalue ratio, the `tanh` activation, and the closed contact perimeter that breaks
  its curve fit.
- `notebooks/07_repeats.ipynb` — the repeat harness: the fake-oracle validation, the
  advisor's seed box plots for `lit-ransac`, the measured zero spread of the deterministic
  three, the noise axis as a multiplier on the derived σ_z, and EMD as the metric that sees
  coverage failure.
- `notebooks/08_lit_ppf.ipynb` — `lit-ppf` (Wang et al. 2024): the local vote read
  correctly, the paper's own RANSAC contradiction, the estimate-vs-exact normal arms, and
  the coverage refinement — coplanar seams come back as proxies whose error is O(root gap).
- `notebooks/09_lit_pcaslice.ipynb` — `lit-pcaslice` (Wang et al. 2026): the slicing
  direction from the data, the two-seams-one-mask failure pinned, and the per-instance
  oracle named for what it is — instance segmentation of the seam-separation problem.
- `notebooks/12_seven_methods.ipynb` — the seven-method comparison, every figure a
  groupby over the batch job's csv: the coverage table, both tasks side by side, the seed
  box plots with the measured-zero-spread column, the L0→L1 price of each paper's
  unpublished stage, the noise table, the fixture ablation, and the ladder extras.
- `notebooks/11_first_results.ipynb` — the plan's nine "first real results" plots on the
  balanced corpus: gap, thickness, occlusion, joint type (both conditions), density,
  dihedral, fixture on/off on the paired twin corpus, sensor profile, and the D19
  conversion table as a function of g.
- `notebooks/10_lit_modelreg.ipynb` — `lit-modelreg` (Fang & Tian 2024): the seam
  transferred, never detected; CAD-by-construction from `scene.json`; registration error
  as the whole of the error; and the symmetric-counterpart failure outside the
  roughly-positioned envelope.

All ten are entries in the seven-method Phase 4 comparison; see `notes/dataset_plan.md` §4
and **Baselines** below.

---

## Layout

```
docs/            frozen contracts - read these before changing anything
  SCHEMA.md          scene format, naming registries, reproducibility   (v2.2.0)
  PARAMETERS.md      every range, tagged [ISO] / [repo] / [ds] / [ours] (v2.2.0)
  scene.schema.json  machine-checkable form of SCHEMA.md

weldgen/         the generator - trimesh + NumPy only, no simulator import (D9)
  rng.py             8 named SeedSequence substreams
  geom.py            slab primitive, face registry, exact plane algebra
  joints.py          JointSpec: the sampled fit-up, and tilt relative to each type's nominal
  layouts.py         placement for the five joint types - placement ONLY, no seams
  accessibility.py   the D4/D13 rule: enumerates face pairs and rediscovers the seams
  camera.py          pose sampler, pinhole intrinsics, projection (OpenCV convention)
  visibility.py      exact ray-box occlusion, per-seam visibility, HPR exteriority
  noise.py           analytic stereo depth noise - params stored, realisation not
  sampling.py        area_uniform + camera_raster sampling, exact normals + face ids
  config.py          ISO limit functions, parameter sampling, quality classification
  hashing.py         canonical JSON, content hash, config_id, twin_key
  scene.py           assembles a schema-conforming scene
  viz.py             assembly frame, cross-sections - figures, never truth
  writer.py          scene.json / cloud.npz / seams.npz / index.jsonl
  cli.py             `python -m weldgen generate | verify`

scripts/         evaluation, NOT part of the generator (Phases 4-5)
  run_phase4_batch.py   the batch job: 44 resumable chunks -> one csv.gz every
                        comparison plot reads (notebooks/12)
  annotation/           Phase 5: export scenes for CloudCompare hand-labelling
                        (no truth ships), score clicks against analytic truth,
                        emit the annotator perturbation model
  baselines/         radius_pca.py (`ours`), lit_ransac.py (Yi et al. 2026),
                     lit_regiongrow.py (Wei et al. 2024),
                     lit_lobb.py (Zhang et al. 2025), lit_ppf.py (Wang et al.
                     2024), lit_pcaslice.py (Wang et al. 2026), lit_modelreg.py
                     (Fang & Tian 2024), harness.py (the repeat harness),
                     metrics.py, dataset.py

configs/         phase1, phase2, phase2_fixture, phase3, phase3_raster,
                 bench_{T,corner,butt,lap,edge}  <- the balanced benchmark corpus,
                 smoke, reference_tjoint, playground
tests/           287 tests: geometry, determinism, schema conformance, ISO limits, viz,
                 the Phase 2 joint-type gate, the Phase 3 visibility gate, and the
                 migrated baseline
notes/           the plan, the handoff, and the standards PDFs
papers/          literature
```

---

## What Phase 3 produces

Per scene: two watertight slabs forming one of **five joint types**, every seam the D4 rule
finds, a surface sample with exact per-point normals, `object_id` and `face_id`, every seam
stored three ways (D19), **a posed camera and a per-point visibility mask**. Rejected
candidates stay in the file as `weldable: false` negatives — they are exactly what a
geometric baseline returns, which is what makes the §7 weldable-vs-interior metric
measurable.

```
out/phase2/<scene_id>/
  scene.json      conforms to docs/scene.schema.json
  cloud.npz       xyz, normals, object_id, face_id, visible_from_cam
  seams.npz       per seam: nominal, root, gapmid, arclength, tangent, approach, nA, nB,
                  visible
  scene.sha256    the content hash
```

The **noisy** cloud is not stored: `scene.json` carries the noise parameters and
`weldgen.noise.apply` is deterministic in `noise_model.seed`, which makes the sensor model a
citable convention anyone can re-run with different constants rather than a baked artefact
(SCHEMA.md §5.1). It also halves the release size.

Seam counts over 120 seeds per type, no fixture:

| | weldable seams per scene | reading |
|---|---|---|
| T | 2 in 95, 1 in 21 | one fillet per side of the standing plate |
| corner | 2 in 114, 1 in 4 | inside fillet + outside corner, approaches opposed |
| butt | 2 in 95, 1 in 25 | one centreline per exposed side; 1 when thicknesses differ |
| lap | 2 in 120 of 120 | exactly two toes |
| edge | 1 in 110, 2 in 10 | 1 when the widths differ, 2 when they match |

**Scenes with no usable ground truth are skipped, not fatal.** Two reasons: the sampled
fit-up left no face pair adjacent, so there is no seam to construct (`NoSeamsFound`, ~1% of
seeds, all quality C/D/below_D); or every seam there is happens to be hidden from the camera
(`NoVisibleSeams`). `generate` skips the seed and reports which and why.

Omitted rather than relabelled. A "no seam" class would not mean what it says — whether a
seam is visible depends on where the camera landed and, structurally, on the joint type, so a
model would learn to predict the label from the wrong evidence. Tier 1 still records the
occlusion figures on every scene it does emit; tier 2 is what uses them.

Two consequences worth planning around:

- **`--n` is an attempt count, not a scene count.** Overall yield is ~52%, so ask for
  roughly twice what you want.
- **Yield is not uniform across joint types** — T 79%, butt 57%, corner 44%, edge 42%, lap
  36% — so a config requesting a uniform mix does not emit one. Generate per type if the
  balance matters.
- **Skipped seeds are in `index.jsonl`** with `emitted: false` and a reason, so the selection
  is characterisable rather than merely reported: `df[~df.emitted]` recovers exactly what was
  dropped. A run that wants the untruncated visibility axis sets `require_visible_seam: false`
  and gets every scene.

Seeds are not backfilled past `seed0 + n`: both arms of an ablation twin must attempt the
same seeds, and they do not fail on the same ones. Visibility depends on the camera, so a
sensor-profile or sampling-mode twin loses different seeds than its partner — `twin_key`
pairs what survives in both and drops the singletons.

### The gates

**Phase 1 — determinism.** `generate(config, seed)` twice, in separate processes →
**identical content hash**. Not byte-identity: `np.savez` writes a zip and zip entries embed
a wall-clock timestamp, so byte-identity is unachievable. Hashing canonical JSON plus raw
array bytes is both achievable and the stronger property. `provenance` is excluded, so
re-running next month passes rather than "fails".

**Phase 2 — rediscovery.** `layouts.py` only places parts; it never states where the seams
are. `accessibility.py` enumerates every face pair and decides each on geometry alone. The
two are compared, which is what makes the gate meaningful rather than circular.

**Phase 3 — a real difficulty axis.** Reading it needs the two components apart, which is
what schema 2.3.0 split:

| | = 0 | strictly partial | = 1 |
|---|---|---|---|
| `occluded_fraction` — another part in the way | 40% | 1,7% | 58% |
| `1 - in_frame_fraction` — image edge, blind zone | 69% | **23%** | 8% |

Occlusion is **binary and always will be**: a straight seam under a convex occluder is
shadowed all-or-nothing, because the occluding plate spans the run the two parts share to
begin with. Framing is graded — among seams not occluded by another part, 22% have a
`visible_fraction` strictly inside (0,05 – 0,95). That is what makes an error-vs-visibility
plot a curve, and it only exists because standoff is derived from a sampled **framing
fraction** rather than drawn uniformly: drawn uniformly it framed everything, and the metric
was `{0, 1}` in 99,3% of seams.

```bash
python -m pytest tests/test_determinism.py tests/test_joint_types.py tests/test_visibility.py -q
```

Two results from it worth carrying into the paper:

- **Azimuth dominates; elevation barely matters.** Each fillet of a T-joint is visible from
  ~46% of viewpoints — its own side — and the lobes are complementary, so at least one is
  usable from 92%. The failures are a narrow band where the camera looks *along* the seam
  axis, where the standing plate's near end occludes the whole crease. True at 25° and 85°
  elevation alike.
- **The lower toe of a lap joint is visible from 0% of viewpoints** above the table —
  structurally, not statistically. Only the upper toe is usable, from 44%, which is why lap
  scenes survive the omission policy least often (34%, against T's 81%).
- **A butt joint's top centreline is visible from 100%**; its underside centreline from 21%.
  Dissimilar thickness leaves only the flush underside one, so thickness mismatch and
  visibility interact.

Consequence for consumers: **≈ 40% of scenes have no seam that is even half visible.** They
are kept — they are the single-view condition the dataset exists to characterise — but a
training split must filter on `occluded_fraction` rather than assume every scene carries
usable supervision.

---

## How a seam is found (D4, three arms)

D4 originally named one mechanism. It has three, and the third exists because enumeration
showed the rule structurally *could not* produce an edge-joint seam:

- **Intersecting pair** — non-parallel faces. The seam is the plane intersection, clipped to
  where both faces have support. Fillets and toes come from here.
- **Facing pair** — antiparallel faces separated by a gap (the mid-lap interface). No
  intersection line, and the bisector `n_A + n_B` is the zero vector, so no torch direction
  exists at all. Emitted and rejected `bisector_blocked` — a free hard negative documenting
  exactly what a nearest-point rule would wrongly return.
- **Coplanar exposed pair** — faces sharing a plane and a normal, separated by a gap *within*
  that plane. This is the edge joint and the butt centreline, and it makes
  `PARAMETERS.md` §2.7 literally true in code: a lap at `stack_offset > 0` buries its
  interface, while at `stack_offset = 0` the same interface surfaces and becomes weldable.

Then two semantic passes. `role == "workpiece"` on both faces (D13 → `fixture_contact`), and
`seam_class ∈ ALLOWED_CLASSES[joint_type]` (D22 → `matches_joint_type`). The class is
*derived* from the faces involved — broad `±w` versus edge `±u/±v`, plus the dihedral — never
declared. It is what stops a scene labelled `edge` from reporting a lap toe as an edge seam.

**`weldable` is reachability; `matches_joint_type` is taxonomy; `primary` is both.** They are
separate because every other rejection means *unreachable*, while an off-class seam is
reachable and merely off-label — a baseline that finds the lap toe on the far side of an
unequal-width edge joint has made a correct detection, and scoring it as a false positive
would mix taxonomy errors into the weldable-vs-interior metric that the lap/edge argument
rests on. `index.jsonl` carries both `n_weldable` and `n_primary`, and the gap between them
measures how often a declared joint type under-describes its own scene.

---

## What the camera can see

`visible_from_cam` is **geometry only**: in front of the camera, inside the image, beyond
the sensor's blind zone, front-facing, and not occluded by another part. Per seam it is stored
decomposed — `occluded_fraction` (another part in the way), `in_frame_fraction` (image edge and
blind zone) and `visible_fraction` (their conjunction, = `mean(seam_<i>_visible)`) — because
they are different physics and a consumer may want to filter on one and not the other. Sensor
dropout is
deliberately *not* in it — a grazing surface is visible, the stereo matcher simply fails on
it — and lives in `noise.apply`, which returns its own validity mask. Keeping them apart is
what lets `occluded_fraction` stay comparable across the three sensor profiles instead of
being confounded with them.

Ray-casting uses no ray engine. `trimesh.ray` needs `rtree`, which D9 rules out, and every
part is a box, so ray–box intersection is exact in closed form and vectorises over the whole
cloud at once — the same reasoning that produced the analytic `Slab.contains`.

Two mode-level choices worth knowing:

- **`sampling_mode: camera_raster`** (D20) samples the visible surface at the raster density
  `(f_px / z)²` and keeps the hidden surface from the same draw, flagged false. Both classes
  come from one pass, which is what makes them rate-matched. Density is then *not* uniform
  across the mask boundary, so density-sensitive metrics must be computed within a class.
- **`weldgen.visibility.hpr_exterior`** is the CAD-free exteriority test the Phase 4
  baselines need. Ground truth never uses it — it knows the transforms — but at runtime
  there is no registration, so a point-cloud-only method needs it to reject the buried
  mid-lap interface. On a lap joint it marks that interface 1,3% exterior and the same face
  outside the overlap 100% exterior.

---

## Things that are easy to get wrong

Most of these were found by sweeping a few hundred scenes, not by the tests — a suite in
which β = 0, thicknesses are equal and no fixture is present will pass through all of them.

**The bisector is where a torch *wants* to sit, not the only place it may sit.** A welder
tilts the gun off it to dodge an obstruction, up to a work angle beyond which the arc no
longer reaches both members. Pinned to the bisector, an edge joint — whose two faces are
coplanar, so whose bisector runs horizontally, tangent to the table — lost its only seam in
39 of 40 scenes the moment the fixture was switched on. The accepted axis, not the bisector,
is what `approach_dir` stores. `SCHEMA.md` §2.6.1.

**Cross-runs are told from seams by direction, not by length.** The runs across the plate at
either end of a lap are as long as the overlap, so no length threshold separates them from
the toes. The joint's direction is the length-weighted principal axis of the candidate
tangents — over workpiece pairs only, and over candidates still in the running. Every one of
those qualifiers fixes a real failure; `SCHEMA.md` §2.6.2 lists them.

**"Same plane?" depends on where you measure it.** Between the two face *centres*, a tilt is
amplified by the whole half-width of the plate: 2,8° on a 146 mm plate reads as a 4,4 mm
step, and 27% of butt joints never had a centreline enumerated at all. Measure at the seam.

**Angular misalignment hinges at the contact, not at the part centre.** Two clamped plates
rotate about where they touch. The lever arm is half the plate width, so this is much larger
than it sounds: 0,4° on a 179 mm plate lifts the welded edge by 0,62 mm against a 0,1 mm
root gap. Hinged at the centre, the seam is lost outright rather than recorded as misaligned.

**Linear misalignment is a step between surfaces, not a second root gap.** Applied along the
axis the gap already occupies, it pushes the joint faces `g + h` apart and out of
`contact_tol_mm`. And unequal thicknesses are set **flush on one face** — centring both on
their mid-thickness plane steps *both* faces by half the difference, leaving a butt joint
with no coplanar face pair at all.

**Where the camera aims is a decision, not a detail.** The assembly centroid is dragged
upward by the standing plate, so a camera aimed at it looks over the seam. Aiming exactly at
the seam is worse: it pins the seam to the image centre and lets a model read the answer off
the pose — the same leak §1.1 avoids by not pinning the assembly to the world origin. Aim at
the joint, then miss by ±0,15 of the longest part edge.

**A seam is not subject to the facing test.** A surface point cannot be seen from behind its
own face. A seam point lies on the crease, on the boundary of *both* solids, and is visible
from far more than the 90° either side of its bisector. `approach_dir` lifts the ray origin
off the two faces and nothing more; using it as a surface normal returned
`occluded_fraction: 1.0` for every seam in the first Phase 3 scenes.

**`included_angle_deg` is not `dihedral_deg`.** The first is the ISO 9692-1 *design* nominal,
one per scene. The second is a property of each *seam*. A T-joint at a 70° included angle has
two fillets, at 70° and 110° — one scene, one included angle, two dihedrals.
`tests/test_geometry.py::test_fillet_dihedrals_are_supplementary` pins this.

**`fit.angular_misalignment_deg` is not `included_angle_deg` either.** β is the *deviation*
from nominal, and each joint type has its own nominal — 90° for T and corner, 180° for butt,
0° for lap and edge. Hardcoding `α − 90` stood three of the five layouts on edge.

**Faces are parallel within a tolerance, not exactly.** The sampler tilts B by β, so an exact
parallelism test means butt and edge joints find their centreline only when β happens to be
0 — which is how a genuine bug hid behind a passing suite. `PARALLEL_TOL_DEG = 8`.

**The seam is a choice when the root gap is non-zero.** `nominal`, `root` and `gap_mid` are
three different curves separated by `O(g)` — at the reference 1,1 mm gap that is larger than
the ~0,6 mm RMSE the literature reports. We store `nominal` and ship the other two derived.
See `SCHEMA.md` §1.3.

**Scenes are posed randomly in the world, so world axes mean nothing.** Yaw and translation
are sampled per scene (SCHEMA.md §1.1) precisely so a joint pinned to the world axes cannot
leak the answer. Any cross-section or figure must first go through
`weldgen.viz.assembly_frame`. Slicing on world X instead produces markers that wander across
the plate and move with every seed.

**`w` is always the thickness axis.** For every slab, the broad faces are `±w`. Note the
consequence in `layouts.py`: mapping the standing plate's height onto +Z *and* its thickness
onto +Y at once would be a reflection, not a rotation — so `B:+w` is the face at `y = 0`.

**The root gap answers to two standards that disagree on thin sheet.** ISO 9692-1 allows
`b ≤ 2 mm` for fillets regardless of thickness; ISO 5817 clause 617 scales with the throat,
so on 2 mm sheet level B allows only 0,34 mm. Sampling from the preparation range alone
drives ~37% of scenes to `below_D`. `config.py` draws against the 617 limit for the target
level with the 9692-1 cap as a ceiling.

**`quality_level` is derived, never assumed.** Sampling picks a target and draws inside it,
but what gets stored is recomputed from the realised defects — the weakest of the strictest
levels each imperfection satisfies (PARAMETERS.md §2.5).

**`contact_tol_mm` is capped by plate thickness.** Above it, a plate's own two faces become
mutually reachable and seams wrap around to the underside — on 1,8 mm plate a 2,0 mm
tolerance is enough. This is the same bound radius-PCA lives under (`R < t`), reached from
the other direction.

---

## Baselines (Phase 4)

Seven methods, scored against constructed truth. Two exist so far. `ours` is **not** claimed
as a novel extractor — the contribution is the dataset and the comparison
(`notes/dataset_plan.md` §4).

| Module | Method | Depends on |
|---|---|---|
| `radius_pca.py` | `ours` — radius-PCA surface variation, band only | `object_id` (cross-object gate) |
| `lit_ransac.py` | `lit-ransac` — Yi et al., *Automation in Construction* 183 (2026) 106792: improved multi-plane RANSAC, plane intersections, endpoints by projection, torch pose from the dihedral | a weld-region segmentation (their PointNet++) |
| `lit_regiongrow.py` | `lit-regiongrow` — Wei et al., arXiv:2408.10710 (2024): curvature-seeded region growing, then the edge points that span two grown surfaces | per-**surface** masks (their FastSAM) |
| `lit_lobb.py` | `lit-lobb` — Zhang et al., *RCIM* 95 (2025) 102987 with the LOBB descriptor of *IEEE T-ASE* 22 (2025) 75: bounding-box flatness, tanh activation, binary K-means, polynomial fit | per-**component** masks (their K-Net) |
| `lit_ppf.py` | `lit-ppf` — Wang et al., *Sci. Rep.* 14 (2024) 21137: PPF-coplanarity planes, orthogonal-pair Hough voting, farthest-pair corners. **Deterministic as published**, and the first method here that consumes normals | a weld-box crop (their Faster R-CNN) |
| `lit_pcaslice.py` | `lit-pcaslice` — Wang et al., *Welding in the World* (2026): PCA-adaptive slicing, per-slice geometric centres, cubic B-spline path. The per-slice centre cannot survive two seams in one strip, so the coarse stage is structural | **per-instance** weld masks (their YOLO11 + DeepLab) |
| `lit_modelreg.py` | `lit-modelreg` — Fang & Tian, *RCIM* 89 (2024) 102772: a basic model (CAD-by-construction from `scene.json`) registered by CPD; the seam is **transferred, never detected**. Constitutively L0 — the ladder's top rung | the model itself, plus model-derived target features |

**All seven methods are now implemented** — `ours` plus six literature reimplementations —
each tested against known geometry, each run through the repeat harness with its own
paper's coarse stage, and each with an executed notebook.

**The input conditions are fixed, and exteriority is a condition, not a gate.** A per-point
`exterior` flag — perfect multi-view scan reachability, computed analytically at generation
time from the slabs under the sensor's grazing cone — is stored in `cloud.npz`. Task 1 runs
on `full_exterior`, Task 2 on `single_view`; the full CAD cloud including buried interior
faces is truth only, never a method input. No method carries an exteriority gate of its
own. Measured consequence on `ours`/lap: precision 0,53 → 0,68, band width 5,8 → 3,1 mm,
recall held — the mid-lap phantoms were a CAD-cloud artifact, and they leave with it.
(Single-parameter HPR measurably cannot draw this line on plate assemblies — the corridor
vs slit trade-off is documented in `visibility.exterior_scan`'s docstring — which is why
the stored flag is analytic; `hpr_exterior` remains for runtime no-CAD contexts.)

**The three literature methods have three different coarse stages, and they are not
interchangeable.** One method wants a weld-seam band, one wants per-surface masks, one wants
per-component masks — supplying the same oracle to all three flatters some and starves
others. Measured on a T joint: the seam band is 94 k points and **100%** of it lies within
20 mm of a seam; the surface-derived crop is 331 k points and **31%** does. A plate has six
faces, so every plate rim is a surface junction.

**`lit-lobb`'s coarse stage is `ours`' cross-object gate.** Its eq. 2 calls a pixel an edge
when its neighbourhood holds two different *component* masks — that is `object_id`, obtained
from a trained 2D segmenter at 97,35% mIoU instead of from a stack of registered CAD clouds.
So the dependency `ours` is criticised for is one the literature also takes and publishes,
which makes the honest framing a *ladder every method sits on* rather than a weakness of one.
On a corner joint it reproduces its paper: median **RMSE 0,51 mm** over 25 scenes against a
reported < 0,7 mm, first run, no tuning. **And it collapses exactly as `ours` does when the
masks are withheld** — F1 0,83 → 0,02, against `ours`' 0,75 → 0,03. Two methods sharing
nothing but that one input, failing by the same factor without it.

**`lit-ransac` reproduces its paper.** Checked against §7.2.1's own metric, not ours — RMSE
0,60 mm and ME 0,81 mm against a reported max of 0,64 and 1,16, on a T joint at the paper's
0,6427 mm point spacing with its 40 mm weld region supplied. Noiseless and with the §5.4
refit disabled the error is **exactly zero**, which is the real proof: three exact points
give an exact plane, two exact planes an exact line, and nothing is left for a bug to hide
in. `metrics.matched_path_errors` computes the literature's RMSE/ME form for exactly this.

**`lit-regiongrow` uses the same curvature `ours` does** — eq. 3's `λ₀/Σλ` is
`surface_variation`, pinned by a test — so the two differ only in neighbourhood (k-NN vs
ball), decision rule (region growing vs threshold), and how they tell two parts apart (its
own grown regions vs an `object_id` oracle). Each of those is measurable on one code path
rather than arguable.

**Its accuracy holds and its coverage does not.** Where it returns a seam fragment, that
fragment is on the seam — median RMSE 0,94 mm (corner) and 1,17 mm (T) against Wei et al.'s
reported 0,37–0,56, with 41% of fragments inside their stated 1 mm maximum-error requirement.
What collapses is coverage: a fragment spans **10% of its seam on a T joint**, 35% on a
corner. Their own metric cannot see this — §IV-A samples 3 points per linear seam and reports
the distance at those points, so an accurate 14 mm stub of a 142 mm seam scores well. Exact
constructed truth is what makes the distinction visible, and that is a better argument for
this generator than any accuracy number.

**Its own coarse stage is worth nothing, and a perfect one is worse than none.** Four rungs
over 50 T + 50 corner: a seam band drawn round the truth gives F1 0,38; **§III-C's own
surface-derived crop gives 0,13, which is what no crop at all gives** (0,13); and supplying
*perfect* surface labels gives **0,05**, worse than nothing, at 41 mm RMSE. Region growing was
accidentally helping by under-segmenting. A method improved by a worse segmentation is a
strong statement about its premise.

**And its oracle-free claim fails, for a nameable reason.** Its two-surface test was the
reason it is in the comparison: it asks *"different surfaces?"* using regions it grew itself,
where `ours` asks an `object_id` oracle. Given the paper's own weld-region crop it works —
F1 0,41, and 55% of surviving points within 3 mm of truth over 50 T + 50 corner scenes.
Given a whole workpiece it returns **ten times as many points, 89% of them nowhere near a
seam** (F1 0,11, median 84,7 mm from truth). The flaw is a category error rather than a
threshold: **a plate has six faces, so every edge of every plate is a junction of two
surfaces** — the test asks *"two surfaces?"* when the question that matters is *"two parts?"*.
`ours` collapses the same way for a different reason (F1 0,75 → 0,03). Neither method has a
point-cloud-only way to tell a part boundary from a face boundary, which makes that a missing
primitive rather than a flaw in either paper.

Three things a reader of the paper should know before quoting either.

**`ours`' own validity window was wrong, and correcting it is worth more than any
clustering.** §8 states `gap + spacing < R < thickness`; what bridges a plate's two faces is
the ball's **diameter**, so the bound is `R < thickness/2`. Paired on 44 scenes: F1
0,709 → 0,897, band width 5,70 → 3,31 mm, better in 38 and 44 of 44 scenes. A "valid" radius
under the old bound gave a ball spanning 1,2–1,9× the plate. `validity_window_mm(...,
upper="thickness")` keeps the original claim reachable so the correction stays falsifiable.

**Direction-aware clustering is `ours`-only, on purpose.** Neither Wei et al. nor Zhang et al.
specifies a clustering step, so this repo chose one for them. Filling an unspecified step with
the standard choice is faithful; filling it with something better than the paper suggests
would make the reimplementation beat the published method — the same fidelity failure as
making it worse — and would leave every number unattributable. A test asserts the `lit-*`
modules never import it.

**A published constant can delete a joint type — mostly.** `lit-ransac` terminates plane
fitting when a plane holds under `T_mpp = 2,5%` of the points (their eq. 17). A lap toe is made
by the top plate's *edge face*, under 2% of a whole workpiece, so the plane is never fitted. Over
2 scenes × 4 densities × 8 seeds the joint returns nothing in **89%** of whole-workpiece runs
against **45%** restricted to the paper's own 40 mm weld region; mean F1 0,07 against 0,31. The
constant is fine on the input it was tuned for. *"Mostly"* is doing real work in that sentence:
read at one seed the same comparison looked like a clean 0,00 against 0,55, and it is not.

**Two of the paper's equations are unusable as printed** — the iteration budget (eq. 18, which
reads as ~10¹⁰ samples against a reported 0,187 s runtime) and the plane refit (eq. 19, two
random points and one cross product, against a reported 0,011° residual). Both are implemented
in the reading consistent with the paper's own results, with the literal form behind a switch
and the discrepancy written down in the module docstring rather than smoothed over.

**A single number for a randomised method is noise.** On one fixed `out/phase3` butt scene,
varying only the RANSAC seed over 30 repeats: F1 min 0,00, median 0,94, max 0,96. On another,
20 of 30 runs return nothing at all. The plane *count* varies with the seed, so whether the
seam exists to be found is decided by which three points were drawn first. Report distributions.
Scenes that fail at **every** seed are a separate, structural failure — and telling the two
apart needs many scenes of known truth, which is the argument for this generator.

---

## Not yet implemented

Fields exist in the schema and are filled with honest placeholders rather than being absent,
so the array layout never changes:

| Field | Current value | Filled in |
|---|---|---|
| `tacks` | `null` | Phase 7 |
| `joint.prep` | `"square"` — groove preparations V/U/J/bevel | Phase 6 (D24) |
| curved seams, non-slab primitives | — | Phase 6 |

Parts are plain slabs through Phase 5 (~~D17~~ withdrawn). `part_geometry_id` is
`<primitive>_<dims>`, so the D11 split is a held-out **dimensions** split until Phase 6 —
worth saying in the paper rather than letting "held-out geometry" imply more than it
delivers.
