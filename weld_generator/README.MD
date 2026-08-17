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

configs/         phase1, phase2, phase2_fixture, phase3, phase3_raster,
                 smoke, reference_tjoint, playground
tests/           187 tests: geometry, determinism, schema conformance, ISO limits, viz,
                 the Phase 2 joint-type gate and the Phase 3 visibility gate
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

- **`--n` is an attempt count, not a scene count.** Overall yield is ~57%, so ask for
  roughly twice what you want.
- **Yield is not uniform across joint types** — T 72%, edge 68%, butt 62%, corner 40%, lap
  30% — so a config requesting a uniform mix does not emit one. Generate per type if the
  balance matters.

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

**Phase 3 — a real difficulty axis.** `occluded_fraction` must not cluster near zero. Over
1035 weldable seams: mean 0,59, with 40,5% at exactly 0 and 58% above 0,98. The sampler is
not polite. But the distribution is **binary, not graded** — only 0,7% of seams fall inside
(0,05 – 0,95) — and that is geometry, not a sampler defect: a straight seam under a convex
occluder is shadowed all-or-nothing, because the occluding plate spans the whole seam. The
seam was clipped to the run the two parts share in the first place. Graded occlusion needs a
non-convex occluder, a third body, or a curved seam — Phase 6 and Phase 7.

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

Then two semantic passes: `role == "workpiece"` on both faces (D13 → `fixture_contact`), and
`seam_class ∈ ALLOWED_CLASSES[joint_type]` (D22 → `wrong_class_for_joint`). The class is
*derived* from the faces involved — broad `±w` versus edge `±u/±v`, plus the dihedral — never
declared. It is what stops a scene labelled `edge` from reporting a lap toe as an edge seam.

---

## What the camera can see

`visible_from_cam` is **geometry only**: in front of the camera, inside the image, beyond
the sensor's blind zone, front-facing, and not occluded by another part. Sensor dropout is
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
