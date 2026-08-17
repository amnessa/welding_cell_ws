# Weld Seam Dataset — Implementation Plan

*Companion to `thesis_direction_handoff.md`. Written to be worked through phase by phase.*
*Version 0.1 — read §1 and §2 before touching code.*

---

## 1. The one rule everything else follows from

**Ground truth is constructed, never detected.**

The seam is an *input* to scene generation, not an output of processing it. You sample the joint
specification first, then place the parts to realize it. The label is the curve you drew.

This is the whole differentiator. It is what makes the dataset:

- **exact** — no annotator, no estimator, no intra-rater consistency hand-waving
- **re-samplable** — any point density on demand ("10 dots per mm, or 50 if someone asks")
- **reproducible** — a seed regenerates the scene, so the release can be a *program*, not a zip

**Corollary that must not be forgotten:** radius-PCA curvature extraction is now a **baseline**,
not infrastructure. Running your own detector on your own scenes and calling the output a label
would reintroduce exactly the estimation error the dataset exists to expose — and would bias the
benchmark toward methods that resemble radius-PCA.

---

## 2. Settled design decisions

Recorded so they are not re-litigated. Each was argued through; if one is reopened, note why here.

| # | Decision | Reason |
|---|---|---|
| D1 | Seam GT is analytic, from the placement transform | §1 |
| D2 | Radius-PCA is a baseline, evaluated against GT | §1 |
| D3 | Generator inverts the pipeline: sample joint → sample seam curve → place parts | Makes lap/edge trivially correct; no intersection solving needed at generation time |
| D4 | Weldable seam = intersection of an **exterior** face pair whose dihedral bisector escapes to free space | Single rule covers all 5 joint types; encodes torch reachability, not just geometry |
| D5 | Multiple seams per scene are the norm, not the exception | T = 2 fillets, lap = 2 toes, corner = 2 (inside fillet + outside edge weld), butt = 1–2 centrelines, edge = 1–2 flush edges. **Counts revised 2026-08-16** once D22 made the classes explicit; the originals were informal |
| D6 | Store one full-geometry cloud + per-point `visible_from_cam` mask, not two clouds | Smaller, and gives `occluded_fraction` as a difficulty axis for free |
| D7 | Tier boundary is **sensor realism**, not visibility. Tier 1 does ray-cast occlusion | Ray casting is not rendering; costs milliseconds, no GPU, no Isaac dependency |
| D8 | Tack points are a **derived layer** from a versioned rule function, epistemically separated from geometric GT | Seam = exact geometry; tacks = citable convention. Conflating them lets a reviewer attack the seam claim by proxy |
| D9 | Tier 1 core has **no simulator import**. Renderers are pluggable backends | If the generator needs Isaac Sim + an RTX card, it is as inaccessible as the "available on reasonable request" datasets being criticized |
| D10 | RGB is rendered and stored, but **not benchmarked on** | Handoff §3. Storing is cheap; not storing forecloses the 2D→3D line without full regeneration |
| D11 | Splits hold out **part geometries and joint configurations**, not random frames | Random-frame splits across near-duplicate camera poses inflate every reported number |
| D12 | Fixture (`role: "fixture"`, `object_id 255`) is **sampled, not always present** — ~50/50, with **paired seeds** so on/off twins are exact, and **pose-varied** (tilt ±10°, working surface not pinned to `z = 0`) | Revised 2026-08-13, see below |
| D13 | D4 requires **both faces on `role: "workpiece"` objects**; part–fixture contact is rejected as `fixture_contact` | A plate standing on the fixture has two exterior faces, a 90° dihedral and an escaping bisector — it passes D4 on pure geometry and is *not* weldable. The rejection is not derivable from geometry, which makes it the most interesting class in the weldable-vs-interior metric (§7) |
| D14 | Cloud stores the **noiseless** sample + noise params; the realization comes from a released `apply_noise()` | Same epistemic move as D8: the exact thing is truth, the corruption is a versioned convention. Frozen Zenodo release additionally materialises `xyz_noisy` for bit-comparability |
| D15 | Determinism gate is a **content hash**, not byte-identical files | `np.savez` embeds zip timestamps, so byte-identity is unachievable; hashing canonical JSON + raw array bytes is the stronger property anyway |
| D16 | The depth model is a **stereo-depth model with named sensor profiles**, not a D435i model | Parameterised by `(baseline, focal, subpixel)`, which the schema already does. `d435i` is one profile alongside `stereo_good` / `stereo_poor`. Makes sensor quality a benchmark axis and demotes the datasheet-verification item off the critical path |
| ~~D17~~ | ~~Procedural features (chamfers, holes, slots, notches, stiffeners)~~ — **WITHDRAWN 2026-08-15**. Parts are plain slabs through Phase 5; real geometric diversity arrives in **Phase 6** (curved parts, pipe-on-plate) and **Phase 9** (scanned workpieces) | Procedural features were decoration dressed as diversity — they would have let the D11 split *claim* held-out geometry while really holding out dimensions with cosmetic variation. Phase 6 and 9 are where varied geometry is the actual point. Recorded rather than erased so it is not re-proposed. **Consequence, stated honestly: through Phase 5 the D11 split is a held-out _dimensions_ split, and the docs say so** |
| D18 | `joint.included_angle_deg` is a **sampled design parameter**, independent of `fit.angular_misalignment_deg` | A 70° T-joint is not a 90° T-joint with a 20° defect. β is a deviation from nominal with ISO 5817 tolerance limits; the included angle is the nominal itself, and ISO 9692-1 Tables 3–4 give it a citable range |
| D19 | With `root_gap > 0` the seam is stored as the **nominal zero-gap intersection** of the extended faces; the root line is a derived field | At `g = 1,1 mm` the faces do not intersect, so "the seam" is a *choice* among root line / gap midline / nominal intersection. The ambiguity (~1 mm) is larger than the ~0,6 mm RMSE the literature reports. Nominal is chosen because it is continuous as `g → 0` and independent of which part is called A. **Publish the conversion between the three** — a systematic offset that may explain part of the disagreement across papers |
| D20 | Under `camera_raster`, hidden surface is sampled **separately at matched density** and flagged `visible_from_cam: false` | The mask formulation (D6) assumes the single-view cloud is a subset of the full cloud. That holds for `area_uniform` and breaks for a raster, which has no natural sample of invisible surface |
| D21 | Every object must be **watertight and winding-consistent**; the *union* is not and must not be | Ray-cast visibility, `bisector_blocked` and any `contains` query need a defined interior. The union is genuinely disjoint at `g > 0` — that is the physical truth, and D3 means no boolean union is ever computed |
| D22 | A seam's **class is derived from the faces that form it**, and the joint type determines which classes are legitimate: `edge` = edge×edge, `lap_toe` = edge×face, `butt` = face×face coplanar, `fillet` = face×face angled. Off-class seams are kept as `weldable: false` / `wrong_class_for_joint` | An edge joint means BOTH parts contribute an edge — a weld running along the *surface* of one of them is a lap toe, not an edge weld. A lap is exactly the opposite: the edge of one part against the face of the other. Without this the generator reported an edge scene's 4 seams as 4 edge welds when 2 were lap toes, and a butt joint emitted short cross-runs across the plate thickness. Settled 2026-08-16 |
| D23 | **Stacked joints (lap, edge) cap angular misalignment** at ~0.4° | The plates are clamped face to face, so relative tilt about the seam axis is physically suppressed. At the plate-joint limit a 4° tilt lifts a 100 mm plate's far edge 3.5 mm and the flush edge stops being flush — 19 of 30 edge seeds lost their seam entirely. The defect that *does* occur on a stacked joint is poor contact, which the root gap already carries |
| D24 | **Groove preparations (V, U, J, bevel) are deferred to Phase 6**, alongside curved geometry | Both need non-slab primitives, so the machinery gets built once. Consequence to state: Phases 3–5 measure **square preparation only**, and `PARAMETERS.md` §5.0's radius-PCA result keeps its square-prep scope until Phase 6 lands. Settled 2026-08-16 |

Phase 0 froze D1–D11 in [`../docs/SCHEMA.md`](../docs/SCHEMA.md) and
[`../docs/PARAMETERS.md`](../docs/PARAMETERS.md). D12–D16 were settled 2026-08-13; D17 was withdrawn 2026-08-15; D18–D21 settled 2026-08-15.

### Why D12 is *sampled* rather than *always*

An earlier draft had the fixture always present. Three reasons that was too strong:

1. **It kills the ablation.** The point of modelling the fixture is to measure *how much of
   published seam-extraction performance is an artifact of pre-isolated workpieces*. That
   needs paired on/off scenes. Uniform presence makes the dataset harder without making it
   informative.
2. **A pinned fixture is identifiable by pose alone.** If the working surface is always the
   world `z = 0` plane with only yaw varying, any learned model discovers "contacts with the
   `z = 0` plane are never weldable" almost immediately, and that rule transfers to nothing.
   `fixture_contact` stops being a hard negative. This leaks through geometry even though
   `object_id` is evaluation-only.
3. **It front-loads complexity into Phase 1**, where `contact_mode`, resting-consistency
   placement and the drop-to-plane step all arrive before the determinism gate is proven.

The fix is config, not schema: fixture **off** in Phase 1, **on and sampled** from Phase 2.
`contact_mode` needs a `"free"` member for the fixture-absent case.

### What tier 1 is for (asked and answered)

Tier 1's primary product is **measurement, not training data**. It is the instrument that produces
the paper's headline figures — the annotation-error experiment, the `R`-window sweep, baseline error
vs. occluded fraction. None of those require a trained model. If the model never happens, tier 1
still carries a dataset paper.

As training data its role is real but bounded: noiseless full-visibility clouds are an input
distribution that never occurs at runtime. Visibility flags + analytic noise close most of that gap;
what remains is specular stainless returning nothing and painted MDF behaving differently again —
which is what tier 2 and the real subset are for. The defensible recipe is **pretrain on tier 1 →
fine-tune on tier 2 → validate on real**, and the ablation across those three is itself a
contribution, because it quantifies how much sensor realism seam detection actually needs. No paper
in the literature map can answer that.

---

## 3. Baselines to implement (not contributions — the comparison table)

Two geometric baselines, because two costs almost nothing and makes the table meaningful.

### Baseline A — plane segmentation + pairwise intersection

RANSAC or region-growing → planar patches → intersect every plane pair → clip each line to where
both patches have actual support → reject pairs whose dihedral angle is implausible.

Fixes three of the current `README §8` problems structurally:
- returns a **line**, not a 16 mm band (no post-hoc centreline fit needed)
- handles **multiple seams** natively (every surviving plane pair is a candidate)
- the support-clip is most of the interior/exterior test

Precedent: Yi et al. 2026; the PPF-plane paper (`Weld_seam_object_detection_system…`).

### Baseline B — radius-PCA, fixed

Keep what exists. Two changes needed:
1. **Multi-seam:** connected components on surviving high-`V` points, then one line/spline fit per
   component. Currently assumes a single seam.
2. **Exteriority gate:** see below.

### The exteriority test without CAD

Important observation: the `README §8` blob problem is **partly an artifact of the SEPC being full
CAD clouds baked at poses** — 360° geometry including faces no camera can ever see. A real
single-view cloud contains only exterior surface by construction, so the mid-lap interior candidate
is not merely wrong, it is *absent*.

So the test is only needed on the full-geometry variant. Use **hidden point removal**
(Katz et al.; `open3d.geometry.PointCloud.hidden_point_removal`): run from ~30 viewpoints on a
sphere, union the results, mark every point visible from at least one direction as exterior. Then a
candidate seam is weldable if it has exterior support on both parts and the dihedral bisector
escapes without re-entering material.

This matters because at runtime there is no CAD registration — the test must be point-cloud-only.

### Curved seams

Plane-pair generalizes to **surface-pair intersection**: plane ∩ cylinder for pipe-on-plate,
quadric ∩ plane for a dished end. Same code shape, different primitive fitter. Wang et al.'s PCA
slicing is the third family — cite it even if not implemented.

**Where each baseline stops working is a result, not a bug.** Report it.

---

## 4. Schema (Phase 0 deliverable — freeze before coding)

One JSON per scene, plus binary arrays for the cloud. Every field below has a reason to exist;
adding a field after Phase 4 costs a full regeneration and invalidates every plotted number.

```jsonc
{
  "generator_version": "0.3.1",
  "seed": 8412337,
  "tier": 1,

  "joint": {
    "type": "T",                    // T | corner | butt | lap | edge
    "seam_shape": "line",           // line | arc | spline | closed
    "quality_level": "C"            // ISO 5817 level the sampled defects satisfy
  },

  "parts": [
    { "id": "A", "mesh": "part_A.ply", "T": [[...4x4...]],
      "dims_mm": [200, 120, 8.0], "thickness_mm": 8.0 },
    { "id": "B", "mesh": "part_B.ply", "T": [[...4x4...]],
      "dims_mm": [200, 90, 8.0],  "thickness_mm": 8.0 }
  ],

  "fit": {
    "root_gap_mm": 1.1,
    "linear_misalignment_mm": 0.4,
    "angular_misalignment_deg": 0.8
  },

  "seams": [
    {
      "id": 0,
      "weldable": true,
      "face_pair": ["A:top", "B:side_-x"],
      "curve": { "kind": "polyline", "points_mm": [[...]], "density_per_mm": 10 },
      "parametric": { "kind": "line", "p0": [...], "p1": [...] },
      "length_mm": 232.0,
      "dihedral_deg": 90.0,
      "occluded_fraction": 0.34
    },
    {
      "id": 2,
      "weldable": false,               // interior lap interface — kept as a hard negative
      "reject_reason": "bisector_blocked",
      "face_pair": ["A:top", "B:bottom"],
      "curve": { "...": "..." }
    }
  ],

  "tacks": {
    "rule_version": "tackrule-0.2",
    "params": { "d_min_mm": 40, "d_max_mm": 120, "edge_margin_mm": 16 },
    "points_mm": [[...]],
    "seam_id": [0, 0, 0]
  },

  "camera": {
    "K": [[...3x3...]],
    "T_world_cam": [[...4x4...]],
    "width": 1280, "height": 720,
    "noise_model": { "axial_sigma_mm_at_1m": 2.5, "lateral_px": 0.8,
                     "grazing_dropout_deg": 75 }
  },

  "cloud": {
    "points": "cloud_xyz.npy",         // (N,3) float32, mm, world frame
    "normals": "cloud_n.npy",
    "object_id": "cloud_oid.npy",      // uint8, 0=A 1=B
    "visible_from_cam": "cloud_vis.npy", // bool — D6
    "sample_density_per_mm2": 1.0
  },

  "rgb": "rgb.png",                    // tier 2 only; stored, not benchmarked (D10)
  "depth": "depth.png"
}
```

Notes:
- `weldable: false` seams are **kept**, not dropped. Free hard negatives for a learned model, and
  they document what the accessibility rule rejected.
- `parametric` alongside `curve` is what makes re-sampling at any density possible without rerunning
  the generator.
- `occluded_fraction` per seam (not per scene) — a scene can have one visible and one hidden seam.

---

## 5. Parameter ranges (Phase 0 deliverable)

Ranges should be **cited, not invented**. `ISO_5817_Ed_4_2023.pdf` is in the project and supplies the
defect axes directly.

### From ISO 5817:2023, Table 1 — imperfections in joint geometry

**Linear misalignment between plates (5071)**, `t` = smaller thickness, `h` = misalignment:

| `t` (mm) | Level D | Level C | Level B |
|---|---|---|---|
| 0,5 – 3 | h ≤ 0,25t + 0,2 mm | h ≤ 0,15t + 0,2 mm | h ≤ 0,1t + 0,2 mm |
| > 3 | h ≤ 0,25t, max 5 mm | h ≤ 0,15t, max 4 mm | h ≤ 0,1t, max 3 mm |

**Incorrect root gap for fillet welds (617)**, `aA` = actual throat thickness:

| `t` (mm) | Level D | Level C | Level B |
|---|---|---|---|
| 0,5 – 3 | h ≤ 0,1aA + 0,5 mm | h ≤ 0,1aA + 0,3 mm | h ≤ 0,1aA + 0,2 mm |
| > 3 | h ≤ 0,3aA + 1 mm, max 4 mm | h ≤ 0,2aA + 0,5 mm, max 3 mm | h ≤ 0,1aA + 0,5 mm, max 2 mm |

**Angular misalignment (508) — this plan previously miscited it. Corrected.**

Table 1 does **not** set a D/C/B limit on β. The β ≤ 2° / 1° / 1° values come from Annex B
(informative), Table B.1, whose columns are the **fatigue classes C 63 / B 90 / B 125**, not
quality levels. `PARAMETERS.md` §2.3 carries the corrected reading and the **[ours]** mapping
`D → 4°`, `C → 2°`, `B → 1°`. Writing "β ≤ 2° at quality level C" as if it were Table 1 is a
miscitation a welding reviewer will catch.

**Confirmed 2026-08-15.** Table 1 has **no 3.3 / 508 row** — its clause numbering runs 3.1 (507 /
5071 / 5072) → 3.2 (617) → 4.1, and the string "508" is absent from every Table 1 page (pp. 11–24).
508 appears only in Annex B Table B.1 (p. 24) and in the p. 9 symbol list defining β. Footnote `b`
("Not specified") hangs on the Table B.1 *designation*, marking that Table 1 says nothing — read
against footnote `a` ("same values as … in Table 1"), the pair distinguishes "defer to Table 1"
from "Table 1 is silent". `PARAMETERS.md` §2.3 carries the verification.

Note Annex B also gives fatigue-class 5071 limits that are **stricter** than the Table 1 values
above. Sampling is against Table 1. Always say which table.

**Linear misalignment between tubes (5072)** — for the pipe-on-plate curved cases: h ≤ 0,5t with a
max of 4 / 3 / 2 mm for D / C / B.

Sample defects to span D through B and **label each scene with the quality level it satisfies**.
That is a stratification axis reviewers recognize, and it costs nothing.

### Joint design geometry — ISO 9692-1:2013

This is **preparation** geometry, a different standard from the **imperfection** limits above.
Keep the two vocabularies separate in the prose or a welding reviewer will conflate them.

**Included angle for fillet welds (T, corner).** Table 3 (welded from one side) ref 3.1.1 and
Table 4 (both sides) ref 4.1.1 give `70° ≤ α ≤ 100°` for square preparation with `t₁, t₂ > 2`
(Table 4 requires `t > 3`). Refs 3.1.3 and 4.1.2 widen this to `60° ≤ α ≤ 120°`. Both tables cap
the fillet gap at `b ≤ 2 mm`.

So **60–120° is [ISO], not [ours]** — sample it, and record which sub-clause a given scene falls
under. Note the footnote on both tables: the ISO 2553 fillet symbol is *only applicable at
α = 90°*, which is itself worth a sentence — the standard drawing convention silently assumes the
right angle that your generator is about to stop assuming.

**Root gap for square-preparation butt joints.** This closes the standing open item. Table 1 ref
1.2.1 (`t ≤ 4`, one side) gives `b ≈ t`; Table 2 ref 2.1 (`t ≤ 8`, both sides) gives `b ≈ t/2`.
The gap **scales with thickness** rather than being a flat 0–3 mm range, which changes the sampler.

**Edge joints.** Table 1 ref 1.1, "raised edges", applies at `t ≤ 2 mm` and specifies no dimensions.
That is a citable constraint and a convenient one: edge joints are a **thin-sheet** preparation, and
1–2 mm is exactly the stainless in your lab and exactly where §5 predicts radius-PCA has no valid
radius. Restrict edge-joint scenes to `t ≤ 2 mm` and the joint type stops being an arbitrary
inclusion.

**Table 2 footnote b: "Dimensions given apply to the tacked condition."** Carry this into Phase 7.
The gaps the standard specifies are gaps *after tacking*, which means the tack rule and the fit-up
parameters are coupled, not independent — worth one line in the tack-rule documentation.

**Lap overlap has no ISO citation.** Neither 9692-1 nor 2553 gives an overlap length; ISO 2553
Table 5 no. 7.1 lists "lap" only as an edge-weld symbol with `s` = weld metal thickness. Overlap
stays **[ours]**.

### Lap and edge are the same topology at different offsets

Both have **parallel** parts — included angle 0°, not 90°. They differ only in whether the edges
coincide:

| | `included_angle_deg` | `stack_offset_mm` | Seams | Seam `dihedral_deg` |
|---|---|---|---|---|
| **lap** | 0 | `0 < offset < L` | 2 toes | 90° |
| **edge** | 0 | 0 (flush) | 1 along the free edge | ~180° (degenerate) |

Two consequences worth stating in the paper. **`included_angle_deg` and `dihedral_deg` are not the
same quantity** — a lap joint has parallel parts and a 90° seam dihedral, so the schema must carry
both. And **lap and edge fail the nearest-point rule for the same reason**: face-to-face contact
over an area rather than a line. Unifying them under one `stack_offset_mm` parameter, with edge as
the `offset = 0` degenerate case, makes that shared failure mode a *derivation* rather than two
anecdotes.

### Geometry ranges (set these yourself, record the reasoning)

| Parameter | Range | Note |
|---|---|---|
| Plate thickness `t` | 1 – 12 mm | Covers your 1–2 mm stainless and 8 mm MDF; spans the ISO t≤3 / t>3 boundary |
| Plate length | 80 – 400 mm | 232 mm reference seam sits mid-range |
| Plate width | 50 – 250 mm | |
| Root gap `g` | butt: `≈t` (1-side) / `≈t/2` (2-side) **[ISO]**; fillet: `≤ 2 mm` **[ISO]**; over-range tail to 3 mm **[ours]** | The over-range tail is what generates `below_D` and breaks radius-PCA on purpose |
| `included_angle_deg` | T, corner: 60 – 120° **[ISO]**; butt: 180°; lap, edge: 0° | ISO 9692-1 Tables 3–4 |
| `stack_offset_mm` | lap: `0 < offset < L`; edge: 0 | Lap overlap **[ours]** — no ISO citation found |
| Seam curvature radius | 30 mm – ∞ | **[ours]** Phase 6 |
| Camera standoff | per sensor profile (D16) | The min-Z bound belongs to the profile, not the schema — see §5.1 |
| Camera elevation | 15° – 85° | Below ~20° the vertical plate blocks everything — that is the point |
| Point density | 0.25 – 4 pts/mm² | Sweep it; density is a benchmark axis, not a constant |
| Fixture present | ~50%, paired seeds (D12) | The on/off pair is the ablation |
| Fixture pose | tilt ±10°, surface `z` not pinned | Otherwise the fixture is identifiable by pose alone |
| Part geometry | plain slabs through Ph. 5; curved / pipe-on-plate from Ph. 6 | ~~D17~~ withdrawn — no procedural features. Diversity comes from Phase 6 primitives and Phase 9 scans |

### 5.1 Sensor profiles (D16)

Parameterise by physics, name the profiles. The schema's `noise_model` block already does this;
only the prose was bound to one product.

| Profile | `b` | `f_px` | subpixel | min-Z | Role |
|---|---|---|---|---|---|
| `d435i` | 50 mm | 674 | 0,08 px | 280 mm | matches lab hardware; ties tier 1 to the Phase 9 real subset |
| `stereo_good` | 120 mm | 1100 | 0,05 px | ~200 mm | a better sensor than yours |
| `stereo_poor` | 35 mm | 450 | 0,15 px | ~400 mm | a worse one |

Two consequences. **Sensor quality becomes a benchmark axis**, and the `PARAMETERS.md` §5
validity-window prediction sharpens: `spacing` and `σ_z` now move together as a function of a
sensor rather than being swept independently, so "radius-PCA has no valid radius on 1–2 mm sheet"
becomes "…for any sensor in this class" — a stronger claim from the same experiment.

And the datasheet-confirmation item stops gating the release. It now gates only the `d435i`
profile's claim to match your camera.

**Deliberately include parameter combinations that break the baselines.** The `gap + spacing < R <
thickness` window closing for thin sheet is a *finding*, and you can only report it if the generator
samples into that region.

---

## 6. Phases

Effort estimates assume focused days, not calendar days.

---

### Phase 0 — Freeze schema and parameters

**Do not skip. Do not start coding first.**

- [x] Write `SCHEMA.md` (§4) and `PARAMETERS.md` (§5)
- [ ] Decide directory layout for a scene and for a release
- [ ] Decide file formats: `.npy` for arrays, `.ply` for meshes, `.json` per scene, one
      `index.parquet` or `.jsonl` over the release
- [ ] Write down the naming convention for `face_pair` strings — it must be stable across joint types

**Deliverable:** two markdown files.
**Effort:** 0.5 day.
**Failure mode if skipped:** every field added after Phase 4 forces regeneration and invalidates
every plotted number.

---

### Phase 1 — Tier 1 geometry core, straight seams, T-joint only

**Status: DONE, 2026-08-15.** Code in [`../weldgen/`](../weldgen/), tests in
[`../tests/`](../tests/), presets in [`../configs/`](../configs/).

Simulator-free. `trimesh` + NumPy only. No renderer, no ROS, no Isaac.

- [x] **Seam sampler:** straight segment — length, position, orientation in world
- [x] **Part constructor (T):** given `(t_A, t_B, g, h, β, α)`, emit two box meshes
      positioned to realize the joint; both fillets derived in closed form from the
      placement transforms (D1) — `weldgen/joints.py`
- [x] **Surface sampler:** area-uniform per face, configurable density, exact per-point
      normals, `object_id` **and** `face_id` — `weldgen/sampling.py`
- [x] **Analytic seam label** at requested density, with `weldable` flags, plus the D19
      `root` and `gap_mid` derived curves
- [x] **Writer** conforming to the Phase 0 schema, fully seeded — `weldgen/writer.py`
- [x] **Seed→scene determinism test** — `tests/test_determinism.py`
- [x] Validate every emitted `scene.json` against `scene.schema.json`
      — `tests/test_schema_conformance.py`
- [x] **Watertightness assertion (D21)**, asserted at construction

**Fixture off** in this phase (D12) — `contact_mode: "free"`, `objects[]` is the two
workpieces. Keeps the placement step trivial until the determinism gate is proven.

**Gate (D15): PASSED.** `generate(config, seed)` in two separate processes → identical
content hash, across three seeds. Also verified: the hash survives the `.npz` zip
round-trip, is insensitive to `provenance`, and *is* sensitive to a 1e-3 mm perturbation
and to a float32→float64 widening.

**67 tests pass.** Two findings worth carrying forward:

1. **The root gap answers to two standards that disagree on thin sheet.** ISO 9692-1
   allows `b ≤ 2 mm` for fillets regardless of thickness; ISO 5817 clause 617 scales with
   the throat, so on 2 mm sheet level B allows only 0,34 mm. Drawing the gap from the
   preparation range alone pushed ~37% of scenes to `below_D` and wrecked the §2.5
   stratification balance. The sampler now draws against the 617 limit for the target
   level with the 9692-1 cap as a ceiling.
2. **`quality_level` is now derived, not assumed.** Sampling picks a target and draws
   inside it, but the stored level is recomputed from the realised defects, per
   `PARAMETERS.md` §2.5. A pinned regression fixture can therefore never be mislabelled.

**Effort:** 3–4 days (estimate held).

---

### Phase 2 — All five joint types + the accessibility rule

- [x] Part constructors for corner, butt, lap, edge
- [x] Implement D4 as a **verification** function: enumerate exterior face pairs, intersect, clip,
      test bisector escape — it should independently rediscover the seams you constructed.
      **Three** arms, not two: intersecting, facing, and *coplanar exposed*. The third was added
      after enumeration showed the rule structurally could not express an edge-joint seam —
      coplanar planes never intersect in a line
- [x] Emit rejected interior candidates as `weldable: false` entries
- [x] Multi-seam emission (T → 2 fillets, lap → 2 toes)
- [x] **Fixture on, sampled** (D12): tilt ±10°, surface `z` free, and the `role == "workpiece"`
      precondition on D4 → `fixture_contact` (D13). Presence is a **config axis rather than a
      per-scene draw**: the two arms are generated over the same seed range with
      `fixture_present` true/false, which keeps the workpiece geometry bit-identical across a
      twin. `twin_key` pairs them, so `groupby(["twin_key", "fixture_present"])` is the whole
      ablation. One command per arm; a `--pair-fixture` flag would make it one command total
- [x] **Watertightness assertion (D21):** per object, `mesh.is_watertight and
      mesh.is_winding_consistent`; fail the scene rather than emit it. On plain slabs this is
      trivially satisfied and costs nothing — put it in now anyway, because it is the assertion
      that catches Phase 6's swept and revolved primitives, where degenerate caps and seam
      duplication genuinely do produce non-manifold meshes

**Parts are plain slabs in this phase** (~~D17~~ withdrawn). `objects[]` carries no feature list
and `part_geometry_id` is `<primitive>_<dims>`. The D11 split is therefore a held-out
**dimensions** split until Phase 6 — say so in the paper rather than letting "held-out geometry"
imply more than it delivers.
- [x] **`included_angle_deg` sampling (D18)** and the lap/edge `stack_offset_mm` unification
- [x] **`torch_clearance` becomes a cone**, `{half_angle_deg, standoff_mm}`, not a scalar distance.
      At 90° the bisector test is nearly free; at 60° it is not, because a real nozzle has finite
      width. Acute-angle joints then generate `bisector_blocked` rejections from physical
      reachability instead of an arbitrary threshold, which gives the §7 weldable-vs-interior metric
      a second interesting class alongside `fixture_contact`.
      The cone is **steerable**: the bisector is where a torch wants to sit, not the only place it
      may sit, so the rule tries progressively larger work angles and stores the axis it accepts
      (`SCHEMA.md` §2.6.1). Fixed to the bisector it was not conservative but wrong — an edge
      joint's bisector runs horizontally, tangent to the table, and D12's fixture then deleted the
      only seam in 39 of 40 edge scenes

**Gate — met 2026-08-17.** For every joint type, constructed seams and rediscovered seams agree to
numerical tolerance, and the lap/edge interior candidates are rejected `bisector_blocked`. Measured
over 600 scenes (120 seeds × 5 types) plus a 400-scene paired fixture sweep:

| | weldable seams per scene | reading |
|---|---|---|
| T | 2 in 95, 1 in 21 | second fillet lost only to misalignment |
| corner | 2 in 114, 1 in 4 | inside fillet + outside corner |
| butt | 2 in 95, 1 in 25 | one centreline per exposed side; 1 when thicknesses differ |
| lap | **2 in 120 of 120** | exactly two toes, every scene |
| edge | 1 in 110, 2 in 10 | 1 when the widths differ, 2 when they match |

Empty scenes: **5 of 600 (0,8%)**, all quality C/D/below_D, i.e. fit-up so far out that nothing is
adjacent. The generator skips them by seed rather than aborting the run, and reports which. Every
scene validates against `scene.schema.json`; no weldable seam carries a class outside its joint
type's `ALLOWED_CLASSES`; `twin_key` pairs 200/200 fixture twins with bit-identical workpiece
geometry, and `contact_mode == "free"` iff no fixture.

Seven defects that only a sweep could find were fixed to get there, every one of them hidden by a
test suite in which β = 0, thicknesses were equal and no fixture was present:

1. the torch cone was rigidly centred on the bisector (above);
2. cross-runs were told from seams by *length*, which cannot separate them — a deep lap's end runs
   are as long as its toes. Now by direction, against a length-weighted principal axis over
   in-class workpiece candidates only (`SCHEMA.md` §2.6.2);
3. coplanarity was measured between face *centres*, where a tilt is amplified by half the plate
   width — 2,8° on a 146 mm plate reads as a 4,4 mm step, and 27% of butt joints never had a
   centreline enumerated. Now measured at the seam;
4. angular misalignment hinged at the part centre rather than the contact, lifting the welded edge;
5. dissimilar-thickness butt joints were centred on their mid-thickness plane, stepping *both*
   faces, instead of set flush on one;
6. the corner layout applied linear misalignment along the gap axis, so `h` acted as a second root
   gap and pushed the joint out of contact;
7. `contact_tol_mm` tracked the root gap but not the misalignment, so a near-perfect gap with a
   legitimate misalignment fell out of tolerance.

This function is reused by the baselines in Phase 4 — building it here is not duplicated work.

**Effort:** 3–4 days.

---

### Phase 3 — Visibility layer

- [ ] **Camera pose sampler:** spherical shell, standoff from RealSense valid range, incidence-angle
      constraint, elevation range from §5
- [ ] **Ray-cast visibility:** `trimesh.ray` or `open3d.t.geometry.RaycastingScene` → per-point
      `visible_from_cam`
- [ ] **Analytic depth noise:** axial σ ∝ z² (already in `README §11`), lateral blur,
      grazing-incidence dropout
- [ ] **Per-seam `occluded_fraction`**
- [ ] **HPR exteriority** utility (shared with Phase 4 baseline B)
- [ ] **Pin `camera_raster` mask semantics (D20)** — raster-sample the visible surface as the camera
      would, then additionally sample the hidden surface at density matched to the mean visible
      density and flag it `visible_from_cam: false`. Cheap to implement either way now; expensive to
      change once numbers are plotted. Move it out of `SCHEMA.md` §7 from "reserved slot" to
      "pinned answer"

**Gate:** `occluded_fraction` spans roughly 0 → 0.8 across the camera sampler. If it is always near
zero, the sampler is too polite and the dataset has no difficulty axis — fix the sampler, not the
metric.

**Effort:** 2–3 days.

---

### Phase 4 — Baselines against truth

Now the PCA fix happens, **with a number in front of you** instead of RViz eyeballing.

- [ ] Baseline B: multi-seam connected components + per-component line/spline fit
- [ ] Baseline B: HPR exteriority gate
- [ ] Baseline A: plane segmentation + pairwise intersection, from scratch
- [ ] **Metrics:** Chamfer distance as primary (cheap, standard, report it everywhere);
      Sinkhorn / EMD as secondary (more principled, much slower — the advisor's "transport cost";
      the meeting transcript's "synchron distance" is almost certainly this)
- [ ] Evaluate on the **full-visibility** and **single-view** variants separately and report both

**First real results — the plots to produce:**

1. Error vs. root gap
2. Error vs. thickness → **look for the `gap + spacing < R < thickness` window closing for thin
   sheet.** If it closes entirely at 1–2 mm, that is a finding about the whole method class the
   field uses, and it lands in the same figure set as Phase 5
3. Error vs. `occluded_fraction` — nobody in the literature map can produce this plot, because they
   have no notion of hidden truth
4. Error vs. joint type — expect lap and edge to be where naive methods die
5. Error vs. point density
6. **Error vs. `included_angle_deg` (D18).** Radius-PCA's eigenvalue signature is a function of
   the dihedral, so the §5 validity window is really a *surface* over `(t, g, ρ, θ)`, not a curve.
   The plan currently predicts where it closes in three of those four
7. **Fixture on vs. off, paired seeds (D12).** This is the plot that quantifies how much of
   published seam-extraction performance is an artifact of pre-isolated workpieces. Report the
   `fixture_contact` false-positive rate separately — expect Baseline A to emit a phantom candidate
   along every part–fixture contact, since the fixture is a large clean plane and plane pairing has
   no notion of `role`
8. **Error vs. sensor profile (D16)** — `d435i` / `stereo_good` / `stereo_poor`, same seeds
9. **The D19 conversion table** — seam error under the root-line, gap-midline and nominal
   definitions, as a function of `g`. Small, cheap, and it sits in a definitional crack nobody has
   looked into

**Effort:** ~1 week.

---

### Phase 5 — Annotation-error experiment

Cheap now that the infrastructure exists. Potentially the figure that carries the paper.

- [ ] Hand-label 20–30 tier-1 scenes in CloudCompare, the way LWSNet did
- [ ] Measure annotation against analytic truth → **the label-noise floor of the existing literature**
- [ ] **Get a second annotator if at all possible.** Then report LWSNet-style intra-rater consistency
      *and* actual accuracy side by side — that makes the precision-vs-accuracy point visually
      instead of rhetorically
- [ ] Compare the floor against the ~0.6 mm RMSE these papers report

**Effort:** 1–2 days (plus recruiting the second annotator).

---

### Phase 6 — Curved seams *and* the first real part-geometry diversity

The five joint categories still apply — a circular fillet around a pipe stub on a plate is a T-joint
with a closed curved seam.

- [ ] Seam sampler: arcs, C-shapes, S-shapes (splines), closed curves
- [ ] Part constructor: swept / curved plates, pipe-on-plate, cylinder-on-cylinder
- [ ] **Groove preparations (D24)** — the other job that needs a non-slab primitive.
      ISO 9692-1 makes the choice thickness-driven and citable, so the sampler picks the
      preparation from `t` rather than inventing a distribution:

      | `t` (mm) | preparation | ISO 9692-1 ref |
      |---|---|---|
      | ≤ 2 | raised edges | 1.1 |
      | ≤ 4 | square | 1.2.1 |
      | 3 – 10 | single-V, single-bevel | 1.3, 1.9.1 |
      | 5 – 40 | single-V | 1.5, 2.2 |
      | > 10 | double-V, double-bevel | 2.4, 2.9.1 |
      | > 12 | single-U | 1.6, 2.6 |
      | > 16 | single-J | 1.11, 2.10 |

      **This is what makes a butt joint a single seam.** With a groove the weld has one
      well-defined line at the groove root, instead of the two coplanar face centrelines
      a square preparation yields. The seam is where the groove is placed, so the groove
      is cut on the sampled seam line rather than the seam being read off the geometry
      afterwards — which is D3 applied to preparation.
- [ ] Expand `joint.prep` beyond `"square"` and re-scope `PARAMETERS.md` §5.0
- [ ] Surface-pair intersection for the verification function (plane ∩ cylinder, quadric ∩ plane)
- [ ] `surface` block on non-planar faces; `bspline` parametric form pinned before starting
- [ ] Watertightness (D21) genuinely bites here — swept and revolved primitives produce degenerate
      caps and duplicated seam vertices where slabs never could
- [ ] Re-run Phase 4 baselines → **report where the plane-intersection baseline stops working**

**This phase now carries two loads, not one.** It defuses the trivial-label risk in
`thesis_direction_handoff.md §3` — straight-seam tack labels are arithmetic, curved-seam ones are
not — *and*, since ~~D17~~ was withdrawn, it is the first point at which `part_geometry_id` varies
by anything other than dimensions. The D11 "held-out geometry" split only becomes a genuine
geometry split here.

That concentration is a scheduling risk worth naming: if Phase 6 slips, the paper ships with a
held-out-dimensions split and straight seams only. Both are defensible if stated plainly; neither
is defensible if described as something more.

**Effort:** ~1 week.

---

### Phase 7 — Tack layer

Only meaningful after Phase 6.

- [ ] `tacks = place(seam, t, g, rule_params)` — versioned, released as code
- [ ] Hard constraints: both endpoints mandatory; `d_min ≤ Δs ≤ d_max`; no tack within ~2t of a free
      edge
- [ ] Constants cited from Tomków, Sobota & Krajewski 2020 and the JRM literature — **no FEM**
- [ ] Ship rule output *and* rule source; document explicitly that these are a convention, not
      geometric ground truth (D8)
- [ ] Sanity check: on a straight seam the output should be visibly trivial. Say so in the paper
      before a reviewer does.

**Effort:** 3–4 days.

---

### Phase 8 — Tier 2 rendering

Pluggable backend behind the **same schema**. This is the estimate that will slip — keep tier 1
self-sufficient so a tier-2 delay never blocks a submission.

- [ ] Pick backend. **BlenderProc is the recommended default** — less painful than Isaac Sim and no
      proprietary dependency, which matters for the release (D9). Isaac Sim as a second backend only
      if the lab's existing expertise (Umut, Ege) makes it cheap
- [ ] Materials: painted MDF, stainless, mill scale / rust
- [ ] Lighting variation
- [ ] Structured-light failure modes: specular dropout, shadow-induced depth holes, realistic
      invalid-pixel patterns
- [ ] RGB output (stored, not benchmarked — D10)

**Gate:** a tier-2 scene and its tier-1 twin (same seed, same geometry) differ **only** in sensor
realism. That pairing is what makes the tier-1 → tier-2 → real ablation clean, and it is the reason
the schema is shared.

**Effort:** 1–2 weeks, high variance.

---

### Phase 9 — Real subset + release

- [ ] Scan MDF workpieces at known poses via the existing ICP pipeline → the reality-check subset.
      **This is the other half of the geometry diversity story** (~~D17~~ withdrawn): scanned parts
      carry saw kerf, edge break, warp and paint texture that no procedural feature vocabulary would
      have reproduced honestly
- [ ] Document the pose-uncertainty of the real subset honestly — it is *not* exact truth, and
      saying so protects the synthetic claim
- [ ] Splits by held-out part geometry and joint configuration (D11). **State the scope**: a genuine
      held-out *geometry* split exists only over Phase 6 primitives and the real subset; over
      Phases 1–5 it is a held-out *dimensions* split
- [ ] Generator on GitHub with all seeds; `make dataset` reproduces the release
- [ ] Frozen release on Zenodo with a DOI
- [ ] README with the schema, the parameter ranges, and the baseline numbers

**Effort:** 1 week + scanning time.

---

## 7. Metrics summary

| Metric | Role | Note |
|---|---|---|
| Chamfer distance | Primary | Cheap, standard, report everywhere |
| Sinkhorn / EMD | Secondary | More principled, much slower. Advisor's "transport cost" |
| RMSE / ME on matched points | Comparability | Only for comparing against Yi et al., LWSNet numbers |
| Precision / recall on seam presence | Multi-seam | Did the method find *all* the seams, and no phantoms? |
| Weldable-vs-interior confusion | The lap/edge story | The metric that makes D4 visible |

The last two do not exist in the literature because single-seam is assumed. Introducing them is a
small, defensible contribution on its own.

---

## 8. Named risks

| Risk | Mitigation |
|---|---|
| Phase 8 slips | Tier 1 must be submission-sufficient on its own |
| Schema churn after Phase 4 | Phase 0 exists precisely for this. Resist adding fields |
| Trivial tack labels | Phase 6 before Phase 7; scope paper 1 as seam-only if Phase 6 slips |
| "Synthetic data isn't real" reviewer | Phase 9 real subset + the tier-1/2/real ablation |
| Reviewer reads the dataset as a seam-extraction-accuracy claim | Never frame it that way (handoff §2.5). The claim is about *labels*, not about beating anyone's extractor |
| Generator too heavy for others to run | D9. Test the install on a clean machine with no GPU before release |

---

## 9. Open decisions

- [ ] Second annotator for Phase 5 — who?
- [ ] BlenderProc vs Isaac Sim for Phase 8 — decide before Phase 8, not during
- [ ] Whether tacks ship in paper 1 or are held for paper 2 (depends on whether Phase 6 lands)
- [x] ~~Point-cloud file format for the release~~ → **resolved Phase 0**: `cloud.npz` per scene
      (one file, numpy-native, no dependency), plus a `--emit-meshes` PLY exporter that Phase 5
      needs for CloudCompare anyway
- [ ] Written confirmation from the advisor on the no-physical-welding scope (handoff §6.6 — still
      outstanding)

Opened by Phase 0, all deferred (details in `PARAMETERS.md` §7):

- [x] ~~Butt/edge root gap has no ISO citation~~ → **resolved**: ISO 9692-1:2013 Table 1 ref 1.2.1
      (`b ≈ t`, one side) and Table 2 ref 2.1 (`b ≈ t/2`, both sides) for square preparation; edge
      joints are ref 1.1 "raised edges" at `t ≤ 2 mm`. Gap **scales with thickness** — update the
      sampler, it is not a flat range
- [ ] Confirm the `d435i` profile's baseline / subpixel / min-Z against the real camera. Under D16
      this no longer gates the release — only the claim that `d435i` matches your hardware. One
      afternoon with a flat target at three ranges
- [ ] **Resolve the 10× sim-noise discrepancy**: the twin's `realsense_sim_camera_node.py` defaults
      give σ(1 m) ≈ 25 mm; the derived model gives ≈ 2,4 mm. Deliberate pessimism for ICP threshold
      tuning, or a stale default? Worth knowing which
- [x] ~~Confirm whether ISO 5817 Table 1 has a 3.3 / 508 row~~ → **resolved 2026-08-15**: it does
      not. Numbering skips 3.3 (3.1 → 3.2 → 4.1) and "508" is absent from pp. 11–24. Footnote `b`
      attaches to the Annex B Table B.1 designation. Verified in `PARAMETERS.md` §2.3
- [ ] Measure the real fixture plate. Under D12 the exact dims matter less (pose and presence both
      vary), but the `lab_fixture` preset should still be real

Opened by this revision:

- [x] ~~Pick the feature vocabulary for D17~~ → **withdrawn 2026-08-15**, D17 dropped entirely.
      Parts are plain slabs through Phase 5; diversity comes from Phase 6 primitives and Phase 9
      scans. No feature vocabulary to freeze
- [x] ~~Ship `stereo_good` / `stereo_poor`, or use them only for the plot?~~ → **decided
      2026-08-15: shipped in the release.** All three profiles are first-class. The sensor-quality
      axis is only reproducible by others if the scenes behind it are downloadable, and D16's point
      is that sensor quality *is* a benchmark axis rather than a private ablation. Cost is bounded:
      profiles differ only in substreams 5–6, so the three arms share geometry and are joined by
      `twin_key`
- [ ] Lap overlap length has no ISO citation and stays **[ours]**. AWS D1.1 or a fabrication text
      may give a minimum (commonly quoted as some multiple of `t`) — worth one lookup before
      submission, not before Phase 2
- [ ] Pin the three D19 seam definitions precisely in `SCHEMA.md` §1 and name the stored one