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
| D5 | Multiple seams per scene are the norm, not the exception | T = 2 fillets, lap = 2 toes, corner = 1–2, butt = 1, edge = 1 |
| D6 | Store one full-geometry cloud + per-point `visible_from_cam` mask, not two clouds | Smaller, and gives `occluded_fraction` as a difficulty axis for free |
| D7 | Tier boundary is **sensor realism**, not visibility. Tier 1 does ray-cast occlusion | Ray casting is not rendering; costs milliseconds, no GPU, no Isaac dependency |
| D8 | Tack points are a **derived layer** from a versioned rule function, epistemically separated from geometric GT | Seam = exact geometry; tacks = citable convention. Conflating them lets a reviewer attack the seam claim by proxy |
| D9 | Tier 1 core has **no simulator import**. Renderers are pluggable backends | If the generator needs Isaac Sim + an RTX card, it is as inaccessible as the "available on reasonable request" datasets being criticized |
| D10 | RGB is rendered and stored, but **not benchmarked on** | Handoff §3. Storing is cheap; not storing forecloses the 2D→3D line without full regeneration |
| D11 | Splits hold out **part geometries and joint configurations**, not random frames | Random-frame splits across near-duplicate camera poses inflate every reported number |

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

**Angular misalignment (508)**, from the fatigue-classified table: β ≤ 2° at level C, β ≤ 1° at
level B.

**Linear misalignment between tubes (5072)** — for the pipe-on-plate curved cases: h ≤ 0,5t with a
max of 4 / 3 / 2 mm for D / C / B.

Sample defects to span D through B and **label each scene with the quality level it satisfies**.
That is a stratification axis reviewers recognize, and it costs nothing.

### Geometry ranges (set these yourself, record the reasoning)

| Parameter | Range | Note |
|---|---|---|
| Plate thickness `t` | 1 – 12 mm | Covers your 1–2 mm stainless and 8 mm MDF; spans the ISO t≤3 / t>3 boundary |
| Plate length | 80 – 400 mm | 232 mm reference seam sits mid-range |
| Plate width | 50 – 250 mm | |
| Root gap `g` | 0 – 3 mm | Must include `g` > thickness/2 cases to break radius-PCA deliberately |
| Seam curvature radius | 30 mm – ∞ | Phase 6 |
| Camera standoff | 0.3 – 1.2 m | RealSense valid range; check the **minimum** too |
| Camera elevation | 15° – 85° | Below ~20° the vertical plate blocks everything — that is the point |
| Point density | 0.25 – 4 pts/mm² | Sweep it; density is a benchmark axis, not a constant |

**Deliberately include parameter combinations that break the baselines.** The `gap + spacing < R <
thickness` window closing for thin sheet is a *finding*, and you can only report it if the generator
samples into that region.

---

## 6. Phases

Effort estimates assume focused days, not calendar days.

---

### Phase 0 — Freeze schema and parameters

**Do not skip. Do not start coding first.**

- [ ] Write `SCHEMA.md` (§4) and `PARAMETERS.md` (§5)
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

Simulator-free. `trimesh` + NumPy only. No renderer, no ROS, no Isaac.

- [ ] **Seam sampler:** straight segment — length, position, orientation in world
- [ ] **Part constructor (T):** given seam curve + `(t_A, t_B, g, h, β)`, emit two box meshes
      positioned to realize exactly that seam
- [ ] **Surface sampler:** Poisson-disk or uniform, configurable density, per-point normals and
      `object_id`
- [ ] **Analytic seam label** emitted at requested density, with `weldable` flags
- [ ] **Writer** conforming to the Phase 0 schema, fully seeded
- [ ] **Seed→scene determinism test**

**Gate:** regenerate a scene from its seed alone → bit-identical file. If this fails, everything
downstream is unreproducible and the release argument collapses.

**Effort:** 3–4 days.

---

### Phase 2 — All five joint types + the accessibility rule

- [ ] Part constructors for corner, butt, lap, edge
- [ ] Implement D4 as a **verification** function: enumerate exterior face pairs, intersect, clip,
      test bisector escape — it should independently rediscover the seams you constructed
- [ ] Emit rejected interior candidates as `weldable: false` entries
- [ ] Multi-seam emission (T → 2 fillets, lap → 2 toes)

**Gate:** for every joint type, constructed seams and rediscovered seams agree to numerical
tolerance, **and** the lap/edge interior candidates are correctly rejected with
`reject_reason: "bisector_blocked"`.

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

### Phase 6 — Curved seams

The five joint categories still apply — a circular fillet around a pipe stub on a plate is a T-joint
with a closed curved seam.

- [ ] Seam sampler: arcs, C-shapes, S-shapes (splines), closed curves
- [ ] Part constructor: swept / curved plates, pipe-on-plate, cylinder-on-cylinder
- [ ] Surface-pair intersection for the verification function (plane ∩ cylinder, quadric ∩ plane)
- [ ] Re-run Phase 4 baselines → **report where the plane-intersection baseline stops working**

This phase is what defuses the trivial-label risk in `thesis_direction_handoff.md §3`. Straight-seam
tack labels are arithmetic; curved-seam tack labels are not.

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

- [ ] Scan MDF workpieces at known poses via the existing ICP pipeline → the reality-check subset
- [ ] Document the pose-uncertainty of the real subset honestly — it is *not* exact truth, and
      saying so protects the synthetic claim
- [ ] Splits by held-out part geometry and joint configuration (D11)
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
- [ ] Point-cloud file format for the release: `.npy` pair vs `.ply` vs `.parquet` — matters for
      download size and for whether people can load it without your code
- [ ] Written confirmation from the advisor on the no-physical-welding scope (handoff §6.6 — still
      outstanding)
