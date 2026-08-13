# Weld Seam Dataset — Parameter Ranges

**params_version: 1.1.0** — frozen 2026-08-13, revised for D12/D16/D17.
Phase 0 deliverable of [`../notes/dataset_plan.md`](../notes/dataset_plan.md) §5.
Companion: [`SCHEMA.md`](SCHEMA.md).

---

## 1. Rule

**Ranges are cited, not invented.** Where a number has no source, it is marked
**[ours]** and carries the reasoning inline. Every **[ours]** value is a place a reviewer
can push, so each one is either arbitrary-and-harmless (plate width) or argued.

Provenance tags used below:

| Tag | Meaning |
|---|---|
| **[ISO]** | read out of `../notes/ISO 5817_Ed_4_2023.pdf`, page cited, verified against the PDF |
| **[repo]** | measured in this workspace, source file cited |
| **[ds]** | vendor datasheet value, **not yet verified against hardware** — see §4.2 |
| **[ours]** | our convention, with reasoning |

---

## 2. Defect axes — ISO 5817:2023

### 2.1 Linear misalignment between plates — no. 5071

Table 1, p. 16. `t` = the **smaller** of the two thicknesses. `h` = misalignment along the
`w` (thickness) axis. **[ISO]**

| `t` (mm) | Level D | Level C | Level B |
|---|---|---|---|
| 0,5 – 3 | `h ≤ 0,25t + 0,2 mm` | `h ≤ 0,15t + 0,2 mm` | `h ≤ 0,1t + 0,2 mm` |
| > 3 | `h ≤ 0,25t`, max 5 mm | `h ≤ 0,15t`, max 4 mm | `h ≤ 0,1t`, max 3 mm |

Applies to longitudinal welds in plates and hollow sections.

### 2.2 Linear misalignment between tubes — no. 5072

Table 1, p. 17. For the Phase 6 pipe-on-plate cases. **[ISO]**

| `t` (mm) | Level D | Level C | Level B |
|---|---|---|---|
| ≥ 0,5 | `h ≤ 0,5t`, max 4 mm | `h ≤ 0,5t`, max 3 mm | `h ≤ 0,5t`, max 2 mm |

### 2.3 Angular misalignment — no. 508 — **read this before citing it**

**Angular misalignment is not limited by Table 1 at all.** **[ISO]** Verified two ways, since
this rests on an *absence* and an absence needs to be checkable:

1. Table 1's clause numbering runs `3.1` (507 / 5071 / 5072, p. 16–17) → `3.2` (617, p. 17)
   → `4.1` (p. 17). **There is no `3.3` row.** The `3.3 508` row exists only in Annex B.
2. The string "508" does not occur anywhere in Table 1 (pp. 11–24). It appears only on
   p. 24 (Table B.1) and p. 9 (the symbol list defining β).

The Annex B entry carries footnote `b` — *"Not specified"* — attached to the
**"Angular misalignment" designation** in Table B.1 row 3.3. Read against footnote `a`
(*"Same values as given for quality levels B and C, respectively, in Table 1"*), the pair
of footnotes distinguishes "defer to Table 1" from "Table 1 says nothing". So `b` means
ISO 5817 sets no D/C/B limit on β for statically loaded welds.

The β values everyone quotes come from **Annex B (informative), Table B.1, p. 24 —
"Additional criteria to Table 1 for welds subject to fatigue load"**, whose columns are
**fatigue classes C 63 / B 90 / B 125**, *not* quality levels D/C/B: **[ISO]**

| `t` (mm) | C 63 | B 90 | B 125 |
|---|---|---|---|
| ≥ 0,5 | `β ≤ 2°` | `β ≤ 1°` | `β ≤ 1°` |

**[ours]** We adopt the mapping `D → β ≤ 4°`, `C → β ≤ 2°`, `B → β ≤ 1°`, i.e. the two
fatigue-class values for C and B, and a doubling for D since nothing constrains it. This
is our convention and the paper must say so — writing "β ≤ 2° at quality level C" as if it
were Table 1 is a miscitation a welding reviewer will catch.

Note the same table also gives *fatigue*-class 5071 limits (`h ≤ 0,1t` max 3 mm at C 63,
`h ≤ 0,05t` max 1,5 mm at B 90), which are **stricter** than the Table 1 values in §2.1.
We sample against Table 1. Say which table, always.

### 2.4 Incorrect root gap for fillet welds — no. 617

Table 1, p. 17. `aA` = actual throat thickness. **[ISO]**

| `t` (mm) | Level D | Level C | Level B |
|---|---|---|---|
| 0,5 – 3 | `h ≤ 0,1aA + 0,5 mm` | `h ≤ 0,1aA + 0,3 mm` | `h ≤ 0,1aA + 0,2 mm` |
| > 3 | `h ≤ 0,3aA + 1 mm`, max 4 mm | `h ≤ 0,2aA + 0,5 mm`, max 3 mm | `h ≤ 0,1aA + 0,5 mm`, max 2 mm |

**The throat `aA` is an input we must choose.** ISO 5817 does not set it — it is a design
parameter. **[ours]** `aA = 0,7 · min(t_A, t_B)`, the mitre-fillet relation `a = z/√2` with
leg length `z` taken equal to the thinner plate. Recorded per scene in
`fit.throat_thickness_mm` so anyone can recompute the limits under a different assumption.

**Scope caveat.** Clause 617 is *"incorrect root gap for **fillet** welds"*. It governs
`T`, `corner`, and `lap`. It does **not** apply to `butt` or `edge` joints, where the root
gap is a joint-**preparation** dimension (ISO 9692-1), not an imperfection. For those two
joint types the gap range in §3 is **[ours]** and is *not* ISO-limited — see §7.

### 2.5 Quality level assignment

Per scene: compute, for each applicable imperfection, the strictest level its sampled
value satisfies; the scene's `quality_level` is the **weakest** of those (a scene is only
as good as its worst defect).

```
quality_level ∈ {"B", "C", "D", "below_D"}
```

`below_D` is a **valid, deliberately generated** class, not an error — out-of-tolerance
fit-up is what a real robot meets on a real shop floor, and it widens the gap/misalignment
sweep past the point where the baselines are expected to fail. Target mix **[ours]**:
`B 25% / C 25% / D 25% / below_D 25%`, so the stratification axis has balanced support.

`quality_level` is stored; the per-imperfection breakdown is **not**, because it is exactly
recomputable from `fit` + thicknesses + these tables.

---

## 3. Geometry ranges

| Parameter | Range | Provenance / reasoning |
|---|---|---|
| Joint type | `T, corner, butt, lap, edge` | uniform in Phase 2+; Phase 1 is `T` only |
| Seam shape | `line` (Ph. 1–2), `+ arc, spline, closed` (Ph. 6) | |
| Plate thickness `t_A`, `t_B` | 1 – 12 mm | **[ours]** covers the 1–2 mm stainless and 8 mm MDF actually in the lab, and straddles the ISO `t≤3` / `t>3` boundary so both limit rows get used |
| Dissimilar thickness | `t_A ≠ t_B` in 30% of scenes | **[ours]** ISO defines `t` as the *smaller* thickness, which is only exercised if they differ. Free coverage of a real case |
| Plate length `L` | 80 – 400 mm | **[ours]** the 232 mm reference seam **[repo]** sits mid-range |
| Plate width `W` | 50 – 250 mm | **[ours]** |
| Root gap `g` | 0 – 3 mm | **[ours]** deliberately exceeds the ISO limits at the top end (see §2.5 `below_D`) and, more importantly, exceeds the radius-PCA validity window for thin sheet — §5 |
| Linear misalignment `h` | per §2.1, by target level | **[ISO]** |
| Angular misalignment `β` | per §2.3, by target level | **[ours]** mapping over **[ISO]** values |
| Seam curvature radius | 30 mm – ∞ | **[ours]** Phase 6 |
| `contact_mode` | `free` when no fixture; else `flat 0.6 / on_edge 0.3 / propped 0.1` | **[ours]** how the assembly rests on the fixture; `flat` dominates because it is what magnets on a plate actually give |
| Fixture present | ~50%, **paired** (D12) | see §3.1 |
| Part features | none in Phase 1, sampled from Phase 2 (D17) | see §3.2 |
| Sensor profile | `d435i / stereo_good / stereo_poor` (D16) | see §4 |

### 3.1 Fixture — sampled and pose-varied (D12, revised 2026-08-13)

Models the steel plate the magnets hold against.

| Parameter | Value | Provenance |
|---|---|---|
| Primitive | `slab`, `role: "fixture"`, `object_id: 255` | `SCHEMA.md` §2.1 |
| **Presence** | **off in Phase 1; ~50% from Phase 2, emitted in exact pairs** | **[ours]** D12 |
| Dimensions | 600 × 400 × 10 mm nominal, ±20% on the two in-plane dims | **[ours]** — `lab_fixture` preset should use the measured plate |
| **Tilt** | **±10° from horizontal**, uniform in tilt direction | **[ours]** D12 |
| **Height** | working surface `z` **not pinned**; offset ±50 mm | **[ours]** D12 |
| In-plane yaw | uniform 0 – 360° | **[ours]** |
| Surface | planar; no texture in tier 1 | |

**Pairing is a generation strategy, not a sampling rate.** Drawing presence independently
at p = 0.5 gives random on/off with no guaranteed partner. Instead each geometry seed emits
**both** arms — one fixture-on, one fixture-off — joined by `twin_key` (`SCHEMA.md` §6.3).
The marginal is 50% and every scene has an exact twin.

**Why tilt and free height matter more than they look.** With the working surface pinned to
`z = 0` and only yaw varying, the fixture is identifiable from pose alone: any model learns
"contacts with the `z = 0` plane are never weldable" almost immediately, and that rule
transfers to nothing. `fixture_contact` stops being a hard negative and becomes a giveaway.
The leak runs through *geometry*, so it is not closed by `object_id` being evaluation-only.

**Consequence, by design:** fixture-present scenes contain at least one part–fixture contact
line that is geometrically indistinguishable from a fillet, labelled
`weldable: false, reject_reason: "fixture_contact"` (`SCHEMA.md` §2.5). Hardest negatives in
the dataset, free.

Second consequence, worth anticipating rather than discovering in Phase 4: the fixture is a
large clean planar patch, so **Baseline A (plane segmentation + pairwise intersection) gains
extra plane pairs** and will emit a phantom candidate along every part–fixture contact —
plane pairing has no notion of `role`. That is a result about the baseline, not a bug.

### 3.2 Procedural part features (D17)

Empty in Phase 1, sampled from Phase 2. Vocabulary is **provisional** — freeze before
Phase 2, because it feeds `part_geometry_id` and therefore the D11 split key
(`SCHEMA.md` §2.2.1, §5.4).

| Feature | Proposed range | Note |
|---|---|---|
| `chamfer` | 1 – 5 mm, on free edges | subtractive; `w`-invariant holds |
| `edge_fillet` | r = 1 – 6 mm, on free edges | subtractive; `w`-invariant holds |
| `through_hole` | d = 4 – 20 mm, 0 – 4 per part, ≥ 2d from any edge | subtractive; adds a `lateral` bore face |
| `slot` | 6 – 20 mm wide, 20 – 80 mm long | subtractive |
| `notch` | corner cut-outs, 10 – 40 mm | subtractive; creates extra concave edges — expect these to fire in radius-PCA and be rejected by D4 |
| `stiffener` | 3 – 10 mm thick, 20 – 60 mm tall | **additive — the awkward one, see `SCHEMA.md` §2.2.1** |

Features per part: 0 with p = 0.4, then 1 – 3 **[ours]**. Rate kept moderate so the
featureless case stays well-represented as its own control.

**Why they belong in Phase 2 and not later:** they change `part_geometry_id`, which is the
D11 split key. Adding them after Phase 4 is precisely the schema churn named as a risk.

---

## 4. Sensor — a stereo model with named profiles (D16)

The model is **parameterised by physics and named by profile**, not bound to one product.
`noise_model` already carried `(baseline, focal, subpixel)`; D16 makes the naming explicit
and adds `min_z_mm`, because the blind zone is a sensor property, not a schema constant.

### 4.0 Profiles

| Profile | `b` | `f_px` | subpixel | min-Z | Role |
|---|---|---|---|---|---|
| `d435i` | 50 mm | 674 | 0,08 px | 280 mm | **[ds]** matches the lab hardware (`realsense_camera_node.py` **[repo]**); ties tier 1 to the Phase 9 real subset |
| `stereo_good` | 120 mm | 1100 | 0,05 px | 200 mm | **[ours]** a better sensor than yours |
| `stereo_poor` | 35 mm | 450 | 0,15 px | 400 mm | **[ours]** a worse one |

Two consequences. **Sensor quality becomes a benchmark axis** (Phase 4 plot 7), generated
in exact twins via `twin_key` (`SCHEMA.md` §6.3). And the datasheet-confirmation item stops
gating the release — it now gates only `d435i`'s claim to match the lab camera.

### 4.1 Pose sampling

| Parameter | Range | Reasoning |
|---|---|---|
| Standoff | `max(min_z, 300)` – 1200 mm | lower bound **belongs to the profile** — 280/200/400 mm. Below it the sensor returns nothing. 1200 mm is roughly where `d435i` σ_z passes 3,4 mm and the seam stops being resolvable |
| Elevation above the fixture plane | 15° – 85° | **[ours]** `dataset_plan.md` §5 — below ~20° the vertical plate of a T-joint blocks the seam entirely, **which is the point**: that is the difficulty axis |
| Azimuth | uniform 0 – 360° | |
| Roll about the view axis | ±15° | **[ours]** eye-in-hand mounting is not gravity-aligned |
| Intrinsics | fixed `K` per §4.2, no jitter in tier 1 | |

**Phase 3 gate** (`dataset_plan.md`): `occluded_fraction` must span ≈ 0 → 0.8 across this
sampler. If it clusters near zero, the sampler is too polite — fix the sampler, not the
metric.

### 4.2 Depth noise model

Derived rather than tuned. Stereo depth is `z = f·b/d`, so error propagates as

```
σ_z(z) = subpixel_px · z² / (f_px · b)
```

which is the `σ ∝ z²` shape the repo already uses (admittance README §11) **[repo]**, but
with coefficients that come from the sensor's geometry instead of a guess.

The `d435i` figures follow from baseline `b = 50 mm`, depth HFOV 87° and depth resolution
1280×720 → `f_px = (1280/2)/tan(87°/2) ≈ 674 px`, subpixel RMS ≈ 0,08 px **[ds]**.

`σ_z` in mm, by profile (`n/a` = inside the blind zone):

| `z` | `d435i` | `stereo_good` | `stereo_poor` |
|---|---|---|---|
| 300 mm | 0,21 | 0,03 | n/a |
| 500 mm | 0,59 | 0,09 | 2,38 |
| 1000 mm | 2,37 | 0,38 | 9,52 |
| 1200 mm | 3,42 | 0,55 | 13,71 |
| 2000 mm | 9,50 | 1,52 | 38,10 |

The `d435i` 2 m figure (0,48% of range) sits comfortably inside Intel's published
"< 2% at 2 m" bound, as it should — that bound is a worst-case spec, not a typical value.

Also in the model: lateral blur `σ_lat = 0,8 px` **[ours]**, and dropout at incidence
angles beyond `75°` from the surface normal **[ours]**, which is what actually removes the
seam floor on a steeply-viewed fillet.

> **Discrepancy to resolve — do not silently inherit it.** The digital twin's
> `realsense_sim_camera_node.py` defaults are `depth_noise_std_m = 0.02`,
> `depth_noise_z2_coeff = 0.005`, i.e. **σ(1 m) ≈ 25 mm — about 10× the derived 2,4 mm**
> **[repo]**. That may be deliberate pessimism for ICP threshold tuning, or it may be a
> stale value. Either way the dataset uses the derived model, and the twin's numbers are
> not evidence for it. Under D16 this confirmation gates only the `d435i` profile's claim to
> match your camera, not the release; a flat target at three ranges is enough.
>
> Worth noting: 25 mm at 1 m is close to the `stereo_poor` profile's 9,5 mm — so if the twin's
> default was chosen to make ICP thresholds robust, `stereo_poor` now serves that purpose
> honestly, with a stated sensor behind it.

### 4.3 Point density

| Parameter | Range | Reasoning |
|---|---|---|
| `density_per_mm2` | 0,25 – 4 pts/mm² | **[ours]** density is a benchmark axis, not a constant. Mean spacing `≈ 1/√ρ` = 2,0 / 1,0 / 0,5 mm at ρ = 0,25 / 1 / 4 |

Reference point: the repo's SEPC has ~2,7 mm spacing **[repo]** ≈ 0,14 pts/mm², i.e.
*sparser than the bottom of this range*. Worth reporting.

---

## 5. The radius-PCA validity window — a falsifiable prediction

The repo establishes empirically **[repo]** (admittance README §8) that radius-PCA needs

```
gap + point_spacing  <  R  <  part_thickness
```

with the reference T-joint measured at gap 1,1 mm, spacing 2,7 mm, thickness 8,4 mm →
usable window ≈ 4–8 mm, default `weld_radius_m = 6 mm`.

Substituting `spacing ≈ 1/√ρ` gives a closed-form condition for the window existing at all:

```
g + 1/√ρ  <  t
```

Which turns "look for the window closing on thin sheet" into a **prediction the generator
can falsify**. Maximum root gap `g` admitting any valid `R`, under `area_uniform` sampling
where density is a free parameter:

| `t` (mm) | ρ = 0,25 (s = 2,0) | ρ = 1 (s = 1,0) | ρ = 4 (s = 0,5) |
|---|---|---|---|
| 1 | **closed** | **closed** | g < 0,5 |
| 2 | **closed** | g < 1,0 | g < 1,5 |
| 3 | g < 1,0 | g < 2,0 | g < 2,5 |
| 6 | g < 4,0 (all) | all | all |
| 8,4 | all | all | all |
| 12 | all | all | all |

So the prediction is sharp: **on the 1–2 mm stainless actually in the lab, at any density
the RealSense can deliver, radius-PCA has no valid radius at all.** If Phase 4 measures a
failure boundary that tracks this line, that is a finding about the whole curvature-feature
method class, stated as a formula rather than an anecdote — and it lands in the same figure
set as the Phase 5 annotation-error experiment.

Sampling must therefore deliberately cover the closed region. It is not a region to avoid.

### 5.1 The sharper form under D16 — and what it costs

Under `camera_raster` sampling (`SCHEMA.md` §5.1) density is **not** free: it follows from
the sensor. On a surface at distance `z` tilted `θ` from fronto-parallel, the sampling pitch
is `z / (f_px · cos θ)`, best case `z / f_px`. Substituting:

```
g + z/(f_px · cos θ)  <  t        ⟹        z_max = f_px · (t − g) · cos θ
```

**The method's viability is then decided by whether `z_max` clears the sensor's own
min-Z.** At the reference gap `g = 1,1 mm` **[repo]**, fronto-parallel best case:

| `t` (mm) | `d435i` (min-Z 280) | `stereo_good` (200) | `stereo_poor` (400) |
|---|---|---|---|
| 1 | **closed** | **closed** | **closed** |
| 2 | 607 mm | 990 mm | **405 mm — a 5 mm-wide band, effectively closed** |
| 3 | 1281 mm | 2090 mm | 855 mm |
| 6 | 3303 mm | 5390 mm | 2205 mm |
| 8,4 | 4920 mm | 8030 mm | 3285 mm |
| 12 | 7347 mm | 11990 mm | 4905 mm |

This is the stronger claim D16 buys: not "radius-PCA fails at some density we chose", but
**"radius-PCA has no valid radius on 1 mm sheet for any sensor in this class, and on 2 mm
sheet only within a standoff band that a worse-than-D435i sensor cannot reach at all."**
`spacing` and `σ_z` now move together as functions of one sensor rather than being swept
independently, so the failure is a property of the sensor class, not of a sampling choice.

**The cost, stated plainly:** this form of the claim requires `camera_raster`, which is
currently a reserved schema slot implemented in Phase 3. Under `area_uniform` only the
weaker density-swept table above is supportable. If the headline figure is to be the table
in this section, `camera_raster` is **not optional** — plan it into Phase 3 rather than
discovering the dependency while writing the paper.

---

## 6. Configuration presets

Shipped in `configs/`, hashed into `config_id` (`SCHEMA.md` §3.2).

| Preset | Purpose |
|---|---|
| `default.yaml` | the full sweep over §2–§4 |
| `phase1.yaml` | T-joint only, straight seams, **fixture off** (`contact_mode: "free"`), no features. The Phase 1 working config |
| `thin_sheet.yaml` | `t ∈ [1,3]`, `g ∈ [0,3]`, `ρ ∈ [0.25,4]`, all three sensor profiles — dense coverage of the §5 / §5.1 closure boundary |
| `reference_tjoint.yaml` | pins the repo's measured T-joint: `t = 8,4 mm`, `g = 1,1 mm`, seam 232 mm, spacing 2,7 mm, profile `d435i` **[repo]**. A regression fixture, and the one scene that ties the generator to a real measurement |
| `lab_fixture.yaml` | the measured steel plate, fixture forced on — the closest synthetic analogue of the Phase 9 real scans |
| `annotation_study.yaml` | 30 scenes spread across joint types, for Phase 5 hand-labelling |
| `smoke.yaml` | 8 scenes, fast, for CI and the determinism gate |

Ablation presets emit **paired arms**, never independent draws — `fixture_ablation.yaml`
(on/off) and `sensor_ablation.yaml` (three profiles) both hold substreams 0–2 fixed and vary
one axis, so every scene has an exact `twin_key` partner (`SCHEMA.md` §6.3).

---

## 7. Open items

| Item | Why it matters | When |
|---|---|---|
| ~~Confirm ISO 5817 Table 1 has no 508 row~~ | **RESOLVED** — verified two ways, §2.3. Table 1 numbering skips `3.3`, and "508" is absent from pp. 11–24 | done 2026-08-13 |
| **Freeze the D17 feature vocabulary** (§3.2) | it feeds `part_geometry_id` = the D11 split key. The `stiffener` case is genuinely undecided: additive features break the `w`-is-thickness invariant, so it is either a namespaced sub-body with its own frame or a separate workpiece object | **before Phase 2** |
| `camera_raster` sampling mode (§5.1) | the sharp per-sensor form of the window prediction depends on it. Under `area_uniform` only the weaker density-swept claim holds | **plan into Phase 3** |
| Butt/edge root-gap range has no ISO citation (§2.4) | currently **[ours]**; ISO 9692-1 would make it **[ISO]** | before submission, not before Phase 1 |
| Confirm `d435i` baseline / subpixel / min-Z against hardware (§4.0, §4.2) | **[ds]**; under D16 this gates only that profile's fidelity claim, not the release | before the release is frozen |
| Resolve the 10× sim-noise discrepancy (§4.2) | either a stale default in the twin or deliberate pessimism — worth knowing which. Note it lands near `stereo_poor` | Phase 3 |
| Measure the real fixture plate (§3.1) | under D12 the exact dims matter less — presence and pose both vary — but `lab_fixture.yaml` should be real | Phase 3 |
| Ship `stereo_good` / `stereo_poor` in the release, or use them only for the ablation? | affects release size and how the D16 axis is presented | before Phase 9 |
| `d_min` / `d_max` / edge-margin constants from Tomków and the JRM literature | Phase 7 tack rule; deliberately absent here so they ship as a *separate versioned rule*, per D8 | Phase 7 |
