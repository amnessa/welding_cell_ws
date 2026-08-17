# Weld Seam Dataset — Parameter Ranges

**params_version: 2.2.0** — tracks `SCHEMA.md`. 2.0.0 (2026-08-15) drops procedural features (D17 withdrawn); 1.2.0 added D18–D21 + ISO 9692-1.
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
| **[ISO]** | read out of a standard PDF in `../notes/`, page cited, **verified against the PDF**. ISO 5817:2023 = imperfection limits; ISO 9692-1:2013 = joint preparation |
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

**Where it is applied, per joint type.** `h` is a *step between the members' surfaces*, and
nothing else — in particular it is not a second root gap. It therefore acts along whichever
axis is perpendicular to the plates at that joint: along `z` for butt, corner and T, and
along the gap-free axis for the stacked joints. Adding it to the corner joint's `y` — where
the root gap already lives — made the joint faces sit `g + h` apart, which put them outside
`contact_tol_mm` and cost that joint type ~10% of its scenes to a fit-up that is, by this
very table, in tolerance. **[ours]**

**Unequal thicknesses are set flush on one face.** `h` is measured from the flush side.
Centring both plates on their mid-thickness plane instead — which equal-thickness geometry
silently implies — steps *both* faces by half the thickness difference, so a 6,3 mm plate
butted to a 3,1 mm one has no coplanar face pair at all and no seam is enumerated. On
dissimilar thickness a butt joint therefore has **one** centreline, on the flush side; the
other side is a step, which ISO 9692-1 treats as a transition rather than a second weld.
**[ours]**

**Angular misalignment hinges at the contact,** not at the part centre — two clamped plates
rotate about where they touch. This matters far more than its size suggests, because the
lever arm is half the plate width: 0,4° on a 179 mm plate lifts the welded edge by 0,62 mm
against a 0,1 mm root gap, and 2° on a 133 mm plate by 2,4 mm against a 1,2 mm tolerance.
Hinged at the centre, both cases lost the seam outright instead of recording a misaligned
one. **[ours]**

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
gap is a joint-**preparation** dimension — which is ISO 9692-1's subject, now in §2.6.

### 2.6 Joint preparation — ISO 9692-1:2013 (D18)

**A different standard for a different thing.** ISO 5817 above sets *imperfection* limits —
how far the realized joint may deviate. ISO 9692-1 sets *preparation* geometry — what the
joint is nominally supposed to be. **Keep the two vocabularies apart in the prose**, or a
welding reviewer will read a design angle as a defect. That distinction is exactly D18:
`joint.included_angle_deg` is the nominal, `fit.angular_misalignment_deg` is the deviation
from it. A 70° T-joint is not a 90° T-joint with a 20° defect.

**Fillet included angle (T, corner).** All values verified. **[ISO]**

| Table | Ref | Thickness | Included angle `α` | Gap `b` |
|---|---|---|---|---|
| 3 — one side, p. 11 | 3.1.1 | `t₁ > 2`, `t₂ > 2` | `70° ≤ α ≤ 100°` | `≤ 2 mm` |
| 3 — one side, p. 11 | 3.1.3 | `t₁ > 2`, `t₂ > 2` | `60° ≤ α ≤ 120°` | `≤ 2 mm` |
| 4 — both sides, p. 12 | 4.1.1 | `t₁ > 3`, `t₂ > 3` | `70° ≤ α ≤ 100°` | `≤ 2 mm` |
| 4 — both sides, p. 12 | 4.1.2 | `t₁ > 2`, **`t₂ > 5`** | `60° ≤ α ≤ 120°` | **not specified** |
| 4 — both sides, p. 12 | 4.1.3 | `2 ≤ t₁ ≤ 4`, `2 ≤ t₂ ≤ 4` | not specified | `≤ 2 mm` |

So **60–120° is [ISO], not [ours]**. Two details the summary reading loses: ref 4.1.2 has an
**asymmetric** thickness condition (`t₂ > 5`, not `> 2`), and it is the one fillet ref with
**no** gap limit — so "both tables cap the gap at 2 mm" is true of every fillet ref *except*
4.1.2. Record which sub-clause each scene satisfies.

**Footnote `b` on both Tables 3 and 4: *"Symbol is only applicable for α = 90°."*** Worth a
sentence in the paper — the ISO 2553 fillet symbol, the standard drawing convention itself,
silently assumes the right angle the generator is about to stop assuming.

**Butt root gap, square preparation.** This closes the standing open item. **[ISO]**

| Table | Ref | Thickness | Gap `b` |
|---|---|---|---|
| 1 — one side, p. 3 | 1.2.1 | `t ≤ 4` | `≈ t` |
| 2 — both sides, p. 7 | 2.1 | `t ≤ 8` | `≈ t/2` (process-dependent variants: `≤ t/2` for 13; `≤ 15`, `c = 0` for 52) |

**The gap scales with thickness.** It is not a flat 0–3 mm range, and the sampler must change
accordingly — a 1 mm sheet gets a ~1 mm gap, an 8 mm plate a ~4 mm gap. This interacts
directly with the §5 validity window, where `g` enters as a subtracted term.

**Edge joints.** Table 1 ref 1.1, *"raised edges"*, applies at **`t ≤ 2 mm`** with no
dimensions specified, remarked *"usually without filler metal"*. **[ISO]** A citable
constraint and a convenient one: edge joints are a **thin-sheet** preparation, `t ≤ 2 mm` is
exactly the stainless in the lab, and it is exactly where §5 predicts radius-PCA has no valid
radius. Restricting edge scenes to `t ≤ 2 mm` makes the joint type a consequence of the
standard rather than an arbitrary inclusion.

**Table 2 footnote `b`, p. 10: *"Dimensions given apply to the tacked condition."*** **[ISO]**
Carry this into Phase 7: the gaps the standard specifies are gaps *after tacking*, so the tack
rule and the fit-up parameters are coupled, not independent. One line in the tack-rule docs.

**Lap overlap has no ISO citation.** Neither 9692-1 nor ISO 2553 gives an overlap length;
ISO 2553 Table 5 no. 7.1 lists "lap" only as an edge-weld symbol with `s` = weld metal
thickness. Overlap stays **[ours]** — see §7.

### 2.6.1 Groove preparations — deferred to Phase 6 (D24)

ISO 9692-1 makes the choice **thickness-driven**, so when this lands the sampler picks the
preparation from `t` rather than inventing a distribution: **[ISO]**

| `t` (mm) | Preparation | Ref |
|---|---|---|
| ≤ 2 | raised edges | 1.1 |
| ≤ 4 | square | 1.2.1 |
| 3 – 10 | single-V, single-bevel | 1.3, 1.9.1 |
| 5 – 40 | single-V | 1.5, 2.2 |
| > 10 | double-V, double-bevel | 2.4, 2.9.1 |
| > 12 | single-U | 1.6, 2.6 |
| > 16 | single-J | 1.11, 2.10 |

A groove gives the butt joint **one** seam at the groove root instead of two coplanar face
centrelines, and the groove is cut on the sampled seam line rather than the seam being read
off the geometry afterwards — D3 applied to preparation. It needs a bevelled-edge
primitive, which is why it travels with Phase 6's curved geometry.

**Until then, `joint.prep` is `"square"` and §5.0's radius-PCA result keeps its
square-preparation scope.**

### 2.7 Lap and edge are the same topology at different offsets

Both have **parallel** parts — included angle `0°`, not 90°. They differ only in whether the
free edges coincide:

| | `included_angle_deg` | `stack_offset_mm` | Seams | Seam `dihedral_deg` |
|---|---|---|---|---|
| **lap** | 0 | `0 < offset < L` | 2 toes | 90° |
| **edge** | 0 | 0 (flush) | 1 along the free edge, 2 if the widths match | ~180° (degenerate) |

Part B keeps its **own** width in the edge layout and is aligned flush at the welded edge.
Forcing it to A's width makes every edge joint a pair of twins with both long edges flush
and pins the seam count at 2; real edge joints join parts of different widths, and then only
the aligned edge is a weld — the far side is a lap toe, which D22 classifies and rejects as
`wrong_class_for_joint`. `edge_equal_width_p` (0,35) keeps the flush-both-sides case in the
distribution, since continuous sampling would otherwise never produce an exact match.
**[ours]**

Two consequences worth stating in the paper:

1. **`included_angle_deg` and `dihedral_deg` are different quantities.** A lap joint has
   parallel parts (0° included) and a 90° seam dihedral. The schema carries both, and
   conflating them is easy enough that `SCHEMA.md` §4 annotates the distinction inline.
2. **Lap and edge fail the nearest-point rule for the same reason** — face-to-face contact
   over an *area* rather than a line. Unifying them under one `stack_offset_mm`, with edge as
   the `offset = 0` degenerate case, turns that shared failure mode into a *derivation*
   rather than two anecdotes.

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
| Root gap `g` | butt: `≈ t` (1-side) / `≈ t/2` (2-side) **[ISO]**; fillet: `≤ 2 mm` **[ISO]**; plus an over-range tail to 3 mm **[ours]** | §2.6 — **gap scales with thickness**, it is not a flat range. The over-range tail is what generates `below_D` and breaks radius-PCA on purpose |
| `included_angle_deg` | T, corner: 60 – 120° **[ISO]**; butt: 180°; lap, edge: 0° | §2.6, ISO 9692-1 Tables 3–4. **[ours]** for the butt/lap/edge degenerate values, which the standard states as topology rather than as an angle |
| `stack_offset_mm` | lap: `0 < offset < L`; edge: 0; `null` otherwise | §2.7. Lap overlap **[ours]** — no ISO citation found |
| Edge-joint thickness | `t ≤ 2 mm` **[ISO]** | §2.6, Table 1 ref 1.1. Not an arbitrary restriction — it is what the preparation is defined for |
| Linear misalignment `h` | per §2.1, by target level | **[ISO]** |
| Angular misalignment `β` | per §2.3, by target level | **[ours]** mapping over **[ISO]** values |
| Seam curvature radius | 30 mm – ∞ | **[ours]** Phase 6 |
| `contact_mode` | `free` when no fixture; else `flat 0.6 / on_edge 0.3 / propped 0.1` | **[ours]** how the assembly rests on the fixture; `flat` dominates because it is what magnets on a plate actually give |
| Fixture present | ~50%, **paired** (D12) | see §3.1 |
| Part geometry | plain slabs (Phases 1–5); `swept_slab` / `cylinder` / `tube` from Phase 6 | see §3.2 |
| Sensor profile | `d435i / stereo_good / stereo_poor` (D16) | see §4 |

### 3.1 Fixture — sampled and pose-varied (D12, revised 2026-08-13)

Models the steel plate the magnets hold against.

| Parameter | Value | Provenance |
|---|---|---|
| Primitive | `slab`, `role: "fixture"`, `object_id: 255` | `SCHEMA.md` §2.1 |
| **Presence** | **off in Phase 1; ~50% from Phase 2, emitted in exact pairs** | **[ours]** D12 |
| Dimensions | 600 × 400 × 10 mm nominal, ±20% on the two in-plane dims | **[ours]** — `lab_fixture` preset should use the measured plate |
| **Tilt** | **±10° from horizontal**, uniform in tilt direction | **[ours]** D12 |
| Stacked-joint β cap | 0,4° for `lap` and `edge` | **[ours]** D23 — clamped face to face, so relative tilt is physically suppressed |
| **Height** | working surface `z` **not pinned**; offset ±50 mm | **[ours]** D12 |
| In-plane yaw | uniform 0 – 360° | **[ours]** |
| Surface | planar; no texture in tier 1 | |

**Pairing is a generation strategy, not a sampling rate.** Drawing presence independently
at p = 0.5 gives random on/off with no guaranteed partner. Instead each geometry seed emits
**both** arms — one fixture-on, one fixture-off — joined by `twin_key` (`SCHEMA.md` §6.4).
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

### 3.2 Part geometry — plain slabs through Phase 5

**No procedural features.** D17 proposed generating chamfers, holes, slots, notches and
stiffeners on the slabs; it was **withdrawn 2026-08-15** as decoration dressed as diversity.

| Phase | Primitives generated | What varies |
|---|---|---|
| 1 – 5 | `slab` only | dimensions: `L`, `W`, `t` |
| 6 | `+ swept_slab`, `cylinder`, `tube` | genuine shape — curved plates, pipe-on-plate, cylinder-on-cylinder |
| 9 | scanned MDF workpieces | genuine shape, plus saw kerf, edge break, warp and paint texture |

The consequence to state rather than hide: **over Phases 1–5 the D11 split holds out
_dimensions_, not geometry.** Both are legitimate held-out axes; only one of them is what
"held-out geometry" sounds like. `SCHEMA.md` §5.4 carries the same caveat at the point where
`part_geometry_id` is defined, so a reader meets it wherever they enter.

Real irregularity is what the Phase 9 scans are for, and it is not something a feature
vocabulary reproduces honestly — a modelled chamfer is a clean bevel; a real edge break is
not.

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
in exact twins via `twin_key` (`SCHEMA.md` §6.4). And the datasheet-confirmation item stops
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

### 5.0 What the ISO-scaled gap does to the prediction (D18 / §2.6)

The tables above sweep `g` as a free parameter. Once §2.6 fixes `g` as a **function of `t`**,
the window condition `g + spacing < t` collapses to something much sharper. Scoped to
**square preparation**, which is all Phases 1–6 generate (`joint.prep = "square"`):

| Joint | ISO gap | Window condition | Consequence |
|---|---|---|---|
| **butt, welded one side** (Table 1 ref 1.2.1, `t ≤ 4`) | `b ≈ t` | `t + spacing < t` | **Never satisfiable.** `spacing > 0` always |
| butt, welded both sides (Table 2 ref 2.1, `t ≤ 8`) | `b ≈ t/2` | `spacing < t/2` | `t > 2 · spacing` |
| fillet — T, corner (Tables 3–4) | `b ≤ 2 mm` | `2 + spacing < t` | `t > 2 + spacing` |

| Sampling | spacing | butt 2-side needs | fillet needs |
|---|---|---|---|
| `ρ = 0,25` pts/mm² | 2,00 mm | `t > 4,0` | `t > 4,0` |
| `ρ = 1` | 1,00 mm | `t > 2,0` | `t > 3,0` |
| `ρ = 4` | 0,50 mm | `t > 1,0` | `t > 2,5` |
| `d435i` raster @ 500 mm | 0,74 mm | `t > 1,5` | `t > 2,7` |
| `d435i` raster @ 1000 mm | 1,48 mm | `t > 3,0` | `t > 3,5` |
| `stereo_poor` raster @ 500 mm | 1,11 mm | `t > 2,2` | `t > 3,1` |

**The first row is the striking one:** for a square-preparation butt joint welded from one
side, ISO specifies a root gap equal to the plate thickness — so the radius-PCA validity
window is empty **at every thickness, every density, and every sensor**. Not "closes for thin
sheet"; closed outright, by the standard's own preparation geometry.

Three caveats that must travel with that claim, or it is overstated:

- It is scoped to **square preparation**. Ref 1.3 (single-V, `3 < t ≤ 10`) has `b ≤ 4 mm` and
  a bevel, so it is a different geometry — and out of scope until a `prep` beyond `"square"`
  is generated.
- `b ≈ t` is a *recommended* preparation dimension, not a tolerance. Real fit-up varies, and
  Table 2's footnote says the dimensions apply to the **tacked** condition (§2.6).
- It says radius-PCA has no valid *radius*, not that no method works. A plane-pair baseline is
  unaffected by this bound — which is itself the comparison the table is for.

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
discovering the dependency while writing the paper. Note also D20: under a raster the hidden
surface is sampled separately, so density is not uniform across the visibility boundary and
any density-sensitive metric must be computed within a mask class (`SCHEMA.md` §5.1).

### 5.2 The fourth axis — included angle (D18)

Everything above treats the window as a function of `(t, g, ρ)`. It is really a **surface over
`(t, g, ρ, α)`**, because radius-PCA's `V = λ₃/(λ₁+λ₂+λ₃)` signature is a function of the fold
the neighbourhood spans, and D18 makes that fold a sampled parameter rather than a constant 90°.

The two limits are worth stating even before measuring:

- **`α → 180°`** (flush, the `edge`-joint degenerate case, §2.7) — there is no fold. `λ₃ → 0`,
  `V → 0`, and the seam is invisible to a curvature feature *no matter how good the sensor is*.
- **`α → 0°`** (parallel, the `lap` case) — the neighbourhood spans two parallel surfaces `g`
  apart. `V` is driven by the separation rather than by a fold, so the feature fires on a
  quantity that has nothing to do with the seam being where it is.

That is the mechanism behind "lap and edge are where naive methods die" (Phase 4 plot 4) —
and §2.7's unification means it is **one** mechanism at two ends of one parameter, not two
separate anecdotes. **No closed form is asserted here**; Phase 4 plot 6 measures the surface.
The prediction on offer is only the sign and the limits, which is enough to be falsifiable.

---

## 6. Configuration presets

Shipped in `configs/`, hashed into `config_id` (`SCHEMA.md` §3.2).

| Preset | Purpose |
|---|---|
| `default.yaml` | the full sweep over §2–§4 |
| `phase1.yaml` | T-joint only, straight seams, **fixture off** (`contact_mode: "free"`). The Phase 1 working config |
| `thin_sheet.yaml` | `t ∈ [1,3]`, `g ∈ [0,3]`, `ρ ∈ [0.25,4]`, all three sensor profiles — dense coverage of the §5 / §5.1 closure boundary |
| `reference_tjoint.yaml` | pins the repo's measured T-joint: `t = 8,4 mm`, `g = 1,1 mm`, seam 232 mm, spacing 2,7 mm, profile `d435i` **[repo]**. A regression fixture, and the one scene that ties the generator to a real measurement |
| `lab_fixture.yaml` | the measured steel plate, fixture forced on — the closest synthetic analogue of the Phase 9 real scans |
| `annotation_study.yaml` | 30 scenes spread across joint types, for Phase 5 hand-labelling |
| `smoke.yaml` | 8 scenes, fast, for CI and the determinism gate |

Ablation presets emit **paired arms**, never independent draws — `fixture_ablation.yaml`
(on/off) and `sensor_ablation.yaml` (three profiles) both hold substreams 0–2 fixed and vary
one axis, so every scene has an exact `twin_key` partner (`SCHEMA.md` §6.4).

---

## 7. Open items

| Item | Why it matters | When |
|---|---|---|
| ~~Confirm ISO 5817 Table 1 has no 508 row~~ | **RESOLVED** — verified two ways, §2.3. Table 1 numbering skips `3.3`, and "508" is absent from pp. 11–24 | done 2026-08-13 |
| `camera_raster` sampling mode (§5.1) | the sharp per-sensor form of the window prediction depends on it. Under `area_uniform` only the weaker density-swept claim holds | **plan into Phase 3** |
| ~~Butt/edge root-gap range has no ISO citation~~ | **RESOLVED** — ISO 9692-1:2013 Table 1 ref 1.2.1 (`b ≈ t`), Table 2 ref 2.1 (`b ≈ t/2`), edge = ref 1.1 at `t ≤ 2 mm`. All verified, §2.6. **The sampler must change**: gap scales with thickness | done 2026-08-15 |
| **Lap overlap length** stays **[ours]** (§2.6) | neither ISO 9692-1 nor ISO 2553 gives a minimum. AWS D1.1 or a fabrication text may quote one as a multiple of `t` | one lookup before submission, not before Phase 2 |
| Non-square preparations (single-V, backing, centering lip) | §5.0's "never satisfiable" result is scoped to square prep. Widening `joint.prep` beyond `"square"` is a real extension of the joint space, and Table 1 refs 1.2.2–1.3 give the dimensions | after Phase 6, if at all |
| Confirm `d435i` baseline / subpixel / min-Z against hardware (§4.0, §4.2) | **[ds]**; under D16 this gates only that profile's fidelity claim, not the release | before the release is frozen |
| Resolve the 10× sim-noise discrepancy (§4.2) | either a stale default in the twin or deliberate pessimism — worth knowing which. Note it lands near `stereo_poor` | Phase 3 |
| Measure the real fixture plate (§3.1) | under D12 the exact dims matter less — presence and pose both vary — but `lab_fixture.yaml` should be real | Phase 3 |
| ~~Ship `stereo_good` / `stereo_poor` in the release?~~ | **RESOLVED — yes, all three profiles ship.** The D16 sensor-quality axis is only reproducible by others if the scenes behind it are downloadable. Cost is bounded: profiles differ only in substreams 5–6, so the three arms share geometry and join on `twin_key` | done 2026-08-15 |
| `d_min` / `d_max` / edge-margin constants from Tomków and the JRM literature | Phase 7 tack rule; deliberately absent here so they ship as a *separate versioned rule*, per D8 | Phase 7 |
