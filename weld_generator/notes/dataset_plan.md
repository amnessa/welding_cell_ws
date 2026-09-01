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
| D13 | D4 requires **both faces on `role: "workpiece"` objects**; part–fixture contact is rejected as `fixture_contact` | A plate standing on the fixture has two exterior faces, a 90° dihedral and an escaping bisector — it passes D4 on pure geometry and is *not* weldable. The rejection is not derivable from geometry, which makes it the most interesting class in the weldable-vs-interior metric (§8) |
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
| D25 | The **MPS label is a versioned rule function**, never stored truth: `mps_rule-0.1` = argmax over weldable seams of *visible* arclength, ties broken by larger dihedral fold then lower seam id, null below `min_len_mm` | Fourth instance of a pattern the project already follows — tacks (D8), the noise realisation (D14), the seam-under-gap choice (D19). Visible arclength rather than total length is what makes it well-posed: with partial seams out of scope, seam length equals contact length, so a T-joint's two fillets and a lap's two toes have *equal* total length and an argmax over it would tie in most scenes. Settled 2026-08-20 |
| D26 | **Two camera regimes, sampled per config**: `approach_cone` (from the target seam's well-observed viewpoint region) and `uniform_sphere` (the Phase 3 sampler). Both twin-paired | The Phase 3 gate — "the sampler must not be polite" — encoded an assumption the advisor meeting overturned. Under coarse positioning a polite sampler is the *correct* model of deployment, not a weak one. Keeping both makes the `approach_cone`/`uniform_sphere` delta on paired seeds a measurement of **what coarse positioning is worth**. The good-viewpoint region is a "Swiss cheese slice", not a clean cone, and it is **already computed** — the azimuth × elevation map in `notebooks/02`. Sample it empirically rather than parameterising a cone. Settled 2026-08-20 |
| D27 | Part dimensions vary **across** the seam, never **along** it: the along-seam dimension is pinned by seam length; width, height and overhang are free; thickness is free subject to ISO 9692-1 | The advisor proposed expanding one seam into many scenes via a list of plate sizes. Right perpendicular to the seam, wrong along it — growing a plate along the seam axis lengthens the contact run, and with partial seams out of scope the seam length *is* the contact length, so it is no longer the same seam. The constrained form makes "same seam, many parts" well-defined, and gives a clean held-out axis: train on narrow plates, test on wide ones, same seam. Settled 2026-08-20 |

Phase 0 froze D1–D11 in [`../docs/SCHEMA.md`](../docs/SCHEMA.md) and
[`../docs/PARAMETERS.md`](../docs/PARAMETERS.md). D12–D16 were settled 2026-08-13; D17 was withdrawn 2026-08-15; D18–D21 settled 2026-08-15;
D22–D24 settled 2026-08-16; **D25–D27 settled 2026-08-20** from the advisor meeting — see §3.

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

## 3. Task definitions

*Added 2026-08-20 from the advisor meeting.*

The plan had so far assumed one ground truth per scene: the set of weldable seams. The meeting
split that into **two tasks with different label types**, and the second has a label that is not
derivable from geometry alone. Both are served by the same generator; they differ in what is
given and what is asked.

### Task 1 — Complete seam recovery

> *"Varsayalım ki sen bunu mükemmel taradın. Al olası tüm seam'leri."*

**Given:** the full-geometry cloud, no occlusion.
**Return:** every weldable seam, each with its class and geometry.

This is what Phases 1–3 already produce, and its label is **exact geometry** — no convention
enters anywhere. It is the task the dataset's central claim rests on.

Because generation is inverted (D3), the per-point form of this label is free: every point
carries `face_id`, and every seam names the `face_pair` that produced it, so each surface point
can be labelled with the seam it belongs to, or none. Task 1 is therefore posable three ways
from one file — per-point segmentation, per-seam instance grouping, or curve regression —
without storing anything new.

### Task 2 — Most probable seam (MPS)

> *"Al, baktığın açıdaki en olası seam'i bul."*

**Given:** one view.
**Return:** one seam — its **class** and its **location**.

The advisor's justification, and the assumption the paper must state outright:

> *"Genelde algoritma şöyle çalışıyor: önce bir kaynak bölgesine git. Gittin mi kaynak seam'ini
> bul."*

The robot is coarsely positioned into the weld region first; seam finding happens afterwards.
The camera is therefore already inside the approach region and is not sampling an arbitrary
sphere. This is a citable workflow assumption, not a convenience — and it is what makes a
single-answer task well-posed at all. **Draw it as a figure.**

**The MPS label is a convention, not truth** (D25), handled like every other convention in this
project: ship a versioned rule function over the exact geometry, never a stored label. Store
`mps` alongside `tacks` as a derived block: `{rule_version, params, seam_id, class}`.

Consequence to check in Phase 4: under the Phase 3 occlusion distribution (58% of seams above
0,98 occluded, 40% at exactly 0), visible arclength is close to binary, so MPS will usually
reduce to *"the one seam that is visible at all."* That is a legitimate answer but a weak task.
**Grading the occlusion distribution — D26 — is what gives MPS teeth**, and it must happen
before MPS is evaluated.

### Scope: what was declined from the meeting, and why

Recorded so they are not silently reopened.

| Proposal | Status | Reason |
|---|---|---|
| Ground truth conditioned on **loading conditions** | **Out of scope** | Which seam carries load is structural knowledge, not visible geometry. No vision method can recover it, so it cannot be a vision label. Agree explicitly with the advisor that MPS is a *geometric proxy* adopted for tractability, and say so in the paper — otherwise it arrives as a reviewer question instead of a stated limitation |
| **Partial seams** (pre-welded sections, weld only the middle) | **Out of scope** | Requires knowing which regions are already welded. Nothing in a single view carries that, so the label would encode information the input cannot contain |
| **Same seam, varying part sizes** | **Constrained, not dropped** | D27 — dimensions across the seam are free; the along-seam dimension is pinned by the seam |

---

## 4. Baselines — the seven-method comparison

**Superseded 2026-08-20.** The `A / B / C / D` naming fixed on 2026-08-18 is retired; the
advisor meeting replaced it with a named seven-method comparison. Recorded rather than erased
so the change is traceable: `A` became `ours`, `B` split into `lit-ransac` and `lit-ppf`.

### Framing note — this is the change that matters most

`ours` is **not claimed as a novel extractor.** It is one entry in the comparison, carried into
the thesis as the method the robot pipeline actually uses.

Worth stating plainly in the paper, because it removes the largest review risk in the project:
the contribution is the **dataset and the comparison**, and a reviewer cannot attack the dataset
by attacking a novelty claim that was never made. It also dissolves the tuning-fairness problem
— with no horse in the race, equal treatment of all seven is the obvious protocol rather than a
concession.

### The seven

| Name | Mechanism | Source |
|---|---|---|
| `ours` | radius-PCA curvature + nearest-point midpoint | the robot pipeline (`README §8`) |
| `lit-ransac` | improved RANSAC multi-plane fitting → plane intersection lines → inliers projected onto the weld vector for endpoints → dihedral for torch pose | Yi et al., *Automation in Construction* 2026 |
| `lit-ppf` | PPF-coplanarity planes → orthogonal-pair local Hough voting (eqs. 21–23) → feature points by the distance component → farthest-pair corners; proposed as a RANSAC alternative, **and deterministic as published** | Wang et al., *Sci. Rep.* 14 (2024) 21137 |
| `lit-regiongrow` | region growing seeded at the smoothest point by δ = λ₀/(λ₀+λ₁+λ₂), then least-squares fit of near-edge points | *Coarse-to-Fine Detection of Multiple Seams* |
| `lit-lobb` | K-Net **component** segmentation → edge pixels where two masks meet → ROI by shape extension → LOBB bounding-box flatness → tanh activation → binary K-means → polynomial fit | Zhang et al., *RCIM* 95 (2025) 102987, with the LOBB descriptor of *IEEE T-ASE* 22 (2025) 75 |
| `lit-pcaslice` | PCA-based adaptive slicing with the slicing direction determined from the data, centreline per slice | *3D vision-based intersecting pipe welding path planning* |
| `lit-modelreg` | registration of a CAD model to the scan, welding path transferred from the model | *A novel model-based welding trajectory planning method for identical structural workpieces* |

Two are chosen for what they add to the argument rather than for performance. **`lit-lobb`
classifies crease versus boundary points** — precisely the distinction between a fillet seam and
a lap toe, so it is the one literature method whose feature space can in principle express the
D22 joint taxonomy. **`lit-modelreg` requires CAD by construction**, which anchors the top of
the oracle ladder with a real published method rather than a hypothetical one.

### The coverage prediction — and `ours` is on the wrong side of it too

`lit-ransac` derives seams from plane **intersections**; `lit-ppf` is built on **orthogonal**
plane pairs. Neither mechanism can express a butt or edge seam, which come from D4's *coplanar
exposed* arm — two faces sharing a plane, no intersection line, no orthogonality. That is
structural, not a tuning failure, and it is the same discovery the D4 enumeration made when it
needed a third arm.

**Measured 2026-08-20, and it applies to `ours` as well.** Radius-PCA is a curvature measure,
and two parallel plates have no curvature between them: on an edge joint the band covers the
*entire area between the parts* rather than the flush edge, giving precision 0,01–0,05 against
a band width of 170 mm. Every other joint type sits at 0,29–0,92. This is not a parameter
problem and may not be solvable within the mechanism — a variance ratio has nothing to respond
to when the dihedral is 180°.

That makes the headline stronger, not weaker. The **mechanism × D4-arm coverage table** is
measured rather than argued, and it now reads: *seam extraction as the field practises it —
including the method in our own robot — is built on the fillet assumption, and its geometric
machinery cannot express half the joint taxonomy.* A finding that indicts our own method is
considerably harder to dismiss than one that only indicts everyone else's.

#### `lit-ransac` measured, 2026-08-20 — the prediction holds for edge, and fails for butt

First implementation of one of the seven (`scripts/baselines/lit_ransac.py`, Yi et al. 2026,
§5–§6). Full-visibility arm, 44 `out/phase3` scenes, seed 0, ρ = 1 pt/mm², F1 at 3 mm.
`L0` = input restricted to the paper's ~40 mm annotated weld region (their PointNet++ stage,
supplied here as an oracle); `L1` = whole workpiece. **One seed, one density, and on the
unbalanced corpus** — 2 lap and 4 edge scenes. Every one of those three qualifiers turned out
to move a row; read the per-type numbers as directional and the spread section below as the
actual result.

| joint | L0 F1 | L1 F1 | L0 precision | L0 recall | n |
|---|---|---|---|---|---|
| T | 0,75 | 0,58 | 0,69 | 0,91 | 12 |
| lap | 0,55 ⚠ | **0,00** ⚠ | 0,38 | 1,00 | **2** |
| butt | 0,54 | 0,50 | 0,49 | 0,60 | 15 |
| corner | 0,51 | 0,45 | 0,39 | 0,86 | 11 |
| edge | **0,00** | **0,00** | 0,00 | 0,00 | 4 |

⚠ The lap row did not survive a seed sweep — see item 4. It is left in place as the record of
what one draw looked like, not as a number.

Four things, and three of them revise text written above.

1. **Edge is 0,00 — as predicted, but one gate down from where the prediction put it.**
   The faces are parallel *in the geometry*; they are almost never parallel in the *fit*, because
   RANSAC on sampled points leaves a fraction of a degree between them. So `n₁ × n₂` is not
   numerically zero, `intersection_line` returns a line — in mid-air, hundreds of mm away — and
   it is the **orthogonality gate** that rejects it, at a fold angle of 0,5–7°. Same conclusion,
   different line of code, and worth getting right because the distinguishing evidence is the
   *angle at which pairs are rejected*: every edge pair is thrown out near 0°, never near 90°.
   A tuning failure looks like the opposite. `LitRansacResult.pairs` records the verdict and
   fold angle of every pair for exactly this reason, so the claim is a column in a table: the
   widest fold across every edge pair in the corpus is 15,8°.

   One exception, kept rather than rounded away: over 90 seeded edge runs, 89 return nothing and
   one scores F1 0,23 — a line grazing the truth, at a third of the T-joint figure.

2. **Butt is *not* 0,00, and the prediction above is wrong about it.** The reason is a feature
   of this generator the prediction did not account for: a butt joint with a **root gap** has
   two gap walls, and a gap wall *is* orthogonal to the plate faces. The intersection of wall
   and face lands on the seam. So the coverage claim has to be narrowed — the mechanism cannot
   express a *zero-gap* butt, and D18 samples the gap from an ISO 9692-1 range that is almost
   never zero. Keep the claim for edge, restate it for butt. This is the kind of correction only
   a generator that models joint preparation can produce.

3. **Recall is high and precision is not** — 0,91 against 0,69 on T. The method finds the seam
   and also finds every plate border, because a plate's own top face and edge face are an
   orthogonal intersecting pair indistinguishable from a fillet. **This is the false-positive
   result the Phase 4 checklist asks for**, and it arrives without a fixture in the scene.

4. **The L0→L1 delta is largest on lap — and the first reading of it was wrong.** A lap toe is
   made by the top plate's **edge face**, a ribbon roughly 8 × 200 mm: a small plane, and a badly
   conditioned one.

   At seed 0 and ρ = 1 the numbers said 0,55 → 0,00, and at ρ = 0,5 they said 0,00 at both arms.
   That looked like a clean two-stage mechanism — `T_mpp` deletes the ribbon at L1; density
   decides whether it fits straight at L0. **Neither half survived varying the seed.** Over
   2 scenes × 4 densities × 8 seeds:

   | | L0 | L1 |
   |---|---|---|
   | mean F1 | 0,31 | 0,07 |
   | runs returning nothing | 45% | 89% |
   | runs that fitted the ribbon at all | most | 8 of 64 |

   *What survives:* the segmentation effect is real and large. `T_mpp` is an **area ratio**, the
   ribbon is under 2% of a whole workpiece, and at L1 it is dropped before it is ever a plane —
   in 89% of runs. A published constant *mostly* deletes a joint type, and "mostly" is the
   honest word; "always" is what one seed said.

   *What does not survive:* the density story. Spread across seeds at fixed (scene, ρ) is 0,27;
   spread across ρ at fixed (scene, seed) is 0,31. Same size. At n = 2 lap scenes neither is
   attributable, and the single-seed density effect was noise wearing a mechanism's clothes.

   Recorded at length because it is the **third** time in this project that a per-type conclusion
   from a handful of scenes has been overturned by the next measurement. The rule that follows:
   for a randomised method, no per-type claim is made from fewer than a seed sweep, and no claim
   at all is made from 2 scenes.

#### `lit-ransac` **validated against the paper's own metric**, 2026-08-20

Everything above is measured with this project's metrics, and no number in Yi et al. was ever
computed with them — so none of it can say whether the reimplementation is faithful. §7.2.1
can: one seam per workpiece, taught manually, **max ME 1,16 mm and max RMSE 0,64 mm** across
four workpieces. `metrics.matched_path_errors` computes that form; `notebooks/04` runs it.

Conditions matched to theirs: T joint, 0,6427 mm point spacing (their measured cloud
resolution, §5.2), the ~40 mm weld region (§3), full visibility, `nominal` curve, and only
scenes where §5.2's own validity window is open. 8 scenes × 5 seeds.

| condition | RMSE | ME | paper |
|---|---|---|---|
| noiseless, §5.4 refit on | **0,60** | 0,81 | max RMSE **0,64**, max ME **1,16** |
| noisy, §5.4 refit on | 0,77 | 1,19 | |
| noiseless, **no refit** | **0,00** | **0,00** | — |
| eq. 19 as printed | 2,67 | 5,04 | — |

Three independent confirmations, the third strongest:

1. **The headline lands on it** — 0,60 against 0,64, 0,81 against 1,16, slightly better on
   both. That is the correct side: §7.2.1 measures 0,23 mm of hand-eye, 0,36 mm of TCP and
   0,28 mm of teaching error *inside its own ground truth*, and ours is exact.
2. **§5.4 does what §5.4 claims.** The centroid refit is a **noise** remedy, so it must help
   only where there is noise — and it does: 0,77 against 1,22 with noise, 0,60 against 0,00
   without. A correct implementation has to show that sign flip. Eq. 19 as printed is 4–5×
   worse in both regimes, which settles deviation 2 empirically.
3. **Noiseless without the refit, the error is exactly 0,000.** Three exact points give an
   exact plane; two exact planes give an exact line. The whole §6.1/§6.2 chain returning the
   seam to floating point is a far stronger statement than any millimetre figure — there is
   nothing left for a bug to hide in.

**A harness bug this found, worth generalising.** The first run gave RMSE 3,3 mm and ME
13,7 mm. The decomposition said the *line* was accurate to 0,97 mm and the seam was **38 mm
too long**. The fault was in `seam_region_oracle`, not the method: "within 20 mm of the seam"
is a **capsule**, and a capsule reaches 20 mm past the seam's last point, sweeping up base
plate no annotator would call weld region. §6.2 reads the seam's extent off the points it is
given, so it read the capsule's. `end_margin_mm` now separates the annotation's *width* from
its *extent* and defaults to zero. **An oracle standing in for a learned stage can inject a
failure indistinguishable from a failure of the method it feeds** — check every future one
the same way, by decomposing the metric before believing it.

**A result the paper could not have produced.** §5.2 states a validity window and never tests
it: *"given the steel plate thickness of 6 mm, a threshold exceeding this value inevitably
leads to the erroneous merging of parallel planes."* Their plate is 6 mm and never varies.
This corpus goes to 1,7 mm, where `T_d2 = 2 mm` exceeds the plate, the two faces merge, and
the fitted plane is a compromise between them: **median RMSE 0,57 mm above 4 mm of plate
against 1,51 mm below it**, rank correlation −0,78 against thickness. The same two-sided
bound `ours` has on its PCA radius, on a different quantity — and neither is testable without
a generator that varies the plate.

**D19 confirmed, quantitatively.** The three curve definitions give median RMSE `nominal`
0,599, `root` 0,735, `gap_mid` 1,078 — a **0,48 mm spread against a 0,64 mm target**. D19
predicted exactly this (*"the ambiguity is larger than the ~0,6 mm RMSE the literature
reports"*) and it is now measured. `nominal` wins, as it should: SCHEMA §1.3 defines it as the
intersection of the two extended supporting planes, which is what this method computes.
**Publish the conversion table** — a systematic offset of this size may explain part of the
disagreement between published accuracies.

#### `lit-regiongrow` implemented, 2026-08-20 — and it hits `ours`' wall

Wei et al. §III-D. Coarse-to-fine: FastSAM segments each *surface* in the RGB image and the
cloud is cropped to where two surfaces meet (§III-C, supplied here as the same
`seam_region_oracle`), then region growing on the point cloud finds the seam.

**Its curvature is the same quantity `ours` uses.** Eq. 3 is `δ = λ₀/(λ₀+λ₁+λ₂)`, which is
`radius_pca.surface_variation` under another name — pinned by a test that asserts the two
agree to 1e-9. So the two methods share a feature and differ only in what they do with it,
which makes each difference measurable on its own rather than arguable:

| | `ours` | `lit-regiongrow` |
|---|---|---|
| neighbourhood | radius ball | **k nearest** |
| decision | global threshold on δ | **region growing** from the smoothest point, split on normal-angle jump |
| "two different parts?" | `object_id` **oracle** | **its own grown regions** |

The third row is the most valuable thing in the paper for this project. `ours` loses F1
0,75 → 0,03 when `object_id` is withheld; this method runs the same test off segmentation it
produced itself. Whether that survives contact is now a measurement rather than a hope.

**Only one constant is published** (§IV-A's 3 mm voxel grid). `k`, Threshold1, Threshold2 and
the two-surface radius are given no value anywhere, so a choice has to be made, and PCL's
defaults — the implementation Alg. 1 is pseudocode for — **do not work here**: k = 30 at a
3 mm grid spans a ~9 mm ball, wider than an 8 mm plate, so the normal wraps around the plate's
own two faces and *every point in the scene* comes back an edge. That is `ours`' "must not
bridge a plate's own two faces" bound arriving in a second method by a different route.
Defaults were swept against constructed truth instead and are labelled tuned. Note the
protocol asymmetry: `lit-ransac` publishes all three of its constants and so gets no such
freedom — **equal treatment of the seven means tuning what a paper leaves unspecified and
saying which ones those were.**

**Measured on the balanced corpus, 2026-08-20 — 50 T + 50 corner**, which is the fair
comparison set: Wei et al.'s four workpieces are fillet welds on steel structures, and this is
the joint their method was built and tuned for.

**The coarse-detection ladder, and it revises everything below it.** Four rungs, because the
first measurement gave this method *`lit-ransac`'s* oracle — a band drawn around the truth
seam — which is not the stage §III-C describes:

| arm | what is supplied | T F1 | corner F1 | T RMSE | seam pts |
|---|---|---|---|---|---|
| `L0-band` | a ~40 mm band round the **truth seam** (Yi et al.'s stage) | **0,38** | **0,45** | 2,2 | 166 |
| `L0-crop` | §III-C's own crop, derived from surfaces; regions re-grown | 0,13 | 0,10 | 3,8 | 2 101 |
| `L0-paper` | §III-C's crop **and** perfect surface labels | **0,05** | **0,02** | **40,9** | 5 569 |
| `L1` | nothing | 0,13 | 0,10 | 3,8 | 2 102 |

1. **This method's own coarse stage is worth nothing.** `L0-crop` and `L1` agree to three
   decimals (0,125 / 0,126 and 0,102 / 0,101). The surface-derived crop keeps 331 k of 522 k
   points, so cropping to it is the same as not cropping. Every earlier claim here that "the
   segmentation effect is real and large" was measuring the **seam band** — Yi et al.'s
   oracle, drawn around the answer — and is withdrawn.

2. **Supplying *perfect* surface labels is worse than supplying nothing**: F1 0,05 against
   0,13, RMSE 41 mm against 3,8. This is the face-versus-part problem in its purest form. The
   two-surface test assumes a surface boundary indicates a seam; give it perfect surfaces and
   **every plate edge qualifies**, so the crease set triples and collapses into one or two
   clusters spanning the workpiece. Region growing was accidentally helping by
   *under*-segmenting — merging faces it should not have, which suppressed some rim junctions.
   A method improved by a worse segmentation is a strong statement about its premise.

   *Caveat, unmeasured:* `face_id` supplies all 12 faces including ones no camera sees, while
   FastSAM segments only camera-visible surfaces. On the single-view arm their stage would
   return fewer surfaces and might land between `L0-paper` and `L0-crop`. Worth measuring
   before this is written up.

**The oracle-free claim does not survive, and the reason is precise.** The two-surface test
was the reason this method is in the seven — it asks "do these two points sit on different
surfaces?" using regions it grew itself, where `ours` asks the same question of an `object_id`
oracle. On a cropped weld region it works: 55% of surviving points land within 3 mm of truth.
On a whole workpiece it admits **ten times as many points, 89% of them nowhere near a seam.**

The flaw is a category error, not a threshold: **the test asks "two surfaces?" when the
question that matters is "two parts?"**. A plate has six faces, so region growing gives one
plate several regions, and *every edge of every plate* is a junction of two of them —
indistinguishable, by this test, from a fillet. `ours` collapses the same way (F1 0,75 → 0,03)
for a different reason: it needs the oracle to answer the question at all. Neither method has
a point-cloud-only way to tell a *part boundary* from a *face boundary*, and that is now a
statement about the method class rather than about either paper.

**Accuracy, on the paper's own metric — and the first reading of this was wrong.** Reporting
the per-scene *maximum* RMSE (2,8–3,3 mm) conflated two failures that have to be separated,
because the method fails at only one of them. Per matched fragment, over the same 100 scenes:

| arm | joint | median RMSE | median ME | **coverage** | recall | precision |
|---|---|---|---|---|---|---|
| L0 | corner | **0,94 mm** | 1,37 | **35%** | 0,78 | 0,40 |
| L0 | T | **1,17 mm** | 1,49 | **10%** | 0,37 | 0,76 |
| L1 | T | 3,19 mm | 3,45 | 10% | 0,28 | 0,11 |
| L1 | corner | 3,92 mm | 5,11 | 24% | 0,35 | 0,08 |

**Localisation roughly holds; coverage collapses.** Where the method returns a seam fragment,
that fragment is on the seam — median 0,94–1,17 mm against Wei et al.'s reported 0,37–0,56 mm,
so about 2× rather than 5×, and **35% of fragments meet their worst reported RMSE outright**
with **41% inside their stated 1 mm maximum-error requirement**. What fails is that a
fragment covers **10% of its seam on a T joint** and 35% on a corner. Their metric cannot
show this: §IV-A samples 3 points per linear seam and 10 on the curved one, and reports the
distance at those points. A method returning an accurate 14 mm stub of a 142 mm seam scores
well on it. **This dataset's exact truth is what makes the distinction visible**, and it is a
better argument for the generator than any accuracy number.

**Read the precision/recall pair, not F1.** T and corner have almost the same F1 (0,47 / 0,49)
by opposite routes: T returns short accurate stubs (precision 0,76, recall 0,37), corner
returns over-wide coverage (precision 0,40, recall 0,78). A single scalar hides that the two
joints break the method in opposite directions.

**T against corner separates the mechanism from the method.** Same code, same parameters:
corner coverage 35% against T's 10%, and corner's median fragment is more accurate too. A T
joint's two fillets are one plate-thickness apart and proximity is all the clustering has to
go on. This is the wall `ours` hit, reached independently by a method that shares only the
curvature feature — and **neither paper could have found it**, because their workpieces are
large steel structures whose seams are nowhere near each other.

*(A tempting explanation that the data does not support: fragments being matched to the wrong
fillet. Checked — only 19% of T fragments are closer to the other fillet than to their own.
The dominant effect is fragmentation, not mis-assignment.)*

Written up in `notebooks/05_lit_regiongrow.ipynb`, which also carries the neighbourhood-reach
plot and the fragmentation-versus-merge figure.

Still open: **direction-aware clustering, built once for both methods.** It is now the
highest-value piece of work in Phase 4 — the named blocker for `ours` and the measured blocker
for `lit-regiongrow` — and a fix that moves both is a result about the mechanism rather than
about either method. Second: a **part-boundary versus face-boundary** test that needs no
`object_id`. Both methods now fail on the same missing primitive, which makes it worth
building once and reporting as its own contribution rather than as a patch to either.

#### `lit-lobb` implemented, 2026-08-20 — and the three coarse stages are three different oracles

Zhang et al., *RCIM* 95 (2025) 102987, with the LOBB descriptor from the same group's
*IEEE T-ASE* 22 (2025) 75. Pipeline: K-Net semantic segmentation → edge pixels where two
masks meet → ROI by shape extension → LOBB flatness → tanh activation → binary K-means →
Mean-Shift key points → polynomial fit. Reported: **max error < 1,2 mm, RMSE < 0,7 mm**.

**The coarse-stage correction.** Until this was checked, one `seam_region_oracle` was serving
every literature method. The three papers do not have the same coarse stage, and conflating
them flatters some and starves others:

| method | its coarse stage produces | supplied here from |
|---|---|---|
| `lit-ransac` | a **weld-seam band**, annotated at ~40 mm width (PointNet++) | truth seams |
| `lit-regiongrow` | one mask per **surface** (FastSAM), seam region *derived* from where two meet | `face_id` |
| `lit-lobb` | one mask per **component** (K-Net, labels "A"/"B") | `object_id` |

Measured, the first two are not close: on a T joint the seam band is 94 k points, **100%**
within 20 mm of a seam; the surface-derived crop is 331 k points, **31%**. The paper's own
coarse stage keeps 3–6× more and three-quarters of it is nowhere near a seam — because
**FastSAM segments surfaces and a plate has six of them**, so every plate rim is a surface
junction. The face-versus-part problem is in their *coarse* stage too, not only downstream.

**And `lit-lobb`'s coarse stage is `ours`' cross-object gate.** RCIM eq. 2 declares a pixel an
edge when its neighbourhood holds two different **component** masks — which is exactly what
`cross_object_mask` does with `object_id`, obtained from a K-Net at 97,35% mIoU instead of
from a stack of registered CAD clouds. This plan calls that gate *a dependency on
segmentation `ours` does not publish about*. Here is the literature taking the same
dependency and publishing it. **That materially strengthens the `ours` write-up**: the
dependency is a property of the problem, not a weakness of one method, and the honest framing
is a *ladder* every method sits on rather than an accusation aimed at ours.

**Measured on the balanced corpus, 25 scenes per joint type, both arms:**

| arm | joint | F1 | median RMSE | crease pts within 3 mm |
|---|---|---|---|---|
| L0 | **corner** | **0,83** | **0,51 mm** | 82% |
| L0 | T | 0,47 | 2,82 mm | 76% |
| L0 | butt | 0,33 | 3,14 mm | 64% |
| L0 | lap | 0,04 | 28,5 mm | 52% |
| L0 | edge | 0,02 | 64,7 mm | 3% |
| L1 | corner | **0,02** | 80,1 mm | 10% |
| L1 | T | 0,09 | 48,1 mm | 20% |

**Corner reproduces the paper**: median RMSE 0,51 mm against their reported < 0,7 mm, first
run, no tuning — the strongest reproduction any of the three has produced. 18 of 125 L0 scenes
meet < 0,7 mm outright.

**And the L0 → L1 collapse is `ours`' collapse, on the same input.** Corner goes 0,83 → 0,02
when the component masks are withheld; `ours` goes 0,75 → 0,03. Two methods sharing nothing
but that one input, failing by the same factor when it is removed. That is as direct a
confirmation as this dataset can produce that **the dependency is a property of the problem,
not a weakness of `ours`** — and it is the single most useful thing `lit-lobb` contributes to
the thesis.

On a T joint it returns **F1 0,00**, and the reason is worth recording carefully because the
detection is not what fails. **98% of its crease points land within 3 mm of truth.** What
fails is cutting them into seams: a web sitting on a base plate touches it along a **closed
perimeter** — two long fillets joined by two short cross-runs at the ends. D4 excludes the
cross-runs, so the *label* is two open curves while the *geometry* is one loop. LOBB finds
the loop almost perfectly and then fits one polynomial through it.

Proximity cannot make that cut. The cross-runs are only **2,5%** of the crease points and all
of them sit within 15 mm of a seam endpoint, but they physically bridge the two fillets, so
**no link distance separates them** — measured at 1,5 / 2 / 3 / 5 mm, all give one component.

That is now **three independent methods stopped by the same missing primitive**, reached by
three different routes:

| method | what it cannot do |
|---|---|
| `ours` | separate two parallel centrelines a plate-thickness apart |
| `lit-regiongrow` | hold a seam together without merging both fillets |
| `lit-lobb` | cut a closed contact perimeter into the open runs that are welded |

All three are the same request: **split a point set by direction, not by proximity.** Build
it once, measure it on all three, and report it as a finding about the mechanism. Neither of
the three papers could have found this — their workpieces have well-separated seams and no
closed perimeters.

#### `ours`: the validity window was wrong, and direction-aware clustering is a partial win

Two results, from following the clustering question to its end. Both concern **`ours` only**
— see the fidelity note below.

**1. The upper bound of the validity window is `t/2`, not `t`.** `README §8` states the
claim as `gap + spacing < R < thickness`, and the derivation does not support the right-hand
side: what bridges a plate's own two faces is the ball's **diameter**, not its radius. At `R`
just under `t` the neighbourhood is nearly `2t` across, so a point on the top face has the
bottom face inside it — the exact failure the bound exists to prevent. Measured, a "valid"
`R` under the old bound gave a ball spanning **1,2–1,9× the plate**.

Paired on 44 T / corner / butt / lap scenes where both bounds leave the window open, changing
nothing else:

| | `R < t` | `R < t/2` | improved in |
|---|---|---|---|
| F1 | 0,709 | **0,897** | 38 / 44 |
| precision | 0,583 | **0,839** | 39 / 44 |
| band width | 5,70 mm | **3,31 mm** | 44 / 44 |
| Chamfer | 5,05 mm | **3,68 mm** | 42 / 44 |

This is the *corrected bound, derived and then measured* that the Phase 4 record asked for
after the sweep found the usable range wider than predicted. It closes the window more often
(44 of 70 runs against 70 of 70) — a cost of being right, since those runs were only "valid"
because the bound was wrong. `validity_window_mm(..., upper="thickness")` keeps the original
claim reachable so the correction stays falsifiable.

**2. Direction-aware clustering: real, and not the fix that was predicted.** The recommendation
was that it would resolve the seam-separation failure shared by three methods. It does not.

| | exact seam count | mean abs. count error |
|---|---|---|
| `R < t`, proximity | 2,3% | 1,00 |
| `R < t`, **directional** | **34,1%** | 1,14 |
| `R < t/2`, proximity | 6,8% | 1,07 |
| `R < t/2`, **directional** | 15,9% | 2,39 |

It gets the count *exactly* right 15× more often at the **old** window, and **over-fragments
when it is wrong**. Re-measured in `notebooks/03` on 50 scenes across all five types at the
*corrected* window default: exact 8% → 10%, absolute count error 2,3 → 4,4 — the gain is gone
and the fragmentation is not, worst on edge, whose band has no direction structure at all.
**The window correction absorbed most of what the clustering upgrade was worth**, which is the
right outcome: the fat, bridged band was the disease and direction-aware linking was treating
a symptom. Off by default, revisit only if a fat-band regime returns.

Two mechanisms had been conflated and the measurement separated them. Comparing the two
points' **tangents** to each other cannot work: a tangent smoothed over its own ball blends
straight through a sharp corner, and two *parallel* seams have identical tangents by
construction. Testing whether the **step between them runs along both tangents** cuts both —
a step onto a cross-run is perpendicular to the fillet it leaves, and a step across to a
parallel fillet is perpendicular to both.

And the deeper reason `ours` was never going to be fixed by clustering: **the band is one
continuous region.** Measured on T joints under the old bound, 12–19% of band points sit in
the middle third *between* the two fillets. There is no gap for any clustering algorithm to
find. That is a property of the band, not of the clustering — and the window correction above
is what actually narrows it (5,70 mm → 3,31 mm).

**Fidelity note — this is not applied to any `lit-*` method, deliberately.** Neither Wei et
al. §III-D nor Zhang et al. §3.4.3 specifies a clustering step at all; multi-seam clustering
is this repo's addition in both (`lit_regiongrow` deviation 5, `lit_lobb` deviation 4).
Filling an unspecified step with the standard choice is faithful; filling it with something
better than the paper suggests would make the reimplementation **outperform the published
method** — the same fidelity failure as making it worse, in the flattering direction — and
would leave every number unattributable. `ours` gets the upgrade because it is ours. A test
asserts the `lit-*` modules do not even import it. If the question is *how much of each
method's error is clustering*, run the better splitter as a clearly labelled **diagnostic
arm** across all seven and never quote it as a paper's result.

#### HPR became the condition, not a gate — and then the condition became analytic

Implemented 2026-08-21 exactly as the plan directs: **no method gained an exteriority
gate.** The generator computes a per-point `exterior` flag once, at generation time, stored
in `cloud.npz` beside `visible_from_cam` (SCHEMA §5.1), and the harness's `view=` axis
carries the three conditions:

| condition | input | meaning |
|---|---|---|
| `full_exterior` | `exterior == True` | perfect multi-view scan — **Task 1** (the default) |
| `single_view` | `visible_from_cam == True` | one shot — **Task 2** |
| full geometry | — | truth only, **never a method input**; kept solely to reproduce the literature's own CAD-cloud evaluation condition |

**One correction to the plan's own sketch, made by measurement: the stored flag is
analytic, not HPR.** Single-parameter HPR cannot draw the line the definition names on
plate assemblies — at `radius_factor` 100 its conservative hull deletes **55% of the
concave fillet-root corridor**; at 1 000 the corridor survives but **38% of the lap
interface is sighted through its ~1 mm slit**; at 10 000 the occlusion test keeps
everything; and gating the middle setting by incidence still leaked 93%, because the
hull's per-view visibility is itself dishonest at that radius. The quantity the definition
actually names is **reachability under a sensor's grazing limit**, and at generation time
that is exact: a point is exterior iff some ray within the grazing cone (70°, the same
quantity as the noise model's `grazing_dropout_deg`) of its outward normal escapes every
slab — `visibility.exterior_scan`, built on the existing `ray_hits_slab`, 0,1 s per scene,
zero tuning parameters beyond the sensor's own limit. Validated: buried lap interface
kept **0,000**, open-face seam corridor kept **0,98–1,00**, groove walls graded by depth
against gap exactly as a scanner would see them, and `visible_from_cam ⇒ exterior` at
<0,5% tolerance. `hpr_exterior` remains in `visibility.py` for the runtime no-CAD context
it was written for, with its measured limitation now documented.

The corpus is backfilled (`out/bench`, content hashes and index updated in place;
`scratchpad/backfill_exterior.py` is deterministic and idempotent), and new generations
carry the flag natively.

**The predicted consequence, measured.** `ours` on lap, full geometry → `full_exterior`:
precision **0,53 → 0,68**, F1 **0,61 → 0,77**, band width **5,8 → 3,1 mm**, recall held
(0,999 → 0,975). The mid-lap phantoms leave with the interface that produced them —
*"correct, not a loss"*, as the advisor text says: they were an artifact of feeding CAD
clouds. What remains of the lap failure is the real one, no longer confounded.

#### The nine "first real results" plots — produced

All nine, on the balanced corpus for `ours`, in `notebooks/11_first_results.ipynb`: error
vs root gap (1), vs thickness with the window-closing marker (2 — plus the corrected `t/2`
bound in `nb03`), vs `occluded_fraction` (3 — the plot only constructed truth can draw),
vs joint type under both conditions (4), vs controlled density (5), vs included angle (6),
**fixture on/off on paired seeds** (7 — a fixture twin corpus now exists at
`out/bench_fx`, same configs and seed ranges with `fixture_present: true`, paired on
`twin_key`), vs sensor profile on the noisy single-view arm (8), and the **D19 conversion
table as a function of g** (9 — the same detection scored against `nominal` / `root` /
`gap_mid`, spread growing with the gap as D19 predicts). The seven-method versions are the
same groupbys over the full batch `run_matrix` output — the remaining Phase 4 compute job,
now packaged as **`scripts/run_phase4_batch.py`**: 44 resumable chunks (~10 h) covering the
seven-method coverage table at 50/type under both conditions, every L1 arm, the noise table
at σ×{1, 2} on the single view, the ladder extras (`lit-ppf` exact normals, `lit-modelreg`
dense-features and global-init arms), and the fixture twins for all seven methods.
`--list` prints the chunk plan with estimates; one `csv.gz` per chunk, skip-if-exists, so a
crash costs only the chunk in flight; `phase4_batch.csv.gz` is the concatenation every plot
reads.

Three headlines from the executed run (all 250 scenes, `ours`):

- **Plot 7 is the strongest single figure the project now owns.** Fixture on vs off, 40
  twin-paired seeds: precision **0,74–0,90 → 0,13–0,24**, band width **2–4 mm → 73–191 mm**,
  recall held. The D12 prediction — that a large fraction of published seam-extraction
  performance is an artifact of pre-isolated workpieces — now has a number: one fixture
  plane in the scene costs a curvature method ~4–6× of its precision, because nothing in a
  point-cloud-only method knows `role`.
- **Both conditions, both tasks, side by side** (the confirmed checklist item): under
  `full_exterior`, lap holds the phantom correction at corpus scale (F1 0,77, precision
  0,68 over 50 scenes) while **edge stays at 0,29** — confirming the advisor text's parting
  sentence: the failure that survives the exterior condition is the real one; a flush edge
  is a boundary feature and curvature does not fire on it. Under `single_view` precision
  *rises* everywhere (0,80–0,89) while recall roughly halves — the camera removes
  false-positive surface faster than it removes seam.
- **The corrected window closes in 143 of 250 scenes** at ρ = 0,5 — most of the corpus is
  thin sheet by the method's own admission criterion: plot 2's method-class finding, stated
  at corpus scale.
- **Plot 9, quantified:** the D19 curve-choice spread grows 0,03 → 0,15 → 0,42 → **0,98 mm**
  across the gap bins (0–0,5 / 0,5–1 / 1–2 / 2–5 mm). At the largest gaps the definitional
  crack is the size of the method differences the field publishes.

#### The reproducibility result, and why it is the strongest thing here

The advisor asked for box plots because RANSAC is randomised. The size of the effect is larger
than that framing suggests. Same scene, same parameters, only `seed` varying, 30 repeats:

| scene | F1 min | median | max | zero runs |
|---|---|---|---|---|
| `…412392` (butt, gap 1,82) | 0,00 | 0,94 | 0,96 | 2 / 30 |
| `…412402` (butt, gap 0,24) | 0,00 | 0,00 | 0,96 | 20 / 30 |
| `…412339` (butt, gap 1,95) | 0,00 | 0,00 | 0,00 | 30 / 30 |

The middle row is a coin flip between a near-perfect answer and nothing, on one fixed input.
The mechanism is greedy plane removal: whether the gap wall is ever fitted depends on which
three points were drawn first, and the plane count itself varies (2, 3, 4 or 5 on one scene).

Consequences:

- **Every per-scene number for `lit-ransac` and `lit-ppf` above is meaningless as a single
  value.** The table in this section is one draw and is labelled as such. Do not quote it.
- The repeat harness is not a protocol nicety, it is a prerequisite. Build it before any
  further `lit-*` method.
- The bottom row is the useful control: a scene that fails at every seed is a *structural*
  failure and separable from a sampling failure. That separation is a metric the field has no
  way to compute, because it needs many scenes of known truth.

### The exteriority test without CAD

The `README §8` blob problem is **partly an artifact of the SEPC being full CAD clouds baked at
poses** — 360° geometry including faces no camera can ever see. A real single-view cloud
contains only exterior surface by construction, so the mid-lap interior candidate is not merely
wrong, it is *absent*.

So the test is only needed on the full-geometry variant. Use **hidden point removal**
(Katz et al.): ~30 viewpoints on a sphere, union the results, mark every point visible from at
least one direction as exterior. A candidate seam is then weldable if it has exterior support on
both parts and the dihedral bisector escapes without re-entering material.

This matters because at runtime there is no CAD registration — the test must be
point-cloud-only.

### On the CAD dependency, stated fairly

An earlier draft called `ours`' cross-object gate an unavailable oracle. That was too strong,
and the correction matters for how the ladder is read: in the deployed pipeline `object_id` **is**
available at runtime, because the SEPC is a stack of per-object CAD clouds placed at poses from
FoundationPose/ICP. It is a **dependency on object-level segmentation**, not a cheat.

The dependency still has costs worth measuring, and the dataset can measure all three: you need
CAD models of the parts; registration has to succeed; and **registration error propagates
straight into the seam**. That last one is a plot nothing else can produce — perturb the object
poses by a known amount and watch the seam error move.

Report two arms rather than replacing one with the other:

| arm | segmentation from | answers |
|---|---|---|
| CAD-assisted | registered CAD (the deployed pipeline) | how the deployed method actually does, plus sensitivity to registration error |
| cloud-only | nothing | does the method generalise to parts with no model |

### Curved seams

Plane-pair generalises to **surface-pair intersection**: plane ∩ cylinder for pipe-on-plate,
quadric ∩ plane for a dished end. Same code shape, different primitive fitter. `lit-pcaslice` is
the third family and is most interesting in Phase 6, since slicing is aimed at curved seams.

**Where each method stops working is a result, not a bug.** Report it.

---

## 5. Schema (Phase 0 deliverable — freeze before coding)

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

## 6. Parameter ranges (Phase 0 deliverable)

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
1–2 mm is exactly the stainless in your lab and exactly where §6 predicts radius-PCA has no valid
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

### 6.1 Sensor profiles (D16)

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

## 7. Phases

Effort estimates assume focused days, not calendar days.

---

### Phase 0 — Freeze schema and parameters

**Do not skip. Do not start coding first.**

- [x] Write `SCHEMA.md` (§5) and `PARAMETERS.md` (§6)
- [x] Decide directory layout for a scene and for a release — settled in Phase 1 and
      stable since: `out/<corpus>/<scene_id>/` with `scene.json` + `.npz` payloads,
      corpora as per-joint-type directories with an `index.jsonl` each (`writer.py`,
      SCHEMA.md §5); confirmed ticked 2026-09-01
- [x] Decide file formats — realised as `.npz` (compressed, multi-array) rather than bare
      `.npy`, `.json` per scene, `index.jsonl` per corpus directory, `.ply` only where a
      mesh export is explicitly requested (`--emit-meshes`, Phase 5 annotation export);
      confirmed ticked 2026-09-01
- [x] Write down the naming convention for `face_pair` strings — SCHEMA.md's faceRef
      registry, `<part>:<face>` with `±u/±v/±w`, extended stably ever since (prism `s<i>`,
      tube `lateral±`, prepared-slab `root/fusion/radius` — the schema pattern is the
      contract); confirmed ticked 2026-09-01

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
      reachability instead of an arbitrary threshold, which gives the §8 weldable-vs-interior metric
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

- [x] **Camera pose sampler:** spherical shell, standoff from the profile's valid range, elevation
      15–85° from the world XY plane, azimuth free, roll ±15°. **Where it aims turned out to be the
      decision that mattered:** the assembly centroid is dragged upward by the standing plate, so the
      camera looks over the seam; aiming exactly at the seam pins it to the image centre and leaks
      the answer through the pose, the same leak `SCHEMA.md` §1.1 avoids by not pinning the assembly
      to the world origin. Resolved the same way — aim at the joint, miss by ±0,15 of the longest
      part edge
- [x] **Ray-cast visibility** → per-point `visible_from_cam`. Neither `trimesh.ray` nor open3d in the
      end: `trimesh.ray` needs `rtree`, which D9 rules out, and every part is a box, so ray–box
      intersection is exact in closed form and vectorises over the whole cloud. Phase 6's swept and
      revolved primitives are where a real ray engine earns itself
- [x] **Analytic depth noise:** σ_z = subpixel·z²/(f·b) along the view ray, lateral blur in pixels,
      grazing dropout past 75°. The realisation is **not stored** (`SCHEMA.md` §5.1) — `noise.apply`
      is deterministic in `noise_model.seed`, which makes the noise a citable convention rather than
      a baked artefact
- [x] **Per-seam `occluded_fraction`** — and the split that makes it meaningful: the mask is
      *geometry only* (framed, in range, front-facing, unoccluded). Sensor dropout lives in
      `noise.apply` with its own validity mask, so occlusion stays comparable across the three
      sensor profiles instead of being confounded with them
- [x] **HPR (Hidden Point Removal) exteriority** utility (shared with Phase 4 `ours`). scipy's
      convex hull, no open3d. On a lap joint it marks the buried interface 1,3% exterior and the same
      face outside the overlap 100% exterior
- [x] **Pin `camera_raster` mask semantics (D20)** — pinned and implemented. Both classes come from
      **one** dense area-uniform pass split on the mask, which is what makes them rate-matched; two
      independent draws would have to reconcile two densities that were never equal. Raster density
      is `(f_px/z)²`. `SCHEMA.md` §7 now reads "implemented", not "reserved slot"

**Gate — met.** Measured over primary seams, 80 seeds × 5 joint types. It took splitting the
metric to see it, which is the substance of the 2.3.0 schema bump:

| quantity | = 0 | strictly partial | = 1 |
|---|---|---|---|
| `occluded_fraction` — another part in the way | 40% | **1,7%** | 58% |
| `1 - in_frame_fraction` — image edge, blind zone | 69% | **23%** | 8% |

**Revised by D26 (2026-08-20).** This gate encoded an assumption the advisor meeting
overturned: that a polite sampler is a defect. Under coarse positioning it is the *correct*
model of deployment. The `uniform_sphere` regime below stays as the stress test and keeps this
gate; `approach_cone` is added alongside it, and the delta between them on paired seeds
measures what coarse positioning is worth. The `NoVisibleSeams` yield problem largely
disappears under `approach_cone`, because the camera is drawn from a region where the seam is
observable by construction.

**Occlusion is binary and always will be.** A straight seam under a convex occluder is
shadowed all-or-nothing, because the occluding plate spans the run the two parts share to
begin with. Sampling harder will not change it; partial *occlusion* needs a non-convex
occluder, a third body, or a curved seam (Phase 6, Phase 7).

**Framing is graded, and that was a real bug in the sampler.** Standoff was drawn uniformly
over 300–1200 mm, which framed the assembly comfortably almost every time — so
`occluded_fraction` was `{0, 1}` in **99,3%** of seams and **Phase 4 plot 3 (error vs
visibility) would have been two points, not a curve.** One of the two figures the plan claims
nobody else can produce could not have been produced here either. Standoff is now derived from
a sampled *framing fraction* (0,35–1,45 of the short image side), so the assembly overflows the
frame on purpose. Among seams not occluded by another part, 22% now have a `visible_fraction`
strictly inside (0,05–0,95), populated across every bin. The framing sampler is also the more
realistic one: an eye-in-hand camera frequently cannot get a whole 400 mm seam into one view.

Findings worth carrying into the paper, measured on canonical geometry (24 azimuths × 4
elevations, standoff 700 mm, aimed at the seam midpoint, "usable" = at least half returned):

- **Azimuth dominates, and elevation barely matters.** Each fillet of a T-joint is visible from
  ~46% of viewpoints — its own side — and the two lobes are complementary, so at least one is
  usable from 92%. The 8% that fail are a narrow band where the camera looks *along* the seam
  axis (within ~10° of azimuth 0° or 180°), where the standing plate's near end occludes the
  whole crease. That holds at 25° elevation and at 85° alike. An earlier reading of this as
  "both fillets are hidden from overhead" was wrong: the case that produced it was at azimuth
  0, and elevation had nothing to do with it.
- **The lower toe of a lap joint is visible from 0% of viewpoints** above the table —
  structurally, not statistically. Only the upper toe is usable, from 44%, which is why lap
  scenes survive the omission policy least often. Phase 4 must exclude it from recall on
  single-view metrics, or a baseline is scored for missing something no sensor could see; the
  per-seam `visible_fraction` is what to filter on.
- **A butt joint's top centreline is visible from 100%** of viewpoints: it lies on the upper
  surface with nothing standing near it. Its underside centreline manages 21%. Since dissimilar
  thickness leaves *only* the flush underside centreline, thickness mismatch and visibility
  interact — worth knowing before reading a per-class result.

**Tier-1 omission policy.** A scene with no primary seam the sensor returns carries no
supervision, and is **omitted rather than relabelled**: a "no seam" class would encode joint
type and camera placement rather than anything about the task, so a model would learn to
predict it from the wrong evidence. `min_visible_fraction` is deliberately low (0,1) — a high
bar would eat the partially-framed band, which is the graded middle the axis depends on.

Yield is **≈ 52%**, and **not uniform across joint types**: T 79%, butt 57%, corner 44%,
edge 42%, lap 36%. So `--n` is an attempt count, not a scene count, and a config requesting a
uniform joint mix does not emit one. **Skipped seeds are written to `index.jsonl`** with
`emitted: false` and a reason, so the selection is characterisable rather than merely reported
— `df[~df.emitted]` recovers exactly what was dropped. The policy is a config key, so a
robotics-facing run that wants the untruncated visibility axis sets `require_visible_seam:
false` and gets every scene.

**Effort:** 2–3 days.

---

### Phase 4 — Baselines against truth

Now the PCA fix happens, **with a number in front of you** instead of RViz eyeballing.

**Step 1 done — `ours` migrated and measured (2026-08-18).** `scripts/baselines/`:
`radius_pca.py` (the method), `metrics.py` (Chamfer, P/R/F1, lateral, band width),
`dataset.py` (the harness), `notebooks/03_baselines.ipynb`. Not tuned, not fixed — the point
of this step was to get numbers to argue with. 44 scenes, both views, `R` taken from the
midpoint of the predicted window, density controlled at 0,5 pts/mm².

- [x] `ours`: multi-seam connected components. **The per-component line fit was tried and
      removed** — on these joints the band is a rectangle, and a total-least-squares line
      through a rectangle lands in its middle, which is the mid-surface between two plates
      rather than the seam. `detect` now returns the band and its clusters, scored as a point
      set. Several attempts to replace the estimator (ridge-following, crease projection,
      direction-gated linking, DBSCAN) each won on the case they targeted and lost more
      elsewhere: corpus F1 went 0,64 → 0,16 and 0,64 → 0,43. Kept in
      `scratchpad/radius_pca_experimental.py`
- [x] `ours`: ~~HPR exteriority gate~~ — **superseded 2026-08-21**: exteriority became the
      **condition** (`full_exterior`, stored per point at generation time, identical for
      every method — see §4's HPR entry). `detect(..., exterior=...)` remains wired but no
      longer has a role; no method carries its own gate

**First numbers, and they are not flattering.** Median by joint type, full / single view:

| | Chamfer full | F1 full | Chamfer single | F1 single |
|---|---|---|---|---|
| corner | 1,2 mm | **1,00** | 3,4 mm | 0,84 |
| T | 5,8 mm | 0,66 | 4,4 mm | 0,94 |
| butt | 7,8 mm | **0,00** | 3,0 mm | 0,96 |
| lap | 21,8 mm | **0,00** | 11,8 mm | 0,88 |
| edge | 69,6 mm | 0,21 | 109,0 mm | **0,00** |

Four things fall out of this, all of which the plan predicted in outline and none of which
had a number before:

1. **The cross-object gate is doing nearly all the work, and it is an oracle.** Same scenes,
   single view, gate on vs off: Chamfer **4,2 mm → 56,7 mm**, F1 **0,75 → 0,03**. Per-point
   object membership comes from the registered CAD assembly and no sensor provides it, so
   the honest point-cloud-only number for this method is the second column. This is a
   sharper version of the §4 claim than the plan had, and it belongs in the paper: the
   published method's performance is substantially CAD registration's, not PCA's.
2. **Lap and edge are where it dies**, exactly as the plan expected — and edge is worse than
   lap, at 70–109 mm error. Two coplanar faces have no dihedral for a variance ratio to
   detect: the method has no signal there, rather than a weak one.
3. **Single view often beats full visibility** (F1 0,86 vs 0,64 overall; butt and lap go
   0,00 → 0,96 / 0,88). Consistent with §4's argument that the buried mid-lap interface is
   *absent by construction* from a single-view cloud rather than merely wrong — the hardest
   false positive is one the camera never delivers. Worth confirming directly, since it also
   means full-visibility numbers in the literature are measuring a harder problem than the
   one a robot faces.
4. **The validity window is empty in 16 of 88 runs** at ρ = 0,5, and where it is open the
   usable-F1 range is *wider* than predicted (2,5–11,9 mm against 3,4–6,6 mm on the scene
   swept). The bound may need restating rather than confirming — a corrected bound, derived
   and then measured, is the stronger result.

Two engineering notes carried forward: component linking is the most sensitive knob and
proximity alone cannot separate two parallel centrelines a plate-thickness apart (direction
splitting is the fix); and `surface_variation` was rebatched to ~100× the original speed
(bit-identical, pinned by test) because a corpus sweep otherwise does not finish.
- [x] **`lit-regiongrow`** — Wei et al. arXiv:2408.10710 §III-D, `scripts/baselines/
      lit_regiongrow.py`, 8 tests. Implemented and first-measured; see §4. Its curvature is
      **the same quantity `ours` uses**, so the k-NN-vs-radius substitution and the
      oracle-free two-surface test are now both measurable on one code path
- [x] **`lit-lobb`** — Zhang et al. RCIM 2025 + T-ASE 2025, `scripts/baselines/lit_lobb.py`,
      11 tests. Implemented; first measurement in §4. **Its coarse stage is `ours`'
      cross-object gate**, obtained from a trained 2D segmenter instead of a CAD stack
- [x] **`lit-ransac`** — Yi et al. 2026 §5–§6, `scripts/baselines/lit_ransac.py`, 15 tests.
      Measured; see §4. Edge 0,00 as predicted, butt **not** 0,00 (root gap ⇒ orthogonal wall),
      lap 0,00 without the segmentation oracle because `T_mpp` is an area ratio
- [x] **Repeat harness — BUILT 2026-08-21.** `scripts/baselines/harness.py`, 7 tests,
      `notebooks/07_repeats.ipynb`. One dataframe over method × scene × seed × arm × view ×
      noise_scale. Three commitments baked in: each method's **L0 is its own paper's coarse
      stage** (the shared-mask mistake is structurally excluded); deterministic methods run
      `verify_seeds` times so *zero spread is measured, not assumed*; the noise axis is a
      **multiplier on the derived σ_z** (scales `subpixel_px`/`lateral_sigma_px` in a copy of
      the sensor model — "200%" can never mean a fraction of range). Validated against a
      **fake oracle** (truth + known jitter / phantoms / misses) with closed-form
      expectations before any real method's number flows through, as the order of work
      requires — and that validation caught two real bugs before any method did: a fake
      whose densified mass hid a phantom at 1% precision cost, and a resampler that could
      not coarsen
- [x] **EMD secondary metric** — `metrics.emd_mm`, exact assignment on 256-pt resamples (the
      advisor's "transport cost"). It is the metric that *sees* the coverage failure Chamfer
      forgives: a perfectly-placed 10% stub scores Chamfer ≈ its offset but EMD ≈ L/2. Off
      by default (O(n³)); a table column, not an inner loop
- [x] **`lit-ppf` — IMPLEMENTED 2026-08-21.** Wang et al., Sci. Rep. 14 (2024) 21137;
      `scripts/baselines/lit_ppf.py`, 9 tests, `notebooks/08_lit_ppf.ipynb`. Ran through
      `run_matrix` from its first execution, as committed. Three corrections to what this
      plan assumed about it:
      1. **It is deterministic as published** — grid sampling, Hough voting, DBSCAN,
         farthest-pair corners; no stage draws a random number. The "randomised" grouping
         came from its RANSAC-alternative *framing*, not its method. Registry carries
         `randomised=False`; zero seed-spread measured on real scenes.
      2. **The paper contradicts itself about RANSAC**: the prose proposes PPF instead of
         RANSAC; its own Algorithm 1 says the C++ implementation used RANSAC. The prose (the
         stated contribution) is what is implemented, and the contradiction is recorded.
      3. It consumes **normals** — first method that does. The estimate-vs-exact arms are in
         the harness (`method_kw`); measured price of estimation: F1 0,91 → 0,59 (butt),
         1,00 → 0,44 (edge), 0,66 → 0,41 (T), and normal estimation dominates runtime 4-7×.
- [x] **`lit-ppf` first measurement, 15 scenes/type through the harness.** L0/exact-normals
      F1: butt **0,91**, edge **1,00**, corner 0,67, T 0,66, lap 0,49. L1 collapses to
      precision ~0,1 with 23–36 predicted seams — the universal plate-border phantom. And
      the headline: **the coverage prediction refines rather than holds.** Butt and edge
      are NOT zero — butt through the root-gap walls (`lit-ransac`'s loophole), edge through
      the merged flush-plane × big-face corner (survives single view at F1 0,91, so not a
      hidden-face artifact). But the proxy's error is **O(root gap)** — largest-gap edge
      scenes read RMSE ≈ g/2–g; butt RMSE correlates with gap at 0,55 and with thickness at
      0,05. So `lit-ppf` expresses coplanar seams *only as gap-conditioned proxies*:
      invisible at this corpus's gaps (≤ 2,8 mm vs 3 mm tolerance), unbounded in principle,
      degrading silently — a worse failure mode than returning nothing, and measurable only
      because the gap is a sampled axis. Caveat: all corpus edge plates are ~1,5 mm thin;
      no thick-plate edge test exists yet
- [x] **`lit-pcaslice` — IMPLEMENTED 2026-08-21.** Wang et al., *Welding in the World*
      (2026); `scripts/baselines/lit_pcaslice.py`, 6 tests, `notebooks/09_lit_pcaslice.ipynb`.
      Its coarse stage is **per-instance** (YOLO11 boxes each weld separately), and that is
      structural, not convenience: the per-slice *geometric centre* of a strip holding two
      seams is the midpoint of neither — `ours`' mid-surface failure, reached through a
      different mechanism. So the harness L0 supplies one band per truth seam, and L1
      (whole cloud, one instance) is a broken-assumption arm by construction. Path pipeline
      deterministic; MSAC feeds only the (unscored) torch posture. First numbers: with the
      per-instance oracle it is near-ceiling on straight seams (F1 0,95–1,0) — as the plan
      predicted, it earns its seat at Phase 6, where seams curve. **Its YOLO stage is worth
      naming in the comparison: it is instance segmentation of the seam-separation problem
      itself — the exact problem `ours`, `lit-regiongrow` and `lit-lobb` are stopped by —
      solved in 2D before the cloud is touched**
- [x] **`lit-modelreg` — IMPLEMENTED 2026-08-21, and the open item is resolved.** Fang &
      Tian, *RCIM* 89 (2024) 102772; `scripts/baselines/lit_modelreg.py`, 5 tests,
      `notebooks/10_lit_modelreg.ipynb`. The open item asked whether it is implementable
      without the original CAD assets: **yes — `scene.json` (dims, `T_world_part`,
      `T_world_joint`) is a sufficient CAD source**, verified by a test that overlays the
      rebuilt model on the scan and roundtrips the seam to 1e-9 — **and it is still
      constitutively L0-with-CAD**, because the model's seam IS the stored truth. Every
      number it emits carries that label; it anchors the top of the oracle ladder with a
      real published method. Classical CPD stands in for their Bayesian CPD (same eq.-3
      mechanism; the priors are a robustness refinement), and three findings came from
      building it:
      1. **CPD's σ² must start from nearest-neighbour residuals**, not the all-pairs mean —
         the textbook init made the first E-step uniform and collapsed the scale to 0,14
         from a start already within 2,7°.
      2. **The paper's edge features are load-bearing.** Registering dense surfaces lets a
         lap stack slide ~5 mm along its overlap (faces dominate the correspondence mass;
         only plate ends resist). Their W is the workpiece's *edges*, and both sides of
         their registration are model-derived ("identical workpieces") — a stage a
         scan-only pipeline lacks, supplied here as an oracle like every other method's
         learned stage, with `target_features="dense"` as the withheld arm.
      3. **Outside the roughly-positioned envelope, near-symmetric assemblies register onto
         their symmetric counterpart** — the seam lands a plate-length away (119 mm
         measured on an edge joint). The paper's workstation setting never faces this, and
         the same coarse-positioning assumption is already in this plan as D26.
      First numbers (15/type, `L0-oracle`, `init="near"`): T 1,00 / corner 1,00 /
      butt 0,98 / lap 0,99 at RMSE 0,5–1,6 mm — the registration floor — with edge and one
      lap at ~5 mm: thin flush stacks whose coinciding edges leave a slide direction soft.
      **The residual IS registration error, which is the plot only this method can
      produce**
- [x] **CONFIRMED — Metrics:** Chamfer primary (in `evaluate`/`evaluate_band`, reported
      everywhere) and **EMD secondary** (`metrics.emd_mm`, exact assignment on 256-pt
      resamples — the advisor's "transport cost"). EMD earned its seat empirically: it is
      the only metric of the three that prices coverage gaps (two half-seams leaving a
      10 mm hole: Chamfer 0,91 — indistinguishable from a full seam — EMD 3,17). Demo and
      table in `notebooks/07_repeats.ipynb`
- [x] **CONFIRMED — both conditions evaluated separately and reported.** Now under the
      corrected condition names (see the HPR entry below): `full_exterior` (Task 1) and
      `single_view` (Task 2), side by side per joint type in
      `notebooks/11_first_results.ipynb`, with `view=` a first-class axis of
      `harness.run_matrix`

### Protocol additions from the 2026-08-20 meeting

**Repeats and box plots.** *"Random consensus RANSAC'ın... aynı algoritmayı 100 defa koşsan
birebir aynı şeyi elde etmeyeceksin... box plot'lar, minimum ve deviation'ı gösterecek
şekilde."* `lit-ransac` and `lit-ppf` are randomised, so every number for them is a
distribution: 100 repeats per condition, box plots with min and spread. The deterministic
methods show zero spread — **state that as a finding**, because method reproducibility is a
property this generator can measure and the field does not report.

**Noise sweep as a table axis.** *"Zero noise'da her birinin başarısı, %5 noise'da, %10
noise'da."* Report two ways: sensor profiles (`d435i` / `stereo_good` / `stereo_poor`) for
physical realism, and a scalar multiplier on σ as a comparability axis readers can map onto
other papers. **Define the percentage explicitly** — a multiple of the derived σ_z, not a
fraction of range.

**The oracle ladder.** L0 (xyz + `object_id` + exact normals, noiseless), L1 (no
segmentation), L2 (xyz only, normals estimated), L3 (L2 + noise + single view). Not an
artificial axis: `ours` depends on `object_id` through its cross-object check and
`lit-modelreg` depends on CAD, so two of the seven genuinely live on it. The **L0→L1 delta is
a number for how much each method depends on segmentation it does not publish about** — for
`ours` that delta is F1 0,75 → 0,03.

### Order of work

1. **Harness first** — matching, metrics, per-scene dataframe — validated against a fake
   oracle predictor (ground truth plus noise) before any real method output flows through it.
2. `ours` adapter to `cloud.npz`, run on `reference_tjoint`, compare against `seams.npz`
   nominal. *Done — see the step-1 record above.*
3. **Write down every input `ours` consumes.** That list defines the ladder levels.
4. `lit-regiongrow` *(done — see §4)*, then `lit-lobb`.
5. `lit-ransac` *(done — see §4)*, then the repeat harness *(done — `harness.py`,
   `notebooks/07`)*, then `lit-ppf` *(done — see the checklist above; deterministic as
   published, contra this plan's grouping)*. The order was forced: `lit-ransac`'s seed
   spread is 0,00–0,96 on a fixed scene, so a harness that reports one draw reports noise.
6. `lit-pcaslice`, `lit-modelreg` *(both done — see the checklist; all seven methods are
   now implemented, tested, and run through the repeat harness with their own papers'
   coarse stages)*.

**A corpus that can support these comparisons does not exist yet.** `out/phase3` holds 44
scenes with **2 lap and 4 edge**, and every per-type conclusion drawn from it so far has been
overturned by the next measurement. Before any method comparison: **50 scenes per joint type**
(confirmed 2026-08-20), one config per type in `configs/bench_*.yaml`, emitted to
`out/bench/<joint_type>/`.

Seed budgets are sized from the measured `out/phase3` yields and differ by an order of
magnitude across types — T ~75%, butt ~94%, corner ~69%, edge ~25%, **lap ~12%**. Seeds are
deliberately not backfilled (it would break twin pairing), so each budget is set generously
and the surplus is kept rather than discarded. **The yield spread is itself a result**: lap
and edge lose most seeds to `NoSeamsFound` / `NoVisibleSeams`, which says the omission policy
conditions the dataset unevenly across joint types and has to be reported with any per-type
number, not silently corrected for.

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
   `fixture_contact` false-positive rate separately — expect `lit-ransac` and `lit-ppf` (plane pairing) to emit a phantom candidate
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

**Toolchain built 2026-08-22** (`scripts/annotation/`, validated by a fake annotator of
known error in `tests/test_annotation.py` — injected 0,8 mm sigma, one planted miss, one
planted phantom, all recovered before any real click flows through):

- `export_for_annotation.py` — stratified seeded sample (default 4/type = 20 scenes),
  exported as anonymous ASCII PLYs of the **`full_exterior`** cloud (the annotator sees
  what a perfect scan sees — Task 1's own input). **No truth ships in the export
  directory**; the id↔scene manifest stays outside it. `BRIEFING.md` is generated into the
  export: the fifteen-minute brief, per-joint-type reference text, and (briefed arm only)
  the joint type per scene — the advisor protocol that constrains *selection* and isolates
  *localization*.
- The GUI is **CloudCompare itself**, deliberately: LWSNet's labels were drawn in
  CloudCompare, so the annotator must use the same tool or the measured floor is not the
  literature's. Protocol: `Tools > Point picking > Point list picking`, 4–10 ordered
  clicks per seam, saved as `scene_NN_seamK.txt`.
- `score_annotations.py` — scores clicks against analytic truth with a 15 mm match
  acceptance gate (the uncapped harness matcher absorbed a planted 60 mm phantom — a
  method's bad answer must be scored, an annotator's must be counted as a phantom).
  Outputs: **selection** (missed / extra, needs welding knowledge) separated from
  **localization** (lateral RMSE of accepted matches, needs none); the floor per joint
  type; **which D19 curve the human actually clicked** (nominal / root / gap-mid — a
  finding on its own); and `annotator_model.json`, the measured perturbation model the
  LWSNet training experiment needs.

**The contamination rule is enforced in code**, not by discipline: annotator roles are
`demo` / `briefed` / `unbriefed`, and `demo` — the generator's author showing the task —
is excluded from every headline aggregate automatically (their clicks measure
self-reproduction, not annotation). The agreed workflow: the author annotates as `demo` to
brief Anıl; **Anıl (briefed) is the measurement**; a second, unbriefed annotator, if one
materialises, prices the expertise term as (unbriefed − briefed). Per the advisor note,
the paper says "briefed engineering student, a reasonable proxy for whoever labelled
WeldJoint-PCD" — no credentials claimed.

- [x] Hand-label 20–30 tier-1 scenes in CloudCompare, the way LWSNet did — DONE
      2026-09-01: 20 scenes (4 per joint type, drawn from `out/bench6a` — the current
      shortcut-free plate geometry), annotated by the independent briefed annotator
      (`annotator1`) and, as the briefing record only, by the author (`demo`)
- [x] Measure annotation against analytic truth — DONE 2026-09-01, and **the headline
      floor is 1,4 mm**: briefed lateral RMSE median 1,415 mm, p95 11,8 mm, endpoint
      error median 4,2 mm; miss rate 0,20, extra rate 0,63
      (`out/annotation/annotation_scores.csv`, perturbation model in
      `out/annotation/annotator_model.json`)
- [x] **Get a second annotator if at all possible** — obtained: `annotator1` is the
      independent measurement, distinct from the author's excluded `demo` pass. Still
      open inside this item if the side-by-side figure is wanted: the **intra-rater
      repeat pass** (same annotator, second pass on a subset) that LWSNet-style
      consistency needs — consistency without accuracy is the literature's number;
      we can now show both, but only accuracy is measured so far
- [x] Compare the floor against the ~0.6 mm RMSE these papers report — **the floor is
      2,4× the reported accuracy**: labels drawn with the literature's own tool and
      protocol carry 1,4 mm median lateral noise against exact truth, so a sub-millimetre
      RMSE *measured against such labels* is below its own label noise. This is the
      Phase 5 claim, now a number.

**Findings from the scored pass (2026-09-01), all in `annotation_scores.csv`:**

- **Which D19 curve the annotator clicked depends on the joint type, exactly as the
  physical crease predicts** — butt: gap_mid fits best (1,31 mm vs 3,38 nominal — the
  human clicks the middle of the visible gap, not the face-plane intersection); T:
  root fits best (1,28 vs 1,98 — on a fillet the click lands in the root corner).
  Aggregated over types that structure survives (gapmid 1,16 < root 1,28 < nominal
  1,41), but the per-type table is the honest form. D19 storing all three curves is
  what makes this measurable at all.
- **Selection errors are structured by class, not random**: corner missed 4 seams (the
  second, opposed-approach seam of the pair goes unmarked), edge produced 11 extras
  (the flush stack invites marking every visible edge), lap both misses and extras.
  T was clean (0/0).
- **Lap is the catastrophic class for humans too**: 8,5 mm median lateral RMSE, and
  ~10 mm against *all three* D19 curves — the error is not "clicked a neighbouring
  reference curve", it is "picked the wrong line entirely" (plate edge for toe). The
  same class every Phase 4 baseline struggles with is the one humans cannot label.
- **Endpoint judgement is ~3× worse than lateral placement** (4,2 vs 1,4 mm median) —
  where a seam *ends* is genuinely harder to see than where it *runs*, which matters
  for any consumer scoring recall along the arclength.
- **The author's demo pass (1,92 mm) was WORSE than the briefed annotator (1,42 mm)** —
  knowing where the seams are does not make clicking more precise, which strengthens
  rather than undermines the exclusion rule: it measures a different thing, not a
  better one.

**Effort:** 1–2 days (plus recruiting the second annotator). *(Spent as planned;
annotation restarted once — the pre-6a export was retired with its corpus, see the
archive at `out/annotation_archived_20260901`.)*

---

### Phase 6 — split into 6a (orientation/outline) and 6b (curved seams, grooves)

*Integrated from `notes/patch_phase6.md` (2026-08-27, after Phase 5 self-annotation
returned 2,5 mm RMSE — the demo pass, contaminated toward optimism, and still 4× the
0,6 mm the literature reports against such labels). Two amendments made on integration:*

1. *6a's "no new primitives" is corrected to **"no curved primitives"** — polygon outlines
   require an extruded-polygon prism (planar-faced, but a new face registry, ray test,
   sampler and mesh). This gives 6a a natural internal order: **yaw first** (a yawed slab
   is still a slab; zero new primitives), **outlines second** (the prism).*
2. *The anti-shortcut gate is defined over **non-seam-bearing** boundary edges only — the
   upper part's contact edge IS the seam on T and lap by construction, and a gate that
   counts it can never pass.*

## D28 — Only the seam-bearing edge may be axis-aligned

Every seam generated so far runs **parallel to a plate boundary edge**, because parts are
rectangular slabs placed at orthogonal in-plane orientations. That is a shortcut, and it is
present in every number Phase 4 produces until it is removed:

- "find the long straight boundary, the seam is parallel to it" is a valid heuristic on the
  current dataset and will not generalise to any real workpiece
- `lit-ransac` and `lit-ppf` emit intersection lines parallel to genuine plate edges and
  cannot be penalised for confusing the two
- seam endpoints sit at predictable offsets from plate corners
- a trained network will learn the prior outright rather than learning seam geometry

The rule, stated once rather than as two special cases:

> **The seam-bearing edge is constrained by the joint type. Every other boundary edge of both
> parts must not be systematically parallel or perpendicular to the seam.**

Two mechanisms satisfy it, and which one applies is dictated by the joint type, not chosen:

| Joint | Mechanism | Why |
|---|---|---|
| T, lap | **in-plane yaw** of the upper part about the lower part's normal | the parts meet on a *face*, so the upper part can rotate freely within it |
| corner, butt, edge | **non-rectangular outlines** | the parts share an *edge*; rotating in-plane breaks contact, so diversity has to come from the other boundaries |

Yaw is not dihedral. Dihedral is the fold angle between the parts and is already sampled
(D18); yaw is rotation about the contact face normal, and is currently pinned to
{0°, 90°} by construction.

Outline vocabulary: `rectangle`, `trapezoid`, `parallelogram`, `triangle`, `rhombus`, and a
general convex polygon. For corner, butt and edge the only constraint is that both parts
present a matching seam edge; every other boundary splays. Non-90° in-plane corners are a
free by-product and are additional hard negatives — a 60° polygon corner is a concave-looking
feature that is not a seam.

Record this in `PARAMETERS.md` as an explicit **anti-shortcut measure**. "We deliberately broke
the axis-alignment prior" is a design choice reviewers credit when it is stated and penalise
when they discover it themselves.

---

## D29 — Curved seams are drawn from curve families that admit a two-part realization
*(amended 2026-08-28; originally framed artifact-first, which was the reverse of D3 —
the inversion the whole generator is named for. Same seven cases, same parameters,
renamed by what they produce rather than by what produces them.)*

"Arcs, C-shapes, S-shapes, closed curves" is shape-driven and unbounded — there is no
principled stopping point and no defence against "why not also helices?" The bound is a
**realizability condition**, stated directly rather than smuggled in via an artifact list:

> **The seam curve is drawn from families that admit a two-part realization.** A curve
> is realizable if it lies on the intersection of two constructible surfaces, or if it
> can serve as the sweep path of a plate profile.

An arbitrary 3-D spline is not realizable — no two simple solids meet along it. A
*planar* spline is, because a plate can be swept along it. So the vocabulary is finite
for a geometric reason, not a curated one:

| # | Curve family | Parameters drawn | Realization (derived FROM the curve) | Joint type |
|---|---|---|---|---|
| 1 | line | endpoints | plate placement (Phases 1–6a) | T, corner, butt, lap, edge |
| 2 | circle in the base plane | centre, radius | pipe ⊥ plate: radius = r, axis = plane normal | T |
| 3 | ellipse in the base plane | centre, semi-axes, orientation | tilted pipe: radius = semi-minor, tilt = arccos(b/a), axis in the normal–major plane | T |
| 4 | cylinder–cylinder saddle | `R, r, θ, offset` | exactly the two pipes — the family's parameters ARE the pipe parameters, and its closed-form parametrization is the D33 quadratic (a family evaluation, not a solver) | T |
| 5 | closed rounded rectangle | w, h, corner radius | rectangular tube: the curve IS the outer wall | T |
| 6 | planar arc / spline | control points / radius+span | swept plate: the curve IS the sweep path | T |
| 7 | planar arc | radius, span | curved butt: the curve IS the gap centreline | butt |

In every row the curve parameters are the artifact parameters — sampling "a pipe-on-plate
configuration" and sampling "a circle" are the same draw — so the equivalence to the old
configuration table is exact and the industrial motivation (nozzle welds, stiffener
perimeters, intersecting-pipe path planning) is unchanged. What the reframe buys:

- **Surface∩surface leaves the generation path.** The parts are derived from the drawn
  curve; the intersection machinery appears only in the D4 *verification* arms, where it
  independently rediscovers what construction placed — the same Phase 2 gate as always.
- **D3 holds unqualified across all phases**: one architectural principle instead of two
  that look like they disagree at Phase 6.

---

## D30 — Grooves are restricted to straight butt seams in Phase 6

Sweeping a groove profile along a curved seam requires the profile to stay perpendicular to a
rotating tangent frame, and self-intersects on tight radii. It is a genuinely hard modelling
problem and it is not what the phase is for.

Groove preparations apply to **straight** butt seams only. Curved grooves exist in industry
and are explicitly out of scope. Say so rather than leaving it implied.

---

# Phase 6a — Orientation and outline diversity

**Still planar slabs. Still straight seams. No new primitives.**

This phase exists because D28 is a *correctness fix for work in progress*, not an enhancement.
Every Phase 4 number computed before it lands is measured on a biased distribution, and the
bias flatters precisely the plane-based methods under comparison.

- [x] **In-plane yaw** for T and lap — IMPLEMENTED 2026-08-27. `JointSpec.in_plane_yaw_deg`,
      drawn from the previously unused `seam_curve` substream, so **every pre-6a corpus
      reproduces bit-identically and a yaw-enabled regeneration differs only in yaw** — a
      free ablation twin, pinned by test. The support bound resolved the patch's open item
      the third way: not full-360 and not a configured range, but per-scene — the chord of
      A's face along the yawed seam direction must keep ≥ 50% of `min(L_A, L_B)` (the same
      philosophy as the `length_offset` clamp; two earlier bounds — full footprint on A,
      clipped footprint corners on A — each pinned a quarter of scenes at 0° and were
      caught by the gate they were built to serve). Off by default (`in_plane_yaw: false`).
      AMENDED 2026-08-27: the range is the **full circle**, uniform over the per-scene
      feasible set of [−180°, 180°) (grid + jitter, `layouts.feasible_yaw_deg`) — θ and
      θ+180° are different configurations, so folding to ±90° halved the diversity for
      free; D31 separately rejects the flush-coincidence angles the full circle newly
      exposes (lap yaw ≈ 180° is always edge-flush)
- [x] **Update the D4 verification function** — and this was a *finding*, not housekeeping:
      the seam clip was a 1-D shadow (`face_extent_along`, corner projection onto the seam
      direction), which equals the true overlap at yaw 0 and overhangs it at any other yaw
      — the axis-aligned shortcut D28 describes, living inside the verifier itself. A
      13°-yawed T lost both fillets to a 4 mm "separation" measured at sample points off
      both plates. Replaced with the exact 2-D clip (`Slab.face_clip_line`), with the gap
      slack applied only to line-constant coordinates — both halves of that rule earned by
      measured failures (63 deleted toes; then seams overhanging by exactly the slack)
- [x] **Polygon outlines** — IMPLEMENTED 2026-08-27 as `geom.Prism`: a convex polygon in
      the part's u-v plane extruded along `w`, so **the w-is-thickness invariant survives
      by construction**. Duck-typed to the Slab face interface (`face_names`, `face_plane`,
      `face_clip_line`, `closest_on_face`, `face_extent_along`, `contains`, `mesh`), so
      accessibility, sampling, visibility and scene assembly all dispatch through the part
      and never ask "which of six faces". Two measured bugs on the way in: the parallel-branch
      clip slack compared an edge-length-scaled distance (a 3 mm offset read as 300 — every
      corner scene lost all its seams), and the coplanar run was taken from hull extents,
      which a sheared parallelogram overhangs at both ends (fixed by clipping the centreline
      against each face polygon with `gap/2` slack; pinned by test). Where the outlines
      apply resolved AGAINST the mechanism table's "dictated by joint type" and FOR the
      checklist's "all five": **B is outlined in every joint type** (its seam-bearing edge
      stays pinned straight and full-length by the layout mapping), because B co-rotates
      with the seam on T/lap and yaw can therefore never decorrelate B's own top/end edges
      — the corpus gate measured that residue directly. A stays rectangular for T/lap (yaw
      already decorrelates it, and the yaw support bound is defined on A's rectangle).
      Vocabulary `trapezoid / parallelogram / triangle / quad / convex-pentagon` at weights
      0.10/0.10/0.25/0.30/0.25 — deliberately non-uniform, because trapezoid and
      parallelogram keep a long far edge exactly parallel to the seam (their identity;
      D28's own vocabulary lists them) and a uniform draw put ~35% of free-edge length in
      the 0–10° bin. Drawn from `seam_curve` after the yaw draw: outline-off corpora
      reproduce bit-identically, outline-on is a free twin (pinned by test)
- [x] Update the D4 verification function — face enumeration generalised to
      `part.face_names()`; the coplanar in-plane basis (A's own axes for a slab)
      generalised to each face-polygon edge direction and its in-plane perpendicular
- [x] `faceRef` naming — prism caps keep the slab's `±w` names (so the `BROAD_FACES`
      seam-classification logic is untouched), sides are `s0..s{k-1}` with `s0` the
      seam edge by construction; schema pattern extended, `primitive: "prism"` +
      `outline_uv`/`outline_shape` added to `objects[]`, `part_geometry_id` =
      `prism_<shape>_<k>x<L>x<W>x<t>` (so 6a already makes the D11 split a genuine
      geometry split, as the patch hoped). Per-point `face_id` bases are cumulative,
      `== 6*i` for all-slab scenes — content hashes unchanged
- [x] Seam-support (D27) holds: every outline pins the seam edge at full length from
      (−L/2, 0) to (+L/2, 0) and reaches full depth, so `L` and `W` keep meaning what
      they meant for a slab; the outline redistributes area, it does not shrink the part
- [x] Regenerate `smoke` (12/12, yaw on; `polygon_outlines: true` is a no-op for its
      T-only draw), `reference_tjoint` stays pinned. `configs/bench6a_*.yaml` are ready
      for the Phase 4 re-run: same per-type seed ranges, both mechanisms on
- [x] Anti-shortcut check: `scripts/qa_d28_gate.py` — corpus-level, exit-coded.
      **Statistic:** the length-weighted distribution of angles between each primary seam
      and every FREE boundary edge of both parts, where joint-constrained edges (the
      seam-BEARING ones and the seam-TERMINATING end edges) are excluded by nearness to
      the seam segment alone. Two earlier statistics measured their own artefacts and are
      recorded in the script so they are not rebuilt: "nearest edge excluding
      near-and-parallel" is always won by a terminating end edge (~90°, 43/62 seams), and
      "nearest edge excluding all near edges" is won by whatever sits just past the cutoff
      (winners' distances clustered at 12.0–12.5 mm against a 12 mm threshold — a metric
      measuring its own exclusion boundary)

**Gate:** PASSED 2026-08-27 on a 90-scene trial corpus (`bench6a` configs, both mechanisms):
terminal-bin mass 0.33 vs 0.22 uniform, inside the 2× tolerance, all five joint types
spread. The gate also caught two generator bugs the unit tests could not: **lap yaw was
silently dead** (the support bound tested the contact strip's y = 0 edge, which IS A's
boundary edge, so any tilt halved its chord and every lap pinned to 0° — 21/40 bench seeds;
fixed by testing the primary fillet's leading-toe line and the centreline instead, and only
the centreline when B overspans A), and the uniform outline vocabulary rebuilt the very 0°
spike it was meant to break. Only sampled corpora exercise these paths with the layouts'
own geometry — which is the argument for a corpus gate over unit tests stated by the patch,
now with two measurements behind it.

**Then:** re-run Phase 4 (deferred by decision 2026-08-27; three steps, no new code):
(1) generate the corpus — `python -m weldgen generate --config configs/bench6a_<type>.yaml`
per type with the bench seed ranges, plus the `_fx` twins if the fixture chunks are wanted;
(2) `python scripts/run_phase4_batch.py --corpus out/bench6a` (flag added for this);
(3) re-execute notebook 12 against the new `phase4_batch.csv.gz`. Expect `lit-ransac` and
`lit-ppf` precision to drop — that drop is the measurement of how much the axis-alignment
prior was worth, and is worth reporting as a figure in its own right.

**Effort:** 3 days (spent: yaw 1 day, outlines + gate 1 day).

---

# D31 / D32 — Joint classes disjoint by construction (implemented 2026-08-27)

Full text in `notes/patch_class_disjointness.md`; summary of what landed:

- **D31 clearance** `c = 2·min(t_A, t_B)` keeps the T/corner and lap/edge borders empty,
  near-AND-parallel throughout (a boundary meeting counts only within ~10° of parallel
  AND within `c`). T overhang stays; corner needs no band; lap forbids both flush
  configurations and end coincidence. Behind `class_disjoint` (off by default - the
  checks consume no stream draws, so pre-D31 corpora reproduce bit-identically; enabled
  in `bench6a_*` and `smoke`).
- **Edge is the combined configuration**: lap toes in-class
  (`ALLOWED_CLASSES_DISJOINT`), cross-run demotion off for lap AND edge - the number of
  lap seams is a property of the outlines (contact-polygon rule; D4 already derived the
  approach sides). Underside toes carry a distinct `underside` flag; the harness scores
  single-view against `gt_for_scoring(view)`, which drops them there and nowhere else.
- **D32 stratum**: `class_boundary_stratum` inverts acceptance; `configs/amb_lap.yaml`
  and `amb_T.yaml` sample the forbidden bands, scenes record `joint.ambiguous_with`,
  excluded from the main splits, reserved for the Phase 4 class-boundary analysis.
- **ISO 17659 alignment**: derived `joint.iso_17659_term` per scene;
  `PARAMETERS.md` §3.4 carries the verified mapping (angle joint 3.12 for non-90° T,
  edge = 3.14/3.8 - the parallel joint is our edge class, not a dropped type), the
  clause 5 citation, and the stated exclusions (3.11/3.15/3.16, edge openings > 0°).
- **Regression found and fixed on the way**: yaw × angular-misalignment coupling made
  toe lines NEAR-parallel to their generating face instead of exactly parallel, and the
  Phase 6a exact clip's slack (parallel-branch only) silently deleted the A-side toe of
  every yawed misaligned lap. Fixed with a near-parallel waiver: a clip constraint is
  dropped - never extended - where its breach stays within the slack over the run the
  other constraints allow. Pre-6a corpora were never affected (no yaw, exact
  parallelism).
- **Phase 4 consequence, restated:** edge ground truth grows and lap seam counts rise
  *by definition*; pre/post-D31 per-type numbers are not comparable. Fold into the
  deferred re-run.

---

# Phase 6b — Curved seams, non-planar primitives, grooves

- [x] **Seam sampler** — `weldgen/d29.py`, drawing from the D29 curve families under the
      realizability condition (amended framing, 2026-08-28)
- [x] **Part constructors:** swept plates, pipe-on-plate, pipe-to-pipe, rectangular tube
      on plate — DONE 2026-08-28 as two primitives, `geom.Tube` (base cut by the landing
      surface, exact D33 quadratic; the miter ring IS the seam) and `geom.SweptSlab`
      (offset band about a spine — rect tube, stiffener, and curved butt are the same
      primitive), `constructors.build_d29` for families 2–7 (patch_phase6b step 2)
- [x] **Groove preparations (D24, restricted by D30)** — straight butt seams only. ISO 9692-1
      makes the choice thickness-driven and citable, so the sampler picks the preparation from
      `t` rather than inventing a distribution:

      | `t` (mm) | preparation | ISO 9692-1 ref |
      |---|---|---|
      | ≤ 2 | raised edges | 1.1 |
      | ≤ 4 | square | 1.2.1 |
      | 3 – 10 | single-bevel | 1.9.1 |
      | 5 – 40 | single-V | 1.5, 2.2 |
      | > 12 | single-U | 1.6, 2.6 |

      Scope narrowed to **bevel, single-V, single-U**. Double-sided and J preparations are
      dropped — they do not add an argument the three already make, and each is another
      primitive to keep watertight.

      **This is what makes a butt joint a single seam.** With a groove the weld has one
      well-defined line at the groove root, instead of the two coplanar face centrelines a
      square preparation yields. The groove is cut *on the sampled seam line*, not read off the
      geometry afterwards — D3 applied to preparation.

      DONE 2026-08-28 as D35 (patch_phase6b step 4), with the table corrected against the
      standard's own PDF: the realised pool is `config.valid_preps(t)` over the VERIFIED
      rows — square (≤ 8), single-bevel 1.9.1 (4–10), single-V 1.3/1.5 (4–12 / > 12),
      single-U 1.6+2.6 (> 12) — raised edges (1.1) dropped with the thin-sheet end, and
      "no groove" stays in the pool. `geom.PreparedSlab` (monotone `v_edge(w)`, faces
      named per ISO 17659), D36 `groove_root` at exactly t − c, D37 mouth-anchored at
      zero code cost.
- [x] Expand `joint.prep` beyond `"square"` and re-scope `PARAMETERS.md` §5.0 — DONE
      2026-08-28: prep enum square/single_bevel/single_V/single_U, §5.0's empty-window
      claim scoped load-bearing to `prep == "square"`
- [x] **Surface-pair intersection** for the verification function: plane ∩ cylinder,
      cylinder ∩ cylinder, quadric ∩ plane — DONE 2026-08-28, and per the D29 amendment
      these live ONLY in verification (`ellipse_from_plane_cylinder`,
      `saddle_from_cylinders`, `verify_curved.rediscover_seam` at 1e-13): generation
      draws the curve first and derives the parts
- [x] `surface` block on non-planar faces; `bspline` parametric form pinned before
      starting — DONE 2026-08-28: `faces[].surface` descriptions on tube/swept faces,
      `bspline` pinned as clamped uniform with stored control points (constructed input
      only under D33, never a fitted approximation)
- [x] **Watertightness (D21) genuinely bites here** — budgeted, and the budget was spent
      without incident (2026-08-28): ring-strip topologies for the tube's cut modes and
      the prepared slab's fan caps are watertight by construction; pinned per primitive
      by test
- [ ] Re-run Phase 4 → **report where the plane-intersection baseline stops working**

## Schema changes curved seams force

Decide these before writing curved-seam code, not after.

| Field | Problem | Resolution |
|---|---|---|
| `dihedral_deg` | scalar; varies substantially along a saddle curve | per-sample array, or `{min, max, mean}` |
| `tacks` endpoints | the rule mandates both endpoints; a circle has none | wrap convention + `closed: bool` on the seam |
| `mps_rule` | assumes an open arclength | same wrap convention |
| D19 offsets | nominal/root/gap_mid separate along the local bisector, which rotates | already shipped as derived arrays — no change, but the Phase 4 "conversion table" becomes a distribution, not a constant |
| `length_mm` | ambiguous for closed curves | define as total arclength; `closed` flag disambiguates |

## What this phase carries

Two loads, not one. It defuses the trivial-label risk in `thesis_direction_handoff.md` §3 —
straight-seam tack labels are arithmetic, curved-seam ones are not — *and*, since ~~D17~~ was
withdrawn, it is the first point at which `part_geometry_id` varies by anything other than
dimensions. The D11 "held-out geometry" split only becomes a genuine geometry split here.

Note that **6a partially relieves this**: polygon outlines already make `part_geometry_id`
mean more than dimensions, so a 6b slip is less damaging than it was under the old single-phase
plan.

**Effort:** 3–4 weeks. The previous ~1 week estimate was wrong — it was written when the phase
held only "curved seams," and the section has since accumulated grooves, non-planar primitives,
surface-pair verification, and a full Phase 4 re-run.

**Scheduling risk, restated:** if 6b slips, the paper ships with straight seams and a
held-out-outline split. Both are defensible if stated plainly; neither is defensible if
described as something more.

---

## Open items opened by this patch

- [x] Pick the yaw range for T and lap — RULED 2026-08-27 (user): **full circle**, uniform
      over the per-scene feasible set of [−180°, 180°) (`layouts.feasible_yaw_deg`, 1°
      grid + jitter). The plate never needs to grow: infeasible angles are simply not in
      the set, and D31 separately rejects the flush-coincidence angles (lap yaw ≈ 180°)
- [x] Decide whether polygon outlines are convex-only — DECIDED 2026-08-27: **convex
      only** (trapezoid / parallelogram / triangle / quad / convex pentagon, deliberately
      non-uniform weights — a uniform draw measurably rebuilt the 0° spike). Re-entrant
      corners were not needed to break the axis prior, and the face registry and
      watertightness stayed simple
- [x] Pipe-to-pipe: pin the parametric form for the saddle curve — PINNED 2026-08-28
      (D33): `SaddleCurve` is a branch-parametrised exact quadratic with
      implicit-differentiation tangents — closed form, never a `bspline` approximation,
      so no fitting error enters the ground truth; verified on both cylinders to ≤ 1e-6 mm
- [x] Confirm ISO 9692-1 coverage of pipe preparation — CONFIRMED ABSENT 2026-08-28: Part
      1's rows cover plate butt preparation only; branch/nozzle preparation is not in it
      (nor in Part 2's submerged-arc scope). Recorded honestly rather than stretched:
      grooves stay plate-butt-only (D30) and curved scenes emit `iso_9692_ref: null`

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
      if the lab's existing expertise (Umut, Ege) makes it cheap.

      **Isaac Sim route confirmed available on this machine (2026-08-18):** Isaac Sim
      `5.1.0-rc.19` at `/isaac-sim`, with `omni.replicator.core` **1.12.27** in `extscache/`
      and an RTX 4060 Laptop GPU or RTX 5070 Ti Desktop GPU present; `/isaac-sim/python.sh` resolves both `isaacsim`
      and `omni.kit.app`. Replicator ships *with* Isaac Sim rather than being a separate
      install, so the second backend costs no new procurement — it needs a headless Kit app
      (`SimulationApp` before any `omni.replicator.core` import), which is why the module
      does not import from a plain interpreter.

      This does **not** overturn the default. D9 is about what the *release* requires, and a
      Replicator backend is RTX-only and proprietary, so it stays the second backend behind
      BlenderProc. What it changes is the risk: the fallback is known-present rather than
      hypothetical, and the "1–2 weeks, high variance" estimate can be read as variance in
      materials and failure-mode fidelity rather than in whether a renderer exists at all.
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

## 8. Metrics summary

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

## 9. Named risks

| Risk | Mitigation |
|---|---|
| Phase 8 slips | Tier 1 must be submission-sufficient on its own |
| Schema churn after Phase 4 | Phase 0 exists precisely for this. Resist adding fields |
| Trivial tack labels | Phase 6 before Phase 7; scope paper 1 as seam-only if Phase 6 slips |
| "Synthetic data isn't real" reviewer | Phase 9 real subset + the tier-1/2/real ablation |
| Reviewer reads the dataset as a seam-extraction-accuracy claim | Never frame it that way (handoff §2.5). The claim is about *labels*, not about beating anyone's extractor |
| Generator too heavy for others to run | D9. Test the install on a clean machine with no GPU before release |

---

## 10. Open decisions

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
Opened by the 2026-08-20 advisor meeting:

- [x] **CONFIRMED 2026-08-20 — MPS is a geometric proxy**, and load-path correctness is
      explicitly out of scope. Now a *stated limitation* rather than an open question, so the
      paper must say it in its own voice: the MPS label is a convention over exact geometry
      (D25, `mps_rule-0.1`), adopted for tractability, and it does **not** claim to identify
      the seam that carries load. Which seam is structurally critical is not recoverable from
      a single view by any method, so it cannot be a vision label
- [ ] Decide whether **Task 2 ships in paper 1 or is deferred**. Recommendation: generate the
      data and release the rule; scope paper 1 to Task 1 plus the seven-method comparison, and
      keep MPS as a short section if Phase 4 runs ahead
- [ ] **Grade the occlusion distribution before MPS is evaluated** (D26 sampler work, plus the
      framing-fraction change already landed) — MPS is a weak task while visibility is
      near-binary
- [x] **RESOLVED 2026-08-21** — `lit-modelreg` is implementable without the original CAD
      assets (`scene.json` is a sufficient CAD source, verified in code), *and* it remains
      constitutively L0-with-CAD because the model's seam is the stored truth. Both halves
      of the item turned out true at once; the label ships with every number
- [x] **BUILT 2026-08-20 — balanced benchmark corpus, 50 scenes per joint type.** 250
      scenes in `out/bench/<joint_type>/`, one config per type (`configs/bench_*.yaml`),
      loaded through `baselines.balanced_corpus(root, per_type=50)`. Per-type configs are not
      a convenience: `joint_type` is sampled per seed, so a mixed config gives whatever the
      draw gives — `out/phase3` gave 15 butt against 2 lap, and **three** per-type
      conclusions drawn from it have since been overturned (single-view vs full visibility,
      the lap segmentation delta, the lap density effect).

      Generated counts are 61 / 53 / 60 / 92 / 182 (T / butt / corner / edge / lap);
      `balanced_corpus` trims to the 50 lowest seeds per type, so the selection is a pure
      function of the corpus and not of generation order. It **raises** on a short type
      rather than quietly returning an unbalanced set.

      **The yields are themselves a result and must be reported with any per-type number.**
      Seeds needed per emitted scene, measured: butt 1,7 | T 1,2 | corner 2,2 | edge 2,6 |
      **lap 2,6**, with almost every loss to `NoVisibleSeams` — the omission policy (D-tier-1)
      conditions the dataset unevenly across joint types. That is a property of the joint, not
      a sampling defect, and correcting for it silently would hide it
- [ ] Decide whether radius-PCA's inability to express a coplanar seam is worth one more
      attempt or should be **reported as a mechanism limit**. The measurement says the band
      covers the whole area between two parallel plates; nothing in a variance ratio responds
      to a 180° dihedral

- [ ] Lap overlap length has no ISO citation and stays **[ours]**. AWS D1.1 or a fabrication text
      may give a minimum (commonly quoted as some multiple of `t`) — worth one lookup before
      submission, not before Phase 2
- [ ] Pin the three D19 seam definitions precisely in `SCHEMA.md` §1 and name the stored one