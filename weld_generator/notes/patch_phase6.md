# Plan patch — Phase 6 split, orientation diversity, artifact-driven curved seams

*Replaces the existing Phase 6 section. Decisions continue at D28.*
*Drafted 2026-08-27, after Phase 5 self-annotation returned 2,5 mm RMSE.*

---

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

## D29 — Curved seams are sampled as workpiece configurations, not as curves

"Arcs, C-shapes, S-shapes, closed curves" is shape-driven and unbounded — there is no
principled stopping point and no defence against "why not also helices?"

D3 already gives the answer: seams are constructed, so sample the **artifact** and take
whatever seam it produces. The vocabulary is then finite because the artifact vocabulary is
finite, and every entry is industrially motivated rather than arbitrary.

| # | Configuration | Seam geometry | Joint type |
|---|---|---|---|
| 1 | plate on plate | line | T, corner, lap |
| 2 | pipe perpendicular on plate | circle | T |
| 3 | pipe at an angle on plate | ellipse | T |
| 4 | pipe-to-pipe intersection | saddle curve (Viviani-like) | T or butt |
| 5 | rectangular tube on plate | closed rounded rectangle | T |
| 6 | swept (curved) plate on plate | arc / spline | T, lap |
| 7 | two curved plates edge to edge | arc | butt |

Seven configurations cover essentially every curved seam in the collected literature — the
intersecting-pipe path planning work, nozzle welds, stiffener perimeters. Anything outside
this list is out of scope by construction, which is a defensible sentence in a way that "we
stopped adding spline types" is not.

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

- [ ] **In-plane yaw** for T and lap: rotate the upper part about the lower part's face normal,
      uniform over the range that keeps the seam fully supported on the lower part
- [ ] **Polygon outlines** for all five joint types, from the D28 vocabulary
- [ ] Update the D4 verification function — face enumeration currently assumes 6 faces per slab
- [ ] Update the `faceRef` naming scheme for polygonal parts (`A:side_3` rather than `A:+u`),
      and check the `w`-is-thickness invariant survives
- [ ] Seam-support constraint (D27) still holds: the along-seam extent is pinned by the seam;
      yaw and outline vary everything else
- [ ] Regenerate `smoke`, `reference_tjoint` stays pinned as the regression fixture
- [ ] Add an anti-shortcut check to CI: **the distribution of angles between each seam and the
      nearest non-seam boundary edge must be approximately uniform**, not concentrated at 0°
      and 90°. This is the test that the prior is actually gone

**Gate:** the seam-to-boundary angle histogram is flat. If it spikes at 0/90°, the shortcut is
still present and Phase 4 numbers remain biased.

**Then:** re-run Phase 4. Expect `lit-ransac` and `lit-ppf` precision to drop — that drop is
the measurement of how much the axis-alignment prior was worth, and is worth reporting as a
figure in its own right.

**Effort:** 3 days.

---

# Phase 6b — Curved seams, non-planar primitives, grooves

- [ ] **Seam sampler** driven by the D29 configuration table, not by curve families
- [ ] **Part constructors:** swept plates, pipe-on-plate, pipe-to-pipe, rectangular tube on plate
- [ ] **Groove preparations (D24, restricted by D30)** — straight butt seams only. ISO 9692-1
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
- [ ] Expand `joint.prep` beyond `"square"` and re-scope `PARAMETERS.md` §5.0
- [ ] **Surface-pair intersection** for the verification function: plane ∩ cylinder,
      cylinder ∩ cylinder, quadric ∩ plane
- [ ] `surface` block on non-planar faces; `bspline` parametric form pinned before starting
- [ ] **Watertightness (D21) genuinely bites here** — swept and revolved primitives produce
      degenerate caps and duplicated seam vertices where slabs never could. Budget for this
      specifically; it is the item most likely to consume unplanned days
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

- [ ] Pick the yaw range for T and lap — full 0–360° with a seam-support constraint, or a
      restricted range. Full range is more honest; check it does not make the lower plate
      absurdly large to keep support
- [ ] Decide whether polygon outlines are convex-only. Concave outlines add re-entrant corners
      (strong hard negatives) but complicate the face registry and watertightness
- [ ] Pipe-to-pipe: pin the parametric form for the saddle curve before implementing, since
      `bspline` approximation error would enter the ground truth
- [ ] Confirm ISO 9692-1 covers pipe-on-plate and pipe-to-pipe preparation, or find the
      companion standard (ISO 9692-2 covers submerged arc; branch/nozzle preparation may sit
      elsewhere) — needed before groove work extends past plate butt joints