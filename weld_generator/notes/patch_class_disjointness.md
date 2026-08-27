# Plan patch — joint classes disjoint by construction, lap seams from the contact polygon

*Drafted 2026-08-27 from the class-differentiation discussion. Decisions continue at D31.*
*Status: IMPLEMENTED 2026-08-27 (all rulings in); summarized in `dataset_plan.md` before the Phase 6b section.*

The gap: `joint.type` is a scene-level label, but nothing guarantees the configurations are
disjoint. A T with B's footprint reaching A's edge is locally a corner; a lap with a
near-zero overlap is an edge joint with a lip. The resolution chosen is **disjointness by
construction** — constrain the sampler so each type's configurations cannot resemble
another's — rather than deriving a type per seam. This keeps scene-level `joint.type` for
stratification, and D22 classification keeps doing what it does (off-label seams stay
`matches_joint_type: false`).

Three of five types are already disjoint for free: T/corner/butt separate from each other
and from the stacked pair by dihedral and topology; corner places B *past* A's edge by
construction. The constraints below close the two remaining borders (T–corner, lap–edge).

---

## D31 — Clearance floors keep the class borders empty

**The clearance (settled 2026-08-27, thickness-only):**

```
c = 2 · min(t_A, t_B)
```

The fractional `0.10 · min_edge` term of the first draft is dropped — which also
dissolves the "what is min_edge for a prism" question entirely. The anchor is the
standard, verified against the PDF text (not the paraphrase that first motivated it):

> ISO 17659 defines the joint classes by **where the parts meet**: a T-joint is parts
> meeting "at approximately right angles (forming a T-shape)" (§3.10) and a lap joint is
> parts that "lie parallel to each other and overlap" (§3.9) — both meetings on a *face* —
> while a corner joint is "two parts meet **at their edges** at an angle greater than 30°"
> (§3.13) and an edge joint is the same meeting at 0–30° (§3.14).

`c` operationalises "meets the face, not the edge": the contact line must leave enough
flat surface beside it for an intended fillet weld on the outer side. The quantification
is ours: leg length (ISO 17659 §3.21) is `z ≈ t` of the thinner member, so ~`t` of
material per fillet and `2t` as the floor. Note the standard also separates the *stacked*
pair the same way — "lap joint" (§3.9, partial overlap) vs "parallel joint" (§3.8, which
Annex A maps to the US term "edge joint") — so the lap/edge clearance band is drawing a
line the standard itself draws.

**T vs corner:** the forbidden configuration is the *corner-like* one, not mere nearness:
a contact line running **near-parallel** (within ~10°) to a boundary edge of A **while
within `c` of it**. End-overhang of B is explicitly allowed, as before (the seam clips to
the shared run, `min_overlap_frac` semantics unchanged — the earlier draft's full-extent
containment was withdrawn 2026-08-27): a yawed seam that *crosses* the clearance band at
an angle is overhang, not a corner. This is the same near-AND-parallel test the D28 gate
uses to identify seam-bearing edges, applied at generation time. Reject (route to the D32
stratum) when violated.

**Lap vs edge:** the parts must not share (or nearly share) a common edge. Edge joints are
flush by construction (`stack_offset = 0`, unchanged). For lap the forbidden bands are:

- `overlap < c` — B's leading edge nearly flush with A's welded edge (the lip case)
- `|overlap − W_A| < c` — B's leading edge hovering over A's **far** edge (the same flush
  configuration on the other side; only applies when `overlap ≤ W_A + c` — beyond that B
  cleanly overspans and A's far edge sits mid-face under B)
- end-edge coincidence: no end edge of one part within `c` of a near-parallel end edge of
  the other (near-AND-parallel, like the T rule — an end edge crossing the band at an
  angle is not a flush candidate)

Where parallel faces would put two edges within `c`, either resample the offending offset
or use the D28 yaw to break the coincidence — yaw for lap already exists.

**Corner needs no clearance (ruled 2026-08-27):** the seam stays at the corner. B past
A's edge IS the class definition (§3.13, parts meeting at their edges), so the existing
construction is already unambiguous and no mirrored band applies.

**Edge joints are the combined configuration (decided 2026-08-27):** an edge scene is a
stack of flush plates, and a stack *necessarily* also presents lap-suitable edges — the
narrower part's far edge sits over the wider part's face, and there is no way to construct
an edge-only configuration. The asymmetry is one-directional: **lap-only is constructible
(the clearance bands forbid flush edges), edge-only is not.** So lap scenes stay pure by
construction, and edge scenes declare the union: every suitable boundary is a welding
seam — exactly-flush pairs are edge seams, offset-over-face boundaries are lap toes, and
both are IN-CLASS for `joint.type: "edge"` (`ALLOWED_CLASSES["edge"]` gains the lap-toe
class; the `weldable` / `matches_joint_type` split of D22 was built for precisely this and
already enumerates the toes — today they are merely labeled off-class). Within an edge
scene the D31 band applies to the NON-welded edge pairs: the welded edge is exactly flush
by construction, every other pair must clear by `c` or the scene routes to the D32
stratum — so each boundary is unambiguously one thing.

**Consequence to state in Phase 4:** edge-scene ground truth grows, so per-type edge
metrics change *by definition*, not by method behaviour. Fold into the deferred re-run;
never compare pre- and post-patch edge numbers directly.

---

## D32 — The exclusion band is a new prior; measure it instead of hiding it

D28 broke the axis-alignment shortcut; D31 creates a different one — **contact lines never
occur near plate boundaries**, which a method can learn and which generalises to nothing.
The mitigation is the project's standard move (cf. the load-path tie-rate measurement):

- an **`ambiguous` stratum**, ~5% of corpus size, sampled *inside* the forbidden bands
- labeled with both candidate types (`joint.type` + `joint.ambiguous_with`)
- excluded from the main D11 splits
- used only for the Phase 4 class-boundary analysis: how do methods degrade as a
  configuration approaches the T/corner or lap/edge border?

**Integration amendment (bit-compat discipline):** the stratum is generated from its own
presets via a config flag that *inverts* the clearance constraint — never as an in-config
probability, which would consume extra stream draws in main-corpus scenes and break the
free-twin property every 6a mechanism preserves.

---

## Lap seams are the contact polygon's boundary

With two parallel plates, the contact region is the intersection of the two outlines; its
boundary segments are contributed by one outline or the other:

- segments from the **upper** part's outline → its edge sits on the lower part's face →
  **weldable from above**
- segments from the **lower** part's outline → its edge is underneath → **underside toe,
  distinctly labeled** (ruled 2026-08-27): the seam block gains an `underside` flag,
  derived from the D4 approach direction relative to the stack normal — no new geometry,
  the enumeration already computes the approach. Underside toes stay `weldable: true`
  (reachable in a positioner) and stay in-class, but the **single-view condition scores
  them separately**: from the overhead camera they are invisible by construction, and
  folding them into single-view ground truth would penalise every method identically
  while measuring nothing. Full-exterior keeps them in the target set.

The seam count of a lap is therefore a property of the outlines, not of the joint type: a
triangular upper plate fully inside the lower footprint has three toes; crossed outlines
alternate weldable and underside segments — a hard, realistic case.

**Integration amendment (what actually changes in code):** D4 already *discovers* all of
these segments and already derives their approach sides physically (`_clear_axis`); with
Phase 6a outlines the multi-segment geometry already exists. What suppresses them today is
`_drop_cross_runs`, which demotes candidates off the dominant direction — added when "one
seam per lap side" was the intent, now reversed by this patch **for face-contact joints**.
The implementation is to narrow that pass (prune by `min_seam_length` and D22
classification instead), not to add a seam constructor. The narrowing must cover **lap
AND edge**: with edge as the combined configuration, an edge scene's off-direction
boundary runs (one part's end edge over the other's face) are in-class lap toes, and the
demotion would suppress exactly the seams the combination legitimises. Butt keeps the
demotion; T's short end-segments (width `t_B`) die to `min_seam_length` on their own.

**No superseded allowances:** T overhang stays (withdrawn 2026-08-27 after one round in
the draft); `min_overlap_frac`, `length_offset` slack and the 1 mm margin in
`max_supported_yaw_deg` are all unchanged. D31 for T is a post-draw accept/reject on the
near-AND-parallel test alone.

---

## Consequences to anticipate

- **Yield drops** in lap and edge (band exclusion) and mildly in T (the near-parallel
  rejection is rare once yaw is on). Regenerate the
  per-type yield table after this lands; fold the corpus regeneration into the already
  deferred Phase 4 re-run rather than regenerating `bench6a` twice.
- **Lap seam counts rise** (end segments become toes) and **edge-scene ground truth
  grows** (lap toes in-class). Phase 4 per-seam metrics and the annotation protocol count
  seams — check nothing assumes ≤ 2 seams per lap scene, and re-baseline edge metrics.
- Schema: `joint.ambiguous_with` (array of type strings, optional) + a stratum marker;
  pre-patch scenes stay valid.

## ISO 17659 alignment (added 2026-08-27, verified against the PDF text)

The taxonomy discussion resolved into `PARAMETERS.md` §3.4 and a derived per-scene field
`joint.iso_17659_term`. The load-bearing points:

- **The parallel joint (3.8) is EXCLUDED, and our edge joint keeps its name** (corrected
  ruling 2026-08-27, superseding this section's first draft, which conflated the two).
  The distinction is not overlap fraction but where the join happens: a parallel joint
  is bonded over its entire faying surface (3.4) — explosive cladding is the standard's
  own example — so its join is an AREA, not a curve, and there is no seam for a
  seam-extraction method to find. Excluded by problem definition. Our edge class sits
  at α = 0° inside edge joint 3.14's 0–30° band and is correctly named as-is; 3.13/3.14
  partition edge-contact at 30° with no gap and no overlap. The D31 lap clearance still
  reads straight from the standard: 3.9 is partial overlap ("en se recouvrant
  partiellement"), full overlap is 3.8 and out of scope.
- Non-90° T scenes are the standard's **angle joint (3.12)** — named in the derived
  field, stratification keeps `type: "T"`.
- Clause 5 ("type of joint is determined by the number, dimensions and relative
  orientation of the parts") is the citation under D22/D27/D31.
- Excluded: cruciform (3.11), multiple (3.15), cross (3.16) — multi-part; edge-joint
  openings > 0° (stacked construction is parallel by design, D23).
- The "extends continuously past both sides" sentence and clause-10 route to the
  T/corner threshold came from chat paraphrase; the citable partition is 3.13/3.14's
  30° edge-meeting split and 3.10's "approximately right angles" (tolerance ±10° is
  OURS and recorded as such).

## Open items

All three of the draft's open items were ruled on 2026-08-27 and are folded into the text
above: the clearance is thickness-only (`min_edge` question dissolved), corner needs no
clearance (the seam stays at the corner — §3.13 makes edge-meeting the class definition),
and underside lap toes get a distinct `underside` label scored separately under the
single-view condition.

- [ ] Schema: `underside` flag on the seam block, `joint.ambiguous_with` — write both
      before implementation so pre-patch scenes stay valid
- [ ] Citation hygiene: quote ISO 17659 only by the §-numbers verified above; the
      "extends continuously past both sides" sentence circulating in notes is an
      interpretation, not standard text, and must not appear as a quotation
