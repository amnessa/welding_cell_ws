# Plan patch — Phase 6b kickoff: exactness, grooves, and the decisions made before code

*Drafted 2026-08-28 from the 6b design discussion. Decisions continue at D33.*
*Status: CONFIRMED rulings from the discussion (D33–D35) plus integration resolutions
(D36–D37, scope calls) recorded before implementation. Supersedes nothing; extends the
Phase 6b section of `dataset_plan.md`.*

---

## D29 amendment (2026-08-28) — curve families, not workpiece configurations

D29 as first written was artifact-first — sample the configuration, take whatever seam
it produces — which is the REVERSE of D3, the inversion the generator is named for.
Amended before the constructors consumed the direction (the cheapest possible moment):

- **The seam curve is drawn first**, from families that admit a two-part realization:
  a curve is realizable if it lies on the intersection of two constructible surfaces
  or can serve as the sweep path of a plate profile. That realizability condition is
  the real bound the configuration list was smuggling in — an arbitrary 3-D spline is
  not realizable, a planar one is — and stating it directly makes the scope defence
  geometric rather than curated.
- **The parts are derived FROM the curve** (`realization` in `d29.py`): a circle's
  radius is the pipe radius, the ellipse's semi-minor and aspect are the pipe radius
  and tilt, the rounded rectangle is the tube's outer wall, the arc/spline is the
  sweep path. Same seven cases, same parameters, renamed by what they produce.
- **Surface∩surface leaves the generation path**: the intersection machinery lives
  only in the D4 verification arms (step 3), which rediscover what construction
  placed — the standing Phase 2 gate. The saddle family is the one that LOOKS like an
  intersection; its parameters (R, r, θ, offset) are the family's own, and the D33
  quadratic is its closed-form parametrization — a family evaluation, not a solver
  between independently sampled parts.
- **Noted asymmetry, kept deliberately**: for the line family the recorded seam still
  comes from the D4 enumeration at generation time (its clipped extent genuinely
  depends on placement and support, and Phase 1's gate is that the rule rediscovers
  the intended joint); for the curved families the record is the drawn curve and D4
  purely verifies. One principle — construction is truth, rediscovery is the gate —
  two mechanically different realisations of it, stated rather than hidden.
- The draw ORDER now encodes the inversion: curve parameters → derived realization →
  part-level nuisance parameters (wall, height, gap) drawn by the constructor.

`dataset_plan.md`'s D29 entry carries the amended table (family / parameters drawn /
realization derived / joint type) with the one-line equivalence to the old
configuration list, so the industrial motivation is not lost.

---

## D33 — D1 stands unqualified: every D29 seam is closed-form or constructed

The pipe-to-pipe worry (quartic intersection curve, numeric solving, a tolerance clause
on the project's central claim) dissolves under one observation:

> **Substituting a line into a quadric yields a quadratic.** Intersecting any *ruled*
> surface (plane, cylinder, cone) with any *quadric* is closed-form along the ruling.

For pipe-to-pipe, parametrize around the **branch** cylinder: any branch-surface point is
`P(φ, s) = c(φ) + s·â` with `c(φ) = o + r(cos φ·û + sin φ·v̂)`. Substituting into the
main cylinder's implicit form `|P − p₀|² − ((P − p₀)·â_m)² = R²` gives a quadratic in
`s` for each `φ` — one square root, exact, handling unequal radii, arbitrary branch
angle, and offset axes (nothing requires the axes to intersect). Take the root on the
branch's entering side for a set-on stub.

Per D29 configuration:

| # | Configuration | Seam form | Why exact |
|---|---|---|---|
| 1 | plate on plate | line | Phase 1–6a |
| 2 | pipe ⊥ on plate | circle | plane ∩ cylinder |
| 3 | pipe at angle on plate | ellipse | plane ∩ cylinder |
| 4 | pipe-to-pipe | saddle, `(φ, s(φ))` | line-in-quadric quadratic above |
| 5 | rect tube on plate | rounded rectangle | four lines + four plane∩cylinder arcs |
| 6 | swept plate on plate | arc / spline | **constructed, never intersected** |
| 7 | curved plates edge-to-edge | arc | **constructed, never intersected** |

Configurations 6–7 are the D3 move applied again: **sample the seam curve first, sweep
the plate profile along it** — the seam is an input, so spline approximation error never
enters the label. The earlier "bspline parametric form pinned before starting" open item
resolves to: bsplines appear only as *constructed inputs*, never as fitted intersection
outputs. The tolerance clause proposed in the first round of this discussion is
**withdrawn**; record the argument, because "we checked and it is exact" is worth more
in the paper than silence.

---

## D34 — The one numerical qualification is the MESH, and it is gated

Curved surfaces tessellate; sampled points lie on chords, giving a systematic *inward*
bias on convex surfaces of `≈ R·Δθ²/8`. The label is untouched (it comes from the
parametric form, never the mesh), but the input carries it, so:

- record `max_chord_error_mm` per scene (over all curved faces),
- gate it at **≤ 0,25 mm** — an order of magnitude under the 2,5 mm Phase 5 annotation
  floor — by choosing the tessellation pitch `Δθ ≤ √(8·tol/R)` per radius,
- this is the honest home of the "synthetic geometry is discretised" caveat.

---

## D35 — Grooves sampled from `valid_preps(t)`, with "square" staying in the pool

Preparation is drawn per scene from the set valid at the sampled thickness (ISO 9692-1),
not from a fixed pool and not always:

| `t` (mm) | pool |
|---|---|
| ≤ 4 | square |
| 4 – 8 | square (both sides), single-bevel, single-V |
| 8 – 12 | single-bevel, single-V |
| > 12 | single-V, single-U |

- "Square" (no groove) **stays in the pool** where valid — §5.0's headline (the empty
  window for square-prep butt) lives there and must remain generable.
- Thickness and preparation are **correlated by design**; state it in the paper so it is
  read as realism, not confound.
- With grooves in the corpus, §5.0's claim is scoped to `prep == "square"` explicitly —
  load-bearing now, not cautious. `joint.prep` expands beyond `"square"` and
  `joint.iso_9692_ref` gains the groove rows (exact row values verified against the PDF
  when the sampler is written, same discipline as the 17659 quotes).
- D30 unchanged: grooves on **straight butt seams only**.

---

## D36 — D19 and the ISO "root": no rename, scoped instead

ISO 17659 Table 1 reserves *root face*, *root gap*, *depth of root face* for groove
preparation; D19's `root` curve is a different object and the two would collide in
groove code. The first-round proposal was to rename D19's curve. **Resolved without the
rename**, which would break every stored corpus and the annotation tooling for a
vocabulary problem:

- The D19 triple (`nominal` / `root` / `gapmid`) is **square-prep only** — exactly the
  geometry it was derived for. Array names in `seams.npz` are unchanged.
- Grooved seams emit `nominal` plus one new derived curve, **`groove_root`**, at the
  ISO root (the root-face line the preparation defines). "This is what makes a butt
  joint a single seam" (existing 6b text) attaches to `groove_root`.
- The Phase 4 D19 conversion table is therefore scoped to the square-prep subset,
  stated wherever the table appears.
- SCHEMA.md documents the reservation: in prose about preparation, "root" means ISO's
  root; D19's `root` is referred to as "the D19 root curve".

---

## D37 — Torch clearance for grooves is tested from the MOUTH, not the root

Measured before any groove code was written (the advisor's five-minute check): the D4
torch cone is `half_angle_deg = 30°` about the bisector, cast from the seam point. A
single-V of 40–60° included has a half-opening of 20–30°, so cone rays strike the bevel
faces of every V at or below 60° included, and the work-angle fallback (`±45°`) tilts
one ray family deeper into a wall. **Under the current model, grooved butt seams would
come back `bisector_blocked` across the board.**

The fix is a modelling decision, not a tolerance change: **ISO 9692-1 preparations are
designed to guarantee torch access — that is what the standard is for** — so D4's job on
a grooved seam is to verify the approach *from the groove mouth outward* (cone apex at
the mouth line, i.e. the seam translated to the plate surface along the groove axis),
with the interior of the preparation vouched for by the standard that shaped it. The
cone parameters stay as they are; only the apex moves, and only for `prep != "square"`.

---

## Scope calls for the part constructors (decided before writing them)

- **Pipe-on-plate is SET-ON only** (stub on the surface). Set-through (nozzle
  penetrating the plate) needs a hole — boolean CSG on the plate mesh — which is
  exactly the watertightness trap D21 warns 6b about, purchased for a second variant of
  the same seam curve. Excluded explicitly, one line in the paper.
- **The fixture stays off for non-planar configurations** (D29 #2–#7). A cylinder rests
  on a line, `contact_mode ∈ {flat, on_edge, propped}` does not describe that, and the
  D31 clearance rules are defined on plate footprints. The paired fixture ablation
  remains a plate-joint study; extending it to revolved primitives is future work, not
  a silent gap.
- **D31/D32 are plate-scoped**: `class_ambiguity` returns no bands for the curved
  configurations in 6b v1 (their class borders — e.g. a saddle T vs a saddle butt — are
  set by the D29 constructor, not by continuous offsets). Recorded so the clearance
  rule is not silently assumed to cover geometry it never saw.

## Schema changes (from the existing 6b table, now with owners)

| Field | Resolution |
|---|---|
| `dihedral_deg` | per-sample array on the seam block for curved seams (scalar stays for lines) |
| `tacks` endpoints / `mps_rule` | wrap convention + `closed: bool` on the seam |
| `length_mm` | total arclength; `closed` disambiguates |
| D19 offsets | unchanged (already derived arrays); conversion table scoped per D36 |
| `parametric` | new kinds: `circle`, `ellipse`, `saddle`, `arc`, `bspline` (constructed only) — each with its exact-form parameters |
| `surface` block | populated on non-planar faces (cylinder: axis, radius, span) |
| `max_chord_error_mm` | per scene, D34 |

## Order of work

1. **DONE 2026-08-28.** Schema + `parametric` kinds + `closed` flag, `weldgen/curves.py`
   (Ellipse3D / Arc3D / SaddleCurve / BSplineCurve / Segment3D / CompositeCurve, all
   analytic positions and tangents, arclength sampling exact-on-curve), the factories
   from defining surfaces, and `weldgen/d29.py` (the
   seven-family curve sampler; reframed curve-first by the D29 amendment above — the
   circle/ellipse draws construct the curve directly and derive the pipe from it,
   verified by the same containment tests).
   16 analytic tests: every curve on its defining surfaces to ≤1e-6 mm — unequal radii,
   offset axes, tilted branch saddles included — tangents vs finite differences,
   uniform-arclength density, JSON round-trip, grazing/parallel guards. One scope
   sharpening found by the tests: the 30 mm curvature floor binds only the DRAWN
   families (#6–#7); artifact-derived configurations (#2–#5) carry whatever curvature
   their sampled radii produce — a 10 mm tube corner IS the seam the artifact makes,
   which is D29's point — with the radii ranges as the recorded parameters.
2. **DONE 2026-08-28.** Two primitives cover all six curved configurations:
   `geom.Tube` (wall thickness = the `t` ISO keys on; base end optionally CUT by the
   landing surface — a plane for pipe-on-plate, the main cylinder itself for
   pipe-to-pipe, both from the D33 closed forms, so the miter ring IS the seam offset
   by the gap; verified to 1e-13 through the whole wall) and `geom.SweptSlab` (an
   offset band about a spine curve, extruded — #5 rect tube = closed spine with an
   inward band, #6 stiffener = ±t/2 band standing at the gap, #7 curved butt = two
   flat one-sided bands about the arc, gap-centred, tops flush). Cloud samples are
   ANALYTIC on the true surfaces — D34's chord error lives only in the mesh
   (occlusion/export), and both primitives sit under the 0,25 mm gate by
   construction. `visibility.ray_hits_tube` is an analytic interval refined by exact
   containment (boolean-only sub-sampling, 99,5%+ agreement vs dense marching);
   SweptSlab `contains` resolves against a fine spine polyline, same boolean-only
   caveat. `constructors.build_d29` realises configs 2–7 (#1 stays with
   `layouts.build`); 19 tests tie every constructor back to the step-1 curve on the
   posed surfaces — the D21 watertightness budget was spent without incident (ring-
   strip topologies are watertight by construction in every cut mode).
3. Surface-pair intersection arms for D4 (plane∩cylinder, cylinder∩cylinder) + the D37
   mouth-apex clearance.
4. Grooves (D35 pool, D30 straight-butt-only, `groove_root` per D36).
5. Regenerate gates: chord-error gate (D34), D28 gate re-run (curved seams enter as
   their chord direction? — resolve when the sampler exists), full suite.
6. Phase 4 re-run — per the user's decision, after 6b completes (bench6a + bench6b in
   one batch; nothing regenerates twice).
