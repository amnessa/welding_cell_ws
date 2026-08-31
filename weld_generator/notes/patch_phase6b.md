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
3. **Arms DONE 2026-08-28** (`weldgen/verify_curved.py`); scene assembly is the
   remaining half, and D37's mouth apex moves to step 4 where grooves make it
   testable. What landed: `rediscover_seam` recomputes the seam from the PLACED PARTS
   alone (poses, radii, face planes — never the record) via the step-1 factories, at
   1e-13 against the drawn curve, and detects a 0,5 mm part perturbation — the Phase 2
   gate, curved; `containment_residual` is the consistency half for the constructed
   families. `curved_seam_set` emits every seam a configuration carries with
   PER-POINT exact frames (nA/nB/approach/dihedral all vary along a curved seam:
   73–107° on a tilted pipe, 60–137° on a saddle) — including two findings the
   machinery forced: (a) **a swept stiffener's welds are the OFFSET curves at ±t/2**,
   not the drawn spine (mid-material is not a weld) — new `offset` parametric kind,
   exact positions AND tangents; (b) **an open bore CLEARS the local torch cone**
   (measured: a 50 mm bore passes a 15 mm standoff cone), so the inner fillet is a
   REAL weld on large pipe and the thing that forbids it on small pipe is torch-body
   confinement — a semantic gate with the D13 precedent, parameterised as
   `torch_clearance.bore_min_diameter_mm = 80` **[ours]**, and the verdict genuinely
   cuts both ways across sampled radii (pinned by test). `seam_verdict` derives
   weldable/confined_bore/toe_of_centreline/bisector_blocked with the clear fraction
   recorded (a curved seam can be partially reachable; work-angle fallback per point
   is a noted refinement, not yet needed — every sampled weld cleared at 1,0).
   `ray_hits_part` now dispatches SweptSlab (AABB + exact-containment refinement).

   **Scene assembly DONE 2026-08-28** (`weldgen/scene_curved.py`): a curved scene is a
   SCENE — same `(scene_json, arrays)` contract, same writer, same schema (validated
   per family), same content hash, generated through `weldgen generate` when a preset
   sets `seam_families` (`configs/curved_smoke.yaml`; `out/curved_smoke` at 14 seeds,
   10 emitted across five families). Stream mapping keeps the D3 order legible:
   `seam_curve` draws the family and curve, `joint_config` the realization's nuisance
   dims, `placement` a z-preserving pose (fixture off), camera/surface/noise as in the
   plate pipeline. The generation gate runs BEFORE emission (rediscovery ≤ 1e-6 mm for
   #2–#4, consistency residual for #5–#7, chord ≤ 0,25 mm for D34; a failure raises,
   never emits). Emitted seams carry world-frame `parametric` (`transform_parametric`,
   valid under z-preserving poses — the offset kind refuses anything else), `closed`,
   per-sample `seam_<i>_dihedral` arrays backing `{min,max,mean}` summaries (scalar
   when constant, so plate consumers keep reading numbers), per-point
   tangent/approach/nA/nB, `clear_fraction`, and the D22 negatives (bores, toes).
   Pinned by test: bit-reproducibility, and the D1 receipt at FILE level — the stored
   parametric block resamples into the stored `seams.npz` arrays.
4. **DONE 2026-08-28.** `geom.PreparedSlab` (monotone `v_edge(w)` profile makes
   `contains` exact in closed form; every face a plane strip except the U's radius —
   a cylinder patch, analytic to sample; watertight, volumes exact to the analytic
   cross-section, ISO 17659 face names root/fusion/radius), `config.valid_preps` +
   `sample_groove` (PDF-verified rows; the U draw enforces
   `t − c > R(1 − sin β) + 1` or the fusion face vanishes; groove gap from ITS row's
   b range, overriding no. 617), grooved `_layout_butt` (B = the same primitive
   rotated 180°; single-bevel keeps B square per 1.9.1), and emission per D36
   (`groove_root` at exactly t − c below the top, D19 triple square-only — and the
   single-bevel test pins that the mouth centreline genuinely SHIFTS off the root
   centre, which is why the two curves must be separate). Two integration findings:
   (a) **the coplanar arm needed its own `coplanar_gap_tol_mm`** — a 50° V on 8 mm
   plate opens ~9 mm at the mouth, past `contact_tol`, whose thin-sheet cap must not
   stretch (it closes the wrap-around hole); the wider bound applies only where the
   in-plane gap is measured. (b) **D37 cost zero code**: the nominal was always
   mouth-anchored (D19 defines it on the exposed faces), so the cone never starts
   inside the constriction — the measured blanket-rejection applied to a
   root-anchored seam this pipeline never had. Recorded, because "the risk was real
   and the design already dodged it" is worth more than silence. Reachability note:
   single-U needs t > 12, above the default thickness range — `configs/
   grooved_butt.yaml` (t 3–20) exercises all four preparations. 17 tests.
5. **DONE 2026-08-31 — gates, ruled option (b) with a mandatory caveat.** How curved
   scenes meet the D28 anti-shortcut gate (`scripts/qa_d28_gate.py`), user-ruled:

   - **Closed seams are exempt.** A closed seam's tangent sweeps every direction in
     its plane; the axis-alignment prior is unlearnable from it. Families #2–#5 are
     all closed, so the exemption removes most curved scenes outright.
   - **Open curved seams (#6–#7) are REPORTED, not gated** — option (b). Their spines
     are drawn at random, so the sampler prior D28 polices cannot arise by
     construction; gating would test what construction guarantees. The per-point
     statistic (stored seam tangents, each weighted `edge_length / n_samples` so a
     seam's total weight per edge matches the plate convention) is printed so the
     number exists for the write-up; only the plate histogram binds.
   - **Provenance-based exclusion, implemented regardless of (a)/(b) — the user's
     caveat: without it the printed number is WRONG.** The 12 mm nearness proxy
     cannot catch a swept band's far edges: the longitudinal ones are offsets of the
     seam's own spine (exactly parallel, a band-width away) and even the far vertical
     corners have spine-pinned positions and read 90° under every possible draw — a
     spike that means "stiffener", printed as if it meant "shortcut". Rule: a part
     whose spine carries the seam's own rigid-motion-invariant signature (sorted
     pairwise distances of fixed-arclength samples — pose-independent, so it works
     across the part-local/world divide) is a **D29-derived part, and ALL its
     boundary edges are joint-constrained**. Measured consequence: family #7 reports
     nothing (both parts derive from the curve — no sampler-free direction exists in
     the scene), family #6 reports seam tangents against the independent plate
     outline (real spread, ~34% in the first bin from spines drawn along the plate's
     long axis — an honest number the thesis can discuss).
   - **D34 corpus sweep, BINDING**: every scene recording `cloud.max_chord_error_mm`
     must be ≤ 0,25 mm; the same script checks it and fails the exit code on any
     violation (curved_smoke worst: 0,0498 mm). Plate scenes don't record the field
     (their meshes are exact) — vacuously clean.
   - Pooled re-run: bench6a + curved_smoke → plate gate PASS (terminal mass 0,34 vs
     0,44 allowed), D34 PASS.

   **`out/bench6b` — the balanced 6b benchmark corpus** (`scripts/make_bench6b.py`),
   user-ruled 50–70 per class, realised at 60 with families balanced INSIDE each
   class: T = 10 each of line/circle/ellipse/saddle/rounded_rect/swept_path; butt =
   15 line-square + 15 line-grooved (D35 draw, square draws filtered to the square
   arm) + 30 arc (#7), i.e. 30/30 by family; corner/lap/edge = 60 line (single-family
   classes). Layout matches `balanced_corpus` (per-type dirs + index.jsonl). Seeds
   INTERLEAVED per source (residues modulo the class stride from base 2 000 000),
   not blocked: `balanced_corpus` trims to the lowest N seeds, and interleaving keeps
   every seed-sorted prefix at the intended family proportions — a Phase 4 run at
   `per_type=50` stays balanced (measured: T 6–10 per family, butt 29/21 by family
   at the 50 trim; exact 10/10/…, 30/15/15 at 60 — `run_phase4_batch.py` now passes
   `--per-type` through to `balanced_corpus` so a 60-scene run uses the whole class).
   Rejects recorded, never backfilled; manifest.json carries the recipe.

   Gate run over the finished corpus: plate D28 PASS (terminal mass 0,38 vs 0,44
   allowed), curved report populated (swept_path vs plate outline), D34 PASS
   (worst 0,0500 mm over 80 curved scenes). One legible finding the per-mechanism
   rows surface: the grooved arm reads `mech none` at 71% first-bin mass — a grooved
   plate stays a rectangular `PreparedSlab` (the preparation is machined along a
   straight edge; prism outlines deliberately do not compose with it), so the axis
   prior is STRUCTURAL for grooved rectangles and the corpus-level pooling is what
   absorbs it. Extending outlines to the three non-prepared edges of a PreparedSlab
   is a possible later enhancement, not a Phase 6b need.
6. Phase 4 re-run — per the user's decision, after 6b completes (bench6a + bench6b in
   one batch; nothing regenerates twice).
