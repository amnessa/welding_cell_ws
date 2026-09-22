# Weld-seam extraction — two working modes (plan, 2026-09-21)

*Replaces the radius-PCA seam of README §8 as the plan of record. The benchmark built in
`weld_generator` (Phase 4: seven literature methods against constructed truth) is what
decides the design below; the numbers quoted are its.*

## The finding that reshapes the pipeline

`~/welding_points` runs on the SEPC — the parts' **CAD clouds at their ICP poses** — not on
sensor points. Once `pose_static` of A and B is known, the seam is implied by the two
poses: it is the intersection of the placed parts' surfaces, computable in closed form.
Any detector run on CAD points (radius-PCA today, or a better one) only rediscovers, with
error, what the poses already say. In the benchmark this regime is the "seam transferred
from the registered model" rung, which scores 0,95–1,00 because the model *is* the truth.
So mode A does not detect; it computes. Detection is for the case where there is no CAD.

## Mode A — CAD available (the existing pipeline, seam computed)

```
RGB-D → SAM2 mask → PPF classifier names the part → FoundationPose 6D → ICP-to-CAD
      → pose_static (A), pose_static (B)                              [unchanged]
      → weldgen primitives at those poses → D4 accessibility rule     [new]
      → every seam: analytic polyline, class, torch approach, weldable / reject reason
```

1. **Part registry** `models/weldgen_objects.json`: for each library CAD, its weldgen
   primitive with *measured* dimensions (slab L×W×t; tube r_outer / wall / length;
   rounded-rect band; prism outline). The lab's parts are made to the generator's
   convention, so this is 4–10 entries. A library part without an entry falls back to
   mode B on the sensor points.
2. **`seam_from_registration`** (new module; replaces `_welding_points`): `from_object`
   on each registry entry posed at `pose_static` → `weldgen.accessibility.
   enumerate_candidates(parts, access, joint_type=None)` → publishes one message per
   seam: polyline (mm, `base_link`), `seam_class`, `approach` (the cleared torch axis),
   `weldable`, `reject_reason`. Rejected candidates are kept (they are the hard
   negatives the planner must avoid). `weld_generator` becomes an installable dependency
   of this package (numpy 1.26 under Jazzy is already handled by the `_trapezoid` shim).
3. **Its only error is the registration's.** Carry `icp fitness / inlier_rmse` with every
   seam; the D4 verdicts are exact for the *nominal* parts at the *estimated* poses.
   *Measured 2026-09-21 on the two saved bench assemblies:* the ear of `test_objv2` came
   back 8–11 mm inside the base in one run and 1–8 mm above it (tilted 1,6°, leaning 8°)
   in the other. Single-view ICP does not observe a thin plate sliding in its own plane.
   The generator's D4 rejects both (its gates assume contact), so mode A judges with a
   **pose tolerance** (`weld_pose_tol_mm`, 10 mm until the fiducial bound replaces it),
   never rejects on fit-up, and reports the gap/penetration per seam (`fitup_mm`). The
   seam line — the intersection of the two face planes — is invariant to the sliding
   error, which is why the fillets survive it; the pen-mark check is what tells whether
   the line is right in the world. Stated limit: a lapping sheet thinner than the pose
   tolerance is undecidable (`member_within_pose_tol`) — laps on 2–3 mm sheet need the
   fiducial bound first; T-fillets do not. Live on the bench (third assembly, ear
   tilted 2 mm in / 11 mm out end to end): the seam runs as far as the member stays
   within the tolerance (`clip_registered`). A weldgen bug surfaced by the real poses
   (`_mutually_visible` probing a degenerate segment, which lost both fillets of a T at
   any non-exact rotation) is fixed in `weld_generator` with a regression test.
3b. **Tacks (done 2026-09-21).** `compute_tacks` applies the generator's `tackrule-0.1`
   unchanged to the mode-A seams; each tack has `tack_no` (1-based along its seam) and
   `order` (scene-wide weld sequence, ends first, sides interleaved) so the robot's
   reach ordering can start from either. Published as a cloud + text labels, written to
   `welding_tacks.json`. The thesis' DP tack selection over a quality field is the
   later, richer replacement for the same slot.
4. **Optional measured refinement (mode A+).** `_run_once` already builds the segmented,
   background-subtracted sensor points of each part and discards them after ICP. Keep
   them per part in `static_frame` at `save_object`; label each point by the nearest
   face of the registered CAD (this IS the "surfaces + part membership" input the
   quadric method needs, obtained from registration instead of from truth); fit
   plane / quadric per face to the *measured* points (`lit_quadric.detect(points,
   region_labels=face, part_labels=part)`), intersect. Benchmark: exact given the
   surfaces (pooled F1 0,87), immune to sensor noise, no far-side mirror in a single
   view; seam threshold must exceed the root gap (2,25 mm default vs the 1,1 mm
   reference gap); bench and fixture points excluded by the existing ground cut and
   masks. The difference between the model seam and the measured seam is a **fit-up
   diagnostic** (real gap, warp) and a check on the registration.

## Mode B — no CAD (sensor only, new)

```
RGB-D → SAM2: one mask per part (a click per part; the assembly is NOT one mask)
      → PPF classifier: NO MATCH (confidence below threshold)                 [server change]
      → per-part sensor points, background-subtracted, in base_link
      → surfaces per part: normal-based region growing on the measured points
      → lit-quadric: plane / quadric per surface, pairwise intersection across parts
      → weldability: reduced rule — torch-cone clearance ray-cast against the measured
        cloud (no solids, so D4's exact containment is unavailable; stated)
      → seams with class, approach, `source: sensor`
```

* **Server change:** the PPF classifier must return "no match" (score threshold, or a
  best-match residual) instead of the nearest library part. Until it does, mode B is
  selected by hand.
* **What the benchmark says, honestly.** Without surfaces and part membership the
  quadric method collapsed (0,87 → 0,00) — but that was on *full-surface* synthetic
  clouds where region growing leaks through a thin plate's thickness. A real single
  view sees one side of each face, and SAM2 supplies part membership, so mode B should
  sit between the two rungs. **This is a Phase 9 measurement**, not an assumption: mode B
  on the real scans against pose-derived truth.
* **No library save in mode B.** A single view is an incomplete part; saving it would
  poison the PPF library. `save_object` refuses (or warns) when the current object has
  no CAD. Future fix, when wanted: multi-view fusion — the eye-in-hand poses are known,
  so views transform into `base_link` exactly; subtract background (ground cut + SEPC),
  merge, voxel, then a surface reconstruction that is watertight enough for
  `/add_model`. Parked.

## What stays the same

Stages 1–3 of the thesis (quality field on `(s, φ)`, DP tack selection with spacing
bounds, ordering) consume seam polylines with an approach direction. They are unchanged;
they receive better input, with class and weldability attached, from either mode.

## The selling point, stated

A perception stack that degrades gracefully: with a CAD model the seam is exact by
construction (registration error only); without one it falls back to the best geometric
mechanism the benchmark found (quadric intersection), on sensor points, with its
accuracy known from the same benchmark — and the two are scored on the same real scans
(Phase 9) against the same constructed truth.

## Order of work

1. **DONE 2026-09-21** — Part registry (`admittance_control/weldgen_registry.py`,
   `scripts/build_weldgen_registry.py --verify`: 4 box parts verified at 0,000 mm, 10 meshes
   honestly unsupported) + `seam_from_registration` (mode A) wired into `~/welding_points`
   as the default with radius-PCA as fallback; `test/test_seam_from_registration.py` (7).
   Still to do in this step: hand-written `tube` / `swept_slab` entries when pipes and the
   `270circle` band are used; a run on the real bench (assembly.json from the pipeline).
2. Keep per-part sensor points at `save_object`; face labelling; mode A+ refinement and
   the fit-up diagnostic.
3. PPF "no match" on the server; mode B path; refuse library save without CAD.
4. Phase 9 real subset = the validation campaign for all of the above.
4b. Pen-marking the tacks on the UR5e (approach, force-gated touch, mark):
   `notes/pen_marking_plan.md` (2026-09-22) — the placement check this pipeline is
   validated by.
5. Quality field on the new seams; plot `q(s)` before any selection code (the week-two
   experiment of `quality_field_k_point_selection.md`).

## Validation, and what validates what

* **Mode A is validated by an independent pose measurement**, not by the Phase 9 truth
  (which is itself derived from the same registration): the fiducial board / jig bound
  on `pose_static`, and the pen-mark placement check on the UR5e (~0,1 mm from a flatbed
  scan) — the placement ground truth the thesis already planned.
* **Mode B and the seven benchmark methods are validated on the Phase 9 scans** against
  the pose-derived truth, with its uncertainty stated.
