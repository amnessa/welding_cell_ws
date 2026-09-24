# Open work, both projects (2026-09-24)

*One list, so nothing lives only in a chat. Ordered by what unblocks what. Status words:
DONE / NEXT / OPEN / LATER. The thesis-side plan of record is `seam_two_modes_plan.md`,
the motion side `pen_marking_plan.md`, the dataset side
`weld_generator/notes/dataset_plan.md` + `phase9_plan.md`.*

## Where we are

Mode A is complete and **validated on the robot**: the pipeline registers two parts,
computes the seams from the poses (D4 rule under a pose tolerance), places tacks
(tackrule-0.1), plans elbow-up collision-free motion, descends under force and records
the contact. First real touches: **−0.4 / −0.6 mm along the pen axis** on the tack it
reached, and a **~8 mm lateral** error seen by eye on the mark. That lateral number is
now the object of work, and it is measurable, not a guess.

## NEXT — attribute the 8 mm (an afternoon, in this order)

1. **Tool or world?** The same tack at rolls 0/90/180/270 about the pen
   (`tack_reachability.py --roll <deg>` then `plan`/`next`; photograph each mark). A
   lateral error that TURNS with the wrist is the pen TCP (redo the pendant's 4-point
   TCP with more spread, or the touch-off script); one that STAYS is the camera or the
   registration.
2. **Registration noise floor.** `pose_jitter_probe.py` for 30 s with everything
   stationary: position std / orientation swing of the ICP pose. If it is millimetres or
   tenths of a degree (a 0.5° swing is 2 mm at 250 mm), make `save_object` average N
   tracked poses (small change in the ICP node) and look at the D435i depth noise at the
   working distance.
3. **Camera rotation.** The recalibrated extrinsic's board-in-base rotation spread was
   3.6°; a 1° rotation error is ~9 mm at 0.5 m - the right size for the residual.
   Recapture with 25–30 poses, the board larger or nearer (filling a third of the
   frame), and deliberate ROLLS of the wrist about the camera axis between poses (that is
   what pins the rotation); judge with `resolve_handeye.py` (target: GOOD).
4. **Nominal vs calibrated FK** (~3 mm at the tip): load `config/ur5e_calibration.yaml`
   deltas into `kinematics.py`, or record contacts from TF `tool0` instead of FK.
5. **Measure the lateral error with the pen, not the eye:** a probe mode that touches
   the base plate near the tack and slides toward the standing plate until the lateral
   force rises - the measured root vs the registered one, in 3D, per tack. This is also
   the "mode A+ measured refinement" of the plan, done with the pen instead of the camera.

## OPEN — motion side (`pen_marking_plan.md`)

- **C2 stroke: draw the weldable seams**, not only dots. The seam polylines are in
  `welding_seams.json`; the admittance node's "Z of a plane" becomes "the pen axis";
  or use UR's native `force_mode_controller` (loaded, inactive) to hold 1.5 N along the
  pen while following the seam. Draw the full seam first, tacks as segments later.
- Save the 4-point TCP result and the extrinsic together with a date in the tool config
  (provenance of every number the marks depend on).
- Fixture boxes in the collision model the day clamps replace the magnets; table plane
  from a flange touch if the parts ever sit low.
- Dry-run / twin: the Isaac twin exposes joint commands as a topic, not the trajectory
  action - a small adapter would let the marking node run in the twin for real.

## OPEN — perception side (`seam_two_modes_plan.md`)

- **Mode B** (no CAD): PPF "no match" on the server; SAM2 per-part masks; region growing
  + lit-quadric on sensor points; refuse the library save without CAD until multi-view
  fusion exists. Validated against Phase 9 truth, not against mode A.
- **Mode A+**: keep per-part sensor points at `save_object`, label by nearest CAD face,
  lit-quadric refinement; fit-up diagnostic from the measured seam. (Item 5 above is the
  pen's version of the same measurement.)
- Registry entries for non-box parts when they enter the library: `tube` for the
  pipes, `swept_slab` for the `270circle` band, hand-edited slabs for tabbed plates
  when the envelope is too coarse.
- Quality field on the seams and the DP tack selection (thesis stages 1–3), once the
  tack points are trusted on the bench.

## OPEN — parts and data

- **More CAD objects with matching MDF/metal parts**: every new part needs (a) the CAD in
  `models/`, (b) the server's PPF library entry, (c) a registry entry
  (`build_weldgen_registry.py --verify`; hand entry for non-boxes), (d) one bench cycle:
  register → seams → reach → touch. Pipes and RHS ordered 2026-09-21 for Phase 9 (RHS
  60×60 / 80×80 / 50×100, Ø42.4/60.3/76.1/101.6/114.3 × 2 mm).
- **Phase 9 real subset** (on hold until the metal arrives): `label_real_scan.py`,
  view planner, fiducial-board pose bound, `d435i_measured`, pilot of 3 configs; 11
  strata × 10 configs × 5 views, test-only, truth from registered poses, never hand-made.
- The fiducial board is also what replaces `weld_pose_tol_mm` (10 mm placeholder) with a
  measured pose bound.

## OPEN — weld_generator / paper

- Desktop: commit the final ICRA figures / PDF / source zip to git (repo copies stale).
- Notebook 16 (tier-2 render analysis) not written; no training script on `train_v1` yet.
- Pin numpy for release (determinism across versions); annotator repeat; lap-overlap
  citation; advisor's written no-welding scope.

## LATER

- Force-controlled seam following with the torch (the stroke, with heat) and the
  distortion-aware tack order (`order` field is already computed for it).
- MoveIt planning scene as the upgrade path for transit planning when fixtures get real.
- Thesis writing: the validation chapter has its numbers now (fit-up, contact depth,
  lateral error attribution).
