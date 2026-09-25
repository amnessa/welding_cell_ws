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
2. **Registration noise floor — MEASURED 2026-09-25:** with part and arm stationary the
   ICP pose had position std (5.2, 2.5, 3.0) mm, range up to 24 mm, orientation std 1.4°
   with swings to 6.4° (28 mm at 250 mm). That alone covers the 8 mm. DONE the same day:
   `save_object` now takes the robust mean of the last `save_pose_window` (30) tracked
   poses (median translation, chordal rotation, jumps rejected) and writes the spread
   as `pose_stats` into `assembly.json` (`admittance_control/pose_stats.py`). Keep the
   arm still a few seconds before saving. STILL OPEN: why the tracker wanders that much
   on a stationary scene - candidates: the in-plane slide of a plate is a flat valley
   for point-to-plane ICP (each tick settles elsewhere), D435i depth noise at the
   working distance, the Welsch nu re-estimation, the crop/background subtraction
   changing the point set tick to tick. Log fitness/rmse alongside the pose, try a
   larger `max_corr_dist` decay or a fixed nu, and compare the jitter of the base plate
   vs the standing plate (the flat plate should be the worse one if it is the slide).
   The averaging respects the workflow "register → move the part by hand → save":
   only the ticks since the part came to rest are averaged (`stationary_tail`).
   If the wander is the solver and not the sensor, the ICP variants worth trying, in
   order of effort: (a) a boundary/outline term - point-to-point on the depth-edge
   points of the crop, which is what pins a plate's in-plane slide and rotation that
   point-to-plane cannot see; (b) symmetric ICP (Rusinkiewicz 2019: the symmetric
   objective converges wider and steadier on planar geometry, a small change to the
   residual); (c) generalized ICP (plane-to-plane, covariance-weighted). The current
   solver is Fast & Robust ICP (Welsch + Anderson) on point-to-plane; keep it as the
   inner loop and add (a) first.
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

- **C2 stroke: DONE 2026-09-25** (`stroke_mode` dot | tack | seam on the marking node;
  contact-referenced, chunked, force-corrected depth). Bench run pending. If the
  chunk-rate depth loop is too coarse for long seams, UR's `force_mode_controller` is
  the upgrade.
- **Holder clearance:** the calibrated tip (181.7 mm) leaves 1.6 mm at the bisector of a
  square T. A longer pen or a slimmer holder before the next square fillet.
- Save the 4-point TCP result and the extrinsic together with a date in the tool config
  (provenance of every number the marks depend on).
- Fixture boxes in the collision model the day clamps replace the magnets; table plane
  from a flange touch if the parts ever sit low.
- Dry-run / twin: the Isaac twin exposes joint commands as a topic, not the trajectory
  action - a small adapter would let the marking node run in the twin for real.

## INVESTIGATION — FoundationPose real-time tracking, fused with the ICP

**The idea.** The FoundationPose server has a tracking mode (`track_one`: after a
`register`, it refines the pose frame to frame from the previous pose, ~20–30 Hz on the
server's GPU) that we never use - we call it once per part for the seed, then our own
ICP tracks. A second, independent pose stream could (a) re-seed the ICP when it drifts
or loses the crop, and (b) be fused with it. The fusion the user has in mind is an
EKF on the pose with the ICP and FoundationPose as two measurements.

**What it can and cannot fix - decide this first.** A filter removes NOISE (the jiggle),
never BIAS. The 8 mm lateral error is a bias until shown otherwise (roll test, jitter
probe: items 1–2 of NEXT). If the jitter probe reports sub-millimetre position noise,
fusion buys nothing for the marks and this item drops to LATER. If it reports
millimetres, averaging N ICP poses at `save_object` is the cheap first fix, and fusion
is the next step only if FoundationPose's own noise is measured to be smaller or
independent (different failure modes: FP uses RGB and the render-and-compare, so on the
textureless MDF plates its in-plane slide may be as ambiguous as the ICP's - measure it).
For STATIONARY parts (our case at marking time) the "EKF" is a recursive weighted
average of a constant with two sensors; the gain IS the ratio of the measured
covariances, so without the two noise measurements there is nothing to tune. The EKF
earns its name only for moving parts (a part being pushed into place), which is a
different use case - nice, later.

**The network problem, solved without ROS across Tailscale.** The server lives in
another network; the bridge already talks to it with HTTP POSTs over the Tailscale IP.
Tracking needs a stream, not a round trip per frame with a fresh registration:

  1. Server (`fp_server.py`): three endpoints. `POST /track/start {object, pose_init}`
     builds a tracker session from the last registered pose (or the ICP's pose, sent
     in); `POST /track/step` with one RGB (JPEG) + depth (PNG16, mm) frame returns the
     refined pose and a score; `POST /track/stop`. One session per object; the server
     keeps the previous pose. Keep-alive HTTP is enough: a 640×480 JPEG (~50 KB) + depth
     PNG (~150–250 KB) at 5–10 Hz is ~1–2 MB/s, and Tailscale is WireGuard, so on the
     same LAN the round trip is a few ms, over WAN tens of ms. 5 Hz is plenty for a part
     that does not move; the ICP stays the fast loop.
  2. Bridge (`foundationpose_bridge_node.py`): a `~/start_tracking` service that starts
     the session and a timer that posts frames and publishes the returned pose as
     `/perception/fp/pose` (PoseStamped, camera frame, with the score) - the mirror of
     `/perception/icp/refined_pose`.
  3. Measure before fusing: `pose_jitter_probe.py -p topic:=/perception/fp/pose` next to
     the ICP's; log both for 30 s stationary and during a slow hand push; compare noise,
     latency and the BIAS between them (mean offset; that is the number that matters).
  4. Then, if justified: a `pose_fusion_node.py` - state = pose (6, twist about the
     current estimate), process = constant pose (stationary) or constant velocity
     (moving), measurements = ICP pose with its measured covariance and FP pose with
     its; gate each measurement by Mahalanobis distance so a FP jump (a wrong
     re-registration) is rejected rather than averaged in; publish the fused pose and
     let `save_object` take it. The ICP node takes the fused pose as its next seed
     (robustness against losing the crop) - one line where it reads `_current_pose`.
  Alternative to the stream: run FoundationPose's tracker on this laptop's RTX 4060
  (track_one is light: the heavy part is the registration); then no network at all.
  Worth a try if the docker builds here.

**Order.** Not before NEXT 1–3. Then step 3 (the two probes) decides steps 1–2 vs LATER.

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
