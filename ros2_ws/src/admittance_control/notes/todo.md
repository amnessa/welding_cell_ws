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

1. **Tool or world?** Status 2026-09-28: after the extrinsic recapture and the plane
   ground cut the error is **3–5 mm**. The roll test at a tack is weak (in a fillet the
   corner turns a lateral TCP error into depth; not every roll is reachable), so it is
   done on the measured table instead: `table_touchoff.py -p mode:=tcp_check` touches
   one spot vertically at rolls 0/90/180/270 (paper: the dots must coincide) and tilted
   30° toward four azimuths with the tool yaw held; the contact heights against the
   plane give the TCP error in the pen frame by least squares
   (`table_probe.solve_tcp_error`, conditioning reported; simulated reachable from a
   pen-down start over the table, all 8 orientations). Lateral < 1 mm clears the tool:
   the 3–5 mm is then camera/registration.
   **DONE 2026-09-28: lateral TCP error 0.21 mm (conditioning 0.24, fit residual 0.04 mm,
   8 touches) - the TOOL IS CLEARED; the 3–5 mm is world-side.** The fit also printed
   −4.0 mm "along the pen": it rests on one number, the tilted touches reporting the tip
   +0.54 mm above the plane (vertical ones −0.01), amplified by 1/(1 − cos 30°) = 7.5.
   The same lift at all four azimuths is what a rounded tip (radius ≈ 4 mm with the TCP
   at the apex) or a pen flexing under the 0.75 N side load produces; it is NOT applied
   to the TCP (a 4 mm shorter TCP would put every vertical touch 4 mm off). Effect on
   marking: the contact depth of a 45° descent reads ~1 mm early; lateral position
   unaffected. Unwinding the wrist between the roll round and the tilted round, and a
   controller-result check in the touch-off node, were needed to get here.
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
3. **Camera extrinsic, checked against the robot (2026-09-28).** Today's 33-sample
   capture grades GOOD once the rotation metric is fixed (spread about the MEAN: 1.8°;
   the old max-vs-sample-0 read 6.8°): camera at (34.8, 89.0, 54.5) mm in tool0 with the
   pendant TCP (1.07, −1.35, 180.91) composed. But the 25 Sep calibration put it at
   (15, 85, 55): 20 mm apart while each fits its own samples to 3 mm - a systematic the
   ChArUco residual cannot see (bracket moved, or an unknown TCP during the 25 Sep
   capture). `scripts/extrinsic_check.py` settles it against the pen-measured table:
   the camera's table points through TF must land on `table_plane.json` (tilt < 0.3°,
   offset < 2 mm in every view). Also: `resolve_handeye.py --write` now refuses to run
   without `--tcp-offset` (it had just overwritten the composed file with the TCP-frame
   one; restored).
   **Result 2026-09-28, 4 views at wrist yaws −135/−51/39/127:** height +1.7…+2.8 mm;
   tilt split CAMERA-fixed **0.36°**, WORLD-fixed 0.38° (the table 40–60 cm from the
   touched patch), residual 0.09°. The camera part is ~2 mm at 300 mm - part of the
   3–5 mm - and equals the ChArUco solve's own rotation uncertainty (~0.3°), which is why
   recalibrating kept wandering. `helper/calibration/refine_extrinsic_from_table.py
   --write` → `notebooks/T_tcp_to_cam_refined.npy` (tilt corrected by 0.36°, mostly about
   camera x; translation and rotation about the optical axis unchanged; the file in use
   is not touched). Verify: launch with it, extrinsic_check at four yaws → camera part
   ≲ 0.1°; then re-register and mark.
   **VERIFIED:** with the refined file the camera-fixed tilt is **0.04°** (was 0.36°); the
   0.41° left is world-fixed (the table far from the patch). The user promoted it:
   `notebooks/T_tcp_to_cam.npy` = refined, `T_tcp_to_cam_unrefined.npy` = the ChArUco-only
   solve. Open: a constant +2.3 mm height offset in all views - depth bias of the D435i at
   300 mm or the table 2 mm higher there than the extrapolated pen plane; one round of
   views over the touched hexagon separates them.
3b. **Camera rotation (earlier note).** The recalibrated extrinsic's board-in-base rotation spread was
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

## DONE — the table measured, the ground cut follows it (2026-09-28)

`table_touchoff.py`, hexagon 10 cm + centre, 7 two-speed touches (recorded at
1.52–2.02 N): table plane z = −97.74 mm at (−0.516, 0.096) m, **tilt 1.57°** in base_link,
**RMSE 0.20 mm**; the driver's calibrated tip sits ~1.0 mm above the planner's FK (the
nominal-FK gap, measured). Over the touched patch the table runs −93.7 … −101.8 mm, so
the old flat cut at −100 mm left the table IN over most of the workspace (up to 6 mm
above the cut), and the registered base plate lies 1–12 mm above the table: the base
plate's tracker saw a large flat table right under it every tick. The ICP node now cuts
along the measured plane (`ground_plane_file`, `ground_plane_offset_m` 2 mm); the
real-robot launch picks `notebooks/table_plane.json` up automatically. NEXT: re-run
`pose_jitter_probe.py` on the base plate - this is the first test of whether the table
was the jitter. The collision model's table is still flat (`marking.json` table_z_m
null); a tilted-plane table there is a small follow-up if seams near the table matter.

## (was NEXT) the table / ground cut (found 2026-09-28)

The ICP's ground removal is a hard z cut at `ground_z_m` = −0.10 m in base_link: it sits
~3 mm under the base plate's underside, so it cuts registered plate corners on the low
side and keeps the holder tops and sides inside the 30 mm crop margin - non-part points
next to the model every tracking tick (a candidate for the 5 mm / 6° jitter). Also the
assembly subtraction's 5 mm equals the pose jitter. Steps: (1) measure the table with
`scripts/table_touchoff.py` (hexagon + centre, two-speed pen touches, calibrated TCP
from the driver) → `notebooks/table_plane.json`; (2) holder height from the same run or
a ruler → `ground_z_m` just above the holder tops; (3) re-run `pose_jitter_probe.py`; (4)
if still noisy: a MODEL-relative floor (drop crop points more than a few mm beyond the
model's own bottom face, in the object frame) and a smaller crop margin below the part.

## Contact force on rigid surfaces (found 2026-09-28)

Touching the bare table at 10 mm/s: contact detected at 2.66 N, ~10 N once the arm had
stopped, and the 8 N abort then cancelled the back-off itself, leaving the pen pressed.
Detection at 1.5 N works; the PEAK is approach speed × reaction time (tens of ms) ×
stiffness, and a non-sprung pen on a rigid table is very stiff. Fixed the same day in
`table_touchoff.py` and `tack_marking_node.py`: moves away from the surface are guarded
only by `release_abort_force_n` (40 N), the release starts with no settling pause and
is FK-relative (straight up from where the tip is), approaches are 4 mm/s then 1 mm/s,
the dot's dwell is skipped when the contact is stiff. To actually HOLD 1.5 N on metal:
(a) UR's `force_mode_controller` (loaded, inactive) - compliant along the pen, regulates
the force in the robot's 500 Hz loop; the right tool for the stroke on steel;
(b) a compliant pen holder (a spring of a few N/mm makes 1.5 N a position, not a force);
(c) the marking descent at 20 mm/s was fine on magnet-held MDF/metal (1.5–1.9 N) but
will overshoot the same way on rigidly clamped Phase 9 parts: two-speed descent there too.

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
- **Mode A covers only the 5 PLATE strata today** (T/line, corner, butt square, lap,
  edge): the registry knows slabs only and `judge_registered` skips curved faces. The
  6 curved strata need, in order of effort: (1) registry entries `tube` (r_outer, wall,
  length, axis) for the Phase 9 pipes and RHS, `swept_slab` for strips / the `270circle`
  band, verified against the CAD with the D34 chord budget; (2) pipe-on-plate
  (circle/ellipse: `curves.ellipse_from_plane_cylinder`) and pipe-on-pipe (saddle:
  `curves.saddle_from_cylinders`) from the REGISTERED poses - the same calls as
  `verify_curved.rediscover_seam` - plus the per-point cone test of
  `verify_curved.curved_seam_set` / `cone_clear_fractions`; (3) RHS and swept strip on
  a plate: NEW code (the generator constructs these from the curve, there is no
  rediscovery arm to reuse) - intersect the part's side faces with the plate at the
  registered pose; (4) the tack rule's closed-seam branch already exists; (5) motion:
  a full loop around a pipe needs large wrist rolls and part of the far side may be
  out of reach - the reachability report shows it per tack. Needed before Phase 9's
  curved strata can be marked; the pipe cases (2) are the cheap half.
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
