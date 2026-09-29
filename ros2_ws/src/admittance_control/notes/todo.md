# Open work, both projects (2026-09-28)

*Open items only. What was done and why lives in the README (§14 for the placement
accuracy work) and in the plans: `seam_two_modes_plan.md` (perception),
`pen_marking_plan.md` (motion), `weld_generator/notes/dataset_plan.md` + `phase9_plan.md`
(dataset).*

## Where we are

Mode A is complete and validated on the robot: register → compute seams from the poses →
tacks → elbow-up collision-free motion → force-gated touch → dot or stroke. The marking
error went from ~8 mm to **3.7–4.3 mm** (README §14: what was found, how, and the error
budget). Parked there on purpose; the next topic is FoundationPose tracking.

## NEXT — FoundationPose pose estimation, then a Kalman filter with ICP

Planned by the user (2026-09-29) after the touch decision above. First test how good
FoundationPose's pose is on its own, then fuse it with the ICP; the tracking details
follow.

The server has a tracking mode we never use (`track_one`: after `register`, refine frame
to frame, ~20–30 Hz on its GPU); we seed once and our ICP tracks. Question: is its pose
steadier than the ICP's (which wanders ~3.6 mm per tick on a still part), and could it
re-seed or be fused with it?

1. **Measure first, fuse later.** A filter removes noise, never bias. Fusion is worth it
   only if FoundationPose's noise is smaller than, or independent of, the ICP's. On
   textureless plates its in-plane ambiguity may be the same. For a still part an "EKF"
   is a weighted average whose gain is the ratio of the two measured covariances.
2. **Server** (`fp_server.py`): `POST /track/start {object, pose_init}` (from the last
   registration or the ICP pose), `POST /track/step` (one JPEG + 16-bit depth PNG → pose +
   score), `POST /track/stop`; one session per object, keep-alive HTTP over the Tailscale
   IP at 5–10 Hz (~1–2 MB/s). Alternative: run `track_one` on this laptop's RTX 4060 (the
   heavy part is registration), no network.
3. **Bridge**: `~/start_tracking` + a timer posting frames, publishing
   `/perception/fp/pose` (mirror of `/perception/icp/refined_pose`).
4. **Compare**: `pose_jitter_probe.py` on both topics, still for 30 s and during a slow
   push: noise, latency, and the mean offset between them (that is the number that
   matters).
5. **Only if justified**: `pose_fusion_node.py`, with a pose state, constant-pose or
   constant-velocity process, both poses as measurements with their measured covariances,
   and Mahalanobis gating so a wrong re-registration is rejected. `save_object` takes the
   fused pose; the ICP takes it as its seed.

## NOW — thin-plate face straddle in ICP (2026-09-29)

After the kinematic fix the marks got WORSE (14–16 mm). Cause: the ear (8 mm plate)
registered straddling its own two faces - the model's hidden back face paired with the
visible face (6.5° lean, root 9.5–10.3 mm off vs the live cloud; the earlier "ear lean"
of 5–6.6° in several saves was the same thing). Fix: ICP normal gate
(`icp.NORMAL_GATE_DEG`, node param `normal_gate_deg` 60°, model normals from
`sample_mesh_surface(return_normals=True)`); on the saved cloud it returns the ear to
0.3–0.8° lean and the root to ±0.2 mm. `scripts/check_registration.py` compares the
saved poses with the live cloud face by face (straddle → drift ≈ plate thickness).
Bench run after the fix: all 4 tacks reachable (new holder envelope), contact still
9.6–9.7 mm early - an unmodelled 7 mm metal plate under the ear (7 / 0.69 along the pen;
the 0.9–4.6 mm "fit-up gap" was the same plate). Removed; re-test pending. Transits now
keep `transit_clearance_m` 20 mm (the 3 mm transit to the far side passed 3.7 mm from
the ear and hit it at 9 N; the re-plan keeps ≥ 20.3 mm).
Not done: a refusal at save_object when the check fails.

Clean re-test (plate removed): marks 5.1 / 7.9 / 8.0 mm off, contacts 6.2–11.0 mm early
on BOTH sides of the ear (so not a sideways shift). Pen touches (`scripts/touch_probe.py`)
split it: pen length +1.2..+2.9 mm on the bare table (vs the 28 Sep plane); base top
+2.3 / +4.3 mm (≈ 0 / +2 net of the pen: registered ~0.7° off in tilt, far end low);
ear faces +0.9 / −1.2 mm. At a 45° approach every mm the base sits higher is ~1.4 mm of
early contact and ~1 mm of mark offset; pen 2 + base 0–2 + blunt nib ~1 predicts
4–8 mm early and 3–6 mm off, measured 6–11 and 5–8 → ~2–3 mm unexplained (holder flex,
nib shape; a corner touch at a tack centre would tell). Every term is now 1–3 mm, the
D435i's own level at 0.5 m.

## DECISION — to touch or not to touch (2026-09-29)

The remaining ~5 mm is the sum of 1–3 mm sensor-level terms. Two ways on, and the
thesis should say which scenario it claims:

- **Touch (industrial practice, "touch sensing"):** before each tack, two force-gated
  touches ~15 mm from the root (base top with the pen vertical, standing face with the
  pen horizontal) shift the root line by the measured offsets. Removes camera,
  extrinsic and registration error at the tack; the pen's own error mostly cancels
  (same tip touches and marks). Reuses table_touchoff's two-speed touch, marking's
  planner, touch_probe's per-face offsets. ~10–15 s per tack. Expected < 1 mm.
- **No touch (academic scenario: parts that must not be touched, or a vision-only
  claim):** the error has to come down on the sensing side. **Chosen next: the multi-view
  close-range refinement `~/refine_pose`, planned step by step in
  `notes/multiview_refine_plan.md`.** Options, roughly by payoff:
  - a close-up refinement scan per seam: D435i depth error grows ~z², so 0.5 → 0.3 m
    is ~2.8x less;
  - multi-view registration, fusing 2–3 scan poses: this averages the depth bias and
    the in-plane ambiguity;
  - FoundationPose pose + ICP fused (NEXT below);
  - a depth-bias map of the D435i, measured against the pen-touched table;
  - redo the 4-point TCP (about 2 mm): only pen_tool.json and the table plane change,
    NOT the extrinsic, which is stored in tool0.
- A middle road for the thesis: vision-only as the method, touch as the reference
  that measures its error (touch_probe already does this by hand).

## PARKED — the remaining ~4 mm (README §14, error budget)

- **Kinematic model mismatch - FIXED in code 2026-09-29, bench verification pending.**
  Confirmed: the probe's calibrated row equals the pendant TCP at 5 poses (0.001°), the
  nominal row wanders 2.4–4.2 mm / 0.52–0.55°; a model of the 28 Sep setup predicted
  5.4/5.9 mm lateral and +2.6/+2.1 mm depth (measured 4.3/3.7 and +4.2/+2.8). Fix: URDF
  loads `config/ur5e_calibration.yaml`; the driver must be launched with
  `kinematics_params_file:=` the same file (workspace README); `kinematics.use_kinematics`
  and a `kinematics_file` parameter in every robot-facing tool (reports record the model,
  the marking node refuses a mismatched `tack_reach.json`); `assembly.json` stores
  `T_static_camera`. Step 4 done: through calibrated TF the ChArUco-only extrinsic
  shows a 0.37° camera tilt (its own rotation uncertainty); refined →
  `notebooks/T_tcp_to_cam_refined.npy`, which differs from the 28 Sep refinement by only
  0.085° - so that one had corrected the ChArUco tilt, NOT the kinematic mismatch, which
  stayed fully in the 28 Sep marks (as the model predicted). NEXT: promote the new refined
  file, re-register, `tack_reachability.py` (new kinematics), mark.
- **Extrinsic horizontal translation / rotation about the optical axis** (unmeasured; two
  solves 18 mm apart): register one untouched part twice with the wrist 180° apart about
  the vertical → `scripts/compare_registrations.py --last 2`; half the horizontal
  difference is the error. About 10 minutes.
- Depth bias vs table shape (+2.3 mm): one `extrinsic_check` round over the touched patch.
- ICP wander on a still scene: base vs standing plate jitter; then an outline
  (depth-edge) term, symmetric ICP, or GICP.
- A pen probe that measures the seam root laterally by sliding into it (mode A+ with the
  pen instead of the camera).

## OPEN — motion (`pen_marking_plan.md`)

- **Holder clearance - DONE 2026-09-29:** the envelope is now from the CAD
  (`world/penholder_assembly.usda`): tube r 17.4 → capsule r 20.5 to 130 mm, flange
  r 48; a square T clears by 15.5 mm (was 1.1 with one r 42 capsule). Earlier "seam
  reachable" results came from the straddled ear opening one side to ~96°. The home
  path now returns to the exact `home_q` (it arrived a wrist turn off after rolled tacks).
- **Holding 1.5 N on metal:** UR's `force_mode_controller` (loaded, inactive) for strokes
  on steel, or a sprung holder. The two-speed descent and gentle stroke are the stopgap.
- Stroke (`stroke_mode:=tack|seam`): first full-seam run on the bench.
- Collision model: tilted table plane, fixture boxes when clamps replace magnets.
- Twin: an adapter from the trajectory action to the Isaac joint-command topic.

## OPEN — perception (`seam_two_modes_plan.md`)

- **Mode A covers only the 5 plate strata** (T/line, corner, butt square, lap, edge). The
  6 curved ones need: registry `tube` / `swept_slab` entries; pipe-on-plate and
  pipe-on-pipe via `curves.ellipse_from_plane_cylinder` / `saddle_from_cylinders` from the
  registered poses plus the per-point cone test of `verify_curved`; new code for box tube
  and curved strip on a plate. Motion: full loops around pipes need large wrist rolls.
  Needed before Phase 9's curved strata.
- **Mode B** (no CAD): PPF no-match, SAM2 per-part masks, region growing + lit-quadric;
  no library save without CAD.
- **Mode A+**: per-part sensor points kept at save, labelled by CAD face, lit-quadric
  refinement, fit-up diagnostic.
- Quality field and DP tack selection (thesis stages 1–3).
- Tacks are 4t long (32 mm on 8 mm plate, `tackrule-0.1`); decide on shorter tacks for
  the cell (ROS parameters) or a `tackrule-0.2` for both projects.

## OPEN — parts, data, paper

- More CAD parts with matching MDF/metal pieces: CAD in `models/`, server PPF entry,
  registry entry (`build_weldgen_registry.py --verify`), one bench cycle.
- Phase 9 real subset (on hold until the metal arrives): `label_real_scan.py`, view
  planner, fiducial-board pose bound (it also replaces the 10 mm `weld_pose_tol_mm`),
  `d435i_measured`, a 3-configuration pilot.
- weld_generator: commit the final ICRA figures/PDF/zip from the desktop; notebook 16;
  a training script on `train_v1`; pin numpy; annotator repeat; lap-overlap citation;
  advisor's written no-welding scope.

## LATER

- Torch instead of pen: force-controlled seam following, the distortion-aware tack
  `order` (already computed).
- MoveIt planning scene when fixtures become real.
- Thesis: the validation chapter (fit-up, contact depth, README §14's error budget).
