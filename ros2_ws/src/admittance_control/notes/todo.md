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

## NEXT — FoundationPose real-time tracking

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
NEXT: restart perception, re-register, `check_registration.py` OK, reachability, mark.
Not done: a refusal at save_object when the check fails.

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
