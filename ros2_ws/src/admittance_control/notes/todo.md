# Open work, both projects (2026-10-08)

*Open items only. What was done, why and with which numbers lives in the README (§14
placement accuracy, §15 multi-view refinement, §16 camera depth calibration), in
`notes/thesis_notes.md` (the running log of experiences, newest first) and in the plans:
- `realtime_fp.md` (FoundationPose tracking);
- `aps_transit_plan.md` (the transit planner);
- `multiview_refine_plan.md`;
- `seam_two_modes_plan.md` (perception);
- `pen_marking_plan.md` (motion);
- `curved_parts_print_list.md` (parts to print);
- `weld_generator/notes/dataset_plan.md` + `phase9_plan.md` (dataset).*

## Where we are

Mode A runs end to end on the robot, by gestures, and the pen marks every tack within
**2.3–2.7 mm** (2026-10-06, possibly within the measurement error; ~8 mm on 24 Sep).
The chain:

1. calibrated kinematics; camera depth Tare'd against the robot; extrinsic tilt refined;
2. register (desktop: SAM2 → PPF → FoundationPose);
3. **track the part while it is moved: FoundationPose on the laptop's GPU** (25–29 Hz);
4. **save:** robust mean of the tracker poses at rest, then **ICP on 5 live clouds**;
5. `~/refine_pose`: 4 close views, online camera-offset correction, ISO 5817 fit-up check;
6. seams and tacks (mode A, D4 rule, tackrule-0.1);
7. reachability; transits by **OMPL's AnytimePathShortening (C++)**, spline, TOTG;
   descents with a roll fallback;
8. force-gated touch; dot or stroke.

## Decisions

- **2026-10-02: no touch sensing.** The method is vision-only. Touches are used only to
  calibrate (pen TCP, table plane, Tare distance) and as the independent reference that
  measures the error (`touch_probe.py`).
- **2026-10-02/06: tracking is FoundationPose's** (`tracking_source: fp`, the default).
  - The desktop registers; the laptop tracks.
  - ICP keeps the stationary work: `run_icp`, the ICP at `save_object`, `refine_pose`.
  - No Kalman fusion.
- **2026-10-06: transits by APS** (`transit_planner: aps`, the default), with RRT-Connect
  as the fallback.

## FOUND IN THE THESIS REVIEW (2026-10-20)

The line-by-line review of both projects for the thesis (`thesis/notes/review_2026-10-20/`,
index `thesis/MATERIAL_MAP.md`). Nothing in the code was changed.

**Code (the cell)**
- [ ] **`kinematics.ur5e_jacobian` has doubled angular rows** (`kinematics.py:296-303`): the rotation
  difference $R(q+\varepsilon)R(q-\varepsilon)^\top\approx I+2\varepsilon[\omega]_\times$ is divided by
  $2\varepsilon$, not $4\varepsilon$. Reproduced 2026-10-20: every angular entry is exactly 2× the true
  one; the linear rows are right. Effect: the camera aim turns at **half** the commanded rate
  (K_eff = 1.25 /s, cap 7.5°/s, roll gain 0.75 /s) and its optical centre drifts up to ~8 mm/s against the
  P pull; `ik_solve` converges more slowly but to the right pose. The aim test checks with the same
  Jacobian (`test_tool0_twist_keeps_the_optical_centre_still`), so it cannot catch it.
  Fix: divide by $4\varepsilon$ (or use $\log(R(q+\varepsilon e_i)R(q)^\top)/\varepsilon$); to keep the
  behaviour tested on 2026-10-15/16, halve `gain_per_s`, `roll_gain_per_s`, `max_rate_deg_s` and
  `max_acc_deg_s2`; add a test of the Jacobian against finite differences of the FK; re-test on the robot
  (record a bag). Then update Ch. 3 §3.4.2 (the TODO there) and Appendix A. THESIS_PLAN decision #11.
- [ ] Camera images are stamped with the ROS clock when `realsense_camera_node.py` publishes them, not
  with the exposure time; every "pose at the image stamp" inherits the offset. Measure it once (the aim's
  latency test) or use the driver's hardware stamp.
- [ ] `multiview_refine._thickness_mm` takes a part's smallest bounding-box extent as its thickness: for a
  tube that is its diameter or length, not its wall, so the ISO 5817 fit-up warning for tube pairs uses
  the wrong gauge. Use the registry's gauge.
- [ ] Stale comments and docstrings (no behaviour change): `camera_aim.py` says 50 Hz (125 Hz);
  `collision.py:20-22` says box-box inflates by the clearance (it does not); `config/pen_tool.json`
  `_comment` says the planner uses the nominal chain (calibrated since 2026-09-29); the `kinematics.py`
  header "RRT-Connect Planner"; the `package.xml` description; `test_multiview_touching_parts.py`
  docstring (lap rule is off); `srv/ComputeTOTG.srv` default 0.1 (node 0.02); `marking.py` 33 vs 34
  boxes; `marking.aps_path` docstring ("Python model"); `tack_marking_node.py` docstring (bias at the
  approach point: it is at the departure point); `MarkingConfig` defaults (10 mm, 15°) differ from
  `marking.json` (3 mm, 30°); `contact_error.py` docstring ±15 mm vs `span` 25 mm, and the script is not
  installed; `scripts_in_foundationpose/server.py` comment `threaded=False` vs code `True`;
  `scripts_in_foundationpose/foundationpose_bridge_node.py` is an old copy (439 lines of diff): delete
  or mirror.

**Code (weld_generator)**: the P1 defects in `thesis/notes/weldset_code_review_2026-10-20.md` (exterior
flag on thin stacked plates, interpenetrating parts, the coplanar-arm comparison, the fixture twins'
yaw, the grooved butts) and its "smallest fix set" (5 steps, then regenerate and re-run the affected
strata on the desktop). THESIS_PLAN decision #10.

**ICRA submission**: 12 statements that do not match the code or the final batch (the same note, last
section). Fix in the camera-ready if accepted; do not carry them into the thesis.

**Thesis: answers only the author has (ASK)**
- [ ] the pen holder's printer, filament and infill; the pen (make, tip); is the clamp rigid?
- [ ] the parts' filament (PLA?); their measured dimensions (callipers); the real MDF thickness; which
  base plate each experiment used;
- [ ] the UR5e payload/CoG setting with the tool; CPU and RAM of both machines; the controller version;
- [ ] how the true weld root is measured for the mark error (instrument, resolution).

**Thesis: on the desktop (DESKTOP)**
- [ ] FoundationPose's hypothesis count: `estimater.py` `make_rotation_grid(min_n_views=40,
  inplane_step=60)` (the notes say 252 = 42 × 6; the paper uses 504);
- [ ] the Docker image's versions (CUDA, PyTorch, FoundationPose and SAM2 commits, OpenCV contrib), the
  Dockerfile and run scripts;
- [ ] WeldSet corpus sizes and yields per stratum, twin-gate statistics, the final batch numbers.

**Literature to fetch** (D_papers Part 4, "fetch first"): MediaPipe Hands + the Gesture Recognizer model
card; Kirillov 2023 (SAM); Chen & Medioni 1992 and Low 2004; Tsai & Lenz 1989, Park & Martin 1994;
Keselman 2017 + Intel's D400 depth white paper; Kunz & Stilman 2012; Geraerts & Overmars 2007 and
Raveh et al. 2011; the ISO/EN tack-weld clauses (JASS 6, EN 1011-2); Tsai et al. 1999; a touch-sensing
source; MegaPose (Labbé 2022), CNOS, nvdiffrast; Hartley 2013 (rotation averaging); Gelfand 2003 /
Censi 2007 (ICP observability); Hogan 1985 (impedance). Verify every bib field against the PDF.

## NEXT (2026-10-08): CAD parts for the strata MDF cannot make

The plate strata (T/line, corner, butt square, lap, edge) are MDF cuts. The curved strata
(T/circle, T/ellipse, T/saddle, T/rounded_rect, T/swept_path, butt arc), and butt grooved
(excluded for metal), are 3D printed. The list, sizes and print notes are in
`curved_parts_print_list.md`.

1. Prune the CAD library: near-duplicates and non-weld parts (the list names candidates).
2. Draw the parts (mm, watertight); export PLY (+ STL to print) from the same CAD.
3. Print, then measure the printed dimensions and record them.
4. Into the cell, per part:
   - the CAD into `models/` and the server's `CAD_DIR` (the same name on both);
   - the PPF library rebuilt;
   - a registry entry;
   - one bench cycle.
5. **Mode A for the curved strata** (`curved_seams_plan.md`): steps 1–3 done on
   2026-10-09.
   - All six printed parts are in the registry (verified to 0.06 mm).
   - `compute_seams` computes their seams with fit-up and per-point approach.
   - Tacks on curved seams are planned with a roll per tack. In front of the robot: C1,
     E2 and RR1 4/4; SP3 4/6 (concave lobes); S3 on R2 3/4 (far side).
   - Pipes and bands are boxed to within ~1 mm.
   - Step 4, first bench (perception only), 2026-10-09: PPF named every part (R2 by a
     thin 0.040 margin). Mode A did not run (an install-list bug, now fixed). Also
     fixed: re-registration switching to the plate, ICP refused on a pipe's arbitrary
     spin, silent near misses.
   - **Deploy the updated `scripts_in_foundationpose/fp_server.py` to the desktop**
     (`object_name` on re-registration).
   - 2026-10-14, C1 and E2 marked:
     - the far marks are 12–14 mm up the part's wall;
     - the contacts say the registered tube is 8–9 mm off sideways and the plate
       1–3 mm high (`scripts/contact_error.py`);
     - **next: step 5** (multi-view refinement for curved parts).
     - Also fixed: APS "Invalid start/goal" (margin), the Python fallbacks (E2 plan
       478 s → 18 s), no table in the collision model (the touched plane is now used),
       fixtures in `marking.json`, runs archived in `marks_history/`.
   - **Fill `marking.json` `fixtures`** (V-blocks, holders): 3–4 pen-tip points on each
     fixture's top, read off the pendant.
   - Transits round a pipe swung up to 0.9 m off (hitting things ~75 cm out). Fixed:
     - 100 mm departure points;
     - the tool's straight (or bowed) Cartesian line tack to tack;
     - the planned path driven instead of re-planned;
     - APS 3 s.
     - E2 tip paths are now 135–211 mm tack to tack, the same every run.
   - RR1 marked (2026-10-14): all within ~3 mm (two close but not touching the root,
     one 3 mm out on the plate, one 2 mm up the wall), clearly better than the round
     pipes' 8–14 mm.
   - **Multi-view (step 5) is needed for curved parts**: one view pins a round part's
     pose poorly.
   - SP3 marked (2026-10-14): reach 4/6 (the concave sides, as predicted); one no
     contact, three 9.5–12.7 mm up the band. Its contacts are 0–8 mm off: a rotated S,
     and the 20 mm stroke climbing the real wall. **Measure accuracy in
     `stroke_mode:=dot`.**
   - S3 on R2 marked (2026-10-14): 4/4, 1–7 mm and one 11 mm (lifted off). **Step 4
     done** (the bench table is in `curved_seams_plan.md`).
   - **Step 5 built (2026-10-14):** the refinement and the view planner use each part's
     true shape (`part_shapes.py`).
     - Synthetic: C1, E2, S3 on R2 and RR1 all come back to within 0.1 mm, where the old
       boxes refused E2 and S3 on R2.
     - C1 needed only `refine_pose`.
     - **Next: bench it**: `refine_pose` before `welding_points`, marks in
       `stroke_mode:=dot`, C1 and E2 first.
     - 2026-10-15 (SP3): first try found no seam because the plate CAD was replaced
       after the registry was built. Rebuilt; a stale entry is now refused loudly.
       `mv_max_correction_deg` raised to 5.
     - SP3 with `refine_pose` (2026-10-15): fit-up −0.9..10.4 → 0.0..0.3 mm. Marks: 2 on
       the root, one 4.4 mm (was 9.5–12.7 mm without multi-view); contacts 2.8–4.2 mm
       early along the pen (was 0–9.4 mm). The first tack hit a fixture: enter
       the fixtures in `marking.json`. The user is testing multi-view further before
       the other todos.
     - E2 (2026-10-15): `refine_pose` refused for overlap; the acceptance rules changed
       (the user's rule, "apply a correction within the limits"):
       - the overlap must not get worse than the saved assembly's;
       - a pipe's spin is not judged;
       - the bore is hollow.
       Replayed, it is accepted and lowers E2's seam 4.5–5.3 mm onto the table-flush plate,
       which the contacts support if the plate lies on the table. **Re-run E2 with
       `refine_pose`, then mark in `stroke_mode:=dot`.**
     - A cut pipe's spin is unobservable to single-view registration (E2 36° off,
       2026-10-15). `welding_points` now settles it on the cut (`settle_cut_spins`).
       E2 with `refine_pose` + the settled spin: marks 2–3.2 mm, 3–4 mm on the tilted
       side (single view: 3.7–6 mm; step 4: up to 14.4 mm). Maybe
       later: the same inside `refine_pose`.
     - The camera error the views estimate is consistent run to run: d ≈ (0–2, 3,
       −3.3..−4.4) mm (SP3, E2), ~5 mm. Applying it to the extrinsic file
       (`scripts/selfcal_extrinsic.py`; first mark or remove the bad 2026-10-06 15:36
       history entry) would take it out of every single-view registration too.
     - **After replacing any CAD in `models/`: `python3
       scripts/build_weldgen_registry.py --verify`** (and the desktop's PPF library).
   - `contact_error.py` reports bands per contact (no fit yet).

## Benchmarks to run (for the thesis)

- **APS vs RRT-Connect** (`aps_transit_plan.md` step 5): `scripts/benchmark_transits.py` on
  a real session, every transit of the plan + home.
  - Planners: RRT-Connect + shortcut + spline, against APS at 0.5 / 1 / 2 s with 2 / 4
    threads.
  - Per transit:
    - planning time;
    - weighted joint travel;
    - tool-tip path length;
    - how far the tip strays from the straight line;
    - the smallest clearance;
    - TOTG duration;
    - validity calls.
  - Output: a table + JSON, and an entry in `thesis_notes.md`.
- **FoundationPose vs ICP tracking** (`realtime_fp.md` step 5): `pose_jitter_probe.py` on
  `/perception/fp/pose` and `/perception/icp/refined_pose` (still part, slow push): noise,
  delay, drift.
- **PPF pose vs FoundationPose pose** on 3–5 scenes: accuracy and time (perception side,
  the 1 Oct meeting).
- **Single view vs multi-view, per stratum** (the curved bench): marks by hand in
  `stroke_mode:=dot`, plus the pen's contacts (`scripts/contact_error.py`, `marks_history/`)
  for each joint. 2026-10-15 so far: E2 3.7–6 → 2–3.2 mm, SP3 9.5–12.7 → 0–4.4 mm. Also the
  multi-view's own numbers: the correction, the camera error d, the view spread.
- **For the thesis, collect the comparisons the project already made** (the user,
  2026-10-15, "not now"):
  - FoundationPose vs ICP tracking;
  - APS vs RRT-Connect (time, path length);
  - single vs multi-view;
  - box vs true-shape neighbours (synthetic: E2 refused → 0.03 mm);
  - the C++ vs Python collision model (590×, E2's plan 478 → 18 s);
  - the Cartesian transit vs APS (tip path 0.14–0.21 vs 0.26–1.6 m).
  `thesis_notes.md` holds most of the numbers.

## LATER, but important: measure everything on this side of the pipeline

The FoundationPose dockers are measured separately (`realtime_fp.md`, the perception
notes). Here: every stage from the received pose to the pen mark, measured, not
estimated, so the thesis can report them. Per stage:

| stage | what to measure |
|---|---|
| bridge reply → detection | delay |
| tracker pose → `base_link` | TF lookup time; rate actually used |
| `save_object` | robust mean; save-time ICP time per cloud, fitness, rmse, correction |
| `refine_pose` | view planning time; motion time per view; capture time (16 frames); preprocessing; refinement rounds and iterations; d estimate; total |
| `welding_points` | seam + tack time |
| `tack_reachability` | time, IK calls, collision checks |
| `~/plan` | per transit: planner, time, cost, validity calls, path points; descent checks; TOTG time |
| execution | per transit duration, descent / contact time, the whole cycle |

- **Quantities:** time (wall and CPU), peak memory (RSS), rmse / fitness wherever ICP
  runs, and point counts.
- **Complexities:** how each stage scales with points, parts, views and transits, both
  measured and stated.
- **How:** one timing/metrics logger shared by the nodes, writing JSON per run, plus a
  script that tabulates runs.

## Multi-view refinement (`multiview_refine_plan.md`)

- **Anchor the camera offset to the pen-measured table** (the main accuracy lever left).
  - On the 2026-10-06 capture, the table per view against `table_plane.json`:

    | | views 0–3 | spread |
    |---|---|---|
    | raw | +2.5 / −1.6 / +2.6 / +5.3 mm | 6.9 mm |
    | with R_v·d taken out | −1.1 / −3.9 / −1.6 / −0.2 mm | 3.7 mm, mean −1.7 mm |

  - So the online correction is real; dropping its vertical part would make it worse.
    But it leaves the scene ~1.7 mm low.
  - Fix: add the table points (every view sees the table) as an observation, with the
    pen plane as the known geometry, in the joint estimate of d.
  - A looser prior does not help: 3 / 8 / 15 mm changed the result by < 0.5 mm
    (`mv_prior_sigma_mm/deg` are parameters now).
- **Self-calibration bench check:**
  1. 3 or more runs from **different** part arrangements;
  2. `selfcal_extrinsic.py --write`;
  3. promote;
  4. a run with d near 0;
  5. `touch_probe` and the marks.

  - Key the history by scene (assembly hash).
  - The 2026-10-06 15:36 entry in `notebooks/selfcal_history.json` came from
    interpenetrating saves, d = (−5.4, −2.3, −1.1): **remove or mark it.**
- **The base's slide in its own plane** varies 1–3 mm between runs while reported as
  measured (grazing, correlated edge points). A noise model that grows for steep points
  would make the σ honest.
- **Turns vs one combined solve:** keep 2 rounds; switch only if the rounds oscillate or
  depend on the order (watch `rounds_run`).
- **Overlap rule and fit-up gap for curved parts:** the distance to the registry's
  `tube` / `swept_slab` surfaces. Together with mode A for the curved strata.
- **A refusal at `save_object`** when `check_registration` fails.
- **FoundationPose per view** as a second measurement in the refinement (plan step 8).
- **Planner transits on APS** (`multiview.plan_views` still uses RRT-Connect).
- **To try:** viewing distance 0.32–0.35 m; a 30° elevation where reachable; tune
  `mv_max_relative_mm`, the 4° limit, `online_selfcal_min_mm`.

## Accuracy: small items

- **Fixture or clamp the parts.** Magnet-held parts can move ~1 mm between scan, marking
  and touches.
- **The ear's foot measured 0.3–2.9 mm above the base,** consistently, with an ISO level C
  warning. Check with a feeler gauge: real, or a camera effect near the contact?
- **Pen length:** the bare table reads +1–3 mm from the plane at some spots. Re-check the
  4-point TCP (it changes only `pen_tool.json` and the table plane).

## OPEN — motion (`pen_marking_plan.md`, `aps_transit_plan.md`)

- **Box–box clearance** (APS plan step 8). A tool box (the camera body) against a part box
  is only an intersection test (0 / ∞), in both the Python and the C++ models: no warning
  margin. On 2026-10-06 it refused a descent ("0.0 mm"), which the roll fallback now works
  around. Add an exact box–box distance (or FCL) to both, with the equivalence test kept
  green.
- **The 29 Sep tack-0 refusal** ("descent clearance 0.0 mm (overshoot)") is probably the same
  box–box case; check that it plans now with the roll fallback.
- **Re-test SP3** with the wrist_3 cable window and the overshoot cut at the table
  (2026-10-16). Offline the same scene now plans all four reachable tacks, with wrist_3
  within −174°..0 (it was −202° and −287°). Confirm that tacks 3 and 4 now touch the
  part, and that tack 2 marks. If a false contact comes back inside the window, log the
  force vector during a descent in the air (only |F| is recorded now).
- **Holding 1.5 N on metal:** UR's `force_mode_controller` (loaded, inactive), or a sprung
  holder. Strokes still stop with "pen lifted off" on some tacks.
- **Marking overshoot:** `overshoot_m` 5–6 mm until the camera offset is anchored.
- Stroke `seam` mode: first full-seam run on the bench.
- Collision model: the tilted table plane and `fixtures` are in (2026-10-14); enter the
  bench's V-blocks and holders in `marking.json` `fixtures`.
- Twin: an adapter from the trajectory action to the Isaac joint-command topic.

## OPEN — perception (`seam_two_modes_plan.md`, `realtime_fp.md`)

- ~~**Mode A covers only the 5 plate strata.**~~ Done: curved seams from the registered
  poses, steps 1–5 of `curved_seams_plan.md` (2026-10-09 .. 10-15).
- **FoundationPose tracking, small items:**
  - ~~does RViz load our PLY meshes for `/perception/fp/marker`?~~ Yes (the user,
    2026-10-16);
  - `use_roi` (off): does the crop raise the rate?
  - symmetric parts (tubes) need `SYMMETRY_INFO` on the server, or the pose may spin about
    the axis.
- **Mode B** (no CAD): PPF no-match, SAM2 per-part masks, region growing + lit-quadric.
  **Probably dropped** (the user, 2026-10-15: "too much hassle and not much improvement
  for a robotics-oriented thesis"; not yet decided).
- **Mode A+:** per-part sensor points kept at save, labelled by CAD face, lit-quadric
  refinement, fit-up diagnostic.
- Quality field and DP tack selection (thesis stages 1–3).
- **Tack length:** 4t (32 mm on 8 mm plate). Shorter tacks for the cell, or a `tackrule-0.2`
  for both projects?
- **The professor's "walking points" seam method** (1 Oct meeting): two points walk toward
  each other over their own surfaces; where they stop lies near the seam; many pairs fit
  the seam. Prototype it outside the pipeline, step by step and visualised; investigate
  why the edges attract (a proof).
- **Laser pointer** next to the pen, to show a tack before touching it (the 1 Oct
  meeting).

## OPEN — tack ordering by a rough heat simulation (asked 2026-10-16)

- **What.** Choose the order of the tacks by simulating, roughly, the heat each tack puts
  into the parts. That replaces the fixed rule of the weld generator's `tackrule-0.1`
  (D38): both ends of a seam first, then bisection, same-class seams interleaved
  round-robin "so heat alternates". `compute_tacks` writes that as each tack's `order`,
  and the marking node visits the tacks in it.
- **Why.** A tack laid next to one that is still hot adds heat where it already is. The
  region grows and shrinks unevenly, and the gap opens or the part pulls out of line. The
  rule's heuristics spread the heat for straight seams on plates. A simulation does it
  for any assembly (pipes, saddles, several seams close together) and gives a number to
  compare orders by.
- **Rough model, to start.**
  - Each tack = a heat source of energy Q (∝ tack length × heat input per mm; nominal
    values, since the cell's pen puts in none) at its `point_mm`.
  - Temperature anywhere = the sum over the tacks laid so far of an analytic cooling
    kernel. For a thin plate: 2-D diffusion
    T(r, t) = Q / (4π k d t) · exp(−r² / 4αt), times a loss term for cooling to air.
    r is the distance over the part, not through the air. Along the seam, and the 3-D
    distance between seams, will do at first.
  - The clock: each tack's weld time, plus the transit between tacks (the marking plan
    already times them).
- **Choosing the order.**
  - Greedy first: next = the tack whose spot is coolest when the torch would get there.
  - Then a cost to compare with: the highest temperature at a tack when it is laid; a
    heat imbalance about the part (a distortion proxy); and travel time.
  - Then search (beam search) if greedy is clearly beaten.
  - Option to keep the rule's must-have: both ends of a seam before its middle, to fix
    the gap.
- **Output.** An `order_heat` beside `order`, and a switch for which one the marking node
  follows.
- **Compare** with `tackrule-0.1` on the bench assemblies (plate T, E2, S3/R2, RR1),
  using the cost above.
- **Check the kernel** once against a small transient FE run on one plate (e.g.
  scikit-fem), so "rough" has a stated error.
- **If it is adopted, it may become `tackrule-0.2` for both projects** (see Tack length
  under perception). The dataset's `order` must stay reproducible.
- **Thesis:** a weld-sequence section (heat-aware ordering vs the rule).

## OPEN — human–robot interaction

- **Finger-pointing camera aim, before the first capture** (asked 2026-10-14; BUILT
  2026-10-15, works on the robot; measurements pending).
  - **Built:** `admittance_control/camera_aim.py` (the control law, 14 tests including a
    closed loop on the UR kinematics), `scripts/camera_aim_node.py`, and `/gesture/aim` in
    the gesture node. It was tested against a fake controller manager and arm: 240 px
    (21°) → 10 px in ~3.5 s, centre drift ≤ 4 mm, the joints handed back 0.6 s after the
    hand drops, and Ctrl-C mid-turn stops it. README §E has the details.
  - **On the bench:**
    1. ~~dry run, then live~~: works on the robot (2026-10-15). Everyday start, both nodes
       in one terminal: `ros2 launch admittance_control gestures.launch.py`
       (`aim_dry_run:=true` to only log, `aim:=false` for gestures without the aim);
    2. measure time-to-aim, overshoot and the hand-lost stop (thesis);
    3. check the image's stamp and `/joint_states` share a clock. If they don't, the node
       says so once ("latency not compensated") and still works, with a little
       overshoot.
  - **To fix: the wrist buzzes when the error is small** (seen on the robot 2026-10-15,
    diagnosed 2026-10-16).
    Large turns are smooth; near the centre the wrist makes a buzzing sound.
    - **Diagnosed 2026-10-16** from a bag (`ros2_ws/rosbag2_2026_10_16-13_49_03`: 45 s,
      32 s of aiming; `/joint_states` 500 Hz with effort, the commands at 50 Hz):
      - **It is the 50 Hz staircase.** The wrist joint velocities peak at 49–51 Hz (30–50×
        the noise floor), and the efforts (motor current) at 50 and 100 Hz. ~8 ms after
        every command step the effort spikes (−0.4 against a ±0.05 background) and the
        velocity overshoots its new value by ~25 %: one kick every 20 ms.
      - **Loudness follows the size of the jump between commands, not the speed.** The
        correlation of the 100 Hz effort with the command step is 0.84, with the
        commanded speed 0.13. During aiming it is 11× the idle level (trajectory
        controller). Cruising at the 15°/s cap is quiet: small steps. Starts, stops and
        reversals are loud: acceleration × 20 ms = big steps.
      - **Why near the centre.** That is where it starts and stops. With the 1280-px
        image the 5 % deadband is 64 px ≈ 4°, and engaging jumps the target from 0 to
        ~8°/s (2.5 /s × 0.8 × 4°). Hand tremor at the edge restarts it, and each restart
        is a full-acceleration burst.
      - **Ruled out:** the I/D question (this is not the loop oscillating); tiny
        commands (the hold-still windows, |cmd| < 0.002 rad/s, are as quiet as idle).
    - **Fixes, in order:**
      1. ~~**Smooth the stream to 500 Hz.**~~ Steps:
         - 2026-10-16: `rate_hz` 50 → 125 (2.5× smaller steps), "definitely improved" by
           ear.
         - Then BUILT: the C++ `aim_rate_relay`. It takes the aim node's 125 Hz rates on
           `/camera_aim/joint_rates`, ramps linearly between them over the measured input
           period (8 ms of delay) and streams to `forward_velocity_controller` at 500 Hz.
           Against the fake cell: 500 Hz (median 1.99 ms), the largest step between
           commands 0.0017 rad/s (the 50 Hz bag: p95 0.009). A `kill -9` of the aim node
           mid-turn: the relay ramps the arm to zero within ~0.1 s (it used to keep the
           last rate). `gestures.launch.py` starts it; the aim node refuses to start
           without it.
         - **CONFIRMED on the robot** (bag `rosbag2_2026_10_16-14_09_33`, compared with
           `scripts/aim_bag_report.py`):
           - commands 50 → 500 Hz;
           - the median jump between commands 0.00104 → 0.00018 rad/s, and the p95
             0.0089 → 0.0009;
           - the buzz index 3.4 → 1.7× idle;
           - the 50.8 Hz line (9× its neighbours) gone, with no new line.

           The 1.7× left is broadband: the motion itself.
      2. **Soften the engagement** (still open; may matter less now). Start the target from zero at the deadband edge and
         let it grow, instead of jumping to ~8°/s. Add a minimum stop time (~0.5 s) so
         tremor cannot restart it at once. Maybe a smaller deadband in degrees for the
         1280-px image.
      3. Alternatives to (1) if the relay disappoints: short spline segments streamed
         to `scaled_joint_trajectory_controller` (interpolated at 500 Hz, no controller
         switching), or MoveIt Servo.
  - **What.** When the gesture node starts, with the robot at home, `Pointing_Up` is a
    live AIM mode: the robot turns the wrist camera to follow the fingertip. The
    operator points the camera at a part in real time. The first `ILoveYou` (capture)
    freezes the frame and hands over to what exists: the FoundationPose bridge, and the
    dwell cursor on the frozen frame.
    - Once a frame is frozen, `Pointing_Up` means the cursor again, so no new gesture is
      needed: the mode follows whether a frame is frozen.
  - **Motion: orientation only.** The camera turns about its own optical centre and its
    position is held.
    - Aiming the optical axis takes 2 rotations (pan, tilt). The third, roll about the
      optical axis, is the over-actuated one: hold it at the home value, so the image
      stays upright.
  - **Control: image-based visual servoing, a filtered P loop, not a full PID.**
    - Error = the index fingertip's pixel (MediaPipe landmark) minus the image centre.
      That error gives an angular velocity about the camera's x and y axes.
    - A gain, a deadband (~5 % of the image, so it stops when roughly centred), a speed
      cap (~15°/s), and acceleration/jerk limits.
    - No integral term: the camera-to-command latency (~70–100 ms) makes it wind up and
      overshoot. No derivative: the landmark is noisy.
    - Smooth the landmark itself: a One-Euro filter at the 30 Hz frame rate (low lag
      when it moves, steady when it is still).
  - **Execution: a streaming interface, not the trajectory action.** Chosen: the velocity
    controller with our own Jacobian. It needs no MoveIt Servo config, and our collision
    model and branch lock run on every cycle anyway. The options were:
    - MoveIt Servo (twist in, joint commands out, with its own singularity and collision
      slowdown and Ruckig/Butterworth smoothing) on the UR driver's
      `forward_position_controller`; or
    - the driver's `forward_velocity_controller` with our own Jacobian solve (6×6, the
      calibrated kinematics).
    - Switch controllers on entering and leaving AIM. `scaled_joint_trajectory_controller`
      must be active again before `refine_pose` and marking.
  - **Safety.**
    - Deadman: no fingertip for 0.3 s → decelerate to a stop. Any other gesture stops it
      too.
    - Excursion limits from home, e.g. ±35° pan and ±25° tilt.
    - Every commanded pose checked with the C++ collision model (µs) and the branch lock.
    - The pendant's speed slider still scales everything.
  - **After it.**
    - The capture happens from the aimed pose: `T_base_cam` comes from TF at the image
      stamp, as now.
    - Decide whether `refine_pose` and marking start from there or from `home_q`. Both
      already plan from the current joints.
  - **Later, option B:** aim where the finger POINTS (a 3D ray from the hand landmarks,
    with depth, onto the table), not where the fingertip is.
  - **Thesis:** a short HRI section. Measure time-to-aim, overshoot, and the hand-lost
    stops.

## OPEN — parts, data, paper

- **Phase 9 real subset** (on hold until the metal arrives): `label_real_scan.py`, the view
  planner (`multiview.py` can serve), a fiducial-board pose bound, `d435i_measured`
  (README §16), a 3-configuration pilot.
- **weld_generator:** commit the final ICRA figures/PDF/zip from the desktop; notebook 16; a
  training script on `train_v1`; pin numpy; annotator repeat; lap-overlap citation; the
  advisor's written no-welding scope.

## LATER

- **Torch instead of pen:** force-controlled seam following, and real heat for the tack
  order (the rule's `order` today; heat-simulated: OPEN above).
- **MoveIt planning scene** when fixtures become real.
- **Thesis:** the validation chapter (fit-up, contact depth, README §14's error budget,
  `thesis_notes.md`).


## important
path finding algorithm fins a path which has sections go through the table in actual robot this causes collision and robot stops.