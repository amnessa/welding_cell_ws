# Pen-marking the tacks: approach, touch, mark (plan, 2026-09-22)

*The next step after mode A. Input: `welding_tacks.json` (tack point, segment, approach
axis, `tack_no`, `order`) in `base_link`. Output: a pen mark on every tack, and a record
of where the pen actually touched. The touch record is the fiducial-independent check the
whole pipeline is validated against (`seam_two_modes_plan.md`, "Validation"). No arc,
no heat, no puddle: the literature report's thermal arguments (work/travel angles,
gravity, distortion sequencing) do not apply here; its motion hierarchy does.*

## What we have

| Piece | Where | State |
|---|---|---|
| Tack points, segments, approach axis (cleared bisector), numbering | mode A, `welding_tacks.json` | done, on the bench |
| Registered parts as boxes at their poses | `weldgen_objects.json` + `assembly.json` | done: the collision geometry of the workpieces is already here |
| FK / damped-LS IK / RRT-Connect / Catmull-Rom / TOTG timing | `admittance_control/kinematics.py`, `totg_service_node` | proven on the real robot by the drawing stack |
| Phase state machine RETRACT → RRT → DESCENT → DRAWING → ASCENT | `drawing_action_server.py` | proven, but for one orthogonal tool on a plane |
| 1-DoF admittance along the tool axis, path following in X/Y | `cartesian_admittance_controller.py` → MoveIt Servo | exists; tuned for a plane, force target −3 N |
| Approach a point along a ray with IK + joint-jump gate | `move_to_object_node.py` | exists (raw test node) |
| Collision checking in the RRT | `kinematics._is_valid` | **joint limits only** — the gap |
| Pen TCP | `tool_length_m` = 0.19 (move_to_object), `orthogonal_tool_length_m` (drawing) | needs a measured value for the pen |

## The motion, per tack (the four phases, as the literature does it)

```
      P_app = point + d_standoff · approach          d_standoff = 35 mm
  1. transit   RRT in joint space, current q → q(P_app)        collision-checked
  2. approach  LIN along −approach, 20 mm/s, force watched     stop at contact (|F| > F_touch)
                                                               or at point + 3 mm overshoot → "no contact"
  3. mark      hold F_hold along the pen axis (admittance), draw p0 → p1 (the tack length)
  4. retract   LIN along +approach to P_app; next tack: arch transfer if same assembly
```

The pen axis is the tack's `approach` (the cleared bisector: 45° to both plates of a
fillet, so the tip lands in the root). The pen is axisymmetric, so the **roll about the
pen axis is free** — that one degree of freedom is what we spend on reachability and on
keeping the arm elbow-up, never a branch flip.

Contact is the measurement: the tip position at `|F| > F_touch` along the approach line,
compared with the commanded tack point, is the seam-root placement error along the
approach axis. The lateral error comes from the mark itself (photo or caliper against a
scribed reference line). Both go into `tack_marks.json` next to the fit-up numbers.

## Options

### A. Motion-planning backend

| | A1. Extend our stack | A2. MoveIt 2 planning scene |
|---|---|---|
| Collision model | write it: UR5e links + pen as capsules, workpieces as the registered boxes, table as a half-space; capsule–box distance is closed form (the "8-cylinder" model of Liu et al. 2023) | FCL against the registered CAD meshes inserted as collision objects; self-collision from the SRDF for free |
| Transit planner | our RRT-Connect with `_is_valid` upgraded | OMPL RRT-Connect |
| LIN approach | Cartesian interpolation + IK seeded from the previous solution (as the drawing server's DESCENT) | `computeCartesianPath` |
| Force / admittance | our admittance node → Servo (already there) | same node; Servo is MoveIt anyway |
| Execution | `scaled_joint_trajectory_controller` via TOTG, as the drawing stack | MoveIt trajectory execution |
| Cost | ~150 lines of geometry + tests; everything else exists and is proven on this robot | a MoveIt config for UR5e + pen TCP, a planning-scene publisher for the registered parts, a second motion stack next to the drawing server |
| Risk | our checker is only as good as its capsules; fixtures/clamps must be added by hand | integration time; two ways of moving the same robot |

**Recommendation: A1 now, A2 as the upgrade path.** The workpieces are already boxes in
the registry, the arm is six capsules, and the rest of the stack is proven. The moment
clamps and fixtures with real shapes enter the cell, or self-collision becomes a
concern, move transit planning to MoveIt and keep the approach/mark phases as they are.

### B. Approach and touch

| | B1. Force-gated LIN descent | B2. Position-only descent to the computed point |
|---|---|---|
| What | descend along −approach through Servo; the admittance node holds a small force; stop on contact | descend to the tack point exactly, no sensing |
| Measures | the placement error along the approach axis (contact depth) | nothing; a pose error of 5 mm either digs in or draws in air |
| Needs | FT wrench (UR5e built-in), Servo running, controller switch trajectory ↔ forward-position | nothing new |

**B1.** The registration error we measured is 4–11 mm (fit-up). Position-only marking
would put the pen 5 mm into the plate or 5 mm above it. Force gating turns that error
into a number instead of a crash, and the number is the validation.

### C. Marking the tack

| | C1. Dot at the contact point | C2. Short stroke p0 → p1 under admittance |
|---|---|---|
| What | touch, dwell, retract | after contact, follow the seam for the tack length holding F_hold along the pen axis |
| Gives | placement error along the approach | the same, plus the mark shows the seam *direction* and the lateral error becomes measurable along a line |
| Needs | B1 only | the admittance controller generalised from "Z of a plane" to "the tool axis" (it is a frame choice in the node: `target_orientation_axis_angle` + the twist frame) |

**C1 first, C2 on the same run once C1 works.** C2 is the surface following the plan
asked for; it is one parameterisation away from what the node does on a plane.

### D. Orientation, elbow-up, no aggressive turns

* **Branch lock.** Read the configuration branch off the current (elbow-up) pose — the
  signs of `shoulder_lift`, `elbow`, `wrist_1` — and reject every IK solution on another
  branch. All tack poses are solved *before* moving; a tack whose only solution needs a
  flip is retried with another roll (see next), then reported unreachable. Never flip
  mid-sequence.
* **Roll about the pen axis** is chosen per seam, not per tack: sample roll in 15° steps,
  keep the solutions on the locked branch, score by distance from joint limits and by
  the smallest joint move from the previous tack, take the best; the same roll for every
  tack of a seam so the wrist does not twist between neighbours.
* **Continuity.** Seed IK from the previous solution (the drawing server already does),
  gate every step by the joint-jump limit (`max_joint_jump_rad`, as `move_to_object`),
  and time everything with TOTG under lowered velocity/acceleration limits for marking.
* **Arch transfers between tacks of one assembly** (retract to a safe height along the
  approach, joint move at that height, descend) instead of a fresh RRT per tack — RRT
  only from/to home, and it is still collision-checked.

### E. Visiting order for the pen

There is no heat, so the rule's `order` (heat-balance) is not the right order for
marking. Use travel: nearest-neighbour over the tacks with arch transfers, seam by
seam, `tack_no` ascending within a seam. Keep `order` in the file for the day the torch
replaces the pen; the sequence field in `tack_marks.json` says which one was used.

## Decisions (2026-09-22)

A1 (own stack + capsule/box collision model), B1 (force-gated LIN descent), C1 first
(dot), elbow-up by branch lock with the free roll, ascending `tack_no` seam by seam.
Facts from the bench: pen tip ≈ 0.19 m from the flange along tool0 +Z, not
spring-loaded → **F_touch = 1.5 N**; the D435i rides on the pen holder on the side
opposite the connector, its pose in tool0 is the hand-eye calibration
(`notebooks/T_tcp_to_cam.npy`: origin (24, 91, 34) mm, optical axis along the pen);
parts are held by magnets on holders above the table, so the table is a plane the
pen never reaches and there are no fixtures to model yet. RViz: the SEPC stays put;
only the green model cloud moved (a camera-frame leftover, cleared at `save_object`
since 2026-09-21).

## Milestone 1 — the tool model (built 2026-09-22)

`config/pen_tool.json` + `admittance_control/tool_model.py` + `scripts/tool_model_marker_node.py`
(tests: `test/test_tool_model.py`). One description in tool0, metres: pen tip, touch
force, standoff, a conservative envelope (flange adapter box, holder capsule, pen
capsule, camera arm box) and the camera box placed BY THE CALIBRATION, never typed.
`ToolModel.T_tool0_for_tip(tip, axis, roll)` is the pose solver the marking node will
use: tip on the tack, pen along the axis, roll free.

To verify: run the marker node, add `/tool_model/markers` in RViz, jog the arm and
check the orange boxes cover the real holder and the blue box covers the camera; edit
the numbers in the JSON until they do (they are envelopes - err large).

Tip touch-off (refines `pen_tip_m`): jog the pen onto one fixed point of the bench
from 3–4 different wrist orientations, record `tool0` from TF each time; the tip offset
`d` in tool0 is the least-squares solution of `R_i d + t_i = p` for all i (unknown `p`
too) — the classic 4-point TCP. The UR pendant's TCP wizard gives the same number;
either way it goes into `pen_tip_m`.

## Milestone 2 — the collision model (built 2026-09-22)

`admittance_control/collision.py` (tests: `test/test_collision.py`, 7). The arm is six
capsules on the joint frames (`kinematics.ur5e_link_frames`, new), the upper-arm and
forearm tubes displaced off the joint line by the URDF's shoulder / elbow offsets
(0.138 / 0.007 m); the tool is the milestone-1 envelope placed by FK; the scene is the
registered parts as boxes (`boxes_from_parts` on mode A's posed slabs, mm → m) and the
table as a plane. Distances are closed form (segment–segment, golden-section
segment–box, separating-axis box–box, lowest point vs plane); `CollisionModel.is_valid`
is joint limits AND clearance, and `rrt_connect(..., is_valid=model.is_valid)` plans with
it (the RRT gained an optional validity callback; the drawing stack's default is
unchanged). `report(q)` prints the worst pairs in mm. Stated gaps: no full
self-collision (the tool is checked against the base column, shoulder and upper arm;
the branch lock does the rest), no fixtures (magnets), box–box is a yes/no test.

Usage sketch for the next milestone:
```python
tool  = load_tool_model()
parts = posed_parts(objects, registry)            # mode A, from assembly.json
model = CollisionModel(tool, boxes_from_parts(parts), table_z=ground_z, clearance=0.01)
path  = rrt_connect(q_now, q_app, is_valid=model.is_valid)
print(model.report(q_app))
```

## Milestone 3 — reachability, before anything moves (built 2026-09-24)

`config/marking.json` (scan home = branch lock, standoff 35 mm, clearance 5 mm, roll
grid 30°, work angles 0/±10/±20°), `admittance_control/tack_reach.py`,
`scripts/tack_reachability.py` (CLI → `<save_dir>/tack_reach.json`),
`scripts/tack_reach_marker_node.py` (RViz: pen axis per tack, green/red, label
`seam.tack_no t<tilt> r<roll>`, arm capsules at the approach pose). Tests:
`test/test_tack_reach.py` (4).

Branch lock: `/joint_states` is alphabetical; in UR order the scan home is
(−0.34, −1.44, −1.56, −1.55, +1.94, −0.86) and the lock is the signs of lift, elbow
and wrist_2 = (−, −, +). IK (damped LS) is seeded from the previous solution and the
home pose and any solution off the branch is discarded. Per seam every (tilt, roll)
is tried on all tacks ascending; admissible = branch-locked approach pose + tack pose
+ LIN descent all clear, joint step ≤ 69° between tacks; the winner has the largest
minimum clearance, then the least travel. Pose-only checks of the tool envelope run
before IK, so the grid is cheap.

**What the first run on the bench says** (`test_objv2` T-joint, ear leaning 8°):
the obtuse side (98°) is reachable at tilt 0, roll 330°, 10.6 mm minimum clearance;
the acute side (82°) is NOT at 5 mm: the holder body (radius 42 mm, 70 mm up the pen)
reaches 70·sin(41°) − 42 = 3.9 mm from the plates at the bisector, and no tilt improves
a max–min at the bisector. That is geometry, not planning: **a longer pen reach or a
slimmer holder** raises it (each +10 mm of reach adds ~6.6 mm in an 82° corner), or the
run accepts 3 mm (`--clearance 0.003`) knowing the envelope is conservative. The pen
gap at the tack pose is 0.0 (touching), as it should be.

**Two fixes in the shared kinematics that came out of milestone 3's tests** (they
affect the drawing stack too): `rrt_connect` assembled the path in the wrong order
when the trees connected after an odd number of swaps (start and goal ended up
adjacent in the middle of the returned path) - fixed; and its edge check gained an
`edge_resolution` parameter (default 0.05 rad unchanged; the pen planner uses 0.01,
~5 mm of tip travel per sample, so an 8 mm plate cannot slip between two samples).

## Milestone 4 — the marking node (built 2026-09-24, dry run first)

`admittance_control/marking.py` (ROS-free: visit order, collision-aware transit with
shortcutting, the descent as an IK chain along the pen axis, timing, contact depth;
tests `test/test_marking.py`, 6) and `scripts/tack_marking_node.py`:

```
ros2 run admittance_control tack_marking_node.py --ros-args -p dry_run:=true \
    -p extrinsic_path:=<ws>/src/admittance_control/notebooks/T_tcp_to_cam.npy
ros2 service call /tack_marking/plan  std_srvs/srv/Trigger   # from the current joints; RViz: /tack_marking/tip_path
ros2 service call /tack_marking/next  std_srvs/srv/Trigger   # one tack: transit, descend, dwell, retract
ros2 service call /tack_marking/all   std_srvs/srv/Trigger   # the rest, then home
ros2 service call /tack_marking/home  std_srvs/srv/Trigger
ros2 service call /tack_marking/abort std_srvs/srv/Trigger
```

Bench facts it is built on (2026-09-24): `scaled_joint_trajectory_controller` active,
action `/scaled_joint_trajectory_controller/follow_joint_trajectory`; wrench on
`/force_torque_sensor_broadcaster/wrench` at 500 Hz; no Servo. The descent is a
FollowJointTrajectory goal at 20 mm/s tip speed (2 mm IK steps) that the node cancels
at |F − bias| > 1.5 N, the bias re-measured at each approach point over 0.5 s; the
joints at the cancel are the contact record (`tack_marks.json`: tip at contact, depth
along the pen axis, force, or `no_contact` after the 3 mm overshoot). Gates before any
goal: current joints clear and on the locked branch; first trajectory point within 20°
of the current joints; |F| > 8 N at any time cancels. `dry_run:=true` (default) sends
nothing and pretends contact at the registered point, so the whole sequence including
the files can be exercised without the robot.

UR's own `force_mode_controller` and `tool_contact_controller` are loaded (inactive):
the first is the native force control the C2 stroke can use instead of incremental
trajectories; the second is UR's stop-on-contact, whose threshold is fixed and probably
above 1.5 N - kept as options, not used.

The digital twin (Isaac) exposes joint commands as a topic, not the trajectory action,
so the twin run is the dry run with the tip path and the reachability markers in RViz;
the first goals go to the real robot, one tack at a time (`~/next`), with the pendant's
speed slider low.

## Milestone 5 — the first real touch (2026-09-24) and what it found

Four tacks on the `test_objv2` T-joint, `dry_run:=false`, one `~/next` each: tack 0.1 no
contact within the overshoot, 0.2 contact at 1.61 N **+18.6 mm** early, 1.1 contact at
1.72 N **+28.9 mm** early, 1.2 no contact. The motion itself did what it should (transit,
descent, cancel at the touch force, retract, home). The numbers say the registered
assembly is off by centimetres, with a pattern - early on one side at one end, air on
the other side at the other end - that is a yaw plus a height error, i.e. an extrinsic
ROTATION error, not a pose noise.

Root cause found offline in `notebooks/handeye_samples.npz`: the 15 samples are fine
(rotation diversity 110°) and a Park-Martin re-solve fits them to 2–4 mm std; the
stored `T_tcp_to_cam.npy` does NOT fit its own samples (board-in-base spread 32/26 mm
std, 120 mm range, 8° rotation): OpenCV's TSAI solve had failed silently in July. The
re-solved camera is at (59, 78, 32) mm in tool0 with the optical axis along the flange
normal (square bracket), against (24, 91, 34) and a 7° tilt / 26° roll in the bad file.
Tools: `helper/calibration/resolve_handeye.py` (numpy re-solve, residual judge,
leave-one-out, `--tcp-offset` to compose the pendant's TCP, `--write`);
`extract_extrinsics.py` now prints the residual after every solve.

Open point: the capture reads the pendant's ACTIVE TCP; ROS attaches the result to
`tool0`. A non-zero pendant TCP is invisible in the residual but shifts the camera by
that offset (the user's "lens at ~60 mm" vs the solved 32 mm could be a ~28 mm TCP z).
Read it on the pendant (Installation → TCP) and compose it with `--tcp-offset`.

Next: publish the re-solved extrinsic (`pointcloud.launch.py extrinsic_path:=.../T_tcp_to_cam_resolved.npy`),
check the green dot sits on the lens face, re-register both parts, `welding_points`,
`tack_reachability.py`, `~/plan` dry, then `~/next`: the depths should fall from
19–29 mm to the pen-tip level. Whatever remains, consistently, is the TCP offset.

**Second touch, with the re-solved extrinsic (2026-09-24):** both tacks of the
reachable seam ran to the overshoot with no contact; by eye the depth along Z looks
right and the error is in the table plane. Z right + XY off is the signature of a pen
tip that is not on the flange axis: `pen_tip_m` was ASSUMED (0, 0, 0.19). A lateral
tip offset moves every touch in the plane by that offset and leaves the depth alone,
and it is invisible to everything on the camera side. Measure it, do not guess it:
`scripts/pen_tip_touchoff.py` (4-point TCP: the tip on one fixed point from 4+ wrist
orientations, `~/record` each, `~/solve` prints `pen_tip_m` and the RMS). The touched
point `p` is then a bench point known from the robot alone - register a part with a
mark there and the camera path is tested with nothing unknown on the pen side. The
pendant TCP question stays open too (compose with `resolve_handeye.py --tcp-offset`).

**The 6 cm, located (2026-09-24, evening).** The re-solved optical origin is (59, 78, 32)
mm in tool0; the camera body you confirmed in RViz is centred at (0, 90, 45). Same
radius from the flange axis (~98 vs 90 mm), 37° apart in azimuth: the solution is the
camera as seen from a TCP frame ROTATED about the tool axis relative to tool0 (plus a
small shift), which is what a pendant TCP defined for the pen tool does. The residual
cannot see it. `scripts/tcp_offset_probe.py` measures D = T_tool0_tcp live from
`tcp_pose_broadcaster` vs FK and prints the `--tcp-offset` line; compose, republish,
re-register, touch again. If D comes out zero, the discrepancy is real and a recapture
with the pendant TCP set to zero is the fix.

**Resolved (2026-09-24, night).** The pendant's TCP is now the pen tip, calibrated on the
pendant: **(0, 0, 183.78) mm cap off** (192.5 with the cap, not used) - in the tool
model. The July hand-eye capture ran under an unknown pendant TCP, so it cannot be
repaired by composition: **recapture** with the TCP known and pass it to the script
(`extract_extrinsics.py --tcp-offset 0 0 0.18378 0 0 0`), which now composes it and
saves the camera in tool0, and prints the residual judge. A good capture: 15–20 poses,
board 30–50 cm from the camera and filling a good part of the frame, rotations of
±30° or more about two axes between poses, the arm at rest ~1 s before each capture.
Then re-register, `welding_points`, `tack_reachability.py`, `plan`, `next`.

Known error terms on the pen side now: the planner's FK is the nominal UR5e chain,
the robot's kinematics are factory-calibrated (`config/ur5e_calibration.yaml`): the
probe read the tip at (2.9, −0.9, 182.5) against the pendant's (0, 0, 183.78), i.e.
~3 mm at the tip. Follow-up if the marks need better than that: load the calibrated
DH deltas into `kinematics.py`, or take tool0 from TF instead of FK when recording
contacts.

With the tip at 183.78 mm the holder sits 63.8 mm above it and clears the plates of a
90° T by 3.1 mm at best, so the default clearance is now **3 mm** (`marking.json`); the
marking node judges with the clearance recorded in `tack_reach.json`, never its own.

**Capture script vs OpenCV 5 (2026-09-24).** The venv carries `opencv-python 5.0.0`
(shadowing `opencv-contrib-python 4.13`), which dropped `interpolateCornersCharuco`,
`estimatePoseCharucoBoard` and `calibrateHandEye`. `extract_extrinsics.py` now detects
through `CharucoDetector.detectBoard` + `matchImagePoints` + `solvePnP` when the legacy
functions are absent, and solves hand-eye with the numpy Park-Martin when
`calibrateHandEye` is absent (`test/test_charuco_detection.py` renders a board at a known
pose and recovers it to < 3 mm on this OpenCV). Two OpenCV wheels in one venv is
fragile; if nothing needs 5.0, `pip uninstall opencv-python` leaves the contrib 4.13.

**Marking node, bench feedback 2026-09-24 (late):** the transit is re-planned from the
arm's CURRENT joints at `~/next` / `~/home` (the plan froze it at `~/plan` time, and
jogging the arm in between - TCP wizard, freedrive - made the first point 48–60° away);
the dry run keeps virtual joints so its gates follow the pretended motion; a goal the
controller rejects is reported as such, with the usual cause (External Control program
not running after the pendant was used - press Play).

**Milestone 5 result (2026-09-24, night), recalibrated extrinsic + pendant TCP:**
tack 1.2 contact at 1.54 N, **−0.4 mm**; repeat 1.87 N, **−0.6 mm** along the pen axis.
Tack 1.1 no contact within the 3 mm overshoot. By eye the mark is **~8 mm off
laterally**. The depth says the registration is right to a millimetre along the pen;
the lateral error is what remains. Attribution experiments and fixes, in order, in
`todo.md` ("NEXT"): roll test (tool vs world), `pose_jitter_probe.py` (ICP noise
floor), hand-eye recapture with wrist rolls (rotation spread 3.6° → ~9 mm at 0.5 m is
the right size), calibrated FK, then a pen probe that measures the root laterally.
`tack_reachability.py --roll/--tilt` force one orientation for the roll test.

## Milestones

1. **Pen TCP + tool envelope.** DONE 2026-09-22 (see above); the touch-off refinement
   and the RViz check of the envelope remain on the bench.
2. **Collision model + tests.** DONE 2026-09-22 (see above).
3. **Reachability report, no motion.** DONE 2026-09-24 (see above); the bench answer
   is "one side yes, the acute side only at 3 mm" — decide pen reach vs clearance.
4. **Marking node + dry run.** DONE 2026-09-24 (see above); the on-robot dry run
   (`~/plan` with the arm at the scan home, tip path in RViz) is the last check before 5.
5. **Real robot, one tack.** DONE 2026-09-24: contact depth −0.4 / −0.6 mm; lateral ~8 mm
   under attribution (todo.md).
6. **Stroke marking (C2)** on the same assembly; compare the marks with the seam by photo.

## Open questions (they change the work)

* **Controller switching — DECIDED 2026-09-24: none.** The bench check showed no Servo
  node in the running stack (`/servo_node` not found) and no `ros2controlcli` installed.
  So the descent does not go through Servo: it is a slow JointTrajectory on the
  controller that is already there (`scaled_joint_trajectory_controller`), sent as an
  action goal, with a wrench watchdog that CANCELS the goal at |F| > 1.5 N and holds
  the current joints. At 20 mm/s with ~30–50 ms of cancel latency the overshoot is
  under 1 mm. Force *holding* (needed for the C2 stroke) is done the same way with
  short incremental trajectories, i.e. a 1-DoF admittance loop on the JTC at ~20 Hz,
  not with Servo. The admittance node stays for when Servo is launched. Useful checks
  (no `ros2 control` needed):
  ```
  ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers
  ros2 topic hz /force_torque_sensor_broadcaster/wrench
  ros2 action list | grep follow_joint_trajectory
  sudo apt install ros-jazzy-ros2controlcli        # if you want `ros2 control` back
  ```
  The old question, for the record: to see what the driver has, with the robot connected:
  ```
  ros2 control list_controllers                      # active / inactive, and their types
  ros2 param get /servo_node moveit_servo.command_out_type   # trajectory_msgs/JointTrajectory or std_msgs/Float64MultiArray
  ros2 param get /servo_node moveit_servo.command_out_topic
  ros2 control switch_controllers --deactivate scaled_joint_trajectory_controller --activate forward_position_controller
  ros2 control switch_controllers --deactivate forward_position_controller --activate scaled_joint_trajectory_controller
  ```
  If Servo is configured with `command_out_type: trajectory_msgs/JointTrajectory` on the
  scaled JTC's topic, no switch is needed at all; the two phases share one controller.
  Otherwise the marking node switches per phase through `controller_manager`, and the
  check that the switch works is: after the descent, does the arm still accept a
  trajectory. Do it once by hand before it is automated.
* **F_touch = 1.5 N** (decided): UR5e FT noise is ~0.5 N; the descent stops at 1.5 N
  and the low-pass in the admittance node (`force_low_pass_alpha`) must not delay that
  by more than ~1 mm of travel at 20 mm/s (50 ms).
* **Table** is a plane below the holders; `ground_z_m` from the ground cut is the
  estimate, a flange force touch gives it exactly if ever needed. No fixtures (magnets).

## RViz: parts moving when the robot moves

Everything that should stay put is published in `base_link` (the SEPC, the seams, the
tacks) with the fixed frame `base_link`; those cannot move with the arm. What does move:
anything latched in the **camera** frame — the model and scene clouds after tracking
stops, the detection markers, and the crop box drawn in the object frame whose TF stops
updating — because RViz re-places an old message with the current transform. The
model/scene clouds and the crop box are now cleared at `save_object` (2026-09-21). To
pin down what is left:

```
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame   # must change as the arm moves
ros2 topic echo /perception/icp/static_env --field header.frame_id  # must be base_link
```

If the orange SEPC itself moves, the robot description's `base_link` and the ICP node's
`static_frame` are not the same frame (check `ros2 run tf2_tools view_frames`); if only
green/white/marker displays move, they are camera-frame leftovers and the remaining
publishers (detection markers) should be moved to `base_link` or cleared on save.
