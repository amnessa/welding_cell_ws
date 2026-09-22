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

## Milestones

1. **Pen TCP + tool envelope.** DONE 2026-09-22 (see above); the touch-off refinement
   and the RViz check of the envelope remain on the bench.
2. **Collision model + tests.** DONE 2026-09-22 (see above).
3. **Reachability report, no motion.** For the live `welding_tacks.json`: every tack's
   pose on the locked branch, the roll chosen per seam, joint moves between tacks,
   collision margin at P_app and at the tack. Published as markers (pen axis at every
   tack) so RViz shows the plan before the robot moves. This alone answers "can the arm
   reach the seams elbow-up" on the current fixture.
4. **Dry run in the digital twin** (`digital_twin_pointcloud.launch.py` + Isaac): the
   full four phases with the twin's contact-less descent stopping at the point.
5. **Real robot, one tack:** transit, force-gated descent, dot, retract, and the first
   `tack_marks.json` entry with its contact depth. Then all six.
6. **Stroke marking (C2)** on the same assembly; compare the marks with the seam by photo.

## Open questions (they change the work)

* **Controller switching.** Transit runs on `scaled_joint_trajectory_controller`; the
  admittance node commands Servo, which needs `forward_position_controller` (or the
  Servo-side JTC). To see what the driver has, with the robot connected:
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
