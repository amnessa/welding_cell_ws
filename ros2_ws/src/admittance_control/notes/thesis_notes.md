# Thesis notes: what we ran into, measured and learned

A running log of specific experiences in the welding cell, kept as raw material for the
thesis. Each entry has three parts:

- **What happened:** the symptom.
- **Evidence:** the numbers.
- **What we learned:** the lesson.

Newest first. Code and procedures live in the README (§14 for placement accuracy) and in
the plans under `notes/`; this file keeps the story and the numbers.

---

## 2026-10-27: Tack sequencing built: gap seed, farthest-first, heat, then transit

**What happened.** The pipeline's last step, ordering the tacks, was built from
`notes/tack_ordering_plan.md` (steps 1–4), once every source it relies on had been read in its
PDF. Three things surfaced on the way:
- the cell had been throwing away where along a seam the gap is smallest;
- the generator's rule order is not a farthest-first order on large loops;
- one source's year was wrong in a secondary reference list.

**Evidence.**
- *Where the gap is.* The fit-up kept only (low, high) per seam: `_edge_fitup` sorted away
  which end was low. Each seam now keeps its gap along its length. On a T with the web tilted
  1.5° (one end 5.2 mm lower), the profile matches the pose to 1e-6 mm and puts the minimum at
  the low end.
- *The bench T* (tacks at 32 and 218 mm on both sides of an 8 mm web; nominal 16 kJ per tack,
  15 s per tack). Both orders hold both seams after two tacks.
  - The rule's round-robin fires the 2nd and 4th tacks across the web from a tack 15 s old:
    a predicted 216 °C rise each.
  - The sequencer fires the 2nd at the far end of the other side (0 °C). Every across-the-web
    tack then comes 30 s after its neighbour: 113 °C at worst. That is the best any order can
    do at a constant step: in four slots, at least one station's two tacks are at most 30 s
    apart.
- *The rule on loops.* On loops of 4, 6 and 8 tacks the rule's order covers the loop as well as
  farthest-first at every prefix. On 10 and 12 its covering radius is twice as large after 4
  and 5 tacks (100.5 vs 50.3 mm and 83.8 vs 41.9 mm), because it bisects the half-loop ends
  first.
- *The guarantee.* r_k ≤ 2·r*_k + ℓ held against brute force on 60 random small cases; an
  id-only order breaks it in 39 of them.
- *The sources.*
  - Jonsson, Karlsson & Lindgren's tack paper is *Welding Journal*, October 1985,
    296-s–301-s. Abid & Siddique's reference list dates it 1995; the companion paper (*J. Eng.
    Mater. Technol.*, October 1985) cites it as accepted, which settles it.
  - Wärmefjord et al.'s "the geometry points lock the geometry" is in their full text, for spot
    welds.

**What we learned.**
- A secondary reference list can carry a wrong year: check the primary PDF.
- A heat criterion may choose *where* the next tack goes, never *when*. Jonsson et al. found
  that letting the first tacks cool before the next raised the forces in the tacks: "the tack
  welding procedure should be performed as fast as possible".
- A rule that reads like bisection is not automatically farthest-first. On a loop it bisects
  the half-loop's indices, which leaves part of the loop uncovered longer.

---

## 2026-10-27: The real test set's parts, checked against the generator's draw ranges

**What happened.** The stock for the real WeldSet test set arrived: 304 stainless (2 mm-wall
pipes Ø42.4–Ø101.6, three RHS sizes, 3 mm sheet), 1–2 mm high-carbon steel sheet, and a CNC
router with a bed of at least 600×600. Checking the planned parts against the generator's code,
not from memory, showed that several first choices put real scenes outside the corpus's own
draws for no reason, and that one flagged extension was not one.

**Evidence.**
- *Needlessly out of range, now fixed:*
  - a 300×300 base (the plate strata draw widths of 50–250; the curved scenes' base is the
    seam's extent plus 50–170 mm) → B1 300×240 for T line and lap, B2 200×200 for the curved T
    scenes;
  - contact fit-ups on the curved T scenes (gaps drawn 0.1–1.2 mm) and the butt arc (0.3–2.0)
    → shims of ~0.2 and ~1 mm; arcs at 0.5 and ~1.5;
  - a 10° mitre (tilts drawn 10–40°) → 15°;
  - a 120° stiffener arc (104 mm; spans drawn 120–300) → 150° and 170° arcs;
  - edge joints in 3 mm (drawn 1–2 mm) → the thin carbon steel.
- *Unavoidable, flagged:* the 2 mm walls (tubes drawn 3–8 mm, the stiffener 4–10), the 3 mm
  base under the curved T scenes (drawn 4–12), C42 (outer radius 21.2 against 30–80), S76 (r/R
  0.75 against at most 0.7), RR50 (50 against 60–160), probably every RHS corner radius (drawn
  from 8 mm), and the Ø76.1 stiffener arc (span 110).
- *Wrongly flagged before:* Ø60.3 (outer radius 30.15, inside 30–80).

**What we learned.** A real test set inherits the stock that was bought. Much of what puts a
scene in or out of distribution is still a free choice, though: base size, fit-up, angles, arc
length. Check those against the generator's code before anything is cut. The extension flags
will be computed from a range table that a test checks against the generator, not kept by hand.
Plan: `weld_generator/notes/phase9_real_set_plan.md`.

---

## 2026-10-22: The review's code defects fixed: the Jacobian, the image stamp, a pipe's gauge

**What happened.** The defects the thesis review found in the cell's code were fixed: the
aim's Jacobian, the camera's image stamps, and the member gauge, which was wrong in two places
(the multi-view ISO 5817 check and the input to the tack rule).

**Evidence.**
- *Jacobian.* `ur5e_jacobian` now divides by 4ε. A new test compares it with the log map of
  the FK over a small step at 20 random poses (to 1e-6); the old code fails it. To keep the
  behaviour tested on 2026-10-15/16, the four angular aim values were halved: gain 2.5 → 1.25 /s,
  roll gain 1.5 → 0.75 /s, cap 15 → 7.5°/s, 60 → 30°/s². In the closed-loop test (three
  targets, 0.1 s image latency), old and new centre the fingertip in the same time (old
  2.20–3.34 s, new 2.18–3.30 s) at the same achieved peak of 7.5°/s, and the optical centre's
  largest wander drops from 4.0–6.5 mm to 0.01–0.2 mm. The closed-loop test now also compares
  the achieved rate (from the FK) with the commanded one.
- *Image stamp.* The camera node stamped every image with `now()` after `wait_for_frames` and the
  depth alignment. It now uses the colour frame's own time: librealsense's global time (the
  camera clock mapped onto the computer's), moved back from the start of readout to
  mid-exposure by the frame metadata (FRAME_TIMESTAMP − SENSOR_TIMESTAMP). Checked on a
  pyrealsense2 software device: global time gives mid-exposure, system time the arrival, the
  hardware clock alone falls back to `now()` with a warning. Not measured on the camera yet:
  the node logs the stamps' age at publish (median, p95) every 30 s.
- *Gauge, multi-view.* The ISO limit took `t` from the bounding box. C1 (r 31 mm, wall 3 mm)
  counted as 62 mm, so on the 8 mm plate the level C limit read 1.62 mm instead of 0.51 mm. It
  now takes the registry primitive's gauge. Found on the way: the registry fits C1's wall at
  3.000002 mm, which falls in ISO's t > 3 mm row (0.92 mm), so `t` is rounded to 0.01 mm.
- *Gauge, tack rule.* `compute_tacks` handed weldgen's tack rule `"sweptslab"` with no params,
  so its swept-band case, min(band, height), never fired. The objects now come from weldgen's
  own serialisers. A curved flat bar 30 mm wide and 6 mm high on the 8 mm plate: `t` 8 → 6 mm,
  tacks 32 → 24 mm. SP3 (band 5 mm, 80 mm high) is unchanged.

**What we learned.**
- When two programs share a data format, write it with the producer's own code. The cell
  re-typed weldgen's object entries, got two names wrong, and nothing complained.
- A table with a boundary at a round number (t ≤ 3 mm) needs the nominal value, not a fitted
  one.
- The stamp of an image is the time the image shows, not the time it is sent.

---

## 2026-10-20: The aim's Jacobian was off by a factor of two, and every test passed

**What happened.** Writing the methodology chapter, the thesis review compared every equation of
the draft with the code. The camera aim's joint rates come from damped least squares on
`kinematics.ur5e_jacobian`, and its angular rows turned out to be twice the true ones. The aim
had been tested on the robot two days before and worked.

**Evidence.**
- The Jacobian takes $R(q+\varepsilon e_i)\,R(q-\varepsilon e_i)^\top\approx I+2\varepsilon[\omega_i]_\times$
  and divides its skew part by $2\varepsilon$ instead of $4\varepsilon$. At a test pose, the true
  angular velocity of wrist 1 is $(-0.296,\ 0.955,\ 0)$ rad/s per rad/s, the Jacobian column
  $(-0.591,\ 1.911,\ 0)$: exactly 2.000×. The linear rows are right.
- So the arm achieves half the commanded rate: 7.5°/s at the "15°/s" cap, an effective gain of
  1.25 /s instead of 2.5 /s. The lever-arm term of the optical-centre hold uses the commanded
  rate, so the optical centre drifts at up to ~8 mm/s, and the P pull holds it.
- The unit test of the optical-centre hold multiplies the joint rates by the same Jacobian, so it
  checks the code against itself. The closed-loop test integrates the true kinematics and passes,
  because a loop with half the gain still converges (centred in < 4 s, drift < 10 mm).

**What we learned.**
- A test that uses the code's own model as its reference cannot find an error in that model. The
  reference must be independent: here, finite differences of the forward kinematics.
- A feedback loop hides a gain error. It converges anyway, just more slowly, and the gain that
  was tuned by feel on the robot (1.2 → 2.5 /s) was really 0.6 → 1.25 /s.
- Writing the method down, equation by equation, against the code is itself a test.

---

## 2026-10-16: A wound-up cable read as a contact, 84 mm above the part

**What happened.** On SP3, two of four reachable tacks stopped their descent in the air:
"contact" at 1.5 N with the pen 66–68 mm above the part, in both runs. A third tack,
which the reachability check had passed, was refused by the marking node: "the pen
reaches the table".

**Evidence.**
- The false contacts were at the same place both runs, 82–84 mm before the registered
  point (16–18 mm into the descent). The planned descent had no kink there: waypoints
  5 mm apart, joint-speed changes ≤ 0.004 rad/s in the fast leg. So the force grew
  slowly, not as an inertial kick. A stroke after one of them read −1 N in the air: the
  reading drifted as the arm moved.
- The marks history separated the cause:
  - every good contact (14 on E2 the day before, one on SP3) had wrist_3 within ±151°
    of home;
  - the two false ones were the only tacks beyond ±180°: −202° and −287°;
  - wrist_1 (to +139° on E2) and the elbow did not separate them.
- The cause in the plan: each transit went to the 2π-equivalent of the next pose
  nearest the previous one (continuity), so the wrist kept winding from tack to tack.
  The same tool pose was available at +73° instead of −287°.
- The table refusal: the root tack sat 4.9 mm above the table (5.5 mm plate), the pen
  was 3.5 mm above it at the tack point, and 5 mm of overshoot along a pen 35° off
  vertical reached it. The reachability check does not look at the overshoot.

**What we learned.**
- A force-gated descent senses everything that pulls on the tool, and the camera cable
  is part of the tool. Joint "continuity" is not free: on a wrist with ±360° of travel
  it can wind a cable without limit. A window of ±180° about home loses no pose (every
  pose has an equivalent inside it) and keeps the cable slack. After the fix the same
  scene plans all four tacks with wrist_3 within −174°..0.
- Rules about measured things should use what is known about them: no surface can lie
  beyond the measured table, so an overshoot that reaches it ends there instead of
  refusing the tack.
- The control group mattered. The user's objection ("yesterday E2 worked with the same
  setup") was answered by E2's own records: it never wound the wrist past 151°.

---

## 2026-10-16: The aim buzz was the command staircase, not the controller

**What happened.** On the robot, the camera aim made the wrist buzz when the fingertip
was near the centre of the image. The first guess was that a P-only controller was
not enough. A 45 s bag of the joint states (500 Hz, with effort) and the velocity
commands (50 Hz) showed that the control law was not the problem.

**Evidence.**
- The wrist joint velocities have a sharp peak at 49–51 Hz, 30–50× the noise floor, and
  the efforts (motor current) at 50 and 100 Hz.
- ~8 ms after every 20 ms command step the effort spikes (−0.4 against a ±0.05
  background) and the velocity overshoots its new value by ~25 %. The UR applies each
  velocity step as a kick.
- The 100 Hz effort tracks the size of the step between commands (correlation 0.84),
  not the commanded speed (0.13). It is 11× the level seen while the trajectory
  controller holds the arm.
- Steady turns at the 15°/s cap are quiet. Starts, stops and reversals are loud.
- Near the centre the aim starts and stops most: a 64-px (≈ 4°) deadband on the
  1280-px image, an engagement that jumps the target rate to ~8°/s, and hand tremor at
  the edge.

**What we learned.** Velocity streaming to a 500 Hz controller at 50 Hz is a staircase.
Each step's size is acceleration × command period, and the arm turns every step into a
torque kick that is audible. That is a problem of signal timing, not of the control
law: adding I or D would not have changed it. The camera rate is no limit on the
command rate here, because between images the error is recomputed from the joint
states. The cure is to interpolate the command to the controller's rate (and to start
gently near the centre). Measure first: the bag took minutes to record, and it pointed
away from the obvious suspect.

**The fix, in two steps** (same day):
1. `rate_hz` 50 → 125: each step is 2.5× smaller. On the robot this "definitely
   improved" the sound (by ear; no bag of it).
2. A C++ relay (`aim_rate_relay`) between the Python node and the controller. It ramps
   linearly between the 125 Hz rates and publishes at the UR's 500 Hz: no steps at all,
   for 8 ms of delay. Against a fake cell the largest jump between commands fell to
   0.0017 rad/s (the 50 Hz bag: p95 0.009). As a by-product it closed a safety gap: a
   `kill -9` of the aim node mid-turn used to leave its last rate on the velocity
   controller, and now the relay ramps the arm to zero within ~0.1 s.

**For a figure later.** The 50 Hz bag is `ros2_ws/rosbag2_2026_10_16-13_49_03` (8.6 MB,
not in git: keep a copy). `scripts/aim_bag_report.py <bags...>` gives the numbers per
bag: command rate, step size, the buzz index (20–250 Hz wrist effort, aiming / idle)
and the strongest vibration line. The first bag: 50 Hz, index 3.4×, a line at 50.8 Hz.
Its `--npz` writes the arrays for plotting. Planned figures:
- the wrist_1 effort spectrum, 50 Hz direct vs the relay;
- an 80 ms zoom of commanded vs measured velocity, showing the kick after each step.

**Confirmed on the robot** (the second bag, `rosbag2_2026_10_16-14_09_33`, 38 s of
aiming; also not in git):

| | 50 Hz direct | relay |
|---|---|---|
| commands | 50 Hz | 500 Hz (input 125 Hz) |
| jump between commands, median / p95 (rad/s) | 0.00104 / 0.0089 | 0.00018 / 0.0009 |
| wrist effort 20–250 Hz, aiming / idle | 3.4× | 1.7× |
| vibration line | 50.8 Hz, 9× its neighbours | none |

At 50 and 100 Hz the second bag is level with the neighbouring frequencies. The
remaining 1.7× is broadband (the aim's own accelerations), with a faint trace at the
125 Hz input rate.

**A trap in the measure.** The first version of the report named the highest bin above
20 Hz as "the peak", and on the relay bag it said "23.4 Hz". That bin was the sloped edge
of the 20 Hz high-pass, not a tone. A line has to stand out from its neighbours, not
from the whole spectrum.

---

## 2026-10-15: Finger-pointing camera aim, where rotation-only motion makes latency compensation exact

**What happened.** For human–robot interaction we added an aim mode: before the
capture, the operator points and the wrist camera turns until the fingertip is centred.
The camera's image is ~0.1 s old when the gesture is recognised, and a visual servo
that acts on old images overshoots. Because the camera only rotates about its own
optical centre (its position is held), the ray to a still fingertip, expressed in the
robot base frame, does not depend on how far away the fingertip is. So the ray from an
old image, turned with the camera pose at that image's stamp (from the joint history),
gives the exact error in the current camera frame. The latency is compensated exactly
instead of being predicted.

**Evidence** (simulation on the UR5e kinematics: 30 Hz camera, 1.5 px landmark noise,
an arm lagging 30 ms; then the node against a fake controller manager):
- A 21° turn (240 px) is centred to ~10 px in ~3.5 s at the 15°/s cap, with no
  overshoot. The roll is held within 0.02° and the optical centre within ~4–6 mm.
- With 0.2 s of image latency: compensated, the camera stops at 20.5° with no
  overshoot; uncompensated, it runs to 21.2° (past the target, then inside the
  deadband).
- The gain could go from 1.2 /s (sized for uncompensated latency) to 2.5 /s: the time
  for 20° fell from ~4.1 s to ~2.9 s to within 32 px.
- The deadband needed hysteresis that ends *inside* the inner radius. Measured from that
  radius itself, the camera crept towards it for ever and never let go of the
  controller.
- Node-level bugs found against the fake cell:
  - one logging line used both info and warn severity, and rclpy raises on that;
  - two aim nodes fought over the velocity controller. A node now refuses when the
    controller is already active.

**What we learned.** Choose the motion so that the measurement becomes easy: restricting
the aim to rotation about the optical centre removes depth from the problem, and the
latency compensation then costs only a joint-state history. Stream rates, not
trajectories, for a human in the loop. Hand the joints back to the trajectory controller
as soon as the arm is still, so the rest of the pipeline never sees the difference.
Bench numbers (time-to-aim, overshoot, the hand-lost stop) are still to be measured.

---

## 2026-10-15: Multi-view on the bench, and the one degree of freedom no camera sees

**What happened.** On the bench, multi-view refinement was first refused for curved
parts, then accepted once its acceptance rules were made consistent. Then a mitred pipe
yielded no seam at all: its spin about its own axis was registered ~36° wrong.

**Evidence.**
- SP3 (S-band): fit-up −0.9..10.4 → 0.0..0.3 mm after `refine_pose`. Marks: 2 on the
  root, one 4.4 mm (was 9.5–12.7 mm).
- E2 (25° mitre): marks 3.7–6 mm from single-view poses; 2–3.2 mm (3–4 mm on the tilted
  side) after `refine_pose`.
- The acceptance rules changed to the user's principle, "apply a correction within the
  limits":
  - the overlap may not be worse than the saved assembly's (the better refinement had
    been refused, the worse saved pose kept);
  - a pipe's spin is not judged (it does not move the seam);
  - a pipe's bore is hollow.
- E2's spin: its axis was right (25.2°), its cut 15.4° off the plate, the seam's gap
  range 26 mm. The spin that rests the cut on the plate (+36.2°) brought it to 0.4 mm.

**What we learned.**
- **A symmetric part's hidden feature is a blind spot.** A cut pipe is symmetric about
  its axis except for its cut, and the cut is exactly where it touches the part it stands
  on, out of sight. Neither single-view nor multi-view registration can measure that
  spin, but physics can: a part standing on its cut rests with the cut on its base.
- **The seam did not depend on the wrong number; the checks did.** The intersection
  curve is spin-free, while the fit-up check that guarded it was not. Knowing which
  quantity a pipeline actually depends on decides which errors matter.

---

## 2026-10-14 (night): Multi-view for curved parts: the views were enough, the neighbours were the problem

**What happened.** Before changing the multi-view refinement for curved parts, its
synthetic test case was run both ways: with the old neighbour model (each part as its
bounding box) and with each part's true shape (a signed distance built from its
registered primitive).

**Evidence** (synthetic views, 4 cameras, 0.5 mm noise):
- C1, registered 8 mm off sideways: back to within 0.08 mm both ways.
- E2 on its 25° mitre: with boxes, the plate's top cut through E2's tilted box; the
  plate was pushed 49.7 mm, both parts were refused, and E2 stayed 5 mm off. True
  shapes: 0.03 mm.
- S3 on R2: with boxes, S3's saddle flanks sit ~5 mm inside R2's box, and both parts
  were refused for overlap. True shapes: 0.08 mm.

**What we learned.** The 8–9 mm sideways error of the round parts on the bench is not a
limit of the method: views from several sides determine a pipe's position, and C1 only
needed `refine_pose`. What did not carry over from the plates was the
NEIGHBOUR model. A bounding box is exact for a plate and wrong for a pipe standing on a
cut or wrapped round another pipe. Making each part's own surface the neighbour model,
the same primitives the seams come from, is what lets the refinement accept curved
joints.

---

## 2026-10-14 (evening): The curved bench without multi-view: flat sides are what pins a pose

**What happened.** All five curved joints were registered, seamed, planned and marked end
to end with the pen: C1, E2, RR1 and SP3 on a plate, and S3 on R2. The seams, tacks,
reachability and motion worked for every stratum. The mark error did not behave the same
on every part.

**Evidence** (by hand from the root, `tack` strokes):
- plate T 2.3–2.7 mm (the 6 Oct reference); RR1 within ~3 mm;
- S3 on R2 1–7 mm, one 11 mm;
- C1 up to 14 mm and E2 up to 14.4 mm, with one side of each pipe near the root and the
  opposite side up its wall;
- SP3 9.5–12.7 mm, with 2 of 6 tacks refused because the pen holder does not fit the
  S-curve's concave side.
- The pen's contacts put C1 and E2 8–9 mm sideways of their registered poses.

**What we learned.**
- The single-view registration of a part is as good as the flat faces it shows. A
  rounded-rect tube's sides pin it to ~3 mm; a round pipe's side leaves its position
  along the view loose by ~8 mm. That is the motivation for multi-view refinement of
  curved parts.
- A 20 mm stroke along the registered seam reports more error than the touch itself on
  tight curves (it climbs the real wall). Measure accuracy with dots, demonstrate with
  strokes.

---

## 2026-10-14 (afternoon): Joint-space shortest is not what a cell should drive round a pipe

**What happened.** With the planner fixed, the transits between the tacks round a pipe
still swung the pen through big loops, up to 0.9 m off, and the arm hit things in the
cell 75 cm away. Giving APS more time made the bad ones rarer, not gone.

**Evidence** (offline, E2, the same transit):
- APS minimises weighted joint travel. Paths of nearly equal joint cost (3.5 vs 3.9 rad)
  had tip paths of 257 vs 772 mm.
- Over repeated runs:
  - 2 s gave 497–1,873 mm of tip travel;
  - 5 s gave 353–595 mm;
  - more threads, or parallel runs, were worse (they share 8 cores).
- Starting each transit 100 mm out along the pen instead of 35 mm made most transits
  consistent, but not all.
- The move a welder would program, the tool sliding in a straight Cartesian line from
  one departure point to the next while it turns, came out at 141–211 mm, at the
  minimum joint cost, identically on every run. A sampling planner only needs to step
  in when that line is blocked.

**What we learned.**
- **Optimise the quantity you care about.** For the cell that is how far the tool sweeps
  through space (what it can hit), not joint-space length.
- **Structure first, then search.** Departure points plus the Cartesian line are the
  common case; APS is the fallback for the hard ones.
- **Re-planning on every small deviation threw the good plan away.** Joining the planned
  path from where the arm actually stands kept it.

---

## 2026-10-14: The pen's contacts measure the registration; the planner was failing on its own margin

**What happened.** The first marks on curved parts (C1, a pipe on a plate; E2, a mitred
pipe) were off by up to 14 mm. On every part one side's tacks were near the root and the
opposite side's were up the pipe's wall, never out on the plate. Separately, planning
took minutes: most transits logged "Invalid start" or "Invalid goal" and fell back to a
slow planner.

**Evidence.**
- Hand-measured, opposite pairs: C1 14 / 1.2 and 6.5 / 3.7 mm; E2 14.4 / 1.2 and
  11.7 / ~0 mm.
- Every descent stops at a 1.5 N touch, so its tip is a point on the real part. Fitting
  "real tube = registered tube moved sideways, real plate = registered plate moved up or
  down" to the four contacts (rms 0.48 mm on E2):
  - E2: 8.1 mm sideways, plate 3.0 mm lower;
  - C1: 8.8 mm sideways, plate 1.2 mm lower.
- A ~45° descent turns a sideways error into one side up the wall and the other near the
  root, which is the measured pattern. The pair sums agree with the ruler (14.2 vs 15.6,
  12.0 vs 11.7 mm).
- The planner: the transit clearance is set to 90 % of an approach pose's own, and the
  2 mm safety margin on top of it made that pose itself invalid. Each fallback transit
  took 1.5–3 min; with the margin capped and the path checks on the C++ model, the same
  plan takes 17.6 s instead of 478 s.

**What we learned.**
- **The force-gated pen is also a probe.** Its contacts measure the registration error
  per joint, with no extra hardware. That is evaluation only; the cell stays
  vision-only.
- **For a small curved part seen from one side, the dominant error is sideways, ~8 mm.**
  That is much larger than the 2.3–2.7 mm of the plate T. Plates have flat faces that
  pin the pose; a pipe's round side does not. This is what multi-view refinement is for.
- **Safety margins compose.** Two individually sensible margins (90 % of the end
  clearance, +2 mm in the optimiser) together made the start of every hard transit
  infeasible, and the planner said so in its first line of output.

---

## 2026-10-09 (night): The first curved-parts bench, and three silent failures

**What happened.** The printed parts went through the live pipeline. Perception worked:
PPF named every part, and FoundationPose registered and tracked them. Behind that,
three things failed quietly:
1. **The seam step never ran.** A new module was missing from the package's explicit
   install list. Every test imports from the source tree and passed. The launched node
   imports the installed package, logged one warning, and fell back to the old
   point-cloud detector.
2. **Re-registration jumped to another part.** When the tracker lost a part, it sent the
   frame with a mask projected from the last pose. The desktop reclassified it and
   answered the plate under the part.
3. **A near miss was invisible.** S3 on R2 produced the right saddle, 69.4° against
   70° drawn, but one end of the gap reached 10.5 mm against a 10 mm tolerance, so the
   pair was dropped without a word.

**Evidence.**
- C1 → `test_objv2_ear` and E2 → ear → base on re-registration.
- PPF margins of 0.04–0.39 (R2 the thinnest).
- ICP at save refused 30.8° on R2: all of it spin about the pipe's own axis, which is
  unobservable.
- S3's axis was registered 7 mm to the side of R2's.

**What we learned.**
- A fallback that only logs makes a failure look like a result. Each one now either
  fails a test (the install list), refuses loudly (a re-registration naming another
  part), or reports the near miss with its numbers.
- Symmetry must be told to the estimator. An axisymmetric part's spin is noise, so the
  gate measures only the tilt of the axis.

---

## 2026-10-09 (evening): A loop round a pipe needs a pen roll per tack

**What happened.** The tack planner chose one pen roll per seam and capped the joint
step between consecutive tacks at 69°. That rule had served the straight plate seams
well. On the first closed seam, the 4 tacks round the 62 mm pipe C1, it failed the
whole seam at the second tack: the wrist would have to turn 115°. The collision model
also boxed each pipe with one box, whose corners stood 41 % of r (13 mm on C1) outside
the wall, right where 45° tacks sit.

**Evidence.**
- With a roll per tack (chosen by clearance up to 10 mm, then the smallest joint step),
  bench-like scenes reach every tack of C1, E2 and RR1, with 6.6–9.7 mm minimum
  clearance.
- The two failures are geometric and reported, not forced:
  - SP3: 4/6, the holder within 1.7 mm of the band at its concave lobes;
  - S3 on R2: 3/4, the far side has no elbow-up solution.
- Pipes as 6 rotated boxes stand ≤ 3.3 % of r proud (1 mm on C1). Mitred and
  saddle-cut pipes become sector boxes, each from its own lowest cut point. Bands become
  box chains within 1 mm. The C++ planner did not change.

**What we learned.** On a straight seam the torch frame is fixed, so "one orientation
per seam" is a smoothness rule. On a closed seam the frame rotates through 360°, and the
same rule becomes an impossibility. The right unit of planning follows the seam's
geometry: per seam for lines, per tack for curves, with each move between tacks left
to the transit planner.

---

## 2026-10-09: Fitting printed parts back to analytic primitives; a CAD that looked right was not

**What happened.** The first six 3D-printed curved parts were fitted back to WeldSet
primitives from their CAD meshes. Each fit was checked in both directions:
- CAD vertices onto the primitive;
- primitive vertices onto the CAD;
- plus the volume.

Five fitted. One, the S-curve stiffener SP3, was refused. Its second side had been
drawn as the first curve **shifted 5 mm sideways**, not offset along the normal. The
model looks right in the CAD view, and the slicer prints it.

**Evidence.**
- C1, E2, R2, S3 (tubes) and RR1 (rounded-rect tube) verify at 0.004–0.057 mm, with
  volume errors of 0.0–0.24 %: only mesh chord error is left.
- SP3's wall ranges from 2.4 to 5.0 mm along the band; the thin parts are where the
  curve runs about 60° to the shift direction. The best constant band would miss the
  real part by about 1.3 mm.
- S3's saddle end is cut at r 51.04 mm, while the run pipe R2 is r 50. The joint
  therefore has a built-in root gap of about 1 mm.

**What we learned.**
- Ground truth from CAD is only as exact as the CAD's construction. A
  translated-copy band is a common drawing shortcut, and it silently breaks "constant
  thickness".
- Checking the fit **both ways** matters: CAD→primitive alone accepts an entry that is
  too long or too thick.
- Fitting refuses with a measured reason instead of approximating. That is the same rule
  as the box registry, and it turned a modelling slip into a one-line finding before
  any bench time was spent.

**Follow-up, same day.** SP3 was redrawn as a true offset and reprinted. The fit split
its cap outline at the four corners and fitted cubic splines to both sides and the
midline. It recovered **the spline that was drawn**: 4 control points (0, −110),
(−200, 0), (200, 0), (0, 110), to 3·10⁻⁶ mm, verified at 0.05 mm. Two numerical
lessons:
- solving the control points and the points' curve parameters together stalls (0.06 mm)
  unless it starts from linear fits alternated with re-projection;
- a "nearest dense sample" error measure adds the sampling gap (≈ 0.1 mm here) to the
  error it reports, while the solver's residual is the real distance.

With the parts registered, the seams of a pipe on a plate, a mitred pipe, a saddle, a
rounded-rect tube and a curved stiffener all follow from the registered surfaces
(`curved_seams.py`). The pose error shows up as fit-up along the curve, for example a
0.4–3.6 mm gap range for a 3° tilt, never as a different seam.

---

## 2026-10-06 (evening): A tracker is not a registration; and a planner needs fast collision checks

**What happened (1).** The first end-to-end run with FoundationPose tracking worked
mechanically, but:
- the saved parts interpenetrated by 6–11 mm;
- `refine_pose` rejected both parts (they would still overlap by 3.8 / 4.0 mm);
- the marks were 12 mm early on one seam, with no contact at all on the other.

With ICP tracking the saved pose had always been an ICP pose; with FoundationPose it was
the tracker's render-and-compare estimate. On a textureless 8 mm plate seen from ~0.6 m
that estimate is good enough to follow a part, but not to weld it.

**Fix:** at `save_object`, ICP on 5 fresh live clouds from the averaged tracker pose,
used only when it fits and the correction stays within limits. On a rendered plate it
took a 4.7 mm / 1.5° tracker error to 0.01 mm along the normal and 0.4° tilt. The
remaining 1–2 mm is in the plate's own plane, the direction one view cannot fix; that
is `refine_pose`'s job.

**What happened (2).** The transits deviated: the planner took the first RRT-Connect
path. OMPL's AnytimePathShortening (parallel planners + path hybridization +
shortcutting) is the remedy, but its Python bindings do not exist, and a Python
collision callback would serialize its threads.

**Fix:** the collision model ported to C++ (a pybind11 module) and checked *against*
the Python model rather than trusted:

| quantity | result |
|---|---|
| FK | equal to 1e-12 |
| `is_valid` | identical on 8 800 random and near-contact configurations |
| distances | equal to 1e-9 |
| speed | 590× faster |
| front↔back transit | ~6× shorter weighted joint travel at a 1 s budget |
| planner threads | CPU/wall 4.8 |

**What we learned:**
1. **A tracker answers "where is it now", a registration answers "where exactly is
   it".** Keep each for its job, and refine before freezing.
2. **To port a model, test the port against the original on the configurations where
   they are most likely to differ** (bisected near the collision boundary), not only on
   random ones.
   - The equivalence test found nothing wrong with the geometry. It did find a pybind11
     output bug that a "looks right" test would have missed, and one exact tie decided
     by the last bit.

---

## 2026-10-06: Live FoundationPose tracking: register on the desktop, track on the laptop

**What happened:** the plan was one GPU host doing both jobs over the network. Hardware
decided otherwise:

- A full-frame `register()` (SAM2 + PPF + the scorer, ~250 hypotheses) does not fit the
  laptop's 8 GB RTX 4060. It stays on the desktop.
- Tracking needs only the refiner (~300 MB), so it runs on the laptop, in the same
  Docker image, next to the camera. Frames never cross the network; only a lost part
  sends one frame (plus the CAD silhouette as mask) back to the desktop to re-register.

**Evidence** (laptop, a stand-in camera publishing a saved frame at 30 Hz):

| quantity | result |
|---|---|
| pose rate | 25–29 Hz (ICP tracking: ~10 Hz on the CPU) |
| delay, image → pose, raw bytes over localhost | 70–90 ms |
| delay with JPEG/PNG encoding | ~100 ms |
| re-registration on the desktop | 4.0 s |

- Encoding only costs time when nothing crosses a network.
- 2 frames in flight doubles the rate over 1 at the same delay.

**Wiring into the cell:**

- The ICP node keeps everything stationary (`run_icp`, `save_object`, `refine_pose`).
  With `tracking_source: fp` it averages the tracker's frames at rest.
- Each tracker pose is put into `base_link` with TF at its **own image stamp** before
  averaging. Camera-frame poses from a moving wrist camera cannot be averaged, because
  each lives in a different frame.
- The bridge now stamps detections with the **registered** frame's stamp instead of the
  reply time. The robot pose at the seed is then the one the image was taken at, even
  when capture, click and registration took a minute. The tracker's "stale seed" rule had
  to change with it: only a latched detection delivered at start-up may be stale.

**What we learned:** split a learned pipeline by what each stage needs, not by where the
software happens to run.

- Registration is rare and heavy, so it goes on the big GPU.
- Tracking is frequent and light, so it goes on the machine with the camera.

The network then carries only the rare event, and the frequent one stays local. Both
machines run the same Docker image, so the split costs no extra maintenance.

---

## 2026-10-02 (evening): Decisions: no touch sensing, FoundationPose for tracking

**What happened:** with the marks at ~2.5 mm by vision alone, two open questions were
settled.

**No touch sensing in the method.** The cell is framed academically: a scenario where the
parts cannot be touched must remain valid. Touches stay in two roles only:

- calibration: pen TCP, table plane, the Tare's ground-truth distance;
- independent measurement of the error (`touch_probe.py`).

They are never a step that corrects a tack. The evidence that this is enough: every
millimetre from 8 mm down to 2.5 mm came from vision and calibration, not from contact:

- the ICP normal gate;
- the depth Tare;
- the extrinsic tilt;
- the multi-view refinement.

**Tracking: FoundationPose instead of ICP, no Kalman for now.**

- ICP runs on the CPU. Each call is seconds of nearest-neighbour search and Gauss-Newton,
  far too slow to follow a part that moves. It is the right tool for what it is used for
  in the literature: aligning point clouds of a stationary object. Here that means the
  `run_icp` seed and the `refine_pose` multi-view refinement.
- FoundationPose's tracking mode registers once and then refines frame to frame on the
  GPU at tens of Hz.
- The planned Kalman fusion of FoundationPose and ICP is dropped for now. It would fuse a
  fast tracker with a slow one that adds nothing while the part moves.

**The open problem this creates is systems, not vision.** FoundationPose runs in Docker on
a GPU host reached over Tailscale. A live pose in RViz needs frames streamed there and
poses streamed back fast enough. The candidates, in `todo.md`, are:

- HTTP keep-alive;
- a WebSocket or gRPC stream;
- ROS 2 inside the container (Zenoh or a discovery server across Tailscale);
- tracking locally on the laptop's RTX 4060.

**What we learned:** split the work by the regime each tool suits.

- A stationary part, millimetre accuracy, seconds allowed: ICP and the multi-view
  refinement (CPU).
- A moving part, tens of Hz: a learned tracker on the GPU.

Choosing one of them does not remove the need for the other; they answer different
questions.

---

## 2026-10-02 (15:00): Headline: all tacks within ~2.5 mm

**The full chain:** calibrated kinematics → Tare-corrected depth → refined extrinsic tilt
→ registration → `refine_pose` (4 close views, online self-calibration iterated, the prior
corrected by R_scan·d, gap measured) → `welding_points` → reachability → marking.

**Results:**

- Both parts ACCEPTED and applied.
- d = (1.61, 3.57, −5.92) mm in the camera frame, 7.1 mm of correction from the camera at
  the scan pose.
- The fit-up warning flagged the ear's foot at 0.6–2.0 mm above the base, against the
  1.6 mm level C limit.
- **Marks:** every tack within about 2.5 mm, on both seams. The first stroke ended early
  but left a dot inside that range.

For comparison:

| when | mark error |
|---|---|
| 28 Sep | 3.7–4.3 mm (with the kinematic error) |
| 1 Oct | 3–8 mm |
| this morning | 14–16 mm (the straddle, the unmodelled plate, the depth bias) |
| now | ≈ 2.5 mm |

**What is left:**

- Contacts 2.8–6.3 mm EARLY along the pen on all four tacks, both sides, so a vertical
  offset, not a sideways one.
- Pen touches: base top +4.8 / +4.7 mm (the real surface is higher than the refined pose);
  ear faces +1.2 / +4.1 mm.

**Reading:** before `refine_pose` the camera placed surfaces about 2 mm high
(`extrinsic_check`); the online correction then lowered the parts by about 7 mm
vertically. So it over-corrected the height by about 4–5 mm. The direction that moves
every view's cloud up or down together is the one the 45°/60° views separate worst: only
via the 60° view, with model leftovers (the depth trend) leaking in.

**Consequences:**

- Do NOT promote a self-calibration from these runs; all of them carry that vertical bias.
- Next step: anchor the height to the pen-referenced table (`extrinsic_check`, which
  agrees to about 2 mm across the working range), and let the online correction apply
  only the horizontal part of d.

**Caveat (the user):** the parts are not fixed in place and can move a millimetre or so
between the scan, the marking and the touches.

---

## 2026-10-02: Multi-view refinement on the bench after the depth fix

### What happened

Three `refine_pose` runs on the same untouched parts:

- **The views were consistent with a pure camera translation:** d = (0.41, 4.54, −5.00),
  (−0.12, 4.05, −4.78) and (0.23, 4.29, −4.99) mm in the camera frame.
- **After taking d out, the views agreed to 0.1–0.2 mm,** with fit RMS 0.73–0.77 mm. The
  RMS was 0.96 mm under the depth bias of the day before.
- **Yet both parts were rejected:** each run "changed the fit-up" by about 8 mm.

### Cause

The saved poses were registered through the same camera, from the scan pose, so they
carry that view's share of the error, R_scan·d. The run took d out of its views, but
held the parts' unmeasured directions at the uncorrected saved pose:

- the base's slide in its own plane stayed put, while
- the ear's measured position moved by the camera error.

The result was a fake relative change between the parts.

### Fix

Take R_scan·d out of the prior too. Each saved object already records its scan camera
(`T_static_camera`). Replaying the same three captures offline with the fix:

- **Runs 2 and 3 accepted.** Corrections of 8–10 mm, of which 6.3–6.6 mm is the camera
  error at the scan pose.
- **The root-relevant quantities repeat to under 1 mm between runs:** the ear's face to
  0.66 mm / 0.08°, the base top at the ear's foot to 0.24 mm.
- **The base's unmeasured in-plane slide and yaw differ by 2.7 mm / 0.48° between runs**:
  in-plane only, and only through the prior.
- **A synthetic test now covers it:** saved poses carrying R_scan·d, views carrying R_cam·d.
  The result lands within 0.5 mm of the truth, the unmeasured slide included.

### Self-calibration says "not ready", and why that is right

- **A fourth run from earlier the same day,** with the same extrinsic file but another
  scene and view set, gave d = (−0.52, 2.04, −3.93): 2.5 mm from the other three. The
  spread rule (1 mm) refused to write a calibration.
- **The three agreeing runs are repeat captures** of one scene from the same four views.
  They show repeatability, not correctness.
- **d therefore absorbs more than the extrinsic translation.** It also picks up
  view-dependent leftovers: the 1.1 mm depth trend over range, the 0.07° tilt residual,
  and kinematic residuals. Different view sets project those differently.

### Then: a standing plate's in-plane rotation is the weak spot

A fresh registration at 14:17 put the ear 3.6° off the 14:06 one, rotated in its own
plane, about its normal. Its lean from vertical was about the same, 0.18° vs 0.35°. The
multi-view refinement wanted to correct that by 3.07°, all of it in-plane.

How the ear's foot sits against the base top:

| ear pose | foot end 1 | foot end 2 | |
|---|---|---|---|
| saved | −2.3 mm | +0.8 mm | one end sunk into the base |
| refined, no limits | +0.8 mm | +3.0 mm | lifted, still tilted 2.3 mm along its 250 mm length |
| reality | ≈ 0 | ≈ 0 | it stands on the base |

- **Why the camera can't pin it down:** only the plate's 8 mm top and end edges constrain
  that rotation, so the views barely measure it ("not measured: turn about its normal").
  The acceptance rules rightly refused a 3° change in that direction.
- **The same weakness behind an older symptom:** the varying "fit-up gap 0.8–3.6 mm" that
  `welding_points` reported registration after registration. The gap was the ear's foot
  tilted along its length: registration error, not a real gap.
- **Why it matters:** the root height changes by that much from one end of the seam to the
  other.
- **First idea: physics.** The ear rests on the base, so a two-sided "resting contact"
  (the foot pulled ONTO the neighbour's face) would fix both the height and that rotation.
  The cost is that the fit-up gap becomes an assumption.

### Resolution: measure the gap and warn; don't assume it

**The user's requirement:** "an operator who places the parts too far apart must be
told." Fillet root gaps have a limit: ISO 5817:2023 Table 1 no. 617, which weldgen already
implements. For 8 mm plates with a throat of a = 0.7·t the limit is 1.06 mm at level B,
1.62 mm at C and 2.68 mm at D. Parts never interpenetrate (the overlap rule), and a
gap is information.

**What the experiments showed:**

1. **The resting pull overrides the data.** A synthetic 3 mm gap was pulled below 1 mm,
   unwarned. On a non-flush synthetic truth the pull moved the ear 1.2 mm off it. So the
   resting contact is an option, OFF by default.
2. **The real cause of the rejections was again a relative criterion.** A part direction
   counted as "measured" only with at least 5% of the strongest direction's information,
   and a plate's faces carry thousands of points. Switched to absolute uncertainty: a
   direction counts as measured at a std of 0.5 mm or better.
3. **With that, the views alone determine the standing plate.** Four real runs, from two
   different registrations, are all accepted. They agree on the surfaces to 0.12–0.41 mm
   (base) and 0.07–0.38 mm (ear), and the run from the other registration agrees to
   0.4 mm.
4. **The gap is measured and warned.** All four runs see the ear's foot 0.5–0.7 mm above
   the base at one end and 2.0–2.8 mm at the other, over the level C limit, so a WARNING.
   It's consistent across registrations, so it's real or a systematic sensor effect, not
   noise. To check by hand: a feeler gauge at the foot.
5. **Exact-box synthetic test:** the ear placed 3 mm off is measured within 0.7 mm and
   warned; placed flush, there is no warning.

**A limitation to report:** the base's slide within its own plane still varies 1–3 mm run
to run, although the absolute criterion counts it as measured. It rests on grazing edge
points, which are noisier and more correlated than the noise model assumes. It doesn't
move the weld roots (base top and ear faces), but the uncertainty estimate is optimistic
there.

### What we learned

1. **A correction learned from the data must be applied consistently** to everything that
   carries the same error, here both the views and the prior. Otherwise the uncorrected
   parts show up as fake geometric changes.
2. **Repeat captures of one scene are not independent evidence for a calibration.** The
   self-calibration should count distinct scenes or view sets: key runs by the registration
   (the assembly) and require 3 distinct ones.
3. **"Online" correction (per run) and "persistent" correction (the extrinsic file) need
   different evidence.** Per run, d only has to make the views agree. Written into the
   extrinsic, it has to hold across scenes.

---

## 2026-10-02: The camera's depth was range-dependent; Tare fixed it on the second try

### What happened

After the kinematics, ICP and holder fixes, every pen mark still met the surface early:
8.3–8.8 mm along the pen on all four tacks, both sides of the T. Marks landed 3–7 mm off.

The registration agreed with the camera's own cloud. `check_registration`: base +0.3 mm,
ear −0.1 mm. The pen disagreed: touching the base top read +6.0 and +7.7 mm above the
registered face, while the ear faces read +0.1 and +3.8 mm.

So the camera's picture and the robot's frame were about 7 mm apart in height.

### Diagnosis

The multi-view capture (`refine_pose`, 4 views at 0.4 m) gave the decisive evidence.
Taking the base top's height from each view separately, against the saved registration:

| source | range to the base top | base top vs registration |
|---|---|---|
| pen touches (truth) | — | +5.9 / +7.8 mm |
| 4 close views | 364–477 mm | +3.9 / +4.1 / +4.3 / +4.3 mm |
| scan-home view (the registration) | ~609 mm | 0 (by definition) |

So the error grows with the distance from the camera:

- **at the scan home:** about 6.9 mm low;
- **in the close views:** about 2.8 mm low.

Including the viewing angles, two models predict different ratios between those:

| model of the depth error | predicted ratio |
|---|---|
| grows linearly with range | 1.8 |
| grows with range² | 2.6 |
| **measured** | **2.5** |

The range² law is a stereo camera's signature: a small disparity offset δ gives a depth
error of z²·δ/(f·B).

### Confirmation

`extrinsic_check`: the camera looking down at the bare table from three heights, all over
the same spot, compared with the pen-touched plane:

| camera range | camera vs pen plane |
|---|---|
| 291 mm | +1.92 mm |
| 439 mm | +0.31 mm |
| 626 mm | −3.28 mm |

- **Fit:** error = a + k·z², with k ≈ −17 mm/m² (depth reported too long). It predicts
  the middle point to 0.2 mm; a straight-line fit misses it by 0.7 mm.
- **Implied disparity offset:** about 0.6 px.
- **Size:** k = −17 mm/m² is about 6.6 mm at 0.62 m and 2.7 mm at 0.4 m, which matches
  the parts.

### Fix, attempt 1: on-chip calibration plus Tare, distance entered by hand

It overshot. The error flipped sign and became linear:

| camera range | camera vs pen plane |
|---|---|
| 285 mm | +1.71 mm |
| 423 mm | +3.25 mm |
| 604 mm | +5.24 mm |

That's +11.1 mm per metre, with the middle point on the line to within 0.01 mm: a depth
SCALE error of about 1.1%, now reading too short.

- **Hypothesis, not confirmed:** the ground truth entered for Tare was measured from the
  wrong origin. The D435/D435i depth origin sits 4.2 mm behind the front glass. A few mm
  wrong at a 0.3–0.6 m target is a 0.7–1.4% scale error, which is the size we saw.

### Fix, attempt 2: Tare against a distance computed from the robot

1. **Re-measured the table plane** with the pen (7 touches, new centre (−0.143, 0.426),
   z −96.15 mm). The old one was from 28 Sep, and by 29 Sep the pen read 1–3 mm off it.
2. **Computed the Tare ground truth in the robot's frame** with `scripts/tare_distance.py`.
   It takes the camera pose from `/joint_states` through the calibrated kinematics and the
   extrinsic file, and returns the depth along the optical axis to the pen plane.
3. **Set up the camera:**
   - square to the TABLE, not to the floor: the table is tilted 1.57°, so "straight
     down" in world terms was 1.69° off;
   - its axis landing on the touched patch;
   - about 0.5 m away.

Result, over the touched patch:

| camera range | camera vs pen plane |
|---|---|
| 297 mm | +0.29 mm |
| 443 mm | +1.06 mm |
| 657 mm | +1.43 mm |

Across the range that's +1.1 mm, against −5.2 mm before any calibration and +3.5 mm after
the first Tare. At the scan home (~0.6 m), surfaces now land about 1.4 mm high instead of
about 7 mm low.

**The remaining tilt, separated:** the same spot at four wrist yaws (−130, −53, 46,
140°; 270° spread, 0.46 m range):

| part of the tilt | size |
|---|---|
| camera-fixed (extrinsic rotation, or the depth sensor after Tare) | 0.49° |
| world-fixed (the table there vs the pen plane) | 0.44° |
| residual | 0.05° |

- `refine_extrinsic_from_table.py` gives a correction of 0.488° about the camera's x axis
  (fit residual 0.07°, conditioning 1.00); it moves points 2.6 mm at a 300 mm range.
- The table's own 0.44° came out separately: it is NOT a camera error, and a single-yaw
  check would have mixed the two.
- Written as `T_tcp_to_cam_refined.npy` and promoted.

**Re-check after promotion** (same spot, wrist at −136, −46, 44, 134°):

- **Every view's tilt now points the same way:** 137–148°, 0.36–0.51°, mean 0.44° toward
  143°. That is the table's own tilt, matching the 0.44° separated above.
- **The part that turns with the wrist (the camera) is about 0.07°,** down from 0.49°.
- **Height:** +1.4 to +2.5 mm. The spread fits the 0.44° table tilt across view spots up
  to 180 mm apart. What remains is a constant of about +2.0 mm at a 0.46 m range: the
  camera places surfaces about 2 mm high.
- **Consequence for marking:** on a 45° approach the pen must travel about 2.8 mm past
  the plan, right at the 3 mm overshoot. Marking tests now use `overshoot_m` 6 mm until
  the constant is calibrated out (multi-view self-calibration, or a touch-based offset).

**Calibration state after 2026-10-02:**

| quantity | value |
|---|---|
| depth trend over 0.3–0.65 m | ≈ 1.1 mm |
| camera tilt | ≈ 0.07° |
| constant height offset | ≈ +2 mm |

Before the day it was −5.2 mm over that range, a 0.49° camera tilt, and about −7 mm at the
scan home.

### Side effects worth reporting

- **The multi-view refinement failed safely under the depth bias.** The close views
  disagreed with each other in a way a camera translation can't explain: fit RMS
  0.96 mm, against 0.35 mm on synthetic data. The online correction couldn't remove a
  range-dependent error. The acceptance rules rejected both parts ("fit-up changed
  11.4 mm / 2.0°"), so the assembly was not corrupted.
- **The self-calibration history recorded a wrong extrinsic offset.** It logged
  (−2.3, −2.0, 1.1) mm, which was the depth bias mis-attributed to the extrinsic. The
  history is keyed on the extrinsic FILE, but recalibrating the camera's depth also
  invalidates earlier runs. The key should include the depth calibration state.

### What we learned

1. **A stereo depth camera's error depends on range, as range² from a disparity offset.**
   A check at one distance says little. Validate at three or more ranges spanning the
   working volume (here 0.3–0.65 m).
2. **The vendor's Tare is only as good as its ground-truth distance.** The distance needs
   a well-defined origin. Deriving it from the robot's own frame (kinematics + extrinsic
   + a pen-touched plane) makes the camera consistent with the frame the robot acts in.
   That consistency is what marking needs.
3. **A touch reference is what exposed it.** The registration agreed with its own camera
   to well under a millimetre, while being 7 mm off in the robot's frame. Vision checking
   vision can't see a shared bias.
4. **Several views at different ranges turn a hidden bias into a measurable
   inconsistency.** That's diagnostic value even when the refinement itself has to be
   rejected.
5. **Order of calibration matters:** robot kinematics, then the pen TCP and touched plane,
   then camera depth (Tare), then the extrinsic (tilt, then translation). A step done on
   top of an uncorrected earlier one learns that earlier error.

---

## Earlier experiences (2026-09-28 … 10-01), in brief

### 2026-09-29: Kinematic model mismatch

- **What:** TF and the planner used nominal UR5e kinematics; the robot uses its factory
  calibration.
- **Size:** 0.55° at the camera, and 2.4–4.2 mm at the pen across poses.
- **Lesson:** every frame consumer must use the same kinematic model. Shared code now
  carries the model, and reports record which one they used.

### 2026-09-29: Thin-plate face straddle in ICP

- **What:** an 8 mm standing plate registered leaning 6.5°: its hidden back face had
  paired with the visible face, so the model straddled its own two faces.
- **Effect:** the weld root was 10 mm off, while the "fit" looked fine.
- **Fix:** a normal-compatibility gate (60°) brought the lean to 0.3–0.8° and the root to
  ±0.2 mm on the same data.
- **Lesson:** model-to-scene ICP needs visibility or normal gating for thin parts.

### 2026-09-29: A too-conservative tool model, hidden by a wrong registration

- **What:** the holder modelled as a 42 mm capsule cleared a square T by only 1.1 mm. Some
  seams counted as reachable only because the straddled registration had opened the
  corner to 96°.
- **Fix:** the envelope from the holder's CAD gives 15.5 mm.
- **Lesson:** fixing one error can unmask another.

### 2026-09-29: An unmodelled 7 mm plate

- **What:** an extra metal plate under the ear produced 9.7 mm early contact (7 mm / cos 45°).
- **The signal was there:** the registration had already reported a fit-up gap of
  0.9–4.6 mm.
- **Lesson:** treat unexpected fit-up gaps as a scene check.

### 2026-09-29 / 10-01: Motion-safety lessons

- **Transit clearance:** with the tack clearance (3 mm) also used for free moves, a
  transit passed 3.7 mm from the ear and hit it at 9 N. Free moves now keep 20 mm.
- **Wrist wind-up:** after rolled tacks, the home path arrived one wrist turn off
  (5.42 rad instead of −0.86). It now returns to the exact home joints.
- **Force-sensor drift:** after a power cycle the F/T sensor read 7 N at rest, and every
  first transit aborted. The bias is now re-measured before every free move.

### 2026-10-01: Time between registration and marking

- **What:** registered at 12:47, marked around 14:40. By then the registration disagreed
  with the live cloud by 3.7–4.6 mm: either the parts moved or the camera drifted.
- **Effect:** the marks were off by 12.5–12.9 mm on one side and 2–2.9 mm on the other.
- **Rule since:** check right after registering, mark right after checking.

### 2026-10-01: Multi-view refinement, synthetic findings

- **The joint fit does NOT cancel an extrinsic translation error.** The part common to
  all views stays. Estimating it (d) and taking it out does: recovered to 0.2 mm, then
  both parts within 0.1 mm of the truth.
- **"Determined" must be judged in absolute terms.** One direction of d looked weak in
  relative terms (4% of the strongest), yet was measured to 0.16 mm.
- **Our own synthetic renderer was biased at first.** Z-buffering surface samples biased
  every view about 1 mm toward its camera, which faked an extrinsic error. Exact ray
  casting removed it.
- **The feared lap-joint swap never happened.** The rule meant to prevent it only cost
  correction range.
