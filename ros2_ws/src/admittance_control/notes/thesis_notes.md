# Thesis notes: what we ran into, measured and learned

A running log of specific experiences in the welding cell, kept as raw material for the
thesis. Each entry has three parts:

- **What happened:** the symptom.
- **Evidence:** the numbers.
- **What we learned:** the lesson.

Newest first. Code and procedures live in the README (§14 for placement accuracy) and in
the plans under `notes/`; this file keeps the story and the numbers.

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
- Written as `T_tcp_to_cam_refined.npy`; promotion and re-check pending.

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
