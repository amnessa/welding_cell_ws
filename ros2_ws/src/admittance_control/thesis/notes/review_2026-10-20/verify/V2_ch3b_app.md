# V2: fact-check of ch3 (§Multi-View Close-Range Refinement → end), Appendix A, Appendix C

Read-only check, 2026-10-20. No repository file was edited.

**Roots used below**
- `ch3` = `/workspaces/welding_cell_ws/ros2_ws/src/admittance_control/thesis/manuscript/chapters/ch3_methodology_cell.tex`
- `A` = `.../thesis/manuscript/appendices/a_parameters.tex`; `C` = `.../thesis/manuscript/appendices/c_derivations.tex` (C is the uncommitted working-tree version)
- Paths like `multiview_refine.py`, `marking.py` are in `/workspaces/welding_cell_ws/ros2_ws/src/admittance_control/admittance_control/`. `scripts/`, `launch/`, `config/`, `notes/`, `test/`, `src/` are relative to `/workspaces/welding_cell_ws/ros2_ws/src/admittance_control/`.
- `WG/` = `/workspaces/welding_cell_ws/weld_generator/weldgen/`.

**Method.** Every number, threshold, algorithm line and causal claim was checked against the code first, then the config, then the dated notes. "In use" means node parameters, launch overrides, `config/marking.json` and `config/pen_tool.json`. Appendix A rows 20–43 (gestures, aim, perception) were checked by a delegated pass; I spot-checked its two I-findings in `pose_stats.py`.

Severity: **E** = wrong (fix the text); **I** = imprecise or incomplete in a way that changes the meaning; **N** = a nuance.

## 1. Findings

### 1a. The E items (fix first)

| # | draft file:line | draft says | correct value / what the code does | evidence file:line | sev |
|---|---|---|---|---|---|
| 1 | ch3:1254 (tab:mv_acceptance) | "for an **uncut** tube, the tilt of its axis, not its spin" | The axis-tilt rule applies to **every** tube primitive, cut or not: `_tube_spin_deg` only tests `shape.kind == "Tube"`. It was introduced for the **mitred** E2, where 5.13° was 5.08° spin + 0.71° tilt. The draft's own ch3:1267-1269 says the mitred tube "is judged by its axis". Write "for a tube (cut or not)". | multiview_refine.py:874-882, 903-911; notes/thesis_notes.md:201-208 | E |
| 2 | ch3:1509-1511 | closed-seam tacks "evenly spaced from a phase … so the layout is deterministic but does not depend on how the curve happens to be parametrised" | Positions are s_k = (phase·L + k·L/n) mod L, as arclength **from the polyline's first point**. They therefore move with the curve's start point and direction. The hash makes the rotation deterministic and seed-free, nothing more. Only the arclength spacing makes the layout independent of the parametrisation's *speed*. | WG/tacks.py:25-30, 115-125; seam_from_registration.py:449-454 | E |
| 3 | ch3:1511-1512 | "the scene's identifier is the directory the assembly was saved in, so a closed seam's tack phase **changes when the same assembly is saved again**" | `scene_id = str(self._save_dir)`, which is a **fixed** directory: `save_dir`, by default `results_dir` = `scripts/foundationpose_results`. Old files are archived to `previous_assemblies/<stamp>/` only on reset. Re-saving gives the **same** phase, and every assembly saved there shares the phase for a given seam index. The phase changes only with the directory or the seam's index (seams are renumbered by weldable/length). | scripts/icp_pose_refiner_node.py:436, 557-561, 1063-1085, 1198; seam_from_registration.py:403-405 | E |
| 4 | ch3:1650-1651 | κ = 0.15 rad/mm "so that 1 mm of clearance below 10 mm is worth about **9°** of base-joint motion" | The joint cost is ‖W δ‖₂ with base weight w₁ = 2. 0.15 cost-rad is therefore 0.075 rad = **4.3°** of base-joint motion (8.6° of a weight-1 joint, wrist 1 or 2). The code comment has the same slip. | tack_reach.py:396-417 (comment :398) | E |
| 5 | ch3:1917-1918; ch3:1938 | "every motion node starts in a dry-run mode until it is told otherwise"; "the motion nodes default to a dry run" | False for two motion paths. The multi-view refinement **moves by default** (`mv_dry_run` false, "the user's choice 2026-10-01"). `gestures.launch.py` starts the camera aim with `dry_run` false (`aim_dry_run` default `"false"`). Only `tack_marking_node`, a bare `camera_aim_node.py` and `table_touchoff` default to a dry run. | multiview_service.py:12-13, 59; scripts/icp_pose_refiner_node.py:857; launch/gestures.launch.py:72-74, 92; motion.py:74; scripts/camera_aim_node.py:94 | E |

### 1b. All findings in draft order (the E items repeated in place)

| # | draft file:line | draft says | correct value / what the code does | evidence file:line | sev |
|---|---|---|---|---|---|
| 6 | ch3:992 | "four close views around the joints" | All four views look at **the centre of the assembly's bounding box**. They are chosen by how much of the seam band they see. | multiview.py:283-284, 300-303 | N |
| 7 | ch3:994-995 | the run "can be replayed offline with the same code, **for example with another extrinsic**" | The replay uses the per-view `T_base_cam` recorded from TF at capture time. `multiview_refine_offline.py` has no extrinsic option. Swapping the extrinsic is possible in principle (camera-frame clouds and joints are stored) but not implemented. | multiview_capture.py:139-140; scripts/multiview_refine_offline.py:99-110 | I |
| 8 | ch3:1003-1005 | a sample "1–30 mm from another part" lies in the seam band | The distance is measured to the other parts' **collision boxes** (near-exact boxes for registered curved parts, else the bounding box), not to their surfaces. | multiview.py:107-131, 161-174 | N |
| 9 | ch3:1007-1009 | sees a target "between 0.28 and 0.60 m away … not behind another part's collision box" | 0.28–0.60 m is the **depth along the optical axis** (camera z), not the range. The occlusion test uses **every** part box, the part's own included, with the ray stopped 2 mm short of the surface. | multiview.py:182-184, 188-197, 74 | N |
| 10 | ch3:1024-1027 | "A 30° view would halve the weakest uncertainty of the bench T-joint" | It is the weakest std of the **extrinsic estimate d**, 0.16 → 0.08 mm (synthetic T views). Not reachable means "IK or a stretched elbow". | notes/multiview_refine_plan.md:543-546; multiview.py:55-59 | N |
| 11 | ch3:1039 (alg:mv_views l.7) | "wrist and elbow away from their singularities" | Quantify: \|sin q₅\| and \|sin q₃\| ≥ sin 0.35 rad. | multiview.py:293, 320-323; config/marking.json:57 | N |
| 12 | ch3:1042-1043 (alg:mv_views) | "plan the transits …; if fewer than two views or no way home then refuse" | A view with no collision-free transit is **dropped** first, then the ≥ 2 test runs. Multi-view transits are the straight edge or Python RRT-Connect (no APS, no Cartesian stage), timed by a 0.3 rad/s speed cap. | multiview.py:397-409, 420-424; multiview_service.py:282, 294 | N |
| 13 | ch3:1054-1055 | the camera pose comes "from the transforms at the cloud's own stamp" | It is the TF at the stamp of the **first** median frame. The median is accepted with ≥ n/2 = 8 fresh frames within 8 s. | multiview_service.py:70, 217-224, 307-310 | N |
| 14 | ch3:1056-1057 | extrinsic "checked against the calibration file (to 0.1 mm)" | 0.1 mm **and 0.01°**. | multiview_service.py:145-147 | N |
| 15 | ch3:1058-1060 | cropped, cut, thinned, "given normals … and transformed into the base frame" | Order in code: normals on the organized cloud (camera frame) → base frame → crop within 30 mm of the saved parts' **bounding boxes** → table cut (vertical, 2 mm) → 3 mm voxel. The depth window is 0.15–1.0 m. | multiview_capture.py:40-46, 129-149 | N |
| 16 | ch3:1087-1096 (eq:mv_objective) and ch3:1124 (alg) | "ω_v = 1/N_v makes the views equal" | The code **normalises** the view weights to mean 1 over the part's owned points: w = (1/N_v)/mean(1/N_v). With a literal ω_v = 1/N_v the data term would total ≈ K, and σ²ξᵀΣ⁻¹ξ and w_o = 10 would be ~N times stronger than intended. w_o = 10 is "× the per-point data weight". Write ω_v = (1/N_v)/mean(1/N). | multiview_refine.py:95, 379, 404-405, 533 | I |
| 17 | ch3:1087-1095 | Σ over scene points i ∈ O_kᵛ "paired with its CAD samples p_i" | The residual sum runs over the part's **CAD samples**, each matched to its nearest owned scene point (model→scene). n_i is that **scene point's** normal, and ω is that point's view weight. "Equal views" is therefore approximate. | multiview_refine.py:390-395, 404-407 | N |
| 18 | ch3:1130 (alg:mv_refine) | "T_k ← exp(ξ) T_k" | The update is (Rodrigues(ω), t), not the SE(3) exponential. Equal to first order. | multiview_refine.py:447 | N |
| 19 | ch3:1143-1145 (eq:observability) | J_i = [(p_i × n_i)ᵀ/ρ, n_iᵀ] | p_i is the posed sample **relative to the part centre**; n_i is the matched scene normal. H uses the normalised view weights only (no Welsch weights, no prior or overlap rows). | multiview_refine.py:372-377, 459-461 | N |
| 20 | ch3:1153-1156 | relative 5 % rule: the plate's in-plane rotation "was refused although its resting contact pinned it" | Correct, but this was while the **resting contact was on**. The draft has just said it is off (ch3:1114-1117). Say "then on". | multiview_refine.py:99-104; notes/thesis_notes.md:726-730 | N |
| 21 | ch3:1157-1158 | "A tube's spin about its own axis is named symmetric and is exempt" | Only an **uncut** tube: `symmetry_axis` returns None when a `base_cut` exists. A mitred or saddle-cut tube's weak spin is **not** exempt from the relative-pose rule. Note that this and #1 are swapped. | multiview_refine.py:507-525; part_shapes.py:67-71; weldgen_registry.py:611-621 | I |
| 22 | ch3:1182-1185; ch3:1229 (alg:mv_selfcal) | S "is the information about d"; keep directions with "1/√λ ≤ 0.5 mm" | The information is **S/σ²** (σ = 1.5 mm). With λ = eig(S), σ_dir = σ/√λ, as in eq:observability. | multiview_refine.py:806-816 | N |
| 23 | ch3:1228 (alg:mv_selfcal) | solve for (ξ₁…ξ_K, d) "by Welsch-weighted least squares" | Add: **no prior** on the parts; ≤ 20 IRLS steps; ν from max(P90, 2σ), × 0.7 → σ. | multiview_refine.py:723-799 | N |
| 24 | ch3:1190-1191; ch3:1232-1235 | refines again "up to four times, until the remaining estimate is below 0.5 mm" | If d′ is still ≥ 0.5 mm after four passes, the last refinement is used anyway (normal rules, no refusal). The recorded D then holds a last increment that was never refined. | multiview_refine.py:597-629 | N |
| 25 | ch3:1203-1204 | "Each run records its estimate and its 3×3 information" | Only when `mv_record_selfcal` is on **and** ≥ 1 direction is determined. D is the converged total; I is the **first** estimate's information. | multiview_service.py:330-338; multiview_refine.py:623-628 | N |
| 26 | ch3:1211-1215 | written "only when at least three runs agree"; repeated captures are not independent; pre-recalibration runs "learned the depth error" | The code keys runs **only** by the extrinsic file's sha1. It neither requires distinct scenes nor excludes runs made before a depth recalibration ("Key the history by scene" is still a todo). The file is written by hand with `selfcal_extrinsic.py --write`. Say so, or the caveats read as implemented. | selfcal.py:122-160; scripts/selfcal_extrinsic.py:42; notes/todo.md:281-288 | N |
| 1 | ch3:1254 | "for an uncut tube" | see 1a | | E |
| 27 | ch3:1257 | "the pose between **two touching parts**" | **Every pair** of parts is checked, touching or not. | multiview_refine.py:928-951 | N |
| 28 | ch3:1265-1267 | "fit was three times worse … the rules refused both parts" | RMS 0.96 vs 0.35 mm (2.7×). The refusal came from the **relative-pose** rule ("fit-up changed 11.4 mm / 2.0°"), not the fit rule. | notes/thesis_notes.md:901-905 | N |
| 29 | ch3:1268 | "a rotation that was mostly its **symmetric** spin" | E2 is mitred, so its spin is not a symmetry (the cut breaks it). It is exempt from the size limit because it does not move the seam. | multiview_refine.py:905-911 | N |
| 30 | ch3:1271-1273 | the resting gap "is measured along the whole contact" | Measured at the part's **sampled** contact points: ≤ 200 samples within 8 mm of the neighbour, facing down within 20° and opposite the neighbour's surface, ≥ 3 required. Reported as (min, max). | multiview_refine.py:342-353, 466-475 | N |
| 31 | ch3:1274-1275; A:56 | "ISO 5817:2023, Table 1, no. 617 … throat a = 0.7 t" | In the standard the row is **No. 3.2** (ISO 6520-1 ref. 617), and its formula uses the **actual** throat a_A. The code uses a design throat 0.7·t_min. The coefficients and caps of eq:iso_gap match the standard exactly. | WG/../notes/ISO 5817_Ed_4_2023.pdf (pdftotext: Table 1 row 3.2, symbols a/a_A); multiview_refine.py:826-838 | N |
| 32 | ch3:1311-1312, 1330-1333; ch3:1349 (alg:registry) | "derived automatically … and verified, never guessed"; "offline: verify … 0.25 mm, volume 1 %" | Hand-written entries are also kept (and re-verified). The both-ways check runs only with `build_weldgen_registry.py --verify` and only sets `verify_failed`, which nothing reads. All current entries pass at 0.037–0.067 mm and ≤ 0.24 %. | weldgen_registry.py:579-600; scripts/build_weldgen_registry.py:38-55; models/weldgen_objects.json | N |
| 33 | ch3:1318-1319 | sweep axis = smallest eigenvector of Σ A_f n_f n_fᵀ | Re-estimated four times on the faces with \|n·a\| < 0.15, so the end faces do not pull it. | weldgen_registry.py:181-194 | N |
| 34 | ch3:1333-1335 | "**The check** [both-ways verification] caught a drawing error" in SP3 | The first SP3 was refused by the **derivation's constant-wall measurement** (wall 2.4–5.0 mm, "refused with the range"), not by the both-ways verification. Move the sentence after "a band whose wall varies … is refused". | notes/curved_seams_plan.md:117-119; test/test_curved_registry.py:131-139; weldgen_registry.py:496-503 | I |
| 35 | ch3:1386-1388; ch3:1407 (alg) | non-weldable "if the dihedral is out of range, … shorter than 10 mm, or … no torch axis" | Also `fixture_contact` when a fixture is involved. The dihedral range is 30–170°. The torch axis may tilt up to 45° off the bisector and must face both members. | seam_from_registration.py:287-302; WG/accessibility.py:44-74, 202-233 | N |
| 36 | ch3:1390-1392 | the post-passes "**remove**" the toes and cross-runs | They **mark** them non-weldable (`toe_of_centreline`; cross-runs rejected) and keep them as negatives. | seam_from_registration.py:338-340; WG/accessibility.py:636-681, 693-719 | N |
| 37 | ch3:1430-1432 | band: x = p₀ + λ z_w, λ = −(nᵀp₀ + d)/(nᵀz_w) | The formula is correct, but here p₀ is the wall point at local z = 0 and d the offset in nᵀx + d = 0. Eq. ellipse_seam (ch3:1420-1424) writes the face as nᵀx = nᵀp₀ with p₀ **on the plate**. Rename or define. | curved_seams.py:283-288 | N |
| 38 | ch3:1461, 1465 (alg:curved_seam) | keep the (F, E) with the smallest max\|g\| ≤ 2τ; "a point lies off the plate" | Tube-on-tube also needs r_branch < r_main and the curve on the main tube's length (± τ). "Off the plate" means outside the broad-face rectangle grown by τ. | curved_seams.py:111-116, 190, 211-212, 227-230 | N |
| 39 | ch3:1494-1497 | PCA fallback "for a part without a registry entry. It is used only when allowed …; … made every curved part fall back to it **silently**" | When mode A raises (any part without a usable entry, or a weldgen import failure), the **whole assembly's** seams come from radius-PCA on the SEPC. The fallback is allowed by default (`weld_fallback_pca` true). In the 2026-10-09 incident the node **logged one warning**: the failure was that the result looked normal. | scripts/icp_pose_refiner_node.py:470-478, 1130-1142; seam_from_registration.py:79-95; notes/thesis_notes.md:343-347 | I |
| 40 | ch3:1508-1509 | "On an open seam both effective ends are tacked and the rest are spaced evenly" | The effective ends sit at margin = max(2t, tack length) from the ends. If L − 2·margin ≤ tack length, there is **one** tack at L/2. | WG/tacks.py:100-112 | N |
| 2 | ch3:1509-1511 | "does not depend on how the curve happens to be parametrised" | see 1a | | E |
| 3 | ch3:1511-1512 | "phase changes when the same assembly is saved again" | see 1a | | E |
| 41 | ch3:1515-1517 | order: "the ends first, then the middles by recursive bisection" | That is for open seams. A closed seam is ordered as **opposite pairs**, the pairs in bisection order over the half-loop. | WG/tacks.py:14-16, 165-169 | N |
| 42 | ch3:1518-1519 | seam by seam in tack-number order, "which is the shorter route" | No comparison is measured. The seams are taken in seam-id order (weldable first, longest first), not optimised. | marking.py:531-534; seam_from_registration.py:403-405; notes/pen_marking_plan.md:104-109 | N |
| 43 | ch3:1570-1571 | fixtures "entered as a few pen-tip points on their tops" | Supported (or as explicit boxes), but none is configured in use (`"fixtures": []`). | collision.py:303-344; config/marking.json:60 | N |
| 44 | ch3:1576-1577 | arm and tool against "… the table" | The base and shoulder capsules are exempt from the table check (they stand on it). | collision.py:383-384, 409-424 | N |
| 45 | ch3:1586-1591 | single capsule (r_h = 42, ℓ ≈ 61) "cleared by about 1 mm"; "3 mm … under which a square fillet allowed at most about 3 mm" | The two numbers come from **different pen-tip calibrations**. The 3 mm was set with the tip at 183.78 mm: ℓ = 63.8, 63.8·sin45° − 42 = 3.1 mm. The 1.1 mm is with the re-measured tip at 180.91 mm (ℓ = 60.9). Read together they contradict each other for one model, so name both tips. | config/marking.json:8-11; config/pen_tool.json:9-19; notes/thesis_notes.md:951-955 | I |
| 46 | ch3:1594-1595 | the pen "may touch the part, and the table under a thin plate, but never reach the table itself" | Wording. The pen's pairs with the part and with the table are exempt from the clearance (it may come within 3 mm of the table under a thin plate), but the pen at or below the table plane counts as a collision. | tack_reach.py:286-309 | N |
| 47 | ch3:1597-1599 | "in Python one plan spent most of its minutes in it. The model was therefore ported to C++" | The order of events is reversed. The port (2026-10-06) was made because APS has no Python bindings and a Python callback would serialise its threads. The "3 of a plan's 3.5 minutes" finding (2026-10-14) came **after** the port and moved the remaining Python re-checks (edges, splines, TOTG samples) onto the C++ model. | notes/thesis_notes.md:464-470, 318-324; marking.py:100-111 | N |
| 48 | ch3:1604-1605; ch3:1761 (alg); A:74 | edges "checked at a resolution of 0.01 rad per joint" | This is true in the C++/OMPL checker (≤ 0.01 rad on every joint). The Python `_edge_valid` (straight edge, Cartesian chain, shortcuts, splines, APS re-checks) steps 0.01 in a **weighted** norm with w = (1, 1, 0.8, 0.5, 0.5, 0.3): up to 0.0125 rad elbow, 0.02 rad wrists 1–2 and 0.033 rad wrist 3 between checks. | kinematics.py:421-425, 466-481; src/transit_cpp.cpp:479-480; notes/aps_transit_plan.md:38-41 | N |
| 49 | ch3:1622-1624; C:86-88 | "a tilt away from the closer plate is what makes a tack on a leaning plate reachable" | By C's own formulas, with the holder on the bisector both clearances are equal, and the smaller one is largest at ϑ = 0. A tilt pays only when one side binds: the seam's approach already off the bisector (`_clear_axis` tilts up to 45°), the asymmetric camera arm/body boxes, or a leaning plate's far part. State the condition. | WG/accessibility.py:202-233; config/pen_tool.json:34-73; config/marking.json:13-17 | N |
| 50 | ch3:1629; ch3:1670 (alg:reach); A:67 | descent "checked at six steps" | `lin_steps` = 6 segments, i.e. **5 interior** IK poses plus the two end poses, each ≤ 0.5 rad from the previous. | tack_reach.py:359-371; config/marking.json:55 | N |
| 4 | ch3:1650-1651 | "worth about 9° of base-joint motion" | see 1a | | E |
| 51 | ch3:1659-1660 | false 1.5 N "contact" "84 mm above the part" | It was 82–84 mm **before the registered point along the pen**, i.e. **66–68 mm above** the part. The note's heading says 84 mm, its evidence says 66–68 mm. | notes/thesis_notes.md:45-52 | N |
| 52 | ch3:1709-1710; ch3:1762 (alg:transit) | Cartesian stage "between two tacks" / "IF the transit joins two tacks" | It is tried on **every transit to a tack**, including the first one from the arm's current pose. It is not tried on the way home or in the multi-view planner. | marking.py:753-756, 780-782; multiview.py:398 | N |
| 53 | ch3:1736-1737 | "Every path is … smoothed by a centripetal Catmull–Rom spline" | Only APS/RRT paths are smoothed. Straight and Cartesian transits are not. | marking.py:262-273, 293-313 | N |
| 54 | ch3:1742-1743 | repeated runs "varied from 0.26 to 1.6 m of tip travel" | This range comes from the `marking.py` docstring. The dated record for one transit (E2) gives 497–1,873 mm at a 2 s budget, 340–743 mm at 3 s and 353–595 mm at 5 s. From 100 mm departure points it gives 194–206 mm (but 432–1,623 mm on 1→2). Quote one setting. | marking.py:124-127; notes/thesis_notes.md:279-288; notes/curved_seams_plan.md:364-373 | N |
| 55 | ch3:1752-1753 | uncapped margin: "every transit fell back to the slow planner" | "**most** transits" logged "Invalid start/goal". | notes/thesis_notes.md:305-308, 318-324 | N |
| 56 | ch3:1753-1754 | a planned transit "is joined from where the arm actually stands instead of being re-planned" | Only when the arm is within 0.35 rad of the path's start on every joint and the joining edge is clear. Otherwise it is re-planned. | scripts/tack_marking_node.py:204-220, 369-379 | N |
| 57 | ch3:1779-1781 | zero drift "by 7 N after one power cycle, which alone would trigger the abort limit" | 7 N is below the 8 N limit. The record is that "every first transit aborted" (the reading moves during a move). Say "brought it within 1 N of the abort limit". | notes/thesis_notes.md:972-973; motion.py:78, 281-284 | N |
| 58 | ch3:1800 | dot "held for 0.5 s if the force is moderate" | If \|F\| < 2 × the touch force (3 N). | scripts/tack_marking_node.py:423-426 | N |
| 59 | ch3:1845 | "only one frame per part crosses the network" | Plus one frame per re-seed while lost. | scripts/fp_tracker_node.py:466-479, 576-608 | N |
| 60 | ch3:1866-1888 (tab:nodes) | the nodes of the cell | `tack_reachability` is a command-line script ("no ROS, no motion"), not a node. `detection_marker_node`, which `pointcloud.launch.py` starts, is missing, as are the UR driver and RViz. | scripts/tack_reachability.py:1-14; launch/pointcloud.launch.py:265-280 | N |
| 61 | ch3:1901 (tab:rates) | "Re-seed while lost: every 3 s" | **At most** every 3 s, never while one is in flight. A successful re-seed restarts TRACKING directly. | scripts/fp_tracker_node.py:198-199, 466-479; scripts/scripts_in_foundationpose/fp_tracking.py:187-189 | N |
| 5 | ch3:1917-1918, 1938 | dry run by default | see 1a | | E |
| 62 | ch3:1927-1928 | "what the Python side computes is geometry, and time comes from the C++ side" | Only free moves (transits, home) are timed by the C++ TOTG service. Descents, retracts and stroke chunks are timed in Python (`time_descent`), multi-view transits by `time_joint_path`, and the TOTG fallback is the Python trapezoid. Also "both" follows a list of three components. | marking.py:326-371, 396-429; multiview_service.py:282, 294; scripts/tack_marking_node.py:193-202, 387-392, 439-444, 491 | I |
| 63 | A:25 | rate and acceleration caps 15°/s, 60°/s² | These are commanded values. Until `ur5e_jacobian` is fixed the arm achieves half (7.5°/s, 30°/s²). Row 23 points to the Jacobian note; this row does not. | kinematics.py:296-303; notes/thesis_notes.md:15-32 | N |
| 64 | A:38 | "re-seed every 3 s" | See #61. | as #61 | N |
| 65 | A:39 | save window "stationary tail 8 mm / 3° / 3 frames, at most 75 frames" | The "3" is the run of **3 consecutive disagreeing poses that ends the tail**. Also missing: at least **5 poses at rest** (`save_pose_min`), otherwise the last pose is saved unaveraged; the last pose must be ≤ 1 s old. | pose_stats.py:47-75; scripts/icp_pose_refiner_node.py:393-394, 1883-1913 | I |
| 66 | A:40 | robust mean "3.5 MAD, 3°; at least max(3, ⌊n/3⌋) inliers" | The gate is 3.5 **robust σ** per axis (3.5 × 1.4826·MAD). The inlier count is a **fallback**: with fewer inliers, all poses are used. | pose_stats.py:86-91 | I |
| 67 | A:41 | ICP row | Add d_max 20 mm (fitness/RMSE only, no cutoff in the robust mode) and 30 iterations. | scripts/icp_pose_refiner_node.py:362-373; icp.py:742-753 | N |
| 68 | A:42 | save-time ICP "5 clouds; fitness ≥ 0.2; ≤ 20 mm, 8°" | **Up to** 5 accepted runs within 3.5 s; one is enough. For an axisymmetric pipe the spin is removed before the 8° check. | scripts/icp_pose_refiner_node.py:418-422, 1942-2019 | N |
| 69 | C:8 | Δz ≈ (fb/d²) δ | The sign is lost: Δz ≈ −(fb/d²)δ, so an overestimated disparity shortens the depth. The magnitude is right. | (math) | N |
| 70 | C:11-12 | "a fit Δz = a + kz² needs at least three ranges" | **Two** ranges determine a and k. A third is what **tests** the quadratic form against a linear one: the middle point was predicted to 0.2 mm, against 0.7 mm for a line. | notes/thesis_notes.md:802-814 | I |
| 71 | C:25 | vertical pen: R_iᵀn ≈ e_z | With tool +z along the pen towards the tip and n pointing up, it is ≈ −e_z. Sign only. | table_probe.py:135-155 | N |
| 72 | C:50-54 vs ch3:480-481 | C: bound arcsin(‖Δc‖/‖x−c‖); ch3: "at most ‖Δc‖/‖x−c‖ radians" | C is right. ch3's bound is only first-order (arcsin u ≥ u), so align ch3 ("about", or the arcsin). | (math) | N |
| 73 | C:57-58 | "a linear least-squares problem" | It is the **linearisation** at the solution (20 IRLS steps, Welsch- and view-weighted rows); the covariance is σ²S⁻¹ with those weights. | multiview_refine.py:783-811 | N |
| 74 | C:69-78 | 1-Lipschitz ⇒ "a pose that does worse than this bound has met something that the straight translation cannot explain, and the descent check refuses it" | The argument covers tool primitives that translate rigidly and have a true distance (capsule–box, anything–table). For those the bound can **never** be violated. The code applies it to **every** non-pen pair, so what it can actually refuse are pairs outside the argument: box–box tests (0/∞, e.g. the camera body against a part box, the case in `marking.py`'s docstring) and the arm's capsules and tool–arm pairs, which do not translate rigidly. | marking.py:604-644; collision.py:261-271, 409-424 | I |
| 49 | C:86-88 | tilting "away from the nearer plate" | see #49 | | N |
| 75 | C:91-95 | coverage: "distance from that axis ≤ w, and the box covers it" | Also needs the along-axis bound \|ρ cos(φ−φ_j)\| ≤ r, which holds since ρ ≤ r. Everything else confirmed. | collision.py:58-62, 76-90 | N |

## 2. Claims checked and CONFIRMED

**Multi-view intro and views**
- 4 views, 0.40 m, elevations {45, 60}°, azimuth step 30°, roll step 90°, 96 candidates — multiview_service.py:63-67; multiview.py:52-75, 297-308.
- Seam-band weight 1 (1–30 mm), 0.1 elsewhere, samples < 1 mm dropped — multiview.py:70-72, 171-173.
- FOV 69°×42°, incidence ≤ 60° — multiview.py:65-69, 186-190.
- Gains ρ = (1, 0.5, 0.25, 0, …), lazy feasibility, 30° separation, 98 % roll siblings with least joint travel from home — multiview.py:73, 336-361.
- IK on the locked branch; ≥ 20 mm (transit clearance) from parts, fixtures and table — multiview.py:292, 317-329; multiview_service.py:262-263.
- Nearest-neighbour order with 2π-equivalents inside the limits and the cable window — multiview.py:256-272, 382-392.
- Refuse with < 2 views or no way home — multiview.py:63, 420-426.
- 30° candidates "doubled the planning time"; 75° views see the ear's faces beyond 60° incidence — notes/multiview_refine_plan.md:543-546.

**Capture**
- Tracking paused, force-watched TrajectoryExecutor, joint speed 0.3 rad/s, settle 0.5 s, median of 16 fresh organized clouds — multiview_service.py:68-71, 110, 275-291.
- A failed or aborted move stops where it is — multiview_service.py:283-284.
- Views stored in the camera frame — multiview_capture.py:4-16, 65-80.
- Crop 30 mm, cut 2 mm, voxel 3 mm, normals towards the camera — multiview_capture.py:40-46; icp.py:194-217.

**Joint refinement**
- Ownership: nearest posed surface, none beyond 15 mm, dropped within 4 mm of a second part, recomputed between rounds — multiview_refine.py:69-70, 280-291, 567-568.
- Prior as MAP with Σ = diag((1°)², (3 mm)²) scaled by σ² — multiview_refine.py:380-381, 435-439.
- Overlap one-sided against the true shape: ≤ 200 samples within 8 mm, ε = 0.5 mm, w_o = 10 — multiview_refine.py:73-74, 94-95, 342-353, 411-422.
- Matching ≤ 10 mm with the 60° normal gate — multiview_refine.py:60, 65, 395.
- Welsch width max(3·median, P90, σ), × 0.7 down to σ = 1.5 mm, with the P90 rationale — multiview_refine.py:59, 398-405, 448.
- Centre-based correction; 1° at 0.5 m ≈ 8.7 mm — multiview_refine.py:372-376.
- GN ≤ 30 iterations, 2 rounds, most-owned part first, early stop at 0.2 mm / 0.05° — multiview_refine.py:96-97, 121-122, 553-570.
- True shapes: slab box; tube wall between the radii, hollow bore, flat top, cut base; band wall along the spine; exact on faces, approximate at edges; why boxes failed — part_shapes.py:4-26, 104-171.
- Resting contact off; the synthetic 3 mm gap was pulled below 1 mm unwarned — multiview_refine.py:75-90; notes/thesis_notes.md:724-726.

**Observability**
- ρ = 90th-percentile sample radius; σ_dir = σ/√λ ≤ 0.5 mm, absolute — multiview_refine.py:104, 377, 486-504.
- The relative 5 % rule and its failure — notes/thesis_notes.md:726-730.
- Unmeasured directions held by the prior and fed to the relative-pose rule — multiview_refine.py:575-576, 936-951.
- The 1–3 mm in-plane slide limitation and the todo (noise model for steep points) — notes/thesis_notes.md:742-746; notes/todo.md:278-280.

**Self-calibration**
- r = nᵀ(T p − q + R_v d) and the cloud shift R_v d (sign chain through `render_synthetic` and the extrinsic) — multiview_refine.py:721-781; multiview_capture.py:196-207.
- Schur complement with a pseudo-inverse; directions with σ ≤ 0.5 mm kept, the others named and zeroed — multiview_refine.py:802-816.
- Synthetic d = (3, −2, 1) mm tilted the base 1.2° and was accepted (before the camera rule) — notes/multiview_refine_plan.md:442-443; test/test_multiview_touching_parts.py:50.
- The per-view estimator gave (7, 13, −15) mm — multiview_refine.py:738-740; notes/multiview_refine_plan.md:524-526.
- Online loop: threshold 0.5 mm, ≤ 4 passes, q ← q − R_v D and t_k⁰ ← t_k⁰ − R_scan,k D, judged against the corrected prior — multiview_refine.py:115-120, 590-621.
- Fake ~8 mm fit-up change without the prior correction — multiview_refine.py:592-596; notes/thesis_notes.md:650-668.
- Combination d* = (ΣI)⁺ΣI d and t_EC ← t_EC − R_EC d* (sign) — selfcal.py:10-14, 57-61, 112-119.
- ≥ 3 runs, spread and leave-one-out ≤ 1 mm, same sha1, promoted by hand — selfcal.py:46-50, 122-160.
- The vertical weak direction; the ~7 mm lowering vs ~2 mm high, 4–5 mm over-correction, not promoted — multiview_refine.py:729-736; notes/thesis_notes.md:618-630.

**Acceptance and fit-up**
- 10 mm / 5° beyond the camera's share; spin > 0.5° warned — multiview_refine.py:124-125, 903-914; multiview_service.py:80-81.
- Fitness < 0.2 — multiview_refine.py:126, 458-463, 915-916.
- Overlap > 1.0 mm and > saved + 0.2 mm, else applied with a warning — multiview_refine.py:917-924.
- Relative pose 2 mm / 1° with > 2 mm along weak directions — multiview_refine.py:928-951.
- Camera rule with online self-cal off — multiview_refine.py:630-637.
- Fallback cascade "until nothing changes" — multiview_refine.py:644-684.
- ISO eq:iso_gap: coefficients, caps, the t ≤ 3 branch (0.1a + 0.2/0.3/0.5) — WG/config.py:197-209; ISO 5817:2023 Table 1 row 3.2.
- 1.06 / 1.62 / 2.68 mm at B/C/D for 8 mm plates; a warning, never a rejection; level C in use — multiview_refine.py:841-858; multiview_service.py:90.
- The same function grades WeldSet's joints — WG/config.py:295-312.
- The TODO about t from the bounding box — multiview_refine.py:822-823.

**Registry**
- Slab: OBB with sorted axes; exact (corners ≤ 0.05 mm, volume ≤ 1 %) or envelope (on the surface, fill ≥ 90 %) — weldgen_registry.py:43-102.
- Tube: circle fit, two radii, ends classified flat/plane/cylinder within 0.05 mm, flat top required — weldgen_registry.py:232-315.
- Swept slab: rounded-rect spine from weldgen; open constant-wall band with a clamped cubic B-spline, 4–16 control points, joint solve, start by alternation, first fit ≤ 0.025 mm wins — weldgen_registry.py:376-459, 462-556.
- Varying wall refused with its range — weldgen_registry.py:496-503.
- Order slab → tube → extrusion — weldgen_registry.py:559-576.
- Both-ways check needed (too long or too thick) — weldgen_registry.py:124-142.
- Stale at > 0.5 mm, refused in mode A, with the 2026-10-15 incident — part_shapes.py:184-211; scripts/icp_pose_refiner_node.py:1165-1173.

**Planar seams**
- eq:plane_intersection — WG/accessibility.py (intersect_planes); seam_from_registration.py:261-264.
- τ = 10 mm, clipped 0.5–30, launch default 10 — seam_from_registration.py:98-123; launch/pointcloud.launch.py:373.
- Bench 8–11 mm inside / 1–8 mm above — seam_from_registration.py:17-27.
- Slack only on the near-constant in-face axis; 44 of 250 mm — seam_from_registration.py:175-226.
- Face extent yes/behind/short and the lap-sheet limit — seam_from_registration.py:148-172, 277-288.
- Signed near-edge fit-up, + gap / − penetration — seam_from_registration.py:141-145, 281.
- Minimum length 10 mm, 5 sample points — seam_from_registration.py:99, 293-302.
- Parallel faces: coplanar butt/edge candidate, or facing within τ as a non-weldable record — seam_from_registration.py:247-259.
- Canonical orientation for tack counting — seam_from_registration.py:375-385.

**Curved seams**
- eq:ellipse_seam (derived independently) and the 10° minimum — curved_seams.py:42-45, 168-186.
- Both faces × both ends; 2τ near miss; per-point n_a, n_b, bisector approach; dihedral 65–115° round a 25° tilt — curved_seams.py:131-163, 174-195.
- Saddle for tube-on-tube — curved_seams.py:209-250.
- Band walls traced along z_w (exact for a ruled wall) — curved_seams.py:268-312.
- ≥ 95 % cone clear; bore negative unless ≥ 200 mm (WeldSet 80 mm) — WG/verify_curved.py:229-250; WG/accessibility.py:45-51; curved_seams.py:38-40; seam_from_registration.py:123.
- Off-plate refusal; unsupported pairs reported — curved_seams.py:190-195, 409-434.
- Near-miss incident (69.4° vs 70°, 10.5 vs 10 mm) — notes/thesis_notes.md:351-353.
- Unmodelled 7 mm plate (0.9–4.6 mm gap, 9.7 mm early) — notes/thesis_notes.md:959-964.
- Install-list test — test/test_install_list.py:1-22.

**Tacks**
- tackrule-0.1 unchanged; t = thinnest workpiece of the scene (a tube's wall); 12 mm and 32 mm tacks — WG/tacks.py:3-16, 100-151; WG/geom.py:599-601; seam_from_registration.py:409-447.
- Closed seams: even n ≥ 4 with a sha256 phase — WG/tacks.py:80-84, 115-125.
- tack_no along the seam; order with same-class interleaving — seam_from_registration.py:464-470; WG/tacks.py:174-184.
- Per-tack approach interpolated on curved seams — seam_from_registration.py:474-492.
- Pen visits seam by seam in tack_no order — marking.py:531-534.
- todo items: tack length 4t; heat ordering — notes/todo.md:341-342, 350-387.

**Collision**
- Six arm capsules with radii 50–75 mm and offsets 0.138/0.007 m — collision.py:40-45, 348-364.
- Tool primitives (flange box, holder capsules r 48 / 20.5 mm, pen r 8 mm ending at the tip, camera arm and body boxes, +3 mm margin, body from the bench) — config/pen_tool.json:15-24, 34-73.
- Six rotated boxes, half-width r·sin15°, 3.3 % / 41 % — collision.py:58-62, 76-90.
- Cut tube 12–36 sectors (39 mm under a 25° mitre) — collision.py:68-73, 91-103.
- Band chain within wall + 1 mm — collision.py:63-65, 106-147.
- Golden section to 1e-4 — collision.py:208-223.
- Box–box intersection only — collision.py:261-271.
- Table from the lowest point — collision.py:274-286.
- Self pairs base/shoulder/upper arm — collision.py:383, 420-423.
- 42 mm capsule and 15.5 mm; the corner opened to 96° — notes/thesis_notes.md:951-955.
- 20 mm transit clearance after the 9 N hit; 0.9 × end-pose clearance, ≥ tack clearance — marking.py:65-75; config/marking.json:18-21, 51-52.

**C++ port**
- pybind11, GIL released, spec exported from Python — src/transit_cpp.cpp:14-25, 517; collision.py:442-476.
- Equivalence on random and bisected near-contact configurations, 1e-9 distances, ~590×, the output-array bug and the last-bit tie — test/test_transit_cpp.py:50-58, 115-125; notes/aps_transit_plan.md:30-37; notes/thesis_notes.md:470-486.

**Reachability**
- Branch lock (lift, elbow, wrist 2) — config/marking.json:45-47; tack_reach.py:157-183.
- DLS from the previous pose, home and 6 random seeds — tack_reach.py:166-183.
- Work angles {0, ±10, ±20}°; rolls every 30°; approach 35 mm; pose-only prechecks; ≤ 1.2 rad — config/marking.json:48-56; tack_reach.py:312-358.
- Straight seams: one (ϑ, φ) by largest minimum clearance, then travel + 3 per near-singular pose — tack_reach.py:381-391, 514-554.
- Viterbi recursion as written, W = diag(2, 2, 1.5, 1, 1, 0.5), δ₀ = 10 mm, κ = 0.15, wrist 3 not wrapped, start at home or the previous seam, backtracking, O(nJ²) — tack_reach.py:396-477, 493-500.
- 115° wrist turn on C1 — notes/thesis_notes.md:371-378.
- 65–90° apart, 0.9 m swings — tack_reach.py:430-432.
- Unreachable kinds (SP3 concave lobes; S3 far side) — notes/thesis_notes.md:384-387.
- Cable window ±180° about home and −202°/−287° vs ±151° — tack_reach.py:83-88, 198-209; config/marking.json:39-43; notes/thesis_notes.md:55-60.

**Descent and transit**
- Descent IK chain every 2 mm, 5 mm overshoot in use (library default 3 mm) — marking.py:374-393; scripts/tack_marking_node.py:126.
- Overshoot relaxation and the table end (SP3: 4.9 mm, 35°) — marking.py:604-644; notes/thesis_notes.md:63-66.
- Roll retries ±30/60/90° — marking.py:593-601, 706-725.
- Departure chain to 100 mm in 5 mm steps, falling back to the approach point — marking.py:647-667, 736-748; config/marking.json:50.
- Cascade: straight edge, then Cartesian. The Bézier control point x_m + 2βb puts the curve midpoint at x_m + βb; b = −(z₀ + z₁)/‖·‖; β ∈ {0, 30, 60, 100} mm; 5 mm / 3° steps; 0.3 rad jump — marking.py:114-180.
- APS: 4 RRT-Connect, hybridise + shortcut, 8 paths, ≥ 3 s then the first exact solution, ≤ 10 s, scaled y = Wq so cost = Σ‖WΔq‖ — src/transit_cpp.cpp:21-25, 493-525; scripts/tack_marking_node.py:112-118.
- Margin min(2 mm, room/2) ≥ 0, re-check at the nominal clearance — marking.py:212-233.
- Then C++ RRT-Connect + shortcut, then Python (3 tries) — marking.py:281-314.
- Spline kept only if clear — marking.py:78-97.
- TOTG 0.3 rad/s, 0.5 rad/s², 5 mrad, 0.02 s, every sample checked, trapezoid fallback — motion.py:82-86, 224-270; scripts/tack_marking_node.py:105-106, 193-202.
- 3.5/3.9 rad → 257/772 mm; Cartesian 141–211 mm; joining instead of re-planning — notes/thesis_notes.md:279-298.
- Python bindings and thread-serialisation reason — notes/thesis_notes.md:464-466.

**Force-gated marking**
- Bias averaged over 0.5 s at rest after the transit — motion.py:80, 179-189; scripts/tack_marking_node.py:385.
- Speeds 50 / 20 / 2 mm/s with 35 mm and 6 mm zones — scripts/tack_marking_node.py:122-125, 387-392; marking.py:396-429.
- One goal, polled every 2 ms, cancelled at \|F − F_bias\| > 1.5 N, the joints at the cancel recorded — motion.py:338-353; config/pen_tool.json:32.
- F_touch = 3 × ~0.5 N noise — notes/pen_marking_plan.md:407.
- δ_c sign — marking.py:25-28, 471-473.
- No contact through the overshoot — scripts/tack_marking_node.py:412-414.
- 8 N abort / 40 N release — motion.py:78-79, 337.
- 0.35 rad gate — motion.py:81, 295-298.
- Stroke: 10 mm chunks, 2e-5 m/N, ≤ 0.2 mm, stop after 2 chunks < 0.4 N, hold = touch force, per-tack curve pieces, retract to the departure distance, record written — scripts/tack_marking_node.py:129-137, 416-447, 450-516; marking.py:537-575.
- "Dots measure, strokes demonstrate" — notes/thesis_notes.md:265-268.
- 4–11 mm → ±5 mm — notes/pen_marking_plan.md:71-73.

**Implementation**
- Desktop RTX 5070 Ti / laptop RTX 4060; Ubuntu 22.04 / Py 3.10 container vs Jazzy / Py 3.12; Humble–Jazzy DDS — notes/readme_perception.md:414, 443-445.
- WebSocket on localhost, ≤ 2 frames in flight, newest-wins slot, raw RGB/depth, header fields — scripts/fp_tracker_node.py:183-190; fp_stream.py:7-50; notes/realtime_fp.md:110-115.
- Camera 1280×720 @ 30 Hz with depth_image_proc — launch/pointcloud.launch.py:188-233, 396-403.
- URDF on the calibrated kinematics; static extrinsic TF — urdf/ur5e.urdf.xacro:22, 31; scripts/camera_extrinsic_tf_publisher.py:2-6, 59.
- Bridge stamps the detection with the frozen frame and keeps it on refusal — scripts/foundationpose_bridge_node.py:645-678.
- Aim 125 Hz, relay 500 Hz to the forward velocity controller — scripts/camera_aim_node.py:105; src/aim_rate_relay.cpp:38-44.
- UR5e controller 500 Hz — /opt/ros/jazzy/share/ur_robot_driver/config/ur5e_update_rate.yaml.
- TOTG service needs no robot model — src/totg_service_node.cpp:1-11.
- Plan 478 s → 17.6 s — notes/thesis_notes.md:322-324.
- The silent July hand-eye solve — README.md:1123.

**Appendix A rows 46–78**
- All values equal the in-use values: multiview_service.py:58-92; multiview_refine.py:58-128; multiview_capture.py:40-46; selfcal.py:46-50; seam_from_registration.py:98-123; curved_seams.py:38-51; config/marking.json; config/pen_tool.json; scripts/tack_marking_node.py:105-137; motion.py:73-87; marking.py:114-118.
- The exceptions are #31, #48, #50.

**Appendix A rows 21–43** (delegated pass, spot-checked)
- Correct: gesture latch 6 frames / 1 s; dwell 1 s; motion hold 2 s; aim start 3 frames; K = 2.5, k_r = 1.5; deadband 5 % of the width, release at 0.4 ε; excursion ±35/±25° with 0.35 s look-ahead; optical-centre hold 2 s⁻¹ capped at 20 mm/s; DLS λ = 0.02, 0.5 rad/s; safety 0.3 s, 0.3 s, 15 N, 0.2 s; rates 125 / 500 Hz with relay 0.6 rad/s, 4 rad/s², 2 rad/s² stop after 0.1 s; SAM2.1 Hiera-S; PPF 600 points / 0.04 / 30 bins, ≤ 2000 points, voxel max(D/60, 1.5 mm), every 5th point, 12 hypotheses, ICP 50 iterations, τ = 8 mm, [0.45, 1.15]; FoundationPose 5 / 2 refinements, no symmetry; spin settling 1° → 0.05°, ≤ max(1.5 mm, ¼), ≥ 1 mm gain.
- Evidence: scripts/gesture_control_node.py:304-341; camera_aim.py:53-67; scripts/camera_aim_node.py:81-113, 360-391; src/aim_rate_relay.cpp:40-44; scripts/scripts_in_foundationpose/{fp_server,ppf_classifier,build_ppf_library}.py; curved_seams.py:319-320, 358-399.
- The exceptions are #63–68.

**Appendix C**
- C.2: (R_iᵀn)ᵀe + δ = −h_i matches `solve_tcp_error`; sin ϑ lateral sensitivity; 1 − cos ϑ = 0.134 at 30°, 0.293 at 45° — table_probe.py:135-175.
- C.3: L(s) = K e^{−sτ}/s, ω_c = K, PM = π/2 − Kτ; two 60 ms lags ≈ 2·arctan(0.15) = 0.298 rad ≈ 0.12 s at ω_c = 2.5; ch3's 68° (τ = 0.15 s) and 54° (τ = 0.25 s) — (math); ch3:507-516.
- C.4: the rotation-about-the-optical-centre argument and the arcsin bound — (math).
- C.5: Cov(d) = σ²(N_dd − N_dx N_xx⁻¹ N_xd)⁻¹ by block inversion. A pseudo-inverse is valid for a Gram matrix (range(N_xd) ⊆ range(N_xx)); the code always uses pinv — multiview_refine.py:802-804.
- C.6: the 1-Lipschitz inequality itself — (math).
- C.7: c = ℓ sin(θ/2) − r_h. Numbers: 60.9·sin45° − 42 = 1.07 mm (≈ 1.1); 50.9·sin45° − 20.5 = 15.50 mm (tip 180.91 mm; capsule ends at 120 / 130 mm). The tilt formulas sin(θ/2 ± ϑ) hold for θ/2 + ϑ ≤ 90° — config/pen_tool.json:31-61.
- C.8: 12 directions every 30°, so every φ is within 15° of one; corners at r√(1 + sin²15°) → 0.033 r (1.0 mm on C1); single box 0.414 r (13 mm on C1) — collision.py:58-62; notes/thesis_notes.md:376-385.

## 3. Cannot be verified from this workspace

- **WeldSet numbers.** The WeldSet number for "seam transferred from the registered model" (ch3:1300-1301, TODO M6 at L0) lives on the desktop.
- **Literature claims.** APS beating RRT* in equal time [luna2013aps], [liu2023rcim], [forney1973viterbi], [kunz2012totg] are not checkable against code.
- **Validity-check count.** "Hundreds of thousands" of validity tests per plan (ch3:1542): `validity_calls` is reported per transit, but no recorded number exists in the notes.
- **Bench-only numbers.** These exist only in the dated notes (not reproducible here): 9 N, 7 N drift, 84/66–68 mm false contacts, 115°, 69.4°, 0.9 m swings, 257/772 mm, the 7 mm over-correction, "three times worse".
- **"Shorter route".** That the seam-by-seam pen order is "the shorter route" (no comparison recorded).
- **Rates.** That the gesture recogniser actually runs at 30 Hz (the camera does). The force sensor's 500 Hz is assumed from the UR5e controller-manager rate.
- **Docker side.** Deployed Docker copies and environment overrides of fp_server/fp_track_server (SAM2 checkpoint, EST_REFINE_ITER, SYMMETRY_INFO, PPF_TAU), the PPF library file actually loaded (`Data/ppf_library.npz` not on disk), and FoundationPose's own `estimater.py`. Values above are the mirrored source defaults.
- **Hardware.** RTX 5070 Ti / RTX 4060 are stated only in notes.

## 4. Stale sources noticed (not draft errors; do not copy from them)

- **README** `tack_marking_node` row: `transit_budget_s` 2 s, "bias measured at the approach point", "descent at 20 mm/s". In use: 3 s, the departure point, 50/20/2 mm/s.
- **`notes/multiview_refine_plan.md:570-579`**: `mv_max_correction_deg` 4 and `mv_dry_run` true. In use: 5 and false.
- **`notes/todo.md:355-356`**: says the marking node visits tacks in `order`. It visits seam by seam, `tack_no` ascending.
- **`tack_reach.py:398`**: the "~9 deg" comment (#4).
- **`motion.py:281-284`**: "7 N … alone reaches abort_force_n" (8 N).
- **`scripts/tack_marking_node.py:32-33`**: "bias re-measured at each approach point".
- **`config/pen_tool.json`** `_comment`: "planner's FK is the nominal UR5e chain".
- **`marking.py`** docstrings: "re-checked with the PYTHON model" (it uses `fast_is_valid`); the 0.26–1.6 m figure (#54).
