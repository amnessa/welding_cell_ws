# Material map: where everything for each section is

*Built 2026-10-20 after the line-by-line review of both projects. For every section of the thesis:
where its material is, and what is still missing. Paths are relative to `thesis/` unless they start
with `ros2_ws/`, `weld_generator/` or `/`. "R:" points into the raw review reports in
`notes/review_2026-10-20/` (machine-generated: check a value in the cited file before using it).*

Missing-item tags:
- **ASK**: only the author knows it (printer, filament, pen, payload, measured dimensions).
- **DESKTOP**: the data are on the desktop (RTX 5070 Ti): WeldSet corpora, batch results, renders,
  the Docker image.
- **OPEN**: work not done yet (in `ros2_ws/src/admittance_control/notes/todo.md`).
- **DECIDE**: a decision for the author and the advisor (`THESIS_PLAN.md`, open decisions).

## Cross-cutting notes

| Need | Where |
|---|---|
| How to write (Kurt's reasoning, model cards, algorithm boxes, phrase bank) | `notes/writing_style_guide.md` |
| Where the thesis sits among the seven example theses; what lifts it | `notes/positioning.md` |
| WeldSet code defects (P1-P3) and the ICRA statements that do not match the code | `notes/weldset_code_review_2026-10-20.md` |
| Every dated incident with its numbers (newest first) | `ros2_ws/src/admittance_control/notes/thesis_notes.md` |
| The library's dated incidents in chronological order | R: B1a Appendix D |
| Values in use vs. library defaults | R: B1a Appendix A; thesis Appendix A (`tab:app_parameters`) |
| The 36 draft-vs-code discrepancies (all fixed in the 2026-10-20 draft except where marked) | R: B1a §4 |
| Bibliography check, the 21 corrected entries, pool papers worth citing, missing literature | R: D_papers Part 3-4 |
| The author's own notes on the draft | `cago_notes_about_current_thesis.md` (all six points are in Ch. 3) |

## Chapter 1: Introduction

| Section | Material | Missing |
|---|---|---|
| Motivation, problem, the two parts | `weld_generator/notes/thesis_direction_handoff.md` §2-4; the framing sentence in `notes/positioning.md` ("seams and tacks are computed from exact geometry, never detected") | DECIDE the 3-4 headline contributions |
| Contributions, outline | Ch. 3 summary; Ch. 4 goals | rewrite after Ch. 6 |

## Chapter 2: Background and Related Work

| Section | Material | Missing |
|---|---|---|
| Welding, tacks, standards | R: D_papers §L; V4 @S 3.8 (the verified pool evidence: Tomków, Mondal, Lowth, Noh, Asadi & Goldak, Jia); `weld_generator/notes/ISO 5817_Ed_4_2023.pdf`, ISO 9692-1 | JASS 6 / EN 1011-2 texts before citing the tack constants (Ch. 4 TODO); Noh 2022 and Asadi & Goldak 2011 into thesis.bib |
| Seam extraction from 3D | R: D_papers §B1-B3; R: C4, C5 (the seven methods, one paragraph each: mechanism, coarse stage, reading choices) | one paragraph per method |
| Datasets and ground truth | R: D_papers §C | |
| Pose estimation, tracking | R: D_papers §D1-E and the digests §2.1 (FoundationPose), §2.2 (SAM 2) | MegaPose is not in thesis.bib yet |
| Registration (ICP, PPF, descriptors) | R: D_papers §F1-F3, digests §2.4-2.5 | |
| Calibration | R: D_papers §I | a hand-eye survey; a touch-sensing source (Ch. 3 TODO; candidates in `notes/todo.md`) |
| Planning | R: D_papers §J, digests §2.6-2.7; V4 (Liu 2023: eight cylinders, a cable-twist term; Jia 2025: linear moves between tacks) | Raveh et al. (hybridisation): not in the folders (Geraerts & Overmars is in `supply_papers/` since 2026-10-26) |
| HRI and gestures | R: D_papers §K, digest §2.3 | the MediaPipe Hands paper and the Gesture Recognizer model card (VERIFY entries in thesis.bib) |

## Chapter 3: Methodology, the cell (`manuscript/chapters/ch3_methodology_cell.tex`)

| Section | Material | Missing |
|---|---|---|
| 3.1 Overview, vision only, two computers | R: B2 @S 3.1; `ros2_ws/.../notes/pipeline_map.dot`; README "Big picture" | the pipeline figure (redraw the .dot in English) |
| 3.2 Problem, notation, assumptions | R: B2 @S 3.2; README §1 (eye-in-hand rules); `admittance_control/frame_stamp.py` (the image stamp, mid-exposure since 2026-10-22) | measure the stamps' age at publish on the bench (the camera node logs it) |
| 3.3 Calibration (kinematics, TCP, table, hand-eye, depth, rotation, translation) | R: B1b §C (every tool, procedure and file); R: B1a A1-A3; R: B2 @S 3.3; thesis_notes 2026-10-02 (depth) | results go to Ch. 6 (`tab:exp_depth` etc.) |
| 3.4.1 Gestures | R: B1b §E.2; R: B2 @S 3.4.1; `scripts/gesture_control_node.py` | VERIFY the MediaPipe model description (Ch. 3 TODO) |
| 3.4.2 Camera aim, the relay, the buzz | R: B1a B1-B2; R: B2 @S 3.4.2; thesis_notes 2026-10-15/16, 2026-10-22 (the Jacobian fixed); the two bags | re-record one aiming bag (the Jacobian and the stamps changed on 2026-10-22); the bag figures |
| 3.5 Perception (capture, SAM2, PPF, FoundationPose, tracking, saving, symmetric/cut tubes) | R: B1b §A (the Docker server step by step); R: D_papers §2.1-2.5; R: B1a C2-C4, E3; `notes/perception_aciklama.md` §6-12 (Turkish; the most detailed) | VERIFY 252 hypotheses on the desktop; the architecture figures (adapted from the papers) |
| 3.6 Multi-view (views, capture, joint refinement, observability, self-calibration, acceptance, ISO fit-up) | R: B1a D1-D6 and algorithms 9-11; R: B2 @S 3.6; `notes/multiview_refine_plan.md` | OPEN: the table-anchored height; the self-calibration bench check |
| 3.7 Seams (registry, planar, curved, fit-up, fallback) | R: B1a E1-E3 and algorithms 5-7; R: B2 @S 3.7; `notes/curved_seams_plan.md`, `seam_two_modes_plan.md` | DECIDE mode B (THESIS_PLAN #7) |
| 3.8 Tacks | R: B1a E2; `weld_generator/weldgen/tacks.py`; R: B2 @S 3.8 | OPEN/DECIDE: heat-based ordering (#9), robot-aware placement (#1), tack length |
| 3.9 Motion (collision, reach, descent, transits, marking) | R: B1a F1-F6 and algorithms 12-15; R: B2 @S 3.9; `notes/pen_marking_plan.md`, `aps_transit_plan.md` | OPEN: fixtures in marking.json; box-box clearance |
| 3.10 Implementation (architecture, nodes, rates, state machine, C++, failing loudly) | R: B1b §E (per-node entries); R: B2 @S 3.10; README node reference | the architecture and state-machine figures; DECIDE the digital twin sentence |

## Chapter 4: Methodology, WeldSet (`manuscript/chapters/ch4_methodology_weldset.tex`)

| Section | Material | Missing |
|---|---|---|
| Goals, pipeline, determinism | R: C, C3; `weld_generator/docs/MANUAL.md`, `notes/dataset_plan.md` | pin the versions (P3) |
| Parts, joints, assemblies | R: C7 (plate core), C2 (ranges as coded) | P1 #2 (interpenetration), #4 (fixture twins), #5 (grooved) |
| Curved seams | R: C (curved families), `notes/patch_phase6b.md` | ellipse coverage (P2 #9) |
| Accessibility rule, tack rule, MPS | R: C7 §D, R: C3; `weldgen/accessibility.py`, `tacks.py`, `mps.py` | P1 #3 (coplanar arm) |
| Sensor model, rendering | R: C (sensor), R: C6; `weldgen/noise.py`, `visibility.py`, `render/` | P1 #1 (exterior flag); DESKTOP gate statistics |
| Corpus, protocol, methods | R: C4, C5, C6 | DESKTOP corpus sizes, yields |
| The two extractors | R: C §3 (footprint, seamfind); `weld_generator/Self_ideas/` | attribution (İsmail Hoca); DECIDE real-scan runs (#8) |
| Annotation experiment | R: C6; `weld_generator/scripts/annotation/` | DESKTOP (the laptop copy may be stale) |

## Chapter 5: Setup (`manuscript/chapters/ch5_experimental_setup.tex`)

| Section | Material | Missing |
|---|---|---|
| Robot, camera, tool | R: B1b §B.1 (the holder CAD and envelope); `config/pen_tool.json` | ASK: printer, filament and infill of the holder; the pen; the payload setting; the desktop's RAM; photo |
| Software | `tab:software` (laptop versions read 2026-10-20) | DESKTOP: the Docker image's versions and Dockerfile |
| Test parts | R: B1b §B.2 (dimensions from the CADs and the registry); `notes/curved_parts_print_list.md` | ASK: filament (PLA?), measured dimensions, real MDF thickness, which base plate per experiment |
| Measuring the error, procedure | R: B1b §C.8 (probes); README Gotchas | DECIDE: how the true root is measured; the repeated trials (positioning.md) |
| WeldSet setup | `weld_generator/docs/MANUAL.md` §7-8 | DESKTOP |

## Chapter 6: Results (`manuscript/chapters/ch6_results.tex`)

| Section | Material | Missing |
|---|---|---|
| Calibration | R: B2 @R 6.1; README §14, §16; thesis_notes 2026-10-02 | the hand-eye residuals, the TCP check, the tilt split as a small table |
| Registration, tracking | R: B2 @R 6.2-6.5; `notes/readme_perception.md` (track_bench) | OPEN: FoundationPose jitter on a still part; FP vs ICP benchmark |
| Multi-view | R: B2 @R 6.3; captures in `scripts/foundationpose_results/multiview/` | |
| Placement accuracy | README §14 (error chain); `tack_marks.json` archives in `marks_history/` | the missing error-chain rows (Ch. 6 TODO); repeated trials |
| Planning, timing, curved parts, aim | R: B2 @R 6.6-6.9; `notes/aps_transit_plan.md` §Measured; the bags | the planned comparisons |
| WeldSet benchmark | DESKTOP: `out/bench6b`, the final batch (271 200 rows), notebooks 15-17 | after the P1 decision (#10) |

## Chapter 7 and the appendices

| Part | Material | Missing |
|---|---|---|
| Limitations, future work | the OPEN items of Ch. 3 and 4; `notes/todo.md` (OPEN HRI, motion, perception) | after Ch. 6 |
| App. A parameters | written (values in use) | re-check before submission |
| App. B standards | ISO 5817 no. 617 (Eq. iso_gap), ISO 9692-1 rows | the tack-rule sources |
| App. C derivations | the TCP check, the Schur complement, the depth law, the overshoot bound, the phase margin, the holder clearance | write out |
| App. D software and data | both repositories; how to reproduce a run | DESKTOP for WeldSet |

## Not yet reviewed

The five notes that B2 did not cover are in `verify/V4_docs_rest.md`: `notes/readme_perception.md`
(the English twin of perception_aciklama, with the timing instrumentation and the CAD-unit checks),
`notes/admitans kontrol açıklama.md` (the robot line from the pose to the pen mark, for the advisor,
2026-10-04/08; not the legacy admittance controller), the two AI-generated literature reports (their
citations checked against the PDFs: do not cite them), and the 2026-10-01 advisor meeting, with a
34-item table of his requests and their status. R: B1b's consolidated §D is unfinished; its per-file
entries (§E) are complete.

## Verification of the drafts

`notes/review_2026-10-20/verify/` holds the fact-checks of the rewritten chapters (V1: Ch. 3 first
half and Ch. 5; V2: Ch. 3 second half and the appendices; V3: Ch. 4). Their findings were applied on
2026-10-20; the reports keep the evidence (file:line) for each value.
