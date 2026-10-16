# Thesis plan: MSc in Robotics, METU (Graduate School of Natural and Applied Sciences)

*Started 2026-10-08. The manuscript is `manuscript/` (official METU LaTeX template v5 +
a Robotics department option). This file maps every part of the thesis to the material
that already exists, and lists what is still to be produced.*

## Folder layout

```
thesis/
  THESIS_PLAN.md            this file
  manuscript/               the thesis (LaTeX)
    thesis.tex              front matter (title, jury, abstract, keywords), chapter order
    chapters/ch1..ch7_*.tex one file per chapter
    appendices/*.tex
    abbreviations.tex       list of abbreviations
    thesis.bib              references (IEEE style); "VERIFY" marks unchecked fields
    figures/                one file per figure, named by its label (figures/README.md)
    metu.cls, metu1x.def    the METU class (+ "rob" = Robotics added)
    Tez_Sablonu_Onay_Formu.docx   sign it and put it in front of every draft
  template/                 the untouched official template, manual, sample pages
  examples/                 six lab theses (FBE, MS; two in Robotics)
  support_documents/        GSSS (Social Sciences) documents: NOT our institute
```

**Compiling** (TeX Live installed in the container 2026-10-16):
```
manuscript/build.sh            # pdflatex, bibtex, pdflatex x2 -> manuscript/build/thesis.pdf
```
The packages it needs are listed at the top of `build.sh` (`metu.cls` loads the Greek LGR
encoding and Turkish babel). Overleaf works too: upload `manuscript/`, main file
`thesis.tex`, pdfLaTeX + BibTeX. The class is two-sided: every chapter opens on an odd page,
so blank pages before chapters are intended (the examples do the same).

## Rules that apply (FBE, MSc)

| item | rule | source |
|---|---|---|
| template | the official Word or LaTeX template; the signed **Template Confirmation Form** at the front of every draft | fbe.metu.edu.tr thesis writing process; template readme |
| abstract / öz | at most **250 words** each; both languages | template |
| keywords | at most **5**, English and Turkish | template |
| acknowledgments | at most **300 words**; dedication at most 2 lines | template |
| CV | **not** for MSc | FBE writing process page |
| jury | 3 (or 5) members: the chair first, the supervisor second, affiliations on one line | template comments |
| formatting rules | `template/thesis_manual_v2.pdf`, `template/thesis_checklist_v01.doc`, `template/sample_pages_v1.docx` | FBE |
| similarity | the supervisor prepares and signs the originality report (first page of the similarity index). The 20 % ceiling is stated on the **GSSS** page: ask the advisor or the department for the FBE limit | FBE submission page |
| before the defence | jury assignment form (department); originality report; draft PDF to **fbetezformat@metu.edu.tr** (subject: department abbreviation + full name); publication list via fbeforms.metu.edu.tr; all **≥ 1 month before** the defence | FBE submission page |
| after the defence (within 1 month) | jury corrections → PDF to fbetezformat → format fixes → title/summary on OIBS → signed originality report → bind ≥ 2 copies (blue-ink signatures) → OpenMETU upload → YÖK thesis form → CD with `<YÖK no>.pdf` → hand in; collect after ≥ 3 working days | FBE submission page |

## Structure

**Restructured 2026-10-16** to the shape of the three examples the user named (Oydu,
Kurt, Ağuş: Introduction → Literature → METHODOLOGY → Setup / Implementation → Results
and Discussion → Conclusion). The user's order: the robotic cell first, then WeldSet. The
methods of the 2026-10-08 skeleton's chapters 3–6 are merged into two methodology
chapters; the old files are removed (git keeps them).

| ch | title | what it holds | main sources in the repo |
|---|---|---|---|
| 1 | Introduction | motivation (human tacks, robot welds; annotated, private data), the problem, the two parts, contributions, outline | `weld_generator/notes/thesis_direction_handoff.md` §2–4 |
| 2 | Background and Related Work | welding / tacks / ISO; seam detection; datasets and ground truth; pose estimation; registration; calibration; planning; the gap | handoff §5 (literature map), `weld_generator/papers/`, `notes/Welding_Robots_Approach_and_Trajectory_Literature_Report.md` |
| 3 | **Methodology: Perception-Guided Robotic Tack Placement** (`ch3_methodology_cell.tex`) | overview, problem formulation; frames and calibration; operator interaction (gestures, camera aim, 500 Hz relay); perception (SAM2 + PPF, FoundationPose, tracking, stationary ICP, swing–twist, spin settling); multi-view refinement; seams from registered primitives (planar, curved, fit-up); tacks and ordering; motion (collision model, reachability with the roll DP and the cable window, transits, force-gated marking); implementation details | README, the plans in `notes/`, module docstrings (each section's `\SRC`) |
| 4 | **Methodology: WeldSet** (`ch4_methodology_weldset.tex`) | design goals, parts, curved seams by construction, the D4 seam rule, the tack rule, MPS, sensor model and rendering, strata, the evaluation protocol (7 literature methods, oracle ladder, metrics); **the two proposed extractors** (footprint from sequential scans; matching + sphere tracing on labelled clouds); **WeldSet ↔ cell: one geometry** | `weld_generator/docs/MANUAL.md`, `Self_ideas/` (both methods), `scripts/baselines/self_*.py`, notebooks 16–17 |
| 5 | Experimental Setup and Procedure (`ch5_experimental_setup.tex`) | robot and tool, software, the test assemblies, how the mark error is measured, the procedure, the WeldSet batch | README "Run it", `curved_parts_print_list.md` |
| 6 | Results and Discussion (`ch6_results.tex`) | calibration, registration, multi-view, placement accuracy, FP vs ICP, APS vs RRT-Connect, timing, curved parts, the camera aim, the WeldSet benchmark, discussion | README §14 tables, `thesis_notes.md`, captures, the aim bags |
| 7 | Conclusion (`ch7_conclusion.tex`) | summary, limitations, future work | |
| App. A–D | parameters; standards; derivations; software and data | | node parameters, ISO PDFs, module docstrings, README "Run it" |

**How the two parts connect** (Ch. 4 §"One Geometry"): the cell computes its seams and
tacks with WeldSet's primitives, seam rule, curve constructors and tack rule, so it marks
WeldSet's exact truth at the measured poses; and the two proposed CAD-free extractors take
as input what the cell's procedure produces (the base scanned before the next part is
placed; clouds labelled per part by the registration), scored on WeldSet before the
robot. Open: do they run on real scans in this thesis (decision 8 below)?

## Inventory

Status: **have** = the content or data exists; **produce** = needs a run, a plot or a
photo; **planned** = needs an experiment that is not done yet.

Generated from the chapters on 2026-10-16 (placeholders in place; to refine with the
user: which formulas, figures, graphs and tables each section needs).

### Figures (placeholders in the chapters)

| chapter | labels |
|---|---|
| 1 | `fig:intro_pipeline` (+ `fig:intro_cell` photo, planned) |
| 3 | `fig:mc_pipeline`, `fig:frames`, `fig:depth_law`, `fig:mc_aim_loop`, `fig:tracking_split`, `fig:views`, `fig:mc_seam_kinds`, `fig:mc_collision`, `fig:mc_roll_dp`, `fig:marking_sequence`, `fig:architecture` |
| 4 | `fig:ws_families`, `fig:ws_own` (the two extractors on one T-joint), `fig:ws_link` |
| 5 | `fig:tool`, `fig:setup_parts`, `fig:exp_marks` |
| 6 | `fig:exp_transits`; to add: the aim's effort spectrum before/after and the 80 ms command zoom (the two bags), benchmark plots per stratum, the error budget |

### Tables

| chapter | labels |
|---|---|
| 3 | `tab:calib_chain`, `tab:gestures`, `tab:mv_acceptance`; to add: a notation table |
| 4 | `tab:ws_strata`; to add: the primitives |
| 5 | `tab:hardware`, `tab:setup_parts` |
| 6 | `tab:exp_depth`, `tab:exp_table_views`, `tab:exp_error_chain`, `tab:ws_benchmark`; planned: repeated placement trials, FP vs ICP, APS vs RRT-Connect, timing per stage |

### Equations (drafted; each to be checked when its section is refined)

| chapter | labels |
|---|---|
| 3 | `eq:frame_chain`, `eq:handeye`, `eq:depth_error`, `eq:aim_error`, `eq:aim_law`, `eq:aim_dls`, `eq:relay`, `eq:ppf`, `eq:p2plane`, `eq:normal_gate`, `eq:swing_twist`, `eq:spin_settle`, `eq:mv_objective`, `eq:observability`, `eq:extrinsic_d`, `eq:plane_intersection`, `eq:ellipse_seam`, `eq:iso_gap` (**verify against the standard**), `eq:roll_dp`, `eq:cartesian_bow`, `eq:transit_cost`, `eq:touch` |
| 4 | `eq:tackrule_len`, `eq:tackrule_n`, `eq:footprint`, `eq:sphere_trace`; to add: the rolling ball, the saddle in general position, the step-ratio angle |

### Notation (keep it consistent)

- `\T{B}{C}`: the pose of frame C in frame B (4×4).
- Frames: B = `base_link`, E = `tool0`, C = camera optical frame, O = object.
- Vectors bold (`\vect{}`); units SI in equations, mm in tables of errors.

## Writing order (suggested)

1. **Chapter 3 first** (the cell's methodology; first shape 2026-10-16). Then decide per
   section the formulas, figures and tables, and write the prose out.
2. **Chapter 4:** WeldSet, from weld_generator's MANUAL, the ICRA paper and the two
   extractors' plans.
3. **Chapter 6:** fill what exists (the error chain, depth, multi-view), then run the
   planned experiments and add them: repeated trials, FP vs ICP, APS vs RRT-Connect,
   timing.
4. **Chapter 2:** with the method chapters written, the related work can be aimed.
5. **Chapters 1 and 8, then Abstract / Öz, keywords, acknowledgments** last.

## Open decisions (with the advisor)

1. **The thesis core.** `thesis_direction_handoff.md` §4 calls robot-aware tack placement
   (quality field over arc length × roll, exact DP for k tacks, ordering) "the thesis
   core". It is **not built**: the cell uses the tack rule. Either build it (a section in
   Ch. 6 + results in Ch. 7) or present the vision-only cell + WeldSet as the
   contribution and move it to future work.
2. **How much of WeldSet** goes into the thesis: the full benchmark, or a chapter
   summarising the paper.
3. **The written no-welding scope** confirmation (open since the handoff).
4. **Title.** Candidates:
   - *Vision-Only Robotic Tack Placement on Registered Weld Assemblies* (in the skeleton)
   - *Analytic Ground Truth and a Vision-Only Robotic Cell for Weld Seam and Tack Localization*
   - *From CAD to Tack: Registration-Based Weld Seam Computation and Robotic Validation*
5. **Jury** (3 members, at least one from outside METU?) and the defence date: these
   decide the deadlines (draft to the institute ≥ 1 month before).
6. The advisor's **title and surname**, the head of the programme, the FBE director: all
   placeholders in `thesis.tex`.
7. **Mode B** (seams without CAD): probably dropped from the thesis (the user, 2026-10-15:
   too much effort for little gain in a robotics thesis). Not yet decided.
8. **The two proposed extractors on the robot** (footprint, seamfind): real-scan results in
   Ch. 6, or WeldSet scores only and the robot run as future work? Their attribution (the
   footprint idea is İsmail Hoca's).
9. **Tack ordering by a rough heat simulation** (todo.md, 2026-10-16): a section of Ch. 3
   with results, or future work?

## Comparisons to dig out for Chapter 6 (later; the user, 2026-10-15)

Most numbers are already in `notes/thesis_notes.md`; each needs a repeatable measurement
(`todo.md`, "Benchmarks to run"):
- FoundationPose vs ICP tracking (noise, delay, drift);
- APS vs RRT-Connect (planning time, joint travel, tip path length);
- the tool's Cartesian transit vs APS round a pipe (tip path 0.14–0.21 m vs 0.26–1.6 m);
- single vs multi-view per stratum (marks; the pen's contacts);
- bounding-box vs true-shape neighbours in the refinement (synthetic);
- the C++ vs Python collision model (590× per check; one plan 478 s → 18 s);
- a cut pipe's spin settled on its base (E2: gap range 26.2 → 0.4 mm).
