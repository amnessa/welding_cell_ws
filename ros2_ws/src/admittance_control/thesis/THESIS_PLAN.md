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
    chapters/ch1..ch8_*.tex one file per chapter
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

**Compiling.** There is no LaTeX here, so use one of:
- **Overleaf:** upload `manuscript/`; `thesis.tex` is the main file; compiler pdfLaTeX;
  BibTeX.
- **Locally:** `apt install texlive-latex-extra texlive-science texlive-fonts-recommended`,
  then:
  ```
  pdflatex thesis && bibtex thesis && pdflatex thesis && pdflatex thesis
  ```

The skeleton has not been compiled yet. Expect small fixes on the first run.

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

The examples share a pattern: Introduction (Motivation and Problem Definition, Proposed
Methods and Models, Contributions and Novelties, Outline) → literature → method chapters
→ experiments → conclusion. The closest example in content is Gülbeden's (a robot that
draws with a pen; it includes depth calibration). This thesis follows the same shape:

| ch | title | what it holds | main sources in the repo |
|---|---|---|---|
| 1 | Introduction | motivation (human tacks, robot welds; annotated, private data), the problem, the two parts, contributions, outline | `weld_generator/notes/thesis_direction_handoff.md` §2–4 |
| 2 | Background and Related Work | welding / tacks / ISO; seam detection; datasets and ground truth; pose estimation; registration; calibration; planning; the gap | handoff §5 (literature map), `weld_generator/papers/`, `notes/Welding_Robots_Approach_and_Trajectory_Literature_Report.md` |
| 3 | WeldSet | design goals, primitives, curve-first families, the D4 seam rule, the tack rule, sensor model and rendering, strata, the benchmark of literature methods, (annotation-error experiment) | `weld_generator/docs/MANUAL.md`, `SCHEMA.md`, `PARAMETERS.md`, `notes/phase8_plan.md`, `phase9_plan.md`, the ICRA submission |
| 4 | System Overview and Calibration | hardware, software (two GPUs), frames, the calibration chain (kinematics, TCP, table, hand-eye, depth) | README "How the pieces connect", Algorithms §1, §14, §16; `thesis_notes.md` |
| 5 | Perception and Registration | classification (PPF), registration (FoundationPose), live tracking, stationary refinement (ICP, normal gate, robust mean, save-time ICP), multi-view refinement (views, joint refinement, observability, the camera offset, acceptance, fit-up) | `notes/perception_aciklama.md`, `readme_perception.md`, `realtime_fp.md`, `multiview_refine_plan.md`, README §15 |
| 6 | Seams, Tacks and Motion | mode A seams, tack placement (+ the open decision below), reachability, collision model, APS transits, force-gated marking | `seam_two_modes_plan.md`, `pen_marking_plan.md`, `aps_transit_plan.md`, README Algorithms §8 |
| 7 | Experiments and Results | setup and protocol, calibration, registration, multi-view, placement accuracy (the error chain), tracking FP vs ICP, APS vs RRT-Connect, timing, curved parts, discussion | README §14 tables, `thesis_notes.md`, captures in `scripts/foundationpose_results/`, `todo.md` "Benchmarks" |
| 8 | Conclusion | summary, limitations, future work | |
| App. A–D | parameters; standards; derivations; software and data | | node parameters, ISO PDFs, module docstrings, README "Run it" |

The Turkish explanations (`notes/perception_aciklama.md`,
`notes/admitans kontrol açıklama.md`, `notes/pipeline map.md`) follow the same order as
Chapters 4–6 and are a good first draft to translate from.

## Inventory

Status: **have** = the content or data exists; **produce** = needs a run, a plot or a
photo; **planned** = needs an experiment that is not done yet.

### Figures

| label | chapter | content | status / how |
|---|---|---|---|
| `fig:intro_cell` | 1 | photo of the cell with a mark | produce: photo |
| `fig:intro_pipeline` | 1 | the pipeline overview | have: `notes/pipeline_map.dot`; redraw in English |
| `fig:ws_families` | 3 | one render per curved family | produce: weld_generator renders |
| `fig:tool` | 4 | pen tool + collision envelope | produce: RViz `/tool_model/markers` screenshot |
| `fig:architecture` | 4 | nodes, topics, two GPUs | produce: diagram (Graphviz, like the pipeline map) |
| `fig:frames` | 4 | TF tree / frame chain | produce: diagram |
| `fig:depth_law` | 4 | depth error vs range, before / after the Tare | have the numbers (README §16); produce the plot |
| `fig:tracking_split` | 5 | desktop registers, laptop tracks | produce: diagram |
| `fig:views` | 5 | the 4 planned views + coloured clouds | produce: RViz screenshot of a `refine_pose` run |
| `fig:marking_sequence` | 6 | joints and force during one tack | produce: plot from a recorded run (bag or `tack_marks.json`) |
| `fig:exp_marks` | 7 | marks on the seam with offsets | produce: photo + measurement |
| `fig:exp_transits` | 7 | tip paths RRT-Connect vs APS | planned: benchmark (APS step 5) |
| (more) | 3, 7 | benchmark plots per stratum; the error budget as a bar chart | weld_generator notebooks; README §14 |

### Tables

| label | chapter | content | status |
|---|---|---|---|
| `tab:lit_seam` | 2 | literature: sensor, data, ground truth, availability | have most (handoff §5); fill the sensors |
| `tab:ws_strata` | 3 | strata, ranges, corpus sizes | have ranges; corpus sizes from the desktop |
| `tab:ws_benchmark` | 3 | seam error per method and stratum | have (Phase 4 batch, desktop) |
| `tab:hardware` | 4 | hardware | have |
| `tab:calib_chain` | 4 | link, method, reference, residual | have |
| `tab:exp_depth` | 7 | depth before / after the Tare | have (filled) |
| `tab:exp_table_views` | 7 | the table seen by each view | have (filled) |
| `tab:exp_error_chain` | 7 | causes, fixes, mark error | have (filled from README §14) |
| (to add) | 7 | repeated placement trials: mean / std / max over N tacks | planned: repeat the bench run N times |
| (to add) | 7 | FP vs ICP tracking; APS vs RRT-Connect; timing per stage | planned (`todo.md` "Benchmarks", "measure everything") |
| (to add) | App. A | all parameters | have (node parameters) |

### Equations (already in the skeleton)

| label | content |
|---|---|
| `eq:plane_intersection` | the seam as two planes' intersection |
| `eq:tackrule_len`, `eq:tackrule_n` | tackrule-0.1 |
| `eq:frame_chain` | base ← tool ← camera ← object |
| `eq:handeye` | AX = XB |
| `eq:depth_error` | Δz = z²δ / (f B) |
| `eq:ppf` | the point-pair feature |
| `eq:p2plane`, `eq:normal_gate` | point-to-plane ICP with Welsch weights; the normal gate |
| `eq:mv_objective` | the multi-view objective (data, prior, overlap) |
| `eq:observability` | σ = σ_noise / √λ ≤ 0.5 mm |
| `eq:extrinsic_d` | the residual with R_v d; the Schur complement |
| `eq:iso_gap` | ISO 5817 no. 617 root-gap limit (**verify against the standard**) |
| `eq:transit_cost` | weighted joint travel |

### Notation (keep it consistent)

- `\T{B}{C}`: the pose of frame C in frame B (4×4).
- Frames: B = `base_link`, E = `tool0`, C = camera optical frame, O = object.
- Vectors bold (`\vect{}`); units SI in equations, mm in tables of errors.

## Writing order (suggested)

1. **Chapters 4–6 first.** The methods exist and are documented (README, plans, the
   Turkish explanations); this is mostly translation and condensation.
2. **Chapter 3:** from weld_generator's MANUAL and the ICRA paper.
3. **Chapter 7:** fill what exists (the error chain, depth, multi-view), then run the
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
