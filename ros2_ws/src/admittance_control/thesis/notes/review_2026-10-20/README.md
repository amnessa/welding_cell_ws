# Review of 2026-10-20: raw reports

**Machine-generated. Verify before use.** These are the working reports of the 2026-10-20 review,
written by review agents that read the code, the notes, the example theses and the paper pool. They
were cross-checked where the thesis draft depends on them (see below). They can still contain a
wrong line number, a misread value or a value that has changed since. Before a number goes into the
thesis, check it in the file the report cites.

The thesis chapters are the curated output: `manuscript/chapters/ch3_methodology_cell.tex`,
`ch4_methodology_weldset.tex` and `ch5_experimental_setup.tex`. The section-by-section index is
`../../MATERIAL_MAP.md`.

| Report | Scope | State |
|---|---|---|
| `A1_kurt_bilaloglu.md` | Umut Kurt's and Cem Bilaloğlu's theses: structure, reasoning, phrases | complete |
| `A2_gulbeden_oydu.md` | Anıl Gülbeden's and Onur Oydu's theses | complete |
| `A3_agus_uzel_akbaba.md` | Ege Uğur Aguş, Denge Uzel, Mustafa Akbaba | complete |
| `B1a_library.md` | the cell's library modules and C++ code: algorithms, values in use, 36 discrepancies, 15 algorithm boxes, dated incidents | complete |
| `B1b_scripts_assets.md` | nodes, scripts, the pose server in Docker, the printed tool and parts, the calibration tools | §D (the consolidated missing/discrepancy list) unfinished; §0, A, B, C, E complete |
| `B2_docs_RAW.md` | every Markdown note of the package, tagged by thesis section | RAW working file: `@S` material, `@C` confirmed, `@X` draft-vs-doc discrepancy, `@M` missing, `@O` open, `@R` result for Chapter 6. Four notes (readme_perception, admitans kontrol açıklama, the two literature reports, the 1 October meeting transcript) are not yet covered |
| `C_weldset.md`, `C2`–`C7` | weld_generator: the generator, curved families, sensor model, schema and tests, the seven literature methods, the harness and the batch | complete; findings condensed in `../weldset_code_review_2026-10-20.md` |
| `D_papers.md` | the paper pool: index, digests of FoundationPose, SAM 2, MediaPipe, PPF (Drost; Birdal), RRT-Connect, APS; the bibliography check; missing literature | complete |

## The verification pass (`verify/`)

After the chapters were rewritten, four more agents checked the new drafts against the code and the
notes, and all their findings were applied to the chapters on 2026-10-20:

| Report | Checked | Findings |
|---|---|---|
| `verify/V1_ch3a_ch5.md` | Ch. 3 §3.1-3.5 and Ch. 5 | 3 wrong, 10 imprecise, 43 nuances |
| `verify/V2_ch3b_app.md` | Ch. 3 §3.6 to the end, Appendices A and C | 5 wrong, 11 imprecise, 59 nuances |
| `verify/V3_ch4.md` | Ch. 4 against weld_generator | 3 wrong, 9 imprecise, 33 nuances; `walker_ratio.py`, `saddle_check.py`, `nook_check.py` are its numerical checks |
| `verify/V4_docs_rest.md` | the five notes B2 did not cover (readme_perception, the Turkish robot-line note, the two AI literature reports, the 2026-10-01 meeting transcript) | material in B2's tag format, the citations of the AI reports checked against the PDFs, and a 34-item table of the advisor's requests |

Two corrections to the earlier reports: D_papers line 74 says the AI literature report gets Tomków's
co-authors wrong (it does not; its wrong Tomków claims are the "16.06 %" result and the description
of the study), and B1a's ISO 5817 row for t ≤ 3 mm misses the `0.1 a` term.

## What was cross-checked

- **Chapter 3** values were taken from B1a/B1b/B2 and the most important ones were re-read in the
  code on 2026-10-20: the aim's `AimConfig`, the gesture node's parameters, the multi-view
  `settle_s`/`n_frames`, `lin_steps`, the ISO 5817 `root_gap_limit`, the tack rule.
- **The Jacobian defect** (B1a A1, discrepancy #36) was reproduced numerically: the angular rows of
  `kinematics.ur5e_jacobian` are exactly 2× the true ones.
- **B1a's ISO 5817 row for t ≤ 3 mm** misses the `0.1 a` term; the code is
  `0.1·a + (0.2, 0.3, 0.5)` at B, C, D (`weldgen/config.py:root_gap_limit`). Chapter 3 uses the code.
- **Chapter 4** values were taken from C2–C7 and re-read in `weldgen/tacks.py`, `camera.py`,
  `scene.py` and `scripts/baselines/metrics.py`.

## Do not copy into the thesis

- Quotes from the example theses (A1–A3): they are for style, not for reuse.
- The literature report in `paper_pool/Welding_Robots_Approach_and_Trajectory_Literature_Report.md`
  and the deep-search PDFs: D_papers found wrong citations in them. Cite only from a PDF you have.
