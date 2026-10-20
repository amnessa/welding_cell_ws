# Where this thesis sits among the lab's example theses (2026-10-20)

Based on full readings of the seven theses in `examples/` (`review_2026-10-20/A1..A3_*.md`).
Depth scale 1-5 (T = theoretical, A = algorithmic, M = methodological, E = experimental):
1 = textbook only / demos; 3 = solid MSc norm (non-trivial adaptation, systematic experiments in
one domain); 5 = publication-level novelty and rigour.

## The examples

| Thesis | Programme, year | Numbered pages | Eq. | Pseudocode | Real robot | Contribution type | T / A / M / E |
|---|---|---|---|---|---|---|---|
| Umut Kurt | Robotics, 2025 | 84 | 32 | 0 | no (handheld camera; navigation in simulation) | algorithm + system (topological SLAM, object VPR) | 2.5 / 3 / 4 / 3 |
| Onur Oydu | Robotics, 2025 | 119 | 49 | 0 (flowcharts, state machine) | yes, TIAGo, 72 OptiTrack trials | system + algorithm adaptation | 2 / 3 / 3 / 3.5 |
| Anıl Gülbeden | EEE, 2025 | 85 | 10 | 1 | yes, TIAGo; 9/15 end-to-end runs | integration + comparison (pen drawing) | 1 / 2 / 3 / 3 |
| Cem Bilaloğlu | ME, 2022 | 86 | 58 tags | 3 | yes, swarm, Vicon | platform design | 1.5 / 2 / 3 / 3 |
| Ege Uğur Aguş | ME, 2023 | 89 | 11 | 4 | no (2D simulation) | new planner | 3 / 4 / 2 / 2 |
| Denge Uzel | ME, 2023 | 57 | 6 | 0 | no | dataset + DL model in an existing RL framework | 1 / 2 / 3 / 3 |
| Mustafa Akbaba | EEE, 2025 | 110 | 127 | 0 (block diagrams) | yes, GO1; single runs | control design with derivations | 4 / 4 / 4 / 4 |

None proves stability, convergence or optimality formally. Only Akbaba compares against a
literature baseline; only Oydu reports real-robot statistics against an external ground truth.

## This thesis (projected from the current draft and the material)

| | Value |
|---|---|
| Programme | Robotics |
| Parts | (1) a perception-guided tack-placement cell on a real UR5e; (2) WeldSet, a dataset and benchmark with analytic ground truth |
| Pages | about 120-150 numbered pages once written out |
| Equations | about 40-60 (26 drafted on 2026-10-16) |
| Algorithm boxes | about 10-14 planned |
| Real robot | yes: calibration chain, registration, multi-view refinement, marking on 6 test assemblies, the camera aim |
| Ground truth | the pen's contact depth (independent of the camera); mark offsets measured on the part; WeldSet's analytic truth |
| Baselines | seven re-implemented literature methods on 720 scenes × 12 strata (271 200 result rows), each with an oracle ladder; the annotation floor (1.415 mm median); two own extractors scored on the same harness |

**Estimated depth** (assuming the planned experiments of §"To reach the upper end" are run):

| Dimension | Now | Reachable | Why |
|---|---|---|---|
| Theoretical | 2.5 | 3 | Correct and adapted geometry (frame chain, stereo error model, point-to-plane ICP, chordal mean, swing-twist, an observability analysis with a Schur-complement extrinsic estimate, analytic intersection curves, a Viterbi recursion, image-based servoing) plus WeldSet's analytic construction. Small propositions with short proofs would lift it to 3: exact latency compensation under rotation about the optical centre; the step-ratio angle of the sphere-tracing walker; why only the swing of a tube's correction is observable; the observability bound |
| Algorithmic | 3.5 | 4 | Many components of its own, not one: multi-view joint refinement with true-shape overlap and an online extrinsic estimate; curved seams from registered primitives; settling a cut pipe's spin; a per-tack roll DP with a cable window; the transit ladder; the force-gated descent; the camera aim with a 500 Hz relay; WeldSet's curve-first generator, accessibility rule, tack rule and gates; two CAD-free extractors. The risk is breadth: each component must be formalised and justified, not only described |
| Methodological | 4 | 4.5 | A calibration chain referenced to the robot itself; the pen contact as an independent measurement; analytic ground truth; an oracle ladder for fair baselines; a fidelity audit and an endorsement by a re-implemented paper's first author. Weak spots now: the WeldSet code defects found on 2026-10-20 (`weldset_code_review_2026-10-20.md`) and seamfind's defaults chosen on 12 scenes of the same corpus |
| Experimental | 3 | 4 | Now: many bench runs, mostly single, with measured mark errors; a 271 200-row benchmark; an annotation study; two rendered corpora. Missing: repeated real-robot trials with statistics, the planned comparisons (FP vs ICP tracking, APS vs RRT-Connect, single vs multi-view, timing per stage) |

## Verdict

- **Is the depth enough for an MSc?** Yes, clearly. In scope it is above every example: they
  have one contribution each and 57-119 pages; this thesis has two connected ones, a real robot
  and an analytic ground truth that none of them has. On algorithmic and methodological depth it
  sits with Kurt and Akbaba at the top of the set; on theory it matches Kurt and Oydu (below
  Akbaba's derivation-heavy control thesis).
- **The main risk is breadth, not depth.** A committee will ask "what is the core contribution?"
  Frame the thesis around one statement, for example: *seams and tacks are computed from exact
  geometry, never detected: the cell computes them from registered CAD primitives, and WeldSet
  makes the same geometry the ground truth for methods that detect them.* Pick 3-4 headline
  contributions for §1.3 and present the rest as the engineering that makes them work.
- **The "it is only integration" objection** (SAM2 + FoundationPose + OMPL): say exactly where
  the pretrained output ends and the thesis's own logic begins (refinement, symmetry and spin,
  seams, tacks, reachability, calibration, the benchmark), and back each of those with an
  ablation.

## To reach the upper end

1. **Repeated real-robot trials**: N tacks per assembly (say ≥ 20 per configuration), single-view
   vs multi-view on the same placement, with mean / std / median / max (Oydu's format).
2. **The planned comparisons** (THESIS_PLAN "Comparisons to dig out"): FP vs ICP tracking; APS
   vs RRT-Connect (time, joint travel, tip path); the Cartesian transit vs APS round a pipe;
   bounding-box vs true-shape neighbours; the C++ vs Python collision model; timing per stage.
3. **Fix the WeldSet P1 defects** before freezing the benchmark numbers, and say what changed.
4. **Formalise the headline algorithms**: algorithm boxes, stated assumptions, one property each.
5. **Claims hygiene**: claim only what Chapter 6 shows; keep the abstract quantified (Oydu).
