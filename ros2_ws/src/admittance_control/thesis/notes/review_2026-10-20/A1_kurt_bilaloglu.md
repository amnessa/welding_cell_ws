# A1 — Model-thesis review: Umut Kurt (2025) and Cem Bilaloğlu (2022)

**Purpose.** Extract structure, explanation patterns and language from the two METU MSc theses the author considers best written, so that the welding-cell + WeldSet thesis can follow them (Kurt first).

**Sources**

- U. Kurt, *Design and Implementation of an Object-Level Indoor Navigation Algorithm Using Spherical Images*, M.S. thesis, METU Robotics, September 2025. PDF: `/workspaces/welding_cell_ws/ros2_ws/src/admittance_control/thesis/examples/Umut_Kurt_MS_Thesis.pdf`
- C. Bilaloğlu, *Development of an Extensible Heterogeneous Swarm Robot Platform*, M.S. thesis, METU Mechanical Engineering, January 2022. PDF: `/workspaces/welding_cell_ws/ros2_ws/src/admittance_control/thesis/examples/Cem_Bilaloglu_Thesis.pdf`
- Extracted text (pdftotext -layout): `/tmp/claude-0/-workspaces-welding-cell-ws/ab70b953-a34b-4bce-b04c-436c69dd0e01/scratchpad/thesis_review/kurt.txt` and `/tmp/claude-0/-workspaces-welding-cell-ws/ab70b953-a34b-4bce-b04c-436c69dd0e01/scratchpad/thesis_review/bilaloglu.txt`

**Conventions**

- "p. N" means the **printed** page number. Kurt: PDF page = N + 20. Bilaloğlu: PDF page = N + 18. Roman numerals are front matter.
- Quotes are verbatim from the PDF text (line-break hyphens removed), at most two sentences each.
- Figure and table counts come from the lists and the captions in the text. Equation counts come from equation tags. Style statistics (sentence length, pronoun counts) were computed automatically over Chapters 1–6 and are indicative (roughly ±10%).
- Key figure pages were rendered and inspected visually (Kurt Figs. 3.1–3.3, 4.1–4.4, 5.2; Bilaloğlu Figs. 3.1–3.2, 4.2, 4.6, Algorithm 1).

---

## 0. Executive summary

1. **Kurt is the template.** Follow him for the introduction structure (1.1 Motivation and Problem Definition, 1.2 Proposed Methods and Models, 1.3 Contribution and Novelties, 1.4 Outline), for the Methodology / Implementation / Results split, and for voice: "we" for his own work, impersonal for the field, "this thesis" for scope. **Bilaloğlu is the template for engineering justification.** He goes from a specification list to a closed-form calculation, adds a caveat, validates by measurement and reconciles in the Discussion.
2. **Kurt explains pretrained models in tiers.**
   - *Task level* in the Methodology: what the module must deliver, which family of methods fits, and why.
   - *Model level* in the Implementation: the model's name, a one-sentence description, the reason it was chosen, how it behaved, and where it sits in the pipeline.
   - *Evidence level* in the Results: a benchmark against alternatives, plus failure images.
   - *Dependency level* in the Conclusion: limits and future work.

   He never describes a network's architecture, training data or losses. OneFormer gets about five sentences in the whole thesis.
3. **Classical algorithms that the author manipulates are derived in full.** These are the spherical projection, the 8-point algorithm with SVD and rank-2 projection, the four pose hypotheses and the pose-graph cost: 32 numbered equations, all in Chapter 3. Black boxes stay closed.
4. **Kurt's own algorithms are told in prose,** with equations, enumerated alternatives and one summary figure. There is no pseudocode at all. The thresholds and threading that make the system run go into the Implementation chapter, and their values are tuned and justified in the Results.
5. **Kurt's reasoning follows a fixed chain:** context → requirement "in our application" → options and their weaknesses → choice → consequence or property → a later experiment that justifies the choice → failure case and remedy.
6. **His sentences are of medium length** (mean about 23 words, median 20), with one idea each. He uses many concessive and causal connectives: "Although", "Since", "While", "In contrast", "However".
7. **Limitations are admitted where the evidence appears.** Each comes with a root cause and a concrete remedy ("A more robust approach would be ...").
8. **Bilaloğlu states a scope policy for third-party software** (p. 31): describe how the tool was used, not how it works. He gives each external module an input/output sentence. He opens a black box only once, for the ToF sensor's ranging sequence, because his novelty exploits that internal behaviour.
9. **Bilaloğlu uses pseudocode** (algorithm2e) for three swarm behaviours, mostly re-implementations of published algorithms. Each block is followed by "Additional implementation preferences" bullets and a "systems used and tested" table.
10. **Both theses have defects to avoid.**
    - Kurt: tools that are used but not cited (LightGlue, SIFT/ORB, RANSAC, Levenberg–Marquardt, Sinkhorn, A*); a key component never written down (the adaptive information matrix Ω); claims with no experiment behind them (real-world object navigation, perspective-camera navigation, the third map level); heading typos ("CONCLUSION AND FEATURE WORK").
    - Bilaloğlu: inconsistencies between text and figures; empty equation numbers; a two-page introduction with no problem statement and no contribution list.
11. **Depth scores** (theory / algorithm / method / experiment, on a 1–5 scale): **Kurt 2.5 / 3 / 4 / 3; Bilaloğlu 1.5 / 2 / 3 / 3.**
12. **For the author.** Both theses were accepted with modest theory. The welding-cell thesis can copy Kurt's structure and exceed both on rigor with:
    - pseudocode and complexity notes for the proposed seam extractors and tack placement;
    - one parameter table;
    - a comparison table against prior systems and datasets (Bilaloğlu's Table 2.1 idea);
    - real-robot trials with counts and statistics;
    - literature baselines on WeldSet.

---

## 1. Profile cards

### 1.1 Umut Kurt — profile card

| Field | Value |
|---|---|
| Title | Design and Implementation of an Object-Level Indoor Navigation Algorithm Using Spherical Images |
| Author | Umut Kurt |
| Degree / program | M.S., Department of Robotics, METU Graduate School of Natural and Applied Sciences |
| Date | September 2025 (approval page dated 01.09.2025) |
| Supervisor / co-supervisor | Assoc. Prof. Dr. Ahmet Buğra Koku (Mechanical Eng., METU) / Prof. Dr. Gökhan Bayar (Mechanical Eng., BEUN) |
| Committee | Assoc. Prof. Dr. Ali Emre Turgut (METU), Assoc. Prof. Dr. A. B. Koku (METU), Assist. Prof. Dr. Tolga Özaslan (AYBÜ) |
| Length | 104 PDF pages: xx front-matter pages + 84 numbered pages (the abstract says "84 pages") |
| References | 48 |
| Abstract | One paragraph, about 155 words, passive voice ("is proposed", "is employed", "have been shown") |
| Keywords | Topological Mapping, Hierarchical Map Representation, Visual SLAM, Spherical Imaging, Object-Level Semantics |
| Context | METU ROMER lab. TÜBİTAK 2210-A scholarship. Acknowledges the "contributions and collaboration" of three colleagues (O. Oydu, O. Okay, A. S. Yoncacılar). |

**Full table of contents (printed page numbers)**

```
Abstract v | Öz vii | Acknowledgments x | Table of Contents xii | List of Tables xv
List of Figures xvi | List of Abbreviations xix
1 INTRODUCTION .................................................. 1
  1.1 Motivation and Problem Definition ......................... 1
  1.2 Proposed Methods and Models ............................... 3
  1.3 Contribution and Novelties ................................ 5
  1.4 The Outline of the Thesis ................................. 6
2 LITERATURE REVIEW ............................................. 7
  2.1 Metric Mapping ............................................ 7
  2.2 Topological Mapping ....................................... 9
  2.3 Semantic SLAM ............................................. 10
  2.4 Hierarchical Map Representations .......................... 12
3 METHODOLOGY ................................................... 15
  3.1 Relative Pose Estimation .................................. 15
      3.1.1 Keypoint Feature Extraction ......................... 16
      3.1.2 Spherical Camera Model and Unit Vector Projection ... 17
      3.1.3 Essential Matrix Estimation and Pose Recovery ....... 19
      3.1.4 Robust Estimation using RANSAC ...................... 21
  3.2 Graph-Based Construction of Topological Map ............... 22
      3.2.1 Graph Structure ..................................... 23
  3.3 Visual Place Recognition and Loop Closure ................. 25
      3.3.1 Segmentation ........................................ 28
      3.3.2 Mask Association .................................... 29
      3.3.3 Global Object Representation Aggregation ............ 33
      3.3.4 Object-Based Place Recognition Query ................ 35
      3.3.5 Geometric Consistency Check ......................... 35
4 IMPLEMENTATION ................................................ 39
  4.1 Mapping ................................................... 39
      4.1.1 Hierarchical Map Representation ..................... 41
  4.2 Navigation with 360-Degree View Camera .................... 43
  4.3 Navigation with Narrow Field of View Camera ............... 45
5 RESULTS AND DISCUSSION ........................................ 51
  5.1 Simulation Environment .................................... 51
  5.2 Feature Extraction and Matching Algorithm Comparison ...... 53
  5.3 Information Matrix ........................................ 55
  5.4 Mask Association .......................................... 58
  5.5 Mapping ................................................... 61
  5.6 Navigation ................................................ 67
  5.7 Real-World Mapping and Visual Place Recognition ........... 69
6 CONCLUSION AND FEATURE WORK [sic] ............................. 75
REFERENCES ...................................................... 79
```

**Pages per chapter**

| Chapter | Printed pages | Count | Share of 84 |
|---|---|---|---|
| 1 Introduction | 1–6 | 6 | 7% |
| 2 Literature Review | 7–13 | 7 | 8% |
| 3 Methodology | 15–37 | 23 | 27% |
| 4 Implementation | 39–49 | 11 | 13% |
| 5 Results and Discussion | 51–74 | 24 | 29% |
| 6 Conclusion and Future Work | 75–77 | 3 | 4% |
| References | 79–84 | 6 | 7% |
| Blank verso pages (14, 38, 50, 78) | – | 4 | 5% |

Inside Chapter 3: §3.1 takes about 7 pages (15–22), §3.2 about 3 (22–25) and §3.3 about 12 (25–37). Inside Chapter 5, sections run from 2 pages (5.2, 5.6) to 6 pages (5.5, 5.7).

**Counts**

- **Numbered equations: 32, all in Chapter 3** (3.1–3.32):
  - spherical projection and equirectangular-to-unit-vector conversion (3.1–3.5);
  - the 8-point linear system, SVD and rank-2 projection (3.6–3.10);
  - the four pose hypotheses (3.11–3.14);
  - pose-graph definition, measurement, BFS initialization, residual and cost (3.15–3.27);
  - subspace distance (3.28–3.29), optimal-transport plan and score (3.30–3.31), row z-score (3.32).

  There are also about 6 unnumbered displays: the descriptor set F_m (p. 30), the matrix W (p. 20), and the bearing distances d_ij, δ_ij and Δ (p. 36). **Chapters 4 and 5 contain no equations.** Even the clustering metrics (ARI, homogeneity, completeness) are described in words only (p. 58).
- **Algorithm / pseudocode blocks: 0.** Procedures are told in prose, numbered or bulleted lists and two flowcharts (Fig. 4.1, Fig. 4.3).
- **Figures: 28** (Chapter 3: 3; Chapter 4: 5; Chapter 5: 20).
  - Concept drawings (3): spherical projection, Fig. 3.1; perspective camera vs. sphere arc, Fig. 4.5; centroid-ray schematic, Fig. 4.2.
  - Notation diagram (1): pose graph, Fig. 3.2.
  - Four-panel pipeline figure with a step-by-step caption (1): Fig. 3.3.
  - Block diagram of the mapping pipeline (1): Fig. 4.1.
  - Decision flowchart (1): Fig. 4.3.
  - Qualitative matching comparison (1): Fig. 4.4.
  - Results (20): bar chart, boxplots, log-scale line plot, threshold sweeps, pose graph vs. ground truth, failure images.
- **Tables: 2**, both in Chapter 5: Table 5.1 (heading-error statistics of 5 matchers) and Table 5.2 (peak mask-association scores).

**Experiments** (almost all in simulation: Habitat-Sim + HM3D, 360° RGB at 1024×512)

| # | Section | Setup | Scale | Metric / result |
|---|---|---|---|---|
| E1 | 5.2 | Feature extraction + matching comparison | 288 equirectangular pairs × 36 rotations = 10,368 pairs; 5 methods | Heading error; SuperPoint+LightGlue mean 0.53° vs SPHORB 10.42°, SIFT 33.65°, ORB 46.90° (Table 5.1); timing 0.19–0.58 s |
| E2 | 5.3 | Information matrix: off-plane rotation vs heading error; then PGO with vs without adaptive weighting | Same pairs; 4×4 grid of 16 images at 30 cm; keypoints reduced from 500 to 25 | Translation RMSE about 0.15 vs 0.6 "cm" at 500 keypoints (Fig. 5.5) |
| E3 | 5.4 | Mask association treated as clustering | 144 images; 641 masks of 28 objects (≥15 keypoints, ≥3 frames) | ARI / homogeneity / completeness vs z-score threshold; OT+RANSAC 0.97 / 0.98 / 0.97 |
| E4 | 5.5 | Mapping in 3 environments | ≈3×7 m room; ≈10×22 m with 320 images; ≈10×12 m with 133 images | **Qualitative only** (pose graph plotted against ground-truth node positions); one spurious-edge failure analysed |
| E5 | 5.6 | Image-goal navigation in simulation | 15 scenarios, 14 succeeded; motion steps of 5° and 10 cm | Success rate; waypoint error about 3 cm mean (Fig. 5.16) |
| E6 | 5.7 | Real world: handheld Ricoh Theta X in METU ROMER labs | One loop scenario reported ("scenario 1"); 32 objects | **Qualitative** maps with and without VPR loop closure |

Not evaluated, although claimed in the abstract, the contributions or the conclusion:

- real-robot (or real-world) navigation;
- the narrow-FOV / perspective-camera navigation of §4.3, beyond the qualitative Fig. 4.4;
- the "third level" of the hierarchy (pose graph segmented into regions by object co-visibility, p. 5);
- comparison with existing VPR or SLAM systems;
- end-to-end runtime.

**Stated contributions (§1.3, p. 5, verbatim bullets)**

- "Designed and implemented a framework that constructs dense topological maps using 360° equirectangular images, which also supports navigation with perspective cameras without prior calibration."
- "Introduced a novel VPR method leveraging co-visible object instances and their spatial arrangements, enhancing loop closure detection and localization in visually repetitive or dynamic environments."
- "Designed a multi-level map overlaying objects on a dense pose graph and segmenting regions based on object co-visibility, reducing search space and improving scalability for localization and loop closure."

§1.2 (pp. 3–5) gives four components: (i) spherical image acquisition and topological mapping, (ii) object-based VPR, (iii) semantic object extraction and representation, (iv) hierarchical map-based navigation. The conclusion (p. 75) adds "A pose graph optimization implementation with an adaptive information matrix is developed from empirical tests."

**Contribution type.** Algorithmic and system integration. A pipeline built from pretrained perception models and classical multi-view geometry, with four new components:

- mask association by subspace or optimal-transport similarity, z-score selection and connected components;
- object-based VPR with bearing-angle consistency;
- an adaptive information matrix;
- iterative-RANSAC object localization, plus tangent-image navigation for perspective cameras.

It is validated empirically, mostly in simulation.

**Depth of theory.** Textbook derivations, correctly adapted to the spherical camera. Examples: the cheirality test is replaced by angular reprojection error (p. 21), scale ambiguity is argued to be harmless for topological maps (p. 20), and the measurement is reduced to bearing and yaw (p. 23). There are no proofs and no convergence, observability or complexity analysis. The new components are defined by equations with short property arguments: yaw invariance and translation sensitivity of the disparity Δ (p. 36). The adaptive information matrix is never given as a formula (pp. 25, 57). The subspace rank r, the OT threshold τ, τ_assign and top-k are never given values.

**Pretrained / third-party models, algorithms, data and hardware**

| Item | Role | Where | Cited? |
|---|---|---|---|
| SuperPoint | Keypoints + 256-D descriptors (also restricted to masks) | pp. 17, 30, 43, 53–55 | Yes [35] |
| LightGlue | Attention-based matcher paired with SuperPoint | pp. 53–55, 76 | **No reference entry** |
| OneFormer | Panoptic segmentation (thing + stuff masks) | p. 40; task context pp. 28–29 | Yes [45] |
| SIFT, ORB, BRISK, Brute-Force, FLANN | Hand-crafted baselines | pp. 17, 53–55 | **No** |
| SPHORB; spherical SIFT | Sphere-adapted baselines | pp. 17, 53–55 | Yes [33], [34] |
| LoFTR, DINOv2 | Mentioned as alternatives / future work | pp. 17, 77 | Yes [36], [48] |
| YOLOv10, Faster R-CNN | Rejected alternatives (bounding boxes) | p. 28 | Yes [43], [44] |
| DBoW2, NetVLAD, MixVPR, EigenPlaces, SALAD | Positioned VPR alternatives (not compared) | pp. 26–27 | Yes [38–42] |
| 8-point algorithm, SVD | Essential-matrix estimation | pp. 19–21 | **No original source** |
| RANSAC | Outlier rejection (pose, object localization) | pp. 21–22, 42 | **No** |
| Levenberg–Marquardt ("an implementation of") | PGO solver | p. 25 | **No** (graph-SLAM tutorial [37] cited) |
| Sinkhorn algorithm | Optimal-transport plan | p. 31 | **No** |
| A* | Path planning on the graph | pp. 43, 45, 48 | **No** |
| Habitat-Sim; HM3D | Simulator; 1,000 indoor reconstructions | pp. 51–52 | Yes [46], [47] |
| Ricoh Theta X | Real-world 360° camera (handheld) | p. 69 | Name only |

No software library is named (no OpenCV, PyTorch, g2o or GTSAM), and no compute hardware beyond "a GPU-accelerated device" (p. 54).

### 1.2 Cem Bilaloğlu — profile card

| Field | Value |
|---|---|
| Title | Development of an Extensible Heterogeneous Swarm Robot Platform |
| Author | Cem Bilaloğlu |
| Degree / program | M.S., Department of Mechanical Engineering, METU |
| Date | January 2022 (approval page dated 13.01.2022) |
| Supervisor / co-supervisor | Assist. Prof. Dr. Ali Emre Turgut (ME, METU) / Assoc. Prof. Dr. Erol Şahin (CENG, METU) |
| Committee | Assoc. Prof. Dr. A. B. Koku, Assist. Prof. Dr. A. E. Turgut, Assoc. Prof. Dr. E. Şahin, Assoc. Prof. Dr. M. B. Özer (METU), Assist. Prof. Dr. K. B. Arıkan (TED U.) |
| Length | 104 PDF pages: xviii front-matter pages + 86 numbered pages (the abstract says "86 pages") |
| References | 64 |
| Abstract | About 100 words; claims "multiple novel swarm behaviors implemented for the first time in real robots" |
| Keywords | Swarm Robotics, Multi Robot Systems, Heterogeneous Swarm, Robot Perception, Decentralized Control |
| Extras | List of Algorithms; Appendix with links to videos, design-calculation notebook, CAD, PCB, git repositories and SD-card images |

**Full table of contents (printed page numbers)**

```
Abstract v | Öz vi | Acknowledgments viii | Table of Contents ix | List of Tables xii
List of Figures xiv | List of Algorithms xvii | List of Abbreviations xviii
1 INTRODUCTION .................................................. 1
2 REVIEW OF AVAILABLE SWARM ROBOTS .............................. 3
  2.1 Swarm Robot Design Trends ................................. 3
  2.2 Heterogeneous Swarm Robots ................................ 4
  2.3 Comparison of Swarm Robots ................................ 4
      2.3.1 Contribution of Kobot Platform ...................... 6
3 KOBOT PLATFORM HARDWARE ....................................... 9
  3.1 Kobot CoRe ................................................ 9
      3.1.1 High-level Controller Layer ......................... 11
      3.1.2 Swarm Sensory-Motor, Communication Layer ............ 11
      3.1.3 Locomotion & Power Layer ............................ 23
  3.2 Mechanical Design ......................................... 25
  3.3 Extended Robots ........................................... 27
      3.3.1 Kobot-W ............................................. 27
      3.3.2 Kobot-F ............................................. 28
      3.3.3 Alternative Locomotion Prototypes ................... 29
4 KOBOT PLATFORM SOFTWARE ....................................... 31
  4.1 Swarm Supervision ......................................... 31
  4.2 Behavioral Programming .................................... 33
      4.2.1 Self-organized Flocking ............................. 33
      4.2.2 Cue-based Aggregation ............................... 38
  4.3 Hardware Drivers .......................................... 41
      4.3.1 Floor Sensing ....................................... 42
      4.3.2 Localization ........................................ 43
  4.4 Embedded Firmware ......................................... 45
      4.4.1 Differential Driver ................................. 45
      4.4.2 ToF R&B ............................................. 46
5 EXPERIMENTS ................................................... 53
  5.1 Experimental Setup ........................................ 53
  5.2 System Validation ......................................... 54
      5.2.1 2-DoF Compliant Gripper ............................. 54
      5.2.2 Wheel Odometry ...................................... 55
      5.2.3 Marker-Based Visual Localization .................... 57
      5.2.4 Range & Bearing ..................................... 59
  5.3 Swarm Scenario Validation ................................. 63
      5.3.1 Cue-Based Aggregation ............................... 63
      5.3.2 Self-Organized Flocking wih [sic] Ground Robots ..... 65
6 DISCUSSION AND CONCLUSION ..................................... 69
  6.1 Discussion ................................................ 69
      6.1.1 The CoRe Systems .................................... 69
      6.1.2 Swarm Scenarios ..................................... 73
  6.2 Conclusion ................................................ 75
REFERENCES ...................................................... 77
7 APPENDIX ...................................................... 85
  7.1 Experiment Videos 85 | 7.2 Design calculations 85 | 7.3 CAD Models 86
  7.4 PCB Designs 86 | 7.5 Git Repositories 86 | 7.6 SD Card Images 86
```

Below the subsection level there are about 25 unnumbered run-in paragraph headings, for example "IR Range and Bearing System" (p. 11), "ToF Range and Bearing System" (p. 12), "2-DoF Compliant Gripper System" (p. 18), "Off-the-shelf Systems" (p. 21), "Robot Detection" (p. 47) and "Embedded Filtering" (p. 51). The Discussion chapter repeats the subsystem names as paragraph headings.

**Pages per chapter**

| Chapter | Printed pages | Count | Share of 86 |
|---|---|---|---|
| 1 Introduction | 1–2 | 2 | 2% |
| 2 Review of Available Swarm Robots | 3–7 | 5 | 6% |
| 3 Kobot Platform Hardware | 9–30 | 22 | 26% |
| 4 Kobot Platform Software | 31–52 | 22 | 26% |
| 5 Experiments | 53–68 | 16 | 19% |
| 6 Discussion and Conclusion | 69–75 | 7 | 8% |
| References | 77–84 | 8 | 9% |
| 7 Appendix | 85–86 | 2 | 2% |
| Blank (8, 76) | – | 2 | 2% |

**Counts**

- **Equation tags: 58** (Chapter 3: 27; Chapter 4: 25; Chapter 5: 6). About 42 are real formulas. About 13 are numeric-result lines, e.g. "(3.2) = ±312.1 mm" and "(3.11) m_obj = 251 g". Three are empty tags: (4.5), (4.20) and (4.25).
- **Algorithm blocks: 3** (algorithm2e style, with `Data:` inputs, `/* */` comments and `while true` loops):
  - Algorithm 1, Self-Organized Flocking [1] (p. 35);
  - Algorithm 2, BEECLUST (p. 39);
  - Algorithm 3, Landmark Based Aggregation [2] (p. 40), abridged with "...".
- **Figures: 35** (Chapter 1: 1; Chapter 3: 14; Chapter 4: 7; Chapter 5: 13). The List of Figures omits Fig. 4.3. Types: robot and PCB photos, CAD exploded views, hardware/software flow diagrams (Figs. 3.2, 3.10, 4.1, 4.2), timing diagrams (Figs. 4.5, 4.7), raw signal plots (Fig. 4.6), and experiment plots and snapshots.
- **Tables: 12**, including the comparison of 30 swarm robots (Table 2.1), motor parameters, peripherals, cost breakdown, "systems used" tables for each behaviour (Tables 4.1–4.3), a register map, kinematic parameters, experiment parameter sets (Tables 5.2–5.3) and an algorithm × subsystem summary (Table 6.1).

**Experiments** (all on real hardware; ground truth from a 12-camera Vicon Vantage system, plus a ceiling webcam with ArUco for Kobot-W)

| Section | Experiment | Scale | Outcome |
|---|---|---|---|
| 5.2.1 | Gripper load: glass cup filled with bolts | Trials not stated | Holds 200 g (cylinder) and 580 g (conical top); elevates 120 g |
| 5.2.2 | Wheel odometry on a wall-following trajectory; kinematic parameter estimation | About 10 m path | Displacement error about 4% with design parameters, about 1% after calibration |
| 5.2.3 | ArUco pose estimation | 3 view angles, 250–1000 mm, repeated 5 times | Error vs distance plots (Fig. 5.4) |
| 5.2.4 | IR R&B (2 Kobot-W); ToF R&B ranging (2 Kobot-F, 3 configurations); robot detection with and without random delay; sunlight tests | About 1,400 samples per detection plot | ToF fit y = 1.00x − 38.5, R² = 0.98 |
| 5.3.1 | Aggregation: BEECLUST vs LBA | 6 Kobot-W, 5 repetitions each, 70 min | NAS curves (Fig. 5.10) |
| 5.3.2 | Flocking with Kobot-W at β = 1.3, 1.5, 1.7 (Table 5.3 also lists 2.0); flocking with Kobot-F | Number of runs not stated; Kobot-F only as snapshots | Angular order and equivalence classes (Fig. 5.12); best β = 1.3 |

**Stated contributions.** There is no contribution list in Chapter 1. §2.3.1 (p. 7) ends with "To sum up, Kobot platform:" and four bullets: it uses open-source or educationally licensed programs; it is based on and upgrades with OTS components, 3-D printing and open-source software; it extends to fully onboard ground and aerial robots with a common architecture; it enables multiple algorithms to be implemented on real robots for the first time. "First" claims are repeated across the thesis:

- the first robot-detection-capable ToF R&B (pp. 12, 47, 75);
- Kobot-F as "the first of its kind" using only onboard resources (p. 28);
- the first fully decentralized onboard flocking of indoor aerial robots (pp. 66, 75);
- the first real-robot implementation of LBA (p. 40).

**Contribution type.** Hardware and software platform design plus integration, validated empirically on real robots. Algorithmic novelty is limited to sensor-level mechanisms: random-delay de-synchronization, robot detection from the IQR of the ambient signal, and buffer filtering rules. The swarm algorithms come from the literature.

**Depth of theory.** Closed-form design calculations from datasheet values: FoV height margin, blind-zone width, LED resistor and range, gripper torque and tip-over, encoder resolution, maximum velocity and acceleration. Added to these are textbook differential-drive kinematics and Euler integration, a discretized PID, swarm metric definitions (NAS, angular order, equivalence classes) and one informal probability argument (p. 50). There are no proofs and no uncertainty propagation.

**Third-party components**

- Raspberry Pi 3/4/Zero, Arduino Nano MCUs, Pololu gearmotors (datasheet footnote, p. 23)
- ST VL53L0/VL53L1X ToF sensors with the ST ULD API (footnote, p. 47), Xbee S3, RPi camera v2.1, 9-axis IMU, LED ring, e-ink display
- Ryze Tello drone [31] with telloPy
- ROS (launch files, parameter server, multi-master), with smbus2, pyserial and spidev
- OpenCV ArUco [56, 57], Madgwick filter [59], ROS EKF package [60]
- Vicon Vantage
- Re-used algorithms: flocking [1], BEECLUST [61] and LBA [2]

No learned models are used.

---

## 2. How they explain pretrained and third-party models

### 2.1 Kurt

#### 2.1.1 The overall pattern: a tiered treatment

| Tier | Chapter | What is said | Typical length |
|---|---|---|---|
| Task and requirement | 3 Methodology | What the module must deliver in this application; families of methods with one pro and one con each; why one family fits | 0.5–1.5 pages per task |
| Model | 4 Implementation | Model name, one-sentence characterization, why chosen, observed behaviour on the thesis data, where it runs in the pipeline (thread, rate, coupling) | 3–5 sentences |
| Evidence | 5 Results | Benchmark against alternatives on the downstream task (not on the model's own metric); caveats on timing and hardware; failure images | 1–2.5 pages, 1 figure + 1 table |
| Dependency | 6 Conclusion | The system depends on these modules; known domain gap; what to retrain or replace | 1 bullet each |

The governing rule is an **inverse depth rule**. The more opaque a component is (a pretrained network), the less of its inside is shown, and the more the thesis talks about its role, its inputs and outputs, and evidence of its fitness. The more a component is mathematics that the author adapts or composes (8-point algorithm, PGO), the more it is derived.

#### 2.1.2 Model by model

**(a) SuperPoint (+ LightGlue): keypoint features and matching**

- *Order of presentation*, mostly in §3.1.1 (pp. 16–17):
  1. role in pose estimation;
  2. generic requirements (invariance, repeatability, descriptors);
  3. **application-specific requirements** (even distribution over 360°, real time);
  4. hand-crafted family (SIFT, ORB, BRISK) with pros and cons;
  5. sphere-adapted variants [33, 34];
  6. learned family (SuperPoint [35], LoFTR [36]) in three sentences.

  Later mentions: descriptor dimension (p. 30), use in navigation (p. 43), empirical selection in §5.2 (pp. 53–55), limitations in Chapter 6 (pp. 75–77).
- *Covered:* purpose; requirements; alternatives; that the model is "trained on large datasets ... end-to-end"; output dimension d = 256 (p. 30); how descriptors are restricted to mask regions (pp. 29–30); empirical superiority on the downstream task, with a figure and a table; GPU-dependence caveat; domain-gap limitation.
- *Not covered:* network architecture (shared encoder, detector and descriptor heads), training procedure or data, keypoint budget (500 is implied in §5.3), LightGlue's mechanism (only "attention-based", p. 54) and **any LightGlue citation**.
- *Figures:* no model figure. The evidence figures are the bar chart (Fig. 5.2) and Table 5.1. The qualitative match visualization (Fig. 4.4) serves the tangent-image argument.
- *Quotes:*
  - "With the advancements in deep learning, feature detection and description methods such as SuperPoint [35] and LOFTR [36] have gained popularity. Instead of manually crafted heuristics, these feature extractors are trained on large datasets to detect and describe keypoints in an end-to-end process." (p. 17)
  - "Across all thresholds, the SuperPoint+LightGlue combination consistently outperforms the other methods, with a significant margin." (p. 53)
  - "It is important to note that the reported execution time for SuperPoint+LightGlue corresponds to runs on a GPU-accelerated device and may vary significantly depending on the available computational resources." (pp. 54–55)

**(b) OneFormer: panoptic segmentation**

- *Order of presentation:*
  1. §3.3.1 (pp. 28–29) justifies the **task** without naming any model: why persistent anchors are needed, why detectors are not enough, and a semantic vs instance vs panoptic taxonomy with a running "three chairs" example. It adds the requirement that stuff classes are needed (the floor gives free space).
  2. §4.1 (p. 40) names OneFormer in four sentences: what it is, why it was chosen, how it behaved on spherical images, and its role as a slower parallel thread. It also covers the integration trade-off: optional dynamic-object filtering costs parallelism.
  3. §5.7 (pp. 71–72, Fig. 5.20) shows the failure mode: adjacent desks merged, which shifts the centroid and breaks triangulation.
  4. Chapter 6 (pp. 75–76) states the domain gap (trained on perspective images) and the future work.
- *Covered:* purpose; input/output ("pixel-level class labels along with instance masks", p. 4); reason chosen (state of the art + observed robustness); no fine-tuning; runtime role; failure mode; limitation.
- *Not covered:* backbone, training dataset and label set, input resolution, inference time (only "significantly slower"), which classes count as "dynamic" (the examples "humans or office chairs" appear on p. 28).
- *Figures:* a box labelled "Panoptic Segmentation" in the pipeline block diagram (Fig. 4.1); the failure image (Fig. 5.20).
- *Quotes:*
  - "OneFormer is a recent transformer architecture that unifies semantic, instance, and panoptic segmentation within a single framework. OneFormer was chosen because it provides state-of-the-art performance on segmentation tasks." (p. 40)
  - "In practice, we observed that OneFormer performs robustly on spherical images, producing consistent segmentation masks even in highly distorted image regions, without fine-tuning." (p. 40)
  - "The reason for that faulty localization is the inconsistent mask predictions that our panoptic segmentation model produces." (p. 71)

**(c) Hand-crafted and sphere-adapted baselines (SIFT, ORB, ORB+FLANN, SPHORB)**

- Each gets one characterizing sentence with a pro and a con in §3.1.1, and then a quantitative comparison in §5.2. Example: "SIFT is a pioneering approach that provides accurate, scale, and rotation invariant features, yet its high computational overhead limits its suitability for real-time SLAM systems." (p. 17)
- The comparison is framed by the downstream goal: "We aim to assess how well each algorithm performs under spherical distortions introduced by equirectangular projections, and to find the most suitable for integration into our visual SLAM framework." (p. 53)
- The trade-off is stated explicitly: "While ORB remains the most computationally efficient, the trade-off in accuracy is substantial." (p. 54)

**(d) Alternatives positioned but not used**

- **Object detectors** are rejected in one sentence: "Although object detection models like YOLO [43] and Faster R-CNN [44] are shown to be successful in locating objects, the coarse bounding boxes limit the precision and lack the granularity necessary for scene understanding in cluttered indoor environments." (p. 28)
- **VPR networks** (DBoW2, NetVLAD, MixVPR, EigenPlaces, SALAD) are summarized as one family, and their shared gap motivates the novel method: "VPR methods rely solely on appearance-based associations and do not consider spatial proximity or geometric constraints during retrieval. To integrate into a vSLAM pipeline, a geometric validation step is necessary to verify the consistency of candidate matches." (p. 27) None of them is used as an experimental baseline.
- **LoFTR and DINOv2** come back in future work as denser matchers (p. 77).

**(e) Simulator and dataset (Habitat-Sim, HM3D)**, §5.1, pp. 51–52, about 1.5 pages plus Fig. 5.1

- *Order:*
  1. why testing is needed;
  2. what the simulator offers (photorealistic, large-scale, configurable sensors);
  3. **three enumerated motivations for simulation** (labour, ground truth, controllability);
  4. the photorealism problem, solved by HM3D (1,000 reconstructions, semantic labels, fidelity claim cited [47]);
  5. sensor configuration (equirectangular 1024×512);
  6. batch extraction for large-scale evaluation.
- The dataset's limitation appears in future work: HM3D panoramas lack temporal continuity, which "motivates the generation of new datasets tailored for 360° vSLAM" (p. 76). This rhetoric maps directly onto WeldSet.
- Quote: "The main problem with robotic simulation is obtaining photorealistic observations. To address this, we used the HM3D (Habitat-Matterport 3D) dataset, which is currently one of the most comprehensive sources of 3D indoor environments." (p. 51)

**(f) Classical external algorithms**

| Algorithm | Treatment | Pages |
|---|---|---|
| 8-point algorithm + SVD | Full derivation: linear system Ae = 0 (3.6–3.7), SVD (3.8), rank-2 projection (3.9–3.10), four hypotheses (3.11–3.14); spherical twist on cheirality | 19–21 |
| RANSAC | Prose only: "in theory / in practice" motivation, the loop in words, dynamic iteration count, final re-fit on inliers; no thresholds, no pseudocode, no citation | 21–22 |
| Pose-graph optimization | Full formulation (3.15–3.27) with a notation figure; front-end/back-end framing from [37] | 22–25 |
| Levenberg–Marquardt | Two sentences: "an implementation of"; Jacobian and update loop described verbally | 25 |
| Sinkhorn | Named only, inside the OT method | 31 |
| A* | Named only | 43, 45, 48 |

Quote showing the light touch on solvers: "In practice, we use an implementation of the Levenberg–Marquardt algorithm to solve the nonlinear least-squares optimization problem." (p. 25)

#### 2.1.3 Coverage matrix (Kurt)

| Aspect | SuperPoint | LightGlue | OneFormer | Habitat-Sim / HM3D | 8-PA / PGO |
|---|---|---|---|---|---|
| Purpose / role in pipeline | Yes (pp. 16, 43) | Implicit | Yes (pp. 4, 40) | Yes (p. 51) | Yes (pp. 15–16, 22) |
| Task-level background and taxonomy | Yes (pp. 16–17) | – | Yes (pp. 28–29) | Partial (three motivations) | Yes |
| Architecture | One phrase ("end-to-end") | "attention-based" (p. 54) | One sentence (p. 40) | n/a | n/a |
| Training data | "large datasets" | – | – ("without fine-tuning") | Dataset stats (p. 52) | n/a |
| Inputs / outputs | d = 256 (p. 30) | – | Masks + labels + IDs (p. 4) | 1024×512 equirect (p. 52) | Full |
| Key equations | – | – | – | – | Full (3.1–3.27) |
| Why chosen over alternatives | Benchmark (§5.2) | Benchmark (§5.2) | "state of the art" + observation | Three motivations + fidelity | 8-PA vs 5-point: choice stated, not argued (p. 16) |
| Limitations | GPU timing, domain gap (pp. 54–55, 75) | Mismatch sensitivity (p. 76) | Slow; inconsistent masks; domain gap | No temporal continuity (p. 76) | Scale ambiguity accepted (p. 20) |
| Integration details | Mask-restricted descriptors; navigation heading | Navigation | Thread design, sync option (pp. 39–40) | Batch extraction | — |
| Figure | Results only (Fig. 5.2, Table 5.1) | Same | Pipeline box; failure (Fig. 5.20) | Fig. 5.1 | Figs. 3.1, 3.2 |
| Citation | [35] | **Missing** | [45] | [46], [47] | 8-PA: missing; PGO: [37] |

#### 2.1.4 What Kurt leaves out (a committee could ask)

- One sentence per model on **what it was trained on and why that matters** for the domain gap. Kurt raises the domain gap only in Chapter 6.
- **Version, weights and inference settings**: checkpoint, input resolution, keypoint budget, thresholds.
- **Runtime per module** on stated hardware. Only matcher timings appear, with a hedge.
- **Citations for every tool used.** The LightGlue omission is the most visible gap, because LightGlue is the chosen matcher.

### 2.2 Bilaloğlu

**(a) OpenCV ArUco marker pose: the most complete "module card" in either thesis** (§4.3.2, p. 44; validation §5.2.3, pp. 57–58; discussion p. 70)

- *Order:*
  1. the problem it solves (PnP);
  2. the principle (known 3-D configuration, reprojection error);
  3. alternatives (QR codes, coloured markers);
  4. the choice with two reasons;
  5. **outputs** (ID, tvec, rvec);
  6. uses (robot pose w.r.t. a landmark; Kobot-F tracking Kobot-W, Fig. 4.4);
  7. a library documentation link in a footnote;
  8. validation with view-angle and distance sweeps;
  9. a limitation tied to a library caveat, with a mitigation and a better alternative.
- *Quotes:*
  - "From a general point of view, all three are used together to solve the Perspective-n-Point (PnP) problem." (p. 44)
  - "However, we preferred ArUco markers since they are computationally efficient to detect using the available OpenCV library and embed ID information." (p. 44)
  - "The output of the marker detection and pose estimation are the ID of the ArUco marker, tvec corresponding to x,y,z distances of the marker resolved in the camera frame and minimal representation of 3-D rotation denoted as rvec." (p. 44)
  - "We overcome this issue by tilting the markers vertically with the marker holder parts. However, an even better solution would be to use multiple markers placed on a rigid body at skew angles if landmark size is not a constraint." (p. 70)
- About 0.7 page of description, 1.5 pages of validation and 0.5 page of discussion. One usage figure (Fig. 4.4) and one validation figure (Fig. 5.4).

**(b) ST VL53L0/VL53L1X ToF sensors + ULD API: open the black box only where your novelty acts**

- In §3.1.2 (pp. 12–15), datasheet parameters (half-FoV 13.5°, 1300 mm range) feed design equations (3.1–3.4).
- In §4.4.2 (pp. 46–52), the firmware encapsulates the vendor API. The text then explains the sensor's **internal ranging sequence** (ambient, emit 1, emit 2; Fig. 4.5), because the robot-detection method exploits the ambient phase. Measured evidence follows (Fig. 4.6), then a timing diagram of the fix (Fig. 4.7).
- Quote: "The robot detection is closely related to the ranging sequence of the sensor. For that reason, first, the ranging sequence of the sensor will be presented." (p. 47)
- **Lesson:** describe internals of a third-party component only to the depth that your own method depends on. For the welding cell this could be the D435i depth-noise model, or how FoundationPose consumes a mask and CAD mesh, if your refinement depends on it.

**(c) IMU package + Madgwick filter + EKF: the one-paragraph pass-through** (p. 43)

- Interface, calibrations performed, the filter's role, the output frame, visualization and downstream use, all in three sentences with citations [59, 60]. There are no equations and no validation.
- Quote: "Then, Madgwick filter [59] implementation in ROS is used for filtering IMU messages and combining readings of the sensors to a transformation from a fixed frame to the IMU frame." (p. 43)

**(d) Off-the-shelf boards and platforms (RPi, Arduino, Xbee, camera, LED ring, e-ink, Tello)**

- *Selection rationale at platform level:* "Thus, the CoRe is based upon off-the-shelf, mature hardware, which will not change any time soon." (p. 9)
- *Per item:* a bullet list on p. 21 gives function, configuration choice and interface for each item. Example: the camera runs at 480p5 to minimize CPU load, and the IMU "communicates with the RPi using the I2C bus".
- *Black-box platform in two sentences:* "In Kobot-F, a commercial quadrotor Ryze Tello [31] is used as the flying base. Tello interfaces only with the RPi to receive the reference velocity command and feedback internal IMU readings." (p. 25)

**(e) ROS and open-source tooling: an explicit scope policy**

- "Therefore, rather than describing how the software tools work, which are documented in the source files, the subsections corresponding to the layers in Figure 4.1 explain how we leveraged the open-source software tools to solve challenges related to swarm robots." (p. 31)
- The small libraries (smbus2, pyserial, spidev, telloPy) appear only as boxes in the modularity diagram (Fig. 4.2), with a caption that explains how to read the diagram. OpenCV is the only library that also gets a sentence in the text (p. 44).

**(f) Re-used algorithms from the literature (flocking [1], BEECLUST, LBA [2])**

- The pattern is to cite the source, show pseudocode, then explain only the hardware-specific adaptation. For example, the virtual force law is quadratic for the noisy IR sensor (Eqs. 4.1–4.2) and linear for the ToF sensor (Eq. 4.3), with the reason for each.
- After that come a bullet list of "Additional implementation preferences", a "systems used and tested" table, and in the Discussion a check against published results.
- Quote: "The optimal value for the weight of proximal control is β = 1.3 by our experiments, and the results are in agreement with the results published by [1]." (p. 74)

**(g) Ground-truth system (Vicon).** Number of cameras, marker configuration, and a second interchangeable low-cost system (pp. 53–54), with a brochure footnote.

### 2.3 Synthesis: a "module card" for the author's pretrained components

This combines Kurt's tiers with Bilaloğlu's I/O discipline. Apply it to SAM2, FoundationPose, the hand-gesture recognizer, and the ICP / OMPL implementations used.

1. **Methodology (task level, no brand names needed).** State what the stage must deliver in the cell, as a requirement:
   - e.g. a per-part mask with a stable identity across views;
   - e.g. a 6-DoF pose of a known CAD part within x mm at the working distance.

   Give the families of solutions with one pro and one con each, and why the chosen family fits (CAD-model-based, zero-shot, promptable). Use Kurt's taxonomy-with-running-example device (the "three chairs" of p. 29). A welding analogue could be "two identical gusset plates".
2. **Implementation (model level).** Write 4–6 sentences covering:
   - name and citation;
   - one sentence on architecture or principle;
   - **one sentence on training data and whether you fine-tuned** (Kurt omits this; add it);
   - inputs and outputs with types, frames and rates (Bilaloğlu's "The output ... are ..." sentence);
   - why this model over the alternatives (benchmark, citation or constraint, e.g. GPU latency);
   - where it runs (process or container, thread, rate) and what happens when it fails.
3. **Results (evidence level).** Give a downstream-task comparison or ablation if you have one, a caveat on hardware and timing, and at least one failure image with its root cause (Kurt Fig. 5.20 style).
4. **Conclusion (dependency level).** State the dependency and the domain gap, and what would be retrained or replaced.
5. **Equations only where your own math consumes the output.** Examples: pose composition, ICP cost, multi-view fusion. Do not reproduce network internals.
6. **Cite every tool,** including classical algorithms (RANSAC, ICP variant, OMPL planners) and libraries (Open3D, MoveIt, etc.), as applicable.

---

## 3. How they explain their own algorithms

### 3.1 Kurt's template

```
Context sentence (why this step matters in the system)
 -> Goal / problem statement ("Given ..., the goal is ...")
 -> Standard approach and where it fails here (often an escalating sequence of baselines)
 -> Proposal statement ("In this thesis, we ... / we propose ...")
 -> Advantages (bulleted) and/or decomposition into subtasks ("can be divided into three subtasks")
 -> For each subtask: notation -> modelling assumption -> equations -> what the output means
 -> One summarizing figure with a step-numbered caption
 -> Property analysis (invariance, sensitivity, cost) in prose
 -> [Chapter 4] how it is wired: threads, data structures, thresholds, flowchart
 -> [Chapter 5] tuning/ablation that justifies choices; failure cases with remedies
 -> [Chapter 6] residual limitation and future work
```

Devices that **are** used: numbered lists for competing formulations (two similarity methods, pp. 30–31) and for data-structure fields (database entry, p. 34); bullets for advantages, modes and future work; pipeline figures; flowcharts in Chapter 4; forward references to experiments.

Devices that are **not** used: algorithm or pseudocode environments, complexity analysis beyond qualitative statements ("quadratically", "significantly reduced"), parameter tables, and formal problem statements of the form "Given ... find ... such that ...". The only exception is the PGO arg-min (3.27).

### 3.2 Worked example A: object-based VPR (§3.3, pp. 25–37), Kurt's showcase

1. **Problem from the system's point of view** (pp. 25–26). Map building needs neighbouring image pairs, and this matters for both PGO and path planning.
2. **Escalating baselines with failure reasons** (p. 26):
   - exhaustive pairwise search (quadratic): "As a result, exhaustive search across all graph nodes is not a feasible method for real-time applications and computationally constrained systems." (p. 26);
   - a temporal window (misses loops);
   - spatial-proximity candidates (fail under drift): "A major flaw in the described approach is the detection of revisited places in the presence of a high accumulation of error." (p. 26).

   A concrete scenario anchors it: the office corridor loop (p. 26).
3. **Literature family and its gap** (pp. 26–27). Global-descriptor VPR is appearance-only and needs geometric validation.
4. **Proposal statement** (p. 27). It combines the temporal window, spatial candidates and the novel object-based VPR, followed by a scoping note: "Note that these landmarks are not predefined or specially structured objects but consist of everyday items that can be commonly found in indoor environments." (p. 27)
5. **Four advantages as bullets** (pp. 27–28): search over objects rather than images, partial similarity, robustness to dynamic objects, and geometric validation without world alignment.
6. **Decomposition into subtasks**, which become the subsections (p. 28). "This process can be divided into three subtasks: identifying and describing objects in each frame, finding correspondences between instances of objects across frames, and aggregating the descriptors for all instances of each object to obtain a global representation." (p. 28)
7. **§3.3.1 Segmentation** (task choice through a taxonomy) → **§3.3.2 Mask association**:
   - notation O, F_m, d = 256;
   - soft assignment by **two alternative similarity measures**, each introduced as *assumption → computation → equation*;
   - hard assignment by row z-score (3.32);
   - an association graph whose connected components are objects;
   - summary Fig. 3.3, whose caption numbers the four steps (rendered: thumbnails → similarity matrix → z-score plot → graph).
8. **§3.3.3 Aggregation**, which mirrors the two similarity methods as two bullets, then the database schema as a numbered list of three fields (pp. 33–34).
9. **§3.3.4 Query** (p. 35): similarity matrix S (n×m), threshold τ_assign, node score from the share of co-visible matched objects.
10. **§3.3.5 Geometric check** (pp. 35–36): pairwise bearing distances, disparity Δ, then a **property analysis**. "This method has the advantage of not requiring a global orientation: since only relative angles are compared, a pure yaw rotation (e.g., a robot turning in place) shifts all bearings equally, leaving the pairwise angular differences unchanged." (p. 36) "However, it is sensitive to translation, which changes the observed pairwise angles. The approach is also computationally efficient as it involves only simple trigonometric operations such as dot products and arccos." (p. 36) This is followed by top-k selection, confirmation by feature matching, and loop-closure insertion.
11. **Chapter synthesis** (pp. 36–37), opening with "To conclude, the methodology presented in this chapter outlines a robust approach ..." (p. 36).
12. **Implementation** (pp. 39–42): the segmentation thread, and object localization by ray intersection with iterative RANSAC (Fig. 4.2, with its colour code explained in the text).
13. **Evaluation** (§5.4, pp. 58–61). The task is reformulated as clustering ("For the numerical evaluation of our mask association methods, we formulate the task as a clustering problem and assess performance using three widely used clustering evaluation metrics.", p. 58). Then come the dataset filter, threshold sweeps (Figs. 5.6–5.7) with the trade-off explained, the RANSAC disambiguation example (Fig. 5.8) and Table 5.2.
14. **Limitations** (p. 71): object-poor corridors, inconsistent masks.

Gaps: no pseudocode, no runtime numbers for the VPR query, unspecified r, τ, τ_assign and k, and no comparison with NetVLAD-type baselines.

### 3.3 Worked example B: relative pose and pose graph (§3.1–3.2, pp. 15–25)

- **§3.1 opener** (pp. 15–16): importance → goal → typical pipeline → spherical twist (E and F coincide) → method choice. "In this thesis work, 8-PA coupled with RANSAC is used to robustly estimate the essential matrix by removing outlier correspondences." (p. 16)
- **§3.1.2 camera model**: contrast with the perspective model, then the conversion chain (pixel → angles → unit vector, Eqs. 3.3–3.5). The physical caveat is stated: "Although the spherical camera model explained so far provides a convenient mathematical formulation, there is no single sensor that can capture light from all directions." (p. 18)
- **§3.1.3 estimation**: linear system → SVD → rank-2 projection with a geometric interpretation ("This step can be interpreted as the projection of the estimated essential matrix onto the space of valid essential matrices.", p. 20) → four hypotheses → spherical replacement of the cheirality test (p. 21). Scale ambiguity is turned into a non-issue by the system design (p. 20).
- **§3.1.4 RANSAC**: the "in theory / in practice" motivation (p. 21), then the loop in prose (pp. 21–22).
- **§3.2 PGO**: why consistency still matters for topology (p. 22) → front-end/back-end → graph definition (3.15–3.18) → scale-free unit translation (p. 23) → BFS initialization, justified by local minima (pp. 23–24) → residuals with the notation figure (Fig. 3.2) → information matrix justified by a forward reference to an experiment (p. 25) → cost (3.27) → solver.

### 3.4 Worked example C: algorithms that live in the Implementation chapter

- **Object localization** (§4.1.1, pp. 41–42): data linkage between layers → centroid-to-yaw ray casting → intersection → RANSAC hypothesis and inlier count → iterative removal to split look-alike objects → Fig. 4.2, with the reading guide in the text. "Since noise in feature detection and segmentation can lead to inaccurate or contradictory ray intersections, we adopt a RANSAC-based refinement procedure." (p. 42)
- **Navigation with a 360° camera** (§4.2, p. 43): two modes as bullets → reactive control loop in prose with explicit thresholds → flowchart (Fig. 4.3, with decision diamonds). "If the normalized similarity score exceeds the threshold of 0.9, the local target node is considered reached." (p. 43)
- **Navigation with a narrow FOV** (§4.3, pp. 45–49): motivation → why standard methods fail (no intrinsics) → idea (use the spherical nodes) → sub-obstacle (distortion) → tangent images introduced with an intuitive image ("Conceptually, this can be imagined as placing a virtual camera at the center of the sphere, pointing in that direction.", p. 46) → qualitative evidence (Fig. 4.4) → routine → FOV/distance ambiguity → calibration phase → geometric intuition (Fig. 4.5: "To visualize, imagine a unit sphere ...", p. 48) → loop.
- **Lesson.** Kurt keeps general-purpose algorithms in Methodology and puts *behaviours that assemble them* (localize-then-plan-then-act loops) in Implementation, explained with flowcharts and threshold values. For the welding cell, a natural home in Implementation would be the gesture-driven operator workflow and the marking routine's force-gated state logic. The seam computation and tack placement would sit in Methodology.

### 3.5 Bilaloğlu's templates

**(a) Design-calculation pattern** (ToF ring, LED resistor, gripper, drive train)

The steps are: specification list → datasheet constants → closed-form expression → numeric value (often tagged as an equation) → engineering choice with a margin → caveat → validation in Chapter 5 → reconciliation in Chapter 6.

The gripper case shows all of it:

- specification bullets (p. 18);
- geometric sizing by CAD constraints (p. 19);
- torque balance giving 251 g (p. 20): "This value corresponds to a theoretical maximum and is used only as an initial design calculation." (p. 20);
- tip-over check (p. 20);
- measured 120 g (p. 54);
- discussion of the 52% loss with ranked causes (p. 69): "We believe the most prominent cause of this loss is the flexibility of the components." (p. 69).

**(b) Hypothesis-driven mechanism design** (ToF robot detection, pp. 47–52)

1. firmware responsibilities as bullets;
2. the sensor's internal sequence (Fig. 4.5);
3. hypothesis by analogy ("carrier sense", p. 48);
4. a minimal test rig: "A setup comprising two ToF sensors was configured without a random delay and aligned with intersecting FoVs to test that hypothesis." (p. 49);
5. an observed problem, false negatives when the sensors are in sync (p. 49);
6. the fix, a random delay, with a timing diagram (Fig. 4.7);
7. a worst-case probability argument (p. 50);
8. parameter derivation, a buffer of 5 at 66 Hz → 76 ms versus robot speed (p. 51);
9. filtering rules as three bullets (p. 51);
10. the metric chosen empirically (IQR, p. 52).

**(c) Behaviour pattern** (Chapter 4)

Pseudocode block → concept (virtual spring) → sensor-specific force law with the reason for its form → constants derived from range limits (Eqs. 4.4–4.5) → adaptation to the second platform (height control, larger spacing) → force-curve figure (Fig. 4.3) → "Additional implementation preferences" bullets (angle wrapping, Eq. 4.7) → "systems used" table.

**(d) Software architecture pattern.** A layered diagram (Fig. 4.1) whose layers become the section headings (4.1 Swarm Supervision, 4.2 Behavioral Programming, 4.3 Hardware Drivers, 4.4 Embedded Firmware). Each section states the problem it solves first: "Operating a swarm task is challenging for the researchers. We addressed this issue in the CoRe by using a unicast network between robots and the ‘base computer’." (p. 31)

### 3.6 Side by side: which devices each thesis uses for its own methods

| Device | Kurt | Bilaloğlu |
|---|---|---|
| Algorithm or pseudocode environment | Never | 3 blocks (List of Algorithms) |
| Numbered step lists | Yes: alternatives and schema (pp. 30–34); caption steps (Fig. 3.3) | Mostly bullets |
| Flowcharts and block diagrams | Fig. 4.1 (blocks), Fig. 4.3 (decision flowchart) | Hardware/software flow diagrams (Figs. 3.2, 4.1, 4.2) |
| Concept or geometry figures | Figs. 3.1, 3.2, 4.2, 4.5 | Timing diagrams (Figs. 4.5, 4.7) |
| Equations for own method | Yes (3.28–3.32, Δ) | Yes (force laws, filters, metrics) |
| Explicit modelling assumptions | Yes ("This formulation assumes ...", p. 30) | Implicit ("theoretical maximum") |
| Parameter values | Scattered in Chapters 4–5 | Tables 5.2–5.3 plus text |
| Complexity / runtime | Qualitative; matcher timings | Sampling rates and latencies (66 Hz, 76 ms) |
| Validation tied to method | Ablations (§5.3, §5.4) | Component tests (§5.2), then scenarios (§5.3) |
| Failure analysis | Strong (pp. 62–64, 68, 71) | Calculation-vs-measurement reconciliation (Chapter 6) |

---

## 4. Language and reasoning style

### 4.1 Quantitative profile (Chapters 1–6, automated)

| Metric | Kurt | Bilaloğlu |
|---|---|---|
| Body words (approx.) | 17,900 | 16,300 |
| Sentences analysed | 765 | 611 |
| Mean / median sentence length (words) | 22.7 / 20 | 25.0 / 20 |
| Standard deviation of sentence length | 12.0 | 19.8 |
| Sentences over 35 words, by chapter | 5–13% | 8–19% |
| "we" (by chapter 1–6) | 84 (1, 2, 50, 13, 18, 0) | 76 (3, 6, 15, 21, 15, 16) |
| "our" | 33 | 9 |
| First-person "I" | 0 | 0 |
| "this thesis" / "this work" | 11 / 3 | 4 / 2 |
| "the proposed ..." | 32 | 6 |
| "one can / must / should / might" | 1 | 23 |
| Passive constructions per 100 sentences (approx.) | about 39 | about 34 |
| "Although" / "Since" / "While" | 13 / 19 / 10 | 5 / 2 / 0 |
| "However," / "In contrast" | 14 / 8 | 21 / 1 |
| "limitation(s)" / "failure" | 12 / 6 | 0 / 0 |
| "In other words" / "Accordingly" / "Note that" | 0 / 0 / 1 | 6 / 6 / 6 |
| "(see Fig. ...)" | 0 | 15 |

**Reading.** Kurt writes concessive, causal academic prose ("Although ...", "Since ...", "While ...") and names limitations explicitly. Bilaloğlu writes engineering-notebook prose ("one can calculate ...", "In other words ...", "Accordingly ...") and rarely uses the word "limitation"; he shows limits through numbers and caveats instead.

### 4.2 Voice

- **Kurt.**
  - Impersonal in Chapters 1–2: only 3 "we" in 164 sentences. Third-person subjects carry the argument ("Topological maps ...", "SLAM ...").
  - "We" for his own methodological agency in Chapters 3–5 ("we extract features", "we propose", "we adopt", "we observed").
  - "this thesis" or "this thesis work" for scope and choices ("In this thesis work, 8-PA coupled with RANSAC is used ...", p. 16).
  - "the proposed framework / method / approach" (32 times) as the subject of claims.
  - "the system / the algorithm / the framework" as the agent of runtime behaviour ("the algorithm performs an initial heading and FOV calibration phase", p. 48).
  - The passive for standard procedures ("Outlier matches are filtered using RANSAC", p. 4).
  - Never "I". The abstract is entirely passive.
- **Bilaloğlu.** "We" as designer ("we modified", "we took a different approach", "we believe"); textbook "one can" for derivations; "should" in specifications ("Gripper system should be placed as low as possible", p. 18); passive in hardware descriptions. Never "I".

### 4.3 Sentence length and paragraph shape

- **Kurt.** Typical sentences have a main clause plus one subordinate or participial clause. The variance is low and sentences over 40 words are rare. Paragraphs run 4–8 sentences and usually end with a **consequence sentence**: "Therefore, ...", "This design choice ensures that ...", "This makes ...". Definitions are tight and use symbols ("where µi and σi are the mean and standard deviation of similarity scores of the ith row", p. 32).
- **Bilaloğlu.** Higher variance: some long, loosely chained sentences and a few fragments, e.g. "Because the IR R&B system is noisy, low resolution, and reports non-linear distance readings." (p. 35). Calculation paragraphs alternate one prose sentence, one display, one prose sentence.

### 4.4 How chapters open and close

- **Kurt opens almost every chapter with a roadmap paragraph** that names the components in the order they will appear:
  - "This chapter introduces the motivation, problem definition, and objectives of this thesis. It begins by discussing the limitations of traditional metric SLAM approaches and the advantages of topological and vision-based mapping for autonomous navigation." (p. 1)
  - "In this chapter, we outline our approach to visual robot localization and topological mapping using spherical camera images. Our method integrates three main components: relative pose estimation, graph-based map construction, and object-based visual place recognition." (p. 15)
  - "In this chapter, we present the implementation details of the proposed algorithms, beginning with the mapping procedure. Building on this, we introduce two navigation strategies that operate on the constructed topological map." (p. 39)
  - The exception is Chapter 5, which jumps straight into §5.1. Its first sentence works as a chapter opener ("The proposed visual SLAM pipeline comprises several components that require extensive testing to inform design choices and evaluate the performance and robustness of the algorithms.", p. 51).
- **Kurt closes the Methodology chapter with a synthesis paragraph** (pp. 36–37) that restates the components and their joint benefit. In the literature review, every section closes with a gap synthesis: "In general, scalability, sensitivity to measurement noise, dependence on sensor calibration, long-term robustness, and lack of semantics remain as open challenges for metric methods." (p. 8)
- **Bilaloğlu opens chapters with design principles or a link to previous chapters.** "Initial calculations for the design decisions were performed in previous Chapters 3 and 4. However, it is not feasible to verify all the CoRe systems without an experimental setup providing ground truth pose information." (p. 53) Chapter 3 opens with principles (upgradability, expandability) before the roadmap line (p. 9).

### 4.5 How sections open

Kurt's section openers fall into three types:

1. **Importance → goal:** "Estimating the relative pose between two images taken from distinct viewpoints is a crucial step in robot localization and mapping. Given a pair of images captured from two different poses, the goal is to determine the transformation that relates one camera center to the other." (pp. 15–16)
2. **Field-level framing:** "Graph-based representations offer an intuitive way of formulating the SLAM problem that prioritizes connectivity and spatial relations over metric accuracy." (p. 22)
3. **Chained "next challenge":** "Once the individual objects have been segmented and identified, the next challenge is to find correspondences of the same objects across multiple frames." (p. 29)

Results sections open with **setup → metrics**. Example: §5.2 states its aim, the dataset construction and the metrics before any result (p. 53).

Bilaloğlu's section openers are mostly problem statements ("Operating a swarm task is challenging for the researchers.", p. 31) or the goal of the experiment ("Verifying the critical systems of the proposed design is the first goal of experimentation with Kobot robots.", p. 54).

### 4.6 How design decisions are justified

Kurt's chain is **problem → requirement → choice → consequence**, often with a stated trade-off.

- *Requirement from the application:* "In our application, the even distribution of features across the 360° FOV is especially important. The accumulation of keypoint features along a narrow direction can dominate RANSAC and result in biased relative pose estimations." (pp. 16–17)
- *Choice framed as a balance:* "Therefore, the selection of a feature extraction algorithm that finds a good balance between robustness, accuracy, and computational efficiency is essential for a precise pose estimation pipeline." (p. 17)
- *Architecture choice with consequence:* "The framework runs feature-based pose estimation and panoptic segmentation in parallel threads, since the panoptic segmentation runs significantly slower. This design choice ensures that the mapping is not delayed by the slower process of segmentation." (p. 39)
- *Trade-off made configurable:* "In practice, the framework can be configured to either prioritize real-time operation by running the threads asynchronously or prioritize robustness by synchronizing them." (p. 40)
- *Turning a limitation into a non-issue through system scope:* "The topological framework used in this work eliminates the dependence on metric scale knowledge. The relative heading information between two camera poses is sufficient to construct a topological graph." (p. 20)
- *Empirical justification promised in Methodology and delivered in Results:* "Through an experimental study, we observed a linear relationship between off-plane rotation and heading error. Using this relationship, we constructed a form of information matrix." (p. 25), then "This empirical correlation justifies the use of an adaptive information matrix in the pose graph optimization, ..." (p. 57).

Bilaloğlu justifies by **constraint and calculation**. "Bus-type architectures are preferred since it is straightforward to add new peripherals without introducing extra complexity to the wiring scheme." (p. 11) "In contrast to using a configuration with overlapping FoVs, we took a different approach considering that the robots will be moving and blind zones will move with the robots. Hence, we restricted the blind zones to be smaller than the diagonal projection of the robot onto the sensor plane." (p. 14)

### 4.7 How alternatives are discussed

- **Kurt.**
  - *Category sweeps* with one strength and one weakness each ("X ..., yet Y limits ...", p. 17).
  - Literature items in the form "X [n] does A. However, X relies on B." ("However, FAB-MAP relies heavily on handcrafted visual features, and its performance degrades under drastic appearance changes.", p. 9).
  - *Taxonomies with a running example* (chairs, pp. 28–29).
  - *Two of his own alternative formulations,* distinguished by their assumption (subspace vs. OT, pp. 30–31) and then compared empirically (§5.4).
  - *Baseline benchmarks* on the downstream task (§5.2).
  - What he does not do is compare his novel VPR against existing VPR methods experimentally.
- **Bilaloğlu.**
  - A big comparison table of 30 robots (Table 2.1), from which the required attributes are "deducted" (p. 3).
  - Tried-and-failed alternatives inside design narratives ("The first approach we took was to make the robots’ cross-sections larger and fill the gaps of truss-like structures.", p. 12; it failed because of drag and negligible signal gain, p. 13).
  - Choosing among library options (QR vs coloured vs ArUco, p. 44).

### 4.8 Transitions between sections

Kurt writes backward-and-forward links:

- "Once X ..., the next challenge is Y" (p. 29);
- "Once mask associations are established, ..." (p. 33);
- "After the global object database has been constructed, it can be queried to detect previously visited places ..." (p. 35);
- "Building on this, ..." (p. 39);
- "Based on this initial estimate, ..." (p. 27).

He also uses forward references to later proof, e.g. E = F under the spherical model "as will be demonstrated mathematically later in this chapter" (p. 16), and backward references by section number ("The front-end, described in detail in Section 3.1, ...", p. 22).

Bilaloğlu chains with "Next, ...", "Then, ...", "Lastly, ...", "Moreover, ...", "Over and above that, ...", and points to later chapters ("will be discussed further in Chapter 5", p. 42).

### 4.9 Figures: how they are referenced and captioned

- **Kurt.** Every figure is introduced in a sentence before or next to it. He uses a small set of verbs: *illustrates, is shown in, is summarized in, is depicted in, compares, presents*.
  - "The mask association pipeline is summarized in Figure 3.3. Starting with local panoptic segmentation masks, one can observe that local object IDs change between frames." (p. 32)
  - "To provide a clearer overview of the proposed navigation algorithm, the overall architecture is depicted in Figure 4.3." (p. 43)

  The introduction is then followed by a **reading guide** that explains encodings: "Each node projects the centroid of its associated object mask into the map coordinate frame, represented by an arrow. The intersection of consistent nodes, shown in green, determines the estimated object position, while inconsistent nodes, shown in red, are treated as outliers and excluded." (p. 42) It may also explain axes ("In both plots, the vertical axis represents ...", p. 55) or panels: "In Figure 4.4a, the perspective image is matched directly to an equirectangular image. Although the visual similarity is evident, the number of detected matches is low, and almost all correspond to incorrect feature pairings." (p. 46)

  Captions are short noun phrases ("Flowchart of the proposed mapping pipeline."). There are two deliberate exceptions. Fig. 3.3 has a step-numbered caption with bold labels, "(1) Per-frame masks ... (4) Association graph", which is self-contained. Fig. 4.5's caption explains the geometry in three sentences. Result captions name the layout ("Pose graph (top) and corresponding ground-truth node positions (bottom) ...").
- **Bilaloğlu.** Parenthetical "(see Fig. 3.4)" style. Captions name the plot type and the statistic: "Shaded line plots showing mean, minimum, and maximum errors of pose estimation vs. distance to marker, with 480p camera resolution" (Fig. 5.4). Sub-panels (a)/(b)/(c) are described in the caption. The architecture diagrams carry colour/arrow legends and a long caption that explains the "/" notation (Fig. 4.2).

### 4.10 How limitations are admitted

- **Kurt admits limits at the point of evidence,** in a fixed sequence: observation → mechanism → fix → residual risk → principled future remedy.
  - "During early experiments, a failure case was observed in the graph construction process. Specifically, our system relies on a keypoint threshold to decide whether two nodes should be considered neighbors." (p. 62)
  - "The problem of incorrect matches can lead to a critical failure mode, as they alter the connectivity and perceived traversability of the environment." (p. 63)
  - "A more robust approach would be to design the optimization to explicitly handle such spurious edges, for example, by integrating robust cost functions, switchable constraints, or outlier rejection mechanisms that eliminate inconsistent measurements." (pp. 63–64)
  - "Nevertheless, the pose graph maintains a topologically accurate representation suitable for navigation purposes." (p. 64) This is the closing reassurance.
  - "A current limitation of the proposed method is that VPR performance relies on the presence of distinctive objects." (p. 71)
  - Hedged positive claims: "Although the method does not explicitly target metric accuracy, the results demonstrate that the arrival at the waypoints is detected with high precision, indicating robust performance of the navigation system." (p. 69)
- **Bilaloğlu admits limits** through "theoretical maximum" caveats (pp. 20, 24), scope restrictions inside experiments ("Thus, we limited the active region of pose estimation to 1000[mm] and ±70° for the experiments.", p. 57), and explanations of the gap between calculation and measurement in the Discussion (p. 69).

### 4.11 Writing pattern catalogue (20 patterns)

Patterns 1–16 are Kurt; 17–20 are Bilaloğlu. Each entry gives the pattern, when to use it, and examples.

**1. Bracketed chapter (roadmap opener + synthesis closer).** Open with a paragraph that names the components in order; close with a paragraph that restates their joint benefit.
- "In this chapter, we outline our approach to visual robot localization and topological mapping using spherical camera images. Our method integrates three main components: relative pose estimation, graph-based map construction, and object-based visual place recognition." (p. 15)
- "To conclude, the methodology presented in this chapter outlines a robust approach to topological mapping and localization using spherical cameras." (p. 36)

**2. Importance → goal → standard pipeline → our choice (section opener).**
- "Estimating the relative pose between two images taken from distinct viewpoints is a crucial step in robot localization and mapping." (p. 15)
- "In this thesis work, 8-PA coupled with RANSAC is used to robustly estimate the essential matrix by removing outlier correspondences." (p. 16)

**3. Requirement "in our application" → "Therefore, the selection ... is essential".** Derive the selection criterion from the application before naming any method.
- "In our application, the even distribution of features across the 360° FOV is especially important." (p. 16)
- "Therefore, the selection of a feature extraction algorithm that finds a good balance between robustness, accuracy, and computational efficiency is essential for a precise pose estimation pipeline." (p. 17)

**4. One-line pro/con for each alternative ("X provides A, yet B limits it").** For literature and for method families.
- "SIFT is a pioneering approach that provides accurate, scale, and rotation invariant features, yet its high computational overhead limits its suitability for real-time SLAM systems." (p. 17)
- "However, FAB-MAP relies heavily on handcrafted visual features, and its performance degrades under drastic appearance changes." (p. 9)

**5. Taxonomy with a running concrete example.** It justifies a task choice (here, panoptic segmentation) through a familiar case.
- "Returning to the chair example, instance segmentation would not just classify chairs but also distinguish them as chair 1, chair 2, chair 3." (p. 29)
- "Knowing the boundaries of such objects may be critical for our indoor navigation task. For instance, we can infer free space by detecting the boundaries of the floor." (p. 29)

**6. "Unlike the standard case ..., therefore ..." (adapting a textbook method to your setting).**
- "Unlike perspective cameras, where projection is modeled as x = K [R | t] X, the spherical model does not include intrinsic parameters such as focal length or principal point. Therefore, camera extrinsics are sufficient to describe its relationship with the world." (p. 18)
- "This positive depth constraint is applicable for perspective cameras. However, in the case of the spherical cameras, the term “in front of the camera” becomes irrelevant due to the 360° FOV." (p. 21)

**7. "In theory ... However, in practice ... Therefore, ... is required."** Motivates a robustness step.
- "In theory, having at least eight feature correspondences is sufficient to estimate the relative pose between two spherical cameras. However, in practice, feature correspondences often contain outliers due to sensor noise, image distortions, and errors in the feature matching pipeline." (p. 21)

**8. Escalating baselines with a concrete failure scenario, then the proposal.**
- "A major flaw in the described approach is the detection of revisited places in the presence of a high accumulation of error." (p. 26)
- "For example, in a scenario where the robot traverses a long corridor loop, as commonly found in office buildings, drift tends to accumulate as a result of the robot moving in a long, straight trajectory with only incremental measurements." (p. 26)

**9. Decomposition sentence + chained "next challenge" transitions.** The decomposition maps one-to-one onto subsections, and each subsection opens by linking to the previous one.
- "This process can be divided into three subtasks: identifying and describing objects in each frame, finding correspondences between instances of objects across frames, and aggregating the descriptors for all instances of each object to obtain a global representation." (p. 28)
- "Once the individual objects have been segmented and identified, the next challenge is to find correspondences of the same objects across multiple frames." (p. 29)

**10. Explicit modelling assumption → when it fails → a second formulation.**
- "This formulation assumes that the descriptors belonging to the same physical object lie approximately on a linear subspace." (p. 30)
- "While the subspace-based approach assumes that descriptors associated with a physical object lie approximately on a linear subspace, this assumption may not hold if the descriptors lie on a nonlinear manifold rather than a strictly linear subspace." (p. 31)

**11. Proposal statement + scoping note + advantages list.**
- "Note that these landmarks are not predefined or specially structured objects but consist of everyday items that can be commonly found in indoor environments." (p. 27)
- "In this approach, we search for objects instead of images, as opposed to other VPR methods. Since the number of distinctive objects is smaller than the number of images in a densely mapped environment, the search complexity is significantly reduced." (p. 27)

**12. Property analysis right after the equations (invariance, sensitivity, cost).**
- "This method has the advantage of not requiring a global orientation: since only relative angles are compared, a pure yaw rotation (e.g., a robot turning in place) shifts all bearings equally, leaving the pairwise angular differences unchanged." (p. 36)
- "However, it is sensitive to translation, which changes the observed pairwise angles. The approach is also computationally efficient as it involves only simple trigonometric operations such as dot products and arccos." (p. 36)

**13. Design choice → consequence → trade-off made configurable.**
- "The framework runs feature-based pose estimation and panoptic segmentation in parallel threads, since the panoptic segmentation runs significantly slower. This design choice ensures that the mapping is not delayed by the slower process of segmentation." (p. 39)
- "However, enabling this filtering step requires waiting for the segmentation output before feature matching can proceed. As a result, segmentation becomes the bottleneck, and the two modules no longer operate fully in parallel." (p. 40)

**14. Evaluation design justified by enumerated motivations.** This is directly reusable for WeldSet's analytic ground truth.
- "There were three main motivations for preferring a simulation environment for evaluation before real-world tests. First of all, conducting extensive real-world experiments would require significant manual labor to set up the environment, collect data, and annotate object masks." (p. 51)
- "Secondly, the lack of annotated ground truth data, such as object positions, segmentation masks, and camera trajectories, requires additional equipment and effort for precise quantitative assessment of the developed algorithms. In contrast, simulation environments provide highly accurate ground-truth data with minimal effort." (p. 51)

**15. Experiment-justifies-design loop + empirical parameter disclosure.** Announce in Methodology that a choice rests on data; run the experiment in Results; state how each value was chosen.
- "We conducted an experimental study on the effect of off-plane rotation components on the accuracy of heading estimation to justify the choice of the adaptive information matrix in pose graph optimization." (p. 55)
- "The number of frames is selected by empirical testing as a value that yields stable node initialization without excessive computational time." (pp. 39–40)

**16. Failure narrative and scoped limitation** (observation → cause → fix → residual risk → remedy; then a limitation with its condition).
- "This failure mode highlights a limitation in handling precise heading estimation in constrained passages." (p. 68)
- "Possible solutions include incorporating local geometric cues or introducing a recovery behavior to escape oscillatory loops. Addressing these issues would further improve the robustness of the system in real-world scenarios." (p. 68)
- Related: the honest-caveat sentence "It is important to note that the reported execution time ... corresponds to runs on a GPU-accelerated device ..." (pp. 54–55).

**17. Scope policy for third-party tools (Bilaloğlu).** Say once what you will and will not explain about tools you did not write.
- "Therefore, rather than describing how the software tools work, which are documented in the source files, the subsections corresponding to the layers in Figure 4.1 explain how we leveraged the open-source software tools to solve challenges related to swarm robots." (p. 31)

**18. Specification → closed-form calculation → caveat → choice with margin (Bilaloğlu).**
- "This value corresponds to a theoretical maximum and is used only as an initial design calculation. Considering Kobot-W’s mass is 550[g], the maximum 251[g] is satisfactory." (p. 20)
- "We should note that these values are theoretical maximum and not possible to attain in reality due to imperfections like backlash in gearing and clearances of the drive elements." (p. 24)

**19. Hypothesis → minimal test → observed problem → fix → worst-case argument (Bilaloğlu).**
- "A setup comprising two ToF sensors was configured without a random delay and aligned with intersecting FoVs to test that hypothesis." (p. 49)
- "Let us assume the worst-case for a basic calculation, in which two robots are perfectly in sync, and their ambient reading phases always collide. If we have no random delay, these two robots will not detect each other under any circumstances." (p. 50)

**20. Reconcile calculation and measurement, and check against published results (Bilaloğlu).**
- "We believe the most prominent cause of this loss is the flexibility of the components." (p. 69)
- "The optimal value for the weight of proximal control is β = 1.3 by our experiments, and the results are in agreement with the results published by [1]." (p. 74)

### 4.12 Phrase bank (short reusable formulations, mostly Kurt)

| Function | Formulation (page) |
|---|---|
| Role statement | "This estimation serves as a backbone for constructing the pose graph of the environment within the scope of this thesis." (p. 16) |
| Choice statement | "In this thesis work, X coupled with Y is used to ..." (p. 16) |
| Interpretation of a math step | "This step can be interpreted as the projection of ... onto the space of valid ..." (p. 20) |
| Generalization disclaimer | "without loss of generality, ..." (p. 18) |
| Necessity | "Hence, it is still necessary to build a spatially consistent map that is not necessarily metrically accurate." (p. 22) |
| Initialization rationale | "Otherwise, solvers may get stuck at local minima or take too long to converge to a solution." (p. 24) |
| Weighting rationale | "Incorporating the information matrix ensures that more reliable measurements contribute more significantly to the resulting optimization." (p. 25) |
| Scope note | "Note that these landmarks are not predefined or specially structured objects but ..." (p. 27) |
| Observation from practice | "In practice, we observed that ..." (p. 40) |
| Evidence → adoption | "For this reason, we adopt tangent images in our implementation during downstream localization and navigation tasks." (p. 46) |
| Intuition device | "Conceptually, this can be imagined as ..." (p. 46); "To visualize, imagine ..." (p. 48) |
| Trade-off in results | "While this increases cluster purity, it also leads to missed associations and over-clustering." (p. 59) |
| Robustness claim from a sweep | "This suggests that the optimal transport method is more robust to parameter tuning." (p. 59) |
| Residual error attribution | "The main reason for the remaining faulty matches is inconsistencies in the predicted segmentation masks." (p. 60) |
| Generalization of setup | "This setup allows us to assess how well the method generalizes across environments with different spatial layouts and object distributions." (p. 61) |
| Reassurance after failure | "Nevertheless, the pose graph maintains a topologically accurate representation suitable for navigation purposes." (p. 64) |
| Dependency statement | "The accuracy and robustness of the proposed framework are strongly dependent on its core perception modules: ..." (p. 75) |
| Future-work opener | "Future work could explore adapting and training these data-driven models directly on ..." (pp. 75–76) |
| Dataset motivation | "This scarcity motivates the generation of new datasets tailored for ..." (p. 76) |
| Novelty hedge (Bilaloğlu) | "To the best of our knowledge, ... is the first of its kind." (p. 47) |
| Bridging two test levels (Bilaloğlu) | "Isolated tests of these systems provided insight into the performance. However, this is not always enough ..." (p. 74) |

### 4.13 What not to imitate

**Kurt**

- **Heading typos:** "CONCLUSION AND FEATURE WORK" (TOC and p. 75); "feature research" (p. 6).
- **Body typos:** "Adjusted Rank Index" (p. 58), "spruious" (p. 62), "With a loss of small loss of reliability" (p. 75), "approximately 10 x 12 floor space" with no unit (p. 64).
- **Wrong cross-reference:** "the epipolar constraint given in Equation 3.3" (p. 22) should be Eq. 3.2.
- **Missing references:** LightGlue, SIFT, ORB, BRISK, FLANN, RANSAC, the 8-point algorithm, Levenberg–Marquardt, Sinkhorn and A* have no citation.
- **Parameters left open:** the information matrix Ω is never specified (pp. 25, 57); r, τ, τ_assign, top-k and the RANSAC thresholds have no values; the "higher threshold" used after the spurious edge is unnamed (p. 62).
- **Claims without evidence:** real-world object navigation (abstract), perspective-camera navigation (contribution 1, p. 75) and the third map level (p. 5) are not demonstrated.
- **Missing chapter opener** in Chapter 5, and no summary table of experiments.
- **Mapping evaluated only qualitatively:** no trajectory or node error metric is reported, although ground truth was available.

**Bilaloğlu**

- **Weak introduction:** two pages, with no problem statement, objectives, contribution list or outline.
- **Contributions buried** in §2.3.1, and partly phrased as features ("use open-source or educationally licensed programs").
- **Inconsistencies between text and figures:** the text says the view angles are 0°, ±30° and ±45° (p. 57), but the figure legend shows ±60°; the text says "three different β values" (p. 65), but Table 5.3 lists four.
- **Typos:** "R2 = 098" (p. 72), "caclulate" (p. 20).
- **List-of-figures error:** Fig. 4.3 is missing from the List of Figures.
- **Equation numbering:** empty tags (4.5), (4.20), (4.25), and numeric results numbered as equations.
- **Unreported trial counts** for the gripper and flocking experiments; Kobot-F flocking is shown only as snapshots.

---

## 5. Structure lessons for a robotics-system thesis

### 5.1 Kurt's Methodology / Implementation / Results split

The outline sentence states the split itself: "Chapter 3 details the theoretical foundations, introduces the algorithms selected, and explains the novel approach we propose. Chapter 4 describes the design of the developed pipeline and provides insights into implementation choices." (p. 6)

| Content | Ch. 3 Methodology | Ch. 4 Implementation | Ch. 5 Results |
|---|---|---|---|
| Problem statement, notation, assumptions | Yes | – | – |
| Models, estimators, costs (equations) | All 32 numbered equations | None | None (metrics in words) |
| Choice of task family (e.g. panoptic vs instance) | Yes | – | – |
| Named pretrained models | Only as examples (SuperPoint) | Yes (OneFormer) | Yes (benchmarked) |
| Pipeline block diagram / flowchart | Concept figures only | Yes (Figs. 4.1, 4.3) | – |
| Threads, synchronization, data structures | – | Yes | – |
| Behaviours assembled from the algorithms (navigation loops) | – | Yes | Evaluated |
| Parameter values | None | Some (5 frames, 0.9, 5°) | Some (z = 4.4/4.8, 200 keypoints) |
| Experiments that justify design choices | Forward references | – | First (5.2–5.4) |
| System-level evaluation | – | – | Then (5.5–5.7) |
| Failure analysis | – | – | Yes |

**Principle:** *what and why* (Methodology) vs *how it runs* (Implementation) vs *how well, and where it breaks* (Results). The Methodology chapter is almost free of numbers and brand names; engineering choices live in Implementation.

### 5.2 Bilaloğlu's split by artefact layer, with a separate Discussion

- The chapters follow the artefact: Hardware (Chapter 3) and Software (Chapter 4). Their sections mirror the layer diagrams (Fig. 3.2: controller / sensory-motor / locomotion layers; Fig. 4.1: supervision / behaviour / drivers / firmware).
- Experiments are split into **component validation** (§5.2) and **scenario validation** (§5.3).
- **Discussion is a separate chapter** (6.1), organized by the same subsystem names, so each subsystem can be followed from design to validation to discussion. Results in Chapter 5 carry little interpretation.
- This is suitable for a platform thesis. For a perception-to-action pipeline, Kurt's split is clearer. One idea is worth borrowing: **mirror the subsystem headings across chapters** so that each subsystem can be traced.

### 5.3 Granularity of sections

| | Kurt | Bilaloğlu |
|---|---|---|
| Chapters (excluding references) | 6 | 6 + appendix |
| Numbered sections | 21 | 15 (+6 appendix) |
| Numbered subsections | 11 | 21 |
| Deeper headings | None | About 25 unnumbered run-in headings |
| Typical subsection length | 1–4 pages (§3.3.2 Mask Association is longest at about 4) | 1–12 pages (§3.1.2 runs 12 pages using run-in headings) |
| Results section length | 2–6 pages, no subsections | 1–4 pages per subsection |

**Lesson:** Kurt's granularity reads better. Give every methodological sub-problem its own subsection of 1–4 pages; no fourth heading level is needed.

### 5.4 Anatomy of "Proposed Methods and Models" (Kurt §1.2, pp. 3–5, about 560 words)

1. **One synthesis sentence** naming what the framework brings together: "In this thesis, the proposed framework brings together spherical imaging, precise pose estimation, semantic object understanding, and hierarchical mapping to support robust and semantically informed navigation in complex environments." (p. 3)
2. **"The system can be divided into four main components:"** (pp. 3–4), then **four bullets with bold run-in labels**, each 4–7 sentences long. Each bullet follows the data flow: sensor or input → processing step → algorithm *family* named at a high level ("learning-based approaches", "an 8-point algorithm", "RANSAC", "a panoptic segmentation model", "nonlinear optimization") → output → how it feeds the next component. Example: "Each spherical image is processed using a panoptic segmentation model, which provides pixel-level class labels along with instance masks, to add semantic information to the map." (p. 4)
3. **A closing paragraph on operating modes** (image-goal and object-goal navigation, p. 5).
4. **There are no citations, no numbers and no model names,** only algorithm families such as the 8-point algorithm, RANSAC and "a panoptic segmentation model". It is a narrative of the pipeline, not of the literature.

**Template for the welding-cell thesis.** "In this thesis, the proposed cell brings together [sensing], [CAD-based perception], [geometric seam reasoning], and [planning and contact-controlled execution] to ... The system can be divided into N main components:" Then write bullets such as:

- **Operator interaction:** hand gestures select parts and confirm actions.
- **Part perception and registration:** promptable segmentation followed by CAD-model-based 6-D registration, ICP refinement and multi-view refinement.
- **Seam and tack derivation:** seams computed from the registered CAD primitives; tack placement.
- **Motion and execution:** reachability analysis, collision-free planning, force-gated marking with a pen standing in for the torch.
- **WeldSet:** a synthetic dataset with analytic seam ground truth and a benchmark of literature methods and two proposed extractors.

Close with the operating modes or the experiment types (real cell vs. dataset).

### 5.5 Anatomy of "Contribution and Novelties"

- **Kurt §1.3 (p. 5, about 90 words).**
  - It opens with "Our key contributions are as follows:".
  - It has three bullets, each starting with a past-tense verb ("Designed and implemented", "Introduced a novel", "Designed a multi-level map").
  - Each bullet says what artefact was built, what distinguishes it, and the benefit ("reducing search space and improving scalability ...").
  - No evidence numbers are given. The bullets map onto the components of §1.2 and onto Chapters 3–5.
  - Weakness: some claims are never delivered later (perspective navigation without calibration, region segmentation).
- **Bilaloğlu §2.3.1 (pp. 6–7).** Contributions are derived *after* a comparison table of 30 prior robots (Table 2.1). This makes the novelty checkable. He ends with "To sum up, Kobot platform:" and four feature-style bullets. "First ..." claims are hedged with "to the best of our knowledge" elsewhere.
- **Lesson.**
  - Keep Kurt's placement (§1.3) and phrasing (verb-first, artefact + distinguishing feature + benefit, 3–5 bullets).
  - Add Bilaloğlu's evidence: one comparison table in the literature chapter (prior welding-perception cells, or seam-extraction datasets vs. WeldSet) that the contribution bullets can point to.
  - Only claim what Chapters 3–6 actually demonstrate.
  - Separate the two thesis parts explicitly: system contributions vs. dataset and benchmark contributions.

### 5.6 Results-chapter lessons

- Kurt's order works for a pipeline thesis:
  1. setup and environment;
  2. **component-selection experiments** (which matcher);
  3. **design-validation experiments** (information matrix);
  4. **evaluation of the novel component** (mask association);
  5. **system-level evaluation** (mapping, navigation);
  6. **real-world demonstration**.
- Within each section: aim → data and protocol → metric definitions → figure or table → interpretation → failure cases → remedy.
- Add what Kurt omits:
  - a chapter opener and a **summary table of all experiments** (purpose, data, n, metric);
  - quantitative metrics for system-level tests;
  - **baselines for the novel parts**;
  - trial counts and dispersion for real-robot tests (Bilaloğlu's 5 repetitions and min/mean/max bands are a minimum);
  - a parameter table per experiment (Bilaloğlu Tables 5.2–5.3).

### 5.7 Implications for the welding-cell + WeldSet thesis (suggested, to be adapted)

- **Chapter 1:** copy Kurt's four subsections, and open with a roadmap paragraph. In 1.1, follow Kurt's arc: broad trend → limitation of current practice, with one concrete example (Kurt's "robot vacuum in a kitchen" device, p. 2) → sensing and learning opportunity → "In response to ..., this thesis presents ..." (p. 3).
- **Chapter 2:** one section per field, each ending with a gap synthesis. Suggested fields: robotic welding automation and seam detection; segmentation and 6-D pose of CAD parts; refinement and multi-view; planning; seam datasets and benchmarks. Add a comparison table (Bilaloğlu style).
- **Chapter 3 Methodology:** one section per pipeline stage, using the pattern *requirement → family → formulation → assumptions → properties*. Derive the geometry you own (frames and calibration chain, ICP cost, multi-view fusion, seam derivation from primitives, tack spacing and placement, reachability criteria). Keep SAM2 and FoundationPose at task level here. Unlike Kurt, write **pseudocode** for the two proposed seam extractors and the tack-placement procedure, and give complexity notes.
- **Chapter 4 Implementation:** cell hardware, ROS 2 node graph (a Kurt-style block diagram), process and container layout, model cards for SAM2, FoundationPose and the gesture recognizer (Section 2.3 template), the gesture HMI and marking routine as flowcharts, and **one parameter table**.
- **WeldSet:** its own chapter (dataset design, analytic ground truth, splits, metrics, protocol, baselines, proposed extractors). Justify synthetic analytic ground truth with Pattern 14 ("three main motivations").
- **Results:** design-justification ablations first (e.g. with and without ICP, single vs multi-view), then component accuracy against ground truth, then system trials on the real cell with n and statistics, then the WeldSet benchmark. Failure cases follow Pattern 16.
- **Conclusion:** dependency statements on the pretrained models (Kurt p. 75) and the dataset-motivation sentence (Kurt p. 76).

---

## 6. Frank assessment of depth

Scale: 1 = essentially absent; 2 = standard or textbook material applied correctly; 3 = non-trivial adaptation or new formulation with some analysis; 4 = new formulation with analysis or strong controlled evaluation; 5 = publication-level novelty and rigor. Half points are used where a thesis sits between levels.

| Dimension | Kurt | Justification | Bilaloğlu | Justification |
|---|---|---|---|---|
| Theoretical | **2.5** | Correct, well-presented textbook geometry and PGO, sensibly adapted to the sphere (cheirality, scale, bearing-and-yaw measurement), plus defined but unanalysed new similarity and disparity measures; no proofs, and the adaptive Ω is never written down. | **1.5** | Datasheet-driven closed-form design calculations, textbook kinematics and PID discretization, and one informal probability argument; useful engineering, not theory. |
| Algorithmic | **3** | Several genuinely new but heuristic components (subspace/OT mask association with z-score graph clustering, object-bearing VPR, iterative-RANSAC object localization, tangent-image navigation), none formally analysed, none in pseudocode, and several parameters unspecified. | **2** | The swarm algorithms are re-implementations; the novelty lies in sensor-level mechanisms (random-delay de-synchronization, IQR-based robot detection, buffer filtering), which are clever but small. |
| Methodological | **4** | Clear decomposition with every component motivated by a requirement, alternatives weighed, and design choices validated by targeted experiments and honest failure analysis; held back by missing baselines and claims that are not followed through. | **3** | A disciplined spec → calculation → measurement → reconciliation loop on every subsystem; held back by a weak problem and contribution framing and by text–figure inconsistencies. |
| Experimental | **3** | Large, well-designed simulated ablations (10,368 pairs, 641 masks, 15 navigation runs), but real-world evidence is one handheld mapping sequence, there is no real navigation, mapping is evaluated qualitatively, and nothing is compared against existing VPR/SLAM. | **3** | Everything on real robots with Vicon ground truth, plus a 6-robot swarm with 5 repetitions for aggregation; small or unreported trial counts, little statistics, and aerial flocking shown only qualitatively. |
| Writing quality (extra) | **4** | Consistent voice, strong signposting and justification chains; typos in headings and a few broken cross-references. | **2.5** | Direct and concrete, but uneven grammar, some fragments, numeric results numbered as equations and a minimal introduction. |

**Positioning note.** Both theses passed with theory around 2 and experiments around 3; their strength lies in *argumentation* (Kurt) and *engineering traceability* (Bilaloğlu). A welding-cell + WeldSet thesis structured like Kurt would likely match him on method. It would exceed both if it adds four things:

- formalized proposed algorithms (pseudocode and stated assumptions or properties);
- quantitative real-robot trials with counts and dispersion;
- literature baselines on a benchmark with analytic ground truth (WeldSet gives a natural advantage that neither thesis has);
- complete citations and parameters for every pretrained model and tool.
