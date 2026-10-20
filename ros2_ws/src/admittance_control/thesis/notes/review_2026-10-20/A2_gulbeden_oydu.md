# A2: Gülbeden (2025) and Oydu (2025), structure, explanation style and depth

Analysis written for the UR5e perception-guided tack-welding cell / WeldSet MSc thesis (METU Robotics).

---

## 0. Scope and conventions

- **Sources.** Both PDFs were converted with `pdftotext -layout` and read in full: Gülbeden is 2,968 lines (about 22.3k words), Oydu is 4,199 lines (about 25.7k words). Diagram pages that carry structural meaning were rendered and looked at directly: Gülbeden Figs. 3.1, 3.2, 3.8, 3.11, 3.16, 3.19, 4.9/4.10 and 4.26; Oydu Figs. 2.3, 2.4, 2.5, 3.6, 3.9, 3.11, 3.12, 3.13 and 5.10.
- **Page numbers** are the printed Arabic page numbers of the main matter, with Roman numerals for front matter. PDF page = printed page + 20 for Gülbeden and printed page + 24 for Oydu.
- **Quotes** are verbatim and at most two sentences. Original typos are kept (e.g. Oydu's "/similarty").
- **Context that matters.** Both theses come from METU ROMER. Assoc. Prof. Dr. Ahmet Buğra Koku is co-supervisor of one and supervisor of the other, and Assoc. Prof. Dr. Mustafa Mert Ankaralı sat on both committees. Oydu's thesis is in the **Robotics** MSc programme, the same programme as the welding-cell thesis. Gülbeden's is in Electrical and Electronics Engineering. Both were accepted, so together they show the bar this committee has accepted for integration-heavy, real-robot MSc work.

---

## 1. Profile cards

### 1.1 Gülbeden: profile card

| Field | Value |
|---|---|
| Title | *Design and Implementation of a Whiteboard Drawing System for the TIAGo Mobile Manipulator* |
| Author | Anıl Gülbeden |
| Date | September 2025 (approval page dated 01.09.2025) |
| Degree / programme | MSc, Electrical and Electronics Engineering, METU |
| Supervisor / co-supervisor | Prof. Dr. Afşar Saranlı / Assoc. Prof. Dr. Ahmet Buğra Koku (ME) |
| Committee | Assoc. Prof. Dr. M. Mert Ankaralı, Prof. Dr. Afşar Saranlı, Assoc. Prof. Dr. İsmail Uyanık (Hacettepe) |
| Length | 105 PDF pages: xx front matter + 85 numbered pages ("85 pages" in the abstract) |
| Keywords | mobile manipulator, robotic drawing, robot motion planning |
| References | 34 |
| Abstract | About 120 words with no numerical results. It is purely descriptive ("We present experimental results on various parts of this problem.", p. v) |

**Table of contents (printed pages)**

```
ABSTRACT v · ÖZ vi · ACKNOWLEDGMENTS viii · TOC ix · LIST OF TABLES xii · LIST OF FIGURES xiii · ABBREVIATIONS xix
1 INTRODUCTION ......................................... 1
  1.1 Motivation and Problem Definition ................ 1
  1.2 Proposed Methods and Models ...................... 2
  1.3 Contributions and Novelties ...................... 2
  1.4 The Outline of the Thesis ........................ 3
2 LITERATURE REVIEW .................................... 5
3 SUBPROBLEMS IN MOBILE MANIPULATION ................... 7
  3.1 System Overview .................................. 7
  3.2 Board Detection .................................. 9
      3.2.1 Depth-based Board Pose Estimation .......... 11
      3.2.2 RGB-based Mask Estimation .................. 12
      3.2.3 Marker-based Board Detection and Pose Est. . 14
  3.3 Depth Camera Calibration ......................... 15
      3.3.1 Distortion Model and Correction Algorithm .. 17
      3.3.2 Calibration Methods ........................ 19
  3.4 Stable Pen Contact ............................... 23
      3.4.1 Overheating ................................ 25
  3.5 Motion Planning and Control ...................... 27
      3.5.1 Probabilistic Planners ..................... 28
      3.5.2 Pre-Recorded Motions ....................... 31
      3.5.3 Resolved-Rate Motion Control ............... 31
  3.6 Trajectory Generation ............................ 34
  3.7 Supervising Algorithm ............................ 35
  3.8 System Architecture .............................. 36
4 EXPERIMENTS .......................................... 39
  4.1 Experimental Setup ............................... 39
      4.1.1 TIAGo ...................................... 39
      4.1.2 The Laboratory ............................. 41
  4.2 Comparison of Depth Calibration Methods .......... 41
      4.2.1 Plane-fitting Method ....................... 43
      4.2.2 Extrusion Method ........................... 43
      4.2.3 Qualitative Comparison ..................... 46
      4.2.4 Comparison and Conclusion .................. 49
  4.3 Qualitative Evaluation of the Depth Correction Module 49
  4.4 Qualitative Comparison of Board Detection Methods  52
  4.5 Quantifying Pose Estimation Accuracy ............. 54
  4.6 Evaluation of Probabilistic Motion Planners ...... 62
  4.7 Comparison of Resolved-Rate Motion Controllers ... 64
      4.7.1 Formulation of Metrics ..................... 65
      4.7.2 Experiments and Results .................... 65
      4.7.3 Discussion ................................. 69
  4.8 Measurement of Arm Backlash ...................... 69
  4.9 Operating Algorithm Reliability .................. 72
5 CONCLUSIONS .......................................... 75
  5.1 Future Work ...................................... 76
REFERENCES ............................................. 77
A APPENDIX ............................................. 81
  A.1 Technical Drawings for the Gripper Attachment .... 81
  A.2 Additional Data from the Quantifying Pose Estimation Accuracy Experiment 81
```

**Pages per chapter**

| Part | Pages | Count | Share of body (pp. 1–76) |
|---|---|---|---|
| Ch. 1 Introduction | 1–4 | 4 | 5% |
| Ch. 2 Literature Review | 5–6 | 2 | 3% |
| Ch. 3 Subproblems (method) | 7–38 | 32 | 42% |
| Ch. 4 Experiments | 39–74 | 36 | 47% |
| Ch. 5 Conclusions | 75–76 | 2 | 3% |
| References | 77–80 | 4 | n/a |
| Appendix A | 81–85 | 5 | n/a |

Within Chapter 3: 3.1 overview about 2.5 pp; 3.2 board detection about 6; **3.3 depth calibration about 8**; **3.4 pen contact about 4.5**; 3.5 planning/control about 7; 3.6 trajectory generation about 1.5; 3.7 supervisor about 1.5; 3.8 architecture about 2.5.

**Counts**

| Item | Count | Notes |
|---|---|---|
| Numbered equations | 11 labels, 10 with content | (3.1)–(3.9) and (4.1)–(4.2). Label (3.9) is an empty line under the PID equation (p. 32) |
| Algorithm / pseudocode blocks | 1 | "Algorithm 1 Vectorized Rectification Algorithm" (p. 19) |
| Figures | 57 | 19 in Ch. 3, 33 in Ch. 4, 5 in App. By type, roughly: 24 photos, 8 RViz screenshots, 14 plots, 8 self-drawn diagrams (incl. 1 flowchart and 2 block diagrams), 3 technical drawings |
| Tables | 7 | 6 in Ch. 4 plus Table A.1 |

**Experiments (all on the real TIAGo, none in simulation)**

| § | What | Trials / conditions | Reference / ground truth | Metric | Headline result | Pages |
|---|---|---|---|---|---|---|
| 4.2 | Depth calibration: plane-fit vs. extrusion | Coefficients from 4 wall recordings (50 cm to 4 m). Evaluation at 9 distances (35–210 cm); repetitions not stated | Hand-measured depth of the top-centre pixel; PCA flatness | Accuracy error, flatness (3rd PCA explained-variance ratio) | Extrusion error about 1 cm at all distances vs. plane-fit error rising to about 9 cm (read from Fig. 4.10). Plane-fit is flatter. Extrusion chosen | 41–49 |
| 4.3 | Depth correction, qualitative | Board at 50, 104, 170, 232 cm | LiDAR line used as visual reference | RViz screenshots | Corrected cloud parallel to LiDAR at short range, with residual distortion at long range | 49–52 |
| 4.4 | Board detectors, qualitative | Lab scenes | n/a | Screenshots | RANSAC tolerates the arm in view. YOLO false positives fixed by adding negatives. Single-marker ArUco is skewed | 52–54 |
| 4.5 | Board-pose agreement between estimators | 2, 3, 4, 5 m × 50 repeats; single vs. double marker | **None**: the large-ArUco "baseline" was abandoned (data moved to App. A.2) | Geodesic angle ‖log(R₁R₂ᵀ)‖ (Eq. 4.1) | 0.055–0.076 rad at 2–3 m, 0.205 rad at 5 m; single marker up to 0.333 rad at 4 m | 54–62, 81 |
| 4.6 | RRT-Connect (MoveIt/OMPL) reliability | 100 attempts × 6 distances (unfurl) and 100 × 6 (unfurl + 5 cm square), 2 s timeout | n/a | Success rate | Unfurl 100% only from 52 to 80 cm. Drawing at 100% only at 61 cm. A 10 cm square never succeeded | 62–64 |
| 4.7 | RRMC (PID / Skipping PID) sweep | 15 configurations, apparently one square each | Time-interpolated commanded path (Eq. 4.2) | Weighted MSE "Score" + time | Lenient threshold with no sub-waypoints is best. The Skipping-PID effect is "negligible" | 64–69 |
| 4.8 | Arm backlash | Hand nudges with the marker on the board, axes 5 and 6; CW vs. CCW squares drawn twice | Ruler on the marks | Deflection length | 1.72 cm (axis 5), 1.34 cm (axis 6) | 69–74 |
| 4.9 | End-to-end operating algorithm | 15 consecutive runs from about 5 m away | n/a | Success (detect, move, draw a complete square) | **9/15**. All 6 failures attributed to the ROS navigation stack (5 localisation, 1 false obstacle) | 72 |

**Stated contributions (pp. 2–3).**
> "The main contributions of the thesis are in segmenting this example problem into sub-components that map well to the sub-problems studied by the robotics community, and in evaluating some popular solutions to these sub-problems in the context of a real-life problem." (p. 3)
> "Along the way, we develop relatively novel modifications to the algorithms we use to adapt them to our current context." (p. 3)

The modifications he names are the multiple-threshold "skipping" RRMC (§3.5.3) and the modified depth-camera calibration (§3.3). Contributions he makes but does not list as such: the compliant pen holder (§3.4, App. A.1) and the Python TIAGo wrapper library, which he made available online (§3.8).

**Contribution type.** Mostly **system integration** and **empirical comparison**, with light **algorithmic** modification. There is no theory contribution. He frames the work as integration research, citing Bajracharya et al.: "This highlights the importance of continued efforts at integration-heavy robotics work." (p. 3)

**Depth of theory.** Low. The formal content is (i) a dense, depth-dependent multiplicative correction with clipped linear interpolation (Eqs. 3.1–3.2, Alg. 1), (ii) pinhole back-projection / plane intersection (Eqs. 3.3–3.6), (iii) a PID-on-error RRMC written loosely (Eq. 3.7 reads `e(t) = J(q)^-1 v`, Eq. 3.8 is a PID, and (3.9) is empty), and (iv) two metric definitions (Eqs. 4.1–4.2). RANSAC, PnP, RRT-Connect and the ROS navigation stack get prose only.

**Third-party models and libraries.**
- YOLOv11-seg (Ultralytics), trained on his own 130 images.
- ArUco markers with an iterative Levenberg–Marquardt PnP solver (the library is not named).
- RANSAC (his own implementation) and least-squares plane fitting.
- PCA, used for the flatness metric.
- The ROS environment that ships with TIAGo, including the built-in navigation stack and localiser. The ROS version is not stated.
- MoveIt with the OMPL backend (RRT-Connect, Cartesian planning interface).
- Resolved-rate control (Whitney 1969).
- spatialmath-python (Corke & Haviland).
- IPython, Jupyter and RViz.
- The Teichman et al. "depth multiplier image", which he adapts.
- Hardware: Orbbec Astra S RGB-D, base laser scanner, Dynamixel gripper motors.

---

### 1.2 Oydu: profile card

| Field | Value |
|---|---|
| Title | *Development of an Indoor Topological Navigation Framework for Mobile Robots Using 360-Degree Cameras* |
| Author | Onur Oydu |
| Date | April 2025 (approval 07.04.2025) |
| Degree / programme | MSc, **Robotics**, METU |
| Supervisor / co-supervisor | Assoc. Prof. Dr. Ahmet Buğra Koku / Assist. Prof. Dr. Murat Durmaz (Geomatics, Hacettepe) |
| Committee | Assoc. Prof. Dr. M. Mert Ankaralı, Assoc. Prof. Dr. A. Buğra Koku, Assist. Prof. Dr. Akif Hacınecipoğlu (OSTIM Tech.) |
| Length | 143 PDF pages: xxiv front matter + 119 numbered pages ("119 pages") |
| Keywords | Topological Navigation, Equirectangular Image, Feature-Matching, 8-Point Algorithm, Metric-Free Navigation |
| References | 49 |
| Abstract | About 300 words, **quantified**: 72 trials, 0.32 m mean error, 88.9% success, 4.37 cm / < 2.5° triangulation error, 0.87 m workspace |

**Table of contents (printed pages)**

```
ABSTRACT v · ÖZ vii · ACKNOWLEDGMENTS x · TOC xi · LIST OF TABLES xv · LIST OF FIGURES xvi · ABBREVIATIONS xxiv
1 INTRODUCTION ......................................... 1
  1.1 Problem Definition 1 · 1.2 Motivation 2 · 1.3 Proposed Methods and Models 3
  1.4 Contributions and Novelties 4 · 1.5 The Outline of the Thesis 5
2 BACKGROUND AND RELATED WORK .......................... 7
  2.1 Indoor Navigation Paradigms ...................... 7
      2.1.1 Metric-Based Navigation .................... 8
            2.1.1.1 Wheel-Odometry and 2D LiDAR SLAM 8 · 2.1.1.2 Visual–Inertial and Stereo SLAM 8
            2.1.1.3 Dense RGB-D Mapping 9 · 2.1.1.4 Limitations of Metric Approaches 9
      2.1.2 Metric-Free Topological Navigation ......... 10
            2.1.2.1 Early Symbolic and Qualitative Methods 10 · 2.1.2.2 Vision-Based Topological Methods 10
            2.1.2.3 360-Degree and Panoramic Vision Approaches 11 · 2.1.2.4 Limitations of Topological Approaches 11
  2.2 360-degree Camera Model and Equirectangular Projection 12
  2.3 Feature Extraction and Matching ................. 13
      2.3.1 ALIKEd 15 · 2.3.2 LightGlue 15
  2.4 Relative Pose Estimation with the 8-Point Algorithm and RANSAC 16
  2.5 Image Similarity Measure and Visual Place Recognition 17
  2.6 Obstacle Avoidance 19 · 2.7 Robot Control Architecture 19 · 2.8 Research Gaps 20
3 METHODOLOGY .......................................... 21
  3.1 Proposed Algorithm 21 · 3.2 Problem Formulation 22 · 3.3 Camera Modeling and Back-Projection 23
  3.4 Heading Vector and Relative Orientation Estimation 24
      3.4.1 8-Point Algorithm 24 · 3.4.2 RANSAC 32
  3.5 Similarity Metric ................................ 34
  3.6 Topological Navigation and Mapping ............... 34
      3.6.1 Node Discovery and Uniqueness 35 · 3.6.2 Edge Construction 36
      3.6.3 Pose Embedding and Refinement 36 · 3.6.4 Route Planning and Online Localization 39
  3.7 3D Position Estimation from Triangulation ........ 41
  3.8 Implementation Details ........................... 44
      3.8.1 Obstacle Avoidance and Motion Control 44 · 3.8.2 ROS Node Architecture 47 · 3.8.3 State Machine Design 50
4 EXPERIMENTAL SETUP AND PROCEDURE ..................... 53
  4.1 Robot 53 · 4.2 Software 54 · 4.3 Hardware 55 · 4.4 Test Environment 55 · 4.5 Experimental Procedure 56
5 RESULTS AND DISCUSSION ............................... 59
  5.1 Results .......................................... 59
      5.1.1 Performance of the Topological Mapping Algorithm 60
      5.1.2 Performance of the Navigation Algorithm 61
            5.1.2.1 Reliability Analysis 62 · 5.1.2.2 Localization Error Analysis 65
      5.1.3 Performance of the Triangulation Algorithm 69
            5.1.3.1 Reliability Analysis 69 · 5.1.3.2 Pose Estimation Error Analysis 74
  5.2 Discussion ....................................... 80
6 CONCLUSION ........................................... 85
REFERENCES ............................................. 89
APPENDICES: A Additional Navigation Scenarios 95 · B Final Positions for Each Goal 103
            C Additional Triangulation Scenarios 108 · D Additional Pose Estimation Error Results 112
```

**Pages per chapter**

| Part | Pages | Count | Share of body (pp. 1–87) |
|---|---|---|---|
| Ch. 1 Introduction | 1–5 (+ blank 6) | 5 | 6% |
| Ch. 2 Background & Related Work | 7–20 | 14 | 16% |
| Ch. 3 Methodology | 21–51 (+ blank 52) | 31 | 36% |
| Ch. 4 Experimental Setup & Procedure | 53–57 (+ blank 58) | 5 | 6% |
| Ch. 5 Results & Discussion | 59–84 | 26 (Results 21, Discussion 5) | 30% |
| Ch. 6 Conclusion | 85–87 | 3 | 3.5% |
| References | 89–93 | 5 | n/a |
| Appendices A–D | 95–119 | 25 | n/a (mostly repeated scenario figures) |

Within Chapter 3: 3.1 about 0.7 pp; 3.2 about 1; 3.3 about 1.3; **3.4 (8-PA + RANSAC) about 9.5**; 3.5 about 0.8; 3.6 about 6.5; 3.7 about 3; **3.8 Implementation Details about 7.5** (3.8.1 about 3, 3.8.2 about 3, 3.8.3 about 2).

**Counts**

| Item | Count | Notes |
|---|---|---|
| Numbered equations | **49** | All in Chapter 3, (3.1)–(3.49). Each is usually followed by a "where:" bullet list of symbols |
| Algorithm / pseudocode blocks | **0** | Algorithms are shown as flowcharts (Figs. 3.6, 3.9), "Step 1/2/3" lists (§3.6.2) and mode lists (§3.8.3) |
| Figures | **104** | 53 in the body (Ch. 2: 6, Ch. 3: 13, Ch. 4: 3, Ch. 5: 31) and 51 in the appendices. Three are reproduced from the literature with "taken from [n]" (Figs. 2.3, 2.4, 3.5). Method figures: about 10 self-made explanatory diagrams/plots, 2 flowcharts, 1 state-machine diagram, 1 rqt_graph screenshot |
| Tables | 10 | Body: 3.1 node rates, 4.1 Python packages, 5.1–5.4 results. Appendix: D.1–D.4 |

**Experiments (all on the real TIAGo; ROS Noetic; base-station laptop with RTX 4070; two Ricoh Theta X)**

| # | What | Trials | Ground truth | Metrics | Headline result | Pages |
|---|---|---|---|---|---|---|
| 1 | Large mapping run (ROMER, 2–3 rooms plus corridors; hand-driven video, offline map) | 1 map | Floor sketch (qualitative overlay) | Visual agreement | Overall layout captured; drift grows with distance from the origin node | 56, 60–61, 80 |
| 2 | Bidirectional navigation reliability | 4 routes × forward/reverse = 8 | **None**. Success = goal node reached | Success, duration | 8/8. Durations 34–128 s | 56, 62–64, 80–81, App. A |
| 3 | Localisation accuracy (OptiTrack room, 9 fully connected nodes) | 9 × 8 = 72 start–goal pairs | **OptiTrack** | Success %, mean/std/median position error | 88.9%; mean 0.319 m, std 0.191 m, median 0.318 m; worst case < 0.83 m | 57, 65–68, 81–83, App. B |
| 4 | Triangulation during navigation | 2 routes × forward/reverse = 4 | None (hand-camera views) | Success | All succeeded (shown with images) | 57, 69–73, 83–84, App. C |
| 5 | Triangulation pose accuracy at 3 fixed bases (feature-rich lab, corridor, cluttered board) | 3 × 56 = 168 | Robot forward kinematics | Position and orientation error mean/std/median/max | 0.041 / 0.032 / 0.058 m; 2.13 / 2.49 / 1.97° | 57, 74–79, 84, App. D |

**Stated contributions (p. 4).** There are three bold-labelled bullets:
> "Metric-free Topological Navigation Framework: A complete topological navigation pipeline, implemented as modular ROS nodes, that relies solely on 360-degree vision for mapping, localization, path planning, and object-level approach, eliminating the need for metric sensors is proposed." (p. 4)
> "A Novel Topological Mapping Algorithm: An adaptation of the 8-point algorithm robustly estimates heading vectors and relative orientations, thereby enabling the construction of a scale-independent map without any metric measurements." (p. 4)

The third bullet is "Extensive Real-World Evaluation". A separate "Research Gaps" section (§2.8, p. 20) restates five gaps the framework addresses.

**Contribution type.** **System integration** together with **algorithm adaptation/assembly**: an adopted Robust 360-8PA, plus his own similarity metric, graph construction heuristics, least-squares pose embedding, head–hand triangulation and APF with memory. There is also a sizeable **empirical** part. The theory is adopted, not new.

**Depth of theory.** Moderate in volume, low in originality. About ten pages restate epipolar geometry (fundamental and essential matrix, SVD, rank-2 projection, four-fold decomposition, DLT triangulation, angular re-projection selection, Gold-Standard refinement, the RANSAC trial-count formula). This material is textbook (Hartley & Zisserman) plus Robust 360-8PA (Solarte et al.). It contains errors (see §5).

**Third-party models and libraries.**
- ALIKEd (pretrained keypoint and descriptor network, no fine-tuning).
- LightGlue (pretrained attention matcher).
- Robust 360-8PA, 8-point algorithm, RANSAC, DLT and the Gold Standard Method.
- Uniform-Cost Search and BFS.
- Artificial Potential Fields (Khatib).
- ROS Noetic (topics, tf) and MoveIt (arm motion).
- Python 3.10.15 with torch 2.0.1 (CUDA), numpy 1.26.4, kornia 0.7.3, scipy 1.14.1 and opencv-contrib-python 4.10.0.84 (Table 4.1).
- OptiTrack, used as ground truth.
- Code on GitHub (p. 54).

### 1.3 Side by side

| Dimension | Gülbeden | Oydu |
|---|---|---|
| Programme | EEE | Robotics (same as the welding-cell thesis) |
| Numbered pages / refs | 85 / 34 | 119 / 49 |
| Literature | 2-page chapter. Related work is mostly spread through Ch. 3 | 14-page "Background and Related Work" chapter: paradigms, components, research gaps |
| Method chapter | "Subproblems": one section per subsystem, each with its own mini-literature review | "Methodology": overview, then problem formulation, then per-module math, then implementation details |
| Setup | §4.1 inside the Experiments chapter (3 pp) | Separate Ch. 4 with robot, software versions, hardware, environment and protocols |
| Results | One section per module, discussion inline | Results by component × (reliability, accuracy), then a separate Discussion |
| Equations / algorithms | 10 / 1 algorithm block | 49 / 0 algorithm blocks |
| Flowcharts / state machines | 1 flowchart (operating algorithm) | 2 flowcharts + 1 state machine + ROS graph |
| ROS detail | Explains why most code is *not* split into ROS nodes; one ROS node | Full node list with topics, rates table, rqt graph, mode list |
| Voice | First-person plural ("we"; about 375 occurrences of we/our/us) | Impersonal passive (0 first-person pronouns in the body) |
| Ground truth | Mostly absent or hand-measured | OptiTrack (navigation), forward kinematics (triangulation) |
| Statistics | Mean/std, success rates | Mean/std/median/max, boxplots, histograms, per-target tables |

---

## 2. Gülbeden in depth: how the pen-drawing robot pipeline is described

### 2.1 Pipeline at a glance

| Stage | Where | What is described | Granularity |
|---|---|---|---|
| Hardware | §4.1.1 (pp. 39–41), §4.1.2 (p. 41); scattered in §3.3 (p. 15), §3.4.1 (pp. 25–26) | TIAGo: differential-drive base, 7-DoF arm, parallel gripper with Dynamixel motors, pan/tilt head, lift torso, head RGB-D (Orbbec Astra S, named only on p. 15), base laser, 3 rear ultrasonic sensors. Whiteboard 1.22 × 2.43 m, mounted 0.88 m above the ground | Prose only. No specification table: no camera resolution or frame rate, no arm repeatability, no onboard compute |
| Software architecture | §3.1 (pp. 7–9), §3.8 (pp. 36–38); Fig. 3.1 (block diagram, yellow = swappable submodules, red = system interfaces), Fig. 3.2 (board-detection signal flow) | "Competency" decomposition, central module, swappable submodules, Python wrapper around the TIAGo ROS interface, single process, class hierarchy with mixins, depth-correction ROS node | Conceptual and code-structure level. No topic names, message types, rates or ROS distribution |
| Calibration | §3.3 (pp. 15–22); evaluation §4.2–4.3 (pp. 41–52) | **Depth only**: a dense, multi-distance depth-multiplier calibration with the base LiDAR as reference, recorded by plane-fit or by extrusion. Camera–LiDAR transform taken from ROS TF. A head-pan offset tweak was tried. **No intrinsic, hand-eye or TCP calibration** | The most formal part of the thesis: equations 3.1–3.6, Algorithm 1, three explanatory diagrams |
| Perception | §3.2 (pp. 9–15); §4.4–4.5 (pp. 52–62) | Board detection: depth RANSAC plane; YOLOv11-seg mask feeding the depth plane fit; ArUco PnP. Confidence scores, modal selection, collision box | Method plus 1–3 key parameters each |
| Navigation | §3.1 (p. 9), §3.7 (pp. 35–36), §4.9 (p. 72) | The built-in ROS navigation stack | Black box (and it caused every end-to-end failure) |
| Planning / control | §3.5 (pp. 27–34); §4.6–4.7 (pp. 62–69) | Two actions. **Unfurl** to the drawing-ready pose: RRT-Connect via MoveIt, or a pre-recorded motion. **Draw**: MoveIt Cartesian planner (rejected) vs. RRMC with PID or Skipping PID through the velocity interface | Behavioural and comparative. Parameters: 2 s timeout, gains and thresholds in Table 4.5 |
| Trajectory generation | §3.6 (pp. 34–35) | Drawing API: a list of (x, y, pen-down). Transform into the detected board frame, optional linear sub-waypoints, a pre-draw pose computed from stored pen and attachment measurements | Interface level |
| Supervision | §3.7 (pp. 35–36), Fig. 3.19 | Operating algorithm: rotate until ArUco is found → face the board → YOLO pose → move to 1.5 m → depth pose → move to about 1 m → unfurl → draw | Nested flowchart whose blocks are named after code routines (`move_to_board`, `find_board`, `draw`), plus one paragraph |
| Pen contact / force | §3.4 (pp. 23–27), App. A.1 (pp. 81–84) | Spring-compliant pen holder; command to "push in"; force control turned into position control; motor-current F/T sensing rejected; holder redesigned after gripper overheating | Narrative, photos and technical drawings. **No stiffness, travel or preload numbers** |
| Error measurement | §4.2.3, 4.5, 4.6, 4.7.1, 4.8, 4.9 | PCA flatness, hand-measured depth, geodesic rotation difference between estimators, success rates, weighted MSE against a time-interpolated reference, ruler-measured marks, end-to-end success | Defined in prose plus Eqs. 4.1–4.2 |

### 2.2 Stage by stage, with the key quotes

**Framing and decomposition.** He chooses classical methods on purpose and gives the reasons: on-board compute, debuggability and iteration speed.
> "In our work, we choose to go with the "classical" approach as opposed to an end-to-end or machine learning-heavy solution." (p. 7)
> "Minimizing the use of learning-based approaches also promised an easier time debugging and developing the system, due to increased interpretability of program behavior, and faster iteration capabilities as fixing problems does not often depend on retraining a model with a new set of data." (p. 8)

He then defines **levels of competency**, the most reusable framing device in the thesis (p. 8): "1. The robot can draw a specified shape on a board in front of it starting from its home configuration. 2. Starting from a random location, the robot can locate the whiteboard in the room, move to it and draw on it." Each competency is then mapped to the modules it uses (pp. 8–9).

**Hardware.** It appears late, in Chapter 4, and is short. He frames the platform's problems candidly:
> "In particular, we had to correct the depth readings ourselves, had to work around motors overheating since restoring operation once any motor heated up too much was very time consuming, and could not shut down the robot programmatically." (p. 41)

**Software architecture (§3.8).** This section justifies a *departure* from ROS conventions: everything runs in one Python process for interactive development.
> "Defining new message types requires a recompile and cannot be done during runtime, hurting interactive development prospects." (p. 37)
> "Due to these reasons, we create an abstraction library that wraps the capabilities of the TIAGo robot's ROS interface and presents them as a Python class." (p. 37)

The wrapper subscribes to all topics and buffers them, converts transforms to spatialmath SE(3), and exposes high-level commands as class methods that can be called from the REPL (e.g. `draw_square` calls `draw` and `move_arm_to_board`). The class hierarchy is described concretely: "To give a concrete example, the cartesian motion planner is a subclass of the RRTConnect motion planner with the cartesian planning mixin." (p. 38). The one exception is the depth rectifier, which is a separate ROS node: "For the depth calibration module, we write a ROS node that takes in the uncorrected depth image and publishes the corrected depth image and pointcloud on new topics." (p. 38)

**Calibration (§3.3).** This is the best-written subsection and the most relevant to the welding cell. It follows a **symptom → diagnosis → literature → model → algorithm → procedures → evaluation** arc:
1. *Why it matters*: "Thus the validity and accuracy of the sensor data directly affects the reliability of the system." (p. 15)
2. *Symptom*: "This was spurred by the observation that the robot could write accurately in certain regions of its field of view but either failed to make contact or collided with the board in other regions." (p. 15). Fig. 3.7 shows depth-camera points against LiDAR points at about 5 m.
3. *Wrong first hypothesis, told honestly*: "This led to an initial diagnosis that the mismeasurements were being caused by an error in the head pan motor joint offset, and that the distortions seen at larger distances were a separate phenomenon unrelated to this offset." (p. 16)
4. *Literature taxonomy*: "These works used different distortion models to formulate the correction of errors in measurement, which can be broadly separated into the categories of parametric and dense." (p. 16), with 3-, 9-parameter and look-up-table examples.
5. *Constraint-driven design*: "Instead of unmounting the camera from the robot for calibration, which is impractical, we formulate our calibration process to take advantage of the additional sources of information provided by the robot platform." (p. 17). The extrinsic is not calibrated: "We can obtain the transformation between the camera and the rangefinder via ROS." (p. 18)
6. *Model*: "For our distortion model, we modify the depth multiplier image approach used in [26]. We use multiple depth multiplier images recorded at different distances and linearly interpolate between them to find the appropriate multiplier per pixel at runtime." (p. 18). Eq. 3.1 gives the corrected point and Eq. 3.2 the clipped piecewise-linear α(u,v,d). Fig. 3.8 is a two-axis mapping diagram.
7. *Algorithm 1* (p. 19): vectorised rectification in eight lines, plus the engineering note "we reimplement some ROS functions handling pointclouds, gaining a 100x speedup" (pp. 18–19).
8. *Two recording procedures* (§3.3.2), each with equations (3.3–3.6) and a top-down diagram (Figs. 3.9, 3.10) whose multi-sentence captions explain every marker. The trade-off is stated: "This second approach has the advantage of being more accurate compared to the first approach when the wall has slight deformation and is not perfectly flat." (p. 20)
9. *Evaluation* (§4.2–4.3): two defined metrics, flatness via PCA and accuracy against hand measurement (p. 46), over 9 distances. The decision follows: "We opt to use the coefficients calculated via the extrusion method in our project due to the high amounts of error in the plane-fitting method." (p. 49). The error is then related to how the depth is used: "Since depth readouts in the medium and long range are used only for moving toward the board, they do not need to be too accurate." (p. 50)

**Perception (§3.2).** Each estimator gets about one page: the sensor used, the algorithm by name and citation, 1–3 parameters, the confidence definition, the known failure mode, and a pointer to Ch. 4. The architecture is a *module with slots*: Mask Estimators and Pose Estimators, run together, each returning a confidence, with a mode switch: "The board detector instance used in the work subclasses the module to implement a discrete, modal behavior: The module takes in a mode switch, which determines which submodule should be used." (p. 10). He also states his own limits: fusion and temporal averaging were "an expedient solution" left out (p. 10). The module also builds the board's collision volume (p. 11; Fig. 4.16 shows an extra "lip" volume).

**Planning and control (§3.5).** He splits the motion into **two actions with different priorities**:
> "The reason for this separation is that the primary concerns during the two actions are different: For the first action of moving the arm into a drawing-ready configuration, the reliable generation of a collision-free, smooth path is paramount." (p. 27)

The text then works through alternatives as an engineering diary: OMPL planners time out or wander, and MoveIt configuration is "an opaque process" (p. 29). The Cartesian interface draws well but rarely plans: "However, obtaining a motion plan from the cartesian planning interface was highly unreliable, and would fail more often than it returned a working plan for our drawing planning problems." (p. 30). That leads to pre-recorded motions for unfurling (p. 31) and RRMC for drawing (pp. 31–34). Human-factors arguments recur: "For our project which aspires to have robots collaborate with humans, the robot should move in a natural, predictable way." (p. 29)

**Execution of drawing (§3.6, §3.7).** The drawing interface is defined compactly: "The drawing API takes in a list of 3D points. The first two dimensions of the point correspond to 2D positions on the board, and the last dimension is a boolean indicator that specifies whether the pen is touching the board or not." (p. 34). Points are transformed into the currently detected board frame, optionally linearly interpolated, and the approach pose uses "stored measurements of the pen and the gripper attachment" (p. 35). The supervisor routine is described in one paragraph and the Fig. 3.19 flowchart (p. 36).

**Pen contact and force (§3.4).** This is the section closest to the welding cell's force-gated pen marking. Its arc is **problem → why modelling is a bad idea → alternatives tried → mechanical solution → consequence for control → side-effects**:
> "Estimating the pose of a tool gripped by an end-effector is fraught with difficulties." (p. 23)
> "The factors listed above form a compelling case against a detailed mathematical modelling and estimation approach." (p. 23)
> "We prototyped a force/torque sensor using current readings from the motors of TIAGo's arm, but the current noise and chattering due to the arm controllers meant that attempting to extract reliable and stable force/torque readings from the motor currents would require substantial filtering work, extending outside the purview of this project." (p. 23)
> "The high degree of compliance in our gripper design compensates for errors in board pose estimation, pen tip deformations, and other unanticipated conditions. As such, we can convert the force-control problem of drawing on the whiteboard into a position-control problem." (p. 24)

The overheating subsection (§3.4.1) is a good model of an "unexpected practical failure" passage. It gives the cause (gripper motors holding the attachment), the quantitative symptom ("This posed a major problem as the motors would heat up after short drawing sessions to the order of 10-20 minutes.", p. 26), the attempted fix (current limit), the actual fix (redesign so the fingers sit on their mechanical stops), and the outcome ("used for multiple hours without resulting in motor overheating errors", p. 27). Contact compliance is carried into the metric as well: z-error is down-weighted "because our drawing operation is not sensitive to changes in z due to the mechanical compliance of the gripper" (p. 65).

**Error measurement.** The ways he measures error are varied and inexpensive:
- the geodesic angle ‖log R₁R₂ᵀ‖ (Eq. 4.1, p. 58);
- path error against a reference interpolated in time between consecutive waypoints, p₀,₁ = p₀ + (t₀,₁ − t₀,₀)/(t₁,₀ − t₀,₀)·(p₁ − p₀) (Eq. 4.2, p. 65);
- planner success rate over 100 attempts (pp. 62–64);
- deflection marks measured with a ruler (Table 4.6, p. 71);
- CW-versus-CCW squares to show hysteresis (Fig. 4.33);
- 15 end-to-end runs with failure attribution (p. 72).

He is candid about lacking ground truth: "A challenge we face in measuring the accuracy of the estimates is that we lack a ground-truth source of board pose data." (p. 54)

### 2.3 Chapter-by-chapter structure and granularity

- **Ch. 1 (4 pp).** Motivation (with an epigraph), "Proposed Methods and Models" in one paragraph, a contributions paragraph, then the outline. This is the METU template order, also used by Oydu.
- **Ch. 2 (2 pp).** A literature survey of drawing robots and mobile manipulation that is very thin, with no comparison table. It ends on the gap: "We note that drawing robots that can automatically detect and move toward their drawing surfaces are a relatively underexplored area within the existing body of work, which is one of the goals of our project." (p. 6)
- **Ch. 3 (32 pp), "Subproblems".** One section per subsystem. Each follows a stable micro-template:
  1. role in the system (and which competency needs it);
  2. related work, three to six citations inline;
  3. chosen method, with rejected alternatives and reasons;
  4. a few concrete parameters;
  5. observed problems and fixes;
  6. a forward pointer to the Ch. 4 section that evaluates it ("We present a more quantitative analysis of probabilistic planners in Section 4.6.", p. 31).

  Ch. 3 ends with how the system was built in software (§3.8).
- **Ch. 4 (36 pp), "Experiments".** It mirrors Ch. 3 almost one-to-one: calibration → depth correction → detection → pose accuracy → planners → controllers → hardware limit (backlash) → whole-system reliability. Each experiment section opens with its aim ("In this section, we aim to quantify the reliability of…", p. 62), describes the procedure, gives a table or figure, and discusses the result inline. There is no separate Discussion chapter.
- **Ch. 5 (2 pp).** A one-paragraph recap of what was built and learnt, and one paragraph of future work.

**Granularity in practice.** About 1 page per method and about 4 pages per experiment. Equations appear only where he made his own modification. Parameters appear only when they decide the outcome (RANSAC 3/100/5 mm; YOLO 130 images, 441 ms; planner 2 s / 100 attempts; PID gains and thresholds). The level of implementation detail is "class X inherits Y", not topics or messages. Photos are used heavily to document hardware and failures (gripper, overheating version, backlash marks, gear trains).

### 2.4 Weak spots to avoid

- **No ground truth** for the main perception output (board pose). The large-ArUco baseline was abandoned after "visual inspection" (p. 58).
- **Labelling inconsistencies.**
  - Table 4.1's caption says YOLO vs. ArUco, but its column header reads "YOLO vs. Depth", and its numbers are identical to the "Double ArUco vs. YOLO" column of Table 4.2 (pp. 59, 61).
  - §4.2.3 is titled "Qualitative Comparison" but is the quantitative one.
  - Fig. 4.10's x-axis says "(m)" while the values run 50–200, i.e. centimetres (p. 47).
  - Fig. 4.26/4.27 titles are raw Python reprs (`<class 'planner.ProbPID'>{'samples_per_interp': 5, …}`, p. 67).
- **Loose formalism.** Eq. 3.7 (`e(t) = J(q)^-1 v`, with v undefined) and an empty equation number (3.9) on p. 32.
- **Vague operating numbers.** The drawing distance is "approximately 1m" on p. 36 but "approximately 80cm" on p. 63.
- **Repetition counts often missing.** The calibration evaluation and the RRMC sweep look like single trials per configuration.
- **The compliant holder is not characterised** (no stiffness, travel or preload), although the whole contact strategy depends on it.

### 2.5 What the welding-cell thesis can borrow

| Gülbeden device | Where | Analogue in the welding cell |
|---|---|---|
| **Competency levels** | p. 8 | L1: mark planned tacks on a part at a registered pose. L2: SAM2 → FoundationPose → ICP → multi-view refine → seams from CAD primitives → tack placement → reachability → OMPL → force-gated marking. L3: operator-in-the-loop selection through MediaPipe gestures. Map each level to modules, as on pp. 8–9 |
| **Block diagram, swappable submodules vs. interfaces** (Fig. 3.1) | p. 9 | Colour the swappable parts differently from the fixed interfaces. Swappable: FP registration vs. FP tracking vs. ICP, single- vs. multi-view, planner choice. Interfaces: UR5e driver, D435i, gesture camera, force/torque |
| **Estimator module with confidence and modal selection** (Fig. 3.2) | p. 10 | Pose module that reports FoundationPose score / ICP fitness, with an accept/reject threshold (cf. "we wait for the confidence estimate to improve and reject single-marker estimates", p. 14) |
| **Depth-calibration arc** (symptom → wrong first diagnosis → taxonomy → model → Algorithm → two procedures → metrics by distance) | pp. 15–22, 41–52 | The hand-eye plus depth calibration chain. Start from the observed contact/mark error, give the transform chain with equations, add an algorithm block for the correction, compare recording procedures, and report error against distance |
| **"Stable Pen Contact" section** | pp. 23–27 | "Pen tool and force-gated contact": why a pen stands in for the torch; tool/TCP definition; alternatives (pure position, compliant holder, F/T gate, admittance); the chosen gate and its threshold; what compliance absorbs (perception error, surface error); side-effects (pen wear, preload). Put drawings of the holder in an appendix as in A.1 |
| **Two-action split with different priorities** | p. 27 | Free-space approach planned with OMPL (collision-free, reliable) vs. the contact/marking segment (accuracy, force gate). Explain why each uses a different planner/controller |
| **Drawing API** | p. 34 | A tack-marking command: an ordered list of tack poses in the part (CAD) frame plus a pen-down/approach/retract flag, transformed through the registered part pose and the TCP offset |
| **Metric with in-plane emphasis and time-interpolated reference** (Eq. 4.2) | p. 65 | Mark error measured in the plate plane (planned vs. marked tack), with the normal-direction error handled by the force gate and reported separately |
| **Planner success vs. distance tables** | pp. 63–64 | Planning/IK success rate against part pose or tack location, with N attempts and a timeout stated |
| **Hardware-limit experiment through marks** (backlash; CW vs. CCW squares) | pp. 69–74 | Characterise the systematic error of the marking chain (TCP error, pen run-out, holder compliance) by drawing a test pattern and measuring the marks |
| **End-to-end reliability with failure attribution** | p. 72 | N full runs (gesture → perception → planning → marking). Report success and attribute each failure to a module |
| **Engineering-diary candour** (overheating, opaque MoveIt configuration) | pp. 26–29 | Report practical failures with a number, the cause, the attempted fix, the final fix and the outcome |

Two things are **not** in Gülbeden and must come from elsewhere: a hand-eye or TCP calibration formalism (neither thesis has one; Oydu §3.7 at least writes frame chains explicitly), and quantitative ground truth. WeldSet's analytic ground truth and any measured mark positions are a clear methodological advantage over both theses.

---

## 3. How the two theses explain third-party models, external algorithms and their own algorithms

### 3.1 Gülbeden: third-party components

✔ = covered, ◐ = partial, ✘ = absent.

| Component | Purpose | Architecture | Training / data | I/O | Equations | Why chosen | Limitations | Integration |
|---|---|---|---|---|---|---|---|---|
| **YOLOv11-seg** (pp. 12–15, 56–57) | ✔ mask to filter the point cloud before plane fitting | ✘ "an iteration on the YOLO [21] series" only | ✔ 81 positive + 49 negative = 130 images, polygon labels (Fig. 3.4), count justified by Hermens et al. | ✔ "This model outputs a mask, a dense 2D array that indicates which pixels of the image correspond to the detected object." (p. 12) | ✘ | ◐ "YOLO is a light enough model that it can be run on-board the robot with acceptable response times at 441ms on average per inference." (p. 13) | ✔ false positives on white walls, fixed with negatives (p. 13) | ✔ tracking persistence on; detection confidence used as the estimate's confidence |
| **RANSAC** plane (pp. 11–12) | ✔ robust board plane | ✔ prose description of the iteration loop | n/a | ✔ | ✘ | ✔ against least squares: outliers from foreign objects (p. 11) | ◐ assumes the board dominates the view (p. 12) | ✔ 3 points, 100 iterations, 5 mm |
| **ArUco + iterative PnP (LM)** (p. 14) | ✔ board pose and mask | ◐ one sentence | n/a | ✔ | ✘ | ◐ implicit | ✔ single-marker skew (pp. 54, 61) | ✔ confidence = number of markers; single-marker estimates rejected for navigation |
| **MoveIt / OMPL / RRT-Connect** (pp. 28–31) | ✔ | ◐ backends, probabilistic nature, timeout/attempts, Cartesian interface | n/a | ◐ | ✘ | ◐ "informal evaluations" (p. 29) | ✔ timeouts, odd paths, near-misses, opaque configuration | ✔ collision volumes for board and pen; 2 s timeout |
| **RRMC** (Whitney) (pp. 31–34) | ✔ | ◐ | n/a | ◐ | ◐ Eqs. 3.7–3.8 (loose) | ✔ planners unreliable for drawing | ✔ "Our controller does not do any collision checking or attempt to avoid singularities." (p. 34), with NEO named as the fix | ✔ thresholds, gains (Table 4.5) |
| **ROS navigation stack** | ✔ | ✘ | n/a | ✘ | ✘ | ◐ pre-existing | ✔ only in results (5 localisation failures, p. 72) | ✘ |

His approach to an external model: **say what job it does in the pipeline, the minimum I/O, the data you gave it, its runtime on your hardware, how it failed in practice and what you changed.** He never explains how a network works internally. That is defensible for YOLO but would look thin for FoundationPose or SAM2.

### 3.2 Oydu: third-party components

| Component | Purpose | Architecture | Training | I/O | Equations | Why chosen | Limitations | Integration |
|---|---|---|---|---|---|---|---|---|
| **ALIKEd** (pp. 13–15) | ✔ | ✔ prose walk-through of the blocks + figure "taken from [1]": "The architecture of the ALIKEd starts with a CNN backbone, which processes the input image and outputs feature maps that contain texture and edge information of the scene (Feature Encoding)." (p. 15) | ✘ (pretrained, used as is) | ✔ keypoints + descriptors | ✘ | ✔ bold-labelled criteria (keypoint quality, efficiency) against SIFT/ORB/SuperPoint/DISK | ◐ later: dynamic objects, the robot's own arm (p. 74) | ✔ bottom half of head-camera image discarded (p. 14, Fig. 2.2); only 40% of the hand image used in triangulation tests (p. 74) |
| **LightGlue** (pp. 14–16) | ✔ | ✔ attention / cross-attention / confidence pruning + figure "taken from [2]" | ✘ | ✔ two keypoint/descriptor sets → matches with confidence | ✘ | ✔ with numbers: "LightGlue matches 1000 keypoints in 15 ms on a modern GPU, compared to 40 ms for SuperGlue and 80 ms+ for LoFTR/RoMa [2, 32]." (p. 14) | ◐ | ✔ match count ≥ 10 → `/matching_status` true (p. 47) |
| **8-point / Robust 360-8PA** (pp. 16–17 concept, 24–32 math) | ✔ heading vector = translation direction | ✔ full derivation (Eqs. 3.4–3.23) | n/a | ✔ unit vectors → R, t̂ | ✔ 20 equations | ✔ analytic, stable for wide FOV, popular for 360° | ✔ chirality fails for 360°, so angular re-projection is used instead (p. 30); degeneracy (pp. 80–81) | ✔ DLT selection, GSM refinement |
| **RANSAC** (pp. 17, 32–33) | ✔ | ✔ flowchart (Fig. 3.6) | n/a | ✔ | ✔ residual (3.24), trial count (3.25) | ✔ | ◐ | ✔ p_success 0.99, outlier ratio 0.5, m = 8 |
| **RMSE / SSIM / NetVLAD / BoQ / VAE** (pp. 17–19) | ✔ | ◐ one line each | ◐ VAE needs clustering loss | ✘ | ✘ | ✔ *rejected*: orientation dependence (Fig. 2.6), compute, training | n/a | n/a. Replaced by his own metric (§3.5) |
| **APF** (Khatib) (pp. 19, 44–46) | ✔ "basic safety layer" | ✔ | n/a | ✔ | ✔ Eqs. 3.43–3.49 | ✔ simple, explicitly out of scope | ✔ local minima, so the repulsive vector is kept for 5 s in `/odom` | ✔ k₁ = 0.25, k₂ = 0.5, θ_thr = 30° |
| **ROS / MoveIt** (pp. 19, 47–48, 54) | ✔ | ✔ nodes, topics, pub/sub defined | n/a | ✔ per node | n/a | ✔ "ROS Noetic version was chosen to ensure consistency with the hardware platform, matching the version already deployed on the robot." (p. 54) | ✘ | ✔ node list, rates, graph |

His approach to an external model has two layers. **Layer 1 (Background chapter):** a taxonomy of method families, then criteria-based selection with bold-labelled bullets and literature numbers, then a one-paragraph architecture walk-through with the original paper's figure ("taken from [n]"). **Layer 2 (Methodology chapter):** only the math needed to *use* it, then the integration rules and thresholds. Back-references link the two: "This section gives the mathematical formulation of the adopted Robust 360-8PA and RANSAC (Section 2.4)." (p. 24), and the reverse pointer "Detailed mathematical explanation of the 8-point algorithm and RANSAC are given in Section 3.4." (p. 17). He never covers training data, licences, failure statistics of the pretrained nets, or ablations.

**Recipe for SAM2, FoundationPose and MediaPipe, combining both:** Oydu's layer 1 (taxonomy, selection with numbers, architecture paragraph and figure), plus Gülbeden's operational facts (data or prompts used, runtime on your GPU or in Docker, the observed failure and your fix, how its confidence is used downstream), plus Oydu's layer 2 (the equations you actually use: the ICP objective, the pose-score threshold, the multi-view fusion).

### 3.3 Their own algorithms, step by step

| Own algorithm | Motivation | Formulation | Equations | Pseudocode / diagram | Parameters | Evaluation link |
|---|---|---|---|---|---|---|
| **G: multi-distance depth-multiplier calibration** (pp. 15–22) | Observed contact failures; Fig. 3.7 | Dense per-pixel α, clipped linear interpolation in d | 3.1–3.6 | Algorithm 1; Figs. 3.8–3.10 with explanatory captions | 4 recordings (50 cm–4 m); 2π/3 LiDAR cone | §4.2–4.3 |
| **G: Skipping PID** (pp. 32–34) | Pauses at sub-waypoints; rounded corners as the trade-off | Primary/secondary arrival thresholds | (3.7–3.8 for the base PID) | Figs. 3.16 vs. 3.17 (threshold circles) | Thresholds and gains in Table 4.5 | §4.7: effect "negligible" (p. 69) |
| **G: board-detection module** (pp. 9–15) | Several sensing modalities | Mask Estimators + Pose Estimators with confidence | none | Fig. 3.2 signal-flow block diagram | Confidence threshold (value not given) | §4.4–4.5 |
| **G: supervising algorithm** (pp. 35–36) | Demonstrate competency 2 | Sequenced routines | none | Fig. 3.19 nested flowchart | 1.5 m / about 1 m distances | §4.9 |
| **O: similarity metric** (p. 34) | RMSE/SSIM fail under rotation (Fig. 2.6) | Mean pairwise angle-difference between matched unit vectors; a yaw variant for goal alignment | 3.26–3.27 | none | 5° (mapping), 4° (navigation), 15° (goal) | §5.1.2, Discussion p. 83 |
| **O: topological map construction** (pp. 34–40) | Scale-free map from video | G = (V, E); node uniqueness; edge rules; BFS chaining; least squares; pruning | 3.28–3.33 | "Step 1/2/3" list (p. 36); Fig. 3.9 flowchart; Fig. 3.8 geometry | 60 matches, ≥ 3 edges, 0.1 units ("selected iteratively", p. 39) | §5.1.1 (qualitative) |
| **O: head–hand 360° triangulation** (pp. 41–44) | Vision-only object approach | Gravity alignment of camera 2, closest point between two rays, transform to base | 3.34–3.42 | Fig. 3.10 | none | §5.1.3 |
| **O: APF + motion command** (pp. 44–46) | Safety | Repulsive vector magnitude, heading + repulsion, (v, ω) law | 3.43–3.49 | Fig. 3.11 frames | k₁, k₂, θ_thr, 5 s memory | Implicit |

The two styles differ clearly. **Gülbeden** motivates *from a failure he observed* and formalises only his change to an existing method. **Oydu** motivates *from a requirement* ("A precise and robust approach is needed to calculate the heading vector in a real-world application.", p. 17) and formalises everything. Each step gets an equation with a symbol list, and the thresholds are given as "taken as …" values.

### 3.4 Presentation devices compared

| Device | Gülbeden | Oydu |
|---|---|---|
| `algorithm` environment | 1 (Algorithm 1, p. 19) | 0 |
| Numbered step lists | Competency list (p. 8) | Edge construction "Step 1–3" (p. 36); Mode 0–5 list (p. 50); experimental protocols 1–5 (pp. 56–57) |
| Flowcharts | Fig. 3.19 operating algorithm (nested groups named after code routines) | Fig. 3.6 RANSAC; Fig. 3.9 map construction (decision diamonds, loops) |
| State machine | none (the flowchart plays this role) | Fig. 3.13: six states, `nav_mode = 0…5`, labelled transitions ("Goal Image Set", "Localization Done", "Orientation Aligned", "Object Located", "External Flag Set") |
| Block diagrams | Fig. 3.1 system; Fig. 3.2 module signal flow | none at system level (the pipeline is given as a paragraph on p. 21 and bullets on p. 22) |
| Geometry / explanatory diagrams | Figs. 3.8–3.10, 3.16–3.17 | Figs. 2.5, 3.1, 3.2, 3.3/3.4, 3.8, 3.10, 3.11 |
| Symbol glossary after equations | rarely ("where α(u,v,d) is…", p. 18) | always ("where:" bullet lists) |
| Problem-to-section map | prose outline (pp. 3–4) | bullets ending "; see Section 3.x" (p. 22) |

### 3.5 How ROS nodes and implementation details are described

**Oydu §3.8 (pp. 44–51)** is the template the welding-cell thesis should follow.

1. **Intro roadmap paragraph**: "This section details the practical software components that transform the previously described algorithms into a working robotic system." (p. 44), followed by a list of the three subsections.
2. **§3.8.1, control layer with equations.** APF and the (v, ω) law (Eqs. 3.43–3.49), parameters in a "where:" list, frames named (`/odom`, `/base_link`, `/tf`), and a frames figure (Fig. 3.11).
3. **§3.8.2, node list.** One bold bullet per node. Each gives a **functional name and executable name** ("Heading Vector Calculation (heading_publisher)"), its role, the topics it subscribes to and publishes, decision thresholds, links back to the equations, and how it couples with the state machine. Example: "Also, it publishes to /matching_status topic. The number of matches determines the success of the feature matching process; matching status is true if there are more than or equal to 10 matches." (p. 47). The list is followed by a **rates paragraph with reasons** ("Rates of image publisher nodes were set at 15Hz because cameras send images in 15 frames per second. Rates of other nodes are set at 10Hz, which is the approximate time for completing feature extraction and matching processes.", p. 48), **Table 3.1** (node → Hz), and **Fig. 3.12**, a full-page rqt_graph screenshot. The screenshot is rotated and nearly unreadable in print. A hand-made Graphviz or TikZ diagram would serve better.
4. **§3.8.3, state machine.** "Mode k: Name, behaviour, transition condition", then the diagram. Example: "Mode 1: Localization, find the node with the highest number of matches and go to this node until /similarty becomes true." (p. 50). There is a small inconsistency: "has 5 modes" while six are listed (0–5).
5. **The Ch. 4 complement.** The software versions table (Table 4.1), the GitHub link, and a hardware topology note: "This laptop is used as a base station; the robot's software receives movement commands from this base station." (p. 55)

**Gülbeden §3.8 (pp. 36–38)** is the counter-example and a useful complement. Instead of a node inventory it gives the *reasons for the architecture* (iteration speed, REPL, hot reload) and the class hierarchy, with one node justified by reuse and speed: "We implement this as a ROS node since it's likely to be individually useful in other endeavours and running the republisher as a separate process is advantageous in terms of speed." (p. 38)

**Template for the ROS 2 cell (synthesis).**
- §X.1 Architecture rationale (Gülbeden-style): why these node boundaries, where Docker sits (FoundationPose), where the GPU sits, ROS_DOMAIN isolation.
- §X.2 Node inventory (Oydu-style): bullets with *functional name (executable)*, role, subscribes/publishes, services/actions, thresholds with equation links. Add a node/topic table with rates and QoS.
- §X.3 A legible hand-drawn computation graph, not a raw rqt_graph.
- §X.4 A cell state machine (idle → gesture selection → perception → registration → seam/tack planning → reachability check → approach → force-gated marking → retract → verify) with labelled transitions and failure/abort edges. Neither thesis draws failure edges, so adding them would be an improvement.
- Ch. 4: versions table (ROS 2 distribution, Python, torch, Open3D, MoveIt 2/OMPL, SAM2/FoundationPose commits, ur_robot_driver, realsense-ros), hardware and compute topology.

---

## 4. Language and style

### 4.1 Voice

- **Gülbeden** uses first-person plural throughout (about 375 we/our/us), active voice and a conversational, diary-like tone: "We discovered that…", "We speculate that…", "We opt to…" (5×), "We note…" (11×). Mild literary touches appear too: a Brian Eno epigraph (p. 1), and the whiteboard task called "a drosophila" (p. 3).
- **Oydu** uses an impersonal passive with no first-person pronouns in the body. Connectives carry the logic: "Therefore," (24×), "Also," (14×), "In other words," (7×), "As explained in Section…" (5×), "taken as" (13×). The tone is formal and somewhat template-like, with a few grammar slips ("the problem is breakdown into", p. 21; "is breakdown into three sections", p. 59).

For a Robotics MSc that wants to read like Oydu, use **passive or impersonal voice for methods and results**. Allow Gülbeden-style "we" sparingly for design decisions and lessons learnt, if the supervisor accepts it. Keep one voice per chapter.

### 4.2 How sections open

- **Oydu opens chapters with a roadmap** naming every section: "This chapter first defines the shortcomings of metric-based indoor navigation, then motivates a metric-free, vision-only alternative, outlines the proposed topological framework, and previews the thesis organization." (p. 1). Similar openings appear on pp. 7, 21, 44 and 53. **Sections open with a definition** ("Feature extraction is defined as the process of identifying unique elements…", p. 13; "A similarity metric numerically measures how close or alike two images or feature sets are.", p. 17) **or a back-reference** ("As explained in Section 2.5, an orientation-independent similarity metric is required.", p. 34).
- **Gülbeden opens sections with the module's role** ("The trajectory generation module is responsible for converting the drawing commands supplied through the drawing interface into a path that the robot arm can follow.", p. 34), **a general challenge** ("Detection and pose estimation of objects is a common challenge in manipulation and autonomous driving research.", p. 9) **or a dependency** ("Various parts of the system depend on readouts from this sensor…", p. 15). His Ch. 3 opening (p. 7) also separates method from comparison: "Comparisons of the proposed solutions to these subproblems according to performance metrics will be given in Chapter 4."

### 4.3 How decisions are justified

- **Gülbeden** gives empirical, practical and human-factors reasons, often after the fact.
  - Practical: on-board compute and debuggability (pp. 7–8); "an opaque process" (p. 29).
  - Experience: "informal evaluations determined that the RRT-Connect [31] implementation in OMPL provided reasonably predictable point-to-point trajectories" (p. 29).
  - Safety and social acceptability: "natural, predictable", "less imposing, elbow-down resting pose" (pp. 29, 63).
  - Evidence from his own experiment: "We note that a 23% chance of failure is highly detrimental to the successful operation of the system." (p. 64)
- **Oydu** gives requirement-driven reasons and reasons by elimination with literature numbers: requirement, then candidates, then criteria (bold labels), then numbers, then "Therefore, X is selected/adopted". Examples: pp. 13–14 (ALIKEd/LightGlue), p. 17 (8-PA), pp. 18–19 ("NetVLAD and BoQ are unsuitable for real-time applications due to their high computational cost."), p. 54 (ROS Noetic for platform consistency).

### 4.4 Transitions

- **Gülbeden** links sections by *problem → consequence → next attempt*: "The unreliability of the MoveIt planners stemming from their probabilistic nature motivated us to create a module that would execute pre-planned motions…" (p. 31); "This precludes the use of our pre-recorded motion modules for this action." (p. 31). Each method section ends with a pointer into Ch. 4.
- **Oydu** links by explicit section cross-references in both directions (Background ↔ Methodology ↔ Results) and "Therefore," chains. Results paragraphs cite equations by number ("Equation 3.27 is used to compute the similarity measure (15-degree threshold is taken)", p. 69).

### 4.5 Figure captions

- **Gülbeden**:
  - **Long explanatory captions for his own diagrams.** Figs. 3.8, 3.9, 3.10, 3.16 and 3.17 run four to six sentences and define each marker and colour.
  - **Legend and attention cues.** "Yellow indicates swappable submodules and red indicates system interfaces." (Fig. 3.1); "uncorrected (green) vs corrected (blue)" (Figs. 4.12–4.14); "Note the extra collision volume for the board lip." (Fig. 4.16).
  - **Very short captions for photos** ("LiDAR readout", Fig. 4.4).
- **Oydu**:
  - **Mostly one-line captions** ("Flowchart of RANSAC algorithm.", "State machine of the implemented algorithm.").
  - **Attribution** for reproduced figures ("taken from [1]").
  - **Layout cues** ("View of the current scene (left) and target scene (right) when robot starts navigation…").
  - **Legend-in-caption** for result plots, e.g. Fig. 5.29: "Ground truth positions (black boxes) and estimated positions (blue circles)… All positions are given according to robot's base frame (red circle)." Fig. 5.31 adds the task tolerance to the plot: a dashed 0.87 m reach circle.
  - **The in-text reading guide** does the explanatory work his captions leave out (pattern 12 below).

### 4.6 Reusable writing patterns (15)

Each pattern gives the example quote, then a template for the welding thesis in *italics*.

1. **Chapter roadmap opener (Oydu).** "This chapter surveys the foundational concepts and related work that underpin the proposed metric-free navigation framework." (p. 7). *"This chapter details the perception-to-marking pipeline. Section 3.1 summarises…, Section 3.2 formulates…, and Section 3.8 describes the ROS 2 implementation."*
2. **Problem-to-section map (Oydu).** "The problem formulation is divided into five sections, each covering a key aspect of the proposed algorithm." (p. 22), followed by bullets ending "; see Section 3.x". *One bullet per sub-problem (registration, seam extraction, tack placement, reachability, planning, contact), each with its inputs and outputs and the section number.*
3. **Competency levels (Gülbeden).** "1. The robot can draw a specified shape on a board in front of it starting from its home configuration." (p. 8). *"Level 1: the cell marks planned tacks on a part at a registered pose. Level 2: …"*
4. **Concept now, math later, with pointers both ways (Oydu).** "Detailed mathematical explanation of the 8-point algorithm and RANSAC are given in Section 3.4." (p. 17) ↔ "This section gives the mathematical formulation of the adopted Robust 360-8PA and RANSAC (Section 2.4)." (p. 24).
5. **Selection by named alternatives and numbers (Oydu).** "In this work, a robust feature extractor and matcher pair is required to utilize 8-point algorithm for precise relative pose estimation between images and for correct topological map construction. ALIKEd and LightGlue are selected over other algorithms due to their advantages." (p. 13), followed by bold-labelled criteria with ms figures (p. 14). *Use the same shape for FoundationPose vs. alternatives and SAM2 vs. alternatives.*
6. **Symptom-first motivation (Gülbeden).** "This was spurred by the observation that the robot could write accurately in certain regions of its field of view but either failed to make contact or collided with the board in other regions." (p. 15). *Open the calibration section with the observed mark or contact error.*
7. **Honest wrong-first-hypothesis (Gülbeden).** "This led to an initial diagnosis that the mismeasurements were being caused by an error in the head pan motor joint offset…" (p. 16). It builds credibility and documents the diagnostic process.
8. **Equation, then "where:" list, then "taken as" (Oydu).** Eq. (3.44)–(3.49) followed by "k₁: Linear velocity constant, taken as 0.25." (p. 46). *Give every threshold (force gate, ICP fitness, FP score) a symbol, a value and a reason.*
9. **Parameter disclosure with its reason (both).** "Our implementation of the algorithm samples 3 points per iteration, which permits a unique solution to the plane equation without the need for optimization algorithms. We repeat for 100 iterations and use a distance threshold of 5mm to determine inliers." (Gülbeden p. 12). "Rates of image publisher nodes were set at 15Hz because cameras send images in 15 frames per second." (Oydu p. 48).
10. **Trade-off with a visible consequence (Gülbeden).** "An option is to increase the arrival threshold for the waypoints, but this comes with the concomitant decrease in drawing precision. A square drawn with lenient thresholds will have rounded corners." (p. 33). *"A lower force gate reduces pen wear but increases missed marks on tilted plates."*
11. **Scope fence (both).** "Even though obstacle avoidance is not in the scope of this thesis, a basic safety layer is required for experiments." (Oydu p. 19). "…would require substantial filtering work, extending outside the purview of this project." (Gülbeden p. 23). *Use for actual welding, torch dynamics, and Kalman fusion.*
12. **Reading guide for plots (Oydu).** "Each box covers the interquartile range with the median marked inside, while the whiskers extend to the most extreme values that are not outliers, which are shown as individual points." (p. 65).
13. **Relate the error to the task's tolerance (both).** "Since the TIAGo arm's horizontal workspace radius is 0.87 m, every recorded stopping pose lies at least 4 cm inside the arm's maximum reach. Therefore, the manipulator can perform fine servoing without needing the mobile base to inch forward." (Oydu p. 82). "Since depth readouts in the medium and long range are used only for moving toward the board, they do not need to be too accurate." (Gülbeden p. 50). *Relate the roughly 2.5 mm registration error to the tack-position tolerance and the pen-tip width.*
14. **When an experiment changed the design (Gülbeden).** "The findings obtained regarding the inaccuracy of the single-marker case in this experiment resulted in the addition of mechanisms to ensure that estimates from single-marker detections would not be used for navigation of the robot in the operating algorithm as described in Section 3.7." (p. 61).
15. **Missing ground truth and deliberate stress tests (both).** "There is no ground truth for comparing results. Therefore, success criteria are defined as reaching the goal node (success rate)." (Oydu p. 62). "This case is selected intentionally to show how unsuccessful matching affects the results." (Oydu p. 75). "We attribute the 6 failures to errors in the built-in navigation stack." (Gülbeden p. 72). *State the success criterion before the numbers, include a deliberate hard case (reflective plate, occlusion), and attribute every failure to a module.*

Two more structural habits are worth copying from Oydu: **one representative case in the body, the rest in an appendix** ("The figures for the first scenario appear here; the corresponding illustrations for scenarios 2–4 are provided in Appendix A.", p. 62), and **a quantified abstract**.

---

## 5. Frank depth assessment

Scale: 1 = minimal, 3 = solid MSc norm, 5 = strong publishable depth.

| Dimension | Gülbeden | Justification | Oydu | Justification |
|---|---|---|---|---|
| **Theoretical** | **1** | The only formal content is a clipped piecewise-linear correction, plane/ray intersection, a garbled PID-RRMC equation and two metric definitions. There is no derivation, error propagation or analysis. | **2** | Ten carefully typeset pages (49 equations) restate Hartley–Zisserman and Robust 360-8PA with errors and add no new analysis (e.g. no degeneracy or error-propagation study). |
| **Algorithmic** | **2** | Two modest, well-motivated modifications: the interpolated multi-distance depth multipliers with two LiDAR-referenced recording schemes, and a dual-threshold PID that his own sweep finds "negligible". Plus an extensible estimator architecture. | **3** | A coherent, fully specified original pipeline (rotation-invariant similarity, edge heuristics, least-squares scale-free embedding, gravity-aligned head–hand triangulation, APF with memory), but each component is heuristic and hand-tuned ("selected iteratively", p. 39). |
| **Methodological** | **3** | Exemplary decomposition, real-system comparison of alternatives and decisions traced to experiments. Weak where it matters: no board-pose ground truth, unclear repetitions, an ad hoc weighted score. | **3** | Clear formulation, explicit protocols and a separate setup/results/discussion, with OptiTrack and FK references. But no baseline against the direct predecessors [5, 6] or a metric stack, no ablations (feature masking, thresholds), and the map is only judged visually. |
| **Experimental** | **3** | Broad real-robot coverage (nine experiments; 100-trial planner tests; 50-repeat pose comparisons; a 15-configuration sweep; 15 end-to-end runs), but much of it is qualitative or single-condition and end-to-end success is 60%. | **3.5** | 72 OptiTrack-referenced trials and 168 triangulation estimates in three deliberately varied scenes with mean/std/median/max, boxplots and histograms. But the reliability tests are small (8 + 4 runs), the triangulation reference is the robot's own FK, and some interpretations contradict the numbers. |

**Errors and inconsistencies to avoid repeating (Oydu).**
- "Similar to the fundamental matrix, the essential matrix is rank 2 and, by definition, has 7 degrees of freedom." (p. 27). The essential matrix has 5 DoF.
- Eq. 3.13 writes "t1 = u3, t1 = −u3" (the second should be t₂).
- Eq. 3.36 defines θx but composes Ry(θy)Rz(θz).
- "Inspection of the boxplot errors, Figure 5.11, …" (p. 82): Fig. 5.11 is a histogram, and the sentence is garbled.
- "…only about 5 cm away from the robot" (p. 82): confusing reasoning about the 54 cm base.
- The Experiment 2 discussion calls 0.032 m "a slight decrease", then says features were excluded, "leading to higher errors than Experiment 2" (p. 84).
- "has 5 modes" while listing six (p. 50).
- Orientation error is taken as the Euclidean norm of Euler angles (p. 74). A geodesic metric, as in Gülbeden's Eq. 4.1, is the sounder choice.

**Errors and inconsistencies to avoid repeating (Gülbeden).** See §2.4: Table 4.1 header/caption mismatch, cm/m axis label, raw Python plot titles, the empty equation number, the 1 m vs. 80 cm drawing distance, and the uncharacterised compliance.

**Overall verdict.**
- **Gülbeden** is a candid, readable *engineering-integration* thesis. Its value lies in the decomposition, the way it narrates practical problems, and its pen/contact and calibration sections. It is not a model for formal rigour or evaluation design.
- **Oydu** is a well-*structured* thesis in the Robotics-programme house style: roadmaps, a formulation map, equations with symbol lists, implementation details with nodes and a state machine, a separate setup chapter, statistics. Its formal sections are partly decorative and contain errors, and it lacks baselines.

The welding-cell thesis can take **Oydu's skeleton and formal presentation** and **Gülbeden's honesty and pen-tool narrative**, and exceed both on **ground truth** (WeldSet's analytic GT, directly measured mark positions, calibration residuals) and on **baselines/ablations** (e.g. FoundationPose with and without ICP and multi-view refinement, the force gate on and off, planner choices).

---

## 6. Practical takeaways (condensed)

1. **Chapter skeleton (Oydu, with Gülbeden's per-subsystem template inside the methodology):** Introduction → Background & Related Work (paradigms, then third-party components: SAM2, FoundationPose, ICP, MediaPipe, OMPL, then research gaps) → Methodology (overview, problem formulation map, one section per subsystem, calibration chain, pen tool and force-gated contact, implementation details with ROS 2 nodes and a state machine) → Experimental Setup & Procedure (hardware, software versions table, numbered protocols) → Results (by subsystem × reliability/accuracy) → Discussion → Conclusion. WeldSet can be its own methodology and results block, or its own chapter.
2. **For each pretrained model, cover:** role in the pipeline; I/O; the alternatives and why this one (with numbers); an architecture paragraph plus the source figure with attribution; prompts or data used; runtime on your hardware or Docker; how its confidence is used; failure modes you observed and your fix; and the equations you actually rely on downstream.
3. **For each of your own algorithms, cover:** the observed symptom or requirement; the formulation with symbols; the equations, each followed by a "where:" list; an algorithm block for any non-trivial loop (seam extraction, tack placement, multi-view refinement). Neither example thesis uses enough of these, and a Robotics committee reads them easily. Then give the parameters with reasons, a figure, and a forward pointer to the experiment that evaluates it.
4. **Pen tool section modelled on Gülbeden §3.4, but quantified:** holder stiffness and travel, force-gate threshold, contact detection latency, and mark error measured against the planned tack in the plate plane.
5. **Evaluation modelled on Oydu §4.5/§5, with what both lack:** a defined ground truth, a success criterion stated before the results, per-condition tables (mean/std/median/max), a deliberate hard case, baselines/ablations, and end-to-end runs with failure attribution (Gülbeden §4.9).
