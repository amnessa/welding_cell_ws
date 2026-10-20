# Three METU theses profiled: Aguş (2023), Uzel (2023), Akbaba (2025)

Source PDFs, in `/workspaces/welding_cell_ws/ros2_ws/src/admittance_control/thesis/examples/`:
`EGE UGUR AGUSthesis.pdf`, `DENGE UZELthesis.pdf`, `MustafaAkbaba_Tez.pdf`.
I extracted the text with `pdftotext -layout` into this folder (`agus.txt`, `uzel.txt`, `akbaba.txt`) and read all of it.

**Conventions**

- Page numbers are the printed (Arabic) numbers. To get the PDF page, add 18 for Aguş and 22 for Uzel and Akbaba.
- All three theses are in English. Only the required Turkish abstract ("Öz") is in Turkish. "Tez" in Akbaba's file name means "thesis".
- Equations are counted as distinct numbered tags such as (3.4). Sub-labelled constraint rows such as (2.15a) are counted separately.
- Figures and tables are counted as distinct numbered captions. References are counted as numbered bibliography entries.
- Pages per chapter run from the chapter's first page to the start of the next chapter, so they include any blank page at the end of a chapter.

**Rating scale (1–5), used for all three theses**

| Dimension | 1 | 2 | 3 | 4 | 5 |
|---|---|---|---|---|---|
| Theoretical | textbook equations only | light modelling | own formal statements or substantial derivations | extensive derivations plus numerical stability or convergence analysis | formal proofs of stability, convergence or optimality |
| Algorithmic | off-the-shelf | composition of existing blocks | new pipeline with non-trivial design choices | new algorithms or controllers specified in detail | new algorithm with complexity or optimality analysis |
| Methodological | method barely specified | method motivated, but problem formulation or evaluation protocol is thin or inconsistent | method and setup reproducible, design choices partly justified, gaps in the evaluation protocol | every component formulated, assumptions stated, design choices ablated, sim-to-real check | fully specified, with threats to validity and a rigorous protocol |
| Experimental | illustrative demos | a few quantitative results, no baselines or statistics | systematic experiments with ablations or baselines in one domain | systematic simulation plus real hardware, with baselines or ablations | extensive real-world evaluation with repeated trials and statistics |

---

## 1. Ege Uğur Aguş: "A Sampling-Based Planning Method for Dynamic Optimization Problems"

### 1.1 Profile card

| Field | Value |
|---|---|
| Author | Ege Uğur Aguş |
| Year | September 2023 (defence 07.09.2023) |
| Department / program | Mechanical Engineering, METU (not the Robotics program); no lab named |
| Degree | M.S. |
| Advisors | Supervisor Assoc. Prof. Dr. Ali Emre Turgut (ME); co-supervisor Assoc. Prof. Dr. Ahmet Buğra Koku (ME) |
| Committee | Koku; Turgut; Prof. Dr. Gökçe Nur Yılmaz (CENG, TED U.); Assist. Prof. Dr. Kutluk Bilge Arıkan (ME, TED U.); Assoc. Prof. Dr. Ender Yıldırım (ME, METU) |
| Total pages | 107 PDF pages = 18 front-matter pages (i–xviii) + 89 numbered pages |
| Numbered equations | 11 (Ch. 2: 1, Ch. 3: 5, Ch. 4: 5) |
| Formal statements | 11 "lemmas" (most are actually definitions), 2 corollaries, 2 theorems, 3 informal proofs |
| Algorithm / pseudocode blocks | 4 pseudocode listings, placed as figures: Fig. 3.5 POPS (15 lines), Fig. 3.9 "Algorithm 1" (25 lines), Fig. 3.10 "Algorithm 2" (28 lines), Fig. 3.19 final version (44 lines over two pages). There are also 4 numbered step lists in the prose: RRT, PRM and LNS in Ch. 2, and the greedy + LNS flow in Ch. 4 |
| Figures | 52 (Ch. 2: 4, Ch. 3: 29, Ch. 5: 12, App. A: 7). Of these, 4 are the pseudocode listings, 1 is an adjacency matrix and 7 are search-tree diagrams |
| Tables | 0 (the thesis has no list of tables) |
| References | 22 |

**Table of contents (printed pages)**

- 1 Introduction, 1
  - 1.1 Motivation and Problem Definition, 1 · 1.2 Proposed Methods and Models, 4 · 1.3 Contributions and Novelties, 5 · 1.4 The Outline of the Thesis, 5
- 2 Literature Survey, 7
  - 2.1 Introduction, 7 · 2.2 Planning Algorithms, 7
    - 2.2.1 Search Methods, 8
      - 2.2.1.1 Uninformed Search, 9 (BFS 9, DFS 10)
      - 2.2.1.2 Informed Search, 10 (Dijkstra 10, A* 10, Best-First 11)
    - 2.2.2 Combinatorial Planning, 12 (Roadmap 12)
      - 2.2.2.1 Vertical Cell Decomposition, 12 · 2.2.2.2 Shortest Path Roadmap, 13
    - 2.2.3 Sampling-Based Planning, 15
      - 2.2.3.1 Sampling Methods, 15
      - 2.2.3.2 Single-Query Models, 17 (RRT, RRT*, FMT*, Informed RRT* 18; BIT* 19)
      - 2.2.3.3 Multi-Query Models, 19 (PRM 19, PRM* 20, VRM 20)
    - 2.2.4 Discrete Optimization, 21 (Branch and Bound, Constrained Programming, Local Search 21; Linear Programming, Genetic Algorithms / Meta-heuristics, LNS 22)
- 3 Methodology, 25
  - 3.1 Building Paths, 25 · 3.2 POPS: Possible Optimal Paths Search, 29 · 3.3 Building Roadmap, 32 · 3.4 Algorithm Design, 35 · 3.5 Improving Algorithm's Performance, 37 · 3.6 Building Regions, 50 · 3.7 Finding Solution to a Goal Node, 55 · 3.8 Region Orders, 56
- 4 Formulation of the Case Study, 59
  - 4.1 Formulation, 59 ("Constrain 1", 60; 4.1.1 Assumptions, 60) · 4.2 Applying Discrete Optimization to Find Optimal Order of Targets, 60
- 5 Simulation Results and Discussion, 63
  - 5.1 Simulation Results, 63
    - 5.1.1 Results for Map 1, 63 (5.1.1.1 Scenario 1, 63; 5.1.1.2 Scenario 2, 64)
    - 5.1.2 Results for Map 2, 65
  - 5.2 Discussion, 66
- 6 Conclusion, 77
- References, 81
- Appendix A Evaluations of the Methods in the Literature Survey, 83
  - A.1 Method Evaluation, 83 (A.1.1 Planning Algorithms' Evaluation, 83; A.1.2 Discrete Optimization Algorithms' Evaluation, 86)

**Pages per chapter**

| Part | Pages | Count | Share of 89 |
|---|---|---|---|
| Front matter | i–xviii | 18 | – |
| 1 Introduction | 1–6 | 6 | 7% |
| 2 Literature Survey | 7–24 | 18 | 20% |
| 3 Methodology | 25–58 | 34 | 38% |
| 4 Formulation of the Case Study | 59–62 | 4 | 4% |
| 5 Simulation Results and Discussion | 63–76 | 14 | 16% |
| 6 Conclusion | 77–80 | 4 | 4% |
| References | 81–82 | 2 | 2% |
| Appendix A | 83–89 | 7 | 8% |

**Experiments**

- **Setup.** 2D simulation only, written in Python and visualised with Pygame. No hardware.
- **Illustrative dense map (3000 samples).** The planner produces 198 roadmap nodes and 1132 complete paths. With "brotherhood" pruning this falls to 127 nodes (−46%) and 580 paths (−49%), grouped into 69 regions (pp. 50–54).
- **Map 1.** 2621 samples on 450 × 450 units, brotherhood threshold 18, region threshold 36, 79 regions.
  - Scenario 1: 4 targets. Alternative paths are shown for 2 and 5 drones; the solution uses 2 drones.
  - Scenario 2: 2 targets, 2 drones.
- **Map 2.** 2021 samples, 231 regions, 4 targets. Alternatives are shown for 2 and 3 drones; the solution uses 3 drones.
- **Common settings.** Each target needs 100 time units of laser exposure. The battery constraint is relaxed in every scenario.
- **Results reporting.** Each scenario is one run, shown only as figures. No runtime, mission-time, cost or success metric is tabulated. The only performance claim is "within seconds" against "for hours" (p. 49).
- **Baselines (Appendix A).** Qualitative screenshots only:
  - RRT, 4 runs; one hits the 10,000-node limit without a solution.
  - RRT*, 4 runs.
  - RRT and RRT* in a narrow passage; both fail.
  - Visibility Roadmap, 3 runs.
- **Negative result.** The LNS optimiser "could not improve the initial solution" from the greedy step (p. 68), and the thesis says so openly.

**Stated contributions (p. 5)**

1. A new search method that finds, from a single source, every path that could be optimal (POPS, Possible Optimal Paths Search).
2. A new hybrid roadmap–tree planner that keeps all alternative solutions for dynamic optimisation problems.
3. A new system concept: drones relay a laser beam to extend what laser weapon systems can do.

**Contribution type.** An algorithm (a new graph search plus roadmap construction), with light formal theory and an application case study that is only simulated.

**Depth of theory.** Formal in appearance, informal in rigour.
- Chapter 3 is built on numbered lemmas, corollaries and theorems. Most "lemmas" define terms: sub-optimal, proper and complete paths; the seen-nodes and candidates-common vectors; domination; exploration versus connectivity; brotherhood; region.
- Theorem 3.3.3 (p. 33): a roadmap built from all complete paths can reach any goal in a continuous free space. The proof is a short argument by contradiction.
- Theorem 3.5.6 (pp. 46–49): once exploration is complete, the roadmap holds between U and U + (AP − 1) nodes, where AP is the number of active paths.
- What is missing:
  - complexity analysis;
  - a proof that POPS returns exactly the set of paths optimal for some weight vector, which the conclusion claims (p. 77);
  - any analysis of sampling completeness.
- Terminology is non-standard: "sub-optimal" means "non-redundant, possibly optimal".

**Pretrained models and third-party libraries.**
- No pretrained models.
- Python and Pygame (cited as [22]) get one sentence, in the conclusion (p. 78).
- Literature methods are summarised in Ch. 2 at about one paragraph each, some as numbered steps: BFS/DFS, Dijkstra, A*, cell decomposition, the RRT and PRM families, VRM, branch and bound, constraint programming, LNS.
- The collision checker and the LNS implementation are never described.

### 1.2 How the methodology chapter is organised and how algorithms are presented

**Organisation.** Chapter 3 (34 pp.) is written as a design narrative, not a specification.

1. **Requirements.** It opens with three things the new planner must do:
   - provide alternative paths;
   - make those alternatives optimal rather than redundant;
   - need no rebuild when the start is fixed and the goal changes.
2. **Toy example.** On a 9-node roadmap (Fig. 3.1) it shows that classical BFS loses alternatives and that BFS without visited-marking floods with redundant paths.
3. **First definitions.** It defines "sub-optimal" paths (Lemma 3.1.1 and two corollaries), then gives POPS (Fig. 3.5).
4. **Proper and complete paths.** These lead to Theorem 3.3.3, followed by Algorithm 1 and then Algorithm 2, which adds the seen-nodes vector.
5. **Scaling problems.** On the 3000-sample map, performance problems motivate a series of refinements:
   - candidates-common vector and node domination;
   - exploration versus connectivity, and Theorem 3.5.6;
   - an adjacency-matrix cache of collision checks;
   - a brotherhood threshold;
   - regions;
   - goal insertion and region orders.
6. **Each refinement follows the same pattern.** A failure case is shown in a search-tree or map figure, formalised as a "lemma", folded into the next pseudocode version and quantified with node and path counts.

Chapter 4 (4 pp.) is the only formal problem statement: set definitions, one constraint, two assumptions, and the LNS flow as a bullet list. It has no objective function in equation form.

**Presentation mix**

| Format | How it is used |
|---|---|
| Prose | dominant |
| Figures | 29 in Ch. 3, mostly toy maps and search trees |
| Equations | 5 in Ch. 3 |
| Pseudocode | 4 listings in Require/Ensure style with numbered lines, placed as figure images, each explained line by line in the prose |
| Flowcharts | none |

**Example quotes**

- "However, we need a new approach to planning algorithms to find a set of possible solutions mainly because planning algorithms in the literature, regardless of being multi-query or single-query, are only interested in finding a single path from x_i to x_g." (p. 25)
- "Lemma 3.1.1. A path is sub-optimal if any of the subsets, including its first and last node, of the path is not feasible." (p. 26). This shows the definition-as-lemma style.
- "The pseudo-code for finding complete paths is presented in the figure 3.9. Between line 6 to line 14 algorithm finds sub-optimal paths of the currently inspected path." (p. 35). This shows the line-referenced walkthrough of each listing.
- "It can solve the figure 3.21 within seconds, while it could not find the solution for hours without theorem 3.5.6." (p. 49). This is the only performance evidence, and it is qualitative.

### 1.3 Depth ratings

| Dimension | Score | Justification |
|---|---|---|
| Theoretical | 3 | It is the only one of the three with its own lemmas and theorems with proofs, but the proofs are informal, most lemmas are definitions, and there is no complexity or optimality analysis. |
| Algorithmic | 4 | A new search and roadmap algorithm is refined through four pseudocode versions with several pruning mechanisms. It still lacks complexity analysis, and the listings contain syntax slips. |
| Methodological | 2 | The design narrative is well motivated, but the case study gets four pages with an inconsistent constraint, LNS is underspecified, and no evaluation protocol or metrics are defined. |
| Experimental | 2 | Two hand-made 2D maps and three single-run scenarios, presented as pictures with no runtime or cost metrics. Baselines are screenshots only. |

**Quality issues that passed the committee**

- The outline in §1.4 does not match the actual chapters, ending with "in Chapter 5, the conclusion" (p. 5).
- Constraint 1 says a path has at most n + 2 elements (n = number of drones), but Eq. 4.5 writes n(τ) ≤ m, where m is the number of targets (p. 60).
- The pseudocode has slips: "add_flag == 0" where an assignment is meant, and "dominated = 0E" (Fig. 3.19, pp. 45–46).
- §5.2 repeats a paragraph almost word for word (p. 66).
- The cross-reference between Fig. 3.17 and Fig. 3.22 is swapped (p. 50).

---

## 2. Denge Uzel: "Learning to Assemble Furniture from Their 2D Drawings"

### 2.1 Profile card

| Field | Value |
|---|---|
| Author | Denge Uzel |
| Year | December 2023 (defence 07.12.2023) |
| Department / program | Mechanical Engineering, METU (not the Robotics program); METU ROMER |
| Degree | M.S. |
| Advisors | Supervisor Assoc. Prof. Dr. Ahmet Buğra Koku (ME); co-supervisor Prof. Dr. Sinan Kalkan (CENG) |
| Committee | Assoc. Prof. Dr. Ali Emre Turgut (ME); Koku; Kalkan; Assoc. Prof. Dr. Erol Şahin (CENG); Assist. Prof. Dr. Kutluk Bilge Arıkan (ME, TED U.) |
| Funding | TÜBİTAK 1001 KALFA project 120E269 ("New Methods for Assembly Scenarios with Collaborative Robots") |
| Total pages | 79 PDF pages = 22 front-matter pages (i–xxii) + 57 numbered pages |
| Numbered equations | 6 (Ch. 3: 4, Ch. 5: 2), plus 1 unnumbered threshold rule |
| Algorithm / pseudocode blocks | 0 |
| Figures | 42 (Ch. 1: 2, Ch. 2: 1, Ch. 3: 14, Ch. 4: 8, Ch. 5: 17). Five are reproduced from cited papers and one is an AI-generated illustration (Adobe Firefly, Fig. 1.2). The 17 results figures are loss/accuracy curves, confusion matrices and RL training curves |
| Tables | 7: Table 2.1 dataset comparison; Tables 4.1–4.5 furniture lists, splits, ModelNet40 classes, negative-sample counts, hyperparameters; Table 5.1 RL split. None tabulates results |
| References | 38 |

**Table of contents (printed pages)**

- 1 Introduction, 1
  - 1.1 Motivation and Problem Definition, 1 · 1.2 Contributions, 3 · 1.3 Outline of the Thesis, 4
- 2 Background and Related Work, 7
  - 2.1 Furniture Assembly, 7
    - 2.1.1 Assembly Learning Based on 3D Model Knowledge, 7 · 2.1.2 Assembly Learning Based on 2D Instruction Manual Knowledge, 8
  - 2.2 Datasets and Simulation Environments, 10
- 3 Proposed Model: AssembleRL-2D, 13
  - 3.1 Overview, 13
  - 3.2 Data Collection, 13 (3.2.1 2D Data, 14; 3.2.2 3D Data, 17)
  - 3.3 A Novel Network 2D-3D Similarity, 20
    - 3.3.1 Input Features and Outputs, 21
    - 3.3.2 Loss Function, 21 (3.3.2.1 Binary Cross Entropy Loss, 22; 3.3.2.2 Focal Loss, 22)
    - 3.3.3 Network Layers, 23 (3.3.3.1 2D Feature Extractor, 23; 3.3.3.2 3D Feature Extractor, 24, covering PointNet++ and PPFNet; 3.3.3.3 Multi-layer Perceptron, 26)
  - 3.4 Assemble Learning with 2D-3D Similarity Network, 26
- 4 Experimental Setup, 29
  - 4.1 Datasets, 29
    - 4.1.1 Datasets for Pretraining Feature Extractors, 30 (4.1.1.1 ImageNet, 30; 4.1.1.2 ModelNet10, 32; 4.1.1.3 ModelNet40, 32)
    - 4.1.2 2D Dataset, 33 · 4.1.3 3D Dataset, 33
  - 4.2 Paired Input Generation, 34 · 4.3 Implementation and Training Details, 38
- 5 Experiments and Results, 39
  - 5.1 Experiments and Architecture Details, 39 · 5.2 Experiment 1: Quantitative Analysis of Assembly, 40 · 5.3 Experiment 2: Ablation and Hyperparameter Analyses, 41 · 5.4 Experiment 3: Visual Results, 44 · 5.5 Experiment 4: Learning Furniture Assembly Stages with AssembleRL-2D, 45
- 6 Conclusion and Future Work, 51 (6.1 Future Work, 52)
- References, 53

**Pages per chapter**

| Part | Pages | Count | Share of 57 |
|---|---|---|---|
| Front matter | i–xxii | 22 | – |
| 1 Introduction | 1–6 | 6 | 11% |
| 2 Background and Related Work | 7–12 | 6 | 11% |
| 3 Proposed Model: AssembleRL-2D | 13–28 | 16 | 28% |
| 4 Experimental Setup | 29–38 | 10 | 18% |
| 5 Experiments and Results | 39–50 | 12 | 21% |
| 6 Conclusion and Future Work | 51–52 | 2 | 4% |
| References | 53–57 | 5 | 9% |

**Experiments**

- **Data.**
  - 16 IKEA models in 3 classes: 5 bookcases, 6 chairs, 5 tables. Split 9 / 3 / 4 into train, validation and test furniture (Table 4.2).
  - 2D: 360 Blender renders per train/validation model (one per degree) and 90 front views per test model.
  - 3D: one positive (fully assembled) point cloud per model, and up to 1000 sampled negatives per model (raw counts range from 7 to 2115, Table 4.4).
- **Experiment 1.** PointNet++ against PPFNet as the 3D encoder, pretrained on ModelNet10 and ModelNet40 (200 epochs, batch 32). PPFNet on ModelNet40 is chosen.
- **Experiment 2.**
  - Learning-rate grid: 27 combinations, 10 epochs each, chair class only.
  - MLP shape: 2 or 3 layers × 5 hidden widths, 20 epochs.
  - Focal-loss γ sweep for each class.
- **Experiment 3.** Per-class training for 20 epochs with a single seed (8).
  - Training accuracy above 95% and validation above 90%.
  - Test accuracy "nearly 80%" for bookcases and tables. The chair test accuracy is not reported as a number.
- **Experiment 4.** AssembleRL-2D used as the reward inside AssembleRL on the Ivar chair, with 3 seeds (1, 99, 14710).
  - The episodic return converges near 3.5.
  - Success is judged by inspecting the saved assemblies.
  - The explained variance never approaches 1.
- **Hardware.** None. Training ran on one RTX 3060 Ti.
- **Baselines.** No external baseline method is compared.

**Stated contributions (p. 4)**

1. A new dataset pairing 2D manual-like drawings with 3D representations of possible assembly actions for IKEA furniture.
2. Assembly learning framed as reinforcement learning whose reward comes only from the 2D drawing of the target ("to the best of our knowledge, we are the first").
3. A network that learns the similarity between a 2D drawing and a 3D object, so the robot can tell whether the current assembly matches the drawing.
4. Generalisation to furniture not seen in training.

**Contribution type.** A dataset plus an applied deep-learning model (pretrained encoders composed together), plugged into the lab's existing RL framework (AssembleRL, Aslan et al., IROS 2022). No hardware and no robot-system integration.

**Depth of theory.** Minimal. There are six textbook equations:
- sigmoid;
- binary cross-entropy (Eq. 3.2), printed without its minus sign;
- the two-case form of cross-entropy;
- focal loss;
- the AdamW update;
- linear learning-rate decay.

There are no derivations, and no analysis of the similarity measure or of the reward it creates in the RL loop.

**Pretrained models and third-party libraries, and how they are explained**

- **ResNet18**, pretrained on ImageNet-1K and loaded from torchvision.
  - The average-pooling layer is removed, giving a 512-dimensional feature.
  - The text notes "approximately 11.7M parameters" and quotes torchvision's top-1/top-5 accuracy (69.758% / 89.078%).
- **PointNet++ and PPFNet**, pretrained by the author on ModelNet10/40.
  - PPFNet is chosen because it uses point normals and is rotation-invariant.
  - For both, the last MLP layers are removed to get 512-dimensional features.
- **AssembleRL** (PPO with a graph-convolutional policy, reward from incompleteness and incorrectness) serves as the agent and environment.
- **The IKEA Furniture Assembly Environment** supplies the XML files and part meshes.
- **Rendering and training tools**: Blender (Python API, Grease Pencil line art) renders the manual-like images; training uses PyTorch, torchvision transforms, PIL, AdamW and sigmoid focal loss.
- **Explanation style.**
  - One to three paragraphs per model, usually with the original paper's architecture figure reproduced ("Figure Source: [n]").
  - Background on the pretraining datasets: an ImageNet/WordNet hierarchy figure and a ModelNet40 class table.
  - Fine-tuning is never stated outright; it is only implied by the separate learning rates per network (Table 4.5).

### 2.2 How the methodology chapter is organised and how algorithms are presented

**Organisation.** The method is split across Ch. 3, "Proposed Model" (16 pp.), and Ch. 4, "Experimental Setup" (10 pp.).

- **Chapter 3** has four parts:
  1. Overview (Fig. 3.1 places AssembleRL-2D next to AssembleRL).
  2. Data collection:
     - 2D: a step-by-step Blender recipe with concrete values for scale, camera pose, line thickness and angle step.
     - 3D: the AssembleRL pipeline reused to produce positive and negative point clouds.
  3. The network: a block diagram (Fig. 3.8), then inputs and outputs, the loss functions, and the layers.
     - The layers are ResNet18, then the switch from PointNet++ to PPFNet, then a 3-layer MLP on the element-wise sum of the two 512-dimensional features.
  4. Integration as the RL reward (Fig. 3.14).
- **Chapter 4** covers:
  - the datasets, including background on the pretraining datasets;
  - how input pairs are generated, with the transform values and the positive/negative sampling ratio;
  - the hyperparameter table (Table 4.5).

**Presentation mix**

| Format | How it is used |
|---|---|
| Prose | implementation-level and configuration-heavy |
| Block diagrams | Figs. 3.8, 3.13, 3.14, 4.1, 4.8 |
| Reproduced figures | architecture figures from the cited papers |
| Equations | four textbook loss equations |
| Pseudocode | none (the pairing and reward logic are described only in prose) |

**Example quotes**

- "The main problem addressed here is to calculate the similarity between two-dimensional and three-dimensional inputs." (p. 20)
- "The brush radius of the Grease Pencil object is set to 1. The object input type is designated as a line art object (LRT_OBJECT) to be added to the active object, i.e., the furniture." (p. 15). This shows the implementation-level detail of the data pipeline.
- "The study initially employed PointNet++ for 3D feature extraction. However, inconsistencies were noted in the results when the same point cloud transformed." (p. 24). Design changes are justified by short empirical observations.
- "Using this recorded model, 2D manual images are passed through AssembleRL-2D, and the network result is considered a mean without employing any thresholding, serving as the reward signal, outlined in Figure 3.14." (p. 27). The RL integration is given in one prose paragraph plus a diagram.

### 2.3 Depth ratings

| Dimension | Score | Justification |
|---|---|---|
| Theoretical | 1 | Only textbook loss and optimiser equations, one with a sign error; no derivations or analysis. |
| Algorithmic | 2 | Two pretrained encoders, an element-wise addition and a 3-layer MLP, used as a reward in an existing RL framework; the novelty is the problem framing, not the algorithm. |
| Methodological | 3 | Reproducible data generation, test furniture held out, and design choices partly justified, but the splits disagree between tables, only one seed is used and no baseline is defined. |
| Experimental | 3 | Systematic ablations (encoder, learning-rate grid, MLP shape, γ) plus an RL demonstration, but test results appear only in prose, the confusion matrices show training results, and there is no baseline or hardware. |

**Quality issues that passed the committee**

- §3.3.3 says the 3D features come from "pre-trained PointNet++", but the final architecture uses PPFNet (p. 23).
- ModelNet10 is described using statistics for the whole ModelNet collection: "more than 150K 3D CAD models belonging to 660 object categories" (p. 32).
- Table 4.2 (p. 30) assigns bookcases and tables to train/validation/test differently from Table 4.4 and Fig. 4.7 (pp. 35–36).
- The conclusion calls the result an "unambiguous success" and the architecture "groundbreaking", with roughly 80% test accuracy on two of the three classes (pp. 51–52).
- The abbreviation list still contains a placeholder: "STL Stereolithography / Standard Triangle Language (?)".

---

## 3. Mustafa Akbaba: "Template-Based and Optimal Control of Dynamic Quadrupedal Gaits"

### 3.1 Profile card

| Field | Value |
|---|---|
| Author | Mustafa Akbaba |
| Year | November 2025 (defence 26.11.2025) |
| Department / program | Electrical and Electronics Engineering, METU (not the Robotics program); METU ROMER |
| Degree | M.S. |
| Advisors | Supervisor Assoc. Prof. Dr. Mustafa Mert Ankaralı (EEE); co-supervisor Assoc. Prof. Dr. Ali Emre Turgut (ME) |
| Committee | Prof. Dr. Afşar Saranlı (EEE); Ankaralı; Assoc. Prof. Dr. İsmail Uyanık (EEE, Hacettepe U.) |
| Funding | TÜBİTAK 2210-A scholarship; support from Türk Telekom |
| Total pages | 132 PDF pages = 22 front-matter pages (i–xxii) + 110 numbered pages |
| Numbered equations | 127 (Ch. 2: 43, Ch. 3: 52, Ch. 4: 15, App. A: 17), plus 19 sub-labelled constraint rows such as (2.15a–c) and (4.14a–c). Several numbered tags hold multi-line systems or block matrices |
| Algorithm / pseudocode blocks | 0. Instead there are 4 controller block diagrams (Figs. 2.8, 2.13, 3.5, 4.5) and 1 flowchart (Fig. 4.2) |
| Figures | 51 (Ch. 2: 25, Ch. 3: 15, Ch. 4: 10, App. A: 1) |
| Tables | 6. Two report results (2.3 sim-vs-real RMSE; 2.4 stance-timing RMSE); four list parameters (2.1, 2.2, 3.1, 4.1) |
| References | 79 |

**Table of contents (printed pages)**

- 1 Introduction, 1
  - 1.1 Motivation and Background, 1 · 1.2 Contributions, 6 · 1.3 The Outline of the Thesis, 8
- 2 A Comprehensive Clock Torque Actuated SLIP Framework for Trotting in Quadrupeds, 11
  - 2.1 Template Model, 11 · 2.2 Controller Model, 16 · 2.3 Trotting Gaits: Walking to Running, 20 · 2.4 Turning in the Yaw Direction, 22 · 2.5 Whole-Body Controller, 24 · 2.6 State Estimation, 26 · 2.7 Experimental Setup, 28
  - 2.8 Simulation Results, 31
    - 2.8.1 Gait Transitions and Disturbance Rejection, 31 · 2.8.2 Turning with and without compensation, 33 · 2.8.3 Template-based Control vs. Whole-body Control Improved Template-based Control, 34
  - 2.9 Hardware Experiments, 35 (2.9.1 Comparison with Simulation, 38)
- 3 Template-Based Quadrupedal Pronking: Analysis and Extension with Model Predictive Control, 47
  - 3.1 Spring-Damper Template, 47 · 3.2 Motion Generation From Template, 48 · 3.3 Controller Design, 49
  - 3.4 Approximate Analytical Foot Step Controller, 55 (3.4.1 Formulating The Model, 55; 3.4.2 Solving For The Optimal Foot Location, 57)
  - 3.5 Proposed Approach, 59 · 3.6 Simulation Analysis, 61 (3.6.1 Results From Simulation, 63) · 3.7 Hardware Experiments, 65
- 4 An Optimization-Driven Foot Placement Controller for Quadruped Landing, 77
  - 4.1 Template Model and Stance Controller, 77 · 4.2 Foot Placement Controller, 80 · 4.3 System-Level Control Architecture, 85
  - 4.4 Experimental Results, 87 (4.4.1 Simulation Experiments, 88; 4.4.2 Real Robot Experiments, 90)
- 5 Conclusions, 97
- References, 99
- Appendix A Equations of Motion of a Quadrupedal Robot, 107

**Pages per chapter**

| Part | Pages | Count | Share of 110 |
|---|---|---|---|
| Front matter | i–xxii | 22 | – |
| 1 Introduction (includes the literature review) | 1–10 | 10 | 9% |
| 2 CT-SLIP trotting framework | 11–46 | 36 | 33% |
| 3 Pronking with MPC | 47–76 | 30 | 27% |
| 4 Landing foot-placement NMPC | 77–96 | 20 | 18% |
| 5 Conclusions | 97–98 | 2 | 2% |
| References | 99–106 | 8 | 7% |
| Appendix A (equations of motion) | 107–110 | 4 | 4% |

Inside each core chapter the split between method and results is roughly even:

| Chapter | Method sections | Results sections |
|---|---|---|
| Ch. 2 | §§2.1–2.7, about 20 pp. | §§2.8–2.9, about 16 pp. |
| Ch. 3 | §§3.1–3.5, about 14 pp. | §§3.6–3.7, about 16 pp. |
| Ch. 4 | §§4.1–4.3, about 10 pp. | §4.4, about 9 pp. |

**Experiments**

All three behaviours (trotting, pronking, landing) were tested both in simulation (MuJoCo, Unitree GO1 model) and on a real Unitree GO1.

- **Ch. 2, trotting, simulation**
  - A 30-s gait-transition run (walking trot, then trot, then flying trot).
  - Lateral push recovery for each gait: up to 0.53, 0.54 and 0.49 m/s.
  - Yaw compensation: maximum yaw rate 2.1 rad/s without it, 2.9 rad/s with it.
  - Template-based control (TBC) against TBC plus whole-body control (WBC): push recovery up to 1.21 and 1.27 m/s respectively.
- **Ch. 2, trotting, hardware**
  - Robot setup: a Raspberry Pi 4 runs the 300 Hz controller and the 600 Hz Kalman estimator; a Pixhawk 4 supplies the IMU.
  - One 30-s run across the three gaits.
  - Mechanical cost of transport 0.7, against 1.94 for GO1's default controller at about 0.35 m/s.
  - Sim-vs-real RMSE for one 10-s trial (Tables 2.3 and 2.4).
- **Ch. 3, pronking, simulation**
  - Adaptive pitch reference tested at 0.5, 1 and 1.5 m/s.
  - Raibert heuristic against the proposed analytical footstep planner. Mean apex velocity is 1.10 m/s for Raibert and 1.21 m/s for the planner (target 1.5 m/s); mean apex height is 36 cm and 38 cm (target 40 cm).
  - Extreme start (0.7 m height, 5 m/s): the Raibert planner falls; the proposed planner stabilises within two hops.
  - TBC, LTI-MPC and LTV-MPC compared at 0.44 m apex height.
  - Push recovery up to 1.8 (LTV-MPC), 1.6 (TBC) and 1.4 m/s (LTI-MPC).
  - A basin-of-attraction grid over initial apex states, 12-s simulation each.
- **Ch. 3, pronking, hardware.** One pronking run at 0.5 m/s with contact estimated from joint torque.
- **Ch. 4, landing, simulation**
  - 4 drop heights (0.35, 0.5, 0.8, 1.0 m) × throwing directions in 30° steps, recording the highest stable throwing velocity for each.
  - Compared against the VHSIP baseline [45]: 3.6 against 1.75 m/s at 0.5 m (+105%), and 5.0 against 3.0 m/s at 1.0 m (+67%).
  - The NMPC solves in under 1 ms.
- **Ch. 4, landing, hardware.** One throw from 66 cm at 0.66 m/s. Control runs on a laptop over Ethernet, and OptiTrack (120 Hz) estimates state during flight.
- **Trial counts.** Hardware results are single demonstration runs per behaviour, with no repeated trials or statistics. Simulation tests are mostly single runs per condition, apart from the basin and polar sweeps.

**Stated contributions (pp. 6–8)**

1. **Trotting.**
   - The CT-SLIP template (spring-loaded inverted pendulum with clock-driven hip torque) generalised to 3D.
   - Projectile motion added to the template so the flying trot has a flight phase.
   - Smooth transitions among walking trot, trot and flying trot under one controller.
   - Hardware validation.
   - A whole-body controller added on top.
2. **Pronking.**
   - The template regenerates the reference at every control step, and an LTV-MPC built from the system Jacobians tracks it.
   - An analytical study of the template produces an approximate analytical footstep planner, implemented as an NMPC.
   - The stated novelty is combining template-based control with MPC.
3. **Landing.**
   - An NMPC foot-placement controller that minimises the gap between the touchdown moments the template predicts and those the ground reaction forces produce.
   - It respects friction and geometric constraints and includes angular dynamics.
   - Validated in simulation and on the real robot.

**Contribution type.** Control-algorithm design (template-based control combined with QP, LTV-MPC and NMPC), supported by modelling derivations and validated in simulation and on hardware. It also involves some on-robot system integration (Raspberry Pi controller, extra IMU).

**Depth of theory.** Heavy on modelling and derivation; no formal proofs.
- **Ch. 2 derivations:**
  - clock and phase logic, and CT-SLIP stance dynamics (2.1–2.7);
  - 3D template forces (2.8–2.10);
  - floating-base decomposition and the ground-reaction-force QP (2.11–2.15);
  - the swing-leg polynomial (2.17);
  - the WBC rewritten as a single QP (2.22–2.31);
  - a linear Kalman filter (2.32–2.41).
- **Ch. 3 derivations:**
  - reference generation by Taylor stepping (3.2–3.9);
  - the LTV-MPC condensed into a convex QP using Euler-stepped Jacobians, with an explicit convexity argument (3.11–3.32);
  - closed-form damped-spring solutions and the apex return map behind the footstep NMPC (3.33–3.49).
- **Ch. 4 derivations:** the landing NMPC with friction-cone projection (4.7–4.15).
- **Appendix A:** full Lagrangian equations of motion, generated with MATLAB Symbolic.
- **Stability evidence:** numerical only. A basin-of-attraction study shows convergence to one limit cycle within two hops. There are no Lyapunov or formal convergence proofs, and two relations (3.50, 3.51) are empirical fits.

**Pretrained models and third-party libraries, and how they are explained**

- No pretrained models.
- **MuJoCo** gets a full paragraph on its soft-contact penalty model and Coulomb friction (pp. 28–29).
- **Robot model**: the GO1 MJCF file comes from Google DeepMind's MuJoCo Menagerie [62].
- **Solvers and numerics**, each mapped to the equation it solves:

  | Library | Solves |
  |---|---|
  | qpOASES | the ground-reaction-force and MPC quadratic programs |
  | ALGLIB | the equality-constrained WBC quadratic program |
  | NLopt | the footstep and landing NMPCs |
  | GSL | inverse-kinematics root finding |
  | Eigen | linear algebra |
  | MATLAB Symbolic Math Toolbox | equations of motion and Jacobians |

- **Hardware**: OptiTrack, Raspberry Pi 4 and Pixhawk 4 are also described.
- Each tool gets one or two sentences saying what it is and which equation it solves, for example "To solve the optimization problem in equation 2.15, the qpOASES library is used" (p. 30).

### 3.2 How the methodology chapter is organised and how algorithms are presented

**Organisation.** There is no single "Methodology" chapter. The thesis has three self-contained core chapters, apparently adapted from papers (the text still says "In this paper" in several places). Each follows the same template:
1. The template model, as a schematic figure plus equations.
2. The controller formulation, written as min / s.t. optimisation blocks with lettered constraints: a stance QP (Ch. 2), an LTV-MPC (Ch. 3) or an NMPC (Ch. 4).
3. Supporting modules: swing trajectory, yaw compensation, WBC, Kalman state estimation, contact estimation.
4. A block diagram of the whole controller.
5. The experimental setup and libraries.
6. Simulation results.
7. Hardware experiments.

The derivations proceed step by step and state matrix dimensions, for example "The size of W is 18 × 18 for full stance" (p. 25). The full equations of motion are moved to Appendix A.

**Presentation mix**

| Format | How it is used |
|---|---|
| Equations | dominant (about 127 numbered) |
| Prose | defines each symbol and assumption |
| Figures | template schematics, 4 controller block diagrams, 1 flowchart (Fig. 4.2, the foot-placement optimisation loop) |
| Pseudocode | none (control logic is stated in prose summaries such as "To sum up, first, the controller detects…", p. 60) |

**Example quotes**

- "In this study, the template is used solely to generate reference motions for the robot, not for direct control." (p. 48)
- "In this paper, the controller directly controls the robot using the generalized coordinate accelerations for the COM rather than the robot's states. This approach reduces tuning parameters from 12 to 6 for the optimal control problem." (p. 50)
- "The objective function is convex because M is positive definite. The inequality constraints are affine." (p. 54). This shows the brief analytical justification attached to each formulation.
- "The controller runs an optimization loop in which each candidate foot location is evaluated by first estimating the corresponding touchdown state, then simulating the stance motion and computing the resulting template-based forces and moments." (p. 80). This is the prose that accompanies the Fig. 4.2 flowchart.

### 3.3 Depth ratings

| Dimension | Score | Justification |
|---|---|---|
| Theoretical | 4 | Extensive self-contained derivations (equations of motion, WBC and MPC condensing, analytical stance solutions, return map) plus a numerical basin-of-attraction analysis; no formal stability or convergence proofs. |
| Algorithmic | 4 | Three distinct controllers (template-based control with QP, LTV-MPC with a footstep NMPC, landing NMPC) plus WBC and a state estimator, all fully specified as optimisation problems. The novelty lies in formulation and integration rather than in new solvers. |
| Methodological | 4 | Every component is formulated with stated assumptions, ablated against alternatives (with/without compensation, TBC/WBC, LTI/LTV/TBC, Raibert/analytical) and checked sim-to-real. Gains and fitted terms are hand-tuned. |
| Experimental | 4 | Systematic simulation, ablations, a literature baseline and hardware for all three behaviours. The hardware evidence is single demonstration runs, and hardware landing performance is far below simulation. |

**Quality issues that passed the committee**

- Ch. 4 cites Fig. 2.25 for the landing sim-vs-real comparison; it should be Fig. 4.10 (p. 91).
- "The robot is in the air for approximately 20 milliseconds" after a throw from 66 cm (p. 91). A free fall from that height takes about 0.37 s.
- The abstract says "Simulation and hardware experiments demonstrate successful landings at horizontal velocities up to 5 m/s" (p. vi). The 5 m/s result is simulation only; the hardware landing is one throw at 0.66 m/s, and the thesis outline calls it "preliminary hardware experiments" (p. 9).
- The baseline [45] appears as dashed curves; the text says only "we conducted equivalent landing experiments using our NMPC-based controller" (pp. 88–89). It is not stated whether the baseline was re-run or taken from the paper.

---

## 4. Comparison of the three theses

| Thesis | Dept / year | Pages (PDF / numbered) | Chapters | Numbered equations | Pseudocode blocks | Figures | Tables | Refs | Real-robot experiments | Contribution type | Ratings T / A / M / E |
|---|---|---|---|---|---|---|---|---|---|---|---|
| Aguş, sampling-based planning for dynamic optimisation | ME, Sep 2023 | 107 / 89 | 6 + 1 appendix | 11 | 4 (as figure images) | 52 | 0 | 22 | No (2D Pygame simulation) | Algorithm with light formal theory; simulated case study | 3 / 4 / 2 / 2 |
| Uzel, furniture assembly from 2D drawings | ME, Dec 2023 | 79 / 57 | 6 | 6 | 0 | 42 | 7 | 38 | No (dataset and RL simulation) | Dataset plus applied deep-learning model inside an existing RL framework | 1 / 2 / 3 / 3 |
| Akbaba, template-based and optimal control of quadruped gaits | EEE, Nov 2025 | 132 / 110 | 5 + 1 appendix | 127 (+19 sub-rows) | 0 (4 block diagrams + 1 flowchart) | 51 | 6 | 79 | Yes (Unitree GO1: trotting, pronking, landing; single demonstration runs) | Control-algorithm design with derivations, validated in simulation and on hardware | 4 / 4 / 4 / 4 |

T = theoretical, A = algorithmic, M = methodological, E = experimental (scale defined at the top).

---

## 5. Takeaways for positioning the tack-welding cell / WeldSet thesis

- **Length and structure.**
  - These accepted MSc theses run 57–110 numbered pages, 5–6 chapters and 22–79 references.
  - A thesis with two contributions (the cell and WeldSet) will naturally land near Akbaba's 110 pages.
  - His layout is the closest model: a short shared introduction and conclusion, one self-contained chapter per contribution (model, method, setup, simulation, hardware), and derivations in an appendix.
  - Leftover "In this paper" wording and broken cross-references are the cost of that approach.
- **Committee expectations.** Turgut appears in all three (supervisor of Aguş, examiner of Uzel, co-supervisor of Akbaba), and Koku in two. None of the three is from the Robotics program, so they show ROMER-adjacent norms rather than that program's own rules.
- **Pseudocode.**
  - Only Aguş uses pseudocode, as figure images with syntax slips. Uzel and Akbaba rely on block diagrams, and Akbaba adds one flowchart.
  - Typeset, numbered algorithm blocks for the seam extractors, multi-view refinement and the benchmark protocol would beat all three on algorithm presentation.
- **Pretrained models.**
  - Uzel sets the lab's precedent: a paragraph, a reproduced architecture figure, the source of the weights and their published accuracy.
  - For SAM2 and FoundationPose, go further. State their inputs and outputs, what stays frozen, the failure modes you observed, and exactly where the thesis's own logic begins (prompting, ICP, multi-view refinement, seams from registered CAD primitives).
  - Doing so answers the "it is only integration" objection before an examiner raises it.
- **Theory.**
  - None of the three proves stability or convergence formally. Aguş's theorems are informal; Akbaba's depth comes from derivations.
  - The calibration chain (composing transforms, propagating errors) and WeldSet's analytic ground truth are natural places for numbered derivations at Akbaba's level.
- **Experiments and baselines.**
  - Only Akbaba ran on hardware, and only as single demonstration runs. Only Akbaba compares against a literature baseline; Uzel has none and Aguş shows screenshots.
  - Repeated real-robot trials with error statistics, plus a benchmark of seven re-implemented methods on data with analytic ground truth, would exceed all three experimentally.
  - Report how faithful each re-implementation is, and borrow Akbaba's sim-vs-real RMSE table format.
- **Claims.**
  - Uzel's "groundbreaking" next to about 80% test accuracy, and Akbaba's abstract presenting a simulation-only 5 m/s result as hardware, both passed the committee.
  - Claims that match the evidence are still the safer standard to set.
