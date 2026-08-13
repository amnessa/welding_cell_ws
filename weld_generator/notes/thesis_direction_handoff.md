# Thesis & Paper Direction — Conversation Handoff

*State as of the end of the previous conversation. Written to be pasted into a new chat.*

---

## 1. Who / setup

- Robotics MSc at METU, advisor Ahmet Buğra (ADEP project context; lab colleagues Umut Kurt, Ege Ecevit — CHERI/ROMER, Isaac Sim experience in-house).
- Hardware: UR5e, eye-in-hand RGB-D camera **mounted off-axis from the flange Z axis**, pen-holder tool (no torch).
- Workpieces: metal-painted 8 mm MDF held on a steel plate by magnets. Stainless sheet available is ~1–2 mm.
- Existing pipeline: RGB-D → SAM2 → PPF + FoundationPose → ICP-to-CAD → radius-PCA curvature seam extraction → admittance-controlled execution. Reference geometry used throughout: T-joint, ~232 mm seam, ~16 mm extracted band, ~1.1 mm root gap, 8.4 mm plate.
- GitHub: github.com/amnessa

---

## 2. How the direction evolved (read this before re-litigating anything)

The arc went **tack-ordering cost function → distortion/FEM → robotics-only planner → dataset**. Each pivot was toward *narrower and more defensible*, and the same failure mode recurred three times: reaching for a bigger contribution (FEM, DL-on-FEM, "foundation model") when the honest move was to go smaller.

**Decisions already made and settled:**

1. **No physical welding.** Degree is robotics; welding adds fume/fixturing/metrology burden and a distortion-measurement problem. Pen marks on MDF are the validation apparatus — they give ~0.1 mm placement ground truth via flatbed scan, far better than the RealSense could.
2. **No FEM.** Key finding that ended this branch: **FEM cannot produce tack-point labels, because tack positions are FEM's *input*, not its output.** FEM takes tack locations + sequence and returns distortion/stress. What the distortion literature *does* provide is constraints (spacing bounds vs. thickness, mandatory endpoint tacks, edge margin, "too many tacks increases angular distortion") and ordering heuristics. Use published inherent-deformation constants (Liang & Deng, *Thin-Walled Structures*; Tian & Luo, *J. Intelligent Manufacturing*) — cite, don't compute.
3. **No DL-on-FEM surrogate.** Crowded field (Wu/Wang/Kim *EAAI* 2022; Karimi & Jelovica *Welding in the World* 2025), requires *more* FEM competence than a single analysis, and the ablation is circular (a surrogate trained on FEM cannot be validated against FEM).
4. **Seam extraction from point clouds is a dead contribution.** Yi et al. (*Automation in Construction* 2026) do PointNet++ → multi-scale ICP → improved RANSAC → fillet path + torch pose on rusted real steel T-joints. Your perception stack is now **infrastructure, cite-and-move-on**, not contribution.
5. **Never frame the work as an extraction-accuracy comparison.** Their sensors: Mech-Eye NANO Ultra (0.07 mm @ 0.5 m), CSL (0.1 mm @ 0.5 m). Yours: RealSense-class, ~2.5–5 mm @ 1 m. You lose that fight on hardware alone.

---

## 3. Current plan (as of last conversation)

Advisor meeting (~3 h, last 10 min transcribed) went well. He was struck by how basic the published work is, that code is never released, and that every dataset is "available on reasonable request." He now wants a **dataset paper** and floated a three-paper arc:

1. Dataset (synthetic + real) + methodology + a trained baseline model
2. Actual tack welding operation
3. Community benchmark / call for others to run algorithms on it

**Three papers is his enthusiasm, not an obligation.** Build paper 1 to stand alone.

### The core insight (his, and it's the strongest thing in the meeting)

> *"Ground truth'ün point cloud'dan gelmesi yanlış. Çünkü o ground truth değil, o estimation."*

Every existing dataset is hand-annotated: LWSNet used CloudCompare manually; Yi et al. hand-labelled binary and defended it with **95% intra-rater consistency** (which measures *precision*, not accuracy — a biased annotator agrees with themselves perfectly); Lin et al. used an experienced gluer's teach-playback.

If two plates are placed at known pose, the seam is the **plane–plane intersection** — closed form, exact, no annotator. And because it's analytic, the point density is re-samplable on demand (his "10 dots per mm, or 50 if someone asks" point). No frozen dataset can do that.

**Critical: this only works if the generator code is public.** His own argument ("sen aynı kodları koşacaksın") requires it. Releasing the generator is the bigger differentiator than releasing the data.

### The killer experiment (not yet done)

Hand-annotate a subset of synthetic scenes the way LWSNet did, then measure the annotation against exact analytic truth. This yields **the label-noise floor of the entire existing literature**. If it lands near the ~0.6 mm RMSE these papers report, the field has been reporting accuracies below its own labelling error. One figure, potentially carries the paper. ~1 day of work.

### What to store vs. benchmark vs. regenerate

| Data | Store in release | Benchmark on | Regenerate from code |
|---|---|---|---|
| Part A / Part B point clouds, analytic seam GT | yes | yes | — |
| Tack points, joint type, camera pose | yes | yes | — |
| RGB, thickness / gap parameters | yes | **no** | — |
| Quality field, robot joint states, poses | no | no | yes |

- Advisor wanted RGB dropped entirely ("noise"). **Deviate mildly:** rendering is one flag, storing is cheap; not storing forecloses the 2D→3D line (SANet, Lin et al., Zhang's K-Net) without full regeneration. Store it, benchmark on point clouds only, say so explicitly. This is the ClearGrasp lesson — their auxiliary JSON is part of why the dataset outlived the paper.
- Advisor right to exclude quality field: it is robot-specific and **regenerable** from geometry + robot model. It belongs to the *method* paper, not the dataset. Not in conflict with your contribution.
- His minimal per-part label format happens to be exactly the input K-Net-style methods need (segment A, segment B, intersect masks). Not an accident of simplicity.

### Known risk to name before a reviewer does

Tack labels on a **straight** T-joint seam = endpoints + uniform spacing. A model predicting that learns arithmetic. The tack task only has content where placement is non-trivial: curved seams, geometric discontinuities, varying joint types — or robot-aware placement (the method paper). **Either include curved/corner geometry from the start, or scope paper 1 as seam-only and hold tacks for paper 2.**

---

## 4. The method contribution (separable from the dataset)

Three units, deliberately decoupled so unit 1 survives if 2 and 3 don't:

1. **Method** — automated tack placement + ordering. Validated on UR5e with pen marks; baselines = uniform spacing and geometry-only. Publishable alone (CASE / ETFA / IROS). **This is the thesis core.**
2. **Dataset** — Isaac Sim generation + real scans.
3. **Model** — RGB-D → tack points. Depends on 1 and 2.

### Novelty statement (survived all literature checks)

> Existing point-cloud weld path planners compute torch pose from workpiece geometry alone and generate continuous paths; we place **discrete** tack points and choose their poses using **robot kinematic conditioning under an eye-in-hand visibility constraint**.

Supporting gap, quotable: **LWSNet §5.5.1** states that in industry a human manually tacks before the robot welds (T-joints won't stand unwelded). The field's own framing is *human tacks, robot welds* — LWSNet's contribution is teaching a net to *skip* human tacks. Automated tack placement is absent because it's assumed manual.

### Stage 1 — quality field

Parameterize by arc length `s` and roll `φ` about the tool axis. Nominal orientation is free from the literature — cite it, don't reinvent:
- Yi et al.: `d_weld = n₁ × n₂`, approach `v₀ = v₁ + v₂` (dihedral bisector)
- Wang et al.: `Z_W = −(n_b + n_m)` — same construction

Then `R(s,φ) = R_nom(s) · Rot_z(φ)`. The free φ is where you depart from them.

Field components at each (s, φ):
- **IK feasibility** — hard gate; record which of the UR5e's 8 closed-form branches
- **σ_min(J)** — *not* |det J| (mixes m and rad units on a 6-DOF arm; physically meaningless)
- **Force transmission along contact normal**: `1/√(nᵀ (J_v J_vᵀ) n)`, translational block only. Ties to the admittance controller. Note in the paper: this is the *dual* of the velocity ellipsoid — manipulability work almost always optimises the wrong one for contact tasks.
- **Joint-limit margin**
- **Camera terms**: frustum containment, standoff within valid depth range, incidence angle away from grazing, self-occlusion by the pen holder
- **Collision clearance** — with camera mounted, widest geometry is likely the **camera body**, not the tool tip; T-joint vertical plate restricts the approach cone

Normalize each to [0,1]. **This vector is the robot-agnostic interface** — downstream (and any network) sees only numbers attached to 3D positions, never joint angles. Swap arms, field changes, architecture doesn't.

Grid cost: s @ 2 mm over 232 mm = 116; φ @ 5° = 72 → ~8,400 poses. Closed-form IK + Jacobian each; vectorized NumPy, sub-second.

Collapsing φ → one roll per arc position: **shortest path / DP over the (s,φ) grid** with transition penalty `μ|Δφ|` and a hard block on IK-branch changes. Exact, smooth roll profile.

### Stage 2 — selecting k tacks (corrected)

Earlier suggestion of greedy submodular maximization was **wrong for this problem** — candidates lie on a line, so 1D + spacing constraints is **exactly solvable by DP**:

```
f(i,j) = q(s_i) + max over m with d_min ≤ s_i − s_m ≤ d_max
                    [ f(m, j−1) − λ·g(s_m, s_i) ]
```

`g` penalises non-uniform spacing; because it's **pairwise** it stays inside the recursion (uniformity for free, no loss of exactness). Complexity O(n²k); n=116, k≤15 → ~200k ops, milliseconds, **global optimum not a bound**. Worth a sentence in the paper.

Hard constraints: both endpoints mandatory; `d_min ≤ Δs ≤ d_max`; no tack within ~2t of a free edge; IK feasible.

Choosing k: sweep it, or drop the j index and use a fixed per-tack cost c. Lower bound `k_min = ⌈L/d_max⌉ + 1`. Tomków (2020) measured that **five tacks gave higher angular distortion** than fewer — that's the citation for a real penalty on k.

Distortion literature enters **only** through `d_min`, `d_max`, edge margin, and `c`. All constants, all citable, no FEM.

### Stage 3 — ordering

Classical solver, **not** the network (permutation prediction = second hard research problem). Joint-space travel time (trapezoidal per-joint, duration = max_j t_j → asymmetric, not Euclidean) + **hard penalty on IK branch changes** (mid-sequence wrist flip is slow and unsafe near the workpiece; this constraint often dominates) + wrist-3 wrap + low-weight distortion priors.

### Model (if pursued)

- **Point-based, best fit:** PointNet++ / LWSNet-style, per-point score head. Conditioning is clean — input feature vector per point becomes `[x,y,z, nx,ny,nz, q₁…q₅]`. PointNet++ handles extra channels natively; LWSNet already feeds normals this way. Small, defensible delta from a known architecture.
- **Image-based baseline:** U-Net heatmap regression + NMS + back-projection; FiLM conditioning on pooled quality field. Heatmaps >> direct coordinate regression.
- **Ablation with real content:** geometry-only vs. geometry + quality-field channels.
- **Not** an autoencoder (reconstruction objective spends capacity on surface detail you don't need). **Not** a transformer at this data scale.
- Anti-circularity framing that must be explicit: the labeler consumes exact CAD registration + kinematics; the network consumes a raw RGB-D frame. It's **distillation of a geometry+kinematics rule into a perception model** — useful precisely because CAD registration isn't available at runtime. Leave implicit → reviewer kills it.

---

## 5. Literature map

### Dead / occupied (cite, don't compete)
- **Yi et al. 2026**, *Automation in Construction* 183:106792 — PointNet++ + multi-scale ICP + improved RANSAC, fillet path + torch pose, real rusted steel T-joints. 800 samples / 4 workpieces → 4800 augmented. ME 1.16 mm, RMSE 0.64 mm. Data on request.
- **LWSNet (Song et al. 2025)**, *Measurement* 243:116290 — lightweight PointNet++ (DSConv + NAM + linear bottleneck). **94 raw clouds** → 658 total. T/lap/butt/corner. §5.5.1 = your gap quote. Data on request.
- **Wang et al. 2026**, *Welding in the World* — YOLO11 + improved DeepLab V3+ on intersecting pipes; PCA slicing; MSAC-WTLSD torch posture. 750 images → 1000. Accuracy <1 mm. Not public.
- **Lin et al. 2026**, *IEEE/ASME T-Mech* 31(1):639 — coating seams; cascaded U-Net thick/thin masks + 3D-SRTN (SFRM + APTM) + 6-DoF post-processing. **13 workpieces, 1690 groups, 31.29 h on one RTX 3080, UR5e.** RMSE 0.593/1.099/1.252 mm. Has a **public project page** — the rare exception. Records `ᴮT_C = ᴮT_E · ᴱT_C`.
- **SANet (Zhu et al. 2025)**, *Applied Sciences* 15:11296 — RGB-only 2D segmentation, 7-channel input, 4000+ images. **No depth, no 3D, no robot.** Its §5.3 future work asks for depth integration, synthetic data, and robotic path planning = your thesis, written by someone else as an open direction. Note: HintUNet ties its headline IoU/Dice exactly; ablation table numbers don't match the comparison table; sensitivity 18.3%. MDPI Q2. Data not public.
- **Liu, Tang, Tian & Yang 2023**, *RCIM* 83:102549 — **NSGA-II offline programming for multi-pipe intersections.** *Correction to an earlier claim in the conversation: this paper DOES put the robot in the loop* — energy (joint motor power × displacement), cable twist (variance of J4/J6 rotation), posture-adjustment count, cylinder-based collision checking. **Cite prominently**; its three-objective structure is a good template. What it still doesn't do: position along seam is fixed (adaptive discretization bounds chord error — interpolation, not selection); no conditioning metric (energy ≠ manipulability); offline, no camera during execution → no visibility constraint; no ordering.

### Shared blind spots across all of them
- Torch pose is **pure geometry** (dihedral bisector / dominant normal + standoff). Only place kinematics appears anywhere: Yi et al. avoid near-singular configurations during **TCP calibration** — a manual setup precaution, i.e. an admission it matters in the one place they didn't build it in.
- **Nothing is discrete.** NURBS, cubic splines, equidistant waypoints, least-squares fitting throughout. "Which finite subset of points, in what order" does not exist.
- **Every dataset private.** LWSNet, Yi, Wang, Zhang (K-Net, 316 images), WeldNet, PAEAR, YOLO-LaserGalvo, tubesheet — all "on request" or explicitly not public. Lin et al.'s project page is the exception. This is now a well-evidenced claim, not a hunch.

### Distortion / ordering background (for constants and Chapter 2 only)
Tsai, Park & Cheng 1999 *Welding Journal* (Joint Rigidity Method — weld most rigid joint first; buckling not a concern above 1.6 mm skin); Park & An 2016 (JRM → fillet, ~10 mm → ~5 mm); Satoh restraint intensity K = E·t/L; Okerblom / White tendon force `F = k·Q_net`, k ≈ 160–200 N per J/mm; Ueda & Murakawa inherent strain/deformation; Rosenthal 2D/3D closed-form thermal field; Zhao & Zaeh (TU München iwb) WAAM semi-analytical superposition + interpass-temperature minimax + "thermal eccentricity" — the closest computational analogue to tack ordering; Tomków, Sobota & Krajewski 2020 (*Facta Universitatis*) — the only recent tack-sequence paper, 16.06% angular distortion reduction, 5 tacks worst.

### German angle (for the PhD move)
TU München iwb (Zaeh, Zhao, Panzer — closest niche fit), BAM Berlin (Bachmann, Rethmeier), Fraunhofer IPK (Biegler). Standards: DVS 1710 (Schweißfolgeplan), EN ISO 13920, EN 1090.

---

## 6. Immediate next actions

1. **Build the quality field and plot q(s) across the seam before writing any selection code.** If σ_min and force transmission vary <~10% over 232 mm in comfortable workspace, the kinematic half has no signal — better to know in week 2 than week 10. **Expectation: conditioning terms will be fairly flat; camera visibility and collision will carry the variation** (the vertical plate is a hard geometric block). If so the framing shifts to "visibility-and-reachability-aware," which is arguably more distinctive anyway. Plan for it now.
2. **Mount the camera and measure the actual feasible φ range against the vertical plate.** If narrow, the 5° grid may be too coarse. Check the depth sensor's *minimum* range too — you may be inside the blind zone for whole bands of φ.
3. **Fix the occlusion bug:** currently sampling every CAD face. Don't hand-roll ray casting — use a renderer for occlusion-correct depth, or `trimesh.ray` outside Isaac Sim. Generate **both** variants (full 3D and single-view) from the same scene; they're different benchmark conditions.
4. **Run the annotation-error experiment** (§3) — cheap, high payoff.
5. Decide the joint-type scope before building the generator (T-only vs. T + corner + curved). Trivial-label risk (§3) makes curved/corner worth including if tacks are in paper 1.
6. Get the no-physical-welding scope confirmation **in writing** from the advisor — he skipped the question once already.

**Transcription note:** "synchron distance" in the meeting transcript is almost certainly **Sinkhorn** (optimal transport — matches his "transport cost"). Search that. Chamfer distance is the cheap standard and should be the primary reported metric; Sinkhorn/EMD is more principled and much slower. İsmail's PhD thesis reportedly has related material.

---

## 7. Standing notes on how to use this

- Recurring failure mode: reaching for a bigger contribution when the honest move is narrower. It has happened three times (FEM → DL-on-FEM → foundation model). If a new idea feels exciting because it's *big*, that's the signal to check it.
- Calibration point: the T-Mech coating paper is 13 workpieces, one RTX 3080, LabelMe annotation, 31 hours of training, a UR5e. The publication bar is lower than assumed.
- Published work is not airtight (see SANet's internally inconsistent tables). Your own standard is already higher.
- Deadline pressure: advisor said "pazartesiye kadar gömül bunlara" — get immersed and report back.
