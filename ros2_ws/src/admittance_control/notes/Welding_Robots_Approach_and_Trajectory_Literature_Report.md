# State-of-the-Art Literature Report: Robotic Welding Approach Motion, Torch Orientation Planning, and Collision Avoidance in 6D Registered Workspaces

**Author:** Antigravity AI (Prepared for Master's Thesis Research)  
**Date:** September 2026  
**Target Environment:** 6-DOF Manipulator (e.g., Universal Robots UR5e / ABB IRB), Eye-in-Hand / Eye-to-Hand RGB-D Vision, 6D CAD-to-Scan Registration, Metal Workpieces with Weldable Edges and Discrete Tack Locations  
**File Location:** `/media/cago/CAGO_32G/Tez related makaleler/Welding_Robots_Approach_and_Trajectory_Literature_Report.md`

---

## 1. Executive Summary & Problem Scope

In automated robotic arc and tack welding, bridging the gap between **perceptual registration** (estimating the 6D pose of CAD models in RGB-D scans or reconstructing 3D point clouds) and **safe physical execution** is one of the most critical engineering challenges. Once weldable seam lines and discrete tack locations are identified in the robot's workspace, two interdependent geometric and kinematic problems must be solved before striking an arc:

1. **How to define the torch orientation (posture) along the seam or at tack locations?** While taking the angle bisector of the dihedral angle between adjacent plates is the classic geometric baseline, industrial and academic literature demonstrates that a naive bisector is often insufficient due to thermal mass asymmetries, gravitational weld pool flow, torch neck interference, joint limits, cable twisting, and kinematic singularities.
2. **How to make the robot arm approach the region of interest (ROI) and weldable line without collision?** The robot must traverse free space cluttered by metal workpieces, clamps, fixtures, and adjacent sub-assemblies, transition from high-speed motion into the confined groove, and reach the arc strike point safely.

This report synthesizes the state of the art in robotic welding literature—drawing deeply from the specialized papers in this repository (e.g., *Fang & Tian 2024*, *Liu et al. 2023*, *Geng et al. 2022*, *Peng et al. 2020*, *Jia et al. 2024*, *Zhou et al. 2022*, *Wang et al. 2024*, *Ramsey et al. 2023*, *Ahmed et al. 2018*, *Bedaka et al. 2019*, and *Tomków et al. 2020*) as well as modern industrial offline programming (OLP) frameworks.

```
+----------------------------------------------------------------------------------------------------+
|                                    END-TO-END SYSTEM PIPELINE                                      |
+----------------------------------------------------------------------------------------------------+
|  [RGB-D / 3D Vision Scan] + [Nominal CAD Models]                                                   |
|                         │                                                                          |
|                         ▼                                                                          |
|  [6D Pose Estimation & Registration] (Scan2CAD / FPFH / ICP / FoundationPose)                     |
|                         │                                                                          |
|                         ▼                                                                          |
|  [Seam & Tack Extraction] (CAD B-Rep Topology Traversal / 3D Point Cloud Multi-Plane Intersection)  |
|                         │                                                                          |
|                         ▼                                                                          |
|  [Torch Orientation Planning]                                                                      |
|  ├─ Dihedral Angle Bisector Baseline (Geometry)                                                    |
|  ├─ Work Angle (Asymmetric Thickness) & Travel Angle (Push/Pull Metallurgy)                         |
|  ├─ Gravity Alignment (Downhand Pool Retention)                                                   |
|  ├─ Curvature-Continuous Moving Frames (RMF / Bishop Frame along 3D Space Curves)                  |
|  └─ 1-DOF Tool Roll Redundancy Resolution (Singularity / Clearance / Cable Optimization)           |
|                         │                                                                          |
|                         ▼                                                                          |
|  [Hierarchical Collision-Free Motion Planning]                                                      |
|  ├─ Phase 1: Global Free-Space Transit (OMPL: RRT* / Bi-RRT / Lazy-PRM / TrajOpt)                 |
|  ├─ Phase 2: Local Linear Approach (Cartesian LIN along Approach Vector from Standoff via-point)   |
|  ├─ Phase 3: Weld Process / Tack Arc Dwell                                                         |
|  └─ Phase 4: Local Linear Retract (Cartesian LIN back to safe clearance)                          |
|                         │                                                                          |
|                         ▼                                                                          |
|  [Physical Robot Execution & Collision Scene] (MoveIt2 / ROS 2 / FCL Mesh-Capsule Checking)       |
+----------------------------------------------------------------------------------------------------+
```

---

## 2. Torch Orientation Determination: Beyond the Dihedral Angle Bisector

### 2.1 The Dihedral Angle Bisector Baseline

In structural fabrications (T-joints, lap joints, V-grooves, corner joints, and pipe intersections), the seam line is defined by the intersection of two adjacent surface manifolds $S_1$ and $S_2$. 

Let $\mathbf{p} \in \mathbb{R}^3$ be a path point along the seam, and let $\mathbf{n}_1, \mathbf{n}_2 \in \mathbb{R}^3$ denote the unit outward normal vectors of surfaces $S_1$ and $S_2$ evaluated at $\mathbf{p}$. The dihedral angle $\phi$ between the two plates is given by:

$$\cos \phi = \mathbf{n}_1 \cdot \mathbf{n}_2$$

The unit tangent vector along the weld seam is:

$$\mathbf{t} = \frac{\mathbf{n}_1 \times \mathbf{n}_2}{\|\mathbf{n}_1 \times \mathbf{n}_2\|}$$

The **symmetric dihedral angle bisector** vector $\mathbf{b}$ points directly into or out of the groove:

$$\mathbf{b} = \frac{\mathbf{n}_1 + \mathbf{n}_2}{\|\mathbf{n}_1 + \mathbf{n}_2\|}$$

In standard practice (*Geng et al. 2022*, *Yi et al. 2024*, *Peng et al. 2020*), the torch tool coordinate frame $\{T\}$ at path point $\mathbf{p}$ is initially aligned such that its tool approach axis (conventionally the tool $+Z$ or $-Z$ axis) is collinear with $\mathbf{b}$, while the tool travel axis is aligned with $\mathbf{t}$.

#### Why the Naive Bisector Fails in Real-World Cells:
1. **Asymmetric Thermal Dissipation:** If plate 1 is 12 mm thick and plate 2 is 3 mm thick, heat conducts much faster into plate 1. A 45° symmetric bisector causes severe undercut on the thin plate and lack of fusion on the thick plate.
2. **Gravitational Runoff:** In horizontal fillet welds (2F position) or vertical welds (3F), gravity causes molten metal to sag downward. The torch must be tilted against gravity.
3. **Mechanical Interference:** Symmetrically dividing the angle often points the bulky gas cup ($D \approx 16-25\text{ mm}$) directly into an overhanging web, stiffener flange, or hold-down clamp.
4. **Normal Degeneration at Boundaries:** When normals are computed from discrete 3D point clouds (*Peng et al. 2020*, *Fang & Tian 2024*), points near edges suffer from missing returns, mixed pixels, and edge-rounding artifacts, causing high-frequency angular jitter.

---

### 2.2 What Else Does the Literature Do? (Advanced Orientation Strategies)

To overcome the limits of the naive dihedral bisector, literature introduces several advanced techniques:

#### 1. Process-Governed Work and Travel Angles (Push vs. Drag Metallurgy)
Industrial welding standards (AWS D1.1, ISO 5817, EN ISO 13920) require distinct **Work Angles** ($\theta_w$) and **Travel Angles** ($\theta_t$):
- **Work Angle ($\theta_w$):** Angle between the torch axis and the base plate in the cross-sectional plane perpendicular to the weld seam. While $45^\circ$ is standard for equal-thickness T-joints, it is actively shifted in asymmetric joints.
- **Travel Angle ($\theta_t$):** Angle between the torch axis and the normal plane in the longitudinal direction along the seam tangent $\mathbf{t}$.
  - **Drag (Pull / Backhand) Angle ($5^\circ - 15^\circ$):** The torch points backward toward the deposited bead. Deepens penetration and narrows bead width; standard for medium-to-thick steel plates (GMAW / FCAW).
  - **Push (Forehand) Angle ($5^\circ - 15^\circ$):** The torch points forward along the travel direction. Creates a shallower, wider penetration profile; standard for thin sheet metal, stainless steel, and aluminum (GMAW / GTAW).

Mathematically, the nominal tool orientation matrix $\mathbf{R}_{tool} \in SO(3)$ is formed by rotating the bisector frame $\{S\}$ by work angle $\theta_w$ around $\mathbf{t}$ and by travel angle $\theta_t$ around the lateral normal axis $\mathbf{y}_s = \mathbf{b} \times \mathbf{t}$:

$$\mathbf{R}_{tool} = \mathbf{R}(\mathbf{t}, \theta_w) \cdot \mathbf{R}(\mathbf{y}_s, \theta_t)$$

#### 2. Asymmetric Tilt for Unequal Plate Thickness & Dissimilar Metals
In shipbuilding and structural steel fabrication (*Wang et al. 2024*, *Geng et al. 2022*), T-joints and stiffened panels frequently couple plates of disparate gauges.
- Literature applies an empirical or analytical heat-distribution offset angle $\Delta \theta$:

$$\theta_w = \frac{\phi}{2} + \Delta \theta, \quad \text{where } \Delta \theta = f\left(\frac{k_1 \rho_1 c_{p1} h_1}{k_2 \rho_2 c_{p2} h_2}\right)$$

Here, $h_i, k_i, \rho_i, c_{pi}$ represent plate thickness, thermal conductivity, density, and specific heat capacity. For steel plates with thickness ratio $h_1 / h_2 > 2$, the work angle is shifted $5^\circ - 12^\circ$ toward the thicker plate to balance fusion depth.

#### 3. Gravity-Compensated Posture Planning (Downhand Pool Retention)
In multi-axis welding of spatial 3D seams (such as intersecting pipes, *Liu et al. 2023*, *Yan et al. 2019*), the seam orientation changes continuously with respect to the world gravity vector $\mathbf{g} = [0, 0, -1]^T$.
- When welding horizontal fillets (2F), the torch is tilted upward by $5^\circ - 10^\circ$ relative to the bisector to counteract gravity-induced puddle sag.
- When welding vertical seams, torch posture dynamically transitions between vertical-up ($10^\circ - 15^\circ$ push angle) and vertical-down ($10^\circ - 15^\circ$ drag angle).
- In coordinated systems (robot arm + 2-axis tilt-rotary positioner), posture planning algorithms optimize positioner angles so that the weld pool tangent $\mathbf{t}$ is kept horizontal and the bisector $\mathbf{b}$ is kept antiparallel to gravity ($\mathbf{b} \parallel -\mathbf{g}$), enforcing flat (1G/1F downhand) welding.

#### 4. Curvature-Continuous Moving Frames Along 3D Space Curves (RMF vs. Frenet)
For complex 3D curved seams (e.g., saddle curves in pipe-pipe intersections, multi-blade impellers, freeform automotive pressings), moving frames defined by classical differential geometry (Frenet-Serret) fail catastrophically:
- At points of inflection or straight segments where curvature $\kappa \to 0$, the Frenet normal and binormal vectors become undefined, causing the torch orientation to undergo violent $180^\circ$ wrist flips and triggering robot kinematic joint limit/speed errors.
- **Literature Solution:** Modern studies (*Liu et al. 2023*, *Fang & Tian 2024*, *Guo et al. 2024*) implement **Rotation-Minimizing Frames (RMF)** (also known as **Bishop Frames**) or geodesic parallel transport along the seam curve. The frame is integrated along the path such that the angular velocity vector of the frame has zero component along the tangent $\mathbf{t}$:

$$\mathbf{u}'(s) = -(\mathbf{t}'(s) \cdot \mathbf{u}(s)) \mathbf{t}(s)$$

This ensures that the torch orientation evolves with minimal rotation and zero torsional twisting along the seam.

#### 5. Exploiting 1-DOF Torch Symmetry (Kinematic Redundancy Resolution)
A conventional arc welding torch (GMAW/GTAW) is geometrically and physically **axisymmetric** about its electrode / gas cup centerline ($\mathbf{z}_{tool}$). 
- To position the torch tip at point $\mathbf{p} = [x, y, z]^T$ and align the electrode along unit vector $\mathbf{v}_{torch}$, only **5 degrees of freedom** are functionally constrained.
- A standard 6-DOF articulated robot (such as UR5e, ABB IRB2600, KUKA KR16, Fanuc ARC Mate) therefore has **1 degree of kinematic redundancy** ($\phi_{roll} \in [0, 2\pi)$) around the tool axis!

Literature exploits this redundancy parameter $\phi_{roll}$ for:
- **Collision Avoidance:** Rotating the robot's wrist assembly and upper elbow links away from clamps, fixtures, and adjacent metal walls without altering the torch tip position or work/travel angles.
- **Singularity Avoidance:** Adjusting $\phi_{roll}$ to keep wrist joints (specifically Joint 5 on 6-axis spherical-wrist robots) away from $0^\circ$ alignment, preventing infinite joint velocity demands.
- **Manipulability Maximization:** Maximizing Yoshikawa's manipulability measure $w = \sqrt{\det(\mathbf{J}(\mathbf{q}) \mathbf{J}^T(\mathbf{q}))}$ or distance from mechanical joint limits:

$$\max_{\phi_{roll}} \sum_{i=1}^6 \left(\frac{q_i - q_{i,mid}}{q_{i,max} - q_{i,min}}\right)^2$$

- **Cable Twist Minimization:** Arc welding dress packs (power cables, shielding gas hoses, water cooling lines, wire conduit) are stiff. *Liu et al. (2023)* formulated a dedicated objective function $G_{wnd}$ in NSGA-II to minimize the cumulative cable torsion $\sum |\Delta q_4 + \Delta q_6|$, preventing wire feed motor stalling and cable fatigue failure.

#### 6. Admissible Orientation Tolerance Cones & Multi-Objective Optimization
Rather than treating torch orientation as an inflexible rigid constraint, metallurgical studies demonstrate that acceptable weld penetration and bead appearance are preserved within an **admissible orientation tolerance cone** $\mathcal{C}_{allowable}$ (typically $\pm 5^\circ$ to $\pm 10^\circ$ in work and travel angles).
- *Liu et al. (2023)*, *Wang et al. (2022)*, and *Jia et al. (2024)* formulate torch posture planning as a constrained multi-objective optimization problem:

$$\min_{\mathbf{q}} \left[ f_{collision}(\mathbf{q}), \; f_{energy}(\mathbf{q}), \; f_{cable}(\mathbf{q}) \right]$$

$$\text{subject to: } \angle(\mathbf{z}_{tool}(\mathbf{q}), \mathbf{b}_{nominal}) \le \theta_{cone}^{max}, \quad \mathbf{q}_{min} \le \mathbf{q} \le \mathbf{q}_{max}, \quad d_{min}(\text{Robot}, \text{Obstacles}) \ge d_{safe}$$

Solving this using NSGA-II, SQP, or PSO allows the planner to slightly tilt the torch within metallurgically safe bounds whenever a clamp or fixture would otherwise cause a hard collision.

#### 7. CAD-Derived Auxiliary Guide Curves (*Fang & Tian 2024*)
In vision-guided welding where 3D point clouds suffer from occlusion, noise, and edge degradation at workpiece corners, *Fang & Tian (2024)* introduce the **Auxiliary Trajectory Method**:
- In the nominal CAD model, an auxiliary curve is generated parallel to the seam at an offset $d_{aux} \approx 5-10\text{ mm}$ on one of the adjacent faces.
- When the CAD model is registered to the physical scan via non-rigid/rigid point set registration (e.g., GMM / Coherent Point Drift), the auxiliary curve maps directly to the physical workpiece.
- The vector connecting each seam point $\mathbf{p}_i$ to its corresponding auxiliary point $\mathbf{p}'_{aux,i}$ establishes a noise-free, structurally grounded normal reference plane, bypassing local point cloud noise and boundary point loss.

---

## 3. Collision-Free Approach Trajectory Generation

Reaching a weldable line or tack location requires navigating from a global configuration into a narrow, highly constrained geometric region. 

```
                                  [ HOME / RETRACTED POSE ]
                                              │
                                              │  Phase 1: Global Free-Space Motion
                                              │  (Joint Space RRT* / Bi-RRT / TrajOpt)
                                              │  High speed, large obstacle clearance
                                              ▼
                               [ PRE-WELDING APPROACH POINT (P_app) ]
                                              │
                                              │  Phase 2: Local Linear Approach
                                              │  (Cartesian LIN along v_app)
                                              │  d_standoff = 20 - 100 mm, reduced speed
                                              ▼
    [METAL PART A] ───►   \       * [P_arc_start]       /   ◄─── [METAL PART B]
                           \___________________________/
                                  [ WELD GROOVE ]
                                        │
                                        │  Phase 3: Arc Welding / Tack Dwell
                                        ▼
                                  [P_arc_end]
                                        │
                                        │  Phase 4: Local Linear Retract
                                        │  (Cartesian LIN along -z_tool)
                                        ▼
                                [ RETRACT POINT (P_ret) ]
```

### 3.1 The Canonical Four-Phase Motion Hierarchy

Both modern industrial OLP engines (Robotmaster, Verbotics Weld, ROBCAD, CypWeld) and academic research decompose robotic welding paths into a strict four-phase motion hierarchy:

| Phase | Motion Type | Controller Domain | Velocity Profile | Primary Objective & Constraints |
| :--- | :--- | :--- | :--- | :--- |
| **1. Global Transit / Transfer** | Free-space point-to-point | Joint space ($\mathcal{C}$-space, $\mathbb{R}^6$) | High ($500 - 1500\text{ mm/s}$) | Global obstacle avoidance against cell fixtures, workpiece CAD, and clamps via RRT*, Bi-RRT, Lazy-PRM, or TrajOpt. |
| **2. Local Approach** | Straight-line path (LIN) | Cartesian space ($SE(3)$) | Moderate/Slow ($50 - 150\text{ mm/s}$) | Controlled penetration into groove along the **Approach Vector** from standoff distance $d_{standoff}$; fixed/smooth orientation. |
| **3. Process Execution** | Path tracking / Dwell | Cartesian space ($SE(3)$) | Process speed ($3 - 12\text{ mm/s}$) or Dwell ($0.5 - 2\text{ s}$) | Arc welding seam following or tack spot melting; tight geometric and orientation adherence. |
| **4. Local Retract** | Straight-line path (LIN) | Cartesian space ($SE(3)$) | Moderate ($100 - 200\text{ mm/s}$) | Linear withdrawal along $-\mathbf{z}_{tool}$ or surface normal bisector back to safe clearance altitude before Phase 1. |

#### Why Pure Joint-Space Planning Directly into the Groove Causes Collisions:
If a motion planner attempts to plan a joint-space trajectory (e.g., standard MoveIt `move_group.plan()`) directly from a home pose into the arc strike point inside a V-groove or T-joint:
- In joint space, interpolation between joint angles $\mathbf{q}_{start}$ and $\mathbf{q}_{goal}$ produces a **curved, non-linear Tool Center Point (TCP) trajectory in Cartesian space**.
- The torch tip will swing laterally or "whip" its elbow/wrist, causing the gas nozzle ($D \approx 20\text{ mm}$) to collide with the vertical plate, the groove sidewall, or adjacent tack beads.
- **Literature Axiom:** The final approach motion into the groove MUST be executed as a **Cartesian Linear Path (LIN)** along an approach vector!

---

### 3.2 Definition of the Approach Vector ($\mathbf{v}_{app}$) and Standoff Via-Point

The Pre-Welding Approach Point $\mathbf{P}_{app} \in \mathbb{R}^3$ is parameterized by a standoff distance $d_{standoff}$ and a unit approach direction vector $\mathbf{v}_{app}$:

$$\mathbf{P}_{app} = \mathbf{P}_{arc\_start} + d_{standoff} \cdot \mathbf{v}_{app}$$

Literature uses three distinct methods to establish $\mathbf{v}_{app}$:

#### 1. Collinear Tool-Axis Approach ($\mathbf{v}_{app} = \mathbf{z}_{tool}$)
The standoff point is placed directly along the tool's centerline axis:
- **Advantage:** The robot maintains a completely constant orientation during Phase 2. The physical motion is a pure 1D translation along the torch nozzle axis.
- **Usage:** Standard across most OLP systems and commercial robotic cells (*Peng et al. 2020*, *Jia et al. 2024*).
- **Typical Standoff Distance:** $d_{standoff} = 20\text{ mm}$ to $50\text{ mm}$ for small assemblies; $50\text{ mm}$ to $100\text{ mm}$ for deep web structures.

#### 2. Surface Normal Bisector Approach ($\mathbf{v}_{app} = \mathbf{b}$)
The standoff point is placed along the dihedral angle bisector:
- **Advantage:** Maximizes clearance from both sidewalls of a deep V-groove or 90° fillet joint during insertion.
- If the welding process requires a non-zero travel angle (e.g., $10^\circ$ push/drag), the orientation is smoothly interpolated from a neutral bisector orientation at $\mathbf{P}_{app}$ to the required process tilt at $\mathbf{P}_{arc\_start}$.

#### 3. Obstacle-Gradient Repulsive Approach (Visibility Cone Optimization)
In cluttered fixtures where clamps or upper ribs obstruct the direct collinear line:
- The torch approach vector $\mathbf{v}_{app}$ is computed by finding the central ray of the **Torch Accessibility Cone (Shadow Cone)**.
- A ray-casting or spherical distance field query is evaluated on the workpiece CAD mesh centered at $\mathbf{P}_{arc\_start}$. The collision-free solid angle $\Omega_{free} \subset S^2$ is computed, and $\mathbf{v}_{app}$ is set to the center of mass of $\Omega_{free}$.

---

### 3.3 Collision Modeling & Geometric Representations

To verify collision-free motion across both global transit and local approach phases, the literature utilizes specialized geometric representations for the robot, torch, and environment:

```
                  [ ROBOT ARM BOUNDING CAPSULES ]
                     (Link 1 ... Link 6 Capsules)
                                  │
                                  ▼
                    [ TORCH COMPOUND MODEL ]
                    ├─ Torch Body (Cylinder / OBB)
                    ├─ Angled Gooseneck (22° / 45° Torus / Cylinder)
                    ├─ Gas Cup / Nozzle (Cylinder / Truncated Cone)
                    └─ Wire Electrode (Line Segment / Ray)
                                  │
                    Distance Checking: d(Robot, Mesh) >= d_safe
                                  │
                                  ▼
           [ WORKSPACE & WORKPIECE COLLISION GEOMETRY ]
           ├─ Registered CAD Surface Meshes (FCL / Bullet)
           ├─ Fixture & Clamp Bounding Boxes / Convex Hulls
           ├─ Environmental Octree / OctoMap (Sensor Data)
           └─ SIMD Collision-Affording Point Trees (CAPT, Ramsey et al. 2023)
```

1. **Robot & Torch Geometry:**
   - *Liu et al. (2023)* model an ABB IRB2600 arm + welding torch as **8 enclosing bounding cylinders/capsules**. Segment-to-segment minimum Euclidean distance formulas are solved in closed form at millisecond rates.
   - For high-precision clearance, the torch is modeled as a compound body:
     - **Gas Nozzle / Cup:** Modeled as a truncated cylinder/cone ($D_{base} \approx 25\text{ mm}, D_{tip} \approx 16\text{ mm}, L \approx 60\text{ mm}$).
     - **Gooseneck:** Angled bent cylinder (typically bent at $22^\circ$ or $45^\circ$).
     - **Torch Body & Cable Connector:** Enclosing cylinder or Oriented Bounding Box (OBB).
2. **Workpiece & Clamping Environment:**
   - **Registered CAD Meshes:** Once the 6D pose of the CAD model is registered to the scan, the exact CAD triangle mesh is loaded into collision engines like FCL (Flexible Collision Library) within MoveIt / ROS.
   - **Convex Decomposition (V-HACD):** Non-convex structural metal parts are decomposed into sets of convex hulls, accelerating distance and collision checks by $10\times - 50\times$.
   - **OctoMap / Voxel Grids:** Used when unregistered obstacles (cables, table edges, operator fixtures) are present in the RGB-D sensor field of view.
   - **SIMD Collision-Affording Point Trees (CAPT, *Ramsey, Kingston, Thomason, Kavraki, IROS 2023*):**
     Direct point-cloud collision checking is historically slow. *Ramsey et al.* introduced CAPT, an explicit spatial data structure optimized with AVX-512 SIMD parallelism. CAPT performs nearest-neighbor distance queries directly against raw unorganized sensor point clouds in under $10\text{ }\mu\text{s}$, enabling real-time collision-checked motion planning in MoveIt without requiring surface reconstruction.

---

### 3.4 Motion Planning Algorithms in Current Literature

As documented in comprehensive reviews (*Key technologies and latest research progress of automated robotic welding: a review, 2024*; *Zhou et al. 2022*; *Wang et al. 2024*):

#### 1. Sampling-Based Motion Planning (RRT* / Bi-RRT / Lazy-PRM)
- **RRT-Connect / Bi-RRT:** The standard default for global transit in MoveIt / OMPL. Rapidly finds feasible paths between home and approach points by growing two trees from start and goal configurations.
- **Lazy-PRM / Lazy-PRM-TP (*Zhou et al. 2022*, *Wang et al. 2022*):** Constructing dense PRM roadmaps in welding cells is expensive because collision checking dominates CPU time. Lazy-PRM delays collision checking until an optimal graph path is found, evaluating collisions only along candidate edges. This reduces path generation time by over $60\%$.
- **Informed RRT* / Improved RRT* (*Wang et al. 2024*):** Applied to gantry welding robots in shipbuilding. Once an initial path is found, the search space is focused into a hyper-ellipsoid, pruning suboptimal nodes and optimizing path length and joint smoothness.

#### 2. Optimization-Based Trajectory Generation (TrajOpt, CHOMP, STOMP)
- Rather than sampling randomly, trajectory optimization formulates path planning as continuous non-linear optimization:
  - Objective: Minimize joint accelerations, execution time, and proximity to obstacles.
  - Obstacles are represented as Signed Distance Fields (SDF). The optimizer pushes the robot trajectory out of collision while enforcing kinematic joint limits and torch orientation constraints.
  - *TrajOpt* is particularly favored for welding because it can handle hard kinematic equality and inequality constraints (such as keeping the torch axis inside an admissible orientation cone).

#### 3. Reinforcement Learning & Neural Planners (*Petrik & von Lukas 2023*, *D3QN*)
- Recent literature explores Deep Reinforcement Learning (e.g., Deep Double Dueling Q-Networks - D3QN) for autonomous obstacle avoidance in gantry and multi-axis welding cells. While promising for dynamic environments, industrial production still overwhelmingly relies on deterministic sampling/optimization planners (OMPL/TrajOpt) due to formal safety and predictability guarantees.

---

## 4. Tack Welding Specific Trajectory & Sequence Optimization

When the defined features are discrete **tack locations** rather than continuous seams, the planning problem shifts into combinatorial optimization:

### 4.1 Combinatorial Sequence Optimization (TSP / GTSP Formulation)

For a workpiece with $N$ discrete tack welds ($P_1, P_2, \dots, P_N$), the robot must visit and weld each point before returning to home.
- *Jia, Pan, Zhang, Yang, Chen, Chen (2024)*, *"Robotic Tack Welding Path and Trajectory Optimization Using an LF-IWOA"*, formulate tack path planning as a Travelling Salesperson Problem (TSP) variant with robot dynamic constraints (joint velocity, acceleration, and jerk limits).
- They introduce a **Lévy Flight-Enhanced Improved Whale Optimization Algorithm (LF-IWOA)** combining elite opposition-based learning and differential evolution to minimize total cycle time, path length, and energy consumption.
- Between tack points, straight-line Cartesian motions with via-points are planned to ensure repeatable, collision-free transfers.

### 4.2 Distortion-Aware Tack Ordering

While computer science path planners traditionally minimize travel distance ($\min \sum \|\mathbf{P}_{\pi(i)} - \mathbf{P}_{\pi(i+1)}\|$), **welding physics dictates that the shortest travel sequence is almost always the worst sequence for thermal distortion!**

As established in the thesis investigation document (*"Analytical & Heuristic Methods for Ordering Tack Welds to Minimize Distortion: A Route-by-Route Investigation"*, citing *Tomków et al. 2020*, *Murakawa et al.*, *Schenk & Heinze 2011*, and *Tsai 1999*):
1. **Thermal Distortion Physics:**
   - Placing consecutive tacks adjacent to each other concentrates heat, reduces local plate yield strength, and causes massive angular distortion (transverse bending $\theta_y$) and root gap closure.
   - *Tomków et al. (2020)* experimentally proved that "welding away from the center" (skip-tacking / alternating outwards) reduced angular distortion by **16.06%** compared to sequential tacking.
2. **The Joint Rigidity Method (JRM, Tsai 1999):**
   - The optimal sequence tacks the **most rigid / most restrained joint segment first**, progressing toward less rigid segments.
3. **The Literature-Backed Multi-Objective Cost Function:**
   For a permutation $\pi \in S_N$, the literature supports a scalar cost function balancing travel time and distortion:

$$J(\pi) = w_{travel} \cdot \frac{T_{travel}(\pi)}{T_{max}} + w_{dist} \cdot \frac{J_{distortion}(\pi)}{J_{dist,max}} + w_{coll} \cdot J_{collision}(\pi)$$

Where $J_{distortion}(\pi)$ includes:
- A **Heat-Accumulation Penalty** (analytical Rosenthal solution or exponential decay $e^{-\Delta t / \tau} / \|\mathbf{P}_i - \mathbf{P}_j\|$) penalizing consecutive tacks that are close in space and time.
- An **Inherent-Deformation Rigidity Term** based on structural stiffness.

### 4.3 Jump Trajectories (Arch / Door-Frame Transfers)

Between discrete tack points located on the same metal plate, running a full 2-second global RRT* search for every transfer is computationally wasteful.
- Industrial systems implement **Arch / Door-Frame Via-Point Transfers**:
  1. Retract vertically along $-\mathbf{z}_{tool}$ to safe transit altitude $Z_{safe}$ ($20-50\text{ mm}$ above the highest clamp/flange).
  2. High-speed Cartesian or joint-space transfer horizontally across to the next tack's $(X, Y)$ coordinate at $Z_{safe}$.
  3. Controlled linear approach straight down into the next tack position along its approach vector.

---

## 5. Comprehensive Synthesis of Key Papers in the Repository

The following table summarizes the core papers in this workspace directly addressing welding trajectory generation, torch orientation, collision avoidance, and CAD-to-sensor integration:

| Paper & Authors | Core Focus | Workpiece Type | Torch Posture Method | Approach & Collision Strategy | Software / Hardware Setup |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Fang & Tian (2024)**<br>*Robotics & Computer-Integrated Mfg.* | Model-based welding trajectory planning for identical workpieces | Fillet welds, brackets, structural plates | **Auxiliary Trajectory Method:** Extracts guide curves from CAD parallel to seam to bypass point cloud normal degradation at boundaries. | B-spline curve fitting; normal plane projection; orientation derived from CAD-scan transformation. | Custom 6-axis industrial robot, structured-light 3D camera. |
| **Liu, Tang, Tian, Yang (2023)**<br>*Robotics & Computer-Integrated Mfg.* | Offline programming for multi-pipe intersections | Multi-pipe intersection structures (MPIS) | **NSGA-II Multi-Objective Optimization:** Evaluates dihedral bisector, then optimizes torch posture under cable twisting and all-position constraints. | **8-Cylinder Robot Collision Model:** Closed-form minimum distance checks between robot arm/torch cylinders and pipe cylinders. | ABB IRB2600 robot, Eye-in-hand 3D camera, NSGA-II solver. |
| **Geng, Lai, Tian, Xu, Jiang, Zhang (2022)**<br>*Robotics & Computer-Integrated Mfg.* | Seam extraction & path planning for medium-thick plates | Structural steel boxes, plates, ribs | **Dihedral Structure Method:** Multi-plane RANSAC fitting to extract adjacent planes; bisector normal calculation. | Plane sorting relative to camera coordinate frame; approach along normal bisector. | 6-DOF industrial robot, 3D laser-fringe phase-shift camera. |
| **Peng, Navarro-Alarcon, Wu, Yang (2020)**<br>*IEEE / IROS* | Automatic groove detection & trajectory generation | V-groove plates, boxes, cylinders | **Local Normal Averaging:** $\mathbf{o}_w = \sum \mathbf{n}_{si} / \|\sum \mathbf{n}_{si}\|$ across segmented groove regions (yields bisector). | **MoveIt Cartesian Path Planner:** Generates 6-DOF waypoints and uses MoveIt for collision avoidance and execution. | Universal Robots UR3, Intel RealSense RGB-D camera, PCL, ROS MoveIt. |
| **Jia, Pan, Zhang, Yang, Chen, Chen (2024)**<br>*Applied Sciences* | Tack welding path & trajectory optimization | Industrial plates with distributed tack points | Uniform fixed posture $(180^\circ, 35^\circ, 0^\circ)$ chosen to maximize tool accessibility and avoid wrist flips. | **LF-IWOA TSP Optimization:** Lévy flight whale optimization; straight-line Cartesian motions with via-points to eliminate collisions. | 6-DOF manipulator, MATLAB kinematic simulation, dynamic trajectory model. |
| **Ramsey, Kingston, Thomason, Kavraki (2023)**<br>*IEEE/RSJ IROS* | Fast collision checking against sensor data | General robotic manipulators in point clouds | N/A (Focus on general motion planning) | **CAPT (Collision-Affording Point Trees):** SIMD-accelerated k-d trees evaluating robot-to-point-cloud collisions in $<10\text{ }\mu\text{s}$. | ROS, OMPL, AVX-512 vectorization, benchmarked on Kinova/UR arms. |
| **Zhou et al. (2022) / Wang et al. (2024)**<br>*Robotics & Computer-Integrated Mfg.* | Motion planning for gantry & arc welding robots | Large ship blocks, structural assemblies | Admissible orientation cone tolerances enforced along seams. | **Improved Lazy-PRM & Improved RRT\*:** Delays collision checking to candidate edges; hyper-ellipsoid search pruning. | Gantry welding robot systems, Open-source / proprietary OLP engines. |
| **Ahmed, Tan, Chew, Mamun, Wong (2018)**<br>*IEEE/RSJ IROS* | Edge & corner detection for robotic welding | Tubular joints, intersecting steel members | Edge curvature and local normal variation across unorganized point clouds. | Motion planning with geometric obstacle bounding boxes surrounding tubular joints. | Industrial manipulator, 3D point cloud library (PCL). |
| **Bedaka, Vidal, Lin (2019)**<br>*Int. J. Adv. Manuf. Technol.* | 3D vision + automated offline programming (AOLP) | Multi-part structural assemblies | Roll, pitch, yaw defined by orthogonal matrix of path tangent and underlying CAD surface normals. | CAD model pose estimation in cell coordinate frame; collision detection in commercial OLP environment. | KUKA robot, Ensenso 3D stereo camera, Point Pair Features (PPF). |
| **Tomków, Sobota, Krajewski (2020)**<br>*Facta Universitatis: Mech. Eng.* | Tack weld distribution & sequence vs. angular distortion | Thin stainless steel (AISI 441) sheets | Fixed torch orientation per tack; focus on permutation order. | Jump trajectories between discrete tack points; CMM distortion verification. | TIG welding setup, Coordinate Measuring Machine (CMM). |

---

## 6. Concrete Implementation Blueprint for Your Setup

Based on the synthesis of this literature, here is the recommended end-to-end architecture for your specific research setup (**ROS 2, UR5e manipulator, 6D CAD registration to RGB-D scans, metal plates with weldable edges and tack points**):

```
+──────────────────────────────────────────────────────────────────────────────────────────────────+
|                       RECOMMENDED IMPLEMENTATION ARCHITECTURE (ROS 2 / UR5e)                      |
+──────────────────────────────────────────────────────────────────────────────────────────────────+

  STEP 1: 6D CAD Registration & Scene Synchronization
  ┌───────────────────────────────────────────────────────────────────────────────────────────────┐
  │ • Capture RGB-D scan of metal parts in welding fixture.                                       │
  │ • Estimate 6D pose T_{base}^{cad} via FoundationPose / Scan2CAD / PPF + Point-to-Plane ICP.   │
  │ • Broadcast tf2 transform: base_link -> cad_workpiece_frame.                                  │
  │ • Insert registered CAD collision mesh (or convex V-HACD hulls) into MoveIt 2 PlanningScene.  │
  └───────────────────────────────────────────────────────────────────────────────────────────────┘
                                                │
                                                ▼
  STEP 2: Seam & Tack Feature Extraction (CAD B-Rep Space)
  ┌───────────────────────────────────────────────────────────────────────────────────────────────┐
  │ • Extract edges intersecting faces with dihedral angle phi in [60°, 150°] (fillets/grooves).   │
  │ • For each seam edge, sample path points P_i and tack points P_tack.                          │
  │ • Compute face normals n_1, n_2 and nominal bisector b = (n_1 + n_2) / ||n_1 + n_2||.         │
  │ • Generate CAD auxiliary guide curve (Fang & Tian 2024) to ensure robust normal reference.    │
  └───────────────────────────────────────────────────────────────────────────────────────────────┘
                                                │
                                                ▼
  STEP 3: Advanced Torch Posture Generation & Redundancy Resolution
  ┌───────────────────────────────────────────────────────────────────────────────────────────────┐
  │ • Apply process work angle theta_w (biased toward thicker plate) and travel angle theta_t.    │
  │ • Align torch tool Z-axis: z_tool = R(t, theta_w) * R(y_s, theta_t) * b.                      │
  │ • Formulate 1-DOF Roll Optimization around z_tool:                                            │
  │     phi_roll* = argmax [ w_manip * sqrt(det(J*J^T)) + w_dist * d_min(Links, Mesh)             │
  │                          - w_limit * sum((q_i - q_mid)^2) ]                                   │
  │ • Check inverse kinematics (IK) feasibility using fast analytical UR5e IK solver.             │
  └───────────────────────────────────────────────────────────────────────────────────────────────┘
                                                │
                                                ▼
  STEP 4: Hierarchical Motion Generation (MoveIt 2 / OMPL)
  ┌───────────────────────────────────────────────────────────────────────────────────────────────┐
  │ 1. For each tack or seam start:                                                               │
  │    Compute Standoff Point: P_app = P_start + (d_standoff * z_tool)  [d_standoff = 35-50 mm]    │
  │ 2. Phase 1 (Global Transit):                                                                  │
  │    Plan joint-space path from current pose to P_app using RRT-Connect / Lazy-PRM in MoveIt 2. │
  │ 3. Phase 2 (Local Approach):                                                                  │
  │    Plan Cartesian straight-line path (computeCartesianPath) from P_app to P_start.            │
  │    Enforce tight Cartesian tolerance and constant tool orientation.                           │
  │ 4. Phase 3 (Execution): Arc ignition / tack dwell / seam tracking.                            │
  │ 5. Phase 4 (Local Retract):                                                                   │
  │    Plan Cartesian straight-line path from P_end to P_retract = P_end + (d_ret * z_tool).      │
  └───────────────────────────────────────────────────────────────────────────────────────────────┘
                                                │
                                                ▼
  STEP 5: Tack Weld Permutation Ordering (Physics + Geometry)
  ┌───────────────────────────────────────────────────────────────────────────────────────────────┐
  │ • For n <= 10 tack points, solve discrete permutation pi minimizing:                          │
  │     J(pi) = w_travel * T_travel(pi) + w_dist * J_distortion(pi)                               │
  │ • J_distortion incorporates:                                                                  │
  │     - Center-outward skip-tacking penalty (Tomków et al. 2020)                                │
  │     - Analytical thermal dissipation penalty between consecutive tacks (Rosenthal model)      │
  │     - Joint Rigidity ranking (Tsai JRM: most rigid joint first)                               │
  └───────────────────────────────────────────────────────────────────────────────────────────────┘
```

---

## 7. Conclusions & Strategic Recommendations

1. **Do Not Rely Exclusively on the Symmetric Dihedral Bisector:**
   The dihedral angle bisector is an excellent initial guess for the tool normal plane, but you must augment it with process travel angles ($5^\circ - 15^\circ$ push/drag), thermal mass bias (for unequal plate thicknesses), and molten pool retention under gravity.
2. **Exploit the 1-DOF Torch Symmetry:**
   Your UR5e has 6 degrees of freedom, but an arc/tack torch only needs 5 DOF. Treat the rotation angle around the torch nozzle ($\phi_{roll}$) as a free optimization parameter. This single degree of freedom is your most powerful tool to resolve inverse kinematics, maximize manipulability, avoid wrist singularities, and rotate the robot elbow away from workpiece clamps.
3. **Always Enforce the 4-Phase Motion Hierarchy:**
   Never plan directly from a home pose or between seams in joint space into the weld line. Always insert a **Standoff Approach Point ($25 - 50\text{ mm}$ above the seam)** and execute the final approach as a **Cartesian Linear Path (LIN)**. This guarantees that the bulky torch nozzle will never swing into workpiece flanges or tack beads.
4. **Distortion-Aware Tack Sequencing Over Pure Travel Time:**
   When scheduling tack welds, do not use a standard Traveling Salesperson Problem that only minimizes distance. Incorporate the physical findings of *Tomków et al. (2020)*, *Zhao & Zaeh (2024)*, and the Joint Rigidity Method: skip-tack outward from rigid restraints rather than grouping sequential tacks in close spatial and temporal proximity.

---

### Key References Cited
- **Fang, W., & Tian, X.** (2024). *A novel model-based welding trajectory planning method for identical structural workpieces.* Robotics and Computer-Integrated Manufacturing, 89, 102772.
- **Liu, Y., Tang, Q., Tian, X., & Yang, S.** (2023). *A novel offline programming approach of robot welding for multi-pipe intersection structures based on NSGA-II and measured 3D point-clouds.* Robotics and Computer-Integrated Manufacturing, 83, 102549.
- **Geng, Y., Lai, M., Tian, X., Xu, X., Jiang, Y., & Zhang, Y.** (2022). *A novel seam extraction and path planning method for robotic welding of medium-thickness plate structural parts based on 3D vision.* Robotics and Computer-Integrated Manufacturing, 78, 102381.
- **Peng, R., Navarro-Alarcon, D., Wu, V., & Yang, W.** (2020). *A Point Cloud-Based Method for Automatic Groove Detection and Trajectory Generation of Robotic Arc Welding Tasks.* IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 381–386.
- **Jia, B., Pan, H., Zhang, L., Yang, Y., Chen, H., & Chen, L.** (2024). *Robotic Tack Welding Path and Trajectory Optimization Using an LF-IWOA.* Applied Sciences, 14(15), 6542.
- **Ramsey, C. W., Kingston, Z., Thomason, W., & Kavraki, L. E.** (2023). *Collision-Affording Point Trees: SIMD-Amenable Nearest Neighbors for Fast Collision Checking.* IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS).
- **Zhou, X., et al.** (2022). *Improved Lazy-PRM for offline path planning problem of arc welding robots.* Robotics and Computer-Integrated Manufacturing, 74, 102273.
- **Wang, X., Gao, J., Zhou, X., & Gu, X.** (2024). *Path Planning for the Gantry Welding Robot System Based on Improved RRT\*.* Robotics and Computer-Integrated Manufacturing, 85, 102643.
- **Ahmed, S. M., Tan, Y. Z., Chew, C. M., Al Mamun, A., & Wong, F. S.** (2018). *Edge and corner detection for unorganized 3D point clouds with application to robotic welding.* IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 7350–7355.
- **Bedaka, A. K., Vidal, J., & Lin, C. Y.** (2019). *Automatic robot path integration using three-dimensional vision and offline programming.* International Journal of Advanced Manufacturing Technology, 102(5), 1935–1950.
- **Tomków, J., Sobota, K., & Krajewski, S.** (2020). *Influence of Tack Welds Distribution and Welding Sequence on the Angular Distortion of TIG Welded Joint.* Facta Universitatis, Series: Mechanical Engineering, 18(4), 611–621.
- **Zhao, H., Zapata, C., & Zaeh, M. F.** (2024). *Simplified welding simulation for WAAM sequence optimization.* Virtual and Physical Prototyping.
