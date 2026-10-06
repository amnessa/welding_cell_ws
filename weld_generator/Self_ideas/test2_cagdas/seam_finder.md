# Seam extraction from labelled point clouds — implementation and test plan

Oct 6, 2026 · @Çağdaş Güven

## Goal and scope

Build and test a seam extractor that takes two labelled point clouds (part A, part B) and returns every weldable seam with toe lines, gap, joint angle and feasible torch directions. The method is matching-based seeding plus sphere-tracing refinement; rolling-ball tracing is an optional extension.

**Inputs**

- Point cloud A and point cloud B, labels known, in one common frame (mm).
- Optional: camera poses (eye-in-hand) for normal orientation and visibility.
- Torch model: nozzle radius, allowed work-angle and travel-angle range, standoff.

**Outputs, per seam**

- Ordered seam polyline (root line), resampled at a fixed step.
- Toe line on A and toe line on B.
- Per point: gap (mm), joint angle (deg), joint type, torch direction and feasible-orientation interval.
- Accessibility flag per span (weldable / not weldable).

**Assumptions**

- Labels are correct away from the joint; small label noise near the joint is tolerated.
- Parts are rigid, stationary, and each part is a single connected surface patch per seam side.
- Gap is at most about 3 mm; larger gaps are treated as "no seam".
- Development starts on synthetic analytic clouds, then moves to UR5e RGB-D scans.

## Pipeline overview

&#91;embedded content: seam pipeline · 6 stages + 1 optional\]

Synthetic scenes feed the pipeline during development; real scans replace them later. The torch-cone check (2c) runs last and only on seam points, because it is the expensive one.

## Stage 0 — Synthetic test data

Build the generator first: every later stage is judged against its analytic ground truth. Each scene is two meshes (one per part) sampled to points, with the true root line and both true toe lines stored alongside.

**Scene families**

| Scene | Geometry | What it tests |
| --- | --- | --- |
| T-joint | 6 mm stem on base plate, 90° | Two seams from one joint; one-step walker convergence |
| Angled joint | Plate on base at 30°, 45°, 60° | Walker step ratio vs angle |
| Lap joint | Two overlapping plates | Hidden contact face; accessibility filter |
| Butt joint | Coplanar plates, optional V-groove | Normal angle near 0°; groove geometry |
| Pipe on plate | Cylinder Ø 60 mm on plate | Curved seam; ordering and smoothing |
| Corner box | Three plates meeting | Several seams meeting at a node |

**Perturbations (sweep each independently, then combined)**

- Gap: 0, 0.5, 1, 2, 3 mm.
- Sampling spacing: 0.5, 1, 2, 4 mm.
- Gaussian noise along the normal: σ = 0, 0.25, 0.5, 1, 2 mm (RGB-D at 0.5 m is roughly 1–2 mm).
- Outliers: 0, 1, 5 % uniform in the bounding box.
- Partial view: drop points not visible from 1, 2 or 4 virtual camera poses (raycast against the mesh).
- Label noise: flip labels of points within 2 mm of the joint with probability 0, 0.1, 0.3.

**Ground truth stored per scene**

- Root line as a dense polyline; toe lines on A and B.
- Gap and joint angle along the seam.
- Mesh of each part (for raycasting checks), random seed and all parameters in a JSON sidecar.

Tools: trimesh or Open3D for meshes, sampling and raycasting; save as PLY + JSON.

## Stage 1 — Preprocessing

Compute everything per part, never on the merged cloud: the labels are what keep normals clean at the corner.

1. Voxel-downsample each part to spacing h (start h = 1 mm); remove statistical outliers.
2. Estimate normals per part with PCA on k = 20–30 neighbours. Because neighbourhoods only contain same-part points, normals do not blur across the joint.
3. Orient normals outward. Synthetic: from the mesh. Real scans: flip each normal to face the camera position that observed it.
4. Build a KD-tree per part (`scipy.spatial.cKDTree`).
5. Define an MLS projection Π\_A(x) and Π\_B(x): gather neighbours within radius r\_mls = 3h, fit a Gaussian-weighted plane (degree 1; degree 2 as an option), project x onto it. Return the projected point and its normal.
6. Define the distance d\_B(x) = |x − Π\_B(x)| and the direction to B, u\_B(x) = (Π\_B(x) − x) / d\_B(x). Same for A.

**Unit tests**

- On a synthetic plane: Π error below 0.1h with zero noise; normals within 1°.
- Near a 90° corner: per-part normals stay within 3° of truth at 1h from the corner (compare against merged-cloud normals to show the benefit).

## Stage 2 — Accessibility filter

Run a cheap filter on all points before matching, and an exact torch-cone check on seam points after Stage 5. Checking every point with the full torch model is too expensive and mostly wasted.

**2a. Coarse prefilter (all points, before Stage 3)**

- For each point p with outward normal n, cast M rays (M = 16–32) in a cone of half-angle 60° around n, starting at p + εn.
- Ray hit test against both parts. Options, in order of preference:
  1. Synthetic: Open3D `RaycastingScene` on the meshes.
  2. Real scans: ray-march with sphere tracing on the unsigned distance of the merged cloud (step = distance to nearest point, hit when distance < h). No meshing needed.
- Keep p if the free-ray fraction is at least τ\_vis (start 0.3). This removes the hidden contact face of a lap joint and deep internal surfaces.

**2b. Camera visibility (real scans only, optional)**

- Hidden Point Removal (Katz et al. 2007) from each camera pose, to separate "not seen" from "not reachable".

**2c. Torch-cone check (seam points, after Stage 5)**

- At each seam point, sample torch directions inside the allowed work-angle × travel-angle box around the normal bisector.
- For each direction, test a capsule (nozzle radius + clearance, length = standoff + nozzle length) against both parts.
- Output the feasible orientation set; mark a span not weldable when the set is empty.

**Tests**

- Lap joint: the hidden contact face must be fully removed by 2a while the visible fillet corner survives.
- T-joint at 30°: the acute side must report a narrower feasible set than the obtuse side.

## Stage 3 — Seed generation

Seeds are midpoints of cross-part pairs that pick each other as nearest neighbour; they lie within about h/2 of the root line. This is where the N × M matrix idea lives, on samples rather than the full clouds.

1. Farthest-point sampling on the filtered clouds: N points from A, M from B (start N, M = 1000–2000).
2. Cross-distance matrix D (N × M), Euclidean. At 2000 × 2000 this is 4M floats, about 32 MB in float64.
3. Mutual nearest neighbours: keep (i, j) if j = argmin over row i, i = argmin over column j, and D\[i, j\] < τ\_d. Start τ\_d = gap\_max + 2h.
4. For each kept pair store: a\_i, b\_j, midpoint s = (a\_i + b\_j)/2, n\_A, n\_B, and θ\_n = angle between n\_A and n\_B.
5. Joint type per seed from θ\_n: about 90° fillet (T, corner or lap: a lap weld joins the top plate's edge to the lower plate's face), about 0° butt (normals parallel), about 180° facing surfaces (normally a hidden contact face that Stage 2 should already have removed; treat surviving ones as a filter failure). θ\_n alone cannot separate T from lap; use the extent of the A-side face (plate-edge width) for that. Build the histogram of θ\_n over all seeds and over a sliding window along the seam (after Stage 5) to detect joint-type changes.

**Ablation variants (same interface, swap in)**

- One-way NN with threshold only (no mutual check): expected to give a thick band of false seeds.
- Partial optimal transport (POT library, `ot.partial` or unbalanced Sinkhorn on D): soft matching, transported mass m as a parameter.
- Seed density: N = 250, 500, 1000, 2000.

**Tests**

- T-joint, no noise: every seed within h of the true root; seeds appear on both corners.
- Far-face pairs: zero seeds farther than 3h from any true seam.

## Stage 4 — Sphere-tracing walkers

Each seed launches two walkers: one on A walking toward B, one on B walking toward A. Each step is the current distance to the other part, projected onto the walker's own surface, so it cannot overshoot and shrinks on its own.

**Walker on A (mirror for B)**

```text
input: start x0 = seed's A-end, params alpha, eps, tol, K_max
x = Pi_A(x0); d_prev = inf
for k in 1..K_max:
    q, d = Pi_B(x), |x - Pi_B(x)|        # closest point on B and distance
    if d < eps or d_prev - d < tol:       # touching, or stalled at a gap
        break
    u = (q - x) / d                       # direction to B
    n = normal_A(x)
    t = u - (u . n) n                     # project onto A's tangent plane
    if |t| < 1e-3: break                  # B is straight along the normal: at the toe
    x = Pi_A(x + alpha * d * t / |t|)     # step, then snap back onto A
    log d_k; d_prev = d
return toe_A = x, gap = d, ratios r_k = d_{k+1} / d_k
```

**Parameters**

- α = 0.9 (safety factor), ε = 0.05h, tol = 0.01h, K\_max = 50.

**Outputs per seed**

- toe\_A, toe\_B; gap g = |toe\_A − toe\_B| at convergence.
- Root point: for a fillet, intersect the two MLS tangent planes at the toes with the plane through the seed normal to the local seam direction; for gaps, use the midpoint of toe\_A and toe\_B.
- Joint angle, two independent estimates: θ\_n from normals, and θ\_r = asin((1 − r̄) / α) from the mean step ratio (valid on planar faces). Their disagreement is a quality flag.
- Iteration count, a convergence flag.

**Tests**

- Analytic angled joints: iterations to 0.05h match log(0.05h / d0) / log(1 − α sinθ) within one step.
- θ\_r within 3° of truth at 30°, 45°, 60° with zero noise.
- Gap 1 mm, spacing 1 mm: gap error below 0.25 mm.
- Never-overshoot check: d\_k is non-increasing on every run.

## Stage 5 — Seam assembly

Refined root points become ordered, smooth seams with a torch frame at every sample.

1. Drop seeds whose walkers did not converge or whose θ\_n and θ\_r disagree by more than 10°.
2. Cluster root points into separate seams with DBSCAN (eps = 3–5h, min\_samples = 5). Split a cluster where the seed normals (n\_A, n\_B) jump, so the two sides of a T-joint stay separate.
3. Order each cluster: build a kNN graph, take its minimum spanning tree, and use the longest path through the tree as the seam order. Points off that path are projected onto it. If the path's two ends are closer than 3h, close it into a loop (pipe-on-plate).
4. Fit a smoothing spline (`scipy.interpolate.splprep`, smoothing set from noise σ), resample at step Δs (start 2 mm).
5. Interpolate toe lines, gap and joint angle onto the resampled points.
6. Torch frame per point: tangent t from the spline; torch axis = −normalize(n\_A + n\_B) tilted by the travel angle about t; work angle measured from the bisector.
7. Run Stage 2c on each resampled point; split the seam into weldable and not-weldable spans.

**Tests**

- Pipe on plate: one closed seam, ordered without jumps; spline radius within 1 % of truth.
- Corner box: three seams, correctly separated at the shared node.

## Stage 6 — Extension: rolling-ball spine tracing

Once Stages 0–5 work, replace the spline assembly with continuous tracing of the rolling-ball spine. This turns discrete seeds into a curve defined by the geometry itself, with the ball radius r as the weld leg size.

The spine is the set of ball centres c where both distance constraints hold:

```latex
d_A(c) = r, \qquad d_B(c) = r
```

Trace it with predictor–corrector steps:

1. Start: from a converged seed, solve for c with Newton on the two constraints (start at the root point offset along the bisector by r / sin(φ/2), where φ = 180° − θ\_n is the opening angle between the faces).
2. Predict: tangent t = normalize(∇d\_A × ∇d\_B); step c' = c + Δs t.
3. Correct: Gauss–Newton on F(c) = \[d\_A(c) − r, d\_B(c) − r\], minimum-norm update in the plane normal to t.
4. Adapt Δs: halve if correction needs more than 3 iterations or the tangent turns more than 10°; grow by 1.5 otherwise.
5. Stop when the ball cannot satisfy both constraints (end of seam), when it meets another spine, or when the torch-cone check fails.

Outputs: spine (torch path offset), contact points on A and B (toe lines), and an accessibility answer for free: if no ball of radius r fits, nothing is traced.

**Test**: on the T-joint and the 30° joint, the spine must match the analytic offset line within 0.1h, and its toe lines must match Stage 4 within 0.5h.

## Evaluation

Judge the method on position accuracy and completeness first; gap, angle and runtime second. Report every metric in mm or degrees, as mean, 95th percentile and max over 10 random seeds per setting.

**Metrics**

| Metric | Definition | Unit |
| --- | --- | --- |
| Root error | Distance from each output root point to the true root polyline | mm |
| Completeness | Fraction of true seam length with an output point within 1 mm | % |
| Precision | Fraction of output seam length within 1 mm of a true seam | % |
| Toe error | Distance from output toe lines to true toe lines | mm |
| Gap error | Output gap minus true gap | mm |
| Angle error | θ\_n and θ\_r minus true joint angle | deg |
| Joint-type accuracy | Correct class per seam | % |
| Accessibility accuracy | Weldable / not-weldable spans vs mesh-based ground truth | % |
| Runtime | Per stage, for 50k and 200k points per part | s |

**Experiments**

1. Sweeps on every scene family: noise σ, spacing h, gap, angle, outliers, partial view, label noise (Stage 0 lists the values). Plot each metric against the swept parameter.
2. Ablations:
   - Seeding: mutual NN vs one-way NN vs partial OT.
   - Refinement: seeds only vs seeds + walkers.
   - Surface model: raw nearest point vs MLS projection.
   - Accessibility: with vs without Stage 2a (lap joint shows the difference).
   - Normals: per-part vs merged-cloud.
3. Baselines on the same scenes:
   - RANSAC plane fit per part + plane intersection line.
   - Contact-band method: points of A within τ\_d of B, then line or spline fit.
   - Curvature-based region growing on the merged cloud (labels ignored).

**Real data**

- Scan 5–10 physical joints (T, angled, lap, pipe-on-plate) with the UR5e eye-in-hand RGB-D camera, several poses each.
- Ground truth: register the CAD model to the scan (ICP) and take the CAD root line, or touch-sense the seam with the robot at 10–20 points.
- Report the same metrics, plus a weld trial on a subset if the cell is available.

**Novelty check (before writing)**

- Scholar/Scopus queries: "rolling ball" + weld + point cloud; "mutual nearest neighbor" + weld seam; "sphere tracing" + weld; accessibility shading + robotic welding; fillet/blend recognition in CAD feature recognition.

## Code structure and parameters

Python package, one module per stage, each stage a pure function from dataclass in to dataclass out so every stage can be tested and swapped alone. Wrap it in a ROS 2 node only after the synthetic results hold.

```text
seamfind/
  synth/        scenes.py (generators), perturb.py, groundtruth.py
  geom/         normals.py, kdtree.py, mls.py (Pi_A, Pi_B, d, u)
  access/       prefilter.py (2a), hpr.py (2b), torch_cone.py (2c)
  seeds/        fps.py, mutual_nn.py, partial_ot.py, joint_type.py
  walk/         walker.py (Stage 4), root.py
  assemble/     cluster.py, order.py (MST path), spline.py, frames.py
  spine/        rolling_ball.py (Stage 6)
  eval/         metrics.py, sweeps.py, baselines.py, plots.py
  config/       default.yaml
tests/          one test file per stage, using synth scenes
```

Libraries: numpy, scipy (cKDTree, splprep), open3d (IO, normals, RaycastingScene, visualisation), trimesh (scene meshes), POT (partial OT), scikit-learn (DBSCAN), networkx (MST).

**Default parameters (all relative to spacing h)**

| Parameter | Default | Stage |
| --- | --- | --- |
| h, voxel spacing | 1 mm | 1 |
| k, normal neighbours | 25 | 1 |
| r\_mls, MLS radius | 3h | 1 |
| M rays, cone half-angle | 24, 60° | 2a |
| τ\_vis, free-ray fraction | 0.3 | 2a |
| N, M, FPS samples | 1500 | 3 |
| τ\_d, pair distance | gap\_max + 2h | 3 |
| α, ε, tol, K\_max | 0.9, 0.05h, 0.01h, 50 | 4 |
| DBSCAN eps, min\_samples | 4h, 5 | 5 |
| Δs, resample step | 2 mm | 5 |
| r, ball radius | target leg length | 6 |

Log every run with its config, scene seed, per-stage timings and metrics to one CSV row, so sweeps are a groupby, not a rerun.

## Milestones

Work in this order; each milestone ends with a passing test, not a finished module. Tick them off as they land.

- [ ] **M1 — Synthetic scenes.** Done when all six scene families generate with ground truth and a viewer overlays root and toe lines on the points.
- [ ] **M2 — Geometry core.** Done when MLS projection and per-part normals pass the Stage 1 unit tests.
- [ ] **M3 — Seeds.** Done when mutual-NN seeds on the clean T-joint all lie within h of the true root, on both corners, with zero far-face seeds.
- [ ] **M4 — Walkers.** Done when iteration counts and θ\_r match theory on the 30°, 45° and 60° joints, and gap error is below 0.25 mm at 1 mm gap.
- [ ] **M5 — Assembly.** Done when the pipe-on-plate gives one closed ordered seam and the corner box gives three separated seams.
- [ ] **M6 — Accessibility.** Done when the lap joint's hidden face is removed by 2a and 2c returns narrower feasible sets on acute sides.
- [ ] **M7 — Sweeps and ablations.** Done when every Evaluation plot exists for the synthetic scenes and the three baselines.
- [ ] **M8 — Real scans.** Done when 5–10 physical joints are scanned on the UR5e cell with ground truth and the same metrics are reported.
- [ ] **M9 — Rolling-ball spine (optional).** Done when spine tracing matches the analytic offset within 0.1h on the T-joint and 30° joint.

## Risks and open questions

| Risk | Effect | Mitigation |
| --- | --- | --- |
| RGB-D noise (1–2 mm) is larger than h | Walker stalls early; gap overestimated | Raise r\_mls; degree-2 MLS; average several views before Stage 1 |
| Shallow joints (below 20°) | Walker needs many steps; θ\_r unstable | Cap K\_max; fall back to θ\_n; flag the seam |
| Label errors at the joint | False mutual pairs, toe shift | Erode labels by 1–2h near B before matching; test with Stage 0 label noise |
| Gap above τ\_d | Seam missed entirely | Report "no seam within τ\_d"; sweep τ\_d in experiments |
| Missing data in the corner (self-occlusion from the camera) | Seed gaps along the seam | Multi-view scans; bridge gaps in Stage 5 only when below 10 mm |
| Ray-march hit test on raw points leaks through sparse regions | Inaccessible points kept | Hit threshold tied to local density, not fixed h |
| Novelty already published | Weaker paper claim | Run the novelty-check queries before Milestone 4 |

**Open questions**

- Is the root point (tangent-plane intersection) or the toe midpoint the right torch target for your welding process? This changes Stage 4 outputs.
- Will scans come from one pose or several? Multi-view changes how much Stage 2b matters.
- Is Stage 6 needed for the paper, or are seeds + walkers + spline enough?
