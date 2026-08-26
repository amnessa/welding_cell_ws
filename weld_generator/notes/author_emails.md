# Author correspondence — six implementation-confirmation emails

Drafts for the corresponding authors of the six reimplemented literature methods.
Each email: (1) notifies them their method is reimplemented for comparison in an MSc
thesis and will be cited, (2) explains the oracle substitution for their learned/coarse
stage and asks whether it is a fair stand-in, (3) asks the *specific* questions our
implementation had to answer by guessing — the numbered deviations documented in each
module's docstring. Offer of code and pre-submission numbers in every mail.

Fill in: **[Your full name]**, **[advisor's name]**, **[repo/code link if you decide to
share one]**. Note that emails 3 and 6 go to the same person (Prof. Xincheng Tian) about
two different papers — each references the other so it does not read as spam.

---

## 1. `lit-ransac` — Prof. Xuan Kong <kongxuan@hnu.edu.cn>
*(cc: yijinxin@hnu.edu.cn)*

**Subject:** Question on reimplementing your Automation in Construction 2026 weld-seam
extraction method for a benchmark comparison

Dear Prof. Kong,

I am an MSc student at Middle East Technical University (Ankara), working on a synthetic
benchmark for weld-seam extraction in which the ground-truth seam geometry is constructed
analytically rather than annotated. As part of the thesis I have reimplemented the
plane-fitting and path-generation stages (Sections 5–6) of your paper "Weld seam
extraction and path generation for robotic welding of steel structures based on 3D
vision" (Autom. Constr. 183, 2026, 106792) as one of six literature methods in a
controlled comparison, and I would like to both notify you of this use and ask three
questions so the reimplementation represents your work correctly.

Since I cannot retrain your PointNet++ stage, I supply its output as an oracle: the
input cloud is cropped to a ~40 mm band around the true seam, matching the annotation
width and near-perfect mIoU you report. Results obtained with and without this stage are
reported separately. Could you confirm this is a fair stand-in for Section 3?

My specific questions:

1. Eq. (18), read literally with the term (3/N_fp)^3, implies an iteration count of
   order 1e10 for a 10,000-point subset, which seems inconsistent with the 0.187 s
   runtime in Table 6. I implemented the standard adaptive form with w = n_in/N_fp in
   its place. Is that the intended reading?
2. Section 5.4 describes the refit plane normal from two randomly selected neighborhood
   points and one cross product (Eq. 19). A two-point estimator seems higher-variance
   than the 0.011° residual reported in Fig. 16; I used a least-squares fit over the
   centroid neighborhood instead. Was that your implementation as well?
3. For Section 6.2, I read "the vertical point clouds" projected for endpoints as the
   plate that terminates at the seam (the shorter extent along the weld direction).
   Is that correct?

For validation, my implementation reproduces your Section 7.2.1 accuracy on T-joints
(RMSE 0.60 mm and ME 0.81 mm against constructed truth, versus your reported maxima of
0.64/1.16 mm against taught paths). I would gladly share the code and the comparison
numbers before any submission.

With thanks and best regards,
[Your full name]
MSc student, Middle East Technical University
(supervisor: [advisor's name])

---

## 2. `lit-regiongrow` — Prof. Wei Zhang <davidzhang@sdu.edu.cn>

**Subject:** Parameter questions on your coarse-to-fine multi-seam detection (arXiv:2408.10710) for a benchmark reimplementation

Dear Prof. Zhang,

I am an MSc student at Middle East Technical University (Ankara), building a synthetic
weld-seam benchmark with analytically constructed ground truth, and I have reimplemented
the fine-stage region-growing extractor of your paper "Coarse-to-Fine Detection of
Multiple Seams for Robotic Welding" as one of six literature methods in a controlled
comparison. I am writing to notify you of this use (with citation), and to check four
points the paper leaves open, so my implementation does not misrepresent yours.

Since I cannot reproduce your FastSAM stage, I supply per-surface masks from the
generator's ground truth as an oracle for Section III-C, and report results with and
without them separately. Is that a fair stand-in?

Questions:

1. Algorithm 1 gives no numeric values for the smoothness threshold (Threshold1), the
   curvature threshold (Threshold2), or the neighborhood size k. PCL's RegionGrowing
   defaults (k=30, 3°) mark nearly every point as an edge on thin plates in my data
   (a k=30 neighborhood at the published 3 mm voxel spans ~9 mm, wider than the plate).
   Could you share the values used in your experiments?
2. I read Algorithm 1 as pseudocode for the standard region-growing formulation (visited
   set, outer loop over unvisited minimum-curvature seeds, minimum region size), since
   as printed it grows a single region. Is that the intended reading?
3. For "points that satisfy both conditions" in Section III-D, I test that an edge
   point's neighborhood contains points of at least two grown regions. Is that how your
   implementation identified weld edges?
4. On assemblies where two seams lie one plate-thickness apart (e.g., both fillets of a
   T-joint), Euclidean clustering either merges or fragments them in my reimplementation.
   Did your implementation include any additional step for separating nearby seams?

I would gladly share code and results before submission.

Best regards,
[Your full name] — MSc student, METU (supervisor: [advisor's name])

---

## 3. `lit-lobb` — Prof. Xincheng Tian <txch@sdu.edu.cn>
*(re: RCIM 95 (2025) 102987 + the LOBB descriptor of IEEE T-ASE 22 (2025); cc: gys_sdu@163.com)*

**Subject:** Reimplementation questions on your RCIM 2025 semantic-segmentation + LOBB weld extraction

Dear Prof. Tian,

I am an MSc student at Middle East Technical University (Ankara), constructing a
synthetic weld-seam benchmark with exact analytic ground truth, and I have reimplemented
the pipeline of "A novel weld seam extraction method with semantic segmentation and point
cloud feature for irregular structure workpieces" (RCIM 95, 2025), using the LOBB
descriptor from your group's IEEE T-ASE 2025 paper, as one of six literature methods in a
controlled comparison. (I am writing separately about your 2024 RCIM model-based
trajectory paper, which I have also reimplemented.) I would like to notify you of this
use and confirm three implementation choices.

Since I cannot retrain K-Net, I supply its per-component masks from the generator's
ground truth (equivalent to perfect segmentation at your reported 97.35% mIoU) as an
oracle, and report with/without arms separately. Fair?

Questions:

1. Section 3.4.2 motivates Mean-Shift key-point filtering by the higher point density
   scanners return at creases. My synthetic clouds are sampled uniformly by area, so this
   density gradient does not exist and I have disabled the filter by default (keeping it
   implemented and switchable). Do you agree that is the faithful treatment on
   density-uniform data?
2. The T-ASE paper selects the LOBB radius of 2.5 mm "according to the weld width". I
   have treated it as a weld-width parameter (not a free tuning parameter). Correct?
3. On T-joints, the two fillet seams plus the short contact runs at the plate ends form
   a closed contact perimeter; a single polynomial fit (Section 3.4.3) cannot represent
   it. Did your workpieces contain such cases, and if so how were they handled?

Encouragingly, my reimplementation reproduces your reported accuracy on corner-type
geometry (median RMSE 0.51 mm vs your <0.7 mm). Code and results available on request.

Best regards,
[Your full name] — MSc student, METU (supervisor: [advisor's name])

---

## 4. `lit-ppf` — Dr. Yaobin Yue <ybyue2020@qust.edu.cn>

**Subject:** Two clarifications on your Scientific Reports 2024 PPF weld-seam method, reimplemented for a benchmark

Dear Dr. Yue,

I am an MSc student at Middle East Technical University (Ankara), building a synthetic
weld-seam benchmark with analytically constructed ground truth. I have reimplemented the
point-cloud half of "Weld seam object detection system based on the fusion of 2D images
and 3D point clouds using interpretable neural networks" (Sci. Rep. 14:21137, 2024) —
the PPF-based plane extraction, the orthogonal-plane detection with local Hough voting
(Eqs. 21–23), and the corner extraction — as one of six literature methods in a
controlled comparison. I am notifying you of this use (with citation) and would
appreciate clarification on three points.

The Faster R-CNN stage is supplied as an oracle (the cloud cropped to a box around the
true weld, standing in for your detector's output), with results reported with and
without it. Fair stand-in?

Questions:

1. The methods section proposes the PPF plane extraction explicitly as an alternative to
   RANSAC, while Algorithm 1 in the implementation section states planes were "extracted
   ... using RANSAC". I implemented the PPF path as the paper's contribution. Which was
   used for the reported experiments?
2. I read the (θ, ρ) voting as local — for a fixed reference point, ρ is constant over
   all partners on a true plane — so my global consensus statistic is the θ peak, with
   the intersection line recovered from the voted pair's plane parameters. Is that
   consistent with your implementation?
3. The 0.1 mm feature-point threshold appears matched to the 50 µm scanner. On clouds
   with ~1 mm point spacing I scale the threshold with spacing (reporting the literal
   0.1 mm as well). Reasonable?

My implementation reproduces your corner-distance accuracy (median error 0.4–0.5%
against your reported 2.17% average). Code and results gladly shared.

Best regards,
[Your full name] — MSc student, METU (supervisor: [advisor's name])

---

## 5. `lit-pcaslice` — Tianqi Wang <wtq0622@163.com>
*(cc the corresponding author listed in the final publication record of "3D vision-based
intersecting pipe welding path planning", Welding in the World, 2026)*

**Subject:** Reimplementation of your PCA-slicing weld path method for a benchmark — three questions

Dear Mr. Wang,

I am an MSc student at Middle East Technical University (Ankara), building a synthetic
weld-seam benchmark with analytically constructed ground truth. I have reimplemented the
point-cloud stages of "3D vision-based intersecting pipe welding path planning" (Welding
in the World, 2026) — the filtering, the PCA-based slicing centreline (Section 4.2), the
cubic-NURBS path fit, and the MSAC + WTLSD posture planes — as one of six literature
methods in a controlled comparison. This email is to notify you of that use (with
citation) and to check three details.

The YOLO11 + DeepLab V3+ stage is supplied as an oracle: one mask per weld instance,
derived from ground truth, standing in for your per-instance detection. Results with and
without it are reported separately. Fair?

Questions:

1. The slice width in Section 4.2 is not given numerically; I default to ~3× the point
   spacing. What value did your experiments use?
2. With uniform weights, the cubic NURBS of Section 4.3 is equivalent to a cubic
   B-spline; since no weights are specified, I implemented it as such. Correct?
3. Your pipeline processes one weld instance per ROI. On scenes where two straight seams
   fall in one region, the per-slice geometric centre lands between them. Am I right
   that the per-instance detection stage is what excludes this case in your system?

On straight seams with the per-instance oracle my implementation reaches F1 0.94–1.0
against constructed truth; I expect its distinctive value to show on curved seams, which
my benchmark adds in its next phase. Code and results gladly shared.

Best regards,
[Your full name] — MSc student, METU (supervisor: [advisor's name])

---

## 6. `lit-modelreg` — Prof. Xincheng Tian <txch@sdu.edu.cn>
*(re: RCIM 89 (2024) 102772, Fang & Tian; second email, cross-referenced with #3)*

**Subject:** Reimplementation questions on your RCIM 2024 model-based welding trajectory method

Dear Prof. Tian,

Further to my email about your 2025 RCIM segmentation paper: I have also reimplemented
"A novel model-based welding trajectory planning method for identical structural
workpieces" (RCIM 89, 2024, 102772) for the same six-method benchmark comparison, and
have three questions specific to it.

In my setting the basic model is constructed from the generator's own part parameters
(the benchmark is synthetic, so CAD is available by construction), and I label every
result of this method accordingly — it anchors the model-based end of the comparison
rather than competing as a scan-only method. Two implementation questions and one
finding:

1. The target workpiece's feature point set X: in your experiments, was it extracted
   from the scan itself (and if so, by what edge extraction), or derived from the target
   workpiece's own 3D model? I could not find the extraction specified, and the answer
   decides what a scan-only reproduction should register against.
2. I implemented the registration as classical CPD (similarity transform, then a coherent
   Gaussian-kernel deformation field), rather than the Bayesian variant, treating the
   difference as a robustness refinement rather than a different mechanism. Do you
   consider that a faithful reduction?
3. A possibly interesting observation: registering dense surfaces instead of edge point
   sets lets stacked-plate assemblies slide along their overlap direction (~5 mm in my
   tests) — your Section 3.1 choice of edge features appears load-bearing, not
   incidental. Was that a deliberate design consideration?

With the model-derived features, my implementation reaches registration-floor accuracy
(RMSE 0.5–1.6 mm) on four of five joint types against exact truth. Code and the
comparison gladly shared before submission.

Best regards,
[Your full name] — MSc student, METU (supervisor: [advisor's name])
