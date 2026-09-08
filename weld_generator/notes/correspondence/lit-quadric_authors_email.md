# Draft — to the authors of lit-quadric (Li, Wang & Wang, JMST 2026)

**To:** Dongmin Li <ldm753@163.com> (corresponding author)
**Subject:** Reimplementation of your JMST 2026 impeller weld-seam method — seven questions on the parts the paper leaves open

Dear Dr. Li,

I am a graduate student in robotics at METU (Middle East Technical University), working on a
benchmark for 3D weld-seam extraction methods evaluated against *constructed* ground truth
— synthetic assemblies in which the seam is known in closed form from the placement of the
parts, so that published methods can be compared under identical conditions (joint type,
curvature, groove preparation, sensor noise, fixture presence) rather than on each paper's
own workpieces.

Your paper, *Automatic recognition on impeller shape and weld seam based on normal of point
clouds and PCA* (J. Mech. Sci. Technol. 40(3), 2026), is one of seven methods I have
reimplemented for this comparison, and it occupies a distinctive place in it: it is the only
one whose welding-surface model is a quadric rather than a plane, which makes a pipe
standing on a plate (plane ∩ cylinder) or a pipe on a pipe (cylinder ∩ cylinder) part of
its mechanism. I would like to make sure my reimplementation is faithful before any number
attributed to your method is reported, and there are a few points where the paper leaves a
choice to the implementer. I would be grateful if you could confirm or correct my readings.

1. **The flat/curved decision (§3.1, §4.2).** §3.1 states that a surface is flat when "the
   distribution probability of the normal vectors within the standard deviation range is
   over 90 %". Read literally, the share of samples within one standard deviation of the
   mean is close to 68 % for any roughly normal distribution regardless of the surface, so
   a flat blade would fail the test. §4.2 applies the rule as an angle threshold ("more than
   30 % of the normal deviation angle is over 10°, thus … curved"). I have implemented the
   §4.2 form: a surface is flat if at least 90 % of its point normals deviate less than 10°
   from the surface's mean normal. Is that the rule you ran, and is the mean normal the
   arithmetic mean of the (sign-aligned) point normals?

2. **The plane fit (eq. 7–11).** The loss in eq. 8 is minimised over a, b, c, d without a
   normalisation constraint, for which the minimiser of the printed normal equations is the
   zero vector. I have implemented the estimator your reference [21] describes — total least
   squares, with the plane normal as the smallest-eigenvalue direction of the point
   covariance. Is that the intended fit?

3. **The quadric fit (eq. 12).** For the curved surface I solve the algebraic least-squares
   problem as a homogeneous system (unit-norm coefficient vector, smallest right singular
   vector of the design matrix) on centred and scaled coordinates. Did you condition the
   coordinates, and how was the trivial solution excluded?

4. **The seam threshold (§3.3).** The actual weld points are "a series of point clouds within
   the threshold" of the theoretical intersection. I take a point to be near the
   intersection when it is near both fitted surfaces (first-order distance |f|/‖∇f‖ below a
   threshold of 1.5 × the voxel size, i.e. ≈ 2 mm at a 1.5 mm voxel), restricted to points
   whose projection lands on the other surface's actual patch, and then project it onto the
   intersection by alternating projections. What threshold did you use, and did you compute
   the intersection curve analytically or work directly with the nearby points? A related
   point: when the two parts are separated by a root gap larger than the threshold, no
   point is near both surfaces and the seam is not found — was the blade always in contact
   with the hub in your experiments, or is the threshold chosen relative to the gap?

5. **The ordering (§3.3).** After the graph walk of eq. 13 identifies the initial point, the
   points are "sorted in order from small to large" by their distance from it. On a straight
   or gently curved open seam this reproduces the arclength order; on a closed seam (a pipe
   on a plate) points on opposite sides at equal distance interleave and the path folds back
   across the ring. Was the method intended for open seams only, or is there an additional
   step for closed contours that the paper does not describe? In my implementation I keep
   your distance ordering as the published arm and report a nearest-neighbour chain from the
   same initial point as a separate, clearly labelled corrected arm.

6. **Segmentation (§2.3).** Region growing "divides the initial point clouds into impeller
   hub point clouds and fan blade point clouds". Could you share the growth criteria and
   thresholds (normal-angle and curvature), and confirm that the cloud comes from a single
   camera pose, so that each part contributes one welding surface? In my benchmark the
   equivalent stage is supplied as an oracle (per-surface labels with part membership) at
   one rung of the evaluation, and grown by a standard region-growing algorithm at the next,
   so that the cost of the segmentation stage is reported separately from the fitting.

7. **The metric (Fig. 6).** The reported < 1 mm errors per axis are against a taught path.
   Could you say what the error budget of the teaching itself was (repeatability, TCP
   calibration), and whether the plotted errors are per-experiment means or maxima along
   the path? This determines how your number should be compared with an error measured
   against exact geometry.

For what it is worth, on synthetic exact geometry my implementation recovers the
plane ∩ plane, plane ∩ cylinder and cylinder ∩ cylinder intersections to numerical
precision, and on the benchmark's pipe-on-plate scenes it recovers the complete circular
seam at ~0.01 mm RMSE with the chain ordering — so the mechanism as I read it works as your
paper says it should; my questions are only about whether it is the mechanism you ran.

I will of course share the reimplementation and the evaluation with you before anything is
published, and I would be glad to cite any clarification as personal communication.

With thanks and best regards,

Çağdaş Güven
MSc student, Robotics — Middle East Technical University (METU)
cagdas96vkp@gmail.com
