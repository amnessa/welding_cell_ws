To make the PCA method work across a gap, you must switch your KD-Tree from a $k$-NN search to a **Radius Search ($R$-NN)**.

Here is the exact methodology to implement this robustly in your pipeline:

### Step 1: The "Gap-Bridging" Radius Search

Instead of asking the KD-Tree for the "30 closest points," you ask it for "all points within radius $R$."

* **The Rule:** You must set $R$ to be strictly greater than your maximum expected gap size plus your voxel downsampling leaf size.
* **Why it works:** If the gap is 2mm, and you set $R = 5mm$, a point on the edge of Object A will form a spherical search radius that reaches *across* the gap and grabs points from Object B. The covariance matrix is now populated with points from two orthogonal planes, causing the PCA curvature to spike exactly as if they were physically touching.

### Step 2: PCA and Curvature Calculation

Iterate through your merged CAD point cloud. For every point $p_i$:

1. Retrieve all neighbor points within radius $R$ using your SciPy or Open3D KD-Tree.
2. Calculate the centroid ($\bar{p}$) of these neighbors.
3. Compute the $3 \times 3$ covariance matrix $C_{cov} = \frac{1}{N} \sum_{j=1}^{N} (p_j - \bar{p})(p_j - \bar{p})^T$.
4. Perform Eigenvalue Decomposition on $C_{cov}$ to get $\lambda_1, \lambda_2, \lambda_3$ (sorted largest to smallest).
5. Calculate the Surface Variation (Curvature): $V = \frac{\lambda_3}{\lambda_1 + \lambda_2 + \lambda_3}$.

### Step 3: Thresholding

Points on the flat faces of your CAD models will have a Surface Variation $V$ very close to $0.0$. Points on the edges bounding the gap will have a $V$ value typically between $0.05$ and $0.15$.

* Apply a strict `if (V < threshold) { delete }` filter.
* This instantly strips away 99% of your point cloud, leaving only the edges.

### Step 4: Fixing the "Double Line" Artifact (The Gap Consequence)

Because there is a physical gap, Step 3 will actually output **two parallel lines** of high-curvature points—one line on the edge of Object A, and one line on the edge of Object B. A welding robot needs a single toolpath down the middle.

* **The Fix:** Apply a Voxel Grid Downsampler to these remaining edge points, setting the `voxel_size` to be slightly larger than your gap.
* **The Result:** The downsampler will grab the parallel points from Object A and Object B, average them together, and place a single point perfectly in the geometric dead-center of the gap.