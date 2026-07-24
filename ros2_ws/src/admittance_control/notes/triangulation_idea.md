Yes — boolean union, but only if the parts actually overlap or touch. If they're disjoint it degenerates to concatenation, which is still fine and still watertight. Worth pinning that down early because it decides how much of Stage 7 you need.

One expectation to set before the plan: **Delaunay preserves your input points exactly, but it can't recover edges that aren't in the sampling.** If your 2500 points/part came from uniform surface sampling, there are points *near* every CAD edge but none *on* them, so the reconstructed edge will be a jagged chamfer rather than a crisp crease. Delaunay beats Poisson here (Poisson actively rounds), but the honest ceiling is set by your sampling. Delaunay is also *less* forgiving of sparsity than Poisson — it interpolates, so 2500 points shows up directly as visible faceting. If you can resample with edge-aware sampling, that's the single highest-leverage change.

---

## Stage 0 — Split by label

```python
import numpy as np
pts    = np.load("combined.npy")      # (5000, 3)
labels = np.load("labels.npy")        # (5000,) part id
parts  = {l: pts[labels == l] for l in np.unique(labels)}
```

Gate: each part is ~2500 points, and the two bounding boxes tell you immediately whether you're in the overlap case or the disjoint case. Record that answer — Stage 7 depends on it.

## Stage 1 — Normals, per part, exact if possible

If you still have the CAD, carry the face normal from the tessellation you sampled from. Don't estimate. The graph-cut data term is built entirely on normal orientation, so a flipped region here is fatal in a way it isn't for other pipelines.

If you must estimate: `estimate_normals` then `orient_normals_consistently_with_tangent_plane(k=20)` — run it **per part, before any merging**, for the concave-junction reason from before.

## Stage 2 — Delaunay tetrahedralization

```python
from scipy.spatial import Delaunay

def tetrahedralize(P, pad=0.25):
    lo, hi = P.min(0), P.max(0)
    d = (hi - lo) * pad
    corners = np.array(np.meshgrid(*zip(lo - d, hi + d))).T.reshape(-1, 3)
    P_aug = np.vstack([P, corners])          # 8 bbox corners
    tri = Delaunay(P_aug)
    return P_aug, tri, len(P)                # n_real = index cutoff
```

The bounding-box corners matter: Delaunay fills only the convex hull, and you need genuine "outside" tetrahedra to anchor the cut against. Everything from index `n_real` onward is synthetic and never appears in the output surface.

`tri.simplices` gives you (n_tets, 4) vertex indices and `tri.neighbors` gives (n_tets, 4) adjacency with `-1` for hull faces. Those two arrays are your graph.

## Stage 3 — Data term: inside/outside evidence per tetrahedron

The classic Labatut/Vu formulation uses **visibility rays** — camera-to-point segments that must be free space. You have no cameras, so that term doesn't exist. Two workable replacements:

**Preferred — generalized winding number** (Barill et al. 2018). It evaluates inside/outside directly on an oriented point cloud, gives ≈1 inside and ≈0 outside, and is exact for clean normals like yours.

```python
import igl
centroids = P_aug[tri.simplices].mean(axis=1)
areas = np.full(len(P), spacing**2)          # per-point area estimate
w = igl.fast_winding_number(P, N, areas, centroids)   # check binding signature
```

**Fallback — KD-tree normal vote**, if the libigl binding is awkward to install:

```python
from scipy.spatial import cKDTree
tree = cKDTree(P)
dist, idx = tree.query(centroids, k=16)
signed = np.einsum('ijk,ijk->ij', centroids[:,None,:] - P[idx], N[idx])
wgt = 1.0 / (dist + 1e-9)
score = (signed * wgt).sum(1) / wgt.sum(1)   # >0 outside, <0 inside
w = 1.0 / (1.0 + np.exp(score / spacing))    # squash to [0,1]
```

Less accurate far from the surface, but the smoothness term absorbs most of that.

## Stage 4 — Graph cut

```python
import maxflow
INF = 1e9

g = maxflow.Graph[float]()
nodes = g.add_nodes(len(tri.simplices))

# source = OUTSIDE, sink = INSIDE
lam = 1.0
for t in range(len(tri.simplices)):
    g.add_tedge(nodes[t], lam * (1 - w[t]), lam * w[t])

# hard-constrain hull tets to outside
for t, nb in enumerate(tri.neighbors):
    if (nb == -1).any() or (tri.simplices[t] >= n_real).any():
        g.add_tedge(nodes[t], INF, 0)

# pairwise: shared-facet area penalises surface area
lam_s = 2.0
for t, nb in enumerate(tri.neighbors):
    for f in range(4):
        u = nb[f]
        if u > t:
            a = facet_area(P_aug, tri.simplices[t], f)
            g.add_edge(nodes[t], nodes[u], lam_s * a, lam_s * a)

g.maxflow()
inside = np.array([g.get_segment(nodes[t]) for t in range(len(nodes))]) == 1
```

`lam_s / lam` is your only real tuning knob — raise it for smoother, more watertight-robust output, lower it to keep detail. Sweep it over maybe 0.5–5 and look at the results.

## Stage 5 — Extract the interface

Every facet separating an inside tet from an outside tet is a surface triangle. Orient each so the normal points from the inside tet toward the outside tet — get this wrong and every downstream volume check reports negative.

Gate: `trimesh.Trimesh(...).is_watertight` should be `True` here already. If it isn't, the bug is in Stage 5's facet enumeration, not in the cut.

## Stage 6 — Manifold repair

**This is the step people skip and regret.** The cut boundary is guaranteed closed, but *not* guaranteed manifold — two solid regions can meet at a single vertex or edge, which is watertight but non-manifold. `manifold3d` will reject it outright.

```python
import pymeshlab
ms = pymeshlab.MeshSet()
ms.add_mesh(pymeshlab.Mesh(V, F))
ms.meshing_repair_non_manifold_edges()
ms.meshing_repair_non_manifold_vertices()
ms.meshing_remove_duplicate_vertices()
```

Gate before proceeding: `is_watertight and is_winding_consistent and mesh.volume > 0`, plus Euler characteristic sanity.

## Stage 7 — Boolean union

```python
from manifold3d import Manifold, Mesh

def to_manifold(tm):
    return Manifold(Mesh(tm.vertices.astype('float32'),
                         tm.faces.astype('uint32')))

result = to_manifold(mesh_a) + to_manifold(mesh_b)
out = result.to_mesh()
```

`manifold3d` is the right pick — fast, and it guarantees manifold output rather than hoping for it. CGAL's `corefine_and_compute_union` is the alternative if you're already in that ecosystem.

If the parts turned out disjoint at Stage 0: union produces two components in one mesh. That is watertight and correct. "Single mesh" and "single connected component" are different requirements and you almost certainly only need the first.

## Stage 8 — Final validation

`is_watertight`, `is_volume`, positive volume, no self-intersections, component count matches expectation. Then compare against a ground-truth boolean union of the original CAD B-reps if you have it — Hausdorff distance and volume ratio will tell you honestly what the reconstruction cost you.

---

## Where this will actually hurt

The two failure modes, ranked by likelihood: **non-manifold output at Stage 6** (very common with graph-cut methods, and it hard-blocks the boolean), and **faceted, jagged surfaces from 2500-point density** (unavoidable given the input; Delaunay interpolates rather than smooths).

If Stage 6 turns into a fight, CGAL's `alpha_wrap_3` is a strong escape hatch — it guarantees watertight, manifold, self-intersection-free output in one shot. The cost is that it wraps at a small offset from the true surface rather than interpolating your points exactly, which trades away some of what you chose Delaunay for. Worth knowing it exists before you sink a day into manifold repair.