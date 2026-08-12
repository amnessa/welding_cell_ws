Nothing fundamental changes, but two things get sharper: the literature hands you the nominal orientation for free, and I was wrong to reach for greedy submodular selection — your problem has 1D structure that admits an exact solution.

## Stage 1 — the quality field

**Parameterization.** Two variables: arc length $s$ along the extracted centerline, and roll $\phi$ about the tool axis.

The nominal orientation is already settled in the literature, so don't reinvent it. Yi et al. compute the weld direction as $\mathbf{d}_{weld} = \mathbf{n}_1 \times \mathbf{n}_2$ from the two fitted plate normals, and the approach vector as $\mathbf{v}_0 = \mathbf{v}_1 + \mathbf{v}_2$, the dihedral bisector. Wang et al. use the same construction, $\mathbf{Z}_W = -(\vec{n_b} + \vec{n_m})$. Take that as $R_{nom}(s)$, then

$$R(s,\phi) = R_{nom}(s) \cdot \text{Rot}_z(\phi)$$

Cite them for the nominal, and the free $\phi$ is where you depart.

**Field components at each $(s,\phi)$:**

- **IK feasibility** — hard gate. UR5e has closed-form IK with 8 branches; record which branch, you need it in Stage 3.
- **$\sigma_{\min}(J)$** — not $|\det J|$. On a 6-DOF arm the determinant mixes m and rad units and the scalar is physically meaningless.
- **Force transmission along the contact normal** — $1/\sqrt{\mathbf{n}^\top (J_v J_v^\top)\,\mathbf{n}}$, using the translational block only. This is the one that ties to your admittance controller, and it's the *dual* of the velocity ellipsoid: directions where the arm pushes hard are directions where it moves slowly. Worth saying explicitly in the paper, because manipulability work almost always optimises the velocity ellipsoid, which is the wrong one for contact.
- **Joint-limit margin** — distance to nearest limit, normalized.
- **Camera terms** — seam neighbourhood inside frustum, standoff within the depth sensor's valid range, incidence angle away from grazing, and self-occlusion by the pen holder. Check the minimum-range one first; at close working distance you may be inside the camera's blind zone for entire bands of $\phi$.
- **Collision clearance** — with the camera mounted, your widest geometry is probably the camera body, not the tool tip, and the vertical plate of a T-joint restricts the approach cone. Verify this before optimising anything.

Normalize each to $[0,1]$ and store the vector. That vector *is* the robot-agnostic interface from last message — everything downstream sees numbers attached to 3D positions, never joint angles.

**Computation.** Grid it: $s$ at 2 mm over 232 mm gives 116 samples, $\phi$ at 5° gives 72. About 8,400 poses. Closed-form IK plus a Jacobian each — vectorized NumPy handles it in well under a second. No need to be clever.

**Collapsing $\phi$.** You want one roll per arc position. The obvious move is $\phi^*(s) = \arg\max_\phi Q(s,\phi)$ independently, but that can flip the wrist between neighbouring positions. Better: solve a shortest path over the $(s,\phi)$ grid with a transition penalty $\mu|\Delta\phi|$ and a hard block on IK-branch changes. It's a small DP, it's exact, and it gives you a smooth roll profile along the seam. For discrete tacks the robot retracts between points so continuity matters less than in continuous welding — but branch consistency still does.

## Stage 2 — selecting k tacks

Here's the correction. I previously suggested greedy submodular maximization with the $(1-1/e)$ guarantee. That's the right tool for facility location in general, but **your candidates lie on a line**, and 1D with spacing constraints is exactly solvable by dynamic programming.

Candidates $s_1 < s_2 < \dots < s_n$. Hard constraints: both endpoints mandatory (standard welding practice), spacing $d_{\min} \le \Delta s \le d_{\max}$, no tack within $\sim 2t$ of a free edge, IK feasible.

$$f(i,j) = q(s_i) + \max_{m \;:\; d_{\min} \le s_i - s_m \le d_{\max}} \Big[ f(m, j-1) - \lambda\, g(s_m, s_i) \Big]$$

where $g$ penalises non-uniform spacing. The key point: because $g$ is **pairwise**, it stays inside the DP recursion. You get uniformity for free without losing exactness.

Complexity $O(n^2 k)$ — with $n=116$, $k\le15$, that's ~200k operations. Milliseconds, and it's the global optimum, not a bound. Say so in the paper; exactness is worth a sentence.

**Choosing $k$.** Either sweep it, or drop the $j$ index and put a fixed cost $c$ per tack, letting the DP choose. The lower bound is $k_{\min} = \lceil L/d_{\max}\rceil + 1$. Tomków's measurement that five tacks gave *higher* angular distortion than fewer is your argument for a real penalty on $k$ rather than "more is safer."

**Where the distortion literature enters.** Only through $d_{\min}$, $d_{\max}$, the edge margin, and $c$. All constants, all citable, no FEM. This is the concrete form of what I said last message.

## The experiment that decides everything

Build the field first and **plot $q$ across the seam before writing any selection code.** If $\sigma_{\min}$ and the force-transmission term vary by less than ~10% over a 232 mm seam in a comfortable UR5e workspace, the kinematic part of your contribution has no signal, and you'd rather know in week two than week ten.

My honest expectation: the conditioning terms will be fairly flat, and the **camera visibility and collision terms will carry most of the variation** — because the vertical plate physically blocks approach angles, and that's a hard geometric effect rather than a soft numerical one. If that's how it comes out, it's still a good paper; the framing just shifts from "manipulability-aware" to "visibility-and-reachability-aware," which is arguably more honest and more distinctive anyway. But plan for it now.

Second thing to check early: mount the camera and measure the actual feasible $\phi$ range against the vertical plate. If it's narrow, your 5° grid may be too coarse to resolve the window.