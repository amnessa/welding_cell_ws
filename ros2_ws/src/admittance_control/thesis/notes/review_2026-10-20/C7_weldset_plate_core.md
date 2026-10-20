# WeldSet plate-joint core: material map (read-only audit, 2026-10-20)

**Scope and method.** I read every line of `weldgen/geom.py` (2078), `joints.py` (231), `layouts.py` (659), `config.py` (515), `scene.py` (643) and `accessibility.py` (782). I also read the requested docs sections and the eight requested test files, plus relevant parts of `docs/MANUAL.md`, `README.MD`, the notes, the presets and `out/bench_phase4`.

Every behavioural claim marked **[verified]** was checked with read-only `python3 -B -c` runs (no `.pyc`, no files written, no git changes). All paths are relative to `/workspaces/welding_cell_ws/weld_generator/`.

"Local corpus" means `out/bench_phase4` in this workspace. Project notes say a laptop `out/` may be stale, but every scene I regenerated with the current code matched its stored copy.

---

## 0. Findings to put in front of the thesis author first (all [verified])

1. **The two workpieces often interpenetrate.** This contradicts D21 and SCHEMA §6.3 L931 ("the union is genuinely disjoint at g > 0"). The cause is that the rotation hinge sits on the wrong side in some configurations.
   - **T with α < 90°.** Tilt is θ = α − 90 + β < 0. B is hinged at its *near*-bottom edge (`layouts.py:280-284`, `joints.py:161-167`), so its far-bottom edge sinks to z = g + t_B·sin θ.
     - Penetration depth: $\max(0,\,t_B|\sin\theta|-g)$.
     - Sampler, bench_T / bench6a_T (4000 specs): 28.3% of specs, median 1.0 mm, p90 3.0, max 5.4 mm.
     - Local corpus: 18 of 60 plate-T scenes, median 1.58 mm, max 3.10 mm.
   - **Edge and lap.** The hinge is at the welded edge for edge (`layouts.py:649`) and mid-overlap for lap (`:627`). β > 0 rotates B's far side **down** into A.
     - Example: H_B = 200, β = 0.4°, g = 0.1 puts B's far-bottom edge at z = −1.296 mm, inside a 2 mm A.
     - Local corpus: edge 36/60 scenes (median ≥ 0.45 mm; this is a lower bound on thin sheet), lap 10/60 (median 0.16, max 0.31 mm).
   - **Square butt:** rare (1.3% of specs, ≤ 0.19 mm; 0/120 local). **Corner:** never.
   - **Consequences:**
     - The acute-side fillet of an acute T has root = gap_mid = nominal (offset 0), which biases the D19 statistics.
     - When B pierces a thin A, phantom *primary* seams appear on A's underside. Example: bench6a_T seed 33 gives scene `9a8eb697-0000000033` with a primary fillet `A:-w × B:-w`. Rate is about 0.75% of T specs; 0 of 60 in the local corpus.
     - Buried surface points stay in the cloud (`sampling.py:134-151` does not filter them).
   - **Docs that state the opposite:**
     - The test docstring at `test_joint_types.py:298-299` says "the FAR edge opens"; it sinks.
     - The D23 rationale talks about "lifting" the far edge.
     - Disjointness is only tested at α = 90° (`test_geometry.py:32-40`).

2. **Coplanar-arm bug at `accessibility.py:572`.** The line is `if best is None or (hi - lo) > best[4] - best[3]:`. `best[4] - best[3]` is `lo − mid` of the previous best, which mixes two axes. The intended comparison is `best[5] - best[4]`, the previous run length.
   - It only bites when A is not a `Slab` (`Prism` / `PreparedPrism`, i.e. outlines on). That covers the Phase 4 butt-square (bench6a_butt) and grooved strata.
   - 250-seed samples, with in-memory corrected copy for comparison:
     - bench6a_butt: 9 of 500 top/bottom pairs give a skewed centreline (0.02–0.84° off the seam edge, lateral deviation up to 7.3 mm).
     - grooved: 10 of 412 centrelines dropped and 6 of 412 skewed (up to 3.3°).
   - Local corpus: 7 of 234 stored weldable butt centrelines are skewed 0.05–1.19° against A's seam edge (≈0.05–1.1 mm at the ends).
   - 3 of 60 local grooved scenes have **no** mouth centreline at all (`c7bd0d37-0002000305`, `-033`, `-221`). The current code reproduces these exactly.
   - Even the documented criterion "widest shared run wins" (`:526-531`) once picked a 2.9° skewed axis. It does not guarantee the seam-edge perpendicular.

3. **Grooved butt: ISO row chosen at the wrong thickness.** The row is chosen at min(t_A, t_B), then the code forces t_B := t_A (`config.py:491-498`). When t_A > t_B the plate ends up thicker than the row allows.
   - 6000 grooved_butt draws: 714 have final t ≠ the selection t.
     - 145 single-bevel plates are > 10 mm, outside row 1.9.1's 3 < t ≤ 10 (up to 19.9 mm).
     - 128 single-V plates carry `iso_ref` "1.3" at t > 10 (should be 1.5; the parameters drawn are row 1.3's).
   - Local corpus examples: `c7bd0d37-0002000113` (single_bevel at 17.0 mm), `c7bd0d37-0002000297` (single_V, ref 1.3, at 15.6 mm).

4. **Grooved scenes ship two primary butt seams, not one.** They are the top/mouth centreline `A:+w×B:+w` and the underside `A:-w×B:-w`. Both get the *same* `_grooveroot` array (`scene.py:424-439` applies to any butt-class seam).
   - Local corpus: 57 of 60 grooved scenes have both; 3 have the underside only.
   - This contradicts "a groove gives the butt joint one seam at the groove root" (PARAMETERS §2.6.1 L198-200; D36).

5. **Some defects are stored but never built into the geometry.**
   - `h` is not used by `_layout_lap` or `_layout_edge` (`layouts.py:613-650`).
   - β is not used by `_grooved_butt` (`:552-584`).
   - For T, `h` shifts B along **y** (`:503`), not z.
   - All of them are still stored in `fit`, used in `classify_quality`, and used to widen `contact_tol`.

6. **Quality level applies the fillet no. 617 gap limit to every joint type** (`config.py:290-316`), including butt, edge and grooved butt. PARAMETERS §2.4 L125-127 and the `root_gap_limit` docstring (`config.py:200-201`) both say 617 does not apply to butt/edge.
   - Grooved sampler mix: B 5% / C 20% / D 33% / below_D 42%.
   - Local grooved stratum: B 2 / C 12 / D 14 / below_D 32 out of 60.

7. **Effective dihedral floor is 60°, not 30°.** The 30° half-angle cone blocks any open nook with dihedral ≤ 60° (59° blocked, 60.5° clears; exactly 60.0° is a floating-point tie).
   - Obtuse T joints also lose their obtuse-side fillet silently. Its terminating edge lifts by t_B·sin θ, so the separation gate drops it with no reject reason.
   - α > 95° (400-spec sample): 48% have one primary fillet, 5% have none.
   - At α = 120° the test spec had no weldable fillet at all.

8. **D19 `gap_mid` has three different definitions.**
   - SCHEMA §1.3 L143: closest-point pairs between the two faces.
   - Pipeline `accessibility.d19_curves` (`:449-476`): midpoint of each face's closest point to the nominal sample.
   - Phase 1 `joints._fillet` (`:214-216`): root and its projection onto Π_A.
   - They agree only at 90°. Example at α = 120°, g = 1.1: pipeline 0.635 mm vs closest-pair 0.840 mm.

9. **Not every rejected candidate is stored.**
   - Facing pairs (no line, including the buried lap interface) are filtered out of `seams[]` (`scene.py:411-414`).
   - Pairs that fail adjacency are dropped silently.
   - `no_contact` and `interior_face` are never emitted.
   - Off-class seams are not "rejected": they stay `weldable: true` with `matches_joint_type: false`.

---

## A. Per module

### A.1 `geom.py`: primitives, face registries, exact algebra, meshes, `from_object`

**Purpose.** Each part is a primitive with a fixed local frame and face registry. The invariant is that **w is always the thickness axis** (`:3-9`), so the broad faces are ±w. The module also gives exact planes, analytic `contains`, exact face clipping for D4, watertight meshes for D21, and the inverse of the serialiser.

1. **Transforms** `rot_x/rot_y/rot_z/translate` (`:38-71`): 4×4, angles in degrees.
2. **`Plane`** (`:74-82`): $\Pi=\{\mathbf p:\mathbf n\cdot\mathbf p+d=0\}$ with n the unit **outward** normal. Signed distance is $\mathbf n\cdot\mathbf p+d$.
3. **`Slab`** (`:85-277`): centred box, u ∈ [−L/2, L/2], v ∈ [−W/2, W/2], w ∈ [−t/2, t/2]. All placement lives in `T_world_part`.
   - Mesh: `trimesh.creation.box` (`:109-113`).
   - Contains (`:115-125`): exact $|R^\top(\mathbf p-\mathbf t)|\le \tfrac12(L,W,t)+\mathrm{tol}$, done without rtree (D9).
   - Face normal is $R\,\mathbf n_{loc}$; face plane uses $d=-\mathbf n\cdot\mathbf c_{face}$ (`:145-159`).
   - `closest_on_face` clamps to the rectangle (`:130-143`).
   - `face_clip_line` (`:173-250`) gives the exact 2-D parameter interval of $\mathbf p+s\hat{\mathbf d}$ over the face rectangle:
     - For each in-face axis k: if $|d_k|<10^{-12}$ the line is parallel and must satisfy $|p_k|\le h_k+\epsilon+10^{-9}$, with ε = slack = `contact_tol`. Otherwise the interval is $[\min(a,b),\max(a,b)]$ with $a=(-h_k-p_k)/d_k$, $b=(h_k-p_k)/d_k$.
     - The axis intervals are intersected.
     - If the result is empty and ε > 0, a **near-parallel waiver** applies: one axis may be dropped if its breach stays within $h_k+\epsilon$ over the run the other axis allows. The run is never extended (`:228-250`).
   - `face_extent_along` (`:252-277`) is the legacy 1-D corner projection. The coplanar and facing arms still use it.
4. **`Prism`** (`:280-544`, Phase 6a, D28): convex outline in u-v, forced CCW (`:305-314`), w ∈ [−t/2, t/2].
   - Faces: `+w`, `-w` caps and `s0..s{k-1}` sides.
   - Side outward normal $\mathbf n_2=(e_y,-e_x)$ (`:339-346`).
   - Contains: all half-spaces satisfy $\mathbf n\cdot\mathbf p+d\le \mathrm{tol}+10^{-9}$ (`:508-520`).
   - Clip (`:412-470`): half-plane parameter clip, slack on parallel constraints, and the same waiver.
   - Mesh: fan caps plus side quads built by hand, no shapely (`:522-544`).
   - `part_geometry_id` is `prism_<shape>_<k>x<L>x<W>x<t>`.
5. **`Tube`** (`:547-793`, Phase 6b, D29 #2–#4): faces `lateral+`, `lateral-`, `+w`, `-w`; w is the axis.
   - Optional `base_cut`:
     - plane: $z=(d-n_xx-n_yy)/n_z+g$.
     - cylinder saddle: $Az^2+Bz+C=0$ with $A=1-m_z^2$, first contact root $z=(-B+\sqrt{B^2-4AC})/2A+g$ (`:612-650`).
   - Mesh ring count: $n_\phi=\max(48,\lceil 2\pi/\sqrt{8\cdot0.05/r}\rceil)$, giving chord error $r\Delta\phi^2/8\le0.05$ mm (D34, `:761-767`).
6. **`SweptSlab`** (`:796-1067`, Phase 6b, D29 #5–#7): a band $[o_{lo},o_{hi}]$ offset along $\hat{\mathbf n}=\hat{\mathbf z}\times\hat{\mathbf T}$ about a spine, extruded over z ∈ [z0, z1].
   - Faces: `+w`/`-w` are the offset surfaces, `+v`/`-v` the z1/z0 caps, `+u`/`-u` the ends (open spine only).
   - Self-intersection guard: $1-\kappa o>0.05$ (`:840-844`).
   - `contains` uses the fine polyline, a boolean approximation (`:990-1013`).
7. **`PreparedSlab`** (`:1070-1474`, Phase 6b step 4, D35).
   - Frame: u ∈ [−L/2, L/2]; v ∈ [−W, 0] with the prepared edge at v = 0; w ∈ [−t, 0] with the top at 0.
   - Validation: 0 < c < t; for U, the straight run $t-c-R(1-\sin\beta)>0.5$ mm (`:1107-1123`).
   - The profile `v_edge(w)` (`:1126-1142`) has root-face top $w_r=-(t-c)$:
     - V / bevel: $v_e(w)=0$ for $w\le w_r$, and $v_e(w)=-(w-w_r)\tan\beta$ above it. β is α/2 for V and the full angle for bevel.
     - U: an arc centred at $(0,\,w_r+R)$, $v_e=-\sqrt{R^2-(w-w_c)^2}$, up to $w_a=w_r+R(1-\sin\beta)$ where $v_a=-R\cos\beta$. Above that, $v_e=v_a-(w-w_a)\tan\beta$.
     - Mouth: $v_e(0)$; for V this is $-(t-c)\tan\beta$.
   - Faces: `+u -u -v +w -w root fusion` (+`radius` for U; that face has no plane and a concave cylinder `surface`).
   - Contains is exact: $v\le v_e(w)$ (`:1369-1380`).
   - End caps and `radius` never clip a seam line (`:1321-1332`).
   - Mesh: extruded profile, fan end caps from the back-bottom corner (`:1449-1474`).
8. **`PreparedPrism`** (`:1477-1985`, added 2026-09-07): a D28 outline whose seam edge carries the PreparedSlab profile. An internal `PreparedSlab` acts as the edge oracle.
   - `__post_init__` (`:1520-1542`): rolls the outline so the v = 0 edge is edge 0, requires material at v ≤ 0, and enforces a mouth guard (no other vertex shallower than the mouth).
   - Slices are the outline clipped by $v\le v_e(w)$; only the two edges adjacent to the seam edge are cut (`:1562-1575`). The mesh is a loft of same-count slices and is watertight (`:1965-1985`).
   - Faces: `+w -w root fusion (radius) s1..s{k-1}`. `s0` is the prepared edge and is not a face; `s1` and `s{k-1}` are the notched ends.
9. **Algebra.**
   - `intersect_planes` (`:1988-2003`): $\hat{\mathbf d}=\frac{\mathbf n_A\times\mathbf n_B}{\|\mathbf n_A\times\mathbf n_B\|}$, with the point solving
     $$\begin{bmatrix}\mathbf n_A^\top\\\mathbf n_B^\top\\\hat{\mathbf d}^\top\end{bmatrix}\mathbf p_0=\begin{bmatrix}-d_A\\-d_B\\0\end{bmatrix}$$
     (the point on the line closest to the origin). Returns None if the cross product is < 1e-9.
   - `dihedral_deg` (`:2006-2014`): $\varphi=180^\circ-\arccos(\mathbf n_A\cdot\mathbf n_B)$.
   - `approach_dir` (`:2017-2023`): $\hat{\mathbf b}=\frac{\mathbf n_A+\mathbf n_B}{\|\mathbf n_A+\mathbf n_B\|}$, pointing seam → torch. Torch z is −b (SCHEMA §1.2). Raises if the normals are antiparallel.
10. **`from_object`** (`:2035-2077`): rebuilds each of the six `PRIMITIVES` (`:2032`) from a `scene.json` entry; it is the exact inverse of the serialisers.

### A.2 `joints.py`: `JointSpec`, tilt relative to each type's nominal, Phase-1 T constructor

1. **`JointSpec` fields** (`:46-83`):
   - Required: `L_A, W_A, t_A, L_B, H_B, t_B, root_gap_mm (g), linear_misalignment_mm (h), angular_misalignment_deg (β), included_angle_deg (α)`.
   - Defaults: `stack_offset_mm=None`, `length_offset_mm=0.0`, `in_plane_yaw_deg=0.0`, `outline_A/B=None`, `outline_shape_A/B=None`, `prep="square"`, `groove=None`.
2. `NOMINAL_INCLUDED_DEG = {T: 90, corner: 90, butt: 180, lap: 0, edge: 0}` (`:88-89`).
3. `tilt_deg` (`:91-106`): $\theta=(\alpha-\alpha_{nom}(J))+\beta$. For butt, lap and edge, α = α_nom, so θ = β.
4. Throat (`:108-111`): $a_A=0.7\,\min(t_A,t_B)$, [ours] (PARAMETERS §2.4).
5. `build_t_joint` (`:143-177`) is the **Phase-1 path and is used only by `tests/test_geometry.py`**; the pipeline uses `layouts.build` plus D4.
   - $T_A=T\,\mathrm{Tr}(0,0,-t_A/2)$.
   - $T_B=T\,\mathrm{Tr}(0,h,g)\,R_x(\theta)\,\mathrm{Tr}(0,t_B/2,H_B/2)\,R_x(90^\circ)$.
   - `B:+w` is the near face (y = h before tilt), `B:-w` the far face.
6. `_fillet` (`:180-226`):
   - nominal = Π_A ∩ Π_B, clipped by 1-D projected extents;
   - root = Π_{B,face} ∩ Π_{B,-v};
   - gap_mid = $\mathbf r-\tfrac12(\mathbf n_A\cdot\mathbf r+d_A)\mathbf n_A$ (`:229-231`).

### A.3 `layouts.py`: placement only (no seams)

Canonical frame: +X along the seam, +Z up out of A's top, +Y toward B (`:3-7`). Δ = length offset, ψ = yaw.

1. **A** (`_base_A`, `:315-323`): $T_A=T\,\mathrm{Tr}(0,y_c,-t_A/2)$, so A's `+w` is z = 0.
   - y_c = 0 for T; −W_A/2 for corner, butt, lap and edge (A then occupies y ∈ [−W_A, 0]).
   - With outlines on, A is a `Prism` (corner, butt, edge only) with the seam at +v.
2. **T** (`:497-505`): $T_B=T_\psi\,\mathrm{Tr}(\Delta,h,g)\,R_x(\theta)\,\mathrm{Tr}(0,t_B/2,H_B/2)\,R_x(90^\circ)$.
   - $T_\psi$ rotates about the vertical line through (Δ, h + t_B/2) (`_yawed`, `:326-334`).
   - The hinge is the line (y = h, z = g). **h is along y.**
   - Fillets: `A:+w×B:+w` with dihedral 180 − α − β, and `A:+w×B:-w` with α + β [verified].
3. **Corner** (`:508-526`): $T_B=T\,\mathrm{Tr}(\Delta,g,h)\,R_x(\theta)\cdots$, so B stands beside A at y ≥ g, bottom at z = h. **h is along z.**
   - Inside fillet `A:+w×B:+w` has dihedral **180 − α − β**; outside edge weld `A:+v×B:-v` has **α + β** [verified]. The recorded α is therefore the *outside* angle.
4. **Butt, square** (`:587-610`): B is flat with centre $(\Delta,\,g+H_B/2,\,z_c)$, $z_c=-t_A+t_B/2+h$, pivot $(g,z_c)$.
   - Plates are flush on the **underside**; h steps B up.
   - Equal thickness gives two centrelines (`+w`/`+w` and `-w`/`-w`); dissimilar thickness gives only the underside one.
5. **Butt, grooved** (`_grooved_butt`, `:552-584`): A is a `PreparedPrism` (or `PreparedSlab`) at T.
   - V/U: B is the same preparation at $T\,\mathrm{Tr}(\Delta,g,h)R_z(180^\circ)$.
   - Bevel: B is square at $T\,\mathrm{Tr}(\Delta,g+H_B/2,h-t_B/2)$.
   - **No β is applied.** `_prepared_plate` (`:537-549`) falls back to `PreparedSlab` if the mouth guard fails.
6. **Lap** (`:613-628`): yaw about (Δ, −s/2). B occupies y ∈ [−s, H_B − s], z ∈ [g, g + t_B].
   - $T_B=T_\psi\,\mathrm{Tr}(\Delta,-s/2,g)R_x(\beta)\mathrm{Tr}(0,H_B/2-s/2,t_B/2)$. **No h.**
   - Toes: `A:+w×B:-v` (upper) and `A:+v×B:-w` (underside). The interface `A:+w×B:-w` is a facing pair.
7. **Edge** (`:631-650`): B occupies y ∈ [−H_B, 0], z ∈ [g, g + t_B], flush at y = 0.
   - $T_B=T\,\mathrm{Tr}(\Delta,0,g)R_x(\beta)\mathrm{Tr}(0,-H_B/2,t_B/2)$. **No h.**
   - Seam `A:+v×B:+v` (coplanar), plus `A:-v×B:-v` when H_B = W_A.
8. **Outlines** (`sample_outline`, `:183-247`): the seam edge (−L/2, 0)→(L/2, 0) is pinned at full length; depth is W_A (for A) or H_B (for B).
   - Shapes and weights `(trapezoid, parallelogram, triangle, quad, convex) = (0.10, 0.10, 0.25, 0.30, 0.25)`. Measured over 20,000 draws: .107 / .095 / .252 / .298 / .248.
   - trapezoid: a, b = ±U(.15,.55)L; if the far edge is under 0.2L it is recentred.
   - parallelogram: shear ±U(.25,.65)L.
   - triangle: apex at U(−.4,.4)L.
   - quad: one far vertex at full depth, the other at U(.55,.85)·depth, u offsets U(−.2,.35)L; up to 8 convexity redraws.
   - pentagon: (u_r, d_r), (u_m, depth), (u_l, d_l) with offsets U(−.05,.3)L, d U(.35,.75), u_m U(−.3,.3)L.
   - `_outline_local` (`:250-261`) mirrors v for the seam-at-+v side. **Then `Prism` re-orders the vertices to CCW, so the seam edge becomes `s{k-2}`, not `s0` [verified].**
9. **Yaw** (D28; `_yaw_support_geometry`, `:395-434`; `feasible_yaw_deg`, `:444-478`; `sample_yaw_deg`, `:481-494`).
   - Feasible set: 1° grid over [−180, 180). An angle is feasible if the chords through the contact-strip lines, inside A's rectangle shrunk by 1 mm, are all ≥ 0.5·min(L_A, L_B).
   - T strip lines: offsets {−(t_B/2 − 1), 0, +(t_B/2 − 1)} about y = h + t_B/2.
   - Lap strip lines: {−(w/2 − 1), 0} about y = −w/2 with w = min(s, W_A); only {0} if s ≥ W_A.
   - Draw: a random cell plus U(−0.5°, 0.5°); an empty set gives 0 with no draws.
   - `max_supported_yaw_deg` (`:337-392`) is legacy and test-only.
10. **D31** `class_ambiguity` (`:57-151`): clearance $c=2\min(t_A,t_B)$ (`:37-43`); near-parallel means < 10° (`:34`); segment distance is sampled at 17 points.
    - T is flagged "corner" if B's contact centreline runs near-parallel within c of an A edge.
    - Lap is flagged "edge" if any edge of B's yawed footprint runs near-parallel within c of an A edge.
    - Edge is flagged "lap" if $0<|W_A-H_B|<c$ or an end edge lies within (1e-9, c) of an A end.
    - Corner and butt have no bands.

### A.4 `config.py`: `DEFAULT_CONFIG`, ISO limits, D35 preps, classification, sampler

1. **`DEFAULT_CONFIG`** (`:25-151`):

   | Key | Value |
   |---|---|
   | `joint_type` | [T, corner, butt, lap, edge] |
   | `plate_length_mm` | [80, 400] |
   | `min_overlap_frac` | 0.6 |
   | `in_plane_yaw`, `polygon_outlines`, `class_disjoint`, `class_boundary_stratum`, `groove_preps` | False |
   | `plate_width_mm` | [50, 250] |
   | `thickness_mm` | [1, 12] |
   | `dissimilar_thickness_p` | 0.30 |
   | `included_angle_deg` | [60, 120] |
   | `stack_offset_frac` | [0.15, 0.60] |
   | `edge_max_thickness_mm` | 2.0 |
   | `stacked_max_beta_deg` | 0.4 |
   | `edge_equal_width_p` | 0.35 |
   | `quality_mix` | 0.25 each |
   | `root_gap_mm` | [0, 2] |
   | `root_gap_over_range_mm` | 3.0 |
   | `fixture_present` | False |
   | `fixture_tilt_deg` | 10 |
   | `fixture_height_mm` | [−50, 50] |
   | `fixture_dims_mm` | L [480, 720], W [320, 480], t [8, 12] |
   | `assembly_translation_mm` | 150 |
   | `density_per_mm2` | [0.25, 4] |
   | `elevation_deg` | [15, 85] |
   | `camera_roll_deg` | ±15 |
   | `framing_frac` | [0.35, 1.45] |
   | `require_visible_seam` | True |
   | `min_visible_fraction` | 0.1 |
   | `aim_jitter_frac` | 0.15 |
   | `accessibility` | `{torch_clearance {half_angle 30, standoff 15}, max_work_angle 45, dihedral 30–170, contact_tol 4.0, coplanar_tol 0.5, cross_run_tol 45, min_seam_length 10}` |

2. **Linear misalignment, ISO 5817 no. 5071** (`:188-194`): t is the smaller thickness.
   - t ≤ 3: $\{0.25, 0.15, 0.10\}t+0.2$ for D/C/B.
   - t > 3: $\min(\{0.25, 0.15, 0.10\}t,\ \{5, 4, 3\})$.
3. **Root gap, no. 617** (`:197-209`), with $a=0.7\,t_{min}$:
   - t ≤ 3: $0.1a+\{0.5, 0.3, 0.2\}$.
   - t > 3: $\min(\{0.3, 0.2, 0.1\}a+\{1, 0.5, 0.5\},\ \{4, 3, 2\})$.
4. **Angular misalignment, no. 508** (`:212-219`): D 4°, C 2°, B 1°. This is [ours]: Annex B fatigue values mapped onto quality levels, with D doubled.
5. **`valid_preps(t)`, D35** (`:222-245`):

   | t | Pool |
   |---|---|
   | ≤ 4 | square |
   | (4, 8] | square, single_bevel, single_V |
   | (8, 10] | single_bevel, single_V |
   | (10, 12] | single_V |
   | > 12 | single_V, single_U |

   Refs: square "1.2.1/2.1", bevel "1.9.1", V "1.3" if t ≤ 10 else "1.5", U "1.6/2.6".
6. **`sample_groove`** (`:248-287`): uniform choice from the pool, then the row's draws (listed in C.2).
7. **`classify_quality`** (`:290-316`): see C.4.
8. **`assert_accessibility_covers_gap`** (`:327-342`): **never called**, and it would raise on `DEFAULT_CONFIG` (4.0 < 6.0) [verified].
9. **`sample_joint` draw order** (`:345-515`):
   - g0 = `joint_config`: type; L_A; L_B; W_A; H_B; [edge equal-width coin]; t_A (edge range clamped to [min(lo, 2), 2]); dissimilar coin; [t_B]; [α for T/corner]; [lap overlap fraction].
   - g1 = `defects`: target level; h, β, g (C.4).
   - Then: stacked β clamp; `_draw` overrides when the keys are set.
   - g0 again: Δ ~ U(±0.4·min(L_A, L_B)).
   - g2 = `seam_curve`: yaw; outline A; outline B; groove (which overrides t_B := t_A and g := the row's b).
   - Returns `(spec, classify_quality(...), joint_type)`.

### A.5 `scene.py`: `generate_scene(cfg, seed)` (`:218-643`)

1. `Streams(seed)` (8 SeedSequence children), then `sample_joint`, which uses substreams 0, 1 and 2.
2. D31/D32 acceptance, consuming no draws (`:226-240`). `AmbiguousClassConfiguration` if `class_disjoint` and ambiguous; `ClassBoundaryMiss` if the stratum flag is on and the seed is clean.
3. Placement, substream 3 (`:245-256`):
   $$T_{wj}=\mathrm{Tr}(U_{\pm150},U_{\pm150},0)\,\mathrm{Tr}(0,0,U_{\pm50}\ \text{if fixture})\,R_z(U_{0,360})\,R_x(U_{\pm\tau})\,R_y(U_{\pm\tau})$$
   with τ = 10° with the fixture and 0 without. The z draw is consumed and zeroed. The height draw happens only with the fixture, so twin arms get different yaw [verified].
4. `build_layout` (`:258`). With the fixture on: fixture dims U(480,720) × U(320,480) × U(8,12); its top is placed at the lowest workpiece vertex along $\hat z_{joint}$; `contact_mode` "flat", otherwise "free" (`:260-270`).
5. D21: every mesh must be watertight and winding-consistent (`:272`, `:109-121`).
6. Accessibility parameters (`:280-304`):
   $$\tau_c=\mathrm{clip}\big(\min(2g+|h|+0.5,\ \max(0.95\,t_{min},\ 1.1(g+|h|))),\,0.5,\,12\big)$$
   For grooves only: $\tau_{cop}=1.1\,(g+\sum|v_{mouth}|)+0.5$.
7. `enumerate_candidates(slabs, access, joint_type, class_disjoint)` (`:305-306`).
8. Camera, substream 5 (`:313-380`): profile; standoff U(max(min_z, 300), 1200), then replaced by framing; elevation; azimuth; roll; aim jitter; framing. Aim point = mean of weldable seam midpoints + jitter × 0.15 × the largest workpiece dimension. Optional `approach_cone` regime (D26).
9. Surface sampling, substream 4 (`:386-404`): density U(0.25, 4); `area_uniform` or `camera_raster`; `visible_from_cam`; `exterior`.
10. Seams (`:407-490`): keep candidates with `p0` and length > 1e-6; polyline at 10 pts/mm; arrays (see D.7).
11. Omission (`:498-516`), when `require_visible_seam`:
    - **`NoSeamsFound`** if there is no primary seam (weldable and in-class).
    - **`NoVisibleSeams`** if max `visible_fraction` over primary seams < 0.1.
    - A second `NoSeamsFound` fires if there are no candidates at all (`:522-532`).
12. Stored joint frame from seam 0 (`:533-541`): origin p0, x = tangent, z = approach (orthogonalised), y = z × x.
13. Scene dict (`:544-636`):
    - `scene_id = "{config_id}-{seed:010d}"`, `twin_key` (geometry keys).
    - `joint{type, seam_shape, quality_level, contact_mode, included_angle_deg, in_plane_yaw_deg, iso_17659_term, iso_9692_ref, ambiguous_with, groove, stack_offset_mm, prep}`.
    - `seam_definition: "nominal"`, `accessibility`, `objects`, `faces`, `fit{g, h, β, throat}`.
    - `noise_model` (stereo_z2, `lateral_sigma_px` 0.8, `grazing_dropout_deg` 75, seed); optional `mps` / `tacks`.
    - Substreams 6 and 7 are never drawn here.
14. ISO terms:
    - `_iso_17659_term` (`:175-185`): T is "T-joint (3.10)" if |α − 90| ≤ 10, else "angle joint (3.12)"; that is 2/3 of T scenes.
    - `_iso_9692_ref` (`:188-215`):
      - T and corner: None if t_min ≤ 2; "3.1.1/4.1.1" if 70 ≤ α ≤ 100; else "3.1.3/4.1.2".
      - Butt: 1.2.1 if t ≤ 4, 2.1 if ≤ 8, else None.
      - Edge: 1.1. Lap: None.
      - Grooves use `groove.iso_ref`.

### A.6 `accessibility.py`

Full detail is in section D. Unused helper: `_plane_basis` (`:484`).

### A.7 What each scoped test pins (one line each)

**`test_geometry.py`** (Phase-1 `build_t_joint`):
- `two_fillets`: `A:+w×B:-w` and `A:+w×B:+w`, both weldable.
- `objects_watertight_union_is_not`: watertight; B is above A **at α = 90° only**.
- `seam_length_is_the_overlap`: 180 mm when L_B = 180.
- `d19_offsets_at_right_angle[g ∈ {0, .5, 1.1, 3}]`: root = g, gap_mid = g/2.
- `d19_offsets_depend_on_alpha`: at α = 120 the root offset is > g.
- `fillet_dihedrals_are_supplementary[60, 70, 90, 120]`: the two dihedrals sum to 180.
- `seam_position_matches_closed_form`: the `B:+w` fillet lies at y = h + g·tan θ.
- `approach_escapes_the_material`: midpoint + 25 mm along the approach is outside both meshes.
- `world_pose_is_a_rigid_transform`: length, dihedral and root offset are pose-invariant.

**`test_joint_types.py`** (Phase-2 D4 gate):
- Every type yields a weldable seam; all objects are watertight.
- T: two full-length fillets. Corner: two seams with opposed approaches.
- Butt: centreline with separation = g, and its toes rejected `toe_of_centreline`.
- Lap: the interface is a facing pair, `bisector_blocked`; lap has two toes.
- Edge: coplanar `A:+v×B:+v` seam, 200 mm. Lap and edge are one topology.
- Pose invariance (β = 0 only).
- α is not the dihedral: the T/corner pair sums to 180.
- `fixture_contact` exists, including one with a line and dihedral in (30, 170).
- **xfail (non-strict): wrap-around phantom lap seams; known residual about 2–6 per 40.**
- `contact_tol ≤ max(0.95t, 1.1·sep)`.
- Narrow lap: weldable seams are ≥ 0.25 × the longest.
- Butt and edge centrelines survive β ≤ 2 (and the butt survives β ≤ 4).
- Edge primary seam survives β ∈ {0, .4, 2, 4}.
- Clip to the shared run (180); offset 60 gives 240.
- The table never blocks a seam whose approach z > 0.
- A deep lap has exactly two primary seams.
- Short-wide edge keeps its edge seam.
- Dissimilar-thickness butt: flush underside; centreline on `-w`.
- Corner h does not widen the gap.

**`test_class_disjointness.py`**:
- c = 2·min(t).
- T corner band only when near and parallel.
- Lap bands: small overlap, near the far edge, near the ends; yaw breaks the band.
- Edge bands on non-welded pairs only.
- Lap at yaw 175–180 is "edge"; 160 is clean.
- Full-circle yaw sampling.
- ISO term and 9692 ref mapping.
- Corner and butt have no bands.
- bench6a_lap rejects in-band seeds and amb_lap inverts the acceptance.
- Edge scene declares lap_toe and edge in-class.
- The D31 flag does not change draws.
- Underside lap toes are flagged and skipped by single-view scoring.
- Yawed, misaligned lap keeps `A:+v×B:-w` (near-parallel waiver).

**`test_grooves.py`**:
- `valid_preps` pools and refs at t = 3, 6, 8, 9, 11, 16.
- `sample_groove` ranges at t = 14 (U β 8–12, b 1–3, run ≥ 0.5; V α 58–62).
- `PreparedSlab` watertight; volume within 0.2%; chord ≤ 0.25.
- Face samples sit on the boundary.
- Oversized U radius raises.
- Profile is monotone.
- Mouth centreline is weldable (V, bevel, U).
- `_grooveroot` replaces root/gapmid, at depth t − c (1e-5), with `iso_ref`.
- Square scenes keep the D19 triple.
- Bevel mouth-vs-root lateral offset > 0.5 mm.
- Flag off reproduces square draws.

**`test_prepared_prism.py`**:
- Watertight; loft volume and area exact (U to 2e-4) for 3 preps × 5 outlines.
- A rectangle reproduces `PreparedSlab`.
- Face samples on the boundary.
- Root face clips a line to ±L/2; `s1` never clips.
- Mouth guard raises.
- Grooved scenes validate, use `prepared_prism`, and keep `_grooveroot`.
- D28 gate reads "outline".
- Outlines off gives `prepared_slab`.

**`test_parameters.py`**:
- 5071 values including caps and the t = 3 boundary; monotone in level.
- 617 values and caps.
- β limits B 1, C 2, D 4.
- Worst-defect rule; below_D reachable; boundary inclusive.
- Realised mix: each level in (0.10, 0.50) over 1500 draws.

**`test_accessibility_registered_pose.py`**: two 200 mm fillets at 12 random rigid poses (the `_mutually_visible` 1e-3 skip).

**`test_from_object.py`**: serialise → rebuild → serialise is the identity for all six primitives, plus the fixture; meshes watertight; unknown primitive raises; vocabulary complete; plate, grooved and curved scenes round-trip.

---

## B. Primitives

| Primitive (`objects[].primitive`) | Constructor parameters | Face registry | Models | Since |
|---|---|---|---|---|
| `Slab` (`slab`) | id, role, object_id, dims_mm=(L,W,t), T_world_part | +u −u +v −v +w −w | rectangular plate; the fixture | Phase 1 |
| `Prism` (`prism`) | id, role, object_id, outline_uv (k×2, CCW forced), thickness, T_world_part, shape="polygon" | +w −w s0…s{k−1} (seam edge s0, or s{k−2} for mirrored outlines) | convex-outline plate (D28 anti-shortcut) | 6a, 2026-08-27 |
| `PreparedSlab` (`prepared_slab`) | id, role, object_id, length_mm, width_mm, t_mm, prep{kind ∈ single_V/single_bevel/single_U, bevel_deg, root_face_mm[, radius_mm]}, T_world_part | +u −u −v +w −w root fusion [radius] | rectangular plate with an ISO 9692-1 groove edge | 6b step 4 (D35), 2026-08-28 |
| `PreparedPrism` (`prepared_prism`) | id, role, object_id, outline_uv, t_mm, prep, T_world_part, shape | +w −w root fusion [radius] s1…s{k−1} | outline plus groove (D28 on grooved butts) | 2026-09-07 (post-Phase-4 fix) |
| `Tube` (`tube`) | id, role, object_id, r_outer_mm, wall_mm, length_mm, T_world_part, base_cut=None (plane or cylinder), gap_mm=0 | lateral+ lateral− +w −w | pipe with a miter or saddle-cut end | 6b (D29 #2–#4) |
| `SweptSlab` (`swept_slab`) | id, role, object_id, spine, offset_lo_mm, offset_hi_mm, z0_mm, z1_mm, T_world_part | +w −w +v −v [+u −u if the spine is open] | band swept along an exact spine (rect tube, stiffener, arc butt) | 6b (D29 #5–#7) |

---

## C. Joint types: what is sampled (code values and PARAMETERS.md tags)

### C.1 Common geometry (substream 0) and defects (substream 1)

| Parameter | T | Corner | Butt, square | Butt, grooved (preset `grooved_butt.yaml`) | Lap | Edge | Tag (PARAMETERS) |
|---|---|---|---|---|---|---|---|
| t_A; t_B (= t_A with p 0.7, else independent) | U[1,12] | U[1,12] | U[1,12] | preset U[3,20]; grooves only when min(t) > 4; t_B forced = t_A | U[1,12] | **U[1,2]** | 1–12 [ours]; 30% dissimilar [ours]; edge t ≤ 2 [ISO] |
| L_A, L_B (along the seam) | U[80,400] each | same | same | same | same | same | [ours] |
| W_A; H_B (B height or width) | U[50,250] each | same | same | same | same | same; H_B = W_A with p 0.35 | [ours]; equal-width p [ours] |
| Δ length offset | U(±0.4·min(L_A,L_B)) | same | same | same | same | same | (untagged; `min_overlap_frac` 0.6) |
| α | **U[60,120]** | **U[60,120]** (outside angle) | 180 fixed | 180 | 0 | 0 | 60–120 [ISO]; degenerate values [ours] |
| Overlap s | — | — | — | — | **U(0.15,0.60)·H_B** (7.5–150 mm) | 0 | [ours] |
| g (target B/C/D) | U(0, min(617_lim, 2)) | same | same (not t-scaled) | **row b** | same | same | fillet ≤ 2 [ISO]; tail [ours] |
| g (below_D) | U(g_D, max(1.5 g_D, 3)); up to 5.28 at t = 12 | same | same | row b | same | ≤ 3 | — |
| h | U(0, h_lim); below_D U(1,2)·h_D (≤ 6) | same | same | same | **drawn, not applied** | **drawn, not applied** | per §2.1 [ISO] |
| β | U(0, β_lim); below_D U(4,8) | same | same | **drawn, not applied** | ≤ 0.4 cap | ≤ 0.4 cap | [ours] mapping; 0.4 cap [ours] D23 |
| Yaw (if `in_plane_yaw`) | full-circle feasible set | 0 | 0 | 0 | full-circle | 0 | §3.3 (untagged) |
| Outlines (if `polygon_outlines`) | B | A, B | A, B | A (and B for V/U; square B for bevel) | B | A, B | §3.3 |

Ranges observed over 4000 bench6a draws per type: g max 5.13; h max 5.60; β max 7.98; tilt T/corner −29.5…+37.0; edge g ≤ 3.0, h ≤ 1.38.

### C.2 Grooves (D35, D30: straight butts only; `config.py:248-287`)

| Prep | Thickness window in code (t = min(t_A,t_B) at selection) | Row (iso_ref) | Draws |
|---|---|---|---|
| square | ≤ 8 | 1.2.1 (t ≤ 4) / 2.1 (≤ 8) / None (> 8) | no groove; gap from 617 |
| single_bevel | (4, 10] (realised up to 19.9 mm, bug F3) | 1.9.1 | β U(35,60) on A only (included angle = β); b U(2,4); c U(1,2) |
| single_V, t ≤ 10 | (4, 10] | 1.3 | α U(40,60), β = α/2; b U(1,4); c U(0.5,2) |
| single_V, t > 10 | (10, 20] | 1.5 | α U(58,62); b U(1,4); c U(2,4) |
| single_U | > 12 | 1.6/2.6 | β U(8,12) per side (included 2β); b U(1,3); c U(4,5); R U(6, min(9, max(6.01, (t−c−1)/(1−sin β)))) |

- PARAMETERS tag: [ISO] (rows "verified against the PDF" per the repo; I did not re-read the PDFs). The lower bounds c ≥ 0.5 and b ≥ 1 are in effect choices.
- Mix over 6000 draws: V 2871, U 1177, square 993, bevel 959.
- Each groove forces equal thickness, and b overrides the 617 gap.

### C.3 Fixture (D12/D13)

- **Presence:** config flag (`fixture_present`). Twins are separate presets (`bench_*_fx`) on the same seeds, paired by `twin_key`.
- **Object:** a Slab with id "F", role "fixture", object_id 255.
- **Dims:** U(480,720) × U(320,480) × U(8,12). PARAMETERS says 600 × 400 × 10 ± 20% on the in-plane dims.
- **Tilt:** U(±10°) about x then y, so the combined tilt can reach about 14°.
- **Height:** U(±50) mm, applied to the whole assembly.
- **Placement:** its top touches the lowest workpiece vertex. `contact_mode` is "flat".
- **D13:** any face pair involving F is rejected `fixture_contact` before any geometry test.
- **Stored contact lines:** A's four edge faces × `F:+w`, class `lap_toe`, dihedral 90° [verified].
- Tags: [ours] (D12).

### C.4 Quality classification (`classify_quality`, `config.py:290-316`)

For each of (h, β, g): the level is the first of B, C, D whose limit is ≥ the value + 1e-12, otherwise below_D. The scene level is the **worst** of the three. The function is passed t_min and a = 0.7·t_min.

Targets are drawn from `quality_mix`:
- B/C/D: h ~ U(0, h_lim); β ~ U(0, β_lim); g ~ U(g_lo, max(g_lo, min(617_lim(target), 2))).
- below_D: h ~ U(1,2)·h_D; β ~ U(4, 8); g ~ U(g_D, max(1.5·g_D, 3)).

Applied to all types (see F6). Realised mix from 4000 sampler draws:

| Stratum | B | C | D | below_D |
|---|---|---|---|---|
| T / corner / square butt | .331 | .224 | .204 | .241 |
| Lap | .429 | .189 | .141 | .241 |
| Edge | .48 | .141 | .138 | .241 |
| Grooved (sampler) | .05 | .20 | .33 | .42 |

### C.5 D31/D32 class clearance

c = 2·min(t_A, t_B); near-parallel < 10°. It rejects in-band seeds in `class_disjoint` presets (all bench6a_* and grooved_butt). The `amb_*` presets invert the acceptance (`amb_T` forces h U(38,62) mm; `amb_lap` forces overlap 0.01–0.12 × H_B).

### C.6 Which presets the Phase 4 corpus uses

From `scripts/make_bench6b.py:75-91`:
- T: `bench6a_T.yaml`
- butt: `bench6a_butt.yaml` (15 per family) and `grooved_butt.yaml` with square draws filtered out
- corner / lap / edge: `bench6a_*.yaml`

All of these use DEFAULT ranges except grooved thickness, and all have yaw, outlines and `class_disjoint` on.

---

## D. The accessibility rule (`accessibility.py`) in full

1. **Enumeration** (`enumerate_candidates`, `:262-334`): every face pair of every pair of distinct objects (i < j), fixture included. The face pair is a sorted string pair. There is **no explicit exterior test**; "exterior" is implied by the gates below.
2. **`_judge`** (`:337-446`). Curved faces (plane None) are skipped. Then classify by $\|\mathbf n_A\times\mathbf n_B\|$ against `PARALLEL_TOL = sin 8°` (`:110-111`):
   - **Coplanar exposed** (parallel, $\mathbf n_A\cdot\mathbf n_B>0.5$, `:492-633`):
     1. $\bar{\mathbf n}=\frac{\mathbf n_A+\mathbf n_B}{\|\cdot\|}$.
     2. Candidate in-plane axes: A's own axes if A is a Slab; otherwise every edge direction of A's face and its in-plane perpendicular.
     3. On each axis, the face extents must be disjoint with gap γ = b₀ − a₁ (strict). Requires γ ≤ `coplanar_gap_tol_mm` (grooves) or else τ_c.
     4. Shared run along $\bar{\mathbf n}\times\text{axis}$; keep the "widest run" (**bug at `:572`**).
     5. Base point $-d_A\bar{\mathbf n}$ + mid-gap.
     6. Non-Slab parts are clipped with slack γ/2 + τ_c.
     7. Step test: $\max|\mathbf n_B\cdot\mathbf p+d_B|\le$ coplanar_tol (0.5) + τ_c (`:609`).
     8. Reasons: `fixture_contact`, then `too_short` (< 10 mm), then the clearance test with axis $\bar{\mathbf n}$.
     9. Stored dihedral 180, separation γ.
   - **Facing** (antiparallel): separation is |signed distance of B's face centre from Π_A|. Requires separation ≤ τ_c and overlapping projections. Always rejected (`fixture_contact` or `bisector_blocked`), with **p0 None**, so it is not written to `seams[]`.
   - **Intersecting:**
     1. Line from `intersect_planes`.
     2. Exact 2-D clip on each face with slack τ_c; $s\in[\max(s^A_{lo},s^B_{lo}),\min(s^A_{hi},s^B_{hi})]$.
     3. `_mutually_visible` (`:738-764`): sample t ∈ linspace(0.15, 0.85, 5); probes at fractions .25/.5/.75 toward each face's closest point; any probe strictly inside a solid (tol −1e-6) drops the pair. Samples with span ≤ 1e-3 are skipped.
     4. Separation $\sigma=\max_{k\in\{A,B\}}\max_{j}\|\mathbf q_j-\pi_{F_k}(\mathbf q_j)\|$ over 5 samples (`:767-776`); σ > τ_c drops the pair.
     5. Reasons in order (`:427-440`): `fixture_contact`; then `degenerate_dihedral` if φ ∉ [30°, 170°]; then `too_short` if length < 10 mm; then `bisector_blocked` if $\|\mathbf n_A+\mathbf n_B\|<10^{-9}$; then the clearance search, giving `bisector_blocked` if it fails.
3. **Clearance search** (`_clear_axis`, `_work_angle_axes`, `_escapes`, `_cone_directions`; `:164-259`).
   - Candidate axes, nearest the bisector first: $\hat{\mathbf a}=\cos\vartheta\,\hat{\mathbf b}+\sin\vartheta(\cos\phi\,\mathbf u+\sin\phi\,\mathbf v)$ with ϑ ∈ {0, 15, 30, 45}° and φ = 2πk/8, i.e. 25 axes.
   - An axis is admissible only if $\hat{\mathbf a}\cdot\mathbf n_A>0$ and $\hat{\mathbf a}\cdot\mathbf n_B>0$.
   - Cone: the axis plus 8 rim rays at 30°. Probes at $\mathbf q+10^{-3}\mathbf r+\{3.75, 7.5, 15\}\mathbf r$ (standoff 15 mm). The test runs at 5 points along the seam, endpoints included. Any probe inside any solid (inclusive) fails that axis.
   - The first clearing axis is stored as `approach`.
   - Consequence: an open nook clears only if φ > 60°.
4. **Post-passes:**
   - `_suppress_toes` (`:693-719`): for each weldable 180° centreline, any weldable intersecting candidate parallel to it within 1e-6 and within reach 1.5γ + 1e-6 becomes `toe_of_centreline`.
   - Class pass (`:315-321`): `matches_joint_type` = class ∈ `ALLOWED_CLASSES` (`:96-105`): T {fillet}, corner {fillet, edge}, butt {butt}, lap {lap_toe}, edge {edge}; disjoint mode adds lap_toe to edge.
   - `_drop_cross_runs` (`:636-690`, skipped for disjoint lap/edge):
     - Voters are primary workpiece runs, else weldable runs, else all runs.
     - $S=\sum_i\ell_i\hat{\mathbf t}_i\hat{\mathbf t}_i^\top$; the dominant direction is the leading eigenvector.
     - Voters still weldable with $|\hat{\mathbf t}\cdot\hat{\mathbf e}|<\cos45^\circ$ become `too_short`.
   - Sort (`:330-333`): (not primary, not weldable, face_pair[0], face_pair[1], round(p0, 6)).
5. **Seam class** (`Candidate.seam_class`, `:142-155`): broad faces are {+w, −w}.
   - Both broad: `butt` if φ > 172°, else `fillet`.
   - Neither broad: `edge`.
   - One broad: `lap_toe`.
   - This is purely mechanical: prepared root/fusion and prism side faces count as edge faces, and fixture contacts come out `lap_toe`.
   - `primary` = weldable ∧ matches_joint_type.
6. **Reject vocabulary.**
   - Frozen enum (SCHEMA §2.6; schema.json adds the curved-only `confined_bore`).
   - The plate code emits only `fixture_contact`, `bisector_blocked`, `degenerate_dihedral`, `too_short` and `toe_of_centreline`.
   - `no_contact` and `interior_face` are never emitted (those pairs are dropped). `wrong_class_for_joint` was retired in 2.3.0.
7. **What a `seams[]` entry contains** (`scene.py:458-490`): id, weldable, reject_reason, face_pair, parametric{kind "line", p0_mm, p1_mm}, length_mm, dihedral_deg, seam_class, matches_joint_type, underside (lap_toe only: approach·ẑ_joint < 0), sampled{array, density_per_mm 10, n}, occluded_fraction, in_frame_fraction, visible_fraction.
   - `seams.npz` arrays: `seam_i` (nominal), `_root` and `_gapmid` (square), or `_grooveroot` (grooved butt class), `_s`, `_tangent`, `_approach` (clear axis, or bisector if rejected), `_nA`, `_nB`, `_visible`.
8. **D19, the three curves.**
   - **nominal** = Π_A ∩ Π_B clipped (or the coplanar centreline). It is **stored and scored** (`seam_definition: "nominal"`; the harness default is `ground_truth(curve="nominal", primary_only=True)`).
   - Why nominal: it is a design property invariant to g, symmetric under A↔B, and continuous as g → 0 (SCHEMA §1.3 L145-154).
   - Pipeline definitions: $\mathbf a=\pi_{F_A}(\mathbf q)$, $\mathbf b=\pi_{F_B}(\mathbf q)$, gap_mid $=(\mathbf a+\mathbf b)/2$, root = the farther of a and b (a tie goes to b).
   - For a fillet: $\delta_{root}=H_e/\sin\varphi$, where H_e is the height of the terminating edge above Π_A. For the hinge face H_e = g; for the opposite face H_e = g + t_B·sin θ [verified, e.g. 3.390 mm at α = 105°]. gap_mid = δ_root/2. At 90° these reduce to g and g/2.
   - **D36 groove_root:** in A's frame $(x,\ g/2,\ -(t-c))$, i.e. the gap centre at t − c below the top (`scene.py:431-439`).
9. **Measured offset statistics.**
   - Notebook 15 §3.2, executed output, primary square-prep seams in bench_phase4:

     | seam_class | n | gap median | root median | root max | gap_mid median | root > 1 mm | root > 3 mm |
     |---|---|---|---|---|---|---|---|
     | butt | 117 | 1.10 | 1.24 | 7.21 | 0.63 | .58 | .23 |
     | edge | 120 | 0.55 | 0.54 | 6.73 | 0.02 | .37 | .06 |
     | fillet | 181 | 0.89 | 0.85 | 5.83 | 0.43 | .48 | .14 |
     | lap_toe | 438 | 0.43 | 0.43 | 4.83 | 0.22 | .32 | .06 |

   - Fillet root/gap ratio by α: 0.96 for (59, 80], 1.00 for (80, 100], 1.25 for (100, 121].
   - MANUAL L210-213 quotes these numbers.
   - Caveats: the acute bin includes interpenetrating fillets with zero offset (F1); the edge row mixes corner outside welds with flush seams (flush seams are exactly g/2 and 0).
   - lit-ransac, scored against each curve (`notes/dataset_plan.md` L352-354): median RMSE nominal 0.599, root 0.735, gap_mid 1.078.

---

## E. Decision numbers referenced in these modules

| D | Meaning | When |
|---|---|---|
| D1 | Seam truth is analytic, from the placement transforms | Phase 0 |
| D3 | Inverted pipeline: sample the joint, then place the parts, then construct the seam (no detection) | Phase 0 |
| D4 | Weldable = exterior face pair whose bisector escapes (now three arms) | Phase 0 / 2 |
| D5 | Several seams per scene is the norm | Phase 0 (counts revised 2026-08-16) |
| D8 | Tacks are a versioned rule, not truth | Phase 0 |
| D9 | Tier-1 core has no simulator and no optional native deps | Phase 0 |
| D12 | Fixture sampled and twin-paired, pose-varied (±10° tilt, height not pinned) | 2026-08-13 |
| D13 | Both faces must belong to workpieces, else `fixture_contact` | 2026-08-13 |
| D16 | Stereo noise model with named profiles | 2026-08-13 |
| D18 | Included angle is a design parameter, independent of β | 2026-08-15 |
| D19 | nominal / root / gap_mid; nominal is stored | 2026-08-15 |
| D20 | camera_raster samples the hidden surface separately | 2026-08-15 |
| D21 | Each part watertight; the union is disjoint | 2026-08-15 |
| D22 | Seam class derived from faces; joint type decides which classes are legitimate | 2026-08-16 |
| D23 | Stacked joints cap β at 0.4° (implemented, not cited by number in code) | 2026-08-16 |
| D25 | MPS is a rule | 2026-08-20 |
| D26 | Two camera regimes | 2026-08-20 |
| D27 | Dimensions vary across the seam, not along it | 2026-08-20 |
| D28 | Only the seam edge may be axis-aligned (yaw + outlines) | Phase 6a, 2026-08-27 |
| D29 | Curve families (as amended) | 2026-08-28 |
| D30 | Grooves on straight butts only | Phase 6 |
| D31 | Clearance c = 2t keeps class borders empty | 2026-08-27 |
| D32 | Class-boundary stratum | Phase 6a |
| D33 | Curves closed-form or constructed | 2026-08-28 |
| D34 | Mesh chord ≤ 0.25 mm (tolerance 0.05) | 2026-08-28 |
| D35 | Grooves drawn from `valid_preps(t)`, square stays in the pool | 2026-08-28 |
| D36 | D19 triple is square-only; grooves get groove_root | 2026-08-28 |
| D38 | tackrule-0.1 | 2026-09-01 |

Also referenced in the docs for this code: D37 (clearance tested from the groove mouth; "realised by construction") and D24 (grooves deferred to 6b).

---

## F. Discrepancies between code and docs

### Critical (ground truth or physics)

1. **F1: interpenetration** (section 0, item 1). Contradicts D21 / SCHEMA §6.3 L931, the `_flat_B` docstring (`layouts.py:293-300`), `test_joint_types.py:298-299`, and PARAMETERS §2.1 L56-61.
2. **F2: `accessibility.py:572` best-axis bug** (section 0, item 2).

### Major

3. **F3:** groove row selected at t_min, then t_B := t_A (`config.py:491-498`); `iso_ref` and row window violated.
4. **F4:** β not applied to grooved butts; h not applied to lap/edge. PARAMETERS §2.1 L40-46 says h acts "along z for butt, corner and T, and along the gap-free axis for stacked joints"; the code moves T along y and stacked joints not at all.
5. **F5:** the 617 limit is applied to butt and edge, against PARAMETERS §2.4 L125-127 and `config.py:200-201`.
6. **F6:** the square-butt gap is not thickness-scaled (b ≈ t or t/2), although PARAMETERS §2.6 L157-166, §3 L278 and §7 L736 say "the sampler must change". `iso_9692_ref` 1.2.1/2.1 is assigned by thickness only (`scene.py:207-212`).
7. **F7:** two primary butt seams per grooved scene, against PARAMETERS §2.6.1 L198-200 and D36.
8. **F8:** "every rejected candidate is stored" (MANUAL L199-201), "weldable: false seams are kept, never dropped" (SCHEMA §4.1 L639), and "facing pair emitted and rejected" (README L363-366). In practice facing pairs and pairs failing the gates are not in `seams[]`; `no_contact` and `interior_face` are never emitted.
9. **F9:** off-class seams are still described as `weldable: false / wrong_class_for_joint` in SCHEMA §2.5.1 L318-320, the `scene.schema.json` `seam_class` description, PARAMETERS §2.7 L233-234 and `layouts.py:641-643`. This was retired in 2.3.0 (SCHEMA §0 L51-58, §2.6 L338).
10. **F10:** reject precedence. SCHEMA §2.6 L340-341 gives the table order and says "applied order is recorded". The code order is `fixture_contact` > `degenerate_dihedral` > `too_short` > `bisector_blocked`, then post-passes that touch only weldable seams; nothing is recorded.
11. **F11:** `gap_mid` definition: SCHEMA §1.3 L143 vs `d19_curves` vs `joints._fillet` (section 0, item 8). The `d19_curves` docstring claim "reduces to the Phase 1 result exactly" is true only at 90°.
12. **F12:** seam order. SCHEMA §2.7 L387-391 says (weldable, pair, start, end); the code says (not primary, not weldable, pair, p0). The docstring at `:271-272` also says weldable first.
13. **F13:** SCHEMA §2.5 L270-273 and the §4 example (L561-569) give the fixture contact as `A:-w × F:+w` with a 90° dihedral and a line. In code that pair is a facing pair; the real stored contacts are `A:±u/±v × F:+w` (class lap_toe).
14. **F14:** effective dihedral floor 60° (cone), versus `dihedral_min_deg` 30 and the SCHEMA §2.5.1 table "fillet 60–120°" (real fillet dihedrals run about 53–127° before the cone gate).
15. **F15:** obtuse T joints lose a fillet: 48% of α > 95° specs keep one, 5% keep none. This qualifies "a T-joint has two fillets (D5)" (`joints.py:18-23`).

### Minor (stale or imprecise wording)

16. **F16:** corner α is the outside angle (the inside fillet is 180 − α − β), but `iso_9692_ref` tests 70 ≤ α ≤ 100 on that outside angle. Rows may be wrong for α ∈ [70, 80) ∪ (100, 110]. Rows 4.1.1 (t > 3) and 4.1.2 (t₂ > 5) thickness conditions are not checked.
17. **F17:** the "same plane?" tolerance is documented as 0.5 mm (`accessibility.py:64-67`, SCHEMA §4 L499); the code uses 0.5 + τ_c.
18. **F18:** pose-dependent centreline under β: offset from Π_A is $|d_A|(1-\cos\beta/2)$ (`:578`). Examples: 0.015 mm at β = 2°, 0.061 mm at β = 4°, with |d_A| = 100 mm.
19. **F19:** the PARALLEL_TOL comment says "β up to 4°" (`:76-79`); below_D draws reach 8°. Centrelines survive up to 7.99° in a sweep.
20. **F20:** at exactly g = 0 there is no butt or edge centreline (strict `b0 > a1`, `:553-558`). Probability zero under the sampler, but it breaks "continuous as g → 0" for pinned presets.
21. **F21:** the seam edge is `s{k−2}`, not `s0`, for mirrored outlines (A of corner/butt/edge, B of edge). Docs claim `s0`: `geom.py:286-300` and SCHEMA §2.2 L215.
22. **F22:** fixture wording. PARAMETERS §3.1 L298 says only the in-plane dims vary (code also varies thickness 8–12); L299 says "uniform in tilt direction" (code uses independent x/y tilts); §3 L285 lists a contact_mode mix (code is always "flat"). In about 0.5% of corner specs B's corner sits lower than A, so A floats up to 3 mm above the table.
23. **F23:** fixture twins differ in assembly yaw because the height draw comes before the yaw draw. The bench_*_fx comment "identical in every sampled respect" is not true in the world frame (SCHEMA §6.4 does allow placement to differ).
24. **F24:** the gap tail "to 3 mm" (PARAMETERS §3 L278, `config.py:80-81`) actually reaches 5.28 mm.
25. **F25:** groove windows. PARAMETERS §2.6.1 L192/L204 say "3–10" for V and bevel; the code needs t > 4. The D35 note table gives "8–12 bevel+V"; the code stops bevel at 10.
26. **F26:** `assert_accessibility_covers_gap` is dead and would fail on defaults; the DEFAULT_ACCESS comment (`:58-62`) points at a non-existent `configs.` module.
27. **F27:** the `_line_separation` docstring says "nearer face"; the code takes the max over both faces.
28. **F28:** MANUAL L166-167 defines [ds] as "dataset design"; PARAMETERS L21 defines it as vendor datasheet.
29. **F29:** version strings out of step: SCHEMA header 2.3.0 vs code 2.4.0; `__init__` docstring and `scene.py:1` say 2.0.0; README says v2.2.0.
30. **F30:** "all Phases 1–6 generate square" (PARAMETERS L621; SCHEMA §4 L485) is stale.
31. **F31:** README L719-722 "prepared plate is a rectangle by construction" is superseded by L737-745.
32. **F32:** the PARAMETERS §6 preset list (default, thin_sheet, lab_fixture, annotation_study, fixture/sensor_ablation) does not exist in `configs/`.
33. **F33:** the lap offset is described as "0 < offset < L" (PARAMETERS L227/L280, SCHEMA L484); the code uses 0.15–0.60 × H_B.
34. **F34:** SCHEMA §5.2 L784 still uses the old `occluded_fraction` formula, and L792's "identical at g = 0" ignores h and tilt.
35. **F35:** SCHEMA §2.2 lists a `cylinder` primitive that does not exist in code; the `confined_bore` reject reason is missing from SCHEMA §2.6.
36. **F36:** a known residual remains: the xfail test for phantom lap seams (wrap-around).

---

## G. Thesis-draft claims checked

**(1) "The rule (D4) takes every pair of exterior faces of two parts that meet, computes their nominal seam as the intersection of the two face planes, and keeps it when the bisector of the dihedral angle escapes to free space."**

Partly right; it needs correction.
- The rule enumerates every face pair of every object pair, fixture included (`accessibility.py:288-297`). "Exterior" and "meet" are not tested directly; they are approximated by the clip with slack τ_c, the separation ≤ τ_c gate, and the mutual-visibility gate.
- The plane intersection is only one of **three arms**. Non-parallel faces (> 8°) intersect. Same-direction parallel faces give the in-plane gap centreline (edge joints, butt centrelines). Antiparallel faces have no line and are always rejected.
- "Escapes" means a 30° nozzle cone (9 rays, probes at 3.75/7.5/15 mm) clears at 5 points along the seam. The torch may tilt up to 45° off the bisector in 15° steps, keeping a positive component on both normals. The stored approach is the first axis that clears. In practice this means φ > 60°.
- The `fixture_contact`, `degenerate_dihedral` (30–170°) and `too_short` (< 10 mm) gates run first. Toe suppression, class taxonomy and cross-run demotion run afterwards.
- Suggested wording: "…enumerates every face pair of every pair of parts; pairs that are adjacent (within a per-scene contact tolerance and mutually visible) yield a candidate line (plane intersection for intersecting faces, gap centreline for coplanar ones); a candidate is weldable when both parts are workpieces, its dihedral lies in 30–170°, it is ≥ 10 mm long, and a 30° nozzle cone, tilted up to 45° off the dihedral bisector if needed, clears the scene."

**(2) "Each seam gets a class (fillet, butt, lap toe, edge); a rejected candidate is kept with the reason."**

- The class half is true. Every stored seam has a `seam_class` derived from whether its faces are broad (±w) and from its dihedral (> 172° → butt).
- The second half needs qualifying:
  - Only candidates with a line of non-zero length reach `seams[]` (`scene.py:411-414`).
  - Facing pairs, such as the buried lap interface, are judged `bisector_blocked` but not stored.
  - Pairs failing adjacency are dropped without a reason (`no_contact` is never emitted).
  - Off-class seams are not rejected (weldable, `matches_joint_type: false`).
- Suggested: "…rejected candidates that carry a seam line are kept as negatives with a frozen reason (fixture_contact, degenerate_dihedral, too_short, bisector_blocked, toe_of_centreline)."

**(3) Ranges** (code and presets; see C.1 and C.2):
- **T:** t 1–12 mm (U, 30% dissimilar); α U[60, 120] ✓; L 80–400; W_A and H_B 50–250; g 0–2 (617-bounded) with a below_D tail to 5.28; h to 6; β to 8.
- **Corner:** same as T; α 60–120 is the outside angle.
- **Butt, square:** t 1–12, α 180, same gap rule (not thickness-scaled).
- **Butt, grooved:** "4–20 mm, V/bevel/U" is approximately right for grooved scenes. The preset draws t ∈ U[3, 20] and grooves need t > 4.
  - V: (4, 20], α 40–60 (≤ 10 mm) or 58–62 (> 10 mm); c 0.5–2 or 2–4; b 1–4.
  - Bevel: β 35–60, c 1–2, b 2–4; designed for 4–10 mm but realised up to ~19.9 mm (F3).
  - U: t > 12, β 8–12, c 4–5, R 6–9, b 1–3.
- **Lap:** t 1–12; overlap 15–60% of B's width (7.5–150 mm); β ≤ 0.4°; h has no geometric effect.
- **Edge:** t 1–2 mm ✓ (U[1, 2], ISO 9692-1 ref 1.1); offset 0; widths equal with p 0.35; β ≤ 0.4; gap ≤ 3 mm.
