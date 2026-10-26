# Heat-Aware Tack Weld Sequencing for a Robotic Welding Cell: Accepted Lightweight Models, Literature Evidence, and a Defensible Thesis Methodology

The most defensible non-FEM approach for your thesis is to model each tack as a short stationary heat source, compute the temperature field by **time-shifted superposition of analytical point/line-source (Green's function) solutions with image sources for plate edges** (Rosenthal/Rykalin family), and to combine that thermal cost with **FEM-derived and practice-derived mechanical rules** (lock the geometry first, alternate/opposite tacks, center-out, keep tack pitch small) inside a permutation search. No paper I found does this for tacks specifically: the analytical models are textbook-standard, and the "analytical thermal model + sequence search" idea has been published and validated in WAAM/DED path planning, but tack-order optimization with an analytical thermal model is a genuine literature gap and therefore a legitimate MSc contribution.

## TL;DR

- **Thermal model (well-established, non-FEM):** superpose closed-form instantaneous/stationary point- or line-source solutions (Rosenthal 1941/1946; Rykalin; Eagar & Tsai 1983; Goldak-source analytical solutions by Nguyen et al. 1999/2004 and Fachinotti et al. 2011) over time for each tack, adding mirror sources for finite plate edges. This costs microseconds per evaluation, so even exhaustive search over 8–10 tacks is feasible. The same "analytic thermal model + greedy/tree search" pattern is already validated for DED/WAAM deposition ordering: Hatala, Reutzel & Wang (Metals 2025) combined a greedy planner with a modified Rosenthal model for ER100S-G laser hot-wire DED and cut deposition time by 9.2% on a 66 × 73 × 16.5 mm part (15 layers, 300 beads) while meeting interpass limits.
- **Tack-order evidence is real but almost entirely FEM-based and sparse:** Jonsson, Karlsson & Lindgren (1985, Welding Journal) showed root-opening change depends on the order the tacks are made; Abid & Siddique (2005, IJPVP; Acta Mechanica) showed pipe-flange distortion depends strongly on tack orientation relative to the weld start; Murakawa et al. (2012, Trans. JWRI) showed tacks stop gap opening and smaller tack pitch reduces it; Heinze et al. (2012, JMPT) showed tacks must be modelled to predict distortion at all. Spot-weld sequencing (Wärmefjord, Söderberg, Tabar et al.) adds the key rule that the first two "geometry" welds lock the assembly: Wärmefjord, Söderberg & Lindkvist (NordDesign 2010) found that "after two welding points were executed, the geometry was quite stable, so… two geometry points would be enough".
- **Recommendation:** implement (1) an analytical superposition thermal cost + mechanical rule penalties + travel time, solved exactly for N≤9 and by beam search/simulated annealing above; optionally (2) a lightweight sequential-stiffness (spring/compliance) shrinkage model; and (3) use a small FEM/inherent-strain study only for calibration/validation. Validate with an IR camera/thermocouples and pre/post-tack gap measurement against your linear and fixed 0°/90°/180°/270° baselines—and note that the classic pipe practice is actually 0°→180°→90°→270° (opposite pairs), not sequential 0°→90°.

## Key Findings

1. **Analytical heat conduction is fully accepted welding science and is fast enough for sequence search.** Rosenthal's moving point/line-source solution is the classic basis of welding heat-flow analysis; the superposition of point-source solutions (Rosenthal's original approach, extended to distributed Gaussian sources by Eagar & Tsai) is linear, so a sequence of short tacks can be represented as a sum of time-shifted source responses. Its known weaknesses (singular at the source, constant properties, no latent heat, no convection/radiation unless added) matter for weld-pool prediction but matter far less for the quantity you need: *how much residual heat sits near tack k when tack k is fired*, which is a far-field quantity.
2. **Finite-geometry corrections are standard.** Adiabatic plate edges are handled by mirror (image) sources. A study in the Journal of Welding and Joining (42(4):396) showed why this matters: "The temperature at the corner node was predicted to be 1,266°C, but it was 633°C when the heat source and corner were sufficiently distant" (at 2.436 s). In other words, an edge exactly doubled the local temperature. Tacks near part ends are exactly where this occurs.
3. **Goldak double-ellipsoid analytical solutions exist.** Nguyen et al. published them in the Welding Journal in 1999 and 2004, and Fachinotti, Anca & Cardona corrected and extended them in 2011, showing that Nguyen's solution is only exact when both semi-ellipsoids are equal. Meirelles, Lourenço & Caprace (Heliyon 2025) then derived a closed form for finite plates combining a double-ellipsoid source with convective boundaries, noting that Fachinotti's equation "is not capable of adequately representing the edges of the plate". These solutions are useful for near-field accuracy but are overkill for tack *ordering*; a point/line source plus a source-size cutoff is sufficient.
4. **Inherent strain/inherent deformation (Ueda, Murakawa, Deng, Liang) is "lightweight", not "non-FEM".** Inherent deformations are computed once per joint type (by thermal-elastic-plastic FEM, inverse analysis or measurement) and then applied in a *linear elastic* FE model; it is the accepted industrial route for distortion of large structures and has been used to study welding sequence, initial gap and restraint. A crucial insight for your design: in a purely linear elastic model with fixed stiffness, order does not matter—sequence effects arise only because each new weld/tack changes the stiffness (interface elements closing the gap) and because of thermal interaction. Any lightweight mechanical model you use must therefore add stiffness incrementally.
5. **Welding sequence optimization (WSO) is mature but FEM-heavy.** Approaches include:
   - GA + thermo-mechanical FEM (Kadivar et al. 2000);
   - surrogates (Voutchkov et al. 2005; Asadi & Goldak 2011);
   - GA/PSO/ACO for spot welds (Tabar et al. 2018);
   - Q-learning: Romero-Hdz et al. (Trans. Intell. Weld. Manuf. 2018) used Simufact on "a typical widely used mounting bracket which contains eight welding beads" and reported a "reduction of structural deformation up to ~66%"; their domain-knowledge version (Eng. Appl. Artif. Intell. 2020) reports about 71% between the best and worst sequences;
   - ANN + swarm (2022);
   - domain-knowledge self-learning (Scientific Reports 2025, Table 7), which reached a maximum deformation of 0.44 mm versus 0.65 mm for rule-based reasoning (32.31% lower) and 0.62 mm for an RL-based method.

   Reported gains of 20–71% show that order genuinely matters, but almost all evaluations rely on FEM or FEM-trained surrogates.
6. **Robotic sequencing with thermal constraints is mostly found outside welding.** A 2026 IJAMT paper on PCB drilling (TC-HBA) formulates a TSP with a minimum safety distance between consecutive operations as a geometry-based thermal-risk proxy; path length rose 2.92% (sparse) to 25.95% (dense layouts). This is a directly transferable, citable formulation of a "don't tack next to the last tack" constraint.
7. **Practice rules for tack order are consistent but mostly non-peer-reviewed.** Shop/teaching sources agree: center first then alternate outward on plates; on pipe, tack 12 o'clock, then the opposite 6 o'clock, then 3 and 9; balance shrinkage with alternating sides; back-step/skip. Codes (AWS D1.1 and pipe codes) regulate tack *quality* (tacks must meet weld requirements or be removed/incorporated) but, as an AWS forum respondent puts it, "Code is silent on tack lengths and location." Your 0/90/180/270 baseline is justified by practice, not by code.

## Details

### 1. Non-FEM / lightweight thermal and distortion models

**1.1 Analytical conduction solutions (well-established).**

- *Instantaneous point source on a semi-infinite body* (thick parts; heat Q deposited at the surface at time t_k):
  ΔT(r, t) = 2Q / [ρc (4πa(t − t_k))^{3/2}] · exp(−r² / (4a(t − t_k)))
- *Instantaneous line source through a thin plate of thickness h* (sheet/plate tacks with full or near-full penetration):
  ΔT(r, t) = Q / [ρc h · 4πa(t − t_k)] · exp(−r² / (4a(t − t_k))) · exp(−b(t − t_k)), with b = 2α_s/(ρch) an optional surface heat-loss term (Rykalin form).
- *Stationary source of duration τ (tack arc time)*: integrate the instantaneous solution over [t_k, t_k+τ], or—since tacks last ~0.5–3 s—treat as instantaneous with Q = η·U·I·τ (η ≈ 0.7–0.85 for GMAW, ~0.6–0.7 for GTAW; standard process-efficiency ranges).
- *Moving source (if tacks are short stitches)*: Rosenthal thin-plate solution ΔT = q/(2πkh) · exp(−vξ/2a) · K₀(vr/2a).
- *Superposition for a sequence*: T(x, t) = T₀ + Σ_k G(x − x_k, t − t_k)·H(t − t_k), where H is the Heaviside step. This is exactly the "time-shifted superposed sources" approach and is valid because the conduction equation is linear for constant properties.
- *Finite body corrections*: for each adiabatic edge, add a mirror source reflected across it (repeat for corners/opposite edges, truncating the series once terms are negligible). For a cylinder/pipe, "unroll" the circumference into a strip and use periodic images at ±πD, which correctly captures heat arriving from both directions around the pipe.
- *Distributed sources*: Eagar & Tsai's Gaussian traveling source, and Nguyen/Fachinotti closed forms for double-ellipsoid sources, are the accepted refinements when near-field accuracy matters.

Accepted references: Rosenthal (Welding Journal 1941; Trans. ASME 1946), Rykalin (1951/1957 monograph), Carslaw & Jaeger (*Conduction of Heat in Solids*), Radaj (*Heat Effects of Welding*, 1992), Grong (*Metallurgical Modelling of Welding*), Goldak & Akhlaghi (*Computational Welding Mechanics*, 2005).

**1.2 Lumped/network and reduced-order models (established in AM; niche in welding).**
A thermal RC network (nodes at tack locations and a few part nodes, conductances from geometry) is defensible but adds calibration burden with little gain over analytical superposition for plates. More relevant is the AM literature: an ETH Zurich WAAM study reduced a thermal model by balanced truncation and searched deposition order with Monte Carlo tree search; a follow-up ("Too hot to print, too slow to handle") combined an RL agent for deposition time with an MCTS thermal optimizer using a reduced-order model. These are strong precedents for "fast thermal model inside a sequence search".

**1.3 Inherent strain / inherent deformation (lightweight, still FE).**
Ueda–Murakawa–Deng elastic FEM: inherent deformations (longitudinal/transverse shrinkage, angular distortion) for each joint type are obtained once (TEP-FEM, inverse analysis, or experiments) and applied as initial strains in an elastic model; interface elements represent the gap and are "closed" step by step following the welding sequence. Liang, Deng, Sone & Murakawa showed inherent deformations can be estimated through inverse analysis and used to predict distortion; Deng/Murakawa used the method to study heat input, sequence, initial gap and constraint. For a thesis that "doesn't want FEM", this counts as **lightweight FE**, not non-FEM: you would still need a linear elastic solver (shell/beam), but no nonlinear thermal-elastic-plastic analysis. A Korean review (Journal of Welding and Joining 2020) explicitly recommends inherent-strain elastic FEM as the evaluator inside AI-based sequence optimization because TEP-FEM is too expensive for thousands of evaluations.

**1.4 Empirical/semi-empirical shrinkage formulas (accepted for estimates, crude).**
- Transverse shrinkage of butt welds (Spraragen–Ettinger/Blodgett family): Δ_t ≈ 0.2·A_w/t + 0.05·g (A_w = weld cross-section, t = thickness, g = root gap). Note it contains no heat-input term.
- Angular distortion (Okerblom-type): θ ∝ Q/h², i.e., proportional to heat input per length divided by thickness squared.
- Longitudinal shrinkage force/volume (Okerblom, Guyot): proportional to heat input per unit length.
- Masubuchi (*Analysis of Welded Structures*, 1980) and Satoh & Terasaki (1980, J. Japan Welding Society, root-gap change in butt welds) are the canonical sources.
For tacks, these give an *order-of-magnitude pull-in per tack* that you can feed into a mechanical proxy (Method 2 below), but they do not by themselves encode order.

**1.5 Surrogates/metamodels (established for WSO, require training data).**
Voutchkov, Keane, Bhaskar & Olsen (CMAME 2005) and Asadi & Goldak (IJMMD 2011) built surrogates of FEM distortion over sequences; a Procedia Structural Integrity study showed naïve surrogates fail on symmetric 8-bead X-welds on plates and pipes and proposed corrections; Tabar et al. used DoE-based and NN surrogates for spot welds. A small FEM-trained surrogate is a reasonable "optional extension" but not necessary for the core thesis.

**1.6 WAAM/DED heat-aware ordering (directly transferable).**
- A *semi-analytical* simplified WAAM simulation (Virtual and Physical Prototyping, 2024), calibrated against FE thermal histories, was embedded in a deposition-sequence optimization—this is the closest methodological template to what you want (analytical model, FE used only for calibration).
- *Path Planning for Rapid DEDAM Processing Subject to Interpass Temperature Constraints* (Hatala, Reutzel & Wang, Metals, 2025): a compact analytic thermal model (modified Rosenthal) plus a **greedy** algorithm selecting the next bead to satisfy interpass temperature; experimentally validated with in-situ temperature measurement. It reduced deposition time by 9.2% for a 66 × 73 × 16.5 mm part with 15 layers and 300 beads, and the authors' model predicts a 40% reduction if that part dimension is made five times larger.
- *Selection of Trajectories to Improve Thermal Fields During Electric Arc Welding Using Hybrid Model CFD-FNN* (Metals, 2025): the objective was to minimize the area under temperature-history curves at multiple welding sites, explicitly because temperature is cheaper to measure than deformation. This objective transfers directly to tacks.

### 2. Literature on welding sequence

**2.1 Tack welding order (sparse, FEM-based).**
- **Jonsson, Karlsson & Lindgren (1985), Welding Journal 64(10):296s–301s**: FE simulation of tack procedures in butt welding of fine-grain steel plates (360 MPa yield); seven analyses (three tack procedures, four starting temperature fields). The change in root opening during the first part of welding depends on the order in which the tacks are made and on where the arc starts. This is the most direct peer-reviewed support that tack *order* matters. (Which procedure was best could not be verified from the full text.)
- **Abid & Siddique (2005), IJPVP 82(11):860–871**: 3-D coupled thermo-mechanical FE of a 100 mm pipe–flange MIG joint, validated experimentally; tack pairs at 0/180°, 45/225°, 90/270°, 135/315° relative to the weld start and root gaps of 0.8–2.0 mm. Axial displacement and flange tilt depend strongly on tack orientation and only weakly on root gap; follow-up work adopts 90°/270° tacks.
- **Abid & Siddique (2005), Acta Mechanica 178:53–64**: with tacks at 90° and 270°, the peak temperature reached by the tacks when re-heated during the butt weld is the key parameter for deformation; tack temperature has negligible effect on residual stress.
- **Shibahara, Serizawa & Murakawa (2002), Trans. JWRI 31(1)**, and **Murakawa et al. (2012), "Influence of Root Gap and Tack Weld on Transverse Shrinkage during Welding", Trans. JWRI 41(1)**: interface-element TEP-FEM. Transverse shrinkage increases almost linearly with initial root gap; the gap opens ahead of the torch at large heat input/speed and closes at small; tacks prevent gap opening and smaller tack pitch reduces it; in fillet welds the root gap closes in the first pass.
- **Heinze, Schwenk & Rethmeier (2012), JMPT 212(1):308–314**: 5 mm S355J2+N plate, pulsed GMAW, SYSWELD; distortion is only predicted adequately when tacks are modelled, best when modelled as actually fabricated.
- **Electron-beam shell–flange study (Int. J. Pressure Vessels and Piping, 2022)**: tack welds strongly mitigated opening/sliding gap distortion; circumferentially continuous tacks outperform discrete tacks; more extensive tacking gives less gapping.
- **Ti-6Al-4V EBW study (Measurement, 2021)**: tack operation before full-pass welding lowered angular deflection and plastic strain.

**2.2 Resistance spot weld (RSW) sequencing (mature; geometry-driven, not thermal).**
In body-in-white work, heat is generally *not* modelled; distortion is computed with compliant variation simulation (CAT tools, method of influence coefficients) where each spot weld locks nodes. Key results:
- Wärmefjord, Söderberg & Lindkvist (ASME IMECE 2010): strategies based on general guidelines were not successful, nor was step-wise variance minimization; a strategy based on the movement of the not-yet-welded points gave the best or one of the better sequences in all eight industrial cases. Their NordDesign 2010 paper (Gothenburg) adds that "after two welding points were executed, the geometry was quite stable, so… two geometry points would be enough" to lock it.
- Tabar, Wärmefjord & Söderberg: GA/PSO/ACO comparison (Procedia CIRP 2018), rule-based GA initialization (J. Manuf. Sci. Eng. 2019), surrogate (IJAMT 2020), stepwise algorithm (Engineering Optimization 2020/2021), critical-weld identification showing a small fraction of welds removes >60% of deviation variability (J. Intelligent Manufacturing 2021).
- Huang, Hsieh & Arora (1997) GA for spot weld sequence; Xie & Hsieh, clamping and welding sequence for cycle time and deformation; Beik, Marzbani & Jazar (2019) review: GA is the predominant method.
- Distortion-oriented path optimization with elastic net + GA (JMPT 2008/2009) combines minimum cycle time and minimum distortion via a thermo-mechanical model—the closest RSW analogue to "travel time + distortion penalty".

**2.3 Laser/stitch/multi-seam sequencing.** Kadivar et al. (2000) divided a circular patch into eight segments and showed clear sequence effects; Sattari-Far & Javadi (IJPVP 2008) studied sequence effects in pipes; Shao et al. (2018) optimized sequence and direction of a bogie side beam with a surrogate + discrete PSO, reporting ~22% lower deformation and ~38% lower stress than the enterprise plan; an aluminum pipe-structure GA study (IJPEM 2020) optimized both sequence and direction with TEP-FEM.

**2.4 Robotic sequencing as a combinatorial problem.** Standard formulations are TSP/shortest-sequence variants with robot IK choices (robotic TSP/GTSP). Thermal coupling appears as: (i) minimum cooling time / interpass temperature constraints (DED greedy planner; Caterpillar weld-sequence patents using NSGA-II reporting 50% shorter cycle time at equal distortion versus fixed cooling times—note patent claims, not peer-reviewed); (ii) minimum distance between consecutive operations (TC-HBA, 2026); (iii) multi-objective PSO with FE deformation (Wang, Min & Gu, IJARS 2019).

**2.5 Standards/industry guidance.** AWS D1.1 governs tack quality (tacks become part of the weld or must be removed) and preheat, not order. Pipe welding training texts and fabricator guides consistently specify four equally spaced tacks at 12, 3, 6 and 9 o'clock (or 11, 2, 5, 8), with tack length ~¾ in (≈19 mm) and 5–10% of diameter on large pipe; one tutorial explicitly orders 12 → 6 (opposite) → 3 → 9. TWI guidance shows three tack procedures (straight through with clamps/wedges, one end then back-step, centre then back-step). Treat these as practice justification, citing them as such.

### 3. FEM-backed rules you can adopt as heuristics/costs

| Rule | Evidence (type) | How to encode |
|---|---|---|
| Tack order changes root opening; arc start matters | Jonsson et al. 1985 (FEM) | Order is a decision variable; penalize sequences leaving long free spans early |
| Lock geometry with the first 1–2 tacks, far apart | Wärmefjord et al. 2010 (RSW variation sim.) | Reward large spacing / opposite positions for tacks 1–2 |
| Opposite/symmetric pairs on circumferential joints | Abid & Siddique 2005 (FEM); pipe practice | Thermal-centroid imbalance penalty; angular-balance term |
| Smaller tack pitch limits gap opening | Murakawa et al. 2012 (FEM) | Penalize max unconstrained span after each step |
| Transverse shrinkage ∝ root gap | Murakawa et al. 2012; Spraragen–Ettinger | Weight tacks at larger measured gap earlier/later as per pull-in model |
| Hot tacks (re-heated) drive deformation | Abid & Siddique, Acta Mech. 2005 | Penalize preheat temperature at each tack location when fired |
| Avoid consecutive operations close together | TC-HBA 2026; DED interpass 2025 | Minimum distance / temperature-threshold constraint |
| Center-out alternating on long seams | Practice (TWI, fabricator guides) | Baseline sequence and/or prior for initialization |

## Recommendations

### Method 1 (core thesis method): Analytical superposition thermal cost + rule penalties + travel, with exact/heuristic search

**Model.** For tacks i = 1..N at positions x_i (from your seam extraction), heat Q_i = η·U·I·τ_i, firing times t_i = t_{i−1} + τ_{i−1} + d(x_{i−1}, x_i)/v_robot + t_settle. Temperature from superposed thin-plate line sources (or semi-infinite point sources for thick sections), with mirror sources for edges within ~3–4·√(a·t_cycle) and periodic images for cylinders.

**Cost function for a permutation π:**
J(π) = w₁·Σ_k [T(x_{π(k)}, t_k⁻) − T₀] (residual heat at each tack site when fired — "re-heated tack" proxy)
 + w₂·Σ_k max_x [T(x, t_k) − T_lim]⁺ (local heat accumulation / overheating)
 + w₃·Σ_k ‖c_T(t_k) − c_geom‖ (thermal-centroid imbalance; zero for symmetric heating)
 + w₄·Σ_k L_free(k) (longest unconstrained seam span after k tacks — gap-opening proxy from Murakawa 2012/Jonsson 1985)
 + w₅·[1 − s(π(1), π(2))] (geometry-lock term: normalized separation of first two tacks)
 + w₆·T_travel(π)
Normalize each term by its value for the linear baseline so the weights are interpretable, and report Pareto fronts (thermal/mechanical vs. cycle time) rather than a single weighted optimum.

**Search.** N ≤ 9: exhaustive enumeration (9! = 362,880 sequences; with an analytic evaluator of O(N²) source evaluations this runs in seconds in NumPy/C++). N ≤ ~15: branch-and-bound (the partial cost is monotone non-decreasing, so lower bounds are trivial) or beam search. Larger N: simulated annealing or GA with swap/2-opt moves, seeded with center-out and opposite-pair heuristics (as Tabar et al. did with rule-based initialization). Exploit symmetry on cylinders (rotations/reflections) to cut the search by 2N.

**Inputs.** Material (k, ρc, a for mild/stainless steel at a mean temperature), plate thickness and edge geometry from the point cloud, tack heat input (from your WPS: U, I, τ, η), robot travel times from the ROS 2 / MoveIt planner.

**Assumptions to state.** Constant properties, linear conduction, adiabatic or simple linear-loss surfaces, no fixture conduction (or a single effective loss coefficient fitted to IR data), tacks as instantaneous/short sources. Thermal cost is a proxy for distortion; mechanical terms are rule-based.

### Method 2 (optional mechanical extension): Sequential-stiffness shrinkage model

Represent the joint as two compliant members (beam/plate strips or ring segments for pipes) connected by springs that are activated as each tack is made; each tack imposes a transverse pull-in δ_i from the empirical shrinkage formula (scaled by the local measured gap g_i, and by local temperature at firing time from Method 1), and the resulting gap profile g(s) is updated by a small linear solve. Because stiffness changes as tacks are added, the model is order-dependent—exactly the mechanism that inherent-deformation/interface-element FEM captures, but at the scale of an N×N system. Cost: final gap variance or max |g(s) − g_nominal| plus misalignment. This is a lightweight analogue of the method of influence coefficients used in spot-weld variation simulation. Present it as a reduced-order model inspired by Ueda/Murakawa/Deng and Wärmefjord/Söderberg, and calibrate δ with 3–5 measured tacks.

### Method 3 (fallback/validation): FEM-calibrated rules

Run a small number of FEM or inherent-strain cases (e.g., in a free/academic tool) only to (a) check that Method 1's ranking of 5–10 candidate sequences matches the FEM ranking (Spearman rank correlation), and (b) calibrate weights. This mirrors the semi-analytical WAAM approach: FE used for calibration, analytical model used for search.

### Experimental validation in your UR5e cell

- **Specimens:** (i) 300–500 mm butt joint in 2–3 mm mild steel with a set root gap (e.g., 1.5 mm), 5–7 tacks; (ii) T-joint; (iii) a pipe/cylinder with 4 and 8 tacks.
- **Sequences:** linear left-to-right; current fixed angular 0/90/180/270; pipe practice 0/180/90/270; center-out alternating; Method 1 optimum; Method 1 worst (to show range). At least 3 repetitions each.
- **Thermal:** IR camera (calibrated emissivity—paint or tape for steel) plus 2–4 Type-K thermocouples near tack sites; compare measured vs. predicted temperature at tack sites at firing time (the J₁ term).
- **Geometric:** gap along the seam before and after tacking with feeler gauges or a laser line scanner/3D scanner as ground truth; use your RGB-D camera only if its depth noise is shown to be well below the expected changes (tenths of a mm), otherwise report it as secondary. Also measure misalignment (hi-lo) and angular distortion.
- **Statistics:** report mean ± SD of gap change, max deviation, and cycle time; test differences with ANOVA or Wilcoxon tests; show whether model-predicted ranking correlates with measured ranking.

## Annotated Bibliography (grouped)

**Analytical thermal models (well-established)**
- Rosenthal, D. (1941). Mathematical theory of heat distribution during welding and cutting. *Welding Journal* 20(5):220s–234s; and (1946) The theory of moving sources of heat and its application to metal treatments, *Trans. ASME* 68:849–866. Foundation of point/line source solutions and superposition.
- Eagar, T.W. & Tsai, N.S. (1983). Temperature fields produced by traveling distributed heat sources. *Welding Journal* 62(12):346s–355s. Gaussian source as superposition of point sources; reduces to Rosenthal in the limit.
- Nguyen, N.T., Ohta, A., Matsuoka, K., Suzuki, N., Maeda, Y. (1999). Analytical solutions for transient temperature of semi-infinite body subjected to 3-D moving heat sources. *Welding Journal* 78:265s–274s.
- Nguyen, N.T., Mai, Y.W., Simpson, S., Ohta, A. (2004). Analytical approximate solution for double ellipsoidal heat source in finite thick plate. *Welding Journal* 83:82s–93s.
- Fachinotti, V.D., Anca, A.A., Cardona, A. (2011). Analytical solutions of the thermal field induced by moving double-ellipsoidal and double-elliptical heat sources in a semi-infinite body. *Int. J. Numer. Methods Biomed. Eng.* doi:10.1002/cnm.1324. Corrects Nguyen for unequal semi-ellipsoids.
- Chujutalli, J.H., Lourenço, M.I., Estefen, S.F. (2020). Experimental-based methodology for the double ellipsoidal heat source parameters in welding simulations. *Marine Systems & Ocean Technology*. doi:10.1007/s40868-020-00074-4. Uses Fachinotti's analytical solution to fit Goldak parameters.
- Goldak, J., Chakravarti, A., Bibby, M. (1984). A new finite element model for welding heat sources. *Metall. Trans. B* 15:299–305. Source of the double-ellipsoid model.
- Textbooks: Radaj, *Heat Effects of Welding* (Springer, 1992); Goldak & Akhlaghi, *Computational Welding Mechanics* (Springer, 2005); Masubuchi, *Analysis of Welded Structures* (Pergamon, 1980); Carslaw & Jaeger, *Conduction of Heat in Solids*.

**Inherent strain / lightweight distortion (established in shipbuilding; lightweight FE)**
- Liang, W., Deng, D., Sone, S., Murakawa, H. Prediction of welding distortion by elastic finite element analysis using inherent deformation estimated through inverse analysis. *Welding in the World*. doi:10.1007/BF03266500.
- Deng, D., Murakawa, H., Liang, W. (2007). Numerical simulation of welding distortion in large structures. *Comput. Methods Appl. Mech. Eng.* Elastic FEM with inherent strain and interface elements for gap and local shrinkage.
- Murakawa, H., Deng, D., Ma, N., Wang, J. (2012). Applications of inherent strain and interface element to simulation of welding deformation in thin plate structures. *Computational Materials Science* 51.
- Review: Mitigation of welding-induced distortion based on FEM analysis (2020). *Journal of Welding and Joining* 38(1). doi:10.5781/JWJ.2020.38.1.6. Recommends inherent-strain elastic FEM inside AI-based sequence optimization.

**Tack weld studies (FEM; niche but directly relevant)**
- Jonsson, M., Karlsson, L., Lindgren, L.-E. (1985). Simulation of tack welding procedures in butt joint welding of plates. *Welding Journal* 64(10):296s–301s. Root-opening change depends on tack order and arc start.
- Abid, M., Siddique, M. (2005). Numerical simulation to study the effect of tack welds and root gap on welding deformations and residual stresses of a pipe-flange joint. *IJPVP* 82(11):860–871. doi:10.1016/j.ijpvp.2005.06.008.
- Abid, M., Siddique, M. (2005). Finite-element simulation of tack welds in girth welding of a pipe-flange joint. *Acta Mechanica* 178:53–64. doi:10.1007/s00707-005-0241-3.
- Shibahara, M., Serizawa, H., Murakawa, H. (2002). Finite element analysis using interface elements for predicting deformation during butt welding considering root gap and tack welds. *Trans. JWRI* 31(1):63–70.
- Murakawa, H. et al. (2012). Influence of root gap and tack weld on transverse shrinkage during welding. *Trans. JWRI* 41(1).
- Heinze, C., Schwenk, C., Rethmeier, M. (2012). The effect of tack welding on numerically calculated welding-induced distortion. *JMPT* 212(1):308–314. doi:10.1016/j.jmatprotec.2011.09.016.

**Welding sequence optimization (FEM/surrogate-based; mature)**
- Kadivar, M.H., Jafarpur, K., Baradaran, G.H. (2000). Optimizing welding sequence with genetic algorithm. *Computational Mechanics* 26:514–519. doi:10.1007/s004660000195.
- Voutchkov, I., Keane, A.J., Bhaskar, A., Olsen, T.M. (2005). Weld sequence optimization: the use of surrogate models for solving sequential combinatorial problems. *CMAME* 194.
- Asadi, M., Goldak, J. (2011). Combinatorial optimization of weld sequence by using a surrogate model to mitigate a weld distortion. *Int. J. Mech. Mater. Des.* 7.
- Romero-Hdz, J., Saha, B., Toledo-Ramirez, G., Beltran-Bqz, D. (2016). Welding sequence optimization using artificial intelligence techniques: an overview. *Int. J. Comput. Sci. Eng.* 3:90–95; and Romero-Hdz et al. (2018) RL-based WSO, *Transactions on Intelligent Welding Manufacturing*.
- Shao, Q. et al. (2018). Optimization of the welding sequence and direction for the side beam of a bogie frame based on discrete PSO. *Proc. IMechE Part B*. doi:10.1177/0954405416667432.
- Self-learning method with domain knowledge integration for intelligent welding sequence planning (2025). *Scientific Reports*. doi:10.1038/s41598-025-11333-y.
- Sattari-Far, I., Javadi, Y. (2008). Influence of welding sequence on welding distortions in pipes. *IJPVP* 85:265–274. doi:10.1016/j.ijpvp.2007.07.003.

**Spot weld sequencing (mature; variation-simulation based)**
- Wärmefjord, K., Söderberg, R., Lindkvist, L. (2010). Strategies for optimization of spot welding sequence with respect to geometrical variation in sheet metal assemblies. *ASME IMECE 2010*.
- Tabar, R.S., Wärmefjord, K., Söderberg, R. (2018). Evaluating evolutionary algorithms on spot welding sequence optimization with respect to geometrical variation. *Procedia CIRP* 75:421–426.
- Tabar, R.S. et al. (2019). A novel rule-based method for individualized spot welding sequence optimization with respect to geometrical quality. *J. Manuf. Sci. Eng.* 141(11):111013.
- Tabar, R.S., Wärmefjord, K., Söderberg, R. (2020). A new surrogate model–based method for individualized spot welding sequence optimization. *IJAMT*. doi:10.1007/s00170-019-04706-x; and Rapid sequence optimization of spot welds using a novel stepwise algorithm, *Engineering Optimization*. doi:10.1080/0305215X.2020.1757090.
- Beik, V., Marzbani, H., Jazar, R. (2019). Welding sequence optimisation in the automotive industry: a review. *Proc. IMechE Part C*. doi:10.1177/0954406219859909.

**Robotic path/sequence planning with thermal or distortion terms**
- Distortion-oriented welding path optimization based on elastic net method and genetic algorithm. *JMPT* (2009).
- Wang, X.-W., Min, Y., Gu, X.-S. (2019). Multi-objective path optimization for arc welding robot based on discrete DN multi-objective PSO. *Int. J. Adv. Robot. Syst.* doi:10.1177/1729881419879827.
- TC-HBA: a thermal-constrained honey badger algorithm for PCB drilling path planning (2026). *IJAMT*. doi:10.1007/s00170-026-19031-3. Minimum-distance thermal proxy in TSP.

**WAAM/DED heat-aware ordering (directly transferable)**
- A semi-analytical approach to WAAM simulation for deposition sequence optimisation (2024). *Virtual and Physical Prototyping*. doi:10.1080/17452759.2024.2368648.
- Path planning for rapid DEDAM processing subject to interpass temperature constraints (2025). *Metals* 15(6):570. doi:10.3390/met15060570.
- Selection of trajectories to improve thermal fields during the electric arc welding process using hybrid model CFD-FNN (2025). *Metals* 15(2):154. doi:10.3390/met15020154.
- Scalable path planning and reduced-order modeling for WAAM (ETH Zurich) and "Too hot to print, too slow to handle; finding optimal path characteristics for WAAM" (2024, *Manufacturing Letters*). ROM + MCTS.

**Practice and guidance (non-peer-reviewed; cite as practice)**
- Pipe welding training texts (e.g., LWTech open textbook) and fabricator tutorials: four tacks at 12/3/6/9, opposite-first ordering; TWI FAQ on tack procedures; AWS D1.1 for tack quality requirements.

## Caveats

- **Literature gap (your contribution):** I found no peer-reviewed paper that optimizes tack *order* using an analytical thermal model; existing tack-order evidence is FEM-based (and only a handful of studies), and the most direct one (Jonsson et al. 1985) could not be verified for which sequence was best. Frame the thesis as transferring a validated WAAM/DED approach to tack sequencing.
- **Thermal cost ≠ distortion.** Heat balance is a proxy; tack-order effects are also mechanical (constraint changes as tacks are added, gap opening ahead of the arc). That is why Method 1 includes rule-based mechanical terms and why validation must measure geometry, not only temperature.
- **Some rules rest on practice sources**, not peer-reviewed studies (center-out, 12→6→3→9). Cite them as industry practice; codes do not prescribe tack order.
- **Secondary-source numbers:** several figures (e.g., reported gains of GA/RL methods, patent claims of 50% cycle-time reduction) come from abstracts, reviews or patents and were not independently verified from full text.
- **Analytical model limits:** constant properties, no latent heat, and idealized boundaries; near-source temperatures will be wrong, but relative ranking of sequences—what you need—is much less sensitive. Calibrate one heat-loss parameter against IR data.
- **Small N reality:** with 4 tacks there are only 24 orders (3 distinct up to cylinder symmetry), so improvements will be modest; demonstrate the method on parts with 6–12 tacks where ordering has real leverage.