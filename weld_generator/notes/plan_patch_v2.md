# Plan patch — task definitions, camera regimes, seven-method comparison

*Drafted from the 2026-08-20 advisor meeting. Paste into `dataset_plan.md`; section numbers
are suggestions. Decisions continue the D-series at D25.*

---

## New §3 — Task definitions

The plan has so far assumed one ground truth per scene: the set of weldable seams. The
advisor meeting split that into **two tasks with different label types**, and the second one
has a label that is not derivable from geometry alone. Both are supported by the same
generator; they differ in what is given and what is asked.

### Task 1 — Complete seam recovery

> *"Varsayalım ki sen bunu mükemmel taradın. Al olası tüm seam'leri."*

**Given:** the full-geometry cloud, no occlusion.
**Return:** every weldable seam, each with its class and geometry.

This is what Phases 1–3 already produce, and its label is **exact geometry** — no convention
enters anywhere. It is the task the dataset's central claim rests on.

Because generation is inverted (D3), the per-point form of this label is free: every point
carries `face_id`, and every seam names the `face_pair` that produced it, so each surface
point can be labelled with the seam it belongs to, or none. That makes Task 1 posable three
ways from one file — per-point segmentation, per-seam instance grouping, or curve regression
— without storing anything new.

### Task 2 — Most probable seam (MPS)

> *"Al, baktığın açıdaki en olası seam'i bul."*

**Given:** one view.
**Return:** one seam — its **class** and its **location**.

The advisor's justification, and the assumption the paper must state outright:

> *"Genelde algoritma şöyle çalışıyor: önce bir kaynak bölgesine git. Gittin mi kaynak
> seam'ini bul."*

The robot is coarsely positioned into the weld region first; seam finding happens afterwards.
The camera is therefore already inside the approach region and is not sampling an arbitrary
sphere. This is a citable workflow assumption, not a convenience — and it is what makes a
single-answer task well-posed at all. Draw it as a figure.

**The MPS label is a convention, not truth.** This is the fourth instance of a pattern the
project already follows — tacks (D8), the noise realisation (D14), the seam-under-gap choice
(D19) — and it should be handled identically: **ship a versioned rule function over the exact
geometry, never a stored label.**

### Scope: what was declined from the meeting, and why

Recording these so they are not silently reopened.

| Proposal | Status | Reason |
|---|---|---|
| Ground truth conditioned on **loading conditions** | **Out of scope** | Which seam carries load is structural knowledge, not visible geometry. No vision method can recover it, so it cannot be a vision label. Agree explicitly with the advisor that MPS is a *geometric proxy* adopted for tractability, and say so in the paper — otherwise it arrives as a reviewer question instead of a stated limitation |
| **Partial seams** (pre-welded sections, weld only the middle) | **Out of scope** | Requires knowing which regions are already welded. Nothing in a single view carries that, so the label would encode information the input cannot contain |
| **Same seam, varying part sizes** | **Constrained, not dropped** | See D27 — dimensions across the seam are free; the along-seam dimension is pinned by the seam |

---

## D25 — The MPS label is a versioned rule, not stored truth

```
mps_rule-0.1:  argmax over weldable seams of visible arclength
               ties broken by (larger dihedral fold, then lower seam id)
               returns null if no seam has visible arclength > min_len_mm
```

Advisor's formulation: *"görünen yerdeki en büyük seam'i alıyoruz"* — *"şurada kısacık kalıyor
senin bir seam'in, burada kocaman bir seam var. Senin ground truth'un bu."*

**Why the rule survives the no-partial-seams decision.** With partial seams out, seam length
equals contact length, so the two fillets of a T-joint and the two toes of a lap joint have
*equal* total length — an argmax over total length would tie in most scenes. The rule is over
**visible** arclength, which differs by viewpoint by construction. That is the whole point of a
view-conditioned task, and it is already computed: `1 − occluded_fraction` times `length_mm`.

Consequence to check in Phase 4: under the current bimodal occlusion distribution (58% of
seams above 0,98 occluded, 40% at exactly 0), visible arclength is close to binary, so MPS
will usually reduce to "the one seam that is visible at all." That is a legitimate answer but a
weak task. Grading the occlusion distribution — see D26 — is what gives MPS teeth.

Store `mps` alongside `tacks` as a derived block: `{rule_version, params, seam_id, class}`.

---

## D26 — Two camera regimes, sampled per config

The Phase 3 gate ("the sampler must not be polite; `occluded_fraction` must span 0 → 0,8")
encoded an assumption the meeting overturned. Under coarse positioning, a polite sampler is
the *correct* model of deployment. Keep both rather than replacing one with the other.

| Regime | Sampling | Answers |
|---|---|---|
| `approach_cone` | from the target seam's well-observed viewpoint region | the deployment condition — Task 2 |
| `uniform_sphere` | the current sampler | stress test, occlusion characterisation — Task 1 |

The advisor's *"Swiss cheese slice"* remark is worth taking literally: the good-viewpoint
region has holes and is not a clean cone. **It is already computed** — the azimuth × elevation
visibility map in `notebooks/02`. Sample from it empirically rather than parameterising a cone.

Two things fall out:

- The `NoVisibleSeams` yield problem largely disappears under `approach_cone`, because the
  camera is drawn from a region where the seam is observable by construction.
- The `approach_cone` / `uniform_sphere` delta, on paired seeds, **measures what coarse
  positioning is worth** — another paired ablation in the same style as fixture on/off.

Both regimes must be twin-paired so the comparison is exact.

---

## D27 — Part dimensions vary across the seam, never along it

The advisor proposed one seam expanding into many scenes via a list of plate sizes. That is
right in the directions perpendicular to the seam and wrong along it: growing a plate along
the seam axis lengthens the contact run, and with partial seams out of scope the seam length
*is* the contact length — so the seam changes and it is no longer the same seam.

```
along-seam dimension     pinned by seam length
across-seam dimensions   free (width, height, overhang)
thickness                free, subject to the ISO 9692-1 preparation bounds
```

This makes "same seam, many parts" a well-defined family, which is what the advisor wanted,
without the contradiction. It also gives a clean held-out axis: train on narrow plates, test
on wide ones, same seam.

---

## Revised Phase 4 — seven methods

### Framing note

`ours` is **not claimed as a novel extractor**. It is one entry in the comparison, carried
into the thesis as the method the robot pipeline uses. This is worth stating plainly in the
paper, because it removes the largest review risk in the whole project: the contribution is
the **dataset and the comparison**, and a reviewer cannot attack the dataset by attacking a
novelty claim that was never made. It also removes the tuning-fairness problem — with no
horse in the race, equal treatment of all seven is the obvious protocol rather than a
concession.

### The seven

| Name | Mechanism | Source |
|---|---|---|
| `ours` | radius-PCA curvature + nearest-point midpoint | the robot pipeline (`README §8`) |
| `lit-ransac` | improved RANSAC multi-plane fitting → plane intersection lines → inliers projected onto the weld vector for endpoints → dihedral for torch pose | Yi et al., *Automation in Construction* 2026 |
| `lit-ppf` | point-pair-feature coplanarity + voting for orthogonal plane pairs and their intersections; explicitly proposed as a RANSAC alternative on grounds of speed and threshold sensitivity | *Scientific Reports* 2024 fusion paper |
| `lit-regiongrow` | region growing seeded at the smoothest point by δ = λ₀/(λ₀+λ₁+λ₂), then least-squares fit of near-edge points | *Coarse-to-Fine Detection of Multiple Seams* |
| `lit-lobb` | local oriented bounding box descriptor, nonlinear feature activation, hierarchical K-means separating **face / crease / boundary / corner** points | Zhang et al., *IEEE T-ASE* 22, 2025 |
| `lit-pcaslice` | PCA-based adaptive slicing with the slicing direction determined from the data, centreline per slice | *3D vision-based intersecting pipe welding path planning* |
| `lit-modelreg` | registration of a CAD model to the scan, welding path transferred from the model | *A novel model-based welding trajectory planning method for identical structural workpieces* |

Two of these are chosen deliberately for what they add to the argument rather than for
performance. **`lit-lobb` classifies crease versus boundary points** — precisely the
distinction between a fillet seam and a lap toe, so it is the one literature method whose
feature space can in principle express your joint taxonomy. **`lit-modelreg` requires CAD by
construction**, which anchors the top of the oracle ladder with a real published method rather
than a hypothetical one.

### The coverage prediction

`lit-ransac` derives seams from plane **intersections**; `lit-ppf` is built on **orthogonal**
plane pairs. Neither mechanism can express a butt or edge seam, which come from the *coplanar
exposed* arm — two faces sharing a plane, no intersection line, no orthogonality. That is
structural, not a tuning failure, and it is the same discovery your own D4 enumeration made
when it needed a third arm.

The headline is therefore a **mechanism × D4-arm coverage table**, measured rather than
argued: *the classical literature is built on the fillet assumption, and its geometric
machinery cannot express half the joint taxonomy.*

### Protocol additions from the meeting

**Repeats and box plots.** *"Random consensus RANSAC'ın... aynı algoritmayı 100 defa koşsan
birebir aynı şeyi elde etmeyeceksin... box plot'lar, minimum ve deviation'ı gösterecek
şekilde."* `lit-ransac` and `lit-ppf` are randomised. Every number for them is a distribution:
100 repeats per condition, box plots with min and spread. The deterministic methods will show
zero spread — state that as a finding, since **method reproducibility is a property the
generator can measure and the field does not report**.

**Noise sweep as a table axis.** *"Zero noise'da her birinin başarısı, %5 noise'da, %10
noise'da."* Report two ways: sensor profiles (`d435i` / `stereo_good` / `stereo_poor`) for
physical realism, and a scalar multiplier on σ for a comparability axis readers can map onto
other papers. Define the percentage explicitly — as a multiple of the derived σ_z, not as a
fraction of range.

**The oracle ladder.** Unchanged from the previous draft, and now more valuable with seven
methods: L0 (xyz + `object_id` + exact normals, noiseless), L1 (no segmentation), L2 (xyz
only, normals estimated), L3 (L2 + noise + single view). `ours` depends on `object_id` through
its cross-object check, and `lit-modelreg` depends on CAD — so the ladder is not an artificial
axis, it is where two of the seven methods actually live. The L0→L1 delta is a number for
**how much each method depends on segmentation it does not publish about.**

### Order of work

1. Harness first — matching, metrics, per-scene dataframe — validated against a fake oracle
   predictor (ground truth plus noise) before any real method output flows through it.
2. `ours` adapter to `cloud.npz` (split on `object_id`), run on `reference_tjoint`, compare
   against `seams.npz` nominal. One hour. Do this before anything else.
3. Write down every input `ours` consumes. That list defines the ladder levels.
4. `lit-regiongrow` and `lit-lobb` next — both are pure feature-space methods with no
   registration or CAD dependency, so they are the cheapest faithful reimplementations.
5. `lit-ransac`, `lit-ppf` — add the repeat harness with these.
6. `lit-pcaslice`, `lit-modelreg` last; both may slip to Phase 6, and `lit-pcaslice` is most
   interesting there anyway, since slicing is aimed at curved seams.

---

## Open items opened by this patch

- [x] Confirm with the advisor that MPS is a **geometric proxy** and that load-path
      correctness is explicitly out of scope — CONFIRMED by the advisor (recorded
      2026-09-01); state it in the paper as a limitation, per the scope table above
- [x] Decide whether Task 2 is evaluated in paper 1 or deferred — DECIDED 2026-09-01 per
      the recommendation: Task 2 as a **short section** in paper 1 (advisor concurs);
      paper 1's core stays Task 1 plus the method comparison
- [x] Grade the occlusion distribution before MPS is evaluated — RESOLVED 2026-09-01 by
      Phase 6c(b): the `approach_cone` regime grades the margin where geometry permits
      (edge, lap, curved closed seams) and *measures* that T, corner and saddle margins
      are structurally pinned (complementary lobes / same-corner tie / single weldable
      seam) — so the MPS section reports per-class margin distributions rather than
      assuming a global graded axis exists. See dataset_plan Phase 6c for the numbers
- [x] Verify `lit-modelreg` is implementable without the original CAD assets — RESOLVED
      2026-08-21: implemented as a genuine reimplementation, with the "basic model" built
      CAD-by-construction from `scene.json` (the paper's own premise — identical
      structural workpieces — makes the generator's transforms the CAD); the L0-upper-
      bound fallback labelling never triggered
- [ ] **Candidate roster change under evaluation (opened 2026-09-01):** replace or
      augment `lit-modelreg` with `lit-nurbs` — the line-laser 3D-curve path/posture
      planning paper (`papers/3D curve weld seam path and posture planning based on line
      laser sensors_compressed.pdf`) — now that the corpus has curved families. Assess
      feasibility on area-scan clouds first; note what replacing modelreg costs (it
      anchors the top of the oracle ladder with a published CAD-dependent method)