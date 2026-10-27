# Tack sequencing: plan

*Written 2026-10-26 from `Quick literature review for tack ordering.md` (an AI deep-research
report) and `talk about tack ordering.md` (the question and an AI answer). Both were checked
against the code, the papers in `thesis/paper_pool/` and a few records online. Context:
`todo.md` → OPEN — tack ordering by a rough heat simulation; THESIS_PLAN #9.*

*2026-10-27: steps 1–4 done. Every source the rules cite is read in its PDF
(`thesis/supply_papers/`, see Literature), and the sequencing is built and tested but not
yet run on the bench (the node's default order is still `seam`).*

## Where things stand

- `tackrule-0.1` already gives every tack an `order`:
  - on an open seam: ends first, then bisection;
  - on a loop: opposite pairs (0°, 180°, 90°, 270°);
  - across seams: seams of the same class interleaved round-robin, the classes taken in
    alphabetical order.
- **The robot does not follow it.** `marking.visit_order` goes seam by seam in `tack_no` order,
  and `tack_marks.json` says so (`'sequence': 'seam by seam, tack_no ascending'`). The robot
  tacks a pipe at 0°, 90°, 180°, 270°.
- The bench assemblies have **4 tacks each**: 2 per side of the 250 mm T at t = 8, and 4 round
  C1 or E2. Orders differ little at N = 4, so the method has to be shown on the corpus and in
  sweeps, where N is larger.
- The cell measures the root gap of every seam (`fitup_mm`, per member). Today only the two
  values (low, high) are kept, sorted, so where the gap is smallest is lost.
- The marking record has no per-step times.

## The two notes, checked

| claim | check |
|---|---|
| Superposed analytic point/line sources (Rosenthal, Rykalin) are an accepted non-FEM thermal model and cost microseconds | yes; textbook conduction |
| Heat barely couples the tacks on 20–30 cm parts | **true along a seam, false across a web** (table below). The talk's 3 kJ per tack is low for a 32 mm GMAW tack: at ~0.5–0.6 kJ/mm that is 15–20 kJ |
| Farthest-first traversal (Gonzalez 1985) gives ends → middle → quarters, and opposite pairs on a loop | yes on an open seam and on loops of 4 or 6 tacks, where the rule's per-seam `order` is such a traversal. **Not on loops of 8 or more** (checked 2026-10-27): the rule bisects the half-loop ends first, so after 0° and 180° it takes 135°. Its covering radius still matches on 8; on 10 and 12 it is twice the traversal's after 4 and 5 tacks. Gonzalez proves the bound for the largest cluster *diameter* (Theorem 2.2); the covering-radius bound used here follows from his Lemma 2.1 argument |
| "Your current order 0/90/180/270 is not the practice order" | the rule's `order` already is 0/180/90/270; it is the robot's order that is not |
| "The first two welds lock the geometry" (Wärmefjord et al.) | **verified in the full text (2026-10-27), NordDesign 2010.** The "geometry points" are welded first; "these welding points lock the geometry", and the rest "mainly with respect to cycle time" (§2.1). After two points "the geometry was quite stable" (§5, §6). The study is variation simulation of spot-welded sheet metal, not arc tacks |
| "Tack order changes the root opening" (Jonsson, Karlsson & Lindgren) | **verified (2026-10-27): *Welding Journal* (Research Supplement), October 1985, 296-s–301-s**; Abid & Siddique's "1995" is a misprint. FEM of 1 × 1 m plates, 10 mm thick, four 30 mm tacks: "the change in root opening during the first part of the welding depends strongly on the order in which the tack welds are made" (p. 301-s), about 0.1 mm between the orders (Fig. 15). Letting the first two tacks cool before the next raised the forces in the tacks, so "the tack welding procedure should be performed as fast as possible" (p. 301-s) |
| Pipe practice is 12 → 6 → 3 → 9 o'clock | textbooks give the positions (12/3/6/9 or 11/2/5/8, LWTech open textbook), not the order; the order is shop practice (forums). Cite it as practice, or not at all: farthest-first produces it anyway |
| A six-term weighted cost (w₁…w₆), a sequential-stiffness shrinkage model, validation with IR and thermocouples | not defensible here: no data to set the weights or calibrate the stiffness, and no torch or IR camera. Future work |
| **Lowth et al. 2015** (CIRP Annals; in the pool and in `thesis.bib`) | measured on GTAW-tacked Ti-6Al-4V rings: tacking the **contact end** first rather than the gap end gave **19× less out-of-plane distortion**. Once the first tacks are placed, later tacks over the gaps cost little. The mechanism is volumetric shrinkage, not thermal expansion |

**Heat at the next tack site.** Thin-plate instantaneous line source, mild steel (k = 45 W/m·K,
a = 1.2·10⁻⁵ m²/s), no surface losses (their time constant is ~10 min), Euclidean distance.
Order of magnitude only:

| next tack | 18 kJ, 8 mm, after 15 s | 4 kJ, 8 mm, after 15 s | 4 kJ, 3 mm wall, after 15 s |
|---|---|---|---|
| other side of a T web, same station (~14 mm) | 202 °C | 45 °C | 120 °C |
| next quarter of a 62 mm pipe (49 mm) | 9 °C (29 °C after 60 s) | 2 °C | 6 °C |
| one pitch along a seam (93 mm) | 0 °C (3 °C after 60 s) | 0 °C | 0 °C |
| other end of a 250 mm seam | 0 °C | 0 °C | 0 °C |

The diffusion length √(4aΔt) is 27 mm at 15 s and 54 mm at 60 s. The base plate's mean rise
after 4 tacks is 8–37 °C, whatever the order.

**So:** along a seam, the order is a question of holding the parts, not of heat. Between
seams that sit close together (the two sides of a web, a branch near a run pipe), heat decides.
That is exactly where farthest-first cannot choose.

## The method: gap-seeded farthest-first with a physics tie-break

1. **Candidates:** the tacks the reachability check has solved (`tack_reach.json`).
2. **First tack:** where the measured gap is smallest (Lowth et al. 2015). This applies only
   when the gap spread along that seam exceeds the gap's repeatability (the multi-view runs
   agreed to 0.4 mm); otherwise take the tack nearest the robot.
   - *As built (2026-10-27), per part pair:* the first tack between two parts goes to
     their smallest gap when it varies by more than 0.4 mm over their tacks.
   - The assembly's first tack is the smallest such gap. A pair entered later is entered at
     its own contact end.
3. **Next tack:** the candidate farthest from the tacks already placed (farthest-first,
   Gonzalez 1985), measured in a *seam metric*:
   - the distance along the seam to the nearest placed tack of the same seam;
   - the distance around the loop on a closed seam;
   - a seam that has no tack yet counts as farthest of all.

   Every seam is therefore held once before any seam gets its second tack. On one seam, the
   method reproduces the rule: ends, middle, quarters; opposite pairs on a loop.
4. **Ties:** candidates within one tack length of the farthest are equivalent. Among them,
   take in turn:
   1. the coolest site when it would be fired, from the thermal kernel;
   2. then the shortest robot transit (joint-space, between the reachability report's approach
      poses);
   3. then the lowest id.

There are no weights. Each level answers its own question, in a stated priority: holding the
parts (mechanics), heat (physics), cycle time (robot).

The heat level chooses only *where* the next tack goes among equivalent ones; it never adds
waiting. Jonsson et al. 1985 found that letting the first tacks cool before the next raised
the forces in the tacks, and advise tacking as fast as possible (p. 301-s).

**Why this holds up in a defence:**
- **A guarantee at every step.** After any k tacks, every planned tack position lies within
  r_k of a placed tack, and r_k ≤ 2·r*_k + ℓ. Here r*_k is the best any k tacks could
  achieve, and ℓ (one tack length) comes from the tie tolerance. Gonzalez's 2-approximation
  for k-center holds for every prefix of the traversal. The seam metric is a true metric (it
  satisfies the triangle inequality) once "farthest of all" is any distance ≥ the longest
  seam. So if the run stops early, the tacks already placed are still well spread.
- **It contains the rule.** On an open seam and on loops of 4 or 6 it is a `tackrule-0.1`
  order: the tests compare the covering radius at every prefix. On the loops of 10 and 12
  checked, it spreads better than the rule (see the table above).
- **Its first decision uses the cell's own measurement.** The measured gap per seam is
  something no generic planner has, and the rule that uses it is a measured result.
- **The heat model only decides where the geometry cannot.** It ranks candidates by distance
  and elapsed time. The unknown heat input Q enters only as a ratio between tacks, so the
  ranking does not depend on it. The rank stability is checked against ±30 % changes of the
  material constants.
- **Robot time is last.** Practice comes before productivity, and the price is measured.
  Jia et al. 2025 (in the pool) order tacks for travel alone; that is the comparison.

**What to call it:** "gap-seeded, thermally informed tack sequencing". Not "optimal", not
"distortion-minimising".

## Steps

1. **Literature, before any writing** (list below). Done when every source cited for a rule is
   read in its PDF. **Done 2026-10-27.**
2. **Where the gap is.**
   - Plate seams: keep which end each fit-up value belongs to (`_edge_fitup` sorts it away).
   - Curved seams: store the gap profile, or at least the arclength of its minimum.

   Done when a test with a tilted standing plate picks the low end.

   **Done 2026-10-27.**
   - Every seam in welding_seams.json has `fitup_along_mm`:
     - plate seams: each member's two near-edge corners, placed by arclength from `p0_mm`
       with their signed gaps; the gap is linear between them;
     - curved seams: the gap at every polyline point.
   - `gap_profile` and `gap_at` (seam_from_registration.py) read them.
   - Tests: `test_tilted_standing_plate_keeps_where_its_gap_is_smallest` and
     `test_tilted_pipe_keeps_where_its_gap_is_smallest`.
   - On the way: `_face_corners` takes a primitive's own `face_vertices`. Before, a prism's
     side face raised there; the registry builds no prisms yet, but the real set's polygon
     plates will.
3. **`admittance_control/tack_order.py`** (pure Python, tested):
   - the seam metric, farthest-first, the seed and the tie chain;
   - a simple thermal kernel: constant time per step, Euclidean distance, h = the thinner
     member's gauge (`member_gauge_mm`);
   - metrics: r_k per prefix, the step at which every seam is held, the predicted rise at each
     firing, travel.

   Done when the tests pass:
   - one open seam gives the rule's bisection order;
   - a loop of 4 gives 0, 180, 90, 270;
   - two parallel seams are each held once before any second tack;
   - on the two sides of a web, the second tack goes to the far end of the other side, not
     across from the first;
   - r_k ≤ 2·r*_k + ℓ against brute force on small random cases;
   - the result is deterministic;
   - the gap seed picks the contact end.

   **Done 2026-10-27.**
   - `admittance_control/tack_order.py`: `sequence`, `evaluate`, `best_radius`.
     `decided_by` records, per step, the criteria that narrowed the candidates in turn
     (e.g. `farthest>heat`, `gap>id`).
   - `test/test_tack_order.py`: 10 tests, the list above plus:
     - a loop of 10, where the rule is not a farthest-first order;
     - a pair entered later goes to its own smallest gap;
     - the web order survives ±30 % on every thermal constant.
   - Three mutations confirmed the tests can fail: heat off, `far` too small, and an
     id-only order (which breaks the bound in 39 of the 60 random cases).
   - Within a level of equal distances, heat or transit picks: on a four-tack seam, after
     both ends, which of the two inner tacks comes first. So the tests compare the covering
     radius at each prefix with the rule's, not the order within a level.
4. **Into the cell.**
   - `marking.visit_order(report, mode)`, with the node parameter `tack_order`: `seam` (today),
     `rule` (the generator's `order`) or `sequenced`. The default stays `seam` until the bench
     run.
   - `tack_marks.json` records the sequence name and each step's start and end time.

   **Done 2026-10-27; not yet run on the bench.**
   - `marking.plan_visit` / `visit_order(report, mode, ...)` builds the sites:
     - from welding_tacks.json and welding_seams.json: the gap at each tack, and the part
       pair from `face_pair`;
     - the transit cost is the largest joint change between the report's approach poses.
   - The node takes `tack_order` (`seam` | `rule` | `sequenced`, default `seam`) and
     `tack_order_step_s` (15 s, the heat model's time per tack until it is measured).
   - `tack_marks.json` now has:
     - `sequence` and `sequence_detail`: the order, `decided_by`, the gaps, the predicted
       rises, r_k, the hold step and the transit;
     - per mark, `t_start` (before the transit) and `t_end` (after the retract).
   - Tests: `test_visit_orders_seam_rule_and_sequenced` and
     `test_sequenced_order_starts_at_the_contact_end` (test_marking.py).
   - **Node dry run, 2026-10-27**, on a copy of the 2026-10-16 save dir (SP3 on its base, 4
     reachable tacks; a fake `/joint_states` on domain 77): `~/plan` returned PLAN OK.
     - The order: s0#3 > s1#2 > s1#1 > s0#2, decided by farthest>transit, farthest>transit,
       farthest>heat, only. The third tack went to the far start of side 1, not across the
       band from the second.
     - That save dir predates `fitup_along_mm`, so the gap rule could not act. Recompute the
       seams before the bench run.
     - Run the node with the workspace venv's `python3`; the system Python lacks `trimesh`.
5. **`tack_heat.py`, the analysis layer** (not in the loop):
   - superposition with mirror sources at plate edges and an unrolled, periodic strip for
     tubes;
   - a code check against an explicit 2-D finite-difference solver on a plate (a grid and a
     loop, not FEM);
   - a sweep giving a regime map: distance vs time between tacks, contours at 1, 10 and 100 °C,
     with the cell's own cases plotted on it;
   - the per-assembly temperature at each firing for every order;
   - a heatmap over the point cloud (a figure for the defence).
6. **Corpus evaluation**, offline, on WeldSet scenes with their exact seams (optionally on the
   extractors' seams). Orders compared:
   - O1 seam by seam (today's robot);
   - O2 the rule's `order`;
   - O3 the new sequencing;
   - O4 travel only: exact TSP for N ≤ 10, the Jia et al. objective;
   - O5 random.

   Metrics per stratum: rule conformance, r_k curves, hold step, predicted rise and a travel
   proxy (tool-tip path length, labelled as a proxy).
7. **Bench.** Dry run, then real. Three assemblies: the double-sided T, C1 on its plate, and E2
   or S3/R2. Run O1 against O3 (and O2), at least 3 runs each. Record:
   - that every mark is placed;
   - the planned and measured cycle time, i.e. what the order costs in seconds.

   Feed the measured step times back into step 5.
8. **Thesis:**
   - Ch. 3: a section after "Tacks", with the method, an algorithm box, the kernel equation and
     the assumptions;
   - Ch. 6: tables of rule conformance, r_k and time; figures of the regime map, the heatmap
     and the per-assembly orders;
   - Ch. 7: real tacks with a gap measurement, IR, FEM or inherent strain, sequence
     optimisation, and a `tackrule-0.2` order for the dataset.

## Limits to state

- **No distortion is measured** (no torch). The evaluation is rule conformance, guarantees,
  heat estimates and robot time.
- **Lowth's study is on titanium rings** with autogenous GTAW tacks. The gap seed is an
  adopted finding, cheap and harmless when the gap is uniform, not something proven on steel
  plates.
- **The thermal model is order-of-magnitude.** It assumes constant properties, no latent heat,
  instantaneous sources, and Euclidean distance (an underestimate round corners, so it errs
  hot). It is used to rank ties and to draw the regime map, not to predict temperatures.
- **The WeldSet `order` stays `tackrule-0.1`** (reproducible). This is a cell-side step.
- **Cooler is not always better.** Jonsson et al. 1985 found larger forces when the first
  tacks had cooled before the next ones. The heat level is a tie-break that avoids firing
  into still-hot material next to a fresh tack, never a reason to wait, and the thesis must
  not claim it reduces distortion.
- **Q matters only at the tie floor.** Between two candidates more than `heat_tie_c` (1 °C)
  apart, the ranking does not depend on the heat input Q. Q sets what counts as a tie, so
  the ±30 % check varies Q too.
- **Wärmefjord's two geometry points are a spot-welding result** (guns clamp the parts to
  nominal). Here they support only "the first decisions matter most", not a number of
  tacks.

## Literature

- **In the pool** (PDFs read in the 2026-10-20 review):
  - Lowth et al. 2015 (the gap seed);
  - Tomków et al. 2020 and Mondal et al. 2015 (tack distribution and sequence change
    distortion);
  - Jia et al. 2025 (travel-only tack sequencing on a robot);
  - Asadi & Goldak 2011 and Noh et al. 2022 (sequence optimisation with FEM or surrogates:
    the future-work direction);
  - Wu & Kim 2020 (review);
  - EN 1011-2 and JASS 6 (tack length).
- **Fetched and read, 2026-10-27** (`thesis/supply_papers/`):
  - **Gonzalez 1985**, "Clustering to minimize the maximum intercluster distance",
    *Theoretical Computer Science* 38:293–306.
    - Algorithm APPROX: an arbitrary first head, then the point farthest from its head
      (pp. 295–296).
    - Lemma 2.1 and Theorem 2.2: within 2·OPT when the weights satisfy the triangle
      inequality. OPT is the largest cluster *diameter*.
    - Section 4: 2 is best possible unless P = NP.
  - **Carslaw & Jaeger**, *Conduction of Heat in Solids*, 2nd ed., Oxford 1959. §10.2 III,
    eq. (10), p. 258: an instantaneous source in a thin sheet of thickness D losing heat
    through conductance H:
    v = Q′/(4πKDt)·exp(−κk²t − r²/4κt), k² = 2H/KD.
    With H = 0 this is the plan's kernel; the loss term's time constant ρcD/2H is ~10 min
    at 8 mm and ~4 min at 3 mm (H ≈ 25 W/m²K).
  - **Wärmefjord, Söderberg & Lindkvist**, "Variation Simulation of Spot Welding Sequence for
    Sheet Metal Assemblies", NordDesign 2010, Göteborg (§2.1, §5, §6, quoted above).
  - **Jonsson, Karlsson & Lindgren 1985**, "Simulation of Tack Welding Procedures in Butt
    Joint Welding of Plates", *Welding Journal* (Research Supplement), October 1985,
    296-s–301-s.
    - The order claim and "as fast as possible" (quoted above).
    - The volume number is not printed on these pages.
  - **Jonsson, Karlsson & Lindgren 1985**, "Deformations and Stresses in Butt-Welding of
    Large Plates With Special Reference to the Mechanical Material Properties", *J. Eng.
    Mater. Technol.* (ASME) 107:265–270, October 1985. The companion paper: one tack
    procedure (tacks 1 and 2, then 3 after 60 s, then 4 after another 60 s), and the steel's
    properties varied.
    - "The change in gap width depends also on the tack-welding procedure" (abstract).
    - Stronger tacks close the gap more at the start and open it less at the end
      (conclusions).
    - Tacks 1–2 end in compression, 3–4 in tension (§4.1).
    - It cites the Welding Journal paper as "accepted for publication", which dates that
      paper to 1985.
  - **Abid & Siddique 2005**, "Finite-element simulation of tack welds in girth welding of a
    pipe-flange joint", *Acta Mechanica* 178:53–64, doi:10.1007/s00707-005-0241-3.
    - Two tacks on a 100 mm pipe-flange.
    - The first tack's reheating is the key parameter for the axial distortion
      (conclusions).
    - Its p. 54 summary of Jonsson and Shibahara: tack sequence and spacing "have a
      significant effect on root opening and transverse shrinkage".
    - Its reference list dates the Welding Journal paper 1995: a misprint.
  - **Hatala, Reutzel & Wang 2025**, "Path Planning for Rapid DEDAM Processing Subject to
    Interpass Temperature Constraints", *Metals* 15:570, doi:10.3390/met15060570.
    - A modified Rosenthal solution as the reduced-order predictor.
    - A greedy choice of the next bead, "only optimal at the moment of decision" (§2.2.2,
      p. 7).
    - Pyrometer-validated, 9.2 % shorter build. The closest precedent for "analytic model
      + greedy".
  - **Tsai, Park & Cheng 1999**, "Welding Distortion of a Thin-Plate Panel Structure",
    *Welding Journal* (Research Supplement), May 1999, 156-s–165-s (the volume is not
    printed on the pages).
    - Weld sequences, not tacks: an FEM of an aluminium stiffened panel.
    - Welding from the more rigid joints toward the less rigid bent the skin least ("Optimum
      Welding Sequence by Joint Rigidity Method").
    - Context for Ch. 7 only.
- **Do not cite from the AI report itself.** Several of its numbers come from abstracts or
  patents.

## Decision (THESIS_PLAN #9)

Proposed:
- this sequencing as a Ch. 3 section, with results in Ch. 6;
- a heat-optimising planner as future work, with the regime map saying when it would be
  needed.
