# Tack sequencing: plan

*Written 2026-10-26 from `Quick literature review for tack ordering.md` (an AI deep-research
report) and `talk about tack ordering.md` (the question and an AI answer). Both were checked
against the code, the papers in `thesis/paper_pool/` and a few records online. Context:
`todo.md` → OPEN — tack ordering by a rough heat simulation; THESIS_PLAN #9.*

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
| Farthest-first traversal (Gonzalez 1985) gives ends → middle → quarters, and opposite pairs on a loop | yes, and that is exactly the rule's per-seam `order`; Gonzalez adds a proven guarantee for every prefix |
| "Your current order 0/90/180/270 is not the practice order" | the rule's `order` already is 0/180/90/270; it is the robot's order that is not |
| "The first two welds lock the geometry" (Wärmefjord et al.) | plausible, but the quote is in neither abstract (NordDesign 2010, DS 61 pp. 519–528; ASME IMECE 2010). Get the full text before quoting |
| "Tack order changes the root opening" (Jonsson, Karlsson & Lindgren) | the report says *Welding Journal* 1985, 64(10):296s–301s; Abid & Siddique's reference list says 1995. Verify |
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

**Why this holds up in a defence:**
- **A guarantee at every step.** After any k tacks, every planned tack position lies within
  r_k of a placed tack, and r_k ≤ 2·r*_k + ℓ. Here r*_k is the best any k tacks could
  achieve, and ℓ (one tack length) comes from the tie tolerance. Gonzalez's 2-approximation
  for k-center holds for every prefix of the traversal. The seam metric is a true metric (it
  satisfies the triangle inequality) once "farthest of all" is any distance ≥ the longest
  seam. So if the run stops early, the tacks already placed are still well spread.
- **It contains the rule.** On a single seam it is `tackrule-0.1`'s order, and the tests check
  that.
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
   read in its PDF.
2. **Where the gap is.**
   - Plate seams: keep which end each fit-up value belongs to (`_edge_fitup` sorts it away).
   - Curved seams: store the gap profile, or at least the arclength of its minimum.

   Done when a test with a tilted standing plate picks the low end.
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
4. **Into the cell.**
   - `marking.visit_order(report, mode)`, with the node parameter `tack_order`: `seam` (today),
     `rule` (the generator's `order`) or `sequenced`. The default stays `seam` until the bench
     run.
   - `tack_marks.json` records the sequence name and each step's start and end time.
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
- **To fetch:**
  - Gonzalez 1985, *Theoretical Computer Science* 38:293–306, doi:10.1016/0304-3975(85)90224-5
    (record checked online);
  - one heat-conduction source for the thin-plate line source (Carslaw & Jaeger, *Conduction of
    Heat in Solids*; or Rosenthal 1946; or Radaj 1992);
  - the Wärmefjord et al. NordDesign 2010 full text;
  - Jonsson, Karlsson & Lindgren (check the year);
  - optionally Abid & Siddique 2005 (tack orientation on a pipe-flange joint), and Hatala et
    al. 2025 (*Metals*; an analytic thermal model with a greedy order for DED, the closest
    precedent for "analytic model + greedy").
- **Do not cite from the AI report itself.** Several of its numbers come from abstracts or
  patents.

## Decision (THESIS_PLAN #9)

Proposed:
- this sequencing as a Ch. 3 section, with results in Ch. 6;
- a heat-optimising planner as future work, with the regime map saying when it would be
  needed.
