# WeldSet material map: SCHEMA, notes, configs, tests, CLI (read-only audit, 2026-10-20)

**What I did.** I read every line of `docs/SCHEMA.md` (976 lines), `docs/scene.schema.json` (1334), `docs/render.schema.json`, `docs/MANUAL.md` (519), `README.MD` (821), `docs/PARAMETERS.md` (743), the six notes in scope, all 29 YAML configs, `requirements.txt`, `.gitignore` and `weldgen/cli.py`. I also read the header and test list of all 41 files in `tests/`.

I did targeted greps of `weldgen/` and `scripts/` to settle which side of each doc conflict is current. I used `git log`/`git show` only to date files and schema versions. Nothing was modified. I did not run pytest; test counts are static.

**Base path:** `/workspaces/welding_cell_ws/weld_generator/`

---

## 0. The most important flags (most likely to put an error into the thesis)

1. **Schema version labels disagree everywhere.**
   - The code emits `2.4.0` (`weldgen/__init__.py`: `SCHEMA_VERSION = "2.4.0"`, `GENERATOR_VERSION = "0.1.0"`).
   - The SCHEMA.md header line still says `2.3.0`, while its §0 documents 2.4.0.
   - `scene.schema.json` `$id` and `description` still say `2.2.0`; its enum is `["2.3.0","2.4.0"]`.
   - The README layout says SCHEMA and PARAMETERS are `v2.2.0`. PARAMETERS says `params_version: 2.2.0`.
   - The `weldgen/__init__.py` docstring says SCHEMA 2.0.0 and PARAMETERS 2.0.0. `tests/schema_contract.py` says 2.2.0.
   - Version 2.1.0 existed (git) but SCHEMA.md never records it.
2. **The semver policy in §0 was not applied from 2026-08-17 to 2026-10-19.**
   - Under the label "2.3.0", the JSON schema gained many new enum members: `prep` single_bevel/single_V/single_U; primitives prism, prepared_slab, prepared_prism; parametric kinds circle, ellipse, saddle, composite, offset; reject_reason `confined_bore`; the `camera.regime` enum.
   - It also gained many optional fields and the `mps` block.
   - By §0 each of these is a patch or minor bump. None was made.
   - The JSON schema as it stood on 2026-08-17 (git d4ef01d) had `prep` enum `["square"]`, primitives `slab/swept_slab/cylinder/tube`, kinds `line/arc/bspline`, and no `mps`. It would reject September scenes that are also labelled "2.3.0".
   - SCHEMA §7's "the patch-bump it was frozen for" (tacks) never happened.
3. **SCHEMA §2.5.1 contradicts SCHEMA §0, §2.6 and §4.1.** §2.5.1 still says off-class seams are `weldable: false, reject_reason: "wrong_class_for_joint"`. That reason was retired in 2.3.0; off-class seams now stay weldable with `matches_joint_type: false`. The JSON `seam_class` description and PARAMETERS §2.7 repeat the stale sentence.
4. **D31 changed the edge-joint class table, and SCHEMA.md does not say so.**
   - With `class_disjoint: true` (bench6a, bench_phase4, train_v1, smoke, grooved, task2), the code uses `ALLOWED_CLASSES_DISJOINT`, in which edge = {edge, lap_toe} (`weldgen/accessibility.py:103`).
   - The cross-run demotion is also skipped for lap and edge (`accessibility.py:326`).
   - SCHEMA §2.5.1, §2.6.2 and §4.1's "canonical" off-class example (the lap toe of an unequal-width edge joint), README "How a seam is found", the README seam-count table and PARAMETERS §2.7 all describe the pre-D31 behaviour.
5. **`confined_bore` is emitted but missing from the "frozen" vocabulary.**
   - It is emitted by `weldgen/verify_curved.py:247` and is in the JSON enum.
   - It is missing from SCHEMA §2.6.
   - It is the semantic bore gate, `bore_min_diameter_mm = 80` [ours] (README).
6. **SCHEMA §5.2 is stale and incomplete.**
   - The `seam_<i>_visible` row says `occluded_fraction = 1 - mean(visible)`, which is the pre-2.3.0 meaning. The code computes `1 - mean(unoccluded)` and `visible_fraction = mean(visible)`.
   - The table omits `seam_<i>_grooveroot` and `seam_<i>_dihedral`.
   - **Curved seams carry no `_root`/`_gapmid` arrays at all** (`weldgen/scene_curved.py:362-371`). This contradicts the table and the "already shipped as derived arrays, no change" line in patch_phase6 and patch_phase6b. The D19 triple exists only for square-prep plate seams.
7. **The content-hash recipe in SCHEMA §6.2 does not reproduce stored hashes.**
   - The code converts arrays to little-endian and hashes `dtype.str` (e.g. `"<f4"`), not `str(arr.dtype)` (`weldgen/hashing.py`).
   - `render.sha256` also hashes `depth_valid` and records undrawable views. SCHEMA says "depth + three masks".
8. **Substream 7 (`render`) is never used.**
   - Render draws come from `np.random.default_rng(int.from_bytes(sha256(f"{scene_id}|{render_id}")[:8],"big"))` (`weldgen/render/draws.py:35`), "never from the tier-1 substreams".
   - SCHEMA §6.1 and README Phase 8 ("on substream 7") say otherwise.
9. **There are no sensor-profile twins in the shipped corpora.**
   - SCHEMA §6.4 says "one scene per profile per geometry seed", and PARAMETERS §4.0 says the profiles are "generated in exact twins".
   - The code draws one profile per scene: `g5.choice(cfg["sensor_profiles"])` (`scene.py:314`, `scene_curved.py:275`). No shipped config pins a profile for paired arms (`sensor_ablation.yaml` does not exist).
   - The Phase 4 noise axis is a 0/1/2× multiplier on the stored profile's σ_z.
10. **The square-prep butt root gap does not follow ISO 9692-1 "b ≈ t".**
    - PARAMETERS §2.6, §3, §5.0 and §7 say the sampler scales butt gaps with t: b ≈ t (ref 1.2.1, t ≤ 4) and b ≈ t/2 (ref 2.1, t ≤ 8). §7 marks this "RESOLVED – the sampler must change".
    - The code draws every square-prep joint, butt included, from the ISO 5817 no. 617 fillet limit capped at 2 mm; below_D draws from [617 D limit, max(1.5×, 3 mm)] (`config.py:408-425`). Only grooved butts take their row's b (`config.py:487-498`).
    - Yet `iso_9692_ref` for butts is set to "1.2.1" or "2.1" from thickness alone (`scene.py:207-211`).
    - So PARAMETERS §5.0's "empty window at every thickness" butt result describes the ISO geometry, not the generated corpus.
11. **The radius-PCA validity-window bound is stale in PARAMETERS §5–§5.1.** Those sections use `R < t`. README Baselines corrected it to `R < t/2`: paired on 44 scenes, F1 0.709 → 0.897, band 5.70 → 3.31 mm. Every closure table in PARAMETERS §5–§5.1 would shift.
12. **Phase 9 matrix size disagrees.**
    - The phase9_plan body says 11 strata × 10 configurations × 5 views = 110 configurations, 550 frames (grooved butts excluded 2026-09-21).
    - Its own preamble says "the 120-configuration matrix".
    - README says "12 strata × 10 × 5". MANUAL §11 says 11.
13. **The Phase 9 real parts are metal, not MDF.** phase9_plan, MANUAL and README say metal from the lab inventory. SCHEMA §2.2.1, §5.4, PARAMETERS §3.2, PARAMETERS §3 (thickness rationale), last_talk and thesis_direction_handoff still say scanned (painted) MDF.
14. **There are two "Phase 5 annotation floor" numbers.**
    - MANUAL and README: 1.4 mm (1.415 mm) median lateral RMSE from the independent briefed annotator, 11.8 mm p95, lap 8.5 mm, "2.4×" the ~0.6 mm.
    - patch_phase6 header: "self-annotation returned 2,5 mm RMSE". The `scene.schema.json` `max_chord_error_mm` description calls 2.5 mm "the Phase 5 annotation floor".
15. **Method counts disagree.**
    - MANUAL §0 and §13 say "six published methods". MANUAL §8.1, §9 and §11, the README and the batch runner say seven literature methods (`lit-quadric` added 2026-09-08). MANUAL §9 item 3 says "five of six".
    - plan_patch_v2's "seven" included `ours`, which was excluded by ruling on 2026-09-03.
16. **Test counts.**
    - Static count today: 40 `test_*.py` files hold 417 test functions, which expand through parametrization to **579 collected items**.
    - MANUAL says 460 (written 2026-09-08). README says 564 (commit 4bf6747, 2026-09-21 12:47 UTC).
    - 579 = 564 + 15 items in four files added later.
    - `tests/schema_contract.py` is a standalone, stale script with 0 tests and is not collected.
17. **`weldgen verify` is not recursive.** It checks only immediate children of `--out` (`cli.py:87`). On a per-joint-type corpus such as `out/bench_phase4/T/...` it reports "0/0 scenes verify" with exit 0. MANUAL §6 says it "re-hashes everything on disk".
18. **`requirements.txt` is incomplete for the tests and the Phase 4 harness.**
    - `pandas` is imported by `harness.run_matrix`/`run_task2`, `run_phase4_batch`, annotation scoring and several tests.
    - `scikit-learn` is imported by `radius_pca` (DBSCAN).
    - Neither is listed. Pillow comes in transitively via matplotlib. Isaac Sim is intentionally absent.

Full registers follow in §6.

---

## 1. `docs/SCHEMA.md`, section by section

### Header (line 3) and version history

The header text reads: "**schema_version: 2.3.0** — 1.0.0 frozen 2026-08-13; 1.1.0 D12/D16; 1.2.0 D18–D21; **2.0.0 (2026-08-15) removes `objects[].features`** — D17 withdrawn; **2.3.0 (2026-08-17)** adds `seams[].matches_joint_type`, `in_frame_fraction`, `visible_fraction` and **narrows `occluded_fraction`**". It calls itself the Phase 0 deliverable of `notes/dataset_plan.md` §4; its companion is PARAMETERS.md.

Version history reconstructed from git (read-only):

| Version | When (git, UTC) | Commit | What changed |
|---|---|---|---|
| 1.0.0 | frozen 2026-08-13 | — | initial freeze |
| 1.1.0 | 2026-08-13 | a6c08af (first JSON, const "1.1.0") | "minor bump 2026-08-13 for D12/D16/D17": fixture, sensor profiles, features (D17 later withdrawn) |
| 1.2.0 | between 08-13 and 08-15 | (no separate commit) | D18–D21 + ISO 9692-1. By 4d297f3 the JSON requires `seam_definition` and `accessibility` |
| 2.0.0 | 2026-08-15 17:43 | 4d297f3 | major: removed `objects[].features` (D17 withdrawn). No scenes existed yet, so it cost neither a regeneration nor a DOI |
| 2.1.0 | 2026-08-16 16:53 | 70a9adb | JSON only: reject_reason `toe_of_centreline`. **Not in the SCHEMA.md header** |
| 2.2.0 | 2026-08-17 11:48 | 458c49f | `seams[].seam_class` (D22, §2.5.1); reject_reason `wrong_class_for_joint`; `too_short` widened to "…or than `min_seam_length_frac` of the joint's longest edge", later replaced by the direction test of §2.6.2 |
| 2.3.0 | 2026-08-17 15:47 | d4ef01d | adds `matches_joint_type` (required), `in_frame_fraction`, `visible_fraction`; narrows `occluded_fraction` to occlusion by another part; retires `wrong_class_for_joint`. JSON `$id`/description left at 2.2.0 |
| (still "2.3.0") | 2026-08-27 … 09-07 | 50cd0db, d80cca9, 0c8cb39, 1452743, a449d87, ac560f2, 12f4b33, b741948, 33fc820, bf84796, 07dbe6d, 7822176 | all Phase 6a/6b/6c/7 schema additions, no bump (see flag 2) |
| 2.4.0 | 2026-10-19 15:24 | 64d4325 | adds `contact_mode: "line"`; `schema_version` enum becomes `["2.3.0","2.4.0"]`; `SCHEMA_VERSION = "2.4.0"` |

### §0 What "frozen" means

- Adding a field after Phase 4 forces full regeneration (dataset_plan §8).
- The change policy is semver on `schema_version`:

| Change | Bump | Consequence |
|---|---|---|
| new optional field | patch | none |
| new required field or new enum member | minor | regenerate affected splits |
| meaning narrowed or widened, same name and type | minor, stated in the version note | regenerate; "the dangerous one", because readers do not error |
| field removed, renamed, retyped or unit changed | major | full regeneration, new DOI |
| enum member retired (kept valid, nothing emits it) | minor | none; enums are append-only |

- `schema_version` is the format; `generator_version` is the code. They move independently: a geometry bugfix bumps `generator_version` only.
- **2.4.0 (2026-10-19).** Adds `contact_mode: "line"`, a cylinder resting on the table along a line. It exists because the D12 fixture twins were extended from 6 plate strata to all 12. Five curved families rest flat on a plate or strip; the saddle's main pipe lies on the table ("user decision: on the table, not in V-blocks"). Affected split: `bench_phase4_fx`. 2.3.0 scenes stay valid. Because `schema_version` is hashed, regenerating a 2.3.0 corpus with current code reproduces everything except that string, so stored hashes no longer match.
- **2.3.0.**
  - `occluded_fraction` used to mean "everything the sensor did not return" (occlusion + image bounds + blind zone). It now means occlusion by another part only.
  - Framing moved to `in_frame_fraction`, and the conjunction is `visible_fraction`.
  - `weldable` had folded reachability and taxonomy together. Now `weldable` = reachability and `matches_joint_type` = taxonomy, and `wrong_class_for_joint` is retired. The motivating example: a lap toe in an edge scene would have been scored as a false positive.

### §1 Conventions

| Concern | Convention |
|---|---|
| Length | millimetres everywhere; metres only at the ROS boundary (`MESH_SCALE=0.001`, admittance README §13) |
| Angle | degrees in JSON; radians never stored |
| dtype | float32 for point data (resolves ~6e-5 mm at 1000 mm); float64 for transforms and parameters |
| Transforms | 4×4 row-major nested lists, last row [0,0,0,1]; `p_world = T_world_x @ p_x` |
| Naming | `T_<to>_<from>`, so `T_world_cam @ T_cam_pt` reads left to right |
| Handedness | right-handed throughout |
| NaN/Inf | never written; missing = `null` |

**§1.1 Frames.**
- `world`: all stored geometry; +Z up against gravity (load-bearing because `camera.elevation_deg` is measured from world XY). Nothing is pinned to the origin: the assembly pose is sampled (substream 3) and the fixture is tilted and z-offset (D12), to avoid the "contacts with z = 0 are never weldable" leak.
- `part`: origin at the primitive centroid, axes `(u,v,w)`, placed by `T_world_part`.
- `joint`: defined from seam id 0. Origin = seam0 start (s = 0); +X = unit tangent; +Z = the admissible torch axis at s = 0 (the bisector unless an obstruction forced a work-angle tilt; points into free space); +Y = Z × X. Stored as `T_world_joint`. It is "a derived convenience, not truth"; `parametric` is truth.
- `cam`: OpenCV optical (+X right, +Y down, +Z forward). To get the ROS body frame, apply `(x,y,z)→(z,-x,-y)`.

**§1.2 Torch approach direction and the sign disagreement.**
- `approach_dir = unit(n_A + n_B)` points from the seam toward the torch; `torch_z_axis = -approach_dir` (tool +Z into the work).
- Yi et al. write `v₀ = v₁ + v₂`; Wang et al. write `Z_W = −(n_b + n_m)`. It is the same geometry with opposite signs.
- The dataset stores `approach_dir` (torch-ward).

**§1.3 The seam definition at non-zero root gap (D19): a definition, not a measurement.**
- At g > 0 there are three candidate curves. Their spread is O(g); at the reference gap 1.1 mm [repo] that exceeds the ~0.6 mm RMSE the literature reports.
- Let F_A ⊂ Π_A and F_B ⊂ Π_B be the joint faces.

| Name | Definition | A↔B symmetric? | Moves with g? |
|---|---|---|---|
| `nominal` (**STORED**) | Π_A ∩ Π_B, the extended supporting planes, clipped to where both faces have support; the line at g = 0 | yes | no |
| `root` | boundary edge of the terminating face (for a T-fillet, the bottom edge of the standing plate's side face) | no | yes |
| `gap_mid` | midpoints of closest-point pairs between F_A and F_B | yes | yes |

- Nominal is stored for three reasons, in order of weight: it is a property of the design, not of the realised fit-up (a stable reference); it is A↔B symmetric (root would encode part ordering); it is continuous as g → 0.
- Closed form: for a 90° fillet with the standing plate lifted by g, the three are collinear with offsets **0, g/2, g**. That does not survive D18 (α sampled over 60–120°), so `root` and `gap_mid` are emitted as derived arrays (§5.2). The Phase 4 D19 conversion table is then a subtraction over arrays.
- What is scored: the harness defaults to `curve="nominal"`, and root/gapmid are selectable (`scripts/baselines/dataset.py:160`). `metrics.py` argues nominal is "the right curve" for plane-intersection methods.
- MANUAL §3.3 measured on the Phase 4 corpus: root offset ≈ one gap (×0.96 at acute fillets, ×1.25 at obtuse), gap-mid ≈ half. Root lies more than 3 mm from nominal on 23% of butt centrelines and 14% of fillets. Phase 5 annotators clicked gap-mid on butts and root on T-joints.
- Grooved seams store `_grooveroot` instead (D36). Curved seams store neither (code).

### §2 Naming registries

**§2.1 Objects.**
- `id`: "A","B","C"… in construction order; the fixture is always "F".
- `role`: "workpiece" | "fixture".
- `object_id`: uint8; workpieces 0,1,2… in `objects[]` order; **the fixture is always 255**.
- `primitive`: plain slabs through Phase 5; `prism` in 6a (D28); `swept_slab`, `cylinder`, `tube` in 6b. This list omits prepared_slab/prepared_prism, and cylinder is never emitted.
- `outline_uv` and `outline_shape` are prism-only (CCW convex outline in u-v, extruded over w ∈ [−t/2, +t/2]; `dims_mm` = bounding [L, W, t]).
- At most one fixture. When absent, `contact_mode = "free"` and no object_id 255 appears.
- Fixture presence is "sampled ~50/50 from Phase 2"; off in Phase 1.
- `objects[]` is open-ended (3-part assemblies would not need regeneration).

**§2.2 Face names.** Invariant: **`w` is always the thickness axis**. Broad faces are ±w, edge faces ±u, ±v. This is what makes the tack edge margin (2t) and linear misalignment (h along w) uniform across joint types.

| Primitive | Axes | Faces |
|---|---|---|
| `slab` (Ph. 1–2, fixture) | u = L, v = W, w = t | ±u ±v ±w |
| `prepared_slab` (6b, D35) | u = seam direction, v = width (prepared edge at v = 0), w = thickness (top at w = 0) | ±u (ends), −v (back), ±w (broad), `root`, `fusion`, `radius` (single-U curved run; plane null, cylinder surface). Grooved seams store `seam_<i>_grooveroot` in place of `_root`/`_gapmid`, which are square-prep only (D36) |
| `prepared_prism` (2026-09-07) | prepared_slab frame + prism outline (CCW, seam edge = vertices 0→1) | ±w caps, root/fusion/radius, `s1`…`s{k-1}` (s1 and s{k-1} are the notched end faces); `s0` is the prepared edge and is not a face |
| `prism` (6a) | u, v = outline plane; w = t | ±w (caps), `s0`…`s{k-1}` (s0 = vertices 0→1, the seam-bearing edge) |
| `swept_slab` | u = sweep parameter, v = width, w = thickness | ±u end caps, ±v "lateral swept", ±w broad offset surfaces. (Code differs: ±v are the top/bottom caps, and ±u are absent on closed spines; `geom.py:810`, `:878`) |
| `cylinder`/`tube` | w = axis | `lateral+` (outer), `lateral-` (inner, tube only), ±w (caps) |

- Face reference string: `"<object_id>:<face_name>"`, e.g. "A:+w". JSON pattern: `^[A-Z]:([+-][uvw]|lateral[+-]|s\d+|root|fusion|radius)$`.

**§2.2.1 Where diversity comes from.** D17 (chamfers, holes, slots, notches, stiffeners) was withdrawn 2026-08-15 as "decoration dressed as diversity". What the D11 split holds out:
- Phases 1–5: dimensions.
- Phase 6 (swept_slab/cylinder/tube): genuine geometry.
- Phase 9: "scanned MDF workpieces" (stale; now metal).

**§2.3 Scene face registry.**
- `faces[]` is flat and scene-wide; its index is the per-point `face_id`.
- Each entry: `{face_id, ref, object, name, plane:{n,d} (n·p + d = 0, n outward), area_mm2}`.
- It buys per-point face segmentation (a superset of the K-Net A/B/intersect masks), a stable face_id → ref map, and plane equations for D4 and the plane-pair methods.
- Non-planar faces: `plane: null`, `surface: {"kind":"cylinder","axis","point","radius_mm"}`. (Code keys: `point_mm`, `axis`, `radius_mm`, `outward`, plus a second kind, `offset_extrusion`, for swept ±w.)

**§2.4 `face_pair` ordering.** Two elements, sorted by (object id, face name) lexicographically, fixture "F" last, e.g. `["A:+w","B:-v"]`. String equality is meaningful.

**§2.5 `role`, and why the fixture forces it.**
- A plate standing on the fixture produces two exterior faces (A:−w, F:+w), a clean 90° dihedral and a bisector into free space. It passes D4 in full but must not be welded.
- So D4 gains a semantic precondition: both faces must be on `role == "workpiece"`, otherwise `reject_reason: "fixture_contact"`. MANUAL and README call this D13.
- "Not derivable from geometry": a purely geometric detector cannot tell it from a fillet. It is the most interesting class in the weldable-vs-interior confusion metric.
- Why the fixture is sampled, not always present (D12): the hard negative pays only against a fixture-free control (the Phase 4 pre-isolation plot). Arms are joined by `twin_key`.

**§2.5.1 `seam_class` (D22).** Derived from the forming faces, never declared. Broad = ±w; edge = ±u, ±v.
- Classes: `fillet` (face × face, 60–120°, T/corner inside), `butt` (face × face, ~180° coplanar), `lap_toe` (edge × face, ~90°), `edge` (edge × edge, ~180° coplanar).
- Allowed per joint type: T → fillet; corner → fillet, edge; butt → butt (short end runs are not seams); lap → lap_toe; edge → edge.
- Stale sentence: "Off-class seams … kept as `weldable: false` with `reject_reason: "wrong_class_for_joint"`."
- History it records: before the rule, an edge scene reported four edge welds (two were lap toes), and butt scenes emitted a rectangle of seams.
- Missing: the D31 table (edge also allows lap_toe when class_disjoint).

**§2.6 `reject_reason` vocabulary, "frozen".** Precedence = table order, top to bottom ("the applied order is recorded"; I found no field that records it; each seam carries one reason).

| Value | Meaning |
|---|---|
| `null` | weldable |
| `fixture_contact` | one or both faces belong to a fixture |
| `interior_face` | one or both faces have no exterior support (HPR / construction) |
| `bisector_blocked` | no admissible torch axis clears the nozzle cone (§2.6.1) |
| `degenerate_dihedral` | dihedral outside [dihedral_min_deg, dihedral_max_deg] |
| `no_contact` | faces separated by more than `contact_tol_mm` along the line |
| `too_short` | shorter than `min_seam_length_mm`, or running across the joint (§2.6.2) |
| `toe_of_centreline` | an intersecting line bounding a coplanar centreline's gap (the toe, not a seam) |
| ~~`wrong_class_for_joint`~~ | retired in 2.3.0, kept in the enum |

`confined_bore` (JSON enum, emitted by the code) is missing from this table.

**§2.6.1 The torch may tilt off the bisector.**
- Try the bisector, then larger tilts; accept the first axis whose whole nozzle cone clears at every seam sample. The accepted axis is what `approach_dir` stores and what defines joint +Z.
- Bounds: `max_work_angle_deg` = 45°; the axis must keep a positive component along both face normals. In a 30° nook the normals are 150° apart, so almost nothing is admissible; on a coplanar edge joint a hemisphere opens.
- Why: pinned to the bisector, the edge-joint bisector is horizontal and tangent to the table, and with D12's fixture on, **39 of 40 edge scenes lost their only seam**.

**§2.6.2 `too_short`: cross-runs are told by direction, not length.**
- A shorter candidate more than `cross_run_tol_deg` = 45° off the joint direction is demoted.
- The direction is the principal axis of candidate tangents, each weighted by its own length, over workpiece pairs only and over candidates still in the running.
- Each qualifier fixes a failure:
  - length-based: deep laps doubled the seam count;
  - longest single candidate: the real seam was deleted in 15 of 40 edge scenes;
  - including fixture pairs: a quarter of T fillets were demoted;
  - including rejected candidates: an edge weld was deleted.
- Not stated: since D31 the demotion is skipped for lap and edge when `class_disjoint` is on.

**§2.7 Seam ids.**
- SCHEMA's sort key: `(0 if weldable else 1, face_pair[0], face_pair[1], round(start_mm,6), round(end_mm,6))`, numbered 0…n−1.
- Ids are stable per scene but must not be used to match constructed vs rediscovered seams (use Hungarian assignment on Chamfer distance).
- Seam 0 defines the joint frame.
- **Code differs:** `(not primary, not weldable, face_pair[0], face_pair[1], round(p0,6))`, i.e. primary seams first and no end point (`accessibility.py:330`). Curved scenes use a weldable-first stable order (`scene_curved.py:351`).

### §3 Directory layout

**§3.1 One scene, `<scene_id>/`:**
- `scene.json`, `cloud.npz`, `seams.npz`, `scene.sha256` (one hex line).
- `mesh_<id>.ply`: convenience only, unhashed (`--emit-meshes` / `scripts/emit_meshes.py`).
- Tier 2 (Phase 8): `render.json` (render_id, draws, twin-gate report, view list) and `render.sha256`.
- `views/<k>/`: k = 0 is the tier-1 camera (the twin), 1..N−1 are drawn. Each view holds:

| File | Content |
|---|---|
| `rgb.png` | 8-bit; D10: stored, never benchmarked |
| `depth.png` | uint16, 0.05 mm/unit, 0 = no return, z along the optical axis |
| `depth_valid.png` | uint8 {0,255} |
| `mask_seam.png` | seam id + 1 for weldable seams; maskrule-0.1: 2 mm physical width, analytic ray-cast visibility, no blind zone |
| `mask_tack.png` | tack index + 1 (D38 intervals) |
| `mask_object.png` | object_id + 1 for workpieces, 255 fixture, 254 environment, 0 none |
| `view.json` | K, T_world_cam, width, height, view_kind, draw record, per-file hashes |

- Tier 2 never touches tier 1: those files are byte-identical, and `rgb`/`depth` in scene.json stay null.
- Meshes are not written per scene: parts are parametric and only "non-primitive geometry (Phase 6)" would write a `.ply` into `assets/parts/` keyed by `part_geometry_id`. No such library exists; every Phase 6 part is a parametric primitive.

**§3.2 A release, `release_v<X.Y.Z>/`:**
- `README.md`, `MANIFEST.json` (generator_version, schema_version, config set, scene count), `index.jsonl`, `splits.json`, `configs/`, `assets/parts/`, `scenes/<shard>/<scene_id>/` (shard = first 2 chars of scene_id, above ~10k scenes).
- `scene_id = "<config_id>-<seed:010d>"`; `config_id` = first 8 hex chars of sha256 of the canonical config.
- No release exists yet. Current corpora use `<corpus>/<joint_type>/<scene_id>/` with a per-type `index.jsonl`, plus `manifest.json` and, for bench_phase4, `facts.csv` (MANUAL §5).

### §4 `scene.json`

The prose example contains these top-level keys:

`schema_version` ("1.0.0" in the example), `generator_version` ("0.1.0"), `scene_id`, `config_id`, `seed`, `tier` (1), `twin_key`, `units {length:"mm", angle:"deg"}`, `joint`, `seam_definition` ("nominal"), `accessibility`, `objects`, `faces`, `fit`, `T_world_joint`, `seams`, `tacks`, `camera`, `noise_model`, `cloud`, `rgb` (null), `depth` (null), `provenance`.

`mps` is in the JSON but not in the prose example.

**joint** (example values):
- `type` (T | corner | butt | lap | edge), `seam_shape` (line | arc | spline | closed), `quality_level` ("C"; ISO 5817 level the sampled defects satisfy).
- `contact_mode` (free | flat | on_edge | propped | line; "free" ⟺ no fixture).
- `included_angle_deg` 90.0 (D18 design nominal, ISO 9692-1; not a defect, not dihedral).
- `in_plane_yaw_deg` 23.8 (D28; rotation of B about the contact-face normal, T and lap only, 0 elsewhere and in pre-6a corpora; support-bounded per scene).
- `stack_offset_mm` null (lap 0 < offset < L, edge 0, null otherwise).
- `prep` "square" ("square for all of Phases 1–6", stale).

**accessibility** (all recorded so every reject_reason is reproducible):
- `torch_clearance {half_angle_deg 30.0, standoff_mm 15.0}`
- `dihedral_min_deg 30.0`, `dihedral_max_deg 170.0`, `max_work_angle_deg 45.0`
- `contact_tol_mm 3.0` (tracks root gap and linear misalignment, capped by thickness)
- `coplanar_tol_mm 0.5` (measured at the seam)
- `cross_run_tol_deg 45.0`, `min_seam_length_mm 10.0`

**objects:**
- A: slab 200×120×8, `part_geometry_id` "slab_200.0x120.0x8.0", `mesh` null, `T_world_part` identity.
- B: slab 200×90×8.
- F: fixture, object_id 255, slab 600×400×10, "tilted ±10°, working surface NOT pinned to z = 0"; present in ~50% of scenes from Phase 2.

**faces:** `{face_id, ref, object, name, plane{n,d}, surface, area_mm2}`.

**fit:**
- `root_gap_mm` 1.1 (ISO 6520-1 no. 617)
- `linear_misalignment_mm` 0.4 (no. 5071, along w)
- `angular_misalignment_deg` 0.8 (no. 508, Annex B, see PARAMETERS §2.3)
- `throat_thickness_mm` 5.6 (aA, which the 617 limit depends on)

**seams[]** (prose fields):
- `id`, `weldable`, `reject_reason`, `face_pair`.
- `parametric {kind (line|arc|bspline in the example comment), p0_mm, p1_mm}`.
- `length_mm` 232.0.
- `dihedral_deg` 90.0 (the seam's dihedral; a lap has 0° included and a 90° seam dihedral, D18).
- `sampled {array:"seam_0", density_per_mm 10.0, n 2321}`.
- `matches_joint_type`, `occluded_fraction` 0.34 (another part; near-binary on straight seams), `in_frame_fraction` 0.71 (inside image and past the blind zone; the graded axis), `visible_fraction` 0.66 (= mean(seam_<i>_visible)).
- Second example: id 3, `fixture_contact`, face_pair ["A:−w","F:+w"], length 200, n 2001, matches false, occluded 0.12, in_frame 1.0, visible 0.88.
- The prose example omits `seam_class`, `underside`, `closed` and `clear_fraction`.

**tacks** (Phase 7, D38; null unless `emit_tacks`):
- `rule_version "tackrule-0.1"` (`weldgen/tacks.py`); a pure function of scene.json + seams.npz, computable retroactively.
- `params {k_max 33.0, d_abs_mm 400.0, k_min 10.0, tack_len_per_t 4.0, tack_len_min_mm 10.0, tack_len_max_mm 50.0, margin_per_t 2.0, t_mm 6.0 (min(t_A,t_B)), phase_by_seam {"0": 0.4173} (D39, closed loops only)}`.
- `points_mm [[24,0,0],[208,0,0]]`, `seam_id [0,0]`, `arclength_mm [24,208]`, `tack_length_mm [24,24]` (clip(4t, 10, 50)), `order [0,1]` (ends → centre → bisect; opposite pairs on loops; same-class seams interleaved round-robin).
- The example is not self-consistent: t_mm 6 against 8 mm plates, and a `phase_by_seam` entry for an open line.

**camera:** `model "pinhole"` (no distortion in tier 1), `K [[674,0,640],[0,674,360],[0,0,1]]`, 1280×720, `T_world_cam` (OpenCV), `standoff_mm 750.0`, `elevation_deg 55.0`.

**noise_model** (parameters only; the realisation is not stored): `kind "stereo_z2"`, `profile "d435i"` (D16: d435i | stereo_good | stereo_poor), `subpixel_px 0.08`, `baseline_mm 50.0`, `focal_px 674.0`, `min_z_mm 280.0` (blind zone is a sensor property), `lateral_sigma_px 0.8`, `grazing_dropout_deg 75.0`, `seed 8412337`.

**cloud:** `file "cloud.npz"`, `frame "world"`, `sampling_mode` (area_uniform | camera_raster), `density_per_mm2 1.0`, `n_points 184203`.

**provenance** (excluded from the hash): `created_utc "2026-08-13T14:31:07Z"`, `git_commit "9dd4212"`, python 3.12.3, numpy 2.3.5, trimesh 4.12.2, open3d 0.19.0.

**§4.1 Notes on specific fields:**
- `weldable: false` seams are kept (free hard negatives). Fixture scenes have at least one `fixture_contact`.
- `parametric` is truth; `sampled` is convenience ("10 dots per mm, or 50"). If they disagree, parametric wins.
- Curved seams (6b, D33):
  - `kind` extends to arc, circle, ellipse (plane∩cylinder), saddle (cylinder∩cylinder, branch-parametrised quadratic), composite (the tube-on-plate rounded rectangle) and bspline (constructed input only, never a fitted intersection). The JSON enum also has `offset`.
  - Parameters are written by `weldgen.curves.to_parametric` and rebuilt by `from_parametric`.
  - `closed: true` marks full intersection curves; `s` wraps at `length_mm`, and `s = 0` is the curve's own φ = 0. Endpoint rules (tacks, MPS) use the wrap convention. `length_mm` = total arclength.
  - `dihedral_deg` becomes `{min, max, mean}` (in code, only when it actually varies by more than 1e-6), with per-sample `seams.npz:seam_<i>_dihedral`.
  - Non-planar faces populate `surface`. `cloud.max_chord_error_mm` holds the worst tessellation chord error (D34, gated ≤ 0.25 mm; it never enters the label).
- `visible_from_cam` is geometry, not sensor yield: framed, in range, front-facing, unoccluded. Grazing dropout lives in `noise.apply`.
- A seam is not subject to the facing test. Using `approach_dir` as a surface normal returned `occluded_fraction: 1.0` for every seam in the first Phase 3 scenes.
- **primary** = weldable ∧ matches_joint_type, which is what supervision and the omission policy target. Report against primary, all weldable, or both; `index.jsonl` carries `n_weldable` and `n_primary`.
- Occluded vs in-frame are different physics. Occlusion is near-binary (a convex occluder spans the shared run); framing cuts partway.
- `occluded_fraction` is per seam.
- `throat_thickness_mm` is in `fit` because the 617 limit depends on it.
- `contact_mode`: "flat" = a face rests on the table (plates; base plate or strip of curved families); "line" (2.4.0) = a cylinder lies on it; "free" ⟺ no fixture; Phase 1 all free.
- `noise_model.profile`: `min_z_mm` lives here, so standoff is legal down to 200 mm for stereo_good and 400 mm for stereo_poor.

### §5 Binary arrays and release files

**§5.1 `cloud.npz`:**

| Array | dtype | Meaning |
|---|---|---|
| `xyz` | float32 (N,3) | exact, noiseless, world, mm |
| `normals` | float32 (N,3) | exact outward unit normals |
| `object_id` | uint8 (N,) | fixture = 255 |
| `face_id` | uint8 (N,) | index into faces[] |
| `visible_from_cam` | bool (N,) | Phase 3 ray-cast (D6), geometry only |
| `exterior` | bool (N,) | perfect multi-view scan reachability: True where some ray within the sensor grazing cone (70°) of the normal escapes every part; analytic, once; the Task 1 input condition (`full_exterior`) |

- The full cloud including interior faces is truth only, never a method input.
- Code detail: `exterior` is computed on a 2 mm voxel subsample with a fan at 0.55× and 0.95× of 70° (38.5°, 66.5°) plus the normal, and propagated by nearest neighbour (`visibility.py:419-511`).
- The noisy cloud is not stored: `xyz_noisy = weldgen.noise.apply(xyz, normals, T_world_cam, noise_model)`, deterministic in its seed (the D8 move: corruption as a citable convention; it halves release size). Exception: the frozen Zenodo release will additionally materialise `xyz_noisy`.
- `sampling_mode`: area_uniform is not what a depth camera returns; both modes have existed since day one.
- **D20** (pinned and implemented in Phase 3):
  - Under `camera_raster` the visible points come at raster density `(f_px/z)²` and then the D6 test; invisible points are the same draw's rejects, kept and flagged false.
  - Both classes come from one dense area-uniform pass, so they are rate-matched.
  - Density is not uniform across the mask, so `density_per_mm2` is nominal and density-sensitive metrics must be computed within a mask class.

**§5.2 `seams.npz`:** per seam, `seam_<i>` (float32 (M,3), nominal), `_root`, `_gapmid`, `_s` (arclength), `_tangent`, `_approach` (torch-ward), `_nA`, `_nB`, and `_visible` (bool, with the stale formula noted in flag 6). The arrays are regenerable from parametric + faces but stored so the file is self-contained. `_root`/`_gapmid` are identical to `seam_<i>` when g = 0.

**§5.3 `index.jsonl`:**
- One flat row per scene. Example fields: scene_id, config_id, seed, tier, twin_key, joint_type, seam_shape, quality_level, contact_mode, fixture_present, t_min_mm, t_max_mm, root_gap_mm, linear_misalignment_mm, angular_misalignment_deg, n_seams, n_weldable, n_fixture_contact, total_seam_length_mm, mean_occluded_fraction, n_points, density_per_mm2, sensor_profile, standoff_mm, elevation_deg, part_geometry_ids, content_hash.
- `df.groupby(["twin_key","fixture_present"])` is the fixture ablation; swap in `sensor_profile` for D16.
- `content_hash` lives here and in `scene.sha256`, not in scene.json.
- The writer also emits `emitted`, `included_angle_deg`, `in_plane_yaw_deg`, `n_primary`, `mean_in_frame_fraction`, `best_visible_fraction` and `sampling_mode`, plus skip rows `{emitted:false, seed, config_id, skip_reason, skip_detail}` (`weldgen/writer.py:59-131`).

**§5.4 `splits.json` (D11):**
- Hold out part geometries and joint configurations, never random frames.
- Policy "held_out_geometry_and_joint_config"; key `["part_geometry_id","joint_type","seam_shape"]`; train/val/test; `held_out_part_geometry_ids`; `held_out_joint_configs` (e.g. [["lap","arc"],["edge","spline"]]).
- `part_geometry_id`: primitives → `<primitive>_<dims>` (e.g. `slab_200.0x90.0x8.0`); Phase 6 asset mesh → `mesh_<hash>`. The code uses only parametric ids for all Phase 6 primitives.
- "Say what this split actually holds out": dimensions through Phase 5; genuine geometry from Phase 6 and Phase 9.
- **No `splits.json` exists anywhere and there is no split code.**

### §6 Reproducibility

**§6.1 Substreams.** `SeedSequence(seed).spawn(8)`, PCG64, fixed order:

| # | Name | Draws |
|---|---|---|
| 0 | joint_config | joint type, part dims, thicknesses (code adds included angle) |
| 1 | defects | root gap, linear/angular misalignment, quality level |
| 2 | seam_curve | seam shape, length, curvature/control points (yaw, outlines and grooves are appended here) |
| 3 | placement | assembly pose, fixture presence/pose/dims, contact_mode |
| 4 | surface_sample | point sampling, density |
| 5 | camera | sensor profile, pose, intrinsics jitter (approach-cone draws appended) |
| 6 | noise | depth-noise realisation |
| 7 | render | Phase 8, tier 2 only (unused in code, see flag 8) |

- 0–2 determine geometry and seam truth; 3–6 are the ablation axes. That is why `twin_key` is well-defined and why fixture presence lives in 3.
- Substreams and the draws within them are appended, never reordered. No `random`, no `np.random` global.

**§6.2 Content hash, not byte identity.**
- `np.savez` zips embed timestamps.
- `h.update(canonical_json(scene_without_provenance))`, then for each sorted array name: the name, `str(arr.dtype)`, `str(arr.shape)` and `tobytes()`. The code differs (flag 7).
- `canonical_json` = `json.dumps(sort_keys=True, separators=(",",":"), ensure_ascii=True, allow_nan=False)`.
- `provenance` is excluded.
- Phase 1 gate: `generate(config, seed)` twice, separate processes, different machines → identical hash.
- `render.sha256`: canonical resolved render config plus per-view digests of depth and masks. RGB is not bit-stable across GPUs; its hash is informational in `view.json`. `python -m weldgen verify-render --out <dir>`.

**§6.3 D21 invariants, asserted and not stored.** Each object watertight; each object winding-consistent; the union is not watertight and must not be (no boolean union is ever computed, D3). A violation fails the scene. This is trivial on slabs and bites in Phase 6.

**§6.4 `twin_key`.**
- `twin_key = sha256(canonical_json(config_geometry_part) + seed.to_bytes(8,"big"))[:16]`, over the config consumed by substreams 0–2 (`config.GEOMETRY_KEYS`).
- Twins share bit-identical geometry and seam truth; the ablation variable is read from `fixture_present`, `sensor_profile`, `tier` and `density_per_mm2`.
- Arms are emitted deliberately in pairs ("same pattern for sensor profiles", not implemented).
- Phase 9 warning: `twin_key` must not straddle a split.

### §7 Open items

| Item | Status as written | Actual |
|---|---|---|
| `camera_raster` | implemented in Phase 3 | ok |
| `surface` block | "reserved; fill in Phase 6" | filled in 6b |
| bspline encoding | "reserved, pin before Phase 6" | pinned (D33) |
| `tacks` | FILLED 2026-09-01 (`tackrule-0.1`, `emit_tacks`; `tack_length_mm` + `order` added as optional fields) | ok, but no patch bump |
| confirm the d435i profile against the real camera | open | gates only that profile's claim |

### `docs/scene.schema.json` (machine form)

- Draft 2020-12; `$id …/scene-2.2.0.json`; title "Weld seam dataset — scene"; description "Machine-checkable form of docs/SCHEMA.md v2.2.0. Prose wins on intent; this file wins on shape."
- `schema_version` enum `["2.3.0","2.4.0"]`. Top-level `additionalProperties: false`.
- **24 properties:** schema_version, generator_version (semver pattern), scene_id (`^[0-9a-f]{8}-\d{10}$`), config_id (8 hex), seed (int ≥ 0), tier (enum [1,2]), twin_key (16 hex), units, joint, seam_definition (const "nominal"), accessibility, objects, faces, fit, T_world_joint, seams, mps, tacks, camera, noise_model, cloud, rgb, depth, provenance.
- **20 required:** all except mps, tacks, rgb, depth.
- `$defs`: mat4, vec3, faceRef.

| Block | Properties (required ones marked *) |
|---|---|
| joint (additionalProperties false) | type*, seam_shape*, quality_level* {B, C, D, below_D}, contact_mode* {free, flat, on_edge, propped, line}, included_angle_deg* (0–180), stack_offset_mm*, prep* {square, single_bevel, single_V, single_U}, in_plane_yaw_deg (−180..180), ambiguous_with (D32), iso_17659_term, iso_9692_ref, seam_family (circle/ellipse/saddle/rounded_rect/swept_path/arc_butt), realization, groove (kind, groove_included_angle_deg, bevel_deg_per_side, root_face_mm, radius_mm, iso_ref) |
| accessibility | the 8 required fields; `torch_clearance.bore_min_diameter_mm` (6b, [ours]); `coplanar_gap_tol_mm` (D35) |
| objects (minItems 2, at most one fixture) | items require id, role, object_id (0–255), primitive {slab, prism, prepared_slab, prepared_prism, swept_slab, cylinder, tube}, dims_mm, thickness_mm, part_geometry_id (`^([a-z_]+_[0-9.x]+|mesh_[0-9a-f]{8,})$`), mesh, T_world_part; optional outline_uv, outline_shape, params |
| faces | all 7 fields required |
| fit | all 4 required |
| seams (additionalProperties false) | id*, weldable*, matches_joint_type*, reject_reason* (9 values + null, incl. `confined_bore`), face_pair*, parametric* (kind ∈ line, arc, circle, ellipse, saddle, bspline, composite, offset), length_mm*, dihedral_deg* (number or {min,max,mean}), sampled* {array `^seam_\d+$`, density_per_mm, n ≥ 2}; optional seam_class, occluded_fraction, in_frame_fraction, visible_fraction, underside (D31), closed, clear_fraction. Conditional: weldable ⇒ reject_reason null; not weldable ⇒ string |
| mps (D25/6c) | rule_version, params, seam_id, class |
| tacks | null or {rule_version, params, points_mm, seam_id, arclength_mm} required; tack_length_mm, order optional |
| camera | 7 required (model const pinhole); regime {uniform_sphere, approach_cone}, target_seam_id, target_face_pair, target_visible_fraction, approach_attempts, approach_cleared |
| noise_model | 9 required |
| cloud | 5 required; max_chord_error_mm |
| provenance | created_utc, git_commit required; open otherwise |

Root `allOf` (8 rules):
1. D12: contact_mode is "free" iff there is no fixture.
2. T/corner: included angle 60–120°, stack_offset null.
3. Butt: included angle const 180, stack_offset null.
4. Lap/edge: included angle const 0, stack_offset ≥ 0.
5. Edge: stack_offset const 0.
6. d435i: standoff ≥ 280.
7. stereo_good: standoff ≥ 200.
8. stereo_poor: standoff ≥ 400.

`docs/render.schema.json`:
- `render_schema_version` const "1.0.0"; `render_id` 8 hex.
- Required: scene_id, backend {name, version}, draws {alloy, surface_condition, substrate, dome, key_light}, config, views, twin_gate {pass, residual_p99_mm, coverage, objects}.
- Views require view and view_kind ∈ {tier1, drawn, undrawable}.

---

## 2. The notes

### 2.1 `notes/last_talk.md` (206 lines; committed 2026-08-13 in 9dd4212, never edited; no date inside)

**What it is.** A planning chat transcript between "cagdas" (the user) and Claude. It is not the advisor meeting; it relays advisor points from the handoff. It produced `dataset_plan.md`.

**Content and decisions:**
- Seam truth must not come from PCA: radius-PCA is an estimator and becomes the baseline. The handoff's advisor quote is "Ground truth'ün point cloud'dan gelmesi yanlış. Çünkü o ground truth değil, o estimation." (my translation: "Ground truth coming from the point cloud is wrong, because it is an estimate, not ground truth.").
- Invert the generator: sample the joint (type, dims, t, g, misalignment as EN ISO 5817 defect axes), draw the seam curve, then place A and B. Truth can be resampled at any density ("10 dots per mm, or 50").
- Edge/lap is a definition problem. A candidate is the clipped intersection of an exterior face pair; it is valid if the bisector `n_A + n_B` leaves without passing through either solid. This must be done on CAD, not on clouds.
- Most joints have several seams (T 2 fillets, lap 2 toes, edge 1, butt 1). Each gets a `weldable` flag and its face pair; negatives are kept.
- Store one cloud plus a `visible_from_cam` mask; fraction occluded is a difficulty axis. The per-scene manifest includes the RNG seed. Splits hold out part geometries and joint configs, not random frames (LWSNet's 94 raw clouds).
- Tacks are a convention: "endpoints mandatory, d_min ≤ Δs ≤ d_max (Tomków, JRM), no tack within ~2t of a free edge". Ship them as a versioned function `place(seam,t,g,rule_params)`.
- Two tiers: tier 1 = trimesh + NumPy, no simulator; tier 2 = Isaac Sim or BlenderProc as a pluggable backend ("BlenderProc is likely less painful"). Tier 1 noise: axial σ ∝ z², lateral blur, grazing dropout.
- Tier 1 is "measurement, not training data". The recipe is pretrain on tier 1, fine-tune on tier 2, validate on real; the ablation is a contribution.
- Two geometric baselines: (A) plane segmentation + pairwise intersection (Yi, PPF-plane) and (B) radius-PCA fixed. Wang's PCA slicing is cited as a third family. HPR (Katz; Open3D `remove_hidden_points`) is proposed for exteriority.
- Plan (Phase 0–9) with efforts:

| Phase | Plan | Effort / gate |
|---|---|---|
| 0 | `SCHEMA.md` + `PARAMETERS.md` | half a day |
| 1 | tier 1, T, straight seam | 3–4 days; gate "bit-identical file" |
| 2 | five types + rediscovery | 3–4 days |
| 3 | visibility | 2–3 days; gate: "occluded fraction spans 0 to ~0.8" |
| 4 | radius-PCA multi-seam + HPR gate, plane-segmentation baseline; Chamfer primary, Sinkhorn/EMD secondary | 1 week |
| 5 | hand-label 20–30 scenes in CloudCompare, second annotator if possible | 1–2 days |
| 6 | curved seams | 1 week |
| 7 | tack layer | 3–4 days |
| 8 | tier 2 | 1–2 weeks, high variance; gate: twin differs only in sensor realism |
| 9 | real MDF scans via ICP; GitHub + `make dataset` + Zenodo DOI | — |

- ISO 5817 numbers quoted:
  - h ≤ 0.25t+0.2 (D) / 0.15t+0.2 (C) / 0.1t+0.2 (B) for 0.5–3 mm;
  - above 3 mm: 0.25t max 5 / 0.15t max 4 / 0.1t max 3;
  - "β ≤ 2° at C, ≤ 1° at B";
  - fillet root gap h ≤ 0.3aA+1 (max 4) at D and ≤ 0.1aA+0.5 (max 2) at B for t > 3.
- Proposed new metrics: precision/recall over seam presence; weldable-vs-interior confusion.

**Superseded or corrected later:**
- β ≤ 2°/1° is a miscitation: Table 1 has no β limit. These are Annex B fatigue classes C63/B90; the [ours] mapping is D 4°, C 2°, B 1° (PARAMETERS §2.3).
- Byte-identity gate → content hash (SCHEMA §6.2).
- BlenderProc → Isaac Sim/Replicator.
- `trimesh.ray`/Open3D → closed-form analytic per-primitive casts (rtree banned by D9), later exact Möller–Trumbore.
- Window `gap + spacing < R < t` → `R < t/2`.
- HPR gate → stored analytic `exterior` flag; no method carries its own gate.
- Phase 3 gate → the 2.3.0 split. Occlusion is binary (40% = 0, 1.7% partial, 58% = 1); framing is the graded axis. D26 later declared a "polite" sampler correct for Task 2.
- Phase 6 "1 week" → 3–4 weeks.
- MDF scans → metal.
- Chamfer-primary → F1 at 3 mm plus Chamfer, RMSE/ME and EMD (MANUAL §1.4/§8.2).
- "Endpoints mandatory" → effective endpoints at the margin `max(2t, tack_len)`.
- Seam counts → measured and post-D31 different.
- `make dataset`: no Makefile exists.

### 2.2 `notes/thesis_direction_handoff.md` (182 lines; committed 2026-08-13 in 9dd4212, never edited)

**What it is.** A handoff "written to be pasted into a new chat". It reflects state before the dataset plan.

**§1 Setup:**
- Robotics MSc at METU, advisor Ahmet Buğra (ADEP project context); lab colleagues Umut Kurt and Ege Ecevit (CHERI/ROMER, in-house Isaac Sim experience).
- UR5e; eye-in-hand RGB-D camera off-axis from the flange Z; pen-holder tool (no torch).
- Workpieces: metal-painted 8 mm MDF on a steel plate with magnets; stainless sheet ~1–2 mm.
- Pipeline: RGB-D → SAM2 → PPF + FoundationPose → ICP-to-CAD → radius-PCA → admittance execution.
- Reference geometry: T-joint, ~232 mm seam, ~16 mm extracted band, ~1.1 mm gap, 8.4 mm plate. GitHub amnessa.

**§2 Settled decisions:**
1. No physical welding; pen marks on MDF give ~0.1 mm truth via flatbed scan.
2. No FEM: tack positions are FEM's input, not its output. Use cited inherent-deformation constants (Liang & Deng; Tian & Luo).
3. No DL-on-FEM surrogate (Wu/Wang/Kim EAAI 2022; Karimi & Jelovica 2025).
4. Seam extraction is a "dead contribution" (Yi et al. 2026); perception is infrastructure.
5. Never frame the work as an extraction-accuracy comparison: Mech-Eye NANO Ultra 0.07 mm @ 0.5 m and CSL 0.1 mm @ 0.5 m vs RealSense ~2.5–5 mm @ 1 m.

**§3 Current plan:**
- Advisor meeting (~3 h, last 10 min transcribed). He wants a dataset paper and floated three papers: (1) dataset (synthetic + real) + methodology + trained baseline model; (2) actual tack welding; (3) community benchmark. "Three papers is his enthusiasm, not an obligation."
- Core insight (his): the Turkish quote above. LWSNet annotated in CloudCompare; Yi et al. claimed 95% intra-rater consistency (precision, not accuracy); Lin used gluer teach-playback. Plane–plane intersection is exact and resampleable.
- The generator must be public ("sen aynı kodları koşacaksın", my translation: "you will run the same code").
- **Annotation experiment ("killer", ~1 day):** hand-annotate synthetic scenes the LWSNet way and measure against analytic truth. That gives the label-noise floor of the literature, compared with ~0.6 mm.
- Store / benchmark / regenerate:

| Data | Store | Benchmark | Regenerate |
|---|---|---|---|
| A/B clouds, analytic seam truth | yes | yes | — |
| tacks, joint type, camera pose | yes | yes | — |
| RGB, thickness/gap | yes | no | — |
| quality field, joint states, poses | no | no | yes |

- The advisor wanted RGB dropped; the decision was to deviate mildly and store it (ClearGrasp lesson), but not benchmark it (this became D10).
- Risk: straight-seam tacks are arithmetic. Include curved geometry, or scope paper 1 as seam-only.

**§4 Method contribution, "the thesis core":**
- Automated tack placement + ordering, validated with pen marks on the UR5e; baselines uniform spacing and geometry-only; venues CASE/ETFA/IROS. Then dataset (Isaac Sim + real scans), then model (RGB-D → tacks).
- Novelty statement: "discrete tack points … robot kinematic conditioning under an eye-in-hand visibility constraint". LWSNet §5.5.1 says "human tacks, robot welds".
- Stage 1, quality field over (s, φ):
  - nominal from Yi (`n₁×n₂`, `v₀=v₁+v₂`) and Wang (`Z_W=−(n_b+n_m)`);
  - IK feasibility (8 branches); σ_min(J), not |det J|; force transmission `1/√(nᵀ(J_vJ_vᵀ)n)`; joint margin; camera terms; collision (the camera body);
  - grid s @ 2 mm over 232 → 116, φ @ 5° → 72, ~8,400 poses;
  - DP with `μ|Δφ|` and a hard block on IK-branch changes.
- Stage 2, k tacks by exact DP `f(i,j)`, O(n²k) (n = 116, k ≤ 15, ~200k ops). Constraints: endpoints, d_min ≤ Δs ≤ d_max, ~2t margin, IK. `k_min = ⌈L/d_max⌉+1`. Tomków 2020: five tacks gave more angular distortion.
- Stage 3, ordering: joint-space travel time, IK-branch penalty, wrist-3 wrap, distortion priors.
- Model: PointNet++/LWSNet per-point head with `[x,y,z,n,q₁…q₅]`; U-Net heatmap + FiLM; not an autoencoder or transformer; "distillation" framing.

**§5 Literature numbers:**
- Yi 2026: 800 samples / 4 workpieces → 4800; ME 1.16, RMSE 0.64.
- LWSNet 2025: 94 → 658.
- Wang 2026: 750 → 1000 images, < 1 mm.
- Lin 2026: 13 workpieces, 1690 groups, 31.29 h, RTX 3080, RMSE 0.593/1.099/1.252, public page.
- SANet 2025: 4000+ images.
- Liu 2023 (RCIM 83:102549): NSGA-II.
- Distortion background: Tsai/Park/Cheng 1999 JRM; Park & An 2016; Satoh K = E·t/L; Okerblom/White k ≈ 160–200 N per J/mm; Tomków 2020 16.06% reduction.
- German PhD angle: TUM iwb, BAM, Fraunhofer IPK; DVS 1710, EN ISO 13920, EN 1090.

**§6 Next actions:** quality-field flatness test; φ range and blind zone; the occlusion bug ("don't hand-roll ray casting"); the annotation experiment; joint scope; the advisor's written no-welding scope. "synchron distance" in the transcript is probably Sinkhorn; Chamfer primary.

**§7 How to use it:** the recurring failure mode is reaching for a bigger contribution. "pazartesiye kadar gömül bunlara" (my translation: "immerse yourself in these until Monday").

**Superseded later:**
- The "thesis core = tack-placement method" framing. Later docs make the dataset + seven-method comparison the contribution, with tacks a shop-practice rule (tackrule-0.1: JASS 6 via the Kobelco handbook; EN 1011-2 unverified; Tomków anchor). The planned contribution became "an improvement to the winning method" (MANUAL §13, README).
- "Dataset = Isaac Sim generation" → tier 1 has no simulator (D9); Isaac only for tier 2.
- "Don't hand-roll ray casting" → it was hand-rolled analytically.
- MDF → metal (Phase 9).
- "Tacks: store and benchmark" → a versioned rule (D8).
- Chamfer primary → F1@3 mm primary.
- Paper 1's "trained baseline model": not done (README: no learned detector). The ICRA 2027 submission (2026-09-15) is on tier-1 results.
- The written no-welding scope is still open per README (2026-09-21). phase9_plan plans one out-of-view fixing tack per configuration as "fabrication, not the experiment".

### 2.3 `notes/plan_patch_v2.md` (239 lines; from the **2026-08-20 advisor meeting**; created 2026-08-20, last edit 2026-09-03)

**Purpose.** Task definitions, camera regimes and the method comparison. Decisions D25–D27.

**Task 1, complete seam recovery.** "Varsayalım ki sen bunu mükemmel taradın. Al olası tüm seam'leri." (my translation: "Suppose you scanned it perfectly; take all possible seams.") Given the full cloud, return every weldable seam with class and geometry. The label is exact and can be posed as per-point segmentation, instance grouping or curve regression.

**Task 2, most probable seam (MPS).** "Al, baktığın açıdaki en olası seam'i bul." (my translation: "find the most probable seam from the angle you are looking at"). Given one view, return one seam's class and location. Workflow assumption: "önce bir kaynak bölgesine git. Gittin mi kaynak seam'ini bul" (first go to the weld region, then find the seam). The MPS label is a convention (fourth after D8, D14 and D19).

**Declined from the meeting:** load-condition truth (out of scope; MPS is a geometric proxy); partial seams (out); "same seam, varying part sizes" (constrained by D27).

**D25.** `mps_rule-0.1` = argmax over weldable seams of visible arclength; ties broken by larger dihedral fold, then lower id; null if nothing exceeds `min_len_mm`. Advisor: "görünen yerdeki en büyük seam'i alıyoruz" (my translation: "we take the biggest seam in the visible area"). Visible arclength stated as "`1 − occluded_fraction` times `length_mm`". With the bimodal occlusion (58% of seams above 0.98 occluded, 40% at exactly 0), MPS is near-binary. Stored as an `mps` block `{rule_version, params, seam_id, class}`.

**D26.** Two regimes: `approach_cone` (deployment, Task 2) and `uniform_sphere` (stress test, Task 1). The "Swiss cheese slice" remark: sample from the notebook-02 azimuth×elevation map empirically. NoVisibleSeams mostly disappears; the paired delta measures what coarse positioning is worth; both regimes are twin-paired.

**D27.** Along-seam dimension pinned by the seam; across-seam free; thickness free within ISO 9692-1.

**Phase 4, seven methods.** `ours` is not claimed as novel.

| Method | Mechanism and source |
|---|---|
| `ours` | radius-PCA (the robot pipeline) |
| `lit-ransac` | Yi 2026 |
| `lit-ppf` | Sci. Rep. 2024 |
| `lit-regiongrow` | "Coarse-to-Fine Detection of Multiple Seams", δ = λ₀/Σλ |
| `lit-lobb` | Zhang T-ASE 2025; face/crease/boundary/corner |
| `lit-pcaslice` | intersecting-pipe paper |
| `lit-modelreg` | model-based identical workpieces |

- Coverage prediction: plane-intersection and orthogonal-pair methods cannot express butt or edge seams (the coplanar arm). The headline would be a mechanism × D4-arm coverage table.
- Protocol: 100 repeats with box plots (min and spread) for the randomised `lit-ransac` and `lit-ppf`; zero spread for deterministic methods as a finding. Noise sweep "0 / 5% / 10%" defined as multiples of σ_z, plus the three profiles. Oracle ladder L0 (xyz + object_id + exact normals) … L3 (L2 + noise + single view).
- Order of work: harness with a fake oracle first, then `ours` on `reference_tjoint`, regiongrow/lobb, ransac/ppf, pcaslice/modelreg last.

**Open items:**
- MPS as geometric proxy: CONFIRMED by the advisor (recorded 2026-09-01).
- Task 2 as a short section of paper 1: DECIDED 2026-09-01.
- Occlusion grading: RESOLVED 2026-09-01 by 6c(b). T, corner and saddle margins are structurally pinned (T → 0, corner → ~1, saddle → 0).
- lit-modelreg without CAD: RESOLVED 2026-08-21 (CAD from scene.json).
- `lit-nurbs` roster change: OPEN since 2026-09-01.

**Superseded later:**
- `ours` excluded from the Phase 4 run (ruling 2026-09-03).
- `lit-quadric` added 2026-09-08 as the seventh literature method; `lit-nurbs` is listed as an "eighth" open item (MANUAL §11).
- `lit-ppf` is deterministic as published (test_lit_ppf; MANUAL table), so only `lit-ransac` is randomised.
- 100 repeats → 30 seeds (deterministic methods get verify_seeds = 2).
- Noise 0/5/10% → 0/1/2× σ_z.
- Ladder L0–L3 → L0/L1 arms plus separate conditions (full_exterior/single, noise, fixture), a lit-ppf exact-normals rung and modelreg arms.
- D25's `1 − occluded_fraction` → implemented as `visible_fraction × length_mm` (`weldgen/mps.py`), with fold = |180° − dihedral| and the mean dihedral for curved seams.
- D26 is implemented as rejection sampling of (elevation, azimuth) until the target seam is ≥ 0.5 visible, up to 40 attempts, falling back to the best pose with `approach_cleared: false`.
- Coverage prediction refined: lit-ppf returns coplanar proxies with O(gap) error (README).

### 2.4 `notes/patch_phase6.md` (207 lines; drafted 2026-08-27 "after Phase 5 self-annotation returned 2,5 mm RMSE"; status 2026-09-01 "IMPLEMENTED in full")

**Purpose.** The Phase 6 split; decisions D28–D30.

**D28.** "The seam-bearing edge is constrained by the joint type; every other boundary edge must not be systematically parallel or perpendicular to the seam."
- T and lap: in-plane yaw (until then pinned to {0°, 90°}).
- Corner, butt and edge: non-rectangular outlines (rectangle, trapezoid, parallelogram, triangle, rhombus, general convex).
- To be recorded as an anti-shortcut measure.

**D29 (as drafted).** Seven workpiece configurations:
1. plate on plate (line; T, corner, lap)
2. pipe ⊥ plate (circle; T)
3. tilted pipe (ellipse; T)
4. pipe-to-pipe (saddle; T or butt)
5. rectangular tube on plate (rounded rectangle; T)
6. swept plate (arc/spline; T, lap)
7. two curved plates (arc; butt)

**D30.** Grooves on straight butts only.

**Phase 6a.**
- Checklist: yaw, outlines for all five types, D4 update, faceRef naming ("A:side_3"), D27, regenerate smoke, CI histogram gate.
- Gate: flat seam-to-boundary histogram. Then re-run Phase 4 (expect a ransac/ppf precision drop). Effort 3 days.

**Phase 6b.**
- Configuration-driven sampler; constructors; grooves (D24 restricted by D30) from a thickness table: ≤ 2 raised edges 1.1; ≤ 4 square 1.2.1; 3–10 single-bevel 1.9.1; 5–40 single-V 1.5, 2.2; > 12 single-U 1.6, 2.6. Scope narrowed to bevel/V/U; a groove makes a butt a single seam.
- Surface-pair verification; surface block; bspline pinned; D21 bites. Re-run Phase 4.
- Schema changes forced by curves: dihedral array or {min,max,mean}; tack wrap + `closed`; mps wrap; "D19 offsets already shipped as derived arrays, no change"; `length_mm` = total arclength.
- Effort 3–4 weeks. Risk: ship with straight seams and an outline split.

**Open items resolved:**
- Yaw: full circle uniform over the per-scene feasible set (2026-08-27).
- Outlines: convex only, non-uniform weights (2026-08-27).
- Saddle: D33 exact quadratic, ≤ 1e-6 mm (2026-08-28).
- ISO 9692-1 pipe preparation: confirmed absent (2026-08-28), so `iso_9692_ref: null` on curved scenes.

**Superseded later:**
- D29 amended 2026-08-28 to curve-first families with a realizability condition. Family 4 became T only and family 6 T only (dataset_plan D29).
- `A:side_3` naming → `s0…s{k-1}`.
- Outline vocabulary → trapezoid 0.10 / parallelogram 0.10 / triangle 0.25 / quad 0.30 / convex-pentagon 0.25 (PARAMETERS §3.3).
- Groove rows → implemented `valid_preps(t)`:
  - t ≤ 4: square
  - 4–8: square, single_bevel, single_V
  - 8–10: single_bevel, single_V
  - 10–12: single_V
  - > 12: single_V, single_U
  - refs: square 1.2.1/2.1, bevel 1.9.1, V 1.3 (t ≤ 10) / 1.5, U 1.6/2.6
  - so "single-V 1.5, 2.2" is superseded and raised edges are not a groove prep.
- "D19 offsets already shipped" is false for curved seams (no `_root`/`_gapmid`) and for grooved seams (`_grooveroot`).
- Grooved plates were rectangular until `PreparedPrism` (2026-09-07).

### 2.5 `notes/patch_class_disjointness.md` (199 lines; drafted and implemented 2026-08-27; ticks confirmed 2026-09-01)

**Purpose.** Joint classes disjoint by construction; lap seams from the contact polygon. Decisions D31 and D32.

**D31.**
- Clearance `c = 2·min(t_A, t_B)`. The draft's 0.10·min_edge term was dropped.
- Anchored in ISO 17659 (verified against the PDF): T-joint §3.10 "approximately right angles"; lap §3.9 "lie parallel … and overlap"; corner §3.13 "meet at their edges at an angle greater than 30°"; edge §3.14 0–30°. Leg length §3.21 z ≈ t, so 2t is the floor (the quantification is "ours"). Parallel joint §3.8 ↔ US "edge joint" in Annex A.
- T vs corner: reject a contact line near-parallel (within ~10°) to an edge of A and within c of it. Overhang is allowed; the line clips to the shared run.
- Lap vs edge: no boundary edge of B's yawed footprint may run near-parallel within c of an edge of A. This subsumes the three yaw-0 cases; laps cannot take yaw within ~10° of 180° (pinned by test).
- Corner needs no clearance.
- Edge scenes declare the union: exactly-flush pairs are edge seams, offset-over-face boundaries are lap toes, and both are in class (`ALLOWED_CLASSES["edge"]` gains lap_toe). Non-welded pairs must clear by c or route to D32.
- Consequence: edge metrics change by definition, so never compare pre- and post-patch edge numbers.

**D32.** The exclusion band is a new prior and is measured rather than hidden: an `ambiguous` stratum of ~5% of corpus size, sampled inside the bands, labelled `joint.type` + `joint.ambiguous_with`, excluded from D11, generated by a config flag that inverts acceptance (never an in-config probability, to keep bit compatibility).

**Lap seams from the contact polygon.**
- Upper-outline segments are weldable from above.
- Lower-outline segments are underside toes: `underside` flag, still weldable and in class, scored separately in single view, kept in full-exterior.
- Seam count becomes a property of the outlines (a triangle inside gives 3 toes).
- Code change: narrow `_drop_cross_runs` for lap and edge (butt keeps it; T end segments die to `min_seam_length`).

**Consequences.** Yield drops (lap and edge, mildly T); lap seam counts rise; edge truth grows; schema gains `ambiguous_with` + a stratum marker.

**ISO 17659 alignment.**
- The parallel joint (3.8) is excluded: its join is an area bond (explosive cladding).
- Edge keeps its name (α = 0° inside 3.14). Lap 3.9 is partial overlap ("en se recouvrant partiellement").
- Non-90° T = angle joint 3.12.
- Clause 5 is cited under D22/D27/D31.
- Excluded: cruciform 3.11, multiple 3.15, cross 3.16.
- ±10° T threshold is "ours". The "extends continuously" sentence is flagged as chat paraphrase.

**Open items, all ruled 2026-08-27:** `underside` and `ambiguous_with` DONE (optional in the schema); citation hygiene VERIFIED 2026-09-01.

**Later status.**
- Implemented as stated (`ALLOWED_CLASSES_DISJOINT`, `accessibility.py:326`).
- The D32 stratum configs exist (`amb_T.yaml`, `amb_lap.yaml`), but no amb corpus is evidenced in `out/` or in MANUAL §7.
- D31/D32 are plate-scoped (patch_phase6b).
- SCHEMA.md, PARAMETERS §2.7 and the README seam-count table were not updated.

### 2.6 `notes/phase9_plan.md` (138 lines; "written 2026-09-21", commit 82033e6)

**Purpose.** The real subset (Phase 9): purpose, how truth is constructed, inventory and purchases, the configuration matrix, the capture protocol, order of work. Generator ranges are measured on all 720 `bench_phase4` scenes so the real parts land inside the synthetic distribution.

**§0 Rules.**
- A test set, never training data: **11 strata × 10 configurations × 5 views = 550 frames**, strictly held out.
- Truth is constructed from measured poses. Parts are weldgen primitives with measured (caliper) dimensions. The lab pipeline (`admittance_control`: D435i → SAM2 → FoundationPose → ICP-to-CAD) registers `T_A`, `T_B`; scene.json is built and the seam comes from the same D4 rule. The scan is the "tier-3 twin" of tier 1 and tier 2. No annotator.
- Truth quality = pose accuracy, measured: ICP residual, repeatability (re-scan without touching), a fiducial-board or calibrated-jig bound on a subset. Stored as `pose_uncertainty_mm`; every real-subset number is reported with it.
- Depth is the deliverable; RGB is stored (D10) and not benchmarked. The real set is metal because active stereo drops out on specular steel. A flat-target session gives a `d435i_measured` profile (never edit `d435i`).
- Bare joints; parts fixed by a single tack out of view, magnets, or wedges + clamps. The tack is recorded (`fixture: {tack: true, where}`) as fabrication; the no-physical-welding scope stands.

**§1 Inventory (2026-09-21).**
- On hand: plates 1 and 2 mm high-carbon steel, 3 mm stainless; RHS 25×25, 20×40, 30×30, 30×50, 40×60 (wall unknown, likely ≤ 2.5).
- "Ordered 2026-09-21 (7 items)": RHS 60×60, 80×80, 50×100; pipes Ø42.4×2, Ø60.3×2, Ø76.1×2, Ø101.6×2, Ø114.3×2 (1 m each). That list has 8 items.
- Not ordered by decision: 6/8 mm plate (the lab cannot process it), so plates are 1/2/3 mm and there are no grooved butts.

Strata (12 rows; grooved excluded leaves 11):

| Stratum | Generator range on bench_phase4 | Real-set plan |
|---|---|---|
| T/line, corner, butt square, lap | t 1–12, L 80–600, W 50–250, α 60–120°, gap 0–5, polygon outlines | 1/2/3 mm stock + 4–6 laser-cut outlines |
| edge | t 1–2 | 1/2 mm |
| butt grooved | t 4–20; V 36 / bevel 16 / U 8 scenes; bevel 8–58° | EXCLUDED 2026-09-21 |
| T/circle | tube r_out 31–80 (Ø62–160), wall 3–8, length 80–220, plate 3–12 | ordered pipes, 2 mm wall = labelled extension. Ø60.3 has r_out 30.15, just below 31; not flagged in the plan |
| T/ellipse | same tubes cut 60–80° | mitre cuts 60/70/80 |
| T/saddle | branch r 15–80 on run r ≤ 80, α 60–89° | Ø42.4 & Ø60.3 on Ø101.6/Ø114.3; mostly 90°, two at ~70° |
| T/rounded_rect | footprint 64–194, R ≈ 15, wall 3–8, height 80–220 | RHS 60×60, 80×80, 50×100 |
| T/swept_path | band 4–10, height 50–156, arc/B-spline spine, plate 220–460 | 3 mm SS strips, R 150–300 + one S-curve (extension, or drop if flimsy) |
| butt arc | R ≈ 250, band 50–145, t 3–10 | 3 pairs R 200/250/300, widths 60/100/140 (+ one 2 mm pair) |

- Fit-up hardware: feeler gauges/shims 0.5–4 mm; angle wedges 60/75/90/105/120°; magnets; clamps; an AprilTag board glued at a known offset.
- Thickness axis: the real set covers the thin third of 1–12 mm. A per-stratum in-range/extension flag is stored; a `bench_real_ranges` stratum is "a config file away" (it does not exist).

**§2 Reuse rule.**
- Worth is lost only when one geometry stands for a whole stratum, which would make the D11 held-out-geometry split degenerate.
- Per stratum: ≥ 3 distinct geometries, ≥ 2 thicknesses where possible, ≥ 1 non-rectangular outline (plates). 10 configurations = 3 geometries × 3 fit-ups + 1.
- Fit-up vocabulary mirrors B/C/D/below-D: gap {0, ~1, ~3}; h {0, ~0.15t, ~0.3t}; β {0, ~1°}; T/corner α {90, 75, 110}; lap overlap two values; edge flush vs offset. "Measure what was set, store what was measured."

**§3 The 110 configurations.**

| Stratum | Parts | Fit-ups → 10 |
|---|---|---|
| T/line | 2 on 2; 3 SS on 3 SS; 2 on 3 (one trapezoid) | α 90/75/110 × gap 0/1/3 + 1 with misalignment |
| corner | 2/2, 3/3, 2/3 (one pentagon) | α 60/90/120 × gap + 1 (differs from the §2 α vocabulary) |
| butt square | 1/1, 2/2, 3/3 | gap 0/1/3, h 0/0.3t, + dissimilar |
| butt arc | R pairs | gap 0/1/2 + 1 |
| lap | 1/1, 2/2, 2/3 | overlap × gap 0/1, yaw, + 1 |
| edge | 1/1, 2/2, 1/2 | flush/offset × gap 0/1/3 + 1 |
| T/circle | 4 pipes × 2 bases | + 2 with 1 mm gap |
| T/ellipse | 3 pipes × 60/70/80 | + 1 |
| T/saddle | 4 pairs at 90° × 2 fit-ups | + 2 at ~70° |
| T/rounded_rect | 3 RHS | 3 × 3 + 1 |
| T/swept_path | R150 / R300 / S strips | 3 × 3 + 1 |

- Views: 5 per configuration from the tier-1 camera sampler seeded per configuration (standoff 300–1200, elevation 15–85° incl. one grazing view, azimuth uniform, aim jitter 0.15 × span), executed as robot targets. `K` and `T_world_cam` come from eye-in-hand kinematics + extrinsic. 110 × 5 = 550.

**§4 Pipeline (none built yet; no files found).**
1. `scripts/label_real_scan.py`: measured dims + poses → scene.json/seams.npz under the schema, a `real_v1` config, D4 as-is, `provenance.real = {icp_residual_mm, repeatability_mm, fiducial_bound_mm, fixture_tack}`, `cloud.npz` = the scan (xyz + object_id) with `tier: 3`; tested on a fake scan.
2. `scripts/plan_real_views.py`.
3. Fiducial and pose-error script.
4. `d435i_measured` at three ranges.
5. Pilot: 3 configurations (plate, pipe-on-plate, arc butt) end to end through registration, the twin gate against tier 2, and the seven methods.
6. Capture ≈ 15 min/configuration → ~30 lab hours over ~2 weeks, then `notebooks/17_real_subset.ipynb`.

**§5 Stated exclusions.** Grooved butts (no ≥ 6 mm plate; no ISO 9692-1 bevel row below it); plates > 3 mm; angled saddles beyond two. Extensions (2 mm walls, 3 mm band, 2 mm bases) are flagged per scene and named in the release notes.

**Splits and release.** The plan mentions D11 only in §2 and release notes in §1/§5. dataset_plan Phase 9 adds: splits by held-out part geometry and joint configuration with the scope stated; splits by `twin_key`, never by view (Phase 8 section); GitHub with all seeds; `make dataset`; Zenodo DOI; README with schema, ranges and baselines.

**Inconsistencies and implications:**
- "120-configuration matrix" (preamble) vs 110 (body); README says 12 strata.
- "(7 items)" vs 8 listed.
- §2 α {90, 75, 110} vs corner row 60/90/120.
- `fixture: {tack, where}` (§0) vs `provenance.real.fixture_tack` (§4); `pose_uncertainty_mm` (§0) vs `provenance.real.pose_uncertainty_mm` (dataset_plan). Provenance is excluded from the content hash, so truth-quality metadata would be unhashed.
- `tier: 3` and `d435i_measured` need schema enum changes (tier enum [1,2]; profile enum of 3), i.e. minor bumps.
- A tier-3 `cloud.npz` cannot carry exact normals or face ids, so the array semantics change.
- The measured L 80–600 and gap 0–5 exceed PARAMETERS §3 nominal (L 80–400; fillet gap ≤ 2 + tail 3). The gap side is explained by the below_D draw (up to max(1.5 × 617 D limit, 3), which can reach 6 mm). The L side is not verified (likely outline bounding dims and curved parts).

---

## 3. Configs (29 YAML; keys override `weldgen/config.py:DEFAULT_CONFIG`)

**Defaults for reference:**
- all five joint types; line; square; L 80–400; W 50–250; t 1–12; dissimilar_thickness_p 0.30; α 60–120; stack_offset_frac 0.15–0.60; edge t ≤ 2; stacked β ≤ 0.4; edge_equal_width_p 0.35;
- quality_mix 25% each of B/C/D/below_D; gap 0–2 + over-range 3; fixture off (tilt 10°, height ±50 mm, dims 480–720 × 320–480 × 8–12);
- density 0.25–4 pts/mm²; one profile drawn per scene from all 3; standoff 300–1200; elevation 15–85; roll ±15; framing_frac 0.35–1.45; 1280×720; omission policy on (min_visible_fraction 0.1); aim jitter 0.15; area_uniform;
- no camera_regime (implicitly uniform_sphere); no emit_mps or emit_tacks;
- yaw, outlines, class_disjoint, class_boundary_stratum and groove_preps all off; seam_families None.

`n` is a CLI attempt count, not a config key; sizes below come from comments and docs.

| File | Purpose | Key settings | Builds / used by |
|---|---|---|---|
| `phase1.yaml` | Phase 1 working config | T, line, square, fixture off | `out/phase1` (CLI default --n 8); schema-conformance preset |
| `phase2.yaml` | Phase 2, all five types, D12 off-arm | 5 types, fixture false, `require_visible_seam: false` | `out/phase2` (README --n 40) |
| `phase2_fixture.yaml` | D12 on-arm | as phase2, fixture true | `out/phase2_fixture` |
| `phase3.yaml` | visibility layer live | 5 types, fixture false, area_uniform, 3 profiles | `out/phase3` (used by several tests) |
| `phase3_raster.yaml` | D20 camera-like condition | as phase3 + `camera_raster` | `out/phase3_raster` |
| `reference_tjoint.yaml` | regression fixture (admittance README §8) | T; L 232 fixed; overlap 1.0; W 90–120; t 8.4; dissimilar 0; α 90; gap 1.1; h = β = 0; quality C; density 0.14; d435i only; omission off | tests only |
| `smoke.yaml` | "8 fast scenes for CI and the determinism gate" | T; L 100–240; W 60–120; density 0.05–0.15; yaw, outlines (no-op), class_disjoint on; omission off | `out/smoke`; tests |
| `bench_{T,butt,corner,edge,lap}.yaml` (5) | balanced per-type benchmark, "50 emitted scenes each" | one type; fixture false; area_uniform; 3 profiles | `out/bench` (250) |
| `bench_*_fx.yaml` (5) | fixture twin of each bench config | identical except fixture true; same `name` as partner | `out/bench_fx`; fixture plot 7 |
| `bench6a_*.yaml` (5) | Phase 6a plate corpus | as bench + yaw, outlines, class_disjoint (D28, D31) | `out/bench6a` (Phase 5 annotation source); the plate-line sources of `make_bench6b.py` for `out/bench6b` (60/class), `out/bench_phase4` (60/family = 720) and `train_v1` (300/family, base seed 3,000,000; benchmark base seed 2,000,000), plus `_fx` |
| `amb_T.yaml` | D32 T/corner border stratum | T; W 100–130; linear_misalignment 38–62 mm; class_disjoint + class_boundary_stratum; outlines; yaw off | no corpus or test usage found |
| `amb_lap.yaml` | D32 lap/edge border | lap; stack_offset_frac 0.01–0.12; class_disjoint, class_boundary_stratum, yaw, outlines | used by test_class_disjointness; no corpus found |
| `curved_smoke.yaml` | Phase 6b smoke, one scene per family 2–7 | seam_families [2..7]; fixture false; area_uniform; 3 profiles | `out/curved_smoke` (README --n 14); base config for every curved stratum in `make_bench6b.py` (family overridden), hence the curved strata of bench6b, bench_phase4, train_v1 and their twins |
| `grooved_butt.yaml` | Phase 6b grooved butts (D35) | butt; t 3–20; groove_preps; class_disjoint; yaw + outlines (since 2026-09-07; now `prepared_prism`, every scene re-identified) | `line_grooved` stratum (square draws filtered) |
| `playground.yaml` | notebooks 01/02 scratch preset | lap; L 40–400; W 90–120; t 3–20; yaw, outlines, class_disjoint; groove_preps; **`seam_families: [ 4]` active** (the comment says "uncomment to switch"); α 60–120; gap 0–2; quality C; density 0.14; d435i; h = β = 0 | notebooks only |
| `task2_cone.yaml` | Phase 6c Task-2 deployment (D26) | T; yaw, outlines, class_disjoint; `camera_regime: approach_cone` (defaults 0.5 visibility, 40 attempts shown commented); `emit_mps: true` | test_approach_cone; the bench_phase4 approach_cone twin arm is still open (MANUAL §11) |
| `render/lab_v1.yaml` | tier-2 render config (Phase 8) | see below | `train_v1` (3600 scenes × 10 = 36,000 frames) and the tier-2 gate on 12/12 benchmark strata |

`render/lab_v1.yaml` settings:
- render_config_version "1.0"; backend isaac_replicator; views 10; rt_subframes 16; depth scale 0.05 mm.
- masks: maskrule-0.1, seam 2.0 mm, tack 3.0 mm, occlusion analytic_ray_cast.
- materials-1.1: roughness jitter 0.85–1.15; uv rotation 0–360; 7 alloys (mild_steel, stainless_steel, aluminium, cast_iron, bronze, brass, titanium); 5 conditions (mill_scale, ground, rusted, primed, oily); dissimilar_probability 0.0.
- environment: backgrounds, panoramas and asset manifests (paths excluded from render_id; set hashes included); use_lab_panoramas; plane_half_m 3.0; span_jitter 0.3; roughness 0.45–0.85.
- lighting: dome_base 300; target mean luminance 0.75; dome_base_ldr 900; exposure 0.6–1.5; rotation 0–360; key light intensity 100–600, elevation 30–70, azimuth 0–360.
- drawn views: elevation 25–70; framing 0.5–1.0; aim jitter 0.35; roll ±10; standoff 300–1200; min visible fraction 0.1; min_seam_px 100; max_attempts 40.

Notes on the configs:
- No config sets `emit_tacks`. bench_phase4's mps + tacks were retro-applied by `scripts/apply_rule_blocks.py`.
- No config pins a single sensor profile for paired arms.
- The README configs list omits `task2_cone.yaml` and the `_fx` files.
- PARAMETERS §6's preset table names `default.yaml`, `thin_sheet.yaml`, `lab_fixture.yaml`, `annotation_study.yaml`, `fixture_ablation.yaml` and `sensor_ablation.yaml`. **None of them exist.**

---

## 4. Tests

Static count: `grep -c '^def test_\|^    def test_'`. Collected items come from expanding `@pytest.mark.parametrize` by hand (pytest not run). There are no parametrized fixtures and no pytest config file.

| File | Funcs (collected) | What it pins |
|---|---|---|
| `schema_contract.py` | 0 | standalone script for the 2.2.0-era schema contract; not collected; stale |
| `test_accessibility_registered_pose.py` | 2 | T fillets survive arbitrary floating-point rigid poses (registered-pose regression, 2026-09-21) |
| `test_annotation.py` | 3 | Phase 5 tools: export ships no truth; a fake annotator with 0.8 mm σ, a phantom and a miss is recovered; demo role excluded |
| `test_approach_cone.py` | 3 | D26 regime is a free twin, draws appended to substream 5, edge/lap gate, curved pipeline carries it |
| `test_baselines.py` | 20 | `ours` migration: mm units, validity window R < t/2, cross-object oracle, multi-seam, metric definitions, directional clustering |
| `test_class_disjointness.py` | 13 | D31/D32: c = 2·min(t), bands, yaw 180 rejected, full circle, ISO fields, stratum inversion, edge union, bit-identity, underside, A-side toe |
| `test_constructors.py` | 13 (19) | Tube watertight in every cut mode, exact saddle/miter ring, samples on true surfaces, constructors carry the step-1 curve, gap without penetration, swept chord gate, configs 5/6/7 |
| `test_curves.py` | 16 | D33 exact curves on their surfaces, tangents, arclength sampling, closed wrap, JSON round-trip, D29 sampler |
| `test_determinism.py` | 9 (11) | Phase 1 gate (D15): repeatability, separate processes, provenance excluded, geometry/dtype sensitivity, npz round-trip, substream isolation, twin_key, byte order |
| `test_from_object.py` | 11 | `geom.from_object` inverts the serialisers for all six primitives and three pipelines |
| `test_geometry.py` | 9 (18) | T closed forms: D19 offsets 0/g/2/g at 90° (g ∈ {0, 0.5, 1.1, 3.0}), supplementary dihedrals, y = h + g·tanθ, watertight objects but not the union |
| `test_grooves.py` | 11 (17) | D35–D37: valid_preps rows, exact volumes, mouth centreline weldable, groove_root replaces the D19 triple, single-bevel asymmetry, flag-off bit-identity |
| `test_harness.py` | 7 | fake-oracle validation of the harness: jitter, phantom, miss, seeds, zero spread, noise multiplier, EMD vs Chamfer |
| `test_joint_types.py` | 25 (65; 4 xfail) | Phase 2 rediscovery gate across five types: role-based fixture_contact, contact_tol ≤ t, β robustness, misalignment hinge, the 39/40 edge regression. `test_no_seams_wrap_around_to_the_far_face` is a non-strict xfail × 4: residual ~2–6 phantoms per 40 tilted laps |
| `test_lit_lobb.py` | 16 | Zhang 2025 fidelity: weld-width radius, box flatness ≠ eigenvalue ratio, coarse stage = cross-object gate, reproduces corner accuracy |
| `test_lit_modelreg.py` | 8 | Fang & Tian 2024: model = scene geometry, CPD, constitutively L0, curved models |
| `test_lit_pcaslice.py` | 12 | Wang 2026: data-driven slicing, two-seam failure, closed rings per arc, eq. 12 signs |
| `test_lit_ppf.py` | 16 | Wang 2024 Sci. Rep.: descriptor, vote, deterministic as published, exact-normals arm |
| `test_lit_quadric.py` | 21 (23) | Li, Wang & Wang 2026 JMST: exact intersections, measured-point default output, ordering, far-side mirror |
| `test_lit_ransac.py` | 25 | Yi 2026: constants in mm, eqs. 18/19/24, coverage limit on parallel faces, accuracy reproduction, randomised seeds |
| `test_lit_regiongrow.py` | 16 | Wei 2024: eq. 3 = surface_variation, kNN vs ball, keypoint crop, curvature gate |
| `test_mps.py` | 5 | mps_rule-0.1 order, null below min length, curved mean dihedral, flag-gated emission |
| `test_parameters.py` | 8 (25) | ISO 5071 (10 cases) and 617 (9 cases) tables; angular D 4°/C 2°/B 1° convention; worst-defect quality |
| `test_phase6a.py` | 7 | D28 yaw: rediscovery, yaw ≠ dihedral, support bound, bit-identity, twin, gate moves off 0/90° |
| `test_phase6a_outlines.py` | 12 (16) | Prism parity with Slab, watertight, rediscovery on 5 types, convex vocabulary, bit-identity |
| `test_prepared_prism.py` | 8 (32) | PreparedPrism watertight with exact volume (3 preps × 5 outlines), rectangle = PreparedSlab, groove_root kept, D28 gate reads the outline |
| `test_qa_gate_curved.py` | 4 | D28 curved arm (provenance signature, families 6/7, closed seams exempt) and the D34 sweep |
| `test_scene_curved.py` | 6 (13) | curved scenes schema-valid (families 2–7), reproducible, parametric regenerates arrays, dihedral summary |
| `test_scene_curved_fixture.py` | 3 (4) | 2.4.0 curved fixture twins: flat (family 7) / line (family 4), deterministic, fixture-off reproduces the benchmark (2026-10-19) |
| `test_schema_conformance.py` | 7 (14) | every emitted scene validates (6 seeds; 3 presets), nominal definition, reference T, face registry, unit outward normals |
| `test_self_footprint.py` | 3 | the `Self_ideas/test1_ismail_hoca` extractor through the harness (2026-10-06) |
| `test_self_seamfind.py` | 6 | `Self_ideas/test2_cagdas` seam finder stages (MLS < 0.1 h, gap error < 0.25 mm at 1 mm, pipe ring) (2026-10-06) |
| `test_tacks.py` | 8 | tackrule-0.1/D39: bounds, 400 mm ceiling, one-tack collapse, closed loops, stagger, flag gating |
| `test_task2.py` | 4 | `run_task2`: MPS matching, empty prediction, rotation-invariant rings, band output refused |
| `test_tier2_gate.py` | 9 | twin gate on analytic depth; the Isaac test uses `importorskip("isaacsim")` |
| `test_tier2_sensor.py` | 3 | D16 on rendered depth: validity, seeded σ_z, hashed |
| `test_tier2_writer.py` | 15 | render_id, draws, masks, writer round-trip, render hash, materials-1.1 |
| `test_verify_curved.py` | 10 (16) | curved D4 arms: rediscovery to machine precision, perturbation detection, offsets, bore gate |
| `test_visibility.py` | 32 (39) | Phase 3 gate: ray cases, σ_z table (0.21/0.59/2.37/9.50 mm at 300/500/1000/2000 mm), raster density, fractions, omission, exterior |
| `test_viz.py` | 9 (33) | assembly frame undoes the world pose (4 seeds × 8) |
| `test_writer.py` | 2 | index merge, no duplicate rows |

**Totals:**
- **417 functions in 40 test files → 579 collected items.** The 15 parametrized files expand from 183 functions to 345 items.
- README "564 tests" (2026-09-21 12:47 UTC) + 2 (`test_accessibility_registered_pose`, 2026-09-21 15:52) + 3 + 6 (`test_self_*`, 2026-10-06) + 4 (`test_scene_curved_fixture`, 2026-10-19) = 579. This matches exactly.
- MANUAL "460 tests" is from 2026-09-08.
- Many tests `skip` when the corpora are absent (`out/bench`, `out/phase3`, `out/bench_phase4`). The laptop `out/` is stale, per README.

---

## 5. requirements.txt, .gitignore, CLI

**`requirements.txt`:**
- Content: `numpy>=1.24`, `trimesh>=4.0`, `PyYAML>=6.0` (tier 1, D9: "no simulator, no GPU, no ROS"); `jsonschema>=4.20`, `pytest>=7.0`; `matplotlib>=3.7`; `scipy>=1.10` (HPR hull; already a trimesh dependency); `# open3d>=0.18` commented out.
- Its comment "every part is a box … ray-box exact in closed form; trimesh.ray would pull rtree, which D9 rules out" is stale since Phase 6.
- Missing: pandas (`scripts/baselines/harness.py:428,480`, `run_phase4_batch.py`, annotation scoring; tests test_harness, test_task2, the lit-* harness tests, test_annotation, test_self_*); scikit-learn (`radius_pca.py:312`, DBSCAN); Pillow (`weldgen/render/writer.py:24`, transitive via matplotlib); open3d (viewers and Self_ideas only); Isaac Sim (tier 2, by design).
- No version pins. README lists numpy pinning for the release as open.

**`.gitignore`:** `out`, `out/`, `out_nvme_old/`, `out_nvme_old`, `__pycache__/`, `*.pyc`, `.ipynb_checkpoints/`, `.pytest_cache/`. Corpora are not versioned; the authoritative data is on the desktop render disk (README).

**CLI** (`weldgen/cli.py`; `python -m weldgen` runs `cli.main`):
- `generate --config PATH (default None = DEFAULT_CONFIG) --n INT (8) --seed0 INT (8412337) --out DIR (out/phase1) [--emit-meshes] [--quiet]`.
  - Seeds `seed0 … seed0+n−1`; `n` is attempts, not scenes.
  - Routes to `generate_curved_scene` when the config has `seam_families`.
  - A `SceneRejected` becomes a skip row (`emitted:false`, `skip_reason` = exception class, `skip_detail`). No backfill, to keep twins attempting the same seeds.
  - Prints the yield; `write_index` merges rows by (config_id, seed).
- `verify --out DIR (out/phase1)`: re-hashes immediate subdirectories containing scene.json against `scene.sha256`; exit 1 on mismatch; not recursive.
- `verify-render --out DIR` (required): recursive over `render.sha256`, calling `render.writer.verify_render`.

---

## 6. Discrepancy register (beyond §0)

### A. SCHEMA.md internal

1. Header 2.3.0 vs §0 2.4.0.
2. §2.5.1 retired-reason text (flag 3).
3. §5.2 `occluded_fraction` formula (flag 6).
4. §2.3 surface uses `point`; §4.1 uses `point_mm`; neither lists `outward` or the `offset_extrusion` kind.
5. §4 example is stale (`schema_version` "1.0.0"; tacks `t_mm` 6 against 8 mm plates; `phase_by_seam` on an open seam; no `seam_class`).
6. §4 `prep` "square for all of Phases 1–6" and §7 bspline/surface rows are stale.
7. §2.1 primitive list omits prepared_slab/prepared_prism.
8. "Fixture presence sampled ~50/50" (§2.1, §2.5, §4) vs §6.4 "emitted deliberately in pairs" (a config axis in practice).
9. §1.1 "fixture tilted ±10°" does not hold for curved fixture twins (z-preserving pose, no tilt; `make_fixture_twins.py`, patch_phase6b).
10. §2.6 "applied order is recorded": no field records it.

### B. SCHEMA.md vs scene.schema.json and code

1. JSON `$id`/description 2.2.0.
2. Added enums and fields not in the prose: `confined_bore`; kind `offset`; joint `ambiguous_with`, `iso_17659_term`, `iso_9692_ref`, `seam_family`, `realization`, `groove`; seams `underside`, `closed`, `clear_fraction`; `mps`; camera regime and target fields; `bore_min_diameter_mm`; `coplanar_gap_tol_mm`; `objects[].params`.
3. `cylinder` primitive is in the enum and prose but never implemented. `geom.PRIMITIVES` = slab, prism, prepared_slab, prepared_prism, tube, swept_slab. The rounded-rect "tube" is a closed-spine SweptSlab.
4. Phase 6 `part_geometry_id` is parametric (`prism_<shape>_<k>x…`, `tube_<D>x<wall>x<len>`, `swept_<kind>_<len>x<t>x<H>`, `prepared_<kind>_…`, `prepared_prism_<kind>_…`), not `mesh_<hash>`; no `assets/parts/`.
5. Swept-slab face semantics (§2.2) differ from code (±v are caps; ±u absent on closed spines).
6. Seam-id sort key (flag 10 of §0).
7. Content hash (flag 7).
8. Substream 7 (flag 8).
9. Sensor twins (flag 9).
10. Exterior 70° vs dropout 75°, while the code docstring claims "the same quantity".
11. index.jsonl and splits (§1 §5.3/§5.4).
12. Curved seams have no D19 arrays.

### C. SCHEMA.md vs MANUAL / README / PARAMETERS

1. Versions: README "v2.2.0"; PARAMETERS 2.2.0 "tracks SCHEMA.md".
2. MANUAL §6 "verify re-hashes everything" vs non-recursive verify.
3. README/MANUAL say six primitives and SCHEMA lists `cylinder`.
4. SCHEMA/PARAMETERS "Phase 9 = MDF" vs MANUAL/README "metal".
5. PARAMETERS `contact_mode` "flat 0.6 / on_edge 0.3 / propped 0.1" vs SCHEMA §4.1 and code (only flat and line are ever emitted).
6. PARAMETERS §2.7 and README seam-count table vs D31 (and §2.5.1 likewise).
7. SCHEMA §6.4 / PARAMETERS §4.0 sensor twins vs README "Fixture presence is a config axis" (fixture paired, sensor not).
8. PARAMETERS §3/§3.2 "swept_slab / cylinder / tube".
9. README Phase 8 "on substream 7".
10. MANUAL §1.4/§4 name the Task 2 condition `single`; README says `single_view`.

### D. MANUAL / README / PARAMETERS among themselves

1. Six vs seven methods (flag 15).
2. 460 vs 564 vs 579 tests.
3. Phase 9 11 vs 12 strata.
4. PARAMETERS root-gap scaling vs code (flag 10); README matches the code ("draws against the 617 limit … 9692-1 cap as a ceiling").
5. PARAMETERS §5 `R < t` vs README `R < t/2`.
6. PARAMETERS stale sections: §2.6.1 "deferred" table (lists double-V/double-bevel/single-J, never implemented); §5.0 "all Phases 1–6 square"; §5.1 "camera_raster … reserved slot"; §7 items for camera_raster, non-square preps, `lab_fixture.yaml`, tack constants.
7. PARAMETERS §6 nonexistent presets.
8. Groove rows differ between patch_phase6, PARAMETERS §2.6.1 and the implementation/README (use the implemented rows in §2.4).
9. README Phase 6 paragraph "the prepared plate is a rectangle by construction" contradicts its own 2026-09-07 PreparedPrism paragraph and MANUAL §3.2.
10. README "every part is a box" (and requirements.txt) is stale.
11. README layout "make_fixture_twins.py … plate strata" is stale (all 12 strata since 2026-10-19; `bench_phase4_fx` = 634 = 274 plate + 360 curved).
12. MANUAL §2/§4 "D31 makes the four seam classes disjoint": D31 concerns joint types; seam class comes from faces (D22).
13. Two "annotation floor" values, 1.4 vs 2.5 mm (flag 14).

### E. Notes vs later docs

Covered per note in §2 above. The main points:
- β miscitation (last_talk).
- MDF → metal; BlenderProc → Isaac.
- Byte identity → content hash.
- `ours` excluded; lit-quadric added; lit-ppf deterministic.
- 100 repeats → 30 seeds; 0/5/10% noise → 0/1/2× σ.
- `1 − occluded_fraction` → `visible_fraction` in MPS.
- D29 framing and joint types amended.
- Groove rows; outline vocabulary; `side_3` naming.
- "D19 already shipped" false for curved and grooved seams.
- phase9 internal counts (120 vs 110; 7 vs 8 items; α sets).
- Phase 9 tooling and schema hooks not built.

---

## 7. D-series glossary (MANUAL §14, for cross-referencing)

| D | Meaning | D | Meaning |
|---|---|---|---|
| D1 | analytic truth | D21 | watertight parts, disjoint union |
| D2 | radius-PCA is a baseline | D22 | seam class from faces |
| D3 | inverted pipeline | D23 | stacked joints cap β at 0.4° |
| D4 | accessibility rule | D24 | grooves deferred to 6b |
| D5 | several seams per scene | D25 | MPS is a rule |
| D6 | one cloud + visibility mask | D26 | two camera regimes |
| D7 | tier boundary = sensor realism | D27 | dimensions vary across the seam, not along it |
| D8 | tacks are a rule | D28 | only the seam edge may be axis-aligned |
| D9 | no simulator in tier 1 | D29 | curve-first families |
| D10 | RGB stored, not benchmarked | D30 | grooves on straight butts only |
| D11 | held-out geometry splits | D31/D32 | class disjointness + boundary stratum |
| D12 | fixture sampled, twin-paired, pose-varied | D33 | curves constructed, never fitted |
| D13 | both faces on workpieces | D34 | chord ≤ 0.25 mm (measured 0.05) |
| D14 | noise realisation is a rule | D35–D37 | groove rows, `groove_root`, mouth-anchored nominal |
| D15 | content-hash determinism | D38 | tackrule-0.1 |
| D16 | stereo model with profiles | D39 | closed-loop phase, rotation-invariant scoring |
| D17 | withdrawn | | |
| D18 | included angle is a design parameter | | |
| D19 | nominal / root / gap-mid | | |
| D20 | raster mode samples hidden surface separately | | |

**Key absolute paths:**
- `/workspaces/welding_cell_ws/weld_generator/docs/SCHEMA.md`
- `/workspaces/welding_cell_ws/weld_generator/docs/scene.schema.json`
- `/workspaces/welding_cell_ws/weld_generator/docs/render.schema.json`
- `/workspaces/welding_cell_ws/weld_generator/docs/MANUAL.md`
- `/workspaces/welding_cell_ws/weld_generator/docs/PARAMETERS.md`
- `/workspaces/welding_cell_ws/weld_generator/README.MD`
- `/workspaces/welding_cell_ws/weld_generator/notes/{last_talk,thesis_direction_handoff,plan_patch_v2,patch_phase6,patch_class_disjointness,phase9_plan}.md`
- `/workspaces/welding_cell_ws/weld_generator/configs/`
- `/workspaces/welding_cell_ws/weld_generator/tests/`
- `/workspaces/welding_cell_ws/weld_generator/weldgen/{cli,hashing,rng,config,accessibility,scene,scene_curved,visibility,mps,tacks,writer,geom}.py`
- `/workspaces/welding_cell_ws/weld_generator/weldgen/render/{draws,writer}.py`
- `/workspaces/welding_cell_ws/weld_generator/scripts/{make_bench6b,make_fixture_twins,run_phase4_batch}.py`