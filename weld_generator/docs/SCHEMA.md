# Weld Seam Dataset — Scene Schema

**schema_version: 1.1.0** — frozen 2026-08-13, minor bump 2026-08-13 for D12/D16/D17.
Phase 0 deliverable of [`../notes/dataset_plan.md`](../notes/dataset_plan.md) §4.
Companion: [`PARAMETERS.md`](PARAMETERS.md).

---

## 0. What "frozen" means, and how to change it

Every field below has a reason to exist. Adding a field after Phase 4 forces a full
regeneration and invalidates every plotted number (`dataset_plan.md` §8).

Change policy, semver on `schema_version`:

| Change | Bump | Consequence |
|---|---|---|
| New **optional** field, absent readers unaffected | patch | none |
| New **required** field, or new enum member | minor | regeneration of affected splits |
| Field removed, renamed, retyped, or unit changed | major | full regeneration, new release DOI |

`schema_version` is the *format*. `generator_version` is the *code that filled it in*.
They move independently: a generator bugfix that changes geometry bumps
`generator_version` and forces regeneration without touching `schema_version`.

---

## 1. Conventions

These are load-bearing. Every one of them has caused a silent bug in somebody's pipeline.

| Concern | Convention | Why |
|---|---|---|
| **Length unit** | **millimetres**, everywhere, no exceptions | ISO 5817, root gap, thickness, plate stock are all specified in mm; the repo's CAD library is already mm (`MESH_SCALE=0.001`, admittance README §13). Metres appear only at the ROS boundary. |
| **Angle unit** | **degrees** in JSON, radians never stored | ISO 5817 states β in degrees; JSON is read by humans |
| **Array dtype** | `float32` for point data, `float64` for transforms/parameters | float32 at 1000 mm resolves ~6e-5 mm — far below any real tolerance. float64 for transforms so composition doesn't drift |
| **Transforms** | 4×4 **row-major**, nested lists, homogeneous, last row `[0,0,0,1]` | `T_world_x` maps a point in frame `x` to world: `p_world = T_world_x @ p_x` |
| **Naming** | `T_<to>_<from>` | reading left-to-right composes: `T_world_cam @ T_cam_pt` |
| **Handedness** | right-handed throughout | |
| **NaN/Inf** | never written; a missing value is `null` | JSON NaN is non-standard and breaks strict parsers |

### 1.1 Frames

Four frames, all right-handed.

**`world`** — the frame all stored geometry lives in. Everything in `cloud.npz` and
`seams.npz` is in world. Origin is the nominal workcell reference point. **+Z is up,
against gravity** — that is the only load-bearing part of the definition, because
`camera.elevation_deg` is measured from the world XY plane.

**Nothing is pinned to the origin.** The assembly pose is sampled (substream 3), and the
fixture, when present, is tilted and z-offset (D12). Pinning either would leak the answer:
a seam that always starts at the origin is trivially predictable, and a fixture whose
working surface is always `z = 0` teaches a model "contacts with the `z = 0` plane are
never weldable" — a rule that transfers to nothing.

**`part`** (one per object) — origin at the primitive's centroid, axes are the
primitive's own `(u, v, w)` (see §2.1). `T_world_part` places it.

**`joint`** — the canonical seam-local frame, one per scene, defined from **seam id 0**:

```
origin = seam0 start point (arc length s = 0)
+X     = unit seam tangent at s = 0
+Z     = unit dihedral bisector at s = 0, pointing into free space (the torch side)
+Y     = Z × X
```

Stored as `T_world_joint`. It exists so scenes are comparable after canonicalisation,
and because the tack rule (Phase 7) and the quality field are naturally expressed in it.
It is a *derived convenience*, not truth — `seams[].parametric` is truth.

**`cam`** — **OpenCV optical convention**: +X right, +Y **down**, +Z **forward along the
view axis**. Stated explicitly because ROS REP-103 body frames (+X forward, +Z up) and
ROS `*_optical_frame` differ, and mixing them silently mirrors every scene. `T_world_cam`
uses this convention. A consumer wanting the ROS body frame applies the standard
`(x,y,z)→(z,-x,-y)` rotation themselves.

### 1.2 The torch approach direction, and a sign disagreement in the literature

For a seam whose two faces have outward normals `n_A`, `n_B`:

```
approach_dir = unit(n_A + n_B)          # points FROM the seam TOWARD the torch
torch_z_axis = -approach_dir            # tool +Z points INTO the work
```

Both cited constructions are the same geometry with opposite sign conventions —
Yi et al. write `v₀ = v₁ + v₂`, Wang et al. write `Z_W = −(n_b + n_m)`. We store
`approach_dir` (torch-ward) and say so here so nobody has to guess.

---

## 2. Naming registries

These are the strings that must be stable across joint types and across phases
(`dataset_plan.md` Phase 0, bullet 4).

### 2.1 Objects

Objects are listed in `objects[]`. Each has:

- `id` — `"A"`, `"B"`, `"C"`, … assigned in construction order; the fixture is always `"F"`.
- `role` — **`"workpiece"` | `"fixture"`**. Load-bearing, see §2.5.
- `object_id` — the `uint8` written per-point in `cloud.npz`. Workpieces are `0,1,2,…`
  in `objects[]` order; **the fixture is always `255`**, so workpiece ids stay
  contiguous from 0 no matter how many parts a scene has.
- `features` — D17, see §2.2.1. Empty list when the part is a bare primitive.

**The fixture is optional (D12).** A scene contains at most one `role: "fixture"` object.
When absent, `joint.contact_mode` is `"free"` and no `object_id` 255 appears in the cloud.
Presence is sampled ~50/50 from Phase 2 onward and is **off throughout Phase 1**, so the
determinism gate is proven before resting-consistency placement exists.

The `objects[]` list is open-ended from day one. Phases 1–6 emit two workpieces and
optionally the fixture, but nothing in the schema assumes it, so 3-part assemblies later
cost no regeneration.

### 2.2 Face names, per primitive

Every primitive declares a local frame `(u, v, w)` and a **fixed** face-name registry.
The rule that makes this stable:

> **`w` is always the thickness axis** — the ISO 5817 `t`. For every slab in the dataset,
> the broad faces are `±w` and the edge faces are `±u`, `±v`.

That single invariant means the tack edge-margin rule (`2t` from a free edge, Phase 7)
and the linear-misalignment defect (`h` along `w`) are expressible without a per-joint
special case.

| Primitive | Local axes | Face names |
|---|---|---|
| `slab` (Phases 1–2, and the fixture) | `u` = length `L`, `v` = width `W`, `w` = thickness `t` | `+u` `-u` `+v` `-v` `+w` `-w` |
| `swept_slab` (Phase 6, curved plate) | `u` = sweep parameter, `v` = width, `w` = thickness | `+u` `-u` (end caps), `+v` `-v` (lateral swept), `+w` `-w` (broad offset surfaces) |
| `cylinder` / `tube` (Phase 6) | `w` = axis | `lateral+` (outer), `lateral-` (inner, tube only), `+w` `-w` (caps) |

**Face reference string:** `"<object_id>:<face_name>"` → `"A:+w"`, `"F:+w"`, `"B:-v"`.

#### 2.2.1 Procedural feature faces (D17)

Parts carry `features: []` — chamfers, hole, slots and so on, applied to the base
primitive. This exists because with bare slabs the D11 "held-out geometry" split is really
a held-out *dimensions* split, which is a much weaker claim than it sounds.

Proposed vocabulary (**provisional** — freeze before Phase 2; adding a member afterwards is
a minor bump, §0):

```
chamfer | edge_fillet | through_hole | slot | notch | stiffener
```

Faces a feature introduces are named `<kind><index>`, optionally `.<sub>` where the feature
contributes several faces:

```
A:through_hole0        A:chamfer2        A:slot1.wall+u        A:stiffener0.+w
```

**The `w`-is-thickness invariant survives — with one exception, and it is worth naming
before the vocabulary is frozen.** `chamfer`, `edge_fillet`, `through_hole`, `slot` and
`notch` are *subtractive*: they remove material from the slab without touching its local
`(u, v, w)` frame, so `±w` remain the broad faces and the Phase 7 edge-margin rule is
unaffected. **`stiffener` is additive** — it is a second slab with its own thickness axis,
which is generally *not* the parent's `w`. Two honest options: model it as a sub-body whose
faces are namespaced `stiffener<i>.<face>` **and** carry their own local frame, or promote
it to a separate `role: "workpiece"` object. The second is cleaner but changes the joint's
semantics (a stiffener welded to a plate *is* a T-joint), so it is not free. This is the
item to settle when picking the vocabulary.

Features change `part_geometry_id` — see §5.4. They must, or the split key is wrong.

### 2.3 The scene face registry

`faces[]` in `scene.json` is a flat, scene-wide list. Its index **is** the per-point
`face_id` in `cloud.npz`:

```jsonc
"faces": [
  { "face_id": 0, "ref": "A:+w", "object": "A", "name": "+w",
    "plane": { "n": [0,0,1], "d": -8.0 },     // n·p + d = 0, n outward
    "area_mm2": 24000.0 }
]
```

Cost: a few hundred bytes. Buys three things — free per-point **face segmentation
labels** (a superset of the per-part A/B/intersect masks that K-Net-style methods
consume, `thesis_direction_handoff.md` §3), a stable `face_id → ref` map, and the plane
equations that both the D4 verification function (Phase 2) and Baseline A (Phase 4) need.

`plane` is `null` for non-planar faces (Phase 6); those carry `surface` instead
(`{"kind":"cylinder","axis":[...],"point":[...],"radius_mm":...}`).

### 2.4 `face_pair` canonical ordering

Always a 2-element list, sorted by `(object_id, face_name)` lexicographically, with the
fixture `"F"` sorting **last**. So `["A:+w", "B:-v"]`, never the reverse. String equality
on the pair is therefore meaningful.

### 2.5 `role`, and why the fixture forces it

When a scene contains the fixture (the steel plate the magnets hold the MDF against —
sampled ~50/50 from Phase 2, D12), a plate standing on it produces a contact line with:

- two **exterior** faces (`A:-w` and `F:+w`),
- a clean 90° dihedral,
- a bisector that escapes to free space.

It passes rule **D4** in full and would be labelled `weldable: true`. It is not — welding
the workpiece to the fixture is exactly the failure you are trying to avoid.

So D4 gains a semantic precondition: **both faces must belong to objects with
`role == "workpiece"`.** Failing that yields `reject_reason: "fixture_contact"`.

This is worth stating in the paper rather than hiding: the rejection is *not* derivable
from geometry. A purely geometric seam detector — including both of our baselines — cannot
distinguish the part–fixture contact line from a genuine fillet. That makes
`fixture_contact` the most interesting class in the weldable-vs-interior confusion metric
(`dataset_plan.md` §7), and it is a hard negative that only exists because the fixture is
in the scene.

**Why sampled rather than always (D12).** The hard negative only pays if there is a
fixture-free control to compare it against — that pairing is the Phase 4 plot quantifying
how much published seam-extraction performance is an artifact of pre-isolated workpieces.
Uniform presence makes the dataset harder without making it informative. See `twin_key`
(§6.3) for how the on/off arms are joined.

### 2.6 `reject_reason` vocabulary — frozen

| Value | Meaning |
|---|---|
| `null` | seam is weldable (`weldable: true`) |
| `fixture_contact` | one or both faces belong to a `role: "fixture"` object (§2.5) |
| `interior_face` | one or both faces have no exterior support (HPR / construction) |
| `bisector_blocked` | the bisector ray re-enters material within `torch_clearance_mm` |
| `degenerate_dihedral` | dihedral angle outside `[dihedral_min_deg, dihedral_max_deg]` |
| `no_contact` | faces separated by more than `contact_tol_mm` along the candidate line |
| `too_short` | clipped intersection shorter than `min_seam_length_mm` |

Adding a member is a **minor** bump (§0). Precedence when several apply is the table order,
top to bottom, and the applied order is recorded so it is never ambiguous.

### 2.7 Seam ids

Assigned after sorting all candidates by

```
(0 if weldable else 1, face_pair[0], face_pair[1], round(start_mm,6), round(end_mm,6))
```

then numbered `0…n-1`. Ids are stable for a given scene but **must not** be used to match
constructed against rediscovered seams in the Phase 2 gate — match those geometrically
(Hungarian assignment on Chamfer distance). Seam id 0 defines the `joint` frame (§1.1),
which is why weldable seams sort first.

---

## 3. Directory layout

### 3.1 A single scene

```
<scene_id>/
  scene.json          # everything in §4
  cloud.npz           # §5.1
  seams.npz           # §5.2
  scene.sha256        # §6.2 content hash, one hex line
  rgb.png             # tier 2 only  (D10: stored, not benchmarked)
  depth.png           # tier 2 only  (uint16, mm)
```

Meshes are **not** written per scene. Parts are parametric primitives; `scene.json`
carries the primitive and its dims, and the generator reconstructs the exact mesh. Only
non-primitive geometry (Phase 6) writes a `.ply`, into the shared `assets/parts/` library
keyed by `part_geometry_id`. This avoids duplicating ~20 identical slab meshes across
every scene, and it is what makes the D11 split possible — see §5.4.

A `--emit-meshes` flag writes per-scene `mesh_<id>.ply` for CloudCompare work (Phase 5).
Those are a convenience artifact, excluded from the content hash.

### 3.2 A release

```
release_v<X.Y.Z>/
  README.md               # schema, ranges, baseline numbers
  MANIFEST.json           # generator_version, schema_version, config set, scene count
  index.jsonl             # one flat row per scene — see §5.3
  splits.json             # §5.4
  configs/                # the exact YAML parameter files used
  assets/parts/           # non-primitive meshes, keyed by part_geometry_id
  scenes/
    <shard>/<scene_id>/   # shard = first 2 chars of scene_id, above ~10k scenes
```

`scene_id = "<config_id>-<seed:010d>"`, where `config_id` is the first 8 hex chars of
the sha256 of the canonical config. Self-describing, collision-free across presets, and
regenerable — you can reproduce any single scene from its id alone.

---

## 4. `scene.json`

```jsonc
{
  "schema_version": "1.0.0",
  "generator_version": "0.1.0",
  "scene_id": "3f9a21c4-0008412337",
  "config_id": "3f9a21c4",
  "seed": 8412337,
  "tier": 1,
  "twin_key": "a91cf3e2b7d40518",  // §6.3 — joins ablation arms with identical geometry

  "units": { "length": "mm", "angle": "deg" },

  "joint": {
    "type": "T",                  // T | corner | butt | lap | edge
    "seam_shape": "line",         // line | arc | spline | closed
    "quality_level": "C",         // ISO 5817 level the sampled defects satisfy — PARAMETERS.md §3
    "contact_mode": "flat"        // free | flat | on_edge | propped;  "free" ⟺ no fixture (D12)
  },

  "objects": [
    { "id": "A", "role": "workpiece", "object_id": 0,
      "primitive": "slab", "dims_mm": [200.0, 120.0, 8.0], "thickness_mm": 8.0,
      "features": [],
      "part_geometry_id": "slab_200.0x120.0x8.0",
      "mesh": null,
      "T_world_part": [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]] },

    { "id": "B", "role": "workpiece", "object_id": 1,
      "primitive": "slab", "dims_mm": [200.0, 90.0, 8.0], "thickness_mm": 8.0,
      "features": [                                    // D17, §2.2.1
        { "kind": "through_hole", "index": 0, "params": { "d_mm": 12.0, "at_uv_mm": [60.0, 0.0] } },
        { "kind": "chamfer", "index": 0, "params": { "edge": "+u/+w", "size_mm": 2.0 } }
      ],
      "part_geometry_id": "slab_200.0x90.0x8.0_f3e91a0c7",
      "mesh": null,
      "T_world_part": [[...]] },

    // present in ~50% of scenes from Phase 2 (D12); absent ⟺ contact_mode "free"
    { "id": "F", "role": "fixture", "object_id": 255,
      "primitive": "slab", "dims_mm": [600.0, 400.0, 10.0], "thickness_mm": 10.0,
      "features": [],
      "part_geometry_id": "slab_600.0x400.0x10.0",
      "mesh": null,
      "T_world_part": [[...]] }        // tilted ±10°, working surface NOT pinned to z = 0
  ],

  "faces": [
    { "face_id": 0, "ref": "A:+w", "object": "A", "name": "+w",
      "plane": { "n": [0.0, 0.0, 1.0], "d": -8.0 }, "surface": null,
      "area_mm2": 24000.0 }
    // … one entry per face of every object, including the fixture
  ],

  "fit": {
    "root_gap_mm": 1.1,                    // ISO 6520-1 no. 617
    "linear_misalignment_mm": 0.4,         // no. 5071, along w
    "angular_misalignment_deg": 0.8,       // no. 508  (Annex B — see PARAMETERS.md §2.3)
    "throat_thickness_mm": 5.6             // aA, the 617 limit depends on it
  },

  "T_world_joint": [[...4x4...]],

  "seams": [
    {
      "id": 0,
      "weldable": true,
      "reject_reason": null,
      "face_pair": ["A:+w", "B:-v"],
      "parametric": {
        "kind": "line",                    // line | arc | bspline
        "p0_mm": [0.0, 0.0, 0.0],
        "p1_mm": [232.0, 0.0, 0.0]
      },
      "length_mm": 232.0,
      "dihedral_deg": 90.0,
      "sampled": { "array": "seam_0", "density_per_mm": 10.0, "n": 2321 },
      "occluded_fraction": 0.34
    },
    {
      "id": 3,
      "weldable": false,
      "reject_reason": "fixture_contact",  // §2.5 — geometrically indistinguishable from a fillet
      "face_pair": ["A:-w", "F:+w"],
      "parametric": { "kind": "line", "p0_mm": [...], "p1_mm": [...] },
      "length_mm": 200.0,
      "dihedral_deg": 90.0,
      "sampled": { "array": "seam_3", "density_per_mm": 10.0, "n": 2001 },
      "occluded_fraction": 0.12
    }
  ],

  "tacks": {                               // Phase 7; null until then
    "rule_version": "tackrule-0.1",
    "params": { "d_min_mm": 40.0, "d_max_mm": 120.0, "edge_margin_factor": 2.0 },
    "points_mm": [[0.0,0.0,0.0], [116.0,0.0,0.0], [232.0,0.0,0.0]],
    "seam_id": [0, 0, 0],
    "arclength_mm": [0.0, 116.0, 232.0]
  },

  "camera": {
    "model": "pinhole",                    // no distortion in tier 1; stated, not implied
    "K": [[674.0,0.0,640.0],[0.0,674.0,360.0],[0.0,0.0,1.0]],
    "width": 1280, "height": 720,
    "T_world_cam": [[...4x4...]],          // OpenCV optical convention, §1.1
    "standoff_mm": 750.0,
    "elevation_deg": 55.0
  },

  "noise_model": {                         // params only; the realization is NOT stored — §5.1
    "kind": "stereo_z2",
    "profile": "d435i",                    // D16 — d435i | stereo_good | stereo_poor
    "subpixel_px": 0.08,
    "baseline_mm": 50.0,
    "focal_px": 674.0,
    "min_z_mm": 280.0,                     // the sensor's blind-zone bound, not a schema constant
    "lateral_sigma_px": 0.8,
    "grazing_dropout_deg": 75.0,
    "seed": 8412337
  },

  "cloud": {
    "file": "cloud.npz",
    "frame": "world",
    "sampling_mode": "area_uniform",       // area_uniform | camera_raster — §5.1
    "density_per_mm2": 1.0,
    "n_points": 184203
  },

  "rgb": null,                             // tier 2 only
  "depth": null,

  "provenance": {                          // EXCLUDED from the content hash — §6.2
    "created_utc": "2026-08-13T14:31:07Z",
    "git_commit": "9dd4212",
    "python": "3.12.3",
    "numpy": "2.3.5",
    "trimesh": "4.12.2",
    "open3d": "0.19.0"
  }
}
```

### 4.1 Notes on specific fields

- **`weldable: false` seams are kept, never dropped.** Free hard negatives, and they
  document what the accessibility rule rejected. Fixture-present scenes have at least one
  `fixture_contact` on top of whatever the joint geometry rejects (§2.5).
- **`parametric` is the truth; `sampled` is a convenience.** The parametric form is what
  makes "10 dots per mm, or 50 if someone asks" real — resampling needs no regeneration.
  If the two ever disagree, `parametric` wins.
- **`occluded_fraction` is per seam, not per scene.** A scene can have one fully visible
  and one fully hidden seam; that is the point.
- **`throat_thickness_mm` (`aA`)** is in `fit` because the ISO 5817 no. 617 root-gap limit
  is a function of it. See `PARAMETERS.md` §2.4 for the convention used to set it.
- **`contact_mode`** describes how the assembly rests on the fixture — it has to sit in a
  physically consistent way, and *how* it sits changes which seams the camera can see.
  **`"free"` means no fixture in the scene**, and is the only legal value when no
  `role: "fixture"` object is present. Phase 1 is entirely `"free"` (D12).
- **`noise_model.profile`** (D16) names a sensor rather than hard-coding one. `min_z_mm`
  lives here, not in the schema's bounds, because the blind zone is a property of the
  sensor — which is what makes `standoff_mm` legal down to 200 mm for `stereo_good` and
  only 400 mm for `stereo_poor`.

---

## 5. Binary arrays and release-level files

### 5.1 `cloud.npz`

| Array | dtype | shape | Meaning |
|---|---|---|---|
| `xyz` | float32 | (N,3) | **exact, noiseless** surface sample, world frame, mm |
| `normals` | float32 | (N,3) | exact outward unit normals |
| `object_id` | uint8 | (N,) | §2.1; fixture = 255 |
| `face_id` | uint8 | (N,) | index into `faces[]` — free per-point face segmentation |
| `visible_from_cam` | bool | (N,) | Phase 3 ray-cast result (D6) |

**The noisy cloud is not stored.** `xyz` is the exact sample; the sensor realization is
produced by the released, versioned function

```python
xyz_noisy = weldgen.noise.apply(xyz, normals, T_world_cam, noise_model)   # deterministic in noise_model.seed
```

Same epistemic move as D8 makes for tacks: the exact thing is ground truth, the corruption
is a citable convention anyone can re-run with different constants. It also halves the
release size. **Exception:** the frozen Zenodo release additionally materialises
`xyz_noisy` so cross-paper numbers are bit-comparable without depending on a library
version; the git-based generator does not.

**`sampling_mode` matters more than it looks.** `area_uniform` samples uniformly over
surface area — that is *not* what a depth camera returns. A real single-view cloud is a
pixel raster whose surface density falls off with range and incidence angle. Both modes
are in the schema from day one so the single-view benchmark condition can be made
genuinely camera-like without a schema bump.

### 5.2 `seams.npz`

One group of arrays per seam, named by `sampled.array` (`seam_0`, `seam_1`, …):

| Array | dtype | shape | Meaning |
|---|---|---|---|
| `seam_<i>` | float32 | (M,3) | sampled polyline, world, mm |
| `seam_<i>_s` | float32 | (M,) | arc length from start, mm |
| `seam_<i>_tangent` | float32 | (M,3) | unit tangent |
| `seam_<i>_approach` | float32 | (M,3) | unit `approach_dir`, torch-ward (§1.2) |
| `seam_<i>_nA`, `seam_<i>_nB` | float32 | (M,3) | the two face normals |
| `seam_<i>_visible` | bool | (M,) | Phase 3; `occluded_fraction = 1 - mean(visible)` |

The frames are strictly regenerable from `parametric` + `faces[]`, but storing them makes
the file self-contained — the difference between a dataset someone can use and one that
requires your code. They vary along the curve for every Phase 6 seam.

### 5.3 `index.jsonl`

One flat JSON object per line, one line per scene. Purely denormalised — this is the file
people load into pandas to make the Phase 4 plots without touching 10k scene files.

```jsonc
{"scene_id":"3f9a21c4-0008412337","config_id":"3f9a21c4","seed":8412337,"tier":1,
 "twin_key":"a91cf3e2b7d40518",
 "joint_type":"T","seam_shape":"line","quality_level":"C","contact_mode":"flat",
 "fixture_present":true,"n_features":2,
 "t_min_mm":8.0,"t_max_mm":8.0,"root_gap_mm":1.1,
 "linear_misalignment_mm":0.4,"angular_misalignment_deg":0.8,
 "n_seams":4,"n_weldable":2,"n_fixture_contact":1,"total_seam_length_mm":432.0,
 "mean_occluded_fraction":0.34,"n_points":184203,"density_per_mm2":1.0,
 "sensor_profile":"d435i","standoff_mm":750.0,"elevation_deg":55.0,
 "part_geometry_ids":["slab_200.0x120.0x8.0","slab_200.0x90.0x8.0_f3e91a0c7"],
 "content_hash":"9c1f…"}
```

`fixture_present`, `sensor_profile` and `twin_key` are denormalised here specifically so
the Phase 4 ablation plots are a `groupby` on this one file — `df.groupby(["twin_key",
"fixture_present"])` is the whole fixture ablation, and swapping the second key for
`sensor_profile` is the D16 plot.

`content_hash` lives here and in `scene.sha256`, **not** in `scene.json` — a file cannot
contain its own hash.

### 5.4 `splits.json`

D11: hold out **part geometries and joint configurations**, never random frames.

```jsonc
{
  "policy": "held_out_geometry_and_joint_config",
  "key": ["part_geometry_id", "joint_type", "seam_shape"],
  "train": ["3f9a21c4-0008412337", "…"],
  "val":   ["…"],
  "test":  ["…"],
  "held_out_part_geometry_ids": ["slab_320.0x150.0x3.0", "…"],
  "held_out_joint_configs": [["lap", "arc"], ["edge", "spline"]]
}
```

`part_geometry_id` is:

| Case | Form | Example |
|---|---|---|
| Bare primitive | `"<primitive>_<dims joined by x>"` | `slab_200.0x90.0x8.0` |
| With features (D17) | `… + "_f" + sha256(canonical(features))[:8]` | `slab_200.0x90.0x8.0_f3e91a0c7` |
| Phase 6 asset mesh | content hash of the `.ply` | `mesh_3e91a0c7…` |

**Features must enter this id.** With bare slabs the D11 "held-out geometry" split is really
a held-out *dimensions* split — a much weaker claim than it reads as. Hashing the feature
spec is what makes "held-out geometry" mean what it says. It is also the second reason
parts are parametric and shared rather than baked per scene (§3.1).

---

## 6. Reproducibility

### 6.1 RNG substreams

One seed per scene. `numpy.random.SeedSequence(seed).spawn(8)`, `PCG64`, in this **fixed**
order:

| # | Substream | Draws |
|---|---|---|
| 0 | `joint_config` | joint type, part dims, thicknesses |
| 1 | `defects` | root gap, linear/angular misalignment, resulting quality level |
| 2 | `seam_curve` | seam shape, length, curvature / control points |
| 3 | `placement` | assembly world pose, **fixture presence / pose / dims**, `contact_mode` |
| 4 | `surface_sample` | point sampling, density |
| 5 | `camera` | sensor profile, camera pose, intrinsics jitter |
| 6 | `noise` | depth-noise realization |
| 7 | *reserved* | |

The split between 0–2 and 3–6 is not arbitrary: **substreams 0–2 determine the workpiece
geometry and the seam truth, 3–6 determine everything that is an ablation axis.** That is
exactly what makes `twin_key` (§6.3) well-defined, and it is why fixture presence lives in
3 rather than 0.

**Substreams are appended, never reordered. Within a substream, draws are appended, never
reordered.** This is what lets Phase 3 add camera sampling without changing a single byte
of Phase 1–2 geometry — the single most valuable property in the whole document, given
that "schema churn after Phase 4" is a named risk.

No `random`, no `np.random` global, ever. Only explicit `Generator` objects.

### 6.2 The determinism gate is a content hash, not byte-identity

`np.savez` writes a zip, and zip entries embed a wall-clock timestamp — so re-running the
generator produces a **different file** with identical contents. The Phase 1 gate is
therefore defined on content, which is the stronger property anyway:

```python
h = sha256()
h.update(canonical_json(scene_json_without("provenance")))
for name in sorted(all_arrays):               # "cloud.npz:xyz", "seams.npz:seam_0", …
    h.update(name.encode())
    h.update(str(arr.dtype).encode())
    h.update(str(arr.shape).encode())
    h.update(np.ascontiguousarray(arr).tobytes())
```

`canonical_json` = `json.dumps(obj, sort_keys=True, separators=(",", ":"),
ensure_ascii=True, allow_nan=False)`. Python 3 `float.__repr__` is the shortest
round-tripping representation, so floats survive a write/read cycle exactly.

`provenance` is excluded precisely because it holds a timestamp and library versions —
otherwise re-running next month would "fail" a gate it actually passes.

**Phase 1 gate:** `generate(config, seed)` twice, in separate processes, on different
machines → identical `content_hash`. If this fails, the release-as-a-program argument
collapses, and nothing downstream is worth measuring.

### 6.3 `twin_key` — how paired-ablation arms are joined

D12 needs fixture-on and fixture-off scenes with *identical* geometry; D16 needs the same
scene across three sensor profiles; Phase 8 needs a tier-2 scene and its tier-1 twin. All
three are the same requirement, so they get one mechanism instead of three ad-hoc ones.

```python
twin_key = sha256(canonical_json(config_geometry_part) + seed.to_bytes(8, "big"))[:16]
```

where `config_geometry_part` is the subset of the config consumed by **substreams 0–2**
only. Scenes sharing a `twin_key` have bit-identical workpiece geometry and bit-identical
seam truth; they differ only in placement, sampling, sensor or tier. The ablation variable
is then read from the existing fields — `fixture_present`, `sensor_profile`, `tier`,
`density_per_mm2` — so no per-ablation bookkeeping field is ever needed.

**Generation strategy, not just a field.** A ~50% marginal fixture rate does *not* arise
from sampling presence independently — that gives random on/off with no guaranteed partner.
Arms are emitted **deliberately in pairs**: for each geometry seed, one scene with the
fixture forced on and one forced off. The marginal comes out at 50% and every scene has an
exact twin. Same pattern for sensor profiles (one scene per profile per geometry seed).

A consequence to respect in Phase 9: **`twin_key` must not straddle a split boundary.**
Splitting on `part_geometry_id` (D11) already keeps twins together, since twins share their
part geometries by construction — but if the split policy ever changes, check this again,
because a twin pair split across train and test leaks geometry perfectly.

---

## 7. Open items carried into Phase 1

| Item | Status |
|---|---|
| `camera_raster` sampling mode | schema slot reserved; implement in Phase 3 alongside visibility |
| `surface` block for non-planar faces | schema slot reserved; fill in Phase 6 |
| Phase 6 `bspline` parametric form (knots, degree, control points) | slot reserved, exact encoding to be pinned before Phase 6 starts |
| `tacks` block | `null` until Phase 7; shape frozen here so adding it is a patch bump |
| **D17 feature vocabulary** | provisional (§2.2.1). **Freeze before Phase 2** — it feeds `part_geometry_id`, which is the D11 split key. The `stiffener` question is the one to settle |
| Confirm the `d435i` profile against the real camera | `PARAMETERS.md` §4. Under D16 this gates only that profile's claim to match the lab hardware, not the release |
