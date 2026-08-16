# weld_generator

Analytic weld-seam scene generator. **Ground truth is constructed, never detected** —
seam geometry is computed in closed form from the placement transforms, so there is no
annotator, no estimator, and no intra-rater-consistency hand-waving anywhere in the label
path.

Status: **Phase 1 complete** (tier-1 geometry core, straight seams, T-joint only).
Plan: [`notes/dataset_plan.md`](notes/dataset_plan.md).

---

## Quick start

```bash
pip install -r requirements.txt

python -m weldgen generate --config configs/phase1.yaml --n 16 --out out/phase1
python -m weldgen verify   --out out/phase1          # re-hash every scene on disk
python -m pytest tests/ -q
```

`notebooks/01_inspect_scene.ipynb` builds one scene and plots the cloud, the seams and the
three D19 curves.

---

## Layout

```
docs/            frozen contracts - read these before changing anything
  SCHEMA.md          scene format, naming registries, reproducibility   (v2.0.0)
  PARAMETERS.md      every range, tagged [ISO] / [repo] / [ds] / [ours] (v2.0.0)
  scene.schema.json  machine-checkable form of SCHEMA.md

weldgen/         the generator - trimesh + NumPy only, no simulator import (D9)
  rng.py             8 named SeedSequence substreams
  geom.py            slab primitive, face registry, exact plane algebra
  joints.py          T-joint construction; seams derived from the transforms
  sampling.py        area-uniform surface sampling with exact normals + face ids
  config.py          ISO limit functions, parameter sampling, quality classification
  hashing.py         canonical JSON, content hash, config_id, twin_key
  scene.py           assembles a schema-conforming scene
  writer.py          scene.json / cloud.npz / seams.npz / index.jsonl
  cli.py             `python -m weldgen generate | verify`

configs/         phase1, smoke, reference_tjoint presets
tests/           92 tests: geometry, determinism, schema conformance, ISO limits, viz
notes/           the plan, the handoff, and the standards PDFs
papers/          literature
```

---

## What Phase 1 produces

Per scene: two watertight slabs forming a T-joint, **two fillets** (a T-joint has one per
side of the standing plate), an area-uniform surface sample with exact per-point normals,
`object_id` and `face_id`, and every seam stored three ways (D19).

```
out/phase1/<scene_id>/
  scene.json      conforms to docs/scene.schema.json
  cloud.npz       xyz, normals, object_id, face_id, visible_from_cam
  seams.npz       per seam: nominal, root, gapmid, arclength, tangent, approach, nA, nB
  scene.sha256    the content hash
```

### The gate

`generate(config, seed)` twice, in separate processes → **identical content hash**. Not
byte-identity: `np.savez` writes a zip and zip entries embed a wall-clock timestamp, so
byte-identity is unachievable. Hashing canonical JSON plus raw array bytes is both
achievable and the stronger property. `provenance` is excluded, so re-running next month
passes rather than "fails".

```bash
python -m pytest tests/test_determinism.py -q
```

---

## Things that are easy to get wrong

**`included_angle_deg` is not `dihedral_deg`.** The first is the ISO 9692-1 *design*
nominal, one per scene. The second is a property of each *seam*. A T-joint at a 70°
included angle has two fillets, at 70° and 110° — one scene, one included angle, two
dihedrals. `tests/test_geometry.py::test_fillet_dihedrals_are_supplementary` pins this.

**`fit.angular_misalignment_deg` is not `included_angle_deg` either.** β is the *deviation*
from nominal, with ISO 5817 tolerance limits. A 70° joint is not a 90° joint with a 20°
defect (D18).

**The seam is a choice when the root gap is non-zero.** `nominal`, `root` and `gap_mid` are
three different curves separated by `O(g)` — at the reference 1,1 mm gap that is larger
than the ~0,6 mm RMSE the literature reports. We store `nominal` and ship the other two
derived. See `SCHEMA.md` §1.3.

**Scenes are posed randomly in the world, so world axes mean nothing.** Yaw and
translation are sampled per scene (SCHEMA.md §1.1) precisely so a joint pinned to the world
axes cannot leak the answer. Any cross-section or figure must first go through
`weldgen.viz.assembly_frame`, which puts A's top face on `z = 0`, B at `y >= 0` starting at
`z = g`, and the seam along `+X`. Slicing on world X instead produces markers that wander
across the plate and move with every seed.

**`w` is always the thickness axis.** For every slab, the broad faces are `±w`. Note the
consequence in `joints.py`: mapping the standing plate's height onto +Z *and* its thickness
onto +Y at once would be a reflection, not a rotation — so `B:+w` is the face at `y = 0`.

**The root gap answers to two standards that disagree on thin sheet.** ISO 9692-1 allows
`b ≤ 2 mm` for fillets regardless of thickness; ISO 5817 clause 617 scales with the throat,
so on 2 mm sheet level B allows only 0,34 mm. Sampling from the preparation range alone
drives ~37% of scenes to `below_D`. `config.py` draws against the 617 limit for the target
level with the 9692-1 cap as a ceiling.

**`quality_level` is derived, never assumed.** Sampling picks a target and draws inside it,
but what gets stored is recomputed from the realised defects — the weakest of the strictest
levels each imperfection satisfies (PARAMETERS.md §2.5).

---

## Not yet implemented

Phase 1 deliberately stops short. Fields exist in the schema and are filled with honest
placeholders rather than being absent, so the array layout never changes:

| Field | Phase 1 value | Filled in |
|---|---|---|
| `cloud.visible_from_cam` | all `True` — no camera yet | Phase 3 |
| `seams[].occluded_fraction` | `0.0` | Phase 3 |
| `camera.T_world_cam` | identity | Phase 3 |
| `tacks` | `null` | Phase 7 |
| fixture object | absent, `contact_mode: "free"` (D12) | Phase 2 |
| joint types other than `T` | — | Phase 2 |
| curved seams, non-slab primitives | — | Phase 6 |
