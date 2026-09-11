# Phase 8 execution plan — RTX 5070 Ti machine (written 2026-09-10)

*Companion to `handoff_phase8.md` §4 (the agreed plan) and `dataset_plan.md` Phase 8. This
file is the machine-specific execution plan: what was verified on this box today, what the
feasibility prototype measured, the decisions Phase 8 forces, and the milestone order.
Nothing here re-litigates §2 of the handoff.*

---

## 0. State of this machine, verified 2026-09-10

| item | result |
|---|---|
| GPU / driver | RTX 5070 Ti (16 GB), driver 580.173.02, CUDA 13.0, Vulkan OK inside the container |
| Isaac Sim | `/isaac-sim` = 5.1.0-rc.19 (Kit 107.3.3), headless `SimulationApp` starts in ~9 s |
| Replicator | `omni.replicator.core` 1.12.27 loads after `SimulationApp`; `pxr` importable only after it |
| Isaac Python | 3.11.13 at `/isaac-sim/python.sh`; ships numpy 1.26.0, trimesh, scipy, **rtree**, PIL, imageio, yaml. No pandas, no OpenEXR |
| `weldgen` inside Isaac Python | imports cleanly with `sys.path.insert(0, <repo>/weld_generator)` — the renderer can call `geom.mesh()` directly, no PLY round-trip |
| System Python | 3.12.3; tier-1 deps installed with `pip install --break-system-packages` (PEP 668 container; debian-owned matplotlib 3.6.3 / kiwisolver left in place, so do not `pip install -r requirements.txt` blindly — install the missing names individually). Also installed `pandas` for `run_phase4_batch.py --list` |
| `out/` | fully synced: `phase4_batch` 26 MB, `annotation` 109 MB, `bench_phase4` 2,9 GB, `bench_phase4_fx` 3,8 GB, plus every older corpus (`bench_fx` alone is 7,6 GB) |
| `weldgen verify --out out/bench_phase4/T` | **360/360 verify** — hashes reproduce on this machine with trimesh 5.1.0 (laptop had 4.12.2) |
| `qa_d28_gate.py out/bench_phase4` | D34 PASS (worst 0,050 mm vs 0,25); D28 reported |
| `run_phase4_batch.py --list` | 53 chunks, ~28,8 h on this hardware, all existing → skip |
| `pytest tests/ -q` | **venv: 473 passed + 4 xpassed in 18:55** (niced, 4 threads — matches the handoff); system Python: 39 failed, all environmental (§0.1) |
| Disk | container root is the NVMe overlay: 469 GB, **89 % used, 52 GB free**. `/dev/sda1`, `/dev/sdb1`, `/dev/sdb2`, `/dev/sdc5` are visible (container is `--privileged` with `/dev` bind-mounted) but they are **NTFS** Windows volumes and are not mounted anywhere in the container |

Phase 8 output is small: ~3 MB per scene (rgb.png + depth.png + mask) × 994 scenes ≈ 3 GB,
plus a few GB of RTX shader cache on first run. 52 GB is enough. If space is wanted anyway,
the safe order is: `rm -rf /root/.cache/vscode-cpptools` (2,5 GB), then the regenerable
older corpora (`out/bench_fx` 7,6 GB, `out/bench` 1,4 GB) — never the four synced ones.

**The render disk (updated 2026-09-10, afternoon).** The user reformatted `/dev/sdb1` as
ext4 (label `renders`) and mounts it on the host at `/mnt/renders`. It reaches the
container through one added line in `.devcontainer/devcontainer.json` `runArgs`:
`"--volume=/mnt/renders:/mnt/renders:rw"` — takes effect after *Dev Containers: Rebuild
Container* — done 2026-09-10 evening; `/mnt/renders` shows 916 GB ext4. Then, inside the container:

```bash
mkdir -p /mnt/renders/weld_generator
cp -a out /mnt/renders/weld_generator/out        # 19 GB; no rsync in this image. Verify after:
.venv/bin/python -m weldgen verify --out /mnt/renders/weld_generator/out/bench_phase4/T
mv out out_nvme_old && ln -s /mnt/renders/weld_generator/out out    # then delete out_nvme_old
```

**Done 2026-09-11:** copied (19 GB, file lists identical), all 994 scenes of `bench_phase4` and
`bench_phase4_fx` verify on the render disk, `out` is now the symlink, `.gitignore` gained a bare
`out` entry (the `out/` pattern matches directories only, not symlinks). `out_nvme_old/` on the
NVMe is the redundant copy, left for the user to delete.

Everything under `out/` (corpora, renders, the 3600-scene training corpus) then lives on
the render disk; the repo and `.venv` stay on the NVMe. The other partitions (`sda1`,
`sdb2`, `sdc5`) are still NTFS Windows volumes — leave them alone.

**Tier-1 venv** lives at `/workspaces/welding_cell_ws/.venv` (same layout as the laptop;
`.gitignore` already lists it, but five of its files are tracked from an old commit —
`git rm -r --cached .venv` once to stop git reporting them). Created with
`uv venv --python 3.12 .venv && uv pip install --python .venv/bin/python "numpy>=2"
"scipy>=1.13" trimesh pyyaml jsonschema pytest matplotlib pandas scikit-learn jupyter
nbconvert ipykernel` (`python3 -m venv` lacks ensurepip in this image; `uv` lives in
`/root/.local/bin` and **does not survive a container rebuild** — nothing under `/root` does,
including the Claude memory dir, while the workspace and its `.venv` are bind-mounted and do). A full `pytest`
run once froze VS Code on this box — run it niced with capped BLAS threads:
`OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 nice -n 19 .venv/bin/python -m pytest tests/ -q`.

### 0.1 Test suite on this machine

First run under the container's system Python 3.12 (numpy 1.26.4): **39 failed, 434 passed,
4 xpassed in 9:57**. Every failure is environmental, none is a regression:

| cause | tests | fix |
|---|---|---|
| `np.trapezoid` (numpy ≥ 2 only; laptop had numpy 2.3.5 per `provenance`) | 28 (`test_prepared_prism`, `test_grooves`) | run tier 1 in a venv with numpy 2 |
| `pandas` missing at start of run | 6 (`test_harness`, `test_annotation`) | in the venv |
| `scikit-learn` missing (`cluster_method='dbscan'`) | 5 (`test_lit_ppf`) | in the venv |

Tier 1 therefore runs from `/workspaces/welding_cell_ws/.venv` on this machine (see §0 for
how it is created), `.venv/bin/python -m pytest tests/ -q` from `weld_generator/`.
`requirements.txt` should gain `numpy>=2` (the code already requires it) and note pandas /
scikit-learn as harness extras.

**Consequence for tier 2:** Isaac Python is pinned to numpy 1.26.0, so
`weldgen/geom.py` has `_trapezoid = getattr(np, "trapezoid", None) or np.trapz` at all three
call sites (M1, 2026-09-11) so `PreparedPrism.mesh()` runs inside the renderer.

---

## 1. Feasibility prototype — milestone (a) already passes

Script: `scripts/tier2_gate_proto.py` (kept verbatim as the prototype; to be ported into the
repo as `weldgen/render/` + `scripts/tier2_gate.py` in M2).
Scene: `out/bench_phase4/T/1ce3c6d2-0002000008` (T/tube-on-plate: one slab + one tube with a
planar base cut, 292-vertex mesh).

Pipeline: `scene.json` objects → `weldgen.geom` primitives → `mesh()` → `UsdGeom.Mesh`
(points × 0,001, `subdivisionScheme = none`, `primvars:weldgen_object_id`) → `UsdGeom.Camera`
driven from the stored `K` / `T_world_cam` → Replicator `render_product` +
`distance_to_image_plane` + `rgb` annotators → back-project → distance to the exact meshes
(`trimesh.proximity.closest_point`, rtree available in Isaac Python).

| measurement | value |
|---|---|
| frame time, 1280×720, `rt_subframes=16`, no materials | 0,11 s (first frame 0,26 s) |
| valid depth pixels | 7,6 % of the image (69 546 px), z 282–455 mm |
| **residual to exact surface** (pixel-centre convention) | median 0,0002 mm, p95 0,0005, p99 0,0006, **max 0,001 mm**, 0 px over 0,25 mm |
| residual with pixel-corner convention | median 0,263 mm, 71 % over budget — a convention error, not a renderer error |
| tier-1 `visible_from_cam` points landing on a depth pixel within 1 mm | 93,5 % (rest are edge pixels under nearest-pixel sampling) |

**The D34 gate holds with three orders of magnitude to spare.** The renderer's ray cast is
exact on these meshes; the whole error budget of tier 2 will come from the sensor model,
which is the design intent.

Conventions the prototype pinned down (record in SCHEMA.md when M3 lands):

1. **Pixel centres.** Depth pixel `(i, j)` back-projects through `(u, v) = (j + 0,5, i + 0,5)`
   with the stored `K` (principal point `W/2, H/2`). Using `(j, i)` puts a 0,26 mm bias in.
2. **OpenCV → USD camera.** `T_usd = T_world_cam @ diag(1, −1, −1, 1)` (USD looks down −Z,
   +Y up), translation × 0,001. Set with one `AddTransformOp().Set(Gf.Matrix4d(T_usd.T))`
   (Gf is row-vector).
3. **Intrinsics.** `focalLength = fx · horizontalAperture / W`, `verticalAperture =
   horizontalAperture · H / W`, aperture 20,955 (default). fx = fy holds by construction.
4. **Units.** Stage stays in metres (Isaac default; RTX clipping and ray offsets are tuned
   for it). Multiply by 0,001 in, by 1000 out. `distance_to_image_plane` is the optical-axis
   depth `z_cam`, i.e. exactly the `z` that `weldgen.noise` and `camera.project` use — not
   Euclidean range.
5. **Build the stage with `pxr`, not `rep.create.*`.** Prims made through `rep.create`
   are graph nodes that were not evaluated before the first `step()` (the depth came back
   empty across four steps); prims defined through `UsdGeom` rendered on step 0. Replicator
   is used only for render products, annotators and the writer.
6. **`visible_from_cam` is not "seen by the renderer".** It also applies the sensor profile's
   `min_z_mm` blind zone (400 mm for `stereo_poor`) and the frustum, so on this scene only
   5 % of the cloud is flagged visible while the renderer sees everything from 282 mm.
   The coverage half of the gate must apply `camera.in_frustum` with the profile's
   `min_z_mm` to the rendered pixels before comparing — or compare against a
   geometry-only occlusion mask. Do the former: it is the stored semantics.
7. **The primitive factory is the missing piece.** `bench_phase4` has *no* all-slab scene:
   T = prism+slab / slab+swept_slab / slab+tube / tube+tube; butt = prepared_prism ×2,
   prepared_prism+prism, prism×2, swept_slab×2; corner/edge = prism×2; lap = prism+slab.
   `scene.json` carries everything needed (`outline_uv` + `outline_shape` for prisms,
   `params` for tube / swept_slab incl. `spine` from `to_parametric()`, `params` = `prep`
   for prepared prisms), and `weldgen.curves.from_parametric` exists. The generic
   `from_object(entry)` in the prototype rebuilt slab and tube with no missing fields.

---

## 1.1 Look-before-render pilot (2026-09-11)

`scripts/tier2_pilot_proto.py <scene_dir> <out_dir> [alloy]` renders view 0 (the tier-1
camera) plus two drawn views with RGB, `depth.png` (uint16, 0,05 mm), `mask_seam`,
`mask_tack`, `mask_object`, `view.json` and a `review.png` composite (RGB | depth | overlay,
seam green, tacks red). Outputs under `out/pilot_2026-09-11/<stratum>_<alloy>/<view>/`
(render disk, git-ignored) — open the `review.png` and `review_crop3x.png` files in VS Code.

Findings, in the order they were hit:

1. **Renders are right.** Geometry, pinned camera and depth match tier 1 (§1 gate); the
   seam mask sits on the joint in every view; tacks appear as short segments of the
   right length on the seam; the T/tube scene shows 2 of 8 tacks in view 0, correctly
   (seam 1 is the inner ring inside the bore, the far half of seam 0 is behind the tube).
2. **Seam visibility must be the analytic ray cast, not depth agreement.** The first
   pilot drew the seam dashed near the tube's silhouette: at grazing angles one pixel
   spans ~10 mm of depth, and with a 1,2 mm root gap the D19 midline floats in free air,
   so "rendered depth agrees with the seam point" flickers. `visibility.visible_mask`
   (no blind zone, `face_test=False`) is exact and is what tier 1 reports anyway; the
   depth test is now a printed cross-check. Recorded in `dataset_plan.md` Phase 8.
   After the switch the T/tube seam is continuous along the visible arc; analytic and
   depth visibility agree on 92–97 % of seam points on T and stiffener scenes, and the
   disagreements are all silhouette pixels (lap seams that run along a plate edge: the
   depth test calls half of them "visible" because the pixel falls just outside the
   rendered silhouette).
3. **Plate-pipeline seams carry no `closed` key**; the curved pipeline adds it. Consumers
   use `s.get("closed", False)`. (Schema: worth making `closed` mandatory in 2.4.)
4. **Materials look like matte plastic** under `UsdPreviewSurface` + dome light — expected;
   this is what M5 exists for (OmniPBR/MDL with roughness maps, mill scale, HDRI). The
   six-alloy palette in the pilot is a placeholder for telling scenes apart.
5. **Background is a void** — no table, plate floating on grey. **Rejected by the user the
   same day, rightly**: shortcut learning on the silhouette, depth cliffs at the part
   boundary. Decision (`dataset_plan.md` Phase 8, "The environment layer"): a substrate
   plane in *every* view at the lowest workpiece point, domain-randomised texture (the
   lab's MDF board from photographs, slotted steel table, scratched steel, concrete,
   rubber), workshop HDRI dome; labelled `mask_object = 254`; twin gate on workpiece
   pixels. Second pilot (`*_on_mdf`, `*_on_steel`, `*_hdr` dirs) renders it: depth
   valid fraction goes from 1–14 % to 61–100 % of the frame, the twin gate on workpiece
   pixels is unchanged (p99 ≤ 0,0008 mm, max ≤ 0,0012 mm over nine views), seam and tack
   pixel counts are identical to the void renders. The plane is 2 m square at the lowest
   workpiece vertex; at low elevations (the lap scene's 20° tier-1 camera) its far edge
   is in frame, beyond which RGB shows the dome and depth is 0/invalid — physically
   right for a sensor out of range, but the production plane should be 4 m or the HDRI
   should be a real workshop so the RGB beyond the edge is not a flat grey.

   **Photos needed from the lab (the MDF substrate and the bench) — for M5:**
   * the bare MDF board, top-down, filling the frame, diffuse even light (overcast window
     or bounced), no parts, 4–6 shots at different spots and after use (scratches, marks);
     one with a ruler or a known-size object in frame for scale; highest resolution the
     phone/camera gives, no HDR/beauty processing;
   * 2–3 shots of the board at a grazing angle with a single lamp, to read its roughness;
   * the bench context as the camera sees it: 6–8 shots from the eye-in-hand camera's
     typical poses (300–1200 mm standoff, 20–70° elevation) with and without parts — these
     become the reference for the HDRI / backdrop choice and for the Phase 9 twin;
   * if available, a 360° panorama of the cell from bench height (any phone panorama app)
     — the cheapest source of a lab-specific HDRI.
6. **Framing for training views.** The lap scene's tier-1 camera frames the assembly at
   0,7 % of the image, edge-on — `framing_frac` up to 1,45 and low elevation are the
   benchmark's difficulty axis by design. View 0 keeps that (it is the twin), but views
   1–9 exist to train on, so their sampler should use a tighter framing range (proposal:
   `framing_frac` 0,5–1,0, elevation 25–70°) — a render-config parameter, recorded per
   view, never touching the tier-1 draw. Open item added to `dataset_plan.md`.
7. **Drawn views must reuse the tier-1 camera sampler**, aim point included. The pilot's
   ad-hoc `sample_pose` calls (fixed elevation over world XY, cloud-centroid aim) looked
   at the lap assembly from below in both extra views, with every seam hidden.
8. **Exposure.** Stainless and aluminium render nearly white under dome 800 + key 2500 —
   set explicit exposure/tonemapping in M5 and draw light intensity in a range that keeps
   the brightest alloy unclipped.
9. **Backgrounds are the user's photographs** (`out/backgrounds/`, 41 JPEGs at 1500×2000 /
   2000×1500 / 2816×1536): the MDF grid board on its frame, benches with tools, concrete,
   slotted steel, pallets. The pilot maps one photo per view onto the 4 m plane as albedo
   (1,5 m span, mirror wrap, random rotation); the procedural `textures/` are retired.
10. **Joint always mid-frame** (user): the pilot's drawn views aimed at the seam centroid
    exactly. Fixed with the tier-1 aim-jitter idea at 0,35 × span; rule in `dataset_plan.md`.
11. **Views with no reachable seam** (user, lap scene): fixed with the redraw rule — a drawn
    view must show ≥ 10 % of a primary seam by the analytic ray cast, else redraw (≤ 40).
12. **Photo backgrounds rendered** (`*_bg` dirs): welding-table, concrete and bench photos
    under the four scenes. They read as bench shots; the photo's own perspective and
    mirrored repeats are visible at the frame edges (a domain-randomisation artefact, not a
    depth error — depth is the flat plane). Near-top-down photos (the MDF board shots)
    look most physical; tag the set so the substrate draw prefers them for view 0.
    Drawn views: joint off-centre, first attempt accepted in all draws, best primary
    visibility 0,41–1,0. Two fixes from the numbers, both verified on a re-render:
    standoff clamped to 300–1200 mm (one draw had put the camera 13 mm from the tube;
    now ≥ 168 mm to the nearest surface), and render draws seeded from `scene_id` (the
    brass and stainless scenes share seed 2000005 and had drawn the same photo and cameras).
13. **Round 2 — one scene per stratum, all six alloys** (`out/pilot_2026-09-11/round2/`,
    `sheet_view0_a.png`, `sheet_view0_b.png`, `sheet_drawn.png`). Ten of twelve rendered
    first time; T/circle and T/rounded_rect were **SIGKILLed** (no traceback) in a drawn
    view that fills the frame with a large curved mesh — the pilot's object-mask step ran a
    nearest-triangle query on ~400 k pixels at once. Chunked (40 k) for the pilot — both
    scenes then rendered all three views; M3 uses the renderer's id buffer. The new
    ≥ 100 px seam rule fired once on the re-run (edge/line, redrawn from 29 px to 874 px
    of seam on attempt 3). Findings from the sheets:
    * **Beyond the plane is a flat grey dome** in every low-elevation view 0 (T/line,
      grooved butt, corner, edge, lap at 20–30°). Depth there is correctly invalid, but
      the RGB needs a world: a workshop HDRI dome (the user's panorama, or CC0 sets) and
      a 6 m plane. M5 item.
    * **Photo scale is wrong for close-up photos.** All photos are mapped at 1,5 m of
      bench, so a cutting-mat photo taken from 40 cm shows a water bottle larger than the
      workpiece. Domain randomisation tolerates it, but a per-photo span tag (the user
      knows roughly what each photo covers) plus a ±30 % scale jitter is cheap and keeps
      the world plausible. Ask the user for the tags; store in `backgrounds/manifest.json`.
    * **A visible seam can be tiny in pixels**: edge/line's second drawn view passed the
      10 % rule with the seam 29 px long. Added: the best primary seam must also be
      ≥ 100 px in the image (rule in `dataset_plan.md`).
    * Masks are right on all ten strata, including the two parallel seams of the
      rounded-rect stiffener with tacks on both; drawn views are off-centre; all first- or
      second-attempt accepts.
14. **Round 3 — user decisions applied** (`round3/`): backgrounds carry the user's span tags
    (`out/backgrounds/manifest.json`: 1–16 → 0,5 m, 17–25 → 1 m, 26–41 → 1,5 m, ±30 % jitter,
    31–41 flagged generated); the two lab panoramas in `out/background_panorama/` are the
    dome by default (LDR JPEG works as a `DomeLight` texture) — at the T/line tier-1 camera
    (20° elevation) the region beyond the plane is now the lab; titanium added as the seventh
    alloy. Exposure: an LDR dome at intensity 900 over-brightens bare steel — M5 sets exposure
    per dome and draws intensity in a range that keeps the brightest alloy unclipped.
15. **Speed**: ~35 s for three views when the object mask comes cheap, ~120–140 s when
   `trimesh.proximity.closest_point` labels ~70 k pixels against the meshes without
   embree. M3 takes `mask_object` from the semantic annotator instead (free).

## 2. Decisions Phase 8 forces (record in `dataset_plan.md` §10 as they close)

- [x] **Depth encoding.** SCHEMA.md §3.1 says `depth.png (uint16, mm)`. A 1 mm quantum is
      4× the D34 budget and would fail the gate by construction. Standoff in the corpus is
      300–1200 mm, so far corners reach ~1,5 m. **Proposal:** `depth.png` uint16 at
      **0,05 mm per unit** (range 3,28 m, quantisation ±0,025 mm), with `depth.scale_mm:
      0.05` and `depth.invalid_value: 0` in `scene.json`. Not `.npz`: `weldgen verify`
      hashes *every* `*.npz` in the scene directory, so a `depth.npz` would break every
      stored hash. Float32 EXR is the alternative if sub-0,025 mm ever matters (it does not
      for a stereo noise model whose σ at 400 mm is ~1 mm).
      **→ DECIDED 2026-09-11 (user): as proposed — `depth.png` uint16 at 0,05 mm/unit, 0 = no return, `depth_valid.png` alongside.**
- [x] **Hashed or unhashed.** Rendered files are **unhashed** in `scene.sha256` (like
      meshes: convenience artefacts derived from hashed geometry) but the render pass writes
      its own `render.sha256` (sha256 of the depth array + the resolved render config) so a
      re-render can be *compared*, not gated: RTX path-traced RGB is not bit-deterministic
      across drivers; clean depth from the ray cast is, and that is what the gate measures.
      **→ DECIDED 2026-09-11: `scene.sha256` stays tier-1 only (unchanged by rendering). Comparability comes from `render.sha256` = sha256 over every view's depth array + every mask + the resolved render config (`render_id`, background set hash, alloy/light/camera draws). Depth is a ray cast and masks are constructed, so two machines with the same render_id must agree bit-for-bit there; RGB is path-traced and NOT bit-stable across GPUs/drivers, so its hash is recorded per view as informational and compared by tolerance (PSNR), never gated.**
- [x] **Where the render block lives.** `scene.json` already reserves `rgb` and `depth`
      (`null` in tier 1). Filling them changes the content hash (only `provenance` is
      excluded). Precedent: `apply_rule_blocks.py` fills `mps`/`tacks` in place and rewrites
      `scene.sha256` — identity preserved, same corpus. **Do the same**: the render pass
      fills `rgb`, `depth` (file, encoding, scale, size, backend, renderer version,
      `render_id`, material/light draw summary) and rewrites `scene.sha256`; `verify` stays
      green; `scene_id`/`twin_key` untouched. `tier` stays `1` for the cloud; the render
      block carries `"tier": 2`. The tier-1 twin *is* the same directory before the pass.
      **→ DECIDED 2026-09-11 — proposal AMENDED after the user's question "does this break tier-1-only use?": filling `rgb`/`depth` inside `scene.json` would not break tier-1 consumers (extra keys are ignored) but would give a rendered scene a different `scene.sha256` than a freshly generated one, so `generate(config, seed)` on a clean machine would no longer reproduce the released hash — the D9 release-as-a-program argument. Therefore the render block lives in its own **`render.json`** next to `scene.json`; `scene.json` and `scene.sha256` are byte-identical whether or not a scene was rendered, `verify` never sees the renders, and the tier-1 twin is literally the same files. `scene.json`'s reserved `rgb`/`depth` keys stay `null` and SCHEMA 2.4 points them at `render.json`; `index.jsonl` gains a `render_id` column.**
- [x] **Render config and substream.** Materials/lighting parameters come from a separate
      YAML (`configs/render/*.yaml`) with its own `render_id` (first 8 hex of the canonical
      sha256), never from `DEFAULT_CONFIG` — no scene re-ids. Draws come from the reserved
      substream index 7: rename `_reserved7` → `render` in `rng.SUBSTREAMS` (index
      unchanged, `SeedSequence.spawn` children are index-keyed, so no existing draw moves).
      A scene's material/lighting realisation is then a pure function of `(seed, render_id)`.
      **→ DECIDED 2026-09-11 (user): as proposed. `rng.SUBSTREAMS[7]` renamed `_reserved7` → `render` (index unchanged; SeedSequence children are index-keyed, so no existing draw moves — checked by the determinism tests). Render draws are seeded per scene from `sha256(scene_id, render_id)` on top of it, D39-style, so strata sharing a seed index do not share a photo or a camera.**
- [x] **Training masks (user, 2026-09-10).** Yes — and they are *constructed*, never detected
      (the one rule). Per rendered view the writer rasterises, from the truth already on
      disk: `mask_seam.png` (uint8, value = seam id + 1 for weldable seams; the D19 nominal
      curve from `seams.npz`, projected through `K`/`T`, kept only where its `z` agrees with
      the rendered depth within a tolerance — the occlusion test is the render itself);
      `mask_tack.png` (the `tacks` block: `points_mm` + `arclength_mm` + tack length →
      arclength intervals on the seam polyline, rasterised the same way); `mask_object.png`
      (Replicator `semantic_segmentation` on the `weldgen_object_id` primvar, free);
      optionally `seam_dist.png` (distance transform of the seam mask, a friendlier training
      target than a 1-px line). Line width is a versioned rule (`mask_rule-0.1`: N px, or a
      physical width in mm projected at the pixel's depth — prefer physical, it is
      scale-consistent across standoffs). (Superseded, see the verdict below:) tacks in the corpus are a
      *plan* (where tacks go), not geometry — nothing is visible at a tack in RGB or depth.
      A `mask_tack` therefore trains tack *placement* from seam geometry, which is the MPS
      task, not tack *detection*. Visible tack beads would be tier-2-only geometry and
      would break the "differs only in sensor realism" gate; if wanted, it is a separate
      arm (`render.tack_beads: true`) rendered *in addition*, never in the twin.
      **→ DECIDED 2026-09-11 (user): yes, all three masks, exactly as proposed. The "caveat" above was my
      misreading of the task: this project proposes weldable seam areas and tackable spots on bare
      joints — the same task the seven literature methods were scored on in tier 1 — and never
      detects welds already made. Both masks are proposal labels; nothing is drawn on the parts by
      design. Plain-language statement in `dataset_plan.md` Phase 8.**
- [x] **Corpus size and views (user, 2026-09-10).** `bench_phase4` is 720 scenes, not 360
      (T = 360). It is the *benchmark* corpus: 60 per family was chosen for per-stratum
      reporting and the 70 h seven-method batch, and it stays held-out. Training needs its
      own corpus: `out/train_v1` from the same generator with `--per-family 300` (3600
      scenes) and a **different `BASE_SEED`** (add `--base-seed` to `make_bench6b.py`, e.g.
      3 000 000) so no seed overlaps the benchmark. Cost from the `bench_phase4` manifest:
      13,7 s/scene single-process average, but `rounded_rect` is 90 s and `swept_path` 60 s
      per scene, so 3600 scenes ≈ 14 h serial; the builder is single-process, so run one
      process per class (16 cores here) → ~8 h wall, dominated by T/rounded_rect (split it
      by seed residue if that matters). Disk: 2,9 GB per 720 → ~15 GB (+ ~19 GB fixture
      twins) — this is where the NTFS volume (§0) becomes necessary, not optional.
      **Ten views per scene:** keep view 0 pinned to the tier-1 camera (the gate, the twin),
      and draw views 1–9 from the `render` substream with the *existing* camera sampler
      (`camera.sample_pose` + `standoff_for_framing`, same ranges as tier 1) — stored as
      `views/<k>/{rgb,depth,depth_valid,mask_*}.png` with each view's `K`/`T_world_cam` in
      the render block. Extra views have no tier-1 cloud twin; they are training-only
      renders whose labels are still exact (same geometry, same truth). 3600 × 10 = 36 000
      frames at 0,3–1 s ≈ 3–10 h of GPU — an overnight run. Splits (D11) are by `twin_key`,
      never by view, or ten near-duplicates leak across train/val. Honest note: views of
      one geometry are correlated; if the budget is ever tight, more geometries × fewer
      views generalises better than the reverse.
      **→ DECIDED 2026-09-11 (user): 3600 scenes = 12 strata × 300 (1800 T + 1800 non-T), 10 views each, 36 000 frames.**
- [x] **Materials (user, 2026-09-10).** Six-way alloy split as proposed: carbon/mild steel,
      stainless steel, aluminium, cast iron, brass, bronze — drawn per scene from the
      `render` substream (both parts the same alloy; dissimilar-metal joints at a small
      opt-in probability). Two additions argued for: (1) a **surface-condition axis** drawn
      per part — mill scale / ground / rusted / primed-painted / oily — because for a depth
      sensor the roughness and specularity decide dropout, not the alloy; (2) **MDF as a
      seventh "lab" material** used only for the twin of the Phase 9 real subset (the real
      parts are MDF); without it the tier-2 → real ablation has no matching sim material.
      Sourcing: Isaac ships no metal library offline (311 `.mdl` files, only base
      OmniPBR / `physically_metallic_roughness`); NVIDIA vMaterials 2 (free download,
      MDL) has Steel, Stainless, Aluminium, Cast Iron, Brass, Bronze with finishes.
      Recommendation: author the six as **parametric OmniPBR** (albedo, metallic,
      roughness, procedural normal/roughness maps) checked into `configs/render/` — fully
      versioned, no external dependency — and treat vMaterials as an optional RGB-fidelity
      upgrade recorded by name + version in the render block.
      **→ DECIDED 2026-09-11 (user): **seven** alloys, uniform — mild steel, stainless steel, aluminium, cast iron, bronze, brass, titanium. **No painted MDF** material (the MDF board is the *substrate* photo, never a workpiece). Surface-condition axis kept.**
- [x] **The two D16 open items** (`d435i` constants vs the real camera; the 10× sim-noise
      discrepancy in the ROS twin) do **not** gate Phase 8. M1–M3 are on clean depth; M4
      applies each scene's stored `noise_model` as is; the discrepancy lives in
      `realsense_sim_camera_node.py`, not here. They matter for Phase 9's claim that
      `d435i` matches the hardware. If measured constants differ, add a **new profile
      name** (`d435i_measured`) — profile constants sit in the resolved config, so editing
      `d435i` re-ids every corpus. The flat-target afternoon can run in parallel with M5.
      **→ DECIDED 2026-09-11 (user): not gating Phase 8; stay in `dataset_plan.md` §10 for Phase 9.**
- [x] **Two interpreters.** Tier-1 (`python3`, 3.12, pytest) and tier-2 (`/isaac-sim/python.sh`,
      3.11) stay separate. New pure code (`geom.from_object`, the gate maths, the depth
      codec, D16-on-depth) lives in `weldgen/` and is tested under plain pytest with small
      synthetic depth arrays; anything importing `isaacsim`/`omni`/`pxr` lives in
      `weldgen/render/` behind lazy imports and is exercised by an `@pytest.mark.isaac`
      test that skips unless run under `python.sh`. D9 is preserved: `pip install` of the
      core never pulls Isaac.
      **→ DECIDED 2026-09-11: as proposed; in force since M1.**
---

## 3. Milestones, in order

Each milestone ends with a check that can be run cold. Estimated total ≈ 8–10 working days;
the variance is all in M5.

**M1 — `weldgen.geom.from_object` + mesh emission** (½ day, tier-1 only) — **landed 2026-09-11**:
`geom.from_object` (all six primitives, `PRIMITIVES` vocabulary), the `_trapezoid` shim
(numpy 1.26 in Isaac Python meshes prepared prisms and swept slabs from real corpus scenes),
`scripts/emit_meshes.py` with a `--check` round-trip gate, `tests/test_from_object.py`
(11 tests: hand-built parts for all six, plus generated plate / grooved / curved scenes).
- Inverse of `scene._object_entry` / `scene_curved._objects_block` for all six primitives
  (slab, prism, prepared_slab, prepared_prism, tube, swept_slab; `spine` via
  `curves.from_parametric`).
- Test: for each of the 12 strata's first scene, `from_object(o).mesh()` equals the mesh
  from regenerating `(config, seed)` (vertices allclose 1e-9) and `part_geometry_id` matches.
- `scripts/emit_meshes.py <corpus>` → `mesh_<id>.ply` next to existing files (handoff §4
  item 10; unhashed, ids untouched). Run it over `bench_phase4` and `_fx`.

**M2 — USD stage + pinned camera + clean depth + the gate** (1 day; prototype exists)
- `weldgen/render/usd_stage.py` (pure pxr: meshes with `object_id`/`face_id` primvars,
  camera from `K`/`T_world_cam`, one distant light), `weldgen/render/replicator.py`
  (render product, `rgb` + `distance_to_image_plane` + `normals` annotators, step, fetch),
  `weldgen/render/gate.py` (numpy: back-project with pixel centres, residual to meshes,
  frustum+`min_z` coverage vs `visible_from_cam`).
- `scripts/tier2_gate.py <scene_dir>` prints the §1 table; `tests/test_tier2_gate.py`
  checks the maths on a synthetic depth of a known box (plain pytest) and the full run
  under `@pytest.mark.isaac`.
- Check: residual p99 < 0,25 mm and coverage agreement > 0,9 on one scene per stratum
  (12 scenes; the semantic annotator's `object_id` must agree with the nearest tier-1 point).

**M3 — writer, schema fields, `verify` green** (1 day)
- Custom writer emits `rgb.png`, `depth.png` (0,05 mm/unit uint16), `depth_valid.png`,
  fills the `rgb`/`depth` blocks, rewrites `scene.sha256`, writes `render.sha256`.
- SCHEMA.md §3.1 / §4.1 / §6.2 amended; `docs/scene.schema.json` gets the two blocks;
  `dataset_plan.md` §10 entries closed. `python -m weldgen verify` on a rendered class: all pass.
- Twin identity test: rendered scene's geometry keys (`objects`, `seams`, `twin_key`) are
  byte-identical to the pre-render `scene.json`.

**M4 — D16 noise on rendered depth** (½ day)
- Back-project `depth.png` → xyz (+ normals from the `normals` annotator), run
  `noise.apply` with the scene's stored `noise_model` → the tier-2 noisy cloud. Same seed ⇒
  same realisation class as tier 1; report σ_z vs range on both tiers in one cell.
- Check: `noise.apply` on the back-projected clean depth and on the tier-1 `cloud.npz`
  give the same `valid` fraction within 2 % on the 12 gate scenes.

**M5 — materials + lighting substream** (2–3 days; the variance)
- `configs/render/lab_v1.yaml`: material set {painted MDF, stainless, mild steel with mill
  scale, rust} as MDL/OmniPBR with parameter ranges; lighting {dome HDRI or distant + area,
  intensity, colour temperature, elevation/azimuth}. Draws from the `render` substream,
  appended in a fixed order. Never geometry, never pose (handoff §4.4).
- Specular dropout + shadow holes as post-processing informed by rendered normals and the
  material's roughness (an `invalid` mask), documented as a versioned rule.
- Gate re-run under every material: clean depth residual unchanged (materials must not
  move the ray cast); tests for the draw order (appending a material keeps earlier draws).

**M6 — batch render `bench_phase4` + `_fx`** (an evening)
- `scripts/render_tier2.py --corpus out/bench_phase4 --render-config configs/render/lab_v1.yaml
  [--only <class>] [--resume]`, run as `nohup /isaac-sim/python.sh … > log 2>&1 &` with a
  watcher (handoff §2 lessons; never `pkill -f` from a shell containing the pattern).
  One `SimulationApp` for the whole batch; per scene: clear `/World`, rebuild, 1–2 steps,
  write. At 0,1–0,5 s per frame this is minutes of GPU time; stage build and PNG encoding
  dominate. Print the gate residual per scene into the log; abort the batch on the first
  violation.
- Check: `verify` green on all classes; 994 `render.sha256` files; a `facts.csv` column
  `render_id`.

**M7 — `notebooks/16_tier2.ipynb`** (1 day, via `notebooks/build_16_tier2.py`)
- Tier-1 vs tier-2 residual and coverage tables per stratum; noise-on-depth vs noise-on-cloud;
  lit-quadric and lit-lobb on the tier-2 noisy cloud vs tier-1 (the ablation the shared
  schema exists for); material breakdown. Real subset column left empty for Phase 9.

---

## 4. Practical notes for this box

- Always: `cd /tmp && PYTHONUNBUFFERED=1 /isaac-sim/python.sh script.py` — `print` output
  is lost on `app.close()` without unbuffered stdout, and Kit writes a crash dump under
  `/isaac-sim/kit/data/Kit/` on any Python exception after startup (harmless, but noisy).
- Kit logs every stderr line as `[Error] [omni.kit.app._impl] [py stderr]`; grep for the
  script's own markers, not for "Error".
- The first RTX run compiles shaders for Blackwell (~2 min once); subsequent app starts are 9 s.
- Isaac Python numpy is 1.26.0 vs 1.26.4 system — `content_hash` is over bytes of arrays
  the renderer never touches, and `verify` on this box already reproduces the laptop's hashes.
- The 4060 laptop is no longer the development target; both development and batch happen here.
