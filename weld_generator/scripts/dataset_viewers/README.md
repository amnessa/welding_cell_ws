# WeldSet tier-1 benchmark: dataset viewers

This folder holds the tier-1 (analytic) part of WeldSet and two viewers for browsing it.

| folder | scenes | what it is |
|---|---|---|
| `bench_phase4/` | 720 | the benchmark: 12 strata × 60 scenes, no fixture |
| `bench_phase4_fx/` | 274 | fixture twins: plate scenes of `bench_phase4` regenerated on a clamping table |

The 12 strata: T-joints with straight, circular, elliptical, saddle, rounded-rectangle and
curved-strip (arc / B-spline) seams; corner joints; square, grooved (ISO 9692-1 single V / U /
bevel) and curved-strip butt joints; lap joints; edge joints. Joint terms follow ISO 17659,
preparations ISO 9692-1, quality levels ISO 5817. All lengths are in millimetres.

## Running the viewers

Needs Python 3.9 or newer with numpy and matplotlib:

```
pip install numpy matplotlib
python view_bench_phase4.py          # the 720 benchmark scenes
python view_bench_phase4_fx.py       # the 274 fixture twins, each next to its original
```

Run them from this folder (they find the data next to themselves). `--stratum T/circle` opens
one stratum, `--list` prints the catalogue, and `view_bench_phase4.py --save-all pngs/` writes
one picture per scene without opening a window. `pip install open3d` adds a smooth 3D view of
the current scene (key `o`).

In the window: the slider and Prev / Next move through scenes, the left panels choose the
stratum, the input condition and the point colouring, and dragging rotates the 3D view.
Keys: ← → scene, ↑ ↓ stratum, 1 2 3 input condition, c colouring, k camera, s save PNG.

If the window ever misbehaves, the first line the script prints (Python, numpy, matplotlib
versions and the plotting backend) and any message shown in the right-hand panel are what we
need to fix it. Holding an arrow key or dragging the slider skips through scenes and draws only
where you stop.

## What a scene shows

* **Point cloud**, under one of three input conditions: *exterior* (what a perfect multi-view
  scan returns), *camera view* (what the scene's single depth camera sees) and *all samples*
  (every surface sample, including faces buried at the joint).
* **Ground-truth seams**, constructed from the geometry and never annotated. Green: the seams
  the joint type defines as its own. Amber: other weldable seams. Dotted grey: candidates
  rejected for a stated reason (for example `confined_bore`: the torch cannot enter a bore
  under 80 mm; `toe_of_centreline`: the edge of a butt gap, not a seam). *MPS* marks the seam
  the most-promising-seam rule selects. Black crosses are tack welds.
* **Camera view**: the depth image of the scene's camera, with the seams projected onto it.
* **Parameters**: joint term, preparation, quality level, root gap, angle, plate thickness,
  sensor profile, sample density, and a table of every seam candidate.

## Files in each scene folder

| file | contents |
|---|---|
| `scene.json` | every parameter of the scene: parts, faces, joint, camera, sensor noise model, seam list with class / length / weldability / reject reason, tack welds |
| `cloud.npz` | the point cloud: `xyz`, `normals`, `object_id` (0 = part A, 1 = part B, 255 = fixture), `face_id`, and the masks `exterior` and `visible_from_cam` |
| `seams.npz` | each seam as a dense polyline `seam_<i>` (0.1 mm spacing) with per-point tangent, approach direction, face normals and camera visibility |
| `mesh_*.ply` | triangle mesh of each part (and of the fixture) |

The clouds are noise-free. The sensor noise model's parameters are stored in `scene.json`
(`noise_model`). A seam is the intersection of the two joint faces' supporting planes (the line
the weld would follow at zero gap); for butt and edge joints it is the centreline of the gap.
