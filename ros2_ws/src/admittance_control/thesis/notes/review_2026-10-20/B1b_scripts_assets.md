# B1b — Scripts, tools and assets: thesis material map

Reviewer scope: everything in `ros2_ws/src/admittance_control` **except** the library modules in
`admittance_control/admittance_control/` (another reviewer): `scripts/` (all nodes and CLIs, incl.
`scripts/scripts_in_foundationpose/`), `src/*.cpp`, `launch/`, `config/`, `helper/`, `notebooks/`,
`models/`, `test/data/`, `urdf/`, `world/`, plus `IsaacSim-ros_workspaces`,
`Universal_Robots_ROS2_Description`, `.devcontainer`, the FoundationPose Docker set-up.
Draft checked: `thesis/manuscript/chapters/ch3_methodology_cell.tex` (ch3) and
`ch5_experimental_setup.tex` (ch5), state of 2026-10-16.

Conventions. Paths are relative to `ros2_ws/src/admittance_control/` unless absolute.
`file:N` = line N. "Library-side" marks a value that lives in `admittance_control/*.py` (the
other reviewer's scope) and was looked up only to verify a draft statement. Verdicts:
**CONFIRMED**, **WRONG** (correct value given), **PARTLY** (true but incomplete or imprecise),
**NOT VERIFIABLE** (no evidence in the files). Numbers measured on the bench are flagged
*(result → ch6)*.

Method note. Every number below was read from the file it is cited to (code, docstring,
notes). Where a statement rests only on notes (not code), that is said.

---

## 0. Short list of the most important findings

1. **Hand-eye calibration is not done by the draft's AX = XB with a ChArUco board.** The
   ChArUco notebook (`notebooks/camera_calibration_test.ipynb`) exists, but the extrinsic that the
   cell uses is produced by `helper/calibration/extract_extrinsics.py` / `resolve_handeye.py`
   chain and then *re-estimated against the robot* (table touch-off, `refine_extrinsic_from_table.py`,
   `selfcal_extrinsic.py`, the "4 wrist yaws" tilt check in `extrinsic_check.py`). See §C for the exact
   procedure and which file wrote the current `notebooks/T_tcp_to_cam.npy`.
2. **The pose server is more than "SAM2 + PPF + FoundationPose"**: an extent pre-filter rejects CAD
   models by size before any voting; 12 hypotheses *per model* are ICP-polished and scored by
   `coverage × explained` with a visibility test (backface + coarse z-buffer) at τ = 8 mm; the score
   is the only cross-model comparison; FoundationPose then registers from scratch with 5 refiner
   iterations. Details and line numbers in §A.
3. **Tracking**: LOST = render-based fit < 0.5 for 3 frames (fit = fraction of rendered CAD pixels
   whose measured depth is within 10 mm); recovery at ≥ 0.6; re-seed through the desktop with a
   projected CAD silhouette and the *forced* model name (incident 2026-10-09). The laptop tracker
   cannot register (no scorer network).
4. **The camera images are stamped with the ROS clock at publication** (`realsense_camera_node.py`),
   not the sensor exposure time; every "TF at the image stamp" in ch3 (aim loop, tracking ego-motion)
   inherits this offset. The draft's "delivered about 0.1 s later" must be read against this.
5. **Printed parts**: the pen holder / camera mount CAD is `world/penholder_assembly.step`
   (+ `.usda`); its collision envelope and pen tip are in `config/pen_tool.json`. Test workpieces and
   their dimensions are in `models/weldgen_objects.json`, `test/data/bench_models/README.md` and
   `notes/curved_parts_print_list.md` — see §B for the table the draft's TODO asks for.
6. Several draft numbers are confirmed (1.5 N touch, 35 mm approach, 100 mm departure,
   W = diag(2,2,1.5,1,1,0.5), δ0 = 10 mm, κ = 0.15 rad/mm, 125 Hz/500 Hz, cos 60° gate, ±180°
   cable window) and some are wrong or incomplete — consolidated in §D.

(Section numbers: §A pose server, §B printed tool and parts, §C calibration tools, §E per-file entries,
§D missing material and all discrepancies.)

---

## A. The pose server in Docker (`scripts/scripts_in_foundationpose/`)

### A.0 What this folder is, and where it runs

* `README.md:116`: "**Reference copies of the GPU host's code** … They run on the pose-server machine
  (reached over its Tailscale IP), not in the ROS graph; kept here so client and server contracts stay
  in sync. Edit on the host, mirror back."
* The code imports the FoundationPose repository it is copied into: `CODE_DIR` = parent of
  `scripts/` (`fp_server.py:121-122`), then `from estimater import FoundationPose, ScorePredictor,
  PoseRefinePredictor, dr, set_logging_format, set_seed` and `from Utils import draw_posed_3d_box,
  draw_xyz_axis, symmetry_tfs_from_info` (`fp_server.py:125-133`; `fp_tracking.py:33` imports
  `nvdiffrast_render`). Paths it expects but which are **not in this workspace**: `Data/Input/*.ply`
  (the CAD library), `Data/ppf_library.npz`, `weights/sam2/sam2.1_hiera_small.pt`,
  `Data/Output/foundationpose_results/`, `Data/Output/timings.csv` (`fp_server.py:147-167`).
* **Docker**: no Dockerfile or run script exists anywhere on this machine (searched `/` for
  `*run_container*`, `docker-compose*`, `*foundationpose*`, `estimater.py`). They live in the
  FoundationPose repository on the GPU hosts. What the docs say:
  - desktop: `docker/run_container_blackwell.sh` — "X11 forwarded, workspace bind-mounted"
    (`notes/readme_perception.md:663`; `fp_server.py:691` names the same script);
  - laptop: `docker build -f docker/Dockerfile.blackwell --build-arg TORCH_CUDA_ARCH_LIST=8.9 -t
    fp-sam2:ada .` then `docker/run_container.sh` (`notes/readme_perception.md:678-679`,
    `notes/perception_aciklama.md:929-930`); `run_container.sh` sets `REGISTER_CROP=1`
    (`fp_server.py:89-91`) and passes `HOST_UID/HOST_GID` so files written as root are chowned back
    (`fp_server.py:174-199`).
  - "Both machines run the same Docker image" (`notes/thesis_notes.md:508`).
  - Network: desktop reached over **Tailscale** (`README.md:116,149`; `notes/realtime_fp.md:70`);
    "ODTÜ ağında Tailscale engelli" — Tailscale is blocked on the METU network, registration needs a
    VPN (`notes/admitans kontrol açıklama.md:907`).
* Hardware (evidence in notes only, not in code): laptop "camera, ROS 2 Jazzy, RTX 4060", desktop
  "RTX 5070 Ti" (`notes/readme_perception.md:414`, `notes/perception_aciklama.md:791`); the 8 GB of
  the RTX 4060 is in code comments (`fp_server.py:89-91`, `fp_tracking.py:57-58`,
  `fp_track_server.py:1`).

| file | role | status |
|---|---|---|
| `fp_server.py` (1117 l.) | registration server: Flask `:5000` (`/predict_pose`, `/classify`, `/add_model`, `/health`) + optional tracking WebSocket `:5001` | active, desktop |
| `ppf_classifier.py` (743) | PPF CAD classifier (OpenCV `ppf_match_3d`) + verification score | active (imported by `fp_server.py:136-142`) |
| `build_ppf_library.py` (144) | offline CLI: CAD dir → `ppf_library.npz` | tool |
| `ppf_selftest.py` (133) | synthetic confusion matrix of the library | tool |
| `fp_tracking.py` (487) | `Tracker` state machine + `TrackingServer` (WebSocket) + `register_cropped` | active |
| `fp_track_server.py` (117) | tracking-only server for the laptop (refiner only) | active, laptop container |
| `fp_stream.py` (357) | wire protocol; **byte-identical** to `admittance_control/fp_stream.py` (`diff` empty; `test/test_fp_stream_copy.py`) | active |
| `track_bench.py` (203) | GPU-only tracking benchmark ("Step 0 of realtime_fp.md") | benchmark |
| `track_replay.py` (386) | ROS-free simulated tracking client (motion, blackout, forced re-seed) | benchmark |
| `server.py` (193) | the superseded SAM-6D Flask server (`demo.sh`) | legacy |
| `foundationpose_bridge_node.py` (482) | **older** copy of `scripts/foundationpose_bridge_node.py` (773): lacks the capture/click/`stamp_ns` flow and the `assembly_mesh.ply` default upload (`diff`: 439 lines) | stale copy |
| `sam6d_bridge_node.py` (272) | identical to `scripts/sam6d_bridge_node.py` (`diff` empty) | legacy copy |

### A.1 Server configuration (environment variables, `fp_server.py:67-101, 150-172`)

| variable | default | meaning |
|---|---|---|
| `MESH_PATH` | `Data/Input/test_objv2_ear.ply` | fallback CAD (PPF off or failed) |
| `MESH_SCALE` | `0.001` | mm → m applied to every mesh on load |
| `PPF_ENABLE` | `1` | classify; `0` pins the server to `MESH_PATH` |
| `CAD_DIR` | `Data/Input` | library .ply directory |
| `PPF_LIBRARY` | `Data/ppf_library.npz` | library file (auto-built if missing, `fp_server.py:403-433`) |
| `PPF_MIN_MARGIN` | `0.0` | top-minus-runner-up gap below which the answer is "ambiguous" (0 ⇒ never) |
| `PPF_STRICT` | `0` | refuse (HTTP 422) instead of guessing / falling back |
| `PPF_TAU` | `0.008` m | verification inlier radius ("your depth noise") |
| `SYMMETRY_INFO` | `""` | BOP `models_info` symmetries → `symmetry_tfs`; **unset by default** |
| `EST_REFINE_ITER` | `5` | refiner iterations in `register()` |
| `REGISTER_CROP` | `0` | register on the network's ROI (needed on 8 GB; `run_container.sh` sets 1) |
| `ZFAR` | `3.0` m | depth beyond is discarded |
| `SAM2_CKPT` / `SAM2_CFG` | `weights/sam2/sam2.1_hiera_small.pt` / `configs/sam2.1/sam2.1_hiera_s.yaml` | SAM 2.1 Hiera-Small |
| `TIMING_CSV` | `Data/Output/timings.csv` | per-stage timings (long format) |
| `PORT` | `5000` | Flask |
| `TRACK_ENABLE` / `TRACK_PORT` | `1` / `5001` | in-process tracking WebSocket |

Start-up (`fp_server.py:1076-1109`): `set_seed(0)`; load one `FoundationPose` with `ScorePredictor`,
`PoseRefinePredictor`, `dr.RasterizeCudaContext()` on `MESH_PATH` (`load_estimator`, 359-387); load
the PPF library and train every detector; load SAM2; optionally start the tracking server.
Flask runs with `threaded=False` (`fp_server.py:1117`) — requests are serial on the main thread (the
OpenCV click window needs it); a non-blocking `_lock` returns HTTP 503 "busy" to a second request
(`fp_server.py:181-183, 931-933`) and a blocking `GPU_LOCK` serialises every use of the estimator
between registration and the tracking thread (`fp_server.py:185-187, 991-1011`).

### A.2 `/predict_pose`, from request to returned pose (step by step)

0. **Client side** (laptop, `scripts/foundationpose_bridge_node.py`): `~/capture` freezes one
   synchronised RGB-D pair (|Δstamp| ≤ `max_sync_delta_sec` = 0.25 s, `:275, 524-530`), writes
   `rgb.png` / `depth.png` / `camera.json` to `scripts/rgb_depth_to_send/` and republishes the frame
   latched on `/perception/frozen_rgb` (`:532-583`). The gesture node publishes the operator's points
   on `/perception/sam2_click` as `{"stamp_ns", "points", "labels"}`; a click whose `stamp_ns` differs
   from the frozen frame is ignored (`:426-445`), one older than `click_max_age_sec` = 120 s is dropped
   (`:262, 585-608`). `~/trigger` POSTs multipart `rgb` (PNG, 8-bit), `depth` (16-bit PNG, mm,
   aligned to colour), `camera` (`{"cam_K":[9], "depth_scale":1.0}` — written from the live colour
   intrinsics by `realsense_camera_node.py:164-178`) and, if held, `click` + `click_labels`
   (`:610-643`; timeout `request_timeout_sec` = 300 s, `:274`).
1. **Receive and decode** (`_receive_frame`, `fp_server.py:781-805`): files saved to `Data/Input/`;
   depth → float32 metres = raw × `depth_scale` / 1000, zeroed outside [0.001 m, `ZFAR`) and
   non-finite (`read_depth`, 526-534); RGB converted BGR→RGB (518-523); RGB and depth must have the
   same size ("the depth must be registered to the colour frame", 802-804).
2. **Mask** (`resolve_mask`, 664-701), in order of preference:
   (i) an uploaded `mask` file (used by the tracker's re-seed);
   (ii) client `click` points — validated by `parse_click` (621-661): labels parallel to points,
   **at least one positive** ("negatives only carve, they cannot start a mask"), every point inside the
   frame (else HTTP 400: "the client latched the point against a different image");
   (iii) an interactive OpenCV window on the server (left = object, right = not-object, u = undo,
   r = reset, Enter, Esc; mask recomputed after every click, 550-618) — only if `DISPLAY` is set.
   **SAM2 prompt** (`sam2_mask`, 539-547): `predictor.set_image(rgb)`;
   `predictor.predict(point_coords=points, point_labels=labels, multimask_output=(len(points)==1))`
   — one click is ambiguous so SAM2 proposes three masks; the mask with the highest predicted score
   is taken. Empty mask → 400; no valid depth inside the mask → 400 ("out of the depth range, or
   beyond ZFAR", 939-946).
3. **Which CAD** (948-987):
   * if the request carries `object_name` (form field or file), PPF is **skipped** and that CAD is
     registered — "A re-registration (fp_tracker_node on LOST) already knows the part: register that
     CAD and skip PPF, which reclassified the projected mask and answered the plate under the part
     (2026-10-09, C1 -> test_objv2_ear)" (952-964);
   * else `classify_object` (445-495) → PPF classifier (§A.3). No answer → HTTP 422 if `PPF_STRICT`,
     otherwise fall back to `MESH_PATH` with a warning (967-979); ambiguous + strict → 422 (980-987).
4. **Registration** (989-1011): wait for `GPU_LOCK` (stage "wait for GPU (tracking)"); `use_cad`
   (390-400) swaps the CAD into the one resident estimator with `EST.reset_object(model_pts,
   model_normals, mesh)` — networks are loaded once, only the mesh changes; meshes are cached and
   scaled to metres (329-356). Then
   `EST.register(K=K, rgb=rgb, depth=depth, ob_mask=mask, iteration=EST_REFINE_ITER)` with
   `EST_REFINE_ITER = 5` (1000-1006), or `register_cropped` when `REGISTER_CROP=1`
   (`fp_tracking.py:55-66`: crop to `fs.register_roi`, a square of radius f·diameter·1.2/2/z_min around
   the mask plus 40 px, principal point shifted by `fs.shift_K`; "a full 1280x720 register does not
   fit the 8 GB RTX 4060"; `fp_stream.py:228-251` says the scorer's full-frame warps for "~250
   hypotheses" need 2.6 GB). What `register()` does internally is FoundationPose's own code (not in
   this workspace); per the paper (Wen et al. 2024, §3.3): translation initialised at the median depth
   inside the detection, rotations from N_s icosphere viewpoints × N_i in-plane rotations, each
   hypothesis refined by the render-and-compare refiner (`iteration` passes, pose-conditioned crop),
   then ranked by the hierarchical scorer. The notes give "~250 hypotheses"
   (`notes/thesis_notes.md:470`). Symmetries are **not** used unless `SYMMETRY_INFO` is set (default
   empty, 152, 368-371). If an in-process tracking session is active, `TRACKER.on_registered()` makes
   the new pose its starting point (1008-1009; `fp_tracking.py:207-216`).
5. **Score** (1013-1017): `EST.scores[0]`, the scorer's top ranking score — "unnormalized … (tens,
   not 0-1) … NOT a probability like SAM-6D's score". (Example on disk:
   `scripts/foundationpose_results/detection_pem.json` has `"score": 69.0`.)
6. **Artefacts** (1018-1040, `sam6d_style_artifacts` 719-754), written to
   `Data/Output/foundationpose_results/` and base64-encoded in the reply: `detection_pem.json` (SAM-6D
   format: `R`, `t` **in millimetres**, `bbox`, `score`, extra `obj_name`), `detection_ism.npz`
   (`segmentation` (1,H,W) bool — the mask the laptop ICP node crops its scene with), `mask.png`,
   `vis_pose.png` (oriented 3-D box + axes drawn at `pose·inv(to_origin)`, 706-716),
   `object_name.txt`.
7. **Reply** (1045-1061): `status`, `elapsed_sec`, `timings` (every stage CUDA-synchronised before
   the clock is read, `StageTimer` 204-321; operator click time reported apart), `units: "m"`,
   `pose` = 4×4 **T_cam_obj in metres, camera optical frame of that image, CAD frame**, `score`,
   `object_name`, `object_file`, `classification` (full PPF report), `mask_score`, `mask_source`,
   `artifacts`.
8. **Back on the laptop**: the bridge saves the artefacts to `results_dir`
   (`scripts/foundationpose_results/`), drops the frozen frame, publishes a
   `vision_msgs/Detection3DArray` **in the camera frame, stamped with the frozen image's stamp**,
   `class_id` = the PPF name (`use_ppf_name_as_class_id` = True), and the name latched on
   `/perception/object_name` (`foundationpose_bridge_node.py:660-689, 714-757`). A reply below
   `min_score` (default 0.0) keeps the frozen frame for a retry (`:665-671`). The ICP node and
   `fp_tracker_node` consume it (§E.1).

Other endpoints. `/classify` = steps 1-3 only, returns the whole score table "in about a second"
(808-848). `/add_model` uploads a .ply, indexes it incrementally (each model owns its detector),
persists the library, returns `diameter_m`, `extents_m`, `scale_warning` and `similar_diameter`
(other models within 10 % in diameter) (851-926); the bridge's `~/add_model` uploads
`<results_dir>/assembly_mesh.ply` (the exported assembly) by default
(`foundationpose_bridge_node.py:59-71`). `/health` reports the loaded state (759-778).

### A.3 The PPF classifier (`ppf_classifier.py`)

Design statement (docstring 1-47): the classifier returns **a name and nothing else**; the PPF pose
is "scaffolding for the score, not an output", FoundationPose re-estimates the pose from scratch
(`fp_server.py:452-455, 949-951`). Votes rank hypotheses only *within* a model ("vote counts are not
comparable across models": peak height scales with model point count and self-similarity); identity
is decided by a normalised two-sided score, "structurally Birdal & Ilic's Eq. 11 (3DV 2015) with the
occlusion discount replaced by an explicit visibility test". Kept from Birdal: model representation
(§2.1), voting (§2.2.1, inside OpenCV), hypothesis verification (§2.2.4); dropped: the coarse-to-fine
segmentation (§2.2.2) and pose clustering (§2.2.3), redundant after SAM2.

**Library (offline / start-up).** Per `.ply` in `CAD_DIR` (`index_ply`, 318-332):
1. load, scale by `mesh_scale` = 0.001 (73);
2. scale sanity check: diameter after scaling must be 10 mm–1.5 m (`PLAUSIBLE_DIAMETER_M`, 146,
   156-184) and within ×5 of the library median (`scale_outliers`, 283-314) — FreeCAD's export unit is
   per document;
3. surface sampling (`sample_model_cloud`, 187-230): `trimesh.sample.sample_surface` with
   max(60 000, 60·n) points, exact face normals, then voxel thinning with the voxel size bisected
   (≤ 40 steps) until ≈ `n_model_points` = 600 points, keeping one *representative* per voxel (not an
   average: averaging would round box edges and create 45° normals) — "which is why none of the MLS
   normal smoothing in the paper's §2.1.2 is needed";
4. robust diameter = 99.5th percentile of pairwise distances of ≤ 1200 sampled vertices
   (`cloud_diameter`, 233-243) and AABB extents;
5. save `.npz` (format 1: clouds, offsets, names, diameters, extents, per-model sampling steps,
   params JSON) — **not** the hashtables: "OpenCV cannot serialize a trained PPF3DDetector" (398-427);
6. train at start-up, ~1-2 s per model: `cv2.ppf_match_3d_PPF3DDetector(relative_sampling_step =
   0.04 (per-model override, e.g. 0.025 for thin curved parts), relative_distance_step = 0.04,
   num_angles = 30)` then `trainModel(cloud)` (81-83, 371-379).

**Query (`classify`, 455-560; `QueryParams` 90-138).**
1. Scene cloud (`scene_cloud_from_mask`, 674-735): erode the SAM2 mask by 2 px (5×5 kernel; skipped
   if < 50 px would remain) to remove mixed-depth silhouette pixels; dilate by 4 px for normal
   estimation; back-project with K; normals by `cv2.ppf_match_3d.computeNormalsPC3d(pts, 12, True,
   (0,0,0))` — 12 neighbours, oriented toward the camera (CAD normals point outward; the angle features
   are sign-sensitive); keep only the eroded-mask points; voxel thinning with voxel =
   max(diameter/60, 1.5 mm); at most 2000 points. Fewer than 20 points → no answer
   (`fp_server.py:466-468`).
2. Subsample to `max_scene_points` = 2000, build a KD-tree, robust scene diameter (474-479).
3. **Extent pre-filter** (482-508): reject a model if scene_diameter/model_diameter > 1 + 0.15
   (`extent_over`) or < 1 − 0.55 (`extent_under`); if every model is rejected, the filter is ignored.
   Dated reasoning in the docstring: "On the first real frame tested, extent_over=0.30 let a 206mm CAD
   compete against a 255mm segment … and it won on score, giving the wrong answer. At 0.15 it is
   rejected outright and the correct model wins by 0.438" (127-130).
4. **Voting** (571): `detector.match(scene, relativeSceneSampleStep = 1/5, relativeSceneDistance =
   0.04)` — OpenCV's implementation of Drost et al. 2010: point-pair features
   F = (‖d‖, ∠(n1,d), ∠(n2,d), ∠(n1,n2)) quantised (distance step 0.04 × the model's own diameter,
   30 angle bins), every 5th scene point as reference, Hough voting over (model reference point,
   rotation angle). Poses come back sorted by votes.
5. **Hypotheses**: the top `n_hypotheses` = **12 per model** (98, 581).
6. **ICP polish** of each (`_icp_refine`, 598-616): `cv2.ppf_match_3d_ICP(iterations=50,
   tolerence=0.01, rejectionScale=2.5, numLevels=8)` (515) on the *posed* model, correction composed on
   the left; identical for every candidate, so it cannot bias the ranking.
7. **Verification** (`_verification_score`, 656-669): visible model points = backface test
   n·(c − p) > 0 with the camera at the origin, plus a coarse z-buffer with cells ≈ τ wide at the
   object's depth (cell = f_x·τ/median z pixels; keep points within τ of the nearest in their cell)
   (619-653); < 4 visible ⇒ 0.
   `explained` = fraction of visible model points with a scene point within τ;
   `coverage` = fraction of scene points within τ of a visible model point;
   **score = coverage × explained**, τ = `tau_m` = 0.008 m (108; `PPF_TAU`). "Coverage alone lets a
   big model win by blanketing the segment; explained alone lets a small model win by hiding inside
   it" (27-28). A model's score is the best over its 12 hypotheses (581-590).
8. **Decision** (526-560): sort by score; winner = top if score > 0; `margin` = top − runner-up;
   `ambiguous` = margin < `min_margin` (default 0 ⇒ never); the full table, per-model detail, rejected
   models and timings are returned.

Tools. `build_ppf_library.py` (CLI: `--cad-dir`, `--out`, `--add PLY` (repeatable),
`--sampling-step`, `--mesh-scale` 0.001, `--model-points` 600, `--relative-sampling-step` 0.04,
`--num-angles` 30, `--no-train`; prints extents in mm, scale warnings and "models within 10% of each
other in diameter — the extent pre-filter cannot separate these", 44-141). `ppf_selftest.py`
(synthetic confusion matrix: random rotation, backface culling, Gaussian noise; defaults `--views 3`,
`--noise 0.0015` m, `--distance 0.8` m, K = [[600,0,320],[0,600,240]]; flags self-scores < 0.25 as
needing a finer sampling step; "Treat the numbers here as an upper bound" because synthetic views carry
exact normals, 1-24, 49-58, 85-93).

### A.4 Live tracking (`fp_tracking.py`, `fp_track_server.py`, `fp_stream.py`; client `scripts/fp_tracker_node.py`)

Deployed split (`fp_track_server.py:1-21`; `notes/realtime_fp.md:8-14`; `notes/thesis_notes.md:466-509`,
entry **2026-10-06**): desktop registers (`fp_server.py`), laptop tracks in its own container
(`fp_track_server.py`, refiner only — `_NoScorer` stands in for the scorer, "~300 MB", the README
measures "~250 MB"), on `ws://127.0.0.1:5001` (`TRACK_HOST` 127.0.0.1, `TRACK_PORT` 5001).
`fp_server.py` can also track in-process (`TRACK_ENABLE=1`), used when one machine plays both roles.

1. **Seed** (`fp_tracker_node.py:305-348`): every new bridge detection (auto_start = True; a latched
   detection older than `max_seed_age_sec` = 30 s within `latched_grace_sec` = 3 s of start-up is
   ignored) → `T_cam_obj`, `class_id` (the CAD name), and `T_base_cam` from TF **at the detection's
   stamp** → text message `{"type":"start","seed":"pose","pose":[16],"object":name,"T_base_cam":[16],
   "refine_iter":2,"ego_motion":true,"fit_tol_m":0.010,"fit_lost":0.5,"fit_recover":0.6,
   "lost_frames":3}`. Server: `use_cad(name)` → `reset_object` (`fp_track_server.py:84-94`); the CAD-frame
   pose is converted to FoundationPose's centred-mesh frame, `pose·inv(tf_to_centered)`
   (`fp_tracking.py:163-198`).
2. **Streaming** (`fp_tracker_node.py:429-497`): RGB/depth paired by stamp (|Δ| ≤ 0.005 s), `T_base_cam`
   looked up **at each image stamp** (TF timeout 0.05 s; without TF it tracks without ego-motion,
   `:294-303`), binary frame = `"FPT1"` | uint32 header length | header JSON (`seq`, `stamp`, `t_sent`,
   full-frame `K`, `full_hw`, `roi`, rgb/depth encodings and lengths, `depth.scale`, `T_base_cam`,
   `want_reseed_mask`) | RGB bytes | depth bytes (`fp_stream.py:7-53, 120-145`). Defaults on the laptop:
   raw RGB and raw uint16 depth ("encoding JPEG/PNG costs ~20-30 ms of delay for nothing (measured
   ~100 ms -> ~75 ms)", `fp_tracker_node.py:183-188`), full frame (`use_roi` False), at most
   `max_in_flight` = 2 frames outstanding; the server keeps a **latest-wins slot** (a slow GPU drops
   frames instead of queueing, `fp_stream.py:282-320`, `fp_tracking.py:407-471`).
3. **Per frame** (`Tracker.step`, `fp_tracking.py:220-280`): depth → metres, zero outside
   (0.001, ZFAR); **ego-motion prediction** `pose ← inv(T_base_cam_now)·T_base_cam_prev·pose`
   (`fp_stream.predict_ego`, 271-280) — the frame chain of ch3 Eq. (frame_chain) at the two image
   stamps; `est.track_one(rgb, depth, K, iteration = refine_iter = 2)`; **fit** = fraction of the
   CAD's rendered pixels (nvdiffrast) whose measured depth is within `fit_tol_m` = 10 mm, plus
   coverage = rendered pixels with any depth (`_fit`, 148-159).
4. **Lost-track logic**: fit < `fit_lost` = 0.5 for `lost_frames` = 3 consecutive frames → LOST
   (258-266). While LOST (`_recover`, 298-344): a cheap `track_one` from the last good pose
   (ego-predicted); leave LOST only when fit ≥ `fit_recover` = 0.6 (hysteresis); otherwise stay
   anchored to the last good pose. In-process server (`can_register=True`): at most every
   `reseed_interval_s` = 1.0 s, `register()` on a mask projected from the last good pose. **Laptop
   server (`can_register=False`)**: the client sets `want_reseed_mask` at most every
   `reseed_interval_sec` = 3.0 s (`fp_tracker_node.py:198-199, 466-479`); the server replies with the
   CAD silhouette at the last good pose, dilated by 0.25 × the crop radius (r = f·1.2·diameter/2/z)
   (`fp_tracking.py:282-296, 346-353`); the client POSTs that frame + mask + **`object_name` = the
   tracked part** to the desktop `/predict_pose` (`fp_stream.post_predict_pose`, 323-357; timeout
   `request_timeout_sec` = 60 s) and **refuses an answer naming another part** ("2026-10-09: C1 ->
   test_objv2_ear, E2 -> ear -> base", `fp_tracker_node.py:590-601`), then sends a new `start`.
5. **Output into ROS** (`fp_tracker_node.py:1-35, 612-660`): `/perception/fp/pose`
   (`geometry_msgs/PoseStamped`, camera optical frame, stamped with the image stamp), TF
   `camera_color_optical_frame → fp_object`, `/perception/fp/status` (JSON, 2 Hz: state, rate, delay,
   fit), `/perception/fp/marker` (latched mesh `package://admittance_control/models/{object}.ply`, scale
   0.001). Services `~/start`, `~/stop`, `~/reseed` (std_srvs/Trigger). The ICP node with
   `tracking_source = fp` takes these poses into `base_link` with TF at each pose's own stamp and
   averages them at rest (`notes/realtime_fp.md:36-46`; see §E.1).
6. **Measured** *(result → ch6)*: laptop, a stand-in camera publishing a saved frame at 30 Hz, **both
   servers on the same RTX 4060**: pose rate 25-29 Hz (ICP tracking ~10 Hz on the CPU), image → pose
   70-90 ms raw / ~100 ms JPEG-PNG, blanked image 1.5 s → LOST after 3 frames, recovered by itself
   (fit 0.90), re-registration 3.9-4.0 s; tracking fell to ~8 Hz during a registration on the same GPU
   (`notes/readme_perception.md:495-519`; `notes/realtime_fp.md:26-34`). `track_bench.py` GPU-only,
   200 frames, still part: 1 pass 28.3 ms (35 Hz), 2 passes 43.2 ms (23 Hz), jitter 0.5 mm /
   0.16-0.19° (`notes/readme_perception.md:497-505`). A warm `/predict_pose` on the RTX 5070 Ti with 14
   CADs: SAM2 0.053 s, register 1.198 s (5 iterations), total 4.506 s (`notes/readme_perception.md:781-797`).
   `realtime_fp.md:48-51` (2026-10-06) still lists "run on the real camera and arm" as to-do — the
   draft should not claim real-arm rates unless measured later.

Benchmark CLIs. `track_bench.py` (`--n` 300, `--warmup` 20, `--iters 1 2`, `--margin` 40,
`--register-iter` 5, `--mesh-scale` 0.001; registers once on the saved frame, then runs `Tracker.step`
N times on full frame and ROI; reports ms/frame mean/p50/p95, peak GPU memory, fit, jitter, payload
sizes). `track_replay.py` (ROS-free client; `--rate` 30 Hz, `--seconds` 15, `--max-in-flight` 2,
`--refine-iter` 2, `--reseed-interval` 3.0, `--wobble`, `--blackout START SECONDS`,
`--force-reseed-at`; seeds `register` | `pose` | `last`).

### A.5 The superseded SAM-6D server (`server.py`)

Flask `/predict_pose` that ran the whole SAM-6D pipeline as a subprocess (`bash demo.sh`: template
rendering → instance segmentation (ISM) → pose estimation (PEM)) on every request, with a CAD fixed
by `CAD_PATH` (default `test_objv2_base.ply`) and `SEGMENTOR_MODEL` = `fastsam` by default ("~2-3 GB
VRAM … so the pipeline fits on an 8 GB GPU"; `sam` = SAM ViT-H ~7-8 GB), returning SAM-6D detections
with translations in mm (`server.py:16-28, 99-178`). The notes give 1-1.5 min per request
(`notes/perception_aciklama.md:974`). Minor code/comment mismatch: comment says `threaded=False`, code
runs `threaded=True` (`server.py:191-193`). Thesis use: one sentence on why the cell moved from SAM-6D
to SAM2 + PPF + FoundationPose (no CAD identity needed in advance, resident networks, seconds instead
of a minute).

### A.6 Fact check of the draft (§3.5.1, §3.5.2, Table tab:hardware)

| draft statement | verdict | evidence / correction |
|---|---|---|
| "The clicked points (positive and negative) and the frozen colour image go to SAM2" | CONFIRMED | `fp_server.py:539-547, 679-686`; bridge ties points to the frozen frame by `stamp_ns` (`foundationpose_bridge_node.py:426-445`). Add: at least one positive point required; a single click uses SAM2's multimask output and the best-scored mask. |
| "which returns one mask per part" | CONFIRMED | one request per part, one mask (`sam2_mask` returns the argmax mask). |
| "the masked depth becomes the part's point cloud" | PARTLY | On the server the eroded masked depth becomes the ≤ 2000-point *classification* cloud (`ppf_classifier.py:674-735`); FoundationPose receives the **whole** depth image plus the mask (`fp_server.py:1005`); the laptop's ICP node crops the scene with the returned mask (`detection_ism.npz`). Say which cloud is meant. |
| Eq. (ppf), PPF feature [Drost; Birdal] | CONFIRMED (as OpenCV's feature) | the code calls `cv2.ppf_match_3d_PPF3DDetector` (`ppf_classifier.py:373-376`); the feature is computed inside OpenCV, not in this code. Quantisation: distance step 0.04 × model diameter, 30 angle bins. |
| "voted against every model of the library" | PARTLY | only models surviving the extent pre-filter (scene/model diameter in [0.45, 1.15]) are voted (`ppf_classifier.py:482-508`). |
| "the best hypotheses are verified by how much of the cloud they explain and how much of the model they cover" | CONFIRMED | `ppf_classifier.py:656-669`. Note the code's naming: `coverage` = fraction of *scene* points explained by the model; `explained` = fraction of *visible model* points supported by the scene. Define both in the thesis; mention the visibility test and τ = 8 mm. |
| TODO "12 hypotheses" | CONFIRMED, add "per model, each ICP-polished (50 it.)" | `ppf_classifier.py:98, 515, 581-584`. |
| TODO "forcing the model name on a re-registration" | CONFIRMED | `fp_server.py:952-964`; client `fp_tracker_node.py:586-601`; incident 2026-10-09. |
| "FoundationPose registers the CAD model to the masked RGB-D frame (pose hypotheses, render-and-compare refinement, scoring)" | CONFIRMED | `fp_server.py:1005-1006`; give `iteration = 5` (`EST_REFINE_ITER`), SAM2.1 Hiera-S, the mesh in metres, no symmetry info. |
| "then tracks it frame to frame while the operator positions the part" | CONFIRMED | `track_one`, 2 refiner passes per frame (`fp_tracking.py:250-251`; `fp_tracker_node.py:182`). Default `tracking_source:=fp` since 2026-10-06 (`launch/pointcloud.launch.py:396-401`). |
| "Registration runs on the desktop GPU, tracking on the laptop's" | CONFIRMED (deployed configuration) | `fp_track_server.py:1-6`; `notes/realtime_fp.md:8-14`. Nuance: `fp_server.py` can track too (TRACK_ENABLE=1) and can register on 8 GB with REGISTER_CROP=1. |
| "the camera's own motion is cancelled by looking up Eq. (frame_chain) at each image's time stamp" | CONFIRMED, with a caveat | `fp_tracker_node.py:294-303, 461`; `fp_stream.py:271-280`. Caveat: the "image stamp" is the ROS clock when `realsense_camera_node.py` publishes (`:205-206`), not the exposure time. |
| TODO "Lost-track detection and re-registration locked to the tracked model's name" | CONFIRMED | values in §A.4 step 4. |
| TODO "why FoundationPose tracking replaced the ICP tracker (moving parts, CPU load)" | CONFIRMED in notes | `notes/thesis_notes.md:529-541` (2026-10-02: "ICP runs on the CPU. Each call is seconds of nearest-neighbour search … far too slow to follow a part that moves"; Kalman fusion dropped) and 2026-10-06 numbers (25-29 Hz vs ~10 Hz). |
| fig:tracking_split "data flow, rates and latencies" | content available | rates/latencies above were measured with a stand-in camera and both servers on the laptop GPU; no real-arm measurement recorded. |
| ch5: Desktop "GPU (RTX 5070 Ti) — registration (SAM2, PPF, FoundationPose)" | CONFIRMED (notes only) | `notes/readme_perception.md:414, 781`. |
| ch5: Laptop "GPU (RTX 4060, 8 GB) — ROS 2, tracking, refinement, planning" | CONFIRMED | `fp_track_server.py:1`; `fp_server.py:89-91`; `notes/readme_perception.md:414`. |
| ch5 §5.2 "the pose server (SAM2, PPF, FoundationPose) in Docker on the desktop" | CONFIRMED, incomplete | add: the laptop runs the same image for tracking; Docker files are in the FoundationPose repo (not in this workspace); Tailscale/VPN link; Flask HTTP for registration, WebSocket for tracking. |

### A.7 What the draft should add about the pose server

* A paragraph (§3.5.1) on the **extent pre-filter** and the **scale checks**: they decide which models
  are voted at all; the dated 206 mm vs 255 mm example motivates `extent_over` = 0.15.
* The **verification score** with its visibility test (backface + coarse z-buffer), τ = 8 mm, and why
  votes are not compared across models (one paragraph + one equation:
  s = coverage · explained).
* The **classification cloud** preparation (mask erosion 2 px, normals on a 4-px-dilated crop,
  oriented to the camera, voxel = max(D/60, 1.5 mm), ≤ 2000 points) — it determines PPF's success
  on thin parts.
* **Library persistence**: clouds stored, hashtables retrained at start-up (OpenCV limitation);
  `/add_model` lets an exported assembly become a classifiable model at run time (link to
  `assembly_mesh.py`/`export_mesh`).
* FoundationPose **settings actually used**: 5 refinement iterations at registration, 2 per tracked
  frame, SAM 2.1 Hiera-Small, meshes in metres, untextured CAD (flat grey, `fp_server.py:343-346`), no
  symmetry information, `REGISTER_CROP` on 8 GB.
* The **two-process split and its transport** (§3.10 or ch5): HTTP multipart for one frame per
  registration, WebSocket binary frames for tracking, latest-wins dropping, 2 frames in flight, raw
  encoding on localhost.
* **Lost-track detection** (render-and-compare fit) and the re-seed with projected silhouette + forced
  name, with the 2026-10-09 incident as motivation.
* ch5 §5.2 table: FoundationPose (Wen et al. 2024) in Docker image `fp-sam2` (desktop Blackwell build,
  laptop `fp-sam2:ada` with `TORCH_CUDA_ARCH_LIST=8.9`), SAM 2.1, OpenCV `ppf_match_3d`
  (opencv-contrib), Flask, `websockets` (server) / `websocket-client` (client).
* One sentence on the SAM-6D predecessor (`server.py`) and why it was replaced.

---
## B. The printed tool and the test parts

### B.1 Pen holder / camera mount on the flange

**CAD files.**
* `world/penholder_assembly.step` — FreeCAD export ("FreeCAD Model", Open CASCADE STEP processor 7.8,
  `FILE_NAME` timestamp **2026-07-04T16:45:33**), AP214. Products: `penholder_assembly`, `Body`,
  `Fillet001`, `chuck` (STEP header and `PRODUCT(...)` records). This is the only CAD of the tool in the
  workspace; README calls it "the pen and camera holder" (`README.md:131`).
* `world/penholder_assembly.usda` — the same assembly converted for Isaac Sim (creator
  `omni.kit.converter.hoops_core 509.1.0`, `metersPerUnit = 0.001`, `upAxis = "Z"`), referenced by
  the digital-twin scene (`world/welding_world.usda:2324`). Component extents computed from its meshes
  and instance transforms (assembly frame, mm; z measured from the flange face, i.e. from the
  lowest point of the assembly):

| component (STEP name) | instances | size x × y × z (mm) | z range above flange face | reading |
|---|---|---|---|---|
| `Fillet001` | 1 | 94.1 × 125.8 × 65.0 | 0 – 65 | flange plate with the side arm (reaches 95.4 mm to one side of the pen axis, 30.4 mm to the other) |
| `Body` | 1 | 84.2 × 33.0 × 80.0 | 60 – 140 | the holder: per `pen_tool.json:15-17` "body flange r 45 at 55-70 mm … tube and nose r 17.4 at 75-140 mm" |
| `chuck` | 2 | 25 × 30 × 15 each | ≈ 44 – 59 | clamp pieces (name only; function not documented) |
| whole assembly | | 94.1 × 125.8 × 140.0 | 0 – 140 | |

  (The "reading" column is an interpretation of the geometry; the files name the parts only.)
* No file documents that the holder is **3D printed**, the printer, the filament or the infill —
  the print list (`notes/curved_parts_print_list.md`) covers only the workpieces. If the thesis says
  "printed", that comes from the author, not from the repository.
* `world_parts/ZenTool_4Sided_Assembly_V4.step` (Onshape, 2026-03-14; products `Base`, `Pen`,
  `StraightRake`, `TriangularRake`) is **not** the welding-cell tool: it is the rake tool of the
  earlier sand-drawing project (see §E.7). `world_parts/robot_tablev1.step` (FreeCAD, 2026-02-09) is
  the robot table used in the Isaac scene (`welding_world.usda:2654`).

**The tool as the software sees it: `config/pen_tool.json`** (version `pen_tool-0.1`, frame `tool0`,
metres; read by `admittance_control/tool_model.py` — "One description … read by everything that needs
the tool - the marking node … and the collision model", `tool_model.py:1-6`).

| key | value | meaning / source |
|---|---|---|
| `pen_tip_m` | (0.00107, −0.00135, 0.18091) m = (1.07, −1.35, 180.91) mm | "the pendant's 4-point TCP calibration IN USE, 2026-09-28, CAP OFF … keep it equal to the pendant's TCP. Before: 2026-09-25 (1.35, -0.17, 181.73); 2026-09-24 (0, 0, 183.78); with the cap 192.5 - not used." (`pen_tool.json:9-11, 31`) |
| `touch_force_n` | 1.5 N | contact threshold (`:32`) |
| `standoff_m` | 0.035 m | approach point before the tack (`:33`) |
| `flange_adapter` | box, centre (0, 0, 25) mm, half (47, 47, 25) mm → 94 × 94 × 50 mm | `:36-40` |
| `holder_flange` | capsule (0,0,58)–(0,0,67) mm, r 48 mm | from the USD: "body flange r 45 at 55-70 mm -> holder_flange r 48" (+3 mm margin) (`:15-16, 42-47`) |
| `holder_body` | capsule (0,0,70)–(0,0,130) mm, r 20.5 mm | "tube and nose r 17.4 at 75-140 mm -> holder_body r 20.5 from 70 to 130 (its round cap ends at 150.5, past the 140 mm nose)" (`:16-18, 48-54`) |
| `pen` | capsule (0,0,120) mm → (1.07, −1.35, 172.91) mm, r 8 mm | p1 = tip − radius so the round cap ends at the tip (`:20-21, 55-61`) |
| `camera_arm` | box, centre (0, 52.5, 32) mm, half (26, 52.5, 32) mm → 52 × 105 × 64 mm | `:62-67` |
| `camera_body` | box, centre (0, 90, 45) mm, half (50, 20, 20) mm → 100 × 40 × 40 mm | "the D435i as a box in tool0 from the bench (2026-09-22): centre ~9 cm along +y … 90 x 25 x 25 mm plus margin, long axis along tool0 x" (`:22-24, 68-73`) |
| `camera.extrinsic_path` | `notebooks/T_tcp_to_cam.npy` | hand-eye result, used only for the reference marker at the calibrated optical origin, "which must sit on the lens face" (`:25-27, 75-78`; `tool_model.py:14-19`) |

Dated decisions in `pen_tool.json`: the camera bracket is "on the side OPPOSITE the end-effector
connector" (`:5-6`); the holder envelope was derived from the USD on **2026-09-29**: before, "ONE capsule
r 42 to 120 (cap to 162 mm): in a 90 deg T it cleared the plates by only 1.1 mm, so square tees were
unreachable" (`:18-19`; `README.md:1146`: clearance 1.1 mm → 15.5 mm with the CAD envelope;
`notes/thesis_notes.md:925-928`). The planner/robot FK difference at the tip: "~3 mm apart at the tip
(tcp_offset_probe read 2.9, -0.9, 182.5 against the pendant's 0, 0, 183.78)" (`:12-14`).

Visual check: `scripts/tool_model_marker_node.py` publishes the primitives latched on
`/tool_model/markers` (`visualization_msgs/MarkerArray`, frame `tool0`, re-stamped every 2 s), a sphere
at the tip and, in green, the calibrated optical origin and axis — "that green dot must sit on the lens
face of the blue camera box; if it does not, the hand-eye calibration is off" (`:1-21, 36-51`;
parameters `config` '', `extrinsic_path` '', `topic` `/tool_model/markers`, `alpha` 0.45).

**Fact check (tool).**
| draft statement | verdict | evidence |
|---|---|---|
| ch3 §3.9.1 "the pen tool is its CAD envelope (capsules for the pen and holder, boxes for the camera body)" | PARTLY | capsules: `holder_flange`, `holder_body`, `pen`; boxes: `flange_adapter`, `camera_arm`, `camera_body` (`pen_tool.json:34-74`). The envelope is a conservative hand-fitted approximation (+3 mm radial margin) of the CAD, not the CAD itself; the camera body box was measured on the bench. |
| ch5 Table tab:hardware "Tool: pen in a holder (CAD envelope)" | PARTLY | add the camera bracket and the D435i: they are part of the tool envelope (`camera_arm`, `camera_body`). |
| ch5 TODO "the pen holder's geometry" | data available | table above; TCP (1.07, −1.35, 180.91) mm, cap off. |
| ch5 TODO "the camera's cable (it limits the wrist's turn)" | CONFIRMED (in marking code) | ±180° wrist_3 window (`config/marking.json` `wrist3_window_deg`, see §E.3); incident 2026-10-16 (`notes/thesis_notes.md:25-45`). |
| ch5 fig:tool "(RViz, /tool_model/markers)" | CONFIRMED | `tool_model_marker_node.py:12-13, 41`. |

### B.2 The workpieces

**Where they are defined.** `models/*.ply` (the live CAD library the cell and, as a copy in `CAD_DIR`,
the pose server use; mm), `models/weldgen_objects.json` (the registry: CAD → exact primitive, version
`weldgen_registry-1.0`, units mm, 13 entries), `notes/curved_parts_print_list.md` (the print list and
nominal dimensions, 2026-10-08), `test/data/bench_models/` (frozen copies for the tests).

**Manufacture** (`notes/curved_parts_print_list.md`): "Printing now on the Creality K1 Max (300 × 300 ×
300 mm bed): C1, E2, R2, S3, RR1, SP3" (status 2026-10-08, `:3-5`); plate strata and the base plates are
MDF: "The MDF stock is 8 mm: no plate thinner than 8 mm" (`:9-13`); printing notes: one watertight CAD per
part in mm, PLY for the cell and STL for the printer from the same model (`:65-67`); datum on the pipe axis
at the base centre / a band's spine start (`:68-70`); "matte, light (white or light grey). Black, glossy or
translucent filament gives the D435i holes or wrong depth" (`:71-72`); walls ≥ 3 mm, bands and plates
printed lying flat, pipes upright, brim on R1/R2 (`:73-78`); "Measure after printing (calipers)" (`:79-80`).
**The filament material (PLA or other) is not stated anywhere in the repository.**

**Holding.** "Fixtures: the pipes and bands stand by themselves on the plate. The saddle runs (R1, R2)
need two V-blocks; those can be MDF" (`curved_parts_print_list.md:87-88`); "Magnet-held parts can move
~1 mm between scan, marking and touches" (`notes/todo.md:232`; `README.md:1163`). Fixtures enter the
collision model as pen-tip points in `config/marking.json` `fixtures` (`notes/todo.md:78-79`).

**The test parts (dimensions measured from the CAD files in `models/`, registry parameters from
`models/weldgen_objects.json`, nominal values from the print list).**

| model | role in the draft | CAD extents (mm) | registry primitive and parameters | print-list nominal (`curved_parts_print_list.md`) | material |
|---|---|---|---|---|---|
| `test_objv2_ear.ply` | plate T-joint, upright plate | 250 × 8 × 99 | `slab`, exact | — (MDF plate) | MDF 8 mm (stock size) |
| `test_objv2_base.ply` (live, since 2026-10-15) | base plate | 220 × 220 × 5.5 | `slab`, exact | — | not stated |
| `test_objv2_base.ply` (old, in `test/data/bench_models/`) | base plate before 2026-10-15 | 249 × 8 × 256 (README: "256 × 249 × 8 mm") | `slab`, exact | — | MDF 8 mm |
| `C1.ply` | tube on a plate | 62 × 62 × 100 | `tube`, r_outer 31.0, wall 3.0, length 100.0, base end flat | "pipe stub, square end, Ø 62 (r 31), wall 3, length 100" (`:31`) | printed |
| `E2.ply` | mitred tube on a plate | 90 × 90 × 110 | `tube`, r_outer 45.0, wall 5.0, length 110.0, base cut = plane with n_local = (0.4226, 0, −0.9063) i.e. **25°** | "tilted pipe, Ø 90, wall 5, tilt 25°" (`:35`) | printed |
| `R2.ply` | main (run) tube | 100 × 100 × 180 | `tube`, r_outer 50.0, wall 6.0, length 180.0, flat ends | "run pipe Ø 100 (R 50), wall 6, length 180" (`:38`) | printed; held by two V-blocks |
| `S3.ply` | branch tube on R2 | 50 × 99.2 × 80.7 (in R2-related placement) | `tube`, r_outer 25.0, wall 4.0, length 99.3, base cut = cylinder r 51.04 mm, axis (0, 0.940, 0.342) → **70° branch** | "branch, 70° (tilt 20°), on R2, Ø 50 (r 25), wall 4, length 100" (`:41`) | printed |
| `RR1.ply` | rounded rectangle on a plate | 64 × 100 × 64 | `swept_slab`, shape rounded_rect, corner r 12.0, wall 3.0, height 100 | "closed rounded-rectangle tube 64 × 64, corner R 12, wall 3, height 100" (`:43`) | printed |
| `SP3.ply` | S-band on a plate | 120.5 × 80 × 224.4 | `swept_slab`, open band, B-spline spine (4 control points), wall 5.0, height 80 | "curved band, S-curve (B-spline, 5 control points), span 220, band 5, height 80. Redrawn 2026-10-09 as a true 5 mm offset (the first version's second side was an x-shift, wall 2.4–5 mm)" (`:48`) | printed |
| `270circle.ply` | not in draft | 150 × 50 × 150 | `swept_slab`, open band (270° arc), 8 control points, wall 5.0, height 50 | — (older library, 2026-07-02) | — |
| `plate.ply`, `smallplate.ply` | not in draft | 150 × 4 × 100; 100 × 50 × 4 | `slab`, exact | — | — |
| `vplate.ply` | not in draft | 100 × 104 × 100 | none: "extrusion: constant-wall open band, but no cubic spine fits it" | — | — |
| `assembly_mesh.ply` | not in draft | 325.9 × 329.9 × 113.0 (21 vertices) | none ("not a pipe", not a box) — the exported T-assembly (`~/export_mesh`, 2026-08-12) | — | — |

Registry fit quality (`weldgen_objects.json`): fitted primitives have `verified_max_dev_mm` 0.037-0.067 mm
(C1 0.048, E2 0.049, R2 0.049, RR1 0.037, S3 0.057, SP3 0.050, 270circle 0.067); slabs are exact
(≈ 1e-14 mm). The stale-entry guard (library-side) refuses an entry whose CAD vertices lie more than
`STALE_TOL_MM` = 0.5 mm from its primitive (`admittance_control/part_shapes.py:186-188, 200-244`),
introduced after **2026-10-15**: "the new 220 x 220 x 5.5 mm plate kept the old 256 x 249 x 8 mm entry,
turned 90 deg, and mode A found no seam at all, silently" (`part_shapes.py:186-187`;
`notes/curved_seams_plan.md:484-497`; `test/data/bench_models/README.md:3-6`).

`test/data/bench_models/`: frozen copies for the tests (`README.md` there): the old plate "from git
(`f0bc2de~1`)", the ear, C1, E2, R2, S3, RR1 "copies of 2026-10-15", and their registry (7 entries, built
for exactly these files). "Do not refresh this folder from `models/`." (md5 of C1/E2/R2/RR1/S3/ear equal
to `models/`; the base differs.)

Isaac Sim versions: `models/270circle.usda`, `plate.usda`, `smallplate.usda`, `vplate.usda` (2026-07-02)
are USD conversions of the older library parts for the digital twin; the curved bench parts have none.

**Fact check (workpieces, ch5 Table tab:setup_parts and TODO).**
| draft statement | verdict | evidence / correction |
|---|---|---|
| "Plate T-joint — two 8 mm MDF plates — two straight fillets" | PARTLY | true for the ear (250 × 99 × 8) on the **old** base (256 × 249 × 8). Since 2026-10-15 the base CAD is 220 × 220 × 5.5 mm (`models/test_objv2_base.ply`); state which base each experiment used. The two-fillet output is documented (`README.md:1467-1469`, "mode A: 2 weldable seam(s): fillet 180mm …"). |
| "C1 — tube on a plate — ellipse/circle" | CONFIRMED | flat-ended tube r 31, wall 3, L 100 standing on the plate → circle (an ellipse if tilted). |
| "E2 — mitred tube on a plate — ellipse" | CONFIRMED | base cut plane at 25° (registry `base_cut` normal). Note the print list calls it "tilted pipe … tilt 25°": the CAD is a vertical tube whose base is cut at 25°, so it stands tilted on the plate; the thesis should use one description. |
| "S3 on R2 — branch tube on a main tube — saddle" | CONFIRMED | S3 base cut = cylinder (r 51.0 mm ≈ R2's 50 mm), 70° branch. R2 needs two V-blocks. |
| "RR1 — rounded-rectangle section on a plate — rounded rectangle" | CONFIRMED | 64 × 64, corner R 12, wall 3, height 100. |
| "SP3 — S-shaped band on a plate — swept path" | CONFIRMED | B-spline band, 5 mm, height 80, span ≈ 220 (redrawn 2026-10-09). Registry spine has 4 control points ("side_a (4 control points)") while the print list says 5 control points — say "a cubic B-spline" without the count, or check the CAD. |
| TODO "material (MDF, PLA prints)" | NOT VERIFIABLE for PLA | MDF is documented; the filament type is not in any file (only "matte, light" filament advised). |
| TODO "how parts are held (magnets, fixtures)" | CONFIRMED | magnets (`todo.md:232`), V-blocks for R2 (`curved_parts_print_list.md:87-88`, `curved_seams_plan.md:68`), fixtures as pen-tip points in `marking.json`. |
| TODO "the base plate change (220×220×5.5 mm)" | CONFIRMED | 2026-10-15 (`test/data/bench_models/README.md:4-6`; `part_shapes.py:186-187`). |
| ch3 §3.7.1 "a registry entry whose CAD file has changed since the fit is detected and refused" | CONFIRMED, mechanism to state | detected geometrically (a CAD vertex > 0.5 mm off the primitive), not by a file hash; `welding_points` refuses and `refine_pose` falls back to the mesh's box (`curved_seams_plan.md:491-495`). |

---
## C. Calibration tools: procedures as implemented

### C.0 The chain as the repository runs it

Order (`README.md:1517-1541`, "Calibration routine (in this order)"; rationale `README.md:1262-1264`:
"kinematics → pen TCP and table plane → camera depth → extrinsic rotation → extrinsic translation. A step
done on top of an uncorrected earlier one learns its error (the self-calibration history recorded the depth
bias as an extrinsic offset)"):

| # | link | tool(s) | reference | output | last written |
|---|---|---|---|---|---|
| 0 | kinematics | `config/ur5e_calibration.yaml` (robot's factory calibration "extracted from the controller", `urdf/ur5e.urdf.xacro`); `scripts/tcp_offset_probe.py` (check) | the controller's own FK (`tcp_pose_broadcaster`) | yaml used by URDF, driver, every tool | 2026-09-29 decision |
| 1 | pen TCP | the **pendant's 4-point TCP wizard** (value in use); `table_touchoff.py mode:=tcp_check` (roll/tilt check); `pen_tip_touchoff.py` (alternative, unused output) | a fixed point / the touched table | `config/pen_tool.json` `pen_tip_m`; `notebooks/tcp_check.json` | TCP 2026-09-28; check 2026-09-28 13:36 |
| 2 | table plane | `scripts/table_touchoff.py` (plane mode) | robot (pen tip from the driver's TCP pose) | `notebooks/table_plane.json` | 2026-10-02 12:52 |
| — | initial hand-eye | `helper/calibration/extract_extrinsics.py` → `resolve_handeye.py` (ChArUco, AX = XB) | ChArUco board | `handeye_samples.npz`, `T_tcp_to_cam_unrefined.npy/.json` | 2026-09-28 15:07 |
| 3 | camera depth | `scripts/tare_distance.py` + realsense-viewer **Tare**; check `extrinsic_check.py` at 3 heights | table plane + calibrated FK + extrinsic | (camera firmware) | 2026-10-02 |
| 4 | extrinsic rotation | `scripts/extrinsic_check.py` (4 wrist yaws) → `helper/calibration/refine_extrinsic_from_table.py` | table plane | `extrinsic_check.json`, `T_tcp_to_cam_refined.npy/.json` → promoted to `T_tcp_to_cam.npy` | 2026-09-28 and 2026-10-02 13:06 |
| 5 | extrinsic translation | online inside `~/refine_pose` (library `multiview_service.py:79` `online_selfcal: True`); offline accumulation `scripts/selfcal_extrinsic.py`; test `scripts/compare_registrations.py` | disagreement between views | `notebooks/selfcal_history.json` (23 runs) | 2026-10-01 … 10-16; never promoted to a file |
| — | verification | `touch_probe.py`, `check_registration.py`, `pose_jitter_probe.py`, `contact_error.py` | pen touches / live cloud | stdout, `--json` | — |

### C.1 Kinematics: `config/ur5e_calibration.yaml` and `scripts/tcp_offset_probe.py`

* `urdf/ur5e.urdf.xacro` includes `ur_description`'s `ur_macro.xacro` with
  `kinematics_parameters_file="$(find admittance_control)/config/ur5e_calibration.yaml"`; comment: "the
  controller, RTDE and the hand-eye samples all use the calibrated chain; TF on the nominal one put the camera
  0.55 deg / ~5 mm off at the scan home (measured 2026-09-29)". The driver gets the same file via
  `kinematics_params_file:=` (workspace `README.md`: without it "TF differs from the controller's own
  kinematics by 2.4-4.2 mm and ~0.5 deg at the tool (measured 2026-09-29), which alone put the pen marks ~4-6
  mm off"). Every robot-side tool loads it by default (`table_touchoff.py:102`, `pen_tip_touchoff.py:52`,
  `tcp_offset_probe.py:67`, `tare_distance.py:69`) through `kinematics.use_kinematics` (library).
* `tcp_offset_probe.py` (installed; node `tcp_offset_probe`; params `tcp_pose_topic`
  `/tcp_pose_broadcaster/pose`, `joint_states_topic` `/joint_states`, `base_frame_rotation_deg` 180.0,
  `calibration_file` `config/ur5e_calibration.yaml`): prints once per second
  D = inv(T_baselink_tool0)·T_baselink_base·T_base_tcp with the nominal and with the calibrated FK — "the
  calibrated row equals the pendant TCP at every pose while the nominal row wanders with the pose - the
  kinematic mismatch, directly" (`:1-27`). README §14: calibrated FK reproduces the pendant TCP to 0.001°,
  nominal wanders 2.4-4.2 mm / 0.55° *(result → ch6)*.

### C.2 Pen TCP

1. **Value in use: the pendant's 4-point TCP**, cap off, (1.07, −1.35, 180.91) mm, 2026-09-28
   (`config/pen_tool.json:9-11`); earlier values 2026-09-25 (1.35, −0.17, 181.73) and 2026-09-24
   (0, 0, 183.78); with the cap 192.5 mm (not used). The pen capsule's p1 must follow it (`pen_tool.json:20-21`).
2. **Roll/tilt check: `table_touchoff.py mode:=tcp_check`** (`table_touchoff.py:42-50, 126-134, 487-628`):
   needs the table plane; from a pen-down start above an empty spot the pen touches *that* spot vertically at
   rolls 0/90/180/270° ("on a sheet of paper, a correct TCP leaves one dot") and tilted 30° toward four
   azimuths (0/90/180/270°) with the tool yaw held; approach 20 mm along the pen (`check_hover_m` 0.02),
   re-orientations 50 mm above the spot (`check_safe_m` 0.05), joint speed cap 0.3 rad/s. Least squares
   returns the TCP error e in the pen frame, its lateral part and a conditioning figure; writes
   `notebooks/tcp_check.json`. On disk (written 2026-09-28 13:36:29): lateral 0.21 mm, along-pen −4.01 mm,
   residual RMSE 0.04 mm *(result)*; README interprets the −4 mm as the tilted touches' +0.54 mm × 1/(1−cos 30°),
   "a rounded or flexing tip, not a length error; not applied" (`README.md:1127`). Advice printed: "If lateral
   > 1 mm, correct the pendant TCP (and pen_tool.json) by minus its LATERAL part only" (`table_touchoff.py:623`).
3. **`pen_tip_touchoff.py`** (installed; node `pen_tip_touchoff`; services `~/record`, `~/solve`, `~/clear`;
   params `kinematics_file`, `joint_states_topic`, `out` = `notebooks/pen_tip_touchoff.json`): the classic
   4-point TCP from the planner's FK: bring the tip onto one fixed point from ≥ 4 clearly different wrist
   orientations (tilt 30-60° between touches), solve R_i d + t_i = p, print d (`pen_tip_m`), p (a bench point
   known from the robot alone) and the RMS ("aim for under 1 mm") (`:1-21`). **No `pen_tip_touchoff.json`
   exists in `notebooks/`**: the value in use came from the pendant, not from this tool.

### C.3 Table plane: `scripts/table_touchoff.py` (plane mode)

Node `table_touchoff` (installed), services `~/run`, `~/abort` (std_srvs/Trigger); action client
`/scaled_joint_trajectory_controller/follow_joint_trajectory`; subscribes `/joint_states`,
`/force_torque_sensor_broadcaster/wrench`, `/tcp_pose_broadcaster/pose` (`:121-156`).
Parameters (defaults, `:102-134`): `kinematics_file` `config/ur5e_calibration.yaml`, `dry_run` True,
`sides` 6, `edge_m` 0.10, `with_centre` True, `repeats` 1, `v_fast_m_s` 0.004, `v_slow_m_s` 0.001,
`v_move_m_s` 0.03, `backoff_m` 0.003, `max_travel_m` 0.12, `abort_force_n` 8.0, `release_abort_force_n`
40.0, `bias_window_s` 0.3, `max_joint_jump_rad` 0.35, `tool_config` '', `out` `notebooks/table_plane.json`,
`mode` `plane`; touch force = `pen_tool.json` `touch_force_n` (1.5 N).

Procedure (docstring `:1-57`, code `:369-485`):
1. Empty table; jog the pen roughly vertical 3-8 cm above the middle of the area; External Control running.
2. Pattern: a regular hexagon of edge 10 cm (radius 10 cm) centred under the pen, plus its centre — 7 touches,
   "so the plane fit has four spare points and a real residual" (`sides:=3 edge_m:=0.15` gives a triangle).
3. Per spot: move at hover height above the spot, pen vertical (tool yaw kept); measure the wrench bias in
   the air (0.3 s window); **fast touch** at 4 mm/s until |F − bias| > 1.5 N; release at once (straight up
   3 mm, guarded only by the 40 N release limit), re-measure the bias; **slow touch** at 1 mm/s → the recorded
   point; release; retract.
4. The contact point is the **driver's TCP pose** (`tcp_pose_broadcaster`, pendant TCP = pen tip, calibrated
   kinematics) at the moment the force crosses the threshold; the FK tip is logged next to it; UR `base` =
   base_link rotated 180° about z (`:31-36, 450-454`).
5. Least-squares plane, RMSE, optional repeatability; writes `table_plane.json` (frame base_link, points,
   plane, log) and prints the `marking.json` `table_z_m` and ICP `ground_z_m` lines (`:466-485`).

Why slow (dated): "at 10 mm/s the first run read 2.66 N at detection and ~10 N once stopped, and the 8 N abort
then cancelled the back-off itself, leaving the pen pressed" (`:22-29`).

Current file *(result → ch6)*: written 2026-10-02 12:52:34, 7 touches, normal (0.0151, −0.0255, 0.9996) ⇒
**tilt 1.70°**, z at the centroid −96.15 mm (base_link), RMSE 0.22 mm, z span 5.2 mm over the hexagon;
contact forces 1.55-1.97 N on the slow touches. (README §14/§16 quote "tilt 1.57°" from the 28 Sep plane;
use the value of the plane actually in use.)

Consumers: ICP ground cut follows the plane with a 2 mm offset (`launch/pointcloud.launch.py:96-105, 290-293,
481-485`), the collision model's tilted table (unless `marking.json` `table_z_m` is set; `README.md:532`),
`extrinsic_check.py`, `tare_distance.py`, `refine_extrinsic_from_table.py`, `touch_probe.py`.

### C.4 Hand-eye calibration (ChArUco): `helper/calibration/extract_extrinsics.py`, `resolve_handeye.py`, notebook

**Capture and solve — `extract_extrinsics.py`** (stand-alone CLI, not a ROS node; pyrealsense2 + RTDE):
1. Board: `cv2.aruco.CharucoBoard((squares_x, squares_y), square_length, marker_length, dictionary)`;
   defaults **4 × 4 squares, 30 mm squares, 22 mm markers, `DICT_4X4_250`**, legacy pattern parity on
   (`:24, 136-162, 245-257`). The saved reference image `notebooks/charuco_reference.png` (1400 × 1400 px,
   2026-09-28 14:54) decodes as 8 markers of `DICT_4X4_250` (checked with OpenCV 5.0.0), i.e. this board
   (a 120 mm board: "a 12 cm board at 30 cm", `resolve_handeye.py:91-94`).
2. Camera: RealSense colour stream 1280 × 720 @ 30 fps (`--width/--height/--fps`, `:121-123`), intrinsics
   K from `notebooks/realsense_intrinsics.npy`, **zero distortion** ("the saved file contains only K",
   `:172-187`) — the factory intrinsics (fx 919.47, fy 918.94, cx 650.85, cy 350.62; model
   inverse_brown_conrady with all-zero coefficients, notebook cell 3 output).
3. Robot: RTDE `RTDEReceiveInterface(192.168.8.4).getActualTCPPose()` per sample — i.e. the pose of the
   pendant's **active TCP** (`:190-203, 602-611`).
4. Per frame: detect markers (`ArucoDetector`), interpolate ChArUco corners, board pose by
   `estimatePoseCharucoBoard` or (OpenCV ≥ 4.8/5) `CharucoDetector.detectBoard` + `matchImagePoints` +
   `solvePnP(SOLVEPNP_ITERATIVE)` (`:259-287`); key `c` captures (failed detections save
   `handeye_debug_latest.png`), `q` solves; target 15 samples, minimum 5 (`:124-135`).
5. Solve **AX = XB**: `cv2.calibrateHandEye(R_gripper2base, t_gripper2base, R_board2cam, t_board2cam,
   method=CALIB_HAND_EYE_TSAI)`; if the OpenCV build lacks it (OpenCV 5), the numpy **Park–Martin** solver of
   `resolve_handeye.py` (`:379-424`).
6. **Residual judge** printed after every solve (added after the 2026-07 file "carried a 120 mm spread from a
   silently failed solve", `:426-445`): `T_base_tcp_i · X · T_cam_board_i` must be one pose; position std and
   range (mm) and rotation RMS about the chordal mean; verdict GOOD if ‖std‖ < 6 mm and rotation < 2°, FAIR if
   < 10 mm and < 3°, else BAD (`resolve_handeye.py:87-118`); the Park–Martin result is shown next to Tsai.
7. **TCP composition**: `T_tool0_cam = T_tool0_tcp · T_tcp_cam` with `--tcp-offset X Y Z RX RY RZ`; the
   tool **refuses to save without it** — "without the composition the camera lands wrong by exactly the TCP
   (2026-09-25: a file with the camera 129 mm behind the pen tip)" (`:447-466`).
8. Writes `notebooks/T_tcp_to_cam.npy` and `notebooks/handeye_samples.npz` (`R_gripper2base`,
   `t_gripper2base`, `R_board2cam`, `t_board2cam`) (`:336-360`).

**Re-solve and judge — `resolve_handeye.py`** (numpy only; `--samples`, `--compare`, `--write`,
`--tcp-offset`): Park & Martin (1994) "rotation from the log-map least squares, translation from a linear
least squares"; consistency residual, leave-one-out table, refusal to `--write` without `--tcp-offset`
("2026-09-28: 126 mm behind the pen tip"); writes a JSON sidecar (samples, solver, composed TCP, board-in-base
std, rotation spread, camera origin) (`:1-28, 121-171`). Docstring: "The file this repo carried on 2026-09-24 had
a 120 mm range against its own samples (the OpenCV solve had failed silently); the re-solve of the same samples
has a 3 mm std" (`:17-21`).

**The samples on disk** (`notebooks/handeye_samples.npz`, 2026-09-28 15:01): 33 samples; board 0.25-0.45 m from
the camera (median 0.32 m); wrist orientations up to 174° from their mean (median 72°); the TCP (pen tip) at
**z = 102.35 ± 0.04 mm in every sample** with x, y spread over 268 × 322 mm — the poses were taken by re-orienting
about the pen tip at one height (how they were produced is not documented). Re-running the judge (report mode)
gives: Park–Martin camera origin in the TCP frame (33.8, 90.3, −126.4) mm, board-in-base std (1.2, 1.1, 3.0) mm,
range (5.4, 5.3, 13.7) mm, rotation RMS 1.8° → GOOD; robot rotation diversity max 180°, median 94°
*(result → ch6)*.

**The notebook `notebooks/camera_calibration_test.ipynb`** (6 cells, kernel `.venv (3.12.3)`): cell 0 markdown
on K; cells 1-2 RealSense discovery/preview (output: "Intel RealSense D435" — the SDK's device name string);
cell 3 reads the factory intrinsics and saves `realsense_intrinsics.npy` (output above); cell 4 an earlier
eye-in-hand capture loop (ChArUco 4 × 4, 30/22 mm, **`DICT_6X6_250`**, TSAI, output `T_tool0_camera.npy`) that
instructs "set the UR TCP to 0 (flange) before capturing" — the later script reads at the active TCP and composes
instead; cell 5 compares TSAI/PARK/HORAUD/DANIILIDIS by the board-in-base spread. Cells 4-5 have no outputs.
**The current extrinsic was not produced by the notebook** but by the scripts above.

**Extrinsic files and what they document** (all `notebooks/`; translation of the camera origin in tool0 or TCP
frame; Δ to the file in use):

| file | written | camera origin (mm) | Δ rotation to current | what it is |
|---|---|---|---|---|
| `T_tcp_to_cam_wrong.npy` | 2026-07-02 | (24.4, 90.9, 34.1) | 26.2° | the July file of the failed OpenCV solve ("120 mm range against its own samples"; pen_marking_plan.md:254 "(24, 91, 34) and a 7° tilt / 26° roll in the bad file") |
| `T_tcp_to_cam_raw_tcpframe_2026-09-25.npy` | 2026-09-25 | (15.1, 85.1, −128.8) | 1.9° | a solve left in the TCP frame: the camera 129 mm "behind the pen tip" |
| `T_tcp_to_cam_uncomposed_2026-09-28.npy` | 2026-09-28 15:07 | (33.8, 90.3, −126.4) | 0.19° | the 33-sample Park–Martin solve before composing the TCP |
| `T_tcp_to_cam_unrefined.npy/.json` | 2026-09-28 15:07 | (34.84, 88.98, 54.50) | 0.19° | the same composed into tool0 with TCP (1.07, −1.35, 180.91) mm: board std (1.16, 1.12, 3.04) mm, rotation 1.81°, 33 samples, Park–Martin |
| `T_tcp_to_cam_before_tilt_2026-10-02.npy` | (backup made 2026-10-02) | (34.84, 88.98, 54.50) | 0.49° | the file in use until 2 Oct: unrefined rotated by 0.37° (the 28 Sep table refinement, camera-fixed tilt 0.36° → 0.04°, `README.md:1128`) |
| `T_tcp_to_cam_refined.npy/.json` = **`T_tcp_to_cam.npy`** (identical sha1 73f10748…) | 2026-10-02 13:06:52 | (34.84, 88.98, 54.50) | 0 | in use: before_tilt × Rc, Rc = 0.488° (about camera x −0.480°, y +0.086°), table's own tilt 0.441°, fit residual 0.071°, conditioning 0.9997 |
| `T_base_to_cam.npy` | 2026-07-02 | (338.7, −124.4, 245.6) | — | an eye-to-hand result (`--calibration-setup eye-to-hand`), not used by the cell |

`notebooks/T_tcp_to_cam.json` and `T_tcp_to_cam_refined.json` are identical sidecars (base extrinsic, check file,
plane, correction, note "translation and rotation about the optical axis unchanged"). Optical axis in tool0 of
the file in use: (−0.0033, −0.0011, 1.0000), i.e. the camera looks along +z of tool0 within 0.2°, offset
(34.8, 89.0, 54.5) mm from the flange centre.

### C.5 Depth: `scripts/tare_distance.py` + the RealSense Tare

`tare_distance.py` (installed; plain script with one-shot `/joint_states` read; `--extrinsic`
`notebooks/T_tcp_to_cam.npy`, `--plane` `notebooks/table_plane.json`, `--kinematics`
`config/ur5e_calibration.yaml`) (`:1-105`):
1. camera pose = calibrated FK(q) · extrinsic (no TF: the perception launch must be stopped, because
   realsense-viewer cannot open the camera while the ROS node holds it);
2. ground truth = depth **along the optical axis** to the pen-measured plane, perp/cos;
3. prints the tilt of the camera to the table (keep < 1°) and how far the axis lands from the touched patch
   (< 80 mm), exit code 2 otherwise;
4. operator: stop the launch, realsense-viewer → More → Calibration → Tare with that distance, restart.
Why: "the D435/D435i depth origin sits 4.2 mm behind the glass … (2026-10-02: after a Tare, the camera read the
table +1.7 / +3.3 / +5.2 mm high at 0.29 / 0.42 / 0.60 m - linear, 11 mm per metre)"; the colour optical
origin used by the pipeline is coplanar with the depth origin, 15 mm to the side (`:7-25`).
Check: `extrinsic_check.py` at three heights over the same spot (`README.md:1241-1244`). Before the Tare
+1.92 / +0.31 / −3.28 mm at 291 / 439 / 626 mm, fitted by a + k z² with k ≈ −17 mm/m², δ ≈ 0.6 px; after the
robot-computed Tare +0.29 / +1.06 / +1.43 mm at 0.3 / 0.45 / 0.6 m (`README.md:1239-1260`) *(results → ch6)*.

### C.6 Extrinsic rotation: `scripts/extrinsic_check.py` → `helper/calibration/refine_extrinsic_from_table.py`

`extrinsic_check.py` (installed; node `extrinsic_check`; services `~/capture`, `~/report`, `~/clear`;
subscribes `/camera/depth/color/points`; TF base_link ← cloud frame; params `cloud_topic`, `base_frame`
`base_link`, `plane_file`, `radius_m` 0.0, `search_m` 0.25, `band_m` 0.008, `min_range_m` 0.28 ("D435i minimum
depth: ~0.28 m at 1280x720 … seen 2026-09-28"), `out` `notebooks/extrinsic_check.json`) (`:1-88`):
1. per capture, take the cloud to base_link through TF (i.e. through the extrinsic in use);
2. find the dominant flat surface within ±25 cm of the pen plane by a height histogram ("so a large extrinsic
   error is measured rather than filtered out"), keep ±8 mm, robust plane fit;
3. report the HEIGHT offset (mm) and TILT (deg, with azimuth) against the pen plane extended to that spot, and
   the extrapolation distance from the touched patch;
4. with views at wrist yaws 0/90/180/270° about the vertical, `table_probe.separate_tilt` (library) splits the
   tilt into a **camera-fixed** part (turns with the wrist → extrinsic rotation) and a **table-fixed** part
   (does not) once the yaws span ≥ 90°. Good extrinsic: tilt < 0.3° and offsets within ~2 mm in every view;
   yaw about the optical axis and horizontal translation are invisible to a plane (`:28-33`).
Motivation (dated): "two calibrations that each fit to 3 mm disagreed by 20 mm on 2026-09-28" (`:13-16`).

`refine_extrinsic_from_table.py` (numpy; `--check`, `--extrinsic` (the file IN USE during the captures),
`--plane`, `--write`) (`:1-86`): needs ≥ 3 views with the camera rotation; fit
`table_probe.refine_camera_rotation` (library) → small rotation Rc; **T_tool0_cam_refined =
T_tool0_cam_used · [[Rc, 0],[0, 1]]**; rotation about the optical axis and translation untouched; prints the
correction, the table's own tilt, residual, conditioning (> 0.3 ok, else "use yaws spread over 180+ deg") and
the shift at 300 mm; writes a NEW file `T_tcp_to_cam_refined.npy` (+ .json), never overwrites; promote by hand
and re-check: "the camera-fixed tilt should drop to ~0.1 deg or below". Docstring: "The ChArUco hand-eye leaves
~0.3 deg of rotation uncertainty (1.8 deg per view over ~30 views)".
On disk: `extrinsic_check.json` (2026-10-02 13:05) — 4 views at camera yaws −130.5°, −52.7°, 46.1°, 139.7°, range
0.46 m, height offsets 1.2-2.5 mm, tilts 0.13-0.97°, plane RMSE 1.1-1.4 mm *(result)*; the refinement it fed is
the 0.488° correction above.

### C.7 Extrinsic translation: online in `~/refine_pose`, offline `scripts/selfcal_extrinsic.py`, `scripts/compare_registrations.py`

* Online (library-side): `multiview_service.py:77-79` `record_selfcal: True`, `selfcal_history:
  notebooks/selfcal_history.json`, `online_selfcal: True`; each run estimates d (camera frame) jointly with the
  poses and appends it with its information matrix and the extrinsic file's sha1.
* `selfcal_extrinsic.py` (installed; no ROS; `--history`, `--extrinsic`, `--min-runs` 3, `--max-spread-mm` 1.0,
  `--write`): combines the runs made with the same extrinsic (sha1) by their information; with ≥ 3 runs agreeing
  within 1 mm writes `T_tcp_to_cam_selfcal.npy` next to the file in use (undetermined directions left as they
  were; a direction counts as determined when its σ ≤ `max_sigma_mm` 0.5 mm, library `selfcal.py:46-50`);
  promotion by hand; "After promotion the history of the NEW file starts empty; the next runs should then
  estimate d near 0 - that is the check" (`:1-19`).
* On disk *(result)*: 23 runs 2026-10-01 17:39 → 2026-10-16 15:27; 22 with the current extrinsic (sha1
  73f10748…), d typically ≈ (0, +3, −4.4) mm with outliers (e.g. (−6.9, −3.8, 0.1) on 2026-10-06 15:19).
  **No `T_tcp_to_cam_selfcal.npy` exists**: the offline correction was never written/promoted; the translation
  correction acts only online, per run.
* `compare_registrations.py` (installed; no ROS; positional `a`, `b` or `--last N`): registers the same untouched
  part from two scan poses whose wrist yaw differs by 180° — a horizontal extrinsic error d appears as R_tool d and
  flips, so the two registrations differ by ~2d (`:1-16`; motivation 2026-09-28: "two ChArUco calibrations
  disagreed by 18 mm in exactly that position").

### C.8 Verification probes (evaluation only)

* `touch_probe.py` (installed; node `touch_probe`; params `results_dir`, `models_dir`, `table_plane`): with the
  tip freedriven lightly onto a surface, prints the height above the pen plane (the pen-length check), the signed
  distance to the nearest face of every registered part in `assembly.json` (+ = real surface outside the
  registered face) and, per tack, tip minus planned centre; tip = TF base_link → `tool0_controller` (the
  controller's TCP) (`:1-22`). Dated: "A few touches separate the two causes of early contact (2026-09-29)".
* `check_registration.py` (installed; params `cloud_topic` `/camera/depth/color/points`, `results_dir`,
  `models_dir`, `max_dist_m` 0.015): per registered part and visible CAD face, median offset of the live cloud
  along the normal and its drift across the face (10 mm strips); GOOD under 3 mm (`GOOD_MM`); "A plate
  registered straddling its own two faces (2026-09-29: the ear, 6.5 deg lean, root 10 mm off) … read drift
  6.3 mm here, the gated re-fit of the same cloud 1.2 mm"; tests the registration only, not the hand-eye
  (`:1-46`).
* `pose_jitter_probe.py` (installed; params `topic` `/perception/icp/refined_pose`, `seconds` 30.0): std and
  range of position and orientation of a static part's pose — the registration noise floor (`:1-13, 34-39`);
  README §14 (25 Sep): ICP std (5.2, 2.5, 3.0) mm, range 24 mm, swings to 6.4°, which led to the robust mean at
  save.
* `contact_error.py` (**not installed** in CMakeLists; CLI `--save-dir`, `--marks`, `--registry`,
  `--weldgen-path`, `--json`, `--plate-on-table`): fits, from the pen's recorded contact points
  (`tip_contact_m`), the real tube's lateral offset and the real plate's height offset by grid search (step
  0.25 mm), then reports each mark's distance from the real root; "Vision-only stays the method: this is
  evaluation, never fed back" (`:1-29`). Docstring says "±15 mm lateral" but the code's default is
  `span = 25.0` mm (`:44-47`) — docstring/code mismatch. README: "2026-10-14: C1 8.8 mm / E2 8.1 mm sideways,
  plates 1-3 mm high" (`README.md:528`).

`helper/calibration/sam6d_client.py` is **not a calibration tool**: a 2026 stand-alone test client that grabs one
640 × 480 RealSense frame and POSTs it to the SAM-6D server at `http://127.0.0.1:5000/predict_pose`
(`:1-66`); legacy.

### C.9 Fact check of §3.3 and Table tab:calib_chain

| draft statement | verdict | evidence / correction |
|---|---|---|
| "The robot is that reference: its factory-calibrated kinematics and the pen tip, touched onto surfaces" | CONFIRMED | table plane from the driver's TCP pose (`table_touchoff.py:31-36`); calibrated kinematics in every tool. |
| "a step calibrated on top of an uncorrected earlier one learns that error" | CONFIRMED | `README.md:1262-1264` (the self-cal history recorded the depth bias as an extrinsic offset). |
| Row "Kinematics — factory calibration used in every tool — the controller" | CONFIRMED | `urdf/ur5e.urdf.xacro`, driver `kinematics_params_file`, tools' `kinematics_file` defaults. |
| Row "Pen tip (TCP) — 4-point TCP; roll and tilt touches — table plane" | CONFIRMED with detail | the 4-point TCP is the **pendant's** (not `pen_tip_touchoff.py`); the roll/tilt touches are `tcp_check` (rolls 0/90/180/270°, 30° tilts at four azimuths) on the table plane. |
| Row "Table plane — pen touches, plane fit — the robot" | CONFIRMED | 7-point hexagon, two-speed touches, 1.5 N. |
| Row "Camera depth — Tare with a robot-computed distance — table plane" | CONFIRMED | `tare_distance.py`. |
| Row "Extrinsic rotation — camera tilt separated from table tilt at four wrist yaws — table plane" | CONFIRMED | `extrinsic_check.py` + `refine_extrinsic_from_table.py`; only the two tilt angles are refined; rotation about the optical axis is not observable. |
| Row "Extrinsic translation — online, from multi-view disagreement (§mv_selfcal) — views" | CONFIRMED | `online_selfcal: True` (library); the offline `selfcal_extrinsic.py` accumulation exists but was never promoted. |
| Missing row | MISSING | the **initial ChArUco hand-eye** (AX = XB) is the starting point of rows 5-6 and is not in the table; also the **verification** probes (touch_probe, check_registration, contact_error) belong in the narrative or in §5.4. |
| §3.3.1 TODO "nominal vs. calibrated differ by millimetres at the tip" | CONFIRMED | 2.4-4.2 mm / 0.55° (README §14; workspace README). |
| §3.3.1 TODO "the pen TCP from four orientations" | CONFIRMED | pendant 4-point TCP. |
| §3.3.1 TODO "the table as a plane n·x + d = 0 fitted to pen touches, tilted (it is not level), used as the height reference everywhere (ground cut, depth check, collision model)" | CONFIRMED | tilt 1.70° in the file in use; consumers listed in C.3. |
| §3.3.2 Eq. (handeye) AX = XB, A_i relative flange motions, B_i relative board motions [Tsai, Park] | CONFIRMED with a precision | A_i are relative motions of the **pendant's active TCP** (RTDE `getActualTCPPose`), not of the flange; the result is composed with the TCP into tool0. Solver in use: Park–Martin (numpy) for the 33-sample file; Tsai in OpenCV builds that have it. |
| §3.3.2 TODO "the ChArUco board" | data | 4 × 4 squares, 30 mm / 22 mm, DICT_4X4_250 (120 mm board). |
| TODO "the sample poses" | data | 33 samples at 0.25-0.45 m, orientations to 174°, TCP held at one height. |
| TODO "the residual judge" | data | board-in-base consistency (std/range, rotation RMS about the chordal mean), GOOD < 6 mm & < 2°. |
| TODO "the TCP-frame pitfall (a solve in the wrong frame that looked plausible)" | CONFIRMED | 2026-09-24/25 (README §14 row 2; `extract_extrinsics.py:450-458`): the residual cannot see a rigid change of the gripper frame. |
| TODO "separating the camera's tilt from the table's by viewing the table at four wrist yaws" | CONFIRMED | done twice: 28 Sep (0.36° → 0.04°) and 2 Oct (0.49° → ≈0.07°). Put it in §3.3.2 or a new §3.3.4 "Extrinsic refinement against the table". |
| §3.3.3 Eq. (depth_error) Δz = z²δ/(fb) | CONFIRMED (README §16 model) | δ ≈ 0.6 px fitted. |
| §3.3.3 "the Tare needs a ground-truth distance; here it is computed from the robot (the camera pose by Eq. frame_chain and the touched table plane)" | CONFIRMED | along the optical axis, calibrated FK · extrinsic (`tare_distance.py:69-91`). |
| §3.3.3 TODO "Measurement at three heights; the a + k z² fit; why a hand-measured distance left a scale error" | CONFIRMED | three heights 291/439/626 mm; hand-measured Tare: 1.1 % scale error (D435i depth origin 4.2 mm behind the glass). Note `tare_distance.py:9-11` calls the hand-measured residual "linear, 11 mm per metre", the README a "1.1 % scale error" — the same thing. |

---
## E. Per-file entries

### E.0 Inventory and status

Installed as package executables (`CMakeLists.txt` `install(PROGRAMS …)`): `realsense_camera_node.py`,
`realsense_sim_camera_node.py`, `foundationpose_bridge_node.py`, `fp_tracker_node.py`, `gesture_control_node.py`,
`camera_aim_node.py`, `sam6d_bridge_node.py`, `detection_marker_node.py`, `icp_pose_refiner_node.py`,
`camera_extrinsic_tf_publisher.py`, `move_to_object_node.py`, `cartesian_admittance_controller.py`,
`drawing_action_server.py`, `drawing_dispatcher.py`, `build_weldgen_registry.py`, `tool_model_marker_node.py`,
`tack_reachability.py`, `tack_reach_marker_node.py`, `tack_marking_node.py`, `pen_tip_touchoff.py`,
`tcp_offset_probe.py`, `pose_jitter_probe.py`, `table_touchoff.py`, `extrinsic_check.py`,
`compare_registrations.py`, `check_registration.py`, `touch_probe.py`, `selfcal_extrinsic.py`,
`multiview_refine_offline.py`, `tare_distance.py`. **Not installed**: `scripts/aim_bag_report.py`,
`scripts/contact_error.py` (run with python), everything in `scripts/scripts_in_foundationpose/` (runs on the GPU
hosts), `helper/**` (installed only as a share directory, run directly). C++ targets: `admittance_control_jacobian_node`,
`totg_service_node`, `aim_rate_relay`, pybind11 module `_transit_cpp`. Interfaces: `action/ExecuteDrawing.action`,
`srv/ComputeTOTG.srv`.

Which launch starts what:

| launch file | starts | role in the thesis |
|---|---|---|
| `launch/pointcloud.launch.py` (real robot) | robot_state_publisher (URDF with factory kinematics), `camera_extrinsic_tf_publisher`, `realsense_camera_node` (`launch_camera` true), depth_image_proc `PointCloudXyzrgbNode`, `foundationpose_bridge_node` (**`launch_foundationpose` default false**), `detection_marker_node`, `icp_pose_refiner_node` (`launch_icp` true), `fp_tracker_node` (when `tracking_source` = `fp`, the default), `totg_service_node` (`launch_totg` true), rviz2 (`launch_rviz` false) | the cell |
| `launch/gestures.launch.py` | `gesture_control_node`, `camera_aim_node` + `aim_rate_relay` (`aim` true; `aim_dry_run` default **false** = live), shuts everything down when the gesture window closes | operator interface |
| `launch/digital_twin_pointcloud.launch.py` | the same graph on Isaac Sim (`/isaac_joint_states`, `realsense_sim_camera_node`) | digital twin, not in the draft |
| `launch/admittance_control.launch.py` | legacy sand-drawing action stack (`drawing_action_server`, `drawing_dispatcher`, `totg_service_node`) | legacy |
| `helper/legacy_sand_drawer/safe_probe_preview.launch.py` | legacy SAM3 safe-probe preview + GUI | legacy |
| none (run by hand) | `tack_reachability.py`, `tack_reach_marker_node.py`, `tack_marking_node.py`, `tool_model_marker_node.py`, all calibration probes | — |

### E.1 Perception nodes (laptop)

#### `scripts/realsense_camera_node.py` — the real camera
* **Purpose.** RealSense D435i driver node (pyrealsense2, not the `realsense2_camera` ROS driver) publishing aligned
  RGB-D and CameraInfo; started by `pointcloud.launch.py`.
* **How it works.** (1) `rs.pipeline` with colour `rgb8` and depth `z16` at the configured size/fps (`:114-125`);
  (2) `rs.align(rs.stream.color)` aligns depth to colour so one K serves both (`:127-128`); (3) depth device units →
  millimetres (16UC1, `depth_scale` 1.0 in camera.json) (`:130-133, 196-201`); (4) writes the live colour intrinsics
  to `scripts/rgb_depth_to_send/camera.json` (`:149-178`); (5) drops 5 frames for auto-exposure (`:152-157`);
  (6) a timer at fps waits for frames, aligns, and publishes RGB, depth and CameraInfo with **one shared stamp taken
  from the ROS clock at publication** (`stamp = self.get_clock().now()`, `:203-210`) — not the sensor timestamp;
  (7) CameraInfo: plumb_bob with the device coefficients (`:58-78`).
* **ROS interface.** Node `realsense_camera`; publishes `/camera/color/image_raw` (Image rgb8),
  `/camera/depth/image_rect_raw` (Image 16UC1 mm), `/camera/color/camera_info` (CameraInfo); frame
  `camera_color_optical_frame`. Parameters: `serial_no` '', `color_width` 1280, `color_height` 720, `depth_width`
  1280, `depth_height` 720, `fps` 30, topic names, `frame_id`, `write_camera_json` True (`:86-96`); the launch passes
  `camera_width` 1280, `camera_height` 720, `camera_fps` 30 (`pointcloud.launch.py:427-438`).
* **No depth post-processing** (no spatial/temporal/hole-filling filters, no advanced-mode preset) is configured.
* **Thesis.** ch5 §5.1 (camera row), §3.10.1; the stamping caveat in §3.4.2 / §3.5.2.
* **Fact check.** ch5 "Intel RealSense D435i … RGB-D, 1280×720" CONFIRMED (`:2`, defaults). Note: the SDK reports
  the device name as "Intel RealSense D435" (notebook cell 3 output); the docs and code call it D435i. Figure
  caption "image (30 Hz, 0.1 s late)": 30 Hz CONFIRMED (fps 30); "0.1 s late" is an estimate from the notes
  (`notes/thesis_notes.md:127`), not a measurement in code.

#### `scripts/realsense_sim_camera_node.py` — Isaac Sim camera relay (digital twin)
* Relays Isaac's clean `/camera/color/image_raw_sim` and `/camera/depth/image_rect_raw_sim` to the real topic names
  with depth noise **σ(z) = `depth_noise_std_m` + `depth_noise_z2_coeff`·z²** (defaults 0.02 m and 0.005 m⁻¹), optional
  RGB noise (`rgb_noise_std` 0), optional frame-id rewrite (`output_frame_id`) and synthesized CameraInfo
  (`publish_camera_info` False; `fx` 919.469, `fy` 918.938, `cx` 650.852, `cy` 350.620 = the real camera's) (`:1-40,
  102-129`). Used by `digital_twin_pointcloud.launch.py`. **Not in the draft** (digital twin).

#### `scripts/camera_extrinsic_tf_publisher.py`
* Loads the 4×4 extrinsic `.npy` and broadcasts it once as a static TF `tool0 → camera_color_optical_frame`
  (`StaticTransformBroadcaster`) (`:1-59`). Parameters `extrinsic_path` '' (launch default
  `notebooks/T_tcp_to_cam.npy`), `parent_frame` `tool0` (launch arg `extrinsic_parent`, or `flange`), `child_frame`.
* **Thesis.** §3.3 (fig:frames). **Fact check** fig:frames "base_link -- tool0 -- camera_color_optical_frame --
  object": CONFIRMED; the object frames are `sam6d_object` (ICP node, `icp_pose_refiner_node.py:355`) and `fp_object`
  (tracker, `fp_tracker_node.py:170`); the URDF root is `world` (`urdf/ur5e.urdf.xacro:17`); the driver's TCP pose
  is in the UR `base` frame = base_link rotated 180° about z (`table_touchoff.py:34-36`); `tool0_controller` is the
  controller's TCP frame used by `touch_probe.py`.

#### `scripts/foundationpose_bridge_node.py` — laptop ↔ pose server
See §A.2 step 0 and step 8 for the flow. Node `foundationpose_bridge`; subscribes `/camera/color/image_raw`,
`/camera/depth/image_rect_raw`, `/perception/sam2_click` (String JSON); publishes `/perception/detections`
(`vision_msgs/Detection3DArray`, camera frame, stamped with the **registered frame's** stamp), `/perception/object_name`
(String, latched), `/perception/frozen_rgb` (Image bgr8, latched); services `~/trigger`, `~/capture`,
`~/clear_click`, `~/add_model` (all std_srvs/Trigger) (`:248-344`). Parameters (`:248-281`): `server_url`
`http://127.0.0.1:5000/predict_pose`, topics, `click_max_age_sec` 120.0, `camera_frame`
`camera_color_optical_frame`, `object_id` '1', `use_ppf_name_as_class_id` True, `model_ply_path` '',
`add_model_url` '' (→ `/add_model` on the same server), `request_timeout_sec` 300.0 (launch arg
`foundationpose_timeout_sec` 300.0), `max_sync_delta_sec` 0.25, `min_score` 0.0, `auto_trigger` False, `results_dir`
'' (→ `scripts/foundationpose_results`). Transport: HTTP multipart POST via `sam6d_io.post_files` (urllib). The class
`FrozenFrame` keeps the PNG bytes and the stamp; a stale click (other stamp or older than 120 s) is dropped
(`:426-445, 585-608`). Dated: the docstring's "Freeze, then point, then segment" rationale; README/notes
2026-10-06: detections stamped with the registered frame's stamp so the seed's TF is exact
(`notes/thesis_notes.md:491-496`). **Thesis.** §3.5.1-3.5.2, §3.10.1. **Fact check:** see §A.6. The copy in
`scripts/scripts_in_foundationpose/` is an older version (no capture/click flow) — do not quote it.

#### `scripts/fp_tracker_node.py` — live FoundationPose tracking client
See §A.4. Node `fp_tracker`; subscribes RGB, depth (sensor QoS), `/camera/color/camera_info`,
`/perception/detections`; publishes `/perception/fp/pose` (PoseStamped), `/perception/fp/status` (String JSON, 2 Hz),
`/perception/fp/marker` (Marker, latched), TF `<camera frame> → fp_object`; services `~/start`, `~/stop`, `~/reseed`
(`:243-255`). Parameters (`:160-202`): `track_url` `ws://127.0.0.1:5001`, `register_url`
`http://127.0.0.1:5000/predict_pose`, topics, `base_frame` `base_link`, `child_frame` `fp_object`, `auto_start`
True, `max_seed_age_sec` 30.0, `latched_grace_sec` 3.0, `max_sync_delta_sec` 0.005, `max_in_flight` 2, `refine_iter`
2, `rgb_encoding` `raw`, `jpeg_quality` 90, `depth_encoding` `raw`, `use_roi` False, `roi_margin_px` 40,
`ego_motion` True, `fit_tol_m` 0.010, `fit_lost` 0.5, `fit_recover` 0.6, `lost_frames` 3, `reseed` True,
`reseed_interval_sec` 3.0, `request_timeout_sec` 60.0, `mesh_marker` True, `mesh_resource`
`package://admittance_control/models/{object}.ply`, `mesh_scale` 0.001. Libraries: `websocket-client`
(`python3-websocket`), `fp_stream` (identical copy). Dated: 2026-10-09 re-seed refusal of a different part
(`:590-601`); 2026-10-06 raw encoding measurement (`:183-188`). **Thesis.** §3.5.2, fig:tracking_split, §3.10.

#### `scripts/icp_pose_refiner_node.py` — registration refinement, tracking, assembly, seams, multi-view (2232 lines)
* **Purpose.** The central perception/assembly node: ICP seed of a registration, CPU ICP tracking (legacy source),
  consumption of the FoundationPose tracker, robust save of a resting part into the static environment (SEPC),
  seams and tacks (mode A), assembly mesh export, environment reset, and the host of `~/refine_pose` (multi-view,
  implemented in library `multiview_service.py`). Started by both perception launches.
* **How it works (as implemented).**
  1. *Inputs*: the organized cloud `/camera/depth/color/points` (from depth_image_proc) with its own callback group so
     a long tick never blocks the newest cloud (`:578-590`); TF via `tf2_ros.Buffer`.
  2. *`~/run_icp` (Phase 1)* (`:741-765, 1400-1446`): read `detection_pem.json` (pose, R and t in mm) and
     `detection_ism.npz` (mask) from `results_dir` (`_load_detection`, `:1363`); load the CAD named by `obj_name`
     (`model_dir/<name>.ply`, fallback `model_path`, `:679-735`); cut the scene with the mask from the **latest live
     cloud** (`scene_from` `pointcloud`, or `depth_png` = the exact frame sent to the server); subtract the SEPC;
     voxel 6 mm, ≤ 15 000 points; point-to-plane ICP (`icp.icp_point_to_plane`, library) with Welsch robust
     weights, Anderson acceleration depth 5, max correspondence 20 mm, 30 iterations, **normal gate 60°**; seed the
     tracker. With `tracking_source` = `fp` the ICP tick then idles (`:755-757`).
  3. *ICP tracking (`tracking_source` = `icp`)* (`:1565-1631`, docstring `:17-41`): every tick at
     `tracking_rate_hz` (node default 10 Hz; `pointcloud.launch.py` passes 15 Hz) crop the live cloud with a box
     around the current pose (model AABB + 30 mm), computed first as an image rectangle (`roi_crop`), ground cut
     (flat `ground_z_m` −0.10 m, or the touched plane + 2 mm), background subtraction against the SEPC within 5 mm,
     Open3D voxel + normals, Fast & Robust ICP from the current pose; fitness < `lost_fitness` 0.1 pauses tracking;
     "Measured on a 640x480 cloud: ~74 ms/tick before, ~23 ms after" (`:40`).
  4. *FoundationPose tracking (`tracking_source` = `fp`, default since 2026-10-06)* (`:1809-1863`): every
     `/perception/fp/pose` is put into `static_frame` (base_link) **with TF at its own image stamp** and appended to a
     window of 75 poses (~3 s at ~25 Hz); a new bridge detection resumes following and clears the window; the
     tracked part is shown as the green model cloud at ≤ 10 Hz.
  5. *`~/save_object`* (`:828-853, 2046-2148`): (a) pose = robust mean of the poses since the part came to rest
     (library `pose_stats.stationary_tail`: a new resting place starts at a jump > 8 mm / 3°; `robust_pose_mean`:
     per-axis **median translation**, **chordal mean** rotation, outliers beyond 3.5 MAD or 3° rejected); at least
     `save_pose_min` = 5 poses, else the last pose is saved with a warning; the FoundationPose source refuses a last
     pose older than 1.0 s; (b) fp source only: **ICP on 5 fresh live clouds** from the averaged pose (each through
     crop, ground cut, background subtraction, normal gate), averaged, applied only if fitness ≥ 0.2 and the
     correction ≤ 20 mm / 8°; for an uncut tube (registry symmetry axis) only the **swing** of the correction is kept,
     the tracker's spin is retained (`remove_twist`) (`:1933-2027`); (c) the CAD points at that pose are appended to
     the SEPC in base_link, the pose, its statistics, the ICP info, the tracker pose before ICP and the camera pose
     are written to `assembly.json`, `static_env.ply/.npy` saved, tracking stopped (`:2093-2147`).
  6. *`~/welding_points`* (`:1093-1210`): mode A (`weld_method` `registration`): refuse stale registry entries
     (`part_shapes.stale_entries`, 0.5 mm), place each part's registry primitive at its saved pose
     (`seam_from_registration.posed_parts`), access rule with `weld_pose_tol_mm` (default 10, launch arg 10.0,
     clipped 0.5-30 per the launch description), **settle the spin of cut tubes** on their base
     (`curved_seams.settle_cut_spins`; the settled pose is written back to `assembly.json` and the SEPC), compute seams
     (every candidate kept with its reject reason and fit-up) and tacks (`tackrule-0.1`); publish
     `/perception/icp/welding_points`, `/perception/icp/welding_tacks`, `/perception/icp/welding_tack_labels`;
     write `welding_seams.json`, `welding_tacks.json`, `welding_points.npy/.ply`. Fallback: radius-PCA detector
     (`weld_radius_m` 6 mm, `weld_curvature_thresh` 0.03, `weld_voxel_m` 4 mm) when a part has no registry entry
     (`weld_fallback_pca` True).
  7. *`~/refine_pose` / `~/abort_refine`* (`:636-660, 855-911`): the multi-view refinement (`MultiviewRefiner`,
     library), which declares its own `mv_*` parameters on this node (library `multiview_service.py:58-88`: 4 views at
     0.40 m, elevations 45°/60°, azimuth step 30°, max incidence 60°, 16 frames per view, `max_correction_mm` 10,
     `max_correction_deg` 5 (4 until 2026-10-15), prior 3 mm / 1°, `max_relative_mm` 2 / `deg` 1, `online_selfcal`
     True, `quality_level` C, voxel 3 mm) and publishes `/perception/icp/multiview_views`,
     `/perception/icp/multiview_cloud`; `_apply_multiview` writes the accepted poses back (`:883-911`). A failure to
     import the module leaves the node running without it (dated 2026-10-01, `:642-657`).
  8. *`~/export_mesh`* (`:913-1015`): boolean union of the saved CADs (trimesh + manifold3d) into a watertight
     `assembly_mesh.ply` in `save_dir` **and** `model_dir` (both must exist for the next classification to resolve).
  9. *`~/reset_environment`* (`:1017-1085`): drop SEPC/saved list/tracking, archive `static_env.*`, `assembly.json`,
     `welding_*`, `assembly_mesh.ply` into `previous_assemblies/<timestamp>/`.
* **ROS interface.** Node `icp_pose_refiner`. Subscribes `/camera/depth/color/points` (PointCloud2); with fp:
  `/perception/fp/pose`, `/perception/fp/status`, `/perception/detections` (latched); clients `/fp_tracker/stop`,
  `/fp_tracker/start`. Publishes (latched) `/perception/icp/scene_cloud`, `/model_cloud`, `/refined_pose`,
  `/crop_box`, `/static_env`, `/welding_points`, `/welding_tacks`, `/welding_tack_labels`; TF
  `camera_color_optical_frame → sam6d_object`. Services `~/run_icp`, `~/stop_tracking`, `~/start_tracking`,
  `~/save_object`, `~/welding_points`, `~/export_mesh`, `~/reset_environment`, `~/refine_pose`, `~/abort_refine`.
  Main parameters with defaults (`:344-483`): `n_model_points` 2500, `voxel_size_m` 0.006, `max_target_points` 15000,
  `max_corr_dist_m` 0.02, `max_iter` 30, `anderson_depth` 5, `robust` True, `normal_gate_deg` 60.0, `use_open3d`
  True, `roi_crop` True, `noise_floor_refresh` 60, `crop_margin_m` 0.03, `tracking_rate_hz` 10.0, `save_pose_window`
  30, `save_pose_min` 5, `tracking_source` `fp`, `fp_save_pose_window` 75, `fp_max_pose_age_s` 1.0, `fp_display_hz`
  10.0, `fp_save_icp` True, `fp_save_icp_frames` 5, `fp_save_icp_min_fitness` 0.2, `fp_save_icp_max_mm` 20.0,
  `fp_save_icp_max_deg` 8.0, `auto_track` True, `lost_fitness` 0.1, `min_scene_points` 50, `static_frame`
  `base_link`, `bg_subtract` True, `bg_subtract_dist_m` 0.005, `ground_removal` True, `ground_z_m` −0.10,
  `ground_plane_file` '' (launch: `notebooks/table_plane.json`), `ground_plane_offset_m` 0.002, weld parameters
  (`weld_method` `registration`, `weldgen_path` `/workspaces/welding_cell_ws/weld_generator`, `weld_pose_tol_mm`
  10.0, `weld_seam_density_per_mm` 1.0, `weld_fallback_pca` True, `weld_tacks` True).
* **Libraries.** NumPy ICP (`admittance_control/icp.py`), Open3D (voxel + normals on the crop, optional), SciPy
  cKDTree (SEPC subtraction), trimesh + manifold3d (export), tf2.
* **Dated decisions in the code.** 2026-09-25 jitter → robust mean (`:388-392`); 2026-09-29 normal gate (`:369-373`);
  2026-10-01 optional multi-view import (`:645-647`); 2026-10-06 fp default and save-ICP ("the pure FoundationPose
  saves interpenetrated by 6-11 mm and refine_pose could not recover them", `:395-417`); 2026-10-09 R2 spin ("ICP
  'corrected' 30.8 deg of it", `:2006-2007`); 2026-10-14 tracker pose kept for analysis ("C1 moved 13.1 mm at rmse
  3.6 mm", `:2073-2074`); 2026-10-15 stale registry (`:1166-1167`) and E2 spin "36 deg wrong" (`:1213-1216`).
* **Thesis.** §3.5.2-3.5.4, §3.6 (host), §3.7, §3.8, §3.10.1.
* **Fact check.**

| draft statement | verdict | evidence |
|---|---|---|
| "tracked while it is placed and saved when it rests" | CONFIRMED | §E.1 steps 4-5. |
| "averaged over the still frames (median translation, chordal mean of the rotations)" | CONFIRMED | `pose_stats.robust_pose_mean` (library), called at `:1886-1899, 2079-2092`; add the outlier rejection (3.5 MAD, 3°) and the resting-place detection (8 mm / 3°). |
| "and refined by point-to-plane ICP against the scene" | CONFIRMED (fp source) | on 5 fresh clouds, applied only within 20 mm / 8° and fitness ≥ 0.2 (`:1933-2027`); motivation 2026-10-06. |
| Eq. (p2plane) with Welsch weights | CONFIRMED (library-side) | `robust` True, "Welsch-reweighted, dynamic-nu, Anderson-accelerated" (`:21-22`). |
| Eq. (normal_gate) cos 60° | CONFIRMED | `normal_gate_deg` 60.0 (`:373`). |
| "The saved part becomes part of the static environment that the next part is registered against" | PARTLY | the SEPC is **subtracted** from the scene (points within 5 mm deleted) so ICP "goes blind to already-assembled parts" (`:42-51, 432-434`); the next part is *not* registered against it (FoundationPose registers on the mask). Rephrase. |
| §3.5.4 "only the swing is kept" for a plain tube | CONFIRMED | `remove_twist` at save (`:2002-2013`); also the multi-view rotation limit judges only the axis tilt (library `multiview_refine.judge`). |
| §3.5.4 mitre spin from the cut face sitting flat (Eq. spin_settle) | CONFIRMED (library-side) | `curved_seams.settle_cut_spins` minimises the gap range (`:1212-1248`, `gap_range_before/after_mm`); applied inside `~/welding_points`, not at save. |
| §3.5.2 "Lost-track detection and re-registration locked to the tracked model's name" | CONFIRMED (tracker) | the ICP node itself only has `lost_fitness` 0.1 for ICP tracking. |
| §3.7 "Seams are not detected in the point cloud: they are computed from the registered CAD models" | CONFIRMED with fallback | mode A default; the radius-PCA *detector* on the SEPC remains as fallback (`:1130-1142, 1274`). |
| "the rule uses a pose tolerance instead of a contact tolerance" | CONFIRMED | `weld_pose_tol_mm` 10 (`:473`). |
| §3.7.4 "a seam whose gap exceeds what a welder could bridge is reported, not silently dropped" | CONFIRMED | rejected candidates kept with reason (`:144-146`); launch description: "a curved pair within twice it is reported as fitup_beyond_pose_tol" (`pointcloud.launch.py:404-409`). |
| §3.8 tack order "ends first, then by bisection, alternating between seams" | CONFIRMED on the data | `welding_tacks.json`: order s0#1, s1#1, s0#3, s1#3, s0#2, s1#2 (`tackrule-0.1`, tack length 4t, end margin 2t). |
| §3.6.5 acceptance: ">10 mm or >5° (a pipe: the tilt of its axis, not its spin)" | CONFIRMED (library-side) | `multiview_service.py:80-81`; `multiview_refine.py:885-913`. |
| "Overlap with neighbours: worse than the saved assembly's by more than 0.2 mm" | PARTLY | refused only if the overlap exceeds `max_penetration_mm` = 1.0 mm **and** the saved one + 0.2 mm (`multiview_refine.py:917-925, 127`). |
| "Fit: *to fill*" | to fill with | fitness < `min_fitness` 0.2 → refused; relative pose of two touching parts changed by > 2 mm / 1° along a direction the views did not measure → refused (`multiview_refine.py:915-940, 126-129`). |
| §3.6.3 observability σ ≤ 0.5 mm | CONFIRMED (library-side) | `observable_max_sigma_mm` 0.5 (`multiview_refine.py:104`); extrinsic directions also 0.5 mm (`:109`). |
| §3.6.4 online extrinsic translation | CONFIRMED (library-side) | `online_selfcal` True; `online_selfcal_min_mm` 0.5 (`multiview_refine.py:116`). |
| §3.10.1 TODO "services as the steps of the pipeline; the static environment cloud" | data | services listed above; SEPC on `/perception/icp/static_env`. |

#### `scripts/detection_marker_node.py`
Detection3DArray → MarkerArray: per detection a triad, a sphere, a text "<class_id>: <score>", and for the best one the
oriented bounding box of `bbox_model_path` (`:1-35`). Node `detection_marker`; subscribes `/perception/detections`
(latched), publishes `/perception/detection_markers` (latched); parameters `min_score` 0.0, `axis_length_m` 0.05,
`line_width_m` 0.005, `sphere_diameter_m` 0.01, `text_height_m` 0.02, `bbox_model_path` '' (launch default
`models/test_objv2_base.ply`), `bbox_model_units` mm, `bbox_line_width_m` 0.004 (`:106-116`). RViz helper; not in
the draft (optional figure aid). Caveat in the launch: the box uses a fixed CAD, so it is drawn wrong for other
classified parts (`pointcloud.launch.py:248-252`).

#### `scripts/sam6d_bridge_node.py` (legacy) and `helper/calibration/sam6d_client.py` (legacy)
The pre-FoundationPose bridge: `~/trigger` captures a synced pair, POSTs rgb/depth/camera.json to the SAM-6D server,
converts mm → m, publishes `/perception/detections` (`:1-35, 78-120`; parameters `server_url`, `request_timeout_sec`
120.0, `max_sync_delta_sec` 0.25, `min_score` 0.0, `auto_trigger` False, `results_dir` ''). Its artefacts are in
`scripts/sam6d_results/` (`assembly.json`, `detection_ism.json/.npz`, `detection_pem.json`, `static_env.*`,
`welding_points.*`). `sam6d_client.py`: a stand-alone pyrealsense2 test client (640 × 480) for the same server.
Thesis: one sentence of history at most.

#### `scripts/rgb_depth_to_send/`
The transfer directory (`sam6d_io.resolve_transfer_dir`): the last frozen `rgb.png` (1280 × 720 × 3, uint8),
`depth.png` (1280 × 720, uint16 mm) and `camera.json` (`cam_K` fx 919.469, fy 918.938, cx 650.852, cy 350.620;
`depth_scale` 1.0). Installed to share (`CMakeLists.txt` `install(DIRECTORY … scripts/rgb_depth_to_send …)`).
`track_bench.py`/`track_replay.py` use the same layout on the server (`Data/Input`).

### E.2 Operator interaction

#### `scripts/gesture_control_node.py`
* **Purpose.** Hands-free console: MediaPipe gestures on the RealSense colour stream → Trigger calls on the bridge and
  the ICP node; the index fingertip → SAM2 click points (dwell) or camera aim (`/gesture/aim`). Started by
  `gestures.launch.py`.
* **How it works.**
  1. Recognizer (`_build_recognizer`, `:393-406`): MediaPipe Tasks `GestureRecognizer`, model
     `gesture_recognizer.task` (the stock float16 v1 bundle, `notebooks/gesture_recognizer.task`, 8.4 MB,
     2026-07-27), `RunningMode.VIDEO`, `num_hands` 1, `min_hand_detection_confidence` = `min_tracking_confidence` =
     `min_hand_confidence` 0.6. Per frame: top gesture name + score, and landmark 8 (index fingertip) in pixels
     (`_recognize`, `:408-426`).
  2. **Recognition runs on the live stream** `/camera/color/image_raw`; while a frame is frozen the window shows the
     frozen image and the live fingertip pixel is transferred to it (same camera, same size check, `:671-685`).
  3. Latch (`GestureLatch`, `:125-203`): a gesture fires once when it has held `stable_frames` = 6 consecutive frames
     and `cooldown_sec` (node default 1.0 s; the dataclass default 0.4 s) has passed since the last firing; re-armed by
     a neutral frame **or a stable run of a different gesture** (Pointing_Up → Thumb_Up often slides without None);
     the robot-moving gesture (`Closed_Fist` in fp mode) must additionally be held `motion_hold_sec` = 2.0 s; an on-screen
     progress bar shows the hold.
  4. Dwell cursor (`DwellCursor`, `:205-256`): node `dwell_sec` 1.0 s (dataclass 0.8), `move_tol_px` 18 (movement
     beyond restarts the timer), `keep_tol_px` 36 (a new latch farther than this keeps the previous pending point as
     positive).
  5. Dispatch (`:518-602`), fp mapping (`FP_GESTURE_TO_ACTION`, `:112-116`):

| gesture | no frame frozen | frame frozen (segmenting) | service called |
|---|---|---|---|
| Pointing_Up | aim (after `aim_frames` = 3 consecutive frames; not while a `refine_pose` call is in flight) | dwell → pending point | publishes `/gesture/aim` (PointStamped: x = u, y = v, z = score, header = the **image** header) |
| ILoveYou | capture | re-capture (a new frozen frame invalidates all points) | `/foundationpose_bridge/capture` |
| Thumb_Up | message "nothing frozen" | pending → positive; publish all points on `/perception/sam2_click`; 0.15 s later trigger | `/foundationpose_bridge/trigger` |
| Thumb_Down | cancel: release + clear the bridge's frame and points | pending → negative (needs a pending point) | `/foundationpose_bridge/clear_click` |
| Open_Palm | welding_points | undo (pending, else last kept point) | `/icp_pose_refiner/welding_points` |
| Closed_Fist (held 2 s) | refine_pose (the robot moves) | same | `/icp_pose_refiner/refine_pose` |
| Victory | save object | same | `/icp_pose_refiner/save_object` |

     With `tracking_source` = `icp`: Closed_Fist → `run_icp`, Open_Palm → `stop_tracking` (`GESTURE_TO_ACTION`,
     `:102-110`). Calls are asynchronous; the status line echoes every reply; after a successful trigger (or a clear)
     the frozen frame is released (`:458-495`).
* **ROS interface.** Node `gesture_control`; subscribes `/camera/color/image_raw`, `/perception/frozen_rgb`
  (latched); publishes `/perception/sam2_click` (String JSON), `/gesture/aim` (PointStamped); clients listed above.
  Parameters (`:293-324`): `model_path`, `rgb_topic`, `frozen_rgb_topic`, `click_topic`, `bridge_ns`
  `/foundationpose_bridge`, `icp_ns` `/icp_pose_refiner`, `tracking_source` `fp`, `motion_hold_sec` 2.0,
  `stable_frames` 6, `cooldown_sec` 1.0, `dwell_sec` 1.0, `move_tol_px` 18, `min_hand_confidence` 0.6,
  `display_scale` 1.0, `show_window` True, `aim_enabled` True, `aim_topic` `/gesture/aim`, `aim_frames` 3.
* **Dated.** "Open_Palm … false-fires (20+ times on 2026-10-06); here that only recomputes the seams" (`:53-56`);
  fp mapping default since 2026-10-06 (`:45`).
* **Prototype**: `notebooks/gesture_control_sandbox.ipynb` (2026-07-27: recognizer, debounce/cooldown state
  machine, dwell cursor, wiring), `notebooks/media_pipe_sandbox.ipynb` (MediaPipe tutorial).
* **Thesis.** §3.4.1.

#### `scripts/camera_aim_node.py` (+ library `admittance_control/camera_aim.py`)
* **Purpose.** Turns the wrist camera about its optical centre until the fingertip is centred; started by
  `gestures.launch.py`; dry run by default as a node, **live by default through the launch** (`aim_dry_run` false).
* **How it works.** (1) `/gesture/aim` gives (u, v) with the image stamp; the camera orientation at that stamp is
  interpolated from a joint-state history (`JointHistory`, `:193, 209-219`; a warning if no joint state covers the
  stamp: "latency not compensated"); (2) the pixel ray K⁻¹[u v 1] is put into base_link with that pose and filtered
  there by a One-Euro filter (min cut-off 1.0 Hz, β 5.0, derivative cut-off 1.0 Hz); (3) error in the current camera
  frame, deadband 5 % of the image width with release at 0.4 × that (hysteresis); (4) P law, gain 2.5 /s, roll held
  to the home view with gain 1.5 /s; (5) rate cap 15°/s, acceleration cap 60°/s², two smoothing lags of 0.06 s;
  (6) excursion limits ±35° pan / ±25° tilt from the home view, looked ahead by 0.35 s ("~ the stopping time from
  max_rate"); (7) flange velocity holds the optical centre with k_p 2.0 /s (≤ 0.02 m/s); DLS with λ 0.02 on the
  calibrated Jacobian; joint speed ≤ 0.5 rad/s (library `camera_aim.py:47-72`); (8) at `rate_hz` = 125 Hz publishes
  joint rates on `/camera_aim/joint_rates` (Float64MultiArray) for the relay; (9) on the first aim message, with the
  arm still (< 0.02 rad/s) and the camera within 5 cm of its home position (`home_q` of `marking.json`), switches the
  active trajectory controller to `forward_velocity_controller` (`controller_manager_msgs/SwitchController`), and
  back once still for `idle_s` 0.3 s.
* **Safety every cycle** (docstring `:22-34`): deadman `deadman_s` 0.3 s (no fingertip → decelerate to stop; any
  other gesture does the same); excursion limits; the pose `check_ahead_s` 0.3 s ahead against the collision model
  (table plane and fixtures of `marking.json`, the arm and the pen tool — **no parts**, none is registered before the
  capture, `:122-126`), the branch lock and the wrist singularity (|sin q5| ≥ sin 0.35 rad); |F| > `abort_force_n` 15 N
  (biased at the start); stale joint states; the pendant speed slider scales everything; a refusal latches until the
  hand has been away 1 s.
* **ROS interface.** Node `camera_aim`; subscribes `/gesture/aim`, `/camera/color/camera_info`, `/joint_states`,
  `/force_torque_sensor_broadcaster/wrench`; publishes `/camera_aim/joint_rates` (+ status); clients
  `/controller_manager/list_controllers`, `/controller_manager/switch_controller`. Parameters (`:94-112`):
  `dry_run` True, topics, `controller_manager`, `velocity_controller` `forward_velocity_controller`,
  `marking_config` '', `extrinsic_path` '', `kinematics_file` `config/ur5e_calibration.yaml`, `rate_hz` 125.0,
  `deadman_s` 0.3, `idle_s` 0.3, `home_tol_m` 0.05, `still_rad_s` 0.02, `abort_force_n` 15.0, plus the `AimConfig`
  fields `gain_per_s` 2.5, `deadband_frac` 0.05, `max_rate_deg_s` 15.0, `max_acc_deg_s2` 60.0, `pan_limit_deg` 35.0,
  `tilt_limit_deg` 25.0, `max_joint_speed_rad_s` 0.5, `check_ahead_s` 0.3.
* **Dated.** 2026-10-15 design (exact latency compensation; gain raised from 1.2 /s to 2.5 /s,
  `notes/thesis_notes.md:123-160`); 2026-10-16 buzz diagnosis (50 Hz command staircase, peak at 49-51 Hz, effort spikes
  ~8 ms after every 20 ms step; "a 64-px (≈ 4°) deadband on the 1280-px image", `notes/thesis_notes.md:54-80`;
  README: "a 50 Hz line in the joint velocities and 3.4× the idle effort ripple", `README.md:450`).
* **Thesis.** §3.4.2.

#### `src/aim_rate_relay.cpp`
* Node `aim_rate_relay` (C++, `-O2`); subscribes `/camera_aim/joint_rates`, publishes
  `/forward_velocity_controller/commands` (Float64MultiArray) at `rate_hz` 500 Hz; parameters `timeout_s` 0.1,
  `max_speed_rad_s` 0.6, `max_acc_rad_s2` 4.0, `stop_acc_rad_s2` 2.0, `n_joints` 6 (`:39-46`).
* Per input: start = the value being sent now, goal = the new input; the input period T is measured (EMA 0.8/0.2 of
  consecutive gaps, clamped 2-50 ms, initial 1/125 s) (`:62-77`). Per 500 Hz tick: want = start + clamp((t − t_k)/T,
  0, 1)·(goal − start), then each joint's change per tick clamped to max_acc·dt and its value to ±0.6 rad/s; no input
  for 0.1 s → ramp to zero at 2 rad/s² and go quiet; malformed inputs dropped (`:79-110`). "Cost: one input period of
  delay (8 ms at 125 Hz)" (`:8-9`).
* **Fact check** Eq. (relay): PARTLY — the ramp starts from the value being commanded when the input arrives (equal to
  q̇_{k−1} only if the previous ramp completed) and is acceleration- and speed-clamped; "stops the arm by itself when
  the commands stop" CONFIRMED (0.1 s timeout).

#### `launch/gestures.launch.py`
Arguments: `model_path` (default `notebooks/gesture_recognizer.task` from the source tree), `tracking_source` `fp`,
`aim` true, `aim_dry_run` **false**, `display_scale` 1.0; the gesture window closing (`on_exit`) shuts down the launch
(`:86-101`).

#### `scripts/aim_bag_report.py` (not installed)
Offline analysis of a rosbag of `/joint_states` (500 Hz with effort) and `/forward_velocity_controller/commands`:
command rate and step size (wrist_1), the 20-250 Hz effort content while aiming vs. idle (the "buzz index", 1 = as
quiet as standing still), and a vibration line > 30 Hz standing out ≥ 3× from its neighbours (`:1-28`). "The first
bag, the 50 Hz direct stream: ros2_ws/rosbag2_2026_10_16-13_49_03". Thesis: evidence for §3.4.2 "Command
interpolation" and a ch6 measurement.

#### Fact check of §3.4

| draft statement | verdict | evidence / correction |
|---|---|---|
| "The operator never touches a keyboard" | PARTLY | true for the perception cycle (aim → capture → points → register → save → refine → seams); `export_mesh`, `reset_environment`, `tack_reachability.py`, the marking node's `~/plan`/`~/next`/`~/all` and all launches remain service calls / CLI (`README.md:1584-1586`; §E.3). |
| "MediaPipe's gesture recogniser classifies seven static gestures per frame" | CONFIRMED | the stock bundle's 7 gestures (`gesture_control_node.py:100-110`), plus "None". |
| "A gesture acts only after it has been stable for a number of frames (a latch)" | CONFIRMED | 6 frames + 1.0 s cooldown; re-armed by neutral or a different stable gesture. |
| "a gesture that moves the robot must be held for two seconds" | PARTLY / WRONG as stated | only Closed_Fist (refine_pose) has the 2 s hold; **Pointing_Up moves the robot (the aim) after 3 consecutive frames** (`aim_frames` 3) with its own deadman and safety gates. |
| "the index fingertip is a pixel of the very frame that is segmented" | PARTLY | the fingertip is detected on the **live** frames of the same camera and transferred to the frozen frame (same size; the camera does not move while segmenting, aim disabled). "Same camera, same pixel grid" is the accurate claim. |
| Table tab:gestures | CONFIRMED with three corrections | (i) Pointing_Up (frozen): a dwell latches a *pending* point; pointing elsewhere keeps the previous one as positive; (ii) Thumb_Down (no frame): "cancel: drop the bridge's held frame and points"; (iii) Open_Palm (no frame) = `welding_points` only in fp mode; in icp mode Closed_Fist = run ICP, Open_Palm = stop tracking. |
| "The motion is restricted to a rotation about the camera's own optical centre" | CONFIRMED | `camera_aim_node.py:4-6`; the centre is held by a proportional pull (k_p 2.0 /s). |
| "delivered about 0.1 s later" | CONFIRMED as an estimate | `notes/thesis_notes.md:127`; note that the stamp is the publication time of the camera node. |
| "with the camera orientation at t_k (from the joint-state history)" | CONFIRMED | `JointHistory` (`:193, 209-219`). |
| "filtered there by a One-Euro filter" | CONFIRMED | min cut-off 1.0 Hz, β 5.0, d cut-off 1.0 Hz (library `camera_aim.py:70-72`). |
| Eq. (aim_law) ω_C = (−K e_y, K e_x, −k_r ψ) | CONFIRMED in form | K = 2.5 /s, k_r = 1.5 /s; deadband 5 % with release 40 % of it; rate 15°/s; acc 60°/s²; two lags 0.06 s. |
| "No integral term … and no derivative term" | CONFIRMED | "a filtered P loop, not a PID" (`README.md:450`); 2026-10-16 note: "adding I or D would not have changed it". |
| Eq. (aim_dls) | CONFIRMED | k_p 2.0 /s, λ 0.02 (library). |
| "streams to the UR driver's velocity controller, which the aim switches in and out by itself" | CONFIRMED | via the relay; `SwitchController` scaled JTC ↔ `forward_velocity_controller`. |
| "the pose 0.3 s ahead is checked against the collision model, the branch lock and the wrist singularity" | CONFIRMED, incomplete | also excursion limits (±35°/±25°), |F| > 15 N, stale joint states; the collision model has the table, fixtures, arm and tool but **not the parts**. |
| "a fingertip lost for 0.3 s ramps the rate to zero" | CONFIRMED | `deadman_s` 0.3. |
| "The control law runs at 125 Hz (one cycle costs 2.5 ms in Python); the controller runs at 500 Hz" | CONFIRMED | `camera_aim_node.py:11-12`; relay 500 Hz. |
| fig:mc_aim_loop "image (30 Hz, 0.1 s late) … 125 Hz -> relay 500 Hz" | CONFIRMED | as above. |
| TODO "Deadband with hysteresis; the rate and acceleration limits and the two-lag smoother; the excursion limits looked ahead by the stopping distance" | data | 5 % / 0.4×; 15°/s, 60°/s²; 2 × 0.06 s; ±35°/±25° looked ahead 0.35 s. |

### E.3 Seams, reachability, marking, C++ components

#### `scripts/build_weldgen_registry.py`
* CLI (installed): derive / verify `models/weldgen_objects.json` from `models/*.ply` — "Box meshes become slabs,
  pipes tubes (flat, mitred or saddle-cut base), rounded-rect tubes closed swept_slabs - all fitted and verified";
  anything else "unsupported with the reason"; hand entries marked `"hand_edited": true` (`:1-12`). Options
  `--models-dir`, `--out`, `--weldgen-path` `/workspaces/welding_cell_ws/weld_generator`, `--verify`, `--tol-mm` 0.05
  (fit tolerance), `--accept-mm` 0.25 ("verification budget, both directions (the D34 chord budget)"); an entry fails
  verification if the two-way deviation > 0.25 mm or the volume error > 1 % (`:34-57`).
* Note: the docstring does not mention the open bands (SP3, 270circle are `swept_slab` "open_band" in the registry).
* **Fact check** §3.7.1 "verified both ways … within a fraction of a millimetre, and the volumes agree": CONFIRMED
  (0.25 mm, 1 %; current entries 0.04-0.07 mm, §B.2).

#### `scripts/multiview_refine_offline.py`
CLI (installed, no ROS): replays a capture in `foundationpose_results/multiview/<timestamp>/` (`capture.json`,
`view_<k>.npz` organized clouds in the camera frame, `assembly.json`) through the same refinement; `--write`
(`assembly_refined.json`), `--record` (append d to `selfcal_history.json`, only if the capture's extrinsic sha1 is the
one in use), `--set name=value` overrides, `--make-synthetic DIR [--d dx dy dz]` writes a synthetic capture of the
bench T with a known extrinsic error (`:1-18, 99-110`). Thesis: §3.6 method validation (synthetic findings
2026-10-01, `notes/thesis_notes.md:954-970`).

#### `scripts/tack_reachability.py`
CLI (installed, no ROS, no motion): reads `welding_tacks.json` + `assembly.json`, rebuilds the parts as collision
boxes through the registry, loads `pen_tool.json` and `marking.json` (home pose = branch lock), runs
`tack_reach.plan_tacks` (library), prints per seam the chosen roll and per tack the approach / tack / descent clearances
and the joint step, writes `<save-dir>/tack_reach.json` (`:1-15`). Options: `--save-dir`, `--registry`, `--tool`,
`--marking`, `--extrinsic`, `--table-z`, `--clearance`, `--roll`, `--tilt` (force a work angle), `--kinematics`
(calibrated yaml), `--out` (`:41-58`). On disk `tack_reach.json` keys: `config`, `branch_signature`, `seams`,
`tacks`, `all_ok`, `kinematics`, `source`.

#### `scripts/tack_reach_marker_node.py`
Node `tack_reach_marker`; publishes `/tack_reach/markers` (MarkerArray, latched, base_link): per tack an arrow from the
approach point to the tack along the pen axis (green reachable / red not), a label "seam.tack_no t<tilt> r<roll>",
and for reachable tacks the arm capsules (grey) and the tool envelope (blue) at the approach pose (`:1-12`); parameters
`kinematics_file`, `report` '', `frame` `base_link`, `topic`, `show_arm` True, `tool_config`, `extrinsic_path`
(`:41-48`). Figure source for fig:mc_collision / fig:mc_roll_dp.

#### `config/marking.json` (read by library `tack_reach.load_marking_config`)

| key | value | meaning (comment in the file) |
|---|---|---|
| `home_q` | [−0.3447, −1.4444, −1.5553, −1.5530, 1.9432, −0.8598] rad | "the user's elbow-up camera pose (2026-09-24, /joint_states). Its branch is the lock" |
| `branch_joints` | [1, 2, 4] | lift, elbow, wrist_2 signs kept |
| `standoff_m` | 0.035 | approach point along the pen axis |
| `departure_m` | 0.1 | transits start/end here (2026-10-14: from 35 mm APS paths round a pipe varied 0.5-1.9 m of tip travel, from 100 mm 0.19-0.21 m every time, at 3 s) |
| `clearance_m` | 0.003 | tack clearance (holder clears a 90° T by 3.1 mm at best with the old envelope; 5 mm would refuse every square fillet) |
| `transit_clearance_m` | 0.02 | free moves (2026-09-29: a 3 mm transit hit the standing plate at 9 N); drops to 90 % of an end pose's own clearance |
| `roll_step_deg` | 30.0 | roll sampling about the pen axis |
| `work_angles_deg` | [0, −10, 10, −20, 20] | tilts off the seam bisector; best (tilt, roll) per seam = largest minimum clearance, then least joint travel |
| `lin_steps` | 6 | Cartesian steps checked on the descent |
| `max_joint_step_rad` | 1.2 | largest single-joint move between consecutive tack poses of a seam |
| `wrist_singularity_margin_rad` | 0.35 | |
| `table_z_m` | null | → the touched plane of `table_plane_file` ("Before 2026-10-14 … no plane was used at all: transits ran through it") |
| `table_plane_file` | `notebooks/table_plane.json` | −96.1 mm at (−0.143, 0.426) m, tilted 1.70° (2026-10-02) |
| `fixtures` | **[]** | boxes from 3-4 pen-tip points on a fixture's top, or explicit centre/half/yaw — **none entered** |
| `wrist3_window_deg` | 180.0 | wrist_3 within ±180° of home as the 2π-equivalent; 2026-10-16 cable incident (−202°, −287°; false 1.5 N contact 84 mm above SP3) |

Library constants used with it (other reviewer): `PER_TACK_ENOUGH_CLEARANCE_M` = 0.010 and
`CLEARANCE_COST_RAD_PER_MM` = 0.15 (`tack_reach.py:396-399`), `TRANSIT_WEIGHTS` = (2, 2, 1.5, 1, 1, 0.5)
(`tack_reach.py:402`, `transit_cpp.cpp:597`), Cartesian bulges (0, 0.03, 0.06, 0.10) m (`marking.py:118`), transit
order straight edge → Cartesian line (slerp) → APS (`marking.py:242-277`).

#### `scripts/tack_marking_node.py`
* **Purpose.** Milestone 4-6 of `notes/pen_marking_plan.md`: transit, force-gated descent, dot or stroke, retract;
  run by hand (`ros2 run`), dry run by default.
* **How it works.**
  1. `~/plan` (from the current joints): load `tack_reach.json`, `welding_tacks.json`, `welding_seams.json`,
     `assembly.json`, the registry; build the collision model (parts as boxes, tilted table plane, fixtures, arm and
     tool) and the marking plan (`marking.build_marking_plan`); per tack a transit to its departure point (100 mm out
     along the pen; the 35 mm approach point when that segment is not clear), tack to tack first as a straight /
     bowed Cartesian line, APS only when blocked; corners smoothed (Catmull-Rom) and every spline sample re-checked
     (`smooth_transits` True since 2026-10-05); publishes the tip path (`~/tip_path`, Marker, latched) and writes
     `tack_marking_plan.json` (`README.md:532`).
  2. `~/next` / `~/all`: transits on `scaled_joint_trajectory_controller` (FollowJointTrajectory action), timed by
     TOTG through `/compute_totg` (`v_joint_rad_s` 0.3, `a_joint_rad_s2` 0.5; `totg_path_tolerance` 0.005 rad,
     `totg_resample_dt` 0.02 s, library `motion.py:84-86`), a trapezoidal profile when the service is down; the planned
     transit is joined from the actual joints when near its start.
  3. Descent (IK chain along the pen axis, `marking.time_descent`): departure → approach at `v_approach_m_s` 0.05 m/s;
     approach → tack at `v_tip_m_s` 0.02 m/s; the last `slow_zone_m` 6 mm at `v_slow_m_s` 0.002 m/s; on past the
     registered point by `overshoot_m` 5 mm, the overshoot ending at the table plane (2026-10-16); the wrench bias is
     re-measured at the approach point over `bias_window_s` 0.5 s; the goal is **cancelled** at |F − bias| >
     `touch_force_n` 1.5 N (from `pen_tool.json`), the joints at that moment are the contact (`:1-11, 121-126`).
  4. Mark: `stroke_mode` `dot` (dwell `dwell_s` 0.5 s) or `tack` / `seam` strokes at `v_stroke_m_s` 0.02 m/s,
     contact-referenced depth (measured contact + `press_m` 0.0) in `stroke_chunk_m` 10 mm chunks, depth corrected
     between chunks by `depth_gain_m_per_n` 2e-5 m/N × (hold − measured force along the pen), clipped to
     `max_depth_step_m` 0.2 mm; two chunks under `min_contact_force_n` 0.4 N stop it; on a curved seam each tack
     draws its own piece (`:127-137`, docstring `:30-36`).
  5. Retract along the pen; `~/home`; `~/abort` cancels the running goal.
  6. Writes `<save_dir>/tack_marks.json` (+ a copy per run in `marks_history/` with its assembly): per tack the
     commanded point, the tip at contact (`tip_contact_m`), `contact_depth_m` (+ = surface met before the registered
     point), force, `no_contact`, stroke data (`:38-43`; on disk 21 runs 2026-10-14 → 10-16).
* **Gates.** Current joints clear and on the locked branch; first trajectory point within `max_joint_jump_rad` 0.35;
  |F| > `abort_force_n` **8 N** cancels anything; `release_abort_force_n` 40 N for moves away (library `motion.py:73-82`);
  "first point within 20° of the current joints" (`README.md:532`).
* **ROS interface.** Node `tack_marking`; services `~/plan`, `~/next`, `~/all`, `~/home`, `~/abort`; publishes
  `~/tip_path`, `~/status`; subscribes `/joint_states`, `/force_torque_sensor_broadcaster/wrench`; action client
  `/scaled_joint_trajectory_controller/follow_joint_trajectory`; service client `/compute_totg`. Parameters
  (`:93-137`): `kinematics_file`, `save_dir` `scripts/foundationpose_results`, `registry`, `tool_config`,
  `marking_config`, `extrinsic_path`, `v_joint_rad_s` 0.3, `a_joint_rad_s2` 0.5, `transit_planner` `aps`,
  `transit_budget_s` **3.0** ("1.0 before 2026-10-14"), `transit_max_budget_s` 10.0, `aps_num_planners` 4,
  `aps_max_paths` 8, `aps_clearance_margin_m` 0.002, `smooth_transits` True, the descent/stroke parameters above;
  from `TrajectoryExecutor`: `dry_run` True, `action`, `wrench_topic`, `joint_states_topic`, `abort_force_n` 8.0,
  `release_abort_force_n` 40.0, `bias_window_s` 0.5, `max_joint_jump_rad` 0.35.
* **Note.** README §node table says `transit_budget_s` 2 s (`README.md:532`); the code default is 3.0 s — use the code.
* **Hint in the docstring:** a force error → `ros2 service call /io_and_status_controller/zero_ftsensor` (`:46-48`).
* **Thesis.** §3.9.3-3.9.4, ch5 §5.4 (contact depth as a measurement), ch6.

#### `src/transit_cpp.cpp` → `admittance_control._transit_cpp`
pybind11 module: "a FAITHFUL PORT of admittance_control/collision.py: the same primitives, the same distance
algorithms (closed-form segment-segment, golden-section segment-box to tol 1e-4, separating-axis box-box as 0 /
+inf), the same pairs in the same order" — proven by `test/test_transit_cpp.py` on random and near-contact
configurations (`:13-19`). Planning in scaled joint coordinates y_j = w_j q_j so OMPL's Euclidean length is the
weighted joint travel (`:21-23`); validity resolution `resolution`·w_min/extent (default 0.01 rad, `:479-480`); APS
= `og::AnytimePathShortening` with `num_planners` (4) `RRTConnect` instances, `setHybridize(true)`,
`setShortcut(true)`, `setMaxHybridizationPath(max(2, max_paths=8))` (`:493-502`); single RRT-Connect fallback;
weights default (2, 2, 1.5, 1, 1, 0.5) (`:597`); GIL released while planning. OMPL from ROS (libompl 1.7.0).
**Fact check** §3.9.1 "Distances are exact between capsules and between a capsule and a box; a box against a box is
an intersection test only": PARTLY — capsule-capsule is closed-form; **capsule-box is a golden-section search to
1e-4** (numerically, not closed-form exact); box-box is separating-axis 0/+∞ — CONFIRMED otherwise. "the C++ port of
the model, proven equal to the Python one" CONFIRMED (test). "one plan from minutes to seconds": README: "~590x
faster" (`README.md:532`).

#### `src/totg_service_node.cpp` + `srv/ComputeTOTG.srv`
Node `totg_service_node`, service `compute_totg` (`admittance_control/srv/ComputeTOTG`): request flat waypoints,
`num_joints`, per-joint `max_velocity`/`max_acceleration`, `path_tolerance` (default 0.02 rad if ≤ 0), `resample_dt`
(default 0.1 s if ≤ 0); uses MoveIt's `trajectory_processing` TOTG Path + Trajectory API directly "no robot model,
URDF, SRDF, or MoveIt planning scene required" (Kunz & Stilman 2012) (`:1-11, 72-80`); MoveIt core 2.12.4. The marking
node passes 0.005 rad / 0.02 s. Started by `pointcloud.launch.py` (`launch_totg` true) and by the legacy launch.
**Fact check** §3.9.3 "timed with time-optimal trajectory generation [Kunz]" and ch5 "MoveIt's time-optimal
parameterisation as a service": CONFIRMED.

#### `src/jacobian_calculator_node.cpp` (legacy)
Node name `sand_drawer_jacobian_node`; MoveIt RobotModel/RobotState Jacobian and manipulability for the drawing
stack (`min_manipulability` 0.02, `manipulability_gain` 0.4; subscribes `/joint_states` and an end-effector Twist)
(`:1-90`). Not used by the welding cell; not in the draft.

#### Run artefacts of marking (`scripts/foundationpose_results/`)
`tack_marking_plan.json` (`ok`, `q_start`, `home_path`, `steps`), `tack_marks.json` (`touch_force_n`, `overshoot_m`,
`sequence`, `marks`: per mark `axis_m`, `contact`, `contact_depth_m`, `dry_run`, `force_n`, `point_m`, `q_contact`,
`roll_deg`, `seam_id`, `stroke`, `stroke_mode`, `tack_id`, `tack_no`, `tack_point_m`, `tilt_deg`, `time`,
`tip_contact_m`), `marks_history/tack_marks_<stamp>.json` (21 runs 2026-10-14 14:06 → 2026-10-16 15:35, each with its
`assembly`), `tack_reach.json`. These are the raw data for ch6 and for §5.4 "Measuring the Mark Error".

#### Fact check of §3.9 (motion) not already covered

| draft statement | verdict | evidence |
|---|---|---|
| "IK … on one branch, the elbow-up configuration of the home pose (the signs of the lift, elbow and second wrist joints are locked)" | CONFIRMED | `marking.json` `branch_joints` [1, 2, 4], `home_q` (2026-09-24). |
| "On a straight seam one pen roll and tilt is chosen per seam, maximising the smallest clearance over the approach pose, the tack pose and the straight descent" | CONFIRMED | `marking.json` comment; work angles 0/±10/±20°, roll step 30°; tie-break least joint travel. |
| Eq. (roll_dp) δ0 = 10 mm, κ = 0.15 rad/mm | CONFIRMED (library) | `tack_reach.py:396-399`. |
| "Every pose is taken as the 2π-equivalent that keeps the last wrist joint within ±180° of home" | CONFIRMED | `wrist3_window_deg` 180. |
| "Transits start and end at departure points, 100 mm out along each tack's pen axis" | CONFIRMED | `departure_m` 0.1 (2026-10-14); fallback to the 35 mm approach point when the segment is blocked. |
| order: straight joint edge → Cartesian line (slerp) → bowed Bézier β ∈ {0, 30, 60, 100} mm → APS | CONFIRMED (library) | `marking.py:118-153, 242-277`. |
| APS "(parallel RRT-Connect, hybridisation and shortcutting within a time budget)" | CONFIRMED | 4 RRT-Connect, hybridize, shortcut, max 8 paths; budget 3.0 s (+ up to 10 s for a first path). |
| W = diag(2, 2, 1.5, 1, 1, 0.5) | CONFIRMED | `transit_cpp.cpp:597`; `tack_reach.py:402`. |
| "The path is smoothed, re-checked against the collision model and timed with TOTG" | CONFIRMED | `smooth_transits`, TOTG service, trapezoid fallback. |
| Collision model "the table is the touched plane, tilted" | CONFIRMED | since 2026-10-14. |
| "fixtures are boxes entered by pen-tip points" | CONFIRMED as a mechanism, **none entered** | `marking.json` `fixtures: []`; unmodelled fixtures were hit on the curved bench (`notes/curved_seams_plan.md:449, 518`). |
| "Clearance on tacks vs. transits" (TODO) | data | 3 mm tacks, 20 mm transits (90 % of an end pose's own room). |
| §3.9.4 "fast to the approach point (35 mm before the tack), slower to the tack, slowest in the last millimetres" | CONFIRMED | 50 → 20 → 2 mm/s, last 6 mm; overshoot 5 mm. |
| Eq. (touch) F_touch = 1.5 N, bias measured at rest just before | CONFIRMED | `pen_tool.json:32`; bias over 0.5 s at the approach point. |
| contact depth sign (+ before, − past) | CONFIRMED | `tack_marking_node.py:38-43`. |
| "The overshoot ends at the table" | CONFIRMED | 2026-10-16 (`README.md:532`). |
| "dot, or a stroke along the seam with the depth corrected from the force between chunks" | CONFIRMED | default `dot`; chunk 10 mm, gain 2e-5 m/N. |
| TODO "the 8 N abort" | CONFIRMED | `motion.py:78`. |
| ch5 "Manipulator … F/T sensing" | CONFIRMED | the UR5e's own flange F/T via `/force_torque_sensor_broadcaster/wrench`; the payload setting is **not documented** (only a hint "check the payload (mass, CoG) on the pendant", `motion.py:350`). |

### E.4 Launch, configuration, robot description

#### `launch/pointcloud.launch.py` — the real-robot perception launch
Docstring (`:1-55`): TF `base_link → … → tool0` from `robot_state_publisher` on `/joint_states` (URDF
`urdf/ur5e.urdf.xacro`), static `tool0 → camera_color_optical_frame` from `T_tcp_to_cam.npy`, the RealSense node,
`depth_image_proc::PointCloudXyzrgbNode` (in a `component_container`, remapped to `/camera/depth/color/points`,
`queue_size` 10), all with `use_sim_time` false. Arguments and defaults (`:396-487`): `launch_robot_state` true,
`launch_camera` true, `tracking_source` `fp`, `fp_track_url` `ws://127.0.0.1:5001`, `weld_pose_tol_mm` 10.0,
`launch_totg` true, `launch_rviz` false, `rviz_config` (installed `launch/point_cloud_config.rviz`),
`camera_serial_no` '', `camera_width` 1280, `camera_height` 720, `camera_fps` 30, `extrinsic_parent` `tool0`,
`extrinsic_path` '' (→ `notebooks/T_tcp_to_cam.npy` in the source tree), `launch_foundationpose` **false**,
`foundationpose_server_url` `http://127.0.0.1:5000/predict_pose`, `foundationpose_timeout_sec` 300.0,
`pose_results_dir` '' (one value for bridge and ICP node), `detection_min_score` 0.0, `bbox_model_path` ''
(→ `models/test_objv2_base.ply`), `bbox_model_units` mm, `launch_icp` true, `icp_model_path` '', `icp_scene_from`
`pointcloud`, `icp_anderson_depth` 5, `icp_use_open3d` true, `icp_crop_margin_m` 0.03, `icp_tracking_rate_hz` 15.0,
`icp_auto_track` true, `icp_ground_removal` true, `icp_ground_plane_file` '' (→ `notebooks/table_plane.json` if it
exists, `:84-93`), `icp_ground_plane_offset_m` 0.002, `icp_ground_z_m` −0.10. Note the stale comment at `:228-230`
("Triggering it blocks until the operator clicks the object in the window the server opens on the GPU host") —
with the gesture flow the click comes from the laptop. **Thesis:** §3.10.1 architecture figure, ch5 §5.5.

#### `launch/point_cloud_config.rviz`
Fixed frame `base_link`; displays: Grid, RobotModel (`/robot_description`), Images (`/camera/color/image_raw`,
`/perception/frozen_rgb`), PointClouds (`/camera/depth/color/points`, `/perception/icp/scene_cloud`, `/model_cloud`,
`/static_env`, `/welding_points`, `/welding_tacks`, `/multiview_cloud`), Markers / MarkerArrays
(`/perception/detection_markers`, `/perception/fp/marker`, `/perception/icp/crop_box`, `/welding_tack_labels`,
`/multiview_views`, `/tack_marking/tip_path`, `/tack_reach/markers`, `/tool_model/markers`). Source for the RViz
figures (fig:views, fig:mc_collision, fig:tool).

#### `urdf/ur5e.urdf.xacro`, `urdf/ur5e.urdf`
The xacro includes `ur_description`'s `ur_macro.xacro` (resolved from `/opt/ros/jazzy`, `ur_description` 3.5.1),
root link `world`, `kinematics_parameters_file` = `config/ur5e_calibration.yaml` (factory calibration, 2026-09-29),
joint limits / physical / visual parameters from `ur_description` (`:15-37`). **No tool, pen or camera link is added**:
the camera enters TF only through the static extrinsic. Its header still says "Clean UR5e xacro wrapper for
sand_drawer project" (`:3-10`). `urdf/ur5e.urdf` (2026-05-20) is an old xacro expansion generated from a workspace
build of `ur_description` (`ros2_ws/install/ur_description/…`, `:3`) — not used by the launches.

#### `config/`
* `ur5e_calibration.yaml` — the robot's factory kinematic calibration (DH offsets per joint, e.g. shoulder z
  0.162579 m, forearm x −0.425166 m, wrist_1 x −0.392168 m): used by the URDF, the driver and every tool (§C.1).
* `default_kinematics.yaml` — the nominal UR5e chain (kept as the "nominal" model in `kinematics.py`).
* `pen_tool.json` (§B.1), `marking.json` (§E.3).
* Legacy sand-drawer / MoveIt-style files not used by the welding-cell launches: `kinematics.yaml` (KDL plugin for
  `ur_manipulator`), `ur5e.srdf` ("Auto-generated for sand_drawer project"), `joint_limits.yaml`,
  `physical_parameters.yaml`, `visual_parameters.yaml`, `initial_positions.yaml` ("UR5e home position for sand drawer
  workspace … Set these same angles in Isaac Sim"), `ros2_controllers.yaml` (`ur_manipulator_controller`,
  `forward_velocity_controller`, `joint_state_broadcaster`, 100 Hz), `controllers.yaml` (a generic 7-joint
  `arm_1…arm_7_joint` template — not for the UR5e).
* **Fact check** ch5 "the UR driver (scaled joint trajectory controller, velocity controller)": CONFIRMED — the cell
  uses the driver's `scaled_joint_trajectory_controller` (marking, multi-view, touch-off) and
  `forward_velocity_controller` (aim; "loaded inactive by ur_control.launch.py", `camera_aim_node.py:10-11`); the
  driver is `ur_robot_driver` 3.8.0 (`ur_controllers` 3.8.0), launched with `kinematics_params_file:=…/ur5e_calibration.yaml`
  (workspace `README.md:104-119`), External Control URCap to the laptop at 192.168.8.100:50002, robot at 192.168.8.4.

### E.5 Legacy pipelines (origin of the package name; not part of the welding cell)

The package began as a "sand drawer" / admittance surface-drawing project (`package.xml` description: "UR5e
task-space plane extraction and trajectory preparation pipeline for Isaac Sim based admittance control workflows";
README pipeline 1, `README.md:7-9`). Kept, still installed, not used by the cell:
* `scripts/drawing_action_server.py` (1697 l.) — `ExecuteDrawing` action server: RETRACT_UP → [HOMING] → RRT →
  DESCENT → DRAWING → ASCENT with an orthogonal drawing tool (`orthogonal_tool_length_m`), TOTG timing, sim or real
  (`:1-20, 210-281`). Its Catmull-Rom corner smoothing lives on in `kinematics.bezier_smooth_path`, which the marking
  node's `smooth_transits` reuses (`marking.py:81`).
* `scripts/drawing_dispatcher.py` — loads the plane JSON, generates primitives (random/line/triangle/square/circle) on
  it, checks reachability, sends `ExecuteDrawing` goals (`:1-12`).
* `scripts/cartesian_admittance_controller.py` — 1-DoF admittance in z on a nominal Cartesian path, TwistStamped to
  MoveIt Servo, wrench feedback (`:1-7`) — the "admittance control" of the package name.
* `scripts/move_to_object_node.py` — raw-pose test: pen tip to a standoff in front of the best SAM-6D detection along
  the camera ray; DLS IK; `~/go` (`:1-20, 125-137`).
* `action/ExecuteDrawing.action`, `launch/admittance_control.launch.py` (arguments `use_sim_time` true, `plane_json`,
  `trajectory_key` random, `continuous` false, `approach_height` 0.10, `surface_z_offset` −0.01,
  `orthogonal_tool_length_m` 0.13, `real_robot` false, `max_joint_speed_deg` 90, `max_joint_accel_deg` 40),
  `generated_planes/sand_drawer_plane.json` (freedrive-captured plane points in the UR `base` frame),
  `src/jacobian_calculator_node.cpp` (`sand_drawer_jacobian_node`).
* `helper/gui/drawing_gui.py` — PyQt5 + Matplotlib 3-D trajectory visualizer of `/visualizer/drawing_path` and
  `/visualizer/safe_probe_targets` (refers to `ros2 launch sand_drawer …`, `:1-20`).
* `helper/legacy_sand_drawer/segment_area.py` (one-shot **SAM3** text-prompt segmentation of a RealSense frame,
  publishes `~/mask`, `~/overlay`, `~/depth_m`, `~/metadata`), `depth_to_safe_probe.py` (mask → contour-safe 3-D probe
  targets in the robot base, published for the GUI), `safe_probe_preview.launch.py`; `artifacts/segment_area/20260520_140813/`
  holds one such run; `notebooks/idea.md` (2026-05-14, Turkish) is the plan behind it.
* `meshes/ur5e/{visual,collision}` — UR5e meshes copied for the old description; `plots/` is empty.
Thesis: at most one sentence in ch1/ch2 (how the platform began) — not in ch3/ch5.

### E.6 Notebooks and data artefacts not covered above

| file | content | thesis use |
|---|---|---|
| `notebooks/camera_calibration_test.ipynb` | intrinsics read-out (saved `realsense_intrinsics.npy`), an early ChArUco eye-in-hand loop (`DICT_6X6_250`, TCP = 0) and a 4-solver comparison (§C.4) | history of §3.3.2; the file in use came from the scripts |
| `notebooks/force_control_learning.ipynb` (2026-05-20, 16 cells, RTDE) | force plots, a single-point 1.5 N touch test, a 5-point plane least-squares fit, drawing a circle on the plane, RTDE force mode for plane following | origin of the touch-off idea; not in the draft |
| `notebooks/Sam_to_Surface_Plane.ipynb` (+ `.html`, 2026-06/07) | SAM3 segmentation of a RealSense frame → surface plane, RTDE (sand drawer) | legacy |
| `notebooks/sam_testing2.ipynb` (2026-05-13) | SAM3 text prompts on a "ROMER Lab" dataset, depth extraction → mesh (Open3D, Plotly) | early exploration |
| `notebooks/gesture_control_sandbox.ipynb`, `media_pipe_sandbox.ipynb` (2026-07-27) | prototypes of the gesture node | §3.4.1 background |
| `notebooks/.cache/sam3/bpe_simple_vocab_16e6.txt.gz` | SAM3 tokenizer cache | — |
| `notebooks/handeye_debug_latest.png` (2026-05-12, 1280 × 720) | last failed board detection of an old capture | — |
| `notebooks/T_base_to_cam.npy` (2026-07-02) | eye-to-hand result from the fixed-camera days (`README.md:127`) | — |
| `scripts/foundationpose_results/assembly.json` | `static_frame` + `objects[]`: `model`, `n_points`, `pose_static`, `pose_stats` (n, n_used, std_mm, std_deg, range_mm, max_deg, window, source, icp_refine), `T_static_camera`, `pose_before_refine`, `refine` | ch6 data |
| `…/welding_seams.json` | `method`, `registry`, `access`, `frame`, `units`, `objects`, `seams[]` (approach, per-point normals, cavity width, clear fraction, closed, dihedral, face pair, `fitup_mm`, length, polyline, pose_tol, reject reason, role, seam class, separation, weldable) | ch6 data |
| `…/welding_tacks.json` | `rule_version` `tackrule-0.1`, `params` (tack length 4t, 10-50 mm, margin 2t, k 10-33, d_abs 400 mm), `tacks[]` (point, segment, arclength, tack_no, order, seam flags) | ch6 data |
| `…/multiview/<ts>/` | 23 captures 2026-10-01 → 10-16: `capture.json`, `view_0..3.npz`, `assembly.json` | ch6 data, replayable with `multiview_refine_offline.py` |
| `…/previous_assemblies/<ts>/` | 42 archived assemblies 2026-08-12 → 10-16 (assembly, static_env, welding_*) | ch6 data |
| `…/detection_pem.json`, `detection_ism.npz`, `mask.png`, `vis_pose.png`, `object_name.txt`, `static_env.npy/.ply`, `welding_points.npy/.ply` | the last registration's artefacts (e.g. `test_objv2_base`, score 69.0) and the last SEPC | — |
| `test/data/bench_models/` | frozen CADs + registry for the tests (§B.2) | — |

### E.7 Infrastructure outside the package

* **`/workspaces/welding_cell_ws/.devcontainer/`** — `devcontainer.json`: image **`nvcr.io/nvidia/isaac-sim:5.1.0`**,
  `--network=host`, `--privileged`, `--gpus=all`, `/dev`, USB and udev mounts (for the RealSense), X11, Isaac caches,
  `ROS_DOMAIN_ID` 0, `UR5E_ROBOT_IP` 192.168.8.4, root user, post-create `setup.sh`. `setup.sh` installs ROS 2 Jazzy
  (`ros-jazzy-desktop`, `ros-base`, `cv-bridge`, `image-geometry`, `moveit`, `vision-msgs`, `depth-image-proc`, …),
  `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, clones `IsaacSim-ros_workspaces`, and defines aliases (`isaac-sim`,
  `isaac-check`, `cb`/`cbs` = `colcon build --symlink-install`, `sw`, `sw-isaac`, `ur5e-ping`) (`setup.sh:12-167`).
  This container runs on the **laptop** (`nvidia-smi` here: "NVIDIA GeForce RTX 4060 Laptop GPU, 8188 MiB", driver
  580.178.04; 16 CPU threads, 30 GB RAM; Ubuntu 24.04.2 LTS; Python 3.12.3).
* **Software versions found in this container** (for ch5 §5.2 / Appendix D): ROS 2 Jazzy; `ur_robot_driver` 3.8.0,
  `ur_controllers` 3.8.0, `ur_description` 3.5.1, `moveit_core` 2.12.4, `ompl` 1.7.0 (all `/opt/ros/jazzy`); the
  workspace `.venv`: OpenCV 5.0.0, NumPy 2.3.5, SciPy 1.18.0, Open3D 0.19.0, trimesh 4.12.2, MediaPipe 0.10.35,
  websocket-client 1.9.0, torch 2.12.0+cu130, pyrealsense2, manifold3d; system python3: OpenCV 4.6.0, NumPy 1.26.4,
  SciPy 1.11.4. (Which interpreter runs the nodes depends on the shell; `#!/usr/bin/env python3` resolves to the
  `.venv` here.) The FoundationPose container's versions (CUDA, torch, SAM2 commit) are **not recorded** in this
  workspace.
* **`/workspaces/welding_cell_ws/IsaacSim-ros_workspaces/`** — NVIDIA's upstream repository
  (`https://github.com/isaac-sim/IsaacSim-ros_workspaces.git`, branch `main`, commit `3f060b8`, 2026-06-10 "Bumped Isaac
  Sim and torch versions (#23)"): `humble_ws`, `jazzy_ws` (Isaac Sim ROS 2 bridge workspaces) and `dockerfiles`
  (`ubuntu_22_humble…`, `ubuntu_22_jazzy…`, `ubuntu_24_jazzy_python_312_minimal.dockerfile`). Cloned by `setup.sh`; untracked
  in the thesis repository (`git status`: `?? IsaacSim-ros_workspaces/`); **nothing in `admittance_control` references
  it**. It serves only the Isaac Sim digital twin. Its Dockerfiles are **not** the FoundationPose server's.
* **Digital twin.** `world/welding_world.usda` (Isaac Sim 5.1 assets: Simple_Room, the UR5e USD, the pen holder
  `./penholder_assembly.usda`, the table `world_parts/robot_tablev1.usd`, a RealSense D435i (`rsd435i`), OmniGraph ROS 2
  bridge publishing `isaac_joint_states` / subscribing `isaac_joint_commands`, camera helpers), `world/table_v1.usda`,
  `world/Realsense435i_custom.usda`; driven by `digital_twin_pointcloud.launch.py` and `realsense_sim_camera_node.py`.
  The README markets "the same nodes and topics work in sim and on hardware" (`README.md:33-34`). **The draft never
  mentions Isaac Sim**; decide whether it is used (e.g. for development/tests) — if not, one sentence in §5.2 at most.
* **`ros2_ws/src/Universal_Robots_ROS2_Description/`** — a vendored copy (no own `.git`; tracked by the thesis
  repository), `ur_description` version 2.6.0 (`package.xml`). The current install space has no `ur_description`, so
  `$(find ur_description)` resolves to the apt package 3.5.1; the old `urdf/ur5e.urdf` was generated from a build of
  this copy. Not needed by the cell; not in the draft.
* **`/workspaces/welding_cell_ws/scripts/`** — `prepare_host.sh` (host preparation for the dev container),
  `test_move.py`, `test_move2.py` (2026-03, early motion tests). **`world_parts/`** — `robot_tablev1.step/.usd/.usda`
  (FreeCAD 2026-02-09, the robot table), `ZenTool_4Sided_Assembly_V4.step/.usda` (sand-drawing rake tool).
  **`.tmp_camera_probe/`** (2026-05-12) — ChArUco legacy-pattern test images and a camera probe grid.
* **FoundationPose Docker**: see §A.0 — the Dockerfile (`docker/Dockerfile.blackwell`) and run scripts live in the
  FoundationPose repository on the GPU hosts, not here.
