# D - Papers: index, technical digests, bibliography check and gaps

*Review of `thesis/paper_pool/`, `thesis/supply_papers/`, `weld_generator/papers/` and
`thesis/manuscript/thesis.bib`, written 2026-10-20 for the METU Robotics MSc thesis (robotic
tack-placement cell + WeldSet). Nothing in the repository was modified.*

## How this was produced

- **193 PDF files** were found: 166 in `paper_pool/` (21 sub-folders), 7 in `supply_papers/`, 20 in
  `weld_generator/papers/`. Byte-level duplicates were found by MD5, same-paper copies by comparing
  the first pages. That leaves **165 distinct papers**:
  - 160 in `paper_pool/`. Its 166 files contain 2 byte-identical copies and 4 pairs that are two
    files of the same paper.
  - 4 more in `supply_papers/`. The other 3 supply papers (FoundationPose, Drost, Birdal) are
    already in `paper_pool/`.
  - 1 that exists only in `weld_generator/papers/` (RRT\*-Connect). All other 19 files there are
    byte-identical copies of pool papers.
  `paper_pool/` also holds 2 Markdown reports and 5 images.
- For every PDF, pages 1-2 were extracted (`pdftotext -l 2`), plus `pdfinfo` metadata. Where
  something was missing, later pages were also read. Three volume-sized PDFs were located inside
  their volumes: the ECCV volume containing SceneCAD, the Springer volume containing tag 082, and
  an MSc thesis.
- The **7 supply papers were read in full**. The RRT-Connect PDF has a broken text layer (Type-3
  fonts), so all 7 pages were read from rendered page images. Equations and tables were checked on
  rendered pages wherever the text layer garbled them.
- The bibliography was compared field by field with the PDFs. **Fields that are not printed on
  the PDFs** (pages of published versions of arXiv copies, some DOIs) were checked against public
  records: the Kavraki-lab page, the CVF/mlanthology, ICLR proceedings, the Bedfordshire research
  portal, PolyU repository, Cardiff ORCA, roboticsproceedings.org. Each such field is marked
  *(online)* below. All extracted text is in
  `/tmp/claude-0/-workspaces-welding-cell-ws/ab70b953-a34b-4bce-b04c-436c69dd0e01/scratchpad/thesis_review/`
  (`first2/NNN.txt`, `full/`, `index_parts/*.jsonl`).

## The most important findings

1. **`luo2014aps` cites the wrong paper.** OMPL's AnytimePathShortening is from
   **Luna, Şucan, Moll & Kavraki, "Anytime Solution Optimization for Sampling-Based Motion
   Planning", ICRA 2013, pp. 5053-5059, doi:10.1109/ICRA.2013.6631301**. That is the supply PDF.
   Luo & Hauser (IROS 2014) is a different paper, an empirical study of optimal planners. Replace
   the entry (`luna2013aps`) and change both `\cite{luo2014aps}`: ch2 l.68 and ch3 l.545.
2. **The MediaPipe citation does not support the sentence it is used for.** In ch3 l.158,
   "MediaPipe's gesture recogniser \cite{lugaresi2019mediapipe} classifies seven static gestures"
   cites a paper that only describes the framework (graphs, calculators, scheduling, GPU). It has
   no hand, landmark or gesture model. Add **MediaPipe Hands** (Zhang et al., arXiv:2006.10214,
   CVPR 2020 Workshop on CV for AR/VR) and cite the **Gesture Recognizer** documentation/model card
   (a `@misc`) for the seven canned gestures.
3. **"PPF revisited" is Birdal & Ilic (3DV 2015), not Hinterstoisser et al. "Going Further with
   Point Pair Features" (ECCV 2016).** Hinterstoisser's paper is not in any folder. The thesis
   currently cites only Drost and Birdal, which is consistent. If "Going further" is meant, it must
   be fetched (Part 4).
4. **Bib entries with wrong or missing fields**, each checked against its PDF (full list in
   Part 3):
   - `liu2023rcim` is a placeholder. The paper is tag 105: Liu, Tang, Tian, Yang, RCIM 83, 102549.
   - `zhang_lobb`: year 2025, vol. 95, art. 102987.
   - `fang2024modelreg` has only two authors; remove "and others".
   - `li2026quadric` has a truncated title, initials only, and is missing its end page.
   - `lin2026coating` pages are 639-650.
   - `wei_regiongrow` is IROS 2024, pp. 7138-7144 *(online)*.
   - `wen2024foundationpose` is missing pp. 17868-17879 *(online)*.
   - `ravi2024sam2` has 18 authors and was published at ICLR 2025 *(online)*.
   - `kuffner2000rrtconnect` is correct but is missing vol. 2 and the DOI. Note that the supply
     PDF is the pre-submission **draft**.
5. **Volume-sized PDFs and non-citable files in the pool:**
   - `SceneCAD_...pdf` is the whole ECCV 2020 volume (826 pp., 183 MB); SceneCAD itself is
     pp. 596-612.
   - `superficial/Point Cloud Based Three-Dimensional Reconstruction...pdf` is a whole 180-page
     Springer volume; the named paper is pp. 61-77.
   - `katman1/hernandez_villa_alejandro.pdf` is an MSc thesis (University of Manitoba, 2024).
   - The two "deep search result" PDFs and the `.md` beside them are **AI-generated reports**. One
     of them misreports Tomków et al. 2020.
   - `Welding_Robots_Approach_and_Trajectory_Literature_Report.md` is also AI-generated ("Antigravity
     AI"). Its reference list contains **wrong citations**: Geng et al. is given as 2022/vol. 78
     instead of 2023/79; Peng et al. as IROS instead of UR 2020; Jia et al. as Applied Sciences
     2024 instead of Actuators 2025; Tomków's co-authors are wrong; CAPT as IROS 2023 instead of
     arXiv/RSS 2024. Never copy references from these files.
6. **The pool has no PDFs for several things the thesis already cites or will need:** hand-eye
   calibration, point-to-plane ICP, DLS IK, visual servoing, One-Euro filter, Viterbi, TOTG,
   RealSense depth error, ISO standards, swing-twist decomposition, SAM (v1), Hiera, MediaPipe
   Hands, shortcutting/hybridization. Nine of these already have bib entries written from memory;
   Part 4 lists what to fetch.
7. **Distinctive, relevant pool papers that are not in thesis.bib yet**, recommended for Chapter 2
   (BibTeX in Part 3.3):
   - Zhang et al. 2025, T-ASE: the original **LOBB** descriptor, used by the `lobb` baseline.
   - Kuss et al. 2017: CAD-to-scan ICP to compute the **fit-up gap** for welding.
   - Geng et al. 2024: **analytic intersection-curve models** fitted to pipes.
   - Zhou et al. 2021, RA-L: **gesture-guided** welding scanning.
   - Lowth et al. 2015 and Tomków et al. 2020: **tack welds and gaps vs. distortion**.
   - Schleth et al. 2018: review of **workpiece localisation for welding**.
   - SuperPose and DynamicPose: **SAM2 + FoundationPose** pipelines very close to the cell.
   - Denayer et al. 2024: registration methods compared on **RealSense D435i** scans of printed
     parts.

## Contents

- **Part 1 - Index of the paper pool, grouped by thesis topic** (sections A-N; non-paper files in O; the weld_generator cross-reference in P)
- **Part 2 - Technical digests of the 7 supply papers**: 2.1 FoundationPose, 2.2 SAM 2, 2.3 MediaPipe (+ MediaPipe Hands pointer), 2.4 Drost PPF, 2.5 Birdal & Ilic PPF, 2.6 RRT-Connect, 2.7 Anytime Path Shortening
- **Part 3 - Bibliography check**: 3.1 verdict per entry, 3.2 corrected BibTeX, 3.3 entries without PDFs, 3.4 pool papers worth adding
- **Part 4 - Topics the pool does not cover yet**
- **Sources consulted online**

## Part 1 - Index of the paper pool

Conventions. **Tag** = the extraction number used during this review (001-166 = paper_pool, 167-173 = supply_papers, 182 = weld_generator/papers only). One row per paper; copies of the same paper are listed in the row ("Copies"). **Folder** is relative to `thesis/paper_pool/` unless it says `supply_papers/` or `weld_generator/`. **Supports** names the thesis section (Ch2 background topic or a methodology section). **bib** = key in `manuscript/thesis.bib` (- = not in the bibliography). Venue/year are as printed on the PDF; "(inferred)" marks anything not printed.


### A. Robotic welding systems, sensing and seam tracking (Ch2 "Robotic welding and sensing"; Ch1 motivation)

| Tag | Paper | Gist | Folder | Supports | bib |
|---|---|---|---|---|---|
| 012 | **Application of sensing technology in intelligent robotic arc welding: A review**<br>Xu et al. (4), 2022. *Journal of Manufacturing Processes 79, 854-880 (2022), doi:10.1016/j.jmapro.2022.05.029* | Review of sensing for intelligent robotic arc welding (vision, arc, acoustic, spectral, ultrasonic sensing and multisensor fusion), organised by pre-, in- and post-process stages: weld path recognition, seam tracking, weld pool monitoring, quality diagnosis and bead inspection. | (pool root) | Ch2 Weld seam detection from 3D data (sensing for pre-process weld path recognition); Ch2 Summary and research gap; Ch7 Future work (seam tracking). | - |
| 013 | **Automatic weld type classification, tacked spot recognition and weld ROI determination for robotic welding based on modified YOLOv5**<br>Chen et al. (5), 2023. *Robotics and Computer-Integrated Manufacturing 81, 102490 (2023), doi:10.1016/j.rcim.2022.102490* | Modified YOLOv5 on structured-light stripe images from a Panasonic GMAW robot classifies weld type (fillet/lap vs groove), recognises tacked spots and finds the weld ROI in one inference, using a centre-bias term in the CIoU loss, a weighted classification loss and self-template border padding. Reports 100% precision and recall, 0.91 mIoU, 2.41 px ROI centre bias and 18 ms inference. | (pool root) | Ch2 Welded joints, tack welds and standards (parts are tacked before full-length welding; tack spots disturb structured-light seam tracking); Ch7 Future work (seam tracking over tacks). | - |
| 054 | **Automated Weld Path Generation in Cluttered Environments Using Segmentation and Iterative Closest Point Workpiece Localization**<br>Hill et al. (3), 2025. *Journal of Mechanisms and Robotics 17(4), 044511 (April 2025), Technical Brief, doi:10.1115/1.4066983* | Collaborative-robot welding with a low-cost RGB-D camera on the end-effector: about four views are fused using the robot kinematics, Euclidean and RGB region-growing clusters are paired, the workpiece cluster is picked out of clutter by a volume/singular-value/colour cost, and ICP to the cluster recorded at teaching time transfers the taught weld path to the part's current pose. Targets near-final registration to be completed by near-position strategies. | closely related/katman1/ | Ch2 Weld seam detection from 3D data (registration-based weld-path transfer); Ch3 Perception and registration (eye-in-hand RGB-D, clustering, ICP localisation); Ch3 Multi-view close-range refinement | - |
| 059 | **Advances in weld seam tracking techniques for robotic welding: A review**<br>Rout et al. (3), 2019. *Robotics and Computer Integrated Manufacturing 56, 12-37 (2019), doi:10.1016/j.rcim.2018.08.003* | Review of sensors and techniques for weld seam tracking (arc, laser and vision, infrared, ultrasonic, electromagnetic and others) and of how each extracts seam features: start and end point detection, edge detection, joint width measurement and seam position in the robot frame. Compares the sensors' advantages, drawbacks and industrial use. | to the tack weld sampling/ | Ch2 Weld seam detection from 3D data (sensor background); Ch7 Future work (seam tracking) | - |
| 130 | **YOLO-LaserGalvo: A Vision-Laser-Ranging System for High-Precision Welding Torch Localization**<br>Li et al. (3), 2025. *Sensors 25(20), 6279 (2025), doi:10.3390/s25206279*<br>Copies: weld_generator/papers/YOLO–LaserGalvo.pdf (byte-identical) | Monocular camera + improved YOLOv11 detects the torch tip; a galvanometer steers an IR laser rangefinder to it and the range is fused with the image coordinates for real-time 3D tip position; beats colour-marker and ArUco baselines. | For Welding Dataset/non fem/ | Ch2 Robotic welding and sensing (torch/TCP localisation); probably not needed | - |
| 143 | **Adaptive control method for ship curved hybrid laser-arc welding through lightweight encoder-decoder architecture**<br>Zhang et al. (3), 2025. *Engineering Applications of Artificial Intelligence 145, 110193 (2025), doi:10.1016/j.engappai.2025.110193* | Lightweight encoder-decoder regresses laser-stripe feature points (3.41 px) and centre lines (1.47 px) during hybrid laser-arc welding of curved ship blocks, used for seam tracking, pose adaptation and gap-based wire-feed adjustment. | For Welding Dataset/non fem/reading/ | Ch2 Robotic welding and sensing (seam tracking, gap-adaptive parameters); Ch7 Future work | - |
| 144 | **Application and trends of point cloud in intelligent welding: State of the art review**<br>Wang et al. (5), 2025. *Journal of Manufacturing Systems 79:48-72 (2025), doi:10.1016/j.jmsy.2025.01.001* | Review of point clouds across pre-welding (construction, seam detection, path/posture planning, programming, positioning), in-welding (pool monitoring, tracking) and post-welding (identification, defect and quality inspection) stages, comparing sensors and algorithms. | For Welding Dataset/non fem/reading/ | Ch2 Weld seam detection from 3D data (survey anchor); Ch1 Introduction (motivation) | - |
| 148 | **Hybrid offline programming method for robotic welding systems**<br>Zheng et al. (7), 2022. *Robotics and Computer-Integrated Manufacturing 73, 102238 (2022), doi:10.1016/j.rcim.2021.102238* | Combines CAD-based, vision-based and vision-CAD interactive offline programming for shipbuilding workpieces whose diversity defeats either approach alone. | For Welding Dataset/non fem/reading/ | Ch2 Robotic welding and sensing (CAD vs vision programming); Ch2 Summary and research gap | - |
| 149 | **Key technologies and latest research progress of automated robotic welding: a review**<br>Li et al. (6), 2025. *Journal of Intelligent Manufacturing (online; received 20 Jan 2025, accepted 24 Sep 2025), doi:10.1007/s10845-025-02700-7* | Review of automated robotic welding systems, vision sensing, weld path recognition, weld shape/property control and post-weld inspection, with digital twins and multi-sensor integration. | For Welding Dataset/non fem/reading/ | Ch1 Introduction; Ch2 Robotic welding and sensing (survey) | - |
| 151 | **Progress, challenges and trends on vision sensing technologies in automatic/intelligent robotic welding: State-of-the-art review**<br>Guo et al. (8), 2024. *Robotics and Computer-Integrated Manufacturing 89, 102767 (2024), doi:10.1016/j.rcim.2024.102767* | Review of active/passive vision for initial weld position location, seam identification and tracking, point-cloud and composite-vision path planning, weld pool monitoring (infrared/visible) and defect detection. | For Welding Dataset/non fem/reading/ | Ch1 Introduction; Ch2 Robotic welding and sensing (survey) | - |
| 152 | **Vision-Based Autonomous Robotic Arc Welding: State-of-the-Art Review and Perspectives**<br>Ma et al. (8), 2025. *IEEE Transactions on Automation Science and Engineering 22:21651-21673 (2025), doi:10.1109/TASE.2025.3614992* | Review of vision sensors and the autonomy chain of robotic arc welding: weld type identification, initial point guidance, path generation, parameter planning, feature point extraction, seam tracking, posture adjustment and quality control. | For Welding Dataset/non fem/reading/ | Ch1 Introduction; Ch2 Robotic welding and sensing (survey) | - |
| 158 | **An Efficient and Robust Complex Weld Seam Feature Point Extraction Method for Seam Tracking and Posture Adjustment**<br>Ma et al. (7), 2023. *IEEE Transactions on Industrial Informatics 19(11):10704-10715 (Nov 2023), doi:10.1109/TII.2023.3241595* | Shuffle-YOLO extracts laser-stripe feature points of butt, lap and irregular joints under arc light and spatter; nearest-neighbour + cubic B-spline models give position and posture of complex curved seams for tracking and torch posture adjustment. | For Welding Dataset/non fem/reading/new pickeds/ | Ch2 Robotic welding and sensing (seam tracking); Ch7 Future work | - |
| 161 | **Visual Sensing and Depth Perception for Welding Robots and Their Industrial Applications**<br>Wang et al. (3), 2023. *Sensors 23(24), 9700 (2023), doi:10.3390/s23249700* | Review of active and passive depth perception for welding robots (monocular, binocular, multi-view, deep-learning depth), industrial applications and open problems; 2662 articles screened, 152 cited. | reading/ | Ch2 Robotic welding and sensing; Ch2 Camera calibration (depth sensing background) | - |

Notes:

- **054**: ASME Technical Brief, published online 18 Nov 2024 (issue April 2025). Registers to a taught scan, not to CAD. File name is truncated.
- **059**: PDF metadata gives '(2018)' (available online 4 Sep 2018); the printed issue year is 2019.
- **130**: Copy of weld_generator/papers/YOLO-LaserGalvo.pdf (same bytes).
- **144**: Recommended as the main survey citation for point-cloud welding.
- **149**: 57-page review; volume/pages not printed.
- **151**: '_compressed' file.
- **152**: End page computed from the first page (21651) and the 23 PDF pages; verify.
- **158**: End page computed from first page 10704 and 12 PDF pages; verify.
- **161**: File name starts with a space.

### B1. Weld seam extraction - the seven methods benchmarked on WeldSet (Ch2 seam detection; Ch4 evaluation protocol)

| Tag | Paper | Gist | Folder | Supports | bib |
|---|---|---|---|---|---|
| 118 | **3D vision-based intersecting pipe welding path planning**<br>Wang et al. (4), 2026. *Welding in the World (online first; received 10 Mar 2026, accepted 26 May 2026), doi:10.1007/s40194-026-02509-9; no volume/pages yet*<br>Copies: weld_generator/papers/implemented_papers/lit-pcaslice.pdf (byte-identical) | YOLO11 coarse localisation + improved DeepLab V3+ segmentation of intersecting-pipe welds, pixel-to-point mapping, PCA centre-line extraction and interpolation, torch posture from MSAC-WTLSD plane fits; welding accuracy within 1 mm. | For Welding Dataset/non fem/ | Ch2 Weld seam detection from 3D data; Ch4 Evaluation protocol (benchmarked method 'pcaslice'); Ch4 Curved seams by construction | `wang2026pipes` |
| 119 | **A novel weld seam extraction method with semantic segmentation and point cloud feature for irregular structure workpieces**<br>Zhang et al. (5), 2025. *Robotics and Computer-Integrated Manufacturing 95, 102987 (2025), doi:10.1016/j.rcim.2025.102987*<br>Copies: weld_generator/papers/implemented_papers/lit-lobb.pdf (byte-identical) | K-Net semantic segmentation of RGB-D images from a DLP structured-light camera for coarse seam localisation, edge-point reconstruction and adaptive ROI, then LOBB feature extraction and least-squares polynomial seam fitting; max error < 1.2 mm, RMSE < 0.7 mm, < 5 s. | For Welding Dataset/non fem/ | Ch2 Weld seam detection from 3D data; Ch4 Evaluation protocol (benchmarked method 'lobb') | `zhang_lobb` |
| 121 | **Coarse-to-Fine Detection of Multiple Seams for Robotic Welding**<br>Wei et al. (6), 2024. *arXiv:2408.10710v1 [cs.CV] (20 Aug 2024) on the PDF; published at IEEE/RSJ IROS 2024, Abu Dhabi, pp. 7138-7144, doi:10.1109/IROS58592.2024.10802546 (online, not on the PDF)*<br>Copies: weld_generator/papers/implemented_papers/lit-regiongrow.pdf (byte-identical) | Detects multiple weld seams at once: an RGB image (pre-trained deep model) gives the ROIs, then region growing on the structured-light point cloud extracts fine seam edges within each ROI; linear and curved seams, physical experiments. | For Welding Dataset/non fem/ | Ch2 Weld seam detection from 3D data; Ch4 Evaluation protocol (benchmarked method 'regiongrow') | `wei_regiongrow` |
| 127 | **Weld seam extraction and path generation for robotic welding of steel structures based on 3D vision**<br>Yi et al. (6), 2026. *Automation in Construction 183, 106792 (2026), doi:10.1016/j.autcon.2026.106792*<br>Copies: 047: closely related/; weld_generator/papers/implemented_papers/lit-ransac.pdf (byte-identical) | Coded structured light; deep point-cloud segmentation of weld regions plus improved multi-scale registration to rebuild the complete weld-region cloud, then optimised multi-plane fitting with a seam geometry model for paths and torch poses; four steel workpieces. | For Welding Dataset/non fem/ | Ch2 Weld seam detection from 3D data; Ch4 Evaluation protocol (benchmarked method 'ransac'); tab:lit_seam | `yi2026seam` |
| 128 | **Weld seam object detection system based on the fusion of 2D images and 3D point clouds using interpretable neural networks**<br>Wang et al. (4), 2024. *Scientific Reports 14, 21137 (2024), doi:10.1038/s41598-024-71989-w*<br>Copies: weld_generator/papers/implemented_papers/lit-ppf.pdf (byte-identical) | Improved Faster R-CNN (ResNet50) with interpretable components localises seams in images under spatter/smoke, and an orthogonal-plane intersection-line algorithm extracts the seam from the 3D point cloud. | For Welding Dataset/non fem/ | Ch2 Weld seam detection from 3D data; Ch4 Evaluation protocol (benchmarked method 'ppf' / plane intersection) | `wang2024ppfseam` |
| 134 | **A novel model-based welding trajectory planning method for identical structural workpieces**<br>Fang and Tian (2), 2024. *Robotics and Computer-Integrated Manufacturing 89, 102772 (2024), doi:10.1016/j.rcim.2024.102772*<br>Copies: weld_generator/papers/implemented_papers/lit-modelreg.pdf (byte-identical) | A basic model (point set) carrying the seam/trajectory information is non-rigidly registered to the feature cloud of a similar workpiece to transfer the seams, and torch poses come from geometric projection of the local seam cloud. | For Welding Dataset/non fem/non DL/ | Ch2 Weld seam detection from 3D data (model/registration-based); Ch4 Evaluation protocol (benchmarked method 'modelreg'); Ch2 Summary and research gap | `fang2024modelreg` |
| 137 | **Automatic recognition on impeller shape and weld seam based on normal of point clouds and PCA**<br>Li et al. (3), 2026. *Journal of Mechanical Science and Technology 40(3):2039-2046 (2026), doi:10.1007/s12206-026-0243-8*<br>Copies: weld_generator/papers/implemented_papers/lit-quadric.pdf (byte-identical) | Gaussian + voxel filtering and region growing split the impeller cloud, PCA and normal-distribution analysis classify the (curved) blade surface shape, the seam position comes from least squares and the welding path order from a graph algorithm and bubble sort. | For Welding Dataset/non fem/non DL/ | Ch2 Weld seam detection from 3D data (curved surfaces); Ch4 Evaluation protocol (benchmarked method 'quadric') | `li2026quadric` |

Notes:

- **118**: Byte-identical to weld_generator/papers/implemented_papers/lit-pcaslice.pdf. Bib entry matches the PDF; add note 'online first' and update volume/pages when assigned.
- **119**: Byte-identical to weld_generator/papers/implemented_papers/lit-lobb.pdf. Bib mismatch: year 'VERIFY' -> 2025; add volume 95, article 102987, DOI. Key name zhang_lobb could become zhang2025lobb.
- **121**: Byte-identical to weld_generator/papers/implemented_papers/lit-regiongrow.pdf. Bib is @misc with 'VERIFY venue': the PDF shows only arXiv:2408.10710; the IROS 2024 version was found online (corrected entry in Part 3).
- **127**: Byte-identical to weld_generator/papers/implemented_papers/lit-ransac.pdf; also a different file of the same paper in 'closely related/' (tag 047). Bib matches the PDF; add DOI. [copy 047] Same paper and same final Elsevier version as tag 127 (For Welding Dataset/non fem/...steel.pdf): same publisher PDF (identical metadata, 19 pp., identical full text) but not byte-identical, because the copies carry different reader highlight annotations (38 in this copy, 1 in tag 127). The thesis.bib entry yi2026seam matches (vol. 183, art. 106792, 2026) but has no DOI.
- **128**: Byte-identical to weld_generator/papers/implemented_papers/lit-ppf.pdf. Bib matches the PDF (vol 14, art. 21137, 2024, DOI). The key 'ppfseam' is misleading: the method is plane-plane intersection, not point-pair features.
- **134**: Byte-identical to weld_generator/papers/implemented_papers/lit-modelreg.pdf. Bib mismatch: author list is exactly two authors (Weihua Fang, Xincheng Tian) - remove 'and others'; add DOI.
- **137**: Byte-identical to weld_generator/papers/implemented_papers/lit-quadric.pdf. Bib mismatches: title truncated and 'normals' -> full title '... based on normal of point clouds and PCA'; authors 'Wang, Y.' -> 'Wang, Yu', 'Wang, Z.' -> 'Wang, Zhengyong'; pages '2039' -> '2039--2046' (printed in the citation box). PDF metadata title is wrong ('Molybdenum and Cobalt Silicide Field Emitter Arrays').

### B2. Weld seam extraction - geometric / model-based methods on point clouds, RGB-D and laser profiles (Ch2 seam detection)

| Tag | Paper | Gist | Folder | Supports | bib |
|---|---|---|---|---|---|
| 004 | **Intelligent and Adaptive System for Welding Process Automation in T-Shaped Joints**<br>Aldalur et al. (5), 2023. *Metals 13, 1532 (2023), doi:10.3390/met13091532*<br>Copies: 018: Intelligent and Adaptive System for Welding Process.pdf (byte-identical) | A Queltech Q4-120 laser profilometer on a Fanuc Arc Mate 100iC robot scans thick double-bevel T-joints (60 mm mild-steel plates, 45 degree bevel, corners pre-welded to limit deformation); the profile point cloud is filtered and analysed to set the GMAW torch trajectory for the first layer automatically, with reachability checks. Presented as an initial implementation for T-joints under specific welding conditions. | (pool root) | Ch2 Weld seam detection from 3D data (laser-profile joint scanning for torch path); Ch2 Welded joints, tack welds and standards (thick T-joints pre-welded at the corners before multi-layer welding). | - |
| 008 | **A Point Cloud-Based Method for Automatic Groove Detection and Trajectory Generation of Robotic Arc Welding Tasks**<br>Peng et al. (4), 2020. *2020 17th International Conference on Ubiquitous Robots (UR), Kyoto, Japan, June 22-26, 2020, pp. 380-386 (IEEE); DOI not visible* | An Intel RealSense D415 mounted with the torch on a UR3 captures the workpiece cloud; after MLS smoothing, a per-point surface-variation descriptor (3D feature histogram) is thresholded to segment V-type grooves and a 6-DOF torch trajectory is generated and executed with ROS/PCL/MoveIt. Tested on four workpiece types with V-grooves for runtime, groove-detection accuracy and trajectory execution. | (pool root) | Ch2 Weld seam detection from 3D data (geometric groove extraction from RGB-D); Ch3 Overview (comparable hardware: wrist RGB-D camera on a UR arm with ROS/MoveIt). | - |
| 011 | **A novel feature-guided trajectory generation method based on point cloud for robotic grinding of freeform welds**<br>Feng et al. (7), 2021. *The International Journal of Advanced Manufacturing Technology 115, 1763-1781 (2021), doi:10.1007/s00170-021-07095-2* | A Keyence LJ-V7200 laser profiler on an ABB IRB6700 scans freeform weld beads; per-line parent curves are fitted with a moving-average filter, weld feature points are extracted in two recognition stages, and a B-spline is fitted and discretised into tool contact points and postures for force-controlled grinding. Recognition RMSE below 0.7 mm and 65.12% less computation time than a reverse-engineering approach. | (pool root) | Ch2 Weld seam detection from 3D data (laser-profile weld-bead feature extraction, B-spline path and tool postures); Ch3 Seams from the registered assembly (curved seams: B-spline discretisation and tool orientation). | - |
| 017 | **Intelligent Path Planning System for Welding Robots Integrating Line Laser Sensing and Weld Seam Geometric Constraints**<br>Yan et al. (6), 2025 (inferred). *SSRN preprint, not peer reviewed, https://ssrn.com/abstract=5273135* | Line-laser point clouds of weld grooves: hybrid plane segmentation-clustering finds weldable regions, PCA curvature and orthogonal slicing extract the groove's top/transition/bottom layers, and a contour-constrained zigzag strategy plans multi-layer multi-pass filling paths for variable gaps (0.5-3.0 mm). Total processing and planning within 17.34 s, 17 ms per path. | (pool root) | Ch2 Weld seam detection from 3D data (laser-line groove feature extraction, multi-pass path planning); Ch3 Seams from the registered assembly (fit-up gap: variable-gap grooves). | - |
| 049 | **Edge and Corner Detection for Unorganized 3D Point Clouds with Application to Robotic Welding**<br>Ahmed et al. (5), 2018. *2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Madrid, Spain, October 1-5, 2018, pp. 7350-7355 (DOI not printed)*<br>Copies: 145: For Welding Dataset/non fem/reading/ | Edge detector for unorganised point clouds that marks a point as an edge when the centroid of its k nearest neighbours shifts by more than a density-adaptive threshold, plus a corner detector that clusters the curvature vectors of edge points. Reports higher precision and recall on RGB-D semantic segmentation data and ShapeNet washer models, and extracts weld seams from a panel-workpiece point cloud in comparison with Harris 3D. | closely related/boundary detection/ | Ch2 Weld seam detection from 3D data (geometric edge-based seam extraction from point clouds); Ch4 Proposed seam extractors; Ch6 Results/discussion | - |
| 050 | **Multiple weld seam extraction from RGB-depth images for automatic robotic welding via point cloud registration**<br>Kim et al. (4), 2021. *Multimedia Tools and Applications 80, 9703-9719 (2021), doi:10.1007/s11042-020-10138-7* | Extracts line and curve weld seams in each view of an RGB-D sensor, filters the 3D seam points with edge cues from the RGB image, and merges the seams from several view directions by point cloud registration, so that multiple and occluded seams (including on cylinders and pipes) are recovered. Reported to outperform a state-of-the-art method in detection accuracy and to work on dynamic workpiece scenes. | closely related/boundary detection/ | Ch2 Weld seam detection from 3D data (RGB-D, 2D edge cues fused with 3D); Ch3 Multi-view close-range refinement (merging views by registration); Ch6 Results/discussion | - |
| 053 | **A feature-extraction localization algorithm research for teaching-free automated robotic welding based on 3D point cloud**<br>Li et al. (6), 2025. *The International Journal of Advanced Manufacturing Technology 138, 5397-5412 (2025), doi:10.1007/s00170-025-15797-0* | Teaching-free welding pipeline that scans the workpiece with a 3D vision sensor and either registers it to the standard CAD model (FPFH-RANSAC-ICP) or, without a model, localises seams by point cloud segmentation and feature extraction, then corrects the offline-programmed trajectory. Validated on T-pipe, triplanar fillet and V-groove butt joints. | closely related/katman1/ | Ch2 Weld seam detection from 3D data; Ch2 Point cloud registration (FPFH-RANSAC-ICP); Ch3 Seams from the registered assembly (registering the CAD model to correct offline paths) | - |
| 082 | **Point Cloud Based Three-Dimensional Reconstruction and Identification of Initial Welding Position**<br>Zhang et al. (6), 2018 (volume labelled 'Volume I No. 3 2017'). *Chapter in Transactions on Intelligent Welding Manufacturing, Volume I No. 3 2017 (eds. Shanben Chen, Yuming Zhang, Zhili Feng), Springer Nature Singapore, 2018, pp. 61-77, doi:10.1007/978-981-10-8330-3_4; whole volume ISBN 978-981-10-8329-7, eBook ISBN 978-981-10-8330-3, doi:10.1007/978-981-10-8330-3, ISSN 2520-8519* | Hand-eye-calibrated laser-stripe sensor on a welding robot: subpixel stripe-centre extraction and trajectory-queue interpolation build a point cloud in the robot base frame, a KD-tree background model removes background points, RANSAC fits the workpiece planes, and the initial welding position is located on the seam taken as the intersection of the fitted planes. Verified experimentally with abnormal noise objects present. | superficial/ | Ch2 Weld seam detection from 3D data (laser-stripe point cloud, RANSAC planes, seam as plane intersection); Ch3 Seams from the registered assembly (planar seams); Ch2 Camera calibration for eye-in-hand systems (laser sensor hand-eye) | - |
| 123 | **Path Planning with Automatic Seam Extraction over Point Cloud Models for Robotic Arc Welding**<br>Zhou et al. (5), 2021 (journal); arXiv 2020. *arXiv:2011.11951v1 [cs.RO] (24 Nov 2020) on the PDF; published in IEEE Robotics and Automation Letters 6(3):5002-5009, 2021, doi:10.1109/LRA.2021.3070828 (online, not on the PDF)*<br>Copies: weld_generator/papers/Path  Planning  with  Automatic  Seam ...pdf (byte-identical) | Hand-gesture-guided scanning of partial views, linear-ICP model reconstruction, extended bilateral denoising, and an intensity-based edge-point seam extractor that generates a smooth 6-DOF welding path; several joint types. | For Welding Dataset/non fem/ | Ch2 Weld seam detection from 3D data; Ch3 Operator interaction (gesture-controlled welding cell precedent); Ch3 Multi-view refinement | - |
| 131 | **3D curve weld seam path and posture planning based on line laser sensors**<br>Wang et al. (6), 2025. *Robotics and Computer-Integrated Manufacturing 94, 102985 (2025), doi:10.1016/j.rcim.2025.102985*<br>Copies: weld_generator/papers/3D curve weld seam path ...pdf (byte-identical) | Builds a seam point cloud from multi-frame line-laser scans using robot poses, extracts the seam with RANSAC, and splits torch posture into deflection, elevation and rotation angles computed separately for 3D curved seams. | For Welding Dataset/non fem/non DL/ | Ch2 Weld seam detection from 3D data; Ch3 Reachability (torch posture along curved seams); Ch4 Curved seams | - |
| 132 | **A novel 3D vision-based robotic welding path extraction method for complex intersection curves**<br>Geng et al. (4), 2024. *Robotics and Computer-Integrated Manufacturing 87, 102702 (2024), doi:10.1016/j.rcim.2023.102702* | For co-main intersecting pipes: an analytic model of the intersection curves at any clamping pose, multi-view stitching with eye-in-hand calibration and ICP, and fitting of the model parameters to the cloud to get the actual welding path without teaching. | For Welding Dataset/non fem/non DL/ | Ch2 Weld seam detection from 3D data (model-based); Ch3 Curved seams (analytic saddle curves); Ch3 Multi-view refinement; Ch4 Curved seams by construction | - |
| 133 | **A novel method for dense point cloud reconstruction and weld seam detection for tubesheet welding robot**<br>Wang et al. (4), 2023. *Optics and Laser Technology 163, 109346 (2023), doi:10.1016/j.optlastec.2023.109346*<br>Copies: weld_generator/papers/A novel method for dense point cloud ...pdf (byte-identical) | Monocular camera + laser rangefinder: calibrated multi-sensor fusion, vocabulary-tree similarity, RANSAC matching, EPnP and PMVS dense reconstruction of a tubesheet, then weld detection by voxel point density; row/column length errors < 1%. | For Welding Dataset/non fem/non DL/ | Ch2 Weld seam detection from 3D data; probably minor | - |
| 135 | **A novel seam extraction and path planning method for robotic welding of medium-thickness plate structural parts based on 3D vision**<br>Geng et al. (6), 2023. *Robotics and Computer-Integrated Manufacturing 79, 102433 (2023), doi:10.1016/j.rcim.2022.102433*<br>Copies: weld_generator/papers/A novel seam extraction and path planning ...pdf (byte-identical) | Improved RANSAC multi-plane fitting gives the intersection lines of the plates of three medium-thickness part models, from which seams, welding paths and a dihedral-based torch posture are planned without teaching. | For Welding Dataset/non fem/non DL/ | Ch2 Weld seam detection from 3D data (plane fitting/intersection); Ch3 Planar seams (eq:plane_intersection) | `geng2023seam` |
| 136 | **A Feature Extraction Approach Over Workpiece Point Clouds for Robotic Welding**<br>Zhang et al. (6), 2025. *IEEE Transactions on Automation Science and Engineering 22:75-84 (2025), doi:10.1109/TASE.2023.3345868*<br>Copies: weld_generator/papers/A_Feature_Extraction_Approach_...pdf (byte-identical) | Proposes the LOBB local feature descriptor, a nonlinear activation of the descriptor, and K-means-based hierarchical clustering that extracts crease, boundary and corner points of workpiece clouds as the basis for weld extraction. | For Welding Dataset/non fem/non DL/ | Ch2 Weld seam detection from 3D data; Ch4 Evaluation protocol (origin of the LOBB descriptor used by the 'lobb' baseline) | - |
| 138 | **A novel laser vision-based robotic welding path extraction and tracking method for intersecting curves**<br>Ren et al. (6), 2026. *Measurement 258, 119207 (2026), doi:10.1016/j.measurement.2025.119207* | Large depth-of-field line-laser scan of intersecting-pipe seams, RANSAC + adaptive GA-PSO curve extraction, NURBS reconstruction and interpolation, then real-time tracking with a bidirectional sliding-window point extractor and local curve correction; max deviation 0.83 mm. | For Welding Dataset/non fem/reading/ | Ch2 Weld seam detection (intersecting curves, seam tracking); Ch4 Curved seams by construction; Ch7 Future work (seam tracking) | - |
| 140 | **A point cloud-based welding trajectory planning method for plane welds**<br>Zhang et al. (6), 2023. *The International Journal of Advanced Manufacturing Technology 125:1645-1659 (2023), doi:10.1007/s00170-022-10699-x* | Plane point-cloud density for line/plane segmentation, iterative RANSAC edge-line segmentation and centroid-based feature points, then least-squares seam fitting for V-butt, I-butt and lap plane welds without teaching. | For Welding Dataset/non fem/reading/ | Ch2 Weld seam detection from 3D data (geometric, same Shandong group as LOBB/Geng) | - |
| 147 | **Feature extraction and robot path planning method in 3D vision-guided welding for multi-blade wheel structures**<br>Zhang et al. (4), 2024. *Optics and Lasers in Engineering 176, 108066 (2024), doi:10.1016/j.optlaseng.2024.108066* | Eye-to-hand camera plans the shooting poses (DBSCAN segmentation of blades), an eye-in-hand camera captures the welds; a concave-convex 'flatness' descriptor with K-means clustering extracts weld points, and torch posture follows the neighbourhood centroid / dihedral principle. | For Welding Dataset/non fem/reading/ | Ch2 Weld seam detection from 3D data; Ch3 Multi-view close-range refinement (view planning with an eye-in-hand camera) | - |
| 150 | **Laser scanning-based weld seam parameter extraction for slotted V-groove welds in shipbuilding**<br>Zhang et al. (5), 2026. *Measurement 284, 122195 (2026), doi:10.1016/j.measurement.2026.122195* | Laser point-cloud algorithm for slotted V-grooves (with root face / butt-joint): front/side cloud separation, rectangular-plane recognition, curvature boundaries and RANSAC+endpoint groove angle; MAE of root gap 0.22 mm. | For Welding Dataset/non fem/reading/ | Ch2 Welded joints and standards (groove geometry, root gap, ISO 9692-1 context); Ch3 Fit-up gap | - |
| 153 | **Weld seam positioning method for structural reinforcement based on 3D point cloud**<br>Li and Qin (2), 2024. *Journal of Physics: Conference Series 2896, 012002 (2024), ICETAC-2024, doi:10.1088/1742-6596/2896/1/012002* | Structured-light cloud of reinforcement structures; RANSAC separates side and bottom plates and curvature-rich weld points are selected under the plane constraint (or derived from the plane intersection when the weld is not directly visible). | For Welding Dataset/non fem/reading/ | Ch2 Weld seam detection (plane-intersection seams); probably minor | - |
| 154 | **Welding Seam Extraction from Point Cloud of Large-Sized Structural Steel Workpieces**<br>Zhao et al. (4), 2023. *2023 IEEE International Conference on Robotics and Biomimetics (ROBIO), doi:10.1109/ROBIO58561.2023.10354592* | For large structural-steel beams with large placement deviations: cheap workpiece pose estimation, generated viewpoints over the weld areas, multi-view capture and a new weld extraction method; tested on a runway beam. | For Welding Dataset/non fem/reading/ | Ch2 Weld seam detection; Ch3 Multi-view close-range refinement (pose first, then views) | - |

Notes:

- **004**: File name starts with a space and has a space before '.pdf' (quote the path when scripting). Byte-identical copy (same MD5 48e005e6f1b1e3f901e0d6e2869dafc1) at paper_pool/"Intelligent and Adaptive System for Welding Process.pdf" (tag 018, not indexed separately); one copy can be deleted.
- **011**: Application is weld-bead grinding, not welding.
- **017**: SSRN preprint marked 'has not been peer reviewed'; watermark text is interleaved in the extracted text. Year not printed (PDF created May 2025, latest references 2025). PDF author metadata is the second author (Chong Liu).
- **049**: Same paper as tag 145 with identical body text. This file is the IEEE Xplore published version (IROS header, ISBN line, pp. 7350-7355, METU download stamp); tag 145 is a Word-generated author manuscript (created 31 Jul 2018) without header or page numbers and with leftover template metadata (Subject 'IEEE Transactions on Magnetics', Author 'Lotito, Jessica M.'). Prefer this copy. [copy 145] Same paper as tag 049; this copy is the author/pre-print version without the IROS header (metadata says 'IEEE Transactions on Magnetics' - a template artifact). Keep 049 (published version).
- **050**: File name is truncated ('...from RGB-depth images.pdf'). Published online 13 Nov 2020; issue year 2021.
- **053**: File name uses non-breaking hyphens (U+2011) and is truncated.
- **082**: Not a thesis: the file is the WHOLE 180-page Springer volume 'Transactions on Intelligent Welding Manufacturing, Volume I No. 3 2017' (© 2018; papers selected from IWIWM2017). The named paper is the chapter on pp. 61-77 (PDF pp. 68-84). The volume also holds 'Real-Time Implementation of a Joint Tracking System in Robotic Laser Welding Based on Optical Camera' (Q. Zhang et al., p. 99), which may matter for seam tracking.
- **123**: Copy of weld_generator/papers/Path Planning ... (same bytes). Closest precedent in the pool for gesture control of a welding robot. BibTeX in Part 3.4.
- **131**: Copy of weld_generator/papers/3D curve weld seam ... (same bytes); '_compressed' file.
- **132**: Closest analogue to the cell's 'seams from registered primitives' for pipe intersections; recommended citation.
- **133**: Copy of weld_generator/papers/A novel method for dense point cloud ... (same bytes).
- **135**: Copy of weld_generator/papers/A novel seam extraction ... (same bytes). Bib matches the PDF; add DOI.
- **136**: Copy of weld_generator/papers/A_Feature_Extraction_... (same bytes). Not in thesis.bib, but it is the original LOBB paper; cite it next to zhang_lobb.
- **140**: Author names printed surname-first on the PDF (Zhang Yuankai = Yuankai Zhang).
- **153**: Conference-series paper; low weight as a citation. PDF metadata author/title are wrong.

### B3. Weld seam detection - learning-based and 2D-3D fusion methods (Ch2 seam detection; tab:lit_seam)

| Tag | Paper | Gist | Folder | Supports | bib |
|---|---|---|---|---|---|
| 120 | **A Coarse-to-Fine Multimodal Detection Framework Based on Deep Learning for Robotic Coating Tasks**<br>Lin et al. (6), 2026. *IEEE/ASME Transactions on Mechatronics 31(1):639-650 (Feb 2026), doi:10.1109/TMECH.2025.3595263*<br>Copies: weld_generator/papers/A_Coarse-to-Fine_Multimodal_...pdf (byte-identical) | Robotic seam sealing/gluing: a 2D seam segmentation & classification network for coarse localisation and a 3D seam refinement & transformation network (3D-SRTN) on the point cloud, then smooth 6-DoF paths; introduces an RGB-D coating dataset with 2D and 3D labels. | For Welding Dataset/non fem/ | Ch2 Weld seam detection from 3D data; Ch2 Datasets and ground truth for weld seams (annotated RGB-D seam dataset) | `lin2026coating` |
| 122 | **LWSNet: A lightweight network for automated welding point cloud segmentation**<br>Song et al. (5), 2025. *Measurement 243, 116290 (2025), doi:10.1016/j.measurement.2024.116290*<br>Copies: weld_generator/papers/LWSNet ...pdf (byte-identical) | PointNet++-based part segmentation of weld regions with depthwise-separable convolutions, normalisation-based attention, linear bottleneck and adaptive max pooling; laser-vision-sensor model and the augmented WeldJoint-PCD dataset; ShapeNetPart cls/ins mIoU 82.9/85.5%. | For Welding Dataset/non fem/ | Ch2 Weld seam detection from 3D data; Ch2 Datasets and ground truth for weld seams (tab:lit_seam) | `song2025lwsnet` |
| 124 | **PAEAR: Point Clouds Area Exploration and Active Recognition method driven by reinforcement learning for robotic welding**<br>Tao et al. (9), 2026. *Robotics and Computer-Integrated Manufacturing 102, 103340 (2026), doi:10.1016/j.rcim.2026.103340*<br>Copies: weld_generator/papers/Point Clouds Area Exploration ...pdf (byte-identical) | For workpieces without CAD: an interpretable weld local feature descriptor (W-LFD) with a small-sample prior library, and an RL agent that actively explores point-cloud regions with translation-scaling actions to find and segment weld areas of several joint types. | For Welding Dataset/non fem/ | Ch2 Weld seam detection from 3D data (CAD-free); Ch2 Summary and research gap | - |
| 125 | **SANet: A Pure Vision Strip-Aware Network with PSSCA and Multistage Fusion for Weld Seam Detection**<br>Zhu et al. (6), 2025. *Applied Sciences 15(20), 11296 (2025), doi:10.3390/app152011296*<br>Copies: weld_generator/papers/SANet_ ...pdf (byte-identical) | U-shaped 2D segmentation network with a parallel strip & spatial context-aware module, strip pooling and multistage fusion for elongated weld seams; > 4000 self-collected images, IoU 96.23%, Dice 98.07%. | For Welding Dataset/non fem/ | Ch2 Weld seam detection (image-only methods; tab:lit_seam); Ch2 Datasets and ground truth | `zhu2025sanet` |
| 126 | **Weld seam detection based on 2D image and 3D point cloud segmentation: a coarse-to-fine weld path planning method**<br>Ma et al. (7), 2025. *The International Journal of Advanced Manufacturing Technology 141:5665-5679 (2025), doi:10.1007/s00170-025-16967-w*<br>Copies: weld_generator/papers/Weld seam detection based on 2D image ...pdf (byte-identical) | EfficientWeld network locates seam regions and end points in images to guide a structured-laser fine scan, enhanced gray-gravity stripe extraction, and a pose model for curved seams; path RMSE < 0.25 mm, max lateral error < 0.61 mm. | For Welding Dataset/non fem/ | Ch2 Weld seam detection from 3D data (2D-to-3D coarse-to-fine) | - |
| 129 | **WeldNet: A voxel-based deep learning network for point cloud annular weld seam detection**<br>Wang et al. (6), 2024. *Science China Technological Sciences 67(4):1215-1225 (2024), doi:10.1007/s11431-023-2569-1*<br>Copies: weld_generator/papers/WeldNet.pdf (byte-identical) | Voxel feature encoding adapted to annular (tube-to-tubesheet) seams, sparse convolution and an RPN with an annular-seam loss on RGB-D/photogrammetry point clouds; +23% detection success vs RANSAC, +17.8% IoU vs U-Net. | For Welding Dataset/non fem/ | Ch2 Weld seam detection from 3D data (learned, curved/annular seams); Ch2 Datasets | - |
| 139 | **A novel robot welding path planning system for medium-thick plate based on 3D visual perception**<br>Liu et al. (7), 2025. *Measurement 252, 117365 (2025), doi:10.1016/j.measurement.2025.117365* | Improved SA-SSD 3D object detector (TripletAttention, EPSANet, 3D-MPDIoU loss) finds seam end points on medium-thick plates, then an improved cubic spline fits the path; planning time < 22% of teach-and-playback. | For Welding Dataset/non fem/reading/ | Ch2 Weld seam detection from 3D data (3D detection-based) | - |
| 141 | **A weld feature extraction method based on lightweight instance segmentation and point cloud features**<br>Pan et al. (6), 2025. *Industrial Robot: the international journal of robotics research and application (Emerald), doi:10.1108/IR-07-2025-0236 (received 8 Jul 2025, accepted 15 Oct 2025; volume/pages not printed)* | YOLO-Fast instance segmentation maps image pixels to the point cloud for ROI extraction, and a RANSAC-oriented-bounding-box (R-OBB) algorithm localises weld feature points on tower-base structures; RMSE < 2.98 mm. | For Welding Dataset/non fem/reading/ | Ch2 Weld seam detection from 3D data (2D-to-3D) | - |
| 142 | **Accurate Detection of Weld Seams for Laser Welding in Real-World Manufacturing**<br>Ali et al. (4), 2023. *The Thirty-Seventh AAAI Conference on Artificial Intelligence (AAAI-23) (page numbers not printed)* | CNN pipeline for laser-welding seam images: an edge classifier rejects abnormal images and a modified U-Net predicts the seam as a 1D edge mask; 91% of seams within +-1 pixel (0.023 mm). | For Welding Dataset/non fem/reading/ | Ch2 Weld seam detection (2D image methods); probably minor | - |
| 146 | **Efficient Multiple Seams Detection via CAD-Guided Image Segmentation for Robotic Welding**<br>Cheng et al. (6), 2025. *2025 IEEE International Conference on Real-time Computing and Robotics (RCAR), Toyama, Japan, June 1-6 2025, doi:10.1109/RCAR65431.2025.11139481* | Removes the background using the workpiece CAD model, segments multiple seams in the image (cheap compared with point-cloud processing) and maps them to the 3D point cloud to get robot-executable paths; real-robot experiments. | For Welding Dataset/non fem/reading/ | Ch2 Weld seam detection (CAD-guided); Ch2 Summary and research gap (CAD + perception) | - |
| 155 | **A novel method: YOLO-CE and 3D point cloud-based feature extraction for welding seams of tower bases**<br>Pan et al. (5), 2025. *Measurement Science and Technology 36, 015209 (2025), doi:10.1088/1361-6501/ad89e8* | YOLOv8-based YOLO-CE segments the target region in 2D, the mapped point cloud is plane-segmented with MSAC, weld lines come from plane equations and are refined, for power-tower base structures. | For Welding Dataset/non fem/reading/ | Ch2 Weld seam detection from 3D data (2D-to-3D, plane intersection) | - |
| 156 | **3DWS: reliable segmentation on intelligent welding systems with 3D convolutions**<br>Fernandez et al. (4), 2025. *Journal of Intelligent Manufacturing 36:5-18 (2025; online 31 Oct 2023), doi:10.1007/s10845-023-02230-0* | UNetL++ (UNet/UNet++-based 3D CNN) semantically labels voxelised stereo point clouds of freshly welded parts to recover weld bead shape and volume for quality control; DSC 0.935-0.941. | For Welding Dataset/non fem/reading/ | Ch2 Weld seam detection (post-weld bead segmentation; contrast with pre-weld seams) | - |
| 157 | **A novel welding path generation method for robotic multi-layer multi-pass welding based on weld seam feature point**<br>Xu et al. (6), 2023. *Measurement 216, 112910 (2023), doi:10.1016/j.measurement.2023.112910* | FPLDN network detects seam feature points per pass during torch round trips, adaptive weighted PCA normals and improved RANSAC fit the weld, and torch position/posture are estimated near the feature points; RMSE < 0.771 mm. | For Welding Dataset/non fem/reading/new pickeds/ | Ch2 Weld seam detection (multi-pass, laser profiles); probably minor | - |
| 159 | **BoT-YOLOv8: a highly accurate and stable initial weld position segmentation method for medium-thickness plate**<br>Liu et al. (6), 2025. *The International Journal of Advanced Manufacturing Technology 137:3813-3831 (2025), doi:10.1007/s00170-025-15353-w* | Bottleneck Transformer + YOLOv8 with atrous SPP and HSV post-processing segments initial weld positions on medium-thickness plates; 93.1% mAP0.5 at 26.5 FPS (RTX 3060). | For Welding Dataset/non fem/reading/new pickeds/ | Ch2 Weld seam detection (2D initial weld position) | - |

Notes:

- **120**: Copy of weld_generator/papers/A_Coarse-to-Fine_... (same bytes). Bib mismatch: pages '639' -> '639--650' (last page 650 verified on p.12); add DOI.
- **122**: Copy of weld_generator/papers/LWSNet ... (same bytes). Bib matches the PDF; add DOI.
- **124**: Copy of weld_generator/papers/Point Clouds Area Exploration ... (same bytes).
- **125**: Copy of weld_generator/papers/SANet ... (same bytes). Bib matches (initials only; full first names: Zhijian, Haoran, Zhao, Lijie, Guoli, Qinghui).
- **126**: Copy of weld_generator/papers/Weld seam detection based on 2D image ... (same bytes).
- **129**: Copy of weld_generator/papers/WeldNet.pdf (same bytes). PDF metadata title is 'SCTS-2023-0711 1..11'.
- **141**: Early-view article; volume/issue to be assigned.
- **142**: 2D image-only method despite the folder.
- **146**: Same group as tag 121 (Wei Zhang; P. Wei and S. Cheng are co-authors of both).
- **155**: Same group as tag 141.
- **156**: Post-weld quality task, not seam localisation before welding.
- **159**: Same first author as tag 139.

### C. Datasets, synthetic data and ground truth (Ch2 "Datasets and ground truth"; Ch4 sensor model and rendering)

| Tag | Paper | Gist | Folder | Supports | bib |
|---|---|---|---|---|---|
| 019 | **Point Cloud-Based Deep Learning in Industrial Production: A Survey**<br>Liu et al. (4), 2025. *ACM Computing Surveys 57(7), Article 173, 36 pp. (February 2025), doi:10.1145/3715851* | Survey of point-cloud deep learning in industrial production by application (pose estimation, defect inspection, measurement and estimation, others), with real-time methods per scenario, evaluation metrics and public industrial point cloud datasets; discusses dataset, speed and product-specific challenges. | (pool root) | Ch2 Object recognition and 6D pose estimation (overview of industrial point-cloud pose estimation); Ch2 Datasets and ground truth for weld seams (public industrial point cloud datasets, data scarcity). | - |
| 027 | **Sheet metal localization using deep learning and synthetic data**<br>Behnen et al. (6), 2026. *Journal of Intelligent Manufacturing 37, 399-415 (2026), doi:10.1007/s10845-024-02552-7* | Locates raw rectangular sheet metal in camera images of production machines with a two-stage deep key-point (corner) detector trained on data from a synthetic image generation pipeline. On 1,400 real production images the mean corner error is 2.17 px, close to the human annotation error. | (pool root) | Ch2 Datasets and ground truth for weld seams (synthetic training data for metal-part perception, not welding); Ch4 Sensor model and rendering (synthetic image pipeline evaluated on real images). | - |

Notes:

- **019**: PDF metadata empty.
- **027**: Published online 3 January 2025; issue year 2026.

Cross-references: Papers elsewhere in the index that build or release welding / industrial datasets: 120 (RGB-D coating dataset, 2D+3D labels, project page), 122 (WeldJoint-PCD, laser-sensor model + augmentation), 129 (annular weld seam dataset), 125 (4000+ weld images), 045 (RealSense D435i scans of 3D-printed Cranfield parts with ground-truth poses; dataset and code released), 160 (Welding-Station multi-instance registration set), 058 (rule-based synthetic industrial objects with corruptions), 037 / 167 (synthetic-only training of trackers / pose models), 028 (virtual 'as-scanned' point clouds). All welding seam datasets in the pool are hand-annotated and private or 'on request' - this is the gap WeldSet addresses.

### D1. 6D pose estimation - novel / unseen objects with a CAD model (Ch2 pose estimation; Ch3 registration)

| Tag | Paper | Gist | Folder | Supports | bib |
|---|---|---|---|---|---|
| 039 | **MatchU: Matching Unseen Objects for 6D Pose Estimation from RGB-D Images**<br>Huang et al. (6), 2024 (inferred). *IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR), pp. 10095-10105 (CVF Open Access version; year taken from the CVF file name)*<br>Copies: 096: rgbd to cad/ | Fuse-Describe-Match method for unseen objects: rotation-invariant geometric descriptors are learned for the CAD model and the RGB-D scene, RGB is fused in through a Latent Fusion Attention Module and a Bridged Coarse-level Matching loss, and the pose follows from correspondences without per-object retraining or rendering. The descriptors capture object symmetries without symmetry labels; reported to beat prior unseen-object methods in accuracy and speed (no numbers on pp. 1-2). | realtime scan to cad/ | Ch2 Object recognition and 6D pose estimation (novel-object CAD-based RGB-D pose, correspondence-based alternative to render-and-compare); Ch3 Perception and registration (symmetric and cut parts) | - |
| 088 | **Accurate and efficient zero-shot 6D pose estimation with frozen foundation models**<br>Caraffa et al. (3), 2025 (arXiv). *arXiv:2506.09784v1 [cs.CV] (11 Jun 2025); IEEE-journal-style preprint, venue not stated* | FreeZeV2: training-free zero-shot RGB-D 6D pose estimation of unseen CAD objects with frozen geometric and vision foundation-model features: dense object features precomputed offline, sparse scene features inside the mask, sparse-to-dense matching with RANSAC registration, ICP refinement, a feature-aware scoring, and ensembles of segmentation models. On the 7 BOP core datasets it is 8x faster than FreeZe and 5 % more accurate with the same masks; Best Overall Method at the BOP Challenge 2024. | rgbd to cad/ | Ch2 Object recognition and 6D pose estimation (training-free zero-shot, BOP 2024 winner; alternative to FoundationPose) | - |
| 089 | **Any6D: Model-free 6D Pose Estimation of Novel Objects**<br>Lee et al. (6), 2025 (arXiv). *arXiv:2503.18673v2 [cs.CV] (25 Mar 2025); CVPR 2025 paper (venue not printed; inferred)* | Model-free 6D pose and size estimation of novel objects from a single RGB-D anchor image: an image-to-3D model generates the shape, a joint 2D/3D object alignment fixes metric scale, and render-and-compare generates and refines pose hypotheses. Evaluated on REAL275, Toyota-Light, HO3D, YCBInEOAT and LM-O. | rgbd to cad/ | Ch2 Object recognition and 6D pose estimation (model-free novel objects; probably minor since the cell has CAD) | - |
| 091 | **Efficient encoding and aligning viewpoints for 6D pose estimation of unseen industrial parts**<br>Xu et al. (6), 2025. *Measurement Science and Technology 36, 016034 (2025), 13 pp., doi:10.1088/1361-6501/ad9160* | Depth-only 6D pose estimator for unseen industrial parts: a Self-to-Inter (S2I) viewpoint encoder with an inter-viewpoint attention module and saliency descriptor selection retrieves the viewpoint, and a viewpoint alignment module integrated with an ICP refiner corrects non-uniform viewpoints. Competitive on LINEMOD and validated on eight unseen industrial parts. | rgbd to cad/ | Ch2 Object recognition and 6D pose estimation (unseen industrial parts, ICP refinement); Ch3 Perception and registration | - |
| 093 | **FreeZe: Training-free zero-shot 6D pose estimation with geometric and vision foundation models**<br>Caraffa et al. (4), 2025 (arXiv v3; first submitted Dec 2023 per id 2312). *arXiv:2312.00947v3 [cs.CV] (8 Jan 2025), 34 pp. incl. supplementary; ECCV 2024 paper (venue not printed; inferred)* | Training-free zero-shot 6D pose estimation from RGB-D and a CAD model: fuses 3D geometric descriptors (GeDi) with 3D-lifted DINOv2 visual features into point descriptors, estimates the pose by RANSAC-based 3D registration, and resolves geometrically symmetric cases with visual features. Outperforms trained zero-shot methods on the 7 BOP core datasets. | rgbd to cad/ | Ch2 Object recognition and 6D pose estimation (training-free; registration with foundation features); Ch3 Symmetric and cut parts (symmetry disambiguation) | - |
| 094 | **GenFlow: Generalizable Recurrent Flow for 6D Pose Refinement of Novel Objects**<br>Moon et al. (4), 2024 (arXiv). *arXiv:2403.11510v1 [cs.CV] (18 Mar 2024); CVPR 2024 paper (venue not printed; inferred)* | Generalisable optical-flow-based iterative pose refinement for novel objects, combining shape-constraint recurrent flow, a differentiable PnP layer and a cascade multi-scale correlation design (RGB; RGB-D via RANSAC-Kabsch). Ranked first on the BOP Challenge 2023 unseen-object pose tasks for both RGB and RGB-D. | rgbd to cad/ | Ch2 Object recognition and 6D pose estimation (novel-object refinement; probably minor) | - |
| 095 | **GigaPose: Fast and Robust Novel Object Pose Estimation via One Correspondence**<br>Nguyen et al. (4), 2024 (arXiv v2; first submitted Nov 2023 per id 2311). *arXiv:2311.14155v2 [cs.CV] (15 Mar 2024), 16 pp. incl. supplementary; CVPR 2024 paper (venue not printed; inferred)* | RGB CAD-based coarse pose estimation of novel objects: templates cover only azimuth and elevation (2 DoF) and are matched by fast nearest-neighbour search in feature space, and in-plane rotation, scale and 2D translation come from a single patch correspondence (with RANSAC). 35x faster than MegaPose per detection (0.048 s vs 1.68 s), more robust to segmentation errors, +3.5 % AP on BOP; also works with 3D models reconstructed from one image. | rgbd to cad/ | Ch2 Object recognition and 6D pose estimation (fast coarse pose for novel objects; probably minor) | - |
| 097 | **Pos3R: 6D Pose Estimation for Unseen Objects Made Easy**<br>Deng et al. (7), 2025. *2025 IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR), pp. 16818-16828, doi:10.1109/CVPR52734.2025.01567* | Training-free RGB 6D pose estimation of unseen objects that uses the 3D foundation model MASt3R to match test crops to 40 rendered CAD templates (8 cube-vertex views x 5 in-plane rotations) and solves the 2D-3D correspondences with PnP-RANSAC. Competitive with or better than refinement-free methods on the 7 BOP datasets and compatible with render-and-compare refinement. | rgbd to cad/ | Ch2 Object recognition and 6D pose estimation (training-free RGB; probably minor) | - |
| 098 | **SAM-6D: Segment Anything Model Meets Zero-Shot 6D Object Pose Estimation**<br>Lin et al. (4), 2024. *2024 IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR), pp. 27906-27916, doi:10.1109/CVPR52733.2024.02636* | Zero-shot RGB-D 6D pose estimation of novel objects in two steps: an Instance Segmentation Model scores SAM proposals by semantic, appearance and geometric matching to rendered templates, and a Pose Estimation Model performs coarse-to-fine partial-to-partial point matching with background tokens. Beats prior methods on the 7 BOP core datasets for both instance segmentation and pose. | rgbd to cad/ | Ch2 Object recognition and 6D pose estimation; Ch3 Perception and registration (SAM-based segmentation plus point matching, an alternative to SAM2 + FoundationPose) | - |
| 099 | **SuperPose: Improved 6D Pose Estimation with Robust Tracking and Mask-Free Initialization**<br>Deng et al. (6), 2024 (arXiv). *arXiv:2409.19986v2 [cs.CV] (20 Oct 2024); ICML-template footer 'Proceedings of the 41st International Conference on Machine Learning, Vienna, Austria. PMLR 235, 2024' with unfilled AUTHORERR placeholders (acceptance not evidenced)* | Real-time industrial 6D pose pipeline combining FoundationPose, SAM2 and LightGlue: a one-time click (later a saved reference image matched by LightGlue) prompts SAM2 for the first-frame mask, and centroid-distance checks plus feature re-matching re-register the object after tracking loss, with orientation correction for symmetric objects. Live tests with an Azure Kinect on YCB-Video objects (bleach cleanser) and gears; no quantitative tables. | rgbd to cad/ | Ch3 Perception and registration (segmentation and part classification; 6D pose registration and live tracking; symmetric and cut parts); Ch2 Object recognition and 6D pose estimation | - |
| 100 | **ZeroPose: CAD-Prompted Zero-Shot Object 6D Pose Estimation in Cluttered Scenes**<br>Chen et al. (7), 2025. *IEEE Transactions on Circuits and Systems for Video Technology 35(2), 1251-1264 (February 2025), doi:10.1109/TCSVT.2024.3482439* | CAD-prompted zero-shot RGB-D pose estimation with a Discovery-Orientation-Registration pipeline: SAM proposals are matched to CAD visual embeddings to find all instances, the viewing direction is estimated to keep only visible CAD points, and point matching registers the instance cloud to the filtered model, all by feature matching without online rendering. Comparable to object-specific methods on 7 BOP datasets and 50x faster than the best zero-shot method. | rgbd to cad/ | Ch2 Object recognition and 6D pose estimation (CAD-prompted zero-shot; bin picking); Ch3 Perception and registration | - |
| 101 | **Zero-Shot 3D Pose Estimation of Unseen Object by Two-step RGB-D Fusion**<br>Duan et al. (6), 2024. *Neurocomputing 597, 128041 (2024), doi:10.1016/j.neucom.2024.128041* | Template-matching zero-shot estimation of object identity and orientation for unseen objects: RGB-D templates are rendered from the CAD model (BlenderProc) and embedded by PoseFusion (cross-modality alignment plus geometry-guided cross-attention fusion of texture and geometric features), then matched to the query by masked local similarity. Outperforms template-based baselines on LINEMOD and T-LESS. | rgbd to cad/ | Ch2 Object recognition and 6D pose estimation (texture-less unseen parts, template matching; probably minor) | - |
| 102 | **KeyMatchNet: Zero-Shot Pose Estimation in 3D Point Clouds by Generalized Keypoint Matching**<br>Hagelskjær and Haugaard (2), 2024 (arXiv v3; first submitted Mar 2023 per id 2303). *arXiv:2303.16102v3 [cs.CV] (29 Aug 2024); IEEE-conference-style preprint, venue not stated* | Depth-only zero-shot pose estimation in point clouds: parallel object and scene feature networks (object features precomputable) predict instance segmentation and object-keypoint matches, followed by Kabsch-RANSAC. Trained on synthetic BlenderBin/BlenderProc homogeneous-bin scenes of 1,500 electronic-component CAD models; tested on 7 unseen components, 7 WRS industrial objects (out of class) and real data, compared with FPFH. | depth to cad/ | Ch2 Object recognition and 6D pose estimation (depth-only zero-shot, industrial bin picking); Ch2 Point cloud registration | - |
| 167 | **FoundationPose: Unified 6D Pose Estimation and Tracking of Novel Objects**<br>Wen et al. (4), 2024. *arXiv:2312.08344v2 [cs.CV] (26 Mar 2024) on the PDF; CVPR 2024, pp. 17868-17879 (online, CVF open access)*<br>Copies: 092: rgbd to cad/ | One RGB-D network for model-based (CAD) and model-free (~16 reference images, neural SDF object field) novel-object pose estimation and tracking: icosphere pose hypotheses, transformer render-and-compare refiner, hierarchical (two-level self-attention) pose ranking trained with a pose-conditioned triplet loss, LLM-aided texture-augmented synthetic training in Isaac Sim. | supply_papers/ | Ch2 Object recognition and 6D pose estimation; Ch3 6D pose registration and live tracking | `wen2024foundationpose` |

Notes:

- **039**: Same paper and content as tag 096 (rgbd to cad/MatchU_ Matching Unseen Objects ... RGB-D Images.pdf), which is the arXiv:2403.01517v2 preprint (8 May 2024); full texts differ only in watermark/arXiv stamp, page numbers and one figure position. This CVF copy is the citable CVPR version; the year is not printed in the text. [copy 096] Yes, this is the arXiv version (v2, 8 May 2024). Tag 039 (realtime scan to cad/Huang_MatchU_..._CVPR_2024_paper.pdf) is the CVF open-access CVPR 2024 version of the same paper; both 11 pages.
- **088**: File name truncates the title ('with frozen foundation models' missing). Second generation of FreeZe (tag 093).
- **089**: arXiv v2; venue not printed. Co-author Bowen Wen (FoundationPose).
- **091**: Uses depth images only, despite the 'rgbd to cad' folder. Received 2 Apr 2024, published 26 Nov 2024.
- **093**: arXiv v3 with supplementary (34 pp.); venue not printed. Predecessor of FreeZeV2 (tag 088).
- **094**: arXiv v1; venue not printed. Builds on SCFlow (tag 078).
- **095**: arXiv v2 with supplementary. RGB-only method despite the 'rgbd to cad' folder.
- **097**: IEEE Xplore publisher version. RGB-only method despite the 'rgbd to cad' folder.
- **098**: IEEE Xplore publisher version.
- **099**: The ICML footer looks like template residue (missing \icmlaffiliation/\icmlcorrespondingauthor errors printed; arXiv v1 from Sept 2024 post-dates ICML 2024, inferred), so treat as an arXiv preprint. Very close to the thesis pipeline (SAM2 + FoundationPose, prompted mask, tracking-loss recovery, symmetric gears). File name truncates the title.
- **100**: IEEE Xplore publisher version (published online 17 Oct 2024).
- **101**: '3D pose' in the title means orientation from the best template. Same Zhejiang University CAD&CG group as tag 091.
- **102**: arXiv v3; venue not stated. File name truncates the title ('Generalized Keypoint Matching' missing).
- **167**: Same paper as paper_pool/rgbd to cad/FoundationPose_... (tag 092). Digest in Part 2. [copy 092] arXiv version v2 (arXiv stamp in the margin), NOT the CVPR open-access version (no CVF watermark). Same paper and same version as tag 167 (supply_papers/FoundationPose.pdf: same creation date, different bytes). thesis.bib wen2024foundationpose cites CVPR 2024 without pages.

### D2. 6D pose estimation - instance-level and RGB-only methods (Ch2 pose estimation, background)

| Tag | Paper | Gist | Folder | Supports | bib |
|---|---|---|---|---|---|
| 074 | **CosyPose: Consistent multi-view multi-object 6D pose estimation**<br>Labbé et al. (4), 2020 (arXiv). *arXiv:2008.08465v1 [cs.CV] (19 Aug 2020), 41 pp. incl. appendix; published at ECCV 2020 (venue not printed in this copy; inferred)* | Estimates the 6D poses of multiple known objects from several RGB images with unknown camera poses: DeepIM-style render-and-compare single-view hypotheses, robust matching of object hypotheses across views to recover the cameras, and object-level bundle adjustment that refines all camera and object poses (handles symmetries, no depth needed). State of the art on YCB-Video and T-LESS for single- and multi-view. | rgb to cad/ | Ch2 Object recognition and 6D pose estimation; Ch3 Multi-view close-range refinement (object-level multi-view consistency / bundle adjustment) | - |
| 075 | **GDRNPP: A Geometry-guided and Fully Learning-based Object Pose Estimator**<br>Liu et al. (7), 2025. *IEEE Transactions on Pattern Analysis and Machine Intelligence (2025), doi:10.1109/TPAMI.2025.3553485; this copy is arXiv:2102.12145v5 [cs.CV] (22 Mar 2025), volume/pages not printed* | Fully learning-based instance-level pose estimator: GDRN regresses the 6D pose end-to-end from monocular RGB via dense 2D-3D correspondence and surface-region features with a learnable Patch-PnP, and an optional depth refinement predicts 3D optical flow between observed and rendered RGB-D to build 3D-3D correspondences. GDRNPP was the Overall Best Method of the BOP Challenge in 2022 and 2023. | rgb to cad/ | Ch2 Object recognition and 6D pose estimation (instance-level, BOP 2022/2023 winner) | - |
| 076 | **MRC-Net: 6-DoF Pose Estimation with MultiScale Residual Correlation**<br>Li et al. (4), 2024 (arXiv). *arXiv:2403.08019v3 [cs.CV] (20 Mar 2024), 20 pp. incl. supplementary; CVPR 2024 paper (venue not printed; inferred)* | Single-shot RGB 6-DoF pose estimation for objects with a CAD model: a Siamese two-stage network first classifies the pose (soft probabilistic labels for symmetric objects) and renders it, then regresses the residual pose, linked by a multi-scale residual correlation (MRC) layer between input and rendering. Reported best RGB-only accuracy on T-LESS, LM-O, YCB-V and ITODD (+2.4 % average recall), non-iterative. | rgb to cad/ | Ch2 Object recognition and 6D pose estimation (instance-level RGB; probably minor) | - |
| 077 | **Object Pose Estimation via the Aggregation of Diffusion Features**<br>Wang et al. (3), 2024 (arXiv). *arXiv:2403.18791v3 [cs.CV] (18 Oct 2024); CVPR 2024 paper (venue not printed; inferred)* | Template-based RGB object pose estimation that aggregates multi-granularity features of a frozen Stable Diffusion model with three aggregation networks (linear, bottleneck, context-aware weighting) to generalise to unseen objects. On LM, O-LM and T-LESS: 97.9 % vs 93.5 % on Unseen LM and 85.9 % vs 76.3 % on Unseen O-LM. | rgb to cad/ | Ch2 Object recognition and 6D pose estimation (template matching for unseen objects; probably minor) | - |
| 078 | **Shape-Constraint Recurrent Flow for 6D Object Pose Estimation**<br>Hai et al. (4), 2023 (arXiv). *arXiv:2306.13266v1 [cs.CV] (23 Jun 2023); CVPR 2023 paper (venue not printed; inferred)* | SCFlow: RGB render-and-compare pose refinement in which the RAFT-style correlation volume is indexed by a pose-induced flow (reprojection of the target's 3D shape), and the pose is regressed from the current flow and updated recurrently end-to-end without a PnP solver. Outperforms prior refiners in accuracy and speed on LINEMOD, LINEMOD-Occluded and YCB-V. | rgb to cad/ | Ch2 Object recognition and 6D pose estimation (pose refinement; probably minor) | - |

Notes:

- **074**: arXiv v1 preprint with appendix (LNCS format). File name truncates the title ('6D pose estimation' missing).
- **075**: Accepted-manuscript arXiv v5 of the TPAMI paper (IEEE copyright notice, no volume/issue/pages). Journal extension of GDR-Net (CVPR 2021). File name truncates the title.
- **076**: arXiv v3 with supplementary; Amazon authors; venue not printed.
- **077**: arXiv v3; venue not printed.
- **078**: arXiv v1; venue not printed. Basis of GenFlow (tag 094).

### D3. 6D pose estimation - industrial parts and bin picking (Ch2 pose estimation; Ch3 perception)

| Tag | Paper | Gist | Folder | Supports | bib |
|---|---|---|---|---|---|
| 036 | **Self-supervised 6D Object Pose Estimation for Robot Manipulation**<br>Deng et al. (6), 2020 (arXiv v2; v1 2019). *arXiv:1909.10159v2 [cs.RO] (7 Mar 2020); no venue printed* | Robot system with a hand-mounted RGB-D camera that initialises object poses with PoseRBPF, tracks them over several views using forward kinematics as the motion prior, and uses the auto-labelled real images to fine-tune segmentation and pose networks first trained in simulation, while grasping or pushing objects to create new scenes. Self-supervised fine-tuning improves segmentation, pose accuracy and grasp success (no numbers on pp. 1-2). | 6D tracking/ | Ch2 Object recognition and 6D pose estimation (sim-to-real gap, self-annotation); Ch3 Perception and registration (eye-in-hand multi-view tracking with kinematic prior) | - |
| 051 | **6D Pose Estimation of Reflective and Textureless Object with Improved Accuracy through Multi-view Scanning Using Mobile and Stationary Cameras**<br>Lee et al. (5), 2026. *International Journal of Precision Engineering and Manufacturing 27, 2345-2362 (2026), doi:10.1007/s12541-026-01488-7* | Bin-picking pose estimation for reflective, textureless parts: YOLOv8-OBB selects the most graspable part, then a stationary depth camera and an RGB-D camera on the robot end-effector capture planned multi-view point clouds that are merged, filtered (RANSAC, farthest point sampling) and compared with the reference model. With twelve views the error is below 2 mm along z and below 2 degrees, better than with a single camera. | closely related/katman1/ | Ch2 Object recognition and 6D pose estimation (reflective industrial parts); Ch3 Multi-view close-range refinement (eye-in-hand multi-view fusion); Ch6 Results/discussion (mm-level accuracy comparison) | - |
| 055 | **Automatic robot path integration using three-dimensional vision and offline programming**<br>Bedaka et al. (3), 2019. *The International Journal of Advanced Manufacturing Technology 102, 1935-1950 (2019), doi:10.1007/s00170-018-03282-w* | Automated offline programming (AOLP) system in which a fixed Kinect and an improved point-pair-feature (PPF) voting method estimate the workpiece 6D pose, which places the CAD model in an OLP platform that then generates, simulates and executes the robot path automatically. Precision and robustness are evaluated in four sets of real-world experiments (relative and absolute pose error). | closely related/katman1/ | Ch2 Object recognition and 6D pose estimation (PPF-based workpiece localisation); Ch3 Seams from the registered assembly (paths generated on the registered CAD model) | - |
| 057 | **Real-Time AI-Driven 6D Pose Estimation for Robotic Picking in Cluttered Bins via Two-Stage Segmentation and CAD-Guided Alignment**<br>Tsvetanov et al. (3), 2025. *2025 International Conference on Cybersecurity and AI-Based Systems (Cyber-AI), pp. 285-290, doi:10.1109/Cyber-AI66431.2025.11233319* | AMROPick bin-picking system: low-resolution segmentation (YOLOv12 or SAM2) proposes regions, high-resolution segmentation refines the crops, and masked point clouds from a top-mounted RGB-D camera are aligned to CAD templates by ICP on an embedded Jetson, with ROS 2 and MoveIt integration. Claims over 98% segmentation accuracy on reflective metal parts, under 300 ms response and pose errors below 1 mm. | closely related/katman1/ | Ch2 Object recognition and 6D pose estimation (SAM2/YOLO segmentation plus CAD ICP in a ROS 2 cell); Ch3 Perception and registration (segmentation and part classification) | - |
| 113 | **Fast Object Pose Estimation Using Adaptive Threshold for Bin-Picking**<br>Yan et al. (6), 2020. *IEEE Access 8:63055-63064 (2020), doi:10.1109/ACCESS.2020.2983173* | Bin-picking 6-DoF pose pipeline with an adaptive-threshold segmentation step and geometric cues (contour, normal distribution, curvature) to speed matching; Kinova arm + Ensenso camera, 91.25% success and 0.265 s per estimate on four objects. | background segmentation/ | Ch2 Object recognition and 6D pose estimation (industrial, depth-only); probably not needed | - |
| 115 | **6D Pose Estimation of Industrial Parts Based on Point Cloud Geometric Information Prediction for Robotic Grasping**<br>Zhang et al. (5), 2024. *Entropy 26(12), 1022 (2024), doi:10.3390/e26121022* | RGB + point-cloud feature fusion with instance segmentation, then a registration step using curvature/normal-based edge features against the model for industrial-part pose; tested on LineMod and YCB-Video and on a grasping platform. | not so related/ | Ch2 Object recognition and 6D pose estimation (industrial parts); probably not needed | - |

Notes:

- **036**: arXiv preprint (NVIDIA internship work). The published version is probably ICRA 2020 (inferred, not printed in this copy; verify before citing).
- **051**: Open access; the first two authors contributed equally. File name is truncated.
- **055**: General-purpose OLP (welding is cited only as one application) with an eye-to-hand sensor; file name is truncated.
- **057**: Short paper at a cybersecurity/AI conference; the text says the system returns a graspable pose 'with ongoing development toward full 6D support', so treat the 6D and sub-millimetre claims with care.

### E. 6D pose tracking (Ch2 pose estimation and tracking; Ch3 live tracking; Ch6 FoundationPose vs ICP)

| Tag | Paper | Gist | Folder | Supports | bib |
|---|---|---|---|---|---|
| 032 | **BundleTrack: 6D Pose Tracking for Novel Objects without Instance or Category-Level 3D Models**<br>Wen and Bekris (2), 2021. *IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS) 2021; this file is arXiv:2108.00516v1; pages/DOI not visible* | Model-free 6D pose tracking of novel objects from RGB-D video: video segmentation and learned keypoint matching give coarse poses, refined by a memory-augmented pose graph over stored keyframes; the CUDA implementation runs at 10 Hz. Raises NOCS 5deg5cm accuracy from 33.3% to 87.4% and matches instance-model methods on YCBInEOAT. | 6D tracking/ | Ch2 Object recognition and 6D pose estimation (model-free 6D tracking; same first author as FoundationPose, wen2024foundationpose); Ch3 Perception and registration (live tracking). | - |
| 033 | **PWP3D: Real-time Segmentation and Tracking of 3D Objects**<br>Prisacariu and Reid (2), 2012 (inferred). *not visible (author manuscript dated 17 January 2012; extended version of Prisacariu and Reid, BMVC 2009)* | Region-based simultaneous segmentation and 6-DoF tracking with a known 3D model: maximises a pixel-wise foreground/background posterior energy over the level-set embedding of the projected silhouette, differentiated directly with respect to pose. GPU real-time implementation with temporal consistency, online appearance adaptation and multi-camera/multi-object extensions. | 6D tracking/ | Ch2 Object recognition and 6D pose estimation (classical model-based region tracking with joint segmentation); Ch3 Perception and registration (live tracking). | - |
| 034 | **PoseRBPF: A Rao-Blackwellized Particle Filter for 6D Object Pose Tracking**<br>Deng et al. (6), 2019 (arXiv). *arXiv:1905.09304v1 [cs.CV] (2019); conference/journal not printed* | Rao-Blackwellised particle filter for 6D tracking: particles sample the 3D translation and each holds a discretised rotation distribution (5 degrees, 191,808 bins) scored by matching an auto-encoder embedding against a precomputed codebook. Tracks full pose posteriors of symmetric objects without symmetry labels at about 20 fps; state of the art on YCB-Video and T-LESS. | 6D tracking/ | Ch2 Object recognition and 6D pose estimation (probabilistic 6D tracking of symmetric objects); Ch3 Perception and registration (symmetric and cut parts; live tracking). | - |
| 035 | **Probabilistic Object Tracking using a Range Camera**<br>Wüthrich et al. (5), 2013. *2013 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Tokyo, November 3-7, 2013 (pages/DOI not printed); this copy is arXiv:1505.00241v1 [cs.RO] (1 May 2015)* | Depth-only 6-DoF object tracking with a dynamic Bayesian network whose observation model explicitly handles self-occlusion and occlusion by the manipulating human or robot hand, solved with a Rao-Blackwellised particle filter (process model with or without robot control inputs). Tracks manipulated objects in real time on one CPU core, with fiducial markers used only as the baseline. | 6D tracking/ | Ch2 Object recognition and 6D pose estimation (classical depth-based tracking); Ch3 Perception and registration (6D pose registration and live tracking) | - |
| 037 | **se(3)-TrackNet: Data-driven 6D Pose Tracking by Calibrating Image Residuals in Synthetic Domains**<br>Wen et al. (4), 2020. *IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS) 2020 (printed as 'Accepted in ... IROS 2020'; pages/DOI not printed); this copy is arXiv:2007.13866v1 [cs.CV] (27 Jul 2020)* | Learned RGB-D 6D pose tracker that regresses the relative pose between the current observation and a rendering of the CAD model at the previous estimate, using disentangled feature encoding and a Lie-algebra rotation output, trained only on domain-randomised synthetic data. Reports state-of-the-art YCB-Video tracking without re-initialisation at 90.9 Hz and adds a tracking benchmark recorded with robot end-effectors and YCB objects. | 6D tracking/ | Ch2 Object recognition and 6D pose estimation (render-and-compare tracking, precursor of FoundationPose tracking); Ch3 Perception and registration (6D pose registration and live tracking); Ch4 Sensor model and rendering (synthetic-only training with domain randomisation) | - |
| 040 | **RGBTrack: Fast, Robust Depth-Free 6D Pose Estimation and Tracking**<br>Guo and Yu (2), 2025 (arXiv). *arXiv:2506.17119v1 [cs.CV] (20 Jun 2025)* | Extends FoundationPose to RGB-only pose estimation and tracking: a binary search over depth with render-and-compare recovers translation, XMem 2D tracking with a Kalman filter and a state machine detects tracking loss and re-initialises, and a scale-recovery step adapts CAD models of unknown scale using an initial depth estimate. Evaluated on YCBInEOAT and ClearPose with competitive accuracy at real-time rates. | realtime scan to cad/ | Ch2 Object recognition and 6D pose estimation; Ch3 Perception and registration (6D pose registration and live tracking: FoundationPose tracking-loss detection and recovery, Kalman filtering of poses) | - |
| 090 | **BundleSDF: Neural 6-DoF Tracking and 3D Reconstruction of Unknown Objects**<br>Wen et al. (9), 2023. *2023 IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR), pp. 606-617, doi:10.1109/CVPR52729.2023.00066* | Causal, near-real-time (10 Hz) 6-DoF tracking and neural 3D reconstruction of an unknown rigid object from monocular RGB-D with a mask in the first frame only, using online pose-graph optimisation, a concurrently learned Neural Object Field (hybrid SDF) and a memory pool of posed frames. Outperforms prior methods on HO3D, YCBInEOAT and BEHAVE. | rgbd to cad/ | Ch2 Object recognition and 6D pose estimation (model-free tracking); Ch3 Perception and registration (6D pose registration and live tracking) | - |
| 117 | **DynamicPose: Real-time and Robust 6D Object Pose Tracking for Fast-Moving Cameras and Objects**<br>Liang et al. (4), 2025. *arXiv:2508.11950v1 [cs.CV] (16 Aug 2025)* | Retraining-free tracker built on FoundationPose-style refinement: visual-inertial odometry compensates camera-induced ROI shift, a depth-informed 2D tracker handles large object translation, and a VIO-guided Kalman filter proposes candidate rotations refined hierarchically. | not so related/ | Ch3 6D pose registration and live tracking (camera-motion compensation, lost-track handling); Ch6 FoundationPose vs ICP; Ch7 Future work (Kalman/VIO was decided against) | - |

Notes:

- **032**: arXiv preprint version (v1) carrying the IROS 2021 header; PDF metadata empty.
- **033**: Author manuscript, not the publisher PDF; venue not printed (probably the IJCV 2012 journal version, inferred); extends the BMVC 2009 conference paper of the same title. PDF metadata empty.
- **034**: arXiv preprint v1, published venue not printed; PDF metadata empty (creation date placeholder 2010).
- **035**: arXiv copy (posted 2015) of the IROS 2013 paper; it carries the IROS header but no page numbers or DOI.
- **037**: arXiv accepted-version preprint, not the IEEE Xplore copy. Same first author as FoundationPose (wen2024foundationpose).
- **040**: arXiv preprint. The footnote prints the first author as 'G. Teng' (given and family names swapped; byline is Teng Guo) and the dataset as 'YCBinEAOT'. Filed under 'realtime scan to cad' although it is a tracking paper.
- **090**: IEEE Xplore publisher version (METU download stamp).
- **117**: Preprint. Directly relevant to the cell's FoundationPose tracking with a moving eye-in-hand camera.

### F1. Point cloud registration - ICP and its variants (Ch2 registration; Ch3 stationary refinement)

| Tag | Paper | Gist | Folder | Supports | bib |
|---|---|---|---|---|---|
| 044 | **Voxelized GICP for Fast and Accurate 3D Point Cloud Registration**<br>Koide et al. (4), 2021. *2021 IEEE International Conference on Robotics and Automation (ICRA), Xi'an, China, May 31-June 4, 2021, pp. 11054-11059, doi:10.1109/ICRA48506.2021.9560835* | VGICP extends GICP with voxelisation: each voxel's distribution is aggregated from the covariances of the points it contains (unlike NDT, which fits it from point positions), which removes nearest-neighbour search and allows parallel optimisation. Accuracy comparable to GICP on simulated and real LiDAR data while registering about 15,000-point clouds at 30 Hz on a CPU and 120 Hz on a GPU (fast_gicp library). | realtime scan to cad/ | Ch2 Point cloud registration (ICP variants: GICP, NDT, VGICP); Ch3 Perception and registration (stationary refinement (ICP), fast alternative for fine registration) | - |
| 107 | **Direct Iterative Closest Point for Real-time Visual Odometry**<br>Tykkala et al. (3), 2011. *2011 IEEE International Conference on Computer Vision Workshops (ICCV Workshops), doi:10.1109/ICCVW.2011.6130500* | RGB-D visual odometry that jointly minimises a photometric and a depth (ICP-like) error, robustifying direct tracking on weakly textured / non-Lambertian surfaces; demonstrated on a Mars-rover stereo sequence. | pose refinement/ | Ch2 Point cloud registration (photometric+geometric ICP variants); probably not needed | - |
| 108 | **Fast and Robust Iterative Closest Point**<br>Zhang et al. (3), 2022 (journal); arXiv v3 2023. *arXiv:2007.07627v3 [cs.CV] (14 Apr 2023); published in IEEE Trans. Pattern Analysis and Machine Intelligence 44(7):3450-3466, 2022 (journal details not on this PDF)* | Treats ICP as majorisation-minimisation, accelerates it with Anderson acceleration, and adds a robust Welsch error metric for point-to-point and point-to-plane ICP; Sparse-ICP-level robustness at least an order of magnitude faster. | pose refinement/ | Ch2 Point cloud registration; Ch3 Stationary refinement (eq:p2plane, Welsch weights) | `zhang2022fricp` |
| 109 | **Sparse Iterative Closest Point**<br>Bouaziz et al. (3), 2013. *Computer Graphics Forum 32(5):113-123 (Eurographics Symposium on Geometry Processing 2013)* | Reformulates ICP with sparsity-inducing lp norms (p<1) solved by ADMM, giving robust rigid alignment under large outlier and missing-data fractions without hand-tuned rejection heuristics. | pose refinement/ | Ch2 Point cloud registration (robust ICP); Ch3 Stationary refinement (alternative robust kernels) | - |

Notes:

- **044**: IEEE Xplore final version (METU download stamp); evaluation is LiDAR-oriented.
- **107**: Page numbers not visible on pages 1-2.
- **108**: PDF is the arXiv v3 preprint; the bib's TPAMI 44(7):3450-3466 (2022) is confirmed online (Cardiff ORCA), doi:10.1109/TPAMI.2021.3054619.

### F2. Point cloud registration - point pair features (Ch2 pose/registration; Ch3 part classification)

| Tag | Paper | Gist | Folder | Supports | bib |
|---|---|---|---|---|---|
| 169 | **Model Globally, Match Locally: Efficient and Robust 3D Object Recognition**<br>Drost et al. (4), 2010. *2010 IEEE Computer Society Conference on Computer Vision and Pattern Recognition (CVPR), pp. 998-1005, doi:10.1109/CVPR.2010.5540108*<br>Copies: 103: depth to cad/ | The original PPF method: 4D oriented point-pair feature, hash-table global model description, voting in a 2D (model point, rotation angle) accumulator per scene reference point, pose clustering and averaging. | supply_papers/ | Ch2 Object recognition and 6D pose estimation; Ch3 Segmentation and part classification | `drost2010ppf` |
| 170 | **Point Pair Features Based Object Detection and Pose Estimation Revisited**<br>Birdal and Ilic (2), 2015. *2015 International Conference on 3D Vision (3DV), pp. 527-535, doi:10.1109/3DV.2015.65*<br>Copies: 080: non DL scan to cad/; 104: depth to cad/ | Revised PPF pipeline (MLS normals, ambient-occlusion weights, segmentation-coupled disjoint voting, weighted and interpolated Hough voting, quaternion averaging, occlusion-aware verification without ICP). | supply_papers/ | Ch2 Object recognition and 6D pose estimation; Ch3 Segmentation and part classification (hypothesis verification) | `birdal2015ppf` |

Notes:

- **169**: Same paper as tag 103 (depth to cad/). Digest in Part 2. [copy 103] Same paper as supply_papers/Model_globally_match_locally_... (tag 169); both are IEEE Xplore downloads that differ only in the download watermark. Digest in Part 2.
- **170**: Same paper as tags 080 and 104. It is Birdal & Ilic (3DV 2015), NOT Hinterstoisser et al. 'Going further with point pair features' (ECCV 2016). Digest in Part 2. [copy 080] IEEE Xplore publisher version (METU download stamp 17 Jul 2026). Same paper as tags 104 (depth to cad/, stamp 23 Jul 2026) and 170 (supply_papers/, stamp 20 Oct 2026): identical publisher PDF (same creation date and metadata), the bytes differ only by the download stamp. Matches thesis.bib birdal2015ppf (3DV 2015, pp. 527-535). [copy 104] Third copy of this paper (also tags 080 'non DL scan to cad/' and 170 'supply_papers/'); all three are the IEEE Xplore version, differing only in the watermark. NOTE: this is Birdal & Ilic, not Hinterstoisser et al. 'Going further with point pair features' (ECCV 2016), which is NOT in the folders.

### F3. Point cloud registration - descriptors, global, learned and multi-instance registration; surveys (Ch2 registration)

| Tag | Paper | Gist | Folder | Supports | bib |
|---|---|---|---|---|---|
| 006 | **3D Focusing-and-Matching Network for Multi-Instance Point Cloud Registration**<br>Zhang et al. (5), 2024. *38th Conference on Neural Information Processing Systems (NeurIPS 2024); pages/DOI not visible* | Multi-instance registration of a CAD-model point cloud to a cluttered scene, decomposed into per-object pair-wise registrations: a focusing module regresses object centres with self/cross-attention to form ball-query proposals, and a dual-masking (instance + overlap mask) module registers each proposal. State of the art on Scan2CAD and the ROBI bin-picking set (about 7% higher MR/MP/MF than MIRETR on ROBI). | (pool root) | Ch2 Point cloud registration (learned multi-instance CAD-to-scene registration); Ch3 Perception and registration (several identical parts in one scene). | - |
| 007 | **A Comprehensive Survey and Taxonomy on Point Cloud Registration Based on Deep Learning**<br>Zhang et al. (5), 2024. *Proceedings of the 33rd International Joint Conference on Artificial Intelligence (IJCAI-24), Survey Track, pp. 8344-8353 (2024); DOI not visible* | Survey and taxonomy of deep-learning point cloud registration: datasets and metrics, supervised methods by pipeline stage (descriptor extraction, correspondence search, outlier filtering, transformation estimation) plus ICP-/probabilistic-optimisation and multimodal methods, and unsupervised correspondence-free and correspondence-based methods. | (pool root) | Ch2 Point cloud registration (taxonomy of learned registration, datasets and metrics). | - |
| 014 | **Deep learning-based framework for Shape Instance Registration on 3D CAD models**<br>Figueiredo et al. (3), 2021. *Computers & Graphics 101, 72-81 (2021), Special Section on SIBGRAPI 2021, doi:10.1016/j.cag.2021.08.012* | Finds repeated component instances inside large engineering CAD models: meshes are sampled into point clouds, classified with PointNet++, and fitted with affine transforms (PCA initialisation + Adam optimisation) under a geometric error bound. Instance identification accuracy 99.98%; a real CAD model reduced to 2.61% of its size. | (pool root) | Probably not needed; at most Ch2 Point cloud registration (PCA + optimiser registration of CAD-sampled point clouds with non-uniform scale). | - |
| 023 | **Rigid pairwise 3D point cloud registration: A survey**<br>Lyu et al. (5), 2024. *Pattern Recognition 151, 110408 (2024), doi:10.1016/j.patcog.2024.110408* | Survey of rigid pairwise point cloud registration with the IBTaxon taxonomy: one-stage end-to-end methods versus two-stage methods using intermediates (correspondences), the latter split into alternating optimisation (ICP, GMM/EM) and sequential learned pipelines; also covers datasets and metrics. | (pool root) | Ch2 Point cloud registration (taxonomy covering ICP, GMM and learned methods). | - |
| 024 | **RoReg: Pairwise Point Cloud Registration With Oriented Descriptors and Local Rotations**<br>Wang et al. (9), 2023. *IEEE Transactions on Pattern Analysis and Machine Intelligence 45(8), 10376-10393 (2023), doi:10.1109/TPAMI.2023.3244951* | Learns a rotation-equivariant oriented descriptor (RoReg-Desc, icosahedral-group features) whose local rotation estimates drive a rotation-guided keypoint detector, a rotation-coherence matcher and a one-shot RANSAC. State of the art on 3DMatch/3DLoMatch and generalises to outdoor ETH. | (pool root) | Ch2 Point cloud registration (learned rotation-equivariant descriptors vs PPF/FPFH). | - |
| 042 | **TEASER: Fast and Certifiable Point Cloud Registration**<br>Yang et al. (3), 2021. *IEEE Transactions on Robotics 37(2), 314-333 (April 2021), doi:10.1109/TRO.2020.3033695* | Certifiable registration with outlier correspondences: a truncated-least-squares cost, graph-theoretic decoupling of scale, rotation and translation through invariant measurements with maximum-clique outlier pruning, adaptive voting for scale and translation and a tight SDP relaxation for rotation; TEASER++ replaces the SDP with graduated non-convexity plus a Douglas-Rachford optimality certifier. Robust to more than 99% outliers when the scale is known, runs in milliseconds and can even register without correspondences, outperforming RANSAC, branch-and-bound and ICP. | realtime scan to cad/ | Ch2 Point cloud registration (robust global registration from FPFH or learned correspondences); Ch3 Perception and registration (global initialisation alternative to PPF before ICP) | - |
| 045 | **Comparison of Point Cloud Registration Techniques on Scanned Physical Objects**<br>Denayer et al. (5), 2024. *Sensors 24, 2142 (2024), doi:10.3390/s24072142* | Benchmarks three classical (Go-ICP, RANSAC, FGR) and three learned (PointNetLK, RPMNet, ROPNet) registration methods for CAD-model alignment on Intel RealSense D435i scans of nine 3D-printed Cranfield-benchmark parts with ground-truth transforms (dataset and code released). Go-ICP is the most accurate but needs seconds, PointNetLK, RANSAC and RPMNet are accurate only with ICP refinement, and all methods except Go-ICP fail often on noisier scans or large initial transformations. | surveys/ | Ch2 Point cloud registration (CAD-to-scan comparison on the same RealSense D435i sensor); Ch3 Perception and registration (stationary refinement (ICP), need for a good initial pose); Ch6 Results/discussion | - |
| 046 | **Deep Learning-Based Point Cloud Registration: A Comprehensive Survey and Taxonomy**<br>Zhang et al. (7), 2026. *International Journal of Computer Vision 134, 168 (2026), doi:10.1007/s11263-025-02723-w* | Survey and taxonomy of deep-learning point cloud registration: pairwise methods split into supervised (by registration stage, optimisation strategy, learning paradigm and integration with traditional methods) and unsupervised (correspondence-based or correspondence-free), and multi-scan methods (multiview and multi-instance), with datasets, metrics and losses. Adds a quantitative comparison of recent methods under one training setup and lists open challenges. | surveys/ | Ch2 Point cloud registration (survey of learned registration) | - |
| 052 | **A comprehensive survey on point cloud registration**<br>Huang et al. (4), 2021 (arXiv). *arXiv:2103.02690v2 [cs.CV] (5 Mar 2021)* | Survey of same-source and cross-source point cloud registration (1992-2021) covering optimisation-based and deep-learning methods and the connections between them, benchmark datasets and applications. Builds a new cross-source benchmark (point clouds from different sensors) and evaluates state-of-the-art algorithms on it. | closely related/katman1/ | Ch2 Point cloud registration (survey; cross-source registration is the setting closest to CAD-to-scan alignment) | - |
| 056 | **Deep Global Registration**<br>Choy et al. (3), 2020 (arXiv). *arXiv:2004.11540v2 [cs.CV] (8 May 2020); no venue printed* | Differentiable pairwise registration of real 3D scans: a 6-dimensional convolutional network predicts the inlier confidence of correspondences, a differentiable weighted Procrustes solver gives a closed-form pose in time linear in the number of correspondences, and a robust gradient-based SE(3) optimiser refines it. Outperforms RANSAC, FGR, DCP and other learned and classical methods on 3DMatch and scene-reconstruction datasets. | closely related/katman1/ | Ch2 Point cloud registration (learned global registration) | - |
| 079 | **Fast Point Feature Histograms (FPFH) for 3D Registration**<br>Rusu et al. (3), 2009. *2009 IEEE International Conference on Robotics and Automation (ICRA), Kobe, Japan, May 12-17, 2009, pp. 3212-3217, doi:10.1109/ROBOT.2009.5152473 (DOI from the PDF metadata)* | Speeds up Point Feature Histograms by caching and reordering, and reformulates them into Fast Point Feature Histograms (FPFH) with O(k) cost and online computation, keeping most of the PFH's discriminative power. Proposes SAC-IA (sample consensus initial alignment) to bring overlapping point-cloud views into the convergence basin of a local non-linear registration such as ICP; tested on noisy scans. | non DL scan to cad/ | Ch2 Point cloud registration (local descriptors and global initial alignment); Ch3 Perception and registration (coarse alignment before ICP refinement) | - |
| 160 | **MRG: A Multi-Robot Manufacturing Digital Scene Generation Method Using Multi-Instance Point Cloud Registration**<br>Han et al. (5), 2025. *arXiv:2501.02041v1 [cs.CV] (3 Jan 2025)* | Multi-instance point-cloud registration (instance-focused transformer, hypothesis generation, screening/optimisation) to build digital scenes of multi-robot manufacturing cells; better MR/MP than prior multi-instance methods on Scan2CAD and a Welding-Station dataset. | For Welding Dataset/non fem/DL but not related/ | Ch2 Point cloud registration (multi-instance, learned); Ch2 Scan-to-CAD alignment | - |

Notes:

- **006**: NeurIPS camera-ready format (19 pages incl. appendix), no arXiv stamp; PDF metadata empty.
- **014**: Registers CAD meshes to CAD meshes for model compression, not scans to CAD. PDF has no title metadata.
- **024**: IEEE Xplore download copy with a METU licence stamp on each page.
- **042**: IEEE Xplore final version (METU download stamp, 6 Jul 2026); received 23 Jan 2020, published online 8 Dec 2020.
- **045**: File name has a leading and a trailing space (' Comparison ... Objects .pdf').
- **046**: Extended journal version of tag 007 (Zhang et al., 'A Comprehensive Survey and Taxonomy on Point Cloud Registration Based on Deep Learning', IJCAI-24 survey track, pp. 8344-8353); the IJCV paper states that it extends that work. Cite this IJCV version, or both deliberately. Published online 7 Mar 2026.
- **052**: Not the same paper as tag 007: different authors (Huang, Mei, Zhang, Abbas; University of Sydney/UTS), a 2021 arXiv preprint covering optimisation and deep-learning methods plus cross-source registration, whereas tag 007 is Zhang et al.'s deep-learning-only IJCAI-24 survey (extended as tag 046). No journal version is printed.
- **056**: arXiv preprint; the published version is probably CVPR 2020 (inferred, not printed; verify).
- **079**: IEEE Xplore copy (METU download stamp). DOI only in the PDF metadata, not printed on the page.
- **160**: Preprint; '_compressed' file.

### G. Segmentation (promptable image segmentation; point-cloud segmentation) and edge / boundary / contour detection (Ch2 background; Ch3 segmentation)

| Tag | Paper | Gist | Folder | Supports | bib |
|---|---|---|---|---|---|
| 048 | **An approach to boundary detection for 3D point clouds based on DBSCAN clustering**<br>Chen et al. (5), 2022. *Pattern Recognition 124, 108431 (2022), doi:10.1016/j.patcog.2021.108431* | Revised DBSCAN for plane segmentation and boundary detection: candidate samples chosen by density and normals give a validated three-point fitting plane, and an automatically adapted coplanar threshold is added as a clustering condition so that each cluster holds one plane and the intersection boundaries between planes are separated. Tested on synthetic and real point clouds. | closely related/boundary detection/ | Ch2 Weld seam detection from 3D data (plane segmentation and plane-intersection boundaries as seam candidates); Ch4 Proposed seam extractors | - |
| 111 | **EDC-Net: Edge Detection Capsule Network for 3D Point Clouds**<br>Bazazian and Pares (2), 2021. *Applied Sciences 11(4), 1833 (2021), doi:10.3390/app11041833* | Capsule-network classifier that labels points as edge / non-edge (edge detection posed as semantic segmentation), weakly supervised, evaluated on ABC and ShapeNet. | pointcloud trajectory extraction/ | Ch2 Weld seam detection from 3D data (learned edge detection as a seam-candidate step); Ch4 Evaluation protocol (edge-based baselines) | - |
| 112 | **JSENet: Joint Semantic Segmentation and Edge Detection Network for 3D Point Clouds**<br>Hu et al. (5), 2020. *arXiv:2007.06888v1 [cs.CV] (14 Jul 2020); ECCV 2020 (inferred from the LNCS format, not printed)* | Two-stream fully-convolutional network that jointly performs 3D semantic segmentation and semantic edge detection with a joint refinement module and a boundary-aware loss; evaluated on S3DIS and ScanNet. | pointcloud trajectory extraction/ | Ch2 Weld seam detection from 3D data (boundary/edge learning); probably not needed | - |
| 114 | **Point Cloud Segmentation of 3D Scattered Parts Sampled by RealSense**<br>Gong et al. (3), 2017. *Proceedings of the 2017 IEEE International Conference on Information and Automation (ICIA), Macau, July 2017 (pages not visible)* | Filters a RealSense point cloud, over-segments it into small patches and merges them by an extended convexity criterion and an angle criterion to separate scattered parts for bin picking. | background segmentation/ | Ch2 segmentation background (geometric segmentation of RealSense clouds); probably not needed | - |
| 163 | **Fast and Robust Edge Extraction in Unorganized Point Clouds**<br>Bazazian et al. (3), 2015. *2015 International Conference on Digital Image Computing: Techniques and Applications (DICTA), doi:10.1109/DICTA.2015.7371262* | Detects sharp edges in unorganised point clouds from the eigenvalues of each point's k-NN covariance matrix (surface variation), evaluated on toy dihedral angles and noisy real clouds. | reading/ | Ch2 Weld seam detection from 3D data (curvature/PCA edge features); Ch4 Evaluation protocol (PCA-based baselines) | - |
| 164 | **Contour Detection in Unstructured 3D Point Clouds**<br>Hackel et al. (3), 2016. *IEEE Conference on Computer Vision and Pattern Recognition (CVPR) 2016 (CVF open-access version; page numbers not printed)* | Two-stage discriminative learning: a per-point contour score from neighbourhood features, then selection of an optimal set of candidate contours in a higher-order MRF favouring connected contours; > 10^7 points in minutes. | reading/ | Ch2 Weld seam detection from 3D data (contour/crease detection) | - |
| 165 | **Novel algorithms for 3D surface point cloud boundary detection and edge reconstruction**<br>Mineo et al. (3), 2019. *Journal of Computational Design and Engineering 6:81-91 (2019), doi:10.1016/j.jcde.2018.02.001* | Threshold-free boundary-point detection and spatial-FFT-based edge reconstruction for low-noise tessellated surfaces from laser scans (turbine blade). | reading/ | Ch2 Weld seam detection (boundary detection); probably minor | - |
| 166 | **Contrastive Boundary Learning for Point Cloud Segmentation**<br>Tang et al. (5), 2022. *IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR) 2022 (CVF open-access version; page numbers not printed)* | Contrastive boundary learning (CBL) sharpens features across scene boundaries at multiple scales; improves three point-cloud segmentation baselines on boundary metrics and mIoU. | reading/ | Ch2 segmentation background; probably not needed | - |
| 172 | **SAM 2: Segment Anything in Images and Videos**<br>Ravi et al. (18), 2024. *arXiv:2408.00714v2 [cs.CV] (28 Oct 2024); results are those of the 'SAM 2.1' checkpoints; later published at ICLR 2025 (not on this PDF)* | Promptable visual segmentation in images and videos: Hiera image encoder, memory attention over a FIFO memory bank of recent and prompted frames plus object pointers, SAM-style prompt encoder and two-way mask decoder with occlusion head; trained on SA-1B + the new SA-V dataset (50.9K videos, 642.6K masklets) from a 3-phase data engine. | supply_papers/ | Ch2 Object recognition / segmentation background; Ch3 Segmentation and part classification (point prompts) | `ravi2024sam2` |

Notes:

- **112**: Venue to verify (ECCV 2020, LNCS 12365?).
- **114**: PDF metadata title is 'IEEE Conference Paper Template'.
- **172**: Digest in Part 2. Bib author list is 'Ravi, Nikhila and others' (18 authors on the PDF).

### H. Scan-to-CAD alignment and CAD-based workpiece localisation (Ch2 pose/registration; Ch3 seams from the registered assembly)

| Tag | Paper | Gist | Folder | Supports | bib |
|---|---|---|---|---|---|
| 010 | **A novel autonomous guidance method for welding robot based on point cloud registration and 3D vision**<br>Xiao et al. (7), 2026. *The International Journal of Advanced Manufacturing Technology 143, 6491-6507 (2026), doi:10.1007/s00170-026-17415-z* | Large-field-of-view (LFoV) 3D camera with its own hand-eye calibration; YOLOv5 recognises welding workpieces in the scene image and a coarse-to-fine registration (Super 4PCS, then Anderson-accelerated ICP) aligns the workpiece model to locate it for welding guidance. Average positioning error 0.62 mm and angular error 0.43 degrees. | (pool root) | Ch2 Weld seam detection from 3D data (model-based workpiece localisation for welding guidance, close to the thesis approach); Ch2 Point cloud registration; Ch2 Camera calibration for eye-in-hand systems; Ch3 Perception and registration (detector + coarse-to-fine registration); Ch6 Results/discussion (0.62 mm / 0.43 degree accuracy). | - |
| 020 | **Precise pose and assembly detection of generic tubular joints based on partial scan data**<br>Tan et al. (5), 2022. *Neural Computing and Applications 34, 5201-5211 (2022), doi:10.1007/s00521-021-06246-6* | Finds the pose and fit-up of brace and chord of tubular T/K/Y joints from partial, sparse scans by minimising the area of a boundary enclosing the scan points relative to ideal reference models, also for non-cylindrical cross-sections. On a robotic welding system the RMSE is below 1% of the pipe diameter (diameters above 200 mm). | (pool root) | Ch2 Object recognition and 6D pose estimation (model-based localisation of featureless welding workpieces from partial scans); Ch3 Seams from the registered assembly (fit-up from per-part poses); Ch3 Perception and registration (symmetric and cut parts: featureless tubes). | - |
| 021 | **Remote dismantling system using a digital manufacturing system and workpiece localization for nuclear facility decommissioning**<br>Hyun et al. (5), 2024. *Annals of Nuclear Energy 195, 110182 (2024), doi:10.1016/j.anucene.2023.110182* | Nuclear-decommissioning cutting cell (Staubli TX200 + hydraulic arm) whose robot task sequences are planned in a calibrated digital model and corrected for the real workpiece by ICP registration of 3D-scanner point clouds. Over six imposed pose variations (50 mm offset, +/-5 and +/-10 degrees) the ICP transforms matched within 2%; underwater laser and saw cutting were demonstrated. | (pool root) | Ch2 Object recognition and 6D pose estimation (ICP workpiece localisation correcting model-planned robot paths, non-welding domain); Ch3 Seams from the registered assembly (paths defined on the model and moved with the registered pose). | - |
| 026 | **SceneCAD: Predicting Object Alignments and Layouts in RGB-D Scans**<br>Avetisyan et al. (6), 2020. *Computer Vision - ECCV 2020, Proceedings Part XXII, LNCS 12367, pp. 596-612, Springer (2020), doi:10.1007/978-3-030-58542-6_36* | Aligns CAD models to RGB-D scans via dense scan-CAD correspondences while predicting layout planes hierarchically (corners, edges, planes), with a message-passing graph network over object-object and object-layout relations for globally consistent alignment. CAD alignment accuracy rises from 41.83% to 58.41% on SUNCG and from 50.05% to 61.24% on ScanNet (Scan2CAD). | (pool root) | Ch2 Object recognition and 6D pose estimation (scan-to-CAD alignment line of work); Ch3 Perception and registration (relations between objects for globally consistent multi-part alignment). | - |
| 028 | **Simulated annealing-based fitting of CAD models to point clouds of mechanical parts’ assemblies**<br>Shah et al. (5), 2020. *Engineering with Computers 37(4), 2891-2909 (2020 as given in the HAL citation), doi:10.1007/s00366-020-00970-8; HAL hal-03814968* | Fits parameterised CAD part/assembly models to scanned point clouds: simulated annealing optimises dimensional and assembly-constraint parameters through batch calls to a CAD modeller while ICP sets each part's position and orientation. Evaluated on real scans and on virtual as-scanned clouds with scanner artefacts, from single parts to a full digital mock-up. | (pool root) | Ch2 Object recognition and 6D pose estimation (fitting CAD assemblies to scans with ICP and assembly constraints); Ch3 Seams from the registered assembly (assembly constraints between parts); Ch4 Sensor model and rendering (virtual as-scanned point clouds with scanner artefacts). | - |
| 038 | **FastCAD: Real-Time CAD Retrieval and Alignment from Scans and Videos**<br>Langer et al. (5), 2024 (arXiv). *arXiv:2403.15161v1 [cs.CV] (22 Mar 2024)* | Single-stage network that predicts, for every object in an RGB-D scan, an oriented 3D box with its front-facing side plus a shape embedding distilled from a contrastively trained scan/CAD encoder, then retrieves and places the nearest CAD model in 50 ms per scan. Matches the best Scan2CAD alignment accuracy (61.7% against 61.2%) about 50 times faster, and on video reconstructions at 10 FPS raises accuracy from 43.0% to 48.2%. | realtime scan to cad/ | Ch2 Object recognition and 6D pose estimation (CAD retrieval and alignment from scans; indoor furniture scale, mainly for contrast) | - |
| 041 | **Scan2CAD: Learning CAD Model Alignment in RGB-D Scans**<br>Avetisyan et al. (6), 2019 (inferred). *IEEE Conference on Computer Vision and Pattern Recognition (CVPR), pp. 2614-2623 (CVPR from the PDF metadata; year from the CVF file name of the identical copy, tag 162)*<br>Copies: 162: reading/Avetisyan_Scan2CAD_Learning_CAD_Model_Alignment_in_RGB-D_Scans_CVPR_2019_paper.pdf (byte-identical) | Introduces the Scan2CAD dataset (97,607 keypoint correspondences and 9-DoF alignments between 14,225 object instances of 3,049 ShapeNet models and 1,506 ScanNet scans) and a 3D CNN that predicts scan-to-CAD correspondence heatmaps, followed by a variational energy minimisation for 9-DoF CAD alignment. Outperforms handcrafted descriptors (FPFH, SHOT, PPF) and CNN baselines by 21.39% on its benchmark. | realtime scan to cad/ | Ch2 Object recognition and 6D pose estimation (scan-to-CAD alignment); Ch2 Point cloud registration (handcrafted descriptors fail across the clean-CAD/noisy-scan gap) | - |
| 058 | **A 3D Vision Framework for Generalized Workpiece Localization in Robotic Manufacturing**<br>Hernandez Villa (1), 2024. *MSc thesis (Master of Science), Department of Mechanical Engineering, Faculty of Graduate Studies, University of Manitoba, Winnipeg, July 2024; 91 pp.* | CAD-to-workpiece localisation as partial-to-partial registration with an eye-in-hand Zivid Two structured-light camera on an ABB IRB 1300: SAM-based workpiece-agnostic segmentation, DINO-keypoint image-guided coarse alignment, a learned registration network with global feature embedding and multi-head cross-attention, and a new gradient-based fine registration. Tested on a real composite-part dataset, a rule-based synthetic industrial-object dataset with point cloud corruptions and ModelNet40; the optimisation-based method reaches a 99.8% success rate. | closely related/katman1/ | Ch2 Point cloud registration; Ch2 Object recognition and 6D pose estimation; Ch3 Perception and registration (foundation-model segmentation plus CAD registration with an eye-in-hand camera); Ch4 Sensor model and rendering (rule-based synthetic data with corruptions) | - |
| 083 | **Workpiece localization methods for robotic welding – a review**<br>Schleth et al. (3), 2018. *ISR 2018 (International Symposium on Robotics), June 20-21, 2018, Munich, Germany, pp. 50-55, VDE Verlag, ISBN 978-3-8007-4699-6 (via IEEE Xplore; DOI not printed)* | Reviews how robotic welding cells localise the workpiece (contact and non-contact sensing, low- and high-level features, matching and alignment against the CAD model), compares this with bin-picking methods and with commercial solutions seen at a welding trade fair. Argues for feature-based coarse localisation with offline training and global optimisation that needs no prior pose knowledge, to cut set-up times for SMEs. | superficial/ | Ch2 Object recognition and 6D pose estimation (workpiece localisation for welding); Ch2 Weld seam detection from 3D data (seam vs whole-workpiece localisation); Ch2 Summary and research gap; Ch3 Perception and registration | - |
| 116 | **Automated Planning of Robotic MAG Welding Based on Adaptive Gap Model**<br>Kuss et al. (4), 2017. *Procedia CIRP 62:612-617 (2017), 10th CIRP Conference on Intelligent Computation in Manufacturing Engineering (CIRP ICME '16), doi:10.1016/j.procir.2016.07.008* | Matches the CAD model to a stereo-camera point cloud of the welding assembly with ICP to compute the actual gap geometry, then adapts MAG welding parameters with a gap model; validated on an industrial robot. | not so related/ | Ch2 Welded joints, tack welds and standards (fit-up gap); Ch3 Fit-up gap and weldability; Ch2 Summary and research gap (CAD-to-scan for welding) | - |

Notes:

- **010**: PDF has no title metadata.
- **020**: Published online 26 June 2021; issue year 2022.
- **026**: Whole proceedings volume, not the paper: the PDF is ECCV 2020 Proceedings Part XXII (LNCS 12367, 826 pages, 183 MB; metadata title '504482_1_En_Print.indd'). SceneCAD is at PDF pages 637-653 (printed pp. 596-612); recommend extracting those 17 pages into a separate file and dropping the volume.
- **028**: HAL deposited author version (HAL cover page), not the publisher PDF. The HAL citation gives 2020 together with vol. 37(4); the 37(4) print issue is probably 2021 (inferred), check before citing.
- **038**: arXiv preprint (Qualcomm internship work); the file name drops the subtitle 'from Scans and Videos'.
- **041**: Byte-identical duplicate of tag 162 (reading/Avetisyan_Scan2CAD_Learning_CAD_Model_Alignment_in_RGB-D_Scans_CVPR_2019_paper.pdf; same MD5 beb47da60d81316d9c0d76bddaaae7cf). Year and venue are not printed in the text layer.
- **058**: MSc thesis, not a peer-reviewed paper (supervisor Dr. Matt Khoshdarregi, Intelligent Digital Manufacturing Lab). File name is the author's name; the first2 text covers only the title page and part of the abstract.
- **083**: Fraunhofer IPA; ROBOTT-NET (H2020) funding. Notes manual tack welding as a cause of deviation from nominal geometry. DOI not printed.
- **116**: Despite the folder name 'not so related', this is close to the cell's 'seams from registered CAD + fit-up gap' idea; worth citing.

### I. Vision for robot guidance, calibration and multi-view positioning (Ch2 camera calibration; Ch3 frames and calibration, multi-view refinement)

| Tag | Paper | Gist | Folder | Supports | bib |
|---|---|---|---|---|---|
| 005 | **Robot Guidance Using Machine Vision Techniques in Industrial Environments: A Comparative Review**<br>Pérez et al. (5), 2016. *Sensors 16, 335 (2016), doi:10.3390/s16030335* | Comparative review of machine-vision techniques for industrial robot guidance (photogrammetry, stereo vision, structured light, time of flight, laser triangulation), comparing accuracy, range, sensor weight, safety, processing time and environmental influences. Notes that industrial robots repeat to about 0.1 mm but have absolute errors of several mm. | (pool root) | Ch2 Camera calibration for eye-in-hand systems (eye-in-hand vs fixed cameras; robot absolute accuracy of several mm vs 0.1 mm repeatability); Ch3 Overview (choice of 3D sensing technique). | - |
| 030 | **Vision-based multi-view reconstruction for high-precision part positioning in industrial robot machining**<br>Wu et al. (6), 2025. *Measurement 242, 116042 (2025), doi:10.1016/j.measurement.2024.116042* | Multiple cameras perform passive multi-view stereo reconstruction of an engine hood, and the resulting point cloud is matched to the part's CAD model so that offline-programmed machining trajectories can be transferred to the real part. 85.7% of points within 2 mm, overall RMSE 1.09 mm, process within 5 s. | (pool root) | Ch3 Multi-view close-range refinement (multi-view reconstruction matched to CAD for part positioning); Ch2 Camera calibration for eye-in-hand systems; Ch6 Results/discussion (1.09 mm RMSE). | - |

Notes:

- **005**: File name starts with a space and has a space before '.pdf' (quote the path when scripting).
- **030**: Abstract gives overall RMSE 1.09 mm, while the contribution list says RMSE under 1 mm (internal inconsistency).

Cross-references: The pool has no dedicated hand-eye or depth-calibration paper (see Part 4). Related material: 005 (robot guidance review: industrial robots repeat to ~0.1 mm but have several-mm absolute error), 010 (LFoV camera with its own hand-eye calibration), 132 (eye-in-hand calibration + ICP stitching), 147 (eye-to-hand + eye-in-hand dual-camera planning), 082 (laser-stripe sensor hand-eye), 161 (depth-perception review), 051 / 054 (eye-in-hand multi-view fusion).

### J. Motion planning, collision checking and trajectory generation (Ch2 motion planning; Ch3 motion planning and execution)

| Tag | Paper | Gist | Folder | Supports | bib |
|---|---|---|---|---|---|
| 003 | **Computer Vision-Based Path Planning for Robot Arms in Three-Dimensional Workspaces Using Q-Learning and Neural Networks**<br>Abdi et al. (3), 2022. *Sensors 22, 1697 (2022), doi:10.3390/s22051697* | Two cameras with a YOLO-plus-neural-network detector give 3D coordinates of start, target and obstacle objects; Q-learning finds a sequence of six discrete moves in a 3D grid and a trained network maps the moves to the joint angles of a six-joint arm. Extends the authors' earlier 2D-grid method; tested in simulation and on a real arm. | (pool root) | Ch2 Motion planning and trajectory generation (learned grid planner as a contrast to sampling-based OMPL planners); probably minor. | - |
| 025 | **Safe and Automated Assembly Process using Vision assisted Robot Manipulator**<br>Ahmad and Plapper (2), 2016. *Procedia CIRP 41, 771-776 (2016), 48th CIRP Conference on Manufacturing Systems (CIRP CMS 2015), doi:10.1016/j.procir.2015.12.129* | A low-resolution (56x61 px) 3D-MLI time-of-flight sensor localises unknown, unprogrammed static obstacles in a KUKA-DLR lightweight-robot peg-in-hole cell, and a decision step picks the best detour trajectory, generated in under one second. Dynamic obstacles are left to future work. | (pool root) | Ch2 Motion planning and trajectory generation (depth-sensor-based avoidance of unknown obstacles); Ch3 Motion planning and execution (collision model from depth data); minor. | - |
| 029 | **Trajectory generation for robotic assembly operations using learning by demonstration**<br>Duque et al. (3), 2019. *Robotics and Computer-Integrated Manufacturing 57, 292-302 (2019), doi:10.1016/j.rcim.2018.12.007* | Human assembly demonstrations captured with a first-generation Kinect train task-parameterised GMMs (TP-GMM/GMR) that generate new trajectories from detected object positions, while Petri nets produce the assembly plan from the detected parts. Simulated in RobotStudio and executed on an ABB IRB 140 for simple peg-in-hole (toy car) assembly. | (pool root) | Ch2 Motion planning and trajectory generation (learning from demonstration as an alternative to explicit planning); probably minor. | - |
| 065 | **Robotic Tack Welding Path and Trajectory Optimization Using an LF-IWOA**<br>Jia et al. (6), 2025. *Actuators 14, 287 (2025), doi:10.3390/act14060287* | A Lévy-flight improved whale optimisation algorithm (elite opposition-based learning, differential evolution and Lévy flight) orders the tack-weld points of a workpiece, and a dynamic trajectory model with joint velocity, acceleration and jerk limits times the motion. On two industrial workpieces the path is up to 25.53% shorter than with GA (14.82% on average; 18.41% for workpiece 2), with 26.83% less execution time and 15.40% less mechanical energy. | to the tack weld sampling/ | Ch3 Tack placement and ordering (TSP-like tack sequencing); Ch3 Motion planning and execution; Ch2 Motion planning and trajectory generation | - |
| 105 | **A novel offline programming approach of robot welding for multi-pipe intersection structures based on NSGA-II and measured 3D point-clouds**<br>Liu et al. (4), 2023. *Robotics and Computer-Integrated Manufacturing 83, 102549 (2023), doi:10.1016/j.rcim.2023.102549* | Offline programming for multi-pipe intersection welds from the measured seam point cloud: feature points, cubic NURBS fit with adaptive nodes, chord-error discretisation, and NSGA-II optimisation of joint trajectories for weld quality and cable-twisting fluctuation. | Robot control optimal attack/ | Ch2 Motion planning and trajectory generation (robot-in-the-loop weld planning); Ch3 Reachability (cable window / roll DP); Ch4 Curved seams by construction (pipe intersections) | `liu2023rcim` |
| 106 | **Object-centric Task Representation and Transfer using Diffused Orientation Fields**<br>Bilaloglu et al. (3), 2025. *arXiv:2511.18563v1 [cs.RO] (23 Nov 2025)* | Computes smooth local reference frames on curved objects from raw point clouds by PDE-based diffusion conditioned on keypoints, so surface-contact tasks (inspection, slicing, peeling) transfer across shapes via sparse keypoint correspondences. | Robot control optimal attack/ | Ch3 Reachability / tool orientation along curved seams (related idea); Ch7 Future work; probably not needed | - |
| 110 | **Collision-Affording Point Trees: SIMD-Amenable Nearest Neighbors for Fast Collision Checking**<br>Ramsey et al. (4), 2024. *arXiv:2406.02807v1 [cs.RO] (4 Jun 2024); Robotics: Science and Systems (RSS) 2024 (inferred, not on PDF)* | CAPT, an exact SIMD-friendly spatial structure for robot-vs-point-cloud collision queries (<10 ns average per query), letting sampling-based planners find paths in under a millisecond; plus a space-filling-curve point-cloud filter. | pointcloud trajectory extraction/ | Ch3 Collision model (C++ vs Python collision model, point-cloud obstacles); Ch2 Motion planning | - |
| 171 | **RRT-Connect: An Efficient Approach to Single-Query Path Planning**<br>Kuffner and LaValle (2), 2000. *DRAFT submitted to ICRA 2000 (header of this copy); published in Proc. IEEE ICRA 2000, San Francisco, vol. 2, pp. 995-1001, doi:10.1109/ROBOT.2000.844730 (vol./DOI online)* | Two RRTs grown from start and goal; each iteration extends one tree towards a random sample and greedily CONNECTs the other tree to the new vertex, then swaps; proved probabilistically complete; seconds-scale planning for a 7-DOF arm and a 6-DOF PUMA. | supply_papers/ | Ch2 Motion planning and trajectory generation; Ch3 Transit planning | `kuffner2000rrtconnect` |
| 173 | **Anytime Solution Optimization for Sampling-Based Motion Planning**<br>Luna et al. (4), 2013. *'To appear in Proc. 2013 IEEE Intl. Conf. on Robotics and Automation' on this copy; ICRA 2013, Karlsruhe, pp. 5053-5059, doi:10.1109/ICRA.2013.6631301 (online, Kavraki lab)* | Anytime Path Shortening (APS): a meta-algorithm that repeatedly runs any sampling-based planner within a time budget and alternates shortcutting of the best path with hybridisation of the best m paths; in OMPL; competitive with or better than RRT\* on high-DOF problems (PR2 arm). | supply_papers/ | Ch2 Motion planning and trajectory generation; Ch3 Transit planning (OMPL AnytimePathShortening); Ch6 APS vs RRT-Connect | - |
| 182 | **RRT\*-Connect: Faster, Asymptotically Optimal Motion Planning**<br>Klemm et al. (7), 2015. *2015 IEEE International Conference on Robotics and Biomimetics (ROBIO), Zhuhai, China, pp. 1670-1677, doi:10.1109/ROBIO.2015.7419012* | Bidirectional RRT\* that combines RRT-Connect's two-tree connection with RRT\*'s rewiring, finding first solutions faster than RRT\* while converging asymptotically to the optimum; car parking, legged-robot and mobile-manipulation demos. | weld_generator/papers/ (only there) | Ch2 Motion planning and trajectory generation (asymptotic optimality) | `klemm2015rrtstarconnect` |

Notes:

- **003**: File name starts with a space and has a space before '.pdf' (quote the path when scripting). Extension of the authors' earlier 2D Q-learning path-planning paper.
- **025**: Conference held 2015 (CIRP CMS 2015); proceedings volume dated 2016.
- **065**: The first two authors contributed equally. Orders tacks by path length and time, not by distortion.
- **105**: This IS the paper the placeholder entry liu2023rcim refers to: authors Yan Liu, Qiu Tang, Xincheng Tian, Shuai Yang; the bib title 'VERIFY: NSGA-II offline programming ...' and the surname-only author list must be replaced (corrected entry in Part 3).
- **106**: Preprint (not peer reviewed at time of download).
- **110**: Folder name 'pointcloud trajectory extraction' does not match the content (collision checking). Venue to verify (RSS 2024).
- **171**: Text layer is broken (Type-3 fonts); read from rendered page images. This is the pre-publication draft, not the proceedings version. Digest in Part 2.
- **173**: NOT in thesis.bib under the right key: the entry luo2014aps (Luo & Hauser, IROS 2014) is a different paper. Digest in Part 2; corrected entry in Part 3.
- **182**: Only in weld_generator/papers/ (not in paper_pool or supply_papers). The file name 'RRT-Connect_...' is misleading: it is RRT\*-Connect, not Kuffner & LaValle. Bib entry matches the PDF; add DOI.

### K. Human-robot interaction and gestures (Ch3 operator interaction)

| Tag | Paper | Gist | Folder | Supports | bib |
|---|---|---|---|---|---|
| 168 | **MediaPipe: A Framework for Building Perception Pipelines**<br>Lugaresi et al. (14), 2019. *arXiv:1906.08172v1 [cs.DC] (14 Jun 2019)* | Describes the MediaPipe framework only: perception pipelines as graphs of calculators connected by timestamped packet streams, side packets, GraphConfig/subgraphs, scheduler with input policies and flow control, GPU (OpenGL ES/Metal) support, tracer and visualiser; examples are object detection+tracking and face landmarks+segmentation. | supply_papers/ | Ch3 Gesture interface (framework); hand-landmark/gesture models need MediaPipe Hands (not in folders) | `lugaresi2019mediapipe` |

Notes:

- **168**: Framework paper only - does not describe the hand landmark model or the gesture recogniser. Digest in Part 2.

Cross-references: Other gesture / HRI-related items: 123 (Zhou et al.: hand-gesture-guided scanning of a welding workpiece - the closest precedent for gesture control of a welding robot), 025 (vision-based safe assembly). The hand-landmark and gesture models themselves (MediaPipe Hands, Gesture Recognizer) are NOT in the folders (Part 2.3, Part 4).

### L. Welding distortion, tack welds and sequencing, FEM / eigenstrain, fixtures (Ch2 welded joints and tacks; Ch3 tack ordering; Ch7 future work)

| Tag | Paper | Gist | Folder | Supports | bib |
|---|---|---|---|---|---|
| 022 | **Review on Mitigation of Welding-Induced Distortion Based on FEM Analysis**<br>Wu and Kim (2), 2020. *Journal of Welding and Joining 38(1), 56-66 (2020), doi:10.5781/JWJ.2020.38.1.6* | Literature review of FEM-based prediction (thermal elastic-plastic, inherent-strain elastic) and mitigation of welding distortion: design-stage measures (groove type, pre-heating, tack welding), manufacturing-stage measures (welding parameters, sequence optimisation with GA/ANN, mechanical constraints, thermal balance) and final correction. | (pool root) | Ch2 Welded joints, tack welds and standards (tack welding and welding sequence as distortion control); Ch7 Future work (distortion-aware tack ordering). | - |
| 060 | **An investigation into the influence of inter-component assembly gaps upon the geometry of tack welded annular structures**<br>Lowth et al. (3), 2015. *CIRP Annals - Manufacturing Technology 64, 1-4 (2015), doi:10.1016/j.cirp.2015.04.043* | Studies tack welding (the 'preliminary welding process') of annular assemblies through a geometric study of segmented rings, two-plate Ti-6Al-4V trials with 0, 0.5 and 1.0 mm gaps (autogenous GTAW tacks, DIC measurement) and a fixtured six-component ring, all supported by FEA. Gaps produce 'quantised saddle' and 'quantised conic' distortions driven by volumetric weld shrinkage, and placing the initial tacks to avoid gaps gives a 19× reduction in out-of-plane distortion. | to the tack weld sampling/ | Ch2 Welded joints, tack welds and standards; Ch3 Seams from the registered assembly (fit-up gap); Ch3 Tack placement and ordering; Ch7 Future work (distortion-aware tack ordering) | - |
| 061 | **Combinatorial optimization of weld sequence by using a surrogate model to mitigate a weld distortion**<br>Asadi and Goldak (2), 2011. *International Journal of Mechanics and Materials in Design 7, 123-139 (2011), doi:10.1007/s10999-011-9154-6* | Builds a surrogate model from a few transient FEM runs to search weld sub-pass sequences for minimum distortion instead of simulating every combination (2^n × n! in general). For a pipe girth weld tack-welded at three points with six sub-passes, analysing 14 of the 48 possible combinations found the minimum-distortion sequence, checked against full FEM of all combinations. | to the tack weld sampling/ | Ch7 Future work (distortion-aware sequencing with surrogate models); Ch3 Tack placement and ordering | - |
| 062 | **Development of Hybrid Artificial Neural Network–Particle Swarm Optimization Model and Comparison of Genetic and Particle Swarm Algorithms for Optimization of Machining Fixture Layout**<br>Ramesh et al. (4), 2022. *International Journal of Precision Engineering and Manufacturing 23, 1411-1430 (2022), doi:10.1007/s12541-022-00698-z* | Fixture layout optimisation for a 2D workpiece in end milling: a Taguchi design with ANSYS FEM narrows the locator and clamp ranges, an ANN whose weights are tuned by PSO (trained on central-composite and Box-Behnken samples) predicts the maximum workpiece deformation, and GA and PSO then search the layout. PSO finds layouts 0.1936% better than GA. | to the tack weld sampling/ | Probably not needed (machining fixture layout, not welding); at most Ch7 Future work (fixture design) | - |
| 063 | **Influence of Tack Welds Distribution and Welding Sequence on the Angular Distortion of TIG Welded Joint**<br>Tomków et al. (3), 2020. *Facta Universitatis, Series: Mechanical Engineering 18(4), 611-621 (2020), doi:10.22190/FUME200520044T* | Experiments on 2.5 mm X2CrTiNb18 (AISI 441) stainless-steel TIG fillet joints with seven groups of tack-weld distributions and welding sequences at 80-160 A, with angular distortion measured by CMM and welds checked on macro sections. More tacks gave more angular distortion (lowest 1.03-1.14 degrees with two tacks and the progressive technique) and higher current reduced distortion but caused lack of penetration at 160 A, so the authors recommend tacks outside the groove or only two tacks. | to the tack weld sampling/ | Ch2 Welded joints, tack welds and standards; Ch3 Tack placement and ordering; Ch7 Future work (distortion-aware tack ordering) | - |
| 064 | **Influence of Tacking Sequence on Residual Stress and Distortion of Single Sided Fillet Submerged Arc Welded Joint**<br>Mondal et al. (3), 2015. *Journal of Marine Science and Application 14, 250-260 (2015), Article ID 1671-9433(2015)03-0250-11, doi:10.1007/s11804-015-1320-z* | 3D transient thermo-elasto-plastic FE model (ANSYS, Gaussian moving heat source, element birth and death) of a single-pass, single-sided SAW fillet joint under four tacking sequences, calibrated against published data and validated with in-house angular-distortion measurements. The tacking sequence strongly changes residual stress and deformation, and tack welds placed opposite the fillet-weld side minimise residual stress. | to the tack weld sampling/ | Ch2 Welded joints, tack welds and standards; Ch3 Tack placement and ordering; Ch7 Future work (distortion-aware tack ordering) | - |
| 066 | **Analytical & Heuristic Methods for Ordering Tack Welds to Minimize Distortion: A Route-by-Route Investigation**<br>n/a (0), 2026 (inferred). *not visible (Claude.ai research report saved to PDF from Firefox 153 on 2026-07-31; no venue)* | AI-generated literature synthesis on ordering tack welds to minimise distortion, organised in seven routes (inherent strain/deformation, empirical shrinkage formulas, Tsai's joint-rigidity method, Rosenthal heat-accumulation superposition, classic named sequences, optimisation studies, German standards and groups). Recommends a cost function that combines a joint-rigidity term with a heat-accumulation penalty scaled by shrinkage-force magnitudes, avoiding thermo-elasto-plastic FEA. | to the tack weld sampling/deep search result/ | Not citable; use only as a pointer to primary sources for Ch7 Future work (distortion-aware tack ordering) and Ch3 Tack placement and ordering | - |
| 067 | **Tack Weld Sequence Optimization (2020–2026): A Literature-Backed, Implementable Cost Function J(π) for a ROS 2 / UR5e Welding Cell**<br>n/a (0), 2026 (inferred). *not visible (Claude.ai research report saved to PDF from Firefox 153 on 2026-08-01; no venue)* | AI-generated review claiming that ordering 5-15 tacks on a vision-extracted seam with a cheap cost function is unpublished, and proposing J(π) as a weighted sum of a joint-rigidity/inherent-deformation term, a Rosenthal heat-accumulation penalty (from WAAM sequence planning), shrinkage-moment balance, root-gap evolution and robot travel time. Suggests exhaustive search for n up to 10, metaheuristics beyond that and ICP-to-CAD residuals as the distortion measure, and lists related papers, standards (EN ISO 13920, ISO 5817) and research groups. | to the tack weld sampling/deep search result/ | Not citable; pointer list for Ch7 Future work (distortion-aware tack ordering), Ch3 Tack placement and ordering and Ch2 Welded joints, tack welds and standards | - |
| 069 | **Welding sequence optimization using the strain direct boundary method based on the welding induced change in structural stiffness**<br>Noh et al. (4), 2022. *Journal of Mechanical Science and Technology 36(12), 6193-6199 (2022), doi:10.1007/s12206-022-1133-3* | Determines a distortion-minimising welding sequence: tack welds and gaps are modelled as kinematic constraints in shell FE models, the strain-directed-as-boundary (SDB, inherent-strain) method with elastoplastic properties gives residual strains, and the joint rigidity method compares joint stiffness to order the welds. Validated against experiments; the chosen sequence reduced out-of-plane deformation by 2.01 % relative to the original sequence. | to the tack weld sampling/deeper search area/ | Ch2 Welded joints, tack welds and standards; Ch3 Tack placement and ordering; Ch7 Future work (distortion-aware tack ordering) | - |
| 070 | **Voxel-Based Full-Field Eigenstrain Reconstruction of Residual Stresses**<br>Uzun and Korsunsky (2), 2023. *Advanced Engineering Materials 25(14), 2201502 (2023), 15 pp., doi:10.1002/adem.202201502 (issue 14 from the Wiley download stamp)* | Inverse eigenstrain method that reconstructs full-field residual stresses, residual elastic strains and displacements in discontinuously processed bodies by superposing local eigenstrain radial basis functions fitted to limited measurements, with all six strain components and no regularising assumptions. FEniCS implementation validated on synchrotron X-ray diffraction data from a bent titanium-alloy bar; hybrid high-throughput parallel computing. | to the tack weld sampling/deeper search area/Eigenstrain (inherent strain) induced linear elastic FEM/ | Ch7 Future work (eigenstrain/inherent-strain background for distortion-aware tack ordering); probably not needed otherwise | - |
| 072 | **Deformable Sheet Metal Fixturing: Principles, Algorithms, and Simulations**<br>Cai et al. (3), 1996. *Journal of Manufacturing Science and Engineering (Transactions of the ASME) 118(3), 318-324 (August 1996); DOI not printed (issue 3 from the ASME download URL)* | Proposes the 'N-2-1' locating principle (N>=3 locators on the primary datum, 2 and 1 on the secondary and tertiary datums) for deformable sheet metal instead of the rigid-body '3-2-1' principle, and finds the best N locator positions with finite element analysis plus nonlinear programming to minimise total deformation under loads such as drilling or resistance spot welding. Implemented in the OFixDesign simulation package with numerical examples. | to the tack weld sampling/deeper search area/fixture layout optimization/ | Ch7 Future work (fixture-aware tack placement / distortion control); Ch2 Welded joints, tack welds and standards (fixturing context) | - |
| 073 | **Development of Fixture Layout Optimization for Thin-Walled Parts: A Review**<br>Liu et al. (6), 2024. *Chinese Journal of Mechanical Engineering 37, 17 (2024), 25 pp., doi:10.1186/s10033-024-01004-w* | Review of fixture layout optimisation for thin-walled parts (car bodies, ship hulls, fuselages), organised by optimisation objective (in-plane variation vs out-of-plane deformation), variation/deformation modelling (mechanism-based FEM, Jacobian, state-space vs data-based RSM, regression, ANN) and optimisation algorithm (nonlinear programming vs heuristics). Closes with future research directions. | to the tack weld sampling/deeper search area/fixture layout optimization/ | Ch7 Future work (fixture-aware tack placement / distortion control); Ch2 Welded joints, tack welds and standards (fixturing context) | - |

Notes:

- **022**: PDF title metadata is a hex-encoded Korean HWP file name; Korean journal with a bilingual footer.
- **060**: Open access; the PDF has no title metadata. The '×' after '19' is missing from the text layer (it reads '19 reduction'); checked on the rendered page.
- **061**: Journal printed as 'Int J Mech Mater Des'; '2^n × n!' is garbled as '2n 9 n!' in the text layer. It sequences weld sub-passes, not tack placement.
- **062**: Machining, not welding; tangential to the thesis.
- **063**: Title printed in capitals; file name is truncated. Tag 067 (AI report) describes this paper as testing 'five sequences' with a 16.06% reduction from 'welding away from the center'; neither the figure nor that wording appears in this PDF's text, which reports seven groups A-G.
- **064**: Journal printed as 'J. Marine Sci. Appl.'; the PDF has no title metadata.
- **066**: AI-generated 'deep research' report, not peer-reviewed: no authors or venue, Anthropic UI fonts (AnthropicSerif/AnthropicSans) embedded, TL;DR and Key Findings sections with source chips, and it addresses the user directly ('your 8.4 mm T-joint', 'your ~1.1 mm root gap'). The printed title differs from the file name; bold text is missing from the text layer, so the first2 text is fragmentary (title read from the rendered page). Check every claim in the primary sources.
- **067**: AI-generated 'deep research' report, not peer-reviewed: Anthropic UI fonts, TL;DR layout, and it addresses the user directly ('the exact problem you are solving', 'your existing ICP-to-CAD residual pipeline', 'your doctoral targeting'). The printed title differs from the file name. Its summary of Tomków et al. 2020 (tag 063: 'five sequences', 16.06% reduction by 'welding away from the center') is not supported by the text of tag 063 (seven groups A-G, no 16.06% figure), so check all claims against the primary sources.
- **069**: PDF metadata title is a Word file name ('Microsoft Word - 33-21-01250fso-수식-2003.doc'). Title says 'strain direct boundary' while the abstract says 'strain directed as boundary (SDB)'. In the 'deeper search area' folder.
- **070**: Validation case is a bent Ti bar, not a weld. Open access (CC BY). Oxford group (Korsunsky).
- **072**: Scanned 1996 ASME paper with an OCR text layer (some OCR errors). Only the initial of the first author is printed (W. Cai). Cited as the origin of 'N-2-1' by tag 073. File name has doubled spaces and underscores.
- **073**: Review article, open access. Cites tag 072 (Cai et al.) for the N-2-1 principle.

Cross-references: Also relevant: 013 (recognising tacked spots in structured-light images), 004 (corners pre-welded before multi-layer welding), 065 (tack ordering by path length/time, listed under J), 116 (adaptive gap model). The two 'deep search result' files (066/067) and the .md version of 066 are AI-generated reports: use them only as pointers to primary sources.

### M. Geometry processing and meshing (methodology support, mostly not needed)

| Tag | Paper | Gist | Folder | Supports | bib |
|---|---|---|---|---|---|
| 084 | **Fast Winding Numbers for Soups and Clouds**<br>Barill et al. (5), 2018. *ACM Transactions on Graphics 37(4), Article 43 (August 2018), 12 pp., doi:10.1145/3197517.3201337* | Extends generalized winding numbers to oriented point clouds and evaluates them on triangle soups and point clouds with a tree-based far-field approximation, up to 1000x faster than direct summation. Applications: inside-outside segmentation, voxelization, signed distances, defect-tolerant mesh booleans, point set surfaces and 3D printing. | triangulation/ | Ch4 Sensor model and rendering (robust inside/outside and exterior tests on part meshes); probably not needed | - |
| 085 | **Poisson Surface Reconstruction**<br>Kazhdan et al. (3), 2006. *Eurographics Symposium on Geometry Processing (SGP) 2006, eds. Konrad Polthier, Alla Sheffer; page numbers not printed* | Casts surface reconstruction from oriented points as a Poisson problem: solve for the indicator function whose gradient best matches the normal field, using an adaptive octree with locally supported B-spline bases (a sparse, well-conditioned system), then extract an isosurface. Shown on public scan data with more detail than earlier methods. | triangulation/ | probably not needed (surface reconstruction background; at most Ch4 Sensor model and rendering) | - |
| 086 | **Screened Poisson Surface Reconstruction**<br>Kazhdan and Hoppe (2), 2013 (inferred). *ACM Transactions on Graphics (author preprint with placeholder citation 'ACM Trans. Graph. NN, N, Article NN' and DOI 10.1145/XXXXXXX.YYYYYYY); final volume/article not printed* | Adds point interpolation (screening) constraints to Poisson surface reconstruction while keeping the same finite-element discretisation and multigrid solver, and changes the octree and multigrid so run time is linear in the number of points. Fits the data more closely than unscreened Poisson, e.g. on an 11.4M-point scan of Michelangelo's David at octree depth 10. | triangulation/ | probably not needed (surface reconstruction background; at most Ch4 Sensor model and rendering) | - |
| 087 | **Tetrahedral Mesh Generation by Delaunay Refinement**<br>Shewchuk (1), 1998 (inferred). *not visible; reference [15] reads 'This proceedings, June 1998', i.e. Proc. 14th Annual ACM Symposium on Computational Geometry (SoCG), 1998 (inferred)* | Delaunay refinement algorithm that, for an input complex of vertices, segments and planar facets with a lower bound on input angles, produces a conforming Delaunay tetrahedral mesh with circumradius-to-shortest-edge ratios no greater than two and provably good grading. The implementation produces meshes better than the bounds and removes many slivers in practice. | triangulation/ | probably not needed (meshing background; at most Ch7 Future work for FEM-based distortion prediction) | - |

Notes:

- **084**: SIGGRAPH 2018 journal paper (ACM TOG).
- **085**: Author/proceedings copy without page numbers.
- **086**: Author preprint with placeholder volume/article/DOI. Year inferred from the PDF creation date (Jan 2013).
- **087**: Author copy (Ghostscript) without a venue header; math symbols are garbled in the text layer. Venue and year inferred from ref. [15].

### N. Tangential / probably not needed

| Tag | Paper | Gist | Folder | Supports | bib |
|---|---|---|---|---|---|
| 001 | **A Survey of Deep Learning-Driven 3D Object Detection: Sensor Modalities, Technical Architectures, and Applications**<br>Zhang et al. (3), 2025. *Sensors 25, 3668 (2025), doi:10.3390/s25123668* | Review of deep-learning 3D object detection along two axes: sensor modality (RGB camera, LiDAR, multimodal fusion at data/feature/result level) and architecture (CNN, BEV, occupancy networks, temporal fusion), with applications in autonomous driving and agricultural robotics. | (pool root) | Probably not needed; at most Ch2 Object recognition and 6D pose estimation (general 3D detection background, mostly LiDAR/driving). | - |
| 002 | **Automatic Aluminum Alloy Surface Grinding Trajectory Planning of Industrial Robot Based on Weld Seam Recognition and Positioning**<br>Zhao et al. (5), 2023. *Actuators 12, 170 (2023), doi:10.3390/act12040170* | Locates defect areas and the weld on large curved aluminium-alloy (aerospace) surfaces by combining 2D images with 3D point clouds, computes weld reinforcement by orthogonal projection of the cloud onto the surface, and plans a spatial spiral grinding trajectory. Simulation and experiments report shorter grinding time than other grinding paths and a smoother surface afterwards. | (pool root) | Ch2 Weld seam detection from 3D data (2D image + 3D point cloud weld positioning; grinding rather than welding application). | - |
| 009 | **A contour error definition, estimation approach and control structure for six-dimensional robotic machining tasks**<br>Zhao et al. (4), 2022. *Robotics and Computer-Integrated Manufacturing 73, 102235 (2022), doi:10.1016/j.rcim.2021.102235* | Defines position and orientation contour error for six-dimensional robotic machining (e.g. grinding, polishing), estimates it with a locally iterative bidirectional/Fibonacci search plus Slerp, and compensates weighted error components in task-space velocity commands. On a 6-DOF robot, estimation errors stay within 0.01062 um / 0.8959 urad and mean position/orientation contour errors drop by about 30%/28% vs tracking control alone. | (pool root) | Probably not needed; at most Ch2 Motion planning and trajectory generation (path-following/contour error of 6D tool paths). | - |
| 015 | **Exploiting image quality measure for automatic trajectory generation in robot-aided visual quality inspection**<br>Jafari-Tabrizi et al. (3), 2024. *The International Journal of Advanced Manufacturing Technology 132, 4885-4901 (2024), doi:10.1007/s00170-024-13609-5* | Uses only the image focus measure (thresholded absolute gradient) of an end-effector camera to find the optimal camera-to-surface distance, generate a scanning trajectory over an unknown free-form surface, and localise a known surface at an unknown position, on a KUKA LWR-4. Pose precision under 1 mm and 0.1 degrees; fewer than 50 iterations to generate a trajectory and fewer than 30 to localise. | (pool root) | Probably not needed; at most Ch3 Multi-view close-range refinement (camera stand-off and viewpoint chosen from image feedback). | - |
| 016 | **Industrial Robotic Setups: Tools and Technologies for Tracking and Analysis in Industrial Processes**<br>Makulavičius et al. (5), 2025. *Applied Sciences 15, 10249 (2025), doi:10.3390/app151810249* | Review of vision-based object detection, mainly YOLO variants, and other machine-learning models (e.g. classification and regression trees) for classification, localisation and tracking in industrial robotic setups, including ROS-based systems and robotic machining; compares selected publications and lists open challenges. | (pool root) | Probably not needed; at most Ch2 Object recognition and 6D pose estimation (YOLO-based 2D detection in industrial cells, as contrast to 6D pose). | - |
| 031 | **Vision-centric 3D point cloud technique and custom gripper process for parcel depalletisation**<br>Kim et al. (4), 2025. *Journal of Intelligent Manufacturing 36, 5179-5195 (2025), doi:10.1007/s10845-024-02497-x* | In-truck parcel depalletising: an IMU gravity frame is combined with the point cloud frame, truck planes and out-of-reach regions are removed, flat box patches are detected, and Euclidean distance with regression kNN selects the nearest pick-up point. Validated with a custom robot and gripper. | (pool root) | Probably not needed; at most Ch3 Perception and registration (plane removal and ROI filtering of depth point clouds). | - |
| 043 | **TR3D: Towards Real-Time Indoor 3D Object Detection**<br>Rukhovich et al. (3), 2023 (arXiv v3). *arXiv:2302.02858v3 [cs.CV] (5 Dec 2023); no venue printed* | Fully convolutional sparse-3D-CNN detector derived from FCAF3D (first- and fourth-level heads removed, fewer channels, new assigner, DIoU loss), plus TR3D+FF, an early 2D-3D feature fusion for RGB and point-cloud input. In the S3DIS ablation it reaches 74.5 mAP with 14.7 M parameters at 21 scenes/s (FCAF3D: 66.7 mAP, 70.5 M, 10.9), and it reports state-of-the-art results on ScanNet v2, SUN RGB-D and S3DIS. | realtime scan to cad/ | Probably not needed; at most Ch2 Object recognition and 6D pose estimation (3D object detection in point clouds as an alternative front end) | - |
| 068 | **Modelling crystal plasticity by 3D dislocation dynamics and the finite element method: The Discrete-Continuous Model revisited**<br>Vattré et al. (7), 2014. *Journal of the Mechanics and Physics of Solids 63, 491-505 (2014), doi:10.1016/j.jmps.2013.07.003* | Revisits the Discrete-Continuous Model, which couples 3D dislocation dynamics (microMegas) with finite elements (Z-set) through eigenstrain theory, and details how dislocation slip is regularised into plastic strain at the FE integration points. Validated on test cases and applied to plastic flow in a single-crystal Ni-based superalloy. | to the tack weld sampling/deeper search area/ | Probably not needed (micro-scale crystal plasticity); the only link is the eigenstrain concept behind inherent-strain distortion models | - |
| 071 | **Eigenstrain-Induced Stress in Elastic Cylinder**<br>Dargi et al. (3), 2026. *Journal of Applied Mechanics (Transactions of the ASME) 93(1), 011006 (January 2026), doi:10.1115/1.4070149* | Finite-element elastic analysis, in scaled parameters, of the stress induced in a free elastic cylinder by an axial volumetric eigenstrain field with a transverse discontinuity at the midsection (general case plus ramp and step limits). Motivated by reaction-driven cracking in periclase core-flooding experiments; discontinuities in the eigenstrain or its gradient produce tensile stresses near the front. | to the tack weld sampling/deeper search area/Eigenstrain (inherent strain) induced linear elastic FEM/ | probably not needed (eigenstrain mechanics of rock cores; at most background for Ch7 Future work) | - |
| 081 | **Adaptive Automatic Robot Tool Path Generation Based on Point Cloud Projection Algorithm**<br>Xie et al. (3), 2019. *2019 24th IEEE International Conference on Emerging Technologies and Factory Automation (ETFA), pp. 341-347 (venue from the PDF metadata; IEEE footer 978-1-7281-0303-7/19/$31.00 ©2019 IEEE; DOI not printed)* | Generates robot tool paths from a handheld-scanned point cloud without CAD: segment the cloud, project it onto a plane, design a 2D tool-path pattern (OpenCV), and back-project it to 3D waypoints with tool angles (PCL), then plan and collision-check in ROS (Descartes, RViz). Demonstrated for masking, polishing and painting coupons. | superficial/ | Ch2 Motion planning and trajectory generation (scan-based tool-path generation without CAD, a contrast to CAD-based seams); probably minor | - |

Notes:

- **001**: File name starts with a space and has a space before '.pdf' (quote the path when scripting).
- **002**: File name starts with a space and has a space before '.pdf' (quote the path when scripting).
- **043**: arXiv preprint (v3); title printed in capitals. Probably published at IEEE ICIP 2023 (inferred, not printed; verify). Tangential indoor 3D detection paper filed under 'realtime scan to cad'.
- **068**: Off-topic for the thesis (filed under 'to the tack weld sampling/deeper search area'); the PDF has no title metadata.
- **071**: Not about welding: geomechanics (hydration of periclase to brucite, CO2 mineral storage), filed in the eigenstrain folder. Issue dated January 2026, published online 11 Nov 2025.
- **081**: Venue only in the PDF metadata (no IEEE Xplore stamp, no DOI). First author printed as 'Xie Zhen' (surname assumed to be Xie; may be indexed as 'Zhen, X.'). Filed under 'superficial/'.

### O. Non-paper files in `paper_pool/`

| File | What it is | Use |
|---|---|---|
| `Welding_Robots_Approach_and_Trajectory_Literature_Report.md` (47 KB) | AI-generated report ("Author: Antigravity AI", dated Sept 2026) on torch orientation, approach motion, collision avoidance and tack sequencing. It has a pipeline diagram, a synthesis table of 10 pool papers and a reference list. | **Not citable.** Several references are wrong when checked against the PDFs: Geng et al. is given as "(2022) RCIM 78, 102381" (it is 2023, vol. 79, 102433); Peng et al. 2020 as "IROS, 381-386" (it is the 17th Int. Conf. on Ubiquitous Robots, pp. 380-386); Jia et al. as "(2024) Applied Sciences 14(15), 6542" (it is Actuators 14, 287, 2025); Tomków et al.'s co-authors as "K. Sobota, S. Krajewski" with wrong given names (they are Konrad Sobota and Sławomir Krajewski); Ramsey et al. (CAPT) as "IROS 2023" (it is arXiv 2406.02807, 2024). Zhou et al. 2022 (Lazy-PRM, RCIM 74) and Wang et al. 2024 (gantry RRT\*, RCIM 85) have no PDF here: verify them before use. Useful only as a list of ideas (dihedral bisector, approach/retract phases, roll redundancy). |
| `to the tack weld sampling/deep search result/Analytical & Heuristic Methods ... Route-by-Route Investigation.md` (32 KB) | Markdown version of the AI-generated PDF tag 066 (seven "routes" for ordering tack welds: inherent strain, empirical shrinkage, Tsai's joint rigidity method, Rosenthal heat accumulation, named sequences, optimisation, German standards). | Not citable. Use only as a pointer to primary sources: Tsai et al. 1999 joint rigidity, Ueda/Murakawa inherent strain, Rosenthal 1946, DVS 1710, EN ISO 13920, EN 1090. Each must be fetched and checked. |
| `images/1681389460-28054800.jpg`, `images/4.png`, `images/a-guide-to-modular-fixturing-1579284937.jpg`, `images/s-l1200.jpg`, `images/welding-fixutre2.webp` | Web photos of modular welding/fixture tables with clamps, squares and tube frames. `s-l1200.jpg` carries a vendor watermark and a brand-name clamp. | Copyright unknown: do not reproduce them in the thesis without permission. Photograph your own fixtures or the cell instead. |

### P. Cross-reference: `weld_generator/papers/` (20 files)

| `weld_generator/papers/` file | Same paper in paper_pool (tag) | bib key |
|---|---|---|
| implemented_papers/lit-lobb.pdf | 119 Zhang et al. 2025 (RCIM 95) | `zhang_lobb` |
| implemented_papers/lit-modelreg.pdf | 134 Fang & Tian 2024 (RCIM 89) | `fang2024modelreg` |
| implemented_papers/lit-pcaslice.pdf | 118 Wang et al. 2026 (Welding in the World) | `wang2026pipes` |
| implemented_papers/lit-ppf.pdf | 128 Wang et al. 2024 (Sci. Rep. 14) | `wang2024ppfseam` |
| implemented_papers/lit-quadric.pdf | 137 Li et al. 2026 (JMST 40(3)) | `li2026quadric` |
| implemented_papers/lit-ransac.pdf | 127 (and 047) Yi et al. 2026 (Autom. Constr. 183) | `yi2026seam` |
| implemented_papers/lit-regiongrow.pdf | 121 Wei et al. 2024 (IROS 2024) | `wei_regiongrow` |
| 3D curve weld seam path and posture planning ..._compressed.pdf | 131 Wang et al. 2025 (RCIM 94) | - |
| A novel method for dense point cloud reconstruction and weld seam.pdf | 133 Wang et al. 2023 (Opt. Laser Technol. 163) | - |
| A novel seam extraction and path planning method ... 3D vision.pdf | 135 Geng et al. 2023 (RCIM 79) | `geng2023seam` |
| A_Coarse-to-Fine_Multimodal_Detection_Framework_..._Coating_Tasks.pdf | 120 Lin et al. 2026 (T-Mech 31(1)) | `lin2026coating` |
| A_Feature_Extraction_Approach_Over_Workpiece_Point_Clouds_for_Robotic_Welding.pdf | 136 Zhang et al. 2025 (T-ASE 22) | - (recommended) |
| LWSNet A lightweight network for automated welding point cloud.pdf | 122 Song et al. 2025 (Measurement 243) | `song2025lwsnet` |
| Path  Planning  with  Automatic  Seam  Extraction ... Arc  Welding.pdf | 123 Zhou et al. (arXiv 2020; RA-L 2021) | - (recommended) |
| Point Clouds Area Exploration and Active Recognition ... welding.pdf | 124 Tao et al. 2026 (RCIM 102) | - |
| SANet_ A Pure Vision Strip-Aware Network ... .pdf | 125 Zhu et al. 2025 (Appl. Sci. 15) | `zhu2025sanet` |
| Weld seam detection based on 2D image and 3D point cloud segmentation_ ... .pdf | 126 Ma et al. 2025 (IJAMT 141) | - |
| WeldNet.pdf | 129 Wang et al. 2024 (Sci. China Tech. Sci. 67) | - |
| YOLO–LaserGalvo.pdf | 130 Li et al. 2025 (Sensors 25) | - |
| **RRT-Connect_Faster_asymptotically_optimal_motion_planning.pdf** | **only here** (tag 182): it is **RRT\*-Connect**, Klemm et al., ROBIO 2015, not Kuffner & LaValle's RRT-Connect | `klemm2015rrtstarconnect` |

All 19 copies are byte-identical (same MD5). Only one weld_generator file, RRT\*-Connect, has no
paper_pool copy.


## Part 2 - Technical digests of the seven supply papers

Each digest is written from the full PDF in `supply_papers/`. Equations are given in the papers'
own notation; numbers in parentheses are the papers' equation numbers. Some facts come from the
cell's code rather than the papers; these are marked **[cell]** and were read, read-only, from
`scripts/scripts_in_foundationpose/fp_server.py`, `fp_tracking.py`,
`scripts/gesture_control_node.py`, `admittance_control/marking.py` and `src/transit_cpp.cpp`.
Facts that come from neither the paper nor the code are marked **[not in PDF]**.

---

### 2.1 FoundationPose (Wen, Yang, Kautz, Birchfield - CVPR 2024)

**Exact citation.** B. Wen, W. Yang, J. Kautz, and S. Birchfield, "FoundationPose: Unified 6D
Pose Estimation and Tracking of Novel Objects," in *Proc. IEEE/CVF Conference on Computer Vision
and Pattern Recognition (CVPR)*, 2024, pp. 17868-17879 *(pages: online, CVF/mlanthology)*.
The PDF in hand is **arXiv:2312.08344v2 [cs.CV], 26 Mar 2024**: 15 pp. including the supplementary
material, NVIDIA. Project page: nvlabs.github.io/FoundationPose.

**Problem and setting.** The task is to estimate the rigid transform from object to camera, in
RGB-D images, for objects **not seen during training**, and to **track** it through a video. One
trained network serves both test-time setups:

- **Model-based:** a textured CAD model of the object is given.
- **Model-free:** a small set of posed reference RGB-D images (about 16) is given, and an
  object-centric neural field is built from them for novel-view rendering.

No fine-tuning happens at test time. In the paper's taxonomy, "instance-level" methods train per
object and "category-level" methods train per category; FoundationPose is neither ("novel
object").

**Architecture (paper Fig. 2).**

1. *Language-aided synthetic data generation* (training only, §3.1).
2. *Neural object modelling* (model-free only, §3.2):
   - The geometry network $\Omega: x \mapsto s$ is a signed distance field. The appearance network
     $\Phi: (f_{\Omega(x)}, n, d) \mapsto c$ maps the geometry feature, the normal and the view
     direction to a colour.
   - The input $x$ goes through a multi-resolution hash encoding (as in Instant-NGP); $n$ and $d$
     are embedded with second-order spherical harmonics. The surface is
     $\mathcal{S}=\{x\in\mathbb{R}^3 \mid \Omega(x)=0\}$.
   - $\Omega$ is a 2-layer MLP (hidden size 64) and $\Phi$ a 3-layer MLP (64, sigmoid output). The
     hash encoding has 4 levels with 16-128 features and a table of $2^{22}$. Training takes about
     2k steps, i.e. seconds.
   - After training, marching cubes extract a textured mesh, which is rasterised. Sphere tracing
     was found slower when many hypotheses are rendered.
3. *Pose hypothesis generation* (§3.3): global initialisation, then a **transformer-based pose
   refiner**.
4. *Pose selection* (§3.4): a **hierarchical pose-ranking network**.

**Shared image backbone (Fig. 9, supplementary).**

- Two RGB-D inputs (rendering and observation crop, each resized to 160×160) pass through one
  CNN with shared weights: 7×7 conv, 64 ch, /2; 3×3 conv, 128 ch, /2; then two 3×3 ResBlocks
  (ResNet-34 type).
- The two feature maps are concatenated and passed through ResBlocks (3×3, 256), a 3×3 conv
  (512, /2) and two more 3×3 ResBlocks (512).
- The result is split into patches (tokens) and given a position embedding.

**Pose refiner (§3.3).**

- *Inputs:* a rendering of the object at the coarse pose, and a **pose-conditioned crop** of the
  observation. The crop is centred at the projected object origin and sized by the projected,
  slightly enlarged object diameter. Because the crop moves with the hypothesis, the network gets
  feedback on translation. Unlike MegaPose, only one view (at the coarse pose) is rendered.
- *Outputs:* the tokens feed **two separate transformer encoders** (embedding 512, 4 heads,
  feed-forward 512), each followed by a linear head. One predicts $\Delta t\in\mathbb{R}^3$, the
  other $\Delta R\in SO(3)$ as an axis-angle vector (a 6D rotation representation gave similar
  results).
- *Update:* the update is disentangled, with both terms expressed in the **camera frame**:

  $$ t^{+} = t + \Delta t \quad (8), \qquad R^{+} = \Delta R \otimes R \quad (9), $$

  so the translation step does not depend on the rotation step. In the usual homogeneous update
  $x'=\Delta T\,x=\Delta R\,x+\Delta t$, the translation acts in the object's updated (rotated)
  local frame, so the rotation affects the translation. The disentangled form removes this
  coupling and "stabilizes training" (supplementary §5.3).
- *Loss:*

  $$ \mathcal{L}_{\text{refine}} = w_1\lVert \Delta t-\Delta\bar t\rVert_2 + w_2\lVert \Delta R-\Delta\bar R\rVert_2 \quad (10), \qquad w_1=w_2=1 . $$

- *Training perturbations:* translation noise up to 0.02/0.02/0.05 m (x/y/z) and rotation noise
  up to 20°.
- *Iterations:* 1 during training; at test time **5 for estimation and 1 for tracking**.

**Global pose initialisation (§3.3).**

- The object is detected by an off-the-shelf 2D detector (Mask R-CNN or CNOS).
- Translation is initialised at "the 3D point located at the median depth within the detected
  2D bounding box".
- Rotations: $N_s$ camera viewpoints are sampled uniformly on an icosphere around the object,
  looking at its centre, and each is combined with $N_i$ discretised in-plane rotations. With
  **$N_s=42$ and $N_i=12$** there are **504 hypotheses**, all of which are refined.

**Hierarchical pose ranking (§3.4).**

- *Level 1 (per hypothesis):* the rendering and its pose-conditioned crop pass through the
  **pose-ranking encoder**, which uses the refiner's backbone. The features are concatenated,
  tokenised and passed through multi-head self-attention, then average-pooled into an alignment
  embedding $\mathbf F\in\mathbb R^{512}$.
- *Level 2 (across hypotheses):* the $K$ embeddings are stacked into
  $\mathbf F=[\mathbf F_0,\dots,\mathbf F_{K-1}]^\top\in\mathbb R^{K\times512}$ and treated as a
  sequence for multi-head self-attention. There is **no positional encoding**, so the operation is
  permutation-agnostic and works for any $K$. A linear projection then gives scores
  $\mathbf S\in\mathbb R^{K}$, and the best-scoring pose is the output. The point is that a
  hypothesis is scored *relative to the others* instead of with an absolute score, which the
  authors argue is hard to learn.
- *Training: the pose-conditioned triplet loss*

  $$ \mathcal L(i^+,i^-)=\max\big(\mathbf S(i^-)-\mathbf S(i^+)+\alpha,\,0\big) \quad (11),\qquad \alpha=0.1 . $$

  Positives and negatives are ranked by ADD against ground truth. The anchor is *not* shared,
  because each hypothesis has its own crop. Only pairs whose positive is near the ground-truth
  viewpoint are kept:

  $$ \mathbb V^+=\{i: D(R_i,\bar R)<d\},\qquad \mathbb V^-=\{0,\dots,K-1\},\qquad \mathcal L_{\text{rank}}=\sum_{i^+\in\mathbb V^+,\ i^-\in\mathbb V^-,\ i^+\neq i^-}\mathcal L(i^+,i^-) \quad (12\text{-}14), $$

  where $D$ is the geodesic rotation distance and $d=10^\circ$. $K=5$ when training this network
  alone. InfoNCE was tried and performed worse.

**Neural field losses (model-free only).**

- Volume rendering over a truncated near-surface band:

  $$ c(r)=\int_{z(r)-\lambda}^{z(r)+0.5\lambda} w(x_i)\,\Phi\big(f_{\Omega(x_i)},n(x_i),d(x_i)\big)\,dt,\qquad w(x_i)=\frac{1}{1+e^{-\alpha\Omega(x_i)}}\cdot\frac{1}{1+e^{\alpha\Omega(x_i)}} \quad (1\text{-}2). $$

- The losses are

  $$ \mathcal L_c=\frac{1}{|\mathcal R|}\sum_{r}\lVert c(r)-\bar c(r)\rVert_2,\quad \mathcal L_e=\frac{1}{|\mathcal X_e|}\sum_{x\in\mathcal X_e}|\Omega(x)-\lambda|,\quad \mathcal L_s=\frac{1}{|\mathcal X_s|}\sum_{x\in\mathcal X_s}(\Omega(x)+d_x-d_D)^2, $$
  $$ \mathcal L_{eik}=\frac{1}{|\mathcal X_s|}\sum_{x\in\mathcal X_s}(\lVert\nabla\Omega(x)\rVert_2-1)^2,\qquad \mathcal L=w_c\mathcal L_c+w_e\mathcal L_e+w_s\mathcal L_s+w_{eik}\mathcal L_{eik}, $$

  with $\lambda=1$ cm, $w_e=1$, $w_s=1000$ and $w_c=100$.

**Training data and procedure (§3.1, supplementary §5.2).**

- *3D assets:* Objaverse-LVIS (more than 40K objects in 1156 LVIS categories) and Google Scanned
  Objects (GSO).
- ***LLM-aided texture augmentation:*** a two-level prompt hierarchy. ChatGPT is given a templated
  prompt that contains the object's Objaverse-LVIS tag and asks for a description of its possible
  appearance. The answer becomes the text prompt of **TexFusion**, a text-guided diffusion texture
  synthesiser, together with the object shape and a random noisy initial texture. Each Objaverse
  object gets 3-5 seeds and styles.
  - This replaces FS6D's random pasting of ImageNet/COCO images onto the UV map, which caused
    seams and unrealistic appearance.
- *Rendering:* scenes are rendered in **NVIDIA Isaac Sim with path tracing**, with physics: 70-90
  objects are dropped onto a 1-1.5 m platform with invisible walls and allowed to settle.
  - The objects are scaled to 5-30 cm.
  - Each scene has 0-5 random lights and 2 cameras on a hemisphere 0.2-3.0 m above the platform.
  - Materials are randomised (metallicness, reflection), and a dome light uses 662 Poly Haven HDRIs.
  - The result is about **600K scenes and 1.2M images**. Accuracy saturates around 1M images
    (Fig. 7).
- *Training procedure:*
  - For each asset, a neural object field is first pretrained on synthetic reference images. Its
    renderings are mixed with OpenGL renderings so that the networks see both setups.
  - The refiner is trained for 50 epochs and the ranker for 25; joint end-to-end fine-tuning then
    runs for 5 epochs.
  - Total training takes about **one week on 4 NVIDIA V100s**.
  - Test time uses one RTX 3090. Model-based rendering uses the nvdiffrast rasteriser (ref. [33]).
    Depth is denoised by erosion and a bilateral filter (in Warp), and cropping is batched in
    Kornia.

**Inference pipeline, step by step (model-based, as used in the cell).**

1. Inputs: an RGB image, a depth image (m), the intrinsics $K$, the CAD mesh, and a 2D
   detection or mask of the object.
2. Initialise translation from the depth inside the detection. The paper says median depth in the
   2D box; the released code's `register()` takes an object mask `ob_mask`. **[cell]** passes the
   SAM2 mask here (`EST.register(K=K, rgb=rgb, depth=depth, ob_mask=mask, iteration=EST_REFINE_ITER)`).
3. Build the 504 rotation hypotheses (icosphere × in-plane).
4. Render each hypothesis and crop the observation around it; run the refiner $n$ times
   ($n=5$ by default; **[cell]** `EST_REFINE_ITER = 5`).
5. Score all refined hypotheses with the two-level ranker and output the best
   $\mathbf T_{C\leftarrow O}$ (4×4).
6. **Tracking mode:** for each new frame, render at the previous pose, crop the new frame at that
   pose, and run *only the refiner*, once in the paper. **[cell]** uses `refine_iter = 2` per frame
   in `fp_tracking.py`. The refined pose becomes the current pose. There are no multiple
   hypotheses, no ranking, and no lost-track detection or re-initialisation in the method itself.

**Inputs and outputs.**

- *In:* RGB-D, $K$, a mask or box, and the CAD mesh (or ~16 posed reference views).
- *Out:* the object pose in the camera frame, plus ranking scores during estimation.

**Reported performance.**

- *Model-free pose estimation:*
  - YCB-Video, 16 references (AUC): ADD-S 97.4 / ADD 91.5, against 88.4 / 42.1 for FS6D-DPM
    (Table 1).
  - LINEMOD, ADD-0.1d: average 99.9, against 91.5 for FS6D+ICP (Table 2).
- *Model-based pose estimation (BOP AR, RGB-D, Mask R-CNN detections, Table 3):*
  - FoundationPose: LM-O 78.8, T-LESS 83.0, YCB-V 88.0, **mean 83.3**.
  - MegaPose-RGBD: mean 58.6. The instance-level SurfEmb+ICP: mean 79.7.
  - It was first on the BOP 2023 unseen-object leaderboard at submission (with CNOS detections).
- *Tracking:*
  - YCBInEOAT, model-based, ground-truth initialisation: ADD-S 96.42 / ADD 93.09. For comparison,
    se(3)-TrackNet (instance-level) scores 95.53 / 92.66 and BundleSDF 93.77 / 86.95. "Ours†", with
    its own estimate as initialisation, scores 96.40 / 93.22 (Table 4).
  - YCB-Video, all frames: ADD 96.0 / ADD-S 97.9 model-based and 93.7 / 97.5 model-free, against
    86.4 / 96.5 for ICG and 87.5 / 95.2 for PoseRBPF (Table 5). No re-initialisation is applied
    after a loss of track.
- *Ablation (YCB-V, model-free, ADD / ADD-S, Table 6):*

  | Variant | ADD | ADD-S |
  |---|---|---|
  | full | 91.52 | 97.40 |
  | without LLM texture augmentation | 90.83 | 97.38 |
  | without transformer | 90.77 | 97.33 |
  | **without hierarchical comparison** | **89.05** | **96.67** |
  | InfoNCE loss | 89.39 | 97.29 |

  Accuracy saturates at 12 reference images.
- *Run time (i9-10980XE + RTX 3090):* pose estimation takes about **1.3 s per object** (4 ms
  initialisation, 0.88 s refinement, 0.42 s selection). **Tracking runs at about 32 Hz.** The
  intended use is "pose estimation once for initialization, then tracking mode".

**Limitations stated by the authors (supplementary §5.4).**

- The method relies on **external 2D detection** (CNOS / Mask R-CNN), and "false or missing
  detection frequently bottlenecks the 6D pose estimation". An end-to-end detection + pose +
  tracking framework is left to future work.
- A typical failure mode combines a **texture-less object, severe occlusion and limited edge
  cues**, which leads to the wrong orientation (Fig. 11).
- The method handles a single rigid object; "state estimation beyond single rigid object" is
  future work.

**What this means for the thesis text.**

- *Write:* "model-based setup, CAD given, no fine-tuning; masks from SAM2 replace the 2D detector;
  504 hypotheses refined 5×; the best chosen by a learned ranker; tracking = refiner only".
- *Add (as the thesis's contribution, not the method's):* lost-track detection and
  re-registration, ego-motion cancellation, symmetry handling (swing-twist) and the
  desktop/laptop split. In particular, FoundationPose has **no notion of symmetry** in the ranker
  or the tracker. For an axisymmetric tube the spin about the axis is unobservable, which is
  exactly why `eq:swing_twist` is needed.
- *Caution:* the cell's parts are mostly untextured metal or printed parts, which is the failure
  regime the authors name (texture-less plus occlusion). This is worth one sentence in the
  discussion of FP-vs-ICP results.
- *Run-time numbers:* the paper's figures are for an RTX 3090; quote the cell's own (RTX 5070 Ti
  desktop / RTX 4060 laptop) separately.

---

### 2.2 SAM 2 (Ravi et al. - arXiv 2024 / ICLR 2025)

**Exact citation.** N. Ravi, V. Gabeur, Y.-T. Hu, R. Hu, C. Ryali, T. Ma, H. Khedr, R. Rädle,
C. Rolland, L. Gustafson, E. Mintun, J. Pan, K. V. Alwala, N. Carion, C.-Y. Wu, R. Girshick,
P. Dollár, and C. Feichtenhofer, "SAM 2: Segment Anything in Images and Videos,"
arXiv:2408.00714v2 [cs.CV], 28 Oct 2024 (Meta FAIR). It was published in *International
Conference on Learning Representations (ICLR)*, 2025, as an oral *(online: ICLR proceedings /
mlanthology)*.

Note footnote 1 of v2: "All the results presented in this paper are based on a new version of
SAM 2 ... denoted as **SAM 2.1**". **[cell]** uses `sam2.1_hiera_small.pt` with config
`configs/sam2.1/sam2.1_hiera_s.yaml` through `SAM2ImagePredictor`.

**Task: Promptable Visual Segmentation (PVS).**

- Prompts are **positive or negative clicks, boxes or masks**, given on any frame of a video. The
  model must return a valid mask on the prompted frame immediately, then propagate it to a
  spatio-temporal **masklet**. More prompts on any frame refine it (Fig. 2).
- The image task (SA, "segment anything") and semi-supervised VOS are special cases (Fig. 7). An
  image is a one-frame video: **"When applied to images, the memory is empty and the model behaves
  like SAM."**
- As in SAM, the targets are "valid objects with a clearly defined boundary"; parts and sub-parts
  count, and there is no semantic class.

**Architecture (Fig. 3, Fig. 8, Appendix D).**

- ***Image encoder: Hiera*** (Ryali et al. 2023; Bolya et al. 2023), a hierarchical ViT
  pre-trained with MAE. It runs **once per frame** (streaming) and produces unconditioned
  embeddings.
  - A feature pyramid network fuses the stride-16 and stride-32 features (stages 3-4) into the
    image embedding, which is 256×64×64 for a 1024×1024 input.
  - The stride-4 and stride-8 features (stages 1-2) **skip** the memory attention and are added in
    the mask decoder's upsampling layers for fine detail.
  - Windowed absolute positional embeddings are used, with the global embedding interpolated. All
    relative positional biases are removed, which allows FlashAttention-2 and a large speed-up at
    1024 px with no loss on SA-23 (Table 10).
  - Encoder sizes are T, S, B+ and L. Global attention is used only in some blocks: 5-7-9 (T),
    7-10-13 (S), 12-16-20 (B+), 23-33-43 (L).
- ***Memory attention:*** $L=4$ transformer blocks. Each block applies **self-attention**, then
  **cross-attention to the memory bank** (spatial memories plus object-pointer tokens), then an MLP.
  - Vanilla attention is used.
  - Positions are encoded by sinusoidal absolute embeddings plus **2D RoPE** in self- and
    cross-attention. Object pointers are excluded from RoPE.
  - The first block takes the current frame's image embedding.
- ***Prompt encoder (identical to SAM):***
  - Sparse prompts (points, box corners) are represented by **positional encodings summed with a
    learned embedding per prompt type**, so a positive click and a negative click differ only in
    their learned type embedding.
  - Mask prompts are embedded with convolutions and **summed with the frame embedding**.
- ***Mask decoder (SAM-style):*** stacked **"two-way" transformer blocks** (token→image and
  image→token attention) update the prompt/output tokens and the frame embedding. Two transposed
  convolutions upsample, using the stride-4 and stride-8 skips, and a dot product with each
  output-mask token gives the masks.
  - For ambiguous prompts (a single click) **several masks** are predicted, each with a predicted
    IoU. In video, if later prompts do not resolve the ambiguity, the highest-IoU mask is
    propagated.
  - New in SAM 2: an **occlusion/presence head**, an extra token plus an MLP that scores whether
    the object is visible in this frame. The output mask token also serves as the frame's **object
    pointer**.
- ***Memory encoder:*** downsamples the predicted mask with a conv module, adds it element-wise to
  the frame's unconditioned Hiera embedding (no second image encoder), and fuses the result with
  light conv layers. Memory features are projected to **64 channels**.
- ***Memory bank:*** **FIFO of up to $N$ recent frames** (default $N=6$ past frames) and **FIFO of
  up to $M$ prompted frames**, both stored as spatial maps.
  - It also stores a **list of object pointers**: 256-d vectors, each split into 4 tokens of 64-d.
  - **Temporal position encodings** are added to the $N$ recent memories only, not to the prompted
    ones.
  - A learned occlusion embedding is added to the memories of frames predicted invisible.
  - With several objects, the image features are shared and everything else runs per object, with
    **no inter-object communication**.

**Training data and procedure.**

- *Pre-training* on SA-1B images:
  - The encoder is initialised from MAE-Hiera. Masks covering more than 90% of the image are
    dropped, and 64 masks are sampled per image.
  - The IoU predictions get an ℓ1 loss after a sigmoid. With multi-mask output, the IoU
    predictions of all masks are supervised, but only the mask with the lowest focal+dice loss gets
    the mask loss.
  - 7 correction clicks are used (8 in SAM), with horizontal flips at 1024×1024.
  - Optimisation: AdamW, layer-wise decay, reciprocal-square-root schedule, about 90k steps,
    batch 256, lr 4e-4.
- *Full training* alternates image and video batches, sampled in proportion to dataset size: about
  15.2% SA-1B, 70% SA-V and 14.8% Internal. A variant adds DAVIS, MOSE and YouTube-VOS.
  - Training **simulates interaction** on 8-frame sequences: up to 2 frames (including the first)
    receive prompts.
  - The initial prompt is the ground-truth mask (p = 0.5), **a positive click sampled from the
    ground-truth mask (0.25)** or a box (0.25). Corrective clicks are sampled from the error between
    ground truth and prediction; in 10% of cases they are drawn from the ground truth alone.
  - Each sequence has at most 3 masklets, and the time order is reversed with p = 0.5.
  - Augmentation: horizontal flip, affine (25°, shear 20), colour jitter (also per frame),
    grayscale (0.05) and a **2×2 mosaic** (p = 0.1). The mosaic teaches the model to tell
    identical-looking objects apart and to handle small objects.
  - About 150k steps at batch 256, with lr 6e-5 for the image encoder and 3e-4 for the rest
    (cosine).
  - *Losses:* focal + dice for masks, MAE for IoU, cross-entropy for object presence, weighted
    **20 : 1 : 1 : 1**. A frame without a ground-truth mask gets no mask loss, but the presence
    head is always supervised.
  - A last fine-tuning stage uses **16-frame sequences** from the most-edited (hardest) 50% of
    masklets: 50k iterations at half the learning rate, with the image encoder frozen.
  - The released model was trained on 256 A100s for 108 h.
- ***Data engine*** (three phases, Table 1):

  | Phase | Tool | Time per frame | Output |
  |---|---|---|---|
  | 1 | SAM per frame, at 6 FPS | 37.8 s | 16K masklets; method used for the val/test sets |
  | 2 | SAM + "SAM 2 Mask" (mask-only propagation) | 7.4 s | 63.5K masklets |
  | 3 | full SAM 2 with point refinement | **4.5 s**, about **8.4× faster** than Phase 1 | 197.0K masklets |

  - A separate set of annotators **verifies** every masklet.
  - **Auto masklets** come from a 32×32 point grid on the first frame (plus 16×16 grids on 4
    zoomed crops and 4×4 grids on 16 crops) and are then verified.
  - ***SA-V:*** **50.9K videos** (54% indoor; average 14 s; 47 countries), **642.6K masklets**
    (190.9K manual + 451.7K automatic) and **35.5M masks**, 53× more than any previous VOS
    dataset. Licence CC BY 4.0.

**Inference with point prompts on a single image (the cell's case).**

1. Encode the frozen colour frame once with Hiera + FPN.
2. Encode each click $(u,v)$ as a positional encoding plus the learned *positive* or *negative*
   type embedding.
3. Run the two-way-transformer decoder, which gives one or several mask logits plus predicted IoU.
   A single click returns multiple candidates; the released predictor returns all of them and the
   caller usually keeps the highest predicted IoU.
4. Threshold the logits to get a binary mask, upsampled to image size.
5. To refine, add more clicks, positive or negative, and optionally the previous low-resolution
   mask logits as a mask prompt, then decode again.

The memory path is unused for a single image. **[cell]** gets one mask per part from the clicked
points (`sam2_mask(predictor, rgb, points, labels)`), and the masked depth becomes the part's
cloud.

**Reported performance.**

- *Video:*
  - Interactive (offline and online) evaluation on 9 zero-shot datasets: better than SAM+XMem++
    and SAM+Cutie, with "**>3× fewer interactions**".
  - Semi-supervised VOS on 17 zero-shot datasets (J&F, Table 4):

    | Prompt on the first frame | SAM 2 | SAM+Cutie |
    |---|---|---|
    | 1 click | 64.7 | 56.7 |
    | 3 clicks | 75.3 | 70.1 |
    | 5 clicks | 77.6 | 72.2 |
    | box | 74.4 | 69.4 |
    | ground-truth mask | 79.3 | 74.1 |

  - State-of-the-art VOS (Table 6), Hiera-B+ / Hiera-L:

    | Benchmark | Hiera-B+ | Hiera-L |
    |---|---|---|
    | MOSE val | 76.6 | 77.9 |
    | DAVIS 2017 val | 90.2 | 90.7 |
    | LVOS val | 78.0 | 78.0 |
    | SA-V val | 76.8 | 77.9 |
    | SA-V test | 77.0 | 78.4 |
    | YTVOS19 (G) | 88.6 | 89.3 |

    Speed: **43.8 and 30.2 FPS** on an A100 at batch 1.
- *Images (SA task, 37 datasets, Table 5/15):*
  - 1-click mIoU on SA-23: 58.9 (SA-1B only) and **61.9** (full mix), against 58.1 for SAM ViT-H.
    5-click: 81.7 / 83.5 against 81.3.
  - Speed: **130.1 FPS against 21.7 FPS ("6× faster")**.
  - The paper gives **no 1024-px numbers for Hiera-S**, the variant the cell uses. The only S
    numbers are the 512-px ablation (Table 9f): S scores 57.8 SA-23 mIoU and runs 1.33× faster,
    against 59.7 for B+ and 61.1 for L.
- *Ablations:* accuracy follows a power law in the amount of SA-V data. Object pointers help on
    SA-V and LVOS. A GRU memory gives no gain.

**Limitations stated by the authors (Appendix C).**

- SAM 2 may fail across **shot changes**, and may lose or confuse objects in **crowded scenes,
  after long occlusions or in long videos**.
- It struggles with **very thin or fine details, especially when they move fast**, and with
  **nearby objects that look alike**; more explicit motion modelling might help.
- Objects are processed **separately**, without shared object-level context.
- The data engine still needs humans to verify masklets.
- The fairness study notes that 1-click prompts are ambiguous: the model often segments a *part*
  (the shirt) instead of the whole person.

**What this means for the thesis text.**

- *Name the variant:* "SAM 2.1, Hiera-S, image predictor". In single-image mode the model reduces
  to SAM's promptable segmentation with a Hiera encoder.
- *Cite SAM as well:* the prompt encoder and decoder are "identical to SAM's", so cite Kirillov
  et al., ICCV 2023 (Part 4).
- *Ambiguity:* the part-vs-whole ambiguity of a single click is exactly why the cell lets the
  operator add negative clicks (Thumb_Down, "not this part").
- *Failure modes to discuss:* thin parts (plates seen edge-on) and look-alike neighbouring parts
  are the named failure modes; mention them in the limitations or discussion.

---

### 2.3 MediaPipe (Lugaresi et al. - arXiv 2019)

**Exact citation.** C. Lugaresi, J. Tang, H. Nash, C. McClanahan, E. Uboweja, M. Hays, F. Zhang,
C.-L. Chang, M. G. Yong, J. Lee, W.-T. Chang, W. Hua, M. Georg, and M. Grundmann, "MediaPipe: A
Framework for Building Perception Pipelines," arXiv:1906.08172v1 [cs.DC], 14 Jun 2019 (Google
Research). 9 pp.

**The PDF is the framework paper only.** It does **not** describe:

- hand detection, hand landmarks or gesture classification;
- any model architecture, any training data;
- any accuracy figure.

Its two worked examples are (i) object detection + tracking and (ii) face landmarks + portrait
segmentation. The hand landmark model is in **MediaPipe Hands**: F. Zhang, V. Bazarevsky,
A. Vakunov, A. Tkachenka, G. Sung, C.-L. Chang, M. Grundmann, "MediaPipe Hands: On-device
Real-time Hand Tracking," arXiv:2006.10214, 2020, presented at the CVPR 2020 Workshop on Computer
Vision for AR/VR *(online: arXiv record)*. The gesture classifier is documented only in the
MediaPipe **Gesture Recognizer** task documentation and model card; it has no paper.

**What the framework paper does describe** (citable for "MediaPipe graph / calculators /
timestamps").

The framework has three parts: an inference framework for sensory data, performance-evaluation
tools, and reusable components called **calculators**.

- **Packet:** a numeric timestamp plus a shared pointer to an immutable payload of any C++ type.
  Copies are cheap and reference-counted.
- **Stream:** a sequence of packets with **monotonically increasing timestamps**. One output stream
  can feed many input streams, and each input keeps its own queue.
- **Side packet:** a single packet with no timestamp, used for constant data such as a model file
  path.
- **Calculator:** a graph node. It declares its contract in `GetContract()` (types checked at graph
  start), prepares in `Open()`, runs `Process()` whenever an input set is ready, and finishes in
  `Close()`. A calculator runs on at most one thread at a time unless parallel execution is
  explicitly enabled.
- **Graph / GraphConfig:** the graph is defined in a protobuf: nodes, options, input policies and
  executors.
  - Subgraphs are inlined when the graph is loaded.
  - At initialisation the framework checks that each stream and side packet has exactly one
    producer, that connected types are compatible, and that every node's connections match its
    contract.
- **Scheduling:**
  - Each scheduler queue has exactly one executor; by default there is one queue whose executor is
    a thread pool.
  - Each node is "not ready", "ready" or "running".
  - The queue is a priority queue: nodes are topologically sorted, and those nearer the outputs get
    higher priority.
- **Synchronisation:** there is no global clock; timestamps are the synchronisation key.
  - Each stream has a *timestamp bound*. After a packet at time $T$ the bound becomes $T+1$, and a
    timestamp is **settled** once it is below the bound.
  - Under the **default input policy**, packets with equal timestamps are processed together, in
    strictly increasing order. Nothing is dropped and the result is deterministic. A node is ready
    when some timestamp is settled on all inputs and at least one input has a packet at it.
- **Flow control:** two mechanisms.
  - Back-pressure: a full queue throttles the upstream nodes, with deadlock avoidance.
  - **Flow-limiter nodes:** these drop packets under real-time constraints, with a loop-back from
    the graph output so the limiter knows how many timestamps are in flight.
- **GPU support:** an opaque buffer type with API-specific views (OpenGL ES up to 3.2 on Android,
  ES 3.0 / Metal on iOS). Several GL contexts can be used, and synchronisation through
  producer/consumer **sync fences** is inserted automatically.
- **Tools:** the tracer records `TraceEvent`s (event time, packet timestamp, data id, node id,
  stream id) in a mutex-free circular buffer. The visualiser has a timeline view and a graph view.

**Reported performance.** None, beyond qualitative statements ("immensely successful at Google
for over 6 years").

**Limitations stated.** None explicit. The paper positions MediaPipe against ROS: inter-process
communication (IPC) between ROS nodes costs overhead that MediaPipe avoids by running in one
process.

**MediaPipe Hands, for the thesis [not in PDF].** These are the main points to confirm once the
paper is fetched.

- A two-stage pipeline:
  1. A **palm detector** ("BlazePalm") runs on the full frame and returns an oriented hand box.
  2. A **hand-landmark model** runs on the crop and returns **21 hand landmarks** in 2.5D (x, y,
     relative depth), plus hand-presence and handedness scores.
- On later frames, the crop comes from the previous frame's landmarks; the detector runs again
  only when the presence confidence drops.
- It runs in real time on mobile GPUs.

The **Gesture Recognizer** task adds a gesture-embedding model and a classifier on top of the
landmarks. Its canned classifier outputs exactly the seven classes in `tab:gestures` (Closed_Fist,
Open_Palm, Pointing_Up, Thumb_Down, Thumb_Up, Victory, ILoveYou), plus "None". Check these names
and the model-bundle structure against the current Google AI Edge documentation page and cite that
page as a `@misc` with its URL and access date.

**[cell]** `gesture_control_node.py` uses the MediaPipe Tasks `GestureRecognizer` with the
`gesture_recognizer.task` bundle (`float16/1`), `RunningMode.VIDEO`, `num_hands=1` and
`min_hand_detection_confidence=0.6`.

**What to write.**

- Cite `lugaresi2019mediapipe` for the framework ("MediaPipe [x]").
- Cite MediaPipe Hands for the landmark model, and the Gesture Recognizer documentation for the
  seven canned gestures.
- The latch (N stable frames) and the 2 s hold are the thesis's own debouncing on top of
  per-frame classification.

---

### 2.4 Point pair features I: Drost, Ulrich, Navab, Ilic - CVPR 2010

**Exact citation.** B. Drost, M. Ulrich, N. Navab, and S. Ilic, "Model Globally, Match Locally:
Efficient and Robust 3D Object Recognition," in *2010 IEEE Computer Society Conference on Computer
Vision and Pattern Recognition (CVPR)*, 2010, pp. 998-1005, doi:10.1109/CVPR.2010.5540108. The DOI
is in the PDF metadata; the footer reads 978-1-4244-6985-7/10/$26.00 ©2010 IEEE. Affiliations:
MVTec Software GmbH and TU München. 8 pp.

**Problem.** Recognise free-form rigid objects and estimate their 6D pose in 3D point clouds
(laser, ToF, stereo) with noise, clutter and partial occlusion, **without a coarse initial pose**
and without segmentation. Model and scene are sets of **oriented points** (point + normal).

**The feature (eq. 1).** For two oriented points $(\mathbf m_1,\mathbf n_1)$ and
$(\mathbf m_2,\mathbf n_2)$, with $\mathbf d=\mathbf m_2-\mathbf m_1$:

$$ \mathbf F(\mathbf m_1,\mathbf m_2)=\big(\lVert\mathbf d\rVert_2,\ \angle(\mathbf n_1,\mathbf d),\ \angle(\mathbf n_2,\mathbf d),\ \angle(\mathbf n_1,\mathbf n_2)\big),\qquad \angle(\mathbf a,\mathbf b)\in[0,\pi]. $$

The feature is based on Wahl et al.'s surflet pairs but is **asymmetric**:
$\mathbf F(\mathbf m_1,\mathbf m_2)\neq\mathbf F(\mathbf m_2,\mathbf m_1)$. This is exactly the
thesis's `eq:ppf`.

**Global model description (§3.2).**

- Offline, $\mathbf F$ is computed for **all pairs** $(\mathbf m_i,\mathbf m_j)\in M^2$ of the
  (subsampled) model.
- Distances are quantised in steps of $d_{dist}$ and angles in steps of
  $d_{angle}=2\pi/n_{angle}$. Pairs whose quantised features are equal are grouped together.
- Formally the description is a mapping $L:\mathbb Z^4\to A\subset M^2$ from the quantised feature
  to the set of model pairs having it. It is stored as a **hash table** keyed by the quantised
  feature, so all model pairs similar to a scene pair are retrieved in constant time.
- The angle $\alpha_m$ of each model pair (see below) is precomputed and stored with it.

**Local coordinates and voting (§3.3).**

- Take a scene reference point $\mathbf s_r$ and assume it lies on the object, corresponding to a
  model point $\mathbf m_r$. Once the two points and their normals are aligned, the only remaining
  freedom is a rotation $\alpha$ about the normal. A pose is therefore parametrised by the **local
  coordinates** $(\mathbf m_r,\alpha)$: two degrees of freedom for the model point and one for the
  angle, instead of six.
- $T_{m\to g}$ translates $\mathbf m_r$ to the origin and rotates $\mathbf n^m_r$ onto the
  $x$-axis; $T_{s\to g}$ does the same for $\mathbf s_r$
  ($T_{m\to g}\mathbf n^m_r=T_{s\to g}\mathbf n^s_r=\mathbf e_1$). A model pair
  $(\mathbf m_r,\mathbf m_i)$ matches a scene pair $(\mathbf s_r,\mathbf s_i)$ when

  $$ \mathbf s_i = T_{s\to g}^{-1}\,R_x(\alpha)\,T_{m\to g}\,\mathbf m_i \quad (2), $$

  and the model-to-scene pose is $T_{s\to g}^{-1}R_x(\alpha)T_{m\to g}$.
- **Efficient voting loop.** Split $\alpha=\alpha_m-\alpha_s$, using
  $R_x(\alpha)=R_x(-\alpha_s)R_x(\alpha_m)$:

  $$ \mathbf t = R_x(\alpha_s)\,T_{s\to g}\,\mathbf s_i = R_x(\alpha_m)\,T_{m\to g}\,\mathbf m_i \in \mathbb R\mathbf x+\mathbb R_0^{+}\mathbf y \quad (3). $$

  So $\alpha_m$ is computed **offline** per model pair, $\alpha_s$ **once** per scene pair, and
  $\alpha$ is a subtraction.
- **Voting scheme** (similar to the Generalised Hough Transform). For each $\mathbf s_r$, use a
  **2D accumulator** with $N_m=|M|$ rows (model points) and $N_{angle}=n_{angle}$ columns
  (rotation bins). Pair $\mathbf s_r$ with every other scene point $\mathbf s_i$ and compute
  $\mathbf F_s(\mathbf s_r,\mathbf s_i)$. For every model pair $(\mathbf m_r,\mathbf m_i)$ in the
  hash bucket, compute $\alpha$ and cast **one vote for $(\mathbf m_r,\alpha)$**.
- The accumulator peak gives the best local coordinate and hence a global pose. "For stability",
  all peaks with enough votes relative to the maximum are kept.
- The caption of Fig. 4 has two typos: it reads "vote for $(\mathbf m_i,\alpha)$" and
  "$T_{s\to g}^{-1}R_x(\alpha)T_{s\to g}\mathbf m_i$". The text and eq. (2) give
  $(\mathbf m_r,\alpha)$ and $T_{m\to g}$.

**Pose clustering (§3.4).**

- Many reference points are used, because at least one must lie on the object; each returns
  candidate poses from its peaks.
- The candidates are **clustered** so that the poses in a cluster differ by less than preset
  translation and rotation thresholds.
- A pose's score is its number of votes, and a cluster's score is the sum of its poses' scores.
- The best cluster's poses are **averaged** to give the result. Several clusters can be returned
  for several instances.
- Clustering removes isolated low-score poses, and averaging improves accuracy.

**Parameters and set-up (§4).**

- $d_{dist}=\tau_d\,\mathrm{diam}(M)$ with $\tau_d=0.05$ by default. $n_{angle}=30$, which allows a
  normal deviation of up to 12°.
- Model and scene are subsampled to a minimum point distance $d_{dist}$, and normals are
  recomputed by plane fitting at that scale. 1/5 of the scene points are used as reference points.
- No ICP refinement is used in the results.
- Building the model description from all pairs took "several minutes per model".
- The implementation is single-threaded Java on a 2 GHz Core Duo.

**Reported performance.**

- *Synthetic, single object* (4 models × 50 views = 200 clouds, Gaussian noise relative to the
  diameter): a correct detection means an error below $\mathrm{diam}/10$ and 12°. The Bunny was
  found in 0.96 s at σ = 5% of the diameter.
- *Synthetic, multi-object* (50 scenes, 347 instances, $|S|\approx1690$ points):

  | Reference points | Objects found | Time per object | Note |
  |---|---|---|---|
  | $\lvert S\rvert/5$ | 89.3% | 1.32 s | 98% of objects with >15% visible surface found |
  | $\lvert S\rvert/10$ | - | 0.76 s | |
  | $\lvert S\rvert/40$ | 77.2% | 0.35 s | about 4× faster |

- *Real (Mian et al. dataset, 50 Minolta scenes)*, recognition rate for objects with less than 84%
  occlusion:

  | Method | Recognition rate | Time per object |
  |---|---|---|
  | this method, $\tau_d=0.025$ | **97.0%** | 85 s |
  | tensor matching (Mian et al.) | 96.6% | 90 s |
  | spin images | 87.8% | 2 h |
  | this method, $\tau_d=0.04$ | 89.2% | 1.97 s (more than 40× faster) |

**Limitations stated or implied by the authors.**

- Missed objects are mostly the highly occluded ones (more than 85% occlusion).
- There is a **speed versus recognition-rate trade-off** set by the sampling ($\tau_d$, the number
  of reference points).
- Model building is quadratic in the number of model points (minutes).
- The method needs normals at the sampling scale.

**What to write.** For classification, the cell votes each segmented cloud against every model of
the library and verifies the top hypotheses by explained and covered points; this matches Drost's
voting plus a Birdal-style verification. If quantisation values ($\tau_d$, $n_{angle}$) and the
number of reference points are reported, report them next to Drost's defaults.

---

### 2.5 Point pair features II: Birdal and Ilic - 3DV 2015 ("revisited")

**Exact citation.** T. Birdal and S. Ilic, "Point Pair Features Based Object Detection and Pose
Estimation Revisited," in *2015 International Conference on 3D Vision (3DV)*, 2015, pp. 527-535,
doi:10.1109/3DV.2015.65 (DOI printed). Affiliations: TU München (CAMP) and Siemens AG. 9 pp.

**This is not Hinterstoisser et al.** "Going Further with Point Pair Features" (Hinterstoisser,
Lepetit, Rajkumar, Konolige, ECCV 2016) is a different paper and is **not in the folders**. It is
known for smarter pair sampling and for spreading votes against quantisation noise **[not in
PDF; fetch it]**.

**Motivation.** Drost's method is criticised, in the authors' words citing [4,16,19], for:

- the high dimensionality of the search space;
- sensitivity to 3D correspondences;
- performance drops with many outliers and low-density surfaces.

Its successors on RGB-D (e.g. LINEMOD) "cannot handle occlusions and heavily depend on
post-processing and pose refinement". The paper revises the **whole pipeline while keeping the
feature and the matching scheme unchanged**.

**Model representation (offline).**

1. **Normals.** Normals come from a **second-order MLS** fit: a local polynomial height field,
   weighted by $w=\exp(-\lVert p_i-p_k\rVert^2/2\sigma_{mls}^2)$, onto which each point is
   projected. Linear (PCA) normals are not accurate enough on curved patches.
2. **Feature.** The feature is unchanged (antisymmetric 4D; eq. 1), but every angle is computed
   as
   $$\angle(\mathbf v_1,\mathbf v_2)=\tan^{-1}\!\frac{\lVert\mathbf v_1\times\mathbf v_2\rVert}{\mathbf v_1\cdot\mathbf v_2}\quad(2),$$
   which is numerically safe for small angles and lies in $[0,\pi)$.
3. **Model-point weights from ambient occlusion.** For a point $p$ on the surface:
   $$A_p=\frac{1}{\pi}\int_\Omega V(p\cdot w)\,dw \quad (3),$$
   where $V$ is 1 if $p$ is occluded in direction $w$. It is approximated by rendering the model
   from many views (cosine-weighted).
   - A hash-table entry is weighted by the normalised geometric mean of $A_{m_r}$ and $A_{m_i}$,
     so likely hidden points count less.
   - This weighting is *scene-independent*, unlike Tuzel et al.'s learned weights. It improves
     pose accuracy more than detection rate.
4. **Down-sampling.** Poisson-disk sampling enforces the minimum distance $d_{dist}$ required by
   the quantisation.

**Online matching.**

- Scene normals come from range-image (SRI) normals (Badino et al.).
- **Coarse-to-fine segmentation.** The depth image is segmented by Felzenszwalb-Huttenlocher graph
  segmentation adapted to depth, with edge weight
  $w(v_i,v_j)=\lVert v_i^n-v_j^n\rVert\,\angle(n_i,n_j)$ (7), and the usual predicate (eqs. 4-6):
  - $D(C_1,C_2)=\min_{(v_i,v_j)\in E,\ v_i\in C_1,\ v_j\in C_2} w(v_i,v_j)$;
  - $M_{int}(C_1,C_2)=\min\big(Int(C_1)+\tau(C_1),\,Int(C_2)+\tau(C_2)\big)$;
  - $Int(C)=\max_{e\in MST(C,E)}w(e)$ and $\tau(C)=k/|C|$.
  - Eq. (4) is printed as "1 if $D>M_{int}\le 0$", a typo for the standard "boundary if
    $D>M_{int}$".
  - Segments with many invalid depths, the wrong size, or near-planar normals are dropped. The
    planarity test is $\sigma(\mathbf N^i)=\frac{1}{|\mathbf N^i|}\sum_j(\angle(\mathbf n_{ij},\bar{\mathbf n}_i))^2<\tau_c$ (8).
  - A hierarchy of segmentations, from under- to over-segmented, is used; **each segment is voted
    in its own Hough space**. Less clutter means cleaner votes and better poses. The worst case is
    the whole scene, i.e. the original algorithm.
- **Weighted Hough voting.**
  - Votes carry the model weights and are also added to neighbouring bins to absorb quantisation
    noise.
  - A second accumulator $Acc_\alpha$ keeps the weighted mean $\alpha$ per bin.
  - Sub-bin accuracy comes from fitting a second-order polynomial to the k-NN of the discrete
    maximum.
- **Pose clustering and averaging.**
  - Clustering is agglomerative and done per segment: candidates are sorted by votes, a new
    cluster is opened only if a pose differs significantly from the existing ones, and the cluster
    mean and score are updated as poses join.
  - Rotations are averaged as **quaternions** (Markley et al. 2007):
    $\mathbf A=\frac1{n_q}\sum_i w_i^q\,(\mathbf q_i^T\cdot\mathbf q_i)$ (9), as printed. The method
    intends the 4×4 outer product $\mathbf q_i\mathbf q_i^T$.
  - $w_i^q$ is the number of scene points explained by pose $\mathbf q_i$, and $\mathbf q_{avg}$ is
    the eigenvector of the largest eigenvalue of $\mathbf A$. This is "twice as better as the naive
    mean".
- **Occlusion-aware hypothesis verification (no ICP needed).** For hypothesis $h$ with pose
  $\Theta_h$, each valid pixel gets the error
  $$E_h(p,m)=D_p-\Phi(p\mid\mathbf M,\Theta_h,\mathbf K)\quad(10),$$
  the measured depth minus the rendered model depth. The pixel is classified as
  $$p\in\begin{cases}S_m & |E_h|\le\tau_m\\ S_o & E_h\ge\tau_o\\ S_c&\text{otherwise}\end{cases},\qquad
  S_h=\Big(1-\frac{|p\in S_o|}{N_m}\Big)\cdot\frac{|p\in S_m|}{N_m-|S_o|}\quad(11),$$
  - $S_m$ = on the model, $S_o$ = occluder, $S_c$ = clutter.
  - $N_m$ is the number of model points in the valid projection.
  - Normal agreement is checked with eq. (2).
  - The score is computed with GPU z-buffering, and the top $N_{best}$ poses are returned.

**Complexity.** Pairing $M$ reference points with $N$ scene points costs $O(MN)$. Splitting into
$K$ clusters gives $O(MN/K^2)$ per cluster and $O(MN/K)$ overall.

**Reported performance.**

- Downsampling at 3% of the diameter; $n_{angle}=45$.
- *Synthetic* (7 ACCV3D objects × 162 views = 1134 scenes, success = 10% of the diameter and 10°):
  the rotation error is about half of Drost's for many objects (Fig. 5).
- *Real ACCV3D/LINEMOD subset, depth only, no refinement, Table 1:*

  | Object | LineMod (depth only) | Drost | Ours |
  |---|---|---|---|
  | ape | 42.88% | 65.54% | 81.95% |
  | cam | 68.78% | 84.92% | 91.00% |
  | cat | 35.62% | 87.30% | 95.76% |
  | driller | 51.52% | 81.06% | 81.22% |
  | iron | 35.22% | 87.06% | 93.92% |
  | **average** | **46.80%** | **81.18%** | **88.77%** |
  | average run time | 119 ms | 6.3 s | 2.9 s (2× faster than Drost) |

  - The gains are larger for small objects (detection) and for large ones (pose accuracy).
  - ICP converges in fewer iterations from these poses (Fig. 7).
  - With a good segmentation, matching took 170 ms using 1/50 of the scene points.

**Limitations and future work (authors).**

- The run time depends on the segment size and number, and the optimum is scene-dependent. With
  large segments the method falls back to the original algorithm.
- LineMod was run on depth only (a handicap), and parameters were not tuned per object.
- Future work: "scalable joint detection and pose estimation using segments and adaptive
  sampling".

**What to write.**

- Cite Drost for the feature, the hash table and the voting.
- Cite Birdal for the improvements the cell actually uses or resembles:
  - normal quality;
  - per-segment voting, which the cell gets for free from SAM2 masks;
  - quaternion pose averaging;
  - an occlusion/clutter-aware score $S_h$, analogous to the cell's coverage × explanation
    verification. State this analogy explicitly.

---

### 2.6 RRT-Connect (Kuffner and LaValle - ICRA 2000)

**Exact citation.** J. J. Kuffner, Jr. and S. M. LaValle, "RRT-Connect: An Efficient Approach to
Single-Query Path Planning," in *Proc. IEEE International Conference on Robotics and Automation
(ICRA)*, San Francisco, CA, 2000, vol. 2, pp. 995-1001, doi:10.1109/ROBOT.2000.844730 *(volume,
pages and DOI online; pages also confirmed by ref. [7] of the Luna et al. PDF)*.

**The PDF in hand is the pre-publication draft**, headed "DRAFT: Submitted to the 2000 IEEE
International Conference on Robotics and Automation" (7 pp.). Kuffner was at Stanford and LaValle
at Iowa State. Its text layer is unreadable, so it was read from page images.

**Problem.** Single-query path planning in high-dimensional configuration spaces $\mathcal C$, with
**no differential constraints**: find a continuous path in $\mathcal C_{free}$ from $q_{init}$ to
$q_{goal}$ without preprocessing. A metric $\rho$ on $\mathcal C$ is assumed, and collision checks
answer whether $q\in\mathcal C_{free}$; $\mathcal C_{free}$ is never represented explicitly.

**Algorithms (pseudocode as in Figs. 2 and 5).**

```
BUILD_RRT(q_init):
  T.init(q_init)
  for k = 1..K:  q_rand <- RANDOM_CONFIG();  EXTEND(T, q_rand)
  return T

EXTEND(T, q):
  q_near <- NEAREST_NEIGHBOR(q, T)
  if NEW_CONFIG(q, q_near, q_new):            # step of fixed size eps toward q, collision-checked
      T.add_vertex(q_new); T.add_edge(q_near, q_new)
      return Reached if q_new = q else Advanced
  return Trapped

CONNECT(T, q):                                # greedy: repeat EXTEND toward q
  repeat S <- EXTEND(T, q) until not (S = Advanced)
  return S

RRT_CONNECT_PLANNER(q_init, q_goal):
  Ta.init(q_init); Tb.init(q_goal)
  for k = 1..K:
      q_rand <- RANDOM_CONFIG()
      if not (EXTEND(Ta, q_rand) = Trapped):
          if CONNECT(Tb, q_new) = Reached: return PATH(Ta, Tb)
      SWAP(Ta, Tb)
  return Failure
```

- **Key ideas.**
  - The RRT grows toward random samples. The probability that a vertex is chosen for extension is
    proportional to the volume of its **Voronoi region**, so the tree is biased to explore quickly
    and then covers the space uniformly (Fig. 4).
  - The **Connect heuristic** replaces the single ε-step with repeated steps until the sample or
    an obstacle is reached. It plays the role of the attractive potential in randomized potential
    fields, but its "basin of attraction" moves as the tree grows, which helps avoid local minima.
  - Two trees, rooted at start and goal, take turns: one extends toward a random sample, and the
    other greedily tries to connect to the new vertex.
- **Variants discussed.** EXTEND/EXTEND (two plain RRTs), CONNECT/CONNECT (greedier), and adding
  only the last vertex of a CONNECT. The choice depends on the nearest-neighbour method and the
  problem.

**Analysis (§4).**

- Let $D_k(q)$ be the distance from $q$ to the closest of the $k$ RRT vertices.
- *Lemma 1* (convex, bounded, open $\mathcal C_{free}$): $\lim_{k\to\infty}P[d_k(q)<\epsilon]=1$
  for every $q\in\mathcal C_{free}$ and $\epsilon>0$.
- *Lemma 2:* the same holds for a non-convex, bounded, connected component.
- *Theorem 1:* the vertex distribution $X_k$ converges in probability to the sampling distribution
  $X$, which is uniform over $\mathcal C_{free}$ in the usual case.
- *Corollary 1:* **RRT-Connect is probabilistically complete**, and its vertices converge to a
  uniform distribution over $\mathcal C_{free}$.
- **No convergence-rate result** is given, and there is no optimality claim.

**Reported performance.**

- Hundreds of 2D/3D rigid-body runs on a 200 MHz SGI Indigo2, with paths smoothed afterwards.
  Over 100 trials, the three examples of Fig. 6 took **0.228 s, 5.94 s and 2.92 s**.
- Compared with plain two-RRT growth, Connect sped up most problems by "a factor of three or
  four", especially uncluttered ones. In very cluttered scenes it was slightly slower than
  EXTEND/EXTEND.
- Other demonstrations:
  - a piano moved between rooms;
  - a **6-DOF PUMA** arm moving a book, combined with IK;
  - **7-DOF human-arm** motions for virtual chess, about 2 s per motion, 8000 triangles, RAPID
    OBB-tree collision checking;
  - an assembly-maintenance query with a narrow passage: 80 s on the Indigo2, about 15 s on an
    R10000 machine, 13,000 triangles.

**Limitations (authors).**

- There is no extensive benchmark, so it is not shown to be "the fastest".
- "Pathological cases exist", and more work is needed to find when it performs poorly.
- There is no theory for the convergence rate.
- Proposed improvements: step size from collision-free ball radii, all-CONNECT, not storing
  intermediate CONNECT vertices, approximate nearest neighbours (from $O(n)$ to near-logarithmic
  time), incremental collision detection.
- Implied by the method: no differential constraints, and the paths are **not optimised** (they
  are smoothed afterwards). This is why APS (2.7) and shortcutting are used on top of it.

**What to write.** RRT-Connect is the base planner inside the cell's APS transits: 4 parallel
instances **[cell]**. It is probabilistically complete but neither asymptotically optimal nor
smooth, so its raw paths need shortening. RRT\*-Connect (Klemm et al. 2015, `weld_generator/papers/`)
is the asymptotically optimal bidirectional alternative, already cited.

---

### 2.7 Anytime solution optimisation / AnytimePathShortening (Luna, Şucan, Moll, Kavraki - ICRA 2013)

**Exact citation.** R. Luna, I. A. Şucan, M. Moll, and L. E. Kavraki, "Anytime Solution
Optimization for Sampling-Based Motion Planning," in *Proc. IEEE International Conference on
Robotics and Automation (ICRA)*, Karlsruhe, Germany, 2013, pp. 5053-5059,
doi:10.1109/ICRA.2013.6631301 *(pages and DOI online: Kavraki-lab publication page)*.

The PDF in hand is the authors' copy, headed "To appear in Proc. 2013 IEEE Intl. Conf. on Robotics
and Automation" (7 pp.). Luna, Moll and Kavraki were at Rice University; Şucan at Willow Garage.

**Problem.** Under a fixed time budget, how much can planner-agnostic post-processing shorten
sampling-based paths, and can it compete with asymptotically optimal planners such as RRT\*? The
metric is **path length**. The authors say the scheme applies to other metrics (smoothness,
clearance), possibly with other optimisers.

**The two building blocks.**

- **Shortcutting** (Algorithm 1; Geraerts & Overmars 2007).
  - Repeatedly pick a random segment $\widetilde{ab}$ of the path $\pi$ and form the straight
    path $ab$ between its endpoints.
  - If $|ab|<|\widetilde{ab}|$ and `CheckMotion(ab)` succeeds, substitute it.
  - Repeat for $s$ attempts.
  - It makes "micro-level" improvements within the path's homotopy class.
- **Hybridization** (Raveh, Enosh, Halperin 2011).
  - Build a **hybridization graph** whose vertices are the states of several solution paths; it
    starts as the disjoint union of the paths.
  - Add valid **bridges** between paths, then run a graph search for the shortest hybrid path.
  - All-pairs bridging is $O(n^2)$. For a bounded number of paths, a linear-time dynamic
    programming matching of two paths exists.
  - It makes "macro-level" improvements and can find a **new homotopy class**.

**The meta-algorithm (Algorithm 2, Anytime Path Shortening).**

```
input: planner p, time budget b, m = #solutions to hybridize
smooth <- true
while time() < b:
    if p.Plan():                         # a new solution was found
        if smooth: Shortcut(p.BestSolution())
        else:      Hybridize(m, p.Solutions())
        smooth <- not smooth             # alternate the two optimisers
return p.BestSolution()
```

- Any traditional planner can be wrapped in it, giving the planner the "anytime" property: the
  best path so far is always available and keeps improving.
- With a large budget, only the best $m$ (or a random $m$) solutions are hybridised.
- "For general problem solving, a number of different motion planners can be executed **in
  parallel**", with a counter choosing which optimiser each one applies.

**Implementation in OMPL (§IV).**

- At most **24 paths** are hybridised, because of the cost of the dynamic programming.
- Shortcut attempts are capped at the number of states in the path.
- A shortcut longer than **1/3 of the path length** is not validated; a new pair is drawn instead.

**Reported performance.**

- *Rigid bodies* (50 runs; 2.4 GHz Xeon): Barriers SE(2), 3-DOF; Cubicles SE(3), 6-DOF; Easy, two
  robots in SE(3), 12-DOF.
  - APS tightly converges at or below shortcut-only and hybridize-only. It is about **20% shorter**
    in Barriers (many homotopy classes) and about **10%** in Cubicles, and almost the same as
    shortcutting in Easy.
  - Against **RRT\***: RRT\* is about 5% shorter in the limit in Barriers and Cubicles, but in
    12-DOF Easy APS is **5-10% shorter** than RRT\*.
- *Manipulation* (PR2 7-DOF arm via ROS/MoveIt + OMPL; scenes Post, Table, Clutter; 25 runs):
  - Shortcutting dominates in the sparse scenes (one homotopy class).
  - In Clutter, hybridize-only is about 15% longer than shortcutting or APS.
  - **RRT\* paths are about 20% (Clutter) to nearly 50% (Post) longer** than APS paths.
  - **RRT\* failed to find any solution in 50% or more of the Post and Clutter runs**, even at the
    longest budgets; the traditional planners rarely failed (Fig. 7).
  - Results become more repeatable with more time.

**Limitations and caveats (authors).**

- APS has **no proven optimality guarantee**; asymptotically optimal planners serve "a different
  purpose".
- Wrapping asymptotically optimal planners inside APS is non-trivial, because they do not stop at
  the first solution.
- **Non-holonomic systems** cannot easily be shortcut or hybridised.
- In cluttered PR2 scenes APS gave little over shortcutting, which is left to future work.
- A different cost metric may need different optimisers.

**[cell]** `transit_cpp.cpp` builds `og::AnytimePathShortening` with **4 `RRTConnect`
sub-planners**, `setHybridize(true)`, `setShortcut(true)` and
`setMaxHybridizationPath(8)` (the paper used 24). It plans in scaled joint coordinates under a
`PathLengthOptimizationObjective` with cost threshold 0 ("never good enough: use the whole
budget"), with a 2 s budget extensible to 10 s (`marking.py`). OMPL runs the sub-planners in
**parallel threads**.

**What to write.** Use "AnytimePathShortening (APS) [Luna et al. 2013] in OMPL [Şucan et al.
2012]: several RRT-Connect instances [Kuffner & LaValle 2000] run in parallel within a time
budget; each new solution is shortcut [Geraerts & Overmars 2007] and the solution set is
hybridized [Raveh et al. 2011]". The last two are not in the folders (Part 4). Then justify APS
over RRT\* with Luna's high-DOF results.


## Part 3 - Bibliography check (`manuscript/thesis.bib`)

`thesis.bib` has 36 entries; 21 of them correspond to a PDF in these folders. Each of the 21 was
compared field by field with its PDF: authors in order, title, venue, volume, issue, pages or
article number, year and DOI. Fields that the PDF does not print are marked *(online)* when they
were confirmed from a public record, and *(memory)* when they were written from general
knowledge.

### 3.1 Entries with a PDF in the folders

| Key | PDF (tag) | Verdict | What is wrong or missing (bib → PDF) |
|---|---|---|---|
| `luo2014aps` | 173 supply | **WRONG PAPER** | The entry is Luo & Hauser, "An empirical study of optimal motion planning", IROS 2014. The AnytimePathShortening paper in `supply_papers/` is **Luna, Şucan, Moll, Kavraki, "Anytime Solution Optimization for Sampling-Based Motion Planning", ICRA 2013, pp. 5053-5059** *(pages online)*, doi:10.1109/ICRA.2013.6631301 *(online)*. Replace the entry with `luna2013aps` and update `\cite` in ch2 l.68 and ch3 l.545. |
| `liu2023rcim` | 105 | **placeholder** | Surname-only authors and a "VERIFY" title. PDF: Yan Liu, Qiu Tang, Xincheng Tian, Shuai Yang, "A novel offline programming approach of robot welding for multi-pipe intersection structures based on NSGA-II and measured 3D point-clouds", RCIM 83 (2023) 102549, doi:10.1016/j.rcim.2023.102549. |
| `zhang_lobb` | 119 (= lit-lobb) | **incomplete** | year `VERIFY` → **2025**; add volume **95**, article **102987**, doi:10.1016/j.rcim.2025.102987. The authors and title match. |
| `fang2024modelreg` | 134 (= lit-modelreg) | **wrong authors** | `Fang, Weihua and Tian, Xincheng and others` → exactly **two** authors: Weihua Fang, Xincheng Tian. Vol 89, art. 102772 and 2024 match; add doi:10.1016/j.rcim.2024.102772. |
| `li2026quadric` | 137 (= lit-quadric) | **wrong title / incomplete** | The title is truncated and altered: "... based on normals of point clouds" → "**Automatic recognition on impeller shape and weld seam based on normal of point clouds and PCA**". Authors `Wang, Y.` / `Wang, Z.` → **Yu Wang**, **Zhengyong Wang**. Pages `2039` → **2039-2046** (printed in the citation box). Volume 40(3), 2026 and the DOI match. |
| `lin2026coating` | 120 | **incomplete** | pages `639` → **639-650** (last page verified on p. 12); add month Feb and doi:10.1109/TMECH.2025.3595263. Authors, title and volume/issue match. |
| `wei_regiongrow` | 121 (= lit-regiongrow) | **venue missing** | The PDF is arXiv:2408.10710v1 (20 Aug 2024; no venue printed). Published version: **IEEE/RSJ IROS 2024, Abu Dhabi, pp. 7138-7144, doi:10.1109/IROS58592.2024.10802546** *(online: Univ. Bedfordshire research portal; confirm in IEEE Xplore)*. Authors and title match. |
| `wen2024foundationpose` | 167 supply (= 092) | **incomplete** | The PDF is arXiv:2312.08344v2. Add **pages 17868-17879** *(online: CVF open access / mlanthology)* and doi:10.1109/CVPR52733.2024.01692 *(online, mlanthology; confirm)*. Authors and title match. |
| `ravi2024sam2` | 172 supply | **incomplete** | `Ravi, Nikhila and others` → all **18 authors** (list in 3.2). The PDF is arXiv:2408.00714**v2** (28 Oct 2024; results are SAM 2.1). It was published at **ICLR 2025** *(online: ICLR proceedings / mlanthology; no page numbers)*. Cite ICLR 2025 or keep arXiv, but give the full author list. |
| `kuffner2000rrtconnect` | 171 supply | correct, could be fuller | Pages 995-1001 and 2000 match (also confirmed by ref. [7] of the Luna PDF). Add **vol. 2**, San Francisco, doi:10.1109/ROBOT.2000.844730 *(online)*. The author is printed as "James J. Kuffner, Jr." → `Kuffner, Jr., James J.`. The supply PDF is the **submitted draft**, not the proceedings version. |
| `lugaresi2019mediapipe` | 168 supply | **correct** | All 14 authors in order, the title, arXiv:1906.08172 and 2019 match the PDF. The comment "VERIFY ... written from memory" can be dropped for this entry. **But the sentence citing it (ch3 l.158) needs MediaPipe Hands + the Gesture Recognizer docs** (Part 2.3). |
| `drost2010ppf` | 169 supply (= 103) | correct | Pages 998-1005 and 2010 match. Add doi:10.1109/CVPR.2010.5540108 (PDF metadata). Booktitle per the PDF: "2010 IEEE Computer Society Conference on Computer Vision and Pattern Recognition". |
| `birdal2015ppf` | 170 supply (= 080, 104) | correct | Pages 527-535 and 2015 match. Add doi:10.1109/3DV.2015.65 (printed). |
| `zhang2022fricp` | 108 | correct (cannot be checked on the PDF) | The PDF is arXiv:2007.07627v3 (2023). The bib's TPAMI 44(7):3450-3466, 2022 is confirmed *(online: Cardiff ORCA record)*; add doi:10.1109/TPAMI.2021.3054619 *(online)*. |
| `klemm2015rrtstarconnect` | 182 (weld_generator only) | correct | The 7 authors, title, ROBIO 2015 and pp. 1670-1677 match (page numbers read from the PDF). Add doi:10.1109/ROBIO.2015.7419012 (metadata) and the address Zhuhai, China. |
| `song2025lwsnet` | 122 | correct | Add doi:10.1016/j.measurement.2024.116290. |
| `yi2026seam` | 127 (= 047, lit-ransac) | correct | Add doi:10.1016/j.autcon.2026.106792. |
| `geng2023seam` | 135 | correct | Add doi:10.1016/j.rcim.2022.102433. |
| `zhu2025sanet` | 125 | correct | Optionally expand the initials to Zhijian Zhu, Haoran Gu, Zhao Yang, Lijie Zhao, Guoli Song, Qinghui Wang, and add number = 20. |
| `wang2024ppfseam` | 128 (= lit-ppf) | correct | Vol 14, art. 21137, 2024 and the DOI match. The key's "ppf" is misleading: the method is **orthogonal plane intersection** + Faster R-CNN, not point-pair features. The key may stay, but do not describe the method as PPF in Ch2/Ch4. |
| `wang2026pipes` | 118 (= lit-pcaslice) | correct | Online-first article (accepted 26 May 2026), no volume or pages yet. Add `note = {Online first}` and update before submission. |

### 3.2 Corrected BibTeX for the 21 entries

Paste-ready. The keys are unchanged except `luna2013aps`, which replaces `luo2014aps`.

```bibtex
% ---------------------------------------------------------------- weld seam perception
@article{song2025lwsnet,
  author  = {Song, Limei and Wang, Hongli and Zhang, Yuezhe and Qiao, Zhi and Han, Fengyi},
  title   = {{LWSNet}: A lightweight network for automated welding point cloud segmentation},
  journal = {Measurement},
  volume  = {243},
  pages   = {116290},
  year    = {2025},
  doi     = {10.1016/j.measurement.2024.116290}
}

@article{yi2026seam,
  author  = {Yi, Jinxin and Kong, Xuan and Tang, Hao and Zhang, Jie and Chen, Zhenming and Deng, Lu},
  title   = {Weld seam extraction and path generation for robotic welding of steel structures based on {3D} vision},
  journal = {Automation in Construction},
  volume  = {183},
  pages   = {106792},
  year    = {2026},
  doi     = {10.1016/j.autcon.2026.106792}
}

@article{lin2026coating,
  author  = {Lin, Zhaohui and Dong, Haonan and Kong, Weijie and Huang, Haoran and Chen, I-Ming and Dong, Huixu},
  title   = {A Coarse-to-Fine Multimodal Detection Framework Based on Deep Learning for Robotic Coating Tasks},
  journal = {IEEE/ASME Transactions on Mechatronics},
  volume  = {31},
  number  = {1},
  pages   = {639--650},
  month   = feb,
  year    = {2026},
  doi     = {10.1109/TMECH.2025.3595263}
}

@article{zhu2025sanet,
  author  = {Zhu, Zhijian and Gu, Haoran and Yang, Zhao and Zhao, Lijie and Song, Guoli and Wang, Qinghui},
  title   = {{SANet}: A Pure Vision Strip-Aware Network with {PSSCA} and Multistage Fusion for Weld Seam Detection},
  journal = {Applied Sciences},
  volume  = {15},
  number  = {20},
  pages   = {11296},
  year    = {2025},
  doi     = {10.3390/app152011296}
}

@article{geng2023seam,
  author  = {Geng, Yusen and Lai, Min and Tian, Xincheng and Xu, Xiaolong and Jiang, Yong and Zhang, Yuankai},
  title   = {A novel seam extraction and path planning method for robotic welding of medium-thickness plate structural parts based on {3D} vision},
  journal = {Robotics and Computer-Integrated Manufacturing},
  volume  = {79},
  pages   = {102433},
  year    = {2023},
  doi     = {10.1016/j.rcim.2022.102433}
}

% ---------------------------------------------------------------- the benchmarked methods (WeldSet)
@article{wang2026pipes,
  author  = {Wang, Tianqi and Liang, Yifan and Liu, Xinqi and Wang, Zhigang},
  title   = {{3D} vision-based intersecting pipe welding path planning},
  journal = {Welding in the World},
  year    = {2026},
  doi     = {10.1007/s40194-026-02509-9},
  note    = {Online first}
}

@article{zhang_lobb,
  author  = {Zhang, Yuankai and Geng, Yusen and Tian, Xincheng and Sun, Yujie and Xu, Xiaolong},
  title   = {A novel weld seam extraction method with semantic segmentation and point cloud feature for irregular structure workpieces},
  journal = {Robotics and Computer-Integrated Manufacturing},
  volume  = {95},
  pages   = {102987},
  year    = {2025},
  doi     = {10.1016/j.rcim.2025.102987}
}

@article{fang2024modelreg,
  author  = {Fang, Weihua and Tian, Xincheng},
  title   = {A novel model-based welding trajectory planning method for identical structural workpieces},
  journal = {Robotics and Computer-Integrated Manufacturing},
  volume  = {89},
  pages   = {102772},
  year    = {2024},
  doi     = {10.1016/j.rcim.2024.102772}
}

@article{wang2024ppfseam,
  author  = {Wang, Shengbo and Li, Zengxu and Chen, Guodong and Yue, Yaobin},
  title   = {Weld seam object detection system based on the fusion of {2D} images and {3D} point clouds using interpretable neural networks},
  journal = {Scientific Reports},
  volume  = {14},
  pages   = {21137},
  year    = {2024},
  doi     = {10.1038/s41598-024-71989-w}
}

@article{li2026quadric,
  author  = {Li, Dongmin and Wang, Yu and Wang, Zhengyong},
  title   = {Automatic recognition on impeller shape and weld seam based on normal of point clouds and {PCA}},
  journal = {Journal of Mechanical Science and Technology},
  volume  = {40},
  number  = {3},
  pages   = {2039--2046},
  year    = {2026},
  doi     = {10.1007/s12206-026-0243-8}
}

% venue/pages/doi from the IROS 2024 record (the PDF in the folder is arXiv:2408.10710v1)
@inproceedings{wei_regiongrow,
  author    = {Wei, Pengkun and Cheng, Shuo and Li, Dayou and Song, Ran and Zhang, Yipeng and Zhang, Wei},
  title     = {Coarse-to-Fine Detection of Multiple Seams for Robotic Welding},
  booktitle = {IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  address   = {Abu Dhabi, UAE},
  pages     = {7138--7144},
  year      = {2024},
  doi       = {10.1109/IROS58592.2024.10802546}
}

% ---------------------------------------------------------------- recognition, pose, registration
@inproceedings{drost2010ppf,
  author    = {Drost, Bertram and Ulrich, Markus and Navab, Nassir and Ilic, Slobodan},
  title     = {Model globally, match locally: Efficient and robust {3D} object recognition},
  booktitle = {IEEE Computer Society Conference on Computer Vision and Pattern Recognition (CVPR)},
  pages     = {998--1005},
  year      = {2010},
  doi       = {10.1109/CVPR.2010.5540108}
}

@inproceedings{birdal2015ppf,
  author    = {Birdal, Tolga and Ilic, Slobodan},
  title     = {Point pair features based object detection and pose estimation revisited},
  booktitle = {International Conference on {3D} Vision (3DV)},
  pages     = {527--535},
  year      = {2015},
  doi       = {10.1109/3DV.2015.65}
}

% pages from CVF open access; the PDF in the folders is arXiv:2312.08344v2
@inproceedings{wen2024foundationpose,
  author    = {Wen, Bowen and Yang, Wei and Kautz, Jan and Birchfield, Stan},
  title     = {{FoundationPose}: Unified {6D} Pose Estimation and Tracking of Novel Objects},
  booktitle = {IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR)},
  pages     = {17868--17879},
  year      = {2024},
  doi       = {10.1109/CVPR52733.2024.01692}
}

% Published version (ICLR 2025). If you prefer the preprint, use
% journal = {arXiv preprint arXiv:2408.00714} and year = {2024} with the same author list.
@inproceedings{ravi2024sam2,
  author    = {Ravi, Nikhila and Gabeur, Valentin and Hu, Yuan-Ting and Hu, Ronghang and Ryali, Chaitanya and Ma, Tengyu and Khedr, Haitham and R{\"a}dle, Roman and Rolland, Chloe and Gustafson, Laura and Mintun, Eric and Pan, Junting and Alwala, Kalyan Vasudev and Carion, Nicolas and Wu, Chao-Yuan and Girshick, Ross and Doll{\'a}r, Piotr and Feichtenhofer, Christoph},
  title     = {{SAM} 2: Segment Anything in Images and Videos},
  booktitle = {International Conference on Learning Representations (ICLR)},
  year      = {2025}
}

@article{zhang2022fricp,
  author  = {Zhang, Juyong and Yao, Yuxin and Deng, Bailin},
  title   = {Fast and Robust Iterative Closest Point},
  journal = {IEEE Transactions on Pattern Analysis and Machine Intelligence},
  volume  = {44},
  number  = {7},
  pages   = {3450--3466},
  year    = {2022},
  doi     = {10.1109/TPAMI.2021.3054619}
}

% ---------------------------------------------------------------- planning
@inproceedings{kuffner2000rrtconnect,
  author    = {Kuffner, Jr., James J. and LaValle, Steven M.},
  title     = {{RRT-Connect}: An efficient approach to single-query path planning},
  booktitle = {IEEE International Conference on Robotics and Automation (ICRA)},
  address   = {San Francisco, CA, USA},
  volume    = {2},
  pages     = {995--1001},
  year      = {2000},
  doi       = {10.1109/ROBOT.2000.844730}
}

@inproceedings{klemm2015rrtstarconnect,
  author    = {Klemm, Sebastian and Oberl{\"a}nder, Jan and Hermann, Andreas and Roennau, Arne and Schamm, Thomas and Z{\"o}llner, J. Marius and Dillmann, R{\"u}diger},
  title     = {{RRT*-Connect}: Faster, asymptotically optimal motion planning},
  booktitle = {IEEE International Conference on Robotics and Biomimetics (ROBIO)},
  address   = {Zhuhai, China},
  pages     = {1670--1677},
  year      = {2015},
  doi       = {10.1109/ROBIO.2015.7419012}
}

% REPLACES luo2014aps (that key named a different paper). Update \cite in ch2 l.68 and ch3 l.545.
@inproceedings{luna2013aps,
  author    = {Luna, Ryan and {\c{S}}ucan, Ioan A. and Moll, Mark and Kavraki, Lydia E.},
  title     = {Anytime solution optimization for sampling-based motion planning},
  booktitle = {IEEE International Conference on Robotics and Automation (ICRA)},
  address   = {Karlsruhe, Germany},
  pages     = {5053--5059},
  year      = {2013},
  doi       = {10.1109/ICRA.2013.6631301}
}

@article{liu2023rcim,
  author  = {Liu, Yan and Tang, Qiu and Tian, Xincheng and Yang, Shuai},
  title   = {A novel offline programming approach of robot welding for multi-pipe intersection structures based on {NSGA-II} and measured {3D} point-clouds},
  journal = {Robotics and Computer-Integrated Manufacturing},
  volume  = {83},
  pages   = {102549},
  year    = {2023},
  doi     = {10.1016/j.rcim.2023.102549}
}

% ---------------------------------------------------------------- interaction (framework paper; correct as is)
@article{lugaresi2019mediapipe,
  author  = {Lugaresi, Camillo and Tang, Jiuqiang and Nash, Hadon and McClanahan, Chris and Uboweja, Esha and Hays, Michael and Zhang, Fan and Chang, Chuo-Ling and Yong, Ming Guang and Lee, Juhyun and Chang, Wan-Teh and Hua, Wei and Georg, Manfred and Grundmann, Matthias},
  title   = {{MediaPipe}: A Framework for Building Perception Pipelines},
  journal = {arXiv preprint arXiv:1906.08172},
  year    = {2019}
}
```

If the thesis also wants Luo & Hauser's empirical study of optimal planners, keep it under a new
key (for example `luo2014empirical`) and verify its fields. It is **not** in the folders and is
not the APS reference.

### 3.3 Entries without a PDF in the folders (status only)

These cannot be checked against a PDF here. Fetch the PDFs listed in Part 4.

| Key | Status |
|---|---|
| `besl1992icp` | Fields agree with the standard record (TPAMI 14(2):239-256, 1992); add doi:10.1109/34.121791 *(memory)*. |
| `chen1992p2plane` | IVC 10(3):145-155, 1992 agrees; doi:10.1016/0262-8856(92)90066-C *(memory)*. |
| `tsai1989handeye` | IEEE TRA 5(3):345-358, 1989 agrees; doi:10.1109/70.34770 *(memory)*. |
| `park1994handeye` | IEEE TRA 10(5):717-721, 1994 agrees; doi:10.1109/70.326576 *(memory)*. |
| `keselman2017realsense` | Add **pages 1267-1276** and doi:10.1109/CVPRW.2017.167 *(online: mlanthology)*. Also arXiv:1705.05548. |
| `karaman2011rrtstar` | IJRR 30(7):846-894 agrees (also cited so in the Luna PDF); doi:10.1177/0278364911406761 *(memory)*. |
| `sucan2012ompl` | RAM 19(4):72-82, Dec 2012 agrees with ref. [30] of the Luna PDF; doi:10.1109/MRA.2012.2205651 *(memory)*. |
| `kunz2012totg` | RSS VIII, Sydney, 2012; add doi:10.15607/RSS.2012.VIII.027 *(online: roboticsproceedings.org)*. No page numbers are given there. |
| `casiez2012oneeuro` | CHI '12, pp. 2527-2530, agrees; doi:10.1145/2207676.2208639 *(memory)*. |
| `chaumette2006vs` | IEEE RAM 13(4):82-90, 2006 agrees; doi:10.1109/MRA.2006.250573 *(memory)*. |
| `wampler1986dls` | IEEE TSMC 16(1):93-101, 1986 agrees; doi:10.1109/TSMC.1986.289285 *(memory)*. |
| `forney1973viterbi` | Proc. IEEE 61(3):268-278, 1973 agrees; doi:10.1109/PROC.1973.9030 *(memory)*. |
| `iso5817` | The title is the 2023 edition's title. Check it against the PDF in `notes/` (ISO PDFs mentioned in THESIS_PLAN). |
| `iso9692` | **The title is truncated** after "Part 1". The full title is "Welding and allied processes - Types of joint preparation - Part 1: Manual metal arc welding, gas-shielded metal arc welding, gas welding, TIG welding and beam welding of steels" *(memory; check the cover)*. |
| `own_weldset` | Placeholder. |

### 3.4 Pool papers worth adding to `thesis.bib`

These are all from the PDFs. The fields are as printed unless marked.

```bibtex
% the original LOBB descriptor used by the 'lobb' baseline (zhang_lobb applies it)
@article{zhang2025lobbdesc,
  author  = {Zhang, Yuankai and Geng, Yusen and Tian, Xincheng and Zheng, Fuquan and Jiang, Yong and Lai, Min},
  title   = {A Feature Extraction Approach Over Workpiece Point Clouds for Robotic Welding},
  journal = {IEEE Transactions on Automation Science and Engineering},
  volume  = {22},
  pages   = {75--84},
  year    = {2025},
  doi     = {10.1109/TASE.2023.3345868}
}

% CAD-to-scan ICP to compute the actual gap geometry for welding (Ch3 fit-up gap)
@article{kuss2017gap,
  author  = {Kuss, Alexander and Dietz, Thomas and Spenrath, Felix and Verl, Alexander},
  title   = {Automated planning of robotic {MAG} welding based on adaptive gap model},
  journal = {Procedia CIRP},
  volume  = {62},
  pages   = {612--617},
  year    = {2017},
  note    = {10th CIRP Conference on Intelligent Computation in Manufacturing Engineering (CIRP ICME '16)},
  doi     = {10.1016/j.procir.2016.07.008}
}

% analytic intersection-curve model fitted to pipe clouds (Ch3 curved seams, Ch4 curved seams by construction)
@article{geng2024intersection,
  author  = {Geng, Yusen and Zhang, Yuankai and Tian, Xincheng and Zhou, Lelai},
  title   = {A novel {3D} vision-based robotic welding path extraction method for complex intersection curves},
  journal = {Robotics and Computer-Integrated Manufacturing},
  volume  = {87},
  pages   = {102702},
  year    = {2024},
  doi     = {10.1016/j.rcim.2023.102702}
}

% gesture-guided scanning + seam extraction (folder copy is arXiv:2011.11951; journal fields online, PolyU repository)
@article{zhou2021seam,
  author  = {Zhou, Peng and Peng, Rui and Xu, Maggie and Wu, Victor and Navarro-Alarcon, David},
  title   = {Path Planning With Automatic Seam Extraction Over Point Cloud Models for Robotic Arc Welding},
  journal = {IEEE Robotics and Automation Letters},
  volume  = {6},
  number  = {3},
  pages   = {5002--5009},
  year    = {2021},
  doi     = {10.1109/LRA.2021.3070828}
}

@inproceedings{ahmed2018edge,
  author    = {Ahmed, Syeda Mariam and Tan, Yan Zhi and Chew, Chee Meng and Al Mamun, Abdullah and Wong, Fook Seng},
  title     = {Edge and Corner Detection for Unorganized {3D} Point Clouds with Application to Robotic Welding},
  booktitle = {IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  address   = {Madrid, Spain},
  pages     = {7350--7355},
  year      = {2018}
}

@article{wang2025pcwelding,
  author  = {Wang, Hui and Rong, Youmin and Xu, Jiajun and Huang, Yu and Zhang, Guojun},
  title   = {Application and trends of point cloud in intelligent welding: State of the art review},
  journal = {Journal of Manufacturing Systems},
  volume  = {79},
  pages   = {48--72},
  year    = {2025},
  doi     = {10.1016/j.jmsy.2025.01.001}
}

@inproceedings{schleth2018localization,
  author    = {Schleth, Gesine and Kuss, Alexander and Kraus, Werner},
  title     = {Workpiece localization methods for robotic welding -- a review},
  booktitle = {ISR 2018; 50th International Symposium on Robotics},
  address   = {Munich, Germany},
  pages     = {50--55},
  year      = {2018}
}

@article{lowth2015gaps,
  author  = {Lowth, Stewart and Axinte, Dragos A. and Sun, Wei},
  title   = {An investigation into the influence of inter-component assembly gaps upon the geometry of tack welded annular structures},
  journal = {CIRP Annals - Manufacturing Technology},
  volume  = {64},
  pages   = {1--4},
  year    = {2015},
  doi     = {10.1016/j.cirp.2015.04.043}
}

@article{tomkow2020tacks,
  author  = {Tomk{\'o}w, Jacek and Sobota, Konrad and Krajewski, S{\l}awomir},
  title   = {Influence of Tack Welds Distribution and Welding Sequence on the Angular Distortion of {TIG} Welded Joint},
  journal = {Facta Universitatis, Series: Mechanical Engineering},
  volume  = {18},
  number  = {4},
  pages   = {611--621},
  year    = {2020},
  doi     = {10.22190/FUME200520044T}
}

@article{mondal2015tacking,
  author  = {Mondal, Arpan Kumar and Biswas, Pankaj and Bag, Swarup},
  title   = {Influence of Tacking Sequence on Residual Stress and Distortion of Single Sided Fillet Submerged Arc Welded Joint},
  journal = {Journal of Marine Science and Application},
  volume  = {14},
  pages   = {250--260},
  year    = {2015},
  doi     = {10.1007/s11804-015-1320-z}
}

@article{jia2025tack,
  author  = {Jia, Bingqi and Pan, Haihong and Zhang, Lei and Yang, Yifan and Chen, Huaxin and Chen, Lin},
  title   = {Robotic Tack Welding Path and Trajectory Optimization Using an {LF-IWOA}},
  journal = {Actuators},
  volume  = {14},
  pages   = {287},
  year    = {2025},
  doi     = {10.3390/act14060287}
}

@article{chen2023tacked,
  author  = {Chen, Shengfeng and Yang, Dezhi and Liu, Jian and Tian, Qi and Zhou, Feitao},
  title   = {Automatic weld type classification, tacked spot recognition and weld {ROI} determination for robotic welding based on modified {YOLOv5}},
  journal = {Robotics and Computer-Integrated Manufacturing},
  volume  = {81},
  pages   = {102490},
  year    = {2023},
  doi     = {10.1016/j.rcim.2022.102490}
}

@article{denayer2024registration,
  author  = {Denayer, Menthy and De Winter, Joris and Bernardes, Evandro and Vanderborght, Bram and Verstraten, Tom},
  title   = {Comparison of Point Cloud Registration Techniques on Scanned Physical Objects},
  journal = {Sensors},
  volume  = {24},
  pages   = {2142},
  year    = {2024},
  doi     = {10.3390/s24072142}
}

@inproceedings{rusu2009fpfh,
  author    = {Rusu, Radu Bogdan and Blodow, Nico and Beetz, Michael},
  title     = {Fast Point Feature Histograms ({FPFH}) for {3D} registration},
  booktitle = {IEEE International Conference on Robotics and Automation (ICRA)},
  address   = {Kobe, Japan},
  pages     = {3212--3217},
  year      = {2009},
  doi       = {10.1109/ROBOT.2009.5152473}
}

@article{yang2021teaser,
  author  = {Yang, Heng and Shi, Jingnan and Carlone, Luca},
  title   = {{TEASER}: Fast and Certifiable Point Cloud Registration},
  journal = {IEEE Transactions on Robotics},
  volume  = {37},
  number  = {2},
  pages   = {314--333},
  year    = {2021},
  doi     = {10.1109/TRO.2020.3033695}
}

@inproceedings{avetisyan2019scan2cad,
  author    = {Avetisyan, Armen and Dahnert, Manuel and Dai, Angela and Savva, Manolis and Chang, Angel X. and Nie{\ss}ner, Matthias},
  title     = {{Scan2CAD}: Learning {CAD} Model Alignment in {RGB-D} Scans},
  booktitle = {IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR)},
  pages     = {2614--2623},
  year      = {2019}
}

@inproceedings{wen2023bundlesdf,
  author    = {Wen, Bowen and Tremblay, Jonathan and Blukis, Valts and Tyree, Stephen and M{\"u}ller, Thomas and Evans, Alex and Fox, Dieter and Kautz, Jan and Birchfield, Stan},
  title     = {{BundleSDF}: Neural 6-{DoF} Tracking and {3D} Reconstruction of Unknown Objects},
  booktitle = {IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR)},
  pages     = {606--617},
  year      = {2023},
  doi       = {10.1109/CVPR52729.2023.00066}
}

@inproceedings{lin2024sam6d,
  author    = {Lin, Jiehong and Liu, Lihua and Lu, Dekun and Jia, Kui},
  title     = {{SAM-6D}: Segment Anything Model Meets Zero-Shot {6D} Object Pose Estimation},
  booktitle = {IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR)},
  pages     = {27906--27916},
  year      = {2024},
  doi       = {10.1109/CVPR52733.2024.02636}
}

% two preprints that combine SAM2 and FoundationPose like the cell (cite as related work, arXiv)
@article{deng2024superpose,
  author  = {Deng, Yu and Xue, Jiahong and Cao, Teng and Zhang, Yingxin and Wen, Lanxi and Chen, Yiyang},
  title   = {{SuperPose}: Improved {6D} Pose Estimation with Robust Tracking and Mask-Free Initialization},
  journal = {arXiv preprint arXiv:2409.19986},
  year    = {2024}
}

@article{liang2025dynamicpose,
  author  = {Liang, Tingbang and Zeng, Yixin and Xie, Jiatong and Zhou, Boyu},
  title   = {{DynamicPose}: Real-time and Robust {6D} Object Pose Tracking for Fast-Moving Cameras and Objects},
  journal = {arXiv preprint arXiv:2508.11950},
  year    = {2025}
}
```

Other good candidates from the pool, with fields in the Part 1 rows:

- Xiao et al. 2026 (tag 010): YOLOv5 + Super4PCS + AA-ICP to the workpiece model, 0.62 mm.
- Tan et al. 2022 (tag 020): tubular joint pose and fit-up.
- Shah et al. (tag 028): CAD assemblies fitted to scans.
- Hill et al. 2025 (tag 054): eye-in-hand RGB-D + ICP weld paths.
- Li et al. 2025 (tag 053): FPFH-RANSAC-ICP to CAD for teaching-free welding.
- Wüthrich et al. 2013, se(3)-TrackNet 2020, BundleTrack 2021 (tags 035, 037, 032): tracking
  background.
- Koide et al. 2021 (tag 044): VGICP.
- Bouaziz et al. 2013 (tag 109): Sparse ICP.
- Wu & Kim 2020 (tag 022): distortion review.

## Part 4 - Topics the pool does not cover yet

The table lists what the thesis needs a citation for and has **no PDF in any of the folders**.

- "**In bib**" means an entry already exists (written from memory), so only the PDF has to be
  fetched and the entry checked against it (see 3.3).
- The candidate references are the standard ones for the topic. They are written from general
  knowledge, so **verify every field against the PDF** you fetch.

| # | Topic (where it is used) | In bib? | Candidate references to fetch |
|---|---|---|---|
| 1 | **Hand-eye calibration** AX = XB (Ch2 calibration; Ch3 `eq:handeye`) | yes: `tsai1989handeye`, `park1994handeye` (no PDFs) | Tsai & Lenz 1989 (IEEE TRA 5(3)); Park & Martin 1994 (IEEE TRA 10(5)). Also useful: Horaud & Dornaika 1995 (IJRR 14(3)), Daniilidis 1999 dual quaternions (IJRR 18(3)). If OpenCV's `calibrateHandEye` was used, say which of its methods. |
| 2 | **Camera intrinsics and the calibration target** (Ch3 frames and calibration) | no | Zhang 2000, "A flexible new technique for camera calibration" (TPAMI 22(11)). Garrido-Jurado et al. 2014, ArUco (Pattern Recognition 47(6)), if ChArUco/ArUco boards are used. |
| 3 | **Stereo depth error of the RealSense D435i** ($\sigma_z\propto z^2$; Ch2 calibration; Ch3 `eq:depth_error`) | yes: `keselman2017realsense` (no PDF) | Keselman et al., CVPRW 2017 (arXiv:1705.05548). Intel's "Depth Camera D400 Series datasheet" and the white paper on tuning D400 cameras for best performance (Grunnet-Jepsen et al.), which give the RMS-error formula $\text{RMS}\approx z^2\,\text{subpixel}/(f\,b)$. An empirical depth-camera evaluation, e.g. Halmetschlager-Funek et al., IEEE RAM 2019. |
| 4 | **Point-to-plane ICP and its linearisation; robust kernels** (Ch2 registration; Ch3 `eq:p2plane`) | yes: `chen1992p2plane`, `besl1992icp` (no PDFs); `zhang2022fricp` has a PDF | Chen & Medioni 1992 (IVC 10(3)). Low 2004, "Linear least-squares optimization for point-to-plane ICP surface registration" (UNC TR04-004). Rusinkiewicz & Levoy 2001, "Efficient variants of the ICP algorithm" (3DIM). Holland & Welsch 1977, for the Welsch weight. Segal et al. 2009, Generalized-ICP (RSS), for context. The pool has VGICP, Sparse ICP and FRICP. |
| 5 | **Observability / degeneracy of a registration** (Ch3 `eq:observability`, multi-view) | no | Gelfand et al. 2003, "Geometrically stable sampling for the ICP algorithm" (3DIM). Censi 2007, "An accurate closed-form estimate of ICP's covariance" (ICRA). Zhang, Kaess & Singh 2016, "On degeneracy of optimization-based state estimation problems" (ICRA). |
| 6 | **Rotation averaging and swing-twist decomposition** (Ch3 stationary refinement, `eq:swing_twist`) | no | Hartley et al. 2013, "Rotation averaging" (IJCV 103(3)), for the chordal mean. Markley et al. 2007, "Averaging quaternions" (J. Guid. Control Dyn. 30(4)), as cited in Birdal & Ilic. Dobrowolski 2015, "Swing-twist decomposition in Clifford algebra" (arXiv:1506.05481). Baerlocher & Boulic 2001, on the swing-twist parametrisation of ball-and-socket joints. |
| 7 | **Visual servoing and the camera-aim law** (Ch3 `eq:aim_law`) | yes: `chaumette2006vs` (no PDF) | Chaumette & Hutchinson 2006, Part I (IEEE RAM 13(4)) and 2007, Part II (RAM 14(1)). Hutchinson, Hager & Corke 1996 tutorial (IEEE TRA 12(5)). |
| 8 | **Damped least-squares IK** (Ch3 `eq:aim_dls`) | yes: `wampler1986dls` (no PDF) | Wampler 1986 (IEEE TSMC 16(1)). Nakamura & Hanafusa 1986, singularity-robust inverse (ASME JDSMC 108(3)). Buss 2004, the DLS tutorial report, as an accessible survey. |
| 9 | **One-Euro filter** (Ch3 gesture/aim smoothing) | yes: `casiez2012oneeuro` (no PDF) | Casiez, Roussel & Vogel, CHI 2012. |
| 10 | **Viterbi / dynamic programming** (Ch3 `eq:roll_dp`) | yes: `forney1973viterbi` (no PDF) | Forney 1973 (Proc. IEEE 61(3)). Viterbi 1967 (IEEE TIT 13(2)). Bellman 1957, *Dynamic Programming*. |
| 11 | **MediaPipe Hands and the Gesture Recognizer** (Ch3 gesture interface) | **no** | Zhang et al. 2020, "MediaPipe Hands: On-device Real-time Hand Tracking" (arXiv:2006.10214; CVPR 2020 workshop CV4ARVR) *(record verified online)*. The Gesture Recognizer task guide / model card (Google AI Edge), cited as `@misc` with URL and access date. Optionally Bazarevsky et al. 2019, BlazeFace (arXiv:1907.05047), the detector family of BlazePalm. |
| 12 | **SAM (v1) and the Hiera backbone** (Ch2 segmentation; Ch3 SAM2) | **no** | Kirillov et al. 2023, "Segment Anything" (ICCV). The SAM 2 prompt encoder and decoder are "identical to SAM". Ryali et al. 2023, "Hiera: A hierarchical vision transformer without the bells-and-whistles" (ICML). |
| 13 | **"Going Further with Point Pair Features"** and later PPF variants (Ch2 pose) | **no** | Hinterstoisser, Lepetit, Rajkumar, Konolige 2016 (ECCV). Vidal, Lin & Martí 2018, the improved PPF that was best in BOP 2018. Hodaň et al. 2018, "BOP: Benchmark for 6D object pose estimation" (ECCV), for the ADD/AR metrics also quoted from FoundationPose. |
| 14 | **Render-and-compare background for FoundationPose** (Ch2 pose) | **no** | Labbé et al. 2022, MegaPose (CoRL). Nguyen et al. 2023, CNOS (ICCVW), the detector FoundationPose uses, which the cell replaces with SAM2. Laine et al. 2020, nvdiffrast (ACM TOG 39(6)). The pool has BundleSDF, se(3)-TrackNet and BundleTrack. |
| 15 | **Path shortcutting and path hybridisation** (Ch2/Ch3 APS) | **no** | Geraerts & Overmars 2007, "Creating high-quality paths for motion planning" (IJRR 26(8)). Raveh, Enosh & Halperin 2011, "A little more, a lot better: improving path quality by a path-merging algorithm" (IEEE T-RO 27(2)). Both are cited by Luna et al. as [10] and [11]. |
| 16 | **Time-optimal path parameterisation (TOTG)** (Ch3 transit timing) | yes: `kunz2012totg` (no PDF) | Kunz & Stilman 2012 (RSS VIII, doi:10.15607/RSS.2012.VIII.027 *(online)*). Optionally Bobrow, Dubowsky & Gibson 1985 (IJRR 4(3)) or Pham 2014, TOPP-RA's predecessor TOPP (IEEE T-RO 30(6)), for the classical problem. |
| 17 | **ROS 2, MoveIt (2), collision libraries** (Ch3 implementation; Ch5 software) | **no** | Macenski et al. 2022, "Robot Operating System 2: Design, architecture, and uses in the wild" (Science Robotics 7(66)). Coleman et al. 2014, MoveIt! case study (JOSER 5(1)), or the MoveIt 2 docs. Pan, Chitta & Manocha 2012, FCL (ICRA), if FCL/hpp-fcl is used. The pool has CAPT (point-cloud collision checking). |
| 18 | **Force / impedance control for the force-gated pen descent** (Ch3 force-gated marking) | **no** | Hogan 1985, "Impedance control: an approach to manipulation, Parts I-III" (ASME JDSMC 107(1)). Villani & De Schutter, "Force control", Springer Handbook of Robotics (ch. 9 in the 2016 edition). The UR e-Series force/torque sensing manual, for the sensor used. |
| 19 | **Welding standards: tack-weld rules and tolerances** (Ch2 standards; Ch4 tack rule; Ch3 `eq:iso_gap`) | yes: `iso5817`, `iso9692` (check titles, see 3.3) | ISO 5817:2023 (quality levels; imperfection 617 root gap). ISO 9692-1:2013 (joint preparation, full title). ISO 2553:2019 (weld symbols). ISO 6947:2019 (welding positions). EN 1090-2 (execution of steel structures; contains tack-weld requirements). AWS D1.1/D1.1M (Structural Welding Code - Steel, tack welds). EN ISO 13920 (general tolerances for welded constructions). The tack rule's length/spacing values should cite whichever of these it follows. |
| 20 | **Tack sequence / distortion, classic sources** (Ch2 tacks; Ch3 ordering; Ch7) | **no** (pool has 022, 060, 061, 063, 064, 069 and the AI reports) | Tsai, Park & Cheng 1999, "Welding distortion of a thin-plate panel structure" (Welding Journal 78(5)), the joint-rigidity method recommended by the AI reports. Ueda & Murakawa (JWRI), inherent strain. Deng, Murakawa & Liang 2007, large-structure distortion simulation (CMAME 196). Masubuchi 1980, *Analysis of Welded Structures*. Radaj 1992, *Heat Effects of Welding*. Rosenthal 1946, the moving point heat source (Trans. ASME 68), if a heat-accumulation term is used. |
| 21 | **Synthetic data, domain randomisation and depth-sensor simulation** (Ch4 WeldSet sensor model and rendering) | **no** | Tobin et al. 2017, "Domain randomization" (IROS). Denninger et al. 2023, BlenderProc2 (JOSS 8(82)), if used. An active-stereo / structured-light noise model, e.g. Handa et al. 2014, ICL-NUIM noise (ICRA), or Landau et al. 2016, Kinect simulation (IEEE Trans. Cybern.). The pool has only Behnen et al. 2026 (sheet-metal synthetic data) and the FoundationPose / se(3)-TrackNet synthetic pipelines. |
| 22 | **Geometric primitives used by the seam baselines** (Ch4 evaluation protocol) | **no** | Fischler & Bolles 1981, RANSAC (CACM 24(6)). Torr & Zisserman 2000, MSAC/MLESAC (CVIU 78(1)), used by tags 118 and 155. Schnabel, Wahl & Klein 2007, efficient RANSAC for shape detection (CGF 26(2)). Rabbani et al. 2006, smoothness-constrained region growing (ISPRS). Hoppe et al. 1992, PCA normals (SIGGRAPH). Comaniciu & Meer 2002, mean shift (TPAMI 24(5)), for the LOBB `mean_shift` arm. Qi et al. 2017, PointNet++ (NeurIPS), the base of LWSNet. |
| 23 | **Curve-accuracy metrics** (Ch4 metrics; Ch6) | **no** | Huttenlocher, Klanderman & Rucklidge 1993, Hausdorff distance (TPAMI 15(9)). A Chamfer-distance reference, e.g. Barrow et al. 1977 or Fan, Su & Guibas 2017, if Chamfer is used. |
| 24 | **Bézier / slerp transits** (Ch3 `eq:cartesian_bow`) | **no** | Shoemake 1985, "Animating rotation with quaternion curves" (SIGGRAPH), for slerp. Farin, *Curves and Surfaces for CAGD*, or Piegl & Tiller, *The NURBS Book*, for Bézier and B-splines (also used by tags 105 and 158). |

**Fetch first** (they support sentences that are already drafted):

1. Luna 2013. The PDF is in hand; fix the bib.
2. MediaPipe Hands + Gesture Recognizer docs.
3. Kirillov 2023 (SAM).
4. Hinterstoisser 2016, if "going further" is to be mentioned.
5. Chen & Medioni 1992 and Low 2004.
6. Tsai & Lenz 1989 and Park & Martin 1994.
7. Keselman 2017 + the Intel depth white paper.
8. Kunz & Stilman 2012.
9. Geraerts & Overmars 2007 and Raveh et al. 2011.
10. The ISO/EN tack-weld clauses.
11. Tsai et al. 1999 (joint rigidity).

## Sources consulted online

These were used only for fields that the PDFs in the folders do not print.

- Luna et al. 2013, pages and DOI: [Kavraki Lab publication page](https://kavrakilab.org/publications/luna-sucan2013anytime-solution-optimization.html)
- FoundationPose, CVPR 2024 pages: [CVF open access](https://openaccess.thecvf.com/content/CVPR2024/html/Wen_FoundationPose_Unified_6D_Pose_Estimation_and_Tracking_of_Novel_Objects_CVPR_2024_paper.html), [mlanthology](https://mlanthology.org/cvpr/2024/wen2024cvpr-foundationpose)
- SAM 2 at ICLR 2025: [ICLR proceedings](https://proceedings.iclr.cc/paper_files/paper/2025/hash/45c1f6a8cbf2da59ebf2c802b4f742cd-Abstract-Conference.html), [mlanthology BibTeX](https://mlanthology.org/iclr/2025/ravi2025iclr-sam/)
- RRT-Connect, ICRA 2000 volume/DOI: [Illinois Experts record](https://experts.illinois.edu/en/publications/rrt-connect-an-efficient-approach-to-single-query-path-planning/)
- Fast and Robust ICP, TPAMI fields/DOI: [Cardiff ORCA](https://orca.cardiff.ac.uk/id/eprint/137995/)
- Wei et al., IROS 2024 pages/DOI: [Univ. Bedfordshire research portal](https://researchportal.beds.ac.uk/en/publications/coarse-to-fine-detection-of-multiple-seams-for-robotic-welding/)
- Zhou et al., RA-L 2021: [PolyU institutional repository](https://ira.lib.polyu.edu.hk/handle/10397/95536)
- Kunz & Stilman, RSS 2012: [roboticsproceedings.org](https://roboticsproceedings.org/rss08/p27.html)
- Keselman et al., CVPRW 2017 pages/DOI: [mlanthology](https://mlanthology.org/cvprw/2017/keselman2017cvprw-intel)
- MediaPipe Hands: [arXiv:2006.10214](https://arxiv.org/abs/2006.10214v1)
