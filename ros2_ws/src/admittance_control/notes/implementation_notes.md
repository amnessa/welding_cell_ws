calibrate camera extrinsics in real robot.
create a node that calculates poses relative to the camera to the robot base


example outputs from sam6d

detection ism.json:

ISM detections: 16
keys: ['scene_id', 'image_id', 'category_id', 'bbox', 'score', 'time', 'segmentation']

top detections by score:
  idx  2  cat=1  score=0.4726  bbox=[498, 204, 314, 274]
  idx 13  cat=1  score=0.4035  bbox=[1212, 0, 66, 46]
  idx  1  cat=1  score=0.3810  bbox=[1016, 0, 262, 168]
  idx 11  cat=1  score=0.3690  bbox=[658, 282, 108, 152]
  idx  4  cat=1  score=0.3674  bbox=[110, 420, 624, 296]
  idx 12  cat=1  score=0.3650  bbox=[544, 206, 106, 148]
  idx  7  cat=1  score=0.3622  bbox=[0, 598, 136, 118]
  idx 10  cat=1  score=0.3547  bbox=[490, 342, 208, 174]
  idx  3  cat=1  score=0.3403  bbox=[666, 320, 146, 158]
  idx  5  cat=1  score=0.3389  bbox=[0, 152, 342, 562]

detection ism.npz:

npz members: ['scene_id', 'image_id', 'category_id', 'score', 'bbox', 'time', 'segmentation']
  scene_id       shape=() dtype=int64
  image_id       shape=() dtype=int64
  category_id    shape=(16,) dtype=float32
  score          shape=(16,) dtype=float32
  bbox           shape=(16, 4) dtype=int64
  time           shape=() dtype=int64
  segmentation   shape=(16, 720, 1280) dtype=float32


detection pem.json:

PEM detections: 16
keys: ['scene_id', 'image_id', 'category_id', 'bbox', 'score', 'time', 'segmentation', 'R', 't']

best pose  score=0.4209  cat=1
R=
 [[-0.79374921 -0.60823607  0.0032819 ]
 [-0.57717061  0.75148344 -0.31960317]
 [ 0.19192797 -0.25557879 -0.94754577]]
t (mm)= [ 97.37220764   4.00947142 583.87390137]   -> depth 0.584 m


shapes of the results :

ISM JSON type: list
ISM JSON length: 16
ISM JSON item keys: ['scene_id', 'image_id', 'category_id', 'bbox', 'score', 'time', 'segmentation']
PEM JSON type: list
PEM JSON length: 16
PEM JSON item keys: ['scene_id', 'image_id', 'category_id', 'bbox', 'score', 'time', 'segmentation', 'R', 't']
ISM NPZ keys: KeysView(NpzFile '/home/cagolinux/SAM-6D-cago/SAM-6D/Data/Output/sam6d_results/detection_ism.npz' with keys: scene_id, image_id, category_id, score, bbox...)
scene_id: ()
image_id: ()
category_id: (16,)
score: (16,)
bbox: (16, 4)
time: ()
segmentation: (16, 720, 1280)

first couple outputs of the json files:

ism:

[{"scene_id": 0, "image_id": 0, "category_id": 1, "bbox": [0, 2, 1278, 714], "score": 0.28183379769325256, "time": 0.0, "segmentation": {"counts": [3, 716, 4, 716, 4, 716, 2, 718, 2, 718, 2, 718, 2, 718, 2, 718, 2, 718, 2, 718, 2, 718, 2, 718, 2, 718, 2, 718, 2, 718, 4, 716, 4, 716, 4, 716, 4, 716, 4, 716, 4, 716, 2, 718, 2, 718, 2, 718,

pem:

[{"scene_id": 0, "image_id": 0, "category_id": 1, "bbox": [0, 2, 1278, 714], "score": 0.0037155821919441223, "time": 0.0, "segmentation": {"counts": [3, 716, 4, 716, 4, 716, 2, 718, 2, 718, 2, 718, 2, 718, 2, 718, 2, 718, 2, 718, 2, 718, 2, 718, 2, 718, 2, 718, 2, 718, 4, 716, 4, 716, 4, 716, 4, 716, 4, 716


calibration update note:

One thing to keep in mind downstream: with the camera on the wrist there is no longer a constant base→camera transform. The calibration constant is T_tcp_to_cam; consumers like the SAM-6D bridge need to compute T_base_to_cam = T_base_to_tcp(current pose) @ T_tcp_to_cam at each capture. The legacy sand-drawer script and the Sam_to_Surface_Plane notebook still load a static T_base_to_cam.npy, so they'd need that same update if you plan to reuse them with the new mount.