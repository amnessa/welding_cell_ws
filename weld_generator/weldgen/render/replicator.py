"""Render products and annotators — `omni` is imported inside functions only.

One render product on the stage camera; four annotators: `rgb`, `distance_to_image_plane`
(= z_cam, the depth `camera.project` uses), `instance_id_segmentation_fast` (the id buffer,
keyed by prim path -> our `mask_object` labels), `normals` (world frame). `render()` steps
the orchestrator and returns numpy arrays in weldgen units.
"""

from __future__ import annotations

import numpy as np

from .conventions import LABEL_NONE, MM_PER_M

ANNOTATORS = ("rgb", "distance_to_image_plane", "instance_id_segmentation_fast", "normals")


def labels_from_ids(ids: np.ndarray, id_to_label: dict, label_by_path: dict) -> np.ndarray:
    """Map the id buffer to `mask_object` labels. The lookup table spans every id the
    renderer declares, not only those present in this frame - a prim that is fully hidden
    in a view still has an id (an edge scene crashed on that, 2026-09-12)."""
    n = max([int(ids.max()) if ids.size else 0] + [int(k) for k in id_to_label]) + 1
    lut = np.full(n, LABEL_NONE, dtype=np.uint8)
    for k, path in id_to_label.items():
        lut[int(k)] = label_by_path.get(path, LABEL_NONE)
    return lut[ids]


class Renderer:
    def __init__(self, camera_path: str, width: int, height: int, rt_subframes: int = 16):
        import omni.replicator.core as rep
        self.rep = rep
        self.width, self.height, self.rt_subframes = int(width), int(height), int(rt_subframes)
        self.rp = rep.create.render_product(camera_path, (self.width, self.height))
        self.ann = {}
        for n in ANNOTATORS:
            a = rep.AnnotatorRegistry.get_annotator(n, init_params={"colorize": False} if "segmentation" in n else None)
            a.attach(self.rp); self.ann[n] = a

    def render(self, label_by_path: dict, warmup_steps: int = 1) -> dict:
        """Step until the frame is settled; return rgb, depth_mm, valid, mask_object, normals."""
        for _ in range(warmup_steps + 1):
            self.rep.orchestrator.step(rt_subframes=self.rt_subframes)
        rgb = np.asarray(self.ann["rgb"].get_data())[..., :3].astype(np.uint8)
        d = np.asarray(self.ann["distance_to_image_plane"].get_data(), dtype=np.float64) * MM_PER_M
        valid = np.isfinite(d) & (d > 0)
        d = np.where(valid, d, 0.0)
        seg = self.ann["instance_id_segmentation_fast"].get_data()
        mask_object = labels_from_ids(np.asarray(seg["data"]), seg["info"]["idToLabels"], label_by_path)
        normals = np.asarray(self.ann["normals"].get_data(), dtype=np.float32)[..., :3]
        return {"rgb": rgb, "depth_mm": d, "valid": valid, "mask_object": mask_object, "normals": normals}


def close_app(app) -> None:
    app.close()
