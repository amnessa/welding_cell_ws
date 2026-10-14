"""The true shape of a registered part, for multi-view refinement and view planning
(`notes/curved_seams_plan.md` step 5).

Multi-view refinement treated every neighbour as its bounding box: exact for a plate,
wrong for a curved part. S3's saddle end wraps round R2 and sits deep inside R2's box,
so S3 read as ~5 mm inside it (five times the overlap rule's 1 mm). The plate under E2's mitred end lies inside
E2's box. SP3's box covers most of the plate along the S, and it hid the very seam
region the views are planned to see.

A `Shape` is built from the part's registry entry (`models/weldgen_objects.json`, the
weld_generator primitive and `T_cad_prim`). It answers two questions, both in the part's
CAD frame:

  * `sdf(points)`: the signed distance to the real surface (metres, negative inside)
    and the outward normal of the nearest piece of surface;
    * a slab: its box;
    * a pipe: its cylinder from the base cut (flat, mitre, saddle) to the top cap, with
      the bore counted as solid (no camera point and no neighbour is ever inside it);
    * a band: its walls (offsets about the spine) between its caps, so a rounded-rect
      tube's interior is free;
  * `boxes(T)`: the near-exact collision boxes of `collision.boxes_from_parts`
    (6 rotated boxes per pipe, a chain per band), for occlusion and planning.

The distances are exact on the faces and approximate near edges (the larger term wins),
which is what an overlap penalty and a fit-up gap need. The units are metres in and out;
the primitives work in mm inside.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Optional

import numpy as np

REGISTRY_NAME = "weldgen_objects.json"


class Shape:
    """A registered part's surface in its CAD frame (see the module docstring)."""

    def __init__(self, name: str, entry: dict[str, Any], weldgen_path: Optional[str] = None):
        from .seam_from_registration import posed_parts
        self.name = name
        self.entry = entry
        self.weldgen_path = weldgen_path
        stem = Path(name).stem
        self._reg = {"parts": {stem: entry}}
        # the primitive in the CAD frame (mm): T_world_part = T_cad_prim
        self.prim = posed_parts([(name, np.eye(4))], self._reg, pose_units="mm",
                                weldgen_path=weldgen_path)[0]
        self.kind = type(self.prim).__name__
        self._T = np.asarray(self.prim.T_world_part, float)        # CAD <- primitive, mm
        if self.kind == "SweptSlab":
            sp = self.prim.spine
            n = max(512, int(np.ceil(sp.length_mm / 0.5)))
            self._ts = np.linspace(0.0, sp.t_period, n, endpoint=not sp.closed)
            self._P = sp.point(self._ts)[:, :2]
            tan = sp.tangent(self._ts)[:, :2]
            self._tan = tan / np.linalg.norm(tan, axis=1, keepdims=True)
            self._nrm = np.column_stack([-self._tan[:, 1], self._tan[:, 0]])
            from scipy.spatial import cKDTree
            self._tree = cKDTree(self._P)

    # -------------------------------------------------------------- symmetry -----------
    @property
    def symmetry_axis(self) -> Optional[np.ndarray]:
        """The CAD-frame axis the part is fully symmetric about (an uncut tube), or None."""
        from .weldgen_registry import symmetry_axis
        return symmetry_axis(self.entry)

    # -------------------------------------------------------------- distances ----------
    def sdf(self, p_cad_m: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """(signed distance (m), outward normal (CAD frame)) of points in the CAD frame."""
        p = np.atleast_2d(np.asarray(p_cad_m, float)) * 1000.0
        loc = (p - self._T[:3, 3]) @ self._T[:3, :3]
        if self.kind == "Tube":
            sd, n = self._sdf_tube(loc)
        elif self.kind == "SweptSlab":
            sd, n = self._sdf_band(loc)
        else:
            sd, n = self._sdf_box(loc, np.asarray(self.prim.dims_mm, float) / 2.0)
        return sd / 1000.0, n @ self._T[:3, :3].T

    @staticmethod
    def _combine(terms: list[np.ndarray], normals: list[np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
        """Box-like combination of per-axis distance terms: inside the largest (all <= 0),
        outside the length of the positive parts; the normal of the largest term (outside:
        the positive terms' normals weighted by their size)."""
        D = np.stack(terms, axis=1)                                   # (n, k)
        N = np.stack(normals, axis=1)                                 # (n, k, 3)
        pos = np.maximum(D, 0.0)
        out = np.linalg.norm(pos, axis=1)
        inside = out <= 0.0
        sd = np.where(inside, D.max(axis=1), out)
        k = np.argmax(D, axis=1)
        n_in = N[np.arange(len(D)), k]
        n_out = np.einsum("nk,nkj->nj", pos, N)
        nn = np.linalg.norm(n_out, axis=1, keepdims=True)
        n_out = np.where(nn > 1e-12, n_out / np.clip(nn, 1e-12, None), n_in)
        return sd, np.where(inside[:, None], n_in, n_out)

    def _sdf_box(self, loc: np.ndarray, half: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        terms, normals = [], []
        for k in range(3):
            sgn = np.where(loc[:, k] >= 0.0, 1.0, -1.0)
            terms.append(np.abs(loc[:, k]) - half[k])
            n = np.zeros((len(loc), 3))
            n[:, k] = sgn
            normals.append(n)
        return self._combine(terms, normals)

    def _sdf_tube(self, loc: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        t = self.prim
        x, y, z = loc[:, 0], loc[:, 1], loc[:, 2]
        rho = np.hypot(x, y)
        phi = np.arctan2(y, x)
        radial = np.column_stack([x, y, np.zeros(len(x))]) / np.clip(rho, 1e-9, None)[:, None]
        # the cut's height where the point IS (a saddle's varies across the pipe by up to
        # 4.5 mm over S3's bore: evaluated at the bore radius, R2's surface under S3's bore
        # read as 3.6 mm inside S3); clipped at the outer radius, past which the radial
        # term decides and a saddle cut need not exist
        base = t.base_height(phi, np.clip(rho, 0.0, t.r_outer_mm))
        cut = t.base_cut
        if cut is None:
            n_base = np.tile([0.0, 0.0, -1.0], (len(loc), 1))
        elif cut["kind"] == "plane":
            nl = np.asarray(cut["n_local"], float)
            nl = nl / np.linalg.norm(nl)
            n_base = np.tile(nl if nl[2] < 0 else -nl, (len(loc), 1))
        else:                                       # the saddle: towards the cut cylinder's axis
            p0 = np.asarray(cut["point_local"], float)
            m = np.asarray(cut["axis_local"], float)
            m = m / np.linalg.norm(m)
            v = np.column_stack([x, y, base]) - p0
            u = v - (v @ m)[:, None] * m
            n_base = -u / np.clip(np.linalg.norm(u, axis=1, keepdims=True), 1e-9, None)
        # the distance to the cut along its normal ~ the axial gap x the normal's axial part
        d_base = (base - z) * np.clip(np.abs(n_base[:, 2]), 0.2, 1.0)
        terms = [rho - t.r_outer_mm, z - float(t.length_mm), d_base]
        normals = [radial, np.tile([0.0, 0.0, 1.0], (len(loc), 1)), n_base]
        return self._combine(terms, normals)

    def _sdf_band(self, loc: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        b = self.prim
        xy = loc[:, :2]
        _, k = self._tree.query(xy)
        P, nrm, tan = self._P[k], self._nrm[k], self._tan[k]
        s = np.einsum("ij,ij->i", xy - P, nrm)                         # offset across the band
        lo, hi = float(b.offset_lo_mm), float(b.offset_hi_mm)
        d_hi, d_lo = s - hi, lo - s
        d_off = np.maximum(d_hi, d_lo)
        n_off = np.where((d_hi >= d_lo)[:, None], nrm, -nrm)
        z = loc[:, 2]
        d_z = np.maximum(float(b.z0_mm) - z, z - float(b.z1_mm))
        n_z = np.where((z - float(b.z1_mm) >= float(b.z0_mm) - z)[:, None],
                       [0.0, 0.0, 1.0], [0.0, 0.0, -1.0])
        terms = [d_off, d_z]
        normals = [np.column_stack([n_off, np.zeros(len(xy))]), n_z]
        if not b.spine.closed:                                         # the end caps
            u = np.einsum("ij,ij->i", xy - P, tan)
            end = (k == 0) | (k == len(self._P) - 1)
            sgn = np.where(k == 0, -1.0, 1.0)
            d_u = np.where(end, sgn * u, -np.inf)
            terms.append(d_u)
            normals.append(np.column_stack([sgn[:, None] * tan, np.zeros(len(xy))]))
        return self._combine(terms, normals)

    # -------------------------------------------------------------- planning boxes -----
    def boxes(self, T_world_cad_m: np.ndarray, label: str = "A") -> list[dict[str, Any]]:
        """The near-exact collision boxes at a pose (metres, world): `boxes_from_parts`."""
        from .collision import boxes_from_parts
        from .seam_from_registration import posed_parts
        prim = posed_parts([(self.name, np.asarray(T_world_cad_m, float))], self._reg,
                           pose_units="m", weldgen_path=self.weldgen_path)[0]
        prim.id = label
        return boxes_from_parts([prim])


def load_shapes(models_dir, names, weldgen_path: Optional[str] = None
                ) -> dict[str, Optional[Shape]]:
    """{model name: Shape or None} from `<models_dir>/weldgen_objects.json`. None for a part
    without a usable entry (or no registry, or weld_generator not importable): it keeps its
    bounding box."""
    out: dict[str, Optional[Shape]] = {n: None for n in names}
    path = Path(models_dir) / REGISTRY_NAME
    if not path.is_file():
        return out
    try:
        reg = json.loads(path.read_text())
    except (OSError, ValueError):
        return out
    parts = reg.get("parts", reg)
    for n in names:
        entry = parts.get(Path(n).stem)
        if not entry or not entry.get("primitive"):
            continue
        try:
            out[n] = Shape(n, entry, weldgen_path)
        except Exception:  # noqa: BLE001 - a broken entry or no weldgen: the box stays
            out[n] = None
    return out
