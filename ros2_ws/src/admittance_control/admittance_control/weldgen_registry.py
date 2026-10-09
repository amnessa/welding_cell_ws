"""Part registry: which weldgen primitive each library CAD is - mode A's one input.

Mode A of the seam pipeline (`notes/seam_two_modes_plan.md`) computes the weld seam
from the registered poses of the parts instead of detecting it. That needs each
library `.ply` described as an exact `weld_generator` primitive (slab, prism, tube,
swept band ...) together with the rigid transform between the CAD file's frame and the
primitive's canonical local frame - the SEPC poses (`pose_static`) place the CAD
frame, the primitive lives in its own frame.

`derive_box` reads that description off a box-shaped mesh automatically and VERIFIES it
(every vertex on the box surface, volume match), accepting a plate with small notches,
tabs or holes as its envelope slab with the ignored fraction recorded; anything it
cannot verify is recorded as unsupported with the reason, never guessed.

The curved parts (`notes/curved_seams_plan.md` step 1) are derived the same way:
`derive_tube` fits a pipe (axis, radii, a flat / plane / cylinder cut end) and
`derive_extrusion` a rounded-rectangle tube (a closed `swept_slab`); an open band is
measured and refused unless its wall is constant. `verify_both_ways` then checks the
entry in both directions (CAD vertices on the primitive, primitive vertices on the CAD)
plus the volume, so an entry that is too long or too thick fails as well as one that
misses a face. Hand-written entries in the JSON are kept and checked the same way.

Units: the registry is in millimetres (the library convention, `model_units: mm`).
`T_cad_prim` maps primitive-local (mm) -> CAD-file frame (mm).
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

REGISTRY_VERSION = "weldgen_registry-1.0"


def _load_mesh(path):
    import trimesh
    return trimesh.load(str(path), force="mesh")


def derive_box(mesh, tol_mm: float = 0.05, max_deficit: float = 0.10) -> dict[str, Any]:
    """A slab entry for a box-like mesh, or `{"primitive": None, "reason": ...}`.

    Axes: u = longest extent, v = middle, w = shortest (the thickness - weldgen's broad
    faces are +/-w). The frame is right-handed by construction, centred on the box.

    Two acceptances, both recorded in `approx`:
      * `"exact"`   - every vertex is a box corner and the volume is L*W*t: the CAD IS
                      the slab.
      * `"envelope"` - every vertex lies ON the oriented bounding box's surface and the
                      mesh fills at least `1 - max_deficit` of it: a plate with small
                      features cut from or added to its outline (edge notches, locating
                      tabs, holes) - the lab's slotted `test_objv1` parts. The slab is
                      the envelope; the features are ignored, so a seam that runs across
                      a notch is labelled at its nominal length and a tab that passes
                      through the other part shows up as a nominal penetration of the
                      tab length in `fitup_mm`. `envelope_deficit` records how much was
                      ignored. Hand-edit `dims_mm` / `T_cad_prim` and set
                      `"hand_edited": true` to describe the body instead.
    Anything else (an L-shaped composite, a curved band, a mesh with interior vertices)
    is unsupported with the reason, never guessed.
    """
    v = np.asarray(mesh.vertices, dtype=float)
    if len(v) < 8:
        return {"primitive": None, "reason": f"{len(v)} vertices - not a box"}
    obb = mesh.bounding_box_oriented
    T_obb = np.asarray(obb.primitive.transform, dtype=float)      # obb frame -> cad
    ext = np.asarray(obb.primitive.extents, dtype=float)
    order = np.argsort(ext)[::-1]                                  # longest first
    L, W, t = (float(ext[i]) for i in order)
    axes = T_obb[:3, :3][:, order]
    if np.linalg.det(axes) < 0:
        axes[:, 1] = -axes[:, 1]                                   # keep it right-handed
    T = np.eye(4)
    T[:3, :3] = axes
    T[:3, 3] = T_obb[:3, 3]
    local = (v - T[:3, 3]) @ axes
    half = np.array([L, W, t]) / 2.0
    box_vol = L * W * t
    entry = {"primitive": "slab", "dims_mm": [L, W, t], "T_cad_prim": T.tolist()}
    # exact box: every vertex is a corner (|coord| == half-extent on EVERY axis)
    dev = np.abs(np.abs(local) - half).max()
    vol_err = abs(float(mesh.volume) - box_vol) / box_vol
    if dev <= tol_mm and vol_err <= 0.01:
        return {**entry, "approx": "exact", "max_dev_mm": float(dev),
                "volume_rel_err": float(vol_err)}
    # envelope: every vertex on the box SURFACE (inside it, and on at least one face)
    per_axis = np.abs(np.abs(local) - half)                        # (n, 3)
    inside = (np.abs(local) <= half + tol_mm).all(axis=1)
    on_face = per_axis.min(axis=1) <= tol_mm
    if not (inside & on_face).all():
        return {"primitive": None,
                "reason": f"vertices deviate {dev:.3f} mm from a box (tol {tol_mm})"}
    deficit = 1.0 - float(mesh.volume) / box_vol
    if not (-0.01 <= deficit <= max_deficit):
        return {"primitive": None,
                "reason": (f"fills only {1 - deficit:.0%} of its envelope "
                           f"(features > {max_deficit:.0%}) - not a slab")}
    return {**entry, "approx": "envelope", "max_dev_mm": float(dev),
            "envelope_deficit": float(deficit)}


def verify_entry(entry: dict[str, Any], mesh, weldgen, tol_mm: float = 0.25) -> float:
    """Max distance of the CAD mesh's vertices to the primitive's surface (mm).

    Uses `weld_generator`'s rtree-free mesh distance so the check runs in the ROS
    environment as it is. `tol_mm` is the D34 chord budget: a hand-written tube or
    band entry is accepted when its tessellation and the CAD agree to that.
    """
    from weldgen.geom import from_object
    from weldgen.render.gate import distance_to_mesh
    obj = {"id": "A", "role": "workpiece", "object_id": 0,
           "primitive": entry["primitive"], "T_world_part": entry["T_cad_prim"],
           "dims_mm": entry.get("dims_mm"), "thickness_mm": entry.get("thickness_mm"),
           "outline_uv": entry.get("outline_uv"), "outline_shape": entry.get("outline_shape"),
           "params": entry.get("params")}
    prim = from_object({k: v for k, v in obj.items() if v is not None})
    d = distance_to_mesh(np.asarray(mesh.vertices, float), prim.mesh())
    return float(d.max())


def verify_both_ways(entry: dict[str, Any], mesh, weldgen) -> dict[str, float]:
    """Deviation of an entry from its CAD mesh, in both directions, plus the volume.

    `cad_to_prim_mm`: the CAD vertices to the primitive's surface (`verify_entry`); a
    missing or misplaced face shows here. `prim_to_cad_mm`: the primitive's mesh
    vertices to the CAD surface; an entry that is too long, or a cut on the wrong side,
    shows here, and the first direction cannot see it. Both include the two meshes'
    chord error (each tessellates the same true surface). `volume_rel_err` catches a
    hollow / solid mix-up that both distances miss.
    """
    from weldgen.render.gate import distance_to_mesh
    prim = _primitive(entry)
    pm = prim.mesh()
    cad_to_prim = distance_to_mesh(np.asarray(mesh.vertices, float), pm)
    prim_to_cad = distance_to_mesh(np.asarray(pm.vertices, float), mesh)
    vol = abs(float(pm.volume))
    return {"cad_to_prim_mm": float(cad_to_prim.max()),
            "prim_to_cad_mm": float(prim_to_cad.max()),
            "volume_rel_err": float(abs(vol - abs(float(mesh.volume))) / vol)}


def _primitive(entry: dict[str, Any]):
    from weldgen.geom import from_object
    obj = {"id": "A", "role": "workpiece", "object_id": 0,
           "primitive": entry["primitive"], "T_world_part": entry["T_cad_prim"],
           "dims_mm": entry.get("dims_mm"), "thickness_mm": entry.get("thickness_mm"),
           "outline_uv": entry.get("outline_uv"), "outline_shape": entry.get("outline_shape"),
           "params": entry.get("params")}
    return from_object({k: v for k, v in obj.items() if v is not None})


# --- curved parts: fitting helpers ----------------------------------------------------

def _unit(v) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)


def _frame(z, x_hint, origin) -> np.ndarray:
    """A right-handed 4x4 with local +z = `z`, local x = `x_hint` made perpendicular."""
    z = _unit(z)
    x = np.asarray(x_hint, float) - np.dot(x_hint, z) * z
    if np.linalg.norm(x) < 1e-6:                     # hint along z: any perpendicular
        x = np.cross(z, [1.0, 0.0, 0.0] if abs(z[0]) < 0.9 else [0.0, 1.0, 0.0])
    x = _unit(x)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2] = x, np.cross(z, x), z
    T[:3, 3] = origin
    return T


def _cad_hint(z) -> np.ndarray:
    """The CAD axis most perpendicular to `z`, lowest index on ties - so a part whose
    axis is a CAD axis gets a frame that is the CAD frame up to a shift."""
    return np.eye(3)[int(np.argmin(np.round(np.abs(z), 6)))]


def _sweep_axis(mesh, lateral_cos: float = 0.15, iters: int = 4) -> np.ndarray:
    """The axis every lateral face is parallel to: the least-spread direction of the
    area-weighted face normals, re-estimated on the faces near-perpendicular to it
    (the end faces would otherwise pull it). Sign: positive on its largest CAD axis."""
    n = np.asarray(mesh.face_normals, float)
    w = np.asarray(mesh.area_faces, float)
    sel = np.ones(len(n), bool)
    a = None
    for _ in range(iters):
        C = (n[sel] * w[sel, None]).T @ n[sel]
        a = np.linalg.eigh(C)[1][:, 0]
        sel = np.abs(n @ a) < lateral_cos
    k = int(np.argmax(np.abs(a)))
    return a if a[k] > 0 else -a


def _circle_fit(xy: np.ndarray) -> np.ndarray:
    """Algebraic (Kasa) circle centre of 2D points."""
    A = np.column_stack([xy, np.ones(len(xy))])
    b = -(xy ** 2).sum(axis=1)
    D, E, _ = np.linalg.lstsq(A, b, rcond=None)[0]
    return np.array([-D / 2.0, -E / 2.0])


def _fit_cylinder(pts: np.ndarray, normals: np.ndarray) -> dict[str, Any]:
    """A cylinder through `pts` with a free axis (least squares). The start: the axis
    is the direction the surface normals are perpendicular to; radius and point from a
    circle fit in the plane across it."""
    from scipy.optimize import least_squares
    m0 = np.linalg.eigh(normals.T @ normals)[1][:, 0]
    B = _frame(m0, _cad_hint(m0), np.zeros(3))[:3, :3]
    loc = pts @ B
    c2 = _circle_fit(loc[:, :2])
    q0 = B @ np.array([c2[0], c2[1], 0.0])
    r0 = float(np.linalg.norm(loc[:, :2] - c2, axis=1).mean())

    def unpack(p):
        m = _unit(m0 + B[:, 0] * p[0] + B[:, 1] * p[1])
        q = q0 + B[:, 0] * p[2] + B[:, 1] * p[3]
        return m, q, p[4]

    def resid(p):
        m, q, r = unpack(p)
        return np.linalg.norm(np.cross(pts - q, m), axis=1) - r

    s = least_squares(resid, [0.0, 0.0, 0.0, 0.0, r0])
    m, q, r = unpack(s.x)
    return {"axis": m, "point": q, "radius_mm": float(r),
            "max_resid_mm": float(np.abs(s.fun).max())}


def _classify_end(pts: np.ndarray, normals: np.ndarray, axis: np.ndarray,
                  tol_mm: float) -> dict[str, Any]:
    """What one end of a tube is: `flat` (across the axis), `plane` (a tilted cut) or
    `cylinder` (a saddle cut); with the fit residual."""
    ndot = np.abs(normals @ axis)
    if ndot.min() > 1.0 - 1e-6:
        h = pts @ axis
        return {"kind": "flat", "h": float(h.mean()),
                "max_resid_mm": float(np.abs(h - h.mean()).max())}
    nbar = _unit(normals.mean(axis=0))
    if np.degrees(np.arccos(np.clip((normals @ nbar).min(), -1.0, 1.0))) < 0.05:
        d = float((pts @ nbar).mean())
        return {"kind": "plane", "n": nbar, "d": d,
                "max_resid_mm": float(np.abs(pts @ nbar - d).max())}
    cyl = _fit_cylinder(pts, normals)
    return {"kind": "cylinder", **cyl}


def derive_tube(mesh, tol_mm: float = 0.05) -> dict[str, Any]:
    """A `tube` entry for a pipe mesh, or `{"primitive": None, "reason": ...}`.

    The tube's local frame (weldgen SCHEMA §2.2): the axis is local +z, the base end at
    z = 0, the top cap (flat, required) at z = `length_mm`. The base is the other end:
    flat, or cut by a plane (a mitre) or a cylinder (a saddle). Every lateral vertex
    must sit on one of two radii within `tol_mm`, and each end must fit its surface
    within `tol_mm`; otherwise the mesh is refused with the number. With two flat ends
    the base is the one at the lower axial coordinate (the axis points along its
    largest CAD component), and either end can stand on a plate later.
    """
    v = np.asarray(mesh.vertices, float)
    n = np.asarray(mesh.face_normals, float)
    a = _sweep_axis(mesh)
    lat = np.abs(n @ a) < 1e-3
    if lat.sum() < 8:
        return {"primitive": None, "reason": "no lateral faces parallel to one axis"}
    lv = v[np.unique(mesh.faces[lat])]
    B = _frame(a, _cad_hint(a), np.zeros(3))[:3, :3]
    c2 = _circle_fit((lv @ B)[:, :2])
    rho = np.linalg.norm((v @ B)[:, :2] - c2, axis=1)
    rho_lat = rho[np.unique(mesh.faces[lat])]
    r_o, r_i = float(rho_lat.max()), float(rho_lat.min())
    off = np.minimum(np.abs(rho_lat - r_o), np.abs(rho_lat - r_i)).max()
    if off > tol_mm or r_o - r_i < 0.5:
        return {"primitive": None,
                "reason": (f"lateral vertices are {off:.3f} mm off two radii "
                           f"({r_i:.2f}/{r_o:.2f}) - not a pipe")}
    centre = B @ np.array([c2[0], c2[1], 0.0])       # a point on the axis (axial 0)
    ends = []
    for sign in (+1.0, -1.0):
        f = ~lat & (sign * (n @ a) > 0)
        if not f.any():
            return {"primitive": None, "reason": "an open end"}
        ends.append({"sign": sign, **_classify_end(v[np.unique(mesh.faces[f])], n[f],
                                                   a, tol_mm)})
    bad = [e for e in ends if e["max_resid_mm"] > tol_mm]
    if bad:
        return {"primitive": None,
                "reason": f"{bad[0]['kind']} end fits to {bad[0]['max_resid_mm']:.3f} mm"}
    flats = [e for e in ends if e["kind"] == "flat"]
    if not flats:
        return {"primitive": None, "reason": "no flat end for the top cap"}
    # the top: a flat end; with two, the +axis one (the base is then the lower end)
    top = flats[0] if len(flats) == 1 or flats[0]["sign"] > 0 else flats[1]
    base = ends[1] if top is ends[0] else ends[0]
    z = a * top["sign"]                               # base -> top
    f_base = ~lat & (base["sign"] * (n @ a) > 0)
    z_base_min = float(((v[np.unique(mesh.faces[f_base])] - centre) @ z).min())
    origin = centre + z_base_min * z
    T = _frame(z, _cad_hint(z), origin)
    R = T[:3, :3]
    length = float((top["h"] - centre @ a) * top["sign"] - z_base_min)
    params: dict[str, Any] = {"r_outer_mm": r_o, "wall_mm": r_o - r_i, "length_mm": length}
    if base["kind"] == "plane":
        n_loc = R.T @ base["n"]
        d_loc = float(base["d"] - base["n"] @ origin)
        params["base_cut"] = {"kind": "plane", "n_local": n_loc.tolist(), "d": d_loc}
    elif base["kind"] == "cylinder":
        params["base_cut"] = {"kind": "cylinder",
                              "point_local": (R.T @ (base["point"] - origin)).tolist(),
                              "axis_local": (R.T @ base["axis"]).tolist(),
                              "radius_mm": base["radius_mm"]}
    return {"primitive": "tube", "params": params, "T_cad_prim": T.tolist(),
            "approx": "fitted", "base_end": base["kind"],
            "fit_resid_mm": float(max(off, *(e["max_resid_mm"] for e in ends)))}


def _boundary_loops(faces: np.ndarray) -> list[np.ndarray]:
    """The closed boundary loops (vertex index chains) of a triangle patch."""
    e = np.sort(np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]]),
                axis=1)
    uniq, cnt = np.unique(e, axis=0, return_counts=True)
    nxt: dict[int, list[int]] = {}
    for a, b in uniq[cnt == 1]:
        nxt.setdefault(int(a), []).append(int(b))
        nxt.setdefault(int(b), []).append(int(a))
    loops, seen = [], set()
    for start in nxt:
        if start in seen:
            continue
        loop, prev, cur = [start], None, start
        seen.add(start)
        while True:
            cand = [x for x in nxt[cur] if x != prev and (x not in seen or x == start)]
            if not cand or cand[0] == start:
                break
            prev, cur = cur, cand[0]
            loop.append(cur)
            seen.add(cur)
        loops.append(np.asarray(loop))
    return loops


def _polygon_area(xy: np.ndarray) -> float:
    return 0.5 * float(np.sum(xy[:, 0] * np.roll(xy[:, 1], -1)
                              - np.roll(xy[:, 0], -1) * xy[:, 1]))


def _band_thickness(xy: np.ndarray) -> np.ndarray:
    """Wall thickness of a simple closed outline at each vertex: the distance along the
    inward vertex normal to where it next meets the outline."""
    if _polygon_area(xy) < 0:
        xy = xy[::-1]
    t_prev = xy - np.roll(xy, 1, axis=0)
    t_next = np.roll(xy, -1, axis=0) - xy
    t = t_prev / np.linalg.norm(t_prev, axis=1, keepdims=True) \
        + t_next / np.linalg.norm(t_next, axis=1, keepdims=True)
    inward = np.column_stack([-t[:, 1], t[:, 0]])
    inward /= np.linalg.norm(inward, axis=1, keepdims=True)
    a, b = xy, np.roll(xy, -1, axis=0)
    out = np.full(len(xy), np.inf)
    for k, (p, d) in enumerate(zip(xy, inward)):
        e = b - a
        den = d[0] * e[:, 1] - d[1] * e[:, 0]
        w = a - p
        with np.errstate(divide="ignore", invalid="ignore"):
            s = (w[:, 0] * e[:, 1] - w[:, 1] * e[:, 0]) / den      # along the ray
            u = (w[:, 0] * d[1] - w[:, 1] * d[0]) / den            # along the edge
        ok = (s > 1e-6) & (u >= 0.0) & (u <= 1.0)
        ok[[k, k - 1]] = False
        if ok.any():
            out[k] = float(s[ok].min())
    return out


def _clamped_knots(m: int, p: int = 3) -> np.ndarray:
    """weldgen `BSplineCurve`'s knot vector: clamped, uniform inner knots."""
    return np.concatenate([np.zeros(p), np.linspace(0.0, 1.0, m - p + 1), np.ones(p)])


def _fit_bspline(pts: np.ndarray, m: int) -> tuple[np.ndarray, float]:
    """Cubic with `m` control points through ordered 2D `pts`, ends interpolated:
    control points AND every point's curve parameter solved together (nonlinear least
    squares). It is started from linear fits alternated with re-projecting the points
    onto the curve: from chord-length parameters alone it stalls at 0.06 mm on SP3,
    whose lobes need the parameter to slow down where the points crowd. Returns (control points, max distance of `pts` to the curve)."""
    from scipy.interpolate import BSpline
    from scipy.optimize import least_squares
    from scipy.spatial import cKDTree
    p = 3
    kn = _clamped_knots(m, p)
    d = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])
    u = d / d[-1]
    dense_u = np.linspace(0.0, 1.0, 20000)
    for _ in range(30):            # the start: linear fit <-> re-projection, alternated
        A = BSpline.design_matrix(np.clip(u, 0.0, 1.0 - 1e-12), kn, p).toarray()
        rhs = pts - np.outer(A[:, 0], pts[0]) - np.outer(A[:, -1], pts[-1])
        ci = np.linalg.lstsq(A[:, 1:-1], rhs, rcond=None)[0]
        u = dense_u[cKDTree(BSpline(kn, np.vstack([pts[0], ci, pts[-1]]), p)(dense_u))
                    .query(pts)[1]]
        u[0], u[-1] = 0.0, 1.0
    k = 2 * (m - 2)

    def resid(x):
        C = np.vstack([pts[0], x[:k].reshape(-1, 2), pts[-1]])
        uu = np.concatenate([[0.0], np.clip(x[k:], 0.0, 1.0), [1.0]])
        return (BSpline(kn, C, p)(uu) - pts).ravel()

    # dense on purpose: ~2n x n is small, and the sparse (lsmr) path is far slower here
    sol = least_squares(resid, np.concatenate([ci.ravel(), u[1:-1]]), max_nfev=200)
    ctrl = np.vstack([pts[0], sol.x[:k].reshape(-1, 2), pts[-1]])
    # at the optimum each residual is the point's distance to the curve (a nearest
    # dense sample would add up to half the sample spacing, ~0.1 mm on SP3)
    return ctrl, float(np.linalg.norm(sol.fun.reshape(-1, 2), axis=1).max())


def _fit_open_spine(xy: np.ndarray, wall: float, tol_mm: float, max_ctrl: int = 16):
    """The spine of an open constant-wall band from its cap outline (one loop).

    The outline is two long sides joined by two short ends at four ~90 deg corners.
    Candidate spines: side A, side B, and the midline (side A moved wall/2 toward B).
    Control-point counts are tried upward for all three at once; the first fit within
    `tol_mm / 2` wins (a drawn spline needs few, its offsets many). Returns
    `(control_xy, (offset_lo, offset_hi), fit_resid_mm, which)` or None."""
    if _polygon_area(xy) < 0:
        xy = xy[::-1]
    e1 = xy - np.roll(xy, 1, axis=0)
    e2 = np.roll(xy, -1, axis=0) - xy
    turn = np.degrees(np.arctan2(e1[:, 0] * e2[:, 1] - e1[:, 1] * e2[:, 0],
                                 (e1 * e2).sum(1)))
    corners = np.sort(np.argsort(-np.abs(turn))[:4])
    if np.abs(turn[corners]).min() < 60.0:
        return None
    # the two ends are the corner pairs a wall apart; rotate so side A starts at 0
    n = len(xy)
    gaps = [(corners[(k + 1) % 4] - corners[k]) % n for k in range(4)]
    k0 = int(np.argmax([g if np.linalg.norm(xy[corners[(k + 1) % 4]] - xy[corners[k]])
                        > 2 * wall else -1 for k, g in enumerate(gaps)]))
    order = [corners[(k0 + j) % 4] for j in range(4)]
    side_a = xy[np.arange(order[0], order[0] + (order[1] - order[0]) % n + 1) % n]
    side_b = xy[np.arange(order[2], order[2] + (order[3] - order[2]) % n + 1) % n][::-1]
    if abs(np.linalg.norm(side_a[0] - side_b[0]) - wall) > 0.5 or \
            abs(np.linalg.norm(side_a[-1] - side_b[-1]) - wall) > 0.5:
        return None
    # side A's normal toward B (the outline is CCW, so B is on A's left)
    ta = np.gradient(side_a, axis=0)
    ta /= np.linalg.norm(ta, axis=1, keepdims=True)
    left = np.column_stack([-ta[:, 1], ta[:, 0]])
    mid = side_a + 0.5 * wall * left
    cands = [("side_a", side_a, (0.0, wall)), ("side_b", side_b, (-wall, 0.0)),
             ("midline", mid, (-0.5 * wall, 0.5 * wall))]
    for m in range(4, max_ctrl + 1):                 # fewest control points first
        for which, pts, offs in cands:
            if m > len(pts) // 2:
                continue
            ctrl, err = _fit_bspline(pts, m)
            if err <= 0.5 * tol_mm:
                return ctrl, offs, err, f"{which} ({m} control points)"
    return None


def derive_extrusion(mesh, tol_mm: float = 0.05) -> dict[str, Any]:
    """A `swept_slab` entry for a straight extrusion with two flat caps, or
    `{"primitive": None, "reason": ...}`.

    Supported: a closed rounded-rectangle tube (cap = two loops): the spine is the outer
    outline, CCW, so the band is [0, wall] (weldgen config 5) and z0 = 0, z1 = height.
    An open band (cap = one loop) is MEASURED: its wall must be constant to `tol_mm`
    (a true offset of its spine); a band whose sides are, for example, the same curve
    shifted sideways has a wall that changes along it, which `swept_slab` cannot hold,
    and is refused with the range. A constant-wall band gets a B-spline spine
    (`_fit_open_spine`): whichever of its two sides or its midline a cubic fits with
    the fewest control points (the curve the CAD was drawn from), with the band placed
    on the right side of it.
    """
    from trimesh.bounds import oriented_bounds_2D
    v = np.asarray(mesh.vertices, float)
    n = np.asarray(mesh.face_normals, float)
    a = _sweep_axis(mesh)
    cap = np.abs(n @ a) > 1.0 - 1e-6
    # side triangles of a meshed spline surface can join top and bottom vertices that
    # are not above each other (SP3: up to 2.7 deg); the caps decide, not the sides
    side = np.abs(n @ a) < 0.1
    if not (cap | side).all():
        return {"primitive": None, "reason": "faces neither along nor across one axis"}
    h = v @ a
    lo, hi = float(h.min()), float(h.max())
    base_faces = mesh.faces[cap & (n @ a < 0)]
    if np.abs(h[np.unique(base_faces)] - lo).max() > tol_mm:
        return {"primitive": None, "reason": "caps are not two planes"}
    z = a
    B = _frame(z, _cad_hint(z), np.zeros(3))[:3, :3]
    loops = _boundary_loops(base_faces)
    xy = [(v[lp] @ B)[:, :2] for lp in loops]
    height = hi - lo
    if len(loops) == 1:
        t = _band_thickness(xy[0])
        t = t[np.isfinite(t)]
        lo_t, hi_t = float(np.percentile(t, 5)), float(np.percentile(t, 95))
        if hi_t - lo_t > tol_mm:
            return {"primitive": None, "wall_mm_range": [lo_t, hi_t],
                    "reason": (f"open band with a varying wall ({lo_t:.2f}-{hi_t:.2f} "
                               "mm): not an offset of one spine")}
        wall = 0.5 * (lo_t + hi_t)
        fit = _fit_open_spine(xy[0], wall, tol_mm)
        if fit is None:
            return {"primitive": None, "wall_mm": wall,
                    "reason": "constant-wall open band, but no cubic spine fits it"}
        ctrl, (o_lo, o_hi), resid, which = fit
        T = _frame(z, B[:, 0], lo * z)                    # B's axes, on the base cap
        spine = {"kind": "bspline", "degree": 3,
                 "control_mm": [[float(x), float(y), 0.0] for x, y in ctrl]}
        return {"primitive": "swept_slab", "T_cad_prim": T.tolist(), "approx": "fitted",
                "shape": "open_band", "spine_from": which, "wall_mm": wall,
                "params": {"spine": spine, "offset_lo_mm": o_lo, "offset_hi_mm": o_hi,
                           "z0_mm": 0.0, "z1_mm": height},
                "fit_resid_mm": resid}
    if len(loops) != 2:
        return {"primitive": None, "reason": f"{len(loops)} cap loops"}
    outer, inner = sorted(xy, key=lambda p: -abs(_polygon_area(p)))
    T2, ext = oriented_bounds_2D(outer)
    T2 = np.asarray(T2, float)
    R2, t2 = T2[:2, :2], T2[:2, 2]
    loc_o = outer @ R2.T + t2                         # centred, box-aligned
    loc_i = inner @ R2.T + t2
    w, hh = (float(x) for x in ext)
    wi, hi_ = (float(x) for x in np.ptp(loc_i, axis=0))
    wall = 0.5 * (w - wi)
    if abs(0.5 * (hh - hi_) - wall) > tol_mm:
        return {"primitive": None, "reason": "walls differ on the two sides"}
    # corner radius: the outline vertices off both straight sides lie on four arcs
    m = (np.abs(loc_o[:, 0]) < w / 2 - tol_mm) & (np.abs(loc_o[:, 1]) < hh / 2 - tol_mm)
    if m.sum() < 3:
        return {"primitive": None, "reason": "sharp-cornered rectangle tube"}
    pc = np.abs(loc_o[m])
    from scipy.optimize import least_squares
    s = least_squares(lambda r: np.linalg.norm(pc - [w / 2 - r[0], hh / 2 - r[0]],
                                               axis=1) - r[0], [min(w, hh) / 4])
    r = float(s.x[0])
    if np.abs(s.fun).max() > tol_mm:
        return {"primitive": None,
                "reason": f"corners are not arcs ({np.abs(s.fun).max():.3f} mm)"}
    # the box frame in 3D: centre and x direction on the base cap plane
    Rinv = R2.T
    c_xy = -Rinv @ t2
    x_xy = Rinv @ np.array([1.0, 0.0])
    origin = B @ np.array([c_xy[0], c_xy[1], 0.0]) + lo * z
    T = _frame(z, B @ np.array([x_xy[0], x_xy[1], 0.0]), origin)
    from weldgen.curves import rounded_rect_curve
    spine = rounded_rect_curve([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0],
                               w, hh, r).to_parametric()
    return {"primitive": "swept_slab", "T_cad_prim": T.tolist(), "approx": "fitted",
            "shape": "rounded_rect", "rect_mm": [w, hh], "corner_r_mm": r,
            "params": {"spine": spine, "offset_lo_mm": 0.0, "offset_hi_mm": wall,
                       "z0_mm": 0.0, "z1_mm": height},
            "fit_resid_mm": float(np.abs(s.fun).max())}


def derive_entry(mesh, tol_mm: float = 0.05) -> dict[str, Any]:
    """The first primitive that verifies: slab, then tube, then extrusion. The reasons
    of all three are kept when none does."""
    reasons = []
    for fn in (derive_box, derive_tube, derive_extrusion):
        try:
            e = fn(mesh, tol_mm)
        except Exception as exc:                           # a fit that cannot start
            e = {"primitive": None, "reason": f"{type(exc).__name__}: {exc}"}
        if e.get("primitive"):
            return e
        reasons.append((fn.__name__.removeprefix("derive_"), e))
    # the most specific refusal first: a measured band beats "not a box"
    for kind, e in reversed(reasons):
        if kind == "extrusion" and ("wall_mm_range" in e or "wall_mm" in e):
            return {**e, "reason": f"extrusion: {e['reason']}"}
    return {"primitive": None,
            "reason": "; ".join(f"{k}: {e['reason']}" for k, e in reasons)}


def build_registry(models_dir, existing: dict[str, Any] | None = None,
                   tol_mm: float = 0.05, curved: bool = True) -> dict[str, Any]:
    """Every `*.ply` under `models_dir` -> an entry. Hand-written entries in `existing`
    (marked `hand_edited`, or a non-slab primitive that was not `fitted` here) are kept
    and re-verified, not overwritten. `curved` also tries the tube and extrusion fits
    (needs weld_generator importable for the rounded-rect spine)."""
    models_dir = Path(models_dir)
    reg: dict[str, Any] = {"version": REGISTRY_VERSION, "units": "mm", "parts": {}}
    old = (existing or {}).get("parts", {})
    for ply in sorted(models_dir.glob("*.ply")):
        name = ply.stem
        prev = old.get(name)
        if prev and (prev.get("hand_edited") or (
                prev.get("primitive") not in (None, "slab")
                and prev.get("approx") != "fitted")):
            reg["parts"][name] = prev                              # hand-written: keep
            continue
        mesh = _load_mesh(ply)
        entry = derive_entry(mesh, tol_mm) if curved else derive_box(mesh, tol_mm)
        entry["source"] = ply.name
        reg["parts"][name] = entry
    return reg


def load_registry(path) -> dict[str, Any]:
    return json.loads(Path(path).read_text())


def save_registry(reg: dict[str, Any], path) -> None:
    Path(path).write_text(json.dumps(reg, indent=2))


def symmetry_axis(entry: dict[str, Any] | None) -> np.ndarray | None:
    """The axis (CAD frame, unit) a part is fully symmetric about, or None.

    A tube with no cut end (C1, R2) looks the same at any rotation about its axis, so
    that rotation is unobservable to ICP and to the tracker. A cut end (E2's mitre,
    S3's saddle) fixes it; a rounded-rect tube is only 4-fold symmetric."""
    if not entry or entry.get("primitive") != "tube":
        return None
    if (entry.get("params") or {}).get("base_cut"):
        return None
    return np.asarray(entry["T_cad_prim"], float)[:3, 2]
