"""`lit-modelreg` — Fang & Tian 2024 (RCIM), checked against known geometry.

The method never detects a seam — it registers a model that already carries one — so the
tests pin the registration machinery, the two arms, and the label: every number this
method emits is L0-with-CAD, which resolves the plan's open item on whether it is
implementable without the original CAD assets (it is, from `scene.json`, and it stays an
oracle-anchored method regardless).
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from baselines import REGISTRY, balanced_corpus, prepare, run_matrix  # noqa: E402
from baselines.lit_modelreg import (build_model, cpd_similarity,  # noqa: E402
                                    slab_edge_points, slab_surface_points)


def _bench_scene(jt: str):
    try:
        corpus = balanced_corpus(ROOT / "out" / "bench", per_type=50)
    except (ValueError, FileNotFoundError, NotADirectoryError):
        return None
    p = prepare(corpus[jt][:1])
    return p[0] if p else None


def test_the_rebuilt_model_is_the_scene_geometry_exactly():
    """CAD-by-construction, verified: the model posed back to world overlays the scan
    at sampling resolution, and the seam roundtrip is exact to floating point.

    This is the plan's open item resolved in code: `scene.json` (dims, `T_world_part`,
    `T_world_joint`) is a sufficient CAD source, so the method is implementable — and
    because the model's seam IS the stored truth, it is constitutively L0.
    """
    prep = _bench_scene("T")
    if prep is None:
        pytest.skip("no balanced corpus at out/bench")
    Wm, Pm, Twj = build_model(prep.scene, prep.gt, edges_only=False)
    Ww = Wm @ Twj[:3, :3].T + Twj[:3, 3]
    from scipy.spatial import cKDTree
    d, _ = cKDTree(prep.cloud("full", 0.0)["xyz"]).query(Ww[::7], k=1, workers=-1)
    assert np.median(d) < 1.0
    for pm, g in zip(Pm, prep.gt):
        back = pm @ Twj[:3, :3].T + Twj[:3, 3]
        assert np.abs(back - g).max() < 1e-9


def test_slab_samplers_cover_faces_and_edges():
    dims = (100.0, 40.0, 8.0)
    S = slab_surface_points(dims, np.eye(4), 5.0)
    E = slab_edge_points(dims, np.eye(4), 4.0)
    for pts in (S, E):
        assert np.allclose(np.abs(pts).max(axis=0), [50.0, 20.0, 4.0], atol=1e-9)
    # every edge point sits on at least two face planes; surface points on at least one
    on = lambda p: sum(np.isclose(abs(p), h) for p, h in zip(np.abs(p), [50, 20, 4]))  # noqa: E731
    assert min(on(p) for p in E) >= 2
    assert min(on(p) for p in S) >= 1


def test_cpd_recovers_a_known_similarity_transform():
    """EM correctness in isolation: an exact posed copy comes back exactly.

    The first implementation failed THIS test at scale 0,14 - sigma2 initialised from the
    all-pairs mean made the first E-step uniform and collapsed the scale - so it stays as
    the regression pin for that failure.
    """
    rng = np.random.default_rng(0)
    Y = slab_surface_points((80.0, 30.0, 6.0), np.eye(4), 6.0)
    # A perfect box is exactly self-symmetric under 180-deg rotations, so the flipped
    # pose ties the true one to machine precision and the "recovered" rotation is
    # arbitrary - the first version of this test failed on exactly that. Real assemblies
    # break the symmetry through dims and defects; the test breaks it with a notch.
    Y = Y[~((Y[:, 0] > 25) & (Y[:, 1] > 8) & (Y[:, 2] > 0))]
    a = np.radians(25.0)
    R0 = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1.0]])
    t0 = np.array([30.0, -12.0, 7.0])
    X = 1.1 * Y @ R0.T + t0 + rng.normal(0, 0.05, Y.shape)
    s, R, t = cpd_similarity(X, Y, iters=50)
    assert abs(s - 1.1) < 0.01
    assert np.degrees(np.arccos(np.clip((np.trace(R0.T @ R) - 1) / 2, -1, 1))) < 0.5
    assert np.linalg.norm(t - t0) < 0.5


def test_the_oracle_target_features_are_what_make_it_work_and_are_labelled():
    """The two arms, measured on one scene each way.

    The paper's target carries model-derived edge features ("identical workpieces") - a
    stage a scan-only pipeline does not have, supplied here as an oracle like every other
    method's learned stage. Withholding it (`target_features="dense"`) registers raw
    surfaces, and a lap stack slides along its overlap: the arm exists so that failure is
    a measurement instead of a footnote.
    """
    prep = _bench_scene("lap")
    if prep is None:
        pytest.skip("no balanced corpus at out/bench")
    from baselines.metrics import matched_path_errors

    def rmse(tf):
        pred, aux = REGISTRY["lit-modelreg"].run(prep, 0, True, "full", 0.0,
                                                 target_features=tf)
        errs = [e["rmse"] for e in matched_path_errors(pred, prep.gt) if e["matched"]]
        return float(np.median(errs)) if errs else np.inf

    assert rmse("oracle") < 2.0
    assert rmse("dense") > rmse("oracle")


def test_deterministic_and_constitutively_l0():
    prep = _bench_scene("corner")
    if prep is None:
        pytest.skip("no balanced corpus at out/bench")
    df = run_matrix([prep], methods=["lit-modelreg"], seeds=range(5), verify_seeds=2)
    assert len(df) == 2                                # deterministic repeat policy
    assert df.f1.nunique() == 1
    assert not REGISTRY["lit-modelreg"].randomised
    assert REGISTRY["lit-modelreg"].oracle_name == "model"


def test_curved_primitives_build_a_model_from_their_meshes():
    """2026-09-14: the model for tubes / swept slabs / prepared plates comes from the exact
    D34 meshes, so `lit-modelreg` has a row on every stratum. Surface samples lie on the
    parts; sharp-edge samples are fewer and also on the parts."""
    import json
    from weldgen.geom import from_object
    from weldgen.render.gate import distance_to_mesh
    corpus = ROOT / "out" / "bench_phase4"
    picked = []
    for cls, want in (("T", "tube"), ("T", "swept_slab"), ("butt", "prepared_prism")):
        idx = corpus / cls / "index.jsonl"
        for ln in (idx.read_text().splitlines() if idx.exists() else []):
            r = json.loads(ln)
            if not r.get("emitted") or "scene_id" not in r:
                continue
            sd = corpus / cls / r["scene_id"]
            if not (sd / "scene.json").exists():
                continue
            sc = json.loads((sd / "scene.json").read_text())
            if any(o.get("primitive") == want for o in sc["objects"]):
                picked.append(sc); break
    if len(picked) < 3:
        pytest.skip("bench_phase4 corpus not present")
    for sc in picked:
        parts = [from_object(o) for o in sc["objects"] if o["role"] == "workpiece"]
        gt = [np.zeros((2, 3))]
        W_edges, _, Twj = build_model(sc, gt, edges_only=True)
        W_surf, _, _ = build_model(sc, gt, edges_only=False)
        assert len(W_surf) > len(W_edges) > 20
        for W in (W_edges, W_surf):
            world = W @ Twj[:3, :3].T + Twj[:3, 3]
            d = np.min(np.column_stack([distance_to_mesh(world, p.mesh()) for p in parts]), axis=1)
            assert np.percentile(d, 99) < 0.3, "model points must lie on the parts"


def test_spline_seam_stays_on_the_polyline_open_and_closed():
    from baselines.lit_modelreg import spline_seam
    t = np.linspace(0.0, 1.0, 1200)
    line = np.column_stack([200.0 * t, 5.0 * np.sin(2 * np.pi * t), np.zeros_like(t)])
    S = spline_seam(line, closed=False)
    d = np.min(np.linalg.norm(S[:, None, :] - line[None, :, :], axis=2), axis=1)
    assert 20 < len(S) < len(line) and d.max() < 0.2
    th = np.linspace(0.0, 2 * np.pi, 1500, endpoint=False)
    ring = np.column_stack([40.0 * np.cos(th), 40.0 * np.sin(th), np.zeros_like(th)])
    R = spline_seam(ring, closed=True)
    r = np.linalg.norm(R[:, :2], axis=1)
    assert abs(r - 40.0).max() < 0.2 and len(R) > 100


def test_sharp_edge_points_find_the_box_edges_not_its_faces():
    from baselines.lit_modelreg import sharp_edge_points
    rng = np.random.default_rng(0)
    n = 4000
    faces = []
    for axis in range(3):
        for sign in (-1.0, 1.0):
            q = rng.uniform(-50.0, 50.0, size=(n, 3)); q[:, axis] = 50.0 * sign
            faces.append(q)
    box = np.vstack(faces)
    E = sharp_edge_points(box, k=16, angle_deg=30.0)
    assert 0 < len(E) < 0.35 * len(box)
    # every edge feature lies within a few mm of a box edge (two coordinates near +-50)
    near = (np.abs(np.abs(E) - 50.0) < 4.0).sum(axis=1)
    assert (near >= 2).mean() > 0.9
