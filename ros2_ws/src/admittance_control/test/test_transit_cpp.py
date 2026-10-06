"""The C++ collision model and OMPL APS (src/transit_cpp.cpp, notes/aps_transit_plan.md
steps 1-3) against the Python model they port (collision.py, kinematics.py).

1. FK: the C++ frames equal `ur5e_link_frames_params` (nominal and the calibration).
2. Collision: `is_valid` IDENTICAL and the min distance + closest pair equal on random and
   near-contact configurations, on the T-joint test scene and on a real session.
3. APS: a valid, short front<->back transit within the budget, on several cores.

Needs the built extension (colcon build); skipped without it.
"""

from __future__ import annotations

import json
import pathlib
import sys
import time

import numpy as np
import pytest

PKG = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))
sys.path.insert(0, str(PKG / "test"))
sys.path.insert(0, str(PKG.parents[2] / "weld_generator"))

tc = pytest.importorskip("admittance_control._transit_cpp")

from admittance_control import kinematics as kin  # noqa: E402
from admittance_control import marking as mk  # noqa: E402
from admittance_control.collision import CollisionModel, boxes_from_parts  # noqa: E402
from admittance_control.kinematics import _edge_valid  # noqa: E402
from admittance_control.tool_model import load_tool_model  # noqa: E402

from test_marking import scene  # noqa: E402,F401  (fixture: the T-joint scene)

FRAMES = ("shoulder", "lift", "elbow", "wrist_1", "wrist_2", "wrist_3", "tool0")
HOME = np.array([-0.34, -1.44, -1.56, -1.55, 1.94, -0.86])
CALIB = PKG / "config" / "ur5e_calibration.yaml"


@pytest.fixture
def calibrated():
    kin.use_kinematics(str(CALIB))
    yield
    kin.use_kinematics(None)


def _random_q(n, rng):
    """Half near a working pose (where the scene is), half anywhere within the limits."""
    lo = np.array([a for a, _ in kin.JOINT_LIMITS]); hi = np.array([b for _, b in kin.JOINT_LIMITS])
    near = HOME + rng.normal(0, 0.8, (n // 2, 6))
    anywhere = rng.uniform(lo, hi, (n - n // 2, 6))
    return np.vstack([np.clip(near, lo - 0.2, hi + 0.2), anywhere])   # a few out of limits too


def _near_contact(model, n, rng):
    """Configurations within ~1e-4 rad of the clearance boundary: bisect edges whose two
    ends disagree. These are where a port is most likely to differ."""
    out = []
    while len(out) < n:
        a, b = HOME + rng.normal(0, 0.8, 6), HOME + rng.normal(0, 0.8, 6)
        va, vb = model.is_valid(a), model.is_valid(b)
        if va == vb:
            continue
        for _ in range(14):
            m = 0.5 * (a + b)
            if model.is_valid(m) == va:
                a = m
            else:
                b = m
        out += [a, b]
    return np.array(out[:n])


def _real_session_model():
    from admittance_control import seam_from_registration as sfr
    from admittance_control.weldgen_registry import load_registry
    p = PKG / "scripts" / "foundationpose_results" / "assembly.json"
    if not p.exists():
        pytest.skip("no real session in scripts/foundationpose_results")
    objs = [(o["model"], np.asarray(o["pose_static"], float).reshape(4, 4))
            for o in json.loads(p.read_text())["objects"]]
    parts = sfr.posed_parts(objs, load_registry(str(PKG / "models" / "weldgen_objects.json")))
    return CollisionModel(tool=load_tool_model(), scene_boxes=boxes_from_parts(parts),
                          table_z=None, clearance=0.02)


# ---------------------------------------------------------------- 1. FK ----------------
@pytest.mark.parametrize("which", ["nominal", "calibrated"])
def test_fk_matches_python(which, request):
    if which == "calibrated":
        request.getfixturevalue("calibrated")
    m = tc.Model(CollisionModel(tool=load_tool_model()).export_spec())
    rng = np.random.default_rng(1)
    worst = 0.0
    for q in rng.uniform(-np.pi, np.pi, (1000, 6)):
        Fp = kin.ur5e_link_frames(q)
        Fc = m.frames(q)
        worst = max(worst, max(np.abs(Fc[i] - Fp[name]).max() for i, name in enumerate(FRAMES)))
    assert worst < 1e-12, worst


# ---------------------------------------------------------------- 2. collision ---------
def _assert_equivalent(model, Q):
    m = tc.Model(model.export_spec())
    mismatched = []
    for q in Q:
        dp, ap, bp = model.min_distance(q)
        dc, ac, bc = m.min_distance(q)
        if model.is_valid(q) != m.is_valid(q):
            mismatched.append(("is_valid", q, dp, dc))
        elif np.isfinite(dp) or np.isfinite(dc):
            same_pair = (ap, bp) == (ac, bc)
            if not same_pair:
                # a TIE (2026-10-06: flange_adapter and camera_arm both 52.31 mm from the
                # shoulder, equal to 3e-17) is decided by the last bit of rounding: accept
                # the other name only if Python also puts that pair at the minimum
                d_py = {(a, b): d for d, a, b in model.pair_distances(q)}
                same_pair = abs(d_py.get((ac, bc), np.inf) - dp) <= 1e-9
            if not (abs(dp - dc) <= 1e-9 and same_pair):
                mismatched.append(("distance", q, (dp, ap, bp), (dc, ac, bc)))
    assert not mismatched, f"{len(mismatched)}/{len(Q)} differ, first: {mismatched[0]}"
    # the vectorised call agrees with the single one
    many = m.is_valid_many(Q)
    assert all(many[i] == model.is_valid(q) for i, q in enumerate(Q[:500]))


def test_collision_equivalent_on_the_t_joint_scene(scene):  # noqa: F811
    tool, cfg, model, report = scene
    rng = np.random.default_rng(2)
    Q = np.vstack([_random_q(4000, rng), _near_contact(model, 400, rng)])
    _assert_equivalent(model, Q)


def test_collision_equivalent_on_a_real_session_with_table(calibrated):
    model = _real_session_model()
    model.table_z = -0.096                                   # the measured bench table
    rng = np.random.default_rng(3)
    Q = np.vstack([_random_q(4000, rng), _near_contact(model, 400, rng)])
    _assert_equivalent(model, Q)


def test_cpp_checks_are_much_faster(scene):  # noqa: F811
    tool, cfg, model, report = scene
    m = tc.Model(model.export_spec())
    Q = _random_q(2000, np.random.default_rng(4))
    t = time.perf_counter(); [model.is_valid(q) for q in Q[:300]]; py = (time.perf_counter() - t) / 300
    t = time.perf_counter(); m.is_valid_many(Q); cpp = (time.perf_counter() - t) / len(Q)
    print(f"\nvalidity check: python {py * 1e6:.0f} us, C++ {cpp * 1e6:.1f} us ({py / cpp:.0f}x)")
    assert py / cpp > 20


# ---------------------------------------------------------------- 3. APS ---------------
def _front_back(scene):
    tool, cfg, model, report = scene
    tacks = report["tacks"]
    q_a = np.asarray(next(t for t in tacks if t["seam_id"] == 0)["q_app"])
    q_b = mk._unwrap_to(np.asarray(next(t for t in tacks if t["seam_id"] == 1)["q_app"]), q_a)
    return q_a, q_b, mk.transit_model(model, cfg, q_a, q_b)


def test_aps_finds_a_valid_short_transit_within_budget(scene):  # noqa: F811
    q_a, q_b, m_py = _front_back(scene)
    m = tc.Model(m_py.export_spec())
    budget = 1.0
    cpu0, wall0 = time.process_time(), time.perf_counter()
    res = tc.plan(m, q_a, q_b, planner="aps", budget_s=budget, num_planners=4)
    cpu, wall = time.process_time() - cpu0, time.perf_counter() - wall0
    assert res["solved"], res["status"]
    path = [np.asarray(q) for q in res["path"]]
    assert np.allclose(path[0], q_a) and np.allclose(path[-1], q_b)
    for x, y in zip(path[:-1], path[1:]):                    # the PYTHON model agrees
        assert _edge_valid(x, y, resolution=0.01, is_valid=m_py.is_valid)
    assert res["time_s"] <= budget + 0.3
    print(f"\nAPS: cost {res['cost']:.3f}, {len(path)} waypoints, {res['solutions']} solutions, "
          f"{res['validity_calls']} checks, {res['time_s']:.2f} s wall, CPU/wall {cpu / wall:.1f}")
    assert cpu / wall > 1.5                                  # the planner threads ran in parallel


def test_aps_is_shorter_than_a_single_rrt_connect(scene):  # noqa: F811
    q_a, q_b, m_py = _front_back(scene)
    m = tc.Model(m_py.export_spec())
    aps = tc.plan(m, q_a, q_b, planner="aps", budget_s=1.0)
    rrtc = [tc.plan(m, q_a, q_b, planner="rrt_connect", budget_s=1.0) for _ in range(5)]
    costs = [r["cost"] for r in rrtc if r["solved"]]
    assert aps["solved"] and costs
    print(f"\ncost: APS {aps['cost']:.3f} vs RRT-Connect median {np.median(costs):.3f}")
    assert aps["cost"] <= np.median(costs)


def test_unknown_planner_is_refused(scene):  # noqa: F811
    q_a, q_b, m_py = _front_back(scene)
    with pytest.raises(ValueError):
        tc.plan(tc.Model(m_py.export_spec()), q_a, q_b, planner="nope")


# ---------------------------------------------------------------- 4. integration -------
def test_full_marking_plan_with_aps(scene):  # noqa: F811
    tool, cfg, model, report = scene
    plan = mk.build_marking_plan(report, tool, model, cfg, cfg.home_q, smooth_transits=True,
                                 transit_opts={"planner": "aps", "budget_s": 0.5})
    assert plan.ok, plan.summary()
    hows = [s.transit_how for s in plan.steps] + [plan.home_how]
    assert all(h == "straight" or h.startswith("APS") for h in hows), hows
    for s in plan.steps:
        assert np.allclose(s.transit[-1], s.q_app) and np.allclose(s.descent[0], s.q_app)
    print("\n" + plan.summary())


def test_aps_falls_back_to_rrt_connect_without_the_extension(scene, monkeypatch):  # noqa: F811
    q_a, q_b, m_py = _front_back(scene)
    import admittance_control
    monkeypatch.setitem(sys.modules, "admittance_control._transit_cpp", None)   # import fails
    monkeypatch.delattr(admittance_control, "_transit_cpp", raising=False)      # (cached on the package)
    info = {}
    path = mk.transit_path(q_a, q_b, m_py, planner="aps", info=info)
    assert path is not None
    assert info["how"].startswith("RRT-Connect (APS fallback: _transit_cpp not built")
