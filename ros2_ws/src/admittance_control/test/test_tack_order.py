"""Tack sequencing (notes/tack_ordering_plan.md step 3). Plain pytest, no ROS.

Claims: one open seam goes ends, middle, quarters, with the same covering radius at every
prefix as the generator's `tackrule-0.1` order; a loop of four goes 0, 180, 90, 270; on a
loop of ten the rule's third tack is not the farthest and the sequence covers better; two
parallel seams are each held once before either gets a second tack; on the two sides of a
web the second tack goes to the far end of the other side, not across from the first
(where the rule's order fires into ~200 C); after every prefix r_k <= 2 r*_k + tol against
brute force on random small cases; the result is deterministic and does not depend on the
input order; the gap seed takes the contact end when the gap varies by more than its
repeatability, and only then; a pair entered later is entered at its own smallest gap; and
the web order survives +-30 % on every thermal constant.
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

PKG = pathlib.Path(__file__).resolve().parents[1]
WS = PKG.parents[2]
sys.path.insert(0, str(PKG))
sys.path.insert(0, str(WS / "weld_generator"))

from admittance_control import tack_order as to  # noqa: E402

wt = pytest.importorskip("weldgen.tacks")


def _open_seam(seam, L, ss, y=0.0, pair="A|B", gaps=None, tack=32.0, id0=0):
    sites = [to.Site(id=id0 + k, seam=seam, s_mm=float(s), xyz_mm=(float(s), y, 0.0), pair=pair,
                     gap_mm=None if gaps is None else float(gaps[k]), length_mm=tack)
             for k, s in enumerate(ss)]
    return sites, {seam: to.SeamGeom(L, False)}


def _loop(seam, r, n, tack=12.0, id0=0, centre=(0.0, 0.0)):
    L = 2 * np.pi * r
    sites = [to.Site(id=id0 + k, seam=seam, s_mm=k * L / n,
                     xyz_mm=(centre[0] + r * np.cos(2 * np.pi * k / n),
                             centre[1] + r * np.sin(2 * np.pi * k / n), 0.0), length_mm=tack)
             for k in range(n)]
    return sites, {seam: to.SeamGeom(L, True)}


def _rule_positions(L, t):
    _, ss = wt._counts_open(L, t, wt.DEFAULT_PARAMS)
    return ss


def test_one_open_seam_is_the_rules_order_by_covering_radius():
    for L, t in [(250.0, 3.0), (400.0, 2.0), (600.0, 1.5)]:
        ss = _rule_positions(L, t)
        sites, seams = _open_seam(0, L, ss, tack=float(np.clip(4 * t, 10, 50)))
        res = to.sequence(sites, seams)
        n = len(ss)
        assert n >= 3
        assert set(res.order[:2]) == {0, n - 1}                    # both ends first
        assert res.order[2] == (n - 1) // 2 or res.order[2] == n // 2   # then the middle
        ours = to.evaluate(res.order, sites, seams).r_mm
        rule = to.evaluate(wt._bisect_order(n), sites, seams).r_mm
        assert ours == pytest.approx(rule, abs=1e-9)


def test_a_loop_of_four_goes_0_180_90_270():
    sites, seams = _loop(0, 31.0, 4)
    res = to.sequence(sites, seams)
    assert [90 * i for i in res.order] == [0, 180, 90, 270]
    assert res.decided_by[1] == "farthest"                        # 180: the only farthest


def test_on_a_loop_of_ten_the_rules_third_tack_is_not_the_farthest():
    sites, seams = _loop(0, 80.0, 10)
    half = 5
    rule = [j for i in wt._bisect_order(half) for j in (i, i + half)]
    assert [36 * i for i in rule[:3]] == [0, 180, 144]               # 144 is 36 deg from 180
    res = to.sequence(sites, seams)
    assert {36 * i for i in res.order[2:4]} <= {72, 108, 252, 288}   # 72 deg from both
    ours = to.evaluate(res.order, sites, seams).r_mm
    theirs = to.evaluate(rule, sites, seams).r_mm
    assert ours[3] < theirs[3] - 1.0                                  # 4 tacks: half the radius
    assert all(a <= b + 1e-9 for a, b in zip(ours, theirs))


def test_two_parallel_seams_are_each_held_before_any_second_tack():
    # 400 mm apart: farther by travel than the rest of the first seam, so only the seam
    # metric (an unheld seam is farthest of all) sends the second tack there
    a, sa = _open_seam(0, 250.0, [20.0, 125.0, 230.0], y=0.0)
    b, sb = _open_seam(1, 250.0, [20.0, 125.0, 230.0], y=400.0, id0=10)
    sites, seams = a + b, {**sa, **sb}
    res = to.sequence(sites, seams)
    first_two = {s.seam for s in sites if s.id in res.order[:2]}
    assert first_two == {0, 1}
    assert to.evaluate(res.order, sites, seams).hold_step == 2


def _web(t=8.0, L=250.0):
    """The two fillets of a T at t = 8: tacks at 32 and 218 mm on each side, t apart."""
    a, sa = _open_seam(0, L, [32.0, 218.0], y=-t / 2)
    b, sb = _open_seam(1, L, [32.0, 218.0], y=+t / 2, id0=2)
    return a + b, {**sa, **sb}


def test_on_a_web_the_second_tack_goes_to_the_far_end_of_the_other_side():
    sites, seams = _web()
    res = to.sequence(sites, seams)
    assert res.order[:2] == [0, 3]                                   # side 0 at 32, side 1 at 218
    assert res.decided_by[1] == "farthest>heat"
    ours = to.evaluate(res.order, sites, seams)
    # the rule interleaves the two sides round-robin: its second tack is across the web
    rule = to.evaluate([0, 2, 1, 3], sites, seams)
    assert rule.rise_c[1] > 100.0 and ours.rise_c[1] < 1.0
    assert ours.hold_step == rule.hold_step == 2


def test_the_web_order_survives_30_percent_on_the_thermal_constants():
    sites, seams = _web()
    base = to.sequence(sites, seams).order
    th = to.Thermal()
    for f in (0.7, 1.3):
        for field_ in ("k_w_mk", "a_m2_s", "q_j_per_mm", "h_mm"):
            cfg = to.SequenceConfig(thermal=to.Thermal(**{**th.__dict__,
                                                          field_: getattr(th, field_) * f}))
            assert to.sequence(sites, seams, cfg).order == base, (field_, f)


def test_guarantee_against_brute_force():
    rng = np.random.default_rng(7)
    tol = 12.0
    for _ in range(60):
        sites, seams, nid = [], {}, 0
        for sid in range(int(rng.integers(1, 4))):
            closed = bool(rng.random() < 0.4)
            L = float(rng.uniform(120.0, 320.0))
            n = int(rng.integers(2, 4))
            ss = np.sort(rng.uniform(0.0, L, n))
            for s in ss:
                sites.append(to.Site(id=nid, seam=sid, s_mm=float(s),
                                     xyz_mm=tuple(rng.uniform(-150, 150, 3)), pair=f"p{sid % 2}",
                                     gap_mm=float(rng.uniform(0.0, 2.0)), length_mm=tol))
                nid += 1
            seams[sid] = to.SeamGeom(L, closed)
        res = to.sequence(sites, seams, to.SequenceConfig(tol_mm=tol))
        r = to.evaluate(res.order, sites, seams, to.SequenceConfig(tol_mm=tol)).r_mm
        for k in range(1, len(sites) + 1):
            assert r[k - 1] <= 2 * to.best_radius(sites, seams, k, res.far_mm) + tol + 1e-9


def test_deterministic_and_independent_of_the_input_order():
    a, sa = _open_seam(0, 300.0, [24.0, 100.0, 200.0, 276.0], gaps=[0.2, 0.5, 0.9, 1.4])
    b, sb = _loop(1, 40.0, 6, id0=10, centre=(150.0, 120.0))
    sites, seams = a + b, {**sa, **sb}
    first = to.sequence(sites, seams)
    assert to.sequence(sites, seams).order == first.order
    for seed in range(5):
        shuffled = list(np.random.default_rng(seed).permutation(sites))
        assert to.sequence(shuffled, seams).order == first.order


def test_the_gap_seed_takes_the_contact_end_only_when_the_gap_is_information():
    ss = [24.0, 125.0, 226.0]
    sites, seams = _open_seam(0, 250.0, ss, gaps=[1.5, 0.8, 0.1])   # varies 1.4 mm
    res = to.sequence(sites, seams)
    assert res.order[0] == 2 and res.decided_by[0] == "gap"
    sites, seams = _open_seam(0, 250.0, ss, gaps=[0.4, 0.3, 0.1])   # varies 0.3 < 0.4
    res = to.sequence(sites, seams)
    assert res.order[0] == 0 and "gap" not in res.decided_by[0]


def test_a_pair_entered_later_is_entered_at_its_own_smallest_gap():
    a, sa = _open_seam(0, 250.0, [24.0, 226.0], y=0.0, pair="A|B", gaps=[0.0, 1.2])
    b, sb = _open_seam(1, 250.0, [24.0, 125.0, 226.0], y=300.0, pair="B|C",
                       gaps=[1.6, 1.1, 0.2], id0=10)
    sites, seams = a + b, {**sa, **sb}
    res = to.sequence(sites, seams)
    assert res.order[0] == 0                                         # the smallest gap of all
    assert res.order[1] == 12 and res.decided_by[1] == "farthest>gap"  # B|C at its 0.2 mm end
