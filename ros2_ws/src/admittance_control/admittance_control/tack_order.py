"""Tack sequencing: gap-seeded farthest-first with a physics tie-break.

Step 3 of notes/tack_ordering_plan.md. Pure Python: no ROS, no weld_generator.

    sites  = [Site(id, seam, s_mm, xyz_mm, pair, gap_mm, length_mm), ...]   # reachable tacks
    seams  = {seam_id: SeamGeom(length_mm, closed)}
    result = sequence(sites, seams, SequenceConfig(), transit=None)
    result.order          # tack ids, first to last
    result.decided_by     # per step: which criterion chose it
    evaluate(order, sites, seams, cfg)   # r_k, hold step, rise at each firing, travel

Each step answers one question, in a stated priority, with no weights:

1. **Holding the parts (mechanics).**
   - The first tack between two parts goes where their measured gap is smallest, when it
     varies by more than its repeatability (`gap_repeat_mm`; the multi-view runs agreed to
     0.4 mm). Lowth et al. 2015 measured 19x less distortion tacking the contact end first.
     The assembly's first tack is the smallest such gap; with no such pair, the tack
     nearest the robot.
   - Every next tack is the candidate farthest from the tacks already placed (farthest-
     first traversal, Gonzalez 1985), in the SEAM metric: along the seam, the shorter way
     round a loop, and `far` (beyond any held-seam distance plus the tolerance) from a seam
     with no tack. So every seam is held once before any seam gets its second tack, and one
     open seam goes ends, middle, quarters.
2. **Heat (physics).** Candidates within `tol_mm` (one tack length) of the farthest are
   equivalent; among them, the coolest when it would be fired. The kernel is the
   instantaneous source in a thin sheet (Carslaw & Jaeger 1959, 10.2 III eq. 10, without
   its surface loss, whose time constant is minutes), summed over the placed tacks at a
   constant `step_s` per tack and the straight-line distance. It never adds waiting:
   Jonsson et al. 1985 found that letting the first tacks cool before the last ones raised
   the forces in the tacks, and advise tacking as fast as possible.
3. **Cycle time (robot).** Then the shortest transit from the previous tack (`transit`:
   the node passes the joint-space distance between the reachability report's approach
   poses; the default is the straight-line distance between tack centres, a proxy).
4. Then the lowest id.

The guarantee: after any k tacks every candidate lies within r_k of a placed tack (seam
metric), and r_k <= 2 r*_k + tol, r*_k being the best any k of the candidates could do.
Gonzalez's proof (Lemma 2.1, Theorem 2.2) puts the k heads and the farthest point pairwise
>= h apart; a head taken within `tol` of the farthest weakens that to h - tol, and the gap
seed is his arbitrary first head. He states it for the largest cluster diameter; the same
(k + 1)-clique bounds the covering radius, and since his algorithm picks the same heads
whatever k is, every prefix carries it.

The generator's `tackrule-0.1` order (`weldgen/tacks.py`) is a farthest-first traversal
on an open seam and on loops of 4 or 6 tacks. On loops of 8 or more it is not: it bisects
the half-loop ends first, so after 0 and 180 deg it takes 135, where this takes 90. On 8
the covering radius still matches at every prefix; on loops of 10 and 12 the rule's is
twice this one's after 4 and 5 tacks (checked 2026-10-27).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from itertools import combinations
from typing import Callable, Sequence

import numpy as np


@dataclass(frozen=True)
class Site:
    """One candidate tack (a reachable one)."""
    id: int
    seam: int
    s_mm: float                      # arclength of the tack centre along its seam
    xyz_mm: tuple[float, float, float]
    pair: str = ""                   # the two parts its seam joins ("A|B"); the gap rule is per pair
    gap_mm: float | None = None      # measured signed gap of the abutting member there
    length_mm: float = 0.0           # tack length


@dataclass(frozen=True)
class SeamGeom:
    length_mm: float
    closed: bool = False


@dataclass(frozen=True)
class Thermal:
    """Thin-sheet instantaneous source, order of magnitude only (the plan's table).

    Mild steel by default; 304 stainless is about k 16 W/m K, a 4e-6 m^2/s. The heat
    into the part per mm of tack is nominal (the cell does not weld), so the reported
    rises are estimates; the ranking needs only `a_m2_s` and the timing, and Q only sets
    what `heat_tie_c` means."""
    k_w_mk: float = 45.0
    a_m2_s: float = 1.2e-5
    q_j_per_mm: float = 500.0
    h_mm: float = 8.0                # sheet thickness: the thinnest member's gauge

    def rise_c(self, r_mm, tau_s, tack_mm) -> np.ndarray:
        """Temperature rise (K) at distance `r_mm`, `tau_s` after a tack `tack_mm` long."""
        r = np.asarray(r_mm, float) / 1000.0
        tau = np.asarray(tau_s, float)
        q = self.q_j_per_mm * np.asarray(tack_mm, float)
        with np.errstate(divide="ignore", invalid="ignore", over="ignore"):
            v = q / (4.0 * np.pi * self.k_w_mk * self.h_mm / 1000.0 * tau) \
                * np.exp(-r ** 2 / (4.0 * self.a_m2_s * tau))
        return np.where(tau > 0, v, 0.0)


@dataclass(frozen=True)
class SequenceConfig:
    tol_mm: float | None = None      # equivalent within this of the farthest; None = longest tack
    gap_repeat_mm: float = 0.4       # a gap that varies less than this is not information
    step_s: float = 15.0             # time per tack (transit + mark) for the heat model
    heat_tie_c: float = 1.0          # rises within this count as equal
    thermal: Thermal = field(default_factory=Thermal)


@dataclass
class SequenceResult:
    order: list[int]                 # tack ids
    #: per step, the criteria that narrowed the candidates in turn ("farthest>heat",
    #: "gap>id": the gap chose where, the id which of two equal sides); "only" = the last
    decided_by: list[str]
    tol_mm: float
    far_mm: float


@dataclass
class Metrics:
    r_mm: list[float]                # covering radius after each prefix (far while a seam is unheld)
    hold_step: int | None            # tacks placed when every seam first has one
    rise_c: list[float]              # predicted rise at each tack when it is fired
    travel: float                    # summed transit cost (the transit function's units)
    far_mm: float


def _far(sites: Sequence[Site], seams: dict[int, SeamGeom], tol: float) -> float:
    longest = max((seams[s.seam].length_mm for s in sites), default=0.0)
    return float(longest + tol + 1.0)


def seam_distance(a: Site, b: Site, seams: dict[int, SeamGeom], far: float) -> float:
    """The seam metric: along the seam, the shorter way round a loop, `far` across seams.
    A metric (triangle inequality) for any `far` >= half the longest seam."""
    if a.seam != b.seam:
        return far
    d = abs(a.s_mm - b.s_mm)
    g = seams[a.seam]
    if g.closed and g.length_mm > 0:
        d = d % g.length_mm
        d = min(d, g.length_mm - d)
    return float(d)


def _matrix(sites: Sequence[Site], seams: dict[int, SeamGeom], far: float) -> np.ndarray:
    n = len(sites)
    D = np.zeros((n, n))
    for i in range(n):
        for j in range(i + 1, n):
            D[i, j] = D[j, i] = seam_distance(sites[i], sites[j], seams, far)
    return D


def _euclid(sites_by_id: dict[int, Site]) -> Callable[[int | None, int], float]:
    def cost(prev: int | None, cand: int) -> float:
        if prev is None:
            return 0.0
        return float(np.linalg.norm(np.subtract(sites_by_id[prev].xyz_mm,
                                                sites_by_id[cand].xyz_mm)))
    return cost


def gap_entries(sites: Sequence[Site], repeat_mm: float) -> dict[str, list[int]]:
    """Per part pair whose measured gap varies by more than `repeat_mm` over its tacks:
    the indices (into `sites`) of the tacks at its smallest gap."""
    out: dict[str, list[int]] = {}
    pairs: dict[str, list[int]] = {}
    for i, s in enumerate(sites):
        if s.gap_mm is not None:
            pairs.setdefault(s.pair, []).append(i)
    for p, idx in pairs.items():
        g = np.array([sites[i].gap_mm for i in idx], float)
        if g.max() - g.min() > repeat_mm:
            out[p] = [i for i, v in zip(idx, g) if v <= g.min() + 1e-9]
    return out


def sequence(sites: Sequence[Site], seams: dict[int, SeamGeom],
             cfg: SequenceConfig | None = None,
             transit: Callable[[int | None, int], float] | None = None) -> SequenceResult:
    """The sequence of `sites` (see the module docstring). `transit(prev_id, cand_id)`:
    the cost of moving to a candidate, `prev_id` None for the first move (from where the
    robot is); default, the straight-line distance between tack centres (0 at the start)."""
    cfg = cfg or SequenceConfig()
    sites = sorted(sites, key=lambda s: s.id)                    # input order never matters
    n = len(sites)
    if n == 0:
        return SequenceResult([], [], 0.0, 0.0)
    tol = float(cfg.tol_mm if cfg.tol_mm is not None else max(s.length_mm for s in sites))
    far = _far(sites, seams, tol)
    D = _matrix(sites, seams, far)
    cost = transit or _euclid({s.id: s for s in sites})
    entries = gap_entries(sites, cfg.gap_repeat_mm)
    nearest = np.full(n, far)
    placed: list[int] = []
    order, decided = [], []
    remaining = list(range(n))
    while remaining:
        k = len(placed)
        held = {sites[p].pair for p in placed}
        if k == 0 and entries:
            # the assembly's first tack: the smallest gap of the pairs whose gap is information
            pool = sorted({i for v in entries.values() for i in v})
            g = min(sites[i].gap_mm for i in pool)
            cand = [i for i in pool if sites[i].gap_mm <= g + 1e-9]
            chain = ["gap"]
        else:
            dmax = max(nearest[i] for i in remaining)
            cand = [i for i in remaining if nearest[i] >= dmax - tol - 1e-9]
            chain = ["farthest"]
            # an unheld pair is entered at its smallest gap
            keep = [i for i in cand if sites[i].pair in held or sites[i].pair not in entries
                    or i in entries[sites[i].pair]]
            if len(keep) < len(cand):
                cand = keep
                chain.append("gap")
        if len(cand) > 1:
            t_fire = k * cfg.step_s
            rise = [_rise(sites, placed, cfg, t_fire, i) for i in cand]
            lo = min(rise)
            keep = [i for i, r in zip(cand, rise) if r <= lo + cfg.heat_tie_c]
            if len(keep) < len(cand):
                cand = keep
                chain.append("heat")
        if len(cand) > 1:
            prev = sites[placed[-1]].id if placed else None
            c = [cost(prev, sites[i].id) for i in cand]
            lo = min(c)
            keep = [i for i, v in zip(cand, c) if v <= lo + 1e-9 * max(1.0, abs(lo))]
            if len(keep) < len(cand):
                cand = keep
                chain.append("transit")
        if len(cand) > 1:
            chain.append("id")
        pick = min(cand)
        placed.append(pick)
        remaining.remove(pick)
        nearest = np.minimum(nearest, D[pick])
        order.append(sites[pick].id)
        decided.append(">".join(chain) if remaining or k == 0 else "only")
    return SequenceResult(order, decided, tol, far)


def _rise(sites: Sequence[Site], placed: Sequence[int], cfg: SequenceConfig,
          t_fire: float, cand: int) -> float:
    """The rise at `cand` fired at `t_fire`, the k placed tacks fired at 0, step, 2 step..."""
    if not placed:
        return 0.0
    x = np.asarray(sites[cand].xyz_mm, float)
    P = np.array([sites[p].xyz_mm for p in placed], float)
    tau = t_fire - cfg.step_s * np.arange(len(placed))
    r = np.linalg.norm(P - x, axis=1)
    return float(np.sum(cfg.thermal.rise_c(r, tau, [sites[p].length_mm for p in placed])))


def evaluate(order: Sequence[int], sites: Sequence[Site], seams: dict[int, SeamGeom],
             cfg: SequenceConfig | None = None,
             transit: Callable[[int | None, int], float] | None = None) -> Metrics:
    """The plan's metrics for ANY order of the same sites (today's seam-by-seam, the
    rule's, this one's): r_k, the hold step, the predicted rise at each firing, travel."""
    cfg = cfg or SequenceConfig()
    sites = sorted(sites, key=lambda s: s.id)
    if not sites:
        return Metrics([], None, [], 0.0, 0.0)
    pos = {s.id: i for i, s in enumerate(sites)}
    tol = float(cfg.tol_mm if cfg.tol_mm is not None else max(s.length_mm for s in sites))
    far = _far(sites, seams, tol)
    D = _matrix(sites, seams, far)
    cost = transit or _euclid({s.id: s for s in sites})
    nearest = np.full(len(sites), far)
    seams_all = {s.seam for s in sites}
    held: set[int] = set()
    r, rise, hold, travel, placed, prev = [], [], None, 0.0, [], None
    for k, tid in enumerate(order):
        i = pos[tid]
        rise.append(_rise(sites, placed, cfg, k * cfg.step_s, i))
        travel += cost(prev, tid)
        placed.append(i)
        nearest = np.minimum(nearest, D[i])
        r.append(float(nearest.max()))
        held.add(sites[i].seam)
        if hold is None and held == seams_all:
            hold = k + 1
        prev = tid
    return Metrics(r, hold, rise, float(travel), far)


def best_radius(sites: Sequence[Site], seams: dict[int, SeamGeom], k: int, far: float) -> float:
    """r*_k by brute force: the smallest covering radius any k of the candidates achieve
    (small cases only - the test of the guarantee)."""
    sites = sorted(sites, key=lambda s: s.id)
    D = _matrix(sites, seams, far)
    best = np.inf
    for sub in combinations(range(len(sites)), k):
        best = min(best, float(D[:, list(sub)].min(axis=1).max()))
    return best
