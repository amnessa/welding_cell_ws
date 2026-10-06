#!/usr/bin/env python3
"""
make_synthetic.py - synthetic weld scenes with ground-truth seams.

For every scenario it writes
  base_<name>.ply      the base part A alone (complete surface)
  combined_<name>.ply  A + B in weld position, with contact faces removed
                       (points of one part lying on/inside the other are hidden)
  gt_<name>.json       ground-truth seam polylines

Scenarios: tjoint, tjoint_angled, lap, butt, rod_on_plate (circular seam),
           plate_on_pipe (curved seam on a curved base).
Units are millimetres.
"""
import argparse
import json
import os

import numpy as np

from weld_seam import write_ply


def rot_y(deg):
    a = np.radians(deg)
    return np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])


def grid(lu, lv, s):
    a = np.linspace(-lu / 2, lu / 2, max(int(np.ceil(lu / s)), 1) + 1)
    b = np.linspace(-lv / 2, lv / 2, max(int(np.ceil(lv / s)), 1) + 1)
    A, B = np.meshgrid(a, b, indexing="ij")
    return A.ravel(), B.ravel()


class Box:
    def __init__(self, center, size, R=np.eye(3)):
        self.c, self.h, self.R = np.asarray(center, float), np.asarray(size, float) / 2, R

    def sample(self, s):
        hx, hy, hz = self.h
        pts = []
        for ax in range(3):
            o = [i for i in range(3) if i != ax]
            u, v = grid(2 * self.h[o[0]], 2 * self.h[o[1]], s)
            for sgn in (-1, 1):
                P = np.zeros((len(u), 3))
                P[:, ax] = sgn * self.h[ax]
                P[:, o[0]], P[:, o[1]] = u, v
                pts.append(P)
        return np.vstack(pts) @ self.R.T + self.c

    def inside(self, P, eps):
        L = (P - self.c) @ self.R
        return np.all(np.abs(L) <= self.h + eps, axis=1)


class Rod:
    """Solid cylinder standing on z (bottom centre, radius, height)."""
    def __init__(self, base, radius, height):
        self.b, self.r, self.hgt = np.asarray(base, float), radius, height

    def sample(self, s):
        n_t = int(np.ceil(2 * np.pi * self.r / s))
        n_z = int(np.ceil(self.hgt / s)) + 1
        t, z = np.meshgrid(np.linspace(0, 2 * np.pi, n_t, endpoint=False),
                           np.linspace(0, self.hgt, n_z), indexing="ij")
        side = np.c_[self.r * np.cos(t.ravel()), self.r * np.sin(t.ravel()), z.ravel()]
        u, v = grid(2 * self.r, 2 * self.r, s)
        m = u ** 2 + v ** 2 <= self.r ** 2
        caps = [np.c_[u[m], v[m], np.full(m.sum(), zz)] for zz in (0, self.hgt)]
        return np.vstack([side] + caps) + self.b

    def inside(self, P, eps):
        L = P - self.b
        return (np.hypot(L[:, 0], L[:, 1]) <= self.r + eps) & (L[:, 2] >= -eps) & (L[:, 2] <= self.hgt + eps)


class Pipe:
    """Outer surface of a pipe along y, centred at the origin."""
    def __init__(self, radius, length):
        self.r, self.L = radius, length

    def sample(self, s):
        n_t = int(np.ceil(2 * np.pi * self.r / s))
        t, y = np.meshgrid(np.linspace(0, 2 * np.pi, n_t, endpoint=False),
                           np.linspace(-self.L / 2, self.L / 2, int(np.ceil(self.L / s)) + 1),
                           indexing="ij")
        return np.c_[self.r * np.cos(t.ravel()), y.ravel(), self.r * np.sin(t.ravel())]

    def inside(self, P, eps):
        return (np.hypot(P[:, 0], P[:, 2]) <= self.r + eps) & (np.abs(P[:, 1]) <= self.L / 2 + eps)


def rect_loop(corners):
    c = [list(map(float, p)) for p in corners]
    return c + [c[0]]


def scenario(name):
    if name == "tjoint":
        A = Box([0, 0, 5], [200, 150, 10])
        B = Box([0, 0, 50], [8, 120, 80])
        gt = [rect_loop([[-4, -60, 10], [-4, 60, 10], [4, 60, 10], [4, -60, 10]])]
    elif name == "tjoint_angled":            # standing plate leaning 30 deg
        A = Box([0, 0, 10], [200, 150, 20])
        R = rot_y(30)
        c = np.array([0, 0, 20.0]) + R @ np.array([0, 0, 50.0])
        B = Box(c, [8, 120, 120], R)
        nx = R[:, 0]
        xs = [c[0] + (s * 4 - (20 - c[2]) * nx[2]) / nx[0] for s in (-1, 1)]
        gt = [rect_loop([[xs[0], -60, 20], [xs[0], 60, 20], [xs[1], 60, 20], [xs[1], -60, 20]])]
    elif name == "lap":
        A = Box([0, 0, 3], [200, 150, 6])
        B = Box([0, 85, 9], [160, 100, 6])   # y in [35, 135], overlap y in [35, 75]
        gt = [rect_loop([[-80, 35, 6], [80, 35, 6], [80, 75, 6], [-80, 75, 6]])]
    elif name == "butt":
        A = Box([-50, 0, 3], [100, 150, 6])
        B = Box([50, 0, 3], [100, 150, 6])
        gt = [rect_loop([[0, -75, 0], [0, 75, 0], [0, 75, 6], [0, -75, 6]])]
    elif name == "rod_on_plate":
        A = Box([0, 0, 5], [200, 200, 10])
        B = Rod([0, 0, 10], 30, 80)
        t = np.linspace(0, 2 * np.pi, 361)
        gt = [np.c_[30 * np.cos(t), 30 * np.sin(t), np.full_like(t, 10)].tolist()]
    elif name == "plate_on_pipe":
        A = Pipe(60, 200)
        B = Box([0, 0, 90], [80, 8, 120])     # bottom inside the pipe, clipped by it
        x = np.linspace(-40, 40, 161)
        z = np.sqrt(60 ** 2 - x ** 2)
        side1 = np.c_[x, np.full_like(x, -4), z]
        side2 = np.c_[x[::-1], np.full_like(x, 4), z[::-1]]
        gt = [np.vstack([side1, side2, side1[:1]]).tolist()]
    else:
        raise ValueError(name)
    return A, B, gt


SCENARIOS = ["tjoint", "tjoint_angled", "lap", "butt", "rod_on_plate", "plate_on_pipe"]


def generate(name, out, spacing=1.0, noise=0.05, perturb=0.0, seed=0):
    rng = np.random.default_rng(seed)
    A, B, gt = scenario(name)
    pa, pb = A.sample(spacing), B.sample(spacing)
    eps = 0.3 * spacing
    combined = np.vstack([pa[~B.inside(pa, eps)], pb[~A.inside(pb, eps)]])
    base = pa.copy()
    base += rng.normal(0, noise, base.shape)
    combined += rng.normal(0, noise, combined.shape)
    if perturb > 0:   # small rigid offset of the base scan, to exercise --register
        ang = np.radians(perturb)
        Rz = np.array([[np.cos(ang), -np.sin(ang), 0], [np.sin(ang), np.cos(ang), 0], [0, 0, 1]])
        base = base @ Rz.T + np.array([perturb, -perturb / 2, perturb / 3])
    os.makedirs(out, exist_ok=True)
    paths = {k: os.path.join(out, f"{k}_{name}.ply") for k in ("base", "combined")}
    write_ply(paths["base"], base)
    write_ply(paths["combined"], combined)
    paths["gt"] = os.path.join(out, f"gt_{name}.json")
    with open(paths["gt"], "w") as f:
        json.dump({"scenario": name, "units": "mm", "polylines": gt}, f)
    return paths


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", default="synthetic")
    ap.add_argument("--scenarios", nargs="*", default=SCENARIOS)
    ap.add_argument("--spacing", type=float, default=1.0)
    ap.add_argument("--noise", type=float, default=0.05)
    ap.add_argument("--perturb", type=float, default=0.0, help="deg / mm offset of base scan")
    a = ap.parse_args()
    for s in a.scenarios:
        p = generate(s, a.out, a.spacing, a.noise, a.perturb)
        print(f"{s:15s} -> {p['base']}, {p['combined']}, {p['gt']}")
