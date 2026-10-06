"""Stage 1 — per-part preprocessing: spacing, downsample, normals, KD-tree, MLS projection.

Everything is computed PER PART, never on the merged cloud (plan, Stage 1). One addition
the corpus forces: neighbourhoods are gated by oriented-normal agreement, because a 1 mm
plate puts its own top and bottom face inside one r_mls = 3h ball (config.py).
"""
from __future__ import annotations

import numpy as np
from scipy.spatial import cKDTree


def unit(v, eps=1e-12):
    return v / np.maximum(np.linalg.norm(v, axis=-1, keepdims=True), eps)


def estimate_spacing(P, n_sample=5000, seed=0):
    rng = np.random.default_rng(seed)
    S = P[rng.choice(len(P), min(n_sample, len(P)), replace=False)]
    d, _ = cKDTree(P).query(S, k=5)
    return float(np.median(d[:, 4]))


def voxel_downsample(P, v, *cols):
    keys = np.floor(P / v).astype(np.int64)
    _, inv, cnt = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    inv = inv.reshape(-1)
    mean = lambda X: np.column_stack([np.bincount(inv, X[:, j]) for j in range(X.shape[1])]) / cnt[:, None]
    return (mean(P),) + tuple(mean(np.asarray(c, float)) for c in cols)


def remove_outliers(P, k=12, n_std=3.0):
    d, _ = cKDTree(P).query(P, k=k + 1)
    m = d[:, 1:].mean(1)
    return m < m.mean() + n_std * m.std()


class PartSurface:
    """One labelled part: points, oriented PCA normals, KD-tree, MLS projection Pi(x)."""

    def __init__(self, P, h, k=25, orient=None, cam_pos=None, r_mls_h=3.0, gate=0.5,
                 support_h=0.75, k_select=10):
        self.P = np.asarray(P, float)
        self.h = float(h)
        self.tree = cKDTree(self.P)
        self.r_mls = r_mls_h * self.h
        self.gate = gate
        self.support = support_h * self.h
        k = min(k, len(self.P))
        _, idx = self.tree.query(self.P, k=k)
        nb = self.P[idx] - self.P[idx].mean(1, keepdims=True)
        w, V = np.linalg.eigh(np.einsum("nki,nkj->nij", nb, nb) / k)
        n = V[:, :, 0]
        curv0 = w[:, 0] / np.maximum(w.sum(1), 1e-12)
        if gate:
            # edge-aware: adopt the normal of the flattest neighbourhood among the k_sel
            # nearest. A crease point's own neighbourhood straddles two faces (the tier-1
            # exterior keeps a ~0.4 mm strip of a fillet's buried face at small gaps), so
            # its own PCA normal is a blend that no gate against itself can repair.
            k_sel = min(k_select, k)
            best = idx[np.arange(len(idx)), np.argmin(curv0[idx[:, :k_sel]], axis=1)]
            n = n[best]
        # orientation (plan step 3): synthetic -> from the surface ("mesh") normal sign,
        # real scans -> towards the camera that observed the point
        if orient is not None:
            s = np.sign((n * orient).sum(1))
        elif cam_pos is not None:
            s = np.sign(((np.asarray(cam_pos) - self.P) * n).sum(1))
        else:
            s = np.ones(len(n))
        s[s == 0] = 1
        self.N = n * s[:, None]
        if gate:
            # re-fit with the normal gate: neighbours on the other face of a thin plate
            # (opposite oriented normal) leave the fit - one pass suffices on planes
            ok = (self.N[idx] * self.N[:, None, :]).sum(-1) > gate
            wts = ok.astype(float)[..., None]
            cnt = np.maximum(wts.sum(1), 1)
            c = (self.P[idx] * wts).sum(1) / cnt
            d = (self.P[idx] - c[:, None]) * wts
            _, V = np.linalg.eigh(np.einsum("nki,nkj->nij", d, d))
            n2 = V[:, :, 0]
            s2 = np.sign((n2 * self.N).sum(1)); s2[s2 == 0] = 1
            self.N = n2 * s2[:, None]
        self.curv = w[:, 0] / np.maximum(w.sum(1), 1e-12)

    def nearest(self, X):
        d, i = self.tree.query(X)
        return d, i

    def project(self, X, ref_normal=None):
        """MLS projection: Gaussian-weighted plane through the samples within r_mls of the
        nearest sample, gated by normal agreement; x is projected onto it.
        Returns (foot, normal, on_support)."""
        X = np.asarray(X, float).reshape(-1, 3)
        if not len(X):
            return X.copy(), X.copy(), np.zeros(0, bool)
        _, i0 = self.tree.query(X)
        n0 = self.N[i0] if ref_normal is None else ref_normal
        nbrs = self.tree.query_ball_point(self.P[i0], self.r_mls)
        L = max(len(b) for b in nbrs)
        idx = np.full((len(X), L), -1)
        for r, b in enumerate(nbrs):
            idx[r, :len(b)] = b
        valid = idx >= 0
        ii = np.where(valid, idx, 0)
        Q = self.P[ii]
        if self.gate:
            valid &= (self.N[ii] * n0[:, None, :]).sum(-1) > self.gate
        sig = self.r_mls / 2.0
        w = np.exp(-((Q - self.P[i0][:, None, :]) ** 2).sum(-1) / (2 * sig ** 2)) * valid
        ws = np.maximum(w.sum(1), 1e-12)
        c = (Q * w[..., None]).sum(1) / ws[:, None]
        D = (Q - c[:, None, :]) * np.sqrt(w)[..., None]
        _, V = np.linalg.eigh(np.einsum("nki,nkj->nij", D, D))
        n = V[:, :, 0]
        s = np.sign((n * n0).sum(1)); s[s == 0] = 1
        n = n * s[:, None]
        few = valid.sum(1) < 3
        n[few] = n0[few]
        c[few] = self.P[i0][few]
        foot = X - ((X - c) * n).sum(1, keepdims=True) * n
        on = self.tree.query(foot)[0] <= self.support
        return foot, n, on

    def closest(self, X):
        """Closest point of the finite patch: the MLS foot where it lies on the support,
        else the nearest sample (a finite patch's closest point is on its boundary)."""
        foot, n, on = self.project(X)
        d_s, i = self.tree.query(X)
        q = np.where(on[:, None], foot, self.P[i])
        nq = np.where(on[:, None], n, self.N[i])
        return q, nq, np.linalg.norm(X - q, axis=1)
