"""Check the step ratio of the seamfind walker on two planar half-planes.

A: z = 0, x >= 0 (normal +z into the wedge). B: half-plane through the y axis at
opening angle theta from A. Walk A-points at x = s0 towards B with seamfind.walk.walk,
record d_k, compare the mean ratio with 1 - alpha sin(theta) and 1 - alpha sin^2(theta),
and evaluate seamfind.walk.theta_r.
"""
import sys
import numpy as np

sys.path.insert(0, "/workspaces/welding_cell_ws/weld_generator/Self_ideas/test2_cagdas")
from seamfind.geom import PartSurface  # noqa: E402
from seamfind import walk as W  # noqa: E402

p = 0.5          # grid pitch, mm
alpha = 0.9
for theta_deg in (20.0, 30.0, 45.0, 60.0, 75.0, 90.0, 120.0):
    th = np.radians(theta_deg)
    r = np.arange(0.0, 80.0 + 1e-9, p)
    y = np.arange(-30.0, 30.0 + 1e-9, p)
    R, Y = np.meshgrid(r, y, indexing="ij")
    A = np.column_stack([R.ravel(), Y.ravel(), np.zeros(R.size)])
    B = np.column_stack([R.ravel() * np.cos(th), Y.ravel(), R.ravel() * np.sin(th)])
    nA = np.tile([0.0, 0.0, 1.0], (len(A), 1))
    nB = np.tile([np.sin(th), 0.0, -np.cos(th)], (len(B), 1))
    SA = PartSurface(A, p, orient=nA)
    SB = PartSurface(B, p, orient=nB)
    X0 = np.array([[30.0, 0.0, 0.0], [40.0, 5.0, 0.0]])
    x, n, d_last, steps, conv, H = W.walk(SA, SB, X0, alpha, 0.05 * p, 0.01 * p, 50)
    h = H[0]
    h = h[np.isfinite(h)]
    rat = h[1:] / h[:-1]
    rat = rat[h[:-1] > 0.05 * p * 5]
    thr = W.theta_r(H, d_last, alpha)
    print(f"theta={theta_deg:5.1f}  d_k[:5]={np.round(h[:5], 3)}  mean ratio={rat.mean():.4f}  "
          f"1-a sin={1 - alpha*np.sin(th):.4f}  1-a sin^2={1 - alpha*np.sin(th)**2:.4f}  "
          f"theta_r(code)={np.round(thr, 1)}  arcsin(sin^2)={np.degrees(np.arcsin(min(1, np.sin(th)**2))):.1f}")
