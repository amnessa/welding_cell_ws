"""Check the saddle closed form of weldgen.curves against the draft's Eq. ws_saddle."""
import sys
import numpy as np

sys.path.insert(0, "/workspaces/welding_cell_ws/weld_generator")
from weldgen.curves import saddle_from_cylinders  # noqa: E402

R, r = 60.0, 25.0
# untilted, centred branch, as d29._config_4 with tilt = off = 0
c = saddle_from_cylinders([0, 0, R + r + 60.0], [0, 0, -1.0], r, [0, 0, 0], [1.0, 0, 0], R)
phi = np.linspace(0, 2 * np.pi, 13, endpoint=False)
P = c.point(phi)
print("u_dir, v_dir:", c.u_dir, c.v_dir)
print("max |z - sqrt(R^2 - y^2)|      :", np.abs(P[:, 2] - np.sqrt(R**2 - P[:, 1]**2)).max())
print("max |z - sqrt(R^2 - r^2 cos^2 phi)|:", np.abs(P[:, 2] - np.sqrt(R**2 - r**2 * np.cos(phi)**2)).max())
print("max |z - sqrt(R^2 - r^2 sin^2 phi)|:", np.abs(P[:, 2] - np.sqrt(R**2 - r**2 * np.sin(phi)**2)).max())

# general case: tilted + offset branch; evaluate the draft's A, B, C and compare
tilt, off = np.radians(20.0), 0.3 * (R - r)
a = np.array([np.sin(tilt), 0.0, -np.cos(tilt)])
o = np.array([0.0, off, R + r + 60.0])
m, pm = np.array([1.0, 0, 0]), np.zeros(3)
c2 = saddle_from_cylinders(o, a, r, pm, m, R)
cc = o[None] + r * (np.cos(phi)[:, None] * c2.u_dir[None] + np.sin(phi)[:, None] * c2.v_dir[None])
d = cc - pm
A = 1 - (a @ m) ** 2
B = 2 * (d @ a - (d @ m) * (a @ m))
C = (d * d).sum(1) - (d @ m) ** 2 - R**2
s = (-B - np.sqrt(B * B - 4 * A * C)) / (2 * A)
Pd = cc + s[:, None] * a
print("draft eq vs code, max |P_draft - P_code|:", np.abs(Pd - c2.point(phi)).max())
rad = np.linalg.norm((Pd - pm) - ((Pd - pm) @ m)[:, None] * m, axis=1)
print("max | dist to main axis - R |:", np.abs(rad - R).max())
