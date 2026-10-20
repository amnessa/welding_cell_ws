"""Acute-side fillet verdict of the D4 rule versus the dihedral (effective floor check)."""
import sys
import numpy as np

sys.path.insert(0, "/workspaces/welding_cell_ws/weld_generator")
from weldgen.joints import JointSpec  # noqa: E402
from weldgen.layouts import build  # noqa: E402
from weldgen.accessibility import enumerate_candidates  # noqa: E402

for alpha in (56.0, 58.0, 59.0, 60.0, 61.0, 62.0, 64.0, 66.0, 70.0):
    spec = JointSpec(L_A=200, W_A=150, t_A=8, L_B=200, H_B=120, t_B=8, root_gap_mm=0.0,
                     linear_misalignment_mm=0.0, angular_misalignment_deg=0.0,
                     included_angle_deg=alpha)
    parts = build(spec, "T", np.eye(4))
    acc = {"contact_tol_mm": 0.5}
    cands = enumerate_candidates(parts, acc, joint_type="T")
    fil = [(round(c.dihedral_deg, 1), c.weldable, c.reject_reason) for c in cands
           if c.p0 is not None and c.length_mm > 50 and c.seam_class == "fillet"]
    print(alpha, fil)
