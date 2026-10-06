"""Parameters of the seam finder, every length relative to the point spacing h unless it is
a physical (ISO / torch) quantity in mm.

`seam_finder.md` fixed several numbers without the benchmark's standards in view. The ones
changed here, and why (the README has the full list):

* `gap_max_mm` 3 -> 5. The corpus samples root gaps from ISO 9692-1 preparation ranges
  with the ISO 5817 quality levels B / C / D AND a below-D over-range tail; the largest
  drawn gap is 4.9 mm (corner joints). A seam at 4 mm gap is still a seam in the truth.
* `h` is ESTIMATED per part (median 4th-neighbour distance), never fixed at 1 mm: the
  corpus densities run 0.25-4 pts/mm^2, i.e. h = 0.5-2 mm.
* `mls_normal_gate` (new): MLS and PCA neighbourhoods keep only neighbours whose oriented
  normal agrees. Plates go down to 1.0 mm (edge joints are 1-2 mm thick), thinner than the
  plan's r_mls = 3h, so an ungated neighbourhood mixes the top and bottom face of one part
  - per-part labels cannot prevent that, both faces have the same label.
* torch model: the generator's own accessibility block (cone half-angle 30 deg, standoff
  15 mm, work angle <= 45 deg, dihedral 30-170 deg) instead of an invented nozzle.
* `coplanar_tau_mm` (new): ISO 9692-1 grooves open the top faces of a butt by up to
  b + 2 t tan(beta) ~ 25 mm (t <= 20 mm, beta <= 30 deg); the stored butt seam is the
  centreline between the two top faces, so coplanar boundary pairs are matched over that
  span (Stage 3b).
"""
from __future__ import annotations

from dataclasses import dataclass, field, asdict


@dataclass
class Params:
    # Stage 1
    h_mm: float | None = None          # None -> estimated per part
    outlier_k: int = 12
    outlier_std: float = 3.0
    remove_outliers: bool = False      # tier-1 clouds carry no gross outliers
    k_normals: int = 25
    r_mls_h: float = 3.0
    mls_normal_gate: float = 0.5       # cos 60 deg; None/0 -> ungated (the plan's MLS)
    support_h: float = 0.75            # a projection farther than this from any sample is off the patch (0.71h = grid hole)
    # Stage 2a
    prefilter: bool = True
    n_rays: int = 24
    cone_half_deg: float = 60.0
    tau_vis: float = 0.04              # >= 1 free ray of 24: the generator exterior rule ("some ray escapes"); plan 0.3
    ray_start_h: float = 0.05          # plan: start at p + eps n
    ray_hit_h: float = 0.5
    ray_free_mm: float = 40.0
    ray_max_steps: int = 60
    # Stage 3
    gap_max_mm: float = 5.0
    tau_d_extra_h: float = 2.0         # tau_d = gap_max + 2h
    seeding: str = "oneway"            # "dense" (KD mutual NN on all points) | "fps" (plan: N x M matrix) | "oneway" (ablation)
    n_fps: int = 1500
    coplanar_pairs: bool = True        # Stage 3b
    coplanar_tau_mm: float = 25.0
    coplanar_deg: float = 20.0         # theta_n below this: coplanar (butt / edge)
    facing_deg: float = 150.0          # theta_n above this: facing surfaces, dropped
    coplanar_tol_mm: float = 0.5       # generator's coplanar_tol_mm: one plane, not two parallel ones
    # Stage 4
    alpha: float = 0.9
    eps_h: float = 0.05
    tol_h: float = 0.01
    k_max: int = 50
    crease_free: object = "interior"   # True | "interior" (only toes not at a part edge) | False
    crease_fit_h: float = 6.0
    # Stage 5
    angle_check: bool = False
    angle_disagree_deg: float = 10.0
    dbscan_eps_h: float = 4.0
    dbscan_min: int = 5
    dbscan_eps_spacing: float = 3.0    # eps >= 3 x median seed spacing (seeds are sparser than h)
    normal_eps: float = 0.35           # split of a position cluster on (n_A, n_B), ~20 deg
    suppress_toes: bool = True         # ISO 17659 / generator rule: toes of a coplanar gap are not seams
    min_seam_mm: float = 10.0          # generator's min_seam_length_mm
    cross_runs: bool = True            # generator SCHEMA 2.6.2: runs across the joint are not seams
    cross_run_tol_deg: float = 45.0
    close_h: float = 3.0
    ds_mm: float = 2.0
    # Stage 2c
    torch_check: bool = True
    torch_half_deg: float = 30.0
    torch_standoff_mm: float = 15.0
    max_work_deg: float = 45.0
    travel_deg: tuple = (-15.0, 0.0, 15.0)
    work_step_deg: float = 15.0
    bore_check: bool = True
    bore_min_diameter_mm: float = 80.0  # generator torch_clearance.bore_min_diameter_mm
    output: str = "weldable"          # "weldable" spans (plan's output) | "all" seams
    dihedral_min_deg: float = 30.0
    dihedral_max_deg: float = 170.0
    extra: dict = field(default_factory=dict)

    def as_dict(self):
        return asdict(self)
