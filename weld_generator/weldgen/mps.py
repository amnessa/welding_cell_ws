"""Task 2's label: the Most Probable Seam, as a VERSIONED RULE — never stored truth.

D25 (plan_patch_v2.md): the MPS label is a convention, the fourth instance of the
pattern D8 (tacks), D14 (noise realisation) and D19 (seam-under-gap) already follow —
ship a versioned rule function over the exact geometry, so anyone can re-run it with
different parameters, and no convention is baked into the release as if it were truth.

    mps_rule-0.1:  argmax over WELDABLE seams of visible arclength
                   ties broken by (larger dihedral fold, then lower seam id)
                   null if no seam has visible arclength > min_len_mm

Three choices worth their words:

* **Weldable, not primary.** D25 says "weldable seams", and the task ("return one seam
  — its class and location") is honest about it: an off-class seam is a real weld, and
  from one view it can genuinely be the most probable one. `matches_joint_type` is
  taxonomy, not reachability, and the rule does not consult it.
* **Visible arclength is `visible_fraction × length_mm`** — both stored per seam since
  Phase 3, which is what makes this a PURE function of `scene.json`: it applies
  retroactively to every corpus on disk with no regeneration and no array reads.
  Closed seams need no wrap convention here (total arclength × fraction is
  well-defined either way); the wrap problem D25 flagged bites the tack rule (Phase
  7), not this argmax.
* **The fold is deviation from flat**, `|180° − dihedral|`: between two equally
  visible seams the rule prefers the sharper crease (a fillet over a butt
  centreline), because that is the one a coarse-positioned torch is at. Curved seams
  store a varying dihedral summarised as `{min, max, mean}`; the rule uses the mean.

The block is emitted into `scene.json` only under `emit_mps: true` (default off, and
NOT a geometry key — it consumes no random draw), so every pre-6c corpus reproduces
bit-identically; consumers of existing corpora call `mps_rule(scene)` on the fly
instead and get the identical block.
"""

from __future__ import annotations

from typing import Any, Mapping

RULE_VERSION = "mps_rule-0.1"

#: matches accessibility's `min_seam_length_mm` default — below it, no answer beats
#: a null answer.
MIN_LEN_MM = 10.0


def _fold_deg(dihedral: Any) -> float:
    """Deviation from flat. Curved seams summarise a varying dihedral; use the mean."""
    d = float(dihedral["mean"]) if isinstance(dihedral, Mapping) else float(dihedral)
    return abs(180.0 - d)


def mps_rule(scene: Mapping[str, Any], min_len_mm: float = MIN_LEN_MM
             ) -> dict[str, Any]:
    """The `mps` block for one scene — `{rule_version, params, seam_id, class}`.

    `seam_id`/`class` are null when no weldable seam clears `min_len_mm` of visible
    arclength, which is a legitimate single-view answer, not a failure.
    """
    best_key = None
    best_seam = None
    for s in scene["seams"]:
        if not s.get("weldable"):
            continue
        vis_len = float(s["visible_fraction"]) * float(s["length_mm"])
        if vis_len <= float(min_len_mm):
            continue
        # max() over this tuple implements the D25 order: visible arclength, then
        # larger fold, then LOWER id (hence the negation)
        key = (vis_len, _fold_deg(s["dihedral_deg"]), -int(s["id"]))
        if best_key is None or key > best_key:
            best_key, best_seam = key, s
    return {
        "rule_version": RULE_VERSION,
        "params": {"min_len_mm": float(min_len_mm)},
        "seam_id": None if best_seam is None else int(best_seam["id"]),
        "class": None if best_seam is None else str(best_seam["seam_class"]),
    }
