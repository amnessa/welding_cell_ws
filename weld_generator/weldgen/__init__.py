"""weldgen — analytic weld-seam scene generator.

Tier 1 core: `trimesh` + NumPy only. No renderer, no ROS, no Isaac (D9).

The one rule (dataset_plan.md §1): **ground truth is constructed, never detected.**
Seam geometry is computed in closed form from the placement transforms (D1); nothing
in this package ever estimates a seam from a point cloud.

Frozen contracts this code implements:
  docs/SCHEMA.md      (schema_version 2.0.0)
  docs/PARAMETERS.md  (params_version 2.0.0)
  docs/scene.schema.json
"""

__version__ = "0.1.0"

SCHEMA_VERSION = "2.2.0"
GENERATOR_VERSION = __version__
