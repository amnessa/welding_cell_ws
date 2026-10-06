"""seamfind — matching-based seeding + sphere-tracing refinement (`../seam_finder.md`),
fitted to the weld_generator benchmark. Entry point: `pipeline.extract`."""
from .config import Params
from .pipeline import Result, extract

__all__ = ["Params", "Result", "extract"]
