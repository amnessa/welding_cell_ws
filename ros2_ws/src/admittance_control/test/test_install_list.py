"""Every library module is installed. Plain pytest.

CMakeLists.txt installs the Python modules from an explicit list (it does not call
ament_python_install_package). A module missing from it imports fine from the source
tree - so every other test passes - and fails only in the launched nodes, which load
the installed package: 2026-10-09, `curved_seams` was missing and mode A fell back to
the radius-PCA detector on the whole curved-parts bench.
"""

from __future__ import annotations

import pathlib
import re

PKG = pathlib.Path(__file__).resolve().parents[1]


def test_every_module_is_in_the_cmake_install_list():
    cmake = (PKG / "CMakeLists.txt").read_text()
    listed = set(re.findall(r"admittance_control/(\w+\.py)", cmake))
    modules = {p.name for p in (PKG / "admittance_control").glob("*.py")}
    assert modules - listed == set(), f"add to CMakeLists.txt install(FILES ...): {sorted(modules - listed)}"
