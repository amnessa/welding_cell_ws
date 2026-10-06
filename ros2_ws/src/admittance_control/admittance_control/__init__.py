"""admittance_control: the welding cell's shared Python modules.

The package also has BUILT parts that exist only in the install space: the generated
`srv` / `action` interfaces and the C++ extension `_transit_cpp`. The scripts put the
SOURCE tree first on sys.path (and a symlink install resolves them there), so this
package can be imported from the source copy, which has none of them. Extending the
package path with every other `admittance_control` directory on sys.path makes the
source modules win (they come first) while the built parts are still found.
"""
from pkgutil import extend_path

__path__ = extend_path(__path__, __name__)
