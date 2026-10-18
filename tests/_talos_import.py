"""Import a TALOS table-contact example module by bare name.

``heldout_protocol`` and ``calibration_truth`` exist for several robots, and
other test modules put their example directory first on ``sys.path`` at
collection time. Import the table-contact one explicitly so the result does
not depend on which test files were collected.
"""

import importlib
import sys
from pathlib import Path

EXAMPLE = Path(__file__).resolve().parents[1] / "examples" / "talos_table_contact"


_SIBLINGS = ("heldout_protocol", "calibration_truth")


def talos_module(name: str):
    for n in _SIBLINGS:
        mod = sys.modules.get(n)
        if mod is not None and Path(mod.__file__).parent != EXAMPLE:
            sys.modules.pop(n)
    if str(EXAMPLE) in sys.path:
        sys.path.remove(str(EXAMPLE))
    sys.path.insert(0, str(EXAMPLE))
    return importlib.import_module(name)
