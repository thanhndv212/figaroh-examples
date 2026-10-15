"""TALOS two-chain fit: how much does the identifiability draw move it?

The identifiable parameter set is chosen from random configurations
(figaroh-plus#99). Core now seeds that draw from ``calib_config["random_seed"]``
(default 0), so:

(a) global RNG state (``pin.seed``, ``np.random.seed``) must not change the
    result at all;
(b) neither does the draw itself (``random_seed``): the configs select the
    first independent columns in joint order
    (``estimation.structural_selection: ordered``, figaroh-plus#204), so the
    shared torso axes and the held-out height error are the same for every
    seed. With LAPACK pivoting the draw swapped the shared axes and moved
    the held-out z by ~1.5 mm. Audit:
    docs/development/talos-contact-calibration-audit-2026-10-04.md.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

PROJECT_ROOT = Path(__file__).parent.parent
EXAMPLE_DIR = PROJECT_ROOT / "examples" / "talos_table_contact"
for p in (PROJECT_ROOT, EXAMPLE_DIR):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

pin = pytest.importorskip("pinocchio")

from generate_synthetic_data import _load_robot  # noqa: E402
from run_calibration_real_data import two_chain_metrics  # noqa: E402

pytestmark = pytest.mark.slow  # ~20 s per fit

MAX_SPREAD_MM = 0.1


@pytest.fixture(scope="module")
def robot():
    return _load_robot()


def test_global_rng_state_does_not_change_the_fit(robot):
    runs = []
    for s in (0, 42):
        pin.seed(s)
        np.random.seed(s)
        runs.append(two_chain_metrics(robot, random_seed=0))
    first = runs[0]
    for r in runs[1:]:
        assert r["shared_dx_names"] == first["shared_dx_names"]
        assert r["n_dx"] == first["n_dx"]
        for k in ("left_z_rmse_mm", "right_z_rmse_mm"):
            assert r[k] == pytest.approx(first[k], abs=1e-9)


def test_random_seed_spread_is_small(robot):
    runs = {s: two_chain_metrics(robot, s) for s in range(5)}
    for s, r in runs.items():
        print(
            s,
            r["n_dx"],
            r["left_z_rmse_mm"],
            r["right_z_rmse_mm"],
            r["shared_dx_names"],
        )
    for k in ("left_z_rmse_mm", "right_z_rmse_mm"):
        vals = [r[k] for r in runs.values()]
        assert max(vals) - min(vals) <= MAX_SPREAD_MM, k
    assert len({tuple(r["shared_dx_names"]) for r in runs.values()}) == 1
