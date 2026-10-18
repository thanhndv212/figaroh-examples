"""TALOS fits: how much does the identifiability draw move them?

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
(c) the same holds for the single chains (figaroh-examples#98): the
    held-out transfer height error of the structural fit is identical over
    ``random_seed`` (checked here for seeds 0 and 1). Only the structural
    path draws random configurations; every other protocol model skips the
    draw (fast test in test_talos_table_contact_heldout.py), so its spread is
    zero by construction. Offline, seeds 0 to 4 on both chains for the
    table-only baseline, structural and map gave a spread of exactly 0.
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


# --------------------------------------------------------------------------
# Single chains: the structural fit is the only model that draws
# --------------------------------------------------------------------------
#: draws move nothing: only floating-point noise is tolerated
SINGLE_CHAIN_SPREAD_MM = 1e-6


@pytest.mark.parametrize("side", ["left", "right"])
def test_single_chain_structural_heldout_has_no_draw_spread(robot, side):
    from _talos_import import talos_module

    hp = talos_module("heldout_protocol")
    plan = hp.harness.load_plan(hp.CHAINS[side][1])
    train = plan.folds[0].training[0].path
    val = hp._load_real_touches(plan.folds[0].evaluated[0].path)
    z, n_params = [], set()
    for seed in (0, 1):
        f = hp.fit_model(robot, side, "structural", train, random_seed=seed)
        z.append(hp.gap_errors(f.calib, f.x, val)["transfer_z_mm"])
        n_params.add(f.n_params)
    assert len(n_params) == 1, n_params
    assert max(z) - min(z) <= SINGLE_CHAIN_SPREAD_MM, z
