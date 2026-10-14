"""The shared simulation harness reproduces the pre-harness TIAGo numbers.

``golden/calibration_harness_tiago.json`` holds ``run_case`` results of
``examples/tiago/calibration_truth.py`` before it moved onto
``examples/calibration_harness`` (joint_offset truth, seeds 0 and 1, noise
0.5 mm, three fits). joint_offset rows must match to 1e-4 relative:
solver round-off differs by ~1e-6 across platforms, while a changed RNG
order moves them by O(1). full_params rows keep the established 0.2 mm
tolerance and are compared only when the platform keeps the same number of
parameters, because the structural selection breaks pivot ties by platform
(figaroh-plus#113).
"""

import json
import sys
import tempfile
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
GOLDEN = json.loads((ROOT / "tests/golden/calibration_harness_tiago.json").read_text())
sys.path.insert(0, str(ROOT))

SCALARS = ("train_rms_mm", "heldout_rmse_mm", "heldout_max_mm")


@pytest.fixture(scope="module")
def cases():
    mp = pytest.MonkeyPatch()
    mp.chdir(ROOT / "examples" / "tiago")  # the config uses relative paths
    yield from _cases()
    mp.undo()


def _cases():
    from examples.tiago import calibration_truth as ct

    out = {}
    with tempfile.TemporaryDirectory() as tmp:
        for g in GOLDEN:
            out[(g["fit"], g["seed"])] = ct.run_case(
                g["truth"], g["fit"], g["seed"], g["noise_mm"], tmp
            )
    yield out


@pytest.mark.parametrize("g", GOLDEN, ids=lambda g: f"{g['fit']}-{g['seed']}")
def test_matches_golden(cases, g):
    got = cases[(g["fit"], g["seed"])]
    assert set(got) == set(g)
    if g["fit"] == "full_params":
        # full_params keeps 30 or 31 parameters depending on the platform
        # (QR ties); the values are only comparable when the set matches.
        if got["n_params"] != g["n_params"]:
            pytest.skip(
                f"platform keeps {got['n_params']} full_params, golden has {g['n_params']}"
            )
        for k in SCALARS:
            assert got[k] == pytest.approx(g[k], abs=0.2)
        for k, v in g["strata_rmse_mm"].items():
            assert got["strata_rmse_mm"][k] == pytest.approx(v, abs=0.2)
        return
    for k in (
        "truth",
        "fit",
        "seed",
        "noise_mm",
        "n_params",
        "solver_status",
        "absorbed",
    ):
        assert got[k] == g[k], k
    for k in SCALARS:
        assert got[k] == pytest.approx(g[k], rel=1e-4), k
    for k, v in g["strata_rmse_mm"].items():
        assert got["strata_rmse_mm"][k] == pytest.approx(v, rel=1e-4), k
    assert set(got.get("z_scores", {})) == set(g.get("z_scores", {}))
    for k, v in g.get("z_scores", {}).items():
        assert got["z_scores"][k] == pytest.approx(v, rel=1e-4, abs=1e-9), k
