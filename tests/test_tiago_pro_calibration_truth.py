"""TIAGo Pro calibration recovers a known truth (#97).

Synthetic measurements at the real training postures from a drawn truth
(examples/tiago_pro/calibration_truth.py), measured as poses with noise
estimated from the Qualisys repeat postures. The seed-0 grid is pinned
against ``golden/tiago_pro_mocap.json``.
"""

import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
PRO = ROOT / "examples" / "tiago_pro"
GOLDEN = json.loads((ROOT / "tests/golden/tiago_pro_mocap.json").read_text())
sys.path.insert(0, str(ROOT))


@pytest.fixture(scope="module")
def ct(tmp_path_factory):
    mp = pytest.MonkeyPatch()
    mp.chdir(PRO)
    from examples.tiago_pro import calibration_truth

    yield calibration_truth, tmp_path_factory.mktemp("truth")
    mp.undo()


def test_truth_and_measurements_are_seeded(ct):
    ct, _ = ct
    from examples.tiago_pro import heldout_protocol as hp

    probe = hp.fit("full_params", frames_only=True)
    q = ct.postures(ct.TRAINING)

    def sim(seed):
        truth = ct.draw_truth(
            "full_params", __import__("numpy").random.default_rng(seed), probe.model
        )
        return ct.tool_pose(probe, "full_params", truth, q)

    assert (sim(3) == sim(3)).all()
    assert not (sim(3) == sim(4)).all()


@pytest.mark.slow
def test_noise_free_recovery_is_exact(ct):
    ct, tmp = ct
    r = ct.run_case("full_params", "full_params", 0, 0.0, tmp)
    assert r["train_rms_mm"] < 1e-3
    assert r["heldout_max_mm"] < 1e-2


def test_seed0_grid_matches_golden(ct):
    ct, _ = ct
    df = ct.run(seeds=(0,))
    rows = df.to_dict("records")
    gold = GOLDEN["simulation_seed0"]
    assert [(r["fit"], r["noise_mm"]) for r in rows] == [
        (g["fit"], g["noise_mm"]) for g in gold
    ]
    for r, g in zip(rows, gold):
        if r["fit"] == "full_params" and r["n_params"] != g["n_params"]:
            continue  # structural ties break by platform (figaroh-plus#113)
        assert r["n_params"] == g["n_params"]
        for k in ("train_rms_mm", "heldout_rmse_mm", "heldout_max_mm"):
            assert r[k] == pytest.approx(g[k], rel=1e-3), (r["fit"], r["noise_mm"], k)


@pytest.mark.slow
def test_calibration_beats_registration_under_the_estimated_noise(ct):
    ct, tmp = ct
    sigma = GOLDEN["noise"]["sigma_mm"]
    reg = ct.run_case("full_params", "registration only", 0, sigma, tmp)
    full = ct.run_case("full_params", "full_params", 0, sigma, tmp)
    assert full["heldout_rmse_mm"] < 0.3 * reg["heldout_rmse_mm"]
    # held-out error sits near the noise: the fit does not overfit it
    assert full["heldout_rmse_mm"] < 2 * sigma


def test_estimation_method_reaches_core(ct):
    """``estimation`` is passed to core (figaroh-plus#113): map keeps all."""
    ct, tmp = ct
    r = ct.run_case(
        "full_params",
        "full_params",
        0,
        1.35,
        tmp,
        estimation={"method": "map"},
        label="m",
    )
    assert r["fit"] == "m"
    assert r["n_params"] == 6 + 6 + 48
    assert r["heldout_rmse_mm"] < 5.0
