"""TIAGo calibration recovers a known truth (#26).

Synthetic measurements at the real postures, from a drawn truth
(examples/tiago/calibration_truth.py). Recovery is checked in identifiable
coordinates: held-out tool-point prediction against the noise-free truth and,
when the fit has the truth's model class, the kept parameters against their
standard errors.
"""

import sys
from pathlib import Path

import numpy as np
import pytest

TIAGO = Path(__file__).resolve().parents[1] / "examples" / "tiago"
sys.path.insert(0, str(TIAGO.parents[1]))


@pytest.fixture(scope="module")
def truth(tmp_path_factory, monkeypatch_module):
    monkeypatch_module.chdir(TIAGO)
    from examples.tiago import calibration_truth

    return calibration_truth, tmp_path_factory.mktemp("truth")


@pytest.fixture(scope="module")
def monkeypatch_module():
    mp = pytest.MonkeyPatch()
    yield mp
    mp.undo()


def test_noise_free_recovery_is_exact(truth):
    ct, tmp = truth
    r = ct.run_case("joint_offset", "joint_offset", 0, 0.0, tmp)
    assert r["absorbed"] == ["offsetPZ_torso_lift_joint", "offsetRZ_arm_1_joint"]
    assert r["train_rms_mm"] < 1e-3
    assert r["heldout_max_mm"] < 1e-3


def test_offsets_recovered_within_standard_errors(truth):
    ct, tmp = truth
    z = []
    for seed in (0, 1):
        r = ct.run_case("joint_offset", "joint_offset", seed, 0.5, tmp)
        # residual = noise, less the fitted degrees of freedom (111 rows, 14)
        assert 0.40 < r["train_rms_mm"] < 0.55
        assert r["heldout_rmse_mm"] < 0.5
        z += list(r["z_scores"].values())
    z = np.array(z)
    assert len(z) == 10
    assert np.abs(z).max() < 4.0
    assert 0.4 < np.sqrt(np.mean(z**2)) < 1.6


def test_calibration_beats_registration_on_full_params_truth(truth):
    ct, tmp = truth
    new = {
        fit: ct.run_case("full_params", fit, 0, 0.5, tmp)["strata_rmse_mm"]["new"]
        for fit in ("registration only", "joint_offset", "full_params")
    }
    assert new["joint_offset"] < 0.7 * new["registration only"]
    assert new["full_params"] < 0.7 * new["registration only"]


def test_truth_and_measurements_are_seeded(truth):
    ct, _ = truth
    from examples.tiago import heldout_protocol as hp

    probe = hp.fit("joint_offset", frames_only=True)
    q = ct.postures(ct.TRAINING)
    a = ct.simulate(
        probe, "full_params", ct.make_truth("full_params", 3, probe.model), q, 0.5, 3
    )
    b = ct.simulate(
        probe, "full_params", ct.make_truth("full_params", 3, probe.model), q, 0.5, 3
    )
    c = ct.simulate(
        probe, "full_params", ct.make_truth("full_params", 4, probe.model), q, 0.5, 4
    )
    assert a.equals(b)
    assert not a.equals(c)


def test_estimation_method_reaches_core(truth):
    """``estimation`` is passed to core (figaroh-plus#113): map keeps all 57."""
    ct, tmp = truth
    r = ct.run_case(
        "full_params",
        "full_params",
        0,
        0.5,
        tmp,
        estimation={"method": "map"},
        label="m",
    )
    assert r["fit"] == "m"
    assert r["n_params"] == 6 + 48 + 3
    assert r["heldout_rmse_mm"] < 1.0
