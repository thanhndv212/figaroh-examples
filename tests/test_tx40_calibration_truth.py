"""TX40 calibration recovers a known truth (simulated data only).

Synthetic tool poses at the frozen D-optimal postures, from a drawn truth
(examples/staubli_tx40/calibration_truth.py). Recovery is checked on the
held-out tool position against the noise-free truth and, at joint_offset, on
the kept offsets against their standard errors.
"""

import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

TX40 = Path(__file__).resolve().parents[1] / "examples" / "staubli_tx40"
sys.path.insert(0, str(TX40.parents[1]))


@pytest.fixture(scope="module")
def monkeypatch_module():
    mp = pytest.MonkeyPatch()
    yield mp
    mp.undo()


@pytest.fixture(scope="module")
def truth(tmp_path_factory, monkeypatch_module):
    monkeypatch_module.chdir(TX40)
    from examples.staubli_tx40 import calibration_truth

    return calibration_truth, tmp_path_factory.mktemp("tx40_truth")


@pytest.mark.parametrize("kind", ["joint_offset", "full_params"])
def test_noise_free_recovery_is_exact(truth, kind):
    ct, tmp = truth
    r = ct.run_case(kind, kind, 0, 0.0, tmp)
    assert r["train_rms_mm"] < 1e-6
    assert r["heldout_max_mm"] < 1e-6


def test_full_params_keeps_all_30_identifiable_parameters(truth):
    """4N + 6 = 30 for a 6R arm measured in full pose. Core's default
    (LAPACK) selection keeps 29: the config's ``structural_selection:
    ordered`` is what makes the full_params truth recoverable."""
    ct, tmp = truth
    r = ct.run_case("full_params", "full_params", 1, 0.0, tmp)
    assert r["n_params"] == 30
    assert sorted(r["absorbed"]) == ["d_phiz_joint_6", "d_pz_joint_6"]


def test_offsets_recovered_within_standard_errors(truth):
    ct, tmp = truth
    z = []
    for seed in (0, 1):
        r = ct.run_case("joint_offset", "joint_offset", seed, 0.1, tmp)
        # position residual = noise less the fitted degrees of freedom
        assert 0.085 < r["train_rms_mm"] < 0.105
        assert r["heldout_rmse_mm"] < 0.05
        z += list(r["z_scores"].values())
    z = np.array(z)
    assert len(z) == 8  # joints 2-5; 1 and 6 go to the base and tool frames
    assert np.abs(z).max() < 4.0
    assert 0.4 < np.sqrt(np.mean(z**2)) < 1.6


def test_calibration_beats_registration(truth):
    ct, tmp = truth
    err = {
        f: ct.run_case("full_params", f, 0, 0.1, tmp)["heldout_rmse_mm"]
        for f in ct.FITS
    }
    assert err["full_params"] < 0.1 * err["registration only"]
    assert err["full_params"] < 0.1 * err["joint_offset"]


def test_frozen_design_comes_from_the_optimal_config_candidates(truth):
    ct, _ = truth
    design = ct.design("doptimal").to_numpy()
    pool = ct.candidate_pool().to_numpy()
    meta = pd.read_json(ct.DESIGN_META, typ="series")
    assert len(design) == meta["selected"] == 66
    # every selected posture is one of the 1000 seeded candidates
    d = np.abs(design[:, None, :] - pool[None, :, :]).max(axis=2)
    assert np.all(d.min(axis=1) < 1e-12)
    random = ct.design("random")
    assert len(random) == len(design)
    assert not np.allclose(random.to_numpy(), design)


def test_truth_and_measurements_are_seeded(truth):
    ct, _ = truth
    from examples.calibration_harness import simulation as sim

    st = ct.study("doptimal")
    a = sim._simulate(st, ct.draw_truth("full_params", np.random.default_rng(3), None),
                      "full_params", st.train_postures, 0.1, 3)[0]
    b = sim._simulate(st, ct.draw_truth("full_params", np.random.default_rng(3), None),
                      "full_params", st.train_postures, 0.1, 3)[0]
    c = sim._simulate(st, ct.draw_truth("full_params", np.random.default_rng(4), None),
                      "full_params", st.train_postures, 0.1, 4)[0]
    np.testing.assert_array_equal(a, b)
    assert not np.allclose(a, c)
