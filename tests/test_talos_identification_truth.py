"""TALOS torso-arm identification recovers a known truth (simulated data only).

Synthetic joint torques from a drawn truth on a seeded, self-collision
checked excitation (examples/talos/identification_truth.py), identified by
core's BaseIdentification and judged on held-out torques against the
noise-free truth and on the base parameters against their standard errors.
"""

import io
import contextlib
from pathlib import Path

import numpy as np
import pytest

TALOS = Path(__file__).resolve().parents[1] / "examples" / "talos"


@pytest.fixture(scope="module")
def it():
    mp = pytest.MonkeyPatch()
    mp.chdir(TALOS)
    from examples.talos import identification_truth

    yield identification_truth
    mp.undo()


def test_simulated_torques_match_cores_regressor(it):
    """The truth's torques are W @ phi_std in core's own parameter order."""
    truth = it.draw_truth(0)
    d = it.simulate(truth, "train", 0.0, 0)
    ident = it.SimulatedIdentification(d)
    with contextlib.redirect_stdout(io.StringIO()):
        ident.initialize()
    phi = it.truth_standard_vector(truth, list(ident.standard_parameter))
    tau = ident.dynamic_regressor @ phi
    np.testing.assert_allclose(tau, d["tau_true"].T.ravel(), rtol=0, atol=1e-9)


def test_noise_free_recovery_is_exact(it):
    r = it.run_case(0, 0.0)
    assert r["n_base"] == 82
    assert r["train_nrms_pct"] < 1e-8
    assert r["heldout_nrmse_max_pct"] < 1e-8


def test_base_parameters_within_standard_errors(it):
    z, err = [], []
    for seed in (0, 1):
        r = it.run_case(seed, 0.02)
        assert 1.8 < r["train_nrms_pct"] < 2.2
        err.append(r["heldout_nrmse_pct"])
        z.append(r["z"])
    z = np.concatenate(z)
    assert len(z) == 2 * 82
    assert abs(z.mean()) < 0.3
    assert 0.7 < np.sqrt(np.mean(z**2)) < 1.3
    assert np.abs(z).max() < 4.5
    assert max(err) < 1.5  # held-out torque, % of RMS, well under the 2 % noise


def test_trajectories_are_feasible_and_distinct(it):
    model = it.chain_model()
    train, held = it.trajectory("train"), it.trajectory("heldout")
    assert train["seed"] != held["seed"]
    for tr in (train, held):
        assert it.infeasible(tr["q"], tr["dq"], tr["ddq"]) == ""
        assert np.all(np.abs(tr["dq"]) <= model.velocityLimit)
    assert not np.allclose(train["q"], held["q"])


def test_truth_is_seeded_and_physically_consistent(it):
    a, b, c = it.draw_truth(3), it.draw_truth(3), it.draw_truth(4)
    np.testing.assert_array_equal(a["fv"], b["fv"])
    assert not np.allclose(a["fv"], c["fv"])
    for inertia in a["inertias"]:
        assert inertia.mass > 0
        # rotational inertia positive definite with the triangle inequalities
        ev = np.linalg.eigvalsh(inertia.inertia)
        assert ev.min() > 0
        assert ev[0] + ev[1] >= ev[2] - 1e-12
