"""UR10 dynamic truth fixture (#21).

The committed fixture (examples/ur10/data/truth) is checked against its
manifest, regenerated, and its effort checked independently of the RNEA that
produced it. The base least-squares fit must recover the truth on clean
analytic data, directly and through the UR10 identification pipeline.
"""

import contextlib
import hashlib
import io
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

UR10 = Path(__file__).resolve().parents[1] / "examples" / "ur10"
sys.path.insert(0, str(UR10.parents[1]))

from examples.ur10 import identification_truth as it  # noqa: E402


@pytest.fixture(scope="module")
def regenerated(tmp_path_factory):
    out = tmp_path_factory.mktemp("truth")
    it.generate(out)
    return out


def test_fixture_files_match_manifest():
    assert it.check_fixture() == []


def test_regeneration_reproduces_fixture(regenerated):
    for split in ("train", "validation"):
        new, old = it.read_split(split, regenerated), it.read_split(split)
        for key in old:
            # libm may differ in the last bits between platforms
            np.testing.assert_allclose(new[key], old[key], rtol=0, atol=1e-9)
    new = it.truth_parameters(regenerated)
    old = it.truth_parameters()
    np.testing.assert_allclose(new.truth, old.truth, rtol=1e-12, atol=1e-15)
    new = yaml.safe_load((regenerated / "protocol.yaml").read_text())["rank"]
    old = it.protocol()["rank"]
    for key in ("eliminated_indices", "base_indices", "base_names"):
        assert new[key] == old[key]
    np.testing.assert_allclose(
        list(new["base_truth"].values()), list(old["base_truth"].values()), atol=1e-9
    )


def test_truth_urdf_is_the_saved_truth_and_physical():
    model = it.truth_model()
    saved = it.truth_parameters()
    std = it.get_standard_parameters(model, it.IDENTIF_CONFIG)
    assert list(std) == list(saved.parameter)
    np.testing.assert_array_equal(list(std.values()), saved.truth)
    for j in range(1, model.njoints):
        assert it.pseudo_inertia_min_eig(model.inertias[j]) > 1e-4
    # the truth is not the URDF a fit would start from
    rel = np.abs(saved.truth - saved.nominal) / np.maximum(np.abs(saved.nominal), 1e-3)
    masses = saved.parameter.str.startswith("m_")
    assert np.all(rel[masses] > 0.01)


def test_effort_checked_independently():
    model = it.truth_model()
    entry = it.manifest()
    for split, info in entry["splits"].items():
        data = it.read_split(split)
        coefficients = info["coefficients"]
        checks = it.independent_checks(
            model, data, it.TRAJECTORIES[split], coefficients
        )
        for key in ("crba_nle", "pinocchio_regressor", "figaroh_regressor"):
            assert checks[key] < 1e-9, (split, key, checks[key])
        assert checks["aba_ddq"] < 1e-9
        assert checks["power_balance_w"] < 1e-6 * checks["power_scale_w"]
        # every tangent coordinate: analytic derivatives match the series
        assert max(checks["dq_central_difference"]) < 1e-7
        assert max(checks["ddq_central_difference"]) < 1e-7


def test_every_coordinate_excited_within_limits():
    entry = it.manifest()
    for split, info in entry["splits"].items():
        data = it.read_split(split)
        assert info["rank"] == 36
        assert info["base_condition_number"] < 200
        assert np.all(np.ptp(data["q"], axis=0) > 1.0)
        assert np.all(np.abs(data["dq"]).max(0) > 0.5)
        assert np.all(np.abs(data["ddq"]).max(0) > 1.0)
        assert np.all(np.sqrt(np.mean(data["tau"] ** 2, axis=0)) > 0.05)
        assert np.all(
            np.abs(data["dq"]).max(0)
            <= it.VELOCITY_FRACTION * np.array(it.VELOCITY_LIMIT) + 1e-12
        )
        assert np.all(
            np.abs(data["tau"]).max(0) <= it.TORQUE_FRACTION * np.array(it.TORQUE_LIMIT)
        )
        assert np.all(np.abs(data["q"] - it.CENTER) <= it.AMPLITUDE_RAD + 1e-12)
        assert np.allclose(np.diff(data["t"]), 1 / it.SAMPLE_RATE_HZ)


def test_splits_are_independent():
    entry = it.manifest()
    train, val = entry["splits"]["train"], entry["splits"]["validation"]
    assert train["seed"] != val["seed"]
    assert train["fundamental_hz"] != val["fundamental_hz"]
    assert not set(it.NOISE_SEEDS["train"]) & set(it.NOISE_SEEDS["validation"])
    q_train = {tuple(r) for r in np.round(it.read_split("train")["q"], 9)}
    q_val = {tuple(r) for r in np.round(it.read_split("validation")["q"], 9)}
    assert len(q_train & q_val) == 0
    protocol = it.protocol()
    assert protocol["rank"]["validation_rank"] == protocol["rank"]["base_parameters"]


def test_noise_is_seeded_and_sized():
    a = it.load_split("train", noise="high", noise_seed=101)
    b = it.load_split("train", noise="high", noise_seed=101)
    c = it.load_split("train", noise="high", noise_seed=102)
    np.testing.assert_array_equal(a["tau"], b["tau"])
    assert not np.allclose(a["tau"], c["tau"])
    np.testing.assert_array_equal(a["tau_true"], it.read_split("train")["tau"])
    sigma = it.protocol()["noise"]["effort_sigma_nm"]["high"]
    ratio = np.std(a["tau"] - a["tau_true"], axis=0) / [sigma[j] for j in it.JOINTS]
    assert np.all(np.abs(ratio - 1) < 0.1)
    with pytest.raises(ValueError):
        it.load_split("train", noise="low", noise_seed=201)  # a validation seed


def test_clean_base_fit_recovers_truth():
    r = it.ols_case("analytic", "none", None, None)
    assert r["base_err_max_nm"] < 1e-9
    assert r["val_rmse_nm"].max() < 1e-9


def test_differentiated_positions_are_aligned():
    d = it.load_split("train", derivatives="differentiated")
    full = it.read_split("train")
    n = len(full["t"])
    assert len(d["q"]) == len(d["tau"]) == n - 2
    np.testing.assert_array_equal(d["q"], full["q"][: n - 2])
    # forward-difference velocity sits half a step after its position row
    dt = 1 / it.SAMPLE_RATE_HZ
    midpoint = 0.5 * (full["dq"][: n - 2] + full["dq"][1 : n - 1])
    assert np.abs(d["dq"] - midpoint).max() < 1e-3 * np.abs(full["dq"]).max()
    late = np.abs(d["dq"] - full["dq"][: n - 2]).max()
    assert late > 0.4 * dt * np.abs(full["ddq"]).max()


def test_core_pipeline_recovers_truth():
    with contextlib.redirect_stdout(io.StringIO()):
        ident = it.identification()
        ident.solve(decimate=False, plotting=False)
        metrics = ident._compute_validation_metrics()
    protocol = it.protocol()
    names = protocol["rank"]["base_names"]
    assert list(ident.params_base) == names
    truth = np.array([protocol["rank"]["base_truth"][n] for n in names])
    # core rounds phi_b to 6 decimals (figaroh tools/qrdecomposition.py)
    assert np.abs(np.asarray(ident.phi_base) - truth).max() <= 5e-7 + 1e-12
    assert metrics["validation_source"] == "validation_data"
    assert metrics["n_val_samples"] == 800
    assert metrics["rmse_identified"] < 1e-4 < metrics["rmse_nominal"]


def test_legacy_csvs_untouched():
    # SHA-256 at the signal audit (docs/development/ur10-signal-audit-2026-10-02.md)
    expected = {
        "data/identification_q_simulation.csv": "cee459266b276c995529a2a979a72a5edc05c3d86fd4c67857888b4d063718b6",
        "data/identification_tau_simulation.csv": "95443e9f749afa51db1f89a230c630e1c4e30c3eccee88fc522913c04a860828",
        "data/validation/identification_q_simulation.csv": "95643c9cf273ac5d1fc6207aa85e2fede9e9e5264d466b337c815bb67d4168c7",
        "data/validation/identification_tau_simulation.csv": "730e02b0eeb125e03f8f503001160e6e562d763d6619a7f084984b2e338c6403",
    }
    for path, digest in expected.items():
        assert hashlib.sha256((UR10 / path).read_bytes()).hexdigest() == digest
    assert (
        it.manifest()["model"]["nominal_urdf_sha256"]
        == hashlib.sha256(it.URDF.read_bytes()).hexdigest()
    )
