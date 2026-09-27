"""SO-101 identification: simulated log -> identify -> deployable model.

The simulator (examples/so101/generate_simulated_data.py) uses an arm that
is deliberately heavier than the CAD model, so these tests check that the
identified gravity torque tracks the *truth*, not merely the prior.
"""

from __future__ import annotations

import contextlib
import io
import json
from pathlib import Path

import numpy as np
import pinocchio as pin
import pytest
import yaml

from examples.so101 import identification as ident
from examples.so101.generate_simulated_data import simulate
from examples.so101.utils.so101_tools import (
    ARM_JOINTS,
    IDENTIFIED_FORMAT,
    SO101Identification,
    identified_dynamics_dict,
    load_so101_robot,
    read_log_columns,
)

SO101_DIR = Path(__file__).resolve().parent.parent / "examples" / "so101"
URDF = str(SO101_DIR / "urdf" / "so101_new_calib.urdf")
CONFIG = str(SO101_DIR / "config" / "so101_unified_config.yaml")


@pytest.fixture(scope="module")
def datasets(tmp_path_factory):
    root = tmp_path_factory.mktemp("so101")
    train = simulate(
        URDF,
        root / "train",
        seed=0,
        duration=40.0,
        rate=50.0,
        nm_per_ma=0.001,
        noise_ma=15.0,
    )
    val = simulate(
        URDF,
        root / "val",
        seed=7,
        duration=40.0,
        rate=50.0,
        nm_per_ma=0.001,
        noise_ma=15.0,
    )
    return train, val


@pytest.fixture(scope="module")
def solved(datasets):
    train, val = datasets
    args = ident.parse_args(
        [
            "--config",
            CONFIG,
            "--urdf",
            URDF,
            "--data-dir",
            str(train),
            "--validation-dir",
            str(val),
            "--no-verify",
            "--no-html-report",
            "--no-archive",
        ]
    )
    with contextlib.redirect_stdout(io.StringIO()):
        return ident.run_identification(args)


def test_gripper_is_locked_out_of_the_model():
    robot = load_so101_robot(URDF, {"gripper": 0.3})
    assert list(robot.model.names[1:]) == ARM_JOINTS
    assert robot.model.nv == 5


def test_cad_prior_is_read_from_each_joints_own_body(solved):
    # Guards the figaroh 0.4.8 off-by-one in get_standard_parameters.
    for jname in ARM_JOINTS:
        body = solved.model.inertias[solved.model.getJointId(jname)]
        assert solved.standard_parameter[f"m_{jname}"] == pytest.approx(body.mass)


def test_fit_is_well_conditioned_and_validates(solved):
    result = solved.result
    assert result["condition number"] < 1000
    assert solved.correlation > 0.99
    val = result["validation_metrics"]
    assert val["validation_source"] == "validation_data"
    assert val["correlation"] > 0.99
    assert val["improvement_pct"] > 50
    assert solved.verify().passed


def test_gravity_matches_the_simulated_arm_not_the_cad_model(solved, datasets):
    truth = yaml.safe_load((datasets[0] / "ground_truth.yaml").read_text())
    doc = identified_dynamics_dict(solved, urdf=URDF)

    def model_with(bodies):
        m = solved.model.copy()
        for j, b in bodies.items():
            jid = m.getJointId(j)
            m.inertias[jid] = pin.Inertia(
                b["m"],
                np.array([b["mx"], b["my"], b["mz"]]) / b["m"],
                m.inertias[jid].inertia,
            )
        return m

    true_m, ident_m, cad_m = (
        model_with(truth["bodies"]),
        model_with(doc["bodies"]),
        solved.model,
    )
    rng = np.random.default_rng(3)
    lo, hi = cad_m.lowerPositionLimit, cad_m.upperPositionLimit
    qs = rng.uniform(0.5 * lo, 0.5 * hi, size=(50, 5))

    def rms(m):
        d = m.createData()
        err = [
            pin.computeGeneralizedGravity(m, d, q)
            - pin.computeGeneralizedGravity(true_m, true_m.createData(), q)
            for q in qs
        ]
        return np.sqrt(np.mean(np.square(err), axis=0))

    ident_err, cad_err = rms(ident_m), rms(cad_m)
    assert np.max(ident_err) < 0.03  # N·m; shoulder_lift carries ~0.5 N·m
    assert np.max(ident_err) < 0.5 * np.max(cad_err)


def test_deployment_document(solved):
    doc = identified_dynamics_dict(solved, urdf=URDF, provenance={"arm_id": "sim"})
    assert doc["format"] == IDENTIFIED_FORMAT
    assert doc["joint_names"] == ARM_JOINTS
    assert doc["locked_joints"] == ["gripper"]
    assert doc["signal"] == "current_mA" and doc["nm_per_unit"] == 0.001
    for j in ARM_JOINTS:
        assert set(doc["bodies"][j]) == {"m", "mx", "my", "mz"}
        assert doc["friction"][j]["fs"] > 0
    # Plain YAML, no numpy types: soarm_sdk loads it with yaml.safe_load.
    assert yaml.safe_load(yaml.safe_dump(doc)) == doc


def test_deployment_round_trips_through_soarm_sdk(solved):
    dynamics = pytest.importorskip("soarm_sdk.dynamics")
    doc = identified_dynamics_dict(solved, urdf=URDF)
    dyn = dynamics.IdentifiedDynamics.from_dict(doc, URDF)
    m = solved.model.copy()
    for j, b in doc["bodies"].items():
        jid = m.getJointId(j)
        m.inertias[jid] = pin.Inertia(
            b["m"],
            np.array([b["mx"], b["my"], b["mz"]]) / b["m"],
            m.inertias[jid].inertia,
        )
    d = m.createData()
    for q in np.random.default_rng(0).uniform(-1, 1, size=(10, 5)):
        np.testing.assert_allclose(
            dyn.gravity_torque(q), pin.computeGeneralizedGravity(m, d, q), atol=1e-12
        )


def test_columns_are_matched_by_name(datasets):
    q = read_log_columns(str(datasets[0]), "q", ["wrist_roll", "shoulder_pan"])
    full = read_log_columns(str(datasets[0]), "q", ARM_JOINTS)
    np.testing.assert_array_equal(q, full[:, [4, 0]])


def test_sample_rate_mismatch_is_refused(datasets, tmp_path):
    bad = tmp_path / "bad"
    bad.mkdir()
    for f in datasets[0].iterdir():
        (bad / f.name).write_bytes(f.read_bytes())
    meta = json.loads((bad / "meta.json").read_text())
    meta["rate_hz"] = 100.0
    (bad / "meta.json").write_text(json.dumps(meta))
    iden = SO101Identification(
        load_so101_robot(URDF, {"gripper": 0.0}), CONFIG, data_dir=str(bad)
    )
    with pytest.raises(ValueError, match="sampled at 100.00 Hz"):
        iden.load_trajectory_data()


def test_other_log_formats_are_refused(datasets, tmp_path):
    bad = tmp_path / "other"
    bad.mkdir()
    (bad / "meta.json").write_text(json.dumps({"format": "something/else"}))
    iden = SO101Identification(
        load_so101_robot(URDF, {"gripper": 0.0}), CONFIG, data_dir=str(bad)
    )
    with pytest.raises(ValueError, match="is not a soarm_sdk.dynamics.log/v1 log"):
        iden.load_trajectory_data()
