"""UR10 CSV timing/order oracles, independent of stored simulation torques."""

import numpy as np
import pandas as pd
import pinocchio as pin
import pytest

from examples.ur10.utils.ur10_tools import UR10Identification
from figaroh.utils.error_handling import IdentificationError


def identifier(ts=0.002, cap=100):
    obj = UR10Identification.__new__(UR10Identification)
    obj.model = pin.buildSampleModelManipulator()
    obj.identif_config = dict(
        ts=ts, nb_samples=cap, is_joint_torques=True, is_external_wrench=False
    )
    obj.filter_config = {"filter_params": {"f_sample": 1 / ts}}
    return obj


def write_csvs(folder, ts=0.002, torque_rows=8, reordered=False):
    times = np.arange(10) * ts
    acceleration = np.arange(1, 7) * 0.25
    q = 0.5 * times[:, None] ** 2 * acceleration
    tau = np.arange(torque_rows)[:, None] * 10 + np.arange(6)
    positions = pd.DataFrame(q, columns=[f"q{i}" for i in range(6)])
    torques = pd.DataFrame(tau, columns=[f"tau{i}" for i in range(1, 7)])
    if reordered:
        positions = positions[positions.columns[::-1]]
        torques = torques[torques.columns[::-1]]
    positions.to_csv(folder / "identification_q_simulation.csv", index=False)
    torques.to_csv(folder / "identification_tau_simulation.csv", index=False)
    return q, tau, acceleration


@pytest.mark.parametrize("ts", [0.002, 0.01])
@pytest.mark.parametrize("torque_rows", [8, 10])
def test_configured_clock_all_coordinates_and_matching_rows(tmp_path, ts, torque_rows):
    q, tau, acceleration = write_csvs(tmp_path, ts=ts, torque_rows=torque_rows)
    obj = identifier(ts, cap=7)
    data = obj.load_trajectory_data(str(tmp_path))
    np.testing.assert_allclose(data["timestamps"].ravel(), np.arange(5) * ts)
    np.testing.assert_allclose(data["positions"], q[:5])
    np.testing.assert_array_equal(data["torques"], tau[:5])
    np.testing.assert_allclose(
        data["velocities"], (np.arange(5) + 0.5)[:, None] * ts * acceleration
    )
    np.testing.assert_allclose(
        data["accelerations"], np.broadcast_to(acceleration, (5, 6)), atol=1e-10
    )
    provenance = obj.trajectory_provenance[str(tmp_path.resolve())]
    assert provenance["source_sample_range"] == [0, 5]
    assert provenance["timing_source"] == "configured_assumption"
    assert provenance["torque_generation_verified"] is False


def test_column_order_is_by_documented_names(tmp_path):
    q, tau, _ = write_csvs(tmp_path, reordered=True)
    result = identifier().load_trajectory_data(str(tmp_path))
    np.testing.assert_allclose(result["positions"], q[:-2])
    np.testing.assert_array_equal(result["torques"], tau)


@pytest.mark.parametrize(
    "fault", ["short_torque", "extra_torque", "nonfinite", "columns", "filter_clock"]
)
def test_incoherent_inputs_fail_with_data_paths(tmp_path, fault):
    write_csvs(tmp_path)
    obj = identifier()
    tau_path = tmp_path / "identification_tau_simulation.csv"
    if fault in ("short_torque", "extra_torque"):
        write_csvs(tmp_path, torque_rows=7 if fault == "short_torque" else 11)
    elif fault == "nonfinite":
        frame = pd.read_csv(tau_path).astype(float)
        frame.iloc[0, 0] = np.inf
        frame.to_csv(tau_path, index=False)
    elif fault == "columns":
        frame = pd.read_csv(tau_path).rename(columns={"tau1": "motor_current"})
        frame.to_csv(tau_path, index=False)
    else:
        obj.filter_config["filter_params"]["f_sample"] = 100
    with pytest.raises(IdentificationError, match=str(tmp_path)):
        obj.load_trajectory_data(str(tmp_path))
