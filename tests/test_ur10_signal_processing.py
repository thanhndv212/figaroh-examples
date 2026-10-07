"""UR10 CSV clock/order oracles, independent of stored simulation torques.

The loader reads one split of the truth fixture layout (t, q0..q5,
tau1..tau6; other columns ignored). These cases use quadratic positions with
known derivatives in every coordinate and sentinel efforts (#19, #90).
"""

import numpy as np
import pandas as pd
import pinocchio as pin
import pytest

from examples.ur10.utils.ur10_tools import UR10Identification
from figaroh.utils.error_handling import IdentificationError


def identifier(ts=0.01):
    obj = UR10Identification.__new__(UR10Identification)
    obj.model = pin.buildSampleModelManipulator()
    obj.identif_config = dict(ts=ts, is_joint_torques=True, is_external_wrench=False)
    obj.filter_config = {"filter_params": {"f_sample": 1 / ts}}
    return obj


def write_csv(path, ts=0.01, reordered=False, extra=True):
    times = np.arange(10) * ts
    acceleration = np.arange(1, 7) * 0.25
    q = 0.5 * times[:, None] ** 2 * acceleration
    tau = np.arange(10)[:, None] * 10 + np.arange(6)
    frame = pd.DataFrame(q, columns=[f"q{i}" for i in range(6)])
    frame[[f"tau{i}" for i in range(1, 7)]] = tau
    frame.insert(0, "t", times)
    if extra:  # the fixture's analytic derivatives are present but ignored
        frame[[f"dq{i}" for i in range(6)]] = -1.0
    if reordered:
        frame = frame[frame.columns[::-1]]
    frame.to_csv(path, index=False)
    return times, q, tau, acceleration


@pytest.mark.parametrize("ts", [0.002, 0.01])
def test_recorded_clock_all_coordinates(tmp_path, ts):
    path = tmp_path / "train.csv"
    times, q, tau, acceleration = write_csv(path, ts=ts)
    obj = identifier(ts)
    data = obj.load_trajectory_data(str(path))
    np.testing.assert_allclose(data["timestamps"].ravel(), times[:8])
    np.testing.assert_allclose(data["positions"], q[:8])
    np.testing.assert_array_equal(data["torques"], tau[:8])
    np.testing.assert_allclose(
        data["velocities"], (np.arange(8) + 0.5)[:, None] * ts * acceleration
    )
    np.testing.assert_allclose(
        data["accelerations"], np.broadcast_to(acceleration, (8, 6)), atol=1e-9
    )
    provenance = obj.trajectory_provenance[str(path.resolve())]
    assert provenance["source_sample_range"] == [0, 8]
    assert provenance["timing_source"] == "recorded"


def test_column_order_is_by_documented_names(tmp_path):
    path = tmp_path / "train.csv"
    _, q, tau, _ = write_csv(path, reordered=True)
    result = identifier().load_trajectory_data(str(path))
    np.testing.assert_allclose(result["positions"], q[:-2])
    np.testing.assert_array_equal(result["torques"], tau[:-2])


@pytest.mark.parametrize(
    "fault", ["nonfinite", "columns", "clock", "irregular", "filter_clock"]
)
def test_incoherent_inputs_fail_with_data_paths(tmp_path, fault):
    path = tmp_path / "train.csv"
    write_csv(path)
    obj = identifier()
    frame = pd.read_csv(path).astype(float)
    if fault == "nonfinite":
        frame.loc[0, "tau1"] = np.inf
    elif fault == "columns":
        frame = frame.rename(columns={"tau1": "motor_current"})
    elif fault == "clock":
        frame["t"] *= 2  # recorded at 50 Hz, configured at 100 Hz
    elif fault == "irregular":
        frame.loc[5, "t"] += 0.003
    else:
        obj.filter_config["filter_params"]["f_sample"] = 500
    frame.to_csv(path, index=False)
    with pytest.raises(IdentificationError, match=str(path)):
        obj.load_trajectory_data(str(path))


def test_default_loads_the_truth_fixture(monkeypatch):
    from pathlib import Path

    ur10 = Path(__file__).resolve().parents[1] / "examples" / "ur10"
    monkeypatch.chdir(ur10)
    obj = identifier()
    obj.model = pin.buildModelFromUrdf(str(ur10 / "urdf" / "ur10_robot.urdf"))
    data = obj.load_trajectory_data()
    truth = pd.read_csv(
        ur10 / "data" / "truth" / "train.csv", float_precision="round_trip"
    )
    assert len(data["positions"]) == len(truth) - 2
    np.testing.assert_array_equal(
        data["torques"], truth[[f"tau{i}" for i in range(1, 7)]].to_numpy()[:-2]
    )
