"""Regression for matching torque and kinematic border windows."""

import numpy as np

from examples.staubli_tx40.utils.staubli_tx40_tools import TX40Identification


def test_torque_and_states_keep_same_source_samples():
    identification = TX40Identification.__new__(TX40Identification)
    identification.identif_config = {"reduction_ratio": np.ones(6)}
    identification.filter_config = {"filter_params": {"nbutter": 4}}
    source_index = np.arange(100, dtype=float)
    currents = np.zeros((100, 6))
    currents[:, 0] = source_index
    identification.raw_data = {"torques": currents}
    identification.processed_data = {
        "timestamps": source_index[:, None],
        "positions": np.tile(source_index[:, None], (1, 6)),
        "velocities": np.tile((source_index + 100)[:, None], (1, 6)),
        "accelerations": np.tile((source_index + 200)[:, None], (1, 6)),
    }
    torque = identification.process_torque_data()
    expected = source_index[20:-20]
    np.testing.assert_array_equal(torque[:, 0], expected)
    np.testing.assert_array_equal(
        identification.processed_data["timestamps"][:, 0], expected
    )
    np.testing.assert_array_equal(
        identification.processed_data["positions"][:, 0], expected
    )
    np.testing.assert_array_equal(
        identification.processed_data["velocities"][:, 0], expected + 100
    )
    np.testing.assert_array_equal(
        identification.processed_data["accelerations"][:, 0], expected + 200
    )
    np.testing.assert_array_equal(currents[:, 0], source_index)
