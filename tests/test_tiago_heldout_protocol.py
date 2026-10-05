"""TIAGo mocap held-out protocol (#27).

Checks the freeze described in
docs/development/tiago-mocap-heldout-protocol.md (file hashes, roles, config
wiring, one rigid-body definition) and pins the protocol results.
"""

import hashlib
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pytest
import yaml

TIAGO = Path(__file__).resolve().parents[1] / "examples" / "tiago"
MOCAP = TIAGO / "data/calibration/mocap"
FROZEN = {
    "qualisys_2021-11-30_static_postures.csv": (
        "training",
        37,
        "b6c0051e20c6a077a6d2cf64ba912996945bbb85d6860cd6cea7d8ff65b1b9af",
    ),
    "qualisys_2021-11-26_static_postures.csv": (
        "validation",
        62,
        "7c986df711757c4d31d8bb6db34b354f72318a78673dac280fd3a14abc5460a0",
    ),
    "qualisys_2021-11-30-1403_static_postures.csv": (
        "confirmation",
        63,
        "e44c678fbbc1fbc852107b0fe6793470dbf3a7cef77ddf5faa25c9e4ff4cb503",
    ),
    "qualisys_2021-11-30-1504_static_postures.csv": (
        "confirmation",
        59,
        "cc19eaa74fe8b74be28d758424ef4acd18f516dc59e40754b37d87f8592fbb84",
    ),
}


@pytest.mark.parametrize("name", list(FROZEN))
def test_frozen_files_unchanged(name):
    _, rows, sha = FROZEN[name]
    path = MOCAP / name
    assert hashlib.sha256(path.read_bytes()).hexdigest() == sha
    assert len(pd.read_csv(path)) == rows


def test_roles_match_config_and_script():
    data = yaml.safe_load((TIAGO / "config/tiago_unified_config.yaml").read_text())
    data = data["tasks"]["calibration"]["data"]
    roles = {role: [] for role in ("training", "validation", "confirmation")}
    for name, (role, _, _) in FROZEN.items():
        roles[role].append(name)
    assert Path(data["source_file"]).name == roles["training"][0]
    assert Path(data["validation_data_file"]).name == roles["validation"][0]
    # confirmation sets are never wired into the calibration config
    text = (TIAGO / "config/tiago_unified_config.yaml").read_text()
    assert not any(name in text for name in roles["confirmation"])

    sys.path.insert(0, str(TIAGO.parents[1]))
    from examples.tiago.heldout_protocol import SETS

    assert sorted(SETS) == sorted((role, name) for name, (role, _, _) in FROZEN.items())


def test_one_rigid_body_definition_across_sets():
    """Same Qualisys body on every day: no re-registration is needed."""

    def distances(name):
        df = pd.read_csv(MOCAP / name)
        pts = [df[[f"x{k}", f"y{k}", f"z{k}"]].to_numpy() for k in range(1, 5)]
        return np.array(
            [
                np.linalg.norm(pts[a] - pts[b], axis=1).mean()
                for a in range(4)
                for b in range(a + 1, 4)
            ]
        )

    reference = distances(next(iter(FROZEN)))
    for name in FROZEN:
        np.testing.assert_allclose(distances(name), reference, atol=1e-4)


@pytest.fixture(scope="module")
def protocol(monkeypatch_module):
    monkeypatch_module.chdir(TIAGO)
    sys.path.insert(0, str(TIAGO.parents[1]))
    from examples.tiago import heldout_protocol

    return heldout_protocol.run()


@pytest.fixture(scope="module")
def monkeypatch_module():
    mp = pytest.MonkeyPatch()
    yield mp
    mp.undo()


HELD_OUT = [n for n, (role, _, _) in FROZEN.items() if role != "training"]
# protocol v1 norm RMSE (mm): training, validation, confirmation 14:03, 15:04
EXPECTED_RMSE = {
    "registration only": [3.37, 4.87, 4.58, 4.22],
    "joint_offset": [2.88, 4.23, 4.09, 3.83],
    "full_params": [1.64, 3.07, 2.83, 2.44],
}
# full_params keeps 31 parameters on macOS and 30 on Linux: the structural
# selection breaks pivot ties by platform, and the two sets differ on the
# data (figaroh-plus#113). Linux is up to 0.15 mm higher.
ATOL = {"registration only": 0.05, "joint_offset": 0.05, "full_params": 0.2}


def test_protocol_results(protocol):
    names = list(FROZEN)
    for model, expected in EXPECTED_RMSE.items():
        got = [protocol[model]["sets"][n]["rmse"] for n in names]
        np.testing.assert_allclose(got, expected, atol=ATOL[model], err_msg=model)
    assert protocol["registration only"]["n_params"] == 9
    assert protocol["joint_offset"]["n_params"] == 14
    assert protocol["full_params"]["n_params"] in (30, 31)
    assert protocol["joint_offset"]["arm_5"][0] * 1e3 == pytest.approx(-49.8, abs=1.0)


def test_posture_strata(protocol):
    counts = {
        n: {k: c for k, (c, _) in protocol["joint_offset"]["sets"][n]["strata"].items()}
        for n in HELD_OUT
    }
    assert [counts[n]["repeated"] for n in HELD_OUT] == [37, 37, 35]
    assert [counts[n]["new"] for n in HELD_OUT] == [16, 17, 16]
    assert [counts[n]["out_of_range"] for n in HELD_OUT] == [9, 9, 8]


def test_calibration_beats_registration_on_new_postures(protocol):
    for n in HELD_OUT:
        reg = protocol["registration only"]["sets"][n]["strata"]["new"][1]
        cal = protocol["joint_offset"]["sets"][n]["strata"]["new"][1]
        full = protocol["full_params"]["sets"][n]["strata"]["new"][1]
        assert cal < reg - 0.3, n
        assert full < cal - 1.0, n
