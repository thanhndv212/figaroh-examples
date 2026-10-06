"""Captured baseline of the TIAGo mocap calibration reference (#67).

Pins the observation semantics of the shipped datasets and the current
training and held-out fit, so a change in either is visible. Training is the
clock-corrected 2021-11-30 session, held-out the 2021-11-26 session in the
same Qualisys ``base_frame`` body (examples/tiago/data/README.md).
"""

import contextlib
import io
from pathlib import Path

import numpy as np
import pandas as pd
import pinocchio as pin
import pytest

from figaroh.tools.robot import load_robot

TIAGO = Path(__file__).resolve().parents[1] / "examples" / "tiago"
MOCAP = TIAGO / "data/calibration/mocap"
TRAIN = MOCAP / "qualisys_2021-11-30_static_postures.csv"
HELD_OUT = MOCAP / "qualisys_2021-11-26_static_postures.csv"
JOINTS = ["torso_lift_joint"] + [f"arm_{i}_joint" for i in range(1, 8)]
MARKER_COLUMNS = [f"{a}{k}" for k in range(1, 5) for a in "xyz"]
TIMING_COLUMNS = ["t_start_robot", "t_end_robot", "marker_std_mm"]


def _markers(df):
    return {k: df[[f"x{k}", f"y{k}", f"z{k}"]].to_numpy() for k in range(1, 5)}


@pytest.mark.parametrize(
    "path, n_rows, extra",
    [(TRAIN, 37, ["shipped_row"]), (HELD_OUT, 62, [])],
)
def test_mocap_file_layout(path, n_rows, extra):
    df = pd.read_csv(path)
    assert len(df) == n_rows
    assert list(df.columns) == MARKER_COLUMNS + JOINTS + extra + TIMING_COLUMNS
    assert np.all(np.isfinite(df.to_numpy()))
    # one row per static plateau, in time order, averaged over >= 1.2 s
    duration = df.t_end_robot - df.t_start_robot
    assert duration.min() >= 1.2
    assert np.all(np.diff(df.t_start_robot) > 0)


def _distances(path):
    m = _markers(pd.read_csv(path))
    return {
        (a, b): np.linalg.norm(m[a] - m[b], axis=1)
        for a in range(1, 5)
        for b in range(a + 1, 5)
    }


def test_markers_are_points_of_one_rigid_body():
    """BL, BR, TR, TL are virtual points of one Qualisys rigid body.

    Inter-point distances are constant to < 1 um (optical noise is ~0.2 mm,
    see marker_std_mm), so the four points carry the body's 6D pose rather
    than four independent measurements. Both days use the same body
    definition, which is why held-out scoring needs no re-registration.
    """
    train, held_out = _distances(TRAIN), _distances(HELD_OUT)
    for pair, d in train.items():
        assert d.mean() > 0.05  # distinct points, not copies
        assert d.std() < 1e-6 and held_out[pair].std() < 1e-6
        assert d.mean() == pytest.approx(held_out[pair].mean(), abs=1e-4)
    for path in (TRAIN, HELD_OUT):
        assert pd.read_csv(path).marker_std_mm.median() < 0.5


@pytest.fixture(scope="module")
def calibrated(monkeypatch_module):
    monkeypatch_module.chdir(TIAGO)
    from examples.tiago.utils.tiago_tools import TiagoCalibration

    robot = load_robot(
        "urdf/tiago_48_schunk.urdf", load_by_urdf=True, robot_pkg="tiago_description"
    )
    pin.seed(0)
    np.random.seed(0)
    with contextlib.redirect_stdout(io.StringIO()):
        calib = TiagoCalibration(robot, "config/tiago_unified_config.yaml", del_list=[])
        calib.calib_config["known_baseframe"] = False
        calib.calib_config["known_tipframe"] = False
        calib.initialize()
        calib.solve(plotting=False, enable_logging=False, html_report=False)
    return calib


@pytest.fixture(scope="module")
def monkeypatch_module():
    mp = pytest.MonkeyPatch()
    yield mp
    mp.undo()


def test_observation_semantics(calibrated):
    cc = calibrated.calib_config
    assert cc["calib_model"] == "joint_offset"
    assert cc["start_frame"] == "universe"
    assert cc["end_frame"] == "wrist_ft_tool_link"
    assert cc["NbMarkers"] == 1  # only marker 1 (BL) is used
    assert cc["measurability"] == [True, True, True, False, False, False]
    assert cc["NbSample"] == 37
    assert cc["validation_data_file"].endswith(HELD_OUT.name)
    # no regularisation rows: residuals are the measurements only (#120)
    x = calibrated.LM_result.x
    assert len(calibrated.cost_function(x)) == len(calibrated.PEE_measured)
    # the 6D base absorbs the vertical torso and arm_1 (figaroh-plus#102)
    assert cc["absorbed_param_name"] == [
        "offsetPZ_torso_lift_joint",
        "offsetRZ_arm_1_joint",
    ]
    assert cc["param_name"] == (
        [f"base_p{a}" for a in "xyz"]
        + [f"base_phi{a}" for a in "xyz"]
        + [f"offsetRZ_arm_{i}_joint" for i in range(2, 7)]
        + ["pEEx_1", "pEEy_1", "pEEz_1"]
    )


def test_current_fit_baseline(calibrated):
    """Training and held-out error; arm_5 carries the dominant offset."""
    rmse = calibrated.evaluation_metrics["rmse"] * 1000
    assert rmse == pytest.approx(2.88, abs=0.05)

    held_out = calibrated._compute_validation_metrics()
    assert held_out["validation_source"] == "validation_data"
    assert held_out["n_val_samples"] == 62
    assert held_out["pos_rmse_calibrated_mm"] == pytest.approx(4.23, abs=0.05)
    assert held_out["pos_max_calibrated_mm"] == pytest.approx(11.95, abs=0.2)

    offsets = dict(zip(calibrated.calib_config["param_name"], calibrated.LM_result.x))
    arm_mrad = {k: offsets[f"offsetRZ_arm_{k}_joint"] * 1000 for k in range(2, 7)}
    assert arm_mrad[5] == pytest.approx(-49.8, abs=1.0)
    # the other identifiable arm offsets stay small (|.| < 5 mrad)
    assert max(abs(arm_mrad[k]) for k in (2, 3, 4, 6)) < 5.0
