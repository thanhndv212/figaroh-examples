"""Captured baseline of the TIAGo mocap calibration reference (C1, #24).

Pins the observation semantics of the shipped dataset and the current fit,
so a change in either is visible. These are the *current* values, not
acceptance thresholds: a held-out protocol is #27 (C2).
See docs/development/tiago-mocap-calibration-audit-2026-10-04.md.
"""

import contextlib
import io
from pathlib import Path

import numpy as np
import pandas as pd
import pinocchio as pin
import pytest

from figaroh.calibration.calibration_tools import calc_updated_fkm
from figaroh.tools.robot import load_robot

TIAGO = Path(__file__).resolve().parents[1] / "examples" / "tiago"
CSV = TIAGO / "data/calibration/mocap/qualysis_base_hand_calibration.csv"
# The fit depends on which identifiable parameter set Pinocchio's RNG draws
# (figaroh-plus#99). Observed over draws on macOS/figaroh-dev: RMSE 2.61-3.63
# mm, max 6.27-7.24 mm; pin.seed(0) gives 2.711 / 6.966 there. The C++ RNG
# sequence is platform-dependent, so the test checks the envelope.
RMSE_MM_RANGE, MAX_MM_RANGE = (2.5, 3.8), (6.0, 7.5)
JOINTS = ["torso_lift_joint"] + [f"arm_{i}_joint" for i in range(1, 8)]


def _markers(df):
    return {k: df[[f"x{k}", f"y{k}", f"z{k}"]].to_numpy() for k in range(1, 5)}


def test_mocap_file_layout():
    df = pd.read_csv(CSV)
    assert len(df) == 34
    expected = [f"{a}{k}" for k in range(1, 5) for a in "xyz"] + JOINTS
    assert list(df.columns) == expected  # no timestamps, no orientation columns
    assert np.all(np.isfinite(df.to_numpy()))


def test_markers_are_rigid_points_not_raw_measurements():
    """Inter-marker distances vary by ~0.1 um: derived from a rigid-body pose."""
    m = _markers(pd.read_csv(CSV))
    np.testing.assert_array_equal(m[3], m[4])  # marker 4 is a copy of marker 3
    for a, b in [(1, 2), (1, 3), (2, 3)]:
        d = np.linalg.norm(m[a] - m[b], axis=1)
        assert d.max() - d.min() < 1e-6
    # All markers move with the hand: none is a static base marker.
    for k in (1, 2, 3):
        assert np.linalg.norm(m[k].std(axis=0)) > 0.2


@pytest.fixture(scope="module")
def calibrated(monkeypatch_module):
    monkeypatch_module.chdir(TIAGO)
    from examples.tiago.utils.tiago_tools import TiagoCalibration

    robot = load_robot(
        "urdf/tiago_48_schunk.urdf", load_by_urdf=True, robot_pkg="tiago_description"
    )
    # The identifiable parameter set is drawn from Pinocchio's RNG, so the fit
    # depends on its state (figaroh-plus#99); seed it to pin one draw.
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


def _rmse_mm(calib, var):
    r = calc_updated_fkm(
        calib.model, calib.data, var, calib.q_measured, calib.calib_config
    )
    err = np.linalg.norm((r - calib.PEE_measured).reshape(3, -1), axis=0)
    return float(np.sqrt(np.mean(err**2)) * 1000), float(err.max() * 1000)


def test_observation_semantics(calibrated):
    cc = calibrated.calib_config
    assert cc["start_frame"] == "universe"
    assert cc["end_frame"] == "wrist_ft_tool_link"
    assert cc["NbMarkers"] == 1  # only marker 1 is used
    assert cc["measurability"] == [True, True, True, False, False, False]
    assert cc["NbSample"] == 34
    names = cc["param_name"]
    assert len(names) == 32
    assert names[:6] == [
        "base_px",
        "base_py",
        "base_pz",
        "base_phix",
        "base_phiy",
        "base_phiz",
    ]
    assert names[-3:] == ["pEEx_1", "pEEy_1", "pEEz_1"]


def test_current_fit_baseline(calibrated):
    rmse, worst = _rmse_mm(calibrated, np.asarray(calibrated.var_, dtype=float))
    assert RMSE_MM_RANGE[0] < rmse < RMSE_MM_RANGE[1]
    assert MAX_MM_RANGE[0] < worst < MAX_MM_RANGE[1]
