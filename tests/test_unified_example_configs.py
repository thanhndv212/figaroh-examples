"""Unified example configs, parsed against the real robot models.

Replaces the legacy-vs-unified comparisons that lived in
test_ur10_backward_compatibility.py and test_tiago_backward_compatibility.py
once the legacy flat configs were migrated (figaroh-examples#125). It keeps
their coverage of the unified path with the real UR10 / TIAGo models: the
frames, the marker count, the calibration level and the identification
signal-processing settings the example classes derive from the config.
"""

import os
import warnings
from pathlib import Path

import pytest

EXAMPLES = Path(__file__).parent.parent / "examples"
MODELS = EXAMPLES.parent / "models"


@pytest.fixture
def in_dir():
    """Run in an example directory: configs and URDFs resolve relative to it."""
    start = os.getcwd()
    yield lambda path: os.chdir(path)
    os.chdir(start)


def _load(urdf, **kwargs):
    from figaroh.tools.robot import load_robot

    return load_robot(str(urdf), load_by_urdf=True, **kwargs)


def test_ur10_unified_config(in_dir):
    from examples.ur10.utils.ur10_tools import UR10Calibration, UR10Identification

    ur10_dir = EXAMPLES / "ur10"
    in_dir(ur10_dir)
    robot = _load(ur10_dir / "urdf" / "ur10_robot.urdf", package_dirs=str(MODELS))
    config = "config/ur10_unified_config.yaml"

    with warnings.catch_warnings():
        warnings.simplefilter("error", DeprecationWarning)
        calib = UR10Calibration(robot, config).calib_config
        ident = UR10Identification(robot, config).identif_config

    assert "ur10" in calib["robot_name"].lower()
    assert calib["start_frame"] == "universe"
    assert calib["end_frame"] == "wrist_3_link"
    assert calib["NbMarkers"] == 1
    assert calib["calib_model"] == "full_params"
    assert len(ident["active_joints"]) == 6
    assert ident["ts"] == pytest.approx(0.01)
    assert not ident["has_friction"]


@pytest.mark.parametrize(
    "config,urdf,start,end,level",
    [
        (
            "config/tiago_unified_config.yaml",
            "tiago_48_schunk.urdf",
            "universe",
            "wrist_ft_tool_link",
            "joint_offset",
        ),
        (
            "config/tiago_unified_config_hey5.yaml",
            "tiago_48_hey5.urdf",
            "xtion_rgb_optical_frame",
            "hand_tool_link",
            "joint_offset",
        ),
    ],
)
def test_tiago_unified_configs(in_dir, config, urdf, start, end, level):
    from examples.tiago.utils.tiago_tools import TiagoCalibration, TiagoIdentification

    tiago_dir = EXAMPLES / "tiago"
    in_dir(tiago_dir)
    robot = _load(tiago_dir / "urdf" / urdf, robot_pkg="tiago_description")

    with warnings.catch_warnings():
        warnings.simplefilter("error", DeprecationWarning)
        calib = TiagoCalibration(robot, config).calib_config
        ident = TiagoIdentification(robot, config).identif_config

    assert "tiago" in calib["robot_name"].lower()
    assert calib["start_frame"] == start
    assert calib["end_frame"] == end
    assert calib["NbMarkers"] == 1
    assert calib["calib_model"] == level
    assert len(ident["fv"]) == 12
    assert ident["has_friction"]


def test_tiago_hey5_config_is_eye_hand(in_dir):
    """The hey5 config estimates the camera pose through a known transform."""
    from examples.tiago.utils.tiago_tools import TiagoCalibration

    tiago_dir = EXAMPLES / "tiago"
    in_dir(tiago_dir)
    robot = _load(tiago_dir / "urdf" / "tiago_48_hey5.urdf", robot_pkg="tiago_description")
    calib = TiagoCalibration(robot, "config/tiago_unified_config_hey5.yaml").calib_config
    assert calib["base_to_ref_frame"] == "xtion_link"
    assert calib["ref_frame"] == "head_2_link"
