"""Shipped cross-run data has independent clocks, frozen roles and a consumer."""

from pathlib import Path
import shutil

import numpy as np
import pandas as pd
import pytest
import yaml

from figaroh.data import Protocol
from examples.tiago.utils.identification_extraction import JOINTS, KINDS, WRIST_FT

TIAGO = Path(__file__).resolve().parents[1] / "examples/tiago"
DATA = TIAGO / "data/identification"


def test_frozen_protocol_and_config_roles():
    protocol = Protocol.load(DATA / "protocol.yaml")
    protocol.verify(DATA)
    assert protocol.version == 1
    roles = {s.role: s for s in protocol.sessions}
    assert set(roles) == {"training", "validation", "diagnostic"}
    assert {Path(p).parent.name for p in roles["training"].files} == {"dynamic"}
    assert {Path(p).parent.name for p in roles["validation"].files} == {
        "calibration_slow"
    }
    assert {Path(p).parent.name for p in roles["diagnostic"].files} == {
        "calibration_weight"
    }
    config = yaml.safe_load((TIAGO / "config/tiago_unified_config.yaml").read_text())
    assert config["tasks"]["identification"]["data"]["validation_data_file"] == (
        "data/identification/calibration_slow"
    )


@pytest.mark.parametrize(
    "folder,rows",
    [
        ("dynamic", 8022),
        ("calibration_slow", 14163),
        ("calibration_weight", 7547),
    ],
)
def test_recordings_have_matching_clocks_and_unfiltered_named_channels(folder, rows):
    reference = None
    for kind in KINDS:
        frame = pd.read_csv(DATA / folder / f"tiago_{kind}.csv")
        assert list(frame.columns) == ["t"] + [f"- {j}_{kind}" for j in JOINTS]
        assert len(frame) == rows
        assert np.isfinite(frame.to_numpy(float)).all()
        t = frame.t.to_numpy()
        assert t[0] == 0
        assert (np.diff(t) > 0).all()
        assert 99 < 1 / np.median(np.diff(t)) < 101
        if reference is not None:
            np.testing.assert_array_equal(t, reference)
        reference = t
    wrist_ft = pd.read_csv(DATA / folder / "tiago_wrist_ft.csv")
    assert list(wrist_ft.columns) == ["t"] + list(WRIST_FT)
    assert np.isfinite(wrist_ft.to_numpy(float)).all()
    np.testing.assert_array_equal(wrist_ft.t.to_numpy(), reference)


def test_default_validation_runs_through_the_identification_consumer(monkeypatch):
    from figaroh.tools.robot import load_robot
    from examples.tiago.identification import configure_identification, TRUNCATE
    from examples.tiago.utils.tiago_tools import TiagoIdentification

    monkeypatch.chdir(TIAGO)
    robot = load_robot(
        "urdf/tiago_48_hey5.urdf", load_by_urdf=True, robot_pkg="tiago_description"
    )
    ident = TiagoIdentification(robot, "config/tiago_unified_config.yaml")
    configure_identification(ident)
    assert (
        ident.identif_config["validation_data_file"]
        == "data/identification/calibration_slow"
    )
    ident.initialize(truncate=TRUNCATE)
    assert ident._val_available
    ident.solve(
        decimate=True, plotting=False, save_results=False, html_report=False, wls=False
    )
    metrics = ident._compute_validation_metrics()
    assert metrics and np.isfinite(metrics["rmse_identified"])
    provenance = ident.trajectory_provenance["data/identification/calibration_slow"]
    assert provenance["source_rows"] == 14163
    assert ident._val_num_samples > ident.num_samples  # training slice is not reused


@pytest.mark.parametrize("failure", ["missing_files", "loader_error"])
def test_entry_point_refuses_training_only_fallback(monkeypatch, capsys, failure):
    from examples.tiago import identification

    monkeypatch.chdir(TIAGO)
    args = ["identification.py", "--no-archive", "--no-html-report", "--no-verify"]
    if failure == "missing_files":
        args += ["--validation-data", "data/identification/not-a-recording"]
        expected = "data/identification/not-a-recording/tiago_position.csv"
    else:

        def unavailable(self, source):
            raise ValueError("invalid evaluation schema")

        monkeypatch.setattr(
            identification.TiagoIdentification, "_load_validation_data", unavailable
        )
        expected = (
            "Validation data could not be loaded: data/identification/calibration_slow"
        )
    monkeypatch.setattr("sys.argv", args)
    with pytest.raises(SystemExit) as error:
        identification.main()
    assert error.value.code == 1
    assert expected in capsys.readouterr().err


def test_payload_role_survives_a_directory_rename(tmp_path):
    from examples.tiago.identification import identify_evaluation_session

    relocated = tmp_path / "renamed-recording"
    shutil.copytree(DATA / "calibration_weight", relocated)
    session = identify_evaluation_session(str(relocated))
    assert session.id == "2021-07-01-1327"
    assert session.role == "diagnostic"


@pytest.mark.parametrize(
    "folder,role", [("calibration_weight", "diagnostic"), ("dynamic", "training")]
)
def test_prediction_acceptance_requires_the_validation_role(
    monkeypatch, capsys, folder, role
):
    from examples.tiago import identification

    monkeypatch.chdir(TIAGO)
    monkeypatch.setattr(
        "sys.argv",
        [
            "identification.py",
            "--validation-data",
            f"data/identification/{folder}",
            "--verification-scope",
            "prediction",
            "--no-archive",
            "--no-html-report",
        ],
    )
    with pytest.raises(SystemExit) as error:
        identification.main()
    assert error.value.code == 1
    assert (
        f"{role} recording cannot be used for prediction acceptance"
        in capsys.readouterr().err
    )


def test_payload_check_reports_the_effort_scale_against_the_ft_sensor():
    """#69: payload from the efforts vs the wrist F/T sensor.

    The F/T fit gives the hand alone (~0.79 kg, the D2 audit's 0.794) and the
    payload within the audit's 0.473 kg plus the ±0.02 kg its static-sample
    threshold moves it. With today's drive constants the effort payload
    reads 10-20 % low, outside the stated tolerance.
    """
    from examples.tiago import payload_check

    result = payload_check.check()
    for name in ("dynamic", "calibration_weight"):
        assert result["ft"][name]["residual_rms"] < 1.0
    assert result["ft"]["dynamic"]["mass"] == pytest.approx(0.794, abs=0.01)
    assert result["ft_payload"] == pytest.approx(0.473, abs=0.025)
    assert -0.20 <= result["relative_error"] <= -0.10
    assert result["tolerance"] == 0.10
    assert not result["consistent"]


def test_ft_sessions_are_recognised_by_their_joint_channels_only(tmp_path):
    """The wrist F/T file is frozen in the protocol but not read by the loader."""
    from examples.tiago.identification import identify_evaluation_session

    relocated = tmp_path / "joint-channels-only"
    relocated.mkdir()
    for kind in KINDS:
        shutil.copy(DATA / "calibration_slow" / f"tiago_{kind}.csv", relocated)
    assert identify_evaluation_session(str(relocated)).role == "validation"
