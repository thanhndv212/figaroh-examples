"""Tests for examples/talos_table_contact/export_report.py -- the
YAML + HTML calibration-results report (same metadata /
calibrated_parameters YAML shape as examples/tiago_pro's
write_calibration_results, adapted for TalosTableContactCalibration's
own gap-based residual since it bypasses BaseCalibration.solve() -- see
export_report.py's module docstring for why).
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

PROJECT_ROOT = Path(__file__).parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

EXAMPLE_DIR = PROJECT_ROOT / "examples" / "talos_table_contact"
if str(EXAMPLE_DIR) not in sys.path:
    sys.path.insert(0, str(EXAMPLE_DIR))

pinocchio = pytest.importorskip("pinocchio")

from export_report import (  # noqa: E402
    build_single_chain_report,
    build_two_chain_report,
    write_html_report,
    write_yaml_report,
)
from generate_synthetic_data import (  # noqa: E402
    NOMINAL_CONTACT_OFFSET,
    NOMINAL_TABLE_POSE,
    build_dataset,
)
from utils.talos_table_tools import (
    MultiChainCalibration,
    TalosTableContactCalibration,
)  # noqa: E402

CONFIG_PATH = EXAMPLE_DIR / "config" / "talos_table_left_config.yaml"


@pytest.fixture(scope="module")
def solved_chain(tmp_path_factory):
    df_train, df_val, ground_truth, robot = build_dataset(
        n_sessions=1, n_train_per_session=40, n_val_per_session=10, seed=0
    )
    train_path = tmp_path_factory.mktemp("export_report") / "train.csv"
    df_train.to_csv(train_path, index=False)

    calib = TalosTableContactCalibration(robot, str(CONFIG_PATH))
    calib.calib_config["data_file"] = str(train_path)
    calib._data_path = str(train_path)
    calib.set_nominal_table_poses([NOMINAL_TABLE_POSE])
    calib.set_nominal_contact_offset(NOMINAL_CONTACT_OFFSET)
    calib.initialize()

    var0 = np.zeros(len(calib.calib_config["param_name"]))
    train_before = calib.gap_metrics(var0)
    result = calib.solve_lm()
    train_after = calib.gap_metrics(result.x)
    n_train = calib._fk_config["NbSample"]

    return {
        "calib": calib,
        "result": result,
        "train_before": train_before,
        "train_after": train_after,
        "n_train": n_train,
    }


class TestBuildSingleChainReport:
    def test_report_has_expected_top_level_keys(self, solved_chain):
        report = build_single_chain_report(
            solved_chain["calib"],
            solved_chain["result"],
            solved_chain["train_before"],
            solved_chain["train_after"],
            n_train=solved_chain["n_train"],
        )
        assert set(report.keys()) == {"metadata", "calibrated_parameters"}

    def test_metadata_matches_the_solve(self, solved_chain):
        report = build_single_chain_report(
            solved_chain["calib"],
            solved_chain["result"],
            solved_chain["train_before"],
            solved_chain["train_after"],
            n_train=solved_chain["n_train"],
        )
        meta = report["metadata"]
        assert meta["n_identifiable_delta_x"] == solved_chain["calib"].n_deltaX
        assert meta["optimization_success"] == bool(solved_chain["result"].success)
        assert meta["training"]["z_rmse_mm"]["after"] == pytest.approx(
            solved_chain["train_after"]["z_rmse_mm"]
        )
        assert "validation" not in meta  # not passed here

    def test_every_calibrated_parameter_is_present_with_a_unit(self, solved_chain):
        report = build_single_chain_report(
            solved_chain["calib"],
            solved_chain["result"],
            solved_chain["train_before"],
            solved_chain["train_after"],
            n_train=solved_chain["n_train"],
        )
        names = solved_chain["calib"].calib_config["param_name"]
        assert set(report["calibrated_parameters"].keys()) == set(names)
        for p in report["calibrated_parameters"].values():
            assert p["unit"] in ("m", "rad")

    def test_std_dev_is_populated_when_dof_is_positive(self, solved_chain):
        """40 training samples * 3 measured DOF = 120 residuals, well
        above this chain's ~57+6 parameters -- std_dev should be real
        numbers, not the None fallback used when dof <= 0."""
        report = build_single_chain_report(
            solved_chain["calib"],
            solved_chain["result"],
            solved_chain["train_before"],
            solved_chain["train_after"],
            n_train=solved_chain["n_train"],
        )
        std_devs = [p["std_dev"] for p in report["calibrated_parameters"].values()]
        assert all(s is not None for s in std_devs)
        assert all(s >= 0 for s in std_devs)


class TestWriteReports:
    def test_yaml_and_html_round_trip(self, solved_chain, tmp_path):
        report = build_single_chain_report(
            solved_chain["calib"],
            solved_chain["result"],
            solved_chain["train_before"],
            solved_chain["train_after"],
            n_train=solved_chain["n_train"],
        )
        yaml_path = tmp_path / "sub" / "report.yaml"
        html_path = tmp_path / "sub" / "report.html"

        write_yaml_report(report, str(yaml_path))
        write_html_report(report, str(html_path), title="Test Report")

        assert yaml_path.exists()
        assert html_path.exists()

        import yaml as yaml_module

        with open(yaml_path) as f:
            reloaded = yaml_module.safe_load(f)
        assert (
            reloaded["metadata"]["n_identifiable_delta_x"]
            == solved_chain["calib"].n_deltaX
        )

        html = html_path.read_text()
        assert "Test Report" in html
        assert html.count("<table") == html.count("</table>")
        assert "converged" in html


class TestBuildTwoChainReport:
    def test_report_has_left_and_right_sections(self):
        df_train_l, _, gt, robot = build_dataset(
            n_sessions=1, n_train_per_session=30, n_val_per_session=5, seed=1
        )
        import pandas as pd
        import tempfile

        with tempfile.TemporaryDirectory() as tmpdir:
            train_path_l = Path(tmpdir) / "left.csv"
            df_train_l.to_csv(train_path_l, index=False)

            right_config = EXAMPLE_DIR / "config" / "talos_table_right_config.yaml"
            # Reuse the same synthetic left-chain data as a stand-in right
            # dataset (renaming columns is unnecessary here -- this test
            # only checks the report *shape*, not calibration quality).
            calib_left = TalosTableContactCalibration(robot, str(CONFIG_PATH))
            calib_left.calib_config["data_file"] = str(train_path_l)
            calib_left._data_path = str(train_path_l)
            calib_left.set_nominal_table_poses([NOMINAL_TABLE_POSE])
            calib_left.set_nominal_contact_offset(NOMINAL_CONTACT_OFFSET)
            calib_left.initialize()

            df_train_r = df_train_l.rename(
                columns={
                    c: c.replace("_left_", "_right_")
                    for c in df_train_l.columns
                    if "_left_" in c
                }
            )
            train_path_r = Path(tmpdir) / "right.csv"
            df_train_r.to_csv(train_path_r, index=False)
            calib_right = TalosTableContactCalibration(robot, str(right_config))
            calib_right.calib_config["data_file"] = str(train_path_r)
            calib_right._data_path = str(train_path_r)
            calib_right.set_nominal_table_poses([NOMINAL_TABLE_POSE])
            calib_right.set_nominal_contact_offset(NOMINAL_CONTACT_OFFSET)
            calib_right.initialize()

        coupler = MultiChainCalibration(calib_left, calib_right)
        var0 = np.zeros(len(coupler.param_name))
        before = coupler.gap_metrics(var0)
        result = coupler.solve_lm()
        after = coupler.gap_metrics(result.x)

        report = build_two_chain_report(coupler, result, before, after)
        assert "left_chain" in report["metadata"]
        assert "right_chain" in report["metadata"]
        assert set(report["metadata"]["training"].keys()) == {"left", "right"}
        assert set(report["calibrated_parameters"].keys()) == set(coupler.param_name)
