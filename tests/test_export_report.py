"""Report-export tests, split by which pipeline generates the report:

- Single-chain (``TalosTableContactCalibration``): the *standard*
  FIGAROH pipeline -- ``solve()`` -> ``export_html_report()`` ->
  ``figaroh.tools.geometric_calibration_export.export_geometric_calibration_yaml``,
  the same one ``examples/tiago/calibration.py`` uses. This class feeds
  its plane/contact gap residual through core's own SE3 log-map
  machinery (``get_pose_from_measure``/``cost_function`` overrides, see
  ``utils/talos_table_tools.py``) specifically so it *can* use this
  pipeline unmodified, rather than a bespoke report generator.
- Two-chain (``MultiChainCalibration``): not a ``BaseCalibration``
  subclass at all (it couples two independent instances), so none of
  the above applies -- ``examples/talos_table_contact/export_report.py``
  builds a comparable YAML + HTML report by hand for this case only.
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

from export_report import build_two_chain_report  # noqa: E402
from generate_synthetic_data import (  # noqa: E402
    NOMINAL_CONTACT_OFFSET,
    NOMINAL_TABLE_POSE,
    build_dataset,
)
from utils.talos_table_tools import (  # noqa: E402
    MultiChainCalibration,
    TalosTableContactCalibration,
)
from figaroh.tools.geometric_calibration_export import (  # noqa: E402
    export_geometric_calibration_yaml,
)

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

    result = calib.solve(
        method="lm",
        max_iterations=3,
        outlier_threshold=3.0,
        enable_logging=False,
        html_report=False,
    )
    return {"calib": calib, "result": result}


class TestStandardSingleChainPipeline:
    """calib.solve() populates everything export_html_report() and
    export_geometric_calibration_yaml() need -- this is the thing that
    was broken before TalosTableContactCalibration.get_pose_from_measure/
    cost_function/_compute_validation_metrics were adapted to core's own
    residual/validation contracts (see utils/talos_table_tools.py)."""

    def test_solve_populates_evaluation_metrics(self, solved_chain):
        calib = solved_chain["calib"]
        assert solved_chain["result"].success
        assert hasattr(calib, "evaluation_metrics")
        assert calib.evaluation_metrics["rmse"] >= 0
        assert calib.evaluation_metrics["condition_number"] > 0

    def test_solve_populates_parameter_uncertainty(self, solved_chain):
        """40 training postures * 3 measured DOF = 120 residuals, well
        above this chain's ~57+6 parameters -- std_dev should be real,
        finite numbers here (see the real-data report's own note on the
        degrees-of-freedom edge case when postures are scarce)."""
        calib = solved_chain["calib"]
        assert hasattr(calib, "std_dev")
        assert len(calib.std_dev) == len(calib.calib_config["param_name"])
        assert all(np.isfinite(s) and s >= 0 for s in calib.std_dev)

    def test_export_html_report(self, solved_chain, tmp_path):
        calib = solved_chain["calib"]
        html_path = tmp_path / "sub" / "report.html"
        html_path.parent.mkdir(parents=True, exist_ok=True)
        calib.export_html_report(output_path=str(html_path))

        assert html_path.exists()
        html = html_path.read_text()
        assert html.count("<table") == html.count("</table>")
        assert "CALIBRATION" in html.upper()

    def test_export_geometric_calibration_yaml(self, solved_chain, tmp_path):
        """Only the raw joint-placement (d_p*) parameters belong in a
        PAL-style deploy YAML -- this class's PLANE_TPL/CONTACT_TPL
        entries (plane_z_s0, contact_z, ...) are this method's own
        estimates, not robot joint offsets, and must NOT appear here."""
        calib = solved_chain["calib"]
        yaml_path = tmp_path / "sub" / "master_calibration.yaml"
        yaml_path.parent.mkdir(parents=True, exist_ok=True)
        export_geometric_calibration_yaml(calib, str(yaml_path))

        assert yaml_path.exists()
        import yaml as yaml_module

        with open(yaml_path) as f:
            content = yaml_module.safe_load(f)
        calib_section = content["robot_state_publisher"]["geometric_calibration"]
        assert len(calib_section) > 0
        assert not any(
            k.startswith("plane_") or k.startswith("contact_") for k in calib_section
        )


class TestBuildTwoChainReport:
    def test_report_has_left_and_right_sections(self):
        df_train_l, _, gt, robot = build_dataset(
            n_sessions=1, n_train_per_session=30, n_val_per_session=5, seed=1
        )
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
