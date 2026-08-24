"""Validation of the TALOS table-contact calibration against real
hardware data -- not synthetic, not a round trip against an injected
ground truth (see test_talos_table_contact.py / _multichain.py for
those). This is the actual joint-encoder data the project's manuscript's
own reported results were computed from, copied from the deprecated
FIGAROH repository's data/talos/contacts/*.csv into
examples/talos_table_contact/data/real/ (see
run_calibration_real_data.py's module docstring for exact provenance and
the file-header inconsistency it works around).

There is no ground truth to recover here -- these are real, uncalibrated
joint offsets on a real robot -- so "verified" means what it means for
the manuscript itself: the flush-contact gap drops substantially on the
training postures the solver saw, *and* on a genuinely disjoint held-out
set recorded on a different day, when this implementation's own faithful
(PLANE_TPL/CONTACT_TPL, target-is-zero) cost function is fit to it.
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

from run_calibration_real_data import (  # noqa: E402
    DATA_DIR,
    LEFT_CONFIG,
    RIGHT_CONFIG,
    _build_chain,
    _load_real_touches,
    _load_split,
)
from utils.talos_table_tools import MultiChainCalibration  # noqa: E402
from generate_synthetic_data import _load_robot  # noqa: E402


@pytest.fixture(scope="module")
def robot():
    return _load_robot()


def _solve(calib):
    var0 = np.zeros(len(calib.calib_config["param_name"]))
    before = calib.gap_metrics(var0)
    result = calib.solve_lm()
    after = calib.gap_metrics(result.x)
    return var0, result, before, after


@pytest.fixture(scope="module")
def left_result(robot):
    calib = _build_chain(robot, LEFT_CONFIG, DATA_DIR / "left_train.csv")
    var0, result, train_before, train_after = _solve(calib)
    df_val = _load_real_touches(DATA_DIR / "left_validation.csv")
    _load_split(calib, df_val)
    val_before = calib.gap_metrics(var0)
    val_after = calib.gap_metrics(result.x)
    return {
        "calib": calib,
        "result": result,
        "train_before": train_before,
        "train_after": train_after,
        "val_before": val_before,
        "val_after": val_after,
    }


@pytest.fixture(scope="module")
def right_result(robot):
    calib = _build_chain(robot, RIGHT_CONFIG, DATA_DIR / "right_train.csv")
    var0, result, train_before, train_after = _solve(calib)
    df_val = _load_real_touches(DATA_DIR / "right_validation.csv")
    _load_split(calib, df_val)
    val_before = calib.gap_metrics(var0)
    val_after = calib.gap_metrics(result.x)
    return {
        "calib": calib,
        "result": result,
        "train_before": train_before,
        "train_after": train_after,
        "val_before": val_before,
        "val_after": val_after,
    }


class TestRealDataLoading:
    def test_left_and_right_touch_counts(self):
        assert len(_load_real_touches(DATA_DIR / "left_train.csv")) == 21
        assert len(_load_real_touches(DATA_DIR / "right_train.csv")) == 29
        assert len(_load_real_touches(DATA_DIR / "left_validation.csv")) == 9
        assert len(_load_real_touches(DATA_DIR / "right_validation.csv")) == 9

    def test_joint_values_are_within_talos_limits(self, robot):
        """Catches the exact bug found during development: 3 of the 4
        real CSVs have a header missing its leading placeholder columns,
        silently shifting every column by 4 if read with pandas' own
        header -- which would put metadata strings or wildly wrong
        values into joint columns. If the parser regresses to that, the
        resulting "joint angles" land far outside TALOS's actual limits."""
        model = robot.model
        for path in [
            DATA_DIR / "left_train.csv",
            DATA_DIR / "right_train.csv",
            DATA_DIR / "left_validation.csv",
            DATA_DIR / "right_validation.csv",
        ]:
            df = _load_real_touches(path)
            for name in df.columns:
                if not name.endswith("_joint") or "gripper" in name:
                    continue
                try:
                    idx = model.joints[model.getJointId(name)].idx_q
                except Exception:
                    continue
                lower, upper = (
                    model.lowerPositionLimit[idx],
                    model.upperPositionLimit[idx],
                )
                margin = 0.05  # rad, generous
                assert df[name].between(lower - margin, upper + margin).all(), (
                    f"{name} in {path.name} has values outside TALOS's own "
                    "limits -- likely a column-alignment regression."
                )


class TestRealDataSingleChain:
    @pytest.mark.parametrize("side", ["left", "right"])
    def test_solve_converges(self, side, left_result, right_result):
        result = (left_result if side == "left" else right_result)["result"]
        assert result.success

    @pytest.mark.parametrize("side", ["left", "right"])
    def test_training_gap_drops_substantially(self, side, left_result, right_result):
        r = left_result if side == "left" else right_result
        for key in ("z_rmse_mm", "roll_rmse_deg", "pitch_rmse_deg"):
            assert r["train_after"][key] < r["train_before"][key] / 2.0, (
                f"{side}.{key} training gap did not drop enough: "
                f"{r['train_before'][key]:.4f} -> {r['train_after'][key]:.4f}"
            )

    @pytest.mark.parametrize("side", ["left", "right"])
    def test_held_out_gap_drops_and_generalizes(self, side, left_result, right_result):
        """The real test: does the fit generalize to touches recorded on
        a different day that the solver never saw."""
        r = left_result if side == "left" else right_result
        for key in ("z_rmse_mm", "roll_rmse_deg", "pitch_rmse_deg"):
            assert r["val_after"][key] < r["val_before"][key] / 1.3, (
                f"{side}.{key} held-out gap did not drop enough: "
                f"{r['val_before'][key]:.4f} -> {r['val_after'][key]:.4f}"
            )

    @pytest.mark.parametrize("side", ["left", "right"])
    def test_held_out_residual_is_in_manuscript_ballpark(
        self, side, left_result, right_result
    ):
        """Not exact agreement (different rig/day/condition than the
        manuscript's own run) -- just the same order of magnitude as its
        reported ~2-5mm / ~0.3-0.6deg held-out residual, not, say, still
        hundreds of millimeters off."""
        r = left_result if side == "left" else right_result
        assert r["val_after"]["z_rmse_mm"] < 15.0
        assert r["val_after"]["roll_rmse_deg"] < 1.5
        assert r["val_after"]["pitch_rmse_deg"] < 1.5


class TestRealDataMultiChain:
    def test_shared_torso_coupling_on_real_data(self, robot):
        calib_left = _build_chain(robot, LEFT_CONFIG, DATA_DIR / "left_train.csv")
        calib_right = _build_chain(robot, RIGHT_CONFIG, DATA_DIR / "right_train.csv")
        coupler = MultiChainCalibration(calib_left, calib_right)

        assert len(coupler.shared_dx_names) > 0
        assert all("torso" in n for n in coupler.shared_dx_names)
        naive_sum = coupler.left.n_deltaX + coupler.right.n_deltaX
        assert coupler.n_dx < naive_sum

        var0 = np.zeros(len(coupler.param_name))
        before = coupler.gap_metrics(var0)
        result = coupler.solve_lm()
        assert result.success
        after = coupler.gap_metrics(result.x)

        for side in ("left", "right"):
            for key in ("z_rmse_mm", "roll_rmse_deg", "pitch_rmse_deg"):
                assert after[side][key] < before[side][key] / 2.0
