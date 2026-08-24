"""
Round-trip validation of the TALOS table-contact (plane-calibration)
example: examples/talos_table_contact.

There is no external metrology or physical rig available in CI, so the
test builds a *known* ground truth (injected joint-placement errors,
table pose, and contact-frame offset), synthesizes flush-touch postures
for it via inverse kinematics (so the true model's predicted gap is
exactly zero at every touch, exactly like a real successful contact),
and checks that calibrating a *nominal* model against those postures:

  1. drives the training-set gap (the physical quantity that must read
     zero at a flush contact) down by a large factor,
  2. generalizes to a held-out validation set recorded from the same
     ground truth but different touch points,
  3. lands at an absolute residual level consistent with the injected
     joint-encoder noise, not merely "small on paper".

This mirrors how the project's own manuscript validates the method
(train-set residual + held-out cross-validation, both reported in mm /
degrees) rather than asserting exact recovery of the injected plane and
contact-frame parameters -- which is not fully identifiable from a
single-plane, single-chain measurement alone (a table-height error and
certain leg/torso length errors both shift the same z-gap, and no
amount of least-squares fitting on this measurement alone can tell them
apart). See examples/talos_table_contact/README.md for the full
discussion.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

PROJECT_ROOT = Path(__file__).parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

pinocchio = pytest.importorskip("pinocchio")

from examples.talos_table_contact.generate_synthetic_data import (  # noqa: E402
    NOMINAL_CONTACT_OFFSET,
    NOMINAL_TABLE_POSE,
    build_dataset,
)
from examples.talos_table_contact.utils.talos_table_tools import (  # noqa: E402
    TalosTableContactCalibration,
)
from figaroh.calibration.calibration_tools import cartesian_to_SE3  # noqa: E402

CONFIG_PATH = (
    PROJECT_ROOT
    / "examples"
    / "talos_table_contact"
    / "config"
    / "talos_table_left_config.yaml"
)

# A conservative, realistic joint-encoder noise level (degrees -> rad).
ENCODER_NOISE_STD = np.deg2rad(0.05)


def _load_split(calib: TalosTableContactCalibration, df: pd.DataFrame) -> None:
    """Point an already-initialized calibration at a different dataset,
    reusing its existing (fitted) base-parameter basis -- i.e. a
    held-out cross-validation split."""
    N = len(df)
    q0 = np.asarray(calib.calib_config["q0"], dtype=float)
    q = np.tile(q0, (N, 1))
    for j_id, name in enumerate(calib.model.names[1:], start=1):
        name = str(name)
        if name in df.columns:
            idx = calib.model.joints[j_id].idx_q
            q[:, idx] = df[name].to_numpy()
    calib.q_measured = q
    calib.session_ids = df["session_id"].to_numpy().astype(int)
    calib._fk_config["NbSample"] = N
    # _compute_logmap_residuals (core) reads calib_config["NbSample"] and
    # PEE_measured's own length, not _fk_config's -- all three must stay
    # in sync for a held-out split (PEE_measured is always the zero
    # vector -- the flush-contact target -- just resized).
    calib.calib_config["NbSample"] = N
    calib.PEE_measured = np.zeros(3 * N)


@pytest.fixture(scope="module")
def calibrated_result(tmp_path_factory):
    """Build synthetic data, run the calibration once, and hand back
    everything the individual test functions below need. Module-scoped
    since building the dataset + solving is the expensive part and every
    assertion below is a read-only check on the same result.
    """
    df_train, df_val, ground_truth, robot = build_dataset(
        n_sessions=2,
        n_train_per_session=40,
        n_val_per_session=10,
        seed=42,
        encoder_noise_std=ENCODER_NOISE_STD,
    )
    assert len(df_train) > 0, "No training touches converged -- IK setup regressed."
    assert len(df_val) > 0, "No validation touches converged -- IK setup regressed."

    tmp_dir = tmp_path_factory.mktemp("talos_table_contact")
    train_path = tmp_dir / "train.csv"
    df_train.to_csv(train_path, index=False)

    calib = TalosTableContactCalibration(robot, str(CONFIG_PATH))
    calib.calib_config["data_file"] = str(train_path)
    calib._data_path = str(train_path)

    nominal_poses = [
        NOMINAL_TABLE_POSE * cartesian_to_SE3([dx, dy, 0.0, 0.0, 0.0, 0.0])
        for dx, dy in ground_truth["session_offsets"]
    ]
    calib.set_nominal_table_poses(nominal_poses)
    calib.set_nominal_contact_offset(NOMINAL_CONTACT_OFFSET)
    calib.initialize()

    var0 = np.zeros(len(calib.calib_config["param_name"]))
    train_before = calib.gap_metrics(var0)
    result = calib.solve(
        method="lm",
        max_iterations=3,
        outlier_threshold=3.0,
        enable_logging=False,
        html_report=False,
    )
    train_after = calib.gap_metrics(result.x)

    _load_split(calib, df_val)
    val_before = calib.gap_metrics(var0)
    val_after = calib.gap_metrics(result.x)

    return {
        "calib": calib,
        "result": result,
        "ground_truth": ground_truth,
        "train_before": train_before,
        "train_after": train_after,
        "val_before": val_before,
        "val_after": val_after,
    }


class TestChainIdentifiability:
    def test_identifiable_base_parameter_count(self, calibrated_result):
        """The 15-joint leg-torso-arm chain should reduce to 57
        identifiable base parameters -- the exact count the project's
        own manuscript reports for this chain (Section: Identifiability
        of geometric parameters)."""
        assert calibrated_result["calib"].n_deltaX == 57

    def test_parameter_layout(self, calibrated_result):
        calib = calibrated_result["calib"]
        names = calib.calib_config["param_name"]
        assert len(names) == calib.n_deltaX + 3 * calib.n_sessions + 3
        tail = names[calib.n_deltaX :]
        for s in range(calib.n_sessions):
            for axis in ("plane_z", "plane_phix", "plane_thetay"):
                assert f"{axis}_s{s}" in tail
        for axis in ("contact_z", "contact_phix", "contact_thetay"):
            assert axis in tail


class TestSolveConverges:
    def test_lm_reports_success(self, calibrated_result):
        assert calibrated_result["result"].success


class TestTrainingResidual:
    """Fit quality on the postures the solver actually saw."""

    def test_reduces_by_a_large_factor(self, calibrated_result):
        before, after = (
            calibrated_result["train_before"],
            calibrated_result["train_after"],
        )
        for key in ("z_rmse_mm", "roll_rmse_deg", "pitch_rmse_deg"):
            assert after[key] < before[key] / 3.0, (
                f"{key} did not improve enough: {before[key]:.4f} -> "
                f"{after[key]:.4f}"
            )

    def test_absolute_residual_is_small(self, calibrated_result):
        after = calibrated_result["train_after"]
        assert after["z_rmse_mm"] < 2.0
        assert after["roll_rmse_deg"] < 0.3
        assert after["pitch_rmse_deg"] < 0.3


class TestHeldOutCrossValidation:
    """The real test: does the calibration generalize to touches the
    solver never saw, the same way the manuscript's own held-out
    postures validate its result."""

    def test_reduces_by_a_meaningful_factor(self, calibrated_result):
        before, after = (
            calibrated_result["val_before"],
            calibrated_result["val_after"],
        )
        for key in ("z_rmse_mm", "roll_rmse_deg", "pitch_rmse_deg"):
            assert after[key] < before[key] / 2.0, (
                f"{key} did not improve enough on held-out data: "
                f"{before[key]:.4f} -> {after[key]:.4f}"
            )

    def test_absolute_residual_is_small(self, calibrated_result):
        after = calibrated_result["val_after"]
        assert after["z_rmse_mm"] < 3.0
        assert after["roll_rmse_deg"] < 0.4
        assert after["pitch_rmse_deg"] < 0.4


class TestPlaneAndContactRecoveryOrderOfMagnitude:
    """Not exact recovery (see module docstring) -- just that the
    estimated plane/contact corrections land in the same, physically
    plausible regime as the injected ground truth rather than blowing
    up to compensate for something else entirely."""

    def test_plane_and_contact_estimates_stay_small(self, calibrated_result):
        split = calibrated_result["calib"].split_params(calibrated_result["result"].x)
        for value in split["plane"].values():
            assert abs(value) < np.deg2rad(5.0) + 0.02  # generous: rad or m
        for value in split["contact"].values():
            assert abs(value) < np.deg2rad(5.0) + 0.02
