"""Round-trip validation of MultiChainCalibration: the shared-torso
coupling between the left and right leg-torso-arm table-contact chains
(examples/talos_table_contact), i.e. Integration plan step 6 in the
project's report.

Mirrors tests/test_talos_table_contact.py's methodology for the single
chain: build a *known* ground truth with injected joint-placement errors
(here: ONE shared torso error plus independent leg/arm errors per side)
and ONE physical table observed from both feet, synthesize flush-touch
postures for it via inverse kinematics, and check that fitting a
*nominal* model against both chains *jointly* (torso corrections shared,
everything else independent per chain) drives both chains' gap down and
generalizes to held-out touches -- the same two properties the manuscript
itself validates, now for the two-armed case the single-chain tests
don't cover.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

PROJECT_ROOT = Path(__file__).parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

pinocchio = pytest.importorskip("pinocchio")

from examples.talos_table_contact.generate_synthetic_data import (  # noqa: E402
    BASE_FRAME,
    NOMINAL_CONTACT_OFFSET,
    NOMINAL_TABLE_POSE,
    RIGHT_BASE_FRAME,
    RIGHT_NOMINAL_CONTACT_OFFSET,
    build_two_chain_dataset,
)
from examples.talos_table_contact.utils.talos_table_tools import (  # noqa: E402
    MultiChainCalibration,
    TalosTableContactCalibration,
)
from figaroh.calibration.calibration_tools import (  # noqa: E402
    cartesian_to_SE3,
    get_rel_transform,
)

PROJECT_ROOT_EX = PROJECT_ROOT / "examples" / "talos_table_contact"
LEFT_CONFIG_PATH = PROJECT_ROOT_EX / "config" / "talos_table_left_config.yaml"
RIGHT_CONFIG_PATH = PROJECT_ROOT_EX / "config" / "talos_table_right_config.yaml"

ENCODER_NOISE_STD = np.deg2rad(0.05)


def _nominal_table_poses(robot, session_offsets, base_frame, transform_from_left):
    """Rough, uncalibrated table pose(s) in ``base_frame`` -- the same
    NOMINAL_TABLE_POSE the single-chain tests use, optionally transformed
    through the (error-free, nominal) foot-to-foot transform for the
    right chain. Deliberately computed on the *nominal* model, never the
    injected ground-truth model -- a real calibration never gets to see
    that.
    """
    left_poses = [
        NOMINAL_TABLE_POSE * cartesian_to_SE3([dx, dy, 0.0, 0.0, 0.0, 0.0])
        for dx, dy in session_offsets
    ]
    if not transform_from_left:
        return left_poses
    data = robot.model.createData()
    rightSole_M_leftSole_nominal = get_rel_transform(
        robot.model, data, RIGHT_BASE_FRAME, BASE_FRAME
    )
    return [rightSole_M_leftSole_nominal * p for p in left_poses]


def _build_chain(robot, config_path, df_train, base_frame, session_offsets):
    calib = TalosTableContactCalibration(robot, str(config_path))
    train_path = config_path.parent.parent / "data" / "_mc_test_train.csv"
    df_train.to_csv(train_path, index=False)
    calib.calib_config["data_file"] = str(train_path)
    calib._data_path = str(train_path)

    transform_from_left = base_frame == RIGHT_BASE_FRAME
    poses = _nominal_table_poses(
        robot, session_offsets, base_frame, transform_from_left
    )
    calib.set_nominal_table_poses(poses)
    calib.set_nominal_contact_offset(
        RIGHT_NOMINAL_CONTACT_OFFSET if transform_from_left else NOMINAL_CONTACT_OFFSET
    )
    calib.initialize()
    train_path.unlink(missing_ok=True)
    return calib


def _load_split(calib: TalosTableContactCalibration, df) -> None:
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


@pytest.fixture(scope="module")
def multichain_result():
    data = build_two_chain_dataset(
        n_sessions=2,
        n_train_per_session=30,
        n_val_per_session=10,
        seed=7,
        encoder_noise_std=ENCODER_NOISE_STD,
    )
    assert len(data["left_train"]) > 0, "No left touches converged."
    assert len(data["right_train"]) > 0, "No right touches converged."
    assert len(data["left_val"]) > 0, "No left held-out touches converged."
    assert len(data["right_val"]) > 0, "No right held-out touches converged."

    robot = data["robot"]
    session_offsets = data["session_offsets"]

    # Each chain's own base-parameter reduction (inside initialize(), via
    # figaroh.calibration.calibration_tools.calculate_base_kinematics_regressor)
    # draws its rank-revealing candidate configs from numpy's *global*
    # RNG state (unseeded by that module), so *which* torso axes each
    # chain happens to find identifiable -- not whether a shared-torso
    # axis exists at all -- can otherwise vary from run to run. Seeding
    # here makes that draw reproducible for this test, matching the seed
    # already used for the synthetic-touch generation above.
    np.random.seed(1234)
    calib_left = _build_chain(
        robot, LEFT_CONFIG_PATH, data["left_train"], BASE_FRAME, session_offsets
    )
    calib_right = _build_chain(
        robot, RIGHT_CONFIG_PATH, data["right_train"], RIGHT_BASE_FRAME, session_offsets
    )

    coupler = MultiChainCalibration(calib_left, calib_right)

    var0 = np.zeros(len(coupler.param_name))
    train_before = coupler.gap_metrics(var0)
    result = coupler.solve_lm()
    train_after = coupler.gap_metrics(result.x)

    _load_split(calib_left, data["left_val"])
    _load_split(calib_right, data["right_val"])
    val_before = coupler.gap_metrics(var0)
    val_after = coupler.gap_metrics(result.x)

    return {
        "coupler": coupler,
        "result": result,
        "train_before": train_before,
        "train_after": train_after,
        "val_before": val_before,
        "val_after": val_after,
        "ground_truth": data["ground_truth"],
    }


class TestSharedTorsoCoupling:
    def test_shared_names_are_torso_only_and_nonempty(self, multichain_result):
        """Torso is the only joint set structurally shared by both
        chains (left/right legs and arms are each on one chain only), so
        every shared name must reference a torso joint -- and, with the
        RNG seeded for reproducibility (see the fixture above), at least
        one axis is actually found identifiable by both chains here."""
        coupler = multichain_result["coupler"]
        assert len(coupler.shared_dx_names) > 0
        assert all(
            ("torso_1_joint" in n or "torso_2_joint" in n)
            for n in coupler.shared_dx_names
        )

    def test_union_is_smaller_than_the_naive_sum(self, multichain_result):
        """If sharing did nothing, the union's Delta X count would equal
        left.n_deltaX + right.n_deltaX. It should be strictly smaller,
        by exactly the number of shared torso (joint, axis) names."""
        coupler = multichain_result["coupler"]
        naive_sum = coupler.left.n_deltaX + coupler.right.n_deltaX
        assert coupler.n_dx < naive_sum
        assert naive_sum - coupler.n_dx == len(coupler.shared_dx_names)


class TestSolveConverges:
    def test_lm_reports_success(self, multichain_result):
        assert multichain_result["result"].success


class TestTrainingResidual:
    def test_both_chains_reduce_by_a_large_factor(self, multichain_result):
        before, after = (
            multichain_result["train_before"],
            multichain_result["train_after"],
        )
        for side in ("left", "right"):
            for key in ("z_rmse_mm", "roll_rmse_deg", "pitch_rmse_deg"):
                assert after[side][key] < before[side][key] / 3.0, (
                    f"{side}.{key} did not improve enough: "
                    f"{before[side][key]:.4f} -> {after[side][key]:.4f}"
                )


class TestHeldOutCrossValidation:
    def test_both_chains_reduce_by_a_meaningful_factor(self, multichain_result):
        before, after = (
            multichain_result["val_before"],
            multichain_result["val_after"],
        )
        for side in ("left", "right"):
            for key in ("z_rmse_mm", "roll_rmse_deg", "pitch_rmse_deg"):
                assert after[side][key] < before[side][key] / 1.5, (
                    f"{side}.{key} did not improve enough on held-out "
                    f"data: {before[side][key]:.4f} -> {after[side][key]:.4f}"
                )

    def test_absolute_residual_is_small(self, multichain_result):
        after = multichain_result["val_after"]
        for side in ("left", "right"):
            assert after[side]["z_rmse_mm"] < 4.0
            assert after[side]["roll_rmse_deg"] < 0.5
            assert after[side]["pitch_rmse_deg"] < 0.5
