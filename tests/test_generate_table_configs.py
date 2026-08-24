"""Checks for examples/talos_table_contact/generate_table_configs.py --
the whole-body, double-support-checked candidate-posture generator
(Integration plan step 7 in the project's report).

Unlike tests/test_talos_table_contact.py (which checks the *calibration*
recovers an injected ground truth), this file checks the *generator*: for
every posture it accepts, do the three physical claims it makes actually
hold when independently re-verified from the resulting joint angles --
flush left-gripper contact, right foot flat at the nominal double-support
stance, and whole-body center of mass inside the support polygon -- and
does its own rejection bookkeeping add up.
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

import pinocchio as pin  # noqa: E402

from examples.talos_table_contact.generate_table_configs import (  # noqa: E402
    BASE_FRAME,
    RIGHT_BASE_FRAME,
    RIGHT_LEG_JOINT_NAMES,
    WRIST_FRAME,
    _FLAT_AXES,
    _load_robot,
    com_in_support_polygon,
    generate_configs,
)
from examples.talos_table_contact.utils.talos_table_tools import (  # noqa: E402
    gap_of_pose,
)
from figaroh.calibration.calibration_tools import (  # noqa: E402
    cartesian_to_SE3,
    get_rel_transform,
    get_sup_joints,
)


@pytest.fixture(scope="module")
def generation_result():
    df, report = generate_configs(n_candidates=200, seed=0)
    return {"df": df, "report": report}


class TestReportBookkeeping:
    def test_reasons_account_for_every_candidate(self, generation_result):
        report = generation_result["report"]
        reason_keys = (
            "touch_ik_failed",
            "stance_ik_failed",
            "near_limit",
            "unbalanced",
            "accepted",
        )
        assert sum(report[k] for k in reason_keys) == report["n_candidates"]

    def test_accepted_count_matches_dataframe_length(self, generation_result):
        assert len(generation_result["df"]) == generation_result["report"]["accepted"]

    def test_produces_at_least_one_accepted_posture(self, generation_result):
        """Not a tautology: this is exactly the check that would catch a
        regression silently driving acceptance to zero (as the frame-order
        and over-constrained-stance-target bugs found during development
        did) rather than merely producing a smaller-than-ideal accepted
        set."""
        assert generation_result["report"]["accepted"] > 0


class TestAcceptedPosturesAreActuallyValid:
    """Independent re-verification: re-run the underlying physics on the
    exact joint angles the generator wrote out, using nothing from the
    generator's own internal bookkeeping -- catches the case where the
    accept/reject decision and the recorded row have silently drifted
    apart (e.g. from a stale `q` reused after a failed sub-solve)."""

    @pytest.fixture(scope="class")
    def robot_and_rows(self, generation_result):
        df = generation_result["df"]
        if len(df) == 0:
            pytest.skip("No accepted postures to verify.")
        robot = _load_robot()
        return robot, df

    def _reconstruct_q(self, model, row):
        q = pin.neutral(model)
        left_joint_ids = get_sup_joints(model, BASE_FRAME, WRIST_FRAME)
        names = [str(model.names[j]) for j in left_joint_ids] + RIGHT_LEG_JOINT_NAMES
        for name in names:
            idx = model.joints[model.getJointId(name)].idx_q
            q[idx] = row[name]
        return q

    def test_left_gripper_contact_is_flush(self, robot_and_rows):
        from examples.talos_table_contact.generate_table_configs import (
            NOMINAL_CONTACT_OFFSET,
            NOMINAL_TABLE_POSE,
        )

        robot, df = robot_and_rows
        model, data = robot.model, robot.data
        for _, row in df.iterrows():
            q = self._reconstruct_q(model, row)
            pin.framesForwardKinematics(model, data, q)
            pin.updateFramePlacements(model, data)
            sM_wrist = get_rel_transform(model, data, BASE_FRAME, WRIST_FRAME)
            sM_contact = sM_wrist * NOMINAL_CONTACT_OFFSET
            target_contact_pose = NOMINAL_TABLE_POSE * cartesian_to_SE3(
                [row["touch_x"], row["touch_y"], 0.0, 0.0, 0.0, row["touch_yaw"]]
            )
            gap = gap_of_pose(target_contact_pose, sM_contact)
            assert np.linalg.norm(gap) < 1e-4, f"contact gap too large: {gap}"

    def test_right_foot_is_flat_at_nominal_stance(self, robot_and_rows):
        robot, df = robot_and_rows
        model, data = robot.model, robot.data

        q_neutral = pin.neutral(model)
        pin.framesForwardKinematics(model, data, q_neutral)
        pin.updateFramePlacements(model, data)
        nominal_stance = get_rel_transform(model, data, BASE_FRAME, RIGHT_BASE_FRAME)

        for _, row in df.iterrows():
            q = self._reconstruct_q(model, row)
            pin.framesForwardKinematics(model, data, q)
            pin.updateFramePlacements(model, data)
            sM_right = get_rel_transform(model, data, BASE_FRAME, RIGHT_BASE_FRAME)
            err = pin.log(sM_right.actInv(nominal_stance)).vector[_FLAT_AXES]
            assert np.linalg.norm(err) < 1e-4, f"right foot not flat: {err}"

    def test_com_is_within_support_polygon(self, robot_and_rows):
        robot, df = robot_and_rows
        model, data = robot.model, robot.data
        for _, row in df.iterrows():
            q = self._reconstruct_q(model, row)
            balanced, _ = com_in_support_polygon(model, data, q)
            assert balanced

    def test_no_joint_is_past_its_limit(self, robot_and_rows):
        robot, df = robot_and_rows
        model = robot.model
        left_joint_ids = get_sup_joints(model, BASE_FRAME, WRIST_FRAME)
        names = [str(model.names[j]) for j in left_joint_ids] + RIGHT_LEG_JOINT_NAMES
        for _, row in df.iterrows():
            for name in names:
                idx = model.joints[model.getJointId(name)].idx_q
                lower = model.lowerPositionLimit[idx]
                upper = model.upperPositionLimit[idx]
                assert lower - 1e-6 <= row[name] <= upper + 1e-6
