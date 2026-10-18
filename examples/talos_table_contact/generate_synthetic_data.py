#!/usr/bin/env python3

# Copyright [2021-2026] Thanh Nguyen
# Copyright [2022-2024] [CNRS, Toward SAS]

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

# http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Synthesize flush-touch data for the TALOS table-contact example.

Real hardware recordings are bundled separately under ``data/real/`` (see
``run_calibration_real_data.py``); the physical rig (a table, an
admittance controller, a force-torque sensor) itself lives outside
FIGAROH (see README.md). This script instead builds a
*true* TALOS model with injected joint-placement errors and a *true*
table/contact-frame pose, then solves inverse kinematics for a batch of
random touch points so that, by construction, the true model's predicted
gap (see utils.talos_table_tools.gap_of_pose) is exactly zero at every
recorded posture -- exactly as a real successful contact would be. This
is what makes the calibration verifiable: the injected ground truth is
known, so the recovered parameters and the held-out cross-validation gap
can be checked against it directly.

A second family of helpers (:func:`synthesize_touches_at_postures` and
friends) replays the *real* recorded postures instead of random ones:
each real posture is projected onto the true table surface and re-solved
by IK from the real joint values, so the synthetic study has the same
posture distribution (and counts) as the hardware data. The table can be
planar or a shallow bowl (a surface-shape mismatch the fitted plane
cannot represent). A constant table tilt is NOT a mismatch: the fitted
``plane_phix``/``plane_thetay`` absorb it exactly.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from dataclasses import dataclass
from typing import Optional

import numpy as np
import pandas as pd
import pinocchio as pin

HERE = Path(__file__).parent
# Make `from utils.talos_table_tools import ...` resolve regardless of
# whether this file is run directly (CWD = this directory) or imported
# as examples.talos_table_contact.generate_synthetic_data from elsewhere
# (e.g. the test suite).
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from figaroh.calibration.calibration_tools import (  # noqa: E402
    cartesian_to_SE3,
    get_rel_transform,
    get_sup_joints,
    update_joint_placement,
)

from utils.talos_table_tools import gap_of_pose, solve_touch_ik  # noqa: E402

BASE_FRAME = "left_sole_link"
WRIST_FRAME = "gripper_left_base_link"
ACTIVE_JOINT_NAMES = [
    "leg_left_1_joint",
    "leg_left_2_joint",
    "leg_left_3_joint",
    "leg_left_4_joint",
    "leg_left_5_joint",
    "leg_left_6_joint",
    "torso_1_joint",
    "torso_2_joint",
    "arm_left_1_joint",
    "arm_left_2_joint",
    "arm_left_3_joint",
    "arm_left_4_joint",
    "arm_left_5_joint",
    "arm_left_6_joint",
    "arm_left_7_joint",
]

# The right-side mirror of the chain above -- right leg -> torso -> right
# arm, rooted at the right foot instead of the left. Used by
# build_two_chain_dataset() together with the left chain to exercise
# MultiChainCalibration's shared-torso coupling (utils.talos_table_tools).
RIGHT_BASE_FRAME = "right_sole_link"
RIGHT_WRIST_FRAME = "gripper_right_base_link"
RIGHT_ACTIVE_JOINT_NAMES = [
    "leg_right_1_joint",
    "leg_right_2_joint",
    "leg_right_3_joint",
    "leg_right_4_joint",
    "leg_right_5_joint",
    "leg_right_6_joint",
    "torso_1_joint",
    "torso_2_joint",
    "arm_right_1_joint",
    "arm_right_2_joint",
    "arm_right_3_joint",
    "arm_right_4_joint",
    "arm_right_5_joint",
    "arm_right_6_joint",
    "arm_right_7_joint",
]

# Nominal rig, expressed in the BASE_FRAME (left_sole_link): a table in
# front-left of the standing robot, roughly at gripper-reach height, and
# a contact-point offset roughly matching the real fingertip depth ahead
# of gripper_left_base_link (see README.md for how these were probed).
NOMINAL_TABLE_POSE = cartesian_to_SE3([0.30, 0.28, 1.00, 0.0, 0.0, 0.0])
NOMINAL_CONTACT_OFFSET = cartesian_to_SE3([0.0, 0.0, -0.12, 0.0, 0.0, 0.0])
# The right gripper's own nominal fingertip offset -- same rough depth,
# independent gripper/session from the left one.
RIGHT_NOMINAL_CONTACT_OFFSET = cartesian_to_SE3([0.0, 0.0, -0.12, 0.0, 0.0, 0.0])

FULL_PARAMTPL = ["d_px", "d_py", "d_pz", "d_phix", "d_phiy", "d_phiz"]


def _load_robot():
    from figaroh.tools.robot import load_robot

    return load_robot(
        str(HERE.parent / "talos" / "urdf" / "talos_full_v2.urdf"),
        package_dirs=str(HERE.parent.parent / "models"),
        load_by_urdf=True,
    )


def build_ground_truth(
    model,
    rng: np.random.Generator,
    n_sessions: int = 2,
    joint_trans_std: float = 1.5e-3,
    joint_rot_std: float = np.deg2rad(0.3),
    plane_z_std: float = 4e-3,
    plane_tilt_std: float = np.deg2rad(0.4),
    contact_z_std: float = 3e-3,
    contact_tilt_std: float = np.deg2rad(0.4),
    session_offsets=None,
    base_frame: str = BASE_FRAME,
    wrist_frame: str = WRIST_FRAME,
    nominal_table_pose=None,
    nominal_contact_offset=None,
):
    """Sample a random ground truth (Delta X, Delta P) and build the
    corresponding "true" model.

    ``session_offsets``, if given, is a list of (dx, dy) horizontal
    offsets (m) applied to ``NOMINAL_TABLE_POSE`` per session, on top of
    the random per-session (z, phix, thetay) -- i.e. the table was
    physically moved a bit between recording sessions, the way a rolling
    table would be in the real rig.

    ``base_frame``/``wrist_frame`` select the chain (default: left);
    ``nominal_table_pose``/``nominal_contact_offset`` replace the module
    nominals the draws are applied around (e.g. the table anchored at the
    real-data fit, :func:`build_anchored_truth`). Defaults reproduce the
    original draws bit-identically.
    """
    if nominal_table_pose is None:
        nominal_table_pose = NOMINAL_TABLE_POSE
    if nominal_contact_offset is None:
        nominal_contact_offset = NOMINAL_CONTACT_OFFSET
    true_model = model.copy()
    active_joint_ids = get_sup_joints(model, base_frame, wrist_frame)

    delta_x_true = {}
    for j_id in active_joint_ids:
        j_name = model.names[j_id]
        xyz_rpy = np.concatenate(
            [
                rng.normal(0.0, joint_trans_std, 3),
                rng.normal(0.0, joint_rot_std, 3),
            ]
        )
        for axis_name, val in zip(FULL_PARAMTPL, xyz_rpy):
            delta_x_true[f"{axis_name}_{j_name}"] = float(val)
        true_model = update_joint_placement(true_model, j_id, xyz_rpy)

    if session_offsets is None:
        session_offsets = [(0.0, 0.0)] * n_sessions

    plane_true = []
    true_table_pose = []
    for s in range(n_sessions):
        dx, dy = session_offsets[s]
        session_nominal = nominal_table_pose * cartesian_to_SE3(
            [dx, dy, 0.0, 0.0, 0.0, 0.0]
        )
        z_p = float(rng.normal(0.0, plane_z_std))
        phix_p = float(rng.normal(0.0, plane_tilt_std))
        thetay_p = float(rng.normal(0.0, plane_tilt_std))
        plane_true.append({"z": z_p, "phix": phix_p, "thetay": thetay_p})
        true_table_pose.append(
            session_nominal * cartesian_to_SE3([0.0, 0.0, z_p, phix_p, thetay_p, 0.0])
        )

    contact_true = {
        "z": float(rng.normal(0.0, contact_z_std)),
        "phix": float(rng.normal(0.0, contact_tilt_std)),
        "thetay": float(rng.normal(0.0, contact_tilt_std)),
    }
    true_contact_offset = nominal_contact_offset * cartesian_to_SE3(
        [0.0, 0.0, contact_true["z"], contact_true["phix"], contact_true["thetay"], 0.0]
    )

    return {
        "true_model": true_model,
        "active_joint_ids": active_joint_ids,
        "delta_x_true": delta_x_true,
        "plane_true": plane_true,
        "true_table_pose": true_table_pose,
        "contact_true": contact_true,
        "true_contact_offset": true_contact_offset,
        "session_offsets": session_offsets,
    }


def synthesize_touches(
    model,
    ground_truth: dict,
    n_sessions: int,
    n_per_session: int,
    rng: np.random.Generator,
    table_half_extent=(0.16, 0.11),
    yaw_range=np.deg2rad(25.0),
    encoder_noise_std: float = 0.0,
    base_frame: str = BASE_FRAME,
    wrist_frame: str = WRIST_FRAME,
    active_joint_names=ACTIVE_JOINT_NAMES,
    seed_joints: Optional[dict] = None,
):
    """Solve IK for random touch points on each session's true table.

    Returns a DataFrame of (session_id, <15 active joint columns>) with
    one row per successfully-converged touch -- i.e. exactly what the
    real robot's own joint encoders would have logged.

    ``encoder_noise_std`` (rad), if > 0, perturbs only the *recorded*
    joint angle -- the touch itself is still geometrically exact (the IK
    solve is unaffected) -- mimicking finite encoder resolution/noise on
    an otherwise perfect physical contact.

    ``base_frame``/``wrist_frame``/``active_joint_names`` default to the
    left chain (module-level ``BASE_FRAME``/``WRIST_FRAME``/
    ``ACTIVE_JOINT_NAMES``); pass the ``RIGHT_*`` equivalents to
    synthesize touches for the right chain instead (see
    ``build_two_chain_dataset``). ``seed_joints`` likewise defaults to a
    left-arm-reaching seed; pass a right-arm equivalent for the right
    chain.
    """
    true_model = ground_truth["true_model"]
    true_data = true_model.createData()
    active_joint_ids = ground_truth["active_joint_ids"]
    config_idx = [model.joints[j].idx_q for j in active_joint_ids]

    # A "reaching forward" base seed, not the zero/neutral pose: much
    # closer to the true IK solutions (gripper near the table, torso
    # leaning in) than pin.neutral(), so most targets converge in well
    # under max_iter. A handful of harder targets still need a distinct
    # starting basin, so each target gets a few randomized-seed retries
    # before being counted as failed -- a standard, robust IK pattern.
    q0_base = pin.neutral(model)
    if seed_joints is None:
        seed_joints = {
            "torso_2_joint": 0.15,
            "arm_left_1_joint": 0.3,
            "arm_left_2_joint": 0.2,
            "arm_left_3_joint": -0.2,
            "arm_left_4_joint": -1.4,
        }
    for name, val in seed_joints.items():
        q0_base[model.joints[model.getJointId(name)].idx_q] = val

    n_seed_retries = 5
    rows = []
    n_failed = 0
    for s in range(n_sessions):
        table_pose = ground_truth["true_table_pose"][s]
        for _ in range(n_per_session):
            x = rng.uniform(-table_half_extent[0], table_half_extent[0])
            y = rng.uniform(-table_half_extent[1], table_half_extent[1])
            yaw = rng.uniform(-yaw_range, yaw_range)
            target_contact_pose = table_pose * cartesian_to_SE3(
                [x, y, 0.0, 0.0, 0.0, yaw]
            )
            # Retry perturbations and encoder noise come from a per-target
            # stream, so ``rng`` advances by the same amount whether or not
            # this target's IK needed retries. Otherwise a platform-level
            # difference in IK convergence would shift every later target
            # and generate a different dataset (examples#14).
            target_rng = np.random.default_rng(rng.integers(2**63))
            ok = False
            for attempt in range(n_seed_retries):
                q0 = q0_base.copy()
                if attempt > 0:
                    q0[config_idx] += target_rng.normal(0.0, 0.25, len(config_idx))
                q, ok = solve_touch_ik(
                    true_model,
                    true_data,
                    base_frame,
                    wrist_frame,
                    ground_truth["true_contact_offset"],
                    target_contact_pose,
                    q0,
                    config_idx,
                )
                if ok:
                    break
            if not ok:
                n_failed += 1
                continue
            row = {"session_id": s}
            for name in active_joint_names:
                idx = model.joints[model.getJointId(name)].idx_q
                noise = (
                    target_rng.normal(0.0, encoder_noise_std)
                    if encoder_noise_std
                    else 0.0
                )
                row[name] = float(q[idx]) + noise
            rows.append(row)

    if n_failed:
        print(f"  ({n_failed} touch targets failed to converge and were skipped)")
    return pd.DataFrame(rows)


def build_two_chain_ground_truth(
    model,
    rng: np.random.Generator,
    n_sessions: int = 2,
    joint_trans_std: float = 1.5e-3,
    joint_rot_std: float = np.deg2rad(0.3),
    plane_z_std: float = 4e-3,
    plane_tilt_std: float = np.deg2rad(0.4),
    contact_z_std: float = 3e-3,
    contact_tilt_std: float = np.deg2rad(0.4),
    session_offsets=None,
):
    """Like :func:`build_ground_truth`, but for the left *and* right
    leg-torso-arm chains at once, sharing exactly ONE injected
    torso-joint error and ONE physical table between them -- the
    scenario ``utils.talos_table_tools.MultiChainCalibration`` is built
    to fit (see its docstring for why sharing the raw torso (joint,
    axis) parameters between chains is physically sound).

    Returns a dict with ``"left"``/``"right"`` sub-dicts, each shaped
    exactly like :func:`build_ground_truth`'s return value (so
    :func:`synthesize_touches` can consume either directly), plus the
    shared/injected ground truth for inspection.
    """
    true_model = model.copy()

    left_joint_ids = get_sup_joints(model, BASE_FRAME, WRIST_FRAME)
    right_joint_ids = get_sup_joints(model, RIGHT_BASE_FRAME, RIGHT_WRIST_FRAME)
    torso_joint_ids = [j for j in left_joint_ids if j in right_joint_ids]

    delta_x_true: dict = {}

    def _inject(joint_ids):
        nonlocal true_model
        for j_id in joint_ids:
            j_name = model.names[j_id]
            already_done = f"{FULL_PARAMTPL[0]}_{j_name}" in delta_x_true
            if already_done:
                continue
            xyz_rpy = np.concatenate(
                [
                    rng.normal(0.0, joint_trans_std, 3),
                    rng.normal(0.0, joint_rot_std, 3),
                ]
            )
            for axis_name, val in zip(FULL_PARAMTPL, xyz_rpy):
                delta_x_true[f"{axis_name}_{j_name}"] = float(val)
            true_model = update_joint_placement(true_model, j_id, xyz_rpy)

    # Torso first (and only once) -- both chains then reuse these exact
    # values -- followed by each side's own leg + arm joints.
    _inject(torso_joint_ids)
    _inject(left_joint_ids)
    _inject(right_joint_ids)

    true_data = true_model.createData()

    if session_offsets is None:
        session_offsets = [(0.0, 0.0)] * n_sessions

    # Same physical table for both chains: the right-side pose is the
    # left-side pose transformed through the TRUE (error-injected)
    # foot-to-foot transform, not an independently drawn one.
    rightSole_M_leftSole_true = get_rel_transform(
        true_model, true_data, RIGHT_BASE_FRAME, BASE_FRAME
    )

    plane_true = []
    left_true_table_pose = []
    right_true_table_pose = []
    for s in range(n_sessions):
        dx, dy = session_offsets[s]
        session_nominal = NOMINAL_TABLE_POSE * cartesian_to_SE3(
            [dx, dy, 0.0, 0.0, 0.0, 0.0]
        )
        z_p = float(rng.normal(0.0, plane_z_std))
        phix_p = float(rng.normal(0.0, plane_tilt_std))
        thetay_p = float(rng.normal(0.0, plane_tilt_std))
        plane_true.append({"z": z_p, "phix": phix_p, "thetay": thetay_p})
        table_pose_left = session_nominal * cartesian_to_SE3(
            [0.0, 0.0, z_p, phix_p, thetay_p, 0.0]
        )
        left_true_table_pose.append(table_pose_left)
        right_true_table_pose.append(rightSole_M_leftSole_true * table_pose_left)

    def _draw_contact(nominal_offset):
        true_vals = {
            "z": float(rng.normal(0.0, contact_z_std)),
            "phix": float(rng.normal(0.0, contact_tilt_std)),
            "thetay": float(rng.normal(0.0, contact_tilt_std)),
        }
        offset = nominal_offset * cartesian_to_SE3(
            [0.0, 0.0, true_vals["z"], true_vals["phix"], true_vals["thetay"], 0.0]
        )
        return true_vals, offset

    left_contact_true, left_true_contact_offset = _draw_contact(NOMINAL_CONTACT_OFFSET)
    right_contact_true, right_true_contact_offset = _draw_contact(
        RIGHT_NOMINAL_CONTACT_OFFSET
    )

    left_gt = {
        "true_model": true_model,
        "active_joint_ids": left_joint_ids,
        "true_table_pose": left_true_table_pose,
        "true_contact_offset": left_true_contact_offset,
    }
    right_gt = {
        "true_model": true_model,
        "active_joint_ids": right_joint_ids,
        "true_table_pose": right_true_table_pose,
        "true_contact_offset": right_true_contact_offset,
    }
    return {
        "left": left_gt,
        "right": right_gt,
        "shared_torso_joint_ids": torso_joint_ids,
        "shared_torso_joint_names": [str(model.names[j]) for j in torso_joint_ids],
        "delta_x_true": delta_x_true,
        "plane_true": plane_true,
        "left_contact_true": left_contact_true,
        "right_contact_true": right_contact_true,
        "session_offsets": session_offsets,
    }


# A "reaching forward" seed for the right arm, mirroring the left-arm
# seed in synthesize_touches -- signs on arm_right_1/2 are flipped vs.
# arm_left_1/2 to stay inside TALOS's mirrored joint-limit ranges (see
# examples/talos/urdf/talos_full_v2.urdf); arm_3/4 use the same sign
# convention on both sides.
RIGHT_SEED_JOINTS = {
    "torso_2_joint": 0.15,
    "arm_right_1_joint": 0.3,
    "arm_right_2_joint": -0.2,
    "arm_right_3_joint": -0.2,
    "arm_right_4_joint": -1.4,
}


def build_two_chain_dataset(
    n_sessions: int = 2,
    n_train_per_session: int = 30,
    n_val_per_session: int = 8,
    seed: int = 0,
    encoder_noise_std: float = 0.0,
):
    """Build a two-chain (left + right) train/validation dataset sharing
    one injected torso error and one physical table, for exercising
    ``MultiChainCalibration``. Mirrors :func:`build_dataset`'s role for
    the single-chain case.
    """
    rng = np.random.default_rng(seed)
    robot = _load_robot()
    model = robot.model

    session_offsets = [(0.0, 0.0), (0.03, -0.05)][:n_sessions]
    ground_truth = build_two_chain_ground_truth(
        model, rng, n_sessions=n_sessions, session_offsets=session_offsets
    )

    df_left_train = synthesize_touches(
        model,
        ground_truth["left"],
        n_sessions,
        n_train_per_session,
        rng,
        encoder_noise_std=encoder_noise_std,
    )
    df_left_val = synthesize_touches(
        model,
        ground_truth["left"],
        n_sessions,
        n_val_per_session,
        rng,
        encoder_noise_std=encoder_noise_std,
    )
    df_right_train = synthesize_touches(
        model,
        ground_truth["right"],
        n_sessions,
        n_train_per_session,
        rng,
        encoder_noise_std=encoder_noise_std,
        base_frame=RIGHT_BASE_FRAME,
        wrist_frame=RIGHT_WRIST_FRAME,
        active_joint_names=RIGHT_ACTIVE_JOINT_NAMES,
        seed_joints=RIGHT_SEED_JOINTS,
    )
    df_right_val = synthesize_touches(
        model,
        ground_truth["right"],
        n_sessions,
        n_val_per_session,
        rng,
        encoder_noise_std=encoder_noise_std,
        base_frame=RIGHT_BASE_FRAME,
        wrist_frame=RIGHT_WRIST_FRAME,
        active_joint_names=RIGHT_ACTIVE_JOINT_NAMES,
        seed_joints=RIGHT_SEED_JOINTS,
    )

    return {
        "left_train": df_left_train,
        "left_val": df_left_val,
        "right_train": df_right_train,
        "right_val": df_right_val,
        "ground_truth": ground_truth,
        "robot": robot,
        "session_offsets": session_offsets,
    }


def build_dataset(
    n_sessions: int = 2,
    n_train_per_session: int = 40,
    n_val_per_session: int = 10,
    seed: int = 0,
    encoder_noise_std: float = 0.0,
    include_truth_objects: bool = False,
):
    """Build train + validation DataFrames and the ground-truth dict.

    This is the function the round-trip test imports directly (no disk
    I/O); ``main()`` below calls it and writes files for the standalone
    demo / documentation workflow.
    """
    rng = np.random.default_rng(seed)
    robot = _load_robot()
    model = robot.model

    session_offsets = [(0.0, 0.0), (0.03, -0.05)][:n_sessions]
    ground_truth = build_ground_truth(
        model, rng, n_sessions=n_sessions, session_offsets=session_offsets
    )

    df_train = synthesize_touches(
        model,
        ground_truth,
        n_sessions,
        n_train_per_session,
        rng,
        encoder_noise_std=encoder_noise_std,
    )
    df_val = synthesize_touches(
        model,
        ground_truth,
        n_sessions,
        n_val_per_session,
        rng,
        encoder_noise_std=encoder_noise_std,
    )

    gt = {
        "base_frame": BASE_FRAME,
        "wrist_frame": WRIST_FRAME,
        "n_sessions": n_sessions,
        "nominal_table_pose_xyzrpy": [0.30, 0.28, 1.00, 0.0, 0.0, 0.0],
        "nominal_contact_offset_xyzrpy": [0.0, 0.0, -0.12, 0.0, 0.0, 0.0],
        "session_offsets": session_offsets,
        "delta_x_true": ground_truth["delta_x_true"],
        "plane_true": ground_truth["plane_true"],
        "contact_true": ground_truth["contact_true"],
    }
    if include_truth_objects:
        # True model, table poses and contact offset (not JSON-serializable);
        # tests use them to compute the noise floor of the held-out residual.
        gt["truth_objects"] = ground_truth
    return df_train, df_val, gt, robot


# ---------------------------------------------------------------------------
# Synthetic touches at the real postures (figaroh-examples#98)
# ---------------------------------------------------------------------------

#: Real-data files and chain definition per side.
CHAINS = {
    "left": {
        "base": BASE_FRAME,
        "wrist": WRIST_FRAME,
        "joints": ACTIVE_JOINT_NAMES,
        "config": HERE / "config" / "talos_table_left_config.yaml",
        "train": HERE / "data" / "real" / "left_train.csv",
        "val": HERE / "data" / "real" / "left_validation.csv",
    },
    "right": {
        "base": RIGHT_BASE_FRAME,
        "wrist": RIGHT_WRIST_FRAME,
        "joints": RIGHT_ACTIVE_JOINT_NAMES,
        "config": HERE / "config" / "talos_table_right_config.yaml",
        "train": HERE / "data" / "real" / "right_train.csv",
        "val": HERE / "data" / "real" / "right_validation.csv",
    },
}

#: Bowl sag (m) reached at the 90th-percentile contact radius.
CURVED_SAG = 1e-3


@dataclass(frozen=True)
class TableSurface:
    """True table surface in the table frame: ``z = h(x, y)``.

    ``planar``: h = 0. ``curved``: a bowl
    ``h = sag * ((x-cx)^2 + (y-cy)^2) / radius^2`` centred on the centroid
    of the real contacts, so ``sag`` is the height at ``radius`` (the
    90th-percentile contact radius of the real training postures).
    """

    kind: str = "planar"
    sag: float = 0.0
    center: tuple = (0.0, 0.0)
    radius: float = 1.0

    def height(self, x, y):
        if self.kind == "planar":
            return 0.0 * np.asarray(x, dtype=float)
        r2 = (np.asarray(x) - self.center[0]) ** 2 + (
            np.asarray(y) - self.center[1]
        ) ** 2
        return self.sag * r2 / self.radius**2

    def normal(self, x, y) -> np.ndarray:
        if self.kind == "planar":
            return np.array([0.0, 0.0, 1.0])
        gx = 2.0 * self.sag * (x - self.center[0]) / self.radius**2
        gy = 2.0 * self.sag * (y - self.center[1]) / self.radius**2
        n = np.array([-gx, -gy, 1.0])
        return n / np.linalg.norm(n)


def _normal_rotation(normal: np.ndarray) -> np.ndarray:
    """Minimal rotation taking the table z axis onto ``normal``."""
    z = np.array([0.0, 0.0, 1.0])
    axis = np.cross(z, normal)
    s = np.linalg.norm(axis)
    if s < 1e-15:
        return np.eye(3)
    return pin.exp3(axis / s * np.arctan2(s, normal[2]))


def _contacts_in_table(model, truth, q, base, wrist):
    """Contact poses of postures ``q`` (n, nq) in the true table frame."""
    true_model = truth["true_model"]
    data = true_model.createData()
    table = truth["true_table_pose"][0]
    offset = truth["true_contact_offset"]
    out = []
    for qi in np.atleast_2d(q):
        pin.framesForwardKinematics(true_model, data, qi)
        pin.updateFramePlacements(true_model, data)
        contact = get_rel_transform(true_model, data, base, wrist) * offset
        out.append(table.actInv(contact))
    return out


def make_surface(model, truth, q_train, base, wrist, kind="planar", sag=CURVED_SAG):
    """Surface of ``kind`` sized on the real training postures ``q_train``."""
    if kind == "planar":
        return TableSurface("planar")
    if kind != "curved":
        raise ValueError(f"unknown surface kind {kind!r}")
    xy = np.array(
        [
            c.translation[:2]
            for c in _contacts_in_table(model, truth, q_train, base, wrist)
        ]
    )
    center = xy.mean(axis=0)
    radius = float(np.percentile(np.linalg.norm(xy - center, axis=1), 90))
    return TableSurface(
        "curved", float(sag), (float(center[0]), float(center[1])), radius
    )


def _local_frame(surface: TableSurface, x: float, y: float) -> pin.SE3:
    return pin.SE3(
        _normal_rotation(surface.normal(x, y)),
        np.array([x, y, float(surface.height(x, y))]),
    )


def project_to_table(model, truth, q, base, wrist, surface):
    """Target contact poses (in ``base``) that put the real postures on the
    true surface.

    Under the true model each posture of ``q`` (n, nq) gives a contact pose
    in the table frame. Its (x, y, yaw) is kept; the height and tilt are
    replaced by the surface's: z = h(x, y), z axis along the surface normal
    (yaw about it preserved), so the true gap there is exactly zero.
    """
    table = truth["true_table_pose"][0]
    targets = []
    for rel in _contacts_in_table(model, truth, q, base, wrist):
        x, y = float(rel.translation[0]), float(rel.translation[1])
        yaw = float(pin.rpy.matrixToRpy(rel.rotation)[2])
        local = _local_frame(surface, x, y)
        local = pin.SE3(
            local.rotation @ pin.rpy.rpyToMatrix(0.0, 0.0, yaw), local.translation
        )
        targets.append(table * local)
    return targets


def true_gap(truth, q, surface, base=BASE_FRAME, wrist=WRIST_FRAME):
    """True gap (z, roll, pitch) at postures ``q`` (n, nq), shape (n, 3).

    Relative to the true local surface at the contact's own (x, y): z is
    the distance along the surface normal, roll/pitch are about it.
    Defined at any posture (zero only at projected ones). For a planar
    surface this is :func:`utils.talos_table_tools.gap_of_pose` of the true
    table and contact. A constant table tilt cannot show up here as a
    mismatch: ``plane_phix``/``plane_thetay`` absorb it.
    """
    out = []
    for rel in _contacts_in_table(truth["true_model"], truth, q, base, wrist):
        local = _local_frame(
            surface, float(rel.translation[0]), float(rel.translation[1])
        )
        out.append(gap_of_pose(local, rel))
    return np.array(out)


def synthesize_touches_at_postures(
    model,
    truth,
    q_real,
    seed,
    surface,
    base=BASE_FRAME,
    wrist=WRIST_FRAME,
    max_attempts=8,
):
    """IK-resolve the real postures onto the true surface.

    Each target of :func:`project_to_table` is solved by
    ``solve_touch_ik`` seeded at the real posture (non-chain joints stay as
    recorded). A failed solve is retried from the real posture plus
    ``default_rng([seed, i])`` noise (growing with the attempt). Raises
    ``RuntimeError`` listing every target that never converged, so posture
    counts always match the real data. Returns ``(q_synth, info)``.
    """
    q_real = np.atleast_2d(np.asarray(q_real, dtype=float))
    true_model = truth["true_model"]
    true_data = true_model.createData()
    config_idx = [model.joints[j].idx_q for j in truth["active_joint_ids"]]
    targets = project_to_table(model, truth, q_real, base, wrist, surface)
    q_out = np.empty_like(q_real)
    failed, retries = [], 0
    for i, (q0_real, target) in enumerate(zip(q_real, targets)):
        rng = np.random.default_rng([seed, i])
        ok = False
        for attempt in range(max_attempts):
            q0 = q0_real.copy()
            if attempt > 0:
                q0[config_idx] += rng.normal(0.0, 0.05 * attempt, len(config_idx))
            q, ok = solve_touch_ik(
                true_model,
                true_data,
                base,
                wrist,
                truth["true_contact_offset"],
                target,
                q0,
                config_idx,
                tol=1e-8,
                exact_jacobian=True,
            )
            if ok:
                break
        retries += attempt
        if not ok:
            failed.append(i)
        q_out[i] = q
    if failed:
        raise RuntimeError(
            f"IK failed for {len(failed)} of {len(q_real)} targets: {failed}"
        )
    dev = np.abs(q_out[:, config_idx] - q_real[:, config_idx])
    return q_out, {"max_dev_rad": float(dev.max()), "retries": int(retries)}


def real_postures(model, csv_path) -> np.ndarray:
    """Full joint vectors (n, nq) of a real touch file; unrecorded joints
    stay at the neutral configuration."""
    from run_calibration_real_data import _load_real_touches

    df = _load_real_touches(csv_path)
    q = np.tile(pin.neutral(model), (len(df), 1))
    for j_id, name in enumerate(model.names[1:], start=1):
        if str(name) in df.columns:
            q[:, model.joints[j_id].idx_q] = df[str(name)].to_numpy()
    return q


def build_anchored_truth(robot, side, seed, surface="planar"):
    """Ground truth for one real chain, drawn around the anchored table.

    One session; 15 chain joints get 6D errors (1.5 mm / 0.3 deg), plus
    table and contact draws. The nominal table is first moved to the
    table-only fit on the real training file
    (``run_calibration_real_data.anchored_table_pose``, imported lazily:
    that module imports this one). Returns a dict with ``truth``,
    ``surface``, ``q_real_train``/``q_real_val`` and the synthesized
    ``q_train``/``q_val`` (21/9 left, 29/9 right postures) and ``info``.
    """
    from run_calibration_real_data import anchored_table_pose

    c = CHAINS[side]
    model = robot.model
    anchor = anchored_table_pose(robot, c["config"], c["train"])
    truth = build_ground_truth(
        model,
        np.random.default_rng(seed),
        n_sessions=1,
        base_frame=c["base"],
        wrist_frame=c["wrist"],
        nominal_table_pose=anchor,
    )
    q_real_train = real_postures(model, c["train"])
    q_real_val = real_postures(model, c["val"])
    surf = make_surface(model, truth, q_real_train, c["base"], c["wrist"], surface)
    q_train, info_t = synthesize_touches_at_postures(
        model, truth, q_real_train, seed, surf, c["base"], c["wrist"]
    )
    q_val, info_v = synthesize_touches_at_postures(
        model, truth, q_real_val, seed + 100_000, surf, c["base"], c["wrist"]
    )
    return {
        "truth": truth,
        "surface": surf,
        "q_real_train": q_real_train,
        "q_real_val": q_real_val,
        "q_train": q_train,
        "q_val": q_val,
        "info": {"train": info_t, "val": info_v},
    }


def main(out_dir: Optional[Path] = None):
    out_dir = Path(out_dir) if out_dir else HERE / "data"
    out_dir.mkdir(parents=True, exist_ok=True)

    df_train, df_val, gt, _ = build_dataset()

    train_path = out_dir / "table_contact_train.csv"
    val_path = out_dir / "table_contact_validation.csv"
    gt_path = out_dir / "ground_truth.json"

    df_train.to_csv(train_path, index=False)
    df_val.to_csv(val_path, index=False)
    with open(gt_path, "w") as f:
        json.dump(gt, f, indent=2)

    print(f"Wrote {len(df_train)} training touches to {train_path}")
    print(f"Wrote {len(df_val)} validation touches to {val_path}")
    print(f"Wrote ground truth to {gt_path}")


if __name__ == "__main__":
    main()
