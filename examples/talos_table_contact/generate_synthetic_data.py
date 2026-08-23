#!/usr/bin/env python3

# Copyright [2021-2026] Thanh Nguyen
# Copyright [2022-2023] [CNRS, Toward SAS]

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

There is no real hardware CSV bundled with this example -- the physical
rig (a table, an admittance controller, a force-torque sensor) lives
outside FIGAROH entirely (see README.md). This script instead builds a
*true* TALOS model with injected joint-placement errors and a *true*
table/contact-frame pose, then solves inverse kinematics for a batch of
random touch points so that, by construction, the true model's predicted
gap (see utils.talos_table_tools.gap_of_pose) is exactly zero at every
recorded posture -- exactly as a real successful contact would be. This
is what makes the calibration verifiable: the injected ground truth is
known, so the recovered parameters and the held-out cross-validation gap
can be checked against it directly.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
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

from figaroh.calibration.calibration_tools import (
    cartesian_to_SE3,
    get_sup_joints,
    update_joint_placement,
)

from utils.talos_table_tools import solve_touch_ik

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

# Nominal rig, expressed in the BASE_FRAME (left_sole_link): a table in
# front-left of the standing robot, roughly at gripper-reach height, and
# a contact-point offset roughly matching the real fingertip depth ahead
# of gripper_left_base_link (see README.md for how these were probed).
NOMINAL_TABLE_POSE = cartesian_to_SE3([0.30, 0.28, 1.00, 0.0, 0.0, 0.0])
NOMINAL_CONTACT_OFFSET = cartesian_to_SE3([0.0, 0.0, -0.12, 0.0, 0.0, 0.0])

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
):
    """Sample a random ground truth (Delta X, Delta P) and build the
    corresponding "true" model.

    ``session_offsets``, if given, is a list of (dx, dy) horizontal
    offsets (m) applied to ``NOMINAL_TABLE_POSE`` per session, on top of
    the random per-session (z, phix, thetay) -- i.e. the table was
    physically moved a bit between recording sessions, the way a rolling
    table would be in the real rig.
    """
    true_model = model.copy()
    active_joint_ids = get_sup_joints(model, BASE_FRAME, WRIST_FRAME)

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
        session_nominal = NOMINAL_TABLE_POSE * cartesian_to_SE3(
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
    true_contact_offset = NOMINAL_CONTACT_OFFSET * cartesian_to_SE3(
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
):
    """Solve IK for random touch points on each session's true table.

    Returns a DataFrame of (session_id, <15 active joint columns>) with
    one row per successfully-converged touch -- i.e. exactly what the
    real robot's own joint encoders would have logged.

    ``encoder_noise_std`` (rad), if > 0, perturbs only the *recorded*
    joint angle -- the touch itself is still geometrically exact (the IK
    solve is unaffected) -- mimicking finite encoder resolution/noise on
    an otherwise perfect physical contact.
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
            ok = False
            for attempt in range(n_seed_retries):
                q0 = q0_base.copy()
                if attempt > 0:
                    q0[config_idx] += rng.normal(0.0, 0.25, len(config_idx))
                q, ok = solve_touch_ik(
                    true_model,
                    true_data,
                    BASE_FRAME,
                    WRIST_FRAME,
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
            for name in ACTIVE_JOINT_NAMES:
                idx = model.joints[model.getJointId(name)].idx_q
                noise = rng.normal(0.0, encoder_noise_std) if encoder_noise_std else 0.0
                row[name] = float(q[idx]) + noise
            rows.append(row)

    if n_failed:
        print(f"  ({n_failed} touch targets failed to converge and were skipped)")
    return pd.DataFrame(rows)


def build_dataset(
    n_sessions: int = 2,
    n_train_per_session: int = 40,
    n_val_per_session: int = 10,
    seed: int = 0,
    encoder_noise_std: float = 0.0,
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
    return df_train, df_val, gt, robot


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
