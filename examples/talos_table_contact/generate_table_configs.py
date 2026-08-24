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
Generate whole-body, double-support-checked candidate postures for the
TALOS table-contact rig -- Integration plan step 7 ("posture generation
stays example-local") in examples/talos_table_contact's report.

Role in the pipeline
---------------------
This is deliberately a *different* generator from
``generate_synthetic_data.py``:

- ``generate_synthetic_data.py`` injects a *known, fake* joint-placement
  error into the model and solves IK against it, purely to build a
  round-trip-testable dataset with a ground truth to check the solver
  against (see the module docstring there, and tests/test_talos_table_contact.py).
- This script runs against the plain *nominal* model (no injected error
  -- there is no ground truth to know here) and is meant to produce
  candidate configurations for an actual physical data-collection
  session: every accepted posture has (a) a converged, flush left-gripper
  contact with the table, (b) the right foot re-solved back to a flat,
  neutral double-support stance given that posture's torso lean (not
  just left free to float, which a single-chain contact IK alone would
  do), and (c) the resulting whole-body center of mass verified to fall
  within the support polygon spanned by both feet -- i.e. an actually
  standable, quasi-static double-support posture, not just a kinematic
  solution to the contact constraint in isolation. Postures failing any
  of those checks (including sitting within ``limit_margin_deg`` of a
  joint's hard limit) are rejected and counted, not silently dropped.

Its CSV output uses the same joint-angle-column schema as
``generate_synthetic_data.py``'s, so it drops directly into
``TalosTableContactCalibration``'s ``data_file`` -- and, via
``figaroh.calibration.calibration_tools.load_data``, into
``BaseOptimalCalibration.load_candidate_configurations()``'s CSV path --
letting the (IROC-extended, see figaroh core's
``figaroh.optimal.base_optimal_calibration``) optimal-configuration
selection choose the minimal informative subset of these *physically
valid* candidates to actually execute on the robot.

What this does *not* check: mesh-level self-collision. TALOS's URDF in
this repo ships no SRDF collision-pairs/exclusion list, and naively
calling ``collision_model.addAllCollisionPairs()`` without one flags
every anatomically-adjacent, permanently-touching link pair as a
"collision" -- producing false positives, not a real check. Left as
future work (would need an SRDF for this URDF); see README.md.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Optional

import numpy as np
import pandas as pd
import pinocchio as pin
from scipy.spatial import Delaunay

HERE = Path(__file__).parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from figaroh.calibration.calibration_tools import (
    cartesian_to_SE3,
    get_rel_jac,
    get_rel_transform,
    get_sup_joints,
)

# Same "3 informative DOF" convention the table-contact measurement model
# itself uses (z, roll, pitch -- see talos_table_tools.gap_of_pose):
# indices into a pin.log(SE3).vector 6-vector (tx, ty, tz, rx, ry, rz).
_FLAT_AXES = [2, 3, 4]

BASE_FRAME = "left_sole_link"
WRIST_FRAME = "gripper_left_base_link"
RIGHT_BASE_FRAME = "right_sole_link"
LEFT_FOOT_FRAME = "left_sole_link"
RIGHT_FOOT_FRAME = "right_sole_link"

RIGHT_LEG_JOINT_NAMES = [
    "leg_right_1_joint",
    "leg_right_2_joint",
    "leg_right_3_joint",
    "leg_right_4_joint",
    "leg_right_5_joint",
    "leg_right_6_joint",
]

# TALOS sole dimensions are roughly 25cm x 13cm; used to build the
# support polygon at each candidate posture. Shrunk by SUPPORT_MARGIN on
# every side for a safety buffer (reject postures balanced only right at
# the edge of the feet, which a real admittance-controlled contact could
# easily tip past).
FOOT_HALF_LENGTH = 0.11
FOOT_HALF_WIDTH = 0.055
SUPPORT_MARGIN = 0.02

NOMINAL_TABLE_POSE = cartesian_to_SE3([0.30, 0.28, 1.00, 0.0, 0.0, 0.0])
NOMINAL_CONTACT_OFFSET = cartesian_to_SE3([0.0, 0.0, -0.12, 0.0, 0.0, 0.0])


def _load_robot():
    from figaroh.tools.robot import load_robot

    return load_robot(
        str(HERE.parent / "talos" / "urdf" / "talos_full_v2.urdf"),
        package_dirs=str(HERE.parent.parent / "models"),
        load_by_urdf=True,
    )


def _support_polygon_corners(model, data, q, margin=SUPPORT_MARGIN):
    """4 ground-projected corners per foot (8 total), in the model's own
    root frame, for the feet's rectangular footprint at configuration
    ``q`` -- shrunk inward by ``margin`` on every side."""
    pin.framesForwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    hl = FOOT_HALF_LENGTH - margin
    hw = FOOT_HALF_WIDTH - margin
    corners = []
    for frame in (LEFT_FOOT_FRAME, RIGHT_FOOT_FRAME):
        oMf = data.oMf[model.getFrameId(frame)]
        for sx in (-1.0, 1.0):
            for sy in (-1.0, 1.0):
                local = np.array([sx * hl, sy * hw, 0.0])
                corners.append((oMf.act(local))[:2])
    return np.array(corners)


def com_in_support_polygon(model, data, q):
    """Whole-body quasi-static double-support equilibrium check: does
    the (x, y) center of mass fall within the convex hull of both feet's
    (margin-shrunk) footprints?

    Returns (is_balanced, com_xy, margin_m) where margin_m is the
    signed distance from the CoM to the nearest hull edge (positive =
    inside), useful for ranking "how balanced" accepted postures are,
    not just pass/fail.
    """
    com = pin.centerOfMass(model, data, q)
    com_xy = com[:2]
    corners = _support_polygon_corners(model, data, q)
    hull = Delaunay(corners)
    is_inside = hull.find_simplex(com_xy) >= 0
    return bool(is_inside), com_xy


def solve_axis_masked_ik(
    model,
    data,
    base_frame: str,
    target_frame: str,
    axes,
    target_pose: pin.SE3,
    q0: np.ndarray,
    config_idx,
    max_iter: int = 500,
    damp: float = 1e-6,
    tol: float = 1e-6,
):
    """Damped-least-squares IK (as
    ``utils.talos_table_tools.solve_touch_ik``), generalized to close the
    loop on a chosen subset of the 6 pose-error ``axes`` instead of
    always all 6: ``_FLAT_AXES`` for a flat-on-the-ground foot target (a
    standing robot isn't required to plant its foot at one exact (x, y,
    yaw), only flat -- the same "3 informative DOF" convention the
    table-contact measurement model itself uses, see
    ``talos_table_tools.gap_of_pose``), or ``range(6)`` for a full-pose
    touch target. Solving the full 6-DOF pose for the foot restance step
    over-constrains the (6-DOF, but limited-range) leg for touches that
    lean the torso a lot, and fails to converge for postures that are
    perfectly fine physically.
    """
    q = np.array(q0, dtype=float).copy()
    config_idx = list(config_idx)
    axes = list(axes)
    lower = model.lowerPositionLimit
    upper = model.upperPositionLimit
    q[config_idx] = np.clip(q[config_idx], lower[config_idx], upper[config_idx])
    for _ in range(max_iter):
        pin.framesForwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        sM_t = get_rel_transform(model, data, base_frame, target_frame)
        err = pin.log(sM_t.actInv(target_pose)).vector[axes]
        if np.linalg.norm(err) < tol:
            return q, True
        Jrel = get_rel_jac(model, data, base_frame, target_frame, q)
        Jact = Jrel[axes][:, config_idx]
        dq_active = Jact.T @ np.linalg.solve(
            Jact @ Jact.T + damp * np.eye(len(axes)), err
        )
        dq = np.zeros(model.nv)
        dq[config_idx] = dq_active
        q = pin.integrate(model, q, dq)
        q[config_idx] = np.clip(q[config_idx], lower[config_idx], upper[config_idx])
    return q, False


def _near_limit(model, q, joint_ids, margin_deg=2.0):
    margin = np.deg2rad(margin_deg)
    lower = model.lowerPositionLimit
    upper = model.upperPositionLimit
    for j_id in joint_ids:
        idx = model.joints[j_id].idx_q
        if q[idx] < lower[idx] + margin or q[idx] > upper[idx] - margin:
            return True
    return False


def generate_configs(
    n_candidates: int = 200,
    seed: int = 0,
    table_half_extent=(0.16, 0.11),
    yaw_range=np.deg2rad(25.0),
    limit_margin_deg: float = 2.0,
    table_pose: Optional[pin.SE3] = None,
    contact_offset: Optional[pin.SE3] = None,
):
    """Sample random touch targets on the table, solve the full
    double-support posture for each, and keep only the ones that pass
    every check. Returns (accepted_df, report_dict).

    Expect a low acceptance rate (a handful of percent) at these
    defaults: this IK has no whole-body posture-comfort prior (unlike a
    real task-priority controller), so a fair number of otherwise-valid
    touches end up needing some joint at its limit once the right leg
    also has to re-plant for double support -- an honest property of
    this generator, not a bug (see the module docstring's "role in the
    pipeline" section). Budget ``n_candidates`` in the hundreds to a
    few thousand for a usable accepted set, or narrow
    ``table_half_extent``/``yaw_range`` for a higher yield from fewer
    candidates.
    """
    rng = np.random.default_rng(seed)
    robot = _load_robot()
    model, data = robot.model, robot.data
    table_pose = table_pose if table_pose is not None else NOMINAL_TABLE_POSE
    contact_offset = (
        contact_offset if contact_offset is not None else NOMINAL_CONTACT_OFFSET
    )

    left_joint_ids = get_sup_joints(model, BASE_FRAME, WRIST_FRAME)
    left_config_idx = [model.joints[j].idx_q for j in left_joint_ids]
    right_leg_ids = [model.getJointId(n) for n in RIGHT_LEG_JOINT_NAMES]
    right_config_idx = [model.joints[j].idx_q for j in right_leg_ids]

    # The nominal double-support stance: right foot's pose relative to
    # the left, at the model's own neutral (straight-legged) rest
    # configuration -- the target the right leg is re-solved back to
    # after each touch shifts the torso.
    q_neutral = pin.neutral(model)
    pin.framesForwardKinematics(model, data, q_neutral)
    pin.updateFramePlacements(model, data)
    # get_rel_transform(model, data, start_frame, end_frame) returns the
    # pose of end_frame expressed in start_frame -- so this must be
    # (BASE_FRAME, RIGHT_BASE_FRAME) to match how solve_touch_ik below
    # consumes it as target_wrist_pose for base_frame=BASE_FRAME,
    # wrist_frame=RIGHT_BASE_FRAME (getting this backwards makes the
    # target kinematically unreachable -- the IK then saturates against
    # joint limits chasing it instead of converging).
    nominal_stance = get_rel_transform(model, data, BASE_FRAME, RIGHT_BASE_FRAME)

    q0_base = pin.neutral(model)
    for name, val in {
        "torso_2_joint": 0.15,
        "arm_left_1_joint": 0.3,
        "arm_left_2_joint": 0.2,
        "arm_left_3_joint": -0.2,
        "arm_left_4_joint": -1.4,
    }.items():
        q0_base[model.joints[model.getJointId(name)].idx_q] = val

    rows = []
    reasons = {
        "touch_ik_failed": 0,
        "stance_ik_failed": 0,
        "near_limit": 0,
        "unbalanced": 0,
        "accepted": 0,
    }

    n_seed_retries = 5
    for _ in range(n_candidates):
        x = rng.uniform(-table_half_extent[0], table_half_extent[0])
        y = rng.uniform(-table_half_extent[1], table_half_extent[1])
        yaw = rng.uniform(-yaw_range, yaw_range)
        target_contact_pose = table_pose * cartesian_to_SE3([x, y, 0.0, 0.0, 0.0, yaw])

        target_wrist_pose = target_contact_pose * contact_offset.inverse()
        touch_ok = False
        for attempt in range(n_seed_retries):
            q0 = q0_base.copy()
            if attempt > 0:
                q0[left_config_idx] += rng.normal(0.0, 0.25, len(left_config_idx))
            q, touch_ok = solve_axis_masked_ik(
                model,
                data,
                BASE_FRAME,
                WRIST_FRAME,
                range(6),
                target_wrist_pose,
                q0,
                left_config_idx,
            )
            if touch_ok:
                break
        if not touch_ok:
            reasons["touch_ik_failed"] += 1
            continue

        # Re-plant the right foot flat on the ground at the nominal
        # stance, given this posture's torso lean -- the whole-body
        # double-support consistency step a pure single-chain contact IK
        # skips entirely.
        stance_ok = False
        for attempt in range(n_seed_retries):
            q_seed = q.copy()
            if attempt > 0:
                q_seed[right_config_idx] += rng.normal(0.0, 0.25, len(right_config_idx))
            q_stance, stance_ok = solve_axis_masked_ik(
                model,
                data,
                BASE_FRAME,
                RIGHT_BASE_FRAME,
                _FLAT_AXES,
                nominal_stance,
                q_seed,
                right_config_idx,
            )
            if stance_ok:
                q = q_stance
                break
        if not stance_ok:
            reasons["stance_ik_failed"] += 1
            continue

        if _near_limit(
            model, q, left_joint_ids + right_leg_ids, margin_deg=limit_margin_deg
        ):
            reasons["near_limit"] += 1
            continue

        balanced, com_xy = com_in_support_polygon(model, data, q)
        if not balanced:
            reasons["unbalanced"] += 1
            continue

        reasons["accepted"] += 1
        row = {"session_id": 0, "touch_x": x, "touch_y": y, "touch_yaw": yaw}
        for name in [
            str(model.names[j]) for j in left_joint_ids
        ] + RIGHT_LEG_JOINT_NAMES:
            idx = model.joints[model.getJointId(name)].idx_q
            row[name] = float(q[idx])
        rows.append(row)

    df = pd.DataFrame(rows)
    report = {"n_candidates": n_candidates, **reasons}
    return df, report


def main():
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--n-candidates", type=int, default=200)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--out",
        type=str,
        default=str(HERE / "data" / "table_contact_candidates.csv"),
    )
    args = parser.parse_args()

    df, report = generate_configs(n_candidates=args.n_candidates, seed=args.seed)

    print("Candidate posture generation:")
    for key, val in report.items():
        print(f"  {key:18s} {val}")
    if report["n_candidates"]:
        rate = 100.0 * report["accepted"] / report["n_candidates"]
        print(f"  acceptance rate    {rate:.1f}%")

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(out_path, index=False)
    print(f"\nWrote {len(df)} accepted double-support postures to {out_path}")


if __name__ == "__main__":
    main()
