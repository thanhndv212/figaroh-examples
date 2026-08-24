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
Whole-body geometric calibration of a TALOS leg-torso-arm chain from
repeated flush contact with a single flat table -- no external metrology.

This is a from-scratch port of the plane/table-contact calibration method
described in the project's internal manuscript ("Humanoid Robot Whole-body
Geometric Calibration with Embedded Sensors and a Single Plane") and
prototyped in the deprecated FIGAROH repository's
``scripts/talos_contact_calibration.py``.

Physical model
--------------
The robot stands in quasi-static whole-body equilibrium on both feet and
brings a rigid, three-fingered gripper into flush contact with a single
flat table at many different spots. Let

  - R_p  be a frame attached to the table, z-axis normal to the surface,
  - R_c  be a frame fixed to the gripper such that the 3 contact points
          lie in its own xy-plane,

Because the contact is flush, the relative pose between R_p and R_c must
have zero height and zero roll/pitch (x, y and yaw of both frames are not
observable from a flush touch and are not modeled). Neither the table's
own pose nor the gripper's exact contact-point offset is known in
advance -- both are small, unknown corrections to a rough nominal setup,
identified *jointly* with the chain's joint-placement errors by driving
the predicted gap to its least-squares minimum over every recorded
posture. This is a materially different (and more precise) idea than
regressing samples toward their own running mean, which is what the
earlier prototype script did as a practical approximation.

Parametrization
----------------
  - Delta X   : the standard FIGAROH 6-DOF-per-joint geometric offset
                model (d_px, d_py, d_pz, d_phix, d_phiy, d_phiz), reduced
                to its identifiable ("base") subset via the existing
                QR-based ``calculate_base_kinematics_regressor``.
  - Delta P   : PLANE_TPL (z, phix, thetay), one triplet per recording
                *session* (one physical table setup), plus CONTACT_TPL
                (z, phix, thetay), one triplet shared by every session
                that used the same gripper -- the fingertip-to-base-link
                offset does not change between sessions.

No core FIGAROH code is modified: this module builds on
``calc_updated_fkm`` for the chain's own forward kinematics (Delta X),
and composes the plane- and contact-frame corrections on top of it with
plain Pinocchio SE3 algebra.
"""

from __future__ import annotations

from typing import List, Optional, Sequence

import numpy as np
import pandas as pd
import pinocchio as pin

from scipy.optimize import least_squares

from figaroh.calibration.base_calibration import BaseCalibration
from figaroh.calibration.calibration_tools import (
    calc_updated_fkm,
    cartesian_to_SE3,
    get_rel_jac,
    get_rel_transform,
    initialize_variables,
)

PLANE_TPL = ["plane_z", "plane_phix", "plane_thetay"]
CONTACT_TPL = ["contact_z", "contact_phix", "contact_thetay"]


def _plane_param_names(n_sessions: int) -> List[str]:
    return [f"{name}_s{s}" for s in range(n_sessions) for name in PLANE_TPL]


def gap_of_pose(plane_pose: pin.SE3, contact_pose: pin.SE3) -> np.ndarray:
    """(z, roll, pitch) of ``contact_pose`` expressed in ``plane_pose``.

    Zero at every properly-made flush contact, by construction: the
    contact frame's origin sits on the plane (z) with its axes coplanar
    with it (roll, pitch). x, y and yaw are not returned -- a flush touch
    carries no information about them.
    """
    rel = plane_pose.actInv(contact_pose)
    rpy = pin.rpy.matrixToRpy(rel.rotation)
    return np.array([rel.translation[2], rpy[0], rpy[1]])


def solve_touch_ik(
    model,
    data,
    base_frame: str,
    wrist_frame: str,
    contact_offset: pin.SE3,
    target_contact_pose: pin.SE3,
    q0: np.ndarray,
    config_idx: Sequence[int],
    max_iter: int = 500,
    damp: float = 1e-6,
    tol: float = 1e-6,
) -> "tuple[np.ndarray, bool]":
    """Damped-least-squares IK for one flush-touch posture.

    Solves, over the joints in ``config_idx`` only (every other joint
    stays at ``q0``), for a configuration whose contact frame
    (``wrist_frame`` composed with the fixed ``contact_offset``) reaches
    ``target_contact_pose``, expressed relative to ``base_frame``.

    Mirrors the damped-least-squares loop used throughout FIGAROH's own
    IK helpers (e.g. the deprecated repo's ``CIK_problem``), built here
    from the existing, generic ``get_rel_transform``/``get_rel_jac``
    utilities so it works for any base/wrist frame pair.
    """
    q = np.array(q0, dtype=float).copy()
    target_wrist_pose = target_contact_pose * contact_offset.inverse()
    config_idx = list(config_idx)
    lower = model.lowerPositionLimit
    upper = model.upperPositionLimit
    q[config_idx] = np.clip(q[config_idx], lower[config_idx], upper[config_idx])
    for _ in range(max_iter):
        pin.framesForwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        sM_wrist = get_rel_transform(model, data, base_frame, wrist_frame)
        err = pin.log(sM_wrist.actInv(target_wrist_pose)).vector
        if np.linalg.norm(err) < tol:
            return q, True
        Jrel = get_rel_jac(model, data, base_frame, wrist_frame, q)
        Jact = Jrel[:, config_idx]
        dq_active = Jact.T @ np.linalg.solve(Jact @ Jact.T + damp * np.eye(6), err)
        dq = np.zeros(model.nv)
        dq[config_idx] = dq_active
        q = pin.integrate(model, q, dq)
        # Revolute joints are not wrapped/clamped by pin.integrate(): a
        # target on the edge of reach can otherwise "converge" several
        # full turns away from any physically valid posture. Clamping
        # every iteration keeps the search inside the real joint range,
        # matching what a real robot (and a real IK solver) would do.
        q[config_idx] = np.clip(q[config_idx], lower[config_idx], upper[config_idx])
    return q, False


class TalosTableContactCalibration(BaseCalibration):
    """Whole-body calibration of one TALOS leg-torso-arm chain from
    repeated flush contact with a single flat table.

    Subclasses :class:`~figaroh.calibration.base_calibration.BaseCalibration`
    for config loading and the QR-based base-parameter identification of
    the joint-placement errors (both fully reused, unmodified, from
    FIGAROH core), but replaces the measurement model entirely: there is
    no externally measured target pose, so ``load_data_set`` and
    ``cost_function`` are overridden from scratch rather than customized.

    The nominal table pose(s) -- one per recording session, expressed in
    the chain's ``base_frame`` -- must be supplied via
    :meth:`set_nominal_table_poses` before :meth:`initialize` is called;
    they anchor the small, jointly-estimated plane-pose correction
    (``PLANE_TPL``) the same way the URDF's nominal joint placements
    anchor the joint-offset correction (Delta X).
    """

    def __init__(self, robot, config_file: str, del_list: Optional[list] = None):
        super().__init__(robot, config_file, del_list or [])
        # No externally-known base/tip frame in this method: the plane
        # pose and the contact-frame offset are estimated by this class
        # directly (PLANE_TPL / CONTACT_TPL below), not via the generic
        # 6-DOF BASE_TPL / EE_TPL mechanism.
        self.calib_config["known_baseframe"] = True
        self.calib_config["known_tipframe"] = True
        self.regularization_coefficient = 1e-3
        self._nominal_table_pose: List[pin.SE3] = []
        self._nominal_contact_offset: pin.SE3 = pin.SE3.Identity()

    # ------------------------------------------------------------------
    # Rig configuration
    # ------------------------------------------------------------------
    def set_nominal_table_poses(self, poses: Sequence[pin.SE3]) -> None:
        """Rough, per-session table pose(s), in the chain's base frame.

        One entry per physical table setup ("session"). ``PLANE_TPL``
        estimates a small (z, phix, thetay) correction on top of each
        one -- exactly as the URDF's nominal joint placements are
        corrected by Delta X.
        """
        self._nominal_table_pose = list(poses)

    def set_nominal_contact_offset(self, pose: pin.SE3) -> None:
        """Rough tool-frame -> contact-point offset (e.g. finger length).

        Shared across every session that used the same gripper.
        ``CONTACT_TPL`` estimates a small (z, phix, thetay) correction on
        top of it, the same way ``PLANE_TPL`` corrects the table pose.
        """
        self._nominal_contact_offset = pose

    # ------------------------------------------------------------------
    # Data loading -- joint angles only, no external pose measurement
    # ------------------------------------------------------------------
    def load_data_set(self):
        df = pd.read_csv(self._data_path)
        NbSample = len(df)
        if NbSample == 0:
            raise ValueError(f"No rows in data file: {self._data_path}")

        q0 = np.asarray(self.calib_config["q0"], dtype=float)
        q = np.tile(q0, (NbSample, 1))
        joint_names = [str(n) for n in self.model.names[1:]]
        for j_id, name in enumerate(joint_names, start=1):
            if name in df.columns:
                idx = self.model.joints[j_id].idx_q
                q[:, idx] = df[name].to_numpy()

        if "session_id" in df.columns:
            session_ids = df["session_id"].to_numpy().astype(int)
        else:
            session_ids = np.zeros(NbSample, dtype=int)

        self.q_measured = q
        self.session_ids = session_ids
        self.calib_config["NbSample"] = NbSample
        # No external ground truth exists for this measurement model;
        # the "measurement" is the physical constraint that the gap is
        # zero, computed directly in cost_function().
        self.PEE_measured = np.zeros(3 * NbSample)

    # ------------------------------------------------------------------
    # Parameter list: Delta X (base params, via core FIGAROH) + Delta P
    # ------------------------------------------------------------------
    def initialize(self):
        super().initialize()  # load_data_set() + create_param_list()

        if not self._nominal_table_pose:
            raise RuntimeError("Call set_nominal_table_poses(...) before initialize().")

        self.n_sessions = (
            int(self.session_ids.max()) + 1 if len(self.session_ids) else 1
        )
        if len(self._nominal_table_pose) != self.n_sessions:
            raise ValueError(
                f"Expected {self.n_sessions} nominal table pose(s) "
                f"(one per session id in the data), got "
                f"{len(self._nominal_table_pose)}."
            )

        self.n_deltaX = len(self.calib_config["param_name"])
        plane_names = _plane_param_names(self.n_sessions)
        self.calib_config["param_name"] = (
            self.calib_config["param_name"] + plane_names + list(CONTACT_TPL)
        )

        # A copy of calib_config for calc_updated_fkm(): full 6-DOF chain
        # pose (needed so this class can compose the plane/contact
        # frames on top of it), Delta-X-only parameter list.
        self._fk_config = dict(self.calib_config)
        self._fk_config["measurability"] = [True] * 6
        self._fk_config["calibration_index"] = 6
        self._fk_config["param_name"] = self.calib_config["param_name"][: self.n_deltaX]

    # ------------------------------------------------------------------
    # Measurement model: predicted (z, roll, pitch) gap, target is zero
    # ------------------------------------------------------------------
    def cost_function(self, var: np.ndarray) -> np.ndarray:
        n_dx = self.n_deltaX
        n_plane = 3 * self.n_sessions
        var_dx = var[:n_dx]
        var_plane = var[n_dx : n_dx + n_plane]
        var_contact = var[n_dx + n_plane :]

        PEEe = calc_updated_fkm(
            self.model, self.data, var_dx, self.q_measured, self._fk_config
        )
        N = self._fk_config["NbSample"]
        contact_offset = self._nominal_contact_offset * cartesian_to_SE3(
            [0.0, 0.0, var_contact[0], var_contact[1], var_contact[2], 0.0]
        )

        gap = np.empty((3, N))
        for i in range(N):
            loc = np.array(
                [
                    PEEe[i],
                    PEEe[N + i],
                    PEEe[2 * N + i],
                    PEEe[3 * N + i],
                    PEEe[4 * N + i],
                    PEEe[5 * N + i],
                ]
            )
            # base_frame -> wrist, with Delta X applied
            sM_wrist = cartesian_to_SE3(loc)
            sM_contact = sM_wrist * contact_offset

            s = int(self.session_ids[i])
            p = var_plane[3 * s : 3 * s + 3]
            sM_plane = self._nominal_table_pose[s] * cartesian_to_SE3(
                [0.0, 0.0, p[0], p[1], p[2], 0.0]
            )
            gap[:, i] = gap_of_pose(sM_plane, sM_contact)

        reg = np.sqrt(self.regularization_coefficient) * var
        return np.concatenate([gap.flatten("C"), reg])

    # ------------------------------------------------------------------
    # Solve
    # ------------------------------------------------------------------
    def solve_lm(self, **least_squares_kwargs):
        """Levenberg-Marquardt solve of :meth:`cost_function`.

        Deliberately bypasses ``BaseCalibration.solve()`` /
        ``solve_optimisation()``: their outlier-detection and evaluation
        machinery assumes a residual laid out as one row per (sample,
        measured DOF) matching ``self.PEE_measured`` -- a shape this
        class's residual (gap rows + a regularization tail, and no
        externally measured target at all) does not share. Driving
        ``scipy.optimize.least_squares`` directly here keeps that
        mismatch from ever becoming a silent bug.

        Returns the ``scipy.optimize.OptimizeResult`` and stores it as
        ``self.LM_result``; also sets ``self.STATUS``.
        """
        var0, _ = initialize_variables(self.calib_config, mode=0)
        kwargs = dict(method="lm", xtol=1e-12, ftol=1e-12, max_nfev=2000)
        kwargs.update(least_squares_kwargs)
        result = least_squares(self.cost_function, var0, **kwargs)
        self.LM_result = result
        self.STATUS = "CALIBRATED"
        return result

    # ------------------------------------------------------------------
    # Convenience accessors
    # ------------------------------------------------------------------
    def split_params(self, var: np.ndarray) -> dict:
        """Split a flat parameter vector into its named blocks."""
        n_dx = self.n_deltaX
        n_plane = 3 * self.n_sessions
        return {
            "delta_x": dict(zip(self.calib_config["param_name"][:n_dx], var[:n_dx])),
            "plane": dict(
                zip(
                    self.calib_config["param_name"][n_dx : n_dx + n_plane],
                    var[n_dx : n_dx + n_plane],
                )
            ),
            "contact": dict(
                zip(
                    self.calib_config["param_name"][n_dx + n_plane :],
                    var[n_dx + n_plane :],
                )
            ),
        }

    def gap_rms(self, var: np.ndarray) -> float:
        """RMS of the physical (z, roll/pitch-weighted) gap only, excluding
        the regularization tail of :meth:`cost_function`."""
        n_gap = 3 * self._fk_config["NbSample"]
        residual = self.cost_function(var)[:n_gap]
        return float(np.sqrt(np.mean(residual**2)))

    def gap_metrics(self, var: np.ndarray) -> dict:
        """Per-axis gap RMS in physically interpretable units.

        Mirrors the way the manuscript itself reports calibration
        quality (mm along the table normal, degrees of roll/pitch)
        rather than one mixed-unit combined residual.
        """
        N = self._fk_config["NbSample"]
        residual = self.cost_function(var)[: 3 * N].reshape(3, N)
        z, roll, pitch = residual[0], residual[1], residual[2]
        return {
            "z_rmse_mm": float(np.sqrt(np.mean(z**2)) * 1000.0),
            "roll_rmse_deg": float(np.sqrt(np.mean(roll**2)) * 180.0 / np.pi),
            "pitch_rmse_deg": float(np.sqrt(np.mean(pitch**2)) * 180.0 / np.pi),
        }


class MultiChainCalibration:
    """Couples two :class:`TalosTableContactCalibration` chains (left and
    right leg-torso-arm) so that a joint-placement correction shared by
    both chains -- physically, the torso joints, which sit on the
    kinematic path of *both* ``left_sole_link -> gripper_left_base_link``
    and ``right_sole_link -> gripper_right_base_link`` -- is fit as a
    single, shared value, rather than as two independent (and generally
    inconsistent) per-chain estimates of what is really the same
    physical joint offset.

    Why sharing by name is physically sound, not just cosmetic
    -----------------------------------------------------------------
    Each chain's own base-parameter reduction (the QR step inside
    ``BaseCalibration.create_param_list`` /
    ``calculate_base_kinematics_regressor``) does not report *combined*
    parameters under new names; it reports a well-conditioned *subset* of
    the raw, single-(joint, axis) ``d_px_<joint>`` / ... names, dropping
    the ones that chain cannot identify. ``calc_updated_fkm`` then looks
    each one up by a literal substring match against the joint's own name
    (see ``figaroh.calibration.calibration_tools.calc_updated_fkm``), so
    a raw name's meaning -- "the correction to this joint's own
    placement" -- does not depend on which chain is evaluating it. If the
    same ``(joint, axis)`` name is independently found identifiable by
    *both* chains (expected for the torso here, since torso-axis
    identifiability is primarily a local/structural property), treating
    it as one shared scalar fed into both chains' forward kinematics is
    exactly the coupling a real two-armed calibration needs -- no
    combined-regressor machinery has to be built or added to core to get
    it right.

    Everything else -- each session's plane-pose correction, and each
    side's own gripper contact-frame offset -- stays independent per
    chain, since those really are physically distinct per side/session.

    Both ``calib_left`` and ``calib_right`` must already be constructed,
    given their nominal table pose(s)/contact offset, and
    ``initialize()``-d before being passed in here.
    """

    def __init__(
        self,
        calib_left: TalosTableContactCalibration,
        calib_right: TalosTableContactCalibration,
        regularization_coefficient: float = 1e-3,
    ):
        self.left = calib_left
        self.right = calib_right
        self.regularization_coefficient = regularization_coefficient

        left_dx_names = list(calib_left._fk_config["param_name"])
        right_dx_names = list(calib_right._fk_config["param_name"])
        union = list(left_dx_names)
        for name in right_dx_names:
            if name not in union:
                union.append(name)
        self.dx_names = union
        self.n_dx = len(union)
        self.shared_dx_names = sorted(set(left_dx_names) & set(right_dx_names))

        self._left_tail_names = calib_left.calib_config["param_name"][
            calib_left.n_deltaX :
        ]
        self._right_tail_names = calib_right.calib_config["param_name"][
            calib_right.n_deltaX :
        ]
        self.n_left_tail = len(self._left_tail_names)
        self.n_right_tail = len(self._right_tail_names)

        self.param_name = (
            self.dx_names
            + [f"left_{n}" for n in self._left_tail_names]
            + [f"right_{n}" for n in self._right_tail_names]
        )

    # ------------------------------------------------------------------
    # var (union Delta X | left tail | right tail) <-> each chain's own
    # local layout (that chain's Delta X | that chain's own tail)
    # ------------------------------------------------------------------
    def _split_to_chain_vars(self, var: np.ndarray) -> "tuple[np.ndarray, np.ndarray]":
        n_dx = self.n_dx
        var_dx = var[:n_dx]
        var_left_tail = var[n_dx : n_dx + self.n_left_tail]
        var_right_tail = var[n_dx + self.n_left_tail :]

        dx_dict = dict(zip(self.dx_names, var_dx))
        var_dx_left = np.array([dx_dict[n] for n in self.left._fk_config["param_name"]])
        var_dx_right = np.array(
            [dx_dict[n] for n in self.right._fk_config["param_name"]]
        )
        var_left = np.concatenate([var_dx_left, var_left_tail])
        var_right = np.concatenate([var_dx_right, var_right_tail])
        return var_left, var_right

    def cost_function(self, var: np.ndarray) -> np.ndarray:
        var_left, var_right = self._split_to_chain_vars(var)

        n_gap_left = 3 * self.left._fk_config["NbSample"]
        n_gap_right = 3 * self.right._fk_config["NbSample"]
        # Slice off each chain's own internal regularization tail (sized
        # to that chain's local var, not this class's union var) -- one
        # combined regularization tail over the full union is added once
        # below instead, so shared torso parameters aren't regularized
        # twice.
        gap_left = self.left.cost_function(var_left)[:n_gap_left]
        gap_right = self.right.cost_function(var_right)[:n_gap_right]

        reg = np.sqrt(self.regularization_coefficient) * var
        return np.concatenate([gap_left, gap_right, reg])

    def solve_lm(self, **least_squares_kwargs):
        """Levenberg-Marquardt solve over the coupled parameter vector.
        See :meth:`TalosTableContactCalibration.solve_lm` for why this
        drives ``scipy.optimize.least_squares`` directly."""
        var0 = np.zeros(len(self.param_name))
        kwargs = dict(method="lm", xtol=1e-12, ftol=1e-12, max_nfev=4000)
        kwargs.update(least_squares_kwargs)
        result = least_squares(self.cost_function, var0, **kwargs)
        self.LM_result = result
        return result

    def split_params(self, var: np.ndarray) -> dict:
        n_dx = self.n_dx
        var_dx = var[:n_dx]
        var_left_tail = var[n_dx : n_dx + self.n_left_tail]
        var_right_tail = var[n_dx + self.n_left_tail :]
        return {
            "delta_x": dict(zip(self.dx_names, var_dx)),
            "shared_delta_x_names": list(self.shared_dx_names),
            "left": dict(zip(self._left_tail_names, var_left_tail)),
            "right": dict(zip(self._right_tail_names, var_right_tail)),
        }

    def gap_metrics(self, var: np.ndarray) -> dict:
        """Per-chain gap metrics (see
        :meth:`TalosTableContactCalibration.gap_metrics`), evaluated with
        the shared torso correction plugged into both chains."""
        var_left, var_right = self._split_to_chain_vars(var)
        return {
            "left": self.left.gap_metrics(var_left),
            "right": self.right.gap_metrics(var_right),
        }
