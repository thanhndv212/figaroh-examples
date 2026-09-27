# Copyright [2021-2026] Thanh Nguyen

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
SO-101 dynamic identification: gravity, friction and torque offsets from
servo current.

Data contract
-------------
Reads the directory format soarm_sdk's ``soarm-identify-record`` writes
(``soarm_sdk.dynamics.log``, format ``soarm_sdk.dynamics.log/v1``)::

    <run>/meta.json          joint names, rate, arm, calibration provenance
    <run>/q.csv              t, <joint>...  rad, URDF joint frame
    <run>/current_mA.csv     t, <joint>...  mA, signed in the URDF joint frame
    <run>/load_percent.csv   t, <joint>...  %, signed in the URDF joint frame

Columns are matched by header name, not position, so a log that also
carries the gripper (as the SDK's does) needs no editing.

What is identified
------------------
With ``custom.regressor.inertial_terms: false`` (the default), the model is
``tau = g(q) + fv*qd + fs*sign(qd) + offset``: the inertial columns are
built at zero velocity and acceleration, which leaves only the gravity
(mass and first-moment) columns alive. That is what this data can support —
the excitation is slow and STS3215 current is coarse — and it is what
gravity compensation needs.
"""

from __future__ import annotations

import json
import logging
import os
from typing import Any, Dict, List, Mapping, Optional

import numpy as np
import pandas as pd
import pinocchio as pin
import yaml

from figaroh.identification.base_identification import BaseIdentification
from figaroh.tools.regressor import build_regressor_basic
from figaroh.tools.robot import load_robot

logger = logging.getLogger(__name__)

LOG_FORMAT = "soarm_sdk.dynamics.log/v1"
IDENTIFIED_FORMAT = "soarm_sdk.dynamics.identified/v1"

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


def read_custom_config(config_file: str) -> Dict[str, Any]:
    """The ``custom:`` block of a unified config (not passed through by figaroh)."""
    with open(config_file) as f:
        return (yaml.safe_load(f) or {}).get("custom", {}) or {}


def read_log_meta(data_dir: str) -> Dict[str, Any]:
    with open(os.path.join(data_dir, "meta.json")) as f:
        meta = json.load(f)
    if meta.get("format") != LOG_FORMAT:
        raise ValueError(
            f"{data_dir} is not a {LOG_FORMAT} log (format={meta.get('format')!r})"
        )
    if meta.get("frame") != "urdf":
        raise ValueError(f"{data_dir}: joint angles must be in the URDF frame")
    return meta


def read_log_columns(data_dir: str, signal: str, joints: List[str]) -> np.ndarray:
    """``(N, len(joints))`` from ``<data_dir>/<signal>.csv``, by header name."""
    df = pd.read_csv(os.path.join(data_dir, f"{signal}.csv"))
    missing = [j for j in joints if j not in df.columns]
    if missing:
        raise ValueError(f"{signal}.csv in {data_dir} has no column(s) {missing}")
    return df[joints].to_numpy(dtype=float)


def resolve_locked_joints(
    custom: Mapping[str, Any], data_dir: Optional[str]
) -> Dict[str, float]:
    """Held angle per locked joint: from config, else the log's mean angle."""
    locked: Dict[str, float] = {}
    for name, value in (custom.get("locked_joints") or {}).items():
        if value is None:
            if data_dir is None:
                raise ValueError(
                    f"locked joint {name!r} has no angle in the config and no log "
                    "was given to take it from"
                )
            value = float(np.mean(read_log_columns(data_dir, "q", [name])))
        locked[name] = float(value)
    return locked


def load_so101_robot(urdf: str, locked_joints: Mapping[str, float]):
    """Load the SO-101 with *locked_joints* welded at the given angles.

    ``pin.buildReducedModel`` lumps each locked joint's subtree into its
    parent body, which is exactly the rigid body the identification sees
    when the jaw is held still.
    """
    robot = load_robot(urdf, package_dirs="../../models")
    model = robot.model
    q_ref = pin.neutral(model)
    ids = []
    for name, angle in locked_joints.items():
        jid = model.getJointId(name)
        if jid >= model.njoints:
            raise ValueError(f"no joint {name!r} in {urdf}")
        q_ref[model.joints[jid].idx_q] = angle
        ids.append(jid)
    if ids:
        reduced = pin.buildReducedModel(model, ids, q_ref)
        robot.model = reduced
        robot.data = reduced.createData()
        robot.q0 = pin.neutral(reduced)
        robot.v0 = np.zeros(reduced.nv)
        robot._backend = None
    robot.locked_joints = dict(locked_joints)
    return robot


class SO101Identification(BaseIdentification):
    """SO-101 identification from a soarm_sdk excitation log."""

    def __init__(
        self,
        robot,
        config_file: str = "config/so101_unified_config.yaml",
        data_dir: str = "data/simulated",
        signal: Optional[str] = None,
    ) -> None:
        super().__init__(robot, config_file)
        self.data_dir = data_dir
        self.custom = read_custom_config(config_file)
        sensing = self.custom.get("torque_sensing", {}) or {}
        self.signal = signal or sensing.get("signal", "current_mA")
        scales = sensing.get("nm_per_unit", {}) or {}
        if self.signal not in scales:
            raise ValueError(
                f"no custom.torque_sensing.nm_per_unit entry for signal {self.signal!r}"
            )
        self.nm_per_unit = float(scales[self.signal])
        regressor = self.custom.get("regressor", {}) or {}
        self.inertial_terms = bool(regressor.get("inertial_terms", False))
        self.qr_relative_tolerance = regressor.get("qr_relative_tolerance", 1e-3)

        self.active_joints = list(
            self.identif_config.get("active_joints") or ARM_JOINTS
        )
        self.identif_config["active_joints"] = self.active_joints
        act_Jid = [self.model.getJointId(n) for n in self.active_joints]
        act_J = [self.model.joints[jid] for jid in act_Jid]
        self.identif_config["act_Jid"] = act_Jid
        self.identif_config["act_J"] = act_J
        self.identif_config["act_idxq"] = [J.idx_q for J in act_J]
        self.identif_config["act_idxv"] = [J.idx_v for J in act_J]
        self.identif_config["idx_act_joints"] = [jid - 1 for jid in act_Jid]

        # Set explicitly: a unified config's signal_processing.filter_params
        # is empty by default, which would leave figaroh's own defaults
        # (f_sample=100 Hz) in force whatever the log's real rate is.
        self.filter_config = {
            "differentiation_method": "gradient",
            "filter_params": {
                "nbutter": 4,
                "f_butter": self.identif_config["cut_off_frequency_butterworth"],
                "med_fil": 5,
                "f_sample": 1.0 / self.identif_config["ts"],
            },
        }

    # -- data --------------------------------------------------------------

    def load_trajectory_data(self, data_source: str = None) -> Dict[str, Any]:
        data_dir = data_source or self.data_dir
        meta = read_log_meta(data_dir)
        self.log_meta = meta
        fs = 1.0 / self.identif_config["ts"]
        if abs(meta["rate_hz"] - fs) > 0.01 * fs:
            # Wrong here means wrong friction and inertias with no error, so
            # refuse rather than warn.
            raise ValueError(
                f"{data_dir} is sampled at {meta['rate_hz']:.2f} Hz but the config "
                f"says signal_processing.sampling_frequency: {fs:g}"
            )
        if meta.get("simulated") and not meta.get("synthetic_torque"):
            logger.warning(
                "%s is a soarm-identify-record --dry-run log: its currents are all "
                "zero, so nothing meaningful can be identified from it",
                data_dir,
            )
        q = read_log_columns(data_dir, "q", self.active_joints)
        tau = read_log_columns(data_dir, self.signal, self.active_joints)
        t = pd.read_csv(os.path.join(data_dir, "q.csv"))["t"].to_numpy(dtype=float)

        if self.signal == "current_mA":
            moving = np.ptp(q, axis=0) > 0.1
            one_sided = moving & ((tau.min(axis=0) >= 0) | (tau.max(axis=0) <= 0))
            if one_sided.any():
                logger.warning(
                    "current never changes sign on %s — this servo may report current "
                    "magnitude only. Try --signal load_percent.",
                    [j for j, s in zip(self.active_joints, one_sided) if s],
                )

        self.raw_data = {
            "timestamps": t.reshape(-1, 1),
            "positions": q,
            "velocities": None,  # differentiated from filtered positions
            "accelerations": None,
            "torques": tau * self.nm_per_unit,
        }
        return self.raw_data

    def process_torque_data(self, **kwargs) -> np.ndarray:
        """Low-pass the torque signal with the same filter as the positions."""
        tau = self._apply_filters(
            self.raw_data["torques"], **self.filter_config["filter_params"]
        )
        self.processed_data["torques"] = tau
        return tau

    def initialize_standard_parameters(self) -> None:
        """Standard (CAD) parameters, each read from its own joint's body.

        figaroh 0.4.8's ``get_standard_parameters`` pairs ``model.names[1:]``
        with ``model.inertias[i]`` from 0, so every joint is given the body
        *before* it (``inertias[0]`` is the universe/base). Base parameters
        come from the regressor and are unaffected, but the CAD prior used
        for the "nominal" validation torque and for reconstruction is
        shifted by one body. Rewrite the values from the right index;
        harmless once the library is fixed.
        """
        super().initialize_standard_parameters()
        keys = ("m", "mx", "my", "mz", "Ixx", "Ixy", "Iyy", "Ixz", "Iyz", "Izz")
        for idx, jname in enumerate(self.model.names[1:]):
            values = self.model.inertias[idx + 1].toDynamicParameters()
            for key, value in zip(keys, values):
                self.standard_parameter[f"{key}_{jname}"] = float(value)

    # -- solve -------------------------------------------------------------

    def solve(self, decimate=True, decimation_factor=10, **kwargs):
        """Solve, with the QR rank threshold scaled to this regressor.

        figaroh's default rank threshold is an absolute 1e-6. On this arm
        several gravity combinations are excited only through millimetre
        joint offsets out of the arm's plane: their columns are tiny but not
        zero, pass that threshold, and then soak up the noise (condition
        numbers ~1e8, masses in the thousands). A threshold relative to the
        largest column keeps exactly the well-excited combinations.
        """
        if self.qr_relative_tolerance:
            scale = np.linalg.norm(self.dynamic_regressor, axis=0).max()
            if decimate:
                scale /= np.sqrt(decimation_factor)  # fewer rows after decimation
            self.tol_qr = float(self.qr_relative_tolerance) * scale
        return super().solve(
            decimate=decimate, decimation_factor=decimation_factor, **kwargs
        )

    # -- regressor ---------------------------------------------------------

    def calculate_full_regressor(self) -> None:
        super().calculate_full_regressor()
        if self.inertial_terms:
            return
        nv = self.model.nv
        q = self.processed_data["positions"]
        zeros = np.zeros_like(self.processed_data["velocities"])
        # Inertial block at rest: only the gravity (m, mx, my, mz) columns
        # survive; the inertia-tensor columns are exactly zero and get
        # eliminated. Friction/offset columns keep the real velocity.
        W_static = build_regressor_basic(
            self.robot, q, zeros, zeros, self.identif_config
        )
        self.dynamic_regressor[:, : 10 * nv] = W_static[:, : 10 * nv]
        # Actuator inertia multiplies acceleration: out of this model.
        start = 12 * nv
        self.dynamic_regressor[:, start : start + nv] = 0.0

    def _compute_validation_metrics(self):
        """Validate against the same model that was fitted.

        figaroh rebuilds the validation regressor from the recorded motion
        directly, which would put acceleration back into the gravity
        columns. Evaluate it at zero acceleration instead; what remains is
        the Coriolis part, well under 1% of gravity at these speeds.
        """
        if self.inertial_terms:
            return super()._compute_validation_metrics()
        swapped = []
        for holder in ("_val_processed_data", "processed_data"):
            data = getattr(self, holder, None)
            if data is not None and data.get("accelerations") is not None:
                swapped.append((data, data["accelerations"]))
                data["accelerations"] = np.zeros_like(data["accelerations"])
        try:
            return super()._compute_validation_metrics()
        finally:
            for data, acc in swapped:
                data["accelerations"] = acc


# -- deployment ----------------------------------------------------------------


def identified_dynamics_dict(
    iden: SO101Identification,
    *,
    urdf: str,
    provenance: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """The ``soarm_sdk.dynamics.identified/v1`` document for this result.

    Per-body masses and first moments come from the reconstructed standard
    parameters (``tasks.identification.reconstruction``), friction and
    offsets straight from the base parameters they appear in unchanged.
    """
    recon = (iden.result or {}).get("reconstruction") or {}
    theta = recon.get("theta_r_dict")
    if recon.get("status") not in ("ok", "optimal", "success") and not theta:
        raise RuntimeError(
            "no reconstructed standard parameters; enable "
            "tasks.identification.reconstruction in the config "
            f"(reconstruction={recon.get('status', 'missing')!r})"
        )
    # Unidentifiable combinations fall back to the CAD prior.
    std = dict(iden.standard_parameter)
    std.update(theta)
    base = dict(zip(iden.params_base, np.asarray(iden.phi_base, dtype=float)))

    bodies = {
        j: {k: float(std[f"{k}_{j}"]) for k in ("m", "mx", "my", "mz")}
        for j in iden.active_joints
    }

    def additional(prefix: str) -> Dict[str, float]:
        out = {}
        for j in iden.active_joints:
            key = f"{prefix}_{j}"
            # A friction/offset column is independent of every other, so it is
            # its own base parameter; fall back to the reconstruction if not.
            out[j] = float(base.get(key, std.get(key, 0.0)))
        return out

    fv, fs, off = additional("fv"), additional("fs"), additional("off")
    doc = {
        "format": IDENTIFIED_FORMAT,
        "robot": "so101",
        "urdf": os.path.basename(urdf),
        "joint_names": list(iden.active_joints),
        "locked_joints": sorted(getattr(iden.robot, "locked_joints", {})),
        "held_positions": dict(getattr(iden.robot, "locked_joints", {})),
        "signal": iden.signal,
        "nm_per_unit": iden.nm_per_unit,
        "bodies": bodies,
        "friction": {j: {"fv": fv[j], "fs": fs[j]} for j in iden.active_joints},
        "offset": off,
        "quality": {
            "condition_number": float(iden.result["condition number"]),
            "correlation": float(iden.correlation),
            "rmse": float(iden.rms_error),
            "base_parameters": len(iden.params_base),
        },
        "provenance": provenance or {},
    }
    return doc
