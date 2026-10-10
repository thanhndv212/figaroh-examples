# Copyright [2021-2025] Thanh Nguyen
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
Example refactored TIAGo tools using the new base classes.
This demonstrates how the existing TIAGo implementation would be refactored
to use the generalized base classes and new infrastructure.
"""

from __future__ import annotations

import logging
import os
from os.path import abspath
from typing import Any, List, Optional

import numpy as np
import numpy.typing as npt
import pandas as pd

# Import FIGAROH modules
from figaroh.calibration.calibration_tools import calc_updated_fkm

# Import shared modules using figaroh library
from figaroh.calibration.base_calibration import BaseCalibration
from figaroh.identification.base_identification import BaseIdentification
from figaroh.optimal.base_optimal_calibration import BaseOptimalCalibration
from figaroh.optimal.base_optimal_trajectory import (
    BaseOptimalTrajectory,
    BaseTrajectoryIPOPTProblem,
)
from figaroh.utils.error_handling import handle_calibration_errors

logger = logging.getLogger(__name__)


def estimate_velocity_lag(
    timestamps: npt.NDArray[np.float64],
    positions: npt.NDArray[np.float64],
    velocities: npt.NDArray[np.float64],
    max_lag: int = 50,
) -> int:
    """Delay of a measured velocity channel relative to d(position)/dt.

    Returns the non-negative sample shift ``L`` minimising, summed over
    joints, ``||d q/dt [n] - v[n + L]|| / ||d q/dt||``, where the derivative
    uses the recorded timestamps. ``L = 0`` means the channels are aligned.
    """
    deriv = np.gradient(positions, timestamps, axis=0)
    norms = np.linalg.norm(deriv, axis=0)
    norms[norms == 0] = 1.0
    errors = []
    for lag in range(max_lag + 1):
        n = len(timestamps) - lag
        diff = deriv[:n] - velocities[lag : lag + n]
        errors.append(float(np.sum(np.linalg.norm(diff, axis=0) / norms)))
    lag = int(np.argmin(errors))
    if lag == max_lag:
        raise ValueError(
            f"Velocity lag estimate hit the search limit ({max_lag} samples)"
        )
    return lag


#: Coefficient of the first-order filter in the logged TIAGo velocity:
#: ``v[n] = a·v[n−1] + (1−a)·Δq/Δt`` on the recorded clock (time constant
#: 0.195 s at 100 Hz, #68).
VELOCITY_FILTER_COEFFICIENT = 0.95


def velocity_filter_residual(
    timestamps: npt.NDArray[np.float64],
    positions: npt.NDArray[np.float64],
    velocities: npt.NDArray[np.float64],
    a: float = VELOCITY_FILTER_COEFFICIENT,
) -> float:
    """Worst per-joint relative residual of the logged velocity's filter model.

    Compares ``v[n]`` with ``a·v[n−1] + (1−a)·(q[n]−q[n−1])/(t[n]−t[n−1])``.
    About 1e-4 on the 2021-07 recordings (#68): the channel is a filtered
    position difference and carries nothing the positions do not.
    """
    step = np.diff(positions, axis=0) / np.diff(timestamps)[:, None]
    predicted = a * velocities[:-1] + (1 - a) * step
    norms = np.linalg.norm(velocities[1:], axis=0)
    norms[norms == 0] = 1.0
    return float(np.max(np.linalg.norm(velocities[1:] - predicted, axis=0) / norms))


def duplicate_channel_fractions(
    channels: npt.NDArray[np.float64], names: List[str], threshold: float = 0.5
) -> dict[str, float]:
    """Pairs of channels that repeat each other's non-zero samples.

    For each pair, the fraction of samples where at least one channel is
    non-zero and both are equal. Shared zeros are ignored: quantised,
    mostly-zero channels would otherwise look like copies of each other.
    """
    found = {}
    for i in range(channels.shape[1]):
        for j in range(i + 1, channels.shape[1]):
            a, b = channels[:, i], channels[:, j]
            active = (a != 0) | (b != 0)
            if not active.any():
                continue
            fraction = float(np.mean(a[active] == b[active]))
            if fraction > threshold:
                found[f"{names[i]}/{names[j]}"] = fraction
    return found


def zero_fractions(
    channels: npt.NDArray[np.float64], names: List[str]
) -> dict[str, float]:
    """Fraction of exactly-zero samples per channel."""
    return {n: float(np.mean(channels[:, k] == 0)) for k, n in enumerate(names)}


class TiagoCalibration(BaseCalibration):
    """
    Class for calibrating the TIAGo robot.

    This class provides TIAGo-specific calibration functionality by extending
    the BaseCalibration class with robot-specific cost functions and
    initialization parameters.
    """

    @handle_calibration_errors
    def __init__(
        self,
        robot: Any,
        config_file: str = "config/tiago_config.yaml",
        del_list: Optional[List[Any]] = None,
    ) -> None:
        """Initialize TIAGo calibration with robot model and configuration.

        Args:
            robot: TIAGo robot model loaded with FIGAROH
            config_file: Path to TIAGo configuration YAML file
            del_list: List of sample indices to exclude from calibration
        """
        if del_list is None:
            del_list = []
        super().__init__(robot, config_file, del_list)
        print("TIAGo calibration initialized with new infrastructure")

    def cost_function(self, var: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        """
        TIAGo-specific cost function for the optimization problem.

        SE3 log-map residuals of the measured tool point. No regularisation
        rows: priors on the joint parameters come from core's
        ``estimation.method: map`` (figaroh-plus#120).

        Args:
            var: Parameter vector to evaluate

        Returns:
            Residual vector
        """
        PEEe = calc_updated_fkm(
            self.model, self.data, var, self.q_measured, self.calib_config
        )

        # Main residual: SE3 log map (geometrically correct pose error)
        return self._compute_logmap_residuals(self.PEE_measured, PEEe)


class TiagoIdentification(BaseIdentification):
    """TIAGo-specific dynamic parameter identification class."""

    def __init__(
        self,
        robot: Any,
        config_file: str = "config/tiago_config.yaml",
    ) -> None:
        """Initialize TIAGo identification with robot model and configuration.

        Args:
            robot: TIAGo robot model loaded with FIGAROH
            config_file: Path to TIAGo configuration YAML file
        """
        super().__init__(robot, config_file)
        print("TiagoIdentification initialized for TIAGo robot")

    #: Where the joint velocity comes from. ``"positions"`` leaves ``dq``
    #: absent, so the pipeline differentiates the filtered positions: the
    #: logged channel is a 0.195 s first-order filter of the position
    #: difference (#68). ``"measured"`` uses the logged channel shifted by
    #: :attr:`velocity_lag` (the #20 correction), to reproduce earlier runs.
    velocity_source: str = "positions"
    #: With ``velocity_source="measured"``, how to align the logged velocity:
    #: ``"auto"`` estimates the delay per run (see :func:`estimate_velocity_lag`),
    #: an ``int`` applies that many samples, ``0`` disables the shift.
    velocity_lag: int | str = "auto"
    #: Search range for the automatic lag estimate, in samples.
    max_velocity_lag: int = 50

    def load_trajectory_data(self, data_source: str = None) -> Any:
        """Load the TIAGo position/velocity/effort CSVs (D2-audited, #20).

        The three files share one recorded clock (column ``t``, ~100 Hz).
        The configured filter sample rate must match that clock. By default
        (:attr:`velocity_source` ``"positions"``) the velocity is derived
        from the filtered positions; the logged channel is only checked
        against its filter model. With ``"measured"`` the logged channel is
        shifted earlier by :attr:`velocity_lag` samples and the last samples
        of the other channels are dropped (no padding).

        Returns a :class:`~figaroh.data.trajectory.TrajectoryData`
        (figaroh-plus#55, examples#17): joint order, recorded clock, signal
        origins, source files with sha256 and source rows travel with the
        arrays. The recorded effort (``motor_effort``, raw units, unverified
        per the D2 audit) is converted here with ``reduction_ratio × kmotor``
        (+ ``9.81 × subtree mass`` on the torso) and kept beside the joint
        effort: ``joint_force`` in N on the prismatic torso, ``joint_torque``
        in N·m on the arm. Inspect ``trajectory_provenance`` for the clock,
        lag and data checks.

        Args:
            data_source: Optional directory override. When given, the
                same basenames configured in ``pos_data``/``vel_data``/
                ``torque_data`` are read from this directory instead of
                their configured parent directory — e.g. to load a
                held-out validation set via
                ``identif_config["validation_data_file"]``.
        """
        pos_path = abspath(self.identif_config["pos_data"])
        vel_path = abspath(self.identif_config["vel_data"])
        torque_path = abspath(self.identif_config["torque_data"])
        if data_source:
            pos_path = os.path.join(data_source, os.path.basename(pos_path))
            vel_path = os.path.join(data_source, os.path.basename(vel_path))
            torque_path = os.path.join(data_source, os.path.basename(torque_path))

        frames = {
            "position": pd.read_csv(pos_path),
            "velocity": pd.read_csv(vel_path),
            "effort": pd.read_csv(torque_path),
        }
        joints = self.identif_config["active_joints"]
        arrays, columns = {}, {}
        for kind, df in frames.items():
            # Exact channel names in model order (no substring matching).
            columns[kind] = [f"- {jn}_{kind}" for jn in joints]
            missing = [c for c in columns[kind] + ["t"] if c not in df.columns]
            if missing:
                raise ValueError(f"TIAGo {kind} CSV is missing columns {missing}")
            arrays[kind] = df[columns[kind]].to_numpy(float)
            if not np.all(np.isfinite(arrays[kind])):
                raise ValueError(f"TIAGo {kind} CSV contains non-finite values")

        ts = frames["position"]["t"].to_numpy(float)
        for kind in ("velocity", "effort"):
            if not np.array_equal(ts, frames[kind]["t"].to_numpy(float)):
                raise ValueError(f"TIAGo {kind} timestamps differ from positions")
        dt = np.diff(ts)
        if len(ts) < 3 or not np.all(dt > 0):
            raise ValueError("TIAGo timestamps must be strictly increasing")
        recorded_rate = 1.0 / float(np.median(dt))

        filter_rate = float(self.filter_config["filter_params"]["f_sample"])
        if abs(filter_rate - recorded_rate) > 0.05 * recorded_rate:
            raise ValueError(
                f"Filter sample rate {filter_rate:g} Hz does not match the "
                f"recorded clock ({recorded_rate:.2f} Hz): the Butterworth "
                "cutoff would be scaled by their ratio"
            )

        q, dq, tau = arrays["position"], arrays["velocity"], arrays["effort"]
        filter_residual = velocity_filter_residual(ts, q, dq)
        if self.velocity_source == "positions":
            lag, dq = 0, None
        elif self.velocity_source == "measured":
            if self.velocity_lag == "auto":
                lag = estimate_velocity_lag(ts, q, dq, self.max_velocity_lag)
            else:
                lag = int(self.velocity_lag)
            if not 0 <= lag < len(ts) - 2:
                raise ValueError(f"Invalid velocity lag {lag}")
            n = len(ts) - lag
            dq = dq[lag:]
            ts, q, tau = ts[:n], q[:n], tau[:n]
        else:
            raise ValueError(
                f"velocity_source must be 'positions' or 'measured', "
                f"not {self.velocity_source!r}"
            )

        duplicates = duplicate_channel_fractions(arrays["effort"], joints)
        zeros = zero_fractions(arrays["effort"], joints)

        if not hasattr(self, "trajectory_provenance"):
            self.trajectory_provenance = {}
        self.trajectory_provenance[data_source or "training"] = {
            "position_file": pos_path,
            "velocity_file": vel_path,
            "effort_file": torque_path,
            "timing_source": "recorded",
            "recorded_rate_hz": recorded_rate,
            "filter_sample_rate_hz": filter_rate,
            "source_rows": len(frames["position"]),
            "velocity_source": self.velocity_source,
            "velocity_filter_residual": filter_residual,
            "velocity_lag_samples": lag,
            "velocity_lag_s": lag / recorded_rate,
            "velocity_lag_mode": self.velocity_lag,
            "dropped_trailing_rows": lag,
            "effort_units": "raw (converted in process_torque_data)",
            "duplicate_effort_channels": duplicates,
            "effort_zero_fraction": zeros,
        }
        for name, fraction in zeros.items():
            if fraction > 0.5:
                logger.warning(
                    "TIAGo %s effort is exactly zero on %.0f%% of samples; its "
                    "dynamics are weakly observable from this recording (see #20)",
                    name,
                    100 * fraction,
                )
        for pair, fraction in duplicates.items():
            logger.warning(
                "TIAGo effort channels %s are equal on %.0f%% of their non-zero "
                "samples; the recording may couple these joints' efforts (see #20)",
                pair,
                100 * fraction,
            )

        return self._trajectory(
            ts, q, dq, tau, lag, (pos_path, vel_path, torque_path), data_source
        )

    def _trajectory(self, ts, q, dq, effort, lag, files, data_source):
        """The data contract form of a loaded recording (examples#17)."""
        import pinocchio as pin
        from figaroh.data import (
            JOINT_FORCE,
            JOINT_TORQUE,
            DataSource,
            Session,
            TrajectoryData,
        )

        joints = list(self.identif_config["active_joints"])
        ratio = self.identif_config["reduction_ratio"]
        kmotor = self.identif_config["kmotor"]
        pin.computeSubtreeMasses(self.robot.model, self.robot.data)
        model = self.robot.model
        scale = [ratio[j] * kmotor[j] for j in joints]
        offset = [
            (
                9.81 * self.robot.data.mass[model.getJointId(j)]
                if j == "torso_lift_joint"
                else 0.0
            )
            for j in joints
        ]
        prismatic = [
            model.joints[model.getJointId(j)].shortname().startswith("JointModelP")
            for j in joints
        ]
        recorded = TrajectoryData(
            t=ts,
            joint_names=joints,
            q=q,
            dq=dq,
            effort=effort,
            effort_kind="motor_effort",
            effort_unit="raw (unverified, D2 audit)",
            clock="recorded",
            origin={
                "q": "measured",
                "dq": (
                    "absent; derived from the filtered positions (#68)"
                    if dq is None
                    else f"measured; shifted {lag} samples earlier (velocity lag)"
                ),
            },
            # rows of the files; the last `lag` rows are dropped
            sample_index=np.arange(len(ts)),
            source=DataSource.from_files(
                files,
                adapter="examples.tiago.TiagoIdentification",
                session=Session(id=os.path.basename(data_source or "training")),
                notes=(
                    "velocity derived from positions"
                    if dq is None
                    else f"velocity shifted {lag} samples earlier; "
                    f"last {lag} rows dropped"
                ),
            ),
        )
        return recorded.converted(
            scale,
            offset,
            effort_kind=[JOINT_FORCE if p else JOINT_TORQUE for p in prismatic],
            effort_unit=["N" if p else "N·m" for p in prismatic],
            description="× reduction_ratio × kmotor; + 9.81 × subtree mass (torso)",
        )


class TiagoOptimalCalibration(BaseOptimalCalibration):
    """TIAGo-specific optimal configuration generation for calibration."""

    def __init__(
        self,
        robot: Any,
        config_file: str = "config/tiago_config.yaml",
    ) -> None:
        """Initialize TIAGo optimal calibration."""
        super().__init__(robot, config_file)
        print("TIAGo Optimal Calibration initialized")


class OptimalTrajectoryIPOPT(BaseOptimalTrajectory):
    """
    TIAGo-specific optimal trajectory generation using IPOPT.

    This class extends the BaseOptimalTrajectory to provide TIAGo-specific
    configuration and problem setup.
    """

    def __init__(
        self,
        robot: Any,
        active_joints: List[str],
        config_file: str = "config/tiago_config.yaml",
    ) -> None:
        """Initialize the TIAGo optimal trajectory generator."""
        super().__init__(robot, active_joints, config_file)
        self.logger.info("TIAGo OptimalTrajectoryIPOPT initialized")

    def create_ipopt_problem(
        self,
        n_joints: int,
        n_wps: int,
        Ns: int,
        tps: float,
        vel_wps: npt.NDArray[np.float64],
        acc_wps: npt.NDArray[np.float64],
        wp_init: npt.NDArray[np.float64],
        vel_wp_init: npt.NDArray[np.float64],
        acc_wp_init: npt.NDArray[np.float64],
        W_stack: npt.NDArray[np.float64],
    ) -> TiagoTrajectoryIPOPTProblem:
        """Create TIAGo-specific IPOPT problem instance."""
        return TiagoTrajectoryIPOPTProblem(
            self,
            n_joints,
            n_wps,
            Ns,
            tps,
            vel_wps,
            acc_wps,
            wp_init,
            vel_wp_init,
            acc_wp_init,
            W_stack,
        )


class TiagoTrajectoryIPOPTProblem(BaseTrajectoryIPOPTProblem):
    """
    TIAGo-specific IPOPT problem formulation for trajectory optimization.

    This class extends the BaseTrajectoryIPOPTProblem with TIAGo-specific
    configurations and constraints.
    """

    def __init__(
        self,
        opt_traj: OptimalTrajectoryIPOPT,
        n_joints: int,
        n_wps: int,
        Ns: int,
        tps: float,
        vel_wps: npt.NDArray[np.float64],
        acc_wps: npt.NDArray[np.float64],
        wp_init: npt.NDArray[np.float64],
        vel_wp_init: npt.NDArray[np.float64],
        acc_wp_init: npt.NDArray[np.float64],
        W_stack: npt.NDArray[np.float64],
    ) -> None:
        super().__init__(
            opt_traj,
            n_joints,
            n_wps,
            Ns,
            tps,
            vel_wps,
            acc_wps,
            wp_init,
            vel_wp_init,
            acc_wp_init,
            W_stack,
            "TiagoTrajectoryOptimization",
        )
