# Copyright [2021-2025] Thanh Nguyen
# Copyright [2022-2023] [CNRS, Toward SAS]
#
# Adapted from examples/ur10/utils/ur10_tools.py for the KITT1_5V3 right arm.

from __future__ import annotations

import os
import yaml
from yaml.loader import SafeLoader
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.optimize import least_squares
from typing import Any, Dict, List

from figaroh.calibration.calibration_tools import (
    load_data,
    calc_updated_fkm,
)

# Import base classes from figaroh
from figaroh.calibration.base_calibration import BaseCalibration
from figaroh.identification.base_identification import BaseIdentification
from figaroh.optimal.base_optimal_calibration import BaseOptimalCalibration
from figaroh.utils.results_manager import ResultsManager
from figaroh.utils.error_handling import (
    CalibrationError,
    IdentificationError,
    validate_robot_config,
    handle_calibration_errors,
)
from figaroh.identification.identification_tools import (
    get_param_from_yaml as get_identification_param_from_yaml,
    calculate_first_second_order_differentiation,
)
from figaroh.tools.regressor import (
    build_regressor_basic,
    get_index_eliminate,
    build_regressor_reduced,
)
from figaroh.tools.qrdecomposition import get_baseParams
from figaroh.identification.parameter import (
    get_standard_parameters,
    add_standard_additional_parameters,
    add_custom_parameters,
)
from figaroh.optimal.base_optimal_trajectory import (
    BaseOptimalTrajectory,
    BaseTrajectoryIPOPTProblem,
)

# Fallback for config manager and data processor if needed
try:
    from .data_processing import DataProcessor
except ImportError:
    print("DataProcessor module not found, using basic data loading methods.")


class KITT1_5Identification(BaseIdentification):
    """KITT1_5V3 right-arm-specific dynamic parameter identification class."""

    def __init__(
        self, robot: Any, config_file: str = "config/kitt_1_5_unified_config.yaml"
    ) -> None:
        """Initialize KITT1_5V3 right-arm identification with robot model and config.

        Args:
            robot: KITT1_5V3 right-arm robot model loaded with FIGAROH
            config_file: Path to KITT1_5V3 unified configuration YAML file
        """
        # Call parent constructor
        super().__init__(robot, config_file)
        print("KITT1_5V3 Right-Arm Dynamic Identification initialized")

    def load_trajectory_data(
        self, data_source: str = None
    ) -> Dict[str, np.ndarray]:
        """Load trajectory data from CSV files using DataProcessor.

        Args:
            data_source: Optional directory override. When given, the
                same filenames ("identification_q_simulation.csv" /
                "identification_tau_simulation.csv") are read from this
                directory instead of the default "data/" — e.g. to load
                a held-out validation set via
                ``identif_config["validation_data_file"]``.

        Returns:
            Dictionary with keys 'timestamps', 'positions',
            'velocities', 'accelerations', 'torques'
        """
        print("Loading KITT1_5V3 right-arm trajectory data...")
        data_dir = data_source or "data"

        try:
            # Use DataProcessor for improved data loading
            q_df = DataProcessor.load_csv_data(
                os.path.join(data_dir, "identification_q_simulation.csv")
            )
            tau_df = DataProcessor.load_csv_data(
                os.path.join(data_dir, "identification_tau_simulation.csv")
            )

            q_raw = q_df  # Convert to numpy array
            tau_raw = tau_df  # Convert to numpy array

            print(f"Loaded {len(q_raw)} samples from CSV files")

            # Sample time comes from the unified config so the derivative
            # step matches the simulation sampling rate (unlike UR10, which
            # hard-coded 100 Hz here while the config said 500 Hz).
            dt = float(self.identif_config["ts"])
            fs = 1.0 / dt

            # Zero-phase low-pass filtering of the noisy position
            # measurement (UR10 guarded on a non-existent method, so its
            # filtering branch never ran).
            filter_cfg = self.identif_config.get("filter_config", {})
            cutoff = float(
                filter_cfg.get(
                    "cutoff_frequency",
                    self.identif_config.get("cut_off_frequency_butterworth", 50.0),
                )
            )
            ftype = filter_cfg.get("filter_type", "butterworth")
            # The example's DataProcessor names the zero-phase low-pass
            # Butterworth filter "lowpass" (config keeps the more common
            # "butterworth" spelling).
            if ftype == "butterworth":
                ftype = "lowpass"
            q_raw = DataProcessor.apply_filter(
                q_raw,
                ftype,
                cutoff_freq=cutoff,
                sampling_freq=fs,
                order=int(filter_cfg.get("filter_params", {}).get("nbutter", 4)),
            )

            # Calculate derivatives using FIGAROH function (uses ts from config)
            q_filtered, dq_filtered, ddq_filtered = (
                calculate_first_second_order_differentiation(
                    self.model, q_raw, self.identif_config
                )
            )

            # Create time vector from the config sampling period
            time_vector = np.arange(len(q_filtered)) * dt

            print(f"Processed trajectory data: {len(q_filtered)} samples")

            return {
                "timestamps": time_vector.reshape(-1, 1),
                "positions": q_filtered,
                "velocities": dq_filtered,
                "accelerations": ddq_filtered,
                "torques": tau_raw[: len(q_filtered)],  # Match length
            }

        except Exception as e:
            raise IdentificationError(
                f"Failed to load KITT1_5V3 right-arm trajectory data: {e}"
            )


class KITT1_5OptimalTrajectoryIPOPT(BaseOptimalTrajectory):
    """
    KITT1_5V3 right-arm optimal trajectory generation using IPOPT.

    This class extends the BaseOptimalTrajectory to provide KITT1_5V3-specific
    configuration and problem setup.
    """

    def __init__(
        self,
        robot: Any,
        active_joints: List[str],
        config_file: str = "config/kitt_1_5_unified_config.yaml",
    ) -> None:
        """Initialize the KITT1_5V3 optimal trajectory generator."""
        super().__init__(robot, active_joints, config_file)
        self.logger.info("KITT1_5V3 OptimalTrajectoryIPOPT initialized")

    def create_ipopt_problem(
        self,
        n_joints: int,
        n_wps: int,
        Ns: int,
        tps: float,
        vel_wps: float,
        acc_wps: float,
        wp_init: np.ndarray,
        vel_wp_init: np.ndarray,
        acc_wp_init: np.ndarray,
        W_stack: np.ndarray,
    ) -> KITT1_5TrajectoryIPOPTProblem:
        """Create KITT1_5V3-specific IPOPT problem instance."""
        return KITT1_5TrajectoryIPOPTProblem(
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


class KITT1_5TrajectoryIPOPTProblem(BaseTrajectoryIPOPTProblem):
    """
    KITT1_5V3 right-arm IPOPT problem formulation for trajectory optimization.

    This class extends the BaseTrajectoryIPOPTProblem with KITT1_5V3-specific
    configurations and constraints.
    """

    def __init__(
        self,
        opt_traj: KITT1_5OptimalTrajectoryIPOPT,
        n_joints: int,
        n_wps: int,
        Ns: int,
        tps: float,
        vel_wps: float,
        acc_wps: float,
        wp_init: np.ndarray,
        vel_wp_init: np.ndarray,
        acc_wp_init: np.ndarray,
        W_stack: np.ndarray,
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
            "KITT1_5TrajectoryOptimization",
        )
