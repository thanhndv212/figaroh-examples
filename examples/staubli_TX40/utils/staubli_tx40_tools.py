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
Staubli TX40 robot tools using the new base classes.
This demonstrates how the Staubli TX40 implementation is refactored
to use the generalized base classes.
"""

import numpy as np
import pandas as pd
from scipy import signal
import sys
import os

# Import from shared directory
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'shared'))

from base_identification import BaseIdentification
from figaroh.tools.regressor import build_regressor_basic
from figaroh.identification.identification_tools import (
    get_standard_parameters
)


class TX40Identification(BaseIdentification):
    """Staubli TX40-specific dynamic parameter identification class."""
    
    def __init__(self, robot, config_file="config/TX40_config.yaml"):
        """Initialize TX40 identification with robot model and configuration.
        
        Args:
            robot: Staubli TX40 robot model loaded with FIGAROH
            config_file: Path to TX40 configuration YAML file
        """
        super().__init__(robot, config_file)
        
        # TX40-specific active joints
        self.active_joints = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
        ]
        
        # Set up active joint indices
        self.identif_config["active_joints"] = self.active_joints
        
        # Calculate joint IDs and indices properly
        act_Jid = [self.robot.model.getJointId(i) for i in self.active_joints]
        act_J = [self.robot.model.joints[jid] for jid in act_Jid]
        act_idxq = [J.idx_q for J in act_J]
        act_idxv = [J.idx_v for J in act_J]
        
        # Store in identif_config for base class compatibility
        self.identif_config["act_Jid"] = act_Jid
        self.identif_config["act_J"] = act_J
        self.identif_config["act_idxq"] = act_idxq
        self.identif_config["act_idxv"] = act_idxv
        
        # Legacy compatibility
        idx_act_joints = [jid - 1 for jid in act_Jid]
        self.identif_config["idx_act_joints"] = idx_act_joints

    def solve(self, decimate=True, decimation_factor=10, zero_tolerance=0.001,
              plotting=True, save_params=False, wls=False):
        """Solve TX40 identification with optional weighted least squares.
        
        Args:
            decimate (bool): Whether to apply TX40-specific decimation
            decimation_factor (int): Factor (not used for TX40 custom)
            zero_tolerance (float): Tolerance for eliminating zero columns
            plotting (bool): Whether to generate plots
            save_params (bool): Whether to save parameters to file
            wls (bool): Whether to use weighted least squares
            
        Returns:
            ndarray: Base parameters phi_base
        """
        # Call parent solve method
        phi_base = super().solve(
            decimate=decimate,
            decimation_factor=decimation_factor,
            zero_tolerance=zero_tolerance,
            plotting=False,  # Handle plotting separately
            save_params=False  # Handle saving separately
        )
        
        # Apply weighted least squares if requested
        if wls and hasattr(self, 'dynamic_regressor_base'):
            phi_base = self._apply_weighted_least_squares()
        
        # TX40-specific plotting
        if plotting:
            self._plot_identification_results()
        
        # TX40-specific parameter saving
        if save_params:
            self._save_tx40_parameters()
        
        return phi_base
    
    def load_trajectory_data(self):
        """Load and process CSV data for Staubli TX40 robot."""
        # Load current (torque) and position data
        curr_data = pd.read_csv("data/curr_data.csv").to_numpy()
        pos_data = pd.read_csv("data/pos_read_data.csv").to_numpy()
        
        # validate sizes of loaded data
        if curr_data.shape != pos_data.shape :
            raise ValueError(
                f"Data size mismatch: curr_data has {curr_data.shape} samples, "
                f"pos_data has {pos_data.shape} samples"
            )
        # Create timestamp array based on sampling time
        n_samples = pos_data.shape[0]
        timestamps = np.linspace(
            0, n_samples * self.identif_config["ts"], n_samples
        ).reshape(-1, 1)
        
        # Store raw motor encoder positions and current data
        self.raw_data = {
            "timestamps": timestamps,
            "positions": pos_data,  # Motor encoder positions
            "velocities": None,     # Will be computed from positions
            "accelerations": None,  # Will be computed from velocities
            "torques": curr_data    # Motor currents
        }
        return self.raw_data

    def process_kinematics_data(self, filter_config=None):
        """Apply TX40-specific filtering to data."""
        # TX40-specific filter parameters
        default_config = {
            'differentiation_method': 'gradient',
            'filter_params': {
                'nbutter': 4,
                'f_butter': self.identif_config.get(
                    "cut_off_frequency_butterworth", 100.0
                ),
                'f_sample': 1 / self.identif_config["ts"]
            }
        }
        filter_config = {**default_config, **(filter_config or {})}

        # Convert motor positions to joint positions
        self._convert_motor_to_joint_positions()

        # Apply filtering to kinematics data
        self._filter_kinematics_data(filter_config)

    def _convert_motor_to_joint_positions(self):
        """Convert motor encoder positions to joint positions with
        TX40-specific coupling and calibration offsets.
        
        This method handles:
        - Reduction ratio matrix creation with coupling terms
        - Motor-to-joint position conversion
        - Joint-specific offset calibrations
        
        Side Effects:
            Updates self.raw_data["positions"] with converted joint positions
        """
        # Get reduction ratios from configuration
        reduction_ratios = self.identif_config["reduction_ratio"]
        
        # Create reduction ratio matrix for positions
        reduction_matrix = np.diag([
            reduction_ratios[0], reduction_ratios[1], reduction_ratios[2],
            reduction_ratios[3], reduction_ratios[4], reduction_ratios[5]
        ])
        
        # Add TX40-specific coupling between joints 5 and 4 for positions
        reduction_matrix[5, 4] = reduction_ratios[5]  # Coupling term

        # Convert motor positions to joint positions
        joint_positions = np.dot(np.linalg.inv(reduction_matrix), self.raw_data["positions"].T)
        
        # Apply TX40-specific joint offsets
        joint_positions[:, 1] += -np.pi / 2  # Joint 2 offset
        joint_positions[:, 2] += np.pi / 2   # Joint 3 offset
        
        # Update raw data with converted positions
        self.raw_data["positions"] = joint_positions

    def process_torque_data(self, tau, filter_config=None):
        """Process torque data with TX40-specific motor torque conversion."""
        # TX40-specific filter parameters
        default_config = {
            'differentiation_method': 'gradient',
            'filter_params': {
                'nbutter': 4,
                'f_butter': self.identif_config.get(
                    "cut_off_frequency_butterworth", 100.0
                ),
                'f_sample': 1 / self.identif_config["ts"]
            }
        }
        filter_config = {**default_config, **(filter_config or {})}
        # Get reduction ratios from config
        reduction_ratios = self.identif_config["reduction_ratio"]
        
        # Create reduction ratio matrix for torques
        # Note: TX40 has coupling between joints 4 and 5
        red_tau = np.diag([
            reduction_ratios[0], reduction_ratios[1], reduction_ratios[2],
            reduction_ratios[3], reduction_ratios[4], reduction_ratios[5]
        ])
        red_tau[4, 5] = reduction_ratios[5]  # Coupling term
        
        # Convert motor torques to joint torques
        tau_T = np.dot(red_tau, tau.T)
        tau_processed = tau_T.T

        # Remove border effects
        self.processed_data["torques"] = self._remove_border_effects(
            tau_processed, filter_config['filter_params']
        )

        # Synchronize torque data with kinematic data dimensions
        self._sync_torque_w_kinematics()

        return self.processed_data["torques"]

    def _remove_border_effects(self, tau_processed, filter_params):
        """This function removes border effects from torque data.
        
        Args:
            tau_processed: Processed torque data array
            filter_params: Dictionary containing filter parameters
            
        Returns:
            ndarray: Torque data synchronized with kinematic data
        """
        # Remove both ends to prevent border effect after differentiation
        nbutter = filter_params.get('nbutter', 4)
        nbord = 5 * nbutter
        tau_processed = np.delete(tau_processed, np.s_[0:nbord], axis=0)
        tau_processed = np.delete(tau_processed, np.s_[-nbord:], axis=0)
        return tau_processed

    def _sync_torque_w_kinematics(self):
        """Synchronize torque data with kinematic data."""
        # Ensure torque data matches kinematic data length exactly
        current_kinematic_length = self.processed_data["positions"].shape[0]
        if self.processed_data["torques"].shape[0] != current_kinematic_length:
            # Trim both datasets to match the shorter length
            min_length = min(self.processed_data["torques"].shape[0], current_kinematic_length)
            self.processed_data["torques"] = self.processed_data["torques"][:min_length, :]

            # Also trim kinematic data to match
            kinematic_keys = ["timestamps", "positions", "velocities",
                              "accelerations"]
            for key in kinematic_keys:
                if key in self.processed_data:
                    self.processed_data[key] = self.processed_data[key][
                        :min_length, :
                    ]

    def add_additional_parameters(self):
        """Add additional parameters specific to TX40 and recalculate dynamic regressor."""
        # Add coupling parameters if present
        if self.identif_config.get("has_coupled_wrist", False):
            self.standard_parameter["Iam6"] = float(
                self.identif_config["Iam6"]
            )
            self.standard_parameter["fvm6"] = float(
                self.identif_config["fvm6"]
            )
            self.standard_parameter["fsm6"] = float(
                self.identif_config["fsm6"]
            )
            for key, value in self.standard_parameter.items():
                if isinstance(value, str):
                    self.standard_parameter[key] = float(value)
            self.dynamic_regressor = self._add_coupling(
                self.dynamic_regressor,
                self.model,
                self.data,
                self.num_samples,
                self.model.nq,
                self.model.nv,
                self.model.njoints,
                self.processed_data["positions"],
                self.processed_data["velocities"],
                self.processed_data["accelerations"]
            )
    def _add_coupling(self, W, model, data, N, nq, nv, njoints, q, v, a):
        """Add TX40-specific wrist coupling to regressor matrix.
        
        This method adds coupling terms for the TX40 robot's wrist joints
        (joints 4, 5, and 6) to account for mechanical coupling effects.
        
        Args:
            W: Regressor matrix
            model: Robot model
            data: Robot data
            N: Number of samples
            nq: Number of position variables
            nv: Number of velocity variables
            njoints: Number of joints
            q: Joint positions
            v: Joint velocities
            a: Joint accelerations
            
        Returns:
            ndarray: Regressor matrix with coupling terms added
        """
        W = np.c_[W, np.zeros([W.shape[0], 3])]
        for i in range(N):
            # joint 5
            W[4 * N + i, W.shape[1] - 3] = a[i, 5]
            W[4 * N + i, W.shape[1] - 2] = v[i, 5]
            W[4 * N + i, W.shape[1] - 1] = np.sign(v[i, 4] + v[i, 5])
            # joint 6
            W[5 * N + i, W.shape[1] - 3] = a[i, 4]
            W[5 * N + i, W.shape[1] - 2] = v[i, 4]
            W[5 * N + i, W.shape[1] - 1] = np.sign(v[i, 4] + v[i, 5])

        return W

    def _apply_weighted_least_squares(self):
        """Compute joint-wise variances and apply weighted least squares.
        
        Returns:
            ndarray: Weighted least squares parameter estimates
        """
        # Get data for WLS (use the processed data from base class)
        W_b = self.dynamic_regressor_base
        tau = self.tau_noised  # This contains the processed/decimated data
        nv = self.model.nv
        
        # Calculate joint variances efficiently
        sig_ro_joint, diag_SIGMA = self._calculate_joint_variances(
            W_b, tau, nv
        )
        
        # Store joint variances for analysis
        self._joint_variances = sig_ro_joint
        
        # Apply WLS solution
        return self._solve_weighted_least_squares(W_b, tau, diag_SIGMA)
    
    def _calculate_joint_variances(self, W_b, tau, nv):
        """Calculate joint-wise variances efficiently.
        
        Args:
            W_b: Base regressor matrix
            tau: Torque vector (flattened)
            nv: Number of velocity variables (joints)
            
        Returns:
            tuple: (joint_variances, diagonal_covariance)
        """
        row_size = tau.shape[0]
        
        # Check if data dimensions are compatible
        if W_b.shape[0] != row_size:
            raise ValueError(
                f"Dimension mismatch: regressor has {W_b.shape[0]} rows, "
                f"but torque has {row_size} elements"
            )
        
        samples_per_joint = row_size // nv
        
        # Pre-allocate arrays
        sig_ro_joint = np.zeros(nv)
        diag_SIGMA = np.zeros(row_size)
        
        # Vectorized prediction for all joints at once
        tau_pred_full = W_b @ self.phi_base
        residuals = tau - tau_pred_full
        
        # Calculate variances for each joint using array slicing
        for i in range(nv):
            start_idx = i * samples_per_joint
            end_idx = (i + 1) * samples_per_joint
            
            # Handle case where samples don't divide evenly
            if end_idx > row_size:
                end_idx = row_size
            
            # Extract residuals for joint i
            residuals_joint = residuals[start_idx:end_idx]
            
            # Calculate variance (mean squared error)
            if len(residuals_joint) > 0:
                sig_ro_joint[i] = np.mean(residuals_joint ** 2)
            else:
                sig_ro_joint[i] = 1e-6  # Small default variance
            
            # Fill diagonal covariance matrix
            diag_SIGMA[start_idx:end_idx] = sig_ro_joint[i]
        
        return sig_ro_joint, diag_SIGMA
    
    def _solve_weighted_least_squares(self, W_b, tau, diag_SIGMA):
        """Solve the weighted least squares problem.
        
        Args:
            W_b: Base regressor matrix
            tau: Torque vector
            diag_SIGMA: Diagonal covariance matrix values
            
        Returns:
            ndarray: WLS parameter estimates
        """
        # Build inverse covariance matrix (weights)
        # For efficiency, use 1/diag_SIGMA as weights instead of matrix inverse
        weights = 1.0 / (diag_SIGMA + 1e-12)  # Add small epsilon for stability
        W_sqrt = np.sqrt(weights)[:, np.newaxis]
        
        # Apply weights to regressor and torque
        W_weighted = W_b * W_sqrt
        tau_weighted = tau * W_sqrt.ravel()
        
        # Solve normal equations: (W^T W)^-1 W^T tau
        WTW = W_weighted.T @ W_weighted
        WTtau = W_weighted.T @ tau_weighted
        
        # Compute covariance matrix for standard deviations
        C_X = np.linalg.inv(WTW)
        phi_wls = C_X @ WTtau
        
        # Update results
        self.phi_base = np.around(phi_wls, 6)
        
        # Compute WLS standard deviations efficiently
        std_wls = self._compute_wls_standard_deviations(C_X, phi_wls)
        
        # Update result metrics
        if hasattr(self, 'result'):
            self.result["wls_std_deviations"] = std_wls
            self.result["covariance_matrix"] = C_X
        
        return self.phi_base
    
    def _compute_wls_standard_deviations(self, C_X, phi_wls):
        """Compute weighted least squares standard deviations efficiently.
        
        Args:
            C_X: Covariance matrix
            phi_wls: WLS parameter estimates
            
        Returns:
            ndarray: Standard deviations as percentages
        """
        # Extract diagonal elements (variances)
        variances = np.diag(C_X)
        
        # Compute relative standard deviations as percentages
        # Using vectorized operations instead of loop
        std_deviations = np.sqrt(variances)
        relative_std = 100 * std_deviations / np.abs(phi_wls)
        
        return np.around(relative_std, 2)
    
    def _save_tx40_parameters(self):
        """Save TX40-specific parameters to CSV files."""
        try:
            import csv
            from os.path import join, dirname, abspath
            
            # Save base parameters
            path_save_bp = join(
                dirname(dirname(str(abspath(__file__)))),
                "results/TX40_base_params.csv"
            )
            
            with open(path_save_bp, "w") as output_file:
                w = csv.writer(output_file)
                w.writerow([
                    "Parameter", "OLS_Value", "OLS_Std%",
                    "WLS_Value", "WLS_Std%"
                ])
                for i in range(len(self.params_base)):
                    ols_std = getattr(
                        self, '_ols_std', [0] * len(self.params_base)
                    )[i]
                    wls_std = getattr(
                        self, '_wls_std', [0] * len(self.params_base)
                    )[i]
                    w.writerow([
                        self.params_base[i],
                        self.phi_base[i],
                        ols_std,
                        self.phi_base[i],
                        wls_std
                    ])
            
            print(f"TX40 base parameters saved to {path_save_bp}")
            
        except Exception as e:
            print(f"Error saving TX40 parameters: {e}")
