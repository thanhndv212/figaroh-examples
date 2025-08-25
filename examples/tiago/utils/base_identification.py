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
Base class for robot dynamic parameter identification.
This module provides a generalized framework for dynamic parameter identification
that can be inherited by any robot type (TIAGo, UR10, MATE, etc.).
"""

import yaml
import numpy as np
import pandas as pd
from abc import ABC, abstractmethod
from os.path import abspath

# FIGAROH imports
from figaroh.identification.identification_tools import (
    get_param_from_yaml as get_identification_param_from_yaml,
)
from figaroh.tools.regressor import (
    build_regressor_basic,
    get_index_eliminate,
    build_regressor_reduced,
)
from figaroh.identification.identification_tools import get_standard_parameters


class BaseIdentification(ABC):
    """
    Base class for robot dynamic parameter identification.
    
    Provides common functionality for all robots while allowing
    robot-specific implementations of key methods.
    """
    
    def __init__(self, robot, config_file="config/robot_config.yaml"):
        """Initialize base identification with robot model and configuration.
        
        Args:
            robot: Robot model loaded with FIGAROH
            config_file: Path to robot configuration YAML file
        """
        self._robot = robot
        self.robot = robot
        self.model = robot.model
        self.data = robot.data
        
        # Load configuration using the TIAGo-style approach
        self.load_param(config_file)
        
        # Initialize attributes for identification results
        self.W = None
        self.standard_parameter = None
        self.params_base = None
        self.W_base = None
        self.phi_base = None
        self.rms_error = None
        self.correlation = None
        self.processed_data = None
        self.result = None
        self.Nsample_ = None
        self.tau_ref = None
        self.tau_identif = None
        self.tau_noised = None
        
        print(f"{self.__class__.__name__} initialized")

    def initialize(self, truncate=None):
        self.process_data(truncate=truncate)
        self.calculate_full_regressor()

    def solve(self, decimate=True, plotting=True, save_params=False):
        """Main solving method for dynamic parameter identification.
        
        Args:
            truncate: None for no truncation, or tuple/list (start, end) 
                     for custom truncation indices
            decimate: Whether to apply decimation to reduce data size
            plotting: Whether to show plots
            save_params: Whether to save parameters to file
        """
        print(f"Starting {self.__class__.__name__} dynamic parameter identification...")
        
        self.calculate_baseparam(decimate=decimate, plotting=plotting, save_params=save_params)
        
        print(f"Dynamic identification completed")
        print(f"RMS error: {self.rms_error:.6f}")
        print(f"Correlation: {self.correlation:.4f}")
        print(f"Condition number: {self.result['condition number']:.2e}")
        print(f"{len(self.params_base)} base parameters identified")
        
        return self.phi_base
    
    def load_param(self, config_file, setting_type="identification"):
        """Load the identification parameters from the yaml file."""
        with open(config_file, "r") as f:
            config = yaml.load(f, Loader=yaml.SafeLoader)
        self.identif_config = get_identification_param_from_yaml(
            self._robot, config[setting_type]
        )

    def load_csv_data(self):
        """Load and process CSV data (generic implementation)."""
        ts = pd.read_csv(
            abspath(self.identif_config["pos_data"]), usecols=[0]
        ).to_numpy()
        pos = pd.read_csv(abspath(self.identif_config["pos_data"]))
        vel = pd.read_csv(abspath(self.identif_config["vel_data"]))
        eff = pd.read_csv(abspath(self.identif_config["torque_data"]))

        cols = {"pos": [], "vel": [], "eff": []}
        for jn in self.identif_config["active_joints"]:
            cols["pos"].extend([col for col in pos.columns if jn in col])
            cols["vel"].extend([col for col in vel.columns if jn in col])
            cols["eff"].extend([col for col in eff.columns if jn in col])

        q = pos[cols["pos"]].to_numpy()
        dq = vel[cols["vel"]].to_numpy()
        tau = eff[cols["eff"]].to_numpy()
        return ts, q, dq, tau
    
    def apply_filters(self, t, q, dq, nbutter=4, f_butter=2, med_fil=5, f_sample=100):
        """Apply median and lowpass filters to position and velocity data."""
        from scipy import signal
        b1, b2 = signal.butter(nbutter, f_butter / (f_sample / 2), "low")

        q_filtered = np.zeros(q.shape)
        dq_filtered = np.zeros(dq.shape)

        for j in range(dq.shape[1]):
            q_med = signal.medfilt(q[:, j], med_fil)
            q_filtered[:, j] = signal.filtfilt(
                b1, b2, q_med, padtype="odd", padlen=3 * (max(len(b1), len(b2)) - 1)
            )
            dq_med = signal.medfilt(dq[:, j], med_fil)
            dq_filtered[:, j] = signal.filtfilt(
                b1, b2, dq_med, padtype="odd", padlen=3 * (max(len(b1), len(b2)) - 1)
            )
        return q_filtered, dq_filtered

    def estimate_acceleration(self, t, dq_filtered):
        """Estimate acceleration from filtered velocity."""
        return np.array([
            np.gradient(dq_filtered[:, j]) / np.gradient(t[:, 0])
            for j in range(dq_filtered.shape[1])
        ]).T

    def build_full_configuration(self, q_f, dq_f, ddq_f, N_):
        """Build full configuration arrays for position, velocity, and acceleration."""
        p = np.tile(self._robot.q0, (N_, 1))
        v = np.tile(self._robot.v0, (N_, 1))
        a = np.tile(self._robot.v0, (N_, 1))

        p[:, self.identif_config["act_idxq"]] = q_f
        v[:, self.identif_config["act_idxv"]] = dq_f
        a[:, self.identif_config["act_idxv"]] = ddq_f
        return p, v, a
    
    def process_torque_data(self, tau):
        """Process torque data (generic implementation, should be overridden for robot-specific processing)."""
        # Generic torque processing - robots should override this method
        return tau

    def process_data(self, truncate=None):
        """Load and process data"""
        t_, q_, dq_, tau_ = self.load_csv_data()

        # Truncate data if truncation indices are provided
        if truncate is not None:
            if isinstance(truncate, (list, tuple)) and len(truncate) == 2:
                # Use provided truncation indices
                n_i, n_f = truncate
                t_ = t_[n_i:n_f]
                q_ = q_[n_i:n_f]
                dq_ = dq_[n_i:n_f]
                tau_ = tau_[n_i:n_f]

        # Apply filters and estimate acceleration
        q_filtered_, dq_filtered_ = self.apply_filters(t_, q_, dq_)
        ddq_filtered_ = self.estimate_acceleration(t_, dq_filtered_)

        # after processing, total number of samples
        self.Nsample_ = q_filtered_.shape[0]

        # Build full configuration
        p_, v_, a_ = self.build_full_configuration(
            q_filtered_, dq_filtered_, ddq_filtered_, self.Nsample_
        )

        # Process torque data
        tau_prcd = self.process_torque_data(tau_)
        self.processed_data = {
            "t": t_,
            "p": p_,
            "v": v_,
            "a": a_,
            "tau": tau_prcd,
        }

    def get_standard_parameters(self):
        """Get standard parameters for TIAGo robot."""
        return get_standard_parameters(self.model, self.identif_config)
    
    def calculate_full_regressor(self):
        """Build regressor matrix."""
        self.W = build_regressor_basic(
            self._robot,
            self.processed_data["p"],
            self.processed_data["v"],
            self.processed_data["a"],
            self.identif_config,
        )
        self.standard_parameter = self.get_standard_parameters()
        # joint torque estimated from p,v,a with std params
        phi_ref = np.array(list(self.standard_parameter.values()))
        tau_ref = np.dot(self.W, phi_ref)
        self.tau_ref = tau_ref[range(len(self.identif_config["act_idxv"]) * self.Nsample_)]

    def calculate_baseparam(self, decimate=True, decimation_factor=10, 
                       zero_tolerance=0.001, plotting=True, save_params=False):
        """Calculate base parameters using QR decomposition.
        
        This method implements the complete base parameter identification
        workflow including column elimination, optional decimation, QR
        decomposition, and quality metric computation.
        
        Args:
            decimate (bool): Whether to apply decimation to reduce data size
            decimation_factor (int): Factor for signal decimation (default: 10)
            zero_tolerance (float): Tolerance for eliminating zero columns
            plotting (bool): Whether to generate plots (unused in base class)
            save_params (bool): Whether to save parameters (unused in base class)
            
        Returns:
            dict: Results dictionary containing base parameters and metrics
            
        Raises:
            AssertionError: If prerequisites not met (W, standard_parameter)
            ValueError: If data shapes are incompatible
            np.linalg.LinAlgError: If QR decomposition fails
            
        Side Effects:
            - Updates self.result with complete results dictionary
            - Updates self.W_base, self.phi_base, self.params_base
            - Updates self.tau_identif, self.tau_noised
            - Updates self.rms_error, self.correlation
        """
        self._validate_prerequisites()
        
        # Step 1: Eliminate zero columns
        regressor_reduced, active_params = self._eliminate_zero_columns(
            zero_tolerance)
        
        # Step 2: Apply decimation if requested
        if decimate:
            tau_processed, W_processed = self._apply_decimation(
                regressor_reduced, decimation_factor)
        else:
            tau_processed, W_processed = self._prepare_undecimated_data(
                regressor_reduced)
        
        # Step 3: Calculate base parameters
        results = self._calculate_base_parameters(
            tau_processed, W_processed, active_params)
        
        # Step 4: Store results and compute quality metrics
        self._store_results(results)
        self._compute_quality_metrics()
        
        # Step 5: Optional plotting
        if plotting:
            self._plot_identification_results()
        
        # Step 6: Optional parameter saving
        if save_params:
            self._save_parameters_to_file()
        
        return self.result

    def _validate_prerequisites(self):
        """Validate that required data is available for calculation.
        
        Raises:
            AssertionError: If required attributes are not set
        """
        assert hasattr(self, 'W') and self.W is not None, \
               "Regressor matrix W not calculated. " \
               "Call calculate_full_regressor() first."
        assert hasattr(self, 'standard_parameter') and \
               self.standard_parameter is not None, \
               "Standard parameters not loaded. " \
               "Call calculate_full_regressor() first."
        assert hasattr(self, 'processed_data') and \
               self.processed_data is not None, \
               "Data not processed. Call process_data() first."

    def _eliminate_zero_columns(self, zero_tolerance):
        """Eliminate columns with near-zero values from regressor matrix.
        
        Args:
            zero_tolerance (float): Tolerance for considering columns as zero
            
        Returns:
            tuple: (regressor_reduced, active_parameters)
        """
        idx_eliminated, active_parameters = get_index_eliminate(
            self.W, self.standard_parameter, tol_e=zero_tolerance
        )
        regressor_reduced = build_regressor_reduced(self.W, idx_eliminated)
        return regressor_reduced, active_parameters

    def _apply_decimation(self, regressor_reduced, decimation_factor):
        """Apply signal decimation to reduce data size.
        
        Args:
            regressor_reduced (ndarray): Reduced regressor matrix
            decimation_factor (int): Factor for decimation
            
        Returns:
            tuple: (tau_decimated, regressor_decimated)
        """
        from scipy import signal
        
        # Decimate torque data
        tau_decimated_list = []
        num_joints = len(self.identif_config["act_idxv"])
        
        for i in range(num_joints):
            tau_joint = self.processed_data["tau"][:, i]
            tau_dec = signal.decimate(tau_joint, q=decimation_factor,
                                      zero_phase=True)
            tau_decimated_list.append(tau_dec)

        # Concatenate decimated torque data
        tau_decimated = tau_decimated_list[0]
        for i in range(1, len(tau_decimated_list)):
            tau_decimated = np.append(tau_decimated, tau_decimated_list[i])

        # Decimate regressor matrix
        regressor_decimated = self._decimate_regressor_matrix(
            regressor_reduced, decimation_factor)
        
        return tau_decimated, regressor_decimated

    def _decimate_regressor_matrix(self, regressor_reduced, decimation_factor):
        """Decimate the regressor matrix by joints.
        
        Args:
            regressor_reduced (ndarray): Reduced regressor matrix
            decimation_factor (int): Decimation factor
            
        Returns:
            ndarray: Decimated regressor matrix
        """
        from scipy import signal
        
        num_joints = len(self.identif_config["act_idxv"])
        regressor_list = []
        
        for i in range(num_joints):
            # Extract rows corresponding to joint i
            start_idx = self.identif_config["act_idxv"][i] * self.Nsample_
            end_idx = (self.identif_config["act_idxv"][i] + 1) * self.Nsample_
            
            joint_regressor_decimated = []
            for j in range(regressor_reduced.shape[1]):
                column_data = regressor_reduced[start_idx:end_idx, j]
                decimated_column = signal.decimate(
                    column_data, q=decimation_factor, zero_phase=True)
                joint_regressor_decimated.append(decimated_column)

            # Reconstruct matrix for this joint
            joint_matrix = np.zeros((len(joint_regressor_decimated[0]),
                                     len(joint_regressor_decimated)))
            for k, column in enumerate(joint_regressor_decimated):
                joint_matrix[:, k] = column
            regressor_list.append(joint_matrix)

        # Concatenate all joint matrices
        total_rows = sum(matrix.shape[0] for matrix in regressor_list)
        regressor_decimated = np.zeros((total_rows,
                                        regressor_list[0].shape[1]))
        
        current_row = 0
        for matrix in regressor_list:
            next_row = current_row + matrix.shape[0]
            regressor_decimated[current_row:next_row, :] = matrix
            current_row = next_row
            
        return regressor_decimated

    def _prepare_undecimated_data(self, regressor_reduced):
        """Prepare data without decimation.
        
        Args:
            regressor_reduced (ndarray): Reduced regressor matrix
            
        Returns:
            tuple: (tau_flattened, regressor_reduced)
        """
        tau_data = self.processed_data["tau"]
        if hasattr(tau_data, 'flatten'):
            tau_flattened = tau_data.flatten()
        else:
            tau_flattened = tau_data
        return tau_flattened, regressor_reduced

    def _calculate_base_parameters(self, tau_processed, regressor_processed,
                                   active_parameters):
        """Calculate base parameters using QR decomposition.
        
        Args:
            tau_processed (ndarray): Processed torque data
            regressor_processed (ndarray): Processed regressor matrix
            active_parameters (dict): Active parameter dictionary
            
        Returns:
            dict: Results from QR decomposition
        """
        from figaroh.tools.qrdecomposition import double_QR
        from figaroh.identification.identification_tools import relative_stdev
        
        # Perform QR decomposition
        W_base, base_param_dict, base_parameters, phi_base, phi_std = \
            double_QR(tau_processed, regressor_processed, active_parameters,
                      self.standard_parameter)
        
        # Calculate torque estimation (avoid redundant computation)
        tau_estimated = np.dot(W_base, phi_base)
        
        # Calculate quality metrics
        rmse = np.linalg.norm(tau_processed - tau_estimated) / np.sqrt(
            tau_processed.shape[0])
        std_relative = relative_stdev(W_base, phi_base, tau_processed)

        return {
            "base_regressor": W_base,
            "base_param_dict": base_param_dict,
            "base_parameters": base_parameters,
            "phi_base": phi_base,
            "tau_estimated": tau_estimated,
            "tau_processed": tau_processed,
            "rmse": rmse,
            "std_relative": std_relative
        }

    def _store_results(self, calculation_results):
        """Store calculation results in instance attributes.
        
        Args:
            calculation_results (dict): Results from base parameter calculation
        """
        W_base = calculation_results["base_regressor"]
        phi_base = calculation_results["phi_base"]
        base_param_dict = calculation_results["base_param_dict"]
        tau_estimated = calculation_results["tau_estimated"]
        
        self.result = {
            "base regressor": W_base,
            "base parameters": base_param_dict,
            "condition number": np.linalg.cond(W_base),
            "rmse norm (N/m)": calculation_results["rmse"],
            "torque estimated": tau_estimated,
            "std dev of estimated param": calculation_results["std_relative"],
        }
        
        # Store key results for backward compatibility
        self.W_base = W_base
        self.phi_base = phi_base
        self.params_base = list(base_param_dict.keys())
        self.tau_identif = tau_estimated
        self.tau_noised = calculation_results["tau_processed"]

    def _compute_quality_metrics(self):
        """Compute quality metrics for the identification.
        
        Side Effects:
            - Updates self.rms_error
            - Updates self.correlation
        """
        residuals = self.tau_noised - self.tau_identif
        self.rms_error = np.sqrt(np.mean(residuals**2))
        
        if len(self.tau_noised) > 1 and len(self.tau_identif) > 1:
            try:
                correlation_matrix = np.corrcoef(self.tau_noised,
                                                 self.tau_identif)
                self.correlation = correlation_matrix[0, 1]
            except (np.linalg.LinAlgError, ValueError):
                self.correlation = 1.0
        else:
            self.correlation = 1.0

    def _plot_identification_results(self):
        """Plot identification results if plotting is enabled.
        
        Creates visualizations of:
        - Estimated vs measured torques
        - Parameter estimation quality
        """
        try:
            import matplotlib.pyplot as plt
            
            fig, axes = plt.subplots(2, 1, figsize=(12, 8))
            
            # Plot torque comparison
            time_vector = np.arange(len(self.tau_noised))
            axes[0].plot(time_vector, self.tau_noised, 'b-',
                         label='Measured torques', linewidth=1)
            axes[0].plot(time_vector, self.tau_identif, 'r--',
                         label='Estimated torques', linewidth=1)
            axes[0].set_xlabel('Sample')
            axes[0].set_ylabel('Torque (N⋅m)')
            axes[0].set_title('Torque Identification Results')
            axes[0].legend()
            axes[0].grid(True, alpha=0.3)
            
            # Plot residuals
            residuals = self.tau_noised - self.tau_identif
            axes[1].plot(time_vector, residuals, 'g-', linewidth=1)
            axes[1].set_xlabel('Sample')
            axes[1].set_ylabel('Residual (N⋅m)')
            axes[1].set_title(f'Residuals (RMSE: {self.rms_error:.4f} N⋅m)')
            axes[1].grid(True, alpha=0.3)
            
            plt.tight_layout()
            plt.show()
            
        except ImportError:
            print("Warning: matplotlib not available for plotting")
        except Exception as e:
            print(f"Warning: Plotting failed: {e}")

    def _save_parameters_to_file(self):
        """Save identified parameters to file.
        
        Saves base parameters and quality metrics to a JSON file
        in the same directory as the identification results.
        """
        try:
            import json
            import os
            from datetime import datetime
            
            # Create results directory if it doesn't exist
            results_dir = os.path.join(os.getcwd(), 'identification_results')
            os.makedirs(results_dir, exist_ok=True)
            
            # Prepare data for saving
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"base_parameters_{timestamp}.json"
            filepath = os.path.join(results_dir, filename)
            
            # Convert numpy arrays to lists for JSON serialization
            save_data = {
                "timestamp": timestamp,
                "base_parameters": {
                    name: float(value) if hasattr(value, 'item') else value
                    for name, value in self.result["base parameters"].items()
                },
                "quality_metrics": {
                    "condition_number": float(self.result["condition number"]),
                    "rmse_norm": float(self.result["rmse norm (N/m)"]),
                    "correlation": float(self.correlation),
                    "rms_error": float(self.rms_error)
                },
                "regressor_shape": list(self.result["base regressor"].shape),
                "num_parameters": len(self.result["base parameters"])
            }
            
            with open(filepath, 'w') as f:
                json.dump(save_data, f, indent=2)
            
            print(f"Parameters saved to: {filepath}")
            
        except Exception as e:
            print(f"Warning: Failed to save parameters: {e}")
    
    def plot_results(self):
        """Plot identification results."""
        if not hasattr(self, 'result') or self.result is None:
            print("No identification results to plot. Run solve() first.")
            return
        
        import matplotlib.pyplot as plt
        
        # Plot torque comparison
        plt.figure(figsize=(12, 8))
        
        plt.subplot(2, 1, 1)
        plt.plot(self.tau_noised, label="Measured (with noise)", alpha=0.7)
        plt.plot(self.tau_identif, label="Identified", alpha=0.7)
        plt.xlabel('Sample')
        plt.ylabel('Torque (Nm)')
        plt.title(f'{self.__class__.__name__} Torque Comparison')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # Plot parameter values
        plt.subplot(2, 1, 2)
        plt.bar(range(len(self.phi_base)), self.phi_base, alpha=0.7, label="Base Parameters")
        plt.xlabel('Parameter Index')
        plt.ylabel('Parameter Value')
        plt.title('Identified Base Parameters')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()
    
    def save_results(self, output_dir="results"):
        """Save identification results to files."""
        if not hasattr(self, 'result') or self.result is None:
            print("No identification results to save. Run solve() first.")
            return
        
        import os
        import yaml
        
        os.makedirs(output_dir, exist_ok=True)
        
        # Save identified parameters
        results_dict = {
            'base_parameters': self.phi_base.tolist(),
            'parameter_names': [str(p) for p in self.params_base],
            'rms_error': float(self.rms_error),
            'correlation': float(self.correlation),
            'condition_number': float(self.result['condition number']),
            'standard_deviation': self.result['std dev of estimated param'].tolist() if hasattr(self.result['std dev of estimated param'], 'tolist') else self.result['std dev of estimated param']
        }
        
        robot_name = self.__class__.__name__.lower().replace('identification', '')
        filename = f"{robot_name}_identification_results.yaml"
        
        with open(os.path.join(output_dir, filename), "w") as f:
            yaml.dump(results_dict, f, default_flow_style=False)
        
        print(f"Results saved to {output_dir}/{filename}")
