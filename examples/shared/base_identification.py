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
from abc import ABC, abstractmethod

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
        self.robot = robot
        self.model = self.robot.model
        self.data = self.robot.data

        # Load configuration using the TIAGo-style approach
        self.load_param(config_file)
        
        # Initialize attributes for identification results
        self.dynamic_regressor = None
        self.standard_parameter = None
        self.params_base = None
        self.dynamic_regressor_base = None
        self.phi_base = None
        self.rms_error = None
        self.correlation = None
        self.processed_data = None
        self.result = None
        self.num_samples = None
        self.tau_ref = None
        self.tau_identif = None
        self.tau_noised = None
        self.filter_config = {
            'differentiation_method': 'gradient',
            'filter_params': {}
        }
        print(f"{self.__class__.__name__} initialized")

    def initialize(self, truncate=None):
        self.process_data(truncate=truncate)
        self.calculate_full_regressor()

    def solve(self, decimate=True, decimation_factor=10, zero_tolerance=0.001,
              plotting=True, save_results=False):
        """Main solving method for dynamic parameter identification.
        
        This method implements the complete base parameter identification
        workflow including column elimination, optional decimation, QR
        decomposition, and quality metric computation.
        
        Args:
            decimate (bool): Whether to apply decimation to reduce data size
            decimation_factor (int): Factor for signal decimation (default: 10)
            zero_tolerance (float): Tolerance for eliminating zero columns
            plotting (bool): Whether to generate plots
            save_results (bool): Whether to save parameters to file
            
        Returns:
            ndarray: Base parameters phi_base
            
        Raises:
            AssertionError: If prerequisites not met (dynamic_regressor, standard_parameter)
            ValueError: If data shapes are incompatible
            np.linalg.LinAlgError: If QR decomposition fails
        """
        print(f"Starting {self.__class__.__name__} dynamic parameter identification...")
        
        # Validate prerequisites
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
        self._compute_quality_metrics()
        self._store_results(results)
        
        # Step 5: Optional plotting
        if plotting:
            self.plot_results()
        
        # Step 6: Optional parameter saving
        if save_results:
            self.save_results()
        
        return self.phi_base
    
    def load_param(self, config_file, setting_type="identification"):
        """Load the identification parameters from the yaml file."""
        with open(config_file, "r") as f:
            config = yaml.load(f, Loader=yaml.SafeLoader)
        self.identif_config = get_identification_param_from_yaml(
            self.robot, config[setting_type]
        )

    @abstractmethod
    def load_trajectory_data(self):
        """Load and process CSV data.
        
        This method must be implemented by robot-specific subclasses
        to handle their specific data formats and file structures.
        
        Returns:
            tuple: (timestamps, positions, velocities, torques) as numpy arrays
        """
        pass

    def process_data(self, truncate=None):
        """Load and process data"""
        # Set default filter configuration
        filter_config = self.filter_config

        # load raw data
        self.raw_data = self.load_trajectory_data()
        
        # Truncate data if truncation indices are provided
        self.raw_data = self._truncate_data(self.raw_data, truncate)

        # Apply filtering and differentiation kinematics data
        self.process_kinematics_data(filter_config)

        # Process joint torque data
        self.processed_data["torques"] = self.process_torque_data()

        # Update sample count to ensure consistency
        self.num_samples = self.processed_data["positions"].shape[0]

        # Build full configuration
        self._build_full_configuration()
    
    def calculate_full_regressor(self):
        """Build regressor matrix, compute pre-identified values of standard 
        parameters, compute joint torques based on pre-identified standard 
        parameters."""
        # Build full regressor matrix
        self.dynamic_regressor = build_regressor_basic(
            self.robot,
            self.processed_data["positions"],
            self.processed_data["velocities"],
            self.processed_data["accelerations"],
            self.identif_config,
        )

        # Compute standard parameters
        self.standard_parameter = get_standard_parameters(self.model, self.identif_config)

        # additional parameters can be added in robot-specific subclass
        self.add_additional_parameters()

        # Convert all string values to floats in the standard_parameter dict
        for key, value in self.standard_parameter.items():
            if isinstance(value, str):
                self.standard_parameter[key] = float(value)

        # joint torque estimated from p,v,a with std params
        phi_ref = np.array(list(self.standard_parameter.values()))
        tau_ref = np.dot(self.dynamic_regressor, phi_ref)

        # filter only active joints
        self.tau_ref = tau_ref[range(len(self.identif_config["act_idxv"]) * self.num_samples)]

    def add_additional_parameters(self):
        """Add additional parameters specific to the robot and recalculate dynamic regressor."""
        pass

    def _apply_filters(self, *signals, nbutter=4, f_butter=2, med_fil=5, f_sample=100):
        """Apply median and lowpass filters to any number of signals.
        
        Args:
            *signals: Variable number of signal arrays to filter
            nbutter (int): Butterworth filter order (default: 4)
            f_butter (float): Cutoff frequency in Hz (default: 2)
            med_fil (int): Median filter window size (default: 5)
            f_sample (float): Sampling frequency in Hz (default: 100)
            
        Returns:
            tuple: Filtered signals in the same order as input
        """
        from scipy import signal
        
        # Design Butterworth filter coefficients
        b1, b2 = signal.butter(nbutter, f_butter / (f_sample / 2), "low")
        
        filtered_signals = []
        
        for sig in signals:
            if sig is None:
                filtered_signals.append(None)
                continue
                
            # Ensure signal is 2D array
            if sig.ndim == 1:
                sig = sig.reshape(-1, 1)
                
            sig_filtered = np.zeros(sig.shape)
            
            # Apply filters to each column (joint/channel)
            for j in range(sig.shape[1]):
                # Apply median filter first
                sig_med = signal.medfilt(sig[:, j], med_fil)
                
                # Apply Butterworth lowpass filter
                sig_filtered[:, j] = signal.filtfilt(
                    b1, b2, sig_med, padtype="odd", 
                    padlen=3 * (max(len(b1), len(b2)) - 1)
                )
            
            filtered_signals.append(sig_filtered)
        
        # Return single array if only one signal, otherwise tuple
        if len(filtered_signals) == 1:
            return filtered_signals[0]
        return tuple(filtered_signals)

    def _differentiate_signal(self, time_vector, signal, method='gradient'):
        """Compute first derivative of a time series signal.
        
        Args:
            time_vector (ndarray): Time stamps corresponding to signal samples
            signal (ndarray): Signal to differentiate (1D or 2D array)
            method (str): Differentiation method ('gradient', 'forward', 'backward', 'central')
            
        Returns:
            ndarray: First derivative of the signal with same shape as input
            
        Raises:
            ValueError: If time_vector and signal have incompatible shapes
            ValueError: If method is not supported
        """
        # Validate inputs
        if signal.shape[0] != time_vector.shape[0]:
            raise ValueError(f"Time vector length {time_vector.shape[0]} "
                           f"doesn't match signal length {signal.shape[0]}")
        
        # Ensure signal is 2D
        if signal.ndim == 1:
            signal = signal.reshape(-1, 1)
            squeeze_output = True
        else:
            squeeze_output = False
        
        # Handle time vector shape (extract first column if 2D)
        if time_vector.ndim == 2:
            t = time_vector[:, 0]
        else:
            t = time_vector
        
        # Initialize output array
        derivative = np.zeros_like(signal)
        
        # Apply differentiation method to each column
        for j in range(signal.shape[1]):
            sig_col = signal[:, j]
            
            if method == 'gradient':
                # Use numpy gradient (handles edge cases automatically)
                derivative[:, j] = np.gradient(sig_col, t)
                
            elif method == 'forward':
                # Forward difference: df/dt ≈ (f[i+1] - f[i]) / (t[i+1] - t[i])
                derivative[:-1, j] = np.diff(sig_col) / np.diff(t)
                # Extrapolate last point
                derivative[-1, j] = derivative[-2, j]
                
            elif method == 'backward':
                # Backward difference: df/dt ≈ (f[i] - f[i-1]) / (t[i] - t[i-1])
                derivative[1:, j] = np.diff(sig_col) / np.diff(t)
                # Extrapolate first point
                derivative[0, j] = derivative[1, j]
                
            elif method == 'central':
                # Central difference: df/dt ≈ (f[i+1] - f[i-1]) / (t[i+1] - t[i-1])
                derivative[1:-1, j] = (sig_col[2:] - sig_col[:-2]) / (t[2:] - t[:-2])
                # Handle boundary conditions
                derivative[0, j] = (sig_col[1] - sig_col[0]) / (t[1] - t[0])
                derivative[-1, j] = (sig_col[-1] - sig_col[-2]) / (t[-1] - t[-2])
                
            else:
                raise ValueError(f"Unsupported differentiation method: {method}. "
                               f"Use 'gradient', 'forward', 'backward', or 'central'")
        
        # Return with original dimensionality
        if squeeze_output:
            return derivative.squeeze()
        return derivative

    def _build_full_configuration(self):
        """Build full configuration arrays for position, velocity, acceleration.
        
        This method expands the active joint data to full robot configuration
        by filling in default values for inactive joints. Uses vectorized
        operations for optimal performance.
        """
        # Validate required data
        required_keys = ["positions", "velocities", "accelerations"]
        for key in required_keys:
            if key not in self.processed_data or self.processed_data[key] is None:
                raise ValueError(f"Missing required data: {key}")
        
        # Get active joint data
        q_active = self.processed_data["positions"]
        dq_active = self.processed_data["velocities"]
        ddq_active = self.processed_data["accelerations"]
        
        # Create full configuration arrays efficiently
        config_data = [
            (q_active, self.robot.q0, self.identif_config["act_idxq"]),
            (dq_active, self.robot.v0, self.identif_config["act_idxv"]),
            (ddq_active, self.robot.v0, self.identif_config["act_idxv"])
        ]
        
        full_configs = []
        for active_data, default_config, active_indices in config_data:
            # Initialize with defaults
            full_config = np.tile(default_config, (self.num_samples, 1))
            # Fill active joints
            full_config[:, active_indices] = active_data
            full_configs.append(full_config)
        
        # Update processed data efficiently
        config_keys = ["positions", "velocities", "accelerations"]
        self.processed_data.update(dict(zip(config_keys, full_configs)))

    def _truncate_data(self, data_dict, truncate=None):
        """Truncate data arrays based on provided indices.
        
        Args:
            data_dict (dict): Dictionary containing data arrays to truncate
            truncate (tuple/list): Truncation indices (start, end) or None for no truncation
            
        Returns:
            dict: Dictionary with truncated data arrays
        """
        if truncate is None:
            return data_dict.copy()
            
        if not isinstance(truncate, (list, tuple)) or len(truncate) != 2:
            raise ValueError("Truncate parameter must be a tuple/list of length 2 (start, end)")
            
        n_i, n_f = truncate
        truncated_data = {}
        
        for key, array in data_dict.items():
            if array is not None:
                truncated_data[key] = array[n_i:n_f]
            else:
                truncated_data[key] = None
                
        return truncated_data
    
    def process_kinematics_data(self, filter_config=None):
        """Process kinematics data (positions, velocities, accelerations) with filtering."""
        self.filter_kinematics_data(filter_config)

    def filter_kinematics_data(self, filter_config=None):
        """Apply filtering to data with configurable parameters.
        
        Args:
            filter_config (dict, optional): Filter configuration with keys:
                - differentiation_method: Method for derivative estimation
                - filter_params: Parameters for signal filtering
                
        Raises:
            ValueError: If required data is missing
        """ 
        # Validate required data
        if self.raw_data.get("timestamps") is None:
            raise ValueError("Timestamps are required for data processing")
        if self.raw_data.get("positions") is None:
            raise ValueError("Position data is required for processing")
            
        # Create processed data copy to avoid modifying raw data
        self.processed_data = {}
        
        # Process timestamps (no filtering needed)
        self.processed_data["timestamps"] = self.raw_data["timestamps"]
        
        # Define signal processing pipeline
        signal_pipeline = [
            ("positions", self.raw_data["positions"], None),
            ("velocities", self.raw_data.get("velocities"), "positions"),
            ("accelerations", self.raw_data.get("accelerations"), "velocities")
        ]
        
        # Process signals through pipeline
        for signal_name, signal_data, dependency in signal_pipeline:
            if signal_data is not None:
                # Apply filtering to existing data
                self.processed_data[signal_name] = self._apply_filters(
                    signal_data, **filter_config['filter_params'])
            else:
                # Estimate missing signal from dependency
                if dependency:
                    dependency_data = self.processed_data[dependency]
                    self.processed_data[signal_name] = \
                        self._differentiate_signal(
                            self.processed_data["timestamps"],
                            dependency_data,
                            method=filter_config['differentiation_method'])
                else:
                    raise ValueError(
                        f"Cannot process {signal_name}: no data or dependency")

    def process_torque_data(self, **kwargs):
        """Process torque data (generic implementation, should be overridden for robot-specific processing)."""
        # Generic torque processing - robots should override this method
        if self.raw_data["torques"] is not None:
            self.processed_data["torques"] = self.raw_data["torques"]
            return self.processed_data["torques"]
        else:
            raise ValueError("Torque data is required for processing")

    def _validate_prerequisites(self):
        """Validate that required data is available for calculation.
        
        Raises:
            AssertionError: If required attributes are not set
        """
        assert hasattr(self, 'dynamic_regressor') and self.dynamic_regressor is not None, \
               "Regressor matrix not calculated. " \
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
            self.dynamic_regressor, self.standard_parameter, tol_e=zero_tolerance
        )
        regressor_reduced = build_regressor_reduced(self.dynamic_regressor, idx_eliminated)
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
            tau_joint = self.processed_data["torques"][:, i]
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
        
        # Validate that decimated data is properly aligned
        if tau_decimated.shape[0] != regressor_decimated.shape[0]:
            raise ValueError(
                f"Decimated data size mismatch: "
                f"tau_decimated has {tau_decimated.shape[0]} samples, "
                f"regressor_decimated has {regressor_decimated.shape[0]} rows"
            )

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
            start_idx = self.identif_config["act_idxv"][i] * self.num_samples
            end_idx = (self.identif_config["act_idxv"][i] + 1) * self.num_samples
            
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
        tau_data = self.processed_data["torques"]
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
        
        # Perform QR decomposition
        W_base, base_param_dict, base_parameters, phi_base, phi_std = \
            double_QR(tau_processed, regressor_processed, active_parameters,
                      self.standard_parameter)
        
        # Calculate torque estimation (avoid redundant computation)
        tau_estimated = np.dot(W_base, phi_base)
        
        # Store key results for backward compatibility
        self.dynamic_regressor_base = W_base
        self.phi_base = phi_base
        self.params_base = list(base_param_dict.keys())
        self.tau_identif = tau_estimated
        self.tau_noised = tau_processed

        return {
            "base_regressor": W_base,
            "base_param_dict": base_param_dict,
            "base_parameters": base_parameters,
            "phi_base": phi_base,
            "tau_estimated": tau_estimated,
            "tau_processed": tau_processed,
        }

    def _compute_quality_metrics(self):
        """Compute quality metrics for the identification.
        
        Side Effects:
            - Updates self.rms_error
            - Updates self.correlation
        """
        from figaroh.identification.identification_tools import relative_stdev

        # Calculate quality metrics
        self.std_relative = relative_stdev(self.dynamic_regressor_base, self.phi_base, self.tau_noised)
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

    def _store_results(self, identif_results):
        """Store calculation results in instance attributes.
        
        Args:
            identif_results (dict): Results from base parameter calculation
        """

        # Store results in instance attribute
        self.result = {
            "base regressor": identif_results["base_regressor"],
            "base parameters": identif_results["base_param_dict"],
            "base parameters values": identif_results["phi_base"],
            "base parameters names": list(identif_results["base_param_dict"].keys()),
            "condition number": np.linalg.cond(identif_results["base_regressor"]),
            "torque estimated": identif_results["tau_estimated"],
            "torque processed": identif_results["tau_processed"],
            "std dev of estimated param": self.std_relative,
            "rmse norm (N/m)": self.rms_error,
            "num samples": self.num_samples,
            "identification config": getattr(self, 'identif_config', {}),
            "task type": "identification"
        }

        # Initialize ResultsManager for identification task
        try:
            from .results_manager import ResultsManager
            
            # Get robot name from class or model
            robot_name = getattr(
                self, 'robot_name',
                getattr(
                    self.model, 'name',
                    self.__class__.__name__.lower().replace(
                        'identification', '')))
            
            # Initialize results manager for identification task
            self.results_manager = ResultsManager('identification', robot_name, self.result)
            
        except ImportError as e:
            print(f"Warning: ResultsManager not available: {e}")
            self.results_manager = None
    
    def plot_results(self):
        """Plot identification results using unified results manager."""
        if not hasattr(self, 'result') or self.result is None:
            print("No identification results to plot. Run solve() first.")
            return
        
        # Use pre-initialized results manager if available
        if hasattr(self, 'results_manager') and \
           self.results_manager is not None:
            try:
                # Plot using unified manager with self.result data
                self.results_manager.plot_identification_results()
                return
                
            except Exception as e:
                print(f"Error plotting with ResultsManager: {e}")
                print("Falling back to basic plotting...")
        
        # Fallback to basic plotting if ResultsManager not available
        try:
            import matplotlib.pyplot as plt
            
            # Extract data from self.result dictionary
            tau_measured = self.result.get("torque processed", np.array([]))
            tau_identified = self.result.get("torque estimated", np.array([]))
            parameter_values = self.result.get("base parameters values",
                                               np.array([]))
            
            if len(tau_measured) == 0 or len(tau_identified) == 0:
                print("No torque data available for plotting")
                return
            
            plt.figure(figsize=(12, 8))
            
            plt.subplot(2, 1, 1)
            plt.plot(tau_measured, label="Measured (with noise)", alpha=0.7)
            plt.plot(tau_identified, label="Identified", alpha=0.7)
            plt.xlabel('Sample')
            plt.ylabel('Torque (Nm)')
            plt.title(f'{self.__class__.__name__} Torque Comparison')
            plt.legend()
            plt.grid(True, alpha=0.3)
            
            plt.subplot(2, 1, 2)
            if len(parameter_values) > 0:
                plt.bar(range(len(parameter_values)), parameter_values,
                        alpha=0.7, label="Base Parameters")
                plt.xlabel('Parameter Index')
                plt.ylabel('Parameter Value')
                plt.title('Identified Base Parameters')
                plt.legend()
                plt.grid(True, alpha=0.3)
            
            plt.tight_layout()
            plt.show()
            
        except ImportError:
            print("Warning: matplotlib not available for plotting")
        except Exception as e:
            print(f"Warning: Plotting failed: {e}")
    
    def save_results(self, output_dir="results"):
        """Save identification results using unified results manager."""
        if not hasattr(self, 'result') or self.result is None:
            print("No identification results to save. Run solve() first.")
            return
        
        # Use pre-initialized results manager if available
        if hasattr(self, 'results_manager') and \
           self.results_manager is not None:
            try:
                # Save using unified manager with self.result data
                saved_files = self.results_manager.save_results(
                    output_dir=output_dir,
                    save_formats=['yaml', 'csv', 'npz']
                )
                
                print("Identification results saved using ResultsManager")
                for fmt, path in saved_files.items():
                    print(f"  {fmt}: {path}")
                
                return saved_files
                
            except Exception as e:
                print(f"Error saving with ResultsManager: {e}")
                print("Falling back to basic saving...")
        
        # Fallback to basic saving if ResultsManager not available
        try:
            import os
            import yaml
            import datetime

            os.makedirs(output_dir, exist_ok=True)
            
            # Extract data from self.result dictionary
            parameter_values = self.result.get("base parameters values",
                                               np.array([]))
            parameter_names = self.result.get("base parameters names", [])
            condition_number = self.result.get("condition number", 0)
            rmse_norm = self.result.get("rmse norm (N/m)", 0)
            std_dev_param = self.result.get("std dev of estimated param",
                                            np.array([]))
            
            results_dict = {
                'base_parameters': (
                    parameter_values.tolist()
                    if hasattr(parameter_values, 'tolist')
                    else parameter_values),
                'parameter_names': [str(p) for p in parameter_names],
                'condition_number': float(condition_number),
                'rmse_norm': float(rmse_norm),
                'standard_deviation': (
                    std_dev_param.tolist()
                    if hasattr(std_dev_param, 'tolist')
                    else std_dev_param
                )
            }
            
            robot_name = self.__class__.__name__.lower().replace(
                'identification', '')
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"{robot_name}_identification_results_{timestamp}.yaml"

            with open(os.path.join(output_dir, filename), "w") as f:
                yaml.dump(results_dict, f, default_flow_style=False)
            
            print(f"Results saved to {output_dir}/{filename}")
            return {filename: os.path.join(output_dir, filename)}
            
        except Exception as e:
            print(f"Error in fallback saving: {e}")
            return None
