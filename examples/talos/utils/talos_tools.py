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

import os
import sys
import numpy as np
import matplotlib.pyplot as plt

from figaroh.calibration.calibration_tools import (
    BaseCalibration,
    calc_updated_fkm,
    initialize_variables,
)

# Add path to shared modules
sys.path.append(os.path.join(os.path.dirname(__file__), '../../shared'))


class TALOSCalibration(BaseCalibration):
    """
    Class for calibrating the TALOS humanoid robot's torso-arm system.
    
    This class provides TALOS-specific calibration functionality for the
    torso-arm kinematic chain, extending the BaseCalibration class with
    robot-specific cost functions and TALOS-specific initialization.
    """
    
    def __init__(self, robot, config_file="config/talos_config.yaml",
                 del_list=[]):
        """Initialize TALOS calibration with robot model and configuration.
        
        Args:
            robot: TALOS robot model loaded with FIGAROH
            config_file: Path to TALOS configuration YAML file
            del_list: List of sample indices to exclude from calibration
        """
        super().__init__(robot, config_file, del_list)
        
        # TALOS-specific initialization
        self.dataset_type = "experimental"  # Can be "experimental" or "sample"
        self.regularization_coefficient = 1e-3
        
    def cost_function(self, var):
        """
        TALOS-specific cost function for torso-arm calibration.
        
        Implements regularization for intermediate kinematic parameters
        while excluding base and marker parameters from regularization.
        This is particularly important for TALOS due to its complex
        kinematic structure.
        
        Args:
            var (ndarray): Parameter vector to evaluate
            
        Returns:
            ndarray: Weighted residual vector including regularization terms
        """
        # Calculate forward kinematics with current parameters
        PEEe = calc_updated_fkm(
            self.model, self.data, var, self.q_measured, self.calib_config
        )
        
        # Main residual: difference between measured and estimated poses
        main_residuals = self.PEE_measured - PEEe
        
        # TALOS-specific regularization
        # Exclude base position (first 3 params) and markers (last 3*NbMarkers)
        n_markers = self.calib_config["NbMarkers"]
        regularization_params = var[6:-n_markers * 3]
        regularization_term = (
            np.sqrt(self.regularization_coefficient) * regularization_params
        )
        
        # Combine residuals
        total_residuals = np.append(main_residuals, regularization_term)
        
        return total_residuals

    def initialize_variables(self, mode=0, base_position=None):
        """
        Initialize calibration variables for TALOS.
        
        Args:
            mode (int): Initialization mode (0=zeros, 1=random)
            base_position (array_like, optional): Initial base position
            
        Returns:
            tuple: (initial_variables, n_variables)
        """
        var_0, nvars = initialize_variables(self.calib_config, mode=mode)
        
        # Set TALOS-specific base position if provided
        if base_position is not None:
            var_0[:3] = base_position
        else:
            # Default TALOS base position
            var_0[:3] = np.array([-0.16, 0.047, 0.16])
            
        return var_0, nvars

    def load_experimental_data(self, data_path=None):
        """
        Load experimental data for TALOS torso-arm calibration.
        
        Args:
            data_path (str, optional): Path to experimental data file
            
        Returns:
            tuple: (PEE_measured, q_measured) - marker positions and joint
                configs
        """
        if data_path is None:
            data_path = "data/talos_left_arm_02_10_contact.csv"
            
        # Use base class data loading functionality
        self.load_data_set()
        
        return self.PEE_measured, self.q_measured

    def generate_sample_data(self, seed=0.05):
        """
        Generate synthetic calibration data for testing.
        
        Args:
            seed (float): Random seed magnitude for parameter offsets
            
        Returns:
            tuple: (PEE_sample, q_sample) - synthetic data
        """
        # Create artificial parameter offsets
        var_sample, nvars_sample = initialize_variables(
            self.calib_config, mode=1, seed=seed
        )
        
        print(f"{nvars_sample} sample variables: {var_sample}")
        
        # Generate sample configurations
        q_sample = np.empty((self.calib_config["NbSample"], self.model.nq))
        
        for i in range(self.calib_config["NbSample"]):
            config = self.calib_config["q0"]
            rand_config = self.robot.randomConfiguration()
            config[self.calib_config["config_idx"]] = (
                rand_config[self.calib_config["config_idx"]]
            )
            q_sample[i, :] = config
            
        # Generate synthetic end effector measurements
        PEE_sample = calc_updated_fkm(
            self.model, self.data, var_sample, q_sample, self.calib_config
        )
        
        return PEE_sample, q_sample

    def solve(self, method="lm", max_iterations=3, outlier_threshold=3.0,
              enable_logging=True):
        """
        Solve TALOS calibration using robust optimization.
        
        Args:
            method (str): Optimization method ('lm' for Levenberg-Marquardt)
            max_iterations (int): Maximum iterations for outlier removal
            outlier_threshold (float): Threshold for outlier detection
            enable_logging (bool): Whether to enable detailed logging
            
        Returns:
            scipy.optimize.OptimizeResult: Optimization result
        """
        # Initialize variables
        var_0, nvars = self.initialize_variables(mode=0)
        
        print(f"Initial guess: {var_0}")
        print(f"Number of parameters: {nvars}")
        
        # Use base class optimization with TALOS-specific cost function
        result = self.solve_optimisation(
            var_init=var_0,
            max_iterations=max_iterations,
            outlier_threshold=outlier_threshold,
            enable_logging=enable_logging
        )
        
        return result

    def analyze_results(self, result=None):
        """
        Analyze calibration results and compute quality metrics.
        
        Args:
            result: Optimization result (uses self.LM_result if None)
            
        Returns:
            dict: Analysis results and quality metrics
        """
        if result is None:
            result = self.LM_result
            
        if result is None:
            msg = "No calibration result available. Run solve() first."
            raise ValueError(msg)
        
        # Get solution
        solution = result.x
        
        # Calculate estimated poses with solution
        PEEe_sol = calc_updated_fkm(
            self.model, self.data, solution, self.q_measured,
            self.calib_config, verbose=True
        )
        
        # Calculate RMSE
        rmse = np.sqrt(np.mean((PEEe_sol - self.PEE_measured) ** 2))
        
        # Calculate uncalibrated baseline for comparison
        var_uncalib, _ = self.initialize_variables(mode=0)
        var_uncalib[:3] = solution[:3]  # Keep base position
        var_uncalib[-3:] = solution[-3:]  # Keep marker positions
        
        PEEe_uncalib = calc_updated_fkm(
            self.model, self.data, var_uncalib, self.q_measured,
            self.calib_config
        )
        diff_uncalib = (PEEe_uncalib - self.PEE_measured) ** 2
        rmse_uncalib = np.sqrt(np.mean(diff_uncalib))
        
        analysis = {
            'solution': solution,
            'rmse_calibrated': rmse,
            'rmse_uncalibrated': rmse_uncalib,
            'improvement_ratio': rmse_uncalib / rmse,
            'optimality': result.optimality,
            'cost': result.cost,
            'success': result.success,
            'PEE_estimated': PEEe_sol,
            'PEE_uncalibrated': PEEe_uncalib
        }
        
        print(f"Solution: {solution}")
        print(f"Calibrated RMSE: {rmse:.6f}")
        print(f"Uncalibrated RMSE: {rmse_uncalib:.6f}")
        print(f"Improvement: {analysis['improvement_ratio']:.2f}x")
        print(f"Optimality: {result.optimality}")
        
        return analysis

    def calculate_standard_deviations(self, result=None):
        """
        Calculate standard deviations of estimated parameters.
        
        Args:
            result: Optimization result (uses self.LM_result if None)
            
        Returns:
            tuple: (std_dev, std_percentage) - absolute and relative std devs
        """
        if result is None:
            result = self.LM_result
            
        if result is None:
            msg = "No calibration result available. Run solve() first."
            raise ValueError(msg)
        
        # Calculate standard deviation using Khalil method (Chapter 11)
        n_params = len(result.x)
        n_residuals = len(result.fun)
        dof = n_residuals - n_params
        
        sigma_ro_sq = (result.cost ** 2) / dof
        J = result.jac
        C_param = sigma_ro_sq * np.linalg.pinv(np.dot(J.T, J))
        
        std_dev = []
        std_percentage = []
        
        for i in range(n_params):
            std_val = np.sqrt(C_param[i, i])
            std_dev.append(std_val)
            if abs(result.x[i]) > 1e-10:  # Avoid division by zero
                std_percentage.append(abs(std_val / result.x[i]) * 100)
            else:
                std_percentage.append(float('inf'))
        
        print(f"Standard deviations: {std_dev}")
        print(f"Standard deviation percentages: {std_percentage}")
        
        return std_dev, std_percentage

    def detect_outliers(self, threshold=0.02):
        """
        Detect outlier samples based on position errors.
        
        Args:
            threshold (float): Distance threshold for outlier detection
                (meters)
            
        Returns:
            list: List of (marker_idx, sample_idx) tuples for outliers
        """
        if not hasattr(self, 'evaluation_metrics'):
            raise ValueError("Run solve() first to get residuals.")
        
        # Get residuals from last evaluation
        residuals = self.PEE_measured - self.evaluation_metrics.get(
            'PEE_estimated', self.PEE_measured
        )
        
        # Reshape to analyze per marker and sample
        n_markers = self.calib_config["NbMarkers"]
        n_samples = self.calib_config["NbSample"]
        
        delta_PEE = residuals.reshape((n_markers * 3, n_samples))
        outliers = []
        
        # Calculate distance errors for each marker at each sample
        for marker in range(n_markers):
            for sample in range(n_samples):
                # Calculate 3D distance error
                error_3d = np.sqrt(
                    delta_PEE[marker * 3, sample] ** 2
                    + delta_PEE[marker * 3 + 1, sample] ** 2
                    + delta_PEE[marker * 3 + 2, sample] ** 2
                )
                
                if error_3d > threshold:
                    outliers.append((marker, sample))
        
        threshold_mm = threshold * 1000
        msg = f"Found {len(outliers)} outlier samples"
        msg += f" with >{threshold_mm:.0f}mm deviation"
        print(msg)
        
        return outliers

    def plot_calibration_results(
        self, show_outliers=True, outlier_threshold=0.02
    ):
        """
        Generate comprehensive calibration result plots.
        
        Args:
            show_outliers (bool): Whether to highlight outliers
            outlier_threshold (float): Threshold for outlier detection
        """
        if not hasattr(self, 'evaluation_metrics'):
            raise ValueError("Run solve() first to generate plots.")
        
        # Get analysis results
        analysis = self.analyze_results()
        PEEe_sol = analysis['PEE_estimated']
        
        # Calculate residuals
        delta_PEE = PEEe_sol - self.PEE_measured
        n_markers = self.calib_config["NbMarkers"]
        n_samples = self.calib_config["NbSample"]
        
        # Reshape for plotting
        PEE_xyz = delta_PEE.reshape((n_markers * 3, n_samples))
        PEE_dist = np.zeros((n_markers, n_samples))
        
        for i in range(n_markers):
            for j in range(n_samples):
                PEE_dist[i, j] = np.sqrt(
                    PEE_xyz[i * 3, j] ** 2 +
                    PEE_xyz[i * 3 + 1, j] ** 2 +
                    PEE_xyz[i * 3 + 2, j] ** 2
                )
        
        # Detect outliers if requested
        outliers = []
        if show_outliers:
            outliers = self.detect_outliers(outlier_threshold)
        
        # Plot 1: Error bars per marker
        fig1, ax1 = plt.subplots(n_markers, 1, figsize=(12, 8))
        fig1.suptitle(
            "Position errors between estimated and measured markers (m)",
            fontsize=14
        )
        
        colors = ["blue", "red", "green", "purple", "orange"]
        
        if n_markers == 1:
            ax1 = [ax1]  # Make it iterable
            
        for i in range(n_markers):
            ax1[i].bar(
                np.arange(n_samples), PEE_dist[i, :],
                color=colors[i % len(colors)], alpha=0.7
            )
            ax1[i].set_xlabel("Sample")
            ax1[i].set_ylabel(f"Error marker {i + 1} (m)")
            ax1[i].grid(True, alpha=0.3)
            
            # Highlight outliers
            if show_outliers:
                outlier_samples = [s for m, s in outliers if m == i]
                if outlier_samples:
                    ax1[i].scatter(
                        outlier_samples,
                        PEE_dist[i, outlier_samples],
                        color='red', s=50, marker='x',
                        label='Outliers'
                    )
                    ax1[i].legend()
        
        plt.tight_layout()
        
        # Plot 2: 3D scatter plot of measured vs estimated positions
        fig2 = plt.figure(figsize=(12, 10))
        ax2 = fig2.add_subplot(111, projection='3d')
        
        PEEm_3d = self.PEE_measured.reshape((n_markers * 3, n_samples))
        PEEe_3d = PEEe_sol.reshape((n_markers * 3, n_samples))
        
        for i in range(n_markers):
            # Measured positions (blue)
            ax2.scatter3D(
                PEEm_3d[i * 3, :],
                PEEm_3d[i * 3 + 1, :],
                PEEm_3d[i * 3 + 2, :],
                color=colors[i % len(colors)],
                alpha=0.6,
                label=f'Measured M{i + 1}',
                s=30
            )
            
            # Estimated positions (red)
            ax2.scatter3D(
                PEEe_3d[i * 3, :],
                PEEe_3d[i * 3 + 1, :],
                PEEe_3d[i * 3 + 2, :],
                color=colors[i % len(colors)],
                alpha=0.8,
                marker='^',
                label=f'Estimated M{i + 1}',
                s=30
            )
        
        ax2.set_xlabel("X (m)")
        ax2.set_ylabel("Y (m)")
        ax2.set_zlabel("Z (m)")
        ax2.set_title("Measured vs Estimated Marker Positions")
        ax2.legend()
        
        plt.show()
        
        return fig1, fig2
