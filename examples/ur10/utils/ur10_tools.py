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
import random
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Import FIGAROH modules
import numpy as np
import pandas as pd
from scipy.optimize import least_squares
from typing import Dict, Any, List
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
    handle_calibration_errors
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

# Fallback for config manager and data processor if needed
try:
    from ...shared.config_manager import ConfigManager
    from ...shared.data_processing import DataProcessor
except ImportError:
    # Create simple fallback classes if shared modules not available
    class ConfigManager:
        def __init__(self, config_path):
            import yaml
            with open(config_path, 'r') as f:
                self.config = yaml.safe_load(f)
        
        def get_config(self):
            return self.config
    
    class DataProcessor:
        @staticmethod
        def load_data_csv(filepath, time_col='time', columns=None):
            import pandas as pd
            return pd.read_csv(filepath)


class UR10Calibration(BaseCalibration):
    """
    Class for calibrating the UR10 robot.
    
    This class provides UR10-specific calibration functionality by extending
    the BaseCalibration class with robot-specific cost functions and
    initialization parameters.
    """
    
    @handle_calibration_errors
    def __init__(self, robot, config_file="config/ur10_config.yaml",
                 del_list=[]):
        """Initialize UR10 calibration with robot model and configuration.
        
        Args:
            robot: UR10 robot model loaded with FIGAROH
            config_file: Path to UR10 configuration YAML file
            del_list: List of sample indices to exclude from calibration
        """
        super().__init__(robot, config_file, del_list)

    def cost_function(self, var):
        """
        UR10-specific cost function for the optimization problem.
        
        Implements proper handling of position/orientation units and
        regularization for intermediate parameters to improve numerical
        stability and convergence.
        
        Args:
            var (ndarray): Parameter vector to evaluate
            
        Returns:
            ndarray: Weighted residual vector including regularization terms
        """
        coeff_ = self.calib_config["coeff_regularize"]
        PEEe = calc_updated_fkm(self.model, self.data, var,
                                self.q_measured, self.calib_config)
        
        # Main residual: difference between measured and estimated poses
        raw_residuals = self.PEE_measured - PEEe
        
        # Apply unit-aware weighting using BaseCalibration utility method
        # This handles position (meters) vs orientation (radians) properly
        weighted_residuals = self.apply_measurement_weighting(
            raw_residuals, pos_weight=1.0, orient_weight=0.5)
        
        # Regularization term for intermediate parameters (excludes base/tip)
        # This helps stabilize optimization for UR10's 6-DOF kinematic chain
        n_base_params = 6  # Base frame parameters
        n_markers = self.calib_config["NbMarkers"]
        n_tip_params = n_markers * self.calib_config["calibration_index"]
        regularization_params = var[n_base_params:-n_tip_params]
        regularization_residuals = np.sqrt(coeff_) * regularization_params
        
        # Combine residuals
        res_vect = np.append(weighted_residuals, regularization_residuals)
        return res_vect


class UR10Identification(BaseIdentification):
    """UR10-specific dynamic parameter identification class."""
    
    @handle_calibration_errors
    def __init__(self, robot, config_file="config/ur10_config.yaml"):
        """Initialize UR10 identification with robot model and configuration.
        
        Args:
            robot: UR10 robot model loaded with FIGAROH
            config_file: Path to UR10 configuration YAML file
        """
        # Call parent constructor
        super().__init__(robot, config_file)
        
        # UR10-specific: Set active joint indices (all joints are active)
        if "act_idxq" not in self.identif_config:
            self.identif_config["act_idxq"] = list(range(self.model.nq))
        if "act_idxv" not in self.identif_config:
            self.identif_config["act_idxv"] = list(range(self.model.nv))
        
        print("UR10 Dynamic Identification initialized")
    
    def load_trajectory_data(self):
        """Load trajectory data from CSV files using DataProcessor.
        
        Returns:
            dict: Dictionary with keys 'timestamps', 'positions',
                  'velocities', 'accelerations', 'torques'
        """
        print("Loading UR10 trajectory data...")
        
        try:
            # Use DataProcessor for improved data loading
            q_df = DataProcessor.load_data_csv(
                "data/identification_q_simulation.csv"
            )
            tau_df = DataProcessor.load_data_csv(
                "data/identification_tau_simulation.csv"
            )
            
            q_raw = q_df.values  # Convert to numpy array
            tau_raw = tau_df.values  # Convert to numpy array
            
            print(f"Loaded {len(q_raw)} samples from CSV files")
            
            # Limit samples if needed
            max_samples = min(len(q_raw),
                             self.identif_config.get("nb_samples", 100))
            q_raw = q_raw[:max_samples, :]
            tau_raw = tau_raw[:max_samples, :]
            
            # Apply data filtering if available
            if hasattr(DataProcessor, 'apply_lowpass_filter'):
                q_raw = DataProcessor.apply_lowpass_filter(
                    q_raw, cutoff=10.0, fs=100.0
                )
            
            # Calculate derivatives using FIGAROH function
            q_filtered, dq_filtered, ddq_filtered = \
                calculate_first_second_order_differentiation(
                    self.model, q_raw, self.identif_config
                )
            
            # Create time vector (assuming 100Hz sampling)
            dt = 0.01  # 100Hz
            time_vector = np.arange(len(q_filtered)) * dt
            
            print(f"Processed trajectory data: {len(q_filtered)} samples")
            
            return {
                "timestamps": time_vector.reshape(-1, 1),
                "positions": q_filtered,
                "velocities": dq_filtered,
                "accelerations": ddq_filtered,
                "torques": tau_raw[:len(q_filtered)]  # Match length
            }
            
        except Exception as e:
            raise IdentificationError(
                f"Failed to load UR10 trajectory data: {e}"
            )


class UR10OptimalCalibration(BaseOptimalCalibration):
    """UR10-specific optimal configuration generation for calibration."""
    
    def __init__(self, robot, config_file="config/ur10_config.yaml"):
        """Initialize UR10 optimal calibration with robot model and configuration.
        
        Args:
            robot: UR10 robot model loaded with FIGAROH
            config_file: Path to UR10 configuration YAML file
        """
        super().__init__(robot, config_file)
        print("UR10 Optimal Calibration initialized")
    
    def load_candidate_configurations(self):
        """Load candidate joint configurations from external data files.
        
        For UR10, this method first attempts to load from specified files,
        and if no candidate configurations exist, generates random configurations
        automatically within joint limits.
        """
        from figaroh.calibration.calibration_tools import get_idxq_from_jname
        
        if self._sampleConfigs_file is None:
            print("No sample configurations file specified, generating random configurations")
            self._generate_random_configurations()
            return
        
        # Try to load from specified file
        try:
            if "csv" in self._sampleConfigs_file:
                # Load from CSV file
                self.q_measured, _ = load_data(
                    self._sampleConfigs_file, self.model, self.calib_config, []
                )
                # Update sample count
                self.calib_config["NbSample"] = len(self.q_measured)
                print(f"Loaded {len(self.q_measured)} configurations from CSV file")
                
            elif "yaml" in self._sampleConfigs_file:
                # Load from YAML file
                with open(self._sampleConfigs_file, "r") as f:
                    configs_data = yaml.load(f, Loader=SafeLoader)
                
                joint_names = configs_data["calibration_joint_names"]
                joint_configs = configs_data["calibration_joint_configurations"]
                
                # Convert to numpy array and map to model joint indices
                idxq = get_idxq_from_jname(self.model, joint_names)
                q_configs = np.array(joint_configs)
                
                # Create full configuration array
                self.q_measured = np.zeros((len(joint_configs), self.model.nq))
                self.q_measured[:, idxq] = q_configs
                
                # Store configurations for later use
                self._configs = configs_data
                
                # Update sample count
                self.calib_config["NbSample"] = len(self.q_measured)
                print(f"Loaded {len(self.q_measured)} configurations from YAML file")
                
            else:
                raise ValueError(f"Unsupported file format: {self._sampleConfigs_file}")
                
        except (FileNotFoundError, KeyError, ValueError) as e:
            print(f"Failed to load candidate configurations: {e}")
            print("Generating random configurations instead")
            self._generate_random_configurations()
    
    def _generate_random_configurations(self):
        """Generate random joint configurations for UR10 within joint limits."""
        print("Generating random configurations for UR10...")
        
        # UR10 joint limits (conservative values in radians)
        # Joint limits for UR10: approximately ±2π for most joints
        q_min = np.array([-2*np.pi, -2*np.pi, -np.pi, -2*np.pi, -2*np.pi, -2*np.pi])
        q_max = np.array([2*np.pi, 2*np.pi, np.pi, 2*np.pi, 2*np.pi, 2*np.pi])
        
        # Generate random configurations
        n_samples = 500  # Default 100 samples
        self.q_measured = np.random.uniform(
            low=q_min, high=q_max, size=(n_samples, len(q_min))
        )
        
        # Update configuration with actual sample count
        self.calib_config["NbSample"] = len(self.q_measured)
        
        # Create _configs attribute to store configuration data for base class compatibility
        joint_names = [f"joint_{i+1}" for i in range(len(q_min))]
        self._configs = {
            "calibration_joint_names": joint_names,
            "calibration_joint_configurations": self.q_measured.tolist()
        }
        
        print(f"Generated {len(self.q_measured)} random configurations")


class UR10OptimalTrajectory:
    """UR10-specific optimal trajectory generation for dynamic identification."""
    
    def __init__(self, robot, config_file="config/ur10_config.yaml"):
        """Initialize UR10 optimal trajectory with robot model and configuration.
        
        Args:
            robot: UR10 robot model loaded with FIGAROH
            config_file: Path to UR10 configuration YAML file
        """
        self.robot = robot
        self.model = robot.model
        self.data = robot.data
        
        # Load configuration
        with open(config_file, "r") as f:
            config = yaml.load(f, Loader=SafeLoader)
        
        self.config = config
        self.identif_data = self.config["identification"]
        self.identif_config = get_identification_param_from_yaml(robot, self.identif_data)
        
        # Trajectory parameters
        self.n_waypoints = 10
        self.trajectory_duration = 10.0
        self.dt = 0.01
        
        print("UR10 Optimal Trajectory initialized")
    
    def generate_base_parameters(self):
        """Generate base parameters for trajectory optimization."""
        # Generate random samples for structural analysis
        q_rand = np.random.uniform(
            low=-np.pi, high=np.pi, size=(100, self.model.nq)
        )
        dq_rand = np.random.uniform(
            low=-1, high=1, size=(100, self.model.nv)
        )
        ddq_rand = np.random.uniform(
            low=-1, high=1, size=(100, self.model.nv)
        )
        
        # Build regressor matrix
        W = build_regressor_basic(self.robot, q_rand, dq_rand, ddq_rand, self.identif_config)
        
        # Create dummy standard parameters
        num_standard_params = W.shape[1]
        params_standard = {f"param_{i}": 0.1 for i in range(num_standard_params)}
        
        # Remove zero columns and build reduced regressor
        idx_e, params_r = get_index_eliminate(W, params_standard, 1e-6)
        W_e = build_regressor_reduced(W, idx_e)
        
        # Calculate base regressor and base parameters
        _, params_base, idx_base = get_baseParams(W_e, params_r, params_standard)
        
        self.params_base = params_base
        self.idx_base = idx_base
        
        print(f"Generated {len(params_base)} base parameters")
    
    def cubic_spline_trajectory(self, waypoints, total_time):
        """Generate cubic spline trajectory from waypoints.
        
        Args:
            waypoints: Array of joint waypoints (n_waypoints x n_joints)
            total_time: Total trajectory duration
            
        Returns:
            q, dq, ddq: Position, velocity, acceleration trajectories
        """
        from scipy.interpolate import CubicSpline
        
        n_points = int(total_time / self.dt)
        t = np.linspace(0, total_time, len(waypoints))
        t_eval = np.linspace(0, total_time, n_points)
        
        trajectories = []
        velocities = []
        accelerations = []
        
        for joint in range(self.model.nq):
            cs = CubicSpline(t, waypoints[:, joint])
            trajectories.append(cs(t_eval))
            velocities.append(cs(t_eval, nu=1))
            accelerations.append(cs(t_eval, nu=2))
        
        q = np.array(trajectories).T
        dq = np.array(velocities).T
        ddq = np.array(accelerations).T
        
        return q, dq, ddq
    
    def check_constraints(self, q, dq, ddq):
        """Check if trajectory satisfies joint and dynamic constraints.
        
        Args:
            q, dq, ddq: Trajectory arrays
            
        Returns:
            bool: True if constraints are satisfied
        """
        # Joint limits (simplified - using conservative limits)
        q_min = -np.pi * np.ones(self.model.nq)
        q_max = np.pi * np.ones(self.model.nq)
        
        if np.any(q < q_min) or np.any(q > q_max):
            return False
        
        # Velocity limits
        dq_max = 2.0 * np.ones(self.model.nv)
        if np.any(np.abs(dq) > dq_max):
            return False
        
        # Acceleration limits
        ddq_max = 5.0 * np.ones(self.model.nv)
        if np.any(np.abs(ddq) > ddq_max):
            return False
        
        return True
    
    def evaluate_trajectory_quality(self, q, dq, ddq):
        """Evaluate the quality of a trajectory for parameter identification.
        
        Args:
            q, dq, ddq: Trajectory arrays
            
        Returns:
            float: Condition number of the regressor matrix
        """
        # Build regressor matrix for this trajectory
        W = build_regressor_basic(self.robot, q, dq, ddq, self.identif_config)
        
        # Select base parameters
        W_base = W[:, self.idx_base]
        
        # Calculate condition number
        cond_num = np.linalg.cond(W_base)
        
        return cond_num
    
    def solve(self, n_iterations=50):
        """Generate optimal trajectory for UR10 identification.
        
        Args:
            n_iterations: Number of optimization iterations
            
        Returns:
            dict: Optimal trajectory data
        """
        print("Generating optimal trajectory for UR10 identification...")
        
        # Generate base parameters
        self.generate_base_parameters()
        
        best_trajectory = None
        best_condition_number = float('inf')
        
        for i in range(n_iterations):
            # Generate random waypoints within joint limits
            waypoints = np.random.uniform(
                low=-np.pi, high=np.pi, 
                size=(self.n_waypoints, self.model.nq)
            )
            
            # Generate trajectory
            q, dq, ddq = self.cubic_spline_trajectory(waypoints, self.trajectory_duration)
            
            # Check constraints
            if not self.check_constraints(q, dq, ddq):
                continue
            
            # Evaluate quality
            cond_num = self.evaluate_trajectory_quality(q, dq, ddq)
            
            if cond_num < best_condition_number:
                best_condition_number = cond_num
                best_trajectory = {
                    'q': q, 'dq': dq, 'ddq': ddq,
                    'waypoints': waypoints,
                    'condition_number': cond_num
                }
            
            if i % 10 == 0:
                print(f"Iteration {i}: best condition number = {best_condition_number:.2e}")
        
        self.optimal_trajectory = best_trajectory
        print(f"Optimal trajectory found with condition number: {best_condition_number:.2e}")
        
        return best_trajectory
    
    def plot_results(self):
        """Plot optimal trajectory results."""
        if not hasattr(self, 'optimal_trajectory'):
            print("No optimal trajectory results to plot. Run solve() first.")
            return
        
        traj = self.optimal_trajectory
        time_vector = np.arange(len(traj['q'])) * self.dt
        
        # Plot joint trajectories
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))
        
        # Position
        axes[0].plot(time_vector, traj['q'])
        axes[0].set_ylabel('Joint Position (rad)')
        axes[0].set_title('UR10 Optimal Trajectory - Joint Positions')
        axes[0].grid(True)
        axes[0].legend([f'Joint {i+1}' for i in range(self.model.nq)])
        
        # Velocity
        axes[1].plot(time_vector, traj['dq'])
        axes[1].set_ylabel('Joint Velocity (rad/s)')
        axes[1].set_title('Joint Velocities')
        axes[1].grid(True)
        
        # Acceleration
        axes[2].plot(time_vector, traj['ddq'])
        axes[2].set_ylabel('Joint Acceleration (rad/s²)')
        axes[2].set_xlabel('Time (s)')
        axes[2].set_title('Joint Accelerations')
        axes[2].grid(True)
        
        plt.tight_layout()
        plt.show()
        
        print(f"Trajectory condition number: {traj['condition_number']:.2e}")
    
    def save_results(self, output_dir="results"):
        """Save optimal trajectory results."""
        if not hasattr(self, 'optimal_trajectory'):
            print("No optimal trajectory results to save. Run solve() first.")
            return
        
        os.makedirs(output_dir, exist_ok=True)
        
        traj = self.optimal_trajectory
        
        # Save to YAML
        results_dict = {
            'condition_number': float(traj['condition_number']),
            'waypoints': traj['waypoints'].tolist(),
            'trajectory_duration': self.trajectory_duration,
            'n_waypoints': self.n_waypoints,
            'sampling_time': self.dt
        }
        
        with open(os.path.join(output_dir, "ur10_optimal_trajectory.yaml"), "w") as f:
            yaml.dump(results_dict, f, default_flow_style=False)
        
        # Save trajectory data to CSV
        time_vector = np.arange(len(traj['q'])) * self.dt
        df = pd.DataFrame({
            'time': time_vector,
            **{f'q{i+1}': traj['q'][:, i] for i in range(self.model.nq)},
            **{f'dq{i+1}': traj['dq'][:, i] for i in range(self.model.nv)},
            **{f'ddq{i+1}': traj['ddq'][:, i] for i in range(self.model.nv)},
        })
        df.to_csv(os.path.join(output_dir, "ur10_optimal_trajectory.csv"), index=False)
        
        print(f"Results saved to {output_dir}/ur10_optimal_trajectory.yaml and .csv")
