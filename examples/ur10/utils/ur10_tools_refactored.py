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
Example refactored UR10 tools using the new base classes.
This demonstrates how robot-specific implementations can inherit from
the generalized base classes while implementing robot-specific methods.
"""

import numpy as np
import pandas as pd
from .robot_base_tools import (
    BaseIdentification,
    BaseOptimalCalibration, 
    BaseOptimalTrajectory,
    DetMax,
    SOCP
)
from figaroh.identification.identification_tools import get_standard_parameters
from figaroh.identification.identification_tools import (
    calculate_first_second_order_differentiation,
)
from figaroh.calibration.calibration_tools import (
    BaseCalibration,
    load_data,
)
from figaroh.tools.regressor import build_regressor_basic


class UR10Calibration(BaseCalibration):
    """UR10-specific calibration class extending BaseCalibration."""
    
    def __init__(self, robot, config_file="config/ur10_config.yaml"):
        """Initialize UR10 calibration with robot model and configuration."""
        super().__init__(robot, config_file)
        print(f"UR10 Calibration initialized")
        print(f"Total calibrating parameters: {len(self.param['param_name'])}")
    
    # Robot-specific implementations of abstract methods from BaseCalibration
    def cost_function(self, var):
        """UR10-specific cost function implementation."""
        # Same as before - robot-specific cost function logic
        pass
    
    def solve_optimisation(self):
        """UR10-specific optimization implementation."""
        # Same as before - robot-specific optimization logic
        pass


class UR10Identification(BaseIdentification):
    """UR10-specific dynamic parameter identification class."""
    
    def get_standard_parameters(self):
        """Get standard parameters for UR10 robot."""
        return get_standard_parameters(self.model, self.identif_config)
    
    def load_trajectory_data(self):
        """Load trajectory data from UR10 CSV files."""
        print("Loading trajectory data...")
        
        # Load position data
        q_df = pd.read_csv("data/identification_q_simulation.csv")
        q_raw = q_df.values
        
        # Load torque data
        tau_df = pd.read_csv("data/identification_tau_simulation.csv")
        tau_raw = tau_df.values
        
        print(f"Loaded {len(q_raw)} samples from CSV files")
        
        # Limit samples if needed
        max_samples = min(len(q_raw), self.identif_config.get("nb_samples", 100))
        q_raw = q_raw[:max_samples, :]
        tau_raw = tau_raw[:max_samples, :]
        
        # Calculate velocities and accelerations using FIGAROH function
        self.q, self.dq, self.ddq = calculate_first_second_order_differentiation(
            self.model, q_raw, self.identif_config
        )
        
        # Reshape torque data for identification
        self.tau_noised = np.concatenate([tau_raw[:, j] for j in range(self.model.nq)])
        
        print(f"Processed trajectory data: {len(self.q)} samples")


class UR10OptimalCalibration(BaseOptimalCalibration):
    """UR10-specific optimal configuration generation for calibration."""
    
    def load_candidate_configurations(self):
        """Load candidate configurations from UR10 calibration data."""
        filename = "data/calibration.csv"
        q_candidates, _ = load_data(filename, self.model, self.param, [])
        return q_candidates
    
    def optimize_selection(self, subX_dict, nb_chosen):
        """Optimize configuration selection using SOCP for UR10."""
        try:
            # Try SOCP optimization first
            socp = SOCP(list(subX_dict.values()), nb_chosen)
            selected_indices, weights = socp.solve()
            
            # Map indices back to configurations
            optimal_configs = [list(subX_dict.keys())[i] for i in selected_indices]
            
            return optimal_configs, weights
            
        except Exception as e:
            print(f"SOCP failed: {e}, using determinant maximization")
            
            # Fallback to determinant maximization
            detmax = DetMax(list(subX_dict.values()), nb_chosen)
            selected_indices = detmax.main_algo()
            
            optimal_configs = [list(subX_dict.keys())[i] for i in selected_indices]
            weights = [1.0 if i in selected_indices else 0.0 for i in range(len(subX_dict))]
            
            return optimal_configs, weights


class UR10OptimalTrajectory(BaseOptimalTrajectory):
    """UR10-specific optimal trajectory generation for dynamic identification."""
    
    def get_base_parameter_indices(self):
        """Find base parameters for UR10 trajectory optimization."""
        # Generate random trajectory to find base parameters
        n_samples = 100
        q_rand = np.random.uniform(
            low=-np.pi, high=np.pi, 
            size=(n_samples, self.model.nq)
        )
        dq_rand = np.random.uniform(
            low=-2, high=2, 
            size=(n_samples, self.model.nv)
        )
        ddq_rand = np.random.uniform(
            low=-5, high=5, 
            size=(n_samples, self.model.nv)
        )
        
        W = build_regressor_basic(self.robot, q_rand, dq_rand, ddq_rand, self.identif_config)
        params_std = get_standard_parameters(self.model, self.identif_config)
        
        from figaroh.tools.regressor import get_index_eliminate, build_regressor_reduced
        from figaroh.tools.qrdecomposition import get_baseIndex
        
        idx_e, par_r = get_index_eliminate(W, params_std, tol_e=0.001)
        W_e = build_regressor_reduced(W, idx_e)
        idx_base = get_baseIndex(W_e, par_r)
        
        return idx_e, idx_base
    
    def generate_trajectory(self, waypoints, constraints):
        """Generate UR10 trajectory from waypoints using cubic splines."""
        # Simple linear interpolation for now
        # In practice, would use cubic splines or other trajectory generation
        n_points = 100
        time = np.linspace(0, constraints.get('total_time', 10.0), n_points)
        
        # Linear interpolation between waypoints
        positions = np.zeros((n_points, self.model.nq))
        for i in range(self.model.nq):
            positions[:, i] = np.interp(time, 
                                      np.linspace(0, constraints.get('total_time', 10.0), len(waypoints)),
                                      waypoints[:, i])
        
        # Calculate velocities and accelerations via numerical differentiation
        velocities = np.gradient(positions, time[1] - time[0], axis=0)
        accelerations = np.gradient(velocities, time[1] - time[0], axis=0)
        
        return time, positions, velocities, accelerations
    
    def evaluate_trajectory_quality(self, q, dq, ddq):
        """Evaluate UR10 trajectory quality using condition number."""
        W_b = self.build_base_regressor(q, dq, ddq)
        return np.linalg.cond(W_b)
    
    def generate_random_waypoints(self, n_waypoints):
        """Generate random waypoints for UR10 respecting joint limits."""
        waypoints = np.zeros((n_waypoints, self.model.nq))
        
        for i in range(n_waypoints):
            for j in range(self.model.nq):
                lb = self.model.lowerPositionLimit[j]
                ub = self.model.upperPositionLimit[j]
                # Add some margin to avoid limits
                margin = 0.1 * (ub - lb)
                waypoints[i, j] = np.random.uniform(lb + margin, ub - margin)
        
        return waypoints
    
    def get_default_constraints(self):
        """Get default trajectory constraints for UR10."""
        return {
            'total_time': 10.0,
            'max_velocity': 2.0,  # rad/s
            'max_acceleration': 5.0,  # rad/s^2
        }
