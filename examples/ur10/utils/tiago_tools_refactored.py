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
to use the generalized base classes.
"""

import numpy as np
import pandas as pd
from os.path import abspath
from .robot_base_tools import (
    BaseIdentification,
    BaseOptimalCalibration,
    BaseOptimalTrajectory,
    SOCP
)
from .dynamic_identification_utils import get_standard_parameters
from .simplified_colission_model import build_tiago_simplified
from figaroh.calibration.calibration_tools import BaseCalibration
from figaroh.utils.cubic_spline import CubicSpline, WaypointsGeneration


class TiagoCalibration(BaseCalibration):
    """TIAGo-specific calibration class extending BaseCalibration."""
    
    def __init__(self, robot, config_file, del_list=[]):
        """Initialize TIAGo calibration with robot model and configuration."""
        super().__init__(robot, config_file, del_list)
        print("TIAGo Calibration initialized")
    
    # TIAGo-specific implementations (same as before)
    def cost_function(self, var):
        """TIAGo-specific cost function implementation."""
        # Existing TIAGo cost function logic
        pass
    
    def solve_optimisation(self):
        """TIAGo-specific optimization implementation."""
        # Existing TIAGo optimization logic with outlier detection
        pass


class TiagoIdentification(BaseIdentification):
    """TIAGo-specific dynamic parameter identification class."""
    
    def get_standard_parameters(self):
        """Get standard parameters for TIAGo robot."""
        return get_standard_parameters(self.model, self.params_settings)
    
    def load_trajectory_data(self):
        """Load trajectory data from TIAGo CSV files."""
        print("Loading TIAGo trajectory data...")
        
        # Load CSV data using TIAGo-specific format
        ts = pd.read_csv(abspath(self.params_settings["pos_data"]), 
                        usecols=[0]).to_numpy()
        pos = pd.read_csv(abspath(self.params_settings["pos_data"]))
        vel = pd.read_csv(abspath(self.params_settings["vel_data"]))
        eff = pd.read_csv(abspath(self.params_settings["torque_data"]))
        
        # Extract relevant columns for active joints
        cols = {"pos": [], "vel": [], "eff": []}
        for jn in self.params_settings["active_joints"]:
            for col_type in cols.keys():
                if col_type == "pos":
                    cols[col_type].append(f"{jn}_position")
                elif col_type == "vel":
                    cols[col_type].append(f"{jn}_velocity")
                elif col_type == "eff":
                    cols[col_type].append(f"{jn}_effort")
        
        q_raw = pos[cols["pos"]].to_numpy()
        dq_raw = vel[cols["vel"]].to_numpy()
        tau_raw = eff[cols["eff"]].to_numpy()
        
        # Apply TIAGo-specific filtering
        q_filtered, dq_filtered = self.apply_filters(ts, q_raw, dq_raw)
        ddq_filtered = self.estimate_acceleration(ts, dq_filtered)
        
        # Build full configuration
        N_ = q_filtered.shape[0]
        self.q, self.dq, self.ddq = self.build_full_configuration(
            q_filtered, dq_filtered, ddq_filtered, N_
        )
        
        # Process torque data with TIAGo-specific corrections
        self.tau_noised = self.process_torque_data(tau_raw)
        
        print(f"Processed TIAGo trajectory data: {len(self.q)} samples")
    
    def apply_filters(self, t, q, dq, nbutter=4, f_butter=2, med_fil=5, f_sample=100):
        """Apply TIAGo-specific median and lowpass filters."""
        from scipy import signal
        b1, b2 = signal.butter(nbutter, f_butter / (f_sample / 2), "low")
        
        q_filtered = np.zeros(q.shape)
        dq_filtered = np.zeros(dq.shape)
        
        for j in range(dq.shape[1]):
            q_filtered[:, j] = signal.filtfilt(b1, b2, signal.medfilt(q[:, j], med_fil))
            dq_filtered[:, j] = signal.filtfilt(b1, b2, signal.medfilt(dq[:, j], med_fil))
        
        return q_filtered, dq_filtered
    
    def estimate_acceleration(self, t, dq_filtered):
        """Estimate acceleration from filtered velocity."""
        return np.array([
            np.gradient(dq_filtered[:, j]) / np.gradient(t[:, 0])
            for j in range(dq_filtered.shape[1])
        ]).T
    
    def build_full_configuration(self, q_f, dq_f, ddq_f, N_):
        """Build full configuration arrays for TIAGo."""
        p = np.tile(self.robot.q0, (N_, 1))
        v = np.tile(self.robot.v0, (N_, 1))
        a = np.tile(self.robot.v0, (N_, 1))
        
        p[:, self.params_settings["act_idxq"]] = q_f
        v[:, self.params_settings["act_idxv"]] = dq_f
        a[:, self.params_settings["act_idxv"]] = ddq_f
        
        return p, v, a
    
    def process_torque_data(self, tau):
        """Process torque data with TIAGo-specific motor constants."""
        # Apply TIAGo-specific torque processing (reduction ratios, etc.)
        tau_processed = tau.copy()
        
        # Apply motor constants and reduction ratios
        for i, joint_name in enumerate(self.params_settings["active_joints"]):
            # TIAGo-specific torque corrections would go here
            pass
        
        return tau_processed


class TiagoOptimalCalibration(BaseOptimalCalibration):
    """TIAGo-specific optimal configuration generation for calibration."""
    
    def load_candidate_configurations(self):
        """Load candidate configurations from TIAGo YAML file."""
        import yaml
        
        # Load from TIAGo-specific configuration file
        with open(self.param["sample_configs_file"], "r") as f:
            configs = yaml.load(f, Loader=yaml.SafeLoader)
        
        return np.array(configs["calibration_joint_configurations"])
    
    def optimize_selection(self, subX_dict, nb_chosen):
        """Optimize configuration selection using SOCP for TIAGo."""
        try:
            # Use SOCP optimization
            socp = SOCP(list(subX_dict.values()), nb_chosen)
            selected_indices, weights = socp.solve()
            
            # Map indices back to configurations
            optimal_configs = [list(subX_dict.keys())[i] for i in selected_indices]
            
            return optimal_configs, weights
            
        except Exception as e:
            print(f"SOCP optimization failed: {e}")
            # Fallback to random selection
            import random
            selected_indices = random.sample(list(subX_dict.keys()), nb_chosen)
            weights = [1.0 if i in selected_indices else 0.0 for i in subX_dict.keys()]
            
            return selected_indices, weights


class TiagoOptimalTrajectory(BaseOptimalTrajectory):
    """TIAGo-specific optimal trajectory generation for dynamic identification."""
    
    def __init__(self, robot, config_file, active_joints):
        """Initialize TIAGo optimal trajectory with active joints specification."""
        super().__init__(robot, config_file)
        self.active_joints = active_joints
        
        # Apply TIAGo-specific simplified collision model
        self.robot = build_tiago_simplified(self.robot)
    
    def get_base_parameter_indices(self):
        """Find base parameters for TIAGo trajectory optimization."""
        # Use TIAGo-specific approach with cubic splines
        n_wps_r = 100
        freq_r = 100
        CB_r = CubicSpline(self.robot, n_wps_r, self.active_joints)
        WP_r = WaypointsGeneration(self.robot, n_wps_r, self.active_joints)
        
        # Generate random waypoints
        soft_lim_pool = np.array([
            [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
            [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
            [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
        ])
        WP_r.gen_rand_pool(soft_lim_pool)
        
        wps_r, vel_wps_r, acc_wps_r = WP_r.gen_rand_wp()
        tps_r = np.matrix([0.5 * i for i in range(n_wps_r)]).transpose()
        t_r, p_r, v_r, a_r = CB_r.get_full_config(freq_r, tps_r, wps_r, vel_wps_r, acc_wps_r)
        
        # Build regressor and find base parameters
        from figaroh.tools.regressor import build_regressor_basic, get_index_eliminate, build_regressor_reduced
        from figaroh.tools.qrdecomposition import get_baseIndex
        
        W = build_regressor_basic(self.robot, p_r, v_r, a_r, self.params_settings)
        params_std = get_standard_parameters(self.model, self.params_settings)
        idx_e, par_r = get_index_eliminate(W, params_std, tol_e=0.001)
        W_e = build_regressor_reduced(W, idx_e)
        idx_base = get_baseIndex(W_e, par_r)
        
        return idx_e, idx_base
    
    def generate_trajectory(self, waypoints, constraints):
        """Generate TIAGo trajectory using cubic splines."""
        n_wps = len(waypoints)
        freq = constraints.get('frequency', 100)
        
        CB = CubicSpline(self.robot, n_wps, self.active_joints)
        
        # Create time points for waypoints
        total_time = constraints.get('total_time', n_wps * 0.5)
        tps = np.matrix([total_time * i / (n_wps - 1) for i in range(n_wps)]).transpose()
        
        # Use zero initial and final velocities/accelerations
        vel_wps = np.zeros_like(waypoints)
        acc_wps = np.zeros_like(waypoints)
        
        # Generate full configuration trajectory
        t, p, v, a = CB.get_full_config(freq, tps, waypoints.T, vel_wps.T, acc_wps.T)
        
        return t, p, v, a
    
    def evaluate_trajectory_quality(self, q, dq, ddq):
        """Evaluate TIAGo trajectory quality using condition number."""
        W_b = self.build_base_regressor(q, dq, ddq)
        return np.linalg.cond(W_b)
    
    def generate_random_waypoints(self, n_waypoints):
        """Generate random waypoints for TIAGo respecting joint limits."""
        waypoints = np.zeros((n_waypoints, len(self.active_joints)))
        
        for i in range(n_waypoints):
            for j, joint_idx in enumerate(self.active_joints):
                lb = self.model.lowerPositionLimit[joint_idx]
                ub = self.model.upperPositionLimit[joint_idx]
                # Add some margin to avoid limits
                margin = 0.1 * (ub - lb)
                waypoints[i, j] = np.random.uniform(lb + margin, ub - margin)
        
        return waypoints
    
    def get_default_constraints(self):
        """Get default trajectory constraints for TIAGo."""
        return {
            'total_time': 10.0,
            'frequency': 100,  # Hz
            'max_velocity': 1.5,  # rad/s
            'max_acceleration': 3.0,  # rad/s^2
        }
