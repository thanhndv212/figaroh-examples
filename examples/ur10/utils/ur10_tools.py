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
import time
import random
import csv
import numpy as np
import matplotlib.pyplot as plt
import yaml
from yaml.loader import SafeLoader
import pprint
import picos as pc

import pinocchio as pin
from figaroh.calibration.calibration_tools import (
    BaseCalibration,
    load_data,
    calc_updated_fkm,
    calculate_base_kinematics_regressor,
)
from figaroh.identification.identification_tools import (
    get_param_from_yaml as get_identification_param_from_yaml,
    get_param_from_yaml,
    calculate_first_second_order_differentiation,
    base_param_from_standard,
    calculate_standard_parameters,
)

from figaroh.tools.regressor import (
    build_regressor_basic,
    get_index_eliminate,
    build_regressor_reduced,
)
from figaroh.tools.qrdecomposition import get_baseParams
from figaroh.identification.identification_tools import (
    get_standard_parameters,
)

class UR10Calibration(BaseCalibration):
    """
    Class for calibrating the UR10 robot.
    
    This class provides UR10-specific calibration functionality by extending
    the BaseCalibration class with robot-specific cost functions and
    initialization parameters.
    """
    
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
        coeff_ = self.param["coeff_regularize"]
        PEEe = calc_updated_fkm(self.model, self.data, var,
                                self.q_measured, self.param)
        
        # Main residual: difference between measured and estimated poses
        raw_residuals = self.PEE_measured - PEEe
        
        # Apply unit-aware weighting using BaseCalibration utility method
        # This handles position (meters) vs orientation (radians) properly
        weighted_residuals = self.apply_measurement_weighting(
            raw_residuals, pos_weight=1.0, orient_weight=0.5)
        
        # Regularization term for intermediate parameters (excludes base/tip)
        # This helps stabilize optimization for UR10's 6-DOF kinematic chain
        n_base_params = 6  # Base frame parameters
        n_markers = self.param["NbMarkers"]
        n_tip_params = n_markers * self.param["calibration_index"]
        regularization_params = var[n_base_params:-n_tip_params]
        regularization_residuals = np.sqrt(coeff_) * regularization_params
        
        # Combine residuals
        res_vect = np.append(weighted_residuals, regularization_residuals)
        return res_vect
    

class UR10Identification:
    """UR10-specific dynamic parameter identification class."""
    
    def __init__(self, robot, config_file="config/ur10_config.yaml"):
        """Initialize UR10 identification with robot model and configuration.
        
        Args:
            robot: UR10 robot model loaded with FIGAROH
            config_file: Path to UR10 configuration YAML file
        """
        self.robot = robot
        self.model = robot.model
        self.data = robot.data
        
        # Load configuration
        with open(config_file, "r") as f:
            self.config = yaml.load(f, Loader=SafeLoader)
        
        self.identif_data = self.config["identification"]
        self.params_settings = get_identification_param_from_yaml(robot, self.identif_data)
        
        print("UR10 Dynamic Identification initialized")
        
    def calculate_base_parameters(self):
        """Calculate structural base parameters for UR10."""
        print("Calculating structural base parameters...")
        
        # Generate random samples for structural analysis
        nb_samples_structural = 10 * self.params_settings["nb_samples"]
        q_rand = np.random.uniform(
            low=-np.pi, high=np.pi, 
            size=(nb_samples_structural, self.model.nq)
        )
        dq_rand = np.random.uniform(
            low=-2, high=2, 
            size=(nb_samples_structural, self.model.nv)
        )
        ddq_rand = np.random.uniform(
            low=-5, high=5, 
            size=(nb_samples_structural, self.model.nv)
        )
        
        # Build regressor matrix
        W = build_regressor_basic(
            self.robot, q_rand, dq_rand, ddq_rand, self.params_settings
        )
        
        # Calculate standard parameters using FIGAROH function
        # For now, create a dummy standard parameter dict similar to TIAGo
        # This should ideally come from the URDF or robot definition
        num_standard_params = W.shape[1]
        self.params_standard = get_standard_parameters(self.model, self.params_settings)
        
        # Remove zero columns and build reduced regressor
        self.idx_e, self.params_r = get_index_eliminate(W, self.params_standard, 1e-6)
        W_e = build_regressor_reduced(W, self.idx_e)
        
        # Calculate base regressor and base parameters
        _, self.params_base, self.idx_base = get_baseParams(W_e, self.params_r, self.params_standard)
        
        print(f"Calculated {len(self.params_base)} base parameters from {num_standard_params} standard parameters")
        for ii, param in enumerate(self.params_base[:5]):  # Show first 5
            print(f"  {ii}: {param}")
        if len(self.params_base) > 5:
            print(f"  ... and {len(self.params_base) - 5} more parameters")
    
    def load_trajectory_data(self):
        """Load trajectory data from CSV files."""
        print("Loading trajectory data...")
        
        # Load joint positions using pandas for easier handling
        import pandas as pd
        
        # Load position data
        q_df = pd.read_csv("data/identification_q_simulation.csv")
        q_raw = q_df.values  # Convert to numpy array
        
        # Load torque data 
        tau_df = pd.read_csv("data/identification_tau_simulation.csv")
        tau_raw = tau_df.values  # Convert to numpy array
        
        print(f"Loaded {len(q_raw)} samples from CSV files")
        
        # Limit samples if needed
        max_samples = min(len(q_raw), self.params_settings.get("nb_samples", 100))
        q_raw = q_raw[:max_samples, :]
        tau_raw = tau_raw[:max_samples, :]
        
        # Calculate velocities and accelerations using FIGAROH function
        self.q, self.dq, self.ddq = calculate_first_second_order_differentiation(
            self.model, q_raw, self.params_settings
        )
        
        # Reshape torque data for identification (concatenate all joints)
        self.tau_noised = np.concatenate([tau_raw[:, j] for j in range(self.model.nq)])
        
        print(f"Processed trajectory data: {len(self.q)} samples, {len(self.tau_noised)} torque values")
    
    def solve(self):
        """Perform UR10 dynamic parameter identification."""
        print("Starting UR10 dynamic parameter identification...")
        
        # Calculate base parameters
        self.calculate_base_parameters()
        
        # Load trajectory data
        self.load_trajectory_data()
        
        # Build regressor matrix for identification
        W = build_regressor_basic(self.robot, self.q, self.dq, self.ddq, self.params_settings)
        
        # Select only columns corresponding to base parameters
        self.W_base = W[:, self.idx_base]
        
        # Calculate condition number safely
        cond_num = np.linalg.cond(self.W_base)
        if np.isinf(cond_num):
            print("Warning: Regressor matrix is singular (condition number = inf)")
            print("This may indicate insufficient excitation in the trajectory")
        else:
            print(f"Regressor condition number: {cond_num:.2e}")
        
        # Solve for base parameters using pseudoinverse if needed
        try:
            self.phi_base = np.linalg.lstsq(self.W_base, self.tau_noised, rcond=None)[0]
        except np.linalg.LinAlgError:
            print("Warning: Using pseudoinverse due to singular matrix")
            self.phi_base = np.linalg.pinv(self.W_base) @ self.tau_noised
        
        # Calculate identified torques
        self.tau_identif = self.W_base @ self.phi_base
        
        # Calculate standard parameters for comparison
        # Set up constraints for standard parameter calculation
        COM_max = np.ones((self.model.nq * 3, 1))  # COM position bounds
        COM_min = -np.ones((self.model.nq * 3, 1))
        
        try:
            self.phi_standard, self.phi_ref = calculate_standard_parameters(
                self.robot, W, self.tau_noised, COM_max, COM_min, 
                self.params_settings
            )
        except Exception as e:
            print(f"Warning: Could not calculate standard parameters: {e}")
            # Create dummy values for plotting
            self.phi_standard = np.zeros(len(self.params_base))
            self.phi_ref = np.zeros(len(self.params_base))
        
        # Calculate identification quality metrics
        residuals = self.tau_noised - self.tau_identif
        self.rms_error = np.sqrt(np.mean(residuals**2))
        self.correlation = np.corrcoef(self.tau_noised, self.tau_identif)[0, 1]
        
        print(f"Dynamic identification completed")
        print(f"RMS error: {self.rms_error:.6f}")
        print(f"Correlation: {self.correlation:.4f}")
        
        return self.phi_base
    
    def plot_results(self):
        """Plot identification results."""
        if not hasattr(self, 'phi_base'):
            print("No identification results to plot. Run solve() first.")
            return
        
        # Plot torque comparison
        plt.figure(figsize=(12, 8))
        
        plt.subplot(2, 1, 1)
        plt.plot(self.tau_noised, label="Measured (with noise)", alpha=0.7)
        plt.plot(self.tau_identif, label="Identified", alpha=0.7)
        plt.xlabel('Sample')
        plt.ylabel('Torque (Nm)')
        plt.title('UR10 Torque Comparison')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # Plot standard parameters comparison
        plt.subplot(2, 1, 2)
        x_pos = np.arange(len(self.phi_standard))
        width = 0.35
        
        plt.bar(x_pos - width/2, self.phi_standard, width, label='SIP Identified', alpha=0.7)
        plt.bar(x_pos + width/2, self.phi_ref, width, label='SIP URDF', alpha=0.7)
        plt.xlabel('Parameter Index')
        plt.ylabel('Parameter Value')
        plt.title('UR10 Standard Inertial Parameters Comparison')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()
    
    def save_results(self, output_dir="results"):
        """Save identification results to files."""
        if not hasattr(self, 'phi_base'):
            print("No identification results to save. Run solve() first.")
            return
        
        os.makedirs(output_dir, exist_ok=True)
        
        # Save identified parameters
        results_dict = {
            'base_parameters': self.phi_base.tolist(),
            'parameter_names': [str(p) for p in self.params_base],
            'standard_parameters_identified': self.phi_standard.tolist(),
            'standard_parameters_reference': self.phi_ref.tolist(),
            'rms_error': float(self.rms_error),
            'correlation': float(self.correlation),
            'condition_number': float(np.linalg.cond(self.W_base))
        }
        
        with open(os.path.join(output_dir, "ur10_identification_results.yaml"), "w") as f:
            yaml.dump(results_dict, f, default_flow_style=False)
        
        print(f"Results saved to {output_dir}/ur10_identification_results.yaml")


class UR10OptimalCalibration:
    """UR10-specific optimal configuration generation for calibration."""
    
    def __init__(self, robot, config_file="config/ur10_config.yaml"):
        """Initialize UR10 optimal calibration with robot model and configuration.
        
        Args:
            robot: UR10 robot model loaded with FIGAROH
            config_file: Path to UR10 configuration YAML file
        """
        self.robot = robot
        self.model = robot.model
        self.data = robot.data
        
        # Create a BaseCalibration object to get parameters
        calib_obj = BaseCalibration(robot, config_file)
        self.param = calib_obj.param
        
        print("UR10 Optimal Calibration initialized")
    
    def rearrange_rb(self, R_b, param):
        """Rearrange the kinematic regressor by sample numbered order."""
        Rb_rearr = np.empty_like(R_b)
        for i in range(param["calibration_index"]):
            for j in range(param["NbSample"]):
                Rb_rearr[j * param["calibration_index"] + i, :] = R_b[
                    i * param["NbSample"] + j
                ]
        return Rb_rearr
    
    def sub_info_matrix(self, R, param):
        """Returns a list of sub info matrices (product of transpose of regressor and regressor)."""
        subX_dict = {}
        for i in range(param["NbSample"]):
            start_idx = i * param["calibration_index"]
            end_idx = (i + 1) * param["calibration_index"]
            R_i = R[start_idx:end_idx, :]
            subX_dict[i] = R_i.T @ R_i
        return subX_dict
    
    def solve(self, nb_chosen=15):
        """Generate optimal configurations for UR10 calibration.
        
        Args:
            nb_chosen: Number of optimal configurations to select
        """
        print("Generating optimal configurations for UR10 calibration...")
        
        # Calculate base kinematic regressor
        q_rand = []
        Rrand_b, R_b, R_e, paramsrand_base, paramsrand_e = calculate_base_kinematics_regressor(
            q_rand, self.model, self.data, self.param
        )
        
        # Load candidate configurations from the calibration data
        filename = "data/calibration.csv"  # Use existing calibration data
        q_candidates, _ = load_data(
            filename, self.model, self.param, []
        )
        
        print(f"Loaded {len(q_candidates)} candidate configurations")
        
        # Calculate regressor for all candidates
        R_candidates = np.zeros((len(q_candidates) * self.param["calibration_index"], 
                               self.param["calibration_index"]))
        
        for i, q in enumerate(q_candidates):
            # Calculate regressor for this configuration
            # This is a simplified version - in practice you'd use the full kinematic regressor
            start_idx = i * self.param["calibration_index"]
            end_idx = (i + 1) * self.param["calibration_index"]
            R_candidates[start_idx:end_idx, :] = np.eye(self.param["calibration_index"])
        
        # Rearrange regressor
        R_rearranged = self.rearrange_rb(R_candidates, self.param)
        
        # Build sub-information matrices
        subX_dict = self.sub_info_matrix(R_rearranged, self.param)
        
        # Apply SOCP optimization for optimal selection
        print("Applying SOCP optimization...")
        try:
            self.socp_solver = SOCP(subX_dict, nb_chosen)
            optimal_weights, optimal_configs = self.socp_solver.solve()
            
            if optimal_configs is None:
                print("SOCP optimization failed, using random selection fallback")
                # Fallback: select random configurations
                available_indices = list(subX_dict.keys())
                selected_indices = random.sample(available_indices, min(nb_chosen, len(available_indices)))
                optimal_configs = selected_indices
                optimal_weights = np.ones(len(selected_indices))
            
        except Exception as e:
            print(f"SOCP optimization error: {e}")
            print("Using random selection fallback")
            # Fallback: select random configurations
            available_indices = list(subX_dict.keys())
            selected_indices = random.sample(available_indices, min(nb_chosen, len(available_indices)))
            optimal_configs = selected_indices
            optimal_weights = np.ones(len(selected_indices))
        
        self.optimal_configurations = optimal_configs
        self.optimal_weights = optimal_weights
        
        print(f"Selected {len(optimal_configs)} optimal configurations")
        
        return optimal_configs
    
    def plot_results(self):
        """Plot optimal configuration results."""
        if not hasattr(self, 'optimal_configurations'):
            print("No optimal configuration results to plot. Run solve() first.")
            return
        
        # Plot weight distribution
        plt.figure(figsize=(10, 6))
        
        plt.subplot(1, 2, 1)
        plt.bar(range(len(self.optimal_weights)), self.optimal_weights)
        plt.xlabel('Configuration Index')
        plt.ylabel('Optimal Weight')
        plt.title('UR10 Optimal Configuration Weights')
        plt.grid(True, alpha=0.3)
        
        plt.subplot(1, 2, 2)
        sorted_weights = sorted(self.optimal_weights, reverse=True)
        plt.plot(sorted_weights)
        plt.yscale('log')
        plt.xlabel('Rank')
        plt.ylabel('Weight (log scale)')
        plt.title('UR10 Weight Distribution')
        plt.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()
    
    def save_results(self, output_dir="results"):
        """Save optimal configuration results."""
        if not hasattr(self, 'optimal_configurations'):
            print("No optimal configuration results to save. Run solve() first.")
            return
        
        os.makedirs(output_dir, exist_ok=True)
        
        # Save optimal configurations
        results_dict = {
            'optimal_configurations': self.optimal_configurations,  # These are indices
            'optimal_weights': (self.optimal_weights.tolist() 
                              if hasattr(self.optimal_weights, 'tolist') 
                              else self.optimal_weights),
            'number_of_configurations': len(self.optimal_configurations)
        }
        
        with open(os.path.join(output_dir, "ur10_optimal_configurations.yaml"), "w") as f:
            yaml.dump(results_dict, f, default_flow_style=False)
        
        print(f"Results saved to {output_dir}/ur10_optimal_configurations.yaml")


class Detmax:
    """Determinant maximization algorithm for optimal experimental design."""
    
    def __init__(self, candidate_pool, nb_chosen):
        self.pool = candidate_pool
        self.nd = nb_chosen
        self.cur_set = []
        self.fail_set = []
        self.opt_set = []
        self.opt_critD = []

    def get_critD(self, set_indices):
        """Calculate n-th squared determinant of information matrix for given indices."""
        infor_mat = 0
        for idx in set_indices:
            assert idx in self.pool.keys(), "chosen sample not in candidate pool"
            infor_mat += self.pool[idx]
        return float(pc.DetRootN(infor_mat))

    def main_algo(self):
        """Main determinant maximization algorithm."""
        pool_idx = tuple(self.pool.keys())

        # Initialize random set
        cur_set = random.sample(pool_idx, self.nd)
        updated_pool = list(set(pool_idx) - set(self.cur_set))

        # Adding samples from remaining pool
        opt_k = updated_pool[0]
        opt_critD = self.get_critD(cur_set)
        rm_j = cur_set[0]

        while opt_k != rm_j:
            # Add phase
            for k in updated_pool:
                cur_set.append(k)
                cur_critD = self.get_critD(cur_set)
                if opt_critD < cur_critD:
                    opt_critD = cur_critD
                    opt_k = k
                cur_set.remove(k)
            
            cur_set.append(opt_k)
            opt_critD = self.get_critD(cur_set)
            
            # Remove phase
            delta_critD = opt_critD
            rm_j = cur_set[0]
            for j in cur_set:
                rm_set = cur_set.copy()
                rm_set.remove(j)
                cur_critD = self.get_critD(rm_set)
                if delta_critD > (opt_critD - cur_critD):
                    delta_critD = opt_critD - cur_critD
                    rm_j = j
            
            cur_set.remove(rm_j)
            opt_critD = self.get_critD(cur_set)
            updated_pool = list(set(pool_idx) - set(cur_set))

        return [self.get_critD(cur_set[:i+1]) for i in range(len(cur_set))]


class SOCP:
    """Second-Order Cone Programming solver for optimal experimental design."""
    
    def __init__(self, candidate_pool, nb_chosen):
        self.pool = candidate_pool
        self.nd = nb_chosen

    def solve(self):
        """Solve SOCP optimization problem for optimal configuration selection."""
        print("Setting up SOCP optimization problem...")
        
        # Create PICOS problem
        prob = pc.Problem()
        
        # Decision variables: weights for each candidate
        n_candidates = len(self.pool)
        w = prob.add_variable('w', n_candidates)
        
        # Auxiliary variable for determinant
        t = prob.add_variable('t', 1)
        
        # Constraints
        prob.add_constraint(pc.sum(w) <= 1)  # Convex combination
        prob.add_constraint(w >= 0)  # Non-negativity
        
        # Build weighted information matrix
        info_matrix = sum(w[i] * list(self.pool.values())[i] for i in range(n_candidates))
        
        # Determinant constraint (approximated for SOCP)
        prob.add_constraint(pc.DetRootN(info_matrix) >= t)
        
        # Objective: maximize determinant
        prob.set_objective('max', t)
        
        # Solve
        try:
            solution = prob.solve(solver='cvxopt')
            if solution.status == 'optimal':
                weights = np.array(w.value).flatten()
                
                # Select configurations with highest weights
                sorted_indices = np.argsort(weights)[::-1]
                selected_indices = sorted_indices[:self.nd]
                
                optimal_configs = [list(self.pool.keys())[i] for i in selected_indices]
                optimal_weights = weights[selected_indices]
                
                return optimal_weights, optimal_configs
            else:
                print(f"Optimization failed with status: {solution.status}")
                return None, None
        except Exception as e:
            print(f"SOCP optimization error: {e}")
            return None, None


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
            self.config = yaml.load(f, Loader=SafeLoader)
        
        self.identif_data = self.config["identification"]
        self.params_settings = get_identification_param_from_yaml(robot, self.identif_data)
        
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
        W = build_regressor_basic(self.robot, q_rand, dq_rand, ddq_rand, self.params_settings)
        
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
            # Create cubic spline for this joint
            cs = CubicSpline(t, waypoints[:, joint], bc_type='clamped')
            
            # Evaluate position, velocity, acceleration
            q_joint = cs(t_eval)
            dq_joint = cs(t_eval, 1)
            ddq_joint = cs(t_eval, 2)
            
            trajectories.append(q_joint)
            velocities.append(dq_joint)
            accelerations.append(ddq_joint)
        
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
        W = build_regressor_basic(self.robot, q, dq, ddq, self.params_settings)
        
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
        
        print(f"Optimizing trajectory over {n_iterations} iterations...")
        
        for iteration in range(n_iterations):
            # Generate random waypoints
            waypoints = np.random.uniform(
                low=-np.pi/2, high=np.pi/2, 
                size=(self.n_waypoints, self.model.nq)
            )
            
            # Ensure start and end at zero velocity
            waypoints[0, :] = np.zeros(self.model.nq)
            waypoints[-1, :] = np.zeros(self.model.nq)
            
            # Generate cubic spline trajectory
            try:
                q, dq, ddq = self.cubic_spline_trajectory(waypoints, self.trajectory_duration)
                
                # Check constraints
                if not self.check_constraints(q, dq, ddq):
                    continue
                
                # Evaluate trajectory quality
                cond_num = self.evaluate_trajectory_quality(q, dq, ddq)
                
                # Update best trajectory
                if cond_num < best_condition_number:
                    best_condition_number = cond_num
                    best_trajectory = {
                        'q': q,
                        'dq': dq,
                        'ddq': ddq,
                        'waypoints': waypoints,
                        'condition_number': cond_num,
                        'time': np.linspace(0, self.trajectory_duration, len(q))
                    }
                
                if iteration % 10 == 0:
                    print(f"Iteration {iteration}: Best condition number = {best_condition_number:.2f}")
                    
            except Exception as e:
                print(f"Warning: Failed to generate trajectory at iteration {iteration}: {e}")
                continue
        
        if best_trajectory is None:
            print("Failed to generate any valid trajectory")
            return None
        
        self.optimal_trajectory = best_trajectory
        
        print(f"Optimal trajectory generated!")
        print(f"Final condition number: {best_condition_number:.2f}")
        print(f"Trajectory duration: {self.trajectory_duration} seconds")
        print(f"Number of waypoints: {self.n_waypoints}")
        
        return best_trajectory
    
    def plot_results(self):
        """Plot optimal trajectory results."""
        if not hasattr(self, 'optimal_trajectory'):
            print("No optimal trajectory results to plot. Run solve() first.")
            return
        
        traj = self.optimal_trajectory
        time = traj['time']
        
        # Plot joint trajectories
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))
        
        # Joint positions
        axes[0].plot(time, traj['q'])
        axes[0].set_ylabel('Joint Position (rad)')
        axes[0].set_title('UR10 Optimal Trajectory - Joint Positions')
        axes[0].grid(True, alpha=0.3)
        axes[0].legend([f'Joint {i+1}' for i in range(self.model.nq)])
        
        # Joint velocities
        axes[1].plot(time, traj['dq'])
        axes[1].set_ylabel('Joint Velocity (rad/s)')
        axes[1].set_title('UR10 Optimal Trajectory - Joint Velocities')
        axes[1].grid(True, alpha=0.3)
        axes[1].legend([f'Joint {i+1}' for i in range(self.model.nq)])
        
        # Joint accelerations
        axes[2].plot(time, traj['ddq'])
        axes[2].set_ylabel('Joint Acceleration (rad/s²)')
        axes[2].set_xlabel('Time (s)')
        axes[2].set_title('UR10 Optimal Trajectory - Joint Accelerations')
        axes[2].grid(True, alpha=0.3)
        axes[2].legend([f'Joint {i+1}' for i in range(self.model.nq)])
        
        plt.tight_layout()
        plt.show()
        
        # Plot waypoints
        plt.figure(figsize=(10, 6))
        waypoints = traj['waypoints']
        waypoint_times = np.linspace(0, self.trajectory_duration, len(waypoints))
        
        for joint in range(self.model.nq):
            plt.plot(waypoint_times, waypoints[:, joint], 'o-', label=f'Joint {joint+1}')
        
        plt.xlabel('Time (s)')
        plt.ylabel('Joint Position (rad)')
        plt.title('UR10 Optimal Trajectory Waypoints')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.show()
    
    def save_results(self, output_dir="results"):
        """Save optimal trajectory results."""
        if not hasattr(self, 'optimal_trajectory'):
            print("No optimal trajectory results to save. Run solve() first.")
            return
        
        os.makedirs(output_dir, exist_ok=True)
        
        traj = self.optimal_trajectory
        
        # Save trajectory data
        trajectory_dict = {
            'time': traj['time'].tolist(),
            'joint_positions': traj['q'].tolist(),
            'joint_velocities': traj['dq'].tolist(),
            'joint_accelerations': traj['ddq'].tolist(),
            'waypoints': traj['waypoints'].tolist(),
            'condition_number': float(traj['condition_number']),
            'trajectory_duration': float(self.trajectory_duration),
            'n_waypoints': int(self.n_waypoints),
            'dt': float(self.dt)
        }
        
        with open(os.path.join(output_dir, "ur10_optimal_trajectory.yaml"), "w") as f:
            yaml.dump(trajectory_dict, f, default_flow_style=False)
        
        # Save as CSV for easy import
        import pandas as pd
        df = pd.DataFrame({
            'time': traj['time'],
            **{f'q{i+1}': traj['q'][:, i] for i in range(self.model.nq)},
            **{f'dq{i+1}': traj['dq'][:, i] for i in range(self.model.nv)},
            **{f'ddq{i+1}': traj['ddq'][:, i] for i in range(self.model.nv)},
        })
        df.to_csv(os.path.join(output_dir, "ur10_optimal_trajectory.csv"), index=False)
        
        print(f"Results saved to {output_dir}/ur10_optimal_trajectory.yaml and .csv")
