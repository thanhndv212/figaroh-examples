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
Base classes for robot-specific tools that can be inherited by any robot type.
This module provides a generalized framework similar to BaseCalibration for:
- Dynamic Parameter Identification
- Optimal Configuration Generation
- Optimal Trajectory Generation

Each robot (TIAGo, UR10, MATE, etc.) can inherit from these base classes
and implement robot-specific methods while sharing common functionality.
"""

import os
import yaml
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from abc import ABC, abstractmethod
from yaml.loader import SafeLoader
from scipy.optimize import least_squares

# FIGAROH imports
from figaroh.identification.identification_tools import (
    get_param_from_yaml as get_identification_param_from_yaml,
    calculate_first_second_order_differentiation,
    calculate_standard_parameters,
)
from figaroh.tools.regressor import (
    build_regressor_basic,
    get_index_eliminate,
    build_regressor_reduced,
)
from figaroh.tools.qrdecomposition import get_baseParams
from figaroh.calibration.calibration_tools import (
    BaseCalibration,
    calculate_base_kinematics_regressor,
    load_data,
)


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
        self.model = robot.model
        self.data = robot.data
        
        # Load configuration
        with open(config_file, "r") as f:
            self.config = yaml.load(f, Loader=SafeLoader)
        
        self.identif_data = self.config["identification"]
        self.params_settings = get_identification_param_from_yaml(robot, self.identif_data)
        
        # Initialize attributes
        self.params_standard = None
        self.params_base = None
        self.idx_base = None
        self.idx_e = None
        self.params_r = None
        self.W_base = None
        self.phi_base = None
        self.phi_standard = None
        self.phi_ref = None
        self.rms_error = None
        self.correlation = None
        
        print(f"{self.__class__.__name__} initialized")
    
    def calculate_base_parameters(self):
        """Calculate structural base parameters for the robot."""
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
        
        # Get standard parameters (robot-specific implementation)
        self.params_standard = self.get_standard_parameters()
        
        # Remove zero columns and build reduced regressor
        self.idx_e, self.params_r = get_index_eliminate(W, self.params_standard, 1e-6)
        W_e = build_regressor_reduced(W, self.idx_e)
        
        # Calculate base regressor and base parameters
        _, self.params_base, self.idx_base = get_baseParams(W_e, self.params_r, self.params_standard)
        
        print(f"Calculated {len(self.params_base)} base parameters from {len(self.params_standard)} standard parameters")
        for ii, param in enumerate(self.params_base[:5]):  # Show first 5
            print(f"  {ii}: {param}")
        if len(self.params_base) > 5:
            print(f"  ... and {len(self.params_base) - 5} more parameters")
    
    @abstractmethod
    def get_standard_parameters(self):
        """Get standard parameters for the robot (robot-specific implementation).
        
        Returns:
            dict: Standard parameters for the robot
        """
        pass
    
    @abstractmethod
    def load_trajectory_data(self):
        """Load trajectory data from files (robot-specific implementation).
        
        Should set:
            self.q: joint positions
            self.dq: joint velocities  
            self.ddq: joint accelerations
            self.tau_noised: measured torques
        """
        pass
    
    def solve(self):
        """Perform robot dynamic parameter identification."""
        print(f"Starting {self.__class__.__name__} dynamic parameter identification...")
        
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
        self.calculate_standard_parameters_comparison()
        
        # Calculate identification quality metrics
        residuals = self.tau_noised - self.tau_identif
        self.rms_error = np.sqrt(np.mean(residuals**2))
        self.correlation = np.corrcoef(self.tau_noised, self.tau_identif)[0, 1]
        
        print(f"Dynamic identification completed")
        print(f"RMS error: {self.rms_error:.6f}")
        print(f"Correlation: {self.correlation:.4f}")
        print(f"{len(self.params_base)} base parameters")
        
        return self.phi_base
    
    def calculate_standard_parameters_comparison(self):
        """Calculate standard parameters for comparison with identified ones."""
        # Set up constraints for standard parameter calculation
        COM_max = np.ones((self.model.nq * 3, 1))  # COM position bounds
        COM_min = -np.ones((self.model.nq * 3, 1))
        
        try:
            # Build full regressor for standard parameter calculation
            W_full = build_regressor_basic(self.robot, self.q, self.dq, self.ddq, self.params_settings)
            self.phi_standard, self.phi_ref = calculate_standard_parameters(
                self.robot, W_full, self.tau_noised, COM_max, COM_min, 
                self.params_settings
            )
        except Exception as e:
            print(f"Warning: Could not calculate standard parameters: {e}")
            # Create dummy values for plotting
            self.phi_standard = np.zeros(len(self.params_base))
            self.phi_ref = np.zeros(len(self.params_base))
    
    def plot_results(self):
        """Plot identification results."""
        if not hasattr(self, 'phi_base') or self.phi_base is None:
            print("No identification results to plot. Run solve() first.")
            return
        
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
        
        # Plot standard parameters comparison if available
        if hasattr(self, 'phi_standard') and self.phi_standard is not None:
            plt.subplot(2, 1, 2)
            x_pos = np.arange(len(self.phi_standard))
            width = 0.35
            
            plt.bar(x_pos - width/2, self.phi_standard, width, label='SIP Identified', alpha=0.7)
            plt.bar(x_pos + width/2, self.phi_ref, width, label='SIP URDF', alpha=0.7)
            plt.xlabel('Parameter Index')
            plt.ylabel('Parameter Value')
            plt.title(f'{self.__class__.__name__} Standard Inertial Parameters Comparison')
            plt.legend()
            plt.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()
    
    def save_results(self, output_dir="results"):
        """Save identification results to files."""
        if not hasattr(self, 'phi_base') or self.phi_base is None:
            print("No identification results to save. Run solve() first.")
            return
        
        os.makedirs(output_dir, exist_ok=True)
        
        # Save identified parameters
        results_dict = {
            'base_parameters': self.phi_base.tolist(),
            'parameter_names': [str(p) for p in self.params_base],
            'rms_error': float(self.rms_error),
            'correlation': float(self.correlation),
            'condition_number': float(np.linalg.cond(self.W_base))
        }
        
        # Add standard parameters if available
        if hasattr(self, 'phi_standard') and self.phi_standard is not None:
            results_dict['standard_parameters_identified'] = self.phi_standard.tolist()
            results_dict['standard_parameters_reference'] = self.phi_ref.tolist()
        
        robot_name = self.__class__.__name__.lower().replace('identification', '')
        filename = f"{robot_name}_identification_results.yaml"
        
        with open(os.path.join(output_dir, filename), "w") as f:
            yaml.dump(results_dict, f, default_flow_style=False)
        
        print(f"Results saved to {output_dir}/{filename}")


class BaseOptimalCalibration(ABC):
    """
    Base class for robot optimal configuration generation for calibration.
    
    Provides common functionality for all robots while allowing
    robot-specific implementations of optimization strategies.
    """
    
    def __init__(self, robot, config_file="config/robot_config.yaml"):
        """Initialize optimal calibration with robot model and configuration.
        
        Args:
            robot: Robot model loaded with FIGAROH
            config_file: Path to robot configuration YAML file
        """
        self.robot = robot
        self.model = robot.model
        self.data = robot.data
        
        # Create a BaseCalibration object to get parameters
        calib_obj = BaseCalibration(robot, config_file)
        self.param = calib_obj.param
        
        # Initialize attributes
        self.optimal_configurations = None
        self.optimal_weights = None
        
        print(f"{self.__class__.__name__} initialized")
    
    def rearrange_rb(self, R_b, param):
        """Rearrange the kinematic regressor by sample numbered order."""
        Rb_rearr = np.empty_like(R_b)
        for i in range(param["calibration_index"]):
            for j in range(param["NbSample"]):
                Rb_rearr[param["NbSample"] * i + j, :] = R_b[param["calibration_index"] * j + i, :]
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
    
    @abstractmethod
    def load_candidate_configurations(self):
        """Load candidate configurations for optimization (robot-specific implementation).
        
        Returns:
            np.ndarray: Array of candidate joint configurations
        """
        pass
    
    @abstractmethod
    def optimize_selection(self, subX_dict, nb_chosen):
        """Optimize configuration selection (robot-specific implementation).
        
        Args:
            subX_dict: Dictionary of sub information matrices
            nb_chosen: Number of configurations to select
            
        Returns:
            tuple: (optimal_configs, optimal_weights)
        """
        pass
    
    def solve(self, nb_chosen=15):
        """Generate optimal configurations for robot calibration.
        
        Args:
            nb_chosen: Number of optimal configurations to select
        """
        print(f"Generating optimal configurations for {self.__class__.__name__}...")
        
        # Calculate base kinematic regressor
        q_rand = []
        Rrand_b, R_b, R_e, paramsrand_base, paramsrand_e = calculate_base_kinematics_regressor(
            q_rand, self.model, self.data, self.param
        )
        
        # Load candidate configurations
        q_candidates = self.load_candidate_configurations()
        
        print(f"Loaded {len(q_candidates)} candidate configurations")
        
        # Calculate regressor for all candidates
        R_candidates = np.zeros((len(q_candidates) * self.param["calibration_index"], 
                               self.param["calibration_index"]))
        
        for i, q in enumerate(q_candidates):
            # Calculate kinematic regressor for this configuration
            # This would need robot-specific implementation
            pass
        
        # Rearrange regressor
        R_rearranged = self.rearrange_rb(R_candidates, self.param)
        
        # Build sub-information matrices
        subX_dict = self.sub_info_matrix(R_rearranged, self.param)
        
        # Apply optimization for optimal selection
        optimal_configs, optimal_weights = self.optimize_selection(subX_dict, nb_chosen)
        
        self.optimal_configurations = optimal_configs
        self.optimal_weights = optimal_weights
        
        print(f"Selected {len(optimal_configs)} optimal configurations")
        
        return optimal_configs
    
    def plot_results(self):
        """Plot optimal configuration results."""
        if not hasattr(self, 'optimal_configurations') or self.optimal_configurations is None:
            print("No optimal configuration results to plot. Run solve() first.")
            return
        
        # Plot weight distribution
        plt.figure(figsize=(10, 6))
        
        if hasattr(self, 'optimal_weights') and self.optimal_weights is not None:
            plt.subplot(1, 2, 1)
            plt.bar(range(len(self.optimal_weights)), self.optimal_weights)
            plt.xlabel('Configuration Index')
            plt.ylabel('Weight')
            plt.title(f'{self.__class__.__name__} Configuration Weights')
            plt.grid(True, alpha=0.3)
        
        # Plot configuration distribution (if joint space is low dimensional)
        if len(self.optimal_configurations) > 0 and self.optimal_configurations[0].shape[0] <= 3:
            plt.subplot(1, 2, 2)
            configs = np.array(self.optimal_configurations)
            if configs.shape[1] >= 2:
                plt.scatter(configs[:, 0], configs[:, 1])
                plt.xlabel('Joint 1 (rad)')
                plt.ylabel('Joint 2 (rad)')
                plt.title(f'{self.__class__.__name__} Optimal Configurations')
                plt.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()
    
    def save_results(self, output_dir="results"):
        """Save optimal configuration results to files."""
        if not hasattr(self, 'optimal_configurations') or self.optimal_configurations is None:
            print("No optimal configuration results to save. Run solve() first.")
            return
        
        os.makedirs(output_dir, exist_ok=True)
        
        # Save optimal configurations
        results_dict = {
            'optimal_configurations': [config.tolist() for config in self.optimal_configurations],
            'number_selected': len(self.optimal_configurations)
        }
        
        if hasattr(self, 'optimal_weights') and self.optimal_weights is not None:
            results_dict['optimal_weights'] = self.optimal_weights.tolist()
        
        robot_name = self.__class__.__name__.lower().replace('optimalcalibration', '')
        filename = f"{robot_name}_optimal_configurations.yaml"
        
        with open(os.path.join(output_dir, filename), "w") as f:
            yaml.dump(results_dict, f, default_flow_style=False)
        
        print(f"Results saved to {output_dir}/{filename}")


class BaseOptimalTrajectory(ABC):
    """
    Base class for robot optimal trajectory generation for dynamic identification.
    
    Provides common functionality for all robots while allowing
    robot-specific implementations of trajectory generation and optimization.
    """
    
    def __init__(self, robot, config_file="config/robot_config.yaml"):
        """Initialize optimal trajectory generation with robot model and configuration.
        
        Args:
            robot: Robot model loaded with FIGAROH
            config_file: Path to robot configuration YAML file
        """
        self.robot = robot
        self.model = robot.model
        self.data = robot.data
        
        # Load configuration
        with open(config_file, "r") as f:
            self.config = yaml.load(f, Loader=SafeLoader)
        
        self.identif_data = self.config["identification"]
        self.params_settings = get_identification_param_from_yaml(robot, self.identif_data)
        
        # Initialize attributes
        self.params_base = None
        self.idx_base = None
        self.idx_e = None
        self.T_F = None
        self.P_F = None
        self.V_F = None
        self.A_F = None
        
        print(f"{self.__class__.__name__} initialized")
    
    @abstractmethod
    def get_base_parameter_indices(self):
        """Find base parameters for trajectory optimization (robot-specific implementation).
        
        Returns:
            tuple: (idx_e, idx_base) elimination and base parameter indices
        """
        pass
    
    @abstractmethod
    def generate_trajectory(self, waypoints, constraints):
        """Generate trajectory from waypoints (robot-specific implementation).
        
        Args:
            waypoints: Joint space waypoints
            constraints: Trajectory constraints (velocity, acceleration, etc.)
            
        Returns:
            tuple: (time, positions, velocities, accelerations)
        """
        pass
    
    @abstractmethod
    def evaluate_trajectory_quality(self, q, dq, ddq):
        """Evaluate trajectory quality for identification (robot-specific implementation).
        
        Args:
            q: Joint positions
            dq: Joint velocities
            ddq: Joint accelerations
            
        Returns:
            float: Quality metric (e.g., condition number)
        """
        pass
    
    def build_base_regressor(self, q, v, a, W_stack=None):
        """Build base regressor for given data."""
        W = build_regressor_basic(self.robot, q, v, a, self.params_settings)
        W_e_ = build_regressor_reduced(W, self.idx_e)
        
        # Build base regressor using base parameter indices
        W_b_ = W_e_[:, self.idx_base]
        
        if isinstance(W_stack, np.ndarray):
            W_b_ = np.vstack((W_stack, W_b_))
        return W_b_
    
    def solve(self, n_waypoints=5, n_iterations=10):
        """Generate optimal trajectory for robot identification.
        
        Args:
            n_waypoints: Number of waypoints in trajectory
            n_iterations: Number of optimization iterations
        """
        print(f"Generating optimal trajectory for {self.__class__.__name__}...")
        
        # Get base parameter indices
        self.idx_e, self.idx_base = self.get_base_parameter_indices()
        
        # Initialize trajectory optimization
        best_quality = np.inf
        best_trajectory = None
        
        for iteration in range(n_iterations):
            print(f"Iteration {iteration + 1}/{n_iterations}")
            
            try:
                # Generate random waypoints (robot-specific constraints)
                waypoints = self.generate_random_waypoints(n_waypoints)
                
                # Generate trajectory
                t, q, dq, ddq = self.generate_trajectory(waypoints, self.get_default_constraints())
                
                # Evaluate trajectory quality
                quality = self.evaluate_trajectory_quality(q, dq, ddq)
                
                if quality < best_quality:
                    best_quality = quality
                    best_trajectory = (t, q, dq, ddq)
                    print(f"  New best quality: {quality:.2e}")
                
            except Exception as e:
                print(f"  Iteration {iteration + 1} failed: {e}")
                continue
        
        if best_trajectory is not None:
            self.T_F, self.P_F, self.V_F, self.A_F = best_trajectory
            print(f"Optimal trajectory generated with quality: {best_quality:.2e}")
        else:
            print("Warning: No valid trajectory generated")
        
        return best_trajectory
    
    @abstractmethod
    def generate_random_waypoints(self, n_waypoints):
        """Generate random waypoints respecting robot constraints (robot-specific implementation).
        
        Args:
            n_waypoints: Number of waypoints to generate
            
        Returns:
            np.ndarray: Array of joint space waypoints
        """
        pass
    
    @abstractmethod
    def get_default_constraints(self):
        """Get default trajectory constraints for the robot (robot-specific implementation).
        
        Returns:
            dict: Dictionary of trajectory constraints
        """
        pass
    
    def plot_results(self):
        """Plot optimal trajectory results."""
        if not hasattr(self, 'P_F') or self.P_F is None:
            print("No trajectory results to plot. Run solve() first.")
            return
        
        # Plot joint trajectories
        plt.figure(figsize=(12, 8))
        
        # Plot positions
        plt.subplot(3, 1, 1)
        for i in range(self.P_F.shape[1]):
            plt.plot(self.T_F, self.P_F[:, i], label=f'Joint {i+1}')
        plt.xlabel('Time (s)')
        plt.ylabel('Position (rad)')
        plt.title(f'{self.__class__.__name__} Joint Positions')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # Plot velocities
        plt.subplot(3, 1, 2)
        for i in range(self.V_F.shape[1]):
            plt.plot(self.T_F, self.V_F[:, i], label=f'Joint {i+1}')
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity (rad/s)')
        plt.title(f'{self.__class__.__name__} Joint Velocities')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # Plot accelerations
        plt.subplot(3, 1, 3)
        for i in range(self.A_F.shape[1]):
            plt.plot(self.T_F, self.A_F[:, i], label=f'Joint {i+1}')
        plt.xlabel('Time (s)')
        plt.ylabel('Acceleration (rad/s²)')
        plt.title(f'{self.__class__.__name__} Joint Accelerations')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()
    
    def save_results(self, output_dir="results"):
        """Save optimal trajectory results to files."""
        if not hasattr(self, 'P_F') or self.P_F is None:
            print("No trajectory results to save. Run solve() first.")
            return
        
        os.makedirs(output_dir, exist_ok=True)
        
        # Save trajectory data
        results_dict = {
            'time': self.T_F.tolist(),
            'positions': self.P_F.tolist(),
            'velocities': self.V_F.tolist(),
            'accelerations': self.A_F.tolist(),
            'trajectory_length': len(self.T_F)
        }
        
        robot_name = self.__class__.__name__.lower().replace('optimaltrajectory', '')
        filename = f"{robot_name}_optimal_trajectory.yaml"
        
        with open(os.path.join(output_dir, filename), "w") as f:
            yaml.dump(results_dict, f, default_flow_style=False)
        
        print(f"Results saved to {output_dir}/{filename}")


# Utility classes for optimization algorithms that can be shared across robots

class DetMax:
    """Determinant maximization algorithm for optimal experimental design."""
    
    def __init__(self, candidate_pool, nb_chosen):
        self.candidate_pool = candidate_pool
        self.nb_chosen = nb_chosen
        self.selected_indices = []
    
    def get_critD(self, set_indices):
        """Calculate D-optimality criterion for given set of indices."""
        if len(set_indices) == 0:
            return 0
        
        # Build information matrix from selected candidates
        info_matrix = np.zeros_like(self.candidate_pool[0])
        for idx in set_indices:
            info_matrix += self.candidate_pool[idx]
        
        # Calculate determinant (D-optimality)
        try:
            return np.linalg.det(info_matrix)
        except:
            return 0
    
    def main_algo(self):
        """Main determinant maximization algorithm."""
        n_candidates = len(self.candidate_pool)
        
        # Greedy selection
        for _ in range(self.nb_chosen):
            best_idx = -1
            best_criterion = -np.inf
            
            for i in range(n_candidates):
                if i in self.selected_indices:
                    continue
                
                test_set = self.selected_indices + [i]
                criterion = self.get_critD(test_set)
                
                if criterion > best_criterion:
                    best_criterion = criterion
                    best_idx = i
            
            if best_idx >= 0:
                self.selected_indices.append(best_idx)
        
        return self.selected_indices


class SOCP:
    """Second-Order Cone Programming solver for optimal experimental design."""
    
    def __init__(self, candidate_pool, nb_chosen):
        self.candidate_pool = candidate_pool
        self.nb_chosen = nb_chosen
    
    def solve(self):
        """Solve SOCP optimization problem."""
        try:
            import picos as pc
            
            n_candidates = len(self.candidate_pool)
            
            # SOCP formulation
            problem = pc.Problem()
            w = pc.RealVariable("w", n_candidates, lower=0)
            t = pc.RealVariable("t", 1)
            
            # Constraints
            Mw = pc.sum(w[i] * self.candidate_pool[i] for i in range(n_candidates))
            problem.add_constraint(pc.sum(w) <= self.nb_chosen)
            problem.add_constraint(t <= pc.DetRootN(Mw))
            
            # Objective
            problem.set_objective("max", t)
            
            # Solve
            solution = problem.solve(solver="cvxopt")
            
            if solution is not None:
                w_values = [float(w.value[i]) for i in range(n_candidates)]
                
                # Select configurations with significant weights
                eps_opt = 1e-5
                selected_indices = [i for i in range(n_candidates) if w_values[i] > eps_opt]
                
                return selected_indices, w_values
            else:
                raise Exception("SOCP optimization failed")
                
        except Exception as e:
            print(f"SOCP failed: {e}, falling back to random selection")
            # Fallback to random selection
            selected_indices = np.random.choice(
                len(self.candidate_pool), 
                size=min(self.nb_chosen, len(self.candidate_pool)), 
                replace=False
            ).tolist()
            w_values = [1.0 if i in selected_indices else 0.0 for i in range(len(self.candidate_pool))]
            
            return selected_indices, w_values
