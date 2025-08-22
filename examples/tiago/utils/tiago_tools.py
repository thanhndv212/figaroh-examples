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
import yaml
import logging
from matplotlib import pyplot as plt
from yaml.loader import SafeLoader
from typing import Dict, List, Tuple, Any
from scipy.optimize import least_squares

from .base_identification import BaseIdentification
from .base_optimal_calibration import BaseOptimalCalibration
from figaroh.calibration.calibration_tools import BaseCalibration
from figaroh.identification.identification_tools import get_standard_parameters
from figaroh.calibration.calibration_tools import (
    load_data,
    calc_updated_fkm,
    get_LMvariables,
)
from figaroh.tools.regressor import (
    build_regressor_basic,
    build_regressor_reduced,
    get_index_eliminate,
)
from figaroh.tools.qrdecomposition import get_baseIndex, build_baseRegressor
from figaroh.tools.robotcollisions import CollisionWrapper
from figaroh.identification.identification_tools import get_param_from_yaml
from figaroh.utils.cubic_spline import (
    CubicSpline, WaypointsGeneration, calc_torque
)
from figaroh.tools.robotipopt import (
    IPOPTConfig, BaseOptimizationProblem, RobotIPOPTSolver
)


class TiagoCalibration(BaseCalibration):
    """
    Class for calibrating the TIAGo robot.
    """
    def __init__(self, robot, config_file, del_list=[]):
        super().__init__(robot, config_file, del_list)

    def cost_function(self, var):
        """
        Cost function for the optimization problem.
        """
        coeff_ = self.param["coeff_regularize"]
        PEEe = calc_updated_fkm(self.model, self.data, var, self.q_measured, self.param)
        res_vect = np.append(
            (self.PEE_measured - PEEe),
            np.sqrt(coeff_)
            * var[6 : -self.param["NbMarkers"] * self.param["calibration_index"]],
        )
        return res_vect

    def solve_optimisation(self):
        """
        Solve the optimization problem.
        """

        # set initial guess
        _var_0, _ = get_LMvariables(self.param, mode=0)
        _var_0[0:6] = np.array(self.param["camera_pose"])
        _var_0[-self.param["calibration_index"] :] = np.array(self.param["tip_pose"])[
            : self.param["calibration_index"]
        ]
        self._var_0 = _var_0

        # define solver parameters
        iterate = True
        iter_max = 10
        count = 0
        del_list_ = []
        res = _var_0
        outlier_eps = self.param["outlier_eps"]

        while count < iter_max and iterate:
            print("*" * 50)
            print(
                "{} iter guess".format(count),
                dict(zip(self.param["param_name"], list(_var_0))),
            )

            # define solver
            LM_solve = least_squares(
                self.cost_function,
                _var_0,
                method="lm",
                verbose=1,
                args=(),
            )

            # solution
            res = LM_solve.x
            _PEEe_sol = calc_updated_fkm(
                self.model, self.data, res, self.q_measured, self.param
            )
            rmse = np.sqrt(np.mean((_PEEe_sol - self.PEE_measured) ** 2))
            mae = np.mean(np.abs(_PEEe_sol - self.PEE_measured))

            print("solution of calibrated parameters: ")
            for x_i, xname in enumerate(self.param["param_name"]):
                print(x_i + 1, xname, list(res)[x_i])
            print("position root-mean-squared error of end-effector: ", rmse)
            print("position mean absolute error of end-effector: ", mae)
            print("optimality: ", LM_solve.optimality)

            # check for unrealistic values
            delta_PEE = _PEEe_sol - self.PEE_measured
            PEE_xyz = delta_PEE.reshape(
                (
                    self.param["NbMarkers"] * self.param["calibration_index"],
                    self.param["NbSample"],
                )
            )
            PEE_dist = np.zeros((self.param["NbMarkers"], self.param["NbSample"]))
            for i in range(self.param["NbMarkers"]):
                for j in range(self.param["NbSample"]):
                    PEE_dist[i, j] = np.sqrt(
                        PEE_xyz[i * 3, j] ** 2
                        + PEE_xyz[i * 3 + 1, j] ** 2
                        + PEE_xyz[i * 3 + 2, j] ** 2
                    )
            for i in range(self.param["NbMarkers"]):
                for k in range(self.param["NbSample"]):
                    if PEE_dist[i, k] > outlier_eps:
                        del_list_.append((i, k))
            print(
                "indices of samples with >{} m deviation: ".format(outlier_eps),
                del_list_,
            )

            # reset iteration with outliers removal
            if len(del_list_) > 0 and count < iter_max:
                self.PEE_measured, self.q_measured = load_data(
                    self._data_path,
                    self.model,
                    self.param,
                    self.del_list_ + del_list_,
                )
                self.param["NbSample"] = self.q_measured.shape[0]
                count += 1
                _var_0 = res + np.random.normal(0, 0.01, size=res.shape)
            else:
                iterate = False
        self._PEE_dist = PEE_dist
        param_values_ = [float(res_i_) for res_i_ in res]
        self.calibrated_param = dict(zip(self.param["param_name"], param_values_))
        self.LM_result = LM_solve
        self.rmse = rmse
        self.mae = mae
        if self.LM_result.success:
            self.STATUS = "CALIBRATED"


class TiagoIdentification(BaseIdentification):
    """TIAGo-specific dynamic parameter identification class."""
    
    def __init__(self, robot, config_file="config/tiago_config.yaml"):
        """Initialize TIAGo identification with robot model and configuration.
        
        Args:
            robot: TIAGo robot model loaded with FIGAROH
            config_file: Path to TIAGo configuration YAML file
        """
        super().__init__(robot, config_file)
        print("TiagoIdentification initialized for TIAGo robot")
    
    def get_standard_parameters(self):
        """Get standard parameters for TIAGo robot."""
        return get_standard_parameters(self.model, self.params_settings)
    
    def process_torque_data(self, tau):
        """Process torque data with TIAGo-specific motor constants."""
        import pinocchio as pin
        
        # Apply TIAGo-specific torque processing (reduction ratios, etc.)
        pin.computeSubtreeMasses(self.robot.model, self.robot.data)
        tau_processed = tau.copy()
        
        for i, joint_name in enumerate(self.params_settings["active_joints"]):
            if joint_name == "torso_lift_joint":
                tau_processed[:, i] = (
                    self.params_settings["reduction_ratio"][joint_name]
                    * self.params_settings["kmotor"][joint_name]
                    * tau[:, i]
                    + 9.81 * self.robot.data.mass[
                        self.robot.model.getJointId(joint_name)
                    ]
                )
            else:
                tau_processed[:, i] = (
                    self.params_settings["reduction_ratio"][joint_name]
                    * self.params_settings["kmotor"][joint_name]
                    * tau[:, i]
                )
        return tau_processed
    
    def calculate_regressor_matrix(self, q, qd, qdd):
        """Calculate dynamic regressor matrix for TIAGo robot.
        
        Args:
            q: Joint positions
            qd: Joint velocities  
            qdd: Joint accelerations
            
        Returns:
            np.ndarray: Dynamic regressor matrix
        """
        from figaroh.identification.identification_tools import (
            calculate_base_dynamic_regressor
        )
        
        # Use FIGAROH tools to calculate regressor for TIAGo
        regressor, _ = calculate_base_dynamic_regressor(
            q, qd, qdd, self.model, self.data, self.params_settings
        )
        return regressor


class TiagoOptimalCalibration(BaseOptimalCalibration):
    """TIAGo-specific optimal configuration generation for calibration."""
    
    def __init__(self, robot, config_file="config/tiago_config.yaml"):
        """Initialize TIAGo optimal calibration."""
        super().__init__(robot, config_file)
        print("TIAGo Optimal Calibration initialized")
    
    def load_candidate_configurations(self):
        """Load candidate configurations from TIAGo YAML file.
        
        Note: This method satisfies the abstract base class requirement,
        but actual data loading is handled in the base class load_data_set method.
        """
        return np.array([])
    
    def optimize_selection(self, subX_dict, nb_chosen):
        """Optimize configuration selection using SOCP for TIAGo.
        
        Note: This method satisfies the abstract base class requirement,
        but actual optimization is handled in the base class.
        """
        return [], []


# Configuration and trajectory optimization classes
# transferred from optimal_trajectory.py


class ConfigurationManager:
    """Manages configuration loading and validation."""
    
    @staticmethod
    def load_from_yaml(config_file: str) -> Tuple[Dict[str, Any], Any]:
        """Load trajectory parameters from YAML file."""
        try:
            with open(config_file, "r") as f:
                config = yaml.load(f, Loader=SafeLoader)
            
            identif_data = config["identification"]
            traj_params = identif_data.get("trajectory_params", [{}])[0]
            
            # Set default values if not present in config
            trajectory_config = {
                'n_wps': traj_params.get("n_wps", 5),
                'freq': traj_params.get("freq", 100),
                't_s': traj_params.get("t_s", 2.0),
                'soft_lim': traj_params.get("soft_lim", 0.05),
                'max_attempts': traj_params.get("max_attempts", 1000)
            }
            
            return trajectory_config, identif_data
            
        except FileNotFoundError:
            raise FileNotFoundError(
                f"Configuration file {config_file} not found"
            )
        except KeyError as e:
            raise ValueError(f"Missing required configuration key: {e}")
        except Exception as e:
            raise ValueError(f"Error loading configuration: {e}")


class BaseParameterComputer:
    """Handles base parameter computation and indexing."""
    
    def __init__(self, robot, params_settings, active_joints, soft_lim_pool):
        self.robot = robot
        self.params_settings = params_settings
        self.active_joints = active_joints
        self.soft_lim_pool = soft_lim_pool
        
    def compute_base_indices(self) -> Tuple[np.ndarray, np.ndarray]:
        """Compute base parameter indices from random trajectory."""
        logging.info(
            "Computing base parameter indices from random trajectory..."
        )
        
        try:
            # Generate random trajectory for base parameter computation
            n_wps_r = 100
            freq_r = 100
            CB_r = CubicSpline(self.robot, n_wps_r, self.active_joints)
            WP_r = WaypointsGeneration(self.robot, n_wps_r, self.active_joints)
            WP_r.gen_rand_pool(self.soft_lim_pool)
            
            # Generate waypoints and trajectory
            wps_r, vel_wps_r, acc_wps_r = WP_r.gen_rand_wp()
            tps_r = np.matrix([0.5 * i for i in range(n_wps_r)]).transpose()
            t_r, p_r, v_r, a_r = CB_r.get_full_config(
                freq_r, tps_r, wps_r, vel_wps_r, acc_wps_r
            )
            
            # Compute base indices
            idx_e, idx_b = self._get_idx_from_random(p_r, v_r, a_r)
            logging.info(f"Computed {len(idx_b)} base parameters")
            
            return idx_e, idx_b
            
        except Exception as e:
            logging.error(f"Error computing base indices: {e}")
            raise
    
    def _get_idx_from_random(self, q, v, a) -> Tuple[np.ndarray, np.ndarray]:
        """Get indices of eliminate and base parameters."""
        W = build_regressor_basic(self.robot, q, v, a, self.params_settings)
        params_std = get_standard_parameters(
            self.robot.model, self.params_settings
        )
        idx_e_, par_r_ = get_index_eliminate(W, params_std, tol_e=0.001)
        W_e_ = build_regressor_reduced(W, idx_e_)
        idx_base_ = get_baseIndex(W_e_, par_r_)
        return idx_e_, idx_base_


class TrajectoryConstraintManager:
    """Manages trajectory constraints and bounds."""
    
    def __init__(self, robot, CB, traj_params, params_settings):
        self.robot = robot
        self.CB = CB
        self.n_wps = traj_params['n_wps']
        self.freq = traj_params['freq']
        self.params_settings = params_settings
        self.collision_wrapper = CollisionWrapper(robot=robot, viz=None)
    
    def get_variable_bounds(self) -> Tuple[List[float], List[float]]:
        """Get variable bounds for optimization."""
        lb, ub = [], []
        for i in range(1, self.n_wps):
            lb.extend(self.CB.lower_q)
            ub.extend(self.CB.upper_q)
        return lb, ub
    
    def get_constraint_bounds(self, Ns: int) -> Tuple[List, List]:
        """Get constraint bounds for optimization."""
        cl, cu = [], []
        
        # Position constraint bounds
        for i in range(1, self.n_wps):
            cl.extend(self.CB.lower_q)
            cu.extend(self.CB.upper_q)
        
        # Velocity constraint bounds
        for j in range(Ns):
            cl.extend(self.CB.lower_dq)
            cu.extend(self.CB.upper_dq)
        
        # Torque constraint bounds
        for j in range(Ns):
            cl.extend(self.CB.lower_effort)
            cu.extend(self.CB.upper_effort)
        
        # Collision constraint bounds
        n_cols = len(self.robot.geom_model.collisionPairs)
        cl.extend([0.01] * n_cols * (self.n_wps - 1))  # 1 cm margin
        cu.extend([2 * 1e19] * n_cols * (self.n_wps - 1))  # no upper limit
        
        return cl, cu
    
    def evaluate_constraints(self, Ns: int, X: np.ndarray, opt_cb: Dict,
                             tps, vel_wps, acc_wps, wp_init) -> np.ndarray:
        """Evaluate all constraints for optimization."""
        try:
            # Reshape and arrange waypoints
            X = np.array(X)
            wps_X = np.reshape(X, (self.n_wps - 1, len(self.CB.act_idxq)))
            wps = np.vstack((wp_init, wps_X))
            wps = wps.transpose()
            
            # Generate full trajectory configuration
            t_f, p_f, v_f, a_f = self.CB.get_full_config(
                self.freq, tps, wps, vel_wps, acc_wps
            )
            
            # Compute joint torques
            tau = calc_torque(
                p_f.shape[0], self.robot, p_f, v_f, a_f, self.params_settings
            )
            
            # Evaluate individual constraint types
            q_constraints = self._evaluate_position_constraints(p_f, tps, t_f)
            v_constraints = self._evaluate_velocity_constraints(v_f)
            tau_constraints = self._evaluate_torque_constraints(tau, Ns)
            collision_constraints = self._evaluate_collision_constraints(
                p_f, tps, t_f
            )
            
            # Concatenate all constraints
            return np.concatenate(
                (q_constraints, v_constraints, tau_constraints,
                 collision_constraints),
                axis=None
            )
            
        except Exception as e:
            logging.error(f"Error evaluating constraints: {e}")
            raise
    
    def _evaluate_position_constraints(self, p_f, tps, t_f) -> np.ndarray:
        """Evaluate position constraints at waypoints."""
        idx_waypoints = self._get_waypoint_indices(tps, t_f)
        q_constraints = p_f[idx_waypoints, :]
        return q_constraints[:, self.CB.act_idxq]
    
    def _evaluate_velocity_constraints(self, v_f) -> np.ndarray:
        """Evaluate velocity constraints at all samples."""
        return v_f[:, self.CB.act_idxv]
    
    def _evaluate_torque_constraints(self, tau, Ns) -> np.ndarray:
        """Evaluate torque constraints at all samples."""
        tau_constraints = np.zeros((Ns, len(self.CB.act_idxv)))
        for k in range(len(self.CB.act_idxv)):
            tau_constraints[:, k] = tau[
                range(self.CB.act_idxv[k] * Ns, (self.CB.act_idxv[k] + 1) * Ns)
            ]
        return tau_constraints
    
    def _evaluate_collision_constraints(self, p_f, tps, t_f) -> np.ndarray:
        """Evaluate collision constraints."""
        idx_waypoints = self._get_waypoint_indices(tps, t_f)
        dist_all = []
        for j in idx_waypoints:
            self.collision_wrapper.computeCollisions(p_f[j, :])
            dist_all = np.append(dist_all, self.collision_wrapper.getDistances())
        return np.asarray(dist_all)
    
    def _get_waypoint_indices(self, tps, t_f) -> List[int]:
        """Get indices corresponding to waypoint times."""
        idx_waypoints = []
        time_points = tps[range(1, self.n_wps), :]
        time_points_flat = np.array(time_points).flatten()
        
        for i in range(t_f.shape[0]):
            t_val = float(t_f[i, 0]) if hasattr(t_f[i, 0], 'item') else t_f[i, 0]
            if t_val in time_points_flat:
                idx_waypoints.append(i)
        
        return idx_waypoints


class OptimalTrajectoryIPOPT:
    """
    Improved IPOPT-based optimal trajectory generation for TIAGo robot.
    
    Features:
    - Modular design with separated concerns
    - Better error handling and logging
    - Configuration validation
    - Cleaner interfaces
    """
    
    def __init__(self, robot, active_joints: List[str], 
                 config_file: str = "config/tiago_config.yaml"):
        """Initialize the optimal trajectory generator."""
        self.robot = robot
        self.active_joints = active_joints
        
        # Set up logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        
        # Load configuration
        self.traj_params, identif_data = ConfigurationManager.load_from_yaml(config_file)
        self.params_settings = get_param_from_yaml(robot, identif_data)
        
        # Initialize components
        self._initialize_components()
        
        # Compute base parameters
        self.idx_e, self.idx_b = self.base_computer.compute_base_indices()
        
        # Results storage
        self.results = {
            'T_F': [], 'P_F': [], 'V_F': [], 'A_F': [],
            'iteration_data': [], 'final_regressor_shape': None
        }
        
        self.logger.info(f"OptimalTrajectoryIPOPT initialized with {len(self.idx_b)} base parameters")
    
    def _initialize_components(self):
        """Initialize trajectory generation components."""
        # Create soft limit pool
        n_active_joints = len(self.active_joints)
        self.soft_lim_pool = np.full((3, n_active_joints), self.traj_params['soft_lim'])
        
        # Initialize cubic spline and waypoint generation
        self.CB = CubicSpline(
            self.robot, self.traj_params['n_wps'], 
            self.active_joints, self.traj_params['soft_lim']
        )
        self.WP = WaypointsGeneration(
            self.robot, self.traj_params['n_wps'], 
            self.active_joints, self.traj_params['soft_lim']
        )
        
        # Initialize specialized components
        self.base_computer = BaseParameterComputer(
            self.robot, self.params_settings, self.active_joints, self.soft_lim_pool
        )
        self.constraint_manager = TrajectoryConstraintManager(
            self.robot, self.CB, self.traj_params, self.params_settings
        )
    
    def build_base_regressor(self, q, v, a, W_stack=None) -> np.ndarray:
        """Build base regressor matrix."""
        try:
            W = build_regressor_basic(self.robot, q, v, a, self.params_settings)
            W_e_ = build_regressor_reduced(W, self.idx_e)
            W_b_ = build_baseRegressor(W_e_, self.idx_b)
            
            if isinstance(W_stack, np.ndarray):
                W_b_ = np.vstack((W_stack, W_b_))
            
            return W_b_
        except Exception as e:
            self.logger.error(f"Error building base regressor: {e}")
            raise
    
    def objective_function(self, X, opt_cb, tps, vel_wps, acc_wps, wp_init, W_stack=None):
        """Objective function: condition number of base regressor matrix."""
        try:
            # Reshape and arrange waypoints
            X = np.array(X)
            wps_X = np.reshape(X, (self.traj_params['n_wps'] - 1, len(self.active_joints)))
            wps = np.vstack((wp_init, wps_X))
            wps = wps.transpose()
            
            # Generate full trajectory configuration
            t_f, p_f, v_f, a_f = self.CB.get_full_config(
                self.traj_params['freq'], tps, wps, vel_wps, acc_wps
            )
            
            # Store in callback dictionary
            opt_cb.update({"t_f": t_f, "p_f": p_f, "v_f": v_f, "a_f": a_f})
            
            # Build stacked base regressor and return condition number
            W_b = self.build_base_regressor(p_f, v_f, a_f, W_stack=W_stack)
            return np.linalg.cond(W_b)
            
        except Exception as e:
            self.logger.error(f"Error in objective function: {e}")
            return 1e10  # Return large penalty value
    
    def generate_feasible_initial_guess(self, wp_init, vel_wp_init, acc_wp_init):
        """Generate a feasible initial guess for optimization."""
        self.logger.info("Generating feasible initial trajectory...")
        
        count = 0
        is_constr_violated = True
        
        while is_constr_violated and count < self.traj_params['max_attempts']:
            count += 1
            if count % 100 == 0:
                self.logger.info(f"Attempt {count} to find feasible initial trajectory...")
            
            try:
                # Generate random waypoints
                wps, vel_wps, acc_wps = self.WP.gen_rand_wp(wp_init, vel_wp_init, acc_wp_init)
                
                # Generate time points
                tps = np.matrix([
                    self.traj_params['t_s'] * i_wp 
                    for i_wp in range(self.traj_params['n_wps'])
                ]).transpose()
                
                # Get full configuration
                t_i, p_i, v_i, a_i = self.CB.get_full_config(
                    self.traj_params['freq'], tps, wps, vel_wps, acc_wps
                )
                
                # Compute torques and check constraints
                tau_i = calc_torque(
                    p_i.shape[0], self.robot, p_i, v_i, a_i, self.params_settings
                )
                tau_i = np.reshape(tau_i, (v_i.shape[1], v_i.shape[0])).transpose()
                is_constr_violated = self.CB.check_cfg_constraints(p_i, v_i, tau_i)
                
            except Exception as e:
                self.logger.warning(f"Error in attempt {count}: {e}")
                continue
        
        if count >= self.traj_params['max_attempts']:
            self.logger.warning(
                f"Could not find feasible initial trajectory after {self.traj_params['max_attempts']} attempts"
            )
        else:
            self.logger.info(f"Found feasible initial trajectory after {count} attempts")
        
        return wps, vel_wps, acc_wps, tps, t_i, p_i, v_i, a_i
    
    def solve(self, stack_reps: int = 2) -> Dict[str, Any]:
        """
        Solve the optimal trajectory generation problem.
        
        Args:
            stack_reps: Number of trajectory segments to stack
            
        Returns:
            Dict containing trajectories and optimization info
        """
        self.logger.info(f"Starting optimal trajectory generation with {stack_reps} segments...")
        
        try:
            # Initialize
            self.WP.gen_rand_pool(self.soft_lim_pool)
            wp_init = np.zeros(len(self.CB.act_idxq))
            vel_wp_init = np.zeros(len(self.CB.act_idxv))
            acc_wp_init = np.zeros(len(self.CB.act_idxv))
            
            # Random initial position
            for idx in range(len(self.CB.act_idxq)):
                wp_init[idx] = np.random.choice(self.WP.pool_q[:, idx], 1)[0]
            
            W_stack = None
            
            for s_rep in range(stack_reps):
                self.logger.info(f"Optimizing segment {s_rep + 1}/{stack_reps}")
                self.logger.info(f"Initial waypoint: {wp_init}")
                
                success = self._solve_segment(s_rep, wp_init, vel_wp_init, acc_wp_init, W_stack)
                
                if not success:
                    self.logger.error(f"Failed to solve segment {s_rep + 1}")
                    break
                
                # Update for next segment
                if s_rep < stack_reps - 1:  # Not the last segment
                    wp_init, W_stack = self._prepare_next_segment()
            
            self.logger.info(f"Completed! Generated {len(self.results['T_F'])} trajectory segments")
            self.results['final_regressor_shape'] = W_stack.shape if W_stack is not None else None
            
            return self.results
            
        except Exception as e:
            self.logger.error(f"Error in solve: {e}")
            raise
    
    def _solve_segment(self, s_rep, wp_init, vel_wp_init, acc_wp_init, W_stack) -> bool:
        """Solve a single trajectory segment."""
        try:
            # Generate feasible initial guess
            wps, vel_wps, acc_wps, tps, t_i, p_i, v_i, a_i = self.generate_feasible_initial_guess(
                wp_init, vel_wp_init, acc_wp_init
            )
            
            # Adjust time points for stacking
            tps = self.traj_params['t_s'] * s_rep + tps
            
            # Create and solve IPOPT problem
            problem = TiagoTrajectoryIPOPTProblem(
                self, len(self.active_joints), self.traj_params['n_wps'],
                p_i.shape[0], tps, vel_wps, acc_wps, wp_init, vel_wp_init, 
                acc_wp_init, W_stack
            )
            
            success, result_data = problem.solve_with_waypoints(wps)
            
            if success:
                self.results['T_F'].append(result_data['t_f'])
                self.results['P_F'].append(result_data['p_f'][:, self.CB.act_idxq])
                self.results['V_F'].append(result_data['v_f'][:, self.CB.act_idxv])
                self.results['A_F'].append(result_data['a_f'][:, self.CB.act_idxv])
                self.results['iteration_data'].append(result_data['iter_data'])
                self.logger.info(f"Segment {s_rep + 1} completed successfully!")
                return True
            else:
                return False
                
        except Exception as e:
            self.logger.error(f"Error solving segment {s_rep + 1}: {e}")
            return False
    
    def _prepare_next_segment(self) -> Tuple[np.ndarray, np.ndarray]:
        """Prepare initial conditions for next segment."""
        # Get last result
        last_result = self.results['iteration_data'][-1]
        wp_init = last_result['final_waypoint']
        
        # Stack regressor
        last_p_f = self.results['P_F'][-1]
        last_v_f = self.results['V_F'][-1]
        last_a_f = self.results['A_F'][-1]
        
        # Convert back to full configuration for regressor building
        # This is a simplified version - in practice you'd need to handle this more carefully
        W_stack = self.build_base_regressor(last_p_f, last_v_f, last_a_f)
        
        return wp_init, W_stack
    
    def plot_results(self):
        """Plot the optimization results with improved visualization."""
        if not self.results['T_F']:
            self.logger.warning("No trajectory data to plot")
            return
        
        try:
            # Create subplots
            n_joints = len(self.CB.act_Jid)
            fig, axes = plt.subplots(n_joints, 3, sharex=True, figsize=(15, 2*n_joints))
            if n_joints == 1:
                axes = axes.reshape(1, -1)
            
            fig.suptitle('Optimal Trajectory Results', fontsize=16)
            
            # Plot each segment
            colors = plt.cm.tab10(np.linspace(0, 1, len(self.results['T_F'])))
            
            for seg_idx, (T, P, V, A) in enumerate(zip(
                self.results['T_F'], self.results['P_F'], 
                self.results['V_F'], self.results['A_F']
            )):
                color = colors[seg_idx]
                label = f'Segment {seg_idx + 1}'
                
                for joint_idx in range(n_joints):
                    axes[joint_idx, 0].plot(T, P[:, joint_idx], color=color, label=label)
                    axes[joint_idx, 1].plot(T, V[:, joint_idx], color=color, label=label)
                    axes[joint_idx, 2].plot(T, A[:, joint_idx], color=color, label=label)
            
            # Set labels and formatting
            for joint_idx in range(n_joints):
                axes[joint_idx, 0].set_ylabel(f'Joint {joint_idx+1}\nPosition (rad)')
                axes[joint_idx, 1].set_ylabel(f'Joint {joint_idx+1}\nVelocity (rad/s)')
                axes[joint_idx, 2].set_ylabel(f'Joint {joint_idx+1}\nAcceleration (rad/s²)')
                
                if joint_idx == 0:
                    for col in range(3):
                        axes[joint_idx, col].legend()
                
                for col in range(3):
                    axes[joint_idx, col].grid(True, alpha=0.3)
            
            axes[-1, 0].set_xlabel('Time (s)')
            axes[-1, 1].set_xlabel('Time (s)')
            axes[-1, 2].set_xlabel('Time (s)')
            
            plt.tight_layout()
            plt.show()
            
        except Exception as e:
            self.logger.error(f"Error plotting results: {e}")


class TiagoTrajectoryIPOPTProblem(BaseOptimizationProblem):
    """
    IPOPT problem formulation for TIAGo trajectory optimization.
    
    This class inherits from the generalized BaseOptimizationProblem
    and implements the specific objective and constraints for TIAGo
    trajectory optimization.
    """
    
    def __init__(self, opt_traj, n_joints, n_wps, Ns, tps, vel_wps, acc_wps, 
                 wp_init, vel_wp_init, acc_wp_init, W_stack):
        super().__init__("TiagoTrajectoryOptimization")
        
        self.opt_traj = opt_traj
        self.n_joints = n_joints
        self.n_wps = n_wps
        self.Ns = Ns
        self.tps = tps
        self.vel_wps = vel_wps
        self.acc_wps = acc_wps
        self.wp_init = wp_init
        self.vel_wp_init = vel_wp_init
        self.acc_wp_init = acc_wp_init
        self.W_stack = W_stack
        
        # Storage for optimization callback (inherits callback_data from base)
        self.opt_cb = {"t_f": None, "p_f": None, "v_f": None, "a_f": None}
    
    def get_variable_bounds(self) -> Tuple[List[float], List[float]]:
        """Get variable bounds for optimization."""
        return self.opt_traj.constraint_manager.get_variable_bounds()
    
    def get_constraint_bounds(self) -> Tuple[List[float], List[float]]:
        """Get constraint bounds for optimization."""
        return self.opt_traj.constraint_manager.get_constraint_bounds(self.Ns)
    
    def get_initial_guess(self) -> List[float]:
        """Get initial guess from waypoints."""
        # This will be set when solve() is called with waypoints
        if not hasattr(self, '_initial_wps'):
            # Return zeros as fallback
            return [0.0] * (self.n_joints * (self.n_wps - 1))
        
        X0 = self._initial_wps[:, range(1, self.n_wps)]
        return np.reshape(X0.transpose(), 
                         (self.n_joints * (self.n_wps - 1),)).tolist()
    
    def objective(self, X: np.ndarray) -> float:
        """Objective function: condition number of base regressor matrix."""
        return self.opt_traj.objective_function(
            X, self.opt_cb, self.tps, self.vel_wps, self.acc_wps, 
            self.wp_init, self.W_stack
        )
    
    def constraints(self, X: np.ndarray) -> np.ndarray:
        """Constraint function for IPOPT."""
        return self.opt_traj.constraint_manager.evaluate_constraints(
            self.Ns, X, self.opt_cb, self.tps, self.vel_wps, 
            self.acc_wps, self.wp_init
        )
    
    def jacobian(self, X: np.ndarray) -> np.ndarray:
        """
        Jacobian of constraints - Custom implementation for better performance.
        
        For trajectory optimization, we can use sparse finite differences
        instead of full automatic differentiation which is too slow.
        """
        try:
            # Get current constraint values
            c0 = self.constraints(X)
            n_constraints = len(c0)
            n_vars = len(X)
            
            # Use finite differences with smaller step size for efficiency
            eps = 1e-6
            jac = np.zeros((n_constraints, n_vars))
            
            # Compute Jacobian column by column (forward differences)
            for i in range(n_vars):
                X_plus = X.copy()
                X_plus[i] += eps
                c_plus = self.constraints(X_plus)
                jac[:, i] = (c_plus - c0) / eps
            
            self.logger.debug(f"Constraint jacobian shape: {jac.shape}")
            return jac
            
        except Exception as e:
            self.logger.warning(f"Error computing jacobian: {e}")
            # Return sparse identity matrix as fallback
            n_constraints = len(self.constraints(X))
            n_vars = len(X)
            # Create a sparse jacobian approximation
            jac = np.zeros((n_constraints, n_vars))
            min_dim = min(n_constraints, n_vars)
            jac[:min_dim, :min_dim] = np.eye(min_dim)
            return jac
    
    def solve_with_waypoints(self, wps) -> Tuple[bool, Dict[str, Any]]:
        """
        Solve the optimization problem with given initial waypoints.
        
        Args:
            wps: Initial waypoints
            
        Returns:
            Tuple of (success, results_dict)
        """
        try:
            # Store initial waypoints for get_initial_guess
            self._initial_wps = wps
            
            # Create solver with trajectory optimization config
            config = IPOPTConfig.for_trajectory_optimization()
            # Adjust settings for this complex problem
            config.tolerance = 1e-3
            config.acceptable_tolerance = 1e-2
            config.max_iterations = 200
            config.print_level = 3  # Reduce output
            config.custom_options = {
                b"mu_strategy": b"adaptive",
            }
            solver = RobotIPOPTSolver(self, config)
            
            # Solve the problem
            success, results = solver.solve()
            
            if success:
                # Extract final waypoint for next segment
                X_opt = results['x_opt']
                wps_X = np.reshape(np.array(X_opt), (self.n_wps - 1, self.n_joints))
                final_waypoint = wps_X[-1, :]
                
                # Update results with trajectory-specific data
                results.update({
                    't_f': self.opt_cb["t_f"],
                    'p_f': self.opt_cb["p_f"],
                    'v_f': self.opt_cb["v_f"],
                    'a_f': self.opt_cb["a_f"],
                    'iter_data': {
                        'iterations': self.iteration_data['iterations'],
                        'obj_values': self.iteration_data['obj_values'],
                        'solve_time': results['solve_time'],
                        'status': results['status'],
                        'final_waypoint': final_waypoint
                    }
                })
                
                return True, results
            else:
                self.logger.error("Optimization failed")
                return False, results
                
        except Exception as e:
            self.logger.error(f"Error in IPOPT solve: {e}")
            return False, {'error': str(e)}
