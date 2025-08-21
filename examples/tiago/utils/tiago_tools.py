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


from os.path import abspath
from scipy.optimize import least_squares
import numpy as np
import yaml
import time
import random
import matplotlib.pyplot as plt
import pandas as pd
import picos as pc
from datetime import datetime
import cyipopt
import numdifftools as nd
from scipy import signal

from figaroh.calibration.calibration_tools import (
    load_data,
    calc_updated_fkm,
    get_LMvariables,
    BaseCalibration,
    calculate_base_kinematics_regressor,
    rank_in_configuration,
)
from figaroh.identification.identification_tools import get_param_from_yaml, relative_stdev
from figaroh.tools.regressor import (
    build_regressor_basic,
    build_regressor_reduced,
    get_index_eliminate,
)
from figaroh.tools.qrdecomposition import double_QR, get_baseIndex, build_baseRegressor
from figaroh.tools.robotcollisions import CollisionWrapper
from .simplified_colission_model import build_tiago_simplified
from .dynamic_identification_utils import (
    get_standard_parameters,
)
from figaroh.utils.cubic_spline import CubicSpline, WaypointsGeneration, calc_torque



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
    """
    Generate optimal trajectories for dynamic parameter identification.
    """
    def __init__(self, robot, config_file, active_joints):
        self._robot = robot
        self.model = self._robot.model
        self.data = self._robot.data
        self.active_joints = active_joints
        self.load_param(config_file)
        
        # Apply simplified collision model
        self._robot = build_tiago_simplified(self._robot)
        
        # Get base parameter indices
        self.idx_e, self.idx_b = self._get_base_param_indices()

    def load_param(self, config_file):
        """Load parameters from config file."""
        with open(config_file, "r") as f:
            config = yaml.load(f, Loader=yaml.SafeLoader)
        identif_data = config["identification"]
        self.params_settings = get_param_from_yaml(self._robot, identif_data)

    def _get_base_param_indices(self):
        """Find base parameters for cubic spline trajectory"""
        # Generate random trajectory to find base parameters
        n_wps_r = 100
        freq_r = 100
        CB_r = CubicSpline(self._robot, n_wps_r, self.active_joints)
        WP_r = WaypointsGeneration(self._robot, n_wps_r, self.active_joints)
        
        soft_lim_pool = np.array([
            [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
            [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
            [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
        ])
        WP_r.gen_rand_pool(soft_lim_pool)
        
        wps_r, vel_wps_r, acc_wps_r = WP_r.gen_rand_wp()
        tps_r = np.matrix([0.5 * i for i in range(n_wps_r)]).transpose()
        t_r, p_r, v_r, a_r = CB_r.get_full_config(freq_r, tps_r, wps_r, vel_wps_r, acc_wps_r)
        
        W = build_regressor_basic(self._robot, p_r, v_r, a_r, self.params_settings)
        params_std = get_standard_parameters(self.model, self.params_settings)
        idx_e_, par_r_ = get_index_eliminate(W, params_std, tol_e=0.001)
        W_e_ = build_regressor_reduced(W, idx_e_)
        idx_base_ = get_baseIndex(W_e_, par_r_)
        return idx_e_, idx_base_

    def _build_base_regressor(self, q, v, a, W_stack=None):
        """Build base regressor for given data"""
        W = build_regressor_basic(self._robot, q, v, a, self.params_settings)
        W_e_ = build_regressor_reduced(W, self.idx_e)
        W_b_ = build_baseRegressor(W_e_, self.idx_b)
        
        if isinstance(W_stack, np.ndarray):
            W_b_ = np.vstack((W_stack, W_b_))
        return W_b_

    def _objective_func(self, X, opt_cb, tps, vel_wps, acc_wps, wp_init, W_stack=None):
        """Objective function for optimization - minimize condition number"""
        import time
        
        # Convert waypoints to full trajectory
        X = np.array(X)
        n_wps = len(tps)
        wps_X = np.reshape(X, (n_wps - 1, len(self.active_joints)))
        wps = np.vstack((wp_init, wps_X))
        wps = wps.transpose()

        # Generate cubic spline trajectory
        CB = CubicSpline(self._robot, n_wps, self.active_joints)
        freq = 100
        t_f, p_f, v_f, a_f = CB.get_full_config(freq, tps, wps, vel_wps, acc_wps)
        
        # Store in callback
        opt_cb["t_f"] = t_f
        opt_cb["p_f"] = p_f
        opt_cb["v_f"] = v_f
        opt_cb["a_f"] = a_f

        # Build stacked base regressor
        W_b = self._build_base_regressor(p_f, v_f, a_f, W_stack)
        return np.linalg.cond(W_b)

    def solve(self, n_wps=5, stack_reps=2, t_s=2):
        """Generate optimal trajectory using iterative optimization."""
        import time
        import cyipopt
        import numdifftools as nd
        
        print(f"Generating optimal trajectory with {n_wps} waypoints and {stack_reps} segments...")
        
        # Initialize trajectory parameters
        freq = 100
        soft_lim = 0.05
        
        CB = CubicSpline(self._robot, n_wps, self.active_joints, soft_lim)
        WP = WaypointsGeneration(self._robot, n_wps, self.active_joints, soft_lim)
        
        soft_lim_pool = np.array([
            [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
            [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
            [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
        ])
        WP.gen_rand_pool(soft_lim_pool)
        
        # Initialize starting waypoint
        wp_init = np.zeros(len(CB.act_idxq))
        for idx in range(len(CB.act_idxq)):
            wp_init[idx] = np.random.choice(WP.pool_q[:, idx], 1)
        
        W_stack = None
        T_F, P_F, V_F, A_F = [], [], [], []
        
        for s_rep in range(stack_reps):
            print(f"Optimizing segment {s_rep + 1}/{stack_reps}")
            
            # Generate feasible initial trajectory
            count = 0
            is_constr_violated = True
            while is_constr_violated and count < 100:
                count += 1
                wps, vel_wps, acc_wps = WP.gen_rand_wp(wp_init, np.zeros(len(CB.act_idxv)), np.zeros(len(CB.act_idxv)))
                tps = t_s * s_rep + np.matrix([t_s * i_wp for i_wp in range(n_wps)]).transpose()
                
                t_i, p_i, v_i, a_i = CB.get_full_config(freq, tps, wps, vel_wps, acc_wps)
                tau_i = calc_torque(p_i.shape[0], self._robot, p_i, v_i, a_i, self.params_settings)
                tau_i = np.reshape(tau_i, (v_i.shape[1], v_i.shape[0])).transpose()
                is_constr_violated = CB.check_cfg_constraints(p_i, v_i, tau_i)
            
            if is_constr_violated:
                print(f"Could not find feasible initial trajectory for segment {s_rep + 1}")
                break
                
            # Store results
            T_F.append(t_i)
            P_F.append(p_i[:, CB.act_idxq])
            V_F.append(v_i[:, CB.act_idxv])
            A_F.append(a_i[:, CB.act_idxv])
            
            # Update for next segment
            wp_init = wps[:, -1]
            W_stack = self._build_base_regressor(p_i, v_i, a_i, W_stack)
            print(f"Segment {s_rep + 1} completed. Regressor size: {W_stack.shape}")
        
        self.T_F = T_F
        self.P_F = P_F
        self.V_F = V_F
        self.A_F = A_F
        
        print("Optimal trajectory generation completed!")
        return True


def main():
    return 0


if __name__ == "__main__":
    from figaroh.tools.robot import load_robot
    tiago = load_robot("data/urdf/tiago_hey5.urdf")
    tiago_calib = TiagoCalibration(tiago, "config/tiago_config.yaml")
    tiago_calib.initialize()
    tiago_calib.solve()
    tiago_calib.plot()
    # write_to_xacro(
    #     tiago_calib,
    #     file_name="tiago_master_calibration.yaml",
    #     file_type="yaml",
    # )
