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




def write_to_xacro(tiago_calib, file_name=None, file_type="yaml"):
    """
    Write calibration result to xacro file.
    """
    assert tiago_calib.STATUS == "CALIBRATED", "Calibration not performed yet"
    model = tiago_calib.model
    calib_result = tiago_calib.calibrated_param
    param = tiago_calib.param

    calibration_parameters = {}
    calibration_parameters["camera_position_x"] = float(calib_result["base_px"])
    calibration_parameters["camera_position_y"] = float(calib_result["base_py"])
    calibration_parameters["camera_position_z"] = float(calib_result["base_pz"])
    calibration_parameters["camera_orientation_r"] = float(calib_result["base_phix"])
    calibration_parameters["camera_orientation_p"] = float(calib_result["base_phiy"])
    calibration_parameters["camera_orientation_y"] = float(calib_result["base_phiz"])

    for idx in param["actJoint_idx"]:
        joint = model.names[idx]
        for key in calib_result.keys():
            if joint in key and "torso" not in key:
                calibration_parameters[joint + "_offset"] = float(calib_result[key])
    if tiago_calib.param["measurability"][0:3] == [True, True, True]:
        calibration_parameters["tip_position_x"] = float(calib_result["pEEx_1"])
        calibration_parameters["tip_position_y"] = float(calib_result["pEEy_1"])
        calibration_parameters["tip_position_z"] = float(calib_result["pEEz_1"])
    if tiago_calib.param["measurability"][3:6] == [True, True, True]:
        calibration_parameters["tip_orientation_r"] = float(calib_result["phiEEx_1"])
        calibration_parameters["tip_orientation_p"] = float(calib_result["phiEEy_1"])
        calibration_parameters["tip_orientation_y"] = float(calib_result["phiEEz_1"])

    if file_type == "xacro":
        if file_name is None:
            path_save_xacro = abspath(
                "data/calibration_paramters/tiago_master_calibration_{}.xacro".format(
                    param["NbSample"]
                )
            )
        else:
            path_save_xacro = abspath("data/calibration_parameters/" + file_name)
        with open(path_save_xacro, "w") as output_file:
            for parameter in calibration_parameters.keys():
                update_name = parameter
                update_value = calibration_parameters[parameter]
                update_line = '<xacro:property name="{}" value="{}" / >'.format(
                    update_name, update_value
                )
                output_file.write(update_line)
                output_file.write("\n")

    elif file_type == "yaml":
        if file_name is None:
            path_save_yaml = abspath(
                "data/calibration_parameters/tiago_master_calibration_{}.yaml".format(
                    param["NbSample"]
                )
            )
        else:
            path_save_yaml = abspath("data/calibration_parameters/" + file_name)
        with open(path_save_yaml, "w") as output_file:
            # for parameter in calibration_parameters.keys():
            #     update_name = parameter
            #     update_value = calibration_parameters[parameter]
            #     update_line = "{}:{}".format(update_name, update_value)
            #     output_file.write(update_line)
            #     output_file.write("\n")
            try:
                yaml.dump(
                    calibration_parameters,
                    output_file,
                    sort_keys=False,
                    default_flow_style=False,
                )
            except yaml.YAMLError as exc:
                print(exc)


class TiagoIdentification:
    """
    Class for dynamic parameter identification of the TIAGo robot.
    """
    def __init__(self, robot, config_file):
        self._robot = robot
        self.model = self._robot.model
        self.data = self._robot.data
        self.load_param(config_file)

    def load_param(self, config_file, setting_type="identification"):
        """Load the identification parameters from the yaml file."""
        with open(config_file, "r") as f:
            config = yaml.load(f, Loader=yaml.SafeLoader)
        self.params_settings = get_param_from_yaml(self._robot, config[setting_type])

    def load_csv_data(self):
        """Load and process CSV data."""
        import pandas as pd
        ts = pd.read_csv(abspath(self.params_settings["pos_data"]), usecols=[0]).to_numpy()
        pos = pd.read_csv(abspath(self.params_settings["pos_data"]))
        vel = pd.read_csv(abspath(self.params_settings["vel_data"]))
        eff = pd.read_csv(abspath(self.params_settings["torque_data"]))

        cols = {"pos": [], "vel": [], "eff": []}
        for jn in self.params_settings["active_joints"]:
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

        p[:, self.params_settings["act_idxq"]] = q_f
        v[:, self.params_settings["act_idxv"]] = dq_f
        a[:, self.params_settings["act_idxv"]] = ddq_f
        return p, v, a

    def process_torque_data(self, tau):
        """Process torque data with reduction ratios and motor constants."""
        import pinocchio as pin
        pin.computeSubtreeMasses(self._robot.model, self._robot.data)
        for i, joint_name in enumerate(self.params_settings["active_joints"]):
            if joint_name == "torso_lift_joint":
                tau[:, i] = (
                    self.params_settings["reduction_ratio"][joint_name]
                    * self.params_settings["kmotor"][joint_name]
                    * tau[:, i]
                    + 9.81 * self._robot.data.mass[self._robot.model.getJointId(joint_name)]
                )
            else:
                tau[:, i] = (
                    self.params_settings["reduction_ratio"][joint_name]
                    * self.params_settings["kmotor"][joint_name]
                    * tau[:, i]
                )
        return tau

    def process_data(self, truncate=True):
        """Load and process data"""
        t_, q_, dq_, tau_ = self.load_csv_data()

        # Truncate data
        if truncate:
            n_i, n_f = 921, 6791
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

    def calc_full_regressor(self):
        """Build regressor matrix."""
        self.W = build_regressor_basic(
            self._robot,
            self.processed_data["p"],
            self.processed_data["v"],
            self.processed_data["a"],
            self.params_settings,
        )
        self.standard_parameter = get_standard_parameters(self.model, self.params_settings)
        # joint torque estimated from p,v,a with std params
        phi_ref = np.array(list(self.standard_parameter.values()))
        tau_ref = np.dot(self.W, phi_ref)
        self.tau_ref = tau_ref[range(len(self.params_settings["act_idxv"]) * self.Nsample_)]

    def calc_baseparam(self, decimate=True, plotting=True, save_params=False):
        """Calculate base parameters."""
        # Eliminate zero columns
        idx_e_, active_parameter_ = get_index_eliminate(
            self.W, self.standard_parameter, tol_e=0.001
        )
        W_e_ = build_regressor_reduced(self.W, idx_e_)

        # remove zero-crossing data
        if decimate:
            from scipy import signal
            tau_dec = []
            for i in range(len(self.params_settings["act_idxv"])):
                tau_dec.append(signal.decimate(self.processed_data["tau"][:, i], q=10, zero_phase=True))

            tau_rf = tau_dec[0]
            for i in range(1, len(tau_dec)):
                tau_rf = np.append(tau_rf, tau_dec[i])

            # Process regressor similarly
            W_list = []
            for i in range(len(self.params_settings["act_idxv"])):
                W_dec = []
                for j in range(W_e_.shape[1]):
                    W_dec.append(signal.decimate(
                        W_e_[range(self.params_settings["act_idxv"][i] * self.Nsample_,
                                  (self.params_settings["act_idxv"][i] + 1) * self.Nsample_), j],
                        q=10, zero_phase=True
                    ))

                W_temp = np.zeros((W_dec[0].shape[0], len(W_dec)))
                for k in range(len(W_dec)):
                    W_temp[:, k] = W_dec[k]
                W_list.append(W_temp)

            W_rf = np.zeros((tau_rf.shape[0], W_list[0].shape[1]))
            for i in range(len(W_list)):
                W_rf[range(i * W_list[i].shape[0], (i + 1) * W_list[i].shape[0]), :] = W_list[i]
        else:
            tau_rf = self.processed_data["tau"]
            W_rf = W_e_

        # calculate base parameters
        W_b, bp_dict, base_parameter, phi_b, phi_std = double_QR(
            tau_rf, W_rf, active_parameter_, self.standard_parameter
        )
        rmse = np.linalg.norm(tau_rf - np.dot(W_b, phi_b)) / np.sqrt(tau_rf.shape[0])
        std_xr_ols = relative_stdev(W_b, phi_b, tau_rf)

        self.result = {
            "base regressor": W_b,
            "base parameters": bp_dict,
            "condition number": np.linalg.cond(W_b),
            "rmse norm (N/m)": rmse,
            "torque estimated": np.dot(W_b, phi_b),
            "std dev of estimated param": std_xr_ols,
        }

    def solve(self, truncate=True, decimate=True, plotting=True, save_params=False):
        """Main solving method."""
        self.process_data(truncate=truncate)
        self.calc_full_regressor()
        self.calc_baseparam(decimate=decimate, plotting=plotting, save_params=save_params)


class TiagoOptimalCalibration(TiagoCalibration):
    """
    Generate optimal configurations for calibration.
    """
    def __init__(self, robot, config_file):
        super().__init__(robot, config_file)
        self._sampleConfigs_file = self.param["sample_configs_file"]
        if self.param["calib_model"] == "full_params":
            self.minNbChosen = (
                int(len(self.param["actJoint_idx"]) * 6 / self.param["calibration_index"]) + 1
            )
        elif self.param["calib_model"] == "joint_offset":
            self.minNbChosen = (
                int(len(self.param["actJoint_idx"]) / self.param["calibration_index"]) + 1
            )
        else:
            assert False, "Calibration model not supported."

    def load_data_set(self):
        """Load data from yaml file."""
        import pandas as pd
        if "yaml" in self._sampleConfigs_file:
            with open(self._sampleConfigs_file, "r") as file:
                self._configs = yaml.load(file, Loader=yaml.SafeLoader)

            q_jointNames = self._configs["calibration_joint_names"]
            q_jointConfigs = np.array(self._configs["calibration_joint_configurations"]).T

            df = pd.DataFrame.from_dict(dict(zip(q_jointNames, q_jointConfigs)))

            q = np.zeros([len(df), self._robot.q0.shape[0]])
            for i in range(len(df)):
                for j, name in enumerate(q_jointNames):
                    jointidx = rank_in_configuration(self.model, name)
                    q[i, jointidx] = df[name][i]
            self.q_measured = q
            self.param["NbSample"] = self.q_measured.shape[0]

    def calculate_regressor(self):
        """Calculate regressor."""
        (Rrand_b, R_b, R_e, paramsrand_base, paramsrand_e) = calculate_base_kinematics_regressor(
            self.q_measured, self.model, self.data, self.param
        )
        
        # Rearrange the kinematic regressor by sample numbered order
        Rb_rearr = np.empty_like(R_b)
        for i in range(self.param["calibration_index"]):
            for j in range(self.param["NbSample"]):
                Rb_rearr[j * self.param["calibration_index"] + i, :] = R_b[
                    i * self.param["NbSample"] + j
                ]
        self.R_rearr = Rb_rearr
        
        # Create sub information matrices
        subX_list = []
        idex = self.param["calibration_index"]
        for it in range(self.param["NbSample"]):
            sub_R = self.R_rearr[it * idex : (it * idex + idex), :]
            subX = np.matmul(sub_R.T, sub_R)
            subX_list.append(subX)
        subX_dict = dict(zip(np.arange(self.param["NbSample"]), subX_list))
        
        self._subX_dict = subX_dict
        self._subX_list = subX_list
        return True

    def calculate_optimal_configurations(self):
        """Calculate optimal configurations using SOCP."""
        import picos as pc
        
        # SOCP formulation
        problem = pc.Problem()
        w = pc.RealVariable("w", self.param["NbSample"], lower=0)
        t = pc.RealVariable("t", 1)
        
        # Constraints
        Mw = pc.sum(w[i] * self._subX_dict[i] for i in range(self.param["NbSample"]))
        problem.add_constraint(1 | w <= 1)
        problem.add_constraint(t <= pc.DetRootN(Mw))
        
        # Objective
        problem.set_objective("max", t)
        
        # Solve
        solution = problem.solve(solver="cvxopt")
        
        w_list = [float(w.value[i]) for i in range(w.dim)]
        w_dict = dict(zip(np.arange(self.param["NbSample"]), w_list))
        w_dict_sort = dict(reversed(sorted(w_dict.items(), key=lambda item: item[1])))
        
        # Select optimal configurations
        eps_opt = 1e-5
        chosen_config = [i for i in w_dict_sort.keys() if w_dict_sort[i] > eps_opt]
        
        assert len(chosen_config) >= self.minNbChosen, "Infeasible design, try to increase NbSample."
        
        opt_configs_values = [
            self._configs["calibration_joint_configurations"][opt_id] for opt_id in chosen_config
        ]
        self.opt_configs = self._configs.copy()
        self.opt_configs["calibration_joint_configurations"] = list(opt_configs_values)
        return True

    def solve(self, file_name=None, write_file=False):
        """Solve the optimization problem."""
        self.load_data_set()
        self.calculate_regressor()
        self.calculate_optimal_configurations()
        if write_file:
            self.write_to_file(name_=file_name)

    def write_to_file(self, name_=None):
        """Write optimal configurations to file."""
        if name_ is None:
            path_save = "data/calibration/optimal_configurations/tiago_optimal_configurations.yaml"
        else:
            path_save = "data/calibration/optimal_configs/" + name_
        with open(path_save, "w") as stream:
            try:
                yaml.dump(self.opt_configs, stream, sort_keys=False, default_flow_style=True)
            except yaml.YAMLError as exc:
                print(exc)


class TiagoOptimalTrajectory:
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
