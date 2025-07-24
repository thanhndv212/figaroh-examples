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

from figaroh.calibration.calibration_tools import (
    load_data,
    calc_updated_fkm,
    get_LMvariables,
    BaseCalibration,
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


def main():
    return 0


if __name__ == "__main__":
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
