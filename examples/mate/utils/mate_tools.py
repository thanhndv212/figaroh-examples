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
import numpy as np
import yaml

from figaroh.calibration.calibration_tools import (
    calc_updated_fkm,
)
from ...shared.base_calibration import BaseCalibration


class MateCalibration(BaseCalibration):
    def __init__(self, robot, config_file, del_list=[]):
        super().__init__(robot, config_file, del_list)

    def cost_function(self, var):
        """
        MATE-specific cost function for the optimization problem.
        
        Implements regularization for intermediate parameters to improve
        numerical stability and convergence.
        
        Args:
            var (ndarray): Parameter vector to evaluate
            
        Returns:
            ndarray: Residual vector including regularization terms
        """
        coeff_ = self.calib_config["coeff_regularize"]
        PEEe = calc_updated_fkm(self.model, self.data, var,
                                self.q_measured, self.calib_config)
        
        # Main residual: difference between measured and estimated poses
        position_residuals = self.PEE_measured - PEEe
        
        # Regularization term for intermediate parameters (excludes base/tip)
        n_base_params = 6  # Base frame parameters
        n_tip_params = (self.calib_config["NbMarkers"] *
                        self.calib_config["calibration_index"])
        regularization_params = var[n_base_params : -n_tip_params]
        regularization_residuals = np.sqrt(coeff_) * regularization_params
        
        # Combine residuals
        res_vect = np.append(position_residuals, regularization_residuals)
        return res_vect




def write_to_xacro(tiago_calib, file_name=None, file_type="yaml"):
    """
    Write calibration result to xacro file.
    """
    assert tiago_calib.STATUS == "CALIBRATED", "Calibration not performed yet"
    model = tiago_calib.model
    calib_result = tiago_calib.calibrated_param
    calib_config = tiago_calib.calib_config

    calibration_parameters = {}
    calibration_parameters["camera_position_x"] = float(calib_result["base_px"])
    calibration_parameters["camera_position_y"] = float(calib_result["base_py"])
    calibration_parameters["camera_position_z"] = float(calib_result["base_pz"])
    calibration_parameters["camera_orientation_r"] = float(calib_result["base_phix"])
    calibration_parameters["camera_orientation_p"] = float(calib_result["base_phiy"])
    calibration_parameters["camera_orientation_y"] = float(calib_result["base_phiz"])

    for idx in calib_config["actJoint_idx"]:
        joint = model.names[idx]
        for key in calib_result.keys():
            if joint in key and "torso" not in key:
                calibration_parameters[joint + "_offset"] = float(calib_result[key])
    if tiago_calib.calib_config["measurability"][0:3] == [True, True, True]:
        calibration_parameters["tip_position_x"] = float(calib_result["pEEx_1"])
        calibration_parameters["tip_position_y"] = float(calib_result["pEEy_1"])
        calibration_parameters["tip_position_z"] = float(calib_result["pEEz_1"])
    if tiago_calib.calib_config["measurability"][3:6] == [True, True, True]:
        calibration_parameters["tip_orientation_r"] = float(calib_result["phiEEx_1"])
        calibration_parameters["tip_orientation_p"] = float(calib_result["phiEEy_1"])
        calibration_parameters["tip_orientation_y"] = float(calib_result["phiEEz_1"])

    if file_type == "xacro":
        if file_name is None:
            path_save_xacro = abspath(
                "data/calibration_paramters/tiago_master_calibration_{}.xacro".format(
                    calib_config["NbSample"]
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
                    calib_config["NbSample"]
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
