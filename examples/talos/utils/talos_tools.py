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

import numpy as np

from figaroh.calibration.calibration_tools import (
    calc_updated_fkm,
    initialize_variables,
)
# Import base class from figaroh
from figaroh.calibration.base_calibration import BaseCalibration


class TALOSCalibration(BaseCalibration):
    """
    Class for calibrating the TALOS humanoid robot's torso-arm system.
    
    This class provides TALOS-specific calibration functionality for the
    torso-arm kinematic chain, extending the BaseCalibration class with
    robot-specific cost functions and TALOS-specific initialization.
    """
    
    def __init__(self, robot, config_file="config/talos_config.yaml",
                 del_list=[]):
        """Initialize TALOS calibration with robot model and configuration.
        
        Args:
            robot: TALOS robot model loaded with FIGAROH
            config_file: Path to TALOS configuration YAML file
            del_list: List of sample indices to exclude from calibration
        """
        super().__init__(robot, config_file, del_list)
        
        # TALOS-specific initialization
        self.regularization_coefficient = 1e-3

    def initialize_variables(self, mode=0, base_position=None):
        """
        Initialize calibration variables for TALOS.
        
        Args:
            mode (int): Initialization mode (0=zeros, 1=random)
            base_position (array_like, optional): Initial base position
            
        Returns:
            tuple: (initial_variables, n_variables)
        """
        var_0, nvars = initialize_variables(self.calib_config, mode=mode)
        
        # Set TALOS-specific base position if provided
        if base_position is not None:
            var_0[:3] = base_position
        else:
            # Default TALOS base position
            var_0[:3] = np.array([-0.16, 0.047, 0.16])
            
        return var_0, nvars
    
    def cost_function(self, var):
        """
        TALOS-specific cost function for torso-arm calibration.
        
        Implements regularization for intermediate kinematic parameters
        while excluding base and marker parameters from regularization.
        This is particularly important for TALOS due to its complex
        kinematic structure.
        
        Args:
            var (ndarray): Parameter vector to evaluate
            
        Returns:
            ndarray: Weighted residual vector including regularization terms
        """
        # Calculate forward kinematics with current parameters
        PEEe = calc_updated_fkm(
            self.model, self.data, var, self.q_measured, self.calib_config
        )
        
        # Main residual: difference between measured and estimated poses
        main_residuals = self.PEE_measured - PEEe
        
        # TALOS-specific regularization
        # Exclude base position (first 3 params) and markers (last 3*NbMarkers)
        n_markers = self.calib_config["NbMarkers"]
        regularization_params = var[6:-n_markers * 3]
        regularization_term = (
            np.sqrt(self.regularization_coefficient) * regularization_params
        )
        
        # Combine residuals
        total_residuals = np.append(main_residuals, regularization_term)
        
        return total_residuals