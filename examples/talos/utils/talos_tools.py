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

from __future__ import annotations

from typing import Optional, Tuple

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

    def __init__(
        self,
        robot,
        config_file: str = "config/talos_config.yaml",
        del_list: list = [],
    ) -> None:
        """Initialize TALOS calibration with robot model and configuration.

        Args:
            robot: TALOS robot model loaded with FIGAROH
            config_file: Path to TALOS configuration YAML file
            del_list: List of sample indices to exclude from calibration
        """
        super().__init__(robot, config_file, del_list)

    def initialize_variables(
        self, mode: int = 0, base_position: Optional[np.ndarray] = None
    ) -> Tuple[np.ndarray, int]:
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

    def cost_function(self, var: np.ndarray) -> np.ndarray:
        """
        TALOS-specific cost function for torso-arm calibration.

        SE3 log-map residuals of the measured hand point. No regularisation
        rows: priors on the joint parameters come from core's
        ``estimation.method: map`` (figaroh-plus#120).

        Args:
            var (ndarray): Parameter vector to evaluate

        Returns:
            ndarray: Residual vector
        """
        # Calculate forward kinematics with current parameters
        PEEe = calc_updated_fkm(
            self.model, self.data, var, self.q_measured, self.calib_config
        )

        # Main residual: SE3 log map (geometrically correct pose error)
        return self._compute_logmap_residuals(self.PEE_measured, PEEe)
