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
from typing import List

from .base_identification import BaseIdentification
from .base_optimal_calibration import BaseOptimalCalibration
from figaroh.calibration.calibration_tools import BaseCalibration
from figaroh.calibration.calibration_tools import (
    calc_updated_fkm,
)


class TiagoCalibration(BaseCalibration):
    """
    Class for calibrating the TIAGo robot.
    
    This class provides TIAGo-specific calibration functionality by extending
    the BaseCalibration class with robot-specific cost functions and
    initialization parameters.
    """
    def __init__(self, robot, config_file, del_list=[]):
        super().__init__(robot, config_file, del_list)

    def cost_function(self, var):
        """
        TIAGo-specific cost function for the optimization problem.
        
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
        # This helps stabilize optimization for redundant kinematic chains
        n_base_params = 6  # Base frame parameters
        n_tip_params = (self.calib_config["NbMarkers"] *
                        self.calib_config["calibration_index"])
        regularization_params = var[n_base_params : -n_tip_params]
        regularization_residuals = np.sqrt(coeff_) * regularization_params
        
        # Combine residuals
        res_vect = np.append(position_residuals, regularization_residuals)
        return res_vect


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
    
    def process_torque_data(self, tau):
        """Process torque data with TIAGo-specific motor constants."""
        import pinocchio as pin
        
        # Apply TIAGo-specific torque processing (reduction ratios, etc.)
        pin.computeSubtreeMasses(self.robot.model, self.robot.data)
        tau_processed = tau.copy()
        
        for i, joint_name in enumerate(self.identif_config["active_joints"]):
            if joint_name == "torso_lift_joint":
                tau_processed[:, i] = (
                    self.identif_config["reduction_ratio"][joint_name]
                    * self.identif_config["kmotor"][joint_name]
                    * tau[:, i]
                    + 9.81 * self.robot.data.mass[
                        self.robot.model.getJointId(joint_name)
                    ]
                )
            else:
                tau_processed[:, i] = (
                    self.identif_config["reduction_ratio"][joint_name]
                    * self.identif_config["kmotor"][joint_name]
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
            q, qd, qdd, self.model, self.data, self.identif_config
        )
        return regressor


class TiagoOptimalCalibration(BaseOptimalCalibration):
    """TIAGo-specific optimal configuration generation for calibration."""
    
    def __init__(self, robot, config_file="config/tiago_config.yaml"):
        """Initialize TIAGo optimal calibration."""
        super().__init__(robot, config_file)
        print("TIAGo Optimal Calibration initialized")



# Import trajectory optimization classes from base module
from .base_optimal_trajectory import (
    ConfigurationManager,
    BaseParameterComputer,
    TrajectoryConstraintManager,
    BaseOptimalTrajectory,
    BaseTrajectoryIPOPTProblem
)


class OptimalTrajectoryIPOPT(BaseOptimalTrajectory):
    """
    TIAGo-specific optimal trajectory generation using IPOPT.
    
    This class extends the BaseOptimalTrajectory to provide TIAGo-specific
    configuration and problem setup.
    """
    
    def __init__(self, robot, active_joints: List[str], 
                 config_file: str = "config/tiago_config.yaml"):
        """Initialize the TIAGo optimal trajectory generator."""
        super().__init__(robot, active_joints, config_file)
        self.logger.info("TIAGo OptimalTrajectoryIPOPT initialized")
    
    def _create_ipopt_problem(self, n_joints, n_wps, Ns, tps, vel_wps, acc_wps, 
                              wp_init, vel_wp_init, acc_wp_init, W_stack):
        """Create TIAGo-specific IPOPT problem instance."""
        return TiagoTrajectoryIPOPTProblem(
            self, n_joints, n_wps, Ns, tps, vel_wps, acc_wps,
            wp_init, vel_wp_init, acc_wp_init, W_stack
        )


class TiagoTrajectoryIPOPTProblem(BaseTrajectoryIPOPTProblem):
    """
    TIAGo-specific IPOPT problem formulation for trajectory optimization.
    
    This class extends the BaseTrajectoryIPOPTProblem with TIAGo-specific
    configurations and constraints.
    """
    
    def __init__(self, opt_traj, n_joints, n_wps, Ns, tps, vel_wps, acc_wps, 
                 wp_init, vel_wp_init, acc_wp_init, W_stack):
        super().__init__(
            opt_traj, n_joints, n_wps, Ns, tps, vel_wps, acc_wps,
            wp_init, vel_wp_init, acc_wp_init, W_stack, 
            "TiagoTrajectoryOptimization"
        )
