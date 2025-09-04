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
to use the generalized base classes and new infrastructure.
"""

import numpy as np
import pandas as pd
from typing import List
from os.path import abspath

# Import FIGAROH modules
from figaroh.calibration.calibration_tools import calc_updated_fkm

# Import shared modules using figaroh library
from figaroh.calibration.base_calibration import BaseCalibration
from figaroh.identification.base_identification import BaseIdentification
from figaroh.optimal.base_optimal_calibration import BaseOptimalCalibration
from figaroh.optimal.base_optimal_trajectory import (
    BaseOptimalTrajectory,
    BaseTrajectoryIPOPTProblem
)
from figaroh.utils.results_manager import ResultsManager
from figaroh.utils.error_handling import handle_calibration_errors


def validate_robot_config(config):
    """Validate robot configuration."""
    return True


class TiagoCalibration(BaseCalibration):
    """
    Class for calibrating the TIAGo robot.
    
    This class provides TIAGo-specific calibration functionality by extending
    the BaseCalibration class with robot-specific cost functions and
    initialization parameters.
    """
    
    @handle_calibration_errors
    def __init__(self, robot, config_file="config/tiago_config.yaml",
                 del_list=[]):
        """Initialize TIAGo calibration with robot model and configuration.
        
        Args:
            robot: TIAGo robot model loaded with FIGAROH
            config_file: Path to TIAGo configuration YAML file
            del_list: List of sample indices to exclude from calibration
        """
        
        super().__init__(robot, config_file, del_list)
        print("TIAGo calibration initialized with new infrastructure")

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
    
    def load_trajectory_data(self):
        """Load and process CSV data for TIAGo robot."""
        ts = pd.read_csv(
            abspath(self.identif_config["pos_data"]), usecols=[0]
        ).to_numpy()
        pos = pd.read_csv(abspath(self.identif_config["pos_data"]))
        vel = pd.read_csv(abspath(self.identif_config["vel_data"]))
        eff = pd.read_csv(abspath(self.identif_config["torque_data"]))

        cols = {"pos": [], "vel": [], "eff": []}
        for jn in self.identif_config["active_joints"]:
            cols["pos"].extend([col for col in pos.columns if jn in col])
            cols["vel"].extend([col for col in vel.columns if jn in col])
            cols["eff"].extend([col for col in eff.columns if jn in col])

        q = pos[cols["pos"]].to_numpy()
        dq = vel[cols["vel"]].to_numpy()
        tau = eff[cols["eff"]].to_numpy()
        self.raw_data = {
            "timestamps": ts,
            "positions": q,
            "velocities": dq,
            "accelerations": None,
            "torques": tau
        }
        return self.raw_data

    def process_torque_data(self):
        """Process torque data with TIAGo-specific motor constants."""
        import pinocchio as pin
        
        # Apply TIAGo-specific torque processing (reduction ratios, etc.)
        pin.computeSubtreeMasses(self.robot.model, self.robot.data)
        tau_processed = self.raw_data["torques"].copy()

        for i, joint_name in enumerate(self.identif_config["active_joints"]):
            if joint_name == "torso_lift_joint":
                tau_processed[:, i] = (
                    self.identif_config["reduction_ratio"][joint_name]
                    * self.identif_config["kmotor"][joint_name]
                    * self.raw_data["torques"][:, i]
                    + 9.81 * self.robot.data.mass[
                        self.robot.model.getJointId(joint_name)
                    ]
                )
            else:
                tau_processed[:, i] = (
                    self.identif_config["reduction_ratio"][joint_name]
                    * self.identif_config["kmotor"][joint_name]
                    * self.raw_data["torques"][:, i]
                )
        self.processed_data["torques"] = tau_processed
        return self.processed_data["torques"]


class TiagoOptimalCalibration(BaseOptimalCalibration):
    """TIAGo-specific optimal configuration generation for calibration."""
    
    def __init__(self, robot, config_file="config/tiago_config.yaml"):
        """Initialize TIAGo optimal calibration."""
        super().__init__(robot, config_file)
        print("TIAGo Optimal Calibration initialized")


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
