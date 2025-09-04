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
Base calibration class for FIGAROH examples.

This module provides the BaseCalibration abstract class extracted from the
FIGAROH library for use in the examples. It implements a comprehensive
framework for robot kinematic calibration.
"""

import numpy as np
import yaml
from yaml.loader import SafeLoader
from os.path import abspath
import matplotlib.pyplot as plt
import logging
from scipy.optimize import least_squares
from abc import ABC
from typing import Optional, List, Dict, Any, Tuple

# Import necessary functions from FIGAROH library
try:
    from figaroh.calibration.calibration_tools import (
        get_param_from_yaml,
        calculate_base_kinematics_regressor,
        add_base_name,
        add_pee_name,
        load_data,
        calc_updated_fkm,
        initialize_variables
    )
except ImportError as e:
    raise ImportError(
        f"Failed to import FIGAROH calibration tools: {e}. "
        "Please ensure FIGAROH is installed and accessible."
    )

# Import from local shared modules
from .error_handling import (
    CalibrationError,
    handle_calibration_errors
)


class BaseCalibration(ABC):
    """
    Abstract base class for robot kinematic calibration.
    
    This class provides a comprehensive framework for calibrating robot
    kinematic parameters using measurement data. It implements the Template
    Method pattern, providing common functionality while allowing
    robot-specific implementations of the cost function.
    
    The calibration process follows these main steps:
    1. Parameter initialization from configuration files
    2. Data loading and validation
    3. Parameter identification using base regressor analysis
    4. Robust optimization with outlier detection and removal
    5. Solution evaluation and validation
    6. Results visualization and export
    
    Key Features:
    - Automatic parameter identification using QR decomposition
    - Robust optimization with iterative outlier removal
    - Unit-aware measurement weighting for position/orientation data
    - Comprehensive solution evaluation and quality metrics
    - Extensible framework for different robot types
    
    Attributes:
        STATUS (str): Current calibration status ("NOT CALIBRATED" or
                     "CALIBRATED")
        LM_result: Optimization result from scipy.optimize.least_squares
        var_ (ndarray): Calibrated parameter values
        evaluation_metrics (dict): Solution quality metrics
        std_dev (list): Standard deviations of calibrated parameters
        std_pctg (list): Standard deviation percentages
        PEE_measured (ndarray): Measured end-effector poses/positions
        q_measured (ndarray): Measured joint configurations
        calib_config (dict): Calibration parameters and configuration
        model: Robot kinematic model (Pinocchio)
        data: Robot data structure (Pinocchio)
    
    Example:
        >>> # Create robot-specific calibration
        >>> class MyRobotCalibration(BaseCalibration):
        ...     def cost_function(self, var):
        ...         PEEe = calc_updated_fkm(self.model, self.data, var,
        ...                                self.q_measured, self.calib_config)
        ...         residuals = self.PEE_measured - PEEe
        ...         return self.apply_measurement_weighting(residuals)
        ...
        >>> # Run calibration
        >>> calibrator = MyRobotCalibration(robot, "config.yaml")
        >>> calibrator.initialize()
        >>> calibrator.solve()
        >>> print(f"RMSE: {calibrator.evaluation_metrics['rmse']:.6f}")
    
    Notes:
        - Derived classes should implement robot-specific cost_function()
        - Default cost_function is provided but issues performance warning
        - Configuration files must follow FIGAROH parameter structure
        - Supports both "full_params" and "joint_offset" calibration models
    
    See Also:
        - TiagoCalibration: TIAGo robot implementation
        - UR10Calibration: Universal Robots UR10 implementation
        - calc_updated_fkm: Forward kinematics computation function
        - apply_measurement_weighting: Unit-aware weighting utility
    """
    
    @handle_calibration_errors
    def __init__(self, robot, config_file: str, del_list: List[int] = None):
        """Initialize robot calibration framework.
        
        Sets up the calibration environment by loading robot model,
        configuration parameters, and preparing internal data structures
        for optimization.
        
        Args:
            robot: Robot object containing kinematic model and data structures.
                  Must have 'model' and 'data' attributes compatible with
                  Pinocchio library.
            config_file (str): Path to YAML configuration file containing
                             calibration parameters, data paths, and settings.
            del_list (list, optional): Indices of bad/outlier samples to
                                     exclude from calibration data.
                                     Defaults to [].
        
        Raises:
            FileNotFoundError: If config_file does not exist
            KeyError: If required parameters missing from configuration
            ValueError: If configuration parameters are invalid
            CalibrationError: If robot or configuration is invalid
            
        Side Effects:
            - Loads and validates configuration parameters
            - Sets initial calibration status to "NOT CALIBRATED"
            - Calculates number of calibration variables
            - Resolves absolute path to measurement data file
            
        Example:
            >>> robot = load_robot_model("tiago.urdf")
            >>> calibrator = TiagoCalibration(robot, "tiago_config.yaml",
            ...                              del_list=[5, 12, 18])
        """
        if del_list is None:
            del_list = []
            
        # Validate inputs
        if not hasattr(robot, 'model') or not hasattr(robot, 'data'):
            raise CalibrationError(
                "Robot must have 'model' and 'data' attributes")
        
        self._robot = robot
        self.model = self._robot.model
        self.data = self._robot.data
        self.del_list_ = del_list
        self.calib_config = None
        self.load_param(config_file)
        self.nvars = len(self.calib_config["param_name"])
        self._data_path = abspath(self.calib_config["data_file"])
        self.STATUS = "NOT CALIBRATED"

    def initialize(self):
        """Initialize calibration data and parameters.
        
        Performs the initialization phase of calibration by:
        1. Loading measurement data from files
        2. Creating parameter list through base regressor analysis
        3. Identifying calibratable parameters using QR decomposition
        
        This method must be called before solve() to prepare the calibration
        problem. It handles data validation, parameter identification, and
        sets up the optimization problem structure.
        
        Raises:
            FileNotFoundError: If measurement data file not found
            ValueError: If data format is invalid or incompatible
            AssertionError: If required data dimensions don't match
            CalibrationError: If initialization fails
            
        Side Effects:
            - Populates self.PEE_measured with measurement data
            - Populates self.q_measured with joint configuration data
            - Updates self.calib_config["param_name"] with identified
              parameters
            - Validates data consistency and dimensions
            
        Example:
            >>> calibrator = TiagoCalibration(robot, "config.yaml")
            >>> calibrator.initialize()
            >>> print(f"Loaded {calibrator.calib_config['NbSample']} samples")
            >>> print(f"Calibrating {len(calibrator.calib_config['param_name'])} "
            ...       f"parameters")
        """
        try:
            self.load_data_set()
            self.create_param_list()
        except Exception as e:
            raise CalibrationError(f"Initialization failed: {e}")

    def solve(self, method="lm", max_iterations=3, outlier_threshold=3.0,
              enable_logging=True, plotting=False, save_results=False):
        """Execute the complete calibration process.
        
        This is the main entry point for calibration that:
        1. Runs the optimization algorithm via solve_optimisation()
        2. Optionally generates visualization plots if enabled
        3. Optionally saves results to files if enabled

        The method serves as a high-level orchestrator for the calibration
        workflow, delegating the actual optimization to solve_optimisation()
        and handling visualization based on user preferences.
        
        Side Effects:
            - Updates calibration parameters through optimization
            - Sets self.STATUS to "CALIBRATED" on successful completion
            - May display plots if self.calib_config["PLOT"] is True
            
        See Also:
            solve_optimisation: Core optimization implementation
            plot: Visualization and analysis plotting
        """
        result, outlier_indices = self.solve_optimisation(method=method, 
                                 max_iterations=max_iterations,
                                 outlier_threshold=outlier_threshold,
                                 enable_logging=enable_logging)
        
        # Evaluate solution
        evaluation = self._evaluate_solution(result, outlier_indices)
        
        # Log final results
        if enable_logging:
            logger.info("="*30)
            logger.info("FINAL CALIBRATION RESULTS")
            logger.info("="*30)
            self._log_iteration_results("FINAL", result, evaluation)
            
            if len(outlier_indices) > 0:
                logger.info(f"Outlier samples: {outlier_indices}")
            logger.info("Calibration completed successfully!")
        
        # Store results
        self._store_optimization_results(result, evaluation, outlier_indices)

        # Generate plots if required
        if plotting:
            self.plot_results()
        if save_results:
            self.save_results()
        return result
    
    def plot_results(self):
        """Generate comprehensive visualization plots for calibration results.
        
        Creates multiple visualization plots to analyze calibration quality:
        1. Error distribution plots showing residual patterns
        2. 3D pose visualizations comparing measured vs predicted poses
        3. Joint configuration analysis (currently commented)
        
        This method provides essential visual feedback for calibration
        assessment, helping users understand solution quality and identify
        potential issues with the calibration process.
        
        Prerequisites:
            - Calibration must be completed (solve() called)
            - Measurement data must be loaded
            - Matplotlib backend must be configured
            
        Side Effects:
            - Displays plots using plt.show()
            - May block execution until plots are closed
            
        See Also:
            plot_errors_distribution: Individual error analysis plots
            plot_3d_poses: 3D pose comparison visualization
        """

        # Use pre-initialized results manager if available
        if hasattr(self, 'results_manager') and \
           self.results_manager is not None:
            try:
                # Plot using unified manager with self.result data
                self.results_manager.plot_calibration_results()
                return
                
            except Exception as e:
                print(f"Error plotting with ResultsManager: {e}")
                print("Falling back to basic plotting...")
        
        # Fallback to basic plotting if ResultsManager not available
        try:
            self.plot_errors_distribution()
            self.plot_3d_poses()
            # self.plot_joint_configurations()
            plt.show()
        except Exception as e:
            print(f"Warning: Plotting failed: {e}")

    def load_param(self, config_file: str, setting_type: str = "calibration"):
        """Load calibration parameters from YAML configuration file.
        
        Reads and parses a YAML configuration file to extract calibration
        settings and robot-specific parameters. The configuration structure
        supports multiple setting types (calibration, identification, etc.)
        within the same file.
        
        Args:
            config_file (str): Path to YAML configuration file containing
                             calibration parameters and robot settings
            setting_type (str): Configuration section to load from the YAML
                              file. Common values include "calibration",
                              "identification", or custom section names
                              
        Raises:
            FileNotFoundError: If config_file does not exist
            yaml.YAMLError: If YAML parsing fails
            KeyError: If setting_type section not found in config
            CalibrationError: If configuration is invalid
            
        Side Effects:
            - Updates self.calib_config with loaded configuration
            - Overwrites any existing parameter settings
            
        Example:
            >>> calibrator = BaseCalibration(robot)
            >>> calibrator.load_param("config/robot_config.yaml")
            >>> # Or load identification parameters
            >>> calibrator.load_param("config/robot_config.yaml",
            ...                       "identification")
        """
        try:
            with open(config_file, "r") as f:
                config = yaml.load(f, Loader=SafeLoader)
            
            if setting_type not in config:
                raise KeyError(f"Setting type '{setting_type}' not found in config")
                
            calib_data = config[setting_type]
            self.calib_config = get_param_from_yaml(self._robot, calib_data)
        except FileNotFoundError:
            raise CalibrationError(f"Configuration file not found: {config_file}")
        except Exception as e:
            raise CalibrationError(f"Failed to load configuration: {e}")

    def create_param_list(self, q: Optional[np.ndarray] = None):
        """Initialize calibration parameter structure and validate setup.
        
        This method sets up the fundamental parameter structure for calibration
        by computing kinematic regressors and ensuring proper frame naming
        conventions. It serves as a critical initialization step that must be
        called before optimization begins.
        
        The method performs several key operations:
        1. Computes base kinematic regressors for parameter identification
        2. Adds default names for unknown base and tip frames
        3. Validates the parameter structure for calibration readiness
        
        Args:
            q (array_like, optional): Joint configuration for regressor
                                    computation. If None, uses empty list
                                    which may limit regressor accuracy
                                    
        Returns:
            bool: Always returns True to indicate successful completion
            
        Side Effects:
            - Updates self.calib_config with frame names if not known
            - Computes and caches kinematic regressors
            - May modify parameter structure for calibration compatibility
            
        Raises:
            ValueError: If robot model is not properly initialized
            AttributeError: If required calibration parameters are missing
            CalibrationError: If parameter creation fails
            
        Example:
            >>> calibrator = BaseCalibration(robot)
            >>> calibrator.load_param("config.yaml")
            >>> calibrator.create_param_list()  # Basic setup
            >>> # Or with specific joint configuration
            >>> q_nominal = np.zeros(robot.nq)
            >>> calibrator.create_param_list(q_nominal)
            
        See Also:
            calculate_base_kinematics_regressor: Core regressor computation
            add_base_name: Base frame naming utilities
            add_pee_name: End-effector frame naming utilities
        """
        if q is None:
            q_ = []
        else:
            q_ = q
            
        try:
            (
                Rrand_b,
                R_b,
                R_e,
                paramsrand_base,
                paramsrand_e,
            ) = calculate_base_kinematics_regressor(
                q_, self.model, self.data, self.calib_config, tol_qr=1e-6
            )
            
            if self.calib_config["known_baseframe"] is False:
                add_base_name(self.calib_config)
            if self.calib_config["known_tipframe"] is False:
                add_pee_name(self.calib_config)
                
            return True
            
        except Exception as e:
            raise CalibrationError(f"Parameter list creation failed: {e}")

    def load_data_set(self):
        """Load experimental measurement data for calibration.
        
        Reads measurement data from the specified data path and processes it
        for calibration use. This includes both pose measurements and
        corresponding joint configurations, with optional data filtering
        based on the deletion list.
        
        The method handles data preprocessing, validation, and formatting
        to ensure compatibility with the calibration algorithms. It serves
        as the primary data ingestion point for the calibration process.
        
        Side Effects:
            - Sets self.PEE_measured with processed pose measurements
            - Sets self.q_measured with corresponding joint configurations
            - Applies data filtering if self.del_list_ is specified
            
        Prerequisites:
            - self._data_path must be set to valid measurement data location
            - Robot model must be initialized
            - Calibration parameters must be loaded
            
        Raises:
            FileNotFoundError: If data files are not found at _data_path
            ValueError: If data format is incompatible or corrupted
            AttributeError: If required attributes are not initialized
            CalibrationError: If data loading fails
            
        See Also:
            load_data: Core data loading and processing function
        """
        try:
            self.PEE_measured, self.q_measured = load_data(
                self._data_path, self.model, self.calib_config, self.del_list_
            )
        except Exception as e:
            raise CalibrationError(f"Data loading failed: {e}")

    def get_pose_from_measure(self, res_: np.ndarray) -> np.ndarray:
        """Calculate forward kinematics with calibrated parameters.
        
        Computes robot end-effector poses using the updated kinematic model
        with calibrated parameters. This method applies the calibration
        results to predict poses for the measured joint configurations.
        
        Args:
            res_ (ndarray): Calibrated parameter vector containing kinematic
                          corrections (geometric parameters, base transform,
                          tool transform, etc.)
                          
        Returns:
            ndarray: Predicted end-effector poses corresponding to the
                    measured joint configurations. Shape depends on the
                    number of measurements and pose representation format.
                    
        Prerequisites:
            - Joint configurations must be loaded (q_measured available)
            - Calibration parameters must be initialized
            - Robot model must be properly configured
            
        Example:
            >>> # After calibration
            >>> calibrated_params = calibrator.LM_result.x
            >>> predicted_poses = calibrator.get_pose_from_measure(
            ...     calibrated_params)
            >>> # Compare with measured poses
            >>> errors = predicted_poses - calibrator.PEE_measured
            
        See Also:
            calc_updated_fkm: Core forward kinematics computation function
        """
        return calc_updated_fkm(
            self.model, self.data, res_, self.q_measured, self.calib_config
        )

    def cost_function(self, var: np.ndarray) -> np.ndarray:
        """Calculate cost function for optimization.
        
        This method provides a default implementation but should be overridden
        by derived classes to define robot-specific cost computation with
        appropriate weighting and regularization.
        
        Args:
            var (ndarray): Parameter vector to evaluate
            
        Returns:
            ndarray: Residual vector
            
        Warning:
            Using default cost function. Consider implementing robot-specific
            cost function for optimal performance.
            
        Example implementation:
            >>> def cost_function(self, var):
            ...     PEEe = calc_updated_fkm(self.model, self.data, var,
            ...                            self.q_measured, self.calib_config)
            ...     raw_residuals = self.PEE_measured - PEEe
            ...     weighted_residuals = self.apply_measurement_weighting(
            ...         raw_residuals, pos_weight=1000.0, orient_weight=100.0)
            ...     # Add regularization if needed
            ...     return weighted_residuals
        """
        import warnings
        
        # Issue warning about using default implementation
        warnings.warn(
            f"Using default cost function for {self.__class__.__name__}. "
            "Consider implementing a robot-specific cost function with "
            "appropriate weighting and regularization for optimal "
            "performance.",
            UserWarning,
            stacklevel=2
        )
        
        # Default implementation: basic residual calculation
        PEEe = calc_updated_fkm(self.model, self.data, var,
                                self.q_measured, self.calib_config)
        raw_residuals = self.PEE_measured - PEEe
        
        # Apply basic measurement weighting if configuration is available
        try:
            weighted_residuals = self.apply_measurement_weighting(
                raw_residuals)
            return weighted_residuals
        except (KeyError, AttributeError):
            # Fallback to unweighted residuals if weighting config unavailable
            return raw_residuals

    def apply_measurement_weighting(self, residuals: np.ndarray, 
                                  pos_weight: Optional[float] = None,
                                  orient_weight: Optional[float] = None) -> np.ndarray:
        """Apply measurement weighting to handle position/orientation units.
        
        This utility method can be used by derived classes to properly weight
        position (meter) and orientation (radian) measurements for equivalent
        influence in the cost function.
        
        Args:
            residuals (ndarray): Raw residual vector
            pos_weight (float, optional): Weight for position residuals.
                                        If None, uses 1/position_std
            orient_weight (float, optional): Weight for orientation residuals.
                                           If None, uses 1/orientation_std
            
        Returns:
            ndarray: Weighted residual vector
            
        Example:
            >>> # In derived class cost_function:
            >>> raw_residuals = self.PEE_measured - PEEe
            >>> weighted_residuals = self.apply_measurement_weighting(
            ...     raw_residuals, pos_weight=1000.0, orient_weight=100.0)
        """
        # Get weights from parameters or use provided values
        if pos_weight is None:
            pos_std = self.calib_config.get("measurement_std", {}).get(
                "position", 0.001)
            pos_weight = 1.0 / pos_std
        
        if orient_weight is None:
            orient_std = self.calib_config.get("measurement_std", {}).get(
                "orientation", 0.01)
            orient_weight = 1.0 / orient_std
        
        weighted_residuals = []
        residual_idx = 0
        
        # Process each sample for each marker
        for marker in range(self.calib_config["NbMarkers"]):
            for dof, is_measured in enumerate(self.calib_config["measurability"]):
                if is_measured:
                    for sample in range(self.calib_config["NbSample"]):
                        res = residuals[residual_idx]
                        if dof < 3:  # Position components (x,y,z)
                            weighted_residuals.append(res * pos_weight)
                        else:  # Orientation components (rx,ry,rz)
                            weighted_residuals.append(res * orient_weight)
                            # print(f"Residual index: {residual_idx}")
                        residual_idx += 1
        return np.array(weighted_residuals)

    def _setup_logging(self):
        """Setup logging configuration for terminal output."""
        # Create logger
        logger = logging.getLogger('calibration')
        logger.setLevel(logging.INFO)
        
        # Clear existing handlers to avoid duplicates
        logger.handlers.clear()
        
        # Create console handler
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        
        # Create formatter
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        console_handler.setFormatter(formatter)
        
        # Add handler to logger
        logger.addHandler(console_handler)
        
        return logger

    def _optimize_with_outlier_removal(self, var_init: np.ndarray,
                                     method: str = "lm",
                                     max_iterations: int = 3,
                                     outlier_threshold: float = 1.0) -> Tuple:
        """Optimize with iterative outlier removal.
        
        Args:
            var_init (ndarray): Initial parameter guess
            max_iterations (int): Maximum outlier removal iterations
            outlier_threshold (float): Threshold for outlier detection in
                                  standard deviations
            
        Returns:
            tuple: (result, outlier_indices, final_residuals)
        """
        logger = logging.getLogger('calibration')
        current_var = var_init.copy()
        outlier_indices = []
        
        for iteration in range(max_iterations):
            logger.info(f"Outlier removal iteration {iteration + 1}")
            
            # Run optimization
            result = least_squares(
                self.cost_function,
                current_var,
                method=method,
                max_nfev=1000
            )
            
            if not result.success:
                logger.warning(
                    f"Optimization failed at iteration {iteration + 1}")
                break
                
            # Calculate residuals and detect outliers
            PEE_est = self.get_pose_from_measure(result.x)
            residuals = PEE_est - self.PEE_measured
            new_outliers = self._detect_outliers(residuals, outlier_threshold)
            
            if len(new_outliers) == 0:
                logger.info("No outliers detected, optimization converged")
                break
                
            outlier_indices.extend(new_outliers)
            outlier_indices = list(set(outlier_indices))  # Remove duplicates
            
            logger.info(f"Detected {len(new_outliers)} new outliers, "
                        f"total outliers: {len(outlier_indices)}")
            
            # Update for next iteration
            current_var = result.x
            
        return result, outlier_indices, residuals

    def _detect_outliers(self, residuals: np.ndarray, threshold: float) -> List[int]:
        """Detect outliers using statistical threshold.
        
        Args:
            residuals (ndarray): Residual vector
            threshold (float): Threshold in standard deviations
            
        Returns:
            list: Indices of detected outliers
        """
        # Reshape residuals to per-sample format
        n_dofs = self.calib_config["calibration_index"]
        n_samples = self.calib_config["NbSample"]
        
        if len(residuals) != n_dofs * n_samples:
            return []
            
        residuals_2d = residuals.reshape((n_dofs, n_samples))
        
        # Calculate RMS error per sample
        rms_errors = np.sqrt(np.mean(residuals_2d**2, axis=0))
        
        # Detect outliers
        mean_error = np.mean(rms_errors)
        std_error = np.std(rms_errors)
        threshold_value = mean_error + threshold * std_error
        
        outliers = np.where(rms_errors > threshold_value)[0].tolist()
        return outliers

    def _evaluate_solution(self, result, outlier_indices: List[int]) -> Dict[str, Any]:
        """Evaluate optimization solution quality.
        
        Args:
            result: Optimization result from scipy.optimize.least_squares
            outlier_indices (list): Indices of detected outliers
            
        Returns:
            dict: Solution evaluation metrics
        """
        PEE_est = self.get_pose_from_measure(result.x)
        residuals = PEE_est - self.PEE_measured
        n_dofs = self.calib_config["calibration_index"]
        n_samples = self.calib_config["NbSample"]
        
        # Calculate metrics
        rmse = np.sqrt(np.mean(residuals**2))
        mae = np.mean(np.abs(residuals))
        max_error = np.max(np.abs(residuals))
        
        # Per-sample metrics
        if len(residuals) == n_dofs * n_samples:
            residuals_2d = residuals.reshape((n_dofs, n_samples))
            sample_rms = np.sqrt(np.mean(residuals_2d**2, axis=0))
            mean_sample_rms = np.mean(sample_rms)
            std_sample_rms = np.std(sample_rms)
        else:
            mean_sample_rms = rmse
            std_sample_rms = 0.0
        
        # Calculate standard deviation of estimated parameters
        self.calc_stddev(result)
        
        return {
            'rmse': rmse,
            'mae': mae,
            'max_error': max_error,
            'mean_sample_rms': mean_sample_rms,
            'std_sample_rms': std_sample_rms,
            "param_stdev": self.std_dev,
            "param_stddev_percentage": self.std_pctg,
            'n_outliers': len(outlier_indices),
            'outlier_percentage': len(outlier_indices) / n_samples * 100,
            'optimization_success': result.success,
            'cost': result.cost,
            'n_iterations': getattr(result, 'nit', 0),
            'n_function_evals': getattr(result, 'nfev', 0)
        }

    def _prepare_next_iteration(self, result, iteration: int) -> np.ndarray:
        """Prepare for next optimization iteration.
        
        Args:
            result: Current optimization result
            iteration (int): Current iteration number
            
        Returns:
            ndarray: Initial guess for next iteration
        """
        if result.success:
            return result.x
        else:
            # If optimization failed, add small random perturbation
            perturbation = np.random.normal(0, 0.001, len(result.x))
            return result.x + perturbation

    def _log_iteration_results(self, iteration, result, evaluation: Dict[str, Any]):
        """Log results for current iteration.
        
        Args:
            iteration (int): Current iteration number
            result: Optimization result
            evaluation (dict): Solution evaluation metrics
        """
        logger = logging.getLogger('calibration')
        
        logger.info(f"Iteration {iteration} Results:")
        logger.info(f"  Success: {evaluation['optimization_success']}")
        logger.info(f"  RMSE: {evaluation['rmse']:.6f}")
        logger.info(f"  MAE: {evaluation['mae']:.6f}")
        logger.info(f"  Max Error: {evaluation['max_error']:.6f}")
        logger.info(f"  Cost: {evaluation['cost']:.6f}")
        logger.info(f"  Function Evaluations: {evaluation['n_function_evals']}")
        logger.info(f"  Outliers: {evaluation['n_outliers']} "
                   f"({evaluation['outlier_percentage']:.1f}%)")

    def _store_optimization_results(self, result, evaluation: Dict[str, Any], 
                                  outlier_indices: List[int]):
        """Store optimization results in instance variables.
        
        Args:
            result: Final optimization result
            evaluation (dict): Solution evaluation metrics
            outlier_indices (list): Detected outlier indices
        """
        # Store main results
        self.LM_result = result
        self.var_ = result.x
        self.uncalib_values = np.zeros_like(result.x)  # Store initial guess
        
        # Store evaluation metrics
        self.evaluation_metrics = evaluation
        self.outlier_indices = outlier_indices
        
        # Calculate per-sample error distribution for plotting
        PEE_est = self.get_pose_from_measure(result.x)
        residuals = PEE_est - self.PEE_measured
        n_dofs = self.calib_config["calibration_index"]
        n_samples = self.calib_config["NbSample"]
        n_markers = self.calib_config["NbMarkers"]
        
        # if len(residuals) == n_dofs * n_samples * n_markers:
        #     residuals_3d = residuals.reshape((n_markers, n_dofs, n_samples))
        #     self._PEE_dist = np.sqrt(np.mean(residuals_3d**2, axis=1))
        if len(residuals) == n_dofs * n_samples:
            residuals_2d = residuals.reshape((n_dofs, n_samples))
            sample_rms = np.sqrt(np.mean(residuals_2d**2, axis=0))
            self._PEE_dist = sample_rms.reshape((1, n_samples))
        else:
            # Fallback for unexpected residual shapes
            self._PEE_dist = np.ones((n_markers, n_samples)) * evaluation['rmse']

        # Reshape PEE measured for consistency
        PEEm_LM2d = self.PEE_measured.reshape(
            (
                n_dofs, n_samples
            )
        )
        PEEe_LM2d = PEE_est.reshape(
            (
                n_dofs, n_samples
            )
        )
        # Store results
        self.results_data = {}
        self.results_data["number of calibrated parameters"] = len(result.x)
        self.results_data["calibrated parameters names"] = self.calib_config["param_name"]
        self.results_data["calibrated parameters values"] = result.x.tolist()
        self.results_data.update(evaluation)
        self.results_data["number of samples"] = n_samples
        self.results_data["rms residuals by samples"] = self._PEE_dist
        self.results_data["residuals"] = residuals_2d.T
        self.results_data["PEE measured (2D array)"] = PEEm_LM2d.T
        self.results_data["PEE estimated (2D array)"] = PEEe_LM2d.T
        self.results_data["outlier indices"] = outlier_indices
        self.results_data["calibration config"] = self.calib_config
        self.results_data["task type"] = "calibration"

        # Initialize ResultsManager for calibration task
        try:
            from .results_manager import ResultsManager
            
            # Get robot name from class or model
            robot_name = getattr(
                self, 'robot_name',
                getattr(
                    self.model, 'name',
                    self.__class__.__name__.lower().replace(
                        'calibration', '')))
            # Initialize results manager for calibration task
            self.results_manager = ResultsManager('calibration', robot_name, self.results_data)
            
        except ImportError as e:
            print(f"Warning: ResultsManager not available: {e}")
            self.results_manager = None

        # Update status
        self.STATUS = "CALIBRATED"

    def solve_optimisation(self, var_init: Optional[np.ndarray] = None,
                          method: str = "lm",
                          max_iterations: int = 3,
                          outlier_threshold: float = 3.0, 
                          enable_logging: bool = False):
        """Solve calibration optimization with robust outlier handling.
        
        This method implements a comprehensive optimization strategy:
        1. Sets up logging for progress tracking
        2. Iteratively removes outliers and re-optimizes
        3. Evaluates solution quality with detailed metrics
        4. Stores results for further analysis
        
        Args:
            var_init (ndarray, optional): Initial parameter guess. If None,
                                        uses zero initialization.
            max_iterations (int): Maximum outlier removal iterations
            outlier_threshold (float): Outlier detection threshold (std devs)
            enable_logging (bool): Whether to enable terminal logging
            
        Raises:
            ValueError: If optimization fails completely
            AssertionError: If required data is not loaded
            CalibrationError: If optimization fails
            
        Side Effects:
            - Updates self.LM_result with optimization results
            - Updates self.STATUS to "CALIBRATED" on success
            - Creates self.evaluation_metrics with quality metrics
            - Sets up logging if enabled
        """
        # Verify prerequisites
        if not hasattr(self, 'PEE_measured'):
            raise CalibrationError("Call load_data_set() first")
        if not hasattr(self, 'q_measured'):
            raise CalibrationError("Call load_data_set() first")
        
        # Setup logging
        if enable_logging:
            logger = self._setup_logging()
            logger.info("Starting calibration optimization")
            logger.info(f"Parameters: {len(self.calib_config['param_name'])}")
            logger.info(f"Parameter names: {self.calib_config['param_name']}")
            logger.info(f"Markers: {self.calib_config['NbMarkers']}")
            logger.info(f"Samples: {self.calib_config['NbSample']}")
            logger.info(f"DOFs: {self.calib_config['calibration_index']}")
        
        # Initialize parameters
        if var_init is None:
            var_init, _ = initialize_variables(self.calib_config, mode=0)
        
        try:
            # Run optimization with outlier removal
            result, outlier_indices, final_residuals = \
                self._optimize_with_outlier_removal(
                    var_init, method, max_iterations, outlier_threshold
                )
            
            return result, outlier_indices
            
        except Exception as e:
            if enable_logging:
                logger = logging.getLogger('calibration')
                logger.error(f"Calibration failed: {str(e)}")
            raise CalibrationError(f"Optimization failed: {str(e)}")

    def calc_stddev(self, result):
        """Calculate parameter uncertainty statistics from optimization results.
        
        Computes standard deviation and percentage uncertainty for each
        calibrated parameter using the covariance matrix derived from the
        Jacobian at the optimal solution. This provides confidence intervals
        and parameter reliability metrics.
        
        The calculation uses the linearized uncertainty propagation:
        σ²(θ) = σ²(residuals) * (J^T J)^-1
        
        Where J is the Jacobian matrix and σ²(residuals) is the residual
        variance estimate.
        
        Prerequisites:
            - Calibration optimization must be completed
            - Jacobian matrix must be available from optimization
            
        Side Effects:
            - Sets self.std_dev with parameter standard deviations
            - Sets self.std_pctg with percentage uncertainties
            
        Raises:
            CalibrationError: If calibration has not been performed
            np.linalg.LinAlgError: If Jacobian matrix is singular or ill-conditioned
            
        Example:
            >>> calibrator.solve()
            >>> calibrator.calc_stddev()
            >>> print(f"Parameter uncertainties: {calibrator.std_dev}")
            >>> print(f"Percentage errors: {calibrator.std_pctg}")
        """
        try:
            sigma_ro_sq = (result.cost**2) / (
                self.calib_config["NbSample"] * self.calib_config["calibration_index"] - self.nvars
            )
            J = result.jac
            C_param = sigma_ro_sq * np.linalg.pinv(np.dot(J.T, J))
            std_dev = []
            std_pctg = []
            for i_ in range(self.nvars):
                std_dev.append(np.sqrt(C_param[i_, i_]))
                if result.x[i_] != 0:
                    std_pctg.append(abs(np.sqrt(C_param[i_, i_]) / result.x[i_]))
                else:
                    std_pctg.append(0.0)
            self.std_dev = std_dev
            self.std_pctg = std_pctg
        except Exception as e:
            raise CalibrationError(f"Standard deviation calculation failed: {e}")

    def plot_errors_distribution(self):
        """Plot error distribution analysis for calibration assessment.
        
        Creates bar plots showing pose error magnitudes across all samples
        and markers. This visualization helps identify problematic
        measurements, assess calibration quality, and detect outliers in
        the dataset.
        
        The plots display error magnitudes (in meters) for each sample,
        with separate subplots for each marker when multiple markers are used.
        
        Prerequisites:
            - Calibration must be completed (STATUS == "CALIBRATED")
            - Error analysis must be computed (self._PEE_dist available)
            
        Side Effects:
            - Creates matplotlib figure with error distribution plots
            - Figure remains open until explicitly closed or plt.show() called
            
        Raises:
            CalibrationError: If calibration has not been performed
            AttributeError: If error analysis data is not available
            
        See Also:
            plot_3d_poses: 3D visualization of pose comparisons
            calc_stddev: Error statistics computation
        """
        if self.STATUS != "CALIBRATED":
            raise CalibrationError("Calibration not performed yet")

        fig1, ax1 = plt.subplots(self.calib_config["NbMarkers"], 1)
        colors = ["blue", "red", "yellow", "purple"]

        if self.calib_config["NbMarkers"] == 1:
            ax1.bar(np.arange(self.calib_config["NbSample"]), self._PEE_dist[0, :])
            ax1.set_xlabel("Sample", fontsize=25)
            ax1.set_ylabel("Error (meter)", fontsize=30)
            ax1.tick_params(axis="both", labelsize=30)
            ax1.grid()
        else:
            for i in range(self.calib_config["NbMarkers"]):
                ax1[i].bar(
                    np.arange(self.calib_config["NbSample"]),
                    self._PEE_dist[i, :],
                    color=colors[i],
                )
                ax1[i].set_xlabel("Sample", fontsize=25)
                ax1[i].set_ylabel("Error of marker %s (meter)" % (i + 1), fontsize=25)
                ax1[i].tick_params(axis="both", labelsize=30)
                ax1[i].grid()

    def plot_3d_poses(self, INCLUDE_UNCALIB: bool = False):
        """Plot 3D poses comparing measured vs estimated poses.
        
        Args:
            INCLUDE_UNCALIB (bool): Whether to include uncalibrated poses
        """
        if self.STATUS != "CALIBRATED":
            raise CalibrationError("Calibration not performed yet")

        fig2 = plt.figure()
        fig2.suptitle("Visualization of estimated poses and measured pose in Cartesian")
        ax2 = fig2.add_subplot(111, projection="3d")
        PEEm_LM2d = self.PEE_measured.reshape(
            (
                self.calib_config["NbMarkers"] * self.calib_config["calibration_index"],
                self.calib_config["NbSample"],
            )
        )
        PEEe_sol = self.get_pose_from_measure(self.LM_result.x)
        PEEe_sol2d = PEEe_sol.reshape(
            (
                self.calib_config["NbMarkers"] * self.calib_config["calibration_index"],
                self.calib_config["NbSample"],
            )
        )
        PEEe_uncalib = self.get_pose_from_measure(self.uncalib_values)
        PEEe_uncalib2d = PEEe_uncalib.reshape(
            (
                self.calib_config["NbMarkers"] * self.calib_config["calibration_index"],
                self.calib_config["NbSample"],
            )
        )
        for i in range(self.calib_config["NbMarkers"]):
            ax2.scatter3D(
                PEEm_LM2d[i * 3, :],
                PEEm_LM2d[i * 3 + 1, :],
                PEEm_LM2d[i * 3 + 2, :],
                marker="^",
                color="blue",
                label="Measured",
            )
            ax2.scatter3D(
                PEEe_sol2d[i * 3, :],
                PEEe_sol2d[i * 3 + 1, :],
                PEEe_sol2d[i * 3 + 2, :],
                marker="o",
                color="red",
                label="Estimated",
            )
            if INCLUDE_UNCALIB:
                ax2.scatter3D(
                    PEEe_uncalib2d[i * 3, :],
                    PEEe_uncalib2d[i * 3 + 1, :],
                    PEEe_uncalib2d[i * 3 + 2, :],
                    marker="x",
                    color="green",
                    label="Uncalibrated",
                )
            for j in range(self.calib_config["NbSample"]):
                ax2.plot3D(
                    [PEEm_LM2d[i * 3, j], PEEe_sol2d[i * 3, j]],
                    [PEEm_LM2d[i * 3 + 1, j], PEEe_sol2d[i * 3 + 1, j]],
                    [PEEm_LM2d[i * 3 + 2, j], PEEe_sol2d[i * 3 + 2, j]],
                    color="red",
                )
                if INCLUDE_UNCALIB:
                    ax2.plot3D(
                        [PEEm_LM2d[i * 3, j], PEEe_uncalib2d[i * 3, j]],
                        [
                            PEEm_LM2d[i * 3 + 1, j],
                            PEEe_uncalib2d[i * 3 + 1, j],
                        ],
                        [
                            PEEm_LM2d[i * 3 + 2, j],
                            PEEe_uncalib2d[i * 3 + 2, j],
                        ],
                        color="green",
                    )
        ax2.set_xlabel("X - front (meter)")
        ax2.set_ylabel("Y - side (meter)")
        ax2.set_zlabel("Z - height (meter)")
        ax2.grid()
        ax2.legend()

    def plot_joint_configurations(self):
        """Plot joint configurations within range bounds."""
        fig4 = plt.figure()
        fig4.suptitle("Joint configurations with joint bounds")
        ax4 = fig4.add_subplot(111, projection="3d")
        lb = ub = []
        for j in self.calib_config["config_idx"]:
            lb = np.append(lb, self.model.lowerPositionLimit[j])
            ub = np.append(ub, self.model.upperPositionLimit[j])
        q_actJoint = self.q_measured[:, self.calib_config["config_idx"]]
        sample_range = np.arange(self.calib_config["NbSample"])
        for i in range(len(self.calib_config["actJoint_idx"])):
            ax4.scatter3D(q_actJoint[:, i], sample_range, i)
        for i in range(len(self.calib_config["actJoint_idx"])):
            ax4.plot([lb[i], ub[i]], [sample_range[0], sample_range[0]], [i, i])
            ax4.set_xlabel("Angle (rad)")
            ax4.set_ylabel("Sample")
            ax4.set_zlabel("Joint")
            ax4.grid()

    def save_results(self, output_dir="results"):
        """Save calibration results using unified results manager."""
        if not hasattr(self, 'result') or self.results_data is None:
            print("No calibration results to save. Run solve() first.")
            return
        
        # Use pre-initialized results manager if available
        if hasattr(self, 'results_manager') and \
           self.results_manager is not None:
            try:
                # Save using unified manager with self.result data
                saved_files = self.results_manager.save_results(
                    output_dir=output_dir,
                    save_formats=['yaml', 'csv', 'npz']
                )

                print("Calibration results saved using ResultsManager")
                for fmt, path in saved_files.items():
                    print(f"  {fmt}: {path}")
                
                return saved_files
                
            except Exception as e:
                print(f"Error saving with ResultsManager: {e}")
                print("Falling back to basic saving...")