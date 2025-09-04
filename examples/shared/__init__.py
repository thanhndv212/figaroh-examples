"""
Shared utilities and base classes for FIGAROH robot examples.

This module provides common functionality that can be reused across
different robot implementations in the examples directory.

Available modules:
- base_identification: Base class for robot dynamic parameter identification
- base_optimal_calibration: Base class for optimal configuration generation
- base_optimal_trajectory: Base class for optimal trajectory generation
- config_manager: Centralized configuration management
- error_handling: Custom exceptions and validation utilities
- data_processing: Common data processing utilities
"""

# flake8: noqa F401

# Import base classes
from .base_calibration import BaseCalibration
from .base_identification import BaseIdentification
from .base_optimal_calibration import (
    BaseOptimalCalibration, SOCPOptimizer, Detmax
)
from .base_optimal_trajectory import (
    BaseOptimalTrajectory,
    BaseTrajectoryIPOPTProblem,
    ConfigurationManager,
    BaseParameterComputer,
    TrajectoryConstraintManager
)

from .config_manager import ConfigManager, ConfigurationError
from .error_handling import (
    FigarohExampleError,
    RobotInitializationError,
    DataProcessingError,
    CalibrationError,
    IdentificationError,
    ValidationError,
    validate_robot_config,
    validate_input_data,
    handle_identification_errors,
    handle_calibration_errors,
    setup_example_logging
)
from .data_processing import DataProcessor
from .results_manager import (
    ResultsManager,
    plot_calibration_results,
    plot_identification_results,
    save_results
)

# Include new modules in __all__
_new_modules = [
    'ConfigManager',
    'ConfigurationError',
    'FigarohExampleError',
    'RobotInitializationError',
    'DataProcessingError',
    'CalibrationError',
    'IdentificationError',
    'ValidationError',
    'validate_robot_config',
    'validate_input_data',
    'handle_identification_errors',
    'handle_calibration_errors',
    'setup_example_logging',
    'DataProcessor',
    'ResultsManager',
    'plot_calibration_results',
    'plot_identification_results',
    'save_results'
]


__version__ = "0.2.0"
__author__ = "Thanh Nguyen"

__all__ = [
    # Base classes
    'BaseIdentification',
    'BaseOptimalCalibration',
    'BaseOptimalTrajectory',
    'SOCPOptimizer',
    'Detmax',
    'BaseTrajectoryIPOPTProblem',
    'ConfigurationManager',
    'BaseParameterComputer',
    'TrajectoryConstraintManager',
] + _new_modules
