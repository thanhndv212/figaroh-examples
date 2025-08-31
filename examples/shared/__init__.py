"""
Shared utilities and base classes for FIGAROH robot examples.

This module provides common functionality that can be reused across
different robot implementations in the examples directory.

Available modules:
- base_identification: Base class for robot dynamic parameter identification
- base_optimal_calibration: Base class for optimal configuration generation
- base_optimal_trajectory: Base class for optimal trajectory generation
"""

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

__all__ = [
    'BaseIdentification',
    'BaseOptimalCalibration',
    'SOCPOptimizer',
    'Detmax',
    'BaseOptimalTrajectory',
    'BaseTrajectoryIPOPTProblem',
    'ConfigurationManager',
    'BaseParameterComputer',
    'TrajectoryConstraintManager'
]
