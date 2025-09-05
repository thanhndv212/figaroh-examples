"""
FIGAROH Examples Package

This package contains robot identification and calibration examples
using the FIGAROH framework, with shared utilities and base classes
for different robot implementations.

Available modules:
- shared: Common base classes and utilities
- robots: Robot-specific implementations (TX40, UR10, TALOS, TIAGO, MATE)
"""

# Import base classes from FIGAROH library for easy access
try:
    from figaroh.identification.base_identification import BaseIdentification
    from figaroh.optimal.base_optimal_calibration import BaseOptimalCalibration
    from figaroh.optimal.base_optimal_trajectory import BaseOptimalTrajectory
    from figaroh.utils.results_manager import ResultsManager
    from figaroh.utils.error_handling import (
        CalibrationError, 
        IdentificationError,
        handle_calibration_errors
    )
    
    # Import shared modules that are still only in examples
    from .shared import (
        ConfigurationManager,
        BaseParameterComputer,
        TrajectoryConstraintManager,
        ConfigManager,
        DataProcessor
    )
except ImportError:
    # Graceful fallback if modules are not available
    pass

__version__ = "0.1.0"
__author__ = "Thanh Nguyen"

__all__ = [
    'BaseIdentification',
    'BaseOptimalCalibration',
    'BaseOptimalTrajectory',
    'SOCPOptimizer',
    'Detmax',
    'ConfigurationManager',
    'BaseParameterComputer',
    'TrajectoryConstraintManager'
]
