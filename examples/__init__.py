"""
FIGAROH Examples Package

This package contains robot identification and calibration examples
using the FIGAROH framework, with shared utilities and base classes
for different robot implementations.

Available modules:
- shared: Common base classes and utilities
- robots: Robot-specific implementations (TX40, UR10, TALOS, TIAGO, MATE)
"""

# Import shared modules for easy access
try:
    from .shared import (
        BaseIdentification,
        BaseOptimalCalibration,
        BaseOptimalTrajectory,
        SOCPOptimizer,
        Detmax,
        ConfigurationManager,
        BaseParameterComputer,
        TrajectoryConstraintManager
    )
except ImportError:
    # Graceful fallback if shared modules are not available
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
