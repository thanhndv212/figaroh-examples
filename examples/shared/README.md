# Shared Robot Tools

This directory contains base classes and utilities that can be shared across different robot implementations in the FIGAROH examples.

## Overview

The shared tools provide a common framework for:
- **System Identification**: Dynamic parameter identification workflows
- **Optimal Calibration**: Configuration generation for kinematic calibration
- **Optimal Trajectory**: Trajectory optimization for parameter identification

## Base Classes

### BaseIdentification
Base class for robot dynamic parameter identification with support for:
- Generic data loading and processing
- Regressor matrix computation
- QR decomposition and base parameter calculation
- Quality metrics and validation
- Format-agnostic parameter handling

### BaseOptimalCalibration
Base class for optimal configuration generation with:
- D-optimal experimental design
- SOCP optimization for configuration selection
- Support for multiple calibration models
- Comprehensive visualization tools

### BaseOptimalTrajectory
Base class for optimal trajectory generation featuring:
- IPOPT-based trajectory optimization
- Modular constraint management
- Multi-segment trajectory stacking
- Configuration validation and feasibility checking

## Usage

To use these base classes in your robot implementation:

```python
# Import from shared directory
from examples.shared import BaseIdentification, BaseOptimalCalibration

# Create robot-specific classes
class MyRobotIdentification(BaseIdentification):
    def process_torque_data(self, tau):
        # Robot-specific torque processing
        return processed_tau

class MyRobotCalibration(BaseOptimalCalibration):
    def calculate_regressor(self):
        # Robot-specific regressor calculation
        pass
```

## System Identification Perspective

These base classes are designed with system identification workflows in mind:

1. **Format Flexibility**: Support for URDF, SDF, MJCF parameter formats
2. **Parameter Reconstruction**: Methods for converting between formats
3. **Validation Workflows**: Cross-format validation capabilities
4. **Tool Integration**: Compatible with FIGAROH identification tools

## Examples

See the individual robot directories for specific implementations:
- `examples/tiago/`: TIAGo robot implementation
- `examples/ur10/`: UR10 robot implementation
- `examples/mate/`: MATE robot implementation

## Dependencies

- FIGAROH (core robotics tools)
- NumPy (numerical computation)
- SciPy (scientific computing)
- Matplotlib (visualization)
- YAML (configuration files)
- Pandas (data processing)
