#!/bin/bash

# Script to create a new robot example folder based on the Tiago template
# Usage: ./create_example.sh <robot_name>
# Example: ./create_example.sh ur5

set -e  # Exit on error

# Clean up temporary files on exit or interrupt
trap 'rm -f "$NEW_DIR"/*.tmp "$NEW_DIR"/*/*.tmp 2>/dev/null' EXIT INT TERM

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if robot name is provided
if [ -z "$1" ]; then
    print_error "Robot name is required!"
    echo "Usage: $0 <robot_name>"
    echo "Example: $0 ur5"
    exit 1
fi

ROBOT_NAME=$1
ROBOT_NAME_LOWER=$(echo "$ROBOT_NAME" | tr '[:upper:]' '[:lower:]')
ROBOT_NAME_UPPER=$(echo "$ROBOT_NAME" | tr '[:lower:]' '[:upper:]')
# Determine title case: preserve all-caps acronyms (e.g., UR10, TX40), else capitalize first letter
if [[ "$ROBOT_NAME" =~ ^[A-Z0-9_]+$ ]]; then
    ROBOT_NAME_TITLE="$ROBOT_NAME"
else
    ROBOT_NAME_TITLE=$(echo "$ROBOT_NAME" | awk '{print toupper(substr($0,1,1)) tolower(substr($0,2))}')
fi

# Get the script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
EXAMPLES_DIR="$SCRIPT_DIR"
TEMPLATE_DIR="$EXAMPLES_DIR/tiago"
# The complete example that generated placeholders point readers at.
REFERENCE_EXAMPLE="ur10"
NEW_DIR="$EXAMPLES_DIR/$ROBOT_NAME_LOWER"

print_info "Creating new example for robot: $ROBOT_NAME"
print_info "Template directory: $TEMPLATE_DIR"
print_info "Target directory: $NEW_DIR"

# Check if template directory exists
if [ ! -d "$TEMPLATE_DIR" ]; then
    print_error "Template directory not found: $TEMPLATE_DIR"
    exit 1
fi

# Check if target directory already exists
if [ -d "$NEW_DIR" ]; then
    print_warning "Directory $NEW_DIR already exists!"
    read -p "Do you want to overwrite it? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        print_info "Aborting..."
        exit 0
    fi
    rm -rf "$NEW_DIR"
fi

# Create main directory structure
print_info "Creating directory structure..."
mkdir -p "$NEW_DIR"
mkdir -p "$NEW_DIR/config"
mkdir -p "$NEW_DIR/config/templates"
mkdir -p "$NEW_DIR/data"
mkdir -p "$NEW_DIR/data/calibration"
mkdir -p "$NEW_DIR/data/calibration/mocap"
mkdir -p "$NEW_DIR/data/calibration/optimal_configurations"
mkdir -p "$NEW_DIR/data/identification"
mkdir -p "$NEW_DIR/data/identification/dynamic"
mkdir -p "$NEW_DIR/data/optimal_configurations"
mkdir -p "$NEW_DIR/docs"
mkdir -p "$NEW_DIR/tmp"
mkdir -p "$NEW_DIR/urdf"
mkdir -p "$NEW_DIR/utils"

print_success "Directory structure created!"

# Function to replace robot name in content
# Replaces longer placeholder strings first to prevent substring interference
replace_robot_name() {
    local content="$1"
    # Ordered by descending placeholder length: longer patterns first
    # REF_EXAMPLE must resolve BEFORE the tiago rules below: it marks pointers
    # to a real, complete example that must survive substitution. Without it
    # the tiago->robot rewrite turns "refer to examples/REF_EXAMPLE/x.py" into a
    # self-reference to the placeholder being generated.
    content="${content//REF_EXAMPLE/$REFERENCE_EXAMPLE}"
    # Generic placeholders used in templates
    content="${content//ROBOT_TITLE/$ROBOT_NAME_TITLE}"
    content="${content//robot_lower/$ROBOT_NAME_LOWER}"
    # TIAGo-specific pattern replacements (for template compatibility)
    content="${content//TIAGo/$ROBOT_NAME_TITLE}"
    content="${content//TIAGO/$ROBOT_NAME_UPPER}"
    content="${content//Tiago/$ROBOT_NAME_TITLE}"
    content="${content//tiago/$ROBOT_NAME_LOWER}"
    echo "$content"
}

# Create Python files with templates
print_info "Creating Python example files..."

# 1. calibration.py
cat > "$NEW_DIR/calibration.py" << 'EOF'
"""
Kinematic Calibration for ROBOT_TITLE Robot

This script performs kinematic calibration to correct geometric errors
in the robot structure using external measurements.
"""

import sys
from pathlib import Path

# Add parent directory to path
parent_dir = str(Path(__file__).resolve().parent.parent.parent)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from figaroh.calibration.base_calibration import BaseCalibration


def main():
    """
    Main calibration workflow for ROBOT_TITLE robot.
    """
    print("Starting ROBOT_TITLE kinematic calibration...")

    # TODO: Implement calibration workflow
    # 1. Load robot model
    # 2. Load measurement data
    # 3. Define calibration parameters
    # 4. Run calibration
    # 5. Save results

    print("Calibration workflow not yet implemented!")
    print("Please refer to examples/REF_EXAMPLE/calibration.py for reference.")


if __name__ == "__main__":
    main()
EOF

replace_robot_name "$(cat "$NEW_DIR/calibration.py")" > "$NEW_DIR/calibration.py.tmp"
mv "$NEW_DIR/calibration.py.tmp" "$NEW_DIR/calibration.py"

# 2. identification.py
cat > "$NEW_DIR/identification.py" << 'EOF'
"""
Dynamic Parameter Identification for ROBOT_TITLE Robot

This script identifies dynamic parameters (masses, inertias, friction)
using motion data and force/torque measurements.
"""

import sys
from pathlib import Path

# Add parent directory to path
parent_dir = str(Path(__file__).resolve().parent.parent.parent)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from figaroh.identification.base_identification import BaseIdentification


def main():
    """
    Main identification workflow for ROBOT_TITLE robot.
    """
    print("Starting ROBOT_TITLE dynamic identification...")

    # TODO: Implement identification workflow
    # 1. Load robot model
    # 2. Load trajectory data (positions, velocities, accelerations, torques)
    # 3. Build regressor matrix
    # 4. Solve least squares problem
    # 5. Validate and save parameters

    print("Identification workflow not yet implemented!")
    print("Please refer to examples/REF_EXAMPLE/identification.py for reference.")


if __name__ == "__main__":
    main()
EOF

replace_robot_name "$(cat "$NEW_DIR/identification.py")" > "$NEW_DIR/identification.py.tmp"
mv "$NEW_DIR/identification.py.tmp" "$NEW_DIR/identification.py"

# 3. optimal_config.py
cat > "$NEW_DIR/optimal_config.py" << 'EOF'
"""
Optimal Configuration Generation for ROBOT_TITLE Robot

This script generates optimal robot configurations that maximize
observability of calibration parameters.
"""

import sys
from pathlib import Path

# Add parent directory to path
parent_dir = str(Path(__file__).resolve().parent.parent.parent)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from figaroh.optimal.base_optimal import BaseOptimalConfiguration


def main():
    """
    Generate optimal configurations for ROBOT_TITLE calibration.
    """
    print("Generating optimal configurations for ROBOT_TITLE...")

    # TODO: Implement optimal configuration generation
    # 1. Load robot model
    # 2. Define workspace constraints
    # 3. Define optimization objectives
    # 4. Run optimization
    # 5. Save optimal configurations

    print("Optimal configuration generation not yet implemented!")
    print("Please refer to examples/REF_EXAMPLE/optimal_config.py for reference.")


if __name__ == "__main__":
    main()
EOF

replace_robot_name "$(cat "$NEW_DIR/optimal_config.py")" > "$NEW_DIR/optimal_config.py.tmp"
mv "$NEW_DIR/optimal_config.py.tmp" "$NEW_DIR/optimal_config.py"

# 4. optimal_trajectory.py
cat > "$NEW_DIR/optimal_trajectory.py" << 'EOF'
"""
Optimal Trajectory Generation for ROBOT_TITLE Robot

This script generates optimal exciting trajectories for dynamic
parameter identification.
"""

import sys
from pathlib import Path

# Add parent directory to path
parent_dir = str(Path(__file__).resolve().parent.parent.parent)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from figaroh.tools.robot import load_robot


def main():
    """
    Generate optimal trajectory for ROBOT_TITLE identification.
    """
    print("Generating optimal trajectory for ROBOT_TITLE...")

    # TODO: Implement optimal trajectory generation
    # 1. Load robot model
    # 2. Define active joints
    # 3. Define trajectory constraints
    # 4. Run trajectory optimization
    # 5. Save trajectory

    print("Optimal trajectory generation not yet implemented!")
    print("Please refer to examples/REF_EXAMPLE/optimal_trajectory.py for reference.")


if __name__ == "__main__":
    main()
EOF

replace_robot_name "$(cat "$NEW_DIR/optimal_trajectory.py")" > "$NEW_DIR/optimal_trajectory.py.tmp"
mv "$NEW_DIR/optimal_trajectory.py.tmp" "$NEW_DIR/optimal_trajectory.py"

print_success "Python files created!"

# Create utils module
print_info "Creating utils module..."

cat > "$NEW_DIR/utils/__init__.py" << 'EOF'
"""
Utility functions and classes for ROBOT_TITLE examples.
"""

__all__ = ['robot_lower_tools', 'simplified_collision_model']
EOF

replace_robot_name "$(cat "$NEW_DIR/utils/__init__.py")" > "$NEW_DIR/utils/__init__.py.tmp"
mv "$NEW_DIR/utils/__init__.py.tmp" "$NEW_DIR/utils/__init__.py"

cat > "$NEW_DIR/utils/${ROBOT_NAME_LOWER}_tools.py" << 'EOF'
"""
Custom tools and utilities specific to ROBOT_TITLE robot.

This module contains helper functions, custom classes, and utilities
that are specific to working with the ROBOT_TITLE robot.
"""

import numpy as np
from figaroh.tools.robot import Robot


class OptimalTrajectoryIPOPT:
    """
    Optimal trajectory generation using IPOPT solver.

    This class handles trajectory optimization for the ROBOT_TITLE robot
    to generate information-rich motions for parameter identification.
    """

    def __init__(self, robot: Robot, active_joints: list, config_file: str):
        """
        Initialize the trajectory optimizer.

        Args:
            robot: Robot model
            active_joints: List of joint names to include in optimization
            config_file: Path to configuration YAML file
        """
        self.robot = robot
        self.active_joints = active_joints
        self.config_file = config_file

        print(f"Initialized trajectory optimizer for ROBOT_TITLE")
        print(f"Active joints: {active_joints}")

    def solve(self, stack_reps: int = 2):
        """
        Solve the trajectory optimization problem.

        Args:
            stack_reps: Number of trajectory repetitions to stack

        Returns:
            dict: Results containing trajectory segments and metrics
        """
        print("Trajectory optimization not yet implemented!")
        print("Please refer to examples/REF_EXAMPLE/utils/REF_EXAMPLE_tools.py for reference.")
        return {'T_F': None}

    def plot_results(self):
        """Plot the optimization results."""
        print("Plotting not yet implemented!")


def load_robot_model(robot_name: str = "robot_lower", **kwargs):
    """
    Load the ROBOT_TITLE robot model.

    Args:
        robot_name: Name/identifier for the robot
        **kwargs: Additional arguments for robot loading

    Returns:
        Robot: Loaded robot model
    """
    from figaroh.tools.robot import load_robot

    # TODO: Update with correct parameters for your robot
    robot = load_robot(
        robot_name=robot_name,
        load_by_urdf=True,
        robot_pkg="robot_lower_description",
        **kwargs
    )

    return robot
EOF

replace_robot_name "$(cat "$NEW_DIR/utils/${ROBOT_NAME_LOWER}_tools.py")" > "$NEW_DIR/utils/${ROBOT_NAME_LOWER}_tools.py.tmp"
mv "$NEW_DIR/utils/${ROBOT_NAME_LOWER}_tools.py.tmp" "$NEW_DIR/utils/${ROBOT_NAME_LOWER}_tools.py"

cat > "$NEW_DIR/utils/simplified_collision_model.py" << 'EOF'
"""
Simplified collision model for ROBOT_TITLE robot.

This module provides functions to build a simplified collision model
for the robot, which is useful for trajectory optimization with
collision avoidance constraints.
"""

from figaroh.tools.robot import Robot


def build_robot_lower_simplified(robot: Robot):
    """
    Build a simplified collision model for the ROBOT_TITLE robot.

    Args:
        robot: Original robot model

    Returns:
        Robot: Robot with simplified collision geometry
    """
    print("Building simplified collision model for ROBOT_TITLE...")

    # TODO: Implement simplified collision model
    # This typically involves:
    # 1. Identifying critical links that need collision checking
    # 2. Simplifying complex geometries to basic shapes (spheres, capsules, boxes)
    # 3. Defining collision pairs to check

    print("Simplified collision model not yet implemented!")
    print("Returning original robot model.")

    return robot
EOF

replace_robot_name "$(cat "$NEW_DIR/utils/simplified_collision_model.py")" > "$NEW_DIR/utils/simplified_collision_model.py.tmp"
mv "$NEW_DIR/utils/simplified_collision_model.py.tmp" "$NEW_DIR/utils/simplified_collision_model.py"

print_success "Utils module created!"

# Create configuration files
print_info "Creating configuration files..."

cat > "$NEW_DIR/config/${ROBOT_NAME_LOWER}_unified_config.yaml" << 'EOF'
# Unified configuration for ROBOT_TITLE.
#
# This is the format figaroh actually parses. Every available key is documented
# in examples/templates/base_robot_config.yaml; examples/REF_EXAMPLE/config/
# REF_EXAMPLE_unified_config.yaml is a complete worked example.
#
# Swap the template below for humanoid_robot.yaml if the robot has a mobile or
# floating base.
extends: "../../templates/manipulator_robot.yaml"

robot:
  name: "robot_lower"
  description: "ROBOT_TITLE robot"

  properties:
    joints:
      # TODO: use the EXACT joint names from your URDF. Print them with:
      #   cd examples/robot_lower && python -c "\
      #     from figaroh.tools.robot import load_robot; \
      #     r = load_robot('urdf/robot_lower.urdf', \
      #                    package_dirs='../../models', load_by_urdf=True); \
      #     print([n for n in r.model.names])"
      active_joints:
        - "joint_1"
        - "joint_2"
        - "joint_3"
        - "joint_4"
        - "joint_5"
        - "joint_6"

      # One [min, max] pair per active joint, flattened.
      joint_limits:
        position: [-3.14, 3.14, -3.14, 3.14, -3.14, 3.14,
                   -3.14, 3.14, -3.14, 3.14, -3.14, 3.14]
        velocity: [2.0, 2.0, 2.0, 2.0, 2.0, 2.0]          # rad/s
        acceleration: [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]  # rad/s^2
        torque: [100.0, 100.0, 100.0, 100.0, 100.0, 100.0]  # Nm

    mechanics:
      reduction_ratios: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
      friction_coefficients:
        viscous: [0, 0, 0, 0, 0, 0]   # 0 = to be identified
        static: [0, 0, 0, 0, 0, 0]
      actuator_inertias: [0, 0, 0, 0, 0, 0]
      joint_offsets: [0, 0, 0, 0, 0, 0]  # 0 = to be calibrated

tasks:
  calibration:
    enabled: true

    parameters:
      calibration_level: "full_params"   # or "joint_offset"
      include_non_geometric: false
      outlier_threshold: 0.02            # metres

    kinematics:
      base_frame: "universe"
      tool_frame: "TODO_tool_link"       # must exist in the URDF

    measurements:
      # measurable_dof decides which columns the CSV must have, per marker:
      #   [true]*6 -> x1,y1,z1,phix1,phiy1,phiz1
      #   position only -> x1,y1,z1
      # Joint columns use the exact URDF joint names listed above.
      markers:
        - name: "tool_marker"
          reference_joint: "TODO_last_joint"
          measurable_dof: [true, true, true, false, false, false]
          sensor_type: "mocap"
      poses:
        base_pose: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        tool_pose: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    data:
      source_file: "data/calibration.csv"
      # Independent acquisition only -- never a split of the training file.
      # Empty falls back to the training data with a warning.
      validation_data_file: ""
      number_of_samples: 50

  identification:
    enabled: false

    problem:
      include_external_forces: false
      use_joint_torques: true
      wls: false
      model_components:
        friction: false
        joint_offset: false
        actuator_inertia: false
        static_regressor: true
        inertia_regressor: true

    signal_processing:
      sampling_frequency: 100.0   # Hz -- must match the real log rate
      cutoff_frequency: 10.0      # Hz
      filter_type: "butterworth"
      filter_order: 4

    data:
      validation_data_file: ""

  optimal_configuration:
    enabled: false

    constraints:
      joint_limit_margin: 0.1
      collision_checking: true

    output:
      save_configurations: true
      # Only the DIRECTORY part is used; filenames are timestamped.
      output_file: "data/optimal_configurations/robot_lower_optimal_configs.yaml"

  optimal_trajectory:
    enabled: false

    problem:
      soft_lim: 0.1
      max_attempts: 500

    trajectory:
      waypoints: 5
      frequency: 100              # Hz -- keep consistent with
                                  # identification.signal_processing
      segment_duration: 2.0       # seconds

    constraints:
      velocity_scaling: 0.5
      acceleration_scaling: 0.5

    output:
      save_trajectory: true
      output_file: "data/trajectories/robot_lower_optimal_trajectory.yaml"

environment:
  working_directory: "."
  data_directory: "data"
  results_directory: "results"
EOF

replace_robot_name "$(cat "$NEW_DIR/config/${ROBOT_NAME_LOWER}_unified_config.yaml")" > "$NEW_DIR/config/${ROBOT_NAME_LOWER}_unified_config.yaml.tmp"
mv "$NEW_DIR/config/${ROBOT_NAME_LOWER}_unified_config.yaml.tmp" "$NEW_DIR/config/${ROBOT_NAME_LOWER}_unified_config.yaml"

print_success "Configuration files created!"

# Create README.md
print_info "Creating README.md..."

cat > "$NEW_DIR/README.md" << 'EOF'
# ROBOT_TITLE Robot Calibration and Identification Framework

This directory provides a comprehensive framework for calibrating and identifying the kinematic and dynamic parameters of the ROBOT_TITLE robot using the FIGAROH library.

## Overview

The ROBOT_TITLE robot requires accurate modeling for precision tasks. This framework provides four interconnected tools:

1. **Kinematic Calibration** - Correct geometric errors in robot structure
2. **Dynamic Parameter Identification** - Identify physical properties for accurate dynamics
3. **Optimal Configuration Generation** - Scientifically select measurement poses
4. **Optimal Trajectory Generation** - Create information-rich motion for identification

## Directory Structure

```
robot_lower/
├── calibration.py              # Kinematic calibration implementation
├── identification.py           # Dynamic parameter identification
├── optimal_config.py           # Optimal configuration generation
├── optimal_trajectory.py       # Optimal trajectory generation
├── README.md                   # This file
├── config/                     # Configuration files
│   ├── robot_lower_unified_config.yaml
│   └── templates/
├── data/                       # Data storage
│   ├── calibration/
│   ├── identification/
│   └── optimal_configurations/
├── docs/                       # Documentation
├── urdf/                       # Robot URDF files
└── utils/                      # Utility functions
    ├── __init__.py
    ├── robot_lower_tools.py
    └── simplified_collision_model.py
```

## Quick Start

### Prerequisites

```bash
# Install FIGAROH and dependencies
pip install figaroh numpy scipy matplotlib pandas

# Install Pinocchio and CyIpopt (recommended via conda)
conda install -c conda-forge pinocchio cyipopt
```

### Basic Usage

1. **Kinematic Calibration**:
```bash
python calibration.py
```

2. **Dynamic Identification**:
```bash
python identification.py
```

3. **Generate Optimal Configurations**:
```bash
python optimal_config.py
```

4. **Generate Optimal Trajectories**:
```bash
python optimal_trajectory.py
```

## Implementation Steps

This is a template directory. To complete the implementation for your ROBOT_TITLE robot:

### 1. Robot Model Setup
- [ ] Add URDF file to `urdf/` directory
- [ ] Update robot package name in config files
- [ ] Define active joints in `config/robot_lower_unified_config.yaml`
- [ ] Set joint limits and constraints

### 2. Calibration Implementation
- [ ] Collect measurement data (camera, laser tracker, etc.)
- [ ] Define calibration parameters in config
- [ ] Implement calibration workflow in `calibration.py`
- [ ] Validate calibration results

### 3. Identification Implementation
- [ ] Record robot motion data (positions, velocities, torques)
- [ ] Prepare data files in `data/identification/`
- [ ] Implement identification workflow in `identification.py`
- [ ] Validate identified parameters

### 4. Optimal Configuration
- [ ] Define workspace constraints
- [ ] Implement observability optimization in `optimal_config.py`
- [ ] Generate and save optimal configurations

### 5. Optimal Trajectory
- [ ] Build simplified collision model (if needed)
- [ ] Implement trajectory optimization in `optimal_trajectory.py`
- [ ] Validate trajectory on real robot

## Configuration

Edit `config/robot_lower_unified_config.yaml` to customize:
- Robot model parameters
- Active joints and limits
- Calibration parameters
- Identification settings
- Trajectory constraints
- Data paths

## Utility Functions

The `utils/` directory contains robot-specific helpers:
- `robot_lower_tools.py`: Custom tools and optimization classes
- `simplified_collision_model.py`: Collision model for trajectory planning

## Data Format

### Calibration Data
Store measurement data in `data/calibration/`:
- CSV format with columns: [timestamp, x, y, z, ...]
- Include joint angles and measured positions

### Identification Data
Store trajectory data in `data/identification/`:
- CSV format with columns: [timestamp, q1, q2, ..., dq1, dq2, ..., tau1, tau2, ...]
- Positions, velocities, and torques for all joints

## Reference

This template is based on the TIAGo example. For detailed implementation examples, see:
- `examples/REF_EXAMPLE/` - Complete reference implementation
- FIGAROH documentation: [link to docs]

## Troubleshooting

### Common Issues

1. **Robot model not loading**
   - Check URDF file path in config
   - Verify robot package is installed
   - Check for mesh file dependencies

2. **Optimization fails**
   - Verify joint limits are correct
   - Check initial configuration is valid
   - Reduce trajectory complexity

3. **Import errors**
   - Ensure FIGAROH is installed
   - Check Python path includes parent directory
   - Verify all dependencies are installed

## Contributing

To improve this example:
1. Implement the TODO items in each script
2. Add your specific robot configuration
3. Document your workflow and results
4. Share successful calibration/identification results

## License

[Add your license information here]

## Contact

[Add contact information or links to documentation]
EOF

replace_robot_name "$(cat "$NEW_DIR/README.md")" > "$NEW_DIR/README.md.tmp"
mv "$NEW_DIR/README.md.tmp" "$NEW_DIR/README.md"

print_success "README.md created!"

# Create placeholder data files
print_info "Creating placeholder data files..."

cat > "$NEW_DIR/data/calibration/README.md" << 'EOF'
# Calibration Data

Place your calibration measurement data in this directory.

## Expected Format

CSV files with columns:
- timestamp
- joint angles (q1, q2, q3, ...)
- measured positions (x, y, z)
- measured orientations (optional)

Example:
```
timestamp,q1,q2,q3,q4,q5,q6,x,y,z
0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.5,0.0,0.8
0.1,0.1,0.2,0.0,0.0,0.0,0.0,0.52,0.05,0.82
...
```
EOF

cat > "$NEW_DIR/data/identification/README.md" << 'EOF'
# Identification Data

Place your dynamic identification data in this directory.

## Expected Format

CSV files with columns:
- timestamp
- positions (q1, q2, q3, ...)
- velocities (dq1, dq2, dq3, ...)
- accelerations (ddq1, ddq2, ddq3, ...) [optional, can be computed]
- torques (tau1, tau2, tau3, ...)

Example:
```
timestamp,q1,q2,q3,dq1,dq2,dq3,tau1,tau2,tau3
0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0
0.01,0.001,0.002,0.001,0.1,0.2,0.1,1.5,2.3,0.8
...
```
EOF

print_success "Placeholder data files created!"

# Create .gitkeep files for empty directories
touch "$NEW_DIR/docs/.gitkeep"
touch "$NEW_DIR/tmp/.gitkeep"
touch "$NEW_DIR/urdf/.gitkeep"

# Create a summary file
print_info "Creating setup summary..."

cat > "$NEW_DIR/SETUP_GUIDE.md" << 'EOF'
# Setup Guide for ROBOT_TITLE Example

This guide will help you complete the setup and implementation of the ROBOT_TITLE example.

## ✅ What's Been Created

The following directory structure and files have been automatically generated:

### 📁 Directory Structure
- `config/` - Configuration files
- `data/` - Data storage (calibration, identification, results)
- `docs/` - Documentation
- `urdf/` - Robot URDF models
- `utils/` - Utility functions and tools
- `tmp/` - Temporary files

### 📄 Python Files
- `calibration.py` - Kinematic calibration script
- `identification.py` - Dynamic identification script
- `optimal_config.py` - Optimal configuration generation
- `optimal_trajectory.py` - Optimal trajectory generation

### ⚙️ Configuration
- `config/robot_lower_unified_config.yaml` - Main configuration file

### 📚 Documentation
- `README.md` - Main documentation
- `data/calibration/README.md` - Calibration data format guide
- `data/identification/README.md` - Identification data format guide

## 📝 Next Steps

Follow these steps to complete your implementation:

### Step 1: Robot Model Setup
1. Obtain the URDF file for your ROBOT_TITLE robot
2. Place it in the `urdf/` directory
3. Update `config/robot_lower_unified_config.yaml`:
   - Set `model.urdf_path` to your URDF file
   - Update `model.package_name` if needed
   - List all active joints in `joints.active_joints`
   - Define joint limits in `joints.limits`

### Step 2: Test Robot Loading
Create a simple test script to verify your robot loads correctly:

```python
from figaroh.tools.robot import load_robot

robot = load_robot(
    robot_name="robot_lower",
    load_by_urdf=True,
    robot_pkg="robot_lower_description"
)
print(f"Loaded robot with {robot.nq} DOF")
```

### Step 3: Implement Utils
1. Edit `utils/robot_lower_tools.py`:
   - Update `load_robot_model()` with correct parameters
   - Implement `OptimalTrajectoryIPOPT` if needed

2. Edit `utils/simplified_collision_model.py`:
   - Implement simplified collision geometry if doing trajectory optimization

### Step 4: Collect Data
1. For calibration:
   - Collect end-effector position measurements
   - Save to `data/calibration/` in CSV format

2. For identification:
   - Record robot trajectories with joint positions, velocities, and torques
   - Save to `data/identification/` in CSV format

### Step 5: Implement Workflows
1. Complete `calibration.py`:
   - Load your robot model
   - Load measurement data
   - Configure calibration parameters
   - Run calibration and save results

2. Complete `identification.py`:
   - Load trajectory data
   - Build regressor matrix
   - Solve for dynamic parameters
   - Validate and save results

3. Complete `optimal_config.py`:
   - Define workspace constraints
   - Run optimization for observable configurations

4. Complete `optimal_trajectory.py`:
   - Define trajectory constraints
   - Run trajectory optimization

### Step 6: Test and Validate
1. Run each script individually
2. Verify outputs in `data/` directories
3. Validate results against ground truth or expected values

## 🔍 Reference Implementation

For detailed examples, refer to the REF_EXAMPLE implementation:
- `examples/REF_EXAMPLE/calibration.py`
- `examples/REF_EXAMPLE/identification.py`
- `examples/REF_EXAMPLE/optimal_config.py`
- `examples/REF_EXAMPLE/optimal_trajectory.py`

## 📚 Resources

- FIGAROH documentation: [link]
- Pinocchio documentation: https://stack-of-tasks.github.io/pinocchio/
- Example data formats: See `data/*/README.md` files

## ❓ Common Questions

**Q: Where do I get the URDF file?**
A: Check your robot manufacturer's website, ROS packages, or create one from CAD models.

**Q: What if my robot has a mobile base?**
A: Include the mobile base joints in your URDF and active joints list.

**Q: How do I collect calibration data?**
A: Use external measurement systems (motion capture, laser tracker, etc.) or manual measurements.

**Q: What trajectory should I use for identification?**
A: Use the optimal trajectory generator or exciting trajectories with varying frequencies.

## 💡 Tips

1. Start simple - test each component individually
2. Use visualization to verify robot model and trajectories
3. Check data quality before running optimization
4. Validate results incrementally
5. Document your specific setup and parameters

## 🐛 Troubleshooting

If you encounter issues:
1. Check that all dependencies are installed
2. Verify URDF file loads correctly
3. Ensure data files are in correct format
4. Review configuration file parameters
5. Refer to TIAGo example for working implementation

## ✨ Contributing Back

If you successfully implement this for your robot:
1. Consider sharing your configuration
2. Document robot-specific tips
3. Contribute back to the examples repository

Good luck with your ROBOT_TITLE calibration and identification! 🚀
EOF

replace_robot_name "$(cat "$NEW_DIR/SETUP_GUIDE.md")" > "$NEW_DIR/SETUP_GUIDE.md.tmp"
mv "$NEW_DIR/SETUP_GUIDE.md.tmp" "$NEW_DIR/SETUP_GUIDE.md"

print_success "Setup guide created!"

# Print summary
echo ""
echo "=========================================="
print_success "Successfully created ROBOT_TITLE example!"
echo "=========================================="
echo ""
print_info "Location: $NEW_DIR"
echo ""
print_info "What's been created:"
echo "  ✓ Directory structure (config/, data/, urdf/, utils/, docs/)"
echo "  ✓ Python example files (calibration.py, identification.py, etc.)"
echo "  ✓ Configuration files (robot_lower_unified_config.yaml)"
echo "  ✓ Utility modules (robot_lower_tools.py, simplified_collision_model.py)"
echo "  ✓ Documentation (README.md, SETUP_GUIDE.md)"
echo "  ✓ Data format guides"
echo ""
print_info "Next steps:"
echo "  1. Read SETUP_GUIDE.md for detailed implementation steps"
echo "  2. Add your robot's URDF file to urdf/ directory"
echo "  3. Update config/robot_lower_unified_config.yaml with your robot's parameters"
echo "  4. Implement the TODO items in each Python file"
echo "  5. Refer to examples/tiago/ for reference implementation"
echo ""
print_info "Quick start:"
echo "  cd $NEW_DIR"
echo "  cat SETUP_GUIDE.md"
echo ""
print_success "Happy coding! 🚀"
echo ""
