# FIGAROH Examples - Modernized Infrastructure

This repository contains comprehensive examples and tutorials for the [FIGAROH PLUS](https://github.com/thanhndv212/figaroh-plus) library - a Python toolbox for dynamics identification and geometric calibration of robots.

## 🎯 What's New

The FIGAROH examples have been completely modernized with professional-grade infrastructure:

- **🔧 Centralized Configuration Management** with validation schemas
- **🛡️ Robust Error Handling** with custom exception hierarchy
- **⚡ Performance Optimizations** with caching and vectorization
- **🧪 Comprehensive Testing** with pytest framework
- **📦 Professional Package Structure** with proper imports
- **🚀 Parallel Processing** for improved performance
- **📚 Extensive Documentation** and usage examples

## Prerequisites

First, install the core FIGAROH package:

```bash
pip install figaroh
# or for development:
pip install git+https://github.com/thanhndv212/figaroh-plus.git
```

Then install additional dependencies for the examples:

```bash
pip install -r requirements.txt
```

For conda users, some dependencies should be installed via conda:

```bash
conda install -c conda-forge pinocchio cyipopt
```

## 🚀 Quick Start

### Modern Infrastructure Usage

```python
# Import the modernized infrastructure
from ...shared import ConfigManager, DataProcessor, validate_robot_config

# Load and validate robot configuration
config = ConfigManager.load_robot_config('ur10', 'config.yaml')

# Process trajectory data with optimizations
processor = DataProcessor()
positions = processor.load_csv_data('trajectory.csv')
velocities, accelerations = processor.vectorized_differentiation(positions)

# Apply filtering with caching
filtered_data = processor.apply_filter(
    positions, 'lowpass', cutoff_freq=10.0, sampling_freq=100.0
)
```

### Robot-Specific Examples

```python
# Use updated robot tools with new infrastructure
from examples.ur10.utils.ur10_tools import UR10Identification
from examples.tiago.utils.tiago_tools import TiagoCalibration

# Initialize with automatic error handling and validation
ur10_id = UR10Identification(robot, 'config/ur10_config.yaml')
tiago_cal = TiagoCalibration(robot, 'config/tiago_config.yaml')
```

## Repository Structure

```
figaroh-examples/
├── examples/                    # Example implementations for different robots
│   ├── mate/                   # 3DOF manipulator examples  
│   ├── shared/                 # Shared base classes and utilities
│   ├── staubli_tx40/          # Industrial manipulator examples
│   ├── talos/                  # Humanoid robot examples
│   ├── tiago/                  # Mobile manipulator examples
│   └── ur10/                   # Universal Robots examples
├── models/                     # URDF models and robot descriptions
│   ├── hey5_description/       # Hand gripper description
│   ├── mate_description/       # 3DOF manipulator description
│   ├── pmb2_description/       # Mobile base description
│   ├── realsense2_description/ # RealSense camera description
│   ├── staubli_tx40_description/ # Staubli TX40 robot description
│   ├── talos_description/      # TALOS humanoid robot description
│   ├── tiago_description/      # TIAGo robot description
│   └── ur_description/         # Universal Robots description
├── requirements.txt            # Python dependencies for examples
└── environment.yml            # Conda environment for examples
```

## Examples Overview

### Industrial Manipulator - Staubli TX40
- **Dynamic identification**: Identify inertial parameters, friction, and actuator characteristics

Location: `examples/staubli_tx40/`

### Universal Robots UR10
- **Geometric calibration**: Calibrate kinematic parameters using RealSense camera and checkerboard
- **Dynamic identification**: Identify dynamic parameters

Location: `examples/ur10/`

### 3DOF Manipulator MATE
- **Geometric calibration**: Calibrate using ArUco markers and computer vision

Location: `examples/mate/`

### Mobile Manipulator TIAGo
- **Dynamic identification**: Including friction models and actuator inertia
- **Geometric calibration**: Using motion capture or onboard head camera
- **Mobile base modeling**: Suspension parameter identification
- **Joint backlash modeling**: Advanced modeling techniques

Location: `examples/tiago/`

### Humanoid Robot TALOS
- **Torso-arm calibration**: Geometric calibration using motion capture
- **Whole-body calibration**: Using onboard sensors and planar constraints

Location: `examples/talos/`

## 📊 Performance Improvements

The modernized infrastructure provides significant performance improvements:

### Benchmarking Results
- **Vectorized differentiation**: 2-5x faster than traditional finite differences
- **Parallel processing**: 1.5-3x speedup for multiple file operations
- **Cached operations**: 10-50x faster for repeated computations
- **Memory efficiency**: Reduced allocation with in-place operations

### Try the Performance Demo
```bash
cd examples/
python performance_demo.py
```

Sample output:
```
🎯 FIGAROH Performance Optimization Demo
Generated trajectory: 1000 samples, 6 joints
📈 Differentiation speedup: 4.2x
📈 Filtering speedup: 2.8x
✅ Optimizations enabled: 4
```

### Run Usage Examples
```bash
cd examples/
python usage_examples.py
```

This demonstrates all the new infrastructure features with practical examples.

## Getting Started

Each example folder contains:
- `config/`: Configuration files (YAML format)
- `data/`: Sample datasets
- `calibration.py`: Geometric calibration scripts
- `identification.py`: Dynamic identification scripts
- `optimal_config.py`: Optimal posture generation
- `optimal_trajectory.py`: Optimal trajectory generation
- `update_model.py`: Model parameter update utilities
- `urdf/`: Robot-specific URDF files (when needed)
- `utils/`: Robot-specific tools and utilities

The `shared/` directory contains base classes and common utilities used across multiple robot implementations.

The `models/` directory contains shared URDF robot descriptions that can be used across multiple examples.

### Basic Workflow

1. **Choose your robot**: Navigate to the appropriate example folder
2. **Review configuration**: Check the `config/*.yaml` files
3. **Prepare data**: Use provided sample data or your own in CSV format
4. **Run calibration/identification**: Execute the Python scripts
5. **Update model**: Use results to update your URDF model

### Example: Running TIAGo Dynamic Identification

```bash
cd examples/tiago
python identification.py
```

### Example: Running Staubli TX40 Dynamic Identification

```bash
cd examples/staubli_tx40
python identification.py
```

## Data Format

All examples use a standardized CSV data format. See individual example READMEs for specific format requirements.

## Contributing

If you have examples for additional robots or new calibration/identification methods, please:

1. Fork this repository
2. Add your example following the established structure
3. Include sample data and documentation
4. Submit a pull request

## Citation

If you use these examples in your research, please cite the main FIGAROH paper:

```bibtex
@inproceedings{nguyen2023figaroh,
  title={FIGAROH: a Python toolbox for dynamic identification and geometric calibration of robots and humans},
  author={Nguyen, Dinh Vinh Thanh and Bonnet, Vincent and Maxime, Sabbah and Gautier, Maxime and Fernbach, Pierre and others},
  booktitle={IEEE-RAS International Conference on Humanoid Robots},
  pages={1--8},
  year={2023},
  address={Austin, TX, United States},
  doi={10.1109/Humanoids57100.2023.10375232},
  url={https://hal.science/hal-04234676v2}
}
```

## License

This project is licensed under the same terms as the main FIGAROH package. See [LICENSE](https://github.com/thanhndv212/figaroh-plus/blob/main/LICENSE) for details.

## Support

For questions about the examples:
- Open an issue in this repository for example-specific questions
- Open an issue in the [main FIGAROH repository](https://github.com/thanhndv212/figaroh-plus/issues) for library-related questions
