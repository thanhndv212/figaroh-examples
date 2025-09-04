#!/usr/bin/env python3

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
TALOS torso-arm calibration using the FIGAROH framework.

This script demonstrates calibration of the TALOS humanoid robot's
torso-arm kinematic chain using experimental data.
"""

import os
import sys

# Add the parent directory to Python path to enable proper imports
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(os.path.dirname(current_dir))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from figaroh.tools.robot import load_robot
from examples.talos.utils.talos_tools import TALOSCalibration


def main(data_type="experimental", visualization=True, verbose=True):
    """
    Main function for TALOS torso-arm calibration.
    
    Args:
        data_type (str): Type of data to use ('experimental' or 'sample')
        visualization (bool): Whether to show visualization plots
        verbose (bool): Whether to enable verbose output
    """
    print("=" * 60)
    print("TALOS Torso-Arm Calibration with FIGAROH")
    print("=" * 60)
        
    # Load robot
    robot = load_robot(
        "urdf/talos_full_v2.urdf",
        package_dirs="models",
        load_by_urdf=True,
    )
    
    # Initialize calibration
    print("\nInitializing TALOS calibration...")
    calibration = TALOSCalibration(
        robot=robot,
        config_file="config/talos_config.yaml"
    )
    
    # Set required parameters that aren't in config file
    calibration.calib_config["known_baseframe"] = False
    calibration.calib_config["known_tipframe"] = False
    calibration.initialize()
    
    # Run calibration
    print("\n" + "=" * 40)
    print("Running calibration optimization...")
    print("=" * 40)
    
    calibration.solve(
        method="lm",
        max_iterations=3,
        outlier_threshold=3.0,
        enable_logging=verbose,
        plotting=True
    )
    
    # Display calibration parameters
    param_count = len(calibration.calib_config['param_name'])
    print(f"\nCalibration parameters ({param_count} total):")
    for i, param_name in enumerate(calibration.calib_config['param_name']):
        print(f"  {i:2d}: {param_name}")
    

if __name__ == "__main__":
    try:
        main(
            data_type="experimental",
            visualization=True,
            verbose=False
        )
        
        print("\nCalibration completed successfully!")
        
    except Exception as e:
        print(f"\nError during calibration: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
