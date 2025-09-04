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
UR10 Optimal Configuration Generation Script

Generates optimal robot configurations for kinematic calibration using 
D-optimal experimental design with the refactored UR10OptimalCalibration class.
"""

import time
import sys
import os
import numpy as np

# Add the parent directory to Python path to enable proper imports
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(os.path.dirname(current_dir))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from examples.ur10.utils.ur10_tools import UR10OptimalCalibration

from figaroh.tools.robot import load_robot


def main():
    """Main function for UR10 optimal configuration generation."""
    
    print("UR10 Optimal Configuration Generation")
    print("=" * 50)
    
    # 1. Load robot model
    print("\n1. Loading UR10 robot model...")
    robot = load_robot(
        "urdf/ur10_robot.urdf",
        package_dirs="models",
        load_by_urdf=True,
    )

    # 2. Create optimal calibration instance
    print("\n2. Setting up optimal calibration...")
    opt_calib = UR10OptimalCalibration(robot, "config/ur10_config.yaml")
    
    print(f"Calibration model: {opt_calib.calib_config['calib_model']}")
    print(f"Minimum configurations required: {opt_calib.minNbChosen}")
    
    # 3. Solve optimal configuration problem
    print("\n3. Solving optimal configuration selection...")
    
    start_time = time.time()
    opt_calib.solve(save_file=True)
    solve_time = time.time() - start_time
    
    print(f"Optimization completed in {solve_time:.2f} seconds")
    
    # 4. Display results
    print("\n4. Results Summary:")
    
    if (hasattr(opt_calib, 'optimal_configurations') and 
        'calibration_joint_configurations' in opt_calib.optimal_configurations):
        selected_configs = opt_calib.optimal_configurations['calibration_joint_configurations']
        print(f"Selected {len(selected_configs)} optimal configurations")
        print(f"Total candidates: {opt_calib.calib_config['NbSample']}")
        ratio = len(selected_configs) / opt_calib.calib_config['NbSample']
        print(f"Selection ratio: {ratio:.2%}")
    
    # 5. Show optimization quality
    if hasattr(opt_calib, 'detroot_whole'):
        print(f"Information matrix determinant root: {opt_calib.detroot_whole:.4e}")
    
    if hasattr(opt_calib, 'optimal_weights'):
        weights = opt_calib.optimal_weights
        if hasattr(weights, '__len__') and len(weights) > 0:
            print(f"Weight sum: {np.sum(weights):.4f}")
    
    print("\n" + "=" * 50)
    print("UR10 Optimal Configuration Generation Completed!")
    print("Results saved to 'results/' directory")


if __name__ == "__main__":
    main()
