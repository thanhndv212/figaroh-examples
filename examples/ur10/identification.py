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

import sys
import os

# Add the parent directory to Python path to enable proper imports
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(os.path.dirname(current_dir))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from examples.ur10.utils.ur10_tools import UR10Identification
from figaroh.tools.robot import load_robot


def main():
    """Main function for UR10 dynamic parameter identification."""
    
    # Load UR10 robot model
    ur10 = load_robot(
        "urdf/ur10_robot.urdf",
        package_dirs="../../models",
        load_by_urdf=True,
    )
    
    # Create identification object
    ur10_identif = UR10Identification(
        robot=ur10,
        config_file="config/ur10_config.yaml"
    )
    
    # Initialize with data processing
    ur10_identif.initialize()
    
    # Perform identification using the base class solve method
    ur10_identif.solve(
        decimate=False,
        plotting=True,
        save_results=False,
    )
    
    # Display results
    # Print results summary
    print("\n" + "=" * 60)
    print("UR10 DYNAMIC PARAMETER IDENTIFICATION RESULTS")
    print("=" * 60)
    
    print(
        f"Number of base parameters identified: "
        f"{len(ur10_identif.params_base)}"
    )
    print(f"Correlation coefficient: {ur10_identif.correlation:.4f}")

    if hasattr(ur10_identif, 'result'):
        for key, value in ur10_identif.result.items():
            if isinstance(value, (int, float)):
                if isinstance(value, float):
                    print(f"{key}: {value:.6f}")
                else:
                    print(f"{key}: {value}")
            else:
                print(f"{key}: {type(value).__name__} of length {len(value)}")

    print("\nBase parameters:")
    for i, param_name in enumerate(ur10_identif.params_base):
        print(f"{i + 1:2d}. {param_name}: {ur10_identif.phi_base[i]:10.6f}")

    print("\nIdentification completed successfully!")
    return ur10_identif


if __name__ == "__main__":
    main()
