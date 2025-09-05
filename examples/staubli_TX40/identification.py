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
Refactored Staubli TX40 dynamic parameter identification using base classes.
This demonstrates the clean separation between robot-specific and general
identification functionality.
"""

import sys
import os

# Add the parent directory to Python path to enable proper imports
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(os.path.dirname(current_dir))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from examples.staubli_tx40.utils.staubli_tx40_tools import TX40Identification
from figaroh.tools.robot import load_robot


def main():
    """Main function for TX40 dynamic parameter identification."""
    # Load robot model
    robot = load_robot(
        "urdf/tx40_mdh_modified.urdf", package_dirs="models"
    )

    # Create TX40 identification object
    tx40_iden = TX40Identification(robot, "config/TX40_config.yaml")

    # Initialize identification process with data loading and processing
    tx40_iden.initialize()

    # Solve identification with TX40-specific features
    tx40_iden.solve(
        decimate=True,          # Apply TX40-specific decimation
        plotting=True,          # Generate identification plots
        save_results=False,       # Save parameters to CSV files
        wls=False               # Use weighted least squares
    )

    # Print results summary
    print("\n" + "=" * 60)
    print("TX40 DYNAMIC PARAMETER IDENTIFICATION RESULTS")
    print("=" * 60)
    
    print(
        f"Number of base parameters identified: "
        f"{len(tx40_iden.params_base)}"
    )
    print(f"Correlation coefficient: {tx40_iden.correlation:.4f}")
    
    if hasattr(tx40_iden, 'result'):
        for key, value in tx40_iden.result.items():
            if isinstance(value, (int, float)):
                if isinstance(value, float):
                    print(f"{key}: {value:.6f}")
                else:
                    print(f"{key}: {value}")
            else:
                print(f"{key}: {type(value).__name__} of length {len(value)}")

    print("\nBase parameters:")
    for i, param_name in enumerate(tx40_iden.params_base):
        print(f"{i + 1:2d}. {param_name}: {tx40_iden.phi_base[i]:10.6f}")

    print("\nIdentification completed successfully!")
    return tx40_iden


if __name__ == "__main__":
    # Run the identification
    identification_result = main()
