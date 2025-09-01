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

from utils.ur10_tools import UR10Identification
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
    base_params = ur10_identif.solve()
    
    # Display results
    print("Dynamic identification completed successfully!")
    print(f"RMS error: {ur10_identif.rms_error:.6f}")
    print(f"Correlation: {ur10_identif.correlation:.4f}")
    print(f"Number of base parameters: {len(base_params)}")
    
    # Plot and save results using base class methods
    ur10_identif.plot_results()
    ur10_identif.save_results("results/")
    
    print("Identification results saved to results/ directory")


if __name__ == "__main__":
    main()
