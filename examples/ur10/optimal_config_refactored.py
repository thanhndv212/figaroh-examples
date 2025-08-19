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

from utils.ur10_tools import UR10OptimalCalibration
from figaroh.tools.robot import load_robot
import argparse


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Generate optimal configurations for UR10 calibration",
        add_help=True
    )
    parser.add_argument(
        "-n", "--nb_chosen", 
        type=int,
        default=15, 
        dest="nb_chosen",
        help="Number of optimal configurations to select"
    )
    return parser.parse_args()


def main():
    """Main function for UR10 optimal configuration generation."""
    
    # Parse command line arguments
    args = parse_args()
    nb_chosen = args.nb_chosen
    
    # Load UR10 robot model
    ur10 = load_robot(
        "urdf/ur10_robot.urdf",
        package_dirs="../../models",
        load_by_urdf=True,
    )
    
    # Create optimal calibration object
    ur10_optcalib = UR10OptimalCalibration(
        robot=ur10,
        config_file="config/ur10_config_new.yaml"
    )
    
    # Generate optimal configurations
    optimal_configs = ur10_optcalib.solve(nb_chosen=nb_chosen)
    
    # Display results
    print("Optimal configuration generation completed successfully!")
    print(f"Generated {len(optimal_configs)} optimal configurations")
    
    # Plot and save results
    ur10_optcalib.plot_results()
    ur10_optcalib.save_results("results/")
    
    print("Optimal configuration results saved to results/ directory")


if __name__ == "__main__":
    main()
