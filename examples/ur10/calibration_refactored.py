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

from utils.ur10_tools import UR10Calibration
from figaroh.tools.robot import load_robot


def main():
    """Main function for UR10 kinematic calibration."""
    
    # Load UR10 robot model
    ur10 = load_robot(
        "urdf/ur10_robot.urdf",
        package_dirs="../../models",
        load_by_urdf=True,
    )
    
    # Create calibration object
    ur10_calib = UR10Calibration(
        robot=ur10,
        config_file="config/ur10_config_new.yaml"
    )
    
    # Set required parameters that aren't in config file
    ur10_calib.param["known_baseframe"] = False
    ur10_calib.param["known_tipframe"] = False
    
    print("UR10 Calibration object created successfully!")
    print("Ready to initialize and solve calibration...")
    
    # Initialize the calibration
    ur10_calib.initialize()
    print("Calibration initialized successfully!")
    
    # Solve the calibration
    ur10_calib.solve()
    print("Calibration solved successfully!")


if __name__ == "__main__":
    main()
