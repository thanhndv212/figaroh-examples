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

from utils.tiago_tools import TiagoOptimalCalibration
from figaroh.tools.robot import load_robot
import argparse


def parse_args():
    parser = argparse.ArgumentParser(
        description="parse calibration setups", add_help=False
    )
    parser.add_argument(
        "-e", "--end_effector", default="hey5", dest="end_effector"
    )
    parser.add_argument(
        "-u", "--load_by_urdf", default=True, dest="load_by_urdf"
    )
    args = parser.parse_args()
    return args


def main():
    args = parse_args()
    if args is not None:
        end_effector = args.end_effector
    else:
        end_effector = "schunk"

    # Load robot model
    tiago = load_robot(
        "urdf/tiago_48_{}.urdf".format(end_effector),
        load_by_urdf=True,
        robot_pkg="tiago_description",
    )

    # Create optimal calibration object
    tiago_optcalib = TiagoOptimalCalibration(
        tiago, "config/tiago_config_{}.yaml".format(end_effector)
    )
    
    # Set calibration parameters
    tiago_optcalib.param["known_baseframe"] = False
    tiago_optcalib.param["known_tipframe"] = False
    
    # Solve for optimal configurations
    tiago_optcalib.solve(
        file_name="tiago_optimal_configurations_{}.yaml".format(end_effector),
        write_file=True
    )


if __name__ == "__main__":
    main()
