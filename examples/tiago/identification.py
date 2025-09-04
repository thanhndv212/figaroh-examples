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
from examples.tiago.utils.tiago_tools import TiagoIdentification
from figaroh.tools.robot import load_robot


def main():
    """Main function for Tiago dynamic parameter identification."""
    # Load robot model
    tiago = load_robot(
        "urdf/tiago_48_schunk.urdf",
        load_by_urdf=True,
        robot_pkg="tiago_description"
    )

    # Create identification object
    tiago_iden = TiagoIdentification(tiago, "config/tiago_config.yaml")

    # Define additional parameters excluded from yaml files
    ps = tiago_iden.identif_config
    ps["reduction_ratio"] = {
        "torso_lift_joint": 1,
        "arm_1_joint": 100,
        "arm_2_joint": 100,
        "arm_3_joint": 100,
        "arm_4_joint": 100,
        "arm_5_joint": 336,
        "arm_6_joint": 336,
        "arm_7_joint": 336,
    }
    ps["kmotor"] = {
        "torso_lift_joint": 1,
        "arm_1_joint": 0.136,
        "arm_2_joint": 0.136,
        "arm_3_joint": -0.087,
        "arm_4_joint": -0.087,
        "arm_5_joint": -0.0613,
        "arm_6_joint": -0.0613,
        "arm_7_joint": -0.0613,
    }

    ps["active_joints"] = [
        "torso_lift_joint",
        "arm_1_joint",
        "arm_2_joint",
        "arm_3_joint",
        "arm_4_joint",
        "arm_5_joint",
        "arm_6_joint",
        "arm_7_joint",
    ]

    # Joint parameters
    ps["act_Jid"] = [tiago_iden.model.getJointId(i) for i in ps["active_joints"]]
    ps["act_J"] = [tiago_iden.model.joints[jid] for jid in ps["act_Jid"]]
    ps["act_idxq"] = [J.idx_q for J in ps["act_J"]]
    ps["act_idxv"] = [J.idx_v for J in ps["act_J"]]

    # Dataset paths
    ps["pos_data"] = "data/identification/dynamic/tiago_position.csv"
    ps["vel_data"] = "data/identification/dynamic/tiago_velocity.csv"
    ps["torque_data"] = "data/identification/dynamic/tiago_effort.csv"

    # Initialize identification process
    # Note: truncate parameter now accepts:
    # - None: no truncation
    # - (start, end): custom truncation indices
    tiago_iden.initialize(truncate=(921, 6791))

    # Solve identification
    tiago_iden.solve(
        decimate=True,
        plotting=True,
        save_results=False,
    )

    # Print results summary
    print("\n" + "=" * 60)
    print("TX40 DYNAMIC PARAMETER IDENTIFICATION RESULTS")
    print("=" * 60)

    print(
        f"Number of base parameters identified: "
        f"{len(tiago_iden.params_base)}"
    )
    print(f"Correlation coefficient: {tiago_iden.correlation:.4f}")

    if hasattr(tiago_iden, 'result'):
        for key, value in tiago_iden.result.items():
            if isinstance(value, (int, float)):
                if isinstance(value, float):
                    print(f"{key}: {value:.6f}")
                else:
                    print(f"{key}: {value}")
            else:
                print(f"{key}: {type(value).__name__} of length {len(value)}")

    print("\nBase parameters:")
    for i, param_name in enumerate(tiago_iden.params_base):
        print(f"{i + 1:2d}. {param_name}: {tiago_iden. phi_base[i]:10.6f}")

    print("\nIdentification completed successfully!")
    return tiago_iden


if __name__ == "__main__":
    main()
