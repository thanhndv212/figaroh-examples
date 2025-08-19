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

from utils.tiago_tools import TiagoOptimalTrajectory
from figaroh.tools.robot import load_robot


# Load robot model
tiago = load_robot(
    "urdf/tiago_48_schunk.urdf",
    load_by_urdf=True,
    robot_pkg="tiago_description"
)

# Define active joints
active_joints = [
    "torso_lift_joint",
    "arm_1_joint",
    "arm_2_joint",
    "arm_3_joint",
    "arm_4_joint",
    "arm_5_joint",
    "arm_6_joint",
    "arm_7_joint",
]

# Create optimal trajectory object
tiago_opt_traj = TiagoOptimalTrajectory(
    tiago,
    "config/tiago_config.yaml",
    active_joints
)

# Generate optimal trajectory
tiago_opt_traj.solve(
    n_wps=5,        # Number of waypoints per segment
    stack_reps=2,   # Number of trajectory segments
    t_s=2           # Time between waypoints (seconds)
)

print("Optimal trajectory generation completed!")
print(f"Generated {len(tiago_opt_traj.T_F)} trajectory segments")
