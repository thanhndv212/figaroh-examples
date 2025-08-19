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

from utils.ur10_tools import UR10OptimalTrajectory
from figaroh.tools.robot import load_robot


def main():
    """Main function for UR10 optimal trajectory generation."""
    
    # Load UR10 robot model
    ur10 = load_robot(
        "urdf/ur10_robot.urdf",
        package_dirs="../../models",
        load_by_urdf=True,
    )
    
    # Create optimal trajectory object
    ur10_traj = UR10OptimalTrajectory(
        robot=ur10,
        config_file="config/ur10_config.yaml"
    )
    
    # Generate optimal trajectory
    optimal_trajectory = ur10_traj.solve(n_iterations=100)
    
    if optimal_trajectory is not None:
        # Display results
        print("Optimal trajectory generation completed successfully!")
        print(f"Final condition number: {optimal_trajectory['condition_number']:.2f}")
        print(f"Trajectory duration: {ur10_traj.trajectory_duration} seconds")
        
        # Plot and save results
        ur10_traj.plot_results()
        ur10_traj.save_results("results/")
        
        print("Optimal trajectory results saved to results/ directory")
    else:
        print("Failed to generate optimal trajectory. Check constraints and parameters.")


if __name__ == "__main__":
    main()
