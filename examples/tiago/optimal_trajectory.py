#!/usr/bin/env python3
"""
Optimal Trajectory Generation using IPOPT for TIAGo Robot
"""

from matplotlib import pyplot as plt
from figaroh.tools.robot import load_robot
from utils.simplified_colission_model import build_tiago_simplified
from utils.tiago_tools import OptimalTrajectoryIPOPT


def plot_condition_number_evolution(results):
    """Plot the evolution of condition number during optimization."""
    if not results['iteration_data']:
        return
        
    plt.figure(figsize=(12, 6))
    
    for i, iter_data in enumerate(results['iteration_data']):
        if 'iterations' in iter_data and 'obj_values' in iter_data:
            plt.plot(
                iter_data['iterations'], iter_data['obj_values'],
                label=f"Segment {i + 1}", marker='o', markersize=3
            )
    
    plt.title("Evolution of Condition Number of Base Regressor")
    plt.ylabel("Cond(Wb)")
    plt.xlabel("Iteration")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.yscale("log")
    plt.tight_layout()
    plt.show()


def main():
    """Main function for TIAGo optimal trajectory generation."""
    # Load TIAGo robot model
    robot = load_robot(
        "urdf/tiago_48_schunk.urdf",
        load_by_urdf=True,
        robot_pkg="tiago_description",
    )
    
    # Define active joints for trajectory optimization
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
    
    # Build simplified collision model
    robot = build_tiago_simplified(robot)
    
    # Create trajectory optimizer
    config_file = "config/tiago_config.yaml"
    opt_traj = OptimalTrajectoryIPOPT(robot, active_joints, config_file)
    
    # Run trajectory optimization
    results = opt_traj.solve(stack_reps=2)
    
    # Plot results
    if results['T_F']:
        opt_traj.plot_results()
        plot_condition_number_evolution(results)
        print(f"Generated {len(results['T_F'])} trajectory segments")
    
    return results


if __name__ == "__main__":
    results = main()
    
    if results and results['T_F']:
        print("\nOptimization completed successfully!")
        print(f"Generated {len(results['T_F'])} trajectory segments")
    else:
        print("\nOptimization failed or produced no results")
