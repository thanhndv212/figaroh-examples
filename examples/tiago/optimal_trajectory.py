#!/usr/bin/env python3
"""
Improved Optimal Trajectory Generation using IPOPT for TIAGo Robot

This module implements a refactored and improved version of the trajectory
optimization with better structure, error handling, and modularity. It uses
the generalized IPOPT framework from figaroh.tools.robotipopt.
"""

import time
import logging
from matplotlib import pyplot as plt
from figaroh.tools.robot import load_robot
from utils.simplified_colission_model import build_tiago_simplified
from utils.tiago_tools import OptimalTrajectoryIPOPT


def main():
    """Main function to run the improved optimal trajectory generation."""
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger(__name__)
    
    try:
        logger.info("Loading TIAGo robot model...")
        
        # Load robot
        robot = load_robot(
            "urdf/tiago_48_schunk.urdf",
            load_by_urdf=True,
            robot_pkg="tiago_description",
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
        
        # Build simplified collision model
        robot = build_tiago_simplified(robot)
        
        # Create optimal trajectory generator
        opt_traj = OptimalTrajectoryIPOPT(robot, active_joints)
        
        # Solve optimization problem
        start_time = time.time()
        results = opt_traj.solve(stack_reps=2)
        total_time = time.time() - start_time
        
        logger.info(f"Total runtime: {total_time:.2f} seconds")
        
        if results['T_F']:
            logger.info(
                f"Final regressor shape: {results['final_regressor_shape']}"
            )
            
            # Plot results
            opt_traj.plot_results()
            
            # Plot condition number evolution if we have iteration data
            if results['iteration_data']:
                plt.figure(figsize=(12, 6))
                for i, iter_data in enumerate(results['iteration_data']):
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
        
        return results
        
    except Exception as e:
        logger.error(f"Error in main: {e}")
        raise


if __name__ == "__main__":
    results = main()
