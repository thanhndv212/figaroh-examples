#!/usr/bin/env python3
"""
Optimal Trajectory Generation using IPOPT for TIAGo Robot
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

# Add project root to path for imports (prefer `pip install -e .` instead)
project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

import yaml  # noqa: E402
from matplotlib import pyplot as plt  # noqa: E402
from figaroh.tools.robot import load_robot  # noqa: E402
from examples.tiago.utils.simplified_collision_model import (  # noqa: E402
    build_tiago_simplified,
)
from examples.tiago.utils.tiago_tools import (  # noqa: E402
    OptimalTrajectoryIPOPT,
)


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="TIAGo optimal trajectory generation using IPOPT"
    )
    parser.add_argument(
        "--config",
        type=str,
        default="config/tiago_unified_config.yaml",
        help="Path to unified config YAML file",
    )
    parser.add_argument(
        "--urdf",
        type=str,
        default="urdf/tiago_48_schunk.urdf",
        help="Path to robot URDF file",
    )
    parser.add_argument(
        "--verbose",
        "-v",
        action="store_true",
        help="Enable verbose (INFO) logging",
    )
    return parser.parse_args()


def plot_condition_number_evolution(results: dict) -> None:
    """Plot the evolution of condition number during optimization."""
    if not results.get("iteration_data"):
        return

    plt.figure(figsize=(12, 6))

    for i, iter_data in enumerate(results["iteration_data"]):
        if "iterations" in iter_data and "obj_values" in iter_data:
            plt.plot(
                iter_data["iterations"],
                iter_data["obj_values"],
                label=f"Segment {i + 1}",
                marker="o",
                markersize=3,
            )

    plt.title("Evolution of Condition Number of Base Regressor")
    plt.ylabel("Cond(Wb)")
    plt.xlabel("Iteration")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.yscale("log")
    plt.tight_layout()
    plt.show()


def main() -> dict:
    """Main function for TIAGo optimal trajectory generation."""
    args = parse_args()

    # Configure logging after parsing args
    logging.basicConfig(
        level=logging.INFO if args.verbose else logging.WARNING,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    # Validate files exist
    config_path = Path(args.config)
    urdf_path = Path(args.urdf)
    if not config_path.exists():
        print(f"Error: Config file not found: {config_path}", file=sys.stderr)
        sys.exit(1)
    if not urdf_path.exists():
        print(f"Error: URDF file not found: {urdf_path}", file=sys.stderr)
        sys.exit(1)

    try:
        # Load TIAGo robot model
        robot = load_robot(
            str(urdf_path),
            load_by_urdf=True,
            robot_pkg="tiago_description",
        )

        # Read active joints from config (instead of hardcoding)
        with open(config_path) as f:
            cfg = yaml.safe_load(f)
        active_joints = cfg["robot"]["properties"]["joints"]["active_joints"]

        # Build simplified collision model
        robot = build_tiago_simplified(robot)

        # Create trajectory optimizer
        opt_traj = OptimalTrajectoryIPOPT(robot, active_joints, str(config_path))

        ps = opt_traj.identif_config

        # Joint parameters
        ps["active_joints"] = active_joints
        ps["act_Jid"] = [opt_traj.model.getJointId(i) for i in ps["active_joints"]]
        ps["act_J"] = [opt_traj.model.joints[jid] for jid in ps["act_Jid"]]
        ps["act_idxq"] = [J.idx_q for J in ps["act_J"]]
        ps["act_idxv"] = [J.idx_v for J in ps["act_J"]]

        # Initialize
        opt_traj.initialize()

        # Run trajectory optimization; a segment is kept only if it is feasible
        n_segments = 2
        results = opt_traj.solve(stack_reps=n_segments)
        n_solved = len(results["T_F"])

        if n_solved < n_segments:
            print(
                f"Error: {n_solved}/{n_segments} trajectory segments solved",
                file=sys.stderr,
            )
            sys.exit(1)

        opt_traj.plot_results()
        plot_condition_number_evolution(results)
        return results
    except Exception as e:
        print(
            f"Error during optimal trajectory generation: {e}",
            file=sys.stderr,
        )
        sys.exit(1)


if __name__ == "__main__":
    results = main()
    print("\nOptimization completed successfully!")
    print(f"Generated {len(results['T_F'])} trajectory segments")
