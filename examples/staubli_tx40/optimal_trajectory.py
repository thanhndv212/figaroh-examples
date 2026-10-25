# Copyright [2021-2026] Thanh Nguyen
# Copyright [2022-2024] [CNRS, Toward SAS]

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

# http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Condition-number-optimal excitation trajectory for TX40 dynamic identification.

    python optimal_trajectory.py            # 2 segments, seed 0

The design minimises the condition number of the base regressor that
``identification.py`` fits (coupled-wrist columns included), using core's
IPOPT generator. Simulated design only: the trajectory has not been run on a
TX40. The URDF has no floor or cell, so check it against the installation
before running it. See docs/development/tx40-optimal-trajectory.md.
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

import numpy as np
import yaml

# Add project root to path for imports (prefer `pip install -e .` instead)
project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from examples.staubli_tx40.utils.staubli_tx40_tools import (  # noqa: E402
    TX40OptimalTrajectory,
)
from figaroh.tools.robot import load_robot  # noqa: E402


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Staubli TX40 optimal excitation trajectory"
    )
    parser.add_argument(
        "--config",
        type=str,
        default="config/staubli_tx40_unified_config.yaml",
        help="Path to unified config YAML file",
    )
    parser.add_argument(
        "--urdf",
        type=str,
        default="urdf/tx40_mdh_modified.urdf",
        help="Path to robot URDF file",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=0,
        help="Random seed for the initial waypoints (reproducible runs)",
    )
    parser.add_argument(
        "--segments", type=int, default=2, help="Number of stacked segments"
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true", help="Enable verbose (INFO) logging"
    )
    return parser.parse_args()


def main(args: argparse.Namespace) -> None:
    """Main function for TX40 optimal trajectory generation."""
    for path in (args.urdf, args.config):
        if not Path(path).exists():
            print(f"Error: file not found: {path}", file=sys.stderr)
            sys.exit(1)

    robot = load_robot(args.urdf, package_dirs="../../models", load_by_urdf=True)
    with open(args.config) as f:
        cfg = yaml.safe_load(f)
    active_joints = cfg["robot"]["properties"]["joints"]["active_joints"]

    traj = TX40OptimalTrajectory(robot, active_joints, args.config)

    # Seeded: base indices and waypoints are sampled randomly
    np.random.seed(args.seed)
    traj.initialize()

    results = traj.solve(stack_reps=args.segments)
    n_solved = len(results["T_F"])
    if n_solved < args.segments:
        print(
            f"Failed to generate optimal trajectory: {n_solved}/{args.segments} "
            "segments solved. Check constraints and parameters.",
            file=sys.stderr,
        )
        sys.exit(1)

    W = None
    for q, v, a in zip(results["P_F"], results["V_F"], results["A_F"]):
        W = traj._stack_base_regressors(q, v, a, W)
    print(
        f"{n_solved} segments, {len(traj.idx_b)} base parameters, "
        f"base-regressor condition number {np.linalg.cond(W):.3g}"
    )
    print("Optimal trajectory generation completed successfully!")
    traj.plot_results()


if __name__ == "__main__":
    args = parse_args()
    logging.basicConfig(
        level=logging.INFO if args.verbose else logging.WARNING,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    main(args)
