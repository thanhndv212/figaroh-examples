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

from __future__ import annotations

import argparse
import logging
import sys
import numpy as np
import yaml
from pathlib import Path

# Add project root to path for imports (prefer `pip install -e .` instead)
project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from examples.ur10.utils.ur10_tools import OptimalTrajectoryIPOPT  # noqa: E402
from figaroh.tools.robot import load_robot  # noqa: E402


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description="UR10 optimal trajectory generation")
    parser.add_argument(
        "--config",
        type=str,
        default="config/ur10_unified_config.yaml",
        help="Path to unified config YAML file",
    )
    parser.add_argument(
        "--urdf",
        type=str,
        default="urdf/ur10_robot.urdf",
        help="Path to robot URDF file",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=0,
        help="Random seed for the initial waypoints (reproducible runs)",
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true", help="Enable verbose (INFO) logging"
    )
    return parser.parse_args()


def main(args: argparse.Namespace) -> None:
    """Main function for UR10 optimal trajectory generation."""
    # Validate input files
    urdf_path = Path(args.urdf)
    if not urdf_path.exists():
        print(f"Error: URDF file not found: {urdf_path}", file=sys.stderr)
        sys.exit(1)

    config_path = Path(args.config)
    if not config_path.exists():
        print(f"Error: Config file not found: {config_path}", file=sys.stderr)
        sys.exit(1)

    try:
        # Load UR10 robot model
        ur10 = load_robot(
            args.urdf,
            package_dirs="../../models",
            load_by_urdf=True,
        )

        # Load active joints from unified config (eliminates DRY with config)
        with open(args.config) as f:
            cfg = yaml.safe_load(f)
        active_joints = cfg["robot"]["properties"]["joints"]["active_joints"]

        # Narrow the joint limits to the configured collision-free box; the
        # optimiser's waypoint pool and bounds come from the model limits.
        box = cfg["tasks"]["optimal_trajectory"]["problem"].get("joint_box")
        if box:
            idx = [
                ur10.model.joints[ur10.model.getJointId(j)].idx_q for j in active_joints
            ]
            ur10.model.lowerPositionLimit[idx] = box["lower"]
            ur10.model.upperPositionLimit[idx] = box["upper"]

        # Create optimal trajectory object
        ur10_traj = OptimalTrajectoryIPOPT(
            robot=ur10,
            active_joints=active_joints,
            config_file=args.config,
        )
        ps = ur10_traj.identif_config

        # Joint parameters
        ps["active_joints"] = active_joints
        ps["act_Jid"] = [ur10_traj.model.getJointId(i) for i in ps["active_joints"]]
        ps["act_J"] = [ur10_traj.model.joints[jid] for jid in ps["act_Jid"]]
        ps["act_idxq"] = [J.idx_q for J in ps["act_J"]]
        ps["act_idxv"] = [J.idx_v for J in ps["act_J"]]

        # Initialize (seeded: base indices and waypoints are sampled randomly)
        np.random.seed(args.seed)
        ur10_traj.initialize()

        # Generate optimal trajectory; a segment is kept only if it is feasible
        n_segments = 2
        results = ur10_traj.solve(stack_reps=n_segments)
        n_solved = len(results["T_F"])

        if n_solved < n_segments:
            print(
                f"Failed to generate optimal trajectory: {n_solved}/{n_segments} "
                "segments solved. Check constraints and parameters.",
                file=sys.stderr,
            )
            sys.exit(1)

        print("Optimal trajectory generation completed successfully!")
        ur10_traj.plot_results()
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        raise


if __name__ == "__main__":
    args = parse_args()
    logging.basicConfig(
        level=logging.INFO if args.verbose else logging.WARNING,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    main(args)
