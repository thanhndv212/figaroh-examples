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
import yaml
import numpy as np
from pathlib import Path

# Add project root to path for imports (prefer `pip install -e .` instead)
project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from examples.kitt_1_5.utils.kitt_1_5_tools import KITT1_5OptimalTrajectoryIPOPT
from figaroh.tools.robot import load_robot


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="KITT1_5V3 right-arm optimal trajectory generation"
    )
    parser.add_argument(
        "--config",
        type=str,
        default="config/kitt_1_5_unified_config.yaml",
        help="Path to unified config YAML file",
    )
    parser.add_argument(
        "--urdf",
        type=str,
        default="urdf/right_arm_robot.urdf",
        help="Path to robot URDF file",
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true", help="Enable verbose (INFO) logging"
    )
    return parser.parse_args()


def main(args: argparse.Namespace) -> None:
    """Main function for KITT1_5V3 right-arm optimal trajectory generation."""
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
        # Load KITT1_5V3 right-arm robot model (mesh files live next to the
        # sub-model URDF, so package_dirs points at the local urdf/ folder)
        robot = load_robot(
            args.urdf,
            package_dirs="urdf",
            load_by_urdf=True,
        )

        # Load active joints from unified config (eliminates DRY with config)
        with open(args.config) as f:
            cfg = yaml.safe_load(f)
        active_joints = cfg["robot"]["properties"]["joints"]["active_joints"]

        # Create optimal trajectory object
        traj = KITT1_5OptimalTrajectoryIPOPT(
            robot=robot,
            active_joints=active_joints,
            config_file=args.config,
        )
        ps = traj.identif_config

        # Joint parameters
        ps["active_joints"] = active_joints
        ps["act_Jid"] = [traj.model.getJointId(i) for i in ps["active_joints"]]
        ps["act_J"] = [traj.model.joints[jid] for jid in ps["act_Jid"]]
        ps["act_idxq"] = [J.idx_q for J in ps["act_J"]]
        ps["act_idxv"] = [J.idx_v for J in ps["act_J"]]

        # Initialize
        traj.initialize()

        # Generate optimal trajectory (single segment keeps the data volume
        # small — this is a validation pipeline, not a production run).
        optimal_trajectory = traj.solve(stack_reps=1)

        if optimal_trajectory is not None:
            # Display results
            print("Optimal trajectory generation completed successfully!")
            # Plot and save results. NOTE: figaroh's save_results() accesses
            # self.CB.identif_config but CB is a CubicSpline instance which
            # has no such attribute -> AttributeError. The raw trajectory
            # npz below is what the MuJoCo data generator actually needs.
            traj.plot_results()
            # Concatenate segments into one continuous trajectory for the
            # MuJoCo data generator (segments share end/start waypoints, so
            # de-duplicate by time keeping the latest segment).
            T_F, P_F, V_F, A_F = (
                optimal_trajectory["T_F"],
                optimal_trajectory["P_F"],
                optimal_trajectory["V_F"],
                optimal_trajectory["A_F"],
            )
            t_all = np.concatenate([t.ravel() for t in T_F])
            q_all = np.vstack(P_F)
            dq_all = np.vstack(V_F)
            ddq_all = np.vstack(A_F)
            order = np.argsort(t_all)
            t_all, q_all, dq_all, ddq_all = (
                t_all[order],
                q_all[order],
                dq_all[order],
                ddq_all[order],
            )
            keep = np.concatenate(([True], np.diff(t_all) > 1e-9))
            t_all, q_all, dq_all, ddq_all = (
                t_all[keep],
                q_all[keep],
                dq_all[keep],
                ddq_all[keep],
            )
            out_dir = Path("data/trajectories")
            out_dir.mkdir(parents=True, exist_ok=True)
            np.savez(
                out_dir / "right_arm_optimal_trajectory.npz",
                t=t_all,
                q=q_all,
                dq=dq_all,
                ddq=ddq_all,
                joint_names=active_joints,
            )
            print(
                f"Trajectory saved to {out_dir / 'right_arm_optimal_trajectory.npz'} "
                f"({len(t_all)} samples)"
            )
        else:
            print(
                "Failed to generate optimal trajectory. Check constraints and parameters."
            )
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
