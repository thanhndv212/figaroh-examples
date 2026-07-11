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
from pathlib import Path

# Add project root to path for imports (prefer `pip install -e .` instead)
project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

import yaml  # noqa: E402
from examples.tiago.utils.tiago_tools import TiagoIdentification  # noqa: E402
from figaroh.tools.robot import load_robot  # noqa: E402


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="TIAGo dynamic parameter identification"
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
    parser.add_argument(
        "--verify",
        action="store_true",
        help=(
            "Check the identification against quality thresholds "
            "(condition number, validation correlation/improvement), "
            "write the verdict to results/identification_verification.json, "
            "and exit(1) if it fails."
        ),
    )
    parser.add_argument(
        "--html-report",
        action="store_true",
        help=(
            "Export a self-contained HTML diagnostic report "
            "(results/identification_report.html)."
        ),
    )
    return parser.parse_args()


def main() -> TiagoIdentification | None:
    """Main function for Tiago dynamic parameter identification."""
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
        # Load robot model
        tiago = load_robot(
            str(urdf_path),
            load_by_urdf=True,
            robot_pkg="tiago_description",
        )

        # Create identification object
        tiago_iden = TiagoIdentification(tiago, str(config_path))

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

        # Read active joints from config (instead of hardcoding)
        with open(config_path) as f:
            cfg = yaml.safe_load(f)
        active_joints = cfg["robot"]["properties"]["joints"]["active_joints"]
        ps["active_joints"] = active_joints

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
            html_report=args.html_report,
        )

        # Print results summary
        print("\n" + "=" * 60)
        print("TIAGo DYNAMIC PARAMETER IDENTIFICATION RESULTS")
        print("=" * 60)

        print(
            f"Number of base parameters identified: " f"{len(tiago_iden.params_base)}"
        )
        print(f"Correlation coefficient: {tiago_iden.correlation:.4f}")

        if hasattr(tiago_iden, "result"):
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
            print(f"{i + 1:2d}. {param_name}: {tiago_iden.phi_base[i]:10.6f}")

        if args.verify:
            print("\n" + "=" * 60)
            print("VERIFICATION")
            print("=" * 60)
            verdict = tiago_iden.verify()
            report_path = tiago_iden.export_verification_report()
            print(f"Verdict written to: {report_path}")
            for check in verdict.checks:
                status = "PASS" if check.passed else "FAIL"
                print(
                    f"  [{status}] {check.name}: {check.value:.4g} "
                    f"({check.comparison} {check.threshold:.4g})"
                )
            if verdict.passed:
                print("\nVerification PASSED.")
            else:
                print("\nVerification FAILED.")
                sys.exit(1)

        print("\nIdentification completed successfully!")
        return tiago_iden
    except Exception as e:
        print(
            f"Error during identification: {e}",
            file=sys.stderr,
        )
        sys.exit(1)


if __name__ == "__main__":
    main()
