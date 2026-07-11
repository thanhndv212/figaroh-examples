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
from pathlib import Path

# Add project root to path for imports (prefer `pip install -e .` instead)
project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from examples.ur10.utils.ur10_tools import UR10Identification
from figaroh.tools.robot import load_robot


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="UR10 dynamic parameter identification"
    )
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
        "--verbose", "-v", action="store_true", help="Enable verbose (INFO) logging"
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
    parser.add_argument(
        "--wls",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "Refine the OLS base-parameter estimate with iteratively-"
            "weighted least squares (Gautier, 1997). Off by default."
        ),
    )
    return parser.parse_args()


def main(args: argparse.Namespace) -> None:
    """Main function for UR10 dynamic parameter identification."""
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

        # Create identification object
        ur10_identif = UR10Identification(
            robot=ur10,
            config_file=args.config,
        )

        # Load active joints from unified config (eliminates DRY with config)
        with open(args.config) as f:
            cfg = yaml.safe_load(f)
        active_joints = cfg["robot"]["properties"]["joints"]["active_joints"]

        ps = ur10_identif.identif_config
        ps["active_joints"] = active_joints
        # Joint parameters
        ps["act_Jid"] = [ur10_identif.model.getJointId(i) for i in ps["active_joints"]]
        ps["act_J"] = [ur10_identif.model.joints[jid] for jid in ps["act_Jid"]]
        ps["act_idxq"] = [J.idx_q for J in ps["act_J"]]
        ps["act_idxv"] = [J.idx_v for J in ps["act_J"]]

        # Initialize with data processing
        ur10_identif.initialize()

        # Perform identification using the base class solve method
        ur10_identif.solve(
            decimate=False,
            plotting=True,
            save_results=False,
            html_report=args.html_report,
            wls=args.wls,
        )

        # Display results
        # Print results summary
        print("\n" + "=" * 60)
        print("UR10 DYNAMIC PARAMETER IDENTIFICATION RESULTS")
        print("=" * 60)

        print(
            f"Number of base parameters identified: " f"{len(ur10_identif.params_base)}"
        )
        print(f"Correlation coefficient: {ur10_identif.correlation:.4f}")

        if hasattr(ur10_identif, "result"):
            for key, value in ur10_identif.result.items():
                if isinstance(value, (int, float)):
                    if isinstance(value, float):
                        print(f"{key}: {value:.6f}")
                    else:
                        print(f"{key}: {value}")
                else:
                    print(f"{key}: {type(value).__name__} of length {len(value)}")

        print("\nBase parameters:")
        for i, param_name in enumerate(ur10_identif.params_base):
            print(f"{i + 1:2d}. {param_name}: {ur10_identif.phi_base[i]:10.6f}")

        if args.verify:
            print("\n" + "=" * 60)
            print("VERIFICATION")
            print("=" * 60)
            verdict = ur10_identif.verify()
            report_path = ur10_identif.export_verification_report()
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
