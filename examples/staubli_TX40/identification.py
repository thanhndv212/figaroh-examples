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

"""
Refactored Staubli TX40 dynamic parameter identification using base classes.
This demonstrates the clean separation between robot-specific and general
identification functionality.
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

from examples.staubli_tx40.utils.staubli_tx40_tools import TX40Identification
from figaroh.tools.robot import load_robot


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Staubli TX40 dynamic parameter identification"
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
    parser.add_argument(
        "--wls",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Refine the OLS base-parameter estimate with iteratively-"
            "weighted least squares (Gautier, 1997). On by default for "
            "TX40; use --no-wls for the plain OLS estimate."
        ),
    )
    return parser.parse_args()


def main(args: argparse.Namespace) -> None:
    """Main function for TX40 dynamic parameter identification.

    Args:
        args: Parsed command-line arguments
    """
    # Validate input files exist
    urdf_path = Path(args.urdf)
    if not urdf_path.exists():
        print(f"Error: URDF file not found: {urdf_path}", file=sys.stderr)
        sys.exit(1)

    config_path = Path(args.config)
    if not config_path.exists():
        print(f"Error: Config file not found: {config_path}", file=sys.stderr)
        sys.exit(1)

    try:
        # Load robot model
        robot = load_robot(str(urdf_path), package_dirs="../../models")

        # Create TX40 identification object
        tx40_iden = TX40Identification(robot, str(config_path))

        # Initialize identification process with data loading and processing
        tx40_iden.initialize()

        # Solve identification with TX40-specific features
        tx40_iden.solve(
            decimate=True,  # Apply TX40-specific decimation
            plotting=True,  # Generate identification plots
            save_results=False,  # Save parameters to CSV files
            wls=args.wls,  # Weighted least squares refinement
            html_report=args.html_report,
        )

        # Print results summary
        print("\n" + "=" * 60)
        print("TX40 DYNAMIC PARAMETER IDENTIFICATION RESULTS")
        print("=" * 60)

        print(f"Number of base parameters identified: " f"{len(tx40_iden.params_base)}")
        print(f"Correlation coefficient: {tx40_iden.correlation:.4f}")

        if hasattr(tx40_iden, "result"):
            for key, value in tx40_iden.result.items():
                if isinstance(value, (int, float)):
                    if isinstance(value, float):
                        print(f"{key}: {value:.6f}")
                    else:
                        print(f"{key}: {value}")
                else:
                    print(f"{key}: {type(value).__name__} of length {len(value)}")

        print("\nBase parameters:")
        for i, param_name in enumerate(tx40_iden.params_base):
            print(f"{i + 1:2d}. {param_name}: {tx40_iden.phi_base[i]:10.6f}")

        if args.verify:
            print("\n" + "=" * 60)
            print("VERIFICATION")
            print("=" * 60)
            verdict = tx40_iden.verify()
            report_path = tx40_iden.export_verification_report()
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

    except FileNotFoundError as e:
        print(f"Error: File not found: {e}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"Error during identification: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    # Parse arguments first
    args_ns = parse_args()

    # Configure logging after argument parsing
    logging.basicConfig(
        level=logging.INFO if args_ns.verbose else logging.WARNING,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    # Run the identification
    main(args_ns)
