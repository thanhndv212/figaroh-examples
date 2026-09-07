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
from figaroh.tools.run_archive import archive_run, compute_run_dir


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
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Check the identification against quality thresholds "
            "(condition number, validation correlation/improvement), "
            "write the verdict to the run directory, and exit(1) if it fails. "
            "Use --no-verify to skip."
        ),
    )
    parser.add_argument(
        "--html-report",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Export a self-contained HTML diagnostic report to the run directory. "
            "Use --no-html-report to skip."
        ),
    )
    parser.add_argument(
        "--wls",
        action=argparse.BooleanOptionalAction,
        default=None,
        help=(
            "Refine the OLS base-parameter estimate with iteratively-"
            "weighted least squares (Gautier, 1997). Overrides the "
            "identification.problem.wls config value (True by default "
            "for TX40); use --no-wls for the plain OLS estimate."
        ),
    )
    parser.add_argument(
        "--asset-id",
        type=str,
        default=None,
        help=(
            "Physical unit identifier for the report/verdict provenance "
            "record (e.g. 'TX40-014'). Overrides robot.instance.asset_id "
            "in the config; omit both to render as an unspecified unit."
        ),
    )
    parser.add_argument(
        "--operator",
        type=str,
        default=None,
        help="Operator name for the provenance record. Overrides "
        "robot.instance.operator in the config.",
    )
    parser.add_argument(
        "--archive",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Archive this run to results/runs/<asset>/identification/"
            "<timestamp>/ (provenance, config snapshot, parameters, and "
            "the HTML report / JSON verdict if generated) and append a "
            "summary line to results/runs/index.jsonl. Never overwrites "
            "a prior run. Use --no-archive to skip."
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

        # CLI flag overrides the config value; falls back to False if
        # neither --wls/--no-wls was passed nor identification.problem.wls
        # is set in the YAML config.
        wls_enabled = (
            args.wls if args.wls is not None
            else tx40_iden.identif_config.get("wls", False)
        )

        # CLI --asset-id/--operator override the config's robot.instance
        # block (if any) for this run's provenance record.
        if args.asset_id or args.operator:
            instance = dict(tx40_iden.identif_config.get("instance") or {})
            if args.asset_id:
                instance["asset_id"] = args.asset_id
            if args.operator:
                instance["operator"] = args.operator
            tx40_iden.identif_config["instance"] = instance

        # Initialize identification process with data loading and processing
        tx40_iden.initialize()

        # Solve identification with TX40-specific features
        tx40_iden.solve(
            decimate=True,  # Apply TX40-specific decimation
            plotting=True,  # Generate identification plots
            save_results=False,  # Save parameters to CSV files
            wls=wls_enabled,  # Weighted least squares refinement
            html_report=False,  # Never write to library's default path
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

        # V&V report suite: compute path once, write directly
        run_dir = None
        if args.html_report or args.verify or args.archive:
            run_dir = compute_run_dir(tx40_iden)

        if args.html_report:
            tx40_iden.export_html_report(output_path=str(run_dir / "report.html"))

        verify_failed = False
        if args.verify:
            print("\n" + "=" * 60)
            print("VERIFICATION")
            print("=" * 60)
            verdict = tx40_iden.verify()
            tx40_iden.export_verification_report(
                output_path=str(run_dir / "verdict.json")
            )
            for check in verdict.checks:
                status = "PASS" if check.passed else "FAIL"
                print(
                    f"  [{status}] {check.name}: {check.value:.4g} "
                    f"({check.comparison} {check.threshold:.4g})"
                )
            verify_failed = not verdict.passed

        if args.archive:
            archive_run(tx40_iden, run_dir)

        if run_dir:
            print(f"\nResults written to: {run_dir}")

        if verify_failed:
            print("\nVerification FAILED.")
            sys.exit(1)
        elif args.verify:
            print("\nVerification PASSED.")

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
