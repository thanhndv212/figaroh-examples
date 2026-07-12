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

from examples.ur10.utils.ur10_tools import UR10Identification
from figaroh.tools.robot import load_robot
from figaroh.tools.run_archive import archive_run, compute_run_dir


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
            "identification.problem.wls config value; falls back to "
            "False if neither is set."
        ),
    )
    parser.add_argument(
        "--asset-id",
        type=str,
        default=None,
        help=(
            "Physical unit identifier for the report/verdict provenance "
            "record (e.g. 'UR10-007'). Overrides robot.instance.asset_id "
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

        # active_joints is already resolved (extends-aware) by load_param()
        # into identif_config — read it from there rather than re-parsing
        # args.config raw, which would silently drop anything inherited
        # via extends: (e.g. a per-asset overlay config).
        active_joints = ur10_identif.identif_config.get("active_joints", [])

        ps = ur10_identif.identif_config
        # CLI flag overrides the config value; falls back to False if
        # neither --wls/--no-wls was passed nor identification.problem.wls
        # is set in the YAML config.
        wls_enabled = args.wls if args.wls is not None else ps.get("wls", False)

        # CLI --asset-id/--operator override the config's robot.instance
        # block (if any) for this run's provenance record.
        if args.asset_id or args.operator:
            instance = dict(ps.get("instance") or {})
            if args.asset_id:
                instance["asset_id"] = args.asset_id
            if args.operator:
                instance["operator"] = args.operator
            ps["instance"] = instance

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
            html_report=False,  # Never write to library's default path
            wls=wls_enabled,
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

        # V&V report suite: compute path once, write directly
        run_dir = None
        if args.html_report or args.verify or args.archive:
            run_dir = compute_run_dir(ur10_identif)

        if args.html_report:
            ur10_identif.export_html_report(output_path=str(run_dir / "report.html"))

        verify_failed = False
        if args.verify:
            print("\n" + "=" * 60)
            print("VERIFICATION")
            print("=" * 60)
            verdict = ur10_identif.verify()
            ur10_identif.export_verification_report(
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
            archive_run(ur10_identif, run_dir)

        if run_dir:
            print(f"\nResults written to: {run_dir}")

        if verify_failed:
            print("\nVerification FAILED.")
            sys.exit(1)
        elif args.verify:
            print("\nVerification PASSED.")

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
