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

import numpy as np

# Add project root to path for imports (prefer `pip install -e .` instead)
project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from examples.kitt_1_5.utils.kitt_1_5_tools import KITT1_5Identification
from figaroh.identification.identification_tools import base_param_from_standard
from figaroh.tools.robot import load_robot
from figaroh.tools.run_archive import archive_run, compute_run_dir


def compare_base_params_to_urdf(identif, run_dir: Path | None = None) -> dict:
    """Project URDF standard inertias onto the same base-parameter basis
    and compare against the identified ``phi_base``.

    Base parameters are not uniquely invertible to a full URDF inertia
    table, but the forward map (URDF → φ_b^nom via the QR regrouping
    expressions in ``params_base``) is unique — so this is the right
    apples-to-apples check for simulation-from-URDF experiments.
    """
    phi_id = np.asarray(identif.phi_base, dtype=float)
    phi_nom = np.asarray(
        base_param_from_standard(identif.standard_parameter, identif.params_base),
        dtype=float,
    )
    if phi_id.shape != phi_nom.shape:
        raise ValueError(
            f"phi_base length {phi_id.shape} != URDF-projected "
            f"phi_b^nom length {phi_nom.shape}"
        )

    abs_err = np.abs(phi_id - phi_nom)
    # Relative error vs |φ_nom|; fall back to abs when nominal is ~0
    denom = np.maximum(np.abs(phi_nom), 1e-12)
    rel_err = abs_err / denom

    print("\n" + "=" * 60)
    print("BASE PARAMETERS: identified vs URDF-projected (φ_b^nom)")
    print("=" * 60)
    print(
        f"{'#':>3}  {'φ_id':>12}  {'φ_urdf':>12}  {'|Δ|':>10}  "
        f"{'|Δ|/|φ| %':>10}  expression"
    )
    for i, name in enumerate(identif.params_base):
        # Truncate long regrouping expressions for the console table
        short = name if len(name) <= 60 else name[:57] + "..."
        print(
            f"{i + 1:3d}  {phi_id[i]:12.6f}  {phi_nom[i]:12.6f}  "
            f"{abs_err[i]:10.3e}  {100 * rel_err[i]:10.2f}  {short}"
        )

    summary = {
        "n_base": int(len(phi_id)),
        "rmse": float(np.sqrt(np.mean(abs_err**2))),
        "max_abs": float(np.max(abs_err)),
        "median_rel_pct": float(100 * np.median(rel_err)),
        "max_rel_pct": float(100 * np.max(rel_err)),
        "corr": float(np.corrcoef(phi_id, phi_nom)[0, 1]),
    }
    print(
        f"\n  RMSE(φ_id, φ_urdf)={summary['rmse']:.6g}  "
        f"max|Δ|={summary['max_abs']:.6g}  "
        f"median|Δ|/|φ|={summary['median_rel_pct']:.2f}%  "
        f"max|Δ|/|φ|={summary['max_rel_pct']:.2f}%  "
        f"corr={summary['corr']:.6f}"
    )

    if run_dir is not None:
        out = run_dir / "base_params_vs_urdf.csv"
        run_dir.mkdir(parents=True, exist_ok=True)
        with open(out, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "index",
                    "expression",
                    "phi_identified",
                    "phi_urdf",
                    "abs_error",
                    "rel_error",
                ]
            )
            for i, name in enumerate(identif.params_base):
                writer.writerow(
                    [
                        i + 1,
                        name,
                        f"{phi_id[i]:.10g}",
                        f"{phi_nom[i]:.10g}",
                        f"{abs_err[i]:.10g}",
                        f"{rel_err[i]:.10g}",
                    ]
                )
        print(f"  Wrote {out}")

    # Stash on the object for anything that reads identif after solve
    identif.phi_base_urdf = phi_nom
    identif.base_param_comparison = summary
    return summary


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="KITT1_5V3 right-arm dynamic parameter identification"
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
            "record (e.g. 'KITT1_5V3-001'). Overrides robot.instance.asset_id "
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
        # Load KITT1_5V3 right-arm robot model (mesh files live next to the
        # sub-model URDF, so package_dirs points at the local urdf/ folder)
        robot = load_robot(
            args.urdf,
            package_dirs="urdf",
            load_by_urdf=True,
        )

        # Create identification object
        identif = KITT1_5Identification(
            robot=robot,
            config_file=args.config,
        )

        # active_joints is already resolved (extends-aware) by load_param()
        # into identif_config — read it from there rather than re-parsing
        # args.config raw, which would silently drop anything inherited
        # via extends: (e.g. a per-asset overlay config).
        active_joints = identif.identif_config.get("active_joints", [])

        ps = identif.identif_config
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
        ps["act_Jid"] = [identif.model.getJointId(i) for i in ps["active_joints"]]
        ps["act_J"] = [identif.model.joints[jid] for jid in ps["act_Jid"]]
        ps["act_idxq"] = [J.idx_q for J in ps["act_J"]]
        ps["act_idxv"] = [J.idx_v for J in ps["act_J"]]

        # Initialize with data processing
        identif.initialize()

        # Perform identification using the base class solve method
        identif.solve(
            decimate=False,
            plotting=True,
            save_results=False,
            html_report=False,  # Never write to library's default path
            wls=wls_enabled,
        )

        # Display results
        # Print results summary
        print("\n" + "=" * 60)
        print("KITT1_5V3 RIGHT-ARM DYNAMIC PARAMETER IDENTIFICATION RESULTS")
        print("=" * 60)

        print(
            f"Number of base parameters identified: " f"{len(identif.params_base)}"
        )
        print(f"Correlation coefficient: {identif.correlation:.4f}")

        if hasattr(identif, "result"):
            for key, value in identif.result.items():
                if isinstance(value, (int, float)):
                    if isinstance(value, float):
                        print(f"{key}: {value:.6f}")
                    else:
                        print(f"{key}: {value}")
                else:
                    print(f"{key}: {type(value).__name__} of length {len(value)}")

        print("\nBase parameters:")
        for i, param_name in enumerate(identif.params_base):
            print(f"{i + 1:2d}. {param_name}: {identif.phi_base[i]:10.6f}")

        # V&V report suite: compute path once, write directly
        run_dir = None
        if args.html_report or args.verify or args.archive:
            run_dir = compute_run_dir(identif)

        if args.html_report:
            identif.export_html_report(output_path=str(run_dir / "report.html"))

        verify_failed = False
        if args.verify:
            print("\n" + "=" * 60)
            print("VERIFICATION")
            print("=" * 60)
            verdict = identif.verify(
                # The default thresholds are tuned for physical robots whose
                # nominal (URDF) parameters are far from the true ones.  Here
                # the data is generated from the same model, so the nominal
                # parameters already reproduce the torques almost exactly and
                # "improvement over nominal" is naturally a few percent even
                # though the identified set is excellent (correlation ~0.9999,
                # condition ~40).  Relax only that single threshold and keep
                # the other checks at the library defaults.
                thresholds={
                    "validation_correlation": {
                        "threshold": 0.9,
                        "comparison": "min",
                    },
                    "condition_number": {"threshold": 1000.0, "comparison": "max"},
                    "validation_improvement_pct": {
                        "threshold": 1.0,
                        "comparison": "min",
                    },
                }
            )
            identif.export_verification_report(
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
            archive_run(identif, run_dir)

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
