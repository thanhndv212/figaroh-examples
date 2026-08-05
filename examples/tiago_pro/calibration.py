#!/usr/bin/env python3
"""
Tiago Pro geometric calibration using Figaroh.

Reads a CSV of (q, mocap EE pose) samples collected on the real robot,
runs Figaroh's Levenberg-Marquardt calibration, and writes the identified
kinematic parameters (base frame pose + per-joint DH offsets + marker
position) to a results YAML.

Migrated from the standalone `figaroh_tiagoPro` repo. TiagoProCalibration
subclasses `figaroh.calibration.base_calibration.BaseCalibration`, the same
pattern the `tiago` example's TiagoCalibration uses — see
`utils/tiago_pro_tools.py` for the overrides genuinely specific to this
robot/dataset. Numerically re-verified after migration: RMSE 6.46 mm / MAE
4.95 mm, 94 samples (vs. RMSE 6.46 mm / MAE 4.94 mm pre-migration — see
`data/calibration_results_20260702_0756.yaml`, the frozen pre-migration
reference).

Usage:
    python3 calibration.py --data data/calibration_samples_20260702_0756.csv
    python3 calibration.py \\
        --urdf urdf/tiago_pro.urdf \\
        --data data/calibration_samples_20260702_0756.csv \\
        --output data/calibration_results.yaml
"""

import argparse
import sys
from pathlib import Path

import pinocchio as pin

# Add project root to path for imports (prefer `pip install -e .` instead)
project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from examples.tiago_pro.utils.tiago_pro_tools import (  # noqa: E402
    _Robot,
    TiagoProCalibration,
    write_calibration_results,
)
from figaroh.tools.run_archive import archive_run, compute_run_dir  # noqa: E402

_HERE = Path(__file__).parent
_URDF_DEFAULT = _HERE / "urdf" / "tiago_pro.urdf"
_CONFIG_DEFAULT = _HERE / "config" / "tiago_pro_calibration_config.yaml"
_DATA_DEFAULT = _HERE / "data" / "calibration_samples_20260702_0756.csv"
_OUT_DEFAULT = _HERE / "data" / "calibration_results.yaml"


# ── Main ────────────────────────────────────────────────────────────────────


def main():
    parser = argparse.ArgumentParser(
        description="Run Figaroh geometric calibration for Tiago Pro right arm."
    )
    parser.add_argument(
        "--urdf", default=str(_URDF_DEFAULT), help="Path to the Tiago Pro URDF"
    )
    parser.add_argument("--data", default=str(_DATA_DEFAULT))
    parser.add_argument("--config", default=str(_CONFIG_DEFAULT))
    parser.add_argument("--output", default=str(_OUT_DEFAULT))
    parser.add_argument(
        "--html-report",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Export a self-contained HTML diagnostic report to the run "
            "directory after calibration. Use --no-html-report to skip."
        ),
    )
    parser.add_argument(
        "--archive",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Archive this run to results/runs/<asset>/calibration/"
            "<timestamp>/ (provenance, config snapshot, parameters, and "
            "the HTML report if generated) and append a summary line to "
            "results/runs/index.jsonl. Never overwrites a prior run. "
            "Use --no-archive to skip."
        ),
    )
    parser.add_argument(
        "--asset-id",
        type=str,
        default=None,
        help=(
            "Physical unit identifier for the report/verdict provenance "
            "record (e.g. 'TIAGO-PRO-1'). Omit to render as an "
            "unspecified unit."
        ),
    )
    parser.add_argument(
        "--operator",
        type=str,
        default=None,
        help="Operator name for the provenance record.",
    )
    args = parser.parse_args()

    print(f"Loading robot from {args.urdf} ...")
    robot = _Robot(pin.buildModelFromUrdf(args.urdf), urdf_path=args.urdf)

    calib = TiagoProCalibration(robot, args.config, del_list=[])
    calib.calib_config["known_baseframe"] = False  # co-estimate base-marker->base transform
    calib.calib_config["known_tipframe"] = False  # estimate marker pos rel. to gripper
    calib.calib_config["data_file"] = args.data
    calib._data_path = str(Path(args.data).resolve())

    if args.asset_id or args.operator:
        instance = dict(calib.calib_config.get("instance") or {})
        if args.asset_id:
            instance["asset_id"] = args.asset_id
        if args.operator:
            instance["operator"] = args.operator
        calib.calib_config["instance"] = instance

    calib.initialize()
    calib.solve(
        max_iterations=10,
        enable_logging=True,
        plotting=False,
        html_report=False,
    )

    write_calibration_results(calib, args.output)

    run_dir = None
    if args.html_report or args.archive:
        run_dir = compute_run_dir(calib)

    if args.html_report:
        calib.export_html_report(output_path=str(run_dir / "report.html"))
        print(f"HTML quality report written to {run_dir / 'report.html'}")

    if args.archive:
        archive_run(calib, run_dir)
        print(f"Run archived to {run_dir}")


if __name__ == "__main__":
    main()
