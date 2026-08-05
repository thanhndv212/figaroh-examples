#!/usr/bin/env python3
"""
Tiago Pro geometric calibration using Figaroh.

Reads a CSV of (q, mocap EE pose) samples collected on the real robot,
runs Figaroh's Levenberg-Marquardt calibration, and writes the identified
kinematic parameters (base frame pose + per-joint DH offsets + marker
position) to a results YAML.

Migrated from the standalone `figaroh_tiagoPro` repo. This example keeps
the original self-contained calibration class (it drives
`figaroh.calibration.calibration_tools` directly rather than through
`BaseCalibration`) since that is what was validated against real hardware
data — see `data/calibration_results_20260702_0756.yaml` for the
reference run (RMSE 6.46 mm, MAE 4.94 mm, 94 samples). The class and its
supporting functions live in `utils/tiago_pro_tools.py`, following the
same layout as the other examples (`tiago`, `talos`, `ur10`).

Usage:
    python3 calibration.py --data data/calibration_samples_20260702_0756.csv
    python3 calibration.py \\
        --urdf urdf/tiago_pro.urdf \\
        --data data/calibration_samples_20260702_0756.csv \\
        --output data/calibration_results.yaml
"""

import argparse
import sys
from datetime import datetime, timezone
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
    _prepare_report,
)
from figaroh.tools.report import generate_calibration_report  # noqa: E402
from figaroh.tools.run_archive import archive_run  # noqa: E402

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
    robot = _Robot(pin.buildModelFromUrdf(args.urdf))

    calib = TiagoProCalibration(robot, args.config, args.data)
    calib.load_and_check_data()

    run_started = datetime.now(timezone.utc).isoformat()
    calib.solve()
    run_finished = datetime.now(timezone.utc).isoformat()

    write_calibration_results(calib, args.output)

    if args.html_report or args.archive:
        run_dir = _prepare_report(
            calib,
            config_path=args.config,
            urdf_path=args.urdf,
            asset_id=args.asset_id,
            operator=args.operator,
            run_started=run_started,
            run_finished=run_finished,
        )

        if args.html_report:
            generate_calibration_report(calib, output_path=str(run_dir / "report.html"))
            print(f"HTML quality report written to {run_dir / 'report.html'}")

        if args.archive:
            archive_run(calib, run_dir)
            print(f"Run archived to {run_dir}")


if __name__ == "__main__":
    main()
