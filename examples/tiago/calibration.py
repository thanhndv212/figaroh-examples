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
TIAGo calibration, URDF update, and validation — all-in-one entry-point.

Modes::

    # Full pipeline: calibrate → plot → save → export → viser viz
    python calibration.py

    # Calibrate only (save results with timestamp, skip export)
    python calibration.py --calibrate-only

    # Load saved results → export URDF → verify FK
    python calibration.py --update-model

    # Visually validate a previously exported modified URDF
    python calibration.py --viz-validation
    python calibration.py --viz-validation --model path/to/modified.urdf

    # Interactive step selection
    python calibration.py --interactive

Run ``python calibration.py --help`` for all available flags.
"""

from __future__ import annotations

import argparse
import datetime
import logging
import os
import re
import sys
import time
from pathlib import Path

import numpy as np

# Add project root to path for imports (prefer `pip install -e .` instead)
project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from examples.tiago.utils.tiago_tools import TiagoCalibration  # noqa: E402
from figaroh.tools.robot import load_robot  # noqa: E402
from figaroh.tools.run_archive import archive_run, compute_run_dir  # noqa: E402
from figaroh.tools.urdf_exporter import (  # noqa: E402
    export_urdf,
    frame_settings_doc,
)
from figaroh.tools.export_validation import URDFComparison  # noqa: E402

logger = logging.getLogger(__name__)

DATA_DIR = "data/calibration"
URDF_STEM = "tiago_48_schunk"  # stem for discovering modified URDFs


# ── Timestamp and file discovery helpers ────────────────────────────


def _timestamp_str() -> str:
    """Return a compact timestamp string for filenames (YYYYMMDD_HHMMSS)."""
    return datetime.datetime.now().strftime("%Y%m%d_%H%M%S")


def _discover_npz_files(data_dir: str = DATA_DIR) -> list[Path]:
    """Return sorted list of calibration .npz files, newest last."""
    files = sorted(Path(data_dir).glob("calibration_results_*.npz"))
    if not files:
        # Legacy: timestamped files in flat data/
        files = sorted(Path("data").glob("calibration_results_*.npz"))
    if not files:
        # Legacy: non-timestamped in new or old dir
        for p in [Path(data_dir) / "calibration_results.npz",
                  Path("data") / "calibration_results.npz"]:
            if p.exists():
                files = [p]
                break
    return files


def _discover_modified_urdf_files(stem: str = URDF_STEM) -> list[Path]:
    """Return sorted list of modified URDF files, newest last."""
    files = sorted(Path("urdf").glob(f"{stem}_modified_*.urdf"))
    if not files:
        legacy = Path(f"urdf/{stem}_modified.urdf")
        if legacy.exists():
            files = [legacy]
    return files


def _select_npz(data_dir: str = DATA_DIR) -> str:
    """Select a calibration .npz file. Interactive if TTY, else latest."""
    files = _discover_npz_files(data_dir)
    if not files:
        print(
            f"Error: No calibration results found in {data_dir}/ or data/.",
            file=sys.stderr,
        )
        print(
            "  Run `python calibration.py --calibrate-only` first.",
            file=sys.stderr,
        )
        sys.exit(1)

    if not sys.stdin.isatty():
        return str(files[-1])  # non-interactive → latest

    print("\nAvailable calibration results:")
    print(f"  [0]  {files[-1].name}  (latest)")
    for i, f in enumerate(files):
        if f != files[-1]:
            info = ""
            try:
                d = np.load(f)
                info = f"  ({len(d.get('param_names', []))} params)"
            except Exception:
                pass
            print(f"  [{i + 1}]  {f.name}{info}")
    prompt = f"Select result [0-{len(files)}], default=0: "
    choice = input(prompt).strip()
    idx = int(choice) if choice else 0
    return str(files[-1] if idx == 0 else files[idx - 1])


def _select_modified_urdf(stem: str = URDF_STEM) -> str:
    """Select a modified URDF. Interactive if TTY, else latest."""
    files = _discover_modified_urdf_files(stem)
    if not files:
        print(
            f"Error: No modified URDFs found matching 'urdf/{stem}_modified_*'.",
            file=sys.stderr,
        )
        print(
            "  Run `python calibration.py` or `--update-model` first.",
            file=sys.stderr,
        )
        sys.exit(1)

    if not sys.stdin.isatty():
        return str(files[-1])

    print("\nAvailable modified URDFs:")
    print(f"  [0]  {files[-1].name}  (latest)")
    for i, f in enumerate(files):
        if f != files[-1]:
            print(f"  [{i + 1}]  {f.name}")
    prompt = f"Select model [0-{len(files)}], default=0: "
    choice = input(prompt).strip()
    idx = int(choice) if choice else 0
    return str(files[-1] if idx == 0 else files[idx - 1])


# ── Step selection (--interactive) ──────────────────────────────────


def _select_steps() -> list[str]:
    """Prompt user which steps to include. Returns list of step keys."""
    steps_info = [
        ("calibrate", "Calibration (required if no saved results exist)"),
        ("export", "Export URDF"),
        ("verify", "URDF Consistency"),
        ("viz", "Viser Visualization"),
    ]
    print("\nSelect steps to include (comma-separated numbers, 'all', or 'done'):")
    for i, (key, desc) in enumerate(steps_info, 1):
        print(f"  [{i}] {desc}")
    print()
    choice = input("Steps: ").strip().lower()

    if choice == "all":
        return [s[0] for s in steps_info]
    if not choice:
        return [s[0] for s in steps_info]  # default = all

    selected = []
    try:
        for token in re.split(r"[,\s]+", choice):
            token = token.strip()
            if not token:
                continue
            idx = int(token) - 1
            if 0 <= idx < len(steps_info):
                selected.append(steps_info[idx][0])
    except ValueError:
        print(f"Invalid input: {choice}. Running all steps.")
        return [s[0] for s in steps_info]
    return selected


# ── Parsing ─────────────────────────────────────────────────────────


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description=(
            "TIAGo calibration, URDF update, and validation — all-in-one entry-point. "
            "Default mode runs the full pipeline: calibrate → plot → save → export → "
            "visualize."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Modes:\n"
            "  (default)         Full pipeline: calibrate, plot, save, export, viz\n"
            "  --calibrate-only  Calibrate + plot + save (no export, no viz)\n"
            "  --update-model    Load saved results → export URDF → verify FK\n"
            "  --viz-validation  Visually validate a previously exported URDF\n"
            "  --interactive     Select which steps to run interactively\n"
            "\n"
            "All saved files are timestamped to avoid overwriting.\n"
            "Use --model <path> with --viz-validation to skip file selection."
        ),
    )
    parser.add_argument(
        "--config",
        type=str,
        default="config/tiago_unified_config.yaml",
        help="Path to unified config YAML file (default: %(default)s)",
    )
    parser.add_argument(
        "--urdf",
        type=str,
        default="urdf/tiago_48_schunk.urdf",
        help="Path to robot URDF file (nominal model) (default: %(default)s)",
    )
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        help=(
            "Output path for modified URDF in export mode. "
            "Default: urdf/<stem>_modified_<timestamp>.urdf"
        ),
    )
    parser.add_argument(
        "--update-model",
        action="store_true",
        help=(
            "Load saved calibration results → export URDF → verify FK. "
            "Prompts to select which .npz file to use (interactive) or "
            "picks the latest (non-TTY)."
        ),
    )
    parser.add_argument(
        "--calibrate-only",
        action="store_true",
        help=(
            "Run calibration with plotting, save results (timestamped .npz), "
            "then exit with a reminder about --update-model."
        ),
    )
    parser.add_argument(
        "--viz-validation",
        action="store_true",
        help=(
            "Visually validate a previously exported modified URDF via "
            "viser.  Use --model to specify the file path; otherwise an "
            "interactive menu lists available modified URDFs."
        ),
    )
    parser.add_argument(
        "--model",
        type=str,
        default=None,
        help=(
            "Path to a modified URDF for --viz-validation.  Overrides "
            "interactive file selection."
        ),
    )
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="Interactively select which steps to include (calibrate, export, "
        "verify, viz).",
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Suppress matplotlib plots during calibration (useful in CI).",
    )
    parser.add_argument(
        "--validation-data",
        type=str,
        default=None,
        help=(
            "Path to a separate validation measurement CSV. "
            "Overrides validation_data_file in config. "
            "Used by the quality report to compute FK improvement "
            "against ground truth."
        ),
    )
    parser.add_argument(
        "--html-report",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Export a self-contained HTML diagnostic report to the run directory "
            "after calibration. Use --no-html-report to skip."
        ),
    )
    parser.add_argument(
        "--asset-id",
        type=str,
        default=None,
        help=(
            "Physical unit identifier for the report/verdict provenance "
            "record (e.g. 'TIAGO-42'). Overrides robot.instance.asset_id "
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
            "Archive this run to results/runs/<asset>/calibration/"
            "<timestamp>/ (provenance, config snapshot, parameters, and "
            "the HTML report if generated) and append a summary line to "
            "results/runs/index.jsonl. Never overwrites a prior run. "
            "Use --no-archive to skip."
        ),
    )
    parser.add_argument(
        "--verbose",
        "-v",
        action="store_true",
        help="Enable verbose (INFO) logging.",
    )
    return parser.parse_args()


# ── Calibration ─────────────────────────────────────────────────────


def _run_calibration(
    urdf_path: str,
    config_path: str,
    *,
    plot: bool = True,
    verbose: bool = False,
    validation_data: str | None = None,
    html_report: bool = False,
    asset_id: str | None = None,
    operator: str | None = None,
    archive: bool = False,
) -> tuple[np.ndarray, list[str], str]:
    """Run TIAGo calibration.

    Returns (result_vector, param_names, saved_path) where *saved_path*
    is the timestamped .npz path.
    """
    tiago = load_robot(urdf_path, load_by_urdf=True, robot_pkg="tiago_description")
    tiago_calib = TiagoCalibration(tiago, config_path, del_list=[])
    tiago_calib.calib_config["known_baseframe"] = False
    tiago_calib.calib_config["known_tipframe"] = False
    if validation_data:
        tiago_calib.calib_config["validation_data_file"] = validation_data
    # CLI --asset-id/--operator override the config's robot.instance block
    # (if any) for this run's provenance record.
    if asset_id or operator:
        instance = dict(tiago_calib.calib_config.get("instance") or {})
        if asset_id:
            instance["asset_id"] = asset_id
        if operator:
            instance["operator"] = operator
        tiago_calib.calib_config["instance"] = instance
    tiago_calib.initialize()
    result = tiago_calib.solve(
        plotting=plot, enable_logging=verbose, html_report=False
    )
    param_names = tiago_calib.calib_config["param_name"]

    # V&V report suite: compute path once, write directly
    run_dir = None
    if html_report or archive:
        run_dir = compute_run_dir(tiago_calib)

    if html_report:
        tiago_calib.export_html_report(output_path=str(run_dir / "report.html"))

    if archive:
        archive_run(tiago_calib, run_dir)

    if run_dir:
        print(f"Results written to: {run_dir}")

    # Save with timestamp
    os.makedirs(DATA_DIR, exist_ok=True)
    ts = _timestamp_str()
    saved_path = os.path.join(DATA_DIR, f"calibration_results_{ts}.npz")
    np.savez(saved_path, result=result.x, param_names=param_names)
    print(f"Calibration results saved to {saved_path}")

    # Print log-map residual statistics
    PEE_est = tiago_calib.get_pose_from_measure(result.x)
    residuals = tiago_calib._compute_logmap_residuals(
        tiago_calib.PEE_measured, PEE_est
    )
    calib_config = tiago_calib.calib_config
    n_dofs = calib_config["calibration_index"]
    n_samples = calib_config["NbSample"]
    residuals_2d = residuals.reshape((n_dofs, n_samples))
    rmse = np.sqrt(np.mean(residuals**2))
    print(f"\nPost-calibration residual statistics (log map):")
    print(
        f"  Position RMSE: "
        f"{np.sqrt(np.mean(np.sum(residuals_2d**2, axis=0))) * 1000:.2f} mm"
    )
    print(f"  Overall RMSE: {rmse:.6f}")

    return result.x, param_names, saved_path


# ── Export + verify ─────────────────────────────────────────────────


def export_with_verification(
    params: dict,
    nominal_urdf: str | Path,
    *,
    output_path: str | Path | None = None,
    calibration_type: str = "mocap",
    verbose: bool = False,
) -> tuple[str, URDFComparison, object]:
    """Export URDF with joint-level params and verify via URDFComparison.

    Args:
        params: Dict of {param_name: value} (joint + frame params).
                Frame params are auto-detected and NOT applied.
        nominal_urdf: Path to the nominal URDF.
        output_path: Path for modified URDF.  If None, generates
                     a timestamped path: urdf/<stem>_modified_<ts>.urdf.
        calibration_type: Passed to frame_settings_doc().
        verbose: Enable verbose logging.

    Returns:
        (modified_urdf_path, comparison, errors) where *errors* is a
        FkConsistencyResult from ``comparison.fk_consistency_check()``.
    """
    nominal_path = Path(nominal_urdf)

    # Generate timestamped output path if none provided
    if output_path is None:
        stem = nominal_path.stem
        ts = _timestamp_str()
        output_path = str(nominal_path.parent / f"{stem}_modified_{ts}.urdf")

    # Export URDF — export_urdf() auto-splits joint vs frame params
    modified_path = export_urdf(
        str(nominal_path),
        params,
        output_path=str(output_path),
        verbose=verbose,
    )

    # Show metrology frame documentation
    frame_settings_doc(calibration_type=calibration_type, verbose=verbose)

    # Print metrology-frame params (not auto-applied)
    frame_params = {
        k: v
        for k, v in params.items()
        if k.startswith(("base_", "pEE", "phiEE"))
    }
    if frame_params:
        print("\nMetrology frame parameters (NOT auto-applied to URDF):")
        for k, v in sorted(frame_params.items()):
            print(f"  {k} = {v:.6f}")
        print(
            "  → These define the calibration-setup transform and must be\n"
            "    configured in your controller or pipeline. See\n"
            "    frame_settings_doc() for defaults and explanations."
        )
    else:
        print("\nNo metrology frame parameters in calibration result.")

    # URDF export consistency check

    print("URDF export consistency check (nominal vs. exported URDF)")
    print("=" * 60)
    comp = URDFComparison(str(nominal_path), modified_path)
    errors = comp.fk_consistency_check(n_samples=200)
    print(f"  Position RMSE:    {errors.rmse_position * 1000:.2f} mm")
    print(f"  Orientation RMSE: {errors.rmse_orientation * 180 / np.pi:.4f} deg")
    print(f"  Max position:     {errors.max_position * 1000:.2f} mm")
    print(f"  Max orientation:  {errors.max_orientation * 180 / np.pi:.4f} deg")
    print(f"  (samples: {len(errors.per_sample)})")

    return modified_path, comp, errors


# ── Visual validation ───────────────────────────────────────────────


def show_validation(comp: URDFComparison):
    """Open interactive viser validation with trajectory, static comparison,
    error plots, replay, and opacity controls.

    See :meth:`URDFComparison.show_interactive_validation` for details.
    """
    try:
        import viser  # noqa: F401
    except ImportError:
        print("  viser not installed; skipping visual validation.")
        return

    comp.show_interactive_validation(n_trajectory=50, port=8080)


# ── Mode handlers ───────────────────────────────────────────────────


def _run_update_model(args: argparse.Namespace) -> None:
    """Load saved .npz → export URDF → verify FK."""
    # Select .npz file
    npz_path = args.model if args.model else _select_npz()
    print(f"\nLoading calibration results from: {npz_path}")
    data = np.load(npz_path)
    result_x = data["result"]
    param_names = list(data["param_names"])
    params = dict(zip(param_names, result_x))
    print(f"Loaded {len(params)} calibration parameters.")

    # Export + verify
    modified_urdf, comp, errors = export_with_verification(
        params,
        str(args.urdf),
        output_path=args.output,
        verbose=args.verbose,
    )

    print("\n" + "=" * 60)
    print("Update complete.")
    print(f"  Source results: {npz_path}")
    print(f"  Nominal URDF:   {args.urdf}")
    print(f"  Modified URDF:  {modified_urdf}")
    print("=" * 60)


def _run_viz_validation(args: argparse.Namespace) -> None:
    """Visually validate a previously exported modified URDF."""
    # Select modified URDF
    if args.model:
        modified_path = args.model
    else:
        modified_path = _select_modified_urdf(URDF_STEM)
    print(f"\nValidating modified URDF: {modified_path}")
    print(f"  Against nominal: {args.urdf}")

    comp = URDFComparison(str(args.urdf), modified_path)
    show_validation(comp)


# ── Main ────────────────────────────────────────────────────────────


def main() -> None:
    """Run the full calibration → export → verify → visualise pipeline."""
    args = parse_args()

    logging.basicConfig(
        level=logging.INFO if args.verbose else logging.WARNING,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    # Validate input files
    urdf_path = Path(args.urdf)
    config_path = Path(args.config)
    if not urdf_path.exists():
        print(f"Error: URDF not found: {urdf_path}", file=sys.stderr)
        sys.exit(1)

    try:
        # ── MODE: --update-model ──
        if args.update_model:
            _run_update_model(args)
            return

        # ── MODE: --viz-validation ──
        if args.viz_validation:
            _run_viz_validation(args)
            return

        # Determine which steps to run
        if args.interactive:
            if not sys.stdin.isatty():
                print(
                    "Warning: --interactive with non-TTY stdin. "
                    "Running full pipeline."
                )
                steps = ["calibrate", "export", "verify", "viz"]
            else:
                steps = _select_steps()
        else:
            steps = ["calibrate", "export", "verify", "viz"]
            if args.calibrate_only:
                steps = ["calibrate"]

        # Auto-enable dependencies
        if "viz" in steps and "export" not in steps:
            print("Note: 'viz' requires a modified URDF. Including 'export' + 'verify'.")
            steps.extend(["export", "verify"])

        # Validate config (needed by calibration step)
        if "calibrate" in steps and not config_path.exists():
            print(f"Error: Config not found: {config_path}", file=sys.stderr)
            sys.exit(1)

        # ── Phase 1: Calibration ──
        result_x = None
        param_names = None
        if "calibrate" in steps:
            result_x, param_names, saved_path = _run_calibration(
                str(urdf_path),
                str(config_path),
                plot=not args.no_plot,
                verbose=args.verbose,
                validation_data=args.validation_data,
                html_report=args.html_report,
                asset_id=args.asset_id,
                operator=args.operator,
                archive=args.archive,
            )
            if args.calibrate_only:
                print(
                    f"\nTip: Run `python calibration.py --update-model` "
                    f"to export URDF and verify FK."
                )
                return
        else:
            # Load latest saved results if needed for export
            if "export" in steps or "verify" in steps or "viz" in steps:
                npz_path = _select_npz()
                print(f"Loading calibration results from: {npz_path}")
                data = np.load(npz_path)
                result_x = data["result"]
                param_names = list(data["param_names"])

        params = dict(zip(param_names, result_x))
        print(f"\nLoaded {len(params)} calibration parameters.")

        # ── Phase 2: Export + verify ──
        comp = None
        modified_urdf = None
        if "export" in steps and params is not None:
            modified_urdf, comp, errors = export_with_verification(
                params,
                str(urdf_path),
                output_path=args.output,
                verbose=args.verbose,
            )

        # ── Phase 3: Visual validation ──
        if "viz" in steps and comp is not None:
            show_validation(comp)

        print("\n" + "=" * 60)
        if result_x is not None:
            if modified_urdf:
                print("Calibration + export complete.")
                print(f"  Nominal URDF:  {urdf_path}")
                print(f"  Modified URDF: {modified_urdf}")
            elif args.calibrate_only:
                print("Calibration results saved.")
        print("=" * 60)

    except Exception as e:
        print(f"\nError: {e}", file=sys.stderr)
        import traceback

        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
