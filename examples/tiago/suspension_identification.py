"""Run a deterministic generalized-base suspension identification demonstration."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from examples.tiago.utils.suspension_model import (  # noqa: E402
    build_generalized_base_regressor,
    fit_generalized_base_suspension,
)
from examples.tiago.utils.suspension_data import (  # noqa: E402
    load_vicon_forceplate_trajectory,
)
from examples.tiago.utils.reporting import (  # noqa: E402
    ensure_results_dir,
    plot_suspension_fit,
    save_json_report,
    utc_timestamp,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--samples", type=int, default=120, help="Synthetic sample count"
    )
    parser.add_argument(
        "--data",
        choices=("synthetic", "real"),
        default="synthetic",
        help="Use deterministic synthetic data or the bundled historical Vicon log",
    )
    parser.add_argument(
        "--report",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Save a measured-vs-fitted wrench plot (PNG) and a JSON report "
            "to results/suspension/. Use --no-report to skip."
        ),
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.data == "real":
        trajectory = load_vicon_forceplate_trajectory(
            Path("data/suspension/tiago_xyz_vicon_1640.csv")
        )
        result = fit_generalized_base_suspension(
            trajectory.base_displacement,
            trajectory.base_velocity,
            trajectory.wrench_vector,
        )
        sample_count = trajectory.sample_count
        time = trajectory.time - trajectory.time[0]
        wrench = trajectory.wrench
    else:
        if args.samples < 20:
            raise ValueError("--samples must be at least 20")
        time = np.linspace(0.0, 2.0, args.samples)
        displacement = np.column_stack(
            [0.02 * np.sin((index + 1) * time) for index in range(6)]
        )
        velocity = np.gradient(displacement, time, axis=0, edge_order=2)
        parameters = np.array(
            [
                1200.0,
                45.0,
                900.0,
                30.0,
                1500.0,
                55.0,
                180.0,
                8.0,
                220.0,
                9.0,
                140.0,
                7.0,
            ]
        )
        wrench_vector = (
            build_generalized_base_regressor(displacement, velocity) @ parameters
        )
        result = fit_generalized_base_suspension(
            displacement, velocity, wrench_vector
        )
        sample_count = args.samples
        wrench = wrench_vector.reshape(sample_count, 6, order="F")
    print(
        f"data={args.data} samples={sample_count} rank={result.rank} "
        f"condition_number={result.condition_number:.3g} "
        f"rmse={result.rmse:.3e}"
    )

    if args.report:
        output_dir = ensure_results_dir("suspension")
        plot_path = plot_suspension_fit(
            time,
            wrench,
            result.predicted_wrench,
            output_dir / f"{args.data}_fit.png",
            title=f"TIAGo generalized-base suspension fit ({args.data} data)",
        )
        report = {
            "generated_at": utc_timestamp(),
            "data_source": args.data,
            "sample_count": sample_count,
            "rank": result.rank,
            "condition_number": result.condition_number,
            "rmse": result.rmse,
            "parameter_order": [
                "kx",
                "cx",
                "ky",
                "cy",
                "kz",
                "cz",
                "krx",
                "crx",
                "kry",
                "cry",
                "krz",
                "crz",
            ],
            "parameters": result.parameters,
            "plot_path": str(plot_path),
        }
        report_path = save_json_report(
            report, output_dir / f"{args.data}_report.json"
        )
        print(f"Report written to: {report_path}")
        print(f"Plot written to: {plot_path}")


if __name__ == "__main__":
    main()
