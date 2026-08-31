"""Run a deterministic empirical TIAGo backlash-surface demonstration."""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

import numpy as np

project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from examples.tiago.utils.backlash_surface import (
    EmpiricalBacklashSurface,
    load_backlash_joint_trajectory,
    load_backlash_joint_trajectory_with_gravity,
)  # noqa: E402
from examples.tiago.utils.reporting import (  # noqa: E402
    ensure_results_dir,
    plot_backlash_fit,
    plot_backlash_fit_grid,
    plot_backlash_joint_comparison_bars,
    plot_backlash_surface_3d,
    render_backlash_all_joints_report_html,
    save_json_report,
    utc_timestamp,
)

logger = logging.getLogger(__name__)

ARM_JOINT_NAMES = [f"arm_{index}_joint" for index in range(1, 8)]
BACKLASH_DATA_DIR = Path("data/backlash/sinus_amp3_period10_2023-07-24-13-28-42")
# Same URDF the historical figaroh-plus backlash_inspection.py used to
# compute its `tau_g` gravity-torque feature.
TIAGO_URDF_PATH = Path("urdf/tiago_48_hey5.urdf")

# Matches the historical figaroh-plus default (orders=[5, 5] in
# backlash_inspection.py's __main__, applied to all seven arm joints). Its
# own order-selection study (surface_fitting_rho100_0202.csv) found degree 1
# reaches only R2=0.72 on arm_6_joint, vs. degree 5's R2=0.90.
DEFAULT_DEGREE_REAL = 5

# The synthetic demo's load feature (2*cos(position)) is a deterministic
# function of position alone, and its ground-truth target is itself
# degree-1 in position. A higher degree is both unnecessary and numerically
# rank deficient there (position and the load feature are not independently
# excited), so the synthetic default stays at the degree that matches the
# generator.
DEFAULT_DEGREE_SYNTHETIC = 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--samples", type=int, default=120, help="Synthetic sample count"
    )
    parser.add_argument(
        "--data",
        choices=("synthetic", "real"),
        default="synthetic",
        help="Use deterministic synthetic data or the bundled historical encoder log",
    )
    parser.add_argument(
        "--joint",
        default="arm_6_joint",
        help=(
            "Joint name to load when --data real is selected, or 'all' to "
            "fit every arm joint (arm_1_joint..arm_7_joint) and also emit "
            "a combined comparison plot."
        ),
    )
    parser.add_argument(
        "--degree",
        type=int,
        default=None,
        help=(
            "Bivariate polynomial degree for the directional surfaces. "
            f"Defaults to {DEFAULT_DEGREE_REAL} for --data real (matching "
            f"the historical figaroh-plus default) or "
            f"{DEFAULT_DEGREE_SYNTHETIC} for --data synthetic (matching "
            "the synthetic generator's own ground-truth degree)."
        ),
    )
    parser.add_argument(
        "--load-feature",
        choices=("gravity", "effort"),
        default="gravity",
        help=(
            "Load feature to pair with position for --data real: "
            "'gravity' computes Pinocchio generalized gravity torque from "
            "the logged full-arm configuration (matches the historical "
            "figaroh-plus tau_g feature; default). 'effort' uses the "
            "logged motor effort instead -- kept only for comparison, "
            "since it is heavily quantized on arm_5..arm_7 in this log "
            "and was the source of earlier rank-deficient fits there."
        ),
    )
    parser.add_argument(
        "--report",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Save a measured-vs-fitted encoder-correction plot (PNG) and a "
            "JSON report to results/backlash/. Use --no-report to skip."
        ),
    )
    parser.add_argument(
        "--report-3d",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Also save an interactive 3D measured-vs-fitted surface plot "
            "(HTML) per joint to results/backlash/, linked from the HTML "
            "report when --joint all is selected. Requires the optional "
            "'plotly' package; skipped with a warning if it is not "
            "installed. Use --no-report-3d to skip."
        ),
    )
    return parser.parse_args()


def _load_tiago_model():
    from figaroh.tools.robot import load_robot

    robot = load_robot(str(TIAGO_URDF_PATH.resolve()), load_by_urdf=True)
    return robot.model


def _load_real_joint_data(
    joint_name: str, load_feature: str, robot_model=None
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, str]:
    """Load one joint's (position, load_proxy, velocity, target) for --data real."""
    if load_feature == "gravity":
        trajectory, load_proxy = load_backlash_joint_trajectory_with_gravity(
            BACKLASH_DATA_DIR, joint_name, robot_model
        )
        source = "pinocchio_generalized_gravity"
    else:
        trajectory = load_backlash_joint_trajectory(BACKLASH_DATA_DIR, joint_name)
        load_proxy = trajectory.effort
        source = "historical_introspection_effort_proxy"
    return (
        trajectory.relative_position,
        load_proxy,
        trajectory.velocity,
        trajectory.encoder_difference,
        source,
    )


def _r2_score(target: np.ndarray, predicted: np.ndarray) -> float:
    residual_ss = np.sum((target - predicted) ** 2)
    total_ss = np.sum((target - np.mean(target)) ** 2)
    return float(1.0 - residual_ss / total_ss)


def _fit_joint(
    position: np.ndarray,
    load_proxy: np.ndarray,
    velocity: np.ndarray,
    target: np.ndarray,
    degree: int,
) -> tuple[EmpiricalBacklashSurface, np.ndarray, float, float]:
    model = EmpiricalBacklashSurface(degree=degree).fit(
        position, load_proxy, velocity, target
    )
    predicted = model.predict(position, load_proxy, velocity)
    rmse = float(np.sqrt(np.mean((predicted - target) ** 2)))
    r2 = _r2_score(target, predicted)
    return model, predicted, rmse, r2


def _fit_joint_with_backoff(
    position: np.ndarray,
    load_proxy: np.ndarray,
    velocity: np.ndarray,
    target: np.ndarray,
    max_degree: int,
) -> tuple[EmpiricalBacklashSurface, np.ndarray, float, float]:
    """Fit at the highest degree <= max_degree whose design matrix has full rank.

    Not every joint's load feature supports the same degree: e.g. a joint
    whose rotation axis does not couple to gravity has an identically-zero
    gravity-torque feature (rank deficient at any degree above 0), and a
    joint whose own position dominates its gravity torque can be
    ill-conditioned at high degree with few samples per direction. Degree 0
    (direction-only offsets) is always full rank given at least one sample
    per direction, so this always returns a result rather than raising.
    """
    error: ValueError | None = None
    for degree in range(max_degree, -1, -1):
        try:
            return _fit_joint(position, load_proxy, velocity, target, degree)
        except ValueError as caught:
            error = caught
    assert error is not None
    raise error


def _run_single_joint(args: argparse.Namespace) -> None:
    if args.data == "real":
        robot_model = (
            _load_tiago_model() if args.load_feature == "gravity" else None
        )
        position, load_proxy, velocity, target, source = _load_real_joint_data(
            args.joint, args.load_feature, robot_model
        )
    else:
        if args.samples < 20:
            raise ValueError("--samples must be at least 20")
        position = np.linspace(-0.8, 0.8, args.samples)
        load_proxy = 2.0 * np.cos(position)
        velocity = np.where(np.arange(args.samples) % 2 == 0, 0.2, -0.2)
        target = np.where(
            velocity > 0.0, 0.01 + 0.04 * position, -0.02 + 0.03 * position
        )
        source = "synthetic"

    try:
        model, predicted, rmse, r2 = _fit_joint_with_backoff(
            position, load_proxy, velocity, target, args.degree
        )
    except ValueError as error:
        load_std = float(np.std(load_proxy))
        print(
            f"data={args.data} source={source} degree<=0 "
            f"samples={target.size} load_feature_std={load_std:.3e} "
            f"INSUFFICIENT EXCITATION: {error}"
        )
        return
    if model.degree < args.degree:
        print(
            f"note: requested degree={args.degree} was rank deficient for "
            f"{args.joint}; backed off to degree={model.degree}"
        )
    print(
        f"data={args.data} source={source} degree={model.degree} "
        f"samples={target.size} rmse={rmse:.3e} r2={r2:.4f}"
    )

    if not args.report:
        return

    output_dir = ensure_results_dir("backlash")
    plot_path = plot_backlash_fit(
        position,
        load_proxy,
        velocity,
        target,
        predicted,
        output_dir / f"{args.data}_{args.joint}_fit.png",
        title=(
            f"TIAGo empirical backlash-surface fit "
            f"({args.data} data, {args.joint}, degree={model.degree})"
        ),
    )
    report = {
        "generated_at": utc_timestamp(),
        "data_source": args.data,
        "source_detail": source,
        "joint": args.joint,
        "degree": model.degree,
        "transition_steepness": model.transition_steepness,
        "sample_count": int(target.size),
        "rmse": rmse,
        "r2": r2,
        "position_range": model.position_range,
        "torque_range": model.torque_range,
        "coefficients": model.coefficients,
        "plot_path": str(plot_path),
    }

    surface_plot_path = None
    if args.report_3d:
        try:
            surface_plot_path = plot_backlash_surface_3d(
                position,
                load_proxy,
                target,
                predicted,
                output_dir / f"{args.data}_{args.joint}_surface_3d.html",
                title=(
                    f"TIAGo empirical backlash-surface fit "
                    f"({args.data} data, {args.joint})"
                ),
            )
            report["surface_plot_3d_path"] = str(surface_plot_path)
        except ImportError:
            logger.warning(
                "Skipping 3D surface plot: install the optional "
                "'plotly' package to enable it."
            )

    report_path = save_json_report(
        report, output_dir / f"{args.data}_{args.joint}_report.json"
    )
    print(f"Report written to: {report_path}")
    print(f"Plot written to: {plot_path}")
    if surface_plot_path is not None:
        print(f"3D surface plot written to: {surface_plot_path}")


def _run_all_joints(args: argparse.Namespace) -> None:
    if args.data != "real":
        raise ValueError("--joint all requires --data real")

    output_dir = ensure_results_dir("backlash")
    robot_model = _load_tiago_model() if args.load_feature == "gravity" else None
    joint_results = []
    joint_stats = []
    source = None
    for joint_name in ARM_JOINT_NAMES:
        position, load_proxy, velocity, target, source = _load_real_joint_data(
            joint_name, args.load_feature, robot_model
        )
        load_std = float(np.std(load_proxy))

        try:
            model, predicted, rmse, r2 = _fit_joint_with_backoff(
                position, load_proxy, velocity, target, args.degree
            )
        except ValueError as error:
            print(
                f"joint={joint_name} degree<=0 "
                f"samples={target.size} load_feature_std={load_std:.3e} "
                f"SKIPPED: {error}"
            )
            joint_results.append(
                {
                    "joint_name": joint_name,
                    "position": position,
                    "target": target,
                    "predicted": None,
                    "velocity": velocity,
                    "status_note": "insufficient excitation",
                }
            )
            joint_stats.append(
                {
                    "joint_name": joint_name,
                    "status": "insufficient_excitation",
                    "sample_count": int(target.size),
                    "error": str(error),
                    "load_feature_std": load_std,
                }
            )
            continue

        backoff_note = (
            f" (backed off from degree={args.degree})"
            if model.degree < args.degree
            else ""
        )
        dropped_note = (
            f" ({model.dropped_term_count} structurally-zero terms dropped)"
            if model.dropped_term_count
            else ""
        )
        print(
            f"joint={joint_name} degree={model.degree}{backoff_note}"
            f"{dropped_note} samples={target.size} rmse={rmse:.3e} r2={r2:.4f}"
        )
        joint_results.append(
            {
                "joint_name": joint_name,
                "position": position,
                "target": target,
                "predicted": predicted,
                "velocity": velocity,
                "rmse": rmse,
                "r2": r2,
                "degree": model.degree,
            }
        )
        residual = predicted - target
        n_samples = int(target.size)
        n_coefficients = int(model.coefficients.size)
        n_active_coefficients = n_coefficients - model.dropped_term_count
        degrees_of_freedom = max(n_samples - n_active_coefficients - 1, 1)
        adjusted_r2 = 1.0 - (1.0 - r2) * (n_samples - 1) / degrees_of_freedom
        stat = {
            "joint_name": joint_name,
            "status": "fit",
            "sample_count": n_samples,
            "degree_requested": args.degree,
            "degree_used": model.degree,
            "rmse": rmse,
            "r2": r2,
            "adjusted_r2": adjusted_r2,
            "residual_mean": float(np.mean(residual)),
            "residual_std": float(np.std(residual)),
            "residual_min": float(np.min(residual)),
            "residual_max": float(np.max(residual)),
            "rank": model.rank,
            "condition_number": model.condition_number,
            "coefficient_count": n_coefficients,
            "active_coefficient_count": n_active_coefficients,
            "dropped_term_count": model.dropped_term_count,
            "position_range": model.position_range,
            "torque_range": model.torque_range,
            "coefficients": model.coefficients,
            "load_feature_std": load_std,
            "n_positive": int(np.sum(velocity > 0.0)),
            "n_negative": int(np.sum(velocity <= 0.0)),
        }

        if args.report and args.report_3d:
            try:
                surface_plot_path = plot_backlash_surface_3d(
                    position,
                    load_proxy,
                    target,
                    predicted,
                    output_dir / f"{args.data}_{joint_name}_surface_3d.html",
                    title=(
                        f"TIAGo empirical backlash-surface fit "
                        f"({args.data} data, {joint_name}, "
                        f"degree={model.degree})"
                    ),
                )
                stat["surface_plot_3d_path"] = str(surface_plot_path)
            except ImportError:
                logger.warning(
                    "Skipping 3D surface plot for %s: install the "
                    "optional 'plotly' package to enable it.",
                    joint_name,
                )

        joint_stats.append(stat)

    if not args.report:
        return

    grid_plot_path = plot_backlash_fit_grid(
        joint_results,
        output_dir / f"{args.data}_all_joints_fit.png",
        title=(
            f"TIAGo empirical backlash-surface fit "
            f"({args.data} data, all arm joints, degree<={args.degree})"
        ),
    )
    comparison_plot_path = plot_backlash_joint_comparison_bars(
        joint_stats,
        output_dir / f"{args.data}_all_joints_comparison.png",
        title=f"TIAGo backlash-surface fit quality by joint ({args.data} data)",
    )
    transition_steepness = EmpiricalBacklashSurface(
        degree=args.degree
    ).transition_steepness
    html_report_path = render_backlash_all_joints_report_html(
        joint_stats,
        output_dir / f"{args.data}_all_joints_report.html",
        title="TIAGo Empirical Backlash-Surface — All Joints Report",
        generated_at=utc_timestamp(),
        data_source=args.data,
        load_feature=source,
        degree_requested=args.degree,
        transition_steepness=transition_steepness,
        provenance={
            "data source": str(BACKLASH_DATA_DIR),
            "robot URDF": str(TIAGO_URDF_PATH),
            "origin ref": "figaroh-plus",
            "origin commit": "2218d77638e0148afd3b358fa51702f8b82f4100",
        },
        grid_plot_path=grid_plot_path,
        comparison_plot_path=comparison_plot_path,
    )
    report = {
        "generated_at": utc_timestamp(),
        "data_source": args.data,
        "source_detail": source,
        "degree_requested": args.degree,
        "transition_steepness": transition_steepness,
        "joints": {stat["joint_name"]: stat for stat in joint_stats},
        "grid_plot_path": str(grid_plot_path),
        "comparison_plot_path": str(comparison_plot_path),
        "html_report_path": str(html_report_path),
    }
    report_path = save_json_report(
        report, output_dir / f"{args.data}_all_joints_report.json"
    )
    print(f"Report written to: {report_path}")
    print(f"Grid plot written to: {grid_plot_path}")
    print(f"Comparison plot written to: {comparison_plot_path}")
    print(f"HTML report written to: {html_report_path}")


def main() -> None:
    args = parse_args()
    if args.degree is None:
        args.degree = (
            DEFAULT_DEGREE_REAL
            if args.data == "real"
            else DEFAULT_DEGREE_SYNTHETIC
        )
    if args.joint == "all":
        _run_all_joints(args)
    else:
        _run_single_joint(args)


if __name__ == "__main__":
    main()
