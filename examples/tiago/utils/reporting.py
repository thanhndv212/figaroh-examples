"""Lightweight plotting and JSON reporting for the TIAGo suspension and
backlash empirical-surface examples.

These two examples are standalone (they do not use ``BaseIdentification`` or
``BaseCalibration``), so this module intentionally does not depend on
``figaroh.utils.results_manager`` — it is a small, dependency-light helper
scoped to these two scripts. Matplotlib plots are always rendered to a
non-interactive ("Agg") backend and saved as PNG files rather than shown on
screen, so both scripts run unattended in headless/CI environments.
``plot_backlash_surface_3d`` is the one exception: it writes an interactive
Plotly HTML file and requires the optional ``plotly`` package, matching the
historical ``SurfaceFitting.plot_3dregression``.
"""

from __future__ import annotations

import json
import logging
import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

logger = logging.getLogger(__name__)

WRENCH_LABELS = ("Fx", "Fy", "Fz", "Mx", "My", "Mz")


def ensure_results_dir(subdirectory: str, root: str | Path = "results") -> Path:
    """Create (if needed) and return ``root/subdirectory``."""
    output_dir = Path(root) / subdirectory
    output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir


def utc_timestamp() -> str:
    """Return an ISO-8601 UTC timestamp for report provenance."""
    return datetime.now(timezone.utc).isoformat()


def save_json_report(report: dict[str, Any], path: str | Path) -> Path:
    """Write ``report`` as indented JSON, converting NumPy values first."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as report_file:
        json.dump(_to_jsonable(report), report_file, indent=2, sort_keys=True)
    return path


def _to_jsonable(value: Any) -> Any:
    if isinstance(value, dict):
        return {key: _to_jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_to_jsonable(item) for item in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.floating):
        return float(value)
    return value


def _get_pyplot():
    """Import ``matplotlib.pyplot`` on a non-interactive backend."""
    import matplotlib

    if "MPLBACKEND" not in os.environ:
        matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    return plt


def plot_suspension_fit(
    time: np.ndarray,
    wrench: np.ndarray,
    predicted_wrench: np.ndarray,
    output_path: str | Path,
    *,
    title: str = "TIAGo generalized-base suspension fit",
) -> Path:
    """Plot measured vs. fitted wrench components over time and save a PNG.

    Args:
        time: Timestamps, shape ``(n_samples,)``.
        wrench: Measured wrench, shape ``(n_samples, 6)``.
        predicted_wrench: DOF-major flat prediction from
            :func:`~examples.tiago.utils.suspension_model.
            fit_generalized_base_suspension`, shape ``(n_samples * 6,)``.
        output_path: Destination PNG path (parent directories are created).
        title: Figure title.
    """
    plt = _get_pyplot()
    time = np.asarray(time, dtype=float)
    measured = np.asarray(wrench, dtype=float).reshape(-1, 6)
    n_samples = measured.shape[0]
    predicted = np.asarray(predicted_wrench, dtype=float).reshape(
        n_samples, 6, order="F"
    )

    fig, axes = plt.subplots(6, 1, figsize=(8, 10), sharex=True)
    for dof, (axis, label) in enumerate(zip(axes, WRENCH_LABELS)):
        axis.plot(time, measured[:, dof], label="measured", linewidth=1.0)
        axis.plot(
            time,
            predicted[:, dof],
            label="fitted",
            linewidth=1.0,
            linestyle="--",
        )
        axis.set_ylabel(label)
        axis.grid(True, alpha=0.3)
    axes[0].legend(loc="upper right")
    axes[-1].set_xlabel("time [s]")
    fig.suptitle(title)
    fig.tight_layout()

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return output_path


def plot_backlash_surface_3d(
    position: np.ndarray,
    load_proxy: np.ndarray,
    target: np.ndarray,
    predicted: np.ndarray,
    output_path: str | Path,
    *,
    title: str = "TIAGo empirical backlash-surface fit (3D)",
) -> Path:
    """Plot measured vs. fitted encoder-difference surface in 3D and save HTML.

    Equivalent to the historical ``SurfaceFitting.plot_3dregression``: two
    3D scatter traces (measured, fitted) over position and the load feature.
    Unlike the historical method, this always writes to disk instead of
    opening a browser, so it still runs unattended. Requires the optional
    ``plotly`` package; raises :class:`ImportError` if it is not installed.
    """
    import plotly.graph_objects as go

    position = np.asarray(position, dtype=float)
    load_proxy = np.asarray(load_proxy, dtype=float)
    target = np.asarray(target, dtype=float)
    predicted = np.asarray(predicted, dtype=float)

    fig = go.Figure(
        data=[
            go.Scatter3d(
                x=position,
                y=load_proxy,
                z=target,
                mode="markers",
                marker=dict(size=2),
                opacity=0.8,
                name="measured encoder difference",
            ),
            go.Scatter3d(
                x=position,
                y=load_proxy,
                z=predicted,
                mode="markers",
                marker=dict(size=2),
                opacity=0.5,
                name="fitted encoder difference",
            ),
        ]
    )
    fig.update_layout(
        title=title,
        scene=dict(
            xaxis_title="relative position [rad]",
            yaxis_title="load feature",
            zaxis_title="relative - absolute encoder [rad]",
        ),
        width=1000,
        height=800,
        showlegend=True,
    )

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.write_html(str(output_path))
    return output_path


def _draw_backlash_fit_panel(
    fit_axis,
    residual_axis,
    position: np.ndarray,
    target: np.ndarray,
    predicted: np.ndarray | None,
    velocity: np.ndarray,
    *,
    legend: bool = False,
    status_note: str | None = None,
) -> None:
    """Draw measured/fitted scatter (and optional residuals) on given axes.

    Fitted points are drawn as a scatter, not a line connecting points
    ordered by position: on real (non-monotonic, multi-cycle) data the
    prediction also depends on the load feature and direction at each
    sample, so a position-ordered line zigzags between distant predicted
    values and reads as noise rather than a fitted surface.

    If ``predicted`` is ``None`` (e.g. the fit was rejected as rank
    deficient), only the measured points are drawn and ``status_note`` is
    shown in place of fitted/residual content.
    """
    position = np.asarray(position, dtype=float)
    target = np.asarray(target, dtype=float)
    velocity = np.asarray(velocity, dtype=float)
    positive = velocity > 0.0

    fit_axis.scatter(
        position[positive],
        target[positive],
        s=8,
        alpha=0.4,
        color="tab:blue",
        label="measured (v > 0)",
    )
    fit_axis.scatter(
        position[~positive],
        target[~positive],
        s=8,
        alpha=0.4,
        color="tab:orange",
        label="measured (v <= 0)",
    )
    if predicted is not None:
        predicted = np.asarray(predicted, dtype=float)
        fit_axis.scatter(
            position[positive],
            predicted[positive],
            s=6,
            alpha=0.6,
            marker="x",
            color="tab:cyan",
            label="fitted (v > 0)",
        )
        fit_axis.scatter(
            position[~positive],
            predicted[~positive],
            s=6,
            alpha=0.6,
            marker="x",
            color="tab:red",
            label="fitted (v <= 0)",
        )
    elif status_note:
        fit_axis.text(
            0.5, 0.05, status_note, transform=fit_axis.transAxes,
            ha="center", va="bottom", fontsize=9, color="tab:red",
        )
    fit_axis.set_xlabel("relative position [rad]")
    fit_axis.set_ylabel("relative - absolute encoder [rad]")
    fit_axis.grid(True, alpha=0.3)
    if legend:
        fit_axis.legend(loc="best", fontsize="small")

    if residual_axis is not None:
        if predicted is not None:
            residual = predicted - target
            residual_axis.scatter(
                position, residual, s=8, alpha=0.4, color="tab:red"
            )
            residual_axis.axhline(0.0, color="black", linewidth=0.8)
            residual_axis.set_xlabel("relative position [rad]")
            residual_axis.set_ylabel("fitted - measured [rad]")
        else:
            residual_axis.text(
                0.5, 0.5, status_note or "no fit", transform=residual_axis.transAxes,
                ha="center", va="center", fontsize=9, color="tab:red",
            )
            residual_axis.set_xticks([])
            residual_axis.set_yticks([])
        residual_axis.grid(True, alpha=0.3)


def plot_backlash_fit(
    position: np.ndarray,
    load_proxy: np.ndarray,
    velocity: np.ndarray,
    target: np.ndarray,
    predicted: np.ndarray,
    output_path: str | Path,
    *,
    title: str = "TIAGo empirical backlash-surface fit",
) -> Path:
    """Plot measured vs. fitted encoder difference and residuals; save a PNG.

    Points are colored by motion direction (the same sign convention used by
    :class:`~examples.tiago.utils.backlash_surface.EmpiricalBacklashSurface`'s
    direction gate) so the two fitted directional surfaces are visually
    distinguishable.
    """
    plt = _get_pyplot()
    del load_proxy

    fig, (fit_axis, residual_axis) = plt.subplots(2, 1, figsize=(8, 8))
    _draw_backlash_fit_panel(
        fit_axis, residual_axis, position, target, predicted, velocity,
        legend=True,
    )
    fig.suptitle(title)
    fig.tight_layout()

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return output_path


def plot_backlash_fit_grid(
    joint_results: list[dict[str, Any]],
    output_path: str | Path,
    *,
    title: str = "TIAGo empirical backlash-surface fit (all joints)",
) -> Path:
    """Plot measured vs. fitted encoder difference for several joints at once.

    Args:
        joint_results: One dict per joint with keys ``joint_name``,
            ``position``, ``target``, ``predicted``, ``velocity``, and
            optionally ``rmse``/``r2`` (shown in the row's subtitle).
        output_path: Destination PNG path.
        title: Figure-level title.
    """
    plt = _get_pyplot()
    n_joints = len(joint_results)
    fig, axes = plt.subplots(
        n_joints, 2, figsize=(10, 3.2 * n_joints), squeeze=False
    )

    for row, result in enumerate(joint_results):
        fit_axis, residual_axis = axes[row]
        _draw_backlash_fit_panel(
            fit_axis,
            residual_axis,
            result["position"],
            result["target"],
            result["predicted"],
            result["velocity"],
            legend=(row == 0),
            status_note=result.get("status_note"),
        )
        subtitle = result["joint_name"]
        if "rmse" in result:
            degree_part = (
                f"degree={result['degree']}, " if "degree" in result else ""
            )
            subtitle += f" ({degree_part}rmse={result['rmse']:.3e}"
            subtitle += f", r2={result['r2']:.3f})" if "r2" in result else ")"
        elif result.get("status_note"):
            subtitle += f" ({result['status_note']})"
        fit_axis.set_title(subtitle, fontsize=10, loc="left")

    fig.suptitle(title)
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.98))

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return output_path


def _r2_tier(r2: float | None) -> str:
    """Classify R2 into 'good' (>=0.9), 'fair' (>=0.5), or 'poor'."""
    if r2 is None:
        return "poor"
    if r2 >= 0.9:
        return "good"
    if r2 >= 0.5:
        return "fair"
    return "poor"


_TIER_COLOR = {"good": "#2e7d46", "fair": "#9a6a0a", "poor": "#b3261e"}


def plot_backlash_joint_comparison_bars(
    joint_stats: list[dict[str, Any]],
    output_path: str | Path,
    *,
    title: str = "TIAGo backlash-surface fit quality by joint",
) -> Path:
    """Plot R2 and RMSE side by side as horizontal bars, one row per joint.

    Args:
        joint_stats: One dict per joint with keys ``joint_name`` and,
            when fit succeeded, ``r2`` and ``rmse`` (omitted or ``None``
            for a joint with an insufficient-excitation status).
    """
    plt = _get_pyplot()
    joint_names = [stat["joint_name"] for stat in joint_stats]
    r2_values = [stat.get("r2") for stat in joint_stats]
    rmse_values = [stat.get("rmse") for stat in joint_stats]
    colors = [_TIER_COLOR[_r2_tier(value)] for value in r2_values]
    y_pos = np.arange(len(joint_names))

    fig, (r2_axis, rmse_axis) = plt.subplots(
        1, 2, figsize=(10, 0.5 * len(joint_names) + 1)
    )

    r2_axis.barh(y_pos, [v if v is not None else 0.0 for v in r2_values], color=colors)
    r2_axis.set_yticks(y_pos, joint_names)
    r2_axis.invert_yaxis()
    r2_axis.set_xlabel("R2")
    r2_axis.set_xlim(0.0, 1.0)
    r2_axis.axvline(0.9, color="black", linewidth=0.8, linestyle="--", alpha=0.4)
    r2_axis.grid(True, axis="x", alpha=0.3)
    for y, value in zip(y_pos, r2_values):
        r2_axis.text(
            0.02, y, f"{value:.3f}" if value is not None else "n/a",
            va="center", fontsize=8,
        )

    rmse_plot = [v if v is not None else 0.0 for v in rmse_values]
    rmse_axis.barh(y_pos, rmse_plot, color=colors)
    rmse_axis.set_yticks(y_pos, [])
    rmse_axis.invert_yaxis()
    rmse_axis.set_xlabel("RMSE [rad]")
    rmse_axis.grid(True, axis="x", alpha=0.3)
    label_x = max(rmse_plot) * 0.02 if max(rmse_plot) > 0 else 0.0
    for y, value in zip(y_pos, rmse_values):
        rmse_axis.text(
            label_x, y, f"{value:.2e}" if value is not None else "n/a",
            va="center", fontsize=8,
        )

    fig.suptitle(title)
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.95))

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return output_path


def _image_data_uri(path: str | Path) -> str:
    import base64

    data = Path(path).read_bytes()
    return "data:image/png;base64," + base64.b64encode(data).decode("ascii")


def _html_escape(text: str) -> str:
    return (
        str(text)
        .replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
    )


_BACKLASH_REPORT_CSS = """
:root {
  --bg: #f5f6f8; --surface: #ffffff; --surface-2: #eef0f3;
  --border: #dce0e6; --text: #1b1f27; --text-muted: #5b6472;
  --accent: #2e5c8a; --good: #2e7d46; --good-bg: #e6f4ea;
  --fair: #9a6a0a; --fair-bg: #fbf1dd; --poor: #b3261e; --poor-bg: #fbe7e6;
}
@media (prefers-color-scheme: dark) {
  :root {
    --bg: #14171c; --surface: #1a1e25; --surface-2: #20242c;
    --border: #2b3038; --text: #e7e9ed; --text-muted: #9aa3b0;
    --accent: #7fb0e0; --good: #7fd39a; --good-bg: #16281d;
    --fair: #e0b95c; --fair-bg: #2e2510; --poor: #e88f88; --poor-bg: #331714;
  }
}
* { box-sizing: border-box; }
body {
  margin: 0; padding: 40px 24px 80px; background: var(--bg); color: var(--text);
  font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica,
               Arial, sans-serif;
  line-height: 1.55;
}
.page { max-width: 980px; margin: 0 auto; }
h1 { font-size: 1.5rem; margin: 0 0 4px; }
.subtitle { color: var(--text-muted); font-size: .85rem; margin: 0 0 32px; }
section { margin-top: 34px; }
h2 {
  font-size: 1.05rem; margin: 0 0 12px; padding-bottom: 8px;
  border-bottom: 1px solid var(--border);
}
.card {
  background: var(--surface); border: 1px solid var(--border);
  border-radius: 10px; padding: 18px 20px;
}
.stat-row { display: flex; flex-wrap: wrap; gap: 22px; }
.stat { min-width: 110px; }
.stat-label {
  font-size: .72rem; text-transform: uppercase; letter-spacing: .05em;
  color: var(--text-muted); margin-bottom: 2px;
}
.stat-value { font-size: 1.3rem; font-weight: 600; }
table.data { width: 100%; border-collapse: collapse; font-size: .85rem; }
table.data th, table.data td {
  padding: 7px 9px; text-align: right; border-bottom: 1px solid var(--border);
  white-space: nowrap;
}
table.data th:first-child, table.data td:first-child { text-align: left; }
table.data thead th {
  font-size: .68rem; text-transform: uppercase; letter-spacing: .04em;
  color: var(--text-muted); font-weight: 600;
}
table.data tbody tr:last-child td { border-bottom: none; }
tr.tier-poor td:first-child { color: var(--poor); font-weight: 600; }
tr.tier-fair td:first-child { color: var(--fair); font-weight: 600; }
.tag {
  display: inline-block; padding: 1px 9px; border-radius: 99px;
  font-size: .78rem; font-weight: 600;
}
.tag.good { background: var(--good-bg); color: var(--good); }
.tag.fair { background: var(--fair-bg); color: var(--fair); }
.tag.poor { background: var(--poor-bg); color: var(--poor); }
.tag.info { background: var(--surface-2); color: var(--accent); }
ul.insights { list-style: none; margin: 0; padding: 0; display: flex;
  flex-direction: column; gap: 8px; }
li.insight {
  padding: 10px 14px; border-radius: 8px; font-size: .88rem;
  border-left: 3px solid var(--border);
}
li.insight.warn { background: var(--poor-bg); border-left-color: var(--poor); }
li.insight.info { background: var(--surface-2); border-left-color: var(--accent); }
li.insight.good { background: var(--good-bg); border-left-color: var(--good); }
figure { margin: 0; }
figure img { width: 100%; border: 1px solid var(--border); border-radius: 8px; }
figcaption {
  font-size: .78rem; color: var(--text-muted); margin-top: 6px; text-align: center;
}
.link-grid { display: grid;
  grid-template-columns: repeat(auto-fill, minmax(200px, 1fr)); gap: 12px; }
.link-card {
  display: block; padding: 12px 14px; border-radius: 8px;
  border: 1px solid var(--border); background: var(--surface);
  text-decoration: none; color: var(--text);
}
.link-card:hover { border-color: var(--accent); }
.link-card .name { font-weight: 600; font-size: .92rem; }
.link-card .meta { font-size: .78rem; color: var(--text-muted); margin-top: 3px; }
.link-card.disabled { opacity: .55; pointer-events: none; }
.link-card .open { font-size: .78rem; color: var(--accent); margin-top: 6px; }
.kv-grid { display: grid;
  grid-template-columns: repeat(auto-fit, minmax(230px, 1fr));
  gap: 20px 30px; }
.kv-row { display: flex; justify-content: space-between; gap: 14px;
  padding: 4px 0; font-size: .85rem; border-bottom: 1px dotted var(--border); }
.kv-row:last-child { border-bottom: none; }
.kv-key { color: var(--text-muted); flex-shrink: 0; }
.kv-val { font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
  text-align: right; word-break: break-all; }
footer { margin-top: 48px; font-size: .78rem; color: var(--text-muted);
  border-top: 1px solid var(--border); padding-top: 14px; }
footer code { font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace; }
"""


def render_backlash_all_joints_report_html(
    joint_stats: list[dict[str, Any]],
    output_path: str | Path,
    *,
    title: str,
    generated_at: str,
    data_source: str,
    load_feature: str,
    degree_requested: int,
    transition_steepness: float,
    provenance: dict[str, str],
    grid_plot_path: str | Path,
    comparison_plot_path: str | Path,
) -> Path:
    """Render a self-contained, professional-style HTML report across joints.

    Mirrors the visual language of FIGAROH's own identification/calibration
    HTML reports (cards, tiered tags, a bar-annotated stats table) without
    depending on ``figaroh.utils.results_manager``, keeping this module's
    dependency-light contract. Plots are embedded as base64 PNGs so the
    report is a single portable file.
    """
    fit_stats = [stat for stat in joint_stats if stat.get("status") == "fit"]
    failed_stats = [
        stat for stat in joint_stats if stat.get("status") != "fit"
    ]
    r2_values = [stat["r2"] for stat in fit_stats]
    full_degree_count = sum(
        1 for stat in fit_stats if stat["degree_used"] == stat["degree_requested"]
    )
    backed_off = [
        stat for stat in fit_stats if stat["degree_used"] < stat["degree_requested"]
    ]
    median_r2 = float(np.median(r2_values)) if r2_values else float("nan")
    total_samples = sum(stat["sample_count"] for stat in joint_stats)

    insights: list[tuple[str, str]] = []
    if full_degree_count == len(fit_stats) and not failed_stats:
        insights.append(
            ("good", f"All {len(fit_stats)} fit joints reached the requested "
             f"degree {degree_requested} without rank deficiency.")
        )
    for stat in backed_off:
        insights.append((
            "warn",
            f"<b>{_html_escape(stat['joint_name'])}</b>: requested degree "
            f"{stat['degree_requested']} was still rank deficient after "
            f"dropping structurally-zero terms; backed off to degree "
            f"{stat['degree_used']}. The load feature is only moderately "
            "independent of position for this joint at high degree "
            "(near-collinear, not exactly zero).",
        ))
    dropped_terms = [
        stat for stat in fit_stats if stat.get("dropped_term_count", 0) > 0
    ]
    for stat in dropped_terms:
        insights.append((
            "info",
            f"<b>{_html_escape(stat['joint_name'])}</b>: "
            f"{stat['dropped_term_count']} of {stat['coefficient_count']} "
            f"degree-{stat['degree_used']} terms involve the load feature "
            f"and were dropped as structurally zero "
            f"(load-feature std={stat['load_feature_std']:.2e}) — a "
            "coefficient multiplying an exact-zero column is undetermined, "
            "not just hard to estimate. This joint's rotation axis likely "
            "does not couple to gravity at all (e.g. a near-vertical yaw "
            "axis), so only the position-only terms "
            f"({stat['active_coefficient_count']} of "
            f"{stat['coefficient_count']}) were fit, still at the full "
            f"requested degree {stat['degree_requested']}.",
        ))
    for stat in failed_stats:
        insights.append((
            "warn",
            f"<b>{_html_escape(stat['joint_name'])}</b>: insufficient "
            f"excitation even at degree 0 "
            f"({_html_escape(stat.get('error', 'rank deficient'))}).",
        ))
    for stat in fit_stats:
        if stat["r2"] < 0.5:
            insights.append((
                "warn",
                f"<b>{_html_escape(stat['joint_name'])}</b>: R2="
                f"{stat['r2']:.3f} is a poor fit; treat its coefficients as "
                "uninformative rather than a usable correction.",
            ))
    well_determined = [stat for stat in fit_stats if stat["r2"] >= 0.5]
    ill_conditioned = [
        stat for stat in well_determined if stat["condition_number"] > 1e6
    ]
    if ill_conditioned:
        joint_list = ", ".join(
            f"{_html_escape(stat['joint_name'])} "
            f"({stat['condition_number']:.1e})"
            for stat in ill_conditioned
        )
        better_conditioned = sorted(
            (s for s in well_determined if s not in ill_conditioned),
            key=lambda s: s["condition_number"],
        )
        comparison = (
            " Comparatively well-conditioned: "
            + ", ".join(
                f"{_html_escape(s['joint_name'])} "
                f"({s['condition_number']:.1e})"
                for s in better_conditioned
            )
            + "."
            if better_conditioned
            else ""
        )
        insights.append((
            "warn",
            f"{len(ill_conditioned)} of {len(well_determined)} well-fit "
            f"joints have a design-matrix condition number above 1e6: "
            f"{joint_list}. This is expected for a raw "
            "(non-orthogonalized) degree-5 monomial basis in physical "
            "units, not necessarily joint-specific pathology — but it "
            "means those coefficients are only weakly determined despite "
            f"a good in-sample R2, and should not be trusted to "
            f"extrapolate or transfer to another trajectory.{comparison}",
        ))

    def stat_cell(label: str, value: str) -> str:
        return (
            f'<div class="stat"><div class="stat-label">{label}</div>'
            f'<div class="stat-value">{value}</div></div>'
        )

    summary_stats = "".join([
        stat_cell("Joints fit", f"{len(fit_stats)} / {len(joint_stats)}"),
        stat_cell(
            "At full requested degree",
            f"{full_degree_count} / {len(fit_stats)}",
        ),
        stat_cell("Median R2", f"{median_r2:.3f}" if fit_stats else "n/a"),
        stat_cell("Total samples", f"{total_samples:,}"),
        stat_cell("Degree requested", str(degree_requested)),
        stat_cell("Transition steepness", f"{transition_steepness:g}"),
    ])

    table_rows = []
    for stat in joint_stats:
        joint_name = _html_escape(stat["joint_name"])
        if stat.get("status") != "fit":
            table_rows.append(
                f'<tr class="tier-poor"><td>{joint_name}</td>'
                f'<td colspan="10">insufficient excitation even at degree 0'
                f' — {_html_escape(stat.get("error", ""))}</td></tr>'
            )
            continue
        tier = _r2_tier(stat["r2"])
        backed_off_tag = (
            f'<span class="tag fair">backed off from '
            f'{stat["degree_requested"]}</span>'
            if stat["degree_used"] < stat["degree_requested"]
            else '<span class="tag good">full</span>'
        )
        dropped_tag = (
            f'<span class="tag info">'
            f'{stat["dropped_term_count"]} terms dropped</span>'
            if stat.get("dropped_term_count", 0) > 0
            else ""
        )
        table_rows.append(
            f'<tr class="tier-{tier}">'
            f'<td>{joint_name}</td>'
            f'<td>{stat["degree_used"]} {backed_off_tag} {dropped_tag}</td>'
            f'<td>{stat["sample_count"]:,}</td>'
            f'<td>{stat["r2"]:.4f}</td>'
            f'<td>{stat["adjusted_r2"]:.4f}</td>'
            f'<td>{stat["rmse"]:.3e}</td>'
            f'<td>{stat["residual_std"]:.3e}</td>'
            f'<td>{stat["rank"]}</td>'
            f'<td>{stat["condition_number"]:.2e}</td>'
            f'<td>{stat["load_feature_std"]:.2e}</td>'
            f'<td>{stat["n_positive"]} / {stat["n_negative"]}</td>'
            f'</tr>'
        )

    provenance_rows = "".join(
        f'<div class="kv-row"><span class="kv-key">{_html_escape(key)}</span>'
        f'<span class="kv-val">{_html_escape(value)}</span></div>'
        for key, value in provenance.items()
    )

    link_cards = []
    for stat in fit_stats:
        joint_name = _html_escape(stat["joint_name"])
        surface_path = stat.get("surface_plot_3d_path")
        if surface_path:
            href = Path(surface_path).name
            link_cards.append(
                f'<a class="link-card" href="{_html_escape(href)}" '
                f'target="_blank" rel="noopener">'
                f'<div class="name">{joint_name}</div>'
                f'<div class="meta">degree={stat["degree_used"]}, '
                f'R2={stat["r2"]:.3f}</div>'
                f'<div class="open">Open interactive 3D plot &#8599;</div>'
                f'</a>'
            )
        else:
            link_cards.append(
                f'<div class="link-card disabled">'
                f'<div class="name">{joint_name}</div>'
                f'<div class="meta">3D plot unavailable '
                f'(plotly not installed)</div></div>'
            )
    skipped_names = ", ".join(
        _html_escape(stat["joint_name"]) for stat in failed_stats
    )
    links_3d_section = (
        f'<div class="link-grid">{"".join(link_cards)}</div>'
        if link_cards
        else '<p class="subtitle">No joint reached a fit; no 3D plots '
        "were generated.</p>"
    )
    if skipped_names:
        links_3d_section += (
            f'<p class="subtitle" style="margin-top:10px">'
            f"Not shown (insufficient excitation, no fit): "
            f"{skipped_names}.</p>"
        )

    html = f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>{_html_escape(title)}</title>
<style>{_BACKLASH_REPORT_CSS}</style>
</head>
<body>
<div class="page">
  <h1>{_html_escape(title)}</h1>
  <p class="subtitle">Generated {_html_escape(generated_at)} &middot;
    data={_html_escape(data_source)} &middot;
    load feature={_html_escape(load_feature)}</p>

  <section>
    <h2>Executive Summary</h2>
    <div class="card"><div class="stat-row">{summary_stats}</div></div>
  </section>

  <section>
    <h2>Insights</h2>
    <ul class="insights">
      {"".join(f'<li class="insight {kind}">{text}</li>' for kind, text in insights)}
    </ul>
  </section>

  <section>
    <h2>Per-Joint Statistics</h2>
    <div class="card" style="overflow-x:auto">
      <table class="data">
        <thead><tr>
          <th>Joint</th><th>Degree</th><th>Samples</th><th>R2</th>
          <th>Adj. R2</th><th>RMSE [rad]</th><th>Residual std [rad]</th>
          <th>Rank</th><th>Cond. number</th><th>Load std</th>
          <th>N (v&gt;0 / v&le;0)</th>
        </tr></thead>
        <tbody>{"".join(table_rows)}</tbody>
      </table>
    </div>
  </section>

  <section>
    <h2>Fit Quality by Joint</h2>
    <figure>
      <img src="{_image_data_uri(comparison_plot_path)}" alt="R2 and RMSE by joint">
      <figcaption>R2 (left, dashed line at 0.90) and RMSE (right) per joint,
        colored by R2 tier: green &ge;0.90, amber &ge;0.50, red below.</figcaption>
    </figure>
  </section>

  <section>
    <h2>Measured vs. Fitted, All Joints</h2>
    <figure>
      <img src="{_image_data_uri(grid_plot_path)}"
           alt="Measured vs fitted encoder difference, all joints">
      <figcaption>Left: measured (blue/orange, by direction) vs. fitted
        (cyan/red) encoder difference against relative position. Right:
        fitted-minus-measured residuals.</figcaption>
    </figure>
  </section>

  <section>
    <h2>3D Regression Surfaces</h2>
    <p class="subtitle" style="margin-bottom:14px">
      Interactive Plotly plots (position, load feature, encoder
      difference) — measured vs. fitted point clouds, equivalent to the
      historical <code>SurfaceFitting.plot_3dregression</code>. Each opens
      in a new tab; keep these files alongside this report (same folder).
    </p>
    {links_3d_section}
  </section>

  <section>
    <h2>Data Provenance &amp; Methodology</h2>
    <div class="card">
      <div class="kv-grid">{provenance_rows}</div>
      <p class="subtitle" style="margin-top:16px">
        Load feature is Pinocchio generalized gravity torque, reconstructed
        per sample from every logged arm/torso/head joint position and
        evaluated against the TIAGo hey5 URDF — matching the historical
        figaroh-plus <code>SurfaceFitting.tau_g</code> feature. Polynomial
        degree backs off from the requested degree when the direction-gated
        design matrix is rank deficient (a joint whose axis does not couple
        to gravity, or with too little independent position/torque
        variation at high degree); degree 0 (direction-only offsets) is
        always full rank, so every joint yields a result.
      </p>
    </div>
  </section>

  <footer>
    Generated by <code>examples/tiago/backlash_empirical_surface.py --joint all</code>.
    Research/experimental artifact — not a validated deployable backlash model.
  </footer>
</div>
</body>
</html>
"""

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(html)
    return output_path
