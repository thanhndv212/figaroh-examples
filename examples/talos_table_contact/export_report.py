#!/usr/bin/env python3

# Copyright [2021-2026] Thanh Nguyen
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
Export a calibration results report -- YAML (same metadata /
calibrated_parameters shape as examples/tiago_pro/run_calibration.py's
write_calibration_results) plus a self-contained HTML summary -- for
``MultiChainCalibration`` specifically.

Why only ``MultiChainCalibration``: ``TalosTableContactCalibration`` (the
single-chain case) now drives figaroh core's own standard pipeline
directly -- ``solve()`` -> ``export_html_report()`` ->
``figaroh.tools.geometric_calibration_export.export_geometric_calibration_yaml``,
exactly as ``examples/tiago/calibration.py`` does -- via
``get_pose_from_measure``/``cost_function`` overrides that route its
plane/contact gap residual through core's own SE3 log-map machinery (see
``utils/talos_table_tools.py``) instead of a bespoke one; see
``run_calibration.py``/``run_calibration_real_data.py`` for that path.

``MultiChainCalibration`` isn't a ``BaseCalibration`` subclass at all --
it couples two independent ``TalosTableContactCalibration`` instances
into one joint solve -- so none of core's `solve()`/report machinery
applies to it, and there is no "standard multi-chain calibration report"
in figaroh to match. This module computes the same *kind* of information
(RMSE/MAE, parameter std-dev via the identical linearized-uncertainty
formula core's ``calc_stddev`` uses, condition number) from the coupled
solve's own residual, laid out in the same YAML shape tiago_pro's report
uses, purely for this one case.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np
import yaml

from utils.talos_table_tools import MultiChainCalibration


def _param_std_dev(result, n_measured_residuals: int) -> Optional[np.ndarray]:
    """Same linearized-uncertainty formula as
    figaroh.calibration.base_calibration.BaseCalibration.calc_stddev:
    sigma_ro_sq = cost^2 / dof, C = sigma_ro_sq * pinv(J^T J), std_dev =
    sqrt(diag(C)). ``n_measured_residuals`` is the residual count from the
    physical measurement only (excluding the regularization tail, whose
    rows exactly cancel the usual "- nvars" term in core's own divisor --
    see this module's docstring in the git history / PR description for
    the derivation).
    """
    if result.jac is None:
        return None
    nvars = len(result.x)
    dof = n_measured_residuals - nvars
    if dof <= 0:
        return None
    try:
        sigma_ro_sq = (result.cost**2) / dof
        C_param = sigma_ro_sq * np.linalg.pinv(result.jac.T @ result.jac)
        return np.sqrt(np.clip(np.diag(C_param), 0.0, None))
    except np.linalg.LinAlgError:
        return None


def _condition_number(result) -> Optional[float]:
    if result.jac is None:
        return None
    try:
        return float(np.linalg.cond(result.jac))
    except np.linalg.LinAlgError:
        return None


def _param_unit(name: str) -> str:
    if "phi" in name or "theta" in name:
        return "rad"
    return "m"


def build_two_chain_report(
    coupler: MultiChainCalibration,
    result,
    train_before: dict,
    train_after: dict,
    val_before: Optional[dict] = None,
    val_after: Optional[dict] = None,
    n_train_left: Optional[int] = None,
    n_train_right: Optional[int] = None,
    n_val_left: Optional[int] = None,
    n_val_right: Optional[int] = None,
) -> dict:
    """Build the report dict for one MultiChainCalibration solve."""
    n_measured = 3 * (
        (n_train_left or coupler.left._fk_config["NbSample"])
        + (n_train_right or coupler.right._fk_config["NbSample"])
    )
    std_dev = _param_std_dev(result, n_measured)

    def _side(before, after):
        return {
            "z_rmse_mm": {"before": before["z_rmse_mm"], "after": after["z_rmse_mm"]},
            "roll_rmse_deg": {
                "before": before["roll_rmse_deg"],
                "after": after["roll_rmse_deg"],
            },
            "pitch_rmse_deg": {
                "before": before["pitch_rmse_deg"],
                "after": after["pitch_rmse_deg"],
            },
        }

    metadata = {
        "left_chain": f"{coupler.left.calib_config['start_frame']} -> {coupler.left.calib_config['end_frame']}",
        "right_chain": f"{coupler.right.calib_config['start_frame']} -> {coupler.right.calib_config['end_frame']}",
        "n_left_delta_x": coupler.left.n_deltaX,
        "n_right_delta_x": coupler.right.n_deltaX,
        "n_union_delta_x": coupler.n_dx,
        "shared_delta_x_names": list(coupler.shared_dx_names),
        "n_training_samples": {"left": n_train_left, "right": n_train_right},
        "training": {
            "left": _side(train_before["left"], train_after["left"]),
            "right": _side(train_before["right"], train_after["right"]),
        },
        "optimization_success": bool(result.success),
        "cost": float(result.cost),
        "n_function_evals": int(getattr(result, "nfev", 0)),
        "condition_number": _condition_number(result),
    }
    if val_before is not None and val_after is not None:
        metadata["n_validation_samples"] = {"left": n_val_left, "right": n_val_right}
        metadata["validation"] = {
            "left": _side(val_before["left"], val_after["left"]),
            "right": _side(val_before["right"], val_after["right"]),
        }

    calibrated_parameters = {
        name: {
            "value": float(value),
            "std_dev": float(std_dev[i]) if std_dev is not None else None,
            "unit": _param_unit(name),
        }
        for i, (name, value) in enumerate(zip(coupler.param_name, result.x))
    }

    return {"metadata": metadata, "calibrated_parameters": calibrated_parameters}


def write_yaml_report(report: dict, output_path: str) -> None:
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w") as f:
        yaml.dump(report, f, sort_keys=False, default_flow_style=False)
    print(f"Calibration results written to:\n  {output_path}")


# ---------------------------------------------------------------------------
# HTML summary
# ---------------------------------------------------------------------------

_HTML_TEMPLATE = """<!doctype html>
<html><head><meta charset="utf-8"><title>{title}</title>
<style>
  body {{ font-family: -apple-system, "Segoe UI", sans-serif; max-width: 900px;
         margin: 40px auto; padding: 0 20px; color: #1a1a1a; }}
  h1 {{ font-size: 22px; border-bottom: 2px solid #2a4d69; padding-bottom: 8px; }}
  h2 {{ font-size: 16px; margin-top: 28px; color: #2a4d69; }}
  table {{ border-collapse: collapse; width: 100%; margin: 10px 0 20px; font-size: 13px; }}
  th, td {{ border: 1px solid #ddd; padding: 6px 10px; text-align: right; }}
  th {{ background: #2a4d69; color: white; text-align: center; }}
  td:first-child, th:first-child {{ text-align: left; }}
  .ok {{ color: #1a7a3c; font-weight: 600; }}
  .fail {{ color: #b3261e; font-weight: 600; }}
  .meta {{ color: #555; font-size: 13px; }}
  code {{ background: #f0f0f0; padding: 1px 5px; border-radius: 3px; font-size: 12px; }}
</style></head>
<body>
<h1>{title}</h1>
<p class="meta">{subtitle}</p>
{body}
</body></html>
"""


def _metric_table(rows, headers=("DOF", "Before", "After", "Reduction")) -> str:
    out = ["<table><tr>" + "".join(f"<th>{h}</th>" for h in headers) + "</tr>"]
    for name, before, after, unit in rows:
        factor = before / after if after > 1e-9 else float("inf")
        out.append(
            f"<tr><td>{name}</td><td>{before:.4f} {unit}</td>"
            f"<td>{after:.4f} {unit}</td><td>{factor:.1f}x</td></tr>"
        )
    out.append("</table>")
    return "".join(out)


def _params_table(calibrated_parameters: dict) -> str:
    out = [
        "<table><tr><th>Parameter</th><th>Value</th><th>Std dev</th><th>Unit</th></tr>"
    ]
    for name, p in calibrated_parameters.items():
        val = p["value"]
        std = p["std_dev"]
        unit = p["unit"]
        if unit == "rad":
            val_s = f"{np.rad2deg(val):+.4f} deg"
            std_s = f"{np.rad2deg(std):.4f} deg" if std is not None else "n/a"
        else:
            val_s = f"{val * 1000:+.3f} mm"
            std_s = f"{std * 1000:.3f} mm" if std is not None else "n/a"
        out.append(
            f"<tr><td><code>{name}</code></td><td>{val_s}</td><td>{std_s}</td><td>{unit}</td></tr>"
        )
    out.append("</table>")
    return "".join(out)


def write_html_report(report: dict, output_path: str, title: str) -> None:
    meta = report["metadata"]
    ok = meta.get("optimization_success", False)
    status = (
        '<span class="ok">converged</span>'
        if ok
        else '<span class="fail">did NOT converge</span>'
    )
    body_parts = [
        f"<p>Solve status: {status} &mdash; cost={meta.get('cost'):.3e}, "
        f"nfev={meta.get('n_function_evals')}, "
        f"condition number={meta.get('condition_number')}</p>"
    ]

    if "training" in meta:  # two-chain: left/right nested
        for side in ("left", "right"):
            t = meta["training"][side]
            body_parts.append(f"<h2>{side.title()} chain &mdash; training-set gap</h2>")
            body_parts.append(
                _metric_table(
                    [
                        ("z", t["z_rmse_mm"]["before"], t["z_rmse_mm"]["after"], "mm"),
                        (
                            "roll",
                            t["roll_rmse_deg"]["before"],
                            t["roll_rmse_deg"]["after"],
                            "deg",
                        ),
                        (
                            "pitch",
                            t["pitch_rmse_deg"]["before"],
                            t["pitch_rmse_deg"]["after"],
                            "deg",
                        ),
                    ]
                )
            )
            if "validation" in meta:
                v = meta["validation"][side]
                body_parts.append(
                    f"<h2>{side.title()} chain &mdash; held-out validation gap</h2>"
                )
                body_parts.append(
                    _metric_table(
                        [
                            (
                                "z",
                                v["z_rmse_mm"]["before"],
                                v["z_rmse_mm"]["after"],
                                "mm",
                            ),
                            (
                                "roll",
                                v["roll_rmse_deg"]["before"],
                                v["roll_rmse_deg"]["after"],
                                "deg",
                            ),
                            (
                                "pitch",
                                v["pitch_rmse_deg"]["before"],
                                v["pitch_rmse_deg"]["after"],
                                "deg",
                            ),
                        ]
                    )
                )

    body_parts.append("<h2>Calibrated parameters</h2>")
    body_parts.append(_params_table(report["calibrated_parameters"]))

    subtitle_bits = []
    for key in ("left_chain", "right_chain"):
        if key in meta:
            subtitle_bits.append(f"{key}: {meta[key]}")
    subtitle = " &nbsp;|&nbsp; ".join(subtitle_bits)

    html = _HTML_TEMPLATE.format(
        title=title, subtitle=subtitle, body="".join(body_parts)
    )
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w") as f:
        f.write(html)
    print(f"HTML report written to:\n  {output_path}")
