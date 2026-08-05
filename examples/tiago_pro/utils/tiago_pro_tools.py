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
TIAGo Pro geometric calibration tools.

TiagoProCalibration predates figaroh.calibration.base_calibration and
drives figaroh.calibration.calibration_tools directly rather than through
BaseCalibration. It's kept self-contained rather than rewritten to inherit
from BaseCalibration because it's what was validated against real
hardware data (RMSE 6.46 mm, MAE 4.94 mm, 94 samples — see
data/calibration_results_20260702_0756.yaml in the example directory).

The reporting helpers below (_compute_condition_number,
_compute_per_dof_stats, _compute_uncertainty, _prepare_report) reimplement
the same formulas BaseCalibration uses for the same quality metrics
(condition number, parameter uncertainty, correlation, per-DOF stats) as
free functions, then attach their output to the calib instance so
figaroh's actual report/provenance/archive code (unmodified) can consume
it exactly as it would a BaseCalibration instance (duck-typed, not
isinstance-checked).
"""

from __future__ import annotations

import types
from pathlib import Path

import numpy as np
import pinocchio as pin
import yaml

from figaroh.calibration.calibration_tools import (
    get_param_from_yaml,
    add_base_name,
    add_pee_name,
    load_data,
    calculate_base_kinematics_regressor,
    calc_updated_fkm,
    initialize_variables,
)
from figaroh.tools.provenance import collect_run_provenance
from figaroh.tools.run_archive import compute_run_dir
from scipy.optimize import least_squares


def _is_rotation_param(name: str) -> bool:
    """True for phi* (rotation) params; False for p*/pEE* (translation) params."""
    return "phi" in name


class _Robot:
    def __init__(self, m):
        self.model = m
        self.data = m.createData()
        self.q0 = pin.neutral(m)


# ── Calibration ──────────────────────────────────────────────────────────────


class TiagoProCalibration:

    def __init__(self, robot: _Robot, config_path: str, data_path: str):
        self._robot = robot
        self.model = robot.model
        self.data = robot.data

        with open(config_path) as f:
            config = yaml.safe_load(f)

        self.param = get_param_from_yaml(robot, config["calibration"])
        self.param["known_baseframe"] = False  # co-estimate mocap->universe transform
        self.param["known_tipframe"] = False  # estimate marker pos rel. to gripper
        self.param["data_file"] = data_path

        self._data_path = data_path
        self.STATUS = "NOT CALIBRATED"
        self.calibrated_param: dict = {}

        # pEEx_1/pEEy_1 (marker offset in the gripper plane) are structurally
        # unobservable: gripper_right_tool_mount_joint is continuous but always
        # recorded at q=0, so no sample varies it, and a marker offset in that
        # plane is indistinguishable from an unmeasured rotation of that joint
        # (confirmed by SVD: singular value ~1e-16, eigenvector ~100% pEEx_1/pEEy_1,
        # with zero effect on the other 39 params whether fixed or free).
        # We fix them at 0 per the mocap marker being mounted at the EE frame
        # origin by design — this is a design assumption, not a measured value,
        # since it can't be verified from this data. Only pEEz_1 is estimated.
        self._fixed_tip_xy = True

    def load_and_check_data(self) -> None:
        calculate_base_kinematics_regressor([], self.model, self.data, self.param)
        add_base_name(
            self.param
        )  # inserts base_px/py/pz/phix/phiy/phiz at positions 0-5
        self._rename_base_params()  # these 6 slots are torso_lift_joint's own DH offsets,
        # not just the mocap->universe transform -- see class docstring comment below
        add_pee_name(self.param)  # appends pEEx_1/pEEy_1/pEEz_1 at the end

    def _rename_base_params(self) -> None:
        """Relabel the 6 base_* params inserted by add_base_name().

        add_base_name() overwrites param_name[0:6] positionally with
        base_px/py/pz/phix/phiy/phiz. Those 6 slots are, before the overwrite,
        exactly torso_lift_joint's own DH offset params (d_px/py/pz/phix/phiy/phiz
        _torso_lift_joint) -- torso_lift_joint is the first active joint in the
        chain, right after universe, so its own placement error is mathematically
        indistinguishable from the unknown mocap->universe transform being
        co-estimated (known_baseframe=False). Appending _torso makes that merge
        explicit instead of implying they're purely a mocap frame offset.

        Must keep "base_px"/"base_phix"/etc. as an exact substring (suffix, not
        prefix, insertion) — figaroh's calc_updated_fkm matches param names via
        `base_ax in key` against BASE_TPL, so anything else breaks the update.
        """
        for i in range(6):
            self.param["param_name"][i] = self.param["param_name"][i] + "_torso"
        self._pad_csv_missing_joints()
        self.PEE_measured, self.q_measured = load_data(
            self._data_path, self.model, self.param, del_list=[]
        )

    def _pad_csv_missing_joints(self) -> None:
        import pandas as pd, tempfile, os

        joint_headers = [self.model.names[i] for i in self.param["actJoint_idx"]]
        df = pd.read_csv(self._data_path)
        added = [j for j in joint_headers if j not in df.columns]
        if not added:
            return
        for j in added:
            print(f"[info] Column '{j}' missing from CSV — padding with 0.0")
            df[j] = 0.0
        tmp = self._data_path + ".tmp"
        df.to_csv(tmp, index=False)
        os.replace(tmp, self._data_path)
        print(f"\nLoaded {self.param['NbSample']} samples.")
        print(f"Parameters to identify: {self.param['param_name']}\n")

    def solve(self, outlier_eps: float = 0.05, max_iter: int = 10) -> None:
        var0, _ = initialize_variables(self.param, mode=0)
        ci = self.param["calibration_index"]
        n_tip = self.param["NbMarkers"] * ci

        # Seed base frame DOF with the measured mocap→base transform
        base_pose = self.param.get("base_pose", None)
        if base_pose is not None:
            base_arr = np.atleast_1d(np.array(base_pose, dtype=float))
            var0[: len(base_arr)] = base_arr[:6]

        # Seed EE marker position with tip_pose initial guess
        tip = self.param.get("tip_pose", None)
        if tip is not None:
            tip_arr = np.atleast_1d(np.array(tip, dtype=float))
            if len(tip_arr) >= ci:
                var0[-ci:] = tip_arr[:ci]

        # pEEx_1/pEEy_1 (last-but-one, last-but-two entries of the tip block)
        # are fixed — see comment in __init__.
        fixed_idx = (
            [len(var0) - n_tip, len(var0) - n_tip + 1] if self._fixed_tip_xy else []
        )
        free_idx = [i for i in range(len(var0)) if i not in fixed_idx]

        def cost_free(var_free):
            full = var0.copy()
            full[free_idx] = var_free
            for i in fixed_idx:
                full[i] = 0.0
            return self._cost(full)

        del_list: list = []
        for iteration in range(max_iter):
            print(f"{'=' * 50}")
            print(f"LM iteration {iteration}")

            result = least_squares(cost_free, var0[free_idx], method="lm", verbose=1)
            var0 = var0.copy()
            var0[free_idx] = result.x
            for i in fixed_idx:
                var0[i] = 0.0
            var_full = var0

            PEEe = calc_updated_fkm(
                self.model, self.data, var_full, self.q_measured, self.param
            )
            residuals = (PEEe - self.PEE_measured).reshape(
                self.param["NbMarkers"] * ci, self.param["NbSample"]
            )
            dist = np.linalg.norm(
                residuals.reshape(-1, 3, self.param["NbSample"]), axis=1
            )
            rmse = float(np.sqrt(np.mean(residuals**2)))
            mae = float(np.mean(np.abs(residuals)))
            print(f"RMSE = {rmse*1000:.2f} mm   MAE = {mae*1000:.2f} mm")

            new_outliers = [
                (i, k)
                for i in range(self.param["NbMarkers"])
                for k in range(self.param["NbSample"])
                if dist[i, k] > outlier_eps
            ]
            if new_outliers:
                print(
                    f"Removing {len(new_outliers)} outliers (>{outlier_eps*1000:.0f} mm)"
                )
                del_list += new_outliers
                self.PEE_measured, self.q_measured = load_data(
                    self._data_path, self.model, self.param, del_list=del_list
                )
                var0 = var_full + np.random.normal(0, 0.005, size=var_full.shape)
                for i in fixed_idx:
                    var0[i] = 0.0
            else:
                break

        self.calibrated_param = dict(zip(self.param["param_name"], var_full.tolist()))
        self.rmse = rmse
        self.mae = mae
        self.STATUS = "CALIBRATED"

        # Stashed for report/archive generation only (_prepare_report) —
        # no effect on the calibration result itself.
        self._last_lm_result = result
        self._free_idx = free_idx
        self._fixed_idx = fixed_idx
        self.outlier_indices = del_list

        print(f"\n{'=' * 50}")
        print("Calibration results:")
        for name, val in self.calibrated_param.items():
            if _is_rotation_param(name):
                print(f"  {name:40s}  {val*1000:+8.3f} mrad  ({np.degrees(val):+.4f}°)")
            else:
                print(f"  {name:40s}  {val*1000:+8.3f} mm")
        print(f"\nFinal RMSE: {rmse*1000:.2f} mm   MAE: {mae*1000:.2f} mm")

    def _cost(self, var: np.ndarray) -> np.ndarray:
        coeff = self.param.get("coeff_regularize") or 0.01
        PEEe = calc_updated_fkm(self.model, self.data, var, self.q_measured, self.param)
        ci = self.param["calibration_index"]
        n_base = 6  # base frame DOF (free when known_baseframe=False)
        n_tip = self.param["NbMarkers"] * ci
        return np.append(
            self.PEE_measured - PEEe,
            np.sqrt(coeff) * var[n_base:-n_tip],
        )


# ── Output ───────────────────────────────────────────────────────────────────


def write_calibration_results(calib: TiagoProCalibration, output_path: str) -> None:
    assert calib.STATUS == "CALIBRATED"

    results = {
        "metadata": {
            "rmse_mm": round(calib.rmse * 1000, 3),
            "mae_mm": round(calib.mae * 1000, 3),
        },
        "calibrated_parameters": {
            name: {
                "value": float(value),
                "unit": "rad" if _is_rotation_param(name) else "m",
            }
            for name, value in calib.calibrated_param.items()
        },
    }

    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w") as f:
        yaml.dump(results, f, sort_keys=False, default_flow_style=False)
    print(f"\nCalibration results written to:\n  {output_path}")


# ── Reporting ────────────────────────────────────────────────────────────────


def _compute_condition_number(result) -> tuple:
    """Condition number of the final LM Jacobian. Same thresholds as
    BaseCalibration._compute_condition_number: <100 well-conditioned,
    <1000 moderately conditioned, else ill-conditioned."""
    try:
        J = result.jac
        if J is None:
            return float("nan"), "unavailable (no Jacobian)"
        cond_num = float(np.linalg.cond(J))
        if cond_num < 100:
            label = "well-conditioned"
        elif cond_num < 1000:
            label = "moderately conditioned"
        else:
            label = "ill-conditioned"
        return cond_num, label
    except Exception:
        return float("nan"), "unavailable (computation failed)"


def _compute_per_dof_stats(residuals, PEE_measured, n_dofs, n_samples) -> dict:
    """Per-DOF residual statistics — same layout as
    BaseCalibration._compute_per_dof_stats. Position-only marker here
    (calibration_index == 3), so dof_names is just X/Y/Z."""
    dof_names = ["X (mm)", "Y (mm)", "Z (mm)", "rx (deg)", "ry (deg)", "rz (deg)"]
    dof_names = dof_names[:n_dofs]

    if len(residuals) != n_dofs * n_samples:
        return {
            "dof_names": dof_names,
            "mean": [],
            "std": [],
            "rmse": [],
            "max_abs": [],
            "r_squared": [],
        }

    residuals_2d = residuals.reshape((n_dofs, n_samples))
    PEE_meas_2d = PEE_measured.reshape((n_dofs, n_samples))

    means, stds, rmses, max_abs, r_squareds = [], [], [], [], []
    for i in range(n_dofs):
        row = residuals_2d[i, :]
        meas_row = PEE_meas_2d[i, :]
        scale = 1000.0 if i < 3 else 180.0 / np.pi
        scaled = row * scale
        means.append(float(np.mean(scaled)))
        stds.append(float(np.std(scaled)))
        rmses.append(float(np.sqrt(np.mean(scaled**2))))
        max_abs.append(float(np.max(np.abs(scaled))))
        ss_res = np.sum(row**2)
        ss_tot = np.sum((meas_row - np.mean(meas_row)) ** 2)
        r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-15 else 1.0
        r_squareds.append(float(r2))

    pos_rows = residuals_2d[:3, :] if n_dofs >= 3 else residuals_2d
    orient_rows = residuals_2d[3:6, :] if n_dofs >= 6 else np.zeros((3, n_samples))
    pos_rmse = float(np.sqrt(np.mean(np.sum(pos_rows**2, axis=0)))) * 1000
    orient_rmse = float(np.sqrt(np.mean(np.sum(orient_rows**2, axis=0)))) * 180 / np.pi
    pos_max = float(np.max(np.sqrt(np.sum(pos_rows**2, axis=0)))) * 1000
    orient_max = float(np.max(np.sqrt(np.sum(orient_rows**2, axis=0)))) * 180 / np.pi

    return {
        "dof_names": dof_names,
        "mean": means,
        "std": stds,
        "rmse": rmses,
        "max_abs": max_abs,
        "r_squared": r_squareds,
        "overall": {
            "pos_rmse_mm": pos_rmse,
            "orient_rmse_deg": orient_rmse,
            "pos_max_mm": pos_max,
            "orient_max_deg": orient_max,
        },
    }


def _compute_uncertainty(result, param_names, free_idx, n_samples, ci) -> tuple:
    """Parameter stdev/%-uncertainty + correlated pairs — same formula as
    BaseCalibration.calc_stddev / _compute_parameter_correlation:
    sigma_ro_sq = cost^2 / (n_residuals - n_free_params); C = sigma_ro_sq
    * pinv(J^T J). ``free_idx`` fixed params (pEEx_1/pEEy_1) get 0.0 —
    they were never part of the fit.
    """
    nvars = len(result.x)
    dof = n_samples * ci - nvars
    sigma_ro_sq = (result.cost**2) / dof if dof > 0 else float("nan")
    J = result.jac
    C_param = sigma_ro_sq * np.linalg.pinv(J.T @ J)

    std_dev_full = [0.0] * len(param_names)
    std_pctg_full = [0.0] * len(param_names)
    for k, idx in enumerate(free_idx):
        sd = float(np.sqrt(C_param[k, k]))
        std_dev_full[idx] = sd
        std_pctg_full[idx] = abs(sd / result.x[k]) if result.x[k] != 0 else 0.0

    correlated_pairs = []
    D = np.sqrt(np.diag(C_param))
    with np.errstate(divide="ignore", invalid="ignore"):
        corr = np.where(np.outer(D, D) > 1e-15, C_param / np.outer(D, D), 0.0)
    for i in range(len(D)):
        for j in range(i + 1, len(D)):
            if abs(corr[i, j]) > 0.8:
                correlated_pairs.append(
                    {
                        "param_i": param_names[free_idx[i]],
                        "param_j": param_names[free_idx[j]],
                        "correlation": float(corr[i, j]),
                    }
                )
    return std_dev_full, std_pctg_full, correlated_pairs


def _prepare_report(
    calib: TiagoProCalibration,
    *,
    config_path: str,
    urdf_path: str,
    asset_id: str | None,
    operator: str | None,
    run_started: str,
    run_finished: str,
):
    """Populate the BaseCalibration-shaped attributes figaroh's report /
    provenance / archive functions read (duck-typed, not isinstance-
    checked — see module docstring above), then return this run's
    archive directory.
    """
    param = calib.param
    param_names = param["param_name"]
    ci = param["calibration_index"]
    n_samples = param["NbSample"]

    var_full = np.array([calib.calibrated_param[name] for name in param_names])
    PEE_est = calc_updated_fkm(
        calib.model, calib.data, var_full, calib.q_measured, calib.param
    )
    residuals = calib.PEE_measured - PEE_est

    if len(residuals) == ci * n_samples:
        residuals_2d = residuals.reshape((ci, n_samples))
        sample_rms = np.sqrt(np.mean(residuals_2d**2, axis=0))
        mean_sample_rms = float(np.mean(sample_rms))
        std_sample_rms = float(np.std(sample_rms))
    else:
        mean_sample_rms = calib.rmse
        std_sample_rms = 0.0

    per_dof_stats = _compute_per_dof_stats(residuals, calib.PEE_measured, ci, n_samples)

    result = calib._last_lm_result
    cond_num, cond_label = _compute_condition_number(result)
    std_dev, std_pctg, correlated_pairs = _compute_uncertainty(
        result, param_names, calib._free_idx, n_samples, ci
    )

    n_outliers = len(calib.outlier_indices)
    evaluation_metrics = {
        "rmse": calib.rmse,
        "mae": calib.mae,
        "max_error": float(np.max(np.abs(residuals))),
        "mean_sample_rms": mean_sample_rms,
        "std_sample_rms": std_sample_rms,
        "param_stdev": std_dev,
        "param_stddev_percentage": std_pctg,
        "n_outliers": n_outliers,
        "outlier_percentage": (n_outliers / n_samples * 100) if n_samples else 0.0,
        "optimization_success": bool(result.success),
        "cost": float(result.cost),
        "n_iterations": getattr(result, "nit", 0),
        "n_function_evals": getattr(result, "nfev", 0),
        "per_dof_stats": per_dof_stats,
        "condition_number": cond_num,
        "condition_label": cond_label,
        "correlated_pairs": correlated_pairs,
    }

    calib.evaluation_metrics = evaluation_metrics
    calib.results_data = {
        "number of calibrated parameters": len(calib._free_idx),
        "calibrated parameters names": param_names,
        "calibrated parameters values": var_full.tolist(),
        "number of samples": n_samples,
        **evaluation_metrics,
    }
    calib.calib_config = param  # alias: report/provenance read `calib_config`
    calib.robot_name = "tiago_pro"
    calib.robot = types.SimpleNamespace(robot_urdf=str(urdf_path))
    calib._config_file_path = str(config_path)
    calib._run_started_at = run_started
    calib._run_finished_at = run_finished

    if asset_id or operator:
        instance = dict(param.get("instance") or {})
        if asset_id:
            instance["asset_id"] = asset_id
        if operator:
            instance["operator"] = operator
        param["instance"] = instance

    calib._run_provenance = collect_run_provenance(calib, "calibration")
    return compute_run_dir(calib)
