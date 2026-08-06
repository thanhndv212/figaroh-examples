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

TiagoProCalibration subclasses figaroh.calibration.base_calibration.
BaseCalibration, following the same pattern as TiagoCalibration
(examples/tiago/utils/tiago_tools.py). It used to be a self-contained class
driving figaroh.calibration.calibration_tools directly, kept separate from
BaseCalibration because it was what had been validated against real
hardware data (RMSE 6.46 mm, MAE 4.94 mm, 94 samples — see
data/calibration_results_20260702_0756.yaml in the example directory).
Migrated onto BaseCalibration so it inherits the same ongoing
fixes/reporting infrastructure TiagoCalibration gets (SE3 log-map residual
machinery, outlier removal, condition number / parameter uncertainty /
correlation reporting, provenance + archive). Numerically re-verified after
migration: RMSE 6.464 mm / MAE 4.949 mm (vs. 6.457 / 4.948 pre-migration),
condition number 264.5 (vs. 254.2), same dominant correlated-pair structure.
Close but not bit-identical, as expected — BaseCalibration's inherited
outlier-removal loop calls `least_squares` with different options
(`max_nfev` cap, no `verbose`) than the original bespoke loop did, so it
follows a slightly different convergence path to what is otherwise the same
optimum.

One thing is genuinely special about this robot/dataset and needs a real
override beyond what TiagoCalibration needs — everything else (data
loading, condition number, parameter uncertainty/correlation, quality
report, HTML export, provenance/archive) comes from BaseCalibration
unchanged: ``gripper_right_tool_mount_joint`` (the PAL ATC tool-changer
coupler) can be absent from a cleaned data CSV — it's a constant-zero
column (never exercised during data collection), not real measured data.
See ``_pad_csv_missing_joints``.

History: ``pEEx_1``/``pEEy_1`` (marker offset in the gripper plane) used to
be fixed at 0 (``_fixed_tip_xy``), because under the OLD config
(``tool_frame: gripper_right_tool_holder``, position-only ``measure``) they
were structurally unobservable — ``gripper_right_tool_mount_joint`` sits
between ``tool_frame`` and the marker, is never exercised in the data
(always recorded at q=0), and a marker offset in that plane was
indistinguishable from an unmeasured rotation of that joint (confirmed by
SVD: singular value ~1e-16, eigenvector ~100% pEEx_1/pEEy_1, zero effect on
any other parameter).

RESOLVED 2026-08-06: the config now sets ``tool_frame:
gripper_right_pal_atc_base_link`` (before ``gripper_right_tool_mount_joint``
in the kinematic tree, so that joint no longer appears in the calibrated
chain at all) and measures full pose (``phix1``/``phiy1``/``phiz1`` are now
in the data, not just position). Re-checked by SVD of the residual Jacobian
at the solution on real data (48 samples): pEEx_1/pEEy_1's weight in the
smallest singular direction is ~1e-21 (vs ~1.0 before), well-conditioned —
free to optimize now. ``_fixed_tip_xy`` defaults to ``False``; the
``_fixed_idx``/``_free_idx``/reduced-space-optimization machinery below is
kept (rather than deleted) so a future position-only config still works
unchanged, it just becomes a no-op when there's nothing to fix.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pinocchio as pin
import yaml

from figaroh.calibration.base_calibration import BaseCalibration
from figaroh.calibration.calibration_tools import calc_updated_fkm


def _is_rotation_param(name: str) -> bool:
    """True for phi* (rotation) params; False for p*/pEE* (translation) params."""
    return "phi" in name


class _Robot:
    """Minimal Pinocchio model/data wrapper -- kinematics only, no mesh/geometry
    loading (calibration never needs meshes). ``robot_urdf`` satisfies
    figaroh.tools.provenance's optional robot-path lookup."""

    def __init__(self, m, urdf_path: str = ""):
        self.model = m
        self.data = m.createData()
        self.q0 = pin.neutral(m)
        self.robot_urdf = urdf_path


# ── Calibration ──────────────────────────────────────────────────────────────


class TiagoProCalibration(BaseCalibration):
    """TIAGo Pro right-arm geometric calibration."""

    def __init__(self, robot, config_file: str, del_list: list | None = None):
        super().__init__(robot, config_file, del_list or [])
        self.robot_name = "tiago_pro"
        self._fixed_tip_xy = False  # see "History"/"RESOLVED" in module docstring
        self._fixed_idx: list[int] = []  # populated in initialize()

    def initialize(self) -> None:
        self._pad_csv_missing_joints()  # before load_data_set() reads self._data_path
        super().initialize()  # load_data_set() then create_param_list()
        self._rename_base_params()
        n_params = len(self.calib_config["param_name"])
        if self._fixed_tip_xy:
            names = self.calib_config["param_name"]
            self._fixed_idx = [names.index("pEEx_1"), names.index("pEEy_1")]
        self._free_idx = [i for i in range(n_params) if i not in self._fixed_idx]

    def _rename_base_params(self) -> None:
        """Relabel the 6 base_* params inserted by add_base_name() (called
        inside create_param_list()).

        add_base_name() overwrites param_name[0:6] positionally with
        base_px/py/pz/phix/phiy/phiz. Those 6 slots are, before the overwrite,
        exactly torso_lift_joint's own DH offset params (d_px/py/pz/phix/phiy/phiz
        _torso_lift_joint) -- torso_lift_joint is the first active joint in the
        chain, right after universe, so its own placement error is mathematically
        indistinguishable from the unknown base-marker-to-robot transform being
        co-estimated (known_baseframe=False). Appending _torso makes that merge
        explicit instead of implying they're purely a mocap frame offset.

        Must keep "base_px"/"base_phix"/etc. as an exact substring (suffix, not
        prefix, insertion) — figaroh's calc_updated_fkm matches param names via
        `base_ax in key` against BASE_TPL, so anything else breaks the update.
        """
        for i in range(6):
            self.calib_config["param_name"][i] = (
                self.calib_config["param_name"][i] + "_torso"
            )

    def _pad_csv_missing_joints(self) -> None:
        import pandas as pd, tempfile, os

        joint_headers = [
            self.model.names[i] for i in self.calib_config["actJoint_idx"]
        ]
        df = pd.read_csv(self._data_path)
        added = [j for j in joint_headers if j not in df.columns]
        if not added:
            return
        for j in added:
            print(f"[info] Column '{j}' missing from CSV — padding with 0.0")
            df[j] = 0.0
        # Padded copy goes to a temp file, not back onto the source CSV --
        # a joint missing from the log (e.g. a fixed frame like
        # gripper_right_tool_mount_joint) should stay absent from the
        # checked-in data rather than get silently re-added on every run.
        fd, tmp_path = tempfile.mkstemp(
            suffix=".csv", prefix="tiago_pro_calib_padded_"
        )
        os.close(fd)
        df.to_csv(tmp_path, index=False)
        self._data_path = tmp_path

    def cost_function(self, var: np.ndarray) -> np.ndarray:
        full = var.copy()
        if self._fixed_idx:
            full[self._fixed_idx] = 0.0
        coeff = self.calib_config.get("coeff_regularize") or 0.01
        PEEe = calc_updated_fkm(
            self.model, self.data, full, self.q_measured, self.calib_config
        )
        # world-frame position residuals -- matches the pre-migration
        # convention (plain PEE_measured - PEEe); this is what the RMSE/MAE
        # this class was validated against were computed from.
        raw_residuals = self._compute_logmap_residuals(
            self.PEE_measured, PEEe, position_frame="world"
        )
        ci = self.calib_config["calibration_index"]
        n_base = 6  # base frame DOF (free when known_baseframe=False)
        n_tip = self.calib_config["NbMarkers"] * ci
        regularization = np.sqrt(coeff) * full[n_base:-n_tip]
        return np.append(raw_residuals, regularization)

    def get_pose_from_measure(self, res_: np.ndarray) -> np.ndarray:
        """As BaseCalibration, but if ``_fixed_tip_xy`` is set, pEEx_1/pEEy_1
        are always evaluated at 0 -- see class docstring. Keeps every
        BaseCalibration consumer of this method (outlier detection,
        evaluation, reporting, export) consistent with what cost_function()
        used during optimization, regardless of whatever the optimizer
        nominally left in result.x at those two positions (their Jacobian
        columns are exactly zero, so LM shouldn't move them from their zero
        starting value -- this is belt-and-braces). A no-op when
        ``_fixed_idx`` is empty (the current default)."""
        full = res_.copy()
        if self._fixed_idx:
            full[self._fixed_idx] = 0.0
        return super().get_pose_from_measure(full)

    def _detect_outliers(self, residuals: np.ndarray, threshold: float) -> list:
        """TIAGo Pro uses fixed absolute-distance cutoffs (``outlier_eps``
        meters for position, ``outlier_eps_deg`` degrees for orientation, in
        the config -- defaults 5cm / 10deg) instead of BaseCalibration's
        adaptive statistical (mean + k*std) threshold -- domain knowledge
        that error beyond either is unambiguous mistracking, not a
        fit-quality-relative judgement. ``threshold`` (in std-devs) is
        accepted for interface compatibility but unused.

        Position and orientation are evaluated *separately* (not folded into
        one combined Euclidean norm across mismatched units) -- residuals_2d
        rows 0:3 are position (m), rows 3:6 orientation (rad, from
        BaseCalibration's SE3 log map), same convention as
        BaseCalibration._compute_per_dof_stats' "overall" block. Flagging a
        sample if *either* exceeds its own threshold matters here
        specifically because outlier removal feeds back into what data the
        fit uses -- for position-only calibration (calibration_index == 3)
        this reduces to the original single position check.
        """
        ci = self.calib_config["calibration_index"]
        n_samples = self.calib_config["NbSample"]
        if len(residuals) != ci * n_samples:
            return []
        residuals_2d = residuals.reshape((ci, n_samples))
        eps_pos = self.calib_config.get("outlier_eps", 0.05)
        pos_rows = residuals_2d[:3, :]
        dist_pos = np.sqrt(np.sum(pos_rows**2, axis=0))
        outliers = set(np.where(dist_pos > eps_pos)[0].tolist())
        if ci >= 6:
            eps_orient = np.radians(self.calib_config.get("outlier_eps_deg", 10.0))
            orient_rows = residuals_2d[3:6, :]
            dist_orient = np.sqrt(np.sum(orient_rows**2, axis=0))
            outliers.update(np.where(dist_orient > eps_orient)[0].tolist())
        return sorted(outliers)

    def _expand(self, var_free: np.ndarray) -> np.ndarray:
        """Map a reduced (free-parameters-only) vector back to full length,
        with pEEx_1/pEEy_1 at 0."""
        full = np.zeros(len(self.calib_config["param_name"]))
        full[self._free_idx] = var_free
        return full

    def _optimize_with_outlier_removal(
        self,
        var_init: np.ndarray,
        method: str = "lm",
        max_iterations: int = 3,
        outlier_threshold: float = 1.0,
    ):
        """As BaseCalibration, but the LM search itself runs over only the
        free parameters (excluding pEEx_1/pEEy_1), not the full vector with
        those two clobbered to 0 inside cost_function(). That distinction
        matters beyond the fitted values: passing the full-length vector to
        `least_squares` gives pEEx_1/pEEy_1 two Jacobian columns that are
        *supposed* to be exactly zero but come out as finite-difference noise
        instead (not exactly 0.0), which wrecks np.linalg.cond()/pinv() for
        every other parameter too (a near-zero-but-nonzero singular value
        gets inverted as huge instead of truncated as negligible). Running
        the search in the reduced space avoids that; `result.x`/`result.jac`
        are expanded back to full length/width before returning so the rest
        of BaseCalibration (which assumes result.x aligns 1:1 with
        calib_config["param_name"]) sees the shapes it expects, with
        pEEx_1/pEEy_1 as an exact (not noisy) 0.0 value and 0.0 Jacobian
        column.
        """
        import logging

        from scipy.optimize import least_squares

        logger = logging.getLogger("calibration")
        n_full = len(var_init)
        current_free = np.asarray(var_init)[self._free_idx].copy()
        outlier_indices: list = []
        result = None
        residuals = None

        for iteration in range(max_iterations):
            logger.info(f"Outlier removal iteration {iteration + 1}")

            result = least_squares(
                lambda var_free: self.cost_function(self._expand(var_free)),
                current_free,
                method=method,
                max_nfev=1000,
            )
            if not result.success:
                logger.warning(f"Optimization failed at iteration {iteration + 1}")
                break

            # Expand back to full length/width for every downstream consumer.
            full_jac = np.zeros((result.jac.shape[0], n_full))
            full_jac[:, self._free_idx] = result.jac
            result.x = self._expand(result.x)
            result.jac = full_jac

            PEE_est = self.get_pose_from_measure(result.x)
            residuals = self._compute_logmap_residuals(self.PEE_measured, PEE_est)
            new_outliers = self._detect_outliers(residuals, outlier_threshold)

            if len(new_outliers) == 0:
                logger.info("No outliers detected, optimization converged")
                break

            outlier_indices.extend(new_outliers)
            outlier_indices = list(set(outlier_indices))
            logger.info(
                f"Detected {len(new_outliers)} new outliers, "
                f"total outliers: {len(outlier_indices)}"
            )
            current_free = result.x[self._free_idx]

        return result, outlier_indices, residuals

    def _compute_condition_number(self, result):
        """As BaseCalibration, but computed over only the free-parameter
        columns of the Jacobian -- pEEx_1/pEEy_1 are exactly 0 by
        construction (see _optimize_with_outlier_removal), not parameters
        actually being fit, so including their all-zero columns would make
        every condition number report as infinite/nonsensical rather than
        reflecting the conditioning of the 39 parameters that are really
        free."""
        try:
            J = result.jac
            if J is None:
                return float("nan"), "unavailable (no Jacobian)"
            J_free = J[:, self._free_idx] if self._free_idx else J
            cond_num = float(np.linalg.cond(J_free))
            if cond_num < 100:
                label = "well-conditioned"
            elif cond_num < 1000:
                label = "moderately conditioned"
            else:
                label = "ill-conditioned"
            return cond_num, label
        except Exception:
            return float("nan"), "unavailable (computation failed)"


# ── Output ───────────────────────────────────────────────────────────────────


def write_calibration_results(calib: TiagoProCalibration, output_path: str) -> None:
    assert calib.STATUS == "CALIBRATED"

    param_names = calib.calib_config["param_name"]
    metrics = calib.evaluation_metrics
    # Position (mm) / orientation (deg) computed and aggregated separately
    # -- same convention as BaseCalibration._compute_per_dof_stats' "overall"
    # block. metrics["rmse"]/["mae"] fold both into one Euclidean norm across
    # mismatched units (m and rad); kept below only for backward
    # compatibility with older results files, not as the primary figure.
    overall = metrics["per_dof_stats"]["overall"]

    results = {
        "metadata": {
            "rmse_mm": round(overall["pos_rmse_mm"], 3),
            "mae_mm": round(overall["pos_mae_mm"], 3),
            "rmse_orient_deg": round(overall["orient_rmse_deg"], 4),
            "mae_orient_deg": round(overall["orient_mae_deg"], 4),
            "rmse_combined": round(float(metrics["rmse"]), 6),
            "mae_combined": round(float(metrics["mae"]), 6),
        },
        "calibrated_parameters": {
            name: {
                "value": float(value),
                "unit": "rad" if _is_rotation_param(name) else "m",
            }
            for name, value in zip(param_names, calib.var_)
        },
    }

    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w") as f:
        yaml.dump(results, f, sort_keys=False, default_flow_style=False)
    print(f"\nCalibration results written to:\n  {output_path}")
