#!/usr/bin/env python3
"""
Tiago Pro geometric calibration using Figaroh's BaseCalibration framework.

Migrated from a from-scratch script (see run_calibration_legacy.py) onto
figaroh-plus's BaseCalibration (v0.4.7+), which now natively provides what
that script hand-rolled: SE3 log-map residuals (geometrically correct,
no RPY-wraparound bug -- see calibration_reliability_20260805.md for the
bug that motivated this migration), per-parameter standard errors
(calc_stddev), parameter-correlation flagging, held-out validation support,
and a terminal/HTML quality report.

This subclass only adds what's genuinely TIAGo-Pro-specific on top of that:
- cost_function(): light L2 regularization on per-joint DH-style offsets
  only (not on base_*/pEE*/phiEE*) -- see _joint_param_mask().
- solve_optimisation(): seeds base_pose/tip_pose from config as the LM
  initial guess (BaseCalibration always starts from zero otherwise; a
  ~14cm known offset converges far more reliably from a good seed -- see
  tiago_pro_calibration_config.yaml's comments for where that number comes
  from).

Usage:
    python3 run_calibration.py --urdf tiago_pro_local.urdf
    python3 run_calibration.py --urdf tiago_pro_local.urdf \\
        --data data/calibration_samples.csv \\
        --output data/calibration_results.yaml
"""

import argparse
from pathlib import Path

import numpy as np
import pinocchio as pin
import yaml

from figaroh.calibration.base_calibration import BaseCalibration
from figaroh.calibration.calibration_tools import calc_updated_fkm, initialize_variables
from figaroh.calibration.parameter import BASE_TPL, EE_TPL

_HERE         = Path(__file__).parent
_CONFIG       = _HERE / "tiago_pro_calibration_config.yaml"
_DATA_DEFAULT = _HERE / "data" / "calibration_samples.csv"
_OUT_DEFAULT  = _HERE / "data" / "calibration_results.yaml"


class _Robot:
    def __init__(self, m):
        self.model = m
        self.data  = m.createData()
        self.q0    = pin.neutral(m)


class TiagoProCalibration(BaseCalibration):
    """TIAGo Pro right-arm geometric calibration on the geppetto_vwbc/figaroh
    stack. See the module docstring for what this overrides vs. inherits."""

    def __init__(self, robot, config_file: str, data_path: str = None):
        super().__init__(robot, config_file)
        if data_path is not None:
            # Override the data_file the YAML points to (CLI --data).
            self.calib_config["data_file"] = str(data_path)
            self._data_path = str(Path(data_path).resolve())

        # NOT read from the YAML by figaroh-plus (neither parser populates
        # these) -- co-estimate both the mocap->universe transform and the
        # marker mount rather than treating either as known. See the
        # matching comment in tiago_pro_calibration_config.yaml.
        self.calib_config["known_baseframe"] = False
        self.calib_config["known_tipframe"] = False

    # ── Cost function ────────────────────────────────────────────────────────

    def cost_function(self, var: np.ndarray) -> np.ndarray:
        """SE3 log-map residuals (BaseCalibration's real fix vs. the legacy
        script's RPY-wraparound patch) + light L2 regularization on
        per-joint offsets only.

        Deliberately does NOT call apply_measurement_weighting(): its
        default position/orientation weights (1000x / 100x, from
        measurement_std defaults of 1mm / 0.01rad) blow up the scale
        mismatch against the (still raw-scale) regularization term enough
        to break scipy's method="lm" -- confirmed by hand: with the
        weighting on, `least_squares` hits `xtol` after ~10 evaluations
        with first-order optimality ~5e5 (nowhere near a real optimum,
        MINPACK's internal scaling failing on the ~1e5 dynamic range
        between weighted-residual and regularization column norms); with
        it off, the same problem reaches `ftol` genuinely (optimality
        ~2e-5) and a materially better fit. Raw (unweighted) log-map
        residuals are already well-scaled for position (meters) vs.
        orientation (radians) here -- see the RMSE/orientation-RMSE split
        reported separately in write_calibration_results() instead of a
        single mixed-unit number.

        Regularization is deliberately NOT applied to base_*/pEE*/phiEE*:
        those represent a physical mocap-frame offset and marker mounting
        position, not a redundant joint-placement direction sharing rank
        with a neighbour -- pulling them toward 0 would bias a real,
        non-zero physical quantity for no identifiability benefit.
        """
        PEEe = calc_updated_fkm(
            self.model, self.data, var, self.q_measured, self.calib_config
        )
        residuals = self._compute_logmap_residuals(
            self.PEE_measured, PEEe, position_frame="world"
        )

        coeff = self.calib_config.get("coeff_regularize") or 0.0
        if coeff:
            reg_mask = self._joint_param_mask()
            reg = np.sqrt(coeff) * var[reg_mask]
            return np.concatenate([residuals, reg])
        return residuals

    def _joint_param_mask(self) -> np.ndarray:
        """Boolean mask over calib_config['param_name'], True for per-joint
        DH-style offsets (d_px_arm_right_2_joint, ...), False for base_*/
        pEE*/phiEE* -- see cost_function()'s docstring for why those are
        excluded."""
        names = self.calib_config["param_name"]
        return np.array([n.startswith("d_") for n in names])

    # ── Initial guess ────────────────────────────────────────────────────────

    def solve_optimisation(self, var_init: np.ndarray = None, **kwargs):
        if var_init is None:
            var_init, _ = initialize_variables(self.calib_config, mode=0)
            var_init = self._seed_initial_guess(var_init)
        return super().solve_optimisation(var_init=var_init, **kwargs)

    def _seed_initial_guess(self, var_init: np.ndarray) -> np.ndarray:
        """Seed base_pose/tip_pose from the YAML config into var_init, by
        parameter name (robust to base/pEE ending up at different indices
        than a fixed positional slice would assume)."""
        names = self.calib_config["param_name"]
        var_init = var_init.copy()

        base_pose = self.calib_config.get("base_pose")
        if base_pose is not None:
            for tpl_name, value in zip(BASE_TPL, base_pose):
                if tpl_name in names:
                    var_init[names.index(tpl_name)] = value

        tip_pose = self.calib_config.get("tip_pose")
        if tip_pose is not None:
            for tpl_name, value in zip(EE_TPL, tip_pose):
                pname = f"{tpl_name}_1"  # marker 1 -- NbMarkers == 1 here
                if pname in names:
                    var_init[names.index(pname)] = value
        return var_init


# ── Output ────────────────────────────────────────────────────────────────────

def write_calibration_results(calib: TiagoProCalibration, output_path: str) -> None:
    assert calib.STATUS == "CALIBRATED"

    def _is_rotation_param(name: str) -> bool:
        return "phi" in name

    calc_stddev = getattr(calib, "std_dev", None)
    names = calib.calib_config["param_name"]

    # NOTE: evaluation_metrics["rmse"]/["mae"] combine position (m) and
    # orientation (rad) into one per-sample Euclidean norm before scaling --
    # not a physically meaningful single "mm" number (same class of mixed-
    # unit issue our old wrap-based residual had). Use the position/
    # orientation split BaseCalibration already computes separately instead.
    overall = calib.evaluation_metrics["per_dof_stats"]["overall"]

    results = {
        "metadata": {
            "position_rmse_mm":    round(overall["pos_rmse_mm"], 3),
            "position_mae_mm":     round(overall["pos_mae_mm"], 3),
            "orientation_rmse_deg": round(overall["orient_rmse_deg"], 4),
            "orientation_mae_deg":  round(overall["orient_mae_deg"], 4),
            "n_samples":    calib.calib_config["NbSample"],
            "n_outliers":   len(calib.outlier_indices),
            "condition_number": calib.evaluation_metrics.get("condition_number"),
        },
        "calibrated_parameters": {
            name: {
                "value": float(value),
                "std_dev": (
                    float(calc_stddev[i]) if calc_stddev is not None else None
                ),
                "unit": "rad" if _is_rotation_param(name) else "m",
            }
            for i, (name, value) in enumerate(zip(names, calib.var_))
        },
    }

    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w") as f:
        yaml.dump(results, f, sort_keys=False, default_flow_style=False)
    print(f"\nCalibration results written to:\n  {output_path}")


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Run Figaroh geometric calibration for Tiago Pro right arm."
    )
    parser.add_argument("--urdf",   required=True,          help="Path to the Tiago Pro URDF")
    parser.add_argument("--data",   default=str(_DATA_DEFAULT))
    parser.add_argument("--config", default=str(_CONFIG))
    parser.add_argument("--output", default=str(_OUT_DEFAULT))
    parser.add_argument("--html-report", action="store_true",
                         help="Also export an HTML diagnostic report next to --output.")
    args = parser.parse_args()

    print(f"Loading robot from {args.urdf} ...")
    robot = _Robot(pin.buildModelFromUrdf(args.urdf))

    calib = TiagoProCalibration(robot, args.config, data_path=args.data)
    calib.initialize()
    # NOTE: BaseCalibration's outlier-removal loop (as of figaroh-plus 0.4.7)
    # detects outliers each iteration but doesn't actually exclude them from
    # PEE_measured/q_measured before the next re-fit -- it re-solves the same
    # problem from the previous solution and reports the same outlier every
    # time, up to max_iterations. Harmless (each pass is near-instant once
    # already converged) but doesn't do more with a higher budget, so we
    # don't ask for one.
    calib.solve(
        max_iterations=3,
        outlier_threshold=3.0,
        html_report=args.html_report,
    )

    write_calibration_results(calib, args.output)


if __name__ == "__main__":
    main()
