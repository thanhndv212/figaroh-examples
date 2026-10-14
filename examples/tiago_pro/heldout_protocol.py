"""TIAGo Pro mocap calibration: frozen held-out protocol (#97).

Fits every model on the training session (2026-08-05, full pose) and reports
the position error on the two held-out sessions (2026-07-01 validation,
2026-07-02 confirmation, position only):

- registration only: nominal joints, base (6D) and tool frame fitted;
- full_params: structural base parameters (the reference configuration);
- full map / map_cv / cv_subset: core's regularised estimation methods.

Roles, files and sha256 are fixed in data/protocol.yaml (read through the
shared examples/calibration_harness; changing a file fails at import) and
described in docs/development/tiago-pro-mocap.md. Nothing is tuned on the
held-out sessions; the model is selected on the validation role only.

Two numbers per set, both against the measured positions (mm):

- ``rmse_rereg`` (**primary**, ``PRIMARY_METRIC``): the trained joint
  parameters are kept and only the base frame (6D) and the tool point (3D)
  are refitted on the evaluated session. The mocap frame drifts between days
  (about 7 mm and 0.3 deg between the two held-out sessions), which the
  training frames cannot know;
- ``rmse_transfer`` (secondary): the training frames carried over unchanged.

Run from examples/tiago_pro.
"""

from __future__ import annotations

import contextlib
import io
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pinocchio as pin
import yaml
from scipy.optimize import least_squares

project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from figaroh.calibration.calibration_tools import (  # noqa: E402
    calc_updated_fkm,
    drop_calibration_parameters,
)
from figaroh.calibration.parameter import BASE_TPL  # noqa: E402

from examples.calibration_harness import heldout as harness  # noqa: E402
from examples.tiago_pro.run_calibration import TiagoProCalibration, _Robot  # noqa: E402

TIAGO_PRO = Path(__file__).resolve().parent
DATA = TIAGO_PRO / "data"
CONFIG = TIAGO_PRO / "tiago_pro_calibration_config.yaml"
URDF = TIAGO_PRO / "tiago_pro_local.urdf"
JOINTS = ["torso_lift_joint"] + [f"arm_right_{i}_joint" for i in range(1, 8)]
POSITION = [True, True, True, False, False, False]

#: The held-out metric that selects models and headlines the report. Frozen
#: with the protocol: the harness schema has no key for it.
PRIMARY_METRIC = "rmse_rereg"
SECONDARY_METRIC = "rmse_transfer"

PROTOCOL = DATA / "protocol.yaml"
PLAN = harness.load_plan(PROTOCOL)  # verifies every file's sha256
_FOLD = PLAN.folds[0]
# role, file; training first, then the protocol's order
SETS = [(s.role, s.name) for s in (*_FOLD.training, *_FOLD.evaluated)]
STRATA = PLAN.spec.strata
if tuple(JOINTS) != STRATA.joints:
    raise ValueError(f"{PROTOCOL.name}: strata joints differ from JOINTS")
_source = yaml.safe_load(CONFIG.read_text())["tasks"]["calibration"]["data"]
if Path(_source["source_file"]).name != SETS[0][1]:
    raise ValueError("config source_file is not the protocol's training file")
TRAINING = SETS[0][1]

# expected sizes of the joint errors for the regularised methods (m, rad):
# core's defaults, the same as the TIAGo study
PRIORS = {
    "translation": 1e-3,
    "rotation": 2e-3,
    "joint_offset": 2e-2,
    "prismatic_offset": 2e-3,
}
# label -> (calibration level, frames_only, core estimation settings or None
# for the config's)
MODELS = {
    # the joint_offset level is not available for this arm: arm_right_6_joint
    # is a RevoluteUnaligned joint that core's offset parameters cannot
    # express (create_param_list fails), so every model is built from
    # full_params, the registration baseline by dropping the joint parameters
    "registration only": ("full_params", True, {"method": "structural"}),
    "full_params": ("full_params", False, {"method": "structural"}),
    "full map": ("full_params", False, {"method": "map", "priors": PRIORS}),
    "full map_cv": ("full_params", False, {"method": "map_cv", "priors": PRIORS}),
    "full cv_subset": ("full_params", False, {"method": "cv_subset", "priors": PRIORS}),
}


def is_frame(name: str) -> bool:
    return name in BASE_TPL or "EE" in name


def fit(
    level: str,
    frames_only: bool = False,
    data_file: str | None = None,
    estimation: dict | None = None,
    instance: dict | None = None,
    position_only: bool = False,
) -> TiagoProCalibration:
    """Fit on the training session (config source_file) only.

    ``frames_only`` keeps the base and tool frame parameters (the
    registration baseline). ``data_file`` replaces the training file and
    ``estimation`` sets core's estimation method; the protocol sets neither.
    ``position_only`` fits the position of a session that has no orientation
    (used by diagnostics.py only).
    """
    robot = _Robot(pin.buildModelFromUrdf(str(URDF)), str(URDF))
    calib = TiagoProCalibration(robot, str(CONFIG))
    path = Path(data_file) if data_file else DATA / TRAINING
    calib.calib_config["data_file"] = str(path)
    calib._data_path = str(path.resolve())
    if frames_only and estimation is None:
        estimation = {"method": "structural"}  # nothing to regularise
    if estimation is not None:
        calib.calib_config["estimation"] = dict(estimation)
    if instance:
        calib.calib_config["instance"] = dict(
            calib.calib_config.get("instance") or {}, **instance
        )
    calib.calib_config["calib_model"] = level
    if position_only:
        calib.calib_config["measurability"] = list(POSITION)
        calib.calib_config["calibration_index"] = 3
    calib.calib_config["validation_data_file"] = None  # no held-out at fit time
    with contextlib.redirect_stdout(io.StringIO()):
        calib.initialize()
        if frames_only:
            joints = [n for n in calib.calib_config["param_name"] if not is_frame(n)]
            drop_calibration_parameters(calib.calib_config, joints)
        calib.solve(plotting=False, enable_logging=False, html_report=False)
    return calib


def posture_strata(path: Path, training: Path, tol: float | None = None) -> dict:
    """Split a session's postures by their relation to the training postures."""
    return harness.posture_strata(
        pd.read_csv(path)[JOINTS].to_numpy(),
        pd.read_csv(training)[JOINTS].to_numpy(),
        STRATA.repeated_tol if tol is None else tol,
        STRATA.out_of_range_margin,
    )


def session_postures(calib: TiagoProCalibration, df: pd.DataFrame) -> np.ndarray:
    """Full configurations of a session's rows."""
    q = np.tile(pin.neutral(calib.model), (len(df), 1))
    for j in JOINTS:
        q[:, calib.model.joints[calib.model.getJointId(j)].idx_q] = df[j].to_numpy()
    return q


def position_config(calib: TiagoProCalibration, n: int) -> dict:
    """The fit's config measuring position only (the held-out layout)."""
    return dict(
        calib.calib_config,
        NbSample=n,
        measurability=list(POSITION),
        calibration_index=3,
    )


def predict_positions(calib, x, q, cfg) -> np.ndarray:
    """(n, 3) marker positions of parameter values ``x`` at postures ``q``."""
    pred = calc_updated_fkm(calib.model, calib.data, x, q, cfg)
    return pred.reshape(3, -1).T


def reregister(calib: TiagoProCalibration, q, measured) -> np.ndarray:
    """Trained parameters with base (6D) and tool point refitted on a session.

    Only the base frame and the tool point position are free (rotation of the
    tool frame does not move a point); every joint-level parameter keeps its
    trained value. Starts from the training frames.
    """
    names = list(calib.calib_config["param_name"])
    x0 = np.array(calib.LM_result.x, dtype=float)
    free = [
        i
        for i, n in enumerate(names)
        if n in BASE_TPL or n in ("pEEx_1", "pEEy_1", "pEEz_1")
    ]
    cfg = position_config(calib, len(q))

    def residual(v):
        x = x0.copy()
        x[free] = v
        return (predict_positions(calib, x, q, cfg) - measured).ravel()

    sol = least_squares(residual, x0[free], method="lm")
    x = x0.copy()
    x[free] = sol.x
    return x


def _summary(err_mm: np.ndarray) -> dict:
    norm = np.linalg.norm(err_mm, axis=1)
    return {
        "rmse_xyz": np.sqrt(np.mean(err_mm**2, axis=0)),
        "rmse": float(np.sqrt(np.mean(norm**2))),
        "max": float(norm.max()),
        "norm": norm,
    }


def _by_mask(norm: np.ndarray, masks: dict) -> dict:
    return {
        k: (
            int(m.sum()),
            float(np.sqrt(np.mean(norm[m] ** 2))) if m.any() else np.nan,
        )
        for k, m in masks.items()
    }


def component_errors(
    calib: TiagoProCalibration, path: Path, validation: list | None = None
) -> dict:
    """Marker-1 position error (mm) of a fitted model on one session.

    ``rereg`` refits the base and tool point on the session, ``transfer``
    uses the training frames. ``strata`` splits the primary error by the
    posture's relation to the training postures; ``overlap`` (when
    ``validation`` lists session files) by whether the posture repeats one of
    those sessions.
    """
    df = pd.read_csv(path)
    q = session_postures(calib, df)
    measured = df[["x1", "y1", "z1"]].to_numpy()
    cfg = position_config(calib, len(df))
    x_train = np.array(calib.LM_result.x, dtype=float)
    transfer = _summary((predict_positions(calib, x_train, q, cfg) - measured) * 1000)
    x_re = reregister(calib, q, measured)
    rereg = _summary((predict_positions(calib, x_re, q, cfg) - measured) * 1000)
    strata = posture_strata(path, DATA / TRAINING)
    out = {
        "n": len(df),
        "rereg": {k: v for k, v in rereg.items() if k != "norm"},
        "transfer": {k: v for k, v in transfer.items() if k != "norm"},
        "rmse_rereg": rereg["rmse"],
        "rmse_transfer": transfer["rmse"],
        "frame_shift": {
            n: float(x_re[i] - x_train[i])
            for i, n in enumerate(calib.calib_config["param_name"])
            if n in BASE_TPL or n in ("pEEx_1", "pEEy_1", "pEEz_1")
        },
        "strata": _by_mask(rereg["norm"], strata),
        "strata_transfer": _by_mask(transfer["norm"], strata),
    }
    if validation:
        q_s = df[JOINTS].to_numpy()
        same = np.zeros(len(df), bool)
        for other in validation:
            q_o = pd.read_csv(other)[JOINTS].to_numpy()
            dist = np.abs(q_s[:, None, :] - q_o[None]).max(axis=2).min(axis=1)
            same |= dist < STRATA.repeated_tol
        masks = {"repeats_validation": same, "not_in_validation": ~same}
        out["overlap"] = _by_mask(rereg["norm"], masks)
        out["overlap_transfer"] = _by_mask(transfer["norm"], masks)
    return out


def fit_model(label: str) -> TiagoProCalibration:
    level, frames_only, estimation = MODELS[label]
    return fit(level, frames_only, estimation=estimation)


def evaluate(calib: TiagoProCalibration, held_out: harness.HeldoutSet) -> dict:
    """Metrics of one fitted model on one held-out set (for ``harness.run``)."""
    validation = (
        [DATA / s.name for r, s in _named_sets() if r == "validation"]
        if held_out.role == "confirmation"
        else None
    )
    return component_errors(calib, held_out.path, validation)


def _named_sets():
    return [(s.role, s) for s in (*_FOLD.training, *_FOLD.evaluated)]


def run(models: dict | None = None) -> dict:
    """Return ``{model: {...}}`` with the harness results and parameter count.

    ``results[model]["sets"][file]`` holds ``component_errors``;
    ``results[model]["harness"]`` is the ``harness.run`` structure used by
    ``harness.select``.
    """
    labels = list(MODELS) if models is None else list(models)
    fitted = {}

    def build(label):
        def go(fold):
            fitted[label] = fit_model(label)
            return fitted[label]

        return go

    res = harness.run(PLAN, {m: build(m) for m in labels}, evaluate)
    out = {}
    for label in labels:
        calib = fitted[label]
        names = list(calib.calib_config["param_name"])
        sets = res[label][0]
        train_cfg = calib.evaluation_metrics["per_dof_stats"]["overall"]
        out[label] = {
            "n_params": len(names),
            "n_joint_params": sum(not is_frame(n) for n in names),
            "train_pos_rmse_mm": float(train_cfg["pos_rmse_mm"]),
            "train_ori_rmse_deg": float(train_cfg["orient_rmse_deg"]),
            "sets": sets,
        }
    out["_harness"] = res
    return out


def select_model(results: dict) -> tuple:
    """Model with the lowest mean primary error on the validation role."""
    return harness.select(results["_harness"], PRIMARY_METRIC, role="validation")


def main() -> None:
    results = run()
    models = [m for m in results if not m.startswith("_")]
    print(f"Protocol {PLAN.protocol}: {PLAN.label}")
    print(
        f"Primary metric {PRIMARY_METRIC} (frames refitted on the session), "
        f"secondary {SECONDARY_METRIC} (training frames carried over); "
        "position RMSE, mm"
    )
    for label in models:
        res = results[label]
        print(
            f"\n{label} ({res['n_params']} parameters, "
            f"{res['n_joint_params']} joint-level; training "
            f"{res['train_pos_rmse_mm']:.2f} mm, {res['train_ori_rmse_deg']:.2f} deg)"
        )
        for role, f in SETS[1:]:
            e = res["sets"][f]
            x, y, z = e["rereg"]["rmse_xyz"]
            print(
                f"  {role:12s} {_short(f):10s} n={e['n']:3d}  re-registered "
                f"{e['rmse_rereg']:6.2f} (x {x:.2f} y {y:.2f} z {z:.2f}, "
                f"max {e['rereg']['max']:.2f})  transfer {e['rmse_transfer']:6.2f}"
            )
            print(
                "  "
                + " " * 29
                + "  ".join(f"{k} ({n}) {v:.2f}" for k, (n, v) in e["strata"].items())
            )
            if "overlap" in e:
                print(
                    "  "
                    + " " * 29
                    + "overlap with validation: "
                    + "  ".join(
                        f"{k} ({n}) {v:.2f}" for k, (n, v) in e["overlap"].items()
                    )
                )
    best, scores = select_model(results)
    print(f"\nSelected on the validation role ({PRIMARY_METRIC}): {best}")
    print("  " + "  ".join(f"{m} {s:.2f}" for m, s in scores.items()))


def _short(name: str) -> str:
    return name.removeprefix("calibration_samples_").removesuffix(".csv")


if __name__ == "__main__":
    main()
