"""UR10 hand-eye calibration: k-fold held-out comparison of the models (#99).

The 23 postures of ``data/calibration.csv`` are one session, so the frozen
protocol (``data/hand_eye_protocol.yaml``, sha256-verified at import) is a
5-fold split of its rows: every model is fitted on 4/5 of the postures and
judged on the rest. This is **k-fold within one session**, weaker evidence
than held-out sessions: the folds share the board placement, the camera and
the recording day, so a second session would be needed to measure how well
the frames carry over. The label is printed with every result.

Models, all built from the ``full_params`` level:

- ``registration only``: the 12 frame parameters (base 6D and tool 6D),
  joint parameters dropped: the hand-eye baseline;
- ``joint_offset``: structural, offsets only;
- ``full structural``: structural base parameters (ordered selection);
- ``full map``, ``full map_cv``, ``full excitation``, ``full cv_subset``:
  core's regularised methods with core's default prior sizes.

Metrics are never mixed into one number: ``rmse_pos_mm`` (board-frame
position error, the metric models are selected on, validation role only) and
``rmse_rot_deg`` (geodesic orientation error). There is no re-registration:
the training frames are carried over, since the fold shares the session.
The fit uses ``max_iterations=1`` so no outlier exclusion changes the folds.

Run from anywhere:  python examples/ur10/hand_eye_heldout.py
"""

from __future__ import annotations

import contextlib
import io
import sys
import tempfile
from pathlib import Path

import numpy as np
import pandas as pd
import pinocchio as pin

project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from figaroh.calibration.calibration_tools import (  # noqa: E402
    calc_updated_fkm,
    drop_calibration_parameters,
)
from figaroh.calibration.parameter import BASE_TPL  # noqa: E402
from figaroh.tools.robot import load_robot  # noqa: E402

from examples.calibration_harness import heldout as harness  # noqa: E402
from examples.ur10.utils.ur10_tools import UR10Calibration  # noqa: E402

UR10 = Path(__file__).resolve().parent
DATA = UR10 / "data"
CONFIG = UR10 / "config" / "ur10_unified_config.yaml"
URDF = UR10 / "urdf" / "ur10_robot.urdf"
MODELS_DIR = project_root / "models"
JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
POSE = ["x1", "y1", "z1", "phix1", "phiy1", "phiz1"]

#: headlines and selects models; orientation is reported next to it
PRIMARY_METRIC = "rmse_pos_mm"
SECONDARY_METRIC = "rmse_rot_deg"

PROTOCOL = DATA / "hand_eye_protocol.yaml"
PLAN = harness.load_plan(PROTOCOL)  # verifies the file's sha256
STRATA = PLAN.spec.strata
if tuple(JOINTS) != STRATA.joints:
    raise ValueError(f"{PROTOCOL.name}: strata joints differ from JOINTS")
TRAINING = PLAN.folds[0].training[0].path

# expected sizes of the joint errors for the regularised methods (m, rad):
# core's defaults, the same as the TIAGo and TALOS studies
PRIORS = {
    "translation": 1e-3,
    "rotation": 2e-3,
    "joint_offset": 2e-2,
    "prismatic_offset": 2e-3,
}
# label -> (calibration level, frames_only, core estimation settings)
MODELS = {
    "registration only": ("full_params", True, {"method": "structural"}),
    "joint_offset": ("joint_offset", False, {"method": "structural"}),
    "full structural": (
        "full_params",
        False,
        {"method": "structural", "structural_selection": "ordered"},
    ),
    "full map": ("full_params", False, {"method": "map", "priors": PRIORS}),
    "full map_cv": ("full_params", False, {"method": "map_cv", "priors": PRIORS}),
    "full excitation": ("full_params", False, {"method": "excitation"}),
    "full cv_subset": ("full_params", False, {"method": "cv_subset", "priors": PRIORS}),
}


def is_frame(name: str) -> bool:
    return name in BASE_TPL or "EE" in name


def fit(
    data_file,
    level: str = "full_params",
    frames_only: bool = False,
    estimation: dict | None = None,
    max_iterations: int = 1,
    instance: dict | None = None,
) -> UR10Calibration:
    """Fit ``data_file`` (default layout: the 6D pose and the six joints).

    ``frames_only`` keeps the 12 frame parameters (the registration
    baseline). ``max_iterations=1`` fits once, without outlier exclusion.
    ``estimation=None`` keeps the config's method; ``instance`` adds to the
    run's instance record (asset id, operator).
    """
    robot = load_robot(str(URDF), package_dirs=str(MODELS_DIR), load_by_urdf=True)
    calib = UR10Calibration(robot, str(CONFIG))
    path = Path(data_file).resolve()
    calib.calib_config["data_file"] = str(path)
    calib._data_path = str(path)
    calib.calib_config["known_baseframe"] = False
    calib.calib_config["known_tipframe"] = False
    calib.calib_config["calib_model"] = level
    if estimation is not None:
        calib.calib_config["estimation"] = dict(estimation)
    if instance:
        calib.calib_config["instance"] = dict(
            calib.calib_config.get("instance") or {}, **instance
        )
    calib.calib_config["validation_data_file"] = None
    with contextlib.redirect_stdout(io.StringIO()):
        calib.initialize()
        if frames_only:
            joints = [n for n in calib.calib_config["param_name"] if not is_frame(n)]
            drop_calibration_parameters(calib.calib_config, joints)
        calib.solve(
            plotting=False,
            enable_logging=False,
            html_report=False,
            max_iterations=max_iterations,
        )
    return calib


def postures(calib: UR10Calibration, df: pd.DataFrame) -> np.ndarray:
    """Full configurations of the rows of ``df``."""
    q = np.tile(pin.neutral(calib.model), (len(df), 1))
    for j in JOINTS:
        q[:, calib.model.joints[calib.model.getJointId(j)].idx_q] = df[j].to_numpy()
    return q


def predict(calib: UR10Calibration, x, df: pd.DataFrame) -> np.ndarray:
    """(n, 6) predicted poses (x, y, z, rpy) of parameter values ``x``."""
    cfg = dict(calib.calib_config, NbSample=len(df))
    pred = calc_updated_fkm(calib.model, calib.data, x, postures(calib, df), cfg)
    return np.asarray(pred).reshape(6, -1).T


def pose_errors(measured: np.ndarray, predicted: np.ndarray) -> tuple:
    """Per-row position error (mm) and geodesic rotation error (deg)."""
    pos = 1000 * np.linalg.norm(predicted[:, :3] - measured[:, :3], axis=1)
    rot = np.empty(len(measured))
    for i, (m, p) in enumerate(zip(measured, predicted)):
        r_m = pin.rpy.rpyToMatrix(m[3:])
        r_p = pin.rpy.rpyToMatrix(p[3:])
        rot[i] = np.degrees(np.linalg.norm(pin.log3(r_m.T @ r_p)))
    return pos, rot


def _rms(v: np.ndarray) -> float:
    return float(np.sqrt(np.mean(v**2))) if len(v) else float("nan")


def evaluate_rows(calib: UR10Calibration, df: pd.DataFrame, x=None) -> dict:
    """Errors of a fitted model on the rows of ``df`` (training frames kept)."""
    x = np.array(calib.LM_result.x if x is None else x, dtype=float)
    pos, rot = pose_errors(df[POSE].to_numpy(), predict(calib, x, df))
    return {
        "n": len(df),
        "rmse_pos_mm": _rms(pos),
        "rmse_rot_deg": _rms(rot),
        "max_pos_mm": float(pos.max()),
        "max_rot_deg": float(rot.max()),
        "pos_mm": pos.tolist(),
        "rot_deg": rot.tolist(),
    }


def fit_fold(label: str, fold: harness.Fold) -> UR10Calibration:
    """Fit ``label`` on the training rows of ``fold``."""
    level, frames_only, estimation = MODELS[label]
    (train,) = fold.training
    rows = pd.read_csv(train.path).iloc[list(train.rows)]
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "train.csv"
        rows.to_csv(path, index=False)
        return fit(path, level, frames_only, estimation)


def evaluate(calib: UR10Calibration, held_out: harness.HeldoutSet) -> dict:
    """Metrics of one fitted model on one held-out set (for ``harness.run``)."""
    df = pd.read_csv(held_out.path).iloc[list(held_out.rows)]
    out = evaluate_rows(calib, df)
    names = list(calib.calib_config["param_name"])
    out["n_params"] = len(names)
    out["n_joint_params"] = sum(not is_frame(n) for n in names)
    out["rows"] = list(held_out.rows)
    return out


def run(models=None) -> harness.HeldoutResults:
    """``harness.run`` over the plan for ``models`` (labels; default all)."""
    labels = list(MODELS) if models is None else list(models)
    return harness.run(
        PLAN,
        {m: (lambda fold, m=m: fit_fold(m, fold)) for m in labels},
        evaluate,
    )


def strata_masks(rows: list, train_rows: list) -> dict:
    """Posture strata of held-out ``rows`` against the fold's training rows."""
    df = pd.read_csv(TRAINING)
    return harness.posture_strata(
        df[JOINTS].to_numpy()[rows],
        df[JOINTS].to_numpy()[train_rows],
        STRATA.repeated_tol,
        STRATA.out_of_range_margin,
    )


def summarize(results: harness.HeldoutResults) -> dict:
    """Pool the folds: each row is held out once.

    Returns ``{model: {n_params, n_joint_params, rmse_pos_mm, rmse_rot_deg,
    max_pos_mm, max_rot_deg, strata: {name: (n, rmse_pos_mm)}}}``.
    """
    out = {}
    for model, folds in results.items():
        pos, rot, n_params, strata = [], [], [], {}
        for i, sets in folds.items():
            (m,) = sets.values()
            pos += m["pos_mm"]
            rot += m["rot_deg"]
            n_params.append((m["n_params"], m["n_joint_params"]))
            train = list(PLAN.folds[i].training[0].rows)
            for name, mask in strata_masks(m["rows"], train).items():
                strata.setdefault(name, []).extend(np.array(m["pos_mm"])[mask])
        pos, rot = np.array(pos), np.array(rot)
        out[model] = {
            "n_params": n_params[0][0],
            "n_joint_params": n_params[0][1],
            "rmse_pos_mm": _rms(pos),
            "rmse_rot_deg": _rms(rot),
            "max_pos_mm": float(pos.max()),
            "max_rot_deg": float(rot.max()),
            "strata": {k: (len(v), _rms(np.array(v))) for k, v in strata.items()},
        }
    return out


def select_model(results: harness.HeldoutResults) -> tuple:
    """Model with the lowest mean position error on the validation role."""
    return harness.select(results, PRIMARY_METRIC, role="validation")


def main(models=None) -> None:
    results = run(models)
    table = summarize(results)
    print(f"Protocol {PLAN.protocol}: {PLAN.label}")
    print(
        f"Held-out error of the training frames carried over; {PRIMARY_METRIC} is "
        f"the board-frame position error, {SECONDARY_METRIC} the geodesic "
        "orientation error (reported separately, never mixed)"
    )
    print(f"\n{'model':20s} {'params':>6s} {'pos mm':>8s} {'rot deg':>8s}  strata")
    for label, r in table.items():
        strata = "  ".join(f"{k} ({n}) {v:.2f}" for k, (n, v) in r["strata"].items())
        print(
            f"{label:20s} {r['n_params']:6d} {r['rmse_pos_mm']:8.2f} "
            f"{r['rmse_rot_deg']:8.3f}  {strata}"
        )
    best, scores = select_model(results)
    print(f"\nSelected on the validation role ({PRIMARY_METRIC}): {best}")
    print("  " + "  ".join(f"{m} {s:.2f}" for m, s in scores.items()))
    print(f"({PLAN.label})")


if __name__ == "__main__":
    main()
