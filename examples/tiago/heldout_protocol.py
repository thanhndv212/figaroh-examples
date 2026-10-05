"""TIAGo mocap calibration: frozen held-out protocol (#27).

Fits three models on the training session only and reports per-component
position error on every frozen set:

- registration only: nominal joints, base (6D) and tip (3D) fitted;
- joint_offset: the reference (calibration.py defaults);
- full_params: placement errors reduced to identifiable parameters.

The sets, their roles and their sha256 are fixed in
docs/development/tiago-mocap-heldout-protocol.md and checked by
tests/test_tiago_heldout_protocol.py. Nothing here is tuned on the
held-out sets. Run from examples/tiago.
"""

from __future__ import annotations

import contextlib
import io
import sys
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
from figaroh.tools.robot import load_robot  # noqa: E402

from examples.tiago.utils.tiago_tools import TiagoCalibration  # noqa: E402

TIAGO = Path(__file__).resolve().parent
MOCAP = TIAGO / "data/calibration/mocap"
CONFIG = TIAGO / "config/tiago_unified_config.yaml"
URDF = TIAGO / "urdf/tiago_48_schunk.urdf"
JOINTS = ["torso_lift_joint"] + [f"arm_{i}_joint" for i in range(1, 8)]

# role, file; training is the config's source_file
SETS = [
    ("training", "qualisys_2021-11-30_static_postures.csv"),
    ("validation", "qualisys_2021-11-26_static_postures.csv"),
    ("confirmation", "qualisys_2021-11-30-1403_static_postures.csv"),
    ("confirmation", "qualisys_2021-11-30-1504_static_postures.csv"),
]
MODELS = {
    "registration only": ("joint_offset", True),
    "joint_offset": ("joint_offset", False),
    "full_params": ("full_params", False),
}


def fit(
    level: str,
    frames_only: bool = False,
    data_file: str | None = None,
    estimation: dict | None = None,
) -> TiagoCalibration:
    """Fit on the training session (config source_file) only.

    ``data_file`` replaces the training file and ``estimation`` sets core's
    estimation method (synthetic fixture, ``calibration_truth.py``); the
    protocol itself sets neither.
    """
    robot = load_robot(str(URDF), load_by_urdf=True, robot_pkg="tiago_description")
    calib = TiagoCalibration(robot, str(CONFIG), del_list=[])
    if estimation is not None:
        calib.calib_config["estimation"] = dict(estimation)
    if data_file is not None:
        calib.calib_config["data_file"] = str(data_file)
        calib._data_path = str(Path(data_file).resolve())
    calib.calib_config["calib_model"] = level
    calib.calib_config["validation_data_file"] = None  # no held-out at fit time
    calib.calib_config["known_baseframe"] = False
    calib.calib_config["known_tipframe"] = False
    with contextlib.redirect_stdout(io.StringIO()):
        calib.initialize()
        if frames_only:
            joints = [
                n
                for n in calib.calib_config["param_name"]
                if not (n.startswith("base_") or "EE" in n)
            ]
            drop_calibration_parameters(calib.calib_config, joints)
        calib.solve(plotting=False, enable_logging=False, html_report=False)
    return calib


def posture_strata(path: Path, training: Path, tol: float = 0.01) -> dict:
    """Split a session's postures by their relation to the training postures.

    - ``repeated``: within ``tol`` rad (or m) of a training posture on every
      joint, i.e. the same configuration recorded on another occasion;
    - ``out_of_range``: some joint outside the training range by > 0.05;
    - ``new``: everything else (new configurations inside the training range).
    """
    q = pd.read_csv(path)[JOINTS].to_numpy()
    q_train = pd.read_csv(training)[JOINTS].to_numpy()
    nearest = np.array([np.min(np.max(np.abs(q_train - x), axis=1)) for x in q])
    repeated = nearest < tol
    outside = np.any((q < q_train.min(0) - 0.05) | (q > q_train.max(0) + 0.05), axis=1)
    return {
        "repeated": repeated,
        "new": ~repeated & ~outside,
        "out_of_range": outside & ~repeated,
    }


def component_errors(calib: TiagoCalibration, path: Path) -> dict:
    """Marker-1 position error (mm) of a fitted model on one session."""
    df = pd.read_csv(path)
    q = np.tile(pin.neutral(calib.model), (len(df), 1))
    for j in JOINTS:
        q[:, calib.model.joints[calib.model.getJointId(j)].idx_q] = df[j].to_numpy()
    cfg = dict(calib.calib_config, NbSample=len(df))
    pred = calc_updated_fkm(calib.model, calib.data, calib.LM_result.x, q, cfg)
    err = (pred.reshape(3, -1).T - df[["x1", "y1", "z1"]].to_numpy()) * 1000
    norm = np.linalg.norm(err, axis=1)
    strata = posture_strata(path, MOCAP / SETS[0][1])
    return {
        "n": len(df),
        "rmse_xyz": np.sqrt(np.mean(err**2, axis=0)),
        "rmse": float(np.sqrt(np.mean(norm**2))),
        "max": float(norm.max()),
        "strata": {
            k: (
                int(m.sum()),
                float(np.sqrt(np.mean(norm[m] ** 2))) if m.any() else np.nan,
            )
            for k, m in strata.items()
        },
    }


def run() -> dict:
    """Return ``{model: {file: errors}}`` plus each model's parameter count."""
    results = {}
    for label, (level, frames_only) in MODELS.items():
        calib = fit(level, frames_only)
        values = dict(zip(calib.calib_config["param_name"], calib.LM_result.x))
        stds = dict(zip(calib.calib_config["param_name"], calib.std_dev))
        results[label] = {
            "n_params": len(calib.LM_result.x),
            "arm_5": (
                values.get("offsetRZ_arm_5_joint"),
                stds.get("offsetRZ_arm_5_joint"),
            ),
            "sets": {f: component_errors(calib, MOCAP / f) for _, f in SETS},
        }
    return results


def main() -> None:
    results = run()
    for label, res in results.items():
        line = f"\n{label} ({res['n_params']} parameters)"
        if res["arm_5"][0] is not None:
            line += f", arm_5 {res['arm_5'][0] * 1e3:.1f} ± {res['arm_5'][1] * 1e3:.1f} mrad"
        print(line)
        for role, f in SETS:
            e = res["sets"][f]
            x, y, z = e["rmse_xyz"]
            print(
                f"  {role:12s} {f.removeprefix('qualisys_').removesuffix('_static_postures.csv'):15s} n={e['n']:2d}  x {x:.2f}  y {y:.2f}  z {z:.2f}"
                f"  | RMSE {e['rmse']:.2f}  max {e['max']:.2f} mm"
            )
            if role != "training":
                print(
                    "  "
                    + " " * 29
                    + "  ".join(
                        f"{k} ({n}) {v:.2f}" for k, (n, v) in e["strata"].items()
                    )
                )


if __name__ == "__main__":
    main()
