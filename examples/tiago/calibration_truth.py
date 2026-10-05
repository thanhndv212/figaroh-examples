"""TIAGo calibration on synthetic data with a known truth (#26).

The real mocap sessions cannot tell recovery from overfitting: nobody knows
the true geometry. Here the truth is drawn, measurements are simulated at the
real postures, FIGAROH fits them, and the fit is judged against the truth.

- **Postures:** the joint configurations of the real sessions, so excitation
  matches the held-out protocol (``heldout_protocol.py``): training is the
  37-posture session, held-out the 184 postures of the other three.
- **Truth:** base frame and tool point from the real ``joint_offset`` fit,
  plus seeded random joint errors (``TRUTH_SIGMA``), in either model class:
  ``joint_offset`` (one offset per joint) or ``full_params`` (six placement
  errors per joint, in the joint frame, figaroh-plus#110).
- **Measurements:** the tool point in the base frame, computed with core's
  ``calc_updated_fkm`` (the same model the fit uses), plus seeded Gaussian
  noise of ``noise_mm`` per axis.
- **Gauge:** base and tool frames are estimated, so some joint errors are
  absorbed by them (figaroh-plus#102). Recovery is judged in identifiable
  coordinates: the predicted tool point on held-out postures against the
  noise-free truth, and, when the fit has the truth's model class, the
  parameters the fit keeps.

    python calibration_truth.py                 # default grid, from examples/tiago
    python calibration_truth.py --seeds 0 1 2 --noise 0.5 2.0
"""

from __future__ import annotations

import argparse
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

from figaroh.calibration.calibration_tools import calc_updated_fkm  # noqa: E402
from figaroh.calibration.parameter import (  # noqa: E402
    get_fullparam_offset,
    get_joint_offset,
)

from examples.tiago import heldout_protocol as hp  # noqa: E402

# base frame and tool point of the real joint_offset fit (protocol v1)
FRAMES = {
    "base_px": 0.00662,
    "base_py": 0.19707,
    "base_pz": -0.33271,
    "base_phix": 0.00891,
    "base_phiy": 0.00725,
    "base_phiz": -0.00274,
    "pEEx_1": 0.06306,
    "pEEy_1": -0.0029,
    "pEEz_1": 0.07039,
}
# standard deviations of the drawn joint errors (m, rad). Encoder offsets
# dominate on the real robot (arm_5: -50 mrad); placement errors are
# manufacturing-sized.
TRUTH_SIGMA = {"offset": 0.02, "p": 1e-3, "phi": 2e-3}
TRAINING = hp.SETS[0][1]
HELD_OUT = [f for _, f in hp.SETS[1:]]


def postures(name: str) -> pd.DataFrame:
    return pd.read_csv(hp.MOCAP / name)[hp.JOINTS]


def make_truth(kind: str, seed: int, model: pin.Model) -> dict:
    """Frames plus seeded joint errors of one model class."""
    rng = np.random.default_rng(seed)
    act = [model.getJointId(j) for j in hp.JOINTS]
    truth = dict(FRAMES)
    if kind == "joint_offset":
        names = get_joint_offset(model, hp.JOINTS)  # every model joint
        for name in [n for n in names if n.endswith(tuple(hp.JOINTS))]:
            scale = TRUTH_SIGMA["offset"] if "offsetR" in name else 2e-3
            truth[name] = float(rng.normal(0.0, scale))
    elif kind == "full_params":
        for name in get_fullparam_offset([model.names[j] for j in act]):
            if name.startswith("d_phiz_") and "torso" not in name:
                scale = TRUTH_SIGMA["offset"]  # joint angle offset
            elif name.startswith("d_pz_torso"):
                scale = 2e-3  # prismatic offset
            elif name.startswith("d_phi"):
                scale = TRUTH_SIGMA["phi"]
            else:
                scale = TRUTH_SIGMA["p"]
            truth[name] = float(rng.normal(0.0, scale))
    else:
        raise ValueError(kind)
    return truth


def tool_points(calib, kind: str, values: dict, q: pd.DataFrame) -> np.ndarray:
    """(n, 3) tool point for parameter values ``values`` of class ``kind``."""
    model = calib.model
    Q = np.tile(pin.neutral(model), (len(q), 1))
    for j in hp.JOINTS:
        Q[:, model.joints[model.getJointId(j)].idx_q] = q[j].to_numpy()
    cfg = dict(
        calib.calib_config,
        calib_model=kind,
        param_name=list(values),
        NbSample=len(q),
    )
    pee = calc_updated_fkm(model, calib.data, np.array(list(values.values())), Q, cfg)
    return pee.reshape(3, -1).T


def simulate(calib, kind, truth, q, noise_mm, seed) -> pd.DataFrame:
    """Measurements in the mocap CSV layout (marker 1 only)."""
    rng = np.random.default_rng(10_000 + seed)
    p = tool_points(calib, kind, truth, q)
    p = p + rng.normal(0.0, noise_mm * 1e-3, p.shape)
    df = pd.DataFrame(p, columns=["x1", "y1", "z1"])
    return pd.concat([df, q.reset_index(drop=True)], axis=1)


def run_case(truth_kind, fit_level, seed, noise_mm, workdir) -> dict:
    """Draw, simulate, fit, and judge one case."""
    probe = hp.fit("joint_offset", frames_only=True)  # model and config only
    truth = make_truth(truth_kind, seed, probe.model)
    train = simulate(probe, truth_kind, truth, postures(TRAINING), noise_mm, seed)
    path = Path(workdir) / f"truth_{truth_kind}_{seed}_{noise_mm}.csv"
    train.to_csv(path, index=False)

    frames_only = fit_level == "registration only"
    level = "joint_offset" if frames_only else fit_level
    with contextlib.redirect_stdout(io.StringIO()):
        calib = hp.fit(level, frames_only=frames_only, data_file=str(path))
    names = list(calib.calib_config["param_name"])
    x = calib.LM_result.x
    fitted = dict(zip(names, x))

    result = {
        "truth": truth_kind,
        "fit": fit_level,
        "seed": seed,
        "noise_mm": noise_mm,
        "n_params": len(x),
        "solver_status": int(calib.LM_result.status),
        "nfev": int(calib.LM_result.nfev),
        "absorbed": list(calib.calib_config.get("absorbed_param_name", [])),
    }
    pred = tool_points(calib, level, fitted, postures(TRAINING))
    meas = train[["x1", "y1", "z1"]].to_numpy()
    result["train_rms_mm"] = 1000 * float(np.sqrt(np.mean((pred - meas) ** 2)))

    errors = []
    strata = {k: [] for k in ("repeated", "new", "out_of_range")}
    for f in HELD_OUT:
        q = postures(f)
        e = tool_points(calib, level, fitted, q) - tool_points(
            probe, truth_kind, truth, q
        )
        norm = 1000 * np.linalg.norm(e, axis=1)
        errors.append(norm)
        for k, m in hp.posture_strata(hp.MOCAP / f, hp.MOCAP / TRAINING).items():
            strata[k].append(norm[m])
    errors = np.concatenate(errors)
    result["heldout_rmse_mm"] = float(np.sqrt(np.mean(errors**2)))
    result["heldout_max_mm"] = float(errors.max())
    result["strata_rmse_mm"] = {
        k: float(np.sqrt(np.mean(np.concatenate(v) ** 2))) for k, v in strata.items()
    }

    if fit_level == truth_kind == "joint_offset":
        joints = [n for n in names if n.startswith("offset")]
        std = dict(zip(names, calib.std_dev))
        result["z_scores"] = {
            n: float((fitted[n] - truth[n]) / std[n]) for n in joints if std[n] > 0
        }
    return result


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2, 3, 4])
    parser.add_argument("--noise", type=float, nargs="+", default=[0.5, 2.0])
    args = parser.parse_args(argv)
    fits = ["registration only", "joint_offset", "full_params"]
    with tempfile.TemporaryDirectory() as tmp:
        rows = [
            run_case(t, f, s, n, tmp)
            for t in ("joint_offset", "full_params")
            for n in args.noise
            for f in fits
            for s in args.seeds
        ]
    df = pd.DataFrame(rows)
    df["new_mm"] = [r["new"] for r in df.strata_rmse_mm]
    df["out_mm"] = [r["out_of_range"] for r in df.strata_rmse_mm]
    summary = df.groupby(["truth", "noise_mm", "fit"], sort=False).agg(
        n_params=("n_params", "median"),
        train_rms_mm=("train_rms_mm", "mean"),
        heldout_rmse_mm=("heldout_rmse_mm", "mean"),
        new_mm=("new_mm", "mean"),
        out_mm=("out_mm", "mean"),
        heldout_max_mm=("heldout_max_mm", "max"),
    )
    print(summary.round(3).to_string())
    z = [
        v
        for zs in df.get("z_scores", pd.Series(dtype=object)).dropna()
        for v in zs.values()
    ]
    if z:
        z = np.array(z)
        print(
            f"\njoint_offset recovery: {len(z)} z-scores, mean {z.mean():+.2f}, "
            f"rms {np.sqrt(np.mean(z**2)):.2f}, max |z| {np.abs(z).max():.2f}"
        )


if __name__ == "__main__":
    main()
