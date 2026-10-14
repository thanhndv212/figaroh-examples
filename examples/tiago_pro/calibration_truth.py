"""TIAGo Pro calibration on synthetic data with a known truth (#97).

The real sessions cannot tell recovery from overfitting: nobody knows the
true geometry. Here the truth is drawn, measurements are simulated at the
real postures, FIGAROH fits them, and the fit is judged against the truth.
Same design as examples/tiago/calibration_truth.py, on the harness
(examples/calibration_harness/simulation.py).

- **Postures:** the joint configurations of the real sessions: training is
  the 48-posture 2026-08-05 session (full pose), held out are the
  2026-07-01 and 2026-07-02 postures, judged on the tool position.
- **Truth:** base and tool frames from the real registration-only fit of the
  training session (``FRAMES``), plus seeded random joint errors
  (``TRUTH_SIGMA``) of the ``full_params`` class (six placement errors per
  joint, in the joint frame). Only this class exists here: the joint_offset
  level cannot represent the non-axis-aligned arm_right_6_joint (core
  fails to build its parameter list), see heldout_protocol.MODELS.
- **Measurements:** pose of the tool body in the mocap frame, computed with
  core's ``calc_updated_fkm`` (the model the fit uses), plus seeded Gaussian
  noise. The harness adds one isotropic noise level in millimetres per
  column, so the orientation is carried as the rotation vector from a
  reference orientation (the nominal model with ``FRAMES``, at the same
  posture) in lever-arm units (angle x ``LEVER`` metres): a level of 1.35 mm
  is 1.35e-3 / 0.1 rad = 0.77 deg of orientation noise. Roll-pitch-yaw
  would wrap at +-pi (the tool points down) and corrupt the differences the
  harness takes; the rotation vector from a nearby reference does not.
  ``to_dataset`` converts back to the CSV's roll-pitch-yaw.
- **Noise grid:** (0.5, estimate, 3.0) mm; the estimate is the repeat-posture
  re-registration residual of data_audit.py, a substitute for a within-
  session repeat and an upper bound.
- **Gauge:** base and tool frames are estimated, so some joint errors are
  absorbed by them. Recovery is judged in identifiable coordinates: the
  predicted tool position on held-out postures against the noise-free
  truth, with the training frames carried over (transfer; a simulation has
  no frame drift), and the z-score of the joint-level parameters of the
  fits that report their own uncertainty (map family). Structural fits
  return base-parameter representatives that are not comparable to
  individual truth parameters, so they report zero std and are not scored.

    python calibration_truth.py                 # default grid, from examples/tiago_pro
    python calibration_truth.py --seeds 0 1 2 --noise 0.5 3.0
    python calibration_truth.py --methods       # compare core estimation methods
"""

from __future__ import annotations

import argparse
import contextlib
import functools
import io
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pinocchio as pin

project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from figaroh.calibration.calibration_tools import calc_updated_fkm  # noqa: E402
from figaroh.calibration.parameter import get_fullparam_offset  # noqa: E402

from examples.calibration_harness import simulation as sim  # noqa: E402
from examples.tiago_pro import data_audit  # noqa: E402
from examples.tiago_pro import heldout_protocol as hp  # noqa: E402

# base and tool frames of the registration-only fit of the training session
# (checked against a fresh fit by tests/test_tiago_pro_calibration_truth.py)
FRAMES = {
    "base_px": 0.003392,
    "base_py": 0.002035,
    "base_pz": 0.010818,
    "base_phix": 0.00384,
    "base_phiy": -0.004333,
    "base_phiz": -0.001832,
    "pEEx_1": -0.00046,
    "pEEy_1": 0.001802,
    "pEEz_1": 0.137563,
    "phiEEx_1": -0.019876,
    "phiEEy_1": 0.006873,
    "phiEEz_1": 0.039047,
}
# standard deviations of the drawn joint errors (m, rad): encoder-type angle
# offsets dominate, placement errors are manufacturing-sized
TRUTH_SIGMA = {"offset": 0.02, "p": 1e-3, "phi": 2e-3, "prismatic": 2e-3}
# metres per radian of the orientation columns (see the module docstring)
LEVER = 0.1
TRAINING = hp.SETS[0][1]
HELD_OUT = [f for _, f in hp.SETS[1:]]
TRUTH_KINDS = ("full_params",)
SEEDS = (0, 1, 2, 3, 4)
STRUCTURAL = {"method": "structural"}
ALL = [True] * 6

_PRIORS = {
    "translation": TRUTH_SIGMA["p"],
    "rotation": TRUTH_SIGMA["phi"],
    "joint_offset": TRUTH_SIGMA["offset"],
    "prismatic_offset": TRUTH_SIGMA["prismatic"],
}
# label -> (calibration level, core estimation settings); "map" gets the
# truth's own sizes, the x0.1 / x10 variants show a wrong guess
ESTIMATION_FITS = {
    "full structural": ("full_params", {}),
    "full map": ("full_params", {"method": "map", "priors": _PRIORS}),
    "full map priors x0.1": (
        "full_params",
        {"method": "map", "priors": {k: v * 0.1 for k, v in _PRIORS.items()}},
    ),
    "full map priors x10": (
        "full_params",
        {"method": "map", "priors": {k: v * 10 for k, v in _PRIORS.items()}},
    ),
    "full map_cv": ("full_params", {"method": "map_cv", "priors": _PRIORS}),
    "full cv_subset": ("full_params", {"method": "cv_subset", "priors": _PRIORS}),
}
# methods whose standard deviations can be scored against individual truths
_SCORED = {"map", "map_cv", "cv_subset"}


def postures(name: str) -> pd.DataFrame:
    return pd.read_csv(hp.DATA / name)[hp.JOINTS]


def is_joint_param(name: str) -> bool:
    return name.startswith("d_")


def draw_truth(kind: str, rng: np.random.Generator, model: pin.Model) -> dict:
    """Frames plus joint errors of the ``full_params`` class, from ``rng``."""
    if kind != "full_params":
        raise ValueError(kind)
    truth = dict(FRAMES)
    for name in get_fullparam_offset(hp.JOINTS):
        if name.startswith("d_phiz_") and "torso" not in name:
            scale = TRUTH_SIGMA["offset"]  # joint angle offset
        elif name.startswith("d_pz_torso"):
            scale = TRUTH_SIGMA["prismatic"]
        elif name.startswith("d_phi"):
            scale = TRUTH_SIGMA["phi"]
        else:
            scale = TRUTH_SIGMA["p"]
        truth[name] = float(rng.normal(0.0, scale))
    return truth


def _tool_rpy(calib, kind: str, values: dict, q: pd.DataFrame) -> np.ndarray:
    """(n, 6) tool pose: position (m) and roll-pitch-yaw (rad), as measured."""
    model = calib.model
    Q = np.tile(pin.neutral(model), (len(q), 1))
    for j in hp.JOINTS:
        Q[:, model.joints[model.getJointId(j)].idx_q] = q[j].to_numpy()
    cfg = dict(
        calib.calib_config,
        calib_model=kind,
        param_name=list(values),
        NbSample=len(q),
        measurability=list(ALL),
        calibration_index=6,
    )
    pee = calc_updated_fkm(model, calib.data, np.array(list(values.values())), Q, cfg)
    return pee.reshape(6, -1).T.copy()


def _reference(calib, q: pd.DataFrame) -> np.ndarray:
    """(n, 3, 3) reference orientations: nominal joints, ``FRAMES``."""
    rpy = _tool_rpy(calib, "full_params", FRAMES, q)[:, 3:]
    return np.array([pin.rpy.rpyToMatrix(*r) for r in rpy])


def tool_pose(calib, kind: str, values: dict, q: pd.DataFrame) -> np.ndarray:
    """(n, 6) tool pose: position (m), rotation vector x ``LEVER`` (m)."""
    pose = _tool_rpy(calib, kind, values, q)
    ref = _reference(calib, q)
    for i, r in enumerate(pose[:, 3:]):
        R = pin.rpy.rpyToMatrix(*r)
        pose[i, 3:] = LEVER * pin.log3(ref[i].T @ R)
    return pose


def to_dataset(calib, p: np.ndarray, q: pd.DataFrame) -> pd.DataFrame:
    """Measurements in the CSV layout (position, roll-pitch-yaw, joints)."""
    p = np.asarray(p, dtype=float).copy()
    ref = _reference(calib, q)
    for i in range(len(p)):
        R = ref[i] @ pin.exp3(p[i, 3:] / LEVER)
        p[i, 3:] = pin.rpy.matrixToRpy(R)
    cols = ["x1", "y1", "z1", "phix1", "phiy1", "phiz1"]
    return pd.concat([pd.DataFrame(p, columns=cols), q.reset_index(drop=True)], axis=1)


def _position_error(e: np.ndarray) -> np.ndarray:
    return 1000 * np.linalg.norm(e[:, :3], axis=1)


@functools.lru_cache(maxsize=1)
def study() -> sim.SimulationStudy:
    """The TIAGo Pro synthetic-truth study on the shared harness."""
    probe = hp.fit("full_params", frames_only=True)  # model and config only

    def predict(params, kind, q, handle=None):
        return tool_pose(handle or probe, kind, params, q)

    def fit(train, fit_level, estimation, workdir):
        path = Path(workdir) / "train.csv"
        train.to_csv(path, index=False)
        frames_only = fit_level == "registration only"
        level = "full_params" if frames_only else fit_level
        with contextlib.redirect_stdout(io.StringIO()):
            calib = hp.fit(
                level,
                frames_only=frames_only,
                data_file=str(path),
                estimation=estimation,
            )
        names = list(calib.calib_config["param_name"])
        method = (estimation or {}).get("method", "structural")
        if method in _SCORED and calib.std_dev is not None:
            std = dict(zip(names, calib.std_dev))
        else:
            std = dict.fromkeys(names, 0.0)
        return sim.FitResult(
            params=dict(zip(names, calib.LM_result.x)),
            std=std,
            level=level,
            status=int(calib.LM_result.status),
            nfev=int(calib.LM_result.nfev),
            absorbed=list(calib.calib_config.get("absorbed_param_name", [])),
            handle=calib,
        )

    return sim.SimulationStudy(
        sample_truth=lambda kind, rng: draw_truth(kind, rng, probe.model),
        predict=predict,
        fit=fit,
        to_dataset=lambda p, q: to_dataset(probe, p, q),
        train_postures=postures(TRAINING),
        heldout_postures={f: postures(f) for f in HELD_OUT},
        strata=lambda f: hp.posture_strata(hp.DATA / f, hp.DATA / TRAINING),
        z_params={"full_params": is_joint_param},
        row_error=_position_error,
    )


def run_case(
    truth_kind, fit_level, seed, noise_mm, workdir, estimation=None, label=None
) -> dict:
    """Draw, simulate, fit, and judge one case (``sim.run_case`` on TIAGo Pro)."""
    return sim.run_case(
        study(), truth_kind, fit_level, seed, noise_mm, workdir, estimation, label
    )


def run(seeds=SEEDS, noise=None, methods: bool = False) -> pd.DataFrame:
    """The grid: truth x noise x fit x seed (``noise`` defaults to the grid)."""
    fits = (
        ESTIMATION_FITS
        if methods
        else {
            "registration only": ("registration only", STRUCTURAL),
            "full_params": ("full_params", STRUCTURAL),
        }
    )
    return sim.run_grid(
        study(),
        TRUTH_KINDS,
        fits,
        seeds=seeds,
        noise=data_audit.noise_grid() if noise is None else noise,
    )


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--seeds", type=int, nargs="+", default=list(SEEDS))
    parser.add_argument("--noise", type=float, nargs="+", default=None)
    parser.add_argument(
        "--methods",
        action="store_true",
        help="compare core estimation methods (ESTIMATION_FITS)",
    )
    args = parser.parse_args(argv)
    df = run(args.seeds, args.noise, args.methods)
    summary, z = sim.summarize(df, {"new_mm": "new", "out_mm": "out_of_range"})
    print(summary.round(3).to_string())
    if z:
        print(
            f"\nfull_params joint-level recovery (fits with their own std): "
            f"{z['n']} z-scores, mean {z['mean']:+.2f}, rms {z['rms']:.2f}, "
            f"max |z| {z['max_abs']:.2f}"
        )


if __name__ == "__main__":
    main()
