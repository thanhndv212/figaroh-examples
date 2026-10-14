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
    python calibration_truth.py --methods       # compare core estimation methods

``--methods`` fits every truth with each of core's estimation methods
(figaroh-plus#113; ``ESTIMATION_FITS``), including ``map`` with priors 10x
too small and too large, so the effect of each choice can be compared
against the known truth. Guide: figaroh-plus
``docs/source/tutorials/calibration_estimation_guide.md``.
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
from figaroh.calibration.parameter import (  # noqa: E402
    get_fullparam_offset,
    get_joint_offset,
)

from examples.calibration_harness import simulation as sim  # noqa: E402
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

_PRIORS = {
    "translation": TRUTH_SIGMA["p"],
    "rotation": TRUTH_SIGMA["phi"],
    "joint_offset": TRUTH_SIGMA["offset"],
    "prismatic_offset": 2e-3,
}
# label -> (calibration level, core estimation settings); "map" gets the
# truth's own sizes, the x0.1 / x10 variants show a wrong guess
ESTIMATION_FITS = {
    "joint_offset structural": ("joint_offset", {}),
    "full structural": ("full_params", {}),
    "full excitation": ("full_params", {"method": "excitation", "priors": _PRIORS}),
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
HELD_OUT = [f for _, f in hp.SETS[1:]]


def postures(name: str) -> pd.DataFrame:
    return pd.read_csv(hp.MOCAP / name)[hp.JOINTS]


def draw_truth(kind: str, rng: np.random.Generator, model: pin.Model) -> dict:
    """Frames plus joint errors of one model class, drawn from ``rng``."""
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


def make_truth(kind: str, seed: int, model: pin.Model) -> dict:
    """Frames plus seeded joint errors of one model class."""
    return draw_truth(kind, np.random.default_rng(seed), model)


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
    p = sim.gaussian_mm(tool_points(calib, kind, truth, q), noise_mm, rng)
    return _dataset(p, q)


def _dataset(p: np.ndarray, q: pd.DataFrame) -> pd.DataFrame:
    df = pd.DataFrame(p, columns=["x1", "y1", "z1"])
    return pd.concat([df, q.reset_index(drop=True)], axis=1)


@functools.lru_cache(maxsize=1)
def study() -> sim.SimulationStudy:
    """The TIAGo synthetic-truth study on the shared harness."""
    probe = hp.fit("joint_offset", frames_only=True)  # model and config only

    def predict(params, kind, q, handle=None):
        return tool_points(handle or probe, kind, params, q)

    def fit(train, fit_level, estimation, workdir):
        path = Path(workdir) / "train.csv"
        train.to_csv(path, index=False)
        frames_only = fit_level == "registration only"
        level = "joint_offset" if frames_only else fit_level
        with contextlib.redirect_stdout(io.StringIO()):
            calib = hp.fit(
                level,
                frames_only=frames_only,
                data_file=str(path),
                estimation=estimation,
            )
        names = list(calib.calib_config["param_name"])
        return sim.FitResult(
            params=dict(zip(names, calib.LM_result.x)),
            std=dict(zip(names, calib.std_dev)) if calib.std_dev is not None else {},
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
        to_dataset=_dataset,
        train_postures=postures(TRAINING),
        heldout_postures={f: postures(f) for f in HELD_OUT},
        strata=lambda f: hp.posture_strata(hp.MOCAP / f, hp.MOCAP / TRAINING),
        z_params={"joint_offset": lambda n: n.startswith("offset")},
    )


def run_case(
    truth_kind, fit_level, seed, noise_mm, workdir, estimation=None, label=None
) -> dict:
    """Draw, simulate, fit, and judge one case (``sim.run_case`` on TIAGo).

    ``estimation`` is core's ``calib_config["estimation"]`` (figaroh-plus#113);
    ``label`` names the fit in the result (default ``fit_level``).
    """
    return sim.run_case(
        study(), truth_kind, fit_level, seed, noise_mm, workdir, estimation, label
    )


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2, 3, 4])
    parser.add_argument("--noise", type=float, nargs="+", default=[0.5, 2.0])
    parser.add_argument(
        "--methods",
        action="store_true",
        help="compare core estimation methods (ESTIMATION_FITS)",
    )
    args = parser.parse_args(argv)
    if args.methods:
        fits = {k: v for k, v in ESTIMATION_FITS.items()}
    else:
        fits = {
            f: (f, None) for f in ("registration only", "joint_offset", "full_params")
        }
    df = sim.run_grid(
        study(),
        ("joint_offset", "full_params"),
        fits,
        seeds=args.seeds,
        noise=args.noise,
    )
    summary, z = sim.summarize(df, {"new_mm": "new", "out_mm": "out_of_range"})
    print(summary.round(3).to_string())
    if z:
        print(
            f"\njoint_offset recovery: {z['n']} z-scores, mean {z['mean']:+.2f}, "
            f"rms {z['rms']:.2f}, max |z| {z['max_abs']:.2f}"
        )


if __name__ == "__main__":
    main()
