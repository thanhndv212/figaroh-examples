"""Staubli TX40 calibration on synthetic data with a known truth.

There is no recorded TX40 calibration data. This study is the only evidence
for the TX40 calibration pipeline, and it is simulated: it says what FIGAROH
recovers at a given noise level and posture design, not how far a real TX40
is from its URDF.

- **Postures:** two training designs of the same size, drawn from the same
  1000 candidates as ``optimal_config.py`` (seed 0, collision-checked):
  ``doptimal``, the D-optimal selection frozen in ``data/truth/``, and
  ``random``, a seeded random subset. Held-out: 300 fresh candidates
  (sampler seed 1).
- **Truth:** a base frame and a 6-DOF tool frame (``FRAMES``), plus seeded
  joint errors (``TRUTH_SIGMA``) in one of two model classes:
  ``joint_offset`` (one offset per joint) or ``full_params`` (six placement
  errors per joint).
- **Measurements:** the tool pose (position and orientation) computed with
  core's ``calc_updated_fkm``, the model the fit uses, plus seeded Gaussian
  noise: ``noise`` mm per position axis and ``noise`` mrad per rotation
  component.
- **Gauge:** base and tool frames are estimated, so they absorb the errors of
  joints 1 and 6 (figaroh-plus#102). Recovery is judged on the predicted tool
  position at the held-out postures against the noise-free truth, and, when
  the fit has the truth's model class, on the offsets the fit keeps.

    python calibration_truth.py                     # default grid, from examples/staubli_tx40
    python calibration_truth.py --seeds 0 1 --noise 0.1
    python calibration_truth.py --design            # rerun the D-optimal selection

Guide: ``docs/development/tx40-calibration-synthetic-truth.md``.
"""

from __future__ import annotations

import argparse
import contextlib
import functools
import io
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
project_root = HERE.parents[1]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from figaroh.calibration.calibration_tools import (  # noqa: E402
    calc_updated_fkm,
    drop_calibration_parameters,
)
from figaroh.calibration.parameter import (  # noqa: E402
    get_fullparam_offset,
    get_joint_offset,
)
from figaroh.tools.robot import load_robot  # noqa: E402

from examples.calibration_harness import simulation as sim  # noqa: E402
from examples.candidate_postures import sample_chain_postures  # noqa: E402
from examples.staubli_tx40.utils.staubli_tx40_tools import (  # noqa: E402
    TX40Calibration,
)

URDF = HERE / "urdf" / "tx40_mdh_modified.urdf"
CONFIG = HERE / "config" / "staubli_tx40_unified_config.yaml"
MODELS = HERE.parents[1] / "models"
TRUTH_DIR = HERE / "data" / "truth"
DESIGN_FILE = TRUTH_DIR / "calibration_doptimal_postures.csv"
DESIGN_META = TRUTH_DIR / "calibration_doptimal_postures.json"
JOINTS = [f"joint_{i}" for i in range(1, 7)]
POSE = ["x1", "y1", "z1", "phix1", "phiy1", "phiz1"]

# candidates of optimal_config.py (seed 0) and the held-out pool (seed 1)
CANDIDATES = {"n": 1000, "seed": 0}
HELDOUT = {"n": 300, "seed": 1}
RANDOM_DESIGN_SEED = 0
MARGIN = 0.05  # rad, tasks.optimal_configuration.constraints.joint_limit_margin

# a TX40 on a table, measured by a tracker in the cell frame, with a probe
# 12 cm off the flange
FRAMES = {
    "base_px": 0.45,
    "base_py": -0.30,
    "base_pz": 0.02,
    "base_phix": 0.004,
    "base_phiy": -0.003,
    "base_phiz": 0.05,
    "pEEx_1": 0.010,
    "pEEy_1": -0.015,
    "pEEz_1": 0.120,
    "phiEEx_1": 0.02,
    "phiEEy_1": -0.01,
    "phiEEz_1": 0.03,
}
# standard deviations of the drawn joint errors (rad, m): encoder offsets of
# a few mrad, machining-sized placement errors
TRUTH_SIGMA = {"offset": 5e-3, "p": 3e-4, "phi": 1e-3}
# noise: ``level`` mm on each position axis, ``level`` mrad on each rotation
NOISE = sim.anisotropic([1e-3] * 6)
FITS = ("registration only", "joint_offset", "full_params")
DESIGNS = ("doptimal", "random")


def load_tx40():
    return load_robot(str(URDF), package_dirs=str(MODELS), load_by_urdf=True)


def _frame(q: np.ndarray) -> pd.DataFrame:
    return pd.DataFrame(np.asarray(q)[:, :6], columns=JOINTS)


@functools.lru_cache(maxsize=None)
def candidate_pool() -> pd.DataFrame:
    """The 1000 candidates ``optimal_config.py`` draws (seed 0)."""
    pool = sample_chain_postures(
        load_tx40(),
        range(6),
        CANDIDATES["n"],
        seed=CANDIDATES["seed"],
        margin=MARGIN,
        collision="non_adjacent",
    )
    return _frame(pool.q)


@functools.lru_cache(maxsize=None)
def heldout_postures() -> pd.DataFrame:
    pool = sample_chain_postures(
        load_tx40(),
        range(6),
        HELDOUT["n"],
        seed=HELDOUT["seed"],
        margin=MARGIN,
        collision="non_adjacent",
    )
    return _frame(pool.q)


def design(name: str) -> pd.DataFrame:
    """Training postures of one design."""
    doptimal = pd.read_csv(DESIGN_FILE)[JOINTS]
    if name == "doptimal":
        return doptimal
    if name == "random":
        rng = np.random.default_rng(RANDOM_DESIGN_SEED)
        idx = rng.choice(CANDIDATES["n"], size=len(doptimal), replace=False)
        return candidate_pool().iloc[np.sort(idx)].reset_index(drop=True)
    raise ValueError(f"design must be one of {DESIGNS}")


def select_doptimal() -> dict:
    """Rerun the D-optimal selection of ``optimal_config.py`` (about 1 min)
    and freeze the selected postures in ``DESIGN_FILE``."""
    from examples.staubli_tx40.utils.staubli_tx40_tools import (
        TX40OptimalCalibration,
    )

    opt = TX40OptimalCalibration(
        load_tx40(), str(CONFIG), n_candidates=CANDIDATES["n"], seed=CANDIDATES["seed"]
    )
    with contextlib.redirect_stdout(io.StringIO()):
        opt.solve(save_file=False)
    q = np.asarray(opt.optimal_configurations["calibration_joint_configurations"])
    TRUTH_DIR.mkdir(parents=True, exist_ok=True)
    pd.DataFrame(q, columns=JOINTS).to_csv(DESIGN_FILE, index=False, float_format="%.17g")
    meta = {
        "source": "TX40OptimalCalibration (optimal_config.py), threshold selection",
        "candidates": CANDIDATES,
        "margin_rad": MARGIN,
        "selected": int(len(q)),
        "detroot_whole": float(opt.detroot_whole),
    }
    DESIGN_META.write_text(json.dumps(meta, indent=2) + "\n")
    return meta


def draw_truth(kind: str, rng: np.random.Generator, model) -> dict:
    """Frames plus joint errors of one model class, drawn from ``rng``."""
    truth = dict(FRAMES)
    if kind == "joint_offset":
        for name in get_joint_offset(model, JOINTS):
            scale = TRUTH_SIGMA["offset"] if name.startswith("offsetRZ") else 0.0
            truth[name] = float(rng.normal(0.0, scale)) if scale else 0.0
    elif kind == "full_params":
        for name in get_fullparam_offset(JOINTS):
            if name.startswith("d_phiz_"):
                scale = TRUTH_SIGMA["offset"]  # joint angle offset
            elif name.startswith("d_phi"):
                scale = TRUTH_SIGMA["phi"]
            else:
                scale = TRUTH_SIGMA["p"]
            truth[name] = float(rng.normal(0.0, scale))
    else:
        raise ValueError(kind)
    return truth


def tool_poses(calib, kind: str, values: dict, q: pd.DataFrame) -> np.ndarray:
    """(n, 6) tool pose for parameter values ``values`` of class ``kind``."""
    cfg = dict(
        calib.calib_config,
        calib_model=kind,
        param_name=list(values),
        NbSample=len(q),
    )
    var = np.array(list(values.values()), dtype=float)
    pee = calc_updated_fkm(calib.model, calib.data, var, q[JOINTS].to_numpy(), cfg)
    return pee.reshape(6, -1).T


def to_dataset(p: np.ndarray, q: pd.DataFrame) -> pd.DataFrame:
    """Measurements in the calibration CSV layout."""
    df = pd.DataFrame(p, columns=POSE)
    return pd.concat([df, q[JOINTS].reset_index(drop=True)], axis=1)


def fit(train, level, estimation, workdir, robot=None) -> TX40Calibration:
    """Fit the training CSV at ``level`` (or frames only), as a user would."""
    path = Path(workdir) / "train.csv"
    train.to_csv(path, index=False, float_format="%.17g")
    frames_only = level == "registration only"
    calib = TX40Calibration(robot or load_tx40(), str(CONFIG))
    if estimation is not None:
        calib.calib_config["estimation"] = dict(estimation)
    calib.calib_config["data_file"] = str(path)
    calib._data_path = str(path.resolve())
    calib.calib_config["calib_model"] = "joint_offset" if frames_only else level
    calib.calib_config["validation_data_file"] = None
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


@functools.lru_cache(maxsize=None)
def study(design_name: str = "doptimal") -> sim.SimulationStudy:
    """The TX40 synthetic-truth study for one training design."""
    robot = load_tx40()
    # model and config only: the probe is never solved
    probe = TX40Calibration(robot, str(CONFIG))

    def predict(params, kind, q, handle=None):
        return tool_poses(handle or probe, kind, params, q)

    def run_fit(train, fit_level, estimation, workdir):
        calib = fit(train, fit_level, estimation, workdir, robot=robot)
        names = list(calib.calib_config["param_name"])
        level = calib.calib_config["calib_model"]
        std = calib.std_dev
        return sim.FitResult(
            params=dict(zip(names, calib.LM_result.x)),
            std=dict(zip(names, std)) if std is not None else {},
            level=level,
            status=int(calib.LM_result.status),
            nfev=int(calib.LM_result.nfev),
            absorbed=list(calib.calib_config.get("absorbed_param_name", [])),
            handle=calib,
        )

    return sim.SimulationStudy(
        sample_truth=lambda kind, rng: draw_truth(kind, rng, robot.model),
        predict=predict,
        fit=run_fit,
        to_dataset=to_dataset,
        train_postures=design(design_name),
        heldout_postures={"heldout": heldout_postures()},
        noise=NOISE,
        z_params={"joint_offset": lambda n: n.startswith("offset")},
        # tool position only: orientation is fitted but not scored, and its
        # roll-pitch-yaw columns wrap at +-pi
        row_error=lambda e: 1000 * np.linalg.norm(e[:, :3], axis=1),
        train_residual=lambda e: e[:, :3],
    )


def run_case(truth_kind, fit_level, seed, noise, workdir, design_name="doptimal"):
    """Draw, simulate, fit, and judge one case."""
    row = sim.run_case(study(design_name), truth_kind, fit_level, seed, noise, workdir)
    row["design"] = design_name
    return row


def run(seeds, noise, designs=DESIGNS) -> pd.DataFrame:
    frames = []
    for d in designs:
        df = sim.run_grid(
            study(d),
            ("joint_offset", "full_params"),
            {f: (f, None) for f in FITS},
            seeds=seeds,
            noise=noise,
        )
        df["design"] = d
        frames.append(df)
    return pd.concat(frames, ignore_index=True)


def summarize(df: pd.DataFrame):
    """Per (design, truth, noise, fit) means and the pooled z summary."""
    df = df.copy()
    table = df.groupby(["design", "truth", "noise_mm", "fit"], sort=False).agg(
        n_params=("n_params", "median"),
        train_rms_mm=("train_rms_mm", "mean"),
        heldout_rmse_mm=("heldout_rmse_mm", "mean"),
        heldout_max_mm=("heldout_max_mm", "max"),
    )
    z = np.array(
        [v for zs in df.get("z_scores", pd.Series(dtype=object)).dropna() for v in zs.values()]
    )
    zsum = None
    if z.size:
        zsum = {
            "n": int(z.size),
            "mean": float(z.mean()),
            "rms": float(np.sqrt(np.mean(z**2))),
            "max_abs": float(np.abs(z).max()),
        }
    return table, zsum


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2, 3, 4])
    parser.add_argument("--noise", type=float, nargs="+", default=[0.1, 0.5])
    parser.add_argument("--designs", nargs="+", default=list(DESIGNS), choices=DESIGNS)
    parser.add_argument(
        "--design",
        action="store_true",
        help="rerun the D-optimal selection and freeze it in data/truth/",
    )
    args = parser.parse_args(argv)
    if args.design:
        print(json.dumps(select_doptimal(), indent=2))
        return
    print("SIMULATED DATA: a pipeline check, not an estimate of any TX40.\n")
    table, z = summarize(run(args.seeds, args.noise, args.designs))
    print(table.round(3).to_string())
    if z:
        print(
            f"\njoint_offset recovery: {z['n']} z-scores, mean {z['mean']:+.2f}, "
            f"rms {z['rms']:.2f}, max |z| {z['max_abs']:.2f}"
        )


if __name__ == "__main__":
    main()
