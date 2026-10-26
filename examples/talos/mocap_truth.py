"""TALOS torso-arm mocap calibration on synthetic data with a known truth (#172).

The fixture of the study in mocap_heldout.py, run before any real fit: the
truth is drawn, measurements are simulated at the real postures, every model
of ``mocap_heldout.MODELS`` is fitted on them, and each fit is judged against
the noise-free truth. On the harness (examples/calibration_harness/
simulation.py), as examples/tiago_pro/calibration_truth.py.

- **Truth:** synthetic frames (``FRAMES``: ``B``, base_link in the waist body
  frame, and ``T``, the end-effector body on arm_left_7_link; plausible
  sizes, not fitted to any real run) and ``T_RIGHT`` (the end-effector body
  on arm_right_7_link), plus seeded placement errors of the ``full_params``
  class on all 16 joints (torso, both arms; ``TRUTH_SIGMA``, the joint's own
  axis at the encoder-offset size).
- **Training:** the 56 postures of the 13:54 left run, measured as
  ``B * FK_left * T`` with seeded Gaussian noise.
- **Held out, noise-free:**

  - ``left``: ``N_LEFT`` postures drawn uniformly inside the training range
    (seeded): the within-session question the k-fold plan asks, with the
    training frames carried over;
  - ``right``: the 39 postures of the 14:18 validation run (the confirmation
    run is not used), measured as ``B * FK_right * T_RIGHT * S_k`` with the
    validation run's fit-free flip clusters (``mocap_heldout.flip_clusters``)
    and synthetic flips ``FLIPS`` about a centroid ``CENTROID``. The fit's
    torso is carried, the right arm is nominal, and ``B``, ``T`` and the flips
    are re-registered on the noise-free truth: the primary metric of the
    real study, and a check that the flip repair recovers its transforms.

- **Measurements** are (n, 6): position (m) and the rotation vector from the
  nominal pose (``FRAMES``, the same flip) times ``LEVER`` (m), so the
  harness's one noise level in mm is about the same on both: 0.17 mm is
  1.7 mrad, the training run's torso-registration noise.
- **Noise grid:** ``NOISE`` mm (the training run's 0.17, then 1 and 3).
- **Gauge:** the score is the held-out position error against the truth, in
  identifiable coordinates. z-scores are only taken for the map family,
  whose standard deviations refer to the individual torso and left-arm
  parameters; structural fits return base-parameter representatives.

    python mocap_truth.py                       # default grid, from examples/talos
    python mocap_truth.py --seeds 0 1 --noise 0.17
"""

from __future__ import annotations

import argparse
import functools
import itertools
import sys
import tempfile
from pathlib import Path

import numpy as np
import pandas as pd
import pinocchio as pin

project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from figaroh.calibration.parameter import BASE_TPL, get_fullparam_offset  # noqa: E402

from examples.calibration_harness import simulation as sim  # noqa: E402
from examples.talos import mocap_heldout as mh  # noqa: E402

FRAMES = {
    "base_px": 0.012,
    "base_py": -0.021,
    "base_pz": -0.048,
    "base_phix": 0.020,
    "base_phiy": -0.012,
    "base_phiz": 0.031,
    "pEEx_1": 0.004,
    "pEEy_1": -0.008,
    "pEEz_1": -0.105,
    "phiEEx_1": 0.150,
    "phiEEy_1": -0.080,
    "phiEEz_1": 0.400,
}
T_RIGHT = mh.from_xyzrpy([-0.006, 0.010, -0.098, -0.120, 0.050, -0.350])
# synthetic flips of the right run's clusters (cluster 1, 2, ...): rotation
# vectors in the end-effector body frame, as the real ones look (+-90 deg
# about one axis, 180 deg about another)
FLIPS = [
    np.pi / 2 * np.array([-0.53, 0.10, 0.84]) / np.linalg.norm([-0.53, 0.10, 0.84]),
    -np.pi / 2 * np.array([-0.53, 0.10, 0.84]) / np.linalg.norm([-0.53, 0.10, 0.84]),
    np.pi * np.array([-0.19, 0.94, -0.27]) / np.linalg.norm([-0.19, 0.94, -0.27]),
]
CENTROID = np.array([0.010, -0.015, 0.020])
TRUTH_SIGMA = {"offset": 0.02, "p": 1e-3, "phi": 2e-3}
LEVER = mh.LEVER
#: ``right_exact``: the same draw with the right arm's errors zeroed, so the
#: right run sees the torso alone: the most carrying the torso can show
TRUTH_KINDS = ("full_params", "right_exact")
SEEDS = (0, 1, 2, 3, 4)
NOISE = (0.17, 1.0, 3.0)
N_LEFT = 40
RIGHT = mh.SETS[1][1]  # the validation run's postures
if mh.SETS[1][0] != "validation":
    raise ValueError("the fixture uses the validation run's postures only")
ALL_JOINTS = [*mh.joint_names("left"), *mh.joint_names("right")[2:]]
# map-family fits report standard deviations of individual parameters
_SCORED = {"map", "map_cv", "cv_subset"}


def postures(name: str) -> pd.DataFrame:
    df = pd.read_csv(mh.MOCAP / name)
    return df[mh.joint_names(mh.ARM[name])]


def left_heldout(n: int = N_LEFT, seed: int = 0) -> pd.DataFrame:
    """``n`` left postures uniform inside the training range (seeded)."""
    train = postures(mh.TRAINING)
    rng = np.random.default_rng(seed)
    lo, hi = train.min().to_numpy(), train.max().to_numpy()
    return pd.DataFrame(rng.uniform(lo, hi, (n, len(lo))), columns=train.columns)


def draw_truth(kind: str, rng: np.random.Generator) -> dict:
    """``FRAMES`` plus ``full_params`` errors on every joint, from ``rng``
    (``right_exact``: the same draw, right-arm errors set to zero)."""
    if kind not in TRUTH_KINDS:
        raise ValueError(kind)
    model = mh.robot().model
    truth = dict(FRAMES)
    for name in get_fullparam_offset(ALL_JOINTS):
        joint = mh.joint_of(name)
        axis = model.joints[model.getJointId(joint)].shortname()[-1].lower()
        if name.startswith(f"d_phi{axis}_"):
            scale = TRUTH_SIGMA["offset"]  # the joint's own axis: angle offset
        elif name.startswith("d_phi"):
            scale = TRUTH_SIGMA["phi"]
        else:
            scale = TRUTH_SIGMA["p"]
        truth[name] = float(rng.normal(0.0, scale))
        if kind == "right_exact" and "arm_right" in joint:
            truth[name] = 0.0
    return truth


def is_joint_param(name: str) -> bool:
    return not mh.is_frame(name)


# ── Poses as (n, 6) arrays ────────────────────────────────────────────────────


@functools.lru_cache(maxsize=None)
def right_labels() -> tuple:
    """The validation run's repairable flip clusters (fit-free)."""
    df = pd.read_csv(mh.MOCAP / RIGHT)
    return tuple(mh.repairable(mh.flip_clusters(mh.robot().model, df, "right")))


def _flip(k: int) -> pin.SE3:
    if k == 0:
        return pin.SE3.Identity()
    R = pin.exp3(FLIPS[k - 1])
    return pin.SE3(R, (np.eye(3) - R) @ CENTROID)


def _q(df: pd.DataFrame, arm: str) -> np.ndarray:
    return mh.configurations(mh.robot().model, df, arm)


def left_poses(params: dict, level: str, q: pd.DataFrame) -> list:
    return mh.chain_poses(mh._nominal_calibrator(), params, _q(q, "left"), "left", level)


def right_truth(truth: dict, q: pd.DataFrame) -> list:
    """``B * FK_right * T_RIGHT * S_k`` of the truth (frames from ``truth``)."""
    joint = {n: v for n, v in truth.items() if not mh.is_frame(n)}
    B = mh.from_xyzrpy([truth[n] for n in BASE_TPL])
    F = mh.chain_poses(mh._nominal_calibrator(), joint, _q(q, "right"), "right",
                       "full_params")
    return [B * f * T_RIGHT * _flip(k) for f, k in zip(F, right_labels())]


def _reference(q: pd.DataFrame, side: str) -> list:
    """Nominal joints and frames (and, on the right, the synthetic flip)."""
    if side == "left":
        return left_poses(FRAMES, "full_params", q)
    nominal = {n: v for n, v in FRAMES.items() if n in BASE_TPL}
    return right_truth(nominal, q)


def to_array(poses: list, ref: list) -> np.ndarray:
    return np.array([
        np.r_[p.translation, LEVER * pin.log3(r.rotation.T @ p.rotation)]
        for p, r in zip(poses, ref)
    ])


def from_array(a: np.ndarray, ref: list) -> list:
    return [pin.SE3(r.rotation @ pin.exp3(v[3:] / LEVER), v[:3]) for v, r in zip(a, ref)]


def to_dataset(a: np.ndarray, q: pd.DataFrame) -> pd.DataFrame:
    """The shipped layout: waist at the identity, end-effector = measurement."""
    poses = from_array(a, _reference(q, "left"))
    df = q.reset_index(drop=True).copy()
    for c, v in zip(("x", "y", "z", "qx", "qy", "qz", "qw"), np.eye(7)[-1]):
        df[f"waist_{c}"] = v
    xyzq = np.array([pin.SE3ToXYZQUAT(p) for p in poses])
    for i, c in enumerate(("x", "y", "z", "qx", "qy", "qz", "qw")):
        df[f"endeffector_{c}"] = xyzq[:, i]
    return df


# ── Study ─────────────────────────────────────────────────────────────────────


@functools.lru_cache(maxsize=1)
def study() -> sim.SimulationStudy:
    """The TALOS mocap synthetic-truth study on the shared harness.

    ``predict`` on the ``right`` set needs the truth it is judged against (the
    re-registration targets it): ``sample_truth`` keeps the last draw, and
    the harness draws the truth before it predicts anything.
    """
    state = {}
    left = left_heldout()
    right = postures(RIGHT)
    held = {"left": left, "right": right}

    def sample_truth(kind, rng):
        state["truth"] = draw_truth(kind, rng)
        return state["truth"]

    def side(q):
        return "right" if q is right or "arm_right_1_joint" in q else "left"

    def predict(params, kind, q, handle=None):
        kind = "full_params" if kind in TRUTH_KINDS else kind
        s = side(q)
        ref = _reference(q, s)
        if s == "left":
            return to_array(left_poses(params, kind, q), ref)
        Y = right_truth(state["truth"], q)
        if handle is None:  # the truth itself
            return to_array(Y, ref)
        joint = mh.torso_params(params)
        F = mh.chain_poses(handle, joint, _q(q, "right"), "right", kind)
        B0 = mh.from_xyzrpy([params[n] for n in BASE_TPL])
        re = mh.register(F, Y, np.array(right_labels()), B0)
        return to_array(re["pred"], ref)

    def fit(train, _level, model, workdir):
        path = Path(workdir) / "train.csv"
        train.to_csv(path, index=False)
        level, frames_only, est = mh.MODELS[model["label"]]
        calib = mh.fit(level, frames_only, path, estimation=est)
        names = list(calib.calib_config["param_name"])
        method = est.get("method", "structural")
        if method in _SCORED and calib.std_dev is not None:
            std = dict(zip(names, calib.std_dev))
        else:
            std = dict.fromkeys(names, 0.0)
        return sim.FitResult(
            params=dict(zip(names, map(float, calib.LM_result.x))),
            std=std,
            level=level,
            status=int(calib.LM_result.status),
            nfev=int(calib.LM_result.nfev),
            absorbed=list(calib.calib_config.get("absorbed_param_name", [])),
            handle=calib,
        )

    train = postures(mh.TRAINING)
    strata = {
        "left": {"left": np.ones(len(left), bool), "right": np.zeros(len(left), bool)},
        "right": {"left": np.zeros(len(right), bool), "right": np.ones(len(right), bool)},
    }
    flipped = np.array(right_labels()) > 0
    strata["left"] |= {"flipped": np.zeros(len(left), bool)}
    strata["right"] |= {"flipped": flipped}
    return sim.SimulationStudy(
        sample_truth=sample_truth,
        predict=predict,
        fit=fit,
        to_dataset=to_dataset,
        train_postures=train,
        heldout_postures=held,
        strata=lambda name: strata[name],
        z_params=dict.fromkeys(TRUTH_KINDS, is_joint_param),
        row_error=lambda e: 1000 * np.linalg.norm(e[:, :3], axis=1),
        train_residual=lambda e: e[:, :3],
    )


def run(seeds=SEEDS, noise=NOISE, models=None, truths=TRUTH_KINDS) -> pd.DataFrame:
    """The grid: truth x noise x every model of ``mocap_heldout.MODELS`` x seed.

    The harness scores z-scores when the fit level is the truth's, so every
    case is run at the truth kind and the model label travels in the
    estimation slot (``fit`` looks it up in ``MODELS``); fits without their
    own standard deviations report zero and are not scored.
    """
    labels = list(mh.MODELS) if models is None else list(models)
    rows = []
    with tempfile.TemporaryDirectory() as tmp:
        for t, n, label, s in itertools.product(truths, noise, labels, seeds):
            rows.append(sim.run_case(
                study(), t, t, s, n, tmp, estimation={"label": label}, label=label,
            ))
    return pd.DataFrame(rows)


def summarize(df: pd.DataFrame):
    return sim.summarize(df, {"left_mm": "left", "right_mm": "right",
                              "right_flipped_mm": "flipped"})


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--seeds", type=int, nargs="+", default=list(SEEDS))
    parser.add_argument("--noise", type=float, nargs="+", default=list(NOISE))
    parser.add_argument("--models", nargs="+", default=None)
    parser.add_argument("--truths", nargs="+", default=list(TRUTH_KINDS))
    parser.add_argument("--csv", default=None, help="also write every case here")
    args = parser.parse_args(argv)
    df = run(args.seeds, args.noise, args.models, args.truths)
    if args.csv:
        df.to_csv(args.csv, index=False)
    table, _ = summarize(df)
    print(table.round(3).to_string())
    for (truth, label), part in df.groupby(["truth", "fit"], sort=False):
        zs = [v for z in part.get("z_scores", pd.Series(dtype=object)).dropna()
              for v in z.values()]
        if zs:
            z = np.array(zs)
            print(f"{truth} / {label}: {len(z)} z-scores, mean {z.mean():+.2f}, "
                  f"rms {np.sqrt(np.mean(z**2)):.2f}, max |z| {np.abs(z).max():.2f}")


if __name__ == "__main__":
    main()
