"""UR10 hand-eye calibration on synthetic data with a known truth (#99).

The 23 real postures cannot tell recovery from overfitting: nobody knows the
true geometry. Here a truth is drawn, a camera session is simulated at the
real postures, FIGAROH fits it, and the fit is judged against the truth. The
robot-specific part plugs into examples/calibration_harness/simulation.py.

- **Postures:** training is the 23 real postures; the fit is judged on
  ``N_HELD`` postures drawn (seeded, not shipped) uniformly inside the real
  joint ranges, so extrapolation outside the recorded range is not tested.
- **Truth:** the camera and base frames of the real registration-only fit
  (``FRAMES``) plus seeded joint errors of the ``full_params`` class with
  core's default prior sizes (``TRUTH_SIGMA``).
- **Measurements:** the camera pose in the fixed frame from core's
  ``calc_updated_fkm`` (the model the fit uses), with SE3 noise. The noise
  is **assumed, not measured** (the session has no repeated posture, see
  data/README.md): translation noise in the camera frame with sigma
  ``SIGMA_XY`` (image plane) and ``SIGMA_Z`` (depth, the weak direction of a
  monocular camera), rotation noise in the fixed (board) frame of ``TILT``
  about its x and y axes and ``INPLANE`` about z. ``level`` multiplies all
  of them; the grid is ``NOISE_GRID``.
- **Representation:** a measurement is (position m, ``LEVER`` x rotation
  vector from the nominal reference orientation of the same posture), so the
  harness's metre-valued columns hold both; the rotation vector from a
  nearby reference does not wrap, roll-pitch-yaw would. ``to_dataset``
  converts back to the CSV's roll-pitch-yaw.
- **Judging:** held-out position error (mm, the harness metric) and
  orientation error (deg, ``heldout_rot_deg``) of the fit against the
  noise-free truth, training frames carried over; reported separately.
  Gauge: base and tool frames absorb joint errors, so recovery is judged
  in predictions, plus z-scores of the joint parameters for the map family
  only (the structural fits' parameters are base-parameter representatives).
  The z-score assumes isotropic ``noise_std``; with this anisotropic noise
  the map and map_cv z-scores run above 1 (a per-component measurement
  noise is not a core option).

    python examples/ur10/hand_eye_truth.py                 # full grid (slow)
    python examples/ur10/hand_eye_truth.py --seeds 0 --noise 1
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
from scipy.spatial.transform import Rotation

project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from figaroh.calibration.calibration_tools import calc_updated_fkm  # noqa: E402
from figaroh.calibration.parameter import get_fullparam_offset  # noqa: E402

from examples.calibration_harness import simulation as sim  # noqa: E402
from examples.ur10 import hand_eye_heldout as heh  # noqa: E402

# base and tool frames of the registration-only fit of the 23 real postures
# (checked against a fresh fit by tests/test_ur10_hand_eye_truth.py)
FRAMES = {
    "base_px": 0.015375,
    "base_py": 0.03767,
    "base_pz": -0.063407,
    "base_phix": -0.020465,
    "base_phiy": -0.023473,
    "base_phiz": -0.006348,
    "pEEx_1": -0.032893,
    "pEEy_1": -0.123173,
    "pEEz_1": 0.005984,
    "phiEEx_1": -0.514527,
    "phiEEy_1": -0.005359,
    "phiEEz_1": 0.004608,
}
# standard deviations of the drawn joint errors (m, rad): core's defaults
TRUTH_SIGMA = {"offset": 0.02, "p": 1e-3, "phi": 2e-3}
# assumed noise at level 1 (m, rad): see the module docstring
SIGMA_XY = 0.3e-3
SIGMA_Z = 1.5e-3
TILT = np.radians(0.2)
INPLANE = np.radians(0.05)
NOISE_GRID = (0.5, 1.0, 2.0, 4.0)
SEEDS = (0, 1, 2, 3, 4)
LEVER = 0.1  # metres per radian of the rotation columns
N_HELD = 100
HELD_SEED = 2026
TRUTH_KINDS = ("full_params",)
ALL = [True] * 6
STRUCTURAL = {"method": "structural"}
POSE_COLUMNS = heh.POSE

# the seven models of hand_eye_heldout: label -> (fit level, estimation)
FITS = {
    label: (
        "registration only" if frames_only else level,
        estimation,
    )
    for label, (level, frames_only, estimation) in heh.MODELS.items()
}
# fits whose standard deviations can be scored against individual truths
_SCORED = {"map", "map_cv", "cv_subset"}

_judge: dict = {}  # orientation error of the last judged fit (see row_error)


def training_postures() -> pd.DataFrame:
    return pd.read_csv(heh.TRAINING)[heh.JOINTS]


def heldout_postures() -> pd.DataFrame:
    """``N_HELD`` seeded postures inside the real joint ranges."""
    real = training_postures()
    rng = np.random.default_rng(HELD_SEED)
    q = rng.uniform(real.min().to_numpy(), real.max().to_numpy(), (N_HELD, 6))
    return pd.DataFrame(q, columns=heh.JOINTS)


def is_joint_param(name: str) -> bool:
    return name.startswith("d_")


def draw_truth(kind: str, rng: np.random.Generator) -> dict:
    """Frames plus joint errors of the ``full_params`` class, from ``rng``."""
    if kind != "full_params":
        raise ValueError(kind)
    truth = dict(FRAMES)
    for name in get_fullparam_offset(heh.JOINTS):
        if name.startswith("d_phiz_"):
            scale = TRUTH_SIGMA["offset"]  # joint angle offset
        elif name.startswith("d_phi"):
            scale = TRUTH_SIGMA["phi"]
        else:
            scale = TRUTH_SIGMA["p"]
        truth[name] = float(rng.normal(0.0, scale))
    return truth


def _poses(calib, kind: str, values: dict, q: pd.DataFrame) -> np.ndarray:
    """(n, 6) camera pose: position (m) and roll-pitch-yaw (rad)."""
    cfg = dict(
        calib.calib_config,
        calib_model=kind,
        param_name=list(values),
        NbSample=len(q),
        measurability=list(ALL),
        calibration_index=6,
    )
    pee = calc_updated_fkm(
        calib.model,
        calib.data,
        np.array(list(values.values())),
        heh.postures(calib, q),
        cfg,
    )
    return pee.reshape(6, -1).T.copy()


def _rotations(rpy: np.ndarray) -> np.ndarray:
    return np.array([pin.rpy.rpyToMatrix(*r) for r in rpy])


def _reference(calib, q: pd.DataFrame) -> np.ndarray:
    """(n, 3, 3) reference orientations: nominal joints, ``FRAMES``."""
    return _rotations(_poses(calib, "full_params", FRAMES, q)[:, 3:])


def encode(pos: np.ndarray, rot: np.ndarray, ref: np.ndarray) -> np.ndarray:
    """(n, 6) measurement: position, ``LEVER`` x rotation vector from ``ref``."""
    rel = np.einsum("nji,njk->nik", ref, rot)  # ref^T rot
    return np.hstack([pos, LEVER * Rotation.from_matrix(rel).as_rotvec()])


def decode(p: np.ndarray, ref: np.ndarray) -> tuple:
    """Inverse of :func:`encode`: positions and rotation matrices."""
    rel = Rotation.from_rotvec(np.asarray(p)[:, 3:] / LEVER).as_matrix()
    return np.asarray(p)[:, :3].copy(), np.einsum("nij,njk->nik", ref, rel)


def camera_pose(calib, kind: str, values: dict, q: pd.DataFrame) -> np.ndarray:
    """(n, 6) measurement-representation pose of the camera."""
    rpy = _poses(calib, kind, values, q)
    return encode(rpy[:, :3], _rotations(rpy[:, 3:]), _reference(calib, q))


def add_noise(
    pos: np.ndarray, rot: np.ndarray, level: float, rng: np.random.Generator
) -> tuple:
    """SE3 measurement noise (one random draw): see the module docstring.

    Translation noise ``R @ (sxy, sxy, sz) * z`` is in the camera frame;
    rotation noise ``exp(w) @ R`` is in the fixed frame.
    """
    n = len(pos)
    z = rng.normal(0.0, 1.0, (n, 6)) * level
    cam = z[:, :3] * np.array([SIGMA_XY, SIGMA_XY, SIGMA_Z])
    tilt = z[:, 3:] * np.array([TILT, TILT, INPLANE])
    pos = pos + np.einsum("nij,nj->ni", rot, cam)
    rot = np.einsum("nij,njk->nik", Rotation.from_rotvec(tilt).as_matrix(), rot)
    return pos, rot


def to_dataset(calib, p: np.ndarray, q: pd.DataFrame) -> pd.DataFrame:
    """Measurements in the CSV layout (position, roll-pitch-yaw, joints)."""
    pos, rot = decode(p, _reference(calib, q))
    rpy = np.array([pin.rpy.matrixToRpy(r) for r in rot])
    pose = pd.DataFrame(np.hstack([pos, rpy]), columns=POSE_COLUMNS)
    return pd.concat([pose, q.reset_index(drop=True)], axis=1)


def _position_error(e: np.ndarray) -> np.ndarray:
    """Position error (mm) per row; the orientation error is kept aside."""
    rot_deg = np.degrees(np.linalg.norm(e[:, 3:], axis=1) / LEVER)
    _judge["rot_deg"] = rot_deg
    return 1000 * np.linalg.norm(e[:, :3], axis=1)


@functools.lru_cache(maxsize=1)
def study() -> sim.SimulationStudy:
    """The UR10 hand-eye synthetic-truth study on the shared harness."""
    probe = heh.fit(heh.TRAINING, "full_params", True, STRUCTURAL)  # model, config

    def predict(params, kind, q, handle=None):
        return camera_pose(handle or probe, kind, params, q)

    def simulate(study_, truth, kind, q, level, seed):
        rng = np.random.default_rng(10_000 + seed)
        rpy = _poses(probe, kind, truth, q)
        ref = _reference(probe, q)
        pos, rot = add_noise(rpy[:, :3], _rotations(rpy[:, 3:]), level, rng)
        return encode(pos, rot, ref), q

    def fit(train, fit_level, estimation, workdir):
        path = Path(workdir) / "train.csv"
        train.to_csv(path, index=False)
        frames_only = fit_level == "registration only"
        level = "full_params" if frames_only else fit_level
        calib = heh.fit(path, level, frames_only, estimation)
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
        sample_truth=lambda kind, rng: draw_truth(kind, rng),
        predict=predict,
        fit=fit,
        to_dataset=lambda p, q: to_dataset(probe, p, q),
        train_postures=training_postures(),
        heldout_postures={"in-range": heldout_postures()},
        simulate=simulate,
        z_params={"full_params": is_joint_param},
        row_error=_position_error,
    )


def run_case(
    truth_kind, fit_level, seed, noise, workdir, estimation=None, label=None
) -> dict:
    """One case (``sim.run_case``), plus the held-out orientation error (deg)."""
    out = sim.run_case(
        study(), truth_kind, fit_level, seed, noise, workdir, estimation, label
    )
    out["heldout_rot_deg"] = float(np.sqrt(np.mean(_judge["rot_deg"] ** 2)))
    return out


def run(seeds=SEEDS, noise=None, models=None) -> pd.DataFrame:
    """The grid: truth x noise x fit x seed (``models``: labels of ``FITS``)."""
    import tempfile

    labels = list(FITS) if models is None else list(models)
    noise = NOISE_GRID if noise is None else noise
    rows = []
    with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(
        io.StringIO()
    ):
        for t in TRUTH_KINDS:
            for n in noise:
                for label in labels:
                    level, est = FITS[label]
                    for s in seeds:
                        rows.append(run_case(t, level, s, n, tmp, est, label))
    return pd.DataFrame(rows)


def summary(df: pd.DataFrame) -> tuple:
    """Per (noise, fit) means (position and orientation apart) and z summary."""
    table, z = sim.summarize(df, {})
    rot = df.groupby(["truth", "noise_mm", "fit"], sort=False)["heldout_rot_deg"].mean()
    return table.join(rot), z


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--seeds", type=int, nargs="+", default=list(SEEDS))
    parser.add_argument("--noise", type=float, nargs="+", default=None)
    parser.add_argument("--models", nargs="+", default=None, choices=list(FITS))
    args = parser.parse_args(argv)
    table, z = summary(run(args.seeds, args.noise, args.models))
    print(
        "Assumed noise (no repeats in the session): camera-frame "
        f"{SIGMA_XY * 1e3:.1f}/{SIGMA_Z * 1e3:.1f} mm, board-frame "
        f"{np.degrees(TILT):.2f}/{np.degrees(INPLANE):.2f} deg, times the level"
    )
    print(table.round(3).to_string())
    if z:
        print(
            f"\nfull_params joint-level recovery (fits with their own std): "
            f"{z['n']} z-scores, mean {z['mean']:+.2f}, rms {z['rms']:.2f}, "
            f"max |z| {z['max_abs']:.2f} (isotropic-noise z-scores)"
        )


if __name__ == "__main__":
    main()
