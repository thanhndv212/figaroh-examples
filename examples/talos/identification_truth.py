"""TALOS torso-arm dynamics identification on simulated data (synthetic truth).

There are no TALOS dynamics recordings. This study draws a known truth,
simulates the joint torques it would produce on an excitation trajectory,
identifies them with FIGAROH's ``BaseIdentification`` and judges the fit
against the truth. Everything here is **simulated**: it checks the pipeline
and the excitation at a stated noise level, it does not estimate the
dynamics of any TALOS.

- Chain: ``torso_1``, ``torso_2`` and the seven left-arm joints, with the rest
  of the body locked at ``REST`` (a Pinocchio reduced model: the head, right
  arm and grippers merge into the links they hang from).
- Truth: each link inertia perturbed consistently from the URDF (seeded),
  plus viscous and Coulomb friction and reflected actuator inertia drawn in
  assumed ranges (``FRICTION``, ``ACTUATOR``), in core's regressor
  convention ``tau = RNEA + fv dq + fs sign(dq) + ia ddq``.
- Data: one period of a finite Fourier series per joint (``EXCITATION``),
  analytic q, dq, ddq, checked against the TALOS SRDF self-collision pairs,
  the URDF limits and the effort limits. Held-out: a second series (another
  seed). Torque noise N(0, level x RMS torque) per joint, noise seed
  10 000 + seed. Positions and derivatives are exact.
- Score: held-out torque against the noise-free truth, and the base
  parameters against the truth's base parameters (z-scores from the
  per-joint weighted covariance).

Docs: docs/development/talos-identification-synthetic-truth.md.

Usage (from ``examples/talos``)::

    python identification_truth.py              # grid: seeds 0-4, noise 2 % and 10 %
    python identification_truth.py --seeds 0 --noise 0.05
"""

from __future__ import annotations

import argparse
import contextlib
import io
import sys
from functools import lru_cache
from pathlib import Path

import numpy as np
import pandas as pd
import pinocchio as pin

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1]))

from figaroh.identification.base_identification import BaseIdentification  # noqa: E402
from figaroh.tools.robot import load_robot  # noqa: E402

from examples.candidate_postures import _collision_checker  # noqa: E402

URDF = HERE / "urdf" / "talos_full_v2.urdf"
MODELS = HERE.parents[1] / "models"
SRDF = MODELS / "talos_description" / "srdf" / "talos.srdf"
CONFIG = HERE / "config" / "talos_unified_config.yaml"

CHAIN = ["torso_1_joint", "torso_2_joint"] + [
    f"arm_left_{i}_joint" for i in range(1, 8)
]
SAMPLE_RATE_HZ = 200.0
EXCITATION = {
    "fundamental_hz": 0.1,  # one 10 s period
    "harmonics": 5,
    "amplitude_fraction": 0.6,  # of the half-range less MARGIN
    "max_amplitude_rad": 0.8,
    "velocity_fraction": 0.6,  # of the URDF velocity limit
    "effort_fraction": 0.9,  # of the URDF effort limit, checked
}
MARGIN = 0.1  # rad from each joint limit
COLLISION_STRIDE = 10  # check self-collision every 10th sample (20 Hz)
# locked joints: q0 except the right arm, abducted and bent so it clears the
# right thigh (at q0 the arms hang into the legs' collision meshes)
REST = {"arm_right_2_joint": -0.5, "arm_right_4_joint": -1.0}
TRAJECTORY_SEEDS = {"train": 0, "heldout": 1}  # first seeds searched
MAX_SEED_TRIES = 200

TRUTH_SPREAD = {"mass": 0.10, "com_m": 0.02, "moments": 0.20, "axes_rad": 0.20}
# Assumed ranges, as fractions of each joint's URDF effort limit: Coulomb
# friction 1-4 %, viscous friction giving 1-4 % at 1 rad/s, actuator inertia
# 0.5-1.5 g m^2 per Nm (0.1-0.3 kg m^2 on the 200 Nm torso joints).
FRICTION = {"fs": (0.01, 0.04), "fv": (0.01, 0.04)}
ACTUATOR = {"ia": (0.5e-3, 1.5e-3)}

NOISE_LEVELS = (0.02, 0.10)  # torque noise sigma / per-joint RMS torque
SEEDS = range(5)

FIT_CONFIG = {
    # the study fits joint torques only; no FT sensor, no joint offsets
    "is_external_wrench": False,
    "force_torque": None,
    "has_friction": True,
    "has_actuator_inertia": True,
    "has_joint_offset": False,
    "has_coupled_wrist": False,
    "wls": False,
}

LABEL = "SIMULATED DATA: a pipeline check, not an estimate of any TALOS."


class _Robot:
    def __init__(self, model: pin.Model):
        self.model = model
        self.data = model.createData()
        self.q0 = pin.neutral(model)
        self.v0 = np.zeros(model.nv)


# --- model ------------------------------------------------------------------


@lru_cache(maxsize=None)
def full_robot():
    return load_robot(str(URDF), package_dirs=str(MODELS), load_by_urdf=True)


@lru_cache(maxsize=None)
def rest_posture() -> np.ndarray:
    robot = full_robot()
    q = np.asarray(robot.q0, dtype=float).copy()
    for name, value in REST.items():
        q[robot.model.joints[robot.model.getJointId(name)].idx_q] = value
    return q


@lru_cache(maxsize=None)
def chain_model() -> pin.Model:
    """The 9-DOF torso-arm model, the rest of TALOS locked at ``REST``."""
    m = full_robot().model
    lock = [j for j in range(1, m.njoints) if m.names[j] not in CHAIN]
    reduced = pin.buildReducedModel(m, lock, rest_posture())
    assert list(reduced.names)[1:] == CHAIN
    return reduced


def chain_idx_q() -> list:
    m = full_robot().model
    return [m.joints[m.getJointId(j)].idx_q for j in CHAIN]


# --- truth ------------------------------------------------------------------


def draw_truth(seed: int) -> dict:
    """Seeded truth: consistent link inertias, friction and actuator inertia.

    Inertias as in ``examples/ur10/identification_truth.py``: scaling the
    eigenvalues of the second moment of mass and rotating its eigenvectors
    keeps the pseudo-inertia positive definite.
    """
    model = chain_model()
    rng = np.random.default_rng(seed)
    inertias = []
    for jid in range(1, model.njoints):
        nominal = model.inertias[jid]
        m = nominal.mass * np.exp(rng.normal(0.0, TRUTH_SPREAD["mass"]))
        c = nominal.lever + rng.normal(0.0, TRUTH_SPREAD["com_m"], 3)
        Ic = nominal.inertia
        S = 0.5 * np.trace(Ic) * np.eye(3) - Ic
        s, V = np.linalg.eigh(S)
        s = s * np.exp(rng.normal(0.0, TRUTH_SPREAD["moments"], 3))
        R = pin.exp3(rng.normal(0.0, TRUTH_SPREAD["axes_rad"], 3)) @ V
        S = (R * s) @ R.T
        S = 0.5 * (S + S.T)
        inertias.append(pin.Inertia(m, c, np.trace(S) * np.eye(3) - S))
    effort = model.effortLimit
    return {
        "inertias": inertias,
        "fs": rng.uniform(*FRICTION["fs"], model.nv) * effort,
        "fv": rng.uniform(*FRICTION["fv"], model.nv) * effort,
        "ia": rng.uniform(*ACTUATOR["ia"], model.nv) * effort,
    }


def truth_model(truth: dict) -> pin.Model:
    out = pin.Model(chain_model())
    for jid, inertia in enumerate(truth["inertias"], start=1):
        out.inertias[jid] = inertia
    return out


def truth_torques(truth: dict, q, dq, ddq) -> np.ndarray:
    """Joint torques of the truth, (n, nv), in core's regressor convention."""
    model = truth_model(truth)
    data = model.createData()
    tau = np.array([pin.rnea(model, data, *x).copy() for x in zip(q, dq, ddq)])
    return tau + truth["fv"] * dq + truth["fs"] * np.sign(dq) + truth["ia"] * ddq


def truth_standard_vector(truth: dict, names: list) -> np.ndarray:
    """The truth in core's standard-parameter order and naming."""
    from figaroh.identification.identification_tools import get_standard_parameters

    std = get_standard_parameters(truth_model(truth), FIT_CONFIG | {"act_idxv": []})
    values = dict(std)
    for j, joint in enumerate(CHAIN):
        values[f"fv_{joint}"] = truth["fv"][j]
        values[f"fs_{joint}"] = truth["fs"][j]
        values[f"Ia_{joint}"] = truth["ia"][j]
    return np.array([float(values[n]) for n in names])


# --- excitation -------------------------------------------------------------


def _fourier(a, b, center, t):
    kw = np.arange(1, a.shape[1] + 1) * 2 * np.pi * EXCITATION["fundamental_hz"]
    S, C = np.sin(np.outer(t, kw)), np.cos(np.outer(t, kw))
    q = center + S @ (a / kw).T - C @ (b / kw).T
    return q, C @ a.T + S @ b.T, -S @ (a * kw).T + C @ (b * kw).T


def sample_times() -> np.ndarray:
    n = int(round(SAMPLE_RATE_HZ / EXCITATION["fundamental_hz"]))
    return np.arange(n) / SAMPLE_RATE_HZ


def _candidate(seed: int):
    """Fourier series of one seed, scaled into the limits; (q, dq, ddq)."""
    model = chain_model()
    lo = model.lowerPositionLimit + MARGIN
    hi = model.upperPositionLimit - MARGIN
    center = 0.5 * (lo + hi)
    amplitude = np.minimum(
        EXCITATION["amplitude_fraction"] * 0.5 * (hi - lo),
        EXCITATION["max_amplitude_rad"],
    )
    rng = np.random.default_rng(seed)
    K = EXCITATION["harmonics"]
    a, b = rng.uniform(-1, 1, (model.nv, K)), rng.uniform(-1, 1, (model.nv, K))
    t = sample_times()
    q, dq, _ = _fourier(a, b, center, t)
    gain = np.minimum(
        amplitude / np.abs(q - center).max(0),
        EXCITATION["velocity_fraction"] * model.velocityLimit / np.abs(dq).max(0),
    )
    return _fourier(a * gain[:, None], b * gain[:, None], center, t)


def infeasible(q, dq, ddq) -> str:
    """Why a trajectory is rejected, or "" when it is feasible."""
    model = chain_model()
    if np.any(q < model.lowerPositionLimit) or np.any(q > model.upperPositionLimit):
        return "position limit"
    nominal = {"inertias": list(model.inertias)[1:]} | {
        k: np.zeros(model.nv) for k in ("fs", "fv", "ia")
    }
    tau = truth_torques(nominal, q, dq, ddq)
    if np.any(np.abs(tau) > EXCITATION["effort_fraction"] * model.effortLimit):
        return "effort limit"
    robot = full_robot()
    collides = _collision_checker(robot, "srdf", str(SRDF))
    full_q = np.tile(rest_posture(), (len(q), 1))
    full_q[:, chain_idx_q()] = q
    if any(collides(x) for x in full_q[::COLLISION_STRIDE]):
        return "self-collision"
    return ""


@lru_cache(maxsize=None)
def trajectory(split: str) -> dict:
    """The first feasible Fourier series from ``TRAJECTORY_SEEDS[split]``.

    Seeds are searched in steps of 2 so the two splits never share one.
    """
    first = TRAJECTORY_SEEDS[split]
    rejected = {}
    for k in range(MAX_SEED_TRIES):
        seed = first + 2 * k
        q, dq, ddq = _candidate(seed)
        why = infeasible(q, dq, ddq)
        if not why:
            return {"seed": seed, "rejected": rejected, "t": sample_times(),
                    "q": q, "dq": dq, "ddq": ddq}
        rejected[why] = rejected.get(why, 0) + 1
    raise RuntimeError(f"no feasible {split} trajectory in {MAX_SEED_TRIES} seeds: {rejected}")


# --- identification ---------------------------------------------------------


class SimulatedIdentification(BaseIdentification):
    """``BaseIdentification`` on an in-memory simulated trajectory.

    The derivatives are exact, so the base pipeline's filter is skipped.
    """

    def __init__(self, data: dict):
        self._sim = data
        super().__init__(_Robot(chain_model()), str(CONFIG))
        ps = self.identif_config
        ps.update(FIT_CONFIG)
        ps["ts"] = 1.0 / SAMPLE_RATE_HZ
        ps["active_joints"] = list(CHAIN)
        ps["act_Jid"] = [self.model.getJointId(j) for j in CHAIN]
        ps["act_J"] = [self.model.joints[j] for j in ps["act_Jid"]]
        ps["act_idxq"] = [J.idx_q for J in ps["act_J"]]
        ps["act_idxv"] = [J.idx_v for J in ps["act_J"]]
        ps["idx_act_joints"] = list(ps["act_idxv"])

    def load_trajectory_data(self, data_source=None):
        d = self._sim
        return {
            "timestamps": d["t"].reshape(-1, 1),
            "positions": d["q"],
            "velocities": d["dq"],
            "accelerations": d["ddq"],
            "torques": d["tau"],
        }

    def process_kinematics_data(self, filter_config=None):
        self.processed_data = {
            k: self.raw_data[k]
            for k in ("timestamps", "positions", "velocities", "accelerations")
        }


def simulate(truth: dict, split: str, level: float, seed: int) -> dict:
    """Noise-free and noisy torques of ``truth`` on the split's trajectory."""
    traj = trajectory(split)
    tau = truth_torques(truth, traj["q"], traj["dq"], traj["ddq"])
    offset = {"train": 0, "heldout": 1}[split]
    rng = np.random.default_rng(10_000 + seed + 100_000 * offset)
    sigma = level * np.sqrt(np.mean(tau**2, axis=0))
    return traj | {"tau_true": tau, "tau": tau + rng.normal(0, 1, tau.shape) * sigma,
                   "sigma": sigma}


def base_regressor(ident: SimulatedIdentification, q, dq, ddq) -> np.ndarray:
    """The fit's base regressor rebuilt on another trajectory (joint-major)."""
    from figaroh.tools.regressor import build_regressor_basic, build_regressor_reduced

    W = build_regressor_basic(_Robot(chain_model()), q, dq, ddq, ident.identif_config)
    W_e = build_regressor_reduced(W, ident._idx_eliminated)
    return W_e[:, ident._base_indices]


def run_case(seed: int, level: float) -> dict:
    truth = draw_truth(seed)
    train = simulate(truth, "train", level, seed)
    ident = SimulatedIdentification(train)
    with contextlib.redirect_stdout(io.StringIO()):  # core's quality report
        ident.initialize()
        ident.solve(decimate=False, plotting=False, save_results=False)

    W_b = ident.dynamic_regressor_base
    phi = np.asarray(ident.phi_base, dtype=float)
    nv, n = chain_model().nv, len(train["t"])

    # truth's base parameters: the base regressor reproduces the noise-free
    # truth torques exactly, so solve it on them
    phi_truth, *_ = np.linalg.lstsq(W_b, train["tau_true"].T.ravel(), rcond=None)
    # per-joint weighted covariance of the OLS estimate (sandwich form)
    res = (ident.tau_noised - W_b @ phi).reshape(nv, n)
    var = np.repeat(np.var(res, axis=1, ddof=0) * n / (n - W_b.shape[1] / nv), n)
    WtW_inv = np.linalg.pinv(W_b.T @ W_b)
    cov = WtW_inv @ (W_b.T * var) @ W_b @ WtW_inv
    se = np.sqrt(np.diag(cov))
    # at zero noise the SE vanishes and z is meaningless
    z = (phi - phi_truth) / se if level > 0 else np.array([])

    held = simulate(truth, "heldout", level, seed)
    pred = (base_regressor(ident, held["q"], held["dq"], held["ddq"]) @ phi)
    pred = pred.reshape(nv, -1).T
    rmse = np.sqrt(np.mean((pred - held["tau_true"]) ** 2, axis=0))
    scale = np.sqrt(np.mean(held["tau_true"] ** 2, axis=0))
    train_rms = np.sqrt(np.mean(res**2, axis=1)) / np.sqrt(np.mean(train["tau_true"] ** 2, axis=0))
    return {
        "seed": seed,
        "noise": level,
        "n_base": len(phi),
        "cond": float(np.linalg.cond(W_b)),
        "train_nrms_pct": 100 * float(np.mean(train_rms)),
        "heldout_nrmse_pct": 100 * float(np.mean(rmse / scale)),
        "heldout_nrmse_max_pct": 100 * float(np.max(rmse / scale)),
        "heldout_rmse_nm": rmse,
        "heldout_nrmse_joint_pct": 100 * rmse / scale,
        "z": z,
        "base_names": list(ident.params_base),
    }


def run(seeds=SEEDS, levels=NOISE_LEVELS) -> pd.DataFrame:
    return pd.DataFrame([run_case(s, lv) for lv in levels for s in seeds])


def summarize(df: pd.DataFrame) -> tuple:
    table = df.groupby("noise")[
        ["n_base", "cond", "train_nrms_pct", "heldout_nrmse_pct", "heldout_nrmse_max_pct"]
    ].agg({"n_base": "first", "cond": "first", "train_nrms_pct": "mean",
           "heldout_nrmse_pct": "mean", "heldout_nrmse_max_pct": "max"})
    z = df[df["noise"] > 0].groupby("noise")["z"].apply(
        lambda s: np.concatenate(list(s))
    )
    zstats = pd.DataFrame({
        "n": z.apply(len),
        "mean": z.apply(np.mean),
        "rms": z.apply(lambda v: np.sqrt(np.mean(v**2))),
        "max_abs": z.apply(lambda v: np.abs(v).max()),
    })
    joints = df.groupby("noise")["heldout_nrmse_joint_pct"].apply(
        lambda s: pd.Series(np.mean(np.stack(list(s)), axis=0), index=CHAIN)
    ).unstack()
    return table, zstats, joints


def main(argv=None) -> None:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--seeds", type=int, nargs="+", default=list(SEEDS))
    p.add_argument("--noise", type=float, nargs="+", default=list(NOISE_LEVELS),
                   help="torque noise sigma as a fraction of each joint's RMS torque")
    args = p.parse_args(argv)

    print(LABEL)
    for split in ("train", "heldout"):
        tr = trajectory(split)
        print(f"{split} trajectory: Fourier seed {tr['seed']}, rejected {tr['rejected']}")
    table, z, joints = summarize(run(args.seeds, args.noise))
    with pd.option_context("display.float_format", "{:.3g}".format,
                           "display.width", 200, "display.max_columns", 20):
        print(table)
        print("\nheld-out torque RMSE per joint, % of its RMS torque (mean over seeds):")
        print(joints)
        print("\nbase-parameter z-scores (estimate - truth) / SE:")
        print(z)
    print("\n" + LABEL)


if __name__ == "__main__":
    main()
