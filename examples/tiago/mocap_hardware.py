"""TIAGo mocap across hardware (#100): the study's library and CLI.

One fixed model for every system and session (``config/tiago_mocap_hardware_config.yaml``):
``joint_offset`` level, core's structural selection, the tool point of
``wrist_ft_tool_link`` as the only observation (xyz), base frame (6D) and tool
point (3D) free. Torso and arm_1 are absorbed by the base and arm_7 by the tool
point, so ``arm_2 .. arm_6`` are the comparable parameters. Nothing is selected
on any session.

Evaluating a trained model on a session is a *re-registration*: the trained
joint offsets are kept and only the base frame and the tool point are refitted
on that session. The primary metric is leave-one-out (LOO) re-registered RMSE:
refit the frames on n-1 postures, score the held-out one. Sessions with fewer
than ``MIN_POSTURES`` postures are not evaluated.

Secondary run (2023 OptiTrack and Vicon, which carry the tool orientation):
position and orientation. The residual is core's ``TiagoCalibration`` log-map
residual with the orientation rows scaled by ``W_ROT`` = 0.1 m/rad (core's
default ``measurement_std``, 0.001 m over 0.01 rad, through
``BaseCalibration.apply_measurement_weighting``; TIAGo's own cost bypasses it,
so it is applied here). It is not optimally weighted until figaroh-plus#215, so
it never drives a verdict.

Stages: ``noise`` estimates each system's effective noise from paired postures
with *registration only* (no joint-level fit; the pre-registration, PR A of
#100). ``real`` fits the fixed model on the 2021 and 2023 sessions and reads the
differences against the frozen bands (C1, C2, C3, C5, the arm_5-only
diagnostic and the 6D secondary); ``square`` scores the models on the seven
square-motion runs (C4, lag sweep); ``all`` runs both. ``--archive`` writes the
run record. Design, thresholds and results:
docs/development/tiago-mocap-hardware.md.

    python mocap_hardware.py noise
    python mocap_hardware.py real --archive --no-plot
    python mocap_hardware.py all --archive --no-plot
"""

from __future__ import annotations

import argparse
import contextlib
import io
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import pandas as pd
import pinocchio as pin
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from figaroh.calibration.calibration_tools import calc_updated_fkm  # noqa: E402
from figaroh.tools.robot import load_robot  # noqa: E402

from examples.calibration_harness import heldout as harness  # noqa: E402
from examples.calibration_harness.protocol import read_heldout  # noqa: E402
from examples.tiago.utils.tiago_tools import TiagoCalibration  # noqa: E402

TIAGO = Path(__file__).resolve().parent
CALIB = TIAGO / "data/calibration"
CONFIG = TIAGO / "config/tiago_mocap_hardware_config.yaml"
URDF = TIAGO / "urdf/tiago_48_schunk.urdf"
JOINTS = ["torso_lift_joint"] + [f"arm_{i}_joint" for i in range(1, 8)]
OFFSET_NAMES = ["offsetPZ_torso_lift_joint"] + [
    f"offsetRZ_arm_{i}_joint" for i in range(1, 8)
]
COMPARABLE = [f"offsetRZ_arm_{i}_joint" for i in range(2, 7)]
FRAME_NAMES = ["base_px", "base_py", "base_pz", "base_phix", "base_phiy", "base_phiz"]
TOOL_NAMES = ["pEEx_1", "pEEy_1", "pEEz_1"]
POSITION = [True, True, True, False, False, False]
MIN_POSTURES = 6
W_ROT = 0.1  # m per rad: core's default measurement_std (0.001 m / 0.01 rad)
NOISE_GRID_MM = (0.5, 2.0, 4.0)
# clock-lag uncertainty (s) of the 2023 sessions (data/README.md): the mocap
# stamp of a static posture is read inside a plateau, and a plateau is at least
# LAG_MARGIN_S wider than this on each side
LAG_UNCERTAINTY_S = {"optitrack": 0.04, "vicon": 0.005}
LAG_MARGIN_S = 0.3

PROTOCOLS = {
    "qualisys": CALIB / "mocap/protocol.yaml",
    "optitrack": CALIB / "optitrack/protocol.yaml",
    "vicon": CALIB / "vicon/protocol.yaml",
}


@dataclass(frozen=True)
class Session:
    """One recorded session of a system, from its protocol."""

    system: str
    id: str
    role: str
    path: Path
    evaluated: bool  # False when the protocol lists it in not_evaluated

    @property
    def six_d(self) -> bool:
        return self.system in ("optitrack", "vicon")


def load_sessions(verify: bool = True) -> Dict[str, Session]:
    """``{system/id: Session}`` of the three protocols (sha256-verified)."""
    out = {}
    for system, path in PROTOCOLS.items():
        protocol, spec = read_heldout(path)
        if verify:
            protocol.verify(root=path.parent)
        for s in protocol.sessions:
            (rel,) = s.files
            out[f"{system}/{s.id}"] = Session(
                system, s.id, s.role, path.parent / rel, s.id not in spec.not_evaluated
            )
    return out


def training_session(system: str, sessions: Optional[dict] = None) -> Session:
    sessions = sessions or load_sessions()
    (train,) = [
        s for s in sessions.values() if s.system == system and s.role == "training"
    ]
    return train


@dataclass
class Data:
    """A session's postures and measurements (position, optionally rotation)."""

    q: np.ndarray  # (n, 8) JOINTS
    pos: np.ndarray  # (n, 3)
    rot: Optional[np.ndarray] = None  # (n, 3, 3) measured tool orientation

    def __len__(self) -> int:
        return len(self.q)


def rpy_to_matrices(rpy: np.ndarray) -> np.ndarray:
    """``pin.rpy.rpyToMatrix`` of every row (extrinsic x-y-z)."""
    return Rotation.from_euler("xyz", rpy).as_matrix()


def read_data(path, six_d: bool = False) -> Data:
    df = pd.read_csv(path)
    rot = None
    if six_d:
        rot = rpy_to_matrices(df[["phix1", "phiy1", "phiz1"]].to_numpy())
    return Data(df[JOINTS].to_numpy(), df[["x1", "y1", "z1"]].to_numpy(), rot)


def make_calib(
    data_file, frames_only: bool = False, six_d: bool = False, solve: bool = True
) -> TiagoCalibration:
    """The fixed model on one session file (structural fit unless ``solve=False``).

    ``six_d`` measures position and orientation (secondary run).
    """
    from figaroh.calibration.calibration_tools import drop_calibration_parameters

    robot = load_robot(str(URDF), load_by_urdf=True, robot_pkg="tiago_description")
    calib = TiagoCalibration(robot, str(CONFIG), del_list=[])
    cfg = calib.calib_config
    cfg["data_file"] = str(data_file)
    calib._data_path = str(Path(data_file).resolve())
    cfg["validation_data_file"] = None
    # the run record is filed under this asset (every session is tiago-48)
    cfg["instance"] = {**(cfg.get("instance") or {}), "asset_id": "tiago-48"}
    cfg["known_baseframe"] = False
    cfg["known_tipframe"] = False
    if six_d:
        cfg["measurability"] = [True] * 6
        cfg["calibration_index"] = 6
    if solve or frames_only:
        # start the frames at a registration of the nominal joints on this
        # session (Kabsch + LM): core's closed-form guess lands in a wrong
        # basin when the tool frame is far from the body's (Vicon triad)
        guess = registration_guess(calib, data_file, six_d)
        calib.initial_frame_guess = lambda: guess
    with contextlib.redirect_stdout(io.StringIO()):
        calib.initialize()
        if six_d:
            # TIAGo's cost is unweighted; apply core's default weighting
            raw = calib.cost_function
            calib.cost_function = lambda var: calib.apply_measurement_weighting(
                raw(var), 1.0, W_ROT
            )
        if frames_only:
            joints = [
                n for n in cfg["param_name"] if not (n.startswith("base_") or "EE" in n)
            ]
            drop_calibration_parameters(cfg, joints)
        if solve:
            calib.solve(plotting=False, enable_logging=False, html_report=False)
    return calib


def registration_guess(
    calib: TiagoCalibration, data_file, six_d: bool
) -> Dict[str, float]:
    """Base frame, tool point (and tool rotation) of a registration-only fit."""
    data = read_data(data_file, six_d=six_d)
    R, p = Chain(calib).tool_pose(data.q, np.zeros(8))
    x = reregister(R, p, data.pos, data.rot)
    guess = frames_to_params(x)
    if six_d:
        rpy = matrix_to_rpy(Rotation.from_rotvec(x[9:12]).as_matrix())
        guess.update(zip(("phiEEx_1", "phiEEy_1", "phiEEz_1"), rpy))
    return guess


def fit_session(
    path, six_d: bool = False
) -> Tuple[Dict[str, float], Dict[str, float], TiagoCalibration]:
    """Structural fit on one session: ``(params, std, calib)`` by name."""
    calib = make_calib(path, six_d=six_d)
    names = list(calib.calib_config["param_name"])
    std = calib.std_dev if calib.std_dev is not None else np.full(len(names), np.nan)
    return dict(zip(names, calib.LM_result.x)), dict(zip(names, std)), calib


def offsets_of(params: Dict[str, float]) -> np.ndarray:
    """(8,) joint offsets of a fit, in ``JOINTS`` order; absorbed ones are 0."""
    return np.array([params.get(n, 0.0) for n in OFFSET_NAMES])


class Chain:
    """Forward kinematics of the fixed model through core's ``calc_updated_fkm``.

    The tool pose in the world/base frame is read from core itself (the tool
    point for ``t = 0, e_x, e_y, e_z``), so the joint-offset sign convention
    and the placement of the chain are core's, not re-derived here.
    """

    def __init__(self, calib: TiagoCalibration):
        self.model, self.data = calib.model, calib.data
        self.cfg = dict(
            calib.calib_config,
            param_name=OFFSET_NAMES + TOOL_NAMES,
            measurability=list(POSITION),
            calibration_index=3,
        )
        self.neutral = pin.neutral(self.model)
        self.idx = [self.model.joints[self.model.getJointId(j)].idx_q for j in JOINTS]

    def configurations(self, q8: np.ndarray) -> np.ndarray:
        q = np.tile(self.neutral, (len(q8), 1))
        q[:, self.idx] = q8
        return q

    def tool_pose(self, q8: np.ndarray, offsets: np.ndarray):
        """``(R (n, 3, 3), p (n, 3))`` of the tool frame in the chain's start frame."""
        q = self.configurations(q8)
        cfg = dict(self.cfg, NbSample=len(q))
        pts = []
        for t in np.vstack([np.zeros(3), np.eye(3)]):
            x = np.r_[np.asarray(offsets, dtype=float), t]
            pts.append(
                calc_updated_fkm(self.model, self.data, x, q, cfg).reshape(3, -1).T
            )
        p = pts[0]
        R = np.stack([pts[1] - p, pts[2] - p, pts[3] - p], axis=2)
        return R, p

    def frame_pose(self, q8: np.ndarray, offsets: np.ndarray, name: str):
        """Nominal-chain pose of any frame after ``q + offsets`` (test aid)."""
        q = self.configurations(np.asarray(q8) + np.asarray(offsets))
        fid = self.model.getFrameId(name)
        data = self.model.createData()
        R, p = [], []
        for row in q:
            pin.framesForwardKinematics(self.model, data, row)
            R.append(data.oMf[fid].rotation.copy())
            p.append(data.oMf[fid].translation.copy())
        return np.array(R), np.array(p)


def matrix_to_rpy(R: np.ndarray) -> np.ndarray:
    return pin.rpy.matrixToRpy(R)


# --- re-registration -------------------------------------------------------


def _predict(R, p, x, six_d):
    Rb = Rotation.from_rotvec(x[:3]).as_matrix()
    pos = (p + np.einsum("nij,j->ni", R, x[6:9])) @ Rb.T + x[3:6]
    if not six_d:
        return pos, None
    Rt = Rotation.from_rotvec(x[9:12]).as_matrix()
    return pos, np.einsum("ij,njk,kl->nil", Rb, R, Rt)


def _rot_error(R_meas, R_pred) -> np.ndarray:
    return Rotation.from_matrix(np.einsum("nji,njk->nik", R_meas, R_pred)).as_rotvec()


def kabsch_init(p: np.ndarray, meas: np.ndarray, six_d: bool = False) -> np.ndarray:
    """Initial frames: the rigid transform taking the tool origins ``p`` to ``meas``.

    LM from identity does not converge in a world frame far from the robot
    (the OptiTrack world is y-up); this is the start for every re-registration.
    Returns ``[rotvec_base, t_base, tool point (3)]`` (+ tool rotation, 6D).
    """
    ca, cb = p.mean(axis=0), meas.mean(axis=0)
    u, _, vt = np.linalg.svd((p - ca).T @ (meas - cb))
    rigid = vt.T @ np.diag([1, 1, np.linalg.det(vt.T @ u.T)]) @ u.T
    x0 = np.r_[Rotation.from_matrix(rigid).as_rotvec(), cb - rigid @ ca, np.zeros(3)]
    return np.r_[x0, np.zeros(3)] if six_d else x0


def reregister(
    R: np.ndarray,
    p: np.ndarray,
    meas: np.ndarray,
    R_meas: Optional[np.ndarray] = None,
    x0: Optional[np.ndarray] = None,
) -> np.ndarray:
    """Base frame and tool point (and tool rotation in 6D) fitted on a session.

    ``(R, p)`` are the tool poses of the trained joint offsets; ``meas`` the
    measured tool point (and ``R_meas`` the measured orientation for 6D).
    Returns ``x = [rotvec_base, t_base, tool point (3)]`` (+ ``rotvec_tool``).
    """
    six_d = R_meas is not None
    if x0 is None:
        x0 = kabsch_init(p, meas, six_d)

    def residual(x):
        pos, rot = _predict(R, p, x, six_d)
        r = pos - meas
        if six_d:
            r = np.hstack([r, W_ROT * _rot_error(R_meas, rot)])
        return r.ravel()

    return least_squares(residual, x0, method="lm").x


def frames_to_params(x: np.ndarray) -> Dict[str, float]:
    """Core's names for the base 6D and tool point of a re-registration ``x``."""
    Rb = Rotation.from_rotvec(x[:3]).as_matrix()
    base = np.r_[x[3:6], matrix_to_rpy(Rb)]
    out = dict(zip(FRAME_NAMES, base))
    out.update(zip(TOOL_NAMES, x[6:9]))
    return out


def residuals(R, p, x, meas, R_meas=None):
    """(n, 3) position errors (m) and, in 6D, (n, 3) rotation errors (rad)."""
    pos, rot = _predict(R, p, x, R_meas is not None)
    e_rot = None if R_meas is None else _rot_error(R_meas, rot)
    return pos - meas, e_rot


def loo_errors(R, p, meas, R_meas=None):
    """Leave-one-out re-registered errors, one refit per posture.

    Returns ``(position (n, 3) m, rotation (n, 3) rad or None)``: the error of
    posture ``i`` under frames fitted on every other posture.
    """
    n = len(p)
    x_full = reregister(R, p, meas, R_meas)
    pos_err = np.empty((n, 3))
    rot_err = None if R_meas is None else np.empty((n, 3))
    for i in range(n):
        keep = np.arange(n) != i
        x = reregister(
            R[keep],
            p[keep],
            meas[keep],
            None if R_meas is None else R_meas[keep],
            x0=x_full,
        )
        e, er = residuals(
            R[i : i + 1],
            p[i : i + 1],
            x,
            meas[i : i + 1],
            None if R_meas is None else R_meas[i : i + 1],
        )
        pos_err[i] = e[0]
        if er is not None:
            rot_err[i] = er[0]
    return pos_err, rot_err


def rms_mm(err: np.ndarray) -> float:
    """RMS of the norm of (n, 3) errors in metres, reported in mm."""
    return float(1000.0 * np.sqrt(np.mean(np.sum(err**2, axis=1))))


def rms_6d_mm(pos_err: np.ndarray, rot_err: np.ndarray) -> float:
    """RMS of the 6D error in mm: position plus ``W_ROT * rotation`` (1 mrad = 0.1 mm)."""
    sq = np.sum(pos_err**2, axis=1) + np.sum((W_ROT * rot_err) ** 2, axis=1)
    return float(1000.0 * np.sqrt(np.mean(sq)))


def score(
    chain: Chain, data: Data, offsets: np.ndarray, truth: Optional[np.ndarray] = None
) -> dict:
    """LOO (primary) and in-sample re-registered RMSE of ``offsets`` on a session.

    ``truth`` (n, 3) are noise-free tool points of a simulated session: the
    LOO prediction is then also scored against them (the judge).
    """
    R, p = chain.tool_pose(data.q, offsets)
    pos_err, rot_err = loo_errors(R, p, data.pos, data.rot)
    x = reregister(R, p, data.pos, data.rot)
    ins, ins_rot = residuals(R, p, x, data.pos, data.rot)
    out = {"n": len(data), "loo_mm": rms_mm(pos_err), "insample_mm": rms_mm(ins)}
    if data.rot is not None:
        out["loo_6d_mm"] = rms_6d_mm(pos_err, rot_err)
        out["insample_6d_mm"] = rms_6d_mm(ins, ins_rot)
    if truth is not None:
        pred = data.pos + pos_err  # pos_err = prediction - measurement
        out["loo_vs_truth_mm"] = rms_mm(pred - truth)
    return out


# --- joint offsets and placements (square-motion runs) ----------------------


def apply_joint_offsets(q8: np.ndarray, offsets: np.ndarray) -> np.ndarray:
    """Joint offsets as ``q + offset`` on the nominal model (core's convention)."""
    return np.asarray(q8) + np.asarray(offsets)


def placement_model(
    model: pin.Model, applied: Dict[str, Dict[str, List[float]]]
) -> pin.Model:
    """A copy of ``model`` with the recorded parent-side offsets applied.

    ``applied`` is the output of ``square_motion_extraction.measure_applied_offsets``
    (``{joint: {translation_m, rotation_rad}}``). The arm ``/tf`` is
    ``M0 @ D @ Rz(q)``, so the joint placement becomes ``M0 @ D`` with
    ``D = SE3(exp(rotvec), t)``.
    """
    out = model.copy()
    for joint, d in applied.items():
        jid = out.getJointId(joint)
        delta = pin.SE3(
            pin.exp3(np.asarray(d["rotation_rad"], dtype=float)),
            np.asarray(d["translation_m"], dtype=float),
        )
        out.jointPlacements[jid] = model.jointPlacements[jid] * delta
    return out


def path_rmse(R: np.ndarray, p: np.ndarray, meas: np.ndarray) -> float:
    """Registered path RMSE (mm) of predicted tool poses against a marker path.

    ``(R, p)`` are the predicted tool poses of one model along a run and
    ``meas`` the marker path; a rigid transform and a fixed point in the tool
    frame are fitted on the run itself (in-sample), as in
    ``square_motion_extraction.registration_rmse`` (which takes joints and a
    URDF instead of poses). Generalised to poses so that joint offsets and
    recorded placements go through the same registration.
    """
    x = reregister(R, p, meas)
    err, _ = residuals(R, p, x, meas)
    return rms_mm(err)


# --- effective noise from paired postures (registration only) ---------------


def _pairs(q_a: np.ndarray, q_b: np.ndarray, tol: float) -> List[Tuple[int, int]]:
    """Postures of ``a`` and their nearest of ``b`` within ``tol`` on every joint."""
    out = []
    for i, x in enumerate(q_a):
        d = np.max(np.abs(q_b - x), axis=1)
        j = int(np.argmin(d))
        if d[j] < tol:
            out.append((i, j))
    return out


def sigma_eff(ssr: float, n: int, n_params: int) -> float:
    """Per-axis noise (mm) of one session from ``n`` paired postures.

    The difference of two sessions has variance ``2 sigma^2`` per axis;
    ``n_params`` registration parameters are removed from the 3n residuals:
    ``sigma^2 = SSR / (2 (3n - n_params))``.
    """
    dof = 3 * n - n_params
    if dof <= 0:
        raise ValueError(f"{n} pairs cannot fix {n_params} parameters")
    return float(1000.0 * np.sqrt(ssr / (2.0 * dof)))


def rigid_ssr(a: np.ndarray, b: np.ndarray) -> float:
    """Sum of squares of ``b - (R a + t)`` after the best rigid transform (6 parameters)."""
    ca, cb = a.mean(axis=0), b.mean(axis=0)
    u, _, vt = np.linalg.svd((a - ca).T @ (b - cb))
    r = vt.T @ np.diag([1, 1, np.linalg.det(vt.T @ u.T)]) @ u.T
    return float(np.sum(((a - ca) @ r.T - (b - cb)) ** 2))


def rigid_tool_ssr(a: np.ndarray, b: np.ndarray, rot_b: np.ndarray) -> float:
    """Rigid transform plus a tool-point change, in the gripper frame of ``b`` (9 parameters).

    ``b_i = R a_i + t + Rb_i d``: the triad of the gripper body changed between
    the sessions, so the marker sits at a different point of the gripper frame.
    """
    x0 = np.zeros(9)

    def residual(x):
        r = Rotation.from_rotvec(x[:3]).as_matrix()
        return (a @ r.T + x[3:6] + np.einsum("nij,j->ni", rot_b, x[6:9]) - b).ravel()

    # start from the 6-parameter rigid fit
    ca, cb = a.mean(axis=0), b.mean(axis=0)
    u, _, vt = np.linalg.svd((a - ca).T @ (b - cb))
    r = vt.T @ np.diag([1, 1, np.linalg.det(vt.T @ u.T)]) @ u.T
    x0[:3] = Rotation.from_matrix(r).as_rotvec()
    x0[3:6] = cb - r @ ca
    return float(np.sum(least_squares(residual, x0, method="lm").fun ** 2))


def noise_estimates(sessions: Optional[dict] = None) -> Dict[str, dict]:
    """Effective per-axis noise of each system from paired postures.

    Registration only; no joint-level model is fitted. Pairs are postures
    repeated within the protocol's ``repeated_tol`` on every joint.

    - OptiTrack: 1529 against 1552 (same rigid-body definition), rigid
      registration (6 parameters);
    - Vicon: 1508 against 1607, rigid registration plus the tool-point change
      of the gripper triad (9 parameters);
    - Qualisys: the November sessions' repeated postures against the training
      session, base-frame coordinates (nothing registered).
    """
    sessions = sessions or load_sessions()
    tol = read_heldout(PROTOCOLS["qualisys"])[1].strata.repeated_tol
    out = {}

    a = read_data(sessions["optitrack/2023-09-12-1529"].path)
    b = read_data(sessions["optitrack/2023-09-12-1552"].path)
    pairs = _pairs(a.q, b.q, tol)
    ia, ib = (list(x) for x in zip(*pairs))
    out["optitrack"] = {
        "pairs": len(pairs),
        "params": 6,
        "sigma_eff_mm": sigma_eff(rigid_ssr(a.pos[ia], b.pos[ib]), len(pairs), 6),
    }

    a = read_data(sessions["vicon/2023-09-20-1508"].path, six_d=True)
    b = read_data(sessions["vicon/2023-09-20-1607"].path, six_d=True)
    pairs = _pairs(a.q, b.q, tol)
    ia, ib = (list(x) for x in zip(*pairs))
    out["vicon"] = {
        "pairs": len(pairs),
        "params": 9,
        "sigma_eff_mm": sigma_eff(
            rigid_tool_ssr(a.pos[ia], b.pos[ib], b.rot[ib]), len(pairs), 9
        ),
    }

    train = read_data(training_session("qualisys", sessions).path)
    ssr, n = 0.0, 0
    for s in sessions.values():
        if (
            s.system != "qualisys"
            or s.role == "training"
            or not s.id.startswith("2021-11")
        ):
            continue
        d = read_data(s.path)
        pairs = _pairs(d.q, train.q, tol)
        for i, j in pairs:
            ssr += float(np.sum((d.pos[i] - train.pos[j]) ** 2))
        n += len(pairs)
    out["qualisys"] = {"pairs": n, "params": 0, "sigma_eff_mm": sigma_eff(ssr, n, 0)}

    for k, v in out.items():
        v["noise_level_mm"] = noise_level(v["sigma_eff_mm"])
    return out


def noise_level(sigma_mm: float, grid=NOISE_GRID_MM) -> float:
    """Smallest grid level that is at least ``sigma_mm`` (the largest if none)."""
    above = [g for g in grid if g >= sigma_mm]
    return float(above[0] if above else grid[-1])


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    stages = ["noise", "real", "square", "all"]
    parser.add_argument("stage", nargs="?", choices=stages, default=None)
    parser.add_argument("--stage", dest="stage_opt", choices=stages)
    parser.add_argument(
        "--archive", action="store_true", help="write the run record (real, all)"
    )
    parser.add_argument("--root", default="results/runs", help="archive root")
    parser.add_argument("--out", type=Path, help="write the results as JSON")
    parser.add_argument("--golden", type=Path, help="write the golden file")
    parser.add_argument(
        "--no-plot", action="store_true", help="accepted for validate.py"
    )
    args = parser.parse_args(argv)
    stage = args.stage_opt or args.stage or "noise"
    if stage != "noise":
        from examples.tiago import mocap_hardware_study as study

        sys.exit(study.run(stage, args))
    est = noise_estimates()
    print("effective noise from paired postures (registration only)")
    for system, v in est.items():
        print(
            f"  {system:10s} pairs {v['pairs']:3d}  params {v['params']}  "
            f"sigma_eff {v['sigma_eff_mm']:.3f} mm  -> grid level {v['noise_level_mm']} mm"
        )


if __name__ == "__main__":
    main()
