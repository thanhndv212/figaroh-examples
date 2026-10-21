"""TIAGo head camera, chessboard on the hand (#101): the study's library and CLI.

Question: can the head camera replace motion capture for calibrating the arm?
Both sensors see a chessboard held by the hand. The measurement is the board
pose in the sensor frame, ``M = (T_cam(q + d) X)^-1 T_hand(q + d) Y``, with
``T_cam`` the pose of ``xtion_optical_frame`` (child of ``head_2_link``) and
``T_hand`` that of ``arm_7_link``, both from the nominal URDF at the joint
angles plus offsets ``d``. ``X`` is the camera in ``head_2_link`` (a
correction of the nominal optical frame) and ``Y`` the board on ``arm_7_link``.

Gauge (docs/development/tiago-head-camera.md section 3). Only the relative pose
of camera and hand enters, so the offsets and the two frames are not all
identifiable: the torso offset has no effect, ``head_2`` is absorbed by ``X``,
``arm_7`` by ``Y``, and ``head_1`` and ``arm_1`` (both vertical, 30 mm apart)
act alike except for a 30 mm lever. The fitted set is therefore
``G = {d_arm_1 .. d_arm_6, X, Y}`` (18 parameters), the head held at its
nominal angles: ``d_arm_1`` is "arm_1 relative to head_1".

Residual (primary, 6D): ``[p_meas - p_pred ; log(R_pred R_meas^T)]`` in the
camera frame, whitened by a fixed diagonal sigma (xy, xy, z, tilt, tilt, roll).
``log(R_pred R_meas^T)`` is ``log(R_meas^T R_pred)`` carried to the camera
frame by ``R_meas``. Rotation matrices only: the orientation columns are
roll-pitch-yaw near +-pi and are never differenced. Secondary: position only.
Millimetres and milliradians are always reported separately.

Stages: ``noise`` estimates the camera's effective noise from the 2023-11-27
``center`` and ``topleft`` sessions with *registration only* (the board frame
``Y`` refitted per session at nominal joints, no joint fit) and the mocap noise
from the plateau statistics of the 2023-11-07 runs (the pre-registration, PR A
of #101). The study stages are PR B.

    python head_camera.py noise
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import pandas as pd
import pinocchio as pin
import yaml
from figaroh.data.protocol import Protocol
from scipy.optimize import linear_sum_assignment, least_squares
from scipy.spatial.transform import Rotation

project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from examples.calibration_harness import heldout as harness  # noqa: E402

TIAGO = Path(__file__).resolve().parent
HC = TIAGO / "data/calibration/head_camera"
PROTOCOL = HC / "protocol.yaml"
CONFIG = TIAGO / "config/tiago_head_camera_config.yaml"
URDF = TIAGO / "urdf/tiago_48_hey5.urdf"
CAMERA_FRAME = "xtion_optical_frame"
HAND_FRAME = "arm_7_link"

JOINTS = ["torso_lift_joint", "head_1_joint", "head_2_joint"] + [
    f"arm_{i}_joint" for i in range(1, 8)
]
ARM = list(range(3, 10))  # indices of arm_1 .. arm_7 in JOINTS
FIT_OFFSETS = tuple(range(3, 9))  # arm_1 .. arm_6: the gauge-fixed set G
ALL_OFFSETS = tuple(range(10))  # the naive set: torso, head_1, head_2, arm_1 .. arm_7
AXES = ("x", "y", "z", "rx", "ry", "rz")


def load_settings(path=CONFIG) -> dict:
    """The ``custom.head_camera`` block of the study configuration."""
    return yaml.safe_load(Path(path).read_text())["custom"]["head_camera"]


SETTINGS = load_settings()
GRID_MM = tuple(SETTINGS["noise_grid_mm"])
MIN_POSTURES = int(SETTINGS["min_postures"])


# --- rigid transforms (4x4, batched) ---------------------------------------


def hom(v: np.ndarray) -> np.ndarray:
    """4x4 transform of ``v = [translation, rotation vector]``."""
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(v[3:]).as_matrix()
    T[:3, 3] = v[:3]
    return T


def vec(T: np.ndarray) -> np.ndarray:
    """``[translation, rotation vector]`` of a 4x4 transform."""
    return np.r_[T[:3, 3], Rotation.from_matrix(T[:3, :3]).as_rotvec()]


def inv(T: np.ndarray) -> np.ndarray:
    """Inverse of rigid transforms, ``(..., 4, 4)``."""
    out = np.zeros_like(T)
    Rt = np.swapaxes(T[..., :3, :3], -1, -2)
    out[..., :3, :3] = Rt
    out[..., :3, 3] = -np.einsum("...ij,...j->...i", Rt, T[..., :3, 3])
    out[..., 3, 3] = 1.0
    return out


def from_rpy(rpy: np.ndarray) -> np.ndarray:
    """Rotation matrices of pinocchio roll-pitch-yaw rows (``R = Rz Ry Rx``)."""
    return Rotation.from_euler("xyz", np.atleast_2d(rpy)).as_matrix()


# --- sessions and roles ----------------------------------------------------


@dataclass(frozen=True)
class Session:
    id: str
    role: str
    robot: str
    camera: Optional[Path]  # chessboard_in_camera table
    joints: Optional[Path]  # PAL joints table
    mocap: Optional[Path]

    @property
    def evaluated(self) -> bool:
        return self.robot == SETTINGS["robot"]


def load_sessions(verify: bool = True) -> Dict[str, Session]:
    """``{id: Session}`` of the protocol (sha256-verified)."""
    protocol = Protocol.load(PROTOCOL)
    if verify:
        protocol.verify(root=PROTOCOL.parent)
    out = {}
    for s in protocol.sessions:
        files = {
            kind: HC / name
            for name in s.files
            for kind in ("camera", "joints", "mocap")
            if (
                (
                    kind == "camera"
                    and name.startswith("camera_")
                    and "_chessboard" in name
                )
                or (kind == "joints" and name.endswith("_joints.csv"))
                or (kind == "mocap" and name.startswith("mocap_"))
            )
        }
        out[s.id] = Session(
            s.id,
            s.role,
            getattr(s, "robot", None) or _robot_of(s.id),
            files.get("camera"),
            files.get("joints"),
            files.get("mocap"),
        )
    return out


def _robot_of(session_id: str) -> str:
    raw = yaml.safe_load(PROTOCOL.read_text())["sessions"]
    return {s["id"]: s.get("robot") for s in raw}[session_id]


def role_sets(sessions: Optional[dict] = None) -> Dict[str, List[Session]]:
    """Sessions by role. ``tiago-155c`` (another robot) is not evaluated.

    Built here instead of ``calibration_harness.parse_heldout``, which has no
    ``diagnostic`` role and treats each file as its own set. Roles are the
    protocol's (frozen in v2); the guard against selecting on confirmation is
    kept by :func:`select_model`.
    """
    sessions = sessions or load_sessions()
    out: Dict[str, List[Session]] = {
        "training": [],
        "validation": [],
        "confirmation": [],
        "diagnostic": [],
    }
    for s in sessions.values():
        if s.evaluated:
            out[s.role].append(s)
    if len(out["training"]) != 1:
        raise ValueError(f"expected one training session, got {out['training']}")
    return out


def select_model(results, metric: str, role: str = "validation"):
    """Model selection through the harness: refuses ``confirmation``."""
    return harness.select(results, metric, role)


def read_plan(session: Session) -> np.ndarray:
    """(n, 10) posture plan ``JOINTS`` of a camera session (joints only)."""
    df = pd.read_csv(session.joints)
    return df[JOINTS].to_numpy()


def paired_rows(session: Session) -> np.ndarray:
    """Rows of the camera plan whose posture is also in the session's mocap run.

    Reads the ``posture`` columns only (the pairing index of the two files): no
    pose value, so it is allowed for confirmation sessions in the
    pre-registration.
    """
    plan = pd.read_csv(session.joints, usecols=["posture"])["posture"].to_numpy()
    kept = set(pd.read_csv(session.mocap, usecols=["posture"])["posture"].tolist())
    return np.array([i for i, p in enumerate(plan) if p in kept], dtype=int)


@dataclass
class Obs:
    """Board poses in the sensor frame at ``Q`` (n, 10): ``R`` (n,3,3), ``p`` (n,3)."""

    Q: np.ndarray
    R: np.ndarray
    p: np.ndarray

    def __len__(self) -> int:
        return len(self.Q)

    def take(self, idx) -> "Obs":
        return Obs(self.Q[idx], self.R[idx], self.p[idx])

    @property
    def T(self) -> np.ndarray:
        out = np.tile(np.eye(4), (len(self), 1, 1))
        out[:, :3, :3] = self.R
        out[:, :3, 3] = self.p
        return out


def read_camera(session: Session, allow_confirmation: bool = False) -> Obs:
    """PAL camera observations of a session.

    The pose values of a ``confirmation`` session are not readable unless
    ``allow_confirmation`` is set (the confirmation step of PR B). The joints
    (the posture plan) are not a measurement and are read by :func:`read_plan`.
    """
    if session.role == "confirmation" and not allow_confirmation:
        raise PermissionError(
            f"{session.id}: confirmation values are read only at the confirmation step"
        )
    pose = pd.read_csv(session.camera)
    return Obs(
        read_plan(session),
        from_rpy(pose[["phix", "phiy", "phiz"]].to_numpy()),
        pose[["x", "y", "z"]].to_numpy(),
    )


# --- kinematics ------------------------------------------------------------


class Kinematics:
    """Nominal URDF forward kinematics of the camera and the hand at ``q + d``."""

    def __init__(self, urdf=URDF):
        self.model = pin.buildModelFromUrdf(str(urdf))
        self.data = self.model.createData()
        self.neutral = pin.neutral(self.model)
        self.idx = [self.model.joints[self.model.getJointId(j)].idx_q for j in JOINTS]
        self._frames = {}
        for key, name in (("cam", CAMERA_FRAME), ("hand", HAND_FRAME)):
            f = self.model.frames[self.model.getFrameId(name)]
            self._frames[key] = (f.parentJoint, f.placement.homogeneous)

    def poses(self, Q: np.ndarray, off: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """World poses ``(T_cam, T_hand)`` (n, 4, 4) at ``Q + off``."""
        n = len(Q)
        Tc, Ta = np.empty((n, 4, 4)), np.empty((n, 4, 4))
        q = self.neutral.copy()
        (jc, pc), (ja, pa) = self._frames["cam"], self._frames["hand"]
        for i in range(n):
            q[self.idx] = Q[i] + off
            pin.forwardKinematics(self.model, self.data, q)
            Tc[i] = self.data.oMi[jc].homogeneous @ pc
            Ta[i] = self.data.oMi[ja].homogeneous @ pa
        return Tc, Ta

    def axis_tool(self, joint: str) -> np.ndarray:
        """Unit axis (joint frame) of a revolute joint; every one here is +z."""
        return np.asarray(self.model.joints[self.model.getJointId(joint)].axis)

    def frame_in_joint(self, name: str) -> Tuple[str, np.ndarray]:
        f = self.model.frames[self.model.getFrameId(name)]
        return self.model.names[f.parentJoint], f.placement.homogeneous


def predict(Tc, Ta, X, Y, k: float = 1.0) -> np.ndarray:
    """Board pose in the sensor frame, (n, 4, 4); the depth is scaled by ``k``."""
    M = inv(Tc @ X) @ Ta @ Y
    if k != 1.0:
        M[:, 2, 3] *= k
    return M


# --- residuals and whitening ----------------------------------------------


def sigma_vector(level_mm: float, ratios: Optional[dict] = None) -> np.ndarray:
    """Fixed sigma (m, m, m, rad, rad, rad) of the camera at a grid level.

    ``level_mm`` is the depth sigma; the other axes follow the frozen ratios of
    the noise stage (``noise_ratios`` of the configuration).
    """
    r = ratios or SETTINGS["noise_ratios"]
    z = level_mm * 1e-3
    return np.array(
        [
            z * r["xy"],
            z * r["xy"],
            z,
            z * r["tilt_mrad_per_mm"],
            z * r["tilt_mrad_per_mm"],
            z * r["roll_mrad_per_mm"],
        ]
    )


def mocap_sigma_vector(level_mm: float) -> np.ndarray:
    """Isotropic mocap sigma: ``level_mm`` per axis, rotation through the lever."""
    s = level_mm * 1e-3
    return np.array([s, s, s] + [s / SETTINGS["mocap_rot_lever_m"]] * 3)


def raw_residuals(M_pred: np.ndarray, obs: Obs) -> np.ndarray:
    """(n, 6) ``[p_meas - p_pred, log(R_pred R_meas^T)]`` in the sensor frame."""
    dp = obs.p - M_pred[:, :3, 3]
    dR = M_pred[:, :3, :3] @ np.swapaxes(obs.R, 1, 2)
    return np.hstack([dp, Rotation.from_matrix(dR).as_rotvec()])


def register_board(kin: Kinematics, obs: Obs, X, off) -> np.ndarray:
    """Board frame ``Y`` on the hand that best explains ``obs`` (closed-form mean).

    ``Y_i = T_hand^-1 T_cam X M_meas`` per posture; the rotation mean is the
    chordal one. Starting point of every fit; the nominal joints (or ``off``)
    carry their own error, which a refit then spreads over the other parameters.
    """
    Tc, Ta = kin.poses(obs.Q, off)
    Yi = inv(Ta) @ (Tc @ X) @ obs.T
    u, _, vt = np.linalg.svd(Yi[:, :3, :3].sum(axis=0))
    R = u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt
    Y = np.eye(4)
    Y[:3, :3], Y[:3, 3] = R, Yi[:, :3, 3].mean(axis=0)
    return Y


# --- parametrisation and fit -----------------------------------------------


@dataclass(frozen=True)
class Spec:
    """Which parameters are free.

    ``offsets``: indices into ``JOINTS`` whose offset is fitted; ``rows``:
    ``"6d"`` (pose) or ``"3d"`` (position, ``Y`` reduced to a point);
    ``scale``: a depth scale ``k`` (the parameter is ``k - 1``).
    """

    offsets: Tuple[int, ...] = FIT_OFFSETS
    rows: str = "6d"
    scale: bool = False

    def names(self) -> List[str]:
        n = [f"d_{JOINTS[i][:-6]}" for i in self.offsets]
        n += [f"X_{a}" for a in AXES]
        n += [f"Y_{a}" for a in AXES[: 3 if self.rows == "3d" else 6]]
        return n + (["k"] if self.scale else [])

    @property
    def n_params(self) -> int:
        return len(self.names())


def unpack(theta: np.ndarray, spec: Spec, off_fixed: Optional[np.ndarray] = None):
    """``(off10, x6, y6, k)`` of a parameter vector."""
    off = np.zeros(10) if off_fixed is None else np.array(off_fixed, dtype=float)
    m = len(spec.offsets)
    off[list(spec.offsets)] = theta[:m]
    x = theta[m : m + 6]
    ny = 3 if spec.rows == "3d" else 6
    y = np.zeros(6)
    y[:ny] = theta[m + 6 : m + 6 + ny]
    k = 1.0 + (theta[-1] if spec.scale else 0.0)
    return off, x, y, k


def residual_vector(
    kin: Kinematics,
    obs: Obs,
    theta: np.ndarray,
    spec: Spec,
    sigma: np.ndarray,
    X0: np.ndarray,
    Y0: np.ndarray,
    off_fixed=None,
    poses=None,
) -> np.ndarray:
    off, x, y, k = unpack(theta, spec, off_fixed)
    Tc, Ta = poses if poses is not None else kin.poses(obs.Q, off)
    M = predict(Tc, Ta, X0 @ hom(x), Y0 @ hom(y), k)
    r = raw_residuals(M, obs) / sigma
    return (r[:, :3] if spec.rows == "3d" else r).ravel()


@dataclass
class Fit:
    spec: Spec
    theta: np.ndarray
    X0: np.ndarray
    Y0: np.ndarray
    off_fixed: np.ndarray
    sigma: np.ndarray
    cost: float
    jac: np.ndarray
    n_rows: int
    status: int
    names: List[str] = field(default_factory=list)

    @property
    def offsets(self) -> np.ndarray:
        """(10,) joint offsets of the fit (the others as fixed)."""
        return unpack(self.theta, self.spec, self.off_fixed)[0]

    @property
    def X(self) -> np.ndarray:
        return self.X0 @ hom(unpack(self.theta, self.spec)[1])

    @property
    def Y(self) -> np.ndarray:
        return self.Y0 @ hom(unpack(self.theta, self.spec)[2])

    @property
    def k(self) -> float:
        return unpack(self.theta, self.spec)[3]

    @property
    def singular_values(self) -> np.ndarray:
        return np.linalg.svd(self.jac, compute_uv=False)

    @property
    def cond(self) -> float:
        s = self.singular_values
        return float(s[0] / s[-1]) if s[-1] > 0 else float("inf")

    def rank(self, rtol: float = 1e-8) -> int:
        s = self.singular_values
        return int(np.sum(s > rtol * s[0]))

    def se(self, rcond: float = 1e-12) -> np.ndarray:
        return standard_errors(self.jac, rcond)

    def params(self) -> Dict[str, float]:
        return dict(zip(self.names, self.theta))


def standard_errors(jac: np.ndarray, rcond: float = 1e-12) -> np.ndarray:
    """Standard errors from a whitened Jacobian; null directions get a huge value.

    ``cov = V diag(1 / max(s, rcond s_max)^2) V^T``, so an exactly unobservable
    combination has a standard error of about ``1 / (rcond s_max)``.
    """
    _, s, vt = np.linalg.svd(jac, full_matrices=False)
    s = np.maximum(s, rcond * s[0])
    return np.sqrt(np.einsum("ij,j,ij->i", vt.T, 1.0 / s**2, vt.T))


def fit(
    kin: Kinematics,
    obs: Obs,
    spec: Spec,
    sigma: np.ndarray,
    X0: Optional[np.ndarray] = None,
    Y0: Optional[np.ndarray] = None,
    theta0: Optional[np.ndarray] = None,
    off_fixed: Optional[np.ndarray] = None,
) -> Fit:
    """Weighted least squares of ``spec`` on one session (``X = X0 hom(x)``, ``Y = Y0 hom(y)``)."""
    X0 = np.eye(4) if X0 is None else X0
    off0 = np.zeros(10) if off_fixed is None else np.asarray(off_fixed, dtype=float)
    Y0 = register_board(kin, obs, X0, off0) if Y0 is None else Y0
    theta0 = np.zeros(spec.n_params) if theta0 is None else np.asarray(theta0, float)
    poses = kin.poses(obs.Q, off0) if not spec.offsets else None

    def fun(theta):
        return residual_vector(
            kin, obs, theta, spec, sigma, X0, Y0, off_fixed, poses=poses
        )

    res = least_squares(fun, theta0, method="lm", xtol=1e-12, ftol=1e-12, gtol=1e-12)
    # central differences at the solution: MINPACK's forward-difference Jacobian
    # is too coarse to tell an exactly unobservable direction from a weak one
    jac = np.column_stack(
        [(fun(res.x + e) - fun(res.x - e)) / 2e-6 for e in 1e-6 * np.eye(len(res.x))]
    )
    return Fit(
        spec,
        res.x,
        X0,
        Y0,
        off0,
        sigma,
        float(np.sum(res.fun**2)),
        jac,
        len(res.fun),
        int(res.status),
        spec.names(),
    )


def jacobian(
    kin: Kinematics,
    obs: Obs,
    spec: Spec,
    theta: np.ndarray,
    sigma: np.ndarray,
    X0: np.ndarray,
    Y0: np.ndarray,
    h: float = 1e-6,
) -> np.ndarray:
    """Central-difference Jacobian of the whitened residual at ``theta``."""

    def fun(t):
        return residual_vector(kin, obs, t, spec, sigma, X0, Y0)

    cols = []
    for j in range(len(theta)):
        e = np.zeros_like(theta)
        e[j] = h
        cols.append((fun(theta + e) - fun(theta - e)) / (2 * h))
    return np.column_stack(cols)


def reregister(
    kin: Kinematics,
    obs: Obs,
    off: np.ndarray,
    sigma: np.ndarray,
    X0: np.ndarray,
    rows: str = "6d",
    Y0: Optional[np.ndarray] = None,
    scale: bool = False,
) -> Fit:
    """Keep the joint offsets ``off``; refit ``X`` (from ``X0``), ``Y`` (and ``k``) on ``obs``."""
    return fit(kin, obs, Spec((), rows, scale), sigma, X0=X0, Y0=Y0, off_fixed=off)


def predict_fit(kin: Kinematics, f: Fit, Q: np.ndarray) -> np.ndarray:
    """Board pose (n, 4, 4) predicted by a fit at postures ``Q``."""
    Tc, Ta = kin.poses(Q, f.offsets)
    return predict(Tc, Ta, f.X, f.Y, f.k)


def loo_errors(
    kin: Kinematics,
    obs: Obs,
    off: np.ndarray,
    sigma: np.ndarray,
    X0: np.ndarray,
    rows: str = "6d",
    scale: bool = False,
) -> np.ndarray:
    """Leave-one-out re-registered raw residuals (n, 6): ``p_meas - p_pred`` (m), rotation (rad).

    The frames are refitted on the other postures with the offsets kept, and
    the held-out posture is scored (camera frame, as the fit residual).
    """
    n = len(obs)
    full = reregister(kin, obs, off, sigma, X0, rows, scale=scale)
    out = np.empty((n, 6))
    for i in range(n):
        keep = np.arange(n) != i
        f = fit(
            kin,
            obs.take(keep),
            Spec((), rows, scale),
            sigma,
            X0=X0,
            Y0=full.Y0,
            theta0=full.theta,
            off_fixed=off,
        )
        out[i] = raw_residuals(predict_fit(kin, f, obs.Q[i : i + 1]), obs.take([i]))[0]
    return out


def rmse(err: np.ndarray) -> Tuple[float, float]:
    """``(position mm, rotation mrad)`` RMS of the norms of (n, 6) raw residuals."""
    pos = np.sqrt(np.mean(np.sum(err[:, :3] ** 2, axis=1)))
    rot = np.sqrt(np.mean(np.sum(err[:, 3:] ** 2, axis=1)))
    return float(pos * 1e3), float(rot * 1e3)


def axb_agreement(
    M_cam: np.ndarray, M_mc: np.ndarray, sigma: np.ndarray, X0=None, Y0=None
) -> dict:
    """C0, no kinematics: ``M_cam = X' M_mc Y'`` (12 parameters) on paired postures.

    ``M_cam``, ``M_mc`` are (n, 4, 4) board poses of the two sensors at the same
    postures; ``X' = X_cam^-1 X_mc`` and ``Y' = Y_mc^-1 Y_cam``. Returns the fitted
    frames and the per-axis residual RMS (m, rad): the sensor-agreement floor.
    Started from ``X0``, ``Y0`` (default: identity) with a deterministic
    multi-start on the rotations, the best cost kept.
    """

    def build(t):
        return hom(t[:6]), hom(t[6:])

    def fun(t):
        X, Y = build(t)
        pred = X @ M_mc @ Y
        dp = M_cam[:, :3, 3] - pred[:, :3, 3]
        dR = pred[:, :3, :3] @ np.swapaxes(M_cam[:, :3, :3], 1, 2)
        r = np.hstack([dp, Rotation.from_matrix(dR).as_rotvec()]) / sigma
        return r.ravel()

    starts = []
    if X0 is not None or Y0 is not None:
        starts.append(
            np.r_[
                vec(np.eye(4) if X0 is None else X0),
                vec(np.eye(4) if Y0 is None else Y0),
            ]
        )
    rng = np.random.default_rng(0)
    for _ in range(SETTINGS["axb_starts"]):
        starts.append(
            np.r_[np.zeros(3), rng.normal(size=3), np.zeros(3), rng.normal(size=3)]
        )
    best = None
    for t0 in starts:
        res = least_squares(fun, t0, method="lm", xtol=1e-12, ftol=1e-12)
        if best is None or res.cost < best.cost:
            best = res
    X, Y = build(best.x)
    raw = best.fun.reshape(-1, 6) * sigma
    return {
        "X": X,
        "Y": Y,
        "cost": float(2 * best.cost),
        "rms_per_axis": np.sqrt(np.mean(raw**2, axis=0)),
    }


# --- PAL reference ---------------------------------------------------------


def pal_reference() -> dict:
    """PAL's eye-hand calibration values as published (descriptive in PR B).

    ``offsets``: joint offsets (rad) by joint name; ``camera``: ``xyz`` (m) and
    ``rpy`` (rad) as published. Sign convention and frame of the camera pose are
    not documented: PR B fixes the sign on the training session.
    """
    raw = yaml.safe_load((HC / "pal_reference/master_calibration_pal.yaml").read_text())
    cal = raw["pal_eye_hand_calibration_tiago"]
    offsets = {
        k[: -len("_offset")]: float(v)
        for k, v in cal.items()
        if k.endswith("_joint_offset")
    }
    return {
        "offsets": offsets,
        "camera": {
            "xyz": np.array([cal[f"camera_position_{a}"] for a in "xyz"], dtype=float),
            "rpy": np.array(
                [cal[f"camera_orientation_{a}"] for a in "rpy"], dtype=float
            ),
        },
    }


def pal_gauge_offsets(sign: float = 1.0) -> Dict[str, float]:
    """PAL offsets mapped to the gauge-fixed set: arm_1 .. arm_6 and ``arm_1 - head_1``.

    ``sign`` (+1/-1) is the convention between PAL's offset and ``q + d``.
    """
    o = pal_reference()["offsets"]
    out = {f"arm_{i}": sign * o[f"arm_{i}_joint"] for i in range(1, 7)}
    out["arm_1_minus_head_1"] = sign * (o["arm_1_joint"] - o["head_1_joint"])
    return out


# --- effective noise (registration only) -----------------------------------


def pair_postures(Qa: np.ndarray, Qb: np.ndarray, tol: float) -> List[Tuple[int, int]]:
    """One-to-one pairing of postures within ``tol`` on every moving joint.

    Hungarian assignment on the largest joint difference (the torso is constant).
    """
    cols = list(range(1, 10))
    d = np.max(np.abs(Qa[:, None, cols] - Qb[None, :, cols]), axis=2)
    r, c = linear_sum_assignment(d)
    return [(int(i), int(j)) for i, j in zip(r, c) if d[i, j] < tol]


def registration_residuals(kin: Kinematics, obs: Obs) -> np.ndarray:
    """(n, 6) raw residuals after registering ``Y`` at nominal joints and ``X = I``.

    Weights for the registration only: position 1, rotation ``REG_ROT`` m/rad.
    """
    w = SETTINGS["registration_rot_m_per_rad"]
    sigma = np.array([1, 1, 1, 1 / w, 1 / w, 1 / w])
    f = fit(kin, obs, Spec((), "6d"), sigma)
    return raw_residuals(predict_fit(kin, f, obs.Q), obs)


def sigma_eff(d: np.ndarray) -> np.ndarray:
    """Per-axis sigma of one session from paired residual differences (n, 6).

    The difference of two independent sessions has variance ``2 sigma^2``; each
    session's registration removed 6 of its ``6 n`` residuals, so
    ``sum d^2 = 2 sigma^2 (6 n - 6)`` and per axis ``sigma^2 = sum d^2 / (2 (n - 1))``.
    """
    n = len(d)
    if n < MIN_POSTURES:
        raise ValueError(f"{n} pairs < {MIN_POSTURES}")
    return np.sqrt(np.sum(d**2, axis=0) / (2.0 * (n - 1)))


def noise_level(sigma_mm: float, grid=GRID_MM) -> float:
    """Smallest grid level at or above ``sigma_mm`` (the largest if none)."""
    above = [g for g in grid if g >= sigma_mm]
    return float(above[0] if above else grid[-1])


def noise_estimates(sessions: Optional[dict] = None) -> dict:
    """Camera noise per axis (11-27 center vs topleft) and mocap noise (11-07 plateaus).

    Registration only: no joint offset is fitted. Only ``training`` and
    ``diagnostic`` sessions are read.
    """
    sessions = sessions or load_sessions()
    kin = Kinematics()
    a = sessions[SETTINGS["noise_pair"][0]]
    b = sessions[SETTINGS["noise_pair"][1]]
    for s in (a, b):
        if s.role not in ("training", "diagnostic"):
            raise PermissionError(f"{s.id}: {s.role} sessions are not read for noise")
    oa, ob = read_camera(a), read_camera(b)
    pairs = pair_postures(oa.Q, ob.Q, SETTINGS["pair_tol_rad"])
    ra, rb = registration_residuals(kin, oa), registration_residuals(kin, ob)
    ia, ib = (list(x) for x in zip(*pairs))
    s = sigma_eff(ra[ia] - rb[ib])
    cam = {
        "pairs": len(pairs),
        "sigma_xy_mm": float(np.sqrt(np.mean(s[:2] ** 2)) * 1e3),
        "sigma_z_mm": float(s[2] * 1e3),
        "sigma_tilt_mrad": float(np.sqrt(np.mean(s[3:5] ** 2)) * 1e3),
        "sigma_roll_mrad": float(s[5] * 1e3),
        "per_axis": [float(v) for v in s],
    }
    cam["ratios"] = {
        "xy": cam["sigma_xy_mm"] / cam["sigma_z_mm"],
        "tilt_mrad_per_mm": cam["sigma_tilt_mrad"] / cam["sigma_z_mm"],
        "roll_mrad_per_mm": cam["sigma_roll_mrad"] / cam["sigma_z_mm"],
    }
    cam["level_mm"] = noise_level(cam["sigma_z_mm"])
    stds = []
    for sid in SETTINGS["mocap_noise_sessions"]:
        sess = sessions[sid]
        if sess.role != "diagnostic":
            raise PermissionError(f"{sid}: only diagnostic mocap runs enter the noise")
        stds.append(pd.read_csv(sess.mocap, usecols=["pose_std_mm"])["pose_std_mm"])
    mocap_std = float(np.median(np.concatenate([x.to_numpy() for x in stds])))
    mocap = {"pose_std_mm": mocap_std, "level_mm": noise_level(mocap_std)}
    return {"camera": cam, "mocap": mocap}


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    stages = ["noise"]
    parser.add_argument("stage", nargs="?", choices=stages, default=None)
    parser.add_argument("--stage", dest="stage_opt", choices=stages)
    parser.add_argument(
        "--no-plot", action="store_true", help="accepted for validate.py"
    )
    args = parser.parse_args(argv)
    est = noise_estimates()
    c, m = est["camera"], est["mocap"]
    print("effective noise (registration only, no joint fit)")
    print(
        f"  camera 11-27 center vs topleft: {c['pairs']} pairs, "
        f"sigma xy {c['sigma_xy_mm']:.2f} mm, z {c['sigma_z_mm']:.2f} mm, "
        f"tilt {c['sigma_tilt_mrad']:.1f} mrad, roll {c['sigma_roll_mrad']:.1f} mrad"
        f" -> grid level {c['level_mm']} mm"
    )
    r = c["ratios"]
    print(
        f"  ratios xy/z {r['xy']:.3f}, tilt {r['tilt_mrad_per_mm']:.3f} and roll "
        f"{r['roll_mrad_per_mm']:.3f} mrad per mm of sigma_z"
    )
    print(
        f"  mocap plateau pose_std {m['pose_std_mm']:.2f} mm -> grid level {m['level_mm']} mm"
    )


if __name__ == "__main__":
    main()
