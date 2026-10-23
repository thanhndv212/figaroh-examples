"""TIAGo base suspension (#70): the study's library, baselines and noise stage.

Question: is a linear spring-damper at the base enough to explain the force
plate wrench of the TIAGo xyz oscillation runs (Vicon day 2023-09-20), and does
it explain more than "the arm moved"? The measured quantity is the wrench the
robot applies to the plate (action on the plate), transported to the base
origin, minus its static value; the regressors are the base displacement
(translation of the origin, rotation vector) and its velocity, from the
corrected loader (``utils/suspension_data.py``).

    S1  w = K x + C v          K, C diagonal                (12 parameters)
    S2  S1 about an elastic centre at height h_e on base z  (12 + h_e)
    S3  K, C full symmetric                                  (21 + 21)
    S1B2  S1 plus the B2 arm load, nested (revision F3)     (12 + 4)

with one constant per DOF (static drift). Linear least squares on unit-RMS
columns and unit-variance rows (each DOF scaled by the standard deviation of
its target); standard errors from a moving-block bootstrap over time.

The fits are constrained (maintainer decision O4): S1/S2 k >= 0 and c >= 0
(bounded least squares), S3 K and C symmetric positive semi-definite (an SDP),
B2 m >= 0, S1B2 both. The origin is the base_footprint frame (O1), located from shipped
data (:func:`footprint_derivation`).

Baselines: B0 static (predicts the static wrench), B1 the status-quo loader +
S1 (what #70 reported), B2 rigid base: the wrench of a rigid cube at the
gripper markers moving with the measured arm motion (parameters: the mass and
the first moment; O2), the competitor that asks whether the suspension
explains anything beyond "the arm moved".

Part 1 reads no wrench to fit anything. :func:`read_target` refuses the
validation run until a study stage sets ``unlock_validation`` (part 2),
refuses the confirmation runs always (hash check only), and
:func:`read_regressors` refuses the confirmation runs too. Docs:
docs/development/tiago-suspension.md.

    python suspension_model.py noise      # freezes the noise of the fixture
    python suspension_model.py describe   # the descriptive reads of the note
    python suspension_model.py footprint  # the frozen base_footprint offset
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import yaml
from figaroh.data.protocol import Protocol
from scipy.optimize import lsq_linear

project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from examples.tiago.utils import suspension_data as sd  # noqa: E402

TIAGO = Path(__file__).resolve().parent
SUS = TIAGO / "data/suspension"
VICON_1607 = TIAGO / "data/calibration/vicon/vicon_2023-09-20-1607_static_postures.csv"
MOCAP_GOLDEN = project_root / "tests/golden/tiago_mocap_hardware.json"
PROTOCOL = SUS / "protocol.yaml"
CONFIG = TIAGO / "config/tiago_suspension_study_config.yaml"

DOFS = ("x", "y", "z", "rx", "ry", "rz")


def load_settings(path=CONFIG) -> dict:
    return yaml.safe_load(Path(path).read_text())


SETTINGS = load_settings()
RATE = float(SETTINGS["sampling"]["rate_hz"])
LEVELS = tuple(SETTINGS["noise"]["levels"])
USED = float(SETTINGS["noise"]["used_level"])
RULES = SETTINGS["rules"]
BOOT = SETTINGS["bootstrap"]
G = float(SETTINGS["gravity"])
LOADER = dict(
    rate=RATE,
    static_window_s=float(SETTINGS["static_window_s"]),
    guard_frames=int(SETTINGS["clock"]["guard_frames"]),
    edge_s=float(SETTINGS["clock"]["edge_s"]),
    cutoff_hz=float(SETTINGS["filter"]["cutoff_hz"]),
    plate_centre=tuple(SETTINGS["plate_centre"]),
    frame=SETTINGS["frame"],
    origin={"base_footprint": "footprint"}[SETTINGS["origin"]],
    footprint=dict(
        offset=np.asarray(SETTINGS["footprint"]["offset_triad_m"], dtype=float),
        rotvec=np.asarray(SETTINGS["footprint"]["rotation_triad_rotvec"], dtype=float),
    ),
)


# Real runs only (part 2 revision); simulated rows carry world Fy.
PLATE_FY_SIGN = float(SETTINGS["plate_fy_sign"])


def h_e_grid() -> np.ndarray:
    g = SETTINGS["h_e_grid"]
    n = int(round((g["max"] - g["min"]) / g["step"])) + 1
    return np.round(g["min"] + g["step"] * np.arange(n), 6)


# --------------------------------------------------------------- sessions


@dataclass
class Session:
    id: str
    role: str
    path: Path


def load_sessions(verify: bool = True) -> Dict[str, Session]:
    """``{id: Session}`` of the protocol (sha256-verified when ``verify``)."""
    protocol = Protocol.load(PROTOCOL)
    if verify:
        protocol.verify(root=PROTOCOL.parent)
    return {
        s.id: Session(s.id, s.role, SUS / next(iter(s.files)))
        for s in protocol.sessions
    }


def verify_hashes() -> List[str]:
    """sha256 check of every shipped file (the only thing done to a confirmation run)."""
    load_sessions(verify=True)
    return sorted(load_sessions(verify=False))


@lru_cache(maxsize=None)
def _run(session_id: str, mz_about: str = "centre") -> sd.SuspensionRun:
    session = load_sessions(verify=False)[session_id]
    return sd.load_run(
        session.path, mz_about=mz_about, plate_fy_sign=PLATE_FY_SIGN, **LOADER
    )


@dataclass
class Regressors:
    """Motion of one session: no wrench."""

    session: Session
    t: np.ndarray
    x: np.ndarray
    v: np.ndarray
    valid: np.ndarray
    interpolated: np.ndarray
    grip: np.ndarray
    grip_rot: np.ndarray
    shoulder: np.ndarray
    origin: np.ndarray
    ref: np.ndarray
    axes: np.ndarray
    stats: dict

    @property
    def n(self) -> int:
        return self.t.size


def read_regressors(session: Session, unlock_confirmation: bool = False) -> Regressors:
    """Base motion and arm marker clusters; never a plate channel.

    The confirmation runs are read only by the confirmation stage of part 2
    (``unlock_confirmation``), after validation says "enough".
    """
    if session.role == "confirmation" and not unlock_confirmation:
        raise PermissionError(
            f"{session.id}: confirmation runs are not read (hash check only)"
        )
    return _regressors(session, _run(session.id))


def _regressors(session: Session, r: sd.SuspensionRun) -> Regressors:
    return Regressors(
        session, r.t, r.x, r.v, r.valid, r.interpolated, r.grip, r.grip_rot,
        r.shoulder, r.origin, r.ref, r.axes, dict(r.stats),
    )  # fmt: skip


SYNTHETIC = Session("synthetic", "synthetic", Path("."))


def regressors_from_raw(t, values, **overrides) -> Tuple[Regressors, np.ndarray]:
    """``(Regressors, wrench)`` of simulated raw rows through the corrected loader.

    ``overrides`` replace the frozen loader settings (the ablation switches
    ``origin``, ``transport``, ``frame``, ``clock``, or another ``footprint``).
    Simulated rows only.
    """
    kw = dict(LOADER)
    kw.update(overrides)
    r = sd.process_raw(t, values, **kw)
    return _regressors(SYNTHETIC, r), r.w


def read_target(
    session: Session,
    unlock_validation: bool = False,
    unlock_confirmation: bool = False,
    mz_about: str = "centre",
) -> np.ndarray:
    """(n, 6) plate wrench about the origin, on the rows of :func:`read_regressors`.

    Training, and the diagnostic runs listed in ``sessions.selection_diagnostics``
    (model selection only, O3). The validation wrench is read by the study stage
    of part 2 (``unlock_validation``); the confirmation wrench by the
    confirmation stage (``unlock_confirmation``), which does not exist in part 1.
    Other diagnostic runs (the postures) are not targets. ``mz_about``: the
    ``Mz`` convention of the loader (O7, part 2 step 6).
    """
    if session.role == "confirmation" and not unlock_confirmation:
        raise PermissionError(
            f"{session.id}: confirmation runs are not read (hash check only)"
        )
    if session.role == "validation" and not unlock_validation:
        raise PermissionError(
            f"{session.id}: the validation wrench is read only by the study stage "
            "(unlock_validation)"
        )
    selection = SETTINGS["sessions"]["selection_diagnostics"]
    if session.role == "diagnostic" and session.id not in selection:
        raise PermissionError(f"{session.id}: not a selection run, not a target")
    if session.role not in ("training", "validation", "confirmation", "diagnostic"):
        raise PermissionError(f"{session.id}: {session.role} runs are not targets")
    return _run(session.id, mz_about).w


# ---------------------------------------------------------------- designs

MODELS = ("S1", "S2", "S3")


def _pair_index() -> List[Tuple[int, int]]:
    return [(a, b) for a in range(6) for b in range(a, 6)]


def parameter_names(model: str) -> List[str]:
    if model in ("S1", "S2"):
        return [f"{p}_{d}" for d in DOFS for p in ("k", "c")]
    if model == "S1B2":
        return parameter_names("S1") + list(B2_NAMES)
    if model == "B2":
        return list(B2_NAMES)
    if model == "S3":
        pairs = _pair_index()
        return [f"K_{DOFS[a]}{DOFS[b]}" for a, b in pairs] + [
            f"C_{DOFS[a]}{DOFS[b]}" for a, b in pairs
        ]
    raise ValueError(model)


def elastic_maps(h_e: float) -> Tuple[np.ndarray, np.ndarray]:
    """``(A, B)`` with ``w = A diag(k) B x + A diag(c) B v``.

    ``B`` carries the base displacement to the displacement of the elastic
    centre at ``(0, 0, h_e)`` (translation ``x + theta x r``); ``A`` carries the
    force and moment at the centre to the wrench about the origin
    (``m + r x f``). ``h_e = 0`` gives the identity (S1).
    """
    b = np.eye(6)
    b[0, 4], b[1, 3] = h_e, -h_e
    a = np.eye(6)
    a[3, 1], a[4, 0] = -h_e, h_e
    return a, b


def spring_design(
    model: str, x: np.ndarray, v: np.ndarray, h_e: float = 0.0
) -> np.ndarray:
    """(n, 6, p) regressor of the spring-damper models (offset excluded)."""
    n = x.shape[0]
    if model in ("S1", "S2"):
        a, b = elastic_maps(h_e if model == "S2" else 0.0)
        y, yd = x @ b.T, v @ b.T
        out = np.zeros((n, 6, 12))
        for j in range(6):
            out[:, :, 2 * j] = y[:, j][:, None] * a[:, j][None, :]
            out[:, :, 2 * j + 1] = yd[:, j][:, None] * a[:, j][None, :]
        return out
    if model == "S3":
        pairs = _pair_index()
        out = np.zeros((n, 6, 2 * len(pairs)))
        for q, (a_, b_) in enumerate(pairs):
            for off, u in ((0, x), (len(pairs), v)):
                out[:, a_, off + q] = u[:, b_]
                if a_ != b_:
                    out[:, b_, off + q] = u[:, a_]
        return out
    raise ValueError(model)


B2_NAMES = ["m", "mo_x", "mo_y", "mo_z"]


def cube_kinematics(grip, grip_rot) -> dict:
    """Accelerations of the gripper triad: ``c''`` (n, 3), ``R''`` (n, 3, 3), ``w'`` (n, 3).

    ``grip`` and ``grip_rot`` are the filtered centroid and frame of the gripper
    markers (world); ``w'`` is the angular acceleration (``w`` from ``R' R^T``).
    """
    dt = 1.0 / RATE
    rd = np.gradient(grip_rot, dt, axis=0)
    spin = np.einsum("nab,ncb->nac", rd, grip_rot)  # R' R^T = [w]x
    omega = np.column_stack((spin[:, 2, 1], spin[:, 0, 2], spin[:, 1, 0]))
    return {
        "cdd": np.gradient(np.gradient(grip, dt, axis=0), dt, axis=0),
        "rdd": np.gradient(rd, dt, axis=0),
        "omega_dot": np.gradient(omega, dt, axis=0),
    }


def arm_design(run, edge: Optional[float] = None) -> np.ndarray:
    """(n, 6, 4) wrench per unit ``[m, m o_x, m o_y, m o_z]`` of the B2 cube, rigid base.

    A rigid cube at the gripper markers, centre ``c + R o`` (``c``, ``R`` the
    centroid and frame of the gripper triad, ``o`` the centre in that frame),
    mass ``m``, isotropic inertia ``m a^2 / 6``. The wrench the plate receives
    (action), about the base origin and in the run's axes:

        F = m (g - c'') - R'' (m o)
        M = (c - p) x F + R (m o) x (g - c'') - (m a^2 / 6) w'

    linear in ``m`` and ``m o``; the quadratic term ``(R o) x R'' (m o)`` is
    neglected, ``g = (0, 0, -9.81)``. The static part is absorbed by the offset.
    ``edge`` is ``a`` (default ``b2.edge_m`` of the config).
    """
    a = SETTINGS["b2"]["edge_m"] if edge is None else edge
    kin = cube_kinematics(run.grip, run.grip_rot)
    f0 = np.array([0.0, 0.0, -G]) - kin["cdd"]
    lever = run.grip - run.origin
    n = f0.shape[0]
    out = np.zeros((n, 6, 4))
    out[:, :3, 0] = f0
    out[:, 3:, 0] = np.cross(lever, f0) - (a**2 / 6.0) * kin["omega_dot"]
    for j in range(3):
        rdd_j = kin["rdd"][:, :, j]
        out[:, :3, 1 + j] = -rdd_j
        out[:, 3:, 1 + j] = -np.cross(lever, rdd_j) + np.cross(
            run.grip_rot[:, :, j], f0
        )
    return np.concatenate(
        (
            np.einsum("ka,nkc->nac", run.axes, out[:, :3]),
            np.einsum("ka,nkc->nac", run.axes, out[:, 3:]),
        ),
        axis=1,
    )


def design(
    model: str, run: Regressors, h_e: float = 0.0, edge: Optional[float] = None
) -> np.ndarray:
    """(n, 6, p) regressor of S1, S2 (at ``h_e``), S3, B2 (cube ``edge``) or S1B2 (offset excluded)."""
    if model == "B2":
        return arm_design(run, edge)
    if model == "S1B2":
        return np.concatenate(
            (spring_design("S1", run.x, run.v), arm_design(run, edge)), axis=2
        )
    return spring_design(model, run.x, run.v, h_e)


# -------------------------------------------------------------------- fit


@dataclass
class Fit:
    model: str
    names: List[str]
    params: np.ndarray  # model parameters then the 6 offsets (if offset)
    h_e: float
    offset: bool
    weights: np.ndarray  # (6,) row weights (1 / sd of the target)
    rank: int
    cond: float  # scaled condition number of the weighted design
    se: Optional[np.ndarray] = None
    active: Optional[np.ndarray] = (
        None  # (p,) bool: parameter at its bound (box models)
    )
    constrained: bool = True
    edge: Optional[float] = None  # B2 cube edge used (None: the config's)

    @property
    def p(self) -> int:
        return len(self.names)

    @property
    def theta(self) -> np.ndarray:
        return self.params[: self.p]

    def k(self) -> np.ndarray:
        """Diagonal stiffness (S1, S2, S1B2) or diagonal of K (S3), length 6."""
        if self.model in ("S1", "S2", "S1B2"):
            return self.theta[:12:2]
        pairs = _pair_index()
        return np.array([self.theta[pairs.index((d, d))] for d in range(6)])

    def c(self) -> np.ndarray:
        if self.model in ("S1", "S2", "S1B2"):
            return self.theta[1:12:2]
        pairs = _pair_index()
        return np.array(
            [self.theta[len(pairs) + pairs.index((d, d))] for d in range(6)]
        )

    def c_se(self) -> Optional[np.ndarray]:
        if self.se is None:
            return None
        if self.model in ("S1", "S2", "S1B2"):
            return self.se[1:12:2]
        pairs = _pair_index()
        return np.array([self.se[len(pairs) + pairs.index((d, d))] for d in range(6)])

    def k_se(self) -> Optional[np.ndarray]:
        if self.se is None:
            return None
        if self.model in ("S1", "S2", "S1B2"):
            return self.se[:12:2]
        pairs = _pair_index()
        return np.array([self.se[pairs.index((d, d))] for d in range(6)])

    def matrices(self) -> Tuple[np.ndarray, np.ndarray]:
        """``(K, C)`` 6x6 (S3 full, S1 and S2 diagonal)."""
        if self.model == "S3":
            out = []
            for off in (0, len(_pair_index())):
                m = np.zeros((6, 6))
                for q, (a, b) in enumerate(_pair_index()):
                    m[a, b] = m[b, a] = self.theta[off + q]
                out.append(m)
            return out[0], out[1]
        return np.diag(self.k()), np.diag(self.c())

    def bound_dofs(self) -> np.ndarray:
        """(6,) bool, the DOF with a parameter at its bound (reported).

        S1, S2, S1B2: ``k`` or ``c`` of the DOF at 0. S3: ``K`` or ``C`` with an
        eigenvalue below ``rules.psd_zero_rel`` of the largest flags all six DOF.
        The verdict uses :meth:`stiffness_bound_dofs` (O9).
        """
        if self.model in ("S1", "S2", "S1B2"):
            a = np.asarray(self.active, bool)
            return a[0:12:2] | a[1:12:2]
        if self.model == "S3":
            tol = RULES["psd_zero_rel"]
            zero = False
            for m in self.matrices():
                ev = np.linalg.eigvalsh(m)
                zero = zero or bool(ev[0] <= tol * max(ev[-1], 1e-300))
            return np.full(6, zero)
        raise ValueError(self.model)

    def stiffness_bound_dofs(self) -> np.ndarray:
        """(6,) bool, the verdict item "active bound" (O9): a stiffness at its bound.

        S1, S2, S1B2: ``k`` of the DOF at 0. S3: ``K`` with a zero eigenvalue
        flags all six DOF. A damping at its bound is :meth:`damping_unidentified`.
        """
        if self.model in ("S1", "S2", "S1B2"):
            return np.asarray(self.active, bool)[0:12:2].copy()
        if self.model == "S3":
            ev = np.linalg.eigvalsh(self.matrices()[0])
            return np.full(6, bool(ev[0] <= RULES["psd_zero_rel"] * max(ev[-1], 1e-300)))
        raise ValueError(self.model)

    def damping_unidentified(self) -> np.ndarray:
        """(6,) bool (O9): the damping is at its bound or its relative SE is above
        ``rules.damping_rel_se_max`` (the SE test needs the bootstrap). Reported;
        the DOF is then judged on its stiffness and R2.
        """
        if self.model not in ("S1", "S2", "S1B2"):
            raise ValueError(self.model)
        out = np.asarray(self.active, bool)[1:12:2].copy()
        if self.se is not None:
            c = np.abs(self.c())
            rel = np.where(c > 0, self.c_se() / np.maximum(c, 1e-300), np.inf)
            out |= rel > RULES["damping_rel_se_max"]
        return out

    def b2_at_bound(self) -> bool:
        """B2 or S1B2: the mass is at its bound ``m >= 0``."""
        return bool(np.asarray(self.active, bool)[12 if self.model == "S1B2" else 0])


def _full_design(x3: np.ndarray, offset: bool) -> np.ndarray:
    if not offset:
        return x3
    n = x3.shape[0]
    eye = np.broadcast_to(np.eye(6), (n, 6, 6))
    return np.concatenate((x3, eye), axis=2)


def block_indices(n: int, block: int, rng: np.random.Generator) -> np.ndarray:
    """Moving-block resample of ``range(n)`` (blocks of ``block`` consecutive rows)."""
    block = max(1, min(block, n))
    starts = rng.integers(0, n - block + 1, size=int(np.ceil(n / block)))
    return (starts[:, None] + np.arange(block)[None, :]).reshape(-1)[:n]


def _psd_basis(unit: np.ndarray, diag_scale: np.ndarray, off: int, n_par: int):
    """(36, n_par) map from the scaled parameters to the normalised matrix, column-major.

    ``unit`` converts a scaled parameter to its physical value; ``diag_scale``
    (6,) is the positive diagonal ``d`` of the similarity ``D^-1 M D^-1`` that
    keeps the semidefinite constraint well conditioned.
    """
    basis = np.zeros((36, n_par))
    for q, (a, b) in enumerate(_pair_index()):
        val = unit[off + q] / (diag_scale[a] * diag_scale[b])
        basis[a + 6 * b, off + q] = val
        basis[b + 6 * a, off + q] = val
    return basis


def _solve_psd(r: np.ndarray, z: np.ndarray, scale: np.ndarray, p: int) -> np.ndarray:
    """Scaled S3 parameters: ``min |R th - z|^2`` with ``K`` and ``C`` PSD (SDP)."""
    import picos  # core dependency of figaroh; lazy, the box fits do not need it

    n_par = r.shape[1]
    unit = np.where(np.isfinite(scale), 1.0 / scale, 0.0)
    free = np.linalg.lstsq(r, z, rcond=None)[0]
    pairs = _pair_index()
    n_pair = len(pairs)
    prob = picos.Problem()
    th = picos.RealVariable("th", n_par)
    r_c, z_c = picos.Constant("R", r), picos.Constant("z", z)
    # Divided by |z|^2: the same minimiser, but cvxopt fails (math domain error)
    # on the unscaled objective of the real training run (#70 part 2).
    prob.set_objective("min", abs(r_c * th - z_c) ** 2 / max(float(z @ z), 1e-300))
    for off in (0, n_pair):
        diag = np.array(
            [
                free[off + pairs.index((d, d))] * unit[off + pairs.index((d, d))]
                for d in range(6)
            ]
        )
        floor = 1e-3 * max(float(np.abs(diag).max()), 1e-300)
        d = np.sqrt(np.maximum(np.abs(diag), floor))
        mat = (picos.Constant("M", _psd_basis(unit, d, off, n_par)) * th).reshaped(
            (6, 6)
        )
        prob.add_constraint(mat >> 0)
    prob.solve(solver="cvxopt", verbosity=0)
    if prob.status != "optimal":
        raise RuntimeError(f"S3 PSD fit failed: {prob.status}")
    return np.asarray(th.value, dtype=float).reshape(-1)


def _solve(
    x3: np.ndarray,
    y: np.ndarray,
    weights: np.ndarray,
    model: str,
    p: int,
    scale=None,
    constrained: bool = True,
):
    """``(params, scale, R, active)``: weighted least squares of ``y`` on ``x3``.

    The weighted design is reduced to its triangular factor ``R`` (rows
    stacked, columns scaled to unit RMS), so a bounded or semidefinite fit costs
    the same however many rows. ``p`` leading columns are the model parameters
    (the rest are the offsets, free). Constraints (``constrained``): S1/S2 all
    ``p`` parameters >= 0, B2 ``m`` >= 0, S1B2 the 12 spring parameters and
    ``m`` >= 0 (bounded least squares); S3 ``K`` and
    ``C`` PSD (semidefinite program). ``active`` flags the box parameters at
    their bound.
    """
    xw = x3 * weights[None, :, None]
    yw = y * weights[None, :]
    a = xw.reshape(-1, x3.shape[2])
    if scale is None:
        scale = np.sqrt(np.mean(a**2, axis=0))
        scale = np.where(scale > 1e-12 * max(float(scale.max()), 1e-30), scale, np.inf)
    n_par = a.shape[1]
    tri = np.linalg.qr(np.column_stack((a / scale, yw.reshape(-1))), mode="r")
    r, z = tri[:n_par, :n_par], tri[:n_par, n_par]
    active = np.zeros(p, bool)
    if constrained and model == "S3":
        sol = _solve_psd(r, z, scale, p)
    elif constrained and model in ("S1", "S2", "B2", "S1B2"):
        lower = np.full(n_par, -np.inf)
        lower[: {"B2": 1, "S1B2": 13}.get(model, p)] = 0.0
        res = lsq_linear(r, z, bounds=(lower, np.inf), method="bvls")
        sol, active = res.x, res.active_mask[:p] < 0
    else:
        sol = np.linalg.lstsq(r, z, rcond=None)[0]
    return sol / scale, scale, r, active


def fit_model(
    model: str,
    run: Regressors,
    w: np.ndarray,
    h_e: float = 0.0,
    offset: Optional[bool] = None,
    weights: Optional[np.ndarray] = None,
    bootstrap: bool = False,
    rows: Optional[np.ndarray] = None,
    resamples: Optional[int] = None,
    constrained: bool = True,
    edge: Optional[float] = None,
) -> Fit:
    """Fit ``model`` (S1, S2 at ``h_e``, S3, B2, S1B2) to the target ``w`` (n, 6).

    Rows: ``run.valid`` (or ``rows``). ``weights`` default to one over the
    standard deviation of the target on those rows. ``constrained=False`` is the
    unconstrained least squares (the ablation of decision O4); the bootstrap uses
    the same constraints as the fit. ``edge``: the B2 cube edge (default ``b2.edge_m``).
    """
    offset = SETTINGS["offset"] if offset is None else offset
    use = run.valid if rows is None else rows
    x3 = design(model, run, h_e, edge)[use]
    y = np.asarray(w)[use]
    if weights is None:
        weights = 1.0 / np.maximum(y.std(axis=0), 1e-12)
    names = parameter_names(model)
    p = len(names)
    full = _full_design(x3, offset)
    sol, scale, r, active = _solve(full, y, weights, model, p, constrained=constrained)
    sv = np.linalg.svd(r, compute_uv=False)
    rank = int(np.sum(sv > sv[0] * max(r.shape) * np.finfo(float).eps))
    fit = Fit(
        model, names, sol, h_e, offset, weights, rank, float(sv[0] / sv[-1]),
        active=active, constrained=constrained, edge=edge,
    )  # fmt: skip
    if bootstrap:
        rng = np.random.default_rng(BOOT["seed"])
        block = int(round(BOOT["block_s"] * RATE))
        draws = []
        for _ in range(resamples or BOOT["resamples"]):
            idx = block_indices(full.shape[0], block, rng)
            draws.append(
                _solve(full[idx], y[idx], weights, model, p, scale, constrained)[0][:p]
            )
        fit.se = np.std(draws, axis=0, ddof=1)
    return fit


def fit_s1_eiv(run: Regressors, w: np.ndarray, level: float = USED) -> np.ndarray:
    """(12,) S1 ``[k_x, c_x, ...]`` corrected for errors in the regressors (sensitivity row).

    Per DOF, the normal equations of ``w_j = k x_j + c v_j + const`` with the
    expected noise variance of the displacement and velocity columns
    (``noise.x_sd``, ``noise.v_sd`` of the config times ``level``) removed from
    their diagonal: the classical errors-in-variables correction of the
    attenuation of ``k`` and ``c`` by the marker noise. Unconstrained;
    descriptive only (the fit of record is :func:`fit_model`).
    """
    n_cfg = noise_levels()
    xs = level * np.asarray(n_cfg["x_sd"], float)
    vs = level * np.asarray(n_cfg["v_sd"], float)
    use = run.valid
    x, v, y = run.x[use], run.v[use], np.asarray(w)[use]
    n = y.shape[0]
    out = np.zeros(12)
    for j in range(6):
        a = np.column_stack((x[:, j], v[:, j], np.ones(n)))
        g = a.T @ a
        g[0, 0] -= n * xs[j] ** 2
        g[1, 1] -= n * vs[j] ** 2
        out[2 * j : 2 * j + 2] = np.linalg.solve(g, a.T @ y[:, j])[:2]
    return out


def design_condition(model: str, run: Regressors, h_e: float = 0.0) -> float:
    """Scaled condition number of the weighted design (offset included)."""
    x3 = _full_design(design(model, run, h_e)[run.valid], SETTINGS["offset"])
    a = x3.reshape(-1, x3.shape[2])
    a = a / np.sqrt(np.mean(a**2, axis=0))
    sv = np.linalg.svd(a, compute_uv=False)
    return float(sv[0] / sv[-1])


def fit_status_quo(x: np.ndarray, v: np.ndarray, w: np.ndarray):
    """B1: the status-quo S1 fit (plain LS, no offset, no weights) on any (x, v, w)."""
    from examples.tiago.utils.suspension_model import fit_generalized_base_suspension

    return fit_generalized_base_suspension(x, v, w.reshape(-1, order="F"))


def predict_status_quo(theta: np.ndarray, x: np.ndarray, v: np.ndarray) -> np.ndarray:
    """(n, 6) wrench of the 12-parameter status-quo model."""
    from examples.tiago.utils.suspension_model import build_generalized_base_regressor

    n = x.shape[0]
    return (build_generalized_base_regressor(x, v) @ theta).reshape(6, n).T


def predict(fit: Fit, run: Regressors) -> np.ndarray:
    """(n, 6) dynamic wrench of the fit on a run (the offset is not part of it)."""
    return design(fit.model, run, fit.h_e, fit.edge) @ fit.theta


def score(w: np.ndarray, pred: np.ndarray, valid: np.ndarray) -> Dict[str, np.ndarray]:
    """Per-DOF R2 and NRMSE with the mean of target and prediction removed.

    A constant is not a suspension quantity (static drift): both are centred
    on the usable rows, so B0 (``pred = 0``) scores R2 = 0 by construction.
    """
    y = np.asarray(w)[valid]
    p = np.asarray(pred)[valid]
    y, p = y - y.mean(axis=0), p - p.mean(axis=0)
    sst = np.sum(y**2, axis=0)
    sse = np.sum((y - p) ** 2, axis=0)
    return {
        "r2": 1.0 - sse / sst,
        "nrmse": np.sqrt(sse / y.shape[0]) / np.sqrt(sst / y.shape[0]),
    }


def nonlinear_flag(x_d: np.ndarray, resid: np.ndarray, valid: np.ndarray) -> float:
    """Correlation of a residual with the cubic part of the displacement.

    ``x^3`` is orthogonalised against ``[1, x]``; a spring that is not linear
    leaves its cubic part in the residual. The DOF is flagged above
    ``rules.nonlinear_corr``.
    """
    x = np.asarray(x_d)[valid]
    r = np.asarray(resid)[valid]
    basis = np.column_stack((np.ones_like(x), x))
    c3 = x**3 - basis @ np.linalg.lstsq(basis, x**3, rcond=None)[0]
    r = r - r.mean()
    d = np.linalg.norm(c3) * np.linalg.norm(r)
    return float(c3 @ r / d) if d > 0 else 0.0


# -------------------------------------------------------------- selection


def cv_folds(run: Regressors) -> np.ndarray:
    """Fold id of each usable row: blocks of ``cv_block_s`` dealt to the folds."""
    sel = SETTINGS["selection"]
    n = int(run.valid.sum())
    block = int(round(sel["cv_block_s"] * RATE))
    return (np.arange(n) // block) % sel["folds"]


def cv_score(
    model: str,
    data: Sequence[Tuple[Regressors, np.ndarray]],
    h_e: float = 0.0,
    offset: Optional[bool] = None,
    constrained: bool = True,
) -> float:
    """Blocked CV score: pooled held-out RMSE over the target sd, mean over DOF.

    ``data`` is a list of ``(run, wrench)`` (training, and the selection
    diagnostics, O3); the blocks of every run are dealt to the folds, the fit
    pools all runs (one offset per DOF).
    """
    offset = SETTINGS["offset"] if offset is None else offset
    xs = [design(model, run, h_e)[run.valid] for run, _ in data]
    ys = [np.asarray(w)[run.valid] for run, w in data]
    folds = np.concatenate([cv_folds(run) for run, _ in data])
    x3, y = np.concatenate(xs), np.concatenate(ys)
    weights = 1.0 / np.maximum(y.std(axis=0), 1e-12)
    full = _full_design(x3, offset)
    p = x3.shape[2]
    sse = np.zeros(6)
    for f in np.unique(folds):
        tr, te = folds != f, folds == f
        sol = _solve(full[tr], y[tr], weights, model, p, constrained=constrained)[0]
        res = y[te] - full[te] @ sol
        res = res - res.mean(axis=0)  # a constant is not a suspension quantity
        sse += np.sum(res**2, axis=0)
    return float(np.mean(np.sqrt(sse / y.shape[0]) / y.std(axis=0)))


def select_model(data: Sequence[Tuple[Regressors, np.ndarray]]) -> dict:
    """The simplest of S1 < S2 < S3 within ``tolerance`` of the best CV score.

    ``data``: ``(run, wrench)`` of the training run and, with
    ``selection.use_diagnostics``, of the selection diagnostics (O3). S2 takes
    the ``h_e`` of the grid with the lowest CV score. The validation run is never
    an argument here.
    """
    tol = SETTINGS["selection"]["tolerance"]
    s1 = cv_score("S1", data)
    grid = h_e_grid()
    s2_all = np.array([cv_score("S2", data, h) for h in grid])
    i = int(np.argmin(s2_all))
    s3 = cv_score("S3", data)
    best = min(s1, float(s2_all[i]), s3)
    if s1 <= (1 + tol) * best:
        model = "S1"
    elif s2_all[i] <= (1 + tol) * best:
        model = "S2"
    else:
        model = "S3"
    return {
        "model": model,
        "h_e": float(grid[i]) if model == "S2" else 0.0,
        "cv": {"S1": s1, "S2": float(s2_all[i]), "S3": s3},
        "h_e_cv": float(grid[i]),
    }


def selection_sessions() -> List[Session]:
    """Training session then the selection diagnostics (part 2 reads their targets)."""
    sessions = load_sessions(verify=False)
    ids = [SETTINGS["sessions"]["training"]]
    if SETTINGS["selection"]["use_diagnostics"]:
        ids += list(SETTINGS["sessions"]["selection_diagnostics"])
    return [sessions[i] for i in ids]


# --------------------------------------------------------------- verdicts


def verdict(
    r2_model: np.ndarray,
    r2_b2: np.ndarray,
    at_bound: np.ndarray,
    repeatable: np.ndarray,
    nonlinear: np.ndarray,
    answerable: Optional[np.ndarray] = None,
    b2_at_bound: bool = False,
) -> dict:
    """The "spring-damper is enough" rule (note, section 8), per DOF arrays of 6.

    ``at_bound``: the DOF has a stiffness at its bound
    (:meth:`Fit.stiffness_bound_dofs`; a damping at its bound or with a large SE
    is "damping not identified", :meth:`Fit.damping_unidentified`, and does not
    fail the DOF, O9).
    ``b2_at_bound``: the B2 mass is at its bound, so the comparison with B2 is
    vacuous and "enough" fails.
    """
    answerable = (
        np.ones(6, bool) if answerable is None else np.asarray(answerable, bool)
    )
    explained = r2_model >= RULES["explained_r2"]
    beats = (r2_model - r2_b2) >= RULES["beat_b2"]
    ok = explained & beats & ~at_bound & repeatable & ~nonlinear
    return {
        "explained": explained.tolist(),
        "beats_b2": beats.tolist(),
        "dof_ok": ok.tolist(),
        "b2_at_bound": bool(b2_at_bound),
        "enough": bool(answerable.any() and np.all(ok[answerable]) and not b2_at_bound),
    }


def suspension_adds(r2_s1b2: np.ndarray, r2_b2: np.ndarray, k_at_bound: np.ndarray) -> dict:
    """Q2 (secondary): does the suspension add beyond the arm load, per DOF.

    Present on a DOF if the held-out R2 of S1B2 exceeds that of B2 by
    ``rules.suspension_adds`` and its stiffness (in S1B2) is not at its bound.
    """
    gain = np.asarray(r2_s1b2) - np.asarray(r2_b2)
    adds = (gain >= RULES["suspension_adds"]) & ~np.asarray(k_at_bound, bool)
    return {"gain": gain.tolist(), "adds": adds.tolist(), "present": bool(adds.any())}


# ------------------------------------------------------------ noise stage


def _robust_sd(x: np.ndarray) -> np.ndarray:
    """White-noise sd from first differences: 1.4826 MAD(diff) / sqrt 2."""
    d = np.diff(x, axis=0)
    return 1.4826 * np.median(np.abs(d - np.median(d, axis=0)), axis=0) / np.sqrt(2.0)


def clock_statistics(session_ids: Sequence[str]) -> dict:
    """Pooled receive-clock statistics of the given (non-confirmation) runs."""
    sessions = load_sessions(verify=False)
    rows = frames = drops = dups = 0
    lat, rho = [], []
    for sid in session_ids:
        if sessions[sid].role == "confirmation":
            raise PermissionError(f"{sid}: confirmation runs are not read")
        t, v = sd.read_raw(sessions[sid].path)
        c = sd.rebuild_clock(t, v, RATE)
        rows += t.size
        frames += c.n_frames
        drops += c.drops
        dups += c.duplicates
        lat.append(c.latency)
        rho.append(np.corrcoef(c.latency[:-1], c.latency[1:])[0, 1])
    lat_all = np.concatenate(lat)
    return {
        "drop_rate": drops / frames,
        "duplicate_rate": dups / rows,
        "latency_mean_ms": 1e3 * float(lat_all.mean()),
        "latency_sd_ms": 1e3 * float(lat_all.std()),
        "latency_lag1": float(np.mean(rho)),
    }


def noise_stage(floor_id: str = None, seed: int = 0, n_sim: int = 20000) -> dict:
    """White noise levels from the rest run, and the same pushed through the loader.

    The displacement and velocity column noise is that of the pose of the frozen
    origin and axes (the footprint, O1; the first version used the triad centroid).
    """
    floor_id = floor_id or SETTINGS["sessions"]["noise_floor"]
    session = load_sessions(verify=False)[floor_id]
    if session.role != "diagnostic":
        raise PermissionError(f"{floor_id}: the noise floor is a diagnostic run")
    t, v = sd.read_raw(session.path)
    marker_sd = float(np.mean(_robust_sd(v[:, 0:9])))
    plate_sd = _robust_sd(v[:, 30:36])
    ref = _run(floor_id).ref
    rng = np.random.default_rng(seed)
    mk = ref[None] + marker_sd * rng.normal(size=(n_sim, 3, 3))
    trans, rot, _, _ = sd.base_pose(
        mk, ref, origin=LOADER["origin"], frame=LOADER["frame"],
        footprint=LOADER["footprint"],
    )  # fmt: skip
    x = sd.lowpass(np.column_stack((trans, rot)), RATE, LOADER["cutoff_hz"])
    vel = np.gradient(x, 1.0 / RATE, axis=0)
    wn = sd.lowpass(plate_sd * rng.normal(size=(n_sim, 6)), RATE, LOADER["cutoff_hz"])
    cut = slice(int(LOADER["edge_s"] * RATE), -int(LOADER["edge_s"] * RATE))
    ids = [
        SETTINGS["sessions"]["training"],
        SETTINGS["sessions"]["validation"],
        floor_id,
    ]
    return {
        "marker_sd_mm": 1e3 * marker_sd,
        "plate_sd": plate_sd.tolist(),
        "plate_sd_filtered": wn[cut].std(axis=0).tolist(),
        "x_sd": x[cut].std(axis=0).tolist(),
        "v_sd": vel[cut].std(axis=0).tolist(),
        "clock": clock_statistics(ids),
    }


def noise_levels() -> dict:
    """Frozen noise (from the config); the noise stage must have been run."""
    n = SETTINGS["noise"]
    if n["x_sd"] is None:
        raise RuntimeError(
            "noise stage not frozen in the config: run `suspension_model.py noise`"
        )
    return n


# ----------------------------------------------------------- footprint


def footprint_derivation(
    csv_path=VICON_1607, golden_path=MOCAP_GOLDEN, fit: str = "V1607|3d"
) -> dict:
    """The base_footprint origin and axes in the base-triad frame, from shipped data.

    Expresses the base pose of the #100 Vicon 1607 fit (``golden_path``, the
    fit's ``params`` and ``std``) in the frame of the 1607 base-triad pose
    (``xb..phizb`` of ``csv_path``: mean and std over the postures, pinocchio
    RPY). Returns ``offset`` (m, triad frame) with ``offset_sd`` (the fit std
    and the triad scatter through the triad rotation; the rotation uncertainty
    is neglected), ``rotvec`` (triad frame to footprint axes) and ``rotation_sd``
    (mean of the fit's base_phi* std). The values are frozen in the config.
    """
    import pandas as pd
    from scipy.spatial.transform import Rotation

    cols = ["xb", "yb", "zb", "phixb", "phiyb", "phizb"]
    pose = pd.read_csv(csv_path)[cols].to_numpy(dtype=float)
    pose = pose[~np.isnan(pose).any(axis=1)]
    mean, scatter = pose.mean(axis=0), pose.std(axis=0)
    r_tri = Rotation.from_euler("xyz", mean[3:]).as_matrix()
    names = ["base_px", "base_py", "base_pz", "base_phix", "base_phiy", "base_phiz"]
    entry = json.loads(Path(golden_path).read_text())["fits"][fit]
    params = np.array([entry["params"][k] for k in names])
    std = np.array([entry["std"][k] for k in names])
    r_fp = Rotation.from_euler("xyz", params[3:]).as_matrix()
    offset = r_tri.T @ (params[:3] - mean[:3])
    offset_sd = np.sqrt((r_tri.T**2) @ (std[:3] ** 2 + scatter[:3] ** 2))
    rotvec = Rotation.from_matrix(r_tri.T @ r_fp).as_rotvec()
    return {
        "offset_triad_m": offset.tolist(),
        "offset_sd_m": offset_sd.tolist(),
        "rotation_triad_rotvec": rotvec.tolist(),
        "rotation_sd_rad": float(std[3:].mean()),
        "horizontal_m": float(np.hypot(*offset[:2])),
        "triad_z_world": r_tri[:, 2].tolist(),
        "fit": fit,
    }


# ----------------------------------------------------------- descriptives


def near_duplicates(values: np.ndarray) -> dict:
    """O6: rows whose markers all lie within ``clock.near_duplicate_sd`` marker sd of
    the previous row while the gripper moves (neighbouring steps above 20 sd)."""
    sd_m = 1e-3 * float(SETTINGS["noise"]["marker_sd_mm"])
    tol = float(SETTINGS["clock"]["near_duplicate_sd"]) * sd_m
    mk = np.asarray(values)[:, :30]
    step = np.abs(np.diff(mk, axis=0)).max(axis=1)
    g = mk[:, 21:30].reshape(-1, 3, 3).mean(axis=1)
    gs = np.linalg.norm(np.diff(g, axis=0), axis=1)
    nb = 0.5 * (np.r_[gs[0], gs[:-1]] + np.r_[gs[1:], gs[-1]])
    moving = nb > 20 * sd_m
    near = moving & (step <= tol)
    return {
        "count": int(near.sum()),
        "rate": float(near.sum() / step.size),
        "moving_rows": int(moving.sum()),
    }


def mz_convention(values: np.ndarray) -> dict:
    """O7: RMS of the z component of ``(COP - c) x F`` (plate columns), against ``Mz``.

    If ``Mz`` were about the COP rather than the plate centre ``c``, the moment
    about ``c`` would differ by this term.
    """
    c = np.asarray(SETTINGS["plate_centre"], float)
    f, m, cop = values[:, 30:33], values[:, 33:36], values[:, 36:39]
    dz = (cop[:, 0] - c[0]) * PLATE_FY_SIGN * f[:, 1] - (cop[:, 1] - c[1]) * f[:, 0]
    return {"term_rms": float(np.std(dz)), "mz_sd": float(np.std(m[:, 2]))}


def fz_mode(values: np.ndarray, band=(10.0, 45.0)) -> dict:
    """O8: the largest peak of the unfiltered ``Fz`` spectrum in ``band`` (Hz) and the
    vertical stiffness ``m (2 pi f)^2`` it implies, ``m`` from the static ``Fz``."""
    from scipy.signal import welch

    fz = values[:, 32]
    f, pxx = welch(fz - fz.mean(), fs=RATE, nperseg=1024)
    sel = (f > band[0]) & (f < band[1])
    i = int(np.argmax(pxx[sel]))
    peak = float(f[sel][i])
    n0 = int(SETTINGS["static_window_s"] * RATE)
    mass = float(abs(fz[:n0].mean()) / G)
    return {
        "peak_hz": peak,
        "peak_over_median": float(pxx[sel][i] / np.median(pxx[sel])),
        "mass_kg": mass,
        "k_z": mass * (2 * np.pi * peak) ** 2,
    }


def describe() -> dict:
    """The descriptive reads of the note (no suspension fit, no validation target).

    O7 and O8 read the plate columns of the training run and the noise floor only
    (as the plate-centre check does).
    """
    sessions = load_sessions(verify=True)
    out = {}
    for sid, s in sessions.items():
        if s.role == "confirmation":
            continue
        t, v = sd.read_raw(s.path)
        c = sd.rebuild_clock(t, v, RATE)
        r = read_regressors(s)
        m = r.valid
        out[sid] = {
            "role": s.role,
            "clock": c.stats(),
            "frame_yaw_deg": r.stats["frame_yaw_deg"],
            "origin": r.origin.tolist(),
            "x_ptp": np.ptp(r.x[m], axis=0).tolist(),
            "x_sd": r.x[m].std(axis=0).tolist(),
            "v_sd": r.v[m].std(axis=0).tolist(),
            "near_duplicates": near_duplicates(v),
        }
        if s.role == "training" or sid == SETTINGS["sessions"]["noise_floor"]:
            centre, res = sd.plate_centre_fit(v[:, 30:33], v[:, 33:36], v[:, 36:39])
            out[sid]["plate_centre"] = centre.tolist()
            out[sid]["plate_centre_residual_rms"] = res
            out[sid]["mz_convention"] = mz_convention(v)
            out[sid]["fz_mode"] = fz_mode(v)
    return out


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("stage", choices=("noise", "describe", "footprint"))
    args = parser.parse_args(argv)
    res = {
        "noise": noise_stage,
        "describe": describe,
        "footprint": footprint_derivation,
    }[args.stage]()
    print(json.dumps(res, indent=1, default=float))


if __name__ == "__main__":
    main()
