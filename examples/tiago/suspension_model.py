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

with one constant per DOF (static drift). Linear least squares on unit-RMS
columns and unit-variance rows (each DOF scaled by the standard deviation of
its target); standard errors from a moving-block bootstrap over time.

Baselines: B0 static (predicts the static wrench), B1 the status-quo loader +
S1 (what #70 reported), B2 rigid base: the wrench of an effective point mass
at the gripper cluster moving with the measured arm motion (parameter: the
mass), the competitor that asks whether the suspension explains anything
beyond "the arm moved".

Part 1 reads no wrench to fit anything. :func:`read_target` refuses the
validation run until a study stage sets ``unlock_validation`` (part 2),
refuses the confirmation runs always (hash check only), and
:func:`read_regressors` refuses the confirmation runs too. Docs:
docs/development/tiago-suspension.md.

    python suspension_model.py noise      # freezes the noise of the fixture
    python suspension_model.py describe   # the descriptive reads of the note
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

project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from examples.tiago.utils import suspension_data as sd  # noqa: E402

TIAGO = Path(__file__).resolve().parent
SUS = TIAGO / "data/suspension"
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
    origin={"triad_centroid": "centroid"}[SETTINGS["origin"]],
)


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
def _run(session_id: str) -> sd.SuspensionRun:
    session = load_sessions(verify=False)[session_id]
    return sd.load_run(session.path, **LOADER)


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
    shoulder: np.ndarray
    origin: np.ndarray
    ref: np.ndarray
    stats: dict

    @property
    def n(self) -> int:
        return self.t.size


def read_regressors(session: Session) -> Regressors:
    """Base motion and arm marker clusters; never a plate channel."""
    if session.role == "confirmation":
        raise PermissionError(
            f"{session.id}: confirmation runs are not read (hash check only)"
        )
    return _regressors(session, _run(session.id))


def _regressors(session: Session, r: sd.SuspensionRun) -> Regressors:
    return Regressors(
        session, r.t, r.x, r.v, r.valid, r.interpolated, r.grip, r.shoulder,
        r.origin, r.ref, dict(r.stats),
    )  # fmt: skip


SYNTHETIC = Session("synthetic", "synthetic", Path("."))


def regressors_from_raw(t, values, **overrides) -> Tuple[Regressors, np.ndarray]:
    """``(Regressors, wrench)`` of simulated raw rows through the corrected loader.

    ``overrides`` replace the frozen loader settings (the ablation switches
    ``origin``, ``transport``, ``frame``, ``clock``). Simulated rows only.
    """
    kw = dict(LOADER)
    kw.update(overrides)
    r = sd.process_raw(t, values, **kw)
    return _regressors(SYNTHETIC, r), r.w


def read_target(
    session: Session, unlock_validation: bool = False, unlock_confirmation: bool = False
) -> np.ndarray:
    """(n, 6) plate wrench about the origin, on the rows of :func:`read_regressors`.

    Training only. The validation wrench is read by the study stage of part 2
    (``unlock_validation``); the confirmation wrench by the confirmation stage
    (``unlock_confirmation``), which does not exist in part 1. Diagnostic runs
    are not targets.
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
    if session.role not in ("training", "validation", "confirmation"):
        raise PermissionError(f"{session.id}: {session.role} runs are not targets")
    return _run(session.id).w


# ---------------------------------------------------------------- designs

MODELS = ("S1", "S2", "S3")


def _pair_index() -> List[Tuple[int, int]]:
    return [(a, b) for a in range(6) for b in range(a, 6)]


def parameter_names(model: str) -> List[str]:
    if model in ("S1", "S2"):
        return [f"{p}_{d}" for d in DOFS for p in ("k", "c")]
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


CLUSTERS = {"B2g": ("grip",), "B2gs": ("grip", "shoulder")}


def arm_design(run: Regressors, model: str = "B2g") -> np.ndarray:
    """(n, 6, m) wrench of unit point masses at the marker clusters, rigid base.

    Mass at position ``p`` with acceleration ``a``: force on the plate
    ``m (g - a)`` (action), moment about the base origin ``(p - o) x m (g - a)``,
    ``g = (0, 0, -9.81)``. The static part is absorbed by the offset.
    """
    dt = 1.0 / RATE
    grav = np.array([0.0, 0.0, -G])
    cols = []
    for name in CLUSTERS[model]:
        p = getattr(run, name)
        a = np.gradient(np.gradient(p, dt, axis=0), dt, axis=0)
        f = grav - a
        cols.append(np.column_stack((f, np.cross(p - run.origin, f))))
    return np.stack(cols, axis=2)


def design(model: str, run: Regressors, h_e: float = 0.0) -> np.ndarray:
    return (
        arm_design(run, model)
        if model in CLUSTERS
        else spring_design(model, run.x, run.v, h_e)
    )


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

    @property
    def p(self) -> int:
        return len(self.names)

    @property
    def theta(self) -> np.ndarray:
        return self.params[: self.p]

    def k(self) -> np.ndarray:
        """Diagonal stiffness (S1, S2) or diagonal of K (S3), length 6."""
        if self.model in ("S1", "S2"):
            return self.theta[0::2]
        pairs = _pair_index()
        return np.array([self.theta[pairs.index((d, d))] for d in range(6)])

    def c(self) -> np.ndarray:
        if self.model in ("S1", "S2"):
            return self.theta[1::2]
        pairs = _pair_index()
        return np.array(
            [self.theta[len(pairs) + pairs.index((d, d))] for d in range(6)]
        )

    def c_se(self) -> Optional[np.ndarray]:
        if self.se is None:
            return None
        if self.model in ("S1", "S2"):
            return self.se[1::2]
        pairs = _pair_index()
        return np.array([self.se[len(pairs) + pairs.index((d, d))] for d in range(6)])

    def k_se(self) -> Optional[np.ndarray]:
        if self.se is None:
            return None
        if self.model in ("S1", "S2"):
            return self.se[0::2]
        pairs = _pair_index()
        return np.array([self.se[pairs.index((d, d))] for d in range(6)])


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


def _solve(x3: np.ndarray, y: np.ndarray, weights: np.ndarray, scale=None):
    xw = x3 * weights[None, :, None]
    yw = y * weights[None, :]
    a = xw.reshape(-1, x3.shape[2])
    if scale is None:
        scale = np.sqrt(np.mean(a**2, axis=0))
        scale = np.where(scale > 1e-12 * max(float(scale.max()), 1e-30), scale, np.inf)
    sol = np.linalg.lstsq(a / scale, yw.reshape(-1), rcond=None)[0]
    return sol / scale, scale, a


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
) -> Fit:
    """Fit ``model`` (S1, S2 at ``h_e``, S3, B2g, B2gs) to the target ``w`` (n, 6).

    Rows: ``run.valid`` (or ``rows``). ``weights`` default to one over the
    standard deviation of the target on those rows.
    """
    offset = SETTINGS["offset"] if offset is None else offset
    use = run.valid if rows is None else rows
    x3 = design(model, run, h_e)[use]
    y = np.asarray(w)[use]
    if weights is None:
        weights = 1.0 / np.maximum(y.std(axis=0), 1e-12)
    names = (
        parameter_names(model)
        if model in MODELS
        else [f"m_{c}" for c in CLUSTERS[model]]
    )
    full = _full_design(x3, offset)
    sol, scale, a = _solve(full, y, weights)
    sv = np.linalg.svd(a / scale, compute_uv=False)
    rank = int(np.sum(sv > sv[0] * max(a.shape) * np.finfo(float).eps))
    fit = Fit(model, names, sol, h_e, offset, weights, rank, float(sv[0] / sv[-1]))
    if bootstrap:
        rng = np.random.default_rng(BOOT["seed"])
        block = int(round(BOOT["block_s"] * RATE))
        draws = []
        for _ in range(resamples or BOOT["resamples"]):
            idx = block_indices(full.shape[0], block, rng)
            draws.append(_solve(full[idx], y[idx], weights, scale)[0][: len(names)])
        fit.se = np.std(draws, axis=0, ddof=1)
    return fit


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
    return design(fit.model, run, fit.h_e) @ fit.theta


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


def negative_damping(fit: Fit, n_se: Optional[float] = None) -> np.ndarray:
    """(6,) bool: damping below ``-n_se`` SE (needs a bootstrap)."""
    n_se = RULES["neg_damping_se"] if n_se is None else n_se
    return fit.c() < -n_se * fit.c_se()


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
    run: Regressors,
    w: np.ndarray,
    h_e: float = 0.0,
    offset: Optional[bool] = None,
) -> float:
    """Blocked CV score: pooled held-out RMSE over the target sd, mean over DOF."""
    folds = cv_folds(run)
    x3 = design(model, run, h_e)[run.valid]
    y = np.asarray(w)[run.valid]
    weights = 1.0 / np.maximum(y.std(axis=0), 1e-12)
    offset = SETTINGS["offset"] if offset is None else offset
    full = _full_design(x3, offset)
    p = x3.shape[2]
    sse = np.zeros(6)
    for f in np.unique(folds):
        tr, te = folds != f, folds == f
        sol = _solve(full[tr], y[tr], weights)[0]
        res = y[te] - full[te] @ sol
        res = res - res.mean(axis=0)  # a constant is not a suspension quantity
        sse += np.sum(res**2, axis=0)
    return float(np.mean(np.sqrt(sse / y.shape[0]) / y.std(axis=0)))


def select_model(run: Regressors, w: np.ndarray) -> dict:
    """The simplest of S1 < S2 < S3 within ``tolerance`` of the best CV score.

    S2 takes the ``h_e`` of the grid with the lowest CV score. Training data
    only: the validation run is never an argument here.
    """
    tol = SETTINGS["selection"]["tolerance"]
    s1 = cv_score("S1", run, w)
    grid = h_e_grid()
    s2_all = np.array([cv_score("S2", run, w, h) for h in grid])
    i = int(np.argmin(s2_all))
    s3 = cv_score("S3", run, w)
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


# --------------------------------------------------------------- verdicts


def verdict(
    r2_model: np.ndarray,
    r2_b2: np.ndarray,
    neg_damping: np.ndarray,
    repeatable: np.ndarray,
    nonlinear: np.ndarray,
    answerable: Optional[np.ndarray] = None,
) -> dict:
    """The "spring-damper is enough" rule (note, section 8), per DOF arrays of 6."""
    answerable = (
        np.ones(6, bool) if answerable is None else np.asarray(answerable, bool)
    )
    explained = r2_model >= RULES["explained_r2"]
    beats = (r2_model - r2_b2) >= RULES["beat_b2"]
    ok = explained & beats & ~neg_damping & repeatable & ~nonlinear
    return {
        "explained": explained.tolist(),
        "beats_b2": beats.tolist(),
        "dof_ok": ok.tolist(),
        "enough": bool(answerable.any() and np.all(ok[answerable])),
    }


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
    """White noise levels from the rest run, and the same pushed through the loader."""
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
    trans, rot = sd.triad_pose(mk, ref)
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


# ----------------------------------------------------------- descriptives


def describe() -> dict:
    """The descriptive reads of the note (no suspension fit, no validation target)."""
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
        }
        if s.role == "training" or sid == SETTINGS["sessions"]["noise_floor"]:
            centre, res = sd.plate_centre_fit(v[:, 30:33], v[:, 33:36], v[:, 36:39])
            out[sid]["plate_centre"] = centre.tolist()
            out[sid]["plate_centre_residual_rms"] = res
    return out


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("stage", choices=("noise", "describe"))
    args = parser.parse_args(argv)
    res = noise_stage() if args.stage == "noise" else describe()
    print(json.dumps(res, indent=1, default=float))


if __name__ == "__main__":
    main()
