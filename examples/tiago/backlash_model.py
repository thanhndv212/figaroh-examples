"""TIAGo backlash surface (#71): the study's library and the noise stage.

Question: how much of the transmission error of a TIAGo arm joint is backlash
(direction dependent), how much is the encoder's own angle error, and does the
backlash depend on load? The measured quantity is ``d = q_rel - q_abs``, the
motor-side encoder (over the gear ratio) minus the output absolute encoder,
modelled per joint as

    d = c + E(q_abs) + (W/2) h + G tanh(tau_g / tau0) + kappa tau_g + e

``c`` one constant per session and per encoder-jump segment, ``E`` the
direction-free angle error (piecewise linear in ``q_abs``, frozen from
training), ``h`` the motion-direction state, ``tau_g`` the pinocchio gravity
torque of the joint at the logged configuration. The backlash surface is the
direction-dependent part, ``W`` (the width) and the load terms ``G`` (preload)
and ``kappa`` (compliance). Linear least squares on unit-norm columns; the
standard errors are block-bootstrap ones (the 100 Hz residual is
autocorrelated). Docs: docs/development/tiago-backlash.md.

The regressors (positions, velocities, the configuration and its gravity
torque) are read from any session. The target ``q_abs`` is read through
:func:`read_target`, which refuses the validation run until a study stage sets
``unlock_validation`` (part 2 of the study).

Stage ``noise`` reads the training target (disclosed in the note) and freezes
the noise of the fixture:

    python backlash_model.py noise
"""

from __future__ import annotations

import argparse
import csv
import sys
from dataclasses import dataclass, field
from functools import lru_cache
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import yaml
from figaroh.data.protocol import Protocol
from scipy.ndimage import uniform_filter1d, median_filter

project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

TIAGO = Path(__file__).resolve().parent
BL = TIAGO / "data/backlash"
PROTOCOL = BL / "protocol.yaml"
CONFIG = TIAGO / "config/tiago_backlash_study_config.yaml"
URDF_DIR = TIAGO / "urdf"


def load_settings(path=CONFIG) -> dict:
    return yaml.safe_load(Path(path).read_text())


SETTINGS = load_settings()
JOINTS: List[str] = list(SETTINGS["joints"])
CONFIG_JOINTS: List[str] = list(SETTINGS["configuration_joints"])
LEVELS = tuple(SETTINGS["noise"]["levels"])
TAU0 = float(SETTINGS["surface"]["tau0"])


# --------------------------------------------------------------- sessions


@dataclass
class Session:
    id: str
    role: str
    robot: str
    names: Optional[Path]
    values: Optional[Path]


def load_sessions(verify: bool = True) -> Dict[str, Session]:
    """``{id: Session}`` of the protocol (sha256-verified)."""
    protocol = Protocol.load(PROTOCOL)
    if verify:
        protocol.verify(root=PROTOCOL.parent)
    out = {}
    for s in protocol.sessions:
        names = [BL / n for n in s.files if n.endswith("introspection_datanames.csv")]
        values = [BL / n for n in s.files if n.endswith("introspection_datavalues.csv")]
        out[s.id] = Session(
            s.id,
            s.role,
            getattr(s, "robot", None),
            names[0] if names else None,
            values[0] if values else None,
        )
    return out


def role_sets(sessions: Optional[dict] = None) -> Dict[str, List[Session]]:
    """Sessions by role (training, validation, diagnostic).

    Built here instead of ``calibration_harness.parse_heldout``, which has no
    ``diagnostic`` role. The diagnostic top run holds only ``tf.csv`` (no joint
    values) and is not used (D6). There is no confirmation set (D2).
    """
    sessions = sessions or load_sessions()
    out: Dict[str, List[Session]] = {"training": [], "validation": [], "diagnostic": []}
    for s in sessions.values():
        out[s.role].append(s)
    if len(out["training"]) != 1 or len(out["validation"]) != 1:
        raise ValueError("expected one training and one validation session")
    return out


# ------------------------------------------------------------- data access


def _signal_names(session: Session) -> List[str]:
    with session.names.open(newline="") as f:
        header = next(csv.reader(f))
    return [n.strip().removeprefix("- ") for n in header[header.index("names") + 1 :]]


def _read_columns(
    session: Session, columns: Sequence[str]
) -> Tuple[np.ndarray, np.ndarray]:
    """``(t, values)`` of the named columns; the header clock, finite increasing rows."""
    names = _signal_names(session)
    missing = [c for c in columns if c not in names]
    if missing:
        raise ValueError(f"{session.id}: missing channels {missing}")
    # PAL rows carry one leading value with no entry in the names header.
    idx = [names.index(c) + 1 for c in columns]
    t, rows = [], []
    with session.values.open(newline="") as f:
        for row in csv.DictReader(f):
            v = np.fromstring(row["values"].strip().strip("[]"), sep=",")
            if v.size != len(names):
                continue
            ts = float(row["secs"]) + float(row["nsecs"]) * 1e-9
            sel = v[idx]
            if not np.isfinite(ts) or not np.all(np.isfinite(sel)):
                continue
            if t and ts <= t[-1]:
                continue
            t.append(ts)
            rows.append(sel)
    return np.asarray(t), np.asarray(rows)


@dataclass
class Run:
    """Regressor columns of one session: time, positions and velocities of the
    torso and the seven arm joints (``CONFIG_JOINTS`` order)."""

    session: Session
    t: np.ndarray
    q: np.ndarray
    v: np.ndarray

    @property
    def n(self) -> int:
        return self.t.size


@lru_cache(maxsize=None)
def _read_run(session_id: str) -> Run:
    session = load_sessions(verify=False)[session_id]
    cols = [f"{j}_position" for j in CONFIG_JOINTS] + [
        f"{j}_velocity" for j in CONFIG_JOINTS
    ]
    t, x = _read_columns(session, cols)
    k = len(CONFIG_JOINTS)
    return Run(session, t, x[:, :k], x[:, k:])


def read_regressors(session: Session) -> Run:
    """Positions and velocities only: never an absolute-encoder column."""
    if session.values is None:
        raise ValueError(f"{session.id}: no joint values (pose-only session)")
    return _read_run(session.id)


def read_targets(
    session: Session, joints: Sequence[str], unlock_validation: bool = False
) -> Dict[str, np.ndarray]:
    """``q_abs`` of the joints on the rows of :func:`read_regressors`.

    Refuses the validation run unless ``unlock_validation`` (set by the study
    script of part 2 only) and every session that is not training or
    validation. The training target is the only one read in part 1 (noise
    stage).
    """
    if session.role == "validation" and not unlock_validation:
        raise PermissionError(
            f"{session.id}: the validation absolute encoder is read only by the "
            "study stage (unlock_validation)"
        )
    if session.role not in ("training", "validation"):
        raise PermissionError(f"{session.id}: {session.role} sessions are not read")
    run = read_regressors(session)
    cols = [f"{j}_position" for j in CONFIG_JOINTS] + [
        f"{j}_velocity" for j in CONFIG_JOINTS
    ]
    cols += [f"{j}_absolute_encoder_position" for j in joints]
    t, x = _read_columns(session, cols)
    if t.size != run.n or not np.array_equal(t, run.t):
        raise ValueError(f"{session.id}: target rows differ from the regressor rows")
    k = 2 * len(CONFIG_JOINTS)
    return {j: x[:, k + i] for i, j in enumerate(joints)}


def read_target(
    session: Session, joint: str, unlock_validation: bool = False
) -> np.ndarray:
    """``q_abs`` of one joint (see :func:`read_targets`)."""
    return read_targets(session, [joint], unlock_validation)[joint]


# --------------------------------------------------------------- gravity


@lru_cache(maxsize=None)
def _model(urdf: str):
    from figaroh.tools.robot import load_robot

    return load_robot(str(URDF_DIR / urdf), load_by_urdf=True).model


def gravity_torque(run: Run, urdf: str) -> np.ndarray:
    """(n, 7) pinocchio generalized gravity torque of the arm joints.

    Configuration from the logged torso and arm positions (``q_rel``; the
    head and the hand at the URDF neutral pose, which do not load the arm
    joints). ``urdf`` is ``tiago_48_schunk.urdf`` for the backlash days.
    """
    return _gravity(run.session.id, urdf)


@lru_cache(maxsize=None)
def _gravity(session_id: str, urdf: str) -> np.ndarray:
    import pinocchio as pin

    run = _read_run(session_id)
    m = _model(urdf)
    data = m.createData()
    q0 = pin.neutral(m)
    iq = [m.joints[m.getJointId(j)].idx_q for j in CONFIG_JOINTS]
    iv = [m.joints[m.getJointId(j)].idx_v for j in JOINTS]
    out = np.empty((run.n, len(JOINTS)))
    q = q0.copy()
    for i in range(run.n):
        q[iq] = run.q[i]
        out[i] = pin.computeGeneralizedGravity(m, data, q)[iv]
    return out


# ------------------------------------------------- direction, jumps, spline


def direction_state(
    t: np.ndarray, v: np.ndarray, cfg=None
) -> Tuple[np.ndarray, np.ndarray]:
    """``(h, valid)``: the direction in {-1, 0, +1} and the usable samples.

    ``h`` is the sign of the smoothed velocity, held through ``|v| < v_min``
    (0 before the first movement). Only moving samples (``|v| >= v_min``) are
    valid: a held ``h`` on a dwell can sit on the wrong flank after a slow turn.
    Samples from the end of the old motion to ``mask_s`` after a reversal
    (take-up transient, including the slow stretch where the play is crossed) and those with ``h = 0`` are not valid.
    """
    cfg = cfg or SETTINGS["direction"]
    dt = float(np.median(np.diff(t)))
    k = max(1, int(round(cfg["smooth_s"] / dt)))
    vs = uniform_filter1d(np.asarray(v, float), k, mode="nearest")
    moving = np.abs(vs) >= cfg["v_min"]
    idx = np.where(moving, np.arange(vs.size), -1)
    idx = np.maximum.accumulate(idx)
    h = np.where(idx >= 0, np.sign(vs[np.maximum(idx, 0)]), 0.0)
    valid = (h != 0) & moving
    rev = np.flatnonzero((h[1:] != h[:-1]) & (h[1:] != 0) & (h[:-1] != 0)) + 1
    for i in rev:
        # from the end of the old motion (the last moving sample before the
        # slow stretch the reversal happens in) to mask_s after the new one
        valid &= ~((t >= t[idx[i - 1]]) & (t < t[i] + cfg["mask_s"]))
    return h, valid


def reversal_count(h: np.ndarray) -> int:
    nz = h[h != 0]
    return int(np.sum(nz[1:] != nz[:-1]))


def jump_indices(d: np.ndarray, q_rel: np.ndarray, cfg=None) -> np.ndarray:
    """Indices ``i`` where ``d`` jumps between samples ``i-1`` and ``i`` with
    ``q_rel`` still: ``|delta d| > delta_d`` and ``|delta q_rel| < delta_q``."""
    cfg = cfg or SETTINGS["jumps"]
    dd, dq = np.abs(np.diff(d)), np.abs(np.diff(q_rel))
    return np.flatnonzero((dd > cfg["delta_d"]) & (dq < cfg["delta_q"])) + 1


def segment_ids(n: int, jumps: np.ndarray) -> np.ndarray:
    seg = np.zeros(n, dtype=int)
    for j in jumps:
        seg[j:] += 1
    return seg


def make_knots(q_abs: np.ndarray, spacing: float = None) -> np.ndarray:
    """Knots every ``spacing`` over the visited range of ``q_abs``."""
    dx = spacing or SETTINGS["surface"]["knot_spacing"]
    lo, hi = np.floor(q_abs.min() / dx) * dx, np.ceil(q_abs.max() / dx) * dx
    return lo + dx * np.arange(int(round((hi - lo) / dx)) + 1)


def hat_basis(x: np.ndarray, knots: np.ndarray) -> np.ndarray:
    """(n, K) linear-interpolation weights; clamped outside the knot range."""
    k = knots.size
    xc = np.clip(x, knots[0], knots[-1])
    i = np.clip(np.searchsorted(knots, xc, side="right") - 1, 0, k - 2)
    w = (xc - knots[i]) / (knots[i + 1] - knots[i])
    b = np.zeros((x.size, k))
    r = np.arange(x.size)
    b[r, i] = 1.0 - w
    b[r, i + 1] = w
    return b


def soft_sign(tau: np.ndarray, tau0: float = TAU0) -> np.ndarray:
    return np.tanh(tau / tau0)


# ------------------------------------------------------------------ fit


@dataclass
class JointData:
    """One joint of one session on the usable samples (all arrays aligned)."""

    joint: str
    t: np.ndarray
    q_rel: np.ndarray
    q_abs: np.ndarray
    h: np.ndarray
    tau: np.ndarray
    seg: np.ndarray
    valid: np.ndarray

    @property
    def d(self) -> np.ndarray:
        return self.q_rel - self.q_abs

    def rows(self) -> "JointData":
        v = self.valid
        return JointData(
            self.joint,
            self.t[v],
            self.q_rel[v],
            self.q_abs[v],
            self.h[v],
            self.tau[v],
            self.seg[v],
            np.ones(int(v.sum()), bool),
        )


GROUPS = ("E", "W", "L")


def _terms(groups: Sequence[str]) -> Tuple[str, ...]:
    return tuple(g for g in GROUPS if g in groups)


def load_usable(jd: JointData) -> Tuple[bool, bool]:
    """Which load columns carry information on the usable samples: the soft
    sign (not saturated: its sd exceeds ``min_sg_sd``) and the torque itself
    (``sd >= min_gravity_sd``; zero for a joint with a vertical axis)."""
    f1 = SETTINGS["fixture"]["f1"]
    v = jd.valid
    return (
        bool(np.std(soft_sign(jd.tau[v])) > f1["min_sg_sd"]),
        bool(np.std(jd.tau[v]) >= f1["min_gravity_sd"]),
    )


def design(
    jd: JointData,
    knots: np.ndarray,
    groups: Sequence[str],
    n_seg: int = None,
    use: Tuple[bool, bool] = (True, True),
):
    """``(X, index)``: columns ``c`` (one per segment), ``E`` (hats, first
    dropped: the gauge is the offset), ``W`` (h/2), ``L`` (G, kappa; an
    unusable column is zero and dropped by the solver)."""
    n_seg = n_seg if n_seg is not None else int(jd.seg.max()) + 1
    cols, index = [(jd.seg[:, None] == np.arange(n_seg)).astype(float)], {
        "c": range(n_seg)
    }
    p = n_seg
    if "E" in groups:
        hb = hat_basis(jd.q_abs, knots)[:, 1:]
        cols.append(hb)
        index["E"] = range(p, p + hb.shape[1])
        p += hb.shape[1]
    if "W" in groups:
        cols.append((0.5 * jd.h)[:, None])
        index["W"] = range(p, p + 1)
        p += 1
    if "L" in groups:
        cols.append(np.column_stack([soft_sign(jd.tau) * use[0], jd.tau * use[1]]))
        index["L"] = range(p, p + 2)
        p += 2
    return np.hstack(cols), {k: list(v) for k, v in index.items()}


def _scaled(X: np.ndarray):
    s = np.sqrt(np.mean(X**2, axis=0))
    keep = s > 1e-9 * max(float(s.max()), 1e-30)
    s = np.where(keep, s, 1.0)
    return X / s, s, keep


def e_penalty(index: Dict[str, List[int]], knots: np.ndarray, p: int) -> np.ndarray:
    """Second-difference rows on the ``E`` coefficients, weight ``sqrt(smooth)``.

    Where no sample constrains a knot (an angle visited only in a held-out
    fold, or beyond the training range) the unpenalized minimum-norm solution
    sets ``E`` to zero there; the penalty interpolates it linearly instead.
    Its weight is that of ``smooth`` samples, negligible where data exist."""
    k = knots.size
    if "E" not in index or k < 3:
        return np.zeros((0, p))
    d2 = np.zeros((k - 2, k))
    for i in range(k - 2):
        d2[i, i : i + 3] = (1.0, -2.0, 1.0)
    pen = np.zeros((k - 2, p))
    pen[:, index["E"]] = d2[:, 1:]  # first hat dropped (the gauge)
    return np.sqrt(SETTINGS["surface"]["smooth"]) * pen


def solve(X: np.ndarray, y: np.ndarray, pen: Optional[np.ndarray] = None):
    """Unit-norm columns, minimum-norm least squares; ``(coef, scale, keep, rank, cond)``.
    ``pen`` rows (penalty on the coefficients) are appended with zero target."""
    Xs, s, keep = _scaled(X)
    if pen is not None and pen.size:
        keep = keep | (np.abs(pen).sum(0) > 0)  # a penalized column is determined by it
        Xs = np.vstack([Xs, pen / s])
        y = np.concatenate([y, np.zeros(pen.shape[0])])
    Xk = Xs[:, keep]
    coef_k, _, rank, sv = np.linalg.lstsq(Xk, y, rcond=1e-10)
    coef = np.zeros(X.shape[1])
    coef[keep] = coef_k / s[keep]
    cond = float(sv[0] / sv[-1]) if sv[-1] > 0 else float("inf")
    return coef, s, keep, int(rank), cond


@dataclass
class Fit:
    """A fitted model: coefficients in physical units."""

    joint: str
    groups: Tuple[str, ...]
    knots: np.ndarray
    coef: np.ndarray
    index: Dict[str, List[int]]
    keep: np.ndarray
    rank: int
    cond: float
    n: int
    rmse: float
    se: Optional[np.ndarray] = None  # block-bootstrap SE of coef
    load_use: Tuple[bool, bool] = (True, True)

    def value(self, key: str):
        return self.coef[self.index[key]]

    @property
    def W(self) -> float:
        return float(self.value("W")[0]) if "W" in self.groups else 0.0

    @property
    def G(self) -> float:
        return float(self.value("L")[0]) if "L" in self.groups else 0.0

    @property
    def kappa(self) -> float:
        return float(self.value("L")[1]) if "L" in self.groups else 0.0

    def z(self, key: str, i: int = 0) -> float:
        j = self.index[key][i]
        return float(abs(self.coef[j]) / self.se[j]) if self.se[j] > 0 else 0.0

    def terms(self, jd: JointData) -> np.ndarray:
        """Prediction without the offset (``E``, ``W``, ``L`` terms)."""
        X, _ = design(jd, self.knots, self.groups, n_seg=1, use=self.load_use)
        c = [j for g in self.groups for j in self.index[g]]
        return X[:, 1:] @ self.coef[c]


def fit_groups(
    jd: JointData,
    knots: np.ndarray,
    groups: Sequence[str],
    block_s: float = None,
    bootstrap: bool = False,
    rng=None,
) -> Fit:
    """Least squares of ``d`` on the usable samples; optional block bootstrap SE."""
    groups = _terms(groups)
    r = jd.rows()
    use = load_usable(jd)
    X, index = design(r, knots, groups, use=use)
    y = r.d
    pen = e_penalty(index, knots, X.shape[1])
    coef, s, keep, rank, cond = solve(X, y, pen)
    res = y - X @ coef
    fit = Fit(
        jd.joint,
        groups,
        knots,
        coef,
        index,
        keep,
        rank,
        cond,
        y.size,
        float(np.sqrt(np.mean(res**2))),
        load_use=use,
    )
    if bootstrap:
        fit.se = bootstrap_se(r, X, y, keep, s, block_s, rng, pen)
    return fit


def bootstrap_se(r, X, y, keep, s, block_s=None, rng=None, pen=None) -> np.ndarray:
    """Block-bootstrap SE (pairs of rows resampled in ``block_s`` blocks)."""
    b = SETTINGS["selection"]["bootstrap"]
    block_s = block_s or b["block_s"]
    rng = rng or np.random.default_rng(b["seed"])
    blk = np.floor((r.t - r.t[0]) / block_s).astype(int)
    nb = blk.max() + 1
    Xs = (X / s)[:, keep]
    G = np.zeros((nb, Xs.shape[1], Xs.shape[1]))
    c = np.zeros((nb, Xs.shape[1]))
    for k in range(nb):
        m = blk == k
        if m.any():
            G[k] = Xs[m].T @ Xs[m]
            c[k] = Xs[m].T @ y[m]
    P = np.zeros((Xs.shape[1], Xs.shape[1]))
    if pen is not None and pen.size:
        ps = (pen / s)[:, keep]
        P = ps.T @ ps
    out = np.empty((b["resamples"], X.shape[1]))
    for i in range(b["resamples"]):
        pick = rng.integers(0, nb, nb)
        sol = np.linalg.lstsq(G[pick].sum(0) + P, c[pick].sum(0), rcond=1e-10)[0]
        full = np.zeros(X.shape[1])
        full[keep] = sol / s[keep]
        out[i] = full
    return out.std(axis=0, ddof=1)


def predict(fit: Fit, jd: JointData, reanchor: bool = True) -> np.ndarray:
    """``d`` predicted on all samples of ``jd``. The offset is re-anchored on
    ``jd``'s own usable samples, one constant per segment (the c_s protocol)."""
    p = fit.terms(jd)
    if not reanchor:
        return p
    out = p.copy()
    d = jd.d
    for k in np.unique(jd.seg):
        m = (jd.seg == k) & jd.valid
        out[jd.seg == k] += float(np.mean(d[m] - p[m])) if m.any() else 0.0
    return out


def rmse(fit: Fit, jd: JointData) -> float:
    pred = predict(fit, jd)
    return float(np.sqrt(np.mean((jd.d - pred)[jd.valid] ** 2)))


def cv_rmse(jd: JointData, knots: np.ndarray, groups: Sequence[str]) -> float:
    """Blocked K-fold CV RMSE: folds of whole ``cv_block_s`` periods, the held-out
    fold re-anchored per segment (as validation will be)."""
    cfg = SETTINGS["selection"]
    r = jd.rows()
    period = np.floor((r.t - r.t[0]) / cfg["cv_block_s"]).astype(int)
    fold = period * cfg["folds"] // (period.max() + 1)
    sse, n = 0.0, 0
    for k in range(cfg["folds"]):
        te = fold == k
        tr = ~te
        if not te.any():
            continue
        sub = _subset(r, tr)
        f = fit_groups(
            JointData(
                sub.joint,
                sub.t,
                sub.q_rel,
                sub.q_abs,
                sub.h,
                sub.tau,
                sub.seg,
                np.ones(sub.t.size, bool),
            ),
            knots,
            groups,
        )
        held = _subset(r, te)
        pred = predict(f, held)
        sse += float(np.sum((held.d - pred) ** 2))
        n += held.t.size
    return float(np.sqrt(sse / n))


def _subset(jd: JointData, m: np.ndarray) -> JointData:
    return JointData(
        jd.joint,
        jd.t[m],
        jd.q_rel[m],
        jd.q_abs[m],
        jd.h[m],
        jd.tau[m],
        jd.seg[m],
        jd.valid[m],
    )


def select_groups(jd: JointData, knots: np.ndarray, rng=None) -> dict:
    """The frozen selection rule (training only). ``E`` is structural (the
    encoder angle error, D1): always in the model, never selected, because a
    blocked fold cannot validate an angle that only one fold visits. ``W``,
    then ``L``, each enter if blocked CV RMSE drops by more than ``cv_gain_min``
    and a coefficient has ``|z| > z_min``. Returns the groups, the CV RMSEs and
    the z values."""
    cfg = SETTINGS["selection"]
    chosen: List[str] = ["E"]
    base = cv_rmse(jd, knots, chosen)
    cv = {"E": base}
    z: Dict[str, float] = {}
    for g in ("W", "L"):
        if g == "L" and gravity_degenerate(jd):
            z["L"] = 0.0
            continue
        trial = chosen + [g]
        cv[g] = cv_rmse(jd, knots, trial)
        f = fit_groups(jd, knots, trial, bootstrap=True, rng=rng)
        z[g] = max(f.z(g, i) for i in range(len(f.index[g])))
        if 1.0 - cv[g] / base > cfg["cv_gain_min"] and z[g] > cfg["z_min"]:
            chosen.append(g)
            base = cv[g]
    return {"groups": tuple(chosen), "cv": cv, "z": z}


def gravity_degenerate(jd: JointData) -> bool:
    """No usable load column: the load group cannot be fitted."""
    return not any(load_usable(jd))


# ----------------------------------------------------- baselines (part 2)


def fit_b0(jd: JointData) -> Fit:
    """B0: offset only."""
    return fit_groups(jd, np.array([0.0, 1.0]), ())


def fit_b1(jd: JointData) -> Fit:
    """B1: offset and a direction-only constant, ``c + (W/2) h`` (#71's comparator)."""
    return fit_groups(jd, np.array([0.0, 1.0]), ("W",))


def fit_b2(jd: JointData, degree: int = 5):
    """B2: the existing degree-``degree`` bivariate surface
    (``EmpiricalBacklashSurface``, unchanged), offset re-anchored on the
    validation samples by :func:`predict_b2`."""
    from examples.tiago.utils.backlash_surface import EmpiricalBacklashSurface

    r = jd.rows()
    surf = EmpiricalBacklashSurface(degree=degree)
    surf.fit(r.q_rel, r.tau, np.gradient(r.q_rel, r.t), r.d)
    return surf


def predict_b2(surf, jd: JointData) -> np.ndarray:
    v = np.gradient(jd.q_rel, jd.t)
    pred = surf.predict(jd.q_rel, jd.tau, v, allow_extrapolation=True)
    m = jd.valid
    return pred + float(np.mean((jd.d - pred)[m]))


# ------------------------------------------------------------ data assembly


def joint_data(
    run: Run, joint: str, q_abs: np.ndarray, urdf: str, jumps: bool = True
) -> JointData:
    """``JointData`` of one joint from a run and its (real or simulated) ``q_abs``."""
    j = CONFIG_JOINTS.index(joint)
    h, valid = direction_state(run.t, run.v[:, j])
    d = run.q[:, j] - q_abs
    seg = segment_ids(
        run.n, jump_indices(d, run.q[:, j]) if jumps else np.array([], int)
    )
    tau = gravity_torque(run, urdf)[:, JOINTS.index(joint)]
    return JointData(joint, run.t, run.q[:, j], q_abs, h, tau, seg, valid)


# ------------------------------------------------------------ identifiability


def identifiability(run: Run, joint: str, urdf: str) -> dict:
    """Design conditioning from regressor columns only (no ``q_abs``).

    ``q_rel`` stands in for ``q_abs`` as the argument of ``E``: the two differ
    by the encoder error, so this is the conditioning of the design at the
    visited angles, not of the final fit. Columns: one offset, ``E`` hats,
    ``W``, ``G``, ``kappa`` (the full ``M3``).
    """
    j = CONFIG_JOINTS.index(joint)
    h, valid = direction_state(run.t, run.v[:, j])
    tau = gravity_torque(run, urdf)[:, JOINTS.index(joint)]
    q = run.q[:, j]
    jd = JointData(joint, run.t, q, q, h, tau, np.zeros(run.n, int), valid).rows()
    knots = make_knots(jd.q_abs)
    sg = soft_sign(jd.tau)
    out = {
        "joint": joint,
        "tau_sd": float(np.std(jd.tau)),
        "reversals": reversal_count(h),
        "corr_h_sg": _corr(jd.h, sg),
        "corr_h_tau": _corr(jd.h, jd.tau),
    }
    use = load_usable(jd)
    degenerate = not any(use)
    groups = ("E", "W") if degenerate else ("E", "W", "L")
    X, idx = design(jd, knots, groups, use=use)
    Xs, _, keep = _scaled(X)
    sv = np.linalg.svd(Xs[:, keep], compute_uv=False)
    out.update(
        columns=int(keep.sum()),
        rank=int(np.sum(sv > sv[0] * 1e-10)),
        cond=float(sv[0] / sv[-1]),
        degenerate_load=bool(degenerate),
        load_use=[bool(u) for u in use],
        overlap=direction_overlap(jd),
    )
    # load against the angle error: how much of the load columns the E hats explain
    if not degenerate:
        E = hat_basis(jd.q_abs, knots)
        cols = [c for c, u in zip((sg, jd.tau), use) if u]
        L = np.column_stack(cols)
        EH = np.column_stack([E, jd.h])
        out["load_explained_by_E"] = _explained(L, E)
        out["load_explained_by_E_h"] = _explained(L, EH)
    return out


def _explained(L: np.ndarray, B: np.ndarray) -> List[float]:
    resid = L - B @ np.linalg.lstsq(B, L, rcond=None)[0]
    return [
        float(1.0 - np.sum(resid[:, k] ** 2) / np.sum((L[:, k] - L[:, k].mean()) ** 2))
        for k in range(L.shape[1])
    ]


def direction_overlap(
    jd: JointData, bin_width: float = None, min_s: float = 0.5
) -> float:
    """Fraction of the visited angle bins that have at least ``min_s`` of usable
    samples in *both* directions: where the direction-free ``E`` and the
    direction-dependent ``W`` can be told apart."""
    bw = bin_width or SETTINGS["surface"]["knot_spacing"]
    dt = float(np.median(np.diff(jd.t)))
    b = np.floor(jd.q_abs / bw).astype(int)
    need = max(1, int(round(min_s / dt)))
    visited = np.unique(b)
    both = 0
    for k in visited:
        m = b == k
        both += int((jd.h[m] > 0).sum() >= need and (jd.h[m] < 0).sum() >= need)
    return both / len(visited)


def _corr(a, b) -> float:
    if np.std(a) == 0 or np.std(b) == 0:
        return 0.0
    return float(np.corrcoef(a, b)[0, 1])


def gravity_sign_fraction(run: Run, joint: str) -> float:
    """Rule N5: fraction of samples (``|tau_g| > min`` under Schunk) where the Schunk
    and Hey5 gravity torques have opposite sign."""
    i = JOINTS.index(joint)
    a = gravity_torque(run, SETTINGS["urdf"]["backlash_days"])[:, i]
    b = gravity_torque(run, SETTINGS["urdf"]["sensitivity"])[:, i]
    m = np.abs(a) > SETTINGS["fixture"]["f1"]["min_gravity_sd"]
    return float(np.mean(np.sign(a[m]) != np.sign(b[m]))) if m.any() else 0.0


# --------------------------------------------------------------- noise stage


def noise_estimates() -> dict:
    """Noise of ``d`` from the *training* target: ``sigma_d`` per joint (second
    differences on moving samples, jump neighbourhoods excluded), AR(1) ``rho``
    (residual of a 1 s moving median) and the quantization step of ``q_abs``.
    Reads no other target (:func:`read_target` refuses validation)."""
    sessions = load_sessions()
    train = role_sets(sessions)["training"][0]
    run = read_regressors(train)
    dt = float(np.median(np.diff(run.t)))
    win = int(round(1.0 / dt)) | 1
    out = {
        "sigma_d_mrad": {},
        "rho": {},
        "quantization_mrad": {},
        "jumps": {},
        "dt": dt,
    }
    targets = read_targets(train, JOINTS)
    for joint in JOINTS:
        j = CONFIG_JOINTS.index(joint)
        q_abs = targets[joint]
        d = run.q[:, j] - q_abs
        jumps = jump_indices(d, run.q[:, j])
        out["jumps"][joint] = int(jumps.size)
        h, valid = direction_state(run.t, run.v[:, j])
        moving = np.abs(run.v[:, j]) > SETTINGS["direction"]["v_min"]
        d2 = d[2:] - 2 * d[1:-1] + d[:-2]
        ok = (
            moving[1:-1]
            & moving[:-2]
            & moving[2:]
            & (np.abs(np.diff(d))[1:] < SETTINGS["jumps"]["delta_d"])
            & (np.abs(np.diff(d))[:-1] < SETTINGS["jumps"]["delta_d"])
        )
        out["sigma_d_mrad"][joint] = float(np.sqrt(np.mean(d2[ok] ** 2) / 6.0) * 1e3)
        res = d - median_filter(d, size=win, mode="nearest")
        res = res[valid]
        out["rho"][joint] = float(np.corrcoef(res[1:], res[:-1])[0, 1])
        steps = np.abs(np.diff(q_abs))
        steps = steps[steps > 0]
        out["quantization_mrad"][joint] = float(steps.min() * 1e3)
    return out


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "stage", nargs="?", choices=["noise", "identifiability"], default="noise"
    )
    parser.add_argument(
        "--no-plot", action="store_true", help="accepted for validate.py"
    )
    args = parser.parse_args(argv)
    sessions = load_sessions()
    roles = role_sets(sessions)
    if args.stage == "noise":
        est = noise_estimates()
        print("noise of d = q_rel - q_abs, training 2023-07-24 (target read)")
        for j in JOINTS:
            print(
                f"  {j:12s} sigma_d {est['sigma_d_mrad'][j]:7.3f} mrad  rho {est['rho'][j]:.3f}"
                f"  q_abs step {est['quantization_mrad'][j]:.4f} mrad  jumps {est['jumps'][j]}"
            )
    else:
        urdf = SETTINGS["urdf"]["backlash_days"]
        for role in ("training", "validation"):
            run = read_regressors(roles[role][0])
            print(f"{role} {run.session.id}")
            for j in JOINTS:
                r = identifiability(run, j, urdf)
                print(
                    f"  {j:12s} cond {r['cond']:10.3g} rank {r['rank']}/{r['columns']} "
                    f"corr(h,sg) {r['corr_h_sg']:+.3f} tau_sd {r['tau_sd']:.3f} "
                    f"reversals {r['reversals']} overlap {r['overlap']:.2f} N5 {gravity_sign_fraction(run, j):.3f}"
                    + (
                        f" load|E {r['load_explained_by_E']}"
                        if "load_explained_by_E" in r
                        else ""
                    )
                )


if __name__ == "__main__":
    main()
