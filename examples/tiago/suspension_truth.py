"""TIAGo base suspension (#70): what the loader, the models and the noise alone explain.

A simulated fixture. One truth per seed (diagonal stiffness and damping drawn
around plausible TIAGo values) is measured on the *real* base motion of the
training (xyz_1640) and validation (xyz_1628) runs: the corrected pose of the
base triad and the arm marker columns of the two runs are taken as the motion;
the plate columns of the real runs are never used. The diagnostic runs of
``sessions.selection_diagnostics`` are replayed the same way for the selection
(F4a, F5; maintainer decision O3). From that motion and a
truth the fixture writes the raw rows a Vicon + force-plate log would hold
(three base markers, the plate wrench about the plate centre with a static
-730 N load, white noise at the levels of the noise stage, a receive clock with
the measured drop rate and latency), and passes them through the same loader
(``utils/suspension_data.py``) and fit (``suspension_model.py``) the real
study uses. Criteria are fixed in ``config/tiago_suspension_study_config.yaml``
before any run.

- F1 identifiability (real motion): scaled condition numbers of S1, S2
  (worst over the h_e grid) and S3; per-column signal-to-noise (reported).
- F2 recovery: S1 fit on the corrected loader path recovers an S1 truth (z of
  the 12 parameters from the block bootstrap, held-out R2 against the noise
  ceiling, exact at level 0); F2-bias (revision): the mean relative error over
  seeds at the used level, and an errors-in-variables corrected S1 as a
  sensitivity row.
- F3 status-quo signature: the status-quo loader + S1 on the same truth
  reproduces "forces R2 below 0.2, My R2 high, negative dampings"; the four
  loader errors are put back one at a time (ablation) and the full fix is
  checked against the ceiling. F3b (revision): the signature under the mixed
  truth (spring plus arm cube).
- F4 misspecified truths: (a) elastic centre away from the origin, S1/S2/S3,
  with the truth height moved (sensitivity rows); (b) hardening and deadband
  springs, the nonlinearity flag; (c) arm load only (no suspension): a rigid
  cube at an offset from the gripper markers, B2 (cube, mass and first moment),
  the "enough" rule and Q2 (S1B2 over B2); (d, revision) the mixed truth: S1
  "enough", S1B2 against the ceiling and Q2.
- Origin sensitivity (descriptive, no threshold): S1 refit with the footprint
  offset moved by +-1 sd per axis, and with the triad centroid as origin.
- F5 selection: the blocked-CV choice recovers S1 (S1 truth) and S2 with its
  h_e (elastic truth).

RNG: ``default_rng(seed)`` for the truth; noise ``default_rng(10000 + 100 k +
seed)``, ``k = 0`` white noise, ``1`` receive clock.

At level 0 there is neither white noise nor a clock error (every frame
received at its frame time); at the other levels the clock is the measured one
and only the white levels scale.

    python suspension_truth.py --out bands.json
    python suspension_truth.py --golden tests/golden/tiago_suspension_truth.json
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass, field
from functools import lru_cache
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
from scipy.signal import lfilter

project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from examples.tiago import suspension_model as sm  # noqa: E402
from examples.tiago.utils import suspension_data as sd  # noqa: E402

SETTINGS = sm.SETTINGS
FX = SETTINGS["fixture"]
SEEDS = tuple(FX["seeds"])
LEVELS = sm.LEVELS
USED = sm.USED
RATE = sm.RATE
NOISE = SETTINGS["noise"]
RULES = sm.RULES
STREAMS = {"white": 0, "clock": 1}
STATIC_FZ = -730.0  # N, static plate load of the real runs (descriptive)
C = np.asarray(SETTINGS["plate_centre"], dtype=float)
# loader errors put back one at a time (status quo value, fixed value)
ERRORS = {
    "origin": ("corner", "footprint"),
    "transport": (False, True),
    "frame": ("legacy", "footprint"),
    "clock": ("receive", "grid"),
}


def stream(kind: str, seed: int) -> np.random.Generator:
    """Noise stream: independent of the truth and of the other kinds."""
    return np.random.default_rng(10000 + 100 * STREAMS[kind] + seed)


# ------------------------------------------------------------------ truth


@dataclass
class Truth:
    seed: int
    kind: str  # spring | elastic | hard | deadband | arm | mixed
    k: np.ndarray  # (6,)
    c: np.ndarray  # (6,)
    h_e: float
    mass: float
    cube_offset: np.ndarray = field(default_factory=lambda: np.zeros(3))

    @property
    def theta(self) -> np.ndarray:
        """Parameter vector in the S1/S2 order ``[k_x, c_x, k_y, c_y, ...]``."""
        out = np.zeros(12)
        out[0::2], out[1::2] = self.k, self.c
        return out

    @property
    def theta_s1b2(self) -> np.ndarray:
        """S1B2 parameters ``[theta, m, m o]`` (B2 neglects the quadratic term in ``o``)."""
        return np.concatenate((self.theta, [self.mass], self.mass * self.cube_offset))


def draw_truth(seed: int, kind: str = "spring", h_e: Optional[float] = None) -> Truth:
    """Parameters around the configured values; the same draw for every ``kind``.

    ``h_e`` replaces the configured elastic-centre height (elastic truth; the
    sensitivity rows of F4a). ``mixed`` is the spring truth plus the arm cube.
    """
    t = FX["truth"]
    rng = np.random.default_rng(seed)
    s = t["spread"]
    k = np.asarray(t["k"]) * (1.0 + s * rng.uniform(-1.0, 1.0, 6))
    c = np.asarray(t["c"]) * (1.0 + s * rng.uniform(-1.0, 1.0, 6))
    h_e = (t["h_e"] if h_e is None else h_e) if kind == "elastic" else 0.0
    mass = t["arm_mass_kg"] if kind in ("arm", "mixed") else 0.0
    cube_offset = np.asarray(t["cube_offset_m"], dtype=float)
    if kind == "arm":
        k, c = np.zeros(6), np.zeros(6)
    return Truth(seed, kind, k, c, h_e, mass, cube_offset)


# ----------------------------------------------------------------- motion


class Motion:
    """Real corrected base pose and arm markers of one run (no plate channel).

    ``x`` is the pose of the footprint origin in the footprint axes (the loader's
    ``frame``), at rest in the static window; ``origin`` and ``axes`` the world
    point and axes of that frame; ``grip``, ``grip_rot`` the gripper-triad
    centroid and frame (filtered like the loader's).
    """

    def __init__(self, session_id: str):
        session = sm.load_sessions(verify=False)[session_id]
        selection = SETTINGS["sessions"]["selection_diagnostics"]
        if (
            session.role not in ("training", "validation")
            and session_id not in selection
        ):
            raise PermissionError(f"{session_id}: not a fixture motion")
        t, v = sd.read_raw(session.path)
        clock = sd.rebuild_clock(t, v, RATE)
        grid, _ = sd.to_grid(clock, v)
        self.id = session_id
        self.n = grid.shape[0]
        nw = int(round(sm.LOADER["static_window_s"] * RATE))
        markers = grid[:, 0:9].reshape(self.n, 3, 3)
        self.ref = markers[:nw].mean(axis=0)
        trans, angle, self.origin, self.axes = sd.base_pose(
            markers,
            self.ref,
            origin=sm.LOADER["origin"],
            frame=sm.LOADER["frame"],
            footprint=sm.LOADER["footprint"],
        )
        self.x = sd.lowpass(
            np.column_stack((trans, angle)), RATE, sm.LOADER["cutoff_hz"]
        )
        # at rest in the static window, so that the loader's rest pose is the truth's
        self.x[:nw] = 0.0
        self.v = np.gradient(self.x, 1.0 / RATE, axis=0)
        self.arm = grid[:, 9:30].copy()  # shoulder (4) then gripper (3) markers
        grip = grid[:, 21:30].reshape(self.n, 3, 3)
        self.grip = sd.lowpass(grip.mean(axis=1), RATE, sm.LOADER["cutoff_hz"])
        self.grip_rot = sd.lowpass(
            sd.marker_frames(grip).reshape(self.n, 9), RATE, sm.LOADER["cutoff_hz"]
        ).reshape(self.n, 3, 3)
        self.sd = self.x[nw:].std(axis=0)


@lru_cache(maxsize=None)
def motion(session_id: str) -> Motion:
    return Motion(session_id)


def true_wrench(truth: Truth, m: Motion, sd_ref: np.ndarray) -> np.ndarray:
    """(n, 6) wrench the plate receives about the base origin (noise free)."""
    x, v = m.x, m.v
    if truth.kind in ("spring", "elastic"):
        return sm.spring_design("S2", x, v, truth.h_e) @ truth.theta
    if truth.kind == "hard":
        h = FX["truth"]["hardening"]
        kx = x * truth.k
        kx[:, :3] = kx[:, :3] * (1.0 + (x[:, :3] / (h * sd_ref[:3])) ** 2)
        return kx + v * truth.c
    if truth.kind == "deadband":
        d = FX["truth"]["deadband"]
        eff = x.copy()
        lim = d * sd_ref[:2]
        eff[:, :2] = np.sign(x[:, :2]) * np.maximum(np.abs(x[:, :2]) - lim, 0.0)
        return eff * truth.k + v * truth.c
    if truth.kind == "arm":
        return cube_wrench(truth, m)
    if truth.kind == "mixed":  # additive: a stiff parallel path for the arm load
        return sm.spring_design("S1", x, v) @ truth.theta + cube_wrench(truth, m)
    raise ValueError(truth.kind)


def cube_wrench(truth: Truth, m: "Motion") -> np.ndarray:
    """(n, 6) exact wrench of a rigid cube (mass, centre ``c + R o``) on the plate.

    ``F = m (g - p'')``, ``M = (p - origin) x F - (m a^2 / 6) w'`` with ``p`` the
    cube centre; the fit (B2) neglects the quadratic term in ``o``.
    """
    kin = sm.cube_kinematics(m.grip, m.grip_rot)
    p = m.grip + m.grip_rot @ truth.cube_offset
    pdd = kin["cdd"] + kin["rdd"] @ truth.cube_offset
    f = truth.mass * (np.array([0.0, 0.0, -sm.G]) - pdd)
    edge = SETTINGS["b2"]["edge_m"]
    mom = np.cross(p - m.origin, f) - (truth.mass * edge**2 / 6.0) * kin["omega_dot"]
    return np.column_stack((f @ m.axes, mom @ m.axes))


def simulate_clock(n: int, level: float, rng: np.random.Generator):
    """``(keep, t)``: received frames and their host stamps, measured statistics."""
    if level == 0.0:
        return np.ones(n, dtype=bool), np.arange(n) / RATE
    c = NOISE["clock"]
    keep = rng.random(n) >= c["drop_rate"]
    keep[0] = True
    rho, burn = c["latency_lag1"], 200
    e = rng.normal(size=n + burn)
    u = lfilter([np.sqrt(1.0 - rho**2)], [1.0, -rho], e)[burn:]
    lat = np.clip(1e-3 * (c["latency_mean_ms"] + c["latency_sd_ms"] * u), 0.0, None)
    t = np.arange(n) / RATE + lat
    step = 1e-4 * np.arange(n)  # host stamps strictly increase
    t = np.maximum.accumulate(t - step) + step
    return keep, t[keep]


def raw_rows(truth: Truth, m: Motion, sd_ref, level, rng_white, rng_clock):
    """Raw receive-stamped rows ``(t, values)`` of the truth measured on a motion."""
    n = m.n
    w = true_wrench(truth, m, sd_ref)
    w = np.column_stack((w[:, :3] @ m.axes.T, w[:, 3:] @ m.axes.T))  # to world axes
    rot = sd.Rotation.from_rotvec(m.x[:, 3:] @ m.axes.T).as_matrix()
    p0 = m.ref - m.origin
    markers = (
        m.origin
        + (m.x[:, :3] @ m.axes.T)[:, None, :]
        + np.einsum("nab,ib->nia", rot, p0)
        + level * 1e-3 * NOISE["marker_sd_mm"] * rng_white.normal(size=(n, 3, 3))
    )
    plate_sd = level * np.asarray(NOISE["plate_sd"])
    force = np.array([0.0, 0.0, STATIC_FZ]) + w[:, :3]
    moment = w[:, 3:] - np.cross(C - m.origin, force)  # about the plate centre
    force = force + plate_sd[:3] * rng_white.normal(size=(n, 3))
    moment = moment + plate_sd[3:] * rng_white.normal(size=(n, 3))
    cop = np.column_stack(
        (
            C[0] - moment[:, 1] / force[:, 2],
            C[1] + moment[:, 0] / force[:, 2],
            np.zeros(n),
        )
    )
    values = np.hstack((markers.reshape(n, 9), m.arm, force, moment, cop))
    keep, t = simulate_clock(n, level, rng_clock)
    return t, values[keep]


class Fixture:
    """Real motions of the training, validation and selection runs, and the measuring chain."""

    def __init__(self):
        sess = SETTINGS["sessions"]
        self.train = motion(sess["training"])
        self.valid = motion(sess["validation"])
        self.diag = (
            {sid: motion(sid) for sid in sess["selection_diagnostics"]}
            if SETTINGS["selection"]["use_diagnostics"]
            else {}
        )
        self.sd_ref = self.train.sd

    def measure(self, truth: Truth, level: float, diagnostics: bool = False):
        """``{"training": (t, values), "validation": ..., <diagnostic id>: ...}`` for a truth.

        The diagnostic runs are measured after the two others from the same
        streams, so the training and validation measurements do not depend on
        ``diagnostics``.
        """
        white = stream("white", truth.seed)
        clock = stream("clock", truth.seed)
        out = {
            "training": raw_rows(truth, self.train, self.sd_ref, level, white, clock),
            "validation": raw_rows(truth, self.valid, self.sd_ref, level, white, clock),
        }
        if diagnostics:
            for sid, m in self.diag.items():
                out[sid] = raw_rows(truth, m, self.sd_ref, level, white, clock)
        return out

    def load(self, truth: Truth, level: float, diagnostics: bool = False, **overrides):
        """Corrected-loader ``(Regressors, wrench)`` of the runs."""
        raw = self.measure(truth, level, diagnostics)
        return {k: sm.regressors_from_raw(*v, **overrides) for k, v in raw.items()}

    def selection_data(self, truth: Truth, level: float):
        """``[(run, wrench)]`` of training and the selection diagnostics (O3)."""
        d = self.load(truth, level, diagnostics=True)
        return [d["training"]] + [d[sid] for sid in self.diag]


# ------------------------------------------------------------------- F1


def f1_cells() -> dict:
    """Condition numbers and per-column SNR on the real training and validation motion."""
    sess = sm.load_sessions(verify=False)
    out = {}
    cmax = FX["f1"]["cond_max"]
    grid = sm.h_e_grid()
    for role in ("training", "validation"):
        r = sm.read_regressors(sess[SETTINGS["sessions"][role]])
        snr_x = r.x[r.valid].std(axis=0) / np.asarray(NOISE["x_sd"])
        snr_v = r.v[r.valid].std(axis=0) / np.asarray(NOISE["v_sd"])
        cond = {
            "s1": sm.design_condition("S1", r),
            "s2": max(sm.design_condition("S2", r, h) for h in grid),
            "s3": sm.design_condition("S3", r),
        }
        low = [
            f"{kind}_{d}"
            for kind, snr in (("x", snr_x), ("v", snr_v))
            for d, s in zip(sm.DOFS, snr)
            if s < FX["f1"]["snr_min"]
        ]
        out[role] = {
            "cond": cond,
            "pass": all(cond[k] <= cmax[k] for k in cond),
            "snr_x": snr_x.tolist(),
            "snr_v": snr_v.tolist(),
            "low_snr": low,
        }
    return out


# ------------------------------------------------------------------- F2


def _r2(w, pred, valid):
    return sm.score(w, pred, valid)["r2"]


def s1_cell(fx: Fixture, truth: Truth, level: float, bootstrap: bool) -> dict:
    """S1 fit on the training measurement, held out on the validation measurement."""
    d = fx.load(truth, level)
    (rt, wt), (rv, wv) = d["training"], d["validation"]
    fit = sm.fit_model("S1", rt, wt, bootstrap=bootstrap)
    fit_v = sm.fit_model("S1", rv, wv)
    ceiling = _r2(wv, sm.design("S1", rv) @ truth.theta, rv.valid)
    r2 = _r2(wv, sm.predict(fit, rv), rv.valid)
    th = truth.theta
    cell = {
        "seed": truth.seed,
        "rel_err": ((fit.theta - th) / th).tolist(),
        "rel_err_max": float(np.max(np.abs(fit.theta - th) / np.abs(th))),
        "r2": r2.tolist(),
        "ceiling": ceiling.tolist(),
        "gap": (ceiling - r2).tolist(),
        "k_rel_diff": (np.abs(fit.k() - fit_v.k()) / np.abs(fit_v.k())).tolist(),
        "n_active": int(fit.active.sum()),
    }
    if bootstrap:
        z = (fit.theta - th) / fit.se
        eiv = sm.fit_s1_eiv(rt, wt, level)
        cell.update(
            z=z.tolist(),
            k_rel_se=(fit.k_se() / truth.k).tolist(),
            c_rel_se=(fit.c_se() / truth.c).tolist(),
            eiv_rel_err=((eiv - th) / th).tolist(),
            eiv_z=((eiv - th) / fit.se).tolist(),
            c_unidentified=fit.damping_unidentified().tolist(),
        )
    return cell


def f2_cell(fx: Fixture, seed: int, level: float) -> dict:
    return s1_cell(fx, draw_truth(seed), level, bootstrap=level > 0.0)


def z_stats(cells) -> dict:
    z = np.concatenate([c["z"] for c in cells])
    return {
        "rms": float(np.sqrt(np.mean(z**2))),
        "mean": float(np.mean(z)),
        "n": int(z.size),
    }


def bias_stats(cells, key: str = "rel_err") -> dict:
    """Mean over seeds of the signed relative error, stiffness and damping (6 each)."""
    e = np.mean([c[key] for c in cells], axis=0)
    return {"k": e[0::2].tolist(), "c": e[1::2].tolist()}


def context(f2_used: list) -> dict:
    """Answerable DOF, repeatability band, bias and calibrated SE, from the F2 cells of the used level.

    The calibrated relative SE is ``sqrt(SE^2 + bias^2)`` (part 2 reports it):
    the bootstrap sees the scatter, not the errors-in-variables bias.
    """
    rel_se = np.mean([c["k_rel_se"] for c in f2_used], axis=0)
    c_se = np.mean([c["c_rel_se"] for c in f2_used], axis=0)
    diff = np.concatenate([c["k_rel_diff"] for c in f2_used])
    bias = bias_stats(f2_used)
    return {
        "k_rel_se": rel_se.tolist(),
        "answerable": (rel_se <= RULES["answerable_rel_se"]).tolist(),
        "c_rel_se": c_se.tolist(),
        "k_bias": bias["k"],
        "c_bias": bias["c"],
        "k_rel_se_cal": np.hypot(rel_se, bias["k"]).tolist(),
        "c_rel_se_cal": np.hypot(c_se, bias["c"]).tolist(),
        "k_rel_diff_rms": float(np.sqrt(np.mean(diff**2))),
        "band": float(RULES["band_factor"] * np.sqrt(np.mean(diff**2))),
    }


def grade_bias(ctx: dict) -> dict:
    """F2-bias: the used-level mean relative bias within ``f2.bias_max``, answerable DOF."""
    t = FX["f2"]["bias_max"]
    ans = np.asarray(ctx["answerable"], bool)
    k = float(np.max(np.abs(np.asarray(ctx["k_bias"])[ans])))
    c = float(np.max(np.abs(np.asarray(ctx["c_bias"])[ans])))
    return {"k_max": k, "c_max": c, "pass": bool(k <= t["k"] and c <= t["c"])}


def mean_gap(cells, answerable) -> float:
    """Mean over seeds and answerable DOF of (ceiling - held-out R2)."""
    gap = np.asarray([c["gap"] for c in cells])
    return float(gap[:, np.asarray(answerable, bool)].mean())


# ------------------------------------------------------------------- F3


def _fq(r, w, valid=None):
    """Status-quo S1 fit (plain LS) on a loader output."""
    m = r.valid if valid is None else valid
    return sm.fit_status_quo(r.x[m], r.v[m], w[m])


def _arm(fx: Fixture, truth, level, fixed=(), all_fixed=False):
    """Held-out R2 of the status-quo S1 fit with the loader errors in ``fixed`` removed."""
    ov = {k: (v[1] if (all_fixed or k in fixed) else v[0]) for k, v in ERRORS.items()}
    d = fx.load(truth, level, **ov)
    (rt, wt), (rv, wv) = d["training"], d["validation"]
    # the status quo has no edge/guard rows: every row of its own clock is used
    use_t = rt.valid if ov["clock"] == "grid" else np.ones(rt.n, bool)
    use_v = rv.valid if ov["clock"] == "grid" else np.ones(rv.n, bool)
    f = _fq(rt, wt, use_t)
    pred = sm.predict_status_quo(f.parameters, rv.x, rv.v)
    r2 = _r2(wv, pred, use_v)
    return r2, f.parameters[1::2]


def b1_signature(fx: Fixture, truth: Truth, level: float) -> dict:
    """The status-quo loader + S1 (B1): held-out R2 and the damping signs."""
    raw = fx.measure(truth, level)
    traj = {k: sd.status_quo_from_arrays(*v) for k, v in raw.items()}
    f = sm.fit_status_quo(
        traj["training"].base_displacement,
        traj["training"].base_velocity,
        traj["training"].wrench,
    )
    tv = traj["validation"]
    pred = sm.predict_status_quo(f.parameters, tv.base_displacement, tv.base_velocity)
    r2 = _r2(tv.wrench, pred, np.ones(tv.sample_count, bool))
    return {
        "r2": r2.tolist(),
        "forces_r2": float(np.mean(r2[:3])),
        "my_r2": float(r2[4]),
        "neg_damping": int(np.sum(f.parameters[1::2] < 0.0)),
        "c": f.parameters[1::2].tolist(),
    }


def f3_cell(fx: Fixture, seed: int, level: float) -> dict:
    """Status-quo signature and the one-at-a-time ablation of the four errors."""
    truth = draw_truth(seed)
    out = {"seed": seed, "b1": b1_signature(fx, truth, level)}
    base, c_base = _arm(fx, truth, level)
    out["arm_status_quo"] = {
        "forces_r2": float(np.mean(base[:3])),
        "my_r2": float(base[4]),
        "r2": base.tolist(),
    }
    gains = {}
    for name in ERRORS:
        r2a, _ = _arm(fx, truth, level, fixed=(name,))
        gains[name] = float(np.mean(r2a[:3]) - np.mean(base[:3]))
    out["gain"] = gains
    full, _ = _arm(fx, truth, level, all_fixed=True)
    out["plain_fixed_forces_r2"] = float(np.mean(full[:3]))
    out["full"] = s1_cell(fx, truth, level, bootstrap=False)
    return out


def f3b_cell(fx: Fixture, seed: int, level: float) -> dict:
    """F3b: the B1 signature under the mixed truth (spring plus the arm cube)."""
    return {"seed": seed, "b1": b1_signature(fx, draw_truth(seed, "mixed"), level)}


def _signature(cells) -> List[bool]:
    sig = FX["f3"]["signature"]
    return [
        c["b1"]["forces_r2"] < sig["forces_r2_max"]
        and c["b1"]["my_r2"] >= sig["my_r2_min"]
        and c["b1"]["neg_damping"] >= sig["neg_damping_min"]
        for c in cells
    ]


def grade_f3b(cells) -> dict:
    seeds = _signature(cells)
    return {
        "signature_seeds": int(sum(seeds)),
        "signature_pass": bool(sum(seeds) >= FX["f3"]["signature"]["seeds_min"]),
        "forces_r2": float(np.mean([c["b1"]["forces_r2"] for c in cells])),
        "my_r2": float(np.mean([c["b1"]["my_r2"] for c in cells])),
        "neg_damping": float(np.mean([c["b1"]["neg_damping"] for c in cells])),
        "r2": np.mean([c["b1"]["r2"] for c in cells], axis=0).tolist(),
    }


def grade_f3(cells, answerable) -> dict:
    sig = FX["f3"]["signature"]
    seeds = _signature(cells)
    gain = {k: float(np.mean([c["gain"][k] for c in cells])) for k in ERRORS}
    resp = [k for k, v in gain.items() if v >= FX["f3"]["attribution_gain_min"]]
    gap = mean_gap([c["full"] for c in cells], answerable)
    return {
        "signature_seeds": int(sum(seeds)),
        "signature_pass": bool(sum(seeds) >= sig["seeds_min"]),
        "gain": gain,
        "responsible": resp,
        "attribution_pass": bool(resp),
        "full_gap": gap,
        "full_pass": bool(gap <= FX["f3"]["full_r2_gap_max"]),
        "forces_r2": float(np.mean([c["b1"]["forces_r2"] for c in cells])),
        "my_r2": float(np.mean([c["b1"]["my_r2"] for c in cells])),
        "neg_damping": float(np.mean([c["b1"]["neg_damping"] for c in cells])),
    }


# ------------------------------------------------------------------- F4


def _resid(fit, r, w):
    """In-sample residual including the offset (the nonlinearity flag input)."""
    full = sm._full_design(sm.design(fit.model, r, fit.h_e), fit.offset)
    return w - full @ fit.params


def flags(fit, r, w) -> np.ndarray:
    """(6,) |corr(residual, cubic part of x)| of an S1 fit on the training rows."""
    res = _resid(fit, r, w)
    return np.array(
        [sm.nonlinear_flag(r.x[:, j], res[:, j], r.valid) for j in range(6)]
    )


def f4a_cell(
    fx: Fixture, seed: int, level: float, ctx: dict, h_e: Optional[float] = None
) -> dict:
    """Elastic-centre truth (at ``h_e``, default the configured one): selection, S1/S2/S3."""
    truth = draw_truth(seed, "elastic", h_e)
    d = fx.load(truth, level)
    (rt, wt), (rv, wv) = d["training"], d["validation"]
    sel = sm.select_model(fx.selection_data(truth, level))
    f1 = sm.fit_model("S1", rt, wt)
    f2 = sm.fit_model("S2", rt, wt, h_e=sel["h_e_cv"])
    f3 = sm.fit_model("S3", rt, wt)
    r2 = {
        n: _r2(wv, sm.predict(f, rv), rv.valid)
        for n, f in zip(("S1", "S2", "S3"), (f1, f2, f3))
    }
    ceiling = _r2(wv, sm.design("S2", rv, truth.h_e) @ truth.theta, rv.valid)
    ans = np.asarray(ctx["answerable"], bool)
    return {
        "seed": seed,
        "r2": {n: v.tolist() for n, v in r2.items()},
        "ceiling": ceiling.tolist(),
        "gain": float(np.mean(r2["S2"] - r2["S1"])),
        "gap": float(np.mean((ceiling - r2["S2"])[ans])),
        "h_e_true": truth.h_e,
        "h_e_cv": sel["h_e_cv"],
        "model": sel["model"],
    }


def f4a_ok(cell) -> bool:
    """F4a per seed (revision): S2 near the ceiling and h_e recovered."""
    t = FX["f4"]
    return bool(
        cell["gap"] <= t["s2_gap_max"]
        and abs(cell["h_e_cv"] - cell["h_e_true"]) <= t["h_e_tol"]
    )


def summarize_f4a(a) -> dict:
    ok = [f4a_ok(x) for x in a]
    return {
        "gain": float(np.mean([x["gain"] for x in a])),
        "gap": float(np.mean([x["gap"] for x in a])),
        "h_e_true": a[0]["h_e_true"],
        "h_e_cv": [x["h_e_cv"] for x in a],
        "models": [x["model"] for x in a],
        "ok_seeds": int(sum(ok)),
        "pass": bool(sum(ok) >= FX["f4"]["flag_seeds_min"]),
    }


def f4b_cell(fx: Fixture, seed: int, level: float) -> dict:
    out = {"seed": seed}
    for kind, dofs in (("hard", (0, 1, 2)), ("deadband", (0, 1))):
        truth = draw_truth(seed, kind)
        (rt, wt), _ = fx.load(truth, level).values()
        fl = flags(sm.fit_model("S1", rt, wt), rt, wt)
        out[kind] = {
            "corr": fl.tolist(),
            "flagged": [bool(abs(fl[j]) > RULES["nonlinear_corr"]) for j in dofs],
            "dofs": list(dofs),
        }
    truth = draw_truth(seed)  # linear control
    (rt, wt), _ = fx.load(truth, level).values()
    fl = flags(sm.fit_model("S1", rt, wt), rt, wt)
    out["linear"] = {
        "corr": fl.tolist(),
        "false_flags": int(np.sum(np.abs(fl) > RULES["nonlinear_corr"])),
    }
    return out


def f4c_cell(fx: Fixture, seed: int, level: float, ctx: dict) -> dict:
    truth = draw_truth(seed, "arm")
    d = fx.load(truth, level)
    (rt, wt), (rv, wv) = d["training"], d["validation"]
    s1 = sm.fit_model("S1", rt, wt, bootstrap=True)
    s1v = sm.fit_model("S1", rv, wv)
    b2 = sm.fit_model("B2", rt, wt)
    r2_s1 = _r2(wv, sm.predict(s1, rv), rv.valid)
    r2_b2 = _r2(wv, sm.predict(b2, rv), rv.valid)
    out = _s1_verdict(s1, s1v, rt, wt, r2_s1, r2_b2, b2, ctx)
    out.update(_q2(rt, wt, rv, wv, r2_b2))
    out.pop("_s1b2")
    mass = float(b2.theta[0])
    # the isotropic inertia a^2 m / 6: B2 refitted with the edge at 0 and doubled
    inertia = {}
    for name, edge in (
        ("none", 0.0),
        ("assumed", None),
        ("double", 2 * SETTINGS["b2"]["edge_m"]),
    ):
        f = sm.fit_model("B2", rt, wt, edge=edge)
        inertia[name] = _r2(wv, sm.predict(f, rv), rv.valid).tolist()
    out.update(
        seed=seed,
        mass_hat=mass,
        o_hat=(b2.theta[1:] / mass).tolist() if mass > 0 else [float("nan")] * 3,
        inertia_r2=inertia,
    )
    return out


def _s1_verdict(s1, s1v, rt, wt, r2_s1, r2_b2, b2, ctx) -> dict:
    """The "enough" rule on an S1 fit (stiffness bound only, O9) against B2."""
    at_bound = s1.stiffness_bound_dofs()
    kv = np.abs(s1v.k())
    rep = np.abs(s1.k() - s1v.k()) <= ctx["band"] * np.where(kv > 0, kv, np.nan)
    nonlin = np.abs(flags(s1, rt, wt)) > RULES["nonlinear_corr"]
    v = sm.verdict(
        r2_s1, r2_b2, at_bound, rep, nonlin, ctx["answerable"], b2.b2_at_bound()
    )
    return {
        "r2_s1": r2_s1.tolist(),
        "r2_b2": r2_b2.tolist(),
        "b2_at_bound": b2.b2_at_bound(),
        "at_bound": at_bound.tolist(),
        "any_bound": s1.bound_dofs().tolist(),
        "c_unidentified": s1.damping_unidentified().tolist(),
        "repeatable": rep.tolist(),
        "nonlinear": nonlin.tolist(),
        "verdict": v,
    }


def _q2(rt, wt, rv, wv, r2_b2) -> dict:
    """S1B2 on training, held out on validation, and Q2 against B2."""
    f = sm.fit_model("S1B2", rt, wt)
    r2 = _r2(wv, sm.predict(f, rv), rv.valid)
    q2 = sm.suspension_adds(r2, r2_b2, f.stiffness_bound_dofs())
    return {
        "r2_s1b2": r2.tolist(),
        "s1b2_k": f.k().tolist(),
        "s1b2_mass": float(f.theta[12]),
        "q2": q2,
        "_s1b2": f,
    }


def f4d_cell(fx: Fixture, seed: int, level: float, ctx: dict) -> dict:
    """F4d: mixed truth (spring plus arm cube): S1 "enough", S1B2 at the ceiling, Q2."""
    truth = draw_truth(seed, "mixed")
    d = fx.load(truth, level)
    (rt, wt), (rv, wv) = d["training"], d["validation"]
    s1 = sm.fit_model("S1", rt, wt, bootstrap=True)
    s1v = sm.fit_model("S1", rv, wv)
    b2 = sm.fit_model("B2", rt, wt)
    r2_s1 = _r2(wv, sm.predict(s1, rv), rv.valid)
    r2_b2 = _r2(wv, sm.predict(b2, rv), rv.valid)
    out = _s1_verdict(s1, s1v, rt, wt, r2_s1, r2_b2, b2, ctx)
    out.update(_q2(rt, wt, rv, wv, r2_b2))
    f = out.pop("_s1b2")
    ceiling = _r2(wv, sm.design("S1B2", rv) @ truth.theta_s1b2, rv.valid)
    ans = np.asarray(ctx["answerable"], bool)
    gap = ceiling - np.asarray(out["r2_s1b2"])
    out.update(
        seed=seed,
        ceiling=ceiling.tolist(),
        s1b2_gap=float(np.mean(gap[ans])),
        s1b2_k_rel_err=((f.k() - truth.k) / truth.k).tolist(),
        s1b2_mass_rel_err=float((f.theta[12] - truth.mass) / truth.mass),
    )
    return out


def summarize_f4(a, b, c, d, a_sens=None) -> dict:
    t = FX["f4"]
    flagged = {k: sum(any(x[k]["flagged"]) for x in b) for k in ("hard", "deadband")}
    false = sum(x["linear"]["false_flags"] for x in b)
    not_enough = sum(not x["verdict"]["enough"] for x in c)
    q2_absent = sum(not x["q2"]["present"] for x in c)
    d_s1 = sum(not x["verdict"]["enough"] for x in d)
    d_gap = sum(x["s1b2_gap"] <= t["s2_gap_max"] for x in d)
    d_q2 = sum(x["q2"]["present"] for x in d)
    n_min = t["q2_seeds_min"]
    return {
        "a": summarize_f4a(a),
        "a_sensitivity": {
            str(h): summarize_f4a(cells) for h, cells in (a_sens or {}).items()
        },
        "b": {
            "flagged_seeds": flagged,
            "false_flags": int(false),
            "pass": bool(
                all(v >= t["flag_seeds_min"] for v in flagged.values()) and false == 0
            ),
        },
        "c": {
            "not_enough_seeds": int(not_enough),
            "mass_hat": float(np.mean([x["mass_hat"] for x in c])),
            "o_hat": np.mean([x["o_hat"] for x in c], axis=0).tolist(),
            "s1_at_bound_dofs": float(np.mean([np.sum(x["at_bound"]) for x in c])),
            "inertia_max_dr2": float(
                max(
                    np.max(
                        np.abs(
                            np.asarray(x["inertia_r2"][k]) - x["inertia_r2"]["assumed"]
                        )
                    )
                    for x in c
                    for k in ("none", "double")
                )
            ),
            "r2_s1": np.mean([x["r2_s1"] for x in c], axis=0).tolist(),
            "r2_b2": np.mean([x["r2_b2"] for x in c], axis=0).tolist(),
            "r2_s1b2": np.mean([x["r2_s1b2"] for x in c], axis=0).tolist(),
            "q2_gain": np.mean([x["q2"]["gain"] for x in c], axis=0).tolist(),
            "q2_absent_seeds": int(q2_absent),
            "q2_pass": bool(q2_absent >= n_min),
            "pass": bool(not_enough >= t["enough_false_seeds_min"]),
        },
        "d": {
            "s1_not_enough_seeds": int(d_s1),
            "s1b2_gap_seeds": int(d_gap),
            "q2_present_seeds": int(d_q2),
            "s1b2_gap": float(np.mean([x["s1b2_gap"] for x in d])),
            "r2_s1": np.mean([x["r2_s1"] for x in d], axis=0).tolist(),
            "r2_b2": np.mean([x["r2_b2"] for x in d], axis=0).tolist(),
            "r2_s1b2": np.mean([x["r2_s1b2"] for x in d], axis=0).tolist(),
            "ceiling": np.mean([x["ceiling"] for x in d], axis=0).tolist(),
            "q2_gain": np.mean([x["q2"]["gain"] for x in d], axis=0).tolist(),
            "s1b2_k_rel_err": np.mean(
                [np.abs(x["s1b2_k_rel_err"]) for x in d], axis=0
            ).tolist(),
            "s1b2_mass_rel_err": float(np.mean([x["s1b2_mass_rel_err"] for x in d])),
            "pass": bool(d_s1 >= n_min and d_gap >= n_min and d_q2 >= n_min),
        },
    }


# ------------------------------------------------------------------- F5


def f5_cell(fx: Fixture, seed: int, level: float) -> dict:
    out = {"seed": seed}
    for kind in ("spring", "elastic"):
        truth = draw_truth(seed, kind)
        sel = sm.select_model(fx.selection_data(truth, level))
        out[kind] = {
            "model": sel["model"],
            "h_e": sel["h_e"],
            "h_e_cv": sel["h_e_cv"],
            "cv": sel["cv"],
        }
    return out


def summarize_f5(cells) -> dict:
    t = FX["f5"]
    s1 = sum(c["spring"]["model"] == "S1" for c in cells)
    s2 = sum(
        c["elastic"]["model"] == "S2"
        and abs(c["elastic"]["h_e"] - FX["truth"]["h_e"]) <= t["h_e_tol"]
        for c in cells
    )
    return {
        "s1_correct": int(s1),
        "s2_correct": int(s2),
        "models_elastic": [c["elastic"]["model"] for c in cells],
        "h_e_elastic": [c["elastic"]["h_e"] for c in cells],
        "h_e_cv_elastic": [c["elastic"]["h_e_cv"] for c in cells],
        "pass": bool(s1 >= t["correct_seeds_min"] and s2 >= t["correct_seeds_min"]),
    }


# ------------------------------------------------------- origin sensitivity


def origin_variants() -> dict:
    """Loader overrides: the footprint offset moved by +-factor sd per axis, and the centroid."""
    fp = SETTINGS["footprint"]
    base = np.asarray(fp["offset_triad_m"], dtype=float)
    sd_off = np.asarray(fp["offset_sd_m"], dtype=float)
    factor = FX["origin_sensitivity"]["factor"]
    out = {}
    for j, axis in enumerate("xyz"):
        for sign, tag in ((1.0, "+"), (-1.0, "-")):
            off = base.copy()
            off[j] += sign * factor * sd_off[j]
            out[f"{axis}{tag}"] = {
                "footprint": dict(offset=off, rotvec=sm.LOADER["footprint"]["rotvec"])
            }
    out["centroid"] = {"origin": "centroid"}
    return out


def origin_cell(fx: Fixture, seed: int, level: float) -> dict:
    """S1 refit on the S1 truth measured about the footprint, loaded about a moved origin.

    Descriptive: the change of the 12 parameters and of the held-out R2 with
    respect to the nominal (correct) loader.
    """
    truth = draw_truth(seed)
    raw = fx.measure(truth, level)

    def fit_with(**ov):
        d = {k: sm.regressors_from_raw(*v, **ov) for k, v in raw.items()}
        (rt, wt), (rv, wv) = d["training"], d["validation"]
        f = sm.fit_model("S1", rt, wt)
        return f.theta, _r2(wv, sm.predict(f, rv), rv.valid)

    th0, r20 = fit_with()
    out = {"seed": seed, "nominal_r2": float(np.mean(r20))}
    for name, ov in origin_variants().items():
        th, r2 = fit_with(**ov)
        rel = np.abs(th - th0) / np.abs(th0)
        out[name] = {
            "max_rel": float(rel.max()),
            "mean_rel": float(rel.mean()),
            "k_rel": rel[0::2].tolist(),
            "c_rel": rel[1::2].tolist(),
            "dr2": float(np.mean(r2) - np.mean(r20)),
            "r2": float(np.mean(r2)),
        }
    return out


def summarize_origin(cells) -> dict:
    out = {"nominal_r2": float(np.mean([c["nominal_r2"] for c in cells]))}
    for name in origin_variants():
        out[name] = {
            "max_rel": float(np.mean([c[name]["max_rel"] for c in cells])),
            "mean_rel": float(np.mean([c[name]["mean_rel"] for c in cells])),
            "k_rel": np.mean([c[name]["k_rel"] for c in cells], axis=0).tolist(),
            "c_rel": np.mean([c[name]["c_rel"] for c in cells], axis=0).tolist(),
            "dr2": float(np.mean([c[name]["dr2"] for c in cells])),
            "dr2_worst": float(min(c[name]["dr2"] for c in cells)),
        }
    return out


# ----------------------------------------------------------------- driver


def run(seeds=SEEDS, levels=LEVELS, verbose=True) -> dict:
    fx = Fixture()
    out = {"seeds": list(seeds), "levels": list(levels), "used": USED, "f1": f1_cells()}
    out["f2"] = {}
    for lv in levels:
        if verbose:
            print(f"F2 level {lv}", flush=True)
        cells = [f2_cell(fx, s, lv) for s in seeds]
        entry = {"cells": cells, "rel_err_max": max(c["rel_err_max"] for c in cells)}
        if lv > 0.0:
            entry["z"] = z_stats(cells)
            entry["bias"] = bias_stats(cells)
            entry["eiv"] = {
                "bias": bias_stats(cells, "eiv_rel_err"),
                "z": z_stats([{"z": c["eiv_z"]} for c in cells]),
            }
            entry["c_unidentified"] = np.sum(
                [c["c_unidentified"] for c in cells], axis=0
            ).tolist()
        out["f2"][str(lv)] = entry
    used = out["f2"][str(USED)]["cells"]
    ctx = context(used)
    out["context"] = ctx
    out["f2_bias"] = grade_bias(ctx)
    for lv in levels:
        if lv > 0.0:
            out["f2"][str(lv)]["gap"] = mean_gap(
                out["f2"][str(lv)]["cells"], ctx["answerable"]
            )
    if verbose:
        print("F3", flush=True)
    f3 = [f3_cell(fx, s, USED) for s in seeds]
    out["f3"] = {"cells": f3, **grade_f3(f3, ctx["answerable"])}
    f3b = [f3b_cell(fx, s, USED) for s in seeds]
    out["f3b"] = {"cells": f3b, **grade_f3b(f3b)}
    if verbose:
        print("F4", flush=True)
    a = [f4a_cell(fx, s, USED, ctx) for s in seeds]
    a_sens = {
        h: [f4a_cell(fx, s, USED, ctx, h_e=h) for s in seeds]
        for h in FX["truth"]["h_e_sensitivity"]
    }
    b = [f4b_cell(fx, s, USED) for s in seeds]
    c = [f4c_cell(fx, s, USED, ctx) for s in seeds]
    d = [f4d_cell(fx, s, USED, ctx) for s in seeds]
    out["f4"] = {
        "cells": {"a": a, "b": b, "c": c, "d": d},
        **summarize_f4(a, b, c, d, a_sens),
    }
    if verbose:
        print("F5", flush=True)
    f5 = [f5_cell(fx, s, USED) for s in seeds]
    out["f5"] = {"cells": f5, **summarize_f5(f5)}
    if verbose:
        print("origin sensitivity", flush=True)
    org = [origin_cell(fx, s, USED) for s in seeds]
    out["origin"] = {"cells": org, **summarize_origin(org)}
    return out


def grade(res: dict) -> dict:
    """Pass/fail of the pre-registered checks (F2 at every level, F3-F5 at the used level)."""
    t = FX["f2"]
    zs = {lv: e["z"] for lv, e in res["f2"].items() if "z" in e}
    z_ok = all(
        t["z_rms"][0] <= z["rms"] <= t["z_rms"][1] and abs(z["mean"]) < t["z_mean_max"]
        for z in zs.values()
    )
    exact = (
        res["f2"]["0.0"]["rel_err_max"] <= t["exact_tol"]
        if "0.0" in res["f2"]
        else None
    )
    gap_ok = all(e["gap"] <= t["r2_gap_max"] for e in res["f2"].values() if "gap" in e)
    f3, f4, f5 = res["f3"], res["f4"], res["f5"]
    return {
        "F1": bool(all(v["pass"] for v in res["f1"].values())),
        "F2_z": bool(z_ok),
        "F2_exact": exact,
        "F2_gap": bool(gap_ok),
        "F2_bias": res["f2_bias"]["pass"],
        "F3_signature": f3["signature_pass"],
        "F3_attribution": f3["attribution_pass"],
        "F3_full": f3["full_pass"],
        "F3b_signature": res["f3b"]["signature_pass"],
        "F4a": f4["a"]["pass"],
        "F4b": f4["b"]["pass"],
        "F4c": f4["c"]["pass"],
        "F4c_q2": f4["c"]["q2_pass"],
        "F4d": f4["d"]["pass"],
        "F5": f5["pass"],
    }


def _round(x, nd=6):
    if isinstance(x, dict):
        return {k: _round(v, nd) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_round(v, nd) for v in x]
    if isinstance(x, (float, np.floating)):
        return round(float(x), nd)
    if isinstance(x, (np.integer,)):
        return int(x)
    if isinstance(x, (np.bool_,)):
        return bool(x)
    return x


def golden(res: dict) -> dict:
    """What the tests and the note quote: summaries and the seed-0 cells."""
    f2 = {
        lv: {k: v for k, v in e.items() if k != "cells"} for lv, e in res["f2"].items()
    }
    f3 = {k: v for k, v in res["f3"].items() if k != "cells"}
    f3b = {k: v for k, v in res["f3b"].items() if k != "cells"}
    f4 = {k: v for k, v in res["f4"].items() if k != "cells"}
    f5 = {k: v for k, v in res["f5"].items() if k != "cells"}
    origin = {k: v for k, v in res["origin"].items() if k != "cells"}
    return _round(
        {
            "used": res["used"],
            "seeds": res["seeds"],
            "noise": {
                k: NOISE[k]
                for k in ("marker_sd_mm", "plate_sd", "x_sd", "v_sd", "clock")
            },
            "criteria": grade(res),
            "f1": res["f1"],
            "context": res["context"],
            "f2": f2,
            "f2_bias": res["f2_bias"],
            "f3": f3,
            "f3b": f3b,
            "f4": f4,
            "f5": f5,
            "origin": origin,
            "seed0": {
                "f2": res["f2"][str(res["used"])]["cells"][0],
                "f3": res["f3"]["cells"][0],
                "f4a": res["f4"]["cells"]["a"][0],
                "f4c": res["f4"]["cells"]["c"][0],
                "f4d": res["f4"]["cells"]["d"][0],
                "f5": res["f5"]["cells"][0],
            },
        }
    )


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--seeds", type=int, nargs="+", default=list(SEEDS))
    parser.add_argument("--out", type=Path)
    parser.add_argument("--golden", type=Path)
    parser.add_argument(
        "--no-plot", action="store_true", help="accepted for validate.py"
    )
    args = parser.parse_args(argv)
    res = run(tuple(args.seeds))
    print(json.dumps(grade(res)))
    if args.out:
        args.out.write_text(json.dumps(_round(res), indent=1) + "\n")
    if args.golden:
        args.golden.write_text(json.dumps(golden(res), indent=1, sort_keys=True) + "\n")


if __name__ == "__main__":
    main()
