"""TIAGo backlash surface (#71): what the model and the noise alone explain.

A simulated fixture. One truth per seed (backlash width ``W``, load terms
``G`` and ``kappa``, the encoder angle error ``E`` with an arm_5 step, a
session offset) is measured at the *real* configuration of the training
(2023-07-24) and validation (2023-03-09 bottom) runs, taken from the
regressor columns only: ``q_rel``, its velocity and the gravity torque. The
absolute encoder of the runs is never read here; ``q_abs`` is synthesized
from ``q_rel`` and the truth by solving ``q_abs + E(q_abs) = q_rel - c -
b - G sg - kappa tau_g`` exactly (bisection: the fixed-point iteration
of the plan does not contract on the arm_5 step, whose slope exceeds 1).
Noise: the AR(1) level grid of the noise stage, or quantization of ``q_abs``.
Criteria are fixed in ``config/tiago_backlash_study_config.yaml`` before any
run.

- F1 identifiability (real regressor columns): the M3 design is full rank
  with a scaled condition number below the limit on every joint and run;
  the correlation of ``h`` with the soft-signed gravity torque and the
  direction overlap are reported.
- F2 recovery: a free M3 fit recovers ``W``, ``G`` and ``kappa`` (z-scores
  from the block bootstrap) at every noise level, and with a quantized
  ``q_abs``.
- F3 separation (arm_5): with a 170 mrad step in ``E`` and ``W = 0`` the fit
  finds no width; with ``W = 12 mrad`` it finds it; with ``E`` as a function
  of ``q_rel`` it does not (this is why ``E`` takes ``q_abs``).
- F4 transfer: the training model, offset re-anchored on a simulated
  validation run, predicts the noise-free truth; the V1 band (null band under
  a direction-only truth) and the width band of V2 are set here.
- F5 selection: the frozen rule admits the load group rarely without load
  and mostly with a 10 mrad ``G`` on arm_6.
- F6 jumps: injected encoder jumps are all found, none is invented at the
  top noise level.

RNG: ``default_rng(seed)`` for the truth; noise ``default_rng(10000 + 100 k +
seed)``, ``k = 0`` AR(1), ``1`` quantization, ``2`` jumps.

    python backlash_truth.py --out bands.json
    python backlash_truth.py --golden tests/golden/tiago_backlash_truth.json
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
from scipy.signal import lfilter

project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from examples.tiago import backlash_model as bm  # noqa: E402

SETTINGS = bm.SETTINGS
FX = SETTINGS["fixture"]
SEEDS = tuple(FX["seeds"])
LEVELS = bm.LEVELS
USED = float(SETTINGS["noise"]["used_level"])
JOINTS = bm.JOINTS
STREAMS = {"ar1": 0, "quantization": 1, "jumps": 2}
NOISE = SETTINGS["noise"]


def stream(kind: str, seed: int) -> np.random.Generator:
    """Noise stream: independent of the truth and of the other kinds."""
    return np.random.default_rng(10000 + 100 * STREAMS[kind] + seed)


@dataclass
class Truth:
    seed: int
    W: np.ndarray  # (7,) rad
    G: np.ndarray
    kappa: np.ndarray
    e_scale: np.ndarray  # (7,) signed
    e_a2: np.ndarray
    e_a3: np.ndarray
    step_centre: float  # arm_5
    c: Dict[str, np.ndarray]  # session role -> (7,) offsets

    def E(self, i: int, x: np.ndarray) -> np.ndarray:
        e = self.e_scale[i] * x + self.e_a2[i] * x**2 + self.e_a3[i] * x**3
        if JOINTS[i] == FX["e_step"]["joint"]:
            s = FX["e_step"]
            e = e + s["amplitude"] / (
                1.0 + np.exp(-(x - self.step_centre) / s["width"])
            )
        return e


def draw_truth(
    seed: int,
    w=None,
    g_arm6=None,
    load: bool = True,
    e_on: bool = True,
    e_arg: str = "q_abs",
) -> Truth:
    """One truth per seed. The draws are fixed; the options only overwrite
    (``w`` a width in rad for every joint, ``load`` False sets ``G = kappa = 0``,
    ``e_on`` False sets ``E = 0``, ``g_arm6`` a preload in rad on arm_6)."""
    rng = np.random.default_rng(seed)
    centre = np.array([FX["w_center_mrad"][j] for j in JOINTS]) * 1e-3
    spread = FX["w_spread"]
    W = centre * (1.0 + spread * rng.uniform(-1, 1, 7))
    G = rng.normal(0.0, FX["g_sd_mrad"] * 1e-3, 7)
    kappa = rng.normal(0.0, FX["kappa_sd_mrad_per_nm"] * 1e-3, 7)
    sign = rng.choice([-1.0, 1.0], 7)
    scale = FX["e_scale"] * sign
    sd = FX["e_smooth_sd_mrad"] * 1e-3
    a2, a3 = rng.normal(0.0, sd, 7), rng.normal(0.0, sd, 7)
    lo, hi = FX["e_step"]["centre"]
    centre_step = float(rng.uniform(lo, hi))
    c = {
        "training": rng.normal(0.0, FX["c_sd_mrad"] * 1e-3, 7),
        "validation": rng.normal(0.0, FX["c_sd_mrad"] * 1e-3, 7),
    }
    if w is not None:
        W = np.full(7, w) if np.isscalar(w) else np.asarray(w)
    if not load:
        G, kappa = np.zeros(7), np.zeros(7)
    if g_arm6 is not None:
        G = G.copy()
        kappa = kappa.copy()
        G[:] = 0.0
        kappa[:] = 0.0
        G[JOINTS.index(FX["f5"]["true_joint"])] = g_arm6
    if not e_on:
        scale, a2, a3 = np.zeros(7), np.zeros(7), np.zeros(7)
        centre_step = 0.0
    t = Truth(seed, W, G, kappa, scale, a2, a3, centre_step, c)
    if not e_on:
        t.E = lambda i, x: np.zeros_like(x)  # type: ignore[assignment]
    return t


def solve_q_abs(truth: Truth, i: int, rhs: np.ndarray) -> np.ndarray:
    """``x`` with ``x + E(x) = rhs``: bisection (``x + E`` is increasing)."""
    lo, hi = rhs - 1.0, rhs + 1.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        up = mid + truth.E(i, mid) > rhs
        hi = np.where(up, mid, hi)
        lo = np.where(up, lo, mid)
    return 0.5 * (lo + hi)


def play(q_rel: np.ndarray, width: float, h0: float) -> np.ndarray:
    """Backlash state ``b`` in ``[-W/2, W/2]`` of a play operator driven by
    ``q_rel``: ``d = q_rel - q_abs`` moves by ``W`` over the ``W`` of motion
    that follows a reversal (an instantaneous flip would be a jump of ``W`` in
    ``d`` with ``q_rel`` still, which is what the jump detector looks for)."""
    half = 0.5 * width
    out = np.empty(q_rel.size)
    b = half * h0
    prev = q_rel[0]
    for k in range(q_rel.size):
        b = min(half, max(-half, b + q_rel[k] - prev))
        prev = q_rel[k]
        out[k] = b
    return out


class Fixture:
    """The real configuration of both runs (regressor columns only) and the
    simulated measurements on it."""

    def __init__(self):
        sessions = bm.load_sessions()
        roles = bm.role_sets(sessions)
        self.runs = {
            r: bm.read_regressors(roles[r][0]) for r in ("training", "validation")
        }
        self.urdf = SETTINGS["urdf"]["backlash_days"]
        self.skel: Dict[str, Dict[str, bm.JointData]] = {}
        for role, run in self.runs.items():
            self.skel[role] = {}
            tau = bm.gravity_torque(run, self.urdf)
            for i, j in enumerate(JOINTS):
                k = bm.CONFIG_JOINTS.index(j)
                h, valid = bm.direction_state(run.t, run.v[:, k])
                self.skel[role][j] = bm.JointData(
                    j,
                    run.t,
                    run.q[:, k],
                    run.q[:, k],
                    h,
                    tau[:, i],
                    np.zeros(run.n, int),
                    valid,
                )
        self._unit: Dict = {}
        self.sigma = np.array([NOISE["sigma_d_mrad"][j] for j in JOINTS]) * 1e-3
        self.rho = np.array([NOISE["rho"][j] for j in JOINTS])
        self.step = np.array([NOISE["quantization_mrad"][j] for j in JOINTS]) * 1e-3

    # --- noise
    def unit_noise(self, kind: str, seed: int, role: str, i: int) -> np.ndarray:
        """Unit-variance noise of one joint and run: AR(1) (stationary) for
        ``ar1``, white for the other kinds. Drawn once per (kind, seed)."""
        key = (kind, seed)
        if key not in self._unit:
            rng = stream(kind, seed)
            out = {}
            for r in ("training", "validation"):
                n = self.runs[r].n
                z = rng.standard_normal((n, 7))
                if kind == "ar1":
                    for k in range(7):
                        rho = self.rho[k]
                        z[:, k] = lfilter([np.sqrt(1 - rho**2)], [1.0, -rho], z[:, k])
                out[r] = z
            self._unit[key] = out
        return self._unit[key][role][:, i]

    def measure(
        self,
        truth: Truth,
        role: str,
        i: int,
        level: float,
        kind: str = "ar1",
        jumps: Optional[List] = None,
    ) -> dict:
        """Noise-free and observed ``q_abs`` of one joint on one run.

        ``kind`` ``ar1``: ``d`` gets AR(1) noise of sd ``level sigma_d``;
        ``quantization``: ``q_abs`` is rounded to ``level`` times the step of the
        real encoder. ``jumps`` = [(index, size)] added to ``d`` from the index on.
        """
        sk = self.skel[role][JOINTS[i]]
        sg = bm.soft_sign(sk.tau)
        first = sk.h[np.argmax(sk.valid)]
        rhs = (
            sk.q_rel
            - truth.c[role][i]
            - play(sk.q_rel, truth.W[i], first)
            - truth.G[i] * sg
            - truth.kappa[i] * sk.tau
        )
        q_true = solve_q_abs(truth, i, rhs)
        d_true = sk.q_rel - q_true
        q_obs = q_true.copy()
        if kind == "ar1":
            q_obs = q_obs - level * self.sigma[i] * self.unit_noise(
                "ar1", truth.seed, role, i
            )
        elif kind == "quantization":
            step = level * self.step[i]
            q_obs = np.round(q_obs / step) * step
        else:
            raise ValueError(kind)
        for idx, size in jumps or []:
            q_obs[idx:] -= size
        return {"q_true": q_true, "d_true": d_true, "q_obs": q_obs}

    def joint_data(
        self, role: str, i: int, q_obs: np.ndarray, jumps: bool = True
    ) -> bm.JointData:
        sk = self.skel[role][JOINTS[i]]
        d = sk.q_rel - q_obs
        seg = (
            bm.segment_ids(sk.t.size, bm.jump_indices(d, sk.q_rel))
            if jumps
            else np.zeros(sk.t.size, int)
        )
        return bm.JointData(
            sk.joint, sk.t, sk.q_rel, q_obs, sk.h, sk.tau, seg, sk.valid
        )

    def cell_data(self, truth, level, kind="ar1"):
        """Training and validation ``JointData`` and noise-free truth, per joint."""
        out = {}
        for role in ("training", "validation"):
            out[role] = []
            for i in range(7):
                m = self.measure(truth, role, i, level, kind)
                out[role].append((self.joint_data(role, i, m["q_obs"]), m))
        return out


# ------------------------------------------------------------------ F1


def f1_cells(fx: Fixture) -> dict:
    """Identifiability of the M3 design on the real regressor columns of both
    runs (``q_rel`` stands in for ``q_abs`` in ``E``; no absolute encoder)."""
    crit = FX["f1"]
    out = {}
    for role, run in fx.runs.items():
        out[role] = {}
        for j in JOINTS:
            r = bm.identifiability(run, j, fx.urdf)
            r["n5_sign_fraction"] = bm.gravity_sign_fraction(run, j)
            r["pass"] = bool(r["rank"] == r["columns"] and r["cond"] < crit["cond_max"])
            out[role][j] = r
    return out


# ------------------------------------------------------------------ F2


def _z(fit: bm.Fit, key: str, i: int, truth_value: float) -> float:
    j = fit.index[key][i]
    return float((fit.coef[j] - truth_value) / fit.se[j])


def f2_cell(fx: Fixture, seed: int, level: float, kind: str = "ar1") -> dict:
    """Free M3 fit on the simulated training run; z-scores of W, G, kappa."""
    truth = draw_truth(seed)
    rng = np.random.default_rng(SETTINGS["selection"]["bootstrap"]["seed"])
    z = {"W": [], "G": [], "kappa": []}
    per = {}
    for i, j in enumerate(JOINTS):
        m = fx.measure(truth, "training", i, level, kind)
        jd = fx.joint_data("training", i, m["q_obs"])
        knots = bm.make_knots(jd.q_abs)
        f = bm.fit_groups(jd, knots, ("E", "W", "L"), bootstrap=True, rng=rng)
        row = {
            "W_err_mrad": (f.W - truth.W[i]) * 1e3,
            "W_se_mrad": f.se[f.index["W"][0]] * 1e3,
            "cond": f.cond,
        }
        z["W"].append(_z(f, "W", 0, truth.W[i]))
        if f.load_use[0]:
            z["G"].append(_z(f, "L", 0, truth.G[i]))
            row["G_z"] = z["G"][-1]
        if f.load_use[1]:
            z["kappa"].append(_z(f, "L", 1, truth.kappa[i]))
            row["kappa_z"] = z["kappa"][-1]
        row["W_z"] = z["W"][-1]
        per[j] = row
    return {"seed": seed, "z": z, "joints": per}


def z_stats(cells) -> dict:
    allz = np.concatenate([np.asarray(v) for c in cells for v in c["z"].values()])
    out = {
        "rms": float(np.sqrt(np.mean(allz**2))),
        "mean": float(np.mean(allz)),
        "n": int(allz.size),
    }
    for key in ("W", "G", "kappa"):
        zz = np.concatenate([np.asarray(c["z"][key]) for c in cells])
        out[f"rms_{key}"] = float(np.sqrt(np.mean(zz**2)))
        out[f"mean_{key}"] = float(np.mean(zz))
    return out


# ------------------------------------------------------------------ F3


def f3_cell(fx: Fixture, seed: int, level: float) -> dict:
    """arm_5 with the E step: W = 0 and W = 12 mrad, E on q_abs and on q_rel."""
    i = JOINTS.index(FX["e_step"]["joint"])
    out = {"seed": seed}
    for label, w in (("zero", 0.0), ("width", FX["f3"]["w_nonzero_mrad"] * 1e-3)):
        truth = draw_truth(seed, w=w, load=False)
        m = fx.measure(truth, "training", i, level)
        jd = fx.joint_data("training", i, m["q_obs"])
        rng = np.random.default_rng(SETTINGS["selection"]["bootstrap"]["seed"])
        for arg in ("q_abs", "q_rel"):
            jd_arg = (
                jd
                if arg == "q_abs"
                else bm.JointData(
                    jd.joint, jd.t, jd.q_rel, jd.q_rel, jd.h, jd.tau, jd.seg, jd.valid
                )
            )
            # the target must stay d = q_rel - q_abs: only the regressor argument changes
            knots = bm.make_knots(jd_arg.q_abs)
            r = jd.rows()
            ra = jd_arg.rows()
            X, index = bm.design(ra, knots, ("E", "W"))
            pen = bm.e_penalty(index, knots, X.shape[1])
            coef, s, keep, rank, cond = bm.solve(X, r.d, pen)
            se = bm.bootstrap_se(r, X, r.d, keep, s, rng=rng, pen=pen)
            jw = index["W"][0]
            out[f"{label}_{arg}"] = {
                "W_hat_mrad": float(coef[jw] * 1e3),
                "W_se_mrad": float(se[jw] * 1e3),
                "W_true_mrad": w * 1e3,
            }
    return out


def grade_f3(cells) -> dict:
    c = FX["f3"]
    zero_ok = all(
        abs(x["zero_q_abs"]["W_hat_mrad"])
        < c["w_zero_se"] * x["zero_q_abs"]["W_se_mrad"]
        and abs(x["zero_q_abs"]["W_hat_mrad"]) < c["w_zero_mrad"]
        for x in cells
    )
    z_ok = all(
        abs(x["width_q_abs"]["W_hat_mrad"] - x["width_q_abs"]["W_true_mrad"])
        < c["w_width_se"] * x["width_q_abs"]["W_se_mrad"]
        and abs(x["width_q_abs"]["W_hat_mrad"] - x["width_q_abs"]["W_true_mrad"])
        < c["w_width_rel"] * x["width_q_abs"]["W_true_mrad"]
        for x in cells
    )
    # q_rel must fail: some seed has a wrong width or a spurious one
    rel_fails = any(
        abs(x["width_q_rel"]["W_hat_mrad"] - x["width_q_rel"]["W_true_mrad"])
        >= c["w_width_se"] * x["width_q_rel"]["W_se_mrad"]
        or abs(x["zero_q_rel"]["W_hat_mrad"])
        >= c["w_zero_se"] * x["zero_q_rel"]["W_se_mrad"]
        for x in cells
    )
    return {"zero": bool(zero_ok), "width": bool(z_ok), "q_rel_fails": bool(rel_fails)}


# ------------------------------------------------------------------ F4


def _in_range(jd_val: bm.JointData, knots: np.ndarray) -> np.ndarray:
    return (jd_val.q_abs >= knots[0]) & (jd_val.q_abs <= knots[-1])


def f4_cell(fx: Fixture, seed: int, level: float) -> dict:
    """Train on the simulated 07-24, re-anchor on the simulated 03-09, compare to
    the noise-free truth. Scored on the samples whose ``q_abs`` lies inside the
    training knots (E is not extrapolated); the outside fraction is reported."""
    truth = draw_truth(seed)
    null = draw_truth(seed, load=False, e_on=False)
    out = {"seed": seed, "joints": {}}
    rng = np.random.default_rng(SETTINGS["selection"]["bootstrap"]["seed"])
    for i, j in enumerate(JOINTS):
        row = {}
        # full truth: selected model against the noise-free truth
        mt = fx.measure(truth, "training", i, level)
        mv = fx.measure(truth, "validation", i, level)
        jt = fx.joint_data("training", i, mt["q_obs"])
        jv = fx.joint_data("validation", i, mv["q_obs"])
        knots = bm.make_knots(jt.q_abs)
        sel = bm.select_groups(jt, knots, rng=rng)
        f = bm.fit_groups(jt, knots, sel["groups"])
        inside = _in_range(jv, knots) & jv.valid
        pred = bm.predict(f, jv)
        row["groups"] = "".join(sel["groups"])
        # oracle: every group, to separate the cost of the selection from E coverage
        fo = bm.fit_groups(jt, knots, ("E", "W", "L"))
        row["ratio_m3"] = _rmse_reanch(bm.predict(fo, jv), mv["d_true"], inside) / (
            level * fx.sigma[i]
        )
        # the offset of the truth is re-anchored too: compare after removing the mean
        row["rmse_mrad"] = _rmse_reanch(pred, mv["d_true"], inside) * 1e3
        row["ratio"] = _rmse_reanch(pred, mv["d_true"], inside) / (level * fx.sigma[i])
        row["outside_fraction"] = float(
            np.mean(~_in_range(jv, knots) & jv.valid) / max(np.mean(jv.valid), 1e-9)
        )
        # null truth (direction-only): g = RMSE(M*) - RMSE(B1)
        nt = fx.measure(null, "training", i, level)
        nv = fx.measure(null, "validation", i, level)
        jt0 = fx.joint_data("training", i, nt["q_obs"])
        jv0 = fx.joint_data("validation", i, nv["q_obs"])
        kn0 = bm.make_knots(jt0.q_abs)
        sel0 = bm.select_groups(jt0, kn0, rng=rng)
        f0 = bm.fit_groups(jt0, kn0, sel0["groups"])
        b1 = bm.fit_b1(jt0)
        e_m = _rmse_reanch(bm.predict(f0, jv0), nv["d_true"], jv0.valid)
        e_b = _rmse_reanch(bm.predict(b1, jv0), nv["d_true"], jv0.valid)
        row["g_selected_mrad"] = (e_m - e_b) * 1e3
        # primary M* = all groups (maintainer decision 2026-10-22, before any real fit)
        f0a = bm.fit_groups(jt0, kn0, ("E", "W", "L"))
        e_a = _rmse_reanch(bm.predict(f0a, jv0), nv["d_true"], jv0.valid)
        row["g_mrad"] = (e_a - e_b) * 1e3
        row["null_groups"] = "".join(sel0["groups"])
        # V2 width band: W from E+W fits on both simulated sessions (truth with E)
        ft = bm.fit_groups(jt, knots, ("E", "W"))
        fv = bm.fit_groups(jv, bm.make_knots(jv.q_abs), ("E", "W"))
        row["W_train_mrad"] = ft.W * 1e3
        row["W_val_mrad"] = fv.W * 1e3
        row["W_true_mrad"] = truth.W[i] * 1e3
        out["joints"][j] = row
    return out


def _rmse_reanch(pred, d_true, mask) -> float:
    """RMSE against the noise-free truth after removing the common offset."""
    r = (pred - d_true)[mask]
    return float(np.sqrt(np.mean((r - r.mean()) ** 2)))


def summarize_f4(cells) -> dict:
    c = FX["f4"]
    out = {"joints": {}}
    for j in JOINTS:
        rows = [x["joints"][j] for x in cells]
        g = np.array([r["g_mrad"] for r in rows])
        gs = np.array([r["g_selected_mrad"] for r in rows])
        dw = np.array([r["W_train_mrad"] - r["W_val_mrad"] for r in rows])
        out["joints"][j] = {
            "ratio": float(np.mean([r["ratio"] for r in rows])),
            "ratio_m3": float(np.mean([r["ratio_m3"] for r in rows])),
            "outside_fraction": float(np.mean([r["outside_fraction"] for r in rows])),
            "band_v1_mrad": float(
                abs(g.mean()) + c["band_v1_sd_factor"] * g.std(ddof=1)
            ),
            "band_v1_selected_mrad": float(
                abs(gs.mean()) + c["band_v1_sd_factor"] * gs.std(ddof=1)
            ),
            "band_w_mrad": float(c["band_w_factor"] * np.sqrt(np.mean(dw**2))),
            "groups": [r["groups"] for r in rows],
        }
    return out


# ------------------------------------------------------------------ F5


def f5_cell(fx: Fixture, seed: int, level: float) -> dict:
    """Selection with G = kappa = 0 and with G = 10 mrad on arm_6."""
    rng = np.random.default_rng(SETTINGS["selection"]["bootstrap"]["seed"])
    out = {"seed": seed, "none": {}, "true": {}}
    for label, truth in (
        ("none", draw_truth(seed, load=False)),
        ("true", draw_truth(seed, g_arm6=FX["f5"]["true_g_mrad"] * 1e-3)),
    ):
        for i, j in enumerate(JOINTS):
            if label == "true" and j != FX["f5"]["true_joint"]:
                continue
            m = fx.measure(truth, "training", i, level)
            jd = fx.joint_data("training", i, m["q_obs"])
            sel = bm.select_groups(jd, bm.make_knots(jd.q_abs), rng=rng)
            out[label][j] = {
                "admitted": "L" in sel["groups"],
                "groups": "".join(sel["groups"]),
                "load_usable": bool(any(bm.load_usable(jd))),
            }
    return out


def summarize_f5(cells) -> dict:
    c = FX["f5"]
    none = {j: sum(x["none"][j]["admitted"] for x in cells) for j in JOINTS}
    usable = {j: cells[0]["none"][j]["load_usable"] for j in JOINTS}
    true = sum(x["true"][c["true_joint"]]["admitted"] for x in cells)
    return {
        "false_admit": none,
        "load_usable": usable,
        "true_admit": true,
        "pass": bool(
            all(n <= c["false_admit_max"] for j, n in none.items() if usable[j])
            and true >= c["true_admit_min"]
        ),
    }


# ------------------------------------------------------------------ F6


def f6_cell(fx: Fixture, seed: int, level: float) -> dict:
    """Injected jumps (1-3 of 30-100 mrad per joint) against the detector."""
    truth = draw_truth(seed)
    rng = stream("jumps", seed)
    found = missed = false = total = 0
    lo, hi = FX["jump_size_mrad"]
    for i, j in enumerate(JOINTS):
        sk = fx.skel["training"][j]
        quiet = np.flatnonzero(
            sk.valid & (np.abs(np.diff(sk.q_rel, prepend=sk.q_rel[0])) < 0.0015)
        )
        quiet = quiet[(quiet > 500) & (quiet < sk.t.size - 500)]
        n = int(rng.integers(FX["jump_count"][0], FX["jump_count"][1] + 1))
        idx = []
        while len(idx) < n:
            k = int(rng.choice(quiet))
            if all(abs(k - m) > 1000 for m in idx):
                idx.append(k)
        idx.sort()
        size = rng.uniform(lo, hi, n) * 1e-3 * rng.choice([-1.0, 1.0], n)
        jumps = list(zip(idx, size))
        m = fx.measure(truth, "training", i, level, jumps=jumps)
        d = sk.q_rel - m["q_obs"]
        det = bm.jump_indices(d, sk.q_rel)
        hit = [any(abs(int(dd) - k) <= 1 for dd in det) for k in idx]
        found += sum(hit)
        missed += n - sum(hit)
        false += int(sum(not any(abs(int(dd) - k) <= 1 for k in idx) for dd in det))
        total += n
    return {
        "seed": seed,
        "injected": total,
        "found": found,
        "missed": missed,
        "false": false,
    }


# ------------------------------------------------------------------ driver


def run(seeds=SEEDS, levels=LEVELS, verbose=True) -> dict:
    fx = Fixture()
    out = {
        "seeds": list(seeds),
        "levels": list(levels),
        "used": USED,
        "grid": {},
        "f1": f1_cells(fx),
    }
    for lv in levels:
        if verbose:
            print(f"level {lv} x sigma_d", flush=True)
        f2 = [f2_cell(fx, s, lv) for s in seeds]
        f3 = [f3_cell(fx, s, lv) for s in seeds]
        f4 = [f4_cell(fx, s, lv) for s in seeds]
        f5 = [f5_cell(fx, s, lv) for s in seeds]
        f6 = [f6_cell(fx, s, lv) for s in seeds]
        cell = {
            "f2": z_stats(f2),
            "f3": {"cells": f3, "pass": grade_f3(f3)},
            "f4": summarize_f4(f4),
            "f5": summarize_f5(f5),
            "f6": {
                "injected": sum(c["injected"] for c in f6),
                "found": sum(c["found"] for c in f6),
                "false": sum(c["false"] for c in f6),
            },
            "cells": {"f2": f2, "f4": f4, "f5": f5, "f6": f6},
        }
        if lv == USED:
            f2q = [f2_cell(fx, s, lv, kind="quantization") for s in seeds]
            cell["f2q"] = z_stats(f2q)
        out["grid"][str(lv)] = cell
    return out


def grade(res: dict) -> dict:
    """Pass/fail of F1-F6 (F2 at every level, F3-F6 at the used level; F6 false
    detections at the top level)."""
    f1 = all(r["pass"] for role in res["f1"].values() for r in role.values())

    def f2_ok(s):
        c = FX["f2"]
        return (
            c["z_rms"][0] <= s["rms"] <= c["z_rms"][1]
            and abs(s["mean"]) < c["z_mean_max"]
        )

    used = res["grid"][str(res["used"])]
    top = res["grid"][str(max(res["levels"]))]
    ratio_max = FX["f4"]["rmse_ratio_max"]
    n6 = [j for j, v in used["f4"]["joints"].items() if v["ratio_m3"] >= ratio_max]
    return {
        "F1": bool(f1),
        "F2": all(f2_ok(c["f2"]) for c in res["grid"].values()),
        "F2q": bool(f2_ok(used["f2q"])),
        "F3": all(used["f3"]["pass"].values()),
        # primary M* = all groups; N6 joints (ratio >= limit) are not answerable
        "F4": all(
            v["ratio_m3"] < ratio_max
            for j, v in used["f4"]["joints"].items()
            if j not in n6
        ),
        "F4_selected": all(
            v["ratio"] < ratio_max for v in used["f4"]["joints"].values()
        ),
        "N6": n6,
        "F5": used["f5"]["pass"],
        "F6": bool(
            all(c["f6"]["found"] == c["f6"]["injected"] for c in res["grid"].values())
            and top["f6"]["false"] <= FX["f6"]["false_max"]
        ),
    }


def golden(res: dict) -> dict:
    """What the tests and the note quote: summaries per level, seed 0 in full."""
    levels = {}
    for lv, c in res["grid"].items():
        levels[lv] = {k: c[k] for k in ("f2", "f4", "f5", "f6")}
        levels[lv]["f3"] = c["f3"]
        if "f2q" in c:
            levels[lv]["f2q"] = c["f2q"]
    used = res["grid"][str(res["used"])]["cells"]
    return {
        "used": res["used"],
        "seeds": res["seeds"],
        "noise": {
            k: SETTINGS["noise"][k]
            for k in ("sigma_d_mrad", "rho", "quantization_mrad")
        },
        "criteria": grade(res),
        "f1": res["f1"],
        "levels": levels,
        "seed0": {"f2": used["f2"][0], "f4": used["f4"][0], "f6": used["f6"][0]},
    }


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
        args.out.write_text(json.dumps(res, indent=1, default=float) + "\n")
    if args.golden:
        args.golden.write_text(
            json.dumps(golden(res), indent=1, sort_keys=True, default=float) + "\n"
        )


if __name__ == "__main__":
    main()
