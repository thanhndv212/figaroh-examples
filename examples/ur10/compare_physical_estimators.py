"""Compare physically consistent dynamic estimators on the UR10 truth fixture (#22).

Report-only comparison on the frozen fixture (#21, protocol v1) of four
estimators that optimise *different* objectives:

* ``base_ols``              - base-parameter least squares (no physical constraint);
* ``exact_reconstruction``  - closest-to-prior theta with M theta = phi_ols and a
  pseudo-inertia LMI per link (core #59 comparator);
* ``direct_effort_fit``     - regularised effort fit with the same LMIs, no
  equality (comparator);
* ``per_link_projection``   - nullspace representative of phi_ols, then an
  independent LMI projection per link (comparator).

Frozen-extra (friction, actuator inertia fixed at their true value, zero) and
joint-extra (extras estimated; base OLS only, the comparator has no free
extras) experiments are separate. Training and held-out results, and solver
convergence and physical feasibility, are reported separately. For each noise
level ``diagnose_exact`` classifies why exact reconstruction fails, if it does.

The comparator is the private module
``figaroh.identification._physical_comparator`` (core feat/59-physical-comparator).
Nonlinear (log-Cholesky) candidates are not run here (follow-up).

    python compare_physical_estimators.py --output <json>
    python compare_physical_estimators.py --noise none --seeds 1 --output <json>
"""

from __future__ import annotations

import argparse
import json
import multiprocessing as mp
import os
import platform
import subprocess
import sys
import time
from pathlib import Path

# single-threaded BLAS: the small SDP/QR solves are far slower oversubscribed
for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ.setdefault(_v, "1")

import numpy as np  # noqa: E402

HERE = Path(__file__).parent
project_root = HERE.parents[1]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from examples.ur10 import identification_truth as it  # noqa: E402

UNITS = {
    "effort": "N.m (joint side, revolute joints)",
    "mass": "kg",
    "first_moment": "kg.m",
    "inertia": "kg.m^2 (about the joint frame)",
    "base_error": "N.m (error x RMS of its training regressor column)",
    "extras": "fv N.m.s/rad, fs N.m, Ia kg.m^2",
}
EXTRA_KINDS = ("fv", "fs", "Ia")
CERT_TOL = 1e-7  # |s*| below this is not a certificate either way
DEFAULT_OUT_DIR = project_root / "docs" / "development" / "results"
CONFIG = HERE / "config" / "ur10_unified_config.yaml"
P10 = ["m", "mx", "my", "mz", "Ixx", "Ixy", "Iyy", "Ixz", "Iyz", "Izz"]


# --- provenance -----------------------------------------------------------


def _git(repo: Path, *args: str) -> str:
    try:
        return subprocess.run(
            ["git", "-C", str(repo), *args],
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip()
    except Exception:
        return "unknown"


def revision(repo: Path) -> dict:
    return {
        "commit": _git(repo, "rev-parse", "HEAD"),
        "branch": _git(repo, "rev-parse", "--abbrev-ref", "HEAD"),
        "dirty": bool(_git(repo, "status", "--porcelain", "--untracked-files=no")),
    }


def provenance(comparator) -> dict:
    import cvxopt
    import picos

    core = Path(comparator.__file__).resolve().parents[3]
    fx = it.FIXTURE_DIR
    sha = it.sha256
    return {
        "examples": revision(project_root),
        "core": revision(core),
        "comparator_module": str(Path(comparator.__file__).resolve()),
        "comparator_sha256": sha(Path(comparator.__file__)),
        "script_sha256": sha(Path(__file__)),
        "pinocchio": it.pin.__version__,
        "pinocchio_profile": "pin" + "".join(it.pin.__version__.split(".")[:2]),
        "python": platform.python_version(),
        "numpy": np.__version__,
        "picos": picos.__version__,
        "cvxopt": cvxopt.__version__,
        "platform": platform.platform(),
        "hashes": {
            "raw": {n: sha(fx / n) for n in ("train.csv", "validation.csv")},
            "config": {
                "protocol.yaml": sha(fx / "protocol.yaml"),
                "manifest.json": sha(fx / "manifest.json"),
                "train_waypoints.json": sha(fx / "train_waypoints.json"),
                "ur10_unified_config.yaml": sha(CONFIG),
            },
            "model": {
                "ur10_robot.urdf (nominal prior)": sha(it.URDF),
                "ur10_truth.urdf": sha(fx / "ur10_truth.urdf"),
                "truth_parameters.csv": sha(fx / "truth_parameters.csv"),
            },
        },
        "fixture_manifest_check": it.check_fixture(),
    }


# --- problem assembly -----------------------------------------------------


def extra_names() -> list:
    return [f"{k}_{j}" for k in EXTRA_KINDS for j in it.JOINTS]


def regressors(split_data: dict, model) -> np.ndarray:
    """Standard regressor plus fv, fs, Ia columns; joint-major stacking."""
    cfg = dict(it.IDENTIF_CONFIG, has_friction=True, has_actuator_inertia=True)
    return it.build_regressor_basic(
        it._Robot(model), split_data["q"], split_data["dq"], split_data["ddq"], cfg
    )


def stack(tau: np.ndarray) -> np.ndarray:
    return tau.T.ravel()


def per_joint_rmse(res: np.ndarray) -> np.ndarray:
    return np.sqrt(np.mean(res.reshape(len(it.JOINTS), -1) ** 2, axis=1))


def link_report(theta: np.ndarray, truth: np.ndarray, prior: np.ndarray) -> dict:
    """Per-link standard-parameter change vs truth and vs the nominal prior."""
    out = {}
    for k, j in enumerate(it.JOINTS):
        s = slice(10 * k, 10 * k + 10)
        t, tr, pr = theta[s], truth[s], prior[s]
        out[j] = {
            "mass_kg": float(t[0]),
            "mass_err_pct": float(100 * (t[0] - tr[0]) / tr[0]),
            "first_moment_err_kgm": float(np.linalg.norm(t[1:4] - tr[1:4])),
            "inertia_err_rel": float(
                np.linalg.norm(t[4:] - tr[4:]) / np.linalg.norm(tr[4:])
            ),
            "change_vs_prior_norm": float(np.linalg.norm(t - pr)),
        }
    return out


class Case:
    """One (derivatives, noise, seed, weighting) problem and its held-out data."""

    def __init__(self, derivatives, noise, seed, val_seed, weighting, tp, proto):
        self.key = dict(
            derivatives=derivatives, noise=noise, seed=seed, val_seed=val_seed,
            weighting=weighting,
        )
        model = it.nominal_model()
        train = it.load_split("train", derivatives, noise, seed)
        val = it.load_split("validation", derivatives, noise, val_seed)
        self.tp = tp
        self.names = list(tp.parameter)
        self.truth = tp.truth.to_numpy()
        self.prior = tp.nominal.to_numpy()
        self.Y_std = regressors(train, model)
        self.Yv_std = regressors(val, model)
        self.tau = stack(train["tau"])
        self.tauv_noisy = stack(val["tau"])
        self.tauv_true = stack(val["tau_true"])
        n = len(train["t"])
        scale = np.array(list(proto["scaling"]["effort_nm_per_joint"].values()))
        self.effort_scale = scale
        self.row_weight = (
            None if weighting == "none" else np.repeat(1.0 / scale, n)
        )
        self.all_names = self.names + extra_names()
        # noise-free truth: extras are zero
        self.truth_all = np.concatenate([self.truth, np.zeros(18)])
        self.prior_all = np.concatenate([self.prior, np.zeros(18)])

    def problem(self, pcmp):
        extras = pcmp.FixedExtras(
            tuple(extra_names()), tuple(0.0 for _ in extra_names()), "truth"
        )
        return pcmp.build_problem(
            self.Y_std,
            self.tau,
            self.all_names,
            it.JOINTS,
            prior=self.prior_all,
            theta_truth=self.truth_all,
            extras=extras,
            row_weight=self.row_weight,
        )


# --- evaluation -----------------------------------------------------------


_PROTOCOL_BASIS = {}


def protocol_basis(pcmp, tp, proto) -> np.ndarray:
    """Base-parameter map (36 x 60) of the noise-free training regressor.

    The comparator's QR picks base columns from the regressor it is given, and
    with noise it can pick others than the protocol's; base errors are always
    expressed in the protocol's basis so that they match
    ``identification_truth.ols_case``.
    """
    if "M" not in _PROTOCOL_BASIS:
        d = it.load_split("train", "analytic", "none")
        Y = regressors(d, it.nominal_model())
        p = pcmp.build_problem(
            Y, stack(d["tau"]), list(tp.parameter) + extra_names(), it.JOINTS,
            prior=np.concatenate([tp.nominal.to_numpy(), np.zeros(18)]),
            extras=pcmp.FixedExtras(
                tuple(extra_names()), tuple(0.0 for _ in extra_names()), "truth"
            ),
        )
        kept = [i for i in range(60) if i not in proto["rank"]["eliminated_indices"]]
        assert list(p.base_indices) == [kept[i] for i in proto["rank"]["base_indices"]]
        _PROTOCOL_BASIS["M"] = p.M_full
    return _PROTOCOL_BASIS["M"]


def evaluate(case, p, proto, pcmp, *, theta=None, phi=None) -> dict:
    """Training/held-out effort, base error and per-link change."""
    b = list(p.base_indices)
    Yv = case.Yv_std[:, :60]
    physical = theta is not None
    if physical:
        pred_tr = p.Y @ theta
        pred_v = Yv @ theta
        phi = p.M_full @ theta
    else:
        pred_tr = p.Y[:, b] @ phi
        pred_v = Yv[:, b] @ phi
        # base map rows span the identifiable space, so any representative
        # with M theta = phi gives the same protocol base parameters
        theta = np.linalg.pinv(p.M_full) @ phi
    Mp = protocol_basis(pcmp, case.tp, proto)
    scale = np.array(list(proto["scaling"]["base_column_rms"].values()))
    base_err = (Mp @ (theta - case.truth)) * scale
    val_rmse = per_joint_rmse(pred_v - case.tauv_true)
    out = {
        "train": {
            "rmse_vs_noisy_effort_nm": dict(
                zip(it.JOINTS, per_joint_rmse(pred_tr - case.tau).tolist())
            )
        },
        "heldout": {
            "rmse_vs_true_effort_nm": dict(zip(it.JOINTS, val_rmse.tolist())),
            "nrmse_pct": dict(
                zip(it.JOINTS, (100 * val_rmse / case.effort_scale).tolist())
            ),
            "rmse_vs_noisy_effort_nm": dict(
                zip(it.JOINTS, per_joint_rmse(pred_v - case.tauv_noisy).tolist())
            ),
        },
        "base_error": {
            "rms_nm": float(np.sqrt(np.mean(base_err**2))),
            "max_nm": float(np.abs(base_err).max()),
            "n_base": len(b),
        },
        "base_change_vs_ols_rel": float(
            np.linalg.norm(phi - p.phi_ols) / np.linalg.norm(p.phi_ols)
        ),
    }
    if physical:
        out["links"] = link_report(theta, case.truth, case.prior)
        out["standard_error_rel_to_truth"] = float(
            np.linalg.norm(theta - case.truth) / np.linalg.norm(case.truth)
        )
    return out


def record_fields(rec) -> dict:
    """Solver convergence and independent feasibility, kept apart."""
    d = rec.as_dict()
    return {
        "convergence": {
            "solver": d["solver"],
            "solver_status": d["solver_status"],
            "picos_status": d["picos_status"],
            "exception": d["exception"],
            "runtime_s": d["runtime_s"],
            "objective_value": d["objective_value"],
        },
        "feasibility": {
            "all_links_ok": bool(d["feasibility"])
            and all(v["ok"] for v in d["feasibility"].values()),
            "min_eig_min": (
                min(v["min_eig"] for v in d["feasibility"].values())
                if d["feasibility"]
                else None
            ),
            "per_link": d["feasibility"],
        },
        "base_residual": d["base_residual"],
        "accepted": d["accepted"],
        "fallback_used": d["fallback_used"],
        "inputs_hash": d["inputs_hash"],
    }


def phase1_certificate(p, pcmp, solver) -> dict:
    """Is {M theta = phi_ols, per-link LMIs} feasible? Phase-I, no mass bounds.

    ``infeasible`` only when the phase-I solve is optimal with s* < -CERT_TOL;
    a failed solve is ``solver_failure``, never infeasibility.
    """
    raw = pcmp._solve_core(p, "phase1", solver=solver, bounds=False)
    s_star = raw.get("s_star")
    if raw["status"] != "optimal" or s_star is None:
        label = "solver_failure"
    elif s_star < -CERT_TOL:
        label = "infeasible"
    elif s_star > CERT_TOL:
        label = "feasible"
    else:
        label = "marginal"
    return {"s_star": s_star, "label": label, "status": raw["status"],
            "exception": raw.get("exception")}


def run_methods(case, pcmp, proto, solver) -> dict:
    p = case.problem(pcmp)
    methods = {}
    ols = evaluate(case, p, proto, pcmp, phi=p.phi_ols)
    ols["convergence"] = {"solver": "lstsq", "solver_status": "optimal"}
    ols["feasibility"] = {"note": "base parameters only; no physical constraint"}
    methods["base_ols"] = ols
    solvers = {
        "exact_reconstruction": pcmp.solve_exact_reconstruction,
        "direct_effort_fit": pcmp.solve_direct_effort_fit,
        "per_link_projection": pcmp.solve_per_link_projection,
    }
    for name, fn in solvers.items():
        rec = fn(p, solver=solver)
        entry = record_fields(rec)
        if rec.theta is not None:
            entry.update(evaluate(case, p, proto, pcmp, theta=rec.theta))
        methods[name] = entry
    methods["exact_reconstruction"]["phase1"] = phase1_certificate(p, pcmp, solver)
    return p, methods


def joint_extra_ols(case, p, proto, pcmp) -> dict:
    """Base OLS with fv, fs, Ia estimated too (not run for the SDP methods)."""
    all_extras = extra_names()
    b = list(p.base_indices)
    w = np.ones(len(case.tau)) if case.row_weight is None else case.row_weight
    Yw = w[:, None] * case.Y_std
    # extras join the base set only when independent of it (greedy Gram-Schmidt):
    # Ia_j, e.g., is collinear with a base inertia parameter of joint j and is
    # then absorbed there, not estimated separately
    Q, _ = np.linalg.qr(Yw[:, b])
    kept_extras, absorbed = [], []
    for c in range(60, 78):
        r = Yw[:, c] - Q @ (Q.T @ Yw[:, c])
        if np.linalg.norm(r) > 1e-6 * np.linalg.norm(Yw[:, c]):
            Q = np.column_stack([Q, r / np.linalg.norm(r)])
            kept_extras.append(c)
        else:
            absorbed.append(c)
    cols = b + kept_extras
    sol, *_ = np.linalg.lstsq(Yw[:, cols], w * case.tau, rcond=None)
    phi, ex = sol[: len(b)], sol[len(b):]
    pred_v = case.Yv_std[:, cols] @ sol
    rmse = per_joint_rmse(pred_v - case.tauv_true)
    scale = np.array(list(proto["scaling"]["base_column_rms"].values()))
    theta_rep = np.linalg.pinv(p.M_full) @ phi
    base_err = (protocol_basis(pcmp, case.tp, proto) @ (theta_rep - case.truth)) * scale
    return {
        "heldout": {
            "rmse_vs_true_effort_nm": dict(zip(it.JOINTS, rmse.tolist())),
            "nrmse_pct": dict(
                zip(it.JOINTS, (100 * rmse / case.effort_scale).tolist())
            ),
        },
        "base_error": {
            "rms_nm": float(np.sqrt(np.mean(base_err**2))),
            "max_nm": float(np.abs(base_err).max()),
        },
        "extras_estimated": {
            all_extras[c - 60]: float(v) for c, v in zip(kept_extras, ex)
        },
        "extras_absorbed_in_base": [all_extras[c - 60] for c in absorbed],
        "extras_truth": 0.0,
        "extras_abs_max": float(np.abs(ex).max()) if len(ex) else 0.0,
    }


# --- exact-reconstruction diagnosis ---------------------------------------


def _d0_worker(conn, args, kwargs):
    try:
        from figaroh.identification.reconstruction import (
            reconstruct_full_parameters,
        )

        conn.send(("ok", reconstruct_full_parameters(*args, **kwargs)))
    except Exception as exc:  # reported by the caller
        conn.send(("error", f"{type(exc).__name__}: {exc}"))
    finally:
        conn.close()


def budgeted_reconstruct(timeout_s: float):
    """``reconstruct_full_parameters`` with a wall-time budget.

    The production SDP entry point has no time limit and, on this problem,
    can run for many minutes; the D0 stage runs it in a child process that is
    killed at the budget, which ``diagnose_exact`` then records as an error
    (``TimeoutError``), never as infeasibility.
    """

    def call(*args, **kwargs):
        ctx = mp.get_context("spawn")
        recv, send = ctx.Pipe(duplex=False)
        proc = ctx.Process(target=_d0_worker, args=(send, args, kwargs))
        proc.start()
        send.close()
        got = recv.recv() if recv.poll(timeout_s) else None
        if got is None:
            proc.kill()
            proc.join()
            raise TimeoutError(f"D0 exceeded the {timeout_s:.0f} s budget")
        proc.join()
        if got[0] == "error":
            raise RuntimeError(got[1])
        return got[1]

    return call


def classify_exact(case, p, records, main_exact) -> dict:
    """Numerical, formulation or genuine infeasibility, from D0..D3.

    Infeasible only with the phase-I certificate (s* < -CERT_TOL from an
    optimal phase-I solve); a solver error alone is never infeasibility.
    """
    by = {r.objective: r for r in records}
    d1 = by["D1:phase1"]
    s_star = d1.notes.get("s_star")
    phase1_ok = d1.solver_status == "optimal" and s_star is not None
    exact_ok = main_exact["accepted"]
    d0_ok = by["D0:reconstruct_full_parameters_sdp"].accepted
    variants = {
        n: by[f"D2:{n}"].accepted
        for n in ("schur_norm", "scaling_off", "params_r_only", "bounds_off")
    }
    if not phase1_ok:
        verdict, why = "indeterminate", "phase-I solve failed; no certificate either way"
    elif s_star < -CERT_TOL:
        verdict, why = (
            "genuine infeasibility",
            f"phase-I certificate s*={s_star:.3e} < 0: no physical theta satisfies "
            "M theta = phi_ols",
        )
    elif abs(s_star) <= CERT_TOL:
        verdict, why = (
            "indeterminate",
            f"phase-I s*={s_star:.3e} within +-{CERT_TOL}: marginal, no certificate",
        )
    elif exact_ok and d0_ok:
        verdict, why = (
            "no failure",
            f"phase-I feasible (s*={s_star:.3e}); direct exact solve and the "
            "production entry point both accepted",
        )
    elif exact_ok and not d0_ok:
        verdict, why = (
            "formulation",
            f"phase-I feasible (s*={s_star:.3e}) and direct exact solve accepted, "
            "but reconstruct_full_parameters(method='sdp') did not return an "
            "accepted solution (error or time budget). The comparator's scaled "
            "formulation solves the same constraint set in well under a second; "
            "which production difference (unscaled variables, weights, mass "
            "bound handling) matters was not isolated",
        )
    elif any(variants.values()):
        ok = [n for n, v in variants.items() if v]
        verdict, why = (
            "formulation",
            f"phase-I feasible (s*={s_star:.3e}); exact solve fails as posed but "
            f"succeeds when changing only: {ok}",
        )
    else:
        verdict, why = (
            "numerical",
            f"phase-I feasible (s*={s_star:.3e}) yet every exact variant fails: "
            "solver accuracy at the equality, not the constraint set",
        )
    return {
        "verdict": verdict,
        "reason": why,
        "phase1_s_star": s_star,
        "phase1_status": d1.solver_status,
        "certificate_tol": CERT_TOL,
        "direct_exact_accepted": exact_ok,
        "d0_production_accepted": d0_ok,
        "d0_status": by["D0:reconstruct_full_parameters_sdp"].solver_status,
        "d0_exception": by["D0:reconstruct_full_parameters_sdp"].exception,
        "d0_runtime_s": by["D0:reconstruct_full_parameters_sdp"].runtime_s,
        "d2_variants_accepted": variants,
        "d3_relaxed": {
            "solver_status": by["D3:relaxed_equality"].solver_status,
            "accepted": by["D3:relaxed_equality"].accepted,
            "base_residual": by["D3:relaxed_equality"].base_residual,
        },
        "records": [r.as_dict() for r in records],
    }


# --- driver ---------------------------------------------------------------


def summarize(cases: list) -> list:
    """Mean/worst over seeds per (derivatives, weighting, noise, method)."""
    groups = {}
    for c in cases:
        k = (c["derivatives"], c["weighting"], c["noise"])
        for m, r in c["frozen_extra"]["methods"].items():
            groups.setdefault(k + (m,), []).append(r)
    rows = []
    for (der, wt, noise, m), allr in groups.items():
        rs = [r for r in allr if "heldout" in r]
        row = {
            "derivatives": der, "weighting": wt, "noise": noise, "method": m,
            "cases": len(allr),
            "converged": sum(
                r["convergence"]["solver_status"] == "optimal" for r in allr
            ),
            "feasible": sum(
                r["feasibility"].get("all_links_ok", False) for r in allr
            ),
            "accepted": sum(bool(r.get("accepted", False)) for r in allr),
        }
        if m == "exact_reconstruction":
            labels = [r["phase1"]["label"] for r in allr]
            row["phase1"] = {k: labels.count(k) for k in sorted(set(labels))}
        if not rs:
            rows.append(row)
            continue
        nr = np.array([[r["heldout"]["nrmse_pct"][j] for j in it.JOINTS] for r in rs])
        rm = np.array(
            [[r["heldout"]["rmse_vs_true_effort_nm"][j] for j in it.JOINTS] for r in rs]
        )
        tr = np.array(
            [[r["train"]["rmse_vs_noisy_effort_nm"][j] for j in it.JOINTS] for r in rs]
        )
        eigs = [
            r["feasibility"]["min_eig_min"]
            for r in rs
            if r["feasibility"].get("min_eig_min") is not None
        ]
        row.update({
            "solved": len(rs),
            "train_rmse_nm_mean_per_joint": tr.mean(axis=0).tolist(),
            "heldout_rmse_nm_mean_per_joint": rm.mean(axis=0).tolist(),
            "heldout_nrmse_pct_mean_per_joint": nr.mean(axis=0).tolist(),
            "heldout_nrmse_pct_worst": float(nr.max()),
            "base_err_rms_nm_mean": float(
                np.mean([r["base_error"]["rms_nm"] for r in rs])
            ),
            "base_err_max_nm_worst": float(
                max(r["base_error"]["max_nm"] for r in rs)
            ),
            "min_eig_min": min(eigs) if eigs else None,
            "base_change_vs_ols_rel_mean": float(
                np.mean([r["base_change_vs_ols_rel"] for r in rs])
            ),
        })
        rows.append(row)
    return rows


def run(args) -> dict:
    from figaroh.identification import _physical_comparator as pcmp

    t0 = time.perf_counter()
    tp = it.truth_parameters()
    proto = it.protocol()
    out = {
        "issue": "figaroh-examples#22",
        "protocol_version": proto["protocol_version"],
        "units": UNITS,
        "joint_order": it.JOINTS,
        "provenance": provenance(pcmp),
        "settings": {
            "derivatives": args.derivatives,
            "noise": args.noise,
            "seeds": args.seeds,
            "weighting": args.weighting,
            "solver": args.solver,
            "d0_timeout_s": args.d0_timeout,
            "policy": repr(pcmp.PhysicalPolicy()),
            "objectives": {
                "base_ols": "min ||W_b phi - tau||_w (weighted rows if weighting)",
                "exact_reconstruction": "min ||D(theta-theta0)|| s.t. M theta = phi_ols, per-link LMI",
                "direct_effort_fit": "min ||R theta - Q'tau||^2 + lam||D(theta-theta0)||^2, per-link LMI",
                "per_link_projection": "nullspace representative of phi_ols, then per-link LMI projection",
            },
            "weighting_note": "'scaled' weights rows by 1/(noise-free training effort RMS of the joint); 'none' is the protocol's unweighted fit",
            "prior": "nominal URDF inertias (truth_parameters.csv 'nominal')",
            "extras": "frozen_extra: fv, fs, Ia fixed at 0 (= truth) via FixedExtras; "
            "joint_extra: base OLS only (comparator cannot estimate extras)",
            "not_run": ["nonlinear log-Cholesky candidates (follow-up)", "TX40", "TIAGo"],
        },
        "cases": [],
        "diagnose_exact": [],
    }
    for der in args.derivatives:
        for noise in args.noise:
            pairs = (
                [(None, None)]
                if noise == "none"
                else list(
                    zip(it.NOISE_SEEDS["train"], it.NOISE_SEEDS["validation"])
                )[: args.seeds]
            )
            for wt in args.weighting:
                for seed, vseed in pairs:
                    case = Case(der, noise, seed, vseed, wt, tp, proto)
                    p, methods = run_methods(case, pcmp, proto, args.solver)
                    out["cases"].append({
                        **case.key,
                        "base_rank": len(p.base_indices),
                        "frozen_extra": {"methods": methods},
                        "joint_extra": {"base_ols": joint_extra_ols(case, p, proto, pcmp)},
                    })
                    if seed is None or seed == pairs[0][0]:
                        orig = pcmp.reconstruct_full_parameters
                        pcmp.reconstruct_full_parameters = budgeted_reconstruct(
                            args.d0_timeout
                        )
                        try:
                            recs = pcmp.diagnose_exact(p, solver=args.solver)
                        finally:
                            pcmp.reconstruct_full_parameters = orig
                        diag = classify_exact(
                            case, p, recs,
                            methods["exact_reconstruction"],
                        )
                        diag.update(case.key)
                        out["diagnose_exact"].append(diag)
                    print(
                        f"{der:14s} {noise:5s} seed={seed} {wt:6s} "
                        + " ".join(
                            f"{m}:{r['heldout']['nrmse_pct']['wrist_3_joint']:.1f}%"
                            for m, r in methods.items()
                            if "heldout" in r
                        ),
                        flush=True,
                    )
    out["summary"] = summarize(out["cases"])
    out["runtime_s"] = time.perf_counter() - t0
    return out


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--derivatives", nargs="+", default=["analytic"],
                        choices=["analytic", "differentiated"])
    parser.add_argument("--noise", nargs="+", default=["none", "low", "high"],
                        choices=["none", "low", "high"])
    parser.add_argument("--seeds", type=int, default=5,
                        help="paired noise seeds per noisy level (1-5)")
    parser.add_argument("--weighting", nargs="+", default=["none", "scaled"],
                        choices=["none", "scaled"])
    parser.add_argument("--solver", default="cvxopt")
    parser.add_argument("--d0-timeout", type=float, default=30.0,
                        help="wall-time budget (s) of the production-entry-point "
                        "stage D0 of diagnose_exact")
    parser.add_argument("--output", type=Path,
                        help="JSON results (default docs/development/results/"
                        "ur10-physical-comparison-<profile>.json)")
    parser.add_argument("--overwrite", action="store_true",
                        help="replace an existing results file (never by default)")
    args = parser.parse_args(argv)
    if not 1 <= args.seeds <= 5:
        parser.error("--seeds must be 1..5")
    result = run(args)
    out = args.output or (
        DEFAULT_OUT_DIR
        / f"ur10-physical-comparison-{result['provenance']['pinocchio_profile']}.json"
    )
    if out.exists() and not args.overwrite:
        parser.error(f"{out} exists; earlier results are preserved (--overwrite)")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=1) + "\n")
    print(f"results written to {out} ({result['runtime_s']:.0f} s)")
    for d in result["diagnose_exact"]:
        print(
            f"diagnose_exact {d['derivatives']}/{d['noise']}/{d['weighting']}: "
            f"{d['verdict']} - {d['reason']}"
        )


if __name__ == "__main__":
    main()
