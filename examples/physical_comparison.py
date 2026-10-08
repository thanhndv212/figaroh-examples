"""Shared machinery of the dynamic physical-estimator comparison (#22, D4).

Extends the UR10 first pass (``examples/ur10/compare_physical_estimators.py``,
#94) without changing it. One :class:`Spec` (regressor, extras, efforts, prior,
held-out data) is turned into a comparator problem in two modes:

* ``frozen``: the extra parameters (friction, actuator inertia, coupling) are
  fixed at ``Spec.e_fix`` and their effort is subtracted;
* ``joint``: the extras that are independent of the inertial base are
  estimated together with the inertial parameters. The comparator cannot
  estimate extras, so they are eliminated exactly (variable projection): the
  weighted regressor and effort are projected on the orthogonal complement of
  the kept extra columns, the comparator solves that problem, and the extras
  are recovered by least squares on the final residual. Extras collinear with
  the inertial base (for example actuator inertia) are absorbed there and are
  reported, not estimated.

Methods (all on the same problem, no silent fallback):

``base_ols`` (nullspace representative of the base least squares, no physical
constraint), ``exact_reconstruction``, ``direct_effort_fit``,
``per_link_projection`` (core private comparator, figaroh-plus#59) and
``log_cholesky`` (core private spike, started from the repaired nominal and
the repaired OLS representative). Every method is scored with one common
objective::

    J(theta) = || W (Y theta - tau) ||^2 + lam || (theta - theta0) / s ||^2

with the comparator's own ``lam`` (``PhysicalPolicy.prior_weight``), prior
``theta0`` and coordinate scale ``s``. ``direct_effort_fit`` is the convex
optimum of J over the pseudo-inertia cone; ``J`` of every other method is
reported against it. Solver convergence, physical feasibility and held-out
accuracy are separate fields.
"""

from __future__ import annotations

import hashlib
import importlib.util
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

CERT_TOL = 1e-7  # |s*| below this is not a phase-I certificate either way
SPIKE_REL = Path("docs/development/spikes/log_cholesky_feasibility.py")
LOGCHOL = {
    # D5 revised budget (figaroh-plus#30); the no-go there is about termination
    "max_nfev": 2000,
    "max_seconds_per_fit": 600.0,
    "converged_status": (1, 2, 4),
}
P10 = ["m", "mx", "my", "mz", "Ixx", "Ixy", "Iyy", "Ixz", "Iyz", "Izz"]


def sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def core_root(pcmp) -> Path:
    return Path(pcmp.__file__).resolve().parents[3]


def load_spike(pcmp):
    """Import the core log-Cholesky spike unchanged; only its CONFIG is set here."""
    path = core_root(pcmp) / SPIKE_REL
    spec = importlib.util.spec_from_file_location("figaroh_logchol_spike", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    mod.CONFIG = dict(mod.CONFIG)
    mod.CONFIG["max_nfev"] = LOGCHOL["max_nfev"]
    mod.CONFIG["max_seconds_per_fit"] = LOGCHOL["max_seconds_per_fit"]
    return mod, {"path": str(path), "sha256": sha256(path)}


@dataclass
class Spec:
    """One identification problem and its held-out data (joint-major rows)."""

    Y: np.ndarray  # (n, P) standard inertial columns, P = 10 * len(joints)
    E: np.ndarray  # (n, Ne) extra columns
    tau: np.ndarray  # (n,)
    Yv: np.ndarray
    Ev: np.ndarray
    names: List[str]  # f"{key}_{link}" in link-major P10 order
    extra_names: List[str]
    links: List[str]
    joints: List[str]  # actuated joints, row blocks of Y
    prior: np.ndarray  # (P,)
    e_fix: np.ndarray  # (Ne,) frozen extras
    heldout: Dict[str, np.ndarray]  # name -> target effort (n_v,)
    row_weight: Optional[np.ndarray] = None
    joint_units: Optional[List[str]] = None
    nrmse_scale: Optional[np.ndarray] = None  # per-joint normalisation
    truth_report: Any = None  # optional callable(theta) -> dict


def per_joint_rmse(res: np.ndarray, nv: int) -> np.ndarray:
    return np.sqrt(np.mean(np.asarray(res).reshape(nv, -1) ** 2, axis=1))


# --- problem assembly -----------------------------------------------------


def kept_extras(Yw, Ew, base_cols, tol=1e-6):
    """Greedy Gram-Schmidt: extras independent of the inertial base span."""
    Q, _ = np.linalg.qr(Yw[:, base_cols])
    kept, absorbed = [], []
    for c in range(Ew.shape[1]):
        r = Ew[:, c] - Q @ (Q.T @ Ew[:, c])
        if np.linalg.norm(r) > tol * max(np.linalg.norm(Ew[:, c]), 1e-300):
            Q = np.column_stack([Q, r / np.linalg.norm(r)])
            kept.append(c)
        else:
            absorbed.append(c)
    return kept, absorbed


@dataclass
class Built:
    p: Any
    mode: str
    kept: List[int] = field(default_factory=list)
    absorbed: List[int] = field(default_factory=list)


def build(spec: Spec, pcmp, mode: str) -> Built:
    w = np.ones(len(spec.tau)) if spec.row_weight is None else spec.row_weight
    if mode == "frozen":
        tau_eff = spec.tau - spec.E @ spec.e_fix
        p = pcmp.build_problem(
            spec.Y,
            tau_eff,
            spec.names,
            spec.links,
            prior=spec.prior,
            row_weight=spec.row_weight,
        )
        return Built(p, mode)
    if mode != "joint":
        raise ValueError(mode)
    p0 = pcmp.build_problem(
        spec.Y,
        spec.tau,
        spec.names,
        spec.links,
        prior=spec.prior,
        row_weight=spec.row_weight,
    )
    Yw, Ew, tw = w[:, None] * spec.Y, w[:, None] * spec.E, w * spec.tau
    kept, absorbed = kept_extras(Yw, Ew, list(p0.base_indices))
    if kept:
        Q, _ = np.linalg.qr(Ew[:, kept])
        Yw, tw = Yw - Q @ (Q.T @ Yw), tw - Q @ (Q.T @ tw)
    p = pcmp.build_problem(Yw, tw, spec.names, spec.links, prior=spec.prior)
    return Built(p, mode, kept, absorbed)


def extras_for(spec: Spec, b: Built, theta: np.ndarray) -> np.ndarray:
    """Extras used with ``theta``: frozen, or least squares on the residual."""
    if b.mode == "frozen":
        return spec.e_fix
    e = np.zeros(spec.E.shape[1])
    if b.kept:
        w = np.ones(len(spec.tau)) if spec.row_weight is None else spec.row_weight
        sol, *_ = np.linalg.lstsq(
            w[:, None] * spec.E[:, b.kept], w * (spec.tau - spec.Y @ theta), rcond=None
        )
        e[b.kept] = sol
    return e


def common_objective(p, theta) -> Dict[str, float]:
    """J = ||W(Y theta - tau)||^2 + lam||(theta - theta0)/s||^2 on the solved problem."""
    lam = max(p.policy.prior_weight, 0.0)
    data = float(np.sum((p.row_weight * (p.Y @ theta - p.tau)) ** 2))
    reg = float(lam * np.sum(((theta - p.theta_prior) / p.coord_scale) ** 2))
    return {"data": data, "reg": reg, "total": data + reg}


def ols_representative(p) -> np.ndarray:
    """Closest-to-prior theta with M theta = phi_ols (no LMI): base OLS as a theta."""
    D = p.coord_scale
    A = p.M_full * D[None, :]
    z = np.linalg.pinv(A) @ (p.phi_ols - p.M_full @ p.theta_prior)
    return p.theta_prior + D * z


# --- evaluation -----------------------------------------------------------


def evaluate(spec: Spec, b: Built, theta: np.ndarray, pcmp) -> Dict[str, Any]:
    p, nv = b.p, len(spec.joints)
    e = extras_for(spec, b, theta)
    pred_tr = spec.Y @ theta + spec.E @ e
    pred_v = spec.Yv @ theta + spec.Ev @ e
    out: Dict[str, Any] = {
        "train": {
            "rmse_vs_measured_effort": dict(
                zip(spec.joints, per_joint_rmse(pred_tr - spec.tau, nv).tolist())
            )
        },
        "heldout": {},
        "objective": common_objective(p, theta),
        "base_change_vs_ols_rel": float(
            np.linalg.norm(p.M_full @ theta - p.phi_ols) / np.linalg.norm(p.phi_ols)
        ),
        "base_change_abs_max": float(np.abs(p.M_full @ theta - p.phi_ols).max()),
        "n_base": len(p.base_indices),
        "extras": {
            "estimated": {spec.extra_names[c]: float(e[c]) for c in b.kept},
            "absorbed_in_base": [spec.extra_names[c] for c in b.absorbed],
            "frozen": (
                {n: float(v) for n, v in zip(spec.extra_names, spec.e_fix)}
                if b.mode == "frozen"
                else None
            ),
        },
    }
    for name, target in spec.heldout.items():
        rm = per_joint_rmse(pred_v - target, nv)
        out["heldout"][f"rmse_vs_{name}"] = dict(zip(spec.joints, rm.tolist()))
        if spec.nrmse_scale is not None:
            out["heldout"][f"nrmse_pct_vs_{name}"] = dict(
                zip(spec.joints, (100 * rm / spec.nrmse_scale).tolist())
            )
    prior = p.theta_prior
    links = {}
    for k, ln in enumerate(spec.links):
        s = slice(10 * k, 10 * k + 10)
        links[ln] = {
            "mass_kg": float(theta[s][0]),
            "mass_ratio_to_nominal": float(theta[s][0] / prior[s][0]),
            "change_vs_prior_norm": float(np.linalg.norm(theta[s] - prior[s])),
        }
    out["links"] = links
    if spec.truth_report is not None:
        out["truth"] = spec.truth_report(theta)
    return out


def _finite(x):
    """JSON-safe: a non-finite eigenvalue (non-positive mass) becomes None."""
    return float(x) if x is not None and np.isfinite(x) else None


def feasibility_fields(p, theta) -> Dict[str, Any]:
    f = pcmp_feasibility(p, theta)
    per_link = {
        j: {k: (_finite(v) if isinstance(v, float) else v) for k, v in d.items()}
        for j, d in f.items()
    }
    eigs = [v["min_eig"] for v in f.values()]
    return {
        "all_links_ok": all(v["ok"] for v in f.values()),
        "n_links_infeasible": sum(not v["ok"] for v in f.values()),
        "min_eig_min": _finite(min(eigs)),
        "min_eig_min_is_nonfinite": not np.isfinite(min(eigs)),
        "per_link": per_link,
    }


def pcmp_feasibility(p, theta):
    from figaroh.identification import _physical_comparator as pcmp

    return pcmp._feasibility(p, theta)


def record_fields(rec) -> Dict[str, Any]:
    d = rec.as_dict()
    return {
        "convergence": {
            "solver": d["solver"],
            "solver_status": d["solver_status"],
            "picos_status": d["picos_status"],
            "exception": d["exception"],
            "runtime_s": d["runtime_s"],
            "objective_value": d["objective_value"],
            "converged": d["solver_status"] == "optimal",
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


def phase1_certificate(p, pcmp, solver) -> Dict[str, Any]:
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
    return {
        "s_star": s_star,
        "label": label,
        "status": raw["status"],
        "exception": raw.get("exception"),
        "runtime_s": raw.get("runtime_s"),
    }


def run_logchol(
    spec: Spec, b: Built, theta_ols: np.ndarray, spike, pcmp
) -> Dict[str, Any]:
    """Log-Cholesky candidates from two starts, scored with the common objective.

    A fit that returns a candidate is reported with ``converged`` (scipy
    status in ``LOGCHOL['converged_status']``) kept apart from feasibility.
    Not converging does not discard the candidate.
    """
    p = b.p
    spike.CONFIG["prior_strength"] = max(p.policy.prior_weight, 0.0)
    Yw = p.row_weight[:, None] * p.Y
    tw = p.row_weight * p.tau
    prior_rep, _ = spike.repair(p.theta_prior)
    ols_rep, _ = spike.repair(theta_ols)
    out = {}
    for label, start in (("nominal_repaired", prior_rep), ("ols_repaired", ols_rep)):
        theta, rep = spike.fit(Yw, tw, p.theta_prior, p.coord_scale, start)
        entry: Dict[str, Any] = {
            "start": label,
            "convergence": {
                "solver": "scipy.least_squares trf (core spike)",
                "scipy_status": rep.get("status"),
                "message": rep.get("message"),
                "nfev": rep.get("nfev"),
                "njev": rep.get("njev"),
                "optimality": rep.get("optimality"),
                "runtime_s": rep.get("runtime_seconds"),
                "max_nfev": spike.CONFIG["max_nfev"],
                "converged": rep.get("status") in LOGCHOL["converged_status"],
                "budget_exhausted": rep.get("status") == 0,
            },
        }
        if "status" not in rep:  # guard/exception: the start is returned, no candidate
            entry["candidate"] = False
            out[label] = entry
            continue
        entry["candidate"] = True
        feas = feasibility_fields(p, theta)
        entry["feasibility"] = feas
        entry.update(evaluate(spec, b, theta, pcmp))
        entry["spike_cost_check"] = float(2.0 * rep["objective"])
        out[label] = entry
    return out


def run_methods(
    spec: Spec,
    mode: str,
    pcmp,
    spike,
    solver: str,
    *,
    logchol=True,
    second_solver: Optional[str] = None,
):
    """All methods for one problem; returns (Built, methods)."""
    b = build(spec, pcmp, mode)
    p = b.p
    methods: Dict[str, Any] = {}
    theta_ols = ols_representative(p)
    ols = evaluate(spec, b, theta_ols, pcmp)
    ols["convergence"] = {
        "solver": "lstsq",
        "solver_status": "optimal",
        "converged": True,
    }
    ols["feasibility"] = feasibility_fields(p, theta_ols)
    ols["feasibility"][
        "note"
    ] = "nullspace representative of the base OLS; the OLS itself is unconstrained"
    methods["base_ols"] = ols
    for name, fn in (
        ("exact_reconstruction", pcmp.solve_exact_reconstruction),
        ("direct_effort_fit", pcmp.solve_direct_effort_fit),
        ("per_link_projection", pcmp.solve_per_link_projection),
    ):
        rec = fn(p, solver=solver)
        entry = record_fields(rec)
        if rec.theta is not None:
            entry.update(evaluate(spec, b, rec.theta, pcmp))
        methods[name] = entry
        if second_solver and entry["convergence"]["solver_status"] != "optimal":
            # explicit second entry (never a silent fallback): the primary
            # solver's failure stays in ``methods[name]``
            rec2 = fn(p, solver=second_solver)
            e2 = record_fields(rec2)
            if rec2.theta is not None:
                e2.update(evaluate(spec, b, rec2.theta, pcmp))
            e2["note"] = (
                f"{solver} did not return an optimal solution; "
                f"same objective re-solved with {second_solver}"
            )
            methods[f"{name}@{second_solver}"] = e2
    methods["exact_reconstruction"]["phase1"] = phase1_certificate(p, pcmp, solver)
    if second_solver:
        methods["exact_reconstruction"]["phase1_second_solver"] = phase1_certificate(
            p, pcmp, second_solver
        )
    if logchol:
        methods["log_cholesky"] = run_logchol(spec, b, theta_ols, spike, pcmp)
    ref_entry = methods["direct_effort_fit"]
    if "objective" not in ref_entry and second_solver:
        ref_entry = methods.get(f"direct_effort_fit@{second_solver}", ref_entry)
    ref = ref_entry.get("objective", {}).get("total")
    for m in methods.values():
        for entry in (
            [m]
            if "objective" in m
            else [v for v in m.values() if isinstance(v, dict) and "objective" in v]
        ):
            if ref:
                entry["objective"]["relative_excess_vs_direct"] = (
                    entry["objective"]["total"] - ref
                ) / ref
    return b, methods


# --- second solver cross-check --------------------------------------------


def cross_check(spec: Spec, mode: str, pcmp, solvers: List[str]) -> Dict[str, Any]:
    """Re-solve the comparator objectives with each solver and compare."""
    b = build(spec, pcmp, mode)
    p = b.p
    ref: Dict[str, np.ndarray] = {}
    out: Dict[str, Any] = {}
    for solver in solvers:
        res = {}
        for name, fn in (
            ("exact_reconstruction", pcmp.solve_exact_reconstruction),
            ("direct_effort_fit", pcmp.solve_direct_effort_fit),
            ("per_link_projection", pcmp.solve_per_link_projection),
        ):
            rec = fn(p, solver=solver)
            d = record_fields(rec)
            entry = {
                "convergence": d["convergence"],
                "feasibility": {
                    k: d["feasibility"][k] for k in ("all_links_ok", "min_eig_min")
                },
                "accepted": d["accepted"],
            }
            if rec.theta is not None:
                ev = evaluate(spec, b, rec.theta, pcmp)
                entry["objective"] = ev["objective"]
                entry["heldout"] = ev["heldout"]
                key = name
                if solver == solvers[0]:
                    ref[key] = rec.theta
                elif key in ref:
                    entry["theta_diff_vs_first_solver_scaled"] = float(
                        np.linalg.norm((rec.theta - ref[key]) / p.coord_scale)
                    )
            res[name] = entry
        res["phase1"] = phase1_certificate(p, pcmp, solver)
        out[solver] = res
    return out


def agree(cc: Dict[str, Any], first: str, second: str) -> Dict[str, Any]:
    """Per-method agreement verdict between two solvers' cross-check results."""
    verdicts = {}
    for name in ("exact_reconstruction", "direct_effort_fit", "per_link_projection"):
        a, c = cc[first][name], cc[second][name]
        ok = a["convergence"]["solver_status"] == "optimal"
        ck = c["convergence"]["solver_status"] == "optimal"
        row = {
            "first": a["convergence"]["solver_status"],
            "second": c["convergence"]["solver_status"],
        }
        if ok and ck:
            ja, jc = a["objective"]["total"], c["objective"]["total"]
            row["objective_rel_diff"] = abs(ja - jc) / max(abs(ja), 1e-300)
            row["theta_diff_scaled"] = c.get("theta_diff_vs_first_solver_scaled")
        verdicts[name] = row
    verdicts["phase1"] = {
        "first": cc[first]["phase1"]["label"],
        "second": cc[second]["phase1"]["label"],
    }
    return verdicts


def timed(fn, *a, **k):
    t0 = time.perf_counter()
    r = fn(*a, **k)
    return r, time.perf_counter() - t0
