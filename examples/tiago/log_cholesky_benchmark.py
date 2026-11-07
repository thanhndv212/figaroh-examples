#!/usr/bin/env python3
"""LC1 (#11): log-Cholesky against OLS and OLS+SDP on recorded TIAGo data.

RECORDED DATA, frozen cross-run protocol v1
(``data/identification/protocol.yaml``): the fit uses session
2021-07-01-1302 (``dynamic/``, rows [921, 6791)), and every method is scored
on the independent session 2021-07-01-1257 (``calibration_slow/``, the same
path at about half speed), which no fit or setting sees. Effort is the PAL
controller's raw signal converted by the example's constants. The source
audit (#68) found arm_2-arm_4 read 10-20 % low and arm_1's scale is not
identifiable, so held-out errors include that model error. This is a
comparison of estimators on the same real rows. It is not hardware
validation of the identified parameters, and nothing is exported (the
reference workflow ``identification_reference.py --case physical-fit``
covers export).

Three overlays share the reference case's processing (velocity from
positions, automatic lag, decimation, arm_1-arm_4 effort fitted, friction,
offsets and actuator inertia free):

* ``config/tiago_lc1_ols.yaml``: base least squares, null-space
  reconstruction toward the nominal Hey5 URDF.
* ``config/tiago_lc1_ols_sdp.yaml``: plus the per-link LMI projection.
* ``config/tiago_lc1_log_cholesky.yaml``: ``physical_fit.method:
  log_cholesky`` as the selected stage. ``direct`` is the same config with
  ``method: direct``.

Every method's full standard vector is scored by core's own validation
metrics (``_compute_validation_metrics`` with that vector as the
estimate), on the same processed held-out rows.

Run from ``examples/tiago``::

    python log_cholesky_benchmark.py --output-dir /tmp/lc1-tiago
    python log_cholesky_benchmark.py --smoke --output-dir /tmp/lc1-tiago

Exit code 0 when every gate is met, 1 when one is not, 2 when the preflight
fails, 3 when core has no log-Cholesky fit. The archived run is
``docs/development/results/tiago-log-cholesky-lc1-pin37.*``.
"""

from __future__ import annotations

import argparse
import contextlib
import dataclasses
import hashlib
import importlib.util
import io
import json
import logging
import os
import platform
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from examples.tiago import identification as tiago_id  # noqa: E402
from examples.tiago import identification_reference as tref  # noqa: E402
from examples.tiago.identification_inputs_audit import summarise_per_joint  # noqa: E402

CONFIGS = {
    "ols": HERE / "config" / "tiago_lc1_ols.yaml",
    "ols_sdp": HERE / "config" / "tiago_lc1_ols_sdp.yaml",
    "log_cholesky": HERE / "config" / "tiago_lc1_log_cholesky.yaml",
}
METHODS = ("ols", "ols_sdp", "log_cholesky", "direct")
PROTOCOL = HERE / "data" / "identification" / "protocol.yaml"

# Decision-record gates (figaroh-plus#25) that apply without a truth model.
HELDOUT_RATIO_MAX = 1.05  # log-Cholesky / OLS+SDP held-out RMSE
FIT_SECONDS_MAX = 20.0
FEAS_TOL = -1e-8
DDQ_RMS_MIN = 1e-2  # rad/s² (m/s² for the torso), fitted coordinates
PERTURB_SEEDS = (1101, 1102)
PERTURB_SIGMA = 0.2
REPAIR_MIN_EIG = 1e-6


def core_module():
    if importlib.util.find_spec("figaroh.identification.log_cholesky_fit") is None:
        return None
    from figaroh.identification import log_cholesky_fit

    return log_cholesky_fit


@contextlib.contextmanager
def _quiet():
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        yield


# --- provenance -----------------------------------------------------------


def _git(repo: Path, *args: str) -> str:
    try:
        return subprocess.run(["git", "-C", str(repo), *args], capture_output=True,
                              text=True, check=True).stdout.strip()
    except Exception:
        return "unknown"


def revision(repo: Path) -> dict:
    return {
        "commit": _git(repo, "rev-parse", "HEAD"),
        "describe": _git(repo, "describe", "--tags", "--always"),
        "branch": _git(repo, "rev-parse", "--abbrev-ref", "HEAD"),
        "dirty": bool(_git(repo, "status", "--porcelain", "--untracked-files=no")),
    }


def sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def provenance(ident) -> dict:
    import cvxopt
    import figaroh
    import picos
    import pinocchio
    import scipy

    core = Path(figaroh.__file__).resolve().parents[2]
    ps = ident.identif_config
    return {
        "data": "RECORDED: TIAGo 2021-07-01 sessions, frozen cross-run protocol v1",
        "torque_provenance": (
            "PAL controller effort converted by the example's reduction ratio and "
            "motor constants (#68: arm_2-arm_4 read 10-20 % low, arm_1 scale "
            "unidentifiable); not a calibrated torque sensor"
        ),
        "hardware_validated": False,
        "urdf_exported": False,
        "examples": revision(ROOT),
        "core": revision(core),
        "core_version": getattr(figaroh, "__version__", "unknown"),
        "minimum_core": "figaroh-plus devel >= 21994ed; unreleased after 1.1.0",
        "versions": {
            "python": platform.python_version(),
            "numpy": np.__version__,
            "scipy": scipy.__version__,
            "pinocchio": pinocchio.__version__,
            "picos": picos.__version__,
            "cvxopt": cvxopt.__version__,
        },
        "platform": platform.platform(),
        "threads": {k: os.environ.get(k) for k in
                    ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "VECLIB_MAXIMUM_THREADS")},
        "model": {
            "nominal_prior_urdf": str(tref.URDF.relative_to(ROOT)),
            "active_joints": list(ps.get("active_joints", [])),
            "fitted_joints": list(ident.fit_joints()),
            "n_standard_parameters": len(ident.standard_parameter),
            "extras": "friction (viscous, Coulomb), joint offset, actuator inertia: free",
        },
        "split": {
            "train": "dynamic/ (session 2021-07-01-1302), rows [{}, {})".format(*tiago_id.TRUNCATE),
            "heldout": f"{ps['validation_data_file']} (session {ps['evaluation_session']}, "
                       f"role {ps['evaluation_role']}), all rows",
            "processing": "velocity from positions, automatic lag, core filtering, "
                          "decimate=True, wls=False",
        },
        "units": {"torso_lift_joint": "N", "arm_*": "N·m"},
        "inputs": tref.describe_inputs(ident),
        "hashes": {
            "protocol.yaml": sha256(PROTOCOL),
            "tiago_unified_config.yaml": sha256(HERE / "config" / "tiago_unified_config.yaml"),
            **{p.name: sha256(p) for p in CONFIGS.values()},
            "log_cholesky_benchmark.py": sha256(Path(__file__)),
        },
        "seeds": {"perturbation_seeds": list(PERTURB_SEEDS),
                  "perturbation_sigma": PERTURB_SIGMA},
    }


# --- pipeline -------------------------------------------------------------


def build(method: str):
    config = CONFIGS["log_cholesky" if method == "direct" else method]
    with _quiet():
        ident = tref.builder(config)(argparse.Namespace(asset_id=None, operator=None))
    if method == "direct":
        pf = ident.identif_config["physical_fit"]
        pf["method"] = "direct"
        pf.pop("log_cholesky", None)
    return ident


def solve(ident) -> float:
    t0 = time.perf_counter()
    with _quiet():
        ident.solve(decimate=True, plotting=False, save_results=False, html_report=False)
    return time.perf_counter() - t0


def preflight(ident) -> dict:
    """Check the processed accelerations of both recordings before fitting.

    Every fitted joint's coordinate must be excited and finite on training
    and held-out rows; the two must be different frozen sessions.
    """
    ps = ident.identif_config
    names = list(ps.get("active_joints", []))
    fitted = set(ident.fit_joints())
    model = ident.model
    cols = [model.joints[model.getJointId(n)].idx_v for n in names]  # full-nv layout
    report, failures = {}, []
    for split, data in (("train", ident.processed_data), ("heldout", ident._val_processed_data)):
        ddq = np.asarray(data["accelerations"], dtype=float)[:, cols]
        tau = np.asarray(data["torques"], dtype=float)
        rms = np.sqrt(np.mean(ddq**2, axis=0))
        report[split] = {
            "rows": int(ddq.shape[0]),
            "ddq_rms": dict(zip(names, rms.tolist())),
            "finite": bool(np.isfinite(ddq).all() and np.isfinite(tau).all()),
        }
        if not report[split]["finite"]:
            failures.append(f"{split}: non-finite accelerations or effort")
        for j, name in enumerate(names):
            if name in fitted and not rms[j] >= DDQ_RMS_MIN:
                failures.append(f"{split}/{name}: ddq not excited ({rms[j]:.3g})")
    report["heldout_session"] = ps["evaluation_session"]
    report["heldout_role"] = ps["evaluation_role"]
    if ps["evaluation_role"] != "validation":
        failures.append("held-out recording is not the frozen validation session")
    report["failures"] = failures
    report["passed"] = not failures
    return report


def problem(ident):
    """Core's physical-fit problem (selection.run_physical_fit)."""
    from figaroh.identification import physical_fit as pf
    from figaroh.identification.selection import _nominal_vector

    cfg = ident.identif_config["physical_fit"]
    names = list(ident.standard_parameter)
    W = np.asarray(ident._solve_W, dtype=float)
    eliminated = set(int(i) for i in (ident._idx_eliminated or []))
    keep = [i for i in range(len(names)) if i not in eliminated]
    Y = np.zeros((W.shape[0], len(names)))
    Y[:, keep] = W
    return pf.build_problem(
        Y, np.asarray(ident._solve_tau, dtype=float).reshape(-1), names,
        list(ident.model.names[1:]), prior=_nominal_vector(ident),
        policy=pf.PhysicalPolicy(mass_min=float(cfg["mass_min"]),
                                 prior_weight=float(cfg["prior_weight"])),
        row_weight=getattr(ident, "_wls_row_weight", None),
    )


def estimate(method: str, ident) -> tuple:
    names = list(ident.standard_parameter)
    nominal = dict(ident.standard_parameter)
    res = ident.result
    recon = res["reconstruction"]
    if method == "ols":
        values = {**nominal, **recon["theta_r_dict"]}
        rec = {"status": recon["status"], "stage": "reconstruction"}
    elif method == "ols_sdp":
        pc = res["physical consistency"]
        values = {**nominal, **recon["theta_r_dict"], **pc["projected_parameters"]}
        rec = {"status": pc["status"], "source": pc["source"],
               "stage": "physical consistency (not selectable)",
               "failed_links": pc["projection"].get("failed_links")}
    else:
        sel, pf = ident.selected, res["physical_fit"]
        values = dict(zip(sel.names, np.asarray(sel.values, dtype=float)))
        lc = pf.get("log_cholesky") or {}
        rec = {"status": pf["status"], "selected_stage": sel.stage,
               "solver_status": pf["solver_status"], "fit_runtime_s": pf["runtime_s"],
               "reason": pf["reason"], "branch": lc.get("branch"), "rank": lc.get("rank"),
               "null_space_dim": lc.get("null_space_dim"),
               "unconstrained_min_eig": lc.get("unconstrained_min_eig"),
               "start_used": lc.get("start_used")}
    return np.array([float(values[n]) for n in names]), rec


def heldout(ident, theta) -> dict:
    """Core's held-out metrics with ``theta`` as the estimate (same rows,
    masks and fitted-joint blocks for every method)."""
    sel = ident.selected
    assert list(sel.names) == list(ident.standard_parameter)
    ident.selected = dataclasses.replace(sel, values=np.asarray(theta, dtype=float))
    try:
        vm = ident._compute_validation_metrics()
    finally:
        ident.selected = sel
    per = vm["per_joint"]
    return {
        "aggregate": float(vm["rmse_identified"]),
        "per_joint": {j: float(v["rmse_identified"]) for j, v in per.items()},
        "nrmse": {j: float(v["nrmse"]) for j, v in per.items()},
        "correlation_normalised": float(vm["correlation_normalised"]),
        **summarise_per_joint(per),
        "n_samples": int(vm["n_val_samples"]),
    }


def objective(p, theta) -> float:
    r = p.row_weight * (p.Y @ theta - p.tau)
    q = (theta - p.theta_prior) / p.coord_scale
    return float(r @ r + p.policy.prior_weight * q @ q)


def feasibility(p, theta) -> dict:
    from figaroh.identification import physical_fit as pf

    return pf._feasibility(p, np.asarray(theta, dtype=float))


def score(method, ident, wall, lc_ident, p) -> tuple:
    theta, rec = estimate(method, ident)
    links = feasibility(p, theta)
    row = {
        **rec,
        "pipeline_runtime_s": wall,
        "heldout": heldout(lc_ident, theta),
        "train_objective": objective(p, theta),
        "train_rmse": float(np.sqrt(np.mean((p.Y @ theta - p.tau) ** 2))),
        "min_eig": min(v["min_eig"] for v in links.values()),
        "min_mass": min(v["mass"] for v in links.values()),
        "infeasible_links": sorted(j for j, v in links.items() if not v["ok"]),
        "n_links": len(links),
    }
    return row, theta, links


# --- starts ---------------------------------------------------------------


def starts(p, theta_ols, smoke: bool) -> dict:
    from figaroh.identification import physical_fit as pf
    from figaroh.identification.inertial_parameterization import (
        log_cholesky_from_p10, p10_from_log_cholesky, repair_p10)

    blocks = [[idx[k] for k in pf._INERTIAL_KEYS] for idx in pf._link_indices(p).values()]
    nominal = np.asarray(p.theta_prior, dtype=float).copy()
    repaired = np.asarray(theta_ols, dtype=float).copy()
    for cols in blocks:
        repaired[cols] = repair_p10(repaired[cols], min_eig=REPAIR_MIN_EIG).p10
    out = {"nominal": nominal, "repaired_ols": repaired}
    if not smoke:
        # The nominal Hey5 URDF's arm_1 link is itself slightly non-physical
        # (min eig about -1.2e-4), and log-Cholesky coordinates exist only for
        # positive-definite links, so the perturbations start from a repaired
        # copy. The raw nominal start above is kept as is.
        base = nominal.copy()
        for cols in blocks:
            base[cols] = repair_p10(base[cols], min_eig=REPAIR_MIN_EIG).p10
        for s in PERTURB_SEEDS:
            rng = np.random.default_rng(s)
            th = base.copy()
            for cols in blocks:
                z = log_cholesky_from_p10(th[cols]) + PERTURB_SIGMA * rng.standard_normal(10)
                th[cols] = p10_from_log_cholesky(z)
            out[f"perturbed_{s}"] = th
    out["unrepaired_ols"] = np.asarray(theta_ols, dtype=float).copy()
    return out


def run_starts(p, cands, lcf, branch, opts_kw, ref_theta) -> dict:
    out = {}
    for name, start in cands.items():
        t0 = time.perf_counter()
        r = lcf.solve_log_cholesky_fit(p, start=start,
                                       options=lcf.LogCholeskyOptions(branch=branch, **opts_kw))
        info = r.notes.get("interior") or {}
        row = {
            "branch": r.notes.get("branch"),
            "start_used": (r.notes.get("start") or {}).get("used"),
            "solver_status": r.solver_status,
            "accepted": bool(r.accepted),
            "converged": r.theta is not None,
            "stop_reason": info.get("stop_reason"),
            "nfev": info.get("nfev"),
            "identifiable_rms": info.get("identifiable_rms"),
            "runtime_s": time.perf_counter() - t0,
            "objective": None if r.theta is None else objective(p, r.theta),
        }
        if r.theta is not None:
            row["min_eig"] = min(v["min_eig"] for v in feasibility(p, r.theta).values())
            row["max_abs_diff_to_reference"] = float(np.abs(r.theta - ref_theta).max())
        out[name] = row
    return out


def lc_options(ident) -> dict:
    cfg = dict(ident.identif_config["physical_fit"].get("log_cholesky") or {})
    keys = ("rank_rtol", "eigenvalue_margin", "stop_rms", "max_nfev", "max_seconds")
    return {k: cfg[k] for k in keys if k in cfg}


# --- run ------------------------------------------------------------------


def run(args, lcf) -> dict:
    idents = {m: build(m) for m in METHODS}
    lc_ident = idents["log_cholesky"]
    out = {"title": "LC1 log-Cholesky comparison (#11): RECORDED TIAGo data",
           "smoke": args.smoke, "provenance": provenance(lc_ident)}
    pre = preflight(lc_ident)
    out["preflight"] = pre
    if not pre["passed"]:
        return out
    walls = {m: solve(idents[m]) for m in METHODS}
    p = problem(lc_ident)
    methods, thetas, links = {}, {}, {}
    for m in METHODS:
        methods[m], thetas[m], links[m] = score(m, idents[m], walls[m], lc_ident, p)
    core_lc = float(lc_ident.result["validation_metrics"]["rmse_identified"])
    methods["log_cholesky"]["selected_prediction_gap_nm"] = abs(
        core_lc - methods["log_cholesky"]["heldout"]["aggregate"])
    out["methods"] = methods
    out["conditioning"] = {
        "condition_number": float(lc_ident.result["condition number"]),
        "train_rows": int(p.Y.shape[0]),
        "eliminated_columns": len(lc_ident._idx_eliminated or []),
        "base_parameters": len(lc_ident._base_indices),
    }
    arm = list(lc_ident.identif_config.get("active_joints", []))
    worst = [j for j in sorted(links["ols"], key=lambda j: links["ols"][j]["min_eig"])
             if j not in arm][:2]
    nominal = feasibility(p, p.theta_prior)
    out["nominal_infeasible_links"] = {j: v["min_eig"] for j, v in nominal.items() if not v["ok"]}
    out["links"] = {j: {"nominal_mass": nominal[j]["mass"], **{m: links[m][j] for m in METHODS}}
                    for j in [*arm, *worst]}
    out["mass_plausibility"] = mass_plausibility(out["links"])
    cands = starts(p, thetas["ols"], args.smoke)
    opts = lc_options(lc_ident)
    out["starts_auto"] = run_starts(p, cands, lcf, "auto", opts, thetas["log_cholesky"])
    if not args.smoke:
        out["starts_forced_interior"] = run_starts(p, cands, lcf, "interior", opts,
                                                   thetas["log_cholesky"])
    out["gates"] = gates(out)
    h = {m: methods[m]["heldout"]["aggregate"] for m in METHODS}
    print("  held-out RMSE: " + ", ".join(f"{m} {h[m]:.4g}" for m in METHODS))
    return out


def mass_plausibility(links) -> dict:
    """Largest identified / nominal mass ratio per method, over the reported
    links. Reported, not gated: feasibility does not bound how far a weakly
    observed link's mass can drift from the URDF prior."""
    out = {}
    for m in METHODS:
        ratios = {j: v[m]["mass"] / v["nominal_mass"] for j, v in links.items()
                  if v["nominal_mass"] > 0}
        j = max(ratios, key=lambda k: abs(np.log(ratios[k])))
        out[m] = {"link": j, "ratio": ratios[j]}
    return out


def gates(out) -> dict:
    m = out["methods"]
    lc, sdp = m["log_cholesky"], m["ols_sdp"]
    g = {}
    ratio = lc["heldout"]["aggregate"] / sdp["heldout"]["aggregate"]
    g["heldout_ratio"] = {"ratio": ratio, "limit": HELDOUT_RATIO_MAX,
                          "ols_sdp_ok": sdp["status"] == "ok",
                          "met": bool(sdp["status"] == "ok" and ratio <= HELDOUT_RATIO_MAX),
                          "note": "measured effort of the independent session; no truth"}
    g["feasibility"] = {"min_eig": lc["min_eig"], "limit": FEAS_TOL,
                        "met": lc["min_eig"] >= FEAS_TOL and lc["status"] == "accepted"}
    g["fit_runtime"] = {"s": lc["fit_runtime_s"], "limit_s": FIT_SECONDS_MAX,
                        "met": lc["fit_runtime_s"] <= FIT_SECONDS_MAX}
    s = out["starts_auto"]
    g["gated_starts"] = {
        "met": all(s[k]["converged"] and s[k]["accepted"] for k in ("nominal", "repaired_ols")),
        "start_used": any(s[k]["start_used"] for k in ("nominal", "repaired_ols")),
        "note": "production (auto) branch; perturbed and unrepaired-OLS starts are "
                "reported, not gated",
    }
    g["not_applicable"] = ("clean held-out RMSE vs truth and the oracle interior "
                           "check need a truth model; recorded data has none")
    g["all_met"] = all(v["met"] for v in g.values() if isinstance(v, dict))
    return g


# --- report ---------------------------------------------------------------


def _fmt(x, f=".3g"):
    return "—" if x is None else format(x, f)


def markdown(out) -> str:
    pv = out["provenance"]
    L = [f"# {out['title']}", "",
         "> **RECORDED DATA**, frozen cross-run protocol v1. Estimators compared on "
         "the same real rows; effort is a converted controller signal (#68). Not "
         "hardware validation of the parameters; no URDF export.", "",
         f"- examples `{pv['examples']['describe']}` ({pv['examples']['commit'][:8]}"
         f"{', dirty' if pv['examples']['dirty'] else ''}), core `{pv['core']['describe']}` "
         f"({pv['core']['commit'][:8]}{', dirty' if pv['core']['dirty'] else ''})",
         "- " + ", ".join(f"{k} {v}" for k, v in pv["versions"].items()),
         f"- minimum core: {pv['minimum_core']}",
         f"- train: {pv['split']['train']}",
         f"- held-out: {pv['split']['heldout']}",
         f"- processing: {pv['split']['processing']}",
         f"- fitted joints: {', '.join(pv['model']['fitted_joints'])}; "
         f"{pv['model']['n_standard_parameters']} standard parameters; {pv['model']['extras']}",
         ""]
    pre = out["preflight"]
    L += ["## Preflight", "", f"passed: **{pre['passed']}**", ""]
    for split in ("train", "heldout"):
        e = pre[split]
        L.append(f"- {split}: {e['rows']} rows, finite {e['finite']}, ddq RMS " + ", ".join(
            f"{j} {v:.3g}" for j, v in e["ddq_rms"].items()))
    if pre["failures"]:
        L += ["", "Failures: " + "; ".join(pre["failures"])]
        return "\n".join(L) + "\n"
    m = out["methods"]
    joints = list(m["ols"]["heldout"]["per_joint"])
    L += ["", "## Held-out effort RMSE on 2021-07-01-1257 (calibration_slow)", "",
          "| method | aggregate | " + " | ".join(joints) + " | arm_2-4 pooled | "
          "corr (normalised) |", "|---" * (len(joints) + 4) + "|"]
    for k in METHODS:
        h = m[k]["heldout"]
        L.append(f"| {k} | {h['aggregate']:.4g} | " + " | ".join(
            f"{h['per_joint'][j]:.4g}" for j in joints) +
            f" | {h['pooled_arm2_4']:.4g} | {h['correlation_normalised']:.4f} |")
    c = out["conditioning"]
    L += ["", "## Training objective, physical consistency, conditioning", "",
          f"condition number {c['condition_number']:.3g}, {c['train_rows']} training rows, "
          f"{c['base_parameters']} base parameters, {c['eliminated_columns']} eliminated "
          "columns", "",
          "| method | status | J | train RMSE | min eig (all links) | min mass | "
          "infeasible links | pipeline s |", "|---" * 8 + "|"]
    for k in METHODS:
        r = m[k]
        L.append(f"| {k} | {r['status']} | {r['train_objective']:.6g} | "
                 f"{r['train_rmse']:.4g} | {r['min_eig']:.3g} | {r['min_mass']:.3g} | "
                 f"{len(r['infeasible_links'])} of {r['n_links']} | "
                 f"{r['pipeline_runtime_s']:.1f} |")
    lc = m["log_cholesky"]
    L += ["", f"log-Cholesky: branch **{lc['branch']}**, rank {lc['rank']}, null space "
          f"{lc['null_space_dim']}, unconstrained min eig {_fmt(lc['unconstrained_min_eig'])}, "
          f"fit {lc['fit_runtime_s']:.2f} s, start used {lc['start_used']}; "
          f"selected-stage prediction gap {lc['selected_prediction_gap_nm']:.2g} N·m.", "",
          "Per-link mass (kg) and minimum pseudo-inertia eigenvalue (torso/arm links, "
          "then the OLS fit's two least feasible other links):", "",
          "| link | nominal mass | " + " | ".join(f"{k} mass | {k} min eig" for k in METHODS)
          + " |", "|---" * (2 * len(METHODS) + 2) + "|"]
    for j, row in out["links"].items():
        L.append(f"| {j} | {row['nominal_mass']:.4g} | " + " | ".join(
            f"{row[k]['mass']:.4g} | {row[k]['min_eig']:.3g}" for k in METHODS) + " |")
    bad = out["nominal_infeasible_links"]
    L += ["", "Nominal URDF (the prior) non-physical links: " + (", ".join(
        f"{j} (min eig {v:.3g})" for j, v in bad.items()) or "none") + "."]
    mp = out["mass_plausibility"]
    L += ["", "Largest identified / nominal mass ratio (reported, not gated; feasibility "
          "does not bound drift from the prior on a weakly observed link): " + ", ".join(
              f"{k} {mp[k]['ratio']:.3g}× ({mp[k]['link']})" for k in METHODS) + "."]
    L += ["", "## Initialization sensitivity", "",
          "| start | auto: status | start used | max |Δθ| to selected | "
          "forced interior: stop | nfev | identifiable RMS | s |", "|---" * 8 + "|"]
    forced = out.get("starts_forced_interior", {})
    for k, a in out["starts_auto"].items():
        f = forced.get(k, {})
        L.append(f"| {k} | {a['solver_status']} | {a['start_used']} | "
                 f"{_fmt(a.get('max_abs_diff_to_reference'))} | "
                 f"{f.get('stop_reason') or f.get('solver_status', '—')} | "
                 f"{f.get('nfev') if f.get('nfev') is not None else '—'} | "
                 f"{_fmt(f.get('identifiable_rms'))} | "
                 f"{_fmt(f.get('runtime_s'), '.2f')} |")
    g = out["gates"]
    L += ["", "## Gates", "", "| gate | value | limit | met |", "|---|---|---|---|",
          f"| held-out LC / OLS+SDP (SDP ok: {g['heldout_ratio']['ols_sdp_ok']}) | "
          f"{g['heldout_ratio']['ratio']:.4f} | ≤ {HELDOUT_RATIO_MAX} | {g['heldout_ratio']['met']} |",
          f"| log-Cholesky min pseudo-inertia eig, every link | {g['feasibility']['min_eig']:.3g} "
          f"| ≥ {FEAS_TOL:g} | {g['feasibility']['met']} |",
          f"| log-Cholesky fit time (s) | {g['fit_runtime']['s']:.2f} | ≤ {FIT_SECONDS_MAX:g} | "
          f"{g['fit_runtime']['met']} |",
          f"| nominal + repaired-OLS starts converge, feasible | {g['gated_starts']['met']} | "
          f"all | {g['gated_starts']['met']} |",
          "", f"All gates met: **{g['all_met']}**. Not applicable: {g['not_applicable']}. "
          f"{g['gated_starts']['note']}.", ""]
    return "\n".join(L)


def _json(x):
    if isinstance(x, dict):
        return {str(k): _json(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_json(v) for v in x]
    if isinstance(x, np.generic):
        return x.item()
    if isinstance(x, np.ndarray):
        return x.tolist()
    return x


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--output-dir", required=True, help="where to write the report")
    ap.add_argument("--smoke", action="store_true",
                    help="three starts, production branch only")
    ap.add_argument("--stem", default="lc1_benchmark", help="output file name stem")
    ap.add_argument("--verbose", "-v", action="store_true")
    args = ap.parse_args(argv)
    os.environ.setdefault("MPLBACKEND", "Agg")
    logging.basicConfig(level=logging.INFO if args.verbose else logging.WARNING)
    if not args.verbose:
        logging.disable(logging.WARNING)
    lcf = core_module()
    if lcf is None:
        print("core figaroh has no identification.log_cholesky_fit; LC1 needs "
              "figaroh-plus devel >= 21994ed. Skipped.")
        return 3
    os.chdir(HERE)
    out = run(args, lcf)
    dest = Path(args.output_dir)
    dest.mkdir(parents=True, exist_ok=True)
    (dest / f"{args.stem}.json").write_text(json.dumps(_json(out), indent=2) + "\n")
    (dest / f"{args.stem}.md").write_text(markdown(out))
    print(f"wrote {dest / args.stem}.json and .md")
    if not out["preflight"]["passed"]:
        print("preflight FAILED:", "; ".join(out["preflight"]["failures"]))
        return 2
    unmet = [k for k, v in out["gates"].items() if isinstance(v, dict) and v["met"] is False]
    print("gates:", "all met" if not unmet else "UNMET " + ", ".join(unmet))
    return 0 if not unmet else 1


if __name__ == "__main__":
    sys.exit(main())
