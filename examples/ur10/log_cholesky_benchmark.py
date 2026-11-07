#!/usr/bin/env python3
"""LC1 (#11): log-Cholesky against OLS and corrected OLS+SDP on UR10.

SIMULATED DATA. Every number comes from the UR10 dynamic-truth fixture
(``data/truth``): effort is Pinocchio RNEA on a drawn truth model, plus
seeded Gaussian noise. Nothing here was measured on a robot, and no URDF
is exported. The reference workflow (``identification_reference.py
--case log-cholesky``) covers export into a lumped nominal URDF.

Three overlay configs share one frozen split, weighting and prior:

* ``config/ur10_lc1_ols.yaml``: linear baseline (OLS base fit, null-space
  reconstruction toward the nominal URDF).
* ``config/ur10_lc1_ols_sdp.yaml``: the baseline plus the corrected
  per-link LMI projection (figaroh-plus#163), ``physical_consistency``.
* ``config/ur10_lc1_log_cholesky.yaml``: ``select_stage: physical_fit``,
  ``physical_fit.method: log_cholesky`` (core figaroh-plus#25).

Each method gives a full standard vector theta (60 parameters). Every
method is scored the same way: the 60-column regressor of the held-out
trajectory, times theta. It is scored against the noisy measurement and
against the noise-free truth effort.

Weak excitation (decision record: gated on convergence and feasibility
only) is derived in memory from ``train.csv``: ``shoulder_pan_joint`` keeps
its motion, the other joints hold the fixture's centre posture, and effort is
truth RNEA on those rows (plus the same seeded noise). It is a separate
training set, not a fixture revision; validation is the usual held-out split.

Run from ``examples/ur10``::

    python log_cholesky_benchmark.py --output-dir /tmp/lc1          # full grid
    python log_cholesky_benchmark.py --smoke --output-dir /tmp/lc1  # bounded

The output is ``<stem>.json`` and ``<stem>.md`` (``--stem``, default
``lc1_benchmark``). The archived runs are
``docs/development/results/ur10-log-cholesky-lc1-pin{37,41}.*`` (Pinocchio
3.7 in ``figaroh-dev``, 4.1 in a pip venv from core
``ci/pinocchio-4.1.0.txt``). The fixture
files are read only. The exit code is 0 when every gate is met. It is 1
when a gate is unmet (the report lists which ones), 2 when the preflight
fails, and 3 when core has no log-Cholesky fit.
"""

from __future__ import annotations

import argparse
import contextlib
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
sys.path.insert(0, str(ROOT))

from examples.ur10 import identification_truth as truth  # noqa: E402

CONFIGS = {
    "ols": HERE / "config" / "ur10_lc1_ols.yaml",
    "ols_sdp": HERE / "config" / "ur10_lc1_ols_sdp.yaml",
    "log_cholesky": HERE / "config" / "ur10_lc1_log_cholesky.yaml",
}
METHODS = ("ols", "ols_sdp", "log_cholesky", "direct")
P10 = ("m", "mx", "my", "mz", "Ixx", "Ixy", "Iyy", "Ixz", "Iyz", "Izz")
UNITS = {
    "effort": "N·m",
    "m": "kg",
    "mx, my, mz": "kg·m",
    "Ixx..Izz": "kg·m² (at the joint frame)",
    "objective": "N²·m² (sum over stacked rows)",
}

# Decision-record gates (figaroh-plus#25)
CLEAN_RMSE_MAX = 1e-5  # N·m, noise-free held-out vs truth
NOISY_RATIO_MAX = 1.05  # log-Cholesky / OLS+SDP held-out RMSE
FIT_SECONDS_MAX = 20.0
FEAS_TOL = -1e-8  # #22 feasibility oracle: min eig(P) >= -1e-8 on every link

# Preflight tolerances (manifest values are about 1e-9)
DERIV_TOL = 1e-6
DYN_TOL = 1e-8
DDQ_RMS_MIN = 1e-2  # rad/s², every active coordinate must be excited
WEAK_ACTIVE = ("shoulder_pan_joint",)  # the only moving joint of the weak set

PERTURB_SEEDS = (1101, 1102)
PERTURB_SIGMA = 0.2  # in log-Cholesky coordinates
REPAIR_MIN_EIG = 1e-6


def core_module():
    """The core log-Cholesky module, or None when core predates it."""
    if importlib.util.find_spec("figaroh.identification.log_cholesky_fit") is None:
        return None
    from figaroh.identification import log_cholesky_fit

    return log_cholesky_fit


# --- provenance -----------------------------------------------------------


def _git(repo: Path, *args: str) -> str:
    try:
        return subprocess.run(
            ["git", "-C", str(repo), *args], capture_output=True, text=True, check=True
        ).stdout.strip()
    except Exception:
        return "unknown"


def revision(repo: Path) -> dict:
    return {
        "commit": _git(repo, "rev-parse", "HEAD"),
        "describe": _git(repo, "describe", "--tags", "--always"),
        "branch": _git(repo, "rev-parse", "--abbrev-ref", "HEAD"),
        "dirty": bool(_git(repo, "status", "--porcelain", "--untracked-files=no")),
    }


def provenance(seeds: dict) -> dict:
    import cvxopt
    import figaroh
    import picos
    import scipy

    core = Path(figaroh.__file__).resolve().parents[2]
    fx, sha = truth.FIXTURE_DIR, truth.sha256
    return {
        "data": "SIMULATED: Pinocchio RNEA on the drawn truth model (data/truth)",
        "torque_provenance": (
            "tau = RNEA(truth URDF, q, dq, ddq) with analytic q/dq/ddq; noisy "
            "levels add seeded Gaussian effort and position noise (protocol.yaml)"
        ),
        "hardware_validated": False,
        "urdf_exported": False,
        "examples": revision(ROOT),
        "core": revision(core),
        "core_version": getattr(figaroh, "__version__", "unknown"),
        "minimum_core": (
            "figaroh-plus devel >= 21994ed (log-Cholesky #235 plus the #234 "
            "null-space fix #236); unreleased after 1.1.0"
        ),
        "versions": {
            "python": platform.python_version(),
            "numpy": np.__version__,
            "scipy": scipy.__version__,
            "pinocchio": truth.pin.__version__,
            "picos": picos.__version__,
            "cvxopt": cvxopt.__version__,
        },
        "platform": platform.platform(),
        "threads": {
            k: os.environ.get(k)
            for k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "VECLIB_MAXIMUM_THREADS")
        },
        "model": {
            "nominal_prior_urdf": str(truth.URDF.relative_to(ROOT)),
            "truth_urdf": str((fx / "ur10_truth.urdf").relative_to(ROOT)),
            "joints": list(truth.JOINTS),
            "parameter_order": "per joint " + ", ".join(P10),
        },
        "split": {
            "train": "train.csv (24 s spline, 2400 rows at 100 Hz)",
            "heldout": "validation.csv (independent 8 s Fourier, 800 rows)",
            "derivatives": "analytic, unfiltered, undecimated",
        },
        "seeds": seeds,
        "units": UNITS,
        "hashes": {
            **{n: sha(fx / n) for n in ("train.csv", "validation.csv", "manifest.json",
                                         "protocol.yaml", "ur10_truth.urdf",
                                         "truth_parameters.csv")},
            "ur10_robot.urdf": sha(truth.URDF),
            "ur10_unified_config.yaml": sha(HERE / "config" / "ur10_unified_config.yaml"),
            **{p.name: sha(p) for p in CONFIGS.values()},
            "log_cholesky_benchmark.py": sha(Path(__file__)),
        },
    }


# --- preflight ------------------------------------------------------------


def preflight() -> dict:
    """Check every active acceleration coordinate before any benchmark.

    Independent dynamics (CRBA+NLE, Pinocchio regressor, ABA) and central
    differences of the trajectory definition, per coordinate, on both
    splits; then the pipeline must hand the fixture ddq to the regressor
    unchanged.
    """
    model, man = truth.truth_model(), truth.manifest()
    report, failures = {}, []
    for split in ("train", "validation"):
        data = truth.read_split(split)
        checks = truth.independent_checks(model, data, man["splits"][split]["definition"])
        ddq_rms = np.sqrt(np.mean(data["ddq"] ** 2, axis=0))
        entry = {
            "ddq_central_difference": checks["ddq_central_difference"],
            "dq_central_difference": checks["dq_central_difference"],
            "crba_nle": checks["crba_nle"],
            "aba_ddq": checks["aba_ddq"],
            "pinocchio_regressor": checks["pinocchio_regressor"],
            "figaroh_regressor": checks["figaroh_regressor"],
            "ddq_rms": dict(zip(truth.JOINTS, ddq_rms.tolist())),
        }
        for j, joint in enumerate(truth.JOINTS):
            if not checks["ddq_central_difference"][j] <= DERIV_TOL:
                failures.append(f"{split}/{joint}: ddq central difference")
            if not checks["dq_central_difference"][j] <= DERIV_TOL:
                failures.append(f"{split}/{joint}: dq central difference")
            if not ddq_rms[j] >= DDQ_RMS_MIN:
                failures.append(f"{split}/{joint}: ddq not excited")
        for key in ("crba_nle", "aba_ddq", "pinocchio_regressor", "figaroh_regressor"):
            if not checks[key] <= DYN_TOL:
                failures.append(f"{split}: {key} {checks[key]:.3g}")
        report[split] = entry

    ident = truth.identification(config=CONFIGS["ols"])
    with _quiet():
        ident.process_data()
    used = np.asarray(ident.processed_data["accelerations"])
    fixture = truth.read_split("train")["ddq"]
    same = used.shape == fixture.shape and float(np.abs(used - fixture).max()) == 0.0
    report["pipeline_ddq_is_fixture_ddq"] = same
    if not same:
        failures.append("pipeline accelerations differ from the fixture ddq")
    report["failures"] = failures
    report["passed"] = not failures
    return report


def weak_split(noise: str, seed) -> dict:
    """Weak-excitation training rows (see the module docstring).

    The moving joint keeps the fixture's q/dq/ddq, which the preflight has
    verified; the frozen joints have dq = ddq = 0 exactly.
    """
    d = truth.read_split("train")
    n = len(d["t"])
    q = np.tile(np.asarray(truth.CENTER, dtype=float), (n, 1))
    dq, ddq = np.zeros_like(q), np.zeros_like(q)
    for j in (truth.JOINTS.index(name) for name in WEAK_ACTIVE):
        q[:, j], dq[:, j], ddq[:, j] = d["q"][:, j], d["dq"][:, j], d["ddq"][:, j]
    tau_true = truth.rnea(truth.truth_model(), q, dq, ddq)
    tau = tau_true.copy()
    if noise != "none":
        if seed not in truth.NOISE_SEEDS["train"]:
            raise ValueError(f"weak: noise seed must be one of {truth.NOISE_SEEDS['train']}")
        e, pos = truth.standard_draws(seed, n)
        sigma = truth.protocol()["noise"]["effort_sigma_nm"][noise]
        tau = tau + e * np.array([sigma[j] for j in truth.JOINTS])
        q = q + pos * truth.NOISE_LEVELS[noise]["position_rad"]
    return {"t": d["t"], "q": q, "dq": dq, "ddq": ddq, "tau": tau, "tau_true": tau_true}


def weak_summary() -> dict:
    w = weak_split("none", None)
    return {
        "active_joints": list(WEAK_ACTIVE),
        "frozen_posture_rad": list(map(float, truth.CENTER)),
        "ddq_rms": dict(zip(truth.JOINTS,
                            np.sqrt(np.mean(w["ddq"] ** 2, axis=0)).tolist())),
        "effort_rms_nm": dict(zip(truth.JOINTS,
                                  np.sqrt(np.mean(w["tau"] ** 2, axis=0)).tolist())),
        "rows": int(len(w["t"])),
        "note": "derived in memory from train.csv; the active joint's "
                "derivatives are the fixture's (preflight-verified)",
    }


# --- scoring --------------------------------------------------------------


@contextlib.contextmanager
def _quiet():
    with contextlib.redirect_stdout(io.StringIO()):
        yield


def heldout_split(noise: str, val_seed) -> dict:
    """Held-out regressors: as measured (noisy q, tau) and noise-free."""
    model = truth.nominal_model()
    robot = truth._Robot(model)
    meas = truth.load_split("validation", "analytic", noise, val_seed)
    clean = truth.read_split("validation")

    def reg(d):
        return truth.build_regressor_basic(
            robot, d["q"], d["dq"], d["ddq"], truth.IDENTIF_CONFIG
        )

    return {
        "Y_meas": reg(meas),
        "tau_meas": meas["tau"].T.ravel(),
        "Y_clean": reg(clean),
        "tau_clean": clean["tau"].T.ravel(),
    }


def rmse(Y, tau, theta) -> dict:
    res = (Y @ theta - tau).reshape(len(truth.JOINTS), -1)
    per = np.sqrt(np.mean(res**2, axis=1))
    return {
        "aggregate": float(np.sqrt(np.mean(res**2))),
        "per_joint": dict(zip(truth.JOINTS, per.tolist())),
    }


def objective(p, theta) -> float:
    """J = ||W (Y theta - tau)||^2 + lambda ||(theta - theta0) / D||^2."""
    r = p.row_weight * (p.Y @ theta - p.tau)
    q = (theta - p.theta_prior) / p.coord_scale
    return float(r @ r + p.policy.prior_weight * q @ q)


def links(theta) -> dict:
    from figaroh.identification.inertial_parameterization import pseudo_inertia_from_p10

    out = {}
    for b, joint in enumerate(truth.JOINTS):
        p10 = np.asarray(theta[10 * b : 10 * b + 10], dtype=float)
        out[joint] = {
            "mass": float(p10[0]),
            "min_eig": float(np.linalg.eigvalsh(pseudo_inertia_from_p10(p10))[0]),
        }
    return out


def problem(ident, prior=None):
    """The pipeline's own physical-fit problem (selection.run_physical_fit)."""
    from figaroh.identification import physical_fit as pf

    cfg = ident.identif_config["physical_fit"]
    names = list(ident.standard_parameter)
    W = np.asarray(ident._solve_W, dtype=float)
    eliminated = set(int(i) for i in (ident._idx_eliminated or []))
    keep = [i for i in range(len(names)) if i not in eliminated]
    Y = np.zeros((W.shape[0], len(names)))
    Y[:, keep] = W
    nominal = np.array(list(ident.standard_parameter.values()), dtype=float)
    return pf.build_problem(
        Y,
        np.asarray(ident._solve_tau, dtype=float).reshape(-1),
        names,
        list(ident.model.names[1:]),
        prior=nominal if prior is None else prior,
        policy=pf.PhysicalPolicy(
            mass_min=float(cfg["mass_min"]), prior_weight=float(cfg["prior_weight"])
        ),
        row_weight=getattr(ident, "_wls_row_weight", None),
    )


# --- methods --------------------------------------------------------------


def run_pipeline(method: str, noise: str, seed, val_seed, excitation: str = "full"):
    config = CONFIGS["log_cholesky" if method == "direct" else method]
    train = weak_split(noise, seed) if excitation == "weak" else None
    ident = truth.identification(noise=noise, seed=seed, val_seed=val_seed,
                                 config=config, train=train)
    if method == "direct":
        pf = ident.identif_config["physical_fit"]
        pf["method"] = "direct"
        pf.pop("log_cholesky", None)
    t0 = time.perf_counter()
    with _quiet():
        ident.solve(decimate=False, plotting=False, save_results=False)
    return ident, time.perf_counter() - t0


def estimate(method: str, ident) -> tuple:
    """(theta over the 60 standard parameters, status record)."""
    names = list(ident.standard_parameter)
    nominal = dict(ident.standard_parameter)
    res = ident.result
    if method == "ols":
        recon = res["reconstruction"]
        values = {**nominal, **recon["theta_r_dict"]}
        rec = {"status": recon["status"], "stage": "reconstruction"}
    elif method == "ols_sdp":
        pc = res["physical consistency"]
        values = pc["projected_parameters"]
        rec = {
            "status": pc["status"],
            "source": pc["source"],
            "stage": "physical consistency (not selectable)",
            "failed_links": pc["projection"].get("failed_links"),
        }
    else:
        sel, pf = ident.selected, res["physical_fit"]
        values = dict(zip(sel.names, np.asarray(sel.values, dtype=float)))
        lc = pf.get("log_cholesky") or {}
        rec = {
            "status": pf["status"],
            "selected_stage": sel.stage,
            "solver_status": pf["solver_status"],
            "fit_runtime_s": pf["runtime_s"],
            "reason": pf["reason"],
            "branch": lc.get("branch"),
            "rank": lc.get("rank"),
            "null_space_dim": lc.get("null_space_dim"),
            "unconstrained_min_eig": lc.get("unconstrained_min_eig"),
            "active_links": lc.get("active_links"),
            "start_used": lc.get("start_used"),
            "interior": lc.get("interior"),
        }
    return np.array([float(values[n]) for n in names]), rec


def flows_into_predictions(ident, theta, held) -> float:
    """|core's selected held-out prediction - Y_val theta|, all rows."""
    vm = ident.result["validation_metrics"]
    per = vm["tau_identified_per_joint"]
    ours = (held["Y_meas"] @ theta).reshape(len(truth.JOINTS), -1)
    tau_core = np.array([np.asarray(per[j], dtype=float) for j in truth.JOINTS])
    return float(np.abs(tau_core - ours).max())


def score(method, ident, wall, held, p) -> dict:
    theta, rec = estimate(method, ident)
    row = {
        **rec,
        "pipeline_runtime_s": wall,
        "heldout_vs_measurement": rmse(held["Y_meas"], held["tau_meas"], theta),
        "heldout_vs_truth": rmse(held["Y_clean"], held["tau_clean"], theta),
        "train_objective": objective(p, theta),
        "train_rmse": float(np.sqrt(np.mean((p.Y @ theta - p.tau) ** 2))),
        "links": links(theta),
    }
    row["min_eig"] = min(v["min_eig"] for v in row["links"].values())
    row["min_mass"] = min(v["mass"] for v in row["links"].values())
    if method in ("log_cholesky", "direct"):
        row["selected_prediction_gap_nm"] = flows_into_predictions(ident, theta, held)
    return row, theta


# --- start sensitivity ----------------------------------------------------


def starts(nominal, theta_ols) -> dict:
    """Starts over the 60 parameters: nominal URDF, OLS repaired per link,
    seeded perturbations of the nominal in log-Cholesky coordinates, and
    the raw (non-physical) OLS reconstruction."""
    from figaroh.identification.inertial_parameterization import (
        log_cholesky_from_p10,
        p10_from_log_cholesky,
        repair_p10,
    )

    nominal = np.asarray(nominal, dtype=float).copy()
    repaired = theta_ols.copy()
    for b in range(len(truth.JOINTS)):
        cols = slice(10 * b, 10 * b + 10)
        rep = repair_p10(repaired[cols], min_eig=REPAIR_MIN_EIG)
        repaired[cols] = rep.p10
    out = {"nominal": nominal, "repaired_ols": repaired}
    for s in PERTURB_SEEDS:
        rng = np.random.default_rng(s)
        th = nominal.copy()
        for b in range(len(truth.JOINTS)):
            cols = slice(10 * b, 10 * b + 10)
            z = log_cholesky_from_p10(th[cols]) + PERTURB_SIGMA * rng.standard_normal(10)
            th[cols] = p10_from_log_cholesky(z)
        out[f"perturbed_{s}"] = th
    out["unrepaired_ols"] = theta_ols.copy()
    return out


def run_starts(p, candidates, lcf, branch, opts_kw, ref_theta=None) -> dict:
    out = {}
    for name, start in candidates.items():
        opts = lcf.LogCholeskyOptions(branch=branch, **opts_kw)
        t0 = time.perf_counter()
        r = lcf.solve_log_cholesky_fit(p, start=start, options=opts)
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
            "exception": r.exception,
            "runtime_s": time.perf_counter() - t0,
            "objective": None if r.theta is None else objective(p, r.theta),
        }
        if r.theta is not None:
            row["min_eig"] = min(v["min_eig"] for v in links(r.theta).values())
            if ref_theta is not None:
                row["max_abs_diff_to_reference"] = float(np.abs(r.theta - ref_theta).max())
        out[name] = row
    return out


# --- grid -----------------------------------------------------------------


def datasets(smoke: bool) -> list:
    """(excitation, noise, train seed, validation seed) per dataset."""
    if smoke:
        return [("full", "low", 101, 201), ("weak", "low", 101, 201)]
    grid = [("full", "none", None, None)]
    for noise in ("low", "high"):
        grid += [("full", noise, s, v) for s, v in
                 zip(truth.NOISE_SEEDS["train"], truth.NOISE_SEEDS["validation"])]
    grid += [("weak", "none", None, None), ("weak", "low", 101, 201)]
    return grid


def label(r) -> str:
    return r["noise"] if r.get("excitation", "full") == "full" else f"weak/{r['noise']}"


def lc_options(ident) -> dict:
    cfg = dict(ident.identif_config["physical_fit"].get("log_cholesky") or {})
    keys = ("rank_rtol", "eigenvalue_margin", "stop_rms", "max_nfev", "max_seconds")
    return {k: cfg[k] for k in keys if k in cfg}


def run(args, lcf) -> dict:
    grid = datasets(args.smoke)
    seeds = {
        "datasets": [{"excitation": x, "noise": n, "train_seed": s, "validation_seed": v}
                     for x, n, s, v in grid],
        "perturbation_seeds": list(PERTURB_SEEDS),
        "perturbation_sigma": PERTURB_SIGMA,
    }
    out = {"title": "LC1 log-Cholesky comparison (#11): SIMULATED UR10 truth fixture",
           "smoke": args.smoke, "provenance": provenance(seeds)}
    pre = preflight()
    out["preflight"] = pre
    if not pre["passed"]:
        return out
    out["weak_excitation"] = weak_summary()

    rows = []
    for n, (exc, noise, seed, vseed) in enumerate(grid):
        held = heldout_split(noise, vseed)
        entry = {"excitation": exc, "noise": noise, "train_seed": seed,
                 "validation_seed": vseed, "methods": {}}
        idents = {m: run_pipeline(m, noise, seed, vseed, exc) for m in METHODS}
        lc_ident = idents["log_cholesky"][0]
        p = problem(lc_ident)  # same training rows, weights and prior for all
        thetas = {}
        for method, (ident, wall) in idents.items():
            entry["methods"][method], thetas[method] = score(method, ident, wall, held, p)
        entry["condition_number"] = float(lc_ident.result["condition number"])
        entry["rows"] = int(p.Y.shape[0])
        entry["eliminated_columns"] = len(lc_ident._idx_eliminated or [])
        # every start, production branch (cheap) and forced interior (first
        # dataset of each excitation/noise pair, or every run in smoke mode)
        cands = starts(p.theta_prior, thetas["ols"])
        if args.smoke:
            cands = {k: cands[k] for k in ("nominal", "repaired_ols", "unrepaired_ols")}
        opts = lc_options(lc_ident)
        entry["starts_auto"] = run_starts(p, cands, lcf, "auto", opts,
                                          ref_theta=thetas["log_cholesky"])
        first = n == 0 or grid[n - 1][:2] != (exc, noise)
        if first:
            entry["starts_forced_interior"] = run_starts(p, cands, lcf, "interior", opts)
        rows.append(entry)
        print(f"  {label(entry):8s} seed {seed}: log-Cholesky held-out "
              f"{entry['methods']['log_cholesky']['heldout_vs_measurement']['aggregate']:.4g} N·m, "
              f"OLS+SDP {entry['methods']['ols_sdp']['heldout_vs_measurement']['aggregate']:.4g}")
    out["datasets"] = rows
    out["oracle_interior"] = oracle_interior(lcf, args.smoke)
    out["gates"] = gates(rows, out["oracle_interior"])
    return out


def oracle_interior(lcf, smoke: bool) -> dict:
    """The interior branch where it is the right one: prior = truth.

    On noise-free data with the truth as prior, the ridge minimiser is the
    truth, which is physical (every link has min eig >= 1.3e-3), so detection
    must pick the interior branch and every valid start must reach it. With
    noise even the truth prior lands on the boundary (wrist margins are
    small). This is a solver check, not an estimator: no real prior is the
    truth.
    """
    ident, _ = run_pipeline("log_cholesky", "none", None, None)
    theta_true = truth.truth_parameters()["truth"].to_numpy(dtype=float)
    nominal = np.array(list(ident.standard_parameter.values()), dtype=float)
    p = problem(ident, prior=theta_true)
    theta_ols, _ = estimate("ols", run_pipeline("ols", "none", None, None)[0])
    cands = starts(nominal, theta_ols)
    if smoke:
        cands = {k: cands[k] for k in ("nominal", "repaired_ols", "unrepaired_ols")}
    opts = lc_options(ident)
    ref = lcf.solve_log_cholesky_fit(p, options=lcf.LogCholeskyOptions(**opts))
    res = run_starts(p, cands, lcf, "auto", opts, ref_theta=theta_true)
    return {
        "dataset": {"noise": "none", "train_seed": None, "validation_seed": None},
        "prior": "truth_parameters.csv (oracle; diagnostic only)",
        "detected_branch": ref.notes.get("branch"),
        "unconstrained_min_eig": ref.notes["detection"]["unconstrained_min_eig"],
        "reference_status": ref.solver_status,
        "starts": res,
    }


# --- gates ----------------------------------------------------------------


def gates(rows, oracle) -> dict:
    g = {}
    full = [r for r in rows if r["excitation"] == "full"]
    weak = [r for r in rows if r["excitation"] == "weak"]
    clean = [r for r in full if r["noise"] == "none"]
    if clean:
        v = clean[0]["methods"]["log_cholesky"]["heldout_vs_truth"]["aggregate"]
        g["clean_heldout_rmse"] = {"value": v, "limit": CLEAN_RMSE_MAX,
                                   "met": v <= CLEAN_RMSE_MAX}
    else:
        g["clean_heldout_rmse"] = {"met": None, "note": "not run (smoke)"}
    ratios, sdp_ok = [], True
    for r in full:
        if r["noise"] == "none":
            continue
        m = r["methods"]
        sdp_ok &= m["ols_sdp"]["status"] == "ok"
        ratios.append(m["log_cholesky"]["heldout_vs_measurement"]["aggregate"]
                      / m["ols_sdp"]["heldout_vs_measurement"]["aggregate"])
    g["noisy_heldout_ratio"] = {"max_ratio": max(ratios), "limit": NOISY_RATIO_MAX,
                                "ols_sdp_all_ok": bool(sdp_ok),
                                "met": bool(sdp_ok and max(ratios) <= NOISY_RATIO_MAX)}
    e = min(r["methods"]["log_cholesky"]["min_eig"] for r in rows)
    g["feasibility"] = {"min_eig": e, "limit": FEAS_TOL, "met": e >= FEAS_TOL,
                        "note": "boundary branch constrains P >= 0; the 1e-6 "
                                "margin is used for branch detection only"}
    t = max(r["methods"]["log_cholesky"]["fit_runtime_s"] for r in rows)
    g["fit_runtime"] = {"max_s": t, "limit_s": FIT_SECONDS_MAX, "met": t <= FIT_SECONDS_MAX}
    gated = ("nominal", "repaired_ols")
    prod = all(r["starts_auto"][k]["converged"] and r["starts_auto"][k]["accepted"]
               for r in rows for k in gated)
    orc = oracle["detected_branch"] == "interior" and all(
        oracle["starts"][k]["converged"] and oracle["starts"][k]["accepted"]
        and oracle["starts"][k]["start_used"] for k in gated)
    if weak:
        ok = all(r["methods"]["log_cholesky"]["status"] == "accepted"
                 and r["methods"]["log_cholesky"]["min_eig"] >= FEAS_TOL
                 and all(r["starts_auto"][k]["converged"] and r["starts_auto"][k]["accepted"]
                         for k in gated)
                 for r in weak)
        g["weak_excitation"] = {
            "datasets": [label(r) for r in weak],
            "rank": sorted({r["methods"]["log_cholesky"]["rank"] for r in weak}),
            "met": bool(ok),
            "note": "gated on convergence and feasibility only; held-out "
                    "accuracy is reported, not gated",
        }
    g["gated_starts"] = {
        "production_auto": prod,
        "production_start_used": any(r["starts_auto"][k]["start_used"]
                                     for r in rows for k in gated),
        "oracle_interior": orc,
        "met": bool(prod and orc),
        "note": "near-boundary (perturbed) and invalid (unrepaired OLS) starts "
                "are reported, not gated",
    }
    g["all_met"] = all(v["met"] is not False for v in g.values() if isinstance(v, dict))
    return g


# --- report ---------------------------------------------------------------


def _fmt(x, f=".3g"):
    return "—" if x is None else format(x, f)


def markdown(out) -> str:
    pv = out["provenance"]
    L = [f"# {out['title']}", "",
         "> **SIMULATED DATA.** UR10 truth fixture; no hardware validation, "
         "no URDF export.", "",
         f"- examples `{pv['examples']['describe']}` ({pv['examples']['commit'][:8]}"
         f"{', dirty' if pv['examples']['dirty'] else ''}), core `{pv['core']['describe']}`"
         f" ({pv['core']['commit'][:8]}{', dirty' if pv['core']['dirty'] else ''})",
         "- " + ", ".join(f"{k} {v}" for k, v in pv["versions"].items()),
         f"- minimum core: {pv['minimum_core']}",
         f"- joints: {', '.join(pv['model']['joints'])}; effort unit N·m",
         ""]
    pre = out["preflight"]
    L += ["## Preflight", "",
          f"passed: **{pre['passed']}**; pipeline ddq is fixture ddq: "
          f"{pre.get('pipeline_ddq_is_fixture_ddq')}", ""]
    for split in ("train", "validation"):
        e = pre[split]
        L.append(f"- {split}: max ddq central difference "
                 f"{max(e['ddq_central_difference']):.2e} rad/s², min ddq RMS "
                 f"{min(e['ddq_rms'].values()):.3g} rad/s², CRBA+NLE {e['crba_nle']:.1e} N·m")
    if pre["failures"]:
        L += ["", "Failures: " + "; ".join(pre["failures"])]
        return "\n".join(L) + "\n"
    w = out["weak_excitation"]
    L += ["", f"Weak excitation: only {', '.join(w['active_joints'])} moves "
          f"({w['rows']} rows); the others hold {w['frozen_posture_rad']} rad. "
          f"{w['note']}."]
    L += ["", "## Held-out RMSE (aggregate, N·m)", "",
          "| data | seed | OLS | OLS+SDP | log-Cholesky | direct | LC vs truth | LC/SDP |",
          "|---|---|---|---|---|---|---|---|"]
    for r in out["datasets"]:
        m = r["methods"]
        h = {k: m[k]["heldout_vs_measurement"]["aggregate"] for k in METHODS}
        L.append(f"| {label(r)} | {r['train_seed']} | " + " | ".join(
            _fmt(h[k], ".5g") for k in METHODS) +
            f" | {_fmt(m['log_cholesky']['heldout_vs_truth']['aggregate'], '.3g')}"
            f" | {h['log_cholesky'] / h['ols_sdp']:.4f} |")
    L += ["", "## Per-joint held-out RMSE, first dataset of each kind (N·m)", ""]
    seen = set()
    for r in out["datasets"]:
        if label(r) in seen:
            continue
        seen.add(label(r))
        L += [f"**{label(r)}** (seed {r['train_seed']})", "",
              "| joint | " + " | ".join(METHODS) + " |", "|---" * (len(METHODS) + 1) + "|"]
        for j in truth.JOINTS:
            L.append(f"| {j} | " + " | ".join(
                _fmt(r["methods"][k]["heldout_vs_measurement"]["per_joint"][j], ".4g")
                for k in METHODS) + " |")
        L.append("")
    L += ["## Training objective, physical consistency, conditioning", "",
          "| data | seed | J OLS | J OLS+SDP | J LC | min eig OLS | min eig SDP | "
          "min eig LC | branch | rank | null | cond | LC fit s |",
          "|---" * 13 + "|"]
    for r in out["datasets"]:
        m = r["methods"]
        lc = m["log_cholesky"]
        L.append(
            f"| {label(r)} | {r['train_seed']} | {m['ols']['train_objective']:.6g} | "
            f"{m['ols_sdp']['train_objective']:.6g} | {lc['train_objective']:.6g} | "
            f"{m['ols']['min_eig']:.3g} | {m['ols_sdp']['min_eig']:.3g} | "
            f"{lc['min_eig']:.3g} | {lc['branch']} | {lc['rank']} | "
            f"{lc['null_space_dim']} | {r['condition_number']:.3g} | "
            f"{lc['fit_runtime_s']:.3f} |")
    first = out["datasets"][0]
    L += ["", f"Per-link mass (kg) and minimum pseudo-inertia eigenvalue, "
          f"{label(first)} seed {first['train_seed']}:", "",
          "| link | " + " | ".join(f"{k} mass | {k} min eig" for k in METHODS) + " |",
          "|---" * (2 * len(METHODS) + 1) + "|"]
    for j in truth.JOINTS:
        L.append(f"| {j} | " + " | ".join(
            f"{first['methods'][k]['links'][j]['mass']:.4g} | "
            f"{first['methods'][k]['links'][j]['min_eig']:.3g}" for k in METHODS) + " |")
    L += ["", "## Initialization sensitivity", ""]
    for r in out["datasets"]:
        if "starts_forced_interior" not in r:
            continue
        L += [f"**{label(r)} seed {r['train_seed']}**", "",
              "| start | auto: status | auto: start used | auto: max |Δθ| | "
              "forced interior: stop | nfev | identifiable RMS | s |",
              "|---" * 8 + "|"]
        for k, a in r["starts_auto"].items():
            f = r["starts_forced_interior"][k]
            L.append(f"| {k} | {a['solver_status']} | {a['start_used']} | "
                     f"{_fmt(a.get('max_abs_diff_to_reference'))} | "
                     f"{f['stop_reason'] or f['solver_status']} | {_fmt(f['nfev'], 'd') if f['nfev'] is not None else '—'} | "
                     f"{_fmt(f['identifiable_rms'])} | {f['runtime_s']:.2f} |")
        L.append("")
    o = out["oracle_interior"]
    L += ["**Oracle interior check** (prior = truth, noise-free; solver "
          f"diagnostic only): detected branch {o['detected_branch']}, unconstrained "
          f"min eig {o['unconstrained_min_eig']:.3g}", "",
          "| start | status | stop | nfev | identifiable RMS | max |Δθ| to truth | s |",
          "|---" * 7 + "|"]
    for k, s in o["starts"].items():
        L.append(f"| {k} | {s['solver_status']} | {s['stop_reason']} | "
                 f"{s['nfev'] if s['nfev'] is not None else '—'} | {_fmt(s['identifiable_rms'])} | "
                 f"{_fmt(s.get('max_abs_diff_to_reference'))} | {s['runtime_s']:.2f} |")
    g = out["gates"]
    L += ["", "## Decision-record gates", "",
          "| gate | value | limit | met |", "|---|---|---|---|",
          f"| clean held-out RMSE vs truth (N·m) | "
          f"{_fmt(g['clean_heldout_rmse'].get('value'))} | ≤ {CLEAN_RMSE_MAX:g} | "
          f"{g['clean_heldout_rmse']['met']} |",
          f"| noisy held-out LC / OLS+SDP (SDP ok: {g['noisy_heldout_ratio']['ols_sdp_all_ok']}) | "
          f"{g['noisy_heldout_ratio']['max_ratio']:.4f} | ≤ {NOISY_RATIO_MAX} | "
          f"{g['noisy_heldout_ratio']['met']} |",
          f"| log-Cholesky min pseudo-inertia eig, every link and fit | "
          f"{g['feasibility']['min_eig']:.3g} | ≥ {FEAS_TOL:g} | {g['feasibility']['met']} |",
          f"| log-Cholesky fit time (s) | {g['fit_runtime']['max_s']:.3f} | "
          f"≤ {FIT_SECONDS_MAX:g} | {g['fit_runtime']['met']} |",
          f"| nominal + repaired-OLS starts converge, feasible (production / oracle "
          f"interior) | {g['gated_starts']['production_auto']} / "
          f"{g['gated_starts']['oracle_interior']} | all | {g['gated_starts']['met']} |",
          *([f"| weak excitation ({', '.join(g['weak_excitation']['datasets'])}; rank "
             f"{g['weak_excitation']['rank']}): accepted, feasible, starts converge | "
             f"{g['weak_excitation']['met']} | all | {g['weak_excitation']['met']} |"]
            if "weak_excitation" in g else []),
          "", f"All gates met: **{g['all_met']}**. {g['gated_starts']['note']}.", ""]
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
                    help="low-noise seed 101, full and weak excitation, three starts")
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
