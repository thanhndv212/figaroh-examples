"""UR10 physical-estimator comparison, second pass (#22, D4).

Extends the first pass (``compare_physical_estimators.py``, #94; its script,
protocol and result files are unchanged) on the same frozen truth fixture:

* nonlinear log-Cholesky candidates (core private spike, started from the
  repaired nominal and the repaired OLS representative) next to base OLS,
  exact reconstruction, direct effort fit and per-link projection;
* **joint-extra** runs for every physical method (friction and the
  independent actuator-inertia columns are estimated, exactly eliminated by
  variable projection; see ``examples/physical_comparison.py``), kept apart
  from the frozen-extra runs of the first pass;
* a second-solver cross-check (cvxopt against QICS, both through PICOS) of the
  comparator objectives and of the phase-I certificate.

Every method is scored with one common objective on the problem it solves.
Solver convergence (log-Cholesky: scipy status 1, 2 or 4 within 2000
evaluations; core #30 recorded a no-go on termination, so non-convergence
is expected and is **not** treated as infeasibility) and physical
feasibility (independent pseudo-inertia check) are separate fields.
Training and held-out errors are separate fields.

    python compare_physical_extended.py --output <json>
    python compare_physical_extended.py --noise none --seeds 1 --weighting none
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).parent
project_root = HERE.parents[1]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from examples import physical_comparison as pc  # noqa: E402
from examples.ur10 import compare_physical_estimators as first  # noqa: E402
from examples.ur10 import identification_truth as it  # noqa: E402

FIRST_PASS_RESULT = first.DEFAULT_OUT_DIR / "ur10-physical-comparison-{profile}.json"


def make_spec(case, proto, pcmp) -> pc.Spec:
    names_extra = first.extra_names()
    scale_b = np.array(list(proto["scaling"]["base_column_rms"].values()))
    Mp = first.protocol_basis(pcmp, case.tp, proto)

    def truth_report(theta):
        base_err = (Mp @ (theta - case.truth)) * scale_b
        return {
            "base_error_rms_nm": float(np.sqrt(np.mean(base_err**2))),
            "base_error_max_nm": float(np.abs(base_err).max()),
            "links": first.link_report(theta, case.truth, case.prior),
            "standard_error_rel_to_truth": float(
                np.linalg.norm(theta - case.truth) / np.linalg.norm(case.truth)
            ),
        }

    return pc.Spec(
        Y=case.Y_std[:, :60],
        E=case.Y_std[:, 60:],
        tau=case.tau,
        Yv=case.Yv_std[:, :60],
        Ev=case.Yv_std[:, 60:],
        names=list(case.names),
        extra_names=names_extra,
        links=list(it.JOINTS),
        joints=list(it.JOINTS),
        prior=case.prior,
        e_fix=np.zeros(18),  # = truth (the fixture has no friction or Ia)
        heldout={"true_effort": case.tauv_true, "noisy_effort": case.tauv_noisy},
        row_weight=case.row_weight,
        joint_units=["N.m"] * 6,
        nrmse_scale=case.effort_scale,
        truth_report=truth_report,
    )


def provenance(pcmp, spike_info) -> dict:
    import cvxopt
    import picos

    d = first.provenance(pcmp)
    d["script_sha256"] = pc.sha256(Path(__file__))
    d["first_pass_script_sha256"] = pc.sha256(Path(first.__file__))
    d["shared_module_sha256"] = pc.sha256(Path(pc.__file__))
    d["log_cholesky_spike"] = spike_info
    d["log_cholesky_settings"] = {
        **{k: list(v) if isinstance(v, tuple) else v for k, v in pc.LOGCHOL.items()},
        "prior_strength": "PhysicalPolicy.prior_weight (same lam as direct fit)",
    }
    try:
        import qics

        d["qics"] = qics.__version__
    except Exception:  # second solver optional
        d["qics"] = None
    d["picos_solvers"] = picos.available_solvers()
    d["cvxopt"] = cvxopt.__version__
    return d


def regression_vs_first_pass(cases, profile) -> dict:
    """Frozen-extra rows must equal the first pass (same inputs, same solvers)."""
    path = Path(str(FIRST_PASS_RESULT).format(profile=profile))
    if not path.exists():
        return {"checked": False, "reason": f"{path.name} not found"}
    old = json.loads(path.read_text())
    idx = {
        (c["derivatives"], c["noise"], c["seed"], c["weighting"]): c
        for c in old["cases"]
    }
    worst, n = 0.0, 0
    for c in cases:
        o = idx.get((c["derivatives"], c["noise"], c["seed"], c["weighting"]))
        if o is None:
            continue
        for m in (
            "base_ols",
            "exact_reconstruction",
            "direct_effort_fit",
            "per_link_projection",
        ):
            new = c["frozen_extra"]["methods"][m]
            ref = o["frozen_extra"]["methods"][m]
            if "heldout" not in new or "heldout" not in ref:
                continue
            a = np.array(list(new["heldout"]["nrmse_pct_vs_true_effort"].values()))
            b = np.array(list(ref["heldout"]["nrmse_pct"].values()))
            worst = max(worst, float(np.abs(a - b).max()))
            n += 1
    return {
        "checked": True,
        "rows_compared": n,
        "max_abs_diff_nrmse_pct": worst,
        "source": path.name,
    }


def summarize(cases: list, mode: str) -> list:
    groups = {}
    for c in cases:
        for m, r in c[mode]["methods"].items():
            entries = r.items() if m == "log_cholesky" else [(None, r)]
            for sub, e in entries:
                key = (c["weighting"], c["noise"], m if sub is None else f"{m}[{sub}]")
                groups.setdefault(key, []).append(e)
    rows = []
    for (wt, noise, m), allr in groups.items():
        solved = [r for r in allr if "heldout" in r]
        row = {
            "mode": mode,
            "weighting": wt,
            "noise": noise,
            "method": m,
            "cases": len(allr),
            "solved": len(solved),
            "converged": sum(bool(r["convergence"].get("converged")) for r in allr),
            "feasible": sum(
                bool(r.get("feasibility", {}).get("all_links_ok")) for r in solved
            ),
        }
        if solved:
            nr = np.array(
                [
                    list(r["heldout"]["nrmse_pct_vs_true_effort"].values())
                    for r in solved
                ]
            )
            rm = np.array(
                [list(r["heldout"]["rmse_vs_true_effort"].values()) for r in solved]
            )
            tr = np.array(
                [list(r["train"]["rmse_vs_measured_effort"].values()) for r in solved]
            )
            row.update(
                {
                    "train_rmse_nm_mean_per_joint": tr.mean(axis=0).tolist(),
                    "heldout_rmse_nm_mean_per_joint": rm.mean(axis=0).tolist(),
                    "heldout_nrmse_pct_mean_per_joint": nr.mean(axis=0).tolist(),
                    "heldout_nrmse_pct_worst": float(nr.max()),
                    "base_err_rms_nm_mean": float(
                        np.mean([r["truth"]["base_error_rms_nm"] for r in solved])
                    ),
                    "base_change_vs_ols_rel_mean": float(
                        np.mean([r["base_change_vs_ols_rel"] for r in solved])
                    ),
                    "objective_rel_excess_vs_direct_mean": float(
                        np.mean(
                            [
                                r["objective"].get("relative_excess_vs_direct", np.nan)
                                for r in solved
                            ]
                        )
                    ),
                }
            )
            eigs = [
                r["feasibility"]["min_eig_min"]
                for r in solved
                if r.get("feasibility", {}).get("min_eig_min") is not None
            ]
            row["min_eig_min"] = min(eigs) if eigs else None
        rows.append(row)
    return rows


def run(args) -> dict:
    from figaroh.identification import _physical_comparator as pcmp

    t0 = time.perf_counter()
    tp = it.truth_parameters()
    proto = it.protocol()
    spike, spike_info = pc.load_spike(pcmp)
    out = {
        "issue": "figaroh-examples#22",
        "pass": "second (extends ur10-physical-comparison-<profile>.json)",
        "protocol_version": proto["protocol_version"],
        "units": first.UNITS,
        "joint_order": it.JOINTS,
        "joint_units": {j: "N.m" for j in it.JOINTS},
        "provenance": provenance(pcmp, spike_info),
        "settings": {
            "derivatives": args.derivatives,
            "noise": args.noise,
            "seeds": args.seeds,
            "weighting": args.weighting,
            "solver": args.solver,
            "second_solver": args.second_solver,
            "policy": repr(pcmp.PhysicalPolicy()),
            "common_objective": "J = ||W(Y theta - tau)||^2 + lam||(theta-theta0)/s||^2; "
            "lam = PhysicalPolicy.prior_weight, theta0 = nominal URDF, s = comparator "
            "coord_scale; reported as data + reg for every method",
            "frozen_extra": "fv, fs, Ia fixed at 0 (= truth)",
            "joint_extra": "independent extras estimated with theta (variable "
            "projection); extras collinear with the inertial base are absorbed and "
            "listed per case",
            "cross_check": "cvxopt vs QICS on the first seed of every noise/weighting "
            "cell, both modes",
        },
        "cases": [],
        "cross_check": [],
    }
    for der in args.derivatives:
        for noise in args.noise:
            pairs = (
                [(None, None)]
                if noise == "none"
                else list(zip(it.NOISE_SEEDS["train"], it.NOISE_SEEDS["validation"]))[
                    : args.seeds
                ]
            )
            for wt in args.weighting:
                for si, (seed, vseed) in enumerate(pairs):
                    case = first.Case(der, noise, seed, vseed, wt, tp, proto)
                    spec = make_spec(case, proto, pcmp)
                    entry = dict(case.key)
                    for mode, label in (
                        ("frozen", "frozen_extra"),
                        ("joint", "joint_extra"),
                    ):
                        b, methods = pc.run_methods(
                            spec,
                            mode,
                            pcmp,
                            spike,
                            args.solver,
                            second_solver=args.second_solver or None,
                        )
                        entry[label] = {
                            "methods": methods,
                            "base_rank": len(b.p.base_indices),
                        }
                        if mode == "joint":
                            entry[label]["extras_kept"] = [
                                spec.extra_names[c] for c in b.kept
                            ]
                            entry[label]["extras_absorbed"] = [
                                spec.extra_names[c] for c in b.absorbed
                            ]
                        if si == 0 and args.second_solver:
                            cc = pc.cross_check(
                                spec, mode, pcmp, [args.solver, args.second_solver]
                            )
                            out["cross_check"].append(
                                {
                                    **case.key,
                                    "mode": mode,
                                    "solvers": cc,
                                    "agreement": pc.agree(
                                        cc, args.solver, args.second_solver
                                    ),
                                }
                            )
                    out["cases"].append(entry)
                    lc = entry["frozen_extra"]["methods"]["log_cholesky"]
                    print(
                        f"{der:12s} {noise:5s} seed={seed} {wt:6s} "
                        + " ".join(
                            f"{k}:{'conv' if v['convergence']['converged'] else 'nfev' + str(v['convergence']['nfev'])}"
                            for k, v in lc.items()
                        ),
                        flush=True,
                    )
    out["summary"] = {
        "frozen_extra": summarize(out["cases"], "frozen_extra"),
        "joint_extra": summarize(out["cases"], "joint_extra"),
    }
    out["first_pass_regression"] = regression_vs_first_pass(
        out["cases"], out["provenance"]["pinocchio_profile"]
    )
    out["runtime_s"] = time.perf_counter() - t0
    return out


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--derivatives",
        nargs="+",
        default=["analytic"],
        choices=["analytic", "differentiated"],
    )
    parser.add_argument(
        "--noise",
        nargs="+",
        default=["none", "low", "high"],
        choices=["none", "low", "high"],
    )
    parser.add_argument("--seeds", type=int, default=5)
    parser.add_argument(
        "--weighting", nargs="+", default=["none", "scaled"], choices=["none", "scaled"]
    )
    parser.add_argument("--solver", default="cvxopt")
    parser.add_argument(
        "--second-solver", default="qics", help="'' disables the cross-check"
    )
    parser.add_argument("--output", type=Path)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args(argv)
    if not 1 <= args.seeds <= 5:
        parser.error("--seeds must be 1..5")
    result = run(args)
    out = args.output or (
        first.DEFAULT_OUT_DIR
        / f"ur10-physical-extended-{result['provenance']['pinocchio_profile']}.json"
    )
    if out.exists() and not args.overwrite:
        parser.error(f"{out} exists; earlier results are preserved (--overwrite)")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=1) + "\n")
    print(f"results written to {out} ({result['runtime_s']:.0f} s)")
    print("first-pass regression:", result["first_pass_regression"])


if __name__ == "__main__":
    main()
