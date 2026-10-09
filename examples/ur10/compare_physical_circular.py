"""UR10 physical-estimator comparison with periodic (circular) extras (#22, D4).

The first and second passes use a fixture whose friction, actuator-inertia and
offset extras are zero in the truth. This run adds a *periodic* extra to the
truth: a position-periodic torque per joint (cogging / encoder-eccentricity
like), ``a_j sin(q_j) + b_j cos(q_j)`` with amplitude 2 % of the joint's
protocol effort scale and a fixed phase, added to the fixture effort of the
training and held-out splits (measured ``q`` is used for both the regressor and
the added torque). The twelve extra columns ``sin_<joint>``, ``cos_<joint>``
are appended to the usual 18 friction/inertia extras and three variants are
compared on the same data:

* ``frozen_zero``  - periodic extras fixed at 0 (model misspecified),
* ``joint_extra``  - periodic extras estimated with the parameters (exact
  variable-projection elimination),
* ``frozen_truth`` - periodic extras fixed at their true value (oracle).

Methods, objective, convergence/feasibility fields and the second-solver retry
are those of ``compare_physical_extended.py`` (unchanged). Earlier result files
are not touched.

    python compare_physical_circular.py --noise none low high --seeds 3
"""

from __future__ import annotations

import argparse
import dataclasses
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
from examples.ur10 import compare_physical_extended as ext  # noqa: E402
from examples.ur10 import identification_truth as it  # noqa: E402

AMPLITUDE_FRACTION = 0.02
PHASE_RAD = np.array([0.3, 1.1, 2.0, 2.9, 3.8, 4.7])
VARIANTS = (
    ("frozen_zero", "frozen"),
    ("joint_extra", "joint"),
    ("frozen_truth", "frozen"),
)


def truth_coefficients(effort_scale):
    amp = AMPLITUDE_FRACTION * np.asarray(effort_scale)
    return amp * np.sin(PHASE_RAD), amp * np.cos(PHASE_RAD)  # a_j (sin), b_j (cos)


def periodic_columns(q):
    """(6 n, 12) joint-major columns: sin_j, cos_j for j in joints."""
    n, nj = q.shape
    cols = np.zeros((nj * n, 2 * nj))
    for j in range(nj):
        rows = slice(j * n, (j + 1) * n)
        cols[rows, j] = np.sin(q[:, j])
        cols[rows, nj + j] = np.cos(q[:, j])
    return cols


def circular_spec(case, proto, pcmp):
    base = ext.make_spec(case, proto, pcmp)
    k = case.key
    train = it.load_split("train", k["derivatives"], k["noise"], k["seed"])
    val = it.load_split("validation", k["derivatives"], k["noise"], k["val_seed"])
    a, b = truth_coefficients(case.effort_scale)
    coef = np.concatenate([a, b])
    Ct, Cv = periodic_columns(train["q"]), periodic_columns(val["q"])
    names = [f"sin_{j}" for j in it.JOINTS] + [f"cos_{j}" for j in it.JOINTS]
    spec = dataclasses.replace(
        base,
        E=np.c_[base.E, Ct],
        Ev=np.c_[base.Ev, Cv],
        tau=base.tau + Ct @ coef,
        extra_names=list(base.extra_names) + names,
        e_fix=np.concatenate([base.e_fix, np.zeros(12)]),
        heldout={name: y + Cv @ coef for name, y in base.heldout.items()},
    )
    return spec, coef


def run(args) -> dict:
    from figaroh.identification import _physical_comparator as pcmp

    t0 = time.perf_counter()
    tp, proto = it.truth_parameters(), it.protocol()
    spike, spike_info = pc.load_spike(pcmp)
    out = {
        "issue": "figaroh-examples#22",
        "pass": "second, periodic (circular) extras",
        "protocol_version": proto["protocol_version"],
        "units": first.UNITS,
        "joint_order": it.JOINTS,
        "joint_units": {j: "N.m" for j in it.JOINTS},
        "provenance": ext.provenance(pcmp, spike_info),
        "settings": {
            "derivatives": "analytic",
            "noise": args.noise,
            "seeds": args.seeds,
            "weighting": args.weighting,
            "solver": args.solver,
            "second_solver": args.second_solver,
            "periodic_extra": {
                "form": "a_j sin(q_j) + b_j cos(q_j) per joint, measured q",
                "amplitude_fraction_of_effort_scale": AMPLITUDE_FRACTION,
                "phase_rad": PHASE_RAD.tolist(),
                "units": "N.m",
            },
            "variants": {
                "frozen_zero": "periodic extras fixed at 0 (misspecified)",
                "joint_extra": "periodic extras estimated (variable projection)",
                "frozen_truth": "periodic extras fixed at truth (oracle)",
            },
        },
        "cases": [],
        "cross_check": [],
    }
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
                case = first.Case("analytic", noise, seed, vseed, wt, tp, proto)
                spec, coef = circular_spec(case, proto, pcmp)
                entry = dict(case.key)
                entry["true_coefficients_nm"] = {
                    n: float(c) for n, c in zip(spec.extra_names[18:], coef)
                }
                for label, mode in VARIANTS:
                    s = spec
                    if label == "frozen_truth":
                        s = dataclasses.replace(
                            spec, e_fix=np.concatenate([np.zeros(18), coef])
                        )
                    b, methods = pc.run_methods(
                        s,
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
                        entry[label]["extras_kept"] = [s.extra_names[c] for c in b.kept]
                        entry[label]["extras_absorbed"] = [
                            s.extra_names[c] for c in b.absorbed
                        ]
                        d = methods["direct_effort_fit"]
                        if "extras" in d:
                            entry[label]["direct_extras"] = d["extras"]
                    if label == "joint_extra" and si == 0 and args.second_solver:
                        cc = pc.cross_check(
                            s, mode, pcmp, [args.solver, args.second_solver]
                        )
                        out["cross_check"].append(
                            {
                                **case.key,
                                "mode": label,
                                "solvers": cc,
                                "agreement": pc.agree(
                                    cc, args.solver, args.second_solver
                                ),
                            }
                        )
                out["cases"].append(entry)
                print(f"{noise:5s} seed={seed} {wt:6s} done", flush=True)
    out["summary"] = {
        label: ext.summarize(out["cases"], label) for label, _ in VARIANTS
    }
    out["runtime_s"] = time.perf_counter() - t0
    return out


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--noise",
        nargs="+",
        default=["none", "low", "high"],
        choices=["none", "low", "high"],
    )
    parser.add_argument("--seeds", type=int, default=3)
    parser.add_argument(
        "--weighting", nargs="+", default=["none", "scaled"], choices=["none", "scaled"]
    )
    parser.add_argument("--solver", default="cvxopt")
    parser.add_argument("--second-solver", default="qics")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args(argv)
    result = run(args)
    profile = result["provenance"]["pinocchio_profile"]
    path = (
        args.output or first.DEFAULT_OUT_DIR / f"ur10-physical-circular-{profile}.json"
    ).resolve()
    if path.exists() and not args.overwrite:
        sys.exit(f"{path} exists; pass --overwrite to replace it")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(result, indent=1, allow_nan=False) + "\n")
    print(f"results written to {path} ({result['runtime_s']:.0f} s)")


if __name__ == "__main__":
    main()
