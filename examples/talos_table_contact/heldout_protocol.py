#!/usr/bin/env python3
"""TALOS table-contact: held-out protocol on the real postures (#98).

Every model is fitted on a chain's training session and scored on its
validation session, through the shared ``examples/calibration_harness``
(``data/real/protocol_{left,right}.yaml``, sha256-verified plan views of the
master ``data/real/protocol.yaml``). There is no confirmation session, so
the validation role is the only held-out evidence: nothing is selected on it
(the config default stays ``structural`` + ``ordered``) and
``harness.select(..., role="confirmation")`` raises.

Models, all on the chain's own training file with the nominal table
re-anchored at the table-only fit (``run_calibration_real_data._build_chain``):

- ``table+contact only``: plane z/roll/pitch and the contact tilt, no joint
  parameters (the TALOS counterpart of TIAGo Pro's registration only);
- ``structural``: the reference configuration (ordered basis, ridge 1e-3);
- ``map``, ``map_cv``, ``excitation``, ``cv_subset``: core's estimation
  methods with core's default priors, ``cv_folds`` and ``cv_multipliers``.
  ``noise_std`` is given explicitly (:data:`NOISE_STD`); the posture count
  is too small to estimate it.

A two-chain structural fit (shared torso, ``MultiChainCalibration``) is a
diagnostic row, not a protocol model.

Metrics per held-out set, each as z (mm), roll (deg), pitch (deg) of the
flush-contact gap:

- ``transfer`` (**primary**, :data:`PRIMARY_METRIC` is its z): the training
  table and contact carried over unchanged;
- ``rereg``: only the 3 plane parameters are refitted on the evaluated
  session (Delta X and contact frozen). Optimistic: it spends 3 of the
  27 residuals of a 9-posture set on the evaluated data.

The training z is the fit's own in-sample z, so the train/held-out gap is
read per method.

Run from this directory::

    python heldout_protocol.py [--chains left right] [--methods ...]
                               [--json] [--no-plot]
"""

from __future__ import annotations

import argparse
import contextlib
import io
import json
import sys
import time
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

HERE = Path(__file__).resolve().parent
project_root = HERE.parents[1]
for p in (str(project_root), str(HERE)):
    if p not in sys.path:
        sys.path.insert(0, p)

from examples.calibration_harness import heldout as harness  # noqa: E402
from generate_synthetic_data import _load_robot  # noqa: E402
from run_calibration_real_data import (  # noqa: E402
    DATA_DIR,
    LEFT_CONFIG,
    RIGHT_CONFIG,
    _build_chain,
    _build_two_chain,
    _load_real_touches,
    _load_split,
)

PRIMARY_METRIC = "transfer_z_mm"
SECONDARY_METRIC = "rereg_z_mm"

#: Gap noise handed to the regularised methods (m for z, rad for the tilts):
#: the order of the structural fit's own training residual (0.8 to 1.4 mm).
NOISE_STD = 1e-3

CHAINS = {
    "left": (LEFT_CONFIG, DATA_DIR / "protocol_left.yaml"),
    "right": (RIGHT_CONFIG, DATA_DIR / "protocol_right.yaml"),
}

#: label -> (fit level, estimation overrides). ``None`` keeps the config's
#: (``structural`` + ``ordered``).
MODELS = {
    "table+contact only": ("table_contact", None),
    "structural": (None, None),
    "map": (None, {"method": "map", "noise_std": NOISE_STD}),
    "map_cv": (None, {"method": "map_cv", "noise_std": NOISE_STD}),
    "excitation": (None, {"method": "excitation", "noise_std": NOISE_STD}),
    "cv_subset": (None, {"method": "cv_subset", "noise_std": NOISE_STD}),
}
#: kept parameter set depends on the platform (ties in the selection)
PLATFORM_DEPENDENT = ("structural", "cv_subset")
TWO_CHAIN = "two-chain structural (diagnostic)"

_KEYS = (("z", "z_rmse_mm"), ("roll", "roll_rmse_deg"), ("pitch", "pitch_rmse_deg"))


class Fitted:
    """A fitted chain: the calibrator, its solution and its training gap."""

    def __init__(self, side, calib, x, seconds):
        self.side = side
        self.calib = calib
        self.x = np.asarray(x, dtype=float)
        self.train = calib.gap_metrics(self.x)
        self.n_params = len(calib.calib_config["param_name"])
        self.n_dx = int(calib.n_deltaX)
        self.seconds = seconds


def _quiet():
    return contextlib.redirect_stdout(io.StringIO())


def fit_model(robot, side, label, train_csv) -> Fitted:
    level, estimation = MODELS[label]
    config = CHAINS[side][0]
    t0 = time.time()
    with _quiet():
        calib = _build_chain(
            robot,
            config,
            train_csv,
            asset_id=f"talos-{side}",
            estimation=estimation,
            fit_level=level,
        )
        result = calib.solve(
            method="lm",
            max_iterations=1 if level == "table_contact" else 3,
            enable_logging=False,
            html_report=False,
        )
    return Fitted(side, calib, result.x, time.time() - t0)


def reregister(calib, x) -> np.ndarray:
    """``x`` with the plane parameters refitted on the calibrator's current
    data (everything else frozen). Starts from the training plane."""
    names = list(calib.calib_config["param_name"])
    free = [i for i, n in enumerate(names) if n.startswith("plane_")]
    n_gap = 3 * len(calib.q_measured)
    x0 = np.array(x, dtype=float)

    def residual(v):
        y = x0.copy()
        y[free] = v
        return calib.cost_function(y)[:n_gap]

    sol = least_squares(residual, x0[free], method="lm")
    out = x0.copy()
    out[free] = sol.x
    return out


def gap_errors(calib, x, df) -> dict:
    """Transfer and re-registered gap metrics of ``x`` on the session ``df``
    (leaves the calibrator pointing at ``df``)."""
    _load_split(calib, df)
    transfer = calib.gap_metrics(x)
    rereg = calib.gap_metrics(reregister(calib, x))
    return {
        **{f"transfer_{k}_{u}": transfer[m] for k, m in _KEYS for u in [_u(m)]},
        **{f"rereg_{k}_{u}": rereg[m] for k, m in _KEYS for u in [_u(m)]},
        "n": len(df),
    }


def _u(metric: str) -> str:
    return "mm" if metric.endswith("_mm") else "deg"


def _strata(plan, session_path, train_path, calib, x, df) -> dict:
    """Held-out postures by relation to the training postures, with the
    transfer z error of each stratum (needs ``calib`` on ``df``)."""
    spec = plan.spec.strata
    joints = list(spec.joints)
    masks = harness.posture_strata(
        df[joints].to_numpy(),
        _load_real_touches(train_path)[joints].to_numpy(),
        spec.repeated_tol,
        spec.out_of_range_margin,
    )
    n = len(df)
    gap = calib.cost_function(x)[: 3 * n].reshape(3, n)[0] * 1000.0
    return {
        k: {
            "n": int(m.sum()),
            "transfer_z_mm": (
                float(np.sqrt(np.mean(gap[m] ** 2))) if m.any() else float("nan")
            ),
        }
        for k, m in masks.items()
    }


def make_evaluate(plan):
    train_path = plan.folds[0].training[0].path

    def evaluate(fitted: Fitted, held: harness.HeldoutSet) -> dict:
        df = _load_real_touches(held.path)
        out = gap_errors(fitted.calib, fitted.x, df)
        out["strata"] = _strata(plan, held.path, train_path, fitted.calib, fitted.x, df)
        return out

    return evaluate


def run_chain(side, robot=None, methods=None) -> dict:
    """``{model: record}`` for one chain; ``record["sets"][file]`` holds the
    held-out metrics and ``"_harness"`` the ``harness.run`` structure."""
    robot = robot or _load_robot()
    plan = harness.load_plan(CHAINS[side][1])  # verifies every sha256
    train_csv = plan.folds[0].training[0].path
    labels = list(MODELS) if methods is None else list(methods)
    fitted = {}

    def build(label):
        def go(fold):
            fitted[label] = fit_model(robot, side, label, train_csv)
            return fitted[label]

        return go

    res = harness.run(plan, {m: build(m) for m in labels}, make_evaluate(plan))
    out = {}
    for label in labels:
        f = fitted[label]
        out[label] = {
            "n_params": f.n_params,
            "n_dx": f.n_dx,
            "train_z_mm": f.train["z_rmse_mm"],
            "train_roll_deg": f.train["roll_rmse_deg"],
            "train_pitch_deg": f.train["pitch_rmse_deg"],
            "seconds": f.seconds,
            "sets": res[label][0],
        }
    out["_harness"] = res
    out["_label"] = plan.label
    return out


def run_two_chain(robot=None) -> dict:
    """Two-chain structural fit (shared torso): ``{side: record}``.

    A diagnostic row; the chains' tables are anchored at their own table-only
    fits, as in the single-chain models."""
    robot = robot or _load_robot()
    t0 = time.time()
    with _quiet():
        left, right, coupler = _build_two_chain(robot)
        result = coupler.solve_lm()
    seconds = time.time() - t0
    train = coupler.gap_metrics(result.x)
    x_chain = dict(zip(("left", "right"), coupler._split_to_chain_vars(result.x)))
    calibs = {"left": left, "right": right}
    out = {}
    for side in ("left", "right"):
        plan = harness.load_plan(CHAINS[side][1])
        held = plan.folds[0].evaluated[0]
        calib, x = calibs[side], x_chain[side]
        df = _load_real_touches(held.path)
        m = gap_errors(calib, x, df)
        m["strata"] = _strata(
            plan, held.path, plan.folds[0].training[0].path, calib, x, df
        )
        out[side] = {
            "n_params": len(coupler.param_name),
            "n_dx": coupler.n_dx,
            "train_z_mm": train[side]["z_rmse_mm"],
            "train_roll_deg": train[side]["roll_rmse_deg"],
            "train_pitch_deg": train[side]["pitch_rmse_deg"],
            "seconds": seconds,
            "sets": {held.name: m},
        }
    return out


def select_model(results: dict) -> tuple:
    """Lowest mean primary error on the validation role. Informational only:
    the config default is not changed by it."""
    return harness.select(results["_harness"], PRIMARY_METRIC, role="validation")


# --------------------------------------------------------------------------
# Output
# --------------------------------------------------------------------------
def _fmt_row(label, rec, held):
    e = rec["sets"][held]
    return (
        f"  {label:34s} {rec['n_params']:3d} {rec['n_dx']:3d} "
        f"{rec['train_z_mm']:7.2f} | "
        f"{e['transfer_z_mm']:7.2f} {e['transfer_roll_deg']:6.3f} "
        f"{e['transfer_pitch_deg']:6.3f} | "
        f"{e['rereg_z_mm']:7.2f} {e['rereg_roll_deg']:6.3f} "
        f"{e['rereg_pitch_deg']:6.3f} | {rec['seconds']:6.1f}"
    )


def print_chain(side, results, two=None) -> None:
    labels = [m for m in results if not m.startswith("_")]
    held = next(iter(results[labels[0]]["sets"]))
    print(f"\n{side} chain: {results['_label']}; held-out {held}")
    print(
        "  gap RMS: z mm, roll deg, pitch deg. transfer = training table "
        "carried over (primary);\n  re-registered = 3 plane parameters refitted "
        "on the held-out session (in-sample optimistic: 3 of 27 residuals)"
    )
    print(
        f"  {'model':34s} {'par':>3s} {'dx':>3s} {'train z':>7s} | "
        f"{'transfer z':>7s} {'roll':>6s} {'pitch':>6s} | "
        f"{'rereg z':>7s} {'roll':>6s} {'pitch':>6s} | {'sec':>6s}"
    )
    for label in labels:
        print(_fmt_row(label, results[label], held))
    if two is not None:
        print(_fmt_row(TWO_CHAIN, two[side], held))
    for label in labels:
        s = results[label]["sets"][held]["strata"]
        print(
            f"  strata (transfer z) {label}: "
            + "  ".join(
                f"{k} ({v['n']}) {v['transfer_z_mm']:.2f}" for k, v in s.items()
            )
        )
    best, scores = select_model(results)
    print(
        f"  lowest {PRIMARY_METRIC} on the validation role: {best} "
        "(informational; the config default is unchanged, no confirmation set)"
    )


def snapshot(results: dict, two: dict | None) -> dict:
    """The golden's real block, from ``run_chain`` results per side."""

    def rec(r):
        return {
            "n_params": r["n_params"],
            "n_dx": r["n_dx"],
            "train_z_mm": round(r["train_z_mm"], 3),
            "sets": {
                n: {
                    **{
                        k: round(v, 3)
                        for k, v in e.items()
                        if k.startswith(("transfer_", "rereg_"))
                    },
                    "n": e["n"],
                    "strata": {k: v["n"] for k, v in e["strata"].items()},
                }
                for n, e in r["sets"].items()
            },
        }

    out = {"primary_metric": PRIMARY_METRIC, "noise_std": NOISE_STD, "chains": {}}
    for side, res in results.items():
        labels = [m for m in res if not m.startswith("_")]
        out["chains"][side] = {
            "models": {m: rec(res[m]) for m in labels},
            "lowest_validation": select_model(res)[0],
        }
        if two is not None:
            out["chains"][side]["models"][TWO_CHAIN] = rec(two[side])
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--chains", nargs="+", choices=list(CHAINS), default=list(CHAINS))
    ap.add_argument("--methods", nargs="+", choices=list(MODELS), default=None)
    ap.add_argument("--json", action="store_true", help="print the golden block")
    ap.add_argument("--no-plot", action="store_true", help="accepted; nothing plots")
    ap.add_argument(
        "--no-two-chain", action="store_true", help="skip the diagnostic row"
    )
    args = ap.parse_args()

    robot = _load_robot()
    t0 = time.time()
    results = {s: run_chain(s, robot, args.methods) for s in args.chains}
    two = None
    if not args.no_two_chain and set(args.chains) == set(CHAINS):
        two = run_two_chain(robot)
    if args.json:
        print(json.dumps(snapshot(results, two), indent=2, sort_keys=True))
        return
    for side in args.chains:
        print_chain(side, results[side], two)
    print(f"\nTotal runtime {time.time() - t0:.0f} s")


if __name__ == "__main__":
    main()
