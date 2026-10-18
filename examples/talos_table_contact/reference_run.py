#!/usr/bin/env python3
"""TALOS table-contact calibration reference workflow (#98).

One headless command, per chain (left, right): fit the reference model on the
chain's frozen training session, score the table-only baseline, the
structural fit and the MAP fit on the held-out validation session, verify
the reference fit and archive the run with its reproduction record.

    python reference_run.py --no-plot                  # from this directory
    python reference_run.py --chains left --root /tmp/runs

Per chain the run directory is
``<root>/talos-{left,right}/calibration/<timestamp>/``:

0. **Inputs**: the plan views ``data/real/protocol_{left,right}.yaml`` are
   loaded first (sha256 of every session file is checked), and the fit's
   training file must be the plan's training session, else the run stops
   before writing anything.
1. **Fit** (``talos_table_{left,right}_config.yaml``: ``structural`` +
   ``ordered``, the reference configuration): ``report.html``.
2. **Held-out report** (``heldout.json``) on the validation session, for
   ``table+contact only``, ``structural`` and ``map``
   (:data:`heldout_protocol.NOISE_STD`): the primary ``transfer`` error (the
   training table and contact carried over), the labelled optimistic
   ``rereg`` error (plane refitted on the evaluated session, 3 of 27
   residuals) and the training z. There is no confirmation session, so
   nothing is selected on the validation role.
3. **Corrections and gauge** (``corrections.json``): fitted table and
   contact parameters and the Delta X with standard errors. ``contact_z`` is
   collinear with ``plane_z`` (only their sum is observable; the baseline and
   map fix it at nominal, the structural reference lets the ridge split it);
   the contact x, y and yaw are unobservable on a flat table.
4. **Verdict and archive**: ``verdict.json``, ``provenance.json``,
   ``reproduction.json``; the archive audit must find nothing missing.

The full six-model protocol (map_cv, excitation, cv_subset and the two-chain
diagnostic) stays in ``heldout_protocol.py`` and its slow golden. Exits
nonzero when a verdict fails or an archive is incomplete.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
project_root = HERE.parents[1]
for p in (str(project_root), str(HERE)):
    if p not in sys.path:
        sys.path.insert(0, p)

import heldout_protocol as hp  # noqa: E402
from examples.calibration_harness import heldout as harness  # noqa: E402
from examples.run_record import (  # noqa: E402
    audit,
    describe_file,
    missing,
    write_reproduction_record,
)
from examples.verification import add_verification_args, run_verification  # noqa: E402
from figaroh.tools.run_archive import archive_run, compute_run_dir  # noqa: E402
from generate_synthetic_data import _load_robot  # noqa: E402

#: table-only baseline, the reference configuration and the MAP fit
MODELS = ("table+contact only", "structural", "map")
REFERENCE = "structural"
MASTER_PROTOCOL = hp.DATA_DIR / "protocol.yaml"

GAUGE_NOTE = (
    "plane_z and contact_z are collinear on flush contacts: only their sum is "
    "observable. The table-only baseline and the map fit fix contact_z at "
    "its nominal value, so plane_z absorbs its error; the structural fit "
    "(the archived reference) keeps contact_z free and the 1e-3 ridge picks "
    "the minimum-norm split. Either way plane_z is not the table height. "
    "The contact x, y and yaw are unobservable on a flat table and not "
    "estimated."
)
REREG_NOTE = (
    "optimistic: the 3 plane parameters are refitted on the evaluated "
    "session (3 of its 27 residuals); the primary metric is transfer"
)


def check_protocol(calib, plan) -> None:
    """The fit's training file is the plan's training session.

    Raises ValueError otherwise. The sha256 of every session file was checked
    when the plan was loaded.
    """
    name = Path(calib.calib_config.get("data_file") or "").name
    train = plan.folds[0].training[0].path.name
    # _build_chain stages a converted copy next to the raw training file
    if not name.startswith(train):
        raise ValueError(
            f"protocol mismatch: data_file {name!r} is not the plan's "
            f"training session {train!r}"
        )
    if calib.calib_config.get("validation_data_file"):
        raise ValueError("protocol mismatch: a validation file was fitted on")


def _nan_none(x):
    return None if isinstance(x, float) and np.isnan(x) else x


def heldout_report(records: dict) -> dict:
    """``heldout.json``: per model the training z and, per held-out session,
    the transfer (primary) and labelled re-registered errors."""
    out = {
        "primary_metric": hp.PRIMARY_METRIC,
        "noise_std": hp.NOISE_STD,
        "rereg_note": REREG_NOTE,
        "models": {},
    }
    for label in MODELS:
        r = records[label]
        out["models"][label] = {
            "n_params": r["n_params"],
            "n_dx": r["n_dx"],
            "train_z_mm": r["train_z_mm"],
            "train_roll_deg": r["train_roll_deg"],
            "train_pitch_deg": r["train_pitch_deg"],
            "sets": {
                name: {
                    "role": "validation",
                    "n": e["n"],
                    **{
                        k: e[k]
                        for k in (
                            "transfer_z_mm",
                            "transfer_roll_deg",
                            "transfer_pitch_deg",
                        )
                    },
                    "rereg_optimistic": {
                        k: e[k]
                        for k in ("rereg_z_mm", "rereg_roll_deg", "rereg_pitch_deg")
                    },
                    "strata_transfer_z_mm": {
                        k: {"n": v["n"], "rmse": _nan_none(v["transfer_z_mm"])}
                        for k, v in e["strata"].items()
                    },
                }
                for name, e in r["sets"].items()
            },
        }
    return out


def corrections_report(calib) -> dict:
    """Gauge note and the fitted parameters with their standard errors."""
    cfg = calib.calib_config
    names = list(cfg["param_name"])
    x = np.asarray(calib.LM_result.x, dtype=float)
    std = calib.std_dev if getattr(calib, "std_dev", None) is not None else None
    std = np.zeros(len(names)) if std is None else np.asarray(std, dtype=float)
    return {
        "calibration_level": cfg["calib_model"],
        "estimation": cfg.get("estimation"),
        "gauge": {
            "collinear": ["plane_z_s0", "contact_z"],
            "contact_z_free_in_reference_fit": "contact_z" in names,
            "contact_z_fixed_in": ["table+contact only", "map"],
            "unobservable": ["contact_x", "contact_y", "contact_yaw"],
            "note": GAUGE_NOTE,
        },
        "n_deltaX": int(calib.n_deltaX),
        "parameters": {
            n: {"value": float(v), "std": float(s)} for n, v, s in zip(names, x, std)
        },
        "solver": {
            "success": bool(calib.LM_result.success),
            "status": int(calib.LM_result.status),
            "message": str(calib.LM_result.message),
            "nfev": int(calib.LM_result.nfev),
        },
    }


def _print_heldout(side, heldout) -> None:
    print(f"\n{side}: held-out validation (gap z, mm)")
    print(f"  {'model':20s} {'par':>3s} {'train':>7s} {'transfer':>9s} {'rereg*':>7s}")
    for label, m in heldout["models"].items():
        e = next(iter(m["sets"].values()))
        print(
            f"  {label:20s} {m['n_params']:3d} {m['train_z_mm']:7.2f} "
            f"{e['transfer_z_mm']:9.2f} {e['rereg_optimistic']['rereg_z_mm']:7.2f}"
        )
    print("  * re-registered is optimistic (3 plane parameters refitted)")


def run_chain(side, robot, args) -> int:
    config, plan_path = hp.CHAINS[side]
    plan = harness.load_plan(plan_path)  # verifies every sha256
    train_csv = plan.folds[0].training[0].path

    # the archived fit: the reference configuration on the training file
    fitted = hp.fit_model(robot, side, REFERENCE, train_csv)
    calib = fitted.calib
    check_protocol(calib, plan)
    corrections = corrections_report(calib)
    # The fit read a staged copy of the raw session that is deleted after
    # initialisation; provenance (fixed at solve()) must name the frozen raw
    # file, whose sha256 the plan has verified.
    calib.calib_config["data_file"] = str(train_csv)
    calib._run_provenance["data"] = {"data_file": describe_file(train_csv)}
    run_dir = compute_run_dir(calib, root=args.root)

    calib.export_html_report(output_path=str(run_dir / "report.html"))
    print("\n" + "=" * 60 + f"\n{side}: VERIFICATION\n" + "=" * 60)
    verdict = run_verification(
        calib, run_dir, args.verification_scope, args.acceptance_profile
    )
    archive_run(calib, run_dir)

    records = hp.run_chain(side, robot, MODELS)
    heldout = heldout_report(records)
    for name, data in (("heldout.json", heldout), ("corrections.json", corrections)):
        (run_dir / name).write_text(json.dumps(data, indent=2))
    _print_heldout(side, heldout)
    print(f"\nGauge: {GAUGE_NOTE}")

    plan_file = Path(plan_path)
    write_reproduction_record(
        run_dir,
        processing={
            "command": "reference_run.py",
            "chain": side,
            "models": list(MODELS),
            "reference_model": REFERENCE,
            "noise_std": hp.NOISE_STD,
            "verification_scope": args.verification_scope,
            "data_roles": {
                "data_file": {
                    "path": str(train_csv),
                    "session": train_csv.name,
                    "role": "training",
                }
            },
            "heldout": [
                {"path": str(h.path), "role": "validation"}
                for h in plan.folds[0].evaluated
            ],
            "protocol": str(plan_file),
        },
        inputs={
            "protocol": describe_file(plan_file),
            "master_protocol": describe_file(MASTER_PROTOCOL),
        },
        artifacts={
            name: describe_file(run_dir / name)
            for name in ("heldout.json", "corrections.json")
        },
    )
    checks = audit(run_dir)
    print(f"\nRun archived to {run_dir}")

    failures = []
    if not verdict.passed:
        failures.append(f"{side}: verdict {verdict.status}")
    if missing(checks):
        failures.append(f"{side}: archive incomplete: {missing(checks)}")
    if failures:
        print("\nREFERENCE RUN FAILED: " + "; ".join(failures))
        return 1
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--chains", nargs="+", choices=list(hp.CHAINS), default=list(hp.CHAINS)
    )
    parser.add_argument(
        "--root", default="results/runs", help="archive root (default: %(default)s)"
    )
    parser.add_argument(
        "--no-plot", action="store_true", help="accepted for symmetry; never plots"
    )
    add_verification_args(parser)
    args = parser.parse_args(argv)

    robot = _load_robot()
    status = 0
    for side in args.chains:
        status |= run_chain(side, robot, args)
    if not status:
        print("\nReference run complete.")
    return status


if __name__ == "__main__":
    sys.exit(main())
