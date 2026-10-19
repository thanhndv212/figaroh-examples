"""UR10 hand-eye calibration reference workflow (#99).

One headless command: fit all 23 postures with the configured method, report
the k-fold held-out error of that method and of the registration baseline,
verify the fit and archive the run with its reproduction record.

    python hand_eye_reference_run.py --no-plot          # from examples/ur10
    python hand_eye_reference_run.py --asset-id UR10-1 --root /tmp/runs

Steps and what each writes into the run directory
(``<root>/<asset>/calibration/<timestamp>/``):

0. **Inputs**: the fitted file must be the frozen protocol's session
   (``data/hand_eye_protocol.yaml``; its sha256 is checked when the protocol
   is loaded), else the run stops before writing anything.
1. **Fit** (``ur10_unified_config.yaml``: ``full_params``, the configured
   estimation method, all 23 postures): ``report.html``.
2. **Held-out report**: 5-fold error of the configured model and of the
   registration baseline, position and orientation apart: ``heldout.json``.
   This is **k-fold within one session**, weaker than held-out sessions.
3. **Gauge and corrections**: estimated frames and fitted parameters with
   standard errors: ``corrections.json``.
4. **Verdict and archive**: ``verdict.json`` (solver status, scoped checks),
   ``provenance.json``, ``reproduction.json``; the archive audit must find
   nothing missing.

Exits nonzero when the verdict fails or the archive is incomplete. See
docs/development/ur10-hand-eye.md.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from figaroh.tools.run_archive import archive_run, compute_run_dir  # noqa: E402

from examples.run_record import (  # noqa: E402
    audit,
    describe_file,
    missing,
    write_reproduction_record,
)
from examples.ur10 import hand_eye_heldout as heh  # noqa: E402
from examples.verification import add_verification_args, run_verification  # noqa: E402

PROTOCOL = heh.PROTOCOL
BASELINE = "registration only"
# estimation method of the config -> model label of hand_eye_heldout.MODELS
LABELS = {
    "structural": "full structural",
    "map": "full map",
    "map_cv": "full map_cv",
    "excitation": "full excitation",
    "cv_subset": "full cv_subset",
}


def check_protocol(calib) -> None:
    """The fit's data file is the frozen protocol's session.

    Raises ValueError otherwise. The sha256 was checked when
    ``hand_eye_heldout`` loaded the protocol.
    """
    path = Path(calib.calib_config.get("data_file") or "").resolve()
    if path != Path(heh.TRAINING).resolve():
        raise ValueError(
            f"protocol mismatch: data_file {path.name!r} is not the protocol's "
            f"session {Path(heh.TRAINING).name!r}"
        )
    if calib.calib_config.get("validation_data_file"):
        raise ValueError("protocol mismatch: a validation file was fitted on")


def config_model(calib) -> str:
    """The ``hand_eye_heldout.MODELS`` label of the config's method."""
    method = (calib.calib_config.get("estimation") or {}).get("method", "structural")
    if method not in LABELS:
        raise ValueError(f"no held-out model for estimation method {method!r}")
    return LABELS[method]


def heldout_report(calib) -> dict:
    """K-fold error (pooled over the folds) of the config's model and baseline."""
    labels = [config_model(calib)]
    if BASELINE not in labels:
        labels.append(BASELINE)
    results = heh.run(labels)
    table = heh.summarize(results)
    return {
        "label": results.label,
        "protocol": heh.PLAN.protocol,
        "metrics": {
            "position": heh.PRIMARY_METRIC,
            "orientation": heh.SECONDARY_METRIC,
        },
        "models": {
            m: {
                **{k: v for k, v in r.items() if k != "strata"},
                "strata": {
                    k: {"n": n, "rmse_pos_mm": None if np.isnan(v) else v}
                    for k, (n, v) in r["strata"].items()
                },
            }
            for m, r in table.items()
        },
    }


def corrections_report(calib) -> dict:
    """Gauge and fitted parameters with their standard errors."""
    cfg = calib.calib_config
    names = list(cfg["param_name"])
    frames = [n for n in names if heh.is_frame(n)]
    std = calib.std_dev if calib.std_dev is not None else [0.0] * len(names)
    return {
        "calibration_level": cfg["calib_model"],
        "estimation": cfg.get("estimation"),
        "gauge": {
            "estimated_frames": frames,
            "absorbed_by_frames": list(cfg.get("absorbed_param_name", [])),
            "note": (
                "Base frame (fixed frame to robot base, 6D) and tool frame "
                "(flange to camera, 6D) are estimated; only their product with "
                "the joint errors is observable."
            ),
        },
        "parameters": {
            n: {"value": float(v), "std": float(s)}
            for n, v, s in zip(names, calib.LM_result.x, std)
        },
        "solver": {
            "success": bool(calib.LM_result.success),
            "status": int(calib.LM_result.status),
            "message": str(calib.LM_result.message),
            "nfev": int(calib.LM_result.nfev),
            "residual_dof": getattr(calib, "residual_dof", None),
        },
    }


def _print_heldout(report: dict) -> None:
    print(f"\nHeld-out report: {report['label']}")
    print(f"  {'model':20s} {'params':>6s} {'pos mm':>8s} {'rot deg':>8s}")
    for m, r in report["models"].items():
        print(
            f"  {m:20s} {r['n_params']:6d} {r['rmse_pos_mm']:8.2f} "
            f"{r['rmse_rot_deg']:8.3f}"
        )


def run(args) -> int:
    calib = heh.fit(
        heh.TRAINING,
        "full_params",
        max_iterations=3,
        instance={"asset_id": args.asset_id}
        | ({"operator": args.operator} if args.operator else {}),
    )
    check_protocol(calib)
    run_dir = compute_run_dir(calib, root=args.root)

    heldout = heldout_report(calib)
    corrections = corrections_report(calib)
    for name, data in (("heldout.json", heldout), ("corrections.json", corrections)):
        (run_dir / name).write_text(json.dumps(data, indent=2))
    _print_heldout(heldout)
    print(
        f"\nGauge: frames {corrections['gauge']['estimated_frames']}; "
        f"absorbed {corrections['gauge']['absorbed_by_frames']}"
    )

    calib.export_html_report(output_path=str(run_dir / "report.html"))
    print("\n" + "=" * 60 + "\nVERIFICATION\n" + "=" * 60)
    verdict = run_verification(
        calib, run_dir, args.verification_scope, args.acceptance_profile
    )

    archive_run(calib, run_dir)
    write_reproduction_record(
        run_dir,
        processing={
            "command": "hand_eye_reference_run.py",
            "known_baseframe": False,
            "known_tipframe": False,
            "del_list": [],
            "verification_scope": args.verification_scope,
            "data_roles": {
                "data_file": {
                    "path": str(heh.TRAINING),
                    "session": heh.PLAN.folds[0].training[0].session_id,
                    "role": "training",
                }
            },
            "protocol": str(PROTOCOL),
        },
        inputs={"protocol": describe_file(PROTOCOL)},
        artifacts={
            name: describe_file(run_dir / name)
            for name in ("heldout.json", "corrections.json")
        },
    )
    checks = audit(run_dir)
    print(f"\nRun archived to {run_dir}")

    failures = []
    if not verdict.passed:
        failures.append(f"verdict {verdict.status}")
    if missing(checks):
        failures.append(f"archive incomplete: {missing(checks)}")
    if failures:
        print("\nREFERENCE RUN FAILED: " + "; ".join(failures))
        return 1
    print("\nReference run complete.")
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--asset-id", default="UR10", help="unit identifier")
    parser.add_argument("--operator", default=None)
    parser.add_argument(
        "--root", default="results/runs", help="archive root (default: %(default)s)"
    )
    parser.add_argument(
        "--no-plot", action="store_true", help="accepted for symmetry; never plots"
    )
    add_verification_args(parser)
    return run(parser.parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
