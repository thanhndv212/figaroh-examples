"""TIAGo Pro mocap calibration reference workflow (#97).

One headless command: fit on the training session with the configured
reference model, report every frozen held-out session, verify the fit and
archive the run with its reproduction record.

    python reference_run.py --no-plot               # from examples/tiago_pro
    python reference_run.py --asset-id TIAGO-PRO --root /tmp/runs

Steps and what each writes into the run directory
(``<root>/<asset>/calibration/<timestamp>/``):

0. **Inputs**: the training file of the fit must be the frozen protocol's
   (``data/protocol.yaml``; sha256 of every file is checked when the
   protocol is loaded), else the run stops before writing anything.
1. **Fit** (``tiago_pro_calibration_config.yaml``: ``full_params``, the
   estimation method selected on the validation role): ``report.html``.
2. **Held-out report**: position error (mm) of the tool on every session of
   the protocol, re-registered (primary) and transfer, with posture strata:
   ``heldout.json``.
3. **Gauge and corrections**: estimated frames and fitted parameters with
   standard errors: ``corrections.json``.
4. **Verdict and archive**: ``verdict.json`` (solver status, scoped checks),
   ``provenance.json``, ``reproduction.json``; the archive audit must find
   nothing missing.

Exits nonzero when the verdict fails or the archive is incomplete.
Limitations: no URDF or PAL export (the shipped ``calibration_results*.yaml``
and ``master_calibration*.yaml`` come from ``run_calibration.py``); the base
and tool frames are the mocap setup, not robot geometry. See
docs/development/tiago-pro-mocap.md.
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
from examples.tiago_pro import heldout_protocol as hp  # noqa: E402
from examples.verification import add_verification_args, run_verification  # noqa: E402

PROTOCOL = hp.DATA / "protocol.yaml"


def check_protocol(calib) -> None:
    """The fit's training file is the frozen protocol's training session.

    Raises ValueError otherwise. The sha256 of every session file was checked
    when ``heldout_protocol`` loaded the protocol.
    """
    name = Path(calib.calib_config.get("data_file") or "").name
    if name != hp.TRAINING:
        raise ValueError(
            f"protocol mismatch: data_file {name!r} is not the protocol's "
            f"training session {hp.TRAINING!r}"
        )
    if calib.calib_config.get("validation_data_file"):
        raise ValueError("protocol mismatch: a validation file was fitted on")


def heldout_report(calib) -> dict:
    """Position error per component and posture stratum on every session."""
    out = {}
    validation = [hp.DATA / n for role, n in hp.SETS if role == "validation"]
    for role, name in hp.SETS[1:]:
        e = hp.component_errors(
            calib, hp.DATA / name, validation if role == "confirmation" else None
        )
        out[name] = {
            "role": role,
            "n": e["n"],
            hp.PRIMARY_METRIC: e[hp.PRIMARY_METRIC],
            hp.SECONDARY_METRIC: e[hp.SECONDARY_METRIC],
            "rmse_xyz_rereg_mm": [float(v) for v in e["rereg"]["rmse_xyz"]],
            "max_rereg_mm": e["rereg"]["max"],
            "frame_shift": e["frame_shift"],
            "strata_rmse_mm": _json_pairs(e["strata"]),
            **(
                {"overlap_rmse_mm": _json_pairs(e["overlap"])} if "overlap" in e else {}
            ),
        }
    return out


def _json_pairs(d: dict) -> dict:
    return {k: {"n": n, "rmse": None if np.isnan(v) else v} for k, (n, v) in d.items()}


def corrections_report(calib) -> dict:
    """Gauge and fitted parameters with their standard errors."""
    cfg = calib.calib_config
    names = list(cfg["param_name"])
    frames = [n for n in names if hp.is_frame(n)]
    std = calib.std_dev if calib.std_dev is not None else [0.0] * len(names)
    return {
        "calibration_level": cfg["calib_model"],
        "estimation": cfg.get("estimation"),
        "gauge": {
            "estimated_frames": frames,
            "absorbed_by_frames": list(cfg.get("absorbed_param_name", [])),
            "note": (
                "Base frame (mocap to robot base, 6D) and tool frame (6D) are "
                "estimated; they are mocap setup, not robot geometry."
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


def _print_heldout(heldout: dict) -> None:
    print("\nHeld-out report (tool position, mm)")
    print(f"  {'role':12s} {'session':14s} {'n':>3s}  re-registered  transfer")
    for name, e in heldout.items():
        session = name.removeprefix("calibration_samples_").removesuffix(".csv")
        print(
            f"  {e['role']:12s} {session:14s} {e['n']:3d}  "
            f"{e[hp.PRIMARY_METRIC]:13.2f}  {e[hp.SECONDARY_METRIC]:8.2f}"
        )


def run(args) -> int:
    calib = hp.fit(
        "full_params",
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
    train = hp.DATA / hp.TRAINING
    write_reproduction_record(
        run_dir,
        processing={
            "command": "reference_run.py",
            "known_baseframe": False,
            "known_tipframe": False,
            "del_list": [],
            "verification_scope": args.verification_scope,
            "data_roles": {
                "data_file": {
                    "path": str(train),
                    "session": hp.SETS[0][0],
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
    parser.add_argument("--asset-id", default="TIAGO-PRO", help="unit identifier")
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
