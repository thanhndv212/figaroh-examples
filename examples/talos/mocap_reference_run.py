"""TALOS torso-arm mocap calibration reference workflow (#172).

One headless command: fit the training run with the configured reference
model, report both right-arm runs of the frozen protocol, verify the fit
and archive the run with its reproduction record.

    python mocap_reference_run.py --no-plot     # from examples/talos
    python mocap_reference_run.py --asset-id TALOS --root /tmp/runs

Steps and what each writes into the run directory
(``<root>/<asset>/calibration/<timestamp>/``):

0. **Inputs:** the fit's training file must be the frozen protocol's
   (``data/calibration/mocap/protocol.yaml``, every sha256 checked when the
   protocol is loaded), else the run stops before writing anything.
1. **Fit** (``config/talos_mocap_config.yaml``: the level and estimation
   method the k-fold plan selected): ``report.html``.
2. **Held-out report:** position and orientation error of the end-effector
   body on both right-arm runs, the torso carried over and the frames and
   flips re-registered (primary), the waist kept (secondary), the
   gauge-free torso diagnostic, the flip repairs and the posture strata:
   ``heldout.json``.
3. **Gauge and corrections:** estimated frames and fitted parameters with
   standard errors: ``corrections.json``.
4. **Verdict and archive:** ``verdict.json``, ``provenance.json``,
   ``reproduction.json``; the archive audit must find nothing missing.

Exits nonzero when the verdict fails or the archive is incomplete.
Limitations: no URDF export; the base and tool frames are the mocap bodies'
mounts, not robot geometry; the right arm is nominal (no right-arm data
trains anything). See docs/development/talos-mocap.md.
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
from examples.talos import mocap_heldout as mh  # noqa: E402
from examples.verification import add_verification_args, run_verification  # noqa: E402


def check_protocol(calib) -> None:
    """The fit's training file is the frozen protocol's training run.

    Raises ValueError otherwise (sha256 was checked when ``mocap_heldout``
    loaded the protocol).
    """
    name = Path(calib.calib_config.get("data_file") or "").name
    if name != mh.TRAINING:
        raise ValueError(
            f"protocol mismatch: data_file {name!r} is not the protocol's "
            f"training run {mh.TRAINING!r}"
        )
    if calib.calib_config.get("validation_data_file"):
        raise ValueError("protocol mismatch: a validation file was fitted on")


def _jsonable(v):
    if isinstance(v, dict):
        return {k: _jsonable(x) for k, x in v.items() if not k.startswith("_")}
    if isinstance(v, (list, tuple)):
        return [_jsonable(x) for x in v]
    if isinstance(v, (np.floating, float)):
        return None if np.isnan(v) else float(v)
    if isinstance(v, np.integer):
        return int(v)
    return v


def heldout_report(calib) -> dict:
    """Errors of every right-arm run of the protocol."""
    out = {}
    for role, name in mh.SETS[1:]:
        e = mh.heldout_errors(calib, mh.MOCAP / name)
        g = mh.heldout_errors(calib, mh.MOCAP / name, mh.GAUGE_FREE_TORSO)
        e["strata"] = {k: {"n": n, "rmse": r} for k, (n, r) in e["strata"].items()}
        out[name] = {
            "role": role,
            **_jsonable(e),
            "gauge_free_torso_" + mh.PRIMARY_METRIC: g[mh.PRIMARY_METRIC],
        }
    return out


def corrections_report(calib) -> dict:
    """Gauge and fitted parameters with their standard errors."""
    cfg = calib.calib_config
    names = list(cfg["param_name"])
    std = calib.std_dev if calib.std_dev is not None else [0.0] * len(names)
    return {
        "calibration_level": cfg["calib_model"],
        "estimation": _jsonable(cfg.get("estimation")),
        "gauge": {
            "estimated_frames": [n for n in names if mh.is_frame(n)],
            "absorbed_by_frames": list(cfg.get("absorbed_param_name", [])),
            "note": (
                "Base frame (base_link in the waist body frame, 6D) and tool "
                "frame (end-effector body on arm_left_7_link, 6D) are "
                "estimated; they are the mocap bodies' mounts, not robot "
                "geometry."
            ),
        },
        "parameters": {
            n: {"value": float(v), "std": float(s)}
            for n, v, s in zip(names, calib.LM_result.x, std)
        },
        "train": mh.training_errors(calib),
        "solver": {
            "success": bool(calib.LM_result.success),
            "status": int(calib.LM_result.status),
            "message": str(calib.LM_result.message),
            "nfev": int(calib.LM_result.nfev),
            "residual_dof": getattr(calib, "residual_dof", None),
        },
    }


def _print_heldout(heldout: dict) -> None:
    print("\nHeld-out report (end-effector position, mm)")
    print(f"  {'role':12s} {'run':10s} {'n':>3s}  re-registered  waist kept  flipped")
    for name, e in heldout.items():
        print(
            f"  {e['role']:12s} {mh._short(name):10s} {e['n']:3d}  "
            f"{e[mh.PRIMARY_METRIC]:13.2f}  {e[mh.SECONDARY_METRIC]:10.2f}  "
            f"{e['n_flipped']:7d}"
        )


def run(args) -> int:
    level = mh.yaml.safe_load(mh.CONFIG.read_text())["tasks"]["calibration"][
        "parameters"]["calibration_level"]
    calib = mh.fit(
        level,
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
            "command": "mocap_reference_run.py",
            "known_baseframe": False,
            "known_tipframe": False,
            "del_list": [],
            "verification_scope": args.verification_scope,
            "data_roles": {
                "data_file": {
                    "path": str(mh.MOCAP / mh.TRAINING),
                    "session": mh.SETS[0][0],
                    "role": "training",
                }
            },
            "protocol": str(mh.PROTOCOL),
        },
        inputs={
            "protocol": describe_file(mh.PROTOCOL),
            "kfold_protocol": describe_file(mh.KFOLD_PROTOCOL),
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
    parser.add_argument("--asset-id", default="TALOS", help="unit identifier")
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
