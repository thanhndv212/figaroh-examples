"""TIAGo mocap calibration reference workflow (C4, #29).

One headless command: fit on the training session, report every frozen
held-out session, export the URDF and the PAL geometric_calibration, reload
them and check they predict what the fit predicts, then archive the run with
its reproduction record.

    python reference_run.py                        # from examples/tiago
    python reference_run.py --asset-id TIAGO-48 --root /tmp/runs

Steps and what each writes into the run directory
(``<root>/<asset>/calibration/<timestamp>/``):

1. **Fit** (``tiago_unified_config.yaml``: ``joint_offset``, training session,
   validation session as ``validation_data_file``): ``report.html``.
2. **Held-out report**: per-component error (mm) of marker 1 on every
   session of the frozen protocol (``heldout_protocol.py``), with posture
   strata: ``heldout.json``.
3. **Gauge and corrections**: estimated frames, parameters the frames absorb,
   fitted (identifiable) parameters with standard errors, and the joint
   corrections written to the URDF/PAL file (redistributed):
   ``corrections.json``.
4. **Export and reload**: ``calibrated.urdf`` (``export_urdf``), the PAL
   files ``master_calibration{,_conservative}.yaml``; both reloaded with the
   metrology frames must predict the fitted marker on every session
   (``export_check.json``, recorded as the verdict's export stage).
5. **Verdict and archive**: ``verdict.json`` (solver status, scoped
   checks), ``provenance.json``, ``reproduction.json``; the archive audit
   must find nothing missing.

Exits nonzero when the verdict fails, the export does not reload to the fit,
or the archive is incomplete. Limitations: marker 1 only (figaroh-plus#119:
several points are supported but not adopted here); the base frame and marker
point are the mocap setup, not robot geometry, and are not written to the
URDF. See docs/development/tiago-calibration-reference.md.
"""

from __future__ import annotations

import argparse
import json
import sys
import tempfile
from pathlib import Path

import numpy as np
import pinocchio as pin

project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from figaroh.calibration.calibration_tools import calc_updated_fkm  # noqa: E402
from figaroh.tools.geometric_calibration_export import (  # noqa: E402
    build_geometric_calibration,
    export_geometric_calibration_yaml,
)
from figaroh.tools.robot import load_robot  # noqa: E402
from figaroh.tools.run_archive import archive_run, compute_run_dir  # noqa: E402
from figaroh.tools.stages import record_stage  # noqa: E402
from figaroh.tools.urdf_exporter import export_urdf  # noqa: E402

from examples.run_record import (  # noqa: E402
    audit,
    describe_file,
    missing,
    write_reproduction_record,
)
from examples.tiago.calibration import PROTOCOL, _mocap_inputs  # noqa: E402
from examples.tiago.export_check import (  # noqa: E402
    apply_pal,
    changed_elements,
    session_postures,
)
from examples.tiago.heldout_protocol import (  # noqa: E402
    MOCAP,
    SETS,
    component_errors,
)
from examples.tiago.utils.tiago_tools import TiagoCalibration  # noqa: E402
from examples.verification import add_verification_args, run_verification  # noqa: E402

TIAGO = Path(__file__).resolve().parent
CONFIG = TIAGO / "config/tiago_unified_config.yaml"
URDF = TIAGO / "urdf/tiago_48_schunk.urdf"
# reloaded models must predict the fitted marker to float precision (m)
PARITY_TOL = 1e-9


def _fit(asset_id: str | None, operator: str | None) -> TiagoCalibration:
    robot = load_robot(str(URDF), load_by_urdf=True, robot_pkg="tiago_description")
    calib = TiagoCalibration(robot, str(CONFIG), del_list=[])
    calib.calib_config["known_baseframe"] = False
    calib.calib_config["known_tipframe"] = False
    if asset_id or operator:
        instance = dict(calib.calib_config.get("instance") or {})
        instance.update(
            {k: v for k, v in (("asset_id", asset_id), ("operator", operator)) if v}
        )
        calib.calib_config["instance"] = instance
    calib.initialize()
    calib.solve(plotting=False, enable_logging=False, html_report=False)
    return calib


def heldout_report(calib) -> dict:
    """Marker-1 error per component and posture stratum on every session."""
    out = {}
    for role, name in SETS:
        e = component_errors(calib, MOCAP / name)
        out[name] = {
            "role": role,
            "n": e["n"],
            "rmse_xyz_mm": [float(v) for v in e["rmse_xyz"]],
            "rmse_mm": e["rmse"],
            "max_mm": e["max"],
            "strata_rmse_mm": {
                k: {"n": n, "rmse": None if np.isnan(v) else v}
                for k, (n, v) in e["strata"].items()
            },
        }
    return out


def corrections_report(calib) -> dict:
    """Gauge, identifiable parameters and redistributed corrections."""
    cfg = calib.calib_config
    names = list(cfg["param_name"])
    frames = calib.metrology_frames()
    return {
        "calibration_level": cfg["calib_model"],
        "gauge": {
            "estimated_frames": frames,
            "absorbed_by_frames": list(cfg.get("absorbed_param_name", [])),
            "note": (
                "Base frame (mocap to robot base, 6D) and marker point (3D) "
                "are estimated; joint parameters they absorb are not fitted."
            ),
        },
        "identifiable": {
            n: {"value": float(v), "std": float(s)}
            for n, v, s in zip(names, calib.LM_result.x, calib.std_dev)
            if n not in frames
        },
        "redistributed": {k: float(v) for k, v in calib.joint_corrections().items()},
        "solver": {
            "success": bool(calib.LM_result.success),
            "status": int(calib.LM_result.status),
            "message": str(calib.LM_result.message),
            "nfev": int(calib.LM_result.nfev),
            "residual_dof": getattr(calib, "residual_dof", None),
        },
    }


def export_and_reload(calib, run_dir: Path) -> dict:
    """Write URDF and PAL files; reload both and compare with the fit."""
    urdf_out = run_dir / "calibrated.urdf"
    export_urdf(str(URDF), calib.joint_corrections(), output_path=str(urdf_out))
    for name, kw in (
        ("master_calibration.yaml", {}),
        ("master_calibration_conservative.yaml", {"min_sigma": 2.0}),
    ):
        export_geometric_calibration_yaml(
            calib, str(run_dir / name), nominal_urdf=str(URDF), **kw
        )
    gc = build_geometric_calibration(calib, nominal_urdf=str(URDF))[
        "robot_state_publisher"
    ]["geometric_calibration"]
    frames = calib.metrology_frames()
    with tempfile.TemporaryDirectory() as tmp:
        pal_urdf = apply_pal(URDF, gc, Path(tmp) / "pal.urdf")
        models = {
            "urdf": pin.buildModelFromUrdf(str(urdf_out)),
            "pal": pin.buildModelFromUrdf(str(pal_urdf)),
        }
    parity = {}
    for _, name in SETS:
        q = session_postures(calib.model, MOCAP / name)
        cfg = dict(calib.calib_config, NbSample=len(q))
        fitted = calc_updated_fkm(
            calib.model, calib.model.createData(), calib.LM_result.x, q, cfg
        )
        fcfg = dict(cfg, param_name=list(frames))
        fv = np.array(list(frames.values()))
        parity[name] = {
            k: float(
                np.abs(calc_updated_fkm(m, m.createData(), fv, q, fcfg) - fitted).max()
            )
            for k, m in models.items()
        }
    worst = max(v for d in parity.values() for v in d.values())
    changed = changed_elements(URDF, urdf_out)
    ok = worst < PARITY_TOL and not changed["other"]
    report = {
        "max_abs_difference_m": parity,
        "worst_m": worst,
        "tolerance_m": PARITY_TOL,
        "changed_joints": changed["joints"],
        "other_changes": changed["other"],
        "passed": ok,
    }
    record_stage(
        calib,
        "export",
        "ok" if ok else "failed",
        "URDF and PAL file reloaded with the metrology frames; marker "
        "predictions compared with the fit on every session",
        {"worst_reload_difference": (worst, "m")},
    )
    return report


def _print_heldout(heldout: dict) -> None:
    print("\nHeld-out report (marker 1, mm)")
    print(f"  {'role':12s} {'session':16s} {'n':>3s}   x     y     z  |  RMSE   max")
    for name, e in heldout.items():
        x, y, z = e["rmse_xyz_mm"]
        session = name.removeprefix("qualisys_").removesuffix("_static_postures.csv")
        print(
            f"  {e['role']:12s} {session:16s} {e['n']:3d} {x:5.2f} {y:5.2f} {z:5.2f}"
            f"  | {e['rmse_mm']:5.2f} {e['max_mm']:6.2f}"
        )


def run(args) -> int:
    calib = _fit(args.asset_id, args.operator)
    run_dir = compute_run_dir(calib, root=args.root)

    heldout = heldout_report(calib)
    corrections = corrections_report(calib)
    export = export_and_reload(calib, run_dir)
    for name, data in (
        ("heldout.json", heldout),
        ("corrections.json", corrections),
        ("export_check.json", export),
    ):
        (run_dir / name).write_text(json.dumps(data, indent=2))
    _print_heldout(heldout)
    print(
        f"\nGauge: frames {list(corrections['gauge']['estimated_frames'])}; "
        f"absorbed {corrections['gauge']['absorbed_by_frames']}"
    )
    print(
        f"Export: reload difference {export['worst_m']:.2e} m "
        f"(limit {PARITY_TOL:g}); joints changed {len(export['changed_joints'])}"
    )

    calib.export_html_report(output_path=str(run_dir / "report.html"))
    print("\n" + "=" * 60 + "\nVERIFICATION\n" + "=" * 60)
    verdict = run_verification(
        calib, run_dir, args.verification_scope, args.acceptance_profile
    )

    archive_run(calib, run_dir)
    inputs, data_roles = _mocap_inputs(calib.calib_config)
    artifacts = {
        name: describe_file(run_dir / name)
        for name in (
            "calibrated.urdf",
            "master_calibration.yaml",
            "master_calibration_conservative.yaml",
            "heldout.json",
            "corrections.json",
            "export_check.json",
        )
    }
    write_reproduction_record(
        run_dir,
        processing={
            "command": "reference_run.py",
            "known_baseframe": False,
            "known_tipframe": False,
            "del_list": [],
            "verification_scope": args.verification_scope,
            "data_roles": data_roles,
            "protocol": str(PROTOCOL),
        },
        inputs=inputs,
        artifacts=artifacts,
    )
    checks = audit(run_dir)  # printed by write_reproduction_record
    print(f"\nRun archived to {run_dir}")

    failures = []
    if not verdict.passed:
        failures.append(f"verdict {verdict.status}")
    if not export["passed"]:
        failures.append("export does not reload to the fit")
    if missing(checks):
        failures.append(f"archive incomplete: {missing(checks)}")
    if failures:
        print("\nREFERENCE RUN FAILED: " + "; ".join(failures))
        return 1
    print("\nReference run complete.")
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--asset-id", default=None, help="physical unit identifier")
    parser.add_argument("--operator", default=None)
    parser.add_argument(
        "--root", default="results/runs", help="archive root (default: %(default)s)"
    )
    add_verification_args(parser)
    return run(parser.parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
