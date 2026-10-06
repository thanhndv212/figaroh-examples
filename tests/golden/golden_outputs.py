"""Golden outputs of the example scripts (#76).

Runs each calibration and identification example script unchanged, records
what it produces through ``hook/sitecustomize.py``, and compares the result
with ``golden_outputs.json``. A core change that moves any example then
fails with old and new values side by side.

Pinned: calibration level, parameter count, fit RMS and the exported model's
tool deviation from nominal; identification base-parameter count and torque
RMSE. Base-parameter values are not pinned: they depend on which
representatives the QR picks, which differs by platform (figaroh-plus#116).

    python tests/golden/golden_outputs.py              # compare all cases
    python tests/golden/golden_outputs.py --update     # rewrite the reference
    python tests/golden/golden_outputs.py ur10/calibration.py

Update the reference only in the PR that changes the outputs, and say why
there (CONTRIBUTING.md, Validation).
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
HOOK = Path(__file__).resolve().parent / "hook"
REFERENCE = Path(__file__).resolve().parent / "golden_outputs.json"
THREADS = {
    "OMP_NUM_THREADS": "1",
    "OPENBLAS_NUM_THREADS": "1",
    "MKL_NUM_THREADS": "1",
    "VECLIB_MAXIMUM_THREADS": "1",
}

# case -> (command-line arguments, nominal URDF for export, relative to the
# robot folder; None for identification)
CALIBRATE = ["--calibrate-only", "--no-plot"]
CASES = {
    "ur10/calibration.py": (CALIBRATE, "urdf/ur10_robot.urdf"),
    "tiago/calibration.py": (CALIBRATE, "urdf/tiago_48_schunk.urdf"),
    "talos/calibration_upperbody.py": (CALIBRATE, "urdf/talos_full_v2.urdf"),
    # a non-default estimation method (figaroh-plus#113), checked on Linux CI
    "tiago/calibration.py[excitation]": (
        CALIBRATE + ["--config", "config/tiago_calibration_excitation.yaml"],
        "urdf/tiago_48_schunk.urdf",
    ),
    "ur10/identification.py": (["--verify"], None),
    "tiago/identification.py": (["--verify"], None),
    "staubli_tx40/identification.py": (["--verify"], None),
    "so101/identification.py": (["--verify"], None),
}

# Tolerances. Counts must match exactly. Errors and deviations agree to the
# last printed digit on macOS arm64 and Linux x86-64 (Pinocchio 3.7.0 and
# 4.1.0) for most cases; the defaults leave float-noise margin and are far
# below the changes this check exists to catch (e.g. 1.581 -> 1.591 mm).
TOL = {
    "fit_rms_mm": 1e-4,  # relative
    "export_dev_mm": 1e-3,  # mm
    "tau_rmse": 1e-4,  # relative
}
# Known platform dependence, measured when the reference was recorded:
# allowed values per case, with the issue that removes it.
PLATFORM_SPREAD = {
    # estimation.method: map since figaroh-plus#120: every joint parameter is
    # estimated (63), so the structural 38/39 macOS/Linux split (figaroh-plus
    # #113) is gone. Fit and export spreads kept from that record until
    # Linux CI measures the map case.
    "talos/calibration_upperbody.py": {
        "fit_rms_mm": 2e-3,
        "export_dev_mm": 1.0,
    },
    # torque RMSE 2.992245 (macOS) / 3.000672 (Linux) (figaroh-plus#116)
    "staubli_tx40/identification.py": {"tau_rmse": 5e-3},
}


def _robot_script(case: str) -> tuple[str, str]:
    """``"tiago/calibration.py[excitation]"`` -> ("tiago", "calibration.py")."""
    robot, rest = case.split("/", 1)
    return robot, rest.split("[")[0]


def run_case(case: str) -> list[dict]:
    """Run one example script; return the records the hook wrote."""
    args, _ = CASES[case]
    robot, script = _robot_script(case)
    with tempfile.TemporaryDirectory() as tmp:
        log = Path(tmp) / "records.jsonl"
        env = dict(os.environ, **THREADS, MPLBACKEND="Agg")
        env["FIGAROH_GOLDEN_LOG"] = str(log)
        env["PYTHONPATH"] = os.pathsep.join(
            p for p in (str(HOOK), env.get("PYTHONPATH", "")) if p
        )
        proc = subprocess.run(
            [sys.executable, script, *args],
            cwd=ROOT / "examples" / robot,
            env=env,
            capture_output=True,
            text=True,
            timeout=900,
            stdin=subprocess.DEVNULL,
        )
        if proc.returncode != 0:
            raise RuntimeError(
                f"{case} exited {proc.returncode}:\n{proc.stderr[-3000:]}"
            )
        if not log.exists():
            raise RuntimeError(f"{case} produced no record (hook not loaded?)")
        return [json.loads(line) for line in log.read_text().splitlines()]


def export_deviation_mm(robot_dir: Path, urdf: str, record: dict) -> float:
    """Max tool-position change, exported vs nominal URDF, at fixed configs."""
    import pinocchio as pin
    from figaroh.tools.urdf_exporter import export_urdf

    nominal = robot_dir / urdf
    params = dict(zip(record["param_name"], record["x"]))
    with tempfile.TemporaryDirectory() as tmp:
        exported = export_urdf(
            str(nominal), params, output_path=str(Path(tmp) / "exported.urdf")
        )
        models = [pin.buildModelFromUrdf(str(p)) for p in (nominal, exported)]
    frame = record["end_frame"]
    datas = [m.createData() for m in models]
    fids = [m.getFrameId(frame) for m in models]
    rng = np.random.default_rng(0)
    model = models[0]
    worst = 0.0
    for _ in range(20):
        q = pin.neutral(model)
        for j in range(1, model.njoints):
            joint = model.joints[j]
            if joint.nq != 1:
                continue
            lo = max(model.lowerPositionLimit[joint.idx_q], -np.pi)
            hi = min(model.upperPositionLimit[joint.idx_q], np.pi)
            q[joint.idx_q] = rng.uniform(lo, hi) if hi > lo else lo
        for m, d in zip(models, datas):
            pin.framesForwardKinematics(m, d, q)
        delta = datas[0].oMf[fids[0]].translation - datas[1].oMf[fids[1]].translation
        worst = max(worst, float(np.linalg.norm(delta)) * 1000)
    return worst


def summarize(case: str, records: list[dict]) -> list[dict]:
    """The quantities pinned per record (one per solve() call)."""
    _, urdf = CASES[case]
    robot_dir = ROOT / "examples" / case.split("/")[0]
    out = []
    for r in records:
        if r["kind"] == "calibration":
            out.append(
                {
                    "kind": "calibration",
                    "calib_model": r["calib_model"],
                    "n_params": len(r["x"]),
                    "fit_rms_mm": round(r["fit_rms"] * 1000, 6),
                    "export_dev_mm": round(export_deviation_mm(robot_dir, urdf, r), 4),
                }
            )
        else:
            out.append(
                {
                    "kind": "identification",
                    "n_base": len(r["phi_base"]),
                    "tau_rmse": round(r["tau_rmse"], 9),
                }
            )
    return out


def compare(case: str, expected: list[dict], got: list[dict]) -> list[str]:
    """Human-readable differences beyond tolerance; empty when they agree."""
    if len(expected) != len(got):
        return [f"{len(got)} solve() calls, expected {len(expected)}"]
    spread = PLATFORM_SPREAD.get(case, {})
    diffs = []
    for i, (e, g) in enumerate(zip(expected, got)):
        tag = f"[{i}] {e['kind']}"
        for key in ("kind", "calib_model", "n_params", "n_base"):
            if key not in e:
                continue
            allowed = spread.get(key, (e[key],))
            if g.get(key) not in allowed:
                diffs.append(f"{tag} {key}: {e[key]} -> {g.get(key)}")
        for key in ("fit_rms_mm", "tau_rmse"):
            if key in e:
                rtol = spread.get(key, TOL[key])
                if not np.isclose(g[key], e[key], rtol=rtol, atol=0):
                    diffs.append(f"{tag} {key}: {e[key]} -> {g[key]}")
        if "export_dev_mm" in e:
            atol = spread.get("export_dev_mm", TOL["export_dev_mm"])
            if abs(g["export_dev_mm"] - e["export_dev_mm"]) > atol:
                diffs.append(
                    f"{tag} export_dev_mm: {e['export_dev_mm']} -> "
                    f"{g['export_dev_mm']}"
                )
    return diffs


def load_reference() -> dict:
    return json.loads(REFERENCE.read_text()) if REFERENCE.exists() else {}


def main(argv: list[str]) -> int:
    update = "--update" in argv
    cases = [a for a in argv if a != "--update"] or list(CASES)
    reference = load_reference()
    failed = False
    for case in cases:
        got = summarize(case, run_case(case))
        if update:
            reference[case] = got
            print(f"{case}: recorded {len(got)} result(s)")
            continue
        diffs = compare(case, reference.get(case, []), got)
        failed |= bool(diffs)
        print(f"{case}: " + ("OK" if not diffs else "\n  " + "\n  ".join(diffs)))
    if update:
        REFERENCE.write_text(json.dumps(reference, indent=1, sort_keys=True) + "\n")
    return int(failed)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
