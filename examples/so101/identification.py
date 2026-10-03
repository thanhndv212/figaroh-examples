# Copyright [2021-2026] Thanh Nguyen

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

# http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
SO-101 gravity + friction identification from a soarm_sdk excitation log.

    python identification.py                                   # simulated data
    python identification.py --data-dir /path/to/run --validation-dir /path/to/run2

Record a real run with soarm_sdk's ``soarm-identify-record``; deploy the
result with ``update_model.py``. When the data directory holds a
``ground_truth.yaml`` (as ``generate_simulated_data.py`` writes), the
identified gravity torque is also checked against it.
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

import numpy as np
import yaml

# Add project root to path for imports (prefer `pip install -e .` instead)
_project_root = Path(__file__).parents[2]
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

from figaroh.tools.run_archive import archive_run, compute_run_dir  # noqa: E402
from examples.verification import add_verification_args, run_verification  # noqa: E402

from examples.so101.utils.so101_tools import (  # noqa: E402
    SO101Identification,
    load_so101_robot,
    read_custom_config,
    read_log_meta,
    resolve_locked_joints,
)


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="SO-101 dynamic parameter identification")
    p.add_argument("--config", default="config/so101_unified_config.yaml")
    p.add_argument("--urdf", default="urdf/so101_new_calib.urdf")
    p.add_argument(
        "--data-dir",
        default="data/simulated",
        help="a soarm_sdk.dynamics.log/v1 run directory",
    )
    p.add_argument(
        "--validation-dir",
        default=None,
        help="an independently recorded run, same format (default: the "
        "config's validation_data_file; data/simulated_validation "
        "when identifying the simulated data)",
    )
    p.add_argument(
        "--signal",
        choices=["current_mA", "load_percent"],
        default=None,
        help="torque signal (default: custom.torque_sensing.signal)",
    )
    p.add_argument(
        "--wls",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="override identification.problem.wls",
    )
    p.add_argument("--plot", action=argparse.BooleanOptionalAction, default=False)
    add_verification_args(p)
    p.add_argument(
        "--verify",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="check the selected acceptance scope; exit(1) on failed/incomplete evidence",
    )
    p.add_argument("--html-report", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--archive", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--asset-id", default=None, help="physical arm id for provenance")
    p.add_argument("--operator", default=None)
    p.add_argument("--verbose", "-v", action="store_true")
    return p.parse_args(argv)


def run_identification(args: argparse.Namespace) -> SO101Identification:
    """Load, fit and return the solved identification object."""
    for path, what in (
        (args.urdf, "URDF"),
        (args.config, "config"),
        (args.data_dir, "data directory"),
    ):
        if not Path(path).exists():
            raise FileNotFoundError(f"{what} not found: {path}")

    custom = read_custom_config(args.config)
    locked = resolve_locked_joints(custom, args.data_dir)
    robot = load_so101_robot(args.urdf, locked)
    iden = SO101Identification(
        robot, args.config, data_dir=args.data_dir, signal=args.signal
    )

    val = args.validation_dir
    if (
        val is None
        and Path(args.data_dir).resolve() == Path("data/simulated").resolve()
    ):
        val = "data/simulated_validation"
    if val:
        iden.identif_config["validation_data_file"] = val

    instance = dict(iden.identif_config.get("instance") or {})
    instance.setdefault("asset_id", read_log_meta(args.data_dir).get("arm_id"))
    if args.asset_id:
        instance["asset_id"] = args.asset_id
    if args.operator:
        instance["operator"] = args.operator
    iden.identif_config["instance"] = {k: v for k, v in instance.items() if v}

    wls = args.wls if args.wls is not None else iden.identif_config.get("wls", False)
    iden.initialize()
    iden.solve(
        decimate=True,
        decimation_factor=5,
        plotting=args.plot,
        save_results=False,
        wls=wls,
        html_report=False,
    )
    return iden


def compare_with_ground_truth(iden: SO101Identification, truth_file: Path) -> float:
    """Worst per-joint RMS gravity-torque error vs the simulator's truth, N·m."""
    import pinocchio as pin

    truth = yaml.safe_load(truth_file.read_text())
    recon = iden.result.get("reconstruction") or {}
    std = dict(iden.standard_parameter)
    std.update(recon.get("theta_r_dict") or {})

    def model_with(bodies):
        m = iden.model.copy()
        for j, b in bodies.items():
            jid = m.getJointId(j)
            body = m.inertias[jid]
            mass = b["m"]
            lever = np.array([b["mx"], b["my"], b["mz"]]) / mass
            m.inertias[jid] = pin.Inertia(mass, lever, body.inertia)
        return m

    ident = model_with(
        {
            j: {k: std[f"{k}_{j}"] for k in ("m", "mx", "my", "mz")}
            for j in iden.active_joints
        }
    )
    true = model_with(truth["bodies"])
    cad = iden.model
    q = iden.processed_data["positions"]

    def gravity(model):
        d = model.createData()
        return np.array([pin.computeGeneralizedGravity(model, d, qi) for qi in q])

    g_true = gravity(true)
    err = np.sqrt(np.mean((gravity(ident) - g_true) ** 2, axis=0))
    cad_err = np.sqrt(np.mean((gravity(cad) - g_true) ** 2, axis=0))
    print("\nGravity torque vs ground truth (RMS over the trajectory, N·m):")
    print(f"  {'joint':14s} {'identified':>10s} {'CAD':>10s}")
    for j, e, c in zip(iden.active_joints, err, cad_err):
        print(f"  {j:14s} {e:10.5f} {c:10.5f}")
    return float(np.max(err))


def main(args: argparse.Namespace) -> None:
    try:
        iden = run_identification(args)
    except (FileNotFoundError, ValueError) as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

    print("\n" + "=" * 60)
    print("SO-101 IDENTIFICATION RESULTS")
    print("=" * 60)
    print(
        f"data            : {args.data_dir}  (signal: {iden.signal}, "
        f"{iden.nm_per_unit:g} N·m/unit)"
    )
    print(f"locked joints   : {iden.robot.locked_joints}")
    print(f"base parameters : {len(iden.params_base)}")
    print(f"correlation     : {iden.correlation:.4f}")
    print(f"condition number: {iden.result['condition number']:.1f}")
    for name, value in zip(iden.params_base, iden.phi_base):
        print(f"  {name:40s} {value:+.6f}")

    truth = Path(args.data_dir) / "ground_truth.yaml"
    if truth.exists():
        compare_with_ground_truth(iden, truth)

    run_dir = None
    if args.html_report or args.verify or args.archive:
        run_dir = compute_run_dir(iden)
    if args.html_report:
        iden.export_html_report(output_path=str(run_dir / "report.html"))

    failed = False
    if args.verify:
        print("\n" + "=" * 60 + "\nVERIFICATION\n" + "=" * 60)
        verdict = run_verification(
            iden, run_dir, args.verification_scope, args.acceptance_profile
        )
        failed = not verdict.passed
    if args.archive:
        archive_run(iden, run_dir)
    if run_dir:
        print(f"\nResults written to: {run_dir}")
    if failed:
        print(f"\nVerification {verdict.status.upper()} ({verdict.scope}).")
        sys.exit(1)
    print("\nIdentification completed successfully!")
    print(
        "Deploy with: python update_model.py "
        + " ".join(
            f"--{k.replace('_', '-')} {v}" for k, v in (("data_dir", args.data_dir),)
        )
    )


if __name__ == "__main__":
    ns = parse_args()
    logging.basicConfig(
        level=logging.INFO if ns.verbose else logging.WARNING,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    main(ns)
