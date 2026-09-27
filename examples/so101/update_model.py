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
Identify, then write the result where soarm_sdk can use it.

    python update_model.py                                  # simulated data
    python update_model.py --data-dir /path/to/run --output ~/.soarm_sdk/dynamics.yaml

Writes a ``soarm_sdk.dynamics.identified/v1`` YAML: per-body mass and first
moment (reconstructed standard parameters), friction and torque offset per
joint, the torque signal and its scale, and provenance. Load it on the host
with::

    from soarm_sdk.dynamics import IdentifiedDynamics
    dyn = IdentifiedDynamics.load("so101_dynamics.yaml", urdf="so101_new_calib.urdf")
    tau_g = dyn.gravity_torque(q_arm)          # N·m, URDF frame
    i_ff = dyn.to_signal(dyn.torque(q_arm))    # expected servo current

Why a YAML and not a URDF: gravity is fully described by each body's mass
and first moment, which is all this data identifies. figaroh's
``urdf_exporter`` writes masses but not first moments or inertias yet, and a
URDF would carry CAD inertia tensors next to identified masses as if both
were measured.

If ``soarm_sdk`` is importable here, the written file is loaded back through
it and its gravity torque checked against pinocchio's on the identified
model, so a format or frame mismatch fails now rather than on the arm.
"""

from __future__ import annotations

import argparse
import logging
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import yaml

# Add project root to path for imports (prefer `pip install -e .` instead)
_project_root = Path(__file__).parents[2]
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

from examples.so101 import identification as ident  # noqa: E402
from examples.so101.utils.so101_tools import identified_dynamics_dict  # noqa: E402


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--output", default="results/so101_dynamics.yaml")
    p.add_argument(
        "--allow-unverified",
        action="store_true",
        help="write the file even if the identification fails verification",
    )
    args, rest = p.parse_known_args(argv)
    id_args = ident.parse_args(rest + ["--no-html-report", "--no-archive", "--no-plot"])
    id_args.output = args.output
    id_args.allow_unverified = args.allow_unverified
    return id_args


def check_with_soarm_sdk(doc: dict, urdf: str, iden) -> None:
    """Round-trip through soarm_sdk and compare with pinocchio. Optional."""
    try:
        from soarm_sdk.dynamics import IdentifiedDynamics
    except ImportError:
        print("soarm_sdk not importable here; skipped the round-trip check")
        return
    import pinocchio as pin

    dyn = IdentifiedDynamics.from_dict(doc, urdf)
    model = iden.model.copy()
    for j, b in doc["bodies"].items():
        jid = model.getJointId(j)
        model.inertias[jid] = pin.Inertia(
            b["m"],
            np.array([b["mx"], b["my"], b["mz"]]) / b["m"],
            model.inertias[jid].inertia,
        )
    data = model.createData()
    rng = np.random.default_rng(0)
    lo, hi = model.lowerPositionLimit, model.upperPositionLimit
    worst = 0.0
    for _ in range(20):
        q = rng.uniform(lo, hi)
        worst = max(
            worst,
            float(
                np.max(
                    np.abs(
                        dyn.gravity_torque(q)
                        - pin.computeGeneralizedGravity(model, data, q)
                    )
                )
            ),
        )
    if worst > 1e-9:
        raise RuntimeError(
            f"soarm_sdk and pinocchio disagree on gravity by {worst:.3g} N·m"
        )
    print(
        f"soarm_sdk round-trip check: gravity matches pinocchio (max diff {worst:.1e} N·m)"
    )


def main(args: argparse.Namespace) -> None:
    iden = ident.run_identification(args)
    verdict = iden.verify()
    for check in verdict.checks:
        status = "PASS" if check.passed else "FAIL"
        print(
            f"  [{status}] {check.name}: {check.value:.4g} "
            f"({check.comparison} {check.threshold:.4g})"
        )
    if not verdict.passed and not args.allow_unverified:
        print(
            "\nVerification FAILED — not writing a model from this fit. "
            "Pass --allow-unverified to write it anyway.",
            file=sys.stderr,
        )
        sys.exit(1)

    meta = iden.log_meta
    provenance = {
        "identified_at": datetime.now(timezone.utc).isoformat(),
        "data_dir": str(Path(args.data_dir).resolve()),
        "arm_id": meta.get("arm_id"),
        "recorded_at": meta.get("recorded_at"),
        "simulated": bool(meta.get("simulated")),
        "calibration": meta.get("calibration"),
        "calibration_validated": meta.get("calibration_validated"),
        "config": str(Path(args.config).resolve()),
        "verification_passed": bool(verdict.passed),
    }
    doc = identified_dynamics_dict(iden, urdf=args.urdf, provenance=provenance)
    check_with_soarm_sdk(doc, args.urdf, iden)

    out = Path(args.output).expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(
        "# SO-101 identified dynamics — written by figaroh-examples/examples/so101/"
        "update_model.py\n# Load with soarm_sdk.dynamics.IdentifiedDynamics.load(path, urdf)\n"
        + yaml.safe_dump(doc, sort_keys=False)
    )
    print(f"\nwritten to {out}")
    for j, b in doc["bodies"].items():
        print(
            f"  {j:14s} m={b['m']:.4f} kg  h=({b['mx']:+.5f}, {b['my']:+.5f}, "
            f"{b['mz']:+.5f}) kg·m   fv={doc['friction'][j]['fv']:.4f}  "
            f"fs={doc['friction'][j]['fs']:.4f}  off={doc['offset'][j]:+.4f}"
        )


if __name__ == "__main__":
    ns = parse_args()
    logging.basicConfig(
        level=logging.INFO if ns.verbose else logging.WARNING,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    main(ns)
