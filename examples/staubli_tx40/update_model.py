# Copyright [2021-2026] Thanh Nguyen
# Copyright [2022-2024] [CNRS, Toward SAS]

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
Fit physically consistent link inertials to the TX40 recording and write them
into a URDF.

    python update_model.py                         # writes results/tx40_identified.urdf
    python update_model.py --prior-weight 1        # weaker pull towards CAD

**Not independently validated.** The TX40 has a single recording
(``data/curr_data.csv``, ``data/pos_read_data.csv``); the fit is scored on the
data it was fitted to. See docs/development/tx40-update-model.md.

The base-parameter fit of ``identification.py`` has no per-link inertials.
This runs core's direct physical fit instead (``select_stage: physical_fit``):
the standard parameters minimising ``||R theta - Q^T tau||^2 +
lambda^2 ||z||^2`` subject to every link's pseudo-inertia being positive
definite, where ``z`` is the scaled deviation from the URDF (CAD) inertials.
The torque term is a sum over all samples, so ``lambda^2`` (``--prior-weight``)
must be large to matter. Without it the fit is feasible but implausible
(link 2 at 385 kg against 3.6 kg CAD): the extra mass moves along directions
the excitation does not see. At the default of 100 the training RMSE rises
by 0.8 % and every mass stays within 15 % of CAD.

The fit is a semidefinite program. With this prior cvxopt (core's default,
in ``figaroh-dev``) solves it in about a second. With weak priors (``--prior-weight``
1 or less) the problem is nearly singular and cvxopt reports no optimal
solution; QICS (``pip install qics``, optional) still solves it and is tried
second. The two agree to 1e-5 relative where both succeed.
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

import numpy as np

# Add project root to path for imports (prefer `pip install -e .` instead)
project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from examples.staubli_tx40.utils.staubli_tx40_tools import (  # noqa: E402
    TX40Identification,
)
from figaroh.tools.robot import load_robot  # noqa: E402

P10_KEYS = ("m", "mx", "my", "mz", "Ixx", "Ixy", "Iyy", "Ixz", "Iyz", "Izz")
DEFAULT_PRIOR_WEIGHT = 100.0


def parse_args(argv=None) -> argparse.Namespace:
    """Parse command-line arguments."""
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--config", default="config/staubli_tx40_unified_config.yaml")
    p.add_argument("--urdf", default="urdf/tx40_mdh_modified.urdf")
    p.add_argument("--output", default="results/tx40_identified.urdf")
    p.add_argument(
        "--prior-weight",
        type=float,
        default=DEFAULT_PRIOR_WEIGHT,
        help="Tikhonov weight lambda^2 towards the URDF inertials (default 100)",
    )
    p.add_argument("--solver", default="cvxopt", help="first SDP solver (picos name)")
    p.add_argument(
        "--second-solver",
        default="qics",
        help="solver tried when the first fails (if installed); 'none' to disable",
    )
    p.add_argument("--verbose", "-v", action="store_true")
    return p.parse_args(argv)


def physical_fit(robot, args) -> TX40Identification:
    """Identify with the physical fit selected; return the solved object."""
    iden = TX40Identification(robot, args.config)
    second = None if args.second_solver.lower() == "none" else args.second_solver
    iden.identif_config["select_stage"] = "physical_fit"
    iden.identif_config["physical_fit"] = {
        "enabled": True,
        "solver": args.solver,
        "second_solver": second,
        "mass_min": 1e-6,
        "prior_weight": args.prior_weight,
        "max_seconds": None,
    }
    iden.initialize()
    iden.solve(
        decimate=True, plotting=False, save_results=False, wls=True, html_report=False
    )
    return iden


def base_fit_rmse(iden) -> list:
    """Per-joint RMSE of the unconstrained base-parameter (WLS) fit."""
    res = np.asarray(iden.tau_noised) - np.asarray(iden.tau_identif)
    n = len(iden.fit_joints())
    return [float(np.sqrt(np.mean(s**2))) for s in np.split(res, n)]


def degenerate_links(iden, threshold: float = 1e-6) -> list:
    """Links whose COM rotational inertia has an eigenvalue below threshold."""
    import pinocchio as pin

    values = dict(zip(iden.selected.names, iden.selected.values))
    out = []
    for j in range(1, iden.model.njoints):
        name = iden.model.names[j]
        p10 = np.array([values[f"{k}_{name}"] for k in P10_KEYS])
        e = np.linalg.eigvalsh(pin.Inertia.FromDynamicParameters(p10).inertia).min()
        if e < threshold:
            out.append((name, float(e)))
    return out


def export_parity(iden, path: str) -> float:
    """Max |p10 difference| between the written URDF and the estimate."""
    import pinocchio as pin

    model = pin.buildModelFromUrdf(path)
    values = dict(zip(iden.selected.names, iden.selected.values))
    worst = 0.0
    for j in range(1, model.njoints):
        name = model.names[j]
        ref = np.array([values[f"{k}_{name}"] for k in P10_KEYS])
        dp = model.inertias[j].toDynamicParameters()
        worst = max(worst, float(np.abs(dp - ref).max()))
    return worst


def main(args: argparse.Namespace) -> int:
    for path in (args.urdf, args.config):
        if not Path(path).exists():
            print(f"Error: file not found: {path}", file=sys.stderr)
            return 1

    robot = load_robot(args.urdf, package_dirs="../../models")
    iden = physical_fit(robot, args)

    pf = iden.result.get("physical_fit", {})
    sel = iden.result.get("selected", {})
    if sel.get("status") != "accepted":
        print(
            "Error: physical fit not accepted "
            f"(solvers {pf.get('solvers')}, status {pf.get('solver_status')}): "
            f"{sel.get('reason') or pf.get('reason')}.\n"
            "With a weak prior, install QICS (pip install qics) for the fallback.",
            file=sys.stderr,
        )
        return 1

    values = dict(zip(sel["names"], sel["values"]))
    joints = iden.fit_joints()
    rmse_pf = [sel["effort_rmse_fit_per_joint"][j] for j in joints]
    rmse_base = base_fit_rmse(iden)

    print("=" * 64)
    print("TX40 PHYSICALLY CONSISTENT FIT (not independently validated)")
    print("=" * 64)
    print(
        f"solver {pf['solvers'][-1]} ({pf['solver_status']}), "
        f"{pf['runtime_s']:.2f} s, prior weight {args.prior_weight:g}"
    )
    print(f"{'joint':10s} {'RMSE base fit':>14s} {'RMSE physical':>14s}  (N·m)")
    for j, a, b in zip(joints, rmse_base, rmse_pf):
        print(f"{j:10s} {a:14.3f} {b:14.3f}")
    print(f"\n{'link':10s} {'mass CAD':>9s} {'mass fit':>9s}  (kg)")
    for j in range(1, robot.model.njoints):
        name = robot.model.names[j]
        print(
            f"{name:10s} {robot.model.inertias[j].mass:9.3f} "
            f"{values[f'm_{name}']:9.3f}"
        )

    degenerate = degenerate_links(iden)
    if degenerate:
        print(
            "\nNear-degenerate COM inertia (min eigenvalue < 1e-6 kg·m²): "
            + ", ".join(f"{n} {e:.1e}" for n, e in degenerate)
            + ".\nLink 6 is a point mass in the CAD URDF. Simulators that need "
            "strictly positive inertia need a floor on these."
        )

    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    out = iden.export_urdf(args.urdf, output_path=args.output)
    parity = export_parity(iden, out)
    print(f"\nWrote {out}\nexport parity: max |p10 diff| {parity:.1e}")
    if parity > 1e-6:
        print("Error: exported inertials differ from the estimate", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    args_ns = parse_args()
    logging.basicConfig(
        level=logging.INFO if args_ns.verbose else logging.WARNING,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    sys.exit(main(args_ns))
