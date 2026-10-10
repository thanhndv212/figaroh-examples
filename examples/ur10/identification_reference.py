"""UR10 dynamic-identification reference case with a known truth (#23).

``truth``: the physical-consistency-constrained fit (``physical_fit``) on the
low-noise training split of the truth fixture (#21), judged on its held-out
split. Passes when the fit is accepted by cvxopt, the prediction check
(``config/truth_reference_acceptance.json``: per joint
``validation_rmse <= 1.25 * sigma_low + 0.01`` N.m) passes, the exported and
reloaded URDF predicts the fitted inertials to ``PARITY_TOL``, and that URDF
stays within ``TRUTH_TOL`` N.m of the noise-free truth effort on the held-out
split.

    python identification_reference.py --case truth      # from examples/ur10
    python identification_reference.py --case all --root /tmp/runs

Shared machinery and limitations: ``examples/identification_reference.py``.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pinocchio as pin

project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from examples import identification_reference as ref  # noqa: E402
from examples.run_record import describe_file  # noqa: E402
from examples.ur10 import identification_truth as truth  # noqa: E402

HERE = Path(__file__).resolve().parent
CONFIG = HERE / "config" / "ur10_truth_reference.yaml"
PROFILE = HERE / "config" / "truth_reference_acceptance.json"
# held-out effort of the reloaded export against the noise-free truth, N.m
TRUTH_TOL = 0.1
# noise level -> (training seed, held-out seed), the first seeds of the protocol
SEEDS = {"none": (None, None), "low": (101, 201)}


def build(args):
    seed, val_seed = SEEDS[args.noise]
    ident = truth.identification(
        noise=args.noise, seed=seed, val_seed=val_seed, config=CONFIG
    )
    ref.apply_instance(ident, args)
    return ident


def inputs(args):
    seed, val_seed = SEEDS[args.noise]

    def describe(ident):
        return {
            "train": describe_file(truth.FIXTURE_DIR / "train.csv", noise_seed=seed),
            "heldout": describe_file(
                truth.FIXTURE_DIR / "validation.csv", noise_seed=val_seed
            ),
            "protocol": describe_file(truth.FIXTURE_DIR / "protocol.yaml"),
            "truth_urdf": describe_file(truth.FIXTURE_DIR / "ur10_truth.urdf"),
        }

    return describe


def after_export(args):
    seed, val_seed = SEEDS[args.noise]

    def check(ident, export):
        """Reloaded export against the truth effort and the base parameters."""
        model = pin.buildModelFromUrdf(export["identified_urdf"]["path"])
        data = model.createData()
        val = truth.load_split("validation", "analytic", args.noise, val_seed)
        tau = np.array(
            [
                pin.rnea(model, data, q, dq, ddq)
                for q, dq, ddq in zip(val["q"], val["dq"], val["ddq"])
            ]
        )
        rmse = np.sqrt(np.mean((tau - val["tau_true"]) ** 2, axis=0))

        proto = truth.protocol()
        names = proto["rank"]["base_names"]
        reference = np.array([proto["rank"]["base_truth"][n] for n in names])
        column_rms = np.array([proto["scaling"]["base_column_rms"][n] for n in names])
        selected = np.asarray(ident.selected.phi_base_equivalent, dtype=float)
        base_error = np.abs((selected - reference) * column_rms)
        report = {
            "heldout_vs_truth_rmse": dict(zip(truth.JOINTS, map(float, rmse))),
            "heldout_vs_truth_tolerance": TRUTH_TOL,
            "base_parameter_error_nm": {
                "max": float(base_error.max()),
                "rms": float(np.sqrt(np.mean(base_error**2))),
                "scaling": "protocol base_column_rms; N.m",
            },
        }
        return report, {"heldout_vs_truth_max": float(rmse.max())}

    return check


def case_truth(args) -> ref.Case:
    return ref.Case(
        robot="ur10",
        name="truth",
        nominal_urdf=truth.URDF,
        config=CONFIG,
        build=build,
        solve_kwargs={"decimate": False, "plotting": False},
        scope="prediction",
        profile=PROFILE,
        expected={
            "requested_stage": "physical_fit",
            "selected_stage": "physical_fit",
            "status": "accepted",
            "solver_status": "optimal",
            "verify_passed": True,
            "exported": True,
            "parity": ("max", ref.PARITY_TOL),
            "heldout_vs_truth_max": ("max", TRUTH_TOL),
            "training_disjoint_from_heldout": True,
            "archive_gaps": [],
        },
        processing={
            "command": "ur10/identification_reference.py --case truth",
            "noise": args.noise,
            "noise_seeds": {
                "train": SEEDS[args.noise][0],
                "heldout": SEEDS[args.noise][1],
            },
            "decimate": False,
            "derivatives": "analytic",
            "acceptance_profile": str(PROFILE.relative_to(HERE)),
        },
        inputs=inputs(args),
        notes={},
        after_export=after_export(args),
    )


CASES = {"truth": case_truth}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--case", choices=(*CASES, "all"), default="all")
    parser.add_argument(
        "--noise",
        choices=tuple(SEEDS),
        default="low",
        help="effort and position noise level of the fixture (default: %(default)s)",
    )
    ref.add_common_args(parser)
    args = parser.parse_args(argv)
    return ref.run_cases(CASES, args.case, args)


if __name__ == "__main__":
    sys.exit(main())
