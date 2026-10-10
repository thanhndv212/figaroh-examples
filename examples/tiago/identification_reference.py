"""TIAGo dynamic-identification reference cases (#23).

Both cases fit the shipped ``dynamic`` recording (velocity from positions,
effort of arm_1-arm_4 fitted, #68) and are judged on the ``calibration_slow``
recording, which the fit never uses.

- ``physical-fit``: the physical-consistency-constrained fit. Passes when
  cvxopt reports an optimal solution, the estimate is accepted, the execution
  check passes, and the exported and reloaded URDF predicts the fitted
  inertials to ``PARITY_TOL``. The held-out RMSE (overall and arm_2-arm_4
  pooled, the scale-free metric of #68) is reported, not gated.
- ``reject``: the exact reconstruction is requested. It is not physically
  consistent on this data, so it is rejected: the selected stage is ``none``,
  the verdict fails, the export refuses and no ``parameters.csv`` is written.
  Passes when exactly that happens.

    python identification_reference.py --case physical-fit   # from examples/tiago
    python identification_reference.py --case all --root /tmp/runs

Run cases one after the other, not in parallel. Shared machinery and
limitations: ``examples/identification_reference.py``.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from figaroh.tools.robot import load_robot  # noqa: E402

from examples import identification_reference as ref  # noqa: E402
from examples.run_record import describe_file  # noqa: E402
from examples.tiago import identification as tiago_id  # noqa: E402
from examples.tiago.identification_inputs_audit import summarise_per_joint  # noqa: E402
from examples.tiago.utils.tiago_tools import TiagoIdentification  # noqa: E402

HERE = Path(__file__).resolve().parent
URDF = HERE / "urdf" / "tiago_48_hey5.urdf"
CONFIGS = {
    "physical-fit": HERE / "config" / "tiago_reference_physical_fit.yaml",
    "reject": HERE / "config" / "tiago_reference_reject.yaml",
}
NOTES = {
    "arm_1_joint": "effort scale unidentifiable (#68); relative error is not a gate"
}


def builder(config: Path):
    def build(args):
        robot = load_robot(str(URDF), load_by_urdf=True, robot_pkg="tiago_description")
        ident = TiagoIdentification(robot, str(config))
        ident.velocity_source = "positions"
        ident.velocity_lag = "auto"
        ref.apply_instance(ident, args)
        tiago_id.configure_identification(ident)
        ps = ident.identif_config
        source = ps["validation_data_file"]
        session = tiago_id.identify_evaluation_session(source)
        if session is None or session.role != "validation":
            raise ValueError(f"{source}: not a frozen validation recording")
        ps["evaluation_session"], ps["evaluation_role"] = session.id, session.role
        ident.initialize(truncate=tiago_id.TRUNCATE)
        if not ident._val_available:
            raise RuntimeError(f"validation data could not be loaded: {source}")
        ident.trajectory_provenance[source].update(
            session_id=session.id, role=session.role
        )
        return ident

    return build


def describe_inputs(ident):
    source = ident.identif_config["validation_data_file"]
    out = {
        "protocol": describe_file(HERE / "data/identification/protocol.yaml"),
    }
    for role, key in (("train", "training"), ("heldout", source)):
        prov = ident.trajectory_provenance[key]
        for kind in ("position", "velocity", "effort"):
            out[f"{role}_{kind}"] = describe_file(
                prov[f"{kind}_file"],
                rows=prov["source_rows"],
                **({"truncate": list(tiago_id.TRUNCATE)} if role == "train" else {}),
            )
    return out


def heldout_extra(ident):
    """Held-out arm_2-arm_4 pooled RMSE and arm_1 error, selected estimate."""
    per_joint = ident.result["validation_metrics"]["per_joint"]
    return {
        "recording": ident.identif_config["evaluation_session"],
        "role": ident.identif_config["evaluation_role"],
        "reported_not_gated": True,
        **summarise_per_joint(per_joint),
    }


def _case(name: str, expected: dict, processing: dict):
    def make(args) -> ref.Case:
        return ref.Case(
            robot="tiago",
            name=name,
            nominal_urdf=URDF,
            config=CONFIGS[name],
            build=builder(CONFIGS[name]),
            solve_kwargs={
                "decimate": True,
                "plotting": False,
                "save_results": False,
                "html_report": False,
            },
            scope="execution",
            expected=expected,
            processing={
                "command": f"tiago/identification_reference.py --case {name}",
                "truncate": list(tiago_id.TRUNCATE),
                "decimate": True,
                "wls": False,
                "velocity_source": "positions",
                "velocity_lag": "auto",
                **processing,
            },
            inputs=describe_inputs,
            notes=NOTES,
            heldout_extra=heldout_extra,
        )

    return make


CASES = {
    "physical-fit": _case(
        "physical-fit",
        {
            "requested_stage": "physical_fit",
            "selected_stage": "physical_fit",
            "status": "accepted",
            "solver_status": "optimal",
            "verify_passed": True,
            "exported": True,
            "parity": ("max", ref.PARITY_TOL),
            "training_disjoint_from_heldout": True,
            "archive_gaps": [],
        },
        {"held_out": "calibration_slow, reported not gated"},
    ),
    "reject": _case(
        "reject",
        {
            "requested_stage": "reconstruction",
            "selected_stage": "none",
            "status": "rejected",
            "verify_passed": False,
            "exported": False,
            "training_disjoint_from_heldout": True,
            "archive_gaps": ["export"],
        },
        {"expected_outcome": "rejected, nothing exported"},
    ),
}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--case", choices=(*CASES, "all"), default="all")
    ref.add_common_args(parser)
    args = parser.parse_args(argv)
    return ref.run_cases(CASES, args.case, args)


if __name__ == "__main__":
    sys.exit(main())
