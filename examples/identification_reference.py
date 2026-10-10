"""Dynamic-identification reference workflow (D7, #23).

One headless command per case: fit, select the estimate, verify it, export it
to a URDF, reload the URDF and check it predicts what the fit predicts, report
per-joint errors on data the fit never used, and archive the run with its
reproduction record. The case passes only when every observed outcome matches
the case's expectation table.

    python identification_reference.py --robot ur10 --case truth
    python identification_reference.py --robot tiago --case all --root /tmp/runs

The robot modules (``ur10/identification_reference.py``,
``tiago/identification_reference.py``) define the cases; this module holds
what they share.

**Export target.** Pinocchio merges links attached by fixed joints into the
moving body (the UR10 ``wrist_3`` body holds the tool mount, the TIAGo
``arm_7`` body holds the hand and the F/T sensor), and the fit identifies the
whole body. :func:`lumped_nominal_urdf` writes each moving body's inertia onto
its child link and removes the inertials of the links fixed to it; Pinocchio
builds the same dynamics from it. The default ``refuse`` export then writes
the fitted inertials into that file.

Run directory (``<root>/<asset>/identification/<timestamp>/``), besides the
standard archive: ``nominal_lumped.urdf``, ``identified.urdf`` (when
exported), ``export_check.json``, ``reference.json``.

Limitations: offline logs only, no hardware deployment is claimed. The
revisions item of the archive audit is ``ok`` only on a clean checkout; from
a dirty one it is reported and does not fail the run unless
``--strict-revisions`` is given. See
docs/development/identification-reference-workflow.md.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

import numpy as np
import pinocchio as pin

project_root = Path(__file__).parents[1]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from figaroh.tools.run_archive import archive_run, compute_run_dir  # noqa: E402

from examples.run_record import (  # noqa: E402
    audit,
    describe_file,
    missing,
    write_reproduction_record,
)
from examples.verification import (  # noqa: E402
    AcceptanceProfile,
    load_acceptance_profile,
    run_verification,
)

CLAIM = "offline logs; no hardware deployment"
# reloaded export against the fitted inertials: max |tau| difference, N.m or N
PARITY_TOL = 1e-6
_INERTIA_KEYS = {
    "ixx": (0, 0),
    "ixy": (0, 1),
    "ixz": (0, 2),
    "iyy": (1, 1),
    "iyz": (1, 2),
    "izz": (2, 2),
}


# --- lumped nominal model ---------------------------------------------------


def lumped_nominal_urdf(src, dst) -> Path:
    """Copy of ``src`` whose moving links carry their whole body's inertia.

    Each Pinocchio joint's body inertia (the link plus every link fixed to
    it) is written on the joint's child link in the link frame, and the
    fixed-attached links lose their inertials. Pinocchio builds the same
    model from the copy.

    Returns:
        ``dst`` as a ``Path``.

    Raises:
        ValueError: the copy builds a different dynamics (largest difference
            of the body dynamic parameters above 1e-9).
    """
    src, dst = Path(src), Path(dst)
    model = pin.buildModelFromUrdf(str(src))
    tree = ET.parse(src)
    root = tree.getroot()
    joints = root.findall("joint")
    child = {j.get("name"): j.find("child").get("link") for j in joints}
    links = {link.get("name"): link for link in root.findall("link")}
    moving = {child[model.names[jid]] for jid in range(1, model.njoints)}
    attached, grew = set(), True
    while grew:
        grew = False
        for j in joints:
            parent, link = j.find("parent").get("link"), j.find("child").get("link")
            if (
                j.get("type") == "fixed"
                and (parent in moving or parent in attached)
                and link not in attached
            ):
                attached.add(link)
                grew = True
    for name in attached:
        inertial = links[name].find("inertial")
        if inertial is not None:
            links[name].remove(inertial)
    for jid in range(1, model.njoints):
        body = model.inertias[jid]
        inertial = links[child[model.names[jid]]].find("inertial")
        if inertial is None:
            inertial = ET.SubElement(links[child[model.names[jid]]], "inertial")
        for tag in ("origin", "mass", "inertia"):
            if inertial.find(tag) is None:
                ET.SubElement(inertial, tag)
        inertial.find("mass").set("value", repr(float(body.mass)))
        origin = inertial.find("origin")
        origin.set("xyz", " ".join(repr(float(v)) for v in body.lever))
        origin.set("rpy", "0 0 0")
        for key, (a, b) in _INERTIA_KEYS.items():
            inertial.find("inertia").set(key, repr(float(body.inertia[a, b])))
    root.insert(
        0,
        ET.Comment(
            f" Lumped nominal model of {src.name} (figaroh-examples #23): each "
            "moving link carries its whole Pinocchio body inertia. Generated "
            "by examples/identification_reference.py; do not edit. "
        ),
    )
    dst.parent.mkdir(parents=True, exist_ok=True)
    tree.write(dst, encoding="utf-8", xml_declaration=True)
    lumped = pin.buildModelFromUrdf(str(dst))
    diff = max(
        np.abs(
            lumped.inertias[j].toDynamicParameters()
            - model.inertias[j].toDynamicParameters()
        ).max()
        for j in range(1, model.njoints)
    )
    if diff > 1e-9:
        raise ValueError(f"lumped model differs from {src.name} by {diff:.3g}")
    return dst


def reference_model(nominal_urdf, link_p10: dict) -> pin.Model:
    """The nominal model with the fitted body inertias written in.

    ``link_p10`` maps joint names to 10-vectors (``selected.link_p10``). This
    does not go through the URDF exporter, so it is independent of it.
    """
    model = pin.buildModelFromUrdf(str(nominal_urdf))
    for joint, p10 in link_p10.items():
        model.inertias[model.getJointId(joint)] = pin.Inertia.FromDynamicParameters(
            np.asarray(p10, dtype=float)
        )
    return model


def effort_parity(model_a: pin.Model, model_b: pin.Model, n=200, seed=0) -> float:
    """Largest |tau_a - tau_b| (RNEA) over ``n`` seeded random states."""
    if (model_a.nq, model_a.nv) != (model_b.nq, model_b.nv):
        raise ValueError("models differ in size")
    rng = np.random.default_rng(seed)
    data_a, data_b = model_a.createData(), model_b.createData()
    worst = 0.0
    for _ in range(n):
        q = pin.randomConfiguration(model_a)
        v, a = rng.normal(size=model_a.nv), rng.normal(size=model_a.nv)
        diff = pin.rnea(model_a, data_a, q, v, a) - pin.rnea(model_b, data_b, q, v, a)
        worst = max(worst, float(np.abs(diff).max()))
    return worst


# --- reporting --------------------------------------------------------------


def per_joint_table(ident, notes: dict | None = None) -> list[dict]:
    """Per joint: unit, fitted or not, training and held-out error.

    Units are N.m for revolute joints and N for prismatic ones. Joints whose
    effort is not fitted (``torque_fit_joints``) are listed without errors.
    """
    metrics = ident.result.get("validation_metrics") or {}
    per = metrics.get("per_joint", {})
    train = (ident.result.get("selected") or {}).get("effort_rmse_fit_per_joint", {})
    rows = []
    for joint, m in per.items():
        rows.append(
            {
                "joint": joint,
                "unit": m["unit"],
                "fitted": True,
                "train_rmse": train.get(joint),
                "heldout_rmse_selected": m["rmse_identified"],
                "heldout_rmse_nominal": m["rmse_nominal"],
                "heldout_nrmse": m["nrmse"],
                "note": (notes or {}).get(joint, ""),
            }
        )
    for joint in metrics.get("not_fitted_joints", []):
        rows.append(
            {
                "joint": joint,
                "fitted": False,
                "note": "effort neither fitted nor scored",
            }
        )
    return rows


def physical_report(ident) -> dict:
    """Solver status and per-link feasibility of the selected estimate."""
    sel = ident.selected
    pf = (ident.result.get("physical_fit") or {}) if ident.result else {}
    return {
        "requested_stage": sel.requested,
        "stage": sel.stage,
        "status": sel.status,
        "reason": sel.reason,
        "solvers": list(sel.solvers),
        "solver_status": pf.get("solver_status"),
        "runtime_s": pf.get("runtime_s"),
        "links": {
            k: {
                "mass": float(v["mass"]),
                "min_eig": float(v["min_eig"]),
                "ok": bool(v["ok"]),
            }
            for k, v in sel.feasibility.items()
        },
    }


def check_expectations(observed: dict, expected: dict) -> list[str]:
    """Mismatches between observed outcomes and a case's expectation table.

    An expected value is compared for equality, or is ``("max", x)`` for an
    upper bound on a number.
    """
    problems = []
    for key, want in expected.items():
        got = observed.get(key)
        if isinstance(want, tuple) and want[0] == "max":
            if got is None or not got <= want[1]:
                problems.append(f"{key}: {got} not <= {want[1]:g}")
        elif got != want:
            problems.append(f"{key}: {got!r}, expected {want!r}")
    return problems


# --- cases ------------------------------------------------------------------


@dataclass
class Case:
    """One reference case. Robot modules build these."""

    robot: str
    name: str
    nominal_urdf: Path
    config: Path
    build: Callable  # (args) -> initialized identification, not yet solved
    solve_kwargs: dict
    scope: str
    expected: dict
    processing: dict
    inputs: Callable  # (ident) -> {name: describe_file entry}
    profile: Path | None = None
    notes: dict = field(default_factory=dict)
    after_export: Callable | None = None  # (ident, urdf) -> (report, observed)
    heldout_extra: Callable | None = None  # (ident) -> dict


def _export(ident, case: Case, run_dir: Path) -> dict:
    """Export into the lumped nominal, reload, and compare with the fit."""
    lumped = lumped_nominal_urdf(case.nominal_urdf, run_dir / "nominal_lumped.urdf")
    out = {
        "nominal_urdf": describe_file(case.nominal_urdf),
        "lumped_nominal": describe_file(lumped),
        "generator": "identification_reference.lumped_nominal_urdf",
        "tolerance": PARITY_TOL,
        "exported": False,
    }
    target = run_dir / "identified.urdf"
    try:
        path = ident.export_urdf(str(lumped), output_path=str(target))
    except ValueError as exc:
        out["error"] = str(exc)
        return out
    joints = ident._identified_joints()
    reloaded = pin.buildModelFromUrdf(path)
    expected = reference_model(case.nominal_urdf, ident.selected.link_p10(joints))
    out.update(
        exported=True,
        identified_urdf=describe_file(path),
        identified_joints=joints,
        parity=effort_parity(reloaded, expected),
    )
    out["passed"] = out["parity"] <= PARITY_TOL
    return out


def run_case(case: Case, args) -> int:
    """Run one case; returns the exit code (0 when it matches expectations)."""
    start = time.time()
    print(f"\n{'=' * 60}\nCASE {case.robot}/{case.name}\n{'=' * 60}")
    ident = case.build(args)
    ident.solve(**case.solve_kwargs)
    sel = ident.selected
    run_dir = compute_run_dir(ident, root=args.root)

    export = _export(ident, case, run_dir)
    profile = load_acceptance_profile(str(case.profile)) if case.profile else None
    print("\nVERIFICATION")
    verdict = run_verification(ident, run_dir, case.scope, profile)
    ident.export_html_report(output_path=str(run_dir / "report.html"))

    table = per_joint_table(ident, case.notes)
    extra_report, extra_observed = {}, {}
    if case.after_export is not None and export["exported"]:
        extra_report, extra_observed = case.after_export(ident, export)
    heldout = (ident.result.get("validation_metrics") or {}).get("per_joint", {})
    report = {
        "case": f"{case.robot}/{case.name}",
        "claim": CLAIM,
        "scope": case.scope,
        "physical": physical_report(ident),
        "per_joint": table,
        "inputs": case.inputs(ident),
        "heldout_overall_rmse": {
            "selected": (ident.result.get("validation_metrics") or {}).get(
                "rmse_identified"
            ),
            "nominal": (ident.result.get("validation_metrics") or {}).get(
                "rmse_nominal"
            ),
            "estimate_stage": (ident.result.get("validation_metrics") or {}).get(
                "estimate_stage"
            ),
            "gated": case.scope == "prediction",
        },
        "heldout_per_joint": heldout,
        **extra_report,
    }
    if case.heldout_extra is not None:
        report["heldout_extra"] = case.heldout_extra(ident)
    train_hash = {
        v["sha256"] for k, v in report["inputs"].items() if k.startswith("train")
    }
    held_hash = {
        v["sha256"] for k, v in report["inputs"].items() if k.startswith("heldout")
    }
    report["training_disjoint_from_heldout"] = bool(
        train_hash and held_hash and not train_hash & held_hash
    )
    (run_dir / "export_check.json").write_text(json.dumps(export, indent=2))
    (run_dir / "reference.json").write_text(json.dumps(report, indent=2, default=str))

    archive_run(ident, run_dir)
    artifacts = {
        name: describe_file(run_dir / name)
        for name in (
            "nominal_lumped.urdf",
            "identified.urdf",
            "export_check.json",
            "reference.json",
        )
        if (run_dir / name).exists()
    }
    write_reproduction_record(
        run_dir,
        processing={
            **case.processing,
            "claim": CLAIM,
            "verification_scope": case.scope,
        },
        inputs=case.inputs(ident),
        artifacts=artifacts,
    )
    gaps = missing(audit(run_dir))

    observed = {
        "requested_stage": sel.requested,
        "selected_stage": sel.stage if sel.accepted else "none",
        "status": sel.status,
        "solver_status": report["physical"]["solver_status"],
        "verify_passed": bool(verdict.passed),
        "exported": export["exported"],
        "parity": export.get("parity"),
        "training_disjoint_from_heldout": report["training_disjoint_from_heldout"],
        "archive_gaps": sorted(set(gaps) - {"revisions"}),
        **extra_observed,
    }
    problems = check_expectations(observed, case.expected)
    if "revisions" in gaps:
        print(
            "\nNote: revisions item incomplete (uncommitted changes or unknown commit)."
        )
        if args.strict_revisions:
            problems.append("archive: revisions incomplete")
    elapsed = time.time() - start
    shown = {k: v for k, v in observed.items() if v is not None}
    print(f"\nObserved: {json.dumps(shown, default=str)}")
    print(f"Run archived to {run_dir} ({elapsed:.1f} s). Claim: {CLAIM}.")
    if problems:
        print(f"\nCASE {case.robot}/{case.name} FAILED: " + "; ".join(problems))
        return 1
    print(f"\nCase {case.robot}/{case.name} matches its expectations.")
    return 0


# --- command line -----------------------------------------------------------


def add_common_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--root", default="results/runs", help="archive root (default: %(default)s)"
    )
    parser.add_argument("--asset-id", default=None, help="physical unit identifier")
    parser.add_argument("--operator", default=None)
    parser.add_argument(
        "--strict-revisions",
        action="store_true",
        help="fail when the archive's revisions item is not ok (dirty checkout)",
    )


def apply_instance(ident, args) -> None:
    """``--asset-id`` / ``--operator`` into the provenance record."""
    if args.asset_id or args.operator:
        instance = dict(ident.identif_config.get("instance") or {})
        instance.update(
            {
                k: v
                for k, v in (("asset_id", args.asset_id), ("operator", args.operator))
                if v
            }
        )
        ident.identif_config["instance"] = instance


def run_cases(cases: dict, selected: str, args) -> int:
    """Run the named case, or every case for ``all``; sequentially."""
    names = list(cases) if selected == "all" else [selected]
    codes = [run_case(cases[name](args), args) for name in names]
    return int(any(codes))


def main(argv=None) -> int:
    """Dispatch ``--robot`` to its module, run from that robot's directory."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--robot", choices=("ur10", "tiago"), required=True)
    args, rest = parser.parse_known_args(argv)
    os.chdir(Path(__file__).parent / args.robot)
    if args.robot == "ur10":
        from examples.ur10 import identification_reference as module
    else:
        from examples.tiago import identification_reference as module
    return module.main(rest)


if __name__ == "__main__":
    sys.exit(main())
