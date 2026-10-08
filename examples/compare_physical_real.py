"""Physical-estimator comparison on the recorded TX40 and TIAGo data (#22, D4).

Same methods, objectives and result structure as the UR10 runs
(``examples/ur10/compare_physical_extended.py``; shared machinery in
``examples/physical_comparison.py``), applied to the real recordings. There
is no ground truth here:

* held-out error is the error against the **measured** effort of a disjoint
  temporal block of the same recording (not an independent experiment);
* the effort is a conversion of motor current (TX40) or motor effort (TIAGo)
  with constants that cannot be verified (no torque reference, see the D2
  audit for TIAGo); a method comparison on it ranks estimators on this
  recording, it does not validate the identified parameters;
* frozen-extra: friction, actuator inertia (and TX40 coupling/offset) columns
  are fixed at their training-only least-squares value; joint-extra:
  re-estimated with the inertial parameters (independent extras only).

Only links with positive nominal mass and a non-zero training regressor
column are optimised; the other links stay at the nominal URDF and their
effort is subtracted (``fixed_inertial_columns`` in the result).

    python compare_physical_real.py --robot staubli_tx40 --output <json>
    python compare_physical_real.py --robot tiago
"""

from __future__ import annotations

import argparse
import inspect
import json
import os
import platform
import sys
import tempfile
import time
from pathlib import Path

for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ.setdefault(_v, "1")

import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
import pinocchio as pin  # noqa: E402

HERE = Path(__file__).parent
project_root = HERE.parent
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from examples import physical_comparison as pc  # noqa: E402
from examples.ur10 import compare_physical_estimators as first  # noqa: E402

DEFAULT_OUT_DIR = project_root / "docs" / "development" / "results"
MAX_SAMPLES = 600
SPECS = {
    "staubli_tx40": {
        "urdf": "urdf/tx40_mdh_modified.urdf",
        "config": "config/staubli_tx40_unified_config.yaml",
        "files": ["pos_read_data.csv", "curr_data.csv"],
        "directory": "data",
        "provenance": "real motor encoder / current recording",
        "train": [0, 24750],
        "validation": [29250, 45000],
        "units": "N.m (revolute joints)",
    },
    "tiago": {
        "urdf": "urdf/tiago_48_schunk.urdf",
        "config": "config/tiago_unified_config.yaml",
        "files": ["tiago_position.csv", "tiago_velocity.csv", "tiago_effort.csv"],
        "directory": "data/identification/dynamic",
        "provenance": "real position / velocity / motor-effort recording",
        "train": [921, 4149],
        "validation": [4736, 6791],
        "units": "N on torso_lift_joint, N.m on arm joints",
    },
}


def _identification(name, spec):
    from figaroh.tools.robot import load_robot

    robot = load_robot(spec["urdf"], package_dirs="../../models", load_by_urdf=True)
    if name == "tiago":
        from examples.tiago.identification import configure_identification
        from examples.tiago.utils.tiago_tools import TiagoIdentification

        idn = TiagoIdentification(robot, spec["config"])
        configure_identification(idn)
    else:
        from examples.staubli_tx40.utils.staubli_tx40_tools import TX40Identification

        idn = TX40Identification(robot, spec["config"])
        s = idn.identif_config
        joints = [
            robot.model.joints[robot.model.getJointId(j)] for j in s["active_joints"]
        ]
        s["act_Jid"] = [robot.model.getJointId(j) for j in s["active_joints"]]
        s["act_J"] = joints
        s["act_idxq"] = [j.idx_q for j in joints]
        s["act_idxv"] = [j.idx_v for j in joints]
    return robot, idn


def prepare(name, spec, partition, staging):
    """Regressor, effort and prior of one partition (rows joint-major)."""
    source = Path(spec["directory"])
    lo, hi = spec[partition]
    manifest = []
    for filename in spec["files"]:
        path = source / filename
        frame = pd.read_csv(path)
        if not 0 <= lo < hi <= len(frame):
            raise ValueError(f"Frozen range {lo}:{hi} outside {path} ({len(frame)})")
        frame.iloc[lo:hi].to_csv(staging / filename, index=False)
        manifest.append(
            {
                "file": str(Path("examples") / name / path),
                "sha256": pc.sha256(path),
                "raw_rows": len(frame),
                "range": [lo, hi],
            }
        )
    robot, idn = _identification(name, spec)
    s = idn.identif_config
    s["nb_samples"] = hi - lo
    s["validation_data_file"] = ""
    loader = idn.load_trajectory_data
    idn.load_trajectory_data = lambda data_source=None: loader(str(staging))
    idn.initialize()
    if name == "staubli_tx40":
        idn.add_additional_parameters()
    N = idn.num_samples
    samples = np.unique(np.linspace(20, N - 21, min(MAX_SAMPLES, N - 40), dtype=int))
    rows = np.concatenate([i * N + samples for i in s["act_idxv"]])
    W = idn.dynamic_regressor[rows].copy()
    tau = idn.processed_data["torques"][samples].T.reshape(-1)
    model = robot.model
    nbody = model.njoints - 1
    assert model.nv == nbody, "one-DoF movable joints only"
    prior = np.concatenate([x.toDynamicParameters() for x in list(model.inertias)[1:]])
    data = model.createData()
    err = 0.0
    for k, smp in enumerate(samples):
        q, v, a = [
            idn.processed_data[key][smp]
            for key in ("positions", "velocities", "accelerations")
        ]
        Ypin = pin.computeJointTorqueRegressor(model, data, q, v, a).copy()
        np.testing.assert_allclose(
            W[k :: len(samples), : 10 * nbody],
            Ypin[s["act_idxv"]],
            rtol=1e-10,
            atol=1e-12,
        )
        expected = pin.rnea(model, data, q, v, a)[s["act_idxv"]]
        err = max(err, float(np.max(np.abs(Ypin[s["act_idxv"]] @ prior - expected))))
    assert err < 1e-8, err
    n_extra = W.shape[1] - 10 * nbody
    std = getattr(idn, "standard_parameter", None)
    extra_names = None
    if isinstance(std, dict) and len(std) == W.shape[1]:
        extra_names = [str(k) for k in list(std)[10 * nbody :]]
    if extra_names is None:
        extra_names = [f"extra_{i}" for i in range(n_extra)]
    meta = {
        "files": manifest,
        "samples": len(samples),
        "processed_samples": N,
        "active_joints": s["active_joints"],
        "model_link_names": list(model.names)[1:],
        "filter": idn.filter_config,
        "loader_sha256": pc.sha256(Path(inspect.getfile(type(idn)))),
        "median_timestamp_step": float(
            np.median(np.diff(idn.raw_data["timestamps"].ravel()))
        ),
        "ts": s["ts"],
        "native_regressor_rnea_max_error": err,
        "urdf_sha256": pc.sha256(Path(spec["urdf"])),
        "config_sha256": pc.sha256(Path(spec["config"])),
        "trajectory_provenance": [
            {k: v for k, v in rec.items() if not k.endswith("_file")}
            for rec in json.loads(
                json.dumps(getattr(idn, "trajectory_provenance", {}), default=str)
            ).values()
        ],
    }
    return W, tau, prior, extra_names, meta


def build_data(name):
    spec = SPECS[name]
    os.chdir(project_root / "examples" / name)
    with tempfile.TemporaryDirectory(prefix=f"figaroh-{name}-d4-") as tmp:
        base = Path(tmp)
        (base / "train").mkdir()
        (base / "validation").mkdir()
        W, tau, nominal, extras, mt = prepare(name, spec, "train", base / "train")
        Wv, tauv, nominalv, extras_v, mv = prepare(
            name, spec, "validation", base / "validation"
        )
    np.testing.assert_array_equal(nominal, nominalv)
    assert extras == extras_v
    return W, tau, Wv, tauv, nominal, extras, mt, mv


def make_specs(name, weighting, data):
    W, tau, Wv, tauv, nominal, extras, mt, mv = data
    nbody = len(nominal) // 10
    link_names = mt["model_link_names"]
    joints = mt["active_joints"]
    nv = len(joints)
    links = [
        j
        for j in range(nbody)
        if nominal[10 * j] > 0 and np.linalg.norm(W[:, 10 * j : 10 * j + 10]) > 1e-12
    ]
    cols = np.array([10 * j + k for j in links for k in range(10)])
    fixed = np.setdiff1d(np.arange(10 * nbody), cols)
    offset, offsetv = W[:, fixed] @ nominal[fixed], Wv[:, fixed] @ nominal[fixed]
    E, Ev = W[:, 10 * nbody :], Wv[:, 10 * nbody :]
    scale = np.sqrt(np.mean(tau.reshape(nv, -1) ** 2, axis=1))
    names = [f"{k}_{link_names[j]}" for j in links for k in pc.P10]
    spec = pc.Spec(
        Y=W[:, cols],
        E=E,
        tau=tau - offset,
        Yv=Wv[:, cols],
        Ev=Ev,
        names=names,
        extra_names=extras,
        links=[link_names[j] for j in links],
        joints=joints,
        prior=nominal[cols],
        e_fix=np.zeros(E.shape[1]),
        heldout={"measured_effort": tauv - offsetv},
        row_weight=(
            None if weighting == "none" else np.repeat(1.0 / scale, tau.size // nv)
        ),
        joint_units=["N" if j == "torso_lift_joint" else "N.m" for j in joints],
        nrmse_scale=scale,
    )
    info = {
        "optimized_links": spec.links,
        "fixed_inertial_columns": fixed.tolist(),
        "effort_rms_train_per_joint": dict(zip(joints, scale.tolist())),
        "n_extra_columns": int(E.shape[1]),
    }
    return spec, info


def freeze_extras(spec, pcmp):
    """Training-only least-squares extras (independent ones), then frozen."""
    b = pc.build(spec, pcmp, "joint")
    theta = pc.ols_representative(b.p)
    spec.e_fix = pc.extras_for(spec, b, theta)
    return {
        "kept": [spec.extra_names[c] for c in b.kept],
        "absorbed_in_base": [spec.extra_names[c] for c in b.absorbed],
        "values": dict(zip(spec.extra_names, spec.e_fix.tolist())),
    }


def nominal_row(spec, pcmp):
    """Nominal URDF with frozen extras: no fit, the reference to beat."""
    b = pc.build(spec, pcmp, "frozen")
    row = pc.evaluate(spec, b, spec.prior, pcmp)
    row["feasibility"] = pc.feasibility_fields(b.p, spec.prior)
    row["convergence"] = {"solver": "none (nominal)", "converged": True}
    return row


def provenance(pcmp, spike_info, name):
    import cvxopt
    import picos

    spec = SPECS[name]
    core = pc.core_root(pcmp)
    d = {
        "examples": first.revision(project_root),
        "core": first.revision(core),
        "comparator_sha256": pc.sha256(Path(pcmp.__file__)),
        "script_sha256": pc.sha256(Path(__file__)),
        "shared_module_sha256": pc.sha256(Path(pc.__file__)),
        "log_cholesky_spike": spike_info,
        "pinocchio": pin.__version__,
        "pinocchio_profile": "pin" + "".join(pin.__version__.split(".")[:2]),
        "python": platform.python_version(),
        "numpy": np.__version__,
        "picos": picos.__version__,
        "cvxopt": cvxopt.__version__,
        "picos_solvers": picos.available_solvers(),
        "platform": platform.platform(),
        "max_samples_per_partition": MAX_SAMPLES,
        "robot_provenance": spec["provenance"],
        "split": {
            "train_rows": spec["train"],
            "validation_rows": spec["validation"],
            "kind": "disjoint temporal blocks of one recording; not an "
            "independent experiment",
        },
    }
    try:
        import qics

        d["qics"] = qics.__version__
    except Exception:
        d["qics"] = None
    return d


def run(args) -> dict:
    from figaroh.identification import _physical_comparator as pcmp

    t0 = time.perf_counter()
    name = args.robot
    spike, spike_info = pc.load_spike(pcmp)
    data = build_data(name)
    W, tau, Wv, tauv, nominal, extras, mt, mv = data
    out = {
        "issue": "figaroh-examples#22",
        "robot": name,
        "units": {
            "effort": SPECS[name]["units"],
            "mass": "kg",
            "first_moment": "kg.m",
            "inertia": "kg.m^2",
        },
        "provenance": provenance(pcmp, spike_info, name),
        "train": mt,
        "validation": mv,
        "joint_units": dict(
            zip(
                mt["active_joints"],
                [
                    "N" if j == "torso_lift_joint" else "N.m"
                    for j in mt["active_joints"]
                ],
            )
        ),
        "settings": {
            "weighting": args.weighting,
            "solver": args.solver,
            "second_solver": args.second_solver,
            "policy": repr(pcmp.PhysicalPolicy()),
            "common_objective": "J = ||W(Y theta - tau)||^2 + lam||(theta-theta0)/s||^2",
            "weighting_note": "'scaled' divides each joint's rows by the RMS of its "
            "measured training effort; 'none' is the unweighted fit",
            "heldout_target": "measured effort of a disjoint temporal block",
            "no_truth": "base/parameter errors against truth do not exist for real data",
        },
        "cases": [],
        "cross_check": [],
    }
    for wt in args.weighting:
        spec, info = make_specs(name, wt, data)
        frozen_info = freeze_extras(spec, pcmp)
        entry = {
            "weighting": wt,
            "problem": info,
            "frozen_extras": frozen_info,
            "nominal_prior": nominal_row(spec, pcmp),
        }
        for mode, label in (("frozen", "frozen_extra"), ("joint", "joint_extra")):
            b, methods = pc.run_methods(
                spec,
                mode,
                pcmp,
                spike,
                args.solver,
                second_solver=args.second_solver or None,
            )
            entry[label] = {
                "methods": methods,
                "base_rank": len(b.p.base_indices),
                "n_columns": len(b.p.params_std),
                "removed_columns": len(b.p.removed_columns),
            }
            if mode == "joint":
                entry[label]["extras_kept"] = [spec.extra_names[c] for c in b.kept]
                entry[label]["extras_absorbed"] = [
                    spec.extra_names[c] for c in b.absorbed
                ]
            if args.second_solver:
                cc = pc.cross_check(spec, mode, pcmp, [args.solver, args.second_solver])
                out["cross_check"].append(
                    {
                        "weighting": wt,
                        "mode": mode,
                        "solvers": cc,
                        "agreement": pc.agree(cc, args.solver, args.second_solver),
                    }
                )
        out["cases"].append(entry)
        lc = entry["frozen_extra"]["methods"]["log_cholesky"]
        print(
            f"{name} {wt}: log-Cholesky "
            + " ".join(
                f"{k}:{'conv' if v['convergence']['converged'] else 'not-conv'}"
                for k, v in lc.items()
            ),
            flush=True,
        )
    out["runtime_s"] = time.perf_counter() - t0
    return out


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--robot", required=True, choices=list(SPECS))
    parser.add_argument(
        "--weighting", nargs="+", default=["none", "scaled"], choices=["none", "scaled"]
    )
    parser.add_argument("--solver", default="cvxopt")
    parser.add_argument("--second-solver", default="qics")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args(argv)
    if args.output:
        args.output = args.output.resolve()
    result = run(args)
    out = (
        args.output
        or DEFAULT_OUT_DIR
        / f"{args.robot}-physical-comparison-{result['provenance']['pinocchio_profile']}.json"
    )
    if out.exists() and not args.overwrite:
        parser.error(f"{out} exists; earlier results are preserved (--overwrite)")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=1, allow_nan=False) + "\n")
    print(f"results written to {out} ({result['runtime_s']:.0f} s)")


if __name__ == "__main__":
    main()
