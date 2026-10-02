"""Private dataset validation for core issue #22; not a production solver.

Use the existing robot loaders and independently preprocess each partition.
Real-data validation is a separated temporal block, not a new recording.
The core private spike must be explicitly supplied; it is not a public API.
"""

import argparse
import importlib.util
import inspect
import json
import os
import subprocess
import tempfile
import time
from pathlib import Path

import numpy as np
import pandas as pd
import pinocchio as pin
from figaroh.identification.physical_consistency import project_p10_lmi
from figaroh.tools.robot import load_robot

from examples.staubli_tx40.utils.staubli_tx40_tools import TX40Identification
from examples.tiago.utils.tiago_tools import TiagoIdentification
from examples.ur10.utils.ur10_tools import UR10Identification

ROOT = Path(__file__).resolve().parents[1]
SPECS = {
    "ur10": {
        "class": UR10Identification,
        "urdf": "urdf/ur10_robot.urdf",
        "config": "config/ur10_unified_config.yaml",
        "files": [
            "identification_q_simulation.csv",
            "identification_tau_simulation.csv",
        ],
        "directory": "data",
        "provenance": "simulated",
        "train": [0, 500],
        "validation": [0, 400],
        "validation_directory": "data/validation",
    },
    "staubli_tx40": {
        "class": TX40Identification,
        "urdf": "urdf/tx40_mdh_modified.urdf",
        "config": "config/staubli_tx40_unified_config.yaml",
        "files": ["pos_read_data.csv", "curr_data.csv"],
        "directory": "data",
        "provenance": "real motor encoder/current recording",
        "train": [0, 24750],
        "validation": [29250, 45000],
    },
    "tiago": {
        "class": TiagoIdentification,
        "urdf": "urdf/tiago_48_schunk.urdf",
        "config": "config/tiago_unified_config.yaml",
        "files": ["tiago_position.csv", "tiago_velocity.csv", "tiago_effort.csv"],
        "directory": "data/identification/dynamic",
        "provenance": "real position/velocity/effort recording",
        "train": [921, 4149],
        "validation": [4736, 6791],
    },
}
MAX_SAMPLES = 240


def sha(path):
    import hashlib

    return hashlib.sha256(path.read_bytes()).hexdigest()


def prepare(name, spec, partition, staging):
    source = Path(
        spec.get("validation_directory", spec["directory"])
        if partition == "validation"
        else spec["directory"]
    )
    manifest = []
    lo, hi = spec[partition]
    for filename in spec["files"]:
        path = source / filename
        frame = pd.read_csv(path)
        end = (
            hi - 2
            if name == "ur10" and filename.startswith("identification_tau")
            else hi
        )
        if not 0 <= lo < end <= len(frame):
            raise ValueError(
                f"Frozen range {lo}:{hi} is outside {path}: {len(frame)} rows"
            )
        frame.iloc[lo:end].to_csv(staging / filename, index=False)
        manifest.append(
            {
                "file": str(Path("examples") / name / path),
                "sha256": sha(path),
                "raw_rows": len(frame),
                "range": [lo, end],
            }
        )
    robot = load_robot(spec["urdf"], package_dirs="../../models", load_by_urdf=True)
    identification = spec["class"](robot, spec["config"])
    settings = identification.identif_config
    settings["nb_samples"] = hi - lo
    settings["validation_data_file"] = ""
    joints = [
        robot.model.joints[robot.model.getJointId(j)] for j in settings["active_joints"]
    ]
    settings["act_Jid"] = [robot.model.getJointId(j) for j in settings["active_joints"]]
    settings["act_J"] = joints
    settings["act_idxq"] = [j.idx_q for j in joints]
    settings["act_idxv"] = [j.idx_v for j in joints]
    if name == "tiago":
        settings["reduction_ratio"] = dict(
            zip(settings["active_joints"], [1, 100, 100, 100, 100, 336, 336, 336])
        )
        settings["kmotor"] = dict(
            zip(
                settings["active_joints"],
                [1, 0.136, 0.136, -0.087, -0.087, -0.0613, -0.0613, -0.0613],
            )
        )
        settings.update(
            pos_data=str(staging / spec["files"][0]),
            vel_data=str(staging / spec["files"][1]),
            torque_data=str(staging / spec["files"][2]),
        )
    loader = identification.load_trajectory_data
    identification.load_trajectory_data = lambda data_source=None: loader(str(staging))
    identification.initialize()
    # Include the configured TX40 coupling columns explicitly in this benchmark.
    if name == "staubli_tx40":
        identification.add_additional_parameters()
    N = identification.num_samples
    samples = np.unique(np.linspace(20, N - 21, min(MAX_SAMPLES, N - 40), dtype=int))
    rows = np.concatenate([i * N + samples for i in settings["act_idxv"]])
    W = identification.dynamic_regressor[rows].copy()
    tau = identification.processed_data["torques"][samples].T.reshape(-1)
    model = robot.model
    nbody = model.njoints - 1
    assert model.nv == nbody, "This private benchmark requires one-DoF movable joints"
    prior = np.concatenate([x.toDynamicParameters() for x in list(model.inertias)[1:]])
    # Check inertial order against direct native regressors and RNEA, before fitting.
    data = model.createData()
    error = 0.0
    for k, sample in enumerate(samples):
        q, v, a = [
            identification.processed_data[key][sample]
            for key in ["positions", "velocities", "accelerations"]
        ]
        Ypin = pin.computeJointTorqueRegressor(model, data, q, v, a).copy()
        np.testing.assert_allclose(
            W[k :: len(samples), : 10 * nbody],
            Ypin[settings["act_idxv"]],
            rtol=1e-10,
            atol=1e-12,
        )
        expected = pin.rnea(model, data, q, v, a)[settings["act_idxv"]]
        error = max(
            error, float(np.max(np.abs(Ypin[settings["act_idxv"]] @ prior - expected)))
        )
    assert error < 1e-8, error
    return (
        W,
        tau,
        prior,
        {
            "files": manifest,
            "samples": len(samples),
            "processed_samples": N,
            "selected_processed_indices": samples.tolist(),
            "active_joints": settings["active_joints"],
            "model_joint_names": list(model.names)[1:],
            "filter": identification.filter_config,
            "loader_sha256": sha(Path(inspect.getfile(spec["class"]))),
            "median_timestamp_step": float(
                np.median(np.diff(identification.raw_data["timestamps"].ravel()))
            ),
            "ts": settings["ts"],
            "native_regressor_rnea_max_error": error,
            "urdf_sha256": sha(Path(spec["urdf"])),
            "config_sha256": sha(Path(spec["config"])),
        },
    )


def run(name, spike):
    spec = SPECS[name]
    os.chdir(ROOT / "examples" / name)
    with tempfile.TemporaryDirectory(prefix=f"figaroh-{name}-validation-") as temporary:
        base = Path(temporary)
        train, validation = base / "train", base / "validation"
        train.mkdir()
        validation.mkdir()
        W, tau, nominal, train_meta = prepare(name, spec, "train", train)
        Wv, tauv, nominalv, val_meta = prepare(name, spec, "validation", validation)
    np.testing.assert_array_equal(nominal, nominalv)
    nbody = len(nominal) // 10
    # Optimize only positive-mass blocks that affect training torque; fix others at nominal.
    links = [
        j
        for j in range(nbody)
        if nominal[10 * j] > 0 and np.linalg.norm(W[:, 10 * j : 10 * j + 10]) > 1e-12
    ]
    columns = np.array([10 * j + k for j in links for k in range(10)])
    fixed_columns = np.setdiff1d(np.arange(len(nominal)), columns)
    Y, Yv = W[:, columns], Wv[:, columns]
    offset = W[:, fixed_columns] @ nominal[fixed_columns]
    offsetv = Wv[:, fixed_columns] @ nominal[fixed_columns]
    E, Ev = W[:, len(nominal) :], Wv[:, len(nominal) :]
    scale = np.tile([1, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2], len(links))
    prior_raw = nominal[columns]
    prior, prior_changes = spike.repair(prior_raw)
    begin = time.perf_counter()
    total = np.c_[Y, E]
    initial = np.r_[prior, np.zeros(E.shape[1])]
    estimate = (
        initial + np.linalg.lstsq(total, tau - offset - total @ initial, rcond=1e-10)[0]
    )
    ols_seconds = time.perf_counter() - begin
    ols, extras = estimate[: len(prior)], estimate[len(prior) :]
    adjusted = tau - offset - E @ extras
    adjustedv = tauv - offsetv - Ev @ extras
    nv = len(train_meta["active_joints"])
    records = [
        {
            "method": "ols",
            "success": True,
            "runtime_seconds": ols_seconds,
            **spike.metrics(ols, Y, adjusted, Yv, adjustedv, nv),
        }
    ]
    begin = time.perf_counter()
    projected, reports = [], []
    for x, w in zip(ols.reshape(-1, 10), (1 / scale).reshape(-1, 10)):
        p, report = project_p10_lmi(x, weights=w)
        projected.append(p)
        reports.append(
            {
                "status": report.status,
                "objective": report.objective,
                "message": report.message,
            }
        )
    records.append(
        {
            "method": "ols_sdp",
            "success": all(r["status"] == "projected" for r in reports),
            "projection": reports,
            "runtime_seconds": time.perf_counter() - begin,
            **spike.metrics(np.concatenate(projected), Y, adjusted, Yv, adjustedv, nv),
        }
    )
    repaired, changes = spike.repair(ols)
    for label, start, repairs in [
        ("nominal_repaired", prior, prior_changes),
        ("ols_repaired", repaired, changes),
    ]:
        candidate, report = spike.fit(Y, adjusted, prior, scale, start)
        records.append(
            {
                "method": "log_cholesky",
                "initialization": label,
                "repair_p10_norm_per_link": repairs,
                **report,
                **spike.metrics(candidate, Y, adjusted, Yv, adjustedv, nv),
            }
        )
    nominal_record = spike.metrics(prior_raw, Y, tau - offset, Yv, tauv - offsetv, nv)
    singular = np.linalg.svd(total, compute_uv=False)
    rank = int(np.sum(singular > singular[0] * 1e-10))
    return {
        "robot": name,
        "provenance": spec["provenance"],
        "joint_effort_units": [
            "N" if j == "torso_lift_joint" else "Nm"
            for j in train_meta["active_joints"]
        ],
        "train": train_meta,
        "validation": val_meta,
        "split_kind": (
            "separate simulated files"
            if name == "ur10"
            else "disjoint temporal blocks of one recording with gap; not independent experiment"
        ),
        "optimized_links": [train_meta["model_joint_names"][i] for i in links],
        "fixed_inertial_columns": fixed_columns.tolist(),
        "extras": extras.tolist(),
        "extras_policy": "OLS trained on training only; held fixed for SDP and nonlinear fits; no bounds",
        "nominal_repair_p10_norm_per_link": prior_changes,
        "nominal": nominal_record,
        "rank": rank,
        "columns": total.shape[1],
        "observed_condition": float(singular[0] / singular[rank - 1]),
        "records": records,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spike-path", type=Path, required=True)
    parser.add_argument("--robot", choices=list(SPECS), required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    spike_path = args.spike_path.resolve()
    spec = importlib.util.spec_from_file_location("private_spike", spike_path)
    spike = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(spike)
    result = run(args.robot, spike)
    os.chdir(ROOT)
    result.update(
        core_revision=subprocess.check_output(
            ["git", "-C", str(spike_path.parents[3]), "rev-parse", "HEAD"], text=True
        ).strip(),
        examples_revision=subprocess.check_output(
            ["git", "rev-parse", "HEAD"], text=True
        ).strip(),
        script_sha256=sha(Path(__file__)),
        spike_sha256=sha(spike_path),
        solver_config=spike.CONFIG,
        pinocchio=pin.__version__,
        max_samples=MAX_SAMPLES,
    )
    output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
