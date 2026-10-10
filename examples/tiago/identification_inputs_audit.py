"""TIAGo identification inputs: reproducible evidence for issue #68.

Every check reads only files shipped in this repository:

- ``data/identification/{dynamic,calibration_slow,calibration_weight}/``, the
  position, velocity, effort and wrist F/T exports of three 2021-07
  recordings (header clock in ``t``);
- ``data/identification/audit/``, channels trimmed from the 2021-07
  introspection bags (original recordings, not distributed) by
  ``utils/audit_extraction.py``; columns in ``data/identification/audit/README.md``;
- the URDFs in ``urdf/``.

Findings reproduced (see docs/development/tiago-identification-inputs-2026-10-10.md):

a. the logged velocity is ``v[n] = a v[n-1] + (1-a) dq/dt`` with ``a = 0.95``,
   not a delayed copy of the position derivative;
b. the torso effort is 1.7-1.8 while lifting at any speed and negative at
   rest, and ~98 % of the converted torso force is the URDF ``m g`` term;
c. PAL's gravity_compensation torque constants for arm_1-arm_4;
d. arm_6/arm_7 positions and efforts are sums and differences of two motor
   channels;
e. wrist efforts are quantised (step 0.001) and mostly exactly zero;
f. the wrist F/T sensor weighs the hand below it at 0.794 kg, near neither URDF;
g. arm_1's axis is vertical and the payload run moves its torque by ~0.1 N.m;
h. (``--gain-scan``, slow) arm_2-arm_4 held-out RMSE is flat in arm_1's kmotor.

Run from examples/tiago: ``python identification_inputs_audit.py [--gain-scan]``.
"""

from __future__ import annotations

import argparse
import contextlib
import io
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pandas as pd
import pinocchio as pin

project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

import examples.tiago.payload_check as pc  # noqa: E402

TIAGO = Path(__file__).resolve().parent
DATA = TIAGO / "data" / "identification"
AUDIT = DATA / "audit"
RUNS = ("dynamic", "calibration_slow", "calibration_weight")
JOINTS = pc.JOINTS
#: First-order velocity filter coefficient logged by PAL's estimate.
VELOCITY_FILTER = 0.95
#: Largest sample shift tried against the filter (samples at ~100 Hz).
MAX_SHIFT = 40
#: Torso speed (m/s) above which the torso is lifting; below, it is at rest.
TORSO_MOVING = 0.005
#: Joint offsets of the differential wrist (rad): the constant residual of the
#: motor-to-joint position map (0.0197129 and -0.0200 to the digits logged).
WRIST_OFFSET = {"arm_6": 0.0197129, "arm_7": -0.0200}
GAIN_FACTORS = (0.25, 0.4, 0.55, 0.75, 1.0, 1.5)
#: Payload mass from the wrist F/T sensor, weight minus training run (kg).
FT_PAYLOAD = 0.489


# --------------------------------------------------------------- helpers
def read(directory: Path, kind: str) -> pd.DataFrame:
    return pd.read_csv(directory / f"tiago_{kind}.csv")


def rms(x) -> float:
    return float(np.sqrt(np.mean(np.square(x))))


# ------------------------------------------------- (a) velocity filter
def filter_fit(q: np.ndarray, v: np.ndarray, t: np.ndarray) -> dict:
    """Fit ``a`` in ``v[n] = a v[n-1] + (1-a) d[n]``; ``d`` is the backward difference.

    Also the residual of the best constant shift, ``v[n] ~ d[n-s]``.
    """
    d = np.diff(q) / np.diff(t)  # d[k] belongs to sample k+1
    v1, v0 = v[1:], v[:-1]
    x, y = v0 - d, v1 - d
    a = float(x @ y / (x @ x))
    scale = float(np.std(v1))
    residual = rms(v1 - (a * v0 + (1 - a) * d)) / scale
    nominal = np.diff(q) / np.median(np.diff(t))
    nominal_residual = rms(v1 - (a * v0 + (1 - a) * nominal)) / scale
    shifts = {
        s: rms(v1[s:] - d[: len(d) - s]) / float(np.std(v1[s:]))
        for s in range(MAX_SHIFT + 1)
    }
    best = min(shifts, key=shifts.get)
    return {
        "a": a,
        "residual": residual,
        "nominal_clock_residual": nominal_residual,
        "best_shift": best,
        "shift_residual": shifts[best],
    }


def velocity_filter(run: str) -> dict:
    """:func:`filter_fit` for every joint of one run, keyed by joint."""
    directory = DATA / run
    pos, vel = read(directory, "position"), read(directory, "velocity")
    t = pos["t"].to_numpy(float)
    if not np.array_equal(t, vel["t"].to_numpy(float)):
        raise ValueError(f"{run}: velocity clock differs from positions")
    return {
        j: filter_fit(
            pos[f"- {j}_position"].to_numpy(float),
            vel[f"- {j}_velocity"].to_numpy(float),
            t,
        )
        for j in JOINTS
    }


# --------------------------------------------------------- (b) torso
def torso_levels() -> dict:
    """Raw torso effort while lifting and at rest, per torso run."""
    out = {}
    for n in (20, 40, 60, 80):
        d = pd.read_csv(AUDIT / f"torso_{n}.csv")
        v, e = d["torso_lift_joint_velocity"].to_numpy(), d["torso_lift_joint_effort"]
        e = e.to_numpy()
        moving = np.where(np.abs(v) > TORSO_MOVING)[0]
        first, last = moving[0], moving[-1]
        out[n] = {
            "peak_speed": float(v.max()),
            "lifting": float(e[v > TORSO_MOVING].mean()),
            "rest": float(e[np.abs(v) <= TORSO_MOVING].mean()),
            "rest_before": float(e[:first].mean()),
            "rest_after": float(e[last + 1 :].mean()),
        }
    return out


def torso_model_share(urdf: Path = pc.URDF) -> dict:
    """Share of the converted torso force that is the URDF ``m g`` term."""
    model = pin.buildModelFromUrdf(str(urdf))
    data = model.createData()
    pin.computeSubtreeMasses(model, data)
    mass = float(data.mass[model.getJointId("torso_lift_joint")])
    effort = read(DATA / "dynamic", "effort")["- torso_lift_joint_effort"].to_numpy()
    window = effort[921:6791]
    scale = pc.REDUCTION_RATIO["torso_lift_joint"] * pc.KMOTOR["torso_lift_joint"]
    weight = 9.81 * mass
    force = scale * window + weight
    return {
        "subtree_mass": mass,
        "weight": weight,
        "converted_mean": float(force.mean()),
        "share": float(weight / force.mean()),
        "raw_std": float(window.std()),
    }


# ----------------------------------------------- (c) controller constants
def controller_constants() -> dict:
    table = pd.read_csv(AUDIT / "controller_constants.csv")
    return {r.joint: (float(r.value), r.channel) for r in table.itertuples()}


# ------------------------------------------------ (d) differential wrist
def differential_wrist() -> dict:
    """Largest absolute residuals of the four motor/joint identities."""
    d = pd.read_csv(AUDIT / "differential_wrist_calibration.csv")
    m6, m7 = d["arm_6_motor_position"], d["arm_7_motor_position"]
    e6, e7 = d["arm_6_motor_effort"], d["arm_7_motor_effort"]
    q6 = (m7 - m6) / 2 + WRIST_OFFSET["arm_6"]
    q7 = (m6 + m7) / 2 + WRIST_OFFSET["arm_7"]
    return {
        "samples": len(d),
        "q6": float(np.abs(d["arm_6_joint_position"] - q6).max()),
        "q7": float(np.abs(d["arm_7_joint_position"] - q7).max()),
        "tau6": float(np.abs(d["arm_6_joint_effort"] - (e7 - e6)).max()),
        "tau7": float(np.abs(d["arm_7_joint_effort"] - (e6 + e7)).max()),
        "equal_efforts": float(
            (d["arm_6_joint_effort"] == d["arm_7_joint_effort"]).mean()
        ),
    }


# ------------------------------------------------ (e) effort quantisation
def quantisation(run: str = "dynamic") -> dict:
    effort = read(DATA / run, "effort")
    out = {}
    for j in JOINTS:
        x = effort[f"- {j}_effort"].to_numpy(float)
        values = np.unique(np.round(x, 9))
        out[j] = {
            "step": float(np.min(np.diff(values))),
            "distinct": len(values),
            "zero_fraction": float(np.mean(x == 0.0)),
        }
    return out


# ----------------------------------------------------- (f) end effector
def urdf_mass_below(urdf: Path, link: str = pc.FT_FRAME) -> float:
    """Sum of link masses below ``link`` (the link itself, the sensor, is excluded)."""
    root = ET.parse(urdf).getroot()
    mass = {
        e.get("name"): float(e.find("inertial/mass").get("value"))
        for e in root.findall("link")
        if e.find("inertial/mass") is not None
    }
    children: dict[str, list[str]] = {}
    for j in root.findall("joint"):
        children.setdefault(j.find("parent").get("link"), []).append(
            j.find("child").get("link")
        )
    todo, total = list(children.get(link, [])), 0.0
    while todo:
        name = todo.pop()
        total += mass.get(name, 0.0)
        todo += children.get(name, [])
    return total


def end_effector(runs=("dynamic",)) -> dict:
    robot = pc.Model()
    measured = {r: pc.ft_mass(pc.Recording.load(DATA / r), robot) for r in runs}
    lines = (AUDIT / "end_effector_channels.txt").read_text().splitlines()
    return {
        "ft_mass": {r: m["mass"] for r, m in measured.items()},
        "hey5": urdf_mass_below(TIAGO / "urdf" / "tiago_48_hey5.urdf"),
        "schunk": urdf_mass_below(TIAGO / "urdf" / "tiago_48_schunk.urdf"),
        "hand_joint_channels": int(lines[0].rsplit(":", 1)[1]),
        "gripper_channels": int(lines[-1].rsplit(":", 1)[1]),
    }


# ---------------------------------------------------------- (g) arm_1
def arm1_axis(robot: pc.Model, rec: pc.Recording, stride: int = 50) -> dict:
    """World-frame direction of arm_1's axis over the weight run."""
    model, data = robot.model, robot.data
    jid = model.getJointId("arm_1_joint")
    idx = robot.idx_v[JOINTS.index("arm_1_joint")]
    q, dq, ddq = pc.kinematics(rec)
    z, gravity = [], []
    for i in range(0, len(rec.t), stride):
        Q, V, A = robot.state(q[i], dq[i], ddq[i])
        pin.computeJointJacobians(model, data, Q)
        J = pin.getJointJacobian(model, data, jid, pin.LOCAL_WORLD_ALIGNED)
        z.append(abs(J[5, idx]))
        gravity.append(pin.computeGeneralizedGravity(model, data, Q)[idx])
    return {"min_abs_z": float(min(z)), "max_gravity": float(np.abs(gravity).max())}


def arm1_payload(baseline: str = "dynamic", loaded: str = "calibration_weight") -> dict:
    """Payload torque on arm_1 against its model error on the payload run.

    The payload is the F/T mass (:data:`FT_PAYLOAD`) and the difference of the
    effort-derived first moments of the two runs (:func:`payload_check.effort_mass`),
    placed on the arm_7 link; its arm_1 torque is the matching
    ``computeJointTorqueRegressor`` columns.
    """
    robot = pc.Model()
    model, data = robot.model, robot.data
    runs = {n: pc.Recording.load(DATA / n) for n in (baseline, loaded)}
    effort = {n: pc.effort_mass(r, robot) for n, r in runs.items()}
    moment = np.array(effort[loaded]["first_moment"]) - np.array(
        effort[baseline]["first_moment"]
    )
    x = np.concatenate([[FT_PAYLOAD], moment])
    rec = runs[loaded]
    q, dq, ddq = pc.kinematics(rec)
    first = 10 * (model.getJointId(pc.PAYLOAD_LINK_JOINT) - 1)
    row = robot.idx_v[JOINTS.index("arm_1_joint")]
    nominal, payload = [], []
    for i in range(0, len(rec.t), pc.EFFORT_STRIDE):
        Q, V, A = robot.state(q[i], dq[i], ddq[i])
        nominal.append(pin.rnea(model, data, Q, V, A)[row])
        reg = pin.computeJointTorqueRegressor(model, data, Q, V, A)
        payload.append(reg[row, first : first + 4] @ x)
    nominal, payload = np.array(nominal), np.array(payload)
    tau = pc.lowpass(rec.effort, rec.t)[
        :: pc.EFFORT_STRIDE, JOINTS.index("arm_1_joint")
    ]
    tau = tau * pc.REDUCTION_RATIO["arm_1_joint"] * pc.KMOTOR["arm_1_joint"]
    dqj = dq[:: pc.EFFORT_STRIDE, JOINTS.index("arm_1_joint")]
    # nominal RNEA plus viscous, Coulomb and offset friction (as payload_check)
    A = np.column_stack([nominal, dqj, np.sign(dqj), np.ones_like(dqj)])
    coef, *_ = np.linalg.lstsq(A, tau, rcond=None)
    return {
        "payload_mass": FT_PAYLOAD,
        "first_moment_difference": moment.tolist(),
        "payload_rms": rms(payload),
        "model_error_rms": rms(tau - A @ coef),
        "signal_std": float(tau.std()),
        "axis": arm1_axis(robot, rec),
    }


# ------------------------------------------------------ (h) gain scan
def gain_scan(factors=GAIN_FACTORS) -> list[dict]:
    """Held-out arm_2-arm_4 RMSE and arm_1 error against arm_1's kmotor (slow)."""
    import examples.tiago.identification as ident

    original = ident.configure_identification
    base = None
    rows = []
    try:
        for f in factors:

            def patched(tiago_iden, f=f):
                original(tiago_iden)
                nonlocal base
                kmotor = tiago_iden.identif_config["kmotor"]
                base = base or kmotor["arm_1_joint"]
                kmotor["arm_1_joint"] = base * f

            ident.configure_identification = patched
            argv, sys.argv = sys.argv, ["identification.py", "--no-archive"]
            try:
                with contextlib.redirect_stdout(
                    io.StringIO()
                ), contextlib.redirect_stderr(io.StringIO()):
                    it = ident.main()
            finally:
                sys.argv = argv
            per_joint = it._compute_validation_metrics()["per_joint"]
            rows.append(
                {"factor": f, "kmotor": base * f, **summarise_per_joint(per_joint)}
            )
    finally:
        ident.configure_identification = original
    return rows


def summarise_per_joint(per_joint) -> dict:
    """Pooled arm_2-arm_4 RMSE and arm_1 relative error from core's per-joint table."""
    entries = per_joint
    rmse = {j: float(entries[j]["rmse_identified"]) for j in entries}
    pooled = np.sqrt(np.mean([rmse[f"arm_{i}_joint"] ** 2 for i in (2, 3, 4)]))
    a1 = entries["arm_1_joint"]
    relative = a1["nrmse"]
    return {
        "pooled_arm2_4": float(pooled),
        "arm1_rmse": rmse["arm_1_joint"],
        "arm1_relative": float(relative),
    }


# ------------------------------------------------------------- report
def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--gain-scan", action="store_true", help="slow, sequential")
    args = parser.parse_args()

    print("TIAGo identification inputs audit (#68)")
    print("\n(a) velocity filter v[n] = a v[n-1] + (1-a) dq/dt, header clock")
    print(
        f"  {'run':<19}{'joint':<18}{'a':>8}{'resid':>10}{'nominal 10 ms':>15}"
        f"{'best shift':>12}{'shift resid':>13}"
    )
    for run in RUNS:
        fits = velocity_filter(run)
        for j in ("arm_3_joint", "torso_lift_joint"):
            f = fits[j]
            print(
                f"  {run:<19}{j:<18}{f['a']:8.5f}{f['residual']:10.1e}"
                f"{f['nominal_clock_residual']:15.1e}{f['best_shift']:12d}"
                f"{f['shift_residual']:13.3f}"
            )
        a = [f["a"] for f in fits.values()]
        shifts = [f["best_shift"] for f in fits.values()]
        resid = [f["residual"] for f in fits.values()]
        shift_resid = [f["shift_residual"] for f in fits.values()]
        print(
            f"  {run:<19}all 8 joints: a {min(a):.5f}-{max(a):.5f}, residual "
            f"{min(resid):.1e}-{max(resid):.1e}, best shift {min(shifts)}-"
            f"{max(shifts)} samples, shift residual "
            f"{min(shift_resid):.3f}-{max(shift_resid):.3f}"
        )

    print("\n(b) torso raw effort (lifting / rest), torso_20..80")
    for n, r in torso_levels().items():
        print(
            f"  torso_{n}: peak {r['peak_speed']:.3f} m/s, lifting {r['lifting']:+.2f}, "
            f"rest {r['rest']:+.2f} (before {r['rest_before']:+.2f}, "
            f"after {r['rest_after']:+.2f})"
        )
    share = torso_model_share()
    print(
        f"  converted torso force: URDF m g {share['weight']:.1f} N "
        f"({share['subtree_mass']:.2f} kg) is {share['share']:.1%} of the mean "
        f"{share['converted_mean']:.1f} N; raw effort std {share['raw_std']:.2f}"
    )

    print("\n(c) PAL gravity_compensation motor torque constants")
    for j, (value, channel) in controller_constants().items():
        print(f"  {j:<12}{value:+.3f}   <- {channel}")

    print("\n(d) differential wrist, max |residual| of the identities")
    d = differential_wrist()
    print(
        f"  {d['samples']} samples: q6 {d['q6']:.1e}, q7 {d['q7']:.1e}, "
        f"tau6 {d['tau6']:.1e}, tau7 {d['tau7']:.1e}; "
        f"arm_6 == arm_7 effort on {d['equal_efforts']:.1%}"
    )

    print("\n(e) effort quantisation, dynamic run")
    for j, q in quantisation().items():
        print(
            f"  {j:<18}step {q['step']:.3f}  distinct {q['distinct']:5d}  "
            f"exactly zero {q['zero_fraction']:.1%}"
        )

    print("\n(f) end effector")
    e = end_effector(RUNS)
    for r, m in e["ft_mass"].items():
        print(f"  F/T mass below sensor, {r:<18}{m:.3f} kg")
    print(
        f"  URDF below {pc.FT_FRAME}: Hey5 {e['hey5']:.3f} kg, Schunk {e['schunk']:.3f} kg"
    )
    print(
        f"  bag channels: {e['hand_joint_channels']} hand joints, "
        f"{e['gripper_channels']} gripper channels"
    )

    print("\n(g) arm_1")
    a = arm1_payload()
    print(
        f"  axis in world frame: |z| >= {a['axis']['min_abs_z']:.6f} over the run, "
        f"max |gravity torque| {a['axis']['max_gravity']:.1e} N.m"
    )
    print(
        f"  payload {a['payload_mass']:.3f} kg, first moment difference "
        f"{np.round(a['first_moment_difference'], 3).tolist()}"
    )
    print(
        f"  payload torque on arm_1: RMS {a['payload_rms']:.3f} N.m; model error "
        f"RMS {a['model_error_rms']:.2f} N.m (nominal RNEA plus friction, in sample); "
        f"signal std {a['signal_std']:.2f}; payload / model error "
        f"{a['payload_rms'] / a['model_error_rms']:.2f}"
    )

    if args.gain_scan:
        print("\n(h) arm_1 kmotor scan (held-out calibration_slow)")
        for r in gain_scan():
            print(
                f"  x{r['factor']:<5} kmotor {r['kmotor']:.4f}  pooled arm_2-arm_4 "
                f"{r['pooled_arm2_4']:.3f}  arm_1 RMSE {r['arm1_rmse']:.3f} "
                f"(relative {r['arm1_relative']:.2f})"
            )


if __name__ == "__main__":
    main()
