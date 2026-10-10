"""TIAGo identification: payload check of the effort scale (#69).

The payload recording (``calibration_weight``) follows the training path
(``dynamic``) with a payload in the hand. Two independent estimates of that
payload's mass:

- **F/T sensor (reference).** On low-acceleration samples, fit
  ``F = R_sensorᵀ·(m·g) + bias`` to the wrist force; ``m`` is the mass below
  the sensor. The payload is ``m(weight) − m(training)``.
- **Joint efforts (under test).** Convert the arm_2–arm_4 efforts with the
  example's drive constants (the ``drives`` table in
  config/tiago_unified_config.yaml), subtract the
  nominal RNEA torque and fit an extra mass and first moment on the arm_7
  link, with viscous, Coulomb and offset friction per joint. The payload is
  the difference of the two runs' extra masses, so model errors common to
  both runs cancel.

The relative error of the effort payload against the F/T payload measures
the effort scale on arm_2–arm_4. :data:`TOLERANCE` is justified in
docs/development/tiago-identification-cross-run-protocol.md. With today's
constants the efforts read low and the check reports it; it does not fail
the run. Checked by tests/test_tiago_identification_datasets.py.

Run from examples/tiago: ``python payload_check.py``.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd
import pinocchio as pin
from scipy.signal import butter, filtfilt

project_root = Path(__file__).parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from examples.tiago.identification import load_drives  # noqa: E402

TIAGO = Path(__file__).resolve().parent
DATA = TIAGO / "data" / "identification"
REDUCTION_RATIO, KMOTOR = load_drives(TIAGO / "config" / "tiago_unified_config.yaml")
#: The hand fitted to these recordings is the Hey5 (#68). The model only
#: enters through terms common to both runs, which cancel in the payload.
URDF = TIAGO / "urdf" / "tiago_48_hey5.urdf"
JOINTS = ("torso_lift_joint",) + tuple(f"arm_{i}_joint" for i in range(1, 8))
#: Joints whose efforts carry the payload and have usable resolution (#68).
EFFORT_JOINTS = ("arm_2_joint", "arm_3_joint", "arm_4_joint")
FT_FRAME = "wrist_ft_link"
PAYLOAD_LINK_JOINT = "arm_7_joint"
#: Low-pass on positions and efforts before differentiation (Hz, order 4,
#: zero phase): the identification filter's cutoff.
CUTOFF_HZ = 2.0
#: F/T samples whose sensor-frame linear acceleration is below this (m/s²)
#: are quasi-static. The payload moves by ±0.008 kg between 0.05 and 0.4.
STATIC_ACCELERATION = 0.1
#: Use every Nth sample in the effort fit (the signal is low-passed at 2 Hz).
EFFORT_STRIDE = 5
#: Accepted |relative error| of the effort payload against the F/T payload.
TOLERANCE = 0.10


@dataclass(frozen=True)
class Recording:
    """One session: recorded clock, joint positions, raw efforts, wrist F/T."""

    t: np.ndarray
    q: np.ndarray
    effort: np.ndarray
    force: np.ndarray

    @classmethod
    def load(cls, directory: Path) -> "Recording":
        frames = {
            kind: pd.read_csv(directory / f"tiago_{kind}.csv")
            for kind in ("position", "effort", "wrist_ft")
        }
        t = frames["position"]["t"].to_numpy(float)
        for kind in ("effort", "wrist_ft"):
            if not np.array_equal(t, frames[kind]["t"].to_numpy(float)):
                raise ValueError(f"{directory}: {kind} clock differs from positions")
        return cls(
            t=t,
            q=frames["position"][[f"- {j}_position" for j in JOINTS]].to_numpy(float),
            effort=frames["effort"][[f"- {j}_effort" for j in JOINTS]].to_numpy(float),
            force=frames["wrist_ft"][[f"wrist_ft_force_{a}" for a in "XYZ"]].to_numpy(
                float
            ),
        )


def lowpass(x: np.ndarray, t: np.ndarray) -> np.ndarray:
    rate = 1.0 / float(np.median(np.diff(t)))
    b, a = butter(4, CUTOFF_HZ / (rate / 2))
    return filtfilt(b, a, x, axis=0)


class Model:
    """Pinocchio model with the recorded joints mapped into its q/v."""

    def __init__(self, urdf: Path = URDF):
        self.model = pin.buildModelFromUrdf(str(urdf))
        self.data = self.model.createData()
        joints = [self.model.joints[self.model.getJointId(j)] for j in JOINTS]
        if any(J.nq != 1 or J.nv != 1 for J in joints):
            raise ValueError(f"{urdf}: recorded joints must be 1-DoF")
        self.idx_q = [J.idx_q for J in joints]
        self.idx_v = [J.idx_v for J in joints]

    def state(self, q, dq, ddq):
        Q, V, A = (
            pin.neutral(self.model),
            np.zeros(self.model.nv),
            np.zeros(self.model.nv),
        )
        Q[self.idx_q], V[self.idx_v], A[self.idx_v] = q, dq, ddq
        return Q, V, A


def kinematics(rec: Recording):
    """Filtered positions and their first two derivatives on the recorded clock."""
    q = lowpass(rec.q, rec.t)
    dq = np.gradient(q, rec.t, axis=0)
    return q, dq, np.gradient(dq, rec.t, axis=0)


def ft_mass(rec: Recording, robot: Model) -> dict:
    """Mass below the F/T sensor from quasi-static wrist forces."""
    model, data = robot.model, robot.data
    frame = model.getFrameId(FT_FRAME)
    gravity = np.array([0.0, 0.0, -9.81])
    q, dq, ddq = kinematics(rec)
    rows, rhs = [], []
    for i in range(len(rec.t)):
        pin.forwardKinematics(model, data, *robot.state(q[i], dq[i], ddq[i]))
        pin.updateFramePlacement(model, data, frame)
        acceleration = pin.getFrameClassicalAcceleration(
            model, data, frame, pin.LOCAL_WORLD_ALIGNED
        ).linear
        if np.linalg.norm(acceleration) < STATIC_ACCELERATION:
            R = data.oMf[frame].rotation
            rows.append(np.column_stack([R.T @ gravity, np.eye(3)]))
            rhs.append(rec.force[i])
    A, b = np.vstack(rows), np.concatenate(rhs)
    x, *_ = np.linalg.lstsq(A, b, rcond=None)
    residual = b - A @ x
    return {
        "mass": float(x[0]),
        "bias": x[1:].tolist(),
        "residual_rms": float(np.sqrt(np.mean(residual**2))),
        "static_fraction": len(rows) / len(rec.t),
    }


def effort_mass(rec: Recording, robot: Model) -> dict:
    """Extra mass and first moment on the arm_7 link from converted efforts."""
    model, data = robot.model, robot.data
    q, dq, ddq = kinematics(rec)
    tau = lowpass(rec.effort, rec.t)
    cols = [JOINTS.index(j) for j in EFFORT_JOINTS]
    scale = np.array([REDUCTION_RATIO[j] * KMOTOR[j] for j in EFFORT_JOINTS])
    rows_v = [robot.idx_v[c] for c in cols]
    first = 10 * (model.getJointId(PAYLOAD_LINK_JOINT) - 1)
    n = len(EFFORT_JOINTS)
    A, b = [], []
    for i in range(0, len(rec.t), EFFORT_STRIDE):
        Q, V, Acc = robot.state(q[i], dq[i], ddq[i])
        nominal = pin.rnea(model, data, Q, V, Acc)[rows_v]
        # m, m·cx, m·cy, m·cz of the arm_7 link (inertia is not identifiable
        # from a payload this close to the wrist axis)
        inertial = pin.computeJointTorqueRegressor(model, data, Q, V, Acc)[
            rows_v, first : first + 4
        ]
        friction = np.zeros((n, 3 * n))
        for r, c in enumerate(cols):
            friction[r, 3 * r : 3 * r + 3] = [dq[i, c], np.sign(dq[i, c]), 1.0]
        A.append(np.hstack([inertial, friction]))
        b.append(tau[i, cols] * scale - nominal)
    x, *_ = np.linalg.lstsq(np.vstack(A), np.concatenate(b), rcond=None)
    return {"mass": float(x[0]), "first_moment": x[1:4].tolist()}


def check(
    baseline: str = "dynamic", loaded: str = "calibration_weight", urdf: Path = URDF
) -> dict:
    """Payload from the F/T sensor and from the efforts, and their agreement."""
    robot = Model(urdf)
    runs = {name: Recording.load(DATA / name) for name in (baseline, loaded)}
    ft = {name: ft_mass(rec, robot) for name, rec in runs.items()}
    effort = {name: effort_mass(rec, robot) for name, rec in runs.items()}
    ft_payload = ft[loaded]["mass"] - ft[baseline]["mass"]
    effort_payload = effort[loaded]["mass"] - effort[baseline]["mass"]
    error = effort_payload / ft_payload - 1.0
    return {
        "baseline": baseline,
        "loaded": loaded,
        "ft": ft,
        "effort": effort,
        "ft_payload": ft_payload,
        "effort_payload": effort_payload,
        "relative_error": error,
        "tolerance": TOLERANCE,
        "consistent": abs(error) <= TOLERANCE,
    }


def main() -> None:
    result = check()
    print("TIAGo payload check of the effort scale (#69)")
    for name in (result["baseline"], result["loaded"]):
        ft, effort = result["ft"][name], result["effort"][name]
        print(
            f"  {name:<20} F/T mass {ft['mass']:.3f} kg "
            f"(residual {ft['residual_rms']:.2f} N, "
            f"{ft['static_fraction']:.0%} static)   "
            f"effort extra mass {effort['mass']:+.3f} kg"
        )
    print(f"  Payload, F/T sensor:  {result['ft_payload']:.3f} kg (reference)")
    print(f"  Payload, efforts:     {result['effort_payload']:.3f} kg")
    verdict = (
        "consistent"
        if result["consistent"]
        else (
            "efforts read low" if result["relative_error"] < 0 else "efforts read high"
        )
    )
    print(
        f"  Effort scale error:   {result['relative_error']:+.1%} "
        f"(tolerance ±{TOLERANCE:.0%}): {verdict}"
    )


if __name__ == "__main__":
    main()
