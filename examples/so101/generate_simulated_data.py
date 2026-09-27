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
Generate a simulated SO-101 excitation log with a known ground truth.

Writes the same directory format soarm_sdk's ``soarm-identify-record``
does, so ``identification.py`` cannot tell the two apart — except that the
answer is known and saved next to the data in ``ground_truth.yaml``.

The simulated arm is deliberately *not* the CAD model: its links are
heavier than the URDF says and it carries a 40 g payload at the gripper, so
an identification that merely echoed its prior would fail the comparison.
Torque is the full rigid-body inverse dynamics (inertial terms included,
though the gravity-only fit ignores them), plus friction and a constant
offset per joint, converted to servo current, with sensor noise and the
STS3215's 6.5 mA current quantisation.

    python generate_simulated_data.py                        # data/simulated
    python generate_simulated_data.py --seed 7 --out data/simulated_validation
"""

from __future__ import annotations

import argparse
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pinocchio as pin
import yaml

# Add project root to path for imports (prefer `pip install -e .` instead)
_project_root = Path(__file__).parents[2]
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

from examples.so101.utils.so101_tools import ARM_JOINTS, LOG_FORMAT  # noqa: E402

#: Where the gripper jaw is held, rad (URDF frame).
GRIPPER_HELD = 0.0
#: STS3215 PRESENT_CURRENT resolution.
CURRENT_LSB_MA = 6.5

#: The simulated arm's departure from CAD: link mass scale, and a payload.
MASS_SCALE = {"upper_arm_link": 1.15, "lower_arm_link": 1.10, "wrist_link": 1.20}
PAYLOAD_KG = 0.04
PAYLOAD_XYZ = (0.0, 0.0, -0.06)  # in gripper_link, towards the jaw tips

FRICTION_FV = [0.030, 0.045, 0.040, 0.020, 0.015]  # N·m/(rad/s)
FRICTION_FS = [0.030, 0.050, 0.045, 0.025, 0.020]  # N·m
OFFSET = [0.002, -0.010, 0.006, -0.004, 0.001]  # N·m


def fourier_excitation(
    lower,
    upper,
    *,
    duration,
    rate,
    seed,
    base_freq=0.05,
    n_harmonics=3,
    max_vel=0.5,
    margin=0.15,
    scale=0.6,
    ramp=5.0,
):
    """Same shape as soarm_sdk.dynamics.fourier_excitation, centred on zero."""
    lo, hi = np.asarray(lower) + margin, np.asarray(upper) - margin
    t = np.arange(0.0, duration + 0.5 / rate, 1.0 / rate)

    def smooth(x):
        x = np.clip(x, 0.0, 1.0)
        return x**3 * (x * (6.0 * x - 15.0) + 10.0)

    w = smooth(t / ramp) * smooth((duration - t) / ramp)
    rng = np.random.default_rng(seed)
    k = np.arange(1, n_harmonics + 1)
    om = 2.0 * np.pi * base_freq * k
    q = np.zeros((t.size, lo.size))
    for j in range(lo.size):
        a = rng.uniform(-1, 1, k.size) / k
        b = rng.uniform(-1, 1, k.size) / k
        s = np.sin(np.outer(t, om)) @ a + np.cos(np.outer(t, om)) @ b
        shape = w * (s - s[0])
        shape /= np.max(np.abs(shape))
        amp = scale * min(-lo[j], hi[j])
        peak = amp * np.max(np.abs(np.gradient(shape, t)))
        amp *= min(1.0, max_vel / peak)
        q[:, j] = amp * shape
    return t, q


def true_model(urdf: str) -> pin.Model:
    """The simulated arm: CAD, heavier, with a payload; gripper locked."""
    tree = ET.parse(urdf)
    for link in tree.getroot().findall("link"):
        scale = MASS_SCALE.get(link.get("name"))
        if scale is None:
            continue
        mass = link.find("inertial/mass")
        mass.set("value", repr(float(mass.get("value")) * scale))
        inertia = link.find("inertial/inertia")
        for key, value in inertia.attrib.items():
            inertia.set(key, repr(float(value) * scale))
    model = pin.buildModelFromXML(ET.tostring(tree.getroot(), encoding="unicode"))

    grip = model.frames[model.getFrameId("gripper_link")]
    payload = pin.Inertia(PAYLOAD_KG, np.array(PAYLOAD_XYZ), np.eye(3) * 1e-6)
    model.inertias[grip.parentJoint] = model.inertias[
        grip.parentJoint
    ] + grip.placement.act(payload)
    q_ref = pin.neutral(model)
    jaw = model.getJointId("gripper")
    q_ref[model.joints[jaw].idx_q] = GRIPPER_HELD
    return pin.buildReducedModel(model, [jaw], q_ref)


def ground_truth(model: pin.Model) -> dict:
    bodies = {}
    for name in ARM_JOINTS:
        body = model.inertias[model.getJointId(name)]
        h = body.mass * body.lever
        bodies[name] = {
            "m": float(body.mass),
            "mx": float(h[0]),
            "my": float(h[1]),
            "mz": float(h[2]),
        }
    return {
        "bodies": bodies,
        "friction": {
            j: {"fv": fv, "fs": fs}
            for j, fv, fs in zip(ARM_JOINTS, FRICTION_FV, FRICTION_FS)
        },
        "offset": dict(zip(ARM_JOINTS, OFFSET)),
        "held_positions": {"gripper": GRIPPER_HELD},
    }


def simulate(
    urdf: str,
    out: Path,
    *,
    seed: int,
    duration: float,
    rate: float,
    nm_per_ma: float,
    noise_ma: float,
) -> Path:
    model = true_model(urdf)
    data = model.createData()
    lo = model.lowerPositionLimit
    hi = model.upperPositionLimit
    t, q = fourier_excitation(lo, hi, duration=duration, rate=rate, seed=seed)
    dq = np.gradient(q, t, axis=0)
    ddq = np.gradient(dq, t, axis=0)

    fv, fs, off = map(np.asarray, (FRICTION_FV, FRICTION_FS, OFFSET))
    tau = np.array([pin.rnea(model, data, q[i], dq[i], ddq[i]) for i in range(t.size)])
    tau += fv * dq + fs * np.tanh(dq / 0.01) + off

    rng = np.random.default_rng(seed + 1000)
    current = tau / nm_per_ma + rng.normal(0.0, noise_ma, tau.shape)
    current = np.round(current / CURRENT_LSB_MA) * CURRENT_LSB_MA
    # A servo's own velocity estimate, noisier than differentiating q.
    dq_meas = dq + rng.normal(0.0, 0.02, dq.shape)
    q_meas = q + rng.normal(0.0, 2 * np.pi / 4096 / 2, q.shape)  # half-tick noise

    names = ARM_JOINTS + ["gripper"]
    gripper = np.full((t.size, 1), GRIPPER_HELD)
    out.mkdir(parents=True, exist_ok=True)
    signals = {
        "q": np.hstack([q_meas, gripper]),
        "dq": np.hstack([dq_meas, 0 * gripper]),
        "current_mA": np.hstack([current, 0 * gripper]),
        # PWM duty roughly tracks current at low speed; kept for format parity.
        "load_percent": np.hstack([tau / 0.0294, 0 * gripper]),
        "q_cmd": np.hstack([q, gripper]),
    }
    for sig, arr in signals.items():
        with open(out / f"{sig}.csv", "w") as f:
            f.write(",".join(["t"] + names) + "\n")
            for ti, row in zip(t, arr):
                f.write(f"{ti:.6f}," + ",".join(f"{v:.9g}" for v in row) + "\n")
    meta = {
        "format": LOG_FORMAT,
        "joint_names": names,
        "n_samples": int(t.size),
        "rate_hz": float(rate),
        "frame": "urdf",
        "units": {
            "q": "rad",
            "dq": "rad/s",
            "current_mA": "mA",
            "load_percent": "percent",
            "q_cmd": "rad",
        },
        "arm_id": "simulated",
        "simulated": True,
        "synthetic_torque": True,
        "generator": "figaroh-examples/examples/so101/generate_simulated_data.py",
        "seed": seed,
        "noise_mA": noise_ma,
        "nm_per_mA": nm_per_ma,
    }
    (out / "meta.json").write_text(json.dumps(meta, indent=2) + "\n")
    truth = ground_truth(model)
    truth["nm_per_unit"] = nm_per_ma
    (out / "ground_truth.yaml").write_text(yaml.safe_dump(truth, sort_keys=False))
    return out


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--urdf", default="urdf/so101_new_calib.urdf")
    ap.add_argument("--out", default="data/simulated")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--duration", type=float, default=60.0, help="seconds")
    ap.add_argument("--rate", type=float, default=50.0, help="Hz")
    ap.add_argument(
        "--nm-per-ma",
        type=float,
        default=0.001,
        help="current -> torque scale; match the config's nm_per_unit",
    )
    ap.add_argument(
        "--noise-ma", type=float, default=15.0, help="current noise, 1 sigma"
    )
    args = ap.parse_args()
    out = simulate(
        args.urdf,
        Path(args.out),
        seed=args.seed,
        duration=args.duration,
        rate=args.rate,
        nm_per_ma=args.nm_per_ma,
        noise_ma=args.noise_ma,
    )
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
