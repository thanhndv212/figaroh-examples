# Copyright [2021-2025] Thanh Nguyen
# Copyright [2022-2023] [CNRS, Toward SAS]

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
Generate identification datasets for the KITT1_5V3 right arm in MuJoCo.

Pipeline (mirrors the UR10 example's simulation data, but adds configurable
measurement noise and explicit truth-vs-noisy outputs):

1. Load the extracted right-arm submodel MJCF (`urdf/right_arm_robot.mjcf`,
   generated from `right_arm_robot.urdf` by `urdf_to_mjcf.py`).  MuJoCo's
   dynamics then match the Pinocchio RNEA used by the identifier exactly.
2. Sample a trajectory (IPOPT-optimized by default; Fourier fallback when the
   npz is missing or --trajectory fourier is passed).
3. At each sample set qpos/qvel/qacc and call mj_inverse -> joint torques.
4. Optionally add Gaussian measurement noise to q and tau (--noise-scale).
5. Write training CSVs to data/ and a held-out validation set to
   data/validation/ (same filenames as the UR10 convention), plus npz files
   holding the noise-free truth for plotting / cross-checking.

Usage (from examples/kitt_1_5/):
    python generate_mujoco_data.py --noise-scale 1.0
    python generate_mujoco_data.py --noise-scale 0.0
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

try:
    import mujoco
except ImportError:  # pragma: no cover
    print("mujoco is not installed. Run: pip install mujoco", file=sys.stderr)
    sys.exit(1)

ROOT = Path(__file__).resolve().parent
MJCF = ROOT / "urdf" / "right_arm_robot.mjcf"
DEFAULT_TRAJECTORY = ROOT / "data" / "trajectories" / "right_arm_optimal_trajectory.npz"

# The submodel has exactly the 7 right-arm joints, in URDF order.
N_JOINTS = 7

JOINT_NAMES = [
    "right_arm_joint_1",
    "right_arm_joint_2",
    "right_arm_joint_3",
    "right_arm_joint_4",
    "right_arm_joint_5",
    "right_arm_joint_6",
    "right_arm_joint_7",
]

# Measurement noise magnitudes (std) per quantity, scaled by --noise-scale.
Q_NOISE_STD = 1e-4  # rad (encoder resolution scale)
TAU_NOISE_STD = 1e-2  # N.m (torque sensor / current scale)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate MuJoCo identification data for KITT1_5V3 right arm"
    )
    parser.add_argument(
        "--noise-scale",
        type=float,
        default=1.0,
        help="Multiplier on measurement noise std (0 = noise-free). Default 1.0.",
    )
    parser.add_argument(
        "--trajectory",
        type=str,
        choices=["ipopt", "fourier"],
        default="ipopt",
        help="Trajectory source: 'ipopt' loads the optimized npz (default), "
        "'fourier' uses a built-in Fourier excitation trajectory (fallback).",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Seed for measurement noise. Default 42.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=4.0,
        help="Trajectory duration in seconds (Fourier fallback only). Default 4.",
    )
    parser.add_argument(
        "--fs",
        type=float,
        default=100.0,
        help="Sampling frequency in Hz for the generated data. "
        "Must match config sampling_frequency. Default 100.",
    )
    parser.add_argument(
        "--mjfreq",
        type=float,
        default=1000.0,
        help="Physics step frequency for mj_inverse evaluation. Default 1000.",
    )
    return parser.parse_args()


def load_mujoco_model() -> mujoco.MjModel:
    mj_model = mujoco.MjModel.from_xml_path(str(MJCF))
    return mj_model


def load_ipopt_trajectory(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Load the IPOPT-optimized trajectory (t, q, dq, ddq)."""
    data = np.load(path)
    return data["t"], data["q"], data["dq"], data["ddq"]


def fourier_trajectory(
    t: np.ndarray, duration: float, fs: float, seed: int
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Fallback excitation trajectory: sum of harmonically-related sinusoids.

    Each joint gets its own amplitude / phase draw so the motion is rich enough
    to excite the dynamic parameters (low regressor condition number).

    Returns q, dq, ddq of shape (n_samples, n_joints).
    """
    rng = np.random.default_rng(seed)
    # Fundamental frequency so ~5-10 full periods fit in the duration.
    omega = 2.0 * np.pi / duration
    # Keep within ±~60% of the (conservative) joint range so no limit is hit.
    amplitudes = rng.uniform(0.15, 0.45, size=(N_JOINTS, 6))
    phases = rng.uniform(0.0, 2.0 * np.pi, size=(N_JOINTS, 6))
    freqs = np.array([1, 2, 3, 4, 5, 6])  # harmonic multiples

    q = np.zeros((len(t), N_JOINTS))
    dq = np.zeros_like(q)
    ddq = np.zeros_like(q)
    for j in range(N_JOINTS):
        for k in range(6):
            w = freqs[k] * omega
            q[:, j] += amplitudes[j, k] * np.sin(w * t + phases[j, k])
            dq[:, j] += amplitudes[j, k] * w * np.cos(w * t + phases[j, k])
            ddq[:, j] -= amplitudes[j, k] * w**2 * np.sin(w * t + phases[j, k])
    return q, dq, ddq


def run_inverse_dynamics(
    mj_model: mujoco.MjModel,
    mj_data: mujoco.MjData,
    q_traj: np.ndarray,
    dq_traj: np.ndarray,
    ddq_traj: np.ndarray,
) -> np.ndarray:
    """Evaluate mj_inverse at every sample; return the 7 right-arm torques.

    The submodel has no other joints, so qpos/qvel/qacc are set wholesale.
    """
    n_samples = q_traj.shape[0]
    tau = np.zeros((n_samples, N_JOINTS))

    for i in range(n_samples):
        mj_data.qpos[:] = q_traj[i]
        mj_data.qvel[:] = dq_traj[i]

        # Update forward kinematics first. IMPORTANT: mj_forward overwrites
        # qacc from the current forces, so the desired acceleration must be
        # set AFTER forward and immediately before mj_inverse.
        mujoco.mj_forward(mj_model, mj_data)
        mj_data.qacc[:] = ddq_traj[i]
        mujoco.mj_inverse(mj_model, mj_data)
        tau[i] = mj_data.qfrc_inverse.copy()

    return tau


def write_dataset(
    out_dir: Path,
    q_noisy: np.ndarray,
    tau_noisy: np.ndarray,
    truth: dict,
    noise_scale: float,
) -> None:
    """Write training/validation CSVs plus a truth npz for plotting."""
    out_dir.mkdir(parents=True, exist_ok=True)

    q_df_cols = [f"q{i}" for i in range(N_JOINTS)]
    tau_df_cols = [f"tau{i+1}" for i in range(N_JOINTS)]

    # Same filenames as the UR10 example's convention.
    header_q = ",".join(q_df_cols)
    header_tau = ",".join(tau_df_cols)

    np.savetxt(
        out_dir / "identification_q_simulation.csv",
        q_noisy,
        delimiter=",",
        header=header_q,
        comments="",
        fmt="%.9f",
    )
    np.savetxt(
        out_dir / "identification_tau_simulation.csv",
        tau_noisy,
        delimiter=",",
        header=header_tau,
        comments="",
        fmt="%.6f",
    )

    truth["noise_scale"] = noise_scale
    np.savez(out_dir / "identification_truth.npz", **truth)

    print(f"Wrote q/tau CSVs + truth npz to {out_dir}/")


def main(args: argparse.Namespace) -> None:
    fs = args.fs
    dt = 1.0 / fs
    mj_model = load_mujoco_model()
    # Disable joint-limit forces: mj_inverse adds constraint forces whenever a
    # sample overshoots the (narrow, asymmetric) URDF joint ranges — e.g. the
    # excitation trajectory can push joint 4/6 past their asymmetric limits.
    # The identifier solves the ideal RNEA model which has no limits, so the
    # measured torques must be the pure M(q)@ddq + bias, not limit forces.
    mj_model.jnt_limited[:] = False
    mj_data = mujoco.MjData(mj_model)

    # --- Trajectory -----------------------------------------------------
    if args.trajectory == "ipopt" and DEFAULT_TRAJECTORY.exists():
        t, q, dq, ddq = load_ipopt_trajectory(DEFAULT_TRAJECTORY)
        print(f"Loaded IPOPT trajectory: {len(t)} samples, {t[-1]:.2f}s")
    else:
        if args.trajectory == "ipopt":
            print(
                f"[warn] {DEFAULT_TRAJECTORY} not found — "
                "run optimal_trajectory.py first or use --trajectory fourier. "
                "Falling back to Fourier excitation."
            )
        duration = args.duration
        n_samples = int(duration * fs)
        t = np.arange(n_samples) / fs
        q, dq, ddq = fourier_trajectory(t, duration, fs, seed=args.seed)
        print(f"Generated Fourier trajectory: {n_samples} samples, {duration}s")

    # --- Inverse dynamics (noise-free truth) ----------------------------
    print("Running mj_inverse ...")
    tau_true = run_inverse_dynamics(mj_model, mj_data, q, dq, ddq)
    print("Inverse dynamics done.")

    # --- Measurement noise ----------------------------------------------
    rng = np.random.default_rng(args.seed)
    q_noisy = q + rng.normal(0.0, Q_NOISE_STD * args.noise_scale, size=q.shape)
    tau_noisy = tau_true + rng.normal(
        0.0, TAU_NOISE_STD * args.noise_scale, size=tau_true.shape
    )
    print(
        f"Noise: q std={Q_NOISE_STD*args.noise_scale:.2e} rad, "
        f"tau std={TAU_NOISE_STD*args.noise_scale:.2e} N.m"
    )

    # --- Training set ----------------------------------------------------
    write_dataset(
        ROOT / "data",
        q_noisy,
        tau_noisy,
        {
            "t": t,
            "q_true": q,
            "dq_true": dq,
            "ddq_true": ddq,
            "tau_true": tau_true,
            "q_noisy": q_noisy,
            "tau_noisy": tau_noisy,
            "fs": fs,
            "joint_names": JOINT_NAMES,
        },
        args.noise_scale,
    )

    # --- Held-out validation set (same trajectory, independent noise) ----
    rng_val = np.random.default_rng(args.seed + 1000)
    q_val = q + rng_val.normal(0.0, Q_NOISE_STD * args.noise_scale, size=q.shape)
    tau_val = tau_true + rng_val.normal(
        0.0, TAU_NOISE_STD * args.noise_scale, size=tau_true.shape
    )
    write_dataset(
        ROOT / "data" / "validation",
        q_val,
        tau_val,
        {
            "t": t,
            "q_true": q,
            "dq_true": dq,
            "ddq_true": ddq,
            "tau_true": tau_true,
            "q_noisy": q_val,
            "tau_noisy": tau_val,
            "fs": fs,
            "joint_names": JOINT_NAMES,
        },
        args.noise_scale,
    )

    print("Done.")


if __name__ == "__main__":
    args = parse_args()
    main(args)
