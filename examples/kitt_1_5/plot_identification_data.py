#!/usr/bin/env python3
"""Plot the collected MuJoCo identification data for the KITT1_5V3 right arm.

For each dataset (training `data/` and held-out `data/validation/`) this writes
four PNG figures to `results/plots/`:

- `{tag}_q.png`   7 joint positions over time (truth curve + noisy samples)
- `{tag}_dq.png`  7 joint velocities
- `{tag}_ddq.png` 7 joint accelerations
- `{tag}_tau.png` 7 joint torques (truth curve + noisy samples)

The truth (noise-free) traces come from `identification_truth.npz` written by
`generate_mujoco_data.py`, so the noise level is clearly visible.  The figure
titles annotate the applied noise std when non-zero.

Usage (from examples/kitt_1_5/):
    python plot_identification_data.py            # training + validation
    python plot_identification_data.py --only train
"""
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # non-interactive, safe for validate.py
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parent
OUT_DIR = ROOT / "results" / "plots"

JOINT_LABELS = [
    "J1",
    "J2",
    "J3",
    "J4",
    "J5",
    "J6",
    "J7",
]

# dataset tag -> (truth npz, q csv, tau csv)
DATASETS = {
    "train": (ROOT / "data" / "identification_truth.npz", "identification_q_simulation.csv", "identification_tau_simulation.csv"),
    "val": (
        ROOT / "data" / "validation" / "identification_truth.npz",
        "identification_q_simulation.csv",
        "identification_tau_simulation.csv",
    ),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot KITT right-arm identification data")
    parser.add_argument(
        "--only",
        choices=["train", "val", "both"],
        default="both",
        help="Which dataset(s) to plot (default both).",
    )
    parser.add_argument(
        "--subsample",
        type=int,
        default=1,
        help="Plot every N-th noisy sample point (speed; default 1 = all).",
    )
    return parser.parse_args()


def plot_quantity(
    axs_flat,
    t: np.ndarray,
    truth: np.ndarray,
    noisy: np.ndarray | None,
    ylabel: str,
    subsample: int,
) -> None:
    n_j = truth.shape[1]
    for j in range(n_j):
        ax = axs_flat[j]
        ax.plot(t, truth[:, j], color=f"C{j}", lw=1.0, alpha=0.9)
        if noisy is not None:
            ax.plot(
                t[::subsample],
                noisy[::subsample, j],
                ".",
                color=f"C{j}",
                ms=1.2,
                alpha=0.45,
                label="noisy" if j == 0 else None,
            )
        ax.set_title(f"{JOINT_LABELS[j]}", fontsize=9)
        ax.set_ylabel(ylabel, fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=8)
    for ax in axs_flat[n_j:]:
        ax.axis("off")


def render(
    tag: str,
    t: np.ndarray,
    q_true: np.ndarray,
    dq_true: np.ndarray,
    ddq_true: np.ndarray,
    tau_true: np.ndarray,
    q_noisy: np.ndarray | None,
    tau_noisy: np.ndarray | None,
    noise_scale: float,
    subsample: int,
) -> list[Path]:
    """Render the four figures for one dataset; returns written paths."""
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    def noise_str(std: float) -> str:
        return f"  (noise std = {std:.1e}, scale = {noise_scale})" if std > 0 else ""

    written = []

    n_j = q_true.shape[1]
    # 2x4 grid holds the 7 joints (last subplot hidden)
    fig, axs = plt.subplots(2, 4, figsize=(16, 6), sharex=True)
    axs_flat = axs.ravel()
    plot_quantity(
        axs_flat, t, q_true, q_noisy, "q [rad]", subsample,
    )
    fig.suptitle(f"{tag} — joint positions" + noise_str(1e-4 * noise_scale), fontsize=11)
    fig.tight_layout()
    p = OUT_DIR / f"{tag}_q.png"
    fig.savefig(p, dpi=120)
    plt.close(fig)
    written.append(p)

    fig, axs = plt.subplots(2, 4, figsize=(16, 6), sharex=True)
    axs_flat = axs.ravel()
    plot_quantity(
        axs_flat, t, dq_true, None, "dq [rad/s]", subsample,
    )
    fig.suptitle(f"{tag} — joint velocities", fontsize=11)
    fig.tight_layout()
    p = OUT_DIR / f"{tag}_dq.png"
    fig.savefig(p, dpi=120)
    plt.close(fig)
    written.append(p)

    fig, axs = plt.subplots(2, 4, figsize=(16, 6), sharex=True)
    axs_flat = axs.ravel()
    plot_quantity(
        axs_flat, t, ddq_true, None, "ddq [rad/s²]", subsample,
    )
    fig.suptitle(f"{tag} — joint accelerations", fontsize=11)
    fig.tight_layout()
    p = OUT_DIR / f"{tag}_ddq.png"
    fig.savefig(p, dpi=120)
    plt.close(fig)
    written.append(p)

    fig, axs = plt.subplots(2, 4, figsize=(16, 6), sharex=True)
    axs_flat = axs.ravel()
    plot_quantity(
        axs_flat, t, tau_true, tau_noisy, "tau [N·m]", subsample,
    )
    fig.suptitle(f"{tag} — joint torques" + noise_str(1e-2 * noise_scale), fontsize=11)
    fig.tight_layout()
    p = OUT_DIR / f"{tag}_tau.png"
    fig.savefig(p, dpi=120)
    plt.close(fig)
    written.append(p)

    return written


def main() -> None:
    args = parse_args()
    tags = ["train", "val"] if args.only == "both" else [args.only]

    for tag in tags:
        truth_path, q_csv, tau_csv = DATASETS[tag]
        if not truth_path.exists():
            print(f"[skip] {tag}: {truth_path} not found — run generate_mujoco_data.py first")
            continue
        truth = np.load(truth_path)
        t = truth["t"]
        noise_scale = float(truth.get("noise_scale", 0.0))
        q_true = truth["q_true"]
        tau_true = truth["tau_true"]
        q_noisy = truth.get("q_noisy", None)
        tau_noisy = truth.get("tau_noisy", None)
        written = render(
            tag, t, q_true, truth["dq_true"], truth["ddq_true"],
            tau_true, q_noisy, tau_noisy, noise_scale, args.subsample,
        )
        for p in written:
            print(f"Wrote {p}")


if __name__ == "__main__":
    main()
