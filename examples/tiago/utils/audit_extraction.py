"""Derive the identification-input audit data from exported introspection CSVs (#68).

The 2021-07 introspection bags (original recordings, not distributed) were
exported by ``bag_to_csv`` to ``introspection_datanames.csv`` and
``introspection_datavalues.csv`` beside each bag: one row per sample, a
``values`` list of 511 channels, names in the single row of the names file.
:func:`build` trims those exports to the channels the audit uses and writes
``data/identification/audit/``. Running the audit needs only the shipped
output, not this module.

Usage: ``python -m examples.tiago.utils.audit_extraction <session-dir> ...``
(see the argument names in :func:`main`).
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd

TORSO = "torso_lift_joint"
ARM = tuple(f"arm_{i}_joint" for i in range(1, 8))
#: Rows kept every ``DECIMATION``-th sample for the differential-wrist file.
DECIMATION = 4


def _channel_names(path: Path) -> list[str]:
    """First name from the ``names`` cell, the rest from the wide header."""
    names = pd.read_csv(path)
    columns = list(names.columns)
    first = str(names["names"].iloc[0])
    rest = [c[2:] for c in columns[columns.index("names") + 1 : -1]]
    return [first[2:]] + rest


def load_export(directory: Path) -> pd.DataFrame:
    """Channels of one exported session, indexed by the header clock ``t``."""
    channels = _channel_names(directory / "introspection_datanames.csv")
    raw = pd.read_csv(directory / "introspection_datavalues.csv")
    values = np.array(
        [np.array(v.strip("[]").split(","), float) for v in raw["values"]]
    )
    if values.shape[1] != len(channels):
        raise ValueError(
            f"{directory}: {values.shape[1]} values, {len(channels)} names"
        )
    frame = pd.DataFrame(values, columns=channels)
    t = raw["secs"].to_numpy(float) + raw["nsecs"].to_numpy(float) * 1e-9
    frame.insert(0, "t", t - t[0])
    return frame


def torso_run(frame: pd.DataFrame) -> pd.DataFrame:
    cols = [f"{TORSO}_{k}" for k in ("position", "velocity", "effort")]
    return frame[["t"] + cols]


def controller_constants(frame: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for joint in ARM[:4]:
        channel = f"local_control_motor_torque_constant_{joint}"
        x = frame[channel].to_numpy()
        rows.append(
            {
                "joint": joint,
                "channel": channel,
                "value": float(np.median(x)),
                "min": float(x.min()),
                "max": float(x.max()),
            }
        )
    return pd.DataFrame(rows)


def differential_wrist(frame: pd.DataFrame) -> pd.DataFrame:
    cols = []
    for j in (6, 7):
        cols += [
            f"arm_{j}_motor_position",
            f"arm_{j}_motor_effort",
            f"arm_{j}_joint_position",
            f"arm_{j}_joint_effort",
        ]
    return frame[["t"] + cols].iloc[::DECIMATION]


def end_effector(frame: pd.DataFrame) -> list[str]:
    hand = sorted(
        c[: -len("_position")]
        for c in frame.columns
        if c.startswith("hand_") and c.endswith("_joint_position")
    )
    motors = sorted(
        c[: -len("_position")]
        for c in frame.columns
        if c.startswith("hand_") and c.endswith("_motor_position")
    )
    lines = [f"hand joints with logged position channels: {len(hand)}"]
    lines += hand
    lines.append(f"hand motors with logged position channels: {len(motors)}")
    lines += motors
    grip = [c for c in frame.columns if "gripper" in c or "finger" in c]
    lines.append(f"channels containing 'gripper' or 'finger': {len(grip)}")
    return lines


def channel_status(frame: pd.DataFrame) -> pd.DataFrame:
    joints = (TORSO,) + ARM
    rows = []
    for joint in joints:
        motor = joint.replace("_joint", "_motor")
        rows.append(
            {
                "joint": joint,
                "torque_sensor_nan_fraction": float(
                    frame[f"{joint}_torque_sensor"].isna().mean()
                ),
                "motor_mode_values": ";".join(
                    str(int(v)) for v in np.unique(frame[f"{motor}_mode"].dropna())
                ),
                "effort_command_nan_fraction": float(
                    frame[f"{motor}_motor_effort_command"].isna().mean()
                ),
                "max_abs_effort_command": float(
                    frame[f"{motor}_motor_effort_command"].abs().max()
                ),
            }
        )
    return pd.DataFrame(rows)


def build(sessions: dict[str, Path], calibration: Path, out: Path) -> None:
    """Write every audit file. ``sessions`` maps ``torso_20`` ... to export dirs."""
    out.mkdir(parents=True, exist_ok=True)
    for name, directory in sessions.items():
        torso_run(load_export(directory)).to_csv(out / f"{name}.csv", index=False)
    frame = load_export(calibration)
    controller_constants(frame).to_csv(out / "controller_constants.csv", index=False)
    differential_wrist(frame).to_csv(
        out / "differential_wrist_calibration.csv", index=False
    )
    channel_status(frame).to_csv(out / "channel_status.csv", index=False)
    (out / "end_effector_channels.txt").write_text(
        "\n".join(end_effector(frame)) + "\n"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--calibration",
        type=Path,
        required=True,
        help="export directory of the calibration recording",
    )
    parser.add_argument(
        "--torso",
        type=Path,
        required=True,
        help="directory holding torso_{20,40,60,80}/ export directories",
    )
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    sessions = {f"torso_{n}": args.torso / f"torso_{n}" for n in (20, 40, 60, 80)}
    build(sessions, args.calibration, args.out)


if __name__ == "__main__":
    main()
