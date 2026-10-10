"""Export PAL statistics bags to the unfiltered TIAGo identification CSVs.

Run with ``python -m examples.tiago.utils.identification_extraction --help``
from the repository root. Reading bags needs the optional ``rosbags`` package;
running the shipped example does not. No ROS installation is required.
"""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path

import numpy as np
import pandas as pd

JOINTS = ("torso_lift_joint",) + tuple(f"arm_{i}_joint" for i in range(1, 8))
KINDS = ("position", "velocity", "effort")
#: Wrist force/torque sensor channels (N, N·m), in the sensor frame.
WRIST_FT = tuple(
    f"wrist_ft_{quantity}_{axis}" for quantity in ("force", "torque") for axis in "XYZ"
)


def read_identification_bag(
    path: str | Path, *, wrist_ft: bool = False
) -> dict[str, pd.DataFrame]:
    """Select named channels, preserving message order and the header clock.

    Names are resolved by names_version, not column offsets. Incomplete or
    non-finite samples are errors rather than silently dropped observations.
    Effort is kept in its raw controller units; velocity is kept filtered.
    With ``wrist_ft`` the six wrist F/T channels are returned too, under the
    ``"wrist_ft"`` key, on the same clock as the joint channels.
    """
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore

    path = Path(path)
    store = get_typestore(Stores.ROS1_NOETIC)
    names, records = {}, []
    with Reader(path) as reader:
        connections = [
            c
            for c in reader.connections
            if c.msgtype.endswith(("/StatisticsNames", "/StatisticsValues"))
        ]
        for connection in connections:
            if connection.msgtype not in store.types:
                store.register(
                    get_types_from_msg(connection.msgdef.data, connection.msgtype)
                )
        for connection, _, raw in reader.messages(connections=connections):
            message = store.deserialize_ros1(raw, connection.msgtype)
            if connection.msgtype.endswith("/StatisticsNames"):
                version = message.names_version
                channels = list(message.names)
                if version in names and names[version] != channels:
                    raise ValueError(f"{path}: conflicting names for version {version}")
                names[version] = channels
            else:
                stamp = message.header.stamp
                records.append(
                    (
                        stamp.sec + stamp.nanosec * 1e-9,
                        message.names_version,
                        np.asarray(message.values),
                    )
                )
    if not records:
        raise ValueError(f"{path}: no statistics samples")
    wanted = [f"{joint}_{kind}" for kind in KINDS for joint in JOINTS]
    if wrist_ft:
        wanted += list(WRIST_FT)
    indices, rows, timestamps = {}, [], []
    for timestamp, version, values in records:
        if version not in names or len(names[version]) != len(values):
            raise ValueError(f"{path}: missing or inconsistent names version {version}")
        if version not in indices:
            channels = names[version]
            missing = [name for name in wanted if name not in channels]
            if missing:
                raise ValueError(f"{path}: missing channels {missing}")
            indices[version] = [channels.index(name) for name in wanted]
        timestamps.append(timestamp)
        rows.append(values[indices[version]])
    t, values = np.asarray(timestamps), np.asarray(rows)
    if not np.all(np.isfinite(t)) or not np.all(np.diff(t) > 0):
        raise ValueError(f"{path}: header timestamps must be finite and increasing")
    if not np.all(np.isfinite(values)):
        raise ValueError(f"{path}: non-finite selected channels")
    frames = {}
    for k, kind in enumerate(KINDS):
        frame = pd.DataFrame({"t": t - t[0]})
        for j, joint in enumerate(JOINTS):
            frame[f"- {joint}_{kind}"] = values[:, k * len(JOINTS) + j]
        frames[kind] = frame
    if wrist_ft:
        frame = pd.DataFrame({"t": t - t[0]})
        for c, name in enumerate(WRIST_FT):
            frame[name] = values[:, len(KINDS) * len(JOINTS) + c]
        frames["wrist_ft"] = frame
    return frames


def export_identification_bag(
    bag: str | Path,
    output_dir: str | Path,
    *,
    expected_sha256: str | None = None,
    wrist_ft: bool = False,
) -> None:
    """Export all samples, optionally requiring a known source bag hash.

    ``wrist_ft`` also writes ``tiago_wrist_ft.csv``; the joint CSVs are
    byte-identical with or without it.
    """
    bag, output_dir = Path(bag), Path(output_dir)
    if expected_sha256 is not None:
        with bag.open("rb") as stream:
            actual = hashlib.file_digest(stream, "sha256").hexdigest()
        if actual != expected_sha256:
            raise ValueError(f"{bag}: source sha256 mismatch ({actual})")
    frames = read_identification_bag(bag, wrist_ft=wrist_ft)
    output_dir.mkdir(parents=True, exist_ok=True)
    for kind, frame in frames.items():
        frame.to_csv(output_dir / f"tiago_{kind}.csv", index=False)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--expected-sha256", help="Source bag hash from protocol.yaml")
    parser.add_argument(
        "--wrist-ft", action="store_true", help="Also export tiago_wrist_ft.csv"
    )
    args = parser.parse_args()
    export_identification_bag(
        args.bag,
        args.output_dir,
        expected_sha256=args.expected_sha256,
        wrist_ft=args.wrist_ft,
    )


if __name__ == "__main__":
    main()
