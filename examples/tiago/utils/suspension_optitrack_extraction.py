"""Joint and OptiTrack body streams of the TIAGo suspension runs (#112).

Eleven single-oscillation runs of 2023-07-26/28 and eleven sinusoidal runs of
2023-09-12 excite the base by moving the arm, with and without a weight in the
gripper, while OptiTrack tracks rigid bodies on the robot. There is no force
plate. Every run is one ROS bag; :func:`extract_run` writes, per run:

* ``joints.csv``: ``t``, then position and effort of ``torso_lift`` and
  ``arm_1`` to ``arm_7`` and the six wrist force-torque channels, one row per
  PAL introspection message (100 Hz). The recorded joint velocity is the
  controller's estimate and is not shipped. Written by
  :func:`examples.tiago.utils.introspection_extraction.channel_table`, so the
  layout and number format are those of the joint-inspection files (#110).
* ``<body>.csv`` per tracked body: ``t, x, y, z, qx, qy, qz, qw, live``, one row
  per NatNet message (about 120 Hz), in bag order. Nothing is dropped:
  ``live = 0`` marks a frozen sample (the 7 pose values repeat the previous
  sample bit for bit; NatNet republishes the last pose with a new stamp when a
  body is lost), the rule of
  :func:`examples.tiago.utils.optitrack_extraction.drop_frozen`.

One clock. ``t`` is the robot clock in seconds from the first introspection
header stamp, rounded to 1 microsecond, in every file of a run. Body rows are
the NatNet header stamp shifted by the measured lag, ``t_robot = t_mocap +
lag``. The lag is :func:`~examples.tiago.utils.optitrack_extraction.measure_clock_lag`
on the live ``tiago_shoulder`` samples, the body every run tracks: the
difference of the median receive-minus-header offsets of the mocap and robot
streams. It is near -0.05 s on 2023-07 and -1.38 to -1.32 s on 2023-09-12.

Values are ``repr`` of the recorded float64, unchanged (NatNet poses are
float32 widened to float64 by ROS, hence their long text). Pose is in the
OptiTrack world frame (m, unit quaternion, scalar last). The original bags are
not distributed; :func:`audit` counts the samples, frozen samples and repeated
or backward stamps of each stream for ``protocol.yaml``. Run

    python -m examples.tiago.utils.suspension_optitrack_extraction --help

from the repository root (needs the optional ``rosbags`` package). The
byte-for-byte check against the shipped files is
``tests/test_tiago_suspension_optitrack.py`` (needs ``TIAGO_RECORDINGS_ROOT``).
"""

from __future__ import annotations

import io
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from examples.tiago.utils.introspection_extraction import (
    channel_table,
    read_introspection,
)
from examples.tiago.utils.mocap_extraction import file_sha256
from examples.tiago.utils.optitrack_extraction import measure_clock_lag

#: The body tracked in every run; the clock lag is measured on it.
REFERENCE_BODY = "tiago_shoulder"
BODY_PREFIX = "/natnet_ros/"
BODY_COLUMNS = ("t", "x", "y", "z", "qx", "qy", "qz", "qw", "live")
JOINTS = ("torso_lift",) + tuple(f"arm_{i}" for i in range(1, 8))
KINDS = ("position", "effort")
WRIST_FT = tuple(f"wrist_ft_{k}_{a}" for k in ("force", "torque") for a in "XYZ")


def joint_channels() -> List[Tuple[str, str]]:
    """``(column, channel)`` pairs of ``joints.csv``."""
    out = [(f"{j}_{k}", f"{j}_joint_{k}") for j in JOINTS for k in KINDS]
    return out + [(c, c) for c in WRIST_FT]


def read_bodies(path: str) -> Dict[str, np.ndarray]:
    """NatNet body streams of a bag, ``{body: (n, 9)}`` in bag order.

    Columns ``t_header, t_recv, x, y, z, qx, qy, qz, qw``.
    """
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_typestore

    ts = get_typestore(Stores.ROS1_NOETIC)
    bodies: Dict[str, list] = {}
    with Reader(path) as r:
        conns = [c for c in r.connections if c.topic.startswith(BODY_PREFIX)]
        for c, t, raw in r.messages(connections=conns):
            m = ts.deserialize_ros1(raw, c.msgtype)
            p, o = m.pose.position, m.pose.orientation
            stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            body = c.topic[len(BODY_PREFIX) :].split("/")[0]
            bodies.setdefault(body, []).append(
                (stamp, t * 1e-9, p.x, p.y, p.z, o.x, o.y, o.z, o.w)
            )
    return {b: np.array(s, dtype=float) for b, s in sorted(bodies.items())}


def live_mask(stream: np.ndarray) -> np.ndarray:
    """``True`` where the pose is a measurement, ``False`` where it is frozen."""
    if len(stream) == 0:
        return np.zeros(0, dtype=bool)
    same = np.all(stream[1:, 2:9] == stream[:-1, 2:9], axis=1)
    return np.r_[True, ~same]


def robot_stamps(rows: Sequence[tuple]) -> Tuple[np.ndarray, np.ndarray]:
    """``(t_header, t_recv)`` of introspection rows."""
    header = np.array([r[2] + r[3] * 1e-9 for r in rows])
    recv = np.array([r[0] * 1e-9 for r in rows])
    return header, recv


def run_lag(rows: Sequence[tuple], reference: np.ndarray) -> float:
    """``t_robot - t_mocap`` from the live samples of the reference body."""
    live = reference[live_mask(reference)]
    return measure_clock_lag(robot_stamps(rows), (live[:, 0], live[:, 1]))


def body_csv(stream: np.ndarray, lag: float, t0: float) -> str:
    """One body stream on the robot clock, every sample, frozen ones marked."""
    out = io.StringIO()
    out.write(",".join(BODY_COLUMNS) + "\n")
    live = live_mask(stream)
    for row, ok in zip(stream, live):
        out.write(repr(round(float(row[0]) + lag - t0, 6)) + ",")
        out.write(",".join(repr(float(v)) for v in row[2:9]))
        out.write(f",{int(ok)}\n")
    return out.getvalue()


def _stamp_counts(t: np.ndarray) -> Dict[str, int]:
    d = np.diff(t)
    return {"repeated_stamps": int(np.sum(d == 0)), "backward_stamps": int(np.sum(d < 0))}


def audit(rows: Sequence[tuple], bodies: Dict[str, np.ndarray]) -> dict:
    """Sample counts, frozen samples and stamp defects of every stream."""
    header, _ = robot_stamps(rows)
    out = {"joints": {"samples": len(rows), **_stamp_counts(header)}}
    for body, s in bodies.items():
        out[body] = {
            "samples": len(s),
            "frozen": int(np.sum(~live_mask(s))),
            **_stamp_counts(s[:, 0]),
        }
    return out


def extract_run(
    bag: str, expected_sha256: Optional[str] = None
) -> Tuple[Dict[str, str], float, dict]:
    """Bag to ``({file name: text}, lag, audit)``.

    ``expected_sha256`` refuses a bag that is not the recorded one.
    """
    if expected_sha256 is not None:
        actual = file_sha256(bag)
        if actual != expected_sha256:
            raise ValueError(f"{bag}: source sha256 mismatch ({actual})")
    names_msg, rows = read_introspection(bag)
    bodies = read_bodies(bag)
    if REFERENCE_BODY not in bodies:
        raise ValueError(f"{bag}: no {REFERENCE_BODY} stream")
    lag = run_lag(rows, bodies[REFERENCE_BODY])
    t0 = rows[0][2] + rows[0][3] * 1e-9
    files = {"joints.csv": channel_table(names_msg, rows, joint_channels())}
    for body, s in bodies.items():
        files[f"{body}.csv"] = body_csv(s, lag, t0)
    return files, lag, audit(rows, bodies)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("bag")
    parser.add_argument("out_dir")
    parser.add_argument("--expected-sha256", help="Source bag hash from protocol.yaml")
    args = parser.parse_args()
    files, lag, counts = extract_run(args.bag, args.expected_sha256)
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    for name, text in files.items():
        (out / name).write_text(text)
    print(f"lag {lag:+.4f} s -> {out}: {counts}")
