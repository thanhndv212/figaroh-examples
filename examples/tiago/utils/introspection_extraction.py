"""PAL introspection exports from ROS bags (backlash and joint inspection).

TIAGo's PAL statistics (``/introspection_data/names`` and
``/introspection_data/values``) are logged at 100 Hz beside the joint states.
``bag_to_csv`` exported them to ``introspection_datanames.csv`` and
``introspection_datavalues.csv`` (the layout of
``examples/tiago/data/backlash/``): the single names message in one row, the
first channel name in the ``names`` cell and the others in the header with a
``- `` prefix, and one row per values message with the channel values as a
Python list. :func:`names_csv` and :func:`values_csv` write that layout from a
bag so the loader ``backlash_surface.load_backlash_joint_trajectory`` reads it
unchanged. Python's ``repr`` of a float gives the same text as the exporter;
the byte-for-byte check against an exporter-written CSV of the same bag is in
``tests/test_tiago_introspection_extraction.py`` (needs
``TIAGO_RECORDINGS_ROOT``).

:func:`channel_table` is the other output: a long table of selected channels
on the header clock ``t`` (seconds from the first sample), used for the joint
inspection recordings (#110).

The original recordings are not distributed. Reading a bag needs the optional
``rosbags`` package.
"""

from __future__ import annotations

import io
from typing import Dict, List, Sequence, Tuple

VALUES_HEADER = (
    "rosbagTimestamp,header,seq,stamp,secs,nsecs,frame_id,values,names_version"
)
NAMES_TOPIC = "/introspection_data/names"
VALUES_TOPIC = "/introspection_data/values"


def read_introspection(path: str):
    """Return ``(names_msg, rows)`` of one bag.

    ``names_msg`` is ``(bag_ns, seq, secs, nsecs, version, names)``; ``rows`` is
    a list of ``(bag_ns, seq, secs, nsecs, version, values)`` in bag order.
    """
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore

    ts = get_typestore(Stores.ROS1_NOETIC)
    names_msg = None
    rows: List[tuple] = []
    with Reader(path) as r:
        for c in r.connections:
            if c.msgtype not in ts.types:
                ts.register(get_types_from_msg(c.msgdef.data, c.msgtype))
        conns = [c for c in r.connections if c.topic in (NAMES_TOPIC, VALUES_TOPIC)]
        for c, t, raw in r.messages(connections=conns):
            m = ts.deserialize_ros1(raw, c.msgtype)
            h = m.header
            meta = (int(t), int(h.seq), int(h.stamp.sec), int(h.stamp.nanosec))
            if c.topic == NAMES_TOPIC:
                names_msg = meta + (int(m.names_version), list(m.names))
            else:
                rows.append(meta + (int(m.names_version), [float(v) for v in m.values]))
    if names_msg is None:
        raise ValueError(f"{path}: no {NAMES_TOPIC} message")
    return names_msg, rows


#: Joints and signal kinds kept by :func:`trim` for the backlash sessions.
BACKLASH_JOINTS = ("torso_lift",) + tuple(f"arm_{i}" for i in range(1, 8))
BACKLASH_KINDS = ("position", "velocity", "effort", "absolute_encoder_position")


def trim(names_msg, rows: Sequence[tuple], wanted: Sequence[str]):
    """Keep the named channels, in recorded order; values are unchanged."""
    names = names_msg[5]
    keep = [i for i, n in enumerate(names) if n in set(wanted)]
    missing = sorted(set(wanted) - {names[i] for i in keep})
    if missing:
        raise KeyError(f"channels not recorded: {missing}")
    new_names = names_msg[:5] + ([names[i] for i in keep],)
    return new_names, [r[:5] + ([r[5][i] for i in keep],) for r in rows]


#: First channel of the recording. The names file carries it in the ``names``
#: cell, where the loader does not read it, so it must stay first: the loader
#: reads the header names and offsets every value index by one.
LEADING_CHANNEL = "topic_stats./introspection_data.publish_async_attempts"


def backlash_channels() -> List[str]:
    return [LEADING_CHANNEL] + [
        f"{j}_joint_{k}" for j in BACKLASH_JOINTS for k in BACKLASH_KINDS
    ]


def names_csv(names_msg) -> str:
    """The names file: header with ``- `` names, one row with the first name."""
    t, seq, secs, nsecs, version, names = names_msg
    head = ["rosbagTimestamp", "header", "seq", "stamp", "secs", "nsecs", "frame_id"]
    head += ["names"] + [f"- {n}" for n in names[1:]] + ["names_version"]
    row = [str(t), "", str(seq), "", str(secs), str(nsecs), "''", f"- {names[0]}"]
    row.append(str(version))
    return ",".join(head) + "\n" + ",".join(row) + "\n"


def values_csv(rows: Sequence[tuple]) -> str:
    """The values file, one row per message, list written with ``repr``."""
    out = io.StringIO()
    out.write(VALUES_HEADER + "\n")
    for t, seq, secs, nsecs, version, values in rows:
        lst = "[" + ", ".join(repr(v) for v in values) + "]"
        out.write(f"{t},,{seq},,{secs},{nsecs},'',\"{lst}\",{version}\n")
    return out.getvalue()


def channel_table(
    names_msg, rows: Sequence[tuple], channels: Sequence[Tuple[str, str]]
) -> str:
    """CSV of selected channels on one clock.

    ``channels`` is ``[(column, channel_name), ...]``. The first column is
    ``t`` in seconds from the first values message (header stamp, rounded to
    microseconds), followed by the channels. Values are ``repr`` of the float.
    """
    names = names_msg[5]
    index: Dict[str, int] = {n: i for i, n in enumerate(names)}
    missing = [ch for _, ch in channels if ch not in index]
    if missing:
        raise KeyError(f"channels not recorded: {missing}")
    cols = [index[ch] for _, ch in channels]
    t0 = rows[0][2] + rows[0][3] * 1e-9
    out = io.StringIO()
    out.write("t," + ",".join(c for c, _ in channels) + "\n")
    for t, seq, secs, nsecs, version, values in rows:
        out.write(repr(round(secs + nsecs * 1e-9 - t0, 6)) + ",")
        out.write(",".join(repr(values[i]) for i in cols) + "\n")
    return out.getvalue()


#: Columns of the joint inspection files (#110): arm_5 and arm_6, relative
#: (motor side) and absolute (output side) encoder and the commanded position.
INSPECTION_KINDS = (
    ("position", "position"),
    ("absolute_encoder_position", "absolute"),
    ("position_command", "command"),
)


def inspection_channels() -> List[Tuple[str, str]]:
    """``(column, channel)`` pairs for :func:`channel_table`."""
    return [
        (f"arm_{j}_{col}", f"arm_{j}_joint_{kind}")
        for j in (5, 6)
        for kind, col in INSPECTION_KINDS
    ]


def read_tf_csv(path: str):
    """Transforms of a ``bag_to_csv`` export of ``/tf``, one list per row.

    The exporter writes a variable-length row per message: the bag time, then
    17 fields per transform (the first block starts at field 3) with the
    ``-`` list markers dropped, so the header names are shifted and unusable
    (``AUDIT`` defect P3). Each transform is
    ``(secs, nsecs, parent, child, (x, y, z), (qx, qy, qz, qw))``. Rows are
    ``(bag_ns, [transform, ...])``.
    """
    import csv

    out = []
    with open(path, newline="") as f:
        for row in csv.reader(f):
            if not row or row[0] == "rosbagTimestamp":
                continue
            tfs = []
            k = 0
            while 3 + 17 * k + 15 < len(row):
                b = 3 + 17 * k
                tfs.append(
                    (
                        int(row[b + 2]),
                        int(row[b + 3]),
                        row[b + 4].strip('"'),
                        row[b + 5].strip('"'),
                        tuple(float(x) for x in row[b + 8 : b + 11]),
                        tuple(float(x) for x in row[b + 12 : b + 16]),
                    )
                )
                k += 1
            out.append((int(row[0]), tfs))
    return out


if __name__ == "__main__":
    import argparse
    from pathlib import Path

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="cmd", required=True)
    exp = sub.add_parser(
        "export", help="bag to names and values CSV (backlash layout, trimmed channels)"
    )
    exp.add_argument("bag")
    exp.add_argument("out_dir")
    insp = sub.add_parser(
        "inspection", help="bag to the arm_5/arm_6 encoder and command table"
    )
    insp.add_argument("bag")
    insp.add_argument("out_csv")
    args = parser.parse_args()
    names_msg, rows = read_introspection(args.bag)
    if args.cmd == "inspection":
        Path(args.out_csv).parent.mkdir(parents=True, exist_ok=True)
        with open(args.out_csv, "w", newline="") as f:
            f.write(channel_table(names_msg, rows, inspection_channels()))
        print(f"{len(rows)} rows")
        raise SystemExit(0)
    names_msg, rows = trim(names_msg, rows, backlash_channels())
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    (out / "introspection_datanames.csv").write_text(names_csv(names_msg))
    (out / "introspection_datavalues.csv").write_text(values_csv(rows))
    print(f"{len(rows)} values rows, {len(names_msg[5])} channels")
