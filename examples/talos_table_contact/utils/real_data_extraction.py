"""Rebuild the shipped TALOS table-contact postures and their contact evidence.

Run with ``python -m examples.talos_table_contact.utils.real_data_extraction
--help`` from the repository root. The original recordings are not
distributed; this module documents and reproduces how the files under
``data/real/`` were made from them, and checks the result byte for byte.
Rebuilding the posture CSVs needs only the per-posture measurement CSVs.
The contact evidence also reads the PAL statistics bags and needs the
optional ``rosbags`` package (no ROS installation).

``data/real/protocol.yaml`` names every source by sha256 and lists, for each
shipped posture, the source file and line it was taken from.
"""

from __future__ import annotations

import argparse
import hashlib
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

DATA_DIR = Path(__file__).resolve().parents[1] / "data" / "real"
PROTOCOL = DATA_DIR / "protocol.yaml"
EVIDENCE_FILE = "contact_evidence.csv"

#: Column order of the 32 joint values in every posture row.
JOINTS = (
    tuple(f"arm_left_{i}_joint" for i in range(1, 8))
    + tuple(f"arm_right_{i}_joint" for i in range(1, 8))
    + ("gripper_left_joint", "gripper_right_joint", "head_1_joint", "head_2_joint")
    + tuple(f"leg_left_{i}_joint" for i in range(1, 7))
    + tuple(f"leg_right_{i}_joint" for i in range(1, 7))
    + ("torso_1_joint", "torso_2_joint")
)
#: Metadata fields before the joints: gripper, gripper frame, "handle",
#: contact marker, "joint_states".
N_METADATA = 5
#: Protocol v2 header: four empty placeholder cells and ``joint_names`` over
#: the five metadata fields, then the joint names (37 fields, like the rows).
HEADER = ",,,," + ",".join(("joint_names",) + JOINTS)

#: Contact rule, fixed before the postures were screened (examples#113).
MAX_JOINT_DIFF_RAD = 3e-4
CONTACT_FZ_N = 10.0
EVIDENCE_COLUMNS = (
    "posture_file,row,contact_marker,status,bag,stamp_s,max_joint_diff_rad,"
    "force_x_N,force_y_N,force_z_N,torque_x_Nm,torque_y_Nm,torque_z_Nm,contact"
)
WRENCH = tuple(
    f"{{side}}_wrist_ft_{quantity}_{axis}"
    for quantity in ("force", "torque")
    for axis in "XYZ"
)


def sha256_of(path: str | Path) -> str:
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def load_protocol(path: str | Path = PROTOCOL) -> dict:
    return yaml.safe_load(Path(path).read_text())


def _lines(path: Path) -> list[bytes]:
    """Source lines without their terminators (the sources mix CRLF and LF)."""
    return path.read_bytes().splitlines()


def extract_posture_files(
    source_root: str | Path, protocol: dict, *, verify: bool = True
) -> dict[str, bytes]:
    """Return ``{shipped file name: bytes}`` assembled from measurement files.

    Each shipped row is the named line of the named measurement file, written
    with an LF terminator under the protocol header. With ``verify`` the
    source files must match their sha256 in the protocol.
    """
    source_root = Path(source_root)
    cache: dict[str, list[bytes]] = {}
    out = {}
    for session in protocol["sessions"]:
        rows = [HEADER.encode()]
        for ref in session["postures"]:
            name, line = ref.rsplit(":", 1)
            if name not in cache:
                path = source_root / name
                expected = protocol["measurement_files"][name]
                if verify and sha256_of(path) != expected:
                    raise ValueError(f"{path}: source sha256 mismatch")
                cache[name] = _lines(path)
            rows.append(cache[name][int(line) - 1])
        out[session["file"]] = b"\n".join(rows) + b"\n"
    return out


@dataclass(frozen=True)
class Posture:
    """One shipped posture: where it is, its marker and its 32 joint values."""

    file: str
    row: int  # line number in the shipped file (header is line 1)
    marker: str
    q: np.ndarray


def read_postures(files: dict[str, bytes]) -> list[Posture]:
    postures = []
    for name, content in files.items():
        for row, line in enumerate(content.decode().splitlines()[1:], start=2):
            fields = line.split(",")
            if len(fields) != N_METADATA + len(JOINTS):
                raise ValueError(f"{name}:{row}: {len(fields)} fields")
            q = np.array(fields[N_METADATA:], dtype=float)
            postures.append(Posture(name, row, fields[3], q))
    return postures


@dataclass(frozen=True)
class Samples:
    """Decodable introspection samples of one bag, on the ROS header clock."""

    bag: str
    stamps: list[str]  # "seconds.nanoseconds", exact
    q: np.ndarray  # (n, 32) joint positions, rad
    wrench: dict[str, np.ndarray]  # side -> (n, 6): force N, torque N m


def read_bag_names(path: str | Path) -> dict[int, list[str]]:
    """Name lists of a bag, keyed by ``names_version``."""
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore

    store = get_typestore(Stores.ROS1_NOETIC)
    names: dict[int, list[str]] = {}
    with Reader(Path(path)) as reader:
        connections = [
            c for c in reader.connections if c.topic.endswith("names_throttle")
        ]
        for connection in connections:
            if connection.msgtype not in store.types:
                store.register(
                    get_types_from_msg(connection.msgdef.data, connection.msgtype)
                )
        for connection, _, raw in reader.messages(connections=connections):
            message = store.deserialize_ros1(raw, connection.msgtype)
            names[message.names_version] = list(message.names)
    return names


def shared_layouts(name_lists: list[dict[int, list[str]]]) -> dict[int, list[str]]:
    """Joint-carrying name lists keyed by length, if each length has one list.

    Raises if two bags disagree on the channels of a given length, so a
    layout is reused across bags only when it never varies.
    """
    layouts: dict[int, list[str]] = {}
    for names in name_lists:
        for channels in names.values():
            if not all(f"{j}_position" in channels for j in JOINTS):
                continue
            known = layouts.setdefault(len(channels), channels)
            if known != channels:
                raise ValueError(f"{len(channels)}-channel name lists differ")
    return layouts


def read_bag_samples(
    path: str | Path, name: str, layouts: dict[int, list[str]] | None = None
) -> Samples:
    """Joint positions and wrist wrenches of every decodable message.

    A message uses the bag's own name list for its ``names_version``. If the
    bag has none of the right length, ``layouts`` (see ``shared_layouts``)
    supplies the list for that message length; pass it only for bags whose
    protocol status is ``recovered``. Statistics-only messages (4 channels)
    are skipped.
    """
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore

    store = get_typestore(Stores.ROS1_NOETIC)
    names: dict[int, list[str]] = {}
    records = []
    with Reader(Path(path)) as reader:
        connections = list(reader.connections)
        for connection in connections:
            if connection.msgtype not in store.types:
                store.register(
                    get_types_from_msg(connection.msgdef.data, connection.msgtype)
                )
        for connection, _, raw in reader.messages(connections=connections):
            message = store.deserialize_ros1(raw, connection.msgtype)
            if connection.topic.endswith("names_throttle"):
                names[message.names_version] = list(message.names)
            else:
                stamp = message.header.stamp
                records.append(
                    (
                        f"{stamp.sec}.{stamp.nanosec:09d}",
                        message.names_version,
                        np.asarray(message.values),
                    )
                )
    wanted = {"left": [], "right": []}
    stamps, q, wrench = [], [], {"left": [], "right": []}
    index: dict[int, tuple | None] = {}
    for stamp, version, values in records:
        if version not in index:
            channels = names.get(version, [])
            if len(channels) != len(values) and layouts:
                channels = layouts.get(len(values), channels)
            ok = len(channels) == len(values) and all(
                f"{j}_position" in channels for j in JOINTS
            )
            index[version] = (
                (
                    [channels.index(f"{j}_position") for j in JOINTS],
                    {
                        side: [channels.index(c.format(side=side)) for c in WRENCH]
                        for side in wanted
                    },
                )
                if ok
                else None
            )
        if index[version] is None:
            continue
        joints, wrenches = index[version]
        stamps.append(stamp)
        q.append(values[joints])
        for side in wanted:
            wrench[side].append(values[wrenches[side]])
    return Samples(
        name,
        stamps,
        np.array(q).reshape(-1, len(JOINTS)),
        {side: np.array(w).reshape(-1, 6) for side, w in wrench.items()},
    )


def evidence_rows(postures: list[Posture], bags: list[Samples]) -> list[str]:
    """Join each posture to its nearest sample; apply the fixed contact rule."""
    rows = []
    for posture in postures:
        best = None  # (diff, bag, index); first bag/sample wins ties
        for bag in bags:
            if not len(bag.stamps):
                continue
            diff = np.abs(bag.q - posture.q).max(axis=1)
            k = int(np.argmin(diff))
            if best is None or diff[k] < best[0]:
                best = (float(diff[k]), bag, k)
        head = f"{posture.file},{posture.row},{posture.marker}"
        if best is None or best[0] > MAX_JOINT_DIFF_RAD:
            rows.append(f"{head},no_matching_sample,,,,,,,,,,")
            continue
        diff, bag, k = best
        side = posture.file.split("_")[0]
        w = bag.wrench[side][k]
        flag = int(w[2] >= CONTACT_FZ_N)
        values = ",".join(repr(float(v)) for v in w)
        rows.append(
            f"{head},matched,{bag.bag},{bag.stamps[k]},{diff!r},{values},{flag}"
        )
    return rows


def extract_contact_evidence(
    source_root: str | Path, protocol: dict, files: dict[str, bytes]
) -> bytes:
    """Contact evidence CSV for the postures in ``files``."""
    source_root = Path(source_root)
    used = {
        name: info
        for name, info in protocol["bags"].items()
        if info["status"] in ("searched", "recovered")
    }
    for name, info in used.items():
        if sha256_of(source_root / name) != info["sha256"]:
            raise ValueError(f"{source_root / name}: source sha256 mismatch")
    layouts = shared_layouts(
        [
            read_bag_names(source_root / name)
            for name, info in used.items()
            if info["status"] == "searched"
        ]
    )
    bags = [
        read_bag_samples(
            source_root / name,
            name,
            layouts if info["status"] == "recovered" else None,
        )
        for name, info in used.items()
    ]
    rows = evidence_rows(read_postures(files), bags)
    return ("\n".join([EVIDENCE_COLUMNS] + rows) + "\n").encode()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", required=True, type=Path)
    parser.add_argument("--output-dir", type=Path, help="write the files here")
    parser.add_argument(
        "--check", action="store_true", help="compare with the shipped files"
    )
    parser.add_argument("--no-contact", action="store_true", help="skip the bags")
    args = parser.parse_args()
    protocol = load_protocol()
    built = extract_posture_files(args.source_root, protocol)
    if not args.no_contact:
        built[EVIDENCE_FILE] = extract_contact_evidence(
            args.source_root, protocol, built
        )
    for name, content in built.items():
        if args.output_dir:
            args.output_dir.mkdir(parents=True, exist_ok=True)
            (args.output_dir / name).write_bytes(content)
        if args.check:
            same = (DATA_DIR / name).read_bytes() == content
            print(f"{name}: {'identical' if same else 'DIFFERENT'}")
            if not same:
                raise SystemExit(1)


if __name__ == "__main__":
    main()
