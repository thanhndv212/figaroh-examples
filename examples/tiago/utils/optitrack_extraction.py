"""Static-posture extraction for the TIAGo OptiTrack 2023-09-12 runs (#105).

These are the rules that produced the files in
``examples/tiago/data/calibration/optitrack/``. The original ROS bags are not
distributed; the functions take plain arrays so the rules can be reviewed and
tested here, and :func:`read_optitrack_bag` reads a bag when the optional
``rosbags`` package is installed. The plateau rules are imported from
:mod:`examples.tiago.utils.mocap_extraction`, so they are identical to the
Qualisys files by construction.

1. **Static plateaus** (:func:`static_plateaus`): every joint stays within
   1 mrad over 0.5 s, for at least 2 s.
2. **Frozen samples** (:func:`drop_frozen`): when a rigid body is lost, NatNet
   republishes the last pose with a new stamp. A sample whose 7 pose values
   equal the previous sample's bit for bit is not a measurement and is
   removed before windowing.
3. **Clock lag** (:func:`measure_clock_lag`): the NatNet header stamps run
   0.8 to 1.4 s ahead of the recorder and the offset differs per run. The
   lag is the difference of the median receive-minus-header offsets of the
   mocap and robot streams, ``t_mocap = t_robot - lag`` (the sign convention
   of ``mocap_extraction.estimate_clock_lag``).
4. **Averaging** (:func:`extract_static_postures_6d`): joints over the
   plateau; the body poses over ``[start + 0.5 s, end - 0.3 s]`` shifted by
   the lag. Positions are averaged, rotations with ``Rotation.mean``. A
   plateau is dropped when the gripper body has fewer than 20 live samples
   in the window.

Frames. ``frame="world"`` (shipped) writes the ``tiago_schunk`` pose in the
OptiTrack world frame and flags ``points_in_base_frame = 0``; the
``tiago_base`` world pose goes in extra columns (NaN where the base body has
fewer than 20 live samples), so tracked rows can be expressed in the base
frame later and the base pose stays a free registration parameter.
``frame="base"`` expresses the gripper in the ``tiago_base`` body, flags 1 and
drops plateaus where the base body is not tracked.

Orientation columns are pinocchio roll-pitch-yaw (``R = Rz Ry Rx``), which is
scipy's extrinsic ``"xyz"``. Run

    python -m examples.tiago.utils.optitrack_extraction --help

from the repository root; the byte-for-byte check against the shipped files is
``tests/test_tiago_optitrack_extraction.py`` (needs ``TIAGO_RECORDINGS_ROOT``).
"""

from __future__ import annotations

from typing import Dict, Optional, Tuple

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

from examples.tiago.utils.mocap_extraction import JOINTS, file_sha256, static_plateaus

BASE_BODY = "tiago_base"
GRIPPER_BODY = "tiago_schunk"
POSE = ["x", "y", "z", "phix", "phiy", "phiz"]
COLUMNS = (
    [f"{c}1" for c in POSE]
    + JOINTS
    + [f"{c}b" for c in POSE]
    + [
        "t_start_robot",
        "t_end_robot",
        "n_window",
        "n_live",
        "n_base_live",
        "pose_std_mm",
        "pose_std_deg",
        "rigid_body_def",
        "points_in_base_frame",
    ]
)
_INT_COLUMNS = [
    "n_window",
    "n_live",
    "n_base_live",
    "rigid_body_def",
    "points_in_base_frame",
]


def read_optitrack_bag(path: str) -> Dict[str, np.ndarray]:
    """Read joints and OptiTrack body poses from an original run bag.

    Needs the optional ``rosbags`` package. Returns ``{"t": (n,), "t_recv":
    (n,), "q": (n, 8), "tiago_base": (m, 9), "tiago_schunk": (k, 9)}``.
    Joints come from PAL introspection (``<joint>_position``), sorted by
    header stamp; bodies from ``/natnet_ros/<body>/pose`` in bag order with
    columns ``t_header, t_recv, x, y, z, qx, qy, qz, qw``.
    """
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore

    ts = get_typestore(Stores.ROS1_NOETIC)
    topics = {f"/natnet_ros/{b}/pose": b for b in (BASE_BODY, GRIPPER_BODY)}
    names: Dict[int, list] = {}
    values = []
    bodies: Dict[str, list] = {b: [] for b in topics.values()}
    with Reader(path) as reader:
        for c in reader.connections:
            if c.msgtype not in ts.types:
                ts.register(get_types_from_msg(c.msgdef.data, c.msgtype))
        for c, recv, raw in reader.messages():
            if c.topic in topics:
                m = ts.deserialize_ros1(raw, c.msgtype)
                p, o = m.pose.position, m.pose.orientation
                stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                bodies[topics[c.topic]].append(
                    (stamp, recv * 1e-9, p.x, p.y, p.z, o.x, o.y, o.z, o.w)
                )
            elif c.msgtype.endswith("StatisticsNames"):
                m = ts.deserialize_ros1(raw, c.msgtype)
                names[m.names_version] = list(m.names)
            elif c.msgtype.endswith("StatisticsValues"):
                m = ts.deserialize_ros1(raw, c.msgtype)
                stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                values.append(
                    (stamp, recv * 1e-9, m.names_version, np.asarray(m.values))
                )
    rows = []
    for stamp, recv, version, vals in values:
        nm = names.get(version)
        if nm is None or len(nm) != len(vals):
            continue
        rows.append([stamp, recv] + [vals[nm.index(f"{j}_position")] for j in JOINTS])
    joints = np.array(sorted(rows))
    out = {"t": joints[:, 0], "t_recv": joints[:, 1], "q": joints[:, 2:]}
    out.update({b: np.array(s) for b, s in bodies.items()})
    return out


def drop_frozen(stream: np.ndarray) -> np.ndarray:
    """Remove samples whose 7 pose values (columns 2:9) repeat the previous one.

    The first sample is kept. Close but unequal values are measurements.
    """
    if len(stream) == 0:
        return stream
    same = np.all(stream[1:, 2:9] == stream[:-1, 2:9], axis=1)
    return stream[np.r_[True, ~same]]


def stamp_offset(t_header: np.ndarray, t_recv: np.ndarray) -> float:
    """Median of ``t_recv - t_header`` (s)."""
    return float(np.median(np.asarray(t_recv) - np.asarray(t_header)))


def measure_clock_lag(
    robot: Tuple[np.ndarray, np.ndarray], mocap: Tuple[np.ndarray, np.ndarray]
) -> float:
    """``t_robot - t_mocap`` from ``(t_header, t_recv)`` pairs of both streams."""
    return stamp_offset(*mocap) - stamp_offset(*robot)


def mean_pose(stream: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Mean rotation matrix (3, 3) and mean position (3,) of body samples."""
    return Rotation.from_quat(stream[:, 5:9]).mean().as_matrix(), stream[:, 2:5].mean(
        axis=0
    )


def _rpy(R: np.ndarray) -> np.ndarray:
    return Rotation.from_matrix(R).as_euler("xyz")


def _window(stream: np.ndarray, w0: float, w1: float) -> np.ndarray:
    return stream[(stream[:, 0] >= w0) & (stream[:, 0] <= w1)]


def extract_static_postures_6d(
    t: np.ndarray,
    q: np.ndarray,
    base: np.ndarray,
    gripper: np.ndarray,
    lag: float,
    rigid_body_def: int,
    frame: str = "world",
    min_samples: int = 20,
) -> Tuple[pd.DataFrame, Dict[str, int]]:
    """One row per static plateau, in the OptiTrack CSV layout.

    Args:
        t: (n,) robot timestamps.
        q: (n, 8) joint positions, in :data:`JOINTS` order.
        base, gripper: raw body streams ``t_header, t_recv, x, y, z, qx, qy,
            qz, qw`` (frozen samples are removed here).
        lag: ``t_robot - t_mocap`` (:func:`measure_clock_lag`).
        rigid_body_def: Gripper rigid-body definition id written to the file.
        frame: ``"world"`` or ``"base"`` (see the module docstring).

    Returns:
        ``(table, drops)``; ``drops`` has ``plateaus`` and ``gripper_lost``,
        plus ``base_lost`` for ``frame="base"``. In world frame the rows
        without a base pose are the NaN rows of the base columns.
    """
    if frame not in ("base", "world"):
        raise ValueError(f"frame must be 'base' or 'world', got {frame!r}")
    raw_gripper = gripper
    base, gripper = drop_frozen(base), drop_frozen(gripper)
    plateaus = static_plateaus(t, q)
    drops = {"plateaus": len(plateaus), "gripper_lost": 0}
    if frame == "base":
        drops["base_lost"] = 0
    rows = []
    for a, b in plateaus:
        t0, t1 = t[a] + 0.5, t[b - 1] - 0.3
        w0, w1 = t0 - lag, t1 - lag
        g = _window(gripper, w0, w1)
        bs = _window(base, w0, w1)
        n_window = len(_window(raw_gripper, w0, w1))
        base_ok = len(bs) >= min_samples
        if len(g) < min_samples:
            drops["gripper_lost"] += 1
            continue
        if frame == "base" and not base_ok:
            drops["base_lost"] += 1
            continue
        R_g, p_g = mean_pose(g)
        if base_ok:
            R_b, p_b = mean_pose(bs)
            base_pose = np.r_[p_b, _rpy(R_b)]
        else:
            R_b, p_b, base_pose = np.eye(3), np.zeros(3), np.full(6, np.nan)
        if frame == "base":
            pose = np.r_[R_b.T @ (p_g - p_b), _rpy(R_b.T @ R_g)]
        else:
            pose = np.r_[p_g, _rpy(R_g)]
        std_mm = g[:, 2:5].std(axis=0).max() * 1000
        rel = (
            Rotation.from_matrix(R_g).inv() * Rotation.from_quat(g[:, 5:9])
        ).magnitude()
        std_deg = np.degrees(np.sqrt(np.mean(rel**2)))
        rows.append(
            [
                *pose,
                *q[a:b].mean(axis=0),
                *base_pose,
                t0,
                t1,
                n_window,
                len(g),
                len(bs),
                std_mm,
                std_deg,
                rigid_body_def,
                int(frame == "base"),
            ]
        )
    table = pd.DataFrame(rows, columns=COLUMNS)
    return table.astype({c: int for c in _INT_COLUMNS}), drops


def extract_session(
    bag: str,
    rigid_body_def: int,
    lag: Optional[float] = None,
    frame: str = "world",
    expected_sha256: Optional[str] = None,
) -> Tuple[pd.DataFrame, float, Dict[str, int]]:
    """Bag to posture table; measures the lag unless it is given.

    ``expected_sha256`` refuses a bag that is not the recorded one.
    """
    if expected_sha256 is not None:
        actual = file_sha256(bag)
        if actual != expected_sha256:
            raise ValueError(f"{bag}: source sha256 mismatch ({actual})")
    s = read_optitrack_bag(bag)
    if lag is None:
        live = drop_frozen(s[GRIPPER_BODY])
        lag = measure_clock_lag((s["t"], s["t_recv"]), (live[:, 0], live[:, 1]))
    table, drops = extract_static_postures_6d(
        s["t"], s["q"], s[BASE_BODY], s[GRIPPER_BODY], lag, rigid_body_def, frame
    )
    return table, lag, drops


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("bag")
    parser.add_argument("out")
    parser.add_argument(
        "--rigid-body-def", type=int, required=True, help="gripper body definition id"
    )
    parser.add_argument(
        "--frame",
        choices=("base", "world"),
        default="world",
        help="gripper pose in the OptiTrack world frame (default, with the base "
        "pose in extra columns) or in the tiago_base body",
    )
    parser.add_argument(
        "--lag", type=float, help="override the measured t_robot - t_mocap (s)"
    )
    parser.add_argument("--expected-sha256", help="Source bag hash from protocol.yaml")
    args = parser.parse_args()
    table, lag, drops = extract_session(
        args.bag,
        args.rigid_body_def,
        lag=args.lag,
        frame=args.frame,
        expected_sha256=args.expected_sha256,
    )
    table.to_csv(args.out, index=False)
    lost = ", ".join(
        f"{k.replace('_', ' ')} {v}" for k, v in drops.items() if k != "plateaus"
    )
    print(
        f"{len(table)} postures of {drops['plateaus']} plateaus ({lost}), "
        f"lag {lag:+.4f} s -> {args.out}"
    )
