"""Static-posture extraction for the TIAGo Vicon 2023-09-20 runs (#106).

These are the rules that produced the files in
``examples/tiago/data/calibration/vicon/``. The original recordings are not
distributed; the functions take plain arrays so the rules can be reviewed and
tested here. :func:`read_vicon_csv` needs only numpy, :func:`read_joint_bag`
the optional ``rosbags`` package and :func:`read_vicon_c3d` the optional
pure-Python ``c3d`` package (imported when called). The plateau rules and the
lag search are imported from :mod:`examples.tiago.utils.mocap_extraction`, so
they are identical to the Qualisys and OptiTrack files by construction.

Unlike the Qualisys and OptiTrack sessions, the Vicon files hold labelled
**markers**, not rigid-body poses: ``base1..3`` (base plate), ``shoulder1..4``
and ``gripper1..3`` (tool), in the Vicon world frame. A lost marker is
``0, 0, 0`` in the CSV and a negative residual in the c3d; both become NaN
here. The bodies of the shipped layout are the two marker triads:

* origin: centroid of the three markers; ``x`` along marker 2 minus marker 1;
  ``z`` along ``x`` cross (marker 3 minus marker 1); ``y = z cross x``;
* the ``tiago_base`` columns (``xb..phizb``) hold the ``base1..3`` triad and
  the gripper columns (``x1..phiz1``) the ``gripper1..3`` triad.

The relation of these frames to the TIAGo link frames is unknown: it is a free
registration parameter of the calibration. The shoulder markers are not used.

1. **Static plateaus** (:func:`static_plateaus`): every joint stays within
   1 mrad over 0.5 s, for at least 2 s.
2. **Clock lag** (:func:`measure_clock_lag`): the speed of the gripper triad
   centroid is correlated with the speed of the nominal-FK tool point
   (``wrist_ft_tool_link``), over +-60 s at 0.05 s, then 0.005 s around the
   best; ``t_mocap = t_robot - lag`` (the sign convention of
   ``mocap_extraction.estimate_clock_lag``). The CSV clock is the Vicon PC
   wall time written as local time without a zone, read as UTC+2 (CEST, as
   the small residual lag confirms), so its lag is a clock offset. The c3d
   has no absolute clock: its time axis starts at the first joint sample, so
   its lag is that origin offset plus the clock offset.
3. **Averaging** (:func:`extract_static_postures_6d`): joints over the
   plateau; the markers over ``[start + 0.5 s, end - 0.3 s]`` shifted by the
   lag, using the frames where all three markers of a triad are live. Marker
   positions are averaged, then the triad pose is built. A plateau is dropped
   when the gripper triad has fewer than 20 live frames in the window; a
   plateau without a live base triad is kept with NaN base columns.

Orientation columns are pinocchio roll-pitch-yaw (``R = Rz Ry Rx``), which is
scipy's extrinsic ``"xyz"``. Run

    python -m examples.tiago.utils.vicon_extraction --help

from the repository root; the byte-for-byte check against the shipped files is
``tests/test_tiago_vicon_extraction.py`` (needs ``TIAGO_RECORDINGS_ROOT``, and
the ``c3d`` package for the c3d run).
"""

from __future__ import annotations

import calendar
import datetime as dt
from typing import Dict, Optional, Sequence, Tuple

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

from examples.tiago.utils.mocap_extraction import (
    JOINTS,
    estimate_clock_lag,
    file_sha256,
    static_plateaus,
    tool_positions,
)
from examples.tiago.utils.optitrack_extraction import COLUMNS, _INT_COLUMNS

MARKERS = [f"base{i}" for i in (1, 2, 3)] + [f"shoulder{i}" for i in (1, 2, 3, 4)]
MARKERS += [f"gripper{i}" for i in (1, 2, 3)]
BASE_TRIAD = (0, 1, 2)
GRIPPER_TRIAD = (7, 8, 9)
#: UTC offset of the CSV wall clock (CEST on 2023-09-20).
CSV_UTC_OFFSET_S = 7200.0
TOOL_FRAME = "wrist_ft_tool_link"


def read_vicon_csv(path: str) -> Tuple[np.ndarray, np.ndarray]:
    """Vicon CSV export to ``(t, markers)``.

    ``t`` (n,) is the row wall-clock stamp as seconds since the Unix epoch
    (read as UTC+2); ``markers`` (n, 10, 3) is in metres in :data:`MARKERS`
    order, NaN where the marker is lost (exported as 0, 0, 0). The header row
    names one column per marker but each marker has x, y, z columns, and the
    force-plate columns follow the 30 marker columns; they are not read.
    """
    stamps, rows = [], []
    with open(path) as f:
        next(f)
        for line in f:
            cells = line.rstrip("\n").split(",")
            if len(cells) < 31:
                continue
            fmt = "%Y-%m-%d_%H:%M:%S.%f" if "." in cells[0] else "%Y-%m-%d_%H:%M:%S"
            d = dt.datetime.strptime(cells[0], fmt)
            stamps.append(calendar.timegm(d.timetuple()) + d.microsecond * 1e-6)
            rows.append([float(c) if c else np.nan for c in cells[1:31]])
    markers = np.array(rows).reshape(-1, 10, 3)
    markers[np.all(markers == 0, axis=2)] = np.nan
    return np.array(stamps) - CSV_UTC_OFFSET_S, markers


def read_vicon_c3d(path: str) -> Tuple[np.ndarray, np.ndarray]:
    """Vicon c3d to ``(t, markers)``; needs the optional ``c3d`` package.

    ``t`` (n,) is ``frame / rate`` from 0 (the file has no absolute clock);
    ``markers`` (n, 10, 3) in metres in :data:`MARKERS` order (matched by
    label), NaN where the residual is negative (lost).
    """
    import c3d

    with open(path, "rb") as f:
        reader = c3d.Reader(f)
        labels = [str(label).strip() for label in reader.point_labels]
        order = [labels.index(m) for m in MARKERS]
        unit = str(reader.get("POINT:UNITS").string_value).strip()
        frames = [pts[order] for _, pts, _ in reader.read_frames()]
        rate = float(reader.header.frame_rate)
    pts = np.array(frames)
    markers = pts[:, :, :3] * {"mm": 1e-3, "m": 1.0}[unit]
    markers[pts[:, :, 3] < 0] = np.nan
    return np.arange(len(pts)) / rate, markers


def read_joint_bag(path: str) -> Dict[str, np.ndarray]:
    """Joints from a PAL introspection bag; needs the optional ``rosbags``.

    Returns ``{"t": (n,), "q": (n, 8)}``: header stamps and ``<joint>_position``
    in :data:`JOINTS` order, sorted by stamp.
    """
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore

    ts = get_typestore(Stores.ROS1_NOETIC)
    names: Dict[int, list] = {}
    values = []
    with Reader(path) as reader:
        for c in reader.connections:
            if c.msgtype not in ts.types:
                ts.register(get_types_from_msg(c.msgdef.data, c.msgtype))
        for c, _, raw in reader.messages():
            if c.msgtype.endswith("StatisticsNames"):
                m = ts.deserialize_ros1(raw, c.msgtype)
                names[m.names_version] = list(m.names)
            elif c.msgtype.endswith("StatisticsValues"):
                m = ts.deserialize_ros1(raw, c.msgtype)
                stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                values.append((stamp, m.names_version, np.asarray(m.values)))
    rows = []
    for stamp, version, vals in values:
        nm = names.get(version)
        if nm is None or len(nm) != len(vals):
            continue
        rows.append([stamp] + [vals[nm.index(f"{j}_position")] for j in JOINTS])
    joints = np.array(sorted(rows))
    return {"t": joints[:, 0], "q": joints[:, 1:]}


def triad_frames(markers: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Frames of marker triads: ``(R (..., 3, 3), p (..., 3))``.

    Args:
        markers: (..., 3, 3) positions of markers 1, 2, 3 (axes in the module
            docstring).
    """
    m1, m2, m3 = markers[..., 0, :], markers[..., 1, :], markers[..., 2, :]
    x = m2 - m1
    x = x / np.linalg.norm(x, axis=-1, keepdims=True)
    z = np.cross(x, m3 - m1)
    z = z / np.linalg.norm(z, axis=-1, keepdims=True)
    y = np.cross(z, x)
    return np.stack([x, y, z], axis=-1), markers.mean(axis=-2)


def triad_sides_mm(markers: np.ndarray, triad: Sequence[int]) -> np.ndarray:
    """Median side lengths ``|m1m2|, |m1m3|, |m2m3|`` (mm) over live frames."""
    m = markers[_live(markers, triad)][:, list(triad)]
    return 1000.0 * np.array(
        [
            np.median(np.linalg.norm(m[:, a] - m[:, b], axis=1))
            for a, b in ((0, 1), (0, 2), (1, 2))
        ]
    )


def _live(markers: np.ndarray, triad: Sequence[int]) -> np.ndarray:
    return ~np.isnan(markers[:, list(triad)]).any(axis=(1, 2))


def measure_clock_lag(
    t_robot: np.ndarray,
    q: np.ndarray,
    t_mocap: np.ndarray,
    markers: np.ndarray,
    urdf: str,
) -> Tuple[float, float]:
    """``(t_robot - t_mocap, correlation)`` from the gripper triad.

    Nominal FK of :data:`TOOL_FRAME` against the live gripper centroid speed.
    """
    import pinocchio as pin

    tool = tool_positions(pin.buildModelFromUrdf(urdf), TOOL_FRAME, q)
    live = _live(markers, GRIPPER_TRIAD)
    centroid = markers[live][:, list(GRIPPER_TRIAD)].mean(axis=1)
    args = (t_robot, tool, t_mocap[live], centroid)
    coarse, _ = estimate_clock_lag(*args, search=(-60.0, 60.0), step=0.05)
    return estimate_clock_lag(*args, search=(coarse - 0.05, coarse + 0.05), step=0.005)


def _rpy(R: np.ndarray) -> np.ndarray:
    return Rotation.from_matrix(R).as_euler("xyz")


def _triad_pose(window: np.ndarray, triad: Sequence[int]):
    """Mean triad pose over the live frames of a (k, 10, 3) window.

    Returns ``(R, p, n_live, std_mm, std_deg)``; no pose when ``n_live == 0``.
    """
    m = window[_live(window, triad)][:, list(triad)]
    if len(m) == 0:
        return None, None, 0, np.nan, np.nan
    R, p = triad_frames(m.mean(axis=0))
    R_each, _ = triad_frames(m)
    rel = (Rotation.from_matrix(R).inv() * Rotation.from_matrix(R_each)).magnitude()
    std_mm = m.std(axis=0).max() * 1000
    return R, p, len(m), std_mm, np.degrees(np.sqrt(np.mean(rel**2)))


def extract_static_postures_6d(
    t: np.ndarray,
    q: np.ndarray,
    t_mocap: np.ndarray,
    markers: np.ndarray,
    lag: float,
    rigid_body_def: int = 1,
    min_samples: int = 20,
) -> Tuple[pd.DataFrame, Dict[str, int]]:
    """One row per static plateau, in the OptiTrack CSV layout (world frame).

    Args:
        t: (n,) robot timestamps.
        q: (n, 8) joint positions, in :data:`JOINTS` order.
        t_mocap: (m,) mocap timestamps.
        markers: (m, 10, 3) marker positions in metres, NaN where lost.
        lag: ``t_robot - t_mocap`` (:func:`measure_clock_lag`).
        rigid_body_def: marker-triad definition id written to the file.

    Returns:
        ``(table, drops)``; ``drops`` has ``plateaus`` and ``gripper_lost``.
        Rows without a base pose are the NaN rows of the base columns.
    """
    plateaus = static_plateaus(t, q)
    drops = {"plateaus": len(plateaus), "gripper_lost": 0}
    rows = []
    for a, b in plateaus:
        t0, t1 = t[a] + 0.5, t[b - 1] - 0.3
        sel = (t_mocap >= t0 - lag) & (t_mocap <= t1 - lag)
        window = markers[sel]
        R_g, p_g, n_live, std_mm, std_deg = _triad_pose(window, GRIPPER_TRIAD)
        if n_live < min_samples:
            drops["gripper_lost"] += 1
            continue
        R_b, p_b, n_base, _, _ = _triad_pose(window, BASE_TRIAD)
        if n_base >= min_samples:
            base_pose = np.r_[p_b, _rpy(R_b)]
        else:
            base_pose = np.full(6, np.nan)
        rows.append(
            [
                *p_g,
                *_rpy(R_g),
                *q[a:b].mean(axis=0),
                *base_pose,
                t0,
                t1,
                int(sel.sum()),
                n_live,
                n_base,
                std_mm,
                std_deg,
                rigid_body_def,
                0,
            ]
        )
    table = pd.DataFrame(rows, columns=COLUMNS)
    return table.astype({c: int for c in _INT_COLUMNS}), drops


def extract_session(
    mocap: str,
    bag: str,
    urdf: str,
    kind: Optional[str] = None,
    lag: Optional[float] = None,
    rigid_body_def: int = 1,
    expected_sha256: Optional[Dict[str, str]] = None,
) -> Tuple[pd.DataFrame, float, Dict[str, int]]:
    """Vicon file plus joint bag to posture table; measures the lag if not given.

    ``kind`` is ``"csv"`` or ``"c3d"`` (the file suffix by default).
    ``expected_sha256`` maps ``"mocap"`` and ``"bag"`` to the recorded hashes
    and refuses a file that does not match. The c3d time axis starts at the
    first joint sample.
    """
    kind = kind or mocap.rsplit(".", 1)[-1].lower()
    if kind not in ("csv", "c3d"):
        raise ValueError(f"kind must be 'csv' or 'c3d', got {kind!r}")
    for key, path in (("mocap", mocap), ("bag", bag)):
        if expected_sha256 and key in expected_sha256:
            actual = file_sha256(path)
            if actual != expected_sha256[key]:
                raise ValueError(f"{path}: source sha256 mismatch ({actual})")
    joints = read_joint_bag(bag)
    if kind == "csv":
        t_mocap, markers = read_vicon_csv(mocap)
    else:
        t_mocap, markers = read_vicon_c3d(mocap)
        t_mocap = t_mocap + joints["t"][0]
    if lag is None:
        lag, _ = measure_clock_lag(joints["t"], joints["q"], t_mocap, markers, urdf)
    table, drops = extract_static_postures_6d(
        joints["t"], joints["q"], t_mocap, markers, lag, rigid_body_def
    )
    return table, lag, drops


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("mocap", help="Vicon .csv or .c3d export")
    parser.add_argument("bag", help="PAL introspection bag of the same run")
    parser.add_argument("out")
    parser.add_argument(
        "--urdf",
        default="examples/tiago/urdf/tiago_48_schunk.urdf",
        help="URDF for the nominal FK used by the lag search",
    )
    parser.add_argument(
        "--rigid-body-def",
        type=int,
        required=True,
        help="marker-triad definition id (runs with other sides differ)",
    )
    parser.add_argument(
        "--lag", type=float, help="override the measured t_robot - t_mocap (s)"
    )
    parser.add_argument("--mocap-sha256", help="Vicon file hash from protocol.yaml")
    parser.add_argument("--bag-sha256", help="Joint bag hash from protocol.yaml")
    args = parser.parse_args()
    expected = {
        k: v for k, v in (("mocap", args.mocap_sha256), ("bag", args.bag_sha256)) if v
    }
    table, lag, drops = extract_session(
        args.mocap,
        args.bag,
        args.urdf,
        lag=args.lag,
        rigid_body_def=args.rigid_body_def,
        expected_sha256=expected,
    )
    table.to_csv(args.out, index=False)
    print(
        f"{len(table)} postures of {drops['plateaus']} plateaus "
        f"(gripper lost {drops['gripper_lost']}), lag {lag:+.4f} s -> {args.out}"
    )
