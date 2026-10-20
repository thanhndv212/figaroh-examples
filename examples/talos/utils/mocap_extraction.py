"""Static-posture extraction for the TALOS torso-arm motion-capture sessions (#114).

These are the rules that produced the files in
``examples/talos/data/calibration/mocap/``. The original ROS bags are not
distributed; the functions take plain arrays so the rules can be reviewed and
tested here, and :func:`read_bag` reads a bag when the optional ``rosbags``
package is installed (no ROS install).

A bag holds two topics (no other streams were recorded):

* ``/tf_throttle``: every message is either a Qualisys pose (parent frame
  ``Qualisys``, children ``waist_frame``, ``upper_body_frame``,
  ``endeffector_frame``) or the robot's own link transforms (not used here);
* ``/introspection_data/values_throttle``: the PAL statistics, from which the
  joint positions ``<joint>_joint_position`` are taken (names in
  ``/introspection_data/names_throttle``).

Header stamps are used for both (the CSV exports of the same bags lost digits
of the bag time, so they are not a source).

1. **Static plateaus** (:func:`static_plateaus`): the nine joints (seven arm
   joints of the recorded side and ``torso_1``, ``torso_2``) stay within
   ``tol`` (1 mrad) over ``window`` (0.5 s), for at least ``min_duration``
   (2 s).
2. **Clock offset** (:func:`estimate_clock_lag`): the mocap and joint stamps
   come from the same ROS clock to within a fraction of a second; the offset is
   measured, not assumed. It maximises the correlation between the angular
   speed of the ``upper_body_frame`` body and the speed norm of the two torso
   joints (the only joints that move that body), on a 0.05 s grid, searched over
   +-2 s in 0.02 s steps. Positive lag means the mocap clock is behind:
   ``t_mocap = t_robot - lag``. The correlation peak is flat to about 0.1 s.
3. **Averaging** (:func:`extract_static_postures`): joints over
   ``[start + 0.5 s, end - 0.3 s]`` on the robot clock; each body over the same
   physical interval on the mocap clock (shifted by the lag), as mean position
   and mean rotation (a chordal mean, scalar part >= 0), in the Qualisys world
   frame. A plateau is dropped when a body has fewer than ``MIN_SAMPLES``
   samples in it (the end-effector body arrives at about 1 Hz) or when a
   body's coordinate standard deviation exceeds ``MAX_STD_MM``.

Run

    python -m examples.talos.utils.mocap_extraction --help

from the repository root; the byte-for-byte check against the shipped files is
``tests/test_talos_mocap_extraction.py`` (needs ``TALOS_RECORDINGS_ROOT``).
"""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

BODIES = ["waist", "upper_body", "endeffector"]
#: Child frame names in the bags.
_FRAME_KEYS = {
    "waist_frame": "waist",
    "upper_body_frame": "upper_body",
    "endeffector_frame": "endeffector",
}
#: Minimum samples of each body inside a plateau window.
MIN_SAMPLES = {"waist": 20, "upper_body": 20, "endeffector": 3}
#: A plateau is dropped when any body's largest coordinate std exceeds this (mm).
MAX_STD_MM = 5.0
#: A gap longer than this many median sample periods counts as tracking loss.
LOSS_FACTOR = 5.0
POSE = ["x", "y", "z", "qx", "qy", "qz", "qw"]


def joint_names(arm: str) -> List[str]:
    """Shipped joints: the seven arm joints of ``arm`` (left or right), then the torso."""
    if arm not in ("left", "right"):
        raise ValueError(f"arm must be 'left' or 'right', got {arm!r}")
    return [f"arm_{arm}_{i}_joint" for i in range(1, 8)] + [
        "torso_1_joint",
        "torso_2_joint",
    ]


def columns(arm: str) -> List[str]:
    """Column order of a shipped posture file."""
    out = ["plateau", "t_start_robot", "t_end_robot"] + joint_names(arm)
    for body in BODIES:
        out += [f"{body}_{c}" for c in POSE] + [f"n_{body}", f"{body}_std_mm"]
    return out


def static_plateaus(
    t: np.ndarray,
    q: np.ndarray,
    window: float = 0.5,
    tol: float = 1e-3,
    min_duration: float = 2.0,
) -> List[Tuple[int, int]]:
    """Index ranges ``[i0, i1)`` where the joints are static.

    A sample is static when every joint's peak-to-peak over the following
    ``window`` seconds is below ``tol``; consecutive static samples form a
    plateau, kept if it lasts at least ``min_duration`` seconds.
    """
    still = np.zeros(len(t), bool)
    for i in range(len(t)):
        j = np.searchsorted(t, t[i] + window)
        if j < len(t) and np.ptp(q[i:j], axis=0).max() < tol:
            still[i:j] = True
    edges = np.flatnonzero(np.diff(np.r_[0, still.astype(int), 0]))
    return [
        (edges[k], edges[k + 1])
        for k in range(0, len(edges), 2)
        if t[edges[k + 1] - 1] - t[edges[k]] >= min_duration
    ]


def _angular_speed(t: np.ndarray, quat: np.ndarray, grid: np.ndarray) -> np.ndarray:
    """Rotation speed (rad/s) of a body on ``grid`` (NaN outside its stream)."""
    from scipy.spatial.transform import Slerp

    keep = np.r_[True, np.diff(t) > 0]
    inside = (grid >= t[0]) & (grid <= t[-1])
    g = grid[inside]
    rot = Slerp(t[keep], Rotation.from_quat(quat[keep]))(g)
    w = (rot[1:] * rot[:-1].inv()).magnitude() / np.diff(g)
    out = np.full(len(grid), np.nan)
    out[np.flatnonzero(inside)[:-1]] = w
    return out


def estimate_clock_lag(
    t_robot: np.ndarray,
    q_torso: np.ndarray,
    body: np.ndarray,
    search: Tuple[float, float] = (-2.0, 2.0),
    step: float = 0.02,
) -> Tuple[float, float]:
    """Lag (s) maximising the correlation of torso and body rotation speeds.

    Args:
        t_robot: Robot timestamps of ``q_torso``.
        q_torso: (n, 2) ``torso_1``, ``torso_2`` positions.
        body: (m, 8) ``upper_body`` stream: ``t, x, y, z, qx, qy, qz, qw``.
        search: Lag interval searched.
        step: Lag resolution.

    Returns:
        (lag, correlation); ``t_mocap = t_robot - lag``.
    """
    g = np.arange(t_robot[0] + 1.0, t_robot[-1] - 1.0, 0.05)
    qs = np.column_stack([np.interp(g, t_robot, q_torso[:, c]) for c in range(2)])
    speed_robot = np.linalg.norm(np.gradient(qs, g, axis=0), axis=1)
    speed_body = _angular_speed(body[:, 0], body[:, 4:8], g)
    best = (-1.0, 0.0)
    for lag in np.arange(search[0], search[1] + 1e-9, step):
        a = np.interp(g - lag, g, speed_body, left=np.nan, right=np.nan)
        ok = ~np.isnan(a) & ~np.isnan(speed_robot)
        if ok.sum() > 200:
            best = max(best, (np.corrcoef(a[ok], speed_robot[ok])[0, 1], lag))
    return round(float(best[1]), 2), float(best[0])


def _mean_pose(samples: np.ndarray) -> np.ndarray:
    """Mean position and mean rotation of rows ``t, x, y, z, qx, qy, qz, qw``."""
    quat = Rotation.from_quat(samples[:, 4:8]).mean().as_quat()
    if quat[3] < 0:
        quat = -quat
    return np.r_[samples[:, 1:4].mean(axis=0), quat]


def extract_static_postures(
    t: np.ndarray,
    q: np.ndarray,
    bodies: Dict[str, np.ndarray],
    lag: float,
    arm: str,
) -> Tuple[pd.DataFrame, Dict[str, int]]:
    """One row per static plateau, in the layout of :func:`columns`.

    Args:
        t: (n,) robot timestamps.
        q: (n, 9) joint positions, in :func:`joint_names` order.
        bodies: ``{"waist": (k, 8), ...}`` for :data:`BODIES`, rows
            ``t, x, y, z, qx, qy, qz, qw`` in the Qualisys world frame.
        lag: Clock lag from :func:`estimate_clock_lag` (``t_mocap = t - lag``).
        arm: ``left`` or ``right``.

    Returns:
        (table, drops) with ``drops = {"plateaus": found, "too_few_samples": n,
        "unsteady": n}`` (a plateau with too few samples is not also counted as
        unsteady).
    """
    rows = []
    drops = {"plateaus": 0, "too_few_samples": 0, "unsteady": 0}
    for k, (a, b) in enumerate(static_plateaus(t, q)):
        drops["plateaus"] += 1
        t0, t1 = t[a] + 0.5, t[b - 1] - 0.3
        w0, w1 = t0 - lag, t1 - lag
        poses, counts, stds = [], [], []
        for body in BODIES:
            s = bodies[body]
            sel = s[(s[:, 0] >= w0) & (s[:, 0] <= w1)]
            if len(sel) < MIN_SAMPLES[body]:
                drops["too_few_samples"] += 1
                break
            poses.append(_mean_pose(sel))
            counts.append(len(sel))
            stds.append(sel[:, 1:4].std(axis=0).max() * 1000)
        else:
            if max(stds) > MAX_STD_MM:
                drops["unsteady"] += 1
                continue
            row = [k, t0, t1] + list(q[a:b].mean(axis=0))
            for pose, n, std in zip(poses, counts, stds):
                row += list(pose) + [n, std]
            rows.append(row)
    return pd.DataFrame(rows, columns=columns(arm)), drops


def audit_streams(
    t: np.ndarray, bodies: Dict[str, np.ndarray]
) -> Dict[str, Dict[str, float]]:
    """Duration, sample rates, frozen samples and tracking loss of a session.

    A frozen sample is a pose identical in all seven values to the previous
    sample of its body. Tracking loss is a gap longer than :data:`LOSS_FACTOR`
    median sample periods between consecutive samples of a body (the bags hold
    no flag or NaN for a lost body: a lost body publishes nothing).
    """
    out: Dict[str, Dict[str, float]] = {
        "robot": {
            "samples": int(len(t)),
            "duration_s": round(float(t[-1] - t[0]), 1),
            "rate_hz": round(float(1.0 / np.median(np.diff(t))), 1),
        }
    }
    for body in BODIES:
        s = bodies[body]
        dt = np.diff(s[:, 0])
        gap = dt > LOSS_FACTOR * np.median(dt)
        out[body] = {
            "samples": int(len(s)),
            "rate_hz": round(float(1.0 / np.median(dt)), 1),
            "frozen": int((np.abs(np.diff(s[:, 1:8], axis=0)).max(axis=1) == 0).sum()),
            "nan": int(np.isnan(s).sum()),
            "tracking_loss_gaps": int(gap.sum()),
            "tracking_loss_s": round(float(dt[gap].sum()), 1),
        }
    return out


def read_bag(path: str, arm: str) -> Dict[str, np.ndarray]:
    """Read joints and Qualisys bodies from an original session bag.

    Needs the optional ``rosbags`` package. Returns ``{"t": (n,), "q": (n, 9),
    "waist": (k, 8), "upper_body": ..., "endeffector": ...}`` using header
    stamps; ``q`` is in :func:`joint_names` order.
    """
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore

    ts = get_typestore(Stores.ROS1_NOETIC)
    tf: Dict[str, list] = {}
    names: Dict[int, List[str]] = {}
    values = []
    with Reader(path) as reader:
        for c in reader.connections:
            if c.msgtype not in ts.types:
                ts.register(get_types_from_msg(c.msgdef.data, c.msgtype))
        for c, _, raw in reader.messages():
            if c.topic == "/tf_throttle":
                for tr in ts.deserialize_ros1(raw, c.msgtype).transforms:
                    if tr.header.frame_id == "Qualisys":
                        stamp = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
                        a, r = tr.transform.translation, tr.transform.rotation
                        tf.setdefault(tr.child_frame_id, []).append(
                            (stamp, a.x, a.y, a.z, r.x, r.y, r.z, r.w)
                        )
            elif c.msgtype.endswith("StatisticsNames"):
                m = ts.deserialize_ros1(raw, c.msgtype)
                names[m.names_version] = list(m.names)
            elif c.msgtype.endswith("StatisticsValues"):
                m = ts.deserialize_ros1(raw, c.msgtype)
                stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                values.append((stamp, m.names_version, np.asarray(m.values)))
    index: Dict[int, List[int]] = {}
    rows = []
    for stamp, version, vals in values:
        nm = names.get(version)
        if nm is None or len(nm) != len(vals):
            continue
        if version not in index:
            index[version] = [nm.index(f"{j}_position") for j in joint_names(arm)]
        rows.append([stamp] + list(vals[index[version]]))
    joints = np.array(sorted(rows))
    out = {"t": joints[:, 0], "q": joints[:, 1:]}
    for child, key in _FRAME_KEYS.items():
        out[key] = np.array(sorted(tf[child]))
    return out


def file_sha256(path: str | Path) -> str:
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def extract_session(
    bag: str,
    arm: str,
    lag: float | None = None,
    expected_sha256: str | None = None,
) -> Tuple[pd.DataFrame, float, Dict[str, int], Dict[str, Dict[str, float]]]:
    """Bag to posture table. Measures the clock lag if not given.

    ``expected_sha256`` refuses a bag that is not the recorded one. Returns
    ``(table, lag, drops, audit)``.
    """
    joint_names(arm)  # validates the arm before the slow reads
    if expected_sha256 is not None:
        actual = file_sha256(bag)
        if actual != expected_sha256:
            raise ValueError(f"{bag}: source sha256 mismatch ({actual})")
    s = read_bag(bag, arm)
    if lag is None:
        lag, _ = estimate_clock_lag(s["t"], s["q"][:, 7:9], s["upper_body"])
    bodies = {b: s[b] for b in BODIES}
    table, drops = extract_static_postures(s["t"], s["q"], bodies, lag, arm)
    return table, lag, drops, audit_streams(s["t"], bodies)


if __name__ == "__main__":
    import argparse
    import json

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("bag")
    parser.add_argument("out")
    parser.add_argument("--arm", choices=("left", "right"), required=True)
    parser.add_argument(
        "--lag",
        type=float,
        help="t_robot - t_mocap (s); measured from the bag when omitted",
    )
    parser.add_argument("--expected-sha256", help="Source bag hash from protocol.yaml")
    args = parser.parse_args()
    table, lag, drops, audit = extract_session(
        args.bag, args.arm, lag=args.lag, expected_sha256=args.expected_sha256
    )
    table.to_csv(args.out, index=False)
    print(json.dumps({"lag_s": lag, "drops": drops, "audit": audit}, indent=1))
    print(f"{len(table)} postures -> {args.out}")
