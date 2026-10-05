"""Static-posture extraction for the TIAGo Qualisys calibration sessions (#67, #27).

These are the rules that produced the files in
``examples/tiago/data/calibration/mocap/``. The original ROS bags are not
distributed; the functions take plain arrays so the rules can be reviewed and
tested here, and :func:`read_qualisys_bag` reads a bag when the optional
``rosbags`` package is installed.

1. **Static plateaus** (:func:`static_plateaus`): every joint stays within
   ``tol`` (1 mrad / 1 mm) over ``window`` (0.5 s), for at least
   ``min_duration`` (2 s).
2. **Clock lag** (:func:`estimate_clock_lag`): the robot and mocap clocks
   differ by seconds. The lag maximises the correlation between the speed of
   the nominal-FK tool point and the speed of a tracked point. Positive lag
   means the mocap clock is behind: ``t_mocap = t_robot - lag``.
3. **Averaging** (:func:`extract_static_postures`): joints over
   ``[start + 0.5 s, end - 0.3 s]`` on the robot clock; each tracked point over
   the same physical interval on the mocap clock, expressed in the mocap
   ``base_frame`` body (fixed to the robot base). A plateau is skipped when any
   stream has fewer than 20 samples in it.

Columns written: ``x1..z4`` (points BL, BR, TR, TL), the eight joints,
``t_start_robot``, ``t_end_robot`` (s, robot clock) and ``marker_std_mm`` (the
largest standard deviation of any point coordinate over the window).
"""

from __future__ import annotations

from typing import Dict, List, Sequence, Tuple

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

JOINTS = ["torso_lift_joint"] + [f"arm_{i}_joint" for i in range(1, 8)]
POINTS = ["BL", "BR", "TR", "TL"]


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


def estimate_clock_lag(
    t_robot: np.ndarray,
    tool_positions: np.ndarray,
    t_mocap: np.ndarray,
    mocap_positions: np.ndarray,
    search: Tuple[float, float] = (-30.0, 30.0),
    step: float = 0.1,
) -> Tuple[float, float]:
    """Lag (s) maximising the correlation of tool and mocap point speeds.

    Args:
        t_robot: Robot timestamps of ``tool_positions`` (nominal FK of the
            tracked point, any frame).
        tool_positions: (n, 3) tool point positions.
        t_mocap: Mocap timestamps of ``mocap_positions``.
        mocap_positions: (m, 3) tracked point positions.
        search: Lag interval searched.
        step: Lag resolution.

    Returns:
        (lag, correlation); ``t_mocap = t_robot - lag``.
    """
    g = np.arange(t_robot[0], t_robot[-1], 0.08)
    x = np.column_stack([np.interp(g, t_robot, tool_positions[:, c]) for c in range(3)])
    speed_robot = np.linalg.norm(np.gradient(x, g, axis=0), axis=1)
    tm = np.arange(t_mocap[0], t_mocap[-1], 0.1)
    p = np.column_stack(
        [np.interp(tm, t_mocap, mocap_positions[:, c]) for c in range(3)]
    )
    speed_mocap = np.linalg.norm(np.gradient(p, tm, axis=0), axis=1)
    best = (-1.0, 0.0)
    for lag in np.arange(search[0], search[1], step):
        a = np.interp(g, tm + lag, speed_mocap, left=np.nan, right=np.nan)
        ok = ~np.isnan(a)
        if ok.sum() > 200:
            best = max(best, (np.corrcoef(a[ok], speed_robot[ok])[0, 1], lag))
    return float(best[1]), float(best[0])


def extract_static_postures(
    t: np.ndarray,
    q: np.ndarray,
    base: np.ndarray,
    points: Dict[str, np.ndarray],
    lag: float,
    min_samples: int = 20,
) -> pd.DataFrame:
    """One row per static plateau, in the FIGAROH mocap CSV layout.

    Args:
        t: (n,) robot timestamps.
        q: (n, 8) joint positions, in :data:`JOINTS` order.
        base: (m, 8) ``base_frame`` body stream: ``t, x, y, z, qx, qy, qz, qw``.
        points: ``{"BL": (k, 4) [t, x, y, z], ...}`` for :data:`POINTS`.
        lag: Clock lag from :func:`estimate_clock_lag` (``t_mocap = t - lag``).
        min_samples: Minimum samples of every stream inside a window.
    """
    rows = []
    for a, b in static_plateaus(t, q):
        t0, t1 = t[a] + 0.5, t[b - 1] - 0.3
        w0, w1 = t0 - lag, t1 - lag
        in_base = (base[:, 0] >= w0) & (base[:, 0] <= w1)
        if in_base.sum() < min_samples:
            continue
        R_b = Rotation.from_quat(base[in_base, 4:8]).mean().as_matrix()
        p_b = base[in_base, 1:4].mean(axis=0)
        coords, std = [], 0.0
        for name in POINTS:
            s = points[name]
            sel = (s[:, 0] >= w0) & (s[:, 0] <= w1)
            if sel.sum() < min_samples:
                break
            coords.append(R_b.T @ (s[sel, 1:4].mean(axis=0) - p_b))
            std = max(std, s[sel, 1:4].std(axis=0).max() * 1000)
        else:
            rows.append(np.r_[np.ravel(coords), q[a:b].mean(axis=0), t0, t1, std])
    columns = (
        [f"{c}{k}" for k in range(1, 5) for c in "xyz"]
        + JOINTS
        + ["t_start_robot", "t_end_robot", "marker_std_mm"]
    )
    return pd.DataFrame(rows, columns=columns)


def tool_positions(model, frame: str, q: np.ndarray) -> np.ndarray:
    """Nominal FK positions of ``frame`` for joint rows ``q`` (:data:`JOINTS` order)."""
    import pinocchio as pin

    data = model.createData()
    fid = model.getFrameId(frame)
    idx = [model.joints[model.getJointId(j)].idx_q for j in JOINTS]
    out = np.empty((len(q), 3))
    for i, row in enumerate(q):
        config = pin.neutral(model)
        config[idx] = row
        pin.framesForwardKinematics(model, data, config)
        out[i] = data.oMf[fid].translation
    return out


def read_qualisys_bag(path: str) -> Dict[str, np.ndarray]:
    """Read joints and Qualisys streams from an original session bag.

    Needs the optional ``rosbags`` package (no ROS install). Returns
    ``{"t": (n,), "q": (n, 8), "base_frame": (m, 8), "BL": (k, 4), ...}``
    using header stamps: joints from PAL introspection
    (``<joint>_position``), mocap from ``/tf`` transforms whose parent frame
    is ``Qualisys``.
    """
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore

    ts = get_typestore(Stores.ROS1_NOETIC)
    tf: Dict[str, list] = {}
    names: Dict[int, Sequence[str]] = {}
    values = []
    with Reader(path) as reader:
        for c in reader.connections:
            if c.msgtype not in ts.types:
                ts.register(get_types_from_msg(c.msgdef.data, c.msgtype))
        for c, _, raw in reader.messages():
            if c.topic in ("/tf", "/tf_static"):
                for tr in ts.deserialize_ros1(raw, c.msgtype).transforms:
                    if "Qualisys" in tr.header.frame_id:
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
    rows = []
    for stamp, version, vals in values:
        nm = names.get(version)
        if nm is None or len(nm) != len(vals):
            continue
        rows.append([stamp] + [vals[nm.index(f"{j}_position")] for j in JOINTS])
    joints = np.array(sorted(rows))
    out = {"t": joints[:, 0], "q": joints[:, 1:]}
    for child, samples in tf.items():
        arr = np.array(samples)
        key = child.removeprefix("eeframe_")
        out[key] = arr if child == "base_frame" else arr[:, :4]
    return out


def extract_session(
    bag: str,
    lag: float | None = None,
    model=None,
    tool_frame: str = "wrist_ft_tool_link",
) -> Tuple[pd.DataFrame, float]:
    """Bag to posture table. Estimates the lag from ``model`` if not given."""
    s = read_qualisys_bag(bag)
    if lag is None:
        if model is None:
            raise ValueError("give the clock lag, or a model to estimate it")
        lag, _ = estimate_clock_lag(
            s["t"],
            tool_positions(model, tool_frame, s["q"]),
            s["BL"][:, 0],
            s["BL"][:, 1:4],
        )
    return extract_static_postures(s["t"], s["q"], s["base_frame"], s, lag), lag


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("bag")
    parser.add_argument("out")
    parser.add_argument(
        "--lag", type=float, required=True, help="t_robot - t_mocap (s)"
    )
    args = parser.parse_args()
    table, lag = extract_session(args.bag, lag=args.lag)
    table.to_csv(args.out, index=False)
    print(f"{len(table)} postures, lag {lag:+.1f} s -> {args.out}")
