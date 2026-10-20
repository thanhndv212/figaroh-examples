"""Square-motion extraction for the TIAGo Qualisys validation runs (#107).

These are the rules that produced the files in
``examples/tiago/data/validation/square_motion/``. The original recordings are
not distributed: the readers need the optional ``rosbags`` package, the other
functions take plain arrays so the rules can be reviewed and tested here. The
lag search is :func:`examples.tiago.utils.mocap_extraction.estimate_clock_lag`
(the Qualisys, OptiTrack and Vicon files use the same one).

Each run is a ROS bag with the PAL introspection stream (joint positions at
100 Hz) and the Qualisys real-time stream in ``/tf`` (frame ``Qualisys``, about
100 Hz per body, header stamps of the mocap PC). Two files per run, on **one
clock**, the robot clock in seconds from the first joint sample:

* ``<run>_joints.csv``: ``t`` and the eight :data:`JOINTS` positions (rad),
  the introspection ``<joint>_position`` channels at their header stamps;
* ``<run>_markers.csv``: one row per Qualisys sample, ``t, body, x, y, z, qx,
  qy, qz, qw`` (metres, Qualisys world frame, unit quaternion), sorted by
  ``t``. The bodies are ``base_frame`` and the end-effector body (the plate
  points ``eeframe_BL/BR/TR/TL`` in 2021-12, the rigid body
  ``endeffector_frame`` in 2022-01). Samples are never interpolated: a lost
  marker is a gap in ``t``.

1. **Clock lag** (:func:`measure_clock_lag`): speed-profile cross-correlation.
   The speed of the nominal-FK ``wrist_ft_tool_link`` point is correlated with
   the speed of the end-effector marker (``eeframe_BL`` or
   ``endeffector_frame``) over +-60 s at 0.05 s, then 0.005 s around the best.
   ``t_mocap = t_robot - lag`` (the sign of ``mocap_extraction``), so a marker
   stamp goes on the robot clock as ``t_robot = stamp + lag``.
2. **One clock** (:func:`marker_rows`): ``t = stamp + lag - t0`` with ``t0`` the
   first joint header stamp. The lag is rounded to 1 ms.
3. **Applied offsets** (:func:`measure_applied_offsets`): the offset set the
   robot ran leaves a trace in the recording. robot_state_publisher's ``/tf``
   between consecutive arm links is ``M0 @ D @ Rz(q)``, with ``M0`` the
   nominal URDF joint origin and ``q`` the reported ``<joint>_position``; the
   median of ``D`` over the run is the parent-side offset (translation in
   metres, rotation vector in radians). It is zero for the zero set. The
   offsets that PAL applies inside its controller do not show up in ``/tf``
   and are not recoverable from these recordings.
4. **Nominal registration** (:func:`registration_rmse`): the RMS distance
   between the nominal-FK tool point (reported joints) and the measured marker
   path after the best rigid transform and a fixed tool point. It is the
   check that the two files agree on one clock; it is not a calibration.

Run

    python -m examples.tiago.utils.square_motion_extraction --help

from the repository root; the byte-for-byte check against the shipped files is
``tests/test_tiago_square_motion_extraction.py`` (needs
``TIAGO_RECORDINGS_ROOT``).
"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy.spatial.transform import Rotation

from examples.tiago.utils.mocap_extraction import (
    JOINTS,
    estimate_clock_lag,
    file_sha256,
    tool_positions,
)

TOOL_FRAME = "wrist_ft_tool_link"
#: Body whose position is correlated with the tool speed, in order of use.
LAG_BODIES = ("eeframe_BL", "endeffector_frame")
BASE_BODY = "base_frame"
ARM_JOINTS = JOINTS[1:]
#: (parent link, child link) of the robot_state_publisher transform of each arm
#: joint, in :data:`ARM_JOINTS` order.
ARM_PAIRS = [("torso_lift_link", "arm_1_link")] + [
    (f"arm_{i}_link", f"arm_{i + 1}_link") for i in range(1, 7)
]
OFFSET_DECIMALS = 5
JOINT_COLUMNS = ["t"] + JOINTS
MARKER_COLUMNS = ["t", "body", "x", "y", "z", "qx", "qy", "qz", "qw"]


def read_square_bag(path: str) -> Dict[str, object]:
    """Joints, Qualisys bodies and arm ``/tf`` links from a run bag.

    Needs the optional ``rosbags`` package (no ROS install). Returns
    ``{"t": (n,), "q": (n, 8), "bodies": {name: (m, 8)}, "tf": {child: (k,
    8)}}``: header stamps, joints in :data:`JOINTS` order from the PAL
    introspection ``<joint>_position``, rows ``stamp, x, y, z, qx, qy, qz, qw``
    of the ``Qualisys`` child frames, and the same rows of the arm link
    transforms (keyed by child link). Duplicated stamps are not expected and
    are rejected.
    """
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore

    ts = get_typestore(Stores.ROS1_NOETIC)
    wanted = {pair: pair[1] for pair in ARM_PAIRS}
    bodies: Dict[str, list] = {}
    tf: Dict[str, list] = {}
    names: Dict[int, Sequence[str]] = {}
    values = []
    with Reader(path) as reader:
        for c in reader.connections:
            if c.msgtype not in ts.types:
                ts.register(get_types_from_msg(c.msgdef.data, c.msgtype))
        for c, _, raw in reader.messages():
            if c.topic == "/tf":
                for tr in ts.deserialize_ros1(raw, c.msgtype).transforms:
                    pair = (tr.header.frame_id, tr.child_frame_id)
                    is_mocap = tr.header.frame_id == "Qualisys"
                    if not is_mocap and pair not in wanted:
                        continue
                    stamp = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
                    a, r = tr.transform.translation, tr.transform.rotation
                    row = (stamp, a.x, a.y, a.z, r.x, r.y, r.z, r.w)
                    target = bodies if is_mocap else tf
                    target.setdefault(tr.child_frame_id, []).append(row)
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
    if np.any(np.diff(joints[:, 0]) <= 0):
        raise ValueError(f"{path}: duplicated or unordered joint stamps")
    return {
        "t": joints[:, 0],
        "q": joints[:, 1:],
        "bodies": {k: np.array(v) for k, v in bodies.items()},
        "tf": {k: np.array(v) for k, v in tf.items()},
    }


def lag_body(bodies: Dict[str, np.ndarray]) -> str:
    """The end-effector body used for the lag (first of :data:`LAG_BODIES`)."""
    for name in LAG_BODIES:
        if name in bodies:
            return name
    raise ValueError(f"no end-effector body among {sorted(bodies)}")


def measure_clock_lag(
    t: np.ndarray,
    q: np.ndarray,
    stamps: np.ndarray,
    positions: np.ndarray,
    urdf: str,
) -> Tuple[float, float]:
    """``(lag, correlation)`` with ``t_mocap = t_robot - lag``, lag rounded to 1 ms.

    Speed of the nominal-FK :data:`TOOL_FRAME` point against the speed of the
    marker path ``positions`` (n, 3) stamped ``stamps``, over +-60 s at 0.05 s,
    then 0.005 s around the best (the correlation is that of the best lag).
    """
    import pinocchio as pin

    tool = tool_positions(pin.buildModelFromUrdf(urdf), TOOL_FRAME, q)
    args = (t, tool, stamps, positions)
    coarse, _ = estimate_clock_lag(*args, search=(-60.0, 60.0), step=0.05)
    lag, corr = estimate_clock_lag(
        *args, search=(coarse - 0.05, coarse + 0.05), step=0.005
    )
    return round(lag, 3), corr


def measure_applied_offsets(
    tf: Dict[str, np.ndarray], t: np.ndarray, q: np.ndarray, urdf: str
) -> Dict[str, Dict[str, List[float]]]:
    """Parent-side joint offsets seen in the arm ``/tf`` of one run.

    For each arm joint ``/tf`` is ``M0 @ D @ Rz(q)`` (module docstring); returns
    ``{joint: {"translation_m": [3], "rotation_rad": [3]}}``, the median ``D``
    (rotation as a rotation vector) rounded to :data:`OFFSET_DECIMALS`.
    """
    import pinocchio as pin

    model = pin.buildModelFromUrdf(urdf)
    out = {}
    for k, (joint, (_, child)) in enumerate(zip(ARM_JOINTS, ARM_PAIRS), start=1):
        rows = tf[child]
        origin = model.jointPlacements[model.getJointId(joint)]
        qi = np.interp(rows[:, 0], t, q[:, k])
        rz = Rotation.from_euler("z", qi)
        r_delta = (
            Rotation.from_matrix(origin.rotation.T)
            * Rotation.from_quat(rows[:, 4:8])
            * rz.inv()
        )
        p_delta = (rows[:, 1:4] - origin.translation) @ origin.rotation
        out[joint] = {
            "translation_m": [
                float(round(v, OFFSET_DECIMALS)) + 0.0
                for v in np.median(p_delta, axis=0)
            ],
            "rotation_rad": [
                float(round(v, OFFSET_DECIMALS)) + 0.0
                for v in np.median(r_delta.as_rotvec(), axis=0)
            ],
        }
    return out


def marker_rows(
    bodies: Dict[str, np.ndarray], lag: float, t0: float
) -> Tuple[np.ndarray, List[str]]:
    """All Qualisys samples on the robot clock, sorted by time.

    Returns ``(rows, names)``: ``rows`` (n, 8) ``t, x, y, z, qx, qy, qz, qw``
    with ``t = stamp + lag - t0``, and the body name of each row. Bodies enter
    in sorted-name order and the sort by ``t`` is stable.
    """
    parts, names = [], []
    for name in sorted(bodies):
        rows = bodies[name].copy()
        rows[:, 0] = rows[:, 0] + lag - t0
        parts.append(rows)
        names += [name] * len(rows)
    rows = np.vstack(parts)
    order = np.argsort(rows[:, 0], kind="stable")
    return rows[order], [names[i] for i in order]


def format_joints(t: np.ndarray, q: np.ndarray, t0: float) -> str:
    """The joints file: ``t`` (4 decimals) and eight positions (6 decimals)."""
    lines = [",".join(JOINT_COLUMNS)]
    for ti, qi in zip(t - t0, q):
        lines.append(f"{ti:.4f}," + ",".join(f"{v:.6f}" for v in qi))
    return "\n".join(lines) + "\n"


def format_markers(rows: np.ndarray, names: Sequence[str]) -> str:
    """The markers file: ``t`` 4 decimals, positions and quaternion 5 decimals."""
    lines = [",".join(MARKER_COLUMNS)]
    for r, name in zip(rows, names):
        lines.append(f"{r[0]:.4f},{name}," + ",".join(f"{v:.5f}" for v in r[1:]))
    return "\n".join(lines) + "\n"


def tool_poses(model, frame: str, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Nominal FK ``(R (n, 3, 3), p (n, 3))`` of ``frame`` for joint rows ``q``."""
    import pinocchio as pin

    data = model.createData()
    fid = model.getFrameId(frame)
    idx = [model.joints[model.getJointId(j)].idx_q for j in JOINTS]
    rot, pos = np.empty((len(q), 3, 3)), np.empty((len(q), 3))
    for i, row in enumerate(q):
        config = pin.neutral(model)
        config[idx] = row
        pin.framesForwardKinematics(model, data, config)
        rot[i], pos[i] = data.oMf[fid].rotation, data.oMf[fid].translation
    return rot, pos


def registration_rmse(
    t_joint: np.ndarray,
    q: np.ndarray,
    t_marker: np.ndarray,
    marker: np.ndarray,
    urdf: str,
    step: int = 10,
) -> float:
    """RMS (mm) of the nominal tool point against a marker path, rigidly fitted.

    Joints are interpolated at every ``step``-th marker sample inside the joint
    recording (both on the robot clock). The fit has a rigid transform (6) and a
    fixed point in the tool frame (3), started from a Kabsch fit of the tool
    frame origin; no joint offset is fitted.
    """
    import pinocchio as pin
    from scipy.optimize import least_squares

    inside = np.flatnonzero((t_marker >= t_joint[0]) & (t_marker <= t_joint[-1]))[
        ::step
    ]
    m = marker[inside]
    qi = np.column_stack(
        [np.interp(t_marker[inside], t_joint, q[:, k]) for k in range(q.shape[1])]
    )
    rot, pos = tool_poses(pin.buildModelFromUrdf(urdf), TOOL_FRAME, qi)
    ca, cb = pos.mean(axis=0), m.mean(axis=0)
    u, _, vt = np.linalg.svd((pos - ca).T @ (m - cb))
    rigid = vt.T @ np.diag([1, 1, np.linalg.det(vt.T @ u.T)]) @ u.T
    x0 = np.r_[Rotation.from_matrix(rigid).as_rotvec(), cb - rigid @ ca, np.zeros(3)]

    def residual(x):
        r = Rotation.from_rotvec(x[:3]).as_matrix()
        tool = pos + np.einsum("nij,j->ni", rot, x[6:9])
        return ((tool @ r.T + x[3:6]) - m).ravel()

    fit = least_squares(residual, x0)
    return float(1000.0 * np.sqrt(np.mean(fit.fun.reshape(-1, 3) ** 2) * 3))


def extract_run(
    bag: str,
    urdf: str,
    lag: Optional[float] = None,
    expected_sha256: Optional[str] = None,
) -> Dict[str, object]:
    """Bag to the two shipped files plus the measurements stored in the protocol.

    Measures the lag if not given. ``expected_sha256`` refuses a bag that is not
    the recorded one. Returns ``joints_csv`` and ``markers_csv`` (the file
    texts), ``t0`` (first joint stamp, Unix s), ``lag``, ``correlation`` (None
    when ``lag`` is given), ``body`` (lag body), ``offsets``
    (:func:`measure_applied_offsets`), ``registration_rmse_mm``, ``n_joint_rows``
    and ``n_marker_rows``.
    """
    if expected_sha256 is not None:
        actual = file_sha256(bag)
        if actual != expected_sha256:
            raise ValueError(f"{bag}: source sha256 mismatch ({actual})")
    run = read_square_bag(bag)
    t, q, bodies = run["t"], run["q"], run["bodies"]
    body = lag_body(bodies)
    corr = None
    if lag is None:
        lag, corr = measure_clock_lag(
            t, q, bodies[body][:, 0], bodies[body][:, 1:4], urdf
        )
    t0 = float(t[0])
    rows, names = marker_rows(bodies, lag, t0)
    lag_rows = rows[np.array(names) == body]
    return {
        "joints_csv": format_joints(t, q, t0),
        "markers_csv": format_markers(rows, names),
        "t0": t0,
        "lag": lag,
        "correlation": corr,
        "body": body,
        "offsets": measure_applied_offsets(run["tf"], t, q, urdf),
        "registration_rmse_mm": registration_rmse(
            t - t0, q, lag_rows[:, 0], lag_rows[:, 1:4], urdf
        ),
        "n_joint_rows": len(t),
        "n_marker_rows": len(rows),
    }


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("bag", help="run bag (PAL introspection and Qualisys /tf)")
    parser.add_argument(
        "prefix", help="output path prefix: PREFIX_joints.csv, PREFIX_markers.csv"
    )
    parser.add_argument(
        "--urdf",
        default="examples/tiago/urdf/tiago_48_schunk.urdf",
        help="URDF for the nominal FK used by the lag, offset and registration",
    )
    parser.add_argument(
        "--lag", type=float, help="override the measured t_robot - t_mocap (s)"
    )
    parser.add_argument("--expected-sha256", help="bag hash from protocol.yaml")
    args = parser.parse_args()
    result = extract_run(args.bag, args.urdf, args.lag, args.expected_sha256)
    for kind in ("joints", "markers"):
        with open(f"{args.prefix}_{kind}.csv", "w", newline="") as stream:
            stream.write(result[f"{kind}_csv"])
    corr = result["correlation"]
    print(
        f"{result['n_joint_rows']} joint rows, {result['n_marker_rows']} marker rows, "
        f"lag {result['lag']:+.3f} s"
        + ("" if corr is None else f" (correlation {corr:.3f} on {result['body']})")
        + f", nominal registration {result['registration_rmse_mm']:.2f} mm, "
        f"t0 {result['t0']:.6f}"
    )
