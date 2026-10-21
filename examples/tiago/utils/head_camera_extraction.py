"""Head-camera chessboard sessions of 2023-10 to 2023-11 (#108).

These are the rules that produced the files in
``examples/tiago/data/calibration/head_camera/``. The original recordings are
not distributed; the functions take plain arrays or text so the rules can be
reviewed and tested here, and :func:`read_run_bag` reads a ROS bag when the
optional ``rosbags`` package is installed.

Two kinds of source:

1. **PAL tool CSVs** (camera side). PAL's eye-hand calibration tool writes one
   CSV per session: the chessboard pose in the camera frame (``x1..phiz1``,
   from the tool's own pose estimate on the image) and the joint angles, one
   row per posture, 6 significant digits, no timestamps, a trailing comma.
   :func:`camera_files` splits it into a pose table and a joints table and
   keeps every value token exactly as written. The torso is not in the CSV; it
   is taken from the paired bag where one exists and is ``0.30`` (assumed)
   otherwise. The joints are the absolute-encoder readings (they match the
   ``*_absolute_encoder_position`` channel of the bags to about 1 mrad, not
   ``*_joint_position``).
2. **OptiTrack bags** (mocap side, 2023-10-27 and 2023-11-07). The bags hold
   the ``tiago_camera`` and ``tiago_chessboard`` rigid bodies and PAL
   introspection. :func:`extract_mocap_run` gives the chessboard pose in the
   ``tiago_camera`` body per static plateau.

Mocap rules.

* **Static plateaus**: every one of the arm, head joints (absolute
  encoders) within 5 mrad over 0.5 s, for at least 1 s (the encoder noise of
  these runs is a few mrad, so the 1 mrad / 2 s rule of the other sessions
  finds almost nothing). Window ``[start + 0.5 s, end - 0.3 s]`` shifted by the
  measured clock lag (:func:`measure_clock_lag`, as in
  ``optitrack_extraction``).
* **Frozen samples** are removed first (``drop_frozen``), as in the 2023-09-12
  OptiTrack files.
* **Pairing**: a plateau is paired with the PAL CSV row of nearest joints
  (Hungarian assignment, 10 mrad). Plateaus without a row are dropped
  (``unpaired``). The sessions repeat one posture plan, so pairing is by plan
  and joints, never by time.
* **Flips** (the chessboard body is nearly planar and OptiTrack swaps between
  mirror solutions). Two kinds are handled.

  - *Within a plateau*: samples more than 90 degrees from the plateau's
    medoid orientation are a flip. They are removed and the plateau is
    ``repaired`` when they are at most 40 % of the live samples; above that
    the plateau is ``ambiguous`` and dropped. A flip also moves the reported
    origin, so the position is averaged over the kept samples only.
  - *Between plateaus*: the rotation ``X = R_pal^T R_mocap`` between the PAL
    board frame and the mocap body is constant for a rigid body. A plateau
    whose ``X`` is more than 90 degrees from the medoid ``X`` of its run is a
    whole-plateau flip and is dropped (``flipped``). PAL orientation columns
    are read as pinocchio roll-pitch-yaw (``R = Rz Ry Rx``). Genuine plateaus
    scatter by 20 to 65 degrees (the PAL estimate is noisy); flips sit at 105
    degrees and more. The PAL values only select flips and are never written
    into the mocap files.

The mocap pose is ``p = Rc^T (p_cb - p_cam)``, ``R = Rc^T R_cb`` with
orientation as pinocchio roll-pitch-yaw (``Rotation.as_euler("xyz")``).

Run ``python -m examples.tiago.utils.head_camera_extraction --help`` from the
repository root; the byte-for-byte check is
``tests/test_tiago_head_camera_extraction.py`` (needs ``TIAGO_RECORDINGS_ROOT``).
"""

from __future__ import annotations

import io
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial.transform import Rotation

from examples.tiago.utils.mocap_extraction import file_sha256, static_plateaus
from examples.tiago.utils.optitrack_extraction import (
    drop_frozen,
    measure_clock_lag,
)

ARM = [f"arm_{i}_joint" for i in range(1, 8)]
HEAD = ["head_1_joint", "head_2_joint"]
CAMERA_JOINTS = ARM + HEAD
POSE = ["x", "y", "z", "phix", "phiy", "phiz"]
PAL_POSE = [f"{c}1" for c in POSE]
TORSO = "torso_lift_joint"
TORSO_ASSUMED = "0.30"
CAMERA_BODY = "tiago_camera"
BOARD_BODY = "tiago_chessboard"

PLATEAU = dict(window=0.5, tol=5e-3, min_duration=1.0)
PAIR_TOL = 0.01  # rad, plateau mean joints to PAL row
FLIP_DEG = 90.0
MAX_MINORITY = 0.4
MIN_SAMPLES = 20

CAMERA_POSE_HEADER = ["posture"] + POSE
CAMERA_JOINT_HEADER = ["posture", TORSO] + CAMERA_JOINTS + ["torso_source"]
MOCAP_HEADER = (
    ["posture", "plateau"]
    + POSE
    + [TORSO]
    + CAMERA_JOINTS
    + [
        "t_start_robot",
        "t_end_robot",
        "n_camera",
        "n_board_window",
        "n_board_kept",
        "n_flipped_removed",
        "pose_std_mm",
        "status",
    ]
)


# ----------------------------------------------------------------- camera side
def parse_camera_csv(text: str) -> Tuple[List[str], List[List[str]]]:
    """Header names and value tokens of a PAL tool CSV, tokens kept verbatim."""
    lines = [ln for ln in text.splitlines() if ln.strip()]
    split = [[tok for tok in ln.split(",")] for ln in lines]
    split = [row[:-1] if row and row[-1] == "" else row for row in split]
    return split[0], split[1:]


def camera_files(
    text: str, torso: Optional[Sequence[Optional[str]]] = None
) -> Tuple[str, str]:
    """PAL CSV text to ``(pose table, joints table)`` CSV text.

    ``torso`` gives a torso token per row (``None`` for assumed 0.30). The
    pose and joint tokens are copied as written, never re-formatted.
    """
    header, rows = parse_camera_csv(text)
    want = PAL_POSE + CAMERA_JOINTS
    if header != want:
        raise ValueError(f"unexpected PAL CSV header {header}")
    if torso is None:
        torso = [None] * len(rows)
    pose, joints = io.StringIO(), io.StringIO()
    pose.write(",".join(CAMERA_POSE_HEADER) + "\n")
    joints.write(",".join(CAMERA_JOINT_HEADER) + "\n")
    for i, (row, tq) in enumerate(zip(rows, torso)):
        if len(row) != len(want):
            raise ValueError(f"row {i}: {len(row)} values, expected {len(want)}")
        pose.write(",".join([str(i)] + row[:6]) + "\n")
        src = "assumed" if tq is None else "bag"
        joints.write(",".join([str(i), tq or TORSO_ASSUMED] + row[6:] + [src]) + "\n")
    return pose.getvalue(), joints.getvalue()


def pal_values(text: str) -> Tuple[np.ndarray, np.ndarray]:
    """``(pose (n, 6), joints (n, 9))`` floats of a PAL tool CSV."""
    _, rows = parse_camera_csv(text)
    a = np.array([[float(v) for v in r] for r in rows])
    return a[:, :6], a[:, 6:]


# ------------------------------------------------------------------ mocap side
def read_run_bag(path: str) -> Dict[str, np.ndarray]:
    """Joints and the camera and chessboard bodies from an original run bag.

    Needs the optional ``rosbags`` package. Returns ``{"t", "t_recv"
    (n,), "q" (n, 9) absolute-encoder arm + head, "torso" (n,), "tiago_camera",
    "tiago_chessboard" (m, 9)}``; bodies have columns ``t_header, t_recv, x,
    y, z, qx, qy, qz, qw`` in bag order.
    """
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore

    ts = get_typestore(Stores.ROS1_NOETIC)
    topics = {f"/natnet_ros/{b}/pose": b for b in (CAMERA_BODY, BOARD_BODY)}
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
    chan = [
        f"{j.replace('_joint', '_joint_absolute_encoder')}_position"
        for j in CAMERA_JOINTS
    ] + [f"{TORSO}_position"]
    rows = []
    for stamp, recv, version, vals in values:
        nm = names.get(version)
        if nm is None or len(nm) != len(vals):
            continue
        rows.append([stamp, recv] + [vals[nm.index(c)] for c in chan])
    joints = np.array(sorted(rows))
    out = {
        "t": joints[:, 0],
        "t_recv": joints[:, 1],
        "q": joints[:, 2:11],
        "torso": joints[:, 11],
    }
    out.update({b: np.array(s) for b, s in bodies.items()})
    return out


def pair_plateaus(plateau_q: np.ndarray, pal_q: np.ndarray) -> Dict[int, int]:
    """Plateau index to PAL row index by joint distance (one row per plateau)."""
    if len(plateau_q) == 0 or len(pal_q) == 0:
        return {}
    dist = np.abs(plateau_q[:, None, :] - pal_q[None, :, :]).max(axis=2)
    rows, cols = linear_sum_assignment(dist)
    return {int(i): int(j) for i, j in zip(rows, cols) if dist[i, j] < PAIR_TOL}


def _window(stream: np.ndarray, w0: float, w1: float) -> np.ndarray:
    return stream[(stream[:, 0] >= w0) & (stream[:, 0] <= w1)]


def _medoid(rot: Rotation, n_candidates: int = 40) -> int:
    """Index of the sample with the smallest median angle to all the others."""
    idx = np.arange(0, len(rot), max(1, len(rot) // n_candidates))
    score = [np.median((rot[int(i)].inv() * rot).magnitude()) for i in idx]
    return int(idx[int(np.argmin(score))])


def _fmt(v: float) -> str:
    return f"{v:.9g}"


def extract_mocap_run(
    t: np.ndarray,
    q: np.ndarray,
    torso: np.ndarray,
    camera: np.ndarray,
    board: np.ndarray,
    lag: float,
    pal_pose: np.ndarray,
    pal_q: np.ndarray,
) -> Tuple[str, Dict[str, int]]:
    """Chessboard-in-camera plateau table (CSV text) and the drop counts.

    Args:
        t, q, torso: robot stamps, (n, 9) arm + head and (n,) torso joints.
        camera, board: raw body streams (``t_header, t_recv, x, y, z, qx, qy,
            qz, qw``); frozen samples are removed here.
        lag: ``t_robot - t_mocap`` (:func:`measure_clock_lag`).
        pal_pose, pal_q: the PAL CSV of the paired session, (m, 6), (m, 9).

    Returns:
        ``(csv text, drops)``; ``drops`` has ``plateaus, unpaired, lost,
        ambiguous, flipped`` (plateaus dropped), ``repaired`` (kept plateaus
        that had flipped samples removed), ``removed_samples`` and ``kept``.
    """
    camera, board = drop_frozen(camera), drop_frozen(board)
    plateaus = static_plateaus(t, q, **PLATEAU)
    pq = np.array([q[a:b].mean(axis=0) for a, b in plateaus]).reshape(-1, q.shape[1])
    pairs = pair_plateaus(pq, pal_q)
    drops = dict(
        plateaus=len(plateaus),
        unpaired=0,
        lost=0,
        ambiguous=0,
        flipped=0,
        repaired=0,
        removed_samples=0,
        kept=0,
    )
    cand = []
    for k, (a, b) in enumerate(plateaus):
        if k not in pairs:
            drops["unpaired"] += 1
            continue
        t0, t1 = t[a] + 0.5, t[b - 1] - 0.3
        sc = _window(camera, t0 - lag, t1 - lag)
        sb = _window(board, t0 - lag, t1 - lag)
        if len(sc) < MIN_SAMPLES or len(sb) < MIN_SAMPLES:
            drops["lost"] += 1
            continue
        rb = Rotation.from_quat(sb[:, 5:9])
        ref = rb[_medoid(rb)]
        keep = (ref.inv() * rb).magnitude() <= np.radians(FLIP_DEG)
        minority = 1.0 - keep.mean()
        if keep.sum() < MIN_SAMPLES:
            drops["lost"] += 1
            continue
        if minority > MAX_MINORITY:
            drops["ambiguous"] += 1
            continue
        sb = sb[keep]
        Rc = Rotation.from_quat(sc[:, 5:9]).mean().as_matrix()
        Rb = Rotation.from_quat(sb[:, 5:9]).mean().as_matrix()
        p_rel = Rc.T @ (sb[:, 2:5].mean(axis=0) - sc[:, 2:5].mean(axis=0))
        R_rel = Rc.T @ Rb
        std = (sb[:, 2:5].std(axis=0).max()) * 1000
        pose = np.r_[p_rel, Rotation.from_matrix(R_rel).as_euler("xyz")]
        x_rot = Rotation.from_euler(
            "xyz", pal_pose[pairs[k], 3:]
        ).inv() * Rotation.from_matrix(R_rel)
        cand.append(
            dict(
                k=k,
                a=a,
                b=b,
                pose=pose,
                x=x_rot,
                t0=t0,
                t1=t1,
                n_cam=len(sc),
                n_win=len(keep),
                n_kept=int(keep.sum()),
                std=std,
            )
        )
    if cand:
        xs = Rotation.concatenate([c["x"] for c in cand])
        med = xs[int(np.argmin([np.median((x.inv() * xs).magnitude()) for x in xs]))]
    out = io.StringIO()
    out.write(",".join(MOCAP_HEADER) + "\n")
    for c in cand:
        if (med.inv() * c["x"]).magnitude() > np.radians(FLIP_DEG):
            drops["flipped"] += 1
            continue
        removed = c["n_win"] - c["n_kept"]
        status = "repaired" if removed else "ok"
        if removed:
            drops["repaired"] += 1
            drops["removed_samples"] += removed
        drops["kept"] += 1
        a, b = c["a"], c["b"]
        row = (
            [str(pairs[c["k"]]), str(c["k"])]
            + [_fmt(v) for v in c["pose"]]
            + [_fmt(torso[a:b].mean())]
            + [_fmt(v) for v in q[a:b].mean(axis=0)]
            + [_fmt(c["t0"]), _fmt(c["t1"])]
            + [
                str(c["n_cam"]),
                str(c["n_win"]),
                str(c["n_kept"]),
                str(c["n_win"] - c["n_kept"]),
            ]
            + [_fmt(c["std"]), status]
        )
        out.write(",".join(row) + "\n")
    return out.getvalue(), drops


def extract_run(
    bag: str, pal_csv_text: str, expected_sha256: Optional[str] = None
) -> Tuple[str, float, Dict[str, int]]:
    """Bag and its PAL CSV to ``(mocap table text, lag, drops)``."""
    if expected_sha256 is not None:
        actual = file_sha256(bag)
        if actual != expected_sha256:
            raise ValueError(f"{bag}: source sha256 mismatch ({actual})")
    s = read_run_bag(bag)
    live = drop_frozen(s[BOARD_BODY])
    lag = measure_clock_lag((s["t"], s["t_recv"]), (live[:, 0], live[:, 1]))
    pal_pose, pal_q = pal_values(pal_csv_text)
    text, drops = extract_mocap_run(
        s["t"], s["q"], s["torso"], s[CAMERA_BODY], s[BOARD_BODY], lag, pal_pose, pal_q
    )
    return text, lag, drops


def paired_torso(bag: str, pal_csv_text: str) -> List[Optional[str]]:
    """Torso token per PAL row from the paired bag (``None``: no plateau)."""
    s = read_run_bag(bag)
    _, pal_q = pal_values(pal_csv_text)
    plateaus = static_plateaus(s["t"], s["q"], **PLATEAU)
    pq = np.array([s["q"][a:b].mean(axis=0) for a, b in plateaus])
    pairs = pair_plateaus(pq, pal_q)
    out: List[Optional[str]] = [None] * len(pal_q)
    for k, row in pairs.items():
        a, b = plateaus[k]
        out[row] = _fmt(s["torso"][a:b].mean())
    return out


if __name__ == "__main__":
    import argparse
    from pathlib import Path

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="cmd", required=True)
    cam = sub.add_parser("camera", help="PAL CSV to pose and joints tables")
    cam.add_argument("csv")
    cam.add_argument("out_prefix")
    cam.add_argument("--bag", help="paired bag, for the torso")
    moc = sub.add_parser("mocap", help="bag and PAL CSV to the mocap table")
    moc.add_argument("bag")
    moc.add_argument("csv")
    moc.add_argument("out")
    moc.add_argument("--expected-sha256")
    args = parser.parse_args()
    text = Path(args.csv).read_text()
    if args.cmd == "camera":
        torso = paired_torso(args.bag, text) if args.bag else None
        pose, joints = camera_files(text, torso)
        Path(f"{args.out_prefix}_chessboard_in_camera.csv").write_text(pose)
        Path(f"{args.out_prefix}_joints.csv").write_text(joints)
    else:
        table, lag, drops = extract_run(args.bag, text, args.expected_sha256)
        Path(args.out).write_text(table)
        print(f"lag {lag:+.4f} s, {drops}")
