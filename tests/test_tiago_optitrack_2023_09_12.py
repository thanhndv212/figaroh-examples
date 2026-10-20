"""TIAGo OptiTrack 2023-09-12 static-posture runs (#105).

Protocol v1 freezes three confirmation sessions in the OptiTrack world frame.
These tests pin their hashes, roles, schema, per-run clock lags, rigid-body
definitions and a registration-only sanity fit, from the shipped files alone.
"""

from pathlib import Path

import numpy as np
import pandas as pd
import pytest
import yaml
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
OPTITRACK = ROOT / "examples/tiago/data/calibration/optitrack"
JOINTS = ["torso_lift_joint"] + [f"arm_{i}_joint" for i in range(1, 8)]
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
# id: (file, rows, rows without a base pose, rigid body, plateaus, gripper lost,
#      sha256)
SESSIONS = {
    "2023-09-12-1529": (
        "optitrack_2023-09-12-1529_static_postures.csv",
        4,
        0,
        1,
        8,
        4,
        "7e27603dc975ad8f0929f376e119ba43ba6df99c79362e8e257128531c70c758",
    ),
    "2023-09-12-1552": (
        "optitrack_2023-09-12-1552_static_postures.csv",
        9,
        1,
        1,
        17,
        8,
        "9bfb7004b13d8d4d094c76696513f6015c47a67aa3caf1c6ac73319672ed111b",
    ),
    "2023-09-12-1658": (
        "optitrack_2023-09-12-1658_static_postures.csv",
        13,
        1,
        2,
        22,
        9,
        "c46bd5d0f1af89ac66bb7af23b6b1d8b48de4d3e7e97a5d7ef01eabfe527d6b5",
    ),
}


ROLES_V2 = {
    "2023-09-12-1529": "confirmation",
    "2023-09-12-1552": "confirmation",
    "2023-09-12-1658": "training",
}


@pytest.fixture(scope="module")
def manifest():
    return yaml.safe_load((OPTITRACK / "protocol.yaml").read_text())


@pytest.fixture(scope="module")
def frames():
    return {sid: pd.read_csv(OPTITRACK / v[0]) for sid, v in SESSIONS.items()}


def test_protocol_lists_the_three_sessions(manifest):
    from figaroh.data import Protocol

    assert (manifest["name"], manifest["version"]) == ("tiago-optitrack-2023-09-12", 2)
    # v2 (#100) only assigns roles: files and hashes are v1's (SESSIONS)
    assert manifest["heldout"]["not_evaluated"] == ["2023-09-12-1529"]
    protocol = Protocol.load(OPTITRACK / "protocol.yaml")
    protocol.verify(root=OPTITRACK)
    assert len(protocol.sessions) == 3
    for sid, v in SESSIONS.items():
        session = protocol.session(sid)
        assert session.role == ROLES_V2[sid]
        assert session.files == {v[0]: v[6]}


def test_sources_are_named_by_sha256_and_not_distributed(manifest):
    raw = {s["id"]: s for s in manifest["sessions"]}
    lags = []
    for sid, v in SESSIONS.items():
        entry = raw[sid]
        source = entry["source"]
        assert "/" not in source["bag"] and source["bag"].endswith(".bag")
        assert len(source["sha256"]) == 64
        assert entry["frame"] == "optitrack:world"
        assert entry["rigid_body"] == v[3]
        assert v[3] in manifest["rigid_bodies"]
        assert entry["drops"] == {"plateaus": v[4], "gripper_lost": v[5]}
        # the table holds every plateau with a tracked gripper
        assert v[4] == v[1] + v[5]
        assert -1.5 < source["lag_s"] < -0.7
        lags.append(source["lag_s"])
    assert len(set(lags)) == 3  # the stamp offset is measured per run
    used = {k: b["used_by"] for k, b in manifest["rigid_bodies"].items()}
    assert used == {1: ["2023-09-12-1529", "2023-09-12-1552"], 2: ["2023-09-12-1658"]}
    assert not list(OPTITRACK.rglob("*.bag"))


@pytest.mark.parametrize("sid", SESSIONS)
def test_schema_counts_and_clocks(sid, frames):
    _, rows, no_base, body, *_ = SESSIONS[sid]
    df = frames[sid]
    assert list(df.columns) == COLUMNS
    assert len(df) == rows
    base_cols = [f"{c}b" for c in POSE]
    assert df.drop(columns=base_cols).notna().all().all()
    assert np.isfinite(df.drop(columns=base_cols).to_numpy()).all()
    assert df[base_cols].isna().all(axis=1).sum() == no_base
    assert df[base_cols].isna().any(axis=1).sum() == no_base  # all or none
    nan_rows = df[base_cols].isna().all(axis=1)
    assert (df.n_base_live[nan_rows] < 20).all()
    assert (df.n_base_live[~nan_rows] >= 20).all()
    assert (df.points_in_base_frame == 0).all()
    assert (df.rigid_body_def == body).all()
    assert (df.n_live >= 20).all() and (df.n_live <= df.n_window).all()
    assert df.pose_std_mm.max() < 1.0 and df.pose_std_deg.max() < 1.0
    dur = df.t_end_robot - df.t_start_robot
    assert (dur >= 1.2).all()
    # windows increase in time and do not overlap
    assert (df.t_start_robot.to_numpy()[1:] > df.t_end_robot.to_numpy()[:-1]).all()
    # recorded on 2023-09-12 (UTC)
    assert 1694476800 < df.t_start_robot.min() and df.t_end_robot.max() < 1694563200
    assert df[JOINTS].abs().max().max() < 3.2


def test_partial_tracking_is_visible_in_the_counts(frames):
    live = {sid: (f.n_live / f.n_window).round(2).tolist() for sid, f in frames.items()}
    assert sum(r < 0.6 for r in live["2023-09-12-1552"]) == 2
    assert sum(r < 0.6 for r in live["2023-09-12-1658"]) == 2
    assert min(live["2023-09-12-1529"]) == 1.0


def _in_base(df):
    """Gripper orientation and position in the tiago_base body, per row."""

    def rot(s):
        return Rotation.from_euler(
            "xyz", df[[f"phi{a}{s}" for a in "xyz"]].fillna(0).to_numpy()
        )

    base = rot("b")
    p = df[["x1", "y1", "z1"]].to_numpy() - df[["xb", "yb", "zb"]].fillna(0).to_numpy()
    return base.inv() * rot("1"), base.inv().apply(p)


def _matched(a, b):
    """Row pairs of the same posture (joints within 0.01 rad), both with a base."""
    pairs = []
    for i, qa in enumerate(a[JOINTS].to_numpy()):
        d = np.abs(b[JOINTS].to_numpy() - qa).max(axis=1)
        j = int(d.argmin())
        if d[j] < 0.01 and not np.isnan(a.xb[i]) and not np.isnan(b.xb[j]):
            pairs.append((i, j))
    return pairs


def test_gripper_rigid_body_was_redefined_before_the_last_run(frames):
    """Same posture, same body: < 2.4 deg. After the redefinition: 5-8 deg."""
    a, b, c = (frames[s] for s in SESSIONS)
    (ra, pa), (rb, pb), (rc, pc) = _in_base(a), _in_base(b), _in_base(c)

    def compare(df1, r1, p1, df2, r2, p2):
        pairs = _matched(df1, df2)
        rel = [r1[i].inv() * r2[j] for i, j in pairs]
        angle = np.degrees([r.magnitude() for r in rel])
        dist = np.array([np.linalg.norm(p1[i] - p2[j]) for i, j in pairs]) * 1e3
        tool_rv = np.degrees([r.as_rotvec() for r in rel])
        return len(pairs), angle, dist, tool_rv

    n, angle, dist, _ = compare(a, ra, pa, b, rb, pb)
    assert n == 4
    assert angle.max() < 3.0 and 3.0 < dist.max() < 10.0  # repeatability
    n, angle, dist, rv = compare(b, rb, pb, c, rc, pc)
    assert n == 8
    assert 4.5 < angle.min() and angle.max() < 8.0
    assert rv.std(axis=0).max() < 1.5  # one constant tool-frame rotation
    n, angle, _, _ = compare(a, ra, pa, c, rc, pc)
    assert n == 4 and 6.5 < angle.min() and angle.max() < 8.0


def test_home_posture_is_in_every_run(frames):
    home = [f[JOINTS].iloc[0].to_numpy() for f in frames.values()]
    for q in home[1:]:
        assert np.abs(q - home[0]).max() < 1e-3


def test_registration_only_fit_sanity(frames):
    """Nominal joints, base 6D + tool 6D fitted on each run (world frame).

    Residual of 6D poses: position in metres and rotation (rotation vector)
    weighted 0.1 m/rad. A sanity check of the files, not an accuracy claim:
    no joint offsets are fitted, and the 15:29 run has 24 equations for 12
    parameters.
    """
    pin = pytest.importorskip("pinocchio")
    from scipy.optimize import least_squares

    model = pin.buildModelFromUrdf(
        str(ROOT / "examples/tiago/urdf/tiago_48_schunk.urdf")
    )
    data = model.createData()
    idx = [model.joints[model.getJointId(j)].idx_q for j in JOINTS]
    fid = model.getFrameId("wrist_ft_tool_link")
    expected = EXPECTED_FIT
    for sid, df in frames.items():
        rot, trans = [], []
        for q in df[JOINTS].to_numpy():
            config = pin.neutral(model)
            config[idx] = q
            pin.framesForwardKinematics(model, data, config)
            rot.append(data.oMf[fid].rotation.copy())
            trans.append(data.oMf[fid].translation.copy())
        rot, trans = np.array(rot), np.array(trans)
        target_R = Rotation.from_euler(
            "xyz", df[["phix1", "phiy1", "phiz1"]]
        ).as_matrix()
        target_p = df[["x1", "y1", "z1"]].to_numpy()
        weight = 0.1

        def residual(x):
            base = Rotation.from_rotvec(x[3:6]).as_matrix()
            tool = Rotation.from_rotvec(x[9:12]).as_matrix()
            model_R = base @ rot @ tool
            model_p = (base @ (rot @ x[6:9] + trans).T).T + x[:3]
            err = Rotation.from_matrix(
                np.einsum("nji,njk->nik", model_R, target_R)
            ).as_rotvec()
            return np.r_[(model_p - target_p).ravel(), weight * err.ravel()]

        best = min(
            (
                least_squares(
                    residual,
                    np.r_[target_p.mean(0) - trans.mean(0), 0, 0, yaw, np.zeros(6)],
                )
                for yaw in np.linspace(-np.pi, np.pi, 6, endpoint=False)
            ),
            key=lambda s: s.cost,
        )
        n = len(df)
        pos = best.fun[: 3 * n].reshape(-1, 3)
        ang = best.fun[3 * n :].reshape(-1, 3) / weight
        pos_rms = np.sqrt((pos**2).sum(1).mean()) * 1e3
        rot_rms = np.degrees(np.sqrt((ang**2).sum(1).mean()))
        assert pos_rms == pytest.approx(expected[sid][0], abs=0.05), sid
        assert rot_rms == pytest.approx(expected[sid][1], abs=0.05), sid


EXPECTED_FIT = {
    "2023-09-12-1529": (9.62, 3.69),
    "2023-09-12-1552": (14.89, 3.88),
    "2023-09-12-1658": (12.27, 3.53),
}
