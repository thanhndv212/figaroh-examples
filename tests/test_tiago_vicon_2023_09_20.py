"""TIAGo Vicon 2023-09-20 static-posture runs (#106).

Protocol v1 freezes two confirmation sessions in the Vicon world frame (marker
triads, see examples/tiago/utils/vicon_extraction.py). These tests pin their
hashes, roles, schema, per-run clock lags, marker-triad definitions and a
registration-only sanity fit, from the shipped files alone (no ``c3d`` needed).
"""

from pathlib import Path

import numpy as np
import pandas as pd
import pytest
import yaml
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
VICON = ROOT / "examples/tiago/data/calibration/vicon"
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
    "2023-09-20-1508": (
        "vicon_2023-09-20-1508_static_postures.csv",
        17,
        0,
        1,
        17,
        0,
        "745b6852eb27907a07bfd26e3cd30fc83cd69d4de33d5719a688bb8b01ebf74c",
    ),
    "2023-09-20-1607": (
        "vicon_2023-09-20-1607_static_postures.csv",
        21,
        0,
        2,
        21,
        0,
        "18f213f8fdc51a267aa4b1dc6a8002d3bbc9186eb62e6a9a39e4f8a444595c62",
    ),
}


@pytest.fixture(scope="module")
def manifest():
    return yaml.safe_load((VICON / "protocol.yaml").read_text())


@pytest.fixture(scope="module")
def frames():
    return {sid: pd.read_csv(VICON / v[0]) for sid, v in SESSIONS.items()}


def test_protocol_lists_two_confirmation_sessions(manifest):
    from figaroh.data import Protocol

    assert (manifest["name"], manifest["version"]) == ("tiago-vicon-2023-09-20", 1)
    assert "heldout" not in manifest  # #100 adds it in v2
    protocol = Protocol.load(VICON / "protocol.yaml")
    protocol.verify(root=VICON)
    assert len(protocol.sessions) == 2
    for sid, v in SESSIONS.items():
        session = protocol.session(sid)
        assert session.role == "confirmation"
        assert session.files == {v[0]: v[6]}


def test_sources_are_named_by_sha256_and_not_distributed(manifest):
    raw = {s["id"]: s for s in manifest["sessions"]}
    for sid, v in SESSIONS.items():
        entry = raw[sid]
        source = entry["source"]
        for key in ("mocap", "bag"):
            assert "/" not in source[key]
        assert source["mocap"].endswith((".c3d", ".csv"))
        assert source["bag"].endswith(".bag")
        assert len(source["mocap_sha256"]) == 64 and len(source["sha256"]) == 64
        assert entry["frame"] == "vicon:world"
        assert entry["rigid_body"] == v[3]
        assert v[3] in manifest["rigid_bodies"]
        assert entry["drops"] == {"plateaus": v[4], "gripper_lost": v[5]}
        assert v[4] == v[1] + v[5]
    # the csv wall clock is read as UTC+2: the residual lag is under a second
    assert abs(raw["2023-09-20-1607"]["source"]["lag_s"]) < 1.0
    # the c3d has no absolute clock: its lag is the origin offset
    assert "no absolute clock" in raw["2023-09-20-1508"]["source"]["clock"]
    assert not list(VICON.rglob("*.bag")) and not list(VICON.rglob("*.c3d"))
    assert not (VICON / "tiago_calib_vicon_1607.csv").exists()


def test_unshipped_recordings_are_recorded_with_a_reason(manifest):
    skipped = {e["recording"]: e for e in manifest["not_shipped"]}
    assert set(skipped) == {
        "calib_mocap_vicon_2023-09-20-13-53-01.bag.active",
        "vicon_postures_2023-09-20-15-22-11.bag",
        "vicon_postures_2023-09-20-15-34-35.bag",
    }
    for entry in skipped.values():
        assert len(entry["sha256"]) == 64 and entry["reason"]
    assert (
        "reindex"
        in skipped["calib_mocap_vicon_2023-09-20-13-53-01.bag.active"]["reason"]
    )


def raw_sessions(manifest):
    return {s["id"]: s for s in manifest["sessions"]}


def test_triad_definitions_differ_on_the_gripper_only(manifest):
    bodies = manifest["rigid_bodies"]
    grip1, grip2 = (np.array(bodies[k]["gripper_sides_mm"]) for k in (1, 2))
    base1, base2 = (np.array(bodies[k]["base_sides_mm"]) for k in (1, 2))
    assert np.abs(base1 - base2).max() < 2.0  # the base plate did not change
    assert np.abs(np.sort(grip1) - np.sort(grip2)).max() > 10.0
    assert raw_sessions(manifest)["2023-09-20-1607"]["relabelled_frames"]["gripper"] > 0
    assert raw_sessions(manifest)["2023-09-20-1508"]["relabelled_frames"] == {
        "gripper": 0,
        "base": 0,
    }
    assert bodies[1]["used_by"] == ["2023-09-20-1508"]
    assert bodies[2]["used_by"] == ["2023-09-20-1607"]


@pytest.mark.parametrize("sid", SESSIONS)
def test_schema_counts_and_clocks(sid, frames):
    _, rows, no_base, body, *_ = SESSIONS[sid]
    df = frames[sid]
    assert list(df.columns) == COLUMNS
    assert len(df) == rows
    base_cols = [f"{c}b" for c in POSE]
    assert df.notna().all().all() == (no_base == 0)
    assert np.isfinite(df.drop(columns=base_cols).to_numpy()).all()
    assert df[base_cols].isna().all(axis=1).sum() == no_base
    assert (df.points_in_base_frame == 0).all()
    assert (df.rigid_body_def == body).all()
    assert (df.n_live >= 20).all() and (df.n_live <= df.n_window).all()
    assert (df.n_base_live >= 20).all()
    assert df.pose_std_mm.max() < 1.0 and df.pose_std_deg.max() < 1.0
    dur = df.t_end_robot - df.t_start_robot
    assert (dur >= 1.2).all()
    # windows increase in time and do not overlap
    assert (df.t_start_robot.to_numpy()[1:] > df.t_end_robot.to_numpy()[:-1]).all()
    # recorded on 2023-09-20 (UTC)
    assert 1695168000 < df.t_start_robot.min() and df.t_end_robot.max() < 1695254400
    assert df[JOINTS].abs().max().max() < 3.2


@pytest.mark.parametrize("sid", SESSIONS)
def test_base_triad_is_fixed_during_the_run(sid, frames):
    """The robot base does not move: one base pose, to within marker noise."""
    df = frames[sid]
    assert df[["xb", "yb", "zb"]].std().max() < 4e-4  # m
    assert df[["phixb", "phiyb", "phizb"]].std().max() < 2e-3  # rad
    # the base plate stands level on the floor plane, about 0.31 m up
    assert df.zb.between(0.30, 0.32).all()


def test_home_posture_is_in_every_run(frames):
    """The first plateau of both runs is the same script posture."""
    a, b = (f[JOINTS].iloc[3].to_numpy() for f in frames.values())
    assert np.abs(a - b).max() < 1e-3


def test_runs_repeat_the_same_postures(frames):
    """The 1607 run replays 15 of the 17 postures of the 1508 run (joints within 5 mrad)."""
    a, b = (frames[s][JOINTS].to_numpy() for s in SESSIONS)
    matched = sum(np.abs(b - row).max(axis=1).min() < 5e-3 for row in a)
    assert matched == 15


def _fit(df, model, data, idx, fid, pin):
    from scipy.optimize import least_squares

    rot, trans = [], []
    for q in df[JOINTS].to_numpy():
        config = pin.neutral(model)
        config[idx] = q
        pin.framesForwardKinematics(model, data, config)
        rot.append(data.oMf[fid].rotation.copy())
        trans.append(data.oMf[fid].translation.copy())
    rot, trans = np.array(rot), np.array(trans)
    target_R = Rotation.from_euler("xyz", df[["phix1", "phiy1", "phiz1"]]).as_matrix()
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
    return (
        np.sqrt((pos**2).sum(1).mean()) * 1e3,
        np.degrees(np.sqrt((ang**2).sum(1).mean())),
    )


def test_registration_only_fit_sanity(frames):
    """Nominal joints, base 6D + tool 6D fitted on each run (world frame).

    Residual of 6D poses: position in millimetres and rotation (rotation
    vector) weighted 0.1 m/rad. A sanity check of the files, not an accuracy
    claim: no joint offsets are fitted.
    """
    pin = pytest.importorskip("pinocchio")
    model = pin.buildModelFromUrdf(
        str(ROOT / "examples/tiago/urdf/tiago_48_schunk.urdf")
    )
    data = model.createData()
    idx = [model.joints[model.getJointId(j)].idx_q for j in JOINTS]
    fid = model.getFrameId("wrist_ft_tool_link")
    for sid, df in frames.items():
        pos_rms, rot_rms = _fit(df, model, data, idx, fid, pin)
        assert pos_rms == pytest.approx(EXPECTED_FIT[sid][0], abs=0.05), sid
        assert rot_rms == pytest.approx(EXPECTED_FIT[sid][1], abs=0.05), sid


EXPECTED_FIT = {
    "2023-09-20-1508": (19.13, 5.83),
    "2023-09-20-1607": (20.25, 5.89),
}
