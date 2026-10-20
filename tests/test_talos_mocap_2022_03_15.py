"""TALOS torso-arm motion-capture sessions of 2022-03-15 (#114).

Protocol v1 freezes three Qualisys sessions in the Qualisys world frame (three
bodies, see examples/talos/utils/mocap_extraction.py). These tests pin their
hashes, roles, schema, per-run clock lags, usability gate and audit figures
from the shipped files alone (no ``rosbags`` needed).
"""

from pathlib import Path

import numpy as np
import pandas as pd
import pytest
import yaml

from examples.talos.utils.mocap_extraction import BODIES, columns, joint_names

ROOT = Path(__file__).resolve().parents[1]
MOCAP = ROOT / "examples/talos/data/calibration/mocap"
# id: (file, arm, role, rows, lag, sha256)
SESSIONS = {
    "2022-03-15-1354-left": (
        "qualisys_2022-03-15-1354_left_static_postures.csv",
        "left",
        "training",
        56,
        -0.08,
        "4bfa50c2219acf776c18d484433553ab5283ba0ecc29a6bf053584a314ae92ef",
    ),
    "2022-03-15-1418-right": (
        "qualisys_2022-03-15-1418_right_static_postures.csv",
        "right",
        "validation",
        39,
        0.02,
        "8b1d0c5e3b4e5befc58b826c3e0376124e01864f5609503b075d724e81dae5ea",
    ),
    "2022-03-15-1439-right": (
        "qualisys_2022-03-15-1439_right_static_postures.csv",
        "right",
        "confirmation",
        23,
        -0.02,
        "c6cc469bd21da32e5c4bdbb901131770f23e5e85267a549700573eb24e2f1989",
    ),
}
# recorded 2022-03-15 (UTC), between 12:00 and 15:00
DAY = (1647302400, 1647388800)


@pytest.fixture(scope="module")
def manifest():
    return yaml.safe_load((MOCAP / "protocol.yaml").read_text())


@pytest.fixture(scope="module")
def frames():
    return {sid: pd.read_csv(MOCAP / v[0]) for sid, v in SESSIONS.items()}


def test_protocol_freezes_three_roles_and_hashes(manifest):
    from figaroh.data import Protocol

    assert (manifest["name"], manifest["version"]) == ("talos-mocap-2022-03-15", 1)
    assert "heldout" not in manifest  # the study adds it in v2
    protocol = Protocol.load(MOCAP / "protocol.yaml")
    protocol.verify(root=MOCAP)
    assert len(protocol.sessions) == 3
    for sid, v in SESSIONS.items():
        session = protocol.session(sid)
        assert session.role == v[2]
        assert session.files == {v[0]: v[5]}
    assert [s.id for s in protocol.with_role("training")] == ["2022-03-15-1354-left"]
    assert [s.id for s in protocol.with_role("validation")] == ["2022-03-15-1418-right"]


def test_sources_are_named_by_sha256_and_not_distributed(manifest):
    raw = {s["id"]: s for s in manifest["sessions"]}
    for sid, v in SESSIONS.items():
        entry = raw[sid]
        source = entry["source"]
        assert "/" not in source["bag"] and source["bag"].endswith(".bag")
        assert len(source["sha256"]) == 64
        assert source["lag_s"] == v[4]
        assert entry["frame"] == "qualisys:world" and entry["arm"] == v[1]
    assert not list(MOCAP.rglob("*.bag"))
    assert not list(MOCAP.rglob("introspection*")) and not list(MOCAP.rglob("tf_*"))
    # a bag per session, all distinct
    assert len({e["source"]["sha256"] for e in raw.values()}) == 3
    # attachment links are not known: never recorded as a guess
    assert {b["attached_to"] for b in manifest["bodies"].values()} == {"unknown"}
    assert set(manifest["bodies"]) == set(BODIES)


def test_usability_gate_and_audit_figures(manifest):
    """Every shipped session keeps at least 15 plateaus; the audit is consistent."""
    for entry in manifest["sessions"]:
        rows = SESSIONS[entry["id"]][3]
        drops = entry["drops"]
        assert entry["usable"] is True and rows >= 15
        assert rows == drops["plateaus"] - drops["too_few_samples"] - drops["unsteady"]
        audit = entry["audit"]
        assert audit["robot"]["rate_hz"] == 62.5
        for body in BODIES:
            assert audit[body]["frozen"] == 0 and audit[body]["nan"] == 0
            assert audit[body]["tracking_loss_gaps"] >= 0
        # the end-effector body is the sparse one (about 1 Hz)
        assert audit["endeffector"]["rate_hz"] < 2 < audit["upper_body"]["rate_hz"]
        assert audit["upper_body"]["rate_hz"] < audit["waist"]["rate_hz"]
        # joints are sampled well inside every run's duration
        assert audit["robot"]["duration_s"] > 480


@pytest.mark.parametrize("sid", SESSIONS)
def test_schema_counts_and_clocks(sid, frames):
    _, arm, _, rows, *_ = SESSIONS[sid]
    df = frames[sid]
    assert list(df.columns) == columns(arm)
    assert len(df) == rows
    assert np.isfinite(df.to_numpy()).all()
    for body in BODIES:
        assert (df[f"n_{body}"] >= {"endeffector": 3}.get(body, 20)).all()
        assert df[f"{body}_std_mm"].max() < 5.0
        quat = df[[f"{body}_q{c}" for c in "xyzw"]].to_numpy()
        np.testing.assert_allclose(np.linalg.norm(quat, axis=1), 1.0, atol=1e-9)
        assert (quat[:, 3] >= 0).all()
        # the lab is a few metres wide
        assert df[[f"{body}_{c}" for c in "xyz"]].abs().max().max() < 3.0
    dur = df.t_end_robot - df.t_start_robot
    assert (dur >= 6.0).all()
    # plateaus increase in time, do not overlap, and are indexed
    assert (df.t_start_robot.to_numpy()[1:] > df.t_end_robot.to_numpy()[:-1]).all()
    assert df.plateau.is_unique and df.plateau.is_monotonic_increasing
    assert DAY[0] < df.t_start_robot.min() and df.t_end_robot.max() < DAY[1]
    assert df[joint_names(arm)].abs().max().max() < 3.2


@pytest.mark.parametrize("sid", SESSIONS)
def test_recorded_arm_and_torso_move(sid, frames):
    """Each run moves its arm and both torso joints through wide ranges."""
    df = frames[sid]
    arm = SESSIONS[sid][1]
    assert df[f"arm_{arm}_5_joint"].std() > 0.5
    assert df[joint_names(arm)[7:]].std().min() > 0.2


def test_runs_do_not_repeat_each_other(frames):
    """The two right-arm runs hold different postures (no pair within 5 mrad)."""
    cols = joint_names("right")
    a = frames["2022-03-15-1418-right"][cols].to_numpy()
    b = frames["2022-03-15-1439-right"][cols].to_numpy()
    matched = sum(np.abs(a - row).max(axis=1).min() < 5e-3 for row in b)
    assert matched == 0


def test_waist_and_upper_body_move_with_the_torso(frames):
    """No body is a fixed base: waist and upper body poses spread by decimetres."""
    for df in frames.values():
        for body in ("waist", "upper_body"):
            spread = (
                df[[f"{body}_x", f"{body}_y"]].max()
                - df[[f"{body}_x", f"{body}_y"]].min()
            )
            assert spread.max() > 0.1
