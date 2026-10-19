"""TIAGo Qualisys 2021-10-26 static-posture sessions (#104).

Protocol v2 adds two confirmation sessions in the Qualisys world frame. These
tests pin their hashes, roles, schema and posture counts, and check that the
base-frame protocol (v1 results) does not see them.
"""

import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
MOCAP = ROOT / "examples/tiago/data/calibration/mocap"
SESSIONS = {
    "2021-10-26-1051": (
        "qualisys_2021-10-26-1051_static_postures.csv",
        26,
        "db245498e1bdc023d84d15ccfe5e7ff80b447d47c44d23bef5e0f8b3e5fc2026",
    ),
    "2021-10-26-1102": (
        "qualisys_2021-10-26-1102_static_postures.csv",
        25,
        "94685056aac421aa511e1d56ab4fbdd9726904a3bbaab3d70db7897fae7b39cc",
    ),
}
JOINTS = ["torso_lift_joint"] + [f"arm_{i}_joint" for i in range(1, 8)]
COLUMNS = (
    [f"{a}{k}" for k in range(1, 5) for a in "xyz"]
    + JOINTS
    + ["t_start_robot", "t_end_robot", "marker_std_mm", "points_in_base_frame"]
)


@pytest.fixture(scope="module")
def manifest():
    return yaml.safe_load((MOCAP / "protocol.yaml").read_text())


def test_protocol_v2_lists_the_sessions_as_confirmation(manifest):
    from figaroh.data import Protocol

    assert manifest["version"] == 2
    protocol = Protocol.load(MOCAP / "protocol.yaml")
    protocol.verify(root=MOCAP)  # every file, v1 and v2, matches its sha256
    for sid, (name, _, sha) in SESSIONS.items():
        session = protocol.session(sid)
        assert session.role == "confirmation"
        assert session.files == {name: sha}
    # v1 roles are unchanged
    assert [s.id for s in protocol.with_role("training")] == ["2021-11-30-1544"]
    assert [s.id for s in protocol.with_role("validation")] == ["2021-11-26-1105"]
    assert len(protocol.sessions) == 6


def test_sources_are_named_by_sha256_and_not_distributed(manifest):
    raw = {s["id"]: s for s in manifest["sessions"]}
    for sid in SESSIONS:
        source = raw[sid]["source"]
        assert len(source["sha256"]) == 64
        assert source["bag"].endswith(".bag")
        assert "/" not in source["bag"]  # a name, not a path
        assert raw[sid]["frame"] == "qualisys:world"
    assert not list(MOCAP.rglob("*.bag"))


@pytest.mark.parametrize("sid", SESSIONS)
def test_schema_and_posture_counts(sid):
    name, rows, _ = SESSIONS[sid]
    df = pd.read_csv(MOCAP / name)
    assert list(df.columns) == COLUMNS
    assert len(df) == rows
    assert np.isfinite(df.to_numpy()).all()
    assert (df.points_in_base_frame == 0).all()  # world frame, base not in view
    assert (df.t_end_robot - df.t_start_robot >= 1.2).all()
    assert df.marker_std_mm.max() < 1.0
    # the 2021-10-26 plan, executed twice: the same postures in both sessions
    assert df[JOINTS].abs().max().max() < 3.2


def test_both_sessions_share_one_rigid_body_and_postures():
    frames = {sid: pd.read_csv(MOCAP / name) for sid, (name, _, _) in SESSIONS.items()}

    def distances(df):
        pts = [df[[f"x{k}", f"y{k}", f"z{k}"]].to_numpy() for k in range(1, 5)]
        return np.array(
            [
                np.linalg.norm(pts[a] - pts[b], axis=1).mean()
                for a in range(4)
                for b in range(a + 1, 4)
            ]
        )

    a, b = (distances(f) for f in frames.values())
    np.testing.assert_allclose(a, b, atol=1e-4)
    # the same plan was run twice: 24 postures agree within 2 mrad on every joint
    first, second = (f[JOINTS].to_numpy() for f in frames.values())
    nearest = np.abs(second[:, None] - first[None]).max(-1).min(1)
    assert (nearest < 2e-3).sum() == 24


def test_base_frame_protocol_does_not_see_the_world_frame_sessions():
    sys.path.insert(0, str(ROOT))
    from examples.calibration_harness.heldout import load_plan

    plan = load_plan(MOCAP / "protocol.yaml")
    names = [s.name for s in (*plan.folds[0].training, *plan.folds[0].evaluated)]
    assert len(names) == 4
    assert not any("2021-10-26" in n for n in names)


def test_not_evaluated_must_name_known_sessions(manifest):
    from figaroh.data import Protocol

    from examples.calibration_harness.protocol import parse_heldout

    protocol = Protocol.load(MOCAP / "protocol.yaml")
    spec = parse_heldout(manifest["heldout"], protocol)
    assert spec.not_evaluated == tuple(SESSIONS)
    bad = dict(manifest["heldout"], not_evaluated=["2099-01-01-0000"])
    with pytest.raises(KeyError):
        parse_heldout(bad, protocol)


def test_registration_only_fit_sanity():
    """Nominal joints, base 6D + one tool point 3D: the files are consistent.

    Same model as the v1 "registration only" row (3.37 mm on the training
    session). The shipped sessions have no fitted joint offsets here, so the
    residual is larger than the 3.4 mm of the offline fits that include them.
    """
    pin = pytest.importorskip("pinocchio")
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation

    model = pin.buildModelFromUrdf(
        str(ROOT / "examples/tiago/urdf/tiago_48_schunk.urdf")
    )
    data = model.createData()
    idx = [model.joints[model.getJointId(j)].idx_q for j in JOINTS]
    fid = model.getFrameId("wrist_ft_tool_link")
    expected = {"2021-10-26-1051": 4.93, "2021-10-26-1102": 5.14}
    for sid, (name, _, _) in SESSIONS.items():
        df = pd.read_csv(MOCAP / name)
        rot, trans = [], []
        for q in df[JOINTS].to_numpy():
            config = pin.neutral(model)
            config[idx] = q
            pin.framesForwardKinematics(model, data, config)
            rot.append(data.oMf[fid].rotation.copy())
            trans.append(data.oMf[fid].translation.copy())
        rot, trans = np.array(rot), np.array(trans)
        target = df[["x1", "y1", "z1"]].to_numpy()

        def residual(x):
            base = Rotation.from_rotvec(x[3:6]).as_matrix()
            local = np.einsum("nij,j->ni", rot, x[6:9]) + trans
            return ((base @ local.T).T + x[:3] - target).ravel()

        best = min(
            (
                least_squares(
                    residual,
                    np.r_[target.mean(0) - trans.mean(0), 0, 0, yaw, np.zeros(3)],
                )
                for yaw in np.linspace(-np.pi, np.pi, 6, endpoint=False)
            ),
            key=lambda s: s.cost,
        )
        rms = np.sqrt((best.fun.reshape(-1, 3) ** 2).sum(1).mean()) * 1e3
        assert rms == pytest.approx(expected[sid], abs=0.05), sid
