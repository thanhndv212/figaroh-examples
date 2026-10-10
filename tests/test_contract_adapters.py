"""Data contract from real adapters (examples#17, figaroh-plus#55).

One dynamic adapter (TIAGo identification) and the TIAGo mocap adapter
produce the contract types with explicit frames, units, sessions/roles,
source rows and measured/derived signals, and stay compatible with the
legacy inputs they replace.
"""

import hashlib
import shutil
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

pytest.importorskip("figaroh.data")

ROOT = Path(__file__).resolve().parents[1]
TIAGO = ROOT / "examples" / "tiago"
sys.path.insert(0, str(ROOT))

from figaroh.calibration.data_loader import load_data  # noqa: E402
from figaroh.data import (  # noqa: E402
    JOINT_FORCE,
    JOINT_TORQUE,
    PoseObservations,
    Protocol,
    TrajectoryData,
    file_sha256,
)


@pytest.fixture(scope="module")
def in_tiago():
    mp = pytest.MonkeyPatch()
    mp.chdir(TIAGO)
    yield
    mp.undo()


# ── dynamic: TIAGo identification ──


@pytest.fixture(scope="module")
def identification(in_tiago):
    from figaroh.tools.robot import load_robot

    from examples.tiago.identification import configure_identification
    from examples.tiago.utils.tiago_tools import TiagoIdentification

    robot = load_robot(
        "urdf/tiago_48_hey5.urdf", load_by_urdf=True, robot_pkg="tiago_description"
    )
    ident = TiagoIdentification(robot, "config/tiago_unified_config.yaml")
    configure_identification(ident)
    return ident


@pytest.fixture(scope="module")
def trajectory(identification):
    return identification.load_trajectory_data()


def test_dynamic_adapter_states_its_conventions(identification, trajectory):
    traj = trajectory
    assert isinstance(traj, TrajectoryData)
    assert list(traj.joint_names) == identification.identif_config["active_joints"]
    assert traj.clock == "recorded"
    assert 99 < 1 / np.median(np.diff(traj.t)) < 101  # ~100 Hz, D2 audit
    # measured vs derived: positions measured; velocities and accelerations
    # not provided, derived downstream (the logged velocity is filtered, #68)
    assert traj.origin["q"] == "measured"
    assert traj.dq is None
    assert traj.origin["dq"].startswith("absent; derived from the filtered positions")
    assert traj.origin["ddq"] == "absent"
    # units: the prismatic torso in N, the arm in N·m, from a raw signal
    assert traj.effort_kind == (JOINT_FORCE,) + (JOINT_TORQUE,) * 7
    assert traj.effort_unit == ("N",) + ("N·m",) * 7
    assert set(traj.effort_raw_kind) == {"motor_effort"}
    assert "kmotor" in traj.effort_conversion
    traj.check_effort(identification.model)


def test_dynamic_source_rows_and_files(identification, trajectory):
    traj = trajectory
    cfg = identification.identif_config
    files = [cfg["pos_data"], cfg["vel_data"], cfg["torque_data"]]
    rows = len(pd.read_csv(files[0]))
    # every file row is kept: no velocity shift, nothing dropped
    np.testing.assert_array_equal(traj.sample_index, np.arange(rows))
    for f in files:
        path = str(Path(f).resolve())
        assert traj.source.files[path] == file_sha256(path)
    assert traj.source.session.id == "training"


def test_dynamic_effort_matches_the_legacy_conversion(identification, trajectory):
    """Independent of the adapter: raw CSV × reduction × kmotor (+ m g)."""
    import pinocchio as pin

    cfg = identification.identif_config
    raw = pd.read_csv(cfg["torque_data"])
    n = trajectory.n_samples
    model, data = identification.model, identification.robot.data
    pin.computeSubtreeMasses(model, data)
    for i, joint in enumerate(cfg["active_joints"]):
        recorded = raw[f"- {joint}_effort"].to_numpy(float)[:n]
        expected = cfg["reduction_ratio"][joint] * cfg["kmotor"][joint] * recorded
        if joint == "torso_lift_joint":
            expected = expected + 9.81 * data.mass[model.getJointId(joint)]
        np.testing.assert_array_equal(trajectory.effort[:, i], expected)
        np.testing.assert_array_equal(trajectory.effort_raw[:, i], recorded)


def test_dynamic_pipeline_runs_on_the_contract(identification):
    identification.initialize(truncate=(921, 6791))
    assert identification.trajectory is not None
    stage = identification.stages[0]
    assert stage.stage == "data" and "TrajectoryData" in stage.reason
    # accelerations were derived by the pipeline (origin "absent" above)
    assert identification.processed_data["accelerations"] is not None


# ── geometric: TIAGo mocap ──


@pytest.fixture(scope="module")
def calibration(in_tiago):
    from figaroh.tools.robot import load_robot

    from examples.tiago.utils.tiago_tools import TiagoCalibration

    robot = load_robot(
        "urdf/tiago_48_hey5.urdf", load_by_urdf=True, robot_pkg="tiago_description"
    )
    return TiagoCalibration(robot, "config/tiago_unified_config.yaml", del_list=[])


@pytest.fixture(scope="module")
def sessions(calibration):
    from examples.tiago.utils.mocap_observations import protocol_observations

    return protocol_observations(calibration.model, calibration.calib_config)


def test_protocol_matches_the_frozen_heldout_table(sessions):
    sys.path.insert(0, str(ROOT / "tests"))
    from test_tiago_heldout_protocol import FROZEN

    from examples.tiago.utils.mocap_observations import PROTOCOL

    protocol = Protocol.load(PROTOCOL)
    table = {
        name: (s.role, sha) for s in protocol.sessions for name, sha in s.files.items()
    }
    assert table == {name: (role, sha) for name, (role, _, sha) in FROZEN.items()}
    roles = sorted(role for role, _ in sessions.values())
    assert roles == ["confirmation", "confirmation", "training", "validation"]


def test_mocap_adapter_states_its_conventions(sessions, calibration):
    sizes = {"training": [37], "validation": [62], "confirmation": [63, 59]}
    seen = {k: [] for k in sizes}
    for session_id, (role, obs) in sessions.items():
        assert isinstance(obs, PoseObservations)
        assert obs.point_names == ("BL", "BR", "TR", "TL")
        assert obs.frame == "qualisys:base_frame"
        assert obs.registered_to == calibration.calib_config["start_frame"]
        assert obs.measurability[:, :3].all() and not obs.measurability[:, 3:].any()
        assert set(obs.session) == {session_id}
        assert obs.source.session.date == session_id[:10]
        np.testing.assert_array_equal(obs.sample_index, np.arange(obs.n_samples))
        (path,) = obs.source.files
        assert (
            obs.source.files[path]
            == hashlib.sha256(Path(path).read_bytes()).hexdigest()
        )
        seen[role].append(obs.n_samples)
    assert {k: sorted(v) for k, v in seen.items()} == {
        k: sorted(v) for k, v in sizes.items()
    }


def test_mocap_point_bl_is_what_calibration_loads(sessions, calibration):
    """Legacy compatibility: point BL equals core's load_data on the file."""
    import copy

    from examples.tiago.utils.mocap_observations import MOCAP

    for session_id, (_, obs) in sessions.items():
        cfg = copy.deepcopy(calibration.calib_config)
        (path,) = obs.source.files
        pee, q = load_data(path, calibration.model, cfg, [])
        pee2, q2 = obs.select_points(["BL"]).to_legacy(
            calibration.model, calibration.calib_config
        )
        np.testing.assert_array_equal(pee2, pee)
        np.testing.assert_array_equal(q2, q)
    assert MOCAP.exists()


def test_a_changed_session_file_is_refused(tmp_path, calibration):
    from examples.tiago.utils.mocap_observations import (
        MOCAP,
        PROTOCOL,
        protocol_observations,
    )

    for f in MOCAP.glob("qualisys_*_static_postures.csv"):
        shutil.copy(f, tmp_path / f.name)
    shutil.copy(PROTOCOL, tmp_path / "protocol.yaml")
    target = tmp_path / "qualisys_2021-11-26_static_postures.csv"
    target.write_text(target.read_text().replace("0.", "0.0", 1))
    with pytest.raises(ValueError, match="2021-11-26-1105.*does not match"):
        protocol_observations(
            calibration.model, calibration.calib_config, tmp_path / "protocol.yaml"
        )
