"""TIAGo square-motion validation runs (#107).

Protocol v1 freezes seven validation sessions (Qualisys, 2021-12-16 and
2022-01-12). These tests pin hashes, roles, schema, the one-clock layout, the
offset sets and a registration-only sanity check, from the shipped files alone
(no bag needed).
"""

from pathlib import Path

import numpy as np
import pandas as pd
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
SQUARE = ROOT / "examples/tiago/data/validation/square_motion"
URDF = ROOT / "examples/tiago/urdf/tiago_48_schunk.urdf"
JOINTS = ["torso_lift_joint"] + [f"arm_{i}_joint" for i in range(1, 8)]
MARKER_COLUMNS = ["t", "body", "x", "y", "z", "qx", "qy", "qz", "qw"]
# id: (offset set, end-effector body, lag s)
RUNS = {
    "2021-12-16-15-04-54_zero": ("zero", "eeframe_BL", 5.770),
    "2021-12-16-15-07-04_zero": ("zero", "eeframe_BL", 5.775),
    "2021-12-16-15-38-03_mocap": ("mocap", "eeframe_BL", 6.040),
    "2022-01-12-10-55-17_zero": ("zero", "endeffector_frame", 7.730),
    "2022-01-12-11-25-38_mocap": ("mocap", "endeffector_frame", 8.105),
    "2022-01-12-11-10-58_mocap_inv": ("mocap_inv", "endeffector_frame", 7.915),
    "2022-01-12-11-38-43_pal": ("pal", "endeffector_frame", 8.100),
}


@pytest.fixture(scope="module")
def manifest():
    return yaml.safe_load((SQUARE / "protocol.yaml").read_text())


def test_protocol_lists_seven_validation_sessions(manifest):
    from figaroh.data import Protocol

    assert (manifest["name"], manifest["version"]) == (
        "tiago-square-motion-validation",
        1,
    )
    protocol = Protocol.load(SQUARE / "protocol.yaml")
    protocol.verify(root=SQUARE)
    assert {s.id for s in protocol.sessions} == set(RUNS)
    assert {s.role for s in protocol.sessions} == {"validation"}


def test_sources_are_named_by_sha256_and_not_distributed(manifest):
    for entry in manifest["sessions"]:
        source = entry["source"]
        assert source["bag"].endswith(".bag")
        assert len(source["sha256"]) == 64
        assert source["description"] == "original recording, not distributed"
    for item in manifest["not_shipped"]:
        assert len(item["sha256"]) == 64 and item["reason"]
    assert len(manifest["not_shipped"]) == 7 + 13


def test_offset_sets_are_recorded_with_values_or_unknown(manifest):
    sets = manifest["offset_sets"]
    assert set(sets) == {"zero", "mocap", "mocap_inv", "pal"}
    assert sets["pal"]["values"] == "unknown"
    mocap, inv = sets["mocap"]["values"], sets["mocap_inv"]["values"]
    assert len(mocap) == len(inv) == 7
    for joint, value in mocap.items():
        got = np.r_[value["translation_m"], value["rotation_rad"]]
        neg = np.r_[inv[joint]["translation_m"], inv[joint]["rotation_rad"]]
        assert np.abs(got + neg).max() < 1e-3
    for entry in manifest["sessions"]:
        assert entry["offset_set"] == RUNS[entry["id"]][0]


@pytest.mark.parametrize("run", sorted(RUNS))
def test_files_share_one_clock_and_schema(run, manifest):
    entry = next(s for s in manifest["sessions"] if s["id"] == run)
    _, body, lag = RUNS[run]
    assert entry["end_effector"] == body
    assert entry["source"]["lag_s"] == pytest.approx(lag, abs=1e-9)
    assert entry["source"]["lag_correlation"] > 0.95
    joints = pd.read_csv(SQUARE / f"{run}_joints.csv")
    markers = pd.read_csv(SQUARE / f"{run}_markers.csv")
    assert list(joints.columns) == ["t"] + JOINTS
    assert list(markers.columns) == MARKER_COLUMNS
    assert (entry["rows"]["joints"], entry["rows"]["markers"]) == (
        len(joints),
        len(markers),
    )
    assert joints["t"].iloc[0] == 0.0
    assert (np.diff(joints["t"]) > 0).all()
    assert (np.diff(markers["t"]) >= 0).all()
    assert {"base_frame", body} <= set(markers["body"])
    # markers sit on the robot clock: they overlap the joint recording
    ee = markers[markers["body"] == body]
    assert ee["t"].min() < joints["t"].max() and ee["t"].max() > joints["t"].min()
    quat = markers[["qx", "qy", "qz", "qw"]].to_numpy()
    assert np.abs(np.linalg.norm(quat, axis=1) - 1).max() < 1e-3


@pytest.mark.parametrize("run", sorted(RUNS))
def test_nominal_registration_residual_matches_protocol(run, manifest):
    pytest.importorskip("pinocchio")
    from examples.tiago.utils.square_motion_extraction import registration_rmse

    entry = next(s for s in manifest["sessions"] if s["id"] == run)
    joints = pd.read_csv(SQUARE / f"{run}_joints.csv")
    markers = pd.read_csv(SQUARE / f"{run}_markers.csv")
    ee = markers[markers["body"] == RUNS[run][1]]
    rmse = registration_rmse(
        joints["t"].to_numpy(),
        joints[JOINTS].to_numpy(),
        ee["t"].to_numpy(),
        ee[["x", "y", "z"]].to_numpy(),
        str(URDF),
    )
    assert rmse < 6.0
    # CSV rounding moves the fit by a few micrometres only
    assert rmse == pytest.approx(entry["nominal_registration_rmse_mm"], abs=0.05)
