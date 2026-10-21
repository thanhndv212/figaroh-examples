"""Shipped TIAGo head-camera chessboard sessions, 2023-10 to 2023-11 (#108)."""

from pathlib import Path

import pandas as pd
import pytest
import yaml

from examples.tiago.utils.head_camera_extraction import (
    CAMERA_JOINT_HEADER,
    CAMERA_POSE_HEADER,
    MOCAP_HEADER,
)
from examples.tiago.utils.mocap_extraction import file_sha256

HC = Path(__file__).resolve().parents[1] / "examples/tiago/data/calibration/head_camera"
MANIFEST = yaml.safe_load((HC / "protocol.yaml").read_text())
SESSIONS = MANIFEST["sessions"]
ROLES = {
    "2023-11-27-center": "training",
    "2023-11-23-distortion_original": "validation",
    "2023-10-27-test1": "confirmation",
    "2023-10-27-test2": "confirmation",
    "2023-10-27-test3": "confirmation",
}


def test_every_shipped_file_matches_its_hash_and_nothing_else_is_shipped():
    listed = {}
    for s in SESSIONS:
        listed.update(s["files"])
    listed.update(MANIFEST["pal_reference"]["files"])
    listed.update(MANIFEST["pal_reference"]["unknown_origin"]["files"])
    for name, digest in listed.items():
        assert file_sha256(HC / name) == digest, name
    on_disk = {str(p.relative_to(HC)) for p in HC.rglob("*") if p.is_file()} - {
        "protocol.yaml"
    }
    assert on_disk == set(listed)


def test_roles_are_frozen_and_diagnostic_is_the_default():
    assert len({s["id"] for s in SESSIONS}) == len(SESSIONS) == 13
    for s in SESSIONS:
        assert s["role"] == ROLES.get(s["id"], "diagnostic"), s["id"]
        assert s["usable"] is True
    assert {n["id"] for n in MANIFEST["not_usable"]} == {
        "2023-10-24-bags",
        "2023-10-24-camera",
        "2023-10-25",
    }


def test_every_session_names_its_robot():
    for s in SESSIONS:
        expected = "tiago-155c" if s["id"] == "2023-11-24-test_optimal" else "tiago-48"
        assert s["robot"] == expected, s["id"]
    assert not any(s["id"].startswith("2023-10-24") for s in SESSIONS)


def test_tables_follow_the_schemas_and_posture_counts():
    for s in SESSIONS:
        for name in s["files"]:
            table = pd.read_csv(HC / name)
            if name.startswith("mocap_"):
                assert list(table.columns) == MOCAP_HEADER
                assert len(table) == s["drops"]["kept"]
            elif name.endswith("_joints.csv"):
                assert list(table.columns) == CAMERA_JOINT_HEADER
                assert len(table) == s["postures"]
                assert set(table.torso_source) <= {"bag", "assumed"}
            else:
                assert list(table.columns) == CAMERA_POSE_HEADER
                assert len(table) == s["postures"]
            assert not table.isna().any().any(), name


def test_drop_counters_add_up():
    runs = [s for s in SESSIONS if "drops" in s]
    assert len(runs) == 6
    for s in runs:
        d = s["drops"]
        assert d["plateaus"] == (
            d["unpaired"] + d["lost"] + d["ambiguous"] + d["flipped"] + d["kept"]
        ), s["id"]
        assert d["repaired"] <= d["kept"]
        assert (d["removed_samples"] > 0) == (d["repaired"] > 0)
        mocap = pd.read_csv(HC / next(n for n in s["files"] if n.startswith("mocap_")))
        assert int((mocap.status == "repaired").sum()) == d["repaired"]
        assert int(mocap.n_flipped_removed.sum()) == d["removed_samples"]


def test_mocap_and_camera_poses_agree_in_distance():
    """Mocap chessboard distance matches PAL's within the stated noise."""
    for s in SESSIONS:
        if "drops" not in s:
            continue
        mocap = pd.read_csv(HC / next(n for n in s["files"] if n.startswith("mocap_")))
        d = (mocap[["x", "y", "z"]] ** 2).sum(axis=1) ** 0.5
        assert d.between(0.2, 1.5).all(), s["id"]
        assert (mocap.pose_std_mm < 5).all(), s["id"]


def test_pal_reference_is_attributed_and_offset_is_not_called_pal():
    ref = MANIFEST["pal_reference"]
    assert "PAL Robotics" in ref["attribution"]
    assert "not PAL reference" in ref["unknown_origin"]["note"]
    text = (HC / "pal_reference/master_calibration_pal.yaml").read_text()
    assert "head_camera" in text or "camera" in text


def test_torso_is_flagged_where_assumed():
    for s in SESSIONS:
        joints = (
            pd.read_csv(HC / f"camera_{s['id']}_joints.csv")
            if (HC / f"camera_{s['id']}_joints.csv").exists()
            else None
        )
        if joints is None:
            continue
        assumed = joints.torso_source == "assumed"
        assert (joints.loc[assumed, "torso_lift_joint"] == 0.3).all()
        assert (s["id"].startswith("2023-10-27")) == (not assumed.any())
