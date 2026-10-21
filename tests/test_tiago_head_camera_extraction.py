"""Extraction rules for the TIAGo head-camera chessboard sessions (#108).

The original recordings are not distributed; the synthetic cases pin the rules
in examples/tiago/utils/head_camera_extraction.py that produced the shipped
files. The slow test re-creates every shipped file when TIAGO_RECORDINGS_ROOT
points at the original recordings.
"""

import os
from io import StringIO
from pathlib import Path

import numpy as np
import pandas as pd
import pytest
import yaml
from scipy.spatial.transform import Rotation

from examples.tiago.utils.head_camera_extraction import (
    CAMERA_JOINT_HEADER,
    CAMERA_POSE_HEADER,
    MOCAP_HEADER,
    camera_files,
    extract_mocap_run,
    extract_run,
    pair_plateaus,
    paired_torso,
    parse_camera_csv,
)
from examples.tiago.utils.mocap_extraction import file_sha256

RATE = 100.0
HEADER = (
    "x1,y1,z1,phix1,phiy1,phiz1,arm_1_joint,arm_2_joint,arm_3_joint,arm_4_joint,"
    "arm_5_joint,arm_6_joint,arm_7_joint,head_1_joint,head_2_joint,\n"
)


def test_camera_files_keep_the_value_tokens_verbatim():
    text = (
        HEADER
        + "-0.0674158,-0.0527256,0.433775,2.57052,0.357638,-0.156976,"
        + (
            "1.36988,0.248528,-2.18558,1.79153,0.766358,1.06135,-1.93865,0.360229,0.258735,\n"
        )
    )
    pose, joints = camera_files(text)
    p, j = pose.splitlines(), joints.splitlines()
    assert p[0].split(",") == CAMERA_POSE_HEADER
    assert j[0].split(",") == CAMERA_JOINT_HEADER
    assert p[1] == "0,-0.0674158,-0.0527256,0.433775,2.57052,0.357638,-0.156976"
    assert j[1].startswith("0,0.30,1.36988,0.248528,-2.18558,")
    assert j[1].endswith(",0.360229,0.258735,assumed")
    pose, joints = camera_files(text, ["0.296123"])
    assert joints.splitlines()[1].startswith("0,0.296123,1.36988,")
    assert joints.splitlines()[1].endswith(",bag")


def test_camera_files_refuse_a_foreign_header():
    with pytest.raises(ValueError, match="header"):
        camera_files("a,b,c,\n1,2,3,\n")
    header, rows = parse_camera_csv(HEADER + "1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,\n")
    assert len(header) == 15 and len(rows[0]) == 15


def test_pair_plateaus_is_one_to_one_and_thresholded():
    pal = np.array([[0.0] * 9, [1.0] * 9, [2.0] * 9])
    plateaus = np.array([[1.002] * 9, [0.001] * 9, [5.0] * 9])
    assert pair_plateaus(plateaus, pal) == {0: 1, 1: 0}


def _scene():
    """Five 4 s postures at 100 Hz; the board orientation differs per posture."""
    rng = np.random.default_rng(0)
    postures = rng.uniform(-1, 1, (5, 9))
    segs = []
    for k, p in enumerate(postures):
        segs.append(np.tile(p, (int(4 * RATE), 1)))
        if k + 1 < len(postures):
            ramp = np.linspace(0, 1, int(2 * RATE))[:, None]
            segs.append(p + ramp * (postures[k + 1] - p))
    q = np.vstack(segs)
    t = np.arange(len(q)) / RATE
    lag = -1.3
    R_cam = Rotation.from_euler("xyz", [0.2, -0.1, 0.4])
    p_cam = np.array([0.5, 0.2, 1.0])
    X = Rotation.from_euler("xyz", [np.pi, 0.0, 0.0])  # PAL board to mocap body
    pal_rot, board_rel = [], []
    for k in range(5):
        r = Rotation.from_euler("xyz", [0.1 * k, 0.2, -0.1 * k])
        pal_rot.append(r)
        board_rel.append(r * X)
    pal_pose = np.array(
        [np.r_[0.1, 0.0, 0.5, r.as_euler("xyz")] for r in pal_rot], dtype=float
    )

    def stream(rot_for_sample):
        n = len(t)
        quat = np.zeros((n, 4))
        pos = np.zeros((n, 3))
        for i in range(n):
            k = min(int(t[i] // 6.0), 4)
            R = rot_for_sample(i, k)
            quat[i] = (R_cam * R).as_quat()
            pos[i] = p_cam + R_cam.apply([0.1, 0.0, 0.5])
        pose = np.column_stack([pos, quat])
        pose[:, 0] += 1e-9 * (-1.0) ** np.arange(n)
        return np.column_stack([t - lag, t, pose])

    cam_q = np.tile(R_cam.as_quat(), (len(t), 1))
    camera = np.column_stack(
        [
            t - lag,
            t,
            np.tile(p_cam, (len(t), 1)) + 1e-9 * (-1.0) ** np.arange(len(t))[:, None],
            cam_q,
        ]
    )
    return t, q, postures, camera, stream, board_rel, pal_pose, lag


def _rel(rotations):
    return lambda i, k: rotations[k]


def test_clean_run_keeps_every_plateau_and_writes_the_schema():
    t, q, postures, camera, stream, board_rel, pal_pose, lag = _scene()
    board = stream(_rel(board_rel))
    text, drops = extract_mocap_run(
        t, q[:, :], np.full(len(t), 0.3), camera, board, lag, pal_pose, postures
    )
    table = pd.read_csv(StringIO(text))
    assert list(table.columns) == MOCAP_HEADER
    assert drops["plateaus"] == 5 and drops["kept"] == 5
    assert drops["flipped"] == drops["repaired"] == drops["unpaired"] == 0
    assert (table.status == "ok").all()
    np.testing.assert_allclose(table.torso_lift_joint, 0.3)
    np.testing.assert_allclose(
        table[["x", "y", "z"]].to_numpy(), np.tile([0.1, 0.0, 0.5], (5, 1)), atol=1e-6
    )


def test_flipped_samples_are_repaired_and_flipped_plateaus_dropped():
    t, q, postures, camera, stream, board_rel, pal_pose, lag = _scene()
    flip = Rotation.from_euler("xyz", [np.pi, 0, 0])
    bad = list(board_rel)
    # posture 3: the whole plateau reports the mirror solution
    bad[3] = board_rel[3] * flip
    # posture 1: 20 % of the samples flip inside the plateau
    state = {"n": 0}

    def rot(i, k):
        if k == 1 and (i % 5 == 0):
            return board_rel[1] * flip
        return bad[k]

    board = stream(rot)
    text, drops = extract_mocap_run(
        t, q, np.full(len(t), 0.3), camera, board, lag, pal_pose, postures
    )
    table = pd.read_csv(StringIO(text))
    assert drops["flipped"] == 1 and drops["repaired"] == 1
    assert drops["removed_samples"] > 0 and drops["kept"] == 4
    assert set(table.posture) == {0, 1, 2, 4}
    assert table.loc[table.posture == 1, "status"].item() == "repaired"
    np.testing.assert_allclose(
        table[["x", "y", "z"]].to_numpy(), np.tile([0.1, 0.0, 0.5], (4, 1)), atol=1e-6
    )


def test_unpaired_and_lost_plateaus_are_counted():
    t, q, postures, camera, stream, board_rel, pal_pose, lag = _scene()
    board = stream(_rel(board_rel))
    text, drops = extract_mocap_run(
        t, q, np.full(len(t), 0.3), camera, board, lag, pal_pose[:3], postures[:3]
    )
    assert drops["unpaired"] == 2 and drops["kept"] == 3
    frozen = board.copy()
    sel = (frozen[:, 1] >= 5.9) & (frozen[:, 1] <= 10.1)
    frozen[sel, 2:] = frozen[np.flatnonzero(sel)[0] - 1, 2:]
    text, drops = extract_mocap_run(
        t, q, np.full(len(t), 0.3), camera, frozen, lag, pal_pose, postures
    )
    assert drops["lost"] == 1 and drops["kept"] == 4


def test_extract_run_refuses_a_wrong_bag_hash(tmp_path):
    bag = tmp_path / "x.bag"
    bag.write_bytes(b"x")
    with pytest.raises(ValueError, match="sha256 mismatch"):
        extract_run(str(bag), HEADER, expected_sha256="0" * 64)


HC = Path(__file__).resolve().parents[1] / "examples/tiago/data/calibration/head_camera"


@pytest.mark.slow
@pytest.mark.skipif(
    not os.environ.get("TIAGO_RECORDINGS_ROOT"),
    reason="needs the original recordings: set TIAGO_RECORDINGS_ROOT",
)
def test_shipped_files_are_re_created_byte_for_byte():
    manifest = yaml.safe_load((HC / "protocol.yaml").read_text())
    root = Path(os.environ["TIAGO_RECORDINGS_ROOT"])

    def find(entry):
        for path in root.rglob(entry["name"]):
            if file_sha256(path) == entry["sha256"]:
                return path
        raise AssertionError(
            f"{entry['name']} (sha256 {entry['sha256']}) not under {root}"
        )

    for s in manifest["sessions"]:
        src, files = s["source"], s["files"]
        csv_text = find(src["camera_csv"]).read_text()
        bag = find(src["bag"]) if "bag" in src else None
        for name in files:
            if name.startswith("mocap_"):
                table, lag, drops = extract_run(str(bag), csv_text)
                assert abs(lag - src["bag"]["lag_s"]) < 5e-4
                assert drops == s["drops"]
                assert table == (HC / name).read_text()
        if any(n.startswith("camera_") for n in files):
            torso = paired_torso(str(bag), csv_text) if bag else None
            pose, joints = camera_files(csv_text, torso)
            assert (
                pose == (HC / f"camera_{s['id']}_chessboard_in_camera.csv").read_text()
            )
            assert joints == (HC / f"camera_{s['id']}_joints.csv").read_text()
