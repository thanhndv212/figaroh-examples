"""Square-motion extraction rules (#107): synthetic checks and re-creation."""

import hashlib
import os
from pathlib import Path

import numpy as np
import pytest
import yaml

from examples.tiago.utils.mocap_extraction import JOINTS, file_sha256
from examples.tiago.utils.square_motion_extraction import (
    JOINT_COLUMNS,
    MARKER_COLUMNS,
    extract_run,
    format_joints,
    format_markers,
    lag_body,
    marker_rows,
)

ROOT = Path(__file__).resolve().parents[1]
SQUARE = ROOT / "examples/tiago/data/validation/square_motion"


def test_marker_rows_put_stamps_on_the_robot_clock_sorted_and_stable():
    a = np.array([[10.0, 1, 2, 3, 0, 0, 0, 1], [10.2, 1, 2, 3, 0, 0, 0, 1]])
    b = np.array([[10.1, 4, 5, 6, 0, 0, 0, 1], [10.2, 4, 5, 6, 0, 0, 0, 1]])
    rows, names = marker_rows({"b": b, "a": a}, lag=2.0, t0=11.0)
    assert np.allclose(rows[:, 0], [1.0, 1.1, 1.2, 1.2])
    assert names == ["a", "b", "a", "b"]  # ties keep sorted-name order


def test_lag_body_prefers_plate_point_then_rigid_body():
    assert (
        lag_body({"base_frame": 0, "eeframe_BL": 0, "endeffector_frame": 0})
        == "eeframe_BL"
    )
    assert lag_body({"base_frame": 0, "endeffector_frame": 0}) == "endeffector_frame"
    with pytest.raises(ValueError):
        lag_body({"base_frame": 0})


def test_formats_are_fixed_precision_with_header():
    t = np.array([100.0, 100.01])
    q = np.arange(16, dtype=float).reshape(2, 8) / 7
    text = format_joints(t, q, 100.0)
    lines = text.splitlines()
    assert lines[0].split(",") == JOINT_COLUMNS == ["t"] + JOINTS
    assert lines[2].startswith("0.0100,") and text.endswith("\n")
    rows = np.array([[0.5, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0]])
    assert format_markers(rows, ["base_frame"]).splitlines() == [
        ",".join(MARKER_COLUMNS),
        "0.5000,base_frame,1.00000,2.00000,3.00000,0.00000,0.00000,0.00000,1.00000",
    ]


def test_shipped_files_match_protocol_hashes():
    manifest = yaml.safe_load((SQUARE / "protocol.yaml").read_text())
    for entry in manifest["sessions"]:
        for name, digest in entry["files"].items():
            assert hashlib.sha256((SQUARE / name).read_bytes()).hexdigest() == digest


@pytest.mark.slow
@pytest.mark.skipif(
    not os.environ.get("TIAGO_RECORDINGS_ROOT"),
    reason="needs the original recordings: set TIAGO_RECORDINGS_ROOT",
)
@pytest.mark.parametrize(
    "session",
    [
        "2021-12-16-15-04-54_zero",
        "2021-12-16-15-07-04_zero",
        "2021-12-16-15-38-03_mocap",
        "2022-01-12-10-55-17_zero",
        "2022-01-12-11-25-38_mocap",
        "2022-01-12-11-10-58_mocap_inv",
        "2022-01-12-11-38-43_pal",
    ],
)
def test_shipped_files_are_re_created_byte_for_byte(session):
    """Original bag (named by sha256) to the two shipped files, lag re-measured."""
    pytest.importorskip("rosbags")
    manifest = yaml.safe_load((SQUARE / "protocol.yaml").read_text())
    entry = next(s for s in manifest["sessions"] if s["id"] == session)
    source = entry["source"]
    root = Path(os.environ["TIAGO_RECORDINGS_ROOT"])
    bag = next(root.rglob(source["bag"]), None)
    assert bag is not None, f"{source['bag']} not under {root}"
    assert file_sha256(bag) == source["sha256"]
    result = extract_run(
        str(bag), str(ROOT / "examples/tiago/urdf/tiago_48_schunk.urdf")
    )
    assert abs(result["lag"] - source["lag_s"]) < 5e-4
    assert result["correlation"] == pytest.approx(source["lag_correlation"], abs=1e-3)
    assert round(result["t0"], 6) == source["t0_unix_s"]
    assert result["registration_rmse_mm"] == pytest.approx(
        entry["nominal_registration_rmse_mm"], abs=0.01
    )
    offsets = manifest["offset_sets"][entry["offset_set"]]["values"]
    if offsets not in ("unknown",) and isinstance(offsets, dict):
        for joint, value in offsets.items():
            got = result["offsets"][joint]
            assert np.allclose(got["translation_m"], value["translation_m"], atol=1e-3)
            assert np.allclose(got["rotation_rad"], value["rotation_rad"], atol=1e-3)
    for kind in ("joints", "markers"):
        name = f"{session}_{kind}.csv"
        assert result[f"{kind}_csv"].encode() == (SQUARE / name).read_bytes()
