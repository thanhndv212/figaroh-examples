"""CREPS Vicon and force-plate runs of 2023-09-20, one layout (#111).

The slow test re-creates the shipped files when TIAGO_RECORDINGS_ROOT points at
the original recordings.
"""

import os
from pathlib import Path

import numpy as np
import pytest
import yaml

from examples.tiago.utils.mocap_extraction import file_sha256
from examples.tiago.utils.suspension_data import load_vicon_forceplate_trajectory
from examples.tiago.utils.suspension_extraction import (
    COLUMNS,
    MARKERS,
    PLATE,
    full_header,
    normalise,
)

SUS = Path(__file__).resolve().parents[1] / "examples/tiago/data/suspension"
MANIFEST = yaml.safe_load((SUS / "protocol.yaml").read_text())
SESSIONS = {s["id"]: s for s in MANIFEST["sessions"]}
ROLES = {
    "xyz_1640": "training",
    "xyz_1628": "validation",
    "xyz_mirror_1630": "confirmation",
    "xyz_mirror_1642": "confirmation",
    "around_x_1634": "diagnostic",
    "around_y_1638": "diagnostic",
    "around_z_1622": "diagnostic",
    "x_fold_1632": "diagnostic",
    "y_fold_1636": "diagnostic",
    "postures_1720": "diagnostic",
    "postures_1734": "diagnostic",
}


def _path(sid):
    (name,) = SESSIONS[sid]["files"]
    return SUS / name


def _write(tmp_path, header, rows, eol="\r\n"):
    path = tmp_path / "raw.csv"
    lines = ([",".join(header)] if header else []) + [",".join(r) for r in rows]
    path.write_bytes((eol.join(lines) + eol).encode())
    return str(path)


def _row(n):
    return [f"2023-09-20_16:00:00.{n:06d}"] + [f"{n}.{i}0" for i in range(39)]


def test_layout_is_time_ten_marker_triples_and_nine_plate_values():
    assert len(COLUMNS) == 40 and COLUMNS[0] == "time"
    assert COLUMNS[1:4] == ("base1_x", "base1_y", "base1_z")
    assert COLUMNS[28:31] == ("gripper3_x", "gripper3_y", "gripper3_z")
    assert COLUMNS[31:] == PLATE and len(MARKERS) == 10


def test_both_header_forms_and_no_header_name_the_same_columns():
    short = ["time"] + list(MARKERS) + list(PLATE)
    assert len(short) == 20
    assert full_header(short) == list(COLUMNS)
    assert full_header(list(COLUMNS)) == list(COLUMNS)
    assert full_header(None) == list(COLUMNS)
    with pytest.raises(ValueError):
        full_header(["time", "base1"])


def test_normalise_drops_the_header_ends_lines_in_lf_and_keeps_every_field_text(
    tmp_path,
):
    rows = [_row(1), _row(2)]
    rows[0][1] = "0.7102592128300758"
    rows[1][2] = "1e-05"
    short = ["time"] + list(MARKERS) + list(PLATE)
    text = normalise(_write(tmp_path, short, rows))
    assert text == "".join(",".join(r) + "\n" for r in rows)
    assert normalise(_write(tmp_path, None, rows, eol="\n")) == text


def test_normalise_selects_columns_by_name_when_the_header_is_reordered(tmp_path):
    header = list(COLUMNS)
    header[1], header[2] = header[2], header[1]  # base1_y before base1_x
    row = _row(1)
    text = normalise(_write(tmp_path, header, [row]))
    out = text.strip().split(",")
    assert out[1] == row[2] and out[2] == row[1]
    assert out[0] == row[0] and out[3:] == row[3:]


def test_normalise_rejects_a_short_row_and_unknown_columns(tmp_path):
    with pytest.raises(ValueError, match="39 fields"):
        normalise(_write(tmp_path, None, [_row(1)[:39]]))
    bad = list(COLUMNS)
    bad[5] = "other"
    with pytest.raises(ValueError, match="not the CREPS layout"):
        normalise(_write(tmp_path, bad, [_row(1)]))


def test_every_shipped_file_matches_its_hash_and_nothing_else_is_shipped():
    listed = {n: d for s in SESSIONS.values() for n, d in s["files"].items()}
    assert len(listed) == 11
    for name, digest in listed.items():
        assert file_sha256(SUS / name) == digest, name
    on_disk = {p.name for p in SUS.iterdir()} - {"protocol.yaml"}
    assert on_disk == set(listed)


def test_roles_and_the_shipped_run_are_frozen():
    assert {i: s["role"].split()[0] for i, s in SESSIONS.items()} == ROLES
    one = SESSIONS["xyz_1640"]
    # shipped before the protocol: its source is the same file
    assert one["source"]["sha256"] == one["files"]["tiago_xyz_vicon_1640.csv"]
    for s in SESSIONS.values():
        assert s["usable"] and s["traced"] and s["robot"] == "unknown"
        assert len(s["source"]["sha256"]) == 64


@pytest.mark.parametrize("sid", sorted(ROLES))
def test_each_file_has_40_columns_lf_ends_and_a_receive_clock_the_loader_reads(sid):
    path = _path(sid)
    raw = path.read_bytes()
    assert b"\r" not in raw and raw.endswith(b"\n")
    lines = raw.decode().split("\n")[:-1]
    assert len(lines) == SESSIONS[sid]["samples"]
    assert all(len(line.split(",")) == 40 for line in lines)
    assert lines[0].split(",")[0] == SESSIONS[sid]["start_local"]
    traj = load_vicon_forceplate_trajectory(path)
    assert traj.sample_count == len(lines)
    assert traj.time[-1] - traj.time[0] == pytest.approx(
        SESSIONS[sid]["duration_s"], abs=0.05
    )
    # about 100 Hz receive time, with gaps
    assert np.median(np.diff(traj.time)) == pytest.approx(0.01, abs=1e-3)
    # the plate carries the robot's weight at rest, about 730 N down
    fz = np.array([float(line.split(",")[33]) for line in lines[:100]])
    assert fz.mean() == pytest.approx(-729.8, abs=1.0)


def test_postures_runs_hold_the_base_still_and_oscillation_runs_move_it():
    def span(sid):
        t = np.array(
            [
                [float(v) for v in line.split(",")[1:4]]
                for line in _path(sid).read_text().splitlines()
            ]
        )
        return np.ptp(t, axis=0).max()

    assert span("postures_1720") < 1e-3 and span("postures_1734") < 1e-3
    assert span("around_z_1622") > 0.015  # up to 16.6 mm in y


def _find(root: Path, entry: dict) -> Path:
    for path in root.rglob(entry["name"]):
        if file_sha256(path) == entry["sha256"]:
            return path
    raise AssertionError(f"{entry['name']} (sha256 {entry['sha256']}) not under {root}")


@pytest.mark.slow
@pytest.mark.skipif(
    not os.environ.get("TIAGO_RECORDINGS_ROOT"),
    reason="needs the original recordings: set TIAGO_RECORDINGS_ROOT",
)
def test_suspension_files_are_re_created_byte_for_byte():
    root = Path(os.environ["TIAGO_RECORDINGS_ROOT"])
    for s in SESSIONS.values():
        (name,) = s["files"]
        text = normalise(str(_find(root, s["source"])))
        assert (SUS / name).read_bytes() == text.encode(), name
