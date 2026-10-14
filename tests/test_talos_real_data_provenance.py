"""Provenance of the TALOS table-contact postures (examples#113).

The original recordings are not distributed, so these tests check the
shipped files against ``data/real/protocol.yaml`` and the extraction rules
on fixtures. Set ``TALOS_RECORDINGS_ROOT`` to also rebuild from the sources.
"""

import csv
import os
import re
from pathlib import Path

import numpy as np
import pytest
import yaml

from examples.talos_table_contact.utils import real_data_extraction as rde
from examples.talos_table_contact.utils.real_data_extraction import (
    HEADER,
    JOINTS,
    Posture,
    Samples,
    evidence_rows,
    extract_posture_files,
    sha256_of,
)

DATA = rde.DATA_DIR
PROTOCOL = yaml.safe_load((DATA / "protocol.yaml").read_text())
SESSIONS = {s["file"]: s for s in PROTOCOL["sessions"]}
COUNTS = {
    "left_train.csv": 21,
    "left_validation.csv": 9,
    "right_train.csv": 29,
    "right_validation.csv": 9,
}


def test_protocol_hashes_and_roles():
    assert PROTOCOL["version"] == 2
    assert set(SESSIONS) == set(COUNTS)
    for name, session in SESSIONS.items():
        assert sha256_of(DATA / name) == session["sha256"]
        assert session["rows"] == COUNTS[name] == len(session["postures"])
        assert session["role"] in {"training", "validation"}
    assert SESSIONS["left_train.csv"]["role"] == "training"
    assert SESSIONS["left_validation.csv"]["role"] == "validation"
    evidence = PROTOCOL["contact_evidence"]
    assert sha256_of(DATA / evidence["file"]) == evidence["sha256"]
    assert {s["role"] for s in PROTOCOL["unshipped_sessions"]} == {"diagnostic"}


def test_old_hashes_recorded_and_only_headers_changed():
    old = PROTOCOL["previous_version"]["files"]
    assert set(old) == set(COUNTS)
    # left_train always had the right header: unchanged.
    assert old["left_train.csv"] == SESSIONS["left_train.csv"]["sha256"]
    for name in COUNTS:
        if name != "left_train.csv":
            assert old[name] != SESSIONS[name]["sha256"]


def test_sources_named_with_sha256_and_self_contained():
    sha = re.compile(r"^[0-9a-f]{64}$")
    assert all(sha.match(h) for h in PROTOCOL["measurement_files"].values())
    assert all(sha.match(b["sha256"]) for b in PROTOCOL["bags"].values())
    assert "original recordings, not distributed" in (DATA / "README.md").read_text()
    for name in ("README.md", "protocol.yaml"):
        text = (DATA / name).read_text()
        for forbidden in (
            "robot-calibration-" + "identification-dataset",
            "AU" + "DIT",
            "tools/" + "audit",
            "/Users/",
        ):
            assert forbidden not in text


@pytest.mark.parametrize("name", sorted(COUNTS))
def test_posture_file_schema(name):
    lines = (DATA / name).read_bytes().decode().split("\n")
    assert lines[-1] == "" and "\r" not in "".join(lines)
    assert lines[0] == HEADER
    header = lines[0].split(",")
    assert header[4] == "joint_names" and tuple(header[5:]) == JOINTS
    rows = [line.split(",") for line in lines[1:-1]]
    assert len(rows) == COUNTS[name]
    assert all(len(r) == len(header) for r in rows)
    assert all(np.isfinite(np.array(r[5:], dtype=float)).all() for r in rows)


def test_repeated_markers_kept():
    left = (DATA / "left_train.csv").read_text().splitlines()
    right = (DATA / "right_train.csv").read_text().splitlines()
    assert sum(r.split(",")[3] == "table/contact_21" for r in left) == 2
    assert sum(r.split(",")[3] == "table/contact_20" for r in right) == 2


def _evidence():
    with (DATA / "contact_evidence.csv").open() as f:
        return list(csv.DictReader(f))


def test_contact_evidence_schema_and_rule():
    rows = _evidence()
    assert ",".join(rows[0]) == rde.EVIDENCE_COLUMNS
    assert len(rows) == 68
    matched = [r for r in rows if r["status"] == "matched"]
    assert len(matched) == 21
    assert {r["status"] for r in rows} == {"matched", "no_matching_sample"}
    for r in rows:
        if r["status"] != "matched":
            assert all(
                v == ""
                for k, v in r.items()
                if k not in ("posture_file", "row", "contact_marker", "status")
            )
            continue
        assert float(r["max_joint_diff_rad"]) <= rde.MAX_JOINT_DIFF_RAD
        assert int(r["contact"]) == int(float(r["force_z_N"]) >= rde.CONTACT_FZ_N)
        assert PROTOCOL["bags"][r["bag"]]["status"] == "searched"


def test_contact_evidence_clock():
    for r in _evidence():
        if r["status"] != "matched":
            continue
        assert re.fullmatch(r"\d{10}\.\d{9}", r["stamp_s"])
        day = re.search(r"(\d{4})-(\d{2})-(\d{2})", r["bag"]).groups()
        from datetime import datetime, timezone

        stamp = datetime.fromtimestamp(float(r["stamp_s"]), timezone.utc)
        assert (f"{stamp:%Y}", f"{stamp:%m}", f"{stamp:%d}") == day
        # evidence rows point at real shipped postures
        assert (
            r["posture_file"] in COUNTS
            and 2 <= int(r["row"]) <= COUNTS[r["posture_file"]] + 1
        )


def _fake_sources(tmp_path):
    """Write measurement files whose lines are the shipped rows (CRLF header)."""
    wanted = {}
    for name, session in SESSIONS.items():
        shipped = (DATA / name).read_text().splitlines()[1:]
        for ref, row in zip(session["postures"], shipped):
            src, line = ref.rsplit(":", 1)
            wanted.setdefault(src, {})[int(line)] = row
    for src, rows in wanted.items():
        path = tmp_path / src
        path.parent.mkdir(parents=True, exist_ok=True)
        top = max(rows)
        lines = [HEADER.split(",", 4)[4] + "\r"] + [
            rows.get(i, "x") for i in range(2, top + 1)
        ]
        path.write_bytes(("\n".join(lines) + "\n").encode())


def test_posture_files_rebuilt_byte_for_byte_from_fixture_sources(tmp_path):
    _fake_sources(tmp_path)
    built = extract_posture_files(tmp_path, PROTOCOL, verify=False)
    assert set(built) == set(COUNTS)
    for name, content in built.items():
        assert content == (DATA / name).read_bytes()


def test_source_hash_is_enforced(tmp_path):
    _fake_sources(tmp_path)
    with pytest.raises(ValueError, match="sha256"):
        extract_posture_files(tmp_path, PROTOCOL)


def test_evidence_rows_apply_fixed_rule():
    q = np.linspace(0.1, 0.9, len(JOINTS))
    wrench = np.array([[0, 0, 12.0, 0, 0, 0], [0, 0, 9.0, 0, 0, 0]])
    bag = Samples(
        "b.bag",
        ["1.000000000", "2.000000000"],
        np.vstack([q, q + 1]),
        {"left": wrench, "right": wrench},
    )
    posture = Posture("left_train.csv", 2, "table/contact_01", q + 1e-4)
    far = Posture("left_train.csv", 3, "table/contact_02", q + 5e-4)
    other = Posture("left_train.csv", 4, "table/contact_03", q + 1 + 1e-4)
    rows = [r.split(",") for r in evidence_rows([posture, far, other], [bag])]
    assert (
        rows[0][3] == "matched" and rows[0][-1] == "1" and rows[0][5] == "1.000000000"
    )
    assert rows[1][3] == "no_matching_sample" and len(rows[1]) == 14
    assert rows[2][3] == "matched" and rows[2][-1] == "0"  # Fz 9 N < 10 N


def test_shared_layouts_reused_only_when_identical():
    joints = [f"{j}_position" for j in JOINTS]
    stats = ["a", "b", "c", "d"]
    layouts = rde.shared_layouts([{1: stats, 2: joints}, {6: joints}])
    assert layouts == {len(joints): joints}
    with pytest.raises(ValueError, match="differ"):
        rde.shared_layouts([{2: joints}, {4: joints[::-1]}])
    recovered = [n for n, b in PROTOCOL["bags"].items() if b["status"] == "recovered"]
    assert recovered == ["20221028/pyrene-calibration_2022-10-28-09-00-43.bag"]


@pytest.mark.skipif(
    "TALOS_RECORDINGS_ROOT" not in os.environ, reason="recordings not distributed"
)
def test_rebuild_from_original_recordings():
    root = Path(os.environ["TALOS_RECORDINGS_ROOT"])
    built = extract_posture_files(root, PROTOCOL)
    for name, content in built.items():
        assert content == (DATA / name).read_bytes()
    pytest.importorskip("rosbags")
    evidence = rde.extract_contact_evidence(root, PROTOCOL, built)
    assert evidence == (DATA / "contact_evidence.csv").read_bytes()
