"""Shipped TIAGo backlash sinusoid sessions, 2023-03 and 2023-07 (#109)."""

import csv
from pathlib import Path

import numpy as np
import pytest
import yaml

from examples.tiago.utils.backlash_surface import load_backlash_joint_trajectory
from examples.tiago.utils.introspection_extraction import read_tf_csv
from examples.tiago.utils.mocap_extraction import file_sha256

BL = Path(__file__).resolve().parents[1] / "examples/tiago/data/backlash"
MANIFEST = yaml.safe_load((BL / "protocol.yaml").read_text())
SESSIONS = {s["id"]: s for s in MANIFEST["sessions"]}
ROLES = {
    "2023-07-24-13-28-42": "training",
    "2023-03-09-16-19-17-bottom": "held-out",
    "2023-03-09-15-28-20-top": "diagnostic",
}
ARM = [f"arm_{i}_joint" for i in range(1, 8)]


def test_every_shipped_file_matches_its_hash_and_nothing_else_is_shipped():
    listed = {n: d for s in SESSIONS.values() for n, d in s["files"].items()}
    for name, digest in listed.items():
        assert file_sha256(BL / name) == digest, name
    on_disk = {str(p.relative_to(BL)) for p in BL.rglob("*") if p.is_file()} - {
        "protocol.yaml"
    }
    assert on_disk == set(listed)


def test_roles_robots_and_trace_are_frozen():
    assert {i: s["role"] for i, s in SESSIONS.items()} == ROLES
    assert SESSIONS["2023-07-24-13-28-42"]["traced"] is False
    assert SESSIONS["2023-03-09-16-19-17-bottom"]["traced"] is True
    assert SESSIONS["2023-03-09-16-19-17-bottom"]["robot"] == "tiago-48"
    # the top run has no encoder channel to fingerprint
    assert SESSIONS["2023-03-09-15-28-20-top"]["robot"] == "unknown"
    for s in SESSIONS.values():
        assert s["usable"] is True


@pytest.mark.parametrize("sid", ["2023-07-24-13-28-42", "2023-03-09-16-19-17-bottom"])
def test_value_rows_match_the_names_file_and_the_header_clock_increases(sid):
    s = SESSIONS[sid]
    names_path = next(BL / p for p in s["files"] if p.endswith("datanames.csv"))
    values_path = next(BL / p for p in s["files"] if p.endswith("datavalues.csv"))
    header = next(csv.reader(names_path.open()))
    n_names = len(header) - header.index("names") - 1
    last = None
    count = 0
    with values_path.open(newline="") as f:
        for row in csv.DictReader(f):
            values = row["values"].strip("[]").split(",")
            assert len(values) == n_names
            t = float(row["secs"]) + float(row["nsecs"]) * 1e-9
            assert last is None or t > last
            last = t
            count += 1
    assert count == s["samples"]


def test_bottom_run_loads_with_the_example_loader_for_every_arm_joint():
    d = BL / "sinus_amp3_period10_2023-03-09-16-19-17_bottom"
    for joint in ARM:
        traj = load_backlash_joint_trajectory(d, joint)
        assert traj.sample_count == 41953
        assert traj.time[-1] - traj.time[0] == pytest.approx(419.52, abs=0.01)
        # every arm joint moves over at least one radian
        assert np.ptp(traj.relative_position) > 1.0
    # the +111 mrad arm_5 gap of the audit
    traj = load_backlash_joint_trajectory(d, "arm_5_joint")
    assert np.median(traj.encoder_difference) == pytest.approx(0.1106, abs=1e-3)


def test_top_run_is_pose_only_and_its_arm_frames_move():
    rows = read_tf_csv(str(BL / "sinus_amp3_period10_2023-03-09-15-28-20_top/tf.csv"))
    assert len(rows) == SESSIONS["2023-03-09-15-28-20-top"]["rows"]
    quats = {}
    for _, tfs in rows[::50]:
        for _, _, parent, child, p, q in tfs:
            if child.startswith("arm_") and child.endswith("_link"):
                quats.setdefault(child, []).append(q)
    assert set(quats) == {f"arm_{i}_link" for i in range(1, 8)}
    for child, q in quats.items():
        assert np.ptp(np.array(q), axis=0).max() > 0.2, child
