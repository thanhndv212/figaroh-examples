"""TIAGo suspension OptiTrack runs of 2023-07 and 2023-09-12 (#112).

Synthetic cases pin the extraction rules of
examples/tiago/utils/suspension_optitrack_extraction.py; the shipped files are
checked against protocol.yaml (hashes, schema, one clock, counts, roles). The
slow test re-creates every shipped file when TIAGO_RECORDINGS_ROOT points at
the original recordings.
"""

import os
from pathlib import Path

import numpy as np
import pytest
import yaml

from examples.tiago.utils.mocap_extraction import file_sha256
from examples.tiago.utils.suspension_optitrack_extraction import (
    BODY_COLUMNS,
    REFERENCE_BODY,
    WRIST_FT,
    audit,
    body_csv,
    extract_run,
    joint_channels,
    live_mask,
    run_lag,
)

DATA = Path(__file__).resolve().parents[1] / "examples/tiago/data/suspension_optitrack"
MANIFEST = yaml.safe_load((DATA / "protocol.yaml").read_text())
SESSIONS = {s["id"]: s for s in MANIFEST["sessions"]}
ROLES = {
    "2023-07-26-1017_around_z_no_weight": "training",
    "2023-07-26-1058_around_y_no_weight": "training",
    "2023-07-26-1143_y_fold_no_weight": "training",
    "2023-07-28-1239_y_fold_no_weight": "diagnostic",
    "2023-07-28-1301_y_fold_no_weight": "diagnostic",
    "2023-07-28-1307_around_z_weight": "diagnostic",
    "2023-07-28-1312_y_fold_weight": "training",
    "2023-07-28-1314_around_y_weight": "training",
    "2023-07-28-1324_around_x_weight": "training",
    "2023-07-28-1328_around_x_weight": "training",
    "2023-07-28-1333_x_fold_weight": "training",
    "2023-09-12-1625_around_z_no_weight": "validation",
    "2023-09-12-1626_y_fold_no_weight": "validation",
    "2023-09-12-1628_x_fold_no_weight": "validation",
    "2023-09-12-1629_around_x_no_weight": "validation",
    "2023-09-12-1641_around_z_weight": "validation",
    "2023-09-12-1643_y_fold_weight": "validation",
    "2023-09-12-1644_around_z_weight": "confirmation",
    "2023-09-12-1645_y_fold_weight": "confirmation",
    "2023-09-12-1646_x_fold_weight": "validation",
    "2023-09-12-1648_around_x_weight": "validation",
    "2023-09-12-1649_around_y_weight": "validation",
}


def _stream(n, t0=100.0, dt=1 / 120, shift=0.05):
    """Body stream ``t_header, t_recv, pose``; live samples never repeat."""
    t = t0 + dt * np.arange(n)
    pose = np.tile([1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0], (n, 1))
    pose[:, 0] += 1e-6 * np.arange(n)
    return np.column_stack([t, t + shift, pose])


def _rows(n, t0=100.0, dt=0.01, shift=0.02):
    """Introspection rows ``(bag_ns, seq, secs, nsecs, version, values)``."""
    out = []
    for k in range(n):
        t = t0 + k * dt
        secs = int(t)
        nsecs = int(round((t - secs) * 1e9))
        out.append((int(round((t + shift) * 1e9)), k, secs, nsecs, 1, [0.0]))
    return out


def _load(sid, name):
    with open(DATA / sid / name) as f:
        header = f.readline().strip().split(",")
    return header, np.loadtxt(DATA / sid / name, delimiter=",", skiprows=1, ndmin=2)


# --- extraction rules -------------------------------------------------------


def test_joint_columns_are_position_and_effort_then_the_wrist_force_torque():
    cols = [c for c, _ in joint_channels()]
    assert cols[:4] == [
        "torso_lift_position",
        "torso_lift_effort",
        "arm_1_position",
        "arm_1_effort",
    ]
    assert len(cols) == 8 * 2 + 6 and cols[-6:] == list(WRIST_FT)
    assert not any("velocity" in c for c in cols)
    assert dict(joint_channels())["arm_7_effort"] == "arm_7_joint_effort"


def test_a_sample_is_frozen_only_when_all_seven_pose_values_repeat():
    s = _stream(5)
    s[2, 2:9] = s[1, 2:9]  # NatNet repeats the last pose
    s[3, 2:9] = s[2, 2:9]
    s[4, 2:9] = s[3, 2:9]
    s[4, 8] += 1e-12  # one value differs: a measurement
    assert live_mask(s).tolist() == [True, True, False, False, True]
    assert live_mask(np.zeros((0, 9))).size == 0


def test_body_rows_keep_every_sample_on_the_robot_clock():
    s = _stream(4)
    s[2, 2:9] = s[1, 2:9]
    text = body_csv(s, lag=-1.25, t0=98.0)
    lines = text.splitlines()
    assert lines[0] == ",".join(BODY_COLUMNS)
    assert len(lines) == 5  # nothing dropped
    first = lines[1].split(",")
    assert float(first[0]) == pytest.approx(100.0 - 1.25 - 98.0)
    assert first[1] == repr(1.0) and first[-1] == "1"
    assert [ln.split(",")[-1] for ln in lines[1:]] == ["1", "1", "0", "1"]


def test_lag_is_the_difference_of_receive_minus_header_medians():
    rows = _rows(500, shift=0.02)
    body = _stream(600, shift=1.40)
    # mocap headers trail their receipt by 1.38 s more than the robot's do:
    # t_robot - t_mocap = 1.40 - 0.02
    assert run_lag(rows, body) == pytest.approx(1.40 - 0.02, abs=1e-6)
    frozen = body.copy()
    frozen[::2, 1] += 5.0  # receive stamps of samples that will be frozen
    frozen[::2, 2:9] = np.roll(frozen, 1, axis=0)[::2, 2:9]
    frozen[0, 2:9] += 1.0
    assert run_lag(rows, frozen) == pytest.approx(1.40 - 0.02, abs=1e-6)


def test_audit_counts_samples_frozen_and_stamp_defects():
    s = _stream(6)
    s[3, 2:9] = s[2, 2:9]
    s[5, 0] = s[4, 0]  # repeated stamp
    rows = _rows(4)
    counts = audit(rows, {"b": s})
    assert counts["joints"] == {
        "samples": 4,
        "repeated_stamps": 0,
        "backward_stamps": 0,
    }
    assert counts["b"] == {
        "samples": 6,
        "frozen": 1,
        "repeated_stamps": 1,
        "backward_stamps": 0,
    }


def test_extract_run_refuses_a_bag_that_is_not_the_recorded_one(tmp_path):
    bag = tmp_path / "other.bag"
    bag.write_bytes(b"not the recording")
    with pytest.raises(ValueError, match="sha256 mismatch"):
        extract_run(str(bag), expected_sha256="0" * 64)


# --- shipped files ----------------------------------------------------------


def test_every_shipped_file_matches_its_hash_and_nothing_else_is_shipped():
    listed = {n: d for s in SESSIONS.values() for n, d in s["files"].items()}
    assert len(listed) == 68
    for name, digest in listed.items():
        assert file_sha256(DATA / name) == digest, name
    on_disk = {
        p.relative_to(DATA).as_posix() for p in DATA.rglob("*") if p.is_file()
    } - {"protocol.yaml"}
    assert on_disk == set(listed)


def test_roles_are_frozen_and_every_load_case_has_training_and_held_out_runs():
    assert {i: s["role"] for i, s in SESSIONS.items()} == ROLES
    assert MANIFEST["force_plate"] is False and MANIFEST["robot"] == "tiago-48"
    for load in ("no_weight", "weight"):
        roles = {s["role"] for s in SESSIONS.values() if s["load"] == load}
        assert {"training", "validation"} <= roles
    for s in SESSIONS.values():
        assert s["usable"] and s["traced"] and len(s["source"]["bag"]["sha256"]) == 64
        assert (s["role"] == "diagnostic") == ("role_reason" in s)
        assert s["id"].endswith(f"{s['motion']}_{s['load']}")


def test_the_role_rule_follows_from_the_shipped_files():
    """Rules 1 to 4 of protocol.yaml, re-derived from the data."""
    force, torso, frozen = {}, {}, {}
    for sid in SESSIONS:
        header, j = _load(sid, "joints.csv")
        f = np.column_stack([j[:, header.index(c)] for c in WRIST_FT[:3]])
        force[sid] = np.median(np.linalg.norm(f, axis=1))
        torso[sid] = np.ptp(j[:, header.index("torso_lift_position")]) * 1e3
        _, b = _load(sid, f"{REFERENCE_BODY}.csv")
        frozen[sid] = 1 - b[:, -1].mean()
    weight = {}
    for sid, s in SESSIONS.items():
        assert s["load_evidence"]["wrist_force_n"] == pytest.approx(force[sid], abs=0.01)
        if s["load"] == "weight":
            weight.setdefault(sid[:10], []).append(force[sid])
    seen = set()
    for sid in sorted(SESSIONS):
        s = SESSIONS[sid]
        w = weight.get(sid[:10])
        diagnostic = (
            torso[sid] > 50
            or frozen[sid] > 0.10
            or (
                s["load"] == "no_weight"
                and w is not None
                and abs(force[sid] - np.median(w)) < 1.0
            )
        )
        if diagnostic:
            expected = "diagnostic"
        elif sid.startswith("2023-07"):
            expected = "training"
        else:
            key = (s["motion"], s["load"])
            expected = "confirmation" if key in seen else "validation"
            seen.add(key)
        assert s["role"] == expected, sid
    # the load gap the rule relies on: about 4.2 N on 2023-09-12, none on 07-28
    sept = {s["load"]: [] for s in SESSIONS.values()}
    for sid, s in SESSIONS.items():
        if sid.startswith("2023-09-12"):
            sept[s["load"]].append(force[sid])
    assert np.median(sept["weight"]) - np.median(sept["no_weight"]) == pytest.approx(
        4.2, abs=0.3
    )


@pytest.mark.parametrize("sid", sorted(ROLES))
def test_each_run_has_the_schema_the_counts_and_one_clock(sid):
    s = SESSIONS[sid]
    header, j = _load(sid, "joints.csv")
    assert header == ["t"] + [c for c, _ in joint_channels()]
    assert len(j) == s["samples"] and j[0, 0] == 0.0
    assert np.all(np.diff(j[:, 0]) > 0)
    assert np.median(np.diff(j[:, 0])) == pytest.approx(0.01, abs=1e-4)
    assert j[-1, 0] == pytest.approx(s["duration_s"], abs=0.01)
    assert np.all(np.isfinite(j))
    assert set(s["streams"]) == {
        Path(n).stem for n in s["files"] if not n.endswith("joints.csv")
    }
    assert REFERENCE_BODY in s["streams"]
    for body, counts in s["streams"].items():
        header, b = _load(sid, f"{body}.csv")
        assert header == list(BODY_COLUMNS)
        assert len(b) == counts["samples"]
        assert int((b[:, -1] == 0).sum()) == counts["frozen"]
        assert set(np.unique(b[:, -1])) <= {0.0, 1.0}
        assert np.all(np.diff(b[:, 0]) > 0)
        assert np.median(np.diff(b[:, 0])) == pytest.approx(1 / 120, rel=0.05)
        # one clock: the body stream spans the joint stream within 0.2 s
        assert abs(b[0, 0] - j[0, 0]) < 0.2 and abs(b[-1, 0] - j[-1, 0]) < 0.2
        live = b[b[:, -1] == 1]
        assert np.allclose(np.linalg.norm(live[:, 4:8], axis=1), 1.0, atol=1e-5)


# --- re-creation from the original recordings ------------------------------


def _find(root: Path, folder: str, entry: dict) -> Path:
    for path in root.rglob(entry["name"]):
        if folder in path.parts and file_sha256(path) == entry["sha256"]:
            return path
    raise AssertionError(f"{entry['name']} (sha256 {entry['sha256']}) not under {root}")


@pytest.mark.slow
@pytest.mark.skipif(
    not os.environ.get("TIAGO_RECORDINGS_ROOT"),
    reason="needs the original recordings: set TIAGO_RECORDINGS_ROOT",
)
def test_suspension_optitrack_files_are_re_created_byte_for_byte():
    root = Path(os.environ["TIAGO_RECORDINGS_ROOT"])
    for s in SESSIONS.values():
        src = s["source"]
        bag = _find(root, src["folder"], src["bag"])
        files, lag, counts = extract_run(str(bag), src["bag"]["sha256"])
        assert lag == pytest.approx(s["lag_s"], abs=5e-5), s["id"]
        assert counts["joints"]["samples"] == s["samples"]
        for name, text in files.items():
            rel = f"{s['id']}/{name}"
            assert (DATA / rel).read_text() == text, rel
