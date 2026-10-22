"""Shipped TIAGo arm_5/arm_6 joint-inspection recordings, 2023-12 to 2024-02 (#110)."""

from pathlib import Path

import numpy as np
import pandas as pd
import pytest
import yaml

from examples.tiago.utils.mocap_extraction import file_sha256

JI = Path(__file__).resolve().parents[1] / "examples/tiago/data/joint_inspection"
MANIFEST = yaml.safe_load((JI / "protocol.yaml").read_text())
SESSIONS = {s["id"]: s for s in MANIFEST["sessions"]}
COLUMNS = ["t"] + [
    f"arm_{j}_{k}" for j in (5, 6) for k in ("position", "absolute", "command")
]
FOLDERS = {
    "2023-12_arm6_inspection": 4,
    "2024-01-25_arm6_backlash": 6,
    "2024-02-02_arm5_positions": 14,
}


def _frame(sid):
    (path,) = SESSIONS[sid]["files"]
    return pd.read_csv(JI / path)


def test_every_shipped_file_matches_its_hash_and_nothing_else_is_shipped():
    listed = {n: d for s in SESSIONS.values() for n, d in s["files"].items()}
    assert len(listed) == 24
    for name, digest in listed.items():
        assert file_sha256(JI / name) == digest, name
    on_disk = {str(p.relative_to(JI)) for p in JI.rglob("*") if p.is_file()} - {
        "protocol.yaml"
    }
    assert on_disk == set(listed)


def test_all_sessions_are_diagnostic_traced_and_the_audits_arm_5_count_is_14():
    assert len(SESSIONS) == 24
    for s in SESSIONS.values():
        assert s["role"] == "diagnostic" and s["usable"] and s["traced"]
        assert len(s["source"]["bag"]["sha256"]) == 64
    per_folder = {f: 0 for f in FOLDERS}
    for s in SESSIONS.values():
        per_folder[s["folder"]] += 1
    assert per_folder == FOLDERS
    assert [n["id"] for n in MANIFEST["not_usable"]] == ["2023-10-25"]


def test_robot_identity_is_tiago_48_from_2024_and_unknown_for_2023_12():
    for s in SESSIONS.values():
        want = "unknown" if s["folder"].startswith("2023-12") else "tiago-48"
        assert s["robot"] == want, s["id"]


@pytest.mark.parametrize("sid", sorted(SESSIONS))
def test_schema_clock_and_commands_of_each_file(sid):
    d = _frame(sid)
    assert list(d.columns) == COLUMNS
    assert len(d) == SESSIONS[sid]["samples"]
    assert not d.isna().any().any()
    assert d.t.iloc[0] == 0.0
    assert (np.diff(d.t) > 0).all()
    # 100 Hz introspection on the header clock
    assert np.median(np.diff(d.t)) == pytest.approx(0.01, abs=1e-3)
    assert d.t.iloc[-1] == pytest.approx(SESSIONS[sid]["duration_s"], abs=0.01)


def test_the_two_manual_probes_hold_the_motor_still_while_the_output_moves():
    for sid in (
        "arm_6_backlash_abs_inspect_2024-01-26-18-07-23",
        "arm_6_backlash_abs_inspect_2024-01-26-18-07-54",
    ):
        d = _frame(sid)
        assert np.ptp(d.arm_6_position) < 1e-3
        assert 0.035 < np.ptp(d.arm_6_absolute) < 0.045


def test_arm_6_sweeps_and_arm_5_offset_moves_between_sessions():
    d = _frame("arm_inspect_1_2023-12-05-16-20-45")
    assert np.ptp(d.arm_6_position) == pytest.approx(2.728, abs=1e-3)
    assert np.ptp(d.arm_5_position) < 1e-3
    gaps = {
        sid: np.median(_frame(sid).arm_5_position - _frame(sid).arm_5_absolute)
        for sid in (
            "arm5_minus1_59_2024-02-02-16-17-50",
            "arm5_0_8_2024-02-02-17-43-00",
        )
    }
    assert gaps["arm5_minus1_59_2024-02-02-16-17-50"] == pytest.approx(
        -0.0924, abs=1e-3
    )
    assert gaps["arm5_0_8_2024-02-02-17-43-00"] == pytest.approx(0.1564, abs=1e-3)
