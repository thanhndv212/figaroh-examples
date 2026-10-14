"""TIAGo Pro mocap data and protocol freeze (#97).

Checks the freeze described in docs/development/tiago-pro-mocap.md: file
hashes and sizes, roles, config wiring, the data defects recorded in
examples/tiago_pro/data/README.md, and the measurement-noise estimate.
"""

import hashlib
import subprocess
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
PRO = ROOT / "examples" / "tiago_pro"
DATA = PRO / "data"
sys.path.insert(0, str(ROOT))

FROZEN = {
    "calibration_samples_20260805_1246.csv": (
        "training",
        48,
        "bd6bb49399d71e2b8c72a00527325914cac745571f142ac2316b65773dc26114",
    ),
    "calibration_samples_20260701_0913.csv": (
        "validation",
        44,
        "3ac035a8a483d91210e39018134f2b0a2f8f954516351b66f48c6ee87461982d",
    ),
    "calibration_samples_20260702_0756.csv": (
        "confirmation",
        94,
        "dd2b7f6d6320ce4fee61125b693cad2eae87ff42770f225220b1f440aea47789",
    ),
}
POSE = ["phix1", "phiy1", "phiz1"]


@pytest.mark.parametrize("name", list(FROZEN))
def test_frozen_files_unchanged(name):
    _, rows, sha = FROZEN[name]
    path = DATA / name
    assert hashlib.sha256(path.read_bytes()).hexdigest() == sha
    assert len(pd.read_csv(path)) == rows


def test_protocol_manifest_matches_the_freeze():
    manifest = yaml.safe_load((DATA / "protocol.yaml").read_text())
    got = {
        name: (s["role"], sha)
        for s in manifest["sessions"]
        for name, sha in s["files"].items()
    }
    assert got == {n: (role, sha) for n, (role, _, sha) in FROZEN.items()}
    assert manifest["heldout"]["split"] == "sessions"
    assert manifest["heldout"]["selection_role"] == "validation"


def test_only_the_training_session_measures_pose():
    for name, (role, _, _) in FROZEN.items():
        cols = set(pd.read_csv(DATA / name).columns)
        assert set(POSE) <= cols if role == "training" else not set(POSE) & cols


def test_gripper_column_of_position_only_sessions_is_zero():
    for name, (role, _, _) in FROZEN.items():
        if role != "training":
            df = pd.read_csv(DATA / name)
            assert (df["gripper_right_tool_mount_joint"] == 0).all()


def test_config_trains_on_the_protocol_training_file():
    cfg = yaml.safe_load((PRO / "tiago_pro_calibration_config.yaml").read_text())
    task = cfg["tasks"]["calibration"]
    training = next(n for n, (r, _, _) in FROZEN.items() if r == "training")
    assert Path(task["data"]["source_file"]).name == training
    assert (PRO / task["data"]["source_file"]).is_file()
    assert "estimation" in task["parameters"]  # explicit, not a core default
    assert task["parameters"]["estimation"]["method"] == "map_cv"


def test_two_sessions_share_14_postures():
    from examples.tiago_pro import data_audit as da

    a, b = pd.read_csv(DATA / da.SESSION_A), pd.read_csv(DATA / da.SESSION_B)
    ia, _ = da.repeated_postures(a, b)
    assert len(ia) == 14


def test_noise_estimate_from_repeat_postures():
    from examples.tiago_pro import data_audit as da

    n = da.reregistration_noise()
    assert n["n_pairs"] == 14
    assert n["sigma_mm"] == pytest.approx(1.35, abs=0.02)
    assert n["rotation_deg"] == pytest.approx(0.30, abs=0.02)
    assert da.noise_grid() == (0.5, 1.35, 3.0)


def test_data_readme_records_the_outlier_decision():
    text = (DATA / "README.md").read_text()
    assert "## Outlier decision" in text
    assert "no row is dropped" in text.lower()
    for row in ("row 15", "row 40"):
        assert row in text


@pytest.mark.slow
def test_outlier_scan_matches_the_recorded_decision():
    from examples.tiago_pro import data_audit as da

    scan = da.outlier_scan()
    assert scan["registration_only"]["flagged"] == {
        "position_mm": [],
        "orientation_deg": [],
    }
    full = scan["full_fit"]["flagged"]
    # the full-fit position flag sits at the threshold and depends on which
    # structural parameters the platform keeps; the orientation flag is stable
    assert full["position_mm"] in ([], [40])
    assert full["orientation_deg"] == [15]
    ev = scan["evidence"]["rows"]
    assert all(e["min_limit_margin"] > 0.05 for e in ev.values())
    assert ev[15]["sigma_min_rank"] <= 3  # near-singular
    assert ev[40]["sigma_min"] > 0.2


def test_tracked_pro_files_are_self_contained():
    files = subprocess.run(
        [
            "git",
            "ls-files",
            "examples/tiago_pro",
            "docs/development/tiago-pro-mocap.md",
        ],
        cwd=ROOT,
        capture_output=True,
        text=True,
    ).stdout.split()
    banned = ("robot-calibration-identification-dataset", "tools/audit")
    for f in files:
        path = ROOT / f
        if path.suffix in {".md", ".py", ".yaml", ".csv", ".json"} and path.is_file():
            text = path.read_text(errors="ignore")
            assert not any(b in text for b in banned), f
