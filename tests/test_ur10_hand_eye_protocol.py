"""UR10 hand-eye data and protocol freeze (#99).

The 23 postures are one session: the protocol is a k-fold split of its rows
("k-fold within session"), frozen with the file's sha256.
"""

import hashlib
import json
import subprocess
import sys
from pathlib import Path

import pandas as pd
import yaml

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "examples" / "ur10" / "data"
PROTOCOL = DATA / "hand_eye_protocol.yaml"
sys.path.insert(0, str(ROOT))

SHA = "31d5a3d56cbb5ea3f69775a3f2620bbe31cb750e68c6a2258ed585c3f01cdfd6"
JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def test_frozen_file_unchanged():
    path = DATA / "calibration.csv"
    assert hashlib.sha256(path.read_bytes()).hexdigest() == SHA
    assert len(pd.read_csv(path)) == 23


def test_manifest_matches_the_freeze():
    manifest = yaml.safe_load(PROTOCOL.read_text())
    (session,) = manifest["sessions"]
    assert session["files"] == {"calibration.csv": SHA}
    assert manifest["heldout"]["split"] == "kfold"
    assert manifest["heldout"]["kfold"]["k"] == 5
    assert tuple(manifest["heldout"]["strata"]["joints"]) == tuple(JOINTS)


def test_plan_verifies_and_holds_each_row_out_once():
    from examples.calibration_harness import heldout as harness

    plan = harness.load_plan(PROTOCOL)  # raises on a sha256 mismatch
    assert plan.split == "kfold"
    assert "k-fold within session" in plan.label
    assert len(plan.folds) == 5
    held = [i for f in plan.folds for s in f.evaluated for i in s.rows]
    assert sorted(held) == list(range(23))
    for f in plan.folds:
        (train,) = f.training
        (test,) = f.evaluated
        assert not set(train.rows) & set(test.rows)
        assert len(train.rows) + len(test.rows) == 23


def test_inventory_lists_the_protocol():
    inv = json.loads((ROOT / "docs" / "data-inventory.json").read_text())
    ds = {d["id"]: d for d in inv["datasets"]}["ur10-hand-eye"]
    assert "examples/ur10/data/hand_eye_protocol.yaml" in ds["paths"]
    out = subprocess.run(
        [sys.executable, str(ROOT / "scripts" / "data_inventory.py"), "check"],
        capture_output=True,
        text=True,
    )
    assert out.returncode == 0, out.stderr
