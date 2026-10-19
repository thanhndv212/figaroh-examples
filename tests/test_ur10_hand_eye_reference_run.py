"""UR10 hand-eye reference workflow (#99).

``hand_eye_reference_run.py`` fits all 23 postures with the configured
method, reports the k-fold error and archives the run. The run directory
must hold every artifact and the held-out numbers must be the golden's.
"""

import json
import os
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
UR10 = ROOT / "examples" / "ur10"
GOLDEN = ROOT / "tests" / "golden" / "ur10_hand_eye.json"
sys.path.insert(0, str(ROOT))

ARTIFACTS = [
    "report.html",
    "verdict.json",
    "provenance.json",
    "reproduction.json",
    "heldout.json",
    "corrections.json",
    "parameters.csv",
    "config.snapshot.yaml",
]


@pytest.fixture(scope="module")
def run_dir(tmp_path_factory):
    root = tmp_path_factory.mktemp("runs")
    env = dict(os.environ, MPLBACKEND="Agg")
    proc = subprocess.run(
        [sys.executable, "hand_eye_reference_run.py", "--no-plot", "--root", str(root)],
        cwd=UR10,
        env=env,
        capture_output=True,
        text=True,
        timeout=600,
    )
    dirs = list(root.glob("*/calibration/*"))
    assert len(dirs) == 1, proc.stdout[-2000:] + proc.stderr[-2000:]
    return dirs[0], proc


@pytest.mark.slow
def test_writes_every_artifact(run_dir):
    path, _ = run_dir
    for name in ARTIFACTS:
        assert (path / name).is_file(), name


@pytest.mark.slow
def test_succeeds_or_only_lacks_committed_revisions(run_dir):
    """From a dirty checkout the run fails on ``revisions`` alone."""
    path, proc = run_dir
    from examples.run_record import audit, missing

    gaps = missing(audit(path))
    assert set(gaps) <= {"revisions"}, gaps
    if gaps:
        assert proc.returncode == 1
        assert "archive incomplete: ['revisions']" in proc.stdout
    else:
        assert proc.returncode == 0, proc.stdout[-2000:]


@pytest.mark.slow
def test_fit_uses_the_config_and_heldout_matches_golden(run_dir):
    path, _ = run_dir
    c = json.loads((path / "corrections.json").read_text())
    assert c["calibration_level"] == "full_params"
    assert c["estimation"]["method"] == "structural"
    assert c["solver"]["success"]
    assert {"base_px", "pEEx_1"} <= set(c["gauge"]["estimated_frames"])
    held = json.loads((path / "heldout.json").read_text())
    assert "k-fold within session" in held["label"]
    gold = json.loads(GOLDEN.read_text())["kfold"]["models"]
    for model, row in held["models"].items():
        assert row["rmse_pos_mm"] == pytest.approx(gold[model]["rmse_pos_mm"], abs=2e-3)
        assert row["rmse_rot_deg"] == pytest.approx(
            gold[model]["rmse_rot_deg"], abs=2e-3
        )


def test_refuses_inputs_outside_the_frozen_protocol():
    from examples.ur10 import hand_eye_heldout as heh
    from examples.ur10 import hand_eye_reference_run as rr

    ok = {"data_file": str(heh.TRAINING)}
    rr.check_protocol(SimpleNamespace(calib_config=ok))
    with pytest.raises(ValueError, match="protocol's session"):
        rr.check_protocol(
            SimpleNamespace(
                calib_config={"data_file": str(UR10 / "data" / "simulation.csv")}
            )
        )
    with pytest.raises(ValueError, match="validation file"):
        rr.check_protocol(
            SimpleNamespace(calib_config=dict(ok, validation_data_file="x.csv"))
        )


def test_config_method_maps_to_a_heldout_model():
    from examples.ur10 import hand_eye_heldout as heh
    from examples.ur10 import hand_eye_reference_run as rr

    for method, label in rr.LABELS.items():
        cfg = {"estimation": {"method": method}}
        assert rr.config_model(SimpleNamespace(calib_config=cfg)) == label
        assert label in heh.MODELS
    with pytest.raises(ValueError, match="no held-out model"):
        rr.config_model(SimpleNamespace(calib_config={"estimation": {"method": "x"}}))
