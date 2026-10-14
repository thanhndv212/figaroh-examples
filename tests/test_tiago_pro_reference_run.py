"""TIAGo Pro calibration reference workflow (#97).

``reference_run.py`` fits with the configured model, reports every frozen
held-out session, verifies the fit and archives the run. The run directory
must hold every artifact and the held-out numbers must be the protocol's.
"""

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
PRO = ROOT / "examples" / "tiago_pro"
GOLDEN = json.loads((ROOT / "tests/golden/tiago_pro_mocap.json").read_text())

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
        [sys.executable, "reference_run.py", "--no-plot", "--root", str(root)],
        cwd=PRO,
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
    """Nothing is missing but, from a dirty checkout, the revisions.

    ``revisions`` is ``ok`` only on a clean checkout (as in
    test_run_record.py); a run from uncommitted code fails on it alone.
    """
    path, proc = run_dir
    sys.path.insert(0, str(ROOT))
    from examples.run_record import audit, missing

    gaps = missing(audit(path))
    assert set(gaps) <= {"revisions"}, gaps
    if gaps:
        assert proc.returncode == 1
        assert "archive incomplete: ['revisions']" in proc.stdout
    else:
        assert proc.returncode == 0, proc.stdout[-2000:]


@pytest.mark.slow
def test_fit_is_the_selected_model_and_converged(run_dir):
    path, _ = run_dir
    c = json.loads((path / "corrections.json").read_text())
    assert c["calibration_level"] == "full_params"
    assert c["estimation"]["method"] == "map_cv"
    assert c["solver"]["success"]
    assert {"base_px", "pEEx_1"} <= set(c["gauge"]["estimated_frames"])
    verdict = json.loads((path / "verdict.json").read_text())
    assert verdict["stages"]["solver"] == "pass"


@pytest.mark.slow
def test_heldout_matches_the_golden(run_dir):
    path, _ = run_dir
    heldout = json.loads((path / "heldout.json").read_text())
    gold = GOLDEN["models"]["full map_cv"]["sets"]
    assert set(heldout) == set(gold)
    for name, g in gold.items():
        assert heldout[name]["n"] == g["n"]
        assert heldout[name]["rmse_rereg"] == pytest.approx(g["rmse_rereg"], rel=1e-4)
        assert heldout[name]["rmse_transfer"] == pytest.approx(
            g["rmse_transfer"], rel=1e-4
        )
    assert {v["role"] for v in heldout.values()} == {"validation", "confirmation"}


def test_refuses_inputs_outside_the_frozen_protocol(monkeypatch):
    """A held-out session fitted on is a protocol mismatch."""
    from types import SimpleNamespace

    sys.path.insert(0, str(ROOT))
    monkeypatch.chdir(PRO)
    from examples.tiago_pro import reference_run as rr

    train = "data/calibration_samples_20260805_1246.csv"
    rr.check_protocol(SimpleNamespace(calib_config={"data_file": train}))
    with pytest.raises(ValueError, match="training session"):
        rr.check_protocol(
            SimpleNamespace(
                calib_config={"data_file": "data/calibration_samples_20260702_0756.csv"}
            )
        )
    with pytest.raises(ValueError, match="validation file"):
        rr.check_protocol(
            SimpleNamespace(
                calib_config={"data_file": train, "validation_data_file": "x.csv"}
            )
        )
