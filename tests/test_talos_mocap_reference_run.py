"""TALOS torso-arm mocap reference workflow (#172).

``mocap_reference_run.py`` fits the configured model (the one the k-fold
plan selected), reports both right-arm runs of the frozen protocol, verifies
the fit and archives the run. The run directory must hold every artifact
and the held-out numbers must be the golden's for the selected model.
"""

import json
import os
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
TALOS = ROOT / "examples" / "talos"
GOLDEN = json.loads((ROOT / "tests/golden/talos_mocap.json").read_text())

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
        [sys.executable, "mocap_reference_run.py", "--no-plot", "--root", str(root)],
        cwd=TALOS,
        env=env,
        capture_output=True,
        text=True,
        timeout=900,
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
    """Nothing is missing but, from a dirty checkout, the revisions."""
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
def test_fit_is_the_selected_model_and_converged(run_dir):
    from examples.talos import mocap_heldout as mh

    path, _ = run_dir
    level, _, est = mh.MODELS[GOLDEN["selected"]]
    c = json.loads((path / "corrections.json").read_text())
    assert c["calibration_level"] == level
    assert c["estimation"]["method"] == est["method"]
    assert c["solver"]["success"]
    assert {"base_px", "pEEx_1"} <= set(c["gauge"]["estimated_frames"])


@pytest.mark.slow
def test_heldout_matches_the_golden(run_dir):
    path, _ = run_dir
    heldout = json.loads((path / "heldout.json").read_text())
    gold = GOLDEN["models"][GOLDEN["selected"]]["sets"]
    assert set(heldout) == set(gold)
    for name, g in gold.items():
        e = heldout[name]
        assert (e["role"], e["n"], e["n_flipped"]) == (g["role"], g["n"], g["n_flipped"])
        for k in ("pos_rmse_rereg_mm", "pos_rmse_waist_mm"):
            assert e[k] == pytest.approx(g[k], rel=1e-4, abs=1e-3), (name, k)
    assert {v["role"] for v in heldout.values()} == {"validation", "confirmation"}


def test_refuses_inputs_outside_the_frozen_protocol():
    """A right-arm run, or a validation file, fitted on is a mismatch."""
    from examples.talos import mocap_heldout as mh
    from examples.talos import mocap_reference_run as rr

    rr.check_protocol(SimpleNamespace(calib_config={"data_file": mh.TRAINING}))
    with pytest.raises(ValueError, match="training run"):
        rr.check_protocol(SimpleNamespace(calib_config={"data_file": mh.SETS[1][1]}))
    with pytest.raises(ValueError, match="validation file"):
        rr.check_protocol(
            SimpleNamespace(
                calib_config={"data_file": mh.TRAINING, "validation_data_file": "x.csv"}
            )
        )
