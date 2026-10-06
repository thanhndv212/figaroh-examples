"""TIAGo calibration reference workflow (C4, #29).

``reference_run.py`` fits, reports every frozen held-out session, exports the
URDF and PAL files, reloads them, and archives the run. The run directory
must hold every artifact, the reloaded models must predict the fit, and the
held-out numbers must be the protocol's.
"""

import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
TIAGO = ROOT / "examples" / "tiago"

ARTIFACTS = [
    "report.html",
    "verdict.json",
    "provenance.json",
    "reproduction.json",
    "calibrated.urdf",
    "master_calibration.yaml",
    "master_calibration_conservative.yaml",
    "heldout.json",
    "corrections.json",
    "export_check.json",
]


@pytest.fixture(scope="module")
def run_dir(tmp_path_factory):
    root = tmp_path_factory.mktemp("runs")
    env = dict(os.environ, MPLBACKEND="Agg")
    proc = subprocess.run(
        [sys.executable, "reference_run.py", "--root", str(root)],
        cwd=TIAGO,
        env=env,
        capture_output=True,
        text=True,
        timeout=600,
    )
    dirs = list(root.glob("*/calibration/*"))
    assert len(dirs) == 1, proc.stdout[-2000:] + proc.stderr[-2000:]
    return dirs[0], proc


def test_writes_every_artifact(run_dir):
    path, _ = run_dir
    for name in ARTIFACTS:
        assert (path / name).is_file(), name


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
        assert "verdict" not in proc.stdout.split("REFERENCE RUN FAILED:")[1]
    else:
        assert proc.returncode == 0, proc.stdout[-2000:]


def test_export_reloads_to_the_fit(run_dir):
    path, _ = run_dir
    export = json.loads((path / "export_check.json").read_text())
    assert export["passed"]
    assert export["worst_m"] < 1e-9
    assert export["other_changes"] == []
    verdict = json.loads((path / "verdict.json").read_text())
    assert verdict["stages"]["export"] == "pass"
    assert verdict["stages"]["solver"] == "pass"


def test_reports_gauge_and_corrections(run_dir):
    path, _ = run_dir
    c = json.loads((path / "corrections.json").read_text())
    assert c["calibration_level"] == "joint_offset"
    assert c["gauge"]["absorbed_by_frames"] == [
        "offsetPZ_torso_lift_joint",
        "offsetRZ_arm_1_joint",
    ]
    assert set(c["gauge"]["estimated_frames"]) >= {"base_px", "pEEx_1"}
    assert set(c["identifiable"]) == {f"offsetRZ_arm_{i}_joint" for i in range(2, 7)}
    assert c["solver"]["success"]


def test_heldout_matches_the_protocol(run_dir):
    path, _ = run_dir
    heldout = json.loads((path / "heldout.json").read_text())
    sys.path.insert(0, str(ROOT))
    cwd = os.getcwd()
    os.chdir(TIAGO)
    try:
        from examples.tiago import heldout_protocol as hp

        calib = hp.fit("joint_offset")
        for role, name in hp.SETS:
            expected = hp.component_errors(calib, hp.MOCAP / name)
            assert heldout[name]["role"] == role
            np.testing.assert_allclose(
                heldout[name]["rmse_mm"], expected["rmse"], rtol=1e-6
            )
    finally:
        os.chdir(cwd)
