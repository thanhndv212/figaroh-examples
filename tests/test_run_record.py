"""Reproduction record of archived runs (examples#30).

Unit tests audit a synthetic run directory; the integration test archives
one dynamic and one geometric TIAGo run and checks every item of the
checklist is present.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
import uuid
from pathlib import Path

import pytest

from examples.run_record import (
    CHECKLIST,
    add_artifacts,
    audit,
    describe_file,
    missing,
    write_reproduction_record,
)

ROOT = Path(__file__).resolve().parents[1]
TIAGO = ROOT / "examples" / "tiago"


def _archive(run_dir: Path, *, dirty=False, verdict=True) -> None:
    """What ``archive_run`` and the report/verdict exports leave behind."""
    run_dir.mkdir(parents=True)
    provenance = {
        "config": {"path": "config.yaml", "sha256": "c" * 64},
        "model": {"urdf_path": "robot.urdf", "urdf_sha256": "u" * 64},
        "data": {"data_file": {"path": "data.csv", "sha256": "d" * 64}},
        "software": {
            "git_commit": "e" * 40,
            "git_dirty": dirty,
            "figaroh_revision": {"commit": "f" * 40, "dirty": False},
        },
    }
    (run_dir / "provenance.json").write_text(json.dumps(provenance))
    (run_dir / "config.snapshot.yaml").write_text("a: 1\n")
    (run_dir / "parameters.csv").write_text("parameter,value\n")
    (run_dir / "report.html").write_text("<html></html>")
    stages = {"stages": [{"stage": "fit", "status": "ok", "reason": "converged"}]}
    (run_dir / "stages.json").write_text(json.dumps(stages))
    if verdict:
        (run_dir / "verdict.json").write_text(
            json.dumps(
                {"selected_stage": "fit", "splits": {"validation_source": "held_out"}}
            )
        )


def test_archive_without_record_names_what_is_missing(tmp_path):
    _archive(tmp_path / "run", verdict=False)
    assert missing(audit(tmp_path / "run")) == [
        "invocation",
        "processing",
        "splits",
        "selected_stage",
    ]


def test_record_completes_the_checklist(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    _archive(tmp_path / "run")
    Path("protocol.yaml").write_text("version: 1\n")
    Path("results.npz").write_bytes(b"npz")
    record = write_reproduction_record(
        tmp_path / "run",
        processing={"truncate": [0, 10]},
        inputs={"protocol": describe_file("protocol.yaml")},
        artifacts={"results": describe_file("results.npz")},
        argv=["calibration.py", "--no-plot"],
    )
    assert set(record["checklist"]) == set(CHECKLIST)
    assert record["missing"] == []
    saved = json.loads((tmp_path / "run" / "reproduction.json").read_text())
    assert saved["invocation"]["argv"] == ["calibration.py", "--no-plot"]


def test_changed_or_absent_files_are_incomplete(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    _archive(tmp_path / "run")
    Path("results.npz").write_bytes(b"npz")
    write_reproduction_record(
        tmp_path / "run",
        processing={"x": 1},
        inputs={"protocol": describe_file("absent.yaml")},
        artifacts={"results": describe_file("results.npz")},
        argv=["x"],
    )
    Path("results.npz").write_bytes(b"changed")
    checks = audit(tmp_path / "run")
    assert checks["inputs"]["status"] == "incomplete"
    assert "protocol" in checks["inputs"]["detail"]
    assert checks["export"]["status"] == "incomplete"
    assert "results" in checks["export"]["detail"]


@pytest.mark.parametrize("problem", [None, "changed", "missing", "outside", "no_files"])
def test_validation_directory_requires_matching_consumed_file_hashes(
    tmp_path, monkeypatch, problem
):
    monkeypatch.chdir(tmp_path)
    run_dir = tmp_path / "run"
    _archive(run_dir)
    folder = tmp_path / "validation"
    folder.mkdir()
    files = {}
    for name in ("position.csv", "velocity.csv", "effort.csv"):
        path = folder / name
        path.write_text("t,q\n0,1\n")
        files[str(path)] = describe_file(path)["sha256"]
    provenance = json.loads((run_dir / "provenance.json").read_text())
    provenance["data"]["validation_data_file"] = {"path": "validation", "sha256": None}
    (run_dir / "provenance.json").write_text(json.dumps(provenance))
    verdict = json.loads((run_dir / "verdict.json").read_text())
    verdict["splits"]["validation"] = {"files": files}
    if problem == "changed":
        (folder / "position.csv").write_text("t,q\n0,2\n")
    elif problem == "missing":
        (folder / "position.csv").unlink()
    elif problem == "outside":
        outside = tmp_path / "outside.csv"
        outside.write_text("t,q\n0,1\n")
        files[str(outside)] = describe_file(outside)["sha256"]
    elif problem == "no_files":
        verdict["splits"]["validation"]["files"] = {}
    (run_dir / "verdict.json").write_text(json.dumps(verdict))
    record = write_reproduction_record(run_dir, processing={"x": 1}, argv=["x"])
    expected = "ok" if problem is None else "incomplete"
    assert record["checklist"]["inputs"]["status"] == expected


def test_uncommitted_changes_make_revisions_incomplete(tmp_path):
    _archive(tmp_path / "run", dirty=True)
    checks = audit(tmp_path / "run")
    assert checks["revisions"]["status"] == "incomplete"
    assert "examples" in checks["revisions"]["detail"]


def test_artifacts_added_after_the_record_are_audited(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    _archive(tmp_path / "run")
    write_reproduction_record(tmp_path / "run", processing={"x": 1}, argv=["x"])
    Path("modified.urdf").write_text("<robot/>")
    record = add_artifacts(tmp_path / "run", {"urdf": describe_file("modified.urdf")})
    assert record["artifacts"]["urdf"]["sha256"]
    assert record["missing"] == []


@pytest.mark.slow
@pytest.mark.integration
def test_tiago_reference_runs_archive_the_full_record():
    """One dynamic and one geometric TIAGo run: nothing missing but revisions.

    ``revisions`` is ``ok`` only on a clean checkout, so it is checked to be
    present, not clean.
    """
    asset = f"test-{uuid.uuid4().hex[:8]}"
    env = dict(os.environ, MPLBACKEND="Agg")
    runs = {
        "identification": ["identification.py"],
        "calibration": ["calibration.py", "--calibrate-only", "--no-plot"],
    }
    try:
        for task, args in runs.items():
            proc = subprocess.run(
                [sys.executable, *args, "--asset-id", asset],
                cwd=TIAGO,
                env=env,
                capture_output=True,
                text=True,
                timeout=300,
                stdin=subprocess.DEVNULL,
            )
            assert proc.returncode == 0, proc.stderr[-2000:]
            (run_dir,) = (TIAGO / "results" / "runs" / asset / task).iterdir()
            record = json.loads((run_dir / "reproduction.json").read_text())
            assert set(record["missing"]) <= {"revisions"}, record["checklist"]
            assert record["checklist"]["revisions"]["status"] != "missing"
            if task == "calibration":
                roles = record["processing"]["data_roles"]
                assert roles["data_file"]["role"] == "training"
                assert roles["validation_data_file"]["role"] == "validation"
                npz = Path(record["artifacts"]["results"]["path"])
                (TIAGO / npz).unlink()
    finally:
        shutil.rmtree(TIAGO / "results" / "runs" / asset, ignore_errors=True)
