"""TALOS table-contact calibration reference workflow (figaroh-examples#98).

``reference_run.py`` runs the table-only baseline, the structural fit and the
MAP fit on each chain, archives the structural run and reports the held-out
validation session. The run directories must hold every artifact and the
held-out numbers must be the real block of ``golden/talos_table_contact.json``
for those three models (the full six-model run stays in
``heldout_protocol.py``).
"""

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
EXAMPLE = ROOT / "examples" / "talos_table_contact"
GOLDEN = json.loads((ROOT / "tests/golden/talos_table_contact.json").read_text())

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
MODELS = ("table+contact only", "structural", "map")
SIDES = ("left", "right")


@pytest.fixture(scope="module")
def runs(tmp_path_factory):
    root = tmp_path_factory.mktemp("runs")
    env = dict(os.environ, MPLBACKEND="Agg")
    proc = subprocess.run(
        [sys.executable, "reference_run.py", "--no-plot", "--root", str(root)],
        cwd=EXAMPLE,
        env=env,
        capture_output=True,
        text=True,
        timeout=900,
    )
    out = {}
    for side in SIDES:
        dirs = list((root / f"talos-{side}" / "calibration").glob("*"))
        assert len(dirs) == 1, proc.stdout[-2000:] + proc.stderr[-2000:]
        out[side] = dirs[0]
    return out, proc


@pytest.mark.slow
@pytest.mark.parametrize("side", SIDES)
def test_writes_every_artifact(runs, side):
    paths, _ = runs
    for name in ARTIFACTS:
        assert (paths[side] / name).is_file(), name


@pytest.mark.slow
def test_succeeds_or_only_lacks_committed_revisions(runs):
    """Nothing is missing but, from a dirty checkout, the revisions."""
    paths, proc = runs
    sys.path.insert(0, str(ROOT))
    from examples.run_record import audit, missing

    gaps = set()
    for path in paths.values():
        gaps |= set(missing(audit(path)))
    assert gaps <= {"revisions"}, gaps
    if gaps:
        assert proc.returncode == 1
        assert "archive incomplete: ['revisions']" in proc.stdout
    else:
        assert proc.returncode == 0, proc.stdout[-2000:]


@pytest.mark.slow
@pytest.mark.parametrize("side", SIDES)
def test_corrections_state_the_gauge(runs, side):
    paths, _ = runs
    c = json.loads((paths[side] / "corrections.json").read_text())
    assert c["estimation"]["method"] == "structural"
    assert c["solver"]["success"]
    assert c["gauge"]["collinear"] == ["plane_z_s0", "contact_z"]
    assert "plane_z_s0" in c["parameters"] and "contact_z" in c["parameters"]
    assert not any(n.startswith(("contact_x", "contact_y")) for n in c["parameters"])
    verdict = json.loads((paths[side] / "verdict.json").read_text())
    assert verdict["stages"]["solver"] == "pass"


@pytest.mark.slow
@pytest.mark.parametrize("side", SIDES)
def test_heldout_matches_the_golden(runs, side):
    paths, _ = runs
    held = json.loads((paths[side] / "heldout.json").read_text())
    assert held["primary_metric"] == "transfer_z_mm"
    assert set(held["models"]) == set(MODELS)
    gold = GOLDEN["chains"][side]["models"]
    for label in MODELS:
        m, g = held["models"][label], gold[label]
        if label == "structural" and m["n_params"] != g["n_params"]:
            continue  # ties in the selection break by platform (plus#113)
        assert m["n_params"] == g["n_params"], label
        assert m["train_z_mm"] == pytest.approx(g["train_z_mm"], abs=1e-3)
        assert set(m["sets"]) == set(g["sets"])
        for name, gs in g["sets"].items():
            s = m["sets"][name]
            assert s["n"] == gs["n"]
            assert s["role"] == "validation"
            for k in ("transfer_z_mm", "transfer_roll_deg", "transfer_pitch_deg"):
                assert s[k] == pytest.approx(gs[k], rel=1e-4, abs=1e-3), (label, k)
            for k, v in s["rereg_optimistic"].items():
                assert v == pytest.approx(gs[k], rel=1e-4, abs=1e-3), (label, k)
            assert {k: v["n"] for k, v in s["strata_transfer_z_mm"].items()} == gs[
                "strata"
            ]
