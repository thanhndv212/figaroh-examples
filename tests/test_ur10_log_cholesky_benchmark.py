"""LC1 (#11): log-Cholesky against OLS and OLS+SDP on the UR10 truth fixture.

The smoke run is bounded (one low-noise dataset, three starts) and writes
into a temporary directory; the fixture is read only. SIMULATED data.
"""

import importlib.util
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
UR10 = ROOT / "examples" / "ur10"
sys.path.insert(0, str(ROOT))

HAS_LC = importlib.util.find_spec("figaroh.identification.log_cholesky_fit") is not None
needs_lc = pytest.mark.skipif(
    not HAS_LC, reason="core has no log_cholesky_fit (figaroh-plus devel >= 21994ed)"
)


def _load(path):
    return yaml.safe_load(Path(path).read_text())


def test_overlays_share_one_policy_and_differ_only_in_the_stage():
    ols = _load(UR10 / "config/ur10_lc1_ols.yaml")
    sdp = _load(UR10 / "config/ur10_lc1_ols_sdp.yaml")
    lc = _load(UR10 / "config/ur10_lc1_log_cholesky.yaml")
    assert ols["extends"] == "ur10_unified_config.yaml"
    assert sdp["extends"] == lc["extends"] == "ur10_lc1_ols.yaml"
    assert ols["tasks"]["identification"] == {
        "select_stage": "fit",
        "reconstruction": {"enabled": True, "method": "nullspace"},
    }
    assert set(sdp["tasks"]["identification"]) == {"physical_consistency"}
    ident = lc["tasks"]["identification"]
    assert ident["select_stage"] == "physical_fit"
    fit = ident["physical_fit"]
    assert fit["method"] == "log_cholesky"
    assert (fit["mass_min"], fit["prior_weight"]) == (1e-6, 1e-6)
    assert fit["log_cholesky"]["branch"] == "auto"
    assert fit["log_cholesky"]["repair_start"] is False


@needs_lc
def test_overlays_parse_through_the_pipeline(monkeypatch):
    from examples.ur10 import identification_truth as truth

    monkeypatch.chdir(UR10)
    ident = truth.identification(config=UR10 / "config/ur10_lc1_log_cholesky.yaml")
    cfg = ident.identif_config
    assert cfg["select_stage"] == "physical_fit"
    assert cfg["physical_fit"]["method"] == "log_cholesky"
    assert cfg["reconstruction"]["enabled"] is True


@pytest.fixture(scope="module")
def smoke(tmp_path_factory):
    out = tmp_path_factory.mktemp("lc1")
    env = dict(
        os.environ,
        MPLBACKEND="Agg",
        OMP_NUM_THREADS="1",
        OPENBLAS_NUM_THREADS="1",
        VECLIB_MAXIMUM_THREADS="1",
    )
    proc = subprocess.run(
        [sys.executable, "log_cholesky_benchmark.py", "--smoke", "--output-dir", str(out)],
        cwd=UR10,
        env=env,
        capture_output=True,
        text=True,
        timeout=300,
    )
    return proc, out


@needs_lc
def test_smoke_meets_the_gates_and_archives_both_reports(smoke):
    proc, out = smoke
    assert proc.returncode == 0, proc.stdout[-2000:] + proc.stderr[-2000:]
    assert (out / "lc1_benchmark.md").is_file()
    report = json.loads((out / "lc1_benchmark.json").read_text())
    assert report["provenance"]["data"].startswith("SIMULATED")
    assert report["provenance"]["hardware_validated"] is False
    assert report["preflight"]["passed"]
    assert report["preflight"]["pipeline_ddq_is_fixture_ddq"]
    g = report["gates"]
    assert g["noisy_heldout_ratio"]["met"] and g["fit_runtime"]["met"]
    assert g["gated_starts"]["met"]
    assert g["clean_heldout_rmse"]["met"] is None  # smoke has no noise-free run


@needs_lc
def test_smoke_methods_and_diagnostics(smoke):
    _, out = smoke
    report = json.loads((out / "lc1_benchmark.json").read_text())
    (row,) = report["datasets"]
    m = row["methods"]
    lc, direct, sdp = m["log_cholesky"], m["direct"], m["ols_sdp"]
    assert lc["branch"] == "boundary" and lc["status"] == "accepted"
    assert lc["min_eig"] > 0 and sdp["min_eig"] > 0 and m["ols"]["min_eig"] < 0
    # the selected stage is what the pipeline predicts with
    assert lc["selected_prediction_gap_nm"] < 1e-9
    # boundary branch: log-Cholesky is the direct physical fit
    assert lc["heldout_vs_measurement"]["aggregate"] == pytest.approx(
        direct["heldout_vs_measurement"]["aggregate"], rel=1e-6
    )
    assert lc["train_objective"] <= sdp["train_objective"]
    forced = row["starts_forced_interior"]
    assert forced["unrepaired_ols"]["solver_status"] == "invalid_start"
    assert not forced["nominal"]["converged"]  # boundary optimum: unreachable
    oracle = report["oracle_interior"]
    assert oracle["detected_branch"] == "interior"
    for start in ("nominal", "repaired_ols"):
        s = oracle["starts"][start]
        assert s["converged"] and s["start_used"]
        assert s["max_abs_diff_to_reference"] < 1e-6


@needs_lc
def test_reference_case_selects_exports_and_reports_log_cholesky(tmp_path):
    proc = subprocess.run(
        [sys.executable, "identification_reference.py", "--case", "log-cholesky",
         "--root", str(tmp_path)],
        cwd=UR10,
        env=dict(os.environ, MPLBACKEND="Agg"),
        capture_output=True,
        text=True,
        timeout=300,
    )
    assert proc.returncode == 0, proc.stdout[-2000:] + proc.stderr[-2000:]
    (run,) = sorted(tmp_path.glob("*/identification/*"))
    verdict = json.loads((run / "verdict.json").read_text())
    assert verdict["selected_stage"] == "physical_fit" and verdict["status"] == "pass"
    report = json.loads((run / "reference.json").read_text())
    assert report["log_cholesky"]["branch"] == "boundary"
    assert report["physical"]["status"] == "accepted"
    assert (run / "identified.urdf").is_file()
    assert "log_cholesky" in (run / "report.html").read_text()
