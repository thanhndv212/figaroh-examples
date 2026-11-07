"""LC1 (#11): log-Cholesky against OLS and OLS+SDP on recorded TIAGo data.

RECORDED data, frozen cross-run protocol v1. The fast tests pin the overlays
and the gate logic; the slow smoke run fits every method on the training
session and scores it on the independent held-out session, writing only into
a temporary directory.
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
TIAGO = ROOT / "examples" / "tiago"
sys.path.insert(0, str(ROOT))

HAS_LC = importlib.util.find_spec("figaroh.identification.log_cholesky_fit") is not None
needs_lc = pytest.mark.skipif(
    not HAS_LC, reason="core has no log_cholesky_fit (figaroh-plus devel >= 21994ed)"
)


def _load(name):
    return yaml.safe_load((TIAGO / "config" / name).read_text())


def test_overlays_share_one_policy_and_differ_only_in_the_stage():
    ols = _load("tiago_lc1_ols.yaml")
    sdp = _load("tiago_lc1_ols_sdp.yaml")
    lc = _load("tiago_lc1_log_cholesky.yaml")
    assert ols["extends"] == "tiago_unified_config.yaml"
    assert sdp["extends"] == lc["extends"] == "tiago_lc1_ols.yaml"
    assert ols["tasks"]["identification"]["select_stage"] == "fit"
    assert ols["tasks"]["identification"]["reconstruction"]["enabled"] is True
    assert set(sdp["tasks"]["identification"]) == {"physical_consistency"}
    ident = lc["tasks"]["identification"]
    assert ident["select_stage"] == "physical_fit"
    fit = ident["physical_fit"]
    assert fit["method"] == "log_cholesky"
    assert (fit["mass_min"], fit["prior_weight"]) == (1e-6, 1e-6)
    assert fit["log_cholesky"]["branch"] == "auto"
    assert fit["log_cholesky"]["repair_start"] is False


def _fake(lc_rmse=1.0, sdp_rmse=1.0, sdp_status="ok", min_eig=0.0, fit_s=1.0, start_ok=True):
    start = {"converged": start_ok, "accepted": start_ok, "start_used": False}
    return {
        "methods": {
            "log_cholesky": {"heldout": {"aggregate": lc_rmse}, "status": "accepted",
                             "min_eig": min_eig, "fit_runtime_s": fit_s},
            "ols_sdp": {"heldout": {"aggregate": sdp_rmse}, "status": sdp_status},
        },
        "starts_auto": {"nominal": dict(start), "repaired_ols": dict(start)},
    }


@pytest.mark.parametrize(
    "kw, failing",
    [
        ({}, None),
        ({"lc_rmse": 1.06}, "heldout_ratio"),
        ({"sdp_status": "failed"}, "heldout_ratio"),
        ({"min_eig": -1e-6}, "feasibility"),
        ({"fit_s": 21.0}, "fit_runtime"),
        ({"start_ok": False}, "gated_starts"),
    ],
)
def test_gates_have_no_truth_terms_and_each_can_fail(kw, failing):
    from examples.tiago import log_cholesky_benchmark as b

    g = b.gates(_fake(**kw))
    assert set(k for k, v in g.items() if isinstance(v, dict)) == {
        "heldout_ratio", "feasibility", "fit_runtime", "gated_starts"}
    assert "truth" in g["not_applicable"]
    unmet = [k for k, v in g.items() if isinstance(v, dict) and not v["met"]]
    assert unmet == ([] if failing is None else [failing])
    assert g["all_met"] is (failing is None)


def test_mass_plausibility_picks_the_largest_log_ratio():
    from examples.tiago import log_cholesky_benchmark as b

    links = {
        "a": {"nominal_mass": 2.0, **{m: {"mass": 2.0} for m in b.METHODS}},
        "b": {"nominal_mass": 1.0, **{m: {"mass": 0.1 if m == "ols" else 30.0}
                                      for m in b.METHODS}},
    }
    mp = b.mass_plausibility(links)
    assert mp["ols"] == {"link": "b", "ratio": pytest.approx(0.1)}
    assert mp["log_cholesky"] == {"link": "b", "ratio": pytest.approx(30.0)}


@pytest.fixture(scope="module")
def smoke(tmp_path_factory):
    out = tmp_path_factory.mktemp("lc1-tiago")
    env = dict(os.environ, MPLBACKEND="Agg")
    proc = subprocess.run(
        [sys.executable, "log_cholesky_benchmark.py", "--smoke", "--output-dir", str(out)],
        cwd=TIAGO, env=env, capture_output=True, text=True, timeout=600,
    )
    return proc, out


@needs_lc
@pytest.mark.slow
def test_smoke_on_recorded_data_meets_the_gates(smoke):
    proc, out = smoke
    assert proc.returncode == 0, proc.stdout[-2000:] + proc.stderr[-2000:]
    assert (out / "lc1_benchmark.md").is_file()
    r = json.loads((out / "lc1_benchmark.json").read_text())
    pv = r["provenance"]
    assert pv["data"].startswith("RECORDED")
    assert pv["hardware_validated"] is False and pv["urdf_exported"] is False
    assert pv["model"]["fitted_joints"] == [f"arm_{i}_joint" for i in range(1, 5)]
    assert r["preflight"]["passed"]
    assert r["preflight"]["heldout_role"] == "validation"
    assert r["preflight"]["train"]["rows"] != r["preflight"]["heldout"]["rows"]
    assert r["gates"]["all_met"]


@needs_lc
@pytest.mark.slow
def test_smoke_scores_every_method_on_the_same_heldout_rows(smoke):
    _, out = smoke
    r = json.loads((out / "lc1_benchmark.json").read_text())
    m = r["methods"]
    n = {k: m[k]["heldout"]["n_samples"] for k in m}
    assert len(set(n.values())) == 1, n
    assert m["ols"]["min_eig"] < 0 and m["ols"]["infeasible_links"]
    for k in ("ols_sdp", "log_cholesky", "direct"):
        assert m[k]["min_eig"] >= -1e-8 and not m[k]["infeasible_links"], k
    lc = m["log_cholesky"]
    assert lc["selected_stage"] == "physical_fit" and lc["branch"] in ("boundary", "interior")
    # The selected estimate's own validation metrics equal the rescored ones.
    assert lc["selected_prediction_gap_nm"] < 1e-9
    assert set(r["mass_plausibility"]) == set(m)
