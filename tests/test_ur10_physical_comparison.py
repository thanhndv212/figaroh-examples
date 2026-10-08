"""Smoke test of the UR10 physical-estimator comparison (#22).

Noise-free, unweighted, analytic derivatives only: every estimator must be
consistent with the truth fixture, the results must be JSON-serialisable and
separate convergence from feasibility, and ``diagnose_exact`` must not call a
feasible problem infeasible. Needs core's private comparator
(feat/59-physical-comparator) and picos/cvxopt; skipped otherwise.
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pytest

UR10 = Path(__file__).resolve().parents[1] / "examples" / "ur10"
sys.path.insert(0, str(UR10.parents[1]))

pytest.importorskip("picos")
pytest.importorskip("cvxopt")
pytest.importorskip("figaroh.identification._physical_comparator")

from examples.ur10 import compare_physical_estimators as cmp  # noqa: E402


@pytest.fixture(scope="module")
def result():
    args = argparse.Namespace(
        derivatives=["analytic"], noise=["none"], seeds=1, weighting=["none"],
        solver="cvxopt", d0_timeout=5.0,
    )
    return cmp.run(args)


def test_methods_recover_truth_without_noise(result):
    methods = result["cases"][0]["frozen_extra"]["methods"]
    assert set(methods) == {
        "base_ols", "exact_reconstruction", "direct_effort_fit",
        "per_link_projection",
    }
    for name in ("base_ols", "exact_reconstruction"):
        assert methods[name]["base_error"]["max_nm"] < 1e-6, name
        assert max(methods[name]["heldout"]["nrmse_pct"].values()) < 1e-4, name
    assert max(methods["direct_effort_fit"]["heldout"]["nrmse_pct"].values()) < 1e-1
    for name in ("exact_reconstruction", "direct_effort_fit", "per_link_projection"):
        m = methods[name]
        assert m["convergence"]["solver_status"] == "optimal"
        assert m["feasibility"]["all_links_ok"], name
        assert m["feasibility"]["min_eig_min"] > -1e-8  # comparator feas_tol
        assert m["fallback_used"] is False


def test_joint_extra_ols_finds_no_extras(result):
    jx = result["cases"][0]["joint_extra"]["base_ols"]
    assert jx["extras_abs_max"] < 1e-6


def test_exact_diagnosis_needs_a_certificate(result):
    d = result["diagnose_exact"][0]
    assert d["phase1_s_star"] > cmp.CERT_TOL
    assert d["verdict"] != "genuine infeasibility"
    assert d["direct_exact_accepted"] is True


def test_provenance_and_json(result):
    prov = result["provenance"]
    assert prov["fixture_manifest_check"] == []
    assert len(prov["hashes"]["raw"]) == 2
    assert prov["core"]["commit"] != "unknown"
    json.dumps(result)
    assert np.isfinite(result["runtime_s"])
