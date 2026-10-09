"""Second pass of the physical-estimator comparison (#22): log-Cholesky,
joint-extra and the second-solver cross-check.

Skipped without core's private comparator and the picos/cvxopt solvers.
"""

import json
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

pytest.importorskip("picos")
pytest.importorskip("cvxopt")
pcmp = pytest.importorskip("figaroh.identification._physical_comparator")

from examples import physical_comparison as pc  # noqa: E402


def synthetic_spec(rng, n=240, links=2, extras=3):
    """Random but exactly consistent: effort from a physical truth plus extras."""
    names, prior = [], []
    for k in range(links):
        names += [f"{key}_L{k}" for key in pc.P10]
        prior += [2.0 + k, 0.1, 0.05, 0.2, 0.3, 0.01, 0.3, 0.02, 0.01, 0.2]
    prior = np.array(prior)
    p = len(prior)
    Y = rng.standard_normal((n * 2, p))
    E = rng.standard_normal((n * 2, extras))
    truth_e = np.array([0.5, -0.2, 0.1])
    tau = Y @ prior + E @ truth_e + 1e-3 * rng.standard_normal(n * 2)
    Yv = rng.standard_normal((n, p))
    Ev = rng.standard_normal((n, extras))
    return pc.Spec(
        Y=Y,
        E=E,
        tau=tau,
        Yv=Yv,
        Ev=Ev,
        names=names,
        extra_names=[f"e{i}" for i in range(extras)],
        links=[f"L{k}" for k in range(links)],
        joints=["j0", "j1"],
        prior=prior,
        e_fix=np.zeros(extras),
        heldout={"measured_effort": Yv @ prior + Ev @ truth_e},
        nrmse_scale=np.ones(2),
    )


def test_joint_extra_equals_full_least_squares():
    spec = synthetic_spec(np.random.default_rng(0))
    b = pc.build(spec, pcmp, "joint")
    assert b.kept == [0, 1, 2] and b.absorbed == []
    theta = pc.ols_representative(b.p)
    e = pc.extras_for(spec, b, theta)
    full, *_ = np.linalg.lstsq(np.c_[spec.Y, spec.E], spec.tau, rcond=None)
    np.testing.assert_allclose(
        spec.Y @ theta + spec.E @ e, np.c_[spec.Y, spec.E] @ full, atol=1e-8
    )


def test_methods_objectives_and_separate_fields():
    spike, _ = pc.load_spike(pcmp)
    spec = synthetic_spec(np.random.default_rng(1))
    for mode in ("frozen", "joint"):
        b, methods = pc.run_methods(spec, mode, pcmp, spike, "cvxopt")
        direct = methods["direct_effort_fit"]
        assert direct["convergence"]["converged"]
        assert direct["objective"]["relative_excess_vs_direct"] == 0.0
        for start, lc in methods["log_cholesky"].items():
            # never fewer fields: convergence and feasibility are separate
            assert "converged" in lc["convergence"]
            if lc["candidate"]:
                assert "all_links_ok" in lc["feasibility"]
                # convex optimum: no feasible candidate can be below it
                assert lc["objective"]["relative_excess_vs_direct"] > -1e-6
        assert "train" in direct and "heldout" in direct
        json.dumps(methods, allow_nan=False)


def test_cross_check_and_retry_with_second_solver():
    pytest.importorskip("qics")
    spec = synthetic_spec(np.random.default_rng(2))
    cc = pc.cross_check(spec, "frozen", pcmp, ["cvxopt", "qics"])
    ag = pc.agree(cc, "cvxopt", "qics")
    assert ag["direct_effort_fit"]["objective_rel_diff"] < 1e-4
    assert ag["phase1"]["first"] == ag["phase1"]["second"]


def test_periodic_columns_are_joint_major():
    cc = pytest.importorskip("examples.ur10.compare_physical_circular")
    q = np.random.default_rng(3).standard_normal((5, 6))
    C = cc.periodic_columns(q)
    assert C.shape == (30, 12)
    np.testing.assert_allclose(C[5:10, 1], np.sin(q[:, 1]))
    np.testing.assert_allclose(C[5:10, 7], np.cos(q[:, 1]))
    assert not C[:5, 1:6].any() and not C[5:10, 0].any()
