# UR10 dynamic physical-estimator comparison (first pass)

Issue: [#22](https://github.com/thanhndv212/figaroh-examples/issues/22) (D4, parent figaroh-plus#39).
Inputs: the frozen truth fixture and protocol v1 ([#21](ur10-dynamic-truth-fixture.md)).
Comparator: core `figaroh.identification._physical_comparator`, branch
`feat/59-physical-comparator` at `a53f10e` (private, unmerged).

- Script: `examples/ur10/compare_physical_estimators.py` (about 25 min for the full run, one core).
- Results: `docs/development/results/ur10-physical-comparison-pin37.json` and `-pin41.json`
  (identical numbers, maximum difference 0 in every summary row). Earlier result files are untouched.
- Test: `tests/test_ur10_physical_comparison.py` (noise-free smoke, about 12 s; skipped without the comparator).

```bash
cd examples/ur10
PYTHONPATH=<core>/src python compare_physical_estimators.py --derivatives analytic differentiated
```

## Objectives (all on the same problem; no silent fallback)

| Method | Objective |
|---|---|
| `base_ols` | base-parameter least squares, no physical constraint |
| `exact_reconstruction` | closest-to-nominal theta with M theta = phi_ols and a pseudo-inertia LMI per link |
| `direct_effort_fit` | regularised effort fit, same LMIs, no equality |
| `per_link_projection` | nullspace representative of phi_ols, then an independent LMI projection per link |

Weighting: `none` (protocol's unweighted fit) and `scaled` (rows divided by the joint's noise-free
training effort RMS). Prior: nominal URDF inertias. Frozen-extra: fv, fs, Ia fixed at 0 (= truth) through
`FixedExtras`; joint-extra: base OLS only (the comparator cannot estimate extras; follow-up). In joint-extra
the Ia columns are collinear with base inertias and are absorbed, so only independent extras are estimated
(reported per case). Units: efforts N.m, masses kg, first moments kg.m, inertias kg.m^2, base errors N.m
(protocol scaling). Base errors are expressed in the protocol's base basis and equal `ols_case`.

## Held-out NRMSE (%, mean of 5 paired seeds), analytic derivatives, per joint (pan, lift, elbow, w1, w2, w3)

| Noise | Wt | Method | Solved | Held-out NRMSE per joint | Base err RMS |
|---|---|---|---|---|---|
| low | none | base_ols | 5/5 | 0.76 0.09 0.05 0.35 2.82 5.07 | 0.0074 |
| low | none | exact | 4/5 | 0.68 0.08 0.06 0.35 2.70 5.32 (4 solved) | 0.0069 |
| low | none | direct | 5/5 | 0.76 0.09 0.05 0.35 2.69 5.23 | 0.0073 |
| low | none | projection | 5/5 | 6.98 3.14 3.56 6.16 31.0 25.1 | 0.209 |
| low | scaled | base_ols = exact = direct | 5/5 | 0.26 0.08 0.04 0.08 0.09 0.06 | 0.0045 |
| low | scaled | projection | 5/5 | 6.98 3.03 3.44 5.89 31.2 23.6 | 0.202 |
| high | none | base_ols | 5/5 | 3.8 0.45 0.26 1.77 14.1 25.4 | 0.037 |
| high | none | exact | 0/5 | none solved, 5/5 phase-I infeasible | |
| high | none | direct | 5/5 | 3.63 0.41 0.25 1.44 9.98 23.8 | 0.035 |
| high | none | projection | 5/5 | 8.1 3.4 4.0 7.1 31.2 36.0 | 0.241 |
| high | scaled | base_ols | 5/5 | 1.32 0.39 0.19 0.38 0.46 0.32 | 0.0227 |
| high | scaled | exact | 1/5 | 1.41 0.51 0.18 0.15 0.43 0.26; 4/5 phase-I infeasible | 0.036 |
| high | scaled | direct | 5/5 | 1.23 0.29 0.16 0.37 0.46 0.32 | 0.0207 |
| high | scaled | projection | 5/5 | 7.7 2.9 3.5 5.9 31.2 23.7 | 0.207 |

Noise-free: base OLS, exact and direct recover the truth (NRMSE < 0.01 %); projection does not
(3 to 31 %, base RMS 0.20 N.m) because it is not an effort fit. Differentiated derivatives, all rows and
training/held-out per-joint RMSE in N.m, per-link mass/COM/inertia changes, pseudo-inertia minimum
eigenvalues and solver status are in the JSON (`summary`, `cases`). Base OLS matches the #21 baseline table.

## Findings

1. Where the OLS base is physically reconstructable, the LMI constraints are inactive: exact and direct
   equal base OLS to plotted precision; weighting (scaled rows) changes held-out error far more than any
   physical method (wrist_3 5.1 % to 0.06 % at low noise).
2. Per-link projection changes the base parameters (RMS 0.2 N.m) and is 5 to 30 times worse in held-out
   effort. It is a different objective, not a competitor for effort accuracy.
3. Direct effort fit always converged and was always feasible; at high noise it slightly improves wrist_2.
4. Convergence and feasibility are separate fields. Smallest pseudo-inertia eigenvalues sit at the solver
   tolerance (about -1e-9 to -1e-8 against `feas_tol` -1e-8): accepted, but not strictly interior.

## Exact reconstruction diagnosis (`diagnose_exact`, one seed per cell)

| Derivatives / noise | Phase-I s* (none / scaled) | Verdict |
|---|---|---|
| analytic none, low | +2e-3 / +2e-3 (low: +1.4e-3 / +2.0e-3) | formulation |
| analytic high | -4.1e-3 / -4.9e-2 | genuine infeasibility (certificate) |
| differentiated none | solver failure / +2.2e-3 | indeterminate / no failure |
| differentiated low | -9.1e-2 / solver failure | genuine infeasibility / indeterminate |
| differentiated high | -2.1 / -1.8 | genuine infeasibility |

- Noise-free and low noise (analytic): the constraint set is feasible (phase-I s* > 0), the comparator's
  scaled exact solve is accepted in under a second, but the production entry point
  `reconstruct_full_parameters(method="sdp")` errors or exceeds the budget (30 s or more; unbounded runs
  of many minutes seen). Verdict: formulation (unscaled variables), not infeasibility; which production
  difference matters was not isolated. The D2 variants (Schur norm, no scaling, parameters-only, no bounds)
  all succeed.
- Noisy OLS bases can be genuinely non-physical: phase-I certificates s* < 0 (optimal phase-I solve) for
  every high-noise analytic case and for differentiated low/high. Every failed exact solve carries its own
  per-case phase-I label in the JSON; "solver_failure" is never called infeasible.
- Only solver available: cvxopt (picos 2.6.1). A second solver (for example Clarabel/SCS/MOSEK) would
  separate solver-numerical from formulation causes; decision for the maintainer.

## Reproduction

| Item | Value |
|---|---|
| Core | `feat/59-physical-comparator` `a53f10e`, comparator sha256 in the JSON |
| Examples | script commit `62e684f` (JSON `provenance.examples`) |
| Profiles | pin37: Pinocchio 3.7.0 (`figaroh-dev`); pin41: Pinocchio 4.1.0 (read-only use of an existing clone env), picos 2.6.1, cvxopt 1.3.2, NumPy 2.3.4 |
| Raw | `train.csv` `014cdb14...`, `validation.csv` `3dec2056...` |
| Config | `protocol.yaml` `83f1620c...`, `ur10_unified_config.yaml` `2dc2b5c3...` |
| Model | nominal URDF `1da0c0de...`, truth URDF `e3ce01ca...`, `truth_parameters.csv` `bc39953a...` |

Full hashes are in `provenance.hashes` of each JSON.

## Not done (follow-ups)

- Nonlinear log-Cholesky candidates: the existing runner lives on `feature/log-cholesky-dataset-validation`
  / a stash and needs a private core spike from another branch; not cheap.
- TX40 and TIAGo runs.
- Joint-extra for the physical methods (needs free extras in the comparator).
- Isolating the production `reconstruct_full_parameters` failure; a second SDP solver.
- Circular (periodic) extras are not in this fixture (extras are zero in the truth).
