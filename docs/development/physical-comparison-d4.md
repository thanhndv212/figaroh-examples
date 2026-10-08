# Physical-estimator comparison, second pass (D4, #22)

Extends [`ur10-physical-comparison.md`](ur10-physical-comparison.md) (first pass, unchanged; its JSON
files are unchanged and re-checked, see "Regression"). Adds the four items that note left open:
log-Cholesky candidates, TX40 and TIAGo, joint-extra runs, and a second SDP solver.

## What is compared

Same problem per case, every method scored with the same explicit objective
`J = ||tau - Y theta - E e||^2 (weighted) + lam * ||theta - theta_prior||^2` (`lam = PhysicalPolicy.prior_weight`),
reported as `relative_excess_vs_direct` against the convex direct fit (the optimum of `J` over the
physical set). Nothing is hidden behind a fallback: a failed solve stays in the JSON with its status.

| Method | Meaning |
|---|---|
| `base_ols` | least squares in the identifiable base, nullspace representative (not constrained; `feasible=False` means some link is not physically consistent) |
| `exact_reconstruction` | physical parameters that reproduce the OLS base exactly (phase-I certificate when infeasible) |
| `direct_effort_fit` | convex fit of the effort under the physical (LMI) constraints |
| `per_link_projection` | project the OLS representative link by link |
| `log_cholesky[start]` | nonlinear log-Cholesky candidates (core spike, 2000 evaluations max); two starts: `nominal_repaired`, `ols_repaired` |

- **Frozen-extra** keeps friction/inertia/offset extras at the first-pass (or nominal) value.
  **Joint-extra** eliminates the independent extras exactly (variable projection, no core change);
  extras collinear with the base (for example `Ia`) are absorbed into the base and listed.
- Training and held-out are separate fields (held-out target: measured effort, plus truth parameters
  for UR10 only). Units: N.m on revolute joints, N on the TIAGo prismatic torso; `joint_units` and
  `units` are in every JSON.
- Log-Cholesky candidates are finite and checked for physical feasibility **separately** from whether
  the optimizer converged (scipy status 1/2/4). "Not converged" (evaluation budget) is reported, not
  hidden, and is not a closure requirement. Spike scripts in core are not modified (core D5 no-go).
- Phase-I certificate: optimal `s* < -1e-7` is "infeasible". A solver failure is never called infeasible.
- Second solver: QICS (via picos) re-solves any case where cvxopt did not return optimal (entry
  `<method>@qics`) and cross-checks the objective/parameters on the first seed of each UR10 cell.

## UR10 (synthetic, known truth), weighting none, mean of 5 paired seeds, analytic derivatives

Held-out NRMSE (%), worst joint of the 5-seed mean; "solved/conv/feas" are counts out of 5.
Noise-free cell (1 case): every physical method except projection is at the 1e-3 % level, base error
below 1e-5 N.m.

| Mode | Noise | Method | solved | conv | feas | worst-joint NRMSE | base err (N.m) | J excess |
|---|---|---|---|---|---|---|---|---|
| frozen | low | base_ols | 5 | 5 | 0 | 5.07 | 7.4e-3 | -3e-6 |
| frozen | low | exact | 4 | 4 | 4 | 5.32 | 6.9e-3 | 6e-8 |
| frozen | low | direct | 5 | 5 | 5 | 5.23 | 7.4e-3 | 0 |
| frozen | low | projection | 5 | 5 | 5 | 31 | 0.21 | 7.3 |
| frozen | low | log-Cholesky (both starts) | 5 | 5 | 5 | 5.23 | 7.4e-3 | 4e-8 |
| frozen | high | base_ols | 5 | 5 | 0 | 25.4 | 0.037 | -1e-4 |
| frozen | high | exact | 0 | 0 | 0 | none (phase-I infeasible) | | |
| frozen | high | direct | 5 | 5 | 5 | 23.7 | 0.035 | 0 |
| frozen | high | projection | 5 | 5 | 5 | 36 | 0.24 | 0.38 |
| frozen | high | log-Cholesky nominal | 5 | 0 | 5 | 23.5 | 0.035 | 2e-6 |
| frozen | high | log-Cholesky ols | 3 | 1 | 3 | 26.3 | 0.032 | 9e-7 |
| joint | low | direct | 5 | 5 | 5 | 5.47 | 7.3e-3 | 0 |
| joint | low | projection | 5 | 5 | 5 | 25.7 | 0.21 | 6.6 |
| joint | high | direct | 5 | 5 | 5 | 26.2 | 0.035 | 0 |
| joint | high | projection | 5 | 5 | 5 | 39 | 0.24 | 0.32 |
| joint | high | log-Cholesky nominal | 5 | 1 | 5 | 26.0 | 0.035 | 2e-6 |

Complete tables (all noise levels, both weightings, per-joint arrays, link mass ratios, extras) are in
`results/ur10-physical-extended-pin{37,41}.json` (`summary`).

- Where the OLS base is reconstructable, the LMI constraints are inactive: exact and direct equal OLS.
  Noise makes the OLS base non-physical on every `base_ols` row (feas 0/5), and exact reconstruction then
  has no solution (certificate), while direct fit and log-Cholesky still return feasible parameters.
- Direct fit and log-Cholesky reach the same held-out error; log-Cholesky stays within 1e-5 relative
  objective excess of the convex optimum. It converges at low noise and exhausts the 2000-evaluation
  budget at high noise (feasible, not converged). The `ols_repaired` start occasionally gives no
  candidate at high noise (3 of 5 cases), reported as such.
- Per-link projection is the clear loser on held-out error (5 to 6 times worse at low noise).
- Joint-extra changes held-out error by a fraction of a percent versus frozen-extra on UR10 (the
  fixture's true extras are zero), so it is mainly a check that the separation is consistent.

## TX40 and TIAGo (recorded data)

Held-out is a disjoint temporal block of the same recording (not an independent experiment); target is
measured effort (TX40 motor-current effort, TIAGo joint effort). There is no truth. Frozen-extra
values come from the nominal prior; "nominal" below is the unmodified URDF prior.

TX40 (6 joints, rows 0:24750 train, 29250:45000 held-out), held-out RMSE (N.m), weighting none, frozen-extra:

| Method | feasible | converged | joint_1 | joint_2 | joint_3 | joint_4 | joint_5 | joint_6 |
|---|---|---|---|---|---|---|---|---|
| nominal URDF | | | 8.10 | 10.32 | 4.13 | 1.70 | 2.68 | 1.72 |
| base_ols | no (6/6 links) | yes | 6.64 | 8.32 | 4.28 | 1.80 | 2.65 | 1.72 |
| exact_reconstruction | infeasible (phase-I, both solvers) | | | | | | | |
| direct_effort_fit (QICS) | yes | yes | 6.40 | 8.21 | 4.13 | 1.72 | 2.71 | 1.73 |
| per_link_projection | yes | yes | 5.61 | 7.42 | 4.07 | 1.75 | 2.64 | 1.72 |
| log-Cholesky nominal | yes | no | 6.40 | 8.21 | 4.12 | 1.72 | 2.72 | 1.73 |
| log-Cholesky ols | yes | no | 6.40 | 8.21 | 4.13 | 1.72 | 2.71 | 1.73 |

cvxopt fails on the direct fit ("math domain error"); the primary failure is kept and QICS is the
reported solve. Joint-extra gives the same picture (direct 6.36/8.28/4.07/1.78/2.60/1.74; log-Cholesky
within 1e-3 relative objective of it). Projection is lower on the two largest-error joints here although its
training objective is 0.2 to 0.3 above the direct fit; this is a temporal-block, motor-current result
and is not evidence that projection is a better estimator. With scaled weighting
the ranking is unchanged.

TIAGo (12 optimized joint links; the table lists the 8 actuated torso/arm joints with effort; rows 921:4149 train,
4736:6791 held-out), held-out RMSE, weighting none, frozen-extra:

| Method | feasible | converged | torso (N) | arm_1 | arm_2 | arm_3 | arm_4 | arm_5 | arm_6 | arm_7 |
|---|---|---|---|---|---|---|---|---|---|---|
| nominal URDF | | | 1.09 | 1.26 | 4.01 | 1.36 | 1.48 | 0.78 | 0.50 | 0.18 |
| base_ols | no (9 links) | yes | 0.29 | 1.34 | 2.49 | 1.35 | 1.52 | 0.17 | 0.19 | 0.07 |
| exact_reconstruction | infeasible (phase-I, both solvers) | | | | | | | | | |
| direct_effort_fit | yes | yes | 0.29 | 1.32 | 2.51 | 1.33 | 1.51 | 0.19 | 0.22 | 0.06 |
| per_link_projection | yes | yes | 1213 | 1.28 | 10.7 | 3.36 | 2.06 | 0.16 | 0.19 | 0.07 |
| log-Cholesky nominal | yes | no | 0.29 | 1.31 | 2.51 | 1.33 | 1.51 | 0.19 | 0.22 | 0.06 |
| log-Cholesky ols | no candidate | | | | | | | | | |

The `ols_repaired` start lies outside the coordinate bounds of the spike (+-12); it is reported as no
candidate, not clipped. Projection of the TIAGo OLS representative is badly off on the torso
(1213 N) because the unconstrained representative is far from the physical set.

## Cross-solver (cvxopt vs QICS)

Per case the JSON records `status`, objective relative difference and scaled parameter difference.
Direct fit and projection agree to 1e-4 relative objective or better wherever both solve. On TX40/TIAGo
the cvxopt failures (direct fit; exact reconstruction) are solver issues for the direct fit but a
genuine infeasibility certificate for exact reconstruction (phase-I infeasible under both solvers, so the
earlier "exact reconstruction fails in production" finding is a property of the data, not of cvxopt).

## Profiles

Both Pinocchio 3.7.0 and 4.1.0 were run with the same commit, comparator hash, picos 2.6.1,
cvxopt 1.3.2, QICS 1.1.3. UR10 results are identical between profiles to printed precision. On the
recorded data, held-out RMSE differs by at most 3e-4 relative between profiles (weakly determined
parameters differ more, up to O(1) relative in `change_vs_prior_norm` for individual links).

## Regression

The frozen-extra rows reproduce `ur10-physical-comparison-pin<37|41>.json` (78 rows compared, max
|diff| of NRMSE 5.4e-11 %); recorded in `first_pass_regression` of the extended JSON.

## Limitations

- TIAGo: wrist and head efforts are about 87 % zero (weak observability); effort constants are not
  verified and there is no torque-sensor reference, so "measured effort" is a motor-side proxy.
- Held-out for TX40 and TIAGo is a disjoint temporal block of one recording, not a new experiment;
  generalization claims beyond that block are not supported.
- TX40/TIAGo have no truth parameters; only effort-prediction error and physical feasibility can be
  compared. Lower held-out RMSE of a non-physical OLS is not evidence of a better model.
- Exact reconstruction is infeasible on both recorded datasets; there is nothing to compare there.
- The cvxopt direct-fit failure on the real data is not root-caused; QICS is the reported solve.
- Log-Cholesky never reaches the scipy convergence status on the real data in 2000 evaluations. Its
  candidates are feasible and within 1e-3 of the convex objective, which is what the issue asked to
  report; no convergence claim is made.
- Differentiated-derivative cells and circular extras are not run in this pass (analytic only).

## Reproduction

```bash
conda activate figaroh-dev
export ROS_PACKAGE_PATH=$(python scripts/fetch_models.py --print-ros-package-path | tail -1)
export OPENBLAS_NUM_THREADS=1
# QICS: pip install qics picos extras into a separate directory and add it to PYTHONPATH
python examples/compare_physical_real.py --robot staubli_tx40
python examples/compare_physical_real.py --robot tiago
python examples/ur10/compare_physical_extended.py --noise none low high --seeds 5
```

Run once per profile (Pinocchio 3.7.0 and 4.1.0); outputs go to `docs/development/results/`
(`--overwrite` is required to replace). Hashes of the raw data, configs, URDFs, scripts, comparator
and log-Cholesky spike, plus the paired core (`e719de1`) and examples revisions, are in `provenance`
of each JSON. The examples revision recorded in the results is the commit that contained the scripts
(`49f6608`, clean tree).
