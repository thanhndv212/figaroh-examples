# Physical-estimator comparison, second pass (D4, #22)

Extends [`ur10-physical-comparison.md`](ur10-physical-comparison.md) (first pass, unchanged; its JSON
files are unchanged and re-checked, see "Regression"). Adds the four items that note left open:
log-Cholesky candidates, TX40 and TIAGo, joint-extra runs, and a second SDP solver.

## Summary and D4 verdict

- **Recommended for the D7 reference workflow: the direct LMI effort fit.** It is the convex optimum of the
  common objective, always physically feasible, and its held-out error equals or beats every other
  feasible method on UR10 and is within noise of base OLS on the recorded data.
- **Exact reconstruction** works only when the OLS base is physically reconstructable (UR10 noise-free,
  mostly low noise). It has a phase-I infeasibility certificate under noise (UR10 high noise) and on both
  recorded datasets (TX40, TIAGo), under both solvers: a property of the data, not of cvxopt.
- **Per-link projection** is feasible but has poor held-out error (about 5x worse on UR10 at low noise,
  1213 N on the TIAGo torso). Not recommended.
- **Log-Cholesky** matches the direct fit (objective within about 1e-3 relative, same held-out error) and its
  candidates are feasible, but it does not reliably reach scipy convergence within 2000 evaluations
  (core D5 no-go; the method change is tracked in figaroh-plus#155). Treat as a cross-check, not a default.
- **cvxopt vs QICS**: they agree to 1e-4 relative objective where both solve; cvxopt fails the direct fit on
  TX40 and TIAGo ("math domain error") where QICS solves it, so the reference workflow should use or fall
  back to QICS.
- **Data limitations**: TX40/TIAGo have no truth, held-out is a temporal block of one recording; TIAGo wrist
  efforts are about 87 % zero, effort constants are unverified and there is no torque reference.

Acceptance criteria of #22:

| Criterion | Status |
|---|---|
| Comparable explicit objectives | met: `settings.common_objective`, `objective` per method in every JSON |
| Frozen-extra and joint-extra separated | met: separate keys (`frozen_extra`, `joint_extra`) in all JSONs |
| Training and held-out separated | met: `train` / `heldout` fields per method |
| Per-joint units, parameter/base changes | met: `joint_units`, `units`, `base_change_vs_ols_rel`, per-link `change_vs_prior_norm` |
| Raw/config/model hashes, paired core+examples revisions | met: `provenance` of each JSON |
| Both Pinocchio profiles (3.7, 4.1) | met: `*-pin37.json` and `*-pin41.json` for every result set |
| TX40/TIAGo fair comparison or explicit limitation | met: results plus the limitations below |

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

## UR10 differentiated derivatives (weighting none, 3 paired seeds)

Same fixture, but q is differentiated by core's helper instead of the saved analytic dq/ddq (the first
pass's `differentiated` cells). Worst-joint mean held-out NRMSE (%), noise `low` and `high`
(`ur10-physical-extended-differentiated-pin{37,41}.json`; frozen rows re-checked against the first pass:
41 rows, max diff 2e-11 %). Counts are solved/converged/feasible out of 3.

| Mode | Noise | Method | s/c/f | worst-joint NRMSE | base err (N.m) | J excess |
|---|---|---|---|---|---|---|
| frozen | low | base_ols | 3/3/0 | 28.5 | 0.048 | -6e-3 |
| frozen | low | exact | 0/0/0 | none (infeasible) | | |
| frozen | low | direct | 3/3/3 | 26.2 | 0.047 | 0 |
| frozen | low | projection | 3/3/3 | 35.7 | 0.21 | 1.3 |
| frozen | low | log-Cholesky nominal | 3/0/3 | 26.1 | 0.046 | 2e-5 |
| frozen | high | base_ols | 3/3/0 | 323 | 0.68 | -0.57 |
| frozen | high | direct (cvxopt) | 0/0/0 | solver failure | | |
| frozen | high | direct (QICS) | 3/3/3 | 178 | 0.85 | 0 |
| frozen | high | projection | 3/3/3 | 159 | 1.5 | 1.1 |
| frozen | high | log-Cholesky nominal | 3/0/3 | 186 | 0.80 | 0.03 |
| joint | low | direct | 3/3/3 | 33.2 | 0.047 | 0 |
| joint | high | direct (QICS) | 3/3/3 | 183 | 0.84 | 0 |
| joint | high | log-Cholesky nominal | 3/0/3 | 191 | 0.79 | 0.03 |

Differentiation noise dominates: held-out error is 5 to 30 times the analytic cells and exact
reconstruction has no solution even at low noise (phase-I infeasible, both solvers). The ranking
direct ~ log-Cholesky < projection on base error is unchanged; log-Cholesky candidates are feasible but
none converged. At high noise cvxopt fails the direct fit and QICS solves it; at high noise projection has
the lowest held-out NRMSE in the frozen mode (159 vs 178) while having 1.7 times the base error, so low
held-out error alone does not favour it.

## UR10 periodic (circular) extras (weighting none, 3 paired seeds)

A truth with a position-periodic torque `a_j sin q_j + b_j cos q_j` per joint (amplitude 2 % of the
joint's effort scale), added to the fixture effort of both splits
(`compare_physical_circular.py`, `ur10-physical-circular-pin{37,41}.json`). Three variants on the same
data: periodic extras fixed at 0 (misspecified), estimated (joint-extra), fixed at truth (oracle).
`sin/cos` of the shoulder_lift joint are collinear with the inertial base and absorbed (listed).

| Noise | Method | periodic extras at 0 | estimated | at truth (oracle) |
|---|---|---|---|---|
| low | base_ols | 25.4 | 6.9 | 4.4 |
| low | direct | 22.5 | 7.2 | 4.6 |
| low | projection | 31.4 | 30.8 | 31.3 |
| low | log-Cholesky nominal (conv 1/3, 3/3, 3/3) | 22.4 | 7.2 | 4.6 |
| high | base_ols | 34.3 | 34.7 | 22.0 |
| high | direct | 24.9 | 28.4 | 18.4 |
| high | projection | 38.1 | 54.9 | 36.4 |
| high | log-Cholesky nominal (conv 0/3) | 24.9 | 28.4 | 18.2 |

Worst-joint mean held-out NRMSE (%, vs the true effort including the periodic term). Base error
(N.m) at low noise: 0.19 (at 0), 0.11 (estimated), 0.0074 (oracle); at high noise: 0.18, 0.14, 0.036.
Ignoring the periodic term costs about 3 times the held-out error at low noise and biases the base;
estimating it recovers most of it at low noise, but at high noise the 12 extra columns overfit
(held-out worse than leaving them at zero, base error still lower). All physical methods stay feasible
(direct, projection, log-Cholesky); exact reconstruction solves only 2 of 3 low-noise cases when the periodic term is estimated or fixed at
truth, none otherwise (omitted from the table; see the JSON).

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

## Reproduction

```bash
conda activate figaroh-dev
export ROS_PACKAGE_PATH=$(python scripts/fetch_models.py --print-ros-package-path | tail -1)
export OPENBLAS_NUM_THREADS=1
# QICS: pip install qics picos extras into a separate directory and add it to PYTHONPATH
python examples/compare_physical_real.py --robot staubli_tx40
python examples/compare_physical_real.py --robot tiago
python examples/ur10/compare_physical_extended.py --noise none low high --seeds 5
python examples/ur10/compare_physical_extended.py --derivatives differentiated --noise none low high --seeds 3 \
    --output docs/development/results/ur10-physical-extended-differentiated-<profile>.json
python examples/ur10/compare_physical_circular.py --noise none low high --seeds 3
```

Run once per profile (Pinocchio 3.7.0 and 4.1.0); outputs go to `docs/development/results/`
(`--overwrite` is required to replace). Hashes of the raw data, configs, URDFs, scripts, comparator
and log-Cholesky spike, plus the paired core (`e719de1`) and examples revisions, are in `provenance`
of each JSON. The examples revision recorded in the results is the commit that contained the scripts
(`49f6608` for TX40/TIAGo/UR10 extended; `3fb2c71` for the differentiated and circular runs; clean tree).
The result JSONs are git-ignored by default (`results/`) and committed explicitly with `git add -f`.
The committed UR10 files are trimmed: they keep `provenance`, `settings` and `summary` and drop the
per-case `cases` and `cross_check` blocks (marked by a `trimmed` key). Rerun the commands above for the
full output. The TX40 and TIAGo files have no `summary` block and are committed in full.
