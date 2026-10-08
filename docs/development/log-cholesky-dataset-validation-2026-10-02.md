# Log-Cholesky dataset validation — 2026-10-02 (superseded record)

Issue: [figaroh-examples #12](https://github.com/thanhndv212/figaroh-examples/issues/12).
Core: [#22](https://github.com/thanhndv212/figaroh-plus/issues/22) /
[PR #31](https://github.com/thanhndv212/figaroh-plus/pull/31), decision **revise**;
follow-up [core #30](https://github.com/thanhndv212/figaroh-plus/issues/30).
This is an offline dataset comparison, not a production method, a hardware test
or a fitted-model export.

> **Status: superseded, kept as a historical record.** The runner and results are
> not on `main`, because three of their inputs have since changed:
>
> - UR10's legacy simulated CSVs were retired in favour of the truth fixture
>   (`data/truth/`, examples #90), so the UR10 path no longer runs.
> - Core #32 (final-joint acceleration left at zero) is fixed, so the UR10, TX40
>   and TIAGo accelerations behind these numbers are no longer reproduced.
> - Core #30 is revising the nonlinear solver budget and convergence criteria.
>
> A port of the runner to current data is deferred to the core #30 confirmation
> run or examples #11.

## Reproduction

The exact runner, results and audit are at tag
[`evidence/12-log-cholesky-2026-10-02`](https://github.com/thanhndv212/figaroh-examples/tree/evidence/12-log-cholesky-2026-10-02)
(`fe2cbec`): `benchmarks/log_cholesky_datasets.py`, `benchmarks/results/` (UR10,
TX40, TIAGo × Pinocchio 3.7/4.1), `benchmarks/README.md` and
`benchmarks/validation-2026-10-02.md`. Use the core spike
`docs/development/spikes/log_cholesky_feasibility.py` from PR #31 (unchanged
since `b134888`, so the recorded `spike_sha256` still matches). In `figaroh-dev`,
from that tag's checkout:

```bash
PYTHONPATH="$PWD:$PWD/../figaroh/src" OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  python benchmarks/log_cholesky_datasets.py --robot ur10 \
  --spike-path ../figaroh/docs/development/spikes/log_cholesky_feasibility.py \
  --output /tmp/ur10-comparison.json
```

Repeat with `--robot staubli_tx40` and `--robot tiago`. Environment: Python 3.12.11,
NumPy 2.3.2, SciPy 1.16.1, PICOS 2.6.1, CVXOPT 1.3.2.

## Protocol (as run)

| Robot | Provenance | Training rows | Validation rows |
| --- | --- | --- | --- |
| UR10 | Simulated q and torque (legacy CSVs) | q 0:500, torque 0:498 | separate files: q 0:400, torque 0:398 |
| TX40 | Real motor encoder/current recording | 0:24750 | 29250:45000 |
| TIAGo | Real position/velocity/effort recording | 921:4149 | 4736:6791 |

- TX40 and TIAGo validation is a disjoint temporal block of the same recording,
  not an independent experiment. Each partition is filtered and differentiated
  separately.
- Up to 240 equally spaced samples per partition, excluding 20 at each edge.
  Regressor columns and `Y @ nominal` were checked against Pinocchio regressors
  and RNEA on every selected state. This proves regressor and ordering
  consistency, not that the accelerations are physically correct.
- Ordinary least squares with the extra parameters (friction, offsets, wrist
  coupling for TX40) fitted on training only, then frozen for the SDP and
  nonlinear fits. No CAD constraints.
- Nonlinear fits reuse the core spike settings: fixed scales, prior 1e-6,
  analytic Jacobian, ±12 coordinate bounds, **200 evaluations**, 20-second guard.
  Starts: repaired nominal and repaired OLS.

## Results (Pinocchio 3.7)

Held-out aggregate torque error:

| Robot | Nominal | OLS | OLS + SDP | Log-Cholesky nominal / repaired OLS |
| --- | --- | --- | --- | --- |
| UR10 simulation | 0.000348 | 0.000575 | 0.004623 | 0.000513 / 0.000526 |
| TX40 real data | 10.364936 | 5.298469 | 4.470204 | 4.986377 / 5.000695 |
| TIAGo real data | 3.865181 | 1.518097 | failed (candidate 3.960360) | 1.410331 / 1.410392 |

UR10 and TX40 are in Nm. The TIAGo aggregate mixes torso force (N) with arm
torques (Nm) and has no single unit.

- **All six nonlinear fits exhausted the 200-evaluation budget** on both
  Pinocchio 3.7 and 4.1. Their candidates passed the per-link physical check, but
  they are unsuccessful solver results. This matches the core spike and supports
  the revise decision.
- On UR10, the nominal model beats every fitted candidate. On TX40, the nonlinear
  candidates are worse than the SDP baseline on validation.
- TIAGo SDP failed (`math domain error` on arm_2 with 3.7; arm_2 and arm_3 with
  4.1). Its partial candidate keeps an invalid inertia (min eigenvalue about
  -13.90), so it is not a physically consistent baseline.
- Training regressor rank / columns: UR10 36/60, TX40 61/87, TIAGo 73/216. Full
  inertial parameters are not identifiable from these datasets.
- OLS and successful SDP results agree between Pinocchio versions to numerical
  tolerance. TX40 nonlinear held-out errors differ by up to 0.009 Nm between
  versions; both runs fail termination.

## Findings that led to later fixes

- **TX40 sample alignment.** Torque processing trimmed 20 samples at each end
  while kinematics were only shortened at the tail, pairing torque index 20 with
  kinematics index 0. The fix (same border window on all channels, with a
  source-index regression test) is on `main` as `d0b9006` and
  `tests/test_tx40_torque_alignment.py`. The results above use the corrected
  alignment.
- **Final-joint acceleration.** Core
  `calculate_first_second_order_differentiation` left the last joint's
  acceleration at zero; UR10's last joint moves. Fixed by core #32. The UR10
  numbers above are affected.
- **Missing meshes in CI.** UR10 and TIAGo needed meshes from a local
  `ROS_PACKAGE_PATH`. `main` now ships them under `models/`.
- **Sampling rates.** UR10's loader uses 100 Hz while its config says 500 Hz.
  TIAGo's recorded spacing is about 0.01 s (100 Hz), while its config declares
  `ts=0.0002` and the filter uses 500 Hz. Reported, not corrected, here.

## Repository validation at the time

`validate.py` at examples `3a2c8e9` with core `b134888`: 11 checks passed,
4 failed and 1 timed out. The failures came before this change:

- UR10, TIAGo and TX40 identification verification missed their quality gates
  (condition number or improvement). TX40 also failed on the unmodified loader.
- The UR10 optimal trajectory timed out after 600 s.
- The TIAGo optimal trajectory exited nonzero.

The TALOS multichain held-out failure on Pinocchio 3.7 was tracked and closed in
examples #14.
