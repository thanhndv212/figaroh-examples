# Log-Cholesky dataset validation (private feasibility spike)

This runner validates the core issue #22 experiment on existing robot data.
It does **not** install a nonlinear solver or demonstrate high-level pipeline
integration. Core [PR #31](https://github.com/thanhndv212/figaroh-plus/pull/31)
records **revise**, and core #30 tracks convergence/scaling. Production examples
issue #11 remains open and conditional on the accepted API/go decision.

## Reproduce

Use core branch `feature/22-log-cholesky-feasibility` (commit `b134888`) beside
this examples checkout; it includes the corrected pseudo-inertia conventions
and Pinocchio 3.7/4.1 compatibility. In `figaroh-dev`, from examples repo root:

```bash
PYTHONPATH="$PWD:$PWD/../figaroh/src" OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  python benchmarks/log_cholesky_datasets.py --robot ur10 \
  --spike-path ../figaroh/docs/development/spikes/log_cholesky_feasibility.py \
  --output /tmp/ur10-comparison.json
```

Repeat with `--robot staubli_tx40` and `--robot tiago`. Outputs must be explicit;
source datasets are never written. The runner stages CSV partitions in a
TemporaryDirectory and changes into the matching robot directory. It imports
an explicitly supplied **private** core script, not a supported package API.
Run with both Pinocchio environments to reproduce the committed profile
results. Results include core/examples revisions and both script SHA256s;
examples base is `3a2c8e9`. All records use core `b134888` and the same core spike source hash.
Loader hashes identify this examples change, including the TX40 alignment fix.
CI pins that core commit and runs tests and offline comparisons on both
Pinocchio profiles with the core native dependency constraints.

## Data and frozen protocol

| Robot | Provenance | Raw training rows | Raw validation rows |
| --- | --- | --- | --- |
| UR10 | Simulation q and torque | q 0:500, torque 0:498 | separate files: q 0:400, torque 0:398 |
| TX40 | Existing motor encoder/current recording (real) | 0:24750 | 29250:45000 |
| TIAGo | Existing position/velocity/effort recording (real) | 921:4149 | 4736:6791 |

Ranges are half-open. UR10 torque files have two fewer rows than positions;
the existing loader's differentiation trims positions to the matching length.
UR10 uses different simulation files for validation; TX40 and TIAGo use
**disjoint temporal blocks of one recording**, with a gap. They are not new,
independent real experiments. Filtering/differentiation runs separately on
each partition, so training does not use validation to preprocess or fit.
Do not interpret this limited temporal validation as generalization across
loads, days, temperatures or physical units.

Keep existing robot loaders, torque/current conversions and resolved unified
configs. TIAGo uses the same active-joint reduction/motor constants as its
entry point. TX40 additionally includes the three configured coupled-wrist
columns explicitly; the existing entry point does not call that hook. This
comparison is ordinary OLS (no WLS), with all four methods sharing its estimated
extra parameters. Extras are fitted **on training only** and then frozen for
SDP and nonlinear fits, without bounds. This isolates inertial fitting but
is not a claim that these nuisance parameters are physically valid or optimal.
No CAD constraints are applied.

After independent preprocessing, select up to 240 equally spaced samples
per partition, excluding 20 additional samples at each edge. All methods use
the same selected rows, unweighted residuals and joint-major order. Native
regressor columns and `Y @ nominal` are checked against direct Pinocchio
regressors/RNEA for every selected training and validation state (tolerances
1e-10/1e-12 and maximum RNEA error <1e-8). This proves regressor/order consistency;
it does **not** prove numerically differentiated accelerations match physical
truth. UR10's loader has a hard-coded 100 Hz time/filter path while its unified
config says 500 Hz. Current preprocessing is preserved and reported; the
analytic synthetic core experiment isolates the parameterization from it.

Only positive-mass blocks affecting training torque are optimized; other
blocks remain at nominal. Nominal/OLS interior repairs are explicit and their
p10 change norms are recorded. The core spike's fixed scales, 1e-6 prior,
analytic Jacobian, ±12 coordinate bounds, 200 evaluations and 20-second guard
are reused. Neither prior nor extras uses validation. Log-Cholesky starts are
repaired nominal and repaired OLS. No quality-gate thresholds are tuned here.

## Observed results (2026-10-02)

Held-out aggregate errors, Pinocchio 3.7:

| Robot | Nominal | OLS | OLS + SDP | Log-Cholesky nominal / repaired OLS |
| --- | --- | --- | --- | --- |
| UR10 simulation | 0.000348 | 0.000575 | 0.004623 | 0.000513 / 0.000526 |
| TX40 real data (aligned) | 10.364936 | 5.298469 | 4.470204 | 4.986377 / 5.000695 |
| TIAGo real data | 3.865181 | 1.518097 | **failed**, candidate 3.960360 | 1.410331 / 1.410392 |

UR10/TX40 errors are in Nm. TIAGo aggregates mix torso force (N) with arm
torques (Nm), as in the existing workflow; that aggregate has no single
physical unit and should not be used to compare robots. Per-joint errors and
joint units are recorded for interpretation. The nominal baseline has zero
extra parameters; other methods share the training-only OLS extras.

**All six nonlinear fits exhaust 200 evaluations**, on both 3.7 and 4.1.
Their physical candidates pass the independent per-link verdict, but they are
unsuccessful solver results, not selected production outputs. UR10 nominal is
better than every fitted candidate. TIAGo candidate improvements are diagnostic
signals, not evidence of dependable convergence or hardware deployment.
Training regressor ranks/columns are UR10 36/60, TX40 61/87 and TIAGo 73/216.
Full inertial recovery is not identifiable from these datasets.

TIAGo SDP fails with `math domain error` for arm_2 on 3.7. The partial candidate
retains that invalid OLS inertia (min eigenvalue approximately -13.90), so its
prediction is **not an accepted physically consistent baseline**. On 4.1 the
same baseline also fails, with candidate held-out error 2.578441 and failures on arm_2 and arm_3; failed SDP
outputs are sensitive to solver numerical perturbations and must not be
compared as successful models. The raw reports preserve per-link error/status
and candidate physical verdicts. No fallback is relabeled as success.

OLS and successful SDP baselines agree between versions to numerical tolerance.
UR10 and TIAGo nonlinear metrics also agree closely. TX40 nonlinear held-out
errors differ by approximately 0.00107 and 0.00924 Nm between versions; both
runs fail termination. Rank deficiency and local optimization make full
parameter vectors sensitive, so do not require elementwise equality or infer
unique inertials. The aligned TX40 nonlinear candidates are worse than the
successful SDP baseline on validation, strengthening the revise decision. Full machine-readable outputs live in
[results/](results/) for both profiles, including per-joint/train/validation
errors, parameters, solver objective/status/evaluations/runtime, physical
verdicts, source hashes, model/config hashes, selected sample indices and
repair magnitudes. The synthetic core results remain a separate protocol.

## TX40 alignment correction

The data audit found that torque processing removed 20 samples at each end
(`nbutter=4`), while `_sync_torque_w_kinematics` merely shortened kinematics
at the tail. Torque at source index 20 was paired with kinematics at index 0.
The correction applies the identical border window to timestamps, positions,
velocities and accelerations before synchronization. A source-index regression
checks every channel and input preservation on both Pinocchio profiles.
The TX40 table and committed results were rerun after this correction; earlier
misaligned measurements were discarded as an invalid comparison baseline.
Necessary unused-import/Black cleanup in the touched loader satisfies hooks.

TIAGo timestamp median spacing is approximately 0.009997 s (100 Hz), whereas
its config declares `ts=0.0002` (5000 Hz) and the explicit filter uses 500 Hz.
Derivatives use the recorded timestamps, but filter calibration is inconsistent.
This is reported, not silently adjusted. It prevents claiming these real-data
results meet a production signal-validation gate.

## Validation limits

Python 3.12.11; Pinocchio 3.7.0/4.1.0; NumPy 2.3.2 and SciPy 1.16.1;
PICOS 2.6.1/CVXOPT 1.3.2. This is offline dataset validation, not execution
on a robot. There is no fitted-URDF export or production result-stage/report
integration. Future production #11 must verify acceleration provenance and
independent trajectories and pass the eventual numerical/robustness gates.

Mandatory repository validation and hosted checks are recorded in the PR;
passing this private runner's assertions does not make current model-quality
verification gates pass.
