# UR10 dynamic-identification data contract

The UR10 has no hardware identification recordings in this repository. Its
identification example runs on the simulated truth fixture in
[`truth/`](truth/) (#21), whose inertias, clock and effort are known; see the
[fixture report](../../../docs/development/ur10-dynamic-truth-fixture.md).

The earlier `identification_*_simulation.csv` files (and `validation/`) were
removed in #90: they had no saved generator, inertias or clock. Their audit,
fingerprints and the loader corrections made for them are kept in the
[dated signal audit](../../../docs/development/ur10-signal-audit-2026-10-02.md).

## Files

| File | Rows | Duration | Use |
|---|---:|---:|---|
| `truth/train.csv` | 2400 | 24 s | training (default `load_trajectory_data`) |
| `truth/validation.csv` | 800 | 8 s | held-out validation (`validation_data_file`) |

`truth/` also holds the truth URDF and parameters, the frozen optimal
training waypoints, the manifest (seeds, collision clearances, independent
checks, hashes) and the benchmark protocol. Do not edit them: regenerate with
`identification_truth.py` and update the manifest hashes together.

## Channels and units

Columns are selected by name; reordering does not change the joint mapping.
All values must be finite.

| Position | Effort | URDF joint | Units |
|---|---|---|---|
| q0 | tau1 | shoulder_pan_joint | rad; joint-side Nm |
| q1 | tau2 | shoulder_lift_joint | rad; joint-side Nm |
| q2 | tau3 | elbow_joint | rad; joint-side Nm |
| q3 | tau4 | wrist_1_joint | rad; joint-side Nm |
| q4 | tau5 | wrist_2_joint | rad; joint-side Nm |
| q5 | tau6 | wrist_3_joint | rad; joint-side Nm |

`t` is the exact simulation clock in seconds. The effort is Pinocchio RNEA
on the truth model, checked independently (fixture report, section 2). No
current-to-torque conversion, gear ratio or sign change applies. The files
also hold the analytic `dq0..dq5` and `ddq0..ddq5`; the example ignores
them, and the truth benchmark (`identification_truth.py`) uses them.

## Timing, differentiation and filtering

The unified config sets **100 Hz** (`ts = 0.01 s`), the fixture's clock. The
loader requires `t` to be uniform at that `ts` and the filter sample rate to
equal `1/ts`; it performs no resampling.

Velocities are forward differences of positions `i` and `i+1`, centred at
`(i+0.5)*ts`; accelerations are differentiated on these interval centres
(core `calculate_first_second_order_differentiation`). The loader keeps the
first `n-2` rows of positions, efforts and clock; the half-step offset of
the velocities is explicit, not interpolated.

The base pipeline then filters positions, velocities and accelerations
separately: median window 5, then zero-phase Butterworth order 4, cutoff
10 Hz, sample rate 100 Hz. The motion's content is below about 3 Hz.
Effort is not filtered. Filter edges remain in the fit.

## Samples and validation

| Split | Source rows | Returned rows |
|---|---:|---:|
| training | 2400 | 2398 |
| validation | 800 | 798 |

There is no sample cap: every row is used. The default CLI uses
`solve(decimate=False)`. Validation is a separate trajectory of a different
family (a Fourier series, not the optimised training splines), so validation
metrics are held out. `trajectory_provenance`, keyed by absolute file path,
records the source rows, the retained index range, the clock and units.

## Fingerprints

SHA-256 of the data files is in `truth/manifest.json` (`files`), checked by
`tests/test_ur10_identification_truth.py`.
