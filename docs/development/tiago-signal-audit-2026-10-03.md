# TIAGo dynamic signal audit — 2026-10-03

Issue: [figaroh-examples #20](https://github.com/thanhndv212/figaroh-examples/issues/20).
Delivery package: [D2 / core #37](https://github.com/thanhndv212/figaroh-plus/issues/37).
This is an offline adapter audit of the recorded TIAGo arm dataset, not a
physical-model comparison or a hardware test. Companion: the
[UR10 signal audit](ur10-signal-audit-2026-10-02.md).

## Methodology and reproducibility

- Core: figaroh-plus `02f705a515e02c7d0e4f6920e82760d0e7fcb648` (`v0.5.0`).
  Examples baseline: `b8d9c75b58a8c9d74f0a117bf0fbf9225e706093` (`main`); the
  changed revision is the commit containing this report.
- Environment: `figaroh-dev`, Python 3.12.11, Pinocchio 3.7.0, NumPy 2.3.4,
  SciPy 1.16.1, pandas 2.3.2, macOS arm64; single-threaded BLAS for every
  identification run, so results repeat exactly.
- Data: `examples/tiago/data/identification/dynamic/tiago_{position,velocity,effort}.csv`
  (hashes in the [data README](../../examples/tiago/data/README.md)). Raw files
  are not modified; held-out splits are temporary copies.
- Identification uses `identification.py`'s own constants: truncation
  `(921, 6791)`, `decimate=True` (factor 10), the per-joint `reduction_ratio`
  and `kmotor` tables, OLS.
- The diagnosis instrumented the real pipeline (`TiagoIdentification`
  → `BaseIdentification.process_data`), recording which filter and
  differentiation functions run and with which arguments, rather than
  inferring the path from code.

```bash
cd examples/tiago
export ROS_PACKAGE_PATH="$(python ../../scripts/fetch_models.py --print-ros-package-path)"
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 python identification.py
python -m pytest ../../tests/test_tiago_signal_processing.py -q
```

## Diagnosis before correction

**Clock.** The three CSVs share one recorded column `t`: 8022 rows,
0–80.21 s, median step 9.997 ms (~100 Hz), jitter 7.7–12.3 ms (p1/p99
8.7/11.4 ms), strictly increasing, no gaps, identical across files. The
unified config claimed `sampling_frequency: 5000` (`ts = 0.0002`) and
`filter_params.f_sample: 500`; neither matches the recording.

**Filter path.** Core filters each supplied kinematic channel with
`BaseIdentification._apply_filters`, using `filter_params` only: median window
5, then zero-phase Butterworth order 4 designed as `f_butter / (f_sample / 2)`.
It was called twice (positions, measured velocities) with
`f_butter = 2, f_sample = 500`. The `cutoff_frequency: 100` /
`sampling_frequency: 5000` entries were not used by the filter (100 Hz is
also above the 50 Hz Nyquist limit of the data). Designed at 500 Hz but applied
to 100 Hz samples, the normalised cutoff 0.008 corresponds to **0.40 Hz
(single pass), 0.36 Hz after `filtfilt`** — about 5.5× lower than the
configured 2 Hz. No `identification_tools` helper that uses `ts` is reached on
this path; `ts` only labels the clock.

**Differentiation.** Accelerations are not recorded; core differentiates the
*filtered measured velocity* with `np.gradient` on the recorded timestamps.

**Velocity channel.** Units and sign match the position derivative
(correlation 0.988–0.991, least-squares scale 1.002–1.004 on every joint), but
the channel is **delayed by about 18 samples (0.18 s)**: on `arm_3`, the
relative mismatch between d(q)/dt[n] and v[n+L] falls from 13.7% at L = 0 to
6.2% at L = 18 (8.5% at 10, 6.2% at 20). The lag estimate is 18 samples on the
full window and, independently, on each half of a split. Positions, velocities
and efforts carry the same timestamps, so the delay is inside the velocity
signal (e.g. a filtered or late-sampled channel), not a logging offset. Before
correction, every regressor row combined q(t) with v and a from ~0.18 s earlier.

**Effort provenance.** `process_torque_data` multiplies the raw effort by
per-joint `reduction_ratio × kmotor` constants hard-coded in
`identification.py` (100 or 336, torso 1; `kmotor` 0.136, −0.087, −0.0613,
torso 1). The config's `robot.properties.joints.reduction_ratios`
(`[32, 32, 45, -48, 45, 32]`) is a different, unused table. The torso adds
`9.81 × subtree mass`; with the nominal model this gives ~185 N, matching the
RNEA gravity load (184.9–186.3 N), so the raw torso effort is consistent with
a gravity-compensated measurement. Against nominal RNEA on the filtered
kinematics, converted torques correlate 0.04–0.78 per joint with fitted
scales far from 1: the nominal model has no friction and CAD inertias, so this
**neither confirms nor refutes** the constants. Signs agree where gravity
dominates (arm_2–6). No torque reference exists to verify units or scale.

**Wrist efforts.** `arm_5`–`arm_7` efforts are exactly zero on 88%, 90% and
90% of samples, with 40–46 distinct values in steps of 0.001: below the
recorder's resolution most of the time. Wrist dynamics are therefore weakly
observable from this recording. `arm_6` and `arm_7` are equal on 94% of all
samples, but mostly because both are zero; on samples where either is
non-zero they agree ~40% of the time (323 shared non-zero samples, 38 values).
That is weak evidence of coupled channels, recorded as a limit, not a defect.

**Window and decimation.** `truncate=(921, 6791)` keeps 9.21–67.90 s, the
excitation (RMS velocity 0.159 rad/s inside vs 0.017 before and 0.006 after).
`decimate=True` applies core's `scipy.signal.decimate` (factor 10, anti-alias
filtered) per joint to torque and regressor blocks, as audited for UR10.

## Corrections

1. **Filter at the recorded clock.** `tiago_unified_config.yaml` now sets
   `sampling_frequency: 100`, `cutoff_frequency: 2` and `filter_params.f_sample: 100`
   (the 2 Hz cutoff is what the filter already intended). The adapter rejects a
   filter sample rate more than 5% away from the recorded clock.
2. **Velocity delay removed.** The adapter estimates the delay per loaded file
   (`estimate_velocity_lag`: shift minimising the summed normalised mismatch
   with d(q)/dt on recorded timestamps, search 0–50 samples, refusing a
   boundary result), shifts the velocity channel earlier by that many samples
   and drops the same number of trailing rows from the other channels (no
   padding). `identification.py --velocity-lag {auto,N,0}` overrides it.
3. **Stricter loading.** Channels are selected by exact name in model order
   (previously substring matching), files must share identical, strictly
   increasing timestamps, and values must be finite.
4. **Provenance.** `trajectory_provenance[<source>]` records the recorded rate,
   filter rate, lag (samples and seconds), dropped rows, effort zero fractions
   and any duplicate-channel pairs. Warnings are logged for mostly-zero and
   duplicate effort channels.

Raw files, efforts, conversion constants, truncation and decimation are
unchanged.

## Model fitting

Default CLI (`identification.py`, full window, decimated):

| Metric | Before | After |
|---|---|---|
| Retained samples | 5870 | 5870 |
| Base parameters | 73 | 73 |
| Base-regressor condition number | 3617.7 | 2585.0 |
| Training torque RMSE | 0.849 | 0.666 (−22%) |

Held-out check (no TIAGo validation set exists): fit on rows 921–3856,
evaluate on 3856–6791 through core's `validation_data_file` path.

| Variant | Train RMSE | Held-out RMSE | Held-out max |
|---|---|---|---|
| Before | 0.928 | 1.441 | 9.62 |
| Filter clock only | 0.792 | 1.224 (−15%) | 10.33 |
| Filter + velocity from d(q)/dt (rejected) | 0.693 | 1.382 (−4%) | 14.91 |
| **Filter + velocity lag (shipped, lag 18 on each half)** | 0.717 | **1.167 (−19%)** | **9.07** |

Deriving velocity from positions fits training best but generalises worst, so
the measured channel is kept and realigned instead. Correlation is ≥ 0.9997 in
every variant and does not discriminate: large gravity terms dominate it.

## Retained limits

- The lag is a data property estimated from the recording itself (consistent
  at 18 samples across halves), assumed constant over the run.
- Effort units, signs and the `reduction_ratio × kmotor` constants remain
  unverified; they live in `identification.py`, not the config, and the
  config's `reduction_ratios` table is unused.
- Wrist efforts are mostly zero; parameters driven by wrist torques should not
  be trusted from this dataset.
- One recording; held-out evidence is a split of the same session, not an
  independent trajectory.

## Validation

See the PR for the full `validate.py` result and the tested revision pair.
