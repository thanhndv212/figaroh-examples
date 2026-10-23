# TIAGo base suspension, Vicon and force plate (#70): pre-registration

Issue: [#70](https://github.com/thanhndv212/figaroh-examples/issues/70).
This note is part 1: the pre-registration, to be merged before any fit of a
suspension parameter to a real run. It is frozen once merged. The study
(real-data fits, comparison with the baselines, verdicts) is part 2 and changes
no threshold written here. It was revised once before any real fit (the
"Revision" decisions of section 10, 2026-10-23): the earlier numbers stay
beside the current ones.

Question: on the 2023-09-20 CREPS runs (TIAGo on a force plate in a Vicon
room), is a linear spring-damper at the base enough to explain the wrench the
robot applies to the plate, and does it explain more than "the arm moved"? The
example `suspension_identification.py --data real` fits a 12-parameter
spring-damper to `xyz_1640` and explains only `My`; it is **not a suspension
estimate** (section 3, data README).

## 1. What this part contains, and what it does not

- The unified configuration
  `examples/tiago/config/tiago_suspension_study_config.yaml` (loader rules,
  model constants, selection rule, noise, verdict rules, fixture and its pass
  criteria). `tiago_suspension_config.yaml` is unchanged.
- The corrected loader in `examples/tiago/utils/suspension_data.py` (clock,
  base pose, frame, wrench transport). The previous loader stays reachable as
  `load_status_quo` (baseline B1) and its behaviour is unchanged.
- The library `examples/tiago/suspension_model.py` (sessions and roles, the
  `read_regressors` / `read_target` guard, S1/S2/S3, B0/B1/B2, block bootstrap,
  blocked-CV selection, nonlinearity flag, verdict rule, noise stage) and the
  fixture `examples/tiago/suspension_truth.py` with its golden
  `tests/golden/tiago_suspension_truth.json`.
- The thresholds below. They come from the fixture, which fits only
  **simulated** measurements.

No suspension parameter was fitted to any real session, and no wrench of the
training, validation or confirmation runs was used for anything but the
descriptive reads listed in section 3.

Disclosure of every real read ("Real reads"), in order:

- hash check of all eleven files (the only thing done to the confirmation runs
  `xyz_mirror_1630` and `xyz_mirror_1642`; they are never read);
- the clock of the nine other runs (receive stamps and the 39 values only to
  find identical rows): duplicates, drops, latency (section 3); the marker
  columns of the same runs for the near-duplicate count (O6, revision);
- the base-marker triad of the nine other runs: rest pose, origin, frame yaw,
  peak-to-peak motion (section 3);
- the plate columns of `xyz_1640` and `postures_1720` for the plate convention
  `M = (COP - c) x F` (centre fit, residual only: it is a check of the data's
  convention, not of the suspension); on the same two runs (revision): the rms
  of the z component of `(COP - c) x F` against `Mz` (O7) and the spectrum of
  the unfiltered `Fz` with its static mean (O8). These are descriptive reads of
  the training wrench; no parameter is fitted to them;
- the plate columns of `postures_1720` (base at rest) in the noise stage
  (section 5): white noise of the force and moment channels. The markers of the
  same run give the marker noise;
- the receive clock of `xyz_1640`, `xyz_1628` and `postures_1720` for the
  fixture's clock model;
- the displacement and velocity columns of `xyz_1640` and `xyz_1628`
  (regressors, no wrench): condition numbers and signal-to-noise (section 6),
  and the real base motion and gripper markers the fixture replays (section 7);
- the base and gripper marker columns of the five selection diagnostics
  (`around_*`, `*_fold`): regressors and fixture replay, no wrench (decision
  O3); no wrench of a selection diagnostic was read in part 1;
- the #100 Vicon 1607 fit and triad pose shipped in the repository, to locate
  the `base_footprint` point (section 3, decision O1; not suspension data);
- the plate channels of `xyz_1640` and `xyz_1628` are present in the loader
  output but no fit or score of them was computed (`xyz_1628`: no statistic
  either; `xyz_1640`: the reads above only).
  `suspension_model.read_target` raises `PermissionError` for the validation run
  unless `unlock_validation` is set (part 2 only) and always for the
  confirmation runs (tested).

## 2. Roles

`examples/tiago/data/suspension/protocol.yaml`, unchanged: training `xyz_1640`,
validation `xyz_1628`, confirmation `xyz_mirror_1630` and `xyz_mirror_1642`,
diagnostic `around_x_1634`, `around_y_1638`, `around_z_1622`, `x_fold_1632`,
`y_fold_1636`, `postures_1720`, `postures_1734`.

The confirmation runs are a true hold-out: nothing but a sha256 was read, and
no stage of part 1 can read them. The validation run is **not blind** for its
motion (its base motion and gripper markers are replayed by the fixture), but
its wrench is unread.

Diagnostic runs: `around_x_1634`, `around_y_1638`, `around_z_1622`,
`x_fold_1632` and `y_fold_1636` serve model selection (`h_e`, S1/S2/S3) together
with training; they are never confirmation, and the final fit and the verdict
stay on training and validation (maintainer decision O3). `postures_1720` and
`postures_1734` (base at rest) stay description only (noise floor, clock).

## 3. Measurement and loader

Columns (no header): host receive time, `base1..3`, `shoulder1..4`,
`gripper1..3` (x, y, z, m, Vicon world), plate 4 `Fx Fy Fz Mx My Mz COPx COPy COPz`.

**Clock.** Real receive stamps; frame numbers on the 100 Hz Vicon grid are
rebuilt with a lower envelope of the receive delay (`rebuild_clock`): for the
kept rows `n_k = k + D_k - D_0` with `D_k = floor(min_{j >= k} d_j)` and
`d_k = (t_k - t_0) rate - k`; the latency is then `(d_k - D_k) / rate`, at
least zero. Dropped frames are linearly interpolated on the grid and flagged;
rows within 5 frames of one are not fitted, nor the 0.5 s filter edges.
Measured (`describe`):

| run | rows | frames | drops | drop % | max gap | latency mean / sd (ms) |
|---|---|---|---|---|---|---|
| `xyz_1640` | 5547 | 5605 | 58 | 1.03 | 2 | 3.2 / 4.0 |
| `xyz_1628` | 5337 | 5410 | 73 | 1.35 | 4 | 9.7 / 1.2 |
| `around_x_1634` | 5123 | 5206 | 83 | 1.59 | 4 | 9.8 / 2.4 |
| `around_y_1638` | 5305 | 5361 | 56 | 1.04 | 2 | 9.6 / 1.7 |
| `around_z_1622` | 6150 | 6221 | 71 | 1.14 | 4 | 2.7 / 1.1 |
| `x_fold_1632` | 7361 | 7472 | 111 | 1.49 | 2 | 9.9 / 2.1 |
| `y_fold_1636` | 5873 | 5933 | 60 | 1.01 | 2 | 8.6 / 3.9 |
| `postures_1720` | 13706 | 13837 | 131 | 0.95 | 1 | 9.5 / 2.5 |
| `postures_1734` | 9918 | 10006 | 88 | 0.88 | 1 | 6.6 / 1.0 |

Drops are 0.9 to 1.6 % of the frames (the data README says 1.5 to 2.5 %), and
**no row is an exact duplicate of the previous one** in any run (the README
says 0.5 to 0.8 % duplicated, judged from position continuity): the duplicate
rule (all 39 values equal) removes nothing. A near-duplicate rule (every
marker coordinate within 3 marker sd, 0.033 mm, of the previous row while the
gripper moves, `describe` `near_duplicates`) finds **0** rows in each of the
nine runs: the README's duplicates are not in these files (O6, resolved; the
data README is corrected and the clock statistics stand). The latency level is only
defined up to the lowest latency of the run (hence the spread of the means).
The pooled statistics of the three runs the fixture uses are in the
configuration: drop rate 0.0105, latency mean 8.1 ms, sd 3.8 ms, lag-1
correlation 0.73.

**Base pose.** Origin: the `base_footprint` point (O1 of the decisions below),
a point rigidly attached to the base marker triad: `point = c0 + R_rest
offset`, with `c0` the rest centroid (mean of the first second) and `R_rest`
the rest frame of the triad. Axes: the footprint axes (x forward, z up),
`R_rest` times the fixed rotation of the footprint in the triad frame.
Rotation: Kabsch fit of all three markers to the rest triad, as a rotation
vector; translation of the footprint point, rotation vector and wrench are
expressed in the footprint axes. The footprint is **not** a point a pure
rotation of the triad leaves fixed: its motion contains `theta x r` of the 0.31
m lever. The status-quo loader takes marker `base1` (a corner) as the origin,
builds the frame `x = b2 - b1`, `z = x x (b3 - b1)` and reports RPY relative to
the first row. Measured on the nine runs: the triad frame, taken z-up, is
yawed -5.3 to -6.0 degrees from world about z (-5.54 on `xyz_1640`, -5.33 on
`xyz_1628`) and its normal `(b2 - b1) x (b3 - b1)` **points down** (confirming
the issue): the status quo mixes world translations with a down-pointing
frame's angles. The footprint point lies at z = -0.0038 to -0.0040 m in the
Vicon world in all nine runs (the plate surface; the rest centroid of the
triad is at (0.905, 0.453, 0.310) m, between runs 0.4 mm in x, 7 mm in y, 0.3
mm in z).

Derivation of the offset (`python examples/tiago/suspension_model.py
footprint`): the #100 Vicon 1607 fit gives the base pose `xb..phizb` and the
triad pose of 1607 gives the triad frame; the offset of the footprint in the
triad frame is (0.010973, -0.077375, 0.313235) m with sd (0.0022, 0.0026,
0.0016) m, the rotation (rotation vector in the triad frame) (-2.2686,
-2.1598, -0.0143) rad with sd 0.0024 rad. It is frozen in the configuration.

Peak-to-peak motion of the origin (mm) and rotation vector (mrad) at the
footprint, usable rows:

| run | x | y | z | rx | ry | rz |
|---|---|---|---|---|---|---|
| `xyz_1640` | 2.65 | 0.79 | 0.31 | 5.80 | 2.04 | 46.76 |
| `xyz_1628` | 3.03 | 0.80 | 0.30 | 5.41 | 1.65 | 39.47 |

(At the triad centroid in world axes the first version of this note had x 5.06,
y 3.48, z 0.31 mm for `xyz_1640`: the footprint is nearer the centre of the
base rotation about z.) Base motion is at the oscillation frequency, 0.2 Hz,
with power below about 3.5 Hz in x and y.

**Plate convention** (checked on `xyz_1640` and `postures_1720`). The wrench
is the action of the robot on the plate (static `Fz` about -730 N, the 74 kg of
the robot). The moments are about the plate centre `c = (0.900, 0.450, 0)` m
in world: `M = (COP - c) x F`, fitted as a free `c` from `Mx`, `My` with the
residual shown:

| run | `plate centre fit` (m) | residual rms (N m) |
|---|---|---|
| `xyz_1640` | (0.9000, 0.4500, -0.0000) | 8.6e-07 |
| `postures_1720` | (0.9000, 0.4500, -0.0000) | 5.7e-07 |

`Mz` is the free moment: its reference (about `c` or the COP) cannot be
verified from the data (O7). The two readings differ by the z component of
`(COP - c) x F`: rms 0.30 N m on `xyz_1640` (`Mz` sd 2.42 N m) and 0.026 N m on
`postures_1720`, above the plate noise of 0.12 N m on the moving run, so part 2
scores `Mz` under both (section 9). Moment transport to the origin `p_b` (rest
footprint point): `M_b = M_c + (c - p_b) x F`, then rotated to the footprint
axes. The status quo skips this and fits the
moments about the plate centre as if they were about the base.

**Filter and static offset.** Every channel is filtered with a zero-phase
4th-order Butterworth at **5 Hz** (revision; it was 10 Hz). The base motion is
at about 0.2 Hz; the lower cutoff lowers the noise of the velocity columns (the
errors-in-variables bias of F2, section 7.1) and keeps the vertical mode out of
the fit. That mode (O8): the largest peak of the unfiltered `Fz` between 10 and
45 Hz is at 24.2 Hz on `xyz_1640` (12 times the median of the band), which with
the static load of 74.4 kg implies `k_z = m (2 pi f)^2` = 1.7e6 N/m, about 17
times the fixture's `k_z`; at rest (`postures_1720`) the band holds only a weak
peak at 37.1 Hz (3 times the median). The velocity is the gradient of the
filtered displacement on the frame grid. The static value is the mean of the
first second (the robot is at rest in every
run) and is removed from the wrench; the models carry one constant per DOF.

## 4. Models and baselines

`w` is the 6-vector wrench of the plate about the origin (Fx, Fy, Fz, Mx, My,
Mz), minus its static value; `x` the 6-vector displacement (translation,
rotation vector) and `v` its derivative.

- **S1** `w = K x + C v`, `K`, `C` diagonal: 12 parameters, `k >= 0`, `c >= 0`
  (bounded least squares).
- **S2** the same about an **elastic centre** at `(0, 0, h_e)` in the base
  frame: `w = A diag(k) B x + A diag(c) B v`, `B` maps the displacement to the
  centre (`x + theta x r`), `A = B^T` maps back to the origin, the origin being
  the `base_footprint` point. `h_e` (above the footprint) on the grid 0 to 0.60
  m, step 0.01, by blocked CV on training and the selection diagnostics.
  12 + `h_e`, same bounds as S1.
- **S3** full symmetric `K` and `C`, both positive semi-definite: 21 + 21
  parameters (a semidefinite program solved with picos/cvxopt, on the
  diagonally scaled matrices; no new dependency).
- **S1B2** (revision) S1 plus B2, nested: 12 + 4 parameters, `k, c >= 0`,
  `m >= 0`. Linear; it carries the secondary question Q2 (section 8).
- Each with one constant per DOF (static drift), not reported.
- **B0** static: predicts the static wrench (R2 = 0 by construction).
- **B1** the status quo: `status_quo_from_arrays` and the plain 12-parameter
  least squares of `utils/suspension_model.py` (no offset, no weights).
- **B2** rigid base: the arm load alone, a rigid cube whose pose follows the
  gripper-marker triad (position and frame from `gripper1..3`). The wrench on
  the plate is linear in the mass `m` and the first moment `m o` (`o` the
  cube centre in the gripper frame): `F = m (g - a_c) - R'' (m o)`, `M = (c - p)
  x F + R (m o) x (g - a_c) - I w'` with the isotropic inertia `I = m a^2 / 6`,
  edge `a` = 0.11 m (an assumption: 159 mm, the largest marker distance, is a
  face diagonal of a 0.112 m cube). 4 parameters, `m >= 0`. The term
  quadratic in `o` is neglected. The arm itself is not modelled. The
  competitor that tests whether the suspension explains more than "the arm
  moved" (O2).

Fit: constrained least squares (bounds for S1/S2/B2/S1B2, PSD for S3) on unit-RMS
columns, rows weighted by the inverse standard deviation of each DOF's target.
Standard errors: moving-block bootstrap over the usable rows, 200 resamples,
5 s blocks (one oscillation), seed 7, same constraints. Selection: the simplest
of S1 < S2 < S3 within 5 % of the best 5-fold blocked CV score (10 s blocks),
pooled over training and the five selection diagnostics (blocks dealt to folds
per run); the final fit is on training only. The metrics are held-out R2 and
NRMSE per DOF, **with the mean of target and prediction removed** (a constant
is not a suspension quantity), the active-bound report (a parameter at its
bound; a PSD matrix with a zero eigenvalue flags all six DOF; a damping at its
bound or with a relative SE above 50 % is "damping not identified", O9) and the
repeatability of the stiffnesses between a training and a validation refit
(refit on validation: part 2 only).

## 5. Noise stage

`python examples/tiago/suspension_model.py noise` (the `postures_1720` run, base
at rest). White levels from first differences (1.4826 MAD(diff)/sqrt 2); the
same levels pushed through the loader (Kabsch pose about the footprint, in the
footprint axes, 5 Hz filter, gradient) for the displacement and velocity
columns. The clock statistics are pooled over `xyz_1640`, `xyz_1628` and
`postures_1720`. The levels are frozen in the configuration; grid levels 0.5, 1
and 2 times the white levels, used level 1, level 0 the noise-free check (no
white noise and no clock error). Revision: the first freeze pushed the noise
through the triad centroid in world axes and the 10 Hz filter (values in
brackets); the loader of the fit uses the footprint, so the levels are
refrozen with it.

| quantity | value |
|---|---|
| `noise marker` | 0.0111 mm per coordinate (unchanged) |
| `noise plate force` | Fx 0.32, Fy 0.28, Fz 0.93 N (unfiltered; 0.099, 0.083, 0.28 N after the 5 Hz filter) |
| `noise plate moment` | Mx 0.22, My 0.27, Mz 0.12 N m (unfiltered; 0.066, 0.081, 0.036 N m after the filter) |
| `noise displacement` | x 5.4e-06, y 4.1e-06, z 2.3e-06 m; rx 1.1e-05, ry 1.6e-05, rz 9.4e-06 rad (x, y, z 2.7e-06 m; 2.2e-05, 1.6e-05, 1.3e-05 rad) |
| `noise velocity` | x 9.3e-05, y 7.0e-05, z 4.0e-05 m/s; rx 1.9e-04, ry 2.7e-04, rz 1.6e-04 rad/s (x, y, z 8.8e-05 m/s; 7.2e-04, 5.4e-04, 4.3e-04 rad/s) |
| `noise clock` | drops 1.05 %, latency 8.1 ms (sd 3.8, lag-1 0.73), duplicates 0, near-duplicates 0 |

The x and y translation noise of the footprint is larger than that of the
centroid: the rotation noise of the triad moves a point 0.31 m below it. With
the 5 Hz filter the rotation-rate noise is a quarter to a half of the earlier
value.

## 6. Identifiability (real motion, F1)

`python examples/tiago/suspension_truth.py` (F1 part). Scaled condition number
of the weighted design on the real training and validation motion (S2: worst
over the `h_e` grid); signal-to-noise = sd of the usable column over the noise
column of section 5. Criterion: scaled condition number below 1e3 for S1 and
S2 and 1e4 for S3. Passed (before the revision in brackets).

| | training | validation |
|---|---|---|
| `condition s1` | 2.29 | 2.34 |
| `condition s2` | 2.71 | 2.44 |
| `condition s3` | 17.89 | 13.30 |

Before: training 2.29, 2.69, 14.66; validation 2.34, 2.43, 10.53.

Signal-to-noise per DOF (x, y, z, rx, ry, rz), training:

| columns | x | y | z | rx | ry | rz |
|---|---|---|---|---|---|---|
| displacement | 65 | 39 | 19 | 65 | 23 | 426 |
| velocity | 12.0 | 11.0 | 6.6 | 17.4 | 5.5 | 131.7 |

Before: displacement 130, 61, 16, 34, 23, 304; velocity 18.1, 14.1, 4.4, 6.1,
4.0, 56.1. Validation: displacement 68, 35, 20, 61, 14, 700; velocity 16.1,
11.1, 6.8, 20.3, 4.8, 144.9. No column is below the reported limit of 3 (the
z-translation columns, the issue's worry, have 19 and 6.6). The change comes
from the refrozen noise (section 5), not from the motion. These are the SNR of
the **regressors** against measurement noise; the fixture uses the measured
noise only. Damping is weaker than stiffness in every column: the velocity
column is `2 pi 0.2 = 1.26` times the displacement, so `c` contributes a few
percent of the wrench (section 7, F2: its SE is 1 to 10 % of the truth here,
and a real suspension need not behave like the fixture truth: the rule of O9).

## 7. Pre-registered thresholds (simulated)

Fixture `suspension_truth.py`, seeds 0 to 4. The truth: `k` = (1e4, 1e4, 1e5,
3e4, 2e4, 1e3) N/m (x, y, z) and N m/rad (rx, ry, rz), `c` = (150, 150, 500,
100, 80, 12), each parameter times `1 +- 25 %` per seed. The values are the
order of magnitude of the ratio of the standard deviations of the plate wrench
and the base displacement, descriptive and not a fit. The base motion is the
**real** corrected pose of `xyz_1640` (fit) and `xyz_1628` (held out), held at
rest in the first second, about the footprint point and in the footprint axes;
the arm marker columns are the real ones; the three base markers (the footprint
motion mapped back through the rest triad), the plate (static -730 N, moments
about the plate centre) and the receive clock (measured drops and latency) are
simulated from the truth, with noise at the used level. The same motion
replay serves the selection diagnostics (`around_*`, `*_fold`), whose real
base and gripper markers are replayed in the same way (section 1, "Real
reads"). Truth streams: `default_rng(seed)`; noise
`default_rng(10000 + 100 k + seed)`, `k` = 0 white, 1 clock. The criteria
were written in the configuration before the first run and revised once
(section 10, "Revision"), before any real fit.

The ceiling of a DOF is the held-out R2 of the true parameters on the same
noisy regressors. A DOF is **answerable** if the mean fixture SE of its
stiffness over the truth is at most 50 %. All six are answerable (stiffness SE
0.04 to 0.3 % of the truth; damping SE 1.1 to 9.6 %: `context`).

### 7.1 F2 to F3: recovery and the status-quo signature

Results of the revised fixture (5 Hz filter, refrozen noise; F2 bias and F3b
added by the revision) next to the previous run ("before": 10 Hz, centroid
noise):

| criterion | pass rule | result (before) |
|---|---|---|
| F2 exact | level 0: relative parameter error below 1e-6 | pass, 4e-08 (5.8e-10) |
| F2 recovery | pooled z-RMS of the 12 parameters in [0.75, 1.33] and abs(mean z) below 0.3 at every level | **fail**, kept and reported (fail) |
| F2 bias (revision) | at level 1, mean relative error over seeds of every answerable stiffness within 1 % and of every identified damping within 10 % | pass, k 0.20 %, c 9.6 % (not a criterion) |
| F2 ceiling | held-out R2 within 0.05 of the ceiling, answerable DOF, every level | pass (pass) |
| F3 signature | status quo + S1 reproduces forces R2 < 0.2, My R2 >= 0.75, >= 3 negative dampings, in >= 4 of 5 seeds | **fail** (fail) |
| F3b signature (revision) | the same signature under the mixed truth (S1 spring plus the F4c cube), in >= 4 of 5 seeds | pass, 4 of 5 (not a criterion) |
| F3 attribution | fixing one error raises the mean force R2 by 0.2 or more | pass, origin only (pass, origin only) |
| F3 full fix | all four fixes + filter within 0.05 of the ceiling, answerable DOF | pass (pass) |

| | 0.5 x noise | 1.0 x noise | 2.0 x noise |
|---|---|---|---|
| `recovery z rms` | 0.98 | 1.05 | 1.24 |
| `recovery z mean` | -0.30 | -0.45 | -0.70 |
| `recovery rel err` | 0.09 | 0.20 | 0.45 |
| `recovery gap` | 0.00 | 0.00 | 0.00 |
| `recovery eiv z rms` | 0.95 | 0.95 | 0.96 |
| `recovery eiv z mean` | -0.13 | -0.12 | -0.10 |

Before: z rms 1.24, 1.53, 2.07; z mean -0.66, -1.01, -1.55; rel err 0.11, 0.26,
0.55. F2 recovery still fails, now on the mean z only: the z-RMS is inside
[0.75, 1.33] at every level, and the mean z is -0.30 (0.5 x, just over the
limit), -0.45 and -0.70. The largest relative parameter error is 9, 20 and 45 %
of the truth. The held-out R2 is at the ceiling at every level
(gap 0 to -4e-5) and exact when noiseless; no parameter is at a bound and no
damping is unidentified on the recovery fits.

**Cause (corrected).** The first version blamed the clock reconstruction. That
is wrong: the cause is **errors in variables**. The displacement and velocity
columns are built from the noisy markers, so the noise of the regressors
attenuates the least-squares estimate towards zero (the classical errors-in-
variables bias), which the block bootstrap of the residuals does not see. The
velocity column, the derivative of the marker noise, is the noisiest, so the
dampings carry the bias. The 5 Hz cutoff lowers the velocity noise and
the bias (mean z at level 1 from -1.01 to -0.45); the remainder is measured by a correction. `fit_s1_eiv` subtracts the
noise power of each column (`n sd^2`, the frozen levels of section 5) from the
normal equations, unconstrained. With it the z of the 12 parameters is centred
(mean -0.13 to -0.10, rms 0.95), which would pass F2. It is a sensitivity
row and not the estimator: it needs the noise level to be known, and the real
regressor noise is the frozen estimate of section 5, not a truth.

Bias at level 1 (mean relative error over the five seeds, x, y, z, rx, ry, rz):

| | x | y | z | rx | ry | rz |
|---|---|---|---|---|---|---|
| `bias k` | +0.02 % | -0.20 % | -0.17 % | -0.04 % | -0.18 % | -0.00 % |
| `bias c` | -0.1 % | -0.9 % | -9.6 % | -2.3 % | -5.4 % | -0.2 % |
| `bias c eiv` | +0.6 % | -0.0 % | -7.5 % | -2.0 % | -2.0 % | -0.2 % |
| `rel se k` | 0.23 % | 0.27 % | 0.30 % | 0.07 % | 0.18 % | 0.04 % |
| `rel se c` | 2.7 % | 4.2 % | 9.6 % | 3.5 % | 9.4 % | 1.1 % |
| `rel se c calibrated` | 2.7 % | 4.3 % | 13.6 % | 4.2 % | 10.8 % | 1.1 % |

The stiffnesses are biased by at most 0.2 % (their SE is 0.04 to 0.3 %); the
damping of z by -9.6 %, at the limit of F2 bias. Part 2 reports the
**calibrated SE** `sqrt(SE^2 + bias^2)` of the fixture, relative to the
estimate, beside the bootstrap SE, and keeps the bootstrap SE as its lower bound.

F3 at level 1, status quo (B1, unconstrained, unchanged) on the S1 truth, held
out on the simulated `xyz_1628`:

| quantity | value |
|---|---|
| `status quo forces r2` | 0.46 (seed 0: Fx 0.07, Fy 0.55, Fz 0.75) |
| `status quo my r2` | 0.99 (seed 0: Mx 0.87) |
| `status quo negative damping` | 3.2 of 6 DOF on average (2 to 5) |
| `signature seeds` | 0 of 5 (forces R2 0.43 to 0.48) |
| `ablation origin` | +0.49 |
| `ablation transport` | +0.00 |
| `ablation frame` | +0.02 |
| `ablation clock` | +0.04 |

Before: forces R2 0.44, My 0.98, negative dampings 2.8, ablations origin +0.50,
transport +0.00, frame +0.02, clock +0.03. Under an S1-diagonal truth the status
quo still does **not** reproduce "only My": forces R2 0.46 (limit 0.2); 0 of 5
seeds pass. F3 stays a reported fail. The corner-marker origin is the one error
that moves the force fit (+0.49 of mean force R2); the moment transport and the
frame change the **moment** rows, which this metric hardly sees, and the clock
error costs 0.04. After all four fixes the S1 fit is at the ceiling (F3 full
gap 3e-06).

**F3b, mixed truth (revision).** The truth is the S1 spring plus the F4c arm
cube (25 kg, offset (0.04, -0.03, 0.05) m), added. The status quo then
reproduces the signature in 4 of 5 seeds: forces R2 0.15 (seed 4: 0.22), My
0.93, 3.4 negative dampings on average; mean R2 (Fx..Mz) -0.08, 0.08, 0.43,
0.64, 0.93, 0.40. So "only My" is the signature of an arm path **plus** the four
loader errors, not of the errors alone (F3) nor of the arm alone (F4c). This
is a fixture statement: the real wrench may hold other paths. **Caveat:** an
additive mix means that the cube's inertial load reaches the plate in parallel
with the spring, that is, through a load path stiff compared with the
suspension (the arm and the base frame on top of the springs move with the
base markers). A real arm load passes through the suspension in series; the
mixed truth is a bound case and not a model of the robot.

### 7.2 F4: misspecified truths

| criterion | pass rule | result (before) |
|---|---|---|
| F4a elastic centre (revised rule) | S2 selected within 0.05 of the ceiling and `abs(h_e - truth) <= 0.05`, in >= 4 of 5 seeds; the S2 gain over S1 is descriptive | **pass**, 5 of 5, gap 0.00 (pass under the old rule, gain 0.17, `h_e` 0.25) |
| F4b nonlinear | hardening and deadband flagged in >= 4 of 5 seeds, 0 flags under the linear truth | pass (5 of 5 and 5 of 5, 0) (same) |
| F4c arm load | "enough" false in >= 4 of 5 seeds under the arm-only truth | pass (5 of 5) (same) |
| F4c Q2 (revision) | Q2 (section 8) absent in >= 4 of 5 seeds under the arm-only truth | pass, absent in 5 of 5 (not a criterion) |
| F4d mixed (revision) | under the mixed truth: S1 "not enough" >= 4, S1B2 within 0.05 of the ceiling >= 4, Q2 present >= 4 of 5 seeds | pass, 5, 5 and 5 of 5 (not a criterion) |

**F4a (revised: a test-design change).** The truth's elastic centre is now
`h_e` = **0.0985 m** above the footprint, the height of `base_link` in the
URDF, the place a real elastic centre of a wheeled base is first expected (the
wheel axles and the springs sit between the floor and the base frame). The
previous run placed it at 0.25 m, kept from the 25 cm lever of the first
version (the first version's position, 0.06 m above the footprint, failed the
gain). The pass rule is changed with it: the old "S2 beats S1 by 0.1" is a
statement about the lever, not about the estimator, and fails for any centre
near the origin. The new rule asks what part 2 uses: that S2 is selected,
reaches the ceiling and finds `h_e`. Result: S2 selected in 5 of 5 seeds, gap
1.4e-5, `h_e` = 0.10 on the 0.01 m grid in every seed. The gain is
**0.03** (`elastic gain`, 0.033 to 0.037; mean held-out R2 S1 0.96, S2 1.00;
seed 0: the worst S1 DOF is Fy, 0.80 against 0.99), reported only.
Sensitivity rows (same rule):

| truth `h_e` (m) | `h_e` found | S2 gain over S1 | F4a rule |
|---|---|---|---|
| 0.065 | 0.06 to 0.07 | 0.01 | pass, 5 of 5 |
| 0.0985 (used) | 0.10 | 0.03 | pass, 5 of 5 |
| 0.25 (previous) | 0.25 | 0.17 | pass, 5 of 5 |

The gain scales with the lever (about the square of `h_e`); the selection
finds the centre at every lever. A real S1/S2 gain of the order of 0.03 is
therefore consistent with an elastic centre at the base frame.

F4b: the nonlinearity flag (|correlation| of the S1 residual with the cubic
part of the displacement, above 0.2) fires on the hardening truth (x, y, z) and
the deadband truth (x, y) in every seed (corr 0.93 to 1.0 on the hardening DOF,
0.71 to 0.86 on the deadband DOF) and never on the linear truth (largest |corr|
0.14, on a moment DOF).

F4c: the truth has **no suspension**: the real base motion and a wrench of the
B2 rigid cube of 25 kg, its centre offset from the gripper-marker frame by
(0.04, -0.03, 0.05) m (the wrench path the spring-damper cannot see). Mean
held-out R2 over seeds (x, y, z, rx, ry, rz):

| model | Fx | Fy | Fz | Mx | My | Mz |
|---|---|---|---|---|---|---|
| S1 `arm only s1` | 0.12 | 0.00 | 0.10 | 0.89 | 0.63 | 0.28 |
| B2 `arm only b2` | 0.97 | 0.98 | 0.94 | 1.00 | 1.00 | 0.97 |
| S1B2 `arm only s1b2` | 0.97 | 0.98 | 0.95 | 1.00 | 1.00 | 0.97 |

Before: S1 0.07, 0.00, 0.02, 0.86, 0.60, 0.25; B2 0.92, 0.90, 0.95, 0.98, 0.99,
0.88. B2 recovers a mass of 24.3 kg (before 23.5; truth 25) and a first-moment
offset (0.066, -0.053, 0.047) m (truth (0.04, -0.03, 0.05); the term quadratic
in the offset, neglected by the linear fit, and the marker noise account for
the difference). The rule of section 8 says "not enough" in 5 of 5 seeds. In
every seed the S1 stiffness of Fy is at its bound (the active-bound item); the
dampings of x, y and z are mostly "not identified" (O9), which no longer fails
a DOF by itself. B2's `m` is never at its bound. S1B2 adds at most 0.007 of R2
to B2 on any DOF, so Q2 is absent in 5 of 5 seeds: the nested model does not
invent a suspension when there is none. Effect of the assumed inertia (B2
refitted with the edge 0 and doubled): the held-out R2 of any DOF changes by at
most 0.0008 (before 0.003).

**F4d, mixed truth (revision).** The F3b truth (S1 spring plus the cube),
through the fixed loader. Mean held-out R2 over seeds:

| model | Fx | Fy | Fz | Mx | My | Mz |
|---|---|---|---|---|---|---|
| S1 `mixed s1` | 0.27 | 0.00 | 0.51 | 0.93 | 0.68 | 0.50 |
| B2 `mixed b2` | 0.89 | 0.95 | 0.73 | 0.96 | 1.00 | 0.82 |
| S1B2 `mixed s1b2` | 0.97 | 0.98 | 0.97 | 1.00 | 1.00 | 0.98 |
| ceiling `mixed ceiling` | 0.96 | 0.97 | 0.96 | 1.00 | 1.00 | 0.98 |

S1 is "not enough" in 5 of 5 seeds; S1B2 is within 0.05 of the ceiling in 5 of
5 (gap -0.004: the ceiling is the S1B2 design at the true parameters, and the
true cube wrench has the quadratic term the linear B2 lacks, so the fit can
exceed it); Q2 is present in 5 of 5 (S1B2 adds 0.1 or more over B2 on Fz and
Mz in every seed, on Fx in one). **Caveat:** the S1B2 stiffnesses are confounded
with the cube under the mix: their mean relative error is 1.2, 33, 2.6, 19, 31
and 8.3 % (x to rz), and the mass is off by -4.1 %. Q2 answers "does a
suspension add to the arm path", not "what are its parameters"; under a mixed
wrench part 2 reports the S1B2 stiffnesses as descriptive.

### 7.3 F5: selection

| criterion | pass rule | result (before) |
|---|---|---|
| F5 selection | S1 chosen under the S1 truth and S2 with `abs(h_e - truth) <= 0.05` under the elastic truth, each in >= 4 of 5 seeds | pass (pass) |

| | result |
|---|---|
| `selection s1` | 5 of 5 |
| `selection s2` | 5 of 5 (`h_e` = 0.10 m above the footprint in every seed, truth 0.0985, on the grid 0 to 0.60; before 0.25, truth 0.25) |

The CV is pooled over training and the five selection diagnostics; the
S1-truth CV picks `h_e` = 0 (the grid floor, S2 reduces to S1) with S1 within
5 % of the best (seed 0: CV score 0.061 for S1 and S2; 0.197 and 0.074 for S1
and S2 under the elastic truth).

### 7.4 Origin sensitivity and B2 edge (descriptive, no threshold)

`origin sensitivity` (the S1 truth, level 1, five seeds): the truth is
measured about the nominal footprint, and the loader is given a moved origin;
the table is the mean over seeds of the largest relative change of the 12 S1
parameters and the change of mean held-out R2, against the nominal loader
(nominal R2 0.997, before 0.994).

| origin | max relative change of a parameter | change of mean R2 (worst seed) |
|---|---|---|
| footprint +1 sd x | 6.3 % | -0.0002 (-0.0003) |
| footprint -1 sd x | 6.2 % | -0.0002 (-0.0002) |
| footprint +1 sd y | 3.3 % | -0.0023 (-0.0024) |
| footprint -1 sd y | 3.3 % | -0.0024 (-0.0026) |
| footprint +1 sd z | 1.1 % | -0.0000 (-0.0000) |
| footprint -1 sd z | 1.2 % | -0.0000 (-0.0000) |
| triad centroid (ablation) | 100 % (a damping; mean 39 %) | -0.193 (-0.197) |

Before: 3.2, 3.1, 1.6, 1.6, 0.6, 0.6 % and the centroid 100 % (mean 32 %). The
footprint offset uncertainty (2 mm, 3 mm, 2 mm) moves a parameter by up to
6 % (the x and z dampings, the z stiffness) and the R2 by less than 0.003,
about twice the changes of the 10 Hz run. The triad centroid, 31 cm above the footprint, moves a damping by
100 % and the R2 by 0.19. The origin matters at the lever of the base, not at
the survey's uncertainty. The rotation uncertainty (0.0024 rad) is not
perturbed.

## 8. Verdict rules (written before the real fit)

Applied to the validation run (part 2), with the model frozen on training
(selection of section 4). A DOF is:

- **explained** if its held-out R2 on validation is at least 0.5 (`explained_r2`);
- **beating B2** if the selected model's R2 exceeds B2's by at least 0.1;
- **repeatable** if its stiffness differs by at most `band` between the
  training fit and the validation refit. `band` is 2 times the RMS relative
  difference of the stiffnesses between the fixture's training and validation
  fits (`k_rel_diff_rms` 0.0031, `band` 0.0061; before 0.0039 and 0.0077) --
  so tight that any real session-to-session change will exceed it, which is
  read as "not repeatable to the noise" (O4);
- free of an **active stiffness bound**: no stiffness of the DOF at its bound
  of 0, no zero eigenvalue of the S3 `K` (all six DOF then), and B2's `m` not
  at 0 (the comparison with B2 would be vacuous);
- free of the **nonlinearity flag** (correlation above 0.2).

**Damping not identified** (revision, O9): a damping at its bound of 0, or
with a bootstrap SE above 50 % of its value (`damping_rel_se_max`), is reported
as "not identified" and does not fail the DOF; the DOF is judged on its
stiffness and its R2. A stiffness at its bound still fails it.

The **spring-damper is enough** iff on every answerable DOF the DOF is
explained, beats B2, is repeatable, has no active stiffness bound and no flag.
Then the confirmation runs must pass the same rule with the frozen model.
Meanings of a failure:

- explained on `My` only, and B2 matches it: the moment fit is the arm's
  gravity load, not a suspension estimate;
- explained but not beating B2: the suspension adds nothing the arm motion does
  not;
- active stiffness bound: the unconstrained fit wants a negative stiffness (the
  arm's inertial coupling, a wrong origin); the constrained fit sits on the
  bound and the DOF is not a spring-damper estimate;
- not repeatable: parameters are session properties, not constants;
- flag: the spring is nonlinear on that DOF.

Not answerable: a DOF whose stiffness SE in the fixture is above 50 % of the
truth (none in the fixture); the DOF is reported but does not enter "enough".

**Secondary question Q2** (revision, O10): *does a suspension add to the arm
path?* Q2 is present on a DOF if the nested S1B2 beats B2 on validation by at
least 0.1 of R2 (`suspension_adds`) and the S1B2 stiffness of the DOF is not at
its bound; Q2 is present if any answerable DOF has it. Q2 does not change
"enough"; it separates "an arm load only" (F4c: Q2 absent) from "a suspension
plus an arm load" (F4d: Q2 present). Its stiffnesses are descriptive (F4d
caveat).

**Uncertainty.** Each parameter is reported with the bootstrap SE (a lower
bound, F2) and the calibrated SE `sqrt(SE^2 + bias^2)` of section 7.1.

## 9. Design of part 2 (not run)

1. Load `xyz_1640` and `xyz_1628` through `suspension_model.read_regressors`;
   read the wrenches with `read_target` (training; validation with
   `unlock_validation` in the study script only).
2. B0, B1, B2 (cube), S1B2 and the constrained S1/S2/S3 on training; selection
   by blocked CV over training and the selection diagnostics; bootstrap SE and
   the calibrated SE.
3. Score the frozen model, B2 and S1B2 on validation; refit on validation for
   repeatability.
4. Apply section 8 ("enough", then Q2); confirmation runs only if validation
   says "enough", read once, with the frozen model.
5. Selection diagnostics (O3) join training in the CV of step 2 (wrench read
   with `read_target`, which allows them and never the confirmation runs); the
   posture runs stay description only. An active stiffness bound (O4) is
   reported and fails the DOF; an unidentified damping (O9) is reported.
6. `Mz` (O7): the term `(COP - c) x F` of `xyz_1640` (0.30 N m rms) exceeds the
   plate noise (0.12 N m), so `Mz` is scored with the free moment taken about
   `c` and about the COP; `Mz` counts in the verdict only if both give the same
   verdict, otherwise it is reported as convention-dependent and leaves the
   answerable set.
7. Vertical mode (O8): the fitted `k_z` is compared with the 1.7e6 N/m of the
   24.2 Hz `Fz` peak (section 3), as a description: a `k_z` one order below is
   the quasi-static stiffness at 0.2 Hz and the peak is then not the suspension
   mode. The 10 to 45 Hz band of `Fz` is also read on the posture runs: a peak
   that persists at rest is a structural or plate mode (on `postures_1720` it
   is weak, 3 times the median, at 37 Hz).

## 10. Open points for the maintainer

- **O1 base origin** (resolved by the decisions below). The origin is the
  `base_footprint` point, derived from the shipped #100 Vicon data (offset
  (0.0110, -0.0774, 0.3132) m in the triad frame, sd 2 to 3 mm); a point not
  fixed under a rotation of the triad, so the translation columns carry
  `theta x r` of a 0.31 m lever. Section 7.4 gives the sensitivity.
- **O2 B2 definition** (resolved). A rigid cube at the gripper markers, linear in
  `m` and `m o`, isotropic inertia from an assumed edge; the arm itself is not
  modelled and a link-by-link model needs robot channels that are not
  recorded.
- **O3 diagnostics** (resolved). The `around_*` and `*_fold` runs serve model
  selection with training; they are not confirmation. The posture runs stay
  description only.
- **O4 fixture failures** (not relaxed): F2 recovery (z-RMS 0.98 to 1.24, now
  inside the limits; mean z -0.30 to -0.70: errors in variables, section 7.1)
  and F3 signature (the status quo does not reproduce "only My" under a spring
  truth; it does under the mixed truth, F3b). F4a passes under the revised rule
  (section 7.2). Consequences: part 2 reads the bootstrap SE as a lower bound
  and reports the calibrated SE; the repeatability band (0.0061) stays so
  tight that any real session-to-session change exceeds it; the S1/S2 gain of
  the real run is read against the sensitivity rows of F4a.
- **O5 "with and without weight"** (closed, out of scope). The OptiTrack
  sessions of the plan (#112) have no force-plate data; this study uses only
  the Vicon day.
- **O6 clock** (resolved by the revision). The data README reported 0.5 to
  0.8 % duplicated frames. The near-duplicate rule (section 3) finds none in
  the nine runs, so these duplicates are not in the shipped files; the README
  is corrected and the clock statistics stand.
- **O7 `Mz`** (resolved by the revision). The convention cannot be checked, but
  its effect can: the convention term is 0.30 N m rms on `xyz_1640`, above
  the plate noise, so part 2 scores `Mz` under both readings (section 9, step 6).
- **O8 bandwidth** (resolved by the revision). Cutoff 5 Hz; the 24.2 Hz mode
  implies `k_z` = 1.7e6 N/m, a cross-check for part 2 (section 9, step 7).
- **O9 damping** (resolved by the revision). A damping at 0 or with a relative
  SE above 50 % is "not identified" and does not fail the DOF (section 8). In
  the fixture no damping of the S1 truth is unidentified; under the arm-only and
  mixed truths the x, y and z dampings mostly are.
- **O10 F4c definition** (resolved by the revision). F4c keeps the arm-only
  truth; F4d adds the mixed truth, the nested S1B2 and the question Q2
  (sections 7.2 and 8). Under the mix S1 is "not enough" and Q2 is present.
- **O11 plate-frame convention** (resolved). The wrench is expressed in the
  footprint axes about the footprint origin; the status-quo and triad-world
  conventions stay reachable (`frame: world`, `base`, `legacy`).

### Decisions taken (maintainer, 2026-10-23)

Taken after the fixture of the first version of this note was seen and before
any suspension parameter was fitted to a real session. The thresholds of
section 7 and the repeatability band are unchanged; the failures of F2 and F3
stay reported as results (F4a: see below). Sections 2 to 10 are revised for
these decisions and carry the rerun fixture, with the first version's numbers
beside the new ones; the config `tiago_suspension_study_config.yaml` carries
the frozen values.

- **O1 origin: the `base_footprint` frame, from shipped data only.** The origin
  is the `base_footprint` frame origin (`base_link` is 0.0985 m above it in the
  URDF), a point rigidly attached to the base marker triad. It is located with
  the #100 Vicon 1607 fit (`tests/golden/tiago_mocap_hardware.json`,
  `fits/V1607|3d`, `params` and `std`) and the 1607 base-triad pose
  `xb..phizb` of `data/calibration/vicon/`: the fitted base pose is expressed
  in the triad frame, which gives a robot-fixed offset, and the offset is
  mapped to each suspension run through that run's rest triad. The pose axes
  are the footprint axes (x forward, z up); the displacement, the rotation
  vector and the wrench are expressed in them, and the moments are transported
  to the footprint origin. Derived offset in the triad frame (z points down):
  (0.010973, -0.077375, 0.313235) m with 1 sd (0.0022, 0.0026, 0.0016) m:
  7.8 cm horizontal and 0.313 m below the triad centroid. Check: mapped
  through the rest triad of `xyz_1640`, the footprint lies at z = -0.004 m in
  the Vicon world, that is on the plate surface, and the footprint z axis is
  up (the triad z sign is handled by the rotation to the footprint axes). The
  offset and its uncertainty are frozen in the config. The fixture adds a
  descriptive sensitivity check: the offset moved by +-1 sd per axis and the
  triad centroid as an ablation origin. The footprint rotation uncertainty
  (0.0024 rad) is not perturbed. The S2 grid `h_e` is re-expressed from the
  footprint, 0 to 0.60 m (floor at 0), where it was -0.50 to 0.30 m from the
  triad centroid (that is -0.19 to +0.61 m from the footprint). The F4a truth
  `h_e` becomes +0.25 m above the footprint (the old -0.25 m below the centroid
  would be +0.06 m above the footprint; the lever magnitude of 25 cm is kept
  instead, a deliberate change of the truth, reported).
- **O2 B2 is a rigid cube at the gripper markers.** Its pose follows the
  gripper triad (frame built like the base one: x from `gripper1` to
  `gripper2`, z = x cross (`gripper3` - `gripper1`)); inter-marker distances
  94.4, 106.6 and 159.0 mm, so the centre is not the marker centroid. Linear fit
  of the mass `m` and the first moment `m o` (`o` the centre in the gripper
  frame): 4 parameters, `m` >= 0. Isotropic inertia `m a^2 / 6`, the edge
  `a` = 0.11 m is an assumption frozen from the marker spread (159 mm is a face
  diagonal of a 0.112 m cube); its effect is reported. The arm itself is not
  modelled. F4c uses this B2 as truth (a cube whose centre is offset from the
  marker centroid).
- **O3 diagnostic runs may serve model selection.** `around_x_1634`,
  `around_y_1638`, `around_z_1622`, `x_fold_1632` and `y_fold_1636` are used,
  with training, for the choice of `h_e` and of S1/S2/S3 (blocked CV over all
  of them, blocks dealt per run). The two posture runs (base at rest) stay
  out. Never confirmation; validation stays held out for the verdict, and the
  final fit stays on training. The CV and F5 are updated.
- **O4 constrained fits.** S1 and S2: `k` >= 0 and `c` >= 0 (bounded least
  squares). S3: `K` and `C` symmetric positive semi-definite (a semidefinite
  program; no new dependency). The bootstrap uses the same constraints. The
  verdict item "negative damping `c < -2 SE`" is replaced by "active bound": a
  parameter at its bound (or a PSD matrix with a zero eigenvalue, then all six
  DOF) is reported and the DOF fails "enough"; B2 `m` >= 0 likewise (an active
  `m` makes the comparison with B2 vacuous, so it fails "enough"). The
  repeatability band is unchanged and stays flagged (O4 below). The existing
  F2, F3 and F4a failures stay reported as results, with the thresholds
  unchanged.
- **O5 closed, out of scope.** The OptiTrack sessions of the plan (#112) have no
  force-plate data and cannot serve this study. No code change.

Fixture result of these decisions (before the revision below, which
supersedes these numbers in sections 3 to 9): F1, F2 exact, F2 gap,
F3 attribution, F3 full, F4b, F4c and F5 pass as before; F2 recovery (z-RMS
1.24, 1.53, 2.07) and the F3 signature (0 of 5 seeds; forces R2 0.44) still
fail; F4a moves from fail (gain 0.03) to pass (gain 0.17), which is caused by
the F4a truth, not by the estimator (7.2). Deviations and assumptions to read
with it:

- the F4a truth `h_e` is +0.25 m above the footprint, the lever magnitude of
  the first version, not its position (0.06 m above the footprint);
- the cube edge 0.11 m is an assumption; its effect on the F4c fits is at most
  0.003 of R2;
- the term of the cube wrench quadratic in the offset `o` is neglected (so the
  recovered `o` differs by about 1.3 cm from the truth);
- the S3 bound flag marks all six DOF when either matrix has a zero
  eigenvalue (a PSD constraint couples the DOF);
- the final fit stays on training only; the selection diagnostics enter the CV
  alone;
- the rotation uncertainty of the footprint (0.0024 rad) is not perturbed in
  the sensitivity check;
- the posture runs are excluded from the selection (base at rest, no
  excitation).

### Revision (maintainer, 2026-10-23)

Taken after the fixture of the decisions above was seen, still before any
suspension parameter was fitted to a real session. Each item is a test-design
or reporting change, with the earlier result kept beside the new one in
sections 3 to 9; config entries marked "revision".

- **F4a, test design.** Truth `h_e` 0.0985 m (the `base_link` height) instead
  of 0.25 m; sensitivity rows at 0.065 and 0.25 m. Pass rule: S2 within 0.05 of
  the ceiling and `abs(h_e - truth) <= 0.05` in >= 4 of 5 seeds; the old
  `s2_gain_min` (0.1) is removed and the gain is descriptive. Reason: the gain
  measures the lever of the truth, not the estimator (7.2).
- **F2, cause and bias.** The cause note is corrected (errors in variables of
  the marker noise, not the clock). Filter cutoff 10 -> 5 Hz and noise refrozen
  at the footprint. F2 recovery is kept and reported (still a fail, mean z
  only). New F2 bias (stiffness within 1 %, identified damping within 10 %) and
  the calibrated SE; the errors-in-variables correction is a sensitivity row,
  not the estimator.
- **F3 stays a reported fail.** F3b (mixed truth signature) added; the nested
  S1B2 and Q2 added (section 8); the additive-mix caveat is stated (7.1).
- **O6** near-duplicate rule, 3 marker sd: none found; README corrected.
- **O7** `Mz` scored under both conventions in part 2 (term 0.30 N m > 0.12 N m).
- **O8** 5 Hz cutoff; `k_z` cross-check against 1.7e6 N/m and the 10 to 45 Hz
  band at rest.
- **O9** "damping not identified" (at 0 or relative SE > 50 %) does not fail a
  DOF; a stiffness at its bound does.
- **O10** F4d mixed truth (S1 not enough >= 4, S1B2 gap <= 0.05 >= 4, Q2 present
  >= 4 of 5) and Q2 absent under the arm-only truth (F4c, >= 4 of 5).

Fixture result of the revision: F1, F2 exact, F2 bias, F2 gap, F3 attribution,
F3 full, F3b, F4a, F4b, F4c, F4c Q2, F4d and F5 pass; F2 recovery (mean z -0.30,
-0.45, -0.70) and F3 signature (0 of 5 seeds) are the two reported fails.
Deviations and assumptions to read with it:

- the F4a pass rule changed after a result was seen; the old rule passes only
  at the 0.25 m lever and is kept as the 0.25 row;
- the F2 bias limits (1 %, 10 %) were set after the bias was seen (z damping
  -9.6 % at level 1 is close to its limit);
- the errors-in-variables correction assumes the frozen noise levels are the
  regressor noise; it is not applied to the real fit;
- the mixed truth is additive (a stiff parallel arm path), a bound case;
- the S1B2 stiffnesses are confounded with the cube under the mix (up to 33 %);
- the 24.2 Hz `k_z` uses the static load as the moving mass, an upper bound of
  the mass above the springs.

## 11. Reproduction

```bash
conda activate figaroh-dev
cd figaroh-examples
export MPLBACKEND=Agg
python examples/tiago/suspension_model.py noise      # section 5 (frozen in the config)
python examples/tiago/suspension_model.py describe   # section 3, "Real reads"
python examples/tiago/suspension_model.py footprint  # section 3, base_footprint offset
python examples/tiago/suspension_truth.py --golden tests/golden/tiago_suspension_truth.json
python -m pytest tests/test_tiago_suspension_model.py tests/test_tiago_suspension_truth.py -q
```

The full fixture takes about 6 minutes.
