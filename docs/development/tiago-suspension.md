# TIAGo base suspension, Vicon and force plate (#70): pre-registration

Issue: [#70](https://github.com/thanhndv212/figaroh-examples/issues/70).
This note is part 1: the pre-registration, to be merged before any fit of a
suspension parameter to a real run. It is frozen once merged. The study
(real-data fits, comparison with the baselines, verdicts) is part 2 and changes
no threshold written here.

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
  find identical rows): duplicates, drops, latency (section 3);
- the base-marker triad of the nine other runs: rest pose, origin, frame yaw,
  peak-to-peak motion (section 3);
- the plate columns of `xyz_1640` and `postures_1720` for the plate convention
  `M = (COP - c) x F` (centre fit, residual only: it is a check of the data's
  convention, not of the suspension);
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
  output but no fit, score or statistic of them was computed.
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
rule (all 39 values equal) removes nothing; see O6. The latency level is only
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
verified from the data (O7). Moment transport to the origin `p_b` (rest
footprint point): `M_b = M_c + (c - p_b) x F`, then rotated to the footprint
axes. The status quo skips this and fits the
moments about the plate centre as if they were about the base.

**Filter and static offset.** Every channel is filtered with a zero-phase
4th-order Butterworth at 10 Hz (O8: the 24 Hz mode in z is out of band); the
velocity is the gradient of the filtered displacement on the frame grid. The
static value is the mean of the first second (the robot is at rest in every
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

Fit: constrained least squares (bounds for S1/S2/B2, PSD for S3) on unit-RMS
columns, rows weighted by the inverse standard deviation of each DOF's target.
Standard errors: moving-block bootstrap over the usable rows, 200 resamples,
5 s blocks (one oscillation), seed 7, same constraints. Selection: the simplest
of S1 < S2 < S3 within 5 % of the best 5-fold blocked CV score (10 s blocks),
pooled over training and the five selection diagnostics (blocks dealt to folds
per run); the final fit is on training only. The metrics are held-out R2 and
NRMSE per DOF, **with the mean of target and prediction removed** (a constant
is not a suspension quantity), the active-bound report (a parameter at its
bound; a PSD matrix with a zero eigenvalue flags all six DOF) and the
repeatability of the stiffnesses between a training and a validation refit
(refit on validation: part 2 only).

## 5. Noise stage

`python examples/tiago/suspension_model.py noise` (the `postures_1720` run, base
at rest). White levels from first differences (1.4826 MAD(diff)/sqrt 2); the
same levels pushed through the loader (Kabsch pose, filter, gradient) for the
displacement and velocity columns. The clock statistics are pooled over
`xyz_1640`, `xyz_1628` and `postures_1720`. The levels are frozen in the
configuration; grid levels 0.5, 1 and 2 times the white levels, used level 1,
level 0 the noise-free check (no white noise and no clock error).

| quantity | value |
|---|---|
| `noise marker` | 0.0111 mm per coordinate |
| `noise plate force` | Fx 0.32, Fy 0.28, Fz 0.93 N (unfiltered) |
| `noise plate moment` | Mx 0.22, My 0.27, Mz 0.12 N m (unfiltered) |
| `noise displacement` | x, y, z 2.7e-06 m; rx 2.2e-05, ry 1.6e-05, rz 1.3e-05 rad (filtered) |
| `noise velocity` | x, y, z 8.8e-05 m/s; rx 7.2e-04, ry 5.4e-04, rz 4.3e-04 rad/s (filtered) |
| `noise clock` | drops 1.05 %, latency 8.1 ms (sd 3.8, lag-1 0.73), duplicates 0 |

## 6. Identifiability (real motion, F1)

`python examples/tiago/suspension_truth.py` (F1 part). Scaled condition number
of the weighted design on the real training and validation motion (S2: worst
over the `h_e` grid); signal-to-noise = sd of the usable column over the noise
column of section 5. Criterion: scaled condition number below 1e3 for S1 and
S2 and 1e4 for S3. Passed.

| | training | validation |
|---|---|---|
| `condition s1` | 2.29 | 2.34 |
| `condition s2` | 2.69 | 2.43 |
| `condition s3` | 14.66 | 10.53 |

Signal-to-noise per DOF (x, y, z, rx, ry, rz), training:

| columns | x | y | z | rx | ry | rz |
|---|---|---|---|---|---|---|
| displacement | 130 | 61 | 16 | 34 | 23 | 304 |
| velocity | 18.1 | 14.1 | 4.4 | 6.1 | 4.0 | 56.1 |

Validation: displacement 137, 54, 17, 32, 14, 497; velocity 23.3, 13.4, 4.7,
6.5, 4.2, 60.2. No column is below the reported limit of 3 (the z-translation
columns, the issue's worry, have 16 and 4.4). The values are those of the
footprint origin and axes (the first version had the triad centroid in world
axes). These are the SNR of the
**regressors** against measurement noise; the fixture uses the measured noise
only. Damping is weaker than stiffness in every column: the velocity column is
`2 pi 0.2 = 1.26` times the displacement, so `c` contributes a few percent of
the wrench (section 7, F2: its SE is 1 to 7 % of the truth here, but a real
suspension need not behave like the fixture truth, O9).

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
were written in the configuration before the first run.

The ceiling of a DOF is the held-out R2 of the true parameters on the same
noisy regressors. A DOF is **answerable** if the mean fixture SE of its
stiffness over the truth is at most 50 %. All six are answerable (stiffness SE
0.04 to 0.3 % of the truth; damping SE 0.8 to 7 %: `context`).

### 7.1 F2 to F3: recovery and the status-quo signature

Results of the revised fixture (footprint origin, bounded fits, selection
diagnostics) next to the first version ("before"; same thresholds):

| criterion | pass rule | result (before) |
|---|---|---|
| F2 exact | level 0: relative parameter error below 1e-6 | pass, 5.8e-10 (8.4e-10) |
| F2 recovery | pooled z-RMS of the 12 parameters in [0.75, 1.33] and abs(mean z) below 0.3 at every level | **fail** (fail) |
| F2 ceiling | held-out R2 within 0.05 of the ceiling, answerable DOF, every level | pass (pass) |
| F3 signature | status quo + S1 reproduces forces R2 < 0.2, My R2 >= 0.75, >= 3 negative dampings, in >= 4 of 5 seeds | **fail** (fail) |
| F3 attribution | fixing one error raises the mean force R2 by 0.2 or more | pass, origin only (pass, origin only) |
| F3 full fix | all four fixes + filter within 0.05 of the ceiling, answerable DOF | pass (pass) |

| | 0.5 x noise | 1.0 x noise | 2.0 x noise |
|---|---|---|---|
| `recovery z rms` | 1.24 | 1.53 | 2.07 |
| `recovery z mean` | -0.66 | -1.01 | -1.55 |
| `recovery rel err` | 0.11 | 0.26 | 0.55 |
| `recovery gap` | 0.00 | 0.00 | 0.00 |

Before: z rms 1.33, 1.54, 1.95; z mean -0.58, -0.82, -1.19; rel err 0.14, 0.30,
0.62. F2 fails at every level, as before: the bootstrap SE is smaller than the
error and the errors have a common sign (negative mean z), growing with the
noise. The largest relative parameter error is 11, 26 and 55 % of the truth.
The held-out R2
is at the ceiling at every level (gap 0 to -3e-4), and exact when noiseless:
the error comes from the interaction of the clock reconstruction (interpolated
frames, the guard rows and the drop position off by a few rows) with the noise,
not from the model. No parameter is at a bound on the recovery fits (seed 0:
none on seed 0), so the constraints do not explain the z. It is reported as it is; the
limits are not moved, and part 2 reads the bootstrap SE as a lower bound (O4).

F3 at level 1, status quo (B1, unconstrained, unchanged) on the S1 truth, held
out on the simulated `xyz_1628`:

| quantity | value |
|---|---|
| `status quo forces r2` | 0.44 (seed 0: Fx 0.04, Fy 0.55, Fz 0.74) |
| `status quo my r2` | 0.98 (seed 0: Mx 0.87) |
| `status quo negative damping` | 2.8 of 6 DOF on average (2 to 4) |
| `signature seeds` | 0 of 5 |
| `ablation origin` | +0.50 |
| `ablation transport` | +0.00 |
| `ablation frame` | +0.02 |
| `ablation clock` | +0.03 |

Before: forces R2 0.58, My 0.97, negative dampings 2.8, ablations origin +0.39,
transport +0.00, frame +0.00, clock +0.01. The status quo is now nearer the
"only My" signature (forces R2 0.44 instead of 0.58, My 0.98) but still does
**not** reproduce it under an S1-diagonal truth: forces R2 stay at 0.44
(limit 0.2) and 2.8 dampings are negative (limit 3); 0 of 5 seeds pass.
The corner-marker origin is the one error that moves the force fit (+0.50 of
mean force R2, from 0.44 to 0.95); the moment transport and the frame change
the **moment** rows (frame +0.02 of force R2), which this metric hardly sees,
and the clock error costs 0.03. After all four fixes the S1 fit is at the
ceiling (F3 full gap -2e-05). So a spring-damper truth makes the status quo
look good on the forces; the reported "only My" is not explained by the four
loader errors alone, see F4c below.

### 7.2 F4: misspecified truths

| criterion | pass rule | result (before) |
|---|---|---|
| F4a elastic centre | S2 beats S1 by 0.1 of mean held-out R2 and is within 0.05 of the ceiling | **pass**, gain 0.17, gap 0.00 (fail, gain 0.03) |
| F4b nonlinear | hardening and deadband flagged in >= 4 of 5 seeds, 0 flags under the linear truth | pass (5 of 5 and 5 of 5, 0) (same) |
| F4c arm load | "enough" false in >= 4 of 5 seeds under the arm-only truth | pass (5 of 5) (same) |

F4a: truth with an elastic centre at `h_e` = +0.25 m **above the footprint**.
Mean held-out R2 over the six DOF: S1 0.82, S2 and S3 0.99 to 1.00 (seed 0:
S1 0.82, S2 0.99). The gain of S2 over S1 is **0.17** (`elastic gain`, limit
0.1), and S2 reaches the ceiling and recovers `h_e` (F5). **This pass comes
from the change of truth, not of the fit:** the first version put the elastic
centre 0.25 m below the triad centroid, which is 0.06 m above the footprint,
a lever too short for S1 to miss (gain 0.03). The maintainer's origin moves
the reference to the footprint, and the lever of 25 cm is kept. Rerun with the
old position (0.06 m above the footprint) the gain is 0.010 (0.009 to 0.011 over
the five seeds), a fail; the fixture's F4a verdict therefore depends on how far
the true elastic centre is from the chosen origin, and the pass is not a
property of the estimator alone. The worst DOF of S1 is Fy (R2 0.006 against
0.98), the lever of the `theta x r` coupling.

F4b: the nonlinearity flag (|correlation| of the S1 residual with the cubic
part of the displacement, above 0.2) fires on the hardening truth (x, y, z) and
the deadband truth (x, y) in every seed (corr 0.92 to 1.0 on the hardening DOF,
0.70 to 0.83 on the deadband DOF) and never on the linear truth (largest |corr|
0.11, on a moment DOF).

F4c: the truth has **no suspension**: the real base motion and a wrench of the
B2 rigid cube of 25 kg, its centre offset from the gripper-marker frame by
(0.04, -0.03, 0.05) m (the wrench path the spring-damper cannot see). Mean
held-out R2 over seeds (x, y, z, rx, ry, rz):

| model | Fx | Fy | Fz | Mx | My | Mz |
|---|---|---|---|---|---|---|
| S1 `arm only s1` | 0.07 | 0.00 | 0.02 | 0.86 | 0.60 | 0.25 |
| B2 `arm only b2` | 0.92 | 0.90 | 0.95 | 0.98 | 0.99 | 0.88 |

Before (point mass at the marker centroid): S1 0.22, 0.08, 0.08, 0.54, 0.88,
0.25; B2 0.91, 0.92, 0.96, 0.99, 0.99, 0.89. B2 recovers a mass of 23.5 kg
(truth 25) and a first-moment offset (0.053, -0.037, 0.037) m (truth (0.04,
-0.03, 0.05); the term quadratic in the offset, neglected by the linear fit,
and the marker noise account for the difference). The rule of section 8 says
"not enough" in 5 of 5 seeds. In every seed one DOF of the S1 fit has a
parameter at its bound (the active-bound item; seed 0: Fy); B2's `m` is never
at its bound. The "only My" signature of #70 (forces R2 below 0.25, `My` 0.88)
is reproduced less sharply than before: forces R2 0.00 to 0.07 and `My` 0.60,
with `Mx` 0.86. A truth that is **arm load and no suspension** still gives low
force R2 for a spring-damper, the four loader errors do not (F3); that does not
say that the real wrench is an arm load, only that B2 is a necessary baseline.
Effect of the assumed inertia (B2 refitted with the edge 0 and doubled): the
held-out R2 of any DOF changes by at most 0.003 (`inertia_max_dr2`) over the
five seeds, so the assumed edge does not matter at the fixture's
accelerations.

### 7.3 F5: selection

| criterion | pass rule | result (before) |
|---|---|---|
| F5 selection | S1 chosen under the S1 truth and S2 with `abs(h_e - truth) <= 0.05` under the elastic truth, each in >= 4 of 5 seeds | pass (pass) |

| | result |
|---|---|
| `selection s1` | 5 of 5 |
| `selection s2` | 5 of 5 (`h_e` = 0.25 m above the footprint in every seed, on the grid 0 to 0.60) |

The CV is now pooled over training and the five selection diagnostics, and
the S1-truth CV picks `h_e` = 0 (the grid floor, S2 reduces to S1) with S1
within 5 % of the best (seed 0: CV score 0.084 for S1 and S2, 0.261 and 0.073
for S1 and S2 under the elastic truth).

### 7.4 Origin sensitivity and B2 edge (descriptive, no threshold)

`origin sensitivity` (the S1 truth, level 1, five seeds): the truth is
measured about the nominal footprint, and the loader is given a moved origin;
the table is the mean over seeds of the largest relative change of the 12 S1
parameters and the change of mean held-out R2, against the nominal loader
(nominal R2 0.994).

| origin | max relative change of a parameter | change of mean R2 (worst seed) |
|---|---|---|
| footprint +1 sd x | 3.2 % | -0.0003 (-0.0003) |
| footprint -1 sd x | 3.1 % | -0.0002 (-0.0002) |
| footprint +1 sd y | 1.6 % | -0.0022 (-0.0023) |
| footprint -1 sd y | 1.6 % | -0.0023 (-0.0025) |
| footprint +1 sd z | 0.6 % | -0.0000 (-0.0000) |
| footprint -1 sd z | 0.6 % | -0.0000 (-0.0000) |
| triad centroid (ablation) | 100 % (a damping; mean 32 %) | -0.193 (-0.197) |

The footprint offset uncertainty (2 mm, 3 mm, 2 mm) moves a stiffness or
damping by up to 3 % and the R2 by less than 0.003; the triad centroid, 31 cm
above the footprint, moves a damping by 100 % and the R2 by 0.19. The origin
matters at the lever of the base, not at the survey's uncertainty. The
rotation uncertainty (0.0024 rad) is not perturbed.

## 8. Verdict rules (written before the real fit)

Applied to the validation run (part 2), with the model frozen on training
(selection of section 4). A DOF is:

- **explained** if its held-out R2 on validation is at least 0.5 (`explained_r2`);
- **beating B2** if the selected model's R2 exceeds B2's by at least 0.1;
- **repeatable** if its stiffness differs by at most `band` between the
  training fit and the validation refit. `band` is 2 times the RMS relative
  difference of the stiffnesses between the fixture's training and validation
  fits (`k_rel_diff_rms` 0.0039, `band` 0.0077) -- so tight that any real
  session-to-session change will exceed it, which is read as "not repeatable
  to the noise" (O4);
- free of an **active bound**: no parameter of the DOF (`k`, `c`) at its bound
  of 0, no zero eigenvalue of the S3 `K`, `C` (all six DOF then), and B2's `m`
  not at 0 (the comparison with B2 would be vacuous);
- free of the **nonlinearity flag** (correlation above 0.2).

The **spring-damper is enough** iff on every answerable DOF the DOF is
explained, beats B2, is repeatable, has no active bound and no flag. Then
the confirmation runs must pass the same rule with the frozen model.
Meanings of a failure:

- explained on `My` only, and B2 matches it: the moment fit is the arm's
  gravity load, not a suspension estimate;
- explained but not beating B2: the suspension adds nothing the arm motion does
  not;
- active bound: the unconstrained fit wants a non-dissipative or negative
  stiffness (the arm's inertial coupling, a wrong origin); the constrained fit
  sits on the bound and the DOF is not a spring-damper estimate;
- not repeatable: parameters are session properties, not constants;
- flag: the spring is nonlinear on that DOF.

Not answerable: a DOF whose stiffness SE in the fixture is above 50 % of the
truth (none in the fixture); the DOF is reported but does not enter "enough".

## 9. Design of part 2 (not run)

1. Load `xyz_1640` and `xyz_1628` through `suspension_model.read_regressors`;
   read the wrenches with `read_target` (training; validation with
   `unlock_validation` in the study script only).
2. B0, B1, B2 (cube) and the constrained S1/S2/S3 on training; selection by
   blocked CV over training and the selection diagnostics; bootstrap SE.
3. Score the frozen model on validation; refit on validation for repeatability.
4. Apply section 8; confirmation runs only if validation says "enough", read
   once, with the frozen model.
5. Selection diagnostics (O3) join training in the CV of step 2 (wrench read
   with `read_target`, which allows them and never the confirmation runs); the
   posture runs stay description only. An active bound (O4) is reported and
   fails the DOF.

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
- **O4 fixture failures** (not relaxed): F2 recovery (z-RMS 1.24 to 2.07, mean
  z -0.66 to -1.55: the bootstrap SE is an underestimate), F3 signature (the
  status quo does not reproduce "only My" under a spring truth). F4a now passes
  (gain 0.17) but only because the truth lever was kept at 25 cm above the
  footprint; at 6 cm above it (the old truth position) the gain is 0.010
  (section 7.2). Consequences: part 2 reads the bootstrap SE as a lower bound,
  the repeatability band (0.0077) stays so tight that any real session-to-
  session change exceeds it, and the S1/S2 gain of the real run has no
  fixture band.
- **O5 "with and without weight"** (closed, out of scope). The OptiTrack
  sessions of the plan (#112) have no force-plate data; this study uses only
  the Vicon day.
- **O6 clock.** The data README reports 1.5 to 2.5 % dropped and 0.5 to 0.8 %
  duplicated frames. The rebuild finds 0.9 to 1.6 % dropped and no exact
  duplicate. A near-duplicate rule (rows equal within the marker noise) was not
  tried: if the README's duplicates are near-identical rows they are still in
  the data, as extra rows with a latency of one frame.
- **O7 `Mz`.** Whether `Mz` is about the plate centre or the COP cannot be
  checked (it is the free moment); it is taken as about `c`.
- **O8 bandwidth.** The 10 Hz cutoff keeps the 0.2 Hz motion and excludes the
  24 Hz vertical mode seen in the wrench; whether that mode is the suspension
  resonance is a question for part 2.
- **O9 damping.** The fixture's dampings (damping ratio about 0.1) are
  identifiable here; real damping at 0.2 Hz may not be (the velocity is
  `2 pi f` times smaller than the displacement). The verdict is on the whole
  wrench, not on the dampings alone.
- **O10 F4c definition.** "Arm load" is taken as the **only** wrench path
  (no suspension), the purest case of a path the spring-damper cannot see. A
  mixed truth (suspension plus arm load) may be judged "enough" by the rule.
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

Fixture result of these decisions (sections 7.1 to 7.4): F1, F2 exact, F2 gap,
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

The full fixture takes about 4.5 minutes.
