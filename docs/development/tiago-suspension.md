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

Diagnostic runs are used for description only (open point O3).

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

**Base pose.** Origin: the centroid of the three base markers at rest (the
mean of the first second), the one point a pure rotation of the triad leaves
fixed (O1: no surveyed `base_link` point exists). Rotation: Kabsch fit of all
three markers to the rest triad, as a world-frame rotation vector. Translation
and rotation are expressed in Vicon **world** axes. The status-quo loader
takes marker `base1` (a corner) as the origin, builds the frame
`x = b2 - b1`, `z = x x (b3 - b1)` and reports RPY relative to the first row.
Measured on the nine runs: the rest centroid is at (0.905, 0.453, 0.310) m
(between runs: 0.4 mm in x, 7 mm in y, 0.3 mm in z); the triad frame, taken z-up, is yawed
-5.3 to -6.0 degrees from world about z (-5.54 on `xyz_1640`, -5.33 on
`xyz_1628`) and its normal `(b2 - b1) x (b3 - b1)` **points down** (confirming
the issue): the status quo mixes world translations with a down-pointing
frame's angles.

Peak-to-peak motion of the origin (mm) and rotation vector (mrad), usable
rows:

| run | x | y | z | rx | ry | rz |
|---|---|---|---|---|---|---|
| `xyz_1640` | 5.06 | 3.48 | 0.31 | 1.98 | 5.62 | 46.79 |
| `xyz_1628` | 4.37 | 3.75 | 0.35 | 1.65 | 5.26 | 39.49 |

Base motion is at the oscillation frequency, 0.2 Hz, with power below about
3.5 Hz in x and y.

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
centroid): `M_b = M_c + (c - p_b) x F`. The status quo skips this and fits the
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

- **S1** `w = K x + C v`, `K`, `C` diagonal: 12 parameters.
- **S2** the same about an **elastic centre** at `(0, 0, h_e)` in the base
  frame: `w = A diag(k) B x + A diag(c) B v`, `B` maps the displacement to the
  centre (`x + theta x r`), `A = B^T` maps back to the origin. `h_e` on the
  grid -0.50 to 0.30 m, step 0.01, by blocked CV on training only. 12 + `h_e`.
- **S3** full symmetric `K` and `C`: 21 + 21 parameters.
- Each with one constant per DOF (static drift), not reported.
- **B0** static: predicts the static wrench (R2 = 0 by construction).
- **B1** the status quo: `status_quo_from_arrays` and the plain 12-parameter
  least squares of `utils/suspension_model.py` (no offset, no weights).
- **B2** rigid base: the arm load alone. Gravity and inertial wrench of one
  effective point mass at the gripper-marker centroid moving with the measured
  arm motion (`m (g - a)`, moment about the origin); the parameter is the mass.
  The competitor that tests whether the suspension explains more than "the arm
  moved" (O2).

Fit: linear least squares on unit-RMS columns, rows weighted by the inverse
standard deviation of each DOF's target. Standard errors: moving-block
bootstrap over the usable rows, 200 resamples, 5 s blocks (one oscillation),
seed 7. Selection (training only): the simplest of S1 < S2 < S3 within 5 % of
the best 5-fold blocked CV score (10 s blocks). The metrics are held-out R2 and
NRMSE per DOF, **with the mean of target and prediction removed** (a constant
is not a suspension quantity), negative-damping flags (`c < -2 SE`) and the
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
| `condition s1` | 2.22 | 2.34 |
| `condition s2` | 2.35 | 2.34 |
| `condition s3` | 35.26 | 47.22 |

Signal-to-noise per DOF (x, y, z, rx, ry, rz), training:

| columns | x | y | z | rx | ry | rz |
|---|---|---|---|---|---|---|
| displacement | 145 | 184 | 13 | 17 | 45 | 304 |
| velocity | 25.8 | 22.7 | 3.9 | 3.1 | 7.8 | 56.1 |

Validation: displacement 210, 186, 16, 10, 42, 497; velocity 30.0, 25.0, 4.4,
3.2, 8.5, 60.2. No column is below the reported limit of 3 (the z-translation
columns, the issue's worry, have 13 and 3.9). These are the SNR of the
**regressors** against measurement noise; the fixture uses the measured noise
only. Damping is weaker than stiffness in every column: the velocity column is
`2 pi 0.2 = 1.26` times the displacement, so `c` contributes a few percent of
the wrench (section 7, F2: its SE is 1 to 9 % of the truth here, but a real
suspension need not behave like the fixture truth, O9).

## 7. Pre-registered thresholds (simulated)

Fixture `suspension_truth.py`, seeds 0 to 4. The truth: `k` = (1e4, 1e4, 1e5,
3e4, 2e4, 1e3) N/m (x, y, z) and N m/rad (rx, ry, rz), `c` = (150, 150, 500,
100, 80, 12), each parameter times `1 +- 25 %` per seed. The values are the
order of magnitude of the ratio of the standard deviations of the plate wrench
and the base displacement, descriptive and not a fit. The base motion is the
**real** corrected pose of `xyz_1640` (fit) and `xyz_1628` (held out), held at
rest in the first second; the arm marker columns are the real ones; the three
base markers, the plate (static -730 N, moments about the plate centre) and the
receive clock (measured drops and latency) are simulated from the truth, with
noise at the used level. Truth streams: `default_rng(seed)`; noise
`default_rng(10000 + 100 k + seed)`, `k` = 0 white, 1 clock. The criteria
were written in the configuration before the first run.

The ceiling of a DOF is the held-out R2 of the true parameters on the same
noisy regressors. A DOF is **answerable** if the mean fixture SE of its
stiffness over the truth is at most 50 %. All six are answerable (stiffness SE
0.04 to 0.4 % of the truth; damping SE 0.8 to 9 %: `context`).

### 7.1 F2 to F3: recovery and the status-quo signature

| criterion | pass rule | result |
|---|---|---|
| F2 exact | level 0: relative parameter error below 1e-6 | pass (8.4e-10) |
| F2 recovery | pooled z-RMS of the 12 parameters in [0.75, 1.33] and abs(mean z) below 0.3 at every level | **fail** |
| F2 ceiling | held-out R2 within 0.05 of the ceiling, answerable DOF, every level | pass |
| F3 signature | status quo + S1 reproduces forces R2 < 0.2, My R2 >= 0.75, >= 3 negative dampings, in >= 4 of 5 seeds | **fail** |
| F3 attribution | fixing one error raises the mean force R2 by 0.2 or more | pass (origin only) |
| F3 full fix | all four fixes + filter within 0.05 of the ceiling, answerable DOF | pass |

| | 0.5 x noise | 1.0 x noise | 2.0 x noise |
|---|---|---|---|
| `recovery z rms` | 1.33 | 1.54 | 1.95 |
| `recovery z mean` | -0.58 | -0.82 | -1.19 |
| `recovery rel err` | 0.14 | 0.30 | 0.62 |
| `recovery gap` | 0.00 | 0.00 | 0.00 |

F2 fails at every level: the bootstrap SE is smaller than the error and the
errors have a common sign (negative mean z), growing with the noise. The
largest relative parameter error is on a damping (14, 30 and 62 % of the
truth); the stiffnesses are recovered to 0.1 to 0.4 %. The held-out R2 is at
the ceiling at every level, and exact when noiseless: the error comes from the
interaction of the clock reconstruction (interpolated frames, the guard rows
and the drop position off by a few rows) with the noise, not from the model.
It is reported as it is; the limits are not moved, and part 2 reads the
bootstrap SE as a lower bound (O4).

F3 at level 1, status quo (B1) on the S1 truth, held out on the simulated
`xyz_1628`:

| quantity | value |
|---|---|
| `status quo forces r2` | 0.58 (seed 0: Fx 0.30, Fy 0.70, Fz 0.73) |
| `status quo my r2` | 0.97 (seed 0: Mx 0.87) |
| `status quo negative damping` | 2.8 of 6 DOF on average (2 to 3) |
| `signature seeds` | 0 of 5 |
| `ablation origin` | +0.39 |
| `ablation transport` | +0.00 |
| `ablation frame` | +0.00 |
| `ablation clock` | +0.01 |

The status quo does **not** reproduce the "only My" signature under an
S1-diagonal truth: forces R2 stay at 0.58 (limit 0.2) and only 2.8 dampings are
negative (limit 3). The corner-marker origin is the one error that moves the
force fit (+0.39 of mean force R2, from 0.58 to 0.97); the moment transport and
the frame do not change the **force** R2 (they change the moment rows, which
this metric does not see) and the clock error costs 0.01. After all four fixes
the S1 fit is at the ceiling (F3 full gap -3e-06). So a spring-damper truth
makes the status quo look good on the forces; the reported "only My" is not
explained by the four loader errors alone, see F4c below.

### 7.2 F4: misspecified truths

| criterion | pass rule | result |
|---|---|---|
| F4a elastic centre | S2 beats S1 by 0.1 of mean held-out R2 and is within 0.05 of the ceiling | **fail** (gain 0.03, gap 0.00) |
| F4b nonlinear | hardening and deadband flagged in >= 4 of 5 seeds, 0 flags under the linear truth | pass (5 of 5 and 5 of 5, 0) |
| F4c arm load | "enough" false in >= 4 of 5 seeds under the arm-only truth | pass (5 of 5) |

F4a: truth with an elastic centre at `h_e = -0.25` m. Mean held-out R2 over the
six DOF: S1 0.97, S2 and S3 0.99 to 1.00 (seed 0). The gain of S2 over S1 is
**0.03** (`elastic gain`, limit 0.1): the 25 cm lever of the fixture moves the S1 fit by little
(the worst DOF, Fx, has 0.90 against 1.00), not because the elastic centre is
invisible: S2 reaches the ceiling and recovers `h_e` (F5).

F4b: the nonlinearity flag (|correlation| of the S1 residual with the cubic
part of the displacement, above 0.2) fires on the hardening truth (x, y, z) and
the deadband truth (x, y) in every seed (corr up to 1.0 on x and 0.7 to 0.86 on
the deadband DOF) and never on the linear truth (largest |corr| 0.12, on a
moment DOF).

F4c: the truth has **no suspension**: the real base motion and a wrench of an
effective 25 kg point mass at the gripper cluster (the wrench path the
spring-damper cannot see). Mean held-out R2 over seeds (x, y, z, rx, ry, rz):

| model | Fx | Fy | Fz | Mx | My | Mz |
|---|---|---|---|---|---|---|
| S1 `arm only s1` | 0.22 | 0.08 | 0.08 | 0.54 | 0.88 | 0.25 |
| B2 `arm only b2` | 0.91 | 0.92 | 0.96 | 0.99 | 0.99 | 0.89 |

B2 recovers a mass of 23.5 kg (truth 25). The rule of section 8 says "not
enough" in 5 of 5 seeds (S1 cannot beat B2 by 0.1, nor reach 0.5 on the forces).
This is the "only My" signature of #70: forces R2 below 0.25, `My` 0.88. A
truth that is **arm load and no suspension** reproduces what the status quo
reported on the real run, the four loader errors do not (F3); that does not say
that the real wrench is an arm load (O5 of the plan is not imported), only that
B2 is a necessary baseline.

### 7.3 F5: selection

| criterion | pass rule | result |
|---|---|---|
| F5 selection | S1 chosen under the S1 truth and S2 with `abs(h_e - truth) <= 0.05` under the elastic truth, each in >= 4 of 5 seeds | pass |

| | result |
|---|---|
| `selection s1` | 5 of 5 |
| `selection s2` | 5 of 5 (`h_e` = -0.25 m in every seed, from the CV grid minimum) |

## 8. Verdict rules (written before the real fit)

Applied to the validation run (part 2), with the model frozen on training
(selection of section 4). A DOF is:

- **explained** if its held-out R2 on validation is at least 0.5 (`explained_r2`);
- **beating B2** if the selected model's R2 exceeds B2's by at least 0.1;
- **repeatable** if its stiffness differs by at most `band` between the
  training fit and the validation refit. `band` is 2 times the RMS relative
  difference of the stiffnesses between the fixture's training and validation
  fits (`k_rel_diff_rms` 0.0036, `band` 0.0072) -- so tight that any real
  session-to-session change will exceed it, which is read as "not repeatable
  to the noise" (O4);
- free of **negative damping**: `c < -2 SE`;
- free of the **nonlinearity flag** (correlation above 0.2).

The **spring-damper is enough** iff on every answerable DOF the DOF is
explained, beats B2, is repeatable, has no negative damping and no flag. Then
the confirmation runs must pass the same rule with the frozen model.
Meanings of a failure:

- explained on `My` only, and B2 matches it: the moment fit is the arm's
  gravity load, not a suspension estimate;
- explained but not beating B2: the suspension adds nothing the arm motion does
  not;
- negative damping: the model absorbs a non-dissipative effect (the arm's
  inertial coupling, a wrong origin);
- not repeatable: parameters are session properties, not constants;
- flag: the spring is nonlinear on that DOF.

Not answerable: a DOF whose stiffness SE in the fixture is above 50 % of the
truth (none in the fixture); the DOF is reported but does not enter "enough".

## 9. Design of part 2 (not run)

1. Load `xyz_1640` and `xyz_1628` through `suspension_model.read_regressors`;
   read the wrenches with `read_target` (training; validation with
   `unlock_validation` in the study script only).
2. B0, B1, B2 and S1/S2/S3 on training; selection by blocked CV; bootstrap SE.
3. Score the frozen model on validation; refit on validation for repeatability.
4. Apply section 8; confirmation runs only if validation says "enough", read
   once, with the frozen model.
5. Diagnostics (O3) as description only.

## 10. Open points for the maintainer

- **O1 base origin.** The triad centroid is the origin (the only point a
  rotation of the triad leaves fixed). No survey of `base_link` against the
  markers exists. A surveyed point changes the translation columns by
  `theta x r` (S2 absorbs part of it through `h_e`).
- **O2 B2 definition.** One point mass at the gripper-marker centroid (the
  shoulder cluster is available, `B2gs`). It is the minimal arm-load
  competitor; a link-by-link model needs robot channels that are not recorded.
- **O3 diagnostics.** The `around_*`, `*_fold` and `postures` runs are used for
  description only (noise floor, clock). Using them for selection would add
  excitation (rotation about each axis) and is not done without a decision.
- **O4 fixture failures** (not relaxed): F2 recovery (z-RMS 1.33 to 1.95, mean
  z -0.58 to -1.19: the bootstrap SE is an underestimate, mostly for the
  dampings), F3 signature (the status quo does not reproduce "only My" under a
  spring truth), F4a (S2 gains 0.03, not 0.1, under a 25 cm elastic centre).
  Consequences: part 2 reads the bootstrap SE as a lower bound, the
  negative-damping rule is read with that in mind, and the S1/S2 gain of the
  real run has no fixture band.
- **O5 "with and without weight".** The OptiTrack sessions of the plan (#112)
  are not imported; this study uses only the Vicon day.
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
- **O11 plate-frame convention.** The wrench is expressed in world axes about
  the origin; `frame: base` (axes of the triad) is implemented and is a
  yaw of -5.4 degrees away.

## 11. Reproduction

```bash
conda activate figaroh-dev
cd figaroh-examples
export MPLBACKEND=Agg
python examples/tiago/suspension_model.py noise      # section 5 (frozen in the config)
python examples/tiago/suspension_model.py describe   # section 3, "Real reads"
python examples/tiago/suspension_truth.py --golden tests/golden/tiago_suspension_truth.json
python -m pytest tests/test_tiago_suspension_model.py tests/test_tiago_suspension_truth.py -q
```

The full fixture takes about 1.5 minutes.
