# TIAGo Pro mocap calibration: held-out study

Issue: [#97](https://github.com/thanhndv212/figaroh-examples/issues/97).
Delivery package: [C5 / core #167](https://github.com/thanhndv212/figaroh-plus/issues/167).

This brings the TIAGo Pro right-arm motion-capture study to the C5 standard
(the same one as TIAGo, [tiago-mocap-heldout-protocol.md](tiago-mocap-heldout-protocol.md)):
frozen data, a fixed split, an explicit estimation method, a registration
baseline, a simulated fixture with a noise estimate, an archived reference
run and a golden output. Everything below is reproduced by the scripts in
`examples/tiago_pro/` (commands in section 10). The checks are
`tests/test_tiago_pro_protocol.py`, `test_tiago_pro_heldout.py`,
`test_tiago_pro_calibration_truth.py` and `test_tiago_pro_reference_run.py`;
the golden values are `tests/golden/tiago_pro_mocap.json`.

## 1. Data and defects

Three Qualisys sessions of the right arm exist and are all the data that will
exist. Sensor, frame convention, columns and the full defect list are in
`examples/tiago_pro/data/README.md`; roles and sha256 are in
`examples/tiago_pro/data/protocol.yaml` (the harness refuses to run when a
file changes).

| Session | Rows | Measures | Role |
|---|---|---|---|
| 2026-08-05 12:46 | 48 | pose (x, y, z, roll, pitch, yaw) | training |
| 2026-07-01 09:13 | 44 | position (x, y, z) | validation |
| 2026-07-02 07:56 | 94 | position (x, y, z) | confirmation |

Defects that shape the study:

- Only 08-05 records orientation. The two July sessions are position only,
  and carry a `gripper_right_tool_mount_joint` column that is always 0.
- 07-01 and 07-02 repeat 14 postures (joint difference below 1e-3 rad), so
  the confirmation set is not independent of the validation set on those
  rows. Errors are reported by stratum (repeats validation / not).
- No held-out posture repeats a training posture. Six of 44 (07-01) and 20 of
  94 (07-02) held-out postures lie outside the training joint range.
- The mocap-to-robot frame moves between days: re-registering the 14 shared
  postures needs a shift of (-0.38, -2.34, -6.64) mm and a 0.30 deg
  rotation. The training frames cannot know it.
- The 08-05 rows are 48 of the 51 postures of `optimal_configs.yaml` (the
  D-optimal plan), in order; recorded joints differ from the commanded ones
  by 0.026-0.041 rad on every row.
- The sessions record different encoders of the series elastic arm: the July
  sessions the motor side, 08-05 the link side (section 8.5). The CSVs carry
  no column saying so.

## 2. Split choice

Training is 2026-08-05, validation 2026-07-01, confirmation 2026-07-02.
Issue #97 first proposed training on 07-02, validating on 07-01 and
confirming on 08-05. That split would train a 6D fit on position-only data
and confirm on the only session with orientation, so it was changed before
any fit: 08-05 is the only session that can train the 6D model. The price is
that the held-out error is a position (3D) error, and that held-out sessions
are earlier than the training session. Methods are selected on the validation
role only, never on the confirmation set (the harness raises if asked to).

## 3. Noise estimate

The 14 postures shared by 07-01 and 07-02 give a noise estimate after a rigid
registration of one session onto the other (6 degrees of freedom spent on the
transform): **1.35 mm** per axis for one measurement (0.30 deg rotation
removed). It is a substitute for a plateau-based estimate (the recordings
have no time stamps to cut plateaus from) and an upper bound: it also holds
the day-to-day repeatability of the robot reaching "the same" joint
configuration. The simulation grid is 0.5, 1.35 and 3.0 mm.

## 4. Outlier decision

Decided before fitting (`data_audit.py`; evidence in the data README). A
registration-only fit flags no row at 3 standard deviations. After the full
fit two rows pass 3 sigma, row 40 in position (14.4 mm) and row 15 in
orientation (4.9 deg). All rows keep at least 8.5% of their joint range from
the limits, the gap to the commanded posture is like every other row, and no
time stamp or marker-residual column exists, so occlusion or a marker swap
cannot be tested. Row 15 is close to a singularity (smallest singular value
of the tool Jacobian 0.033, second lowest of 48). **No row is dropped**:
dropping rows on residual size alone would tune the data to the model.

## 5. Models

All models are built from `full_params` (arm joints: 4 geometric parameters
per joint, plus the base frame in the mocap frame, 6D, and the tool frame,
6D). Same data, same config, one explicit `parameters.estimation.method`:

| Label | Estimation | Parameters (joint-level) |
|---|---|---|
| registration only | base and tool frame fitted, joints nominal | 12 (0) |
| full_params | structural (QR) selection | 35 (23) |
| full map | MAP with core's default priors | 60 (48) |
| full map_cv | MAP, prior scale by cross-validation | 60 (48) |
| full cv_subset | CV-selected parameter subset | 18 (6) |

Counts are the golden's. The structural and cv_subset selections break ties
by platform, so their counts can differ (the tests skip those values when the
count differs; CI runs Linux). The config default is `map_cv`, the model the
validation role selected.

**joint_offset is missing.** TIAGo also compares a `joint_offset` level
(14 parameters). It is not available here: `arm_right_6_joint` is a
`RevoluteUnaligned` joint that core's offset parameters cannot express
(`create_param_list` fails). This is core bug
[figaroh-plus#205](https://github.com/thanhndv212/figaroh-plus/issues/205);
the model stays dropped until it is fixed, so the "registration, joint_offset,
full_params" ladder of TIAGo has no middle rung here.

## 6. Held-out results

Position RMSE (mm) against the measured marker. Two numbers per set:
**re-registered** (primary: the trained joint parameters are kept and only
the base frame 6D and tool point 3D are refitted on the evaluated session)
and **transfer** (the training frames carried over). The mocap frame drifts
between days, so transfer largely measures that drift. Training RMS is the
fit's own error on 08-05 (position / orientation).

| Model | Training (mm / deg) | 07-01 validation: re-reg. | transfer | 07-02 confirmation: re-reg. | transfer |
|---|---|---|---|---|---|
| registration only | 19.78 / 3.09 | 28.56 | 38.31 | 28.06 | 33.52 |
| full_params | 6.69 / 1.81 | 20.49 | 26.68 | 17.50 | 22.17 |
| full map | 7.44 / 1.93 | 20.41 | 27.31 | 16.92 | 22.05 |
| full map_cv | 6.69 / 1.81 | **20.40** | 26.73 | 17.28 | 22.03 |
| full cv_subset | 7.47 / 1.87 | 20.76 | 27.13 | 17.71 | 22.33 |

Selection on the validation role (primary metric): **map_cv**. Its margin
over map (0.01 mm) and over full_params (0.09 mm) is not meaningful, and
cv_subset trails by 0.36 mm; the four fitted models are equivalent on these
data. What the data do support is that any joint-level calibration beats the
registration baseline by 8 mm (validation) and 11 mm (confirmation) on
re-registered error. The confirmation set agrees with the validation order
among the fitted models only loosely (map is best there by 0.4 mm), which is
expected from the same equivalence. Split by overlap, the confirmation error
on the 14 postures that repeat the validation set (about 17 mm) matches the
other 80 (about 17-18 mm).

The held-out error (17-20 mm) is about three times the training RMS (6.7
mm). The shipped `master_calibration_*.yaml` are training-session fits, not
validated corrections.

## 7. Simulation recovery

`calibration_truth.py` draws a `full_params` truth for the TIAGo Pro chain,
simulates pose measurements at the 48 training postures with the estimated
noise (0.5 / 1.35 / 3.0 mm, seeds 0-4), fits, and scores held-out position
error against the noise-free truth on the 07-01 and 07-02 postures. Seed 0,
noise 1.35 mm (golden): registration only 12.5 mm held-out RMSE, full_params
1.15 mm (the held-out error stays near the noise at every grid level: 0.48
mm at 0.5, 1.15 mm at 1.35, 2.52 mm at 3.0). In simulation the fitted
models generalise; the real data do not, which is the contrast the study
exists to show.

`--methods` compares the estimation methods on the same fixture. Reduced run
(truth full_params, noise 1.35 mm, seeds 0 and 1 only;
the full grid was not run):

| Fit | Parameters | Training RMS (mm) | Held-out RMSE (mm) |
|---|---|---|---|
| full structural | 35 | 1.34 | 1.68 |
| full map | 60 | 1.81 | 3.45 |
| full map priors x0.1 | 60 | 3.67 | 8.70 |
| full map priors x10 | 60 | 1.34 | 1.66 |
| full map_cv | 60 | 1.38 | 1.76 |
| full cv_subset | 32.5 | 1.38 | 1.84 |

Core's default priors (map) are too tight for the drawn truth, map_cv
recovers from that by choosing the prior scale, and structural selection is
best on a truth that the model class contains. The simulation therefore does
not predict which method wins on real data, where the truth is outside the
model class. The joint-level z-scores of fits with their own standard errors
are not usable as a calibration check here (425 z-scores, rms 47.6): the
standard errors of unidentifiable directions are far too small.

## 8. The train / held-out gap (diagnostics only)

Training RMS 6.7 mm against 17-20 mm re-registered on held-out sessions. The
following diagnostics look for the cause. **They are diagnostics only:
none is used for model selection and none enters the frozen validation or
confirmation selection** (`diagnostics.py`, separate from
`heldout_protocol.py`).

### 8.1 Overfitting: k-fold within the training session

Decision rule fixed before computing: if the selected model's (map_cv) k-fold
position RMSE is at most 1.5 times its training RMS, overfitting is not the
main cause of the gap. `data/protocol_0805_kfold.yaml`: k = 5, seed 0, labelled
"k-fold within session; weaker than held-out sessions". Each model is fitted
on four folds and scored on the left-out fold with its frames unchanged (same
session), pooled over folds.

| Model | Training RMS, all rows (mm) | Left-out fold position (mm) | Ratio | Left-out fold orientation (deg) |
|---|---|---|---|---|
| registration only | 19.78 | 21.02 | 1.06 | 3.21 |
| full_params | 6.68 | 8.70 | 1.30 | 2.10 |
| full map | 7.44 | 8.84 | 1.19 | 2.19 |
| full map_cv | 6.69 | 8.24 | 1.23 | 2.07 |
| full cv_subset | 7.47 | 8.17 | 1.09 | 2.05 |

The selected map_cv model is at 1.23 times its training RMS, below 1.5:
**overfitting is not the main cause**. Within the session the models carry
over to unseen postures at 8.2-8.8 mm, a third of the held-out session error.
(The older 36/12 split test of the superseded analysis, appendix A.2, gave
17-50 mm on 12 held-out rows; it used a different pipeline and is not
comparable. It is not reproduced here.)

### 8.2 One day apart: fit on 07-01, evaluate on 07-02

07-01 and 07-02 are one day apart and share 14 postures. Fitting each model on
07-01 (position only, 44 rows) and evaluating on 07-02 (94 rows) separates a
marker or frame move between days from robot changes over the month.

| Model | Parameters | Training RMS 07-01 | 07-02 re-reg. | 07-02 transfer | Shared 14, re-reg. | Other 80, re-reg. |
|---|---|---|---|---|---|---|
| registration only | 9 | 22.70 | 28.06 | 36.54 | 30.59 | 27.60 |
| full_params | 30 | 11.59 | 11.58 | 15.73 | 12.07 | 11.50 |
| full map | 57 | 12.55 | 12.42 | 16.15 | 13.18 | 12.28 |
| full map_cv | 57 | 11.77 | 11.76 | 15.70 | 12.05 | 11.71 |
| full cv_subset | 24 | 11.86 | 11.84 | 15.83 | 12.58 | 11.70 |

(mm). Fitted on 07-01, the models predict 07-02 re-registered as well as they
fit their own training data (map_cv 11.8 against 11.8), and the shared
postures are no easier than the others. Carrying the frames over costs about
4 mm (15.7 against 11.8): the frame move between the two days is real but
small. Two things stand out. The July-trained model's own training RMS (11.8
mm) is well above the 08-05 one (6.7 mm), although it has fewer residual
components to fit: the July data are harder to fit with this kinematic model,
whatever the model is trained on. And the 08-05-trained model on 07-02 (17.3
mm) is worse than the July-trained one (11.8 mm).

### 8.3 In-sample bias of the re-registered metric

The primary metric refits nine frame parameters on the evaluated session, so
it is optimistic. Split-half check (random halves of each held-out session,
seed 0): refit the base and tool frames on one half, score the other half,
and swap. map_cv:

| Session | In-sample re-reg. (all rows) | Other half | Increase |
|---|---|---|---|
| 07-01 (44) | 20.40 | 23.71 | +16% |
| 07-02 (94) | 17.28 | 18.53 | +7% |

(mm; pooled over both swaps. The other models give the same picture: 23.7-24.0
and 18.0-19.0.) The in-sample bias is real but small (a few mm at most on 44
rows). It does not explain a threefold gap.

### 8.4 What the gap is

Overfitting within the session (1.23x), the frame move between days (about 4
mm one day apart), and the in-sample bias of re-registration (7-16%) together
do not account for 17-20 mm against 6.7 mm. Most of the remainder is a change
of encoder between the sessions (section 8.5): July recorded the motor-side
joint angles, 08-05 the link-side ones, and the two differ by a per-joint
offset plus a spring deflection that changes with the posture. A model fitted
on link-side angles cannot be evaluated on motor-side angles as recorded.
Mapping the July joints to the link side brings map_cv to 12.6 mm on 07-01
and 7.2 mm on 07-02, against 8.2 mm on unseen postures of the training
session itself (8.1).

The remaining 07-01 excess (12.6 against 7.2 mm) is unexplained. It matches
the 07-01 to 07-02 picture of 8.2, where the July-trained model fits its own
July session at 11.8 mm. The frozen held-out numbers of section 6 stay as
they are: the correction uses a fit on the training session's joint data, and
it was found after the protocol was frozen. Fitting the springs from the
mocap instead (section 8.6) gives 11.5 and 5.2 mm.

### 8.5 Series elastic actuators: which encoder each session recorded

The TIAGo Pro arm joints are series elastic actuators: the transmission has
an elastic element, so the link angle differs from the motor angle by a
deflection that grows with the joint torque. PAL's hardware interface gives
each joint a motor-side `position` (motor encoder through the gear
reduction) and a separate `absolute_position`, and estimates the torque
from their difference. The recordings do not say which one was logged; the
joint data do.

**July: motor side.** 4 of the 44 postures of 07-01 and 8 of the 94 of 07-02
are postures of the commanded plan `optimal_configs.yaml`, and match the
commanded angles to within 1.6e-4 rad. That is what the motor-side angle,
which the position controller tracks, gives.

**08-05: link side.** None of the 48 postures matches its commanded posture
(median gap 0.027 rad). Per joint, commanded minus recorded is fitted by a
constant offset `c` plus a compliance `s` times the gravity torque of the
nominal URDF at the recorded posture (least squares on the joint data only,
no mocap):

| Joint | Offset c (mrad) | Compliance s (mrad/Nm) | Stiffness 1/s (Nm/rad) | Correlation with gravity torque | Residual RMS (mrad) | Peak torque (Nm) |
|---|---|---|---|---|---|---|
| arm_right_1 | +18.60 | 0.632 | 1581 | 0.999 | 0.36 | 28.3 |
| arm_right_2 | +21.80 | 0.666 | 1501 | 0.999 | 0.43 | 29.3 |
| arm_right_3 | +8.61 | 0.631 | 1585 | 0.992 | 0.53 | 13.6 |
| arm_right_4 | +5.96 | 0.662 | 1511 | 0.997 | 0.36 | 12.2 |
| arm_right_5 | +19.86 | 0.723 | 1384 | 0.876 | 0.49 | 2.2 |
| arm_right_6 | -22.72 | 1.450 | 690 | 0.994 | 0.20 | 2.3 |
| arm_right_7 | +26.61 | (offset only) | | | 0.40 | 0.007 |

One stiffness of about 1500 Nm/rad on joints 1-4 explains the difference
to half a milliradian: the deflection of a spring under gravity, read on the
link side, plus a constant offset between the two encoders of 6-27 mrad.
A controller tracking error would not scale with the gravity torque
this closely.

The values are effective, not the actuator's specification: they depend on
the URDF masses and leave out the marker plate on the tool. Joint 7 carries
almost no gravity torque, so only its offset is fitted.

**Mocap check.** The fit above uses no mocap, so the held-out sessions test
it independently. Each model is trained on 08-05 as recorded and evaluated
on the July sessions with their joints mapped to the link side,
q_link = q_motor - c - s * tau. Re-registered position RMSE (mm):

| Model | 07-01 as recorded | offsets only | deflection only | offsets + deflection | 07-02 as recorded | offsets only | deflection only | offsets + deflection |
|---|---|---|---|---|---|---|---|---|
| registration only | 28.56 | 23.65 | 27.45 | 21.58 | 28.06 | 22.42 | 26.68 | 20.44 |
| full_params | 20.49 | 16.71 | 18.18 | 12.60 | 17.50 | 12.01 | 14.88 | 7.15 |
| full map_cv | 20.40 | 16.78 | 18.02 | 12.61 | 17.28 | 12.10 | 14.60 | 7.24 |

Both parts are needed; together they remove about 40-60% of the held-out
error. The reverse mapping agrees. Moving 08-05 to the motor side
(q_motor = q_link + c + s * tau) and training on it raises the map_cv
training RMS from 6.69 to 11.12 mm (full_params 6.68 to 10.55), and gives
17.02 and 12.58 mm on the July sessions as recorded. The mocap fits the
link-side angles better, as it should: it measures where the link is.
Calibration data should record the link side, and a model trained on it
should be fed link-side angles.

### 8.6 Joint elasticity fitted from the mocap

Section 8.5 fits the compliance from the joint data. Core can also fit it
from the mocap: `parameters.include_non_geometric: true` adds one compliance
`k` per joint, and the forward model uses q + k * tau, tau the gravity
torque of the nominal URDF ([figaroh-plus#206](https://github.com/thanhndv212/figaroh-plus/issues/206),
fixed for this robot in figaroh-plus#207). `diagnostics.py elastic` fits
full_params with structural estimation (map_cv refuses these terms) with
and without `k` on arm joints 1-6; the torso is prismatic and joint 7
carries no gravity torque. "08-05 motor" is 08-05 with each row replaced by
its commanded posture, the motor-side angle the controller tracked.

| Training set | Model | Parameters | Training RMS (mm) | 07-01 re-reg. | 07-02 re-reg. |
|---|---|---|---|---|---|
| 08-05 link (as recorded) | rigid | 35 | 6.68 | 20.49 | 17.50 |
| 08-05 motor (commanded) | rigid | 35 | 10.52 | 16.53 | 12.01 |
| 08-05 motor (commanded) | **elastic** | 41 | 5.36 | **11.49** | **5.20** |
| 08-05 link (as recorded) | elastic | 41 | 5.36 | 18.97 | 16.32 |
| 07-01 motor, position only | rigid | 30 | 11.59 | (training) | 11.58 |
| 07-01 motor, position only | **elastic** | 36 | 3.76 | (training) | **5.36** |

(mm; held-out sessions as recorded, motor side.) Fitted on motor-side
angles, the elastic model takes 08-05 from 6.7 to 5.4 mm and the July
sessions from 20.5 / 17.5 to 11.5 / 5.2 mm, as well as or better than the
joint-data mapping of 8.5 (12.6 / 7.2 mm), with no joint data and no
commanded postures at evaluation. Trained on 07-01, the elastic model fits
3.8 mm and predicts 07-02 at 5.4 mm, against 11.6 mm for the rigid model:
the July data are not harder to fit (8.2) once the springs are in the model.

Compliance fitted, as `-k` in mrad/Nm (core's `k` is negative: Pinocchio's
torque is the holding torque, link = motor + k * tau):

| Joint | 08-05 motor | 08-05 link | Difference | Joint data `s` (8.5) | 07-01 motor |
|---|---|---|---|---|---|
| arm_right_1 | 1.197 | 0.572 | 0.625 | 0.632 | 1.245 |
| arm_right_2 | 0.647 | -0.016 | 0.663 | 0.666 | 0.748 |
| arm_right_3 | 1.823 | 1.189 | 0.634 | 0.631 | 2.171 |
| arm_right_4 | 0.699 | 0.039 | 0.660 | 0.662 | 0.929 |
| arm_right_5 | 0.589 | -0.108 | 0.697 | 0.723 | 0.762 |
| arm_right_6 | 4.146 | 2.508 | 1.638 | 1.450 | 0.321 |

The two elastic fits of 08-05, on motor-side and on link-side angles, reach
the same 5.36 mm: the encoder map is one more offset and compliance per
joint, which the joint offsets and `k` absorb, so the two fits differ by
the transmission. Their difference
recovers the joint-data compliance `s` to within 4% on joints 1-5 (13% on
joint 6) from the mocap alone, an independent check of 8.5. On joints 2, 4
and 5 the link-side `k` is near zero: the link encoder sees all the
compliance. On joints 1, 3 and 6 it is not (0.6, 1.2 and 2.5 mrad/Nm):
some compliance, or a gravity-torque error of the nominal masses, sits
beyond the link encoder. 07-01 gives similar values on joints 1-5 (within
about 30%); with position only it does not determine joint 6.

The 07-01 excess remains (11.5 mm against 5.2 mm on 07-02 from the same
08-05 fit). These fits are diagnostics, like 8.5: the elastic model is not
a protocol model, and its held-out numbers do not enter the selection of
section 6.

## 9. Limits

- One pose-measuring session (48 postures). Held-out error is position only.
- The validation and confirmation sets share 14 postures and are one day
  apart; the confirmation set is not a fully independent check.
- Held-out sessions precede the training session by about five weeks.
- The noise estimate (1.35 mm) is an upper bound from repeated postures, not
  from plateaus.
- The base and tool frames are mocap setup, not robot geometry; the PAL
  `master_calibration_*.yaml` files are exported from training-session fits
  by `run_calibration.py` (not a primary use) and carry no held-out claim.
- All models are rigid and trained on link-side joint angles (08-05). The
  July held-out sessions recorded the motor side; the frozen held-out
  numbers of section 6 use them as recorded. The encoder-side correction
  (section 8.5) is fitted on the training session's joint data and the
  nominal URDF masses, and is a diagnostic, not part of the protocol; so
  is the elastic model fitted from the mocap (section 8.6). Its stiffness
  values rest on the nominal URDF masses and no tool payload.
- joint_offset is not available ([figaroh-plus#205](https://github.com/thanhndv212/figaroh-plus/issues/205)).
- The differences among the four fitted models (0.01-0.4 mm) are below any
  meaningful resolution; the study does not rank them.

## 10. Reproduction

All commands run from `examples/tiago_pro` inside the `figaroh-dev` conda
environment with `MPLBACKEND=Agg`.

```bash
python heldout_protocol.py              # section 6, about 80 s
python calibration_truth.py             # section 7 grid, seeds 0-4
python calibration_truth.py --methods --seeds 0 1 --noise 1.35   # reduced
python diagnostics.py kfold             # 8.1, about 10 min
python diagnostics.py crossday          # 8.2
python diagnostics.py splithalf         # 8.3
python diagnostics.py encoder           # 8.5, about 1 min
python diagnostics.py elastic           # 8.6, about 30 s
python data_audit.py                    # noise estimate and outlier scan
python reference_run.py --no-plot       # fit, held-out report, archive
```

The reference run writes `results/runs/TIAGO-PRO/calibration/<time>_<rev>/`
(report.html, verdict.json, provenance.json, reproduction.json,
heldout.json, corrections.json, parameters.csv). From a dirty checkout the
archive check reports `revisions` as its only gap.

Tests: `pytest tests/test_tiago_pro_*.py` (default suite, cheap);
`-m slow` adds the held-out golden, the reference run, the 10-minute
k-fold check, the encoder-side correction and the elastic fit. Regenerate
the golden with
`python ../../tests/test_tiago_pro_heldout.py > ../../tests/golden/tiago_pro_mocap.json`.

## Appendix A. Earlier analyses folded into this note

These earlier notes are superseded by sections 1-8 and removed from `data/`.
Their numbers come from older pipelines on single sessions, with no held-out
check, and are kept as records.

### A.1 Offsets from the 2026-07-02 fit

From `data/calibration_results_20260702_0756.yaml` (94 samples, RMSE 6.46 mm):
deltas to ADD to the existing joint origins (x, y, z in m, roll, pitch, yaw
in rad). Where to apply them was never decided (the left/right arm sharing
problem was open). Not included: `base_*` (six parameters, mocap and torso
cannot be separated) and `pEEx_1`/`pEEy_1` (fixed at 0 by construction).
Axes not listed had no identifiable correction and stay unchanged. This fit
is position-only and its session is now the confirmation set, so these
deltas carry no held-out evidence.

| Joint | Deltas |
|---|---|
| arm_right_1 | roll -0.016283, pitch +0.013443, yaw +1e-8 |
| arm_right_2 | x +0.019881 m, z +3e-8 m, roll -0.009891, pitch -0.019360, yaw -0.042015 |
| arm_right_3 | x -0.004110 m, z -0.020218 m, roll +0.011929, pitch -0.006161, yaw -0.018246 |
| arm_right_4 | x +0.000076 m, z +0.000329 m, roll +0.003165, pitch +0.034312, yaw +0.009909 |
| arm_right_5 | x -0.006723 m, roll +0.006466, pitch +0.012907, yaw +0.041628 |
| arm_right_6 | x +0.000259 m, y +0.001029 m, roll +0.000689, pitch -0.006098, yaw -0.013188 |
| arm_right_7 | x +0.004952 m, y +5e-8 m, z +0.000944 m |
| gripper_right_tool_mount | x -0.002998 m, y +0.001139 m, tip z (pEEz_1, outside the URDF) -0.005066 m |

### A.2 Reliability of the 2026-08-05 parameters

Run on `calibration_results_20260805_1246.yaml` (48 samples, one outlier
flagged and not excluded, position RMSE 7.11 mm, orientation 1.92 deg), after
migrating `run_calibration.py` onto core's `BaseCalibration` (figaroh-plus
v0.4.7) with an SE3 log-map residual in place of the legacy script's
element-wise RPY residual. Standard errors come from `calc_stddev()`.
Verdict by standard score: solid at least 3 sigma, acceptable 2-3, marginal
1-2, noise below 1. **22 solid, 3 acceptable, 4 marginal, 9 noise of 38**
(the legacy residual gave 8, 4, 4, 22): the tighter residual, not more data,
sharpened the estimates (`pEEx_1` 1.67 to 15.1 sigma, `pEEy_1` 0.46 to 3.3).
Noise-level entries were mostly translations on joints 5-7
(`d_py_arm_right_5/6`, `d_pz_arm_right_6`, `d_pz_arm_right_7`, standard
error 12.6 mm, plus `d_phiy_arm_right_7`, `d_phiy_arm_right_2`,
`d_phiz_arm_right_2/3`, `d_px_arm_right_3`). 27 parameter pairs had
correlation above 0.8, several at exactly 1 (e.g. `d_phiy_arm_right_6` with
`d_phiz_arm_right_7`): genuinely degenerate directions in this regressor,
which more pose diversity (not more samples of the same plan) would fix.

Migration pitfall: core's `apply_measurement_weighting()` (1000 x position,
100 x orientation) together with an L2 term made `least_squares(method="lm")`
stop on `xtol` after 10 evaluations at a first-order optimality of 5.5e5,
because the column norms differed by about five orders of magnitude.
`cost_function()` in `run_calibration.py` therefore uses raw log-map
residuals, and regularisation is core's `estimation.method`, not appended
rows.

The 36/12 random split test (4 seeds) of that analysis gave held-out
position errors of 34.4 mm for a conservative subset (at least 2 sigma) and
33.5 mm for all 38 parameters (nominal about 139 mm): the "full is worse"
gap of the legacy residual (about 1.6 times) was gone. It is not comparable
with the k-fold of section 8.1 (8-9 mm), which refits with core's
estimation methods; the discrepancy was not investigated.

### A.3 Correcting the `tiago_endEffector` rigid body in QTM

The 08-05 fit estimated the tool frame relative to `tool_frame`
(`gripper_right_pal_atc_base_link`). To make the QTM rigid body report the
pose of `gripper_right_tool_holder` directly, the local origin correction
from that fit was:

| Parameter | Value | Standard error | Sigma |
|---|---|---|---|
| x (`pEEx_1`) | -4.672 mm | 0.309 mm | 15.1 |
| y (`pEEy_1`) | -1.032 mm | 0.311 mm | 3.3 |
| z (`pEEz_1`) | +128.776 mm | 1.000 mm | 128.8 |
| roll (`phiEEx_1`) | -1.100 deg | 0.017 deg | 63.8 |
| pitch (`phiEEy_1`) | -0.356 deg | 0.017 deg | 21.1 |
| yaw (`phiEEz_1`) | +1.942 deg | 1.023 deg | 1.9 (marginal) |

Correction applied to the rigid body's local axes: translation (4.786,
0.621, 13.137) mm; rotation `R = Rz(yaw) Ry(pitch) Rx(roll)` with RPY
(+1.0875, +0.3928, -1.9348) deg, quaternion (x, y, z, w) =
(0.009546, 0.003267, -0.016915, 0.999806). The z translation moved from
+4.76 mm in the earlier run to +13.14 mm (`pEEz_1` 137.16 to 128.78 mm) with
the pose subset and the corrected residual, not through noise. The correction
touches only the mocap body definition; the joint offsets of
`master_calibration_20260805*.yaml` are independent. It rests on 48 samples of
one session. If an earlier correction was applied in QTM, re-measure rather
than assume it still holds. The fit's frame parameters are mocap setup (the
mocap frame moves by several mm between days, section 1), which limits how
long such a correction stays valid.
