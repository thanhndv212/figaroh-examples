# TIAGo mocap across hardware (#100): pre-registration and results

Issue: [#100](https://github.com/thanhndv212/figaroh-examples/issues/100).
Part 1 (sections 1-9) is the pre-registration, merged before any real
joint-level fit; it is frozen. Part 2 (sections 10-16) is the study: real-data
fits, square motion, verdicts. Part 2 changes no threshold of part 1.

Question: when the same TIAGo arm is calibrated from different mocap systems
(Qualisys 2021, OptiTrack and Vicon 2023), how much of the difference between
the fitted models is explained by the measurement hardware alone? This note
fixes the model, the metrics and the thresholds **before any joint-level model
is fitted on the 2023 data**, so the real differences are read against bands
that were written down from simulation only.

## 1. What this part contains, and what it does not

- Protocol v2 of the OptiTrack and Vicon sessions (roles, section 2).
- The unified configuration `examples/tiago/config/tiago_mocap_hardware_config.yaml`.
- The library `examples/tiago/mocap_hardware.py` (re-registration, leave-one-out,
  effective noise) and the fixture `examples/tiago/mocap_hardware_truth.py`.
- The thresholds below. They come from the fixture output, which fits only
  **simulated** measurements.

No joint-level fit of a real 2023 session is run in this part. The only real
data used are the registration-only estimates of the effective noise
(section 4), which fit a rigid transform (and a tool point) and no joint offset.
`mocap_hardware.py` has a `noise` stage only; there is no real or square stage.

Disclosure. While prototyping, a throw-away script fitted the full structural
`joint_offset` model once on OptiTrack 1658, Vicon 1508 and the Qualisys
training session, to see that the three fits keep the same 14 parameters. The
script was deleted, nothing from it is stored or used, and no design choice
depended on it (the choices follow the architect plan of 2026-10-20). It is
listed here because it departs from the rule above.

## 2. Protocol v2

v1 (#105, #106) deferred the held-out split to this study; its
"confirmation" role meant only "not yet looked at". v2 assigns roles by a rule
stated before any fit: **training is the largest usable run of a definition,
which leaves the most new postures for confirmation.**

| System | Training | Confirmation | Not evaluated |
|---|---|---|---|
| OptiTrack | 2023-09-12 16:58 (13) | 2023-09-12 15:52 (9) | 15:29 (4 < 6 postures) |
| Vicon | 2023-09-20 15:08 (17) | 2023-09-20 16:07 (21) | none |
| Qualisys | unchanged (v1 protocol, 2021-11-30 15:44) | | |

Why this is legitimate (rule 4 of the held-out protocol: a role changes only
in a new version, with a reason, and the file hashes are kept): the data files
and their sha256 are unchanged and the v1 roles stay in a comment of each
file; no joint-level model was fitted or scored on either session (the
registration-only sanity fits are the baseline and informed no choice); nothing
is selected on any role, since the harness `select` still refuses
confirmation. The Qualisys `mocap/protocol.yaml` is byte-identical.

## 3. Fixed model and metrics

No method selection anywhere. Every fit uses the `joint_offset` level,
`estimation.method: structural`, the xyz position of `wrist_ft_tool_link`
(6D: plus orientation), base 6D and tool point 3D free. The base absorbs
`torso` and `arm_1`; the tool point absorbs `arm_7`; the comparable
parameters are the offsets of `arm_2 .. arm_6`.

- **Re-registration**: keep the trained joint offsets, refit base 6D and the
  tool point 3D (6D: the tool rotation too) on the evaluated session.
- **Primary metric**: leave-one-out (LOO) re-registered RMSE: refit the frames
  on n-1 postures, score the held-out one. Sessions with fewer than 6 postures
  are not evaluated.
- **Secondary (6D)**: the 2023 OptiTrack and Vicon sessions are also scored
  with position and orientation. The residual is core's `TiagoCalibration`
  log-map residual with the orientation rows scaled by 0.1 m/rad (`W_ROT`):
  core's default `measurement_std` (position 0.001 m, orientation 0.01 rad)
  through `BaseCalibration.apply_measurement_weighting`. TIAGo's own cost
  function bypasses that weighting, so the study script applies it. The same
  weighting enters the 6D joint fit, the re-registration and the metric. 6D
  bands are **not optimal until core #215** and never drive a verdict. The
  Qualisys model is trained on position only, so its 6D gaps include the
  orientation it was never fitted to and are larger by construction.

## 4. Effective noise per system

Registration only, from paired postures, no joint fit:
`sigma_eff = sqrt(SSR / (2 (3n - p)))`, with n postures, p the registration
parameters. OptiTrack 15:29 and 15:52 share a rigid-body definition (rigid
registration, p = 6); Vicon 15:08 and 16:07 share 15 postures (rigid plus a
tool-point shift through the gripper orientation, p = 9); Qualisys uses the
November repeated postures against the training session (the training session
itself is excluded).

| | pairs | registration params | sigma_eff (mm) | noise level used (mm) |
|---|---|---|---|---|
| `noise qualisys` | 109 | 0 | 0.57 | 2.0 |
| `noise optitrack` | 4 | 6 | 1.25 | 2.0 |
| `noise vicon` | 15 | 9 | 0.78 | 2.0 |

The noise level of a system is the smallest grid level in {0.5, 2.0, 4.0} mm
at or above its sigma_eff, so every system is simulated at 2.0 mm per axis.
The OptiTrack figure rests on 4 pairs and is the least certain.

Clock lag has no term on static postures: a posture's mocap stamp is read
inside a plateau at least 0.3 s wider than the lag uncertainty on each side
(OptiTrack 0.04 s, Vicon 0.005 s; 15:29 excluded), a margin the tests assert.
Lag is measured where it matters, on the square motion, in part 2.

## 5. Pre-registered thresholds (simulated)

Fixture: one truth per seed (`draw_truth("joint_offset", default_rng(seed))`,
offsets only), shared by all systems; each session is simulated at its real
postures with its own registration-only frames and its own noise stream
`default_rng(10000 + 100 k + seed)` (qualisys 0, optitrack 1, vicon 2), so the
systems are never correlated. Seeds 0-4. The same structural fit is run on each
system's simulated training session; the three systems keep identical parameter
sets (asserted; 14 names in 3D, 17 in 6D).

Rules:

- transfer band(A to E) = |mean_s G_s| + 2 sd_s(G_s), where
  `G = e(M_A, E) - e(M_own, E)`, the LOO re-registered RMSE of system A's model
  on session E minus that of E's own system, over seeds;
- parameter band_j(A, B) = 2 RMS_s(theta_jA - theta_jB), j in arm_2 .. arm_6.

| | value |
|---|---|
| `recovery z rms` | 1.25 |
| recovery z mean / max abs / n | +0.10 / 3.75 / 125 |

Recovery z-scores are (fitted - true) / reported standard error of the
comparable parameters; an RMS near 1 means the standard errors are honest at
the used noise.

### 5.1 Primary: 3D transfer bands (mm, used level 2.0 mm)

| Band | mean gap (mm) | sd (mm) | band (mm) | band at 0.5 / 4.0 mm |
|---|---|---|---|---|
| `optitrack->vicon/2023-09-20-1508|3d` | +0.82 | 0.30 | **1.42** | 0.35 / 2.84 |
| `optitrack->vicon/2023-09-20-1607|3d` | +0.18 | 0.66 | **1.49** | 0.37 / 2.98 |
| `qualisys->optitrack/2023-09-12-1552|3d` | +0.60 | 1.11 | **2.82** | 0.70 / 5.68 |
| `qualisys->optitrack/2023-09-12-1658|3d` | +1.22 | 0.76 | **2.74** | 0.68 / 5.51 |
| `qualisys->vicon/2023-09-20-1508|3d` | +1.01 | 0.38 | **1.78** | 0.44 / 3.56 |
| `qualisys->vicon/2023-09-20-1607|3d` | +0.34 | 0.26 | **0.86** | 0.21 / 1.73 |
| `vicon->optitrack/2023-09-12-1552|3d` | -0.59 | 1.08 | **2.75** | 0.68 / 5.52 |
| `vicon->optitrack/2023-09-12-1658|3d` | +0.73 | 0.29 | **1.31** | 0.33 / 2.61 |

### 5.2 Primary: parameter bands (mrad, used level 2.0 mm)

| Pair | arm_2 | arm_3 | arm_4 | arm_5 | arm_6 |
|---|---|---|---|---|---|
| `optitrack-vicon|3d` | 5.41 | 5.58 | 5.27 | 13.69 | 8.10 |
| `qualisys-optitrack|3d` | 7.18 | 2.97 | 6.01 | 25.08 | 9.20 |
| `qualisys-vicon|3d` | 7.77 | 5.07 | 5.91 | 12.69 | 6.77 |

### 5.3 Secondary: 6D transfer bands (mm-equivalent, `W_ROT = 0.1` m/rad; not optimal until core #215)

| Band | mean gap (mm) | sd (mm) | band (mm) | band at 0.5 / 4.0 mm |
|---|---|---|---|---|
| `optitrack->vicon/2023-09-20-1508|6d` | +0.70 | 0.28 | **1.26** | 0.30 / 2.52 |
| `optitrack->vicon/2023-09-20-1607|6d` | +0.16 | 0.56 | **1.28** | 0.29 / 2.60 |
| `qualisys->optitrack/2023-09-12-1552|6d` | +0.74 | 1.07 | **2.89** | 0.70 / 5.83 |
| `qualisys->optitrack/2023-09-12-1658|6d` | +1.22 | 0.75 | **2.71** | 0.62 / 5.47 |
| `qualisys->vicon/2023-09-20-1508|6d` | +1.06 | 0.44 | **1.93** | 0.46 / 3.88 |
| `qualisys->vicon/2023-09-20-1607|6d` | +0.44 | 0.26 | **0.96** | 0.22 / 1.95 |
| `vicon->optitrack/2023-09-12-1552|6d` | -0.50 | 0.94 | **2.38** | 0.58 / 4.77 |
| `vicon->optitrack/2023-09-12-1658|6d` | +0.64 | 0.27 | **1.17** | 0.29 / 2.34 |

### 5.4 Secondary: 6D parameter bands (mrad, OptiTrack versus Vicon)

| Pair | arm_2 | arm_3 | arm_4 | arm_5 | arm_6 |
|---|---|---|---|---|---|
| `optitrack-vicon|6d` | 5.09 | 5.13 | 4.92 | 10.06 | 6.27 |

## 6. Verdict rules (written before the real fit)

- A real transfer gap or parameter difference **beyond its band** reads "not
  explained by measurement hardware alone".
- Within the band reads "consistent with measurement hardware alone"; this
  is absence of evidence of another cause, not proof.
- Five seeds make a band uncertain by about 35 %; a real value within 35 % of
  its band is reported as borderline, not as a verdict.
- The 6D rows are reported next to the 3D ones and never decide.
- Date and hardware are confounded for every 2021-versus-2023 comparison;
  only OptiTrack versus Vicon (both September 2023) separates hardware.

## 7. Positive control (C5)

The Qualisys sessions 2021-11-30 10:51 and 11:02 differ in the recorded
encoder offset of `arm_6` by about -19.3 mrad (sign by convention). A fit of
each must show a difference of `arm_6` within 2 sqrt(SE1^2 + SE2^2) of that
value, `arm_1 .. arm_5` within their errors of 0, and `arm_7` (+20.5 mrad)
unobservable with a 3D tool point. If the control fails, nothing else in part
2 is interpreted.

## 8. Design of part 2 (not run here)

C1 per-system fit and own-system LOO; C2 the 2021 model on the 2023 sessions
against the transfer bands; C3 near-date hardware, the OptiTrack model on
Vicon and the reverse, against the parameter bands; C4 square-motion path
error with a lag sweep; C5 above. Descriptive per-session fits are allowed
(they fix a model, nothing is chosen on them).

## 9. Reproduction

```bash
cd examples/tiago
python mocap_hardware.py --stage noise --no-plot          # effective noise
python mocap_hardware_truth.py --out bands.json            # about 5 min, 5 seeds x 3 levels
python mocap_hardware_truth.py --golden ../../tests/golden/tiago_mocap_hardware_truth.json
```

`tests/test_tiago_mocap_hardware.py` and `tests/test_tiago_mocap_hardware_truth.py`
check the library, the fixture (slow: seed 0 against
`tests/golden/tiago_mocap_hardware_truth.json`) and that this note quotes the
golden values.

---

# Part 2: study and results

## 10. Design of the study stages

Run with `python mocap_hardware.py real|square|all` (examples/tiago); code in
`examples/tiago/mocap_hardware_study.py`. The model, bands and verdict rules
are those of sections 3-7, unchanged. The `noise`-only statement of section 1
described part 1 and is superseded.

- `real`: the one fixed model (section 3) is fitted on each training session
  (Qualisys 1544, OptiTrack 1658, Vicon 1508) and descriptively on the others.
  Every model is then scored on every evaluated session by leave-one-out (LOO)
  re-registration (refit the frames on n-1 postures with the offsets kept,
  score the held-out posture). LOO is primary, in-sample is secondary.
  Sessions with fewer than 6 postures are not scored (OptiTrack 1529).
- Structural selection kept the same 14 parameters for every system in 3D (17
  in 6D) and every fit converged; no outliers were removed.
- Arm_5-only diagnostic: the Qualisys model with only the arm_5 offset
  refitted on each 2023 session.
- 6D secondary (2023 data only, `W_ROT = 0.1` m/rad): reported next to the 3D
  results, labelled not optimal until core #215, never decides a verdict.
- `square`: each of the seven square-motion runs is registered rigidly (plus
  tool point) in-sample for each offset set, applied as `q + offset` on the
  nominal model; the recorded `mocap` set is applied as a placement. A lag
  sweep reads the joints at `t + delta` for delta = -0.05 and +0.05 s.
- `--archive` writes a run record (`examples/run_record.py`) under the asset
  `tiago-48` with the sessions, protocols and frozen bands as inputs; `validate.py` runs
  `real --archive`.

Verdict aggregation (convention of this study, a review point): a comparison
is "not explained by measurement hardware alone" if any item is beyond its band
(ratio > 1.35), else "borderline" if any is within 35 % of the band, else
"consistent with measurement hardware alone" (absence of evidence of another
cause, not proof).

## 11. Verdicts against the pre-registered bands

| Comparison | Items beyond / borderline / within | Verdict |
|---|---|---|
| `C1` per-system fit and own LOO | no band | descriptive (no pre-registered band) |
| `C2` 2021 model on 2023 sessions | 3 beyond, 0 borderline, 1 within | not explained by measurement hardware alone |
| `C3` OptiTrack vs Vicon (parameters and transfer) | parameters 2 beyond, 3 within; transfer 3 beyond, 1 within | not explained by measurement hardware alone |
| `C5` positive control, Qualisys 1051 vs 1102 | every parameter within its limit | control passed |
| `C4` square motion | see section 13 | descriptive |
| 6D secondary | same pattern as 3D | not explained by measurement hardware alone (secondary, not a verdict) |

C2 is interpreted because C5 passed, but C5 has low power (section 15).

### 11.1 C1: per-system fits (mrad; arm_2..arm_6 are the comparable parameters)

| Model | n | arm_2 | arm_3 | arm_4 | arm_5 | arm_6 | own LOO (mm) |
|---|---|---|---|---|---|---|---|
| `M_Q` Qualisys 1544 | 37 | -2.3 | 1.0 | -3.7 | -49.8 | 2.9 | 3.20 |
| `M_OT` OptiTrack 1658 | 13 | -11.5 | -2.1 | -1.7 | -79.1 | 2.7 | 9.45 |
| `M_V` Vicon 1508 | 17 | -12.3 | -2.5 | -2.4 | -119.8 | 32.6 | 8.72 |

Standard errors are 2-3 mrad (Q), 4-5 (OT, arm_2..4), 11 (arm_5). Fit RMSE on
the 2023 sessions is 7-9 mm against about 2 mm of simulated noise, so
unmodelled error dominates the 2023 data.

### 11.2 C2: Qualisys model on the 2023 sessions

| Held-out session | own LOO (mm) | Q-model LOO (mm) | gap (mm) | band (mm) | ratio | class |
|---|---|---|---|---|---|---|
| OptiTrack 1552 | 11.08 | 16.43 | 5.35 | 2.82 | 1.90 | beyond |
| OptiTrack 1658 | 9.45 | 11.13 | 1.68 | 2.74 | 0.61 | within |
| Vicon 1508 | 8.72 | 16.56 | 7.84 | 1.78 | 4.42 | beyond |
| Vicon 1607 | 8.72 | 17.34 | 8.62 | 0.86 | 10.05 | beyond |

### 11.3 C3: OptiTrack against Vicon

Parameter differences (OT minus V, mrad) against the band: arm_2 0.85/5.41,
arm_3 0.46/5.58, arm_4 0.62/5.27 (all within); arm_5 40.7/13.7 (ratio 2.97) and
arm_6 -29.9/8.1 (ratio 3.69), both beyond. Transfer: the OptiTrack model on
Vicon 1508 and 1607 is beyond (ratios 2.94, 3.29); the Vicon model on
OptiTrack 1658 is beyond (3.41) and on 1552 within (0.09; 1552 is a weakly
determined session, n=9).

### 11.4 C5: positive control

Qualisys 2021-10-26 1051 against 1102 (same hardware, same robot, different
session). Limit = 2 sqrt(SE1^2 + SE2^2). arm_2 6.2 (limit 7.3), arm_3 1.2
(6.6), arm_4 7.0 (7.3), arm_5 2.6 (24.0), arm_6 deviation 16.5 from the
expected -19.3 (limit 36.9): all pass. arm_1 and arm_7 are unobservable.

### 11.5 Arm_5-only diagnostic

Refitting only arm_5 of the Qualisys model (-49.8 mrad) on the 2023 sessions
gives -91.0 (OT 1552), -71.7 (OT 1658), -116.2 (V 1508), -122.0 (V 1607) mrad
and LOO 14.3, 10.8, 12.4, 12.9 mm. It closes part of the OptiTrack gap but not
the Vicon gap (own LOO 8.7 mm), so a single joint does not explain the
transfer failure on Vicon.

### 11.6 Residual-step diagnostic (post hoc)

Added after the results above, not pre-registered, not part of any verdict; it
selects nothing and changes no model. For each session with its own fit, the
per-posture residual norm of that fit is taken in recording order (the rows of
the shipped CSVs; `t_start_robot` increases in every file). The best single
split minimises the within-segment sum of squares of the residual norm, with at
least 3 postures on each side. For that split, the arm_5 offset is refitted
separately before and after (frames refitted, other offsets kept).

| Session | Model | Split (rows before / n) | Mean before (mm) | Mean after (mm) | RMSE (mm) | RMSE with split arm_5 (mm) | arm_5 jump (mrad) |
|---|---|---|---|---|---|---|---|
| `step optitrack/2023-09-12-1552` | OT1552 | 3 / 9 | 2.97 | 5.28 | 5.06 | 5.00 | -26.0 |
| `step optitrack/2023-09-12-1658` | M_OT | 3 / 13 | 4.81 | 7.42 | 7.09 | 7.07 | +21.9 |
| `step qualisys/2021-10-26-1051` | Q1051 | 5 / 26 | 3.73 | 1.98 | 2.62 | 2.61 | +4.4 |
| `step qualisys/2021-10-26-1102` | Q1102 | 3 / 25 | 3.79 | 2.01 | 2.56 | 2.54 | +8.8 |
| `step qualisys/2021-11-30-1544` | M_Q | 34 / 37 | 2.53 | 4.06 | 2.88 | 2.85 | -22.9 |
| `step vicon/2023-09-20-1508` | M_V | 13 / 17 | 7.39 | 4.76 | 7.26 | 7.24 | -7.4 |
| `step vicon/2023-09-20-1607` | V1607 | 5 / 21 | 5.37 | 7.44 | 7.29 | 7.25 | +15.5 |

Reading: no session shows a step that a separate arm_5 offset explains. The
RMSE drop from the extra arm_5 term is at most 0.06 mm against 5-7 mm residuals
in the 2023 sessions, and most best splits sit at the 3-posture edge, where a
two-segment mean always fits a little better. The fitted jumps (-26 to +22
mrad) change sign across sessions and come with no RMSE gain.
This diagnostic finds no evidence of a slip inside a session; it cannot exclude
one between sessions, which is what the fits above compare, and with 9 to 37
postures per session it has little power for a small step.

## 12. 6D secondary (2023 data)

At `W_ROT = 0.1` m/rad, OT minus V: arm_2..4 within (differences 0.6, 0.3,
-0.01 mrad), arm_5 43.9 (band 10.1, ratio 4.36) and arm_6 -30.7 (band 6.3,
ratio 4.90) beyond. Of eight transfer items, six are beyond and two within,
the same items as in 3D. The 6D fit does not move arm_5 or arm_6 (OT -75.8,
V -119.7 for arm_5), so adding the tool orientation does not resolve the OT/V
disagreement. Not optimally weighted until core #215; reported, never decisive.

## 13. C4: square motion (path RMSE, registered, in-sample, mm)

Mean over the seven runs (per-run values in the golden). Lower is better; the
tool point, base frame and a rigid registration are refitted per run.

| Model | Mean RMSE (mm) |
|---|---|
| `C4 M_Q` | 2.83 |
| `C4 M_OT` | 2.90 |
| `C4 Q1051` | 3.02 |
| `C4 Q1102` | 3.18 |
| `C4 M_V` | 3.28 |
| `C4 zero` | 3.52 |
| `C4 mocap set` | 8.05 |
| `C4 mocap_inv set` | 8.93 |

Reading: every fitted model (2021 Qualisys, 2023 OT and V) improves on the
nominal model by 7-20 %, and they differ by 0.5 mm, less than the spread
between runs. The square motion therefore does not separate the hardware; it
shows the offsets of all three systems describe the same joint errors to about
this precision. The two recorded sets applied directly are much worse than
zero: a placement set is not a joint-offset set and is reported only for
completeness (the inverse set is the sign check; neither helps).

Lag sweep (mean over runs, reading the joints at t + delta): `M_Q` 2.96 mm at
-0.05 s, 2.83 at 0, 3.71 at +0.05; `zero` 3.53, 3.52, 4.34. The minimum is at
zero lag, so the clock lag of the sessions (section 4) is not a material
error source here.

## 14. Recovery z of the fixture, 3D and 6D

The fixture output (`mocap_hardware_truth.py --noise 2.0`, 5 seeds, used level)
reports the z of (fit - truth) / standard error:

| Mode | n | z rms |
|---|---|---|
| `recovery z 3d` | 75 | 1.01 |
| `recovery z 6d` | 50 | 1.54 |
| `recovery z all` | 125 | 1.25 |

The pre-registered 1.25 is the mix of both. The 3D standard errors are honest
(z rms about 1). The 6D standard errors are understated by about 1.5: the
`W_ROT = 0.1` m/rad weighting is not the optimal covariance (core #215). The
frozen 6D bands come from the simulated spread of the fits, not from the
standard errors, so they are unaffected; only the 6D standard errors printed
here should be inflated by about 1.5 when read. No band was changed.

## 15. Limits

- Date confound. Qualisys is 2021, OptiTrack and Vicon are 2023. A 2021 to
  2023 difference mixes hardware with any physical change of the robot, so C2
  cannot attribute a gap to hardware even when it were within band; it can only
  say a gap is beyond what hardware noise alone produces.
- OptiTrack has small n (13 postures in the training session, 9 in 1552), so
  its standard errors and LOO are coarse.
- Vicon separates base and tool poorly. Its arm_5/arm_6 pair (-119.8, +32.6
  mrad) trades against the frames; the OT/Vicon difference is concentrated in
  exactly these two parameters, and the near-date C3 failure may reflect this
  weak separation instead of the hardware.
- The pre-registered bands come from 5 seeds; the band uncertainty is about
  +-35 %, which is why the borderline zone is +-35 %. Items within that zone are
  not resolved.
- C5 has low power. The arm_6 control limit (36.9 mrad) contains both the
  expected -19.3 and zero, so arm_6 cannot discriminate; arm_2 and arm_4 sit at
  6.2 and 7.0 of limits 7.3 and 7.3. Passing the control shows the method does
  not manufacture a large difference between two same-hardware sessions; it
  does not show it can detect a small one.
- The 2023 fit RMSE (7-9 mm) is far above the simulated noise (about 2 mm), so
  the data carry unmodelled error (a series-elastic or process-history effect,
  frame offsets). The bands assume noise-only error and are therefore optimistic
  for the 2023 data.
- Square-motion RMSE is in-sample with the registration refitted, so it is
  insensitive to a frame-level difference between models.
- Determinism. Each fit uses a fresh calibration object and a deterministic
  Kabsch plus Levenberg-Marquardt initial frame. Fitting in reverse order gives
  parameters identical to forward order (maximum difference 0), and the slow
  test runs the real stage in reverse order against the golden. The small
  process-history drift (about 4e-5 rad) seen in part 1 did not reproduce.

## 15a. Interpretation: one robot, arm_5

Maintainer information (2026-10-21): every session of this study (Qualisys
2021, OptiTrack 2023-09-12, Vicon 2023-09-20 and the square motion) is the same
robot, tiago-48, which the backlash (#71) and suspension (#70) studies also
use. A difference between robots is therefore ruled out as a cause. The same
information says arm_5 of tiago-48 has a mechanical issue: it slips when an
external force is applied while the robot is at rest.

Read with the results: the arm_5 shifts of C2 and C3 (40.7 mrad OptiTrack
against Vicon, 8 days apart; about 90 mrad from 2021 to 2023) are likely caused
by that slip, not by the measurement hardware. A slip inside a session would
also raise the 2023 fit RMSE (7-9 mm); the residual-step diagnostic (11.6)
neither shows nor excludes it. arm_6 (-29.9 mrad OptiTrack minus Vicon) is also
beyond its band, and refitting only arm_5 does not close the Vicon gap, so
either arm_6 also changed between sessions or it absorbs part of the arm_5
slip through the weak base/tool separation; these data do not tell which. The
slip is not demonstrated by this study: it is the maintainer's explanation of
the arm_5 shifts. The C2 and C3 verdicts stay "not explained by measurement
hardware alone". Joint-inspection recordings of arm_5 and arm_6 are tracked in
#110; the backlash study is #71.

## 16. Conclusion and reproduction

Against the pre-registered bands, the difference between the Qualisys 2021
model and the 2023 OptiTrack and Vicon sessions, and between OptiTrack and
Vicon, is not explained by measurement hardware alone: the excess is carried by
arm_5 and arm_6. Possible causes (the arm_5 slip of section 15a, weak Vicon
base/tool separation, unmodelled error) cannot be separated with these data;
the square-motion check shows the three offset sets perform alike on an
independent task. Follow-up: per-session frame registration with shared joint
parameters would separate frame and joint effects (a core issue).

```bash
cd examples/tiago
python mocap_hardware.py real --archive --no-plot       # about 1 min, writes a run record
python mocap_hardware.py square --no-plot
python mocap_hardware.py all --no-plot --golden ../../tests/golden/tiago_mocap_hardware.json
python mocap_hardware_truth.py --noise 2.0 --out z.json  # recovery z by mode
```

`tests/test_tiago_mocap_hardware_study.py` checks the verdict rules, that the
golden follows the frozen bands and the shipped registration RMSE, that this
note quotes the golden, and (slow) reruns the real stage in reverse fit order
and the square stage against `tests/golden/tiago_mocap_hardware.json`.
