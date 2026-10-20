# TIAGo mocap across hardware (#100): pre-registration

Issue: [#100](https://github.com/thanhndv212/figaroh-examples/issues/100).
Part 1 of 2 (pre-registration). The study itself (real-data fits, square
motion, verdict) follows in part 2, after this note is merged.

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
