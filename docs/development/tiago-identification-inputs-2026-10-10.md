# TIAGo identification inputs: evidence for #68

2026-10-10. Branch `fix/68-tiago-id-inputs`. This is the reproducible evidence
behind the input decisions of the TIAGo identification example: velocity,
effort constants, torso, wrist and end effector. Every number below is printed
by `examples/tiago/identification_inputs_audit.py`, asserted in
`tests/test_tiago_identification_inputs_audit.py`, or copied from
`examples/tiago/data/README.md`. It builds on the signal audit
([#20](tiago-signal-audit-2026-10-03.md)) and the cross-run protocol
([#69](tiago-identification-cross-run-protocol.md)).

Environment: `figaroh-dev`, run from `examples/tiago`:

```bash
python identification_inputs_audit.py              # about 2 s
python identification_inputs_audit.py --gain-scan  # adds section 4's scan, about 2 min
python -m pytest tests/test_tiago_identification_inputs_audit.py
```

## 1. Sources

**Shipped before this work** (`examples/tiago/data/identification/`,
hashes frozen in `protocol.yaml`): for the `dynamic`, `calibration_slow` and
`calibration_weight` recordings, `tiago_{position,velocity,effort}.csv` and
`tiago_wrist_ft.csv`, on the message-header clock `t`. These alone give the
velocity filter, the wrist effort quantisation and the F/T mass below the
sensor.

**Derived for this report** (`data/identification/audit/`, described column by
column in its `README.md`): channels trimmed from the 2021-07 introspection
bags (original recordings, not distributed) by
`examples/tiago/utils/audit_extraction.py`. Total 996 kB.

| File | Content |
|---|---|
| `torso_{20,40,60,80}.csv` | torso position, PAL's velocity estimate and raw effort of the four torso-lift runs |
| `controller_constants.csv` | PAL's `gravity_compensation` motor torque constants, with their channel names |
| `differential_wrist_calibration.csv` | arm_6/arm_7 motor and joint positions and efforts of the training run, every 4th sample |
| `channel_status.csv` | NaN share of `*_torque_sensor`, `motor_mode` values and effort command per joint |
| `end_effector_channels.txt` | the `hand_*` channels logged, and the count of gripper channels |

## 2. Findings

### 2.1 Velocity is a first-order filter, not a delay

The shipped velocity satisfies `v[n] = a v[n-1] + (1-a) (q[n]-q[n-1])/(t[n]-t[n-1])`
on the header clock `t` (checked: position and velocity files carry the same
`t`). Fitting `a` per joint, over all eight joints of the three runs:

| Run | a | Relative residual | Same, nominal 10 ms clock (arm_3, torso) | Best constant shift | Relative residual of that shift |
|---|---|---|---|---|---|
| dynamic | 0.95011-0.95022 | 1.5e-4 | 2.8e-3, 3.0e-3 | 14-18 samples | 0.107-0.186 |
| calibration_slow | 0.95014-0.95026 | 1.3e-4 - 1.4e-4 | 2.8e-3, 2.7e-3 | 7-21 samples | 0.117-0.173 |
| calibration_weight | 0.95009-0.95021 | 1.3e-4 - 1.4e-4 | 2.8e-3, 2.8e-3 | 14-18 samples | 0.108-0.186 |

(Ranges cover all eight joints; the nominal-clock column is printed for arm_3
and the torso.) So `a = 0.95`, a 0.195 s time constant. A pure
shift leaves 11-19 % relative error against 0.015 % for the filter, and its
best value changes from joint to joint, so it is not a delay. The header
clock matters: with a nominal 10 ms step the residual is about 20 times
larger.

Reproduce: section (a) of the script output.

### 2.2 arm_1's torque constant: PAL logs 0

PAL's loaded but inactive `gravity_compensation` controller logs these motor
torque constants (`audit/controller_constants.csv`; constant over the run):

| Joint | Constant | Channel |
|---|---|---|
| arm_1 | 0 | `local_control_motor_torque_constant_arm_1_joint` |
| arm_2 | 0.136 | `local_control_motor_torque_constant_arm_2_joint` |
| arm_3 | -0.087 | `local_control_motor_torque_constant_arm_3_joint` |
| arm_4 | -0.087 | `local_control_motor_torque_constant_arm_4_joint` |

The controller has no entry for the torso or the wrist. arm_2-arm_4 match the
config's drive table. arm_1's 0.136 is the example's choice, copied from arm_2.
arm_1's axis is vertical (its world-frame axis has |z| = 1.000000 over the
payload run and its gravity torque is below 1.1e-14 N·m), so gravity cannot
anchor its scale. The payload run, which is the only independent scale check
(F/T payload 0.489 kg, from `payload_check.py`), moves arm_1 by 0.099 N·m RMS.
The payload is placed on the arm_7 link with the F/T mass and the difference of
the two runs' effort-derived first moments (`[0.003, 0.003, 0.079]`), through
`computeJointTorqueRegressor`. Against it, arm_1's model error on that run is
0.99 N·m RMS (nominal inverse dynamics plus friction, in sample; the held-out
RMSE with the identified model is 1.310), a ratio of 0.10. The payload is below
the model error, so it cannot calibrate the constant.

Scan, `--gain-scan`: arm_1's `kmotor` scaled and the example re-run, held-out
`calibration_slow`:

| Scale | kmotor | Pooled arm_2-arm_4 RMSE | arm_1 RMSE | arm_1 relative error |
|---|---|---|---|---|
| 0.25 | 0.0340 | 1.839 | 0.378 | 0.48 |
| 0.40 | 0.0544 | 1.839 | 0.555 | 0.45 |
| 0.55 | 0.0748 | 1.838 | 0.741 | 0.43 |
| 0.75 | 0.1020 | 1.838 | 0.992 | 0.42 |
| 1.00 | 0.1360 | 1.839 | 1.310 | 0.42 |
| 1.50 | 0.2040 | 1.841 | 1.949 | 0.42 |

The arm_2-arm_4 RMSE is flat (1.838-1.841) and arm_1's relative error stays at
0.42-0.48, because arm_1's base parameters absorb the scale.

### 2.3 Torso effort

Raw torso effort of the four torso-lift runs (arm still; lifting means speed
above 0.005 m/s, rest means below):

| Run | Peak speed (m/s) | Lifting | Rest | Rest before the lift | Rest after the lift |
|---|---|---|---|---|---|
| torso_20 | 0.022 | +1.71 | -1.10 | -3.58 | +0.75 |
| torso_40 | 0.037 | +1.77 | -1.13 | -3.63 | +0.64 |
| torso_60 | 0.053 | +1.79 | -1.75 | -3.64 | +0.45 |
| torso_80 | 0.067 | +1.78 | -1.87 | -3.66 | +0.36 |

Effort while lifting is 1.7-1.8 at every speed (Coulomb-like, no viscous
term), and the resting value depends on where the torso rests, not on the
speed: about -3.6 before the lift and +0.4 to +0.75 after it. That looks like a
current with a counterbalance offset, in an unknown unit. The config converts
it with `ratio x kmotor = 1` and adds `9.81 x subtree mass`: on the training
window the URDF term is 181.8 N (18.53 kg) of a mean converted force of
182.9 N, i.e. 99.4 % (raw effort std 0.75). The torso "measurement" is the
model's own weight. Issue #68 quoted about 98 %; the 99.4 % here uses the
Hey5 URDF on the training window, rows 921-6791.

Reproduce: section (b).

### 2.4 Wrist: quantisation and the differential

**Quantisation** (training run, from the shipped effort file):

| Joint | Step | Distinct values | Exactly zero |
|---|---|---|---|
| torso | 0.01 | 349 | 0.0 % |
| arm_1 | 0.001 | 1515 | 0.3 % |
| arm_2 - arm_4 | 0.001 | 1963-2088 | 0.0-0.1 % |
| arm_5 | 0.001 | 40 | 87.7 % |
| arm_6 | 0.001 | 42 | 90.1 % |
| arm_7 | 0.001 | 42 | 90.1 % |

**Differential.** On the 2006 kept samples of the training run, arm_6 and
arm_7 positions and efforts are exact sums and differences of the two motor
channels (m: motor position, e: motor effort):

- `q6 = (m7 - m6)/2 + 0.0197129` and `q7 = (m6 + m7)/2 - 0.0200` (the first
  offset is 0.0197129 to the digits logged; the constant residual has a
  standard deviation of 5.6e-17);
- `tau6 = e7 - e6` and `tau7 = e6 + e7`.

Largest residuals: q6 2.2e-16, q7 2.2e-16, tau6 9.9e-17, tau7 9.9e-17. Both
efforts are built from the same two near-zero currents, hence arm_6 and arm_7
efforts equal on 94.4 % of the samples. The wrist torque fit is therefore
limited by the resolution of two motor currents.

Reproduce: sections (d) and (e).

### 2.5 End effector

- **Channels.** The bags log 36 `hand_*` joints (and 3 hand motors) and no
  channel containing `gripper` or `finger` (`audit/end_effector_channels.txt`):
  a Hey5 hand, not the Schunk gripper.
- **F/T mass below the sensor** (`payload_check.ft_mass`, Hey5 model):
  0.792 kg (dynamic), 0.793 kg (calibration_slow), 1.280 kg
  (calibration_weight), so the payload is 0.488 kg.
- **URDF mass below `wrist_ft_link`:** Hey5 1.032 kg, Schunk 0.865 kg.

The measured 0.79 kg is lighter than both. It is 0.24 kg below the Hey5 URDF
and 0.07 kg below the Schunk URDF, so the sensor does not by itself favour the
Hey5 hand; the bag channels do. Issue #68 quoted 0.794 kg; the 0.002 kg difference comes from a different
filter and quasi-static selection.

Reproduce: section (f).

### 2.6 Not applicable and inactive channels

`*_torque_sensor` is NaN on every joint (no joint torque sensor), `motor_mode`
is 0 on every joint, and the effort command is 0 where logged (arm_1-arm_4) and
NaN elsewhere: the arm ran in position control (`audit/channel_status.csv`).

## 3. Decisions on `fix/68-tiago-id-inputs`

Held-out RMSE is on `calibration_slow` (N·m; the torso is N).

**Velocity from positions** (default). The velocity is derived from the
filtered positions, since the logged channel holds nothing the positions do
not. The measured channel stays available as `velocity_source="measured"`.

| Velocity | Fit RMSE | Held-out RMSE | Condition |
|---|---|---|---|
| from positions (default) | 0.643 | 1.317 | 1564 |
| logged, shifted (`measured`) | 0.666 | 1.209 | 1779 |
| logged, filter inverted | 0.809 | 1.304 | 1695 |

The held-out RMSE of 1.317 was accepted by the maintainer instead of the
issue's bar of 1.21 or less. The shifted channel predicts the slow run better
although it is the less faithful signal.

**Hey5 URDF.** `identification.py` loads `urdf/tiago_48_hey5.urdf` for the
2021-07 recordings. The Schunk-to-Hey5 switch left the fit and held-out RMSE
(1.209, measured with the shifted velocity) unchanged, and the base-parameter
count (73) too. 30 base-parameter values shifted; the nominal model's held-out
RMSE moved from 3.375 to 3.407.

**Drive table in the config.** The per-joint `reduction_ratio` and `kmotor` are
in `config/tiago_unified_config.yaml` under `drives`, each with a source
(PAL's controller for arm_2-arm_4; "unverified" for the rest).

**`torque_fit_joints` limited to arm_1-arm_4.** The torso force is 99 % the
URDF's own weight and the wrist efforts are exactly zero on 88-90 % of the
samples, so only arm_1-arm_4 effort is fitted and scored; torso and wrist
motion still enter the regressor. This needs the core option
`problem.torque_fit_joints` from figaroh-plus (pull request to be opened,
branch `feat/torque-fit-joints`).

| Effort fitted | Base parameters | arm_1 | arm_2 | arm_3 | arm_4 | Pooled arm_1-arm_4 | Pooled arm_2-arm_4 |
|---|---|---|---|---|---|---|---|
| arm_1-arm_4 (default) | 57 | 1.310 | 2.540 | 1.471 | 1.237 | 1.722 | 1.839 |
| all eight joints | 73 | 1.316 | 2.504 | 1.450 | 1.237 | 1.706 | 1.817 |
| arm_2-arm_4 | 53 | - | 2.583 | 1.480 | 1.248 | - | 1.864 |
| nominal model | - | 3.187 | 7.838 | 3.122 | 2.858 | 4.730 | 5.143 |

Leaving the torso out changes nothing on the arm and leaving the wrist out
costs about 1 %; the gain is that no parameter is fitted to a signal known to
be unreliable.

**arm_1 constant kept at 0.136 and in the fit.** It is not identifiable (scan
and payload signal-to-noise ratio in 2.2). Leaving arm_1 out makes
arm_2-arm_4 worse (1.864 against 1.839), so it stays. Its identified
parameters and absolute RMSE carry the unknown scale; compare runs on the
arm_2-arm_4 column (pooled 1.839).

## 4. Remaining limits

- **Torso force constant:** unknown, undocumented unit; the torso is left out
  of the fit, not corrected.
- **Wrist:** the efforts come from two quantised motor currents; the wrist
  torque fit is not usable until the resolution improves (log the current in mA
  or raise the effort resolution).
- **arm_1 constant:** 0.136 remains an unverified choice. It needs a vendor
  source or a test with a known load about the vertical axis.
- **Payload check:** with the current constants the arm_2-arm_4 efforts read
  13.5 % low in `payload_check.py`, outside the ±10 % tolerance.
- **Hand mass:** the Hey5 URDF is 0.24 kg heavier below the sensor than the F/T
  measurement; the base parameters absorb it, so held-out RMSE is unchanged.
- **Held-out data:** the slow and payload runs share the training path, so
  they test speed and load, not new geometry; they were inspected in the audit
  and are not unseen test sets.
- **Torque scale:** the F/T sensor and the payload run are the only
  independent torque references, and the payload run adds mass.
- **Source:** the derived channels come from the 2021-07 introspection bags
  (original recordings, not distributed); they are an export, not new data.
