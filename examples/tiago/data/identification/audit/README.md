# Identification-input audit data (#68)

Channels trimmed from the 2021-07 introspection bags (original recordings, not
distributed), so that `examples/tiago/identification_inputs_audit.py` and
`tests/test_tiago_identification_inputs_audit.py` reproduce the findings in
[`docs/development/tiago-identification-inputs-2026-10-10.md`](../../../../../docs/development/tiago-identification-inputs-2026-10-10.md)
without the bags. The files were written by
`examples/tiago/utils/audit_extraction.py` from the introspection export
beside each bag (one row per sample, 511 channels). All clocks `t` are the
message header stamp minus the first stamp (s), as in the shipped CSVs of
`../dynamic/`. Nothing here is used by `identification.py`.

| File | Source recording | Rows | Columns |
|---|---|---|---|
| `torso_{20,40,60,80}.csv` | torso lifted at 20/40/60/80 % speed, arm still | 3435 / 2283 / 1538 / 1304 | `t`, `torso_lift_joint_position` (m), `torso_lift_joint_velocity` (m/s, PAL's filtered estimate), `torso_lift_joint_effort` (raw, unit unknown) |
| `controller_constants.csv` | `calibration.bag` (the training run) | 4 | `joint`, `channel` (introspection channel the value came from), `value` (median over the run), `min`, `max` |
| `differential_wrist_calibration.csv` | `calibration.bag`, every 4th sample | 2006 | `t`, then for arm_6 and arm_7: `*_motor_position` (rad), `*_motor_effort` (raw), `*_joint_position` (rad), `*_joint_effort` (raw) |
| `channel_status.csv` | `calibration.bag` | 8 | per joint: fraction of NaN `*_torque_sensor` samples, distinct `*_motor_mode` values, NaN fraction and maximum absolute value of `*_motor_motor_effort_command` (empty where the channel is NaN) |
| `end_effector_channels.txt` | `calibration.bag` | - | the `hand_*` joints and motors with logged position channels, and the count of channels named `gripper` or `finger` (0) |

Notes:

- **Controller constants:** the motor torque constants of PAL's loaded but
  inactive `gravity_compensation` controller: arm_1 0, arm_2 0.136,
  arm_3 and arm_4 -0.087. They are constant over the run. The controller has no
  entry for the torso or the wrist.
- **Differential wrist:** the identities
  `q6 = (m7 - m6)/2 + 0.0197129`, `q7 = (m6 + m7)/2 - 0.0200`,
  `tau6 = e7 - e6`, `tau7 = e6 + e7` (m: motor position, e: motor effort) hold
  at floating-point precision on every kept sample.
- **Channel status:** `*_torque_sensor` is NaN on every joint (no joint torque
  sensor); `*_motor_mode` is 0 on every joint; the effort command is 0 where it
  exists (arm_1-arm_4) and NaN elsewhere.
- Torso efforts have step 0.01; their unit is not documented, and the torso
  conversion in the config is unverified (#68).
