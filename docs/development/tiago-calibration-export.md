# TIAGo mocap calibration: exported URDF and PAL file

Issue: [#28](https://github.com/thanhndv212/figaroh-examples/issues/28).
Delivery package: [C3 / core #45](https://github.com/thanhndv212/figaroh-plus/issues/45).
Core changes: [figaroh-plus#62](https://github.com/thanhndv212/figaroh-plus/issues/62)
(FK parity, explicit rejection) and
[figaroh-plus#123](https://github.com/thanhndv212/figaroh-plus/issues/123)
(PAL file).

This checks that what the TIAGo calibration writes reproduces what it
estimated:
- the modified URDF;
- the PAL `master_calibration.yaml`;
- the metrology frames kept beside them.

It also records what each output means.

`examples/tiago/export_check.py` reproduces every number below, and
`tests/test_tiago_export_check.py` checks them. The fits are those of the
[held-out protocol](tiago-mocap-heldout-protocol.md): training session
only, base frame and marker point estimated.

```bash
cd examples/tiago && python export_check.py
```

## 1. Three outputs, three meanings

| Output | Contents | Where it goes |
|---|---|---|
| Modified URDF (`calibration.py` export) | Joint corrections, `calibrator.joint_corrections()`, written into the corrected joints' `<origin>` | Simulation, planning, any URDF consumer |
| PAL `master_calibration.yaml` (`results/runs/.../`) | The same joint corrections, as additive deltas on the nominal URDF origins' xyz/rpy (`<joint>_dx` ... `_dyaw`) | PAL `robot_state_publisher`, on top of the unmodified URDF |
| Metrology frames, `calibrator.metrology_frames()` | Base frame (`base_*`: robot base in the mocap `base_frame` body) and marker point (`pEE*_1`, in the `arm_7` frame) | The measurement setup only; never in the URDF or PAL file |

The metrology frames belong to the mocap setup: where the marker body
was taped, and how the `base_frame` body sits on the base. Another setup
has other values. To reproduce the calibrated marker prediction from either
robot output, apply them outside it, as `export_check.py` does.

## 2. Identified vs written

The fit identifies combinations of parameters, not every joint error
separately. `joint_corrections()` writes one choice of joint values that
reproduces the fit.

**`joint_offset` (the reference).**

| | Parameters |
|---|---|
| Identified (fitted) | `offsetRZ` of `arm_2` ... `arm_6`: −2.3, 1.0, −3.7, −49.8, 2.9 mrad |
| Absorbed by the base frame, written as 0 | `offsetPZ_torso_lift_joint`, `offsetRZ_arm_1_joint` |
| Not a parameter | `offsetRZ_arm_7_joint`: turning `arm_7` about its axis moves the single marker exactly as the marker point does, so the structural selection drops it |

There is one offset per joint and no dependent group, so the written values
are the fitted values.

The torso height and the `arm_1` angle errors aren't recovered. They are
inside the estimated base frame, which is correct for predicting this setup
but leaves the robot model without them.

**`full_params`.**

- **Fitted:** 22 joint placement parameters, plus `d_pz_arm_7_joint`,
  which the frames absorb.
- **Written:** `joint_corrections()` spreads them over all 47 placement
  parameters with the weighted minimum-norm lift (figaroh-plus#111),
  weighted by the expected error sizes. Rows the base frame carries and the
  absorbed parameter are held at 0.
- **Example of a dependent group:** `d_px_arm_5_joint` is fitted at
  8.5 mm, but `d_pz_arm_4_joint` moves the same direction. The written
  values are 4.2 mm each.
- **Reading single joint corrections:** only their combination is
  identified, so a single `full_params` correction is not a measured
  property of that joint.

The values written to the PAL file and the URDF are the same set. Before
#28, `calibration.py` wrote the URDF from the fitted vector (one
representative per group) and the PAL file from the lift. Both reproduced
the fit, but they disagreed joint by joint.

## 3. Parity

The test sets are the four protocol sessions: training (37 postures),
validation (62) and two confirmation sessions (63, 59). On each, the test
compares the marker position predicted by the calibrated model with that
predicted by:
- the reloaded URDF, with the frames applied;
- the nominal URDF plus the PAL deltas (applied by `export_check.py`, not
  by the exporter), with the frames applied.

| Level | URDF, max over all postures | PAL, max over all postures |
|---|---|---|
| `joint_offset` | 2.3e-12 m | 8.4e-15 m |
| `full_params` | 5.6e-12 m | 3.1e-15 m |

The test requires 1e-9 m. The URDF is limited by the 12 significant digits
the exporter writes; the PAL check adds unrounded deltas.

Two defects, found and fixed in core while doing this:
- **Empty PAL file (figaroh-plus#123).** At `joint_offset` it was
  `geometric_calibration: {}`, because only `d_*` names were exported.
- **RPY at gimbal lock (figaroh-plus#123).** `arm_4` and `arm_5` origins
  sit at pitch −π/2. The PAL keys add to the rpy written in the URDF, so
  the deltas are solved against that triplet, not the one Pinocchio
  decomposes. Decomposed angles left 2e-9 to 6e-8 m here, and can leave
  first-order errors at exact gimbal lock.

**Reading the PAL file near gimbal lock.** The URDF origins of `arm_4`,
`arm_5` and `arm_6` are written at pitch −π/2. There, roll and yaw turn about
the same axis, and a small rotation about that axis has no small rpy change.

- **`arm_6`:** its origin is `0 −π/2 −π/2`, so its own axis is that shared
  axis. The reference file therefore carries `arm_6_droll` = −π/2 and
  `arm_6_dyaw` = +π/2 beside `arm_6_dpitch` = 2.9 mrad. That is the 2.9 mrad
  offset, written by trading roll against yaw. It is not a large
  correction.
- **`arm_4` and `arm_5`:** their axes are not the shared axis, so they get
  small pitch deltas.
- **`full_params`:** the three-component corrections at `arm_4` and `arm_5`
  trade about ±0.5 rad of roll against yaw in the same way.

The export picks the smallest exact delta, and the PAL parity above includes
these keys. A runtime that wraps, clamps or interpolates angle deltas
separately would not reproduce them; this is one of the things to confirm on
a robot (section 5).

## 4. What is kept

- The exported URDF differs from the nominal one only in the `<origin>` of
  the corrected joints:
  - `arm_2` ... `arm_6` at `joint_offset`;
  - torso and `arm_1` ... `arm_7` at `full_params`.

  The comparison is numeric. Links, inertias, meshes, transmissions,
  sensor frames (cameras, force-torque) and every other joint are
  identical.
- The nominal URDF is not modified (checked by hash).
- The exporter rejects what it cannot write (figaroh-plus#62). Examples:
  a correction for a joint missing from the URDF, or elastic parameters.

## 5. Limitations

- **Nothing here is applied to a robot.** The PAL semantics, additive
  deltas on the URDF origin xyz/rpy, are the maintainer's reading of PAL's
  `master_calibration.yaml`. They are not verified against a running
  `robot_state_publisher`, including how it handles the roll/yaw trades at
  `arm_6` (section 3).
- **Single marker point.** The corrections explain one marker point on
  `arm_7`. Gripper or camera frames beyond `arm_7`, and the torso/`arm_1`
  errors absorbed by the base frame, are not calibrated.
- **Parity is not accuracy.** It shows the outputs carry the calibrated
  model, not that the model is right. Accuracy on unused postures is the
  [held-out protocol](tiago-mocap-heldout-protocol.md)'s subject, and its
  numbers apply unchanged.
- **One robot, one dataset.** The `full_params` lift depends on the expected
  error sizes (`parameters.estimation.priors`); other sizes write other
  joint values with the same prediction.
