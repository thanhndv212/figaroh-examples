# TIAGo Pro right-arm motion-capture data

Frozen inputs of the held-out study (docs/development/tiago-pro-mocap.md).
Roles, files and sha256 are in `protocol.yaml`; the harness refuses to run
if a file changes. The CSVs are never rewritten.

## Sensor and rigid bodies

Qualisys motion capture, one rigid body on the arm tool, `tiago_endEffector`
(columns `x1,y1,z1`, plus `phix1,phiy1,phiz1` where the pose is recorded).
A second body, `tiago_base`, marks the robot base in the mocap frame. Its
pose is not a column: the base frame is a 6D parameter of the fit
(`base_px..base_phiz`), as is the tool frame (`pEE*_1`, `phiEE*_1`).

Positions are in metres in the mocap world frame, angles in radians.
Roll-pitch-yaw convention: `R = Rz(yaw) Ry(pitch) Rx(roll)`.
Joint columns: `torso_lift_joint` and `arm_right_1_joint` ..
`arm_right_7_joint`.

## Sessions

| File | Date | Rows | Measures | Role |
|---|---|---|---|---|
| calibration_samples_20260805_1246.csv | 2026-08-05 | 48 | pose (6D) | training |
| calibration_samples_20260701_0913.csv | 2026-07-01 | 44 | position (3D) | validation |
| calibration_samples_20260702_0756.csv | 2026-07-02 | 94 | position (3D) | confirmation |

The split departs from the original plan of issue #97 because only the
2026-08-05 session records orientation, so it is the only one a 6D fit can
train on. The two earlier sessions are evaluated on position only.

## Known defects

- **Zero column.** The two position-only sessions carry a
  `gripper_right_tool_mount_joint` column that is always 0. The loader
  ignores it; a test checks that it stays zero.
- **Position-only sessions.** 07-01 and 07-02 have no orientation columns.
- **Shared postures.** 07-01 and 07-02 repeat 14 postures (joint difference
  below 1e-3 rad). They are not independent, so the confirmation set is
  less independent of the validation set than its size suggests. The study
  reports errors by stratum (repeats validation / not in validation) so the
  overlap is visible. No held-out posture repeats a training posture.
- **Frame drift.** The mocap-to-robot frame moves between days: re-
  registering the 14 shared postures needs a shift of
  (-0.38, -2.34, -6.64) mm and a 0.30 deg rotation. The training frames
  cannot know this, hence the re-registered primary metric.
- **Plan.** The 08-05 rows are 48 of the 51 postures of
  `optimal_configs.yaml` (indices 1, 19 and 43 are missing), in order.
  Recorded joints differ from the commanded ones by 0.026-0.041 rad on
  every row, a systematic offset.

## Outlier decision

Decided before any model was fitted, from `data_audit.py` (registration-only
fit: base 6D and tool 6D, nominal joints).

- Registration-only residuals: no row exceeds 3 standard deviations. The
  largest are row 35 in position (34.3 mm, z = 2.42) and row 7 in
  orientation (5.7 deg, z = 2.46).
- After the 35-parameter fit, two rows pass 3 sigma: row 40 in position
  (14.4 mm, z = 3.09) and row 15 in orientation (4.9 deg, z = 3.54). These
  are the rows a fit-based detector flags, so the registration-only view
  does not see them.
- Physical evidence for rows 7, 15, 35 and 40: all keep at least 8.5% of the
  joint range from the limits; the gap to the
  commanded posture is 0.026-0.036 rad, like every other row (median
  0.027); no timestamp or marker-residual column exists, so occlusion or a
  marker swap cannot be tested. Row 15 is close to a singularity (smallest
  singular value of the tool Jacobian 0.033, second lowest of 48, median
  0.17), which makes the fit sensitive to it; row 40 is well conditioned
  (0.256).
- **Decision: no row is dropped.** No physical cause was found. Row 15 is a
  modelling sensitivity near a singularity, not a measurement defect, and
  dropping rows on residual size alone would tune the data to the model.
  All 48 rows are used.

## Other files

- `calibration_results_*.yaml`, `master_calibration_*.yaml`: outputs of the
  earlier runs (superseded pipeline, kept as records).
- `protocol_0805_kfold.yaml`: the k-fold plan within the training session
  (diagnostic only, `diagnostics.py`; not a protocol and never used to select).
- `optimal_configs.yaml`: the D-optimal posture plan the sessions executed.
- The superseded dated notes were folded into
  docs/development/tiago-pro-mocap.md.
