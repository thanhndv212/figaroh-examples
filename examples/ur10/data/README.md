# UR10 dynamic-identification data contract

The UR10 has no hardware identification recordings in this repository. Its
identification example runs on the simulated truth fixture in
[`truth/`](truth/) (#21), whose inertias, clock and effort are known; see the
[fixture report](../../../docs/development/ur10-dynamic-truth-fixture.md).

The earlier `identification_*_simulation.csv` files (and `validation/`) were
removed in #90: they had no saved generator, inertias or clock. Their audit,
fingerprints and the loader corrections made for them are kept in the
[dated signal audit](../../../docs/development/ur10-signal-audit-2026-10-02.md).

## Files

| File | Rows | Duration | Use |
|---|---:|---:|---|
| `truth/train.csv` | 2400 | 24 s | training (default `load_trajectory_data`) |
| `truth/validation.csv` | 800 | 8 s | held-out validation (`validation_data_file`) |

`truth/` also holds the truth URDF and parameters, the frozen optimal
training waypoints, the manifest (seeds, collision clearances, independent
checks, hashes) and the benchmark protocol. Do not edit them: regenerate with
`identification_truth.py` and update the manifest hashes together.

## Channels and units

Columns are selected by name; reordering does not change the joint mapping.
All values must be finite.

| Position | Effort | URDF joint | Units |
|---|---|---|---|
| q0 | tau1 | shoulder_pan_joint | rad; joint-side Nm |
| q1 | tau2 | shoulder_lift_joint | rad; joint-side Nm |
| q2 | tau3 | elbow_joint | rad; joint-side Nm |
| q3 | tau4 | wrist_1_joint | rad; joint-side Nm |
| q4 | tau5 | wrist_2_joint | rad; joint-side Nm |
| q5 | tau6 | wrist_3_joint | rad; joint-side Nm |

`t` is the exact simulation clock in seconds. The effort is Pinocchio RNEA
on the truth model, checked independently (fixture report, section 2). No
current-to-torque conversion, gear ratio or sign change applies. The files
also hold the analytic `dq0..dq5` and `ddq0..ddq5`; the example ignores
them, and the truth benchmark (`identification_truth.py`) uses them.

## Timing, differentiation and filtering

The unified config sets **100 Hz** (`ts = 0.01 s`), the fixture's clock. The
loader requires `t` to be uniform at that `ts` and the filter sample rate to
equal `1/ts`; it performs no resampling.

Velocities are forward differences of positions `i` and `i+1`, centred at
`(i+0.5)*ts`; accelerations are differentiated on these interval centres
(core `calculate_first_second_order_differentiation`). The loader keeps the
first `n-2` rows of positions, efforts and clock; the half-step offset of
the velocities is explicit, not interpolated.

The base pipeline then filters positions, velocities and accelerations
separately: median window 5, then zero-phase Butterworth order 4, cutoff
10 Hz, sample rate 100 Hz. The motion's content is below about 3 Hz.
Effort is not filtered. Filter edges remain in the fit.

## Samples and validation

| Split | Source rows | Returned rows |
|---|---:|---:|
| training | 2400 | 2398 |
| validation | 800 | 798 |

There is no sample cap: every row is used. The default CLI uses
`solve(decimate=False)`. Validation is a separate trajectory of a different
family (a Fourier series, not the optimised training splines), so validation
metrics are held out. `trajectory_provenance`, keyed by absolute file path,
records the source rows, the retained index range, the clock and units.

## Fingerprints

SHA-256 of the data files is in `truth/manifest.json` (`files`), checked by
`tests/test_ur10_identification_truth.py`.

# UR10 hand-eye calibration data

`calibration.csv` is the UR10's only calibration recording: 23 postures of
one session. Its roles, hash and fold settings are frozen in
[`hand_eye_protocol.yaml`](hand_eye_protocol.yaml); the design is described in
the [hand-eye note](../../../docs/development/ur10-hand-eye.md).

## Columns and frame convention

| Columns | Meaning | Units |
|---|---|---|
| `x1, y1, z1` | position of the measured frame | m |
| `phix1, phiy1, phiz1` | orientation of the measured frame, roll-pitch-yaw | rad |
| `shoulder_pan_joint` ... `wrist_3_joint` | joint angles, selected by name | rad |

The file is fitted as core's `T = B * FK(q) * T_tip`, so the six pose columns
are the moving frame (the camera, rigid on the flange) in a fixed frame (the
board, or the frame the poses were pre-expressed in). The fitted base
parameters are the robot base in that frame; they come out small (at most
6.4 cm and 1.4 deg from identity), so the poses are already expressed close
to the robot base frame, and how that was done is unknown. Only the product
of the base and tool frames with the joint errors is observable, which is why
the frame parameters carry a gauge (see the note).

## Known facts and defects

- Postures: 23, one session, joints within the ranges recorded in the
  protocol's strata; the nearest posture pair differs by at least 0.7 rad on
  some joint, so no posture is repeated and the file gives no estimate of the
  measurement noise.
- Camera, board geometry, detection method and the recording date: unknown.
- Orientation noise and the depth (camera z) accuracy: unknown. The
  simulated study assumes values and says so.
- There is no second session: held-out evidence is a k-fold split of the
  rows, which is weaker than a held-out session.

`simulation.csv` (29 postures, same columns) has no recorded provenance and is
not used by the hand-eye study.
