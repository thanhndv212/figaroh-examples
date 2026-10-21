# TIAGo Experimental Data

The `suspension/` and `backlash/` directories contain historical offline logs
copied verbatim from the reachable `figaroh-plus` Git ref at commit
`2218d77638e0148afd3b358fa51702f8b82f4100`.

- `suspension/tiago_xyz_vicon_1640.csv` is a Vicon and force-plate recording
  (the other ten runs of the day are described in the next section).
  The example reconstructs the base-marker frame from markers `base1`, `base2`,
  and `base3`, subtracts the initial base pose and initial 100-sample wrench
  mean, then fits the fixed-transform generalized-base model. Its reported
  result is an experimental data fit, not a validated deployable suspension
  parameter set.
- `backlash/sinus_amp3_period10_2023-07-24-13-28-42/` contains the PAL
  introspection name/value exports. By default the example fits
  relative-minus-absolute encoder difference using relative position,
  velocity direction, and a Pinocchio-computed generalized gravity torque
  (`load_backlash_joint_trajectory_with_gravity`) as the load feature,
  reconstructed from every logged arm/torso/head joint position at each
  sample and evaluated against `urdf/tiago_48_hey5.urdf`. This reproduces
  the historical investigation's `tau_g` feature. Logged motor effort
  (`--load-feature effort`) is kept only for comparison: it is heavily
  quantized on the wrist joints (arm_5_joint..arm_7_joint, tens of unique
  values across thousands of samples) and was the source of earlier
  rank-deficient fits there. Two joints still need a lower polynomial
  degree than the rest even with the gravity feature: arm_1_joint's
  rotation axis does not couple to gravity (its gravity torque is
  identically ~0, so `backlash_empirical_surface.py` backs off to
  degree 0 there), and arm_2_joint's gravity torque is only moderately
  independent of its own position (backs off to degree 4).

Both artifacts are research data. Do not use them as a model-parameter source
without checking hardware identity, sensor calibration, coordinate conventions,
and the intended use approval.
## Backlash sinusoid sessions (`backlash/`)

Two days of the same sinusoidal protocol (period 10 s, amplitude 3), for the
backlash transfer study
([#109](https://github.com/thanhndv212/figaroh-examples/issues/109), for
[#71](https://github.com/thanhndv212/figaroh-examples/issues/71)). Roles are
frozen in `backlash/protocol.yaml` (`tiago-backlash-sinus-2023` v1).

| Session | Robot | Role | Content |
|---|---|---|---|
| `2023-07-24-13-28-42` | tiago-48 | training | 12,092 samples, 121 s, all 209 channels |
| `2023-03-09-16-19-17-bottom` | tiago-48 | validation | 41,953 samples, 420 s, 33 channels |
| `2023-03-09-15-28-20-top` | unknown | diagnostic | `tf.csv` only, no joint values |

- **Provenance:** the 2023-03-09 recordings are original ROS bags and an
  exporter `tf.csv` (not distributed), named and hashed in `protocol.yaml`.
  The **2023-07-24 CSV pair is untraced**: it was shipped before this protocol
  and no raw bag exists to match it by hash or value, so its provenance is the
  historical export only. Its role (training) is a choice, not a trace.
- **Layout:** `introspection_datanames.csv` and `introspection_datavalues.csv`
  as in the 2023-07-24 export: the names file holds the first channel name in
  the `names` cell and the rest in the header with a `- ` prefix; the values
  file has one row per message with the channel list as text and the
  `names_version` column. The loader
  (`load_backlash_joint_trajectory`) reads both runs unchanged: it skips the
  first value, which has no name in the header. The 2023-03-09 bottom run
  keeps only 33 of the 209 channels (the leading statistics channel and
  position, velocity, effort and absolute encoder of `torso_lift` and
  `arm_1..arm_7`), values unchanged, because all 209 would make a 123 MB file.
  The other 176 channels (motor-side, temperatures, controller
  state, head, hand) are not shipped.
- **Units and clock:** radians, rad/s and N m for the arm, metres for
  `torso_lift`. The clock is the **header stamp** (`secs`, `nsecs`), the robot
  clock. `rosbagTimestamp` is the recording PC's clock and differs from it by a
  per-session offset (64 s on 2023-03-09, 2454 s on 2023-07-24); do not use it.
  Samples arrive at about 100 Hz.
- **What moves in `imu_test` (raw name for 2023-03-09):** the raw README calls
  it a "sinusoidal torso test". The bottom bag shows otherwise. All seven arm
  joints move sinusoidally from about 10 s to 408 s (ranges 1.4 to 3.0 rad,
  peak speeds 0.19 to 0.43 rad/s). The torso does **not** oscillate: it steps
  between plateaus (0.04 to 0.35 m, 0.31 m span, about 0.03 m/s) at roughly 80, 160, 220,
  280 and 360 s. The head and the gripper stay still. The top run's `tf` agrees:
  every arm frame rotates, `torso_lift_link` translates by 0.31 m, head frames
  stay put.
- **Top run (`tf.csv`, pose only):** the exporter's `/tf` dump, 43,529 messages
  over 453 s, shipped as found (52.7 MB, one row per message, 17 fields per
  transform). The exporter dropped the YAML list markers, so the header names
  are shifted and unusable; `read_tf_csv` in
  `examples/tiago/utils/introspection_extraction.py` reads it by position.
  There are no joint values or encoder channels: arm joint angles can only be
  recovered from the transform rotations.
- **Robot identity:** the bottom run is `tiago-48` by the encoder-gap
  fingerprint: median position minus absolute-encoder gap on arm_1, arm_2 and
  arm_7 is -3.7, -5.3 and +2.3 mrad, equal to the 2023-07-24 run (-3.6, -5.2,
  +2.7) and inside the range of the tiago-48 Hey5 bags of 2023-11 to 2024-02
  (-3.8 to -4.6, -4.1 to -6.9, +3 to +7). Caveats: both backlash days carry
  gripper channels, not the Hey5 hand channels of the later bags, so the end
  effector was different; and no bag of the other robot (155c) is available to
  contrast, so the fingerprint supports but does not prove the identity. The
  top run has no encoder channel and is `unknown`. 2023-07-24 is listed as
  tiago-48 on the same evidence.
- **Limits:** arm_5 relative minus absolute differs by about +111 mrad on
  2023-03-09 and +24 mrad on 2023-07-24 (a session offset, not backlash), so
  a surface fitted on one day does not transfer to the other for arm_5 and
  arm_6 without removing it. The two runs differ in length (420 s against
  121 s) and in torso height history.
- **Re-creating the files:**
  `python -m examples.tiago.utils.introspection_extraction export <bag> <out_dir>`
  writes the two CSVs;
  `tests/test_tiago_introspection_extraction.py` re-creates them byte for
  byte when `TIAGO_RECORDINGS_ROOT` is set.

## Arm_5/arm_6 joint-inspection recordings (`joint_inspection/`)

Bench recordings of the arm_5 and arm_6 joint encoders, for the backlash
transfer study
([#110](https://github.com/thanhndv212/figaroh-examples/issues/110), for
[#71](https://github.com/thanhndv212/figaroh-examples/issues/71)). They bear
on a joint offset that changes by session, absolute-encoder jumps, backlash
and a scale error between the two encoders. Every session is **diagnostic**,
frozen in `joint_inspection/protocol.yaml`
(`tiago-joint-inspection-2023-2024` v1): they test whether the backlash
surface explains the encoder differences, and nothing is fitted or scored on
them.

| Folder | Bag dates (UTC) | Bags | Motion |
|---|---|---|---|
| `2023-12_arm6_inspection/` | 2023-12-05, 2023-12-06 | 4 | arm_6 sweeps 2.73 to 2.79 rad, all else still |
| `2024-01-25_arm6_backlash/` | 2024-01-25, 2024-01-26 | 6 | arm_6 vertical (2), horizontal, step; 2 manual backlash probes |
| `2024-02-02_arm5_positions/` | 2024-02-02 | 14 | arm_5 held near fixed angles (the angle is in the file name) |

- **Not usable:** the **2023-10-25** joint-encoder inspection survives as
  photos and screenshots only. No bag, CSV or other recording exists, so
  nothing ships for it. Photos and videos of the other sessions are not
  shipped either.
- **Files:** one CSV per bag, named after the bag, native 100 Hz, all rows
  (about 44 MB in all). Columns: `t`, then for `arm_5` and `arm_6`:
  `*_position` (relative: motor-side encoder divided by the gear ratio, what
  control and calibration use), `*_absolute` (output-side absolute encoder)
  and `*_command` (commanded position, recorded in every bag). Radians.
  Relative minus absolute is the transmission error: backlash, compliance,
  offset and scale.
- **Clock:** `t` is seconds from the first sample on the introspection
  **header stamp**, the robot clock, one clock for every column. The bag
  receive time (`rosbagTimestamp`) runs 10 to 16 s behind it and is not used.
  `protocol.yaml` records both start times per bag.
- **Dates and names:** dates are the bag stamps (UTC). Folder and file names
  carry the recording PC's local time (UTC+1 in winter), so a name's hour is
  one more than the bag's. The two `arm_6_backlash_abs_inspect` bags are
  stamped 2024-01-26 but sit in the 2024-01-25 folder. `arm5_minus1_59` was
  recorded an hour before the other 2024-02-02 bags; its stamp agrees with
  its name.
- **Counts:** the raw-data audit counts 15 arm_5 bags; 14 exist. All 24 bags
  are shipped.
- **Setup, in words:** the robot stood on the bench with the arm free, in
  position control (`motor_mode` 0). In the arm_6 sessions arm_6 was driven
  over the range given above (vertical and horizontal arm orientations, one
  step profile) with the other joints holding their pose. In the two manual
  backlash probes the arm_6 motor is held still (relative encoder moves 0.4
  mrad peak to peak) while the output is moved by hand and the absolute
  encoder moves 38.3 and 42.2 mrad: the free play, measured directly. In the
  arm_5 sessions arm_5 was brought to fixed angles; in these bags arm_6 also
  travels 2.77 rad (peak to peak) and arm_5 itself moves in some bags (up to
  2.65 rad in `arm5_0_8`).
- **Known limits:** the median relative minus absolute gap of arm_5 changes
  with the session, from -92 mrad (`arm5_minus1_59`) through +25 to +35 mrad
  (2023-12, 2024-01) to +111 to +156 mrad (2024-02, arm_5 moving). The
  absolute encoder jumps in places. These recordings are single runs
  without repeats, so a gap difference between two of them is an observation,
  not a measured spread.
- **Robot identity:** the 2024-01 and 2024-02 bags are `tiago-48` by the
  encoder-gap fingerprint (median position minus absolute encoder on arm_1,
  arm_2, arm_7: -3.1 to -4.6, -5.7 to -6.9, +0.8 to +7 mrad), the same as the
  Hey5 bags of 2023-11 to 2024-02, and all carry the 423 Hey5 channels. The
  four 2023-12 bags carry the same channels, but their arm_1 (-17.6 to -2.2
  mrad) and arm_2 (-1.7 to +2.1) gaps do not match, so they are `unknown`.
- **Re-creating the files:**
  `python -m examples.tiago.utils.introspection_extraction inspection <bag> <out.csv>`;
  `tests/test_tiago_introspection_extraction.py` re-creates all 24 byte for
  byte when `TIAGO_RECORDINGS_ROOT` is set.

## CREPS Vicon and force-plate runs (`suspension/`)

Eleven runs recorded on 2023-09-20 at the CREPS Vicon room with a force plate
under the robot, for the base-suspension study
([#111](https://github.com/thanhndv212/figaroh-examples/issues/111), for
[#70](https://github.com/thanhndv212/figaroh-examples/issues/70)). Roles are
frozen in `suspension/protocol.yaml` (`tiago-creps-vicon-2023` v1). Ten of
the files are new; `tiago_xyz_vicon_1640.csv` was already shipped.

| Run (`tiago_<name>_vicon_<hhmm>.csv`) | Local start | Rows | Role | Motion |
|---|---|---|---|---|
| `xyz_1640` | 16:40 | 5,547 | training | xyz oscillation |
| `xyz_1628` | 16:28 | 5,337 | validation (first held-out repeat) | xyz oscillation, repeat |
| `xyz_mirror_1630`, `xyz_mirror_1642` | 16:30, 16:42 | 5,607, 5,416 | confirmation | xyz oscillation, mirrored |
| `around_x_1634`, `around_y_1638`, `around_z_1622` | 16:35, 16:39, 16:26 | 5,123, 5,305, 6,150 | diagnostic | rotation about x, y, z |
| `x_fold_1632`, `y_fold_1636` | 16:33, 16:37 | 7,361, 5,873 | diagnostic | fold about x, y |
| `postures_1720`, `postures_1734` | 17:22, 17:34 | 13,706, 9,918 | diagnostic | posture sequences, base at rest |

The roles beyond the two fixed by the issue (1640 training, 1628 first
held-out) are a proposal by motion type, open to the maintainer: the mirrored
repeats are the same motion family, the single-axis rotations and folds and
the static postures are different motions, useful to see what a fit does not
transfer to.

- **Layout:** all eleven files have no header, LF line ends and 40 columns:
  `time`, the x, y, z (m, Vicon world) of `base1..3`, `shoulder1..4` and
  `gripper1..3`, then force plate 4: `Fx4, Fy4, Fz4` (N), `Mx4, My4, Mz4`
  (N m) and `COPx4, COPy4, COPz4` (m). Ten of the sources had a header, of two
  kinds (20 names, one per marker triple; 40 names in the two
  postures runs) and CRLF line ends. The extractor selects the columns by
  name; every field is copied as text, so no value is changed. The example
  loader `load_vicon_forceplate_trajectory` reads all eleven.
- **Text precision:** `xyz_1640` was written with 15 significant digits, the
  others with 16 or 17. The text is kept as found, so the files differ in
  length for the same data.
- **Clock:** `time` is the **host receive time** of each frame
  (`YYYY-MM-DD_HH:MM:SS.ffffff`, recording PC local time), not the Vicon frame
  time. Spacing is about 10 ms with gaps up to 65 ms and jitter, so a time
  rebuilt from receive stamps is noisy. There is **no frame-index column** in
  any source (the issue asks for indices: none were recorded); the row order
  is the frame order, so `n x 10 ms` is the rebuilt clock. Frames the
  recorder dropped or duplicated are not marked. The audit counts 1.5 to 2.5 %
  dropped and 0.5 to 0.8 % duplicated, judged from position continuity.
- **Frames and points:** markers are in the Vicon world frame; the plate
  moments are about the plate centre (0.900, 0.450, 0) m, not the base. The
  base-marker triad frame is yawed about -5.6 degrees from world and its z axis
  points down (the audit's reading of the example's frame definition). The
  example's model treats these points loosely; fixing that is #70, not this
  data.
- **Static load:** Fz is about -730 N at rest in every run (74 kg on the
  plate); the first 100 samples are the example's zero.
- **Robot side:** the nine oscillation runs each have a robot introspection
  bag recorded the same day (bag stamp UTC 14:xx, two hours behind the local
  16:xx of the Vicon log), and the two postures runs have joints-only bags
  stamped 15:22:11 and 15:34:35 UTC beside the geometric-calibration
  recordings (they match 17:22 and 17:34 local). None is aligned to the Vicon
  clock, and none is shipped here.
- **Robot identity: unknown** for all eleven. A Vicon and force-plate log has
  no robot channel, and the encoder-gap fingerprint of the day's robot bags
  does not settle it: arm_7 reads -13 to -16 mrad in the oscillation bags and
  +4 mrad in the 14:08 calibration bag of the same day, so the gap follows the
  posture, not only the robot.
- **Known limits:** base motion is small (marker `base1` moves 0.2 to 2 mm in
  x and z; y up to 16.6 mm in `around_z`, 8 to 13 mm in the xyz runs). The
  audit's fits of the example model on these runs explain the forces poorly
  (Fx, Fy, Fz R-squared below 0.2 in most runs), which is a model limit.
- **Re-creating the files:**
  `python -m examples.tiago.utils.suspension_extraction <raw.csv> <out.csv>`;
  `tests/test_tiago_suspension_runs.py` re-creates all eleven byte for byte
  when `TIAGO_RECORDINGS_ROOT` is set (`xyz_1640` is byte-identical to its
  source).

## Dynamic identification data (`identification/dynamic/`)

Audited in [#20](https://github.com/thanhndv212/figaroh-examples/issues/20);
full report: [`docs/development/tiago-signal-audit-2026-10-03.md`](../../../docs/development/tiago-signal-audit-2026-10-03.md).
The raw files are kept unmodified.

| File | sha256 (prefix) | Columns |
|---|---|---|
| `tiago_position.csv` | `4297a60d434ee89f` | `t`, `- <joint>_position` |
| `tiago_velocity.csv` | `269f681ca7a9511a` | `t`, `- <joint>_velocity` |
| `tiago_effort.csv` | `12c635987705efad` | `t`, `- <joint>_effort` |

Joints, in model order: `torso_lift_joint`, `arm_1_joint` … `arm_7_joint`.

- **Clock:** column `t` is recorded and identical in all three files: 8022
  rows, 0–80.21 s, median step 9.997 ms (~100 Hz), jitter 7.7–12.3 ms,
  strictly increasing, no gaps. Filters must be designed at this rate.
- **Velocity:** a logged first-order filtered position derivative:
  `v[n] = 0.95*v[n-1] + 0.05*Δq/Δt` on the recorded clock (0.195 s time
  constant; relative residual 1.4–1.6e-4 on training and `calibration_slow`,
  recorded as `velocity_filter_residual` in the run provenance). It holds
  nothing the positions do not, so `identification.py` derives the velocity
  from the filtered positions (#68). `--velocity-source measured` restores
  the earlier shifted channel: its estimated shift (~18 samples on training)
  only approximates the filter's phase. Held-out RMSE on `calibration_slow`,
  with all eight joints' effort fitted (before the fit was restricted to
  arm_1–arm_4, see Effort):

  | Velocity | Fit RMSE | Held-out RMSE | Condition |
  |---|---|---|---|
  | from positions (default) | 0.643 | 1.317 | 1564 |
  | logged, shifted (`measured`) | 0.666 | 1.209 | 1779 |
  | logged, filter inverted | 0.809 | 1.304 | 1695 |

  The shifted channel predicts the slow run better although it is the
  less faithful signal; the default follows the measured physics, and the
  maintainer accepted the held-out cost (#68). Delaying the effort by 3–18
  samples or lowering the cutoff to 0.8 Hz did not recover it. The raw
  exports preserve the logged signal unchanged.
- **Effort:** raw values converted in `process_torque_data` with the
  per-joint `reduction_ratio × kmotor` from the `drives` table in
  `config/tiago_unified_config.yaml` (with a source per joint), plus
  `9.81 × subtree mass` on the torso. Units, signs and constants are
  documented assumptions, not verified against a torque reference. The
  wrist efforts (`arm_5`–`arm_7`) are exactly zero on 88–90% of samples
  (step 0.001), so wrist dynamics are weakly observable. The torso force is
  ~99% the URDF's own `m g` term (its `ratio × kmotor = 1` is undocumented).
  Only arm_1–arm_4 effort is therefore fitted and scored
  (`problem.torque_fit_joints` in the config, #68); torso and wrist motion
  still enter the regressor. Held-out RMSE on `calibration_slow`:

  | Effort fitted | Base parameters | arm_1 | arm_2 | arm_3 | arm_4 | Pooled arm_1–arm_4 | Pooled arm_2–arm_4 |
  |---|---|---|---|---|---|---|---|
  | arm_1–arm_4 (default) | 57 | 1.310 | 2.540 | 1.471 | 1.237 | 1.722 | 1.839 |
  | all eight joints | 73 | 1.316 | 2.504 | 1.450 | 1.237 | 1.706 | 1.817 |
  | arm_2–arm_4 | 53 | – | 2.583 | 1.480 | 1.248 | – | 1.864 |
  | nominal model | – | 3.187 | 7.838 | 3.122 | 2.858 | 4.730 | 5.143 |

  Leaving the torso out changes nothing on the arm; leaving the wrist out
  costs ~1%. The gain is that no parameter is fitted to a signal known to be
  unreliable. Fit RMSE is 0.884 on the arm rows (0.643 when all eight joints
  were fitted and pooled).

  arm_1's torque constant (0.136, the same as arm_2) cannot be sourced or
  estimated from these recordings. PAL's gravity_compensation controller
  sets 0 for arm_1 because its axis is vertical and it carries no gravity
  load. That also means its effort has no gravity term to anchor the scale.
  Scaling arm_1's constant by 0.25–1.5 leaves arm_2–arm_4 held-out RMSE
  between 1.838 and 1.841, and arm_1's own error stays at 42–48 % of its
  signal, because arm_1's base parameters absorb the scale. The payload run
  adds only 0.10 N·m RMS on arm_1, against about 1.3 N·m of model error, so
  it cannot calibrate the constant either. arm_1 stays in the fit, because
  leaving it out makes arm_2–arm_4 worse (1.864). Its identified parameters
  and its absolute RMSE carry the unknown scale, so compare runs on the
  arm_2–arm_4 column.
- **End effector:** a Hey5 hand (the recordings log its `hand_*` joints and
  no gripper), so `identification.py` loads `urdf/tiago_48_hey5.urdf`. This
  applies to all three 2021-07 recordings below. The wrist F/T sensor weighs
  0.792 kg below the sensor, lighter than both the Hey5 (1.032 kg) and the
  Schunk (0.865 kg) URDFs, so the hand is identified from the logged
  channels, not from the mass (#68). The
  switch from the Schunk URDF (measured with the shifted velocity) left the
  fit and held-out RMSE (1.209) and the 73 base parameters' count unchanged; 30 base-parameter values
  shifted to absorb the hand's inertia, and the nominal model's held-out
  RMSE moved from 3.375 to 3.407.
- **Evidence:** the findings above (velocity filter, arm_1 constant, torso
  levels, differential wrist, end effector) are reproduced from shipped files
  by `identification_inputs_audit.py` with the channels in
  [`identification/audit/`](identification/audit/README.md); report:
  [`tiago-identification-inputs-2026-10-10.md`](../../../docs/development/tiago-identification-inputs-2026-10-10.md).
- **Window:** `identification.py` keeps rows 921–6791 (9.21–67.90 s), the
  excitation; RMS velocity 0.159 rad/s inside vs 0.017 / 0.006 rad/s before /
  after.

`tiago_bp_19_Oct_2024_2320.csv` and `tiago_nov_30_64.csv` are not used by
`identification.py` and were not audited.

### Independent recordings (identification #69)

`identification/calibration_slow/` and `identification/calibration_weight/`
each contain `tiago_{position,velocity,effort}.csv` in the same columns and
joint order as `dynamic/`. All three directories also hold
`tiago_wrist_ft.csv`: `t` and `wrist_ft_{force,torque}_{X,Y,Z}` (N, N·m,
sensor frame) on the same clock, used by `payload_check.py`. All samples are preserved. Each triplet shares
one finite, strictly increasing header clock, rebased to zero, at ~100 Hz.

| Directory | Recording (UTC, 2021-07-01) | Rows | Frozen role |
|---|---|---|---|
| `dynamic/` | 13:02, `calibration.bag` | 8022 | training; rows [921, 6791) |
| `calibration_slow/` | 12:57, `calibration_slow.bag` | 14163 | validation; same path at roughly half speed |
| `calibration_weight/` | 13:27, `calibration_weight.bag` | 7547 | changed-payload diagnostic; same path, added load |

The slow run is configured by default. To inspect the payload diagnostic:

```bash
cd examples/tiago
python identification.py --validation-data data/identification/calibration_weight
```

These runs were already inspected in the source audit; they are not unseen
test sets. Do not tune on them and then claim independent acceptance.
The payload run changes the system mass and is not an unchanged-model
prediction acceptance set. Raw efforts remain unverified controller values:
the torso force conversion is unsupported and wrist effort quantisation
prevents reliable identification there; both are left out of the torque fit.
The arm_1 constant is unverified and not identifiable from these runs (see
Effort above). Shipping these files does not resolve those modelling
limits or validate the identified physical parameters.
The CLI refuses prediction acceptance for the frozen payload or training
recording, recognising their hashes even if the CSV directory is renamed.

`python payload_check.py` compares the payload mass derived from the arm_2–arm_4
efforts with the wrist F/T sensor's (0.489 kg). With the current drive
constants the efforts read 13.5 % low, outside the ±10 % tolerance justified
in the protocol document.

Source bag names/hashes and exported file hashes are frozen in
[`identification/protocol.yaml`](identification/protocol.yaml). The bags
are original recordings, not distributed; retain the Thanh Nguyen / CNRS /
Toward attribution. Running the examples needs only the shipped CSVs. Extraction instructions and freeze rules are in
[`tiago-identification-cross-run-protocol.md`](../../../docs/development/tiago-identification-cross-run-protocol.md).

## Mocap calibration data (`calibration/mocap/`)

Rebuilt from the raw Qualisys recordings in
[#67](https://github.com/thanhndv212/figaroh-examples/issues/67). The C1 audit
([#24](https://github.com/thanhndv212/figaroh-examples/issues/24),
[report](../../../docs/development/tiago-mocap-calibration-audit-2026-10-04.md))
covers the file these replace.

| File | Role | Session | Postures | sha256 (prefix) |
|---|---|---|---|---|
| `qualisys_2021-11-30_static_postures.csv` | training (`source_file`) | `calib_mocap_2021-11-30-15-44-33` | 37 | `b6c0051e20c6a077` |
| `qualisys_2021-11-26_static_postures.csv` | validation (`validation_data_file`) | `calib_mocap_2021-11-26-11-05-59` | 62 | `7c986df711757c4d` |
| `qualisys_2021-11-30-1403_static_postures.csv` | confirmation (not in any config) | `calib_mocap_2021-11-30-14-03-27` | 63 | `e44c678fbbc1fbc8` |
| `qualisys_2021-11-30-1504_static_postures.csv` | confirmation (not in any config) | `calib_mocap_2021-11-30-15-04-05` | 59 | `cc19eaa74fe8b74b` |

Roles, freeze rules and results are fixed in the held-out protocol
([`docs/development/tiago-mocap-heldout-protocol.md`](../../../docs/development/tiago-mocap-heldout-protocol.md),
#27). Do not tune anything on the confirmation sets. The same roles and
hashes are machine-readable in `calibration/mocap/protocol.yaml`, a
data-contract `Protocol`
([adapters](../../../docs/development/data-contract-adapters.md), #17).

**Columns:**
- `x1,y1,z1 … x4,y4,z4`: the points BL, BR, TR, TL of the Qualisys hand
  rigid body, in metres. They are virtual points of one tracked body, not four
  independent marker measurements: inter-point distances are constant to
  < 1 µm and identical on both days. So the four points carry the body's 6D
  pose.
- The eight joint positions: `torso_lift_joint` (m), `arm_1_joint` …
  `arm_7_joint` (rad).
- `t_start_robot`, `t_end_robot`: the averaging window, robot clock (s).
- `marker_std_mm`: the largest marker standard deviation over that window.
- `shipped_row` (training file only): the matching row of the replaced file,
  or −1 for a posture that file did not have.

**How the rows were built:**
- **Static plateaus:** one row per period where every joint stays within
  1 mrad over 0.5 s, lasting at least 2 s.
- **Joints:** averaged over [start + 0.5 s, end − 0.3 s].
- **Clock correction:** markers are averaged over the same physical interval
  on the mocap clock, which runs 3.9 s (Nov-30) and 2.6 s (Nov-26) behind the
  robot clock. Each lag was estimated by cross-correlating joint and marker
  speed.
- **Frame:** markers are expressed in the Qualisys `base_frame` rigid body,
  which is fixed to the robot base. Both sessions share it, so a model fitted
  on one day can be scored on the other without re-registration.
- **Noise:** the body's point standard deviation per plateau is 0.2 mm
  (median).

The original ROS bags are not distributed. The rules above are implemented
and tested in `examples/tiago/utils/mocap_extraction.py`
(`tests/test_tiago_mocap_extraction.py`); `read_qualisys_bag` reads an
original bag when the optional `rosbags` package is installed.

**Used by calibration:** marker 1 (BL), position only (`measurable_dof` xyz),
expressed as a point fixed in `wrist_ft_tool_link`. Core supports one marker
per sample (`NbMarkers == 1`), so points 2–4 are unused; fitting all four
(or the 6D pose) is figaroh-plus#119. Since all four come
from one body pose, a 6D pose measurement would carry the same information. The end effector
for these sessions was not recorded; calibration loads `tiago_48_schunk.urdf`,
and only the estimated tip offset depends on that choice.

**Reference result:** `calibration.py`, `calibration_level: joint_offset`,
base and tool estimated, no regularisation, figaroh-plus `devel` with #101,
#105 and #102. Core drops the torso and arm_1 offsets, which the estimated
6D base absorbs (vertical axes), so 14 parameters remain and the problem is
full rank (condition number 62.2, as `calibration.py` reports it).

| Training RMSE | Held-out RMSE (Nov-26) | Held-out max | arm_5 offset |
|---|---|---|---|
| 2.88 mm | 4.23 mm | 12.0 mm | −49.8 mrad |

The template's regularisation coefficient of 0.01 shrank arm_5 to
−37.5 mrad and raised held-out error to 4.35 mm. About 3–4 mm of residual
remains in every Nov-2021 session after calibration, which is not geometric:
it is the practical floor for this setup.

The held-out sessions mostly replay training configurations. Of each set's
59–63 postures, 35–37 are training postures, 16–17 are new but inside the
training joint ranges, and 8–9 are outside them. On the new in-range
postures, held-out RMSE is 4.1–4.5 mm for `joint_offset`, 4.7–5.1 mm for
registration only, and 2.6–2.8 mm for `full_params` (31 parameters on macOS,
30 on Linux; figaroh-plus#110, #113). Outside the training ranges
registration only and `joint_offset` are at 6–7 mm, `full_params` at
3.8–5.3 mm. See the protocol for the full table.

**Deployment:** at `joint_offset` level, `calibration.py` writes an empty PAL
`master_calibration.yaml` (`geometric_calibration: {}`), because the PAL
export carries only `full_params` placement corrections. Mapping joint
offsets to PAL's `arm_k_joint_offset` entries is #28 (C3). The exported URDF
(`update_model.py`) does contain the offsets, written into each joint's
`<origin>` (figaroh-plus#101).

**Sessions not shipped:** the protocol's inventory lists the other
recordings and why they are not used. In particular, the 2023-11-07 OptiTrack
eye-hand runs are unusable for kinematics: the chessboard rigid body flips
(~170° in 13 of 16 postures).

### 2021-10-26 static-posture sessions (confirmation, world frame)

Two earlier Qualisys sessions
([#104](https://github.com/thanhndv212/figaroh-examples/issues/104)), a month
before the files above. Same sensor (Qualisys, 100 Hz, `/tf` under the
`Qualisys` frame), same robot, same static-posture plan run twice; another
hand rigid body than November (one plate, inter-point distances 171.4, 132.5,
195.8, 165.4, 106.0 and 108.7 mm; identical in both sessions, but not the
November body, so its tool point is a different one).

| File | Role | Session | Postures | Joint offsets | sha256 (prefix) |
|---|---|---|---|---|---|
| `qualisys_2021-10-26-1051_static_postures.csv` | confirmation (not in any config) | `calib_mocap_2021-10-26-10-51-07` | 26 | applied | `db245498e1bdc023` |
| `qualisys_2021-10-26-1102_static_postures.csv` | confirmation (not in any config) | `calib_mocap_nooffsets_2021-10-26-11-02-56` | 25 | not applied | `94685056aac421aa` |

- **Frame:** the points are in the **Qualisys world frame**, not the
  `base_frame` body. The base body was in view in only 7 of 26 plateaus (first
  session) and 0 of 25 (second), so there is no base pose to express them in.
  The base pose is a free registration parameter of any fit. Nothing is
  borrowed from the other session. The extra column `points_in_base_frame` is
  0 in every row (it is 1 for a file in the `base_frame` body); the November
  files have no such column and are all in the base body.
- **Columns:** as above, without `shipped_row`, plus `points_in_base_frame`.
- **Extraction:** the same rules as November (plateaus, averaging window,
  clock lag), run with `--frame world`. Joints come from the PAL introspection
  topic, like every other session. Mocap and robot clocks agree to 0.2 s
  (lag −0.21 s and −0.19 s, speed correlation 0.93 and 0.90), much closer
  than in November (2.6–3.9 s). Marker noise per plateau: 0.8 mm at most.
- **Protocol:** `calibration/mocap/protocol.yaml` v2 lists both as
  `confirmation` sessions with the sha256 of the original bag. They are in
  `heldout.not_evaluated`: the base-frame protocol and its v1 results do not
  include them. Fitting them is the motion-capture study
  ([#100](https://github.com/thanhndv212/figaroh-examples/issues/100)).
- **Registration-only sanity fit** (nominal joints, base 6D and one tool point
  3D fitted on the session itself, point BL; the model of the v1 "registration
  only" row, which gives 3.37 mm on the November training session): 4.93 mm
  RMS (26 postures, max 13.8 mm) and 5.14 mm (25 postures, max 16.1 mm). The
  residual is larger than November's because no joint offsets are fitted.
  `tests/test_tiago_mocap_2021_10_26.py` reproduces both numbers.
- **Joint offsets loaded that day:** the recordings hold the introspection
  and `/tf` topics only, not the controller parameters, so the offset *values*
  are not recorded. What the data shows: at the same joint readings (24
  postures agree to < 2 mrad on every joint), the absolute encoders of the
  two sessions differ by −19.3 mrad on arm_6 and +20.5 mrad on arm_7 (first
  minus second session) and by < 0.2 mrad on arm_1 to arm_4, and the torso has
  no absolute encoder. arm_5 agrees to −0.2 mrad with a 2 mrad spread. The
  offset sets of the two sessions therefore differ by about ±0.02 rad on
  arm_6 and arm_7, which is consistent with the ±0.02 rad entries of a PAL
  calibration file. The `/tf` joint transforms are identical to the nominal
  URDF origins in both sessions, so the offsets are not in the kinematic tree.
  The absolute set of either session, and the sign convention, are not
  confirmable from the bags. The first-session file is the "offsets applied"
  one by the session names only.
- **Known defects:** the second recording has 26 plateaus; the last one
  falls after the mocap stream ended (15 s), so it has no row (25 rows). The
  two sessions share 24 postures (within 2 mrad on every joint); the first
  session's first and last postures and the second session's first have no
  counterpart. The continuous-motion run and the mocap-only recordings of the
  same day are not shipped. The `tf.csv` exports that accompany the original
  recordings have shifted columns and truncated precision and were not used;
  everything is extracted from the bags. Offline fits on both sessions (joint
  offsets fitted, not shipped) gave arm_5 at −76 to −84 mrad, in line with
  November, and 3.4 mm training RMSE; reproducing them is for #100.
- **Extraction:** `python -m examples.tiago.utils.mocap_extraction BAG OUT
  --lag LAG --frame world --expected-sha256 SHA` from the repository root
  (`rosbags` required). The bag name, sha256 and lag of each session are in
  `protocol.yaml`.
  `tests/test_tiago_mocap_extraction.py` re-creates both files byte for byte
  when `TIAGO_RECORDINGS_ROOT` points at a folder holding the original
  recordings; it is skipped otherwise.

**Superseded file, kept unmodified and unused by any config:**
`qualysis_base_hand_calibration.csv` (34 postures, sha256 prefix
`d7fcbd96e3e67319`). It came from the same
session, but joints and markers were paired by raw timestamp with the 3.9 s
clock offset uncorrected. In 16 of 34 rows the marker sample was taken after
the arm had started moving, with errors up to 7.8 mm (row 29 is the worst).
Its "marker 4" repeated marker 3, so TL was missing.

## OptiTrack calibration data (`calibration/optitrack/`)

Three static-posture runs recorded on 2023-09-12
([#105](https://github.com/thanhndv212/figaroh-examples/issues/105)) with an
OptiTrack system, extracted from the original recordings. They are
sessions of their own protocol (`calibration/optitrack/protocol.yaml`,
`tiago-optitrack-2023-09-12` v2, independent of the Qualisys protocol). The
motion-capture study
([#100](https://github.com/thanhndv212/figaroh-examples/issues/100)) fixed the
roles in v2: 1658 `training`, 1552 `confirmation`, 1529 `confirmation` and not
evaluated (4 postures, fewer than the 6 a re-registration needs). The v1 roles
were all `confirmation`.

| File | Role | Session (bag stem) | Postures | Plateaus | Gripper lost | Rigid body | Lag (s) | sha256 (prefix) |
|---|---|---|---|---|---|---|---|---|
| `optitrack_2023-09-12-1529_static_postures.csv` | confirmation | `optitrack_tiago_calibration_2023-09-12-15-29-23` | 4 | 8 | 4 | 1 | −0.8036 | `7e27603dc975ad8f` |
| `optitrack_2023-09-12-1552_static_postures.csv` | confirmation | `optitrack_tiago_calibration_2023-09-12-15-52-58` | 9 | 17 | 8 | 1 | −1.0840 | `9bfb7004b13d8d4d` |
| `optitrack_2023-09-12-1658_static_postures.csv` | training | `optitrack_caibration_2023-09-12-16-58-30` | 13 | 22 | 9 | 2 | −1.3820 | `c46bd5d0f1af89ac` |

- **Provenance:** original recordings (ROS bags, not distributed) of
  2023-09-12, named and hashed in `protocol.yaml`. Recorded on the TIAGo
  (Schunk gripper) with OptiTrack at 120 Hz through `natnet_ros`, joints from
  PAL introspection at 100 Hz. Everything is extracted from the bags; the CSV
  exports that accompany the recordings are not used (shifted columns and
  truncated precision). Lab and licence line: to be added by the maintainer.
- **Units and frames:** metres and radians. Orientation is pinocchio
  roll-pitch-yaw (`R = Rz Ry Rx`). `x1..phiz1` is the pose of the
  `tiago_schunk` body in the **OptiTrack world frame**
  (`points_in_base_frame` = 0, as for the 2021-10-26 Qualisys sessions).
  `xb..phizb` is the world pose of the `tiago_base` body, NaN where that body
  had fewer than 20 live samples (one row in each of the 15:52 and 16:58
  runs); `n_base_live` is the sample count. A tracked row can be expressed
  in the base body (`R_bᵀ (p_g − p_b)`, `R_bᵀ R_g`), but the base body frame
  is fixed to the base, is not `base_link`, and is likely y-up like the
  OptiTrack world, so any fit estimates base 6D and tool 6D. The base did
  not move across the three runs (0.5 mm, 0.4° between the start and end of
  each run and between runs); the base pose stays a free registration
  parameter. `rigid_body_def` is the gripper body definition (below).
- **Columns:** `x1, y1, z1, phix1, phiy1, phiz1`, the eight joints (plateau
  mean of the `*_position` channels), `xb..phizb`, `t_start_robot`,
  `t_end_robot` (s, robot clock), `n_window` (gripper samples in the window,
  frozen ones included), `n_live` (live gripper samples), `n_base_live`,
  `pose_std_mm` (largest per-axis position std), `pose_std_deg` (RMS rotation
  deviation from the mean), `rigid_body_def`, `points_in_base_frame`. Loads
  in core with `NbMarkers: 1`; extra columns are ignored.
- **Clock:** the NatNet header stamps run 0.78, 1.06 and 1.36 s ahead of the
  recorder, so the lag differs per run: −0.80, −1.08 and −1.38 s
  (`t_mocap = t_robot − lag`), measured as the median receive-minus-header
  offset of the mocap stream minus that of the robot stream. The often-quoted
  1.36 s is the 16:58 value only. The offset is stable within a run (IQR
  2 to 9 ms, medians of the thirds within 12 ms); robot stamps lag the
  recorder by a constant 20 ms. A speed cross-correlation with the nominal
  forward kinematics agrees to within 0.04 s (0.2 s on 15:29, where 63 % of
  the gripper samples are frozen).
- **Extraction rules:** the plateau and window rules of the Qualisys files
  (imported from `mocap_extraction.py`): 1 mrad over 0.5 s, at least 2 s,
  window `[start + 0.5 s, end − 0.3 s]` shifted by the lag, at least 20
  samples. Frozen NatNet samples (the pose repeated bit for bit when
  tracking is lost) are removed first. A plateau is dropped only when the
  gripper has fewer than 20 live samples in the window: always a complete
  loss (zero live samples). Plan postures 0, 3, 5 and 6 are occluded in every
  run; the rest of the drops are tracking losses in the later plan postures.
  The start posture is in every file. Plan postures 17, 21 and 22 were never
  executed. No orientation flips: the largest in-window deviation from the
  mean rotation is 3° and the position std is at most 0.92 mm.
- **Rigid-body definitions:** definition 1 (15:29, 15:52) and definition 2
  (16:58). At the same posture (joints within 1 mrad), the gripper in the
  base body differs by 3.7 to 9.0 mm and 0.4 to 2.3° between 15:29 and 15:52
  (robot repeatability, no common rotation) but by 5.2 to 7.1° between 15:52
  and 16:58, with a nearly constant tool-frame rotation about
  (5.7, −3.1, 0)°: a redefined body. The recordings hold no model
  description, so the id is an annotation backed by
  `tests/test_tiago_optitrack_2023_09_12.py`. Never pool runs across
  definitions without a tool registration per definition.
- **Registration-only sanity fit** (nominal joints, base 6D and tool 6D
  fitted on each run, `wrist_ft_tool_link`, rotation weighted 0.1 m/rad):
  9.6 mm and 3.7° (4 postures), 14.9 mm and 3.9° (9), 12.3 mm and 3.5° (13).
  A sanity check of the files, not an accuracy claim: no joint offsets are
  fitted, and 4 postures give 24 equations for 12 parameters.
- **Known limits:** few postures, and the calibrated region is biased by
  systematic occlusion. Partial tracking in four kept rows (shown by
  `n_live / n_window`: 0.21 and 0.50 in 15:52, 0.11 and 0.49 in 16:58).
  Robot repeatability of 4 to 9 mm is about ten times the mocap noise. The
  joint offsets loaded that day are unknown. The 15:29 run is too small to
  fit joint offsets. Robot state, hand and bodies differ from the Qualisys
  files, so these are not comparable with them without re-registration. The
  `tiago_shoulder` body is tracked in the bags and is not shipped.
- **Extraction:** `python -m examples.tiago.utils.optitrack_extraction BAG OUT
  --rigid-body-def N --expected-sha256 SHA` from the repository root
  (`rosbags` required; the lag is measured; `--frame world` is the default).
  The bag name, sha256, lag and drops of each run are in `protocol.yaml`.
  `tests/test_tiago_optitrack_extraction.py` re-creates the three files byte
  for byte when `TIAGO_RECORDINGS_ROOT` points at a folder holding the
  original recordings; it is skipped otherwise.

## Head-camera chessboard data (`calibration/head_camera/`)

Chessboard observations by the TIAGo head camera, 2023-10 to 2023-11
([#108](https://github.com/thanhndv212/figaroh-examples/issues/108), for the
camera study [#101](https://github.com/thanhndv212/figaroh-examples/issues/101);
the study is `examples/tiago/head_camera_study.py`, results in
`docs/development/tiago-head-camera.md`).
Thirteen sessions in one protocol (`calibration/head_camera/protocol.yaml`,
`tiago-head-camera-2023` v2): 10 sessions of PAL eye-hand tool output
(`camera_*`) and 6 mocap runs (`mocap_*`) of the same boards, three of
which pair with a PAL CSV of their own and three with the 2023-11-07 CSV.

| Session | Robot | Role | Postures | Mocap kept / plateaus (unpaired, lost, flipped, repaired) |
|---|---|---|---|---|
| `2023-10-27-test1` | tiago-48 | confirmation | 12 | 11 / 12 (0, 1, 0, 1) |
| `2023-10-27-test2` | tiago-48 | confirmation | 12 | 11 / 13 (1, 1, 0, 1) |
| `2023-10-27-test3` | tiago-48 | confirmation | 12 | 8 / 13 (1, 4, 0, 0) |
| `2023-11-07-camera` | tiago-48 | diagnostic | 19 | none |
| `2023-11-07-1444` | tiago-48 | diagnostic | (mocap only) | 12 / 20 (2, 3, 3, 0) |
| `2023-11-07-1516` | tiago-48 | diagnostic | (mocap only) | 14 / 20 (2, 4, 0, 0) |
| `2023-11-07-1528` | tiago-48 | diagnostic | (mocap only) | 14 / 21 (2, 4, 1, 0) |
| `2023-11-23-distortion_calib` | tiago-48 | diagnostic | 18 | none |
| `2023-11-23-distortion_original` | tiago-48 | validation | 17 | none |
| `2023-11-23-distortion_zero` | tiago-48 | diagnostic | 17 | none |
| `2023-11-24-test_optimal` | tiago-155c | diagnostic | 11 | none |
| `2023-11-27-center` | tiago-48 | training | 17 | none |
| `2023-11-27-topleft` | tiago-48 | diagnostic | 18 | none |

Not usable, recorded in `protocol.yaml` with the reason: the two 2023-10-24
PAL CSVs (`test_1`, `test_2`; their files are not shipped), the four
2023-10-24 bags (motor-encoder joints only, no camera observation) and the eight
2023-10-25 bags (joints and PAL introspection only, no camera observation and
no PAL CSV).

- **Robot identity:** two robots appear. Sessions 2023-10-27 to 2023-11-27
  with the Hey5 hand are `tiago-48` (PAL's tool files call it `48c`); the
  2023-11-24 `test_optimal` session is `tiago-155c`, a different robot. The
  evidence: the file names (`48c_hey5` / `155c`); the end effector in the PAL
  tool archives' URDFs (Hey5 hand vs gripper); and the camera projection
  intrinsics in those archives (2023-10-27 fx 536.246, equal to PAL's
  reference calibration exactly; 2023-11-27 fx 528.73; 2023-11-24, 155c,
  fx 502.44). Encoder state is not constant on tiago-48: the median gap
  position minus absolute encoder on arm_1 is 0.0 mrad on 2023-10-25/10-27
  and -3.8 mrad on 2023-11-07 and the 2023-09 mocap sessions. Encoders were
  not re-zeroed (maintainer), so these gaps are robot-state slips.
- **Roles (frozen in v2):** training = `2023-11-27-center`; validation =
  `2023-11-23-distortion_original`; confirmation = the three 2023-10-27
  sessions. Everything else is diagnostic: `2023-11-27-topleft`, the other
  2023-11-23 variants, 2023-11-07 (camera CSV and the three mocap runs, which
  sit about 45 mm off the 10-27/11-23/11-27 group) and `2023-11-24-test_optimal`
  (the other robot). The 2023-10-24 sessions are not usable: chessboard-in-camera
  positions at matched postures differ by 50-100 mm from 10-27, 11-23 and
  11-27 and by 55 mm between test_1 and test_2, and the 10-24 bags show a
  motor-minus-absolute arm_5 gap of +207 / +437 mrad.
- **Provenance:** original recordings (PAL eye-hand calibration tool CSVs and
  ROS bags, not distributed), named and hashed in `protocol.yaml`. The six PAL
  tool archives that hold the URDF and the image corners are not shipped; the
  CSV is their derived camera-pose table.
- **Units and frames:** metres and radians. `x..phiz` is the **chessboard pose
  in the camera frame**, orientation as pinocchio roll-pitch-yaw
  (`R = Rz Ry Rx`). Camera rows are in PAL's camera optical frame; mocap rows
  are in the `tiago_camera` body frame, `p = Rc^T (p_board - p_cam)`,
  `R = Rc^T R_board`. The two frames differ by a fixed rotation, so the
  orientation columns of the two kinds are not directly comparable.
- **Camera tables:** `camera_<id>_chessboard_in_camera.csv` and
  `camera_<id>_joints.csv` keep PAL's 6-significant-digit values as written.
  The joints are PAL's **absolute-encoder** readings (arm 1-7, head 1-2). The
  torso is not in the PAL CSV: it comes from the paired bag for the three
  2023-10-27 sessions (`torso_source = bag`) and is **assumed 0.30 m**
  everywhere else (`torso_source = assumed`).
- **Mocap tables:** one row per static plateau of the 9 arm and head joints
  (5 mrad over 0.5 s, at least 1 s), averaged over `[start+0.5, end-0.3]`
  shifted by the measured clock lag (10-27 about +6.71 s, 11-07 about
  +8.58 s). A plateau is paired to a PAL row by joints (10 mrad, one-to-one),
  never by time, because the camera CSV has no clock. For 2023-11-07 the
  pairing is by posture plan and joints only.
- **Flip handling:** the board rigid body sometimes jumps to a mirrored
  orientation. Samples more than 90 degrees from the plateau's medoid are
  removed (`repaired`; a plateau with more than 40 % removed is `ambiguous`
  and dropped). A plateau more than 90 degrees from the run's medoid of
  `R_pal^T R_mocap` is `flipped` and dropped. PAL values only select flips;
  they are never written into the mocap files. The drop counts per run are in
  `protocol.yaml` and the table above (11-07-1444 has one borderline plateau
  at 105 degrees, treated as flipped).
- **Splits are not independent in posture:** all sessions repeat the same
  posture plan, so training, validation and confirmation share postures.
- **PAL reference** (`pal_reference/`): `master_calibration_pal.yaml` is
  PAL Robotics' camera calibration (intrinsics and extrinsics), verbatim,
  attributed to PAL Robotics S.L. `offset.xacro` is **of unknown origin** and
  not PAL's reference (its values differ); it is shipped as found.
- **Limits:** torso assumed where unpaired; 11-07 pairing by plan only; which
  distortion setting each 2023-11-23 variant applied is not documented; the
  camera has no clock, so there is no timestamp pairing; the intrinsics exist
  only in the PAL yaml.
- **Re-creating the files:** with the original recordings available,
  `python -m examples.tiago.utils.head_camera_extraction camera <csv> <prefix> [--bag B]`
  and `... mocap <bag> <csv> <out>`; `tests/test_tiago_head_camera_extraction.py`
  re-creates every file byte for byte when `TIAGO_RECORDINGS_ROOT` is set.

## Vicon calibration data (`calibration/vicon/`)

Two static-posture runs recorded on 2023-09-20
([#106](https://github.com/thanhndv212/figaroh-examples/issues/106)) with a
Vicon system, extracted from the original recordings. They belong to their own
protocol (`calibration/vicon/protocol.yaml`, `tiago-vicon-2023-09-20` v2,
independent of the Qualisys and OptiTrack protocols). The motion-capture study
([#100](https://github.com/thanhndv212/figaroh-examples/issues/100)) fixed the
roles in v2: 1508 `training`, 1607 `confirmation` (v1: both `confirmation`).
The issue asked for the unclosed bag of the
session; that recording is not read here (see Not shipped).

| File | Role | Run (local time) | Postures | Plateaus | Gripper lost | Marker triad | Lag (s) | sha256 (prefix) |
|---|---|---|---|---|---|---|---|---|
| `vicon_2023-09-20-1508_static_postures.csv` | training | 1508, `.c3d` | 17 | 17 | 0 | 1 | +1.9350 (origin) | `745b6852eb27907a` |
| `vicon_2023-09-20-1607_static_postures.csv` | confirmation | 1607, `.csv` | 21 | 21 | 0 | 2 | −0.6100 | `18f213f8fdc51a26` |

- **Provenance:** original recordings (not distributed), named and hashed in
  `protocol.yaml`: a Vicon export per run (c3d for 1508, CSV for 1607) and
  the PAL introspection bag of the same run for the joints (100 Hz, 307.8 s
  and 414.0 s). The Vicon recordings hold 10 labelled markers (`base1..3`,
  `shoulder1..4`, `gripper1..3`) at 100 Hz: 32545 frames (325.4 s) in the
  c3d (plus two unlabelled points, not read) and 41313 rows (417.7 s, two
  gaps over 50 ms, the longest 0.20 s) in the CSV. The CSV also holds a force
  plate wrench and centre of pressure (plate 4); they are not read here.
  Lab, hardware and licence line: to be added by the maintainer. The Vicon
  software version, the camera count and the marker model are unknown.
- **Markers, not bodies:** unlike the Qualisys and OptiTrack files there is
  no rigid-body pose. The two bodies of the layout are marker triads:
  `gripper1..3` for `x1..phiz1` and `base1..3` for `xb..phizb`. Frame:
  origin at the centroid, `x` from marker 1 to marker 2, `z` along `x` cross
  (marker 3 − marker 1), `y = z cross x`. The triad frames are not the
  TIAGo link frames, and their offsets are unknown: the base pose and the
  tool pose of any fit are free registration parameters. The shoulder
  markers are not shipped.
- **Units and frames:** metres and radians (the c3d is converted from mm).
  Orientation is pinocchio roll-pitch-yaw (`R = Rz Ry Rx`). Poses are in the
  **Vicon world frame** (`points_in_base_frame` = 0). `xb..phizb` is the
  base triad pose in the same frame; it is the same in every row of a run
  (position std ≤ 0.26 mm, orientation std ≤ 0.04°) and differs between runs
  (the base stood at (1.060, 0.384) m in 1508 and (0.904, 0.460) m in
  1607). `n_base_live` counts the frames where all three base markers are
  live; every row has at least 231 live frames of both triads, so no row has
  NaN base columns (the columns are kept for the shared layout).
- **Columns:** the OptiTrack layout (`x1..phiz1`, the eight joints,
  `xb..phizb`, `t_start_robot`, `t_end_robot`, `n_window`, `n_live`,
  `n_base_live`, `pose_std_mm`, `pose_std_deg`, `rigid_body_def`,
  `points_in_base_frame`). `n_window` counts mocap frames in the window,
  `n_live` frames where the three gripper markers are live,
  `pose_std_mm` the largest per-axis marker std, `pose_std_deg` the RMS
  rotation deviation of the per-frame triad from the mean. `rigid_body_def`
  is the marker-triad definition (below). Loads in core with
  `NbMarkers: 1`; extra columns are ignored.
- **Clock:** the robot clock is the introspection header stamp. The CSV
  stamps are the Vicon PC wall time written as local time without a zone;
  they are read as UTC+2 (CEST), and the measured residual is −0.61 s
  (`t_mocap = t_robot − lag`): the first CSV row is 1.46 s after the first
  joint sample. The c3d has no absolute clock, so its time axis starts at the
  first joint sample and its lag (+1.935 s) is that origin plus the clock
  offset, not comparable with the CSV one. Both lags are measured per run in
  the extractor by correlating the speed of the gripper triad centroid with
  the speed of the nominal-FK `wrist_ft_tool_link` over ±60 s (0.05 s steps,
  refined to 0.005 s): correlation 0.98 in both runs.
- **Extraction rules:** the plateau and window rules of the Qualisys files
  (imported from `mocap_extraction.py`): 1 mrad over 0.5 s, at least 2 s,
  window `[start + 0.5 s, end − 0.3 s]` shifted by the lag, at least 20
  frames. Lost markers (`0, 0, 0` in the CSV, negative residual in the c3d)
  are NaN; a triad frame is live when its three markers are. Marker
  positions are averaged over the live frames of the window, then the pose is
  built. No plateau had fewer than 20 live gripper frames (lowest 231), so
  nothing is dropped. Averaging windows last 2.3 to 30.3 s; pose std ≤ 0.84 mm and
  ≤ 0.34°.
- **Label exchange (1607):** Vicon exchanges the labels `gripper1` and
  `gripper2` on 16494 of 41310 live frames (about 276 jumps of 100 to
  150 mm between consecutive frames). The extractor relabels by side
  lengths before averaging (`repair_triad_labels`, majority labelling kept)
  and reports the count. The c3d has no exchange (no marker moves more than
  19 mm between frames; its gripper triad is nearly isosceles, so the side
  rule is not applied to it). Without the repair, half of the 1607 gripper
  orientations are mirrored and the registration fit below gives 87° instead
  of 6°.
- **Marker triads:** the base triad is the same in both runs (sides 411–412,
  326, 334–335 mm). The gripper markers were placed differently: sides
  162.7, 109.5, 104.1 mm in 1508 (definition 1) and 142.9, 49.8, 113.5 mm in
  1607 (definition 2; its 50 mm side limits the orientation accuracy). Never
  pool the two runs without a tool registration per definition.
- **Postures:** both runs replay the same script: 15 of the 17 postures of
  1508 are in 1607 (joints within 5 mrad, 14 within 1 mrad), and the 1607 run
  adds 6.
- **Registration-only sanity fit** (nominal joints, base 6D and tool 6D
  fitted on each run, `wrist_ft_tool_link`, rotation weighted 0.1 m/rad):
  19.1 mm and 5.8° (17 postures), 20.2 mm and 5.9° (21). A sanity check of
  the files, not an accuracy claim: no joint offsets are fitted.
- **Known limits:** markers, not bodies (the triad frames are arbitrary and
  the 1607 triad is narrow); the c3d clock is relative; the force plate is
  not shipped; the joint offsets loaded that day are unknown; the base never
  moves within a run, so base and tool registration are weakly separated.
- **Not shipped:** `calib_mocap_vicon_2023-09-20-13-53-01.bag.active`
  (sha256 `1ebae14e0900320781fff44c481ff6d25480f30a4377a59d7aa92027fc6e17e1`)
  is an unclosed bag. It was neither read nor reindexed here (re-scoped from
  the issue: recovering it needs `rosbag reindex` on a copy under ROS Noetic),
  and no Vicon export of that run was found. The two posture bags
  `vicon_postures_2023-09-20-15-22-11.bag` and `…-15-34-35.bag` hold robot
  introspection only (no joints in the first, three moving joints in the
  second) and no Vicon recording, so they give no postures. Hashes are in
  `protocol.yaml`.
- **Extraction:** `python -m examples.tiago.utils.vicon_extraction MOCAP BAG
  OUT --rigid-body-def N --mocap-sha256 SHA --bag-sha256 SHA` from the
  repository root (`rosbags` required, and `c3d` for a c3d; the lag is
  measured). `tests/test_tiago_vicon_extraction.py` re-creates the two files
  byte for byte when `TIAGO_RECORDINGS_ROOT` points at a folder holding the
  original recordings (the c3d run is skipped without the `c3d` package);
  `tests/test_tiago_vicon_2023_09_20.py` checks the shipped files alone.

## Square-motion validation runs (`validation/square_motion/`)

Seven end-effector square-motion runs recorded with Qualisys on 2021-12-16
(three) and 2022-01-12 (four)
([#107](https://github.com/thanhndv212/figaroh-examples/issues/107)),
extracted from the original recordings. They are `validation` sessions of
their own protocol (`validation/square_motion/protocol.yaml`,
`tiago-square-motion-validation` v1): never used to fit anything. Each run is
two files on **one clock** (the robot clock, seconds from the first joint
sample): `<run>_joints.csv` (`t` and the eight joints) and
`<run>_markers.csv` (`t, body, x, y, z, qx, qy, qz, qw`, metres, Qualisys
world frame). Both are cropped and decimated to a common **20 Hz** grid
(`t = k / 20` s), 8.4 MB in total (the full-rate files were 43 MB). Bodies: `base_frame` and the end-effector body (the plate point
`eeframe_BL`, with `BR/TR/TL`, in 2021-12; the rigid body `endeffector_frame`
in 2022-01).

| Run | Offset set | Recording / kept (s) | Lag (s) | Correlation | Registration full / shipped (mm) | Rows joints / markers | sha256 (prefix) joints / markers |
|---|---|---|---|---|---|---|---|
| `2021-12-16-15-04-54_zero` | zero | 111.2 / 102.1 | +5.770 | 0.989 | 4.82 / 4.69 | 2043 / 7775 | `ede5e54fca93` / `61c39b0a47ab` |
| `2021-12-16-15-07-04_zero` | zero | 106.2 / 101.9 | +5.775 | 0.997 | 4.23 / 3.96 | 2039 / 7065 | `d49801f1ca6f` / `237bd3baf394` |
| `2021-12-16-15-38-03_mocap` | mocap | 161.0 / 101.5 | +6.040 | 0.959 | 4.05 / 3.85 | 2031 / 2897 | `64c2e1116a11` / `80eb4bbeb89b` |
| `2022-01-12-10-55-17_zero` | zero | 491.2 / 487.5 | +7.730 | 0.999 | 2.90 / 2.89 | 9751 / 19285 | `1c25f23ec76e` / `244df4579a87` |
| `2022-01-12-11-25-38_mocap` | mocap | 317.5 / 316.4 | +8.105 | 0.985 | 3.75 / 3.74 | 6329 / 11065 | `b65540449732` / `9a58161aaa6a` |
| `2022-01-12-11-10-58_mocap_inv` | mocap_inv | 318.2 / 316.6 | +7.915 | 0.984 | 2.18 / 2.21 | 6333 / 12305 | `12dc638ca8bf` / `1756a5e49975` |
| `2022-01-12-11-38-43_pal` | pal | 327.8 / 321.9 | +8.100 | 0.998 | 3.30 / 3.27 | 6440 / 12764 | `0ff775d1ced1` / `4231bfd5f471` |

- **Decimation and crop:** each value is the mean of the full-rate (100 Hz)
  samples in `[t - 0.025, t + 0.025)` s, a 0.05 s boxcar that is the
  anti-alias filter; quaternions are sign-aligned, averaged and normalised.
  A body with no sample in a window has no row there: tracking loss stays a
  gap and is never interpolated (the absence of a row is the loss flag; the
  joints have no gap). The idle lead-in and lead-out are cropped: joint speed
  over a 0.5 s baseline above 0.02 rad/s marks motion, and the window is
  1 s before the first moving sample to 1 s after the last moving span. The
  crop window of each run is `crop_s` in `protocol.yaml`; the 2022-01 runs
  start moving at once and lose 1.1 to 5.9 s, the 2021-12 runs 4.3 to 59.5 s.
- **Provenance:** the PAL introspection bag of each run (joints, Qualisys
  `/tf`), named and hashed in `protocol.yaml` as original recordings, not
  distributed. Sensor model, camera count and marker placement are unknown.
- **Clock:** `lag = t_robot - t_mocap` is measured per run by the correlation
  of the nominal-FK `wrist_ft_tool_link` speed with the end-effector marker
  speed on the full-rate data (+-60 s at 0.05 s, refined to 0.005 s), stored in the protocol with
  its correlation and method. The marker stamps are put on the robot clock as
  `t = stamp + lag - t0` (lag rounded to 1 ms). The lag is a measured value
  (5.8 to 8.1 s), not a constant of the setup.
- **Offset sets:** the joint offsets the robot was running (zero, mocap,
  mocap_inv, pal) are recovered from the recorded `/tf` against the nominal
  URDF and written with their values in `protocol.yaml`. `zero` is all 0;
  `mocap` is the same on both dates (within 1e-5); `mocap_inv` is its sign
  reversal (within 8e-4). PAL's own offsets are applied inside its controller
  and are not in the recordings: the `pal` values are `unknown`, not guessed.
- **Nominal registration:** RMS distance of the nominal-FK tool point to the
  marker path after a rigid fit and a fixed tool point: 2.2 to 4.8 mm on the
  full-rate data (in `protocol.yaml` as `nominal_registration_rmse_mm`), 2.2
  to 4.7 mm on the shipped files (`nominal_registration_rmse_shipped_mm`, the
  decimation moves it by at most 0.27 mm). A check that the files agree on
  one clock, not a calibration; no joint offset is fitted.
- **2022-01-12 zero folder:** the issue expected no usable joint data; the
  bag holds a complete run (491 s, correlation 0.999) and it is shipped.
- **Not shipped:** seven 2021-12 recordings (one with marker loss, two
  aborted and static, four without end-effector markers) and thirteen QTM
  exports of 2022-01 (Qualisys clock only, no joints); reasons and sha256 in
  `protocol.yaml`.
- **Re-creation:** `python -m examples.tiago.utils.square_motion_extraction
  BAG PREFIX --expected-sha256 SHA` from the repository root (`rosbags`
  required). `tests/test_tiago_square_motion_extraction.py` re-creates every
  file byte for byte when `TIAGO_RECORDINGS_ROOT` points at the original
  recordings; `tests/test_tiago_square_motion.py` checks the shipped files
  alone.
