# TIAGo Experimental Data

The `suspension/` and `backlash/` directories contain historical offline logs
copied verbatim from the reachable `figaroh-plus` Git ref at commit
`2218d77638e0148afd3b358fa51702f8b82f4100`.

- `suspension/tiago_xyz_vicon_1640.csv` is a Vicon and force-plate recording.
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
`confirmation` sessions of their own protocol
(`calibration/optitrack/protocol.yaml`, `tiago-optitrack-2023-09-12` v1,
independent of the Qualisys protocol). None is evaluated before the
motion-capture study
([#100](https://github.com/thanhndv212/figaroh-examples/issues/100)), which
adds the held-out block in v2.

| File | Role | Session (bag stem) | Postures | Plateaus | Gripper lost | Rigid body | Lag (s) | sha256 (prefix) |
|---|---|---|---|---|---|---|---|---|
| `optitrack_2023-09-12-1529_static_postures.csv` | confirmation | `optitrack_tiago_calibration_2023-09-12-15-29-23` | 4 | 8 | 4 | 1 | −0.8036 | `7e27603dc975ad8f` |
| `optitrack_2023-09-12-1552_static_postures.csv` | confirmation | `optitrack_tiago_calibration_2023-09-12-15-52-58` | 9 | 17 | 8 | 1 | −1.0840 | `9bfb7004b13d8d4d` |
| `optitrack_2023-09-12-1658_static_postures.csv` | confirmation | `optitrack_caibration_2023-09-12-16-58-30` | 13 | 22 | 9 | 2 | −1.3820 | `c46bd5d0f1af89ac` |

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
