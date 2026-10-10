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
  ~98% the URDF's own `m g` term (its `ratio × kmotor = 1` is undocumented).
  Only arm_1–arm_4 effort is therefore fitted and scored
  (`problem.torque_fit_joints` in the config, #68); torso and wrist motion
  still enter the regressor. Held-out RMSE on `calibration_slow`, pooled over
  arm_1–arm_4:

  | Effort fitted | Base parameters | arm_1 | arm_2 | arm_3 | arm_4 | Pooled |
  |---|---|---|---|---|---|---|
  | arm_1–arm_4 (default) | 57 | 1.310 | 2.540 | 1.471 | 1.237 | 1.722 |
  | all eight joints | 73 | 1.316 | 2.504 | 1.450 | 1.237 | 1.706 |
  | nominal model | – | 3.187 | 7.838 | 3.122 | 2.858 | 4.730 |

  Leaving the torso out changes nothing on the arm; leaving the wrist out
  costs ~1%. The gain is that no parameter is fitted to a signal known to be
  unreliable. Fit RMSE is 0.884 on the arm rows (0.643 when all eight joints
  were fitted and pooled).
- **End effector:** a Hey5 hand (the recordings log its `hand_*` joints and
  no gripper), so `identification.py` loads `urdf/tiago_48_hey5.urdf`. This
  applies to all three 2021-07 recordings below. The wrist F/T sensor weighs
  0.794 kg below the sensor, against 1.032 kg in the Hey5 URDF (#68). The
  switch from the Schunk URDF (measured with the shifted velocity) left the
  fit and held-out RMSE (1.209) and the 73 base parameters' count unchanged; 30 base-parameter values
  shifted to absorb the hand's inertia, and the nominal model's held-out
  RMSE moved from 3.375 to 3.407.
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
the torso force conversion and arm_1 constant are unsupported, and wrist
effort quantisation prevents reliable identification there; both are left
out of the torque fit (see Effort above). Shipping these files does not resolve those modelling
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
full rank (condition number 26).

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

**Superseded file, kept unmodified and unused by any config:**
`qualysis_base_hand_calibration.csv` (34 postures, sha256 prefix
`d7fcbd96e3e67319`). It came from the same
session, but joints and markers were paired by raw timestamp with the 3.9 s
clock offset uncorrected. In 16 of 34 rows the marker sample was taken after
the arm had started moving, with errors up to 7.8 mm (row 29 is the worst).
Its "marker 4" repeated marker 3, so TL was missing.
