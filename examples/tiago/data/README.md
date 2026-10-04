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
- **Velocity:** a measured channel, consistent with d(position)/dt in units
  and sign (correlation ≈ 0.99, scale ≈ 1.00), but **delayed by ~18 samples
  (0.18 s)**. The loader estimates and removes this delay.
- **Effort:** raw values converted in `process_torque_data` with the
  per-joint `reduction_ratio × kmotor` set in `identification.py`, plus
  `9.81 × subtree mass` on the torso. Units, signs and constants are
  documented assumptions, not verified against a torque reference. The
  wrist efforts (`arm_5`–`arm_7`) are exactly zero on 88–90% of samples
  (step 0.001), so wrist dynamics are weakly observable.
- **Window:** `identification.py` keeps rows 921–6791 (9.21–67.90 s), the
  excitation; RMS velocity 0.159 rad/s inside vs 0.017 / 0.006 rad/s before /
  after.

`tiago_bp_19_Oct_2024_2320.csv` and `tiago_nov_30_64.csv` are not used by
`identification.py` and were not audited.

## Mocap calibration data (`calibration/mocap/`)

Rebuilt from the raw Qualisys recordings in
[#67](https://github.com/thanhndv212/figaroh-examples/issues/67). The C1 audit
([#24](https://github.com/thanhndv212/figaroh-examples/issues/24),
[report](../../../docs/development/tiago-mocap-calibration-audit-2026-10-04.md))
covers the file these replace.

| File | Role | Session | Postures | sha256 (prefix) |
|---|---|---|---|---|
| `qualisys_2021-11-30_static_postures.csv` | training (`source_file`) | `calib_mocap_2021-11-30-15-44-33` | 37 | `b6c0051e20c6a077` |
| `qualisys_2021-11-26_static_postures.csv` | held-out (`validation_data_file`) | `calib_mocap_2021-11-26-11-05-59` | 62 | `7c986df711757c4d` |

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

Raw bags and the extraction scripts (`tools/audit/nov30.py`,
`tools/audit/figaroh_mocap_csv.py`) are in the private
`robot-calibration-identification-dataset` repository, under `tiago/`.

**Used by calibration:** marker 1 (BL), position only (`measurable_dof` xyz),
expressed as a point fixed in `wrist_ft_tool_link`. Core supports one marker
per sample (`NbMarkers == 1`), so points 2–4 are unused. Since all four come
from one body pose, a 6D pose measurement would carry the same information. The end effector
for these sessions was not recorded; calibration loads `tiago_48_schunk.urdf`,
and only the estimated tip offset depends on that choice.

**Reference result:** `calibration.py`, `calibration_level: joint_offset`,
base and tool estimated, figaroh-plus `devel` with #101 and #105.

| Training RMSE | Held-out RMSE (Nov-26) | Held-out max | arm_5 offset |
|---|---|---|---|
| 2.90 mm | 4.35 mm | 12.6 mm | −37.5 mrad |

With regularisation 1e-4 instead of the default 0.01, arm_5 is −49.7 mrad.
The default shrinks it (figaroh-plus#102). About 3.7 mm of residual remains in
every Nov-2021 session after calibration, which is not geometric: it is the
practical floor for this setup. `full_params` (32 parameters) gains at most
0.6 mm on held-out postures.

**Deployment:** at `joint_offset` level, `calibration.py` writes an empty PAL
`master_calibration.yaml` (`geometric_calibration: {}`), because the PAL
export carries only `full_params` placement corrections. Mapping joint
offsets to PAL's `arm_k_joint_offset` entries is #28 (C3). The exported URDF
(`update_model.py`) does contain the offsets, written into each joint's
`<origin>` (figaroh-plus#101).

**Superseded file, kept unmodified and unused by any config:**
`qualysis_base_hand_calibration.csv` (34 postures, sha256 prefix
`d7fcbd96e3e67319`). It came from the same
session, but joints and markers were paired by raw timestamp with the 3.9 s
clock offset uncorrected. In 16 of 34 rows the marker sample was taken after
the arm had started moving, with errors up to 7.8 mm (row 29 is the worst).
Its "marker 4" repeated marker 3, so TL was missing.
