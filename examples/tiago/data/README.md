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

Audited in [#24](https://github.com/thanhndv212/figaroh-examples/issues/24);
full report: [`docs/development/tiago-mocap-calibration-audit-2026-10-04.md`](../../../docs/development/tiago-mocap-calibration-audit-2026-10-04.md).
The raw file is kept unmodified.

`qualysis_base_hand_calibration.csv` (sha256 prefix `d7fcbd96e3e67319`): 34
static postures, one row each. Columns `x1,y1,z1 … x4,y4,z4` then the eight
joint positions `torso_lift_joint` (m) and `arm_1_joint` … `arm_7_joint` (rad).

- **No clock:** rows are independent postures; there are no timestamps.
- **Units:** marker coordinates in metres in the mocap world frame (consistent
  with a millimetre-level fit); joints in rad (torso in m).
- **Markers are derived points, not raw measurements:** inter-marker distances
  vary by only ~0.1 µm across postures (optical noise is ~0.1 mm), so the four
  points were computed from a rigid-body pose. Marker 4 is an exact copy of
  marker 3; all markers move with the hand (none is on the base).
- **Used:** only marker 1, position only (`measurable_dof` xyz), expressed by
  calibration as a point fixed in `wrist_ft_tool_link`. The three distinct
  points would also determine orientation; that information is unused.
- **End effector unrecorded:** the file name says "hand" and the sample set is
  `…_pmb2_hey5.yaml`, but calibration loads `tiago_48_schunk.urdf` (WSG
  gripper). Only the estimated tip offset depends on it.
