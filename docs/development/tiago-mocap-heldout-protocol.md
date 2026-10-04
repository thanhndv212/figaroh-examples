# TIAGo mocap calibration: held-out protocol

Issue: [#27](https://github.com/thanhndv212/figaroh-examples/issues/27).
Delivery package: [C2 / core #44](https://github.com/thanhndv212/figaroh-plus/issues/44).
Frozen 2026-10-05.

This fixes which recordings evaluate the TIAGo mocap calibration reference,
which rules keep them independent of the fit, and how results are reported.
`examples/tiago/heldout_protocol.py` reproduces every number below.
`tests/test_tiago_heldout_protocol.py` checks the freeze (file hashes, roles,
config wiring) and pins the results.

## 1. Inventory of recordings

All TIAGo mocap calibration recordings are in the private
`robot-calibration-identification-dataset` repository (`tiago/`, audited in
its `AUDIT.md`).

| Session | Postures | Frame | Status |
|---|---|---|---|
| 2021-11-30 15:44 | 37 | Qualisys `base_frame` body | **training** |
| 2021-11-26 11:05 | 62 | `base_frame` body | **validation** |
| 2021-11-30 14:03 | 63 | `base_frame` body | **confirmation** |
| 2021-11-30 15:04 | 59 | `base_frame` body | **confirmation** |
| 2021-11-12 16:38 (`calib_mocap64`) | 63 | mocap world (no base body) | not used: needs re-registration, so it cannot test the frames |
| 2021-11-12 15:07 | 26 | world | not used: same reason |
| 2021-10-26 ×2 | 26 + 25 | world | not used: same reason |
| 2021-09 sessions | — | — | unusable: end-effector body under two alternating names |
| 2023-09-12 OptiTrack ×3 | 4–12 | world, 6D | not used: too few postures, different robot state |
| 2023-11-07 OptiTrack | 16 | world, 6D | unusable: chessboard body flips |

**Same frame:** the four sessions used here express the hand's Qualisys
rigid-body points in the Qualisys `base_frame` body, which is fixed to the
robot base. Its definition is identical on both days: the six inter-point
distances agree to < 0.1 mm. So a model fitted on one session (base and tool
frames included) is evaluated on another **without re-registration**.

**Extraction:** every file was extracted with the same frozen rules, fixed
before any fit (`tools/audit/nov30.py`, `tools/audit/figaroh_mocap_csv.py` in
the dataset repository):
- one row per static plateau (all joints within 1 mrad over 0.5 s, ≥ 2 s);
- joints averaged over [start + 0.5 s, end − 0.3 s];
- marker points averaged over the same interval on the mocap clock, shifted
  by the session's measured clock lag (3.9 s on 2021-11-30, 2.6 s on
  2021-11-26).

No sample is filtered or removed after extraction. Core does not remove
outliers (figaroh-plus#98).

## 2. Frozen sets

Files in `examples/tiago/data/calibration/mocap/`:

| Role | File | Rows | sha256 |
|---|---|---|---|
| training | `qualisys_2021-11-30_static_postures.csv` | 37 | `b6c0051e20c6a077a6d2cf64ba912996945bbb85d6860cd6cea7d8ff65b1b9af` |
| validation | `qualisys_2021-11-26_static_postures.csv` | 62 | `7c986df711757c4d31d8bb6db34b354f72318a78673dac280fd3a14abc5460a0` |
| confirmation | `qualisys_2021-11-30-1403_static_postures.csv` | 63 | `e44c678fbbc1fbc852107b0fe6793470dbf3a7cef77ddf5faa25c9e4ff4cb503` |
| confirmation | `qualisys_2021-11-30-1504_static_postures.csv` | 59 | `cc19eaa74fe8b74be28d758424ef4acd18f516dc59e40754b37d87f8592fbb84` |

**Rules:**
1. **Fit on training only.** Base, tool and joint parameters all come from
   `source_file`. Nothing is re-estimated on held-out data.
2. **No tuning on the confirmation sets.** No model, level, regularisation,
   tolerance or extraction choice may be made by looking at them. They are
   read only by `heldout_protocol.py` and its test, to report a change that
   was decided on other grounds.
3. **Validation is reported by default, but it is not pristine.**
   `tiago_unified_config.yaml` points `validation_data_file` at it, so
   `calibration.py` prints its error. It was consulted once: when
   regularisation was set to 0 (figaroh-examples#73), its RMSE (4.23 vs
   4.35 mm) was quoted alongside the actual reason (a full-rank problem
   after figaroh-plus#102). So treat the confirmation sets as the
   independent check.
4. **Frozen files.** Changing a frozen file, or a file's role, needs a new
   protocol version, with the reason and the old hashes kept here.

## 3. What is measured

- **Observation:** the position (x, y, z) of one rigid-body point, BL
  (`x1,y1,z1`), in metres, in the `base_frame` body. The other three points
  (BR, TR, TL) are unused: core supports one marker per sample. Since all four
  come from one body pose, the hand orientation is observable in principle
  but unobserved here.
- **Estimated frames (gauge):** the base frame (6D, `base_*`) and the tool
  point (3D, `pEE*_1`) are estimated on training, together with the joint
  parameters. Core drops joint parameters the frames absorb
  (figaroh-plus#102): at `joint_offset`, the torso and arm_1 offsets (vertical
  axes, absorbed by base z and yaw); at `full_params`, 4 of the 32. Reported
  joint offsets are therefore relative to this gauge. An offset on the torso,
  on arm_1, or on arm_7 roll (not in the observed chain) is not estimable from
  this data.
- **Baseline ("nominal"):** **registration only**, meaning nominal joints with
  base and tool fitted on training (9 parameters). The all-zero "nominal" in
  core's validation report puts the base frame at the origin, which is
  meaningless here (419 mm), and is not used.
- **Metric:** per-axis RMSE and norm RMSE of the BL position error, in mm, plus
  the maximum.
- **Posture groups**, relative to the training postures (training's 37-posture
  plan is a subset of the 63-posture plan used on the other days):

  | Group | Definition |
  |---|---|
  | repeated | every joint within 0.01 rad (m) of a training posture: the same configuration on another occasion |
  | new | not repeated, inside the training joint ranges (± 0.05) |
  | out of range | some joint outside the training range: extrapolation |

## 4. Results (protocol v1, 2026-10-05)

figaroh-plus `devel` `5591b9d`, figaroh-examples `main` `7a57e16` plus this
change; `figaroh-dev`, Python 3.12, Pinocchio 3.7.0, single-threaded BLAS.
`python heldout_protocol.py` from `examples/tiago`.

Norm RMSE (mm) per set, with posture-group counts in brackets:

| Model (parameters) | Training | Validation 11-26 | Confirmation 11-30 14:03 | Confirmation 11-30 15:04 |
|---|---|---|---|---|
| registration only (9) | 3.37 | 4.87 | 4.58 | 4.22 |
| `joint_offset` (14), **reference** | 2.88 | 4.23 | 4.09 | 3.83 |
| `full_params` (28) | 1.75 | 3.68 | 3.35 | 2.89 |

By posture group, held-out sets:

| Model | repeated (37 / 37 / 35) | **new (16 / 17 / 16)** | out of range (9 / 9 / 8) |
|---|---|---|---|
| registration only | 3.93 / 3.48 / 3.25 | 5.12 / 4.85 / 4.73 | 7.34 / 7.23 / 6.35 |
| `joint_offset` | 3.31 / 3.08 / 2.84 | **4.47 / 4.29 / 4.13** | 6.53 / 6.61 / 6.21 |
| `full_params` | 2.43 / 2.10 / 1.99 | **3.24 / 3.13 / 3.06** | 7.10 / 6.46 / 5.07 |

Per axis, `joint_offset`, mm:

| Set | x | y | z |
|---|---|---|---|
| training | 1.78 | 1.59 | 1.61 |
| validation 11-26 | 2.37 | 2.59 | 2.35 |
| confirmation 14:03 | 2.32 | 2.56 | 2.20 |
| confirmation 15:04 | 2.32 | 2.31 | 1.98 |

The `joint_offset` fit's only clearly non-zero joint parameter is arm_5,
−49.8 ± 10.1 mrad (SE per figaroh-plus#107).

## 5. What this supports, and what it does not

- **Calibration helps, modestly, on every held-out set and posture group.**
  On new postures inside the training range, `joint_offset` is 0.56–0.65 mm
  better than registration only (about 12 %). `full_params` is a further
  1.1–1.2 mm better. Its training-to-held-out gap is larger (1.75 →
  3.1–3.2 mm on new postures), but it still generalises better within the
  training range on this robot and day range.
- **No better than registration outside the training range.** On out-of-range
  postures all models are at 5–7 mm. The worst two postures (8.5 and 12 mm in
  every held-out session) are beyond anything in training: one has arm_3 at
  −3.14 and arm_7 at −1.58 rad, the other arm_6 at −1.18 and arm_7 at
  +1.77 rad. A calibration fitted on the 37-posture
  plan should not be expected to hold there.
- **The "repeated" group measures day-to-day repeatability, not
  generalisation.** Same configurations, different day: 2.8–3.3 mm for
  `joint_offset`, against 2.88 mm in training.
- **Residual floor:** about 3–4 mm of residual remains for every model and is
  not explained by geometry. The likely sources are arm_6 backlash (~40 mrad
  free play, approach-direction dependent) and arm_5's encoder issues. See the
  raw-data audits (`AUDIT_joints.md` in the dataset repository).
- **One robot, three days in one week, one end effector, one marker point.**
  Nothing here supports a claim about other units, later dates (arm_5's
  encoder offset shifted by ~90 mrad between 2021 and 2023), or orientation
  accuracy.
- **Default level:** the reference stays at `joint_offset`, chosen for
  interpretability (one offset per joint, exported into the URDF), not for
  held-out error. `full_params` is better by ~1 mm on new in-range postures.
  Changing the default is a separate decision; per rule 2, it must not be
  made on the confirmation sets alone.
