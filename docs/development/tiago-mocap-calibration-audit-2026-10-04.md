# TIAGo mocap calibration audit — 2026-10-04

Issue: [figaroh-examples #24](https://github.com/thanhndv212/figaroh-examples/issues/24).
Delivery package: [C1 / core #43](https://github.com/thanhndv212/figaroh-plus/issues/43).
This records the observation semantics and the current behaviour of the TIAGo
geometric calibration reference. It is a capture, not a new reference: the
held-out protocol and truth fixture are C2 (#26, #27).

## Methodology

- Core figaroh-plus `02f705a` (`v0.5.0`); examples `main` at `cbfa8c6`. The
  changed revision is the commit containing this report.
- `figaroh-dev`, Python 3.12.11, Pinocchio 3.7.0, NumPy 2.3.4, macOS arm64;
  meshes from `scripts/fetch_models.py`.
- The calibration runs exactly as `calibration.py --calibrate-only`:
  `TiagoCalibration(robot, "config/tiago_unified_config.yaml", del_list=[])`,
  `known_baseframe = known_tipframe = False`, `initialize()`, `solve()`
  (`least_squares`, method `lm`). Behaviour was observed by instrumenting the
  run (resolved `calib_config`, solver result, spies on outlier detection and
  `robot.q0`), not inferred from code.
- `tests/test_tiago_mocap_baseline.py` pins everything below that is a number.

## Observation semantics

| Item | Value |
|---|---|
| Source | `data/calibration/mocap/qualysis_base_hand_calibration.csv` (sha256 `d7fcbd96e3e67319…`), Qualisys export, 34 static postures |
| Clock | None: no timestamps; each row is an independent posture |
| Observed quantity | Marker 1 position `x1, y1, z1` only (`NbMarkers = 1`, `measurability` xyz); no orientation |
| Units | Metres for marker coordinates (mocap world frame), rad for arm joints, m for the torso |
| Joint order | `torso_lift_joint`, `arm_1_joint` … `arm_7_joint`, matched by column name |
| Kinematic chain | `universe` → `torso_lift_joint` (prismatic) → `arm_1` … `arm_7` (revolute) → fixed `arm_tool_link` → `wrist_ft_link` → `wrist_ft_tool_link` (`end_frame`) |
| Gauge | Mocap-to-robot base: 6D, estimated (`base_px … base_phiz`). Marker in the tool frame: 3D, estimated (`pEEx_1 … pEEz_1`). Both start from zero |
| Joint geometry | 23 identifiable offsets on `arm_1`–`arm_7` (`d_px/py/pz/phix/phiy/phiz_*`); none on the torso |
| Fixed transforms | URDF chain beyond `arm_7_link`; not estimated separately (absorbed into the tip offset) |
| Payload / end effector | Not recorded. The file name says "hand" and the configured sample set is `…_pmb2_hey5.yaml`, but calibration loads `tiago_48_schunk.urdf` (WSG gripper); only the estimated tip offset depends on it |
| Split policy | None: `validation_data_file` is empty; all 34 postures are used for the fit and every reported metric is a training metric |
| Regularisation | `coeff_regularize = 0.01` on the 23 intermediate parameters (TIAGo cost function) |

### What the file actually contains

- Four marker points, but the inter-marker distances vary by only ~0.1 µm
  across postures (angle between edges: std 1.2e-5°). Real optical markers
  carry ~0.1 mm noise, so these points were **computed from a rigid-body
  pose**, not measured individually. The "measured" marker 1 position is
  therefore a processed estimate.
- Marker 4 is an exact copy of marker 3 in all 34 rows.
- All four points move with the hand (spread ≈ 0.27–0.31 m); none is a base
  marker, despite the file name.
- Three distinct, non-collinear points determine a full 6D pose per posture;
  the calibration uses only marker 1's position. This is not a contact-only
  dataset, but 6D information is discarded.

### Inert configuration

- `measurements.poses.base_pose` / `tool_pose` are parsed into `calib_config`
  but never read by calibration (only by the config-migration tool); the frames
  are estimated from zero. Marked inert in the config.
- `parameters.outlier_threshold: 0.05` (m) is recorded but unused: `solve()`
  flags residuals above **3 standard deviations** and does **not** remove them —
  each "outlier removal" iteration re-solves on the full set
  ([figaroh-plus#98](https://github.com/thanhndv212/figaroh-plus/issues/98)).

## Current behaviour (baseline)

**The fit is not unique.** `initialize()` chooses the identifiable parameter
set from a kinematic regressor built on random configurations drawn with
`pin.randomConfiguration`, i.e. from Pinocchio's C++ generator, which
`np.random.seed` does not control and which advances with every call in the
process. Different draws select different 32-parameter sets (1–5 of the 23
joint parameters swap), and the 0.01 regularisation then yields different
calibrated models ([figaroh-plus#99](https://github.com/thanhndv212/figaroh-plus/issues/99)).
Numbers below use `pin.seed(0)` (repeatable: identical over three runs in one
process) and single-threaded BLAS.

| Fit | Training RMSE | Max error | Postures > 5 mm |
|---|---|---|---|
| All parameters zero (no base registration) | 418.9 mm | — | — |
| Registration only: base 6D + tip 3D, CAD joint geometry | ≈ 4.7–4.8 mm | ≈ 11.4–11.6 mm | 10–11 / 34 |
| **Current full fit, `pin.seed(0)`** | **2.71 mm** (x 0.88, y 1.22, z 2.26) | **6.97 mm** | 2 / 34 |
| Current full fit, other draws | 2.61–3.63 mm | 6.27–7.24 mm | — |

(The registration-only fit is started from the full fit's base/tip values, so
its local optimum moves slightly with the draw.)

- Solver: `success`, status 3 (`xtol`), 18–22 evaluations.
- 102 observations for 32 parameters. The measurement Jacobian at the solution
  has full numerical rank but condition number **5 × 10⁸ to 5 × 10⁹** (smallest /
  largest singular value ~10⁻⁹–10⁻¹⁰): the joint offsets are barely
  identifiable and held in place by the regularisation. The largest are about
  0.01–0.02 rad / 1 cm (e.g. `d_phiy_arm_5_joint` 0.0173); they should not be
  read as physical corrections, and which ones exist depends on the draw.
- The joint parameters reduce training error by roughly 40% over registration
  alone; with no held-out set, how much of that generalises is unknown (C2).
- Estimated base frame (`pin.seed(0)`): `base_px` 0.005, `base_py` 0.210,
  `base_pz` −0.332 m; tip offset `pEE` (0.059, 0.001, 0.072) m. Base and tip
  are stable to a few mm across draws.
- Outlier detection flags posture 29, the largest residual, in each of three
  iterations; it stays in the fit.
- `tests/test_tiago_mocap_baseline.py` asserts the observed envelope (RMSE
  2.5–3.8 mm, max 6.0–7.5 mm) rather than the seeded values, because the
  seeded C++ sequence is platform-dependent.

### Side effect found

Running the calibration overwrites `robot.q0`, because `calib_config["q0"]` is
the same array and two loaders write samples into it in place (max change
1.84 rad on the active joints). The fit is unaffected; later users of
`robot.q0` are not. Tracked as
[figaroh-plus#97](https://github.com/thanhndv212/figaroh-plus/issues/97).

## Missing assets and data

- The raw Qualisys capture (individual markers, residuals, timestamps) and the
  rigid-body definition used to compute the points.
- Which end effector and marker plate were mounted, and where marker 1 sits on it.
- Any held-out postures or a second session; recording date and robot unit.
- Configured `sample_configurations_file` (`…_500_pmb2_hey5.yaml`) is a
  500-posture candidate pool, not the 34 recorded postures; how the 34 were
  chosen is unrecorded.

## Changes in this issue

No numerical change. Added: this report, the mocap data contract in
`examples/tiago/data/README.md`, inert-key comments in
`tiago_unified_config.yaml`, and `tests/test_tiago_mocap_baseline.py`
(file layout, rigid-point evidence, observation semantics, current-fit
envelope). The three core defects (#97, #98, #99) are filed, not fixed here.
