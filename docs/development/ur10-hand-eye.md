# UR10 hand-eye calibration: k-fold study

Issue: [#99](https://github.com/thanhndv212/figaroh-examples/issues/99).
Core used for every number below: figaroh-plus devel `521c69c` (per-component
measurement noise, figaroh-plus#215/#223; numbers regenerated for #176).

This brings the UR10 hand-eye data to the C5 standard of
[TIAGo Pro](tiago-pro-mocap.md) and [TALOS](talos-table-contact.md): a frozen
split, explicit estimation methods, a registration baseline, a synthetic-truth
study, an archived reference run and a golden output. Code is in
`examples/ur10/` (`hand_eye_heldout.py`, `hand_eye_truth.py`,
`hand_eye_reference_run.py`); checks are `tests/test_ur10_hand_eye_*.py`; the
golden values are `tests/golden/ur10_hand_eye.json`. The dynamic-identification
fixture of the same robot ([truth fixture](ur10-dynamic-truth-fixture.md)) is
independent and untouched.

## 1. Data and defects

`data/calibration.csv` is 23 postures from one session: a 6D pose of a camera
(rigid on the flange) in a fixed frame, plus the six joint angles. Frozen in
`data/hand_eye_protocol.yaml` (protocol v1, sha256 checked on import). The
column units and the frame convention are in
[`data/README.md`](../../examples/ur10/data/README.md).

- **One session.** The only held-out evidence is a k-fold split of the rows
  ("k-fold within session; weaker than held-out sessions"). Folds share the
  camera, the board placement and the recording day, so nothing here measures
  how the frames carry over to another day.
- **No repeats.** The nearest pair of postures differs by at least 0.7 rad on
  some joint, so the file gives no within-session noise estimate. The
  simulation therefore assumes its noise (section 4).
- **Unknown:** camera, board, detection method, recording date, orientation
  and depth accuracy.
- `data/simulation.csv` (29 postures) has no recorded provenance and is not
  used here.

## 2. Frame convention and gauge

The fit is core's `T = B * FK(q) * T_tip`. The base parameters `B` are the
robot base in the fixed frame; `T_tip` is the flange-to-camera transform. Only
their product with the joint errors is observable: the wrist-3 translation
along z and rotation about z are absorbed by the tool frame
(`absorbed_param_name`: `d_pz_wrist_3_joint`, `d_phiz_wrist_3_joint`), and
structural fits return base-parameter representatives that are not individual
joint errors. The registration fit puts the base within 6.4 cm and 1.4 deg of
identity, so the poses are already close to the robot base frame.

## 3. Models and the held-out result

All models start from `full_params`; the baseline drops the joint parameters
(12 frame parameters). Core's default prior sizes (1 mm, 2 mrad, 0.02 rad)
are used by the regularised methods. Each fold fits on 4/5 of the postures
with a single fit (no outlier exclusion, so the folds do not change) and
judges the held-out rows with the training frames carried over. Pooled over
the five folds (every posture held out once; golden `kfold`):

| Model | Params | Position RMSE (mm) | Orientation RMSE (deg) |
|---|---:|---:|---:|
| registration only | 12 | 4.125 | 0.570 |
| joint_offset | 16 | 4.090 | 0.598 |
| full structural (ordered) | 30 | 4.152 | 0.402 |
| full map | 48 | 3.975 | 0.414 |
| full map_cv | 48 | 3.975 | 0.414 |
| full excitation | 29 | 4.093 | 0.413 |
| full cv_subset | 25 | 3.953 | 0.414 |

Position and orientation are reported apart and never mixed into one number
(the cost function weights orientation by 0.5, so its mixed residual is not a
position error).

Reading:

- Joint parameters cut the held-out orientation error (0.57 to about 0.40 deg)
  but not the position error: every model sits at about 4 mm. Differences of
  0.1 mm between models are not meaningful with 23 postures (4 to 5 held-out
  rows per fold).
- The model selected on the validation role (lowest mean fold position RMSE)
  is `full cv_subset` (mean 3.78 mm against 3.82 mm for `map` and `map_cv`);
  it was `full map_cv` before the regularised fits estimated their noise per
  component, a change of a few hundredths of a millimetre. The unified config keeps `estimation.method: structural`,
  the unchanged reference fit; the k-fold result does not justify changing it.
- The 4 mm floor is not explained by what the study can see: it is not
  reduced by more parameters. A measurement-side systematic (board detection,
  camera model, a pre-applied frame change) is a candidate; a second session
  would show it.

## 4. Synthetic-truth study

`hand_eye_truth.py` draws a truth (the camera and base frames of the real
registration fit plus seeded `full_params` joint errors of the default prior
size), simulates a session at the 23 real postures and judges each fit on 100
seeded postures inside the real joint ranges, against the noise-free truth,
with the training frames carried over. Postures outside the recorded range
are not tested.

The noise is **assumed, not measured**: translation in the camera frame with
sigma 0.3 mm in the image plane and 1.5 mm in depth, rotation in the fixed
frame with 0.2 deg tilt and 0.05 deg in-plane, times a level (grid 0.5, 1, 2,
4; seeds 0 to 4).

**Noise model of the fits** (#176). The regularised fits (`map`, `map_cv`,
`excitation`, `cv_subset`) whiten their residuals with core's per-component
`noise_std` and a per-sample `noise_rotation`. The cost function residual is
`log(M_meas⁻¹ M_est)` with the position in the camera (body) frame and the
orientation weighted by 0.5, so camera-frame translation noise enters as an
identity block and fixed-frame rotation noise `w` as `R_measᵀ w`: the
rotation of sample i is `block_diag(I3, R_meas,iᵀ)`, and the residual sigma is
the level times (0.3, 0.3, 1.5 mm, 0.5 × (0.2, 0.2, 0.05 deg)).
`hand_eye_truth.py` passes that known vector; `hand_eye_heldout.py` (real
data, noise unknown) and `hand_eye_truth.py --estimated-noise` use
`noise_std: per_component`, which estimates it from the residuals. These
frames are the synthetic study's assumption, not a measured property of the
sensor. Structural fits ignore the noise.

Seed 0, level 1 (golden `simulation_seed0`):

| Fit | Params | Held-out position (mm) | Held-out orientation (deg) |
|---|---:|---:|---:|
| registration only | 12 | 6.44 | 0.397 |
| joint_offset | 16 | 5.47 | 0.249 |
| full structural | 30 | 5.04 | 0.082 |
| full map | 48 | 2.88 | 0.065 |
| full map_cv | 48 | 2.88 | 0.065 |
| full excitation | 29 | 6.73 | 0.070 |
| full cv_subset | 27 | 6.72 | 0.089 |

Full grid, held-out position RMSE in mm averaged over seeds 0 to 4
(`python examples/ur10/hand_eye_truth.py`, 140 fits, about 20 min
single-threaded on core `devel` 521c69c). Each cell is the known-noise fit,
with the scalar `noise_std` fit (the code before #176) in brackets; the
`per_component` column is `--estimated-noise` at ×1:

| Fit | ×0.5 | ×1 | ×2 | ×4 | ×1 `per_component` |
|---|---:|---:|---:|---:|---:|
| registration only | 19.3 | 19.3 | 19.2 | 19.3 | 19.3 |
| joint_offset | 5.46 | 5.58 | 6.00 | 7.31 | 5.58 |
| full structural | 10.9 | 21.7 | 43.3 | 85.6 | 21.7 |
| full map | **1.44** (1.52) | **1.68** (1.94) | **2.33** (2.99) | 3.79 (5.31) | **1.70** |
| full map_cv | 2.80 (4.43) | 3.68 (5.05) | 4.03 (3.12) | **3.76** (5.25) | 4.17 |
| full excitation | 2.30 (2.34) | 2.59 (2.85) | 3.30 (2.98) | 6.37 (8.42) | 2.59 |
| full cv_subset | 4.14 (9.60) | 6.35 (15.0) | 10.8 (16.7) | 19.6 (30.3) | 6.34 |

Held-out orientation of `full map` at ×1 drops from 0.112 to 0.077 deg.

- `map` is the most accurate up to ×2 and level with `map_cv` at ×4.
  Whitening with the right noise helps most where the scalar fit was worst:
  `cv_subset` halves at ×0.5 and ×1, `map` at ×4 goes from 5.31 to 3.79 mm.
  Estimating the noise (`per_component`) recovers nearly all of it for
  `map` (1.70 against 1.68 mm); `map_cv` gains less. It is not a gain in
  every cell: at ×2, `map_cv` (3.12 to 4.03 mm) and `excitation` (2.98 to
  3.30 mm) are worse with the vector noise, a reminder that these
  regularised fits choose their prior scale from 23 postures.
- Full `structural` fits the 23 training postures to the noise level
  (0.27 mm training RMS at ×0.5), then grows linearly with noise on new
  postures (10.9 to 85.6 mm, max 211 mm at ×4): 30 free parameters on 23 6D
  postures extrapolate badly inside the joint range. `cv_subset` inherits
  this whenever it keeps nearly all parameters. Seed 0 (table above) is one
  of the mild draws; the spread across seeds is large.
- The registration-only baseline stays near 19 mm whatever the noise, since
  it cannot absorb the joint errors.

Where the real data sees about 4 mm for every model, the simulation separates
them. Whatever limits the real held-out position error is therefore not the
assumed noise plus joint errors of core's prior size. Its k-fold held-out sets
(4 to 5 postures each, near the training postures) are also far less
demanding than the 100 simulated judge postures, which is why the real data
does not show the structural overfit. On the real rows, `per_component`
estimates about 2.7 / 1.8 / 1.1 mm and 0.18 / 0.20 / 0.27 deg per residual
component: noise plus whatever the model misses, far above the assumed
sensor noise.

Joint-level recovery of the `full_params` fits, z = (estimate − truth) /
the fit's own standard error, RMS per method over the grid:

| Fit | scalar `noise_std` | known vector + rotation |
|---|---:|---:|
| full map | 0.85 | 0.87 |
| full map_cv | 0.79 | 0.76 |
| full cv_subset | 10.5 | 13.2 |

`map` and `map_cv` are already calibrated (z RMS near 0.8) with the scalar
noise; per-component noise leaves that unchanged. The pooled RMS (4.56 with
the scalar noise, 5.76 with the vector; max |z| 88) comes from `cv_subset`,
whose standard errors ignore the bias of the parameters it drops: its z RMS
falls with the noise level (9.25 at ×1 to 2.92 at ×4, scalar), because the
dropped parameters' bias matters less against larger noise. What
per-component noise improves is the held-out accuracy above, not the
calibration of the standard errors.

## 5. Limits

- One session and no repeated posture: k-fold evidence only, assumed noise.
  A second session upgrades the protocol to a full held-out one, with a
  re-registered metric like TIAGo Pro's.
- The sensor is unknown, so the noise grid is a stand-in, not a calibration
  of it.
- The noise frames (camera-frame position, fixed-frame orientation) are
  assumed; the real-data fits estimate the per-component size, not the frame.
- No URDF export from this workflow.

## 6. Reproduction

From the repository root (conda env `figaroh-dev`, single thread):

```bash
python examples/ur10/hand_eye_heldout.py          # k-fold table, about 2.5 min
python examples/ur10/hand_eye_truth.py            # full grid (slow)
python examples/ur10/hand_eye_truth.py --seeds 0 --noise 1
python examples/ur10/hand_eye_truth.py --noise 1 --estimated-noise
cd examples/ur10 && python hand_eye_reference_run.py --no-plot   # archive
pytest tests/test_ur10_hand_eye_*.py -m "not slow"    # fast checks
pytest tests/test_ur10_hand_eye_*.py -m slow          # golden comparisons
python tests/test_ur10_hand_eye_heldout.py > tests/golden/ur10_hand_eye.json
```
