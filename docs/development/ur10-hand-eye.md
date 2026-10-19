# UR10 hand-eye calibration: k-fold study

Issue: [#99](https://github.com/thanhndv212/figaroh-examples/issues/99).
Core used for every number below: figaroh-plus devel `b575b5b`.

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
| full map | 48 | 4.022 | 0.403 |
| full map_cv | 48 | 4.012 | 0.449 |
| full excitation | 28 | 4.127 | 0.402 |
| full cv_subset | 25 | 4.004 | 0.472 |

Position and orientation are reported apart and never mixed into one number
(the cost function weights orientation by 0.5, so its mixed residual is not a
position error).

Reading:

- Joint parameters cut the held-out orientation error (0.57 to about 0.40 deg)
  but not the position error: every model sits at about 4 mm. Differences of
  0.1 mm between models are not meaningful with 23 postures (4 to 5 held-out
  rows per fold).
- The model selected on the validation role (lowest mean fold position RMSE)
  is `full map_cv`. The unified config keeps `estimation.method: structural`,
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
4; seeds 0 to 4). Seed 0, level 1 (golden `simulation_seed0`):

| Fit | Params | Held-out position (mm) | Held-out orientation (deg) |
|---|---:|---:|---:|
| registration only | 12 | 6.44 | 0.397 |
| joint_offset | 16 | 5.47 | 0.249 |
| full structural | 30 | 5.04 | 0.082 |
| full map | 48 | 2.88 | 0.072 |
| full map_cv | 48 | 3.13 | 0.081 |
| full excitation | 28 | 7.45 | 0.082 |
| full cv_subset | 25 | 7.45 | 0.082 |

Where the real data sees about 4 mm for every model, the simulation separates
them. Whatever limits the real held-out position error is therefore not the
assumed noise plus joint errors of core's prior size. The full grid and the
z-scores of the map family (which assume an isotropic `noise_std`, so they
run above 1 under this anisotropic noise) come from
`python examples/ur10/hand_eye_truth.py`. A per-component measurement noise
for the estimation methods would remove that caveat; it is not a core option
today.

## 5. Limits

- One session and no repeated posture: k-fold evidence only, assumed noise.
  A second session upgrades the protocol to a full held-out one, with a
  re-registered metric like TIAGo Pro's.
- The sensor is unknown, so the noise grid is a stand-in, not a calibration
  of it.
- No whitening of the residuals and no URDF export from this workflow.

## 6. Reproduction

From the repository root (conda env `figaroh-dev`, single thread):

```bash
python examples/ur10/hand_eye_heldout.py          # k-fold table, about 2.5 min
python examples/ur10/hand_eye_truth.py            # full grid (slow)
python examples/ur10/hand_eye_truth.py --seeds 0 --noise 1
cd examples/ur10 && python hand_eye_reference_run.py --no-plot   # archive
pytest tests/test_ur10_hand_eye_*.py -m "not slow"    # fast checks
pytest tests/test_ur10_hand_eye_*.py -m slow          # golden comparisons
python tests/test_ur10_hand_eye_heldout.py > tests/golden/ur10_hand_eye.json
```
