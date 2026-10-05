# TIAGo calibration: synthetic truth fixture

Issue: [#26](https://github.com/thanhndv212/figaroh-examples/issues/26).
Delivery package: [C2 / core #44](https://github.com/thanhndv212/figaroh-plus/issues/44).
Companion to the [held-out protocol](tiago-mocap-heldout-protocol.md).

On the real mocap sessions nobody knows the true geometry, so a good
held-out error cannot be told apart from luck, and a bad one from
overfitting. This fixture draws a truth, simulates the measurements at the
real postures, fits them with FIGAROH, and judges the fit against the truth.
`examples/tiago/calibration_truth.py` reproduces every number below;
`tests/test_tiago_calibration_truth.py` runs a fast subset in CI.

## 1. Design

| Item | Choice |
|---|---|
| Postures | Joint configurations of the real sessions: training = the 37 postures of 2021-11-30 15:44, held-out = the 184 postures of the other three sessions, with the protocol's posture groups (repeated / new / out of range). Excitation is therefore the real one. |
| Base and tool frames | The real `joint_offset` fit: base (6D) and tool point (3D), constants `FRAMES`. Estimated in every fit, as in the protocol. |
| Truth, `joint_offset` class | One offset per joint: revolute N(0, 20 mrad), torso N(0, 2 mm). |
| Truth, `full_params` class | Six placement errors per joint in the joint frame (figaroh-plus#110): joint-angle term `d_phiz` N(0, 20 mrad), torso `d_pz` N(0, 2 mm), other rotations N(0, 2 mrad), translations N(0, 1 mm). |
| Measurements | Tool point in the base frame from core's `calc_updated_fkm` (the model the fit uses), plus N(0, σ) per axis, σ = 0.5 mm (mocap-like) or 2.0 mm (about the real residual floor). |
| Seeds | Truth seed s; noise seed 10 000 + s. Grid: s = 0…4. |
| Fits | registration only (frames, nominal joints; 9 parameters), `joint_offset` (14), `full_params` (31); the solver and options of `heldout_protocol.py`. |

**Gauge and what is compared.** The base and tool frames absorb some joint
errors (torso and arm_1 offsets at `joint_offset`; `d_pz_arm_7` at
`full_params`, figaroh-plus#102), so fitted parameters are not compared
with the truth one by one. Recovery is judged in identifiable coordinates:

- **prediction**: the fitted model's tool point on the held-out postures
  against the noise-free truth (norm, mm);
- **parameters**, only when the fit has the truth's model class
  (`joint_offset`): each kept offset against its standard error, as
  z = (fitted − true) / SE.

The truth is drawn from the same model class the fit uses, so this measures
estimation and excitation, not model mismatch (no backlash, deflection or
encoder errors; see section 4).

## 2. Results

figaroh-plus `devel` `2631b40`, figaroh-examples `main` `267f42c` plus this
change; `figaroh-dev`, Python 3.12, Pinocchio 3.7.0, macOS arm64,
single-threaded BLAS. `python calibration_truth.py` from `examples/tiago`,
mean over seeds 0–4 (`heldout_max` is the maximum).

| Truth | σ (mm) | Fit | Params | Training RMS | Held-out | New | Out of range | Held-out max |
|---|---|---|---|---|---|---|---|---|
| `joint_offset` | 0.5 | registration only | 9 | 3.100 | 6.092 | 5.234 | 9.786 | 38.4 |
| | | `joint_offset` | 14 | 0.455 | **0.246** | 0.233 | **0.330** | 1.1 |
| | | `full_params` | 31 | 0.413 | 1.166 | 0.668 | 2.748 | 14.6 |
| | 2.0 | registration only | 9 | 3.593 | 6.156 | 5.328 | 9.825 | 38.2 |
| | | `joint_offset` | 14 | 1.818 | **0.985** | 0.933 | **1.318** | 4.6 |
| | | `full_params` | 31 | 1.597 | 3.449 | 2.435 | 7.626 | 33.0 |
| `full_params` | 0.5 | registration only | 9 | 2.364 | 4.573 | 3.817 | 7.444 | 25.1 |
| | | `joint_offset` | 14 | 0.684 | 1.158 | 0.921 | **2.179** | 5.9 |
| | | `full_params` | 31 | 0.408 | **1.090** | **0.656** | 2.553 | 11.9 |
| | 2.0 | registration only | 9 | 3.000 | 4.648 | 3.927 | 7.485 | 25.5 |
| | | `joint_offset` | 14 | 1.913 | **1.497** | **1.338** | **2.482** | 6.6 |
| | | `full_params` | 31 | 1.593 | 3.228 | 2.396 | 7.009 | 24.1 |

Training RMS and prediction errors in mm. Training RMS is per axis; the
held-out columns are norms. With 111 residuals and 14 parameters, a correct
`joint_offset` fit leaves √(97/111) ≈ 0.93 σ per axis: 0.47 and 1.87 mm.

**Parameter recovery** (`joint_offset` truth and fit, both noise levels, 5
seeds, 5 kept offsets each): 50 z-scores, mean +0.03, RMS 0.82, max |z|
1.80. The offsets are unbiased and their standard errors (figaroh-plus#107)
are calibrated, slightly conservative.

## 3. What this supports

- **FIGAROH recovers a known truth when the model class matches and the
  postures excite it.** Noise-free, `joint_offset` reproduces the truth to
  < 1 µm on every held-out posture; with noise, held-out prediction error is
  about half the per-axis noise and the offsets fall within their standard
  errors.
- **37 postures do not support 31 parameters.** Even when the truth is a
  `full_params` robot, the 31-parameter fit is only marginally better than
  `joint_offset` at 0.5 mm noise (1.09 vs 1.16 mm overall, 0.66 vs 0.92 mm
  on new postures) and worse out of range (2.55 vs 2.18 mm); at 2 mm noise
  it is twice as bad everywhere (3.23 vs 1.50 mm). Weakly excited directions
  are fitted to noise and extrapolate badly (held-out maximum 24 mm).
- **Selection is an excitation question, not only an identifiability one.**
  Parameters that are identifiable in principle should not all be estimated
  freely from these postures. This is the evidence for figaroh-plus#113: a
  selection rule (excitation threshold, or a prior) should be chosen on this
  fixture, where the truth is known, and not on the real held-out sets
  (protocol rule 2).

## 4. What it does not support

- **Model mismatch is absent.** The real sessions have a 2–3 mm residual
  floor from effects this fixture does not simulate (arm_6 backlash, arm_5
  encoder behaviour, #71). On real data `full_params` beats `joint_offset`
  on held-out postures (protocol section 4); here it does not. So the real
  robot either has larger placement errors than `TRUTH_SIGMA`, or
  `full_params` partly absorbs unmodelled effects. The fixture cannot tell
  which; adding mismatch (e.g. a backlash model) is a follow-up.
- **One posture plan.** Results are for the real 37-posture training plan.
  A designed plan (figaroh `optimal`) may support more parameters.
- **One marker point, position only**, as in the real sessions.
- **Priors are assumptions.** `TRUTH_SIGMA` sets how large the truth errors
  are; conclusions about `full_params` depend on it.
