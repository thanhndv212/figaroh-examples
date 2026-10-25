# TX40 calibration: synthetic truth study

There are no TX40 calibration measurements. This study draws a truth,
simulates the tool poses a tracker would record, fits them with FIGAROH and
judges the fit against the truth. Everything here is **simulated**: it
checks the pipeline and the posture design at a stated noise level, it does
not estimate any TX40's geometry.

`examples/staubli_tx40/calibration_truth.py` reproduces every number below;
`tests/test_tx40_calibration_truth.py` runs a fast subset in CI. The study
uses the shared [calibration harness](calibration-harness.md), with the
rules of the [TIAGo study](tiago-calibration-synthetic-truth.md).

## 1. Design

| Item | Choice |
|---|---|
| Postures | Training: 66 postures, as two designs drawn from the 1000 seeded candidates of `optimal_config.py` (joints 1-6 uniform in the URDF limits less 0.05 rad, non-adjacent self-collision rejected). `doptimal` is the D-optimal selection, frozen in `data/truth/calibration_doptimal_postures.csv` (`--design` reruns it). `random` is a seeded random subset of the same size. Held-out: 300 fresh candidates (sampler seed 1). |
| Base and tool frames | Robot on a table in a tracker frame (45 cm, -30 cm, 2 cm; 50 mrad yaw), probe 12 cm off the flange (`FRAMES`). Estimated in every fit. |
| Truth, `joint_offset` class | One offset per joint, N(0, 5 mrad). |
| Truth, `full_params` class | Six placement errors per joint: joint angle `d_phiz` N(0, 5 mrad), other rotations N(0, 1 mrad), translations N(0, 0.3 mm). |
| Measurements | Tool pose (position and orientation) from core's `calc_updated_fkm`, plus N(0, σ) per position axis in mm and per rotation component in mrad, σ = 0.1 (laser tracker) or 0.5 (mocap). |
| Seeds | Truth seed s, noise seed 10 000 + s, s = 0…4. |
| Fits | Registration only (frames, nominal joints; 12 parameters), `joint_offset` (16), `full_params` (30). |
| Score | Tool position at the held-out postures against the noise-free truth (norm, mm). Training RMS is per position axis. |

## 2. Core finding: the default selection drops an identifiable parameter

A 6R arm measured in full pose has 4N + 6 = 30 identifiable kinematic
parameters. With core's default structural selection (LAPACK pivoting) the
TX40 fit keeps 29, and a noise-free `full_params` truth leaves 0.16 mm of
training residual.

The cause is in core's `add_base_name`. At `full_params` it renames the
first six structural parameters to the base frame, assuming they are joint
1's six placement errors. On the TX40, LAPACK groups joint 1's yaw with
joint 2 (`d_phiy_joint_2 - 1.0*d_phiz_joint_1`), so joint 1 contributes five
entries and the sixth renamed one is `d_px_joint_2`. That parameter is
independent of the other 29 (98 % of its column remains after projection,
and adding it leaves the conditioning unchanged), so a truth with a
`d_px_joint_2` error cannot be fitted.

With `estimation.structural_selection: ordered` the first independent
columns are taken in joint order, joint 1's six lead the list, and the
renaming is right: 30 parameters, noise-free recovery to 1e-13 mm. The TX40
config now sets it. Held-out error, `full_params` truth and fit, D-optimal
design, mean over seeds 0-4:

| Selection | Params | σ = 0 | σ = 0.1 mm |
|---|---|---|---|
| core default (`lapack`) | 29 | 0.282 mm | 0.287 mm |
| `ordered` (TX40 config) | 30 | < 1e-6 mm | **0.044 mm** |

The fix belongs in core (`add_base_name` should replace the parameters of
the first joint, not the first six entries). Until then, any robot whose
first joint shares a structural group with the second needs `ordered`.

## 3. Results

figaroh-plus `devel` `cf0f7a8`, figaroh-examples `main` `1dac70f` plus this
change; `figaroh-dev`, Python 3.12, Pinocchio 3.7.0, macOS arm64.
`python calibration_truth.py` from `examples/staubli_tx40` (about 3 min),
mean over seeds 0-4 (`max` is the maximum).

| Design | Truth | σ | Fit | Params | Training RMS | Held-out | Held-out max |
|---|---|---|---|---|---|---|---|
| D-optimal | `joint_offset` | 0.1 | registration only | 12 | 1.265 | 2.454 | 7.67 |
| | | | `joint_offset` | 16 | 0.094 | **0.028** | 0.069 |
| | | | `full_params` | 30 | 0.092 | 0.044 | 0.093 |
| | | 0.5 | registration only | 12 | 1.368 | 2.456 | 7.60 |
| | | | `joint_offset` | 16 | 0.470 | **0.142** | 0.344 |
| | | | `full_params` | 30 | 0.460 | 0.220 | 0.465 |
| | `full_params` | 0.1 | registration only | 12 | 1.264 | 2.364 | 9.03 |
| | | | `joint_offset` | 16 | 0.733 | 1.370 | 3.76 |
| | | | `full_params` | 30 | 0.092 | **0.044** | 0.093 |
| | | 0.5 | registration only | 12 | 1.342 | 2.369 | 9.07 |
| | | | `joint_offset` | 16 | 0.851 | 1.380 | 3.75 |
| | | | `full_params` | 30 | 0.460 | **0.220** | 0.467 |
| random | `joint_offset` | 0.1 | `joint_offset` | 16 | 0.094 | 0.024 | 0.064 |
| | | | `full_params` | 30 | 0.093 | 0.039 | 0.118 |
| | | 0.5 | `joint_offset` | 16 | 0.472 | 0.121 | 0.321 |
| | | | `full_params` | 30 | 0.464 | 0.195 | 0.588 |
| | `full_params` | 0.1 | `full_params` | 30 | 0.093 | 0.039 | 0.118 |
| | | 0.5 | `full_params` | 30 | 0.464 | 0.195 | 0.589 |

Errors in mm. The random rows omit the underfitted fits, which match the
D-optimal ones within 0.1 mm (full table: run the script).

**Parameter recovery** (`joint_offset` truth and fit, both designs and noise
levels, 5 seeds, offsets of joints 2-5): 80 z-scores, mean +0.01, RMS 0.93,
max |z| 2.35. Joints 1 and 6 go to the base and tool frames.

## 4. What this supports

- **The TX40 pipeline recovers a known geometry** once the selection is
  ordered: noise-free to < 1 µm, and with noise the held-out tool error is
  about half the per-axis noise at `full_params`, a third at `joint_offset`.
- **66 postures support all 30 parameters.** Unlike TIAGo's 37 postures
  (where `full_params` overfits), the full model costs only 0.016 mm over
  `joint_offset` on a `joint_offset` robot, and gains 1.3 mm on a
  `full_params` one. The safer default is `full_params`.
- **The D-optimal design does not beat random sampling on average.** On the
  same 1000 candidates, 66 random postures give a slightly *lower* held-out
  RMS (0.039 against 0.044 mm at σ = 0.1) and the D-optimal set a lower
  worst case (0.093 against 0.118 mm). D-optimality maximises the
  determinant of the parameter information matrix, which mixes metres and
  radians, and it pushes joints 2, 3 and 5 outward (mean |q| 0.64-0.71 of
  the half-range, against about 0.5 for random); the score here is tool
  position at uniformly drawn postures. Its use is guaranteeing
  identifiability with fewer postures, not better average prediction.

## 5. Limits

- Same model class for truth and fit: no compliance, gear backlash or
  encoder nonlinearity, so these numbers are a floor on what real data
  would give.
- Noise is isotropic and independent; real trackers are anisotropic and the
  probe orientation is noisier than its position.
- Postures are checked against self-collision only; the URDF has no table
  or cell.
- Tool orientation is fitted but not scored (its roll-pitch-yaw columns
  wrap at ±π; `SimulationStudy.train_residual` keeps the training RMS to
  positions).
