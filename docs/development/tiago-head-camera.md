# TIAGo head camera, chessboard on the hand (#101): pre-registration

Issue: [#101](https://github.com/thanhndv212/figaroh-examples/issues/101).
This note is part 1: the pre-registration, merged before any real joint-level
fit. It is frozen. The study (real-data fits, comparison with the motion
capture on the same plan, verdicts) is part 2 and changes no threshold written
here.

Question: can the head camera replace motion capture for calibrating the TIAGo
arm, when both see a chessboard held by the hand? The comparison that decides
is **same-day**: the 2023-10-27 runs, where camera and motion capture observed
the same postures. This note fixes the model, the gauge, the metrics and the
bands **before any joint offset is fitted to a real session**, from simulation
only.

## 1. What this part contains, and what it does not

- The unified configuration `examples/tiago/config/tiago_head_camera_config.yaml`
  (held parameters, noise ratios and grid, the fixture and its pass criteria).
- The library `examples/tiago/head_camera.py` (two-chain model, residuals,
  gauge-fixed fit, re-registration, leave-one-out, effective noise) and the
  fixture `examples/tiago/head_camera_truth.py`.
- The thresholds below. They come from the fixture, which fits only
  **simulated** measurements.

Disclosure of the real data read:

- the registration-only noise stage (section 5) on the training session
  (2023-11-27 `center`) and the diagnostic `topleft`: it registers the board
  frame `Y` at the nominal joints and fits no joint offset;
- the median `pose_std_mm` of the three 2023-11-07 mocap runs (diagnostic);
- the **joint columns** and the **`posture` pairing index** of the 2023-10-27
  runs (to know the postures and which of them both sensors saw: 11, 11, 8);
  the camera and mocap **pose values** of 2023-10-27 are never read, and
  `head_camera.read_camera` refuses them until the confirmation step;
- the training session's registration `Y0` at nominal joints, the common
  starting board frame of the fixture (the fixture reads nothing else).

A scratch computation during design (a rank check of the Jacobian at the
nominal joints on the posture columns of 11-27 and 10-27, no pose values) set
the gauge of section 3. While writing the tests, one throw-away command fitted
the naive 22-parameter set to the real training session to look at the
standard errors; its output was not stored and no decision depended on it (the
test that remains uses simulated data).

## 2. Roles

Protocol v2 of `examples/tiago/data/calibration/head_camera/protocol.yaml`,
unchanged: training `2023-11-27-center` (17 postures), validation
`2023-11-23-distortion_original` (17), confirmation `2023-10-27-test1/2/3`
(camera and mocap, 12 postures each; 11, 11 and 8 paired with mocap).
Diagnostic: 11-27 `topleft`, the 11-07 day, the other 11-23 variants.
`2023-11-24-test_optimal` is another robot (tiago-155c) and is not evaluated.

The role sets are built in `head_camera.role_sets` (the harness
`parse_heldout` has no `diagnostic` role and treats each file as its own
set). The guard against selecting on the confirmation runs is kept:
`select_model` delegates to `calibration_harness.heldout.select`, which
refuses `confirmation`, and `read_camera` refuses their pose values. No
`protocol.yaml` change.

## 3. Gauge

Both sensors give the board pose in a sensor frame. For the camera,
`M = (T_cam(q + d) X)^-1 T_hand(q + d) Y`, with `T_cam` the pose of
`xtion_optical_frame` (child of `head_2_link`) and `T_hand` that of
`arm_7_link`, from the nominal URDF `tiago_48_hey5.urdf`. `X` is the correction
of the camera in `head_2_link`; `Y = Y0 se3(y)` is the board on `arm_7_link`.
Both chains hang off `torso_lift_link`, so only the relative pose matters.

The naive set `{d_torso, d_head_1, d_head_2, d_arm_1 .. d_arm_7, X, Y}` has 22
parameters and **rank 19** (numerical Jacobian, `rtol 1e-8`, every session):

1. `d_torso` alone: both chains move together; the torso offset has no effect.
2. `d_head_2` with `X`: `T(q + e) = T Rz(e) F`, so `X <- F^-1 Rz(-e) F X`
   keeps the pose (the group orbit, computed in the fixture).
3. `d_arm_7` with `Y`: the same.
4. Near-null: `d_head_1 + d_arm_1`. Both axes are vertical and 30 mm apart, so
   moving both is a pure translation by the 30 mm lever: `sigma / sigma_max`
   about 5e-3 and **no rotation signature**. Only `d_arm_1 - d_head_1` is
   observable.

Gauge fix: hold `d_head_1 = d_head_2 = 0` (nominal), drop `d_torso` and
`d_arm_7`, and fit `G = {d_arm_1 .. d_arm_6, X, Y}`, 18 parameters, full rank,
condition number 13.2 on the training postures (rotation rows at the
fixed sigma). `d_arm_1` is reported as **arm_1 relative to head_1**. The
held-out predictions do not depend on the fix (F1 below). The comparable
parameters between sensors are `d_arm_1 .. d_arm_6`. Re-rooting the chain on
the camera gives the same quotient.

## 4. Model, residual, depth scale

Residual (primary, 6D): `[p_meas - p_pred ; log(R_pred R_meas^T)]`, with
`log(R_pred R_meas^T) = R_meas log(R_meas^T R_pred) R_meas^T`, the rotation of
`log(R_meas^T R_pred)` expressed in the camera frame (tilt about the camera x,
y axes, roll about z). It is whitened by a **fixed diagonal sigma** (xy, xy, z,
tilt, tilt, roll) = `level` x (r_xy, r_xy, 1, r_tilt, r_tilt, r_roll), the
ratios frozen from the noise stage (section 5). Rotation matrices only: the
orientation columns are roll-pitch-yaw near +-pi, and angles are never
differenced. Secondary (3D): position rows only, `Y` reduced to a point.
Millimetres and milliradians are always reported separately.

Re-registration keeps the joint offsets and refits `X` and `Y` (12 parameters)
on a session; the leave-one-out re-registered RMSE is the transfer metric
(as in #100). Mocap uses the same chain with its own frames (the camera body
for `X`, the board body for `Y`), isotropic sigma from the plateau
statistics, rotation sigma = position sigma / 0.1 m (an assumption, no
rotation noise is recorded).

Depth scale `k` (`z_cam x k`, one per camera session): the camera intrinsics
differ between sessions (fx 528.73 on 11-27, 536.25 on 10-27 and PAL), about a
1.4 % depth difference, 5 to 7 mm at 0.5 m. `k` is decided by simulation only
(F4, section 6): **k is in** for the study.

## 5. Effective noise

Registration only, no joint fit. Camera: 11-27 `center` against `topleft`
(same day, same intrinsics), postures paired within 20 mrad on every moving
joint (one-to-one), `Y` registered per session at the nominal joints and
`X = I` (position weight 1, rotation 0.5 m/rad); `d = r_center - r_topleft`
per axis and `sigma^2 = sum d^2 / (2 (n - 1))` (the registration removed 6
parameters per session). Mocap: the median plateau `pose_std_mm` of the
2023-11-07 runs 1444, 1516 and 1528 (the 10-27 mocap columns are not read).
The grid level is the smallest of {0.5, 2, 4} mm at or above `sigma_z`.

| quantity | value |
|---|---|
| `noise camera` | 17 pairs: xy 1.34 mm, z 1.38 mm, tilt 3.3 mrad, roll 2.2 mrad; grid level 2.0 mm |
| `noise mocap` | plateau pose_std 0.76 mm; grid level 2.0 mm |
| `noise ratios` | xy/z 0.973; tilt 2.400 and roll 1.559 mrad per mm of sigma_z |

The ratios are frozen in the configuration; the used level is 2.0 mm.

## 6. Pre-registered thresholds (simulated)

Fixture: one truth per seed `default_rng(seed)` (offsets of torso, head and
arm ~ N(0, 10 mrad); `X` = identity plus a 10 mm / 20 mrad perturbation; `Y`
per session = `Y0` plus the same, the re-grasp), measured at the real postures
of training, validation and 10-27 (paired rows). Noise stream
`default_rng(10000 + 100 k + seed)`, `k` = 0 camera, 1 mocap, 2 depth scale.
Seeds 0, 1, 2, 3, 4, grid 0.5, 2.0, 4.0 mm. The mocap twin has fixed synthetic
frames (`x_mocap`, `y_mocap` in the configuration). All pass criteria are in
the configuration and were written before the first run.

### 6.1 F1 to F3: gauge, fixed fit, held head

| criterion | pass rule | result |
|---|---|---|
| F1 rank | 19 of 22, every seed and session, `rtol 1e-8` | pass |
| F1 null space | principal angles to the analytic null directions < 1e-3 | pass |
| F1 near-null | `sigma_19 / sigma_1 < 0.02`, concentrated on head_1 + arm_1, no rotation signature (analytic direction, rotation rows / all rows < 1e-3) | pass |
| F1 naive fit | SE of `d_head_2`, `d_arm_7` > 1e3 x the fixed fit's; LOO RMSE changes by < 0.1 x level (here 0.09 mm) | pass |
| F2 recovery | pooled z-RMS in [0.75, 1.33], abs(mean z) < 0.3, validation error < 1 x level | pass |
| F3 held head | same z criteria with 11 mrad head offsets in the truth, predictions within 0.1 x level of F2 | pass |

Implementation notes. The near-null rotation signature is measured on the
analytic direction `d_head_1 + d_arm_1`, not on the SVD vector (which carries
small admixtures of other parameters and gave 0.2); the threshold (1e-3) did
not change. Standard errors come from a central-difference Jacobian at the
solution (the forward-difference Jacobian of the optimiser was too coarse to
separate an exact null from a weak direction). A naive fit on data is free to
drift along its null directions, so only its standard errors and predictions
are used.

Recovery of `d_arm_2 .. d_arm_6` and `d_arm_1 - d_head_1` (z-scores,
(estimate - truth) / SE, pooled over seeds; validation error is the RMS
position error of the re-registered prediction against the noise-free truth):

| | 0.5 mm | 2.0 mm | 4.0 mm |
|---|---|---|---|
| `recovery z rms` | 1.05 | 1.06 | 1.06 |
| `recovery z mean` | -0.20 | -0.18 | -0.18 |
| `validation error` | 0.39 mm | 1.38 mm | 2.74 mm |
| `recovery z rms (F3)` | 1.06 | 1.06 | 1.06 |
| `F3 vs F2 prediction` | 0.061 mm | 0.022 mm | 0.016 mm |
| `fixed-fit cond` | 13.2 | 13.2 | 13.3 |

### 6.2 F4: the depth-scale rule

Training depth scaled by `s = 528.73 / 536.25 = 0.9860`; the model fitted
without and with `k`. `k` enters only if the fit without it is biased by more
than one standard error on some comparable parameter **and** `k` raises the
condition number by less than 2x. Used level 2.0 mm, bias = mean error over seeds:

| parameter | bias / SE, no k | bias / SE, no scale at all (noise level) | bias / SE, with k (no-k SE) |
|---|---|---|---|
| arm_2 | 1.09 | 0.33 | 0.14 |
| arm_3 | 0.63 | 0.06 | 0.09 |
| arm_4 | 0.34 | 0.63 | 0.71 |
| arm_5 | 0.16 | 0.58 | 0.69 |
| arm_6 | 0.03 | 0.61 | 0.78 |
| arm_1_minus_head_1 | 0.12 | 0.21 | 0.29 |

| level | decision | max bias / SE | cond no k / with k |
|---|---|---|---|
| `depth scale k @ 0.5 mm` | in | 3.31 | 13.2 / 20.5 |
| `depth scale k @ 2.0 mm` | in | 1.09 | 13.2 / 20.5 |
| `depth scale k @ 4.0 mm` | out | 0.73 | 13.2 / 20.6 |

Decision at the used level: **k is in** (`k_hat` 0.982 against `s` 0.986; cond 13.2 to 20.5). The
bias at the used level is marginal (largest ratio 1.09, the same statistic
without any scale reaches 0.63); it is clear at 0.5 mm and absent at 4 mm. The rule is
applied as written. `k` is fitted per camera session; the mocap side has none.

### 6.3 F5: parameter bands (mrad, camera minus mocap twin, `d_arm_1 .. d_arm_6`)

`band_j = 2 RMS(theta_cam - theta_mc)` over seeds and the three runs, fits on the
paired postures (11, 11, 8) of each run.

| band (mrad) | 0.5 mm | 2.0 mm | 4.0 mm |
|---|---|---|---|
| `band arm_1` | 5.3 | 21.5 | 43.3 |
| `band arm_2` | 2.7 | 10.9 | 21.7 |
| `band arm_3` | 2.0 | 7.3 | 14.6 |
| `band arm_4` | 1.5 | 6.2 | 12.2 |
| `band arm_5` | 2.7 | 9.9 | 19.9 |
| `band arm_6` | 3.0 | 12.2 | 24.2 |

At the used level the bands are of the order of the offsets themselves (10 mrad
scatter, PAL's own values reach 20 mrad): with 8 to 11 postures and a 2 mm sensor, individual
`d_arm_j` of the two sensors can differ by that much from noise alone.

### 6.4 F5: transfer bands

`G = e(M_A, E) - e(M_same, E)`: the LOO re-registered RMSE of sensor A's model on
run E (a different run of the same day) minus that of E's own sensor, 6 ordered
pairs per seed, both directions. Band = `abs(mean G) + 2 sd G`, position (mm)
and rotation (mrad) separately.

| band | 0.5 mm | 2.0 mm | 4.0 mm |
|---|---|---|---|
| `transfer cam_to_mc` | 0.67 mm, 1.2 mrad | 2.75 mm, 4.8 mrad | 5.57 mm, 9.7 mrad |
| `transfer mc_to_cam` | 0.66 mm, 2.0 mrad | 2.67 mm, 8.0 mrad | 5.33 mm, 16.1 mrad |

### 6.5 C0: sensor-agreement floor

C0 fits `M_cam = X' M_mc Y'` (12 parameters, no kinematics) per run on the
paired postures. At the used level the residual floor per axis is
x 2.32, y 2.65, z 2.33 mm and
18.0, 18.6, 20.1 mrad. The recovery error of `X'`, `Y'` (RMS over seeds and runs) is
11.3 mm / 8.3 mrad for `X'` and 7.9 mm / 13.0 mrad for `Y'`: the 10-27 postures
constrain these frames only loosely, so C0 is a check of the residual and of
consistency across runs, not a calibration.

### 6.6 F6: arm_5 slip (descriptive)

A 5 mrad `arm_5` shift between test1 and test2/test3 (the slip seen in other
sessions) is recovered as 4.3 mrad by the camera and 9.1 mrad by the
mocap twin (test2 minus test1, mean over seeds). It changes the transfer bands little:
camera to mocap 2.75 mm against 2.75 mm
without it. It is not a pass criterion.

## 7. Verdict rules (written before the real fit)

Primary verdict: the same-day cross-run comparison on 2023-10-27. Bands only,
no absolute millimetre margin. For a parameter or transfer quantity with band
`b` and observed gap `g`:

- **beyond band**: `abs(g) > b`;
- **borderline**: `abs(g)` within 35 % of `b` of the edge;
- **within**: otherwise. Within on the transfer bands and on the parameter bands
  reads "camera consistent with mocap for arm calibration".

Nothing is selected on 10-27. A gap beyond the band is a valid "no", reported
as such.

Pre-registered not-usable rules, each reported as "not answerable with this
data" (the camera-only results are still reported):

1. C0 fails: `X'`, `Y'` of the three 10-27 runs differ from their mean by more
   than 2x the simulated recovery error of section 6.5 (any component), or the
   residual exceeds 3x the simulated floor on any axis. The frame part of this
   check is loose (about 16-26 mm and mrad at 2x the recovery error): it catches
   a gross inconsistency only, and the residual part carries most of the weight.
2. The fixture standard error exceeds 10 mrad, or half the band, for at least
   3 of `d_arm_2 .. d_arm_6` at the 10-27 postures.
3. A run with fewer than 6 paired postures is not scored (`min_postures`).

## 8. Design of part 2 (not run here)

- **B0/B1/B1'**: baselines on validation and confirmation with the same LOO
  re-registered RMSE. B0 nominal offsets, `X`, `Y` refitted; B1 PAL's offsets
  (`master_calibration_pal.yaml`) with refitted frames; B1' descriptive only,
  PAL as published (its camera pose in place of `X`, `Y` refitted). PAL's sign
  convention (`q + d` or `q - d`) is fixed on the training session only. PAL's
  arm_1, arm_5 and arm_7 sit exactly on +-0.02 (a bounded solver absorbing the
  gauge).
- **C0**: sensor agreement on each 10-27 run (section 6.5).
- **C1**: per run and per sensor, fit `G` on the paired rows; compare
  `d_arm_1 .. d_arm_6` against the parameter bands; report each sensor's run-to-run spread.
- **C2** (primary): evaluated run `r`, training run `r' != r`; G in both
  directions with the six ordered pairs; secondary G with the frozen 11-27
  training model.
- **Joints on 10-27**: both sensors use the PAL-CSV joints (absolute
  encoders). The joints of the bag (motor-side `/joint_states`) are a
  sensitivity check: the same C1/C2 with bag joints, reported next to the
  primary, not in it.
- **Diagnostics**: 11-07 camera postures, 11-27 `topleft`.
- Intrinsics: if `k` is in, it is fitted per camera session including
  re-registration; the same-day comparison is immune to the 11-27 versus 10-27
  difference.

Limits stated now: the sessions repeat one posture plan (11 of 12 of 10-27
within 20 mrad of 11-27), so held-out error is mainly repeatability; offsets
change between sessions (arm_5 slip, arm_1 on 11-07), so only the same-day
comparison decides; the PnP noise is depth-tilt correlated (the diagonal
whitening is approximate); with 5 seeds and 6 pairs the bands are about 35 %
uncertain.

## 9. Core gaps worked around here

1. **Stale eye-hand anchor.** `calc_updated_fkm` evaluates
   `get_rel_transform(start_frame, base_to_ref_frame)` once, before the per-sample
   loop, from whatever `data.oMf` holds. With a head camera (the chain from the
   torso to the camera moves) the anchor does not follow the sample. Checked
   with a script on this URDF: with zero parameters the marker pose differs from
   the correct forward kinematics by about 90 to 490 mm per sample, zero only at the
   sample evaluated last. The study therefore builds the two chains in
   pinocchio at `q + d` (`head_camera.Kinematics`); a test checks the arm chain
   against core's `joint_offset` kinematics to 1e-12 (all TIAGo arm axes are
   +z, so core's axis sign is +1). A core issue is drafted, not filed.
2. No QR/base-parameter selection for a custom residual: a numerical Jacobian
   and an SVD are used (the gauge of section 3).
3. `parse_heldout` refuses the `diagnostic` role: the role sets are built in
   `head_camera.py`.

## 10. Reproduction

```bash
conda activate figaroh-dev
cd figaroh-examples
MPLBACKEND=Agg python examples/tiago/head_camera.py noise --no-plot
MPLBACKEND=Agg python examples/tiago/head_camera_truth.py --no-plot \
    --golden tests/golden/tiago_head_camera_truth.json   # about 1 min
MPLBACKEND=Agg python -m pytest tests/test_tiago_head_camera.py \
    tests/test_tiago_head_camera_truth.py -q                 # fast
MPLBACKEND=Agg python -m pytest tests/test_tiago_head_camera_truth.py -m slow -q
```
