# TIAGo backlash surface, arm transmission error (#71): pre-registration

Issue: [#71](https://github.com/thanhndv212/figaroh-examples/issues/71).
This note is part 1: the pre-registration, to be merged before any real fit of
a backlash parameter. It is frozen once merged. The study (real-data fits,
comparison with the baselines, verdicts) is part 2 and changes no threshold
written here.

Question: on the TIAGo arm, the encoder on the motor side (divided by the gear
ratio, `q_rel`) and the absolute encoder on the output side (`q_abs`) differ by
`d = q_rel - q_abs`. How much of `d` is a backlash that depends on the motion
direction, how much is an angle error of the absolute encoder, and does the
backlash depend on the gravity load? Does the model fitted on the 2023-07-24
run predict the 2023-03-09 run better than a constant per direction?

## 1. What this part contains, and what it does not

- The unified configuration `examples/tiago/config/tiago_backlash_study_config.yaml`
  (model constants, selection rule, noise, verdict rules, fixture and its pass
  criteria) and the label fix of `tiago_backlash_surface_config.yaml`
  (`target: relative_encoder_minus_absolute_encoder`; the code was right, the
  label was wrong).
- The library `examples/tiago/backlash_model.py` (sessions and roles, direction
  state, jump segmentation, spline, design, block bootstrap, selection,
  baselines, the validation guard, noise and identifiability stages) and the
  fixture `examples/tiago/backlash_truth.py`.
- The thresholds below. They come from the fixture, which fits only
  **simulated** measurements. The corrected `protocol.yaml` wording about the
  hand (section 2) and the refreshed data inventory.

The validation run is **not blind**: the issue table and the README already
report held-out errors and the +111 mrad arm_5 gap of 2023-03-09. There is no
confirmation set and none is created. The verdict of part 2 is labelled
**validation-level** (D2).

Disclosure of every real read, in order:

- while planning: the channel-name files of both runs, the first 300 bytes of
  the top run's `tf.csv`, the header row of `arm_6_step_2024-01-25-18-14-40.csv`,
  and one values row (the first 400 bytes of the 2023-03-09 bottom
  `introspection_datavalues.csv`: arm_1 to arm_4 sample 1). Nothing was derived
  from the values row;
- the **regressor columns** of both runs (torso and arm positions and
  velocities, never an absolute encoder of the validation run): direction state,
  gravity torque, identifiability (section 3), the fixture's configuration, and
  the Schunk against Hey5 gravity sign check (rule N5);
- the **training run's absolute encoders** (seven arm joints) in the noise stage
  (section 4): `sigma_d`, `rho`, the quantization step and the jump count (0).
  This is a target read, but nothing is fitted to it. The validation run's
  absolute encoders are never read: `backlash_model.read_target` raises
  `PermissionError` unless `unlock_validation` is set, and only the part 2
  script sets it (tested).

Not read: the 2023-03-09 top run (a pose-only `tf.csv`, unused, D6); the
joint-inspection bags (described in section 8, read in part 2 only).

## 2. Roles

`examples/tiago/data/backlash/protocol.yaml`, roles unchanged: training
`2023-07-24-13-28-42` (untraced, a choice), validation
`2023-03-09-16-19-17-bottom` (the primary verdict set), diagnostic
`2023-03-09-15-28-20-top` (not used, D6: its `tf` is built from `q_rel`, so it
carries no transmission error). Role sets are built in `backlash_model.role_sets`
(`parse_heldout` has no `diagnostic` role).

**Hand (D4).** Both backlash days are tiago-48 with the SCHUNK hand: the
hand channels are a single `gripper_finger_joint` plus fingertip forces, not
Hey5 channels. The gravity torque of the backlash days uses
`tiago_48_schunk.urdf`. The 2024 joint-inspection bags use `tiago_48_hey5.urdf`
with Schunk as the sensitivity check; the hand of those bags is **not
verified**. The earlier `protocol.yaml` wording that suggested the PAL gripper
for the backlash days is corrected in this PR.

## 3. Model

Per arm joint (torso excluded: it only steps, rule N2), on the moving samples:

`d = c_s + E(q_abs) + b + G tanh(tau_g / tau0) + kappa tau_g + e`

- `c_s`: one offset per session and per jump segment, re-anchored on each
  session (D1).
- `E(q_abs)`: the absolute encoder's angle error, a piecewise-linear function
  of **`q_abs`** with knots every 0.1 rad over the visited range, shared by
  both directions, frozen from the training run (D1). It takes `q_abs` so that
  the 0.7 to 1.1 rad step of arm_5 sits at one abscissa in both directions.
- `b = (W/2) h`: the direction part. `h` is the sign of the 50 ms smoothed
  `q_rel` velocity, held through `|v| < 0.02 rad/s`. The **backlash surface**
  is `W(q, tau_g)`, the direction-dependent part after removing `c_s` and `E`.
- `G`, `kappa`: how the load moves the gear onto one flank (preload) and winds
  it up (compliance). `tau_g` is the pinocchio gravity torque at the logged
  torso and arm configuration. `tau0 = 0.5 N m`.

Velocity enters only through `h`. Fit: linear least squares per joint, columns
of unit norm; block-bootstrap SE (200 resamples, 5 s blocks, seed 7). Selection
(training only): `E` is structural (always in the model); `W`, then the load
group, enter if 5-fold blocked CV (10 s periods) lowers RMSE by more than 5 %
and a coefficient has `|z| > 3`. Baselines: B0 offset, B1 offset plus a
direction constant, B2 the existing degree-5 `EmpiricalBacklashSurface` (kept,
D5), all re-anchored.

Four changes to the plan came from the fixture, before any real fit. They are
disclosed because each moved a result:

1. **`E` is structural, with a second-difference penalty.** The training run
   lasts 121 s in 13 blocks of 10 s and each sweep visits its own angles, so a
   blocked fold cannot validate `E`: CV rejected `E` on arm_1 although the
   truth has a 2 % scale error. `E` is now always in, and a second-difference
   penalty of the weight of one sample (`surface.smooth`) interpolates the
   knots no sample constrains.
2. **Only moving samples are used** (`|v| >= v_min`), and the mask after a
   reversal starts at the end of the old motion and lasts 0.6 s (was 0.3 s,
   from the reversal). A held `h` on a dwell can sit on the wrong flank after a
   slow turn; that gave a W bias of 0.2 to 0.6 mrad on arm_1 to arm_4 (removed
   by using moving samples only). The 0.3 s mask then left 0.47 mrad on arm_6
   (40 mrad of play to cross); at 0.6 s it is 0.06.
3. **The fixture's backlash is a play operator driven by `q_rel`**, not a
   sign flip: a flip is a jump of `W` in `d` with `q_rel` still, which is what
   the jump detector looks for (W of arm_6 is 40 mrad, above the 20 mrad
   threshold).
4. **`q_abs` is synthesized by bisection**, not by the two fixed-point steps of
   the plan, which do not contract on the arm_5 step (slope above 1).

## 4. Noise stage

`python examples/tiago/backlash_model.py noise` reads the training `d`:
`sigma_d` from second differences (`var(diff2 d) / 6`) on moving samples, `rho`
from the residual of a 1 s moving median, the quantization step as the smallest
non-zero step of `q_abs`. Grid levels are 0.5, 1 and 2 times `sigma_d`; the used
level is 1. No encoder jump was found in the training run.

| quantity | value |
|---|---|
| `noise arm_1_joint` | sigma_d 0.53 mrad, rho 0.10, q_abs step 1.53 mrad |
| `noise arm_2_joint` | sigma_d 0.51 mrad, rho 0.15, q_abs step 1.53 mrad |
| `noise arm_3_joint` | sigma_d 0.52 mrad, rho 0.08, q_abs step 1.53 mrad |
| `noise arm_4_joint` | sigma_d 0.53 mrad, rho 0.09, q_abs step 1.53 mrad |
| `noise arm_5_joint` | sigma_d 0.94 mrad, rho 0.06, q_abs step 1.53 mrad |
| `noise arm_6_joint` | sigma_d 0.25 mrad, rho 0.53, q_abs step 0.77 mrad |
| `noise arm_7_joint` | sigma_d 0.26 mrad, rho 0.56, q_abs step 0.77 mrad |

## 5. Identifiability (real regressor columns, F1)

`python examples/tiago/backlash_model.py identifiability`. `q_rel` stands in
for `q_abs` as the argument of `E` (no absolute encoder is read), so this is
the conditioning of the design at the visited angles. Values are training /
validation unless one number.

| joint | scaled cond | corr(h, tanh tau) | direction overlap | reversals | tau_g sd (N m) | N5 sign fraction |
|---|---|---|---|---|---|---|
| arm_1_joint | 210 / 240 | +0.25 / +0.12 | 0.88 / 0.89 | 15 / 65 | 0.00 | 0.00 |
| arm_2_joint | 20 / 35 | +0.00 / +0.13 | 0.75 / 0.93 | 12 / 62 | 5.66 | 0.00 |
| arm_3_joint | 197 / 402 | +0.04 / -0.10 | 0.45 / 0.68 | 14 / 66 | 2.83 | 0.00 |
| arm_4_joint | 127 / 120 | +0.22 / +0.01 | 0.85 / 0.92 | 15 / 65 | 3.77 | 0.00 |
| arm_5_joint | 55 / 117 | -0.02 / +0.04 | 0.71 / 0.81 | 14 / 66 | 0.19 | 0.03 |
| arm_6_joint | 199 / 216 | -0.07 / +0.07 | 0.88 / 0.91 | 2 / 64 | 0.28 | 0.00 |
| arm_7_joint | 38 / 107 | +0.06 / +0.09 | 0.32 / 0.61 | 4 / 66 | 0.02 | 1.00 |

All designs are full rank with a condition number below 1e3 (F1 passes). The
load and the direction are separable: `|corr(h, tanh)|` is at most 0.25
(rule N4 limit 0.95). What is not separable is a property of the joint:

- arm_1 has a vertical axis: no gravity torque (sd 0), so neither load column
  carries information;
- arm_7 has `tau_g` sd 0.02 to 0.03 N m and the Schunk and Hey5 URDFs disagree
  on its sign on every sample (N5 = 1.00): the load is not answerable;
- arm_2: the soft sign is saturated, so `G` is the offset and only `kappa` is
  usable;
- the load columns are largely explained by the `E` hats (R^2 0.4 to 0.8): the
  load needs angles visited at several gravity torques.

Direction overlap (fraction of 0.1 rad bins seen in both directions for 0.5 s
each) in training is 0.45 on arm_3 and 0.32 on arm_7. The training reversals of
arm_6 (2) and arm_7 (4) are few, although arm_6's overlap is 0.88.

## 6. Pre-registered thresholds (simulated)

Fixture `backlash_truth.py`, seeds 0 to 4. The truth per seed has widths around
2.5 (arm_1 to arm_4), 12 (arm_5), 40 (arm_6) and 6 (arm_7) mrad (plus or minus
25 %), `G ~ N(0, 3 mrad)`, `kappa ~ N(0, 2 mrad per N m)`, an `E` made of 2 %
scale, a smooth quadratic and cubic term and, on arm_5, a 170 mrad logistic
step centred in 0.7 to 1.1 rad, and a session offset `N(0, 50 mrad)`. The
configuration (`q_rel`, velocity, gravity torque) is the **real** one of both
runs; `q_abs` is synthesized. Noise streams: truth `default_rng(seed)`, noise
`default_rng(10000 + 100 k + seed)`, `k` = 0 AR(1), 1 quantization, 2 jumps.
The criteria were written in the configuration before the first run.

### 6.1 F1 to F3

| criterion | pass rule | result |
|---|---|---|
| F1 identifiability | M3 full rank, scaled cond < 1e3, every joint and run | pass |
| F2 recovery | pooled z-RMS of W, G, kappa in [0.75, 1.33] and abs(mean z) < 0.3, at every level | **fail** |
| F2 quantization | the same with `q_abs` rounded to the encoder step (level 1) | **fail** |
| F3 separation | arm_5 with the E step: W = 0 gives abs(W_hat) < 2 SE and < 2 mrad; W = 12 mrad gives abs(error) < 3 SE and < 25 % in 5 of 5 seeds | pass |

| | 0.5 x sigma_d | 1.0 x sigma_d | 2.0 x sigma_d |
|---|---|---|---|
| `recovery z rms` | 0.79 | 0.76 | 0.72 |
| `recovery z mean` | -0.34 | -0.28 | -0.20 |
| `recovery z rms W` | 0.98 | 0.94 | 0.89 |
| `recovery z mean W` | -0.75 | -0.62 | -0.44 |
| `recovery z rms G` | 0.49 | 0.45 | 0.40 |
| `recovery z rms kappa` | 0.67 | 0.67 | 0.64 |

F2 fails on two counts and neither is a method bug. The z-RMS is below the
lower limit at level 2 (0.72) and for the quantization run (0.66): the
bootstrap SE is conservative, mostly for `G` (rms 0.45). The
mean z is -0.34 at level 0.5 (the limit is 0.3; -0.28 at level 1): `W` is
under-estimated by 0.00 to 0.06 mrad on every joint (mean z of `W`
-0.62 at level 1), a residual the 0.6 s mask does not remove. They are reported as they are; the limits are not
moved. Part 2 reads SE as an upper bound.

F3 (W estimates in mrad, SE in brackets; level 1; E as a function of `q_abs`
for the first two columns, of `q_rel` for the third):

| seed | W = 0 | W = 12 | W = 12, E on `q_rel` |
|---|---|---|---|
| 0 | +0.01 (0.027) | 11.99 (0.028) | 11.77 (0.030) |
| 1 | +0.00 (0.026) | 11.98 (0.028) | 12.25 (0.028) |
| 2 | +0.03 (0.026) | 12.01 (0.026) | 11.76 (0.032) |
| 3 | -0.03 (0.026) | 11.95 (0.031) | 12.31 (0.042) |
| 4 | -0.05 (0.032) | 11.94 (0.034) | 12.10 (0.069) |

`E` on `q_abs` recovers the width in 5 of 5 seeds and finds none when there is
none. With `E` on `q_rel` the fit does not collapse but is biased by up to 0.3
mrad (2.5 %, 3 to 8 SE in four seeds of five): the argument matters for the local width at the
step, little for the global one. The reason for `q_abs` stays the first one,
that a step at one abscissa in both directions is what the data shows.

### 6.2 F4: transfer and the V1 band

Training model (selected groups) fitted on the simulated 2023-07-24, offset
re-anchored on the simulated 2023-03-09, compared with the noise-free truth on
the samples inside the training knots (the last column is the fraction of
validation samples outside, not scored). Criterion: RMSE below 1.2 times the
noise level.

| criterion | pass rule | result |
|---|---|---|
| F4 transfer | RMSE / (level x sigma_d) < 1.2 on every joint (level 1) | **fail** |

| joint | RMSE / noise | outside |
|---|---|---|
| `transfer arm_1_joint` | 0.3 (selected), 0.27 (all groups) | 5 % |
| `transfer arm_2_joint` | 2.2 (selected), 0.20 (all groups) | 10 % |
| `transfer arm_3_joint` | 1.9 (selected), 0.16 (all groups) | 0 % |
| `transfer arm_4_joint` | 1.3 (selected), 0.21 (all groups) | 9 % |
| `transfer arm_5_joint` | 1.8 (selected), 1.79 (all groups) | 5 % |
| `transfer arm_6_joint` | 2.9 (selected), 2.58 (all groups) | 14 % |
| `transfer arm_7_joint` | 0.4 (selected), 0.36 (all groups) | 6 % |

F4 fails: the selected model is within 1.2 on arm_1 (0.27) and arm_7 (0.36)
only. The all-groups model passes on arm_1 to arm_4 and arm_7 (0.16 to 0.36)
and fails on arm_5 (1.79) and arm_6 (2.58). Two causes. On arm_2 to arm_4 the
failure is the **selection rule not admitting terms it cannot see** (a width of
2.5 mrad, a few mrad of `G`), which costs 1 to 2 mrad of RMSE. On arm_5 the
20 mrad wide step cannot be represented by knots 0.1 rad apart, and on arm_6 a
40 mrad width and `G` are not separable from `E` with 2 reversals. The V1 band
below is therefore a **floor of the method**, not of the noise.

Bands for part 2 (level 1): `b_V1 = abs(mean g) + 2 sd g` with
`g = RMSE(M*) - RMSE(B1)` under a direction-only truth, and
`band_W = 2 RMS(W_train - W_val)` of the E+W fits.

| band | value |
|---|---|
| `band arm_1_joint` | b_V1 0.03 mrad, band_W 0.11 mrad |
| `band arm_2_joint` | b_V1 0.09 mrad, band_W 5.88 mrad |
| `band arm_3_joint` | b_V1 0.03 mrad, band_W 1.85 mrad |
| `band arm_4_joint` | b_V1 0.01 mrad, band_W 3.50 mrad |
| `band arm_5_joint` | b_V1 0.17 mrad, band_W 0.30 mrad |
| `band arm_6_joint` | b_V1 1.61 mrad, band_W 0.28 mrad |
| `band arm_7_joint` | b_V1 0.04 mrad, band_W 0.03 mrad |

### 6.3 F5 and F6: selection and jumps

| criterion | pass rule | result |
|---|---|---|
| F5 selection | load group admitted in <= 1 of 5 seeds per joint with `G = kappa = 0`, in >= 4 of 5 with `G` = 10 mrad on arm_6 | pass |
| F6 jumps | all injected jumps (1 to 3 of 30 to 100 mrad per joint) found, 0 false at the top level | pass |

| | 0.5 x sigma_d | 1.0 x sigma_d | 2.0 x sigma_d |
|---|---|---|---|
| `selection true admit` | 5 of 5 | 5 of 5 | 4 of 5 |
| `selection false admit` | 0 of 35 | 0 of 35 | 0 of 35 |
| `jumps found` | 68 of 68 | 68 of 68 | 68 of 68 |
| `jumps false` | 0 | 0 | 0 |

## 7. Verdict rules (written before the real fit)

Baselines on validation, re-anchored the same way: B0, B1 (the comparator of
#71), B2.

- **V1 transfer**: `g = RMSE(M*) - RMSE(B1)` per joint; "improves" if
  `g < -b_V1`, "worse" if `g > b_V1`, "no change" within; borderline within 35 %
  of `b_V1` of an edge. `b_V1` is the table above. The fixture says the
  method reaches 1 to 2 mrad of residual error on the joints where it cannot
  select (section 6.2): a gain below that is not expected.
- **V2 backlash width**: `abs(W_train - W_val)` within `band_W` gives "W stable
  across sessions". The arm_6 width against the manual probes (38.3 and
  42.2 mrad) is reported, not scored.
- **V3 load dependence**: `G` or `kappa` admitted on training and the
  validation refit has the same sign with `abs(z) > 2` gives "load-dependent",
  otherwise "no load dependence detected".
- A beyond-band result is reported as such. Nothing is tuned on validation.

Not answerable (the rule is applied per joint and verdict):

- N1: more than 3 jumps in a run on a joint, or a segment shorter than 60 s;
  the training run has none (F6 is fixture-only).
- N2: the torso (excluded): fewer than 10 reversals. **Extended for the arm**:
  a joint whose direction overlap in training is below 0.5 has no `W` verdict
  (arm_3 at 0.45 and arm_7 at 0.32); this is conservative, F2 recovers `W`
  there in the fixture. The maintainer may relax it.
- N3: the fixture SE of `W` above `max(5 mrad, W/2)`.
- N4: `abs(corr(h, tanh(tau_g/tau0)))` above 0.95: load not separable (no joint).
- N5: `tau_g` sign differs between the Schunk and Hey5 URDFs on more than 10 %
  of the samples: V3 not answerable (arm_7). V3 is also not answerable on arm_1
  (no gravity torque), and on arm_2 only `kappa` is usable.

## 8. Design of part 2 (not run here)

`backlash_study.py` with `--archive --no-plot`: fit M0 to M3 on training with
the frozen selection; print train and validation RMSE next to B0, B1, B2 per
joint; apply V1 to V3 and N1 to N5; the joint-inspection bags as **prediction
checks only** (D3), the arm_6 width against the 38 to 42 mrad probes and
against the arm_5 angle, with `c_s` re-anchored per bag and no score, using the
Hey5 URDF (hand not verified) with Schunk as the sensitivity check. The
validation absolute encoder is unlocked in this script only. The
`backlash_empirical_surface.py` held-out error and the README pointer close
item 2 of #71.

## 9. Core gaps worked around here

- `parse_heldout` has no `diagnostic` role: role sets are built locally, as in
  the head camera work.
- No hysteresis, backlash or block-bootstrap utility in core: local code, no
  core issue proposed.

Decisions for the maintainer: (a) the N2 coverage extension (arm_3, arm_7);
(b) whether the F2 and F4 failures stand as pre-registered (limits unchanged)
or the F4 criterion is replaced by the all-groups floor; (c) the knot spacing
of 0.1 rad on arm_5 (the 20 mrad step cannot be resolved); (d) the D2 label.

## 10. Reproduction

```bash
conda activate figaroh-dev
cd figaroh-examples
MPLBACKEND=Agg python examples/tiago/backlash_model.py noise
MPLBACKEND=Agg python examples/tiago/backlash_model.py identifiability
MPLBACKEND=Agg python examples/tiago/backlash_truth.py --no-plot \
    --golden tests/golden/tiago_backlash_truth.json          # about 1 min
MPLBACKEND=Agg python -m pytest tests/test_tiago_backlash_model.py \
    tests/test_tiago_backlash_truth.py -q                    # fast
MPLBACKEND=Agg python -m pytest tests/test_tiago_backlash_truth.py -m slow -q
```
