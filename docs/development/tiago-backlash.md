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

The primary part-2 model M* is the **all-groups model** (E, W and the load
group). The selected-groups model is reported as secondary and descriptive
only. This choice was made after seeing the fixture results and before any
real backlash fit. Training model fitted on the simulated 2023-07-24, offset
re-anchored on the simulated 2023-03-09, compared with the noise-free truth on
the samples inside the training knots (the last column is the fraction of
validation samples outside, not scored). Criterion: RMSE below 1.2 times the
noise level.

| criterion | pass rule | result |
|---|---|---|
| F4 transfer, primary (all groups) | RMSE / (level x sigma_d) < 1.2 (level 1) | pass on arm_1 to arm_4 and arm_7; arm_5_joint, arm_6_joint not answerable (N6) |
| F4 transfer, secondary (selected groups) | the same, every joint | **fail** (visible below, descriptive only) |

| joint | RMSE / noise | outside |
|---|---|---|
| `transfer arm_1_joint` | 0.27 (all groups, primary); 0.3 (selected, secondary) | 5 % |
| `transfer arm_2_joint` | 0.20 (all groups, primary); 2.2 (selected, secondary) | 10 % |
| `transfer arm_3_joint` | 0.16 (all groups, primary); 1.9 (selected, secondary) | 0 % |
| `transfer arm_4_joint` | 0.21 (all groups, primary); 1.3 (selected, secondary) | 9 % |
| `transfer arm_5_joint` | 1.79 (all groups, primary) **N6**; 1.8 (selected, secondary) | 5 % |
| `transfer arm_6_joint` | 2.58 (all groups, primary) **N6**; 2.9 (selected, secondary) | 14 % |
| `transfer arm_7_joint` | 0.36 (all groups, primary); 0.4 (selected, secondary) | 6 % |

F4 for the primary model: pass on arm_1 to arm_4 and arm_7 (0.16 to 0.36);
arm_5 (1.79) and arm_6 (2.58) are at or above 1.2 and become not answerable for
V1 by rule N6. The selected-groups model is within 1.2 on arm_1 (0.27) and
arm_7 (0.36) only; that failure is kept visible in the table. Two causes. On arm_2 to arm_4 the
failure is the **selection rule not admitting terms it cannot see** (a width of
2.5 mrad, a few mrad of `G`), which costs 1 to 2 mrad of RMSE. On arm_5 the
20 mrad wide step cannot be represented by knots 0.1 rad apart, and on arm_6 a
40 mrad width and `G` are not separable from `E` with 2 reversals. The V1 band
below is therefore a **floor of the method**, not of the noise. The bands are
computed for the primary (all-groups) model; the selected-model values are
given in brackets.

Bands for part 2 (level 1): `b_V1 = abs(mean g) + 2 sd g` with
`g = RMSE(M*) - RMSE(B1)` under a direction-only truth, and
`band_W = 2 RMS(W_train - W_val)` of the E+W fits.

| band | value |
|---|---|
| `band arm_1_joint` | b_V1 0.03 mrad (selected model: 0.03), band_W 0.11 mrad |
| `band arm_2_joint` | b_V1 0.10 mrad (selected model: 0.09), band_W 5.88 mrad |
| `band arm_3_joint` | b_V1 0.17 mrad (selected model: 0.03), band_W 1.85 mrad |
| `band arm_4_joint` | b_V1 0.02 mrad (selected model: 0.01), band_W 3.50 mrad |
| `band arm_5_joint` | b_V1 0.17 mrad (selected model: 0.17), band_W 0.30 mrad |
| `band arm_6_joint` | b_V1 1.59 mrad (selected model: 1.61), band_W 0.28 mrad |
| `band arm_7_joint` | b_V1 0.04 mrad (selected model: 0.04), band_W 0.03 mrad |

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

- **V1 transfer** (M* is the all-groups model; the selected-groups model is
  secondary and descriptive): `g = RMSE(M*) - RMSE(B1)` per joint; "improves" if
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
  there in the fixture. Decided: no V2 verdict on arm_3 or arm_7; their `W` is
  reported descriptively.
- N3: the fixture SE of `W` above `max(5 mrad, W/2)`.
- N4: `abs(corr(h, tanh(tau_g/tau0)))` above 0.95: load not separable (no joint).
- N6: the F4 transfer ratio of the primary model (all groups) is 1.2 or more at
  level 1 (arm_5 and arm_6): V1 not answerable, the method floor.
- N5: `tau_g` sign differs between the Schunk and Hey5 URDFs on more than 10 %
  of the samples: V3 not answerable (arm_7). V3 is also not answerable on arm_1
  (no gravity torque), and on arm_2 only `kappa` is usable.

## 8. Design of part 2 (not run here)

`backlash_study.py` with `--archive --no-plot`: fit M0 to M3 on training with
the frozen selection; print train and validation RMSE next to B0, B1, B2 per
joint; apply V1 to V3 and N1 to N6; the joint-inspection bags as **prediction
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

### Decisions taken (maintainer, 2026-10-22)

- (a) N2 coverage extension: kept, conservative. No V2 verdict on arm_3 or
  arm_7; their `W` is reported descriptively.
- (b) F2 stands as failed, limits unchanged; part 2 reads the bootstrap SE as
  an upper bound. For F4 the primary part-2 model M* is the all-groups model,
  which the fixture supports on arm_1 to arm_4 and arm_7; the selected-groups
  model is secondary and descriptive. V1 on arm_5 and arm_6 is pre-registered
  not answerable as the method floor (rule N6). This choice was made after
  seeing the fixture and before any real backlash fit.
- (c) Knot spacing of 0.1 rad on arm_5: kept (the 20 mrad step is not resolved;
  see N6).
- (d) D2: the label "validation-level" is confirmed.

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

---

# Part 2: results

Part 2 runs the rules of sections 7 to 9 on the real data. It changes no
threshold. Every number below is in `tests/golden/tiago_backlash.json`, written
by `examples/tiago/backlash_study.py`. A test checks that the tables quote the
golden. All verdicts carry the label **validation-level**: one training run, one
validation run, no held-out run.

## 11. Answer

- On arm_1 to arm_4 the model beats the direction constant (V1 improves). The
  gain comes from the structural term `E(q_abs)`. The width `W` there is below
  1 mrad, under the 1.53 mrad encoder step. The data do not show a backlash on
  these four joints.
- On arm_5 and arm_6 there is a real backlash of 13 to 14 mrad in training. V1
  is not answerable on them (rule N6, the method floor is above the effect).
  The model still lowers the error on arm_5 from 12.3 to 2.1 mrad.
- On arm_6 the width is not stable. It is 13.7 mrad on 2023-07-24 and 22.3 mrad
  on 2023-03-09. The manual probes give 38 to 42 mrad. The driven width is about a
  third of the probed width.
- Nothing shows a load dependence. No joint admitted the load group on
  training.
- The joint-inspection bags show that the arm_6 width changes with the arm_5
  angle (10 to 32 mrad). The trained model has a constant `W` and cannot predict
  this.

## 12. Training fit (2023-07-24)

Model M* is the all-groups model `E + W + L` (primary, decision (b)). The
standard errors are block-bootstrap errors. F2 failed, so read them as upper
bounds. `groups` is the selected model (secondary): E, W and L kept by the
blocked 5-fold CV rule (gain above 5 % and |z| above 3). RMSE is for M*.

| fit | W (mrad) | SE | G (mrad) | SE | kappa (mrad/N m) | SE | RMSE (mrad) | groups |
|---|---|---|---|---|---|---|---|---|
| `train arm_1_joint` | -0.16 | 0.11 | +0.00 | 0.00 | +0.00 | 0.00 | 0.59 | E |
| `train arm_2_joint` | -0.52 | 0.13 | +0.00 | 0.00 | +0.05 | 0.03 | 0.59 | E |
| `train arm_3_joint` | +0.43 | 0.32 | -0.02 | 9.88 | +0.15 | 0.15 | 0.73 | E |
| `train arm_4_joint` | -0.75 | 0.22 | +0.88 | 0.55 | -0.27 | 0.17 | 0.72 | E |
| `train arm_5_joint` | +13.28 | 0.67 | -27.68 | 11.84 | +48.87 | 21.20 | 1.54 | EW |
| `train arm_6_joint` | +14.23 | 4.41 | +15.76 | 75.00 | +0.76 | 22.13 | 4.26 | EW |
| `train arm_7_joint` | +6.18 | 2.33 | +0.00 | 0.00 | +0.00 | 0.00 | 3.25 | E |

Remarks.

- arm_1 to arm_4 and arm_7 select `E` only. The `W` group gains less than 5 %
  in CV on all five (arm_2 has |z| of 4.3 but gains 3 %; arm_7 has |z| of 2.7
  and gains 2 %). The `W` of M* is small there, and negative on arm_1, arm_2
  and arm_4. A negative width has no physical meaning. Read it as zero within
  the noise.
- arm_5 and arm_6 select `E + W`. The load group has z of 2.34 on arm_5 and
  0.21 on arm_6, both below 3.
- arm_6 and arm_7 have few reversals in training (2 and 4). The SE of `G` and
  `kappa` on arm_6 (75 and 22) show that the load terms are not determined.
- Session offsets differ by day on arm_5 (-9.5 mrad against +76.1 mrad) and on
  arm_6 (-29.9 against +3.6). The other joints agree within 0.4 mrad. The
  offset is re-anchored on each session, so this does not enter the RMSE.

## 13. Validation (2023-03-09, bottom run)

RMSE in mrad on all valid moving samples, offset re-anchored. B0 is the offset
only. B1 adds a direction constant. B2 is the degree-5 empirical surface of
`backlash_empirical_surface.py` (arm_2 falls back to degree 4 because degree 5
is rank deficient). `selected` is the secondary model.

| joint | B0 | B1 | B2 | M* | selected | outside knots |
|---|---|---|---|---|---|---|
| `val arm_1_joint` | 3.98 | 3.97 | 1.08 | 1.04 | 1.03 | 0.05 |
| `val arm_2_joint` | 3.18 | 3.29 | 2.16 | 0.66 | 0.73 | 0.06 |
| `val arm_3_joint` | 1.80 | 1.84 | 1.48 | 0.85 | 0.82 | 0.00 |
| `val arm_4_joint` | 9.01 | 8.98 | 4.53 | 1.30 | 1.20 | 0.08 |
| `val arm_5_joint` | 14.37 | 12.34 | 24.58 | 2.13 | 2.40 | 0.03 |
| `val arm_6_joint` | 15.48 | 11.21 | 184.34 | 9.97 | 11.64 | 0.14 |
| `val arm_7_joint` | 6.99 | 6.04 | 185.72 | 4.24 | 5.81 | 0.06 |

Where the gain comes from (descriptive, validation RMSE in mrad):

| joint | offset | + E | + E + W | + E + W + L |
|---|---|---|---|---|
| `dec arm_1_joint` | 3.98 | 1.03 | 1.04 | 1.04 |
| `dec arm_2_joint` | 3.18 | 0.73 | 0.70 | 0.66 |
| `dec arm_3_joint` | 1.80 | 0.82 | 0.84 | 0.85 |
| `dec arm_4_joint` | 9.01 | 1.20 | 1.18 | 1.30 |
| `dec arm_5_joint` | 14.37 | 7.32 | 2.40 | 2.13 |
| `dec arm_6_joint` | 15.48 | 15.97 | 11.64 | 9.97 |
| `dec arm_7_joint` | 6.99 | 5.81 | 4.24 | 4.24 |

## 14. Verdicts

`g` is RMSE(M*) minus RMSE(B1) in mrad. `b` is the V1 band from the frozen F4
table. V2 compares `W` of the E+W fits of the two sessions with the band on `W`.
V3 needs a load group admitted on training. Flags are the rules N1 to N6 of
section 7 that fired.

| joint | g | b | V1 | V2 | V3 | flags |
|---|---|---|---|---|---|---|
| `V arm_1_joint` | -2.92 | 0.03 | improves | W differs | not answerable | none |
| `V arm_2_joint` | -2.63 | 0.10 | improves | W stable across sessions | no load dependence detected | none |
| `V arm_3_joint` | -0.99 | 0.17 | improves | not answerable (W descriptive) | no load dependence detected | N2 |
| `V arm_4_joint` | -7.67 | 0.02 | improves | W stable across sessions | no load dependence detected | none |
| `V arm_5_joint` | -10.21 | 0.17 | not answerable | W differs | no load dependence detected | N6 |
| `V arm_6_joint` | -1.24 | 1.59 | not answerable (borderline) | W differs | no load dependence detected | N6 |
| `V arm_7_joint` | -1.79 | 0.04 | improves | not answerable (W descriptive) | not answerable | N2, N5 |

Per joint.

- arm_1: V1 improves. V2 says `W` differs, with a difference of 0.17 mrad. That
  is far below the encoder step, so it is a statistical flag, not a physical
  one. V3 is not answerable: the joint has no gravity torque.
- arm_2, arm_4: V1 improves, `W` stable, no load dependence detected.
- arm_3: V1 improves. N2 fired (direction overlap 0.48 in training), so there
  is no V2 verdict. `W` is 0.46 mrad in training and 0.19 mrad in validation.
- arm_5: V1 is not answerable (N6). For information, g is -10.21 mrad against a
  band of 0.17 mrad. V2 says `W` differs: 13.18 against 12.70 mrad, a change of
  0.48 mrad, band 0.30 mrad. Both sessions agree to 4 %, so this is a tight band
  and not a large change.
- arm_6: V1 is not answerable (N6). For information, g is -1.24 mrad against a
  band of 1.59 mrad. It would read "no change", and it is borderline. V2 says
  `W` differs: 13.74 against 22.30 mrad.
- arm_7: V1 improves. N2 (overlap 0.32) removes V2: `W` is 6.18 mrad in training
  and 8.15 mrad in validation, descriptive only. N5 fired: the Schunk and Hey5
  torque signs disagree on all samples, so V3 is not answerable.

Secondary model. The selected model gives about the same g on arm_1 to arm_5. On
arm_6 it gives +0.43 mrad and on arm_7 -0.23 mrad. It does not change a verdict
class.

## 15. Joint inspection (prediction checks, descriptive)

No score and no verdict. The 2024-02-02 bags move arm_6 with arm_5 held at
different angles. Nothing is fitted for a verdict. The gravity uses the Hey5 URDF
(hand not verified), with Schunk as a sensitivity check. The inspection files
record only arm_5 and arm_6, so the other joints are set to the training-run
median. This is an assumption. Hey5 and Schunk predictions differ by at most
1.7 mrad.

Probes (absolute encoder peak to peak, manual) against the driven width of the
trained model on arm_6. `W / probe` uses the E+W width of training (13.74 mrad).

| probe | abs p2p (mrad) | rel p2p (mrad) | W / probe | W - probe (mrad) |
|---|---|---|---|---|
| `probe arm_6_backlash_abs_inspect_2024-01-26-18-07-23` | 38.3 | 0.44 | 0.36 | -24.6 |
| `probe arm_6_backlash_abs_inspect_2024-01-26-18-07-54` | 42.2 | 0.41 | 0.33 | -28.4 |

Width of arm_6 per bag (E+W fit on the bag, offset re-anchored) against the
median arm_5 absolute angle. Sorted by angle. `pred` is the RMSE of the trained
M* on the bag (Hey5) and `own` the RMSE of the bag's own E+W fit.

| bag | arm_5 angle (rad) | W (mrad) | SE | pred (mrad) | own (mrad) |
|---|---|---|---|---|---|
| `bag arm5_minus_1_57_2024-02-02-17-40-08` | -1.68 | 9.6 | 1.7 | 14.8 | 3.4 |
| `bag arm5_minus_1_2_2024-02-02-17-37-42` | -1.32 | 10.1 | 1.9 | 14.4 | 3.7 |
| `bag arm5_minus_0_5_2024-02-02-17-30-30` | -0.63 | 15.1 | 4.3 | 11.8 | 7.6 |
| `bag arm5_minus_0_5_2024-02-02-17-32-19` | -0.63 | 12.2 | 2.3 | 10.2 | 4.2 |
| `bag arm5_minus_0_8_2024-02-02-17-34-13` | -0.63 | 12.3 | 2.6 | 10.5 | 4.4 |
| `bag arm5_minus_0_8_2024-02-02-17-35-02` | -0.63 | 12.3 | 2.3 | 10.2 | 4.2 |
| `bag arm5_minus_0_1_2024-02-02-17-50-54` | -0.24 | 27.3 | 1.9 | 11.7 | 3.4 |
| `bag arm5_0_2024-02-02-17-25-39` | -0.03 | 29.7 | 1.1 | 12.9 | 2.3 |
| `bag arm5_0_2_2024-02-02-17-46-17` | +0.06 | 32.1 | 1.2 | 13.7 | 2.5 |
| `bag arm5_0_4_2024-02-02-17-48-10` | +0.25 | 31.5 | 0.7 | 13.9 | 2.0 |
| `bag arm5_0_5_2024-02-02-17-28-22` | +0.35 | 29.1 | 1.1 | 14.1 | 2.4 |
| `bag arm5_0_8_2024-02-02-17-43-00` | +0.64 | 9.6 | 3.2 | 19.6 | 9.5 |
| `bag arm5_1_2_2024-02-02-17-44-25` | +1.20 | 9.8 | 1.9 | 17.4 | 3.9 |

Summary over 13 bags: mean 18.5, sd 9.6,
min 9.6, max 32.1 mrad. The Pearson correlation
with the arm_5 angle is +0.32 (slope +3.9
mrad/rad). A post-hoc correlation with |angle| is -0.80;
it was chosen after looking at the table and is not evidence. One bag
(`arm5minus1_59`) is excluded because arm_6 does not move. The bags at
+0.64 and +1.20 rad (and the bag at +0.64 with arm_5 moving 2.6 rad) are weak:
the own fit of the first has 9.5 mrad RMSE.

## 16. Ambiguities in the frozen rules and how they were resolved

- V1 is scored on all valid validation samples, the sample used to compute the
  F4 band. Samples outside the training knots are reported, not removed. With
  samples inside the knots only, arm_6 would read "improves, borderline" (g -1.61
  mrad against 1.59). It is N6 anyway, so no verdict changes.
- B2 uses the existing rank back-off of `EmpiricalBacklashSurface`.
- V2 uses E+W fits on both sessions. The validation refit builds its own knots.
- N3 uses the seed-0 `W` SE of the frozen fixture. It never fired.
- V3: the load group counts as admitted if the selection rule keeps it on
  training. Then a validation coefficient needs the same sign and |z| above 2.
  This branch was never reached.
- N1, N4 and N5 are checked on both runs.
- Bags in which arm_6 does not move are excluded, and listed in the golden.
- The arm_5 angle of a bag is the median of its absolute encoder.

## 17. Unexpected

- B2 blows up on validation for arm_5, arm_6 and arm_7 (24.6, 184.3 and
  185.7 mrad) although its training RMSE is about 1 mrad. B2 is a degree-5
  polynomial in `q_rel` and the torque. It does not use `q_abs`, and it
  extrapolates badly between sessions. M* does not suffer from this.
- The V1 gain on arm_1 to arm_4 is the `E` term. `W` is below 1 mrad there.
  "The model improves" must not be read as "backlash found".
- V2 flags a 0.17 mrad change on arm_1 as "W differs". The band is tighter than
  the encoder step, so the flag has no physical meaning.
- The driven width of arm_6 (14 to 22 mrad) is about a third of the manual
  probes (38 to 42 mrad). The two methods do not measure the same thing, or
  the load during the probe differs.
- The arm_6 width depends on the arm_5 angle. It is 27 to 32 mrad for arm_5
  between -0.24 and +0.35 rad, and 10 to 12 mrad elsewhere. This pattern appears
  only in the bags, and was not in the pre-registration. The trained model has
  a constant `W` and no load group was admitted, so it cannot predict it.
- The session offset of arm_5 and arm_6 moves by 86 and 33 mrad between days.
  That is why the offset is re-anchored on each session.
- F2 failed. All standard errors are upper bounds.
- About 23 % of the bag samples are outside the training knots, so the bag
  predictions extrapolate `E`.

## 18. Reproduction of part 2

```bash
conda activate figaroh-dev
cd figaroh-examples
MPLBACKEND=Agg python examples/tiago/backlash_study.py --no-plot \
    --golden tests/golden/tiago_backlash.json                # about 10 s
MPLBACKEND=Agg python examples/tiago/backlash_study.py --archive --no-plot
MPLBACKEND=Agg python -m pytest tests/test_tiago_backlash_study.py -q
```
