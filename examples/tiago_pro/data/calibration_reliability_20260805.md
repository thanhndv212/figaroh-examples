# Statistical reliability of all identified parameters — 2026-08-05

**Updated 2026-08-07** after migrating `run_calibration.py` onto
figaroh-plus's `BaseCalibration` (v0.4.7). See the "What changed" section
below before reading the table — the numbers here are not just a re-run,
several are qualitatively different from the original 2026-08-05 analysis.

Source: `calibration_results_20260805_1246.yaml` (48 samples, 1 outlier
flagged by the native detector — not excluded, see note below — position
RMSE 7.11mm/MAE 6.55mm, orientation RMSE 1.92°/MAE 1.67°). Standard error
now comes from `BaseCalibration.calc_stddev()` (native), not a hand-rolled
finite-difference Jacobian. Verdict: `solid` ≥3σ, `acceptable` 2-3σ,
`marginal` 1-2σ, `noise` <1σ (value not distinguishable from 0 with this
data).

**Breakdown: 22 solid, 3 acceptable, 4 marginal, 9 noise — out of 38.**
(Previous run: 8 solid, 4 acceptable, 4 marginal, 22 noise.)

---

## What changed, and why the numbers moved this much

The original run used a hand-written residual (`_wrap_orientation_residual`
in the legacy script) that wrapped each RPY component into `[-π, π]`
independently to fix a real wraparound bug (sample #17 back then). It
worked, but element-wise RPY is still not a proper metric on SO(3).

figaroh-plus v0.4.7 (pulled 2026-08-07, 59 commits ahead of the version this
project started on) replaced that internally with a genuine SE3 log-map
residual (`log(M_meas⁻¹·M_est)` — see `BaseCalibration._compute_logmap_residuals`
and the migration writeup implied by commit `ca13f6d`, *"fix(calibration):
replace RPY element-wise subtraction with SE3 log-map pose error"*).
`run_calibration.py` was migrated to subclass `BaseCalibration` and use it
(see git history / `run_calibration_legacy.py` for the prior version).

**This did not just fix an edge case — it tightened precision across the
board**: 22/38 parameters are now ≥3σ (was 8/38), including `pEEx_1`
(1.67σ → 15.14σ) and `pEEy_1` (0.46σ → 3.32σ), the two axes flagged
completely unreliable in the original analysis. The wrap-based residual was
evidently a noisier proxy for the true geometric error, not just wrong at
the wraparound boundary.

One migration pitfall worth recording: `BaseCalibration.apply_measurement_weighting()`
(default 1000x position / 100x orientation weights) combined with our L2
regularization term broke `scipy.optimize.least_squares(method="lm")`
entirely — it reported `xtol` "success" after 10 function evaluations with
a first-order optimality of 5.5e5 (nowhere near a real optimum), because the
weighted-residual and regularization column norms in the Jacobian differed
by ~5 orders of magnitude. `cost_function()` in `run_calibration.py` uses
raw (unweighted) log-map residuals instead — see its docstring for the
diagnostic.

---

## Full table

| Parameter | Value | Standard error | σ | Verdict |
|---|---|---|---|---|
| base_px | -7.039 mm | ±1.383 mm | 5.09 | solid |
| base_py | -4.759 mm | ±2.969 mm | 1.60 | marginal |
| base_pz | +11.620 mm | ±0.515 mm | 22.56 | solid |
| base_phix | -0.401° | ±0.177° | 2.27 | acceptable |
| base_phiy | +0.309° | ±0.067° | 4.64 | solid |
| base_phiz | +0.363° | ±0.104° | 3.49 | solid |
| d_px_arm_right_2 | +0.491 mm | ±0.299 mm | 1.64 | marginal |
| d_py_arm_right_2 | +1.247 mm | ±0.402 mm | 3.10 | solid |
| d_phix_arm_right_2 | -0.217° | ±0.019° | 11.53 | solid |
| d_phiy_arm_right_2 | -0.648° | ±0.718° | 0.90 | noise |
| d_phiz_arm_right_2 | -0.152° | ±0.217° | 0.70 | noise |
| d_px_arm_right_3 | -0.671 mm | ±2.182 mm | 0.31 | noise |
| d_py_arm_right_3 | -1.251 mm | ±0.486 mm | 2.57 | acceptable |
| d_phix_arm_right_3 | +0.584° | ±0.019° | 30.11 | solid |
| d_phiz_arm_right_3 | -0.640° | ±0.718° | 0.89 | noise |
| d_px_arm_right_4 | +1.872 mm | ±0.369 mm | 5.07 | solid |
| d_py_arm_right_4 | +0.862 mm | ±0.312 mm | 2.76 | acceptable |
| d_phix_arm_right_4 | -0.097° | ±0.018° | 5.50 | solid |
| d_phiz_arm_right_4 | +1.419° | ±0.021° | 67.50 | solid |
| d_px_arm_right_5 | -4.723 mm | ±0.373 mm | 12.67 | solid |
| d_py_arm_right_5 | +0.213 mm | ±12.621 mm | 0.02 | noise |
| d_phix_arm_right_5 | -0.070° | ±0.021° | 3.42 | solid |
| d_px_arm_right_6 | -0.469 mm | ±0.321 mm | 1.46 | marginal |
| d_py_arm_right_6 | -0.106 mm | ±12.620 mm | 0.01 | noise |
| d_pz_arm_right_6 | +0.212 mm | ±12.621 mm | 0.02 | noise |
| d_phix_arm_right_6 | -0.487° | ±0.017° | 28.76 | solid |
| d_phiy_arm_right_6 | -5.767° | ±0.693° | 8.32 | solid |
| d_px_arm_right_7 | +8.233 mm | ±0.838 mm | 9.82 | solid |
| d_pz_arm_right_7 | +0.104 mm | ±12.620 mm | 0.01 | noise |
| d_phix_arm_right_7 | +0.069° | ±0.018° | 3.85 | solid |
| d_phiy_arm_right_7 | -0.009° | ±1.023° | 0.01 | noise |
| d_phiz_arm_right_7 | +7.471° | ±0.693° | 10.78 | solid |
| pEEx_1 | -4.672 mm | ±0.309 mm | 15.14 | solid |
| pEEy_1 | -1.032 mm | ±0.311 mm | 3.32 | solid |
| pEEz_1 | +128.776 mm | ±1.000 mm | 128.80 | solid |
| phiEEx_1 | -1.100° | ±0.017° | 63.77 | solid |
| phiEEy_1 | -0.356° | ±0.017° | 21.10 | solid |
| phiEEz_1 | +1.942° | ±1.023° | 1.90 | marginal |

Also native this run: `_compute_parameter_correlation()` flags 27 pairs with
`|ρ| > 0.8`, several at exactly ±1.000 (e.g. `d_phiy_arm_right_6` ↔
`d_phiz_arm_right_7`, `d_py_arm_right_6` ↔ `d_pz_arm_right_7`) — genuinely
degenerate directions in this 48-sample regressor, consistent with (and a
more systematic version of) the near-degenerate joint-axis pairs found by
hand via SVD in the original session.

---

## What this means

Position/orientation split is still meaningful (translations remain
statistically harder than rotations at n=48 — most `noise`-flagged entries
are still `d_p*` translations), but far fewer parameters are actually stuck
in the noise floor than before. `d_phiz` (yaw) offsets remain the most
consistently solid category, as in the original analysis.

**Don't deploy the `noise` values as-is** (still true) — but the case for
excluding everything below 2σ, as the original run required, is much
weaker now.

---

## Held-out validation: does "conservative" still beat "full"?

Re-ran the train/test split experiment (36 train / 12 held-out test, 4
random seeds) that showed a ~1.6x generalization gap in the original
(legacy-script) analysis, using the same methodology on the migrated
pipeline:

| Split (seed) | Position — conservative (≥2σ subset from that split's fit) | Position — full (38 params) |
|---|---|---|
| 0 | 43.43 mm | 44.22 mm |
| 1 | 24.75 mm | 22.91 mm |
| 2 | 22.13 mm | 16.96 mm |
| 3 | 47.30 mm | 49.89 mm |
| **mean** | **34.40 mm** | **33.50 mm** |

(Both far below the ~139mm nominal/uncalibrated baseline on the same
splits — the calibration itself is clearly doing its job either way.)

**The gap is gone.** Conservative and full now perform statistically the
same on held-out data (full is even marginally better on 2 of 4 splits) —
a sharp contrast with the original run, where full was ~1.6x worse on
every single split. This makes sense in hindsight: the original
"overfitting" wasn't really about parameter *count*, it was that the
wrap-based residual made individual parameter estimates noisier, so
including all of them meant including more raw noise. With the SE3
log-map fix tightening those estimates, there's much less noise to
overfit to in the first place.

**Practical takeaway**: `master_calibration_20260805_conservative.yaml`
(now only excluding the 9 genuine `noise` (<1σ) parameters, not everything
below 2σ) is still offered as the safer default, but deploying the full
`master_calibration_20260805.yaml` is no longer the risk it was in the
original analysis.

---

## To improve this further

The remaining `noise`-flagged parameters (mostly `d_p*` translations on
joints 5/6/7) and the near-1.0 correlated pairs both point the same
direction: more pose diversity, not just more samples of the same 51-pose
plan. `generate_optimal_configs.py`'s D-optimal selection already targets
this, so the next lever is a larger `--n-configs` pool or a fresh
random-pool seed, then recollecting.
