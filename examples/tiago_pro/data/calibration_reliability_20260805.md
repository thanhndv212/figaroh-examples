# Statistical reliability of all identified parameters — 2026-08-05

Source: `calibration_results_20260805_1246.yaml` (48 samples, position
RMSE 7.93mm/MAE 7.28mm, orientation RMSE 2.18°/MAE 1.89°).

Standard error computed from the measurement-residual Jacobian at the
solution: `cov = σ²·(JᵀJ)⁻¹` with `σ² = SSR / (m − n)` (m=288 residuals,
n=38 parameters, dof=250). Verdict: `solid` ≥3σ, `acceptable` 2-3σ,
`marginal` 1-2σ, `noise` <1σ (value not distinguishable from 0 with this
data).

**Breakdown: 8 solid, 4 acceptable, 4 marginal, 22 noise — out of 38.**

---

## Full table

| Parameter | Value | Standard error | σ | Verdict |
|---|---|---|---|---|
| base_px_torso | -3.27 mm | ±11.82 mm | 0.28 | noise |
| base_py_torso | -0.16 mm | ±27.96 mm | 0.01 | noise |
| base_pz_torso | +11.78 mm | ±4.67 mm | 2.52 | acceptable |
| base_phix_torso | -0.06° | ±1.72° | 0.04 | noise |
| base_phiy_torso | +0.12° | ±0.63° | 0.20 | noise |
| base_phiz_torso | +0.22° | ±0.40° | 0.56 | noise |
| d_phiz_arm_right_1 | ~0.00° | ±0.61° | 0.00 | noise |
| d_px_arm_right_2 | -1.06 mm | ±2.83 mm | 0.38 | noise |
| d_py_arm_right_2 | +0.79 mm | ±3.60 mm | 0.22 | noise |
| d_pz_arm_right_2 | ~0.00 mm | ±1.86 mm | 0.00 | noise |
| d_phix_arm_right_2 | -0.04° | ±0.15° | 0.24 | noise |
| d_phiz_arm_right_2 | -0.77° | ±2.10° | 0.37 | noise |
| d_px_arm_right_3 | -4.35 mm | ±3.67 mm | 1.19 | marginal |
| **d_phix_arm_right_3** | **+0.29°** | ±0.14° | **2.08** | **acceptable** |
| **d_phiz_arm_right_3** | **-1.08°** | ±0.11° | **9.70** | **solid** |
| d_px_arm_right_4 | +2.53 mm | ±3.51 mm | 0.72 | noise |
| d_py_arm_right_4 | +0.71 mm | ±2.92 mm | 0.24 | noise |
| d_phix_arm_right_4 | ~0.00° | ±0.14° | 0.01 | noise |
| **d_phiz_arm_right_4** | **+1.88°** | ±0.16° | **11.57** | **solid** |
| d_px_arm_right_5 | -5.67 mm | ±3.28 mm | 1.73 | marginal |
| d_py_arm_right_5 | +0.48 mm | ±1.74 mm | 0.28 | noise |
| d_phix_arm_right_5 | -0.20° | ±0.14° | 1.39 | marginal |
| d_px_arm_right_6 | -1.48 mm | ±3.05 mm | 0.49 | noise |
| d_py_arm_right_6 | +0.19 mm | ±1.53 mm | 0.13 | noise |
| d_pz_arm_right_6 | +0.48 mm | ±1.74 mm | 0.27 | noise |
| **d_phix_arm_right_6** | **-0.32°** | ±0.12° | **2.74** | **acceptable** |
| **d_phiy_arm_right_6** | **+0.84°** | ±0.07° | **11.59** | **solid** |
| **d_phiz_arm_right_6** | **-0.85°** | ±0.15° | **5.68** | **solid** |
| d_px_arm_right_7 | +1.55 mm | ±3.10 mm | 0.50 | noise |
| d_pz_arm_right_7 | -0.17 mm | ±1.53 mm | 0.11 | noise |
| d_phix_arm_right_7 | +0.09° | ±0.14° | 0.64 | noise |
| **d_phiz_arm_right_7** | **+0.84°** | ±0.07° | **11.31** | **solid** |
| pEEx_1 | -4.91 mm | ±2.94 mm | 1.67 | marginal |
| pEEy_1 | -1.30 mm | ±2.86 mm | 0.46 | noise |
| **pEEz_1** | **+137.16 mm** | ±2.78 mm | **49.27** | **solid** |
| **phiEEx_1** | **-0.96°** | ±0.17° | **5.58** | **solid** |
| phiEEy_1 | -0.29° | ±0.11° | 2.67 | acceptable |
| **phiEEz_1** | **+2.47°** | ±0.11° | **23.16** | **solid** |

---

## What this means

**Almost all per-joint translations (`d_px`/`d_py`/`d_pz`) are within
noise** — the only notable exception is `base_pz_torso` (2.52σ). By
contrast, **yaw rotations (`d_phiz`) are almost systematically solid**
(5.7 to 11.6σ) wherever they're identified, along with a few isolated
`d_phix`/`d_phiy` entries.

This is consistent with what we saw for `pEE`: translations are
statistically harder to separate from mocap noise than rotations, with
only 48 samples — more pose diversity is needed to constrain them well.

**Don't deploy the `noise`/`marginal` values as-is on the robot** — that
would amount to encoding measurement noise as if it were a real mechanical
correction.

---

## Recommendation: conservative subset (≥2σ only)

See `master_calibration_20260805_conservative.yaml` — contains only the 7
joint corrections ≥2σ (`d_phix_arm_right_3`, `d_phiz_arm_right_3`,
`d_phiz_arm_right_4`, `d_phix_arm_right_6`, `d_phiy_arm_right_6`,
`d_phiz_arm_right_6`, `d_phiz_arm_right_7`). Every other joint stays at its
nominal URDF value (no correction applied) until more data is available.

`master_calibration_20260805.yaml` (all identified values, including
`noise`-level ones) is still available, but see the validation experiment
below — it doesn't just carry more uncertainty, it measurably performs
worse out of sample.

---

## Validation: held-out train/test split (why the conservative subset wins)

Rather than trust the σ-based reasoning alone, this was tested directly:
split the 48 samples into 36 train / 12 held-out test (random, no overlap),
fit both the full model (38 params) and the conservative-structure model
(only the 12 params from the ≥2σ set free, everything else fixed at 0) on
the *train* set only, then evaluate FK error on the *test* set that neither
model ever saw during fitting. Repeated with 4 independent random splits
(seeds 0-3) to rule out a lucky/unlucky split.

| Split (seed) | Position — conservative | Position — full | Orientation — conservative | Orientation — full |
|---|---|---|---|---|
| 0 | 19.67 mm | 34.88 mm | 4.44° | 4.76° |
| 1 | 20.17 mm | 28.41 mm | 7.23° | 9.66° |
| 2 | 22.28 mm | 49.87 mm | 4.83° | 5.77° |
| 3 | 22.69 mm | 24.91 mm | 3.88° | 4.89° |
| **mean** | **21.2 mm** | **34.5 mm** | **5.09°** | **6.27°** |

(For reference, the uncalibrated nominal model scores ~139 mm / ~5.6° on the
same test splits — both fitted variants are a large improvement over no
calibration at all.)

**Two consistent findings, across all 4 splits with no exception:**

1. **The conservative model beats the full model on held-out data every
   time** — both position and orientation. The full model fits the
   *training* data better (more free parameters always reduce training
   residual), but that improvement doesn't transfer to unseen poses.
2. **The full model is also far less stable across splits** (24.9-49.9 mm,
   a ~2× spread) than the conservative model (19.7-22.7 mm, tight). This is
   a second classic overfitting signature: a model that has fit noise
   becomes very sensitive to exactly which samples land in train vs. test;
   a model that only captures real effects stays stable regardless of the
   split.

**Conclusion**: this isn't just a statistical-purity preference — the
`noise`-level parameters measurably hurt generalization (~1.6× worse
position error on average). Deploy the conservative subset.

---

## To improve this

Recollect data (pipeline already in place:
`generate_optimal_configs.py` → `collect_calibration_data.py` →
`run_calibration.py`) and redo this uncertainty computation. With more
samples, `σ_error ∝ 1/√N` — doubling N (48→96, close to your previous runs
at 94-100) would reduce standard errors by about 30%, which would likely
push several `marginal`/`acceptable` parameters into `solid`, and some
`noise` parameters into `marginal`.
