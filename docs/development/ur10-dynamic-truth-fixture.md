# UR10 dynamic identification: truth fixture and benchmark protocol

Issue: [#21](https://github.com/thanhndv212/figaroh-examples/issues/21).
Delivery package: [D3 / core #38](https://github.com/thanhndv212/figaroh-plus/issues/38).
Follows the [signal audit](ur10-signal-audit-2026-10-02.md) (#19, D2).

The legacy UR10 CSVs have no saved generator or parameter vector, so no
estimator can be checked against ground truth on them. This fixture is
generated from scratch with a known truth, analytic derivatives and an
independent validation trajectory, and is frozen for the D4 comparison
(examples #22, core #59).

- Generator and loaders: `examples/ur10/identification_truth.py`.
- Fixture: `examples/ur10/data/truth/` (1.0 MB).
- Frozen protocol: `examples/ur10/data/truth/protocol.yaml` (v1).
- Tests: `tests/test_ur10_identification_truth.py` (11 tests, about 4 s).

```bash
cd examples/ur10
python identification_truth.py                 # check the fixture, baseline table
python identification_truth.py --write /tmp/t  # regenerate elsewhere and compare
```

## 1. Design

| Item | Choice |
|---|---|
| Model | `urdf/ur10_robot.urdf` (UR10e kinematics, sha256 `1da0c0de…`), six revolute joints. Pinocchio merges the links attached by fixed joints into their moving body; the wrist_3 body includes the tool mount, tool and camera. |
| Truth | Each body inertia of that URDF perturbed with seed 0: mass × exp(N(0, 0.1)); COM + N(0, 2 cm) per axis; principal second moments of mass × exp(N(0, 0.2)); principal axes rotated by N(0, 0.2 rad). Perturbing the second moment, not the inertia, keeps every link physically consistent by construction (smallest pseudo-inertia eigenvalue 0.0013–0.019). Masses move by −23 % to +19 %. |
| Extras | None: no friction, actuator inertia or joint offset in the truth. |
| Truth files | `ur10_truth.urdf`: the URDF with each moving link's inertial replaced by its whole body (COM frame unrotated) and the attached links' inertials removed. `truth_parameters.csv`: the 60 FIGAROH standard parameters, truth and nominal. Reloading the URDF reproduces the generated parameters exactly. |
| Trajectories | One period of a 5-harmonic Fourier series per joint around the UR home posture `[0, −π/2, π/2, −π/2, −π/2, 0]`, gain per joint so that \|q − centre\| ≤ 1.2 rad and \|dq\| ≤ 0.7 × velocity limit; candidates whose effort exceeds 0.8 × torque limit are rejected. Analytic q, dq, ddq are saved. No collision checking. |
| Training | 0.1 Hz fundamental, 10 s, 1000 rows. Seed 18: the best-conditioned of seeds 0–19 (base condition number 77). |
| Validation | 0.125 Hz fundamental, 8 s, 800 rows. Seed 100 (no selection; condition number 72). Never used to fit, select or tune. |
| Clock | 100 Hz, exact (simulation). |
| Effort | Pinocchio RNEA on the truth model, joint-side N·m. |
| Noise | Not stored. Effort σ per joint = 1 % (`low`) or 5 % (`high`) of that joint's noise-free training effort RMS; position σ = 1e-5 or 1e-4 rad. Seeds 101–105 (training) and 201–205 (validation), drawn with NumPy's legacy `RandomState`, whose stream is frozen; the manifest keeps a hash of every draw. |

Effort noise is relative because the joints differ by three orders of
magnitude: the training effort RMS is 47 N·m at the shoulder lift and 0.07
N·m at wrist_3. One absolute level would leave one joint noise-free and
swamp another.

## 2. Independent checks of the saved effort

Every check runs on the saved CSV and the reloaded truth URDF (largest
discrepancy over all rows and joints):

| Check | Training | Validation |
|---|---:|---:|
| CRBA M(q) ddq + nonlinear effects − effort (N·m) | 2.8e-14 | 4.3e-14 |
| Pinocchio joint-torque regressor × truth − effort (N·m) | 3.6e-14 | 4.3e-14 |
| FIGAROH `build_regressor_basic` × standard parameters − effort (N·m) | 3.6e-14 | 4.3e-14 |
| ABA(q, dq, effort) − ddq (rad/s²) | 5.5e-14 | 6.2e-14 |
| Power balance d(T + V)/dt − effort · dq (W; peak power 98 / 106 W) | 3.5e-8 | 4.7e-8 |
| Analytic dq, ddq − central differences of the series, every coordinate | 7.3e-10 | 1.4e-9 |

The power balance uses only kinetic and potential energy, evaluated from the
Fourier coefficients at t ± 10 µs, so it does not share code with RNEA.
The FIGAROH regressor check also confirms the standard-parameter order.
The check found a real defect while the generator was written: the first
truth URDF left the camera mount's inertials on, so the reloaded wrist_3
body was 0.27 kg heavier and the effort was off by 2.6 N·m.

Excitation, per coordinate: every joint moves more than 1 rad, reaches more
than 0.5 rad/s and 1 rad/s², and carries effort; both splits have rank 36
(of 60 standard parameters) with the same base parameters.

## 3. Frozen protocol (v1)

`protocol.yaml` is generated with the fixture and is never edited; a change
becomes `protocol_version: 2` next to it.

- **Splits.** Fit on `train.csv` only; validate on `validation.csv`. No
  decimation, no edge exclusion.
- **Derivatives.** `analytic` (saved q/dq/ddq; isolates the estimator) or
  `differentiated` (q only, core `calculate_first_second_order_differentiation`
  at the fixture clock: first n − 2 rows, velocity half a step late, no
  filter in the baseline; a filter belongs to the method and is reported).
- **Rank.** The 60 standard parameters, zero-column tolerance 1e-6, and the
  36 base parameters of the training regressor: indices, names and true
  values (`rank.base_truth`).
- **Scaling.** Effort per joint: training effort RMS (NRMSE divides by it).
  Base parameters: error × the RMS of its training column, so each base
  error is the effort it accounts for (N·m).
- **Budgets.** 1000 iterations, 600 s wall time per fit, 5 starts (seeds
  0–4) for nonlinear methods. A fit that hits a budget is reported as not
  converged, never rerun with a larger budget under v1.
- **Metrics.** Training effort RMSE per joint; validation effort RMSE and
  NRMSE per joint against the noise-free truth, and RMSE against the noisy
  validation effort; scaled base-parameter error; for physical estimators,
  standard-parameter error per link (diagnostic only, not identifiable) and
  the pseudo-inertia minimum eigenvalue checked independently of the solver.
  Convergence and feasibility are reported separately; a solver error is
  not proof of infeasibility.
- **Rules.** No parameter truth from the legacy CSVs. Extras are zero in the
  truth: a frozen-extra comparison fixes them at zero, a joint-extra one
  reports them separately. Report the tested core/examples revisions, the
  Pinocchio version and the manifest's file hashes.

## 4. Baseline: base least squares

`python identification_truth.py` fits the base parameters by ordinary least
squares on the training split and judges them by the protocol (noisy rows:
five paired seeds; error columns are the mean, or the worst case for max):

| Derivatives | Noise | Base error RMS (N·m) | Base error max (N·m) | Validation RMSE max (N·m) | Validation NRMSE max (%) |
|---|---|---:|---:|---:|---:|
| analytic | none | 2.8e-14 | 7.3e-14 | 1.5e-13 | 6.4e-11 |
| analytic | low | 0.012 | 0.040 | 0.038 | 17.5 |
| analytic | high | 0.062 | 0.20 | 0.19 | 87.7 |
| differentiated | none | 0.028 | 0.069 | 0.21 | 42.1 |
| differentiated | low | 0.037 | 0.13 | 0.81 | 48.9 |
| differentiated | high | 1.0 | 3.7 | 6.5 | 406 |

Per joint, validation NRMSE (mean of five seeds), analytic derivatives:
`low` 0.6, 0.06, 0.09, 0.8, 4.1, 12.6 %; `high` 2.9, 0.3, 0.5, 4.2, 20.3,
63.1 %. The absolute errors are similar on every joint (0.01–0.03 N·m at
`low`), so the wrists' small efforts make their relative error large: the
unweighted stacked fit lets shoulder noise leak into the wrist parameters.
This is the gap weighted least squares and physical estimators are expected
to close in D4.

The same fit through the UR10 identification pipeline
(`identification_truth.identification()`, then `solve(decimate=False)`)
selects the same 36 base parameters and predicts the validation effort to
4e-6 N·m (nominal model: 2.8 N·m). Its base parameters differ from the
truth by up to 5e-7 because core rounds them to six decimals in
`QRDecomposer.double_decomposition` (`figaroh/tools/qrdecomposition.py`),
despite the comment there saying full precision; the test bounds the error
by that rounding.

## 5. What this supports and what it does not

It supports judging an estimator against a known truth on a known clock:
base parameters exactly, standard parameters as a diagnostic, and
held-out effort against the noise-free truth.

It does not include model mismatch: no friction, actuator inertia, joint
flexibility, effort offsets or timing error, and no collision checking of
the trajectories. Results on it are not hardware evidence. The excitation
is a random Fourier series, not an optimised trajectory. The legacy CSVs
remain unchanged and keep no truth claim.

## 6. Reproduction

| Item | Value |
|---|---|
| Core | figaroh-plus `devel` `c80b5fc` (via `PYTHONPATH=<core>/src`) |
| Environment | `figaroh-dev`: Python 3.12.11, Pinocchio 3.7.0, NumPy 2.3.4 |
| `ur10_truth.urdf` | `e3ce01ca3579c6a2057674447dc445bbc46ff37e24da9111ad9d7b4246efee13` |
| `truth_parameters.csv` | `bc39953ab222dcdaf75c6445b4d03c49d1ccdcd4647d64715ea0116b2d3843af` |
| `train.csv` | `3ec88a46b0a45af6355d48ad982005198a9d25409384129b58856b9698aa0cf2` |
| `validation.csv` | `7579e11b6406cd4033ffcda682ab86e3130a6acb3cf8e7cacd8cf81c527f5644` |

On macOS arm64, regenerating with Pinocchio 4.1.0 (a `figaroh-dev` clone with
core's `ci/pinocchio-4.1.0.txt` stack) gives byte-identical fixture files and
protocol, a manifest that differs only in its `environment` block, the same
baseline table, and passes the fixture and signal-processing tests (21).
Another platform may differ in the last bits of `sin`/`cos`; the test
compares regenerated arrays within 1e-9 and the committed files by hash.
