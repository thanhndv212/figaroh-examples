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
- Fixture: `examples/ur10/data/truth/` (1.7 MB).
- Frozen protocol: `examples/ur10/data/truth/protocol.yaml` (v1).
- Tests: `tests/test_ur10_identification_truth.py` (13 tests, about 40 s).

```bash
cd examples/ur10
python identification_truth.py                    # check the fixture, baseline table
python identification_truth.py --write /tmp/t     # regenerate elsewhere and compare
python identification_truth.py --optimize /tmp/o  # rerun the trajectory optimiser (slow)
```

## 1. Design

| Item | Choice |
|---|---|
| Model | `urdf/ur10_robot.urdf` (UR10e kinematics, sha256 `1da0c0de…`), six revolute joints, mounted on a 0.88 m table next to a wall. Pinocchio merges the links attached by fixed joints into their moving body; the wrist_3 body includes the tool mount, tool and camera. |
| Truth | Each body inertia of that URDF perturbed with seed 0: mass × exp(N(0, 0.1)); COM + N(0, 2 cm) per axis; principal second moments of mass × exp(N(0, 0.2)); principal axes rotated by N(0, 0.2 rad). Perturbing the second moment, not the inertia, keeps every link physically consistent by construction (smallest pseudo-inertia eigenvalue 0.0013–0.019). Masses move by −23 % to +19 %. |
| Extras | None: no friction, actuator inertia or joint offset in the truth. |
| Truth files | `ur10_truth.urdf`: the URDF with each moving link's inertial replaced by its whole body (COM frame unrotated) and the attached links' inertials removed. `truth_parameters.csv`: the 60 FIGAROH standard parameters, truth and nominal. Reloading the URDF reproduces the generated parameters exactly. |
| Training | Core's exciting-trajectory optimiser, as in `optimal_trajectory.py` (IPOPT on the base-regressor condition number, unified config): 2 stacked segments of 6 rest-to-rest C2 spline pieces, 2 s each, 24 s, 2400 rows. Joint limits narrowed to a collision-free box (below). Best of seeds 0–15 (condition number 61.7, seed 15; the others 64–130). Frozen as `train_waypoints.json`: IPOPT results depend on the numerical stack (#60), so the fixture is regenerated from these waypoints, not from the optimiser. |
| Validation | A different family, never optimised: one period of a 5-harmonic Fourier series at 0.125 Hz, 8 s, 800 rows, around the UR home posture `[0, −π/2, π/2, −π/2, −π/2, 0]`, \|q − home\| ≤ 1.2 rad, \|dq\| ≤ 0.7 × velocity limit. The first feasible of seeds 100–119 (seed 104; 100–103 collide). Condition number 199. |
| Feasibility, both splits | URDF joint limits, velocity limits, truth effort ≤ 0.8 × torque limit, and collision clearance (below). |
| Clock | 100 Hz, exact (simulation). Analytic q, dq, ddq are saved. |
| Effort | Pinocchio RNEA on the truth model, joint-side N·m. |
| Noise | Not stored. Effort σ per joint = 1 % (`low`) or 5 % (`high`) of that joint's noise-free training effort RMS; position σ = 1e-5 or 1e-4 rad. Seeds 101–105 (training) and 201–205 (validation), drawn with NumPy's legacy `RandomState`, whose stream is frozen; the manifest keeps a hash of every draw. |

Effort noise is relative because the joints differ by three orders of
magnitude: the training effort RMS is 38 N·m at the shoulder lift and 0.07
N·m at wrist_3. One absolute level would leave one joint noise-free and
swamp another.

### Collision

The URDF's collision geometry (arm, robot base, wrist camera mount, camera,
tool, table and wall) plus a floor box the URDF lacks; 71 pairs of
geometries on different, non-adjacent joints. Required clearance: 5 cm to
the table, wall and floor, 1 cm between robot bodies. It is checked on the
analytic trajectory at 400 Hz (a robot point moves at most about 7 mm
between checks).

| Split | Closest to environment | Closest robot pair |
|---|---|---|
| Training | 10.5 cm (upper arm, table top plate) | 2.1 cm (base, upper arm) |
| Validation | 6.1 cm (tool, table) | 2.1 cm (base, upper arm) |

The base–upper arm pair is 2.1 cm apart at home and does not depend on the
motion. The first version of this fixture (before collision checking) had
the wrist camera 3.9 cm inside the table in training and the tool 2.1 cm
inside the upper arm in validation.

**Training joint box.** Offsets from home (rad): shoulder_pan ±π,
shoulder_lift and elbow −0.8 to +0.2, wrist_1 ±1.2, wrist_2 −0.2 to +1.5,
wrist_3 −1.5 to +1.2. It excludes the two contacts found by sampling: the
tool reaching the table when shoulder_lift and elbow both lower the arm,
and the camera mount reaching the forearm when wrist_3 turns towards it.
4000 uniform samples of the box are collision free, and a rest-to-rest
spline piece stays between its two waypoints in every joint. Every
optimised trajectory (16 of 16) passed the 400 Hz check; the box costs
excitation (unconstrained runs reached condition numbers 18–33, all
through the table).

## 2. Independent checks of the saved effort

Every check runs on the saved CSV and the reloaded truth URDF (largest
discrepancy over all rows and joints):

| Check | Training | Validation |
|---|---:|---:|
| CRBA M(q) ddq + nonlinear effects − effort (N·m) | 3.6e-14 | 4.3e-14 |
| Pinocchio joint-torque regressor × truth − effort (N·m) | 2.8e-14 | 2.8e-14 |
| FIGAROH `build_regressor_basic` × standard parameters − effort (N·m) | 2.8e-14 | 2.8e-14 |
| ABA(q, dq, effort) − ddq (rad/s²) | 6.3e-14 | 6.8e-14 |
| Power balance d(T + V)/dt − effort · dq (W; peak power 34 / 53 W) | 1.7e-8 | 2.3e-8 |
| Analytic dq, ddq − central differences of the curve, every coordinate | 1.3e-9 | 1.3e-9 |

The power balance uses only kinetic and potential energy, evaluated from the
trajectory definition at t ± 10 µs, so it does not share code with RNEA.
The FIGAROH regressor check also confirms the standard-parameter order.
On the spline, derivatives are compared away from the waypoints, where the
third derivative jumps; a test checks that the spline passes through each
waypoint at rest. The checks found a real defect while the generator was
written: the first truth URDF left the camera mount's inertials on, so the
reloaded wrist_3 body was 0.27 kg heavier and the effort was off by 2.6 N·m.

Excitation, per coordinate: every joint moves more than 0.8 rad, reaches
more than 0.7 rad/s and 1 rad/s², and carries effort; both splits have rank
36 (of 60 standard parameters) with the same base parameters.

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
| analytic | none | 2.3e-14 | 8.2e-14 | 5.3e-14 | 8.3e-12 |
| analytic | low | 0.0074 | 0.028 | 0.046 | 6.9 |
| analytic | high | 0.037 | 0.14 | 0.23 | 34.4 |
| differentiated | none | 0.0088 | 0.021 | 0.12 | 6.7 |
| differentiated | low | 0.049 | 0.18 | 0.84 | 32.2 |
| differentiated | high | 0.68 | 2.1 | 7.4 | 352 |

Per joint, validation NRMSE (mean of five seeds), analytic derivatives:
`low` 0.8, 0.09, 0.05, 0.4, 2.8, 5.1 %; `high` 3.8, 0.5, 0.3, 1.8, 14.1,
25.4 %. The absolute errors are similar on every joint (0.003–0.03 N·m at
`low`), so the wrists' small efforts make their relative error large: the
unweighted stacked fit lets shoulder noise leak into the wrist parameters.
This is the gap weighted least squares and physical estimators are expected
to close in D4. With the first (random, colliding) training trajectory the
same fits were 2.5 to 6 times worse; excitation matters.

The same fit through the UR10 identification pipeline
(`identification_truth.identification()`, then `solve(decimate=False)`)
selects the same 36 base parameters and predicts the validation effort to
4.8e-6 N·m RMSE (nominal model: 3.1 N·m). Its base parameters differ from the truth by up to 5e-7 because core
rounds them to six decimals in `QRDecomposer.double_decomposition`
(`figaroh/tools/qrdecomposition.py`), despite the comment there saying full
precision; the test bounds the error by that rounding.

## 5. Core findings

- **Base parameters rounded** ([figaroh-plus#142](https://github.com/thanhndv212/figaroh-plus/issues/142)). As above: `phi_b = np.round(..., 6)`.
- **The trajectory optimiser does not avoid collisions on the UR10**
  ([figaroh-plus#143](https://github.com/thanhndv212/figaroh-plus/issues/143)).
  `TrajectoryConstraintManager` builds its collision constraint from
  `robot.geom_model`, whose collision pairs come only from an SRDF; the UR10
  has none, so the constraint is empty. Unconstrained runs (seeds 0–8) put
  the arm 30–46 cm into the table. Given the fixture's 71 pairs, the
  constraint is evaluated at the waypoints only: 6 of 8 runs failed and
  the 2 that finished still crossed the table (23 cm) and the floor
  between waypoints, at about 20 times the run time. The fixture therefore
  narrows the joint box instead and checks the whole motion itself.

## 6. What this supports and what it does not

It supports judging an estimator against a known truth on a known clock:
base parameters exactly, standard parameters as a diagnostic, and held-out
effort against the noise-free truth, on motions that clear the robot's
table, wall, floor and itself.

It does not include model mismatch: no friction, actuator inertia, joint
flexibility, effort offsets or timing error. Collision clearance is checked
on the URDF's collision meshes, not on the real cell. The training
trajectory stops at every waypoint (core's rest-to-rest splines), and the
joint box limits shoulder and elbow travel to 0.8 rad. Results on it are not
hardware evidence. The legacy CSVs remain unchanged and keep no truth claim.

## 7. Reproduction

| Item | Value |
|---|---|
| Core | figaroh-plus `devel` `c80b5fc` (via `PYTHONPATH=<core>/src`) |
| Environment | `figaroh-dev`: Python 3.12.11, Pinocchio 3.7.0, NumPy 2.3.4 |
| `train_waypoints.json` | `012066ee2a0f975ec50f418cc20080a1bb25649a865904fc52041580df5c38b0` |
| `ur10_truth.urdf` | `e3ce01ca3579c6a2057674447dc445bbc46ff37e24da9111ad9d7b4246efee13` |
| `truth_parameters.csv` | `bc39953ab222dcdaf75c6445b4d03c49d1ccdcd4647d64715ea0116b2d3843af` |
| `train.csv` | `014cdb143d2744b2ea0ecb6d4a4c8b7f6d02f40f56a91805ee13e0faa7297a5a` |
| `validation.csv` | `3dec2056016206f2b2c2b8c81e4fa06c4243703382b7fbf9b98855303f3ec894` |

`--optimize` reruns the optimiser over seeds 0–15 (about 80 s each); the
committed file came from one `optimize_training([seed])` per seed in
parallel processes, which is equivalent. On another stack it may select a
different trajectory: that is why its output is committed.

On macOS arm64, regenerating from the committed waypoints with Pinocchio
4.1.0 (a `figaroh-dev` clone with core's `ci/pinocchio-4.1.0.txt` stack)
gives byte-identical fixture files and protocol; the manifest differs only
in its `environment` block and the last digits of three collision
distances. The fixture and signal-processing tests pass there (23). Another
platform may differ in the last bits of `sin`/`cos`; the test compares
regenerated arrays within 1e-9 and the committed files by hash.
