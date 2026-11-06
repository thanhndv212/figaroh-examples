# LC1: log-Cholesky against OLS and OLS+SDP on UR10 (simulated)

Issue: [figaroh-examples #11](https://github.com/thanhndv212/figaroh-examples/issues/11).
Core: [figaroh-plus#25](https://github.com/thanhndv212/figaroh-plus/issues/25)
(decision record `docs/decisions/log-cholesky-boundary-aware.md`, **go**),
implemented in figaroh-plus#235 with the null-space fix #236 (#234).

> **SIMULATED DATA.** Every number below comes from the UR10 dynamic-truth
> fixture (`examples/ur10/data/truth`, [fixture note](ur10-dynamic-truth-fixture.md)).
> Effort is Pinocchio RNEA on a drawn truth model plus seeded noise. Nothing is
> validated on hardware, and the benchmark exports no URDF.

## Minimum core dependency

`figaroh.identification.log_cholesky_fit` and `physical_fit.method:
log_cholesky` are on figaroh-plus **`devel` ≥ `21994ed`** (#235 and #236). They are
unreleased: the 1.1.0 release does not have them, and they are listed under
`[Unreleased]` in core's CHANGELOG. The examples README still pins
`figaroh>=1.1,<1.2`. On an older core:

- `log_cholesky_benchmark.py` prints a skip and exits 3.
- The `log-cholesky` reference case is not listed.
- The tests skip.

The overlay config does not load on an older core, which rejects the unknown method.

## Commands

From `examples/ur10`, in `figaroh-dev`:

```bash
# full grid: noise-free, low x 5 seeds, high x 5 seeds (about 5 min)
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MPLBACKEND=Agg \
  python log_cholesky_benchmark.py --output-dir /tmp/lc1

# bounded smoke run (low noise seed 101; nominal, repaired-OLS and unrepaired-OLS starts; ~20 s)
MPLBACKEND=Agg python log_cholesky_benchmark.py --smoke --output-dir /tmp/lc1-smoke

# the log-Cholesky stage through selection, verification, report and URDF export
python identification_reference.py --case log-cholesky --root /tmp/runs
```

The benchmark reads the fixture files only and writes `<stem>.json` and
`<stem>.md` into `--output-dir`. Exit codes:

| Code | Meaning |
|---|---|
| 0 | Every gate is met. |
| 1 | A gate is unmet; the report names it. |
| 2 | The preflight failed; no method is run. |
| 3 | Core has no log-Cholesky fit. |

The three methods are three overlays of `config/ur10_unified_config.yaml`. They
share one training split, the unweighted rows, the nominal-URDF prior and
`prior_weight` 1e-6:

| Method | Overlay | θ scored |
|---|---|---|
| Linear baseline (OLS) | `config/ur10_lc1_ols.yaml`: `select_stage: fit`, null-space reconstruction | `reconstruction.theta_r_dict`; the 11 unidentifiable columns stay at nominal |
| Corrected OLS+SDP | `config/ur10_lc1_ols_sdp.yaml`: adds `physical_consistency` (cvxopt) | `physical consistency.projected_parameters` (the per-link LMI projection, figaroh-plus#163). Reported only; it is not a selectable stage |
| Log-Cholesky | `config/ur10_lc1_log_cholesky.yaml`: `select_stage: physical_fit`, `method: log_cholesky`, `branch: auto`, `repair_start: false` | `ident.selected.values` |
| Direct (reference row) | the log-Cholesky overlay with `method: direct` | `ident.selected.values` |

## Protocol

| Item | Value |
|---|---|
| Model | Nominal prior `examples/ur10/urdf/ur10_robot.urdf`; truth `data/truth/ur10_truth.urdf` |
| Joints, in order | `shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`, `wrist_2_joint`, `wrist_3_joint` |
| Parameters | 60, per joint: m, mx, my, mz, Ixx, Ixy, Iyy, Ixz, Iyz, Izz. Units: kg, kg·m, kg·m² at the joint frame. No friction or armature terms |
| Training | `train.csv`: 24 s spline, 2400 rows at 100 Hz |
| Held-out | `validation.csv`: an independent 8 s Fourier trajectory, 800 rows. It is never used in a fit |
| Derivatives | Analytic q/dq/ddq, unfiltered and undecimated |
| Torque provenance | RNEA on the truth model. The `low`/`high` levels add seeded Gaussian effort noise and position noise (`protocol.yaml`) |
| Datasets | `none`; `low` and `high` × training seeds 101–105, paired with held-out seeds 201–205 |
| Perturbed starts | Nominal plus N(0, 0.2²) in log-Cholesky coordinates, seeds 1101 and 1102 |
| Rank | 36 of 60. 11 zero columns are eliminated, and 24 directions are null in the regressor |

Effort is in N·m. The training objective is J = ‖Yθ − τ‖² + 1e-6 ‖(θ − θ₀)/D‖²,
summed over the stacked training rows, in N²·m². Each method is scored on the
same 60-column held-out regressor times θ, against two references:

- the noisy measurement, the way the regressor is built from the measured q;
- the noise-free truth effort.

### Acceleration preflight

The preflight runs before any fit and aborts the run (exit 2) if a check fails.
On both splits, and for every active coordinate, it checks:

- **Derivatives.** Analytic dq and ddq against central differences of the
  trajectory definition, away from spline knots. The tolerance is 1e-6; observed ≤1.3e-9.
- **Excitation.** Every joint's ddq RMS is ≥1e-2 rad/s². Observed minimums:
  0.42 rad/s² (training) and 1.54 rad/s² (held-out).
- **Dynamics.** CRBA+NLE, the Pinocchio regressor, ABA and the FIGAROH regressor
  against the stored effort, ≤1e-8 N·m.

It also checks that the accelerations the pipeline hands to the regressor are
the fixture ddq, bit for bit.

## Results (Pinocchio 3.7, archived)

The archive is
`results/ur10-log-cholesky-lc1-pin37.json` and `.md`. It records examples
`v1.1.0-5-g032affd` (clean tree; the script and config SHA-256 identify the
exact files), core `v1.1.0-21-g21994ed` (`devel`), Python 3.12.5, NumPy 2.4.3,
SciPy 1.17.1, Pinocchio 3.7.0, PICOS 2.6.1 and CVXOPT 1.3.2, with single-threaded
BLAS. It also holds the file hashes of the fixture, URDFs, configs and script.
**Pinocchio 4.1 was not run**: no 4.1 environment is available on this machine
(the 4.1 profile from the D4 note was a read-only clone env).

Held-out aggregate RMSE (N·m):

| Noise | Seed | OLS | OLS+SDP | Log-Cholesky | Log-Cholesky vs truth | LC / SDP |
|---|---|---|---|---|---|---|
| none | — | 2.1e-14 | 0.0212 | 5.84e-6 | 5.84e-6 | 0.0003 |
| low | 101–105 | 0.1867–0.1923 | 0.1879–0.2040 | 0.1867–0.1923 | 0.016–0.025 | 0.930–0.997 |
| high | 101–105 | 0.9336–0.9618 | 0.9521–1.0339 | 0.9334–0.9611 | 0.074–0.121 | 0.915–0.999 |

Per seed and per joint, see the archived `.md`. Log-Cholesky equals the direct
physical fit on every dataset; on boundary fits they are the same SDP.

- **Training objective.** Log-Cholesky has the lowest J among the physical
  estimates on every dataset. OLS+SDP is 0.3–11.7% higher on the noisy
  datasets, and 2.32 against 0.0028 N²·m² on noise-free data, because it
  projects the parameters per link without looking at the torques.
- **Feasibility.** The OLS reconstruction is non-physical on every dataset: its
  min pseudo-inertia eigenvalue ranges from −0.07 to −0.02, on 1–5 links,
  always including the elbow. OLS+SDP and log-Cholesky are feasible on every link; the
  smallest log-Cholesky eigenvalue is −4.2e-10, inside the 1e-8 oracle tolerance.
- **Conditioning.** Rank 36, null-space dimension 24, base-regressor condition
  number 92.7, identical for every dataset.
- **Runtime.** Each fit takes at most 0.046 s (≤20 s gate). A full pipeline run takes
  under 1 s.

### Finding: UR10 always takes the boundary branch

The unconstrained ridge minimiser has min eig between −0.18 and −0.13 on 2–5
links, even on noise-free data (−0.155 on 3 links). The reason is the prior: with `prior_weight`
1e-6 it anchors the 24 null-space directions at the nominal URDF, and nominal
plus the identified part is not a physical body.

So the production `auto` branch always runs the direct LMI fit. The start is
unused (`start_used: false`), and every start gives a bit-identical θ. Two
consequences follow:

- The start-sensitivity gate is met on the production path, but trivially.
- The boundary results sit at min eig ≈ 0, not at the 1e-6 margin that the
  decision record's private spike reported. Core's direct SDP constrains
  P ⪰ 0, and `eigenvalue_margin` is used only to *detect* the branch.

To test the interior solver where it is the correct branch, the benchmark adds
an **oracle interior check**. This is a solver diagnostic, not an estimator. It
uses noise-free data with the *truth* as the prior. Every truth link has min
eig ≥ 1.3e-3, so the ridge minimiser is the truth and detection picks
`interior`. From nominal, repaired-OLS and both perturbed starts, dogbox stops
by the 1e-7 N·m identifiable-residual rule after 8–43 evaluations, within
1e-11 of the truth. Even the truth prior lands on the boundary once low noise
is added, because the wrist links have small margins.

### Initialization sensitivity and nonconvergence

The benchmark forces `branch: interior` on the first dataset of each noise
level. This is a diagnostic; `auto` never chooses it here. On those problems
the optimum is on the boundary, which log-Cholesky coordinates cannot reach.
Every valid start therefore stops at `max_nfev` (2000 evaluations, about 2 s)
with identifiable RMS 0.95–47 N·m, returns θ `None`, and the run keeps the
previous stage. This is the designed failure, not a solver defect.

Reading a record (`result["physical_fit"]["log_cholesky"]`, or
`SolveRecord.notes`):

| You see | Meaning | Action |
|---|---|---|
| `branch: boundary`, `solver_status: optimal` | The SDP decided the fit; the start was ignored. | None. |
| `branch: interior`, `interior.stop_reason: converged` | The identifiable residual distance is ≤`stop_rms`. | None. |
| `stop_reason: max_nfev`, `time_budget` or `not_converged`, with θ `None` | Nonconvergence. `physical_fit` is rejected and selection falls back to the previous valid stage (`fit`/`reconstruction`), which `select_stage` and the verdict report. | Check whether the branch was forced (`branch_forced`). With `auto`, raise `max_nfev`/`max_seconds` only after checking `identifiable_rms`; a large value means the optimum is outside the interior. |
| `solver_status: invalid_start` | The start has a link with a non-positive-definite pseudo-inertia; it raises before any evaluation. The unrepaired OLS start always does this (elbow min eig −0.026). | Pass a physical start, or set `repair_start: true` (the repairs are recorded in `start_repairs`). It cannot occur with `auto` here, because the boundary branch ignores the start. |

## Decision-record gates

| Gate | Value | Limit | Met |
|---|---|---|---|
| Clean held-out RMSE vs truth | 5.84e-6 N·m | ≤1e-5 | yes |
| Noisy held-out, log-Cholesky / OLS+SDP (SDP ok on all 10) | max 0.9987 | ≤1.05 | yes |
| Feasibility, min eig over every link and fit | −4.2e-10 | ≥−1e-8 | yes |
| Fit runtime | max 0.046 s | ≤20 s | yes |
| Nominal and repaired-OLS starts converge and are feasible | production: yes (start unused); oracle interior: yes | all | yes |

Gates not covered here, so **the tracker stays open** for them:

- **Pinocchio 4.1.** The decision record requires both profiles; only 3.7 was run.
- **Weak excitation.** The fixture has one well-excited training trajectory,
  with no weak-excitation case.
- **Hardware.** No recorded robot data is used.

The near-boundary (perturbed) and invalid starts are reported, not gated.

## Selected stage into predictions, reports and export

- **Benchmark.** For log-Cholesky and direct, the benchmark checks that core's
  held-out prediction (`validation_metrics.tau_identified_per_joint`) equals
  `Y_val θ_selected`; the gap is 0.
- **Reference case.** `identification_reference.py --case log-cholesky` runs the
  stage through selection, verification (scope `prediction`), the HTML report
  (which names `log_cholesky, boundary branch`), URDF export into the lumped
  nominal, reload parity (2.9e-10 N·m) and the archive audit. The reloaded URDF
  is within 0.037 N·m of the truth on the held-out split. That export is of a
  fit to simulated data: it is a pipeline check, not a robot model.

## Tests

`tests/test_ur10_log_cholesky_benchmark.py` covers:

- the overlay layout;
- config parsing through the pipeline;
- the smoke run into `tmp_path` (gates, preflight, the boundary branch, prediction
  flow, forced-interior nonconvergence, `invalid_start`, oracle convergence);
- the `log-cholesky` reference case.

The core-dependent tests skip on an older core. `validate.py` also runs the
reference case through `identification_reference.py --case all`.
