# Calibration harness: shared simulation and held-out protocol

Issue: [#96](https://github.com/thanhndv212/figaroh-examples/issues/96)
(C5 phase 0). Code: `examples/calibration_harness/`. Tests:
`tests/test_calibration_harness_*.py`.

Before this, the synthetic-truth study (`examples/tiago/calibration_truth.py`,
#26) and the held-out protocol (`examples/tiago/heldout_protocol.py`, #27)
were TIAGo-only scripts, and the protocol's sets were written twice (in the
script and in `protocol.yaml`). The harness lifts the robot-independent parts
so the next studies (#97, #98, #99) add an adapter, not a copy.

## Design

| Module | Owns |
|---|---|
| `simulation.py` | draw a truth, simulate measurements, fit, judge against the truth |
| `heldout.py` | a plan of folds from a frozen protocol; run, select, posture strata |
| `protocol.py` | the optional `heldout:` block of a protocol.yaml |

A robot supplies a `SimulationStudy` (how to draw, predict, fit, and write a
dataset) and, for held-out studies, a `fit`/`evaluate` pair. Nothing else is
robot-specific.

### Simulation

- Random streams are fixed: truth from `default_rng(seed)`, noise from
  `default_rng(10_000 + seed)`, one `rng.normal` call per noise draw. A seed
  reproduces its numbers; the TIAGo numbers are unchanged by the move
  (`tests/golden/calibration_harness_tiago.json`).
- `NoiseModel = (clean (n, m), level, rng) -> noisy`. `gaussian_mm` is the
  TIAGo model (isotropic, `level` mm). `anisotropic(sigmas)` scales a
  per-column sigma by `level`.
- `run_case(study, truth_kind, fit_level, seed, noise, workdir,
  estimation=None, label=None)` returns the dict the TIAGo script always
  returned (`heldout_rmse_mm`, `strata_rmse_mm`, `z_scores`, ...).
  `run_grid` loops truth x noise x fit x seed; `summarize` gives the table
  and the pooled z summary.
- Held-out error is the fit's prediction minus the **noise-free truth's**
  prediction on held-out postures, never against noisy measurements.
- `simulate` can be overridden for noise that is not additive on the
  measurement (encoder noise propagated through IK): it returns
  `(measurements, postures_used)`.

### Held-out plans

`load_plan(path, verify=True)` reads a protocol.yaml, checks every file's
sha256 (`Protocol.verify`, paths relative to the yaml), and returns a
`HeldoutPlan` of immutable `Fold(training, evaluated)`.

- `split: sessions` (default, also when the block is missing): one fold,
  train on the training session, evaluate on all others in protocol order.
- `split: kfold`: `k` row folds inside one session, from
  `default_rng(seed).permutation(n)` and `np.array_split`, fixed at load time
  (`HeldoutSet.rows` are the row indices). The plan label reads
  "k-fold within session <id> (single session; weaker than held-out
  sessions)" and travels with the results (`HeldoutResults.label`).
- `run(plan, models, evaluate)` -> `{model: {fold: {set: metrics}}}`; each
  metrics dict carries the set's `role`.
- `select(results, metric, role="validation")` picks the model with the lowest
  mean metric over sets of that role and raises `ValueError` for
  `role="confirmation"`.
- `posture_strata(q, q_train, tol, margin)` classifies postures as
  repeated / new / out_of_range relative to the training postures.

### protocol.yaml

The block is optional and ignored by core's `Protocol.load`:

```yaml
heldout:
  split: sessions            # or kfold
  selection_role: validation # confirmation is rejected
  kfold:                     # required iff split: kfold
    session: 2021-11-30-1544
    k: 5
    seed: 0
  strata:                    # optional
    joints: [torso_lift_joint, arm_1_joint]
    repeated_tol: 0.01
    out_of_range_margin: 0.05
```

Rules: roles are `training`, `validation`, `confirmation`; `sessions` needs
exactly one training session; `kfold` needs a protocol with exactly one
session; unknown keys are errors.

## TIAGo port

`heldout_protocol.py` derives `SETS` from the plan (same order as before) and
re-exports `posture_strata`, `component_errors`, `fit`; it checks at import
that the config's `source_file` is the plan's training file.
`calibration_truth.py` builds a `SimulationStudy` and keeps `run_case`,
`simulate`, `make_truth`, `postures`, `TRAINING` as thin wrappers. Printed
output of `python calibration_truth.py [--methods]` and
`python heldout_protocol.py` is byte-identical to before.

## Limits and decision points

- **Gauge.** Base and tool frames absorb some truth parameters, so z-scores
  are computed only for the names a fit keeps and only when the fit has the
  truth's model class (`z_params`). No z-scores were added for TIAGo
  `full_params`.
- **k-fold has no confirmation fold.** A single-session k-fold holds nothing
  back for a final check; selection and reporting use the same rows. Treat it
  as development evidence only. A study that needs a confirmation number must
  carry a second session. Decision point for #99: whether to reserve a last
  fold, which this harness does not do by default.
- **Tolerance.** `full_params` rows keep the 0.2 mm tolerance (pivot ties
  differ by platform, figaroh-plus#113); no tolerance was widened.
- **Core data contract.** The `heldout:` block is read by this repository
  only. It should be added to core's data-contract document so other
  protocol consumers know the key exists.

## Reproduction

```bash
cd figaroh-examples
MPLBACKEND=Agg python -m pytest tests/test_calibration_harness_*.py \
    tests/test_tiago_calibration_truth.py tests/test_tiago_heldout_protocol.py \
    tests/test_tiago_reference_run.py tests/test_tiago_mocap_baseline.py -q
cd examples/tiago
MPLBACKEND=Agg python calibration_truth.py > truth.txt
MPLBACKEND=Agg python heldout_protocol.py > heldout.txt
```

## Adapter sketches

- **#97 (another geometric robot).** Subclass nothing: write
  `sample_truth`, `predict` (core `calc_updated_fkm`), `fit` (return a
  `FitResult` with `params`, `std`) and `to_dataset`; list the held-out
  postures and use `gaussian_mm`.
- **#98 (input noise).** Set `SimulationStudy.simulate`: perturb the encoder
  postures, solve IK for the measurement, return `(measurements,
  noisy_postures)`; `to_dataset` writes both. Everything downstream is
  unchanged.
- **#99 (anisotropic noise, k-fold).** `noise=anisotropic([sx, sy, sz])` and a
  protocol.yaml with `split: kfold`; build the models/`evaluate` pair for
  `heldout.run` and report the plan label with the numbers.

## TALOS draw spread (phase 0 check)

`run_calibration_real_data.two_chain_metrics(robot, random_seed=0)` runs the
two-chain fit with `calib_config["random_seed"]` set before `initialize()`
and returns the shared torso axes, `n_dx` and each chain's held-out
`z_rmse_mm`. `tests/test_talos_two_chain_draw_invariance.py` (marked slow, ~2 min; `validate.py` runs it only with `--full-tests`). The measurements below were taken once with more seeds; the test keeps a subset:

- (a) **Global RNG state no longer matters.** `pin.seed(s)` +
  `np.random.seed(s)` for s in 0, 1, 7, 42 gave identical (the test checks 0 and 42) shared axes and
  identical z (difference below 1e-9). Core seeds the identifiability draw
  from `random_seed` (default 0), so the 2026-10-04 audit's dependence on
  Pinocchio's RNG state is gone.
- (b) **The draw itself still matters.** `random_seed` 0-4 (the test checks 0-2, which already spans 1.46 mm on the left):

| random_seed | shared torso axes | n_dx | left held-out z (mm) | right held-out z (mm) |
|---|---|---|---|---|
| 0 | 2 (phix_2, px_2) | 105 | 4.931 | 8.093 |
| 1 | 4 | 103 | 6.395 | 7.759 |
| 2 | 4 | 103 | 5.192 | 8.831 |
| 3 | 2 | 105 | 4.940 | 8.087 |
| 4 | 5 | 102 | 5.759 | 7.324 |

Left spread 1.46 mm, right spread 1.51 mm, against a 0.1 mm bar, so (b) is
`xfail(strict=True)`: it fails loudly once the selection becomes stable. The
shared-axis set changes with the draw, which is the mechanism the audit
named; fixing the seed made it reproducible, not stable. Tracked in figaroh-plus#204.
