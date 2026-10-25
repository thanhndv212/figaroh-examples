# Calibration harness

Shared code for the calibration studies: a synthetic-truth simulation and a
held-out evaluation protocol. A robot example writes a small adapter, and the
harness handles drawing, noise, fitting loops, scoring and model selection.
Every study then follows the same rules.

| Module | Use it to |
|---|---|
| [`simulation.py`](simulation.py) | draw a known truth, simulate noisy measurements, fit them and score the fit against the truth |
| [`heldout.py`](heldout.py) | turn a frozen `protocol.yaml` into folds, run models on them, select a model and split postures into strata |
| [`protocol.py`](protocol.py) | parse the optional `heldout:` block of a `protocol.yaml` |

Design, rules and limits:
[docs/development/calibration-harness.md](../../docs/development/calibration-harness.md).
Reference adapter: [`examples/tiago/`](../tiago/) (`calibration_truth.py`,
`heldout_protocol.py`).

## Synthetic-truth study

A robot supplies a `SimulationStudy`:

```python
from examples.calibration_harness.simulation import (
    FitResult, SimulationStudy, gaussian_mm, run_grid, summarize,
)

study = SimulationStudy(
    sample_truth=sample_truth,  # (kind, rng) -> {name: value}
    predict=predict,            # (params, kind, postures, handle=None) -> (n, m) metres
    fit=fit,                    # (train_df, level, estimation, workdir) -> FitResult
    to_dataset=to_dataset,      # (measurements, postures) -> DataFrame in the study's layout
    train_postures=q_train,     # DataFrame
    heldout_postures={"validation": q_val},
    noise=gaussian_mm,          # or anisotropic([sx, sy, sz])
    z_params={"joint_offset": lambda name: name.startswith("offset")},
)

df = run_grid(
    study,
    truths=["joint_offset", "full_params"],
    fits={"offsets": ("joint_offset", None), "map": ("full_params", {"method": "map"})},
    seeds=range(5),
    noise=(0.5, 2.0),  # mm
)
table, z = summarize(df)
```

- Held-out error compares the fit's prediction with the **noise-free**
  truth's prediction on held-out postures. It never compares against noisy
  measurements.
- z-scores cover only the parameters named in `z_params`, and only when the
  fit uses the truth's model class. Base and tool frames absorb some truth
  parameters, and those parameters can't be recovered.
- The random streams are fixed: the truth uses `default_rng(seed)` and the
  noise uses `default_rng(10_000 + seed)`. Keep this order so that seeds
  reproduce.
- `train_residual` picks the columns of the training RMS (default: all).
  Use it when some columns are angles that wrap, as in the TX40's 6-DOF
  poses.
- When noise is not additive on the measurement (for example encoder noise
  propagated through IK), set `simulate` so that it returns
  `(measurements, postures_used)`.

## Held-out protocol

Add a `heldout:` block to the example's `protocol.yaml`. Core's
`Protocol.load` ignores the block:

```yaml
heldout:
  split: sessions            # train on the training session, evaluate on the rest
  selection_role: validation # selecting on confirmation raises
```

For a single session, use row folds instead. Results from these folds carry
the label "weaker than held-out sessions":

```yaml
heldout:
  split: kfold
  kfold: {session: <id>, k: 5, seed: 0}
```

Then:

```python
from examples.calibration_harness import heldout

plan = heldout.load_plan("data/protocol.yaml")  # checks every file's sha256
results = heldout.run(plan, models, evaluate)   # models: {name: fold -> fitted}
best, scores = heldout.select(results, "rmse_mm", role="validation")
print(results.label)
```

- Session roles are `training`, `validation` and `confirmation`. Confirmation
  sets are reported but never used for selection.
- `posture_strata(q, q_train)` labels each held-out posture as `repeated`,
  `new` or `out_of_range` relative to the training postures. Report the error
  for each stratum, because extrapolation usually dominates.
- A k-fold split has no confirmation set, so treat it as development
  evidence only.

## Tests

```bash
MPLBACKEND=Agg python -m pytest tests/test_calibration_harness_*.py -q
```

`tests/golden/calibration_harness_tiago.json` pins the TIAGo numbers. A
change to the harness must reproduce them, or it must say why it doesn't.
