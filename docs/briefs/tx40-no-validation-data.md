# Experiment brief: Staubli TX40 / recorded logs / dynamic identification without validation data

Instance of the [experiment brief template](../experiment-brief-template.md),
written for a case where one input is missing: **there is no independent
validation recording**. The code handles this case (it falls back to the
identification data with a warning), and the brief shows which claims that
makes unsupported. Items that the repository does not say are marked
`Unknown - action needed`, as the template asks.

## Outcome and scope

- Application and quantity to improve: joint torque prediction (N·m) of the
  TX40 arm model from a recorded motion.
- Task: dynamic identification.
- Supported scope and explicitly excluded effects/tasks: execution of the
  fit (finite, consistent outputs) and the training-set residuals. Excluded:
  any statement about prediction on other motions, physical consistency of
  the parameters, URDF export, calibration.
- Core/examples revisions and linked issue: core `779e830`, examples
  `b167298`. No accepted-evidence issue exists for TX40; this brief
  documents the limit.
- Acceptance criteria and their justification: execution scope only
  (`--verify`, default). A prediction criterion cannot be written, because a
  per-joint limit needs held-out data (see the
  [acceptance policy](../acceptance-policy.md)) and none is set or guessed.

## Available model and data

| Input | Source / revision / hash | Units / frame / order | Measured, commanded, derived or simulated | Missing information |
| --- | --- | --- | --- | --- |
| Nominal URDF and geometry | `examples/staubli_tx40/urdf/tx40_mdh_modified.urdf` | m, kg; `joint_1`..`joint_6` | Nominal | Provenance of the inertials |
| Joint state and timestamps | `data/pos_read_data.csv`, 45000 rows, columns `Joint_1..Joint_6` | The loader stores them as "motor encoder positions" and applies coupling and offsets; unit not written down | Measured | Unknown - action needed: unit, clock (the loader builds `t` from a configured `ts`) |
| Effort or current/conversion | `data/curr_data.csv`, 45000 rows | Motor currents; `reduction_ratios` in the config convert to joint torque | Measured | Unknown - action needed: current unit, torque constants, sign convention |
| Pose, marker or contact observations | Not applicable | | | |
| Payload / operating conditions | Not recorded | | | Unknown - action needed |
| Independent validation recording | **None** (`validation_data_file` not set) | | | The missing input of this brief |

- Data/assets access, license and redistribution policy: shipped files;
  origin listed as unspecified in the [data inventory](../data-inventory.md).
- Clock synchronization and observed sampling intervals: Unknown - action
  needed. The config lists `sampling_frequency: 5000.0` for the filter; the
  recording's real rate is not documented.
- Sensor calibration, saturation/dropout and conversion uncertainty:
  Unknown - action needed.
- Ground truth available? No.

## Model and observability

| Parameter block | Estimate / fix / exclude | Physical reason | Observation coverage / ambiguity | Prior or constraint |
| --- | --- | --- | --- | --- |
| Link inertials | Estimate as 58 base combinations | Dynamic model | Condition 966.9 (moderately conditioned); `Ixz_joint_4` 617% relative sd | None (no physical stage run) |
| Friction, actuator inertia, joint offsets | Estimate | On the real arm | Present in the base set | None |
| Wrist coupling joints 4-5 | Fixed model terms (`has_coupled_wrist`) | TX40 coupling | | Config values |

- Dynamic active joints, fixed/free base, `nq`/`nv` and effort convention:
  six active joints, fixed base, joint torque from motor current.
- Geometric measured components, chain, frame anchor and gauge: not
  applicable.
- Rank/singular-value analysis, scaling and tolerance: 58 base parameters
  from 44960 samples after border removal.
- Expected identifiable combinations vs unobservable individual parameters:
  base combinations only.

## Method selection and frozen protocol

| Baseline / candidate | Objective and constraints | Implementation / supported or research | Extras, weights and priors | Initialization / budget / success rule |
| --- | --- | --- | --- | --- |
| Nominal model | none | Reference | | Residual reported |
| OLS base fit refined by WLS (default for TX40) | Weighted least squares (Gautier 1997) | Supported | `wls: true` | Direct solve |

- Why these methods answer the task: they fit the recording; they cannot
  show that the fit generalises.
- Same-input comparison policy and intentional differences: nominal and
  fitted models are scored on the same samples.
- Development/tuning procedure; final-validation data are excluded: no data
  is held out, so every choice (filter, joints, WLS) is made on the data
  that is then scored.
- How failed solves, fallbacks and selected output stages are recorded: the
  stage line reads "validation fallback (training data, not held-out)"; the
  physical and export stages are `NOT_EVALUATED`.

## Acquisition / experiment design

- Existing-data coverage and missing excitation: one 45000-sample recording
  of unknown trajectory design.
- Planned poses/trajectories and selection rationale: none. To support a
  prediction claim, record a second motion that differs from the first (other
  trajectory family, other speeds) and set `validation_data_file`.
- Robot/controller/sensor constraints and pre-execution checks: joint
  torque limits in the config are 20 / 20 / 8 / 3 / 3 / 1.5 N·m; check them
  before any new run.
- Pilot acquisition checks and final recording procedure: Unknown - action
  needed.
- Independent validation motion/postures/conditions: none yet; see above.
- If no new recording is possible, limits on the eventual claim: the claim
  is "the model fits the recorded motion with this residual", nothing more.

## Processing and partitions

| Step | Method / settings | Estimated from which partition? | Trim / index mapping / diagnostics |
| --- | --- | --- | --- |
| Synchronization / resampling | Time axis built from a configured `ts` | Whole file | No recorded clock |
| Units / signs / frames | Coupling of joints 4-5 and offsets applied by the TX40 tools | Config constants | Unverified |
| Outlier handling / filtering | Butterworth 4th order, cutoff 100 Hz at 5000 Hz | Fixed | |
| Velocity / acceleration derivation | Differentiation of positions | Fixed | |
| Decimation / regressor selection | Border effects removed (44960 of 45000 samples remain) | | |

- Raw immutable inputs and output location: `data/*.csv`;
  `results/runs/<asset>/identification/<timestamp>/`.
- Training / development / final validation files or index ranges: all
  samples are training; development and final validation are empty.
- Temporal guard interval and filter/derivative support rationale: not
  applicable; there is no split.
- Leakage prevention and actual held-out consumption check: the fallback is
  the leak, by construction. The code logs the warning below and marks the
  result `identification_data_fallback`.

## Results and validation record

Commands, headless:

```bash
cd examples/staubli_tx40
MPLBACKEND=Agg python identification.py --no-html-report        # execution scope (default)
MPLBACKEND=Agg python identification.py --no-html-report \
    --verification-scope prediction --acceptance-profile limits.json
```

`limits.json` contains the limit you would declare, for example
`{"validation_rmse:joint_1": {"threshold": 1.0, "comparison": "max"}}`; the
value here is a placeholder for the experiment, not a recommendation. Note:
`--no-archive` skips only the provenance archive and the `index.jsonl` line;
the HTML report (on by default) and the verdict are still written to the run
folder under `results/runs/`. Add `--no-html-report --no-verify` (both on
by default) for a run that writes nothing.

Obtained on the current heads (no accepted-evidence numbers exist for this
case):

| Item | Execution scope | Prediction scope |
|---|---|---|
| Exit status | 0 | 1 |
| Verdict | PASS (`finite_*`, `requested_validation_loaded`, `matching_torque_shapes`) | `NOT_EVALUATED` |
| `validation_rmse:joint_1` | not requested | `NOT_EVALUATED`: "Required metric is unavailable" |
| `independent_validation` | not requested | `NOT_EVALUATED`: "No independently loaded validation data" |
| Prediction acceptance | `NOT_EVALUATED` | `NOT_EVALUATED` |
| Physical model / export | `NOT_EVALUATED` | `NOT_EVALUATED` |

The warning printed, and logged by `figaroh.identification`:

```text
No separate validation data available (validation_data_file not configured or failed to load);
falling back to identification data for validation metrics. These results are NOT an independent
generalization test.
```

- Nominal vs fitted per-joint/component RMSE, bias and residual figures:
  training residual RMSE 2.99 N·m overall (`joint_1` 3.05, `joint_2` 3.85,
  `joint_3` 2.31, `joint_4` 0.89, `joint_5` 4.51, `joint_6` 1.78); max
  residual up to 16.9 N·m on `joint_2`. Correlation 0.988.
- Training vs held-out metrics and correct physical units: there is no
  held-out metric. The fallback "validation" on the identification set gives
  nominal 4.70, identified 4.05 N·m (13.9%; `joint_5` R^2 0.69). It is
  higher than the training RMSE of 2.99 because the two are not computed on
  the same signal: the fit decimates (`scipy.signal.decimate`, which
  low-pass filters) the torques and regressor, while the fallback scores the
  full-rate, unfiltered recording, so its residual keeps the measurement
  noise the filter removed. Neither number is a generalisation error.
- Solver termination / input correctness / physical and gauge verdicts: fit
  ok; physical not evaluated.
- Parameter sensitivity and limits on individual-parameter interpretation:
  `Ixz_joint_4` 617%, `off_joint_6` 257%, `my_joint_4 + mz_joint_5` 190%.
- Validation level: **training fallback**.
- Selected export stage, output model and reload FK/effort parity: none; no
  URDF is exported.
- Task thresholds, computed/skipped checks and pass/fail interpretation:
  computed, execution only; the prediction checks are skipped as
  `NOT_EVALUATED`, which exits nonzero.
- Commands, dependency versions, paired revisions, hashes and artifact paths:
  see this folder's [README](README.md).

## What the brief forces you to conclude

| Claim | Supported? | Why |
|---|---|---|
| The fit ran and produced finite parameters | Yes | Execution checks pass |
| The model reproduces the recorded torques with about 3 N·m RMS | Yes, on this recording | Training residual |
| The model predicts torques on a new motion | **No** | No held-out data; the fallback is the training set |
| The identified model beats the nominal model in general | **No** | 13.9% is measured on the fitted samples only |
| The parameters are physically consistent / can be exported | **No** | Physical and export stages not evaluated |
| Individual masses, inertias or friction values are known | **No** | Base combinations only; several above 100% relative sd |
| The unit is the one the URDF describes | **No** | Provenance of the data is unspecified |

## Unsupported assumptions and limits

- Everything above marked **No**, plus the assumptions the data does not let
  us check: the sampling interval, the unit of the position and current
  channels, and the current-to-torque conversion.
- Any tuning (filters, WLS, joint choice) overfits the only recording there is.
- `NOT_EVALUATED` is not a pass: a script that ignores the nonzero exit code
  would report success on an unsupported claim.

## Integration and review

- Robot README reproduction commands and required assets: `examples/staubli_tx40`
  README; shipped CSVs and URDF.
- Core changes required, with linked issue/PR and exact dependency: none;
  the fallback is in core `BaseIdentification` and the report renderers.
- Regression fixture and automated validation coverage: `validate.py` runs
  the TX40 identification at execution scope.
- Current blockers, unresolved decisions and next bounded action: record a
  second, different motion, declare per-joint limits before looking at it
  and set `validation_data_file`; fill the `Unknown - action needed` cells.
- Review status: draft (no accepted evidence).
