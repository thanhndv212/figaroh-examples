# Experiment brief: <robot / dataset / task>

Copy into `examples/<robot>/EXPERIMENT.md`. This is a human-readable planning
record, **not a configuration schema consumed by FIGAROH**. Fill unknowns with
`Unknown — action needed` and inapplicable items with a reason. Link evidence
instead of copying a changing results table into multiple documents.

## Outcome and scope

- Application and quantity to improve:
- Task: dynamic identification / geometric calibration / experiment design:
- Supported scope and explicitly excluded effects/tasks:
- Core/examples revisions and linked issue:
- Acceptance criteria and their application/measurement justification:

## Available model and data

| Input | Source / revision / hash | Units / frame / order | Measured, commanded, derived or simulated | Missing information |
| --- | --- | --- | --- | --- |
| Nominal URDF and geometry | | | | |
| Joint state and timestamps | | | | |
| Effort or current/conversion | | | | |
| Pose, marker or contact observations | | | | |
| Payload / operating conditions | | | | |
| Independent validation recording | | | | |

- Data/assets access, license and redistribution policy:
- Clock synchronization and observed sampling intervals:
- Sensor calibration, saturation/dropout and conversion uncertainty:
- Ground truth available? If synthetic, specify model, derivatives and seeds:

## Model and observability

| Parameter block | Estimate / fix / exclude | Physical reason | Observation coverage / ambiguity | Prior or constraint |
| --- | --- | --- | --- | --- |
| <inertias, friction, offsets, transforms, ...> | | | | |

- Dynamic active joints, fixed/free base, `nq`/`nv` and effort convention:
- Geometric measured components, chain, frame anchor and gauge:
- Rank/singular-value analysis, scaling and tolerance:
- Expected identifiable combinations vs unobservable individual parameters:

## Method selection and frozen protocol

| Baseline / candidate | Objective and constraints | Implementation / supported or research | Extras, weights and priors | Initialization / budget / success rule |
| --- | --- | --- | --- | --- |
| | | | | |

- Why these methods answer the task:
- Same-input comparison policy and intentional differences:
- Development/tuning procedure; final-validation data are excluded:
- How failed solves, fallbacks and selected output stages are recorded:

## Acquisition / experiment design

- Existing-data coverage and missing excitation:
- Planned poses/trajectories and selection rationale:
- Robot/controller/sensor constraints and pre-execution checks:
- Pilot acquisition checks and final recording procedure:
- Independent validation motion/postures/conditions:
- If no new recording is possible, limits on the eventual claim:

## Processing and partitions

| Step | Method / settings | Estimated from which partition? | Trim / index mapping / diagnostics |
| --- | --- | --- | --- |
| Synchronization / resampling | | | |
| Units / signs / frames | | | |
| Outlier handling / filtering | | | |
| Velocity / acceleration derivation | | | |
| Decimation / regressor selection | | | |

- Raw immutable inputs and output location:
- Training / development / final validation files or index ranges:
- Temporal guard interval and filter/derivative support rationale:
- Leakage prevention and actual held-out consumption check:

## Results and validation record

Link the report containing methodology, procedure, model fitting results,
parameter analysis, held-out validation and limitations. Include:

- Nominal vs fitted per-joint/component RMSE, bias and residual figures:
- Training vs held-out metrics and correct physical units:
- Solver termination / input correctness / physical and gauge verdicts:
- Parameter sensitivity and limits on individual-parameter interpretation:
- Validation level: unavailable / training fallback / temporal holdout /
  separate simulation / independent real recording:
- Selected export stage, output model and reload FK/effort parity:
- Task thresholds, computed/skipped checks and pass/fail interpretation:
- Commands, dependency versions, paired revisions, hashes and artifact paths:

## Integration and review

- Robot README reproduction commands and required assets:
- Core changes required, with linked issue/PR and exact dependency:
- Regression fixture and automated validation coverage:
- Current blockers, unresolved decisions and next bounded action:
- Review status (draft / agreed protocol / executed / accepted evidence):
