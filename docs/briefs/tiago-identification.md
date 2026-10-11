# Experiment brief: TIAGo / 2021-07 arm recordings / dynamic identification (D7)

Instance of the [experiment brief template](../experiment-brief-template.md).
Evidence: [reference workflow](../development/identification-reference-workflow.md);
signal findings: [TIAGo data README](../../examples/tiago/data/README.md#dynamic-identification-data-identificationdynamic).
The brief covers the two cases that matter for a decision: `physical-fit`
(the estimate you would export) and `reject` (the exact reconstruction, which
is refused on this data).

## Outcome and scope

- Application and quantity to improve: arm effort prediction (N·m) for
  `arm_1`-`arm_4` and a lumped-body URDF with fitted link inertials.
- Task: dynamic identification.
- Supported scope and explicitly excluded effects/tasks: rigid-body
  inertials, friction, actuator inertia and offsets of the arm on one
  recorded path. Excluded: torso and wrist (`arm_5`-`arm_7`) effort, payload
  change, suspension and backlash models, deployment.
- Core/examples revisions and linked issue: examples#23, #68 (velocity,
  effort joints), #69 (independent recordings). Accepted on core `658fe99` +
  examples `663d642`; re-run on core `779e830` + examples `b167298`.
- Acceptance criteria and their justification: **execution scope only**. The
  maintainer set the bar at "finite, consistent outputs; physical stage
  accepted; export reloads". Held-out numbers are reported, not gated,
  because arm_1's effort scale is unidentifiable and no per-joint limit was
  justified from this hardware's noise.

## Available model and data

| Input | Source / revision / hash | Units / frame / order | Measured, commanded, derived or simulated | Missing information |
| --- | --- | --- | --- | --- |
| Nominal URDF and geometry | `examples/tiago/urdf/tiago_48_hey5.urdf` | kg, m; torso, `arm_1`..`arm_7`, Hey5 hand | CAD-derived | Hey5 mass chain disagrees with the F/T sensor (0.792 kg below the sensor); hand identified from logged channels |
| Joint state and timestamps | `data/identification/dynamic/tiago_position.csv` (sha256 `4297a60d434ee89f`) | rad / m, s; ~100 Hz, jitter 7.7-12.3 ms | Measured | Source bag not distributed |
| Effort | `tiago_effort.csv` (`12c635987705efad`) | raw values times `reduction_ratio x kmotor` (config `drives` table), torso `+ 9.81 x subtree mass` | Measured, converted | Constants are documented assumptions, not checked against a torque reference; arm_1 constant unsourced |
| Pose, marker or contact observations | Wrist F/T `tiago_wrist_ft.csv` | N, N·m, sensor frame | Measured | Used only by `payload_check.py` |
| Payload / operating conditions | Training and slow runs without added load; `calibration_weight` has added load | | | Payload mass of the added load derived, 13.5% off the F/T sensor |
| Independent validation recording | `data/identification/calibration_slow/` (14163 rows, 2021-07-01 12:57) | same | Measured | Same path as training at half speed |

- Data/assets access, license and redistribution policy: shipped CSVs;
  keep the Thanh Nguyen / CNRS / Toward attribution; original bags are not
  distributed.
- Clock synchronization and observed sampling intervals: recorded clock `t`,
  identical in the three channel files; median step 9.997 ms.
- Sensor calibration, saturation/dropout and conversion uncertainty: wrist
  efforts are exactly zero on 88-90% of samples (quantised to 0.001); the
  torso force is ~99% the URDF's own `m g`.
- Ground truth available? No.

## Model and observability

| Parameter block | Estimate / fix / exclude | Physical reason | Observation coverage / ambiguity | Prior or constraint |
| --- | --- | --- | --- | --- |
| Link inertials | Estimate as 57 base combinations | Needed for the URDF | Motion of torso and wrist enters the regressor; only `arm_1`-`arm_4` effort constrains it | `physical_fit`: pseudo-inertia constraints |
| Friction, actuator inertia, joint offsets | Estimate | Present on real hardware | Weak for wrist joints | None |
| Torso and `arm_5`-`arm_7` effort | Exclude from the fit and the score | Quantised or unsourced signal | | |

- Dynamic active joints, fixed/free base, `nq`/`nv` and effort convention:
  torso and arm joints active, fixed base; fitted effort rows
  `arm_1`-`arm_4` (`torque_fit_joints`).
- Geometric measured components, chain, frame anchor and gauge: not
  applicable.
- Rank/singular-value analysis, scaling and tolerance: 57 base parameters,
  condition 1303.8 ("ill-conditioned" in the report), 5870 samples, 2348
  fitted rows after decimation. The base-parameter uncertainty table lists
  `my_arm_2_joint` at 2811% and several wrist combinations above 200%.
- Expected identifiable combinations vs unobservable individual parameters:
  combinations only. Individual `arm_5`-`arm_7` inertials and masses are not
  claimed.

## Method selection and frozen protocol

| Baseline / candidate | Objective and constraints | Implementation / supported or research | Extras, weights and priors | Initialization / budget / success rule |
| --- | --- | --- | --- | --- |
| Nominal model | none | Reference | | Held-out RMSE reported |
| Base fit (OLS) | min ‖tau - W phi‖ | Supported | `wls: false` | Direct solve |
| `reconstruction` (requested in case `reject`) | Exact standard-parameter reconstruction of the base fit | Supported | | Accept only if every link is feasible |
| `physical_fit` (selected) | Base fit under per-link pseudo-inertia constraints | Supported, cvxopt | | Accept if the solver is optimal and all links feasible |

- Why these methods answer the task: the exported URDF must be physically
  consistent; the reject case documents that the exact route is not.
- Same-input comparison policy and intentional differences: both cases use
  the same training window (rows 921-6791 of `dynamic/`, decimated) and the
  same held-out file; only `tasks.identification.select_stage` differs
  (overlays `config/tiago_reference_physical_fit.yaml`,
  `tiago_reference_reject.yaml`).
- Development/tuning procedure; final-validation data are excluded:
  `calibration_slow` was inspected in the source audit and in the velocity
  choice (#68), so it is **not** an unseen test set (see limits).
- How failed solves, fallbacks and selected output stages are recorded:
  `stages.json`, `verdict.json`; `selected_stage: none` when rejected, with
  `fit_parameters.csv` kept and no `parameters.csv`.

## Acquisition / experiment design

- Existing-data coverage and missing excitation: excitation window 9.21-67.90 s
  (RMS velocity 0.159 rad/s inside, 0.017 / 0.006 before / after). Wrist
  joints are barely excited in effort.
- Planned poses/trajectories and selection rationale: none new; recorded
  data only.
- Robot/controller/sensor constraints and pre-execution checks: not
  applicable (no commanded motion).
- Pilot acquisition checks and final recording procedure: not applicable.
- Independent validation motion/postures/conditions: `calibration_slow`, same
  path at about half speed.
- If no new recording is possible, limits on the eventual claim: held-out
  numbers describe slower replay of a path the model was fitted on.

## Processing and partitions

| Step | Method / settings | Estimated from which partition? | Trim / index mapping / diagnostics |
| --- | --- | --- | --- |
| Synchronization / resampling | Recorded clock, no resampling | n/a | |
| Units / signs / frames | `reduction_ratio x kmotor` per joint; torso gravity term added | Config constants | Assumptions, not verified |
| Outlier handling / filtering | Butterworth 4th order, 2 Hz cutoff at 100 Hz | Fixed | |
| Velocity / acceleration derivation | Velocity from filtered positions (#68), acceleration differentiated | Fixed | Logged velocity residual 1.4-1.6e-4 recorded as provenance |
| Decimation / regressor selection | Decimated; `arm_1`-`arm_4` rows | Fixed | 2348 fitted rows |

- Raw immutable inputs and output location: `data/identification/*`; run
  directory under `results/runs/TIAGO-48/identification/`.
- Training / development / final validation files or index ranges:
  `dynamic/` rows [921, 6791) / the source audit used `calibration_slow` /
  `calibration_slow` all rows.
- Temporal guard interval and filter/derivative support rationale: separate
  recordings; no overlap.
- Leakage prevention and actual held-out consumption check: file hashes in
  `protocol.yaml`; `training_disjoint_from_heldout: true`; the CLI refuses
  prediction acceptance on the frozen payload or training files.

## Results and validation record

Commands (`--case all` runs both in sequence; about 17 s and 14 s). Run TIAGo
cases sequentially.

```bash
cd examples/tiago
MPLBACKEND=Agg python identification_reference.py --case all --asset-id TIAGO-48 --root /path/to/runs
```

Held-out RMSE (N·m) on `calibration_slow`, case `physical-fit`:

| Joint | Train RMSE | Held-out (obtained) | Held-out (accepted) | Nominal | nRMSE |
|---|---|---|---|---|---|
| arm_1 | 1.040 | 1.303 | 1.303 | 3.187 | 0.418 |
| arm_2 | 1.148 | 2.539 | 2.539 | 7.838 | 0.420 |
| arm_3 | 0.713 | 1.446 | 1.446 | 3.122 | 0.366 |
| arm_4 | 0.588 | 1.226 | 1.226 | 2.858 | 0.257 |

Overall held-out 1.713 (nominal 4.730); `arm_2`-`arm_4` pooled 1.829.

Case `reject`: `reconstruction` requested; 43 of 48 links infeasible (lowest
pseudo-inertia eigenvalue -3.67; `arm_1`-`arm_7` among them), stage
`rejected`, `selected_stage: none`, verdict FAIL as expected, `export_urdf`
raises, the audit's only gap is `export`. Held-out numbers in
`reference.json` are the base fit's: 1.722 overall, 1.839 pooled.

- Nominal vs fitted per-joint/component RMSE, bias and residual figures:
  above and in `report.html`.
- Training vs held-out metrics and correct physical units: N·m; held-out is
  worse than training by 1.2-2.2 times per joint.
- Solver termination / input correctness / physical and gauge verdicts:
  `physical_fit` accepted, "cvxopt: optimal; all links feasible"; 58 links,
  none infeasible (smallest eigenvalue -1.3e-9, at the constraint boundary).
- Parameter sensitivity and limits on individual-parameter interpretation:
  see the uncertainty table above.
- Validation level: independent real recording, but with prior inspection
  (not blind).
- Selected export stage, output model and reload FK/effort parity:
  `physical_fit`; `identified.urdf` reloads to the reference model by RNEA
  over 200 states within **1.07e-9 N·m** (tolerance 1e-6). Accepted
  evidence: 6.1e-10.
- Task thresholds, computed/skipped checks and pass/fail interpretation:
  execution scope PASS; physical PASS; export PASS; held-out reported, not
  gated.
- Commands, dependency versions, paired revisions, hashes and artifact paths:
  see this folder's [README](README.md); hashes in `reference.json`.

Difference from the accepted evidence: held-out numbers, base-parameter
count, selected stages and verdicts are identical. Export parity is 1.07e-9
against the quoted 6.1e-10; both are far inside the 1e-6 gate and the
workflow notes last-bit solver differences between runs and platforms.
After core#174 (#163) the `physical` stage here is `ok` for `physical_fit`
and `failed` only for the requested `reconstruction`; no stage reports
`not_run`. `projected` is not selectable.

## Unsupported assumptions and limits

- No prediction claim: only the execution scope is gated. The held-out
  numbers are evidence of a 63% error reduction against the nominal model on
  one path at half speed, not an acceptance result.
- `calibration_slow` is not an unseen test: it informed the velocity and
  effort-joint decisions. Do not tune on it and then claim independent
  acceptance.
- Effort scale is an assumption: `reduction_ratio x kmotor` is documented, not
  verified. arm_1's scale cannot be identified (#68); its parameters and
  absolute RMSE carry the unknown scale. Compare runs on `arm_2`-`arm_4`.
- `payload_check.py` finds the efforts 13.5% low against the F/T sensor
  (tolerance 10%): the conversion constants are therefore not validated.
- `arm_5`-`arm_7` and torso effort are neither fitted nor scored; their
  identified inertials are unconstrained by effort and are not interpretable.
- The identified `arm_7` body (0.243 kg in `physical_fit`) is a lump that
  includes the hand and the F/T sensor; the nominal lump is 0.934 kg. Do not
  read it as the mass of a physical part.
- Condition 1304 and wrist combinations with relative standard deviation
  over 200%: individual parameters are not claimed.
- Offline logs only; one robot unit; no deployment claim.

## Integration and review

- Robot README reproduction commands and required assets: the command
  above; shipped CSVs only (the original bags are not needed).
- Core changes required, with linked issue/PR and exact dependency: core
  0.7.0 (`physical_fit`, hard-reject export); cvxopt from `figaroh-dev`.
- Regression fixture and automated validation coverage: `validate.py` reference
  cases and the example tests.
- Current blockers, unresolved decisions and next bounded action: source and
  sign of the effort constants; a payload or torque reference would remove
  the largest assumption.
- Review status: accepted evidence at execution scope (examples#23).
