# Experiment brief: TIAGo / Qualisys static postures / geometric calibration (C4)

Instance of the [experiment brief template](../experiment-brief-template.md).
Evidence: [calibration reference workflow](../development/tiago-calibration-reference.md),
[held-out protocol](../development/tiago-mocap-heldout-protocol.md) and
[export check](../development/tiago-calibration-export.md).

## Outcome and scope

- Application and quantity to improve: Cartesian position error (mm) of the
  hand marker predicted by forward kinematics, and a corrected arm URDF.
- Task: geometric calibration.
- Supported scope and explicitly excluded effects/tasks: arm joint offsets
  (level `joint_offset`), a 6D mocap base frame and a 3D marker point,
  static postures, one marker, position only. Excluded: link-length and
  orientation parameters (`full_params`), orientation measurements,
  backlash and deflection, deployment of the PAL file.
- Core/examples revisions and linked issue: examples#29 (C4), #67 (data),
  #27 (protocol). Accepted on core `c80b5fc` + examples `88e2fc2`; re-run on
  core `779e830` + examples `b167298`.
- Acceptance criteria and their justification: held-out marker RMSE 3.8-4.2
  mm across the validation and confirmation sessions, and export parity
  <= 2.1e-12 m. The RMSE band is evidence, not a limit: no prediction
  acceptance profile is set for TIAGo, so scope `prediction` stays
  `NOT_EVALUATED`.

## Available model and data

| Input | Source / revision / hash | Units / frame / order | Measured, commanded, derived or simulated | Missing information |
| --- | --- | --- | --- | --- |
| Nominal URDF and geometry | `examples/tiago/urdf/tiago_48_schunk.urdf` (sha256 prefix `e4412357e3ea`) | m, rad | Nominal | End effector of the sessions was not recorded; only the tip offset depends on this choice |
| Joint state and timestamps | Eight joints per posture in each session file, averaged over the plateau | m (torso), rad | Measured (encoders) | Clock lags between mocap and robot (3.9 s and 2.6 s) corrected by cross-correlation |
| Effort or current/conversion | Not applicable | | | |
| Pose, marker or contact observations | `data/calibration/mocap/qualisys_2021-11-30_static_postures.csv` (training, 37 postures, `b6c0051e20c6a077`) and three held-out sessions | m, in the Qualisys `base_frame` rigid body | Measured; four virtual points of one rigid body, only marker 1 used | Points 2-4 unused (figaroh-plus#119) |
| Payload / operating conditions | Static, no payload | | | Not recorded |
| Independent validation recording | `qualisys_2021-11-26_...` (62, `7c986df711757c4d`); confirmation `2021-11-30-1403` (63) and `-1504` (59) | same | Measured | The sessions largely replay training postures |

- Data/assets access, license and redistribution policy: shipped CSVs; the
  original ROS bags are not distributed.
- Clock synchronization and observed sampling intervals: static plateaus of
  at least 2 s with every joint within 1 mrad over 0.5 s; markers averaged
  on the same physical interval after the lag correction.
- Sensor calibration, saturation/dropout and conversion uncertainty: marker
  standard deviation 0.2 mm (median) per plateau; `marker_std_mm` is in the
  file. The mocap-to-robot frame is unknown before the fit.
- Ground truth available? No; the nominal model gives 419.39 mm on the
  validation session because the mocap frame is unregistered.

## Model and observability

| Parameter block | Estimate / fix / exclude | Physical reason | Observation coverage / ambiguity | Prior or constraint |
| --- | --- | --- | --- | --- |
| Joint offsets `arm_2`-`arm_6` | Estimate | Encoder zero errors | Position of one marker | None (no regularisation) |
| `offsetPZ_torso_lift_joint`, `offsetRZ_arm_1_joint` | Absorbed by the base frame | Vertical axes: indistinguishable from the 6D base frame | Gauge | Dropped by core (figaroh-plus#102) |
| Base frame `base_p{x,y,z}`, `base_phi{x,y,z}` | Estimate | Registration of mocap to the robot base | | Metrology frame, not geometry |
| Marker point `pEE{x,y,z}_1` | Estimate | Marker mounting on the hand | | Metrology frame |
| Link lengths, orientations | Fix | Level `joint_offset` | | `full_params` is a separate comparison |

- Dynamic active joints, fixed/free base, `nq`/`nv` and effort convention:
  not applicable.
- Geometric measured components, chain, frame anchor and gauge: marker 1,
  position xyz only; chain from the base frame to `wrist_ft_tool_link`; the
  gauge is the estimated 6D base and 3D marker point.
- Rank/singular-value analysis, scaling and tolerance: 14 parameters,
  full rank; condition 62.2 in this run. Correlated pairs: `base_px` / `base_phiy`
  (-0.95), `offsetRZ_arm_5_joint` / `pEEy_1` (+0.81).
- Expected identifiable combinations vs unobservable individual parameters:
  five joint offsets plus nine frame parameters; `arm_2`-`arm_4` and
  `arm_6` offsets are small against their standard errors.

## Method selection and frozen protocol

| Baseline / candidate | Objective and constraints | Implementation / supported or research | Extras, weights and priors | Initialization / budget / success rule |
| --- | --- | --- | --- | --- |
| Registration only | Fit base frame and marker point, no joint terms | Supported | | Compared in `heldout_protocol.py` |
| `joint_offset` (selected) | Nonlinear least squares of marker position | Supported | No regularisation | Solver `ftol` reached |
| `full_params` | As above with link parameters | Supported, comparison only | | Overfits a known truth at 37 postures |

- Why these methods answer the task: the aim is predicting held-out marker
  positions; the protocol compares levels on the same data.
- Same-input comparison policy and intentional differences: same training
  session and gauge for all levels.
- Development/tuning procedure; final-validation data are excluded: the
  confirmation sessions are not in any config and must not be tuned on.
- How failed solves, fallbacks and selected output stages are recorded:
  `stages.json`, solver status in `reproduction.json`; `selected_stage: fit`.

## Acquisition / experiment design

- Existing-data coverage and missing excitation: of each held-out set's
  59-63 postures, 35-37 are training postures, 16-17 are new but in range,
  8-9 are outside the training ranges.
- Planned poses/trajectories and selection rationale: none new; optimal
  configurations exist (`data/calibration/optimal_configurations`) but were
  not executed for this reference.
- Robot/controller/sensor constraints and pre-execution checks: not
  applicable (recorded data).
- Pilot acquisition checks and final recording procedure: not applicable.
- Independent validation motion/postures/conditions: another day
  (2021-11-26) plus two later sessions on 2021-11-30.
- If no new recording is possible, limits on the eventual claim: the claim
  is the marker error on these postures, with the stratified numbers below.

## Processing and partitions

| Step | Method / settings | Estimated from which partition? | Trim / index mapping / diagnostics |
| --- | --- | --- | --- |
| Synchronization / resampling | Lag correction by cross-correlation, plateau averaging | Each session separately | Window [start + 0.5 s, end - 0.3 s] |
| Units / signs / frames | Markers in `base_frame` body, metres | n/a | |
| Outlier handling / filtering | Exclude samples with position error > 50 mm | Training | 0 of 37 excluded |
| Velocity / acceleration derivation | Not applicable | | |
| Decimation / regressor selection | None | | |

- Raw immutable inputs and output location: `data/calibration/mocap/*`;
  run directory under `results/runs/TIAGO-48/calibration/`.
- Training / development / final validation files or index ranges: training
  2021-11-30 / none / validation 2021-11-26 plus confirmation sessions.
- Temporal guard interval and filter/derivative support rationale: separate
  sessions.
- Leakage prevention and actual held-out consumption check: the command
  checks sha256 and roles against `protocol.yaml` before fitting.

## Results and validation record

Command (clean checkouts of both repositories; the run takes under a minute):

```bash
cd examples/tiago
MPLBACKEND=Agg python reference_run.py --asset-id TIAGO-48 --root /path/to/runs
```

Marker-1 position error (mm), obtained on the current heads:

| Role | Session | n | RMSE obtained | RMSE accepted | max |
|---|---|---|---|---|---|
| training | 2021-11-30 | 37 | 2.88 | 2.88 | 5.55 |
| validation | 2021-11-26 | 62 | 4.23 | 4.23 | 11.95 |
| confirmation | 2021-11-30-1403 | 63 | 4.09 | 4.09 | 12.24 |
| confirmation | 2021-11-30-1504 | 59 | 3.83 | 3.83 | 12.18 |

- Nominal vs fitted per-joint/component RMSE, bias and residual figures:
  validation x / y / z RMSE 2.37 / 2.59 / 2.35 mm; nominal 419.39 mm.
- Training vs held-out metrics and correct physical units: 2.88 mm training
  against 3.83-4.23 mm held out.
- Solver termination / input correctness / physical and gauge verdicts:
  converged (`ftol`); gauge frames listed above; physical model
  `NOT_EVALUATED` (not an identification task).
- Parameter sensitivity and limits on individual-parameter interpretation:
  `arm_5` offset -49.8 mrad +- 10.1 is the only offset well above its
  standard error; the others are within 1-2 sigma.
- Validation level: independent real recording (other days), partly
  replaying training postures.
- Selected export stage, output model and reload FK/effort parity:
  `calibrated.urdf` and both PAL files reload to the fit within **2.07e-12 m**
  on every session (accepted <= 2.1e-12; gate 1e-9); 5 joint origins changed.
- Task thresholds, computed/skipped checks and pass/fail interpretation:
  execution PASS; prediction `NOT_EVALUATED` (no profile).
- Commands, dependency versions, paired revisions, hashes and artifact paths:
  see this folder's [README](README.md); hashes in `reproduction.json`.

Difference from the accepted evidence: none beyond rounding. The validation
session reads 4.23 mm, which is 0.03 mm above the "3.8-4.2" band quoted in
the issue; the reference document lists the same 4.23 (the band is rounded).

Error by posture stratum (reference document, unchanged): repeated postures
about 3 mm, new in-range 4.1-4.5 mm, out of range about 6.5 mm.

## Unsupported assumptions and limits

- The held-out error is an evidence band, not an accepted limit: no
  prediction profile exists and the prediction verdict is `NOT_EVALUATED`.
- The held-out sessions mostly replay training postures; error outside the
  training joint range is about 6.5 mm.
- Residual of 3-4 mm in every session is not geometric; the model has no
  backlash or deflection term.
- One marker, position only: orientation and the other three rigid-body
  points carry no result here.
- The base frame and marker point are metrology frames and are not written to
  the URDF or PAL file. The PAL semantics (additive deltas on the URDF origin
  xyz/rpy) are the maintainer's reading, not checked against a PAL robot
  ([export check](../development/tiago-calibration-export.md)). Individual
  joint offsets other than `arm_5` are not distinguishable from zero.
- Torso and `arm_1` offsets are absorbed by the base frame and cannot be
  claimed.
- Assumes the shipped Schunk URDF; the sessions' true end effector was not
  recorded.
- No hardware was commanded; nothing is deployed.

## Integration and review

- Robot README reproduction commands and required assets: the command above;
  shipped CSVs and URDF.
- Core changes required, with linked issue/PR and exact dependency:
  figaroh-plus#101, #102, #105; core 0.7.0.
- Regression fixture and automated validation coverage:
  `tests/test_tiago_reference_run.py` and `validate.py`.
- Current blockers, unresolved decisions and next bounded action: PAL
  mapping of joint offsets (#28); fitting several rigid-body points (#119).
- Review status: accepted evidence (examples#29).
