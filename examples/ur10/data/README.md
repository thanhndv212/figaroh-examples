# UR10 dynamic-identification data contract

These CSVs are historical simulation inputs, not hardware measurements or a
replayable inertial-ground-truth fixture. See the [dated signal audit](../../../docs/development/ur10-signal-audit-2026-10-02.md)
for the source-history evidence, corrections and validation results.

For a known truth, use the separate fixture in [`truth/`](truth/) (#21):
saved truth inertias and URDF, analytic q/dq/ddq, independently checked
effort, separate training and validation trajectories, seeded noise and a
frozen benchmark protocol. It is generated and checked by
`identification_truth.py`; see the [fixture report](../../../docs/development/ur10-dynamic-truth-fixture.md).

## Channels and units

The position files contain exactly `q0` through `q5`; effort files contain
exactly `tau1` through `tau6`. The loader selects columns by name, so column
reordering does not change the joint mapping. All values must be finite.

| Position | Effort | URDF joint | Assumed units |
|---|---|---|---|
| q0 | tau1 | shoulder_pan_joint | rad; joint-side Nm |
| q1 | tau2 | shoulder_lift_joint | rad; joint-side Nm |
| q2 | tau3 | elbow_joint | rad; joint-side Nm |
| q3 | tau4 | wrist_1_joint | rad; joint-side Nm |
| q4 | tau5 | wrist_2_joint | rad; joint-side Nm |
| q5 | tau6 | wrist_3_joint | rad; joint-side Nm |

Effort values retain their stored signs. No current-to-torque conversion,
gear ratio, sign flip or effort filtering occurs in this adapter. The
historical RNEA-generation claim supports this interpretation, but the
actual generator and parameter vector were not saved. Do not reinterpret
motor currents using this loader.

## Timing, differentiation and filtering

There are no recorded timestamps, velocities or accelerations. The default
unified identification config assumes **500 Hz**, giving `ts = 0.002 s`.
This is a configured assumption, not a verified source clock. The legacy
flat config uses `ts = 0.01 s`; switching configurations changes the assumed
clock and does not establish how the CSVs were generated.

The loader timestamps position row `i` at `i * ts` and uses that same `ts`
for the core finite-difference helper. It requires the resolved filter
sampling rate to equal `1/ts`; it performs no resampling. Use core FIGAROH
with the corrected tangent differentiation from PR #69 (devel commit
`0af819cc32c82c2560d5fc08926cb00a1dc39541`) or later.

Forward-difference velocity uses position rows `i` and `i+1` and is centered
at `(i+0.5)*ts`; acceleration is differentiated on these interval centers.
The retained historical output pairs these derivatives with position row
`i` and effort row `i`. The half-step offset is explicit, not corrected by
interpolation. Sample-count alignment does not prove physical time alignment.

The base pipeline filters supplied positions, velocities and accelerations
separately, after differentiation: median window 5, then zero-phase
Butterworth order 4, cutoff 50 Hz, sample rate 500 Hz with the default unified
config. There is no loader prefilter. Filter boundaries remain in the fit;
no additional edge exclusion is applied. Do not change rates or cutoff values
merely to improve fit without establishing their physical meaning.

## Sample selection and validation

| Dataset | Position rows | Effort rows | Default returned rows | Position time range (assumed) |
|---|---:|---:|---:|---|
| `data/` training | 500 | 498 | 498 | 0–0.994 s |
| `data/validation/` | 400 | 398 | 398 | 0–0.794 s |

The loader first applies the configured position cap, then removes the final
two position rows for differentiation and retains the matching leading effort
rows. It accepts effort files with `N` or historical `N-2` rows for `N`
positions; other lengths fail. `trajectory_provenance`, keyed by absolute
data directory, records source counts, the half-open retained source-index
range, configured timing, assumed units and the unresolved torque generation.
This metadata stays outside the base pipeline's array-only return contract.

The default CLI uses `solve(decimate=False)`: no decimation occurs. The empty
`validation_data_file` means reported validation metrics reuse training data.
The separate directory can be configured for a diagnostic run, but it does
not become verified ground truth merely by being held out. Preserve the raw
files; the fresh, reproducible simulation fixture is [`truth/`](truth/).

## Input fingerprints

SHA-256 fingerprints at the audit baseline are listed below. No CSV values
were changed by the signal-loader fix.

- `data/identification_q_simulation.csv`: `cee459266b276c995529a2a979a72a5edc05c3d86fd4c67857888b4d063718b6`
- `data/identification_tau_simulation.csv`: `95443e9f749afa51db1f89a230c630e1c4e30c3eccee88fc522913c04a860828`
- `data/validation/identification_q_simulation.csv`: `95643c9cf273ac5d1fc6207aa85e2fede9e9e5264d466b337c815bb67d4168c7`
- `data/validation/identification_tau_simulation.csv`: `730e02b0eeb125e03f8f503001160e6e562d763d6619a7f084984b2e338c6403`
