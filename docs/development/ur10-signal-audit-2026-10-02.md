# UR10 dynamic signal audit — 2026-10-02

Issue: [figaroh-examples #19](https://github.com/thanhndv212/figaroh-examples/issues/19).
Delivery package: [D2 / core #37](https://github.com/thanhndv212/figaroh-plus/issues/37).
This is an offline adapter audit, not a physical-model comparison or hardware test.

## Methodology and reproducibility

Both baseline and changed runs use core devel
`0af819cc32c82c2560d5fc08926cb00a1dc39541` (merged PR #69); the examples baseline is
`3a2c8e9b07e10b397cda79dbb04a5e88469bcc22`. The changed examples revision is the
commit containing this report, on `fix/19-ur10-signal-audit`. Checks ran against
that uncommitted implementation tree before publication (the run archive therefore
still labels the examples HEAD as `3a2c8e9b`); resolve the final tested source via
the PR commit, not that archive label. Independent worktrees
prevent baseline runs from importing the changed adapter. `PYTHONPATH` selects
the core checkout explicitly rather than the older editable installation.

All execution uses `figaroh-dev`: Python 3.12, Pinocchio 3.7.0, NumPy 2.3.2,
SciPy 1.16.1 and pandas 2.3.2. An isolated `figaroh-dev` clone additionally runs
the analytic loader regressions with Pinocchio 4.1.0. These tests use quadratic
positions with known derivatives in every coordinate and independent sentinel
efforts; they do not regenerate expected torques with the code under test.

Commands (run from each examples checkout, except the CLI from `examples/ur10`):

```bash
conda run --no-capture-output -n figaroh-dev env PYTHONPATH="$CORE/src" python validate.py
conda run --no-capture-output -n figaroh-dev env PYTHONPATH="$CORE/src" python -m pytest tests/test_ur10_signal_processing.py -q
conda run --no-capture-output -p /tmp/figaroh-pin41/figaroh-dev env PYTHONPATH="$CORE/src" python -m pytest tests/test_ur10_signal_processing.py -q
conda run --no-capture-output -n figaroh-dev env MPLBACKEND=Agg PYTHONPATH="$CORE/src" python identification.py --verify --no-html-report --no-archive
```

Set `CORE` to the checkout at the core revision above. Neither verification
thresholds nor CSV values are changed. Full validation logs were captured locally
as `/tmp/figaroh-issue19-validation-{before,after}.log`; CLI logs use
`/tmp/figaroh-issue19-ur10-{before,after}.log`. The results below are the portable
record; those temporary files are not repository artifacts.

## Diagnosis before correction

The [data-generation commit](https://github.com/thanhndv212/figaroh-examples/commit/200a6af52f26a08b88a4bc0a79d4999cf2278ac9)
claims distinct persistent-excitation Fourier trajectories and torque generation
with `pin.rnea`, using the same finite-difference pipeline as identification.
Its files do not preserve a runnable generator, seeds, generating inertial vector
or source timestamps. Its reported high fits are historical claims, not a
reproduction under the corrected differentiation. The missing generator also
prevents establishing whether the old final-acceleration defect is embedded in
the stored torques. File contents alone cannot identify the real source clock.

The active unified config resolves `ts=0.002`, filter sample rate 500 Hz and
cutoff 50 Hz/order 4. The old loader differentiates using that `ts`, but labels
time with a hard-coded 0.01 s spacing. The legacy flat config separately uses
0.01 s. Choosing either clock without generation evidence is an assumption.

The loader's optional `apply_lowpass_filter` call never executes: the imported
`DataProcessor` does not provide that method. The real filtering is in core
`BaseIdentification.filter_kinematics_data`. This corrects the earlier audit
assumption that a 100 Hz loader prefilter was active; no claim of double filtering
is supported by these runs. Core applies median window 5 and `filtfilt` Butterworth
order 4/cutoff 50 Hz/sample rate 500 Hz to each supplied kinematic channel;
padding is 12 samples and filtered edges remain in the fit. Efforts remain raw.

Every CSV column maps to one of the six scalar URDF joints in the
[data contract](../../examples/ur10/data/README.md). No channel is a timestamp or
measured velocity/acceleration. Effort sign and rad/Nm interpretation follow the
historical RNEA claim and URDF ordering, with no current/gear/sign conversion.
The default CLI disables decimation. If enabled explicitly, the inspected core
path applies `scipy.signal.decimate(..., zero_phase=True)` independently to each
joint's torque series and regressor columns before stacking joint-major blocks;
this is filtering plus downsampling, not a raw-row subset. An additional loader/
core diagnostic exercised factor 10 on this UR10 dataset: all six joint blocks
had 50 output rows (300 stacked rows, 49 regressor columns). Torque and regressor
blocks matched a separately assembled per-joint SciPy decimation reference
exactly (maximum absolute difference 0). This verifies numerical block alignment
for the current contiguous six-joint model, not physical time collocation or
arbitrary active-joint subsets. There is no extra raw-row crop; output grid
indices are nominally 0,10,…,490 but values include anti-alias filtering.

## Correction and retained limits

The adapter now derives timestamps from the same configured `ts` as its
kinematics, rejects a conflicting filter clock, selects named CSV channels in
model order, and rejects nonfinite values or incompatible effort row counts.
It retains all stored values/signs and the historical cap followed by `N-2`
trailing trim. Source counts and retained indices are exposed separately as
`trajectory_provenance`, with `timing_source="configured_assumption"` and
`torque_generation_verified=False`. This attribute is available in memory; the
existing run-archive serializer does not automatically include it. The data
contract and this report preserve the audited indices/hashes; general run-level
provenance integration is outside this adapter fix. Removed dead prefilter code changes no
numerical signal values in the tested environment.

Training has 500 position / 498 effort rows; validation has 400 / 398. The default
cap is 500 positions, so output counts are 498 and 398. Training time labels change
from 0–4.970 s to 0–0.994 s; validation labels change from 0–3.970 s to 0–0.794 s.
These corrected labels express the configured assumption, not a recovered clock.
Forward velocity and acceleration remain at interval centers, half a timestep
after their paired position rows; the offset is 0.001 s at 500 Hz. The historical
torque alignment is preserved, not certified as physically simultaneous.

## Model fitting and parameter interpretation

| Default UR10 CLI metric | Before | After |
|---|---:|---:|
| Retained samples | 498 | 498 |
| Base parameters | 36 | 36 |
| Base-regressor condition number | 21302.771354 | 21302.771354 |
| Training torque RMSE (Nm, pooled joints) | 0.066744 | 0.066744 |
| Nominal-to-identified RMSE improvement | 7.3% | 7.3% |
| Verification exit code | 1 | 1 |

Timestamp correction changes labels, not the default regression inputs; the
unchanged fit is expected. Condition exceeds the existing 1000 limit and
improvement falls below 50%; correlation passes. No thresholds were relaxed.
The high correlation does not establish recoverable inertial truth: this is an
ill-conditioned base fit with large relative uncertainties (up to 485% among
the reported largest values). No SDP reconstruction, log-Cholesky fitting or
URDF export was performed in this issue, so there is no full-parameter physical
verdict or method ranking.

## Held-out validation

The independent-directory CSVs load as 398 rows, with 0.002 s assumed spacing;
loader metadata retains their own paths/counts. The default config keeps
`validation_data_file` empty and its CLI metrics explicitly report a training
fallback. This audit does not treat them as held-out validation. Historical
claims that the separate files used different generator seeds remain unreplayable.
Fresh clock/model/seed provenance and independently generated effort truth belong
to [issue #21](https://github.com/thanhndv212/figaroh-examples/issues/21).

## Validation results

The ten analytic regression cases fail against the original adapter and pass
with the correction on both Pinocchio 3.7 and 4.1. They cover both assumed sample
periods, both supported effort lengths, capped trailing trims, all six derivatives,
column reordering, row-count errors, nonfinite values, unexpected channels and
filter-clock disagreement. The real UR10 CLI also completes on Pinocchio 4.1
with the same displayed condition/RMSE and the same expected verification
failure (exit 1). CSV/model hashes matched the independent baseline worktree.
Full repository validation completed with identical check statuses: **11 passed,
4 failed, 0 skipped and 1 timed out**, exit 1 in both runs. These are counts of
validator checks (one pytest-suite check plus 15 scripts), not pytest test cases.

| Check | Baseline | Changed |
|---|---|---|
| `test/pytest` | PASS (201.7 s) | PASS (239.4 s) |
| `script/ur10/calibration.py` | PASS (2.7 s) | PASS (2.8 s) |
| `script/ur10/update_model.py` | PASS (1.4 s) | PASS (1.4 s) |
| `script/ur10/identification.py` | FAIL (2.6 s) | FAIL (2.7 s) |
| `script/ur10/optimal_config.py` | PASS (65.2 s) | PASS (61.5 s) |
| `script/ur10/optimal_trajectory.py` | TIMEOUT (600.1 s) | TIMEOUT (600.2 s) |
| `script/tiago/calibration.py` | PASS (5.1 s) | PASS (8.8 s) |
| `script/tiago/update_model.py` | PASS (1.6 s) | PASS (2.4 s) |
| `script/tiago/identification.py` | FAIL (5.8 s) | FAIL (6.5 s) |
| `script/tiago/optimal_config.py` | PASS (15.8 s) | PASS (15.7 s) |
| `script/tiago/optimal_trajectory.py` | FAIL (244.0 s) | FAIL (259.1 s) |
| `script/talos/calibration_upperbody.py` | PASS (5.2 s) | PASS (3.9 s) |
| `script/talos/update_model.py` | PASS (1.9 s) | PASS (1.5 s) |
| `script/staubli_tx40/identification.py` | FAIL (27.2 s) | FAIL (22.6 s) |
| `script/so101/identification.py` | PASS (2.8 s) | PASS (2.3 s) |
| `script/so101/update_model.py` | PASS (2.5 s) | PASS (1.9 s) |

The three identification failures are existing quality gates: UR10 condition
21302.77 > 1000 and improvement 7.30% < 50%; TIAGo condition 3617.75 > 1000;
TX40 improvement 12.74% < 50%. Baseline and changed verdicts agree. UR10 trajectory
optimization reaches the validator's 600 s timeout in both runs. TIAGo trajectory
optimization exits unsuccessfully in both (244.0/259.1 s); the validator prints
only trailing parameter warnings and does not expose its cause. That cause
remains unresolved; it is not classified as an identification-gate failure or
an IPOPT timeout. No additional failed check was introduced by the adapter fix.

The full optimizer stdout/stderr was not retained by `validate.py`, which prints
only excerpts. For debugging, raw full-length identification logs, copied verdicts,
HTML reports and optimizer rerun commands are indexed locally in
`/tmp/figaroh-issue19-debug/README.md`; identification reruns are separate from the
original full validation. No claim is made that those optimizer excerpts are
complete logs.

## Input, model and configuration fingerprints

SHA-256 baseline fingerprints follow; the data and model remain unchanged.
The only unified-config edit is a comment explaining the assumed clock; the
resolved numerical settings are unchanged. Template hashes capture inherited
configuration.

- `examples/ur10/urdf/ur10_robot.urdf`: `1da0c0de1909bbf6bb5ea9449ee07ccf88e0b3456edca2955e23cc4770027ecd`
- `examples/ur10/config/ur10_unified_config.yaml`: `bd093e400cd8400d283fb883f49a4a76d02bbdc9d8b523728b6b69dcb3a6f883`
- `examples/templates/manipulator_robot.yaml`: `0513e0c196b9a3cbb4969d98195933c3fd5577a713a024e9451f4f5eef8f5b16`
- `examples/templates/base_robot_config.yaml`: `cc3b9ad641afcf29a34a3c2885a21aaa11116b60a972aef9e3a15c8beb67928c`
- `examples/ur10/data/identification_q_simulation.csv`: `cee459266b276c995529a2a979a72a5edc05c3d86fd4c67857888b4d063718b6`
- `examples/ur10/data/identification_tau_simulation.csv`: `95443e9f749afa51db1f89a230c630e1c4e30c3eccee88fc522913c04a860828`
- `examples/ur10/data/validation/identification_q_simulation.csv`: `95643c9cf273ac5d1fc6207aa85e2fede9e9e5264d466b337c815bb67d4168c7`
- `examples/ur10/data/validation/identification_tau_simulation.csv`: `730e02b0eeb125e03f8f503001160e6e562d763d6619a7f084984b2e338c6403`

Changed unified-config SHA-256 (comment-only edit): `2f06d850ded44631a4fb4f241c231c25fdeb4d4d9cb31266298de79b8b2ec4ea`.
