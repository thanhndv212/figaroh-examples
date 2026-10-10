# TIAGo identification cross-run protocol v1

This shipment adds two recorded 2021-07-01 sessions to the existing training
run. It addresses the data part of examples#69. It does not fix the signal,
effort-conversion or robot-model issues identified by the audits.

## Sources and reproduction

The source bags are original recordings, not distributed. The shipped
CSVs are everything the examples need. Retain the existing Thanh Nguyen /
CNRS / Toward attribution.

`examples/tiago/data/identification/protocol.yaml` records each source bag's
file name and SHA-256 and the twelve exported file hashes (three joint-channel
files and one wrist F/T file per session). Output paths
are relative to the protocol directory. Source hashes identify the original
bytes even when the external archive changes its layout.

From the examples repository root, in `figaroh-dev`, with optional
`rosbags` installed, export one bag into a separate scratch directory:

```bash
python -m examples.tiago.utils.identification_extraction \
  --bag /path/to/calibration_slow.bag \
  --expected-sha256 acb422296615af6c676f047c8ae080d4819a5af79e59146d06d547ed95dcbfa7 \
  --output-dir /tmp/tiago-identification-slow \
  --wrist-ft
```

`--wrist-ft` adds `tiago_wrist_ft.csv`; the joint CSVs are byte-identical
with or without it.

Repeat for training and payload using their protocol source hashes. Compare
the exported hashes to `sessions[].files`; never overwrite a frozen dataset
to make a mismatch pass. Exact CSV serialization was checked with pandas
2.3.2; other pandas versions may change formatting, so distinguish byte
reproduction from numerical equivalence when investigating a mismatch.

The extractor reads PAL StatisticsNames/StatisticsValues using ROS1 message
definitions embedded in the bags. It resolves names_version and selects
the eight joints in protocol order. Time is `header.stamp` converted to
seconds minus the first stamp. It preserves message order and every sample,
performs no trimming/filtering/effort conversion, and rejects missing
channels, inconsistent names, non-finite values or non-increasing clocks.
Columns are `t` followed by `- <joint>_<position|velocity|effort>`.
`tiago_wrist_ft.csv` has `t` and `wrist_ft_{force,torque}_{X,Y,Z}` (N, N·m,
sensor frame) on the same clock.

## Frozen roles

- Training: `dynamic/`, existing bytes unchanged. Fit on source rows
  [921, 6791) after the adapter's velocity alignment, as before.
- Validation: `calibration_slow/`, 14163 source rows. Same path at about
  half speed; tests transfer across recordings/speed, not new geometry.
- Diagnostic: `calibration_weight/`, 7547 source rows. Added payload changes
  the plant; evaluate sensitivity to load without calling it unchanged-model
  prediction acceptance. Its wrist F/T channels give an independent payload
  mass; see [Payload check](#payload-check-of-the-effort-scale).

Roles and exported hashes are versioned. Changing either requires a new
protocol version and an explanation. The source audit already used both
evaluation recordings; neither is an unseen final acceptance test.
Do not select preprocessing, joints, motor constants or model variants on
evaluation data and then report their error as independent acceptance.

`tasks.identification.data.validation_data_file` defaults to the slow
directory. `identification.py --validation-data <directory>` overrides it.
Evaluation uses the full recording before the same adapter alignment and
core filtering/derivative edge exclusions applied to training; the training
slice is not applied to evaluation. The current loader estimates a velocity
shift separately on each run using kinematic channels, without fitting torque
parameters to evaluation data. A configured evaluation that cannot load is
an error in the entry point, not a silent fallback to training-only results.
The entry point recognises frozen recordings by their three joint-channel
CSV hashes (the wrist F/T file is not read by the loader),
including relocated copies. Prediction acceptance is rejected for the
training and payload-diagnostic roles. The recognised session and role are
saved in trajectory provenance and the reproduction record. A modified
recording in a registered directory is rejected as a protocol mismatch.

## Limits of the current consumer

PAL velocity is a first-order filtered derivative (coefficient 0.95), not
a pure delay. The current lag correction is an approximation. Effort is a
raw controller signal; torque conversions remain assumptions. Torso force
conversion and the arm_1 torque constant lack a validated reference, wrist
efforts are strongly quantized, and the bags indicate a Hey5 hand while the
example historically loads Schunk. These limits are carried in the data
README; no new all-joint physical-parameter acceptance is asserted.

## Payload check of the effort scale

`examples/tiago/payload_check.py` estimates the payload twice, as
`calibration_weight` minus `dynamic`, so that model errors common to both
runs cancel:

- **F/T sensor (reference).** On samples whose sensor-frame linear
  acceleration is below 0.1 m/s², fit `F = R_sensorᵀ·(m·g) + bias`. The mass
  below the sensor is 0.792 kg on `dynamic` (0.793 kg on `calibration_slow`)
  and 1.280 kg on `calibration_weight`, residual 0.29–0.55 N. The payload is
  **0.489 kg**; static thresholds from 0.05 to 0.4 m/s² give 0.480–0.495 kg.
- **Joint efforts (under test).** Convert arm_2–arm_4 efforts with the
  example's `REDUCTION_RATIO × KMOTOR`, subtract nominal RNEA (Hey5 URDF)
  and fit an extra mass and first moment on the arm_7 link, with viscous,
  Coulomb and offset friction per joint. Positions and efforts are low-passed
  at 2 Hz (order 4, zero phase); accelerations are differentiated from
  positions, not from the filtered velocity channel. The payload is
  **0.423 kg**.

The relative error is **−13.5 %**: the converted efforts on arm_2–arm_4 read
low, as the source audit found (−10 to −20 %).

**Tolerance: ±10 %.** It covers the repeatability of both estimates on these
recordings. The effort estimate changes by 0.046 kg (9 % of the payload) when
the payload-free baseline is `calibration_slow` instead of `dynamic`
(0.377 kg, −22.7 %); the F/T reference moves by ±0.008 kg (2 %) with the
static threshold. Their root-sum-square is 9.2 %, rounded up to 10 %. An
error inside ±10 % cannot be told apart from run-to-run spread; −13.5 % can,
so the check reports *efforts read low*. It is a diagnostic: it does not
fail the run and does not change the identification constants. Correcting
those constants is #68, which can use this check as its acceptance.

```bash
cd examples/tiago
python payload_check.py
```

## Inventory and checks

The existing training entry lists its four exact files. Slow and payload
have separate `real` entries. The protocol itself has a `notes` entry.
No path belongs to more than one entry. After staging new data files:

```bash
python scripts/data_inventory.py update
python scripts/data_inventory.py check
pytest tests/test_tiago_identification_datasets.py tests/test_data_inventory.py -q
python validate.py
```

Commit both generated inventory files along with data, extraction code,
protocol, provenance and consumer changes. The inventory verifies ownership
and bytes; dataset tests verify schemas/clocks and consumer wiring. Solver
execution verification is not physical-model or prediction acceptance.
