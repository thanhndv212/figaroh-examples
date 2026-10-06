# Data contract: TIAGo adapters

Issue: [#17](https://github.com/thanhndv212/figaroh-examples/issues/17).
Delivery package: [W3 / core #35](https://github.com/thanhndv212/figaroh-plus/issues/35).
Contract:
[figaroh-plus decision record](https://github.com/thanhndv212/figaroh-plus/blob/devel/docs/decisions/data-result-contract.md)
(#55, #131).

Two adapters produce the contract types from shipped data. Conventions that
used to be implicit in adapter code now travel with the arrays:
- for identification: joint order, clock, signal origin, effort kind and
  unit, and the conversion applied;
- for calibration: frame, named points and session;
- for both: source rows and source files (sha256).

`tests/test_contract_adapters.py` checks everything below.

## 1. Dynamic: TIAGo identification (`TiagoIdentification.load_trajectory_data`)

`load_trajectory_data` returns a `TrajectoryData`. Its effort conversion is
recorded on the object, so the class no longer overrides
`process_torque_data`.

| Field | Value |
|---|---|
| `joint_names` | `active_joints`: `torso_lift_joint`, `arm_1_joint` … `arm_7_joint` |
| `t`, `clock` | column `t` of the three files, `recorded` (~100 Hz) |
| `origin` | `q` measured; `dq` measured, shifted 18 samples earlier (velocity lag, D2 audit); `ddq` absent, derived by the pipeline |
| `effort_raw`, kind, unit | the recorded effort, `motor_effort`, "raw (unverified, D2 audit)" |
| `effort`, kind, unit | `joint_force` in N on the prismatic torso, `joint_torque` in N·m on the arm |
| `effort_conversion` | `× reduction_ratio × kmotor; + 9.81 × subtree mass (torso)` |
| `sample_index` | file rows `0 … 8003`; the last 18 rows are dropped by the lag shift |
| `source` | the position, velocity and effort files with sha256; session `training`, or the `data_source` directory |

Compatibility:
- The joint effort equals the legacy conversion bit for bit; the test checks
  it against the raw CSV.
- `identification.py` gives the same base parameters and torque RMSE before
  and after this change (golden-hook record compared exactly: RMSE
  0.6659473586774087 both ways).
- The constants not in the YAML moved to `configure_identification()`, which
  `identification.py` and the test share.

Found while doing this: the legacy `config/tiago_config.yaml` resolves no
active joints. The adapter then built a zero-joint trajectory. Core now
refuses it (figaroh-plus#131).

## 2. Geometric: TIAGo mocap (`utils/mocap_observations.py`)

`mocap_observations(path, model, calib_config, session_id)` reads one
session file as `PoseObservations`:

| Field | Value |
|---|---|
| `point_names`, `values` | BL, BR, TR, TL (`x1..z4`), positions in m |
| `measurability` | x, y, z of every point; no orientation |
| `frame` / `registered_to` | `qualisys:base_frame` (the Qualisys body fixed to the robot base) / the calibration's `start_frame` |
| `joint_names`, `q` | the calibration's active joints, file columns |
| `sample_index` | file rows |
| `session`, `source` | session id and date; the file with sha256 and the extraction notes (clock alignment #67; extra columns `shipped_row`, `t_start_robot`, `t_end_robot`, `marker_std_mm`) |

Roles are not part of the data. `data/calibration/mocap/protocol.yaml` is
the [held-out protocol](tiago-mocap-heldout-protocol.md) as a `Protocol`:
- each of the four sessions, with its role and file sha256;
- `protocol_observations()` verifies the hashes before reading, so a
  changed file is refused.

The manifest equals the frozen table of `tests/test_tiago_heldout_protocol.py`.

Compatibility:
- Calibration fits one point. `obs.select_points(["BL"]).to_legacy(...)`
  equals core's `load_data` on every session file, bit for bit.
- `heldout_protocol.py` and `calibration.py` still read the files directly.
  The reference workflow (`reference_run.py`, #29) checks every session file
  against the manifest (sha256 and role) before fitting, and keeps marker 1
  ([reference workflow](tiago-calibration-reference.md); several points,
  figaroh-plus#119, are supported but not adopted).

## 3. Revisions and validation

Paired revisions:
- core: figaroh-plus `devel` with #131 (PR #132);
- examples: this PR.

Results:
- `tests/test_contract_adapters.py`: 8 tests.
- `validate.py`: see the PR.

Legacy inputs are unaffected:
- adapters that return dicts and calibration CSVs work as before (core
  #130);
- the golden outputs are unchanged.
