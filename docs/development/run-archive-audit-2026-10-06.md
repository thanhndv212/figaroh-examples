# Run archive reproduction record — 2026-10-06

Issue: [figaroh-examples #30](https://github.com/thanhndv212/figaroh-examples/issues/30).
Delivery package: [S1 / core #47](https://github.com/thanhndv212/figaroh-plus/issues/47).
This audit checks whether a run archive holds what is needed to reproduce
and audit a reference run. It covers one dynamic run and one geometric run,
both from TIAGo. It also records the evidence added for what was missing.

## Methodology

- Core figaroh-plus `devel` at `a7a2a02` (#63); examples `main` at `9a9e6d7`.
  The changed revision is the commit containing this report.
- `figaroh-dev`, Python 3.12.11, Pinocchio 3.7.0, macOS arm64.
- Both runs used the existing archive machinery
  (`compute_run_dir` / `archive_run`), from `examples/tiago`:
  - dynamic: `identification.py --asset-id AUDIT30`;
  - geometric: `calibration.py --calibrate-only --no-plot --asset-id AUDIT30`
    (mocap, held-out protocol v1).
- Each archive was checked against the list below, item by item.
  `python -m examples.run_record RUN_DIR` repeats the check on any archive.

## Checklist

| Item | Evidence required |
|---|---|
| invocation | command line and working directory |
| revisions | examples and core commits, both clean |
| config | `config.snapshot.yaml` and the config file sha256 |
| model | nominal URDF sha256 |
| inputs | sha256 of every input file |
| processing | processing applied outside the config |
| splits | training/validation data and the validation source |
| solver | fit stage status and solver message |
| selected_stage | the stage the reported parameters come from |
| report | `report.html` |
| export | `parameters.csv`, and every generated file with matching sha256 |

## Findings before the change

| Item | Dynamic (identification) | Geometric (calibration) |
|---|---|---|
| invocation | **missing** | **missing** |
| revisions | ok: examples `9a9e6d7b`, core `a7a2a026`, both clean | ok |
| config | ok | ok |
| model | ok: `urdf/tiago_48_schunk.urdf` `e4412357…` | ok |
| inputs | ok: position, velocity, effort CSVs | ok: training and validation sessions, sample configurations. Protocol manifest **not recorded** |
| processing | **missing**: truncation window `(921, 6791)`, decimation, WLS and the estimated velocity lag are set in the script and recorded nowhere | **missing**: `known_baseframe`/`known_tipframe` set in the script; no record that the data files play their protocol roles |
| splits | ok (`verdict.json`): validation source `training_fallback` | **missing**: the script writes no verdict |
| solver | ok: `least squares` | ok: `` `ftol` termination condition is satisfied `` |
| selected_stage | ok: `fit` | **missing**: no verdict, and the `index.jsonl` entry has `passed: null` and no stages |
| report | ok | ok |
| export | `parameters.csv` | `parameters.csv`, PAL YAMLs. The results `.npz` (the input to `--update-model`) and the exported URDF are written outside the run directory and **not referenced** |

Other observations, not gaps:
- `verdict.json` splits key the files by absolute path, so they depend on
  the machine. Provenance `data` holds the same hashes with relative paths.
- The splits count the identification samples before truncation (8004 rows);
  the `data` stage counts after truncation (5870). The truncation window
  recorded under processing accounts for the difference.
- Large inputs are referenced by sha256, not copied. That keeps archives
  small and keeps generated files out of tracked fixtures (`results/` and
  `calibration_results_*.npz` are git-ignored).

## Evidence added

`examples/run_record.py` writes `reproduction.json` next to the core archive
files. After `archive_run`, the script records:
- `invocation`: `argv`, working directory, interpreter;
- `processing`: settings applied outside the config;
- `inputs`: input files not in provenance `data`, with sha256;
- `artifacts`: generated files written outside the run directory, with
  sha256.

The script then audits the directory and stores `checklist` and `missing`,
so every archive states what is missing. A changed or deleted
input/artifact, or an uncommitted examples/core tree, shows as `incomplete`.

Per script:
- `tiago/identification.py`: `truncate`, `decimate`, `wls`, `velocity_lag`
  (as requested) and `trajectory` (`trajectory_provenance`: recorded clock,
  estimated lag in samples and seconds, dropped rows, effort audits).
- `tiago/calibration.py`:
  - writes `verdict.json` through the shared `run_verification`
    (`--verify`, default on, scope `execution`, like identification), so the
    archive and the index carry stages, splits and the selected stage;
  - saves the results `.npz` before archiving and references it;
  - records `protocol.yaml` with its sha256, and the session and role that
    each data file plays under it (`training`, `validation`, or `null` when
    the file is not in the protocol);
  - in the full pipeline, adds the exported URDF to the record after
    export.

## Findings after the change

Both runs, same commands, same revisions plus this change:
- every item `ok`;
- except `revisions`, which reads `incomplete` while the examples tree has
  uncommitted changes. That is the intended result: a run from a dirty tree
  cannot be reproduced from its commits.

Fit results are unchanged: torque RMSE 0.6659473586774087, calibration
position RMSE 2.88 mm. `tests/test_golden_outputs.py` passes.

Calibration data roles recorded:

| File | Session | Role |
|---|---|---|
| `qualisys_2021-11-30_static_postures.csv` | `2021-11-30-1544` | training |
| `qualisys_2021-11-26_static_postures.csv` | `2021-11-26-1105` | validation |

## Not covered

- Other robots' scripts still archive without `reproduction.json`.
  `python -m examples.run_record` reports their gaps. Adding the record
  belongs to each robot's reference workflow (#23, #29 for TIAGo).
- The audit checks hashes and presence. It does not re-run a fit to compare
  numbers; that is `tests/test_golden_outputs.py`.

## Tests

`tests/test_run_record.py`:
- unit tests on a synthetic archive: an archive without a record names
  its gaps; a record completes the checklist; changed files, absent files
  and dirty trees are `incomplete`; artifacts can be added later;
- a slow integration test that runs both TIAGo reference runs and requires
  every item except `revisions` to be `ok`.
