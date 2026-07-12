# TALOS example (torso/arm calibration)

This folder contains a runnable example for **kinematic calibration** of the
TALOS humanoid robot's torso/arm kinematic chain using FIGAROH.

## What's included

- `calibration_upperbody.py`: all-in-one entry-point for calibration, URDF
  update, and visual validation (run `--help` for modes).
- `update_model.py`: thin shim that delegates to
  `calibration_upperbody.py --update-model`.
- `utils/talos_tools.py`: `TALOSCalibration`, a TALOS-specific specialization of
  `figaroh.calibration.base_calibration.BaseCalibration`.
- `config/`: YAML configuration files (typically use `talos_unified_config.yaml`).
- `data/`: example measurement CSV(s) used by the calibration and timestamped
  calibration results (`calibration_results_*.npz`).
- `urdf/`: TALOS URDF(s) and timestamped modified URDFs.

## Run

Most scripts assume the working directory is this folder:

```bash
cd examples/talos

# ── Full pipeline: calibrate → plot → save → export → viser viz
python calibration_upperbody.py

# Calibrate only (save timestamped .npz, skip export)
python calibration_upperbody.py --calibrate-only

# Load saved results → export URDF → verify FK
python calibration_upperbody.py --update-model

# Visually validate a previously exported modified URDF
python calibration_upperbody.py --viz-validation

# Interactive step selection
python calibration_upperbody.py --interactive

# Thin shim for update-model mode
python update_model.py

# Show all flags
python calibration_upperbody.py --help
```

The script loads `urdf/talos_full_v2.urdf` and uses `models/` (at the repo
root) as the URDF package directory.

All saved files are timestamped to avoid overwriting:
- Calibration results: `data/calibration/calibration_results_{ts}.npz`
- Modified URDFs: `urdf/talos_full_v2_modified_{ts}.urdf`

## Modes reference

| Flag | Behavior |
|---|---|
| *(none)* | Full pipeline: calibrate → plot → save → export → viz |
| `--calibrate-only` | Calibrate + plot + save, then remind about `--update-model` |
| `--update-model` | Select `.npz` → export URDF → verify FK |
| `--viz-validation` | Select modified URDF → viser visualization |
| `--interactive` | Step-by-step selection of what to run |
| `--model <path>` | Skip file selection for `--viz-validation` |
| `--no-plot` | Suppress matplotlib plots (CI) |
| `--html-report` / `--no-html-report` | Self-contained HTML diagnostic report (on by default) |

## Outputs

- Calibration results saved as timestamped `.npz` files in `data/`.
- Modified URDFs saved as timestamped `_modified_*.urdf` in `urdf/`.
- `update_model.py` (or `--update-model`) exports the calibrated URDF and
  prints FK verification metrics.
- A self-contained HTML diagnostic report (`results/calibration_report.html`
  by default) — summary, auto-flagged insights, per-DOF residuals,
  parameter uncertainty, and (if `validation_data_file` is configured) an
  interactive before/after chart. See FIGAROH's
  [Reporting & Verification guide](https://thanhndv212.github.io/figaroh-plus/guides/reporting_and_verification/).

## Notes

- `calibration_upperbody.py` sets `known_baseframe` and `known_tipframe` flags
  explicitly before initialization.
- The calibration config and data format are governed by the YAML under `config/`.
  If you change the dataset, update the YAML paths/fields accordingly.
- The old workflow (`calibration_upperbody.py` → `update_model.py` with
  `offset.xacro`/`offset.yaml`) has been replaced by the combined pipeline.
