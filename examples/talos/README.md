# TALOS example (torso/arm calibration)

This folder contains a runnable example for **kinematic calibration** of the
TALOS humanoid robot's torso/arm kinematic chain using FIGAROH.

> For whole-body (leg-torso-arm, both sides) calibration from single-plane
> table contact instead of external metrology, see
> [`../talos_table_contact/README.md`](../talos_table_contact/README.md).

## What's included

- `calibration_upperbody.py`: all-in-one entry-point for calibration, URDF
  update, and visual validation (run `--help` for modes).
- `update_model.py`: thin shim that delegates to
  `calibration_upperbody.py --update-model`.
- `optimal_config.py`: D-optimal design of calibration postures (see
  [Posture design](#posture-design)).
- `identification_truth.py`: torso-arm dynamics identification on **simulated
  data** (see [Dynamics identification, simulated only](#dynamics-identification-simulated-only)).
- `utils/talos_tools.py`: `TALOSCalibration`, a TALOS-specific specialization of
  `figaroh.calibration.base_calibration.BaseCalibration`, and
  `TALOSOptimalCalibration` for the posture design.
- `config/`: YAML configuration files (typically use `talos_unified_config.yaml`).
- `data/`: example measurement CSV(s) used by the calibration and timestamped
  calibration results (`calibration_results_*.npz`), and the motion-capture
  sessions of 2022-03-15 (`data/calibration/mocap/`, described in
  [`data/README.md`](data/README.md)).
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
  [Reporting & Verification guide](https://thanhndv212.github.io/figaroh-plus/reporting_and_verification/).

## Posture design

```bash
python optimal_config.py                  # 1000 candidates, seed 0
python optimal_config.py --candidates 2000 --seed 1
```

There is no recorded candidate pool for TALOS, so the candidates are drawn
uniformly over the calibrated chain (`torso_1/2`, `arm_left_1..7`) within the
URDF limits less `tasks.optimal_configuration.constraints.joint_limit_margin`
(rad), with the rest of the body at `q0`. A candidate that collides with the
robot itself under `models/talos_description/srdf/talos.srdf` is drawn again:
about half of the uniform samples do, and `q0` itself has the left hand on
the thigh. The D-optimal subset (107 postures for seed 0, about 2 min) is
written, timestamped, to `data/optimal_configs/` (git-ignored).

This is a design, not a measurement. Only self-collision is modelled: no
table, fixture or camera line of sight, so check the selected postures on the
real setup before recording them.

## Dynamics identification, simulated only

There are no TALOS dynamics recordings, so there is no `identification.py`.
`identification_truth.py` draws a known truth (consistent link inertias,
friction and actuator inertia in assumed ranges), simulates the torso and
left-arm torques on a self-collision-checked Fourier excitation, identifies
them with core's `BaseIdentification` and scores the fit against the truth:

```bash
python identification_truth.py              # seeds 0-4, torque noise 2 % and 10 %
python identification_truth.py --seeds 0 --noise 0.05
```

With 2 % torque noise the held-out torque error is 0.7 % of RMS (0.2-0.4 %
on the torso and shoulder, 1-2 % on the wrist), and 82 base parameters are
recovered within their standard errors. These are pipeline checks, not
estimates of any TALOS: design, results and limits in
[docs/development/talos-identification-synthetic-truth.md](../../docs/development/talos-identification-synthetic-truth.md).

## Notes

- The calibration entry point is `calibration_upperbody.py`, not
  `calibration.py` as in the other examples: it calibrates the torso/arm chain
  only, and the whole-body case lives in `../talos_table_contact/`. The name
  is kept on purpose.

- `calibration_upperbody.py` sets `known_baseframe` and `known_tipframe` flags
  explicitly before initialization.
- The calibration config and data format are governed by the YAML under `config/`.
  If you change the dataset, update the YAML paths/fields accordingly.
- The old workflow (`calibration_upperbody.py` → `update_model.py` with
  `offset.xacro`/`offset.yaml`) has been replaced by the combined pipeline.
