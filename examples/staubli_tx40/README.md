# Staubli TX40 example (dynamic identification)

This folder contains a runnable example for **dynamic parameter identification** of the Staubli TX40 manipulator using the FIGAROH base identification workflow, and a design of calibration postures.

## What’s included

- `identification.py`: entry point that loads the TX40 URDF, initializes the identification pipeline, and runs the solve step.
- `optimal_config.py`: D-optimal design of kinematic calibration postures (see [Posture design](#posture-design)).
- `calibration_truth.py`: calibration on **simulated** tool poses with a known truth (see [Calibration, simulated only](#calibration-simulated-only)).
- `utils/staubli_tx40_tools.py`: TX40-specific classes `TX40Identification` (active joints, filtering/decimation choices, and reporting), `TX40Calibration` (6-DOF tool-pose residual) and `TX40OptimalCalibration`.
- `config/`: YAML configuration files (typically use `staubli_tx40_unified_config.yaml`).
- `data/`: input CSV logs used by the example (`pos_read_data.csv`, `curr_data.csv`).
- `urdf/`: the TX40 URDF model used by the scripts.

## Run

Most scripts assume the working directory is this folder:

```bash
cd examples/staubli_tx40
python identification.py
```

## Inputs and configuration

- URDF: `urdf/tx40_mdh_modified.urdf` (loaded by `identification.py`).
- Config: `config/staubli_tx40_unified_config.yaml` (passed to `TX40Identification`).
- Data: CSV files under `data/` are loaded according to paths/fields configured in the YAML.

If you swap datasets or change sample time / filtering, update the YAML config accordingly.

## Outputs

The script prints a short identification summary (base parameter count, correlation, and base parameter values). Plotting and saving are controlled by arguments passed to `TX40Identification.solve(...)` in `identification.py` (e.g., `plotting=True`, `save_results=False`).

## Quality reports & verification

`--verify` and `--html-report` are **on by default** — a plain `python identification.py` run already:

- prints a terminal quality report (condition number, RMSE, per-joint residuals, base-parameter uncertainty),
- writes a self-contained HTML diagnostic report with an interactive before/after chart, and
- runs scoped verification (default `--verification-scope execution`: finite, consistent fit outputs) and writes a machine-readable verdict, exiting non-zero on FAIL or NOT_EVALUATED. An execution PASS is not prediction, physical-model or export acceptance; see the top-level README for `--verification-scope prediction --acceptance-profile`.

Both are archived to `results/runs/<asset>/identification/<timestamp>/` (see `--archive`). To skip either:

```bash
python identification.py --no-verify --no-html-report
```

`--wls`/`--no-wls` overrides the config's `identification.problem.wls` value
and is **on by default for TX40** (unlike UR10/TIAGo, which default off) —
it refines the OLS base-parameter estimate with weighted least squares
before quality metrics are computed. Use `--no-wls` for the plain OLS
estimate:

```bash
python identification.py --no-wls
```

## Posture design

```bash
python optimal_config.py                  # 1000 candidates, seed 0
python optimal_config.py --candidates 2000 --seed 1
```

There is no recorded candidate pool for the TX40, so candidates are drawn
uniformly over joints 1-6 within the URDF limits less
`tasks.optimal_configuration.constraints.joint_limit_margin` (rad), and those
where non-adjacent links collide are drawn again. The D-optimal subset (66
postures for seed 0, about 1 min) is written, timestamped, to
`data/optimal_configs/` (git-ignored). The URDF has no floor or cell, so
check the postures against the real installation before recording.

## Calibration, simulated only

```bash
python calibration_truth.py                 # 2 designs x 2 truths x 2 noise x 3 fits x 5 seeds, ~3 min
python calibration_truth.py --seeds 0 --noise 0.1 --designs doptimal
python calibration_truth.py --design        # rerun the D-optimal selection into data/truth/
```

There are no TX40 calibration measurements, so the calibration is checked on
simulated ones: a drawn truth, tool poses from it with seeded noise at the 66
D-optimal postures (or 66 random ones), a FIGAROH fit, and the fit's tool
position at 300 held-out postures against the truth. The output says what
the pipeline recovers, not what a real TX40 looks like. Results, and a core
selection bug the study found (fixed for this robot by
`structural_selection: ordered` in the config):
[docs/development/tx40-calibration-synthetic-truth.md](../../docs/development/tx40-calibration-synthetic-truth.md).

## Excitation trajectory (`optimal_trajectory.py`)

```bash
python optimal_trajectory.py        # 2 segments, seed 0, about 13 min
```

Designs a condition-number-optimal excitation for the identification model,
coupled-wrist columns included, with core's IPOPT generator. Seed 0 gives a
base-regressor condition number of **114** over 61 base parameters, against 967
for the recorded data. **Simulated design only**: not run on a TX40, and the
URDF has no floor or cell, so check it against the installation first.
`TX40OptimalTrajectory` picks base columns with a relative QR tolerance;
core's absolute one keeps a collinear pair and pins the condition number
near 1e13. Details in
[docs/development/tx40-optimal-trajectory.md](../../docs/development/tx40-optimal-trajectory.md).

## Updating the model (`update_model.py`)

```bash
python update_model.py              # writes results/tx40_identified.urdf
```

Fits **physically consistent** link inertials to the recording (core's
direct physical fit, `select_stage: physical_fit`, regularised towards the
CAD URDF with `--prior-weight`, default 100) and writes them into a copy of
the URDF. **Not independently validated**: the TX40 has one recording, so the
fit is scored on the data it was fitted to. Training torque RMSE rises by
0.8 % over the base-parameter fit; masses stay within 15 % of CAD. Link 6 is
a point mass in the CAD URDF and link 3 comes out rod-like; the script flags
both. Details in
[docs/development/tx40-update-model.md](../../docs/development/tx40-update-model.md).

## Not included

- **`calibration.py`** on real data: there are no TX40 calibration
  measurements in this repository. `TX40Calibration` reads them in the
  layout `calibration_truth.py` writes (`x1 … phiz1`, `joint_1 … joint_6`)
  once they exist.

See FIGAROH's [Reporting & Verification guide](https://thanhndv212.github.io/figaroh-plus/reporting_and_verification/) for what the report/verdict contain and how to compare two runs.
