# Staubli TX40 example (dynamic identification)

This folder contains a runnable example for **dynamic parameter identification** of the Staubli TX40 manipulator using the FIGAROH base identification workflow, and a design of calibration postures.

## What’s included

- `identification.py`: entry point that loads the TX40 URDF, initializes the identification pipeline, and runs the solve step.
- `optimal_config.py`: D-optimal design of kinematic calibration postures (see [Posture design](#posture-design)).
- `utils/staubli_tx40_tools.py`: TX40-specific classes `TX40Identification` (active joints, filtering/decimation choices, and reporting) and `TX40OptimalCalibration`.
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

## Not included

- **`calibration.py`**: there are no TX40 calibration measurements in this
  repository, so there is nothing to calibrate against. The posture design
  above is the first step toward recording them.
- **`optimal_trajectory.py`**: tried with the shared IPOPT generator and this
  config, it does not give a usable excitation: each 5-waypoint segment runs
  to the 200-iteration limit (about 9 min) with a base-regressor condition
  number near 2e12 over 59 base parameters, against 967 for the recorded
  data (58 base parameters). That needs a look at the trajectory
  parametrisation first.
- **`update_model.py`**: the identification gives base parameters, and a URDF
  needs physically consistent standard parameters; writing one would put
  unidentified values in a model as if they were measured.

See FIGAROH's [Reporting & Verification guide](https://thanhndv212.github.io/figaroh-plus/reporting_and_verification/) for what the report/verdict contain and how to compare two runs.
