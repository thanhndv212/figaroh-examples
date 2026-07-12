# Staubli TX40 example (dynamic identification)

This folder contains a runnable example for **dynamic parameter identification** of the Staubli TX40 manipulator using the FIGAROH base identification workflow.

## What’s included

- `identification.py`: entry point that loads the TX40 URDF, initializes the identification pipeline, and runs the solve step.
- `utils/staubli_tx40_tools.py`: TX40-specific class `TX40Identification` (active joints, filtering/decimation choices, and reporting).
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

The script prints a short identification summary (base parameter count, correlation, and base parameter values). Plotting and saving are controlled by arguments passed to `TX40Identification.solve(...)` in `identification.py` (e.g., `plotting=True`, `save_results=False`, `wls=True`).

## Quality reports & verification

`--verify` and `--html-report` are **on by default** — a plain `python identification.py` run already:

- prints a terminal quality report (condition number, RMSE, per-joint residuals, base-parameter uncertainty),
- writes a self-contained HTML diagnostic report with an interactive before/after chart, and
- checks the run against quality thresholds and writes a machine-readable verdict, exiting non-zero if it fails.

Both are archived to `results/runs/<asset>/identification/<timestamp>/` (see `--archive`). To skip either:

```bash
python identification.py --no-verify --no-html-report
```

See FIGAROH's [Reporting & Verification guide](https://thanhndv212.github.io/figaroh-plus/guides/reporting_and_verification/) for what the report/verdict contain and how to compare two runs.
