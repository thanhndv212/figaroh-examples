# FIGAROH Examples

Examples for the [FIGAROH PLUS](https://github.com/thanhndv212/figaroh-plus) library (robot dynamics identification and geometric calibration).

## Install

```bash
pip install figaroh
pip install -r requirements.txt
```

If you use conda, some dependencies may be easier to install via conda:

```bash
conda install -c conda-forge pinocchio cyipopt
```

## Run

Most scripts assume you run them from inside the corresponding robot folder:

```bash
cd examples/ur10
python calibration.py
```

## Basic workflow

1. Choose an example under `examples/<robot>/`.
2. Review the YAML files under `examples/<robot>/config/`.
3. Place or update CSV logs under `examples/<robot>/data/` (or update paths in the YAML).
4. Run one of: `calibration.py`, `identification.py`, `optimal_config.py`, `optimal_trajectory.py` (if present).
5. Review printed results/plots. If applicable, use `update_model.py` to materialize estimated parameters.
6. Add `--html-report` (calibration/identification scripts) for a shareable HTML diagnostic
   report, and `--verify` (identification scripts) for a machine-readable pass/fail verdict
   you can gate CI on. See each robot's README for the exact flags it supports, and
   FIGAROH's [Reporting & Verification guide](https://thanhndv212.github.io/figaroh-plus/reporting_and_verification/)
   for the full walkthrough (HTML reports, `verify()`, and comparing two runs offline).
7. Add `--wls`/`--no-wls` (identification scripts) to override the config's
   `identification.problem.wls` value and refine the OLS base-parameter
   estimate with weighted least squares before quality metrics are computed
   (Staubli TX40's config defaults this on; UR10/TIAGo default off).
8. By default (unless run with `--no-archive`), each run is archived to a
   timestamped `results/runs/<asset>/<task>/<timestamp>/` directory containing
   a config snapshot, a provenance record (git commit, config hash,
   timestamps), and the HTML report if generated — instead of overwriting a
   single `results/` path — and appends a summary line to
   `results/runs/index.jsonl`.

## Data format

Examples use CSV logs for measurements and trajectories. The required files/columns depend on the robot and task; see each example README for the expected inputs.

## Examples

- UR10 (manipulator): [examples/ur10/README.md](examples/ur10/README.md)
- TIAGo (mobile manipulator): [examples/tiago/README.md](examples/tiago/README.md)
- TIAGo Pro (right-arm geometric calibration, mocap-based): [examples/tiago_pro/README.md](examples/tiago_pro/README.md)
- TALOS (humanoid, torso/arm chain): [examples/talos/README.md](examples/talos/README.md)
- Staubli TX40 (manipulator): [examples/staubli_tx40/README.md](examples/staubli_tx40/README.md)
- Templates and config starting points: [examples/templates/README.md](examples/templates/README.md)

## Common layout (per robot)

Most robot folders follow this pattern:

```
{robot}/
  calibration.py            # kinematic calibration (if present)
  identification.py         # dynamic identification (if present)
  optimal_config.py         # optimal measurement configurations (if present)
  optimal_trajectory.py     # exciting trajectories for identification (if present)
  config/                   # YAML configuration files
  data/                     # CSV logs / measurement data
  urdf/                     # robot URDF(s) used by the scripts
  utils/                    # robot-specific helper classes
```

## Creating a new example

Use the scaffold script to create a new robot folder based on the TIAGo template:

```bash
cd examples
./create_example.sh <robot_name>
```

The generated scripts are placeholders that point you back to the TIAGo example for a complete reference implementation.

## Citation

If you use these examples in your research, please cite the main FIGAROH paper:

```bibtex
@inproceedings{nguyen2023figaroh,
  title={FIGAROH: a Python toolbox for dynamic identification and geometric calibration of robots and humans},
  author={Nguyen, Dinh Vinh Thanh and Bonnet, Vincent and Maxime, Sabbah and Gautier, Maxime and Fernbach, Pierre and others},
  booktitle={IEEE-RAS International Conference on Humanoid Robots},
  pages={1--8},
  year={2023},
  address={Austin, TX, United States},
  doi={10.1109/Humanoids57100.2023.10375232},
  url={https://hal.science/hal-04234676v2}
}
```

## License

Apache License 2.0. See `LICENSE`.

## Support

- Open an issue in this repository for example-specific questions.
- Open an issue in the main FIGAROH repository: https://github.com/thanhndv212/figaroh-plus/issues
