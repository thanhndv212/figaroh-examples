# FIGAROH Examples

Runnable examples for [FIGAROH](https://github.com/thanhndv212/figaroh-plus),
a Python toolbox for robot geometric calibration and dynamic identification.
Each robot folder has the URDF, unified YAML configs, data and the
robot-specific subclasses needed to run a task from start to finish.

## Install

```bash
git clone https://github.com/thanhndv212/figaroh-examples.git
cd figaroh-examples
pip install -r requirements.txt
```

`optimal_trajectory.py` scripts also need `cyipopt`, which is installed via
conda (`conda install -c conda-forge pinocchio cyipopt`), or use the
[`environment.yml`](environment.yml) environment.

Some meshes (the UR10 camera mount, the TIAGo wrist, base casters and WSG
gripper) are not vendored. The UR10 and TIAGo examples that load geometry need
them fetched at pinned revisions:

```bash
python scripts/fetch_models.py
export ROS_PACKAGE_PATH="$(python scripts/fetch_models.py --print-ros-package-path)"
```

See [models/README.md](models/README.md) for sources, revisions and licences.

### Core compatibility

Each examples tag is paired with the core release it was validated against
(`validate.py`, all checks passing). `main` follows core `devel` and may need
unreleased core changes; to use a released core, check out the matching tag.

| Examples tag | figaroh (PyPI) | Validated pair |
|---|---|---|
| `v1.1.0` | `>=1.1,<1.2` | core `v1.1.0` + examples `v1.1.0` (evidence on figaroh-plus#228) |
| `v1.0.0` | `>=1.0,<1.1` | core `v1.0.0` + examples `v1.0.0` (evidence on core `56ca51c` + examples `86ef28d`, figaroh-plus#192) |
| `v0.7.0` | `>=0.7,<0.8` | core `v0.7.0` (`c59f057`) + examples `68c2d1a` |
| `v0.6.0` | `>=0.6,<0.7` | core `v0.6.0` (`96b586f`) + examples `b23b836` |

To run against an unreleased core, install it from a checkout:
`pip install -e <path-to-figaroh-plus>`.

## Quickstart

```bash
cd examples/ur10
python identification_reference.py
```

In a few seconds this fits the UR10 dynamic parameters on a simulated training
log, checks the prediction error on a separate held-out log against declared
per-joint limits, computes a physically consistent estimate, exports an
identified URDF, and archives everything under `results/runs/`. The
[reference workflow](docs/development/identification-reference-workflow.md)
explains each step; `examples/tiago` has the same workflow on real data.

## Robots

| Robot | Tasks | Data |
|---|---|---|
| [UR10](examples/ur10/README.md) | Calibration (camera), identification, optimal postures and trajectories | Real (calibration), simulated with held-out split (identification) |
| [TIAGo](examples/tiago/README.md) | Identification, mocap calibration, optimal postures and trajectories; experimental suspension and backlash models | Real; held-out runs for identification and mocap calibration |
| [TIAGo Pro](examples/tiago_pro/README.md) | Right-arm mocap calibration, optimal postures (contributed by [Clement Pene](https://github.com/clementPene)) | Real |
| [TALOS](examples/talos/README.md) | Torso–arm calibration | Real |
| [TALOS table contact](examples/talos_table_contact/README.md) | Whole-body leg–torso–arm calibration from contact with one plane, no external metrology | Real (held out by day) and simulated |
| [Staubli TX40](examples/staubli_tx40/README.md) | Identification | Recorded logs, no separate validation set |
| [SO-101](examples/so101/README.md) | Identification of gravity and friction from servo current, deployed to [soarm_sdk](https://github.com/thanhndv212/soarm_sdk) | Simulated, with held-out split |

The [data inventory](docs/data-inventory.md) lists every data file with its
kind (real, simulated, generated), role and checksum.

## Running an example

Run scripts from inside the robot folder:

```bash
cd examples/tiago
python identification.py --html-report --verify
```

Each folder has some of `calibration.py`, `identification.py`,
`optimal_config.py`, `optimal_trajectory.py` and `update_model.py`, with
configs under `config/`, data under `data/`, the URDF under `urdf/` and the
robot subclasses under `utils/`. Common flags (run `--help` for each script's
list):

| Flag | Effect |
|---|---|
| `--config`, `--urdf` | Use a different config or model |
| `--html-report` / `--no-html-report` | Write a self-contained HTML diagnostic report |
| `--verify` | Write a verdict and exit nonzero if a required check fails |
| `--verification-scope`, `--acceptance-profile` | Choose what `--verify` checks (see below) |
| `--wls` / `--no-wls` | Refine the identification fit with weighted least squares, overriding the config |
| `--archive` / `--no-archive` | Archive the run to `results/runs/<asset>/<task>/<timestamp>/` with its config, provenance and report (on by default) |

### Outputs

An archived run directory holds what you need to reuse or reproduce it. For
the identification reference workflow:

| File | Contents |
|---|---|
| `identified.urdf` | The identified link inertials written onto `nominal_lumped.urdf` (the nominal model with fixed-attached links merged), both archived |
| `parameters.csv` | Identified parameters |
| `reference.json`, `export_check.json` | Per-joint training and held-out RMSE, per-link feasibility, input checksums; the export parity check |
| `verdict.json`, `stages.json` | Verification scope and status of each stage |
| `report.html` | Self-contained diagnostic report |
| `provenance.json`, `config.snapshot.yaml` | Revisions, config hash and the config used |

Calibration writes the corrected joint placements to a URDF; the metrology
frames (camera or mocap base, marker) are reported separately and not written
to the URDF. Where a robot folder has `update_model.py`, it applies a saved
result to the model. See the
[reference workflow](docs/development/identification-reference-workflow.md#run-directory)
for every archived file.

### Verification

By default, `--verify` checks the **execution** scope: the fit produced finite,
consistent outputs. That is not evidence that the model predicts well. A
**prediction** PASS needs separate validation data and a JSON profile with a
limit for every active joint, chosen before you look at the result:

```bash
python identification.py --verification-scope prediction --acceptance-profile limits.json
```

No example ships or guesses limits. The [acceptance policy](docs/acceptance-policy.md)
gives the profile format and what each status means.

## Adding a robot

1. Fill in the [experiment brief](docs/experiment-brief-template.md) using the
   [new-example guide](docs/new-example-guide.md): what you can measure, which
   model and method fit it, and how you will validate. Worked examples:
   [docs/briefs](docs/briefs/README.md).
2. Scaffold a folder from the TIAGo example:
   `cd examples && ./create_example.sh <robot_name>`.
3. Start the config from [examples/templates](examples/templates/README.md).
4. Register new data files with `python scripts/data_inventory.py update`; CI
   fails on unlisted data.

**Working with an AI coding agent?** [`skills/`](skills/) holds agent skills
for calibration, identification, optimal design and new-robot setup. Start
with [`figaroh-start`](skills/figaroh-start/SKILL.md).

## Validating changes

```bash
python validate.py          # pytest plus the example scripts; exit 1 on any failure or timeout
python validate.py --quick  # skips the slow IPOPT scripts (reported as not run, not passed)
python validate.py --changed  # only the robots the diff against main can affect (PR evidence); skips slow scripts
python validate.py --changed --slow-scripts  # same, including the slow IPOPT scripts
```

Each child's output, exit code and timeout data are kept under the git-ignored
`validation_logs/`. See [CONTRIBUTING.md](CONTRIBUTING.md) for the review and
release process.

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

## License and support

Apache License 2.0; see [LICENSE](LICENSE). Open an issue here for
example-specific questions, or in
[figaroh-plus](https://github.com/thanhndv212/figaroh-plus/issues) for the
library.
