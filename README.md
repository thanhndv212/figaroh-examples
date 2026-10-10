# FIGAROH Examples

Examples for the [FIGAROH PLUS](https://github.com/thanhndv212/figaroh-plus) library (robot dynamics identification and geometric calibration).

Working with an AI coding agent? [`skills/`](skills/) holds agent skills that set up a calibration, identification, optimal-experiment-design, or new-robot task for you — start with [`skills/figaroh-start`](skills/figaroh-start/SKILL.md), which points at the right package, directory, config keys, and data layout for the job.

## Development planning

Review the [draft contributor guide](CONTRIBUTING.md) and the
[core delivery proposal](https://github.com/thanhndv212/figaroh-plus/blob/devel/docs/plans/identification-calibration-delivery.md)
for the planned parallel identification/calibration workstreams. These are
discussion drafts; they do not claim new solver or pipeline support.

## Install

```bash
pip install -r requirements.txt   # installs figaroh>=0.6,<0.7
```

If you use conda, some dependencies may be easier to install via conda:

```bash
conda install -c conda-forge pinocchio cyipopt
```

Some robot meshes (UR10 camera mount, TIAGo wrist, base casters and WSG
gripper) are not vendored. Fetch them at pinned revisions and point
`ROS_PACKAGE_PATH` at them, or the UR10 and TIAGo examples cannot load
their geometry:

```bash
python scripts/fetch_models.py
export ROS_PACKAGE_PATH="$(python scripts/fetch_models.py --print-ros-package-path)"
```

See [models/README.md](models/README.md) for sources, revisions and licences.

### Core compatibility

Each core release is paired with the examples revision it was validated
against (`validate.py`, all checks passing). The examples `main` branch
follows core `devel` in CI and may need unreleased core changes; to use a
released core, check out the matching examples tag.

| Examples tag | figaroh (PyPI) | Validated pair |
|---|---|---|
| `v0.6.0` | `>=0.6,<0.7` | core `v0.6.0` (`96b586f`) + examples `b23b836` |

To develop against unreleased core, install it from a `devel` checkout instead
(`pip install -e <path-to-figaroh-plus>`).

## Run

Most scripts assume you run them from inside the corresponding robot folder:

```bash
cd examples/ur10
python calibration.py
```

Dynamic identification has a validated reference workflow (fit, verify, export,
reload, held-out report, archive): `python identification_reference.py` in
`examples/ur10` and `examples/tiago`, described in
[docs/development/identification-reference-workflow.md](docs/development/identification-reference-workflow.md).

## Basic workflow

For a new robot or dataset, start with the [new-example guide](docs/new-example-guide.md)
and [experiment brief](docs/experiment-brief-template.md). They cover available
measurements, model/method selection, experiment design, processing, fit
interpretation and held-out validation before the commands below.

1. Choose an example under `examples/<robot>/`.
2. Review the YAML files under `examples/<robot>/config/`.
3. Place or update CSV logs under `examples/<robot>/data/` (or update paths in the YAML).
4. Run one of: `calibration.py`, `identification.py`, `optimal_config.py`, `optimal_trajectory.py` (if present).
5. Review printed results/plots. If applicable, use `update_model.py` to materialize estimated parameters.
6. Add `--html-report` (calibration/identification scripts) for a shareable HTML diagnostic
   report, and `--verify` (identification scripts) for a machine-readable scoped verdict
   you can gate CI on (see "Acceptance policy" below for what a PASS does and does not mean). See each robot's README for the exact flags it supports, and
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

## Acceptance policy

Requires scoped verification
([figaroh-plus#78](https://github.com/thanhndv212/figaroh-plus/pull/78)), released
in figaroh 0.5.0.

Identification `--verify` runs with `--verification-scope execution` by default:
it checks that the fit produced finite, consistent numerical outputs. CLI output
and `verdict.json` name the scope and per-stage status. An execution PASS is
**not** independent prediction acceptance, physical feasibility or export
approval; those stages print NOT_EVALUATED. Improvement over nominal, correlation
and raw condition number remain reported diagnostics, without the former
universal 50% / 0.9 / 1000 gates. Existing CSVs and historical verdicts are
unchanged; a status that changed under this policy is a policy change, not an
improvement in the fit.

Prediction acceptance needs separately loaded validation data **and** a JSON
profile with an explicit limit for every active joint, chosen from measurement
uncertainty and the application before you look at the result:

```json
{
  "validation_rmse:<joint>": {"threshold": 0.05, "comparison": "max", "rationale": "..."}
}
```

```bash
cd examples/so101   # ships a separate simulated validation run
python identification.py --verification-scope prediction --acceptance-profile limits.json
```

Thresholds are in the joint's effort unit (N·m for revolute joints). Optional
`required: false` marks a check advisory. The profile is copied into the run
directory. Missing limits or missing independent data give NOT_EVALUATED and a
nonzero exit; no example-specific limits are shipped or guessed. A separate file
alone does not prove an experimentally independent split or adequate coverage.
`so101/update_model.py` takes the same flags and records the scoped verdict in
the written YAML; its legacy `verification_passed` field is true only for a
passed prediction stage.

`validate.py` runs routine execution checks and keeps each child's complete
stdout/stderr plus exit code and timeout metadata under the git-ignored
`validation_logs/`. A required timeout gives `RESULT: INCOMPLETE` and exit 1.
`--quick` skips the slow optimisation scripts; skipped checks are reported as
not established, not as passed.

## Data format

Examples use CSV logs for measurements and trajectories. The required files/columns depend on the robot and task; see each example README for the expected inputs.

Every data file under `examples/*/data/` is listed with its kind (real, simulated,
generated, unspecified), role and sha256 in the [data inventory](docs/data-inventory.md).
Adding or changing a data file needs `python scripts/data_inventory.py update`
(and a dataset entry in `docs/data-inventory.json` for a new file); CI fails otherwise.

## Examples

- UR10 (manipulator): [examples/ur10/README.md](examples/ur10/README.md)
- TIAGo (mobile manipulator): [examples/tiago/README.md](examples/tiago/README.md) —
  identification, calibration, optimal config/trajectory, plus experimental
  suspension identification and empirical backlash-surface examples
- TIAGo Pro (mobile manipulator, right-arm calibration — contributed by
  [Clement Pene](https://github.com/clementPene)): [examples/tiago_pro/README.md](examples/tiago_pro/README.md)
- TALOS (humanoid, torso/arm chain): [examples/talos/README.md](examples/talos/README.md)
- TALOS table-contact (humanoid, whole-body leg-torso-arm calibration
  from single-plane table contact, no external metrology):
  [examples/talos_table_contact/README.md](examples/talos_table_contact/README.md)
- Staubli TX40 (manipulator): [examples/staubli_tx40/README.md](examples/staubli_tx40/README.md)
- SO-101 (desktop arm, STS3215 servos — gravity + friction from servo current,
  deployed to [soarm_sdk](https://github.com/thanhndv212/soarm_sdk)):
  [examples/so101/README.md](examples/so101/README.md)
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

First fill in the [experiment brief](docs/experiment-brief-template.md) using the
[new-example guide](docs/new-example-guide.md). Then, if needed, use the scaffold
script to create a new robot folder based on the TIAGo template:

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
