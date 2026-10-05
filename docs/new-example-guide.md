# Add a robot or dataset example

**Draft guideline for review — 2026-10-02.** Use the core
[plan, fit and validate guide](https://github.com/thanhndv212/figaroh-plus/blob/devel/docs/source/example_workflow.md)
for the scientific decision process. This document owns repository integration;
it does not introduce new solver or acquisition support. The linked draft core
page will become available when the documentation changes are published.

## Begin with an experiment brief

Copy [experiment-brief-template.md](experiment-brief-template.md) into the
robot folder as `EXPERIMENT.md` and fill it in before adapting scripts. Ask what
data/model already exist, what needs estimating, which methods are supported,
what must be collected and how validation will be independent. Mark unanswered
items explicitly. Reuse an existing robot folder for a new dataset unless the
robot/interface differences justify a separate example.

Use UR10 simulation for dynamic truth checks, TX40/TIAGo for real dynamic-data
patterns and TIAGo mocap for the first calibration reference. These are starting
points, not assurance that another dataset has the same rates, sensor semantics
or observability. TALOS contact observations require a contact-specific model.
Read the dated benchmark limitations before borrowing its protocol.

## Put each artifact in its existing location

```text
examples/<robot>/
  README.md                 # purpose, setup, commands, results and limitations
  EXPERIMENT.md             # decisions and frozen estimation/validation protocol
  config/                   # robot-specific unified YAML
  data/README.md            # raw sources, columns, units, clocks and split policy
  urdf/                     # nominal model or documented retrieval instructions
  utils/                    # robot-specific import/adaptation and subclasses
  identification.py         # only the tasks actually supported
  calibration.py
  optimal_config.py
  optimal_trajectory.py
  update_model.py
```

Use `models/` for shared robot-description packages and document all external
asset versions, retrieval and redistribution permissions. Keep large generated
runs outside source control; do not overwrite nominal URDFs or raw recordings.
Use `benchmarks/` for exploratory method comparisons and mark private core
spikes explicitly. Generic numerical changes belong in the core repository.

The scaffold is optional:

```bash
conda activate figaroh-dev
cd examples
./create_example.sh <robot_name>
```

It produces placeholders. Review inherited TIAGo assumptions, remove unsupported
tasks and supply actual model/data adapters; generation does not complete an
example. Extend a suitable [configuration template](../examples/templates/README.md)
using `extends:`. Override inherited rates, frames, markers, joints, mechanics
and enabled task settings from measured facts rather than accepting defaults.

## Build the smallest reproducible path

1. Resolve the nominal model in a clean checkout and document the active joints,
   payload, reference frames and expected model dimensions.
2. Implement and inspect raw-data import before fitting. Check timestamps,
   joint/channel order, units, synchronization and sample trimming with a small
   inspectable fixture; preserve original files.
3. Declare training/development/validation inputs and their separation before
   filtering, tuning or selecting a method. Record unavailable validation.
4. Establish a supported nominal/base or geometric-fit baseline. Add physical
   reconstruction, projection or research methods only under their own named
   objectives, termination and acceptance checks. For calibration, choose and
   record the estimation method (`parameters.estimation`; see figaroh-plus's
   [guide](https://github.com/thanhndv212/figaroh-plus/blob/devel/docs/source/tutorials/calibration_estimation_guide.md));
   the TIAGo truth fixture (`examples/tiago/calibration_truth.py --methods`)
   shows how to compare methods against a known truth.
5. Report training fit, parameter interpretation and unused-data prediction
   separately. Confirm that the validation adapter actually consumes the
   declared held-out input; config presence alone is not proof.
6. Export an accepted stage into a new output, reload it and compare the
   intended FK/effort predictions. Document unsupported export capabilities.
7. Save commands and evidence so another contributor can reproduce the same
   supported outcome from the declared core/examples commit pair.

Run from the robot directory; do not assume every script has the same flags:

```bash
conda activate figaroh-dev
cd examples/<robot>
python identification.py --help
# Or inspect calibration.py --help for a geometric task.
```

Then put the exact tested task command in the robot README. A plot window or
solver return is not an acceptance check. Use headless commands for automated
runs and keep visual inspection instructions separate.

## Review before advertising the example as validated

- [ ] Experiment brief declares available/missing data, parameter scope, supported
  methods, acquisition rationale and independent validation level.
- [ ] Data/model notes specify sources, rights, units, clocks, frames, coordinate
  order, effort provenance and processing/split indices.
- [ ] A fresh checkout resolves assets and reproduces the documented commands.
- [ ] Report retains baseline and candidate failures; includes per-component
  units/errors, solver status, identifiability and applicable physical checks.
- [ ] Final validation is unused by fitting/tuning, or its absence is explicit.
- [ ] Selected result stage and export/reload parity are explicit where applicable.
- [ ] Essential adapter/entry-point behavior has appropriate regression coverage;
  declare how the example is included in automated validation, or why it is not.
- [ ] Paired revisions, environment, hashes, commands and artifacts are retained.

Follow [CONTRIBUTING](../CONTRIBUTING.md) for the focused issue/PR workflow,
full implementation-phase `validate.py` evidence and explicit review before
merge. Documentation planning does not authorize implementing all proposed
methods. A research-only example must retain that label until its acceptance
criteria are met.
