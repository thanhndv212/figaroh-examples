# Contributing to FIGAROH Examples

**Draft for review — 2026-10-02.** This guide consolidates existing execution
constraints and proposes the missing examples-specific contribution workflow.
It does not install tracker/CI automation or authorize implementation/merges.
Discuss it with the [core delivery plan](https://github.com/thanhndv212/figaroh-plus/blob/devel/docs/plans/identification-calibration-delivery.md)
before adopting new rules. Core shared process is in
[FIGAROH CONTRIBUTING](https://github.com/thanhndv212/figaroh-plus/blob/devel/CONTRIBUTING.md).

## Scope and structure

This repository owns robot-specific scripts, CSV adapters, models, unified
configs and reproducible experiments. Generic numerical algorithms belong in
sibling core `figaroh/src/figaroh/`. It is not an installable Python package.

- Supported recipes stay under `examples/<robot>/`; scripts run from that robot
  directory because model/config/data paths are relative.
- Existing shared robot description packages stay in `models/`.
- Research comparisons stay in `benchmarks/`, labeled with their supported or
  private core dependency. A benchmark completing does not establish a
  production API or a scientifically validated model.
- Dataset/model sources, units, joint/observation frames, timestamps, conversions,
  sampling/filter rates and allowed redistribution belong in robot data notes.
- Generated runs use separate output/archive directories. Preserve original
  CSVs/URDFs; do not run destructive model-update steps against source fixtures.
- Do not add another repository-level roadmap. Core owns cross-repository
  milestones; link its plan and create examples issues for concrete local work.

## Adding an example

Use the [new-example guide](docs/new-example-guide.md) and copy its
[experiment brief](docs/experiment-brief-template.md) into the robot folder.
Review data availability, model scope, method objectives, acquisition/processing
and validation before adapting scripts. A scaffold or reference workflow alone
does not establish a validated example. The brief is documentation, not a new
configuration schema or runtime requirement.

## Environment and reproduction

All tests, scripts, hooks and Python runs use **`figaroh-dev`**, defined in the
sibling core repository. The local examples `environment.yml` is not the
contributor environment definition. Use conda for cyipopt/IPOPT. The supported
compatibility evidence includes explicit Pinocchio 3.7/4.1 native profiles;
record exact dependency versions rather than installing an unconstrained
latest version and downgrading afterward.

Record the exact core/examples commits validated together. To reproduce the
hosted CI environment and record verified pairings, follow
[docs/development/environment.md](docs/development/environment.md). Preserve unrelated
changes and existing branches. For a clean environment, reproduce geometry
resolution using the pinned sources in [models/README.md](models/README.md)
(`python scripts/fetch_models.py`). A developer's external
`ROS_PACKAGE_PATH` is not sufficient CI evidence. Do not copy third-party
assets without checking their redistribution terms.

## Planning and PR workflow

1. Discuss a proposed change before implementation when its method, contract
   or scope is unresolved. A plan approval, implementation instruction and
   merge approval are distinct actions.
2. Reuse an existing issue; otherwise create one only when ready, with the
   *Feature / work item* or *Bug report* template (shared with figaroh-plus;
   see [how to read delivery issues](https://github.com/thanhndv212/figaroh-plus/blob/devel/docs/plans/README.md#how-to-read-delivery-issues)).
   Refer to issues as `#N` here and `thanhndv212/figaroh-plus#N` for core, which GitHub links. Specify robot,
   core dependency, immutable inputs, failure classification, expected behavior,
   acceptance evidence and out-of-scope changes. Cross-repository changes link
   a core issue/PR and an examples issue/PR.
3. Branch from this repository's current `main`; normal examples PRs target
   **`main`**. Core normal PRs target `devel`. Do not assume an examples `devel`
   integration branch. One issue/outcome per PR.
4. For a numerical/data fix, reproduce the original behavior and compare the
   changed result under the same inputs. Keep broad formatting separate.
5. Validate and record evidence, then commit/push and open the focused PR.
   Independent work uses a separate branch/worktree; core dependencies are
   exact commits until an accepted compatible release is available.
6. Maintainer reviews. Merge only after explicit approval and required checks
   pass on the current head. Material scope changes require another review.
7. After merge, link/close the issue, sync the base, and delete only the merged
   feature branch locally/remotely and its worktree after checking for local
   or unique work. Preserve main/devel/releases and unrelated older branches.

These steps preserve the established review policy. New tracker states/labels
and CI changes in the delivery plan remain proposals, not deployed automation.

## Validation

From repository root in `figaroh-dev`:

```bash
python validate.py --changed          # PR evidence: what the diff against main can affect
python validate.py --changed --dry-run  # print the selection only
python validate.py --changed --slow-scripts  # also run the slow IPOPT scripts
python -m pytest tests/ -m ur10       # one robot's tests (also tiago, tiago_pro, ...)
pre-commit run --files <changed-files>
git diff --check
```

PR evidence is the `python validate.py --changed [BASE]` output (BASE defaults
to `main`). It diffs `BASE...HEAD`, the working tree and untracked files,
maps the paths through `tests/impact.yaml`, and runs the affected robots'
tests, golden cases and scripts plus the cheap cross-robot checks (smoke,
configs, data inventory). It prints the selection with a reason per path and
writes it to `validation_logs/`. Quote that output in the PR.

Run full `python validate.py` (and `python -m pytest tests/ -v`) only when the
change touches shared code (`examples/calibration_harness/`, `run_record.py`,
`verification.py`, `templates/`, ...), the core revision, the environment or
pins, `tests/conftest.py`, `validate.py`, `models/`, `ci/` or the golden
reference, and for release and acceptance gates. `--changed` already selects
the full suite for those paths and for any path the map does not cover, so
the safe default holds. CI runs the full fast suite on every PR regardless.

`--changed` skips slow scripts (`is_slow` entries such as `optimal_config` and
`optimal_trajectory`, minutes each) by default; add `--slow-scripts` when the
diff touches them. Slow tests (`-m slow`) run weekly in `.github/workflows/slow.yml`
and locally with `python -m pytest tests/ -m slow`; a full `validate.py` still
runs every script.

`--quick`, `--tests-only`, `--scripts-only` and `--full-tests` combine with
`--changed`; `--robot` does not (use one or the other). Run affected robot
commands from their robot folder with `MPLBACKEND=Agg`. Document timeouts and
missing assets separately from wrong numerical results. Documentation-only
discussion changes use hooks/diff/link checks; do not rerun long optimization
merely to validate prose.

`pytest -m <robot>` (for example `-m ur10`, `-m talos_table_contact`) selects
a robot's tests; `-m shared` and `-m cross_robot` select the shared-code and
cross-robot ones. The markers come from `tests/impact.yaml`, not from the test
files. **A new `tests/test_*.py` must be listed in `tests/impact.yaml`** under
the robot it exercises (`robots.<name>.tests`), under the shared code it tests
(`shared.<name>.tests`), or, if it spans robots, under `always` (cheap, runs
every time) or `cross_robot` (rerun everything when it changes). Collection
fails with the file name until you do, and `tests/test_impact.py` checks the
map against the files on disk. A new robot gets its own `robots` entry with
paths, tests, `golden_ids` and the `validate.py` script key.

`tests/test_golden_outputs.py` runs the calibration and identification
example scripts and compares what they produce (parameter counts, fit RMS,
exported-model deviation, torque RMSE) with `tests/golden/golden_outputs.json`.
When a core or example change moves an output on purpose, regenerate the
reference in the same examples PR with
`python tests/golden/golden_outputs.py --update`, and state in the PR which
outputs moved and why. A core PR that moves an output needs that paired
examples PR; merge the core PR first, since examples CI tests against core
`devel`. Do not widen `TOL` or `PLATFORM_SPREAD` to absorb a change.

A phase can be accepted only with passing checks or explicitly reproduced,
known pre-existing failures under the existing agent instructions. Such local
acceptance does not bypass a required failed hosted merge gate. Keep solver
termination, input correctness, parameter feasibility, held-out quality and
export/reload checks as separate verdicts. Never loosen thresholds, exclude
regressions or relabel fallbacks to make CI green.

For dynamic identification, report per-joint effort units and errors, rank,
extras/weighting policy, physical verdict and selected output stage. For
calibration, report frames/gauge, pose/contact residual units, identifiable and
redistributed corrections and reloaded-model FK parity. State whether validation
is training fallback, a temporal block, separate simulation or an independent
real recording.

## Reports and artifacts

A reviewable evidence record contains commit pair, environment/commands/exit
statuses, source/model/config hashes, processing indices/splits, solver status,
metrics, limitations and artifact locations. Freeze a benchmark protocol before
measurement and keep earlier evidence. Small curated reference JSONs may be
tracked intentionally; large generated runs belong in durable archives or CI
artifacts. Do not force-add arbitrary ignored `results/` directories.

Proposed CI tiers and result-contract changes are described in the core plan.
Current hosted failures are tracked in PRs/issues; this guide does not claim
that branch protection is configured or that all workflows pass.
