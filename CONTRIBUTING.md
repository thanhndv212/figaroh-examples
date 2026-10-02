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

Record the exact core/examples commits validated together. Preserve unrelated
changes and existing branches. For a clean environment, reproduce geometry
resolution using documented/pinned fixture sources. A developer's external
`ROS_PACKAGE_PATH` is not sufficient CI evidence. Do not copy third-party
assets without checking their redistribution terms.

## Planning and PR workflow

1. Discuss a proposed change before implementation when its method, contract
   or scope is unresolved. A plan approval, implementation instruction and
   merge approval are distinct actions.
2. Reuse an existing issue; otherwise create one only when ready, with the
   *Feature / work item* or *Bug report* template (shared with figaroh-plus;
   see [how to read delivery issues](https://github.com/thanhndv212/figaroh-plus/blob/devel/docs/plans/README.md#how-to-read-delivery-issues)).
   Refer to issues as `figaroh-plus#N` / `figaroh-examples#N`. Specify robot,
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
python -m pytest tests/ -v
python validate.py
pre-commit run --files <changed-files>
git diff --check
```

Full `validate.py` is required for implementation phases; `--quick`,
`--tests-only`, `--scripts-only` and `--robot` help diagnosis but do not replace
full final evidence. Run affected robot commands from their robot folder with
`MPLBACKEND=Agg`. Document timeouts and missing assets separately from wrong
numerical results. Documentation-only discussion changes use hooks/diff/link
checks; do not rerun long optimization merely to validate prose.

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
