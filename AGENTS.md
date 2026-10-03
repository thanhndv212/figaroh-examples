# AGENTS.md

Compact guidance for OpenCode sessions working in this repo. Read before editing.
Every line here is something an agent would likely get wrong without it.

## What this is

Example scripts and robot assets for the [FIGAROH](https://github.com/thanhndv212/figaroh-plus)
library (robot dynamics identification + geometric calibration). **Not an installable
package** — no `pyproject.toml`/`setup.py`. It is a collection of runnable scripts plus
shared `models/` (robot description packages) and a Viser-based `web-interface/`.

## Setup — conda env is `figaroh-dev`, not what `environment.yml` says

- **All code runs in the `figaroh-dev` conda env.** `conda activate figaroh-dev` first.
  The `environment.yml` in *this* repo names an env `figaroh-examples` — **ignore it**;
  that is not the env anyone uses. The real env is defined in the sibling `figaroh/` repo.
- **`figaroh` is installed editable from the sibling repo** at `../figaroh` (workspace
  layout: `figaroh-ws/{figaroh, figaroh-examples, figaroh-mujoco, ...}`). `import figaroh`
  resolves to `../figaroh/src/figaroh`. After changing core library code, re-run
  `pip install -e .` from `../figaroh` inside `figaroh-dev`.
- `cyipopt` (needed by optimal-trajectory scripts) is conda-only — another reason the env
  is mandatory. See `../figaroh/AGENTS.md` for core-library gotchas.

## Running example scripts — always `cd` into the robot folder first

- Scripts use **relative paths** (`urdf/<file>.urdf`, `config/<file>.yaml`) and
  `package_dirs="../../models"`. They only work when the working directory is
  `examples/<robot>/`:
  ```bash
  conda activate figaroh-dev
  cd examples/ur10
  python calibration.py
  ```
- The correct models path is `"../../models"` — all examples now use this (fixed in
  Phase 1 §1.7). Scripts only work when CWD is `examples/<robot>/`.
- Entry-point scripts: `calibration.py`, `identification.py`, `optimal_config.py`,
  `optimal_trajectory.py`, and `update_model.py` (materializes estimated params into a
  URDF). Not every robot has all of them.
- Robot-specific logic lives in `examples/<robot>/utils/<robot>_tools.py` as subclasses of
  `figaroh.calibration.base_calibration.BaseCalibration`,
  `figaroh.identification.base_identification.BaseIdentification`, and
  `figaroh.optimal.base_optimal_*`.

## Tests & validation — use `validate.py` after every implementation phase

- **Run `python validate.py` from repo root after every phase of implementation work.**
  This is mandatory — it runs both the pytest suite and all example scripts, then
  reports a pass/fail/timeout summary. Do not declare a phase complete until
  `validate.py` passes (or only shows known pre-existing failures).
  ```bash
  conda activate figaroh-dev
  python validate.py                  # tests + all scripts (full)
  python validate.py --quick          # skip slow scripts (optimal_config, optimal_trajectory)
  python validate.py --tests-only     # pytest only
  python validate.py --scripts-only   # example scripts only
  python validate.py --robot ur10     # tests + scripts for one robot
  ```
- `validate.py` sets `MPLBACKEND=Agg` so matplotlib doesn't block on plot windows.
- Slow scripts (optimal_config, optimal_trajectory) use IPOPT and can take 5+ minutes.
  Use `--quick` for fast feedback loops; run full validation before declaring done.
- **Exit code 0 only when every required check passes; 1 on any failure or timeout.**
  A required timeout reports `RESULT: INCOMPLETE` — it is not a pass, even when an
  IPOPT script is merely slow. Full child stdout/stderr, exit code (negative =
  killed by signal) and timeout metadata are kept under the git-ignored
  `validation_logs/`; cite them when classifying a known failure.
- Identification `--verify` checks the **execution** scope by default (finite,
  consistent fit outputs). That PASS is not prediction, physical-model or export
  acceptance — see "Acceptance policy" in `README.md`.
- `pytest` can also be run directly: `pytest tests/ -v` from repo root.
- `tests/conftest.py` adds `../figaroh/src` **and** the examples root to `sys.path`, so
  tests assume the sibling `figaroh/` repo is checked out next to this one.
- Markers: `slow`, `integration`, `real_config`. Skip slow: `pytest -m 'not slow'`.
- **Do not use `tests/run_tests.py`** — it only runs 3 of the 8 test files and parses
  stdout instead of using the pytest API. It is stale. Use `validate.py` instead.

## Config — two formats coexist; scripts use the unified one

- **Unified** (`*_unified_config.yaml`): `tasks.<task>.*` layout with template inheritance
  via `extends:` (e.g. `extends: "../../templates/manipulator_robot.yaml"`). This is what
  entry-point scripts load.
- **Legacy** (`*_config.yaml`): flat format, auto-detected by `figaroh`'s
  `UnifiedConfigParser`. Keep for backward-compat tests; don't write new configs in it.
- Templates live in `examples/templates/` (`base_robot_config.yaml`,
  `manipulator_robot.yaml`, `humanoid_robot.yaml`). Use `extends:` (not `inherit_from`).
  Keep robot specifics under `robot.properties.*`, task specifics under `tasks.<task>.*`.
- TIAGo config files were consolidated in Phase 4 — broken/unused configs moved to
  `config/archive/`, 3 active configs remain. See `config/README.md` for details.
  `extends` is now enabled in `tiago_unified_config.yaml`.

## Gotchas

- **`examples/shared/` does not exist and is gitignored** (`.gitignore`: `shared/`,
  `examples/shared/`). `examples/__init__.py` still does `from .shared import (...)`
  inside a `try/except`, so it silently no-ops. **Do not import or rely on
  `examples.shared.*`.** (Staubli's dead `ConfigManager` import was removed in Phase 1.)
- **Logging is now `WARNING` by default** (changed from `CRITICAL` in Phase 3). Use
  `--verbose` / `-v` flag on any script to enable `INFO`-level logging.
- **All entry-point scripts have argparse** with `--config`, `--urdf`, `--verbose` flags
  (Phase 3). Run `python <script> --help` to see available options.
- **`create_example.sh` bugs fixed** in Phase 6: title-case now preserves acronyms
  (UR10 stays UR10, not Ur10), replace order fixed, temp file cleanup via trap.
- **`.github/workflows/` is tracked** (CI workflow added in Phase 6). The rest of
  `.github/` remains gitignored (local skills, etc.).
- **CI runs pytest on push/PR** via GitHub Actions (Phase 6 §6.1). The workflow
  checks out both `figaroh-examples` and `figaroh-plus` repos, sets up conda
  `figaroh-dev`, and runs `pytest tests/ -v`.

## Web interface

- Viser-based, under active development (expect breaking changes).
- Run from `web-interface/`: `python main.py` (default `http://localhost:8080`).
  Flags: `--port`, `--host`, `--examples-path`, `--models-path`, `--debug`, `--classic`.
- Config discovery fixed in Phase 6: `core/example_loader.py` now looks for
  `config/*_unified_config.yaml` first, then falls back to legacy filenames.

## Branches

- `main` is the default/release branch (currently checked out). `devel` is the dev branch
  — matches the core `figaroh/` repo convention. Other feature branches (`h1v2`, `soarm`,
  `0a`) exist locally.

## Known issues — read `IMPROVEMENT_PLAN.md`

`IMPROVEMENT_PLAN.md` is a 39-item audit (P0–P6) of this repo's current state.
**Phases 1-6 are complete** (39/39 items fixed: failing tests, path bugs, copy-paste
typos, dead code, standardized practices, config cleanup, infrastructure, URDF
deduplication, smoke tests). Only Phase 7 remains:
- **Phase 7** (P6): complete incomplete examples — add missing scripts to TALOS and
  Staubli TX40 (new feature work, deferred)

Run `python validate.py` to check current state. All 9 scripts pass, 3 skip (IPOPT).

## CodeGraph

A `.codegraph/` index exists at the repo root. Prefer `codegraph_explore` / `codegraph_node`
(see `~/.config/opencode/AGENTS.md`) over grep/Read for locating symbols and understanding
how example utils wire into `figaroh` base classes.
