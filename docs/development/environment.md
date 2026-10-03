# Paired examples environment

Example results depend on the exact figaroh-plus (core) and figaroh-examples
commits, the native robotics wheels and the fetched meshes. This page gives the
recipe that reproduces the hosted CI environment on a fresh machine, and keeps
a record of where it has been verified. Hosted CI evidence and local evidence
are recorded separately, because neither substitutes for the other: CI runs
`pytest` only, on Linux, and local runs also execute the example scripts.

## Sources of truth

| Part | Defined in |
|---|---|
| Conda base (Python 3.12, `cyipopt`) | figaroh-plus `environment.yml`, without its `pip:` section |
| Pinocchio and its native wheel stack | figaroh-plus `ci/pinocchio-3.7.0.txt` or `ci/pinocchio-4.1.0.txt` |
| Scientific pins for the examples | figaroh-examples `ci/constraints.txt` |
| Core install | `pip install -e '<figaroh>[dev]'` under those constraints |
| Meshes not vendored in `models/` | figaroh-examples `scripts/fetch_models.py` ([models/README.md](../../models/README.md)) |
| Hosted job | figaroh-examples `.github/workflows/ci.yml`, which uses all of the above |

The day-to-day `figaroh-dev` environment is created from the full
`environment.yml` without these constraints. It is fine for development, but
only the recipe below matches what CI tests.

## Recipe

Check out both repositories side by side, at the commits you want to pair:

```bash
git clone https://github.com/thanhndv212/figaroh-plus.git figaroh
git clone https://github.com/thanhndv212/figaroh-examples.git
WS="$PWD"; PIN=3.7.0          # or 4.1.0
```

Build the environment (pick a name per profile):

```bash
cat "$WS/figaroh/ci/pinocchio-$PIN.txt" "$WS/figaroh-examples/ci/constraints.txt" > constraints.txt
sed '/  - pip:/,$d' "$WS/figaroh/environment.yml" | sed "s/^name: .*/name: figaroh-verify/" > environment.yml
conda env create -f environment.yml
conda activate figaroh-verify
PIP_CONSTRAINT="$PWD/constraints.txt" python -m pip install -e "$WS/figaroh[dev]"
```

Create one environment at a time: concurrent `conda env create` runs race on
the shared package cache.

Fetch the meshes and point Pinocchio at them:

```bash
cd "$WS/figaroh-examples"
python scripts/fetch_models.py
export ROS_PACKAGE_PATH="$(python scripts/fetch_models.py --print-ros-package-path)"
```

## Verify

```bash
python -m pip check                                   # dependency consistency
python -c "import figaroh, pinocchio, cyipopt, picos; print(figaroh.__file__, pinocchio.__version__)"
bash skills/figaroh-setup-env/scripts/doctor.sh --env figaroh-verify --ws "$WS"
python -m pytest tests/test_robot_geometry.py         # meshes resolve for every robot
python validate.py                                    # full suite + every example script
```

`figaroh.__file__` must point into `$WS/figaroh/src`. `validate.py` pins BLAS
threads itself, so its results are reproducible; unset any developer
`ROS_PACKAGE_PATH` entries first so only the fetched packages are used.

## Evidence record

Add a row when a pairing is verified. Keep commit IDs exact.

### Hosted CI

| Date | Run | figaroh-examples | figaroh-plus | Platform | Pinocchio | Result |
|---|---|---|---|---|---|---|
| 2026-10-03 | [37127637416](https://github.com/thanhndv212/figaroh-examples/actions/runs/37127637416) | `d2d8976` | `cc262d8` (`devel`) | ubuntu-latest, Python 3.12 | 3.7.0 | pytest 166 passed, 1 skipped |
| 2026-10-03 | same run | `d2d8976` | `cc262d8` | ubuntu-latest, Python 3.12 | 4.1.0 | pytest 166 passed, 1 skipped |

The skip is `test_so101_identification.py::test_deployment_round_trips_through_soarm_sdk`,
which needs the optional `soarm_sdk` package.

### Local, clean environment

Built from the recipe above in fresh conda environments; `ROS_PACKAGE_PATH`
set only to the fetched packages. figaroh-plus `e681625` is `cc262d8` plus a
docs-only commit.

| Date | figaroh-examples | figaroh-plus | Platform | Pinocchio stack | `pip check` / doctor | `validate.py` |
|---|---|---|---|---|---|---|
| 2026-10-03 | `d2d8976` | `e681625` | macOS arm64, Python 3.12.0 | pin 3.7.0, ndcurves 2.0.0.1, assimp 5.4.3.1, urdfdom 4.0.1, tinyxml2 10.0.0 | clean / Ready (viser optional, missing) | 15 passed, **1 failed**: `tiago/optimal_trajectory.py` (#60, segment 2 infeasible on this stack) |
| 2026-10-03 | `d2d8976` | `e681625` | macOS arm64, Python 3.12.0 | pin 4.1.0, ndcurves 2.3.0, assimp 6.0.5, urdfdom 6.0.0, tinyxml2 11.0.0 | clean / Ready (viser optional, missing) | 15 passed, **1 failed**: `tiago/optimal_trajectory.py` (#59, Pinocchio 4 `GeometryObject` signature) |
| 2026-10-03 | `c6a9c11` + #60 configs | `fe7dc51` | macOS arm64, Python 3.12.0 | pin 3.7.0 (as above) | clean / Ready | **16 passed, 0 failed** — `tiago/optimal_trajectory.py` 351 s: segment 2 failed once, retry succeeded (#60) |

All: numpy 2.3.2, scipy 1.16.1, cyipopt 1.7.0, picos 2.6.1, cvxopt 1.3.2.
The two failures in the first rows were fixed in #59 (Pinocchio 4 collision
primitives) and #60 (retry a failed trajectory segment, `segment_attempts: 2`).

## Known gaps

- CI does not run the example scripts, so script-level Pinocchio 4 regressions
  (such as #59) only show up in a local `validate.py` on that profile; #59 added
  a test that builds the TIAGo collision model so CI now covers that path.
- Only macOS arm64 has local script-level evidence; Linux has CI `pytest` only.
- Optional packages (`viser`, `soarm_sdk`) are not part of the recipe.
- The TIAGo trajectory optimisation is sensitive to its starting point, so
  which segments succeed differs between numerical stacks (#60). The shipped
  configs retry a failed segment once; with that, seeds 0-5 pass on both the
  recipe stack and `figaroh-dev`, but a pass in one environment is still
  verified per environment rather than assumed.
