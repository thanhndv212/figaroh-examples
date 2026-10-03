---
name: figaroh-setup-optimal
description: 'Set up and run FIGAROH optimal experiment design — generating well-conditioned measurement configurations for calibration and exciting trajectories for identification. Use when a calibration or identification is badly conditioned, before collecting data on a new robot, or when the user asks about condition number, dexterity, exciting trajectories, or cyipopt/IPOPT.'
---

# Set Up an Optimal Experiment Design (OED) Task

_Arguments: robot name, and which you need — measurement configurations (calibration)
or an exciting trajectory (identification)._

> **Paths.** `$FIGAROH_WS` is the workspace root — the directory holding the
> `figaroh` and `figaroh-examples` repos side by side. Set it once per shell:
> `export FIGAROH_WS="$(cd /path/to/your/workspace && pwd)"`.

**Prerequisites:** `figaroh-setup-env` exits 0 **and `cyipopt` imports.** Optimal
trajectory generation is the one task that hard-requires it:

```bash
conda run -n figaroh-dev python -c "import cyipopt; print('ok')" \
  || conda install -n figaroh-dev -c conda-forge cyipopt
```

`cyipopt` is conda-only — it is not pip-installable in practice, and it is the entire
reason `figaroh-dev` exists as a conda env.

**Working directory for every command below:**
`$FIGAROH_WS/figaroh-examples/examples/<robot>/`.

## Run this *before* collecting data

OED is upstream of the other two tasks. Both a huge calibration residual spread and a
huge identification condition number are usually **data problems**, and the fix is a
better experiment, not a different solver. Sequence:

```
figaroh-setup-optimal  →  collect data on the real robot  →  calibration / identification
```

## The two sub-tasks

| Sub-task | Script | Produces | Feeds |
|---|---|---|---|
| **Optimal configurations** | `optimal_config.py` | A set of static joint configurations that maximise information for geometric calibration | `tasks.calibration.data.sample_configurations_file` |
| **Optimal trajectory** | `optimal_trajectory.py` | A continuous exciting trajectory (cubic-spline waypoints) for dynamic identification | The motion you execute to log `q`/`tau` |

Availability today: `ur10` and `tiago` have both. `tiago_pro` has
`generate_optimal_configs.py` (a variant entry point). Others have neither — model new
ones on `examples/ur10/`.

## Step 1 — Configure optimal configurations

```yaml
tasks:
  optimal_configuration:
    enabled: true

    parameters:
      number_of_samples: 50
      optimization_objectives: ["condition_number", "dexterity"]

    constraints:
      joint_limit_margin: 0.1       # rad kept away from each limit
      collision_checking: false     # true for anything self-collision-prone

    output:
      save_configurations: true
      output_file: "data/optimal_configs/<robot>_optimal_configs.yaml"
      enable_visualization: true
```

- `number_of_samples` is the count you will physically measure. More is better
  statistically and worse operationally — 30–60 is the usual range for a manipulator.
- **`collision_checking: false` is only safe for open workspaces.** UR10's config sets
  it false because a UR10 in free space rarely self-collides. Any humanoid, mobile
  manipulator, or robot near fixtures should set it `true`.
- `joint_limit_margin` keeps the optimiser from proposing poses the controller will
  refuse at the limit.

## Step 2 — Configure the optimal trajectory

```yaml
tasks:
  optimal_trajectory:
    enabled: true

    problem:
      soft_lim: 0.1          # joint-limit discount
      max_attempts: 500      # feasibility retries before giving up
      max_iterations: 100    # IPOPT iterations per segment (default 200)
      segment_attempts: 2    # retry a failed segment from a new start (default 1)

    trajectory:
      waypoints: 7           # cubic-spline waypoints
      frequency: 20          # Hz — spline sampling for the regressor and output
      segment_duration: 2.0  # seconds between waypoints

    constraints:
      velocity_scaling: 0.5      # fraction of the URDF/config joint limits
      acceleration_scaling: 0.5

    output:
      save_trajectory: true
      output_file: "data/trajectories/<robot>_optimal_trajectory.yaml"
```

- `frequency` sets how densely the spline is sampled: every objective evaluation
  builds the regressor from those samples, so cost grows linearly with it. The
  waypoints, and so the motion itself, do not depend on it. The shipped UR10 and
  TIAGo configs use 20 Hz to stay within the `validate.py` budget. Before executing
  the trajectory, resample the spline at the rate the robot actually logs at; that
  rate is what `identification.signal_processing.sampling_frequency` must match, or
  identification will derive wrong velocities from correct data.
- The objective (condition number of the base regressor) is non-smooth, so IPOPT
  usually stops at `max_iterations` on a plateau rather than converging. A segment
  is kept when its final iterate satisfies every constraint; it is logged as a
  warning and recorded with `converged: False`. An infeasible segment fails the run.
- More `waypoints` and longer `segment_duration` excite more but take much longer to
  solve and to execute.
- `velocity_scaling`/`acceleration_scaling` at 0.5 is conservative. Raising them
  excites inertial terms more strongly — that is exactly what identification needs —
  but is also where you hit real hardware limits. Raise deliberately.
- `max_attempts: 500` bounds the search for a feasible initial guess. Waypoint steps
  are clamped to what the velocity limits allow, so the first attempt usually
  succeeds; exhausting it logs a warning and IPOPT starts from an infeasible guess.
- The example scripts take `--seed` (default 0). Whether a segment ends feasible
  depends on its starting point, so on the seed and even on the numerical stack: with
  a single attempt, two of six TIAGo seeds failed in the CI-recipe environment and one
  in `figaroh-dev`. `segment_attempts: 2` retries a failed segment from a new random
  start; with it, seeds 0-5 passed on both stacks (#60).
- A seed reproduces a run only with single-threaded BLAS. Threaded reductions change
  the last bits, and the optimizer amplifies them into different trajectories (TIAGo
  seed 0 gave a different result on every default-threaded run). `validate.py` and CI
  pin the threads; to reproduce a run by hand, use
  `OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1`.

## Step 3 — Run

```bash
conda activate figaroh-dev
cd $FIGAROH_WS/figaroh-examples/examples/<robot>

python optimal_config.py            # measurement configurations for calibration
python optimal_trajectory.py        # exciting trajectory for identification
```

Both accept `--config`, `--urdf`, and `--verbose`/`-v` only — everything else lives in
the YAML. Both exit(1) with a clear message if the URDF or config path is missing.

**Budget time for `optimal_trajectory.py`, not for `optimal_config.py`.** Both are
marked `is_slow` with a 600 s timeout in `validate.py`, and `--quick` skips both, but
they are not comparable in practice: `optimal_trajectory.py` runs IPOPT and takes
minutes, while `optimal_config.py` on UR10 (500 candidates) finished in ~35 s. In
`validate.py` output, an IPOPT **timeout is reported separately from a failure and is
expected** — do not treat it as a regression.

With `sample_configurations_file` empty, `optimal_config.py` prints `Generating random
configurations instead`. That is the normal path when you have no candidate pool — a
fallback notice, not an error. Set the key to `""`, never to a bare `None`: YAML parses
`None` as the *string* "None", which is then treated as a filename and produces a
confusing `Unsupported file format: None`.

## Step 4 — Use the output

`optimal_config.py` writes a timestamped triple into the directory named by
`optimal_configuration.output.output_file` (only the directory part is used — the
filenames are managed), falling back to `results/` when that is unset:

```
<dir>/<robot>_optimal_calibration_<timestamp>.yaml            # the configurations
<dir>/<robot>_optimal_calibration_<timestamp>.csv             # one-row summary
<dir>/<robot>_optimal_calibration_<timestamp>_metadata.yaml
```

The script prints the directory it used; point the calibration config at the YAML:

```yaml
tasks:
  calibration:
    data:
      sample_configurations_file: "data/optimal_configs/<robot>_optimal_calibration_<timestamp>.yaml"
```

**The selected count is emergent, not requested.** Selection is threshold-based on the
SOCP weights, so you get however many configurations clear the threshold — a UR10 run
selected 70 of 500 candidates. There is no key that fixes the count: the
`optimal_configuration.parameters` block is inert and the shipped configs comment it
out. The summary CSV's `configuration_count` and the count printed to stdout now agree;
if they ever diverge, trust the YAML's contents.

A hand-maintained candidate pool works too, and is what TIAGo uses — see
`data/calibration/optimal_configurations/tiago_calibration_joint_configurations_500_pmb2_hey5.yaml`.

**Trajectory → identification.** Execute it on the robot, log `q` and `tau` at the
configured `frequency`, and write them into `data/` in the layout that robot's
`load_trajectory_data()` expects (see `figaroh-setup-identification`, Step 2).

## Step 5 — Verify the design actually helped

The point of OED is a number, so check the number:

```bash
python identification.py --verify --html-report   # condition number improved?
python calibration.py --calibrate-only --no-plot  # residual spread tighter?
```

If the condition number did not improve, the trajectory is not more exciting — raise
`velocity_scaling`/`acceleration_scaling` or `waypoints` rather than re-running the
same design.

## Failure → cause → fix

| Symptom | Cause | Fix |
|---|---|---|
| `ModuleNotFoundError: cyipopt` | pip install attempted, or wrong env | `conda install -n figaroh-dev -c conda-forge cyipopt` |
| `get_backend("pinocchio")` → `ValueError: not available` | `backends/` has no Pinocchio backend; Pinocchio is used directly, not via the backend abstraction | Do not route through `get_backend` — this is a known `ARCHITECTURE.md` inaccuracy |
| "Failed to generate optimal trajectory" | Infeasible constraints | Lower `velocity_scaling`/`acceleration_scaling`, raise `soft_lim`, reduce `waypoints`, raise `max_attempts` |
| Runs for 10+ minutes with no output | Normal for IPOPT | Use `-v` for progress; budget 600 s; `validate.py --quick` to skip |
| Timeout in `validate.py` | Expected for IPOPT-heavy scripts | Reported separately from failures — not a bug |
| Configurations collide on the real robot | `collision_checking: false` | Set it `true` and regenerate |
| Configurations rejected by the controller at limits | `joint_limit_margin` too small | Raise it and regenerate |

Anything else: Re-run with `--verbose`/`-v` for INFO logging and work from the traceback; each repo's `AGENTS.md` lists the known gotchas.

## Related

`figaroh-setup-calibration` and `figaroh-setup-identification` consume this skill's
output. Speeding up the OED solve, and new objectives or solvers, belong in
`figaroh/src/figaroh/optimal/`.
