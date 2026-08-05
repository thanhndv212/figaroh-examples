# Tiago Pro — right-arm geometric calibration

Migrated from the standalone `figaroh_tiagoPro` repo. Identifies per-joint
kinematic offsets for the right arm chain
`base_footprint → gripper_right_tool_holder` using external EE pose
measurements (Qualisys mocap). Goal: reduce the FK/mocap error at the
source (e.g. for `mocap_mpc_corrector.py` on the real robot).

Unlike the other examples in this repo, this one drives
`figaroh.calibration.calibration_tools` directly through a small
self-contained `TiagoProCalibration` class rather than subclassing
`BaseCalibration` — that is the code that was validated against real
hardware data, so the port keeps it as-is rather than risking a rewrite.
See `data/calibration_results_20260702_0756.yaml` for the reference run
(94 samples, RMSE 6.46 mm, MAE 4.94 mm).

---

## Setup

Run from the `figaroh-dev` conda environment (see the repo-level
`figaroh-examples-workflow` docs):

```bash
conda activate figaroh-dev
cd examples/tiago_pro
```

### Visual meshes

The URDF references mesh packages from PAL/gepetto's external
description repos, pinned in `tiago_pro_description.repos`. **Calibration
does not need them** (Pinocchio's `buildModelFromUrdf` only parses the
kinematic tree, not geometry) — only `view_robot.py` and the Viser
visualization in `generate_optimal_configs.py` use them, both of which
degrade gracefully (frame axes only, no meshes) if a mesh is missing.

Rather than vendoring the full upstream packages, only the mesh files
actually referenced by `urdf/tiago_pro.urdf` were copied into the
shared `../../models/<package>_description/` directories (same
convention as the other examples — resolved via `package_dirs`). All 30
referenced meshes across 8 packages are present — `view_robot.py` and
`generate_optimal_configs.py` load full visual + collision geometry
(verified: 737 active collision pairs) with no missing-mesh fallback.
See each package's `SOURCE.md` for the upstream repo, pinned version,
and license (all Apache-2.0). To pull the full packages instead (e.g.
for simulation/collision configs beyond meshes), use
`tiago_pro_description.repos`:

```bash
mkdir -p /tmp/tiago_pro_full_description
vcs import /tmp/tiago_pro_full_description < tiago_pro_description.repos
```

---

## Workflow

### Step 1 — Generate the URDF (on the real robot / container)

```bash
ros2 run xacro xacro \
  $(ros2 pkg prefix tiago_pro_description)/share/tiago_pro_description/robots/tiago_pro.urdf.xacro \
  end_effector_right:=pal-atc \
  end_effector_left:=pal-pro-gripper \
  wrist_model_right:=spherical-wrist \
  > urdf/tiago_pro_container.urdf
```

Then post-process to make mesh paths portable:

```bash
python3 generate_urdf.py   # urdf/tiago_pro_container.urdf → urdf/tiago_pro.urdf
```

`urdf/tiago_pro.urdf` (already portable) and `urdf/tiago_pro_container.urdf`
(raw, as generated on the robot) are both included so the example runs
out of the box without needing to repeat this step.

### Step 2 — Visualize the robot (host)

```bash
python3 view_robot.py
```

Opens Viser at http://localhost:8080.

### Step 3 — Generate optimal calibration configurations (host)

```bash
python3 generate_optimal_configs.py
```

- Generates a pool of random collision-free configurations
- Selects the D-optimal subset (maximizes information for parameter identification)
- Visualizes each config in Viser (Enter = next, q = quit)
- Saves to `data/optimal_configs.yaml`

Options:
```bash
python3 generate_optimal_configs.py --pool-size 500 --no-viser
```

### Step 4 — Collect calibration data (on the real robot)

Not part of this example (requires the physical robot + Qualisys mocap
running). `data/calibration_samples_20260701_0913.csv` and
`data/calibration_samples_20260702_0756.csv` are real logs captured this
way, included so calibration can be run and checked without hardware.

### Step 5 — Run calibration (host)

```bash
python3 calibration.py \
  --urdf urdf/tiago_pro.urdf \
  --data data/calibration_samples_20260702_0756.csv
```

Outputs identified joint offsets + marker position (RMSE/MAE in mm) to
`data/calibration_results.yaml`.

By default (same reporting format as the `tiago` example — see
`figaroh.tools.report`/`provenance`/`run_archive`) each run also:

- writes a self-contained HTML diagnostic report (condition number,
  per-DOF residuals, parameter uncertainty, correlation, auto-generated
  insights) to `results/runs/<asset>/calibration/<timestamp>_<git-sha>/report.html`
- archives `provenance.json` (software versions, git commit, input file
  hashes, timestamps), `config.snapshot.yaml`, and `parameters.csv`
  alongside it — a run directory is never overwritten
- appends a one-line summary to `results/runs/index.jsonl`

`TiagoProCalibration` predates `figaroh.calibration.base_calibration`
and isn't a `BaseCalibration` subclass (see note above), so
`calibration.py` computes the same quality metrics
(`_compute_condition_number`/`_compute_per_dof_stats`/
`_compute_uncertainty`, mirroring `BaseCalibration`'s formulas) and
attaches them to the calibration object before handing it to figaroh's
actual report/provenance/archive functions — same output format, no
duplicated rendering code.

```bash
# Attach a physical-unit identity to the archive path + provenance record
python3 calibration.py --data data/calibration_samples_20260702_0756.csv \
  --asset-id TIAGO-PRO-1 --operator "Jane Doe"

# Skip reporting/archiving (just the results YAML)
python3 calibration.py --data data/calibration_samples_20260702_0756.csv \
  --no-html-report --no-archive
```

### Step 6 — Apply on robot

Not part of this example — see `data/calibration_application_20260702.md`
and `data/master_calibration_20260702.yaml` for how the reference run's
results were translated into a PAL `master_calibration.yaml` deployment
file, and `urdf/calibration_offset.urdf.xacro` for the PAL calibration
xacro format.

---

## Identified parameters

| Parameter | Description |
|---|---|
| `offsetRZ_arm_right_1..7_joint` | Per-joint angle offset (rad) |
| `offsetPZ_torso_lift_joint` | Torso linear offset (m) |
| `pEEx_1, pEEy_1, pEEz_1` | Mocap marker position rel. to `gripper_right_tool_holder` (m) |

---

## Files

| File | Description |
|---|---|
| `calibration.py` | Figaroh LM calibration (main entry point) |
| `generate_urdf.py` | Post-process a container-generated URDF → portable |
| `view_robot.py` | Visualize robot in Viser |
| `generate_optimal_configs.py` | D-optimal config selection |
| `config/tiago_pro_calibration_config.yaml` | Figaroh calibration config |
| `urdf/tiago_pro.urdf` | Portable URDF (used by scripts) |
| `urdf/tiago_pro_container.urdf` | Raw URDF as generated on the robot |
| `urdf/tiago_pro.srdf` | Collision pairs |
| `urdf/calibration_offset.urdf.xacro` | PAL calibration xacro template/reference |
| `tiago_pro_description.repos` | VCS repos for the full upstream mesh packages (optional; a curated mesh subset is already in `../../models/`) |
| `data/calibration_samples_*.csv` | Real calibration data logs |
| `data/calibration_results_*.yaml` | Reference calibration results for the above logs |
| `data/optimal_configs.yaml` | Pre-generated D-optimal configuration set |
| `data/master_calibration_20260702.yaml`, `data/calibration_application_20260702.md` | Deployment provenance for the reference run |
| `results/runs/<asset>/calibration/<timestamp>/` | Per-run archive: `report.html`, `provenance.json`, `config.snapshot.yaml`, `parameters.csv` |
| `results/runs/index.jsonl` | Append-only one-line-per-run summary index |
