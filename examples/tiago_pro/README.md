# Tiago Pro — right-arm geometric calibration

Migrated from the standalone `figaroh_tiagoPro` repo. Identifies per-joint
kinematic offsets for the right arm chain
`base_footprint → gripper_right_tool_holder` using external EE pose
measurements (Qualisys mocap). Goal: reduce the FK/mocap error at the
source (e.g. for `mocap_mpc_corrector.py` on the real robot).

`TiagoProCalibration` subclasses `figaroh.calibration.base_calibration.
BaseCalibration`, the same pattern the `tiago` example's `TiagoCalibration`
uses (it originally drove `figaroh.calibration.calibration_tools` directly
through a self-contained class, kept separate from `BaseCalibration` because
that was the code validated against real hardware data — migrated onto
`BaseCalibration` since, with numerical re-verification against that
reference run). See `utils/tiago_pro_tools.py` for the overrides genuinely
specific to this robot/dataset — a CSV column that can legitimately be
absent from cleaned data, and a fixed absolute-distance outlier cutoff
(position + orientation, separately) instead of `BaseCalibration`'s default
adaptive statistical threshold.

**As of 2026-08-06** the default config measures full EE pose (position +
orientation — the Qualisys rigid body already reports both) instead of
position-only, and the marker offset (`pEE`/`phiEE`) is estimated relative
to `gripper_right_pal_atc_base_link` instead of `gripper_right_tool_holder`
— stopping the FK chain *before* the never-exercised
`gripper_right_tool_mount_joint` avoids an exact regressor collinearity
between that joint and the marker-offset params (see the comments in
`config/tiago_pro_calibration_config.yaml`). This also makes `pEEx_1`/
`pEEy_1` — previously fixed at 0 because they were structurally
unobservable under the old position-only setup — safe to freely optimize
(re-verified by SVD; `_fixed_tip_xy` in `utils/tiago_pro_tools.py` now
defaults to `False`).

Reference run: 48 samples, position RMSE 8.99 mm / MAE 8.25 mm, orientation
RMSE 2.08 deg / MAE 1.88 deg — see `data/calibration_results_20260805_1246.yaml`
for the upstream figures this was validated against (7.93 mm / 7.28 mm
position, 2.18 / 1.89 deg orientation): close but not bit-identical, same
pattern as the `BaseCalibration` migration below.

The earlier position-only reference run (94 samples, RMSE 6.46 mm, MAE
4.94/4.95 mm — `data/calibration_results_20260702_0756.yaml`) is kept as a
**frozen historical record only**: `data/calibration_samples_2026070*.csv`
have no orientation columns, so they're no longer compatible with the
current default config (which requires full-pose measurements). Reverting
`config/tiago_pro_calibration_config.yaml`'s `tool_frame`/`markers.measure`/
`tip_pose` to the position-only values in that frozen YAML's neighboring
git history would make them runnable again if needed.

`BaseCalibration` migration note: reproduces the frozen pre-migration
position-only reference (94 samples, RMSE 6.46 mm, MAE 4.94 mm) closely
(RMSE 6.46 mm, MAE 4.95 mm) but not bit-identically, since
`BaseCalibration`'s inherited outlier-removal loop calls the LM solver with
slightly different options than the original bespoke loop did — the same
kind of small solver-path difference shows up again in the orientation
upgrade above.

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
running, recording the full 6DOF `tiago_endEffector` rigid body pose, not
just position). `data/calibration_samples_20260805_1246.csv` is a real
log captured this way, included so calibration can be run and checked
without hardware. (`data/calibration_samples_2026070*.csv` are earlier,
position-only logs — see the "frozen historical record" note above.)

### Step 5 — Run calibration (host)

```bash
python3 calibration.py \
  --urdf urdf/tiago_pro.urdf \
  --data data/calibration_samples_20260805_1246.csv
```

Outputs identified marker pose + per-joint offsets (position RMSE/MAE in
mm, orientation RMSE/MAE in deg) to `data/calibration_results.yaml`.

By default (same reporting format as the `tiago` example — see
`figaroh.tools.report`/`provenance`/`run_archive`) each run also:

- writes a self-contained HTML diagnostic report (condition number,
  per-DOF residuals, parameter uncertainty, correlation, auto-generated
  insights) to `results/runs/<asset>/calibration/<timestamp>_<git-sha>/report.html`
- archives `provenance.json` (software versions, git commit, input file
  hashes, timestamps), `config.snapshot.yaml`, and `parameters.csv`
  alongside it — a run directory is never overwritten
- appends a one-line summary to `results/runs/index.jsonl`

`TiagoProCalibration` is a `BaseCalibration` subclass (see note above), so
all of this — condition number, per-DOF stats, parameter uncertainty and
correlation, provenance/archive — comes directly from `BaseCalibration`;
`calibration.py` only calls `initialize()`/`solve()`/`export_html_report()`,
same as the `tiago` example's driver.

```bash
# Attach a physical-unit identity to the archive path + provenance record
python3 calibration.py --data data/calibration_samples_20260805_1246.csv \
  --asset-id TIAGO-PRO-1 --operator "Jane Doe"

# Skip reporting/archiving (just the results YAML)
python3 calibration.py --data data/calibration_samples_20260805_1246.csv \
  --no-html-report --no-archive
```

### Step 6 — Apply on robot

Not part of this example. `data/calibration_reliability_20260805.md` is a
worked example of deciding *which* identified parameters are worth
deploying — a per-parameter statistical significance table (standard
error from the residual Jacobian covariance) plus a held-out train/test
validation showing that a reduced "conservative" subset (only ≥2σ
parameters) generalizes ~1.6x better on unseen poses than the full fit —
don't deploy noise-level parameters as if they were real mechanical
corrections. The same σ-significance numbers are also visible per-run in
`results/runs/.../report.html`'s "Parameter uncertainty" section.
See `data/mocap_frame_correction_20260805.md`,
`data/master_calibration_20260805.yaml` (all parameters) and
`data/master_calibration_20260805_conservative.yaml` (≥2σ subset only)
for how the reference run's results were translated into a PAL
`master_calibration.yaml` deployment file, and
`urdf/calibration_offset.urdf.xacro` for the PAL calibration xacro
format. (`data/calibration_application_20260702.md` and
`data/master_calibration_20260702.yaml` are the equivalent artifacts for
the earlier, position-only reference run.)

---

## Identified parameters

| Parameter | Description |
|---|---|
| `base_p{x,y,z}_torso`, `base_phi{x,y,z}_torso` | Base-marker↔robot transform, merged with torso_lift_joint's own DH offsets (m, rad) |
| `d_p{x,y,z}_arm_right_N_joint`, `d_phi{x,y,z}_arm_right_N_joint` | Per-joint DH offset, torso + arm_right_1..7 (m, rad) |
| `pEEx_1, pEEy_1, pEEz_1` | Mocap marker position rel. to `tool_frame` (`gripper_right_pal_atc_base_link`) (m) |
| `phiEEx_1, phiEEy_1, phiEEz_1` | Mocap marker orientation rel. to `tool_frame` (rad) |

---

## Files

| File | Description |
|---|---|
| `calibration.py` | Thin CLI entry point (argparse/main only) |
| `utils/tiago_pro_tools.py` | `TiagoProCalibration(BaseCalibration)` + `write_calibration_results` — robot/dataset-specific overrides |
| `generate_urdf.py` | Post-process a container-generated URDF → portable |
| `view_robot.py` | Visualize robot in Viser, with per-DOF sliders and frame highlighting |
| `generate_optimal_configs.py` | D-optimal config selection |
| `visualize_optimal_configs.py` | Replay a saved `data/optimal_configs.yaml` in Viser without regenerating it |
| `config/tiago_pro_calibration_config.yaml` | Figaroh calibration config |
| `urdf/tiago_pro.urdf` | Portable URDF (used by scripts) |
| `urdf/tiago_pro_container.urdf` | Raw URDF as generated on the robot |
| `urdf/tiago_pro.srdf` | Collision pairs |
| `urdf/calibration_offset.urdf.xacro` | PAL calibration xacro template/reference |
| `tiago_pro_description.repos` | VCS repos for the full upstream mesh packages (optional; a curated mesh subset is already in `../../models/`) |
| `data/calibration_samples_20260805_1246.csv` | Real calibration data log, full 6DOF pose (current default) |
| `data/calibration_samples_2026070*.csv` | Earlier, position-only logs (frozen historical reference — see note above) |
| `data/calibration_results_*.yaml` | Reference calibration results for the logs above |
| `data/calibration_reliability_20260805.md` | Per-parameter statistical significance + held-out train/test validation study |
| `data/optimal_configs.yaml` | Pre-generated D-optimal configuration set |
| `data/mocap_frame_correction_20260805.md`, `data/master_calibration_20260805*.yaml` | Deployment provenance for the current reference run (`_conservative` = ≥2σ subset only) |
| `data/master_calibration_20260702.yaml`, `data/calibration_application_20260702.md` | Deployment provenance for the earlier, position-only reference run |
| `results/runs/<asset>/calibration/<timestamp>/` | Per-run archive: `report.html`, `provenance.json`, `config.snapshot.yaml`, `parameters.csv` |
| `results/runs/index.jsonl` | Append-only one-line-per-run summary index |
