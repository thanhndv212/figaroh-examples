# TIAGo Pro — right-arm geometric calibration

Contributed by [Clement Pene](https://github.com/clementPene).

Identifies full kinematic parameters for the chain `base_footprint → gripper_right_pal_atc_base_link`
(torso + arm_right_1..7 DH offsets, base-frame pose, marker mount) from external
end-effector pose measurements (Qualisys mocap or any other 6DOF source), using
Figaroh's `BaseCalibration` (SE3 log-map residuals, per-parameter standard
errors, outlier removal, condition-number/correlation reporting). Results are
written to a results YAML and to `calibration_offset.urdf.xacro`, the format
used by the PAL calibration system.

**Goal**: reduce the FK/mocap error at the source, so that a downstream mocap-based
corrector applies a near-zero correction instead of fighting a large, systematic
kinematic error.

Reference run: 48 samples, position RMSE 7.11&nbsp;mm / MAE 6.55&nbsp;mm, orientation
RMSE 1.92° / MAE 1.67° — see `data/calibration_results_20260805_1246.yaml` and
`data/calibration_reliability_20260805.md` for the reliability study behind
these numbers (split test, mocap-frame correction).

---

## Setup

### 1. Robot description packages (meshes)

Meshes are already vendored under `models/` at the repo root (see the top-level
README). If you need to refresh them from their sources:

```bash
vcs import < examples/tiago_pro/tiago_pro_description.repos
```

### 2. Install dependencies

From the repo root:

```bash
pip install figaroh
pip install -r requirements.txt
```

---

## Workflow

### Step 1 — Generate the URDF

The URDF is generated from `tiago_pro_description`'s xacro on the robot/container
(where the ROS packages are installed), then post-processed here to make mesh
paths portable (`package://<pkg>/...` instead of absolute container paths):

```bash
# On the robot/container, with xacro available:
ros2 run xacro xacro \
  $(ros2 pkg prefix tiago_pro_description)/share/tiago_pro_description/robots/tiago_pro.urdf.xacro \
  end_effector_right:=pal-atc \
  end_effector_left:=pal-pro-gripper \
  wrist_model_right:=spherical-wrist \
  > tiago_pro.urdf

# Then, from examples/tiago_pro/:
python3 generate_urdf.py --input tiago_pro.urdf --output tiago_pro_local.urdf
```

`tiago_pro.urdf`/`tiago_pro_local.urdf` here are already-generated snapshots,
checked in so the example runs standalone.

### Step 2 — Visualize the robot

```bash
python3 view_robot.py
```

Opens a Viser viewer at http://localhost:8080.

### Step 3 — Generate optimal calibration configurations

```bash
python3 generate_optimal_configs.py --pool-size 500
```

Generates a pool of random collision-free configurations, selects the
D-optimal subset (maximizes information for parameter identification), and
saves it to `data/optimal_configs.yaml`.

### Step 4 — Collect calibration data (on the robot)

Not part of this repo — drive the robot through each configuration in
`data/optimal_configs.yaml`, record `(q, T_mocap_EE_in_base)` pairs, and save
them as a CSV with columns matching `tiago_pro_calibration_config.yaml`'s
joint list plus `x1,y1,z1,phix1,phiy1,phiz1`. See
`data/calibration_samples_20260805_1246.csv` for the expected format.

### Step 5 — Run calibration

```bash
python3 run_calibration.py \
  --urdf tiago_pro_local.urdf \
  --data data/calibration_samples_20260805_1246.csv \
  --output data/calibration_results.yaml
```

Outputs:
- Identified base-frame pose, per-joint DH offsets, and marker position
  (RMSE/MAE in mm and degrees), with per-parameter standard errors and
  condition-number/correlation diagnostics.
- `calibration_offset.urdf.xacro`-format offsets, ready to deploy on the
  PAL calibration system.

### Step 6 — Apply on robot

Copy the generated xacro to the robot and restart `robot_state_publisher`
per your PAL calibration deployment procedure.

---

## Identified parameters

| Parameter | Description |
|---|---|
| `d_p{x,y,z}_<joint>`, `d_phi{x,y,z}_<joint>` | Per-joint DH-style offset (m / rad), torso_lift + arm_right_1..7 |
| `base_p{x,y,z}`, `base_phi{x,y,z}` | Mocap-frame → `base_footprint` transform (co-estimated, `known_baseframe: False`) |
| `pEE{x,y,z}_1`, `phiEE{x,y,z}_1` | Marker pose relative to `gripper_right_pal_atc_base_link` (co-estimated, `known_tipframe: False`) |

See the comments in `tiago_pro_calibration_config.yaml` for why the chain
stops at `gripper_right_pal_atc_base_link` rather than `gripper_right_tool_holder`.

---

## Files

| File | Description |
|---|---|
| `generate_urdf.py` | Post-process a container-generated URDF → portable (`package://`) paths |
| `view_robot.py` | Visualize the robot in Viser |
| `generate_optimal_configs.py` | D-optimal calibration-configuration selection |
| `run_calibration.py` | Figaroh `BaseCalibration`-based LM calibration |
| `tiago_pro_calibration_config.yaml` | Figaroh calibration config |
| `tiago_pro.srdf` | Collision pairs |
| `tiago_pro_description.repos` | VCS repos for the robot meshes under `models/` |
| `data/calibration_reliability_20260805.md` | Split-test reliability study behind the reference run |
| `data/mocap_frame_correction_20260805.md` | Notes on the mocap frame / tool_frame correction |
