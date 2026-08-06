# Local coordinate system correction for the `tiago_endEffector` mocap rigid body

Source: `calibration_results_20260805_1246.yaml` (48 samples, position
RMSE 7.93mm/MAE 7.28mm, orientation RMSE 2.18°/MAE 1.89°).

Goal: redefine the local origin of the QTM rigid body `tiago_endEffector` so
it directly reports the pose of `gripper_right_tool_holder` instead of the
raw marker pose.

---

## ⚠️ Per-axis reliability (standard error computed from the fit's Jacobian)

48 samples aren't enough to determine every axis with the same confidence —
some are on the same order of magnitude as their own noise:

| Parameter | Value | Standard error | value / error | Verdict |
|---|---|---|---|---|
| x (pEEx_1) | -4.91 mm | ±2.94 mm | 1.67σ | **not significant** |
| y (pEEy_1) | -1.30 mm | ±2.86 mm | 0.46σ | **pure noise** |
| z (pEEz_1) | +137.16 mm | ±2.78 mm | 49.3σ | solid |
| roll (phiEEx_1) | -0.96° | ±0.17° | 5.6σ | solid |
| pitch (phiEEy_1) | -0.29° | ±0.11° | 2.7σ | marginal, acceptable |
| yaw (phiEEz_1) | +2.47° | ±0.11° | 23.2σ | solid |

**x and y are not reliable with this dataset**: their uncertainty is on the
same order as the value itself (even larger for y). Makes physical sense —
a translation at the end of the chain (X/Y, perpendicular to the tool axis)
has a subtler effect on the measured position than an axial translation (Z)
or a yaw rotation; more pose diversity is needed to separate it from mocap
noise.

---

## Recommendation: apply only Z + the 3 rotations, x/y at 0

**Translation** (current local axes of the rigid body):

| axis | value |
|---|---|
| x | 0.024 mm *(≈0, residual of composing with the rotation)* |
| y | -0.080 mm *(same)* |
| z | **+4.7693 mm** |

**Rotation** (roll/pitch/yaw = `R = Rz(yaw)·Ry(pitch)·Rx(roll)`, standard
Pinocchio/RPY convention):

- RPY (deg): roll = **+0.9458°**, pitch = **+0.3289°**, yaw = **-2.4689°**
- Quaternion (x, y, z, w): `[0.008314, 0.002692, -0.021566, 0.999729]`
- Rotation matrix:
  ```
  [ 0.999055   0.043165   0.005024]
  [-0.043076   0.998932  -0.016739]
  [-0.005741   0.016506   0.999847]
  ```

(This rotation is identical to the full variant below — only the
translation changes, since x/y were zeroed before inversion.)

---

## Full identified value (x/y included, use only if you accept the risk
## of correcting on noise)

**Translation**:

| axis | value |
|---|---|
| x | +4.9898 mm ⚠️ 1.67σ |
| y | +1.0117 mm ⚠️ 0.46σ |
| z | +4.7627 mm |

**Rotation** — same as above: roll +0.9458°, pitch +0.3289°,
yaw -2.4689° / quaternion `[0.008314, 0.002692, -0.021566, 0.999729]`.

Numerically verified: composing `T_tool→marker · T_marker→tool` (full value)
correctly gives back the identity (translation ~0, rotation ~Identity) —
computational consistency confirmed; it's not a math error, just the lack
of data for x/y.

---

## Notes

- The "x/y at 0" recommendation isn't just theoretical: a held-out
  train/test validation (4 random splits, same conservative parameter set
  including z/phiEE but excluding x/y) showed this generalizes ~1.6x better
  on unseen poses than including every identified parameter — see the
  "Validation" section in `calibration_reliability_20260805.md`.
- Correction based on only 48 samples. To make x/y reliable: recollect
  data (the pipeline is ready:
  `generate_optimal_configs.py` → `collect_calibration_data.py` →
  `run_calibration.py`), then redo this uncertainty computation before
  deciding.
- This correction only touches the mocap body definition. The arm joint
  URDF offsets (`master_calibration_20260805.yaml`) are independent and
  must be deployed separately on the robot.
- `pEEx_1`/`pEEy_1` used to be fixed at 0 (judged structurally
  unidentifiable). That's no longer the case — the structural issue was
  fixed (see conversation history: `tool_frame` change + orientation
  measurement) — but 48 samples remain insufficient to determine them
  precisely, which is a separate issue from the structural one already
  resolved.
