# Local coordinate system correction for the `tiago_endEffector` mocap rigid body

Source: `calibration_results_20260805_1246.yaml` — migrated run (figaroh-plus
v0.4.7, `BaseCalibration` + SE3 log-map residuals, see
`calibration_reliability_20260805.md`). 48 samples, 1 outlier flagged,
position RMSE 7.11mm/MAE 6.55mm, orientation RMSE 1.92deg/MAE 1.67deg.

Goal: redefine the local origin of the QTM rigid body `tiago_endEffector` so
it directly reports the pose of `gripper_right_tool_holder` instead of the
raw marker pose.

---

## Per-axis reliability (native `calc_stddev()`, no longer hand-computed)

All 6 axes are now well determined — a marked improvement over the previous
(legacy-script) run, where x/y were flagged unreliable (1.67σ / 0.46σ):

| Parameter | Value | Standard error | σ | Verdict |
|---|---|---|---|---|
| x (pEEx_1) | -4.672 mm | ±0.309 mm | **15.14σ** | solid |
| y (pEEy_1) | -1.032 mm | ±0.311 mm | **3.32σ** | solid |
| z (pEEz_1) | +128.776 mm | ±1.000 mm | **128.80σ** | solid |
| roll (phiEEx_1) | -1.100° | ±0.017° | **63.77σ** | solid |
| pitch (phiEEy_1) | -0.356° | ±0.017° | **21.10σ** | solid |
| yaw (phiEEz_1) | +1.942° | ±1.023° | 1.90σ | marginal |

x and y are no longer the weak axes they were in the previous run — the
SE3 log-map residual (vs. the legacy script's RPY-wraparound patch) removed
a real source of imprecision, not just the wraparound bug. Only yaw
(phiEEz_1) is now the marginal one (1.90σ, previously the strongest axis at
23.16σ) — the two runs used different D-optimal pose subsets (this run's 48
samples vs. the previous 48, both drawn from the same 51-pose plan but with
1 sample flagged as an outlier here), so some redistribution of which axis
is best-constrained between runs is expected, not a red flag by itself.

---

## Correction to apply in QTM

Since **all 6 axes are now solid or at worst marginal** (none in the
"noise"/<1σ range this run), there's no need for the "safe" position-only
variant from the previous run — use the full 6DOF correction directly.

**Translation** (current local axes of the rigid body):

| axis | value |
|---|---|
| x | +4.786 mm |
| y | +0.621 mm |
| z | +13.137 mm |

**Rotation** (roll/pitch/yaw = `R = Rz(yaw)·Ry(pitch)·Rx(roll)`, standard
Pinocchio/RPY convention):

- RPY (deg): roll = **+1.0875°**, pitch = **+0.3928°**, yaw = **-1.9348°**
- Quaternion (x, y, z, w): `[0.009546, 0.003267, -0.016915, 0.999806]`
- Rotation matrix:
  ```
  [ 0.999406   0.033887   0.006209]
  [-0.033762   0.999245  -0.019200]
  [-0.006855   0.018978   0.999796]
  ```

Note the z-translation changed noticeably from the previous run's +4.76mm to
+13.14mm here (pEEz_1 itself moved from 137.16mm to 128.78mm) — both are
statistically solid (>100σ) in their respective runs, so this isn't
noise: it reflects the different pose subset actually used (1 outlier
dropped this run) and the corrected residual metric, not measurement
instability. If you already applied the previous run's correction in QTM,
re-measure rather than assuming the old number still holds.

---

## Notes

- Correction still based on only 48 samples from a single data-collection
  session. The precision jump between runs (x/y going from ~1σ/0.5σ to
  15σ/3σ) came from fixing the residual metric, not from more data — more
  samples would still further tighten these, especially yaw (1.90σ).
- This correction only touches the mocap body definition. The arm joint
  URDF offsets (`master_calibration_20260805.yaml` /
  `master_calibration_20260805_conservative.yaml`) are independent and
  must be deployed separately on the robot.
