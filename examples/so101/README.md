# SO-101 — gravity + friction identification

Identifies the gravity parameters, joint friction and torque offsets of a
[TheRobotStudio SO-101](https://github.com/TheRobotStudio/SO-ARM100) arm
from **servo current** (Feetech STS3215, 1:345 — no joint torque sensors),
and exports them in the format `soarm_sdk.dynamics.IdentifiedDynamics`
loads, for gravity compensation and admittance control on the arm.

```
soarm-identify-record  ──►  <run>/  ──►  identification.py  ──►  update_model.py  ──►  so101_dynamics.yaml
   (soarm_sdk, host)      q, current      (this folder)          (this folder)       (soarm_sdk, host)
```

## Quick start (simulated data, no arm)

```bash
conda activate figaroh-dev
cd examples/so101
python identification.py            # fit data/simulated, validate on data/simulated_validation
python update_model.py              # -> results/so101_dynamics.yaml
```

`data/simulated*` come from `generate_simulated_data.py`: an arm 10–20 %
heavier than the CAD model with a 40 g payload at the gripper, full
rigid-body dynamics, friction, offsets, 15 mA current noise and the
STS3215's 6.5 mA quantisation. The truth is in `ground_truth.yaml` next to
the data, and `identification.py` prints the identified gravity torque
against it:

```
Gravity torque vs ground truth (RMS over the trajectory, N·m):
  joint          identified        CAD
  shoulder_lift     0.00576    0.11233
  elbow_flex        0.01480    0.12252
  wrist_flex        0.00497    0.04925
```

## On the real arm

1. **Calibrate and validate** the arm's joint frame in soarm_sdk
   (`soarm-dashboard-calibration`). A wrong direction sign flips that
   joint's gravity torque in the fit.
2. **Record** (host, arm connected; ~2 min, ≤ 0.5 rad/s):
   ```bash
   soarm-identify-record --arm-id my_arm --out runs/ident01
   soarm-identify-record --arm-id my_arm --out runs/ident02 --seed 1   # validation
   ```
   `--dry-run` plans and writes a log without touching hardware (its
   currents are zero — it tests the plumbing, not the fit).
3. **Identify**:
   ```bash
   python identification.py --data-dir runs/ident01 --validation-dir runs/ident02 --asset-id my_arm
   ```
4. **Deploy**:
   ```bash
   python update_model.py --data-dir runs/ident01 --validation-dir runs/ident02 \
       --output ~/.soarm_sdk/so101_dynamics.yaml
   ```
   ```python
   from soarm_sdk.dynamics import IdentifiedDynamics
   dyn = IdentifiedDynamics.load("~/.soarm_sdk/so101_dynamics.yaml", urdf=".../so101_new_calib.urdf")
   tau_g = dyn.gravity_torque(q_arm)          # N·m, URDF frame, 5 arm joints
   i_expected = dyn.to_signal(dyn.torque(q_arm, dq_arm, deadband_rad_s=0.05))
   ```

## What is identified, and what is not

The model is `tau = g(q) + fv·qd + fs·sign(qd) + offset`
(`custom.regressor.inertial_terms: false`). The excitation is slow and
STS3215 current is coarse, so inertia tensors are neither excitable nor
needed for gravity compensation; the inertial regressor columns are built
at zero velocity and acceleration, leaving only the mass/first-moment
(gravity) columns.

On this geometry that is **8 gravity base parameters** — per link of the
pitch chain, a first moment in x and y lumped with the masses beyond it,
plus the wrist-roll body's in-axis offset — and 15 friction/offset terms.
`update_model.py` writes per-body `m, mx, my, mz` reconstructed from them
with the CAD values as prior: **the gravity torque they produce is exact
for the fit, but the split between bodies is not individually measured**,
so don't read a single link's mass off it.

Friction is estimated less reliably than gravity: with the (small)
inertial torque left out of the model, some of it lands in the viscous
term (e.g. `fv_elbow_flex` 0.075 vs 0.040 true on the simulated data).

## Units — why the current scale need not be right

The servos report current (or PWM load), not torque.
`custom.torque_sensing` names the signal and its scale (`nm_per_unit`; the
default 1 N·m/A is a nominal figure from 12 V STS3215 stall data, not a
measurement). A wrong scale scales every identified parameter by the same
factor, and `IdentifiedDynamics.to_signal` divides by it again — so the
*predicted servo current* a controller compares against is still right.
Identify once with a known payload if you need physical units.

If a joint's current never changes sign during the run, the servo may be
reporting magnitude only; `identification.py` warns, and
`--signal load_percent` fits the signed PWM duty instead.

## Notes

- **QR rank threshold.** figaroh's absolute default (1e-6) keeps gravity
  combinations excited only through millimetre out-of-plane joint offsets;
  they fit noise (condition number ~5e8, masses in the thousands).
  `tasks.identification.problem.qr_relative_tolerance: 1e-4` drops them:
  condition number 429, validation correlation 0.9995.
- This example needs figaroh with the fixes for figaroh-plus #11-#14 (CAD
  prior per body, regressor column layout, filter rates from
  `signal_processing`, `qr_relative_tolerance`).

## Files

| File | |
|---|---|
| `identification.py` | Fit + verify + report + archive (`--data-dir`, `--validation-dir`, `--signal`) |
| `update_model.py` | Fit, verify, write `soarm_sdk.dynamics.identified/v1` YAML, round-trip check through soarm_sdk if installed |
| `generate_simulated_data.py` | Simulated log with known ground truth |
| `utils/so101_tools.py` | `SO101Identification`, gripper-locked model loader, log reader, export |
| `config/so101_unified_config.yaml` | Identification task + `custom:` torque sensing / regressor options |
| `urdf/so101_new_calib.urdf` | SO-ARM100 `so101_new_calib.urdf` with visuals/collisions stripped (Apache-2.0, see header) |
