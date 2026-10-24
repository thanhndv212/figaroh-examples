# TALOS identification: synthetic truth study

There are no TALOS dynamics recordings. This study draws a truth, simulates
the joint torques it would produce on an excitation trajectory, identifies
them with FIGAROH's `BaseIdentification` and judges the fit against the
truth. Everything here is **simulated**: it checks the pipeline and the
excitation at a stated noise level, it does not estimate the dynamics of any
TALOS.

`examples/talos/identification_truth.py` reproduces every number below in
about 10 s; `tests/test_talos_identification_truth.py` runs a subset in CI.
It follows the rules of the [UR10 study](../../examples/ur10/identification_truth.py)
without its frozen fixture: every input is regenerated from seeds.

## 1. Design

| Item | Choice |
|---|---|
| Chain | `torso_1`, `torso_2`, `arm_left_1`…`7` (9 DOF). The rest of the body is locked (a Pinocchio reduced model), so the head, right arm and grippers merge into the links they hang from. |
| Locked posture (`REST`) | `q0` with the right arm abducted and bent (`arm_right_2` -0.5, `arm_right_4` -1.0 rad). At `q0` the arms hang into the legs' collision meshes: every posture collides. |
| Truth, inertias | Each URDF link inertia perturbed consistently (seeded): mass x exp N(0, 0.10), COM + N(0, 2 cm), second-moment eigenvalues x exp N(0, 0.20), principal axes rotated N(0, 0.2 rad). |
| Truth, joints | Coulomb friction 1-4 % of the URDF effort limit, viscous friction giving 1-4 % at 1 rad/s, reflected actuator inertia 0.5-1.5 g m² per Nm (0.1-0.3 kg m² on the 200 Nm torso joints). **Assumed ranges**, not TALOS data. |
| Excitation | One 10 s period of a 5-harmonic Fourier series per joint, centred in the joint range, at most 0.6 of the half-range (0.8 rad) and 0.6 of the velocity limit; rejected on the URDF limits, 90 % of the effort limit or self-collision (TALOS SRDF pairs, checked at 20 Hz). Training: first feasible seed from 0, held-out from 1 (steps of 2). |
| Measurements | Exact q, dq, ddq at 200 Hz; torque `RNEA + fv dq + fs sign(dq) + ia ddq` (core's regressor convention) plus N(0, level x RMS torque) per joint, level 2 % or 10 %. |
| Seeds | Truth seed s, noise seed 10 000 + s (held-out: + 100 000), s = 0…4. |
| Fit | Core `solve(decimate=False)`: zero-column elimination, QR base parameters, OLS. Joint torques only, friction and actuator inertia on, joint offsets off. |
| Score | Held-out torque RMSE against the noise-free truth, per joint as % of its RMS torque; base-parameter z-scores, (estimate - truth) / SE, with the SE from the per-joint residual variances. |

## 2. Results

figaroh-plus `devel` `cf0f7a8`, figaroh-examples `main` `1dac70f` plus this
change; `figaroh-dev`, Python 3.12, Pinocchio 3.7.0, macOS arm64.
`python identification_truth.py` from `examples/talos`. Training trajectory:
Fourier seed 2 (seed 0 self-collides); held-out: seed 1.

82 base parameters of 117 standard ones, base-regressor condition number
2.1e3. Noise-free: torques and held-out predictions recovered to 1e-12 %.

| Noise | Training RMS | Held-out, mean of joints | Held-out, worst joint and seed |
|---|---|---|---|
| 2 % | 2.05 % | **0.69 %** | 4.1 % |
| 10 % | 10.3 % | **3.42 %** | 20.2 % |

Held-out RMSE per joint, % of its RMS torque, mean over seeds:

| Noise | torso 1 | torso 2 | arm 1 | arm 2 | arm 3 | arm 4 | arm 5 | arm 6 | arm 7 |
|---|---|---|---|---|---|---|---|---|---|
| 2 % | 0.31 | 0.16 | 0.42 | 0.19 | 0.34 | 0.42 | 2.12 | 1.05 | 1.16 |
| 10 % | 1.55 | 0.80 | 2.08 | 0.93 | 1.69 | 2.10 | 10.6 | 5.25 | 5.82 |

**Base parameters:** 410 z-scores per noise level (82 x 5 seeds), mean
-0.008, RMS 0.91, max |z| 2.61. They are identical at both levels: the noise
draw is the same standard normal scaled by the level, and z does not depend
on the scale.

## 3. What this supports

- **The pipeline recovers a known model exactly** with friction and actuator
  inertia included: the simulated torques equal core's regressor times the
  truth to 3e-14 Nm, and noise-free fits are exact.
- **The standard errors are honest.** z RMS 0.91 over 410 parameters says the
  per-joint weighted covariance describes the estimate's scatter.
- **Held-out error is about a third of the torque noise** on the large
  joints and close to the noise on the wrist. `arm_left_5`-`7` carry about
  1 Nm RMS of torque, mostly friction and actuator inertia, and the
  excitation moves them less than its error budget needs. A real experiment
  would need a wrist-specific excitation (or higher wrist speeds) for 1 %
  wrist torque prediction.

## 4. Limits

- Same model class for truth and fit: no joint flexibility, no
  position-dependent friction, no torque-sensor offsets or gains, no
  cable forces. These numbers are a floor on what real data would give.
- Positions and derivatives are exact. Real velocities and accelerations
  come from filtered encoders, which biases the regressor; this study does
  not exercise core's filter.
- Torque noise is white and Gaussian. Real torque estimates from motor
  current carry gear ripple and temperature drift.
- The arm moves with the other arm locked and the robot fixed to the world.
  A free-floating humanoid's identification (base wrench from the feet FT
  sensors) is a different problem.
- Self-collision only: the URDF has no table or environment.
