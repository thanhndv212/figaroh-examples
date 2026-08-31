# TIAGo Experimental Data

The `suspension/` and `backlash/` directories contain historical offline logs
copied verbatim from the reachable `figaroh-plus` Git ref at commit
`2218d77638e0148afd3b358fa51702f8b82f4100`.

- `suspension/tiago_xyz_vicon_1640.csv` is a Vicon and force-plate recording.
  The example reconstructs the base-marker frame from markers `base1`, `base2`,
  and `base3`, subtracts the initial base pose and initial 100-sample wrench
  mean, then fits the fixed-transform generalized-base model. Its reported
  result is an experimental data fit, not a validated deployable suspension
  parameter set.
- `backlash/sinus_amp3_period10_2023-07-24-13-28-42/` contains the PAL
  introspection name/value exports. By default the example fits
  relative-minus-absolute encoder difference using relative position,
  velocity direction, and a Pinocchio-computed generalized gravity torque
  (`load_backlash_joint_trajectory_with_gravity`) as the load feature,
  reconstructed from every logged arm/torso/head joint position at each
  sample and evaluated against `urdf/tiago_48_hey5.urdf`. This reproduces
  the historical investigation's `tau_g` feature. Logged motor effort
  (`--load-feature effort`) is kept only for comparison: it is heavily
  quantized on the wrist joints (arm_5_joint..arm_7_joint, tens of unique
  values across thousands of samples) and was the source of earlier
  rank-deficient fits there. Two joints still need a lower polynomial
  degree than the rest even with the gravity feature: arm_1_joint's
  rotation axis does not couple to gravity (its gravity torque is
  identically ~0, so `backlash_empirical_surface.py` backs off to
  degree 0 there), and arm_2_joint's gravity torque is only moderately
  independent of its own position (backs off to degree 4).

Both artifacts are research data. Do not use them as a model-parameter source
without checking hardware identity, sensor calibration, coordinate conventions,
and the intended use approval.