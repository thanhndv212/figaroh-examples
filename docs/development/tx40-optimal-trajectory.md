# TX40 excitation trajectory design

`examples/staubli_tx40/optimal_trajectory.py` designs an excitation for TX40
dynamic identification with core's IPOPT generator. **It is a simulated
design**: the trajectory has not been run on a TX40, and the URDF has no
floor or cell.

## 1. The near-singular design was a base-index tolerance

The earlier attempt ran every 5-waypoint segment to the 200-iteration limit
(about 9 min) with a condition number near 2e12, whatever the trajectory.
The trajectory was not the cause. Core's
`BaseParameterComputer._get_idx_from_random` picks base columns with
`get_baseIndex` and an **absolute** QR tolerance of 1e-8. Joint 2's axis is
perpendicular to joint 1's, so `Izz_joint_2` and `Ia_joint_2` are both q̈2 in
τ2. Their QR pivot is round-off, about 1e-8 against a largest pivot of 44, and
it passes the absolute test. With both columns kept, the condition number of
any trajectory's base regressor is about 1e13.

`TX40OptimalTrajectory` (`utils/staubli_tx40_tools.py`) picks the base
columns with `QRDecomposer(relative_tolerance=1e-10)`. It uses the same
regressor `TX40Identification` fits (friction, actuator inertia, joint
offsets and the 3 coupled-wrist columns), giving 61 base parameters (58 + 3).
Random segments then have condition numbers of 290 to 760.

## 2. Settings

| Item | Value |
|---|---|
| Segments | 2 stacked, 5 waypoints, 1.5 s each |
| Regressor sampling | 20 Hz (was 200). The objective builds the regressor at every evaluation; 200 Hz made one segment take about 9 min. |
| IPOPT | 100 iterations per segment. The condition-number objective is non-smooth, so a feasible final iterate is kept at the limit. |
| Retries | 2 attempts per segment from new random starts (`segment_attempts`) |
| Soft limit | 0.05 of the joint range |

## 3. Result

Seed 0, figaroh-plus `devel` `cf0f7a8`, figaroh-examples `main` `1dac70f`
plus this change; `figaroh-dev`, macOS arm64, run alone.

| | |
|---|---|
| Base parameters | 61 |
| Final condition number, 2 segments | **114** |
| Recorded TX40 data, for comparison | 967 (58 base parameters) |
| Segment 1 | attempt 1 restoration failed (133 s); attempt 2 feasible at the iteration limit, objective 259 (148 s) |
| Segment 2 | feasible at the iteration limit, objective 114 (514 s) |
| Total | 803 s |

`validate.py` runs it as a slow script with a 1200 s budget, so `--quick`
and `--changed` skip it by default.

## 4. Limits

- Simulated: no collision with a cell or the floor, and no check against the
  controller's speed, acceleration and torque limits beyond the URDF's.
- A low condition number shows the base parameters are excited. It does not
  show how well a real recording of this trajectory would identify them,
  because filtering and current-to-torque errors are not simulated.
- The fix is local to this example. Core's trajectory path still uses the
  absolute tolerance (figaroh-plus#218).
