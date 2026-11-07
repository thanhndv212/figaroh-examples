# LC1 log-Cholesky comparison (#11): RECORDED TIAGo data

> **RECORDED DATA**, frozen cross-run protocol v1. Estimators compared on the same real rows; effort is a converted controller signal (#68). Not hardware validation of the parameters; no URDF export.

- examples `v1.1.0-10-g1fd64fa` (1fd64fa4), core `v1.1.0-21-g21994ed` (21994ed3)
- python 3.12.11, numpy 2.3.4, scipy 1.16.1, pinocchio 3.7.0, picos 2.6.1, cvxopt 1.3.2
- minimum core: figaroh-plus devel >= 21994ed; unreleased after 1.1.0
- train: dynamic/ (session 2021-07-01-1302), rows [921, 6791)
- held-out: data/identification/calibration_slow (session 2021-07-01-1257, role validation), all rows
- processing: velocity from positions, automatic lag, core filtering, decimate=True, wls=False
- fitted joints: arm_1_joint, arm_2_joint, arm_3_joint, arm_4_joint; 812 standard parameters; friction (viscous, Coulomb), joint offset, actuator inertia: free

## Preflight

passed: **True**

- train: 5870 rows, finite True, ddq RMS torso_lift_joint 0.076, arm_1_joint 0.301, arm_2_joint 0.469, arm_3_joint 0.548, arm_4_joint 0.427, arm_5_joint 0.569, arm_6_joint 0.51, arm_7_joint 0.396
- heldout: 14163 rows, finite True, ddq RMS torso_lift_joint 0.0326, arm_1_joint 0.134, arm_2_joint 0.207, arm_3_joint 0.247, arm_4_joint 0.172, arm_5_joint 0.248, arm_6_joint 0.213, arm_7_joint 0.175

## Held-out effort RMSE on 2021-07-01-1257 (calibration_slow)

| method | aggregate | arm_1_joint | arm_2_joint | arm_3_joint | arm_4_joint | arm_2-4 pooled | corr (normalised) |
|---|---|---|---|---|---|---|---|
| ols | 1.722 | 1.31 | 2.54 | 1.471 | 1.237 | 1.839 | 0.9283 |
| ols_sdp | 1.733 | 1.358 | 2.539 | 1.44 | 1.283 | 1.841 | 0.9269 |
| log_cholesky | 1.713 | 1.303 | 2.539 | 1.446 | 1.226 | 1.829 | 0.9293 |
| direct | 1.713 | 1.303 | 2.539 | 1.446 | 1.226 | 1.829 | 0.9293 |

## Training objective, physical consistency, conditioning

condition number 1.3e+03, 2348 training rows, 57 base parameters, 352 eliminated columns

| method | status | J | train RMSE | min eig (all links) | min mass | infeasible links | pipeline s |
|---|---|---|---|---|---|---|---|
| ols | ok | 1838.49 | 0.8843 | -3.67 | 0.00121 | 43 of 58 | 14.0 |
| ols_sdp | ok | 2162.9 | 0.9597 | -1.14e-09 | 0.00132 | 0 of 58 | 13.2 |
| log_cholesky | accepted | 1912 | 0.902 | -1.18e-09 | 0.00141 | 0 of 58 | 17.7 |
| direct | accepted | 1912 | 0.902 | -1.18e-09 | 0.00141 | 0 of 58 | 16.1 |

log-Cholesky: branch **boundary**, rank 57, null space 755, unconstrained min eig -50.5, fit 2.45 s, start used False; selected-stage prediction gap 0 N·m.

Per-link mass (kg) and minimum pseudo-inertia eigenvalue (torso/arm links, then the OLS fit's two least feasible other links):

| link | nominal mass | ols mass | ols min eig | ols_sdp mass | ols_sdp min eig | log_cholesky mass | log_cholesky min eig | direct mass | direct min eig |
|---|---|---|---|---|---|---|---|---|---|
| torso_lift_joint | 6.932 | 6.932 | 0.0418 | 6.932 | 0.0418 | 7.263 | 0.0957 | 7.263 | 0.0957 |
| arm_1_joint | 1.563 | 1.563 | -0.218 | 1.586 | -1.32e-10 | 1.653 | 0.00161 | 1.653 | 0.00161 |
| arm_2_joint | 1.8 | 1.793 | -2.72 | 1.794 | -4.91e-10 | 3.799 | 1.67e-08 | 3.799 | 1.67e-08 |
| arm_3_joint | 1.8 | 1.781 | -3.67 | 1.783 | -1.59e-11 | 88.8 | -6.9e-10 | 88.8 | -6.9e-10 |
| arm_4_joint | 1.433 | 1.436 | -1.72 | 1.452 | -3.62e-10 | 11.59 | 1.61e-09 | 11.59 | 1.61e-09 |
| arm_5_joint | 1.656 | 1.663 | -1.41 | 1.67 | -1.14e-09 | 2.354 | -2.02e-11 | 2.354 | -2.02e-11 |
| arm_6_joint | 0.3701 | 0.3618 | -1.51 | 0.3624 | -2.72e-10 | 0.182 | -1.12e-09 | 0.182 | -1.12e-09 |
| arm_7_joint | 0.9345 | 0.9261 | -0.0306 | 0.9712 | -5.46e-11 | 0.253 | -1.14e-09 | 0.253 | -1.14e-09 |
| hand_index_flex_3_joint | 0.015 | 0.007455 | -0.033 | 0.007583 | -4.37e-11 | 0.00315 | -1.18e-09 | 0.00315 | -1.18e-09 |
| hand_index_virtual_3_joint | 0.01 | 0.00245 | -0.033 | 0.002533 | -2.69e-11 | 0.003666 | -1.18e-09 | 0.003666 | -1.18e-09 |

Nominal URDF (the prior) non-physical links: arm_1_joint (min eig -0.00012).

Largest identified / nominal mass ratio (reported, not gated; feasibility does not bound drift from the prior on a weakly observed link): ols 0.245× (hand_index_virtual_3_joint), ols_sdp 0.253× (hand_index_virtual_3_joint), log_cholesky 49.3× (arm_3_joint), direct 49.3× (arm_3_joint).

## Initialization sensitivity

| start | auto: status | start used | max |Δθ| to selected | forced interior: stop | nfev | identifiable RMS | s |
|---|---|---|---|---|---|---|---|
| nominal | optimal | False | 0 | invalid_start | — | — | 0.17 |
| repaired_ols | optimal | False | 0 | time_budget | 271 | — | 60.32 |
| perturbed_1101 | optimal | False | 0 | time_budget | 291 | — | 60.23 |
| perturbed_1102 | optimal | False | 0 | time_budget | 300 | — | 60.24 |
| unrepaired_ols | optimal | False | 0 | invalid_start | — | — | 0.19 |

## Gates

| gate | value | limit | met |
|---|---|---|---|
| held-out LC / OLS+SDP (SDP ok: True) | 0.9885 | ≤ 1.05 | True |
| log-Cholesky min pseudo-inertia eig, every link | -1.18e-09 | ≥ -1e-08 | True |
| log-Cholesky fit time (s) | 2.45 | ≤ 20 | True |
| nominal + repaired-OLS starts converge, feasible | True | all | True |

All gates met: **True**. Not applicable: clean held-out RMSE vs truth and the oracle interior check need a truth model; recorded data has none. production (auto) branch; perturbed and unrepaired-OLS starts are reported, not gated.
