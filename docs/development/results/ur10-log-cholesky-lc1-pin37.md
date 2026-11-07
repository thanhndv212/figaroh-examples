# LC1 log-Cholesky comparison (#11): SIMULATED UR10 truth fixture

> **SIMULATED DATA.** UR10 truth fixture; no hardware validation, no URDF export.

- examples `v1.1.0-7-gce2628b` (ce2628b3), core `v1.1.0-21-g21994ed` (21994ed3)
- python 3.12.11, numpy 2.3.4, scipy 1.16.1, pinocchio 3.7.0, picos 2.6.1, cvxopt 1.3.2
- minimum core: figaroh-plus devel >= 21994ed (log-Cholesky #235 plus the #234 null-space fix #236); unreleased after 1.1.0
- joints: shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint; effort unit N·m

## Preflight

passed: **True**; pipeline ddq is fixture ddq: True

- train: max ddq central difference 1.31e-09 rad/s², min ddq RMS 0.421 rad/s², CRBA+NLE 3.6e-14 N·m
- validation: max ddq central difference 1.28e-09 rad/s², min ddq RMS 1.54 rad/s², CRBA+NLE 4.3e-14 N·m

Weak excitation: only shoulder_pan_joint moves (2400 rows); the others hold [0.0, -1.5707963267948966, 1.5707963267948966, -1.5707963267948966, -1.5707963267948966, 0.0] rad. derived in memory from train.csv; the active joint's derivatives are the fixture's (preflight-verified).

## Held-out RMSE (aggregate, N·m)

| data | seed | OLS | OLS+SDP | log-Cholesky | direct | LC vs truth | LC/SDP |
|---|---|---|---|---|---|---|---|
| none | None | 3.2458e-14 | 0.021228 | 5.8404e-06 | 5.8404e-06 | 5.84e-06 | 0.0003 |
| low | 101 | 0.18672 | 0.18789 | 0.18672 | 0.18672 | 0.0187 | 0.9937 |
| low | 102 | 0.19026 | 0.19116 | 0.19026 | 0.19026 | 0.0163 | 0.9953 |
| low | 103 | 0.19236 | 0.19554 | 0.19233 | 0.19233 | 0.0251 | 0.9836 |
| low | 104 | 0.19171 | 0.19236 | 0.19171 | 0.19171 | 0.0169 | 0.9966 |
| low | 105 | 0.18973 | 0.20396 | 0.18972 | 0.18972 | 0.0199 | 0.9302 |
| high | 101 | 0.93359 | 0.9747 | 0.93335 | 0.93335 | 0.0908 | 0.9576 |
| high | 102 | 0.95124 | 0.95205 | 0.95083 | 0.95083 | 0.0744 | 0.9987 |
| high | 103 | 0.9618 | 0.97132 | 0.96106 | 0.96106 | 0.121 | 0.9894 |
| high | 104 | 0.95858 | 0.9644 | 0.9585 | 0.9585 | 0.0834 | 0.9939 |
| high | 105 | 0.94867 | 1.0339 | 0.94639 | 0.94639 | 0.0855 | 0.9153 |
| weak/none | None | 1.5115 | 1.577 | 1.7865 | 1.7865 | 1.79 | 1.1329 |
| weak/low | 101 | 373.33 | 233.55 | 49.308 | 49.308 | 49.3 | 0.2111 |

## Per-joint held-out RMSE, first dataset of each kind (N·m)

**none** (seed None)

| joint | ols | ols_sdp | log_cholesky | direct |
|---|---|---|---|---|
| shoulder_pan_joint | 2.372e-14 | 0.03082 | 2.698e-06 | 2.698e-06 |
| shoulder_lift_joint | 4.896e-14 | 0.02975 | 7.248e-06 | 7.248e-06 |
| elbow_joint | 5.431e-14 | 0.02948 | 5.027e-06 | 5.027e-06 |
| wrist_1_joint | 1.936e-14 | 3.687e-06 | 5.07e-06 | 5.07e-06 |
| wrist_2_joint | 5.066e-15 | 7.177e-07 | 9.633e-06 | 9.633e-06 |
| wrist_3_joint | 3.439e-15 | 5.75e-08 | 1.033e-06 | 1.033e-06 |

**low** (seed 101)

| joint | ols | ols_sdp | log_cholesky | direct |
|---|---|---|---|---|
| shoulder_pan_joint | 0.04064 | 0.05297 | 0.04064 | 0.04064 |
| shoulder_lift_joint | 0.3728 | 0.3737 | 0.3728 | 0.3728 |
| elbow_joint | 0.26 | 0.2615 | 0.26 | 0.26 |
| wrist_1_joint | 0.03041 | 0.03041 | 0.03041 | 0.03041 |
| wrist_2_joint | 0.006202 | 0.006202 | 0.006199 | 0.006199 |
| wrist_3_joint | 0.00387 | 0.00387 | 0.00387 | 0.00387 |

**high** (seed 101)

| joint | ols | ols_sdp | log_cholesky | direct |
|---|---|---|---|---|
| shoulder_pan_joint | 0.2032 | 0.222 | 0.2007 | 0.2007 |
| shoulder_lift_joint | 1.864 | 1.931 | 1.864 | 1.864 |
| elbow_joint | 1.3 | 1.377 | 1.3 | 1.3 |
| wrist_1_joint | 0.152 | 0.1539 | 0.1512 | 0.1512 |
| wrist_2_joint | 0.03109 | 0.03123 | 0.02514 | 0.02514 |
| wrist_3_joint | 0.01936 | 0.01108 | 0.01671 | 0.01671 |

**weak/none** (seed None)

| joint | ols | ols_sdp | log_cholesky | direct |
|---|---|---|---|---|
| shoulder_pan_joint | 0.2008 | 0.2437 | 0.2196 | 0.2196 |
| shoulder_lift_joint | 3.67 | 3.837 | 4.362 | 4.362 |
| elbow_joint | 0.2969 | 0.2635 | 0.2364 | 0.2364 |
| wrist_1_joint | 0.1428 | 0.1577 | 0.06828 | 0.06828 |
| wrist_2_joint | 0.2558 | 0.1947 | 0.08186 | 0.08186 |
| wrist_3_joint | 0.1646 | 0.09335 | 0.04853 | 0.04853 |

**weak/low** (seed 101)

| joint | ols | ols_sdp | log_cholesky | direct |
|---|---|---|---|---|
| shoulder_pan_joint | 704.5 | 378.7 | 7.418 | 7.418 |
| shoulder_lift_joint | 391.4 | 294.7 | 120.5 | 120.5 |
| elbow_joint | 313.2 | 241.1 | 2.761 | 2.761 |
| wrist_1_joint | 213.2 | 145.7 | 0.493 | 0.493 |
| wrist_2_joint | 140.1 | 120.7 | 0.2339 | 0.2339 |
| wrist_3_joint | 153.3 | 55.11 | 0.05422 | 0.05422 |

## Training objective, physical consistency, conditioning

| data | seed | J OLS | J OLS+SDP | J LC | min eig OLS | min eig SDP | min eig LC | branch | rank | null | cond | LC fit s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| none | None | 0.0123546 | 2.3164 | 0.00275929 | -0.026 | 8.8e-10 | 1.94e-08 | boundary | 36 | 24 | 92.7 | 0.022 |
| low | 101 | 541.288 | 543.49 | 541.278 | -0.0259 | 3.48e-09 | 3.02e-09 | boundary | 36 | 24 | 92.7 | 0.020 |
| low | 102 | 506.102 | 507.603 | 506.093 | -0.0216 | 5.27e-09 | 1.29e-08 | boundary | 36 | 24 | 92.7 | 0.019 |
| low | 103 | 545.502 | 557.641 | 545.5 | -0.0329 | -1.53e-10 | -2.36e-10 | boundary | 36 | 24 | 92.7 | 0.021 |
| low | 104 | 504.862 | 508.212 | 504.852 | -0.0321 | 5.28e-09 | 5.43e-09 | boundary | 36 | 24 | 92.7 | 0.022 |
| low | 105 | 513.201 | 564.705 | 513.192 | -0.028 | -3.65e-10 | -2.53e-11 | boundary | 36 | 24 | 92.7 | 0.022 |
| high | 101 | 13530.6 | 14130.7 | 13530.9 | -0.0365 | -1.07e-10 | -1e-10 | boundary | 36 | 24 | 92.7 | 0.025 |
| high | 102 | 12653.6 | 12720.2 | 12654.3 | -0.0426 | -1.88e-10 | -7.63e-11 | boundary | 36 | 24 | 92.7 | 0.055 |
| high | 103 | 13637.8 | 13881.4 | 13640.2 | -0.0704 | -4.69e-11 | -5.39e-12 | boundary | 36 | 24 | 92.7 | 0.024 |
| high | 104 | 12622.2 | 12887.3 | 12622.4 | -0.0648 | -1.05e-10 | -2.73e-10 | boundary | 36 | 24 | 92.7 | 0.022 |
| high | 105 | 12828.4 | 14325.1 | 12831 | -0.0585 | -4.45e-10 | -1.83e-10 | boundary | 36 | 24 | 92.7 | 0.054 |
| weak/none | None | 0.00614177 | 173.92 | 0.00232959 | -0.0188 | -7.54e-11 | 3.74e-09 | boundary | 15 | 45 | 40.6 | 0.021 |
| weak/low | 101 | 601364 | 5.30957e+08 | 543.022 | -769 | 3.43e-09 | 3.97e-09 | boundary | 27 | 33 | 7e+06 | 0.023 |

Per-link mass (kg) and minimum pseudo-inertia eigenvalue, none seed None:

| link | ols mass | ols min eig | ols_sdp mass | ols_sdp min eig | log_cholesky mass | log_cholesky min eig | direct mass | direct min eig |
|---|---|---|---|---|---|---|---|---|
| shoulder_pan_joint | 7.778 | 0.0155 | 7.778 | 0.0155 | 7.669 | 0.0109 | 7.669 | 0.0109 |
| shoulder_lift_joint | 12.93 | 0.0189 | 12.93 | 0.0189 | 12.91 | 0.0102 | 12.91 | 0.0102 |
| elbow_joint | 3.589 | -0.026 | 3.589 | 8.8e-10 | 3.386 | 0.000625 | 3.386 | 0.000625 |
| wrist_1_joint | 1.714 | 0.00276 | 1.714 | 0.00276 | 1.015 | 2.64e-06 | 1.015 | 2.64e-06 |
| wrist_2_joint | 1.722 | 0.00166 | 1.722 | 0.00166 | 2.886 | 6.83e-08 | 2.886 | 6.83e-08 |
| wrist_3_joint | 0.2333 | 0.00129 | 0.2333 | 0.00129 | 0.1817 | 1.94e-08 | 0.1817 | 1.94e-08 |

## Initialization sensitivity

**none seed None**

| start | auto: status | auto: start used | auto: max |Δθ| | forced interior: stop | nfev | identifiable RMS | s |
|---|---|---|---|---|---|---|---|
| nominal | optimal | False | 0 | max_nfev | 2000 | 0.948 | 2.11 |
| repaired_ols | optimal | False | 0 | max_nfev | 2000 | 6.37 | 1.94 |
| perturbed_1101 | optimal | False | 0 | not_converged | 388 | 30.7 | 0.39 |
| perturbed_1102 | optimal | False | 0 | max_nfev | 2000 | 18.5 | 2.06 |
| unrepaired_ols | optimal | False | 0 | invalid_start | — | — | 0.00 |

**low seed 101**

| start | auto: status | auto: start used | auto: max |Δθ| | forced interior: stop | nfev | identifiable RMS | s |
|---|---|---|---|---|---|---|---|
| nominal | optimal | False | 0 | max_nfev | 2000 | 1.09 | 2.18 |
| repaired_ols | optimal | False | 0 | max_nfev | 2000 | 8 | 1.98 |
| perturbed_1101 | optimal | False | 0 | max_nfev | 2000 | 23.9 | 2.10 |
| perturbed_1102 | optimal | False | 0 | not_converged | 121 | 50.1 | 0.11 |
| unrepaired_ols | optimal | False | 0 | invalid_start | — | — | 0.00 |

**high seed 101**

| start | auto: status | auto: start used | auto: max |Δθ| | forced interior: stop | nfev | identifiable RMS | s |
|---|---|---|---|---|---|---|---|
| nominal | optimal | False | 0 | max_nfev | 2000 | 1.39 | 2.10 |
| repaired_ols | optimal | False | 0 | max_nfev | 2000 | 45.1 | 2.27 |
| perturbed_1101 | optimal | False | 0 | max_nfev | 2000 | 38.1 | 2.12 |
| perturbed_1102 | optimal | False | 0 | max_nfev | 2000 | 43.3 | 2.21 |
| unrepaired_ols | optimal | False | 0 | invalid_start | — | — | 0.01 |

**weak/none seed None**

| start | auto: status | auto: start used | auto: max |Δθ| | forced interior: stop | nfev | identifiable RMS | s |
|---|---|---|---|---|---|---|---|
| nominal | optimal | False | 0 | max_nfev | 2000 | 2.19 | 2.19 |
| repaired_ols | optimal | False | 0 | not_converged | 358 | 16.2 | 0.39 |
| perturbed_1101 | optimal | False | 0 | max_nfev | 2000 | 29.3 | 2.14 |
| perturbed_1102 | optimal | False | 0 | max_nfev | 2000 | 97 | 2.24 |
| unrepaired_ols | optimal | False | 0 | invalid_start | — | — | 0.01 |

**weak/low seed 101**

| start | auto: status | auto: start used | auto: max |Δθ| | forced interior: stop | nfev | identifiable RMS | s |
|---|---|---|---|---|---|---|---|
| nominal | optimal | False | 0 | max_nfev | 2000 | 2.58 | 2.26 |
| repaired_ols | optimal | False | 0 | max_nfev | 2000 | 1.67e+03 | 2.05 |
| perturbed_1101 | optimal | False | 0 | max_nfev | 2000 | 426 | 2.08 |
| perturbed_1102 | optimal | False | 0 | max_nfev | 2000 | 452 | 2.07 |
| unrepaired_ols | optimal | False | 0 | invalid_start | — | — | 0.01 |

**Oracle interior check** (prior = truth, noise-free; solver diagnostic only): detected branch interior, unconstrained min eig 0.00134

| start | status | stop | nfev | identifiable RMS | max |Δθ| to truth | s |
|---|---|---|---|---|---|---|
| nominal | optimal | converged | 8 | 2.21e-09 | 6.76e-12 | 0.02 |
| repaired_ols | optimal | converged | 32 | 5.71e-11 | 1.43e-12 | 0.07 |
| perturbed_1101 | optimal | converged | 43 | 9.23e-14 | 8.88e-15 | 0.05 |
| perturbed_1102 | optimal | converged | 31 | 1.52e-11 | 3.87e-13 | 0.03 |
| unrepaired_ols | invalid_start | None | — | — | — | 0.00 |

## Decision-record gates

| gate | value | limit | met |
|---|---|---|---|
| clean held-out RMSE vs truth (N·m) | 5.84e-06 | ≤ 1e-05 | True |
| noisy held-out LC / OLS+SDP (SDP ok: True) | 0.9987 | ≤ 1.05 | True |
| log-Cholesky min pseudo-inertia eig, every link and fit | -2.73e-10 | ≥ -1e-08 | True |
| log-Cholesky fit time (s) | 0.055 | ≤ 20 | True |
| nominal + repaired-OLS starts converge, feasible (production / oracle interior) | True / True | all | True |
| weak excitation (weak/none, weak/low; rank [15, 27]): accepted, feasible, starts converge | True | all | True |

All gates met: **True**. near-boundary (perturbed) and invalid (unrepaired OLS) starts are reported, not gated.
