# TX40 model update: a physically consistent fit

`examples/staubli_tx40/update_model.py` writes identified link inertials into
a copy of the TX40 URDF. **It is not independently validated**: the TX40 has
a single recording ([no-validation brief](../briefs/tx40-no-validation-data.md)),
so every number below is measured on the data the fit was made on.

## 1. Why a physical fit

`identification.py` estimates 58 base parameters (plus the coupled-wrist
terms). They are combinations of the standard parameters, so they have no
per-link inertials to write into a URDF. Core's direct physical fit
(`select_stage: physical_fit`) estimates the standard parameters themselves:

    minimise  ||R theta - Q^T tau||^2 + lambda^2 ||z||^2
    subject to  every link's pseudo-inertia >= 0,  mass >= 1e-6

with `z` the deviation from the URDF (CAD) inertials, scaled per link (mass,
first moment, inertia). `BaseIdentification.export_urdf` then writes the
accepted estimate's inertials and reloads the file with Pinocchio.

## 2. Choosing the prior weight

The torque term sums over every decimated sample, so `lambda^2`
(`--prior-weight`) only matters in the hundreds. Training torque RMSE (all
joints) and fitted masses against CAD, QICS solver:

| `lambda^2` | RMSE (N·m) | link 1 | 2 | 3 | 4 | 5 | 6 (kg) |
|---|---|---|---|---|---|---|---|
| CAD | | 10.5 | 3.6 | 4.07 | 3.62 | 1.02 | 0.2 |
| 1e-6 (core default) | 3.000 | 28.6 | **384.8** | 89.2 | 130.0 | 1.43 | 0.24 |
| 1e-2 | 3.000 | 10.5 | 10.3 | 6.94 | 7.15 | 1.39 | 0.25 |
| 1 | 3.001 | 10.5 | 3.92 | 5.66 | 2.53 | 0.62 | 0.49 |
| **100 (default)** | **3.025** | 10.5 | 3.83 | 4.65 | 3.37 | 0.92 | 0.23 |
| 1e3 | 3.053 | 10.5 | 3.67 | 4.48 | 3.41 | 0.99 | 0.20 |
| 1e4 | 3.160 | 10.5 | 3.60 | 4.35 | 3.48 | 1.01 | 0.20 |
| 1e5 | 3.425 | 10.5 | 3.60 | 4.20 | 3.60 | 1.02 | 0.20 |

With almost no prior the fit is feasible but implausible: mass moves along
directions the excitation does not see, at no cost in torque. Link 1's mass
never moves, because joint 1 is vertical and the mass is not in any torque.
At 100 the RMSE costs 0.8 % and every mass is within 15 % of CAD. That is
the knee of the curve and the default.

Per joint at 100 (N·m), base-parameter WLS fit → physical fit: j1 3.05 → 3.06,
j2 3.85 → 3.90, j3 2.31 → 2.35, j4 0.89 → 0.95, j5 4.51 → 4.55, j6 1.78 → 1.80.

## 3. Solvers

The fit is an SDP solved through picos. With `lambda^2` = 100, cvxopt (in
`figaroh-dev`) gives an optimal solution in about 1 s. It reports no optimal
solution at 1e-6, 1, 1e3 and 1e4; this is the failure that
[physical-comparison-d4.md](physical-comparison-d4.md) recorded at core's
default weight. QICS 1.1.3 (`pip install qics`, not in the env) solves every
row of the table in under 0.5 s. It is the script's fallback when it is
installed. Where both succeed they agree to 1e-5 relative.

## 4. The exported URDF

- Exported vs estimated inertials (p10): max difference 3e-11.
- **Link 6 is a point mass** in the CAD URDF (zero inertia tensor). The fit
  stays there, with eigenvalues of about -1e-10, inside core's -1e-8
  feasibility tolerance. **Link 3 comes out rod-like** (smallest COM
  eigenvalue about 1e-9 against 3.8e-3 in CAD). The script lists such links.
  Simulators that need strictly positive inertia need a floor on them.
  The script does not add one: the URDF would then differ from the estimate.

## 5. What this does not support

- Prediction on other motions: there is no second recording.
- The individual inertia values: only their base combinations are
  identified. The rest come from the prior, which is the CAD model.
- Friction, actuator inertia and the coupled-wrist terms are fitted, but a
  URDF has nowhere to store them; they are not exported.

## Provenance

figaroh-plus `devel` `cf0f7a8`, figaroh-examples `main` `1dac70f` plus
this change; `figaroh-dev`, Python 3.12, picos 2.6.1, cvxopt 1.3.2, QICS
1.1.3 (scan only), macOS arm64. `python update_model.py` from
`examples/staubli_tx40`.
