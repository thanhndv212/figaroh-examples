# LC1: log-Cholesky against OLS and OLS+SDP on TIAGo (recorded data)

Issue: [figaroh-examples #11](https://github.com/thanhndv212/figaroh-examples/issues/11).
Simulated counterpart, with the truth gates, Pinocchio 4.1 and weak excitation:
[ur10-log-cholesky-lc1.md](ur10-log-cholesky-lc1.md).

> **RECORDED DATA.** Two real TIAGo sessions under the frozen cross-run
> protocol v1. Effort is the PAL controller signal converted by the example's
> reduction ratios and motor constants. It is not a torque sensor: the source
> audit (#68) found arm_2–arm_4 read 10–20 % low, and arm_1's scale is not
> identifiable. This compares estimators on the same real rows. It does **not**
> validate the identified parameters on hardware, and nothing is exported.

## Command

From `examples/tiago`, in `figaroh-dev` (core `devel` ≥ `21994ed`):

```bash
# full: five starts, production and forced-interior branches (about 6 min)
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MPLBACKEND=Agg \
  python log_cholesky_benchmark.py --output-dir /tmp/lc1-tiago

# smoke: three starts, production branch only (about 1-2 min)
MPLBACKEND=Agg python log_cholesky_benchmark.py --smoke --output-dir /tmp/lc1-tiago-smoke
```

Exit codes are the UR10 benchmark's: 0 all gates met, 1 a gate unmet, 2
preflight failed, 3 core has no log-Cholesky fit. Archive:
`results/tiago-log-cholesky-lc1-pin37.json` and `.md`, from a clean worktree at
examples `v1.1.0-10-g1fd64fa`, core `v1.1.0-21-g21994ed`, Pinocchio 3.7.0. It
holds the SHA-256 of the protocol, configs, script and every input CSV.

## Protocol

| Item | Value |
|---|---|
| Training | `dynamic/`, session 2021-07-01-1302, rows [921, 6791), decimated to 2348 rows |
| Held-out | `calibration_slow/`, session 2021-07-01-1257 (role `validation`), the same path at about half speed, 14163 rows. No fit or setting sees it |
| Processing | The reference case's: velocity from positions, automatic lag, core filtering, `decimate=True`, unweighted |
| Fitted effort | arm_1–arm_4 (N·m). arm_5–arm_7 effort is mostly zero (#20) and the torso is not fitted |
| Parameters | 812 standard (58 links, hand included, plus friction, offset and actuator inertia per joint); 57 base; condition number 1.3e3 |
| Prior | Nominal `urdf/tiago_48_hey5.urdf`, `prior_weight` 1e-6, `mass_min` 1e-6 |
| Overlays | `config/tiago_lc1_ols.yaml`, `tiago_lc1_ols_sdp.yaml`, `tiago_lc1_log_cholesky.yaml` (direct = the last with `method: direct`) |

Every method's full standard vector is scored by core's own
`_compute_validation_metrics` (the selected estimate swapped for that vector),
so all four share the held-out rows, masks and fitted-joint blocks. The
preflight checks that both splits are finite and every fitted joint is excited
(ddq RMS ≥ 1e-2 rad/s²; observed minimum 0.134 held out).

## Results

Held-out effort RMSE (N·m) on 2021-07-01-1257:

| Method | Aggregate | arm_1 | arm_2 | arm_3 | arm_4 | arm_2–4 pooled | Correlation |
|---|---|---|---|---|---|---|---|
| OLS | 1.722 | 1.310 | 2.540 | 1.471 | 1.237 | 1.839 | 0.9283 |
| OLS+SDP | 1.733 | 1.358 | 2.539 | 1.440 | 1.283 | 1.841 | 0.9269 |
| Log-Cholesky | 1.713 | 1.303 | 2.539 | 1.446 | 1.226 | 1.829 | 0.9293 |
| Direct | 1.713 | 1.303 | 2.539 | 1.446 | 1.226 | 1.829 | 0.9293 |

- **Held out.** Log-Cholesky is 1.2 % better than OLS+SDP (ratio 0.9885) and
  0.5 % better than OLS. The differences are small against the #68 effort
  error: arm_1's relative RMSE is 0.42 for every method.
- **Training objective.** J is 1838 (OLS, non-physical), 2163 (OLS+SDP) and
  1912 (log-Cholesky), in N²·m².
- **Feasibility.** OLS is non-physical on 43 of 58 links (min eig −3.67).
  OLS+SDP and log-Cholesky are feasible on all 58 (min eig −1.2e-9).
- **Branch and runtime.** Boundary (unconstrained min eig −50.5), rank 57,
  null space 755; the fit takes 2.45 s and the start is unused. Log-Cholesky
  equals direct.

### Finding: feasible is not plausible

Masses (kg) on the torso/arm links:

| Link | Nominal | OLS | OLS+SDP | Log-Cholesky |
|---|---|---|---|---|
| arm_2 | 1.80 | 1.79 | 1.79 | 3.80 |
| arm_3 | 1.80 | 1.78 | 1.78 | **88.8** |
| arm_4 | 1.43 | 1.44 | 1.45 | 11.6 |
| arm_7 | 0.93 | 0.93 | 0.97 | 0.25 |

Log-Cholesky puts 88.8 kg on arm_3, 49× the URDF. The fit is physical (every
pseudo-inertia is PSD) and predicts the held-out motion slightly better, so the
recorded motion constrains that mass only weakly, and with `prior_weight`
1e-6 the prior does not hold it near the URDF. OLS and OLS+SDP stay near nominal only
because null-space reconstruction fills those directions with the nominal
values before any fit.

Do not export this fit. A plausible TIAGo model needs a stronger prior,
mass bounds, or more excitation. The decision record has no plausibility
gate; the benchmark reports the largest identified / nominal mass ratio and
does not gate it.

**The nominal URDF is itself slightly non-physical**: arm_1's pseudo-inertia
has min eig −1.2e-4. Log-Cholesky coordinates exist only for positive-definite
links, so the raw nominal start is `invalid_start` under forced interior, and
the perturbed starts perturb a repaired nominal.

### Initialization sensitivity

| Start | Production (auto) | Forced interior |
|---|---|---|
| nominal | optimal, start unused | `invalid_start` (arm_1) |
| repaired OLS | optimal, start unused | `time_budget`, 271 evaluations in 60 s |
| perturbed 1101, 1102 | optimal, start unused | `time_budget`, 291 and 300 evaluations |
| unrepaired OLS | optimal, start unused | `invalid_start` |

Every production start gives the same θ (max |Δθ| 0). As on UR10, forced
interior cannot reach a boundary optimum; here the 60 s `max_seconds` budget
stops it first (812 parameters, about 0.2 s per evaluation).

## Gates

The decision record's truth-free gates:

| Gate | Value | Limit | Met |
|---|---|---|---|
| Held-out LC / OLS+SDP (OLS+SDP status ok) | 0.9885 | ≤1.05 | yes |
| Feasibility, min eig over every link | −1.18e-9 | ≥−1e-8 | yes |
| Fit runtime | 2.45 s | ≤20 s | yes |
| Nominal and repaired-OLS starts converge and are feasible (production) | yes | all | yes |

Not applicable: the clean held-out RMSE against truth and the oracle interior
check need a truth model, which recorded data does not have.

## Tests

`tests/test_tiago_log_cholesky_benchmark.py`: the overlay layout, each gate's
failure case, and the mass-ratio report (fast); the smoke run on the recorded
sessions, which checks every gate, the split and that all methods are scored on
the same held-out rows (`slow`).
