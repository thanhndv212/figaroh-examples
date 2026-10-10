# Dynamic-identification reference workflow (D7)

The accepted way to run and judge a dynamic identification: one headless
command per case fits, selects the estimate, verifies it, exports the
fitted inertials to a URDF, reloads that URDF and checks it predicts what
the fit predicts, reports per-joint errors on data the fit never used, and
archives the run with what is needed to reproduce it (examples#23).

```bash
cd examples/ur10                       # known truth, scored against it
python identification_reference.py --case truth

cd ../tiago                            # shipped recordings; run cases one after the other
python identification_reference.py --case all --asset-id TIAGO-48 --root /path/to/runs

python examples/identification_reference.py --robot tiago --case reject   # from the repository root
```

A case exits 0 only when every observed outcome matches its expectation
table below, including the case that expects a rejection. Offline logs only:
no hardware deployment is claimed (`processing.claim` in `reproduction.json`).
The `revisions` item of the archive audit is `ok` only on a clean checkout of
both repositories; from a dirty one it is printed and does not fail the run,
unless `--strict-revisions` is given.

## Cases and expected outcomes

Measured on core `devel` 658fe99, single-threaded BLAS, `figaroh-dev`.

| Case | Config overlay | Scope | Expected |
|---|---|---|---|
| `ur10 truth` | `ur10/config/ur10_truth_reference.yaml` | prediction | `physical_fit` accepted ("cvxopt: optimal; all links feasible"); every joint's held-out RMSE within `config/truth_reference_acceptance.json`; export reloads to the fit within 1e-6 N·m; reloaded URDF within 0.1 N·m of the noise-free truth effort on the held-out split; archive audit complete |
| `tiago physical-fit` | `tiago/config/tiago_reference_physical_fit.yaml` | execution | `physical_fit` accepted, cvxopt optimal; execution check passes; export reloads to the fit within 1e-6 N·m; archive audit complete; held-out RMSE reported, not gated |
| `tiago reject` | `tiago/config/tiago_reference_reject.yaml` | execution | `reconstruction` requested and rejected (infeasible links); `selected_stage` is `none`; the verdict fails; `export_urdf` raises; the run directory has `fit_parameters.csv` and no `parameters.csv`; the audit's only gap is `export` |

The overlays `extends` the unified config and set only
`tasks.identification.select_stage`, so the config hash in the provenance
record is the overlay's and the rest stays the robot's own.

### UR10 truth (`--noise low`, default)

Training split `train.csv` with noise seed 101, held-out `validation.csv`
with seed 201 (`data/truth`, #21); the fixture's analytic derivatives are
used as given. Runtime about 3 s.

The gate is the committed profile: per joint `validation_rmse <= 1.25 *
sigma_low + 0.01` N·m, with `sigma_low` from `data/truth/protocol.yaml`
(a test keeps the file equal to the protocol). It is a noise-floor bound, not
a tuned one. With noise the OLS base is not physically consistent, so the
constraints actually act; with `--noise none` the physical fit reproduces OLS.

| Joint | Train RMSE | Held-out RMSE | Limit | Nominal model |
|---|---|---|---|---|
| shoulder_pan | 0.0352 | 0.0406 | 0.0532 | 0.807 |
| shoulder_lift | 0.3861 | 0.3728 | 0.4850 | 7.060 |
| elbow | 0.2727 | 0.2600 | 0.3424 | 2.627 |
| wrist_1 | 0.0291 | 0.0304 | 0.0453 | 0.740 |
| wrist_2 | 0.0051 | 0.0062 | 0.0133 | 0.407 |
| wrist_3 | 0.0035 | 0.0039 | 0.0108 | 0.022 |

(N·m. Held-out RMSE is against the noisy held-out measurement.) Further:
solver cvxopt optimal in 0.07 s; export parity 7.9e-11 N·m; reloaded URDF
against the noise-free held-out effort, 0.037 N·m at most (per joint
0.0198 / 0.0374 / 0.0134 / 0.0098 / 0.0054 / 0.0038); base-parameter error
against the truth in the protocol's scaling, 0.028 N·m max and 0.0098 RMS.

### TIAGo physical fit

Training is the `dynamic` recording (rows 921-6791, decimated, velocity from
positions, effort of arm_1-arm_4 fitted, #68); held-out is `calibration_slow`
(role `validation`, checked by content hash against
`data/identification/protocol.yaml`). Runtime about 17 s; the physical fit
itself 2.1 s with cvxopt (QICS is not needed); export parity 6.1e-10 N·m.

| Joint | Train RMSE | Held-out RMSE | Nominal model | Held-out nRMSE |
|---|---|---|---|---|
| arm_1 | 1.040 | 1.303 | 3.187 | 0.418 |
| arm_2 | 1.148 | 2.539 | 7.838 | 0.420 |
| arm_3 | 0.713 | 1.446 | 3.122 | 0.366 |
| arm_4 | 0.588 | 1.226 | 2.858 | 0.257 |

(N·m.) Not fitted and not scored: torso_lift, arm_5, arm_6, arm_7 (#68).
Overall held-out RMSE 1.713 (nominal 4.730); arm_2-arm_4 pooled 1.829, the
scale-free metric of #68 (1.839 for the base fit). These are **reported in
`reference.json`, not gated**: the maintainer set the bar at the execution
scope. arm_1's effort scale is unidentifiable (#68), so its relative error is
not a pass criterion.

### TIAGo reject

The exact reconstruction is requested on the same data. Its nullspace
representative of the base fit is not physical (lowest pseudo-inertia
eigenvalue -3.67 over 48 links; arm_1-arm_7 among the infeasible), so it is
rejected and nothing is substituted: the held-out numbers in
`reference.json` are the base fit's (`estimate_stage: fit`, 1.722 overall,
1.839 pooled). This is the hard-reject contract, consistent with exact
reconstruction being infeasible on this data. About 14 s.

## Export target: the lumped nominal URDF

Pinocchio merges links attached by fixed joints into the moving body. The
UR10 `wrist_3` body holds the tool mount and camera; the TIAGo `arm_7` body
holds the hand and the F/T sensor. The fit identifies the whole body, so its
mass can be below the CAD mass of the links fixed to it, and the exporter
(`refuse`, its default) will not split it.

`identification_reference.lumped_nominal_urdf` therefore writes, at run time
into the run directory, a copy of the nominal URDF where each moving link
carries its whole Pinocchio body inertia and the fixed-attached links have no
inertial. Pinocchio builds the same dynamics from it (difference below 1e-9,
checked at generation). `export_urdf` then writes the fitted inertials into
that file. The parity check compares the reloaded `identified.urdf` with the
nominal model whose body inertias are set directly from the selected
estimate (`reference_model`), by RNEA over 200 seeded random states. Hey5
finger bodies count as identified and get their inertials written, which is
harmless.

## Run directory

Besides the standard archive (`provenance.json`, `config.snapshot.yaml`,
`stages.json`, `verdict.json`, `parameters.csv`, `report.html`,
`reproduction.json`):

| File | Content |
|---|---|
| `reference.json` | per joint: unit, fitted or not, train RMSE, held-out RMSE (selected and nominal), nRMSE; solver status and runtime; per-link `{mass, min_eig, ok}`; inputs with sha256, rows and seeds; held-out extras; check that training and held-out files differ |
| `export_check.json` | nominal and lumped nominal hashes and generator, exported URDF hash, identified joints, parity and tolerance, or the refusal |
| `nominal_lumped.urdf`, `identified.urdf` | the export target and the export |

## Limitations

- Offline logs only; nothing here is a deployment claim.
- arm_1's effort scale is unidentifiable (#68); torso and wrist efforts are
  not fitted.
- `select_stage: projected` is not selectable (#163).
- The UR10 truth is simulated with a known model; its bound is a noise-floor
  check, not evidence about a physical unit.
- Last-bit solver differences between platforms: every gate is a tolerance.
- Run TIAGo cases sequentially; parallel runs have hung.
