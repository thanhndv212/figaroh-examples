# TIAGo calibration reference workflow (C4)

The accepted calibration reference: one headless command fits TIAGo's arm
from motion-capture data, reports every frozen held-out session, exports the
corrected URDF and the PAL file, reloads them, and archives the run with what
is needed to reproduce it (examples#29).

```bash
cd examples/tiago
python reference_run.py                      # archive under results/runs/
python reference_run.py --asset-id TIAGO-48 --root /path/to/runs
```

Run it from a clean checkout of both repositories: the archive audit marks a
run from uncommitted code `incomplete` and the command exits nonzero.

## What it does

| Step | Content | Written to the run directory |
|---|---|---|
| 0. Inputs | Every session file matches the frozen protocol manifest (`data/calibration/mocap/protocol.yaml`, version 1): sha256 and role. The config's training and validation files must be the protocol's. | (stops before writing) |
| 1. Fit | `joint_offset`, marker 1, base frame (6D) and marker point (3D) estimated, on the training session; the validation session is loaded as `validation_data_file`. | `report.html`, `parameters.csv` |
| 2. Held-out report | Marker-1 position error per component (x, y, z, mm), RMSE and max, by posture stratum (repeated / new / out of range), on all four protocol sessions. | `heldout.json` |
| 3. Gauge and corrections | Estimated frames; joint parameters the frames absorb; fitted (identifiable) parameters with standard errors; the joint corrections written to the URDF/PAL file (redistributed); solver status. | `corrections.json` |
| 4. Export and reload | `export_urdf` and both PAL files (full and ≥ 2σ). The URDF, and the nominal URDF with the PAL deltas applied, reloaded with the metrology frames, must predict the fitted marker on every session within 1e-9 m; only corrected joint origins may differ from the nominal file. Recorded as the verdict's export stage. | `calibrated.urdf`, `master_calibration{,_conservative}.yaml`, `export_check.json` |
| 5. Verdict and archive | Scoped verdict (`execution` by default), provenance, config snapshot, reproduction record; the archive audit must find nothing missing. | `verdict.json`, `provenance.json`, `reproduction.json`, `stages.json` |

The command exits 1 when the verdict fails, the export does not reload to
the fit, or the audit finds anything missing.

## Reference result

figaroh-examples `1c330a4` + figaroh-plus `c80b5fc` (`devel`), `figaroh-dev`,
Python 3.12, Pinocchio 3.7.0, macOS arm64, single-threaded BLAS. Exit 0;
every audit item `OK`.

Held-out report (marker 1, mm):

| Role | Session | n | x | y | z | RMSE | max | repeated | new | out of range |
|---|---|---|---|---|---|---|---|---|---|---|
| training | 2021-11-30 | 37 | 1.78 | 1.59 | 1.61 | 2.88 | 5.55 | 2.88 (37) | — | — |
| validation | 2021-11-26 | 62 | 2.37 | 2.59 | 2.35 | **4.23** | 11.95 | 3.31 (37) | 4.47 (16) | 6.53 (9) |
| confirmation | 2021-11-30-1403 | 63 | 2.32 | 2.56 | 2.20 | **4.09** | 12.24 | 3.08 (37) | 4.29 (17) | 6.61 (9) |
| confirmation | 2021-11-30-1504 | 59 | 2.32 | 2.31 | 1.98 | **3.83** | 12.18 | 2.84 (35) | 4.13 (16) | 6.21 (8) |

Nominal model on the validation session: 419.39 mm (the mocap frame is
unknown before the fit).

Gauge: base frame `base_p{x,y,z}`, `base_phi{x,y,z}` and marker point
`pEE{x,y,z}_1` estimated; `offsetPZ_torso_lift_joint` and
`offsetRZ_arm_1_joint` absorbed by the base frame (figaroh-plus#102).

Identifiable parameters (mrad, ± standard error, residual dof 97):

| arm_2 | arm_3 | arm_4 | arm_5 | arm_6 |
|---|---|---|---|---|
| −2.35 ± 2.59 | 1.00 ± 2.42 | −3.74 ± 2.58 | **−49.82 ± 10.08** | 2.88 ± 7.93 |

Solver: `ftol` reached, 5 evaluations. Export: reloaded URDF and PAL file
predict the fit within 2.1e-12 m on every session; 5 joint origins changed,
nothing else.

## Limitations

- **Marker 1 only.** Fitting all four rigid-body points is supported
  (figaroh-plus#119) but did not predict held-out postures better here
  (`joint_offset`: 3.97 against 4.06 mm on marker 1); each extra point also
  brings its own occlusion and reflection errors. Not adopted.
- **Model mismatch.** Held-out error grows from repeated postures (≈ 3 mm)
  to new (≈ 4.3 mm) and out-of-range postures (≈ 6.5 mm): effects the
  `joint_offset` model does not have (backlash, deflection) dominate.
  `full_params` predicts better on these sessions but overfits a known truth
  at 37 postures ([synthetic truth](tiago-calibration-synthetic-truth.md)).
- **Metrology frames are not robot geometry.** The base frame and marker
  point are written to `corrections.json`, not to the URDF or PAL file
  ([export check](tiago-calibration-export.md)).
- **Verdict scope.** `execution` checks numerical outputs, the solver and the
  export. `prediction` additionally needs an acceptance profile
  (`--acceptance-profile`); none is set for TIAGo, so prediction stays
  `NOT_EVALUATED` and the held-out numbers above are evidence, not a pass.
- No hardware was commanded; the URDF and PAL file are not deployed.

## TALOS: regression and contact consumer

TALOS is not a reference workflow. It stays a separate consumer that core
calibration changes must not break:

- `talos/calibration_upperbody.py` (mocap, `full_params`,
  `estimation.method: map` since figaroh-plus#120) is pinned by the golden
  outputs (`tests/golden/`): parameter count, fit RMS, exported deviation.
  No held-out session exists for it.
- `talos_table_contact/` (plane contact, one or two chains) is pinned by
  `tests/test_talos_table_contact*.py`,
  `tests/test_talos_synthetic_determinism.py` and
  `tests/test_talos_contact_semantics.py`. On the real data, the table height
  and the contact offset are not separately identifiable: their fitted
  corrections cancel (left chain −105 / +105 mm, right −74 / +77 mm), so the
  example's own regulariser, not the data, splits them around the assumed
  1.00 m table (measured for figaroh-plus#120; setup in the
  [contact audit](talos-contact-calibration-audit-2026-10-04.md)).

## Related

- [Held-out protocol](tiago-mocap-heldout-protocol.md): sessions, roles,
  rules; `heldout_protocol.py` compares registration only, `joint_offset`
  and `full_params`.
- [Export check](tiago-calibration-export.md): URDF and PAL parity for both
  levels.
- [Run archive audit](run-archive-audit-2026-10-06.md): the reproduction
  record and `python -m examples.run_record <run_dir>`.
- Tests: `tests/test_tiago_reference_run.py` runs the command and checks the
  artifacts, the export parity, the gauge and that the held-out numbers are
  the protocol's.
