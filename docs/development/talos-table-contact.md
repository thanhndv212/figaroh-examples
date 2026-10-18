# TALOS table-contact calibration: held-out study

Issue: [#98](https://github.com/thanhndv212/figaroh-examples/issues/98).
Core hooks: figaroh-plus [#210](https://github.com/thanhndv212/figaroh-plus/issues/210)
(PR #212) and [#211](https://github.com/thanhndv212/figaroh-plus/issues/211)
(PR #213). Core used for every number below: figaroh-plus devel `b575b5b`.

This brings the TALOS table-contact study to the C5 standard (the one of
[TIAGo Pro](tiago-pro-mocap.md) and [TIAGo](tiago-mocap-heldout-protocol.md)):
a fixed split, explicit estimation methods, a table-only baseline, a
synthetic-truth study with an encoder-noise grid, an archived reference run
and a golden output. Code is in `examples/talos_table_contact/`. The checks
are `tests/test_talos_table_contact_{estimation,heldout,generator,truth,reference_run}.py`
and `tests/test_talos_two_chain_draw_invariance.py`; the golden values are
`tests/golden/talos_table_contact.json`. The example itself is described in
[`examples/talos_table_contact/README.md`](../../examples/talos_table_contact/README.md).

## 1. Data and defects

Static postures of TALOS whose gripper was commanded to touch a table flush.
Provenance, hashes and the header fix are in
[`data/real/README.md`](../../examples/talos_table_contact/data/real/README.md);
roles are frozen in `data/real/protocol.yaml` (protocol v2), and the per-chain
plan views the harness reads are `data/real/protocol_left.yaml` and
`data/real/protocol_right.yaml` (copies of the master entries plus a
`heldout:` block; a test pins them to the master).

| File | Postures | Session | Role |
|---|---|---|---|
| `left_train.csv` | 21 | 2022-10-28 | training |
| `left_validation.csv` | 9 | 2022-11-07 | validation |
| `right_train.csv` | 29 | 2022-11-07 | training |
| `right_validation.csv` | 9 | 2022-11-07 | validation |

Defects that shape the study:

- The measurement is a contact: height, roll and pitch of the gripper frame
  against a plane, with zero expected gap. There is no mocap and no
  orientation reference beyond the plane.
- The nominal table is about 0.29 m away from the real one, so the nominal
  model starts with a 280-290 mm gap (section 4, anchoring).
- Two postures are exact repeats inside a training file (`table/contact_21`
  in `left_train.csv`, `table/contact_20` in `right_train.csv`). No held-out
  posture repeats a training posture (strata: left 5 new / 4 out of range,
  right 4 new / 5 out of range; golden `strata`).
- The left chain trains on 2022-10-28 and validates on 2022-11-07. If the
  table or the robot base moved between the days, the training table cannot
  know it. Nothing in the data records such a move.
- The Delta-X count of the structural model is 50 on the left chain and 57 on
  the right (golden `n_dx`, pinned by `test_structural_unchanged`). The
  manuscript's 57 for this chain is the rank found with enough random probes
  (the example README, "Structural check"); why the shipped left selection
  keeps 50 with 21 postures was not investigated here.

## 2. Split choice

Training is the first file of each chain, validation the second. For the left
chain the two files are different days; for the right chain they are the same
day (a within-session split, weaker evidence). There is no confirmation set,
so nothing is selected on validation: the config default stays
`structural` + `ordered`
([figaroh-plus#204](https://github.com/thanhndv212/figaroh-plus/issues/204)),
and any "best" model named below is informational. The harness raises if
asked to `select` on the confirmation role.

## 3. Models and metric

`heldout_protocol.MODELS` (all single chain, single session):

| Model | What it is |
|---|---|
| `table+contact only` | baseline: plane z/roll/pitch and contact roll/pitch (5 parameters), no Delta X, no ridge, `contact_z` fixed |
| `structural` | the config default: identifiable Delta X (ordered basis) plus table and contact, ridge 1e-3 |
| `map` | every joint parameter estimated with a zero-mean Gaussian prior (core `estimation.method`); nothing dropped |
| `map_cv` | `map` with the prior sizes scaled by a factor chosen by k-fold CV |
| `excitation` | start from all joint parameters, drop the worst-determined one at a time while its relative standard error exceeds `excitation_k` |
| `cv_subset` | nested sets in the `excitation` removal order; the size is chosen by k-fold CV over 7 sizes |
| two-chain structural | diagnostic only (shared torso axes, 115 parameters) |

The non-structural methods use the core hooks of figaroh-plus #210
(`per_sample_attributes`, `nuisance_param_names`, `measurement_jacobian`;
PR #212) and #211 (PR #213): the calibrator names its nuisance parameters
(plane per session, contact tilt), supplies the exact measurement Jacobian
(analytic gap derivative chained onto `chain_jacobian`; matches a central
difference to about 1e-10 relative), and core builds the priors. They need a
single session, so the two-chain fit is structural only.

**Primary metric: transfer.** The held-out z is the RMS of the gap of the
validation postures under the training table. Reported next to it:

- *re-registered*: the 3 plane parameters are refitted on the 9 evaluated
  postures. It is in-sample for those 3 parameters, so it is optimistic;
- *train*: the fit's own RMS on the training postures.

`noise_std = 1e-3` is one scalar for the mixed units (metres of height and
radians of roll/pitch) and is given explicitly, not estimated.

## 4. Gauge and anchoring

**Gauge.** A flush-contact gap cannot separate `contact_z` from `plane_z`
(both shift the height), nor the leg z-translations from the plane height.
`contact_z` is fixed at its nominal value and `plane_z` absorbs its error, so
the fitted `plane_z` is not the true table height. For the same reason
per-parameter z-scores are waived (`z_params` is empty in the simulation
study); the score is the gauge-invariant height error of the predicted gap.
x, y and yaw of the table are unobservable.

**Anchoring.** The structural ridge (1e-3) pulls the plane parameters toward
their anchor, so with the nominal table 0.29 m off, the previously frozen
numbers carried that bias. Real-data chains now start from the table-only fit
on their own training file (commit 0fa3785). The frozen #204 / #136 numbers
moved (structural, ordered, held-out z in mm):

| | before (nominal start) | after (anchored) |
|---|---:|---:|
| single chain left | 7.55 | 7.73 |
| single chain right | 7.69 | 7.58 |
| two-chain left | 5.098 | 5.154 |
| two-chain right | 8.394 | 8.774 |

`_build_chain(anchor=False)` keeps the old behaviour and a test pins the old
values. The contact offset stays nominal.

## 5. Real held-out results

Validation RMS in mm (z) and degrees (roll, pitch) on the held-out session;
golden block `chains` of `tests/golden/talos_table_contact.json`.

### Left chain (training 2022-10-28, validation 2022-11-07)

| Model | Params | Transfer z | Transfer roll / pitch | Re-registered z | Train z |
|---|---:|---:|---:|---:|---:|
| table+contact only | 5 | 8.381 | 0.621 / 0.677 | 8.129 | 13.434 |
| structural | 56 | 7.731 | 0.731 / 0.515 | 6.108 | 0.601 |
| map | 95 | 4.809 | 0.459 / 0.226 | 2.834 | 1.533 |
| map_cv | 95 | **4.599** | 0.490 / 0.309 | 2.793 | 2.683 |
| excitation | 26 | 5.304 | 0.422 / 0.179 | 4.842 | 5.457 |
| cv_subset | 22 | 5.503 | 0.473 / 0.180 | 4.963 | 5.407 |
| two-chain structural | 115 | 5.154 | 0.458 / 0.491 | 3.390 | 1.409 |

### Right chain (training and validation 2022-11-07)

| Model | Params | Transfer z | Transfer roll / pitch | Re-registered z | Train z |
|---|---:|---:|---:|---:|---:|
| table+contact only | 5 | 18.233 | 1.911 / 1.035 | 15.340 | 11.775 |
| structural | 63 | **7.580** | 0.998 / 0.577 | 7.836 | 1.234 |
| map | 95 | 7.707 | 0.916 / 0.584 | 5.710 | 2.987 |
| map_cv | 95 | 11.565 | 1.215 / 0.605 | 7.350 | 6.184 |
| excitation | 32 | 10.320 | 1.017 / 0.598 | 7.840 | 5.076 |
| cv_subset | 8 | 15.909 | 1.287 / 0.633 | 12.745 | 8.824 |
| two-chain structural | 115 | 8.774 | 0.998 / 0.584 | 9.079 | 1.312 |

Bold marks the lowest transfer z (informational, see section 2).

### Reading

- **Gain over the table-only baseline (transfer z).** Left: structural only
  0.65 mm (8.381 to 7.731), map 3.57 mm, map_cv 3.78 mm. Right: structural
  10.65 mm (18.233 to 7.580), map 10.53 mm, map_cv 6.67 mm.
- **Training-to-held-out gap (transfer minus train).** Left structural
  7.13 mm (0.601 to 7.731): it fits the 21 training postures and transfers
  almost like the 5-parameter baseline. map closes it to 3.28 mm and map_cv to
  1.92 mm. Right structural 6.35 mm.
- **Right chain.** map does not beat structural (7.707 vs 7.580) and map_cv is
  worse (11.565). The right validation postures are the same day as training,
  so the data does not show a day effect, yet the prior-driven methods lose
  to the ordered structural basis there. No cause was established.
- **Re-registered is lower than transfer** for every model with the left chain
  (for example structural 7.731 to 6.108), the sign of a table offset between
  the two days that the training table cannot know; it also is in-sample for
  3 parameters, so treat it as a lower bound.
- The two-chain structural fit helps the left chain (5.154 vs 7.731) and hurts
  the right one (8.774 vs 7.580). It is a diagnostic.

## 6. Simulation study

`calibration_truth.py`: per chain, a truth is drawn around the anchored real
table (6D errors on the 15 chain joints, table and contact draws); the real
recorded postures are projected onto the true surface and re-solved by IK, so
the study keeps the real posture counts (21/29 train, 9 + 9 validation);
Gaussian encoder noise (deg) is added on the chain joints; each method fits.
The score is the height error of the fitted gap against the true gap at the 9
real validation postures. Surfaces: `planar` (the model class) and `curved`
(a bowl, 1 mm sag at the 90th-percentile contact radius). The encoder-noise
grid {0.01, 0.05, 0.1} deg is assumed: there are no repeated postures to
estimate it from. 5 seeds per cell.

Held-out height error, mm, median [min-max] over seeds 0-4
(the harness train RMS is in the golden and the full summary):

| Chain | Noise (deg) | table+contact only | structural | map | map_cv | excitation | cv_subset |
|---|---:|---|---|---|---|---|---|
| left | 0.01 | 2.746 [2.215-3.623] | 2.556 [1.530-3.434] | 0.449 [0.339-0.642] | 0.341 [0.187-0.518] | 1.982 [1.412-2.541] | 1.982 [1.412-2.687] |
| left | 0.05 | 2.733 [2.087-3.479] | 3.068 [1.890-3.998] | 0.936 [0.500-1.482] | 1.102 [0.534-1.398] | 2.253 [1.540-2.599] | 2.253 [1.540-2.536] |
| left | 0.10 | 2.719 [1.910-3.379] | 3.968 [2.695-4.904] | 1.480 [0.883-2.743] | 1.628 [0.861-2.188] | 2.667 [1.709-3.262] | 2.520 [1.709-2.762] |
| right | 0.01 | 4.104 [3.526-7.720] | 0.561 [0.463-0.651] | 0.534 [0.433-0.795] | 0.481 [0.363-0.625] | 1.686 [1.100-3.087] | 0.472 [0.351-1.059] |
| right | 0.05 | 4.185 [3.648-7.777] | 2.706 [2.076-3.241] | 0.934 [0.497-1.428] | 1.105 [0.695-1.456] | 1.915 [1.111-2.968] | 1.365 [0.772-2.349] |
| right | 0.10 | 4.289 [3.817-7.854] | 4.812 [4.221-6.459] | 1.782 [0.912-2.567] | 2.121 [1.356-2.282] | 2.184 [1.434-3.627] | 1.999 [1.488-3.631] |
| left curved | 0.05 | 2.799 [1.941-3.634] | 3.025 [1.970-4.360] | 0.876 [0.813-1.354] | 1.040 [0.731-1.365] | 2.211 [1.476-2.701] | 2.211 [1.476-2.675] |
| right curved | 0.05 | 4.098 [3.645-7.542] | 2.858 [1.847-3.836] | 0.912 [0.635-1.970] | 1.598 [0.669-2.120] | 1.849 [0.929-3.547] | 1.521 [0.647-3.545] |

Observations:

- **Structural overfits on the left chain.** With 21 postures it is no better
  than the 5-parameter baseline (left 0.05 deg: 3.07 vs 2.73 mm) and gets
  worse as noise grows (2.56, 3.07, 3.97 mm). On the right chain with 29
  postures it is good at 0.01 deg (0.56 mm) and degrades the same way.
- **map and map_cv are about 3 times better** than structural at 0.05 and
  0.10 deg (left 0.05: 0.94 / 1.10 vs 3.07 mm; right 0.10: 1.78 / 2.12 vs
  4.81 mm). Between them, map is not consistently worse than map_cv.
- **excitation and cv_subset** sit between the baseline and map.
- **The curved 1 mm sag is harmless** for the planar model: curved cells at
  0.05 deg are within the seed spread of the planar ones, but a 1 mm effect is
  also small against the noise-driven errors, so this does not show the plane
  assumption is safe at larger sag.
- **Noise-free mechanism check (planar, seed 0).** Structural held-out is
  2.31 mm (left) and 0.14 mm (right) with exact data: 21/29 postures do not pin
  56/63 parameters and validation postures extrapolate, so the planned 0.05 mm
  bound was dropped (commit 6e1f869). The ridge-off training gap is about
  1e-11 mm and noise-free map_cv held-out is below 0.25 mm on both chains
  (L 0.196, R 0.149 mm).
- **Real held-out errors are above every simulated level.** The real left
  transfer (4.6-7.7 mm) and right (7.6-18 mm) exceed the simulated errors at
  0.10 deg, so unmodelled effects dominate over encoder noise (candidates such as a
  table or base move between days were not tested). The simulation ranks methods under the model class; it does
  not predict the real error.

The seed-0 block is pinned in the golden (`synthetic`), with structural and
cv_subset counts skipped when the platform ties differently.

## 7. Draw spread

The held-out result of `structural` with `ordered` selection is identical for
every `random_seed` (the slow test compares seeds 0 and 1 on both chains).
The non-structural methods ignore
`random_seed` altogether: with them the random configuration draw is disabled
at initialisation, so the draw cannot reach them, and the CV folds use
`estimation.cv_seed`. The held-out spread over draws is 0 (commit be9a1bc).

## 8. Sensitivity to noise_std

`noise_std` sets the prior scale of `map` and `map_cv`. Transfer z, mm, with
`noise_std` times 0.3 and times 3 around the default 1e-3 (one run per cell,
real data, not part of the golden):

| Chain | Model | noise_std 3e-4 | 1e-3 (default) | 3e-3 |
|---|---|---:|---:|---:|
| left | map | 5.465 | 4.809 | 4.531 |
| right | map | 9.102 | 7.707 | 8.896 |
| left | map_cv | not run | 4.599 | not run |
| right | map_cv | not run | 11.565 | not run |

`map_cv` was skipped: its first fit (left, 3e-4) was still running after
more than 10 minutes, against 19 s for `map`, so the rule to skip it was
applied for all four cells. The `map` transfer z changes by up to 1.4 mm over
the 10x range (left 4.53-5.47 mm, right 7.71-9.10 mm), small
against the 3.5 mm left gain over the baseline but not negligible on the
right chain, where 7.7 mm at the default becomes 9.1 and 8.9 mm. The default
is not the best on either chain (the lowest left value is at 3e-3, the lowest
right value at 1e-3); this is informational, nothing is selected on it.

## 9. Limits and risks

- No confirmation set and one validation session per chain: any ranking is
  optimistic. The right chain's validation is the same day as training.
- The left transfer error may include an unrecorded table move between the
  two days.
- Held-out postures are real postures, not exactly on the table, so the
  metric is a difference of height predictions, not the contact gap itself.
- `noise_std` is one scalar over metres and radians. The encoder-noise grid is
  assumed.
- The structural ridge bias on the 0.29 m offset is removed by anchoring;
  structural keeps its ridge for backward compatibility.
- The estimation route relies on the core hooks of #210 / #211; the
  calibrator also reads core's private helpers in places, which is brittle
  across core releases.
- `excitation` and `cv_subset` take minutes per fit (accepted).
- `reference_run.py` and `validate.py` run baseline, structural and map only.
  The full six-model run is `heldout_protocol.py` (offline) and the slow
  golden test.

## 10. Reproduction

From `figaroh-examples`, inside the `figaroh-dev` environment, headless
(`MPLBACKEND=Agg`):

```bash
cd examples/talos_table_contact
python heldout_protocol.py                        # real held-out tables (section 5)
python calibration_truth.py --jobs 4              # planar grid (section 6)
python calibration_truth.py --mismatch --jobs 4   # curved cells
python reference_run.py --no-plot                 # baseline + structural + map, archived run
cd ../..
python -m pytest tests/test_talos_* -q            # fast
python -m pytest tests/test_talos_* -q -m slow    # golden, reference run, draw spread
python validate.py --robot talos_table_contact    # includes reference_run.py
```

Regenerate the golden with
`python tests/test_talos_table_contact_heldout.py > tests/golden/talos_table_contact.json`
only after a deliberate change, and review the diff.
