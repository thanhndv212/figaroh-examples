# TALOS torso-arm mocap calibration: held-out study

Issue: [#172](https://github.com/thanhndv212/figaroh-examples/issues/172).
Delivery package: C5 ([core #167](https://github.com/thanhndv212/figaroh-plus/issues/167)).
Data: [#114](https://github.com/thanhndv212/figaroh-examples/issues/114) /
PR #147, `examples/talos/data/calibration/mocap/` (described in
[examples/talos/data/README.md](../../examples/talos/data/README.md)).
Validation level: V2.

This brings the TALOS Qualisys sessions of 2022-03-15 to the C5 standard, the
one of [tiago-pro-mocap.md](tiago-pro-mocap.md):

- frozen data and a fixed split;
- a pre-registered design;
- a simulated fixture;
- a registration (frames-only) baseline;
- an explicit estimation method chosen without the confirmation run;
- a comparison with the table-contact study ([talos-table-contact.md](talos-table-contact.md), #98);
- an archived reference run and a golden output.

**Pre-registration.** Sections 1 to 4 and the code were committed before any
fit of the real data. The git history is the gate: the commit that adds this
note has no real-data result, and section 5 was written after it. Before
that commit, the real files were only read by fit-free rules:

- the attachment invariant;
- the flip rule;
- core's identifiability check on the training postures, which uses no measurement.

## 1. Data and the points PR #147 left open

| Run (2022-03-15, UTC) | Arm | Rows | Flipped rows (fit-free) | Role |
|---|---|---|---|---|
| 13:54 | left | 56 | none | training |
| 14:18 | right | 39 | 14 in three clusters: rows 7-9, 21-22 (5); 10, 14, 35-38 (6); 15-16, 28 (3) | validation |
| 14:39 | right | 23 | 1 (row 0, 176 deg) | confirmation |

The `protocol.yaml` file is now v2. The files and their sha256 are unchanged.

**Attachments.** These are established fit-free and tested in
`test_attachment_links_from_rotation_angles`:

- the waist body is on `base_link`;
- the upper body is on `torso_2_link`;
- the end-effector body is on `arm_<side>_7_link`.

The test uses a conjugation invariant. Take the rotation of a body in the
waist frame between rows i and j. Its angle equals the angle of the nominal
rotation of the body's link between the same rows, because the unknown mount
rotations cancel. Measured against this:

- The upper body matches `torso_2_link` to 0.17 deg RMS. Every other link is off by 8 deg or more.
- The end-effector matches `arm_<side>_7_link` to 0.8 to 1.2 deg. `arm_<side>_6_link` is off by 15 deg or more.

v1 said the waist "moves with the torso". It does not: it moves with the
base, which moves in the lab by about 0.2 m over a run.

**Base pose.** The base pose in the world is not recorded, and it is not
needed. The measurement is the end-effector body in the waist frame, and the
waist rides on `base_link`. The body offsets are estimated, as core's base
and tool frames.

**Sensor hardware** is unknown: the Qualisys model, cameras, markers and
calibration quality. This is a disclosed limit, and nothing is assumed
about it.

**Attribution:** recorded at LAAS-CNRS by @thanhndv212. Apache 2.0, as the
repository.

**Flipped orientations.** On some rows of the right-arm runs, the
end-effector body's orientation jumps by 90 or 180 deg: the marker body was
re-identified with its axes permuted. A row is flipped when its body
rotation relative to `arm_<side>_7_link` is more than 45 deg from the run's
medoid. The rule uses the nominal model and needs no fit. On the shipped runs:

- Kept rows are within 11.4 deg of the medoid, in every run.
- Flagged rows are 90 deg or more away.
- Training has no flipped rows. The loader refuses a training set that has any.

Flipped rows within 45 deg of one another form a cluster. The study repairs
each cluster (section 2) rather than dropping it. The confirmation run's
cluster is a single row. Repairing it would let its 3 rotation parameters
absorb the row, so a cluster of fewer than `MIN_CLUSTER = 3` rows is dropped
from the metrics and listed. This rule was set after the flagged-row counts
of all three runs were seen, which is fit-free, and before any fit.

**Posture coverage.** The torso ranges of the training and validation runs
overlap:

- training: t1 in [-0.91, 0.34], t2 in [-0.13, 0.69];
- validation: t1 in [-0.31, 0.77], t2 in [-0.13, 0.68].

The right-arm runs are the other arm. Strata are over the torso joints only
(repeated, new, or out of range by 0.05 rad), plus flipped and unflipped.

## 2. Measurement model

Each row's measurement is `Y = inverse(waist) * endeffector`, a 6D pose.
It is modelled as `Y = B * FK(q) * T * S_k`:

- `B`: `base_link` in the waist body frame. This is core's base frame (`base_*`).
- `FK`: `base_link` to `arm_<side>_7_link`, through torso_1, torso_2 and arm_<side>_1..7.
- `T`: the end-effector body on `arm_<side>_7_link`. This is core's tool frame (`pEE*_1`, `phiEE*_1`).
- `S_k`: the repair of flip cluster k. It is the identity for the main
  cluster, and `(R_k, (I - R_k) c)` otherwise: a rotation `R_k` about a
  centroid `c` shared by the run.

The residual is the world-frame log map, position in metres and the rotation
vector times `LEVER = 0.1 m`. That lever gives position and orientation
about the same weight. The training run's torso registration has about
0.17 mm of noise per axis and about 1.6 mrad of rotation noise.

**Identifiability.** These counts come from core's check on the training
postures (structural, ordered), which uses no measurement.

- `full_params`: 44 parameters, or 42 in the fixture, whose frames differ from zero:
  - the frames: 12;
  - torso_2: `d_px`, `d_py`, `d_phix`, `d_phiy`;
  - each of arm_left_1..7: 4;
  - torso_1: none; the base frame absorbs it.
- `joint_offset`: 21 parameters: the frames plus the 9 joint-angle offsets.

## 3. Pre-registered design

### 3.1 Models

All models are fitted on the training run (`mocap_heldout.MODELS`). The
noise level is estimated by core (`noise_std` unset).

| Model | Parameters | Estimation |
|---|---|---|
| `registration only` (baseline) | `B`, `T` | structural, ordered |
| `joint_offset` | frames + joint-angle offsets | structural, ordered |
| `full_params` | frames + placement errors | structural, ordered |
| `full map` | frames + every placement error | map, core's default priors |
| `full map_cv` | same | map_cv, same priors |

`cv_subset` and `excitation` are not run: they cost more, and the TALOS
table study found no gain from them. The config
(`config/talos_mocap_config.yaml`) carries an explicit
`parameters.estimation.method`. Its final value is the model the k-fold plan
selects (3.3), set in a commit after the selection. It is never set from the
validation or confirmation runs.

### 3.2 Metrics

- **K-fold** (`protocol_1354_kfold.yaml`): 5 row folds of the training run,
  seed 0. Each fold's own fit, frames included, predicts its held rows: same
  session, same mounting. The metric is `pos_rmse_mm`, the position RMSE
  (mm) of the end-effector body in the waist frame. Orientation is reported
  in degrees.
- **Right-arm runs.** The model's torso parameters are carried over. The
  right arm is nominal (no right-arm data trains anything), and the flips
  are repaired by the registration:
  - primary: `pos_rmse_rereg_mm`, with `B`, `T` and the flip repair refitted on the run;
  - secondary: `pos_rmse_waist_mm`, with `B` kept from training and only `T`
    and the repair refitted. This tests that the waist mount is the same body
    on the same link.
  - Also reported: the unflipped rows alone (no repair), the fitted flip
    angles, axes and centroid, the strata, and a gauge-free diagnostic in
    which only `d_px_torso_2_joint` and `d_phix_torso_2_joint` are carried.

### 3.3 Selection and criteria

- **Selection:** the model with the lowest mean k-fold `pos_rmse_mm`
  (`harness.select` on the k-fold results). The harness raises if asked to
  select on confirmation.
- **C1, the within-session fit:** the selected model's k-fold error must be
  below the baseline's k-fold error. Its k-fold over mean fold-training
  ratio must also be at most 1.5; above that, it is reported as overfitting.
- **C2, carrying the torso to the other arm:** on the validation run,
  compare the selected model's primary metric with the baseline's:
  - helps: at most 0.9 times the baseline;
  - neutral: within ±10%;
  - hurts: more than 1.1 times.
  
  The same verdict is reported on confirmation, with no change to anything.
  **Expected from the fixture (section 4): neutral.** Carried torso errors
  only show on the right arm if the right arm is near nominal. With right-arm
  errors of the fixture's size, every model reaches the same re-registered
  error.
- **C3, against table contact (#98):** run `mocap_table_comparison.py`
  (section 3.4). Its verdict is a reported result. It is not a gate on C1 or
  C2.

### 3.4 C3 against table contact

The table-contact study fits `left_sole_link -> gripper_left_base_link`,
recorded 2022-10-28, seven months later. Its reference model is structural.
Its torso and left-arm placement errors name the same physical errors as the
mocap fit's. Their values are not comparable one by one, because each chain
has its own gauge (the legs and sole, against the waist). The comparison is
therefore on predictions over the mocap training run, with `B` and `T`
re-registered on all its rows:

- `R_table`: the table fit's torso and left-arm parameters, with legs, plane and contact dropped;
- `R_nominal`: nominal joints;
- `R_mocap`: the selected mocap model's k-fold error.

The verdict:

- agrees: `R_table <= R_nominal - 0.5 (R_nominal - R_mocap)`;
- partial: `R_table < R_nominal`;
- disagrees: otherwise.

Also reported is the RMS difference between the table and mocap predictions.
A disagreement does not separate a change of the robot over seven months
from a modelling error.

### 3.5 What transfers, and what cannot be judged on held-out data

| Parameters | Right-arm runs | Why |
|---|---|---|
| `B` (waist on base_link) | transfers: the secondary metric tests it | same body on the same link, same day |
| `T` (end-effector body on arm_7) | refitted | the body moved to the other arm |
| torso_2 `d_px`, `d_phix` | transfers: the gauge-free diagnostic | absorbed by neither frame nor the arm_1 placement |
| torso_2 `d_py`, `d_phiy` | transfers only combined with arm_left_1 | they commute with the torso_2 rotation: confounded with the left arm's first placement |
| torso_1, torso_2 `d_pz`, `d_phiz` | not fitted | absorbed by `B` |
| left-arm parameters | cannot be judged on held-out data | no held-out left run; only the k-fold plan (same session) judges them |
| right-arm parameters | not estimated | no training data for the right arm |

## 4. Simulated fixture (`mocap_truth.py`)

Seeds 0 to 4, noise 0.17, 1 and 3 mm per column, with the orientation as
`LEVER` times the rotation vector, so 0.17 mm is 1.7 mrad. All five models
are fitted (150 fits, about 40 minutes).

**Truth:**
- Frames: the synthetic `FRAMES` and `T_RIGHT`.
- Joint errors: seeded placement errors on all 16 joints. Each joint's own axis uses the encoder-offset size, 0.02 rad. Other placement errors use 1 mm and 2 mrad.
- Two kinds:
  - `full_params`: every joint has errors;
  - `right_exact`: the same draw with the right arm's errors at zero, the most carrying the torso can show.

**Held-out sets:**
- `left`: 40 postures drawn uniformly inside the training range, with the training frames carried over.
- `right`: the validation run's 39 postures, with its fit-free flip clusters and synthetic flips. The torso is carried and the frames and flips are re-registered against the noise-free truth.

Position RMSE in mm against the truth, mean over seeds:

| Truth | Noise | Model | Train | Left | Right | Right, flipped rows |
|---|---|---|---|---|---|---|
| full_params | 0.17 | registration only | 3.40 | 8.10 | 3.81 | 3.92 |
| | | joint_offset | 0.89 | 2.68 | 3.78 | 3.91 |
| | | full_params | 0.15 | 0.38 | 3.77 | 3.91 |
| | | full map / map_cv | 0.15 | 0.37 | 3.77 | 3.91 |
| | 1.0 | registration only | 3.49 | 8.06 | 3.81 | 3.92 |
| | | joint_offset | 1.24 | 2.75 | 3.76 | 3.90 |
| | | full_params | 0.86 | 2.23 | 3.78 | 3.87 |
| | | full map / map_cv | 0.89 | 1.23 | 3.75 | 3.90 |
| | 3.0 | registration only | 4.34 | 8.03 | 3.81 | 3.92 |
| | | joint_offset | 2.87 | 3.21 | 3.74 | 3.90 |
| | | full_params | 2.58 | 6.74 | 4.80 | 4.57 |
| | | full map / map_cv | 2.74 | 2.45 | 3.74 | 3.90 |
| right_exact | 0.17 | registration only | 3.40 | 8.10 | 0.79 | 0.66 |
| | | joint_offset | 0.89 | 2.68 | 0.48 | 0.42 |
| | | full_params | 0.15 | 0.38 | 0.34 | 0.29 |
| | | full map / map_cv | 0.15 | 0.37 | 0.27 | 0.23 |
| | 3.0 | registration only | 4.34 | 8.03 | 0.79 | 0.66 |
| | | full_params | 2.58 | 6.74 | 3.69 | 3.18 |
| | | full map / map_cv | 2.74 | 2.45 | 0.61 | 0.52 |

The z-scores of the map family are calibrated. Over 810 joint-level
parameters per family, the mean is -0.04, the RMS 0.96 and the largest
|z| 2.96.

What the fixture says:

1. **Within the session (the k-fold question), the method matters.**
   - At the training run's noise, the joint-level models recover the truth to about 0.4 mm. Registration only stays at 8.1 mm.
   - At 3 mm, structural `full_params` overfits (6.7 mm), and `map` does not (2.4 mm).
   - The k-fold plan is the gate that can tell these apart. On a simulated training run (`test_kfold_selects_a_joint_level_model_on_simulated_data`), it selects `full_params` over the baseline, with a k-fold over training ratio below 1.5.
2. **On the other arm, the torso is hidden behind the nominal right arm.**
   - With right-arm errors (`full_params` truth), every model re-registers to 3.7 to 3.8 mm, so C2 is expected to be neutral.
   - With an exact right arm, carrying the torso helps (0.79 to 0.27 mm).
   - So a "helps" on the real data would mean the right arm is close to nominal, or that the torso errors are large. A "neutral" says nothing against the torso fit.
3. **The flip repair recovers its transforms.** On the truth's own chain, the residual is about 1e-13 mm, and it recovers 90, 90 and 180 deg and the centroid (`test_flip_repair_recovers_the_synthetic_flips`). Flipped rows score like unflipped ones.
4. **`map_cv` equals `map` here.** The fixture's truth sizes are core's default priors, so the cross-validated scale stays at 1. This is no evidence that `map_cv` adds anything.

The fixture's frames are synthetic, not fitted to the real run: that would
have been a real fit before the pre-registration. They only set the
linearisation point, which is why it gives 42 kept parameters against 44.

## 5. Results

Written after the pre-registration commit (`e4810d5`). Numbers are in
`tests/golden/talos_mocap.json` (`python mocap_heldout.py --json ...`); the
whole study takes about 75 minutes, most of it `full map_cv` on k-fold
fold 4 (below).

### 5.1 Selection and C1

| Model | Parameters | Train (mm) | K-fold, pooled (mm) | Mean fold-train (mm) | Selection score (mm) |
|---|---|---|---|---|---|
| `registration only` | 12 | 7.66 | 8.09 | 7.61 | 8.04 |
| `joint_offset` | 19 | 196.61 | 157.88 | 118.42 | 122.58 |
| `full_params` | 42 | 85.81 | 70.74 | 48.52 | 54.80 |
| `full map` | 66 | 6.88 | 5.39 | 4.65 | **4.58** |
| `full map_cv` | 66 | 4.35 | 164.09 | 95.28 | 106.91 |

The selection score is the harness's: the mean of the per-fold RMSEs.

- **Selected: `full map`.** The config now carries `method: map`, set in the
  commit after this selection.
- **C1 passes.** 4.58 mm against the baseline's 8.04 mm. The k-fold over
  mean fold-training ratio is 5.39 / 4.65 = 1.16 (limit 1.5).

**The structural fits are not a fit of their model.** `joint_offset` and
`full_params` end above the frames-only fit even in training (197 and 86 mm
against 7.66 mm), which no overfitting can do. They start from core's
closed-form frame guess and converge to a wrong basin; on k-fold fold 4,
`full_params` stops at core's 1000-evaluation cap without converging. A
post-hoc diagnostic, not used for anything above
(`mocap_heldout.warm_started`, `test_structural_fits_recover_from_the_baseline_frames`),
starts them from the baseline's fitted frames instead: training 2.92 mm
(`joint_offset`) and 1.02 mm (`full_params`), validation 4.70 and 4.64 mm.
The pre-registration did not foresee a failed fit, so the frozen numbers
stand and the selection is unchanged. The same basin explains `full map_cv`'s
k-fold score: on fold 4 its preliminary free fit and its weakest-prior inner
fits run to their evaluation caps (hours), and its fold fits land far off.
On the whole run it fits well (4.35 mm), but it was not selected.

### 5.2 C2: the torso carried to the right arm

Position RMSE (mm), right arm nominal; orientation in parentheses (deg):

| Run | Model | Re-registered (primary) | Waist kept | Unflipped only | Gauge-free torso |
|---|---|---|---|---|---|
| validation (n=39, 14 flipped) | `registration only` | 4.70 (1.79) | 10.42 | 3.87 | 4.70 |
| | `full map` | 4.70 (1.80) | 9.66 | 3.87 | 4.70 |
| confirmation (n=22, row 0 dropped) | `registration only` | 37.58 (1.05) | 42.09 | 37.58 | 37.58 |
| | `full map` | 37.59 (1.05) | 41.72 | 37.59 | 37.58 |

- **Validation: neutral** (ratio 1.00), as the fixture expected.
  **Confirmation: neutral** (1.00).
- **The neutral is the trivial one.** `full map`'s torso posterior is its
  prior: every torso standard deviation equals the prior size (1 mm, 2 mrad,
  20 mrad on the joint axis) and the largest correction is 0.86 mrad
  (`d_phiy_torso_2_joint`). The training run does not inform the torso
  beyond core's default priors, so there is nothing to carry.
- **The flip repair works on the real run.** The validation run's three
  clusters are repaired as 90.8, 178.9 and 93.0 deg rotations; the flipped
  rows score 5.8 mm against 3.9 mm for the unflipped ones.
- **The waist mount transfers to within about 10 mm.** Keeping `B` from
  training costs 5 mm over re-registering it.
- **The confirmation run is 8 times worse for every model**, its one
  repeated posture included (21.8 mm). Posture range does not explain it; a
  moved body or marker set between 14:18 and 14:39 would, but the data
  cannot tell. It is a disclosed limit, not a verdict on any model.
- For the structural fits as frozen, carrying their torso costs 48 to
  207 mm, and the gauge-free reset brings `joint_offset` back to the
  baseline: the wrong basin sits in the torso and frame gauge.

### 5.3 C3: against table contact

`mocap_table_comparison.py`, the table study's structural left fit (56
parameters, 24 shared):

| Mocap training run, `B` and `T` re-registered | Position RMSE |
|---|---|
| `R_nominal`, nominal joints | 7.66 mm |
| `R_table`, table-contact torso and left arm | 10.93 mm (1.05 deg) |
| `R_mocap`, `full map`, k-fold pooled | 5.39 mm |

**C3: disagrees.** The table parameters make the mocap prediction worse than
nominal joints, and the two predictions differ by 15.3 mm. As pre-registered,
this does not separate a change of the robot over the seven months from a
modelling error in either study.

### 5.4 Reference run

`mocap_reference_run.py` fits `full map` on the training run (66 parameters,
residual dof 270, `ftol`, 15 evaluations), passes the execution-scope
verification and archives under `results/runs/TALOS/calibration/`
(git-ignored) with `report.html`, `heldout.json`, `corrections.json` and the
reproduction record. Its held-out numbers are the golden's.

### 5.5 What this study supports

- Within the session, `full map` predicts the left end-effector body to
  5.4 mm (k-fold), against 8.1 mm for the frames-only baseline.
- Nothing is learned about the torso, and the table-contact geometry does not
  carry over to this session. The left-arm corrections are judged only on
  the training session.
- Not supported: any URDF export, any claim on the right arm, and any
  structural (unregularised) model on these data.

## 6. Reproduction

From `examples/talos` in the `figaroh-dev` env:

```bash
python mocap_truth.py              # fixture, about 40 min (--seeds 0 --noise 0.17: 1 min)
python mocap_heldout.py            # k-fold selection, all models, right-arm runs (~75 min)
python mocap_table_comparison.py   # C3
python mocap_reference_run.py      # archived run under results/runs/
```

Tests:
- `tests/test_talos_mocap_2022_03_15.py`: protocol v2 and the attachments;
- `tests/test_talos_mocap_study.py`: the rules and the fixture;
- `tests/test_talos_mocap_heldout.py`: the real runs against the golden
  (`full map_cv`'s k-fold scores are taken from the golden, not recomputed);
- `tests/test_talos_mocap_reference_run.py`: the archived reference run.
