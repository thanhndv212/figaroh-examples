# TALOS table-contact calibration audit — 2026-10-04

Issue: [figaroh-examples #25](https://github.com/thanhndv212/figaroh-examples/issues/25).
Delivery package: [C1 / core #43](https://github.com/thanhndv212/figaroh-plus/issues/43).
Companion: [TIAGo mocap calibration audit](tiago-mocap-calibration-audit-2026-10-04.md).
This records the observation semantics and current behaviour of the TALOS
table-contact calibration (`examples/talos_table_contact/`), the second
calibration consumer. The method and its synthetic verification are described
in the [example README](../../examples/talos_table_contact/README.md); this
report checks its claims against the code and data and adds what C1 requires.

## Methodology

- Core figaroh-plus `02f705a` (`v0.5.0`); examples `main`. The changed
  revision is the commit containing this report.
- `figaroh-dev`, Python 3.12.11, Pinocchio 3.7.0, macOS arm64,
  single-threaded BLAS.
- `run_calibration_real_data.py --no-save-results --no-archive`, run three
  times in fresh processes: Pinocchio RNG default, `pin.seed(0)`, `pin.seed(7)`.

## Observation semantics

| Item | Value |
|---|---|
| Source | `data/real/{left,right}_{train,validation}.csv`: joint-encoder snapshots of postures commanded to touch a table flush, copied from the deprecated FIGAROH repository |
| Postures | left train 21 (2022-10-28), left validation 9, right train 29, right validation 9 (2022-11-07) |
| Row layout | 5 metadata fields (`gripper`, `talos/<side>_gripper`, `handle`, `table/contact_NN`, `joint_states`) + 32 joint angles (rad), parsed by position (three headers are misaligned) |
| Clock | None: each row is a static posture; no timestamps |
| Observation | **Not a measurement.** Each posture is *assumed* to be a flush contact; the model's predicted contact gap (height z, roll, pitch of the contact frame relative to the table) is driven to zero. No force/contact signal confirms the contact, and x, y, yaw of the contact point carry no information (`measure: [F, F, T, T, T, F]`) |
| Units | z in m (reported in mm), roll/pitch in rad (reported in deg); joints in rad |
| Chain | `<side>_sole_link` (base frame) → leg → torso → arm → `gripper_<side>_base_link` (tool frame); `free_flyer: false` — the sole is the fixed root |
| Gauge | One table-pose correction per session (`plane_z`, `plane_phix`, `plane_thetay`) on a nominal `NOMINAL_TABLE_POSE` = (0.30, 0.28, 1.00) m, and one contact-frame correction per side (`contact_z/phix/thetay`) on `NOMINAL_CONTACT_OFFSET` = z −0.12 m. These nominals are the synthetic generator's rough guesses, not measurements of the real rig |
| Joint geometry | 57 identifiable `Delta X` parameters per chain (of 90 raw); two-chain fit shares torso axes |
| Fixed transforms | URDF; gripper geometry beyond `gripper_<side>_base_link` absorbed into the contact correction |
| Payload | None recorded; quasi-static double support assumed |
| Split policy | Per side, train and validation files are disjoint postures. Left: different days (train 10-28, validation 11-07), scored with the plane fitted on the training day — valid only if the table did not move. Right: same day. Each file is one session (`session_id = 0`; no session column in the data) |
| Regularisation | None (`coeff_regularize: null`) |

**Contact-only observations are not 6D truth.** Three of six pose components
per posture are constrained, by assumption, and the table pose itself is
estimated. Errors along the table plane and about its normal are unobservable.

## Current behaviour (baseline)

Single chain, real data, three RNG states (default / `pin.seed(0)` / `pin.seed(7)`):

| Chain | Training z | Training roll / pitch | Held-out z | Held-out roll / pitch |
|---|---|---|---|---|
| Left | 292.0 → 0.48–0.49 mm | 1.44° → 0.13°; 1.43° → 0.19° | 288.2 → **5.63–5.76 mm** | 1.21° → 0.42°; 1.46° → 0.49° |
| Right | 281.7 → 1.33–1.34 mm | 2.51° → 0.32°; 0.87° → 0.38° | 291.9 → **8.01–8.02 mm** | 1.88° → 0.98°; 1.07° → 0.59° |

The "before" values (~290 mm) mostly reflect the rough nominal table guess,
which the plane parameters absorb by design; they are not a measure of the
robot's geometric error. The meaningful comparison is held-out after fit.

Two-chain fit with shared torso (`MultiChainCalibration`):

| RNG state | Shared torso axes | Union | Left held-out z | Right held-out z |
|---|---|---|---|---|
| default | 4 | 110 | 4.69 mm | 8.29 mm |
| `pin.seed(0)` | 5 | 110 | **9.79 mm** | 8.22 mm |
| `pin.seed(7)` | 4 | 110 | 4.99 mm | 8.00 mm |
| example README | 5 | 109 | 12.32 mm | 7.46 mm |

**The results depend on Pinocchio's RNG state.** The identifiable parameter
set is chosen from random configurations (`MIN_IDENTIFIABILITY_SAMPLES = 65`
probes drawn with `pin.randomConfiguration`), the same mechanism as
[figaroh-plus#99](https://github.com/thanhndv212/figaroh-plus/issues/99).
Single-chain results move by ~2%; the two-chain fit changes which torso axes
are shared and its left held-out height error varies by about 2× (4.7–12.3 mm).
The README's tables are one draw each and are not reproducible as written.

### What the generic quality report gets wrong for contact data

`BaseCalibration`'s printed quality report treats the three residual rows as
**X, Y, Z positions in mm**. For this example they are contact height (mm),
roll and pitch: the "Y 2.21 mm" row is the 0.127° roll residual expressed in
milliradians, and the combined "position RMSE 3.98 mm" has no physical meaning.
It also states "no separate validation data provided" although the example
evaluates a held-out set itself, and reports infinite parameter uncertainty
because 63 observations fit 63 parameters (zero residual degrees of freedom;
`RuntimeWarning: divide by zero` in `base_calibration.py`); tracked as
[figaroh-plus#100](https://github.com/thanhndv212/figaroh-plus/issues/100). The example's own
`Training-set gap` / `Held-out validation gap` blocks are the correct numbers.

## Missing assets and data

- Any confirmation that each posture was in contact (force/torque, gripper
  contact sensing); a rejected-contact log.
- A measurement of the table pose or the contact geometry (the nominals are
  generic guesses).
- Session metadata: which postures belong to which table setup; whether the
  table moved between 2022-10-28 and 2022-11-07.
- The original ROS/HPP recording pipeline (`agimus-demos/talos/calibration/contact`,
  outside FIGAROH) and robot unit identification.
- The `table/contact_NN` handle IDs are kept for traceability but carry no
  table coordinates.

## Changes in this issue

No numerical change. Added: this report and `tests/test_talos_contact_semantics.py`
(measured components, frames, fixed root, no regularisation, raw row layout,
no clock, one session per file). The existing real-data tests already bound
the numerical behaviour loosely enough for the observed RNG range. The example
README now states that its tables are single draws (figaroh-plus#99), warns
about the quality-report labels (figaroh-plus#100) and links this report.
