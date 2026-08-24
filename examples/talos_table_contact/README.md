# TALOS table-contact example (whole-body plane calibration)

This folder implements **whole-body kinematic calibration of TALOS's
leg-torso-arm chains (both sides, sharing the torso) from repeated flush
contact with a single flat table** -- no external metrology at all: no
motion capture, no laser tracker, no camera. It is a from-scratch FIGAROH
port of the method
described in the project's own manuscript, *"Humanoid Robot Whole-body
Geometric Calibration with Embedded Sensors and a Single Plane"*
(internal, anonymized for review), which reduced cross-validated pose
error by a factor of 2.3 on a real TALOS unit using exactly this idea.

## The idea, in one paragraph

TALOS stands in quasi-static whole-body equilibrium on both feet while a
rigid, three-fingered gripper makes flush contact with a table at many
different spots. A flush touch pins down only 3 of the contact point's 6
pose degrees of freedom -- height above the table and the two in-plane
tilts (roll, pitch); where on the table it lands and its yaw about the
table's normal carry no information and are not modeled. Neither the
table's own pose nor the gripper's exact contact-point offset is known
in advance -- both are small corrections to a rough nominal setup,
identified *jointly* with the chain's joint-placement errors by driving
the predicted (height, roll, pitch) gap to its least-squares minimum
across every recorded posture. That's the whole method: there is no
externally measured "ground truth" pose anywhere in it.

See the accompanying engineering report for the full derivation, the
gap analysis against core FIGAROH, and the physical rig this is modeled
on (`agimus-demos/talos/calibration/contact` -- a separate ROS/HPP/
Stack-of-Tasks repo, not part of FIGAROH; see "What's not here" below).

## What's included

- `utils/talos_table_tools.py` -- `TalosTableContactCalibration`, a
  `figaroh.calibration.base_calibration.BaseCalibration` subclass. Reuses
  core FIGAROH, unmodified, for config loading and the QR-based
  identification of the chain's joint-placement errors (`calc_updated_fkm`,
  `calculate_base_kinematics_regressor`); replaces the measurement model
  entirely (`load_data_set`, `cost_function`) since there is no
  externally-measured target to load. Also exposes `solve_touch_ik`, a
  small damped-least-squares IK helper reused by the data generator, and
  `MultiChainCalibration`, which couples a left- and a right-chain
  instance so their shared torso joint-placement corrections are fit as
  one value (see "Two-chain coupling" below).
- `generate_synthetic_data.py` -- there is no real hardware CSV bundled
  with this example (see "What's not here"). This script instead builds
  a *known* ground truth (injected joint-placement errors, table pose,
  contact-frame offset) and solves IK for a batch of touch postures
  against it, so the true model's predicted gap is exactly zero at every
  recorded posture -- exactly as a real successful contact would be.
  That known ground truth is what makes the calibration verifiable.
  `build_two_chain_dataset` does the same for both chains at once,
  sharing one injected torso error and one physical table between them.
- `run_calibration.py` -- runs the calibration end-to-end against
  synthetic data and prints before/after gap metrics on both the training
  set and a held-out validation set, plus the recovered plane/contact-frame
  estimates.
- `run_calibration_real_data.py` -- runs the same calibration against
  **real hardware data** -- see "Real-hardware validation" below.
- `config/talos_table_left_config.yaml` / `talos_table_right_config.yaml`
  -- legacy-format FIGAROH config for the left and right chains (see "Why
  legacy config, not unified" below).
- `data/` -- a committed synthetic demo dataset (noiseless,
  `generate_synthetic_data.py`'s defaults) so `run_calibration.py` has
  something to run against out of the box; `--regenerate` resynthesizes
  it (optionally with `--noise-deg` for realistic joint-encoder noise).
- `data/real/` -- the actual joint-encoder recordings the manuscript's
  own results were computed from (see "Real-hardware validation" below).
- `generate_table_configs.py` -- an exploratory, secondary generator
  (runs against the nominal model, no injected error, to produce
  whole-body double-support-*checked* candidate postures). Not part of
  the core pipeline -- see "Posture generation" for why.
- `../../tests/test_talos_table_contact.py`,
  `test_talos_table_contact_multichain.py`,
  `test_talos_table_contact_real_data.py`,
  `test_generate_table_configs.py` -- the validation suites (see
  "Verification & validation" / "Real-hardware validation" below).

## Run

```bash
cd examples/talos_table_contact

# Calibrate against the committed noiseless synthetic demo data
python run_calibration.py

# Resynthesize synthetic data with realistic joint-encoder noise
python run_calibration.py --regenerate --noise-deg 0.05 --seed 42

# Calibrate against real hardware data (left, right, and two-chain)
python run_calibration_real_data.py

# Round-trip / validation test suites (from the figaroh-examples root)
cd ../..
pytest tests/test_talos_table_contact.py -v               # single-chain, synthetic
pytest tests/test_talos_table_contact_multichain.py -v    # two-chain, synthetic
pytest tests/test_talos_table_contact_real_data.py -v     # real hardware data
```

## Verification & validation

There is no live physical rig to run new experiments on in this
workspace, but there is real recorded data from one (see "Real-hardware
validation" below) -- this section covers the synthetic side: a
**round-trip test with a known ground truth**, which is what makes exact
mechanism correctness checkable (real data alone can't prove the
frame-composition/sign conventions are right, only that the fit
generalizes). Inject
joint-placement errors, a table pose, and a contact-frame offset;
synthesize IK-consistent touches for them (so the true gap is exactly
zero, as with a real successful contact); calibrate a *nominal* model
against those touches; check that the calibrated model's predicted gap
is small both on the training postures and on a **held-out validation
set** the solver never saw. This mirrors exactly how the manuscript
itself validates the method -- residual before/after in mm and degrees,
on both a training and a held-out set -- rather than a "the numbers look
small" claim with nothing to check them against.

**Structural check.** The 15-joint `left_sole_link -> ... ->
gripper_left_base_link` chain (6 leg + 2 torso + 7 arm joints) reduces,
via the same QR-based rank analysis core FIGAROH already ships
(`eliminate_non_dynaffect` / `get_baseParams`), to **57 identifiable
base parameters** -- the exact count the manuscript reports for this
chain. `tests/test_talos_table_contact.py::TestChainIdentifiability`
asserts this directly: if this number ever changes, something about the
chain, the measurement mask, or the QR tolerance changed with it.

**Noiseless round trip** (`python run_calibration.py`, default seed):

| | before | after | reduction |
|---|---:|---:|---:|
| train z | 8.28 mm | 0.006 mm | 1440x |
| train roll | 1.19&deg; | 0.0005&deg; | 2475x |
| train pitch | 0.82&deg; | 0.0006&deg; | 1376x |
| held-out z | 9.54 mm | 0.007 mm | 1293x |
| held-out roll | 1.28&deg; | 0.0007&deg; | 1769x |
| held-out pitch | 0.68&deg; | 0.0011&deg; | 613x |

With no measurement noise, the fit drives the gap essentially to the
floating-point noise floor on *both* the training and the held-out set
-- strong evidence the mechanism itself (the joint composition of
`calc_updated_fkm`'s chain pose with the plane- and contact-frame
corrections, and the residual sign/frame conventions) is correct, not
just "small enough to pass".

**With realistic joint-encoder noise** (`--noise-deg 0.05`, a
conservative ~0.05&deg; per-joint reading noise, seed 42):

| | before | after | reduction |
|---|---:|---:|---:|
| train z | 6.45 mm | 0.54 mm | 12x |
| train roll | 0.51&deg; | 0.10&deg; | 5.3x |
| train pitch | 0.52&deg; | 0.10&deg; | 5.0x |
| held-out z | 7.72 mm | 1.45 mm | 5.3x |
| held-out roll | 0.38&deg; | 0.10&deg; | 3.7x |
| held-out pitch | 0.32&deg; | 0.12&deg; | 2.6x |

These are directly comparable in shape (not magnitude -- there's no
sensor/admittance-controller noise in this synthetic rig, so a larger
reduction than the manuscript's real-hardware 2.3-3x is expected) to
the manuscript's own reported numbers: pre-calibration errors of a
similar order (mm-scale height, sub-degree-to-degree-scale tilt),
reduced by roughly an order of magnitude on training data and somewhat
less on held-out data -- the same qualitative pattern a real, working
calibration should show.

**What the test does *not* assert: exact recovery of the injected plane
and contact-frame values.** A table-height error and certain leg/torso
length errors both shift the observed contact height by the same
amount; a single-plane, single-chain measurement cannot tell them apart
(this is physics, not a bug -- see the manuscript's own identifiability
discussion). The recovered `plane_z`/`contact_z` estimates land in the
same few-millimeter regime as the injected ground truth but are not
expected to match it digit-for-digit; `TestPlaneAndContactRecoveryOrderOfMagnitude`
checks only that they stay physically plausible, not that they're exact.
What *does* have to match, and is asserted directly, is the held-out
gap: if the calibration is wrong, it stops predicting new touches
correctly, whatever the individual parameter values happen to be.

## Real-hardware validation

Everything above is synthetic. This section is not: `data/real/` holds
the actual joint-encoder recordings from the physical rig, copied from
the deprecated FIGAROH repository's `data/talos/contacts/`:

| file | source | postures |
|---|---|---:|
| `left_train.csv` | `compiled_measurements_left_1028.csv` (2022-10-28) | 21 |
| `right_train.csv` | `compiled_measurements_right_1107.csv` (2022-11-07) | 29 |
| `left_validation.csv` | `validation_left_1107.csv` (2022-11-07) | 9 |
| `right_validation.csv` | `validation_right_1107.csv` (2022-11-07) | 9 |

These are the exact files the deprecated prototype script
(`scripts/talos_contact_calibration.py`) used, and the manuscript's own
2.3x cross-validated pose-error reduction was computed from. There is no
ground truth to recover here -- this is real, already-uncalibrated
hardware -- so "verification" means what it means for the manuscript
itself: does the gap actually drop, on training postures *and* on a
genuinely disjoint held-out set recorded on a different day, when this
implementation's own faithful cost function (not the prototype's cruder
session-mean-centering trick) is fit to it.

**A real data-quality bug, found and fixed while wiring this up:** 3 of
the 4 CSVs have a header missing its leading placeholder columns, which
silently shifts every column by 4 if read with `pandas.read_csv`'s own
header -- putting metadata strings like `"talos/left_gripper"` where a
joint angle should be. `run_calibration_real_data.py` parses the raw row
layout directly instead of trusting the header (see its docstring), and
`test_talos_table_contact_real_data.py::test_joint_values_are_within_talos_limits`
guards against this regressing silently again.

**Results** (`python run_calibration_real_data.py`; nominal table pose
and contact offset are the same rough guesses `generate_synthetic_data.py`
uses -- there was no real prior knowledge of *this* rig's actual table
position, and the large pre-calibration numbers below mostly reflect that
gap, which `PLANE_TPL`/`CONTACT_TPL` absorb by design):

| single chain | Training z | Training roll | Training pitch | Held-out z | Held-out roll | Held-out pitch |
|---|---:|---:|---:|---:|---:|---:|
| Left | 292.0 &rarr; 0.78 mm (372x) | 1.44&deg; &rarr; 0.17&deg; (8.6x) | 1.43&deg; &rarr; 0.16&deg; (8.9x) | 288.2 &rarr; 9.26 mm (31x) | 1.21&deg; &rarr; 0.75&deg; (1.6x) | 1.46&deg; &rarr; 0.52&deg; (2.8x) |
| Right | 281.8 &rarr; 1.33 mm (212x) | 2.51&deg; &rarr; 0.32&deg; (7.8x) | 0.87&deg; &rarr; 0.38&deg; (2.3x) | 292.0 &rarr; 8.03 mm (36x) | 1.88&deg; &rarr; 0.98&deg; (1.9x) | 1.07&deg; &rarr; 0.59&deg; (1.8x) |

Fitting both chains jointly via `MultiChainCalibration` shares 3 torso
axes (`d_phix_torso_2_joint`, `d_phiy_torso_2_joint`, `d_px_torso_2_joint`
-- union 104 identifiable params vs. the naive sum of 107) and *improves*
the left chain's held-out generalization noticeably (it has less
training data of its own, so it benefits more from borrowing the shared
torso correction the right chain's 29 postures help pin down):

| two-chain, shared torso | Held-out z | Held-out roll | Held-out pitch |
|---|---:|---:|---:|
| Left | 288.2 &rarr; 4.86 mm (59x) | 1.21&deg; &rarr; 0.44&deg; (2.7x) | 1.46&deg; &rarr; 0.50&deg; (2.9x) |
| Right | 292.0 &rarr; 7.91 mm (37x) | 1.88&deg; &rarr; 0.98&deg; (1.9x) | 1.07&deg; &rarr; 0.59&deg; (1.8x) |

These held-out residuals (5-9mm, 0.4-1&deg;) land in the same order of
magnitude as the manuscript's own reported held-out numbers (mm-scale,
sub-degree-to-1&deg; tilt) -- not identical (different day, and this
rig's real table pose/contact offset were never independently measured
here, unlike the manuscript's own experiment), but the same qualitative
result: a real, substantial, held-out-generalizing improvement from
fitting this exact cost function to real recordings, not just to
synthetic data built to match it.

**One expected identifiability artifact, worth naming rather than
hiding:** the recovered `plane_z` and `contact_z` corrections are close
to equal and opposite (e.g. left: plane -110.5mm, contact +110.7mm).
This is the same single-chain aliasing already documented above (a
table-height error and a contact-offset error shift the same observed
gap) -- here it's large because the *nominal* guesses for both were
never calibrated to this specific rig, so the fit correctly absorbs
nearly all of that gap into the sum, without being able to separate
which one it came from. It doesn't affect the held-out prediction
quality, which is what's actually asserted.

`tests/test_talos_table_contact_real_data.py` (11 tests) asserts:
training-gap reduction, held-out generalization, held-out residual
magnitude in the manuscript's ballpark, the shared-torso structural
properties, and the column-alignment regression guard above.

## Two-chain coupling

`MultiChainCalibration` fits the left chain (`left_sole_link ->
gripper_left_base_link`) and the right chain (`right_sole_link ->
gripper_right_base_link`) *jointly*, so a joint-placement correction both
chains can identify -- physically, this means a torso joint, the only
joint set on the kinematic path of both -- is fit as one shared value
instead of two independent (and generally inconsistent) per-chain
estimates of what is really the same physical offset.

This is sound, not just convenient, because of how
`calc_updated_fkm` (core FIGAROH, unmodified) consumes parameter names:
each chain's own QR-based base-parameter reduction reports a
well-conditioned *subset* of the raw, single-`(joint, axis)`
`d_px_<joint>`/... names -- never a combined linear-combination under a
new name -- and `calc_updated_fkm` looks each one up by a literal
substring match against the joint's own name. A raw name's meaning ("the
correction to this joint's own placement") therefore doesn't depend on
which chain is evaluating it, so if the *same* name is independently
found identifiable by both chains, treating it as one shared scalar fed
into both chains' forward kinematics is exactly the coupling a real
two-armed calibration needs -- see `MultiChainCalibration`'s docstring
for the full argument.

`tests/test_talos_table_contact_multichain.py` validates this the same
way as the single-chain suite (inject a *known* ground truth -- one
shared torso error, one physical table, independent per-side leg/arm
errors and gripper offsets -- synthesize touches for both chains,
calibrate jointly, check training + held-out gap reduction on both), plus
two structural checks specific to the coupling itself:

- the union of both chains' identifiable Delta-X parameters is strictly
  smaller than the naive sum `left.n_deltaX + right.n_deltaX` -- proof
  that sharing actually happened, not just that the code ran;
- every shared parameter name references a torso joint -- the only
  physically-shared joint set, so anything else showing up there would
  indicate a bug, not a feature.

## Posture generation

**Status: exploratory, not part of the core pipeline.** In the real
execution stack, choosing which postures to attempt is
[HPP](https://github.com/agimus/agimus-demos/tree/master/talos/calibration/contact)'s
job -- a Gauss-Newton solve over the *actual* whole-body equilibrium and
contact constraints together (the manuscript's Algorithm 1), not
something worth re-deriving from a bare kinematic IK in FIGAROH. What
follows is a best-effort, self-contained approximation built anyway (real
double-support balance and joint-limit checks, not just random sampling),
kept because it's genuinely tested and honestly limited rather than
discarded, but it should not be read as a substitute for HPP's role, and
no further effort is planned here. Calibrating against real data (above)
did not depend on it in any way -- that data came from postures HPP (or
its predecessor) already picked.

`generate_table_configs.py` produces candidate whole-body postures for an
*actual* data-collection session -- a different job from
`generate_synthetic_data.py`'s (which injects a fake ground truth purely
to make the calibration round-trip testable). Given a random touch target
on the table, it:

1. solves the flush-contact IK for the left chain (as in
   `generate_synthetic_data.py`);
2. re-solves the right leg back to a flat, neutral double-support stance
   *given that posture's torso lean* -- closing the loop on only
   `(z, roll, pitch)`, the same "3 informative DOF" convention the
   table-contact measurement model itself uses (a standing robot isn't
   required to plant its foot at one exact `(x, y, yaw)`, only flat; the
   full 6-DOF pose over-constrains the redundant, limited-range leg and
   fails to converge for postures that are perfectly fine physically);
3. rejects the candidate if any active joint (21 total: the 15-joint left
   chain plus the 6-joint right leg) sits within a limit margin, or if
   the resulting whole-body center of mass falls outside the convex hull
   of both feet's footprints (a real, if approximate, double-support
   quasi-static equilibrium check, via `pin.centerOfMass` and a
   `scipy.spatial.Delaunay` point-in-hull test).

`tests/test_generate_table_configs.py` independently re-verifies every
physical claim an accepted posture makes -- flush contact, flat right
foot, CoM in the support polygon, no joint past its limit -- by
recomputing them from scratch off the recorded joint angles, rather than
trusting the generator's own bookkeeping.

**Expect a low acceptance rate** (low single-digit percent at the
defaults) -- this IK has no whole-body posture-comfort prior (unlike a
real task-priority controller), so a fair number of otherwise-valid
touches end up needing some joint at its limit once the right leg also
has to re-plant. This is an honest property of a bare kinematic
generator, not a bug; budget `--n-candidates` in the hundreds to a few
thousand, or narrow the sampling range for a higher yield.

Its CSV output uses the same joint-angle-column schema as
`generate_synthetic_data.py`'s, so it can be fed directly as
`TalosTableContactCalibration`'s `data_file` -- or, via
`figaroh.calibration.calibration_tools.load_data`, into
`BaseOptimalCalibration.load_candidate_configurations()`'s CSV path,
letting core FIGAROH's SOCP + **IROC** optimal-configuration selection
(`calculate_optimal_configurations(selection_method="iroc")`, added to
`figaroh.optimal.base_optimal_calibration` alongside this example -- rank
candidates by SOCP weight, then grow the selected subset until the
normalized D-optimality criterion plateaus, an automatic minimal-count
selection instead of a fixed weight threshold) pick the minimal
informative subset of these physically-valid candidates to actually
execute on the robot.

**Not checked: mesh-level self-collision.** TALOS's URDF in this repo
ships no SRDF collision-pairs/exclusion list, and naively calling
`collision_model.addAllCollisionPairs()` without one flags every
anatomically-adjacent, permanently-touching link pair as a "collision" --
false positives, not a real check. Left as future work.

## Why legacy config, not unified

`BaseCalibration.load_param()` auto-detects the unified (`extends:` +
`tasks.*`) config format used by the other examples in this repo. That
format's `measurements.markers[]` schema has no concept of an
independently-estimated plane frame (it assumes every marker's target
is either externally measured or a known/unknown *tool* frame via
`EE_TPL`) -- there's nowhere to declare `PLANE_TPL` from YAML yet. Rather
than force-fit this method's parametrization into a schema that doesn't
support it (or extend core FIGAROH's config parser for a single,
unproven example), this class builds `PLANE_TPL`/`CONTACT_TPL` in Python
(`TalosTableContactCalibration.initialize()`) on top of a plain legacy
YAML for everything the standard schema *does* already cover (chain
frames, the DOF mask, sample count). If this pattern proves useful for a
second robot, promoting the plane-frame concept into `calc_updated_fkm`
itself (a third named frame type alongside `BASE_TPL`/`EE_TPL`) is the
natural next step -- see the engineering report's Option B.

## What's not here

The admittance controller (force-normal-to-plane, zero-wrist-moment
regulation, force/torque-based contact detection and release), and the
HPP whole-body-equilibrium-and-contact-constraint path planning /
execution that a *real* run of this method needs on the physical robot
are **not FIGAROH's job** and are not implemented here. They live in a
separate ROS/HPP/Stack-of-Tasks stack:
[`agimus-demos/talos/calibration/contact`](https://github.com/agimus/agimus-demos/tree/master/talos/calibration/contact).
This example assumes that stack (or an equivalent) has already produced
a CSV of joint angles per successful contact -- the same assumption the
other CSV-driven examples in this repo make about their own data.
`generate_table_configs.py` (above) covers the *offline candidate
selection* half of posture generation (which configurations are worth
attempting, kinematically and w.r.t. balance) but not mesh-level
self-collision, nor the real-time execution/planning itself.

## Notes

- Chains: `left_sole_link -> gripper_left_base_link` and
  `right_sole_link -> gripper_right_base_link` (each leg + torso + arm --
  15 joints), matching the manuscript's chain definitions, solved jointly
  via `MultiChainCalibration` (see "Two-chain coupling" above) so the
  shared torso parameters get constrained by both feet at once -- the
  full "whole-body" claim.
- The nominal table pose and contact-frame offset
  (`generate_synthetic_data.NOMINAL_TABLE_POSE` /
  `NOMINAL_CONTACT_OFFSET`) were picked by probing TALOS's own reachable
  workspace and the real `gripper_left_base_link -> fingertip` offset
  from the URDF, not invented arbitrarily -- see the docstrings for how.
