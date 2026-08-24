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
- `generate_table_configs.py` -- a *different* generator: runs against
  the plain nominal model (no injected error) to produce whole-body,
  double-support-*checked* candidate postures for an actual data-collection
  session (see "Posture generation" below).
- `run_calibration.py` -- runs the calibration end-to-end and prints
  before/after gap metrics on both the training set and a held-out
  validation set, plus the recovered plane/contact-frame estimates.
- `config/talos_table_left_config.yaml` / `talos_table_right_config.yaml`
  -- legacy-format FIGAROH config for the left and right chains (see "Why
  legacy config, not unified" below).
- `data/` -- a committed demo dataset (noiseless synthetic touches,
  `generate_synthetic_data.py`'s defaults) so `run_calibration.py` has
  something to run against out of the box; `--regenerate` resynthesizes
  it (optionally with `--noise-deg` for realistic joint-encoder noise).
- `../../tests/test_talos_table_contact.py`,
  `test_talos_table_contact_multichain.py`,
  `test_generate_table_configs.py` -- the round-trip validation suites
  (see "Verification & validation" below).

## Run

```bash
cd examples/talos_table_contact

# Calibrate against the committed noiseless demo data
python run_calibration.py

# Resynthesize data with realistic joint-encoder noise
python run_calibration.py --regenerate --noise-deg 0.05 --seed 42

# Round-trip test suites (from the figaroh-examples root)
cd ../..
pytest tests/test_talos_table_contact.py -v               # single-chain
pytest tests/test_talos_table_contact_multichain.py -v    # two-chain, shared torso
pytest tests/test_generate_table_configs.py -v             # posture generator

# Generate double-support-checked candidate postures (nominal model, no
# ground truth -- see "Posture generation" below)
cd examples/talos_table_contact
python generate_table_configs.py --n-candidates 500
```

## Verification & validation

There is no physical rig available in this workspace, so "verified and
validated" means a **round-trip test with a known ground truth**: inject
joint-placement errors, a table pose, and a contact-frame offset;
synthesize IK-consistent touches for them (so the true gap is exactly
zero, as with a real successful contact); calibrate a *nominal* model
against those touches; check that the calibrated model's predicted gap
is small both on the training postures and on a **held-out validation
set** the solver never saw. This mirrors exactly how the manuscript
itself validates the method -- residual before/after in mm and degrees,
on both a training and a held-out set -- rather than a synthetic-only
"the numbers look small" claim.

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
