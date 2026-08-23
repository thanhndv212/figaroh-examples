# TALOS table-contact example (whole-body plane calibration)

This folder implements **whole-body kinematic calibration of TALOS's
leg-torso-arm chain from repeated flush contact with a single flat
table** -- no external metrology at all: no motion capture, no laser
tracker, no camera. It is a from-scratch FIGAROH port of the method
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
  small damped-least-squares IK helper reused by the data generator.
- `generate_synthetic_data.py` -- there is no real hardware CSV bundled
  with this example (see "What's not here"). This script instead builds
  a *known* ground truth (injected joint-placement errors, table pose,
  contact-frame offset) and solves IK for a batch of touch postures
  against it, so the true model's predicted gap is exactly zero at every
  recorded posture -- exactly as a real successful contact would be.
  That known ground truth is what makes the calibration verifiable.
- `run_calibration.py` -- runs the calibration end-to-end and prints
  before/after gap metrics on both the training set and a held-out
  validation set, plus the recovered plane/contact-frame estimates.
- `config/talos_table_left_config.yaml` -- legacy-format FIGAROH config
  for the `left_sole_link -> gripper_left_base_link` chain (see "Why
  legacy config, not unified" below).
- `data/` -- a committed demo dataset (noiseless synthetic touches,
  `generate_synthetic_data.py`'s defaults) so `run_calibration.py` has
  something to run against out of the box; `--regenerate` resynthesizes
  it (optionally with `--noise-deg` for realistic joint-encoder noise).
- `../../tests/test_talos_table_contact.py` -- the round-trip validation
  suite (see "Verification & validation" below).

## Run

```bash
cd examples/talos_table_contact

# Calibrate against the committed noiseless demo data
python run_calibration.py

# Resynthesize data with realistic joint-encoder noise
python run_calibration.py --regenerate --noise-deg 0.05 --seed 42

# Round-trip test suite (from the figaroh-examples root)
cd ../.. && pytest tests/test_talos_table_contact.py -v
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
regulation, force/torque-based contact detection and release), the HPP
whole-body-equilibrium-and-contact-constraint path planning, and the
traveling-salesman posture ordering that a *real* run of this method
needs are **not FIGAROH's job** and are not implemented here. They live
in a separate ROS/HPP/Stack-of-Tasks stack:
[`agimus-demos/talos/calibration/contact`](https://github.com/agimus/agimus-demos/tree/master/talos/calibration/contact).
This example assumes that stack (or an equivalent) has already produced
a CSV of joint angles per successful contact -- the same assumption the
other CSV-driven examples in this repo make about their own data.

## Notes

- Chain: `left_sole_link -> gripper_left_base_link` (left leg, torso,
  left arm -- 15 joints), matching the manuscript's chain definition.
  Extending to the matching right-leg/right-arm chain, and solving both
  jointly so the shared torso parameters get constrained by both feet at
  once (the full "whole-body" claim), is a natural next step; this
  example validates the single-chain mechanism first, deliberately, per
  the incremental-implementation practice the rest of this workspace
  follows.
- The nominal table pose and contact-frame offset
  (`generate_synthetic_data.NOMINAL_TABLE_POSE` /
  `NOMINAL_CONTACT_OFFSET`) were picked by probing TALOS's own reachable
  workspace and the real `gripper_left_base_link -> fingertip` offset
  from the URDF, not invented arbitrarily -- see the docstrings for how.
