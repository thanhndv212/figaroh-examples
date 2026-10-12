# TIAGo Configuration Files

## Active Configurations (in this directory)

| File | Purpose | Format |
|------|---------|--------|
| `tiago_unified_config.yaml` | **Main unified config** — used by all scripts by default | Unified (extends: `../../templates/humanoid_robot.yaml`) |
| `tiago_unified_config_hey5.yaml` | TIAGo with Hey5 hand (eye-hand calibration; `optimal_config.py` default) | Unified (extends: `../../templates/base_robot_config.yaml`) |
| `tiago_suspension_config.yaml`, `tiago_backlash_surface_config.yaml` | Experimental suspension / backlash experiment descriptions | Not a FIGAROH robot config (no `calibration:`/`identification:`) |

## Archived Configurations (in `archive/`)

| File | Reason |
|------|--------|
| `tiago_unified_config_with_templates.yaml` | Redundant — demo/exploratory copy of unified config |
| `tiago_unified_config_simple.yaml` | Redundant — simplified test copy of unified config; its non-zero `regularization_coefficient` is rejected by figaroh 1.0 |
| `templates/local_template.yaml` | Redundant — local template example, unused |

The four legacy flat-format archive configs (`tiago_config_improved`, `_mocap`,
`_mocap_vicon`, `_palgripper`) were deleted before 1.0, which no longer loads that
format; they remain in git history.

## Which Config to Use

- **All scripts** default to `config/tiago_unified_config.yaml` — this is the recommended
  configuration for TIAGo work.
- The legacy flat `tiago_config.yaml` and `tiago_config_hey5.yaml` were migrated: the
  Schunk variant is covered by `tiago_unified_config.yaml`, the Hey5 variant became
  `tiago_unified_config_hey5.yaml` (figaroh-examples#125).
- Do not add new configs in legacy flat format; always use the unified format with
  `extends:` template inheritance.
