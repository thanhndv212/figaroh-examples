# FIGAROH Configuration Templates

This directory contains YAML template files that serve as reusable starting points for robot-specific unified configuration files. Templates define the canonical structure for robot configurations in the FIGAROH framework, ensuring consistency across all robot examples.

## Before choosing a template

Use the [new-example guide](../../docs/new-example-guide.md) and
[experiment brief](../../docs/experiment-brief-template.md) to define available
data, parameter scope and validation. Template defaults are placeholders to
review: rates, measured DOFs, frames, mechanical terms and enabled tasks must
match the robot and acquisition rather than its broad robot category.

## Overview

Templates provide:

- **Standard structure** — Every robot config follows the same YAML layout (`meta`, `robot`, `tasks`, `environment`, ...), making it easy to navigate and compare configurations across robots.
- **Inheritance** — Robot configs use `extends:` to pull in shared settings from templates, keeping robot-specific configs small and focused on what differs.
- **Defaults** — Sensible default values for calibration, identification, signal processing, and optimization parameters that can be selectively overridden.
- **Discoverability** — New contributors can see at a glance what settings a robot config can accept and what each section means.

## Available Templates

### `base_robot_config.yaml`

The foundational template that all robot configurations should extend. It defines the complete configuration schema with:

- **Metadata** (`meta`) — Schema version, config type, description.
- **Robot properties** (`robot.properties`) — Joint configuration (active joints, limits), mechanical parameters (friction coefficients, reduction ratios, actuator inertias), and coupling information.
- **Task definitions** (`tasks`) — Stub entries for each supported task type (calibration, identification, optimal configuration, optimal trajectory) with default parameters.
- **Environment** (`environment`) — Directory structure, logging, and visualization settings.
- **Custom extensions** (`custom`) — Placeholder for robot-specific settings.

Each task in the template includes documentation of all available parameters in YAML comments, serving as inline reference documentation.

### `manipulator_robot.yaml`

Extends `base_robot_config.yaml` for fixed-base serial manipulators (e.g., UR10, Staubli TX40, KUKA). Key characteristics:

- `robot.type: "manipulator"`
- Calibration and identification tasks enabled by default
- No free-flying base (`free_flying_base: false`)
- Defaults to no external force/torque sensors
- Default wrist-mounted marker configuration (6-DOF measurable)

Example robots: UR10, Staubli TX40

### `humanoid_robot.yaml`

Extends `base_robot_config.yaml` for humanoid and mobile manipulator robots (e.g., TIAGo, TALOS). Key characteristics:

- `robot.type: "humanoid"`
- Coupled wrists enabled (`has_coupled_wrist: true`)
- Mobile base properties (`has_mobile_base: true`, differential drive)
- Head camera and force/torque sensors enabled
- Higher signal processing sampling rates (1000 Hz)
- Eye-hand calibration section available
- Default hand-mounted marker configuration (position-only, 3-DOF measurable)

Example robots: TIAGo, TALOS

## Template Inheritance

### How `extends:` works

A robot configuration file references a template using the `extends:` key at the top level of the YAML file:

```yaml
# examples/ur10/config/ur10_unified_config.yaml
extends: "../../templates/manipulator_robot.yaml"

robot:
  name: "ur10"
  properties:
    joints:
      active_joints: ["shoulder_pan_joint", "shoulder_lift_joint", ...]
```

When the `UnifiedConfigParser` encounters `extends:`, it:

1. Loads the referenced template file.
2. Recursively resolves any `extends:` chain in the template (e.g., `manipulator_robot.yaml` extends `base_robot_config.yaml`).
3. Deep-merges the template with the robot config — robot-specific values override template defaults, but template values for keys not present in the robot config are preserved.

### Override rules

- **Scalar values** (strings, numbers, booleans): Robot value replaces template value.
- **Dictionaries**: Merged recursively. Keys present in both are merged; keys only in the template are kept; keys only in the robot config are added.
- **Lists**: Replaced entirely (not merged). If the template defines a list of markers and the robot config also defines markers, the robot's list replaces the template's.

### Multi-level inheritance

A config can inherit through multiple levels. For example, a UR10 robot config can extend `manipulator_robot.yaml`, which itself extends `base_robot_config.yaml`. The parser resolves the chain automatically:

```
ur10_unified_config.yaml
  └─ extends: manipulator_robot.yaml
       └─ extends: base_robot_config.yaml
```

The final merged config includes settings from all three levels, with the robot-specific settings taking highest priority.

## Usage Examples

### UR10 extending manipulator_robot.yaml

```yaml
# examples/ur10/config/ur10_unified_config.yaml
extends: "../../templates/manipulator_robot.yaml"

meta:
  schema_version: "2.0"

robot:
  name: "ur10"
  properties:
    joints:
      active_joints:
        - "shoulder_pan_joint"
        - "shoulder_lift_joint"
        - "elbow_joint"
        - "wrist_1_joint"
        - "wrist_2_joint"
        - "wrist_3_joint"
      joint_limits:
        position: [-6.28, -6.28, -3.92, -6.28, -6.28, -6.28]
        velocity: [2.09, 2.09, 2.09, 2.09, 2.09, 2.09]
    mechanics:
      reduction_ratios: [50.0, 50.0, 50.0, 50.0, 50.0, 50.0]
      friction_coefficients:
        viscous: [0.1, 0.12, 0.08, 0.05, 0.05, 0.05]
        static: [0.05, 0.04, 0.06, 0.03, 0.03, 0.03]

tasks:
  calibration:
    enabled: true
    kinematics:
      base_frame: "universe"
      tool_frame: "wrist_3_link"
    measurements:
      markers:
        - ref_joint: "wrist_3_joint"
          position: [0.0, 0.0, 0.1]
          measure: [true, true, true, false, false, false]

  identification:
    enabled: true
    problem:
      has_friction: true
      has_external_forces: false
    signal_processing:
      sampling_frequency: 500.0
      cutoff_frequency: 50.0
```

Because `manipulator_robot.yaml` already enables calibration and identification, provides a default marker template, and disables external forces, the UR10 config only needs to specify the robot-specific values (joint names, limits, friction coefficients, frame names).

### Creating a new robot config from a template

1. Identify which template fits your robot type:
   - Fixed-base arm → `manipulator_robot.yaml`
   - Humanoid or mobile manipulator → `humanoid_robot.yaml`
   - Other → `base_robot_config.yaml` (directly)

2. Create your config file with `extends:` pointing to the template:

```yaml
# examples/my_robot/config/my_robot_unified_config.yaml
extends: "../../templates/manipulator_robot.yaml"

robot:
  name: "my_robot"
  properties:
    joints:
      active_joints:
        - "joint_1"
        - "joint_2"
        - ...
  # Override only the values that differ from the template
```

3. Set `tasks.<task>.enabled: true` for each task your robot supports.
4. Fill in robot-specific values for joint names, limits, friction, marker configurations, etc.
5. Leave any parameter with its template default if you don't need to change it.

## Best Practices

### Organizing your config

- **Keep robot specifics under `robot.properties.*`** — Joint configuration, mechanical parameters, and coupling information all go under `robot.properties`.
- **Keep task specifics under `tasks.<task>.*`** — Calibration markers go under `tasks.calibration.measurements`, identification settings go under `tasks.identification.problem`, etc.
- **Keep environment/execution settings under `environment.*`** — Directory paths, logging, and visualization are environment concerns.

### What NOT to put in the template

- The base template is the root; specialized templates may use `extends:`
  to inherit it, as described above.
- Do **not** put robot-specific joint names or limits in templates — those belong in the robot config.
- Do **not** delete template keys — override them with your values or leave them as defaults.

### Template modification

- If you add a new task type or a new parameter that applies across all robots, add it to `base_robot_config.yaml`.
- If you add a parameter specific to manipulators or humanoids, add it to the respective template.
- Keep templates backward-compatible: adding a new default parameter is safe (existing configs inherit the default), but removing or renaming a parameter will break existing configs.

### Testing

After creating or modifying a robot config, verify it parses correctly:

```python
from figaroh.utils.config_parser import UnifiedConfigParser
parser = UnifiedConfigParser("config/my_robot_unified_config.yaml")
config = parser.parse()
print(config["robot"]["name"])  # Should print "my_robot"
```

Run the full test suite to check for regressions:

```bash
conda activate figaroh-dev
pytest tests/ -v
```

## See Also

- `examples/create_example.sh` — Script to scaffold a new robot example folder based on the TIAGo layout.
- `examples/ur10/config/ur10_unified_config.yaml` — Real-world example of a manipulator extending `manipulator_robot.yaml`.
- `examples/tiago/config/tiago_unified_config.yaml` — Real-world example of a humanoid extending `humanoid_robot.yaml`.
- `examples/staubli_tx40/config/staubli_tx40_unified_config.yaml` — Example of a manipulator extending `manipulator_robot.yaml`.
