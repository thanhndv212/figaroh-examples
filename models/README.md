# Robot geometry sources

The example URDFs reference meshes as `package://<package>/...`. Pinocchio
resolves these URIs against the `package_dirs` an example passes to
`load_robot` (this `models/` directory) and then against `ROS_PACKAGE_PATH`.
Every robot used by the examples must load from a clean checkout plus the
pinned packages below. Nothing here depends on a developer's own
`ROS_PACKAGE_PATH`; `tests/test_robot_geometry.py` checks this on both
Pinocchio profiles in CI.

## Set up

```bash
python scripts/fetch_models.py      # sparse, pinned checkouts into models/external/ (gitignored)
export ROS_PACKAGE_PATH="$(python scripts/fetch_models.py --print-ros-package-path)"
```

The fetch is idempotent: a package already at its pinned commit is left alone.
CI runs the same script, so the revisions below are the ones it tests.

## Fetched at pinned revisions (not in this repository)

These packages are fetched from upstream and must not be copied into this
repository.

| Package | Source @ commit | Licence | Needed by |
|---|---|---|---|
| `agimus-demos` (`meshes/`) | [agimus/agimus-demos@7858fd3](https://github.com/agimus/agimus-demos/tree/7858fd3da9b31b4a17cc265722a4647567cda8cd) | BSD-2-Clause | `ur10_robot.urdf`: RealSense D435 mount |
| `tiago_description` (`meshes/`) | [pal-robotics/tiago_robot@234b653](https://github.com/pal-robotics/tiago_robot/tree/234b653b2a063e89bcdf7c9b2c419272531eab5a) | Apache-2.0 | both TIAGo URDFs: `arm_5` wrist-2010 meshes missing from the vendored copy |
| `pmb2_description` (`meshes/`) | [pal-robotics/pmb2_robot@90b419d](https://github.com/pal-robotics/pmb2_robot/tree/90b419d64a786cbcf3ba22623b68259f354698aa) | Apache-2.0 (declared in `package.xml`; no LICENSE file) | both TIAGo URDFs: caster wheels |
| `pal_wsg_gripper_description` (`meshes/`) | [pal-robotics/pal_wsg_gripper@ee54226](https://github.com/pal-robotics/pal_wsg_gripper/tree/ee542262a264f220142c787c0e005b5664bffa66) | **Proprietary** (declared in `package.xml`; no LICENSE file). Fetch for local use only; do not redistribute. | `tiago_48_schunk.urdf`: WSG gripper |

To change a pin, edit `SOURCES` in `scripts/fetch_models.py`, rerun the
fetch, and run `python -m pytest tests/test_robot_geometry.py`.

## Vendored in `models/`

| Package | Licence as shipped | Used by examples |
|---|---|---|
| `hey5_description` | Apache-2.0 (LICENSE, `package.xml`) | TIAGo Hey5 hand, including `palm_collision.stl` |
| `pmb2_description` | Apache-2.0 (`package.xml`) | TIAGo base (partial; casters are fetched) |
| `tiago_description` | none recorded | TIAGo (partial; wrist-2010 meshes are fetched) |
| `talos_description` | none recorded | TALOS |
| `staubli_tx40_description` | none recorded | Stäubli TX40 |
| `ur_description` | BSD-3-Clause (LICENSE) | UR10 |
| `realsense2_description` | Apache-2.0 (`package.xml`) | UR10 camera |
| `tiago_pro_description`, `tiago_pro_head_description`, `omni_base_description`, `pal_sea_arm_description`, `pal_pro_gripper_description`, `pal_urdf_utils` | Apache-2.0 (LICENSE) | TIAGo Pro scripts |
| `pal_atc_description` | none recorded | TIAGo Pro scripts |

"None recorded" means the vendored copy carries no LICENSE file or licence
tag, so its redistribution terms are not established here.

`so101` uses no meshes.
