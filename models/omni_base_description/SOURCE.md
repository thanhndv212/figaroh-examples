# Source

Vendored for the `tiago_pro` example (`examples/tiago_pro/urdf/tiago_pro.urdf`).

- Upstream: https://github.com/pal-robotics/omni_base_robot (package `omni_base_description`)
- Version: `2.17.0`
- License: Apache License 2.0 (see `LICENSE`)

Only the mesh files referenced by `tiago_pro.urdf` are included — not the
full package (no URDF/xacro/config). `meshes/base/base_antena_link.stl` is
copied from upstream's `base_antenna_link.stl`; the URDF was generated
against an older xacro revision that used the misspelled filename.
