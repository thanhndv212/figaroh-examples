#!/usr/bin/env python3
"""
Replay the D-optimal poses already saved in data/optimal_configs.yaml in Viser.

Unlike generate_optimal_configs.py, this does NOT regenerate the pool or
re-run Detmax (which isn't seeded — re-running it picks a different subset).
It just loads the existing YAML and displays it, config by config.

Usage:
    python3 visualize_optimal_configs.py
    python3 visualize_optimal_configs.py --configs data/optimal_configs.yaml
"""

import argparse
import time
from pathlib import Path

import numpy as np
import pinocchio as pin
import viser
import viser.transforms as vtf
import yaml

from generate_optimal_configs import (
    _pkg_dirs, _load_mesh, _q_from_active, _check_collisions, _display_config,
)

_URDF_DEFAULT = Path(__file__).parent / "tiago_pro_local.urdf"
_SRDF = Path(__file__).parent / "tiago_pro.srdf"
_CONFIGS_DEFAULT = Path(__file__).parent / "data" / "optimal_configs.yaml"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", default=str(_URDF_DEFAULT))
    parser.add_argument("--configs", default=str(_CONFIGS_DEFAULT))
    args = parser.parse_args()

    print(f"Loading model from {args.urdf} ...")
    pkg_dirs = _pkg_dirs()
    model = pin.buildModelFromUrdf(args.urdf)
    try:
        _, collision_model, visual_model = pin.buildModelsFromUrdf(
            args.urdf, package_dirs=pkg_dirs
        )
    except ValueError as e:
        print(f"[warn] geometry loading failed ({e})\n  → collision checking disabled.")
        collision_model = pin.GeometryModel()
        visual_model = pin.GeometryModel()
    data = model.createData()

    collision_model.addAllCollisionPairs()
    pin.removeCollisionPairsFromXML(model, collision_model, _SRDF.read_text(), verbose=False)

    with open(args.configs) as f:
        cfg = yaml.safe_load(f)
    joint_names = cfg["calibration_joint_names"]
    configs = [np.array(c) for c in cfg["calibration_joint_configurations"]]
    print(f"Loaded {len(configs)} saved poses from {args.configs}")

    print("\nStarting Viser at http://localhost:8080 ...")
    print("Controls: Enter → next config | q → quit")
    server = viser.ViserServer()
    time.sleep(0.5)

    q0 = pin.neutral(model)
    visual_data = pin.GeometryData(visual_model)
    pin.forwardKinematics(model, data, q0)
    pin.updateGeometryPlacements(model, data, visual_model, visual_data)

    print("Loading meshes into Viser ...")
    mesh_handles = {}
    for i, geom_obj in enumerate(visual_model.geometryObjects):
        mesh = _load_mesh(geom_obj.meshPath)
        if mesh is None:
            continue
        T = visual_data.oMg[i]
        vertices = np.array(mesh.vertices, dtype=np.float32) * geom_obj.meshScale
        faces = np.array(mesh.faces, dtype=np.uint32)
        handle = server.scene.add_mesh_simple(
            f"robot/{geom_obj.name}",
            vertices=vertices,
            faces=faces,
            position=T.translation,
            wxyz=vtf.SO3.from_matrix(T.rotation).wxyz,
            color=(0.8, 0.8, 0.8),
        )
        mesh_handles[i] = handle
    print(f"Loaded {len(mesh_handles)} mesh objects.")

    for idx, active_vals in enumerate(configs):
        q = _q_from_active(model, active_vals, joint_names)
        _display_config(server, mesh_handles, model, visual_model, visual_data,
                         collision_model, data, q, idx, len(configs), joint_names, active_vals)
        print("Press Enter for next config, 'q' to quit.")
        line = input().strip().lower()
        if line == "q":
            break

    print("\nDone.")


if __name__ == "__main__":
    main()
