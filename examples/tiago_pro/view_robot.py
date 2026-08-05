#!/usr/bin/env python3
"""Interactive Viser visualization of the Tiago Pro model.

Meshes are resolved from the shared `../../models/` package directory
(same convention as the other examples in this repo), which holds the
subset of mesh files referenced by `urdf/tiago_pro.urdf` — see each
`models/<package>_description/SOURCE.md` for provenance. Any reference
not covered there fails geometry loading gracefully (frame axes only).
"""
import time
from pathlib import Path

import numpy as np
import pinocchio as pin
import trimesh
import viser
import viser.transforms as vtf

_HERE = Path(__file__).parent
_URDF_DEFAULT = _HERE / "urdf" / "tiago_pro.urdf"
_MODELS_DIR = _HERE.parents[1] / "models"


def _pkg_dirs():
    return [str(_MODELS_DIR)]


def _load_mesh(path: str):
    try:
        mesh = trimesh.load(path, force="mesh")
        if isinstance(mesh, trimesh.Scene):
            mesh = trimesh.util.concatenate(mesh.dump())
        return mesh
    except Exception:
        return None


def main():
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", default=str(_URDF_DEFAULT))
    parser.add_argument("--no-frames", action="store_true", help="Hide frame axes")
    parser.add_argument("--frame-size", type=float, default=0.05,
                        help="Frame axes length in metres (default: 0.05)")
    args = parser.parse_args()

    pkg_dirs = _pkg_dirs()
    model = pin.buildModelFromUrdf(args.urdf)
    try:
        _, _, visual_model = pin.buildModelsFromUrdf(
            args.urdf, package_dirs=pkg_dirs
        )
    except ValueError as e:
        print(f"[warn] geometry loading failed ({e})\n  → visualizing without meshes.")
        visual_model = pin.GeometryModel()
    data = model.createData()
    visual_data = pin.GeometryData(visual_model)
    q = pin.neutral(model)

    print(f"nq = {model.nq}, nv = {model.nv}")

    server = viser.ViserServer()
    time.sleep(0.5)

    mesh_handles = {}
    frame_handles = {}

    def _update_display():
        pin.forwardKinematics(model, data, q)
        pin.updateGeometryPlacements(model, data, visual_model, visual_data)

        for i, handle in mesh_handles.items():
            T = visual_data.oMg[i]
            handle.position = T.translation
            handle.wxyz = vtf.SO3.from_matrix(T.rotation).wxyz

        if not args.no_frames:
            pin.framesForwardKinematics(model, data, q)
            for fid, handle in frame_handles.items():
                T = data.oMf[fid]
                handle.position = T.translation
                handle.wxyz = vtf.SO3.from_matrix(T.rotation).wxyz

    # ── Meshes ────────────────────────────────────────────────────────────────
    pin.forwardKinematics(model, data, q)
    pin.updateGeometryPlacements(model, data, visual_model, visual_data)
    print("Loading meshes ...")
    mesh_handles = {}
    for i, geom_obj in enumerate(visual_model.geometryObjects):
        mesh = _load_mesh(geom_obj.meshPath)
        if mesh is None:
            continue
        T = visual_data.oMg[i]
        vertices = np.array(mesh.vertices, dtype=np.float32) * geom_obj.meshScale
        faces = np.array(mesh.faces, dtype=np.uint32)
        mesh_handles[i] = server.scene.add_mesh_simple(
            f"robot/{geom_obj.name}",
            vertices=vertices,
            faces=faces,
            position=T.translation,
            wxyz=vtf.SO3.from_matrix(T.rotation).wxyz,
            color=(0.8, 0.8, 0.8),
        )
    print(f"Loaded {len(mesh_handles)} mesh objects.")

    # ── Frames ────────────────────────────────────────────────────────────────
    frame_handles = {}
    if not args.no_frames:
        pin.framesForwardKinematics(model, data, q)
        for fid, frame in enumerate(model.frames):
            T = data.oMf[fid]
            frame_handles[fid] = server.scene.add_frame(
                f"frames/{frame.name}",
                position=T.translation,
                wxyz=vtf.SO3.from_matrix(T.rotation).wxyz,
                axes_length=args.frame_size,
                axes_radius=args.frame_size * 0.04,
                show_axes=True,
            )
        print(f"Displaying {len(frame_handles)} frames.")

    print("\nViser running at http://localhost:8080 — Ctrl+C to quit.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
