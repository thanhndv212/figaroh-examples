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

# Joint-control GUI grouping, by name prefix (order = display order).
# Wheels and gripper_right_tool_mount_joint use JointModelRUBZ/RUB* (unbounded
# revolute, nq=2 cos/sin pair) since they have no URDF <limit> -- the slider
# for those is a plain angle in [-pi, pi] that gets converted on update.
_JOINT_GROUPS = [
    ("Mobile base (wheels)", "wheel_"),
    ("Torso", "torso_"),
    ("Head", "head_"),
    ("Left arm", "arm_left_"),
    ("Left gripper", "gripper_left_"),
    ("Right arm", "arm_right_"),
    ("Right tool mount", "gripper_right_"),
]


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

    # ── Joint control ────────────────────────────────────────────────────────
    q_neutral = q.copy()
    slider_handles = []  # (joint_id, is_unbounded, slider)

    def _set_from_slider(joint_id: int, is_unbounded: bool, value: float) -> None:
        idx_q = model.joints[joint_id].idx_q
        if is_unbounded:
            q[idx_q] = np.cos(value)
            q[idx_q + 1] = np.sin(value)
        else:
            q[idx_q] = value
        _update_display()

    grouped = {label: [] for label, _ in _JOINT_GROUPS}
    for jid in range(1, model.njoints):
        name = model.names[jid]
        for label, prefix in _JOINT_GROUPS:
            if name.startswith(prefix):
                grouped[label].append(jid)
                break

    for label, prefix in _JOINT_GROUPS:
        joint_ids = grouped[label]
        if not joint_ids:
            continue
        with server.gui.add_folder(label):
            for jid in joint_ids:
                joint = model.joints[jid]
                name = model.names[jid]
                is_unbounded = joint.nq == 2
                if is_unbounded:
                    lo, hi, init = -np.pi, np.pi, 0.0
                else:
                    lo = float(model.lowerPositionLimit[joint.idx_q])
                    hi = float(model.upperPositionLimit[joint.idx_q])
                    init = float(q_neutral[joint.idx_q])
                slider = server.gui.add_slider(
                    name, lo, hi, (hi - lo) / 200.0, init
                )

                def _make_callback(joint_id=jid, unbounded=is_unbounded):
                    def _callback(event) -> None:
                        _set_from_slider(joint_id, unbounded, event.target.value)

                    return _callback

                slider.on_update(_make_callback())
                slider_handles.append((jid, is_unbounded, slider))
        print(f"  {label}: {len(joint_ids)} joint slider(s)")

    # ── Frame highlight ──────────────────────────────────────────────────────
    highlight_state = {"fid": None}
    highlight_sphere = server.scene.add_icosphere(
        "highlight/marker",
        radius=args.frame_size * 0.35,
        color=(255, 220, 0),
        visible=False,
    )

    def _refresh_highlight() -> None:
        fid = highlight_state["fid"]
        if fid is None:
            highlight_sphere.visible = False
            return
        T = data.oMf[fid]
        highlight_sphere.position = T.translation
        highlight_sphere.visible = True
        frame_handles[fid].axes_length = args.frame_size * 4.0
        frame_handles[fid].axes_radius = args.frame_size * 0.04 * 4.0
        parent_joint = model.names[model.frames[fid].parentJoint]
        info_md.content = (
            f"**{model.frames[fid].name}**\n\n"
            f"parent joint: `{parent_joint}`\n\n"
            f"position (m): `{T.translation.round(4).tolist()}`"
        )

    if frame_handles:
        frame_names = sorted(model.frames[fid].name for fid in frame_handles)
        name_to_fid = {model.frames[fid].name: fid for fid in frame_handles}
        default_name = (
            "gripper_right_tool_holder"
            if "gripper_right_tool_holder" in name_to_fid
            else frame_names[0]
        )
        with server.gui.add_folder("Frame highlight"):
            frame_dropdown = server.gui.add_dropdown(
                "Frame", ["(none)"] + frame_names, initial_value=default_name
            )
            info_md = server.gui.add_markdown("")

            @frame_dropdown.on_update
            def _(_event) -> None:
                prev_fid = highlight_state["fid"]
                if prev_fid is not None:
                    frame_handles[prev_fid].axes_length = args.frame_size
                    frame_handles[prev_fid].axes_radius = args.frame_size * 0.04
                selected = frame_dropdown.value
                highlight_state["fid"] = (
                    None if selected == "(none)" else name_to_fid[selected]
                )
                _refresh_highlight()

        highlight_state["fid"] = name_to_fid[default_name]
        _refresh_highlight()

    _orig_update_display = _update_display

    def _update_display():
        _orig_update_display()
        _refresh_highlight()

    with server.gui.add_folder("Reset"):
        reset_btn = server.gui.add_button("Reset to neutral pose")

        @reset_btn.on_click
        def _(_event) -> None:
            q[:] = q_neutral
            for jid, is_unbounded, slider in slider_handles:
                joint = model.joints[jid]
                slider.value = 0.0 if is_unbounded else float(q_neutral[joint.idx_q])
            _update_display()

    print("\nViser running at http://localhost:8080 — Ctrl+C to quit.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
