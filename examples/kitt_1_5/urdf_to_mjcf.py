#!/usr/bin/env python3
"""Convert the extracted right-arm URDF submodel into an equivalent MJCF.

MuJoCo's built-in URDF parser drops the mass of bodies hanging off a *fixed*
root joint (it treats the chain below a fixed root as non-physical).  That
makes mj_inverse disagree with the Pinocchio RNEA used by the identifier.

Instead, this script emits a hand-rolled MJCF that keeps every URDF link as a
body (Base_R ... right_flange_link), so MuJoCo's dynamics match Pinocchio's
exactly.  Inertia tensors (with off-diagonal terms) are diagonalised via an
eigendecomposition and written as `diaginertia` + `quat`.

Usage (run from examples/kitt_1_5/):
    python urdf_to_mjcf.py
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
URDF_PATH = HERE / "urdf" / "right_arm_robot.urdf"
MJCF_PATH = HERE / "urdf" / "right_arm_robot.mjcf"

GRAVITY = "0 0 -9.81"

_NS = {"u": "http://www.ros.org/wiki/urdf"} if False else {}


def _vec(text) -> str:
    if isinstance(text, str):
        text = [float(x) for x in text.split()]
    return " ".join(f"{float(x):.9g}" for x in text)


def _xyz(xml) -> np.ndarray:
    origin = xml.find("origin")
    if origin is None or origin.get("xyz") is None:
        return np.zeros(3)
    return np.array([float(x) for x in origin.get("xyz").split()])


def _rpy(xml) -> np.ndarray:
    origin = xml.find("origin")
    if origin is None or origin.get("rpy") is None:
        return np.zeros(3)
    return np.array([float(x) for x in origin.get("rpy").split()])


def rot_from_rpy(rpy: np.ndarray) -> np.ndarray:
    """R = Rz(y) @ Ry(p) @ Rx(r) (URDF convention)."""
    rx, ry, rz = rpy
    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def mat_to_quat(R: np.ndarray) -> np.ndarray:
    """Rotation matrix -> [w, x, y, z] (MuJoCo order)."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        return np.array(
            [0.25 * s, (R[2, 1] - R[1, 2]) / s,
             (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
        )
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2.0
    q = np.zeros(4)
    q[0] = (R[k, j] - R[j, k]) / s
    q[i + 1] = 0.25 * s
    q[j + 1] = (R[j, i] + R[i, j]) / s
    q[k + 1] = (R[k, i] + R[i, k]) / s
    return q


def diagonalise_inertia(i3: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """I = V diag(d) V^T -> (d, quat(V)) with d sorted so that the largest
    eigenvalue is first (MuJoCo wants diaginertia with the largest first)."""
    vals, vecs = np.linalg.eigh(i3)
    order = np.argsort(-vals)
    vals, vecs = vals[order], vecs[:, order]
    # Ensure a right-handed frame (det(R) = +1)
    if np.linalg.det(vecs) < 0:
        vecs[:, 0] *= -1.0
    return vals, mat_to_quat(vecs)


def parse_urdf(path: Path):
    tree = ET.parse(path)
    root = tree.getroot()
    links, joints = {}, {}
    for el in root.iter():
        if el.tag == "link":
            links[el.get("name")] = el
        elif el.tag == "joint":
            joints[el.get("name")] = el
    return root, links, joints


def link_inertial(link_el):
    """Extract mass, com, body-frame inertia 3x3 for a URDF link."""
    inert = link_el.find("inertial")
    if inert is None:
        return None
    mass = float(inert.find("mass").get("value"))
    iel = inert.find("inertia")
    I = np.array(
        [
            [float(iel.get("ixx")), float(iel.get("ixy")), float(iel.get("ixz"))],
            [float(iel.get("ixy")), float(iel.get("iyy")), float(iel.get("iyz"))],
            [float(iel.get("ixz")), float(iel.get("iyz")), float(iel.get("izz"))],
        ]
    )
    R = rot_from_rpy(_rpy(inert))  # inertial frame -> body frame
    com = _xyz(inert)
    if not np.allclose(R, np.eye(3)):
        I = R @ I @ R.T  # express inertia in the body frame
    return mass, com, I


def build_mjcf():
    _, links, joints = parse_urdf(URDF_PATH)

    # Locate the root joint (fixed, parent = base_link) -> root body.
    root_joint = next(
        j for j in joints.values()
        if j.get("type") == "fixed" and j.find("parent").get("link") == "base_link"
    )
    root_link_name = root_joint.find("child").get("link")

    def mesh_name(link_el) -> str | None:
        for g in link_el.findall("visual"):
            mesh = g.find("geometry/mesh")
            if mesh is not None:
                return mesh.get("filename")
        return None

    # mesh names referenced by the kept links (only these are emitted as assets)
    meshes = {}
    for lname, lel in links.items():
        fn = mesh_name(lel)
        if fn:
            meshes[fn] = fn

    def body_xml(name: str, pos: np.ndarray, quat: np.ndarray, depth: int) -> str:
        pad = "  " * depth
        link_el = links[name]
        lines = [f'{pad}<body name="{name}" pos="{_vec(pos)}" quat="{_vec(quat)}">']

        inert = link_inertial(link_el)
        if inert is not None:
            mass, com, I = inert
            # Full inertia tensor in the body frame, written with MuJoCo's
            # `fullinertia` attribute (same [ixx iyy izz ixy ixz iyz] order
            # as URDF, expressed in the inertial frame = body frame here as
            # no quat is given).  Using the full tensor avoids the
            # diaginertia + quat eigendecomposition round-trip, which did
            # not reconstruct the same inertia in MuJoCo and broke
            # mj_inverse == RNEA.
            ixx, iyy, izz = I[0, 0], I[1, 1], I[2, 2]
            ixy, ixz, iyz = I[0, 1], I[0, 2], I[1, 2]
            lines.append(
                f'{pad}  <inertial pos="{_vec(com)}" mass="{mass:.9g}" '
                f'fullinertia="{ixx:.9g} {iyy:.9g} {izz:.9g} '
                f'{ixy:.9g} {ixz:.9g} {iyz:.9g}"/>'
            )
        fn = mesh_name(link_el)
        if fn:
            lines.append(
                f'{pad}  <geom type="mesh" mesh="{fn}" pos="0 0 0" '
                f'contype="0" conaffinity="0"/>'
            )

        # joints whose child is this link live inside this body
        for jname, jel in joints.items():
            if jel.find("child").get("link") != name:
                continue
            jtype = jel.get("type")
            if jtype == "fixed":
                continue  # fixed joints are folded into the child body transform
            axis = np.array([float(x) for x in jel.find("axis").get("xyz").split()])
            # The child body frame coincides with the URDF joint frame (the
            # URDF child link has no extra transform), so the hinge axis is
            # already expressed in the child body frame.
            jrng = jel.find("limit")
            rng = ""
            if jrng is not None:
                lo, hi = float(jrng.get("lower")), float(jrng.get("upper"))
                rng = f' range="{lo:.6g} {hi:.6g}"'
            lines.append(
                f'{pad}  <joint name="{jname}" type="hinge" axis="{_vec(axis)}"'
                f' pos="0 0 0"{rng}/>'
            )

        # child bodies (motion through a revolute joint, or welded fixed)
        for jname, jel in joints.items():
            if jel.find("parent").get("link") != name:
                continue
            child = jel.find("child").get("link")
            pos = _xyz(jel)
            quat_c = mat_to_quat(rot_from_rpy(_rpy(jel)))
            lines.append(body_xml(child, pos, quat_c, depth + 1))

        lines.append(f"{pad}</body>")
        return "\n".join(lines)

    root_pos = _xyz(root_joint)
    root_quat = mat_to_quat(rot_from_rpy(_rpy(root_joint)))

    parts = [
        '<?xml version="1.0"?>',
        "<mujoco model=\"kitt_1_5_right_arm\">",
        '  <compiler angle="radian" meshdir="." autolimits="true"/>',
        f'  <option gravity="{GRAVITY}"/>',
        "  <asset>",
    ]
    for fn in sorted(meshes):
        parts.append(f'    <mesh name="{fn}" file="{fn}"/>')
    parts.append("  </asset>")
    parts.append("  <worldbody>")
    parts.append(body_xml(root_link_name, root_pos, root_quat, 2))
    parts.append("  </worldbody>")
    parts.append("</mujoco>")

    MJCF_PATH.write_text("\n".join(parts) + "\n", encoding="utf-8")
    print(f"Wrote {MJCF_PATH} ({len(meshes)} meshes referenced)")


if __name__ == "__main__":
    build_mjcf()
