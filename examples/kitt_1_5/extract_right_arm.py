"""
Extract the right-arm sub-model from the KITT1_5V3 full URDF.

Keeps the chain up to `right_flange_link` (the flange = tool0) and drops
everything after it (gripper base, fingers, camera). The root joint
`right_arm_joint` (fixed) becomes the new model root.

Result: `urdf/right_arm_robot.urdf` — fixed-base 7-DOF arm (nq=7):
    right_arm_joint (root, fixed)
      -> Base_R -> right_arm_joint_1..7 -> Joint7_R
      -> right_flange_joint (fixed) -> right_flange_link  (tool0)
"""
from __future__ import annotations

import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parent
FULL_URDF = ROOT / "urdf" / "kitt_full_robot.urdf"
OUT_URDF = ROOT / "urdf" / "right_arm_robot.urdf"

# Chain root: keep from this joint, then stop after right_flange_link
ROOT_JOINT = "right_arm_joint"
TOOL0_LINK = "right_flange_link"
ROBOT_NAME = "kitt_1_5_right_arm"


def main() -> None:
    tree = ET.parse(FULL_URDF)
    root = tree.getroot()

    links = {l.get("name"): l for l in root.findall("link")}
    joints = {j.get("name"): j for j in root.findall("joint")}
    joint_list = list(joints.values())

    if ROOT_JOINT not in joints:
        raise SystemExit(f"root joint {ROOT_JOINT} not found in {FULL_URDF}")

    # BFS from the root joint, stop the chain at tool0 link.
    keep_links, keep_joints = [], []
    visited = set()
    queue = [ROOT_JOINT]
    while queue:
        jn = queue.pop(0)
        if jn in visited:
            continue
        visited.add(jn)
        joint = joints[jn]
        child = joint.find("child").get("link")
        keep_links.append(child)
        keep_joints.append(joint)
        if child == TOOL0_LINK:
            continue  # stop: do not descend into gripper/camera
        for j in joint_list:
            if j.find("parent").get("link") == child:
                queue.append(j.get("name"))

    # Build the new robot element. Keep the root `<robot>` attributes.
    new_root = ET.Element("robot")
    new_root.set("name", ROBOT_NAME)

    # The root joint becomes the model root: parent = base_link
    root_joint = joints[ROOT_JOINT]
    root_joint.find("parent").set("link", "base_link")
    new_root.append(links["base_link"])
    for jn in keep_joints:
        new_root.append(jn)
    for ln in keep_links:
        new_root.append(links[ln])

    tree_out = ET.ElementTree(new_root)
    ET.indent(tree_out, space="  ")
    tree_out.write(OUT_URDF, encoding="utf-8", xml_declaration=True)
    print(f"Wrote {OUT_URDF}")

    # Report
    n_revolute = sum(1 for j in keep_joints if j.get("type") == "revolute")
    print(f"kept joints: {[j.get('name') for j in keep_joints]}")
    print(f"kept links:  {keep_links}")
    print(f"revolute DOF: {n_revolute}")

    # Copy the meshes referenced by the kept links.
    refs = set()
    for ln in keep_links + ["base_link"]:
        for mesh in links[ln].iter("mesh"):
            refs.add(mesh.get("filename"))
    for ref in sorted(refs):
        src = FULL_URDF.parent / ref
        dst = OUT_URDF.parent / ref
        if src.exists():
            if src.resolve() != dst.resolve():
                shutil.copy2(src, dst)
        else:
            print(f"  [warn] mesh not found: {ref}")
    print(f"copied {len(refs)} mesh files")


if __name__ == "__main__":
    main()
