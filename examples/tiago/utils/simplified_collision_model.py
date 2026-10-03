# Copyright [2021-2025] Thanh Nguyen
# Copyright [2022-2023] [CNRS, Toward SAS]

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

# http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import annotations

from typing import Any, List

import hppfcl
import numpy as np
import pinocchio as pin


def Capsule(
    name: str,
    joint: int,
    radius: float,
    length: float,
    placement: Any,
    color: List[float] | None = None,
) -> pin.GeometryObject:
    """Create a Pinocchio::FCL geometry capsule (cylinder workaround).

    Args:
        name: Name of the geometry object
        joint: Joint index to attach the geometry to
        radius: Cylinder radius
        length: Cylinder length
        placement: SE3 placement relative to joint frame
        color: RGBA color vector (default: light blue)

    Returns:
        Pinocchio geometry object
    """
    if color is None:
        color = [0.7, 0.7, 0.98, 1.0]
    # They should be capsules ... but hppfcl current version is buggy with Capsules...
    # hppgeom = hppfcl.Capsule(radius, length)
    hppgeom = hppfcl.Cylinder(radius, length)
    geom = pin.GeometryObject(name, joint, placement, hppgeom)
    geom.meshColor = np.array(color)
    return geom


def Box(
    name: str,
    joint: int,
    x: float,
    y: float,
    z: float,
    placement: Any,
    color: List[float] | None = None,
) -> pin.GeometryObject:
    """Create a Pinocchio::FCL box geometry.

    Args:
        name: Name of the geometry object
        joint: Joint index to attach the geometry to
        x: Box dimension along X
        y: Box dimension along Y
        z: Box dimension along Z
        placement: SE3 placement relative to joint frame
        color: RGBA color vector (default: light blue)

    Returns:
        Pinocchio geometry object
    """
    if color is None:
        color = [0.7, 0.7, 0.98, 1.0]
    hppgeom = hppfcl.Box(x, y, z)
    geom = pin.GeometryObject(name, joint, placement, hppgeom)
    geom.meshColor = np.array(color)
    return geom


def Sphere(
    name: str,
    joint: int,
    radius: float,
    placement: Any,
    color: List[float] | None = None,
) -> pin.GeometryObject:
    """Create a Pinocchio::FCL sphere geometry.

    Args:
        name: Name of the geometry object
        joint: Joint index to attach the geometry to
        radius: Sphere radius
        placement: SE3 placement relative to joint frame
        color: RGBA color vector (default: light blue)

    Returns:
        Pinocchio geometry object
    """
    if color is None:
        color = [0.7, 0.7, 0.98, 1.0]
    hppgeom = hppfcl.Sphere(radius)
    geom = pin.GeometryObject(name, joint, placement, hppgeom)
    geom.meshColor = np.array(color)
    return geom


def build_tiago_simplified(robot: Any) -> Any:
    """Build a simplified collision model for TIAGo.

    Adds box and capsule collision geometries for the torso, base,
    forearm, and head of the TIAGo robot.

    Args:
        robot: TIAGo robot model (with model, geom_model, visual_model)

    Returns:
        Robot model with added collision geometries
    """
    xyz_1 = np.array([-0.028, 0, -0.01])
    xyz_2 = np.array([0.005, 0, -0.35])
    xyz_3 = np.array([0, 0, 0.20])
    xyz_4 = np.array([0, 0, 0])
    xyz_5 = np.array([-0.03, 0.09, 0])

    robot.geom_model.addGeometryObject(
        Box("torso_up_box", 1, 0.28, 0.35, 0.2, pin.SE3(np.eye(3), xyz_1))
    )
    robot.visual_model.addGeometryObject(
        Box(
            "torso_up_box",
            robot.model.getJointId("torso_lift_joint"),
            0.28,
            0.35,
            0.2,
            pin.SE3(np.eye(3), xyz_1),
        )
    )
    robot.geom_model.addGeometryObject(
        Box(
            "torso_low_box",
            robot.model.getJointId("torso_lift_joint"),
            0.221,
            0.26,
            0.35,
            pin.SE3(np.eye(3), xyz_2),
        )
    )
    robot.visual_model.addGeometryObject(
        Box(
            "torso_low_box",
            robot.model.getJointId("torso_lift_joint"),
            0.221,
            0.26,
            0.35,
            pin.SE3(np.eye(3), xyz_2),
        )
    )
    robot.geom_model.addGeometryObject(
        Capsule(
            "base_cap",
            robot.model.getJointId("universe"),
            0.3,
            0.25,
            pin.SE3(np.eye(3), xyz_3),
        )
    )
    robot.visual_model.addGeometryObject(
        Capsule(
            "base_cap",
            robot.model.getJointId("universe"),
            0.3,
            0.25,
            pin.SE3(np.eye(3), xyz_3),
        )
    )
    robot.geom_model.addGeometryObject(
        Capsule(
            "forearm_cap",
            robot.model.getJointId("arm_5_joint"),
            0.10,
            0.40,
            pin.SE3(np.eye(3), xyz_4),
        )
    )
    robot.visual_model.addGeometryObject(
        Capsule(
            "forearm_cap",
            robot.model.getJointId("arm_5_joint"),
            0.10,
            0.40,
            pin.SE3(np.eye(3), xyz_4),
        )
    )
    robot.geom_model.addGeometryObject(
        Capsule(
            "head_cap",
            robot.model.getJointId("head_2_joint"),
            0.17,
            0.25,
            pin.SE3(np.eye(3), xyz_5),
        )
    )
    robot.visual_model.addGeometryObject(
        Capsule(
            "head_cap",
            robot.model.getJointId("head_2_joint"),
            0.17,
            0.25,
            pin.SE3(np.eye(3), xyz_5),
        )
    )

    for k in range(len(robot.geom_model.geometryObjects)):
        print(
            "object number %d" % k,
            robot.geom_model.geometryObjects[k].name,
        )

    arm_link_names: List[str] = [
        "forearm_cap"
        # "arm_4_link_0",
        # "arm_5_link_0",
        # "arm_6_link_0",
        # "wrist_ft_link_0",
        # "wrist_ft_tool_link_0",
    ]
    arm_link_ids = [robot.geom_model.getGeometryId(k) for k in arm_link_names]
    mask_link_names: List[str] = [
        "torso_up_box",
        "torso_low_box",
        "base_cap",
        "head_cap",
    ]
    mask_link_ids = [robot.geom_model.getGeometryId(k) for k in mask_link_names]
    for i in mask_link_ids:
        for j in arm_link_ids:
            robot.geom_model.addCollisionPair(pin.CollisionPair(i, j))
    print(
        "number of collision pairs of simplified model is: ",
        len(robot.geom_model.collisionPairs),
    )

    return robot
