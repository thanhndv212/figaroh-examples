#!/usr/bin/env python3
"""
sysid_exporters.py

A single-file exporter system that converts a neutral SysID schema (YAML/JSON)
into URDF, SDF, MJCF, and USD (optional) outputs.

Usage:
    python sysid_exporters.py --schema example_schema.yaml --formats urdf,sdf,mjcf

Requirements:
    - Python 3.8+
    - pyyaml (for YAML input): pip install pyyaml
    - For USD export: Pixar USD Python bindings (pxr) installed (optional)

metadata:
  robot_name: my_robot
  schema_version: 0.1
  source: "sysid_experiment_2025-08-31"
  units: "SI"   # meters, kg, seconds, radians

bodies:
  base_link:
    mass: 5.23
    com: [0.01, 0.0, 0.05]   # relative to body frame
    inertia:                 # inertia matrix at COM
      ixx: 0.12
      iyy: 0.11
      izz: 0.09
      ixy: 0.0
      ixz: 0.0
      iyz: 0.0
    collision:
      shape: "mesh"
      mesh_file: "meshes/base_link.stl"
      contact:
        mu_static: 0.8
        mu_dynamic: 0.7
        restitution: 0.1
        stiffness: 10000.0
        damping: 20.0
    visual:
      mesh_file: "meshes/base_link.stl"
      color: [0.8, 0.8, 0.8, 1.0]  # RGBA

joints:
  shoulder_joint:
    type: revolute
    parent: base_link
    child: upper_arm
    axis: [0, 0, 1]
    origin: [0.0, 0.1, 0.0]
    limits:
      lower: -1.57
      upper: 1.57
      effort: 50.0
      velocity: 2.0
    dynamics:
      damping: 0.03
      friction: 0.12
      backlash: 0.0

actuators:
  shoulder_motor:
    joint: shoulder_joint
    model: "dc_motor"
    torque_constant: 0.045     # Nm/A
    gear_ratio: 100
    max_current: 2.5
    efficiency: 0.9
    delay: 0.002               # seconds
    saturation: 25.0           # Nm

sensors:
  imu_1:
    type: imu
    parent: base_link
    pose: [0, 0, 0, 0, 0, 0]   # xyz + rpy
    noise:
      accel_stddev: [0.02, 0.02, 0.02]
      gyro_stddev: [0.001, 0.001, 0.001]
    bias:
      accel: [0.01, 0.0, -0.02]
      gyro: [0.0, 0.0, 0.0005]
    latency: 0.001

contacts:
  foot:
    body: foot_link
    surface:
      mu_static: 0.85
      mu_dynamic: 0.75
      restitution: 0.05
      stiffness: 20000.0
      damping: 50.0

"""

import argparse
import json
import logging
import sys
import os
from typing import Dict, Any

# XML helpers
import xml.etree.ElementTree as ET
from xml.dom import minidom

try:
    import yaml
except Exception:
    yaml = None

# Configure logging
logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
logger = logging.getLogger("sysid_exporters")


# ---------------------
# Utilities
# ---------------------
def pretty_xml(element: ET.Element) -> str:
    """Return pretty-printed XML string for ElementTree element."""
    rough_string = ET.tostring(element, "utf-8")
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")


def safe_get(d: Dict, *keys, default=None):
    """Nested safe get helper: safe_get(d, 'a', 'b', default=...)"""
    cur = d
    for k in keys:
        if not isinstance(cur, dict) or k not in cur:
            return default
        cur = cur[k]
    return cur


# ---------------------
# SysIDModel
# ---------------------
class SysIDModel:
    def __init__(self, schema_file: str):
        self.schema_file = schema_file
        self.data = self._load_schema(schema_file)

    def _load_schema(self, path: str) -> Dict[str, Any]:
        if path.endswith(".json"):
            with open(path, "r") as f:
                return json.load(f)
        elif path.endswith((".yaml", ".yml")):
            if yaml is None:
                raise RuntimeError("pyyaml required to read YAML files. pip install pyyaml")
            with open(path, "r") as f:
                return yaml.safe_load(f)
        else:
            raise ValueError("Unsupported schema extension (use .json or .yaml)")

    def export(self, fmt: str, out_path: str):
        exporters = {
            "urdf": URDFExporter,
            "sdf": SDFExporter,
            "mjcf": MJCFExporter,
            "usd": USDExporter,
        }
        fmt = fmt.lower()
        if fmt not in exporters:
            raise ValueError(f"Unsupported export format '{fmt}'")
        exporter_cls = exporters[fmt]
        exporter = exporter_cls(self.data)
        exporter.write(out_path)


# ---------------------
# BaseExporter
# ---------------------
class BaseExporter:
    def __init__(self, data: Dict[str, Any]):
        self.data = data

    def write(self, out_path: str):
        """
        Default writer: call generate() and dump string to file.
        USDExporter overrides write() because it needs the pxr API.
        """
        text = self.generate()
        with open(out_path, "w", encoding="utf-8") as f:
            f.write(text)
        logger.info("Wrote %s", out_path)

    def generate(self) -> str:
        """Return file contents. Must be overridden by subclasses."""
        raise NotImplementedError


# ---------------------
# URDFExporter
# ---------------------
class URDFExporter(BaseExporter):
    def generate(self) -> str:
        robot_name = safe_get(self.data, "metadata", "robot_name", default="robot")
        robot = ET.Element("robot", name=robot_name)

        # Bodies -> <link>
        bodies = self.data.get("bodies", {})
        for body_name, body in bodies.items():
            link = ET.SubElement(robot, "link", name=body_name)
            # Inertial
            inertial = ET.SubElement(link, "inertial")
            mass_val = body.get("mass")
            if mass_val is None:
                logger.warning("URDF: body '%s' missing mass; defaulting to 0.0", body_name)
                mass_val = 0.0
            ET.SubElement(inertial, "mass", value=str(mass_val))

            com = body.get("com", [0.0, 0.0, 0.0])
            ET.SubElement(inertial, "origin", xyz=f"{com[0]} {com[1]} {com[2]}", rpy="0 0 0")

            I = body.get("inertia", {})
            inertia_el = ET.SubElement(inertial, "inertia",
                                       ixx=str(I.get("ixx", 0.0)),
                                       ixy=str(I.get("ixy", 0.0)),
                                       ixz=str(I.get("ixz", 0.0)),
                                       iyy=str(I.get("iyy", 0.0)),
                                       iyz=str(I.get("iyz", 0.0)),
                                       izz=str(I.get("izz", 0.0)))
            # Visual (best-effort)
            visual = body.get("visual")
            if visual:
                v = ET.SubElement(link, "visual")
                geom = ET.SubElement(v, "geometry")
                mesh_file = visual.get("mesh_file")
                if mesh_file:
                    ET.SubElement(geom, "mesh", filename=mesh_file)
                else:
                    # Not handling primitives here
                    pass
            # Collision omitted or simple copy of visual
            collision = body.get("collision")
            if collision:
                # URDF does not support contact params; warn if present
                contact = collision.get("contact")
                if contact:
                    logger.warning("URDF: contact parameters for body '%s' are not supported; skipping", body_name)
                col = ET.SubElement(link, "collision")
                geom = ET.SubElement(col, "geometry")
                mesh_file = collision.get("mesh_file")
                if mesh_file:
                    ET.SubElement(geom, "mesh", filename=mesh_file)

        # Joints
        joints = self.data.get("joints", {})
        for joint_name, joint in joints.items():
            jtype = joint.get("type", "revolute")
            j = ET.SubElement(robot, "joint", name=joint_name, type=jtype)
            parent = joint.get("parent")
            child = joint.get("child")
            if not parent or not child:
                logger.warning("URDF: joint '%s' missing parent/child", joint_name)
            else:
                ET.SubElement(j, "parent", link=parent)
                ET.SubElement(j, "child", link=child)

            origin = joint.get("origin", [0.0, 0.0, 0.0])
            ET.SubElement(j, "origin", xyz=f"{origin[0]} {origin[1]} {origin[2]}", rpy="0 0 0")
            axis = joint.get("axis", [0.0, 0.0, 1.0])
            ET.SubElement(j, "axis", xyz=f"{axis[0]} {axis[1]} {axis[2]}")

            limits = joint.get("limits")
            if limits:
                ET.SubElement(j, "limit",
                              lower=str(limits.get("lower", 0.0)),
                              upper=str(limits.get("upper", 0.0)),
                              effort=str(limits.get("effort", 0.0)),
                              velocity=str(limits.get("velocity", 0.0)))
            else:
                # Provide safe defaults for URDF validity
                ET.SubElement(j, "limit", lower="0.0", upper="0.0", effort="0.0", velocity="0.0")

            dyn = joint.get("dynamics", {})
            ET.SubElement(j, "dynamics",
                          damping=str(dyn.get("damping", 0.0)),
                          friction=str(dyn.get("friction", 0.0)))
            # URDF does not support many advanced fields (backlash etc.)
            if dyn.get("backlash") is not None:
                logger.warning("URDF: joint '%s' contains 'backlash' which URDF does not support; skipping", joint_name)

        # Write pretty XML
        return pretty_xml(robot)


# ---------------------
# SDFExporter
# ---------------------
class SDFExporter(BaseExporter):
    def generate(self) -> str:
        sdf = ET.Element("sdf", version="1.10")
        world = ET.SubElement(sdf, "world", name="default")

        # Add a simple ambient light to avoid emptiness
        ET.SubElement(world, "include")  # keep simple

        # Bodies -> models
        bodies = self.data.get("bodies", {})
        for body_name, body in bodies.items():
            model = ET.SubElement(world, "model", name=body_name)
            static = body.get("static", False)
            if static:
                ET.SubElement(model, "static").text = "true"
            link = ET.SubElement(model, "link", name=f"{body_name}_link")
            # Inertial
            inertial = ET.SubElement(link, "inertial")
            ET.SubElement(inertial, "mass").text = str(body.get("mass", 0.0))
            com = body.get("com", [0.0, 0.0, 0.0])
            ET.SubElement(inertial, "pose").text = f"{com[0]} {com[1]} {com[2]} 0 0 0"
            I = body.get("inertia", {})
            inertia = ET.SubElement(inertial, "inertia")
            ET.SubElement(inertia, "ixx").text = str(I.get("ixx", 0.0))
            ET.SubElement(inertia, "ixy").text = str(I.get("ixy", 0.0))
            ET.SubElement(inertia, "ixz").text = str(I.get("ixz", 0.0))
            ET.SubElement(inertia, "iyy").text = str(I.get("iyy", 0.0))
            ET.SubElement(inertia, "iyz").text = str(I.get("iyz", 0.0))
            ET.SubElement(inertia, "izz").text = str(I.get("izz", 0.0))

            # Visual
            visual = body.get("visual")
            if visual:
                v = ET.SubElement(link, "visual", name=f"{body_name}_visual")
                geometry = ET.SubElement(v, "geometry")
                mesh = ET.SubElement(geometry, "mesh")
                mesh_file = visual.get("mesh_file")
                if mesh_file:
                    ET.SubElement(mesh, "uri").text = mesh_file
                material = ET.SubElement(v, "material")
                color = visual.get("color")
                if color:
                    ambient = ET.SubElement(material, "ambient")
                    ambient.text = " ".join(map(str, color[:3])) + " 1.0"

            # Collision + contact params
            collision = body.get("collision")
            if collision:
                col = ET.SubElement(link, "collision", name=f"{body_name}_collision")
                geometry = ET.SubElement(col, "geometry")
                mesh = ET.SubElement(geometry, "mesh")
                mesh_file = collision.get("mesh_file")
                if mesh_file:
                    ET.SubElement(mesh, "uri").text = mesh_file
                surface = ET.SubElement(col, "surface")
                contact = collision.get("contact", {})
                if contact:
                    friction = ET.SubElement(surface, "friction")
                    ode = ET.SubElement(friction, "ode")
                    mu = contact.get("mu_static")
                    mu2 = contact.get("mu_dynamic")
                    if mu is not None:
                        ET.SubElement(ode, "mu").text = str(mu)
                    if mu2 is not None:
                        ET.SubElement(ode, "mu2").text = str(mu2)
                    bounce = ET.SubElement(surface, "bounce")
                    if contact.get("restitution") is not None:
                        ET.SubElement(bounce, "restitution_coefficient").text = str(contact.get("restitution"))
                    contact_el = ET.SubElement(surface, "contact")
                    if contact.get("stiffness") is not None:
                        ET.SubElement(contact_el, "ode", ).text = ""  # placeholder
                        # ODE contact details can be added as needed
                # else: no contact info

        # Joints
        joints = self.data.get("joints", {})
        for jname, j in joints.items():
            # SDF expects joints within models - this simplistic approach creates a model per joint's parent
            parent = j.get("parent")
            child = j.get("child")
            if not parent:
                logger.warning("SDF: joint '%s' missing parent - skipping", jname)
                continue
            # Find model element for parent
            model = next((m for m in world.findall("model") if m.get("name") == parent), None)
            if model is None:
                logger.warning("SDF: parent model '%s' for joint '%s' not present; creating model", parent, jname)
                model = ET.SubElement(world, "model", name=parent)
                ET.SubElement(model, "link", name=f"{parent}_link")
            # Add joint to world level (SDF allows joint within model; for simplicity we attach to model)
            joint_el = ET.SubElement(model, "joint", name=jname, type=j.get("type", "revolute"))
            ET.SubElement(joint_el, "parent").text = f"{parent}_link"
            ET.SubElement(joint_el, "child").text = f"{child}_link" if child else ""
            origin = j.get("origin", [0.0, 0.0, 0.0])
            ET.SubElement(joint_el, "pose").text = f"{origin[0]} {origin[1]} {origin[2]} 0 0 0"
            axis = j.get("axis", [0.0, 0.0, 1.0])
            axis_el = ET.SubElement(joint_el, "axis")
            ET.SubElement(axis_el, "xyz").text = f"{axis[0]} {axis[1]} {axis[2]}"
            limits = j.get("limits", {})
            limit_el = ET.SubElement(axis_el, "limit")
            ET.SubElement(limit_el, "lower").text = str(limits.get("lower", 0.0))
            ET.SubElement(limit_el, "upper").text = str(limits.get("upper", 0.0))
            if j.get("dynamics"):
                dyn = j["dynamics"]
                ET.SubElement(axis_el, "damping").text = str(dyn.get("damping", 0.0))
                ET.SubElement(axis_el, "friction").text = str(dyn.get("friction", 0.0))

        # Sensors
        sensors = self.data.get("sensors", {})
        for sname, s in sensors.items():
            stype = s.get("type")
            parent = s.get("parent")
            if parent is None:
                logger.warning("SDF: sensor '%s' missing parent - skipping", sname)
                continue
            model = next((m for m in world.findall("model") if m.get("name") == parent), None)
            if model is None:
                logger.warning("SDF: parent model '%s' for sensor '%s' not present; creating model", parent, sname)
                model = ET.SubElement(world, "model", name=parent)
                ET.SubElement(model, "link", name=f"{parent}_link")
            link_el = next((l for l in model.findall("link") if l.get("name") == f"{parent}_link"), None)
            if link_el is None:
                link_el = ET.SubElement(model, "link", name=f"{parent}_link")
            sensor_el = ET.SubElement(link_el, "sensor", name=sname, type=stype)
            pose = s.get("pose", [0, 0, 0, 0, 0, 0])
            ET.SubElement(sensor_el, "pose").text = " ".join(map(str, pose))
            # Noise block if present
            noise = s.get("noise", {})
            if noise:
                noise_el = ET.SubElement(sensor_el, "noise")
                for k, v in noise.items():
                    ET.SubElement(noise_el, k).text = str(v)

        return pretty_xml(sdf)


# ---------------------
# MJCFExporter
# ---------------------
class MJCFExporter(BaseExporter):
    def generate(self) -> str:
        mj = ET.Element("mujoco", model=self.data.get("metadata", {}).get("robot_name", "robot"))
        worldbody = ET.SubElement(mj, "worldbody")
        # Bodies -> <body>
        for bname, body in self.data.get("bodies", {}).items():
            pos = body.get("com", [0.0, 0.0, 0.0])
            body_el = ET.SubElement(worldbody, "body", name=bname, pos=f"{pos[0]} {pos[1]} {pos[2]}")
            # Inertial attributes in MuJoCo are provided as inertia and mass on <body> or <inertial>
            mass = body.get("mass", 0.0)
            ET.SubElement(body_el, "inertial", pos="0 0 0",
                          mass=str(mass),
                          ixx=str(body.get("inertia", {}).get("ixx", 0.0)),
                          iyy=str(body.get("inertia", {}).get("iyy", 0.0)),
                          izz=str(body.get("inertia", {}).get("izz", 0.0)),
                          ixy=str(body.get("inertia", {}).get("ixy", 0.0)),
                          ixz=str(body.get("inertia", {}).get("ixz", 0.0)),
                          iyz=str(body.get("inertia", {}).get("iyz", 0.0)))
            # Geometry (collision/visual)
            visual = body.get("visual")
            if visual and visual.get("mesh_file"):
                ET.SubElement(body_el, "geom", type="mesh", mesh=visual["mesh_file"])
            collision = body.get("collision")
            if collision and collision.get("mesh_file"):
                ET.SubElement(body_el, "geom", type="mesh", mesh=collision["mesh_file"])
            # MuJoCo uses actuators section for motors; joints are separate

        # Joints (in MuJoCo, joints are part of worldbody as <joint>)
        for jname, j in self.data.get("joints", {}).items():
            parent = j.get("parent")
            child = j.get("child")
            # Place joint under parent body (MuJoCo expects hierarchical bodies); simple approach:
            parent_body = next((b for b in worldbody.findall("body") if b.get("name") == parent), None)
            if parent_body is None:
                logger.warning("MJCF: parent body '%s' not found for joint '%s'; creating parent body", parent, jname)
                parent_body = ET.SubElement(worldbody, "body", name=parent)
            joint_type = "hinge" if j.get("type", "revolute") in ("revolute", "continuous") else "slide"
            ET.SubElement(parent_body, "joint", name=jname, type=joint_type, axis=" ".join(map(str, j.get("axis", [0, 0, 1]))))
            # Add joint damping/friction as attributes on joint (MuJoCo has damping and frictionloss)
            # In more advanced usage, you'd map friction to actuator frictionloss values.

        # Actuators
        actuators = self.data.get("actuators", {})
        if actuators:
            actuators_el = ET.SubElement(mj, "actuator")
            for a_name, a in actuators.items():
                # We map DC motors to <motor> with gear attribute
                joint_name = a.get("joint")
                if joint_name is None:
                    logger.warning("MJCF: actuator '%s' missing joint field; skipping", a_name)
                    continue
                gear = a.get("gear_ratio", a.get("gear", 1.0))
                # ctrlrange maps saturation
                ctrlrange = a.get("saturation")
                motor_attrs = {"name": a_name, "joint": joint_name, "gear": str(gear)}
                if ctrlrange is not None:
                    motor_attrs["ctrlrange"] = f"{-abs(ctrlrange)} {abs(ctrlrange)}"
                ET.SubElement(actuators_el, "motor", **motor_attrs)

        # Default contact parameters mapping
        # MuJoCo default geom friction is set on <default><geom>
        contacts = self.data.get("contacts", {})
        if contacts:
            default_el = ET.SubElement(mj, "default")
            geom_el = ET.SubElement(default_el, "geom")
            # Use the first contact entry as default hint
            first_contact = next(iter(contacts.values()))
            mu = first_contact.get("surface", {}).get("mu_static")
            if mu is not None:
                geom_el.set("friction", f"{mu} {mu} 0.0")

        return pretty_xml(mj)


# ---------------------
# USDExporter
# ---------------------
class USDExporter(BaseExporter):
    def write(self, out_path: str):
        """
        USD export requires Pixar USD Python bindings (pxr).
        If not present, raise a helpful error.
        """
        try:
            from pxr import Usd, UsdGeom, PhysxSchema, Gf
        except Exception as e:
            raise RuntimeError(
                "USD export requested but pxr (Pixar USD Python bindings) not available. "
                "Install USD/Omniverse python bindings or skip USD export."
            ) from e

        stage = Usd.Stage.CreateNew(out_path)
        robot_name = safe_get(self.data, "metadata", "robot_name", default="robot")
        root_xform = UsdGeom.Xform.Define(stage, f"/{robot_name}")

        # Bodies
        for bname, b in self.data.get("bodies", {}).items():
            prim_path = f"/{robot_name}/{bname}"
            xform = UsdGeom.Xform.Define(stage, prim_path)
            # PhysX rigid body components
            try:
                rb_api = PhysxSchema.PhysxRigidBodyAPI.Apply(xform.GetPrim())
                mass = b.get("mass", 0.0)
                rb_api.GetMassAttr().Set(mass)
                # center of mass
                com = b.get("com", [0.0, 0.0, 0.0])
                rb_api.GetCenterOfMassAttr().Set(Gf.Vec3d(*com))
                # Inertia mapping may require specific API calls or custom attributes
                # Set a custom key for inertia for exporters to pick up.
                inertia = b.get("inertia", {})
                rb_api.GetPrim().GetAttribute("inertia:ixx").Set(inertia.get("ixx", 0.0))
            except Exception:
                logger.warning("USD: failed to apply PhysxRigidBodyAPI for '%s' (maybe PhysX not available)", bname)

            # Material/friction
            collision = b.get("collision", {})
            contact = collision.get("contact", {}) if collision else {}
            if contact:
                # Create a material prim with PhysX material attributes
                mat_path = f"/{robot_name}/{bname}/material"
                mat_prim = stage.DefinePrim(mat_path, "PhysxMaterial") if stage else None
                # The actual usd PhysX material schema may vary across installations
                # Set attributes if available
                # We do best-effort set via attributes
                if mat_prim:
                    if contact.get("mu_static") is not None:
                        try:
                            mat_prim.GetAttribute("staticFriction").Set(contact.get("mu_static"))
                        except Exception:
                            pass
                    if contact.get("restitution") is not None:
                        try:
                            mat_prim.GetAttribute("restitution").Set(contact.get("restitution"))
                        except Exception:
                            pass

        # Joints: simplified - create joint prims under root
        for jname, j in self.data.get("joints", {}).items():
            prim_path = f"/{robot_name}/joints/{jname}"
            try:
                joint_prim = stage.DefinePrim(prim_path, "PhysicsJoint")
                # set basic attrs
                joint_prim.GetAttribute("type").Set(j.get("type", "revolute"))
                # axis, limits, damping can be stored as custom attributes
            except Exception:
                logger.warning("USD: failed to create joint '%s'", jname)

        stage.GetRootLayer().Save()
        logger.info("USD exported to %s", out_path)


# ---------------------
# CLI & Example Schema Writer
# ---------------------
EXAMPLE_SCHEMA = {
    "metadata": {
        "robot_name": "example_bot",
        "schema_version": "0.1",
        "source": "example",
        "units": "SI",
    },
    "bodies": {
        "base_link": {
            "mass": 5.0,
            "com": [0.0, 0.0, 0.1],
            "inertia": {"ixx": 0.1, "iyy": 0.1, "izz": 0.05, "ixy": 0.0, "ixz": 0.0, "iyz": 0.0},
            "visual": {"mesh_file": "meshes/base.stl", "color": [0.8, 0.8, 0.8, 1.0]},
            "collision": {"mesh_file": "meshes/base_col.stl", "contact": {"mu_static": 0.8, "mu_dynamic": 0.7, "restitution": 0.1}},
        },
        "link1": {
            "mass": 1.0,
            "com": [0.0, 0.0, 0.1],
            "inertia": {"ixx": 0.01, "iyy": 0.01, "izz": 0.005},
        },
    },
    "joints": {
        "joint1": {
            "type": "revolute",
            "parent": "base_link",
            "child": "link1",
            "axis": [0, 0, 1],
            "origin": [0.0, 0.0, 0.2],
            "limits": {"lower": -1.57, "upper": 1.57, "effort": 10.0, "velocity": 2.0},
            "dynamics": {"damping": 0.01, "friction": 0.05},
        }
    },
    "actuators": {
        "motor_joint1": {
            "joint": "joint1",
            "model": "dc_motor",
            "torque_constant": 0.05,
            "gear_ratio": 100,
            "saturation": 5.0,
        }
    },
    "sensors": {
        "imu_1": {
            "type": "imu",
            "parent": "base_link",
            "pose": [0, 0, 0, 0, 0, 0],
            "noise": {"accel_stddev": [0.02, 0.02, 0.02], "gyro_stddev": [0.001, 0.001, 0.001]},
        }
    },
    "contacts": {
        "foot": {
            "body": "base_link",
            "surface": {"mu_static": 0.85, "mu_dynamic": 0.75, "restitution": 0.05, "stiffness": 20000.0, "damping": 50.0},
        }
    },
}


def write_example_schema(path: str, as_yaml=True):
    if as_yaml:
        if yaml is None:
            logger.error("pyyaml not available; cannot write YAML example. Install pyyaml.")
            return
        with open(path, "w") as f:
            yaml.safe_dump(EXAMPLE_SCHEMA, f)
    else:
        with open(path, "w") as f:
            json.dump(EXAMPLE_SCHEMA, f, indent=2)
    logger.info("Wrote example schema to %s", path)


def main():
    p = argparse.ArgumentParser(description="SysID Schema Exporter")
    p.add_argument("--schema", "-s", type=str, help="Path to schema YAML/JSON file")
    p.add_argument("--formats", "-f", type=str, default="urdf", help="Comma-separated formats: urdf,sdf,mjcf,usd")
    p.add_argument("--outdir", "-o", type=str, default="out", help="Output directory")
    p.add_argument("--write-example", action="store_true", help="Write an example schema to 'example_schema.yaml' and exit")
    args = p.parse_args()

    if args.write_example:
        example_path = "example_schema.yaml"
        write_example_schema(example_path, as_yaml=True)
        print(f"Wrote example schema to {example_path}")
        return

    if not args.schema:
        p.print_help()
        return

    if not os.path.exists(args.outdir):
        os.makedirs(args.outdir, exist_ok=True)

    model = SysIDModel(args.schema)
    fmts = [x.strip().lower() for x in args.formats.split(",")]
    for fmt in fmts:
        out_path = os.path.join(args.outdir, f"{model.data.get('metadata', {}).get('robot_name', 'robot')}.{fmt}")
        try:
            model.export(fmt, out_path)
        except Exception as e:
            logger.error("Failed to export %s: %s", fmt, str(e))


if __name__ == "__main__":
    main()
